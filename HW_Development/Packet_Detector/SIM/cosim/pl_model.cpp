// pl_model.cpp - cycle model of the Zedboard PL for the firmware co-simulation:
// Verilated pd_axi (wrapper + detector) plus an AXI DMA model (MM2S / S2MM, simple mode).
// Firmware register accesses (Xil_In32 / Xil_Out32 at PD_BASE) become AXI-Lite transactions;
// XAxiDma_Busy advances the clock. Env: MM2S_P / S2MM_P = probability (0..1) that MM2S has
// data / S2MM is ready in a cycle (default 0.8 / 0.7); SEED.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include "Vpd_axi.h"
#include "verilated.h"

extern "C" {
#include "xil_types.h"
#include "xaxidma.h"
}

static Vpd_axi *top;
static uint64_t cyc;
static double p_mm2s = 0.8, p_s2mm = 0.7;
static int busy_ticks = 64;
static unsigned rng = 1;
static double rnd() { rng = rng * 1103515245u + 12345u; return ((rng >> 8) & 0xFFFFFF) / 16777216.0; }

struct Mm2s { bool act; uint32_t *p; uint32_t n, i; } mm;
struct S2mm { bool act; uint32_t *p; uint32_t n, got; } s2;
static bool aw_hs, w_hs, b_hs, ar_hs, r_hs;
static uint32_t r_data;
static uint64_t mm_stall, s2_stall;

static void init() {
    if (top) return;
    top = new Vpd_axi;
    if (getenv("MM2S_P")) p_mm2s = atof(getenv("MM2S_P"));
    if (getenv("S2MM_P")) p_s2mm = atof(getenv("S2MM_P"));
    if (getenv("SEED")) rng = atoi(getenv("SEED"));
    if (getenv("BUSY_TICKS")) busy_ticks = atoi(getenv("BUSY_TICKS"));
    top->aresetn = 0;
    for (int i = 0; i < 10; i++) { top->aclk = 0; top->eval(); top->aclk = 1; top->eval(); }
    top->aresetn = 1;
}

// one clock cycle: drive stream inputs, settle, sample handshakes, rising edge, update models
static void tick() {
    bool mv = mm.act && rnd() < p_mm2s;
    top->s_axis_tvalid = mv;
    top->s_axis_tdata  = mv ? mm.p[mm.i] : 0;
    top->s_axis_tlast  = mv && (mm.i == mm.n - 1);
    top->s_axis_tkeep  = 0xF;
    top->m_axis_tready = s2.act && rnd() < p_s2mm;
    top->aclk = 0; top->eval();
    bool mm_fire = mv && top->s_axis_tready;
    if (mv && !top->s_axis_tready) mm_stall++;
    bool s2_fire = top->m_axis_tvalid && top->m_axis_tready;
    uint32_t s2_data = top->m_axis_tdata; bool s2_last = top->m_axis_tlast;
    aw_hs = top->s_axi_awvalid && top->s_axi_awready;
    w_hs  = top->s_axi_wvalid && top->s_axi_wready;
    b_hs  = top->s_axi_bvalid && top->s_axi_bready;
    ar_hs = top->s_axi_arvalid && top->s_axi_arready;
    r_hs  = top->s_axi_rvalid && top->s_axi_rready;
    r_data = top->s_axi_rdata;
    top->aclk = 1; top->eval();
    cyc++;
    if (cyc % 1000000 == 0 && getenv("VERBOSE")) fprintf(stderr, "[pl_model] %llu clocks, MM2S held %llu\n",
                                                        (unsigned long long)cyc, (unsigned long long)mm_stall);
    if (mm_fire) { mm.i++; if (mm.i == mm.n) mm.act = false; }
    if (s2_fire) {
        if (s2.got < s2.n) s2.p[s2.got] = s2_data;
        else { fprintf(stderr, "S2MM overrun\n"); exit(3); }
        s2.got++;
        if (s2_last || s2.got == s2.n) {
            if (!s2_last) fprintf(stderr, "S2MM: buffer full before TLAST\n");
            s2.act = false;
        }
    }
}

static u32 dma_regs[64];                      // DMA register stub (co-sim): plain storage
extern "C" void Xil_Out32(UINTPTR a, u32 v) {
    init();
    if ((a & 0xFFFF0000u) == 0x40400000u) { dma_regs[(a & 0xFF) / 4] = v; return; }
    top->s_axi_awaddr = a & 0xFFF; top->s_axi_wdata = v; top->s_axi_wstrb = 0xF;
    top->s_axi_awvalid = 1; top->s_axi_wvalid = 1; top->s_axi_bready = 1;
    bool aw = false, w = false, b = false;
    for (int k = 0; k < 1000 && !b; k++) {
        tick();
        if (aw_hs) { aw = true; top->s_axi_awvalid = 0; }
        if (w_hs)  { w = true;  top->s_axi_wvalid = 0; }
        if (b_hs)  b = true;
    }
    top->s_axi_bready = 0;
    if (!(aw && w && b)) { fprintf(stderr, "AXI write 0x%lx hung\n", (unsigned long)a); exit(3); }
}

extern "C" u32 Xil_In32(UINTPTR a) {
    init();
    if ((a & 0xFFFF0000u) == 0x40400000u) return dma_regs[(a & 0xFF) / 4];
    top->s_axi_araddr = a & 0xFFF; top->s_axi_arvalid = 1; top->s_axi_rready = 1;
    bool ar = false;
    for (int k = 0; k < 1000; k++) {
        tick();
        if (ar_hs) { ar = true; top->s_axi_arvalid = 0; }
        if (r_hs) {
            top->s_axi_rready = 0;
            static uint32_t lvl_max;
            if ((a & 0xFFF) == 0x14 && r_data > lvl_max) {
                lvl_max = r_data;
                fprintf(stderr, "[pl_model] EV_LEVEL max %u words (FIFO 256, input held above 160)\n", lvl_max);
            }
            return r_data;
        }
    }
    fprintf(stderr, "AXI read 0x%lx hung (ar %d)\n", (unsigned long)a, ar); exit(3);
}

// ---- AXI DMA (simple mode) ---------------------------------------------------------- //
static XAxiDma_Config cfg;
extern "C" {
XAxiDma_Config *XAxiDma_LookupConfig(UINTPTR) { return &cfg; }
int XAxiDma_CfgInitialize(XAxiDma *d, XAxiDma_Config *) { init(); d->RegBase = 0x40400000; return 0; }
int XAxiDma_HasSg(XAxiDma *) { return 0; }
void XAxiDma_Reset(XAxiDma *) { mm.act = false; s2.act = false; }
int XAxiDma_ResetIsDone(XAxiDma *) { return 1; }
void XAxiDma_IntrDisable(XAxiDma *, u32, int) {}
int XAxiDma_SimpleTransfer(XAxiDma *, UINTPTR a, u32 n, int dir) {
    if (dir == XAXIDMA_DEVICE_TO_DMA) { s2 = {true, (uint32_t *)a, n / 4, 0}; }
    else                              { mm = {true, (uint32_t *)a, n / 4, 0}; if (n == 0) mm.act = false; }
    return 0;
}
int XAxiDma_Busy(XAxiDma *, int dir) {
    for (int k = 0; k < busy_ticks; k++) tick();
    return dir == XAXIDMA_DEVICE_TO_DMA ? s2.act : mm.act;
}
u32 XAxiDma_ReadReg(UINTPTR b, u32 o) {
    if (o == XAXIDMA_BUFFLEN_OFFSET && (b & 0x30)) return s2.got * 4;
    return 0;
}
void pl_model_report(void) {
    fprintf(stderr, "[pl_model] %llu clocks, MM2S held %llu clocks by tready\n",
            (unsigned long long)cyc, (unsigned long long)mm_stall);
}
}
