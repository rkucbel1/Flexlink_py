/*
 * pd_server.c  (Flexlink_py/HW_Development/Packet_Detector/FW)
 *
 * Zedboard side of the packet-detector test bench: a TCP server on port 5001 that speaks
 * the protocol in PC/pd_protocol.py. It replaces echo.c in the Vitis "lwIP Echo Server"
 * template, whose main.c (network setup, DHCP with fallback to 192.168.50.10 (patched by build_firmware.py), timers, main
 * loop) is used unchanged and calls start_application(), print_app_header() and
 * transfer_data() defined here.
 *
 * Memory (DDR, fixed addresses above the program image):
 *     0x10000000  sample buffer, 128 MB = 32M samples (one u32 per sample: I low, Q high)
 *     0x18000000  debug buffer,  128 MB = 32M x 4-byte records (S2MM destination)
 * The program itself (code, heap, stack, rx/tx buffers) stays in the first 256 MB.
 *
 * Phase 2 (this version): the PL holds the detector (RTL/pd_axi.v, registers at PD_BASE).
 *   RUN            samples DDR -> AXI DMA MM2S -> detector; events are read from the detector's
 *                  event FIFO while the DMA runs; with the debug flag the debug records go
 *                  detector -> AXI DMA S2MM -> debug buffer. After the DMA the firmware waits
 *                  until the detector is idle, then reports a window that is still open (its
 *                  samples have not all arrived) as a truncated event, once, in the RUN in which
 *                  it was triggered. The detector state carries over to the next RUN.
 *   RUN + loopback samples DDR -> MM2S -> PL (detector bypassed) -> S2MM -> debug buffer.
 * Input stage 2 (40 Msps samples, half-band decimator in the PL): RUN counts must be even
 * (whole input pairs); events, debug records and the truncation check are in 20 Msps samples.
 * The firmware converts the detector's integers to the event record: cfo_valid_cnt = t + N_CFO,
 * ToneCheck dB = 10 log10(num / den) (Q8.8), TONE_THRESH (Q8.8 dB) -> linear threshold.
 *
 * Note: xil_printf (used for the UART log) knows only %d %u %x %s %c; no %l formats.
 *       Error texts for the PC are formatted with newlib's snprintf, which has them.
 *
 * Protocol summary (little-endian; header 16 bytes):
 *     u32 magic 0x44504C46, u16 cmd, u16 status, u32 seq, u32 length, then payload
 *     PING 1, RESET 2, SET_PARAMS 3, LOAD_SAMPLES 4, READ_SAMPLES 5, RUN 6, GET_DEBUG 7,
 *     MEMTEST 8 (diagnostic: board-local DDR test, no network involved)
 *     REG_READ 9 / REG_WRITE 10 (diagnostic: a 32-bit register in the PL range
 *     0x40000000..0x7FFFFFFF, e.g. the AXI DMA or the detector)
 */
#include <stdio.h>
#include <string.h>

#include "xparameters.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "xil_types.h"
#include "xstatus.h"
#include "xaxidma.h"
#include "xil_io.h"
#include "sleep.h"

#include "lwip/err.h"
#include "lwip/tcp.h"

/* ---------------------------------------------------------------------------------- */
/* Configuration                                                                       */
/* ---------------------------------------------------------------------------------- */
#define PD_PORT                 5001
#define PD_PROTOCOL_VERSION     1u
#define PD_BUILD                14u              /* bump when the firmware changes */

#define SAMPLE_BUF_ADDR         0x10000000u
#define DEBUG_BUF_ADDR          0x18000000u
#define BUF_BYTES               0x08000000u     /* 128 MB each */
#define MAX_SAMPLES             (BUF_BYTES / 4u)
#define DMA_CHUNK_BYTES         (16u << 20)     /* per DMA transfer (length field is 26 bits) */
#define DMA_TIMEOUT_POLLS       200000000u

/* DMA instance: SDT flow (Vitis 2023.2 and later) looks it up by base address */
#if defined(XPAR_XAXIDMA_0_BASEADDR)
#define PD_DMA_BASE XPAR_XAXIDMA_0_BASEADDR
#elif defined(XPAR_AXI_DMA_0_BASEADDR)
#define PD_DMA_BASE XPAR_AXI_DMA_0_BASEADDR
#endif

/* Detector registers (RTL/pd_axi.v). Address fixed by HW/build_hw.tcl. */
#if defined(XPAR_PD_0_BASEADDR)
#define PD_BASE XPAR_PD_0_BASEADDR
#else
#define PD_BASE 0x43C00000u
#endif
#define PD_ID_MASK              0xFFFF0000u     /* "PD"; low half = register map version */
#define PD_ID_VALUE             0x50440000u
#define PD_ID_V2                0x0200u         /* 2.0: input stage 2 (40 Msps, half-band) */
#define PD_ID_V3                0x0300u         /* 3.0: Preamble B timing (T_PK, timing FIFO) */
#define PD_ID_V4                0x0400u         /* 4.0: control-field receiver (control FIFO) */
#define PD_ID_V5                0x0500u         /* 5.0: re-arm by the Rx state machine (REARM) */
#define PD_ID_V6                0x0600u         /* 6.0: shared fast FFT, payload records (PL FIFO) */
#define PD_ID_V7                0x0700u         /* 7.0: payload channel estimate (CH FIFO) */
#define R_ID                    0x00u
#define R_CTRL                  0x04u
#define R_STATUS                0x08u
#define R_SAMPLES               0x0Cu
#define R_OPEN_T                0x10u
#define R_EV_LEVEL              0x14u
#define R_EV_DATA               0x18u
#define R_DBG_SENT              0x1Cu
#define R_PARAM0                0x20u           /* ETA_HI_SQ ... TONE_LIN, 4 bytes apart */
#define R_T_PK                  0x44u
#define R_TM_LEVEL              0x48u
#define R_TM_DATA               0x4Cu
#define R_CF_LEVEL              0x50u
#define R_CF_DATA               0x54u
#define R_REARM                 0x58u
#define R_PL_LEVEL              0x5Cu
#define R_PL_DATA               0x60u
#define R_CH_LEVEL              0x64u
#define R_CH_DATA               0x68u
#define CTRL_RESET              0x1u
#define CTRL_DEBUG              0x2u
#define CTRL_LOOPBACK           0x4u
#define STATUS_BUSY             0x1u
#define STATUS_OPEN             0x2u
#define STATUS_EV_OVF           0x100u
#define STATUS_TM_OVF           0x800u
#define STATUS_TM_ERR           0x1000u
#define STATUS_CF_OVF           0x2000u
#define STATUS_CF_ERR           0x4000u
#define STATUS_ARM_LATE         0x8000u
#define STATUS_CH_ERR           0x10000u
#define IDLE_TIMEOUT_POLLS      10000000u

/* ---------------------------------------------------------------------------------- */
/* Protocol constants (keep in step with PC/pd_protocol.py)                            */
/* ---------------------------------------------------------------------------------- */
#define PD_MAGIC                0x44504C46u
#define HDR_SIZE                16u
#define MAX_PAYLOAD             (1u << 20)
#define PARAMS_SIZE_V1          36u             /* SET_PARAMS without t_pk (PC software before build 10) */
#define PARAMS_SIZE_V2          40u             /* SET_PARAMS without rearm (PC software of builds 10, 11) */
#define PARAMS_SIZE             44u
#define T_PK_DEFAULT            4608u           /* 18.0 in Q8.8 */

enum { CMD_PING = 1, CMD_RESET = 2, CMD_SET_PARAMS = 3, CMD_LOAD_SAMPLES = 4,
       CMD_READ_SAMPLES = 5, CMD_RUN = 6, CMD_GET_DEBUG = 7, CMD_MEMTEST = 8,
       CMD_REG_READ = 9, CMD_REG_WRITE = 10, CMD_FPU_TEST = 11,
       CMD_PATTERN_CHECK = 12, CMD_SEG_LOG = 13 };
enum { ST_OK = 0, ST_BAD_CMD = 1, ST_BAD_ARG = 2, ST_NO_SPACE = 3, ST_NO_DEBUG = 4,
       ST_INTERNAL = 5 };

#define RUN_FLAG_DEBUG          0x1u
#define RUN_FLAG_PL_LOOPBACK    0x2u
#define BOARD_FLAG_EMULATOR     0x1u
#define BOARD_FLAG_NO_DETECTOR  0x2u
#define BOARD_FLAG_TIMING       0x4u            /* bitstream has the Preamble B timing (ID 3.0) */
#define BOARD_FLAG_CF           0x8u            /* bitstream has the control-field receiver (ID 4.0) */
#define BOARD_FLAG_REARM        0x10u           /* bitstream has the state machine's re-arm (ID 5.0) */
#define BOARD_FLAG_PAYLOAD      0x20u           /* bitstream has the speculative payload FFT (ID 6.0) */
#define BOARD_FLAG_CHEST        0x40u           /* bitstream has the payload channel estimate (ID 7.0) */
#define EV_CFO_VALID            0x1u
#define EV_TONE_FAIL            0x2u
#define EV_TRUNCATED            0x4u
#define EV_SIZE                 24u
#define TM_SIZE                 32u             /* timing record (pd_protocol.TimingRecord) */
#define MAX_TM                  4096u           /* timing records per RUN */
#define CF_SIZE                 24u             /* control record (pd_protocol.ControlRecord) */
#define MAX_CF                  1024u           /* control records per RUN */
#define PL_SIZE                 16u             /* payload record (pd_protocol.PayloadRecord) */
#define MAX_PL                  8192u           /* payload records per RUN */
#define MAX_CH                  8192u           /* channel-estimate records per RUN (PL_SIZE each) */
#define MAX_EVENTS              ((MAX_PAYLOAD - 24u - MAX_TM * TM_SIZE - MAX_CF * CF_SIZE - MAX_PL * PL_SIZE \
                                  - MAX_CH * PL_SIZE - EV_SIZE) / EV_SIZE)
#define TONE_THRESH_MAX_Q8      (24u * 256u)

/* ---------------------------------------------------------------------------------- */
/* State                                                                               */
/* ---------------------------------------------------------------------------------- */
static XAxiDma dma;
static int dma_ok;
static int det_ok;                  /* detector found (ID register) */
static int det_cfg;                 /* build 7: detector configured (first SET_PARAMS or RUN);
                                     * until then it stays in its power-on reset, as with build 3 */
static u32 det_version;             /* register map version, 0x0100 or 0x0200 */

static struct tcp_pcb *client;

static u8  rx_buf[HDR_SIZE + MAX_PAYLOAD] __attribute__((aligned(64)));
static u32 rx_have;                 /* bytes of the current message received */
static u32 rx_len;                  /* payload length from its header */

/* Diagnostic (build 8): where each received pbuf's payload sat in memory, so the PC can turn a
 * bad sample into the memory address it passed through. Cleared by RESET. */
#define SEG_LOG_MAX             32768u
#define LOAD_LOG_MAX            1024u
static u32 rx_stream;               /* bytes received on this connection */
static u32 msg_start;               /* stream position of the current message's header */
static u32 seg_log[SEG_LOG_MAX][3]; /* stream position, payload address, length */
static u32 seg_n;
static u32 load_log[LOAD_LOG_MAX][3]; /* sample offset, count, stream position of sample 0 */
static u32 load_n;

static u8        tx_hdr[HDR_SIZE];
static u8        tx_small[256];     /* short replies and error messages */
static const u8 *tx_data;
static u32       tx_total;          /* header + payload */
static u32       tx_off;

static u8  params[PARAMS_SIZE];
static u32 samples_stored;          /* samples in the sample buffer */
static u32 debug_count;             /* records in the debug buffer from the last RUN */
static u8  ev_out[MAX_PAYLOAD];      /* RUN reply: n_events, count, events, timing, control */
static u32 ev_n;                    /* events in ev_out */
static int ev_lost;                 /* more than MAX_EVENTS in one RUN */
static u8  tm_out[MAX_TM * TM_SIZE]; /* timing records of the RUN */
static u32 tm_n;
static int tm_lost;
static u8  cf_out[MAX_CF * CF_SIZE]; /* control records of the RUN */
static u32 cf_n;
static int cf_lost;
static u8  pl_out[MAX_PL * PL_SIZE]; /* payload records of the RUN */
static u32 pl_n;
static int pl_lost;
static u8  ch_out[MAX_CH * PL_SIZE]; /* channel-estimate records of the RUN */
static u32 ch_n;
static int ch_lost;

/* ---------------------------------------------------------------------------------- */
/* Helpers                                                                             */
/* ---------------------------------------------------------------------------------- */
static u32 rd32(const u8 *p) { u32 v; memcpy(&v, p, 4); return v; }       /* ARM is LE */
static void wr32(u8 *p, u32 v) { memcpy(p, &v, 4); }
static void wr16(u8 *p, u16 v) { memcpy(p, &v, 2); }

static u32  pd_rd(u32 off)        { return Xil_In32(PD_BASE + off); }
static void pd_wr(u32 off, u32 v) { Xil_Out32(PD_BASE + off, v); }

/* ---------------------------------------------------------------------------------- */
/* Number conversions (no libm: plain double arithmetic, checked against pd_reference) */
/* ---------------------------------------------------------------------------------- */
#define LN2  0.6931471805599453094
#define LN10 2.302585092994045684

static double pd_exp(double y)                  /* y >= 0, small */
{
    int k = (int)(y / LN2 + 0.5), i;
    double r = y - k * LN2, term = 1.0, s = 1.0;
    for (i = 1; i < 25; i++) {
        term = term * r / i;
        s = s + term;
    }
    for (i = 0; i < k; i++) {
        s = s * 2.0;
    }
    return s;
}

static double pd_ln(double x)                   /* x > 0 */
{
    u64 bits;
    int e;
    double m, z, z2, t, s;
    int i;
    memcpy(&bits, &x, 8);
    e = (int)((bits >> 52) & 0x7FF) - 1023;
    bits = (bits & 0x000FFFFFFFFFFFFFull) | 0x3FF0000000000000ull;   /* mantissa in [1, 2) */
    memcpy(&m, &bits, 8);
    if (m > 1.4142135623730951) {
        m = m / 2.0;
        e = e + 1;
    }
    z = (m - 1.0) / (m + 1.0);                  /* ln m = 2 atanh z, |z| < 0.172 */
    z2 = z * z;
    t = z;
    s = z;
    for (i = 3; i < 40; i += 2) {
        t = t * z2;
        s = s + t / i;
    }
    return e * LN2 + 2.0 * s;
}

/* TONE_THRESH (Q8.8 dB) -> linear Q.16, as pd_reference.tone_thresh_lin; 0 = check off */
static u32 tone_thresh_lin(u32 q8)
{
    if (q8 == 0) {
        return 0;
    }
    return (u32)(pd_exp((q8 / 2560.0) * LN10) * 65536.0 + 0.5);
}

/* ToneCheck in Q8.8 dB from the detector's num / den, as pd_reference.tone_db + rounding */
static s16 tone_db_q8(u64 num, u64 den)
{
    double db, v;
    long long f;
    if (den == 0) {
        db = num > 0 ? 127.0 : -128.0;
    } else if (num == 0) {
        db = -128.0;
    } else {
        db = 10.0 * (pd_ln((double)num / (double)den) / LN10);
    }
    v = db * 256.0 + 0.5;
    f = (long long)v;
    if ((double)f > v) {
        f = f - 1;                              /* floor */
    }
    if (f > 32767) {
        f = 32767;
    }
    if (f < -32768) {
        f = -32768;
    }
    return (s16)f;
}

static void reset_connection_state(void)
{
    rx_stream = 0;
    seg_n = 0;
    load_n = 0;
    rx_have = 0;
    rx_len = 0;
    tx_total = 0;
    tx_off = 0;
}

/* ---------------------------------------------------------------------------------- */
/* DMA                                                                                 */
/* ---------------------------------------------------------------------------------- */
static void dma_reset(void)
{
    XAxiDma_Reset(&dma);
    while (!XAxiDma_ResetIsDone(&dma)) {
    }
    XAxiDma_IntrDisable(&dma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);
    XAxiDma_IntrDisable(&dma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);
}

static int dma_init(void)
{
    XAxiDma_Config *cfg;
#ifdef SDT
    cfg = XAxiDma_LookupConfig(PD_DMA_BASE);
#else
    cfg = XAxiDma_LookupConfig(XPAR_AXIDMA_0_DEVICE_ID);
#endif
    if (cfg == NULL) {
        xil_printf("DMA: no configuration found\r\n");
        return -1;
    }
    if (XAxiDma_CfgInitialize(&dma, cfg) != XST_SUCCESS) {
        xil_printf("DMA: initialisation failed\r\n");
        return -1;
    }
    if (XAxiDma_HasSg(&dma)) {
        xil_printf("DMA: built with scatter-gather, simple mode expected\r\n");
        return -1;
    }
    dma_reset();
    return 0;
}

/* ---------------------------------------------------------------------------------- */
/* Detector                                                                            */
/* ---------------------------------------------------------------------------------- */
/* Read every complete event (8 words) from the detector's event FIFO into ev_out. */
static void drain_events(void)
{
    while (pd_rd(R_EV_LEVEL) >= 8u) {
        u32 w[8], i;
        u8 *r;
        u64 num, den;
        for (i = 0; i < 8u; i++) {
            w[i] = pd_rd(R_EV_DATA);
        }
        if (ev_n >= MAX_EVENTS) {
            ev_lost = 1;
            continue;
        }
        num = ((u64)(w[5] & 0xFFu) << 32) | w[4];
        den = ((u64)(w[7] & 0xFFu) << 32) | w[6];
        r = ev_out + 8u + EV_SIZE * ev_n;
        wr32(r + 0, w[0]);                                      /* t_trig */
        wr32(r + 4, w[1]);                                      /* cfo_inc */
        wr32(r + 8, w[0] + rd32(params + 12));                  /* cfo_valid_cnt = t + N_CFO */
        wr16(r + 12, (u16)tone_db_q8(num, den));
        wr16(r + 14, (u16)(EV_CFO_VALID | ((w[5] >> 31) ? EV_TONE_FAIL : 0u)));
        wr16(r + 16, (u16)(w[2] & 0xFFFFu));                    /* ang16 */
        wr16(r + 18, (u16)(w[2] >> 16));                        /* ang32 */
        wr16(r + 20, (u16)(w[3] & 0xFFFFu));                    /* ang128 */
        wr16(r + 22, (u16)(w[3] >> 16));                        /* ang256 */
        ev_n++;
    }
}

/* Read every complete timing record (8 words) into tm_out (pd_protocol.TimingRecord). */
static void drain_timing(void)
{
    if (det_version < PD_ID_V3) {
        return;
    }
    while (pd_rd(R_TM_LEVEL) >= 8u) {
        u32 w[8], i;
        u8 *r;
        for (i = 0; i < 8u; i++) {
            w[i] = pd_rd(R_TM_DATA);
        }
        if (tm_n >= MAX_TM) {
            tm_lost = 1;
            continue;
        }
        r = tm_out + TM_SIZE * tm_n;
        wr32(r + 0, w[0]);                                      /* t_trig */
        wr32(r + 4, w[1]);                                      /* n_end */
        wr32(r + 8, w[2]);                                      /* status */
        wr32(r + 12, w[3]);                                     /* n_pos */
        wr32(r + 16, w[4]);                                     /* p_max (u64) */
        wr32(r + 20, w[5]);
        wr32(r + 24, w[6]);                                     /* p_sum (u64) */
        wr32(r + 28, w[7]);
        tm_n++;
    }
}

/* Read every complete control record (8 words) into cf_out (pd_protocol.ControlRecord). */
static void drain_control(void)
{
    if (det_version < PD_ID_V4) {
        return;
    }
    while (pd_rd(R_CF_LEVEL) >= 8u) {
        u32 w[8], i;
        u8 *r;
        for (i = 0; i < 8u; i++) {
            w[i] = pd_rd(R_CF_DATA);
        }
        if (cf_n >= MAX_CF) {
            cf_lost = 1;
            continue;
        }
        r = cf_out + CF_SIZE * cf_n;
        wr32(r + 0, w[0]);                                      /* t_trig */
        wr32(r + 4, w[1]);                                      /* n_end */
        wr32(r + 8, w[2] & 1u);                                 /* status: CRC ok */
        wr16(r + 12, (u16)((w[2] >> 8) & 0x3Fu));               /* shift */
        wr16(r + 14, (u16)(w[2] >> 16));                        /* metric (signed 16) */
        wr32(r + 16, w[3]);                                     /* message[31:0] */
        wr32(r + 20, w[4] & 0x3FFFFFu);                         /* message[53:32] */
        cf_n++;
    }
}

/* Read every complete payload record (4 words) into pl_out (pd_protocol.PayloadRecord). */
static void drain_payload(void)
{
    if (det_version < PD_ID_V6) {
        return;
    }
    while (pd_rd(R_PL_LEVEL) >= 4u) {
        u32 w[4], i;
        u8 *r;
        for (i = 0; i < 4u; i++) {
            w[i] = pd_rd(R_PL_DATA);
        }
        if (pl_n >= MAX_PL) {
            pl_lost = 1;
            continue;
        }
        r = pl_out + PL_SIZE * pl_n;
        wr32(r + 0, w[0]);                                      /* t_trig */
        wr32(r + 4, w[1] & 0x7Fu);                              /* sym (u16) + 2 pad bytes */
        wr32(r + 8, w[2]);                                      /* sig_a */
        wr32(r + 12, w[3]);                                     /* sig_b */
        pl_n++;
    }
}

/* Read every complete channel-estimate record (4 words) into ch_out (pd_protocol.PayloadRecord
 * layout: t_trig, sym (0x8000 = packet record), sig_a, sig_b). */
static void drain_chest(void)
{
    if (det_version < PD_ID_V7) {
        return;
    }
    while (pd_rd(R_CH_LEVEL) >= 4u) {
        u32 w[4], i;
        u8 *r;
        for (i = 0; i < 4u; i++) {
            w[i] = pd_rd(R_CH_DATA);
        }
        if (ch_n >= MAX_CH) {
            ch_lost = 1;
            continue;
        }
        r = ch_out + PL_SIZE * ch_n;
        wr32(r + 0, w[0]);                                      /* t_trig */
        wr32(r + 4, w[1] & 0xFFFFu);                            /* sym (u16) + 2 pad bytes */
        wr32(r + 8, w[2]);                                      /* sig_a */
        wr32(r + 12, w[3]);                                     /* sig_b */
        ch_n++;
    }
}

static void drain_all(void)
{
    drain_events();
    drain_timing();
    drain_control();
    drain_payload();
    drain_chest();
}

/* Hold the detector in reset, write the parameters, release it. */
static void detector_reset(void)
{
    u32 i;
    if (!det_ok || !det_cfg) {
        return;
    }
    pd_wr(R_CTRL, CTRL_RESET);
    for (i = 0; i < 9u; i++) {
        u32 v = rd32(params + 4u * i);
        if (i == 5u) {
            continue;                                           /* tone_thresh: TONE_LIN below */
        }
        /* register order: ETA_HI ETA_LO QUAL N_CFO BLANK HOLDOFF STAGE SHIFT TONE_LIN */
        pd_wr(R_PARAM0 + 4u * (i < 5u ? i : i - 1u), v);
    }
    pd_wr(R_PARAM0 + 4u * 8u, tone_thresh_lin(rd32(params + 20)));
    if (det_version >= PD_ID_V3) {
        pd_wr(R_T_PK, rd32(params + 36));
    }
    if (det_version >= PD_ID_V5) {
        pd_wr(R_REARM, rd32(params + 40));
    }
    pd_wr(R_CTRL, 0);
}

/*
 * Stream `bytes` from src through the PL, in chunks; with dst != 0 the PL's output stream is
 * written to dst (each MM2S transfer ends with TLAST, which ends the matching S2MM transfer).
 * The output has 1 / div of the input's length (div 2: the half-band, one record per pair).
 * With events != 0 the detector's event FIFO is read while the DMA runs.
 * Returns 0 or a negative error code (message in err).
 */
static int dma_stream(UINTPTR src, UINTPTR dst, u32 bytes, u32 div, int events, char *err, u32 err_len)
{
    u32 off;

    Xil_DCacheFlushRange(src, bytes);
    if (dst) {
        Xil_DCacheInvalidateRange(dst, bytes / div);
    }
    for (off = 0; off < bytes; off += DMA_CHUNK_BYTES) {
        u32 n = bytes - off < DMA_CHUNK_BYTES ? bytes - off : DMA_CHUNK_BYTES;
        u32 polls = DMA_TIMEOUT_POLLS;
        u32 sr_rx = 0, sr_tx, got;

        if (dst && XAxiDma_SimpleTransfer(&dma, dst + off / div, n / div, XAXIDMA_DEVICE_TO_DMA) != XST_SUCCESS) {
            snprintf(err, err_len, "S2MM start failed at byte %lu", (unsigned long)off);
            dma_reset();
            return -1;
        }
        if (XAxiDma_SimpleTransfer(&dma, src + off, n, XAXIDMA_DMA_TO_DEVICE) != XST_SUCCESS) {
            snprintf(err, err_len, "MM2S start failed at byte %lu", (unsigned long)off);
            dma_reset();
            return -2;
        }
        while (XAxiDma_Busy(&dma, XAXIDMA_DMA_TO_DEVICE) ||
               (dst && XAxiDma_Busy(&dma, XAXIDMA_DEVICE_TO_DMA))) {
            if (events) {
                drain_all();
            }
            if (--polls == 0) {
                sr_tx = XAxiDma_ReadReg(dma.RegBase + XAXIDMA_TX_OFFSET, XAXIDMA_SR_OFFSET);
                sr_rx = XAxiDma_ReadReg(dma.RegBase + XAXIDMA_RX_OFFSET, XAXIDMA_SR_OFFSET);
                snprintf(err, err_len, "DMA timeout at byte %lu (MM2S SR 0x%08lx, S2MM SR 0x%08lx)",
                         (unsigned long)off, (unsigned long)sr_tx, (unsigned long)sr_rx);
                dma_reset();
                return -3;
            }
        }
        sr_tx = XAxiDma_ReadReg(dma.RegBase + XAXIDMA_TX_OFFSET, XAXIDMA_SR_OFFSET);
        if (dst) {
            sr_rx = XAxiDma_ReadReg(dma.RegBase + XAXIDMA_RX_OFFSET, XAXIDMA_SR_OFFSET);
        }
        if ((sr_tx | sr_rx) & XAXIDMA_ERR_ALL_MASK) {
            snprintf(err, err_len, "DMA error (MM2S SR 0x%08lx, S2MM SR 0x%08lx)",
                     (unsigned long)sr_tx, (unsigned long)sr_rx);
            dma_reset();
            return -4;
        }
        if (dst) {
            got = XAxiDma_ReadReg(dma.RegBase + XAXIDMA_RX_OFFSET, XAXIDMA_BUFFLEN_OFFSET);
            if (got != n / div) {
                snprintf(err, err_len, "S2MM received %lu bytes, expected %lu",
                         (unsigned long)got, (unsigned long)(n / div));
                return -5;
            }
        }
    }
    if (dst) {
        Xil_DCacheInvalidateRange(dst, bytes / div);
    }
    return 0;
}

/*
 * Detector RUN of `count` samples from the sample buffer. Fills ev_out; returns 0 or an error
 * (message in err).
 */
static int detector_run(u32 count, int debug, char *err, u32 err_len)
{
    u32 start, start20, polls, st, n_cfo = rd32(params + 12);
    u32 div = rd32(params + 28) == 2u ? 2u : 1u;    /* input stage 2: 2 input samples per detector sample */

    ev_n = 0;
    ev_lost = 0;
    tm_n = 0;
    tm_lost = 0;
    cf_n = 0;
    cf_lost = 0;
    pl_n = 0;
    pl_lost = 0;
    ch_n = 0;
    ch_lost = 0;
    start = pd_rd(R_SAMPLES);
    start20 = start / div;
    pd_wr(R_CTRL, debug ? CTRL_DEBUG : 0u);
    if (count > 0 && dma_stream(SAMPLE_BUF_ADDR, debug ? DEBUG_BUF_ADDR : 0, 4u * count, div, 1, err, err_len) != 0) {
        pd_wr(R_CTRL, 0);
        return -1;
    }
    /* the last samples, CFO results and tone checks finish without further input */
    for (polls = IDLE_TIMEOUT_POLLS; (pd_rd(R_STATUS) & STATUS_BUSY) && polls > 0; polls--) {
        drain_all();
    }
    drain_all();
    st = pd_rd(R_STATUS);
    pd_wr(R_CTRL, 0);
    if (st & STATUS_BUSY) {
        snprintf(err, err_len, "detector still busy after the DMA (STATUS 0x%08lx)", (unsigned long)st);
        return -2;
    }
    if (st & STATUS_EV_OVF) {
        snprintf(err, err_len, "detector event FIFO overflow (STATUS 0x%08lx)", (unsigned long)st);
        return -3;
    }
    if (st & (STATUS_TM_OVF | STATUS_TM_ERR)) {
        snprintf(err, err_len, "timing FIFO overflow or start queue overflow (STATUS 0x%08lx)", (unsigned long)st);
        return -3;
    }
    if (st & STATUS_ARM_LATE) {
        snprintf(err, err_len, "re-arm decision after its sample (STATUS 0x%08lx)", (unsigned long)st);
        return -3;
    }
    if (st & STATUS_CH_ERR) {
        snprintf(err, err_len, "channel-estimate packet queue overflow (STATUS 0x%08lx)", (unsigned long)st);
        return -3;
    }
    if (st & (STATUS_CF_OVF | STATUS_CF_ERR)) {
        snprintf(err, err_len, "control FIFO overflow or start queue overflow (STATUS 0x%08lx)", (unsigned long)st);
        return -3;
    }
    if (cf_lost) {
        snprintf(err, err_len, "more than %lu control records in one RUN - run in smaller pieces",
                 (unsigned long)MAX_CF);
        return -5;
    }
    if (pl_lost) {
        snprintf(err, err_len, "more than %lu payload records in one RUN - run in smaller pieces",
                 (unsigned long)MAX_PL);
        return -5;
    }
    if (ch_lost) {
        snprintf(err, err_len, "more than %lu channel-estimate records in one RUN - run in smaller pieces",
                 (unsigned long)MAX_CH);
        return -5;
    }
    if (tm_lost) {
        snprintf(err, err_len, "more than %lu timing records in one RUN - run in smaller pieces",
                 (unsigned long)MAX_TM);
        return -5;
    }
    if (pd_rd(R_SAMPLES) - start != count) {
        snprintf(err, err_len, "detector took %lu of %lu samples",
                 (unsigned long)(pd_rd(R_SAMPLES) - start), (unsigned long)count);
        return -4;
    }
    if (ev_lost) {
        snprintf(err, err_len, "more than %lu events in one RUN - run in smaller pieces",
                 (unsigned long)MAX_EVENTS);
        return -5;
    }
    /* a window still waiting for samples: report it once, in the RUN that triggered it */
    if ((st & STATUS_OPEN) && pd_rd(R_OPEN_T) - start20 < count / div) {
        u32 t = pd_rd(R_OPEN_T);
        u8 *r = ev_out + 8u + EV_SIZE * ev_n;
        memset(r, 0, EV_SIZE);
        wr32(r + 0, t);
        wr32(r + 8, t + n_cfo);
        wr16(r + 14, (u16)EV_TRUNCATED);
        ev_n++;
    }
    wr32(ev_out + 0, ev_n);
    wr32(ev_out + 4, count);
    /* reply: n_events, count, events, then n_timing and the timing records, then (build 11)
     * n_control and the control records, then (build 13) n_payload and the payload records,
     * then (build 14) n_chest and the channel-estimate records */
    wr32(ev_out + 8u + EV_SIZE * ev_n, tm_n);
    memcpy(ev_out + 12u + EV_SIZE * ev_n, tm_out, TM_SIZE * tm_n);
    wr32(ev_out + 12u + EV_SIZE * ev_n + TM_SIZE * tm_n, cf_n);
    memcpy(ev_out + 16u + EV_SIZE * ev_n + TM_SIZE * tm_n, cf_out, CF_SIZE * cf_n);
    wr32(ev_out + 16u + EV_SIZE * ev_n + TM_SIZE * tm_n + CF_SIZE * cf_n, pl_n);
    memcpy(ev_out + 20u + EV_SIZE * ev_n + TM_SIZE * tm_n + CF_SIZE * cf_n, pl_out, PL_SIZE * pl_n);
    wr32(ev_out + 20u + EV_SIZE * ev_n + TM_SIZE * tm_n + CF_SIZE * cf_n + PL_SIZE * pl_n, ch_n);
    memcpy(ev_out + 24u + EV_SIZE * ev_n + TM_SIZE * tm_n + CF_SIZE * cf_n + PL_SIZE * pl_n, ch_out,
           PL_SIZE * ch_n);
    return 0;
}

/* ---------------------------------------------------------------------------------- */
/* Transmit (replies can be up to 1 MB; sent as lwIP's send buffer allows)             */
/* ---------------------------------------------------------------------------------- */
static void tx_pump(struct tcp_pcb *pcb)
{
    while (tx_off < tx_total) {
        u32 space = tcp_sndbuf(pcb);
        const u8 *p;
        u32 n;
        err_t e;

        if (space == 0) {
            break;
        }
        if (tx_off < HDR_SIZE) {
            p = tx_hdr + tx_off;
            n = HDR_SIZE - tx_off;
        } else {
            p = tx_data + (tx_off - HDR_SIZE);
            n = tx_total - tx_off;
        }
        if (n > space) {
            n = space;
        }
        if (n > 0xFFFFu) {
            n = 0xFFFFu;
        }
        e = tcp_write(pcb, p, (u16)n, TCP_WRITE_FLAG_COPY |
                      (tx_off + n < tx_total ? TCP_WRITE_FLAG_MORE : 0));
        if (e == ERR_MEM) {
            break;                  /* lwIP queue full: continue from sent_cb */
        }
        if (e != ERR_OK) {
            xil_printf("tcp_write error %d\r\n", e);
            tx_total = tx_off;      /* drop the rest; the PC will time out */
            break;
        }
        tx_off += n;
    }
    tcp_output(pcb);
}

static void reply(struct tcp_pcb *pcb, u16 cmd, u16 status, u32 seq, const u8 *data, u32 len)
{
    wr32(tx_hdr + 0, PD_MAGIC);
    wr16(tx_hdr + 4, cmd);
    wr16(tx_hdr + 6, status);
    wr32(tx_hdr + 8, seq);
    wr32(tx_hdr + 12, len);
    tx_data = data;
    tx_total = HDR_SIZE + len;
    tx_off = 0;
    tx_pump(pcb);
}

static void reply_error(struct tcp_pcb *pcb, u16 cmd, u16 status, u32 seq, const char *msg)
{
    u32 n = (u32)strlen(msg);
    if (n > sizeof(tx_small)) {
        n = sizeof(tx_small);
    }
    memcpy(tx_small, msg, n);
    xil_printf("cmd %d: %s\r\n", cmd, msg);
    reply(pcb, cmd, status, seq, tx_small, n);
}

/* ---------------------------------------------------------------------------------- */
/* Commands                                                                            */
/* ---------------------------------------------------------------------------------- */
static void handle_message(struct tcp_pcb *pcb)
{
    u16 cmd;
    u32 seq, len;
    const u8 *pl = rx_buf + HDR_SIZE;
    char msg[160];

    memcpy(&cmd, rx_buf + 4, 2);
    seq = rd32(rx_buf + 8);
    len = rx_len;

    switch (cmd) {
    case CMD_PING:
        wr32(tx_small + 0, PD_PROTOCOL_VERSION);
        wr32(tx_small + 4, PD_BUILD);
        wr32(tx_small + 8, MAX_SAMPLES);
        wr32(tx_small + 12, !det_ok ? BOARD_FLAG_NO_DETECTOR :
                            (det_version >= PD_ID_V3 ? BOARD_FLAG_TIMING : 0u) |
                            (det_version >= PD_ID_V4 ? BOARD_FLAG_CF : 0u) |
                            (det_version >= PD_ID_V5 ? BOARD_FLAG_REARM : 0u) |
                            (det_version >= PD_ID_V6 ? BOARD_FLAG_PAYLOAD : 0u) |
                            (det_version >= PD_ID_V7 ? BOARD_FLAG_CHEST : 0u));
        reply(pcb, cmd, ST_OK, seq, tx_small, 16);
        return;

    case CMD_RESET:
        seg_n = 0;
        load_n = 0;
        samples_stored = 0;
        debug_count = 0;
        detector_reset();
        reply(pcb, cmd, ST_OK, seq, NULL, 0);
        return;

    case CMD_SET_PARAMS:
        if (len < PARAMS_SIZE_V1) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "SET_PARAMS payload too short");
            return;
        }
        if (rd32(pl + 20) > TONE_THRESH_MAX_Q8) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "tone_thresh must be 0..24 dB (Q8.8 0..6144)");
            return;
        }
        if (rd32(pl + 28) > 2u) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq,
                        "input_stage must be 0 (LPF output), 1 (20 Msps input) or 2 (40 Msps input)");
            return;
        }
        if (rd32(pl + 28) == 2u && det_ok && det_version < PD_ID_V2) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "input_stage 2 needs the bitstream with the half-band (ID 2.0)");
            return;
        }
        if (rd32(pl + 12) < 128u || rd32(pl + 12) > 4096u) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "n_cfo must be 128..4096");
            return;
        }
        if (rd32(pl + 0) > 0xFFFFu || rd32(pl + 4) > 0xFFFFu || rd32(pl + 8) > 0xFFFFu || rd32(pl + 32) > 15u) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "eta_*_sq and qual_len are 16-bit, dc_shift 0..15");
            return;
        }
        if (len >= PARAMS_SIZE_V2 && rd32(pl + 36) > 0xFFFFu) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "t_pk is 16-bit (Q8.8)");
            return;
        }
        if (len >= PARAMS_SIZE && rd32(pl + 40) > 1u) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "rearm must be 0 (holdoff) or 1 (state machine)");
            return;
        }
        memcpy(params, pl, PARAMS_SIZE_V1);
        wr32(params + 36, len >= PARAMS_SIZE_V2 ? rd32(pl + 36) : T_PK_DEFAULT);
        wr32(params + 40, len >= PARAMS_SIZE ? rd32(pl + 40) : 0u);   /* older PCs: the holdoff */
        det_cfg = 1;
        detector_reset();                   /* new parameters take effect from a reset */
        reply(pcb, cmd, ST_OK, seq, NULL, 0);
        return;

    case CMD_LOAD_SAMPLES: {
        u32 offset, count;
        if (len < 4 || (len - 4) % 4 != 0) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "LOAD_SAMPLES: payload must be u32 offset + 4-byte samples");
            return;
        }
        offset = rd32(pl);
        count = (len - 4) / 4;
        if (offset > MAX_SAMPLES || count > MAX_SAMPLES - offset) {
            snprintf(msg, sizeof(msg), "buffer holds %lu samples, load ends at %lu",
                     (unsigned long)MAX_SAMPLES, (unsigned long)offset + count);
            reply_error(pcb, cmd, ST_NO_SPACE, seq, msg);
            return;
        }
        memcpy((u8 *)(UINTPTR)SAMPLE_BUF_ADDR + 4u * offset, pl + 4, 4u * count);
        if (load_n < LOAD_LOG_MAX) {
            load_log[load_n][0] = offset;
            load_log[load_n][1] = count;
            load_log[load_n][2] = msg_start + HDR_SIZE + 4u;
            load_n++;
        }
        if (offset + count > samples_stored) {
            samples_stored = offset + count;
        }
        wr32(tx_small, samples_stored);
        reply(pcb, cmd, ST_OK, seq, tx_small, 4);
        return;
    }

    case CMD_READ_SAMPLES: {
        u32 offset, count;
        if (len < 8) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "READ_SAMPLES: payload must be u32 offset, u32 count");
            return;
        }
        offset = rd32(pl);
        count = rd32(pl + 4);
        if (offset > samples_stored || count > samples_stored - offset || count > MAX_PAYLOAD / 4) {
            snprintf(msg, sizeof(msg), "read %lu+%lu outside buffer %lu",
                     (unsigned long)offset, (unsigned long)count, (unsigned long)samples_stored);
            reply_error(pcb, cmd, ST_BAD_ARG, seq, msg);
            return;
        }
        reply(pcb, cmd, ST_OK, seq, (const u8 *)(UINTPTR)SAMPLE_BUF_ADDR + 4u * offset, 4u * count);
        return;
    }

    case CMD_RUN: {
        u32 count, flags;
        if (len < 8) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "RUN: payload must be u32 count, u32 flags");
            return;
        }
        count = rd32(pl);
        flags = rd32(pl + 4);
        if (count == 0) {
            count = samples_stored;
        }
        if (count > samples_stored) {
            snprintf(msg, sizeof(msg), "run %lu samples, buffer has %lu",
                     (unsigned long)count, (unsigned long)samples_stored);
            reply_error(pcb, cmd, ST_BAD_ARG, seq, msg);
            return;
        }
        if (!dma_ok) {
            reply_error(pcb, cmd, ST_INTERNAL, seq, "DMA not initialised - see the UART log");
            return;
        }
        if (!(flags & RUN_FLAG_PL_LOOPBACK)) {
            if (!det_ok) {
                reply_error(pcb, cmd, ST_BAD_CMD, seq,
                            "no detector in this bitstream - use RUN with the PL-loopback flag");
                return;
            }
            if (rd32(params + 28) == 2u && (count & 1u)) {
                reply_error(pcb, cmd, ST_BAD_ARG, seq,
                            "input_stage 2: RUN count must be even (whole 40 Msps sample pairs)");
                return;
            }
            if (!det_cfg) {
                det_cfg = 1;
                detector_reset();
            }
            debug_count = 0;
            if (detector_run(count, (flags & RUN_FLAG_DEBUG) != 0, msg, sizeof(msg)) != 0) {
                reply_error(pcb, cmd, ST_INTERNAL, seq, msg);
                return;
            }
            if (flags & RUN_FLAG_DEBUG) {
                debug_count = rd32(params + 28) == 2u ? count / 2u : count;
            }
            reply(pcb, cmd, ST_OK, seq, ev_out,
                  24u + EV_SIZE * ev_n + TM_SIZE * tm_n + CF_SIZE * cf_n + PL_SIZE * pl_n + PL_SIZE * ch_n);
            return;
        }
        debug_count = 0;
        if (det_ok && !det_cfg) {
            det_cfg = 1;
            detector_reset();
        }
        if (det_ok) {
            pd_wr(R_CTRL, CTRL_LOOPBACK);
        }
        if (count > 0 && dma_stream(SAMPLE_BUF_ADDR, DEBUG_BUF_ADDR, 4u * count, 1, 0, msg, sizeof(msg)) != 0) {
            if (det_ok) {
                pd_wr(R_CTRL, 0);
            }
            reply_error(pcb, cmd, ST_INTERNAL, seq, msg);
            return;
        }
        if (det_ok) {
            pd_wr(R_CTRL, 0);
        }
        debug_count = count;
        wr32(tx_small + 0, 0);          /* no events */
        wr32(tx_small + 4, count);      /* samples processed */
        reply(pcb, cmd, ST_OK, seq, tx_small, 8);
        return;
    }

    case CMD_MEMTEST: {
        /* u32 addr, u32 bytes, u32 wait_ms, u32 flags (bit0: detector held in reset,
         * bit1: ~address instead of address as the pattern). Fill, flush to DDR, wait,
         * invalidate, compare. Reply: u32 bad words, u32 n listed, n x (u32 addr, u32 value). */
        u32 addr, bytes, wait_ms, mflags, i, n_bad = 0, n_list = 0;
        volatile u32 *mp;
        u8 *out = ev_out;
        if (len < 16) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "MEMTEST: payload must be u32 addr, bytes, wait_ms, flags");
            return;
        }
        addr = rd32(pl); bytes = rd32(pl + 4); wait_ms = rd32(pl + 8); mflags = rd32(pl + 12);
        if (addr < 0x04000000u || (addr & 63u) || (bytes & 63u) || bytes == 0 ||
            bytes > 0x20000000u - addr || wait_ms > 60000u) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq,
                        "MEMTEST: addr >= 0x04000000, 64-byte aligned, inside DDR (< 0x20000000), wait <= 60 s");
            return;
        }
        if ((mflags & 1u) && det_ok) {
            pd_wr(R_CTRL, CTRL_RESET);
        }
        mp = (volatile u32 *)(UINTPTR)addr;
        for (i = 0; i < bytes / 4u; i++) {
            u32 a = addr + 4u * i;
            mp[i] = (mflags & 2u) ? ~a : a;
        }
        Xil_DCacheFlushRange(addr, bytes);
        if (wait_ms) {
            usleep(1000u * wait_ms);
        }
        Xil_DCacheInvalidateRange(addr, bytes);
        for (i = 0; i < bytes / 4u; i++) {
            u32 a = addr + 4u * i;
            u32 want = (mflags & 2u) ? ~a : a;
            u32 got = mp[i];
            if (got != want) {
                if (n_list < 1000u) {
                    wr32(out + 8u + 8u * n_list, a);
                    wr32(out + 12u + 8u * n_list, got);
                    n_list++;
                }
                n_bad++;
            }
        }
        if ((mflags & 1u) && det_ok) {
            detector_reset();
        }
        samples_stored = 0;                 /* the test overwrote the buffers */
        debug_count = 0;
        wr32(out, n_bad);
        wr32(out + 4, n_list);
        xil_printf("MEMTEST 0x%08x + 0x%x, wait %d ms: %d bad words\r\n",
                   (unsigned int)addr, (unsigned int)bytes, (int)wait_ms, (int)n_bad);
        reply(pcb, cmd, ST_OK, seq, out, 8u + 8u * n_list);
        return;
    }

    case CMD_REG_READ:
    case CMD_REG_WRITE: {
        u32 a;
        if (len < (cmd == CMD_REG_READ ? 4u : 8u)) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "REG_READ: u32 addr; REG_WRITE: u32 addr, u32 value");
            return;
        }
        a = rd32(pl);
        if (a < 0x40000000u || a > 0x7FFFFFFCu || (a & 3u)) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "register address must be 4-byte aligned in 0x40000000..0x7FFFFFFF");
            return;
        }
        if (cmd == CMD_REG_WRITE) {
            Xil_Out32(a, rd32(pl + 4));
        }
        wr32(tx_small, Xil_In32(a));
        reply(pcb, cmd, ST_OK, seq, tx_small, 4);
        return;
    }

    case CMD_FPU_TEST: {
        /* Diagnostic (build 7): u32 q8 dB, u32 iterations. Runs the double-precision code of
         * tone_thresh_lin / tone_db_q8 without touching the PL. Reply: u32 lin, u32 db_q8. */
        u32 q8, iters, i, lin = 0;
        s16 db = 0;
        if (len < 8) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "FPU_TEST: payload must be u32 q8, u32 iterations");
            return;
        }
        q8 = rd32(pl);
        iters = rd32(pl + 4);
        if (q8 > TONE_THRESH_MAX_Q8 || iters == 0 || iters > 100000u) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "FPU_TEST: q8 0..6144, iterations 1..100000");
            return;
        }
        for (i = 0; i < iters; i++) {
            lin = tone_thresh_lin(q8);
            db = tone_db_q8((u64)lin, 65536u);
        }
        wr32(tx_small, lin);
        wr32(tx_small + 4, (u32)(s32)db);
        reply(pcb, cmd, ST_OK, seq, tx_small, 8);
        return;
    }

    case CMD_PATTERN_CHECK: {
        /* Diagnostic (build 7): compare the sample buffer, on the board, with the pattern
         * sample k = (k & 0x7FF, (k >> 11) & 0x7FF) that pd_bisect.py loads. Bad words found
         * here came in on the receive path (or were written into DDR afterwards); words that
         * are only wrong in a READ_SAMPLES reply were damaged on the send path.
         * Reply: u32 bad, u32 n listed, n x (u32 sample index, u32 value). */
        u32 k, n_bad = 0, n_list = 0;
        const volatile u32 *sp = (const volatile u32 *)(UINTPTR)SAMPLE_BUF_ADDR;
        u8 *out = ev_out;
        for (k = 0; k < samples_stored; k++) {
            u32 want = (k & 0x7FFu) | (((k >> 11) & 0x7FFu) << 16);
            u32 got = sp[k];
            if (got != want) {
                if (n_list < 1000u) {
                    wr32(out + 8u + 8u * n_list, k);
                    wr32(out + 12u + 8u * n_list, got);
                    n_list++;
                }
                n_bad++;
            }
        }
        wr32(out, n_bad);
        wr32(out + 4, n_list);
        reply(pcb, cmd, ST_OK, seq, out, 8u + 8u * n_list);
        return;
    }

    case CMD_SEG_LOG: {
        /* Diagnostic (build 8). Reply: u32 n_loads, u32 n_segs, n_loads x (offset, count,
         * stream pos of sample 0), n_segs x (stream pos, payload address, length). */
        u8 *out = ev_out;
        u32 nl = load_n, ns = seg_n;
        if (8u + 12u * (nl + ns) > sizeof(ev_out)) {
            ns = (u32)((sizeof(ev_out) - 8u) / 12u) - nl;
        }
        wr32(out, nl);
        wr32(out + 4, ns);
        memcpy(out + 8, load_log, 12u * nl);
        memcpy(out + 8 + 12u * nl, seg_log, 12u * ns);
        reply(pcb, cmd, ST_OK, seq, out, 8u + 12u * (nl + ns));
        return;
    }

    case CMD_GET_DEBUG: {
        u32 offset, count;
        if (len < 8) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "GET_DEBUG: payload must be u32 offset, u32 count");
            return;
        }
        if (debug_count == 0) {
            reply_error(pcb, cmd, ST_NO_DEBUG, seq, "no debug capture - RUN first");
            return;
        }
        offset = rd32(pl);
        count = rd32(pl + 4);
        if (offset > debug_count || count > debug_count - offset || count > MAX_PAYLOAD / 4) {
            snprintf(msg, sizeof(msg), "debug %lu+%lu outside capture %lu",
                     (unsigned long)offset, (unsigned long)count, (unsigned long)debug_count);
            reply_error(pcb, cmd, ST_BAD_ARG, seq, msg);
            return;
        }
        reply(pcb, cmd, ST_OK, seq, (const u8 *)(UINTPTR)DEBUG_BUF_ADDR + 4u * offset, 4u * count);
        return;
    }

    default:
        snprintf(msg, sizeof(msg), "unknown command %u", (unsigned)cmd);
        reply_error(pcb, cmd, ST_BAD_CMD, seq, msg);
        return;
    }
}

/* ---------------------------------------------------------------------------------- */
/* lwIP callbacks                                                                      */
/* ---------------------------------------------------------------------------------- */
static void drop_client(struct tcp_pcb *pcb, int abort)
{
    tcp_arg(pcb, NULL);
    tcp_recv(pcb, NULL);
    tcp_sent(pcb, NULL);
    tcp_err(pcb, NULL);
    if (abort) {
        tcp_abort(pcb);
    } else if (tcp_close(pcb) != ERR_OK) {
        tcp_abort(pcb);
    }
    if (pcb == client) {
        client = NULL;
    }
    reset_connection_state();
}

static err_t sent_cb(void *arg, struct tcp_pcb *pcb, u16_t len)
{
    (void)arg;
    (void)len;
    tx_pump(pcb);
    return ERR_OK;
}

static void err_cb(void *arg, err_t err)
{
    (void)arg;
    xil_printf("Connection error %d\r\n", err);
    client = NULL;                  /* lwIP has already freed the pcb */
    reset_connection_state();
}

static err_t recv_cb(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    u32 total, off = 0;
    (void)arg;

    if (p == NULL) {                /* peer closed */
        xil_printf("Client disconnected\r\n");
        drop_client(pcb, 0);
        return ERR_OK;
    }
    if (err != ERR_OK) {
        pbuf_free(p);
        return err;
    }

    total = p->tot_len;
    {
        const struct pbuf *q;
        u32 pos = rx_stream;
        for (q = p; q != NULL && seg_n < SEG_LOG_MAX; q = q->next) {
            seg_log[seg_n][0] = pos;
            seg_log[seg_n][1] = (u32)(UINTPTR)q->payload;
            seg_log[seg_n][2] = q->len;
            seg_n++;
            pos += q->len;
            if (q->len == q->tot_len) {
                break;
            }
        }
    }
    while (off < total) {
        if (rx_have == 0) {
            msg_start = rx_stream + off;
        }
        u32 want = rx_have < HDR_SIZE ? HDR_SIZE - rx_have : HDR_SIZE + rx_len - rx_have;
        u32 n = total - off < want ? total - off : want;

        pbuf_copy_partial(p, rx_buf + rx_have, (u16_t)n, (u16_t)off);
        rx_have += n;
        off += n;

        if (rx_have == HDR_SIZE) {
            if (rd32(rx_buf) != PD_MAGIC) {
                xil_printf("Bad magic 0x%08x - closing connection\r\n", (unsigned int)rd32(rx_buf));
                pbuf_free(p);
                drop_client(pcb, 1);
                return ERR_ABRT;
            }
            rx_len = rd32(rx_buf + 12);
            if (rx_len > MAX_PAYLOAD) {
                xil_printf("Payload %d bytes too long - closing connection\r\n", (int)rx_len);
                pbuf_free(p);
                drop_client(pcb, 1);
                return ERR_ABRT;
            }
        }
        if (rx_have >= HDR_SIZE && rx_have == HDR_SIZE + rx_len) {
            handle_message(pcb);
            rx_have = 0;
            rx_len = 0;
        }
    }
    rx_stream += total;
    tcp_recved(pcb, (u16_t)total);
    pbuf_free(p);
    return ERR_OK;
}

static err_t accept_cb(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    (void)arg;
    if (err != ERR_OK || newpcb == NULL) {
        return ERR_VAL;
    }
    if (client != NULL) {           /* one PC at a time: the newest connection wins */
        xil_printf("New connection replaces the previous one\r\n");
        drop_client(client, 1);
    }
    client = newpcb;
    reset_connection_state();
    tcp_nagle_disable(newpcb);
    tcp_arg(newpcb, NULL);
    tcp_recv(newpcb, recv_cb);
    tcp_sent(newpcb, sent_cb);
    tcp_err(newpcb, err_cb);
    xil_printf("Client connected\r\n");
    return ERR_OK;
}

/* pd_protocol.Params defaults, until the PC sends SET_PARAMS */
static void pr_default_params(void)
{
    static const u32 def[11] = {8028u, 2621u, 128u, 2048u, 288u, 3328u, 131072u, 0u, 10u, T_PK_DEFAULT, 1u};
    u32 i;
    for (i = 0; i < 11u; i++) {
        wr32(params + 4u * i, def[i]);
    }
}

/* ---------------------------------------------------------------------------------- */
/* Entry points called by the template's main.c                                        */
/* ---------------------------------------------------------------------------------- */
void print_app_header(void)
{
    xil_printf("\r\n----- Flexlink packet-detector test bench (build %d) -----\r\n", (int)PD_BUILD);
    xil_printf("TCP port %d, sample buffer %d samples at 0x%08x\r\n",
               PD_PORT, (int)MAX_SAMPLES, (unsigned int)SAMPLE_BUF_ADDR);
    xil_printf("Phase 2: packet detector in the PL, registers at 0x%08x\r\n", (unsigned int)PD_BASE);
}

int start_application(void)
{
    struct tcp_pcb *pcb;
    err_t err;

    dma_ok = (dma_init() == 0);
    if (!dma_ok) {
        xil_printf("WARNING: DMA unavailable - LOAD/READ work, RUN will fail\r\n");
    }
    {
        u32 id = pd_rd(R_ID);
        det_ok = ((id & PD_ID_MASK) == PD_ID_VALUE);
        det_version = id & 0xFFFFu;
        if (det_ok) {
            pr_default_params();            /* build 7: configured at the first SET_PARAMS / RUN */
            xil_printf("Detector found (ID 0x%08x)\r\n", (unsigned int)id);
        } else {
            xil_printf("WARNING: no detector at 0x%08x (ID 0x%08x) - PL loopback only\r\n",
                       (unsigned int)PD_BASE, (unsigned int)id);
        }
    }

    pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (pcb == NULL) {
        xil_printf("Error creating PCB\r\n");
        return -1;
    }
    err = tcp_bind(pcb, IP_ANY_TYPE, PD_PORT);
    if (err != ERR_OK) {
        xil_printf("Unable to bind to port %d: err = %d\r\n", PD_PORT, err);
        return -2;
    }
    pcb = tcp_listen(pcb);
    if (pcb == NULL) {
        xil_printf("Out of memory while tcp_listen\r\n");
        return -3;
    }
    tcp_accept(pcb, accept_cb);
    xil_printf("Packet-detector server listening on port %d\r\n", PD_PORT);
    return 0;
}

int transfer_data(void)
{
    return 0;
}
