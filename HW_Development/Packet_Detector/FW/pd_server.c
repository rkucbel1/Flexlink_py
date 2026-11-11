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
 *     STREAM_START 18 / POLL_RESULTS 19 / STREAM_STOP 20 (build 25): the CoreBoard stream
 *     through the receiver in real time; see "Streaming receiver" below
 *     build 35: while streaming, TCP replies paced one segment per 100 us (GEM overruns);
 *     build 36: the interval set by STREAM_START flags bits 15:8 (25 us units)
 *     build 31: FIFO levels read once per drain; with bitstream 9.2 CTRL.NOREC while streaming
 *     (no PL / CH / LQ records, FIFOs drained every 500 us instead of 50 us)
 *     REG_READ / REG_WRITE 9/10 (build 28): also the DDRC port priorities 0xF8006208..0xF8006224
 *     GEM_STATS 17 (build 18): the Ethernet controller's whole statistics block, summed since
 *     UDP_START (frames, size histogram, FCS / symbol / alignment errors, drops)
 *     UDP_START 14 / UDP_STATS 15 / UDP_STOP 16 (build 17): receive the CoreBoard's ESR2 sample
 *     stream (UDP, 1376-byte datagrams) and count it; mode 1 also converts the samples into the
 *     sample buffer (a ring), as the streaming receiver will. See "CoreBoard UDP stream" below.
 *     REG_READ 9 / REG_WRITE 10 (diagnostic: a 32-bit register in the PL range
 *     0x40000000..0x7FFFFFFF, e.g. the AXI DMA or the detector)
 */
#include <stdio.h>
#include <string.h>

#include "xparameters.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "xil_mmu.h"
#include "xil_exception.h"
#include "xil_types.h"
#include "xstatus.h"
#include "xaxidma.h"
#include "xil_io.h"
#include "sleep.h"

#include "lwip/err.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"

/* ---------------------------------------------------------------------------------- */
/* Configuration                                                                       */
/* ---------------------------------------------------------------------------------- */
#define PD_PORT                 5001
#define PD_PROTOCOL_VERSION     1u
#define PD_BUILD                38u              /* bump when the firmware changes */

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
#define PD_ID_V8                0x0800u         /* 8.0: payload demapper / combiner (LQ FIFO) */
#define PD_ID_V9                0x0900u         /* 9.0: LDPC decoder, transport blocks (TB FIFO) */
#define PD_ID_V91               0x0901u         /* 9.1: CTRL.PACK8, int8 input unpacker (pd_unpack8.v) */
#define PD_ID_V92               0x0902u         /* 9.2: CTRL.NOREC, payload / channel / decoder-input records discarded in the PL */
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
#define R_LQ_LEVEL              0x6Cu
#define R_LQ_DATA               0x70u
#define R_TB_LEVEL              0x74u
#define R_TB_DATA               0x78u
#define CTRL_RESET              0x1u
#define CTRL_DEBUG              0x2u
#define CTRL_LOOPBACK           0x4u
#define CTRL_PACK8              0x8u            /* 9.1: two int8 samples per DMA word */
#define CTRL_NOREC              0x10u           /* 9.2: discard PL / CH / LQ records in the PL */
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
       CMD_PATTERN_CHECK = 12, CMD_SEG_LOG = 13,
       CMD_UDP_START = 14, CMD_UDP_STATS = 15, CMD_UDP_STOP = 16, CMD_GEM_STATS = 17,
       CMD_STREAM_START = 18, CMD_POLL_RESULTS = 19, CMD_STREAM_STOP = 20 };
enum { ST_OK = 0, ST_BAD_CMD = 1, ST_BAD_ARG = 2, ST_NO_SPACE = 3, ST_NO_DEBUG = 4,
       ST_INTERNAL = 5 };

#define RUN_FLAG_DEBUG          0x1u
#define RUN_FLAG_PL_LOOPBACK    0x2u
#define RUN_FLAG_PACK8          0x4u            /* build 24: the buffer holds packed int8 samples */
#define BOARD_FLAG_EMULATOR     0x1u
#define BOARD_FLAG_NO_DETECTOR  0x2u
#define BOARD_FLAG_TIMING       0x4u            /* bitstream has the Preamble B timing (ID 3.0) */
#define BOARD_FLAG_CF           0x8u            /* bitstream has the control-field receiver (ID 4.0) */
#define BOARD_FLAG_REARM        0x10u           /* bitstream has the state machine's re-arm (ID 5.0) */
#define BOARD_FLAG_PAYLOAD      0x20u           /* bitstream has the speculative payload FFT (ID 6.0) */
#define BOARD_FLAG_CHEST        0x40u           /* bitstream has the payload channel estimate (ID 7.0) */
#define BOARD_FLAG_LQ           0x80u           /* bitstream has the demapper / combiner (ID 8.0) */
#define BOARD_FLAG_TB           0x100u          /* bitstream has the LDPC decoder (ID 9.0) */
#define BOARD_FLAG_PACK8        0x200u          /* bitstream has the int8 unpacker (ID 9.1) */
#define BOARD_FLAG_NOREC        0x400u          /* bitstream can discard the debug records (ID 9.2) */
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
#define MAX_LQ                  4096u           /* decoder-input records per RUN (PL_SIZE each) */
#define TB_HDR_SIZE             12u             /* transport block (pd_protocol.TBRecord): header ... */
#define MAX_TB_BYTES            (384u << 10)    /* ... + 4 bytes per 32 bits; per RUN / POLL. build 29:
                                                 * 192 -> 384 KB, ~0.5 s of the stream's blocks */
#define MAX_EVENTS              ((MAX_PAYLOAD - 32u - MAX_TM * TM_SIZE - MAX_CF * CF_SIZE - MAX_PL * PL_SIZE \
                                  - MAX_CH * PL_SIZE - MAX_LQ * PL_SIZE - MAX_TB_BYTES - EV_SIZE) / EV_SIZE)
typedef char max_events_check[(MAX_EVENTS >= 4096u) ? 1 : -1];  /* the buffers fit in MAX_PAYLOAD */
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
/* build 35: while streaming, replies go out one TCP segment per TX_PACE_COUNTS instead of
 * as many as the window allows. The Ethernet controller's single DMA serves send and receive;
 * a burst of back-to-back outgoing frames made it overrun on the 670 Mbit/s incoming stream
 * (about one lost datagram per 7 KB sent, build 34 test). The results need ~1 MB/s; one
 * segment per 100 us allows 14 MB/s. */
static int       tx_paced;
static u64       tx_t_pace;         /* XTime, declared further down */
static u64       tx_pace_counts;    /* build 36: set per stream, default 100 us */
#define TX_PACE_DEFAULT_US      100u
/* build 37: a token bucket - one segment per interval on average, at most TX_PACE_BURST back
 * to back when the main loop came late (build 36 sent one segment per main-loop pass at most,
 * ~550 segments/s whatever the interval, too slow for the results in packet-dense parts) */
#define TX_PACE_BURST           1u      /* build 38: 3 cost ~100 extra lost datagrams in 30 s;
                                         * with the check every ~16 datagrams from the receive
                                         * path one segment at a time still allows ~3,700/s */

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
static u8  lq_out[MAX_LQ * PL_SIZE]; /* decoder-input records of the RUN */
static u32 lq_n;
static u8  tb_out[MAX_TB_BYTES];     /* transport blocks of the RUN */
static u32 tb_n, tb_bytes;
static int tb_lost;
static int lq_lost;

/* ---------------------------------------------------------------------------------- */
/* Helpers                                                                             */
/* ---------------------------------------------------------------------------------- */
static u32 rd32(const u8 *p) { u32 v; memcpy(&v, p, 4); return v; }       /* ARM is LE */
static void wr32(u8 *p, u32 v) { memcpy(p, &v, 4); }
static void wr16(u8 *p, u16 v) { memcpy(p, &v, 2); }

static u32 pd_reads;                /* build 30: AXI-Lite reads of the receiver (streaming CPU cost) */
static u32  pd_rd(u32 off)        { pd_reads++; return Xil_In32(PD_BASE + off); }
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
    u32 lv;
    for (lv = pd_rd(R_EV_LEVEL); lv >= 8u; lv -= 8u) {   /* build 31: level read once */
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
    u32 lv;
    for (lv = pd_rd(R_TM_LEVEL); lv >= 8u; lv -= 8u) {   /* build 31: level read once */
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
    u32 lv;
    for (lv = pd_rd(R_CF_LEVEL); lv >= 8u; lv -= 8u) {   /* build 31: level read once */
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
    u32 lv;
    for (lv = pd_rd(R_PL_LEVEL); lv >= 4u; lv -= 4u) {   /* build 31: level read once */
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
    u32 lv;
    for (lv = pd_rd(R_CH_LEVEL); lv >= 4u; lv -= 4u) {   /* build 31: level read once */
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

/* Read every complete decoder-input record (4 words) into lq_out (PayloadRecord layout: t_trig,
 * code word, sig_a, sig_b). */
static void drain_lq(void)
{
    if (det_version < PD_ID_V8) {
        return;
    }
    u32 lv;
    for (lv = pd_rd(R_LQ_LEVEL); lv >= 4u; lv -= 4u) {   /* build 31: level read once */
        u32 w[4], i;
        u8 *r;
        for (i = 0; i < 4u; i++) {
            w[i] = pd_rd(R_LQ_DATA);
        }
        if (lq_n >= MAX_LQ) {
            lq_lost = 1;
            continue;
        }
        r = lq_out + PL_SIZE * lq_n;
        wr32(r + 0, w[0]);                                      /* t_trig */
        wr32(r + 4, w[1] & 0xFFFFu);                            /* code word (u16) + 2 pad bytes */
        wr32(r + 8, w[2]);                                      /* sig_a */
        wr32(r + 12, w[3]);                                     /* sig_b */
        lq_n++;
    }
}

/* Read every complete transport-block record into tb_out (pd_protocol.TBRecord: u32 t_trig,
 * u16 w, u8 crc_ok, u8 iterations, u16 nbits, 2 pad bytes, then ceil(nbits / 32) u32 words).
 * The FIFO counts only words of complete records. */
static void drain_tb(void)
{
    if (det_version < PD_ID_V9) {
        return;
    }
    u32 lv = pd_rd(R_TB_LEVEL);                 /* build 31: read once; counts whole records */
    while (lv >= 2u) {
        u32 t = pd_rd(R_TB_DATA);
        u32 h = pd_rd(R_TB_DATA);
        u32 nb = h & 0x7FFu, nw = (nb + 31u) / 32u, i;
        lv = (lv >= 2u + nw) ? lv - 2u - nw : 0u;
        int keep = (tb_bytes + TB_HDR_SIZE + 4u * nw <= MAX_TB_BYTES);
        u8 *r = tb_out + tb_bytes;
        if (keep) {
            wr32(r + 0, t);
            wr16(r + 4, (u16)((h >> 14) & 0x3FFu));                 /* code word */
            r[6] = (u8)(h >> 31);                                    /* crc_ok */
            r[7] = (u8)((h >> 24) & 0xFu);                           /* iterations */
            wr16(r + 8, (u16)nb);
            wr16(r + 10, 0);
        }
        for (i = 0; i < nw; i++) {
            u32 w = pd_rd(R_TB_DATA);
            if (keep) {
                wr32(r + TB_HDR_SIZE + 4u * i, w);
            }
        }
        if (keep) {
            tb_bytes += TB_HDR_SIZE + 4u * nw;
            tb_n++;
        } else {
            tb_lost = 1;
        }
    }
}

static void drain_all(void)
{
    drain_events();
    drain_timing();
    drain_control();
    drain_payload();
    drain_chest();
    drain_lq();
    drain_tb();
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

/* Time stamps: the Cortex-A9 global timer (64-bit, CPU clock / 2), read directly. The SDT BSP
 * (Vitis 2023.2+) has no xtime_l.h any more; this is what its XTime_GetTime did. */
typedef u64 XTime;
#define GTIMER_BASE             0xF8F00200u
#if defined(XPAR_CPU_CORE_CLOCK_FREQ_HZ)
#define COUNTS_PER_SECOND       (XPAR_CPU_CORE_CLOCK_FREQ_HZ / 2u)
#elif defined(XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ)
#define COUNTS_PER_SECOND       (XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ / 2u)
#else
#define COUNTS_PER_SECOND       333333333u      /* Zedboard: 666.67 MHz CPU */
#endif

static void gtimer_enable(void)                 /* start it if the BSP has not */
{
    u32 ctrl = Xil_In32(GTIMER_BASE + 0x08u);
    if ((ctrl & 1u) == 0u) {
        Xil_Out32(GTIMER_BASE + 0x08u, (ctrl & ~0xFF00u) | 1u);   /* prescaler 0, enable */
    }
}

static void XTime_GetTime(XTime *t)
{
    u32 hi, lo;
    do {
        hi = Xil_In32(GTIMER_BASE + 0x04u);
        lo = Xil_In32(GTIMER_BASE + 0x00u);
    } while (Xil_In32(GTIMER_BASE + 0x04u) != hi);
    *t = ((u64)hi << 32) | lo;
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
        XTime t_start, t_now;
        XTime_GetTime(&t_start);
        while (XAxiDma_Busy(&dma, XAXIDMA_DMA_TO_DEVICE) ||
               (dst && XAxiDma_Busy(&dma, XAXIDMA_DEVICE_TO_DMA))) {
            if (events) {
                drain_all();
            }
            /* build 27: also a 5 s limit (a hung receiver used to block the board for minutes:
             * 200 M polls with a FIFO drain each) */
            if ((polls & 0x3FFu) == 0u) {
                XTime_GetTime(&t_now);
                if (t_now - t_start > 5u * (XTime)COUNTS_PER_SECOND) {
                    polls = 1;
                }
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
static int detector_run(u32 count, int debug, int pack, char *err, u32 err_len)
{
    /* pack (build 24, bitstream 9.1): the buffer holds count int8 samples, two per word, the
     * PL unpacks them (CTRL.PACK8); the DMA moves 2 * count bytes instead of 4 * count */
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
    lq_n = 0;
    tb_n = 0;
    tb_bytes = 0;
    tb_lost = 0;
    lq_lost = 0;
    start = pd_rd(R_SAMPLES);
    start20 = start / div;
    pd_wr(R_CTRL, (debug ? CTRL_DEBUG : 0u) | (pack ? CTRL_PACK8 : 0u));
    /* dma_stream's div is input bytes per debug byte: 4 * div / 4 normally; packed, the input
     * is 2 bytes per sample, so 1 with input stage 2 (debug with stage 1 is refused by RUN) */
    if (count > 0 && dma_stream(SAMPLE_BUF_ADDR, debug ? DEBUG_BUF_ADDR : 0, (pack ? 2u : 4u) * count,
                                pack ? 1u : div, 1, err, err_len) != 0) {
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
    if (tb_lost) {
        snprintf(err, err_len, "more than %lu bytes of transport blocks in one RUN - run in smaller pieces",
                 (unsigned long)MAX_TB_BYTES);
        return -5;
    }
    if (lq_lost) {
        snprintf(err, err_len, "more than %lu decoder-input records in one RUN - run in smaller pieces",
                 (unsigned long)MAX_LQ);
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
     * then (build 14) n_chest and the channel-estimate records, then (build 15) n_lq and the
     * decoder-input records, then (build 16) n_tb and the transport blocks */
    wr32(ev_out + 8u + EV_SIZE * ev_n, tm_n);
    memcpy(ev_out + 12u + EV_SIZE * ev_n, tm_out, TM_SIZE * tm_n);
    wr32(ev_out + 12u + EV_SIZE * ev_n + TM_SIZE * tm_n, cf_n);
    memcpy(ev_out + 16u + EV_SIZE * ev_n + TM_SIZE * tm_n, cf_out, CF_SIZE * cf_n);
    wr32(ev_out + 16u + EV_SIZE * ev_n + TM_SIZE * tm_n + CF_SIZE * cf_n, pl_n);
    memcpy(ev_out + 20u + EV_SIZE * ev_n + TM_SIZE * tm_n + CF_SIZE * cf_n, pl_out, PL_SIZE * pl_n);
    wr32(ev_out + 20u + EV_SIZE * ev_n + TM_SIZE * tm_n + CF_SIZE * cf_n + PL_SIZE * pl_n, ch_n);
    memcpy(ev_out + 24u + EV_SIZE * ev_n + TM_SIZE * tm_n + CF_SIZE * cf_n + PL_SIZE * pl_n, ch_out,
           PL_SIZE * ch_n);
    wr32(ev_out + 24u + EV_SIZE * ev_n + TM_SIZE * tm_n + CF_SIZE * cf_n + PL_SIZE * (pl_n + ch_n), lq_n);
    memcpy(ev_out + 28u + EV_SIZE * ev_n + TM_SIZE * tm_n + CF_SIZE * cf_n + PL_SIZE * (pl_n + ch_n), lq_out,
           PL_SIZE * lq_n);
    wr32(ev_out + 28u + EV_SIZE * ev_n + TM_SIZE * tm_n + CF_SIZE * cf_n + PL_SIZE * (pl_n + ch_n + lq_n), tb_n);
    memcpy(ev_out + 32u + EV_SIZE * ev_n + TM_SIZE * tm_n + CF_SIZE * cf_n + PL_SIZE * (pl_n + ch_n + lq_n), tb_out,
           tb_bytes);
    return 0;
}

/* ---------------------------------------------------------------------------------- */
/* Transmit (replies can be up to 1 MB; sent as lwIP's send buffer allows)             */
/* ---------------------------------------------------------------------------------- */
static void tx_send(struct tcp_pcb *pcb, u32 budget)
{
    while (tx_off < tx_total && budget > 0) {
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
        if (n > budget) {
            n = budget;
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
        budget -= n;
    }
    tcp_output(pcb);
}

/* build 35/37: paced sending while streaming, from the main loop and the UDP receive path */
static void tx_pace(void)
{
    XTime now;
    u64 k;
    if (!tx_paced || client == NULL || tx_off >= tx_total) {
        return;
    }
    XTime_GetTime(&now);
    if (now - tx_t_pace > (u64)TX_PACE_BURST * tx_pace_counts) {
        tx_t_pace = now - (u64)TX_PACE_BURST * tx_pace_counts;   /* idle: at most a short burst */
    }
    k = (now - tx_t_pace) / tx_pace_counts;     /* segments due */
    if (k == 0) {
        return;
    }
    tx_t_pace += k * tx_pace_counts;
    tx_send(client, (u32)k * (u32)TCP_MSS);
}

static void tx_pump(struct tcp_pcb *pcb)
{
    if (tx_paced && pcb == client) {
        tx_pace();
    } else {
        tx_send(pcb, 0xFFFFFFFFu);
    }
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
/* ---------------------------------------------------------------------------------- */
/* CoreBoard UDP stream (build 17)                                                     */
/* ---------------------------------------------------------------------------------- */
/* The ESP32-S31 CoreBoard (ESP-SDR, or App/FlexlinkCoreBoardEmulator.py on the PC) sends
 * 1376-byte datagrams: 32-byte header <4sHHIIQQ ("ESR2", version, bits, epoch, rate, sample
 * position, time_us) + 672 complex int8 samples (I, Q). At 40 Msps that is 59,524 datagrams/s.
 *
 * This first step only receives and counts, to see whether the PS keeps up:
 *   mode 0  check the header, track the sample position (lost / late / duplicate datagrams)
 * Build 21: modes 1..3 differ in how the ring is written, to measure what each costs:
 *   mode 1  convert into the cached ring, no cache maintenance (CPU cost of the conversion
 *           only; a DMA would not see all the data - test only)
 *   mode 2  convert into the ring mapped non-cacheable
 *   mode 3  convert into the cached ring and flush each datagram's 2688 bytes
 *   mode 4  (build 24) copy the raw int8 bytes into the ring mapped non-cacheable, two samples
 *           per word, for the PL unpacker (bitstream 9.1, CTRL.PACK8): half the bytes of
 *           mode 2 and no arithmetic
 * Overload guard (build 23: also for 10 ms after the driver found the pbuf pool empty, as an
 * overload in the interrupt and callback does not stop the main loop but starves TCP):
 * when the main loop has not run for 20 ms (the receive path cannot keep up and
 * would starve TCP), conversions are skipped and counted ("skipped") until it runs again, so
 * the board keeps answering and the count shows how far short the CPU is.
 * Build 19/20 notes:
 *   mode 1  as mode 0, and convert the samples to the detector's format (u32: I << 4 low half,
 *           Q << 4 high half, 12-bit) into the sample buffer, used as a ring of whole
 *           datagrams: the CPU work of the later streaming receiver. While mode 1 runs, the
 *           sample buffer is mapped non-cacheable (build 19), so the AXI DMA sees the samples
 *           without a cache flush per datagram (84 lines of L1 + L2 maintenance each, too slow
 *           at 59,524 datagrams/s); UDP_STOP maps it cacheable again.
 * The receive path is compiled with -O2 (attribute below) although the rest of the firmware
 * is built with -O0: at -O0 the conversion alone needs more than the whole CPU.
 * The GEM's own statistics count what never reached lwIP: RXRESERRCNT (no receive buffer:
 * the software did not hand buffers back in time) and RXORCNT (receive overrun: the GEM could
 * not write to memory fast enough). They clear when read, so they are summed here. */
#define ESR2_MAGIC              0x32525345u     /* "ESR2" little-endian */
#define ESR2_HDR                32u
#define ESR2_SPP                672u            /* complex samples per datagram */
#define ESR2_LEN                (ESR2_HDR + 2u * ESR2_SPP)
#define UDP_DEFAULT_PORT        4000u
#define UDP_SEEN_WIN            8192u           /* datagrams remembered for late / duplicate */
#define RING_PKTS               (MAX_SAMPLES / ESR2_SPP)
#define RING8_PKTS              (BUF_BYTES / (2u * ESR2_SPP))   /* mode 4: 1344 bytes per datagram */
#define UDP_STATS_SIZE          96u
#define UDP_OVERLOAD_COUNTS     (COUNTS_PER_SECOND / 50u)   /* 20 ms without a main-loop pass */

#if defined(XPAR_XEMACPS_0_BASEADDR)
#define GEM_BASE XPAR_XEMACPS_0_BASEADDR
#elif defined(XPAR_PS7_ETHERNET_0_BASEADDR)
#define GEM_BASE XPAR_PS7_ETHERNET_0_BASEADDR
#else
#define GEM_BASE 0xE000B000u
#endif
/* Statistics block 0x100..0x1B0 (45 registers, clear on read; names in PC/pd_udp_test.py) */
#define GEM_STAT_FIRST          0x100u
#define GEM_STAT_N              45u
#define GEM_IDX(off)            (((off) - GEM_STAT_FIRST) / 4u)
#define GEM_RXCNT               0x158u          /* frames received OK */
#define GEM_RXRESERRCNT         0x1A0u          /* frames lost: no receive buffer */
#define GEM_RXORCNT             0x1A4u          /* frames lost: receive overrun */

static struct udp_pcb *udp_pcb_s;
static u32 udp_mode, udp_port;
static XTime udp_t_start, udp_t_first, udp_t_last, udp_t_busy, udp_t_print;
static u32 udp_ok, udp_bad_size, udp_bad_magic, udp_epoch, udp_epochs;
static u32 udp_lost, udp_late, udp_dup;
static u32 gem_acc[GEM_STAT_N];                 /* statistics summed since UDP_START */
static u32 udp_loops, udp_print_ok, udp_print_lost;
static u64 udp_expect, udp_first_pos, udp_last_pos;
static int udp_have_first;
static u32 udp_skipped;                         /* conversions skipped by the overload guard */
/* Build 22: the lwIP driver's "pbuf pool empty" count in the rx interrupt (xemacpsif_dma.c,
 * patched by build_firmware.py; weak, so an unpatched BSP still links and reports 0) */
extern volatile unsigned int pd_rx_alloc_fail __attribute__((weak));
static u32 udp_alloc0;
static u32 udp_guard_fails;
static XTime udp_guard_until;

static u32 rx_alloc_fail(void) { return &pd_rx_alloc_fail ? pd_rx_alloc_fail : 0u; }
static XTime udp_t_poll;                        /* last main-loop pass (udp_poll) */
static u32 udp_seen[UDP_SEEN_WIN];              /* datagram number + 1 last stored in this slot */
static u8  udp_tmp[ESR2_LEN] __attribute__((aligned(8)));

static u32 ms_since(XTime t0, XTime t1) { return (u32)((t1 - t0) / (COUNTS_PER_SECOND / 1000u)); }

static void gem_counters(void)                  /* sum the clear-on-read GEM statistics */
{
    u32 i;
    for (i = 0; i < GEM_STAT_N; i++) {
        gem_acc[i] += Xil_In32(GEM_BASE + GEM_STAT_FIRST + 4u * i);
    }
}
static u32 gem_rx_frames(void)                  /* rx size histogram 0x168..0x17C, summed */
{
    u32 i, n = 0;
    for (i = GEM_IDX(0x168u); i <= GEM_IDX(0x17Cu); i++) {
        n += gem_acc[i];
    }
    return n;
}
#define gem_reserr  gem_acc[GEM_IDX(GEM_RXRESERRCNT)]
#define gem_overrun gem_acc[GEM_IDX(GEM_RXORCNT)]

static void udp_clear_stats(void)
{
    udp_ok = udp_bad_size = udp_bad_magic = udp_epoch = udp_epochs = 0;
    udp_lost = udp_late = udp_dup = 0;
    udp_loops = udp_print_ok = udp_print_lost = 0;
    udp_skipped = 0;
    udp_alloc0 = udp_guard_fails = rx_alloc_fail();
    udp_guard_until = 0;
    udp_t_first = udp_t_last = udp_t_busy = 0;
    udp_have_first = 0;
    udp_expect = udp_first_pos = udp_last_pos = 0;
    memset(udp_seen, 0, sizeof udp_seen);
    gem_counters();                             /* discard what was counted before */
    memset(gem_acc, 0, sizeof gem_acc);
    XTime_GetTime(&udp_t_start);
    udp_t_print = udp_t_poll = udp_t_start;
}

#define UDP_HOT __attribute__((optimize("O2")))

static int ring_uncached;                       /* sample buffer mapped non-cacheable */

static void ring_map(int uncached)              /* the sample buffer's 128 x 1 MB sections */
{
    u32 a;
    if (uncached == ring_uncached) {
        return;
    }
    Xil_DCacheFlush();                          /* write back / drop its cached lines first */
    for (a = SAMPLE_BUF_ADDR; a < SAMPLE_BUF_ADDR + BUF_BYTES; a += 0x100000u) {
        Xil_SetTlbAttributes(a, uncached ? NORM_NONCACHE : NORM_WB_CACHE);
    }
    ring_uncached = uncached;
}

static UDP_HOT void udp_store(const u8 *iq, u64 dgram, int flush)  /* int8 I/Q -> u32 12-bit, ring */
{
    /* Build 23: read in place, one halfword = one sample (I low byte, Q high byte). The UDP
     * payload is only 2-byte aligned, and the driver has invalidated it, so it comes from DDR:
     * prefetch two cache lines ahead to overlap the misses with the conversion. */
    u32 *dst = (u32 *)(UINTPTR)SAMPLE_BUF_ADDR + (u32)(dgram % RING_PKTS) * ESR2_SPP;
    const u16 *src = (const u16 *)iq;
    u32 k;
    for (k = 0; k < ESR2_SPP; k += 4u) {        /* 4 samples = 8 bytes per pass */
        if ((k & 15u) == 0u) {
            __builtin_prefetch(src + k + 32u);  /* 64 bytes = 2 lines ahead */
        }
        u32 h0 = src[k], h1 = src[k + 1u], h2 = src[k + 2u], h3 = src[k + 3u];
        dst[k]      = ((u32)((s32)(s8)h0 * 16) & 0xFFFFu) | ((u32)((s32)(s8)(h0 >> 8) * 16) << 16);
        dst[k + 1u] = ((u32)((s32)(s8)h1 * 16) & 0xFFFFu) | ((u32)((s32)(s8)(h1 >> 8) * 16) << 16);
        dst[k + 2u] = ((u32)((s32)(s8)h2 * 16) & 0xFFFFu) | ((u32)((s32)(s8)(h2 >> 8) * 16) << 16);
        dst[k + 3u] = ((u32)((s32)(s8)h3 * 16) & 0xFFFFu) | ((u32)((s32)(s8)(h3 >> 8) * 16) << 16);
    }
    if (flush) {
        Xil_DCacheFlushRange((UINTPTR)dst, ESR2_SPP * 4u);
    }
}

static UDP_HOT void udp_store_raw(const u8 *iq, u64 dgram)  /* mode 4: int8 I/Q bytes, ring */
{
    u32 *dst = (u32 *)(UINTPTR)SAMPLE_BUF_ADDR + (u32)(dgram % RING8_PKTS) * (ESR2_SPP / 2u);
    const u16 *src = (const u16 *)iq;           /* 2-byte aligned: halfword loads */
    u32 k;
    for (k = 0; k < ESR2_SPP; k += 8u) {        /* 8 samples = 16 bytes = 4 words per pass */
        if ((k & 15u) == 0u) {
            __builtin_prefetch(src + k + 32u);
        }
        dst[k / 2u]      = (u32)src[k]      | ((u32)src[k + 1u] << 16);
        dst[k / 2u + 1u] = (u32)src[k + 2u] | ((u32)src[k + 3u] << 16);
        dst[k / 2u + 2u] = (u32)src[k + 4u] | ((u32)src[k + 5u] << 16);
        dst[k / 2u + 3u] = (u32)src[k + 6u] | ((u32)src[k + 7u] << 16);
    }
}

static u32 rd16x2(const u8 *p)                  /* u32 from a 2-byte aligned address */
{
    const u16 *h = (const u16 *)p;
    return (u32)h[0] | ((u32)h[1] << 16);
}

static UDP_HOT void stream_rx(const u8 *iq, u64 n, u32 epoch);
static void stream_poll(void);

static UDP_HOT void udp_rx_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    XTime t0, t1;
    const u8 *d;
    u32 epoch, slot;
    u64 pos, n;
    (void)arg; (void)pcb; (void)addr; (void)port;

    XTime_GetTime(&t0);
    if (p->tot_len != ESR2_LEN) {
        udp_bad_size++;
        goto done;
    }
    /* The hot path is -O2, where a 4- or 8-byte memcpy becomes a plain LDR / LDRD, a data
     * abort at an unaligned address when alignment checks are on. The UDP payload is 2-byte
     * aligned (42 bytes into the frame), so since build 23 everything is read with halfword
     * loads in place; build 20-22 copied every datagram (a misaligned 1376-byte memcpy, ~9 us).
     * Only an odd address or a chained pbuf is still copied. */
    if (p->len >= ESR2_LEN && ((UINTPTR)p->payload & 1u) == 0u) {
        d = (const u8 *)p->payload;
    } else {
        pbuf_copy_partial(p, udp_tmp, ESR2_LEN, 0);
        d = udp_tmp;
    }
    if (rd16x2(d) != ESR2_MAGIC) {
        udp_bad_magic++;
        goto done;
    }
    epoch = rd16x2(d + 8);
    pos = (u64)rd16x2(d + 16) | ((u64)rd16x2(d + 20) << 32);
    if (!udp_have_first || epoch != udp_epoch) {  /* first datagram, or the board restarted */
        udp_have_first = 1;
        udp_epoch = epoch;
        udp_epochs++;
        udp_expect = pos / ESR2_SPP;
        udp_first_pos = pos;
        udp_t_first = t0;
        memset(udp_seen, 0, sizeof udp_seen);
    }
    n = pos / ESR2_SPP;                         /* datagram number in the stream */
    slot = (u32)(n % UDP_SEEN_WIN);
    if (n >= udp_expect) {
        udp_lost += (u32)(n - udp_expect);      /* skipped datagrams (may still arrive late) */
        udp_expect = n + 1u;
        udp_last_pos = pos;
    } else if (udp_expect - n <= UDP_SEEN_WIN && udp_seen[slot] == (u32)n + 1u) {
        udp_dup++;
        goto done;
    } else {                                    /* arrived after a later one: not lost after all */
        udp_late++;
        if (udp_lost) {
            udp_lost--;
        }
    }
    udp_seen[slot] = (u32)n + 1u;
    udp_ok++;
    udp_t_last = t0;
    if (udp_mode == 5u) {
        stream_rx(d + ESR2_HDR, n, epoch);
    } else if (udp_mode != 0u) {
        u32 fails = rx_alloc_fail();
        if (fails != udp_guard_fails) {         /* the driver ran out of pbufs: back off 10 ms */
            udp_guard_fails = fails;
            udp_guard_until = t0 + COUNTS_PER_SECOND / 100u;
        }
        if (t0 < udp_guard_until || t0 - udp_t_poll > UDP_OVERLOAD_COUNTS) {
            udp_skipped++;                      /* overloaded: let the main loop (TCP) run */
        } else {
            if (udp_mode == 4u) {
                udp_store_raw(d + ESR2_HDR, n);
            } else {
                udp_store(d + ESR2_HDR, n, udp_mode == 3u);
            }
        }
    }
done:
    pbuf_free(p);
    XTime_GetTime(&t1);
    udp_t_busy += t1 - t0;
}

static int udp_start(u32 port, u32 mode)
{
    err_t err;
    if (udp_pcb_s != NULL) {
        udp_remove(udp_pcb_s);
        udp_pcb_s = NULL;
    }
    udp_pcb_s = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (udp_pcb_s == NULL) {
        return -1;
    }
    err = udp_bind(udp_pcb_s, IP_ANY_TYPE, (u16_t)port);
    if (err != ERR_OK) {
        udp_remove(udp_pcb_s);
        udp_pcb_s = NULL;
        return -2;
    }
    udp_mode = mode;
    udp_port = port;
    ring_map(mode == 2u || mode == 4u || mode == 5u);
    gtimer_enable();
    udp_clear_stats();
    udp_recv(udp_pcb_s, udp_rx_cb, NULL);
    xil_printf("UDP stream receiver on port %d, mode %d (%s)\r\n", (int)port, (int)mode,
               mode == 0u ? "count only" : mode == 1u ? "convert, cached ring, no flush" :
               mode == 2u ? "convert, non-cacheable ring" : mode == 3u ? "convert, cached ring + flush" :
               mode == 4u ? "raw int8 copy, non-cacheable ring (PL unpacker)" : "streaming receiver");
    return 0;
}

/* ---------------------------------------------------------------------------------- */
/* Streaming receiver (build 25)                                                       */
/* ---------------------------------------------------------------------------------- */
/* STREAM_START puts the UDP receiver in mode 5: every ESR2 datagram's raw int8 samples go into
 * the sample buffer, a ring of RING8_PKTS slots of 1344 bytes (one datagram each, mapped
 * non-cacheable), at slot k = datagram number - n0. From the main loop (transfer_data) the AXI
 * DMA feeds the ring to the receiver with CTRL.PACK8 (bitstream 9.1: the PL unpacks two int8
 * samples per word), in chunks of ST_CHUNK slots, without waiting for it; the result FIFOs are
 * drained every 50 us into the record buffers, and POLL_RESULTS hands them to the PC.
 *
 *   lost datagram    its slot is zero-filled when a later one arrives, so the receiver's sample
 *                    count stays that of the stream (t_trig * 2 = stream sample at 40 Msps)
 *   late datagram    written into its slot if the DMA has not read it yet (ST_HOLD slots are
 *                    held back for that), else counted as too late
 *   gap > ST_MAX_FILL slots, or no room in the ring for the zeros: resynchronise (the stream
 *                    jumps; counted), new epoch (CoreBoard restarted): resynchronise too
 *   ring full        (the receiver slower than the stream for > 1.6 s) datagram dropped, counted
 *
 * The CoreBoard sends the stream to whoever sent its HTTP "start": STREAM_START can carry the
 * board's address and the JSON body, and the Zedboard then posts it itself (and {"op":"stop"}
 * at STREAM_STOP); without an address the PC starts the stream (e.g. the emulator's
 * --stream-to). */
#define ST_SLOT_BYTES           (2u * ESR2_SPP)
#define ST_SLOTS                RING8_PKTS
#define ST_CHUNK                64u             /* slots per DMA transfer (~1 ms at 40 Msps) */
#define ST_HOLD                 4u              /* newest slots held back for late datagrams */
#define ST_MAX_FILL             1024u           /* largest gap zero-filled (~17 ms) */
#define ST_IDLE_COUNTS          (COUNTS_PER_SECOND / 500u)    /* 2 ms quiet: flush the tail */
#define ST_DRAIN_COUNTS         (COUNTS_PER_SECOND / 20000u)  /* drain the FIFOs every 50 us */
#define ST_DRAIN_NOREC_COUNTS   (COUNTS_PER_SECOND / 2000u)   /* build 31, CTRL.NOREC: every 500 us
                                     * (the 16-record PL / CH / LQ FIFOs set the 50 us; without them
                                     * the 2048-word TB FIFO holds ~7 ms of blocks) */
#define ST_FLAG_KEEP_PL         0x1u            /* also return payload / channel / decoder-input records */
/* build 36: flags bits 15:8 = P, replies paced one TCP segment per 25 * P us (0: 100 us).
 * (Builds 32-35 used these bits for a register-read test: the reads did not cost frames.) */
#define ST_FLAG_PACE_SHIFT      8u
/* build 34 test: flags bits 7:1 = K, every POLL_RESULTS reply padded with K KB of zeros, to see
 * whether the board sending costs Ethernet frames (GEM overruns) */
#define ST_FLAG_PAD_SHIFT       1u              /* build 35: bits 6:1 only (<= 63 KB) */
#define ST_FLAG_NOPACE          0x80u           /* build 35: send replies unpaced (as before) */
#define ST_STATS_SIZE           260u            /* build 26: + the DMA watchdog block, 27: + gaps,
                                                 * 30: + where the CPU time goes */
#define ST_STALL_COUNTS         (COUNTS_PER_SECOND / 20u)     /* a transfer > 50 ms: stalled */
#define ST_STOP_COUNTS          (2u * COUNTS_PER_SECOND)      /* STREAM_STOP waits at most 2 s */
#define ST_LOST_EV 0x1u
#define ST_LOST_TM 0x2u
#define ST_LOST_CF 0x4u
#define ST_LOST_PL 0x8u
#define ST_LOST_CH 0x10u
#define ST_LOST_LQ 0x20u
#define ST_LOST_TB 0x40u

static int   st_on, st_have;
static int   st_norec;                       /* build 31: CTRL.NOREC set for this stream */
static u32   st_test_pad;                    /* build 34: bytes of zeros added to each poll reply */
static u32   st_flags, st_epoch;
static u64   st_n0, st_wr, st_rd, st_dma_end;
static u64   st_floor;                          /* first slot of the current numbering (resync) */
static u32   st_dma_len;
static XTime st_t_start, st_t_last, st_t_drain;
static u32   st_ok, st_lost, st_late_fixed, st_late_dropped, st_ring_full, st_resyncs;
static u32   st_ring_max, st_chunks, st_dma_errors, st_lost_mask, st_polls;
/* build 26: DMA watchdog - a transfer not done after 50 ms is recorded and the DMA reset */
static XTime st_dma_t0;
static u32   st_stalls, st_stall_sr, st_stall_status, st_stall_samples, st_stall_lv[7];
/* build 27: where the first zero-filled gaps went (slot, length), to match with a hang */
#define ST_GAPS                 8u
static u32   st_gap_k[ST_GAPS], st_gap_n[ST_GAPS], st_gaps;
/* build 30: where the CPU time goes while streaming (global-timer counts) */
static XTime st_drain_t, st_drain_max, st_poll_t, st_poll_max, st_loop_max, st_busy0;
static u32   st_reads0, st_drains;
static u8    poll_out[MAX_PAYLOAD] __attribute__((aligned(64)));  /* POLL / STOP replies (sent
                                                                     * asynchronously: own buffer) */

/* --- HTTP client for the CoreBoard's /api/rx (one request at a time) --- */
static struct tcp_pcb *http_pcb;
static char  http_req[1024];
static u32   http_req_len;
static int   http_state;                        /* 0 none, 1 connecting, 2 sent, 3 answered, 4 error */
static u32   http_code;                         /* HTTP status code of the answer */
static u32   http_err;
static char  http_head[16];
static u32   http_head_n;
static u32   cb_ip;                             /* CoreBoard address (network order as given) */
static u16   cb_port;

static int http_close(void)                     /* returns 1 if the pcb had to be aborted */
{
    int aborted = 0;
    if (http_pcb != NULL) {
        tcp_arg(http_pcb, NULL);
        tcp_recv(http_pcb, NULL);
        tcp_err(http_pcb, NULL);
        if (tcp_close(http_pcb) != ERR_OK) {
            tcp_abort(http_pcb);
            aborted = 1;
        }
        http_pcb = NULL;
    }
    return aborted;
}

static void http_err_cb(void *arg, err_t err)
{
    (void)arg;
    http_pcb = NULL;                            /* freed by lwIP */
    http_state = 4;
    http_err = (u32)(s32)err;
}

static err_t http_recv_cb(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    (void)arg;
    if (p == NULL || err != ERR_OK) {           /* the board closed the connection */
        if (p != NULL) {
            pbuf_free(p);
        }
        if (http_state == 2) {
            http_state = http_code ? 3 : 4;
        }
        return http_close() ? ERR_ABRT : ERR_OK;
    }
    if (http_head_n < sizeof http_head - 1u) {  /* "HTTP/1.x 200 ..." */
        u32 n = pbuf_copy_partial(p, http_head + http_head_n, (u16)(sizeof http_head - 1u - http_head_n), 0);
        http_head_n += n;
        http_head[http_head_n] = 0;
        if (http_head_n >= 12u) {
            http_code = (u32)(http_head[9] - '0') * 100u + (u32)(http_head[10] - '0') * 10u +
                        (u32)(http_head[11] - '0');
        }
    }
    tcp_recved(pcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

static err_t http_connected_cb(void *arg, struct tcp_pcb *pcb, err_t err)
{
    (void)arg;
    if (err != ERR_OK) {
        http_state = 4;
        http_err = (u32)(s32)err;
        return err;
    }
    if (tcp_write(pcb, http_req, (u16)http_req_len, TCP_WRITE_FLAG_COPY) != ERR_OK) {
        http_state = 4;
        return http_close() ? ERR_ABRT : ERR_OK;
    }
    tcp_output(pcb);
    http_state = 2;
    return ERR_OK;
}

static int http_post(const char *json, u32 json_len)
{
    ip_addr_t ip;
    int n;
    http_close();
    if (cb_ip == 0u) {
        return 0;
    }
    n = snprintf(http_req, sizeof http_req,
                 "POST /api/rx HTTP/1.1\r\nHost: %u.%u.%u.%u\r\nContent-Type: application/json\r\n"
                 "Content-Length: %lu\r\nConnection: close\r\n\r\n",
                 (unsigned)(cb_ip & 0xFFu), (unsigned)((cb_ip >> 8) & 0xFFu), (unsigned)((cb_ip >> 16) & 0xFFu),
                 (unsigned)(cb_ip >> 24), (unsigned long)json_len);
    if (n < 0 || (u32)n + json_len > sizeof http_req) {
        return -1;
    }
    memcpy(http_req + n, json, json_len);
    http_req_len = (u32)n + json_len;
    http_state = 1;
    http_code = 0;
    http_err = 0;
    http_head_n = 0;
    http_pcb = tcp_new_ip_type(IPADDR_TYPE_V4);
    if (http_pcb == NULL) {
        http_state = 4;
        return -2;
    }
    tcp_arg(http_pcb, NULL);
    tcp_err(http_pcb, http_err_cb);
    tcp_recv(http_pcb, http_recv_cb);
    IP_ADDR4(&ip, cb_ip & 0xFFu, (cb_ip >> 8) & 0xFFu, (cb_ip >> 16) & 0xFFu, cb_ip >> 24);
    if (tcp_connect(http_pcb, &ip, cb_port, http_connected_cb) != ERR_OK) {
        http_state = 4;
        http_close();
        return -3;
    }
    return 0;
}

/* --- ring --- */
static u8 *st_slot(u64 k) { return (u8 *)(UINTPTR)SAMPLE_BUF_ADDR + (u32)(k % ST_SLOTS) * ST_SLOT_BYTES; }

static void st_reset(u32 flags)
{
    st_flags = flags;
    {
        u32 pu = 25u * ((flags >> ST_FLAG_PACE_SHIFT) & 0xFFu);
        tx_pace_counts = (u64)(pu ? pu : TX_PACE_DEFAULT_US) * (COUNTS_PER_SECOND / 1000000u);
    }
    st_test_pad = 1024u * ((flags >> ST_FLAG_PAD_SHIFT) & 0x3Fu);
    st_have = 0;
    st_epoch = 0;
    st_n0 = st_wr = st_rd = st_dma_end = st_floor = 0;
    st_dma_len = 0;
    st_ok = st_lost = st_late_fixed = st_late_dropped = st_ring_full = st_resyncs = 0;
    st_ring_max = st_chunks = st_dma_errors = st_lost_mask = st_polls = 0;
    st_stalls = st_stall_sr = st_stall_status = st_stall_samples = 0;
    memset(st_stall_lv, 0, sizeof st_stall_lv);
    st_gaps = 0;
    st_drain_t = st_drain_max = st_poll_t = st_poll_max = st_loop_max = 0;
    st_drains = 0;
    st_reads0 = pd_reads;
    st_busy0 = udp_t_busy;
    XTime_GetTime(&st_t_start);
    st_t_last = st_t_drain = st_t_start;
    ev_n = 0; ev_lost = 0; tm_n = 0; tm_lost = 0; cf_n = 0; cf_lost = 0;
    pl_n = 0; pl_lost = 0; ch_n = 0; ch_lost = 0; lq_n = 0; lq_lost = 0;
    tb_n = 0; tb_bytes = 0; tb_lost = 0;
}

/* one datagram (from udp_rx_cb, mode 5); iq = the 1344 payload bytes, 2-byte aligned */
static UDP_HOT void stream_rx(const u8 *iq, u64 n, u32 epoch)
{
    u64 k, gap, used, freeslots, i;
    XTime_GetTime(&st_t_last);
    if (!st_have || epoch != st_epoch) {        /* first datagram, or the CoreBoard restarted */
        if (st_have) {
            st_resyncs++;
        }
        st_have = 1;
        st_epoch = epoch;
        st_n0 = n - st_wr;                      /* continue at the next free slot (modulo 2^64: */
        st_floor = st_wr;                       /* a restarted board counts from 0 again) */
    }
    k = n - st_n0;
    if ((s64)(k - st_wr) < 0) {                 /* late or duplicate (signed: k may be "below 0") */
        u64 lo = st_dma_len ? st_dma_end : st_rd;
        if ((s64)(st_floor - lo) > 0) {
            lo = st_floor;                      /* nothing from before a resync: other numbering */
        }
        if ((s64)(k - lo) >= 0 && st_wr - k < ST_SLOTS) {
            udp_store_raw(iq, k);               /* still unread: its slot gets the real samples */
            st_late_fixed++;
            if (st_lost) {
                st_lost--;
            }
        } else {
            st_late_dropped++;
        }
        return;
    }
    gap = k - st_wr;
    used = st_wr - st_rd;                       /* slots not yet fed (incl. the DMA in flight) */
    freeslots = ST_SLOTS - used;
    if (freeslots == 0u) {
        st_ring_full++;
        return;
    }
    if (gap > ST_MAX_FILL || gap + 1u > freeslots) {  /* resynchronise: the stream jumps */
        st_resyncs++;
        st_n0 += gap;
        k = st_wr;
        gap = 0;
        st_floor = st_wr;
    }
    for (i = 0; i < gap; i++) {                 /* lost datagrams: zeros */
        memset(st_slot(st_wr + i), 0, ST_SLOT_BYTES);
    }
    if (gap && st_gaps < ST_GAPS) {
        st_gap_k[st_gaps] = (u32)st_wr;
        st_gap_n[st_gaps] = (u32)gap;
        st_gaps++;
    }
    st_lost += (u32)gap;
    udp_store_raw(iq, k);
    st_wr = k + 1u;
    st_ok++;
    if ((st_ok & 15u) == 0u) {
        stream_poll();                          /* also during long receive bursts */
    }
    used = st_wr - st_rd;
    if (used > st_ring_max) {
        st_ring_max = (u32)used;
    }
}

static void stream_drain(void)
{
    XTime t0, t1;
    XTime_GetTime(&t0);
    if (st_norec) {                             /* build 31: the PL discards PL / CH / LQ records */
        drain_events();
        drain_timing();
        drain_control();
        drain_tb();
    } else {
        drain_all();
    }

    XTime_GetTime(&t1);
    st_drain_t += t1 - t0;
    if (t1 - t0 > st_drain_max) {
        st_drain_max = t1 - t0;
    }
    st_drains++;
    if (!(st_flags & ST_FLAG_KEEP_PL)) {        /* drained (the PL must not wait), not kept */
        pl_n = ch_n = lq_n = 0;
        pl_lost = ch_lost = lq_lost = 0;
    }
}

/* the DMA: finish the transfer in flight, start the next; returns 1 while a transfer runs */
static int stream_dma(int flush)
{
    u64 commit, avail, len, room;
    if (st_dma_len) {
        if (XAxiDma_Busy(&dma, XAXIDMA_DMA_TO_DEVICE)) {
            XTime now;
            XTime_GetTime(&now);
            if (now - st_dma_t0 <= ST_STALL_COUNTS) {
                return 1;
            }
            /* stalled: a halted DMA (error) never reports idle, a receiver that holds its input
             * never lets the transfer finish - keep what tells them apart, reset, go on */
            st_stalls++;
            st_stall_sr = XAxiDma_ReadReg(dma.RegBase + XAXIDMA_TX_OFFSET, XAXIDMA_SR_OFFSET);
            st_stall_status = pd_rd(R_STATUS);
            st_stall_samples = pd_rd(R_SAMPLES);
            st_stall_lv[0] = pd_rd(R_EV_LEVEL);
            st_stall_lv[1] = pd_rd(R_TM_LEVEL);
            st_stall_lv[2] = pd_rd(R_CF_LEVEL);
            st_stall_lv[3] = pd_rd(R_PL_LEVEL);
            st_stall_lv[4] = pd_rd(R_CH_LEVEL);
            st_stall_lv[5] = pd_rd(R_LQ_LEVEL);
            st_stall_lv[6] = pd_rd(R_TB_LEVEL);
            if (st_stalls <= 3u) {
                xil_printf("STREAM DMA STALL %d: MM2S SR 0x%08x, STATUS 0x%08x, samples %d, levels ev %d tm %d "
                           "cf %d pl %d ch %d lq %d tb %d\r\n", (int)st_stalls, (unsigned)st_stall_sr,
                           (unsigned)st_stall_status, (int)st_stall_samples, (int)st_stall_lv[0],
                           (int)st_stall_lv[1], (int)st_stall_lv[2], (int)st_stall_lv[3], (int)st_stall_lv[4],
                           (int)st_stall_lv[5], (int)st_stall_lv[6]);
            }
            dma_reset();
        } else if (XAxiDma_ReadReg(dma.RegBase + XAXIDMA_TX_OFFSET, XAXIDMA_SR_OFFSET) & XAXIDMA_ERR_ALL_MASK) {
            st_dma_errors++;
            dma_reset();
        }
        st_rd = st_dma_end;
        st_dma_len = 0;
        st_chunks++;
    }
    commit = st_wr > ST_HOLD ? st_wr - ST_HOLD : 0u;
    if (flush) {
        commit = st_wr;
    } else {
        XTime now;
        XTime_GetTime(&now);
        if (now - st_t_last > ST_IDLE_COUNTS) {
            commit = st_wr;                     /* quiet: feed the tail too */
            flush = 1;
        }
    }
    if (commit <= st_rd) {
        return 0;
    }
    avail = commit - st_rd;
    if (avail < ST_CHUNK && !flush) {
        return 0;
    }
    len = avail < ST_CHUNK ? avail : ST_CHUNK;
    room = ST_SLOTS - st_rd % ST_SLOTS;          /* not across the end of the ring */
    if (len > room) {
        len = room;
    }
    if (XAxiDma_SimpleTransfer(&dma, (UINTPTR)st_slot(st_rd), (u32)len * ST_SLOT_BYTES,
                               XAXIDMA_DMA_TO_DEVICE) != XST_SUCCESS) {
        st_dma_errors++;
        dma_reset();
        return 0;
    }
    st_dma_len = (u32)len;
    st_dma_end = st_rd + len;
    XTime_GetTime(&st_dma_t0);
    return 1;
}

static void stream_poll(void)                   /* from transfer_data() */
{
    XTime now;
    if (!st_on) {
        return;
    }
    stream_dma(0);
    tx_pace();                                  /* build 37: also during long receive bursts */
    XTime_GetTime(&now);
    if (now - st_t_drain > (st_norec ? ST_DRAIN_NOREC_COUNTS : ST_DRAIN_COUNTS)) {
        st_t_drain = now;
        stream_drain();
    }
}

static void stream_stats(u8 *o)
{
    XTime now;
    u32 st = det_ok ? pd_rd(R_STATUS) : 0u;
    XTime_GetTime(&now);
    if (udp_pcb_s != NULL) {
        gem_counters();
    }
    st_lost_mask |= (ev_lost ? ST_LOST_EV : 0u) | (tm_lost ? ST_LOST_TM : 0u) | (cf_lost ? ST_LOST_CF : 0u) |
                    (pl_lost ? ST_LOST_PL : 0u) | (ch_lost ? ST_LOST_CH : 0u) | (lq_lost ? ST_LOST_LQ : 0u) |
                    (tb_lost ? ST_LOST_TB : 0u);
    memset(o, 0, ST_STATS_SIZE);
    wr32(o + 0,  ST_STATS_SIZE);
    wr32(o + 4,  (u32)st_on);
    wr32(o + 8,  ms_since(st_t_start, now));
    wr32(o + 12, st_ok);
    wr32(o + 16, st_lost);
    wr32(o + 20, st_late_fixed);
    wr32(o + 24, st_late_dropped);
    wr32(o + 28, st_ring_full);
    wr32(o + 32, st_resyncs);
    wr32(o + 36, (u32)(st_wr - st_rd));
    wr32(o + 40, st_ring_max);
    wr32(o + 44, st_chunks);
    wr32(o + 48, st_dma_errors);
    wr32(o + 52, st);
    wr32(o + 56, rx_alloc_fail() - udp_alloc0);
    wr32(o + 60, gem_reserr);
    wr32(o + 64, gem_overrun);
    wr32(o + 68, st_lost_mask);
    wr32(o + 72, st_epoch);
    wr32(o + 76, det_ok ? pd_rd(R_SAMPLES) : 0u);
    {
        u64 fed = st_rd * ESR2_SPP;
        memcpy(o + 80, &fed, 8);
    }
    wr32(o + 88, (u32)http_state);
    wr32(o + 92, http_code);
    wr32(o + 96, http_err);
    wr32(o + 100, udp_bad_size + udp_bad_magic);
    wr32(o + 104, st_polls);
    wr32(o + 108, udp_dup);
    wr32(o + 112, st_stalls);
    wr32(o + 116, st_stall_sr);
    wr32(o + 120, st_stall_status);
    {
        u32 i;
        for (i = 0; i < 7u; i++) {
            wr32(o + 124u + 4u * i, st_stall_lv[i]);
        }
    }
    wr32(o + 152, st_stall_samples);
    wr32(o + 156, dma_ok ? XAxiDma_ReadReg(dma.RegBase + XAXIDMA_TX_OFFSET, XAXIDMA_SR_OFFSET) : 0u);
    wr32(o + 160, st_gaps);
    {
        u32 i;
        for (i = 0; i < ST_GAPS; i++) {
            wr32(o + 164u + 8u * i, st_gap_k[i]);
            wr32(o + 168u + 8u * i, st_gap_n[i]);
        }
    }
    /* build 30 (microseconds unless noted) */
    wr32(o + 228, pd_reads - st_reads0);
    wr32(o + 232, (u32)((udp_t_busy - st_busy0) / (COUNTS_PER_SECOND / 1000000u)));
    wr32(o + 236, (u32)(st_drain_t / (COUNTS_PER_SECOND / 1000000u)));
    wr32(o + 240, (u32)(st_drain_max / (COUNTS_PER_SECOND / 1000000u)));
    wr32(o + 244, (u32)(st_poll_t / (COUNTS_PER_SECOND / 1000000u)));
    wr32(o + 248, (u32)(st_poll_max / (COUNTS_PER_SECOND / 1000000u)));
    wr32(o + 252, (u32)(st_loop_max / (COUNTS_PER_SECOND / 1000000u)));
    wr32(o + 256, st_drains);
}

/* stats + the records collected since the last poll, in the RUN reply format; clears them */
static u32 stream_pack(void)
{
    u32 o, fixed, n_ev, keep = (st_flags & ST_FLAG_KEEP_PL) != 0u;
    u32 npl = keep ? pl_n : 0u, nch = keep ? ch_n : 0u, nlq = keep ? lq_n : 0u;
    stream_stats(poll_out);
    o = ST_STATS_SIZE;
    fixed = o + 32u + TM_SIZE * tm_n + CF_SIZE * cf_n + PL_SIZE * (npl + nch + nlq) + tb_bytes;
    n_ev = ev_n;
    if (fixed + EV_SIZE * n_ev > MAX_PAYLOAD) {  /* cannot happen with the buffer limits; be safe */
        n_ev = (MAX_PAYLOAD - fixed) / EV_SIZE;
        st_lost_mask |= ST_LOST_EV;
        wr32(poll_out + 68, st_lost_mask);
    }
    wr32(poll_out + o, n_ev);
    wr32(poll_out + o + 4u, 0u);
    memcpy(poll_out + o + 8u, ev_out + 8u, EV_SIZE * n_ev);
    o += 8u + EV_SIZE * n_ev;
    wr32(poll_out + o, tm_n);  memcpy(poll_out + o + 4u, tm_out, TM_SIZE * tm_n);  o += 4u + TM_SIZE * tm_n;
    wr32(poll_out + o, cf_n);  memcpy(poll_out + o + 4u, cf_out, CF_SIZE * cf_n);  o += 4u + CF_SIZE * cf_n;
    wr32(poll_out + o, npl);   memcpy(poll_out + o + 4u, pl_out, PL_SIZE * npl);   o += 4u + PL_SIZE * npl;
    wr32(poll_out + o, nch);   memcpy(poll_out + o + 4u, ch_out, PL_SIZE * nch);   o += 4u + PL_SIZE * nch;
    wr32(poll_out + o, nlq);   memcpy(poll_out + o + 4u, lq_out, PL_SIZE * nlq);   o += 4u + PL_SIZE * nlq;
    wr32(poll_out + o, tb_n);  memcpy(poll_out + o + 4u, tb_out, tb_bytes);        o += 4u + tb_bytes;
    ev_n = 0; ev_lost = 0; tm_n = 0; tm_lost = 0; cf_n = 0; cf_lost = 0;
    pl_n = 0; pl_lost = 0; ch_n = 0; ch_lost = 0; lq_n = 0; lq_lost = 0;
    tb_n = 0; tb_bytes = 0; tb_lost = 0;
    st_lost_mask = 0;
    st_polls++;
    if (st_test_pad && o + st_test_pad <= MAX_PAYLOAD) {   /* build 34 test */
        memset(poll_out + o, 0, st_test_pad);
        o += st_test_pad;
    }
    return o;
}

/* STREAM_STOP: no more datagrams; feed what is in the ring, let the receiver finish */
static void stream_stop(void)
{
    u32 polls;
    if (!st_on) {
        return;
    }
    /* stop storing, but keep the UDP port bound (the callback only counts now): with no pcb,
     * lwIP would answer every datagram still coming from the CoreBoard with an ICMP "port
     * unreachable". UDP_STOP (or the next STREAM_START / UDP_START) closes it. */
    udp_mode = 0;
    {
        XTime t0, now;
        XTime_GetTime(&t0);
        for (polls = 0; stream_dma(1) || st_rd < st_wr; polls++) {
            if ((polls & 0xFFu) == 0u) {
                stream_drain();
                XTime_GetTime(&now);
                if (now - t0 > ST_STOP_COUNTS) {
                    xil_printf("STREAM_STOP: ring not fed to the end in 2 s (%d slots left)\r\n",
                               (int)(st_wr - st_rd));
                    dma_reset();
                    st_dma_len = 0;
                    break;
                }
            }
        }
    }
    for (polls = IDLE_TIMEOUT_POLLS; det_ok && (pd_rd(R_STATUS) & STATUS_BUSY) && polls > 0; polls--) {
        stream_drain();
    }
    stream_drain();
    pd_wr(R_CTRL, 0);
    ring_map(0);
    st_on = 0;
    st_norec = 0;
    tx_paced = 0;
    xil_printf("Stream stopped: %d datagrams, %d lost (zero-filled), %d late fixed, %d too late, "
               "%d ring full, %d resyncs, ring max %d of %d, %d DMA chunks, %d DMA errors\r\n",
               (int)st_ok, (int)st_lost, (int)st_late_fixed, (int)st_late_dropped, (int)st_ring_full,
               (int)st_resyncs, (int)st_ring_max, (int)ST_SLOTS, (int)st_chunks, (int)st_dma_errors);
}

static void udp_stop(void)
{
    if (st_on) {
        stream_stop();
        return;
    }
    if (udp_pcb_s != NULL) {
        udp_remove(udp_pcb_s);
        udp_pcb_s = NULL;
        ring_map(0);
        gem_counters();
        xil_printf("UDP stream receiver stopped: %d datagrams, %d lost, %d late, %d duplicate, "
                   "GEM: %d no-buffer, %d overrun; pool empty %d, skipped %d\r\n", (int)udp_ok, (int)udp_lost,
                   (int)udp_late, (int)udp_dup, (int)gem_reserr, (int)gem_overrun,
                   (int)(rx_alloc_fail() - udp_alloc0), (int)udp_skipped);
    }
}

static void udp_stats(u8 *o)
{
    XTime now;
    XTime_GetTime(&now);
    if (udp_pcb_s != NULL) {
        gem_counters();
    }
    wr32(o + 0,  udp_pcb_s != NULL);
    wr32(o + 4,  udp_mode);
    wr32(o + 8,  udp_port);
    wr32(o + 12, ms_since(udp_t_start, now));
    wr32(o + 16, udp_have_first ? ms_since(udp_t_first, udp_t_last) : 0u);
    wr32(o + 20, udp_ok);
    wr32(o + 24, udp_bad_size);
    wr32(o + 28, udp_bad_magic);
    wr32(o + 32, udp_epoch);
    wr32(o + 36, udp_epochs);
    wr32(o + 40, udp_lost);
    wr32(o + 44, udp_late);
    wr32(o + 48, udp_dup);
    wr32(o + 52, gem_rx_frames());
    wr32(o + 56, gem_reserr);
    wr32(o + 60, gem_overrun);
    wr32(o + 64, (u32)(udp_t_busy / (COUNTS_PER_SECOND / 1000u)));
    wr32(o + 68, udp_loops);
    memcpy(o + 72, &udp_first_pos, 8);
    memcpy(o + 80, &udp_last_pos, 8);
    wr32(o + 88, udp_skipped);
    wr32(o + 92, rx_alloc_fail() - udp_alloc0);
}

static void udp_poll(void)                      /* from transfer_data(): main-loop count, 1 s log */
{
    XTime now;
    if (udp_pcb_s == NULL) {
        return;
    }
    udp_loops++;
    XTime_GetTime(&now);
    if (udp_mode == 5u && st_on && now - udp_t_poll > st_loop_max) {
        st_loop_max = now - udp_t_poll;         /* build 30: longest main-loop pass while streaming */
    }
    udp_t_poll = now;
    tx_pace();                                  /* build 35 */
    if ((udp_loops & 0x3FFu) != 0u) {
        return;
    }
    if (udp_mode == 5u) {
        return;                                 /* build 30: no 11 ms UART line while streaming */
    }
    XTime_GetTime(&now);
    if (now - udp_t_print < COUNTS_PER_SECOND) {
        return;
    }
    if (udp_ok != udp_print_ok) {
        u32 dt = ms_since(udp_t_print, now);
        u32 rate = (u32)((u64)(udp_ok - udp_print_ok) * 1000u / (dt ? dt : 1u));
        gem_counters();
        xil_printf("  UDP %d datagrams/s (%d Mbit/s), lost %d (+%d), late %d, dup %d, skipped %d, pool empty %d, GEM no-buffer %d overrun %d\r\n",
                   (int)rate, (int)(rate * (ESR2_LEN + 42u) * 8u / 1000000u), (int)udp_lost,
                   (int)(udp_lost - udp_print_lost), (int)udp_late, (int)udp_dup, (int)udp_skipped,
                   (int)(rx_alloc_fail() - udp_alloc0),
                   (int)gem_reserr, (int)gem_overrun);
    }
    udp_print_ok = udp_ok;
    udp_print_lost = udp_lost;
    udp_t_print = now;
    XTime_GetTime(&udp_t_poll);                 /* the UART print itself takes ~11 ms */
}

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
                            (det_version >= PD_ID_V7 ? BOARD_FLAG_CHEST : 0u) |
                            (det_version >= PD_ID_V8 ? BOARD_FLAG_LQ : 0u) |
                            (det_version >= PD_ID_V9 ? BOARD_FLAG_TB : 0u) |
                            (det_version >= PD_ID_V91 ? BOARD_FLAG_PACK8 : 0u) |
                            (det_version >= PD_ID_V92 ? BOARD_FLAG_NOREC : 0u));
        reply(pcb, cmd, ST_OK, seq, tx_small, 16);
        return;

    case CMD_RESET:
        if (st_on) {
            stream_stop();
        }
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
        if (st_on) {
            reply_error(pcb, cmd, ST_BAD_CMD, seq, "streaming: STREAM_STOP first");
            return;
        }
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
        if (st_on) {
            reply_error(pcb, cmd, ST_BAD_CMD, seq, "streaming: STREAM_STOP first");
            return;
        }
        if (len < 8) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "RUN: payload must be u32 count, u32 flags");
            return;
        }
        count = rd32(pl);
        flags = rd32(pl + 4);
        if (flags & RUN_FLAG_PACK8) {
            /* build 24: the buffer words hold two int8 samples each; count is in samples */
            if (det_version < PD_ID_V91) {
                reply_error(pcb, cmd, ST_BAD_CMD, seq, "RUN PACK8 needs bitstream 9.1 (pd_unpack8)");
                return;
            }
            if (flags & RUN_FLAG_PL_LOOPBACK) {
                reply_error(pcb, cmd, ST_BAD_ARG, seq, "RUN PACK8: not with the PL loopback");
                return;
            }
            if ((flags & RUN_FLAG_DEBUG) && rd32(params + 28) != 2u) {
                reply_error(pcb, cmd, ST_BAD_ARG, seq, "RUN PACK8 with debug records needs input_stage 2");
                return;
            }
            if (count == 0) {
                count = 2u * samples_stored;
            }
            if (count & 1u) {
                reply_error(pcb, cmd, ST_BAD_ARG, seq, "RUN PACK8: count must be even (two samples per word)");
                return;
            }
            if (count / 2u > samples_stored) {
                snprintf(msg, sizeof(msg), "run %lu packed samples, buffer has %lu words",
                         (unsigned long)count, (unsigned long)samples_stored);
                reply_error(pcb, cmd, ST_BAD_ARG, seq, msg);
                return;
            }
        }
        if (count == 0) {
            count = samples_stored;
        }
        if (!(flags & RUN_FLAG_PACK8) && count > samples_stored) {
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
            if (detector_run(count, (flags & RUN_FLAG_DEBUG) != 0, (flags & RUN_FLAG_PACK8) != 0,
                             msg, sizeof(msg)) != 0) {
                reply_error(pcb, cmd, ST_INTERNAL, seq, msg);
                return;
            }
            if (flags & RUN_FLAG_DEBUG) {
                debug_count = rd32(params + 28) == 2u ? count / 2u : count;
            }
            reply(pcb, cmd, ST_OK, seq, ev_out,
                  32u + EV_SIZE * ev_n + TM_SIZE * tm_n + CF_SIZE * cf_n + PL_SIZE * (pl_n + ch_n + lq_n) + tb_bytes);
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

    case CMD_UDP_START: {
        /* build 17: u16 port (0 = 4000), u16 mode (0 count, 1 count + convert). Reply: u32 port */
        u16 port16 = 0, mode16 = 0;
        if (len >= 2u) memcpy(&port16, pl, 2);
        if (len >= 4u) memcpy(&mode16, pl + 2, 2);
        if (mode16 > 4u) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "UDP_START: mode must be 0 (count), 1..3 (convert) or 4 (raw copy)");
            return;
        }
        if (udp_start(port16 ? port16 : UDP_DEFAULT_PORT, mode16) != 0) {
            reply_error(pcb, cmd, ST_INTERNAL, seq, "UDP_START: cannot open the UDP port");
            return;
        }
        wr32(tx_small, udp_port);
        reply(pcb, cmd, ST_OK, seq, tx_small, 4);
        return;
    }

    case CMD_UDP_STATS:
        udp_stats(tx_small);
        reply(pcb, cmd, ST_OK, seq, tx_small, UDP_STATS_SIZE);
        return;

    case CMD_STREAM_START: {
        /* build 25: u16 udp_port (0 = 4000), u16 flags (ST_FLAG_KEEP_PL), u32 CoreBoard IPv4
         * (bytes a.b.c.d in order; 0 = the PC starts the stream), u16 HTTP port, u16 JSON length,
         * then the JSON body of the board's "start" (the PC fills in the port). The receiver uses
         * the parameters of the last SET_PARAMS (input_stage 1 or 2). Reply: u32 udp port. */
        u16 port16 = 0, fl16 = 0, hp = 80, jl = 0;
        int r;
        if (len >= 2u) memcpy(&port16, pl, 2);
        if (len >= 4u) memcpy(&fl16, pl + 2, 2);
        cb_ip = len >= 8u ? rd32(pl + 4) : 0u;
        if (len >= 10u) memcpy(&hp, pl + 8, 2);
        if (len >= 12u) memcpy(&jl, pl + 10, 2);
        if (len < 12u + jl) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "STREAM_START: payload shorter than its JSON length");
            return;
        }
        if (!det_ok || det_version < PD_ID_V91 || !dma_ok) {
            reply_error(pcb, cmd, ST_BAD_CMD, seq, "STREAM_START needs the detector with PACK8 (bitstream 9.1) and the DMA");
            return;
        }
        if (rd32(params + 28) != 1u && rd32(params + 28) != 2u) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "STREAM_START: SET_PARAMS input_stage must be 1 (20 Msps) or 2 (40 Msps)");
            return;
        }
        if (st_on) {
            stream_stop();
        }
        cb_port = hp ? hp : 80u;
        det_cfg = 1;
        detector_reset();                       /* parameters of the last SET_PARAMS */
        st_reset(fl16);
        if (udp_start(port16 ? port16 : UDP_DEFAULT_PORT, 5u) != 0) {
            reply_error(pcb, cmd, ST_INTERNAL, seq, "STREAM_START: cannot open the UDP port");
            return;
        }
        st_busy0 = udp_t_busy;                  /* build 32: udp_start clears it (st_reset is before) */
        st_norec = det_version >= PD_ID_V92 && !(fl16 & ST_FLAG_KEEP_PL);
        tx_paced = !(fl16 & ST_FLAG_NOPACE);
        pd_wr(R_CTRL, CTRL_PACK8 | (st_norec ? CTRL_NOREC : 0u));
        st_on = 1;
        r = jl ? http_post((const char *)(pl + 12), jl) : 0;
        if (r != 0) {
            snprintf(msg, sizeof(msg), "STREAM_START: HTTP request to the CoreBoard failed (%d)", r);
            stream_stop();
            reply_error(pcb, cmd, ST_INTERNAL, seq, msg);
            return;
        }
        xil_printf("Streaming: UDP port %d, PACK8%s, input_stage %d, dc_shift %d%s\r\n", (int)udp_port,
                   st_norec ? " NOREC" : "", (int)rd32(params + 28), (int)rd32(params + 32),
                   jl ? ", CoreBoard started by HTTP" : "");
        wr32(tx_small, udp_port);
        reply(pcb, cmd, ST_OK, seq, tx_small, 4);
        return;
    }

    case CMD_POLL_RESULTS: {
        /* build 25: stats (ST_STATS_SIZE bytes, pd_protocol.StreamStats) + the records since the
         * last poll in the RUN reply format (pd_protocol.parse_run_reply_tb) */
        u32 n;
        XTime t0, t1;
        XTime_GetTime(&t0);
        if (st_on) {
            stream_drain();
        }
        n = stream_pack();
        reply(pcb, cmd, ST_OK, seq, poll_out, n);
        XTime_GetTime(&t1);                     /* build 30: the poll's CPU time (reply queued) */
        st_poll_t += t1 - t0;
        if (t1 - t0 > st_poll_max) {
            st_poll_max = t1 - t0;
        }
        return;
    }

    case CMD_STREAM_STOP: {
        /* build 25: u16 JSON length + JSON of the board's "stop" (0 = none); the ring is fed to
         * the end, then the reply is as POLL_RESULTS (the last records) */
        u16 jl = 0;
        u32 n;
        if (len >= 2u) memcpy(&jl, pl, 2);
        if (jl && len >= 2u + jl) {
            http_post((const char *)(pl + 2), jl);
        }
        stream_stop();
        n = stream_pack();
        reply(pcb, cmd, ST_OK, seq, poll_out, n);
        return;
    }

    case CMD_GEM_STATS: {
        u32 i;
        gem_counters();
        for (i = 0; i < GEM_STAT_N; i++) {
            wr32(tx_small + 4u * i, gem_acc[i]);
        }
        reply(pcb, cmd, ST_OK, seq, tx_small, GEM_STAT_N * 4u);
        return;
    }

    case CMD_UDP_STOP:
        udp_stop();
        udp_stats(tx_small);
        reply(pcb, cmd, ST_OK, seq, tx_small, UDP_STATS_SIZE);
        return;

    case CMD_REG_READ:
    case CMD_REG_WRITE: {
        u32 a;
        if (len < (cmd == CMD_REG_READ ? 4u : 8u)) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "REG_READ: u32 addr; REG_WRITE: u32 addr, u32 value");
            return;
        }
        a = rd32(pl);
        /* build 28: also the DDR controller's port priorities (UG585 axi_priority_wr_port0..3 at
         * 0xF8006208..0xF8006214, axi_priority_rd_port0..3 at 0xF8006218..0xF8006224) */
        /* build 33: also the Ethernet controller's control registers (GEM0 0xE000B000..0xE000B0FC;
         * not its statistics from 0x100, which clear when read) */
        if ((a & 3u) || !((a >= 0x40000000u && a <= 0x7FFFFFFCu) || (a >= 0xF8006208u && a <= 0xF8006224u) ||
                          (a >= 0xE000B000u && a <= 0xE000B0FCu))) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "register address must be 4-byte aligned in 0x40000000..0x7FFFFFFF, "
                        "0xF8006208..0xF8006224 or 0xE000B000..0xE000B0FC");
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
    if (!tx_paced) {                    /* build 35: paced sends come from tx_pace() */
        tx_pump(pcb);
    }
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


/* Build 20: report CPU exceptions on the UART instead of the BSP's silent endless loop.
 * DataAbortAddr / PrefetchAbortAddr / UndefinedExceptionAddr are set by the BSP's vectors. */
extern u32 DataAbortAddr, PrefetchAbortAddr, UndefinedExceptionAddr;

static void report_exception(void *what)
{
    u32 dfsr, dfar, ifsr, ifar;
    __asm__ volatile("mrc p15, 0, %0, c5, c0, 0" : "=r"(dfsr));
    __asm__ volatile("mrc p15, 0, %0, c6, c0, 0" : "=r"(dfar));
    __asm__ volatile("mrc p15, 0, %0, c5, c0, 1" : "=r"(ifsr));
    __asm__ volatile("mrc p15, 0, %0, c6, c0, 2" : "=r"(ifar));
    xil_printf("\r\n*** CPU EXCEPTION: %s ***\r\n", (const char *)what);
    xil_printf("    data abort at PC 0x%08x: DFSR 0x%08x DFAR 0x%08x\r\n",
               (unsigned int)DataAbortAddr, (unsigned int)dfsr, (unsigned int)dfar);
    xil_printf("    prefetch abort at 0x%08x: IFSR 0x%08x IFAR 0x%08x; undefined instruction at 0x%08x\r\n",
               (unsigned int)PrefetchAbortAddr, (unsigned int)ifsr, (unsigned int)ifar,
               (unsigned int)UndefinedExceptionAddr);
    xil_printf("    (arm-none-eabi-addr2line -e pd_server.elf <PC> gives the source line)\r\n");
    for (;;) {
    }
}

static void exceptions_init(void)
{
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_DATA_ABORT_INT, report_exception, (void *)"data abort");
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_PREFETCH_ABORT_INT, report_exception, (void *)"prefetch abort");
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_UNDEFINED_INT, report_exception, (void *)"undefined instruction");
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
    xil_printf("CoreBoard UDP stream test (UDP_START / UDP_STATS / UDP_STOP), default UDP port %d\r\n",
               (int)UDP_DEFAULT_PORT);
}

int start_application(void)
{
    struct tcp_pcb *pcb;
    err_t err;

    exceptions_init();
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
    udp_poll();
    stream_poll();
    return 0;
}
