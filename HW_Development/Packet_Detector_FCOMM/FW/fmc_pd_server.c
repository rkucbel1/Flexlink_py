/*
 * fmc_pd_server.c  (Flexlink_py/HW_Development/Packet_Detector_FCOMM/FW)
 *
 * The FlexLink receiver pd_0 fed by an AD-FMCOMMS3 (AD9361) on the Zedboard's FMC connector.
 * A copy of ../../Packet_Detector/FW/pd_server.c (CoreBoard build 39) with the CoreBoard parts
 * (UDP sample stream, HTTP client, GEM statistics, int8 / PACK8 streaming) replaced by the
 * FMCOMMS3: fmc_radio.c sets up the AD9361 and its axi_dmac writes the sample buffer as a ring.
 * Everything from the ring on is pd_server.c's code: the AXI DMA feed to pd_0, the result
 * FIFOs, POLL_RESULTS / STREAM_PUSH, the statistics block, RUN / LOAD_SAMPLES for test vectors.
 * Fixes to that code in pd_server.c have to be made here as well.
 *
 * Hardware: ../HW/build_hw_fmcomms3.tcl (step 4: pd_0, AXI DMA MM2S only, axi_ad9361, axi_dmac).
 * Memory: sample buffer 0x10000000, 128 MB (the ring while streaming; LOAD_SAMPLES / CAPTURE
 * otherwise). There is no debug buffer in use: this bitstream has no AXI DMA S2MM, so RUN with
 * the debug or PL-loopback flag is refused.
 *
 * Streaming (STREAM_START / POLL_RESULTS / STREAM_STOP / STREAM_PUSH, as pd_server.c):
 *   ring        FMC_SLOT_SAMPLES samples (12-bit I, Q in one u32, as pd_0 expects) per slot,
 *               axi_dmac fills one slot per transfer, two queued (fmc_ring_poll, main loop)
 *   feed        AXI DMA MM2S, ST_CHUNK slots per transfer, CTRL.NOREC, PACK8 off
 *   lost        an axi_dmac overflow (no transfer queued in time) loses samples: counted in
 *               the statistics' "lost" field; the receiver's sample count then runs behind
 *   input_stage must match the AD9361 rate: 2 at 40 Msps (half-band in the PL), 1 at 20 Msps
 *   STREAM_START's payload is pd_server.c's; only the flags are used (no UDP port, no HTTP).
 *   STREAM_PUSH's "newest slot" field is the ring write position in input samples.
 * New commands: RADIO_CONFIG 22, CAPTURE 23 (as fmc_server.c; not while streaming).
 * PING board flags: pd_server.c's | BOARD_FMCOMMS3 (0x800).
 *
 * Note: xil_printf (used for the UART log) knows only %d %u %x %s %c; no %l formats.
 *       Error texts for the PC are formatted with newlib's snprintf, which has them.
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

#include "fmc_radio.h"

/* ---------------------------------------------------------------------------------- */
/* Configuration                                                                       */
/* ---------------------------------------------------------------------------------- */
#define PD_PORT                 5001
#define PD_PROTOCOL_VERSION     1u
#define PD_BUILD                1u               /* FMCOMMS3 firmware build; bump when it changes */

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
       CMD_STREAM_START = 18, CMD_POLL_RESULTS = 19, CMD_STREAM_STOP = 20,
       CMD_STREAM_PUSH = 21 /* build 39: board -> PC only, results pushed while streaming */,
       CMD_RADIO_CONFIG = 22, CMD_CAPTURE = 23 /* FMCOMMS3 (fmc_server.c) */ };
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
#define BOARD_FLAG_FMCOMMS3     0x800u          /* samples from an AD-FMCOMMS3 (this firmware) */
#define RADIO_FLAG_REINIT       0x1u
#define RADIO_STATUS_SIZE       64u
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
/* (pd_server.c's CoreBoard UDP stream is not in this firmware)                         */
/* ---------------------------------------------------------------------------------- */
static u64 udp_t_busy;                          /* stays 0: kept for the statistics block */
static u32 ms_since(XTime t0, XTime t1) { return (u32)((t1 - t0) / (COUNTS_PER_SECOND / 1000u)); }
static void stream_poll(void);

/* ---------------------------------------------------------------------------------- */
/* Streaming receiver (build 25)                                                       */
/* ---------------------------------------------------------------------------------- */
/* FMCOMMS3: STREAM_START starts the AD9361's axi_dmac on the sample buffer as a ring of ST_SLOTS
 * slots of FMC_SLOT_SAMPLES samples (fmc_ring_*, fmc_radio.c). From the main loop (transfer_data)
 * the ring writer queues the next slots, the AXI DMA feeds completed slots to the receiver in
 * chunks of ST_CHUNK slots, without waiting for it; the result FIFOs are drained into the record
 * buffers, and POLL_RESULTS (or STREAM_PUSH) hands them to the PC. No CPU touches the samples.
 *
 *   lost samples     an axi_dmac overflow (the writer did not queue a slot in time): counted in
 *                    "lost"; the receiver's sample count then runs behind the stream
 *   ring full        (the receiver slower than the AD9361 for longer than the ring, 0.84 s at
 *                    40 Msps) the writer waits for the reader: overflows follow, counted */
#define FMC_SLOT_SAMPLES        32768u          /* 128 KB, 0.82 ms at 40 Msps */
#define ST_SLOT_BYTES           (4u * FMC_SLOT_SAMPLES)
#define ST_SLOTS                (BUF_BYTES / ST_SLOT_BYTES)   /* 1024 */
#define ST_CHUNK                1u              /* slots per AXI DMA transfer to pd_0 (0.82 ms) */
#define ST_HOLD                 0u              /* (no late data: the writer is a DMA) */
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
/* build 39: bit1 PUSH - the board sends STREAM_PUSH messages (no POLL_RESULTS needed);
 * bits 6:2 = P, at most one push per 250 * P us while there are new records (0: 1 ms).
 * (Builds 34-38 used bits 6:1 for a padding test.) */
#define ST_FLAG_PUSH            0x2u
#define ST_FLAG_PUSH_SHIFT      2u
#define ST_PUSH_DEFAULT_US      1000u
#define ST_PUSH_STATS_COUNTS    (COUNTS_PER_SECOND / 10u)    /* statistics every 100 ms */
#define PUSH_HDR_SIZE           16u     /* u32 flags (bit0 statistics follow), u32 0, u64 newest slot */
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
static int   st_push;                        /* build 39: push results */
static u64   st_push_counts, st_t_push, st_t_stats;
static u32   st_pushes;
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

static struct fmc_ring_stat st_ring;            /* FMCOMMS3 ring writer statistics */

/* --- ring --- */
static u8 *st_slot(u64 k) { return (u8 *)(UINTPTR)SAMPLE_BUF_ADDR + (u32)(k % ST_SLOTS) * ST_SLOT_BYTES; }

static void st_reset(u32 flags)
{
    st_flags = flags;
    {
        u32 pu = 25u * ((flags >> ST_FLAG_PACE_SHIFT) & 0xFFu);
        tx_pace_counts = (u64)(pu ? pu : TX_PACE_DEFAULT_US) * (COUNTS_PER_SECOND / 1000000u);
    }
    st_push = (flags & ST_FLAG_PUSH) != 0u;
    {
        u32 pu = 250u * ((flags >> ST_FLAG_PUSH_SHIFT) & 0x1Fu);
        st_push_counts = (u64)(pu ? pu : ST_PUSH_DEFAULT_US) * (COUNTS_PER_SECOND / 1000000u);
    }
    st_t_push = st_t_stats = 0;
    st_pushes = 0;
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
    memset(&st_ring, 0, sizeof st_ring);
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

static void stream_push(void);

static void stream_poll(void)                   /* from transfer_data() */
{
    XTime now;
    if (!st_on) {
        return;
    }
    if (fmc_ring_poll(st_rd, &st_ring)) {       /* FMCOMMS3: slots written by axi_dmac */
        st_wr = st_ring.written;
        st_ok = (u32)st_wr;
        XTime_GetTime(&st_t_last);
        if (st_wr - st_rd > st_ring_max) {
            st_ring_max = (u32)(st_wr - st_rd);
        }
    }
    st_lost = st_ring.overflows;
    st_ring_full = st_ring.ring_full;
    stream_dma(0);
    tx_pace();                                  /* build 37: also during long receive bursts */
    XTime_GetTime(&now);
    if (now - st_t_drain > (st_norec ? ST_DRAIN_NOREC_COUNTS : ST_DRAIN_COUNTS)) {
        st_t_drain = now;
        stream_drain();
    }
    stream_push();                              /* build 39 */
}

static void stream_stats(u8 *o)
{
    XTime now;
    u32 st = det_ok ? pd_rd(R_STATUS) : 0u;
    XTime_GetTime(&now);
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
    wr32(o + 56, st_ring.timeouts);           /* FMCOMMS3: writer transfers not finished at stop */
    wr32(o + 60, 0u);
    wr32(o + 64, 0u);
    wr32(o + 68, st_lost_mask);
    wr32(o + 72, st_epoch);
    wr32(o + 76, det_ok ? pd_rd(R_SAMPLES) : 0u);
    {
        u64 fed = st_rd * FMC_SLOT_SAMPLES;    /* input samples fed to the receiver */
        memcpy(o + 80, &fed, 8);
    }
    wr32(o + 88, 0u);                          /* (CoreBoard HTTP state / code / error, bad datagrams) */
    wr32(o + 92, 0u);
    wr32(o + 96, 0u);
    wr32(o + 100, 0u);
    wr32(o + 104, st_polls);
    wr32(o + 108, 0u);
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
/* The records since the last call into poll_out from base; with stats, the statistics block
 * first (POLL_RESULTS / STREAM_STOP: base 0, stats 1; build 39 STREAM_PUSH: after its header,
 * statistics every 100 ms only - overflow flags are kept until they are reported) */
static u32 stream_pack_at(u32 base, int stats)
{
    u32 o, fixed, n_ev, keep = (st_flags & ST_FLAG_KEEP_PL) != 0u;
    u32 npl = keep ? pl_n : 0u, nch = keep ? ch_n : 0u, nlq = keep ? lq_n : 0u;
    o = base;
    if (stats) {
        stream_stats(poll_out + base);
        o += ST_STATS_SIZE;
    } else {
        st_lost_mask |= (ev_lost ? ST_LOST_EV : 0u) | (tm_lost ? ST_LOST_TM : 0u) | (cf_lost ? ST_LOST_CF : 0u) |
                        (pl_lost ? ST_LOST_PL : 0u) | (ch_lost ? ST_LOST_CH : 0u) | (lq_lost ? ST_LOST_LQ : 0u) |
                        (tb_lost ? ST_LOST_TB : 0u);
    }
    fixed = o + 32u + TM_SIZE * tm_n + CF_SIZE * cf_n + PL_SIZE * (npl + nch + nlq) + tb_bytes;
    n_ev = ev_n;
    if (fixed + EV_SIZE * n_ev > MAX_PAYLOAD) {  /* cannot happen with the buffer limits; be safe */
        n_ev = (MAX_PAYLOAD - fixed) / EV_SIZE;
        st_lost_mask |= ST_LOST_EV;
        if (stats) {
            wr32(poll_out + base + 68, st_lost_mask);
        }
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
    if (stats) {
        st_lost_mask = 0;
    }
    st_polls++;
    return o;
}

static u32 stream_pack(void)
{
    return stream_pack_at(0u, 1);
}

/* build 39: push mode - from stream_poll(), only with nothing being sent (a command that comes
 * meanwhile waits in lwIP, see recv_cb) */
static void stream_push(void)
{
    XTime now;
    u32 n, keep;
    int stats, have;
    if (!st_push || !st_on || client == NULL || tx_off < tx_total) {
        return;
    }
    XTime_GetTime(&now);
    keep = (st_flags & ST_FLAG_KEEP_PL) != 0u;
    have = ev_n || tm_n || cf_n || tb_n || (keep && (pl_n || ch_n || lq_n));
    stats = now - st_t_stats >= ST_PUSH_STATS_COUNTS;
    if (!stats && !(have && now - st_t_push >= st_push_counts)) {
        return;
    }
    st_t_push = now;
    if (stats) {
        st_t_stats = now;
    }
    wr32(poll_out + 0, stats ? 1u : 0u);
    wr32(poll_out + 4, 0u);
    {                                           /* "now" in the stream: input samples written */
        u64 now_s = st_wr * FMC_SLOT_SAMPLES;
        wr32(poll_out + 8, (u32)now_s);
        wr32(poll_out + 12, (u32)(now_s >> 32));
    }
    n = stream_pack_at(PUSH_HDR_SIZE, stats);
    st_pushes++;
    reply(client, CMD_STREAM_PUSH, ST_OK, st_pushes, poll_out, n);
}

/* STREAM_STOP: no more datagrams; feed what is in the ring, let the receiver finish */
static void stream_stop(void)
{
    u32 polls;
    if (!st_on) {
        return;
    }
    st_push = 0;                                /* build 39: the STREAM_STOP reply carries the rest */
    fmc_ring_stop(&st_ring);                    /* FMCOMMS3: no more slots written */
    st_wr = st_ring.written;
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
    st_on = 0;
    st_norec = 0;
    tx_paced = 0;
    xil_printf("Stream stopped: %d slots of %d samples, %d overflows (samples lost), %d ring full, "
               "ring max %d of %d, %d DMA chunks, %d DMA errors\r\n",
               (int)st_ok, (int)FMC_SLOT_SAMPLES, (int)st_lost, (int)st_ring_full,
               (int)st_ring_max, (int)ST_SLOTS, (int)st_chunks, (int)st_dma_errors);
}

/* Radio status, 64 bytes (PC/fmc_protocol.py RadioStatus), as fmc_server.c */
static void put_radio_status(u8 *o)
{
    struct fmc_radio_status rs;
    fmc_radio_get_status(&rs);
    memset(o, 0, RADIO_STATUS_SIZE);
    wr32(o + 0, rs.ok);
    wr32(o + 4, (u32)rs.last_error);
    wr32(o + 8, (u32)rs.cfg.lo_hz);
    wr32(o + 12, (u32)(rs.cfg.lo_hz >> 32));
    wr32(o + 16, rs.cfg.rate_hz);
    wr32(o + 20, rs.cfg.bw_hz);
    wr32(o + 24, rs.cfg.gain_mode);
    wr32(o + 28, (u32)rs.cfg.gain_db);
    wr32(o + 32, rs.cfg.port);
    wr32(o + 36, rs.ensm_mode);
    wr32(o + 40, rs.clk_freq_raw);
    wr32(o + 44, rs.adc_status);
    wr32(o + 48, rs.dma_status);
    wr32(o + 52, rs.chan_status_i);
    wr32(o + 56, rs.chan_status_q);
}

/* Defaults until the PC sends RADIO_CONFIG (fmc_server.c's) */
static const struct fmc_radio_cfg radio_default = {
    2420000000ull, 40000000u, 18000000u, FMC_GAIN_MANUAL, 30, 0u,
};

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
        wr32(tx_small + 12, (!det_ok ? BOARD_FLAG_NO_DETECTOR :
                            (det_version >= PD_ID_V3 ? BOARD_FLAG_TIMING : 0u) |
                            (det_version >= PD_ID_V4 ? BOARD_FLAG_CF : 0u) |
                            (det_version >= PD_ID_V5 ? BOARD_FLAG_REARM : 0u) |
                            (det_version >= PD_ID_V6 ? BOARD_FLAG_PAYLOAD : 0u) |
                            (det_version >= PD_ID_V7 ? BOARD_FLAG_CHEST : 0u) |
                            (det_version >= PD_ID_V8 ? BOARD_FLAG_LQ : 0u) |
                            (det_version >= PD_ID_V9 ? BOARD_FLAG_TB : 0u) |
                            (det_version >= PD_ID_V91 ? BOARD_FLAG_PACK8 : 0u) |
                            (det_version >= PD_ID_V92 ? BOARD_FLAG_NOREC : 0u)) | BOARD_FLAG_FMCOMMS3);
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
        if (flags & (RUN_FLAG_DEBUG | RUN_FLAG_PL_LOOPBACK)) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "RUN: debug records and PL loopback need the AXI DMA S2MM, "
                        "which the FMCOMMS3 bitstream does not have (use the CoreBoard build)");
            return;
        }
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

    case CMD_STREAM_START: {
        /* pd_server.c's payload (u16 udp_port, u16 flags, u32 CoreBoard IPv4, u16 HTTP port,
         * u16 JSON length, JSON); here only the flags count (ST_FLAG_*: KEEP_PL, PUSH and its
         * interval, pacing). The receiver uses the parameters of the last SET_PARAMS; its
         * input_stage must match the AD9361 rate. Reply: u32 0 (no UDP port). */
        u16 fl16 = 0;
        struct fmc_radio_status rs;
        u32 stage = rd32(params + 28);
        if (len >= 4u) memcpy(&fl16, pl + 2, 2);
        if (!det_ok || det_version < PD_ID_V92 || !dma_ok) {
            reply_error(pcb, cmd, ST_BAD_CMD, seq, "STREAM_START needs the receiver with NOREC (bitstream 9.2) and the AXI DMA");
            return;
        }
        fmc_radio_get_status(&rs);
        if (!rs.ok) {
            reply_error(pcb, cmd, ST_BAD_CMD, seq, "STREAM_START: the AD9361 is not set up (RADIO_CONFIG)");
            return;
        }
        if (!((stage == 2u && rs.cfg.rate_hz == 40000000u) || (stage == 1u && rs.cfg.rate_hz == 20000000u))) {
            snprintf(msg, sizeof(msg), "STREAM_START: input_stage %lu does not fit the AD9361 rate %lu Hz "
                     "(stage 2: 40 Msps, stage 1: 20 Msps)", (unsigned long)stage, (unsigned long)rs.cfg.rate_hz);
            reply_error(pcb, cmd, ST_BAD_ARG, seq, msg);
            return;
        }
        if (st_on) {
            stream_stop();
        }
        det_cfg = 1;
        detector_reset();                       /* parameters of the last SET_PARAMS */
        st_reset(fl16);
        samples_stored = 0;                     /* the ring overwrites the sample buffer */
        st_norec = !(fl16 & ST_FLAG_KEEP_PL);
        tx_paced = !(fl16 & ST_FLAG_NOPACE);
        pd_wr(R_CTRL, st_norec ? CTRL_NOREC : 0u);
        msg[0] = '\0';
        if (fmc_ring_start(SAMPLE_BUF_ADDR, ST_SLOT_BYTES, ST_SLOTS, msg, sizeof(msg)) != 0) {
            pd_wr(R_CTRL, 0);
            reply_error(pcb, cmd, ST_INTERNAL, seq, msg[0] ? msg : "STREAM_START: ring writer failed");
            return;
        }
        st_on = 1;
        xil_printf("Streaming: FMCOMMS3 %d Msps, input_stage %d, dc_shift %d%s%s, ring %d x %d samples\r\n",
                   (int)(rs.cfg.rate_hz / 1000000u), (int)stage, (int)rd32(params + 32),
                   st_norec ? ", NOREC" : "", st_push ? ", push" : "", (int)ST_SLOTS, (int)FMC_SLOT_SAMPLES);
        wr32(tx_small, 0u);
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
        u32 n;                                  /* (the payload's CoreBoard "stop" JSON is ignored) */
        stream_stop();
        n = stream_pack();
        reply(pcb, cmd, ST_OK, seq, poll_out, n);
        return;
    }

    case CMD_RADIO_CONFIG: {
        /* empty: status only. Else u64 lo_hz, u32 rate_hz, u32 bw_hz, u32 gain_mode, s32 gain_db,
         * u32 port, u32 flags (bit 0 re-initialise). While streaming only LO, gain and port may
         * change (the rate and bandwidth change the AD9361's clocks). Reply: the radio status. */
        if (len >= 32u) {
            struct fmc_radio_cfg c;
            struct fmc_radio_status rs;
            u32 flags;
            int ret;
            c.lo_hz = (u64)rd32(pl) | ((u64)rd32(pl + 4) << 32);
            c.rate_hz = rd32(pl + 8);
            c.bw_hz = rd32(pl + 12);
            c.gain_mode = rd32(pl + 16);
            c.gain_db = (s32)rd32(pl + 20);
            c.port = rd32(pl + 24);
            flags = rd32(pl + 28);
            fmc_radio_get_status(&rs);
            if (st_on && ((flags & RADIO_FLAG_REINIT) || c.rate_hz != rs.cfg.rate_hz || c.bw_hz != rs.cfg.bw_hz)) {
                reply_error(pcb, cmd, ST_BAD_ARG, seq, "RADIO_CONFIG: rate, bandwidth or re-initialisation only when not streaming");
                return;
            }
            msg[0] = '\0';
            ret = (flags & RADIO_FLAG_REINIT) ? fmc_radio_init(&c, msg, sizeof(msg))
                                              : fmc_radio_config(&c, msg, sizeof(msg));
            if (ret < 0) {
                reply_error(pcb, cmd, ST_BAD_ARG, seq, msg[0] ? msg : "RADIO_CONFIG failed");
                return;
            }
        } else if (len != 0u) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "RADIO_CONFIG: empty (status) or 32 bytes");
            return;
        }
        put_radio_status(tx_small);
        reply(pcb, cmd, ST_OK, seq, tx_small, RADIO_STATUS_SIZE);
        return;
    }

    case CMD_CAPTURE: {
        /* u32 n_samples, u32 timeout_ms: AD9361 samples into the sample buffer (not while
         * streaming); READ_SAMPLES reads them. Reply: u32 n, u32 flags (bit 0 overflow),
         * u32 time_us, u32 dma_status, radio status (64 bytes) */
        struct fmc_capture_result r;
        u32 n, tmo;
        int ret;
        if (len < 8u) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "CAPTURE: u32 n_samples, u32 timeout_ms");
            return;
        }
        n = rd32(pl);
        tmo = rd32(pl + 4);
        if (n == 0u || n > MAX_SAMPLES) {
            snprintf(msg, sizeof(msg), "CAPTURE: 1 .. %lu samples", (unsigned long)MAX_SAMPLES);
            reply_error(pcb, cmd, ST_BAD_ARG, seq, msg);
            return;
        }
        if (st_on) {
            reply_error(pcb, cmd, ST_BAD_CMD, seq, "CAPTURE: not while streaming (STREAM_STOP first)");
            return;
        }
        samples_stored = 0;
        msg[0] = '\0';
        ret = fmc_capture(SAMPLE_BUF_ADDR, n, tmo, &r, msg, sizeof(msg));
        if (ret < 0) {
            reply_error(pcb, cmd, ST_INTERNAL, seq, msg[0] ? msg : "CAPTURE failed");
            return;
        }
        samples_stored = r.n_samples;
        xil_printf("CAPTURE: %d samples in %d us%s\r\n", (int)r.n_samples, (int)r.time_us,
                   r.overflow ? ", OVERFLOW" : "");
        wr32(tx_small + 0, r.n_samples);
        wr32(tx_small + 4, r.overflow ? 1u : 0u);
        wr32(tx_small + 8, r.time_us);
        wr32(tx_small + 12, r.dma_status);
        put_radio_status(tx_small + 16);
        reply(pcb, cmd, ST_OK, seq, tx_small, 16u + RADIO_STATUS_SIZE);
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
    if (tx_off < tx_total) {
        /* build 39: a reply (or a push) is still being queued - lwIP keeps the data and offers
         * it again (next segment or within 250 ms), so a reply never overwrites another one */
        return ERR_MEM;
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
    xil_printf("\r\n----- Flexlink receiver, FMCOMMS3 (fmc_pd_server build %d) -----\r\n", (int)PD_BUILD);
    xil_printf("TCP port %d, sample buffer %d samples at 0x%08x\r\n",
               PD_PORT, (int)MAX_SAMPLES, (unsigned int)SAMPLE_BUF_ADDR);
    xil_printf("Phase 2: packet detector in the PL, registers at 0x%08x\r\n", (unsigned int)PD_BASE);
    xil_printf("AD9361: axi_ad9361 at 0x%08x, axi_dmac at 0x%08x; ring %d slots x %d samples\r\n",
               (unsigned int)FMC_AD9361_BASE, (unsigned int)FMC_DMA_BASE, (int)ST_SLOTS, (int)FMC_SLOT_SAMPLES);
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
    {
        char rmsg[200];
        rmsg[0] = '\0';
        if (fmc_radio_init(&radio_default, rmsg, sizeof(rmsg)) < 0) {
            xil_printf("WARNING: AD9361 set-up failed: %s\r\n", rmsg);
            xil_printf("         (RADIO_CONFIG retries; check VADJ 2.5 V and the FMC seating)\r\n");
        }
    }
    xil_printf("FMCOMMS3 receiver server listening on port %d\r\n", PD_PORT);
    return 0;
}

int transfer_data(void)
{
    stream_poll();
    return 0;
}
