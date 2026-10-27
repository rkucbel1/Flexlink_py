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
 * The firmware converts the detector's integers to the event record: cfo_valid_cnt = t + N_CFO,
 * ToneCheck dB = 10 log10(num / den) (Q8.8), TONE_THRESH (Q8.8 dB) -> linear threshold.
 *
 * Note: xil_printf (used for the UART log) knows only %d %u %x %s %c; no %l formats.
 *       Error texts for the PC are formatted with newlib's snprintf, which has them.
 *
 * Protocol summary (little-endian; header 16 bytes):
 *     u32 magic 0x44504C46, u16 cmd, u16 status, u32 seq, u32 length, then payload
 *     PING 1, RESET 2, SET_PARAMS 3, LOAD_SAMPLES 4, READ_SAMPLES 5, RUN 6, GET_DEBUG 7
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

#include "lwip/err.h"
#include "lwip/tcp.h"

/* ---------------------------------------------------------------------------------- */
/* Configuration                                                                       */
/* ---------------------------------------------------------------------------------- */
#define PD_PORT                 5001
#define PD_PROTOCOL_VERSION     1u
#define PD_BUILD                3u              /* bump when the firmware changes */

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
#define PD_ID_VALUE             0x50440100u
#define R_ID                    0x00u
#define R_CTRL                  0x04u
#define R_STATUS                0x08u
#define R_SAMPLES               0x0Cu
#define R_OPEN_T                0x10u
#define R_EV_LEVEL              0x14u
#define R_EV_DATA               0x18u
#define R_DBG_SENT              0x1Cu
#define R_PARAM0                0x20u           /* ETA_HI_SQ ... TONE_LIN, 4 bytes apart */
#define CTRL_RESET              0x1u
#define CTRL_DEBUG              0x2u
#define CTRL_LOOPBACK           0x4u
#define STATUS_BUSY             0x1u
#define STATUS_OPEN             0x2u
#define STATUS_EV_OVF           0x100u
#define IDLE_TIMEOUT_POLLS      10000000u

/* ---------------------------------------------------------------------------------- */
/* Protocol constants (keep in step with PC/pd_protocol.py)                            */
/* ---------------------------------------------------------------------------------- */
#define PD_MAGIC                0x44504C46u
#define HDR_SIZE                16u
#define MAX_PAYLOAD             (1u << 20)
#define PARAMS_SIZE             36u

enum { CMD_PING = 1, CMD_RESET = 2, CMD_SET_PARAMS = 3, CMD_LOAD_SAMPLES = 4,
       CMD_READ_SAMPLES = 5, CMD_RUN = 6, CMD_GET_DEBUG = 7 };
enum { ST_OK = 0, ST_BAD_CMD = 1, ST_BAD_ARG = 2, ST_NO_SPACE = 3, ST_NO_DEBUG = 4,
       ST_INTERNAL = 5 };

#define RUN_FLAG_DEBUG          0x1u
#define RUN_FLAG_PL_LOOPBACK    0x2u
#define BOARD_FLAG_EMULATOR     0x1u
#define BOARD_FLAG_NO_DETECTOR  0x2u
#define EV_CFO_VALID            0x1u
#define EV_TONE_FAIL            0x2u
#define EV_TRUNCATED            0x4u
#define EV_SIZE                 24u
#define MAX_EVENTS              ((MAX_PAYLOAD - 8u) / EV_SIZE)
#define TONE_THRESH_MAX_Q8      (24u * 256u)

/* ---------------------------------------------------------------------------------- */
/* State                                                                               */
/* ---------------------------------------------------------------------------------- */
static XAxiDma dma;
static int dma_ok;
static int det_ok;                  /* detector found (ID register) */

static struct tcp_pcb *client;

static u8  rx_buf[HDR_SIZE + MAX_PAYLOAD] __attribute__((aligned(64)));
static u32 rx_have;                 /* bytes of the current message received */
static u32 rx_len;                  /* payload length from its header */

static u8        tx_hdr[HDR_SIZE];
static u8        tx_small[256];     /* short replies and error messages */
static const u8 *tx_data;
static u32       tx_total;          /* header + payload */
static u32       tx_off;

static u8  params[PARAMS_SIZE];
static u32 samples_stored;          /* samples in the sample buffer */
static u32 debug_count;             /* records in the debug buffer from the last RUN */
static u8  ev_out[8u + MAX_EVENTS * EV_SIZE];   /* RUN reply: n_events, count, records */
static u32 ev_n;                    /* events in ev_out */
static int ev_lost;                 /* more than MAX_EVENTS in one RUN */

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

/* Hold the detector in reset, write the parameters, release it. */
static void detector_reset(void)
{
    u32 i;
    if (!det_ok) {
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
    pd_wr(R_CTRL, 0);
}

/*
 * Stream `bytes` from src through the PL, in chunks; with dst != 0 the PL's output stream is
 * written to dst (each MM2S transfer ends with TLAST, which ends the matching S2MM transfer).
 * With events != 0 the detector's event FIFO is read while the DMA runs.
 * Returns 0 or a negative error code (message in err).
 */
static int dma_stream(UINTPTR src, UINTPTR dst, u32 bytes, int events, char *err, u32 err_len)
{
    u32 off;

    Xil_DCacheFlushRange(src, bytes);
    if (dst) {
        Xil_DCacheInvalidateRange(dst, bytes);
    }
    for (off = 0; off < bytes; off += DMA_CHUNK_BYTES) {
        u32 n = bytes - off < DMA_CHUNK_BYTES ? bytes - off : DMA_CHUNK_BYTES;
        u32 polls = DMA_TIMEOUT_POLLS;
        u32 sr_rx = 0, sr_tx, got;

        if (dst && XAxiDma_SimpleTransfer(&dma, dst + off, n, XAXIDMA_DEVICE_TO_DMA) != XST_SUCCESS) {
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
                drain_events();
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
            if (got != n) {
                snprintf(err, err_len, "S2MM received %lu bytes, expected %lu",
                         (unsigned long)got, (unsigned long)n);
                return -5;
            }
        }
    }
    if (dst) {
        Xil_DCacheInvalidateRange(dst, bytes);
    }
    return 0;
}

/*
 * Detector RUN of `count` samples from the sample buffer. Fills ev_out; returns 0 or an error
 * (message in err).
 */
static int detector_run(u32 count, int debug, char *err, u32 err_len)
{
    u32 start, polls, st, n_cfo = rd32(params + 12);

    ev_n = 0;
    ev_lost = 0;
    start = pd_rd(R_SAMPLES);
    pd_wr(R_CTRL, debug ? CTRL_DEBUG : 0u);
    if (count > 0 && dma_stream(SAMPLE_BUF_ADDR, debug ? DEBUG_BUF_ADDR : 0, 4u * count, 1, err, err_len) != 0) {
        pd_wr(R_CTRL, 0);
        return -1;
    }
    /* the last samples, CFO results and tone checks finish without further input */
    for (polls = IDLE_TIMEOUT_POLLS; (pd_rd(R_STATUS) & STATUS_BUSY) && polls > 0; polls--) {
        drain_events();
    }
    drain_events();
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
    if ((st & STATUS_OPEN) && pd_rd(R_OPEN_T) - start < count) {
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
        wr32(tx_small + 12, det_ok ? 0u : BOARD_FLAG_NO_DETECTOR);
        reply(pcb, cmd, ST_OK, seq, tx_small, 16);
        return;

    case CMD_RESET:
        samples_stored = 0;
        debug_count = 0;
        detector_reset();
        reply(pcb, cmd, ST_OK, seq, NULL, 0);
        return;

    case CMD_SET_PARAMS:
        if (len < PARAMS_SIZE) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "SET_PARAMS payload too short");
            return;
        }
        if (rd32(pl + 20) > TONE_THRESH_MAX_Q8) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "tone_thresh must be 0..24 dB (Q8.8 0..6144)");
            return;
        }
        if (rd32(pl + 28) > 1u) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "input_stage must be 0 (LPF output) or 1 (decimated input)");
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
        memcpy(params, pl, PARAMS_SIZE);
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
            debug_count = 0;
            if (detector_run(count, (flags & RUN_FLAG_DEBUG) != 0, msg, sizeof(msg)) != 0) {
                reply_error(pcb, cmd, ST_INTERNAL, seq, msg);
                return;
            }
            if (flags & RUN_FLAG_DEBUG) {
                debug_count = count;
            }
            reply(pcb, cmd, ST_OK, seq, ev_out, 8u + EV_SIZE * ev_n);
            return;
        }
        debug_count = 0;
        if (det_ok) {
            pd_wr(R_CTRL, CTRL_LOOPBACK);
        }
        if (count > 0 && dma_stream(SAMPLE_BUF_ADDR, DEBUG_BUF_ADDR, 4u * count, 0, msg, sizeof(msg)) != 0) {
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
    while (off < total) {
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
    static const u32 def[9] = {8028u, 2621u, 128u, 2048u, 288u, 3328u, 131072u, 0u, 10u};
    u32 i;
    for (i = 0; i < 9u; i++) {
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
        det_ok = (id == PD_ID_VALUE);
        if (det_ok) {
            pr_default_params();
            detector_reset();
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
