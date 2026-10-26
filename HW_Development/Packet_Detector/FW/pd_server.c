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
 * Phase 1 (this version): the PL holds an AXIS FIFO, so RUN only supports the PL-loopback
 * flag: samples go DDR -> AXI DMA MM2S -> PL -> AXI DMA S2MM -> debug buffer, and the PC
 * reads them back with GET_DEBUG. RUN without the flag answers "no detector".
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

#include "lwip/err.h"
#include "lwip/tcp.h"

/* ---------------------------------------------------------------------------------- */
/* Configuration                                                                       */
/* ---------------------------------------------------------------------------------- */
#define PD_PORT                 5001
#define PD_PROTOCOL_VERSION     1u
#define PD_BUILD                2u              /* bump when the firmware changes */

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

/* ---------------------------------------------------------------------------------- */
/* State                                                                               */
/* ---------------------------------------------------------------------------------- */
static XAxiDma dma;
static int dma_ok;

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

/* ---------------------------------------------------------------------------------- */
/* Helpers                                                                             */
/* ---------------------------------------------------------------------------------- */
static u32 rd32(const u8 *p) { u32 v; memcpy(&v, p, 4); return v; }       /* ARM is LE */
static void wr32(u8 *p, u32 v) { memcpy(p, &v, 4); }
static void wr16(u8 *p, u16 v) { memcpy(p, &v, 2); }

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

/*
 * Stream `bytes` from src through the PL into dst, in chunks. Each MM2S transfer ends with
 * TLAST, which ends the matching S2MM transfer. Returns 0 or a negative error code.
 */
static int dma_stream(UINTPTR src, UINTPTR dst, u32 bytes, char *err, u32 err_len)
{
    u32 off;

    Xil_DCacheFlushRange(src, bytes);
    Xil_DCacheInvalidateRange(dst, bytes);
    for (off = 0; off < bytes; off += DMA_CHUNK_BYTES) {
        u32 n = bytes - off < DMA_CHUNK_BYTES ? bytes - off : DMA_CHUNK_BYTES;
        u32 polls = DMA_TIMEOUT_POLLS;
        u32 sr_rx, sr_tx, got;

        if (XAxiDma_SimpleTransfer(&dma, dst + off, n, XAXIDMA_DEVICE_TO_DMA) != XST_SUCCESS) {
            snprintf(err, err_len, "S2MM start failed at byte %lu", (unsigned long)off);
            dma_reset();
            return -1;
        }
        if (XAxiDma_SimpleTransfer(&dma, src + off, n, XAXIDMA_DMA_TO_DEVICE) != XST_SUCCESS) {
            snprintf(err, err_len, "MM2S start failed at byte %lu", (unsigned long)off);
            dma_reset();
            return -2;
        }
        while (XAxiDma_Busy(&dma, XAXIDMA_DMA_TO_DEVICE) || XAxiDma_Busy(&dma, XAXIDMA_DEVICE_TO_DMA)) {
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
        sr_rx = XAxiDma_ReadReg(dma.RegBase + XAXIDMA_RX_OFFSET, XAXIDMA_SR_OFFSET);
        if ((sr_tx | sr_rx) & XAXIDMA_ERR_ALL_MASK) {
            snprintf(err, err_len, "DMA error (MM2S SR 0x%08lx, S2MM SR 0x%08lx)",
                     (unsigned long)sr_tx, (unsigned long)sr_rx);
            dma_reset();
            return -4;
        }
        got = XAxiDma_ReadReg(dma.RegBase + XAXIDMA_RX_OFFSET, XAXIDMA_BUFFLEN_OFFSET);
        if (got != n) {
            snprintf(err, err_len, "S2MM received %lu bytes, expected %lu",
                     (unsigned long)got, (unsigned long)n);
            return -5;
        }
    }
    Xil_DCacheInvalidateRange(dst, bytes);
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
        wr32(tx_small + 12, BOARD_FLAG_NO_DETECTOR);
        reply(pcb, cmd, ST_OK, seq, tx_small, 16);
        return;

    case CMD_RESET:
        samples_stored = 0;
        debug_count = 0;
        reply(pcb, cmd, ST_OK, seq, NULL, 0);
        return;

    case CMD_SET_PARAMS:
        if (len < PARAMS_SIZE) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "SET_PARAMS payload too short");
            return;
        }
        memcpy(params, pl, PARAMS_SIZE);    /* phase 2: write to the detector registers */
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
        if (!(flags & RUN_FLAG_PL_LOOPBACK)) {
            reply_error(pcb, cmd, ST_BAD_CMD, seq,
                        "no detector in this bitstream (phase 1) - use RUN with the PL-loopback flag");
            return;
        }
        if (!dma_ok) {
            reply_error(pcb, cmd, ST_INTERNAL, seq, "DMA not initialised - see the UART log");
            return;
        }
        debug_count = 0;
        if (count > 0 && dma_stream(SAMPLE_BUF_ADDR, DEBUG_BUF_ADDR, 4u * count, msg, sizeof(msg)) != 0) {
            reply_error(pcb, cmd, ST_INTERNAL, seq, msg);
            return;
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

/* ---------------------------------------------------------------------------------- */
/* Entry points called by the template's main.c                                        */
/* ---------------------------------------------------------------------------------- */
void print_app_header(void)
{
    xil_printf("\r\n----- Flexlink packet-detector test bench (build %d) -----\r\n", (int)PD_BUILD);
    xil_printf("TCP port %d, sample buffer %d samples at 0x%08x\r\n",
               PD_PORT, (int)MAX_SAMPLES, (unsigned int)SAMPLE_BUF_ADDR);
    xil_printf("Phase 1: PL loopback only (no detector in the bitstream)\r\n");
}

int start_application(void)
{
    struct tcp_pcb *pcb;
    err_t err;

    dma_ok = (dma_init() == 0);
    if (!dma_ok) {
        xil_printf("WARNING: DMA unavailable - LOAD/READ work, RUN will fail\r\n");
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
