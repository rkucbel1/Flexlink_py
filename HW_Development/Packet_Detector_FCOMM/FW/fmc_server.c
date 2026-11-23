/*
 * fmc_server.c  (Flexlink_py/HW_Development/Packet_Detector_FCOMM/FW)
 *
 * Step 3 of the FMCOMMS3 receiver: the Zedboard as an AD9361 capture device. A TCP server on
 * port 5001 with the same message format as ../../Packet_Detector/FW/pd_server.c (magic, header,
 * PING, READ_SAMPLES, REG_READ / REG_WRITE unchanged), so PC/pd_protocol.py talks to it, plus:
 *
 *     RADIO_CONFIG 22   payload empty: status only. Else u64 lo_hz, u32 rate_hz, u32 bw_hz,
 *                       u32 gain_mode, s32 gain_db, u32 port, u32 flags (32 bytes); bit 0 of
 *                       flags = full re-initialisation of the AD9361.
 *                       Reply: radio status (64 bytes, see put_status / PC/fmc_protocol.py)
 *     CAPTURE 23        u32 n_samples, u32 timeout_ms: the AD9361 samples into the sample buffer
 *                       (0x10000000, up to 32M samples), then READ_SAMPLES reads them.
 *                       Reply: u32 n_samples, u32 flags (bit 0 overflow), u32 time_us,
 *                       u32 dma_status, then the radio status (64 bytes)
 *
 * PING board flags: BOARD_NO_DETECTOR (0x2, no pd_0 in this bitstream) | BOARD_FMCOMMS3 (0x800).
 *
 * The template's main.c (network, DHCP with fallback 192.168.50.10, timers, main loop) calls
 * print_app_header(), start_application() and transfer_data(), as for pd_server.c.
 * xil_printf knows only %d %u %x %s %c; messages for the PC use newlib's snprintf.
 */
#include <stdio.h>
#include <string.h>

#include "xparameters.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "xil_exception.h"
#include "xil_types.h"
#include "xil_io.h"

#include "lwip/err.h"
#include "lwip/tcp.h"

#include "fmc_radio.h"

#define FMC_PORT                5001
#define PD_PROTOCOL_VERSION     1u
#define FMC_BUILD               3u              /* bump when the firmware changes */

#define SAMPLE_BUF_ADDR         0x10000000u
#define BUF_BYTES               0x08000000u     /* 128 MB */
#define MAX_SAMPLES             (BUF_BYTES / 4u)

#define PD_MAGIC                0x44504C46u
#define HDR_SIZE                16u
#define MAX_PAYLOAD             (1u << 20)

enum { CMD_PING = 1, CMD_READ_SAMPLES = 5, CMD_REG_READ = 9, CMD_REG_WRITE = 10,
       CMD_RADIO_CONFIG = 22, CMD_CAPTURE = 23 };
enum { ST_OK = 0, ST_BAD_CMD = 1, ST_BAD_ARG = 2, ST_NO_SPACE = 3, ST_NO_DEBUG = 4, ST_INTERNAL = 5 };

#define BOARD_FLAG_NO_DETECTOR  0x2u
#define BOARD_FLAG_FMCOMMS3     0x800u
#define RADIO_FLAG_REINIT       0x1u
#define STATUS_SIZE             64u

/* Defaults until the PC sends RADIO_CONFIG: the cable test of the handover */
static const struct fmc_radio_cfg radio_default = {
    2420000000ull,          /* LO */
    40000000u,              /* 40 Msps: input stage 2 of the receiver */
    18000000u,              /* RF bandwidth (signal 16.4 MHz) */
    FMC_GAIN_MANUAL,
    30,                     /* dB */
    0u,                     /* Rx1A */
};

static struct tcp_pcb *client;
static u8  rx_buf[HDR_SIZE + MAX_PAYLOAD] __attribute__((aligned(64)));
static u32 rx_have, rx_len;
static u8  tx_hdr[HDR_SIZE];
static u8  tx_small[256];
static const u8 *tx_data;
static u32 tx_total, tx_off;
static u32 samples_stored;

static u32 rd32(const u8 *p) { u32 v; memcpy(&v, p, 4); return v; }
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
/* Transmit (as pd_server.c, without the streaming pacing)                             */
/* ---------------------------------------------------------------------------------- */
static void tx_send(struct tcp_pcb *pcb)
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
            break;
        }
        if (e != ERR_OK) {
            xil_printf("tcp_write error %d\r\n", e);
            tx_total = tx_off;
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
    tx_send(pcb);
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

/* Radio status, 64 bytes (PC/fmc_protocol.py RadioStatus) */
static void put_status(u8 *o)
{
    struct fmc_radio_status st;
    fmc_radio_get_status(&st);
    memset(o, 0, STATUS_SIZE);
    wr32(o + 0, st.ok);
    wr32(o + 4, (u32)st.last_error);
    wr32(o + 8, (u32)st.cfg.lo_hz);
    wr32(o + 12, (u32)(st.cfg.lo_hz >> 32));
    wr32(o + 16, st.cfg.rate_hz);
    wr32(o + 20, st.cfg.bw_hz);
    wr32(o + 24, st.cfg.gain_mode);
    wr32(o + 28, (u32)st.cfg.gain_db);
    wr32(o + 32, st.cfg.port);
    wr32(o + 36, st.ensm_mode);
    wr32(o + 40, st.clk_freq_raw);
    wr32(o + 44, st.adc_status);
    wr32(o + 48, st.dma_status);
    wr32(o + 52, st.chan_status_i);
    wr32(o + 56, st.chan_status_q);
}

/* ---------------------------------------------------------------------------------- */
/* Commands                                                                            */
/* ---------------------------------------------------------------------------------- */
static void handle_message(struct tcp_pcb *pcb)
{
    u16 cmd;
    u32 seq, len;
    const u8 *pl = rx_buf + HDR_SIZE;
    char msg[200];

    memcpy(&cmd, rx_buf + 4, 2);
    seq = rd32(rx_buf + 8);
    len = rx_len;

    switch (cmd) {
    case CMD_PING:
        wr32(tx_small + 0, PD_PROTOCOL_VERSION);
        wr32(tx_small + 4, FMC_BUILD);
        wr32(tx_small + 8, MAX_SAMPLES);
        wr32(tx_small + 12, BOARD_FLAG_NO_DETECTOR | BOARD_FLAG_FMCOMMS3);
        reply(pcb, cmd, ST_OK, seq, tx_small, 16);
        return;

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

    case CMD_REG_READ:
    case CMD_REG_WRITE: {
        u32 a;
        if (len < (cmd == CMD_REG_READ ? 4u : 8u)) {
            reply_error(pcb, cmd, ST_BAD_ARG, seq, "REG_READ: u32 addr; REG_WRITE: u32 addr, u32 value");
            return;
        }
        a = rd32(pl);
        if ((a & 3u) || a < 0x40000000u || a > 0x7FFFFFFCu) {
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

    case CMD_RADIO_CONFIG: {
        if (len >= 32u) {
            struct fmc_radio_cfg c;
            u32 flags;
            int ret;
            c.lo_hz = (u64)rd32(pl) | ((u64)rd32(pl + 4) << 32);
            c.rate_hz = rd32(pl + 8);
            c.bw_hz = rd32(pl + 12);
            c.gain_mode = rd32(pl + 16);
            c.gain_db = (s32)rd32(pl + 20);
            c.port = rd32(pl + 24);
            flags = rd32(pl + 28);
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
        put_status(tx_small);
        reply(pcb, cmd, ST_OK, seq, tx_small, STATUS_SIZE);
        return;
    }

    case CMD_CAPTURE: {
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
        put_status(tx_small + 16);
        reply(pcb, cmd, ST_OK, seq, tx_small, 16u + STATUS_SIZE);
        return;
    }

    default:
        snprintf(msg, sizeof(msg), "command %u not supported by the FMCOMMS3 capture firmware", (unsigned)cmd);
        reply_error(pcb, cmd, ST_BAD_CMD, seq, msg);
        return;
    }
}

/* ---------------------------------------------------------------------------------- */
/* TCP (as pd_server.c)                                                                */
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
    tx_send(pcb);
    return ERR_OK;
}

static void err_cb(void *arg, err_t err)
{
    (void)arg;
    xil_printf("Connection error %d\r\n", err);
    client = NULL;
    reset_connection_state();
}

static err_t recv_cb(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
    u32 total, off = 0;
    (void)arg;

    if (p == NULL) {
        xil_printf("Client disconnected\r\n");
        drop_client(pcb, 0);
        return ERR_OK;
    }
    if (err != ERR_OK) {
        pbuf_free(p);
        return err;
    }
    if (tx_off < tx_total) {
        return ERR_MEM;             /* a reply is still being queued; lwIP offers the data again */
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
    if (client != NULL) {
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

/* CPU exceptions on the UART (as pd_server.c build 20) */
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
    xil_printf("    (arm-none-eabi-addr2line -e fmc_server.elf <PC> gives the source line)\r\n");
    for (;;) {
    }
}

static void exceptions_init(void)
{
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_DATA_ABORT_INT, report_exception, (void *)"data abort");
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_PREFETCH_ABORT_INT, report_exception, (void *)"prefetch abort");
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_UNDEFINED_INT, report_exception, (void *)"undefined instruction");
}

/* ---------------------------------------------------------------------------------- */
/* Entry points called by the template's main.c                                        */
/* ---------------------------------------------------------------------------------- */
void print_app_header(void)
{
    xil_printf("\r\n----- Flexlink FMCOMMS3 capture (build %d) -----\r\n", (int)FMC_BUILD);
    xil_printf("TCP port %d, sample buffer %d samples at 0x%08x\r\n",
               FMC_PORT, (int)MAX_SAMPLES, (unsigned int)SAMPLE_BUF_ADDR);
    xil_printf("axi_ad9361 at 0x%08x, axi_dmac at 0x%08x\r\n",
               (unsigned int)FMC_AD9361_BASE, (unsigned int)FMC_DMA_BASE);
}

int start_application(void)
{
    struct tcp_pcb *pcb;
    err_t err;
    char msg[200];

    exceptions_init();
    msg[0] = '\0';
    if (fmc_radio_init(&radio_default, msg, sizeof(msg)) < 0) {
        xil_printf("WARNING: AD9361 set-up failed: %s\r\n", msg);
        xil_printf("         (RADIO_CONFIG retries; check VADJ 2.5 V and the FMC seating)\r\n");
    }

    pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (pcb == NULL) {
        xil_printf("Error creating PCB\r\n");
        return -1;
    }
    err = tcp_bind(pcb, IP_ANY_TYPE, FMC_PORT);
    if (err != ERR_OK) {
        xil_printf("Unable to bind to port %d: err = %d\r\n", FMC_PORT, err);
        return -2;
    }
    pcb = tcp_listen(pcb);
    if (pcb == NULL) {
        xil_printf("Out of memory while tcp_listen\r\n");
        return -3;
    }
    tcp_accept(pcb, accept_cb);
    xil_printf("FMCOMMS3 capture server listening on port %d\r\n", FMC_PORT);
    return 0;
}

int transfer_data(void)
{
    return 0;
}
