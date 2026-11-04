/* ptx_server.c - Flexlink transmitter server on the ADALM-Pluto's ARM (Linux).
 *
 * The PC connects over the Pluto's USB network link (TCP port 5010, ip 192.168.2.1) and
 *   1. sends SET_PARAMS: the Defined sections of Tx/Parameters_Tx_payload.json and
 *      Tx/ControlSignals/Parameters_Control_Field.json (JSON). The server checks them, computes
 *      the Calculated values (ptx_params.c, the same as the PL computes from the header) and
 *      replies PARAMS: the Calculated sections and BlocksPerPacket (AvailableDataBlocks),
 *      TransportBlockSize, WordsPerBlock; then READY with credits (packets it can take).
 *   2. sends BLOCKS, one packet's transport blocks per message, one per credit. The server puts
 *      each in front of a 6-word parameter header and pushes the record into the TX DMA (local
 *      libiio, non-cyclic buffers); the PL (RTL/ptx_core.v) encodes and modulates it.
 *   3. gets SENT for every packet the PL has finished (its packet counter, DAC GPIO in, offset 0xB8 of the TX core)
 *      and a new READY credit with it. Two credits keep one packet on air and one waiting, so
 *      the transmission is continuous while the PC keeps up; when it does not, the PL sends
 *      nothing until the next packet arrives.
 *   The settings stay until the next SET_PARAMS (it waits until the packets already queued are
 *   sent). STOP ends the transmission and switches the Pluto back to its stock data path.
 * Optional "Radio" in SET_PARAMS: {"TxLo": Hz, "TxGain": dB (<= 0), "RfBandwidth": Hz}; the
 * sample rate is always set to 40 Msps.
 *
 * Messages (both directions): 16-byte header, little endian: u32 magic 0x53585450 ("PTXS"),
 * u16 type, u16 0, u32 seq, u32 payload length; then the payload. Types (PC/ptx_client.py):
 *   PC -> Pluto: 1 PING, 3 SET_PARAMS (JSON), 5 BLOCKS (u32 n_blocks, then the blocks: each
 *                TransportBlockSize bits from a new 64-bit word, LSB first), 8 STATUS, 10 STOP
 *   Pluto -> PC: 2 PONG, 4 PARAMS (JSON), 6 READY (JSON), 7 SENT (JSON), 9 STATUS (JSON),
 *                11 ERROR (JSON)
 *
 * Build: make -f ptx.mk (PC, with the PL simulated: -DPTX_STUB writes the records to a file), or
 *        make -f ptx.mk pluto FW=<plutosdr-fw> (buildroot toolchain and sysroot, libiio 0.2x).
 * Run on the Pluto: ./ptx_server [-p port] [-d depth]
 */
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "ptx_params.h"

#ifndef PTX_STUB
#include <iio.h>
#endif

#define PTX_MAGIC 0x53585450u
#define BUILD 1
enum { T_PING = 1, T_PONG, T_SET_PARAMS, T_PARAMS, T_BLOCKS, T_READY, T_SENT, T_STATUS, T_STATUS_R, T_STOP, T_ERROR };
/* DAC GPIO registers of axi_ad9361 (0x40BC / 0x40B8 in the core's full map). The Linux driver of
 * cf-ad9361-dds-core-lpc counts debug register offsets from the TX (DAC) part, which starts at 0x4000. */
#define DAC_GPIO_OUT 0x00BC
#define DAC_GPIO_IN  0x00B8
#define PTX_ENABLE   (1u << 4)
#define QMAX 16

typedef struct {
    uint32_t seq;
    size_t nwords;
    uint64_t *w;          /* header + blocks */
} record;

static struct {
    pthread_mutex_t mu;
    pthread_cond_t cv;
    int sock;                       /* client, -1 none */
    int depth;                      /* credits */
    int have_params;
    ptx_cfg cfg;
    ptx_derived d;
    record q[QMAX];                 /* waiting for the DMA */
    int qn;
    uint32_t fly[QMAX];             /* pushed, not yet reported sent */
    int nfly;
    uint32_t pushed, sent;
    uint32_t pl_count;              /* last PL packet counter (16 bit) */
    int stop_req, quit;
    size_t buf_samples;
#ifdef PTX_STUB
    FILE *stub_out;
    double stub_busy_until;
    double stub_done[QMAX];         /* simulated PL: end time of every pushed packet */
    int stub_n;
    uint32_t stub_count;
#else
    struct iio_context *ctx;
    struct iio_device *dds, *phy;
    struct iio_buffer *buf;
#endif
} S;

static double now_s(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + 1e-9 * t.tv_nsec;
}

/* ------------------------------------------------------------------------------------------ */
/* messages                                                                                    */

static int send_all(int fd, const void *p, size_t n)
{
    const char *c = p;
    while (n) {
        ssize_t k = send(fd, c, n, MSG_NOSIGNAL);
        if (k <= 0) { if (k < 0 && errno == EINTR) continue; return -1; }
        c += k; n -= (size_t)k;
    }
    return 0;
}

static int recv_all(int fd, void *p, size_t n)
{
    char *c = p;
    while (n) {
        ssize_t k = recv(fd, c, n, 0);
        if (k <= 0) { if (k < 0 && errno == EINTR) continue; return -1; }
        c += k; n -= (size_t)k;
    }
    return 0;
}

/* caller holds S.mu */
static void send_msg(int type, uint32_t seq, const void *payload, uint32_t len)
{
    uint8_t h[16];
    uint32_t v[4] = {PTX_MAGIC, (uint32_t)type, seq, len};
    if (S.sock < 0) return;
    memcpy(h, &v[0], 4);
    h[4] = (uint8_t)type; h[5] = (uint8_t)(type >> 8); h[6] = h[7] = 0;
    memcpy(h + 8, &v[2], 4);
    memcpy(h + 12, &v[3], 4);
    if (send_all(S.sock, h, 16) || (len && send_all(S.sock, payload, len))) {
        close(S.sock);
        S.sock = -1;
    }
}

static void send_json(int type, uint32_t seq, const char *fmt, ...)
{
    char b[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    send_msg(type, seq, b, (uint32_t)strlen(b));
}

static void send_error(uint32_t seq, const char *what)
{
    char b[512];
    size_t i, j = 0;
    for (i = 0; what[i] && j < sizeof b - 2; i++) if (what[i] != '"' && what[i] != '\\') b[j++] = what[i];
    b[j] = 0;
    send_json(T_ERROR, seq, "{\"error\": \"%s\"}", b);
}

static int credits(void) { int c = S.depth - S.qn - S.nfly; return c < 0 ? 0 : c; }

/* ------------------------------------------------------------------------------------------ */
/* hardware (or the stub)                                                                      */

static uint32_t hw_status(void)
{
#ifdef PTX_STUB
    double t = now_s();
    while (S.stub_n && t >= S.stub_done[0]) {      /* the oldest packet is through */
        memmove(S.stub_done, S.stub_done + 1, (size_t)(S.stub_n - 1) * sizeof S.stub_done[0]);
        S.stub_n--;
        S.stub_count = (S.stub_count + 1) & 0xFFFF;
    }
    return (1u << 29) | ((S.stub_n ? 1u : 0u) << 28) | S.stub_count;
#else
    uint32_t v = 0;
    iio_device_reg_read(S.dds, DAC_GPIO_IN, &v);
    return v;
#endif
}

static int hw_enable(int on)
{
#ifdef PTX_STUB
    (void)on;
    return 0;
#else
    uint32_t v = 0;
    if (iio_device_reg_read(S.dds, DAC_GPIO_OUT, &v)) return -1;
    v = on ? (v | PTX_ENABLE) : (v & ~PTX_ENABLE);
    return iio_device_reg_write(S.dds, DAC_GPIO_OUT, v);
#endif
}

static void hw_destroy_buffer(void)
{
#ifndef PTX_STUB
    if (S.buf) iio_buffer_destroy(S.buf);
    S.buf = NULL;
#endif
    S.buf_samples = 0;
}

/* push one record (blocks until the kernel has room); called without S.mu */
static int hw_push(const record *r)
{
    size_t samples = r->nwords * 2;                 /* one 64-bit word = two I/Q int16 pairs */
#ifdef PTX_STUB
    double t = now_s(), start, dur;
    fwrite(r->w, 8, r->nwords, S.stub_out);
    fflush(S.stub_out);
    pthread_mutex_lock(&S.mu);
    dur = ((S.cfg.use_pre ? 12480 : 0) + (S.cfg.use_cf ? 9120 : 0) + S.cfg.plen * 2280 +
           (S.cfg.gap + 15) / 16 * 16) / 40e6;
    start = t > S.stub_busy_until ? t : S.stub_busy_until;
    S.stub_busy_until = start + dur;
    S.stub_done[S.stub_n++] = S.stub_busy_until;
    pthread_mutex_unlock(&S.mu);
    (void)samples;
    return 0;
#else
    ssize_t n;
    if (S.buf && S.buf_samples != samples) hw_destroy_buffer();
    if (!S.buf) {
        struct iio_channel *i0 = iio_device_find_channel(S.dds, "voltage0", true);
        struct iio_channel *q0 = iio_device_find_channel(S.dds, "voltage1", true);
        if (!i0 || !q0) return -ENODEV;
        iio_channel_enable(i0);
        iio_channel_enable(q0);
        S.buf = iio_device_create_buffer(S.dds, samples, false);
        if (!S.buf) return -errno;
        S.buf_samples = samples;
    }
    memcpy(iio_buffer_start(S.buf), r->w, r->nwords * 8);
    n = iio_buffer_push(S.buf);
    return n < 0 ? (int)n : 0;
#endif
}

static void hw_radio(double lo, double gain, double bw)
{
#ifdef PTX_STUB
    (void)lo; (void)gain; (void)bw;
#else
    struct iio_channel *ch = iio_device_find_channel(S.phy, "voltage0", true);
    struct iio_channel *lo_ch = iio_device_find_channel(S.phy, "altvoltage1", true);
    if (ch) iio_channel_attr_write_longlong(ch, "sampling_frequency", 40000000);
    if (ch && bw > 0) iio_channel_attr_write_longlong(ch, "rf_bandwidth", (long long)bw);
    if (ch && gain <= 0) iio_channel_attr_write_double(ch, "hardwaregain", gain);
    if (lo_ch && lo > 0) iio_channel_attr_write_longlong(lo_ch, "frequency", (long long)lo);
#endif
}

/* ------------------------------------------------------------------------------------------ */
/* threads                                                                                     */

static void *tx_thread(void *arg)
{
    (void)arg;
    pthread_mutex_lock(&S.mu);
    while (!S.quit) {
        record r;
        int rc;
        while (!S.quit && S.qn == 0) pthread_cond_wait(&S.cv, &S.mu);
        if (S.quit) break;
        r = S.q[0];
        memmove(S.q, S.q + 1, (size_t)(S.qn - 1) * sizeof S.q[0]);
        S.qn--;
        S.fly[S.nfly++] = r.seq;
        pthread_mutex_unlock(&S.mu);
        rc = hw_push(&r);
        free(r.w);
        pthread_mutex_lock(&S.mu);
        if (rc) {
            char e[128];
            snprintf(e, sizeof e, "DMA push failed (%s)", strerror(-rc));
            send_error(r.seq, e);
            S.nfly--;
        } else {
            S.pushed++;
        }
        pthread_cond_broadcast(&S.cv);
    }
    pthread_mutex_unlock(&S.mu);
    return NULL;
}

static void *mon_thread(void *arg)
{
    (void)arg;
    while (1) {
        uint32_t st;
        pthread_mutex_lock(&S.mu);
        if (S.quit) { pthread_mutex_unlock(&S.mu); break; }
        st = hw_status();
        while (S.nfly && ((st - S.pl_count) & 0xFFFF)) {
            uint32_t seq = S.fly[0];
            memmove(S.fly, S.fly + 1, (size_t)(S.nfly - 1) * sizeof S.fly[0]);
            S.nfly--;
            S.pl_count = (S.pl_count + 1) & 0xFFFF;
            S.sent++;
            send_json(T_SENT, seq, "{\"seq\": %u, \"sent\": %u, \"pl_packets\": %u, \"header_errors\": %u, "
                      "\"underflows\": %u}", seq, S.sent, st & 0xFFFF, (st >> 16) & 0xFF, (st >> 24) & 0xF);
            send_json(T_READY, seq, "{\"credits\": %d}", credits());
            pthread_cond_broadcast(&S.cv);
        }
        if (!S.nfly) S.pl_count = st & 0xFFFF;
        pthread_mutex_unlock(&S.mu);
        usleep(500);
    }
    return NULL;
}

/* wait (S.mu held) until the queue and the packets on air are through, at most ms */
static int drain(int ms)
{
    double end = now_s() + ms * 1e-3;
    while (S.qn || S.nfly) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += 2000000;
        if (ts.tv_nsec >= 1000000000) { ts.tv_sec++; ts.tv_nsec -= 1000000000; }
        pthread_cond_timedwait(&S.cv, &S.mu, &ts);
        if (now_s() > end) return -1;
    }
    return 0;
}

static void handle(int type, uint32_t seq, uint8_t *p, uint32_t len)
{
    char err[256], reply[4096];
    switch (type) {
    case T_PING:
        send_json(T_PONG, seq, "{\"server\": \"ptx_server\", \"protocol\": 1, \"build\": %d, \"stub\": %s}",
                  BUILD,
#ifdef PTX_STUB
                  "true"
#else
                  "false"
#endif
        );
        break;
    case T_SET_PARAMS: {
        ptx_cfg c;
        ptx_derived d;
        char *txt = malloc(len + 1);
        if (!txt) { send_error(seq, "out of memory"); break; }
        memcpy(txt, p, len);
        txt[len] = 0;
        if (ptx_cfg_from_json(txt, &c, err, sizeof err) || ptx_derive(&c, &d, err, sizeof err)) {
            free(txt);
            send_json(T_PARAMS, seq, "{\"ok\": false, \"error\": \"%s\"}", err);
            break;
        }
        if (drain(5000)) { free(txt); send_error(seq, "queued packets did not go out within 5 s"); break; }
        {   /* optional radio settings */
            const char *r = strstr(txt, "\"Radio\"");
            double lo = -1, gain = 1, bw = -1;
            if (r) {
                const char *k;
                if ((k = strstr(r, "\"TxLo\""))) lo = strtod(strchr(k, ':') + 1, NULL);
                if ((k = strstr(r, "\"TxGain\""))) gain = strtod(strchr(k, ':') + 1, NULL);
                if ((k = strstr(r, "\"RfBandwidth\""))) bw = strtod(strchr(k, ':') + 1, NULL);
            }
            pthread_mutex_unlock(&S.mu);
            hw_radio(lo, gain, bw);
            pthread_mutex_lock(&S.mu);
        }
        free(txt);
        if (hw_enable(1)) { send_error(seq, "could not set DAC GPIO bit 4 (no Flexlink PL in this firmware?)"); break; }
        if (!(hw_status() & (1u << 29))) { send_error(seq, "the PL does not report enabled: firmware without ptx_pluto"); break; }
        S.cfg = c;
        S.d = d;
        S.have_params = 1;
        ptx_reply_json(&c, &d, reply, sizeof reply);
        send_msg(T_PARAMS, seq, reply, (uint32_t)strlen(reply));
        send_json(T_READY, seq, "{\"credits\": %d, \"depth\": %d}", credits(), S.depth);
        break;
    }
    case T_BLOCKS: {
        uint32_t nb;
        size_t want;
        record r;
        if (!S.have_params) { send_error(seq, "BLOCKS before SET_PARAMS"); break; }
        if (len < 4) { send_error(seq, "BLOCKS without a block count"); break; }
        memcpy(&nb, p, 4);
        want = (size_t)S.d.ndb * S.d.words_per_block * 8;
        if (nb != (uint32_t)S.d.ndb || len - 4 != want) {
            snprintf(err, sizeof err, "BLOCKS: %u blocks / %u bytes, one packet is %d blocks / %zu bytes",
                     nb, len - 4, S.d.ndb, want);
            send_error(seq, err);
            break;
        }
        if (S.qn + S.nfly >= QMAX) { send_error(seq, "BLOCKS without a credit: queue full"); break; }
        r.seq = seq;
        r.nwords = 6 + want / 8;
        r.w = malloc(r.nwords * 8);
        if (!r.w) { send_error(seq, "out of memory"); break; }
        ptx_header(&S.cfg, 1, r.w);
        memcpy(r.w + 6, p + 4, want);
        S.q[S.qn++] = r;
        pthread_cond_broadcast(&S.cv);
        break;
    }
    case T_STATUS: {
        uint32_t st = hw_status();
        send_json(T_STATUS_R, seq, "{\"have_params\": %s, \"queued\": %d, \"on_air\": %d, \"pushed\": %u, "
                  "\"sent\": %u, \"credits\": %d, \"pl_status\": \"%08x\", \"pl_packets\": %u, "
                  "\"header_errors\": %u, \"underflows\": %u, \"busy\": %u, \"enabled\": %u}",
                  S.have_params ? "true" : "false", S.qn, S.nfly, S.pushed, S.sent, credits(), st, st & 0xFFFF,
                  (st >> 16) & 0xFF, (st >> 24) & 0xF, (st >> 28) & 1, (st >> 29) & 1);
        break;
    }
    case T_STOP: {
        int i;
        for (i = 0; i < S.qn; i++) free(S.q[i].w);
        S.qn = 0;
        drain(2000);
        hw_enable(0);
        pthread_mutex_unlock(&S.mu);
        hw_destroy_buffer();
        pthread_mutex_lock(&S.mu);
        S.nfly = 0;
        S.have_params = 0;
        send_json(T_STATUS_R, seq, "{\"stopped\": true, \"pushed\": %u, \"sent\": %u}", S.pushed, S.sent);
        break;
    }
    default:
        send_error(seq, "unknown message type");
    }
}

static void serve(int fd)
{
    uint8_t h[16];
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    pthread_mutex_lock(&S.mu);
    S.sock = fd;
    pthread_mutex_unlock(&S.mu);
    while (recv_all(fd, h, 16) == 0) {
        uint32_t magic, seq, len;
        uint16_t type;
        uint8_t *p = NULL;
        memcpy(&magic, h, 4);
        type = (uint16_t)(h[4] | h[5] << 8);
        memcpy(&seq, h + 8, 4);
        memcpy(&len, h + 12, 4);
        if (magic != PTX_MAGIC || len > (1u << 24)) break;
        if (len && (!(p = malloc(len)) || recv_all(fd, p, len))) { free(p); break; }
        pthread_mutex_lock(&S.mu);
        if (S.sock == fd) handle(type, seq, p, len);
        pthread_mutex_unlock(&S.mu);
        free(p);
    }
    pthread_mutex_lock(&S.mu);
    if (S.sock == fd) { close(fd); S.sock = -1; }
    pthread_mutex_unlock(&S.mu);
}

int main(int argc, char **argv)
{
    int port = 5010, opt, ls, one = 1;
    struct sockaddr_in a;
    pthread_t t1, t2;
    const char *stub_file = "ptx_stub_stream.bin";
    S.depth = 2;
    while ((opt = getopt(argc, argv, "p:d:o:")) != -1) {
        if (opt == 'p') port = atoi(optarg);
        else if (opt == 'd') S.depth = atoi(optarg);
        else if (opt == 'o') stub_file = optarg;
        else { fprintf(stderr, "usage: %s [-p port] [-d credits] [-o stub output]\n", argv[0]); return 2; }
    }
    if (S.depth < 1 || S.depth > QMAX) S.depth = 2;
    signal(SIGPIPE, SIG_IGN);
    pthread_mutex_init(&S.mu, NULL);
    pthread_cond_init(&S.cv, NULL);
    S.sock = -1;
#ifdef PTX_STUB
    S.stub_out = fopen(stub_file, "wb");
    if (!S.stub_out) { perror(stub_file); return 1; }
    fprintf(stderr, "ptx_server (stub: records to %s, PL timing simulated)\n", stub_file);
#else
    (void)stub_file;
    S.ctx = iio_create_local_context();
    if (!S.ctx) { perror("iio_create_local_context"); return 1; }
    S.dds = iio_context_find_device(S.ctx, "cf-ad9361-dds-core-lpc");
    S.phy = iio_context_find_device(S.ctx, "ad9361-phy");
    if (!S.dds || !S.phy) { fprintf(stderr, "no cf-ad9361-dds-core-lpc / ad9361-phy\n"); return 1; }
    S.pl_count = hw_status() & 0xFFFF;
#endif
    pthread_create(&t1, NULL, tx_thread, NULL);
    pthread_create(&t2, NULL, mon_thread, NULL);
    ls = socket(AF_INET, SOCK_STREAM, 0);
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port = htons((uint16_t)port);
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(ls, (struct sockaddr *)&a, sizeof a) || listen(ls, 1)) { perror("bind/listen"); return 1; }
    fprintf(stderr, "ptx_server build %d listening on port %d, %d credits\n", BUILD, port, S.depth);
    for (;;) {
        int fd = accept(ls, NULL, NULL);
        if (fd < 0) { if (errno == EINTR) continue; perror("accept"); break; }
        serve(fd);
    }
    return 0;
}
