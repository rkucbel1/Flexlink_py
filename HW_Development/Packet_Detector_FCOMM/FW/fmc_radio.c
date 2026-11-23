/*
 * fmc_radio.c  (Flexlink_py/HW_Development/Packet_Detector_FCOMM/FW)
 *
 * See fmc_radio.h. The AD9361 is driven by ADI's no-OS driver (rf-transceiver/ad9361) with the
 * platform data copied in fmc_ad9361_param.c; this file changes what this board needs:
 *   - 1R1T (one Rx channel; the bitstream's axi_ad9361 has MODE_1R1T = 1)
 *   - reset on Zynq GPIO 100, no sync / calibration-switch GPIOs (single FMCOMMS3)
 *   - Tx: maximum attenuation, Tx LO 100 MHz away from the Rx LO (this is a receiver; the
 *     AD9361 runs in FDD, so its transmitter is on, and its LO leakage stays out of band)
 *   - Rx LO, RF bandwidth, gain mode and gain from the configuration
 * ad9361_init() runs the digital interface tune at 61.44 MHz (the worst case), so the sample
 * rate is set afterwards with ad9361_set_rx_sampling_freq() and needs no new tune.
 */
#include <stdio.h>
#include <string.h>

#include "xparameters.h"
#include "xil_io.h"
#include "xil_cache.h"
#include "xil_printf.h"

#include "app_config.h"
#include "ad9361_api.h"
#include "ad9361.h"
#include "axi_adc_core.h"
#include "axi_dac_core.h"
#include "fmc_radio.h"

extern AD9361_InitParam fmc_ad9361_default;     /* fmc_ad9361_param.c */
extern struct axi_adc_init fmc_rx_adc_init;
extern struct axi_dac_init fmc_tx_dac_init;

/* axi_ad9361 registers (ADI regmap; offsets from FMC_AD9361_BASE) */
#define AD_CLK_FREQ             0x0054u
#define AD_STATUS               0x005Cu
#define AD_DMA_STATUS           0x0088u         /* bit 2 overflow, bit 1 underflow (W1C) */
#define AD_DMA_OVF              0x4u
#define AD_CHAN_STATUS(c)       (0x0404u + 0x40u * (c))

/* axi_dmac registers (no-OS axi_dmac.h) */
#define DMAC_IRQ_MASK           0x080u
#define DMAC_IRQ_PENDING        0x084u
#define DMAC_CTRL               0x400u
#define DMAC_TRANSFER_ID        0x404u
#define DMAC_TRANSFER_SUBMIT    0x408u
#define DMAC_FLAGS              0x40Cu
#define DMAC_DEST_ADDRESS       0x410u
#define DMAC_X_LENGTH           0x418u
#define DMAC_Y_LENGTH           0x41Cu
#define DMAC_DEST_STRIDE        0x420u
#define DMAC_TRANSFER_DONE      0x428u
#define DMAC_CHUNK_BYTES        (8u << 20)      /* per transfer; two queued keep the stream gap-free */
#define DMAC_GUARD_BYTES        65536u          /* after the last transfer, see fmc_capture() */

#define TX_ATTEN_MAX_MDB        89750u
#define TX_LO_OFFSET_HZ         100000000ull

/* Scratch target of the guard transfer: the samples after the capture go here, so the DMA
 * still accepts data (no overflow) while the end of the capture is checked. 1.6 ms at 40 Msps. */
static u8 guard_buf[DMAC_GUARD_BYTES] __attribute__((aligned(64)));

int fmc_ring_active(void);

static struct ad9361_rf_phy *phy;
static AD9361_InitParam init_param;
static struct fmc_radio_cfg cur;
static int radio_ok;
static s32 last_error;

/* Cortex-A9 global timer (CPU clock / 2), as pd_server.c */
#define GTIMER_BASE             0xF8F00200u
#if defined(XPAR_CPU_CORE_CLOCK_FREQ_HZ)
#define GT_COUNTS_PER_US        (XPAR_CPU_CORE_CLOCK_FREQ_HZ / 2000000u)
#else
#define GT_COUNTS_PER_US        333u            /* Zedboard: 666.67 MHz CPU */
#endif
static u64 gt_now(void)
{
    u32 hi, lo;
    if ((Xil_In32(GTIMER_BASE + 0x08u) & 1u) == 0u) {
        Xil_Out32(GTIMER_BASE + 0x08u, (Xil_In32(GTIMER_BASE + 0x08u) & ~0xFF00u) | 1u);
    }
    do {
        hi = Xil_In32(GTIMER_BASE + 0x04u);
        lo = Xil_In32(GTIMER_BASE + 0x00u);
    } while (Xil_In32(GTIMER_BASE + 0x04u) != hi);
    return ((u64)hi << 32) | lo;
}

static u32 ad_rd(u32 off) { return Xil_In32(FMC_AD9361_BASE + off); }
static void ad_wr(u32 off, u32 v) { Xil_Out32(FMC_AD9361_BASE + off, v); }
static u32 dma_rd(u32 off) { return Xil_In32(FMC_DMA_BASE + off); }
static void dma_wr(u32 off, u32 v) { Xil_Out32(FMC_DMA_BASE + off, v); }

static u64 tx_lo_for(u64 rx_lo)
{
    return rx_lo + TX_LO_OFFSET_HZ <= 6000000000ull ? rx_lo + TX_LO_OFFSET_HZ : rx_lo - TX_LO_OFFSET_HZ;
}

static int check_cfg(const struct fmc_radio_cfg *c, char *err, u32 err_len)
{
    if (c->lo_hz < 70000000ull || c->lo_hz > 6000000000ull) {
        snprintf(err, err_len, "LO %lu kHz outside 70 MHz .. 6 GHz", (unsigned long)(c->lo_hz / 1000u));
        return -1;
    }
    if (c->rate_hz < 2083334u || c->rate_hz > 61440000u) {
        snprintf(err, err_len, "sample rate %lu Hz outside 2.083 .. 61.44 MHz", (unsigned long)c->rate_hz);
        return -1;
    }
    if (c->bw_hz < 200000u || c->bw_hz > 56000000u) {
        snprintf(err, err_len, "RF bandwidth %lu Hz outside 0.2 .. 56 MHz", (unsigned long)c->bw_hz);
        return -1;
    }
    if (c->gain_mode > FMC_GAIN_HYBRID) {
        snprintf(err, err_len, "gain mode %lu (0 manual, 1 fast attack, 2 slow attack, 3 hybrid)",
                 (unsigned long)c->gain_mode);
        return -1;
    }
    if (c->gain_mode == FMC_GAIN_MANUAL && (c->gain_db < -3 || c->gain_db > 71)) {
        snprintf(err, err_len, "manual gain %ld dB outside -3 .. 71 dB", (long)c->gain_db);
        return -1;
    }
    if (c->port > 8u) {
        snprintf(err, err_len, "Rx port %lu unknown", (unsigned long)c->port);
        return -1;
    }
    return 0;
}

static int fail(const char *what, s32 ret, char *err, u32 err_len)
{
    last_error = ret;
    snprintf(err, err_len, "%s failed (%ld)", what, (long)ret);
    xil_printf("AD9361: %s failed (%d)\r\n", what, (int)ret);
    return ret < 0 ? (int)ret : -1;
}

static void log_status(void)
{
    struct fmc_radio_status st;
    fmc_radio_get_status(&st);
    xil_printf("AD9361: LO %d kHz, %d Hz, RF BW %d Hz, gain mode %d, gain %d dB, ENSM %d\r\n",
               (int)(st.cfg.lo_hz / 1000u), (int)st.cfg.rate_hz, (int)st.cfg.bw_hz,
               (int)st.cfg.gain_mode, (int)st.cfg.gain_db, (int)st.ensm_mode);
    xil_printf("axi_ad9361: l_clk %d kHz, status 0x%x, channel status I 0x%x Q 0x%x\r\n",
               (int)(((u64)st.clk_freq_raw * 100000u) >> 16), (unsigned int)st.adc_status,
               (unsigned int)st.chan_status_i, (unsigned int)st.chan_status_q);
}

int fmc_radio_init(const struct fmc_radio_cfg *cfg, char *err, u32 err_len)
{
    s32 ret;
    struct fmc_radio_cfg want = *cfg;

    if (check_cfg(cfg, err, err_len) < 0) {
        return -1;
    }
    if (phy != NULL) {
        ad9361_remove(phy);
        phy = NULL;
    }
    radio_ok = 0;
    last_error = 0;

    init_param = fmc_ad9361_default;
    init_param.two_rx_two_tx_mode_enable = 0;
    init_param.one_rx_one_tx_mode_use_rx_num = 1;
    init_param.one_rx_one_tx_mode_use_tx_num = 1;
    init_param.gpio_resetb.number = FMC_GPIO_RESETB;
    init_param.gpio_sync.number = -1;
    init_param.gpio_cal_sw1.number = -1;
    init_param.gpio_cal_sw2.number = -1;
    init_param.rx_synthesizer_frequency_hz = want.lo_hz;
    init_param.tx_synthesizer_frequency_hz = tx_lo_for(want.lo_hz);
    init_param.rf_rx_bandwidth_hz = want.bw_hz;
    init_param.tx_attenuation_mdB = (int32_t)TX_ATTEN_MAX_MDB;
    init_param.gc_rx1_mode = (uint8_t)want.gain_mode;
    init_param.gc_rx2_mode = (uint8_t)want.gain_mode;
    init_param.rx_rf_port_input_select = want.port;
    /* 1R1T: two converter channels (I, Q), as no-OS main.c does without 2R2T */
    fmc_rx_adc_init.num_channels = 2;
    fmc_rx_adc_init.num_slave_channels = 0;
    fmc_tx_dac_init.num_channels = 2;
    fmc_tx_dac_init.rate = 1;

    xil_printf("AD9361: initialising (LO %d kHz, 1R1T, LVDS) ...\r\n", (int)(want.lo_hz / 1000u));
    ret = ad9361_init(&phy, &init_param);
    if (ret < 0 || phy == NULL) {
        phy = NULL;
        return fail("ad9361_init (reset, SPI, PLLs, digital interface tune)", ret, err, err_len);
    }
    cur = want;
    cur.rate_hz = 0;                    /* force the rate below */
    cur.gain_db = -100;                 /* and the manual gain */
    radio_ok = 1;
    ret = fmc_radio_config(&want, err, err_len);
    if (ret < 0) {
        radio_ok = 0;
        return ret;
    }
    xil_printf("AD9361: ready\r\n");
    log_status();
    return 0;
}

int fmc_radio_config(const struct fmc_radio_cfg *cfg, char *err, u32 err_len)
{
    s32 ret;

    if (!radio_ok || phy == NULL) {
        return fmc_radio_init(cfg, err, err_len);
    }
    if (check_cfg(cfg, err, err_len) < 0) {
        return -1;
    }
    if (cfg->rate_hz != cur.rate_hz) {
        ret = ad9361_set_rx_sampling_freq(phy, cfg->rate_hz);
        if (ret < 0) {
            return fail("set sample rate", ret, err, err_len);
        }
        cur.rate_hz = cfg->rate_hz;
    }
    if (cfg->bw_hz != cur.bw_hz) {
        ret = ad9361_set_rx_rf_bandwidth(phy, cfg->bw_hz);
        if (ret < 0) {
            return fail("set RF bandwidth", ret, err, err_len);
        }
        cur.bw_hz = cfg->bw_hz;
    }
    if (cfg->lo_hz != cur.lo_hz) {
        ret = ad9361_set_rx_lo_freq(phy, cfg->lo_hz);
        if (ret >= 0) {
            ret = ad9361_set_tx_lo_freq(phy, tx_lo_for(cfg->lo_hz));
        }
        if (ret < 0) {
            return fail("set LO", ret, err, err_len);
        }
        cur.lo_hz = cfg->lo_hz;
    }
    if (cfg->port != cur.port) {
        ret = ad9361_set_rx_rf_port_input(phy, cfg->port);
        if (ret < 0) {
            return fail("set Rx port", ret, err, err_len);
        }
        cur.port = cfg->port;
    }
    if (cfg->gain_mode != cur.gain_mode) {
        ret = ad9361_set_rx_gain_control_mode(phy, 0, (uint8_t)cfg->gain_mode);
        if (ret < 0) {
            return fail("set gain mode", ret, err, err_len);
        }
        cur.gain_mode = cfg->gain_mode;
        cur.gain_db = -100;
    }
    if (cfg->gain_mode == FMC_GAIN_MANUAL && cfg->gain_db != cur.gain_db) {
        ret = ad9361_set_rx_rf_gain(phy, 0, cfg->gain_db);
        if (ret < 0) {
            return fail("set manual gain", ret, err, err_len);
        }
        cur.gain_db = cfg->gain_db;
    }
    last_error = 0;
    return 0;
}

void fmc_radio_get_status(struct fmc_radio_status *st)
{
    memset(st, 0, sizeof(*st));
    st->ok = (u32)radio_ok;
    st->last_error = last_error;
    st->cfg = cur;
    if (radio_ok && phy != NULL) {
        uint32_t mode = 0;
        int32_t g = 0;
        if (ad9361_get_en_state_machine_mode(phy, &mode) == 0) {
            st->ensm_mode = mode;
        }
        if (ad9361_get_rx_rf_gain(phy, 0, &g) == 0) {
            st->cfg.gain_db = g;
        }
    }
    st->clk_freq_raw = ad_rd(AD_CLK_FREQ);
    st->adc_status = ad_rd(AD_STATUS);
    st->dma_status = ad_rd(AD_DMA_STATUS);
    st->chan_status_i = ad_rd(AD_CHAN_STATUS(0));
    st->chan_status_q = ad_rd(AD_CHAN_STATUS(1));
}

/* ---------------------------------------------------------------------------------- */
/* Capture: axi_dmac, FIFO source (l_clk) -> DDR, transfers of DMAC_CHUNK_BYTES queued two  */
/* deep so the stream has no gap. The DMA's transfer IDs cycle through its queue slots; with */
/* at most two in flight an ID is never reused before its completion has been seen.         */
/* Overflow (axi_ad9361 0x88 bit 2, sticky) is cleared when the first transfer has finished  */
/* (before that the DMA was starting up) and read when the last one has, while a guard      */
/* transfer into guard_buf still takes the samples (without it, the samples arriving after  */
/* the last transfer would overflow at once and every capture would report an overflow).    */
/* ---------------------------------------------------------------------------------- */
int fmc_capture(UINTPTR addr, u32 n_samples, u32 timeout_ms, struct fmc_capture_result *res,
                char *err, u32 err_len)
{
    u32 bytes, n_chunks, submitted = 0, done = 0;
    u32 ids[2] = {0u, 0u};
    u64 t0, t_end;
    u32 status;

    memset(res, 0, sizeof(*res));
    if (fmc_ring_active()) {
        snprintf(err, err_len, "capture not possible while streaming (STREAM_STOP first)");
        return -1;
    }
    if (!radio_ok) {
        snprintf(err, err_len, "AD9361 not set up (RADIO_CONFIG first)");
        return -1;
    }
    if (addr & 63u) {
        snprintf(err, err_len, "capture address not 64-byte aligned");
        return -1;
    }
    n_samples = (n_samples + 15u) & ~15u;
    if (n_samples == 0u) {
        snprintf(err, err_len, "capture of 0 samples");
        return -1;
    }
    bytes = 4u * n_samples;
    n_chunks = (bytes + DMAC_CHUNK_BYTES - 1u) / DMAC_CHUNK_BYTES;

    Xil_DCacheInvalidateRange(addr, bytes);
    Xil_DCacheInvalidateRange((UINTPTR)guard_buf, DMAC_GUARD_BYTES);
    dma_wr(DMAC_CTRL, 0u);
    dma_wr(DMAC_CTRL, 1u);                      /* enable */
    dma_wr(DMAC_IRQ_MASK, 3u);                  /* polled: completion from TRANSFER_DONE */
    dma_wr(DMAC_IRQ_PENDING, 3u);
    dma_wr(DMAC_FLAGS, 0u);                     /* not cyclic */

    t0 = gt_now();
    t_end = t0 + (u64)(timeout_ms ? timeout_ms : 5000u) * 1000u * GT_COUNTS_PER_US;
    while (done < n_chunks) {
        if (submitted > done) {
            if (dma_rd(DMAC_TRANSFER_DONE) & (1u << ids[done & 1u])) {
                done++;
                if (done == 1u) {
                    /* Clear the sticky overflow once the first transfer has finished: before
                     * that, the DMA was still starting up (samples arrive before it accepts
                     * them). From here on, an overflow means a real gap in the data. */
                    ad_wr(AD_DMA_STATUS, 7u);
                }
                continue;
            }
        }
        if (submitted <= n_chunks && submitted - done < 2u &&
            (dma_rd(DMAC_TRANSFER_SUBMIT) & 1u) == 0u) {
            u32 off = submitted * DMAC_CHUNK_BYTES;
            u32 len;
            UINTPTR dst;
            if (submitted < n_chunks) {
                dst = addr + off;
                len = bytes - off < DMAC_CHUNK_BYTES ? bytes - off : DMAC_CHUNK_BYTES;
            } else {
                dst = (UINTPTR)guard_buf;       /* the guard transfer */
                len = DMAC_GUARD_BYTES;
            }
            ids[submitted & 1u] = dma_rd(DMAC_TRANSFER_ID) & 0x1Fu;
            dma_wr(DMAC_DEST_ADDRESS, (u32)dst);
            dma_wr(DMAC_DEST_STRIDE, 0u);
            dma_wr(DMAC_X_LENGTH, len - 1u);
            dma_wr(DMAC_Y_LENGTH, 0u);
            dma_wr(DMAC_TRANSFER_SUBMIT, 1u);
            submitted++;
            continue;
        }
        if (gt_now() > t_end) {
            dma_wr(DMAC_CTRL, 0u);
            snprintf(err, err_len, "capture timeout after %lu of %lu transfers (l_clk running? "
                     "axi_ad9361 status 0x%lx)", (unsigned long)done, (unsigned long)n_chunks,
                     (unsigned long)ad_rd(AD_STATUS));
            res->n_samples = done * (DMAC_CHUNK_BYTES / 4u);
            Xil_DCacheInvalidateRange(addr, bytes);
            return -2;
        }
    }
    /* the guard transfer is still taking samples here. A capture of one transfer (up to 2M
     * samples) has no overflow check: the flag was cleared at its end. */
    status = n_chunks > 1u ? ad_rd(AD_DMA_STATUS) : 0u;
    dma_wr(DMAC_CTRL, 0u);
    res->time_us = (u32)((gt_now() - t0) / GT_COUNTS_PER_US);
    res->n_samples = n_samples;
    res->dma_status = status;
    res->overflow = (status & AD_DMA_OVF) ? 1u : 0u;
    Xil_DCacheInvalidateRange(addr, bytes);
    return 0;
}

/* ---------------------------------------------------------------------------------- */
/* Ring writer (see fmc_radio.h). Transfer IDs are checked in order, as in fmc_capture():  */
/* with at most FMC_RING_INFLIGHT (2) in flight an ID is never reused unseen. The overflow */
/* flag is cleared when the first transfer has finished (start-up) and then checked at    */
/* every poll; set again means the DMA had no transfer queued: samples were lost.         */
/* ---------------------------------------------------------------------------------- */
static int ring_on;
static UINTPTR ring_base;
static u32 ring_slot_bytes, ring_slots;
static u64 ring_sub, ring_done;
static u32 ring_ids[FMC_RING_INFLIGHT];
static int ring_full_now;

int fmc_ring_active(void)
{
    return ring_on;
}

int fmc_ring_start(UINTPTR base, u32 slot_bytes, u32 n_slots, char *err, u32 err_len)
{
    if (!radio_ok) {
        snprintf(err, err_len, "AD9361 not set up (RADIO_CONFIG first)");
        return -1;
    }
    if ((base & 63u) || (slot_bytes & 63u) || slot_bytes == 0u || n_slots < 2u * FMC_RING_INFLIGHT) {
        snprintf(err, err_len, "ring: base and slot size must be 64-byte aligned, at least %u slots",
                 (unsigned)(2u * FMC_RING_INFLIGHT));
        return -1;
    }
    Xil_DCacheFlush();
    ring_base = base;
    ring_slot_bytes = slot_bytes;
    ring_slots = n_slots;
    ring_sub = ring_done = 0;
    ring_full_now = 0;
    dma_wr(DMAC_CTRL, 0u);
    dma_wr(DMAC_CTRL, 1u);
    dma_wr(DMAC_IRQ_MASK, 3u);
    dma_wr(DMAC_IRQ_PENDING, 3u);
    dma_wr(DMAC_FLAGS, 0u);
    ring_on = 1;
    return 0;
}

u32 fmc_ring_poll(u64 rd, struct fmc_ring_stat *st)
{
    u32 n = 0;
    if (!ring_on) {
        return 0;
    }
    for (;;) {
        if (ring_sub > ring_done &&
            (dma_rd(DMAC_TRANSFER_DONE) & (1u << ring_ids[ring_done % FMC_RING_INFLIGHT]))) {
            ring_done++;
            n++;
            if (ring_done == 1u) {
                ad_wr(AD_DMA_STATUS, 7u);       /* start-up overflow cleared */
            }
            continue;
        }
        if (ring_sub - ring_done < FMC_RING_INFLIGHT && (dma_rd(DMAC_TRANSFER_SUBMIT) & 1u) == 0u) {
            if (ring_sub - rd >= ring_slots) {  /* the reader still needs that slot */
                if (!ring_full_now) {
                    st->ring_full++;
                    ring_full_now = 1;
                }
                break;
            }
            ring_full_now = 0;
            ring_ids[ring_sub % FMC_RING_INFLIGHT] = dma_rd(DMAC_TRANSFER_ID) & 0x1Fu;
            dma_wr(DMAC_DEST_ADDRESS, (u32)(ring_base + (u32)(ring_sub % ring_slots) * ring_slot_bytes));
            dma_wr(DMAC_DEST_STRIDE, 0u);
            dma_wr(DMAC_X_LENGTH, ring_slot_bytes - 1u);
            dma_wr(DMAC_Y_LENGTH, 0u);
            dma_wr(DMAC_TRANSFER_SUBMIT, 1u);
            ring_sub++;
            continue;
        }
        break;
    }
    if (ring_done > 0u && (ad_rd(AD_DMA_STATUS) & AD_DMA_OVF)) {
        st->overflows++;
        ad_wr(AD_DMA_STATUS, 7u);
    }
    st->written = ring_done;
    return n;
}

void fmc_ring_stop(struct fmc_ring_stat *st)
{
    u64 t_end;
    if (!ring_on) {
        return;
    }
    t_end = gt_now() + 50000ull * GT_COUNTS_PER_US;
    while (ring_sub > ring_done && gt_now() < t_end) {
        if (dma_rd(DMAC_TRANSFER_DONE) & (1u << ring_ids[ring_done % FMC_RING_INFLIGHT])) {
            ring_done++;
        }
    }
    if (ring_sub > ring_done) {
        st->timeouts += (u32)(ring_sub - ring_done);
    }
    dma_wr(DMAC_CTRL, 0u);
    st->written = ring_done;
    ring_on = 0;
}
