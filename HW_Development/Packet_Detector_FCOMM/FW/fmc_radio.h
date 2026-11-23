/*
 * fmc_radio.h  (Flexlink_py/HW_Development/Packet_Detector_FCOMM/FW)
 *
 * The FMCOMMS3 (AD9361) as a sample source: chip set-up through ADI's no-OS driver, and
 * captures into DDR through axi_dmac (programmed here at register level, not with no-OS's
 * axi_dmac driver, which runs only one 16 MB transfer without interrupts and cannot do the
 * back-to-back device-to-memory transfers the step-4 ring needs).
 *
 * Hardware: HW/build_hw_fmcomms3.tcl (addresses and GPIO numbers as ADI's fmcomms2/zed design).
 */
#ifndef FMC_RADIO_H_
#define FMC_RADIO_H_

#include "xil_types.h"

#define FMC_AD9361_BASE         0x79020000u     /* axi_ad9361 Rx core; Tx core at +0x4000 */
#define FMC_DMA_BASE            0x7C400000u     /* axi_dmac, AD9361 -> DDR */
#define FMC_GPIO_RESETB         100             /* Zynq GPIO (EMIO bit 46) */

/* gain_mode: as the AD9361's gain control modes (no-OS enum rf_gain_ctrl_mode) */
#define FMC_GAIN_MANUAL         0u
#define FMC_GAIN_FAST_ATTACK    1u
#define FMC_GAIN_SLOW_ATTACK    2u
#define FMC_GAIN_HYBRID         3u

struct fmc_radio_cfg {
    u64 lo_hz;              /* Rx LO, 70 MHz .. 6 GHz */
    u32 rate_hz;            /* sample rate, e.g. 40e6 (input stage 2) or 20e6 */
    u32 bw_hz;              /* Rx RF bandwidth, e.g. 18e6 */
    u32 gain_mode;          /* FMC_GAIN_* */
    s32 gain_db;            /* manual gain (FMC_GAIN_MANUAL only) */
    u32 port;               /* Rx input: 0 = A balanced (FMCOMMS3 RX1A SMA) */
};

struct fmc_radio_status {
    u32 ok;                 /* 1: the AD9361 is set up */
    s32 last_error;         /* last no-OS return code (0 = none) */
    struct fmc_radio_cfg cfg;   /* as set; gain_db read back from the chip */
    u32 ensm_mode;          /* AD9361 state machine (no-OS ENSM_MODE_*, 3 = FDD) */
    u32 clk_freq_raw;       /* axi_ad9361 0x54: l_clk = raw * 100 MHz / 65536 */
    u32 adc_status;         /* axi_ad9361 0x5C: bit 0 = interface status OK */
    u32 dma_status;         /* axi_ad9361 0x88: bit 2 = DMA overflow (sticky) */
    u32 chan_status_i;      /* axi_ad9361 0x404: PN error / PN out of sync / over range */
    u32 chan_status_q;      /* axi_ad9361 0x444 */
};

struct fmc_capture_result {
    u32 n_samples;          /* samples written (one u32 each: I bits 15:0, Q bits 31:16) */
    u32 overflow;           /* 1: samples were lost after the first transfer (8 MB) */
    u32 time_us;            /* capture time */
    u32 dma_status;         /* axi_ad9361 0x88 at the end of the capture */
};

/* Set up the AD9361 (reset, platform data, digital interface tune, then cfg). 0 or a negative
 * no-OS error code; a message for the PC in err. Can be called again to start over. */
int fmc_radio_init(const struct fmc_radio_cfg *cfg, char *err, u32 err_len);

/* Change the settings that differ from the current ones (LO, rate, bandwidth, gain, port). */
int fmc_radio_config(const struct fmc_radio_cfg *cfg, char *err, u32 err_len);

void fmc_radio_get_status(struct fmc_radio_status *st);

/* Capture n_samples (rounded up to a multiple of 16) to addr, a 64-byte aligned DDR address.
 * Cache lines of the range are invalidated before and after. 0 or a negative code. */
int fmc_capture(UINTPTR addr, u32 n_samples, u32 timeout_ms, struct fmc_capture_result *res,
                char *err, u32 err_len);

#endif
