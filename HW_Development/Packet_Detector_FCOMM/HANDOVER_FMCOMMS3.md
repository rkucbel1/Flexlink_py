# Handover: Zedboard + AD-FMCOMMS3-EBZ FlexLink receiver

Written 2026-11-22, for a new development chat. Repo `Flexlink_py` (github `rkucbel1/Flexlink_py`,
branch `main`), on PC1 at `~/SiskinRF/Flexlink_py`. All paths below are relative to the repo.
The new development lives in `HW_Development/Packet_Detector_FCOMM/`; the CoreBoard receiver stays in
`HW_Development/Packet_Detector/`.

## 1. Goal

Build a second Zedboard FlexLink receiver that takes its samples from an **AD-FMCOMMS3-EBZ**
(AD9361) on the Zedboard's FMC connector, instead of from the CoreBoard over Ethernet.

- Keep the FPGA receiver (`pd_axi` / `pd_detect_top`, bitstream 9.2) unchanged as far as possible.
- Keep the results path to the PC unchanged: TCP port 5001, `STREAM_START` with the push flag,
  `STREAM_PUSH` (21) messages carrying transport-block records. Then
  `App/FlexlinkRxVideoWithZedboard.py` needs only a new radio-settings frame.
- The CoreBoard build must keep working. The new design is a build variant, not a replacement.

What the change buys:

- No 670 Mbit/s UDP sample stream. The Zynq Ethernet controller's single DMA engine was the
  main problem in the CoreBoard design (pacing, lost datagrams). Ethernet then only carries
  results and commands.
- 12-bit samples instead of int8. A proper, well-documented RF front end (70 MHz – 6 GHz on
  the FMCOMMS3). A path to a full transceiver later.

## 2. Where things stand (CoreBoard system, working)

Full details are in `HW_Development/Packet_Detector/STATUS.md`. Summary:

| Item | State |
|---|---|
| Bitstream | 9.2 (`PD_ID` 0x50440902): `CTRL.PACK8` (int8 unpacker), `CTRL.NOREC` |
| Firmware | build 39, `FW/pd_server.c`, lwIP raw API, bare metal |
| Vivado | **2025.2** (from `HW/clockInfo.txt`), part xc7z020clg484-1, board `zedboard:part0:1.1` |
| PL clock | everything on FCLK_CLK0 = 100 MHz |
| Timing | WNS **+0.206 ns** (`HW/build/timing_summary.rpt`) |
| Zedboard | 10.42.0.101, TCP 5001, loaded over JTAG with `xsdb run_board.tcl` in `FW/` |
| Link test | Pluto Tx at **2420 MHz** (2400–2440 overlaps local Wi-Fi channel 10): BLER 0 over 18,035 blocks |
| Live video | PC2 + Pi 4 (x264) + Pluto → CoreBoard + Zedboard → PC1. BPSK, QPSK, 16QAM verified |

### Resource use today (`HW/build/utilization.rpt`)

| Resource | Used | Available | % |
|---|---|---|---|
| Slice LUTs | 47,514 | 53,200 | **89 %** |
| Registers | 41,412 | 106,400 | 39 % |
| Block RAM tiles | 123.5 | 140 | **88 %** |
| DSPs | 205 | 220 | **93 %** |

**This is the biggest risk of the whole project.** About 5.7k LUTs, 16 BRAM tiles and 15 DSPs are
left for the AD9361 interface, the capture DMA and anything else. The ADI reference design for
fmcomms2/zed also includes HDMI, audio and I2C, and must not be used as-is. See section 5.

### Block design today (`HW/build_hw.tcl`)

```
PS7 --M_AXI_GP0--> smartconnect --> AXI DMA regs, pd_0 regs (0x43C00000)
DDR <--HP0-- AXI DMA MM2S (samples) --AXIS 32b--> pd_0 --AXIS 32b (debug records)--> AXI DMA S2MM --HP1--> DDR
```

Stream word = one complex sample, `[15:0]` = I, `[31:16]` = Q, 12-bit sign-extended. With
`CTRL.PACK8`, one word holds two int8 samples instead (CoreBoard only).

The receiver input is **not real-time on its own**: `pd_axi` holds `S_AXIS tready` low while the
tone check catches up, during a timing search, and when its event or timing FIFOs are nearly full.
At 20 Msps it is designed never to stall, but "never" was measured with the DDR ring in front. Any new
design needs a buffer in front of `pd_0` that absorbs stalls and counts overflows.

## 3. Hardware

| Item | Note |
|---|---|
| **VADJ** | **Set the Zedboard VADJ jumper (J18) to 2.5 V before plugging in the FMCOMMS3.** The board needs 1.71–2.625 V; 3.3 V damages it |
| Connector | Zedboard FMC LPC (J1). The ADI fmcomms2 project supports the ZED carrier on LPC |
| FMCOMMS3 vs 2 | FMCOMMS3 tunes 70 MHz – 6 GHz. (FMCOMMS2 is 2.4–2.5 GHz only.) Same HDL project as FMCOMMS2 |
| Interface | `axi_ad9361` supports LVDS only on this carrier. Use **1R1T** mode (one Rx channel) |
| Clock | The FMCOMMS3 has its own 40 MHz reference; data clock comes from the AD9361 (`l_clk`) |
| RF | First tests by cable: Pluto Tx → 30–40 dB attenuator → FMCOMMS3 Rx1 SMA. Then antennas |
| Boot | As now: JTAG boot jumpers, PROG USB J17, UART J14 at 115200 |

USB ports on PC1 are tight. JTAG (J17) is needed; the UART (J14) is optional once the network is up.

## 4. Recommended architecture

### Option A (recommended): capture into a DDR ring, feed `pd_0` from it

```
AD9361 --LVDS--> axi_ad9361 (1R1T) --> util_cpack2 --> axi_dmac (S2MM) --HP2--> DDR ring
                                                                                 |
DDR ring --HP0--> existing AXI DMA MM2S --AXIS 32b--> pd_0 (unchanged) --> results FIFOs --> firmware --> TCP push
```

- It is the same structure as the CoreBoard design, with `axi_dmac` writing the ring instead of the
  UDP receive code. The firmware's ring logic, DMA feed and push code stay; the UDP ring writer and
  HTTP client go.
- `axi_dmac` handles the clock-domain crossing from `l_clk` to 100 MHz.
- Stalls in `pd_0` are absorbed by the ring (MBs, not KBs). Overflow is visible as a ring overrun.
- The same ring gives a **CAPTURE command**: read raw AD9361 samples to the PC. This is the
  debugging tool for gain, DC and spectrum, and for making new test vectors from the real front end.
- DDR traffic: 40 Msps × 4 bytes = 160 MB/s in and 160 MB/s out (half at 20 Msps). Well within the
  HP ports.
- Latency: the ring adds about one DMA chunk. With no Ethernet in the way, chunks can be small, e.g.
  1024 samples = 25 µs at 40 Msps.
- `axi_dmac` transfers are queued by the firmware back-to-back. Check whether the HDL version in use
  supports cyclic or scatter-gather S2MM, which would make this simpler.

### Option B (fallback): direct AXI-Stream

`axi_ad9361` → async AXIS FIFO → `pd_0 S_AXIS`, with an overflow counter. It uses fewer LUTs and has the lowest
latency, but it depends on `pd_0` never stalling longer than the FIFO depth, and BRAM is at 88 %. It also
gives no capture path. Try only if option A doesn't fit.

### Sample rate: 20 Msps (input stage 1) or 40 Msps (input stage 2)?

| | 40 Msps, `INPUT_STAGE` 2 | 20 Msps, `INPUT_STAGE` 1 |
|---|---|---|
| Proven | yes (CoreBoard path, bit-true tests) | the detector's native rate; regression vectors exist |
| Filtering | AD9361 + `pd_halfband.v` in the PL | AD9361's own FIR / half-bands only |
| Resources | as now | `pd_halfband` could be removed (DSPs, LUTs). Needs a build parameter |
| DDR traffic | 320 MB/s | 160 MB/s |

Start at **40 Msps, stage 2**: that is the proven path. Move to 20 Msps if resources are short. The
AD9361 RF bandwidth should be about 18 MHz (the signal occupies 841 × 19.53 kHz ≈ 16.4 MHz).

## 5. Fitting it in (resources)

Measure, don't guess: build the AD9361 part alone first and read its utilization. Levers, roughly
in order:

1. **`axi_ad9361` parameters:** `MODE_1R1T` = 1, `ADC_DCFILTER_DISABLE` = 1,
   `ADC_IQCORRECTION_DISABLE` = 1, `DAC_DDS_DISABLE` = 1, `DAC_IQCORRECTION_DISABLE` = 1,
   `ADC_DATAFORMAT_DISABLE` if the sign-extended 12-bit output is kept. The AD9361 does its own DC and
   quadrature tracking. The ADI default output is 12-bit sign-extended to 16 bits, which matches the
   `pd_0` stream word. Verify on the hardware.
2. **Nothing else from the ADI zed base design:** no HDMI (`axi_hdmi_tx`, `axi_clkgen`), no audio, no
   I2S/SPDIF, no `axi_iic`. Keep only PS7, `axi_ad9361`, `util_cpack2` (maybe not even that in 1R1T),
   `axi_dmac`, and the existing AXI DMA + `pd_0`.
3. **Drop the AXI DMA S2MM channel** (debug records) in this variant. `NOREC` already discards them
   while streaming. Regression tests can still run on the CoreBoard build.
4. **20 Msps without `pd_halfband`** (above).
5. Smaller debug FIFOs in `pd_axi` (they cost BRAM).

Timing is also tight (WNS +0.206 ns at 100 MHz). Keep `axi_ad9361` and `axi_dmac`'s source side on
`l_clk`, and only the AXI side on 100 MHz.

## 6. Software: AD9361 control

- **No-OS** (`analogdevicesinc/no-OS`, `drivers/rf-transceiver/ad9361`, example `projects/ad9361`)
  runs bare metal. That fits `pd_server` (lwIP raw, no OS). It needs PS SPI0 (EMIO in the ADI design)
  and GPIOs for reset, sync, enable and txnrx.
- Init sets: Rx LO, sampling rate, RF bandwidth, gain-control mode and gain, FIR, and the digital
  interface tuning (the LVDS delay calibration; ADI's driver does this).
- PC control: a new command on TCP 5001, e.g. `RADIO_CONFIG` = 22 (the next free number; 21 is
  `STREAM_PUSH`), carrying LO, rate, RF bandwidth, gain mode and gain. It replaces the HTTP start/stop
  to the CoreBoard. Add `BOARD_FMCOMMS3` to the PING board flags (next free bit after `BOARD_NOREC`
  0x400 is 0x800).
- `STREAM_PUSH`'s "newest datagram slot" field (336 samples per slot) needs a meaning in the new
  design: use the ring write position in samples. The video app uses `t_trig`, so it doesn't care.

### Version matching

| Branch | Vivado |
|---|---|
| ADI HDL `main` | 2025.1 |
| ADI HDL `hdl_2023_r2` | 2023.2 |

You have **2025.2**. Use HDL `main` with `export ADI_IGNORE_VERSION_CHECK=1`, and the matching
no-OS `main`. Forum reports show mismatched HDL/no-OS branches give build errors. Only the library IPs
(`library/axi_ad9361`, `library/axi_dmac`, `library/util_pack/util_cpack2`, `library/xilinx/util_*`) are
needed; build them with `make` in each directory. Pin constraints come from
`projects/fmcomms2/zed/system_constr.xdc`.

## 7. Plan, with a check at every step

| Step | What | Done when |
|---|---|---|
| 0 | VADJ 2.5 V, FMCOMMS3 on FMC LPC. Boot ADI's prebuilt Kuiper Linux SD card for zed + fmcomms2/3 | `iio_info` sees ad9361-phy. IIO Oscilloscope or pyadi-iio shows the Pluto's FlexLink packets at 2420 MHz |
| 1 | Capture by pyadi-iio at 40 (and 20) Msps, save as `.cs16`. Run it through the Python reference receiver and the SIM testbench | Packets decode offline. Gain, `dc_shift` and SNR known. New test vectors from the real front end |
| 2 | `HW/build_hw_fmcomms3.tcl`: PS7 + `axi_ad9361` + `axi_dmac` only (no `pd_0`) | Builds. Utilization of the AD9361 part known |
| 3 | Firmware: no-OS AD9361 init + capture into DDR + CAPTURE command to the PC | PC capture matches step 1 |
| 4 | Add the existing AXI DMA + `pd_0`. Feed it from the ring (option A) | Fits, timing met. `pd_stream_test.py`-style test gives BLER ~0 with the Pluto |
| 5 | Push mode, `RADIO_CONFIG`, board flag | `FlexlinkRxVideoWithZedboard.py` with an FMCOMMS3 source plays video |
| 6 | Latency and gain-mode tuning | Trigger-to-block latency below the CoreBoard's 3–7 ms |

Step 0 needs no build at all and proves the hardware. Do it first, on a spare SD card. JTAG
boot is used later; the SD card path needs the SD boot jumpers.

## 8. Changes on the PC side

`App/FlexlinkRxVideoWithZedboard.py`:

- Replace or extend the CoreBoard frame with a source choice: CoreBoard / FMCOMMS3.
- New `App/Rx_FCOMMS3/Parameters_FMCOMMS3.json`: Frequency_MHz (2420), SampleRate (40e6), RFBandwidth_MHz
  (18), GainMode (manual / fast_attack / slow_attack / hybrid), Gain, Port (Rx1A).
- For FMCOMMS3, send `RADIO_CONFIG` on the TCP connection instead of the HTTP start to the
  CoreBoard. `detector_params` sets `input_stage` from the sample rate; `dc_shift` from step 1.
- Everything after `STREAM_START` (push parsing, `VideoRx`, `Video_Framing.py`, ffplay) stays.

`PC/pd_protocol.py`: `RADIO_CONFIG`, `CAPTURE` and `BOARD_FMCOMMS3`.

## 9. Open questions and risks

1. **Resources** (section 5). If option A doesn't fit even with 20 Msps and no S2MM, the choice is
   cutting receiver features or moving to a bigger part (Zynq-7030/7035 boards with FMC, or the
   ADRV9361-Z7035 SOM).
2. **Gain control for bursts.** Start with manual gain (cable test). Fast-attack AGC can change gain
   inside a packet, which would hurt the channel estimate; check whether gain can be locked after
   the preamble, or use slow attack.
3. **DC and IQ.** The AD9361 tracks RF/BB DC and quadrature itself. `dc_shift` 6 was for the
   CoreBoard's DC wander and may be too strong or unnecessary here.
4. **Clock offsets.** The Pluto and FMCOMMS3 have independent references, as the Pluto and
   CoreBoard did. CFO correction covers up to about 600 kHz, so this should be fine.
5. **Tool versions** (section 6).
6. **LVDS interface tuning** at 40 Msps in 1R1T. The ADI driver's digital tune must pass. There are
   forum reports of tuning failures at higher rates in 2R2T.

## 10. Lessons from the CoreBoard work (still relevant)

- AXI-Lite reads cost about 0.76 µs each. Read FIFO levels once, not per word.
- The receiver's 32-bit sample counter wraps (107 s at 40 Msps). The PC unwraps it.
- `pl_chest.v` early-decision fix: input pauses could stall the receiver. The SIM testbench has
  `BURST` / `BURSTGAP` / `BURSTOFF` options and a deadlock guard. Use them for the ring feed.
- Commit files to PC1 under a **new staged filename** each time and check the size with a directory
  listing (an old copy was once silently kept).
- 2400–2440 MHz has Wi-Fi channel 10 nearby. Use 2420 MHz on both sides, or move to another band
  now that the FMCOMMS3 covers 70 MHz – 6 GHz (the Pluto too).
- The Pluto runs the transmitter (`HW_Development/Packet_Transmitter/FW/pluto_start.sh`,
  `ptx_server` on 192.168.2.1:5010). The Tx app is `App/FlexlinkTxVideoWithPluto.py` (PC2).

## 11. Files

| File | What |
|---|---|
| `HW_Development/Packet_Detector/STATUS.md` | CoreBoard streaming status, protocol flags, lessons |
| `HW_Development/Packet_Detector/README.md`, `RTL/README.md` | receiver steps, per-block resources and tests |
| `HW_Development/Packet_Detector/HW/build_hw.tcl` | current block design (start for `build_hw_fmcomms3.tcl`) |
| `HW_Development/Packet_Detector/HW/build/utilization*.rpt` | current resource use |
| `HW_Development/Packet_Detector/RTL/pd_axi.v` | registers, CTRL bits, flow control |
| `HW_Development/Packet_Detector/FW/pd_server.c`, `build_firmware.py`, `run_board.tcl` | firmware and loading |
| `HW_Development/Packet_Detector/PC/pd_protocol.py`, `pd_stream_test.py` | protocol and streaming test |
| `HW_Development/Packet_Detector/SIM/` | testbench, `run_sim.sh` |
| `App/FlexlinkRxVideoWithZedboard.py` | receiver video app (CoreBoard source today) |
| `App/FlexlinkTxVideoWithPluto.py`, `Flexlink_BasicModules/Video_Transfer/Video_Framing.py` | video Tx and framing |

### External

- ADI HDL: `github.com/analogdevicesinc/hdl`, `projects/fmcomms2/zed`, `library/axi_ad9361`
- ADI no-OS: `github.com/analogdevicesinc/no-OS`, `projects/ad9361`, `drivers/rf-transceiver/ad9361`
- ADI Kuiper Linux image (for step 0), pyadi-iio, IIO Oscilloscope

## 12. Housekeeping left from the video work

- Delete `App/FlexlinkRxVideoWithPluto.py` (renamed to `...WithZedboard.py`).
- Commit and push the latest `Video_Framing.py` and both video apps.
- Optional: add `App/Video/` to `.gitignore`. "Lost" block counts are inflated when the transmitter
  restarts its sequence counter (cosmetic).
