# Packet detector on the Zedboard

Hardware test bench for the FPGA packet detector (`Specification_PacketDetector.md`).
Everything is built from scripts; the generated Vivado project and Vitis workspace are
throw-away build output.

| Folder | Contents |
| --- | --- |
| `PC/` | Test-vector builder, golden reference, TCP client, board emulator (see `PC/README.md`) |
| `HW/build_hw.tcl` | Vivado 2025.2: block design, bitstream, `HW/build/pd_zed.xsa` |
| `FW/pd_server.c` | Board firmware: lwIP TCP server (port 5001) implementing `PC/pd_protocol.py` |
| `FW/build_firmware.py` | Vitis 2025.2: platform + lwIP + app from the echo-server template, with `pd_server.c` |
| `FW/run_board.tcl` | XSDB: program the FPGA over JTAG and start the firmware |
| `BITTRUE.md` | Bit-true model: word lengths, spec changes, verification against the simulator |
| `RTL/` | Verilog of the detector: DC blocker, LPF, detection core → `t_trig`, debug stream; CFO lane → `cfo_inc`; tone check → event; see `RTL/README.md` |
| `SIM/` | Testbench against the bit-true model, `run_sim.sh` (xsim or Verilator); `SIM/cosim/`: firmware + RTL co-simulation behind a TCP port (Verilator); see `SIM/README.md` |
| `HW/synth_detect.tcl` | Out-of-context synthesis + place & route of `RTL/` at 100 MHz: timing and resources |
| `TestVectors/` | Output of `PC/pd_make_vector.py` |

## Phase 1: plumbing

```
PC ──Ethernet──► ARM: lwIP server ──► DDR sample buffer (0x10000000, 32M samples)
                                           │ AXI DMA MM2S (32-bit stream: I low, Q high)
                                           ▼
                                   PL: AXIS FIFO (stand-in for the detector)
                                           │ AXI DMA S2MM
                                           ▼
PC ◄──Ethernet── ARM ◄──────────── DDR debug buffer (0x18000000)
```

### Build and run

```bash
cd Flexlink_py/HW_Development/Packet_Detector

# 1. Hardware (about 10 minutes)
cd HW && vivado -mode batch -source build_hw.tcl && cd ..
#    optional: inspect the block design   vivado HW/build/pd_zed/pd_zed.xpr

# 2. Firmware
cd FW && vitis -s build_firmware.py

# 3. Board: boot jumpers JP7-JP11 all to GND (JTAG), USB on PROG (J17) and UART (J14),
#    Ethernet cable PC <-> Zedboard, power on
xsdb run_board.tcl
#    UART at 115200 shows the board's IP (DHCP, or 192.168.50.10 on a direct cable).
#    PC side, once (enp1s0 = the wired port; 192.168.50.x keeps clear of a 192.168.1.x home network):
#      nmcli con add type ethernet ifname enp1s0 con-name zedboard ipv4.method manual ipv4.addresses 192.168.50.100/24
#      nmcli con up zedboard
#      ping -c 3 192.168.50.10

# 4. Tests from the PC, in this order
cd ../PC
python pd_hw_test.py ../TestVectors/Waveform_Payload_DefectsModeled_Tx.json --loopback-only   # Ethernet + DDR
python pd_hw_test.py ../TestVectors/Waveform_Payload_DefectsModeled_Tx.json --pl-loopback     # + DMA + PL
python pd_hw_test.py ../TestVectors/Waveform_Payload_DefectsModeled_Tx.json --pl-loopback --chunk 20000
```

`--loopback-only` passing but `--pl-loopback` failing points at the PL side (DMA, FIFO,
cache handling); a timeout or connection error points at the network or firmware - check the UART.
The board's IP can be given with `--host` (default 192.168.50.10, set by `BOARD_IP` in `FW/build_firmware.py`).

### Status

Phase 1 passed on the Zedboard (26 Oct 2026): `--loopback-only`, `--pl-loopback` and
`--pl-loopback --chunk 20000`, Vivado/Vitis 2025.2, direct cable to the PC at 1000 Mbps.

### Problems met during bring-up (all fixed in the scripts)

| Symptom | Cause | Fix |
| --- | --- | --- |
| `libncurses.so.5: cannot open shared object file` | Xilinx tools need ncurses 5; Ubuntu 24.04+ ships only 6 | symlink `libncurses.so.5` / `libtinfo.so.5` to the `.so.6` files |
| Board files not found in batch mode | Vivado Store boards are under `~/.Xilinx/...` | `build_hw.tcl` adds the store's `boards` folder (or set `PD_BOARD_REPO`) |
| BSP build fails in lwIP `init.c` | 64 KB send buffer needs more TCP segments than the default pool | `memp_n_tcp_seg = 1024` |
| "Invalid Template lwip_echo_server" | template requires DHCP, ACD check, PBUF pool 2048, interval timer | `TEMPLATE_CONFIG` in `build_firmware.py` |
| CMake: cannot find `echo.c` | template `CMakeLists.txt` lists sources by name | `echo.c` entry renamed to `pd_server.c` |
| Console stops after the link speed (DHCP never times out) | `XILTIMER_tick_timer` = None, so no 50 ms lwIP tick | `XILTIMER_tick_timer = ps7_ttc_0` |
| "Auto negotiation error" / "Ethernet Link down", but the PC shows a link | **MDIO on EMIO** in the board preset: PHY registers unreachable, the PHY links by itself at power-on | `PCW_ENET0_GRP_MDIO_IO = MIO 52 .. 53` in `build_hw.tcl` (checked) |
| Garbage numbers on the console | `xil_printf` has no `%lu` / `%lx` | `%d` / `%x` with casts |
| Board unreachable on a 192.168.1.x home network | address clash with Wi-Fi | board 192.168.50.10, PC 192.168.50.100 on the wired port |

Two harmless left-overs from the search: lwIP's Marvell auto-negotiation wait is raised from
5 s to 20 s (`PHY_WAIT_S`), and `LINK_SPEED` can force 100/1000 Mbps (keep `"auto"`: a forced
speed leaves lwIP waiting for an auto-negotiation flag that never comes).

## Phase 2: the detector

Step 1 done: **bit-true model** (`BITTRUE.md`, `PC/pd_reference.py`, `PC/pd_bittrue_check.py`).
The test bench now compares exactly, and test vectors come in two forms: LPF output
(`input_stage 0`) and decimated input (`input_stage 1`, the board runs the DC blocker and LPF).

Step 2 done: **detection lane RTL** (`RTL/`), from the decimated samples to `t_trig` and the
per-sample debug stream. It matches the bit-true model on every sample in Verilator: rep3, the
Tx defect file and a full-scale stress vector; both input stages; random `tvalid` gaps of up to 7
clocks; up to 532 triggers per run. Three injected faults were caught. xsim (Vivado 2025.2): stress vector
PASS. Out-of-context place & route on the Zynq-7020 (`HW/synth_detect.tcl`): **100 MHz met, WNS
+1.299 ns**; 2834 LUT (5%), 3026 FF (3%), 33 DSP48 (15%; 18 of them in the LPF), 1 BRAM tile.
Step 3 done: **CFO lane RTL** (`RTL/pd_cfo_lane.v`, `pd_cordic.v`, `pd_cfo_cascade.v`). It
matches the bit-true model in Verilator: four lag accumulators, four angles and `cfo_inc` for
every trigger. Vectors: 40 real packets at ±600 kHz, the stress vector and rep3. The unit tests
cover 22025 CORDIC vectors and 101000 cascade sets with exact rounding boundaries. 6 of 7
injected faults were caught; the 7th turned out to be logically equivalent. xsim passes too.
Place & route: **100 MHz met, WNS +0.856 ns**; 4782 LUT (9%), 5315 FF (5%), 49 DSP48 (22%),
1 BRAM tile.

Step 4 done: **tone-check RTL** (`RTL/pd_tone_check.v`, `pd_tone_rom.v`). After f̂ it replays the
CFO window from an 8192-sample buffer through four DFT units (13 bins in four passes) and
outputs one event per window: the CFO result plus tone `num`, `den` and the abort flag. It
matches the bit-true model in Verilator on every event of every vector: N_CFO 128 / 2048 /
4096, both input stages, one sample per clock up to 20 Msps. The unit test covers 160 windows
with the threshold exactly on the decision boundary; 8 of 9 injected faults were caught, and
the 9th is logically equivalent. At 20 Msps the input is never stalled. Faster input (DMA from
memory) is held back with `tready` while the check catches up; results are unchanged. xsim
passes too. Place & route of the whole detector: **100 MHz met, WNS +0.948 ns**; 7803 LUT (15%),
7438 FF (7%), 78 DSP48 (35%), 9 BRAM tiles (6%).

Step 5: **detector on the board** (this version). `RTL/pd_axi.v` replaces the loopback FIFO in
`HW/build_hw.tcl`: AXI-Lite registers at 0x43C00000 (parameters, control, status, event FIFO),
samples in from the DMA, debug records out to the DMA. Firmware build 3 writes SET_PARAMS to
the registers, reads the events while the DMA runs, converts them to event records and
reports a window that is still open at the end of a RUN as truncated, like the emulator.
The CFO lane now takes its samples from a release FIFO instead of a 32-sample delay, so a
window is complete as soon as its last sample has been processed, also at the end of a RUN.

Checked before the build in a co-simulation (`SIM/cosim/`): the real `pd_server.c` with the
Verilated `pd_axi` behind a TCP port. `pd_hw_test.py` passes against it on rep3 and the defects
file, both input stages, `--chunk` (windows across RUNs), `--debug` (every record exact, now
checked by `pd_hw_test.py`) and `--pl-loopback`; every reply matches `pd_board_emulator.py`
on six sequences with up to 269 events, truncated windows, slow S2MM and a full event FIFO.

```bash
cd HW && vivado -mode batch -source build_hw.tcl && cd ..        # new bitstream (pd_0)
cd FW && vitis -s build_firmware.py && xsdb run_board.tcl && cd ..
cd PC
python pd_hw_test.py ../TestVectors/Waveform_Payload_DefectsModeled_Tx.json --pl-loopback     # plumbing still OK
python pd_hw_test.py ../TestVectors/Waveform_Payload_DefectsModeled_Tx.json                   # detector
python pd_hw_test.py ../TestVectors/Waveform_Payload_DefectsModeled_Tx.json --input-stage 1 --debug --plot
python pd_hw_test.py ../TestVectors/rep3.json --input-stage 1 --chunk 50000
python pd_board_regression.py          # 33 cases vs the bit-true model, a few minutes
python pd_noise_rate.py --seconds 20   # false-trigger rate on noise (spec 3.3)
```

Step 6: **40 → 20 Msps half-band decimator in the PL** (`RTL/pd_halfband.v`, `input_stage 2`).
The board takes 40 Msps samples, as the radio delivers them. Bit-true Monte Carlo: same Pd and
CFO error as the simulator's `resample_poly` at every SNR; without a filter Pd at −12 dB falls
to 0.37 (`BITTRUE.md`, change 7). Verilator: the 40-packet CFO sweep and the stress vector at
40 Msps match the model; 5 of 5 injected faults caught. Co-simulation: `pd_hw_test.py
--input-stage 2` and the stage-2 regression cases pass. Bitstream ID 2.0, firmware build 4.
The test vectors need rebuilding once for the 40 Msps file (the existing files stay
byte-identical):

```bash
cd PC
python pd_make_vector.py --repeat 3 --seed 2 --out ../TestVectors/rep3
python pd_make_vector.py --seed 1                     # Waveform_Payload_DefectsModeled_Tx
python pd_hw_test.py ../TestVectors/rep3.json --input-stage 2 --debug
```

Step 7: **power floor** (`BITTRUE.md`, change 8). At very high SNR with the gain fixed for the
packet (+60 dB), the idle noise drops to about 0.5 LSB and the few isolated ±1 values that are
left can look periodic: 2 of 200 packets triggered early. A sample now has to have R0 and R_D
of at least 1024 (2 LSB rms on average) to count as above η_hi. Pd and every real-packet
result unchanged; +30 to +65 dB and noise-free all clean. New vector `highsnr` (RTL and board
regression), 5 of 5 injected faults caught.

Step 8: **Preamble B timing in the PL** (`RTL/pd_timing.v`, spec 4.5, `BITTRUE.md` change 9).
The 1-bit tap correlator finds the last Preamble B sample for every event that passed the
tone check (input stages 1 and 2): 228 lane units over 5 phases, one output per 5 clocks (real
time at 20 Msps), LUT RAMs instead of multipliers, a 4096-sample block-RAM buffer so the search
can start after the tone check. Each event gets a timing record: `n_end` (payload from
`n_end + 1`), `p_max`, `p_sum`, the peak/mean check against `T_PK` (18.0) and the late-trigger
check; an event inside a running search gets a BUSY record. Bit-true with 6-bit samples: same
correct / aborted counts as the simulator's float 1-bit search at −14 … −10 dB, 0 wrong
timings. Verilator: 12 vectors match the model record for record, 0 stalls at 20 Msps; a
directed test of the block's exact boundaries (`./run_sim.sh timing`) catches 19 of 19 injected
faults. Co-simulation (firmware build 10 + Verilated `pd_axi`): `pd_hw_test.py` at 20 and 40
Msps and 18 regression cases pass. Bitstream ID 3.0 (`T_PK` at 0x44, timing FIFO at 0x48 / 0x4C),
firmware build 10 (SET_PARAMS takes 40 bytes with `t_pk`; RUN replies carry the timing
records after the events, PING sets `BOARD_TIMING`). `pd_hw_test.py` and
`pd_board_regression.py` compare the records exactly when the board has them; older boards
still work, without timing.

```bash
cd HW && vivado -mode batch -source synth_detect.tcl && cd ..   # timing / resources first
cd HW && vivado -mode batch -source build_hw.tcl && cd ..
cd FW && vitis -s build_firmware.py && xsdb run_board.tcl && cd ..
cd PC
python pd_hw_test.py ../TestVectors/rep3.json --input-stage 2 --debug
python pd_board_regression.py          # 33 cases, timing records included
python pd_bittrue_check.py --only timing   # the float vs bit-true timing study (about 15 minutes)
```

Step 9: **control field in the PL** (`RTL/pd_cf.v` + `RTL/cf_*.v`,
`Docs/Specification_ControlField.md`, `BITTRUE.md` change 10). After every good timing record
the receiver decodes the 4 control-field symbols from `n_end + 1`: de-rotation with the event's
`cfo_inc`, its own 1024-point FFT, a fixed 8-tap channel filter with drift correction, soft
values, descrambling, 15-copy combining, Viterbi and CRC10, about 0.45 ms per packet. It reads
its samples from its own 8192-sample buffer of the DC blocker output, so it can start after the
timing search; at 20 Msps the input is never held back. Each good timing record gives a
control record: CRC result, the 54 message bits, Viterbi metric and input scaling. Bit-true:
the same packets decode as with the float chain (AWGN −14 / −13 dB, ETU, EVA); the simulator
has the same estimator (`ControlFieldEqualizerMode` FIR8, its default). The receiver alone
(`SIM/run_cf.sh`): 13 cases, every FFT output, filter output and soft value exact, 24 of 25
injected faults caught (the 25th is equivalent); out of context 100 MHz met, WNS +0.977 ns, 3125
LUT, 37 DSP, 15 BRAM tiles. In the detector (`./run_sim.sh`): rep3 matches record for record at
one sample per clock and at exactly 20 Msps (0 stalls); faults in the start sample, the stall
limit, the record filter, `busy` and the `cfo_inc` hand-over are caught. Co-simulation
(firmware build 11 + Verilated `pd_axi`): `pd_hw_test.py` and 11 regression cases pass.
Bitstream ID 4.0 (control FIFO at 0x50 / 0x54, STATUS bits 13 / 14), firmware build 11 (RUN
replies carry the control records after the timing records, PING sets `BOARD_CF`).
`pd_hw_test.py` prints the decoded fields (`PC/cf_fields.py`, with the simulator's parser) and
compares the message with the Tx one; older boards still work, without control records.

```bash
cd HW && vivado -mode batch -source synth_detect.tcl && cd ..   # timing / resources first
cd HW && vivado -mode batch -source build_hw.tcl && cd ..
cd FW && vitis -s build_firmware.py && xsdb run_board.tcl && cd ..
cd PC
python pd_hw_test.py ../TestVectors/rep3.json --input-stage 1
python pd_board_regression.py          # 33 cases, timing and control records included
```

Step 10: **re-arm by the Rx state machine** (`RTL/pd_arm.v`, `Params.rearm` = 1, the default;
`Docs/Specification_PayloadInput.md` §4, `BITTRUE.md` change 11). The detector stays disarmed
after a trigger until the packet's outcome is known and re-arms at a sample fixed by it: soon
after a failed tone check (2N + 256 samples), after a failed timing search, after a rejected
control field (CRC or a field code with no meaning), or at the end of a good packet
(PacketLength from the control field). The point is a sample index, so board and model agree
exactly; at 20 Msps the decision is always out in time (0 stalls), with faster DMA input the
input waits for it. A false trigger on noise now costs 0.22 ms instead of the 4.75 ms holdoff.
New vector `cfmod` (rep3 with an invalid field code, a failed CRC and a short PacketLength);
9 of 9 injected faults caught once the vectors had an invalid code they could see. Bitstream ID
5.0 (`REARM` at 0x58, STATUS bit 15 ARM_LATE), firmware build 12 (SET_PARAMS takes 44 bytes
with `rearm`; PING sets `BOARD_REARM`). The PC tools fall back to `rearm` 0 on older boards; the
regression keeps the fixed holdoff for the cases built for it and adds six state-machine cases.

```bash
cd SIM && ./run_sim.sh cfmod 1 && FIXGAP=1 GAP=4 ./run_sim.sh stress 1 && cd ..
cd HW && vivado -mode batch -source synth_detect.tcl && vivado -mode batch -source build_hw.tcl && cd ..
cd FW && vitis -s build_firmware.py && xsdb run_board.tcl && cd ..
cd PC && python pd_hw_test.py ../TestVectors/rep3.json --input-stage 1 && python pd_board_regression.py
```

Step 11: **shared fast FFT and speculative payload FFT** (`RTL/pd_fft2.v`, `cf_rx.v`,
`pd_cf.v`; `Docs/Specification_PayloadInput.md` §5, `BITTRUE.md` change 12). The control-field
FFT now does two butterflies per clock (2640 instead of 5200 clocks per symbol, +4 DSP, about
+6 BRAM36, identical results), and the same FFT carries on with the payload symbols straight
after the control field, while the control field is still being decoded (at most 8 symbols
ahead); the control record keeps PacketLength of them or drops them. Each payload symbol gives a
64-bit signature of its 841 bins in the RUN reply, so the board checks the payload FFT bit for
bit against the model (`pd_hw_test` "payload", regression column "payload"). Only with
`rearm` 1. At 20 Msps the control record comes 1600 samples sooner than before (3770 after the
payload start) and the last payload symbol is through the FFT about 700 samples after the packet
ends. Bitstream ID 6.0 (PL_LEVEL 0x5C, PL_DATA 0x60), firmware build 13 (`BOARD_PAYLOAD`).

```bash
cd SIM && ./run_cf.sh && ./run_sim.sh && ./run_sim.sh cfmod 1 && PLGAP=20000 ./run_sim.sh && cd ..
cd HW && vivado -mode batch -source build_hw.tcl && cd ..
cd FW && vitis -s build_firmware.py && xsdb run_board.tcl && cd ..
cd PC && python pd_hw_test.py ../TestVectors/rep3.json --input-stage 1 && python pd_board_regression.py
```

**Board regression** (`PC/pd_board_regression.py`): the RTL simulation's coverage on the board.
Each case runs on the board and, with the same commands, through `pd_board_emulator`'s logic
in-process; every RUN reply and every debug record must match. Cases: rep3 and the defects
file (both input stages), the 40-packet CFO sweep (N_CFO 2048 and 4096), the full-scale stress
vector (holdoff 1, N_CFO 128 / 4096, tone check off / 24 dB, DC blocker off / shift 4), uneven
RUNs with windows split across them, a window cut by the end of the data, and 1 s of noise
(false-trigger rate). Report in `TestVectors/regression_result.json`. Checked against the
co-simulation (all pass) and against the emulator's injected faults (both caught).


The AXIS FIFO in `build_hw.tcl` (`pl_core`) is replaced by the detector RTL with the same stream
ports plus an AXI-Lite port for the registers (spec 5.2) and the event FIFO. The firmware
then writes `SET_PARAMS` to those registers, RUN streams the samples through the detector
(debug records via S2MM), and reads the events back. The HDL gets an xsim testbench driven by
the same test vectors and checked against `PC/pd_reference.py` before it goes on the board.

## Known issue (fixed): corrupted samples on LOAD / READ

Firmware builds 4-8 replaced about 0.13 % of the samples with 0x404, 0x408, 0x624 or 0x628
(LOOPBACK FAIL in `pd_hw_test.py`). Cause: the 2025.2 standalone BSP's interrupt entry does not
save the VFP/NEON registers, and the Ethernet interrupt handler uses them (a `memcpy` of a
register-offset table in `XEmacPs_GetQxOffset`). An interrupt in the middle of a `memcpy` in the
main loop left those values in the copied data. `FW/build_firmware.py` now patches the
platform's `asm_vectors.S` to always save them, gives the IRQ stack 4 KB, and checks the built
ELF ("IRQ handler saves the VFP registers: OK"). Diagnostics kept for the future:
`PC/pd_load_diag.py`, `pd_memtest.py`, `pd_reg_diag.py`, `pd_bisect.py`, `pd_where.py`.
