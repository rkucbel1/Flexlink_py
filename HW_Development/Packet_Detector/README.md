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
```


The AXIS FIFO in `build_hw.tcl` (`pl_core`) is replaced by the detector RTL with the same stream
ports plus an AXI-Lite port for the registers (spec 5.2) and the event FIFO. The firmware
then writes `SET_PARAMS` to those registers, RUN streams the samples through the detector
(debug records via S2MM), and reads the events back. The HDL gets an xsim testbench driven by
the same test vectors and checked against `PC/pd_reference.py` before it goes on the board.
