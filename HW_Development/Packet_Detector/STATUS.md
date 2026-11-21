# Packet Detector: CoreBoard streaming status (handover)

Last updated 2026-11-12. Firmware **build 39**, bitstream **9.2** (`PD_ID` 0x50440902).

## What works

CoreBoard → TP-Link TL-SG1005D (Gigabit) → Zedboard → PC, all on 10.42.0.x:

| Device | Address | Role |
|---|---|---|
| PC | 10.42.0.1 | DHCP for the subnet, runs the scripts (and the emulator) |
| Zedboard | 10.42.0.101 | TCP port 5001 (commands, results), UDP port 4000 (ESR2 stream) |
| CoreBoard (ESP32-S31, ESP-SDR) | 10.42.0.133 (DHCP) | HTTP `/api/rx` on port 80, streams ESR2 to whoever sent `start` |
| Pluto | - | transmits FlexLink at 2440 MHz |

The Zedboard sends the HTTP start and stop to the CoreBoard itself. The CoreBoard streams int8 I/Q at
40 Msps (59,524 datagrams/s, about 670 Mbit/s). The board writes the datagrams into a ring by sequence
number and feeds the FPGA receiver through the AXI DMA. Decoded transport blocks are **pushed** to the
PC over TCP as they come.

### Results

| Test | Lost datagrams | Transport blocks | BLER | Good data |
|---|---|---|---|---|
| CoreBoard + Pluto, 30 s, `--push` | 3 of 1.77 M | 153,528 / 153,528 CRC ok | 0 | 8.15 Mbit/s |
| Emulator, 30 s, `--push` | 1 of 1.79 M | 110,808 / 110,808 CRC ok | 0 | 5.88 Mbit/s |
| Emulator, 120 s, polled (build 38) | 111 of 7.14 M | 415,194 / 415,196 | 4.8e-6 | 5.5 Mbit/s |

The data rate depends on how often the Pluto transmits: 8.15 Mbit/s when it transmits continuously.

Board load while streaming: CPU about 45 % (UDP receive incl. FIFO drains), FIFO drains about 12–16 %,
ring at most about 120–240 datagrams (2–4 ms).

### Latency (push mode)

Time from a packet's trigger until a transport block leaves the board:

| | Median | 99 % | Max | FPGA alone (xsim) |
|---|---|---|---|---|
| First block of a packet | 3.0 ms | 3.7 ms | 4.0 ms | 1.31 ms |
| Last block of a packet | 6.7 ms | 7.4 ms | 7.7 ms | up to 4.92 ms |

The FPGA figures include the packet's own air time. It delivers one block about every 107 µs, as fast
as the symbols arrive. The board adds about 1.7 ms (median), under 3 ms at most:

- Ring: about 1.1 ms (DMA chunks of 64 datagrams, newest 4 held back).
- FIFO drain: every 500 µs.
- Push interval: at most 1 ms.

The transfer to the PC adds a few hundred µs; it isn't measured, since the clocks aren't synchronised.

Ways to lower it, none implemented yet:

- `--push-ms 0.25` saves about 0.4 ms.
- Smaller DMA chunks and less hold-back would cut most of the 1.1 ms of the ring.

## How to run

From `HW_Development/Packet_Detector/PC`:

```
# real CoreBoard (start the Pluto when told)
python pd_stream_test.py --host 10.42.0.101 --coreboard 10.42.0.133 --seconds 30 --push

# emulator on the PC (plays App/CoreBoard/EmuSource.cs8)
python pd_stream_test.py --host 10.42.0.101 --seconds 30 --push

# FPGA receiver regression (no CoreBoard needed; normal LOAD/RUN path)
python pd_board_regression.py --host 10.42.0.101
```

- Radio settings come from `App/Parameters_CoreBoard.json` (2440 MHz, 40 Msps, gain 40).
- Without `--push` the script polls every 0.05 s. That still works, but each poll occasionally costs a
  datagram and adds 25–75 ms of latency.
- Build firmware with `FW/build_firmware.py`, then load with `run_board.tcl`.
- Build the bitstream with `HW/build_hw.tcl`.
- Simulate with `SIM/run_sim.sh ../TestVectors/coreboard.json 2`. It prints the trigger → transport
  block latency.

## Settings that matter

| Setting | Value | Why |
|---|---|---|
| Input | 40 Msps, input stage 2 (half-band in the FPGA) | CoreBoard rate |
| `CTRL.PACK8` (bit 3) | on while streaming | two int8 samples per DMA word, `pd_unpack8.v` (9.1) |
| `CTRL.NOREC` (bit 4) | on while streaming, unless `--keep-pl` | FPGA discards payload / channel / decoder-input records (9.2) |
| `dc_shift` | 6 | CoreBoard DC wander; 6 decodes everything, lower values trigger falsely |
| FIFO drain | every 500 µs (NOREC), 50 µs without | the 16-record debug FIFOs needed 50 µs |
| Reply pacing | one TCP segment per 100 µs (`--pace-us`) | see lessons |
| Push interval | at most one push per 1 ms (`--push-ms`) | statistics every 100 ms |
| TB result buffer | 384 KB on the board | about 0.5 s of results |

### STREAM_START flags (u16)

| Bits | Meaning |
|---|---|
| 0 | `KEEP_PL`: also return payload, channel-estimate and decoder-input records |
| 1 | `PUSH`: the board sends `STREAM_PUSH` (21) messages instead of being polled (build 39) |
| 6:2 | push interval in 250 µs units (0 = 1 ms) |
| 7 | `NOPACE`: send replies in bursts (test only; causes losses) |
| 15:8 | pacing interval in 25 µs units (0 = 100 µs) |

A `STREAM_PUSH` payload is:

1. A header: u32 flags (bit 0 = statistics follow), u32 0, u64 newest datagram slot (the stream
   position when the push left).
2. The statistics block, if flagged (`pd_protocol.StreamStats`).
3. The records, in the same layout as `POLL_RESULTS`.

The parser is `pd_protocol.parse_push()`. A command sent while a push is going out waits in lwIP for up
to 250 ms (`recv_cb` returns `ERR_MEM`).

## Lessons (don't relearn these)

1. **The Zynq Ethernet controller has a single DMA engine for send and receive.**
   - Replies sent in bursts of back-to-back frames made it overrun on the 670 Mbit/s incoming stream:
     about one lost datagram per 7 KB sent (BLER ~9e-3).
   - Pacing to one segment per 100 µs fixed it (build 35–38).
   - Even 3 segments back to back cost frames (build 37).
   - A paced segment must be due-checked from the UDP receive path, not only from the main loop, or
     the send rate falls short.
2. **Polling cost datagrams.** About one per few polls, from the request and ACK frames arriving in the
   stream. Push mode (build 39) removed it: 1–3 lost per 30 s.
3. **Not the cause** of the losses (tested and ruled out):
   - DDR port priorities (build 28)
   - CPU load and AXI-Lite register reads (builds 30–32)
   - DMA burst length (already 16)
   - PC emulator pacing
4. **AXI-Lite reads cost about 0.76 µs each.** Reading the FPGA FIFOs word by word was 30 % of the CPU.
   Reading each FIFO level once (build 31) and NOREC (9.2) halved it. The transport-block words
   themselves (~190k reads/s) are what remains.
5. **The receiver's sample counter is 32 bits** and wraps every 107 s at 40 Msps; the PC script unwraps
   it.
6. **About 230 GEM overruns, ~565 "pool empty" and ~770 "no-buffer"** appear in every run, whatever
   changes. They are most likely at stream start or stop and cost nothing visible.
7. **Paced sending reaches at least about 12 Mbit/s.** Results at 8–9 Mbit/s leave only about 1.4×
   headroom, so a faster waveform may need a faster pacing interval.
8. **FPGA RTL fix (`pl_chest.v`):** a control-field decision arriving before payload symbol 0's first
   bin queued a packet with a stale base and stalled the input. Only input pauses triggered it. Fixed,
   and the SIM testbench has burst / pause options (`BURST`, `BURSTGAP`, `BURSTOFF`) and a deadlock
   guard.

## Files

| File | What |
|---|---|
| `FW/pd_server.c` | Zedboard firmware (lwIP raw, bare metal): commands, UDP ring, DMA feed, drains, paced TCP, push, HTTP client |
| `FW/build_firmware.py` | builds the Vitis platform and app; patches the lwIP RX alloc printf into a counter |
| `RTL/pd_axi.v` | AXI-Lite registers, CTRL bits incl. PACK8 / NOREC, ID 9.2 |
| `RTL/pd_unpack8.v` | int8 → 12-bit input unpacker (9.1) |
| `RTL/pl_chest.v`, `RTL/pd_cf.v` | the early-decision fix |
| `SIM/tb_pd_detect.v` | testbench: bursts, deadlock guard, latency report |
| `PC/pd_stream_test.py` | streaming test: emulator or CoreBoard, poll or push, losses, CPU, latency |
| `PC/pd_protocol.py` | protocol: commands, StreamStats, `parse_poll`, `parse_push` |
| `PC/pd_replay_test.py` | replays a capture through LOAD/RUN, with zero-filled gaps and pauses |
| `App/FlexlinkCoreBoardEmulator.py` | CoreBoard emulator: HTTP `/api/rx` plus ESR2 stream from a `.cs8`, paced every 8 datagrams |

## Open items

- [ ] `pd_board_regression.py` on build 39 / bitstream 9.2 (last run: build 27, 42/42 pass).
- [ ] Git commit of all of the above.
- [ ] Optional: lower latency (`--push-ms 0.25`, smaller DMA chunks).
- [ ] Optional: a bigger result buffer (needs smaller debug-record buffers) for more headroom.

## Next: video streaming

- **Capacity:** plan around **~8 Mbit/s** of transport-block payload (Pluto transmitting continuously),
  less some margin.
- **Video quality:** 1080p30 with H.264 or 1080p60 with H.265 at low-latency encoder settings.
- **Latency:** the radio link adds about 3–7 ms per packet, trigger to block on the board.
- **Losses:** a lost block will be rare at this BLER but must be handled. Use periodic intra refresh,
  and possibly light FEC.
- **Results interface:** pushes on the TCP connection (`STREAM_PUSH`), one transport-block record per
  code word: `t_trig`, code word index, CRC flag, bits.
