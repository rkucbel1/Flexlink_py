# Packet detector – PC side of the Zedboard test bench

PC scripts for testing the FPGA packet detector (`Specification_PacketDetector.md`) on a Zedboard.
The PC builds a 12-bit test vector from a simulator `.h5`, sends it to the board over Ethernet,
the board stores it in DDR and streams it through the detector, and the PC compares the board's
events with the bit-true model (`pd_reference.py`, see `../BITTRUE.md`).

```
pd_make_vector.py  ──►  TestVectors/<name>.bin,        ──►  pd_hw_test.py  ──TCP──►  Zedboard (or pd_board_emulator.py)
  (decimation, 12-bit            <name>_dec.bin, .json         (load, loopback,           (ARM: lwIP server, DDR,
   quantisation, bit-true                                       run, compare, plot)        AXI DMA → detector in PL)
   front end and detector)
```

| File | Purpose |
| --- | --- |
| `pd_protocol.py` | Wire protocol: message header, commands, `Params`, `EventRecord`, debug record. The board firmware implements the same structures in C. |
| `pd_reference.py` | **Bit-true model** of spec §4–5: integer DC blocker and LPF, running sums, 18-bit comparator, qualifier/hysteresis, blanking, holdoff, CFO accumulators, CORDIC, integer cascade, integer tone check. The HDL must match it bit for bit. |
| `pd_bittrue_check.py` | Word-length sweeps and Monte Carlo of the bit-true model against the floating-point simulator (results in `../BITTRUE.md`). |
| `pd_make_vector.py` | `.h5` → idle padding → decimate (simulator code) → 12-bit I/Q → bit-true front end → expected events, plus a cross-check against the simulator's float chain. Writes `<name>.bin` (LPF output), `<name>_dec.bin` (decimated input) and, for a 40 Msps source, `<name>_40.bin` (40 Msps input for `--input-stage 2`, with its own expected events). |
| `pd_board_emulator.py` | TCP server that behaves like the board, running `pd_reference.py`. Use it until the board exists; it also defines what the firmware must do. |
| `pd_hw_test.py` | Client: PING, RESET, SET_PARAMS, LOAD, read-back check, RUN (optionally in chunks), GET_DEBUG, compare, plot. Exit 0 = PASS, 1 = FAIL, 2 = no connection. |
| `pd_noise_rate.py` | False-trigger rate on the board: long noise (the bit-true study's decimated noise, white noise, or a `.bin` file from your channel model) streamed continuously, rate with 95% interval per `eta_hi`, triggers passing the tone check. |
| `pd_board_regression.py` | Board regression: 21 cases (real packets, CFO sweep, stress vector with parameter variants, split RUNs, noise), each compared reply by reply and record by record with the emulator's logic run in-process. `--list`, `--only`, `--quick`. Exit 0 = all PASS. |

## Quick start (no board needed)

```bash
cd Flexlink_py/HW_Development/Packet_Detector/PC
python pd_make_vector.py --seed 1                         # Tx/Waveform_Payload_DefectsModeled_Tx.h5 -> ../TestVectors/
python pd_board_emulator.py &                             # emulated board on port 5001
python pd_hw_test.py ../TestVectors/Waveform_Payload_DefectsModeled_Tx.json --host 127.0.0.1 --plot
```

Useful variants:

```bash
python pd_make_vector.py --repeat 3 --out ../TestVectors/rep3          # three packets, tests re-arming
python pd_hw_test.py ../TestVectors/rep3.json --host 127.0.0.1 --chunk 33333   # state carried across RUNs
python pd_hw_test.py ../TestVectors/rep3.json --loopback-only           # Ethernet/DDR plumbing only
python pd_hw_test.py ../TestVectors/rep3.json --pl-loopback             # + DMA through the PL and back
python pd_hw_test.py ../TestVectors/rep3.json --host 127.0.0.1 --input-stage 1  # board runs DC blocker + LPF
python pd_board_emulator.py --port 5002 --fault tshift                  # check that mismatches are caught
```

## What is compared

| Field | Check | Why |
| --- | --- | --- |
| `t_trig`, `status`, `cfo_valid_cnt`, `cfo_inc`, lag angles | exact | bit-true integer datapath (`pd_reference.py`) |
| ToneCheck dB | ±0.01 dB | the abort decision (in `status`) is exact integer; the dB value is the firmware's log10 of two integers |

Vectors built before the bit-true model (no `_dec.bin`) still load, with a note: rebuild them.

## Conventions the HDL must follow

These are defined in the docstring of `pd_reference.py`; the most important:

* **Input**: `input_stage 0` – the board receives the LPF output (`<name>.bin`); `input_stage 1` – it receives the
  decimated stream (`<name>_dec.bin`) and runs the DC blocker (`dc_shift`) and LPF itself. Both give the same events.
* **sample_cnt** counts `tvalid` beats from RESET, not clocks, so the board may process faster or slower than 20 Msps.
* **Comparator**: common right shift so the largest of |Re P|, |Im P|, R₀, R_D fits 18 bits signed, arithmetic shift (floor),
  then `(Re'^2 + Im'^2) * 2^16 > ETA_SQ * R0' * RD'` with `ETA_SQ` in Q0.16.
* **Trigger**: the sample where the qualifier count reaches `QUAL_LEN`. Re-arms `arm_holdoff` samples later
  (a stand-in for the Rx FSM's `arm` pulse; `pd_make_vector.py` sets it to the packet length + 1024).
* **CFO window**: samples `t_trig … t_trig + N_CFO − 1` only; the lag-ℓ accumulator adds `x(n) x*(n − ℓ)` for
  `n = t_trig + ℓ … t_trig + N_CFO − 1` (as the simulator). The cascade is all-integer in NCO units.
* **Word lengths, CORDIC, tone check**: see the table in `../BITTRUE.md`.

## Requirements

Python 3.10+, numpy, scipy, h5py (vector builder only), matplotlib (`--plot` only).
The scripts find the project root by looking for `flexlink_paths.py` above this folder.

## Board side

See `../README.md`: Vivado Tcl, firmware and the bring-up order (`--loopback-only`, then `--pl-loopback`).
A phase-1 board reports "no detector" and accepts only those two modes.
