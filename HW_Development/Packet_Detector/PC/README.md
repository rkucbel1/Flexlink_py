# Packet detector – PC side of the Zedboard test bench

PC scripts for testing the FPGA packet detector (`Specification_PacketDetector.md`) on a Zedboard.
The PC builds a 12-bit test vector from a simulator `.h5`, sends it to the board over Ethernet,
the board stores it in DDR and streams it through the detector, and the PC compares the board's
events with a golden reference.

```
pd_make_vector.py  ──►  TestVectors/<name>.bin + .json  ──►  pd_hw_test.py  ──TCP──►  Zedboard (or pd_board_emulator.py)
  (simulator front end,                                         (load, loopback,           (ARM: lwIP server, DDR,
   12-bit quantisation,                                          run, compare, plot)        AXI DMA → detector in PL)
   golden reference)
```

| File | Purpose |
| --- | --- |
| `pd_protocol.py` | Wire protocol: message header, commands, `Params`, `EventRecord`, debug record. The board firmware implements the same structures in C. |
| `pd_reference.py` | Golden model of spec §4 on the 12-bit samples: integer running sums, 18-bit comparator, qualifier/hysteresis, blanking, holdoff, integer lag cascade, tone check. |
| `pd_make_vector.py` | `.h5` → idle padding → decimate, DC blocker, LPF (the simulator's own code) → 12-bit I/Q → expected events, plus a cross-check against the simulator's detector. |
| `pd_board_emulator.py` | TCP server that behaves like the board, running `pd_reference.py`. Use it until the board exists; it also defines what the firmware must do. |
| `pd_hw_test.py` | Client: PING, RESET, SET_PARAMS, LOAD, read-back check, RUN (optionally in chunks), GET_DEBUG, compare, plot. Exit 0 = PASS, 1 = FAIL, 2 = no connection. |

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
python pd_board_emulator.py --port 5002 --fault tshift                  # check that mismatches are caught
```

## What is compared

| Field | Check | Why |
| --- | --- | --- |
| `t_trig`, `status`, `cfo_valid_cnt` | exact | integer datapath, defined in `pd_reference.py` |
| `cfo_inc` | ±4 LSB (1 LSB = 1.19 Hz) | CORDIC not bit-true yet (reference rounds `atan2` to 16 bits) |
| lag angles | ±2 LSB | same |
| ToneCheck | ±0.5 dB | reference computes the 13 DFT bins in floating point |

Tighten the tolerances to zero once the bit-true model fixes the CORDIC and DFT word lengths.

## Conventions the HDL must follow

These are defined in the docstring of `pd_reference.py`; the most important:

* **Input**: `input_stage = 0` – the board receives the LPF output (DC blocker and LPF run on the PC for now).
  Moving them into the FPGA later needs bit-true integer versions in the reference first.
* **sample_cnt** counts `tvalid` beats from RESET, not clocks, so the board may process faster or slower than 20 Msps.
* **Comparator**: common right shift so the largest of |Re P|, |Im P|, R₀, R_D fits 18 bits signed, arithmetic shift (floor),
  then `(Re'^2 + Im'^2) * 2^16 > ETA_SQ * R0' * RD'` with `ETA_SQ` in Q0.16.
* **Trigger**: the sample where the qualifier count reaches `QUAL_LEN`. Re-arms `arm_holdoff` samples later
  (a stand-in for the Rx FSM's `arm` pulse; `pd_make_vector.py` sets it to the packet length + 1024).
* **CFO window**: `n = t_trig … t_trig + N_CFO − 1`, using `x(n − lag)` from the delay line (may reach before `t_trig`).
  The cascade is all-integer in NCO units: `f_lag = ang16 · 256/lag`, `step = 2^24/lag`.

Note: the simulator's `detect_frequency_offset_cascade` sums only inside the window (`x[lag:] · conj(x[:-lag])`),
so its estimate differs slightly (about 100 Hz on the −8 dB test file, well within the estimation noise).
Pick one convention and align the other.

## Requirements

Python 3.10+, numpy, scipy, h5py (vector builder only), matplotlib (`--plot` only).
The scripts find the project root by looking for `flexlink_paths.py` above this folder.

## Board side

See `../README.md`: Vivado Tcl, firmware and the bring-up order (`--loopback-only`, then `--pl-loopback`).
A phase-1 board reports "no detector" and accepts only those two modes.
