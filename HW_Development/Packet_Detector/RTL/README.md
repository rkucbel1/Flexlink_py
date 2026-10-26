# Packet detector RTL

Verilog-2001, bit-true to `../PC/pd_reference.py`. The checks are in `../SIM`.

## Step 1: detection lane (this version)

| File | Block | Model function |
| --- | --- | --- |
| `pd_dc_blocker.v` | DC blocker, 10 fractional bits, 24-bit accumulator; `dc_shift` 0 = bypass | `dc_blocker` |
| `pd_lpf.v` | 17-tap LPF, symmetric (9 multipliers per rail), 18-bit coefficients | `lpf` |
| `pd_detect_core.v` | products, 256-sample running sums (BRAM history), R_D, 18-bit comparator, blanking / qualifier / holdoff, `t_trig` | `comparator_inputs`, `find_triggers` |
| `pd_div16.v` | pipelined divider for `dbg_metric` = min(65535, num / den) | debug metric |
| `pd_detect_top.v` | `input_stage` 0: LPF output in; 1: decimated input, front end here | `detect(..., debug=True)` |

**Interface.**
- Input is one complex sample per `s_iq_tvalid` beat, I in `[11:0]` and Q in `[27:16]`, the same layout the PC and the DMA use. There is no back-pressure: `tready` is always 1.
- Throughput is at most one sample per clock. Gaps in `tvalid` are allowed, and all state changes only on valid samples, so results depend on sample order, not on timing.
- Outputs:
  - `trig`: a one-clock pulse, with `t_trig` = the index of the triggering sample, counted from reset.
  - `lpf_*`: the detector input.
  - A debug stream with one record per sample: `dbg_metric` and `dbg_bits` = {blank, trig, armed, below_lo, above_hi}.
- Latency from input to debug record: about 31 clocks (front end 8, core 12, divider 17).
- Reset is synchronous and active low (`rst_n`). It clears all state; the history RAM is masked until it holds 256 samples.

**Parameters.** `dc_shift`, `eta_hi_sq`, `eta_lo_sq`, `qual_len`, `blank_len` and `arm_holdoff` are ports, constant while samples flow. They become AXI-Lite registers when the block goes into the Zedboard design.

**Next steps.**
1. The CFO lane: four lag accumulators with products kept inside the window, the CORDIC and the cascade. Then the tone check.
2. The AXI wrappers: AXI-Lite registers, an event FIFO, and the debug stream to the S2MM DMA. The block then replaces `pl_core` in `../HW/build_hw.tcl`.
