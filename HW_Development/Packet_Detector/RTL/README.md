# Packet detector RTL

Verilog-2001, bit-true to `../PC/pd_reference.py`. The checks are in `../SIM`.

## Steps 1–3: detection lane, CFO lane and tone check (this version)

| File | Block | Model function |
| --- | --- | --- |
| `pd_dc_blocker.v` | DC blocker, 10 fractional bits, 24-bit accumulator; `dc_shift` 0 = bypass | `dc_blocker` |
| `pd_lpf.v` | 17-tap LPF, symmetric (9 multipliers per rail), 18-bit coefficients | `lpf` |
| `pd_detect_core.v` | products, 256-sample running sums (BRAM history), R_D, 18-bit comparator, blanking / qualifier / holdoff, `t_trig` | `comparator_inputs`, `find_triggers` |
| `pd_div16.v` | pipelined divider for `dbg_metric` = min(65535, num / den) | debug metric |
| `pd_cfo_lane.v` | CFO lane: 32-sample input delay, window control, four lag accumulators (16 multipliers), CORDIC sequence | `cfo_accumulators` |
| `pd_cordic.v` | iterative vectoring CORDIC, 16 iterations, 22-bit angle accumulator; 21 clocks per angle | `cordic_angle` |
| `pd_cfo_cascade.v` | integer cascade 16 → 32 → 128 → 256 in NCO units, 3-clock pipeline | `cfo_cascade` |
| `pd_tone_check.v` | tone-structure check: 8192-sample window buffer, `NU` = 4 DFT units (13 bins in 4 passes), common shift, powers, integer threshold decision; one event per window | `tone_check`, abort decision in `detect` |
| `pd_sdp_ram.v` | simple dual-port block RAM (the tone check's 8192 × 24 sample buffer), kept as its own hierarchy so it stays block RAM | – |
| `pd_tone_rom.v` | cosine table 1024 × 16 bit, two read ports (cos and sin = cos − 256); **generated** by `gen_tone_rom.py` from `pd_reference.TONE_COS` | `TONE_COS` |
| `pd_detect_top.v` | `input_stage` 0: LPF output in; 1: decimated input, front end here | `detect(..., debug=True)` |

**Interface.**
- Input is one complex sample per `s_iq_tvalid` && `s_iq_tready` beat, I in `[11:0]` and Q in `[27:16]`, the same layout the PC and the DMA use.
- Throughput is at most one sample per clock. Gaps in `tvalid` are allowed, and all state changes only on accepted samples, so results depend on sample order, not on timing.
- Back-pressure: `tready` stays 1 at the real sample rate (one sample per 5 clocks or slower; checked with N_CFO 4096 and back-to-back windows). With faster input, it goes low while a check is running and a CFO result is waiting, or when the buffer would overwrite a window not yet checked.
- Outputs:
  - `trig`: a one-clock pulse, with `t_trig` = the index of the triggering sample, counted from reset.
  - `lpf_*`: the detector input.
  - A debug stream with one record per sample: `dbg_metric` and `dbg_bits` = {blank, trig, armed, below_lo, above_hi}.
- Latency from input to debug record: about 31 clocks (front end 8, core 12, divider 17).
- CFO result: `cfo_valid` pulse with `cfo_t` (= `t_trig`), `cfo_inc` (NCO increment for −f̂, LSB Fs/2²⁴), the four 16-bit angles and the four lag accumulators.
  - The CFO lane works on the LPF stream delayed by 32 samples, so the trigger for sample t is known before sample t reaches it.
  - A window is therefore complete 32 samples after its last sample, and the result follows about 95 clocks later: 4 angles × 21 clocks, plus the cascade.
- One CFO window at a time. The detection core re-arms only `max(arm_holdoff, n_cfo, 1)` samples after a trigger; the model does the same.
- Event: `ev_valid` pulse, one per window, about 4 N + 40 clocks after `cfo_valid`, in order. It carries `ev_t`, `ev_cfo_inc`, the four angles, `ev_tone_num` = 9 (ΣP_tone − max P_tone), `ev_tone_den` = ΣP_ref and `ev_tone_fail` = `tone_lin` ≠ 0 and num·2¹⁶ < `tone_lin`·den. The firmware turns num and den into ToneCheck in dB (`tone_db`) and fills the event record.
- Reset is synchronous and active low (`rst_n`). It clears all state; the history RAM is masked until it holds 256 samples.

**Parameters.** `dc_shift`, `eta_hi_sq`, `eta_lo_sq`, `qual_len`, `blank_len`, `arm_holdoff`, `n_cfo` (128…4096, default 2048) and `tone_lin` (threshold as a linear ratio in Q.16, `tone_thresh_lin`: 13 dB → 1307615, 0 = off; 24 dB maximum) are ports, constant while samples flow. They become AXI-Lite registers when the block goes into the Zedboard design.

**`pd_tone_check` parameter `NU`** (DFT units, default 4) trades DSPs for time: 4 multipliers per unit, ⌈13 / NU⌉ passes of N clocks per check. NU = 1, 3, 4 and 13 all pass bit-true. Real time (no stalls at 20 Msps) needs ⌈13 / NU⌉ < 5, so NU ≥ 3; NU = 3 has no margin, which is why the default is 4.

**Next step.** The AXI wrappers: AXI-Lite registers, an event FIFO, and the debug stream to the S2MM DMA. The block then replaces `pl_core` in `../HW/build_hw.tcl`.
