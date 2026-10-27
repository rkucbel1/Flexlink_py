# Packet detector – bit-true model

26 Oct 2026. Closes spec §7 "Bit-true model" and §8 item 5.

`PC/pd_reference.py` is now an integer model of the whole detector datapath, from the 12-bit
decimated samples to the event record. Each operation is defined exactly, including widths,
rounding, saturation and table contents. An HDL implementation that follows it reproduces its
outputs bit for bit. The board emulator, the test-vector builder and `pd_hw_test.py` all use it,
so the hardware test now compares `t_trig`, `status`, `cfo_inc` and the four CORDIC angles
**exactly**. ToneCheck in dB is compared to 0.01 dB; its abort decision is exact.

`PC/pd_bittrue_check.py` picks the word lengths and checks the model against the floating-point
simulator. Its results are below and in `BITTRUE_results.json`.

## Datapath and word lengths

| Stage | Definition | Widths |
| --- | --- | --- |
| Input | decimated, AGC-normalised I/Q, 20 Msps | 12-bit signed per rail |
| DC blocker | `acc(n) = acc(n-1) - rnd(acc(n-1) >> k) + ((x(n) - x(n-1)) << 10)`, `y = sat12(rnd(acc >> 10))`, k = `DC_SHIFT` (10), 0 = bypass | difference 13 b, shifted 23 b, accumulator 24 b signed (\|y\| ≤ 2·2048) |
| LPF | 17 taps, `z = sat12(rnd(Σ h_q[k]·y(n-k) >> 17))` | coefficients 18 b signed (`round(h·2¹⁷)`, max 36923); accumulator 30 b |
| Products | c₃₂ = x(n)x*(n−32), e = \|x\|² | 25 b signed |
| Running sums | P, R₀ over L = 256, R_D = R₀ delayed by 32 | 33 b, exact |
| Comparator | common floor shift so the largest of \|Re P\|, \|Im P\|, R₀, R_D fits 18 b signed; `(Re'² + Im'²) << 16` vs `ETA_SQ · R₀' · R_D'` | 18 b inputs; 52 b / 50 b compare |
| CFO accumulators | `A_ℓ = s_ℓ Σ_{n = t+ℓ}^{t+N−1} x(n) x*(n−ℓ)`, ℓ = 16/32/128/256 | 36 b (N = 2048), 37 b (N = 4096) |
| CORDIC | vectoring; normalise to 22 b magnitude, pre-rotate by π if x < 0, 16 iterations, `ATAN[i] = round(atan 2⁻ⁱ / 2π · 2²²)` | x, y 25 b signed; z 22 b + wrap; output 16 b |
| Cascade | integer in NCO units (Fs/2²⁴): `f_ℓ = ang·256/ℓ`, step 2²⁴/ℓ (shifts only) | 24 b; `cfo_inc = −f` |
| Tone check | 13 single-bin DFTs, phase accumulator 24 b, address = top 10 bits, cos/sin table 1024 × 16 b (amplitude 32767); common floor shift to 18 b before squaring | DFT accumulators 39 b; powers 35 b; num ≤ 40 b, den ≤ 39 b; 64-bit compare |
| Tone decision | `fail = (num << 16) < lin · den`, `num = 9(ΣP_tone − max)`, `den = ΣP_ref`, `lin = floor(10^(T/10)·2¹⁶ + 0.5)` | lin 24 b ⇒ `TONE_THRESH` 0–24 dB |

Rounding: `rnd(v >> k) = (v + 2^(k−1)) >> k`. All shifts are arithmetic. `sat12` clips to −2048…2047.

These confirm spec §5.3: 12 / 25 / 33 / 18 / 37 bits and a 16-bit angle. The other widths in the
table are new.

## Changes to the spec

1. **§4.3, CFO window: products stay inside the window.** The lag-ℓ accumulator sums
   x(n)x\*(n−ℓ) for n = t_trig+ℓ … t_trig+N_CFO−1. Each lag accumulator starts ℓ samples after
   the trigger, and that costs no hardware. Reaching back through the delay line before
   t_trig pairs Preamble A with the AGC burst, which biased the estimate by about 40 Hz at high
   SNR. With products kept inside the window, the model matches the simulator's
   `detect_frequency_offset_cascade`.
2. **§4.7, tone check: the abort decision is integer, the dB value is for reporting.** The
   hardware compares `num·2¹⁶` against `lin·den`. The firmware converts `TONE_THRESH` (dB, Q8.8)
   to `lin` and computes ToneCheck in dB from num and den. The limit is 24 dB. The lowest
   ToneCheck seen on a real packet was 14.4 dB at −14 dB SNR, against the 13 dB threshold.
3. **§4.2 / §4.6, reset: the DC blocker and LPF start from zero.** With a DC offset already
   present, the blocker settles over about 5·2^DC_SHIFT samples. During that time the detector
   triggers once, right after blanking (sample 415). The tone check rejects that trigger every
   time (50/50 below). Set `BLANK_LEN` ≥ 5120 after reset to avoid it, or leave the front end
   running across detector resets. In steady state, DC never triggers.
4. **§4.2, re-arm: the effective holdoff is at least `N_CFO`.** The detector re-arms
   `max(ARM_HOLDOFF, N_CFO, 1)` samples after a trigger, because there is one CFO lane. This is
   what spec §4.2 describes: "ignores further triggers until it is re-armed". `N_CFO` is
   128…4096; the hardware's CORDIC sequence needs about 95 clocks between windows.
5. **§5.1 input.** The model and the test bench accept either the LPF output
   (`input_stage 0`) or the decimated stream (`input_stage 1`, the board runs the front end). The
   half-band decimator 40 → 20 Msps (§8 item 4) is still floating point on the PC.
6. **§4.7 / §5.4, tone check: the window is replayed from a buffer.** f̂ is only known once the
   window has ended, so the hardware keeps the LPF stream in an 8192-sample buffer (6 BRAM36,
   N up to 4096) and runs the 13 bins over it afterwards: 4 DFT units, 4 passes, 16 DSPs. The
   result is ready about 4 N clocks after `cfo_valid`, before the next window can end at the real
   sample rate. The spec's estimate of two complex MAC units would not keep up with back-to-back
   windows. The decision and the integers are exactly those of the model.

## Verification

### Word-length sweeps (each block against its floating-point version)

| Block | Result | Chosen |
| --- | --- | --- |
| DC blocker | fractional bits 0 / 4 / 8 / 10: limit cycle 200 / 32 / 2 / **0** LSB, error 6.3 / 0.47 / 0.29 / 0.29 LSB rms | **10** (error = output rounding) |
| LPF | coefficient bits ÷2¹⁰ / ÷2¹⁴ / ÷2¹⁷: error 0.52 / 0.29 / 0.29 LSB rms, response deviation 0.07 / 0.006 / 0.002 dB | **÷2¹⁷** (18-bit, one DSP48 port) |
| CORDIC | iterations 12 / 14 / 16 / 18 (z 22 b): max angle error 5.5 / 1.8 / **0.82** / 0.58 LSB (0.5 is the final rounding itself) | **16**; 0.82 LSB ≈ 1 Hz at lag 256 |
| Tone check | table 2⁶ / 2⁸ / 2¹⁰ entries: max error 0.11 / 0.07 / **0.05** dB on 68 windows (packets at −12 dB and CW); 12-bit powers instead of 18: 0.6 dB | **2¹⁰ × 16 b, 18-bit powers** |

### Monte Carlo: bit-true vs float

Real packets (`Tx/Waveform_Payload_Tx.h5`), AWGN, CFO uniform ±500 kHz, 200 packets per SNR.
SNR is the preamble power over the noise power across the 40 MHz band. Thresholds: η_hi 0.35,
Q 128, N_CFO 2048, ToneCheck 13 dB. The float chain is the simulator's own modules. The
bit-true chain gets the same samples, quantised to 12 bits at −12 dBFS rms.

| SNR | Pd float / bit | late triggers | CFO rms float / bit | CFO p99 float / bit | CFO max bit | min ToneCheck | tone decisions agree |
| --- | --- | --- | --- | --- | --- | --- | --- |
| −14 dB | 0.815 / 0.820 | 42 | 1648 / 1685 Hz | 3597 / 3955 Hz | 4210 Hz | 14.4 dB | 164/165 |
| −13 dB | 0.970 / 0.970 | 15 | 1131 / 1128 Hz | 2958 / 2876 Hz | 3518 Hz | 16.3 dB | 194/194 |
| −12 dB | 1.000 / 1.000 | 0 | 908 / 913 Hz | 2302 / 2272 Hz | 2466 Hz | 18.9 dB | 200/200 |
| −10 dB | 1.000 / 1.000 | 0 | 604 / 604 Hz | 1351 / 1352 Hz | 1662 Hz | 20.0 dB | 200/200 |
| −6 dB | 1.000 / 1.000 | 0 | 255 / 255 Hz | 606 / 613 Hz | 769 Hz | 24.2 dB | 200/200 |
| 0 dB | 1.000 / 1.000 | 0 | 100 / 100 Hz | 258 / 258 Hz | 293 Hz | 31.8 dB | 200/200 |
| +10 dB | 1.000 / 1.000 | 0 | 25 / 25 Hz | 60 / 59 Hz | 66 Hz | 40.4 dB | 200/200 |
| +30 dB | 1.000 / 1.000 | 0 | 3 / 3 Hz | 7 / 6 Hz | 8 Hz | 55.9 dB | 200/200 |

"Late" means the CFO window runs past Preamble A, so the late-trigger rule (§4.3) aborts the
packet. No CFO error reached 5 kHz at any SNR (the §4.5 requirement). At −12 dB the spec had
0.89 kHz rms and 2.1 kHz p99; here it is 0.91 / 2.27 kHz.

### Noise only and interferers

| Test | Float | Bit-true | Passing the tone check |
| --- | --- | --- | --- |
| Noise, 5 s, η_hi 0.33 | 20 triggers | 20 triggers | 0 |
| Noise, 5 s, η_hi 0.35 | 5 triggers | 5 triggers | 0 |
| DC −6 … +30 dB re noise, steady state | 0 / 50 records | 0 / 50 records | – |
| DC, after reset (front end from zero) | – | 50 / 50 triggered | 0 (all rejected) |
| CW 1.1 MHz −10 … +30 dB | 50 / 50 triggered | 50 / 50 triggered | 0 (all rejected) |

**Resolved on the board (27 Oct 2026).** The false-trigger rate was the open point: about 1/s
at η_hi 0.35 here, against "< 0.2/s" in spec §3.3. `PC/pd_noise_rate.py` measured it on the
Zedboard, 20 s of the same decimated noise per threshold, streamed continuously:

| η_hi | Board | 95% interval | Passing the tone check |
| --- | --- | --- | --- |
| 0.33 | 3.5/s | 2.7 – 4.4 | 0 |
| 0.35 | 0.5/s | 0.24 – 0.92 | 0 |
| 0.40 | 0.05/s | 0.001 – 0.28 | 0 |

The spec's figure came from 0 triggers in 5 s, which only bounds the rate below about 0.74/s.
The board confirms this study's rates (0.33: 4/s here). Every false trigger fails the tone
check and costs one 2048-sample window (0.1 ms, 0.005% of the time at 0.5/s), so η_hi stays
0.35: 0.40 would cut Pd at −13 dB from 0.975 to 0.675. Proposed spec change: "about 0.5/s at
η_hi 0.35, all rejected by the tone check".

## Rerunning

```bash
cd Flexlink_py/HW_Development/Packet_Detector/PC
python pd_bittrue_check.py              # all four parts, about 10 minutes (the float detector is a Python loop)
python pd_bittrue_check.py --quick      # about 1 minute
python pd_bittrue_check.py --only mc    # or sweeps / noise / interferers
```

## Next: HDL

Build the detection front end first: DC blocker, LPF, products, running sums, comparator,
blanking/qualifier/holdoff → `t_trig` and `dbg_metric`. Run an xsim testbench fed from
`TestVectors/*_dec.bin` and compare its output with `pd_reference.detect(..., debug=True)` sample
by sample. Then add the CFO accumulators, the CORDIC, the cascade and the tone check.
