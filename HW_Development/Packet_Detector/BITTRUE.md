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

7. **§8 item 4, front-end decimation: a 43-tap half-band in the PL** (`input_stage 2`). The
   board can now take the radio's 40 Msps stream. `pd_reference.halfband`: causal, 18-bit
   coefficients (exact unity DC gain), one output per input pair after its second sample;
   passband ±8.2 MHz with 0.0065 dB ripple, stopband from 11.8 MHz at 62.7 dB (the level of
   12-bit quantisation at −12 dBFS). 11 multipliers per rail. It delays the stream by 10.5
   samples at 20 Msps, so `t_trig` comes 10 samples later than with the simulator's
   zero-phase `resample_poly`. Bit-true Monte Carlo on the same trials (200 per SNR,
   `pd_bittrue_check.py --only decim`):

   | SNR | `resample_poly` (float, as before) | half-band (bit-true) | every other sample, no filter |
   | --- | --- | --- | --- |
   | −14 dB | Pd 0.825, CFO 1370 Hz rms | Pd 0.825, CFO 1372 Hz rms | Pd 0.075 |
   | −13 dB | Pd 0.990, CFO 1192 Hz rms | Pd 0.990, CFO 1194 Hz rms | Pd 0.140 |
   | −12 dB | Pd 1.000, CFO 889 Hz rms | Pd 1.000, CFO 889 Hz rms | Pd 0.370 |
   | −10 dB | Pd 1.000, CFO 595 Hz rms | Pd 1.000, CFO 595 Hz rms | Pd 0.985 |

   The half-band costs nothing. Without a filter the folded noise costs far more than the
   3 dB estimated in §8: at −12 dB only 37% of the packets are detected.

8. **§4.2, power floor: near-silent input can't trigger.** With the gain fixed for the packet
   and very high SNR (+60 dB at −12 dBFS), the idle noise is about 0.5 LSB rms: after the
   filters the detector sees mostly zeros and a few isolated ±1 values, and the normalised
   ratio |P|²/(R0·R_D) of such patterns is meaningless. Two −1s 32 samples apart look
   perfectly periodic and stay in the 256-sample sums long enough to pass the qualifier. In
   200 packets at +60 dB this triggered 2 times about 1000 samples before Preamble A; the
   window still reached the preamble, so the tone check passed them. Now a sample with
   R0 < E_MIN or R_D < E_MIN (exact sums, E_MIN = 1024: an average of 4 LSB², 2 LSB rms,
   about 48 dB below a packet at −12 dBFS) never counts as above η_hi and always counts as
   below η_lo; debug bit 0x20 marks it. A real AGC keeps the idle noise well above this.

   | SNR | idle noise (40 Msps) | without floor | with floor |
   | --- | --- | --- | --- |
   | −14 / −12 dB | 355 LSB rms | Pd 0.835 / 1.000 | Pd 0.835 / 1.000 |
   | +30 … +55 dB | 13 … 0.8 LSB rms | Pd 1.000, 0 early | Pd 1.000, 0 early |
   | +60 dB | 0.47 LSB rms | Pd 0.990, 2 early | Pd 1.000, 0 early |
   | +65 dB, noise-free | ≤ 0.3 LSB rms | Pd 1.000 | Pd 1.000 |

   All real-packet vectors give identical events. In the stress vector the floor removes two
   false triggers on near-silence (the DC blocker's decaying tail, about 1 LSB rms); the
   periodic triggers after them shift by 2 samples through the holdoff.

9. **§4.5, Preamble B timing: the 1-bit tap correlator is in the model and the PL** (input
   stages 1 and 2; it needs the unfiltered stream). `pd_reference.timing_records`, RTL
   `pd_timing.v`. For each event that passed the tone check:

   | Step | Definition |
   | --- | --- |
   | Window | positions n = t + N + 1099 … t + 6163 for the last Preamble B sample (§4.5: latency 0 … L_A − N, D_LPF = 8, guard 32) |
   | Input | DC blocker output y(n0 … hi), n0 = lo − 1139, same sample index as t_trig |
   | De-rotator | φ(n0) = 0, φ += cfo_inc (24 bit); tone-check table, a = φ[23:14]; `xr = sat6(rnd((yi·c − yq·s) >> 21))`, `xi = sat6(rnd((yi·s + yq·c) >> 21))` |
   | Taps | signs of Re and Im of the 20 Msps Preamble B (`Preamble.h5`), 1140 each, stored as constants |
   | Output | Y(n) = Σ x(n−1139+j)(s_I(j) − j s_Q(j)) exact (18 bit), P = \|Y\|² (34 bit), first maximum, Σ P (48 bit) |
   | Decision | PEAK_OK: `p_max > 0` and `p_max · n_pos · 256 ≥ T_pk · p_sum` (T_pk Q8.8, 18.0 default, 0 = off; a silent window has no peak); LATE: `n_end < t + N + 1131` (§4.3) |
   | One search at a time | an event whose first sample n0 is not after the window end of the last search started gets a BUSY record; a failed tone check gets no record |

   Six bits per rail are enough: on the same trials as the simulator's float 1-bit search
   (200 per SNR, its own detection, CFO and tone check, AWGN, CFO ±500 kHz), with T_pk 18 and
   the late-trigger rule:

   | SNR | float 1-bit: correct / wrong / aborted | bit-true, 6-bit samples | bit-true, 4-bit samples |
   | --- | --- | --- | --- |
   | −14 dB | 120 / 0 / 25 | 121 / 0 / 23 | 121 / 0 / 23 |
   | −13 dB | 180 / 0 / 12 | 179 / 0 / 10 | 179 / 0 / 10 |
   | −12 dB | 200 / 0 / 0 | 200 / 0 / 0 | 200 / 0 / 0 |
   | −10 dB | 200 / 0 / 0 (min peak/mean 98.1) | 200 / 0 / 0 (98.3) | 200 / 0 / 0 (92.9) |

   The rest of the 200 were not detected (bit-true and float detectors differ by a few packets
   at −14 dB, as in the Monte Carlo below). Without Preamble B in the window (300 noise
   records, random CFO), peak/mean reached at most 16.2 (p99 13.2, median 8.6) with 6-bit
   samples, as with 12: T_pk = 18 still sits in the gap. 12-bit and 8-bit samples gave the
   same counts as 6; 4-bit loses about 3 % of the peak/mean, 3-bit 10 %.

   Hardware (spec §4.5 table, K = 5 at 100 MHz): 228 lane units, one phase of 228 taps per
   clock, an output every 5 clocks, which is real time at 20 Msps. The 1140 de-rotated samples
   sit in 228 LUT RAMs (A = xr + xi and B = xr − xi, 7 bits each), so each tap's term is ±A or
   ±B selected by its two signs: no multipliers, and a negation is an inversion whose +1s are
   a constant (599 for Re, 576 for Im). Two 256-input adder trees, 4 DSPs in the de-rotator
   and 2 for |Y|². The samples come from a 4096-sample buffer (block RAM), because the search
   only starts once the tone check has passed, about 0.8 N samples after the window that it
   needs began; a fill at one sample per clock catches up. The spec's 256-sample FIFO is not
   enough for that.

   Results go to a timing FIFO (pd_axi 3.0) and the RUN reply (firmware build 10). The RTL
   testbench compares every record with the model: 12 vectors (all stages, N_CFO 128 / 2048 /
   4096, holdoff 1 … 95044, real time with 0 stalls) match. A directed test of the block alone
   (`tb_pd_timing.v`, `make_tm_vectors.py`) hits the exact boundaries real vectors miss:
   rounding at exactly .5, the late and one-search-at-a-time limits, saturation, silent windows.
   19 of 19 fault-injection mutants of `pd_timing.v` are caught. A silent window (all P = 0)
   never passes the peak check; that rule came out of the directed test.

10. **Control field** (`PC/cf_reference.py`, tables `PC/cf_tables.py`; spec
   `Docs/Specification_ControlField.md` §4.9). For every timing record with status TM_GOOD, in
   order, the 4560 samples y(n_end + 1 …) of the DC blocker output are de-rotated (12 bit, phase
   0 at n_end + 1), the four symbols go through a fixed-point 1024-point FFT (18-bit data and
   twiddles, rounding ÷2 per stage), the channel comes from a fixed 8-tap LMMSE filter over the
   Ref_P0 pilots of symbols 0 and 3 (Q14 coefficients) with the common drift removed and put
   back by s·φ/3, and the soft values Re(Y·H*) are descrambled, combined over the 15 copies,
   de-interleaved, scaled to 5 bits (a shift per packet) and Viterbi-decoded (ties to 0); CRC10
   over the 64 bits. Float vs bit-true, 150 packets per case after the bit-true detector and
   timing: AWGN −14 dB 81 / 81, −13 dB 128 / 128, ETU-300 −10 dB 129 / 129, EVA-70 −12 dB
   104 / 104 decoded; 4-bit Viterbi inputs lose 5 at −14 dB, 3-bit 42. No wrong message passed
   the CRC. `detect()` returns them in `Result.control` (`control=False` skips the decoding);
   a control field the samples end inside is not reported.

11. **Re-arm by the Rx state machine** (`Params.rearm` = 1; `pd_reference.rearm_point`,
   `triggers_from`). After a trigger the detector stays disarmed until the packet's outcome is
   known and re-arms at a sample R that only depends on the outcome (table in
   `Docs/Specification_PayloadInput.md` §4): t + 2N + 256 after a failed tone check,
   t + 2N + 6656 after a timing record that is not good, n_end + 2N + 12753 after a rejected
   control field, the end of the packet after a good one. `detect()` now runs the event,
   timing and control field of each trigger before it looks for the next one. `rearm` = 0 is
   the old fixed holdoff and gives the old results unchanged.

12. **Shared fast FFT, speculative payload FFT** (`cf_reference.payload_records`,
    `grid_symbol`, `grid_signature`; RTL `pd_fft2.v`, `cf_rx.v`, `pd_cf.v`; bitstream 6.0,
    firmware build 13). The FFT is unchanged in its arithmetic (`fft_fixed`; `pd_fft2` does two
    butterflies per clock in a different order, same results, `SIM/tb_fft2.v`). After the control
    field the de-rotator and FFT carry on with the payload: payload symbol s is stream symbol 4 + s,
    window `1140 (4 + s) + 104 … + 1127` after `n_end`, the same phase ramp continued (the
    simulator's `FFT_StartTime` −12 for the payload too). Kept only for a control record with CRC ok
    and every field code valid, with `Params.rearm` (packets cannot overlap), s = 0 … PacketLength − 1,
    and only symbols whose window lies inside the data. Each symbol's 841 bins (natural order,
    column c = bin (c + 604) mod 1024) give a Fletcher-style signature: x = re0, im0, re1, im1, …
    (each mod 2^18), A = Σ x, B = Σ of the running sums, both mod 2^32. That checks the payload
    FFT bit for bit on the board before the rest of the payload chain exists.

13. **Payload receiver bit-true model** (`PC/pl_reference.py`, tables `PC/pl_tables.npz` from
    `gen_pl_tables.py`, checks `PC/pl_check.py`; `Docs/Specification_PayloadInput.md` §6b).
    It starts from the payload grid of change 12 plus the control-field settings and ends with
    the transport blocks and their CRC24 results. The steps are: channel estimate
    (window-select FIR8, drift, smoothing, linear time), noise, max-log LLRs as integers,
    descrambling, copy combining, de-interleaving, the frozen layered decoder of §5b, and CRC24.
    No RTL yet.

14. **Payload channel estimate in the PL, and its records** (`pl_reference.chest_records`;
    RTL `pl_chest.v`, `pl_roms.v`, `pl_mulseq.v`, `pd_cf.v`; bitstream 7.0, firmware build 14).
    The channel-estimate part of change 13 (packet shift k, pilots, delay class, SNR mode, FIR8,
    drift, smoothing, time interpolation) runs on the board. Its delay-profile FFTs use the
    control-field FFT between symbols (`cf_rx` job port). Per accepted packet the RUN reply gets
    one packet record (sym 0x8000; sig_a = k | class << 8 | hi << 12 | n0 << 16, sig_b = ΣN mod
    2^32), then per payload symbol the signature of H(s) (as change 12, 841 values sign-extended
    to 18 bit). H(s) is reported once the reference rows it needs are in the stream, so a packet
    cut by the end of the data gives fewer records; the model pads missing symbols with zeros
    and stops where the stream stops. The control-field check is stricter (also `cf_rx`,
    `pd_arm`, `cf_reference.fields_valid`): PacketLength 4 needs RefSymbolSpacing 1 or 3 (else
    there are fewer than two reference rows), and NoOfTxAntenna must be 1. Such packets are
    now rejected and re-armed at once.

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
python pd_bittrue_check.py --only mc    # or sweeps / noise / interferers / decim / timing
```

## Next: HDL

Build the detection front end first: DC blocker, LPF, products, running sums, comparator,
blanking/qualifier/holdoff → `t_trig` and `dbg_metric`. Run an xsim testbench fed from
`TestVectors/*_dec.bin` and compare its output with `pd_reference.detect(..., debug=True)` sample
by sample. Then add the CFO accumulators, the CORDIC, the cascade and the tone check.
