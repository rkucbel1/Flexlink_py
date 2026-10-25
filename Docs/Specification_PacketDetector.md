# Flexlink Rx – Packet Detector Specification

Oct 24, 2026 · @Roman

## 1. Scope and context

The packet detector is an always-on block that watches the 20 Msps receive stream and, on finding Preamble A, hands the packet to the synchronization and decode chain along with a sample index and a coarse CFO. It replaces the block-based Python chain, which processes one captured waveform end to end, with a streaming front end that runs whether or not a packet is present.

This section covers detection and coarse CFO. Fine timing on Preamble B and the control field and payload decode are interfaces only and get their own specifications.

### Preamble structure

The preamble is transmitted at 40 Msps and processed at 20 Msps after decimation by 2. Lengths below are at 20 Msps.

| Part | Samples @ 20 Msps | Duration | Waveform | Receiver use |
| --- | --- | --- | --- | --- |
| AGC burst | 100 | 5 µs | First 100 samples of the Preamble B body (Zadoff-Chu, N\_ZC = 887, u = 54), wideband | AGC settling |
| Preamble A | 5000 | 250 µs | cos(2π·625 kHz·t + π/4) + cos(2π·1.875 MHz·t + 3π/4); real-valued, period 32 samples | Packet detection, CFO |
| Preamble B | 1140 | 57 µs | 116-sample CP + 1024-sample ZC body, about ±8.65 MHz wide | Fine timing |
| Total | 6240 | 312 µs |  |  |

Two properties of Preamble A drive the detector design:

- **32-sample periodicity.** Both tones are whole multiples of 20 MHz / 32 = 625 kHz, so r(n)·r\*(n−32) is constant over Preamble A. This is what the delay-and-correlate detector uses.
- **16-sample anti-periodicity.** Both tones are odd multiples of 625 kHz, so x(n−16) = −x(n) exactly. A lag-16 correlation gives a CFO estimate that is unambiguous to ±625 kHz, the same range as the 4096-point FFT in the current model, with no FFT.

Preamble A sits entirely within ±1.875 MHz, which is why the 2.75 MHz low-pass filter suits the detection branch. Preamble B is about 17 MHz wide, so it must reach the timing correlator unfiltered (F4).

## 2. Current Python reference model

The reference chain in `Flexlink_BasicModules/Preamble` is five scripts run in sequence on one HDF5 capture. Each step reads the previous step's whole output file.

| Step | Script | Operation | Key parameters |
| --- | --- | --- | --- |
| 1 | `Receive_Decimate.py` | 40 → 20 Msps, zero-phase (`filtfilt`) Kaiser FIR | 103 taps, passband 0.8·Fs\_out/2, 80 dB |
| 2 | `Receive_LP_Filter.py` | Causal FIR low-pass | 17 taps, Fc = 2.75 MHz, delay 8 samples |
| 3 | `Receive_Packet_Detector.py` | Delay-and-correlate with hysteresis | Lag D = 32, window SA\_Len = 256, Thresh\_Hi = 0.45, Thresh\_Lo = 0.2 |
| 4 | `Receive_Get_Frequency_Offset.py` | Coarse: 4096-pt FFT peak of the 625 kHz tone. Fine: lag-32 autocorrelation | Window starts at the first flag sample; search ±600.6 kHz in 4.88 kHz bins |
| 5 | `Receive_Get_Timing_Offset.py` | 1140-tap matched filter against Preamble B, global maximum | Run on the LP-filtered stream |

The detector metric, for each sample n, is:

```latex
M(n) = \frac{\left|\sum_{k=0}^{L-1} x(n-k)\,x^*(n-k-D)\right|}{\sum_{k=0}^{L-1} |x(n-k)|^2}, \qquad L = 256,\; D = 32
```

The flag sets when M(n) > 0.45 and clears when M(n) < 0.2.

The included test vector `Test_Preamble.h5` is the clean transmit preamble: no noise, no CFO, no idle samples before or after. With it, the chain reports a detection at sample 200, a CFO of 0 Hz and a timing peak at sample 5224.

## 3. Simulation findings

The detector algorithm works well. Detection is essentially certain down to −10 dB SNR, and the metric never reached 0.45 in 10⁷ noise-only samples. The simulation did surface four problems the clean test vector cannot show, and all four need fixing before the FPGA port.

Setup: your own Python functions were driven by a streaming harness. The front end used causal filters only, as hardware would. The test signal was 1500 noise samples (at 20 Msps), then the preamble, then noise, with AWGN and CFO uniform on ±500 kHz. SNR is mean preamble power over noise power across the full 40 MHz band. Each point averages 100–200 trials.

| # | Finding | Evidence | Fix (§4) |
| --- | --- | --- | --- |
| F1 | False trigger at the end of every strong packet | Flag rises \~256 samples after the signal ends: 2% of packets at 10 dB, 64% at 20 dB, 100% at 30 dB | Normalise by √(R₀·R\_D) instead of R₀ |
| F2 | The reported timing peak (5224) is wrong | The true peak, sample 6248, lies past the end of the record because of the 8-sample LPF delay. With trailing samples added, the peak lands correctly | Test vectors need idle samples on both sides |
| F3 | Timing fails with any CFO above \~5 kHz. The integrated chain already corrects CFO first; the standalone test script does not, so it only works on a zero-CFO vector | The ZC correlation peak jumps 133 samples per 19.5 kHz of residual CFO; 0% correct timing at 20 kHz | Correct CFO before the Preamble B correlator |
| F4 | Standalone test script only: Receive\_Get\_Timing\_Offset.py defaults to the 2.75 MHz-filtered file. The integrated chain (Rx/Packet\_Detector.py) correctly times on the unfiltered, CFO-corrected waveform | The LPF removes 71% of Preamble B energy, halves the correlation peak (817 → 356) and cuts peak-to-sidelobe from 4.2 to 3.6 | Change the standalone script's default input. In hardware, keep the integrated chain's order: tap the timing correlator before the LPF |
| F5 | Start-up transient | One false trigger at sample 0, before the averaging windows had filled | Blank the output for L + D = 288 samples after reset |
| F6 | CFO window can overrun Preamble A | At −12 dB, 95th-percentile detection latency is 3299 samples. A 4096-sample window from the trigger then runs into Preamble B | Shorter window plus a latency check |

&#91;embedded content: Monte Carlo, 200 trials per SNR, bounded-normaliser detector, CFO uniform ±500 kHz\]

Pd is 1.000 from −10 dB up and 0.955 at −12 dB. Latency only matters for the CFO window (F6), and above −8 dB it is never close.

### 3.1 Noise-only false alarms

The metric on noise has mean 0.105 and maximum 0.434 over 10⁷ samples (0.5 s). That mean is above the white-noise value of 1/√256 = 0.0625 because the LPF leaves roughly 70 independent samples per window, not 256. Extrapolating the tail gives a per-sample exceedance probability of about 3×10⁻⁷ at 0.45. That works out to roughly one false trigger per second at 20 Msps.

Noise excursions are short. Above 0.40 the longest lasted 36 samples, and none lasted 64. On a real packet, the metric stays above 0.45 for more than 2700 samples even at −8 dB. A minimum-duration qualifier of 128 samples therefore removes noise triggers at a cost of 6.4 µs of latency.

### 3.2 CFO estimation

The lag-16 → lag-32 estimator matched or beat the existing FFT + lag-32 method at every SNR and offset range tested. It had no gross errors on ±600 kHz.

| SNR (dB) | FFT + lag-32, rms error | Lag-16 → lag-32, rms error |
| --- | --- | --- |
| −10 | 4.6 kHz | 2.7 kHz |
| −6 | 1.7 kHz | 1.2 kHz |
| 0 | 364 Hz | 281 Hz |
| 10 | 55 Hz | 42 Hz |
| 30 | 11 Hz | 3 Hz |

CFO uniform on ±600 kHz, 4096-sample window. F3 sets the requirement: residual CFO at the Preamble B correlator must stay below about 5 kHz. Both estimators meet that down to about −8 dB, but neither is reliable at −10 dB.

### 3.3 Reaching −12 dB

With the 128-sample qualifier, η\_hi = 0.35 detects every packet at −12 dB (200/200) and gives no false triggers in 5 s of noise. The original 0.45 is too strict once the qualifier is added: Pd at −12 dB falls to 0.595, against 0.955 without the qualifier.

| η\_hi | False triggers, noise only | Pd −14 dB | Pd −13 dB | Pd −12 dB | Max trigger latency at −12 dB |
| --- | --- | --- | --- | --- | --- |
| 0.33 | 1.4 /s (7 in 5 s) |  |  |  |  |
| 0.35 | < 0.2 /s (0 in 5 s) | 0.845 | 0.975 | 1.000 | 2361 samples |
| 0.40 | 0 in 0.5 s | 0.295 | 0.675 | 0.985 | 4840 samples |
| 0.45 | 0 in 0.5 s | 0.045 | 0.200 | 0.595 | 4984 samples |

All rows use Q = 128 and the bounded normaliser. Latency runs from the start of Preamble A to the trigger and includes the qualifier. At η\_hi = 0.35 and −12 dB, the latest trigger plus N\_CFO = 2048 ends at sample 4409, inside Preamble A's 5000. At −14 dB a quarter of triggers come too late for that, so −13 dB is the practical floor.

## 4. Proposed FPGA detector architecture

The FPGA detector keeps the Python algorithm, delay-and-correlate at lag 32 over 256 samples, with four changes: a bounded normaliser, a duration qualifier, start-up blanking, and a lag 16 → 32 → 128 → 256 CFO cascade that replaces the 4096-point FFT. It also adds a DC blocker in front (§4.6) and a tone-structure check after the CFO estimate (§4.7), which make it robust to DC offsets and steady interfering tones. Every operation is a streaming recursion with one update per sample. Nothing is divided or square-rooted per sample.

Preamble A is 250 µs long, and detection completes 110–450 samples into it at SNRs above −8 dB. So nothing before the trigger has to be kept. The CFO window starts at the trigger, and Preamble B has not yet arrived. The downstream chain therefore needs no packet capture buffer, only the live stream, the trigger timestamp and the CFO word.

### 4.1 Datapath equations

Per-sample products, where x(n) is the LPF output:

```latex
c_{32}(n) = x(n)\,x^*(n-32), \quad c_{16}(n) = x(n)\,x^*(n-16), \quad e(n) = |x(n)|^2
```

Running sums over L = 256, updated recursively: add the newest term and subtract the term from L samples earlier. In integer arithmetic this is exact and does not drift.

```latex
P(n) = P(n-1) + c_{32}(n) - c_{32}(n-L), \quad R_0(n) = R_0(n-1) + e(n) - e(n-L), \quad R_D(n) = R_0(n-32)
```

The threshold test is evaluated squared, so no divider or square root is needed:

```latex
\text{above}_{hi}(n):\; |P(n)|^2 > \eta_{hi}^2\,R_0(n)\,R_D(n), \qquad \text{below}_{lo}(n):\; |P(n)|^2 < \eta_{lo}^2\,R_0(n)\,R_D(n)
```

R\_D is just R\_0 delayed by 32 samples, so the change in normaliser costs one delay line and one multiplier. By Cauchy–Schwarz the ratio |P|/√(R₀R\_D) is at most 1 for any input, which removes the end-of-packet spike (F1). At η\_hi = 0.45 the Pd and latency are unchanged from the original.

### 4.2 Trigger logic

1. **Blanking.** After reset only, ignore the comparator for L + D = 288 samples, until the windows hold real data. The running sums keep updating while a packet is being received, so re-arming needs no blanking. This is unrelated to the AGC burst, which arrives while the windows are already full.
2. **Qualifier.** Count consecutive samples with above\_hi. The trigger fires when the count reaches Q = 128. A below\_lo sample resets the count, and samples in between hold it (hysteresis).
3. **Trigger.** Latch the free-running sample counter as `t_trig`, start the CFO accumulators, and hand control to the Rx FSM (§6). The detector ignores further triggers until it is re-armed.

### 4.3 CFO estimation (lag 16 → 32 → 128 → 256)

Four lagged correlations, each resolving the ambiguity of the next, give a ±625 kHz range with a 99th-percentile error of 2.1 kHz at −12 dB. That is well inside the 5 kHz the Preamble B correlator tolerates (F3).

**N\_CFO** is the number of samples the CFO accumulators sum after the trigger, i.e. the averaging length. A longer window gives a more accurate estimate, but the window has to end before Preamble A does. The default is 2048 samples (102 µs).

For each lag ℓ ∈ {16, 32, 128, 256}, accumulate from t\_trig over N\_CFO samples:

```latex
A_\ell = s_\ell \sum x(n)\,x^*(n-\ell), \qquad s_{16} = -1,\; s_{32} = s_{128} = s_{256} = +1, \qquad \hat f_\ell = \frac{\angle A_\ell\,F_s}{2\pi\ell}
```

The estimate from lag ℓ is unambiguous to ±F\_s/(2ℓ): ±625 kHz at lag 16, ±312.5 kHz at 32, ±78 kHz at 128 and ±39 kHz at 256. Starting from f̂ = f̂₁₆, each longer lag refines it:

```latex
\hat f \leftarrow \hat f_\ell + \operatorname{round}\!\left(\frac{\hat f - \hat f_\ell}{F_s/\ell}\right)\frac{F_s}{\ell}, \qquad \ell = 32,\,128,\,256
```

The minus sign at lag 16 applies the anti-periodicity x(n−16) = −x(n). Lags 128 and 256 are whole multiples of the 32-sample period. Each step works as long as the previous error is below half the next lag's range, and at −12 dB every step has at least a 2× margin. One CORDIC in vectoring mode computes the four angles in sequence, once per packet. The output is an NCO phase increment for the downstream de-rotator.

| SNR (dB) | Lag 16 → 32: rms / p99 | Lag 16 → 256 cascade: rms / p99 |
| --- | --- | --- |
| −12 | 6.3 / 15.9 kHz | 0.89 / 2.1 kHz |
| −10 | 3.7 / 9.9 kHz | 0.59 / 1.5 kHz |
| −6 | 1.6 / 3.8 kHz | 0.27 / 0.67 kHz |

N\_CFO = 2048, CFO uniform on ±600 kHz, 300 trials per SNR, η\_hi = 0.35. Without lags 128 and 256, 42% of packets at −12 dB would miss the 5 kHz requirement.

**Range check.** With 20 ppm crystals at both ends, the worst-case offset is 40 ppm of the carrier. ±625 kHz therefore covers carriers up to 15.6 GHz, and ±312.5 kHz (lag 32 alone) covers up to 7.8 GHz.

**Late-trigger check.** If the CFO window runs past the end of Preamble A, it sums Preamble B samples and the estimate is wrong, so the packet is aborted. The check runs after Preamble B timing, because that is the first point where Preamble A's end is known exactly:

```latex
\text{abort if}\quad t_{trig} - D_{LPF} + N_{CFO} > n_{PB,start} = n_{PB,end} - 1139, \qquad D_{LPF} = 8
```

D\_LPF accounts for t\_trig being measured on the low-pass-filtered stream while timing uses the unfiltered stream. The abort is reported as "Packet aborted: late trigger", and the receiver returns to Search.

The detector ratio alone could not do this job, because at −12 dB it dips below η\_lo for short spells even on Preamble A. With Bounded, Q = 128 and η\_hi = 0.35:

| Rule | Late triggers caught (−12 / −13 / −14 dB) | Good packets falsely aborted at −12 dB |
| --- | --- | --- |
| Abort if the flag drops during the CFO window | all | 116 of 150 |
| Abort if the ratio stays below η\_lo for 192 samples | 1 of 1 / 7 of 8 / 25 of 26 | 17 of 149 |
| **Abort after timing (above)** | **1 of 1 / 5 of 5 / 33 of 33** | **0 of 149** |

With the post-timing rule, every packet that was kept had exact timing, at all three SNRs. The check is in the simulation (`Rx/Packet_Detector.py`, exit status 2 with the message "Packet aborted: late trigger …").

### 4.4 Block diagram

&#91;embedded content: Packet detector datapath · detection lane and coarse CFO lane\]

The left lane decides whether a packet is present. The right lane reuses its products to estimate CFO once the trigger fires. The highlighted block is the one that differs in behaviour from the Python model.

### 4.5 Preamble B timing correlator

The recommendation is to do Preamble B timing in the frequency domain, using the 1024-point FFT the OFDM demodulator already has. An 1140-tap matched filter running at the full sample rate is not needed. At −12 dB this method finds the exact sample 99.3% of the time.

| Option | Work | Rough cost | Correct timing at −12 dB |
| --- | --- | --- | --- |
| Direct, full-precision taps | 1140 complex multiplies per sample | \~3400 DSP48 at 1 sample/clock | 100% |
| Direct, 1-bit taps | \~4560 add/subtracts per sample | \~60–90k LUTs at 1 sample/clock; about 1/8 of that if time-shared over a search window at 8× the sample rate | 100% |
| FFT domain, 4 windows | 4 × (FFT + 1024 complex multiplies + IFFT) per packet | Reuses the OFDM FFT core, plus a 1024-entry reference ROM and \~3 DSP48 | 99.3% |

Results: 150 trials, AWGN, residual CFO 900 Hz rms (what the §4.3 cascade leaves at −12 dB). The direct rows searched the whole record; in hardware they would search a window.

**Why 1-bit taps need no multipliers.** If each reference tap is quantised to ±1 ± j, the product with a sample reduces to sign changes and additions:

```latex
(I + jQ)(s_I - j s_Q) = (s_I I + s_Q Q) + j\,(s_I Q - s_Q I), \qquad s_I, s_Q \in \{\pm 1\}
```

The simulation showed no loss from this quantisation even at −14 dB. It is still 1140 taps of adders, though, which is why the FFT route wins.

**How the FFT method works.** The Preamble B body is a 1024-sample OFDM symbol with a cyclic prefix. Take any 1024-sample window that starts within ±450 samples of the body start, d samples off. Its FFT, multiplied by the conjugate of the stored body spectrum B\_k and inverse-FFT'd, gives a circular correlation that peaks at index −d. The body start is then the window start plus that peak index.

1. The trigger comes 0–2952 samples into Preamble A (later triggers are aborted, §4.3), so the body starts between t\_trig + 2156 and t\_trig + 5108.
2. Place four windows 800 samples apart to cover that span.
3. Keep the window whose peak-to-rms ratio is highest.

The same IFFT output is the channel impulse response, so the method also gives a first channel estimate for free. That will matter when multipath is added.

**Search window for the 1-bit tap correlator.** The correlator computes outputs only inside a window fixed by t\_trig, about 3000 positions, and takes the peak there. This keeps payload and control-field correlations out of the search and fixes the time at which the decision is made. It does not reduce the hardware: that is set by how many clocks each output is shared over.

The detection latency λ, from the start of Preamble A to t\_trig, is unknown, but it is bounded by 0 ≤ λ ≤ L\_A − N\_CFO = 2952, since later triggers are aborted (§4.3). The last Preamble B sample therefore lies in:

```latex
n_{PB,end} \in \big[\,t_{trig} + L_A + L_B - 1 - (L_A - N_{CFO}) - D_{LPF} - G,\;\; t_{trig} + L_A + L_B - 1 - D_{LPF} + G\,\big] = \big[\,t_{trig} + 3147,\; t_{trig} + 6163\,\big]
```

This uses L\_A = 5000, L\_B = 1140, N\_CFO = 2048, D\_LPF = 8 and a guard of G = 32 samples each side, giving 3017 candidate positions.

1. **Input.** The correlator runs on the unfiltered stream after the CFO de-rotator. Its first output, at t\_trig + 3147, needs samples from t\_trig + 2008, slightly before `cfo_valid` at t\_trig + 2048 plus CORDIC latency. A 256-sample FIFO ahead of the de-rotator covers this. All window indices are in sample-counter time, before that delay.
2. **Search.** For each position n in the window, compute y(n) with the ±1 ±j taps. Track the maximum |y|², its index and the running mean of |y|².
3. **Decision** at the window end: n\_PB,end = argmax. The result is valid if peak / mean ≥ T\_pk; otherwise abort with "no peak in window". T\_pk = 18 (AWGN; below).
4. **Late-trigger check** (§4.3) on that n\_PB,end.
5. **Hand-off.** The control field starts at n\_PB,end + 1, but the decision is only made at the window end. So control-field samples are held in a FIFO of about 3.3k samples (3017 window positions plus pipeline), about 80 kb or 3 BRAM36.

| Clocks per output, K | Fabric clock | Tap units per clock, ⌈1140 / K⌉ | Adders (4 per tap unit) |
| --- | --- | --- | --- |
| 4 | 80 MHz | 285 | \~1140 |
| 8 | 160 MHz | 143 | \~570 |
| 10 | 200 MHz | 114 | \~460 |

The correlator also needs the 1140-sample delay line (about 27 kb, one BRAM36) and a 2.3 kbit sign ROM.

**Why T\_pk is required.** A trigger that comes too late puts the true Preamble B peak before the window opens. The search then picks a noise peak inside the window, and that wrong peak also gets past the late-trigger check. With no Preamble B in the window, peak/mean never exceeded 15.4 in 300 trials, with a median of 8.5. Correct peaks were at least 20 at −14 dB, and at −12 dB at least 38 with 1-bit taps and 27 with the FFT method. T\_pk = 18 sits between the two:

| SNR | Method | T\_pk = 0: correct / wrong timing / aborted | T\_pk = 18: correct / wrong timing / aborted |
| --- | --- | --- | --- |
| −12 dB | 1-bit taps | 99 / 1 / 0 | 99 / 0 / 1 |
| −13 dB | 1-bit taps | 91 / 8 / 0 | 91 / 0 / 6 |
| −14 dB | 1-bit taps | 59 / 24 / 1 | 63 / 0 / 24 |
| −14 dB | FFT method | 56 / 19 / 2 | 60 / 1 / 15 |

Each row is 100 packets with Bounded, Q = 128, η\_hi = 0.35 and the lag cascade; the remainder were not detected. At −10 dB both settings gave 100 correct. ETU multipath lowers the correct-peak values, so check T\_pk against those runs.

The simulation matches this design. With `Defined_Rx.TimingMethod` set to "1-bit Taps" or "FFT method", `detect_timing_offset()` searches only this window, and `Defined_Rx.TimingPeakThresh` (default 18) is T\_pk. "Full Precision Taps" stays the MATLAB whole-record search, as a reference.

### 4.6 DC blocker

A DC-blocking filter on the 20 Msps stream, ahead of all preamble processing, stops a receiver DC offset from triggering the detector. It costs two adders and a shift, and no sensitivity. Without it, a DC offset 10 dB below the noise triggers the detector, and at −6 dB the flag stays high continuously. A direct-conversion radio always leaves some DC offset.

```latex
y(n) = x(n) - x(n-1) + \alpha\, y(n-1), \qquad \alpha = 1 - 2^{-10}, \qquad f_{3\,dB} \approx \frac{(1-\alpha) F_s}{2\pi} = 3.1\ \text{kHz}
```

- **Placement:** on the whole stream, before the 2.75 MHz LPF, so the timing correlator and the payload see it too. Preamble A has no energy at DC. The payload's DC subcarrier is empty, and the neighbouring subcarriers at ±19.5 kHz lose 0.1 dB, which the equalizer absorbs.
- **Large frequency offsets:** an offset of up to ±625 kHz can move one Preamble A tone onto DC, where the blocker removes it. The other three tones still satisfy the lag-16 and lag-32 relations, so detection and the frequency estimate are unaffected.
- **Word length:** the feedback term needs about 10 extra fractional bits to avoid limit cycles. That gets settled in the bit-true model.

With your channel model (ETU at 300 Hz Doppler, phase noise, IQ imbalance, drift, SNR −12 dB) plus a 0 dBc DC offset, packet outcomes with the blocker on match a run with no DC offset at all. With the blocker off, 38 of 40 packets were lost.

### 4.7 Tone-structure check

Any single steady tone within ±2.75 MHz, a DC offset or a CW interferer, passes the packet detector, because it repeats at every lag. The tone-structure check rejects these after the frequency estimate. Preamble A is four tones; an interferer is one.

At t\_trig + N\_CFO the frequency estimate f̂ is known. Using the stored 2048-sample CFO window, compute single DFT bins at the four Preamble A tones, f̂ + {−1.875, −0.625, +0.625, +1.875} MHz. Also compute them at nine frequencies where Preamble A has no energy: f̂ + {0, ±0.3125, ±0.9375, ±1.25, ±1.5625} MHz.

```latex
\text{ToneCheck} = 10\log_{10}\frac{\sum_{k=1}^{4} P_{tone,k} - \max_k P_{tone,k}}{\tfrac{1}{9}\sum_{m=1}^{9} P_{ref,m}}, \qquad \text{abort if ToneCheck} < T_{tone} = 13\ \text{dB}
```

The lag cascade always places a single interfering tone on one of the four tone positions. So the strongest tone is dropped, and the three weaker tones must stand clear of the noise.

| Input | Lowest ToneCheck seen | Highest ToneCheck seen |
| --- | --- | --- |
| Preamble A, AWGN, −13 dB (200 trials) | 14.1 dB | 26.0 dB |
| Preamble A, static ETU, −10 dB (300) | 12.0 dB | 31.5 dB |
| Preamble A, static ETU, −12 dB (300) | 0.9 dB (6 of 240 detected below 10 dB) | 28.9 dB |
| CW, −10 to +30 dB relative to noise (900) | −10.5 dB | 12.7 dB |
| DC, −6 to +30 dB relative to noise (500) | −4.8 dB | 11.5 dB |

Two simpler checks were tested and rejected:

- **Correlation at lag 8**, where Preamble A's tones cancel: under ETU fading the tones become unequal, and real packets overlapped with interference.
- **The second-strongest tone alone**, with three noise-reference bins: the two groups overlapped between 11 and 14 dB.

Through your full channel model at −12 dB, the check aborted 1 of 60 packets, and the outcome of every other packet was unchanged.

**What it does and doesn't solve.** It stops the receiver from committing to false triggers, and it frees the receiver at t\_trig + 2048 instead of at the end of the timing window, t\_trig + 6163. It can't rescue a packet that arrives with a strong in-band interferer: the frequency estimate is then dominated by the interferer. With a 0 dBc CW at 1.1 MHz, 54 of 60 packets were aborted by the check and the rest failed later. That case needs a notch filter, which is in §8.

**Hardware:** a 2048 × 24-bit buffer for the CFO window (49 kb, two BRAM36), and 13 single-bin DFTs of 2048 points each, about 26.6k complex multiply-accumulates. On two complex MAC units at 160 MHz that takes about 85 µs, which finishes well inside the timing window and runs in parallel with it.

## 5. Interfaces, parameters and resources

The detector, with the DC blocker, the four-lag CFO cascade and the tone-structure check, is about 26 DSP slices and three BRAM36 at one sample per clock, or about 8 DSPs if the fabric clock is at least 5× the 20 Msps sample rate. Widths below are proposals to be confirmed by bit-true simulation (§7).

### 5.1 Ports

| Port | Dir | Width | Description |
| --- | --- | --- | --- |
| `s_iq_tdata` / `s_iq_tvalid` | in | 2 × 12 b | LPF output, 20 Msps, signed I/Q |
| `sample_cnt` | in | 32 b | Free-running sample counter shared with the downstream chain |
| `arm` | in | 1 b | Re-arm pulse from the Rx FSM; re-enables the trigger (no blanking, the running sums never stop) |
| `trig` | out | 1 b | One-cycle pulse when the qualifier completes |
| `t_trig` | out | 32 b | `sample_cnt` latched at trigger |
| `cfo_inc` / `cfo_valid` | out | 24 b / 1 b | NCO phase increment for −f̂, valid N\_CFO samples after trigger |
| `pa_end` / `pa_end_cnt` | out | 1 b / 32 b | Optional: flag falling edge, an end-of-Preamble-A marker (§7) |
| `dbg_metric` | out | 16 b | √-free metric \|P\|² / (R₀R\_D), for ILA capture only |

### 5.2 Run-time parameters

| Register | Default | Notes |
| --- | --- | --- |
| `ETA_HI_SQ` | 0.1225 (η = 0.35) | Sensitivity control. 0.35 meets −12 dB at < 0.2 false triggers/s (§3.3); raise it for a less sensitive, quieter receiver |
| `ETA_LO_SQ` | 0.04 (η = 0.2) | Hysteresis release |
| `QUAL_LEN` | 128 | Minimum consecutive samples above η\_hi |
| `N_CFO` | 2048 | CFO accumulation length; 4096 allowed when latency allows |
| `BLANK_LEN` | 288 | L + D, after reset only |
| `DC_SHIFT` | 10 | DC blocker α = 1 − 2^−DC\_SHIFT (3.1 kHz corner); 0 = bypass (§4.6) |
| `TONE_THRESH` | 13 dB | Tone-structure check (§4.7); 0 = off |
| `T_PK` | 18 | Minimum peak/mean of the Preamble B search (§4.5); 0 = off |

L = 256 and D = 32 are fixed at synthesis. L = 256 is a power of two, so no 1/L scaling is needed: it cancels in the ratio.

### 5.3 Word lengths

| Signal | Bits | Reasoning |
| --- | --- | --- |
| x(n) I/Q | 12 | AGC-normalised LPF output |
| c₃₂, c₁₆ (each part) | 25 | 12 × 12 product + 1 for the complex add |
| P, R₀ accumulators | 33 | +8 bits of growth over 256 terms; full precision so the recursion stays exact |
| P, R₀, R\_D into the comparator | 18 | Top 18 bits after a common shift. A \~1% comparison accuracy is enough, and each fits one DSP48 port |
| A₁₆, A₃₂ | 37 | +12 bits over 4096 terms |
| CORDIC angle | 16 | 360°/2¹⁶ at lag 32 ≈ 10 Hz resolution, below the 3 Hz-to-kHz estimation noise floor |

### 5.4 Resource estimate (one sample per clock)

| Item | DSP48 | Memory |
| --- | --- | --- |
| DC blocker | 0 (adders) |  |
| c₃₂ and c₁₆ complex multiplies | 6 | 32-sample x delay (LUTRAM) |
| c₁₂₈ and c₂₅₆ complex multiplies (CFO only) | 6 | 256-sample x delay, 256 × 24 b ≈ 6 kb |
| e(n) = \|x\|² | 2 |  |
| P, R₀ running sums | 0 | 256 × 75 b history ≈ 19 kb, one BRAM36 |
| R\_D delay | 0 | 32 × 33 b (LUTRAM) |
| \|P\|² and R₀·R\_D, two threshold scales | 5 |  |
| Four CFO accumulators | 0 (adders) |  |
| CORDIC, four angles once per packet | 0–1 | Shared IP |
| Tone-structure check: 13 single-bin DFTs, two complex MAC units | \~6 | 2048-sample window buffer, 49 kb, two BRAM36 |
| **Detector total** | **≈26** | **3 BRAM36 + LUTRAM** |

Preamble B timing (§4.5) comes on top: about 570 adders at 160 MHz, one BRAM36 delay line, and about three BRAM36 for the control-field FIFO.

The two threshold multiplies can become shift-add constants once η is frozen. At a 100 MHz fabric clock the multipliers can be time-shared five ways.

## 6. Control state machine and recovery

The detector belongs to the Search state. Every other state either finishes or aborts and comes back to Search, so a false trigger costs at most one aborted control-field decode and can never lock the receiver up.

&#91;embedded content: Rx control FSM · 5 states, 3 abort exits\]

While any state other than Search is active, the detector's trigger output is ignored. Mid-packet re-triggering on a stronger packet is not supported in this version. Leaving Search always pulses `arm` on return, which re-enables the trigger. The 288-sample blanking happens only after reset.

Each exit condition needs a hard bound, so every state has a timeout:

| State | Normal exit | Abort condition | Bound |
| --- | --- | --- | --- |
| CFO estimate | `cfo_valid` after N\_CFO samples | Tone-structure check fails (§4.7). The late-trigger check runs after timing | N\_CFO samples |
| Preamble B timing | Correlation peak above threshold | No peak inside the search window (peak / mean < T\_pk), or late trigger (§4.3) | Window end, t\_trig + 6163 (§4.5) |
| Control field | CRC passes | CRC fails | Control field length |
| Payload | Last symbol decoded | None | Length from the control field |

## 7. Open questions and next steps

Decided (24 Oct 2026):

- [x] **Sensitivity** is a run-time parameter (`ETA_HI_SQ`). The target is −12 dB, met at η\_hi = 0.35 with Q = 128 (§3.3).
- [x] **False alarms.** One per second is acceptable for now; η\_hi = 0.35 measured below 0.2/s.
- [x] **CFO method.** The lag cascade 16 → 32 → 128 → 256 replaces the 4096-point FFT (§4.3).
- [x] **Crystals.** 20 ppm worst case, 10 ppm typical, so up to 40 ppm relative offset between ends.
- [x] **Test vectors** get idle samples and the chain is re-run; multipath goes through the existing simulator.

Decided (25 Oct 2026):

- [x] **DC blocker** ahead of the preamble processing (§4.6), in the simulation as `Defined_Rx.DCBlocker` (default 1).
- [x] **Tone-structure check** to reject DC and CW interference (§4.7), in the simulation as `Defined_Rx.ToneCheckThresh` (default 13 dB).
- [x] **DC offset and CW interferer defects** added to `Channel_Defects.json` and `Defect_Model.py` (both off by default).
- [x] **Search window and T\_pk = 18** for the 1-bit and FFT timing methods (§4.5); **late-trigger abort** after timing (§4.3).

Still open:

- [ ] **Carrier frequency.** Needed to confirm the range: ±625 kHz covers 40 ppm up to 15.6 GHz. Below 7.8 GHz, lag 16 could be dropped.
- [ ] **Timing method.** Choose between the FFT-domain method (recommended) and a time-shared 1-bit correlator (§4.5). The choice depends on whether the OFDM FFT core is free during the preamble.
- [ ] **Preamble B search window.** Confirm the four-window placement from t\_trig against your own analysis.
- [ ] **Bit-true model.** Build a fixed-point Python model of §4–5 and generate reference vectors for HDL co-simulation.
- [ ] **Multipath.** In progress: you are running the ETU model in your simulator. Use those runs to confirm T\_pk = 18 and the timing figures.

## 8. Implementation considerations

The detector design fits comfortably in a mid-size FPGA. On a Zynq-7020 (Zedboard), the whole preamble path would use roughly 10–15% of DSPs, 5% of BRAM and 25% of LUTs. These items are outside the detector datapath but have to be settled before the HDL is written.

| # | Item | Why it matters | Proposed direction |
| --- | --- | --- | --- |
| 1 | AGC | Gain control is done by the radio (AD9361 fast-attack AGC), not the FPGA. The FPGA only has to make sure the gain doesn't change between t\_trig and the end of the packet: a gain step, which can bring a phase step, inside the 2048-sample CFO window or the payload corrupts the estimate | AD9361 fast-attack AGC settles on the AGC burst and early Preamble A; ADI targets gain lock within an 8 µs preamble. The FPGA state machine locks the gain at t\_trig (the radio's external AGC control) and releases it at packet end or abort. Set the AGC target level so the 12-bit samples leave headroom (§5.3). At −12 dB SNR the packet raises total power by only 0.3 dB, so the gain doesn't move. |
| 2 | Strong in-band interference | The tone-structure check rejects false triggers, but a packet arriving with a strong CW (around noise level or above) is still lost, because the frequency estimate is dominated by the tone | Adaptive notch: after repeated tone-check failures, estimate the tone frequency and notch it ahead of the detector, avoiding the Preamble A tone positions |
| 3 | Multiple receive antennas | The simulator runs up to 4 antennas; the spec covers one stream | Sum P and R₀·R\_D across antennas before the threshold, which costs little and gives diversity against ETU fading at −12 dB; then one CFO estimate and one timing search |
| 4 | Front-end decimation | Hardware needs a causal, fixed-point 40 → 20 Msps filter. The simulator uses `resample_poly` with defects on, and keeps every other sample without a filter when defects are off | A half-band FIR decimator. Without a filter, noise from 10–20 MHz folds into the band and costs about 3 dB |
| 5 | Bit-true model | The §5.3 word lengths, the DC-blocker feedback precision and the CORDIC angle width are unverified | A fixed-point Python model of §4, checked against these results at −12 dB, then used to generate HDL test vectors |
| 6 | Sample-clock offset | 40 ppm moves timing 0.2 samples over Preamble A (negligible), but drifts over a long payload | Timing tracking in the payload chain (pilots), outside this spec |
| 7 | Interfaces and clocking | Radio interface, crossing from the 20 Msps sample clock to the fabric clock, and handoff to control-field and payload processing | Define in the system-level specification |
| 8 | Post-packet hold-off | The bounded normaliser already prevents the end-of-packet spike | Optional: disable the trigger for 288 samples after the last payload sample |

**Radio front-end alternative: ESP32-S31.** Undocumented IQ-capture firmware for ESP32 chips has been reported ([CNX Software](https://www.cnx-software.com/2026/10/04/esp-sdr-firmware-turns-esp32-into-a-2-4-5-ghz-software-defined-radio-sdr/), [RTL-SDR.com](https://rtl-sdr.com/various-projects-independently-find-hidden-sdr-capabilities-in-esp32-microcontrollers)). It changes several of the items above:

- **Sample rate (blocking):** continuous streaming is reported at up to 16 MS/s over Gigabit Ethernet, and higher rates only as snapshots into on-chip SRAM. Flexlink processes at 20 MS/s, and its 841 tones span about ±8.2 MHz. So continuous reception would need either a direct high-rate path from the radio to the FPGA, or a narrower Flexlink mode.
- **Gain:** there's no documented gain lock. With fixed gain, the detector is unaffected, since its ratio is normalised; dynamic range is then set by the unknown ADC width. If gain moves on its own, a step inside the CFO window or the payload has to be detected (a sudden power change) and the packet aborted or compensated.
- **Spurs:** harmonics of the 40 MHz crystal are reported at exactly 2400 and 2440 MHz. These are the steady in-band tones that §4.6–4.7 and the notch in item 2 deal with.
- **Unknowns:** ADC bit depth, gain control and DC behaviour aren't published. The firmware is unofficial and the chip was a preview at the time of the reports.
