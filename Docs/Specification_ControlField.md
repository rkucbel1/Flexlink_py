# Flexlink Rx – Control Field Receiver Specification

28 Oct 2026. Draft for review, before the bit-true model and the RTL.

## 1. Scope and context

The control field follows Preamble B. It tells the receiver how the payload is built: modulation,
packet length, code, rate matching, number of data blocks, the payload scrambler id. The packet
detector (`Specification_PacketDetector.md`) and the Preamble B timing (§4.5 there, `RTL/pd_timing.v`)
already deliver, per packet, `t_trig`, the CFO (`cfo_inc`) and `n_end`, the last Preamble B sample.
This block takes the samples from `n_end + 1`, decodes the control field and hands the 54 message bits,
with a CRC result, to the rest of the receiver. Payload demodulation and decoding are outside this spec.

Decided: everything runs in the PL (FFT, channel estimate, soft values, descrambling, copy combining,
deinterleaving, Viterbi, CRC). The FFT and the channel estimator are built so the payload chain can reuse them.

### Control field format (fixed by design, `Tx/ControlSignals/Parameters_Control_Field.json`)

| Item | Value |
| --- | --- |
| Position | 4 OFDM symbols right after Preamble B: samples `n_end + 1 … n_end + 4560` at 20 Msps (40 Msps: 9120 samples at 12480 … 21599 of the packet) |
| OFDM | 1024-point FFT, 116-sample CP, 1140 samples per symbol, 841 tones k = −420 … 420 |
| Symbols | Ref, Data, Data, Ref (RefSymbolSpacing 3) |
| Ref symbols (0, 3) | P0 pilots on k ≡ 2 (mod 3), 280 per symbol, BPSK ±1 from `ReferenceSignals/Ref_P0_3_1.h5` (C_init 129), the same 280 values in both; P1 tones (k ≡ 1) empty; 7 DC tones; 274 data tones |
| Data symbols (1, 2) | 826 data tones; 15 DC tones (k = −6 … 8, columns 414 … 428) |
| Data signals | 2200 = 274 + 826 + 826 + 274, taken symbol by symbol in increasing k |
| Mapping | BPSK, bit 0 → −1, bit 1 → +1 |
| Coding | 54 message bits + CRC10 (Gen10, zero init, no reflection) = 64, + 6 zero tail bits = 70, BCC K = 7 [133, 171] rate 1/2 = 140 bits, BCC interleaver (140-entry table), 15 copies = 2100 bits, 100 zero bits padding = 2200, scrambled with the LTE Gold code, c_init 130 |
| Message | 13 fields (RefSymbolSpacing 2 b, RefSignalSpacing 2, PacketLength 3, PayloadFieldMod 2, NoOfTxAntenna 1, CBS_A_Flag 2, FEC_A_Flag 2, RM_A_Flag 4, NDB_A 10, ClientFlag 1, Tx_Ref_Clk_Cnt 14, C_init 8, Reserved 3) |

## 2. Current Python reference

`Rx/Process_Control_Field_Rx.py` (step 4a of the Rx chain) on each antenna's packet file:
`demodulate_waveform()` (FFT window starting `FFT_StartTime` = −12 samples before the end of each CP,
then the phase ramp exp(−j2πk·FFT_StartTime/1024) removed), `equalize_payload_grid()` (default the
adaptive LMMSE of `Equalizer/Channel_Estimator.py`: delay window chosen per packet, Wiener filter across
time), LLR = 4 Re(Y/H) / (σ²/|H|²) = 4 Re(Y H*) / σ², then descramble, add the 15 copies, deinterleave,
Viterbi (soft, end state 0) and CRC.

Since σ² is one value per packet, the LLRs are Re(Y·Ĥ*) up to a common factor, and the Viterbi
decision does not depend on that factor. **The hardware needs no division and no noise estimate:** the
soft value is Re(Y·Ĥ*). Only the channel estimate Ĥ matters.

## 3. Findings (float study behind the board's bit-true detector and timing)

The study runs real Tx packets (`Tx/Waveform_Payload_Tx.h5`, control field included) through AWGN or
fading (`Channel_Defects/Delay_Doppler_Channel.py`), CFO uniform ±500 kHz, 12-bit quantisation at −12
dBFS, then the **bit-true** detector and timing model (`pd_reference.detect`, input stage 1). For every
packet with a good timing record it de-rotates with the event's `cfo_inc`, demodulates from `n_end + 1`
and decodes with several channel estimators. SNR as in `BITTRUE.md` (preamble power over the noise in the
40 MHz band). Counts are control fields decoded with the right message, out of the packets with good timing.

| Channel | SNR | good timing | simulator adaptive LMMSE | linear interpolation | **fixed 8-tap FIR (chosen)** |
| --- | --- | --- | --- | --- | --- |
| AWGN | −14 dB | 116 | 84 | 5 (of 120, earlier run) | **103** |
| AWGN | −13 dB | 184 | 168 | 79 (of 175) | **181** |
| AWGN | −12 dB | 200 | 197 | 168 (of 199) | **199** |
| ETU, 300 Hz | −10 dB | 177 | 174 | 125 (of 131) | **171** |
| ETU, 300 Hz | −8 dB | 196 | 194 | 143 (of 145) | **194** |
| EVA, 70 Hz | −12 dB | 100 | 93 | – | **99** |
| EVA, 70 Hz | −10 dB | 127 | 124 | – | **126** |
| EVA, 70 Hz | −8 dB | 143 | 142 | – | **143** |

(200 trials per SNR for AWGN and ETU, 150 for EVA; the linear-interpolation column is from an earlier
run on other trials.)

- **Plain linear interpolation of the pilots fails at low SNR.** At −14 dB it decodes 5 of 120: the
  pilots themselves are at about −8 dB per tone, and the estimate needs smoothing.
- **A fixed short-window LMMSE does better than the simulator's adaptive one** in AWGN (103 vs 84 of
  116 at −14 dB) and in EVA, and is 0–3 packets behind in ETU-300. The adaptive window choice in the
  simulator seems to pick too long a window at low SNR; worth a look in the simulator itself.
- Longer windows (−12 … 48 samples) lose in AWGN; 8 or 12 taps make no difference; how the two
  reference symbols are combined in time (common drift only, or drift plus linear per tone) makes at most
  1 packet of difference. Choice: 8 taps, window −8 … 24, per-tone linear interpolation after removing the
  common drift (it follows Doppler better).
- No wrong message passed the CRC in these runs. One did in an earlier run of a weaker estimator at
  −14 dB: a 10-bit CRC lets about 1 in 1000 garbage decodes through (open point 1).

## 4. Proposed hardware

```
DC blocker output y (20 Msps) ──► sample buffer (shared with the timing search, 8192 samples)
                                        │ from n_end + 1, once the timing record is good
                                        ▼
                     de-rotator (cfo_inc, phase 0 at n_end + 1)  12-bit I/Q
                                        ▼
                     FFT, 1024 points, 4 symbols (window CP end − 12), 841 tones
                                        ▼ grid buffer 4 × 841
         ┌──────────────────────────────┴──────────────────────────────┐
         │ channel estimate                                            │ data tones (2200)
         │  LS on 280 P0 pilots of symbols 0 and 3 (sign flip by ±1)   │
         │  8-tap FIR across the pilots per tone (coefficient ROM)     │
         │  common drift φ = angle Σ H3 H0* (CORDIC)                   │
         │  per tone: H(s) = ((3−s) H0 + s H3 e^−jφ)/3 · e^jφs/3        │
         └──────────────────────────────┬──────────────────────────────┘
                                        ▼
                     soft value Re(Y Ĥ*) per data tone, in data order
                                        ▼
                     descramble (Gold LFSR, c_init 130), add 15 copies (140 accumulators),
                     deinterleave (140-entry ROM), scale to the Viterbi width
                                        ▼
                     Viterbi, 64 states, 70 steps, end state 0  ──►  CRC10  ──►  control record
```

### 4.1 Samples

The timing search keeps its samples in a 4096-sample block-RAM buffer. The control field starts at
`n_end + 1`, which can be up to about 3000 samples before the end of the search window, and the timing
record only comes out when that window is done (plus the backlog that lets the search start after the
tone check). So the control field gets **its own 8192-sample buffer** (as built: `pd_cf.v`, the timing buffer
stays at 4096). It holds back the input while the next sample of the running control field or the
first one of a queued one would be overwritten; samples of a timing search still running are safe
because `pd_timing` keeps them within 4032 samples of the input.
At 20 Msps nothing stalls; with DMA input the stall logic stays as it is.

### 4.2 De-rotator

Same NCO and cos/sin table as the timing search (24-bit phase, 10-bit table address), but 12-bit
outputs instead of 6. The phase starts at 0 at `n_end + 1`; the channel estimate takes up any constant
phase. The residual CFO after the cascade (about 0.9 kHz rms, 2.3 kHz p99 at −12 dB) turns the phase by
up to 2.5 rad between symbols 0 and 3; that is what the drift correction in the channel estimate is for.

### 4.3 FFT

1024 points, 4 symbols per control field. Proposal: **an own memory-based radix-2 FFT** (one butterfly
per clock, 10 passes of 512 butterflies = 5120 clocks per symbol, two ping-pong RAM banks, 4 DSPs,
twiddle ROM), with a fixed scaling per stage that the bit-true model defines. At 100 MHz one symbol takes
51 µs, less than the 57 µs a symbol lasts, so **the same engine also keeps up with the payload** at the
real sample rate. Alternative: the Xilinx FFT IP (faster, but its arithmetic is only bit-exact through
Xilinx's C model, which does not fit the test flow used so far). Word lengths (input 12, internal 16–18,
scaling per stage) come from the bit-true study.

The window of symbol s starts at `n_end + 1 + 1140 s + 104` (CP end − 12, as `FFT_StartTime` = −12);
the phase ramp this causes is the same on every symbol and the channel estimate absorbs it, so it is not
removed separately.

### 4.4 Channel estimate

- LS on the pilots: Ĥp = Y · ref, ref = ±1 (a sign flip).
- Across frequency: for every tone, Ĥ = Σ w·Ĥp over the 8 nearest pilots. The weights are the LMMSE
  weights for a flat delay profile over −8 … 24 samples and a pilot noise of 0.5 (−3 dB), computed once
  on the PC. There are 25 different weight sets (3 in the middle of the band, the rest at the edges), so the
  ROM is small. The weights are complex (imaginary parts up to 0.085, from the asymmetric window).
- Across time: φ = angle(Σ Ĥp,3 Ĥp,0*) over the 280 pilots (one CORDIC); Ĥ(s) =
  ((3 − s) Ĥ0 + s Ĥ3 e^−jφ) / 3 · e^jφs/3 for s = 0 … 3.

### 4.5 Soft values and decoding

- Soft value of data tone i: Re(Y Ĥ*) (2 multiplies), in the data order of §1.
- Descramble: negate where the Gold bit is 1. The sequence is fixed (c_init 130), so either an LFSR
  (two 31-bit registers, 1600 steps of preroll) or a 2200-bit ROM.
- Copies: soft value i (i < 2100) adds to accumulator i mod 140; the 100 padding values are dropped.
- Deinterleave: code bit j = accumulator `IDX[j]` (the 140-entry table of `Interleave_BCC.py`).
- Viterbi: maximum correlation, 64 states, 70 steps, start and end in state 0; soft inputs scaled to a
  few bits (width from the bit-true study). It is small either way; a serial ACS (one state per clock,
  4480 clocks) or 8 ACS units in parallel.
- CRC10 over the 64 decoded bits: 0 = pass.

### 4.6 Output

One control record per packet that had a good timing record: `t_trig`, `n_end`, CRC pass, the 54
message bits, and a quality value (for example the mean |soft value| after combining, or the Viterbi
path-metric margin) for diagnostics. The firmware passes it to the PC like the events and the timing
records; the PC decodes the fields (`Initialize_Control_Field.parse_control_message`,
`CONTROL_FIELD_CODING`) and compares with the Tx message.

### 4.7 Resources (estimate)

| Block | LUT | DSP | BRAM36 |
| --- | --- | --- | --- |
| buffer growth 4096 → 8192 | – | – | +3 |
| de-rotator | ~200 | 4 | – |
| FFT (radix-2, 1 butterfly/clock) | ~1500 | 4 | 2–3 |
| grid buffer 4 × 841 × 2 × 18 b | – | – | 2 |
| channel estimate (FIR, drift, interpolation) | ~1000 | 4–6 | 1 |
| soft values, combining, deinterleave | ~800 | 2 | – |
| Viterbi + CRC | ~1000–2000 | – | – |
| Total | **~5–6k** | **~16** | **~8** |

The board design is at about 22k LUT (41 %), 110 DSP (50 %); with this it would be about 28k LUT and
126 DSP, which leaves room for the payload chain.

As built (`RTL/cf_*.v`) the receiver has 33 multipliers: de-rotator 4, FFT 4, drift 4, FIR 8
(both reference symbols at once, 8 clocks per tone), H3′ rotation 4, time interpolation 2,
rotation 4, soft value 2, φ/3 1. They are used in different phases, so they could be shared to get
near the estimate above if DSPs get short; the out-of-context run (`HW/synth_cf.tcl`) gives the real
numbers.

### 4.8 Timing

The control field lasts 228 µs. The four FFTs take 205 µs; the channel estimate, soft values and
Viterbi a few thousand clocks more. So the control record comes out roughly 0.3 ms after the last
control-field sample, while the payload samples keep arriving. A payload receiver will need them
buffered (or the FFT run ahead on them); that is for the payload spec.

### 4.9 Word lengths (frozen by the bit-true study)

`PC/cf_reference.py` is the bit-true model (tables from `PC/gen_cf_tables.py` → `PC/cf_tables.py`).
It was run against the float chain with the same estimator on 150 packets per case, after the
bit-true detector and timing (packets decoded with CRC pass and the right message; no wrong messages
passed the CRC in any case):

| Case | timed | float | bit-true | Viterbi 4 b | Viterbi 3 b | FFT 16 b | twiddles 12 b | FIR coef 12 b |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| AWGN −14 dB | 87 | 81 | **81** | 76 | 39 | 80 | 81 | 81 |
| AWGN −13 dB | 131 | 128 | **128** | 128 | 109 | 128 | 128 | 128 |
| ETU-300 −10 dB | 131 | 129 | **129** | 129 | 123 | 129 | 129 | 129 |
| EVA-70 −12 dB | 106 | 104 | **104** | 103 | 91 | 104 | 104 | 104 |

Frozen: de-rotated samples 12 b; FFT input << 5 into **18 b**, twiddles **18 b** (Q17), per-stage
rounding ÷2 with saturation; FIR coefficients 16 b (Q14); full-width soft values; Viterbi inputs
**5 b** after a per-packet shift. The bit-true chain loses nothing against float. The integer Viterbi
was also checked bit for bit against `Convolutionalencoder.ViterbiDecoder` (200 random inputs, 0
differences).

## 5. Open questions

Decided (Oct 2026): CRC10 as is; the simulator got the FIR as `ControlFieldEqualizerMode` FIR8
(default for the control field); own FFT; one antenna; all in the PL.

1. **CRC strength.** CRC10 lets about 1 in 1024 wrong decodes through. With false triggers at about
   0.5/s that all fail the tone check, and good-timing packets that are real, the risk is low; a check
   that the decoded fields are valid codes (`Process_Control_Field_Rx.py` already rejects codes "with no
   meaning") and the Viterbi metric margin could add margin. Acceptable as is?
2. **Simulator estimator.** The fixed short-window FIR does better than the simulator's adaptive LMMSE
   in AWGN and EVA. Should the simulator get it as an `EqualizerMode`, so both use the same algorithm?
3. **FFT.** Own radix-2 engine (proposed: bit-true, small, reusable) or the Xilinx FFT IP?
4. **Several antennas.** The simulator adds the antennas' soft values (MRC). One antenna for now; the
   structure allows adding soft values from more FFT/estimate chains later.

## 5a. Implementation status

| Step | State |
| --- | --- |
| Bit-true model `PC/cf_reference.py`, tables `PC/cf_tables.py` | done; equal to float on AWGN, ETU, EVA (§4.9) |
| Simulator: `ControlFieldEqualizerMode` FIR8 (`Channel_Estimator.FixedFIRChannelEstimator`) | done; decodes the same packets as the bit-true model (150 packets each at AWGN −14 / −13 dB, ETU-300 −10 dB, EVA-70 −12 dB) |
| RTL `RTL/cf_*.v` (FFT, channel estimate, soft values, Viterbi, CRC) | done; bit-true on 11 receiver cases and 400 Viterbi cases (`SIM/run_cf.sh`), fault injection |
| Out-of-context synthesis `HW/synth_cf.tcl` | done: WNS +0.977 ns at 100 MHz; LUT 3125 (657 as memory), FF 1854, DSP 37, BRAM tiles 15 |
| xsim (`SIM/run_cf.sh`) | all 14 checks pass (Vivado 2025.2) |
| Into the detector: `RTL/pd_cf.v` (own 8192-sample buffer, start queue, stall), control FIFO in `pd_axi.v` 4.0, firmware build 11, PC compare (`pd_hw_test.py`, `pd_board_regression.py`) and field decode (`PC/cf_fields.py`) | done; on the board (bitstream 4.0 / 5.0, firmware builds 11, 12), regression all pass |
| Faster FFT `RTL/pd_fft2.v` (two butterflies per clock, 2640 instead of 5200 clocks per symbol, same results; `SIM/tb_fft2.v`), shared with the speculative payload FFT (`Specification_PayloadInput.md` §5) | bitstream 6.0, firmware build 13: simulation and co-simulation pass; the control record comes ~8000 clocks sooner |

## 6. Verification plan (as for the detector)

1. Bit-true model (`pd_reference` or a new `cf_reference.py`): every word length, rounding and table
   defined; a float-vs-bit-true check like `pd_bittrue_check.py` on AWGN, ETU and EVA.
2. RTL with unit tests (FFT against the model on random and full-scale inputs; Viterbi and CRC on
   random messages with noise, exactly on decision boundaries) and the full chain in `tb_pd_detect.v`.
3. Fault injection, out-of-context synthesis, then the board: control records in the RUN reply,
   `pd_hw_test.py` and `pd_board_regression.py` comparing every record with the model, and the decoded
   fields with the Tx message.
