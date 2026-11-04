# Flexlink Rx – Payload Input Buffer and Packet Descriptor (draft)

Status: proposal, October 2026. Built: the re-arm state machine (section 4; bitstream 5.0,
firmware build 12) and the shared fast FFT with the speculative payload FFT (section 5;
bitstream 6.0, firmware build 13; on the board: regression 39/39, every payload signature exact). Follows `Specification_ControlField.md`. It fixes how the payload receiver finds its first sample and its settings, what happens
when a packet is rejected on the way, and how the payload chain catches up with the input.

## 1. Idea

The input stream is written, without gaps, into one circular sample buffer. Every sample keeps
its index (the counter that `t_trig` and `n_end` already use), so a position in the stream is
just a number. Each packet gets a **packet descriptor** that is filled in step by step as the
packet passes the detector, the timing search and the control field. While a descriptor is
alive, the buffer keeps every sample from its payload start on (a keep pointer); the input is
held back if a needed sample would otherwise be overwritten, as the control-field block does
today.

- **Rejected packet:** the descriptor is dropped. Nothing is erased or "flushed"; the keep
  pointer is released and the samples are overwritten in the normal way.
- **Accepted packet:** the payload engine starts reading at the descriptor's `payload_start`,
  with the settings from the control field: no search, no guessing.

## 2. Timeline (measured)

From the RTL simulation of rep3 at exactly 20 Msps (`./run_sim.sh` with `FIXGAP=1 GAP=4`,
N_CFO 2048), in input samples:

| Point | Sample (packet 1) | Relative |
| --- | --- | --- |
| Preamble A starts | 3108 | 0 |
| Trigger `t_trig` | 3400 | +292 |
| Last Preamble B sample `n_end` | 9253 | +6145 |
| Timing record out | 10375 | n_end + 1122 |
| Payload start `n_end + 4561` (after the control field) | 13814 | |
| Control record out | 19211 | timing record + 8836 (44 000 clocks) |
| **Lag of the payload engine at its start** | | **5397 samples ≈ 4.7 symbols** |

The lag is `d + 4293 + Δ` samples, with `d` = trigger position in Preamble A (292 here) and
`Δ` = the timing record's delay after the end of its search window (812 here, mostly the tone
check's backlog; it grows with N_CFO). The trigger can come at most 2966 samples into Preamble A
before the late-trigger check aborts (N_CFO 2048), so the **worst case is about 8100 samples**
(7.1 symbols). The other packets of rep3 give 5396 and 5409.

## 3. Packet descriptor

| Field | From | Filled when |
| --- | --- | --- |
| `t_trig` | detector | trigger |
| `cfo_inc` | CFO lane | event (after the tone check) |
| `n_end`, `payload_start = n_end + 4561` | timing | timing record (status good) |
| `RefSymbolSpacing`, `RefSignalSpacing`, `PacketLength`, `Mod`, `NoOfTxAntenna`, `EncoderBlockSize`, `EncoderCodeRate`, `Repetition`, `ClientFlag`, `NDB_A`, `C_init` | control field | control record (CRC ok, every code valid) |
| `packet_end = payload_start + 1140 · PacketLength − 1` | computed | control record |
| state: `WAIT_TIMING`, `WAIT_CONTROL`, `READY`, `RUNNING`, `DONE`, `DROPPED` (+ reason) | | |

A descriptor is created at the event (only events that passed the tone check), so the
existing records already carry everything up to the control field. In hardware it is a small
register set per entry, a queue of 2 (a second packet's control field can finish while the
first payload is still being processed).

## 4. Drop rules

| Reason | Where it is known |
| --- | --- |
| tone check failed | event (`EV_TONE_FAIL`) – no descriptor is created |
| timing: no peak, late trigger, BUSY | timing record status ≠ good |
| control field CRC failed | control record |
| a field code with no meaning (as `Process_Control_Field_Rx.py`), or a setting the board does not support (e.g. `NoOfTxAntenna` 2, a modulation not built yet) | control record |
| the samples of `payload_start` were lost (cannot happen while the keep pointer works; checked anyway) | keep-pointer check |

A drop releases the keep pointer at once and reports the descriptor with its reason (a new
"packet record" in the RUN reply, so the PC can compare drops with the model too).

**Re-arming (built: `RTL/pd_arm.v`, `pd_reference.rearm_point`, `Params.rearm` = 1, the
default).** The detector stays disarmed after a trigger until the packet's outcome is known,
then re-arms at a sample R fixed by that outcome (N = N_CFO):

| Outcome | R |
| --- | --- |
| tone check failed | t + 2N + 256 |
| input stage 0, tone check passed | t + max(arm_holdoff, N, 1, 2N + 256) |
| timing record not good | t + 2N + 6656 |
| control field: CRC failed, or a code with no meaning (PacketLength 000 / 010, CBS_A_Flag 11, RM_A_Flag > 1000) | n_end + 2N + 12753 |
| control field good | max(n_end + 4561 + 1140 · PacketLength, n_end + 2N + 12753): the end of the packet |

R is a sample index, not a time, so the board and the model agree exactly. Each R lies after
the last sample its decision needs, and at 20 Msps the decision is out before sample R − 64
arrives (checked in the RTL simulation at exactly 5 clocks per sample: 0 stalls, N_CFO 128 /
2048 / 4096). With faster input (DMA) the input is held at R − 64 until the decision is in.
After a false trigger on noise the detector is deaf for 2N + 256 samples (0.22 ms at N = 2048)
instead of the fixed holdoff (4.75 ms). `rearm` = 0 keeps the old fixed holdoff for tests.

## 5. Catching up: three options

The payload starts arriving about 5400 (up to 8100) samples before the control record is out.
A symbol arrives every 5700 clocks at 20 Msps. The control-field FFT (one butterfly per clock)
takes about 5970 clocks per symbol with its unload, so as it is it would not keep up with the
input, and the lag would keep growing (about 54 samples per symbol).

| | A. Sample buffer, current FFT | B. Sample buffer, faster FFT | C. Speculative payload FFT |
| --- | --- | --- | --- |
| What | read the payload from the sample buffer at the FFT's own speed | FFT with 2 butterflies per clock (about 2600 + unload clocks per symbol) | payload FFTs start at `payload_start` in real time, while the control field is still decoding; grids wait in memory |
| Lag at the end of a 121-symbol packet | 8100 + 121 × 54 ≈ 14 600 samples | catches up after about 18 symbols | none at the FFT; the grids wait about 7 symbols |
| Input buffer | **16 384 samples** (11 BRAM36) | 16 384 (8192 has no margin for the worst case) | 8192 (today's) |
| Grid buffer | RefSymbolSpacing + 1 grids | RefSymbolSpacing + 1 grids | about 8 + RefSymbolSpacing grids (1 BRAM36 each) |
| Extra DSP | 0 (FFT shared with the control field if it is idle) | +4 | a second FFT (+4) or one shared fast FFT |
| Result latency after the packet ends | about 1 ms | a few symbols | a few symbols |
| Drop on CRC fail | release the keep pointer | same | also discard the grids |

The FFT is probably not the hard part: the **LDPC decoder** decides how fast the payload chain
really is (up to 279 000 coded bits per 73-symbol packet at 64QAM, 1944-bit code words). If it
is slower than real time, buffering moves after the FFT (grids or soft values) anyway, and the
choice between A, B and C is about where the buffer sits, not whether there is one. A grid is
smaller than its samples (841 tones instead of 1140 samples per symbol).

**Decided (Oct 2026):** low latency is preferred, PacketLength 73 or less is the normal case
(97 and 121 are nice to have). That favours **option C** (payload FFTs start at `payload_start`,
grids wait about 7 symbols for the control record, the 8192-sample input buffer stays), with
option B's faster FFT as the shared engine, so the payload result comes a few symbols after the
packet ends. With PacketLength ≤ 73 option A would need about 12 000 samples; it remains the
fallback if the FFT has to be shared with the control field.

### Built: shared fast FFT, speculative payload FFT (bitstream 6.0, firmware build 13)

One FFT for the control field and the payload, inside the control-field receiver
(`RTL/pd_fft2.v`, `cf_rx.v`, `pd_cf.v`; model `cf_reference.payload_records`):

- **Faster FFT.** Two butterflies per clock (each bank split into two 512-word dual-port RAMs
  by address bit 9, so a clock's 4 reads and 4 writes use the 4 ports), 2640 clocks per symbol
  instead of 5200, results identical to `fft_fixed`. +4 DSP, the banks need 8 instead of 2
  BRAM36.
- **No second FFT, no arbiter.** The control field and the payload are consecutive symbols of
  one stream from the same sample buffer (`pd_cf`'s 8192 samples), so the receiver simply goes
  on: payload symbol s is stream symbol 4 + s, window `1140 (4 + s) + 104 … + 1127` after `n_end`,
  the de-rotator's phase ramp continued. The receiver is split into a front end (de-rotator,
  FFT, unload) and a back end (channel estimate, Viterbi, CRC) that run at the same time.
- **Speculation.** While the back end decodes the control field the front end starts at most 8
  payload symbols. The decision (CRC ok and every field code valid, as the re-arm) then keeps the
  first PacketLength of them or drops them, and the front end stops at PacketLength (or at once).
  Only with `rearm` 1, where packets cannot overlap.
- **Speed.** Per symbol 1140 samples in, then 2640 FFT + ~850 unload clocks with the input held:
  about 4630 clocks against 5700 at 20 Msps, so the FFT keeps up (0 stalls in the RTL simulation
  at exactly 5 clocks per sample) and lags at most about one symbol.
- **Check without the rest of the chain.** Each payload symbol's 841 bins give a 64-bit signature
  (Fletcher-style, `cf_reference.grid_signature`) that goes into a 16-record FIFO (PL_LEVEL 0x5C,
  PL_DATA 0x60) and the RUN reply (`PayloadRecord`), so the board regression checks the payload
  FFT bit for bit. A full FIFO holds the FFT, never loses a record. When the grid buffer and the
  payload chain are built, the bins go there instead (`pg_*` of `cf_rx`) and the signature stays
  as a check.

Measured at 20 Msps (rep3, `FIXGAP=1 GAP=4`): the control record now comes 3770 samples after
the payload start (it was 5400), with 2 payload symbols already done; every payload symbol's
FFT is done about 700 samples after its window, the last one about 690 samples (35 µs) after the
packet ends. So with the grid buffer the grids wait at most ~4 symbols for the control record
(the worst-case trigger position adds ~2700 samples: ~6 symbols), not 7.

## 5a. LDPC decoder sizing

Decided (Oct 2026): size the LDPC decoder first; the payload supports every modulation and
setting; no packet record in the RUN reply for now.

**What the simulator does** (`Rx/Get_Decoded_Bitstream.py`, `LDPC_Encoder/LDPC_Decode.py`):
802.11n quasi-cyclic codes (N = 648 / 1296 / 1944, Z = 27 / 54 / 81, rates 1/2 … 5/6, 12 /
8 / 6 / 4 block rows, 79–88 non-zero circulants), flooding min-sum, 15 iterations, no early
stop. The soft values are P(bit = 1) from the max-log demapper, averaged over the Repetition + 1
copies and mapped to 2p − 1. The hardware will combine the copies' LLRs instead (a sum: the
optimal combining) and keep them as integers; that is a separate float study, as for the
control field.

**Worst case load.** Repetition is at least 1 (2 copies), so a packet carries at most
`NumberOfDataSigs · bits / (2 · N)` code words. The largest grid (PacketLength 121,
RefSymbolSpacing 12, RefSignalSpacing 24) has 98 846 data tones. At 64QAM that is 457 code words
of 648, 228 of 1296 or 152 of 1944 in 6.9 ms: **about 43 Mbit/s of coded bits, whatever N**
(the same for every PacketLength). Higher Repetition divides it.

**Architecture.** Layered min-sum (it needs about half the iterations of flooding for the same
result), one block row (layer) at a time, P parallel lanes, one circulant per clock, a cyclic
shifter for the 27 / 54 / 81 shifts. Clocks per iteration ≈ circulants × ⌈Z / P⌉ + layers ×
pipeline depth (6); with Z < P several code words run side by side. Coded throughput at 100 MHz
(range over the four rates):

| Lanes P | 648, 8 it. | 648, 15 it. | 1296, 8 / 15 it. | 1944, 8 / 15 it. | LUT (estimate) | BRAM36 |
| --- | --- | --- | --- | --- | --- | --- |
| 27 | 51–72 | **27–39** ✗ | 66–84 / 35–45 | 74–93 / 39–50 | ~6 k | ~5 |
| **54** | 101–145 | 54–77 | 103–149 / 55–79 | ~100–150 / ~53–80 | **~11 k** | ~7 |
| 81 | 152–217 | 81–116 | 103–149 / 55–79 | 154–236 / 82–126 | ~17 k | ~9 |

(Mbit/s; the need is 43.) No DSPs: min-sum is additions, comparisons and a shifter.
**Proposal: P = 54.** It keeps up in every case even with 15 iterations and no early stop, has
room for early termination (syndrome check) to save power, and fits next to the rest of the
payload chain (about 31 k LUT left today). P = 27 only works with ≤ 8 iterations at N = 648.

**The rest of the payload chain** (estimate): FFT (option B) +4 DSP; channel estimate as the
control field, about 8–12 DSP; equaliser and demapper without division (LLRs as piecewise-linear
functions of Y·H* with breakpoints at multiples of |H|², up to 64QAM), about 8 DSP; copy
combining, de-interleaver, de-scrambler: memory and adders. About 30 of the 73 free DSP.
(Superseded by §5b: the demapper's LLRs are scaled by 1/σ², so the equaliser needs the noise
variance; the LLR combining of the copies is a sum.)

## 5b. LDPC float-vs-bit-true study (done, Oct 2026)

Scripts and raw results: `PC/ldpc_study/` (README there). AWGN, max-log demapper as the
simulator, 32 configurations: BPSK / QPSK / 16QAM / 64QAM × N 648 / 1944 × rate 1/2 / 5/6 ×
2 / 20 copies (Repetition 1 / 19), 300–600 code words per point. Figure of merit: Es/N0 per
copy for 10 % block errors, interpolated in 0.25 dB steps.

**Decoder variants, loss against layered float 15 iterations** (mean / worst over the 32):

| Variant | Mean | Worst |
| --- | --- | --- |
| simulator today: flooding 15, averaged P(bit = 1) | 0.56 dB | 1.02 dB |
| flooding 15, summed LLRs | 0.34 | 0.74 |
| layered α = 0.75, 5 iterations | 0.59 | 1.36 |
| layered, 8 iterations | 0.25 | 0.76 |
| **layered, 10 iterations** | **0.14** | **0.32** |
| layered, 15 iterations | 0 | 0 |

Findings:
1. **Summing the copies' LLRs** is better than averaging probabilities (0.2 dB mean, up to
   0.5 dB at 2 copies, rate 5/6).
2. **Layered at 10 iterations** beats today's simulator decoder in every configuration
   (0.4 dB mean). 15 iterations buy another 0.14 dB for 50 % more decode time; with latency
   first, 10 is the maximum, with early stop on a zero syndrome.
3. **The LLR step must be an absolute one** (LLR units, i.e. scaled with the noise variance).
   Scaling by the mean |LLR| of the code word (what is available without a noise estimate)
   is unusable: the best target moves by 4× from case to case and 64QAM loses up to 1.6 dB.
   So the demapper needs σ² — **new item for the channel estimate**: a noise-variance estimate
   from the pilot residuals (the payload equivalent of the control field's |H|² path).
4. **Widths.** LLR 7 bit, APP (posterior) 10 bit, check-node messages 7 bit, α as
   m − (m >> 2): within the Monte-Carlo noise of float layered-10 in all 32 configurations
   at the right step (results/llr_step.jsonl). 6/8/6 is 0.1–0.3 dB worse at 20 copies; 8/10/8
   gives nothing more. APP 10 vs 8 bit matters only with 20 summed copies.
5. **LLR step per mode** (step = LLR units per integer LSB; 7-bit saturation at ±63 steps):

   | Mode | Step | Note |
   | --- | --- | --- |
   | everything except below | **1/4** | 1/8 equal within noise; 1/2 loses 0.1–0.3 dB |
   | 64QAM, rate 1/2 | **1** | 0.5–1 dB *better* than float (BLER 10 % → < 1 %) |

   The coarse step works like an offset min-sum on the over-confident max-log 64QAM LLRs
   (the integer m − m>>2 rounds small messages to zero). 64QAM at rates 2/3 and 3/4 prefer 1/4
   (checked at N 1296, 20 copies). The step is a 2-bit shift chosen per packet from the
   control field (modulation, rate) by a fixed table in the payload state machine.
6. Offset min-sum in general (all modes) would likely help a bit more; not needed now, a later
   option for the same hardware (one subtractor per message).

**Frozen for RTL:** layered normalised min-sum, α = 0.75 (m − m>>2), max 10 iterations, early
stop on zero syndrome, LLRs of the copies summed before the decoder (saturating at 7 bit),
LLR 7 / APP 10 / message 7 bit, LLR step 1/4 (64QAM rate 1/2: 1 — superseded by §6b: 1/4
in every mode), σ² from the payload channel estimate. Bit-true model: `decode_layered_fixed(code, quantise(llr_sum, 1/step, 7), 10, 10, 7)`.

**Effect on the sizing above:** 10 iterations instead of 15 raise the P = 54 throughput to
80–120 Mbit/s (need 43), so the decoder idles about half the time and a packet's last code
word is out sooner. Wider words (10-bit APP, 7-bit messages instead of 8 / 6) add about 15 %
to the decoder: **~13 k LUT**, BRAM unchanged (~7).

## 6. Grids for the channel estimate

The payload channel estimate interpolates between reference symbols (every RefSymbolSpacing =
1, 3, 6 or 12 symbols). Data symbols between two reference symbols need both, so up to
RefSymbolSpacing + 1 grids (13 at spacing 12) are held at once: about 13 BRAM36 at 841 × 36 bit.
The control field's fixed 8-tap FIR is not enough for the payload (long delay spread at
high rates); the payload uses window-select FIR8 plus smoothing across reference rows, §6a.

## 6a. Payload channel-estimate study (float, done Oct 2026)

Float Monte Carlo through the simulator's time-domain channel (`Delay_Doppler_Channel`, 20 Msps,
EPA-5 / EVA-70 / ETU-300 Hz and AWGN), 80 packets per point, the frozen decoder of §5b, σ² from
`estimate_noise_var`, SNR at 10 % code-word error. Results are within about ±0.2 dB
(Monte-Carlo noise). Raw data: `pl_sweep.jsonl` (64 configurations: all 16 pilot patterns,
QPSK 648_12, plus 16 / 64QAM and code 1944 at pattern 3/3) and `pl_sweep2.jsonl` (time
smoothing).

**Frequency.**
- The control field's fixed FIR8 (delay window −20 … 12 samples) is not enough for the payload.
  ETU spreads about 100 samples. At 3/3 the FIR8 loss against linear time interpolation is:
  - QPSK 648_12, ETU: 1.3 dB.
  - 16QAM ETU: 5.5 dB.
  - QPSK 1944_56 ETU: 6.9 dB.
  - 64QAM 648_12 EVA: 2.2 dB.
- **Window-select FIR8** fixes it. It keeps 4 fixed weight sets (delay windows ending at 12 /
  28 / 52 / 100, one ROM each), chosen per packet from the measured delay profile of the first
  reference row (as the adaptive LMMSE does).
  - It stays within 0.1–0.2 dB of linear time interpolation everywhere: 16QAM ETU 8.0 vs 7.9,
    QPSK 1944 ETU 8.1 vs 8.1, 64QAM EVA 12.7 vs 12.5.
  - In AWGN / EPA it picks the short window, so it equals FIR8 there.
  - 16 taps are no better than 8.
- Pilot spacing 12 and 24 cannot carry ETU-class delay spread: the alias-free window is
  1024 / spacing ≈ 85 / 43 samples. Those curves don't reach 10 % within the sweep, with any
  estimator.

**Time.**
- Linear interpolation loses to Wiener only when reference symbols are dense, because Wiener
  also averages noise across reference rows:
  - RefSymbolSpacing 1: 0.3–1.7 dB.
  - RefSymbolSpacing 3: 0.1–0.7 dB.
  - RefSymbolSpacing 6 and 12: about 0.
- A fixed smoothing across reference rows recovers it. After the frequency FIR, each reference
  row is replaced by a boxcar of its neighbours, phase-aligned with the common drift that the
  linear interpolator already measures, with the weights renormalised at the packet edges.

QPSK 648_12, pilot spacing 3, window-select FIR8, SNR at 10 % (dB):

| Channel, RefSymbolSpacing | Wiener LMMSE | linear LMMSE | none | 3 taps | 5 taps | 9 taps |
| --- | --- | --- | --- | --- | --- | --- |
| AWGN, 1 | −1.17 | −0.89 | −0.38 | −0.85 | −0.97 | **−1.05** |
| AWGN, 3 | −1.10 | −1.01 | −0.66 | −0.93 | **−1.02** | −1.09 |
| AWGN, 6 | −1.04 | −1.03 | −0.61 | **−0.94** | −1.02 | −1.10 |
| EPA-5, 1 | 3.34 | 3.50 | 3.89 | 3.54 | 3.42 | **3.36** |
| EPA-5, 3 | 3.62 | 3.75 | 4.11 | 3.85 | **3.77** | 3.61 |
| EPA-5, 6 | 5.32 | 5.32 | 5.50 | **5.39** | 5.31 | 5.26 |
| EVA-70, 1 | 2.06 | 3.20 | 3.59 | 2.70 | 2.36 | **2.12** |
| EVA-70, 3 | 2.77 | 2.96 | 3.17 | 2.89 | **2.80** | 2.70 |
| EVA-70, 6 | 3.27 | 3.30 | 3.49 | **3.22** | 3.10 | 3.14 |
| ETU-300, 1 | 1.89 | 3.45 | 3.53 | 2.47 | 2.24 | **1.99** |
| ETU-300, 3 | 2.80 | 3.39 | 3.46 | 2.91 | **2.68** | 3.14 |
| ETU-300, 6 | 3.14 | 3.24 | 3.28 | **2.87** | 3.05 | fails |

The smoothing must span no more than about 15–18 OFDM symbols: 9 taps at spacing 6 (54
symbols) fail at 300 Hz.

**Chosen estimator (bold above):**

| RefSymbolSpacing | Smoothing | Span |
| --- | --- | --- |
| 1 | 9 taps | 9 symbols |
| 3 | 5 taps | 15 symbols |
| 6 | 3 taps | 18 symbols |
| 12 | none | – |

The loss against Wiener LMMSE is −0.3 … +0.15 dB in every case measured. That is about Wiener
performance with fixed weights and no adaptation in time, so the "Wiener time" option is
covered. It stays a later option only if the high-order modulations show otherwise; the
smoothing is so far measured on QPSK only.

**Hardware consequences.**
- Frequency: 4 ROM sets of 8 weights per tone (per pilot spacing), plus the window choice from
  the first reference row, which is the delay-profile logic of the adaptive estimator in a
  simple form.
- Time:
  - A store of frequency-filtered reference rows: up to 10 rows of 841 × 36 bit, about 10 BRAM36.
  - The drift estimate, already in the control field.
  - A boxcar over up to 9 rows: adds only, no multiplies except the phase alignment; the
    renormalisation is a small table.
- **Latency:** a data symbol waits for the reference row half a smoothing span past the next
  one. That is 4 / 6 / 6 / 0 extra symbols at RefSymbolSpacing 1 / 3 / 6 / 12, so at most 6 ×
  1140 samples ≈ 0.34 ms at 20 Msps. A shorter smoothing trades dB for latency along the table
  above (3 taps: +2 / +3 / +6 symbols).
- Grids held: up to 13, as in §6. The noise variance comes from the empty comb and nulls
  (`estimate_noise_var`).

**Not done:**
- 12 configurations stopped at the time limit: more ETU pilot patterns, 16QAM EVA, 64QAM AWGN /
  ETU, QPSK 1944 EVA, 64QAM 1944 AWGN / ETU.
- Smoothing with 16 / 64QAM. Done in §6b: at high SNR it hurts, so §6b switches it off by
  pilot SNR.
- The bit-true widths. Done in §6b.

## 6b. Payload bit-true model (Oct 2026)

`PC/pl_reference.py` is an integer model of the payload receiver. Like the control-field model,
every step states its widths, rounding and saturation exactly, so the RTL can follow it bit for
bit. The full definition is in the module text. Tables are in `PC/pl_tables.npz`, written by
`gen_pl_tables.py`.

**Input and output.** It takes the payload grid (PacketLength × 841 bins from the shared FFT of
§5) and the control-field settings: modulation, pilot spacings, PacketLength, CBS, rate,
repetition, NDB_A and C_init. It returns each transport block's bits, CRC24 result and decoder
iterations. `payload_results()` runs it after `pd_reference.detect` on every accepted packet.

**Chain** (single Tx antenna):

1. **Packet shift.** The grid is shifted so the row-0 pilots fit 13 bits. In practice the shift
   is 0: the FFT output stays within 13 bits even at 6 dB input backoff.
2. **Pilots × known sign.** QAM pilots are multiplied by (1 + j). This leaves a known gain of
   2A/norm, which is folded into the LLR constants.
3. **Delay class (window choice).**
   - The input is a Hann-windowed copy of the pilots of reference rows 0 and 1, run through the
     shared FFT (one pass per row).
   - The last delay above max(peak − 30 dB, 8 × noise floor), plus 4 samples, picks one of
     4 windows ending at 24 / 40 / 64 / 112 samples in the hardware frame. Fewer windows exist
     at pilot spacing 12 (3) and 24 (1).
4. **SNR mode.** Pilot power against noise on rows 0 and 1:
   - Pilot SNR about 12 dB or more: high-SNR weights (design noise 0.05) and no smoothing.
   - Otherwise: low-SNR weights (design noise 0.5) and the smoothing of §6a.
5. **Frequency.** 8-tap FIR from ROM. There is one table per (pilot spacing, delay class, mode);
   the number of distinct coefficient sets is 25 / 49 / 97 / 193 for spacing 3 / 6 / 12 / 24.
6. **Time.**
   - Drift: `cordic_angle` of the pilot correlation between consecutive reference rows.
   - Rows phase-aligned, smoothed, interpolated linearly, then rotated back.
   - One table rotation per symbol and tone (the tone-check table).
7. **Noise.** Σ|Y|² over the empty Ref_P1 comb and the in-band DC tones of rows 0 and 1.
8. **LLR without a division per tone.**
   - Per tone: z = Y·conj(H) and e = |H|².
   - Per bit: max-log numerator N = min over the bit-1 levels − min over the bit-0 levels of
     m²·e − C1·m·z (C1 = 2 for BPSK, 4A for QAM).
   - One per-packet scale m·2^−sh = n0·2^(F+2) / (C2·ΣN), computed by one division per packet.
   - Per-copy LLR: 12 bits, at step 1/4 with 3 extra fraction bits.
9. **Descramble, combine, decode.**
   - Gold descrambling.
   - Copies summed in a 16-bit saturating accumulator, then rounded to the 7-bit decoder input
     (step 1/4).
   - De-interleave, then the frozen decoder of §5b: APP 10 bits, messages 7 bits, at most
     10 iterations, stopping per code word on a zero syndrome.
   - CRC24 over the first K bits.

**Changes against §5b and §6a found with the model:**
- **SNR mode (two weight sets).** §6a used a single design noise and always smoothed.
  64QAM 1944_34 on ETU-300 then failed even at 40 dB: the weights were biased, and the smoothing
  averaged over a channel that changes too fast at 300 Hz. With the mode switch it decodes, and
  QPSK keeps its smoothing gain. Cost: twice the weight ROM.
- **Delay class from two reference rows, with a Hann window.**
  - With one unwindowed row, the sinc sidelobes always chose the second window in AWGN, and
    noise peaks often chose the longest window at low SNR. That cost 0.3 dB on QPSK at −1 dB.
  - The cost of using two rows: symbol 0's output waits for reference row 1, which symbols
    1 … R−1 need anyway. The end of the packet is not delayed.
- **LLR step 1/4 in every mode.** §5b's step 1 for 64QAM rate 1/2 loses up to 1 dB end to end.
  Each pair below is 64QAM 648_12 at 10 % BLER, step 1 then step 1/4:
  - AWGN: 0.26 vs 0.06.
  - EVA-70, repetition 1: 0.18 vs 0.14.
  - EVA-70, repetition 19: 0.15 vs 0.12.

  §5b used an ideal AWGN demapper with exact σ²; with a real channel estimate and the per-tone
  σ²/|H|², 1/4 is best or equal in every case. This makes the per-packet step table of §5b
  unnecessary.
- **Grid validity.** PacketLength 4 with RefSymbolSpacing 6 or 12 is not a valid grid (it must
  end with a reference symbol; the simulator's Resource_Grid rejects it). The model rejects
  the packet; `cf_rx`'s field check must reject it too.

**Checks** (`PC/pl_check.py`):
- `tables`: CRC24 against the simulator's GenerateCRC. The test encoder (systematic, parity
  solved from H) gives zero syndrome for all 12 codes.
- `vector ../TestVectors/rep3.json`: the simulator's own packets through the bit-true detector
  and this model. The packets are BPSK 648_12, repetition 19, ETU, −8 dB, with all the
  simulator's defects. All 9 transport blocks pass CRC24. This confirms the scrambler,
  interleaver, repetition, pilot signs and bit mapping against the simulator's transmitter.
- `loop`: one packet per case through the time-domain channel at high SNR; every block decodes
  with the right bits. The cases are:
  - 4 modulations × 12 codes on AWGN;
  - all 16 pilot patterns on EPA-5;
  - every modulation at 1944_34 on ETU-300;
  - repetition 31 with PacketLength 121.
- `ranges` (word lengths):

  | Value | Largest seen (bits) | Allowed (bits) |
  | --- | --- | --- |
  | Y | 13 | 16 |
  | H | 14 | 16 |
  | z | 26 | – |
  | e | 27 | – |
  | N | 33 | – |
  | N·m | 49 | – |

  The cases were 7 configurations × input backoff 6 / 12 / 20 dB; nothing saturates before the
  LLR.

  Plan the 49-bit product N·m for the RTL: two DSPs, or truncate N first; the model will follow
  the RTL's choice.

**Bit-true against floating point** (`pl_check.py bler`).

The floating-point receiver runs the same algorithm (window-select FIR8, the same SNR mode and smoothing, max-log, float layered decoder) on unquantised samples, with the simulator's window choice and noise estimate. The bit-true model gets 12-bit samples at 12 dB backoff.

Each column is the SNR in dB at 10 % block error, on the same packets (20–40 packets per point, 1 dB steps, interpolated). The last two columns force one weight set and smoothing setting to show what the SNR mode chooses between.

| Modulation, code, repetition, pilot pattern, channel | Float | Bit-true | Forced low-SNR | Forced high-SNR |
| --- | --- | --- | --- | --- |
| QPSK 648_12, rep 1, 3/3, AWGN | -0.99 | -0.99 | -0.99 | -0.73 |
| QPSK 648_12, rep 1, 3/3, EVA-70 | 2.41 | 2.41 | 2.41 | 3.37 |
| QPSK 648_12, rep 1, 1/3, ETU-300 | 2.19 | 2.30 | 2.30 | 3.57 |
| QPSK 648_12, rep 1, 6/12, EPA-5 | 4.42 | 4.46 | 4.46 | 5.08 |
| QPSK 648_12, rep 1, 12/6, EVA-70 | 3.53 | 3.43 | 3.43 | 3.52 |
| BPSK 648_12, rep 3, 3/3, EVA-70 | -3.04 | -3.04 | -3.04 | -2.40 |
| 16QAM 648_12, rep 1, 3/3, EVA-70 | 7.71 | 7.68 | 7.68 | 8.01 |
| 16QAM 648_12, rep 1, 3/3, ETU-300 | 8.25 | 8.32 | 8.32 | 8.25 |
| 64QAM 648_12, rep 1, 3/3, AWGN | 8.26 | 8.28 | 8.23 | 8.29 |
| 64QAM 648_12, rep 1, 3/3, EVA-70 | 12.53 | 12.52 | 12.79 | 12.49 |
| 64QAM 1944_34, rep 1, 3/3, EVA-70 | 16.16 | 16.16 | 16.72 | 16.16 |
| QPSK 1944_56, rep 1, 3/3, ETU-300 | 7.40 | 7.48 | 7.48 | 7.65 |
| 64QAM 1944_34, rep 1, 3/3, ETU-300 | 16.52 | 16.56 | does not reach 10 % | 16.56 |
| 16QAM 1296_23, rep 1, 6/6, EVA-70 | 10.64 | 10.57 | 10.56 | 10.44 |

- **Fixed-point loss:** bit-true is within ±0.1 dB of float in every case. The largest gap is QPSK on ETU-300 with a pilot every symbol: +0.11 dB.
- **The SNR mode chooses well:** forcing low-SNR costs up to 0.6 dB at 64QAM and fails on 64QAM ETU-300; forcing high-SNR costs up to 1.3 dB at QPSK. The automatic choice is never more than about 0.1 dB from the better of the two.
- **Input backoff 6 / 12 / 20 dB:** QPSK AWGN −0.99 / −0.99 / −0.99 dB, 64QAM 648_12 AWGN (first run, step 1) 8.76 / 8.71 / 8.70 dB. The input level does not matter.

## 7. Resources (estimate)

| Item | BRAM36 | DSP |
| --- | --- | --- |
| input buffer 8192 → 16 384 | +6 | – |
| descriptors, keep pointer, drop logic | – | – |
| grid buffer (spacing ≤ 12) | about 13 | – |
| option C FFT (built, 6.0) | +6 | +4 |

Today: 34 BRAM tiles of 140 and 147 DSP of 220. The payload chain (channel estimate, equaliser
for 16 / 64QAM, demapper, LDPC) has to fit in the remaining 73 DSP; the demapper's division
by |H|² and the equaliser are the main users.

## 8. Open questions

Decided: LDPC with 54 lanes; every modulation and setting; no packet record for now; the
state machine re-arms (built); low latency first; PacketLength ≤ 73 normal.

1. ~~LDPC float-vs-bit-true study~~ — done, §5b. New: noise-variance estimate for the
   demapper's LLR scale.
2. ~~Shared fast FFT~~ — built (bitstream 6.0). Next: the grid buffer (grids instead of
   signatures, kept / dropped the same way) and the payload channel estimate with its noise
   estimate.
3. ~~Payload channel estimate~~ — float study done, §6a: window-select FIR8 + smoothing in
   time, about Wiener performance. Open: latency vs smoothing length (up to 0.34 ms).
4. ~~Payload bit-true model~~ — done, §6b. The RTL plan is `Specification_PayloadReceiver.md`
   (grid ring, estimator, demapper, LDPC, build steps 12a–12d), including the grid-validity check
   in `cf_rx`.

## 9. Verification (as before)

Model first (`pd_reference`: descriptors, drop reasons, re-arm points, exact sample indices),
then RTL against it in `tb_pd_detect.v` (every descriptor and its drop reason, the keep pointer
under fast DMA input and at exactly 20 Msps, a CRC failure and an invalid field on purpose),
fault injection, co-simulation, and the board regression with the new records.
