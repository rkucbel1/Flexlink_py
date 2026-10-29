# Flexlink Rx – Payload Input Buffer and Packet Descriptor (draft)

Status: proposal, October 2026; the re-arm state machine (section 4) is built (bitstream 5.0,
firmware build 12). Follows `Specification_ControlField.md`; nothing here is built
yet. It fixes how the payload receiver finds its first sample and its settings, what happens
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

**Next study (bit-true, like the control field):** layered vs flooding iterations for the same
BLER as the simulator, LLR and message widths (probably 6–8 bits), offset or normalised
min-sum, early stop, LLR combining of the copies instead of averaged probabilities, per
modulation, N and rate at the SNRs where the simulator decodes.

## 6. Grids for the channel estimate

The payload channel estimate interpolates between reference symbols (every RefSymbolSpacing =
1, 3, 6 or 12 symbols). Data symbols between two reference symbols need both, so up to
RefSymbolSpacing + 1 grids (13 at spacing 12) are held at once: about 13 BRAM36 at 841 × 36 bit.
The control field's estimator (8-tap FIR, drift-corrected linear in time) carries over for the
payload's spacing 3; for 6, 12 and the other pilot spacings the weights and their window are a
study of their own (payload spec).

## 7. Resources (estimate)

| Item | BRAM36 | DSP |
| --- | --- | --- |
| input buffer 8192 → 16 384 | +6 | – |
| descriptors, keep pointer, drop logic | – | – |
| grid buffer (spacing ≤ 12) | about 13 | – |
| option B / C FFT | 0–2 | +4 |

Today: 34 BRAM tiles of 140 and 147 DSP of 220. The payload chain (channel estimate, equaliser
for 16 / 64QAM, demapper, LDPC) has to fit in the remaining 73 DSP; the demapper's division
by |H|² and the equaliser are the main users.

## 8. Open questions

Decided: LDPC with 54 lanes; every modulation and setting; no packet record for now; the
state machine re-arms (built); low latency first; PacketLength ≤ 73 normal.

1. LDPC float-vs-bit-true study (iterations, widths, LLR combining of the copies).
2. Shared fast FFT (option B engine) for the control field and the speculative payload FFTs.

## 9. Verification (as before)

Model first (`pd_reference`: descriptors, drop reasons, re-arm points, exact sample indices),
then RTL against it in `tb_pd_detect.v` (every descriptor and its drop reason, the keep pointer
under fast DMA input and at exactly 20 Msps, a CRC failure and an invalid field on purpose),
fault injection, co-simulation, and the board regression with the new records.
