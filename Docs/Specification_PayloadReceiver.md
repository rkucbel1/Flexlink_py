# Flexlink Rx – Payload Receiver RTL Plan

30 Oct 2026. Draft for review, before the RTL. Follows `Specification_PayloadInput.md` (§5 built
FFT, §5a/5b LDPC, §6a channel estimate, §6b bit-true model).

## 1. Scope and starting point

The payload receiver takes the payload FFT bins that `cf_rx` already produces and returns the
transport blocks with their CRC24 results. The target is the bit-true model
`PC/pl_reference.py`: the RTL must reproduce its outputs bit for bit, like the detector and the
control field before it.

**Already built (bitstream 6.0):**
- **FFT bins.** `cf_rx` delivers the bins of every payload symbol on `pg_valid`, `pg_sym`,
  `pg_col`, `pg_re`, `pg_im`: 841 bins per symbol in column order, 18 bits each, about 700
  samples after the symbol's window.
- **Speculation and decision.** Up to 8 symbols are produced before the control field is
  decided. The decision then arrives on `pl_dec` / `pl_ok` / `pl_len`, together with the control
  record's 54 message bits.
- **Back-pressure.** `pl_room` holds the FFT when there is nowhere to put a symbol, and the
  8192-sample input buffer absorbs the wait.

**Scope:**
- Single Tx antenna (NoOfTxAntenna 2 is dropped by the field check).
- Every modulation, code and pilot pattern.
- Clock: 100 MHz (FCLK0), the same as the rest of the PL.

**Resources used by bitstream 6.0** (`HW/build/utilization.rpt`, routed, 30 Oct 2026):

| Resource | Used | Total | Free |
| --- | --- | --- | --- |
| LUT | 30 416 (4554 of them as memory) | 53 200 | 22 800 |
| BRAM36 tiles | 38.5 | 140 | 101.5 |
| DSP48 | 151 | 220 | 69 |

## 2. Data flow

```
 cf_rx front end (FFT)                                 pl_rx (new)
 ───────────────────────        ┌──────────────────────────────────────────────────────────────┐
 pg_* bins (841 / symbol) ─────►│ pl_grid  16-slot ring of grids (Y, 36 bit, 1 BRAM36 / symbol)│
 pl_dec/ok/len, message ───────►│ pl_ctrl  settings, valid grid, NDB_A, packet FSM             │
                                │    │ reference rows                                          │
 PDP job ◄──── pl_pdp ◄─────────│ pl_pilot  Hp = s·Y'(1+j), k, ΣN, pilot power (rows 0,1)      │
 (FFT pass between symbols)     │    ▼                                                         │
                                │ pl_fir   8-tap FIR per tone, ROM (S, class, mode)  → Hf rows │
                                │ pl_time  drift (CORDIC), align, smooth, interpolate, rotate  │
                                │    │ H per data RE                                           │
                                │    ▼                                                         │
                                │ pl_demap z = Y'H*, e = |H|², N·m, LLR (12 bit)               │
                                │ pl_comb  Gold descramble, copy sum (16 b), round to 7 b,     │
                                │          de-interleave → code-word ping-pong buffer          │
                                │    ▼                                                         │
                                │ pl_ldpc  layered min-sum, P = 54 lanes, ≤ 10 iterations      │
                                │ pl_crc   CRC24 → transport-block FIFO → AXI / RUN reply      │
                                └──────────────────────────────────────────────────────────────┘
```

The blocks follow the model's steps one to one. The numbers below are model steps (§6b; the
full text is in `pl_reference.py`). Every block is a pipeline with valid / ready handshakes.
Each unit has its own small state machine; `pl_ctrl` starts each unit per reference row, per
data symbol or per code word.

## 3. Throughput budget

**Budget.** At 20 Msps a payload symbol arrives every 5700 clocks. The chain must keep up on
average. The grid ring absorbs short bursts, and the FFT waits for room.

**Worst case.** RefSymbolSpacing 1 (every symbol is a reference row) at 64QAM: 6 bits on each
of up to 841 REs per symbol.

| Unit | Work | Clocks per symbol (worst case) |
| --- | --- | --- |
| `pl_pilot` | read 280 pilots of a reference row, Hp, ΣN | 841 (reads the whole row once) |
| `pl_fir` | 841 tones × 8 taps, 4 complex MACs per clock | 1 700 |
| `pl_time` (row part) | align (rotate) 841, smooth (T rows read in parallel) 841 | 1 700 |
| `pl_time` (symbol part) | interpolate + rotate, 1 tone / clock | 841 |
| `pl_demap` | 1 RE / clock, LLRs 2 bits / clock (64QAM: 3 clocks per RE) | 2 520 |
| `pl_comb` | 2 LLRs / clock | 2 520 (with the demapper) |
| `pl_ldpc` | 80–120 Mbit/s coded at 10 iterations, early stop | need ≤ 43 Mbit/s (§5a) |

The units run concurrently, so each has its own row of the table. The row path takes about
4200 clocks per row and the symbol path about 2500 clocks per symbol, both under 5700.
Most packets have one reference row every 3–12 symbols, so the row path is mostly idle.

**The delay profile (step 3) reuses the shared FFT.** It needs one pass per row for reference
rows 0 and 1 of each packet. `pl_pdp` hands `cf_rx` a "PDP job": 1024 inputs, the
Hann-windowed pilots and zeros, 12 bits. The front end runs it between two payload symbols:
load 1024 + FFT 2640 + read 124 bins ≈ 3800 clocks. The front end has about 1100 idle clocks
per symbol, so each job delays the next symbol by about 2700 clocks once. The input buffer
absorbs that, and the lag is made up within 2–3 symbols. This keeps the model's
`fft_fixed`-based profile exact and needs no second FFT. The alternative is a dedicated
coarse DFT of 62 delays × 280 pilots per row with 4 DSPs: about 17 k clocks per row and a
change to the model. It stays the fallback if the arbiter in `cf_rx` turns out to be risky.

## 4. Blocks

**`pl_ctrl` – settings and packet state machine.**
- At `pl_dec` with `pl_ok` it decodes the message into `PLConfig`: modulation, RefSymbolSpacing
  R, RefSignalSpacing S, PacketLength, N, rate, Rep, NDB_A, C_init.
- It checks that the grid is valid: (PacketLength − 1) mod R = 0. The same check is added to
  `cf_rx`'s field check, so an invalid grid is not accepted (model `valid_grid`; today PacketLength 4 with R = 6 / 12 gets
  through).
- It computes the number of code words: min(NDB_A, data bits / ((Rep + 1) N)). The number of
  data REs per symbol comes from a small table per (S, reference row or not).
- It sequences the units: rows 0 and 1 first (pilots, PDP, noise, SNR mode), then per
  reference row and per data symbol.
- It frees grid slots as soon as the time unit no longer needs them. On a drop it clears the
  ring.

**`pl_grid` – grid ring (16 slots).**
- Each slot holds the 841 raw FFT bins of one symbol: 36 bits (18 + 18), 1 BRAM36 (1024 × 36).
  The slot is symbol mod 16.
- Write side: `pg_*` from `cf_rx`. `pl_room` = a free slot exists.
- Read ports: the pilot unit (pilot columns of a reference row) and the demapper (data REs
  of a symbol, in column order).
- How many slots are needed: a data symbol between reference rows j and j + 1 waits for
  reference row j + 1 + T/2 (the smoothing look-ahead).

  | R | T (low-SNR mode) | Data symbol waits for | Grids held at most |
  | --- | --- | --- | --- |
  | 1 | 9 | row j + 5 | 6 |
  | 3 | 5 | row j + 3 | 10 |
  | 6 | 3 | row j + 2 | 13 |
  | 12 | 1 | row j + 1 | 13 |

  The control-field wait (at most about 6 symbols, at the start) overlaps with this. 16 slots
  leave room for the unit running one symbol behind.
- The packet shift k (step 1) is applied when a slot is read: Y' = sat16(rnd(Y >> k)). That is
  why the raw 18-bit bins are stored.

**`pl_pilot` – pilots, scale, noise, SNR mode (steps 1, 2, 4 (mode), 7).**
- For a reference row: reads the Ref_P0 columns (S − 1 + iS) and forms
  Hp = s·Y' (BPSK) or s·Y'(1 + j) (QAM), 17 bits, into a pilot row RAM (280 × 34 bits, LUTRAM
  or BRAM18). The pilot bits come from a ROM per S.
- Rows 0 and 1 only:
  - the max |Hp| of row 0 (gives k);
  - ΣN = Σ|Y'|² over the noise tones (P1 and in-band DC; a mask ROM per S);
  - n0, the number of noise tones;
  - the pilot power Σ|Hp|².
- From these, three per-packet results: the SNR mode (one multiply-compare), the LLR scale
  m·2^−sh (one 32-bit division per packet: a sequential divider, `pd_div16`-style, about 50
  clocks), and the noise threshold of step 3.

**`pl_pdp` – delay class (step 3).**
- Input: rows 0 and 1, conj(Hp)·win (win: Hann ROM per S, 15 bits; 2 DSP), sat12(rnd(·, 17)),
  sent to the FFT job port.
- Output: the 124 bins n = 4 … 127, p(n) = Xr² + Xi² (2 DSP or fabric), summed over the two
  rows (a 124-word buffer).
- Then the threshold max(peak >> 10, noise term), the last bin above it, + 4, and the class.

**`pl_fir` – frequency filter (step 5).**
- Hf(j, k) = sat16(rnd(Σ w·Hp >> 14)), 4 complex MACs per clock, 2 clocks per tone (16 DSP).
- Coefficient ROM:
  - 1560 sets × 8 taps × 32 bits (16-bit re/im; the largest is 17194), about 400 kbit,
    12 BRAM36.
  - Organised 4 taps wide (128 bits) with the set index per (S, class, mode) as the base
    address.
  - A tone → (set, first pilot) map per S: 841 × 17 bits × 4, 2 BRAM18. The map is the same for
    every class and mode (checked: `gen_pl_tables` sets share one grouping per S).
- Output: Hf rows into the row store.

**`pl_time` – drift, alignment, smoothing, interpolation (steps 5, 6).**
- **Drift.** D_j = Σ Hp(j + 1)·conj(Hp(j)) over the pilots: a complex MAC, 4 DSP, 48-bit
  accumulators. Then φ_j = `pd_cordic` (the detector's CORDIC, 16-bit output). Then the
  cumulative angle cum_j (16-bit wrap).
- **Row store.** Aligned rows A_j = sat16(rot(Hf_j, −cum_j)): one table rotation per tone,
  4 DSP. The rotation table is `pd_tone_rom`'s cos/sin, 10-bit address.
  - The store holds up to 10 rows of 841 × 32 bits, 1 BRAM36 each.
  - Each row has its own BRAM, so the smoothing reads all T rows in the same clock.
- **Smoothing.** Sm_j = sat16(rnd(Σ h_o A_j+o · RECIP[w], 15)): an adder tree over up to 9
  rows (weights 1 / 2 are shifts) and one multiply by RECIP (2 DSP). Sm rows go into a 3-row
  store, 3 BRAM36. In high-SNR mode T = 1 (Sm = A).
- **Interpolation, per data symbol.**
  - M = sat16(rnd(((R − t)·Sm_j + t·Sm_j+1)·RS_R, 15)), 4 DSP.
  - θ = cum_j + rnd(φ_j·t·RS_R, 15), once per symbol.
  - H = sat16(rot(M, θ)), 4 DSP.
  - t = 0 and t = R take the rows directly (model).
- Output: H of the symbol's data REs, in column order, aligned with the grid read.

**`pl_demap` – LLRs (step 8).**
- z = Y'·conj(H) and e = |H|²: 6 DSP (16 × 16 products).
- The scale m is applied per RE, not per bit. That makes it exact, because N is linear in
  (z, e): N·m = (m²e − C1·m·z) differences = combinations of e·m and z·m. So zm_r, zm_i and em
  are formed once per RE: 26–27 bits × 16 bits = 43 bits, 2 cascaded DSPs each, 6 DSP.
- Then, in fabric: per axis D(m) = m²·em − C1·m·zm for the 2 / 4 / 8 levels (constant shifts
  and adds, 50 bits), and per bit min(bit-1 levels) − min(bit-0 levels). Then rnd(·, sh) and
  sat12. This gives the model's `_satm(_rnd(N * m, sh), 12)` exactly, because
  N·m = min(D·m) − min(D·m) when m > 0.
- 2 LLRs per clock out (64QAM: 3 clocks per RE).

**`pl_comb` – descramble, combine, de-interleave (steps 9–11).**
- **Gold code.** Generated 2 bits per clock (an unrolled LFSR). The 1600-step start-up runs
  during rows 0 and 1, from C_init.
- **Copy sum.** acc[j] = sat16(acc[j] + Lf) over copies c = 0 … Rep in sequence order. The
  accumulator RAM is 1944 × 16 bits, 2 of them, 2 BRAM36: one fills while the other is read out.
- **Hand-over.** After the last copy each position j goes out as Lq = sat7(rnd(acc, 3)) and is
  written to code[il[j]]. The interleaver ROM holds all three sizes, 3888 × 11 bits, 2 BRAM18.
  It writes into the decoder's input ping-pong buffer (2 × 1944 × 7 bits, 1 BRAM36).
- **Padding.** Zero-padding bits after the last code word are skipped.

**`pl_ldpc` – decoder (step 12).**
- §5a architecture: layered normalised min-sum with P = 54 lanes, one circulant per clock and
  a cyclic shifter (27 / 54 / 81). APP memory is 10 bits and messages 7 bits. It stops a code
  word when its syndrome is zero, after at most 10 iterations.
- Estimate: about 13 k LUT and 7 BRAM36.
- To match the model bit for bit:
  - Layers are processed in prototype row order. Within a layer the order of rows does not
    matter, because each column appears once per layer.
  - The pipeline must not read an APP value that the previous layer has not yet written: stall
    on the hazard or reorder within a layer. Either way the model's sequential result is kept.
  - The minimum finder takes the first index of the minimum, as the model's stable sort does.
  - The syndrome is checked on the hard decisions after a full iteration.
- Output: the K message bits per code word, plus the number of iterations.

**`pl_crc` and the output.**
- CRC24 (Gen24, 1 bit per clock) over the K − 24 bits, compared with the last 24.
- A transport-block FIFO (2048 × 32 bits, 2 BRAM36) holds, per block: a header word (code-word
  index, crc_ok, iterations, K − 24) and the bits packed 32 per word (at most 50 words).
- New AXI registers TB_LEVEL and TB_DATA, like PL_LEVEL / PL_DATA. A full FIFO holds the
  decoder.
- The firmware adds the transport-block records to the RUN reply, after the payload signatures.

## 5. Resources (estimate)

| Block | BRAM36 | DSP | LUT |
| --- | --- | --- | --- |
| `pl_grid` (16 slots) | 16 | – | 0.3 k |
| `pl_pilot`, `pl_pdp`, pilot / mask / window ROMs | 2 | 4 | 1 k |
| `pl_fir` + coefficient ROM + tone map | 13 | 16 | 1 k |
| `pl_time` (row store 10, Sm 3; drift, rotations, interpolation) | 13 | 18 | 2 k |
| `pl_demap` | – | 12 | 3 k |
| `pl_comb` (accumulators, interleaver ROM, decoder input) | 4 | – | 0.5 k |
| `pl_ldpc` | 7 | – | 13 k |
| `pl_crc`, TB FIFO, AXI | 2 | – | 0.5 k |
| `pl_ctrl` | – | 1 | 1 k |
| **Total** | **about 57** | **about 51** | **about 22 k** |
| Free (bitstream 6.0) | 101.5 | 69 | 22.8 k |

BRAM and DSP fit. **LUTs do not fit as planned.** 22 k of 22.8 k free would put the device at
about 98 % LUT, which will not route at 100 MHz. A realistic ceiling is about 80–85 %, which
leaves roughly 12–15 k LUT for the payload receiver. Ways to get there, in this order:

1. **Measure before cutting.**
   - Step 12a's out-of-context synthesis gives the decoder's real size; the 13 k is an
     estimate.
   - `build_hw.tcl` now also writes `build/utilization_hier.rpt`, the use per block, and prints
     the totals at the end of the build.
2. **Move LUT memory to BRAM.** 4554 LUTs are used as memory today, and BRAM is the resource
   with most room (100 tiles free). Large LUTRAMs in the existing blocks (buffers, ROMs) can move
   to block RAM, with the same results and only the read latency to handle. The hierarchical
   report shows which ones are worth it. Potential gain: about 3–4 k LUT.
3. **P = 27 decoder lanes instead of 54:** about −6 k LUT.
   - Throughput at 10 iterations is about 51–72 Mbit/s at N = 648 (8 iterations, §5a), and less
     at 10.
   - The worst case needs 43 Mbit/s: 64QAM, Repetition 1, the largest grid.
   - Early stop lowers the average: 1–2 iterations at high SNR in `pl_check loop`. Near the
     10 % point it is higher, and that still has to be measured. If the average is low enough,
     P = 27 keeps up, with the grid ring and the decoder input buffer as slack.
   - This must be checked with the model's iteration counts before it is chosen.
4. **Smaller demapper and FIR:** 1 LLR per clock and 2 FIR MACs per clock save about 1.5 k LUT
   and 8 DSP. Both still keep up except at RefSymbolSpacing 1 with 64QAM; there the FFT is held
   and the input buffer fills.

## 6. Latency

- **During a packet.** A data symbol's LLRs are out about one symbol after the reference row it
  waits for (§4, `pl_grid` table).
- **After the packet ends**, the last symbol's FFT finishes about 35 µs later. Then the time
  unit and demapper must work through the last R symbols, then the last code words are decoded:

  | Case | Last R symbols | Total until the last transport block is out |
  | --- | --- | --- |
  | QPSK, R = 3 | ≈ 3 × 841 clocks | ≈ 0.1 ms |
  | 64QAM, R = 12 | ≈ 12 × 2520 clocks | ≈ 0.4 ms |

  The decoder adds at most a few code words of ≤ 2500 clocks each. If 0.4 ms matters, a
  4-LLR-per-clock demapper (+6 DSP) halves the 64QAM case.
- **At the start.** Symbol 0 waits for reference row 1 (§6b) and for the control record, which
  arrives about 3–6 symbols into the payload. Only the start of the output moves; the end does
  not.

## 7. Changes to existing blocks

| Block | Change |
| --- | --- |
| `cf_rx` | PDP job port: up to 2 FFT passes per packet between payload symbols; grid-validity check in the field check (`pl_ok`) |
| `pd_cf` | `pg_*`, `pl_dec` and the message go to `pl_rx`; the signature FIFO stays as a check (it now also sees `pl_room` from the grid ring) |
| `pd_detect_top`, `pd_axi` | instantiate `pl_rx`; TB_LEVEL / TB_DATA registers; status bits; bitstream ID 7.x |
| Firmware | `drain_tb`, RUN reply with transport-block records, `BOARD_TB` flag |
| PC | `pd_protocol.TBRecord`; `pd_reference.detect` fills `Result.blocks` from `pl_reference.payload_results`; `pd_hw_test` / regression compare block bits, crc_ok and iterations exactly |
| Model | `pl_reference.receive_debug` (every intermediate per row / symbol / code word, for the testbenches); the PDP-job and interpolation timing do not change results |

## 8. Build order

Each step ends with something that is checked on the board bit for bit, as before. The LDPC
decoder is the biggest risk (size, 100 MHz timing, hazards), so it starts first, on its own.

| Step | What | Checked by |
| --- | --- | --- |
| **12a** | `pl_ldpc` stand-alone: RTL, unit testbench with code-word vectors from `pl_reference` (all 12 codes, noisy and clean, early stop, saturation), out-of-context synthesis for LUT and timing | xsim, bit for bit (hard bits and iteration counts); WNS ≥ 0.3 ns |
| **12b** | `pl_grid`, `pl_pilot`, `pl_pdp` (with the `cf_rx` PDP job), `pl_fir`, `pl_time`, `pl_ctrl`: the channel estimate. A per-symbol signature of H replaces the bin signature in the RUN reply | `tb_pd_detect` and the board regression: every H signature exact, plus the delay class and SNR mode per packet |
| **12c** | `pl_demap`, `pl_comb`: per-code-word signature of the decoder input (Lq) | testbench + board, exact |
| **12d** | `pl_ldpc` + `pl_crc` + TB FIFO, firmware RUN reply with transport blocks | board: every transport block, crc_ok and iteration count equal to the model; regression with real packets (rep3, all modulations from `pl_check`'s generator through `pd_make_vector`) |

Each step also gets fault injection (mutants that the tests must catch), co-simulation with the
firmware, and the regression on the board, as for steps 9–11.

The test vectors for 12b–12d need packets with real payloads at every modulation. `pl_check.py`
can generate them. A small addition writes them as `.h5` waveforms with preamble and control
field, built with the simulator's own Tx chain (`Run_Tx_Chain.py` with a parameter file per
case), so `pd_make_vector.py` can turn them into board vectors.

### Step 12a status (30 Oct 2026): done – decoder core built, bit-exact in simulation, 100 MHz met

`RTL/pl_ldpc_core.v`, with the code tables in `RTL/pl_ldpc_rom.v` (generated by
`gen_pl_ldpc_rom.py`).

**What it is.** One code word at a time with **27 lanes**, for every N and rate. 27 divides
Z = 27 / 54 / 81, so one core design serves every code. It replaces the 54-lane idea of §4:
54 lanes would have to run two 648-bit code words side by side. To get 54 lanes, two cores take
code words in turn; that is a wrapper and not yet needed.

**Verification.**
- `SIM/run_ldpc.sh`: 540 code words, all 12 codes, noisy and extreme inputs, iterations 1–10.
  Every hard bit and every iteration count equals `pl_reference.decode_ldpc`. The runs use
  random input gaps and output back-pressure.
- `SIM/ldpc_mut.py`: 12 of 12 injected faults caught. The docstring lists 4 mutants that cannot
  change a result and 1 that cannot be reached.

**Bit-exact details.**
- Layers are processed in the model's order. Within a layer the circulants are reordered for
  speed, which cannot change the result.
- A column is read only after the previous layer has written it back (a 24-bit scoreboard per
  layer parity).
- The syndrome is checked on the hard decisions after each full iteration. Meanwhile the next
  iteration's read pass of the first layer already runs; it writes nothing, so a stop leaves no
  trace.

**Throughput** (simulation, 100 MHz, clocks per iteration including the syndrome check):

| Code | Clocks per iteration | 10 iterations | 6 iterations | 3 iterations |
| --- | --- | --- | --- | --- |
| 648, rate 1/2 | 242 (254 after the timing fix) | 27 Mbit/s | 45 Mbit/s | 89 Mbit/s |
| 648, rate 5/6 | 184 | 35 Mbit/s | 59 Mbit/s | 117 Mbit/s |
| 1944, rate 1/2 | 519 | 37 Mbit/s | 62 Mbit/s | 125 Mbit/s |
| 1944, rate 5/6 | 437 | 44 Mbit/s | 74 Mbit/s | 148 Mbit/s |

- The absolute worst case needs 43 Mbit/s: 64QAM, Repetition 1, the largest grid.
- Iterations measured with `pl_reference` at 64QAM 648_12, Repetition 1:

  | Condition | Block error | Mean iterations |
  | --- | --- | --- |
  | AWGN, near the 10 % point | 7 % | 6.1 |
  | EVA-70 | 8 % | 3.3 |
  | 2 dB above | – | about 2 |

  With early stop, one core keeps up at and above the 10 % point.
- Below that point most code words take all 10 iterations and fail anyway. Then the decoder
  falls behind: at most by about a third for N = 648, and the grid ring and input buffer fill.
  A second core (2 × the LUTs) removes that.

**Memory per core** (estimate, to be confirmed by the synthesis):

| Memory | BRAM36 |
| --- | --- |
| APP banks (27 × BRAM18) | 13.5 |
| Compressed messages (459 bits wide) | 7 |
| q FIFO (270 bits wide) | 4 |
| Signs | 0.5 |
| Edge ROMs (2 × 2048 × 18) | 2 |
| **Total** | **about 27** |

That is 20 more than §5 assumed; the BRAM total is then about 77 of the 101.5 free. The wide
memories are BRAM because LUTs are the scarce resource.

**Synthesis, first run** (out of context, 30 Oct; the BRAM matches the estimate):

| LUT | LUT as memory | FF | DSP | BRAM tiles |
| --- | --- | --- | --- | --- |
| 8730 | 216 | 6030 | 0 | 26.5 |

- WNS was −0.352 ns. The failing path ran from P2's selection of the row group's minima,
  through the scaling and the add, to the saturated new APP value: 12 logic levels.
- That stage is now split in two (T2 / T2b). Results are unchanged: 540 / 540 code words and
  12 / 12 mutants caught. It costs about 5 % throughput: 254 instead of 242 clocks per iteration
  for 648, rate 1/2.
- Second run: **WNS +0.133 ns**; LUT 8699, FF 6820, BRAM 26.5. The worst paths were the
  syndrome's end-of-iteration decision (hard-bit read → rotation → parity → stop, 10 levels)
  and P1's minimum update (+0.28 ns). The syndrome now has one more register stage, and |q| and
  its sign come from the subtraction in P1's stage 3 instead of stage 4. Still 540 / 540 and
  12 / 12; 255 clocks per iteration.
- Third run: **WNS +0.343 ns, timing met with the margin**. LUT 8705 (216 as memory),
  FF 7060, DSP 0, BRAM 26.5. **Step 12a is done.**
- 8.7 k LUT is well under the 13 k estimate, so **one core fits the LUT budget** of §5
  (12–15 k for the whole payload receiver is still tight, but the decoder is no longer the
  problem).

### Step 12b status (31 Oct 2026): done – channel estimate bit-exact on the board

**Built.**
- `pl_chest.v` is the whole channel estimate as one sequential engine (instead of the six
  blocks of §4). It has a 16-symbol grid ring with a 2-deep packet queue, then k, the pilots,
  the delay profile, class, SNR mode, noise sum, FIR8, drift, smoothing and time interpolation.
  ROMs come from `gen_pl_roms.py`.
- The delay profile's two FFTs run on `cf_rx`'s FFT through a job port, between symbols.
- `pd_cf` adds the signature of H per symbol and a 16-record CH FIFO.
- `pd_axi` has CH_LEVEL 0x64 and CH_DATA 0x68, ID 7.0. Firmware build 14 adds them to the RUN
  reply (`BOARD_CHEST`).
- The PC tools compare the records (`pd_hw_test` "chest", regression column "chest").
- The bin signatures of step 11 stay in the reply as well, not replaced.
- The field check is stricter: PacketLength 4 needs RefSymbolSpacing 1 or 3, and only
  NoOfTxAntenna 1 is accepted.

**Checked.**
- `tb_pl_chest`: 16 / 16 packets bit-exact (every H, Y' and packet record), gaps 0, 1 and 3.
  `chest_mut.py` catches every mutant.
- `tb_pd_detect`, all exact:
  - rep3: 222 records.
  - cfmod: 38.
  - highsnr at 40 Msps: 74.
  - stress, with and without re-arm.
  - slow reader (PLGAP 20000).
- `run_cf` passes.
- Firmware + RTL co-simulation: `pd_hw_test` rep3 PASS; regression rep3_s1, cfmod_s1,
  pieces_s1 (8 uneven RUNs) and pieces_s1_end PASS.

**Speed.**
- At RefSymbolSpacing 3 a packet of 37 symbols takes about 3.1 k clocks per symbol (real time
  is 5.7 k).
- At RefSymbolSpacing 1 it takes about 6.6 k, slower than real time. The FFT and then the input
  are held back, and nothing is lost. A packet of 73 symbols at R 1 ends about 0.6 ms late.
- Speeding up the per-row FIR (2 taps per clock now) is a later upgrade if step 12c needs it.

**Synthesis** (`synth_detect.tcl`, pd_detect_top out of context, 30 Oct 2026):
- First run: WNS −7.5 ns. `pl_chest` had a division, modulos, a 10-input weighted sum and
  memory outputs feeding multipliers in single clocks.
- After the rework (memory outputs registered, one add or multiply per stage, running counters
  instead of divisions; still bit-exact): **WNS +0.144 ns, timing met**. LUT 29 138 (3817 as
  memory), FF 26 175, DSP 193, BRAM 89.
- Against bitstream 6.0's detector the channel estimate costs about 40 DSP and 50 BRAM tiles
  (16-symbol grid ring and FIR coefficient ROM most of it). With the LDPC core the PL is near
  117 of 140 BRAM and 193 of 220 DSP: steps 12c and 12d must be frugal with both (the grid
  ring can drop to 8 symbols if needed).

**Board build** (`build_hw.tcl`, bitstream 7.0, 31 Oct 2026): **WNS +0.213 ns**. Used: LUT
34 625 of 53 200, BRAM 91.5 of 140, DSP 193 of 220. Adding the LDPC core (8.7 k LUT, 26.5 BRAM,
0 DSP) leaves about 9.9 k LUT, 22 BRAM and 27 DSP for the demapper, de-interleaver, CRC and
control (steps 12c, 12d).

**Board** (bitstream 7.0 built 31 Oct 00:10 from the reworked RTL, firmware build 14):
`pd_hw_test` rep3 PASS, every channel-estimate record exact (3 packets: k 0, class 0, low-SNR
mode, n0 570, 73 H signatures each). `pd_board_regression`: 39 / 39 PASS; channel-estimate
records exact in rep3 and defects (20 and 40 Msps), cfo_s1_rearm (148), cfmod (38) and the
split RUNs (pieces). **Step 12b is done.**

### Step 12c status (31 Oct 2026): done – decoder input bit-exact on the board

**Built.**
- `pl_demap.v`: z, e (6 DSP), the 8 parabola values D(m) per axis, thresholds A z < j e, N for
  every bit of the RE in parallel, then two scaling lanes (N m: 2 × 2 DSP, shift by sh,
  rounding, sat12). One RE per 1 (BPSK, QPSK), 2 (16QAM) or 3 (64QAM) clocks.
- `pl_comb.v`: Gold code from a C_init-dependent start state (no 1600-step run-in), copy sums
  in two banks (1024 × 32 each, 2 BRAM36), read-out through the inverse interleaver ROM
  (4096 × 11, dual port), per code-word signature record.
- `pl_chest` sends H with credits (16), so the demapper can be slower than one RE per clock:
  per symbol HOUT then takes 841, 1682 or 2523 clocks.
- `pd_cf`: LQ record FIFO; `pd_axi`: LQ_LEVEL 0x6C / LQ_DATA 0x70, ID 8.0; firmware build 15
  (`BOARD_LQ`); PC tools compare the records (`pd_hw_test` "decoder", regression column "lq").
- The decoder input stream (`lq_*`, two values per clock in code order) is ready for step 12d
  (`lq_ready` tied to 1 for now).

**Checked.**
- `tb_pl_soft`: 12 packets, 160 code words, every Lq and every record exact; gaps 0, 1, 3;
  also under Icarus (unwritten memory as X). `soft_mut.py`: 19 / 19 faults caught.
- `tb_pd_detect`: rep3 (9 code-word records), cfmod (1), cfo (6), highsnr at 40 Msps (3),
  slow reader, stress with and without re-arm: all exact. `run_chest.sh` still passes.
- Firmware + RTL co-simulation: `pd_hw_test` rep3 PASS (build 15); regression rep3_s1,
  cfo_s1_rearm, cfmod_s1, pieces_s1, pieces_s1_end PASS.

**Synthesis** (`synth_detect.tcl`, 31 Oct 2026): **WNS +0.116 ns, timing met**. LUT 34 195
(4014 as memory), FF 29 880, DSP 205, BRAM 93. Against step 12b: +5.1 k LUT, +12 DSP, +4 BRAM
(more LUT than the 3–4 k estimate: the parallel parabola values and the two 61-bit shifters).
Projected whole PL with the LDPC core: about 48 k of 53.2 k LUT, 205 of 220 DSP, about 122 of
140 BRAM. Step 12d (CRC, transport-block FIFO, glue) fits, but with little margin; if the
board build gets tight, the demapper can compute one axis per clock (about −1 k LUT).

**Board** (bitstream 8.0, WNS +0.016 ns; firmware build 15): `pd_hw_test` rep3 and
`pd_board_regression` PASS, every decoder-input record exact. The worst paths were the scale
divider's variable shift; it now shifts by one per step (in the next build). **Step 12c is done.**

### Step 12d status (31 Oct 2026): done – on the board

**Built.**
- `pl_dec.v`: the combiner's Lq (2 per clock, code order) gathered 27 per word into the step-12a
  core (one code word at a time; the combiner's two banks are its input double buffer), CRC24 on
  the first K bits (27 bits per clock), the transport block packed 32 bits per word into a
  2048-word FIFO. A record = t_trig, header {crc_ok, iterations, w, nbits}, the bits; the
  FIFO shows only complete records, and a code word starts only with room for its record, so
  the decoder output never waits.
- `pd_axi`: TB_LEVEL 0x74 / TB_DATA 0x78, ID 9.0; firmware build 16 (`BOARD_TB`) adds the
  blocks to the RUN reply (variable length, `pd_protocol.TBRecord`); `pd_hw_test` "blocks",
  regression column "blocks" (blocks / CRC ok).
- Model: `pl_reference.tb_records` (the code words of `lq_records`, decoded).
- System vectors with every modulation: `make_tb_vectors.mods_vector` replaces rep3's packets
  (control field and payload) with `pl_check` packets: `mods` (QPSK 1296 2/3, 16QAM 648 3/4,
  64QAM 1944 5/6), `mods2` (BPSK 4 copies R 1, 64QAM 648 1/2 at 10 dB with CRC failures,
  16QAM 1296 5/6). Board regression cases `mods_s1`, `mods2_s1`, `mods_pieces_s1`.

**Checked.**
- `tb_pl_soft` (demapper + combiner + decoder): 16 packets, all 12 codes, 184 code words:
  every Lq, record and transport block (bits, crc_ok, iterations) exact; gaps 0, 1, 3.
  `soft_mut.py`: 25 / 25 faults caught (6 in `pl_dec`).
- `tb_pd_detect`: rep3 (9 blocks), mods (192, all CRC ok), mods2 (255, 217 CRC ok), cfmod,
  cfo, highsnr, slow reader: every block exact. In cfo and highsnr the decoded blocks fail
  their CRC in the model too: those vectors' packets are cut short by the next packet, so the
  later symbols hold another packet's samples.
- Firmware + RTL co-simulation: `pd_hw_test` rep3 PASS (build 16); regression cfmod_s1,
  mods_s1, mods2_s1, mods_pieces_s1, pieces_s1, pieces_s1_end PASS.

**Synthesis** (`synth_detect.tcl`, 31 Oct 2026): first run WNS −0.49 ns (67 endpoints): the
transport-block packer (take, mask, variable shift and merge in one clock) and two LDPC-core
paths at −0.1 ns (congestion; the core alone had +0.34 ns). The packer now takes its mask one
word ahead and packs in a second stage; implementation is timing-driven (ExtraTimingOpt
placement, physical optimisation after placement and routing, in `synth_detect.tcl` and
`build_hw.tcl`). Second run: **WNS +0.171 ns, timing met**. LUT 43 762 (4232 as memory),
FF 37 852, DSP 205, BRAM 121.

**Board** (31 Oct 2026). The first `build_hw.tcl` run failed in placement: the unplaced
instances needed 7252 slices with 7091 free (13 300 in the device; 1154 control sets). The 2:1
smartconnect between the DMA and DDR (about 2100 LUT) was replaced by two AXI4 → AXI3 protocol
converters: DMA reads (MM2S) on HP0, writes (S2MM) on HP1, both 32 bit (400 MB/s each way; the
samples need 80 MB/s). No RTL or firmware change. Second run: **WNS +0.100 ns**, LUT 47 290 of
53 200 (89 %), BRAM 123.5 of 140, DSP 205 of 220. Firmware build 16: `pd_hw_test` rep3 and
`pd_board_regression` (all cases, transport blocks included) PASS.

The device is now nearly full (89 % LUT, 88 % BRAM, 93 % DSP); further PL work needs area
savings first (candidates: axis-serial demapper about 1000 LUT, `pd_timing` lane count).

## 9. Open points

1. Wiener time interpolation stays a later option (§6a / §6b). The row store and smoothing
   unit is where it would go.
2. A per-RE N·m product is chosen over truncating N. It keeps the model exact; truncation would
   save about 6 DSP if they are needed.
3. LUT budget (§5). Decide between 54 and 27 decoder lanes after step 12a's synthesis and the
   per-block report of the next build.
