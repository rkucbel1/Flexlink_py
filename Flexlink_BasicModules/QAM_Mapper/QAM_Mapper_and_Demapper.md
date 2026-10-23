# IEEE 802.11 QAM Mapper and Demapper

Python implementation of the constellation mapper and demapper used in the IEEE 802.11 OFDM PHYs (802.11a/g/n/ac/ax). Supports BPSK, QPSK, 16-QAM and 64-QAM.

| File | Purpose |
|---|---|
| `qam_mapper.py` | Bits to complex constellation symbols (`qam_map`) |
| `qam_demapper.py` | Received symbols to hard bits or soft LLRs (`QAM_Demapper`) |

Both files must be in the same folder, because the demapper imports the constellation tables from the mapper. The only dependency is `numpy`.

---

## 1. Constellation definition

Mapping follows the Gray-coded tables in IEEE 802.11-2020, Clause 17.3.5.8. Within each group of bits for one symbol:

- The **first bit is the MSB**. Bits `b0 ... b(n-1)` are taken in the order they appear in the stream.
- The first half of the bits select the **I (real)** amplitude and the second half select the **Q (imaginary)** amplitude. BPSK has only the real axis.
- Every point is multiplied by a normalisation factor `K_MOD` so the average symbol power is 1.

| Modulation | Bits per symbol | Per-axis bit group to amplitude | K_MOD |
|---|---|---|---|
| BPSK | 1 | `0` to -1, `1` to +1 (I only) | 1 |
| QPSK | 2 | `0` to -1, `1` to +1 | 1/√2 |
| 16-QAM | 4 | `00` to -3, `01` to -1, `11` to +1, `10` to +3 | 1/√10 |
| 64-QAM | 6 | `000` to -7, `001` to -5, `011` to -3, `010` to -1, `110` to +1, `111` to +3, `101` to +5, `100` to +7 | 1/√42 |

**Example (16-QAM):** bits `0010` give I from `00` = -3 and Q from `10` = +3, so the symbol is `(-3 + 3j) / √10 = -0.9487 + 0.9487j`.

---

## 2. Mapper: `qam_map`

```python
from qam_mapper import qam_map

Output_QAM = qam_map(Bitstream_In, modulation)
```

### Parameters

| Name | Type | Description |
|---|---|---|
| `Bitstream_In` | list, tuple, numpy array of 0/1, or a string such as `"101100"` | Input bits. Length must be a multiple of the bits per symbol. |
| `modulation` | str | `"BPSK"`, `"QPSK"`, `"16QAM"` or `"64QAM"`. Case, spaces, dashes and underscores are ignored, so `"16-QAM"` works. |

### Returns

`Output_QAM`: 1-D complex numpy array, one element per symbol, already scaled by `K_MOD`.

### Errors (`ValueError`)

- Unsupported modulation name.
- Bitstream contains values other than 0 and 1.
- Bitstream length is not a multiple of the bits per symbol.

### Command line

```bash
python qam_mapper.py 64QAM 000100010110   # map a bit string
python qam_mapper.py --selftest           # run built-in checks
```

---

## 3. Demapper: `QAM_Demapper`

```python
from qam_demapper import QAM_Demapper, llr_to_bits

Bitstream_Out = QAM_Demapper(Input_QAM, modulation, mode="hard", noise_var=1.0)
```

### Parameters

| Name | Type | Description |
|---|---|---|
| `Input_QAM` | array-like of complex | Received symbols, on the same unit-power scale that `qam_map` produces. |
| `modulation` | str | Same names as the mapper. |
| `mode` | str | `"hard"` (default), `"maxlog"` or `"llr"`. See below. |
| `noise_var` | float | Total complex noise variance. Used by the soft modes only, and must be greater than 0. |

### Returns

`Bitstream_Out`: 1-D array of length `len(Input_QAM) * bits_per_symbol`, in the same bit order as the mapper input.

### Modes

| Mode | Output type | Method |
|---|---|---|
| `"hard"` | int, 0 or 1 | Nearest constellation point (minimum Euclidean distance). `noise_var` is ignored. |
| `"maxlog"` | float LLRs | Max-log-MAP approximation. Cheap, and commonly used in hardware. |
| `"llr"` | float LLRs | Exact log-MAP LLR. |

### LLR sign convention

```
LLR_k = ln( P(bit_k = 0 | y) / P(bit_k = 1 | y) )
```

- **Positive** LLR: bit 0 is more likely.
- **Negative** LLR: bit 1 is more likely.
- The magnitude is the confidence.

Some decoders (Viterbi, LDPC) expect the opposite sign. If yours does, negate the LLRs before passing them on.

`llr_to_bits(llr)` converts LLRs to hard bits: `bit = 1` if `LLR < 0`, else `0`.

### Noise variance

`noise_var` is the **total complex** noise variance, `σ² = E[|n|²]`, which is `σ²/2` per real dimension. It is relative to the unit-average-power constellation.

For a target SNR with unit signal power:

```python
noise_var = 10 ** (-SNR_dB / 10)
```

`noise_var` only scales the LLRs. It does not change their signs, so hard decisions derived from the LLRs are unaffected. It does matter when a decoder uses LLR magnitudes.

If the receiver applies per-subcarrier channel equalisation, the effective noise variance differs per subcarrier. In that case, scale the LLRs per symbol yourself, or pass an appropriately scaled value. The current implementation takes a single scalar.

### Exact vs max-log

The exact LLR for bit `k` is

```
LLR_k = ln( sum over s in S0 of exp(-|y - s|^2 / σ²) ) - ln( sum over s in S1 of exp(-|y - s|^2 / σ²) )
```

where `S0` and `S1` are the constellation points whose bit `k` is 0 and 1 respectively. The max-log version replaces each sum with its largest term:

```
LLR_k ≈ ( min over s in S1 of |y - s|^2  -  min over s in S0 of |y - s|^2 ) / σ²
```

The sums are computed with a numerically stable log-sum-exp. Max-log is slightly more optimistic at low SNR and converges to the exact result at high SNR.

---

## 4. End-to-end example

```python
import numpy as np
from qam_mapper import qam_map
from qam_demapper import QAM_Demapper, llr_to_bits

rng = np.random.default_rng(0)
Bitstream_In = rng.integers(0, 2, 6000)

# Transmit
Output_QAM = qam_map(Bitstream_In, "64QAM")

# Channel: AWGN at 25 dB SNR
snr_db = 25
noise_var = 10 ** (-snr_db / 10)
noise = np.sqrt(noise_var / 2) * (rng.standard_normal(Output_QAM.size)
                                  + 1j * rng.standard_normal(Output_QAM.size))
Input_QAM = Output_QAM + noise

# Receive: hard decisions
Bitstream_Out = QAM_Demapper(Input_QAM, "64QAM", mode="hard")
print("BER (hard):", np.mean(Bitstream_Out != Bitstream_In))

# Receive: soft LLRs (feed these to a channel decoder)
llrs = QAM_Demapper(Input_QAM, "64QAM", mode="maxlog", noise_var=noise_var)
print("BER (from LLR signs):", np.mean(llr_to_bits(llrs) != Bitstream_In))
```

---

## 5. Testing

Each file has a built-in self-test:

```bash
python qam_mapper.py --selftest
python qam_demapper.py
```

The mapper test checks known constellation points from the standard, and confirms that every modulation has unit average power.

The demapper test covers, for all four modulations:

- Noiseless round trips in all three modes.
- Agreement between hard decisions and LLR signs at about 20 dB SNR.
- High correlation between max-log and exact LLRs.
- The LLR sign convention (BPSK `+1` gives a negative LLR, `-1` gives a positive LLR).

---

## 6. Scope and limitations

- This covers **mapping and demapping only**. Interleaving, pilot insertion, subcarrier allocation, scrambling and channel coding are not included.
- Higher-order modulations used in newer amendments (256-QAM, 1024-QAM, 4096-QAM in 802.11ac/ax/be) are not implemented.
- The demapper evaluates the distance to every constellation point (at most 64), which is simple and exact but not optimised for very long streams. For large simulations, process the stream in chunks, because the distance matrix has size `N x 2^bits_per_symbol`.
- The noise variance is a single scalar for all symbols.
