#!/usr/bin/env python3
"""make_ifft_vectors.py - IFFT unit-test vectors from ptx_reference.ifft_fixed (SIM/work/ifft/).

  in.txt   one line per input bin: "xr xi" (natural order), symbol after symbol
  exp.txt  one line per output slot: "yr yi" (bit-reversed order, as the SDF IFFT outputs them)
  cfg.txt  number of symbols, seed for the testbench's clock-enable gaps

Symbols: random symbols of every modulation (with pilots), the control field, full-scale
random bins (saturation exercised in the first stages) and single tones.
"""
import sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "PC"))
import ptx_reference as R

def main(seed=1, gap=3):
    rng = np.random.default_rng(seed)
    xs = []
    for mod in ("BPSK", "QPSK", "16QAM", "64QAM"):
        cfg = R.TxConfig(mod=mod, ref_sym=1, ref_sig=3, plen=37, n=648, rate="12", rep=1)
        gi, gq = R.grid_levels(cfg.types()[:3], rng.integers(0, 2, 3 * 274 * R.BITS[mod]), mod, 0, 3)
        xs.append(np.stack(R.ifft_input(gi, gq, R.AMP[mod]), -1))
    ci, cq = R.grid_levels(R.CF_TYPES, R.cf_bits(R.cf_message(R.TxConfig())), "BPSK", 0, 3)
    xs.append(np.stack(R.ifft_input(ci, cq, R.AMP["BPSK"]), -1))
    full = rng.integers(-(1 << 17), 1 << 17, (2, R.N_FFT, 2))          # saturates
    tone = np.zeros((2, R.N_FFT, 2), np.int64)
    tone[0, 5] = (20000, -7000)
    tone[1, 0] = ((1 << 17) - 1, -(1 << 17))
    xs += [full, tone]
    x = np.concatenate(xs)
    yr, yi = R.ifft_fixed(x[..., 0], x[..., 1])
    rev = R.bitrev(11)
    out = HERE / "work" / "ifft"
    out.mkdir(parents=True, exist_ok=True)
    np.savetxt(out / "in.txt", x.reshape(-1, 2), fmt="%d")
    np.savetxt(out / "exp.txt", np.stack([yr[:, rev], yi[:, rev]], -1).reshape(-1, 2), fmt="%d")
    (out / "cfg.txt").write_text(f"{x.shape[0]} {seed} {gap}\n")
    sat = int(np.count_nonzero(np.abs(yr) >= (1 << 17) - 1))
    print(f"{out}: {x.shape[0]} symbols, {sat} saturated output values")

if __name__ == "__main__":
    main(*(int(a) for a in sys.argv[1:]))
