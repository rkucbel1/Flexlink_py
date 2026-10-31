#!/usr/bin/env python3
"""
make_ldpc_vectors.py  (Flexlink_py/HW_Development/Packet_Detector/SIM)

Test vectors for tb_pl_ldpc.v (the LDPC decoder core pl_ldpc_core.v) from the bit-true model
pl_reference.decode_ldpc. Writes work/ldpc_units/:
  n.txt      number of cases
  cases.txt  per case: code index (4 n_sel + rate_sel), N, the N input LLRs (7 bit), the N expected
             hard bits, the expected iteration count

Cases for each of the 12 codes:
  * random code words, BPSK over AWGN, LLR 4 * 2y / sigma^2 rounded and saturated to +-63, at
    SNRs from "always fails" (10 iterations) to "first iteration" (1 iteration)
  * +63 everywhere (the all-zero code word, 1 iteration), -63 everywhere, random +-63
  * all 0 and small random values -2 .. 2 (ties in every minimum search: the tie-break)

    python make_ldpc_vectors.py [--seed 1] [--per-snr 2]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "PC"))
import pl_reference as P                                           # noqa: E402

CODES = [f"{n}_{r}" for n in (648, 1296, 1944) for r in P.RATES]
EBN0 = {"12": (-1.0, 0.5, 1.5, 2.5, 4.0), "23": (0.5, 2.0, 3.0, 4.0, 5.5),
        "34": (1.5, 3.0, 4.0, 5.0, 6.5), "56": (2.5, 4.0, 5.0, 6.0, 7.5)}


def encoder(code: P.Code):
    """Systematic encoder: p = A u from H = [H_u | H_p] (as PC/pl_check.Encoder)."""
    H = np.zeros((code.M, code.N), np.uint8)
    for r, idx in enumerate(code.layers):
        for row in range(code.Z):
            H[r * code.Z + row, idx[:, row]] = 1
    A = np.concatenate([H[:, code.K:], H[:, :code.K]], 1).copy()
    for col in range(code.M):
        piv = col + np.flatnonzero(A[col:, col])[0]
        if piv != col:
            A[[col, piv]] = A[[piv, col]]
        rows = np.flatnonzero(A[:, col])
        A[rows[rows != col]] ^= A[col]
    Am = A[:, code.M:].astype(np.int64)
    return lambda u: np.concatenate([u, (Am @ u) % 2])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--per-snr", type=int, default=2)
    a = ap.parse_args(argv)
    rng = np.random.default_rng(a.seed)
    out = HERE / "work" / "ldpc_units"
    out.mkdir(parents=True, exist_ok=True)
    lines, n, its = [], 0, []
    for ci, t in enumerate(CODES):
        code = P.code_of(t)
        enc = encoder(code)
        rate = code.K / code.N
        llrs = []
        for eb in EBN0[t.split("_")[1]]:
            sigma2 = 1.0 / (2 * rate * 10 ** (eb / 10))
            for _ in range(a.per_snr):
                c = enc(rng.integers(0, 2, code.K))
                y = (1 - 2 * c) + np.sqrt(sigma2) * rng.standard_normal(code.N)
                llrs.append(np.clip(np.floor(4 * 2 * y / sigma2 + 0.5), -63, 63).astype(np.int64))
        llrs += [np.full(code.N, 63), np.full(code.N, -63), 63 * (1 - 2 * rng.integers(0, 2, code.N)),
                 np.zeros(code.N, np.int64), rng.integers(-2, 3, code.N)]
        L = np.stack(llrs)
        hard, used = P.decode_ldpc(code, L)
        for k in range(L.shape[0]):
            lines.append(f"{ci} {code.N}\n" + " ".join(str(int(v)) for v in L[k]) + "\n"
                         + " ".join(str(int(v)) for v in hard[k]) + f"\n{int(used[k])}\n")
            its.append(int(used[k]))
            n += 1
    (out / "cases.txt").write_text("".join(lines))
    (out / "n.txt").write_text(f"{n}\n")
    h = np.bincount(its, minlength=11)
    print(f"Wrote {out}: {n} code words; iterations 1..10: {list(h[1:])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
