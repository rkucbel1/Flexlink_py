#!/usr/bin/env python3
"""
gen_ptx_tables.py  (Flexlink_py/HW_Development/Packet_Transmitter/PC)

Writes ptx_tables.npz, the fixed tables of the transmitter model (ptx_reference.py) and the
contents of the RTL ROMs (RTL/gen_ptx_roms.py reads this file). Like pl_tables.npz on the
receiver side, they are generated once, so the bit-true model never depends on floating-point
library details at run time.

  pre_a        Preamble A, one 64-sample period, int16 I and Q (the 625 kHz and 1.875 MHz cosines
               repeat every 64 samples at 40 Msps; Preamble A is 10000 samples = 156 periods + 16).
  pre_b        Preamble B body (2048 samples, the Zadoff-Chu symbol), int16 I and Q. Preamble B is
               body[1816:2048] + body (cyclic prefix 232); the AGC burst is body[0:200].
               Both at PRE_SCALE = 2^13 per unit of Flexlink_BasicModules/Preamble (rms 1), so
               |value| < 2^15.
  tw_re, tw_im IFFT twiddles W_2048^m = exp(+j 2 pi m / 2048), m = 0 .. 1535, round(2^16 cos / sin),
               18-bit signed (2^16 = 1.0 is representable).
  pilot_p0_<S>, pilot_p1_<S>
               Ref_P0 / Ref_P1 bits of one reference symbol for RefSignalSpacing S = 3, 6, 12, 24
               (840 / S values): alternating 1, 0, 1, ... scrambled with the Gold code
               C_init_ID1 = 129 (P0) or C_init_ID2 = 128 (P1)
               (ReferenceSignals/Generate_Reference_Signal_Vectors.py).
  ref_row_<S>, data_row
               signal types of a reference symbol and of a data symbol (10 Ref_P0, 20 Ref_P1,
               30 DC, 40 Data), Construct_Grid/Construct_Resource_Grid.py.
  il_<N>       LDPC interleaver, 0-based, out[i] = in[il[i]] (copied from pl_tables.npz)
  P_<code>, Z_<code>  802.11n prototype matrices (copied from pl_tables.npz)
  bcc_il       control-field BCC interleaver (140), out[i] = in[bcc_il[i]] (Interleave_BCC.py)

    python gen_ptx_tables.py          # needs ../../Packet_Detector/PC/pl_tables.npz

The preamble is computed here with the simulator's own formulas (Generate_AGC_Burst.py,
Generate_Preamble_A.py, Generate_Preamble_B.py); --check compares it with
Flexlink_BasicModules/Preamble/Preamble.h5 when h5py is installed.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = next((p for p in [HERE, *HERE.parents] if (p / "flexlink_paths.py").is_file()), HERE.parents[2])
PD_PC = ROOT / "HW_Development" / "Packet_Detector" / "PC"
OUT = HERE / "ptx_tables.npz"

FS = 40e6
N_FFT = 2048
CP = 232
PRE_SCALE = 1 << 13
TW_FRAC = 16
C_INIT_ID1, C_INIT_ID2 = 129, 128
SPACINGS = (3, 6, 12, 24)

# Construct_Resource_Grid.py: first Ref_P1 column and DC ranges per RefSignalSpacing
REF_SPEC = {
    3: (1, [(0, 0), (414, 414), (417, 417), (420, 420), (423, 423), (426, 426), (840, 840)]),
    6: (4, [(0, 3), (414, 417), (420, 423), (426, 429), (840, 840)]),
    12: (10, [(0, 9), (408, 417), (420, 429), (840, 840)]),
    24: (22, [(0, 21), (408, 429), (840, 840)]),
}
P0, P1, DC, DATA = 10, 20, 30, 40

# Interleave_BCC.py, 1-based as in the MATLAB code
BCC_IL_140 = [
    62, 88, 45, 106, 127, 37, 121, 85, 83, 54, 113, 99, 1, 12, 19, 23, 111, 79, 96, 59,
    97, 94, 55, 7, 31, 112, 89, 5, 34, 47, 137, 8, 44, 64, 72, 110, 117, 14, 136, 124,
    40, 66, 76, 77, 78, 81, 93, 61, 118, 115, 65, 11, 125, 73, 38, 25, 134, 50, 123, 20,
    15, 53, 109, 138, 75, 6, 41, 105, 52, 92, 128, 9, 10, 27, 82, 60, 84, 32, 74, 48,
    91, 43, 56, 135, 67, 28, 100, 57, 29, 17, 133, 98, 102, 33, 46, 116, 3, 103, 49, 2,
    68, 35, 130, 140, 107, 18, 126, 30, 39, 24, 132, 22, 42, 95, 70, 131, 101, 4, 104, 120,
    129, 13, 69, 63, 21, 36, 122, 114, 90, 108, 16, 58, 139, 26, 119, 87, 86, 71, 51, 80,
]


def gold(c_init: int, n: int, shift: int = 1600) -> np.ndarray:
    """LTE Gold code (Scrambler_Gold_Code_Generator.py)."""
    r1, r2 = 1, c_init & 0x7FFFFFFF
    out = np.zeros(n, np.int64)
    for i in range(n + shift):
        if i >= shift:
            out[i - shift] = (r1 ^ r2) & 1
        r1 = (r1 >> 1) | (((r1 ^ (r1 >> 3)) & 1) << 30)
        r2 = (r2 >> 1) | (((r2 ^ (r2 >> 1) ^ (r2 >> 2) ^ (r2 >> 3)) & 1) << 30)
    return out


def preamble_float():
    """(agc, pre_a, pre_b) exactly as Flexlink_BasicModules/Preamble (40 Msps)."""
    n_zc, u1 = 887, 54
    n = np.arange(n_zc)
    zc = np.exp(-1j * np.pi * u1 * n * (n + 1) / n_zc)
    d = np.fft.fft(zc)
    buf = np.zeros(N_FFT, complex)
    buf[:444] = d[:444]
    buf[-443:] = d[-443:]
    buf[0] = 0
    body = (N_FFT / n_zc) * np.fft.ifft(buf)
    agc = body[:200]
    k = np.arange(10000)
    pre_a = (np.cos(2 * np.pi * 625e3 * k / FS + np.pi / 4)
             + np.cos(2 * np.pi * 1875e3 * k / FS + 3 * np.pi / 4)).astype(complex)
    pre_b = np.concatenate([body[N_FFT - CP:], body])
    return agc, pre_a, pre_b, body


def ref_row(S: int) -> np.ndarray:
    first, dcs = REF_SPEC[S]
    row = np.full(841, DATA, np.int64)
    p1 = np.arange(first, 840, S)
    row[p1] = P1
    row[p1 + 1] = P0
    for lo, hi in dcs:
        row[lo:hi + 1] = DC
    return row


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Write ptx_tables.npz")
    ap.add_argument("--check", action="store_true", help="compare the preamble with Preamble.h5 (h5py)")
    a = ap.parse_args(argv)

    agc, pre_a, pre_b, body = preamble_float()
    pa = pre_a[:64]
    assert np.allclose(pre_a, np.tile(pa, 157)[:10000], atol=1e-12), "Preamble A is not 64-periodic"
    out = {"PRE_SCALE": PRE_SCALE, "TW_FRAC": TW_FRAC}
    q = lambda x: np.round(x * PRE_SCALE).astype(np.int64)             # noqa: E731
    out["pre_a_re"], out["pre_a_im"] = q(pa.real), q(pa.imag)
    out["pre_b_re"], out["pre_b_im"] = q(body.real), q(body.imag)
    for k in ("pre_a_re", "pre_a_im", "pre_b_re", "pre_b_im"):
        assert np.abs(out[k]).max() < 1 << 15, k
    m = np.arange(3 * N_FFT // 4)
    out["tw_re"] = np.round(np.cos(2 * np.pi * m / N_FFT) * (1 << TW_FRAC)).astype(np.int64)
    out["tw_im"] = np.round(np.sin(2 * np.pi * m / N_FFT) * (1 << TW_FRAC)).astype(np.int64)
    for S in SPACINGS:
        n = 840 // S
        alt = (np.arange(n) % 2 == 0).astype(np.int64)
        out[f"pilot_p0_{S}"] = alt ^ gold(C_INIT_ID1, n)
        out[f"pilot_p1_{S}"] = alt ^ gold(C_INIT_ID2, n)
        out[f"ref_row_{S}"] = ref_row(S)
        assert (out[f"ref_row_{S}"] == P0).sum() == n and (out[f"ref_row_{S}"] == P1).sum() == n
    dr = np.full(841, DATA, np.int64)
    dr[414:429] = DC
    out["data_row"] = dr
    out["bcc_il"] = np.array(BCC_IL_140, np.int64) - 1
    assert np.array_equal(np.sort(out["bcc_il"]), np.arange(140))

    plt = np.load(PD_PC / "pl_tables.npz")
    for key in plt.files:
        if key.startswith(("il_", "P_", "Z_")):
            out[key] = plt[key]
    for S in SPACINGS:                      # the receiver's tables must agree with ours
        assert np.array_equal(plt[f"pilot_bits_{S}"], out[f"pilot_p0_{S}"]), S
        assert np.array_equal(plt[f"ref_row_{S}"], out[f"ref_row_{S}"]), S
    assert np.array_equal(plt["data_row"], out["data_row"])

    if a.check:
        import h5py
        with h5py.File(ROOT / "Flexlink_BasicModules" / "Preamble" / "Preamble.h5", "r") as f:
            ref = np.asarray(f["Preamble"][()]).ravel()
        mine = np.concatenate([agc, pre_a, pre_b])
        err = np.abs(mine - ref).max()
        print(f"Preamble vs Preamble.h5: max |difference| {err:.2e} (complex64 file)")
        if err > 1e-5:
            return 1
    np.savez(OUT, **out)
    print(f"Wrote {OUT} ({len(out)} tables)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
