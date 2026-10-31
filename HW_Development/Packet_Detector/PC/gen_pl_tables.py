#!/usr/bin/env python3
"""
gen_pl_tables.py  (Flexlink_py/HW_Development/Packet_Detector/PC)

Writes pl_tables.npz, the fixed tables of the payload receiver model (pl_reference.py). Like
cf_tables.py for the control field, they are generated once, so the bit-true model never depends
on floating-point library details at run time.

  pilot_bits_<S>     Ref_P0 bits of one reference symbol for RefSignalSpacing S = 3, 6, 12, 24
                     (840 / S values): the alternating vector 1, 0, 1, ... scrambled with the
                     Gold code C_init_ID1 = 129 (ReferenceSignals/Generate_Reference_Signal_
                     Vectors.py). Checked against Ref_P0_3_1.h5 when h5py and the file exist.
  fir_start_<S>_<c>  per tone (0..840): first of the 8 pilots its estimate uses (same for every set)
  fir_re_<S>_<c>_<v>, per tone: 8 coefficients, signed, / 2^FIR_FRAC. LMMSE weights for a flat
  fir_im_<S>_<c>_<v>  delay profile over the class-c window and pilot noise FIR_NOISE[v] (relative
                     to the pilot power; v = 0 low-SNR set, 1 high-SNR set), restricted to the 8
                     nearest pilots. Windows are in the
                     frame of the hardware grid (FFT_StartTime ramp kept): delays -8 .. E_c with
                     E_c = 24, 40, 64, 112 (the simulator's -20 .. 12 / 28 / 52 / 100). A class
                     exists only while the window is at most 0.9 * 1024 / S samples long (no
                     aliasing): S = 3, 6 all four, S = 12 classes 0-2, S = 24 class 0.
  ref_row_<S>        signal types (10 Ref_P0, 20 Ref_P1, 30 DC, 40 Data) of a reference symbol,
  data_row           and of every other symbol (Construct_Grid/Construct_Resource_Grid.py):
                     symbol s is a reference symbol when s % RefSymbolSpacing == 0. Checked
                     against Resource_Grid for every PacketLength and spacing.
  win_<S>            Hann window over the Np pilots of a row for the delay profile:
                     round(32767 sin^2(pi (i + 0.5) / Np)), unsigned 15 bit
  il_<N>             LDPC interleaver, 0-based, out[i] = in[il[i]] (Interleaver/Interleaver_<N>.h5)
  P_<type>, Z_<type> 802.11n prototype matrices (LDPC_Encoder/MatrixGeneration_H/get_p_ldpc.py)

    python gen_pl_tables.py            # needs Flexlink_BasicModules (FLEXLINK_MODULES) and h5py
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = next((p for p in [HERE, *HERE.parents] if (p / "flexlink_paths.py").is_file()), HERE.parents[2])
BM = Path(os.environ.get("FLEXLINK_MODULES", str(ROOT / "Flexlink_BasicModules")))
sys.path.insert(0, str(HERE))
import cf_reference as C                                            # noqa: E402  (Gold code)

FIR_TAPS = 8
FIR_FRAC = 14
FIR_NOISE = (0.5, 0.05)               # design pilot noise of the low- and high-SNR weight sets
C_INIT_ID1 = 129
SPACINGS = (3, 6, 12, 24)
CLASS_END = (24, 40, 64, 112)          # hardware frame; the simulator's 12, 28, 52, 100
WIN_START = -8
CODES = [f"{n}_{r}" for n in (648, 1296, 1944) for r in ("12", "23", "34", "56")]


def pilot_cols(S: int) -> np.ndarray:
    """Grid columns (0..840) of the Ref_P0 pilots: S - 1, 2S - 1, ... (Construct_Resource_Grid)."""
    return np.arange(S - 1, 841, S)


def n_classes(S: int) -> int:
    return sum(1 for e in CLASS_END if e - WIN_START <= 0.9 * 1024 / S)


def pilot_bits(S: int) -> np.ndarray:
    n = 840 // S
    alt = (np.arange(n) % 2 == 0).astype(np.int64)
    return alt ^ C.gold(C_INIT_ID1, n)


def fir_table(S: int, end: int, noise: float):
    pt = (pilot_cols(S) - 420).astype(float)
    d = np.arange(WIN_START, end)

    def corr(dk):
        return np.exp(-2j * np.pi * np.multiply.outer(dk, d) / 1024).mean(axis=-1)

    start = np.zeros(841, np.int64)
    wre = np.zeros((841, FIR_TAPS), np.int64)
    wim = np.zeros((841, FIR_TAPS), np.int64)
    for k, t in enumerate(np.arange(-420, 421).astype(float)):
        near = np.sort(np.argsort(np.abs(pt - t), kind="stable")[:FIR_TAPS])
        assert np.all(np.diff(near) == 1)
        p = pt[near]
        w = corr(np.subtract.outer(np.array([t]), p)) @ np.linalg.inv(
            corr(np.subtract.outer(p, p)) + noise * np.eye(FIR_TAPS))
        q = np.round(w[0] * (1 << FIR_FRAC))
        start[k] = near[0]
        wre[k], wim[k] = q.real.astype(np.int64), q.imag.astype(np.int64)
    return start, wre, wim


def read_h5(path: Path, name: str):
    import h5py
    with h5py.File(str(path), "r") as f:
        return np.asarray(f[name][()])


def main() -> int:
    out = {"FIR_TAPS": FIR_TAPS, "FIR_FRAC": FIR_FRAC, "FIR_NOISE": np.array(FIR_NOISE),
           "CLASS_END": np.array(CLASS_END), "WIN_START": WIN_START}
    for S in SPACINGS:
        out[f"pilot_bits_{S}"] = pilot_bits(S)
        out[f"n_classes_{S}"] = n_classes(S)
        Np = 840 // S
        out[f"win_{S}"] = np.round(32767 * np.sin(np.pi * (np.arange(Np) + 0.5) / Np) ** 2).astype(np.int64)
        for c in range(n_classes(S)):
            for v, nz in enumerate(FIR_NOISE):
                st, wr, wi = fir_table(S, CLASS_END[c], nz)
                out[f"fir_start_{S}_{c}"], out[f"fir_re_{S}_{c}_{v}"], out[f"fir_im_{S}_{c}_{v}"] = st, wr, wi
                sets = {(tuple(a), tuple(b)) for a, b in zip(wr, wi)}
                print(f"S {S:2d} class {c} (-8 .. {CLASS_END[c]}) noise {nz}: {len(sets)} distinct "
                      f"coefficient sets, max |w| {max(np.abs(wr).max(), np.abs(wi).max())}")
    ref = ROOT / "ReferenceSignals" / "Ref_P0_3_1.h5"
    try:
        x = read_h5(ref, "data").ravel()
        re = x["r"] if x.dtype.names else x.real
        ok = np.array_equal((re < 0).astype(np.int64), out["pilot_bits_3"])   # BPSK: 1 -> -1
        print(f"Pilot bits {'match' if ok else 'DO NOT MATCH'} {ref}")
        if not ok:
            return 1
    except (ImportError, OSError, KeyError, ValueError) as e:
        print(f"Note: pilot bits not checked against {ref} ({e})")
    import importlib.util
    spec = importlib.util.spec_from_file_location("crg", BM / "Construct_Grid" / "Construct_Resource_Grid.py")
    rg = importlib.util.module_from_spec(spec); spec.loader.exec_module(rg)
    data_row = None
    for S in SPACINGS:
        for rsym in (3, 1, 6, 12):
            for plen in (37, 4, 49, 73, 97, 121):
                if (plen - 1) % rsym:
                    continue                                # not a valid grid (must end with a Symbol_Ref)
                t = np.asarray(rg.Resource_Grid(plen, rsym, S).signal_types, np.int64)
                if rsym == 3 and plen == 37:
                    out[f"ref_row_{S}"] = t[0]
                    if data_row is None:
                        data_row = t[1]
                    assert np.array_equal(t[1], data_row)
                exp = np.array([out[f"ref_row_{S}"] if s % rsym == 0 else data_row for s in range(plen)])
                assert np.array_equal(t, exp), (S, rsym, plen)
        assert np.array_equal(np.flatnonzero(out[f"ref_row_{S}"] == 10), pilot_cols(S))
    out["data_row"] = data_row
    print("Grid layout rule checked for every spacing and PacketLength")
    for n in (648, 1296, 1944):
        il = read_h5(BM / "Interleaver" / f"Interleaver_{n}.h5", "InterleaverIndex").ravel().astype(np.int64)
        if il.min() == 1:
            il = il - 1
        assert np.array_equal(np.sort(il), np.arange(n))
        out[f"il_{n}"] = il
    sys.path.insert(0, str(BM / "LDPC_Encoder" / "MatrixGeneration_H"))
    from get_p_ldpc import get_p_ldpc
    for t in CODES:
        P, Z = get_p_ldpc(t)
        out[f"P_{t}"], out[f"Z_{t}"] = np.asarray(P, np.int64), int(Z)
    np.savez_compressed(HERE / "pl_tables.npz", **out)
    print(f"Wrote {HERE / 'pl_tables.npz'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
