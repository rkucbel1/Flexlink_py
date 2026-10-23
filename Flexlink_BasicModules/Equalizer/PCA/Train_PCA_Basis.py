#!/usr/bin/env python3
"""
Train_PCA_Basis.py  (lives in Flexlink_py/Flexlink_BasicModules/Equalizer/PCA)

Precomputes the principal-component (PCA / eigen-decomposition) channel bases used by
EqualizerMode "PCA" in Rx/Equalize_Payload_Grid.py. Run it once, and again after
changing PCA_Training_Profiles.json or the reference-symbol layouts.

Method
------
For each class in PCA_Training_Profiles.json (e.g. Short / Medium / Long delay spread):
  1. Channel correlation over the 841 tones, R = E[h h^H], averaged over the class's
     channel profiles. For a tapped-delay-line channel with independent Rayleigh taps
     this expectation is exact:
         R(t1, t2) = sum_l p_l exp(-j 2 pi (t1 - t2) tau_l / N)
     (= the average over infinitely many simulated channels). Added to it:
       * a uniform power floor over 0..MaxDelay (UniformFloor_dB) so the basis is not
         tied to the exact tap delays of the models, and
       * a uniform timing error of +/- TimingJitter_samples (packet-detector timing).
  2. For each pilot layout (RefSignalSpacing 3, 6, 12, 24; Ref_P0 and Ref_P1):
     R_pp = R at the pilot tones, eigen-decomposition R_pp = U diag(lambda) U^H, keep the
     r strongest components holding EnergyKept of the energy.
  3. Store U_r (pilots x r), lambda_r, and A0 = R[all tones, pilots] U_r (841 x r).
  4. The same for a fallback twin of every class (<Class>U): power spread uniformly over
     -4..MaxDelay (no channel models). The estimator switches to it when a packet's channel
     does not fit the trained components (energy outside them, see PCA_Estimator.py). This
     is what the LMMSE mode uses, so a channel unlike the models is never worse than LMMSE.

At run time (PCA_Estimator.py) the frequency-direction estimate for one reference symbol is
    H_all = A0 diag(1 / (lambda + sigma^2)) U_r^H h_pilots
i.e. the LMMSE estimate restricted to the r principal components: two thin matrix
products, and the noise level only rescales r numbers (no matrix inversion per packet).

Output
------
PCA_Basis_S<spacing>.h5 in this folder, one per RefSignalSpacing. Datasets (flat names):
    <Class>_P<port>_U       (Np, r)   complex128
    <Class>_P<port>_lambda  (r,)      float64
    <Class>_P<port>_A0      (841, r)  complex128
    <Class>_P<port>_pilot_k (Np,)     int64      pilot k indices the basis was built for
    <Class>U_P<port>_...                         fallback twin (uniform power over -4..MaxDelay)
attrs: Classes (comma-separated names), MaxDelays (samples), Ranks (per class, P0),
       RefSignalSpacing, N_FFT, EnergyKept, UniformFloor_dB, TimingJitter_samples,
       ProfilesFile

Usage:
    python Train_PCA_Basis.py
    python Train_PCA_Basis.py --spacings 3 6
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
for _parent in HERE.parents:
    if (_parent / "flexlink_paths.py").is_file():
        sys.path.insert(0, str(_parent))
        break
from flexlink_paths import CONSTRUCT_GRID  # noqa: E402

PROFILES_FILE = HERE / "PCA_Training_Profiles.json"
N_FFT = 1024
FS = 20e6
TONES = np.arange(-420, 421)             # k = tone + 420
SPACINGS = (3, 6, 12, 24)
REF_CODES = {0: 10, 1: 20}               # Ref_P0, Ref_P1 (Construct_Resource_Grid.SignalType)


def basis_file(spacing: int) -> Path:
    return HERE / f"PCA_Basis_S{spacing}.h5"


def load_ref_layouts():
    path = Path(CONSTRUCT_GRID) / "Construct_Resource_Grid.py"
    spec = importlib.util.spec_from_file_location("Construct_Resource_Grid", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["Construct_Resource_Grid"] = mod
    spec.loader.exec_module(mod)
    return mod.REF_SYMBOL_LAYOUTS


def profile_corr(dt: np.ndarray, delays_ns, power_db) -> np.ndarray:
    """sum_l p_l exp(-j 2 pi dt tau_l / N) with sum p_l = 1, tau in 20 MHz samples."""
    p = 10 ** (np.asarray(power_db, float) / 10)
    p /= p.sum()
    tau = np.asarray(delays_ns, float) * 1e-9 * FS
    return np.exp(-2j * np.pi * np.multiply.outer(dt, tau) / N_FFT) @ p


def uniform_corr(dt: np.ndarray, d0: float, d1: float) -> np.ndarray:
    """Correlation of power spread uniformly over delays d0..d1 samples."""
    return np.exp(-1j * np.pi * dt * (d0 + d1) / N_FFT) * np.sinc(dt * (d1 - d0) / N_FFT)


def class_corr(cfg: dict, cls: dict) -> np.ndarray:
    """841 x 841 correlation for one class."""
    dt = np.subtract.outer(TONES, TONES).astype(float)
    r = np.zeros(dt.shape, complex)
    for name in cls["Profiles"]:
        prof = cfg["Profiles"][name]
        r += profile_corr(dt, prof["Delays_ns"], prof["Power_dB"])
    r /= len(cls["Profiles"])
    d = float(cls["MaxDelay_samples"])
    floor = 10 ** (float(cfg.get("UniformFloor_dB", -300)) / 10)
    r += floor * uniform_corr(dt, 0.0, d)
    j = float(cfg.get("TimingJitter_samples", 0))
    if j > 0:                                   # uniform timing error in [-j, j]
        r *= np.sinc(2 * j * dt / N_FFT)
    return r / (1 + floor)


def train(cfg: dict, spacings) -> list[tuple]:
    import h5py
    layouts = load_ref_layouts()
    keep = float(cfg.get("EnergyKept", 0.99999))
    classes = cfg["Classes"]
    corrs = {c["Name"]: class_corr(cfg, c) for c in classes}
    dt = np.subtract.outer(TONES, TONES).astype(float)
    for c in classes:
        corrs[c["Name"] + "U"] = uniform_corr(dt, -4.0, float(c["MaxDelay_samples"]))
    summary = []
    for s in spacings:
        lay = np.asarray(layouts[s])
        out = basis_file(s)
        ranks = []
        with h5py.File(out, "w") as f:
            for name in corrs:
                R = corrs[name]
                for port, code in REF_CODES.items():
                    pk = np.flatnonzero(lay == code)
                    lam, U = np.linalg.eigh(R[np.ix_(pk, pk)])
                    lam, U = lam[::-1].real, U[:, ::-1]
                    lam = np.maximum(lam, 0)
                    r = int(np.searchsorted(np.cumsum(lam) / lam.sum(), keep) + 1)
                    r = min(r, pk.size)
                    Ur, lr = U[:, :r], lam[:r]
                    A0 = R[:, pk] @ Ur
                    pre = f"{name}_P{port}"
                    f.create_dataset(f"{pre}_U", data=Ur)
                    f.create_dataset(f"{pre}_lambda", data=lr)
                    f.create_dataset(f"{pre}_A0", data=A0)
                    f.create_dataset(f"{pre}_pilot_k", data=pk.astype(np.int64))
                    if port == 0:
                        ranks.append(r)
                        summary.append((s, name, pk.size, r))
            f.attrs["Classes"] = ",".join(c["Name"] for c in classes)
            f.attrs["MaxDelays"] = np.array([c["MaxDelay_samples"] for c in classes], dtype=np.int64)
            f.attrs["Ranks"] = np.array(ranks, dtype=np.int64)      # classes, then their U twins
            f.attrs["RefSignalSpacing"] = s
            f.attrs["N_FFT"] = N_FFT
            f.attrs["EnergyKept"] = keep
            f.attrs["UniformFloor_dB"] = float(cfg.get("UniformFloor_dB", -300))
            f.attrs["TimingJitter_samples"] = float(cfg.get("TimingJitter_samples", 0))
            f.attrs["ProfilesFile"] = PROFILES_FILE.name
        print(f"Wrote {out}")
    return summary


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Precompute the PCA channel bases for EqualizerMode PCA.")
    ap.add_argument("--profiles", type=Path, default=PROFILES_FILE)
    ap.add_argument("--spacings", type=int, nargs="+", default=list(SPACINGS), choices=SPACINGS)
    a = ap.parse_args(argv)
    try:
        if importlib.util.find_spec("h5py") is None:
            raise ValueError("Writing .h5 files needs h5py - install it with: pip install h5py")
        with open(a.profiles, "r", encoding="utf-8") as fh:
            cfg = json.load(fh)
        summary = train(cfg, a.spacings)
        print(f"{'Spacing':>7} {'Class':>7} {'Pilots':>6} {'Rank':>5}   cost per reference symbol "
              "(complex MACs, PCA vs full LMMSE 841 x pilots)")
        for s, name, np_, r in summary:
            print(f"{s:7d} {name:>7} {np_:6d} {r:5d}   {r * (np_ + 841):7d} vs {841 * np_:7d}")
    except (FileNotFoundError, KeyError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
