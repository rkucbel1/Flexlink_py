#!/usr/bin/env python3
"""
Combine_Antennas_Rx.py  (lives in Flexlink_py/Rx)

Maximum-ratio combining (MRC) of the equalized payload grids of several receive
antennas. Runs after Equalize_Payload_Grid.py has run for every antenna
(--ray 1 .. --ray N) and before Get_DataSigs_QAM.py. Step 6b of the Rx chain.

Method
------
Equalize_Payload_Grid.py writes, per antenna r and resource element (RE):
    x_r = Y_r / H_r            unbiased equalized symbol    ('data')
    v_r = sigma_r^2 / |H_r|^2  noise left on it             ('noise_var')
The MRC combination of the N antennas is the inverse-noise weighted average
    x = sum_r (x_r / v_r) / sum_r (1 / v_r)
    v = 1 / sum_r (1 / v_r)
which equals sum_r conj(H_r) Y_r / sigma_r^2 / sum_r |H_r|^2 / sigma_r^2, i.e. classic MRC
with per-antenna noise levels. A RE faded on one antenna is taken from the others, and
v is the noise of the combined symbol, so the soft demapper stays correctly scaled.
REs whose v_r is zero or not finite on an antenna are left out of that antenna's sum.

Inputs
------
Rx/Parameters_Rx_payload.json      File_Rx.Grid_Equalized_Rx, File_Rx.Waveform_Payload_Dec_Rx
Rx/<Grid_Equalized_Rx>_Ray1..N.h5  'data' and 'noise_var' (PacketLength, 841) per antenna.
                                   N = NumberOfAntennas recorded by Decimate_Payload_Rx.py.
                                   A ray file older than Waveform_Payload_Dec_Rx.h5 is left
                                   over from an earlier run and is skipped (e.g. when packet
                                   detection failed on that antenna this time).

Output
------
Rx/<Grid_Equalized_Rx>_Combined.h5
    data       (PacketLength, 841) complex128   combined symbols
    noise_var  (PacketLength, 841) float64      noise of each combined symbol
    attrs      Mod, Port, BitsPerSymbol, EqualizerMode, PacketLength (from the first
               ray), Ray (0 = combined), Rays (antennas used), RaysSkipped,
               NumberOfAntennas, NoiseVar_Rays and SNR_dB_Rays (the equalizer's pilot
               noise per antenna), MedianSNR_dB_Rays and MedianSNR_dB (median per-RE SNR
               after equalization, per antenna and combined), SourceFiles

Nothing is written to the JSON.

Usage
-----
    python Rx/Combine_Antennas_Rx.py
    python Rx/Combine_Antennas_Rx.py --rays 1 3            # only these antennas
    python Rx/Combine_Antennas_Rx.py --inputs a.h5 b.h5 --out My_Combined.h5
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from Rx_Antennas import antennas_in_run, combined_file_name  # noqa: E402
from flexlink_paths import MAX_RAYS, ray_file_name  # noqa: E402

RX_DIR = Path(__file__).resolve().parent
PARAMS_FILE = RX_DIR / "Parameters_Rx_payload.json"
FILE_SECTION = "File_Rx"
GRID_KEY = "Grid_Equalized_Rx"
DEC_KEY = "Waveform_Payload_Dec_Rx"
DATASET, NOISE_DATASET = "data", "noise_var"
COPY_ATTRS = ("Mod", "Port", "BitsPerSymbol", "EqualizerMode", "PacketLength", "PilotPower",
              "MaxDoppler", "ResourceGridFile")


def load_params(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"Parameter file not found: {path}")
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def file_name(params: dict[str, Any], key: str) -> str:
    name = params.get(FILE_SECTION, {}).get(key)
    if not isinstance(name, str) or not name.strip():
        raise KeyError(f"{FILE_SECTION}/{key} not found in {PARAMS_FILE.name}")
    return name.strip()


def load_equalized(path: Path) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    import h5py
    with h5py.File(path, "r") as f:
        for d in (DATASET, NOISE_DATASET):
            if d not in f:
                raise KeyError(f"{path.name} has no '{d}' dataset - run Equalize_Payload_Grid.py")
        x = np.asarray(f[DATASET][()]).astype(np.complex128)
        v = np.asarray(f[NOISE_DATASET][()], dtype=np.float64)
        attrs = dict(f.attrs)
    if x.shape != v.shape or x.ndim != 2:
        raise ValueError(f"{path.name}: 'data' {x.shape} and 'noise_var' {v.shape} must be equal 2-D grids")
    return x, v, attrs


def save_combined(path: Path, x: np.ndarray, v: np.ndarray, attrs: dict[str, Any]) -> None:
    import h5py
    with h5py.File(path, "w") as f:
        f.create_dataset(DATASET, data=x)
        f.create_dataset(NOISE_DATASET, data=v)
        for k, val in attrs.items():
            f.attrs[k] = val


def mrc_combine(xs: list[np.ndarray], vs: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    """Inverse-noise weighted combination of equalized grids (see the module docstring)."""
    num = np.zeros(xs[0].shape, complex)
    den = np.zeros(xs[0].shape)
    for x, v in zip(xs, vs):
        ok = np.isfinite(v) & (v > 0) & np.isfinite(x)
        w = np.zeros(v.shape)
        w[ok] = 1.0 / v[ok]
        num += np.where(ok, w * x, 0)
        den += w
    good = den > 0
    x_c = np.where(good, num / np.where(good, den, 1), np.mean(xs, axis=0))
    v_c = np.where(good, 1.0 / np.where(good, den, 1), np.inf)
    return x_c, v_c


def median_snr_db(v: np.ndarray) -> float:
    ok = np.isfinite(v) & (v > 0)
    return float(-10 * np.log10(np.median(v[ok]))) if ok.any() else float("nan")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Maximum-ratio combining of the per-antenna equalized grids.")
    ap.add_argument("--params", type=Path, default=PARAMS_FILE)
    ap.add_argument("--rays", type=int, nargs="+", choices=range(1, MAX_RAYS + 1), metavar="N",
                    help="antennas to combine (default: 1 .. NumberOfAntennas of this run)")
    ap.add_argument("--inputs", type=Path, nargs="+",
                    help="equalized grid files to combine instead of the *_RayN defaults (no age check)")
    ap.add_argument("--out", type=Path, help=f"output .h5 (default: Rx/<File_Rx.{GRID_KEY}>_Combined.h5)")
    a = ap.parse_args(argv)

    try:
        if importlib.util.find_spec("h5py") is None:
            raise ValueError("Reading/writing .h5 files needs h5py - install it with: pip install h5py")
        params = load_params(a.params)
        grid_name = file_name(params, GRID_KEY)
        out_path = a.out or RX_DIR / combined_file_name(grid_name)

        skipped: list[str] = []
        if a.inputs:
            candidates = [(None, p) for p in a.inputs]
        else:
            n = antennas_in_run(params)
            rays = a.rays or list(range(1, n + 1))
            dec_path = RX_DIR / file_name(params, DEC_KEY)
            dec_time = dec_path.stat().st_mtime if dec_path.is_file() else 0.0
            candidates = []
            for r in rays:
                p = RX_DIR / ray_file_name(grid_name, r)
                if not p.is_file():
                    skipped.append(f"ray {r} (no {p.name})")
                elif p.stat().st_mtime < dec_time:
                    skipped.append(f"ray {r} ({p.name} is older than {dec_path.name})")
                else:
                    candidates.append((r, p))

        xs, vs, used, files, nv_rays, snr_rays, med_rays = [], [], [], [], [], [], []
        first_attrs: dict[str, Any] = {}
        for r, p in candidates:
            x, v, at = load_equalized(p)
            if xs and x.shape != xs[0].shape:
                raise ValueError(f"{p.name} is {x.shape} but {files[0]} is {xs[0].shape}")
            if not first_attrs:
                first_attrs = at
            xs.append(x); vs.append(v); files.append(p.name)
            used.append(int(r if r is not None else at.get("Ray", len(used) + 1)))
            nv_rays.append(float(at.get("NoiseVar", np.nan)))
            snr_rays.append(float(at.get("SNR_dB", np.nan)))
            med_rays.append(median_snr_db(v))
        if not xs:
            raise FileNotFoundError("No equalized grid to combine: " + "; ".join(skipped))

        x_c, v_c = mrc_combine(xs, vs)
        med_c = median_snr_db(v_c)
        attrs = {k: first_attrs[k] for k in COPY_ATTRS if k in first_attrs}
        attrs.update({
            "Ray": 0, "Rays": np.asarray(used, dtype=np.int64), "NumberOfAntennas": len(used),
            "RaysSkipped": "; ".join(skipped) if skipped else "none",
            "NoiseVar_Rays": np.asarray(nv_rays), "SNR_dB_Rays": np.asarray(snr_rays),
            "MedianSNR_dB_Rays": np.asarray(med_rays), "MedianSNR_dB": med_c,
            "SourceFiles": ",".join(files), "Combining": "MRC",
        })
        out_path.parent.mkdir(parents=True, exist_ok=True)
        save_combined(out_path, x_c, v_c, attrs)

        for r, f, s, m in zip(used, files, snr_rays, med_rays):
            print(f"Ray {r}: {f}, equalizer SNR {s:.1f} dB, median per-signal SNR {m:.1f} dB")
        for s in skipped:
            print(f"Skipped {s}")
        print(f"MRC of {len(used)} antenna(s): median per-signal SNR {med_c:.1f} dB "
              f"(best single antenna {np.nanmax(med_rays):.1f} dB)")
        print(f"Wrote {out_path}")
    except (FileNotFoundError, KeyError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
