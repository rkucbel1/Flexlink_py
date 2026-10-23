#!/usr/bin/env python3
"""
Decimate_Payload_Rx.py  (lives in Flexlink_py/Rx)

Brings the received payload waveform to the 20 MHz rate of the OFDM demodulator.
First Rx step; Packet_Detector.py follows.

Steps:
  1. Read Rx/Parameters_Rx_payload.json and
     Flexlink_BasicModules/Channel_Defects/Channel_Defects.json.
  2. Choose the input:
     - No Mode set to 1 in Channel_Defects.json (no defects modeled): load
       <File_Rx.Waveform_Payload_Rx>, dataset 'data', complex, 1-D. Single-waveform
       chain, as before.
     - Any Mode set to 1: load <File_Rx.Waveform_Payload_DefectsModeled_Rx>
       (written by Tx/Defect_Model.py), dataset 'data', (NumberOfRays, N), and keep
       the first Defined_Rx/NumberOfAntennas rows (1 to 4; it must not exceed the
       rays in the file). The later steps then run with --ray 1..N.
     Either file is looked for in Rx/ first, then in Tx/. The sample rate comes
     from the file's SampleRate attribute, or --sample-rate.
  3. Bring every waveform to 20 MHz: a 40 MHz waveform is decimated by 2; a
     20 MHz waveform is used as is. --decimator auto (default) keeps every other
     sample for a clean waveform and low-pass filters first (resample_poly) when
     defects are modeled, so the noise between 10 and 20 MHz does not fold into
     the band. --decimator simple / filter forces one or the other.
  4. Decimation gain correction: OFDM_Modulator_841 scales its IFFT output by
     sqrt(N_IFFT), so a 40 MHz waveform (2048-point IFFT) kept at every other
     sample is sqrt(1024/2048) = 1/sqrt(2) lower in amplitude than the same grid
     modulated at 20 MHz. The waveform is divided by that gain (x sqrt(2) after
     decimating by 2, x 1 at 20 MHz), so it looks like a 20 MHz Tx waveform.
     (--no-normalize keeps it.)
  5. Save it to Rx/<File_Rx.Waveform_Payload_Dec_Rx>.

Nothing is written to the JSON.

Usage:
    python Decimate_Payload_Rx.py
    python Decimate_Payload_Rx.py --decimator filter
    python Decimate_Payload_Rx.py --input other.h5 --output My_Dec.h5 --sample-rate 40e6
    python Decimate_Payload_Rx.py --defects-file My_Defects.json

The output .h5 file contains:
    data   (N,) complex128                    no defects: one waveform at 20 MHz
           (NumberOfAntennas, N) complex128   defects: one row per antenna (ray)
    attrs  SampleRate (20000000), SampleRateIn, Decimation, Decimator, Gain,
           GainRemoved, SourceFile, DefectsModeled (0/1), NumberOfAntennas,
           and with defects: DefectsApplied, SNR_dB and MeasuredSNR_dB of the rays
           kept (copied from the defects file)
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

# Make flexlink_paths importable when the script is run from anywhere.
for _parent in Path(__file__).resolve().parents:
    if (_parent / "flexlink_paths.py").is_file():
        sys.path.insert(0, str(_parent))
        break
from flexlink_paths import TX, CHANNEL_DEFECTS, MAX_RAYS  # noqa: E402

RX_DIR = Path(__file__).resolve().parent          # the Rx folder
PARAMS_FILE = RX_DIR / "Parameters_Rx_payload.json"
DEFINED_SECTION = "Defined_Rx"
FILE_SECTION = "File_Rx"
DEFECTS_FILE = CHANNEL_DEFECTS / "Channel_Defects.json"

INPUT_KEY = "Waveform_Payload_Rx"       # File_Rx/<key> = input waveform (Rx/, else Tx/)
DEFECTS_INPUT_KEY = "Waveform_Payload_DefectsModeled_Rx"  # File_Rx/<key> = Defect_Model.py output (Rx/, else Tx/)
OUTPUT_KEY = "Waveform_Payload_Dec_Rx"  # File_Rx/<key> = output waveform at 20 MHz (Rx/)
DEFAULT_OUTPUT = "Waveform_Payload_Dec_Rx.h5"
DATASET = "data"

OUTPUT_RATE = 20e6
VALID_SAMPLE_RATES = (20e6, 40e6)


# --------------------------------------------------------------------------- #
# Parameters
# --------------------------------------------------------------------------- #
def load_params(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"Parameter file not found: {path}")
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def file_name(params: dict[str, Any], key: str, default: str | None = None) -> str:
    """File name stored under File_Rx/<key> in the JSON."""
    files = params.get(FILE_SECTION)
    name = files.get(key) if isinstance(files, dict) else None
    if name is None and default is not None:
        print(f"Warning: {FILE_SECTION}/{key} not set - using {default}")
        name = default
    if not isinstance(name, str) or not name.strip():
        raise KeyError(f"{FILE_SECTION}/{key} not found in {PARAMS_FILE.name}")
    return name.strip()


def defects_applied(path: Path) -> list[str]:
    """Mode names set to 1 in Channel_Defects.json (empty list = no defects)."""
    mode = load_params(path).get("Mode")
    if not isinstance(mode, dict):
        raise KeyError(f"Mode not found in {path.name}")
    for k, v in mode.items():
        if str(v).strip() not in ("0", "1"):
            raise ValueError(f"{path.name}: Mode/{k} = {v!r} must be 0 or 1")
    return [k for k, v in mode.items() if int(v) == 1]


def number_of_antennas(params: dict[str, Any]) -> int:
    n = params.get(DEFINED_SECTION, {}).get("NumberOfAntennas")
    if n is None:
        raise KeyError(f"{DEFINED_SECTION}/NumberOfAntennas not found in {PARAMS_FILE.name}")
    if str(n).strip() not in {str(k) for k in range(1, MAX_RAYS + 1)}:
        raise ValueError(f"{DEFINED_SECTION}/NumberOfAntennas = {n!r} must be 1 to {MAX_RAYS}")
    return int(n)


def find_input(name: str) -> Path:
    """Rx/<name> if it exists, else Tx/<name>."""
    for d in (RX_DIR, TX):
        if (d / name).is_file():
            return d / name
    raise FileNotFoundError(f"{name} not found in {RX_DIR} or {TX}")


# --------------------------------------------------------------------------- #
# Files
# --------------------------------------------------------------------------- #
def to_complex(arr: np.ndarray) -> np.ndarray:
    """Accept native complex or a (real, imag) compound dtype."""
    if arr.dtype.names:
        names = {n.lower(): n for n in arr.dtype.names}
        re = names.get("real") or names.get("r")
        im = names.get("imag") or names.get("i")
        if re and im:
            return arr[re].astype(float) + 1j * arr[im].astype(float)
    return arr.astype(np.complex128)


def load_waveform(path: Path) -> tuple[np.ndarray, dict[str, Any]]:
    import h5py
    with h5py.File(path, "r") as f:
        if DATASET not in f:
            raise KeyError(f"{path.name} has no '{DATASET}' dataset (has {list(f)})")
        x = to_complex(np.asarray(f[DATASET][()]))
        attrs = dict(f.attrs)
    if x.ndim not in (1, 2):
        raise ValueError(f"{path.name}: '{DATASET}' must be 1-D or (rays, samples), got {x.shape}")
    if x.size == 0:
        raise ValueError(f"{path.name} contains an empty waveform")
    return x, attrs


def save_waveform(path: Path, x: np.ndarray, attrs: dict[str, Any]) -> None:
    import h5py
    with h5py.File(path, "w") as f:
        f.create_dataset(DATASET, data=x)
        for k, v in attrs.items():
            f.attrs[k] = v


# --------------------------------------------------------------------------- #
# Processing
# --------------------------------------------------------------------------- #
def decimate_to_20mhz(x: np.ndarray, fs: float, method: str) -> tuple[np.ndarray, int]:
    """Decimate along the last axis (one waveform, or one row per antenna)."""
    if fs == OUTPUT_RATE:
        return x, 1
    if method == "simple":
        return x[..., ::2], 2   # exact for a clean waveform: 841 tones span only +-8.2 MHz
    if method == "filter":
        from scipy.signal import resample_poly
        return resample_poly(x, 1, 2, axis=-1), 2
    raise ValueError(f"Unknown decimator '{method}' (use simple or filter)")


def decimation_gain(decimation: int) -> float:
    """Amplitude of the decimated waveform relative to the same grid modulated at 20 MHz.

    OFDM_Modulator_841 scales the IFFT output by sqrt(N_IFFT): 2048 at 40 MHz,
    1024 at 20 MHz. Keeping every `decimation`-th sample of the 40 MHz waveform
    therefore leaves it sqrt(1024 / 2048) = 1/sqrt(2) lower; at 20 MHz the gain is 1.
    """
    return float(1.0 / np.sqrt(decimation))


def decimate_payload(x: np.ndarray, fs: float, decimator: str = "simple", normalize: bool = True
                     ) -> tuple[np.ndarray, dict[str, Any]]:
    """Return the 20 MHz waveform and processing info."""
    x20, decimation = decimate_to_20mhz(x, fs, decimator)
    gain = decimation_gain(decimation)
    if normalize:
        x20 = x20 / gain
    return x20, {"Decimation": decimation, "Gain": gain, "GainRemoved": int(normalize)}


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Decimate the received payload waveform to 20 MHz.")
    ap.add_argument("--params", type=Path, default=PARAMS_FILE)
    ap.add_argument("--input", type=Path,
                    help=f"waveform .h5 (default: File_Rx/{INPUT_KEY}, looked for in Rx/ then Tx/)")
    ap.add_argument("--output", type=Path, help=f"output .h5 (default: Rx/<File_Rx.{OUTPUT_KEY}>)")
    ap.add_argument("--sample-rate", type=float,
                    help="input sample rate in Hz (default: the file's SampleRate attribute)")
    ap.add_argument("--decimator", choices=("auto", "simple", "filter"), default="auto",
                    help="40 -> 20 MHz: auto (default: simple without defects, filter with defects), "
                         "simple = keep every other sample, filter = low-pass first")
    ap.add_argument("--defects-file", type=Path, default=DEFECTS_FILE,
                    help="Channel_Defects.json whose Mode flags decide the input (default: "
                         "Flexlink_BasicModules/Channel_Defects/Channel_Defects.json)")
    ap.add_argument("--no-normalize", action="store_true", help="keep the decimation gain")
    a = ap.parse_args(argv)

    try:
        if importlib.util.find_spec("h5py") is None:
            raise ValueError("Reading/writing .h5 files needs h5py - install it with: pip install h5py")

        params = load_params(a.params)
        active = defects_applied(a.defects_file)
        defects = bool(active)
        in_key = DEFECTS_INPUT_KEY if defects else INPUT_KEY
        in_path = a.input or find_input(file_name(params, in_key))
        out_path = a.output or RX_DIR / file_name(params, OUTPUT_KEY, DEFAULT_OUTPUT)
        decimator = a.decimator if a.decimator != "auto" else ("filter" if defects else "simple")

        if defects and a.input is None and in_path.stat().st_mtime < a.defects_file.stat().st_mtime:
            print(f"Warning: {a.defects_file.name} was changed after {in_path.name} was made - "
                  "rerun Tx/Defect_Model.py (Run_Rx_Chain.py does it automatically)")
        x, in_attrs = load_waveform(in_path)
        extra: dict[str, Any] = {"DefectsModeled": int(defects)}
        if defects:
            n_ant = number_of_antennas(params)
            if x.ndim == 1:
                raise ValueError(f"{in_path.name} holds one waveform, but defects are modeled "
                                 f"({', '.join(active)}) - run Tx/Defect_Model.py first")
            if n_ant > x.shape[0]:
                raise ValueError(f"{DEFINED_SECTION}/NumberOfAntennas is {n_ant} but {in_path.name} "
                                 f"has only {x.shape[0]} rays - raise Settings/NumberOfRays in "
                                 f"Channel_Defects.json and rerun Tx/Defect_Model.py")
            x = x[:n_ant, :]
            extra["NumberOfAntennas"] = n_ant
            extra["DefectsApplied"] = str(in_attrs.get("DefectsApplied", ",".join(active)))
            for k in ("SNR_dB", "MeasuredSNR_dB"):
                if k in in_attrs:
                    extra[k] = np.asarray(in_attrs[k], dtype=float).ravel()[:n_ant]
        else:
            if x.ndim == 2:
                if x.shape[0] != 1:
                    raise ValueError(f"{in_path.name} holds {x.shape[0]} waveforms, but no defects are "
                                     f"modeled - set File_Rx/{INPUT_KEY} to a single waveform")
                x = x[0]
            extra["NumberOfAntennas"] = 1
        fs = a.sample_rate or in_attrs.get("SampleRate")
        if fs is None:
            raise KeyError(f"{in_path.name} has no SampleRate attribute - pass --sample-rate")
        fs = float(fs)
        if fs not in VALID_SAMPLE_RATES:
            raise ValueError(f"Sample rate {fs:g} must be 20000000 or 40000000")

        x20, info = decimate_payload(x, fs, decimator, not a.no_normalize)

        out_path.parent.mkdir(parents=True, exist_ok=True)
        save_waveform(out_path, x20, {
            "SampleRate": OUTPUT_RATE, "SampleRateIn": fs, "Decimation": info["Decimation"],
            "Decimator": decimator, "Gain": info["Gain"], "GainRemoved": info["GainRemoved"],
            "SourceFile": in_path.name, **extra,
        })

        n_wave = 1 if x.ndim == 1 else x.shape[0]
        if defects:
            print(f"Defects modeled ({', '.join(active)}): using {n_wave} of the rays in {in_path.name} "
                  f"(NumberOfAntennas {n_wave})")
        else:
            print("No defects modeled: single-waveform chain")
        print(f"Input {in_path.name}: {x.shape[-1]} samples at {fs / 1e6:.0f} MHz"
              + (f" -> decimated by 2 ({decimator})" if info["Decimation"] == 2 else " (already 20 MHz)"))
        print(f"Decimation gain {info['Gain']:.4f} removed: {'yes' if info['GainRemoved'] else 'no'}")
        p = np.mean(np.abs(np.atleast_2d(x20)) ** 2, axis=-1)
        print(f"Output {x20.shape[-1]} samples at 20 MHz x {n_wave} waveform(s), mean power "
              + ", ".join(f"{v:.4f}" for v in p))
        print("Next: Packet_Detector.py " + (f"--ray 1 .. --ray {n_wave}" if n_wave > 1 else "(ray 1)"))
        print(f"Wrote {out_path}")
    except (FileNotFoundError, KeyError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
