#!/usr/bin/env python3
"""
Demodulate_Waveform_Rx.py  (lives in Flexlink_py/Rx)

Turns the 20 MHz payload waveform back into the payload resource grid.
Reverse of Tx/Construct_Payload_Waveform.py, after Decimate_Payload_Rx.py has
brought the waveform to 20 MHz and Packet_Detector.py has removed the frequency
offset and cut the waveform at the first payload sample.

Steps:
  1. Read Rx/Parameters_Rx_payload.json.
  2. Load the 20 MHz waveform from Rx/<File_Rx.Waveform_Payload_Packet_Rx>
     (written by Packet_Detector.py): dataset 'data', complex, 1-D, starting at the
     first payload sample. Its SampleRate attribute must be 20 MHz.
  3. Run ofdm_demodulator_841() from
     Flexlink_BasicModules/OFDM_Modulator/OFDM_Demodulator_841.py with
     start_time = Defined_Rx/FFT_StartTime (e.g. -12: the FFT window starts 12
     samples early, inside the cyclic prefix). Keep the first PacketLength
     OFDM symbols.
  4. Remove the two known, deterministic effects (each can be switched off):
       * timing phase ramp: starting the FFT FFT_StartTime samples away from
         the end of the CP rotates subcarrier k by exp(+j 2 pi k FFT_StartTime / 1024);
         it is undone with exp(-j 2 pi k FFT_StartTime / 1024), k = -420..420.
       * demodulator gain: OFDM_Demodulator_841 divides by sqrt(1024) and by
         sqrt(2), so a 20 MHz Tx-level waveform comes out 1/sqrt(2) lower. The
         grid is divided by it, so an ideal loopback gives back the Tx grid exactly.
         (The decimation gain is removed earlier, by Decimate_Payload_Rx.py.)
     This is NOT channel equalization (Equalize_Payload_Grid.py does that).
  5. Save the grid to Rx/<File_Rx.Grid_Payload_Rx>, laid out like
     Grid_Payload_Tx.h5: 'data' (PacketLength, 841), one row per OFDM symbol.

Nothing is written to the JSON.

Usage:
    python Demodulate_Waveform_Rx.py
    python Demodulate_Waveform_Rx.py --input other.h5 --output My_Grid.h5
    python Demodulate_Waveform_Rx.py --start-time 0 --no-derotate --no-normalize
    python Demodulate_Waveform_Rx.py --ray 2        # antenna 2: *_Ray2.h5 in and out (default --ray 1)

The output .h5 file contains:
    data   (PacketLength, 841) complex128   payload grid, row = OFDM symbol
    attrs  FFT_StartTime, PacketLength, Derotated, Gain, GainRemoved, SourceFile,
           and SampleRateIn, Decimation, Decimator copied from the input file
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType
from typing import Any

import numpy as np

# Make flexlink_paths importable when the script is run from anywhere.
for _parent in Path(__file__).resolve().parents:
    if (_parent / "flexlink_paths.py").is_file():
        sys.path.insert(0, str(_parent))
        break
from flexlink_paths import BASIC_MODULES, MAX_RAYS, ray_file_name  # noqa: E402

try:
    from flexlink_paths import OFDM_MODULATOR  # noqa: E402
except ImportError:
    OFDM_MODULATOR = BASIC_MODULES / "OFDM_Modulator"

RX_DIR = Path(__file__).resolve().parent          # the Rx folder
PARAMS_FILE = RX_DIR / "Parameters_Rx_payload.json"
DEFINED_SECTION = "Defined_Rx"
FILE_SECTION = "File_Rx"

DEMODULATOR_SCRIPT = "OFDM_Demodulator_841.py"
INPUT_KEY = "Waveform_Payload_Packet_Rx"  # File_Rx/<key> = 20 MHz waveform from Packet_Detector.py (Rx/)
OUTPUT_KEY = "Grid_Payload_Rx"          # File_Rx/<key> = output grid (Rx/)
DATASET = "data"

NUM_SUBCARRIERS = 841
FFT_SIZE = 1024                         # OFDM_Demodulator_841 works at 20 MHz
DEMOD_RATE = 20e6
UPSTREAM_ATTRS = ("SampleRateIn", "Decimation", "Decimator")   # copied from the input for reference
SUBCARRIER_K = np.arange(NUM_SUBCARRIERS) - 420   # grid column -> k = -420..420


# --------------------------------------------------------------------------- #
# Parameters
# --------------------------------------------------------------------------- #
def load_params(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"Parameter file not found: {path}")
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def get_defined(params: dict[str, Any], key: str) -> Any:
    value = params.get(DEFINED_SECTION, {}).get(key)
    if value is None:
        raise KeyError(f"{DEFINED_SECTION}/{key} not found in {PARAMS_FILE.name}")
    return value


def file_name(params: dict[str, Any], key: str) -> str:
    """File name stored under File_Rx/<key> in the JSON."""
    files = params.get(FILE_SECTION)
    name = files.get(key) if isinstance(files, dict) else None
    if not isinstance(name, str) or not name.strip():
        raise KeyError(f"{FILE_SECTION}/{key} not found in {PARAMS_FILE.name}")
    return name.strip()


def find_input(name: str) -> Path:
    """Rx/<name>."""
    path = RX_DIR / name
    if not path.is_file():
        raise FileNotFoundError(f"{path} not found - run Packet_Detector.py first")
    return path


# --------------------------------------------------------------------------- #
# Files and demodulator
# --------------------------------------------------------------------------- #
def load_demodulator() -> ModuleType:
    script = OFDM_MODULATOR / DEMODULATOR_SCRIPT
    if not script.is_file():
        raise FileNotFoundError(f"{DEMODULATOR_SCRIPT} not found in {OFDM_MODULATOR}")
    spec = importlib.util.spec_from_file_location("OFDM_Demodulator_841", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if not callable(getattr(module, "ofdm_demodulator_841", None)):
        raise ValueError(f"{script} has no ofdm_demodulator_841 function")
    return module


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
        x = to_complex(np.asarray(f[DATASET][()])).ravel()
        attrs = dict(f.attrs)
    if x.size == 0:
        raise ValueError(f"{path.name} contains an empty waveform")
    return x, attrs


def save_grid(path: Path, grid: np.ndarray, attrs: dict[str, Any]) -> None:
    import h5py
    with h5py.File(path, "w") as f:
        f.create_dataset(DATASET, data=grid)
        for k, v in attrs.items():
            f.attrs[k] = v


# --------------------------------------------------------------------------- #
# Processing
# --------------------------------------------------------------------------- #
def demodulator_gain() -> float:
    """Amplitude OFDM_Demodulator_841 returns for a unit-power Tx grid modulated at 20 MHz.

    Tx (OFDM_Modulator_841 at 20 MHz): 1024-point IFFT, output x sqrt(1024).
    Rx: 1024-point FFT, / sqrt(1024) / sqrt(2)  ->  gain = 1/sqrt(2).
    A 40 MHz waveform has already been brought to this level by Decimate_Payload_Rx.py.
    """
    return float(1.0 / np.sqrt(2))


def demodulate_waveform(x20: np.ndarray, start_time: int, packet_length: int,
                        demodulator: ModuleType, derotate: bool = True, normalize: bool = True
                        ) -> tuple[np.ndarray, dict[str, Any]]:
    """Return the (packet_length, 841) payload grid and processing info."""
    grid = demodulator.ofdm_demodulator_841(x20, start_time).T     # -> (symbols, 841)
    if grid.shape[0] < packet_length:
        raise ValueError(f"Waveform holds only {grid.shape[0]} OFDM symbols, "
                         f"PacketLength is {packet_length}")
    grid = grid[:packet_length, :]

    if derotate and start_time:
        grid = grid * np.exp(-2j * np.pi * SUBCARRIER_K * start_time / FFT_SIZE)[None, :]

    gain = demodulator_gain()
    if normalize:
        grid = grid / gain

    return grid, {"Gain": gain,
                  "Derotated": int(bool(derotate and start_time)),
                  "GainRemoved": int(normalize)}


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Demodulate the 20 MHz payload waveform into the payload grid.")
    ap.add_argument("--params", type=Path, default=PARAMS_FILE)
    ap.add_argument("--input", type=Path,
                    help=f"20 MHz waveform .h5 (default: Rx/<File_Rx.{INPUT_KEY}>)")
    ap.add_argument("--output", type=Path, help=f"output .h5 (default: Rx/<File_Rx.{OUTPUT_KEY}>)")
    ap.add_argument("--start-time", type=int, help="override Defined_Rx/FFT_StartTime")
    ap.add_argument("--no-derotate", action="store_true", help="keep the FFT_StartTime phase ramp")
    ap.add_argument("--no-normalize", action="store_true", help="keep the demodulator gain")
    ap.add_argument("--ray", type=int, choices=range(1, MAX_RAYS + 1), metavar="N", default=1,
                    help="antenna (ray) 1-4, default 1: read and write the _RayN files "
                         "(start with Packet_Detector.py --ray N)")
    a = ap.parse_args(argv)

    try:
        if importlib.util.find_spec("h5py") is None:
            raise ValueError("Reading/writing .h5 files needs h5py - install it with: pip install h5py")

        params = load_params(a.params)
        packet_length = int(get_defined(params, "PacketLength"))
        start_time = a.start_time if a.start_time is not None else int(get_defined(params, "FFT_StartTime"))

        in_path = a.input or find_input(ray_file_name(file_name(params, INPUT_KEY), a.ray))
        out_path = a.output or RX_DIR / ray_file_name(file_name(params, OUTPUT_KEY), a.ray)

        x, in_attrs = load_waveform(in_path)
        fs = in_attrs.get("SampleRate")
        if fs is None:
            print(f"Warning: {in_path.name} has no SampleRate attribute - assuming 20 MHz")
        elif float(fs) != DEMOD_RATE:
            raise ValueError(f"{in_path.name} is at {float(fs):g} Hz; the demodulator needs 20 MHz - "
                             "run Decimate_Payload_Rx.py and Packet_Detector.py first")

        grid, info = demodulate_waveform(x, start_time, packet_length, load_demodulator(),
                                         not a.no_derotate, not a.no_normalize)

        attrs = {k: in_attrs[k] for k in UPSTREAM_ATTRS if k in in_attrs}
        attrs.update({
            "FFT_StartTime": start_time, "PacketLength": packet_length,
            "Derotated": info["Derotated"], "Gain": info["Gain"],
            "GainRemoved": info["GainRemoved"], "SourceFile": in_path.name, "Ray": a.ray,
        })
        out_path.parent.mkdir(parents=True, exist_ok=True)
        save_grid(out_path, grid, attrs)

        nz = grid[np.abs(grid) > 1e-6]
        print(f"Input {in_path.name}: {x.size} samples at 20 MHz")
        print(f"OFDM_Demodulator_841, start_time {start_time}: kept {packet_length} OFDM symbols")
        print(f"Phase ramp removed: {'yes' if info['Derotated'] else 'no'}, "
              f"demodulator gain {info['Gain']:.4f} removed: {'yes' if info['GainRemoved'] else 'no'}")
        print(f"Grid {grid.shape[0]} x {grid.shape[1]}, mean power of non-zero REs "
              f"{np.mean(np.abs(nz) ** 2):.4f}")
        print(f"Wrote {out_path}")
    except (FileNotFoundError, KeyError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
