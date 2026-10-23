#!/usr/bin/env python3
"""
Packet_Detector.py  (lives in Flexlink_py/Rx)

Finds the packet in the 20 MHz waveform from Decimate_Payload_Rx.py, removes the
carrier frequency offset and cuts the waveform so it starts at the first payload
sample. Runs between Decimate_Payload_Rx.py and Demodulate_Waveform_Rx.py.
Python version of the MATLAB Flexlink_PacketDetect_Rx (one waveform).

Defined_Rx/Use_Preamble = 0: the waveform is passed through unchanged (no preamble
was sent). Use_Preamble = 1 (default when the key is missing): the preamble
[AGC burst | Preamble A | Preamble B] is processed with the scripts in
Flexlink_BasicModules/Preamble:

  1. LP filter (2.75 MHz)          Receive_LP_Filter.lp_filter_flexlink
                                   (more sensitive packet detection)
  2. Packet detection              Receive_Packet_Detector.packet_detector_flexlink on
                                   the filtered signal; SA_Len and thresholds from
                                   Receive_Packet_Detector.json
  3. Frequency offset              Receive_Get_Frequency_Offset.detect_frequency_offset
                                   on the filtered signal, from the detection flag:
                                   coarse (FFT of Preamble A) + fine (lag-32 autocorr.)
  4. Frequency correction (NCO)    done here (no BasicModule does it):
                                   x[n] * exp(-j 2 pi n (coarse + fine) / 20e6), n = 1..N
                                   on the unfiltered waveform, as in the MATLAB code
  5. Timing                        Receive_Get_Timing_Offset.detect_timing_offset on the
                                   corrected waveform: matched filter with the 20 MHz
                                   Preamble B from Preamble.h5; PeakPosition (1-based)
                                   is the last Preamble B sample
  6. Extract the payload           output = corrected[PeakPosition:]
                                   (MATLAB Signal(PeakPosition+1:end))

Rays (receive antennas)
-----------------------
When defects are modeled, Waveform_Payload_Dec_Rx holds one row per antenna
(NumberOfAntennas). Pick the ray with --ray N (1-based, default 1); the output
name always ends in _RayN (e.g. Decimated_Payload_Packet_Rx_Ray2.h5, and _Ray1
for a single waveform) and the later steps take the same --ray N.

Inputs
------
Rx/Parameters_Rx_payload.json          Defined_Rx/Use_Preamble
Rx/<File_Rx.Waveform_Payload_Dec_Rx>   'data' complex, 20 MHz (Decimate_Payload_Rx.py),
                                       1-D or (NumberOfAntennas, N)
Flexlink_BasicModules/Preamble/Preamble.h5
                                       local Preamble B for the timing search

Output
------
Rx/<File_Rx.Waveform_Payload_Packet_Rx>  (default name Decimated_Payload_Packet_Rx.h5,
                                          with _RayN added: ..._Ray1.h5 by default)
    data   (M,) complex128   20 MHz waveform starting at the first payload sample
    attrs  SampleRate, UsePreamble, Ray (1-4),
           PayloadStart (input index of output sample 0),
           SourceFile, SampleRateIn/Decimation/Decimator (copied from the input), and
           with the preamble: StartOfPacket (first detection sample, 0-based),
           FreqOffset_Coarse, FreqOffset_Fine, FreqOffset_Total (Hz), PeakPosition
           (1-based), PeakMagnitude, SA_Len, Thresh_Lo, Thresh_Hi, PreambleFile

Nothing is written to the JSON. Exit status 1 for input errors, 2 if no packet is found.

Usage
-----
    python Rx/Packet_Detector.py
    python Rx/Packet_Detector.py --use-preamble 0          # override the JSON
    python Rx/Packet_Detector.py --input other.h5 --output My_Packet.h5
    python Rx/Packet_Detector.py --ray 2                   # second antenna (defects modeled)
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
    from flexlink_paths import PREAMBLE  # noqa: E402
except ImportError:
    PREAMBLE = BASIC_MODULES / "Preamble"

RX_DIR = Path(__file__).resolve().parent          # the Rx folder
PARAMS_FILE = RX_DIR / "Parameters_Rx_payload.json"
DEFINED_SECTION = "Defined_Rx"
FILE_SECTION = "File_Rx"

INPUT_KEY = "Waveform_Payload_Dec_Rx"       # File_Rx/<key> = 20 MHz waveform (Rx/)
OUTPUT_KEY = "Waveform_Payload_Packet_Rx"   # File_Rx/<key> = output (Rx/)
DEFAULT_OUTPUT = "Decimated_Payload_Packet_Rx.h5"
PREAMBLE_FILE = "Preamble.h5"
DATASET = "data"

FS = 20e6
DEFAULT_USE_PREAMBLE = 1
UPSTREAM_ATTRS = ("SampleRateIn", "Decimation", "Decimator", "DefectsModeled", "DefectsApplied")

MODULES = {  # name -> script in Flexlink_BasicModules/Preamble
    "lpf": "Receive_LP_Filter.py",
    "det": "Receive_Packet_Detector.py",
    "freq": "Receive_Get_Frequency_Offset.py",
    "timing": "Receive_Get_Timing_Offset.py",
}


class NoPacketError(RuntimeError):
    pass


# --------------------------------------------------------------------------- #
# Parameters
# --------------------------------------------------------------------------- #
def load_params(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"Parameter file not found: {path}")
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def file_name(params: dict[str, Any], key: str, default: str | None = None) -> str:
    files = params.get(FILE_SECTION)
    name = files.get(key) if isinstance(files, dict) else None
    if name is None and default is not None:
        print(f"Warning: {FILE_SECTION}/{key} not set - using {default}")
        name = default
    if not isinstance(name, str) or not name.strip():
        raise KeyError(f"{FILE_SECTION}/{key} not found in {PARAMS_FILE.name}")
    return name.strip()


def use_preamble(params: dict[str, Any], override: int | None) -> int:
    value = override if override is not None else params.get(DEFINED_SECTION, {}).get("Use_Preamble")
    if value is None:
        print(f"Warning: {DEFINED_SECTION}/Use_Preamble not set - using {DEFAULT_USE_PREAMBLE}")
        value = DEFAULT_USE_PREAMBLE
    if str(value).strip() not in ("0", "1"):
        raise ValueError(f"Use_Preamble {value!r} must be 0 or 1")
    return int(str(value).strip())


# --------------------------------------------------------------------------- #
# Files and modules
# --------------------------------------------------------------------------- #
def to_complex(arr: np.ndarray) -> np.ndarray:
    if arr.dtype.names:
        names = {n.lower(): n for n in arr.dtype.names}
        re = names.get("real") or names.get("r")
        im = names.get("imag") or names.get("i")
        if re and im:
            return arr[re].astype(float) + 1j * arr[im].astype(float)
    return arr.astype(np.complex128)


def load_waveform(path: Path, ray: int = 1) -> tuple[np.ndarray, dict[str, Any]]:
    """Load the 20 MHz waveform; with ray N pick row N of a (NumberOfAntennas, N) file."""
    import h5py
    if not path.is_file():
        raise FileNotFoundError(f"{path} not found - run Decimate_Payload_Rx.py first")
    with h5py.File(path, "r") as f:
        if DATASET not in f:
            raise KeyError(f"{path.name} has no '{DATASET}' dataset (has {list(f)})")
        x = to_complex(np.asarray(f[DATASET][()]))
        attrs = dict(f.attrs)
    if x.ndim == 2:
        if ray > x.shape[0]:
            raise ValueError(f"--ray {ray}: {path.name} holds only {x.shape[0]} antenna(s)")
        x = x[ray - 1]
    elif x.ndim == 1:
        if ray != 1:
            raise ValueError(f"--ray {ray}: {path.name} holds a single waveform (no defects modeled) - "
                             "use --ray 1")
    else:
        raise ValueError(f"{path.name}: '{DATASET}' must be 1-D or (antennas, samples), got {x.shape}")
    if x.size == 0:
        raise ValueError(f"{path.name} contains an empty waveform")
    fs = attrs.get("SampleRate")
    if fs is not None and float(fs) != FS:
        raise ValueError(f"{path.name} is at {float(fs):g} Hz; expected 20 MHz - run Decimate_Payload_Rx.py")
    return x, attrs


def save_waveform(path: Path, x: np.ndarray, attrs: dict[str, Any]) -> None:
    import h5py
    with h5py.File(path, "w") as f:
        f.create_dataset(DATASET, data=x)
        for k, v in attrs.items():
            f.attrs[k] = v


def load_module(script: str) -> ModuleType:
    path = Path(PREAMBLE) / script
    if not path.is_file():
        raise FileNotFoundError(f"{script} not found in {PREAMBLE}")
    name = path.stem
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


# --------------------------------------------------------------------------- #
# Processing
# --------------------------------------------------------------------------- #
def process_preamble(x: np.ndarray, mods: dict[str, ModuleType], preamble_path: Path
                     ) -> tuple[np.ndarray, dict[str, Any]]:
    """MATLAB steps 2b-2f. Returns (payload waveform, info)."""
    det = mods["det"]
    sa_len, thr_lo, thr_hi = det.SA_LEN_DEFAULT, det.THRESH_LO_DEFAULT, det.THRESH_HI_DEFAULT

    # 2b. LP filter for more sensitive packet detection
    x_lpf = mods["lpf"].lp_filter_flexlink(x)

    # 2c. Packet detection
    flag, ratio, _, _ = det.packet_detector_flexlink(x_lpf, sa_len, thr_lo, thr_hi)
    if not flag.any():
        raise NoPacketError(f"No packet detected (max comparison ratio {np.nanmax(ratio):.3f}, "
                            f"Thresh_Hi {thr_hi:g})")

    # 2d. Frequency offset estimate (filtered signal), then NCO on the unfiltered waveform
    coarse, fine, fdet = mods["freq"].detect_frequency_offset(flag, x_lpf)
    n = np.arange(1, x.size + 1)                                   # MATLAB n = 1:length
    x_corr = x * np.exp(-1j * 2 * np.pi * n * coarse / FS) * np.exp(-1j * 2 * np.pi * n * fine / FS)

    # 2e. Timing offset: matched filter with Preamble B (end of Preamble B, 1-based)
    pb = mods["timing"].load_preamble_b(str(preamble_path))
    peak_pos, _, peak_val = mods["timing"].detect_timing_offset(x_corr, pb)
    if peak_pos == 0:
        raise NoPacketError("Timing search found no Preamble B (all-zero correlation)")
    if peak_pos >= x.size:
        raise NoPacketError(f"Preamble B ends at sample {peak_pos} - no payload samples after it")

    # 2f. Extract the OFDM symbols: MATLAB Signal(PeakPosition+1:end)
    payload = x_corr[peak_pos:]
    info = {"StartOfPacket": int(fdet["StartOfPacket"]), "FreqOffset_Coarse": float(coarse),
            "FreqOffset_Fine": float(fine), "FreqOffset_Total": float(coarse + fine),
            "PeakPosition": int(peak_pos), "PeakMagnitude": float(abs(peak_val)),
            "PayloadStart": int(peak_pos), "SA_Len": int(sa_len), "Thresh_Lo": float(thr_lo),
            "Thresh_Hi": float(thr_hi), "PreambleFile": preamble_path.name,
            "PreambleBLength": int(pb.size)}
    return payload, info


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Packet detection, frequency and timing correction "
                                             "from the preamble.")
    ap.add_argument("--params", type=Path, default=PARAMS_FILE)
    ap.add_argument("--input", type=Path, help=f"20 MHz waveform .h5 (default: Rx/<File_Rx.{INPUT_KEY}>)")
    ap.add_argument("--output", type=Path, help=f"output .h5 (default: Rx/<File_Rx.{OUTPUT_KEY}>)")
    ap.add_argument("--preamble-file", type=Path,
                    help=f"Preamble.h5 for the local Preamble B (default: {PREAMBLE_FILE} in "
                         "Flexlink_BasicModules/Preamble)")
    ap.add_argument("--use-preamble", type=int, choices=(0, 1), help="override Defined_Rx/Use_Preamble")
    ap.add_argument("--ray", type=int, choices=range(1, MAX_RAYS + 1), metavar="N", default=1,
                    help="antenna (ray) 1-4, default 1; the output name ends in _RayN")
    a = ap.parse_args(argv)

    try:
        if importlib.util.find_spec("h5py") is None:
            raise ValueError("Reading/writing .h5 files needs h5py - install it with: pip install h5py")

        params = load_params(a.params)
        with_preamble = use_preamble(params, a.use_preamble)
        in_path = a.input or RX_DIR / file_name(params, INPUT_KEY)
        out_path = a.output or RX_DIR / ray_file_name(file_name(params, OUTPUT_KEY, DEFAULT_OUTPUT), a.ray)

        x, in_attrs = load_waveform(in_path, a.ray)
        attrs = {k: in_attrs[k] for k in UPSTREAM_ATTRS if k in in_attrs}
        attrs.update({"SampleRate": FS, "UsePreamble": with_preamble, "SourceFile": in_path.name,
                      "Ray": a.ray})

        if with_preamble:
            preamble_path = a.preamble_file or Path(PREAMBLE) / PREAMBLE_FILE
            mods = {k: load_module(v) for k, v in MODULES.items()}
            y, info = process_preamble(x, mods, preamble_path)
            attrs.update(info)
        else:
            y = x
            attrs["PayloadStart"] = 0

        out_path.parent.mkdir(parents=True, exist_ok=True)
        save_waveform(out_path, y, attrs)

        print(f"Input {in_path.name}, ray {a.ray}: {x.size} samples at 20 MHz")
        if with_preamble:
            print(f"Packet detected at sample {info['StartOfPacket']} "
                  f"(SA_Len {info['SA_Len']}, thresholds {info['Thresh_Lo']:g}/{info['Thresh_Hi']:g})")
            print(f"Frequency offset: coarse {info['FreqOffset_Coarse'] / 1e3:+.3f} kHz, fine "
                  f"{info['FreqOffset_Fine'] / 1e3:+.3f} kHz, total {info['FreqOffset_Total'] / 1e3:+.3f} kHz "
                  "- removed")
            print(f"Preamble B ends at sample {info['PeakPosition'] - 1} (PeakPosition {info['PeakPosition']}, "
                  f"|peak| {info['PeakMagnitude']:.2f}); payload starts at sample {info['PayloadStart']}")
        else:
            print("Use_Preamble 0: waveform passed through unchanged")
        print(f"Output {y.size} samples")
        print(f"Wrote {out_path}")
    except NoPacketError as exc:
        print(f"No packet: {exc}", file=sys.stderr)
        return 2
    except (FileNotFoundError, KeyError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
