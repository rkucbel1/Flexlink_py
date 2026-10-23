#!/usr/bin/env python3
"""
Construct_Payload_Waveform.py  (lives in Flexlink_py/Tx)

Turns the payload resource grid into a time-domain OFDM waveform, with the
preamble in front of it when Defined/Use_Preamble is 1, and the control field
between the preamble and the payload when Defined/Use_ControlField is 1.

Steps:
  1. Read Tx/Parameters_Tx_payload.json.
  2. Load the payload grid from Tx/<File.Grid_Payload_Tx> (written by
     Construct_Payload_Grid.py): dataset 'data', complex, (PacketLength, 841),
     one row per OFDM symbol.
  3. Run every OFDM symbol through ofdm_modulator_841() from
     Flexlink_BasicModules/OFDM_Modulator/OFDM_Modulator_841.py, using
     Defined/SampleRate (20 MHz -> 1024-point IFFT, CP 116;
     40 MHz -> 2048-point IFFT, CP 232), and join the symbols in order.
  4. If Defined/Use_Preamble is 1 (default 1 when the key is missing), prepend the
     preamble [AGC burst | Preamble A | Preamble B] read from
     Flexlink_BasicModules/Preamble/<File.Preamble> (dataset 'Preamble', written
     by Construct_Preamble.py). Its SampleRate must equal Defined/SampleRate.
     The preamble is scaled so its average power equals the average power of
     this payload (scale = sqrt(P_payload / P_preamble), stored as PreambleScale);
     the payload power depends on Mod and the reference-signal layout (16/64QAM
     pilots are boosted), so it is measured, not assumed. Preamble.h5 is not changed.
     Use_Preamble 0 sends the payload alone (for testing).
  5. If Defined/Use_ControlField is 1 (default 1 when the key is missing), insert
     the control field waveform from Tx/ControlSignals/<File.Waveform_Control_Tx>
     of Parameters_Control_Field.json (written by Create_ControlField_Waveform.py)
     between the preamble and the payload. It is built once and only rebuilt when
     the Tx parameters change. Its SampleRate must equal Defined/SampleRate, and it
     is scaled to the payload's average power like the preamble (ControlScale).
  6. Save the waveform to Tx/<File.Waveform_Payload_Tx>.

Nothing is written to the JSON.

Usage:
    python Construct_Payload_Waveform.py
    python Construct_Payload_Waveform.py --grid other.h5 --output My_Waveform.h5
    python Construct_Payload_Waveform.py --use-preamble 0     # override the JSON
    python Construct_Payload_Waveform.py --use-controlfield 1 # override the JSON

The output .h5 file contains:
    data   (PreambleLength + ControlLength + PacketLength * OfdmSymbolLength,) complex128
           waveform samples: [preamble | control field | payload]
    attrs  SampleRate, IfftSize, CpLength, OfdmSymbolLength, NumOfdmSymbols,
           Port, Mod, SourceFile,
           UsePreamble (0/1), PreambleLength (samples, 0 without preamble),
           UseControlField (0/1), ControlLength (samples, 0 without control field),
           ControlStart (index of the first control field sample = PreambleLength),
           PayloadStart (index of the first payload sample = PreambleLength + ControlLength),
           and with the preamble: PreambleFile, PreambleScale, PreamblePower,
           PayloadPower, AgcBurstLength, PreambleALength, PreambleBLength,
           and with the control field: ControlFile, ControlScale, ControlPower
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
from flexlink_paths import TX, BASIC_MODULES  # noqa: E402

try:
    from flexlink_paths import OFDM_MODULATOR  # noqa: E402
except ImportError:
    OFDM_MODULATOR = BASIC_MODULES / "OFDM_Modulator"
try:
    from flexlink_paths import PREAMBLE  # noqa: E402
except ImportError:
    PREAMBLE = BASIC_MODULES / "Preamble"

PARAMS_FILE = TX / "Parameters_Tx_payload.json"
MODULATOR_SCRIPT = "OFDM_Modulator_841.py"
GRID_KEY = "Grid_Payload_Tx"            # File/<key> = input grid (in Tx/)
OUTPUT_KEY = "Waveform_Payload_Tx"      # File/<key> = output waveform (in Tx/)
GRID_DATASET = "data"
PREAMBLE_KEY = "Preamble"               # File/<key> = preamble file (in Flexlink_BasicModules/Preamble/)
PREAMBLE_DATASET = "Preamble"           # dataset name written by Construct_Preamble.py
PREAMBLE_PART_ATTRS = ("AgcBurstLength", "PreambleALength", "PreambleBLength")
DEFAULT_USE_PREAMBLE = 1
DEFAULT_USE_CONTROLFIELD = 1

CONTROL_PARAMS_FILE = TX / "ControlSignals" / "Parameters_Control_Field.json"
CONTROL_KEY = "Waveform_Control_Tx"     # File/<key> in Parameters_Control_Field.json (in Tx/ControlSignals/)

NUM_SUBCARRIERS = 841
VALID_SAMPLE_RATES = (20e6, 40e6)       # the rates OFDM_Modulator_841 is built for


# --------------------------------------------------------------------------- #
# Parameters
# --------------------------------------------------------------------------- #
def load_params(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"Parameter file not found: {path}")
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def file_name(params: dict[str, Any], key: str) -> str:
    """File name stored under File/<key> in the JSON."""
    files = params.get("File")
    name = files.get(key) if isinstance(files, dict) else None
    if not isinstance(name, str) or not name.strip():
        raise KeyError(f"File/{key} not found in {PARAMS_FILE.name}")
    return name.strip()


def sample_rate(params: dict[str, Any]) -> float:
    value = params.get("Defined", {}).get("SampleRate")
    if value is None:
        raise KeyError(f"Defined/SampleRate not found in {PARAMS_FILE.name}")
    fs = float(value)
    if fs not in VALID_SAMPLE_RATES:
        raise ValueError(f"SampleRate {value} must be 20000000 or 40000000 "
                         "(the rates OFDM_Modulator_841 supports)")
    return fs


def use_preamble(params: dict[str, Any], override: int | None) -> int:
    value = override if override is not None else params.get("Defined", {}).get("Use_Preamble")
    if value is None:
        print(f"Warning: Defined/Use_Preamble not set - using {DEFAULT_USE_PREAMBLE}")
        value = DEFAULT_USE_PREAMBLE
    if str(value).strip() not in ("0", "1"):
        raise ValueError(f"Use_Preamble {value!r} must be 0 or 1")
    return int(str(value).strip())


def use_controlfield(params: dict[str, Any], override: int | None) -> int:
    value = override if override is not None else params.get("Defined", {}).get("Use_ControlField")
    if value is None:
        value = DEFAULT_USE_CONTROLFIELD
    if str(value).strip() not in ("0", "1"):
        raise ValueError(f"Use_ControlField {value!r} must be 0 or 1")
    return int(str(value).strip())


# --------------------------------------------------------------------------- #
# Files and modulator
# --------------------------------------------------------------------------- #
def load_modulator() -> ModuleType:
    script = OFDM_MODULATOR / MODULATOR_SCRIPT
    if not script.is_file():
        raise FileNotFoundError(f"{MODULATOR_SCRIPT} not found in {OFDM_MODULATOR}")
    spec = importlib.util.spec_from_file_location("OFDM_Modulator_841", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if not callable(getattr(module, "ofdm_modulator_841", None)):
        raise ValueError(f"{script} has no ofdm_modulator_841 function")
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


def load_grid(path: Path) -> tuple[np.ndarray, dict[str, Any]]:
    """Return the payload grid as (NumOfdmSymbols, 841) and the file's attributes."""
    import h5py
    if not path.is_file():
        raise FileNotFoundError(f"Payload grid not found: {path} - run Construct_Payload_Grid.py first")
    with h5py.File(path, "r") as f:
        if GRID_DATASET not in f:
            raise KeyError(f"{path.name} has no '{GRID_DATASET}' dataset (has {list(f)})")
        grid = to_complex(np.asarray(f[GRID_DATASET][()]))
        attrs = dict(f.attrs)
    if grid.ndim != 2 or grid.shape[1] != NUM_SUBCARRIERS:
        raise ValueError(f"{path.name}: expected (symbols, {NUM_SUBCARRIERS}), got {grid.shape}")
    return grid, attrs


def load_preamble(path: Path, fs: float) -> tuple[np.ndarray, dict[str, Any]]:
    """Return the preamble samples and its attributes (part lengths, SampleRate)."""
    import h5py
    if not path.is_file():
        raise FileNotFoundError(f"Preamble not found: {path} - run Construct_Preamble.py first")
    with h5py.File(path, "r") as f:
        name = PREAMBLE_DATASET if PREAMBLE_DATASET in f else "data" if "data" in f else None
        if name is None:
            raise KeyError(f"{path.name} has no '{PREAMBLE_DATASET}' dataset (has {list(f)})")
        preamble = to_complex(np.asarray(f[name][()])).ravel()
        attrs = {**dict(f.attrs), **dict(f[name].attrs)}     # Construct_Preamble puts them on the dataset
    if preamble.size == 0:
        raise ValueError(f"{path.name} contains an empty preamble")
    p_fs = attrs.get("SampleRate")
    if p_fs is None:
        print(f"Warning: {path.name} has no SampleRate attribute - assuming {fs / 1e6:.0f} MHz")
    elif float(p_fs) != fs:
        raise ValueError(f"{path.name} was built at {float(p_fs) / 1e6:.0f} MHz but SampleRate is "
                         f"{fs / 1e6:.0f} MHz - rerun Construct_Preamble.py")
    return preamble.astype(np.complex128), attrs


def load_controlfield(fs: float) -> tuple[np.ndarray, dict[str, Any], Path]:
    """Return the control field samples, their attributes and the file they came from."""
    import h5py
    control_params = load_params(CONTROL_PARAMS_FILE)
    name = control_params.get("File", {}).get(CONTROL_KEY)
    if not isinstance(name, str) or not name.strip():
        raise KeyError(f"File/{CONTROL_KEY} not found in {CONTROL_PARAMS_FILE.name}")
    path = CONTROL_PARAMS_FILE.parent / name.strip()
    if not path.is_file():
        raise FileNotFoundError(f"Control field waveform not found: {path} - "
                                "run Create_ControlField_Waveform.py first")
    with h5py.File(path, "r") as f:
        control = to_complex(np.asarray(f["data"][()])).ravel()
        attrs = dict(f.attrs)
    if control.size == 0:
        raise ValueError(f"{path.name} contains an empty waveform")
    c_fs = attrs.get("SampleRate")
    if c_fs is None or float(c_fs) != fs:
        raise ValueError(f"{path.name} was built at "
                         f"{'unknown' if c_fs is None else f'{float(c_fs) / 1e6:.0f} MHz'} but SampleRate is "
                         f"{fs / 1e6:.0f} MHz - set SampleRate in {CONTROL_PARAMS_FILE.name} and rebuild "
                         "the control field")
    return control.astype(np.complex128), attrs, path


def save_waveform(path: Path, waveform: np.ndarray, attrs: dict[str, Any]) -> None:
    import h5py
    with h5py.File(path, "w") as f:
        f.create_dataset("data", data=waveform)
        for k, v in attrs.items():
            f.attrs[k] = v


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def construct_payload_waveform(grid: np.ndarray, fs: float, modulator: ModuleType) -> np.ndarray:
    """Modulate each OFDM symbol (one grid row) and join them in order."""
    symbols = [modulator.ofdm_modulator_841(grid[n, :], fs) for n in range(grid.shape[0])]
    return np.concatenate(symbols)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Build the payload OFDM waveform from the payload grid.")
    ap.add_argument("--params", type=Path, default=PARAMS_FILE)
    ap.add_argument("--grid", type=Path, help=f"payload grid .h5 (default: Tx/<File.{GRID_KEY}>)")
    ap.add_argument("--output", type=Path, help=f"output .h5 (default: Tx/<File.{OUTPUT_KEY}>)")
    ap.add_argument("--use-preamble", type=int, choices=(0, 1),
                    help="override Defined/Use_Preamble (1 = prepend the preamble)")
    ap.add_argument("--use-controlfield", type=int, choices=(0, 1),
                    help="override Defined/Use_ControlField (1 = insert the control field)")
    a = ap.parse_args(argv)

    try:
        if importlib.util.find_spec("h5py") is None:
            raise ValueError("Reading/writing .h5 files needs h5py - install it with: pip install h5py")

        params = load_params(a.params)
        fs = sample_rate(params)
        grid_path = a.grid or TX / file_name(params, GRID_KEY)
        out_path = a.output or TX / file_name(params, OUTPUT_KEY)

        with_preamble = use_preamble(params, a.use_preamble)
        with_control = use_controlfield(params, a.use_controlfield)
        grid, grid_attrs = load_grid(grid_path)
        modulator = load_modulator()
        payload = construct_payload_waveform(grid, fs, modulator)

        n_sym = grid.shape[0]
        ifft_size = 1024 if fs == 20e6 else 2048
        cp_len = 116 if fs == 20e6 else 232
        sym_len = ifft_size + cp_len
        if payload.size != n_sym * sym_len:
            raise ValueError(f"Payload waveform has {payload.size} samples, expected "
                             f"{n_sym} x {sym_len} = {n_sym * sym_len}")

        preamble_attrs: dict[str, Any] = {}
        if with_preamble:
            preamble_path = PREAMBLE / file_name(params, PREAMBLE_KEY)
            preamble, p_attrs = load_preamble(preamble_path, fs)
            p_payload = float(np.mean(np.abs(payload) ** 2))
            p_preamble_file = float(np.mean(np.abs(preamble) ** 2))
            if p_preamble_file <= 0:
                raise ValueError(f"{preamble_path.name} has zero power")
            scale = float(np.sqrt(p_payload / p_preamble_file))
            preamble = preamble * scale                     # match the payload average power
            preamble_attrs = {"PreambleFile": preamble_path.name, "PreambleScale": scale,
                              "PreamblePower": float(np.mean(np.abs(preamble) ** 2)),
                              "PayloadPower": p_payload,
                              **{k: p_attrs[k] for k in PREAMBLE_PART_ATTRS if k in p_attrs}}
        else:
            preamble = np.zeros(0, dtype=np.complex128)

        control_attrs: dict[str, Any] = {}
        if with_control:
            control, c_attrs, control_path = load_controlfield(fs)
            p_payload = float(np.mean(np.abs(payload) ** 2))
            p_control_file = float(np.mean(np.abs(control) ** 2))
            if p_control_file <= 0:
                raise ValueError(f"{control_path.name} has zero power")
            c_scale = float(np.sqrt(p_payload / p_control_file))
            control = control * c_scale                     # match the payload average power
            control_attrs = {"ControlFile": control_path.name, "ControlScale": c_scale,
                             "ControlPower": float(np.mean(np.abs(control) ** 2))}
        else:
            control = np.zeros(0, dtype=np.complex128)

        waveform = np.concatenate([preamble, control, payload])

        out_path.parent.mkdir(parents=True, exist_ok=True)
        save_waveform(out_path, waveform, {
            "SampleRate": fs, "IfftSize": ifft_size, "CpLength": cp_len,
            "OfdmSymbolLength": sym_len, "NumOfdmSymbols": n_sym,
            "Port": grid_attrs.get("Port", params.get("Defined", {}).get("Port", -1)),
            "Mod": str(grid_attrs.get("Mod", params.get("Defined", {}).get("Mod", ""))),
            "SourceFile": grid_path.name,
            "UsePreamble": with_preamble, "PreambleLength": preamble.size,
            "UseControlField": with_control, "ControlLength": control.size,
            "ControlStart": preamble.size,
            "PayloadStart": preamble.size + control.size, **preamble_attrs, **control_attrs,
        })

        p_avg = float(np.mean(np.abs(payload) ** 2))
        print(f"SampleRate {fs / 1e6:.0f} MHz -> IFFT {ifft_size}, CP {cp_len}, "
              f"{sym_len} samples per OFDM symbol")
        print(f"Grid {grid.shape[0]} x {grid.shape[1]} from {grid_path.name}")
        print(f"Payload: {n_sym} symbols x {sym_len} = {payload.size} samples, "
              f"RMS {np.sqrt(p_avg):.4f}, peak |x| {np.max(np.abs(payload)):.4f}")
        if with_preamble:
            p_pre = float(np.mean(np.abs(preamble) ** 2))
            print(f"Preamble from {preamble_attrs['PreambleFile']}: {preamble.size} samples "
                  f"({preamble.size / fs * 1e6:.1f} us), scaled by {preamble_attrs['PreambleScale']:.4f} "
                  f"({20 * np.log10(preamble_attrs['PreambleScale']):+.2f} dB) to RMS {np.sqrt(p_pre):.4f} "
                  f"= payload RMS")
        else:
            print("Use_Preamble 0: no preamble")
        if with_control:
            print(f"Control field from {control_attrs['ControlFile']}: {control.size} samples "
                  f"({control.size / fs * 1e6:.1f} us), scaled by {control_attrs['ControlScale']:.4f} "
                  f"({20 * np.log10(control_attrs['ControlScale']):+.2f} dB) to payload RMS, "
                  f"starts at sample {preamble.size}")
        else:
            print("Use_ControlField 0: no control field")
        parts = " + ".join(n for n, x in (("preamble", preamble), ("control field", control)) if x.size)
        print(f"Waveform: {parts + ' + ' if parts else ''}payload = {waveform.size} samples, "
              f"payload starts at sample {preamble.size + control.size}")
        print(f"Wrote {out_path}")
    except (FileNotFoundError, KeyError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
