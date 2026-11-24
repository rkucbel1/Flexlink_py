#!/usr/bin/env python3
"""
Get_DataSigs_QAM.py  (lives in Flexlink_py/Rx)

Picks the Data signals out of the equalized payload grid.
Reverse of the Data part of Tx/Construct_Payload_Grid.py.

Inputs
------
Rx/Parameters_Rx_payload.json        Mod, PacketLength, RefSignalSpacing and
                                     RefSymbolSpacing (checked against the grid),
                                     Calculated_Rx/NumberOfDataSigs (checked).
Rx/<File_Rx.Grid_Equalized_Rx>       'data' (PacketLength, 841) complex and 'noise_var'
                                     (PacketLength, 841) float, written by
                                     Equalize_Payload_Grid.py (default input).
Rx/<File_Rx.Grid_Payload_Rx>         the unequalized grid from Demodulate_Waveform_Rx.py,
                                     used instead with --unequalized (testing only).
Rx/<File_Rx.ResourceGrid_Rx>
                                     'signal_types' (PacketLength, 841): 10=Ref_P0,
                                     20=Ref_P1, 30=DC, 40=Data (codes read from the
                                     file's attrs). Written by Initialize_Rx.py.

Rule
----
Data signals are read in the order Construct_Payload_Grid.py wrote them:
symbol 0 k=0..840, then symbol 1, ... (row-major over the grid). If the grid file has a
'noise_var' dataset (the equalizer's noise left on each signal), the values for the Data
signals are picked out the same way, so noise_var[i] belongs to DataSigs_QAM[i].

Output
------
Rx/<File_Rx.Sequence_DataSigs_QAM>
    data       (number of Data signals,) complex128   DataSigs_QAM
    noise_var  (number of Data signals,) float64      noise variance of each value
                                                      (only when the grid file has it)
    attrs  Mod, BitsPerSymbol, NumberOfSymbols, NumberOfDataSigs, MeanPower,
           GridFile, ResourceGridFile, HasNoiseVar

Nothing is written to the JSON.

Usage
-----
    python Rx/Get_DataSigs_QAM.py
    python Rx/Get_DataSigs_QAM.py --unequalized          # skip the equalizer (testing)
    python Rx/Get_DataSigs_QAM.py --grid other.h5 --out My_QAM.h5
    python Rx/Get_DataSigs_QAM.py --ray 2           # antenna 2: *_Ray2.h5 in and out
    (default: *_Combined.h5 when the run has several antennas, else *_Ray1.h5)
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
from flexlink_paths import MAX_RAYS  # noqa: E402
sys.path.insert(0, str(Path(__file__).resolve().parent))
from Rx_Antennas import antennas_in_run, target_label, target_name  # noqa: E402

RX_DIR = Path(__file__).resolve().parent          # the Rx folder
PARAMS_FILE = RX_DIR / "Parameters_Rx_payload.json"
DEFINED_SECTION = "Defined_Rx"
CALCULATED_SECTION = "Calculated_Rx"
FILE_SECTION = "File_Rx"

GRID_KEY = "Grid_Equalized_Rx"          # File_Rx/<key> = input grid (Rx/), from Equalize_Payload_Grid.py
UNEQUALIZED_GRID_KEY = "Grid_Payload_Rx"  # File_Rx/<key> = grid from Demodulate_Waveform_Rx.py (--unequalized)
RESOURCE_GRID_KEY = "ResourceGrid_Rx"   # File_Rx/<key> = signal types (Rx/)
OUTPUT_KEY = "Sequence_DataSigs_QAM"    # File_Rx/<key> = output (Rx/)
DATASET = "data"
NOISE_DATASET = "noise_var"
TYPES_DATASET = "signal_types"

# Used only if the resource grid file has no code attributes
# (same as SignalType in Construct_Resource_Grid.py).
DEFAULT_CODES = {"Ref_P0": 10, "Ref_P1": 20, "DC": 30, "Data": 40}
MOD_BITS = {"BPSK": 1, "QPSK": 2, "16QAM": 4, "64QAM": 6}


# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------
def load_params(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"Parameter file not found: {path}")
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def get_value(params: dict[str, Any], section: str, key: str) -> Any:
    try:
        value = params[section][key]
    except (KeyError, TypeError):
        value = None
    if value is None:
        raise KeyError(f"{section}/{key} not found in {PARAMS_FILE.name}")
    return value


def file_name(params: dict[str, Any], key: str) -> str:
    name = get_value(params, FILE_SECTION, key)
    if not isinstance(name, str) or not name.strip():
        raise ValueError(f"{FILE_SECTION}/{key} must be a file name, got {name!r}")
    return name.strip()


def mod_to_bits(value: Any) -> tuple[str, int]:
    key = str(value).strip().upper().replace("-", "").replace("_", "").replace(" ", "")
    if key not in MOD_BITS:
        raise ValueError(f"Mod {value!r} must be one of {list(MOD_BITS)}")
    return key, MOD_BITS[key]


# ---------------------------------------------------------------------------
# File reading
# ---------------------------------------------------------------------------
def to_complex(arr: np.ndarray) -> np.ndarray:
    """Accept native complex or a (real, imag) compound dtype."""
    if arr.dtype.names and {"r", "i"} <= set(arr.dtype.names):
        return arr["r"] + 1j * arr["i"]
    if arr.dtype.names and {"real", "imag"} <= set(arr.dtype.names):
        return arr["real"] + 1j * arr["imag"]
    return arr.astype(np.complex128)


def load_payload_grid(path: Path, producer: str = "Equalize_Payload_Grid.py"
                      ) -> tuple[np.ndarray, np.ndarray | None]:
    """Return (grid, noise_var grid or None if the file has no 'noise_var')."""
    import h5py
    if not path.is_file():
        raise FileNotFoundError(f"Payload grid not found: {path} - run {producer} first")
    with h5py.File(path, "r") as f:
        if DATASET not in f:
            raise KeyError(f"{path.name} has no '{DATASET}' dataset (has {list(f)})")
        grid = to_complex(np.asarray(f[DATASET][()]))
        noise_var = np.asarray(f[NOISE_DATASET][()], dtype=np.float64) if NOISE_DATASET in f else None
    if grid.ndim != 2:
        raise ValueError(f"{path.name}: expected a 2-D grid, got shape {grid.shape}")
    if noise_var is not None and noise_var.shape != grid.shape:
        raise ValueError(f"{path.name}: '{NOISE_DATASET}' is {noise_var.shape} but 'data' is {grid.shape}")
    return grid, noise_var


def load_signal_types(path: Path) -> tuple[np.ndarray, dict[str, int], dict[str, Any]]:
    import h5py
    if not path.is_file():
        raise FileNotFoundError(f"Resource grid not found: {path} - run Initialize_Rx.py first")
    with h5py.File(path, "r") as f:
        if TYPES_DATASET not in f:
            raise KeyError(f"{path.name} has no '{TYPES_DATASET}' dataset (has {list(f)})")
        types = np.asarray(f[TYPES_DATASET][()])
        attrs = dict(f.attrs)
    codes = {name: int(attrs.get(name, default)) for name, default in DEFAULT_CODES.items()}
    return types, codes, attrs


def save_qam(path: Path, values: np.ndarray, attrs: dict[str, Any],
             noise_var: np.ndarray | None = None) -> None:
    import h5py
    with h5py.File(path, "w") as f:
        f.create_dataset(DATASET, data=values)
        if noise_var is not None:
            f.create_dataset(NOISE_DATASET, data=noise_var)
        for k, v in attrs.items():
            f.attrs[k] = v


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def get_datasigs_qam(grid: np.ndarray, types: np.ndarray, data_code: int) -> np.ndarray:
    """Data signals in order: symbol 0 k=0..840, symbol 1 k=0..840, ..."""
    if grid.shape != types.shape:
        raise ValueError(f"Payload grid is {grid.shape} but the resource grid is {types.shape}. "
                         "Check PacketLength and rerun Initialize_Rx.py / Demodulate_Waveform_Rx.py.")
    return grid[types == data_code]          # boolean indexing on (symbol, k) is row-major


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Pick the Data signals out of the equalized payload grid.")
    ap.add_argument("--params", type=Path, default=PARAMS_FILE)
    ap.add_argument("--grid", type=Path, help=f"payload grid .h5 (default: Rx/<File_Rx.{GRID_KEY}>)")
    ap.add_argument("--unequalized", action="store_true",
                    help=f"read Rx/<File_Rx.{UNEQUALIZED_GRID_KEY}> instead (skip the equalizer, testing)")
    ap.add_argument("--resource-grid", type=Path,
                    help=f"resource grid .h5 (default: Rx/<File_Rx.{RESOURCE_GRID_KEY}>)")
    ap.add_argument("--out", type=Path, help=f"output .h5 (default: Rx/<File_Rx.{OUTPUT_KEY}>)")
    ap.add_argument("--ray", type=int, choices=range(1, MAX_RAYS + 1), metavar="N",
                    help="antenna (ray) 1-4: read and write its _RayN files. Default: the _Combined "
                         "files when the run has several antennas (Combine_Antennas_Rx.py), else _Ray1")
    a = ap.parse_args(argv)

    try:
        if importlib.util.find_spec("h5py") is None:
            raise ValueError("Reading/writing .h5 files needs h5py - install it with: pip install h5py")

        params = load_params(a.params)
        mod, bits = mod_to_bits(get_value(params, DEFINED_SECTION, "Mod"))
        grid_key = UNEQUALIZED_GRID_KEY if a.unequalized else GRID_KEY
        n_ant = antennas_in_run(params)
        if a.unequalized and a.ray is None and n_ant > 1:
            raise ValueError(f"--unequalized: this run has {n_ant} antennas and unequalized grids are "
                             "not combined - add --ray N")
        grid_path = a.grid or RX_DIR / target_name(file_name(params, grid_key), a.ray, n_ant)
        rg_path = a.resource_grid or RX_DIR / file_name(params, RESOURCE_GRID_KEY)
        out_path = a.out or RX_DIR / target_name(file_name(params, OUTPUT_KEY), a.ray, n_ant)
        print(f"Input: {target_label(a.ray, n_ant)}")

        grid, noise_grid = load_payload_grid(grid_path, "Demodulate_Waveform_Rx.py" if a.unequalized
                                             else "Equalize_Payload_Grid.py")
        types, codes, rg_attrs = load_signal_types(rg_path)

        # The resource grid must have been built from the same parameters.
        checks = (("RefSignalSpacing", "ReferenceSymbolProfile"),
                  ("RefSymbolSpacing", "PacketProfile"),
                  ("PacketLength", "Packet_Len"))
        for param_key, grid_attr in checks:
            want = params.get(DEFINED_SECTION, {}).get(param_key)
            have = rg_attrs.get(grid_attr)
            if want is not None and have is not None and int(have) != int(want):
                raise ValueError(f"{a.params.name} has {param_key} {want} but {rg_path.name} was "
                                 f"built with {grid_attr} {int(have)} - run Initialize_Rx.py")

        DataSigs_QAM = get_datasigs_qam(grid, types, codes["Data"])
        NoiseVar = None if noise_grid is None else get_datasigs_qam(noise_grid, types, codes["Data"])

        n_data_sigs = params.get(CALCULATED_SECTION, {}).get("NumberOfDataSigs")
        if isinstance(n_data_sigs, int) and not isinstance(n_data_sigs, bool):
            if n_data_sigs != DataSigs_QAM.size * bits:
                raise ValueError(f"{DataSigs_QAM.size} Data signals x {bits} bits = "
                                 f"{DataSigs_QAM.size * bits}, but Calculated_Rx/NumberOfDataSigs is "
                                 f"{n_data_sigs} - run Initialize_Rx.py")
        else:
            print("Warning: Calculated_Rx/NumberOfDataSigs not usable - length not checked")

        mean_power = float(np.mean(np.abs(DataSigs_QAM) ** 2))
        out_path.parent.mkdir(parents=True, exist_ok=True)
        save_qam(out_path, DataSigs_QAM, {
            "Mod": mod, "BitsPerSymbol": bits, "NumberOfSymbols": DataSigs_QAM.size,
            "NumberOfDataSigs": DataSigs_QAM.size * bits, "MeanPower": mean_power,
            "GridFile": grid_path.name, "ResourceGridFile": rg_path.name,
            "HasNoiseVar": int(NoiseVar is not None),
            "Ray": a.ray if a.ray is not None else (0 if n_ant > 1 else 1),
        }, NoiseVar)

        counts = {name: int((types == c).sum()) for name, c in codes.items()}
        if a.unequalized:
            print("Note: --unequalized - reading the grid before the equalizer")
        print(f"Grid {grid.shape} from {grid_path.name}, signal types from {rg_path.name}: {counts}")
        print(f"DataSigs_QAM: {DataSigs_QAM.size} values ({mod}, {bits} bits/symbol -> "
              f"{DataSigs_QAM.size * bits} bits), mean power {mean_power:.4f}")
        if NoiseVar is not None:
            print(f"noise_var: one value per Data signal, median {np.median(NoiseVar):.3e} "
                  f"(SNR ~ {-10 * np.log10(np.median(NoiseVar)):.1f} dB)")
        else:
            print(f"No '{NOISE_DATASET}' in {grid_path.name} - soft demapping will estimate one noise value")
        print(f"Wrote {out_path}")
    except (FileNotFoundError, KeyError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
