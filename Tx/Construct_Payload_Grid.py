"""Construct_Payload_Grid.py  (lives in Flexlink_py/Tx)

Fills the resource grid with QAM values for one transmit port.

Inputs
------
Tx/<File.Grid_Resource_Tx>
                                     (e.g. Grid_Resource_Tx.h5) 'signal_types' (Packet_Len, 841) int8 says what
                                     each signal is: 10=Ref_P0, 20=Ref_P1, 30=DC,
                                     40=Data (codes are read from the file's attrs).
Tx/Parameters_Tx_payload.json        Port, RefSignalSpacing, Mod (and PacketLength,
                                     RefSymbolSpacing, checked against the grid).
Tx/<File.Sequence_DataSigs_QAM>      'data': QAM values for the Data signals
                                     (e.g. Sequence_DataSigs_QAM.h5).
ReferenceSignals/Ref_Pz_X_Y.h5       'data': prebuilt values for ONE Symbol_Ref,
                                     z = Port, X = RefSignalSpacing (3, 6, 12, 24),
                                     Y = bits per symbol of Mod (1, 2, 4, 6).

Rules
-----
* Data signals   <- DataSigs_QAM in order: symbol 0 k=0..840, then symbol 1, ...
* Ref_Pz signals <- Ref_Pz_X_Y, the same values repeated in every Symbol_Ref,
                    in increasing k.
* The other port's Ref signals <- 0  (Port 0: Ref_P1 = 0, Port 1: Ref_P0 = 0).
* DC signals     <- 0.

Output
------
Tx/<File.Grid_Payload_Tx>   Name taken from Parameters_Tx_payload.json
                            (e.g. Grid_Payload_Tx.h5). Same layout as
                            the resource grid:
    data          (Packet_Len, 841) complex   payload values
    signal_types, symbol_types, k, Tone_i     copied from the resource grid
    attrs: the grid's attrs plus Port, RefSignalSpacing, Mod, BitsPerSymbol and
           the names of the files used.

Usage
-----
    python Tx/Construct_Payload_Grid.py
    python Tx/Construct_Payload_Grid.py --port 1
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import h5py
import numpy as np

# Make flexlink_paths importable when the script is run from anywhere.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from flexlink_paths import ROOT, TX  # noqa: E402

try:
    from flexlink_paths import REFERENCE_SIGNALS  # noqa: E402
except ImportError:
    REFERENCE_SIGNALS = ROOT / "ReferenceSignals"

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
GRID_DIR = TX                      # grid name comes from File.Grid_Resource_Tx
PARAMS_FILE = TX / "Parameters_Tx_payload.json"
DATA_DIR = TX                      # data name comes from File.Sequence_DataSigs_QAM
OUTPUT_DIR = TX                   # output name comes from File.Grid_Payload_Tx

GRID_DATASET = "signal_types"     # (Packet_Len, 841) signal-type codes
VALUE_DATASET = "data"            # values in the QAM data file and Ref_Pz_X_Y.h5
COPY_DATASETS = ("signal_types", "symbol_types", "k", "Tone_i")

# Used only if the resource grid file has no code attributes
# (same as SignalType in Construct_Resource_Grid.py).
DEFAULT_CODES = {"Ref_P0": 10, "Ref_P1": 20, "DC": 30, "Data": 40}

VALID_PORTS = (0, 1)
VALID_SPACINGS = (3, 6, 12, 24)
MOD_BITS = {"BPSK": 1, "QPSK": 2, "16QAM": 4, "64QAM": 6}


# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------
def get_param(params: dict, key: str):
    """Find key anywhere in the (nested) parameter dict."""
    if key in params:
        return params[key]
    for v in params.values():
        if isinstance(v, dict):
            try:
                return get_param(v, key)
            except KeyError:
                pass
    raise KeyError(f"'{key}' not found in parameter file")


def file_name_from_params(params: dict, key: str) -> str:
    """File name stored in the parameter file's File section."""
    try:
        name = params["File"][key]
    except (KeyError, TypeError):
        raise KeyError(f"File.{key} not found in parameter file") from None
    if not isinstance(name, str) or not name.strip():
        raise ValueError(f"File.{key} must be a file name, got {name!r}")
    return name.strip()


def output_file_from_params(params: dict) -> Path:
    """Tx/<File.Grid_Payload_Tx>."""
    return OUTPUT_DIR / file_name_from_params(params, "Grid_Payload_Tx")


def grid_file_from_params(params: dict) -> Path:
    """Tx/<File.Grid_Resource_Tx>."""
    return GRID_DIR / file_name_from_params(params, "Grid_Resource_Tx")


def data_file_from_params(params: dict) -> Path:
    """Tx/<File.Sequence_DataSigs_QAM>."""
    return DATA_DIR / file_name_from_params(params, "Sequence_DataSigs_QAM")


def check_port(value) -> int:
    if int(value) not in VALID_PORTS or float(value) != int(value):
        raise ValueError(f"Port {value!r} must be one of {VALID_PORTS}")
    return int(value)


def check_spacing(value) -> int:
    f = float(value)
    if not f.is_integer() or int(f) not in VALID_SPACINGS:
        raise ValueError(f"RefSignalSpacing {value!r} must be one of {VALID_SPACINGS}")
    return int(f)


def mod_to_bits(value) -> int:
    """'QPSK' -> 2 etc.; a number 1, 2, 4 or 6 is accepted as is."""
    key = str(value).strip().upper().replace("-", "").replace(" ", "")
    if key in MOD_BITS:
        return MOD_BITS[key]
    try:
        f = float(key)
    except ValueError:
        f = None
    if f is not None and f.is_integer() and int(f) in MOD_BITS.values():
        return int(f)
    raise ValueError(f"Mod {value!r} must be one of {list(MOD_BITS)} or 1, 2, 4, 6")


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


def load_values(path: Path) -> np.ndarray:
    if not path.is_file():
        raise FileNotFoundError(f"File not found: {path}")
    with h5py.File(path, "r") as f:
        if VALUE_DATASET not in f:
            raise KeyError(f"{path.name} has no '{VALUE_DATASET}' dataset (has {list(f)})")
        return to_complex(np.asarray(f[VALUE_DATASET][()])).ravel()


def load_grid(path: Path):
    """Return (signal_types, codes, grid attrs, extra datasets to copy)."""
    if not path.is_file():
        raise FileNotFoundError(f"File not found: {path}")
    with h5py.File(path, "r") as f:
        if GRID_DATASET not in f:
            raise KeyError(f"{path.name} has no '{GRID_DATASET}' dataset (has {list(f)})")
        types = np.asarray(f[GRID_DATASET][()])
        attrs = dict(f.attrs)
        extra = {name: f[name][()] for name in COPY_DATASETS if name in f}
    if types.ndim != 2:
        raise ValueError(f"'{GRID_DATASET}' must be 2-D (symbols x signals), got {types.shape}")
    codes = {name: int(attrs.get(name, default)) for name, default in DEFAULT_CODES.items()}
    return types, codes, attrs, extra


# ---------------------------------------------------------------------------
# Filling
# ---------------------------------------------------------------------------
def fill_data(payload: np.ndarray, mask: np.ndarray, values: np.ndarray) -> None:
    """Data in order: symbol 0 k=0..840, symbol 1 k=0..840, ..."""
    n = int(mask.sum())
    if values.size != n:
        raise ValueError(f"Grid has {n} Data signals but the QAM data file has {values.size} "
                         "values. Check that it and the resource grid were built "
                         "with the same parameters.")
    payload[mask] = values          # boolean indexing on (symbol, k) is row-major


def fill_ref(payload: np.ndarray, mask: np.ndarray, ref: np.ndarray, label: str) -> list[int]:
    """Write the same ref vector into every symbol that has `label` signals."""
    symbols = [n for n in range(mask.shape[0]) if mask[n].any()]
    if not symbols:
        raise ValueError(f"Resource grid has no {label} signals")
    for n in symbols:
        k = np.flatnonzero(mask[n])
        if k.size != ref.size:
            raise ValueError(f"Symbol {n} has {k.size} {label} signals but the Ref file has "
                             f"{ref.size} values")
        payload[n, k] = ref
    return symbols


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def construct_payload_grid(grid_path: Path | None = None, params_path: Path = PARAMS_FILE,
                           data_path: Path | None = None, output_path: Path | None = None,
                           port: int | None = None) -> np.ndarray:
    with open(params_path, "r") as fh:
        params = json.load(fh)
    if grid_path is None:
        grid_path = grid_file_from_params(params)
    if data_path is None:
        data_path = data_file_from_params(params)
    if output_path is None:
        output_path = output_file_from_params(params)
    port = check_port(port if port is not None else get_param(params, "Port"))
    spacing = check_spacing(get_param(params, "RefSignalSpacing"))
    mod = get_param(params, "Mod")
    bits = mod_to_bits(mod)

    types, codes, grid_attrs, extra = load_grid(grid_path)

    # The grid must have been built from the same parameters.
    checks = (("RefSignalSpacing", "ReferenceSymbolProfile"),
              ("RefSymbolSpacing", "PacketProfile"),
              ("PacketLength", "Packet_Len"))
    for param_key, grid_attr in checks:
        try:
            want = get_param(params, param_key)
        except KeyError:
            continue
        have = grid_attrs.get(grid_attr)
        if have is not None and int(have) != int(want):
            raise ValueError(f"{params_path.name} has {param_key} {want} but {grid_path.name} "
                             f"was built with {grid_attr} {int(have)}. Rebuild the grid or "
                             "check the parameter file.")

    active, inactive = ("Ref_P0", "Ref_P1") if port == 0 else ("Ref_P1", "Ref_P0")
    ref_path = REFERENCE_SIGNALS / f"Ref_P{port}_{spacing}_{bits}.h5"

    payload = np.zeros(types.shape, dtype=np.complex128)   # DC and inactive Ref stay 0
    fill_data(payload, types == codes["Data"], load_values(data_path))
    ref_symbols = fill_ref(payload, types == codes[active], load_values(ref_path), active)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(output_path, "w") as f:
        f.create_dataset("data", data=payload)
        for name, arr in extra.items():
            f.create_dataset(name, data=arr)
        for k, v in grid_attrs.items():
            f.attrs[k] = v
        f.attrs["Port"] = port
        f.attrs["RefSignalSpacing"] = spacing
        f.attrs["Mod"] = str(mod)
        f.attrs["BitsPerSymbol"] = bits
        f.attrs["ResourceGridFile"] = grid_path.name
        f.attrs["DataFile"] = data_path.name
        f.attrs["RefFile"] = ref_path.name

    counts = {name: int((types == c).sum()) for name, c in codes.items()}
    print(f"Port {port}, RefSignalSpacing {spacing}, Mod {mod} ({bits} bits/symbol)")
    print(f"Grid {types.shape}: {counts}")
    print(f"Data   <- {data_path.name}")
    print(f"{active} <- {ref_path.name}, repeated on {len(ref_symbols)} Symbol_Ref "
          f"{ref_symbols}")
    print(f"{inactive} and DC set to 0")
    print(f"Wrote {output_path}")
    return payload


def main() -> None:
    ap = argparse.ArgumentParser(description="Fill the Tx resource grid with QAM values.")
    ap.add_argument("--grid", type=Path,
                    help="Override the resource grid (default: Tx/<File.Grid_Resource_Tx>)")
    ap.add_argument("--params", type=Path, default=PARAMS_FILE)
    ap.add_argument("--data", type=Path,
                    help="Override the QAM data file (default: Tx/<File.Sequence_DataSigs_QAM>)")
    ap.add_argument("--output", type=Path,
                    help="Override the output file (default: Tx/<File.Grid_Payload_Tx>)")
    ap.add_argument("--port", type=int, help="Override Port from the parameter file")
    a = ap.parse_args()
    construct_payload_grid(a.grid, a.params, a.data, a.output, a.port)


if __name__ == "__main__":
    main()
