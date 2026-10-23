#!/usr/bin/env python3
"""
Construct_Datasigs_QAM.py - Map the scrambled Tx bitstream to QAM data signals.

Steps:
  1. Read Parameters_Tx_payload.json from the Tx directory (flexlink_paths.TX).
  2. Read Mod (BPSK, QPSK, 16QAM or 64QAM) from the JSON.
  3. Load the bitstream from dataset 'data' of the file named in
     File/Bitstream_Encoded_Tx (e.g. Bitstream_Encoded_Tx.h5) in the Tx
     directory (the scrambled Sequence written by Construct_Encoded_Bitstream_Tx.py).
  4. Take the bits in groups of BitsPerSymbol (1, 2, 4 or 6, first bit = MSB) and
     map each group to its constellation value with qam_map() from
     QAM_Mapper/QAM_Mapper.py (IEEE 802.11 Gray mapping, unit average power).
  5. Check len(DataSigs_QAM) == NumberOfDataSigs / BitsPerSymbol.
  6. Save DataSigs_QAM to the file named in File/Sequence_DataSigs_QAM
     (e.g. Sequence_DataSigs_QAM.h5) in the Tx directory (or --out).

Usage:
    python Construct_Datasigs_QAM.py
    python Construct_Datasigs_QAM.py --input other.h5 --dataset name --out My_QAM.h5

The output .h5 file contains:
    data   (NumberOfDataSigs / BitsPerSymbol,) complex128   DataSigs_QAM
    attrs  Mod, BitsPerSymbol, NumberOfDataSigs, NumberOfSymbols, SourceFile
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import logging
import sys
from pathlib import Path
from types import ModuleType
from typing import Any

import numpy as np

# Locate flexlink_paths.py by walking up from this script's folder.
for _parent in Path(__file__).resolve().parents:
    if (_parent / "flexlink_paths.py").is_file():
        sys.path.insert(0, str(_parent))
        break
from flexlink_paths import TX, QAM_MAPPER  # noqa: E402

PARAMS_FILENAME = "Parameters_Tx_payload.json"
INPUT_FILE_KEY = "Bitstream_Encoded_Tx"        # File/<key> in the JSON = input name (Tx directory)
DEFAULT_DATASET = "data"
OUTPUT_FILE_KEY = "Sequence_DataSigs_QAM"      # File/<key> in the JSON = output name (Tx directory)
MAPPER_SCRIPT = "QAM_Mapper.py"

log = logging.getLogger("Construct_Datasigs_QAM")


# --------------------------------------------------------------------------- #
# JSON
# --------------------------------------------------------------------------- #
def _find_key(obj: Any, key: str) -> tuple[dict | None, Any]:
    if isinstance(obj, dict):
        if key in obj:
            return obj, obj[key]
        for v in obj.values():
            parent, val = _find_key(v, key)
            if parent is not None:
                return parent, val
    return None, None


def load_params(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"Parameter file not found: {path}")
    try:
        with path.open("r", encoding="utf-8") as fh:
            params = json.load(fh)
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON in {path}: line {exc.lineno}, col {exc.colno}: {exc.msg}") from exc
    if not isinstance(params, dict):
        raise ValueError(f"{path} must contain a JSON object at top level")
    return params


def get_param(params: dict[str, Any], key: str) -> Any:
    parent, value = _find_key(params, key)
    if parent is None:
        raise ValueError(f"{key} not found in {PARAMS_FILENAME}")
    return value


def file_name(params: dict[str, Any], key: str) -> str:
    """File name stored under File/<key> in the JSON."""
    name = params.get("File", {}).get(key) if isinstance(params.get("File"), dict) else None
    if not isinstance(name, str) or not name.strip():
        raise ValueError(f"File/{key} not found in {PARAMS_FILENAME}")
    return name.strip()


# --------------------------------------------------------------------------- #
# QAM mapper (QAM_Mapper.py, loaded by file path)
# --------------------------------------------------------------------------- #
def load_mapper() -> ModuleType:
    script = QAM_MAPPER / MAPPER_SCRIPT
    if not script.is_file():
        raise FileNotFoundError(f"{MAPPER_SCRIPT} not found in {QAM_MAPPER}")
    spec = importlib.util.spec_from_file_location("QAM_Mapper", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if not callable(getattr(module, "qam_map", None)):
        raise ValueError(f"{script} has no qam_map function")
    return module


def bits_per_symbol(mapper: ModuleType, mod: str) -> tuple[str, int]:
    """Normalise the Mod name the same way qam_map does; return (name, bits per symbol)."""
    name = str(mod).upper().replace("-", "").replace("_", "").replace(" ", "")
    table = getattr(mapper, "_MODULATIONS", {})
    if name not in table:
        raise ValueError(f"Mod '{mod}' is not supported by {MAPPER_SCRIPT} "
                         f"(choose from: {', '.join(table)})")
    return name, int(table[name][0])


# --------------------------------------------------------------------------- #
# .h5 files
# --------------------------------------------------------------------------- #
def load_bits(path: Path, dataset: str) -> np.ndarray:
    import h5py
    if not path.is_file():
        raise FileNotFoundError(f"Input file not found: {path} - "
                                "run Construct_Encoded_Bitstream_Tx.py first")
    with h5py.File(path, "r") as f:
        if dataset not in f:
            raise ValueError(f"{path.name} has no dataset '{dataset}' - pass --dataset")
        bits = np.asarray(f[dataset][()]).ravel().astype(np.uint8)
    if bits.size == 0:
        raise ValueError(f"{path} contains an empty vector")
    if not np.isin(bits, (0, 1)).all():
        raise ValueError(f"{path} contains values other than 0 and 1")
    return bits


def save_qam(path: Path, symbols: np.ndarray, attrs: dict[str, Any]) -> None:
    import h5py
    with h5py.File(path, "w") as f:
        f.create_dataset("data", data=symbols)
        for k, v in attrs.items():
            f.attrs[k] = v


N_SHOW = 10   # number of elements printed in the info output


def log_first_elements(bits: np.ndarray, symbols: np.ndarray, nbpsc: int, n: int = N_SHOW) -> None:
    """Log the first n symbols next to the bit groups they were mapped from."""
    n = min(n, symbols.size)
    log.info("First %d elements of DataSigs_QAM:", n)
    log.info("  %5s  %-*s  %s", "index", max(nbpsc, 4), "bits", "symbol")
    for i in range(n):
        group = "".join(str(b) for b in bits[i * nbpsc:(i + 1) * nbpsc])
        s = symbols[i]
        log.info("  %5d  %-*s  %+.4f %+.4fj", i, max(nbpsc, 4), group, s.real, s.imag)


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def construct_datasigs_qam(input_path: Path, dataset: str = DEFAULT_DATASET,
                           params_file: Path | None = None) -> tuple[np.ndarray, str, int]:
    """Return (DataSigs_QAM, Mod, BitsPerSymbol)."""
    params = load_params(params_file or TX / PARAMS_FILENAME)
    mapper = load_mapper()
    mod, nbpsc = bits_per_symbol(mapper, get_param(params, "Mod"))

    bits = load_bits(input_path, dataset)
    log.info("Read %d bits from %s", bits.size, input_path)
    if bits.size % nbpsc:
        raise ValueError(f"{bits.size} bits is not a multiple of {nbpsc} bits per symbol for {mod}")

    DataSigs_QAM = np.asarray(mapper.qam_map(bits, mod), dtype=np.complex128).ravel()
    log.info("%s: %d bits -> %d symbols (%d bits/symbol)", mod, bits.size, DataSigs_QAM.size, nbpsc)
    log_first_elements(bits, DataSigs_QAM, nbpsc)

    n_data_sigs = get_param(params, "NumberOfDataSigs")
    if isinstance(n_data_sigs, int) and not isinstance(n_data_sigs, bool):
        expected = n_data_sigs // nbpsc
        if n_data_sigs % nbpsc or DataSigs_QAM.size != expected:
            raise ValueError(f"len(DataSigs_QAM) = {DataSigs_QAM.size} but NumberOfDataSigs "
                             f"{n_data_sigs} / {nbpsc} = {n_data_sigs / nbpsc:g}")
        log.info("len(DataSigs_QAM) = %d = NumberOfDataSigs / %d", DataSigs_QAM.size, nbpsc)
    else:
        log.warning("NumberOfDataSigs not usable in %s - length not checked", PARAMS_FILENAME)

    return DataSigs_QAM, mod, nbpsc


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Map the scrambled Tx bitstream to QAM data signals.")
    ap.add_argument("--input", type=Path,
                    help=f"bitstream .h5; a relative name is looked up in the Tx directory "
                         f"(default: File/{INPUT_FILE_KEY} from {PARAMS_FILENAME})")
    ap.add_argument("--dataset", default=DEFAULT_DATASET, help=f"dataset in the input file (default: {DEFAULT_DATASET})")
    ap.add_argument("--params", type=Path, help=f"path to {PARAMS_FILENAME} (default: Tx directory)")
    ap.add_argument("--out", type=Path,
                    help=f"output .h5; a relative name is put in the Tx directory "
                         f"(default: File/{OUTPUT_FILE_KEY} from {PARAMS_FILENAME})")
    ap.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    return ap.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",
    )
    try:
        if importlib.util.find_spec("h5py") is None:
            raise ValueError("Reading/writing .h5 files needs h5py - install it with: pip install h5py")

        params = load_params(args.params or TX / PARAMS_FILENAME)
        in_name = args.input or Path(file_name(params, INPUT_FILE_KEY))
        in_path = in_name if in_name.is_absolute() else TX / in_name
        out_name = args.out or Path(file_name(params, OUTPUT_FILE_KEY))
        out_path = out_name if out_name.is_absolute() else TX / out_name

        DataSigs_QAM, mod, nbpsc = construct_datasigs_qam(in_path, args.dataset, args.params)

        save_qam(out_path, DataSigs_QAM, {
            "Mod": mod, "BitsPerSymbol": nbpsc, "NumberOfSymbols": DataSigs_QAM.size,
            "NumberOfDataSigs": DataSigs_QAM.size * nbpsc, "SourceFile": in_path.name,
        })
        log.info("DataSigs_QAM saved to %s", out_path)
    except (FileNotFoundError, ValueError) as exc:
        log.error("%s", exc)
        return 1
    except Exception:
        log.exception("QAM mapping failed")
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
