#!/usr/bin/env python3
"""
Rate_Matching_Rx.py - Undo the Tx repetition rate matching on the receive side.

Steps:
  1. Read Parameters_Rx_payload.json from this script's Rx folder.
  2. Read Repetition (Defined_Rx) and load the code word named in
     File_Rx/TestVector_CodeWord_Rx from Tx/Encode (flexlink_paths.TX / "Encode").
     That file is written by Rate_Matching_Tx.py.
  3. Deconstruct with deconstruct_rate_match(sequence, repetition) from
     Deconstruct_Rate_Match.py in the Rate_Matching folder
     (flexlink_paths.RATE_MATCHING): split into Repetition + 1 chunks and
     average them element by element.
  4. Save the result to the .h5 file named in File_Rx/TestVector_InvCodeBlk_Rx
     in the Rx folder.

Checks:
  - error if the code word length isn't divisible by Repetition + 1
  - warning if the code word length differs from Calculated_Rx/CodeWordSize
  - warning if the result length differs from Calculated_Rx/CodeBlockSize

Usage:
    python Rate_Matching_Rx.py
    python Rate_Matching_Rx.py --params my.json --tx-dir /path/to/Tx --rate-match-dir /path/to/Rate_Matching

The output .h5 file contains:
    data   (input length / (Repetition + 1),) float64   averaged soft values, 0..1
    attrs  Repetition, InputLength, SourceFile, plus the input file's attributes
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import logging
import sys
from pathlib import Path
from typing import Any, Callable

import numpy as np

PARAMS_FILENAME = "Parameters_Rx_payload.json"
RX_DIR = Path(__file__).resolve().parent          # the Rx folder

# Sections in Parameters_Rx_payload.json
DEFINED_SECTION = "Defined_Rx"
CALCULATED_SECTION = "Calculated_Rx"
FILE_SECTION = "File_Rx"

IN_KEY = "TestVector_CodeWord_Rx"                 # file in Tx/Encode
TX_SUBDIR = "Encode"
OUT_KEY = "TestVector_InvCodeBlk_Rx"              # file written to the Rx folder

DECONSTRUCT_SCRIPT = "Deconstruct_Rate_Match.py"
DATASET = "data"                                  # dataset name used across the Tx/Rx pipeline

log = logging.getLogger("Rate_Matching_Rx")


# --------------------------------------------------------------------------- #
# Folder locations from flexlink_paths.py
# --------------------------------------------------------------------------- #
def _flexlink_dir(name: str) -> Path | None:
    try:
        import flexlink_paths
    except ImportError:
        log.warning("flexlink_paths.py not importable")
        return None
    value = getattr(flexlink_paths, name, None)
    return Path(value) if value is not None else None


def resolve_tx_dir(explicit: Path | None) -> Path:
    d = explicit or _flexlink_dir("TX")
    if d is None:
        raise FileNotFoundError("Tx directory unknown - pass --tx-dir")
    if not d.is_dir():
        raise FileNotFoundError(f"Tx directory not found: {d}")
    return d


def rate_match_dirs(explicit: Path | None) -> list[Path]:
    if explicit:
        if not explicit.is_dir():
            raise FileNotFoundError(f"Rate matching directory not found: {explicit}")
        return [explicit]
    dirs = []
    rm = _flexlink_dir("RATE_MATCHING")
    if rm is not None:
        dirs.append(rm)
    dirs.append(RX_DIR)
    return dirs


def load_deconstruct(dirs: list[Path]) -> Callable:
    """deconstruct_rate_match() from the first Deconstruct_Rate_Match.py found in `dirs`."""
    script = next((d / DECONSTRUCT_SCRIPT for d in dirs if (d / DECONSTRUCT_SCRIPT).is_file()), None)
    if script is None:
        raise FileNotFoundError(f"{DECONSTRUCT_SCRIPT} not found in: " + ", ".join(str(d) for d in dirs))
    spec = importlib.util.spec_from_file_location("Deconstruct_Rate_Match", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    func = getattr(module, "deconstruct_rate_match", None)
    if not callable(func):
        raise ValueError(f"{script} has no deconstruct_rate_match function")
    log.debug("Using %s", script)
    return func


# --------------------------------------------------------------------------- #
# JSON
# --------------------------------------------------------------------------- #
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


def get_value(params: dict[str, Any], section: str, key: str) -> Any:
    try:
        return params[section][key]
    except (KeyError, TypeError):
        raise ValueError(f"{section}/{key} not set in {PARAMS_FILENAME}") from None


def get_repetition(params: dict[str, Any]) -> int:
    value = get_value(params, DEFINED_SECTION, "Repetition")
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"Repetition is {value!r} in {PARAMS_FILENAME}, expected an integer >= 0")
    return value


# --------------------------------------------------------------------------- #
# .h5 files
# --------------------------------------------------------------------------- #
def load_code_word(path: Path) -> tuple[np.ndarray, dict[str, Any]]:
    import h5py
    if not path.is_file():
        raise FileNotFoundError(f"Code word file not found: {path} - run Rate_Matching_Tx.py first")
    with h5py.File(path, "r") as f:
        if DATASET not in f:
            raise ValueError(f"{path} has no '{DATASET}' dataset (found: {', '.join(f.keys()) or 'none'})")
        values = np.asarray(f[DATASET][()]).ravel()
        attrs = dict(f.attrs)
    if values.size == 0:
        raise ValueError(f"{path} contains an empty vector")
    return values, attrs


def save_result(path: Path, values: np.ndarray, attrs: dict[str, Any]) -> None:
    import h5py
    with h5py.File(path, "w") as f:
        f.create_dataset(DATASET, data=values)
        for k, v in attrs.items():
            f.attrs[k] = v


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Undo Tx repetition rate matching (Deconstruct_Rate_Match).")
    ap.add_argument("--params", type=Path, default=RX_DIR / PARAMS_FILENAME,
                    help=f"path to {PARAMS_FILENAME} (default: the Rx folder)")
    ap.add_argument("--tx-dir", type=Path, help="Tx directory holding the code word (default: flexlink_paths.TX)")
    ap.add_argument("--rate-match-dir", type=Path,
                    help=f"folder holding {DECONSTRUCT_SCRIPT} (default: flexlink_paths.RATE_MATCHING)")
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

        params = load_params(args.params)
        repetition = get_repetition(params)
        calculated = params.get(CALCULATED_SECTION, {})

        in_path = resolve_tx_dir(args.tx_dir) / TX_SUBDIR / get_value(params, FILE_SECTION, IN_KEY)
        code_word, in_attrs = load_code_word(in_path)
        log.info("Read %d values from %s", code_word.size, in_path)

        expected_in = calculated.get("CodeWordSize")
        if expected_in and code_word.size != expected_in:
            log.warning("Code word length %d differs from CodeWordSize %d in %s",
                        code_word.size, expected_in, PARAMS_FILENAME)

        deconstruct = load_deconstruct(rate_match_dirs(args.rate_match_dir))
        result = np.asarray(deconstruct(code_word, repetition), dtype=np.float64).ravel()
        log.info("Repetition %d: %d values -> %d chunks of %d averaged",
                 repetition, code_word.size, repetition + 1, result.size)

        expected_out = calculated.get("CodeBlockSize")
        if expected_out and result.size != expected_out:
            log.warning("Result length %d differs from CodeBlockSize %d in %s",
                        result.size, expected_out, PARAMS_FILENAME)

        out_path = RX_DIR / get_value(params, FILE_SECTION, OUT_KEY)
        out_attrs = dict(in_attrs)
        out_attrs.update({"Repetition": repetition, "InputLength": code_word.size, "SourceFile": in_path.name})
        save_result(out_path, result, out_attrs)
        log.info("Saved %d values to %s", result.size, out_path)

    except (FileNotFoundError, ValueError) as exc:
        log.error("%s", exc)
        return 1
    except Exception:
        log.exception("Rate de-matching failed")
        return 3

    return 0


if __name__ == "__main__":
    sys.exit(main())
