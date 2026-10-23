#!/usr/bin/env python3
"""
Rate_Matching_Tx.py - Repetition rate matching of the interleaved Tx code block.

Steps:
  1. Read Parameters_Tx_payload.json from the Tx directory (flexlink_paths.TX).
  2. Read Repetition (number of extra copies, integer >= 0) from the JSON.
  3. Load the interleaved code block bits from
     Tx/Encode/Sequence_InvCodeBlk_Tx.h5 (written by Interleaver_Tx.py).
  4. Rate match with rate_match(sequence, repetition) from Rate_Match.py in the
     Rate_Matching folder (flexlink_paths.RATE_MATCHING, its subfolders, or the
     project root): the sequence followed by Repetition more copies of it.
  5. Save the result to Tx/Encode/Sequence_CodeWord_Tx.h5 (read by Rx/Rate_Matching_Rx.py). The file
     names are fixed in this script; nothing is written to the JSON.

Usage:
    python Rate_Matching_Tx.py
    python Rate_Matching_Tx.py --params my.json --tx-dir /path/to/Tx --rate-match-dir /path/to/Rate_Matching

The output .h5 file contains:
    data   ((Repetition + 1) * input length,) uint8   the code word
    attrs  Repetition, InputLength, SourceFile, plus the input file's attributes
           (EncoderBlockSize, EncoderCodeRate, NumCodeBlocks, seed, ...)
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

PARAMS_FILENAME = "Parameters_Tx_payload.json"
ENCODE_SUBDIR = "Encode"     # the Sequence_*.h5 test files live in Tx/Encode
IN_FILE = "Sequence_InvCodeBlk_Tx.h5"      # written by Interleaver_Tx.py
OUT_FILE = "Sequence_CodeWord_Tx.h5"     # end of the Tx test chain
RATE_MATCH_SCRIPT = "Rate_Match.py"

log = logging.getLogger("Rate_Matching_Tx")


# --------------------------------------------------------------------------- #
# Folder locations from flexlink_paths.py
# --------------------------------------------------------------------------- #
def _import_flexlink_paths():
    try:
        import flexlink_paths
        return flexlink_paths
    except ImportError:
        pass
    # Not on sys.path: look in this script's folder and its parents.
    for d in Path(__file__).resolve().parents:
        if (d / "flexlink_paths.py").is_file():
            sys.path.insert(0, str(d))
            import flexlink_paths
            return flexlink_paths
    return None


def _flexlink_dir(name: str) -> Path | None:
    flexlink_paths = _import_flexlink_paths()
    if flexlink_paths is None:
        log.warning("flexlink_paths.py not found (in this script's folder, its parents, or PYTHONPATH)")
        return None
    value = getattr(flexlink_paths, name, None)
    return Path(value) if value is not None else None


def resolve_dir(explicit: Path | None, flexlink_name: str, what: str) -> Path:
    d = explicit or _flexlink_dir(flexlink_name)
    if d is None:
        raise FileNotFoundError(f"{what} directory unknown - pass it on the command line")
    if not d.is_dir():
        raise FileNotFoundError(f"{what} directory not found: {d}")
    return d


# --------------------------------------------------------------------------- #
# JSON helpers
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


def get_repetition(params: dict[str, Any]) -> int:
    parent, value = _find_key(params, "Repetition")
    if parent is None:
        raise ValueError(f"Repetition not found in {PARAMS_FILENAME}")
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"Repetition is {value!r} in {PARAMS_FILENAME}, expected an integer >= 0")
    return value


# --------------------------------------------------------------------------- #
# Rate matching module
# --------------------------------------------------------------------------- #
def load_rate_match(dirs: list[Path]) -> Callable:
    """rate_match() from the first Rate_Match.py found in `dirs`."""
    script = next((d / RATE_MATCH_SCRIPT for d in dirs if (d / RATE_MATCH_SCRIPT).is_file()), None)
    if script is None:
        raise FileNotFoundError(f"{RATE_MATCH_SCRIPT} not found in: " + ", ".join(str(d) for d in dirs))
    if str(script.parent) not in sys.path:
        sys.path.insert(0, str(script.parent))
    spec = importlib.util.spec_from_file_location("Rate_Match", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    func = getattr(module, "rate_match", None)
    if not callable(func):
        raise ValueError(f"{script} has no rate_match function")
    log.debug("Using %s", script)
    return func


def rate_match_dirs(explicit: Path | None) -> list[Path]:
    if explicit:
        if not explicit.is_dir():
            raise FileNotFoundError(f"Rate matching directory not found: {explicit}")
        return [explicit]
    dirs = []
    rm = _flexlink_dir("RATE_MATCHING")
    if rm is not None:
        dirs += [rm, rm / "CreateOriginalSequence", rm / "TestGeneration"]
    root = _flexlink_dir("ROOT")
    if root is not None:
        dirs.append(root)
    if not dirs:
        raise FileNotFoundError("Rate matching directory unknown - pass --rate-match-dir")
    return dirs


# --------------------------------------------------------------------------- #
# .h5 files
# --------------------------------------------------------------------------- #
def load_bits(path: Path) -> tuple[np.ndarray, dict[str, Any]]:
    import h5py
    if not path.is_file():
        raise FileNotFoundError(f"Interleaved code block file not found: {path} - run Interleaver_Tx.py first")
    with h5py.File(path, "r") as f:
        if "data" not in f:
            raise ValueError(f"{path} has no 'data' dataset")
        bits = np.asarray(f["data"][()]).ravel().astype(np.uint8)
        attrs = dict(f.attrs)
    if bits.size == 0:
        raise ValueError(f"{path} contains an empty vector")
    if not np.isin(bits, (0, 1)).all():
        raise ValueError(f"{path} contains values other than 0 and 1")
    return bits, attrs


def save_bits(path: Path, bits: np.ndarray, attrs: dict[str, Any]) -> None:
    import h5py
    with h5py.File(path, "w") as f:
        f.create_dataset("data", data=bits)
        for k, v in attrs.items():
            f.attrs[k] = v


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Repetition rate matching of the interleaved Tx code block.")
    ap.add_argument("--params", type=Path, help=f"path to {PARAMS_FILENAME} (default: Tx directory)")
    ap.add_argument("--tx-dir", type=Path, help="Tx directory (default: flexlink_paths.TX)")
    ap.add_argument("--rate-match-dir", type=Path,
                    help="folder holding Rate_Match.py (default: flexlink_paths.RATE_MATCHING, then project root)")
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

        tx_dir = resolve_dir(args.tx_dir, "TX", "Tx")
        params_file = args.params or tx_dir / PARAMS_FILENAME
        params = load_params(params_file)
        repetition = get_repetition(params)

        in_path = tx_dir / ENCODE_SUBDIR / IN_FILE
        bits, in_attrs = load_bits(in_path)
        log.info("Read %d bits from %s", bits.size, in_path)

        rate_match = load_rate_match(rate_match_dirs(args.rate_match_dir))
        code_word = np.asarray(rate_match(bits, repetition)).ravel().astype(np.uint8)
        if code_word.size != bits.size * (repetition + 1):
            raise ValueError(f"rate_match returned {code_word.size} bits, expected "
                             f"{bits.size} x {repetition + 1} = {bits.size * (repetition + 1)}")
        log.info("Repetition %d: %d -> %d bits", repetition, bits.size, code_word.size)

        out_path = tx_dir / ENCODE_SUBDIR / OUT_FILE
        out_attrs = dict(in_attrs)
        out_attrs.update({"Repetition": repetition, "InputLength": bits.size, "SourceFile": in_path.name})
        save_bits(out_path, code_word, out_attrs)
        log.info("Code word saved to %s", out_path)

    except (FileNotFoundError, ValueError) as exc:
        log.error("%s", exc)
        return 1
    except Exception:
        log.exception("Rate matching failed")
        return 3

    return 0


if __name__ == "__main__":
    sys.exit(main())
