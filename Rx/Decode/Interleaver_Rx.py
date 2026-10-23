#!/usr/bin/env python3
"""
Interleaver_Rx.py - Deinterleave the Rx LDPC code block test vector.

Steps:
  1. Read Parameters_Rx_payload.json from this script's Rx folder.
  2. Read EncoderBlockSize (code block size CBS: 648, 1296 or 1944) from Defined_Rx.
  3. Load the interleaved code block values from the .h5 file named in
     File_Rx/TestVector_InvCodeBlk_Rx in the Rx folder (written by
     Rate_Matching_Rx.py). Its length must be a whole number of CBS-value blocks.
  4. Deinterleave each code block with DeInterleave_LDPC(MatrixToDeInterleave, CBS)
     from DeInterleave_LDPC.py in the Interleaver folder (flexlink_paths.INTERLEAVER,
     or its IndexGeneration / TestGeneration subfolders).
  5. Save the deinterleaved blocks, one after another, to the .h5 file named in
     File_Rx/TestVector_CodeBlk_Rx in the Rx folder.

Values are kept as they are (float64 soft values from rate de-matching): the
deinterleaver only reorders them.
TODO: Round to bits once the equalizer is developed.

Usage:
    python Interleaver_Rx.py
    python Interleaver_Rx.py --params my.json --interleaver-dir /path/to/Interleaver

The output .h5 file contains:
    data   (NumCodeBlocks * CBS,) float64   the deinterleaved code blocks, one after another
    attrs  EncoderBlockSize, NumCodeBlocks, SourceFile, plus the input file's attributes
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
FILE_SECTION = "File_Rx"

IN_KEY = "TestVector_InvCodeBlk_Rx"
OUT_KEY = "TestVector_CodeBlk_Rx"
DEINTERLEAVE_SCRIPT = "DeInterleave_LDPC.py"
SUPPORTED_CBS = (648, 1296, 1944)
DATASET = "data"

log = logging.getLogger("Interleaver_Rx")


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


def resolve_interleaver_dir(explicit: Path | None) -> Path:
    d = explicit or _flexlink_dir("INTERLEAVER")
    if d is None:
        raise FileNotFoundError("Interleaver directory unknown - pass --interleaver-dir")
    if not d.is_dir():
        raise FileNotFoundError(f"Interleaver directory not found: {d}")
    return d


def load_deinterleaver(interleaver_dir: Path) -> Callable:
    """DeInterleave_LDPC() from DeInterleave_LDPC.py in the Interleaver folder or its subfolders."""
    search = [interleaver_dir, interleaver_dir / "IndexGeneration", interleaver_dir / "TestGeneration"]
    script = next((d / DEINTERLEAVE_SCRIPT for d in search if (d / DEINTERLEAVE_SCRIPT).is_file()), None)
    if script is None:
        raise FileNotFoundError(f"{DEINTERLEAVE_SCRIPT} not found in: " + ", ".join(str(d) for d in search))
    if str(script.parent) not in sys.path:
        sys.path.insert(0, str(script.parent))
    spec = importlib.util.spec_from_file_location("DeInterleave_LDPC", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    func = getattr(module, "DeInterleave_LDPC", None)
    if not callable(func):
        raise ValueError(f"{script} has no DeInterleave_LDPC function")
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


def get_block_size(params: dict[str, Any]) -> int:
    value = get_value(params, DEFINED_SECTION, "EncoderBlockSize")
    if isinstance(value, bool) or not isinstance(value, int) or value not in SUPPORTED_CBS:
        raise ValueError(f"EncoderBlockSize is {value!r}, expected one of {SUPPORTED_CBS}")
    return value


# --------------------------------------------------------------------------- #
# .h5 files
# --------------------------------------------------------------------------- #
def load_values(path: Path) -> tuple[np.ndarray, dict[str, Any]]:
    import h5py
    if not path.is_file():
        raise FileNotFoundError(f"Interleaved code block file not found: {path} - run Rate_Matching_Rx.py first")
    with h5py.File(path, "r") as f:
        if DATASET not in f:
            raise ValueError(f"{path} has no '{DATASET}' dataset (found: {', '.join(f.keys()) or 'none'})")
        values = np.asarray(f[DATASET][()], dtype=np.float64).ravel()
        attrs = dict(f.attrs)
    if values.size == 0:
        raise ValueError(f"{path} contains an empty vector")
    return values, attrs


def save_values(path: Path, values: np.ndarray, attrs: dict[str, Any]) -> None:
    import h5py
    with h5py.File(path, "w") as f:
        f.create_dataset(DATASET, data=values)
        for k, v in attrs.items():
            f.attrs[k] = v


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Deinterleave the Rx LDPC code block test vector.")
    ap.add_argument("--params", type=Path, default=RX_DIR / PARAMS_FILENAME,
                    help=f"path to {PARAMS_FILENAME} (default: the Rx folder)")
    ap.add_argument("--interleaver-dir", type=Path,
                    help=f"folder holding {DEINTERLEAVE_SCRIPT} (default: flexlink_paths.INTERLEAVER)")
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

        interleaver_dir = resolve_interleaver_dir(args.interleaver_dir)
        params = load_params(args.params)
        cbs = get_block_size(params)

        in_path = RX_DIR / get_value(params, FILE_SECTION, IN_KEY)
        values, in_attrs = load_values(in_path)
        log.info("Read %d values from %s", values.size, in_path)
        if values.size % cbs:
            raise ValueError(f"{in_path.name} has {values.size} values, not a multiple of "
                             f"EncoderBlockSize {cbs}")

        deinterleave = load_deinterleaver(interleaver_dir)
        m = values.size // cbs
        blocks = values.reshape(m, cbs)
        code_blk = np.asarray(deinterleave(blocks, cbs), dtype=np.float64)
        if code_blk.shape != (m, cbs):
            raise ValueError(f"DeInterleave_LDPC returned shape {code_blk.shape}, expected ({m}, {cbs})")
        # A permutation keeps exactly the same values in every block, only reordered.
        if not np.array_equal(np.sort(code_blk, axis=1), np.sort(blocks, axis=1)):
            raise ValueError("Deinterleaved blocks don't hold the same values as the input - check the interleaver")
        code_blk = code_blk.ravel()
        log.info("Deinterleaved %d code block(s) of %d values", m, cbs)

        # TODO: Round to bits once the equalizer is developed. Until then the
        #       soft values from Rate_Matching_Rx.py are passed through as float64.

        out_path = RX_DIR / get_value(params, FILE_SECTION, OUT_KEY)
        out_attrs = dict(in_attrs)
        out_attrs.update({"EncoderBlockSize": cbs, "NumCodeBlocks": m, "SourceFile": in_path.name})
        save_values(out_path, code_blk, out_attrs)
        log.info("Deinterleaved code block saved to %s", out_path)

    except (FileNotFoundError, ValueError) as exc:
        log.error("%s", exc)
        return 1
    except Exception:
        log.exception("Deinterleaving failed")
        return 3

    return 0


if __name__ == "__main__":
    sys.exit(main())
