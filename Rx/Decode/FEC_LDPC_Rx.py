#!/usr/bin/env python3
"""
FEC_LDPC_Rx.py - LDPC-decode the Rx code block test vector into the data block.

Steps:
  1. Read Parameters_Rx_payload.json from this script's Rx folder.
  2. Read EncoderBlockSize (codeword length N) and EncoderCodeRate (string, e.g.
     "5/6") from Defined_Rx; K = N x rate message bits per code block.
  3. Load the code block values from the .h5 file named in File_Rx/TestVector_CodeBlk_Rx
     (written by Interleaver_Rx.py). Its length must be a whole number of N-value blocks.
  4. Map the values (0..1, 0 = bit 0, 1 = bit 1) to decoder beliefs 2*x - 1
     (negative = bit 0, positive = bit 1, 0 = no information).
  5. Get H with LDPC_Get_H('<N>_<rate digits>'), e.g. '1296_56' for N=1296, rate 5/6,
     and decode with LDPC_Decode(CodeBlk, H), both from flexlink_paths.LDPC_ENCODER.
  6. Save the decoded message bits (first K bits of each codeword), one block after
     another, to the .h5 file named in File_Rx/TestVector_DataBlk_Rx in the Rx folder.

Usage:
    python FEC_LDPC_Rx.py
    python FEC_LDPC_Rx.py --iterations 30
    python FEC_LDPC_Rx.py --params my.json --ldpc-dir /path/to/LDPC_Encoder

The output .h5 file contains:
    data   (NumCodeBlocks * K,) uint8   the decoded data blocks, one after another
    attrs  EncoderBlockSize, EncoderCodeRate, K, N, NumCodeBlocks, DataBlockSize, H_Type,
           Iterations, CorrectedBits, SourceFile, plus the input file's attributes
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import logging
import sys
from fractions import Fraction
from pathlib import Path
from types import ModuleType
from typing import Any, Callable

import numpy as np

PARAMS_FILENAME = "Parameters_Rx_payload.json"
RX_DIR = Path(__file__).resolve().parent          # the Rx folder

# Sections in Parameters_Rx_payload.json
DEFINED_SECTION = "Defined_Rx"
CALCULATED_SECTION = "Calculated_Rx"
FILE_SECTION = "File_Rx"

IN_KEY = "TestVector_CodeBlk_Rx"
OUT_KEY = "TestVector_DataBlk_Rx"
DECODE_SCRIPT = "LDPC_Decode.py"
GET_H_SCRIPT = "LDPC_Get_H.py"
DATASET = "data"
DEFAULT_ITERATIONS = 15                           # LDPC_Decode default (as in MATLAB)

log = logging.getLogger("FEC_LDPC_Rx")


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


def resolve_ldpc_dir(explicit: Path | None) -> Path:
    d = explicit or _flexlink_dir("LDPC_ENCODER")
    if d is None:
        raise FileNotFoundError("LDPC_Encoder directory unknown - pass --ldpc-dir")
    if not d.is_dir():
        raise FileNotFoundError(f"LDPC_Encoder directory not found: {d}")
    return d


# --------------------------------------------------------------------------- #
# LDPC decoder modules (LDPC_Get_H.py and LDPC_Decode.py, loaded by file path)
# --------------------------------------------------------------------------- #
def _load_module(path: Path, module_name: str) -> ModuleType:
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _find_script(name: str, dirs: list[Path]) -> Path:
    for d in dirs:
        if (d / name).is_file():
            return d / name
    raise FileNotFoundError(f"{name} not found in: " + ", ".join(str(d) for d in dirs))


def load_decoder(ldpc_dir: Path) -> tuple[Callable, Callable]:
    """Returns (LDPC_Get_H, LDPC_Decode) from the LDPC_Encoder folder."""
    search = [ldpc_dir, ldpc_dir / "MatrixGeneration_H"]
    decode_path = _find_script(DECODE_SCRIPT, search)
    get_h_path = _find_script(GET_H_SCRIPT, search)

    # Let the decoder files import their own helpers.
    for d in dict.fromkeys([get_h_path.parent, decode_path.parent]):
        if str(d) not in sys.path:
            sys.path.insert(0, str(d))

    get_h = getattr(_load_module(get_h_path, "_flexlink_LDPC_Get_H"), "LDPC_Get_H", None)
    decode = getattr(_load_module(decode_path, "_flexlink_LDPC_Decode"), "LDPC_Decode", None)
    if not callable(get_h):
        raise ValueError(f"{get_h_path} has no LDPC_Get_H function")
    if not callable(decode):
        raise ValueError(f"{decode_path} has no LDPC_Decode function")
    log.debug("Using %s and %s", get_h_path, decode_path)
    return get_h, decode


def h_type(n: int, rate_str: str) -> str:
    """1296, '5/6' -> '1296_56'."""
    return f"{n}_{rate_str.replace('/', '').replace(' ', '')}"


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
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"EncoderBlockSize is {value!r} in {PARAMS_FILENAME}, expected a positive integer")
    return value


def get_code_rate(params: dict[str, Any]) -> tuple[str, Fraction]:
    """EncoderCodeRate as a string like "5/6". Returns (string, Fraction)."""
    value = get_value(params, DEFINED_SECTION, "EncoderCodeRate")
    if not isinstance(value, str):
        raise ValueError(f"EncoderCodeRate is {value!r}, expected a string like \"5/6\"")
    try:
        rate = Fraction(value.strip())
    except (ValueError, ZeroDivisionError) as exc:
        raise ValueError(f"EncoderCodeRate {value!r} is not a fraction like \"5/6\"") from exc
    if not 0 < rate < 1:
        raise ValueError(f"EncoderCodeRate {value!r} must be between 0 and 1")
    return value.strip(), rate


# --------------------------------------------------------------------------- #
# .h5 files
# --------------------------------------------------------------------------- #
def load_values(path: Path) -> tuple[np.ndarray, dict[str, Any]]:
    import h5py
    if not path.is_file():
        raise FileNotFoundError(f"Code block file not found: {path} - run Interleaver_Rx.py first")
    with h5py.File(path, "r") as f:
        if DATASET not in f:
            raise ValueError(f"{path} has no '{DATASET}' dataset (found: {', '.join(f.keys()) or 'none'})")
        values = np.asarray(f[DATASET][()], dtype=np.float64).ravel()
        attrs = dict(f.attrs)
    if values.size == 0:
        raise ValueError(f"{path} contains an empty vector")
    if not np.isfinite(values).all():
        raise ValueError(f"{path} contains NaN or infinite values")
    return values, attrs


def save_bits(path: Path, bits: np.ndarray, attrs: dict[str, Any]) -> None:
    import h5py
    with h5py.File(path, "w") as f:
        f.create_dataset(DATASET, data=bits)
        for k, v in attrs.items():
            f.attrs[k] = v


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="LDPC-decode the Rx code block test vector.")
    ap.add_argument("--params", type=Path, default=RX_DIR / PARAMS_FILENAME,
                    help=f"path to {PARAMS_FILENAME} (default: the Rx folder)")
    ap.add_argument("--ldpc-dir", type=Path, help="LDPC_Encoder directory (default: flexlink_paths.LDPC_ENCODER)")
    ap.add_argument("--iterations", type=int, default=DEFAULT_ITERATIONS,
                    help=f"decoder iterations (default: {DEFAULT_ITERATIONS})")
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
        if args.iterations < 1:
            raise ValueError("--iterations must be at least 1")

        ldpc_dir = resolve_ldpc_dir(args.ldpc_dir)
        params = load_params(args.params)

        n = get_block_size(params)
        rate_str, rate = get_code_rate(params)
        k_frac = n * rate
        if k_frac.denominator != 1:
            raise ValueError(f"EncoderBlockSize {n} x EncoderCodeRate {rate_str} is not a whole number of bits")
        k = int(k_frac)

        in_path = RX_DIR / get_value(params, FILE_SECTION, IN_KEY)
        values, in_attrs = load_values(in_path)
        log.info("Read %d values from %s", values.size, in_path)
        if values.size % n:
            raise ValueError(f"{in_path.name} has {values.size} values, not a multiple of "
                             f"EncoderBlockSize {n}")
        m = values.size // n

        get_h, decode = load_decoder(ldpc_dir)
        htype = h_type(n, rate_str)
        H = np.asarray(get_h(htype))
        if H.shape != (n - k, n):
            raise ValueError(f"LDPC_Get_H('{htype}') returned H of shape {H.shape}, expected ({n - k}, {n})")

        # Values 0..1 (0 = bit 0, 1 = bit 1) -> beliefs: negative = bit 0, positive = bit 1.
        blocks = values.reshape(m, n)
        beliefs = 2.0 * blocks - 1.0
        decoded = np.asarray(decode(beliefs, H, NumberOfIterations=args.iterations), dtype=np.uint8)
        if decoded.shape != (m, k):
            raise ValueError(f"LDPC_Decode returned shape {decoded.shape}, expected ({m}, {k})")

        # How many message bits the decoder changed compared with a plain hard decision.
        corrected = int(((beliefs[:, :k] > 0).astype(np.uint8) != decoded).sum())
        data_blk = decoded.ravel()
        log.info("Decoded %d code block(s) with H %s (%d iterations): %d -> %d bits, %d bit(s) corrected",
                 m, htype, args.iterations, values.size, data_blk.size, corrected)

        expected = params.get(CALCULATED_SECTION, {}).get("DataBlockSize")
        if expected and data_blk.size != expected:
            log.warning("Decoded length %d differs from DataBlockSize %d in %s",
                        data_blk.size, expected, PARAMS_FILENAME)

        out_path = RX_DIR / get_value(params, FILE_SECTION, OUT_KEY)
        out_attrs = dict(in_attrs)
        out_attrs.update({
            "EncoderBlockSize": n, "EncoderCodeRate": rate_str, "K": k, "N": n,
            "NumCodeBlocks": m, "DataBlockSize": data_blk.size, "H_Type": htype,
            "Iterations": args.iterations, "CorrectedBits": corrected, "SourceFile": in_path.name,
        })
        save_bits(out_path, data_blk, out_attrs)
        log.info("Data block saved to %s", out_path)

    except (FileNotFoundError, ValueError) as exc:
        log.error("%s", exc)
        return 1
    except Exception:
        log.exception("LDPC decoding failed")
        return 3

    return 0


if __name__ == "__main__":
    sys.exit(main())
