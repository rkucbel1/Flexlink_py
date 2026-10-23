#!/usr/bin/env python3
"""
FEC_LDPC_Tx.py - LDPC-encode the Tx data block test vector into the code block.

Steps:
  1. Read Parameters_Tx_payload.json from the Tx directory (flexlink_paths.TX).
  2. Read EncoderBlockSize (codeword length N), EncoderCodeRate (string, e.g. "1/2")
     and DataBlockSize from the JSON.
  3. Load the data block bits from Tx/Encode/Sequence_DataBlk_Tx.h5
     (written by Attach_CRC.py). Its length must equal DataBlockSize.
  4. Get G with LDPC_Get_G('<N>_<rate digits>'), e.g. '648_12' for N=648, rate 1/2,
     and encode with LDPC_Encode(DataBlk, G), both from flexlink_paths.LDPC_ENCODER.
     The data block is split into DataBlockSize / K code blocks of K bits each.
  5. Save the concatenated codewords to Tx/Encode/Sequence_CodeBlk_Tx.h5
     (the input of Interleaver_Tx.py). The file names are fixed in this script;
     nothing is written to the JSON.

Usage:
    python FEC_LDPC_Tx.py
    python FEC_LDPC_Tx.py --params my.json --tx-dir /path/to/Tx --ldpc-dir /path/to/LDPC_Encoder

The output .h5 file contains:
    data   (NumCodeBlocks * N,) uint8   the codewords, one after another
    attrs  EncoderBlockSize, EncoderCodeRate, K, N, NumCodeBlocks, DataBlockSize, G_Type,
           SourceFile, seed (if the input had one)
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

PARAMS_FILENAME = "Parameters_Tx_payload.json"
ENCODE_SUBDIR = "Encode"     # the Sequence_*.h5 test files live in Tx/Encode
IN_FILE = "Sequence_DataBlk_Tx.h5"      # written by Attach_CRC.py
OUT_FILE = "Sequence_CodeBlk_Tx.h5"     # read by Interleaver_Tx.py
ENCODE_SCRIPT = "LDPC_Encode.py"
GET_G_SCRIPT = "LDPC_Get_G.py"

log = logging.getLogger("FEC_LDPC_Tx")


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


def get_positive_int(params: dict[str, Any], key: str) -> int:
    parent, value = _find_key(params, key)
    if parent is None:
        raise ValueError(f"{key} not found in {PARAMS_FILENAME}")
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{key} is {value!r} in {PARAMS_FILENAME}, expected a positive integer")
    return value


def get_code_rate(params: dict[str, Any]) -> tuple[str, Fraction]:
    """EncoderCodeRate as a string like "1/2". Returns (string, Fraction)."""
    parent, value = _find_key(params, "EncoderCodeRate")
    if parent is None:
        raise ValueError(f"EncoderCodeRate not found in {PARAMS_FILENAME}")
    if not isinstance(value, str):
        raise ValueError(f"EncoderCodeRate is {value!r}, expected a string like \"1/2\"")
    try:
        rate = Fraction(value.strip())
    except (ValueError, ZeroDivisionError) as exc:
        raise ValueError(f"EncoderCodeRate {value!r} is not a fraction like \"1/2\"") from exc
    if not 0 < rate < 1:
        raise ValueError(f"EncoderCodeRate {value!r} must be between 0 and 1")
    return value.strip(), rate


# --------------------------------------------------------------------------- #
# LDPC encoder modules (LDPC_Get_G.py and LDPC_Encode.py, loaded by file path)
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


def load_encoder(ldpc_dir: Path) -> tuple[Callable, Callable]:
    """Returns (LDPC_Get_G, LDPC_Encode) from the LDPC_Encoder folder."""
    search = [ldpc_dir, ldpc_dir / "MatrixGeneration_G"]
    encode_path = _find_script(ENCODE_SCRIPT, search)
    get_g_path = _find_script(GET_G_SCRIPT, search)

    # Let the encoder files import their own helpers.
    for d in dict.fromkeys([get_g_path.parent, encode_path.parent]):
        if str(d) not in sys.path:
            sys.path.insert(0, str(d))

    get_g = getattr(_load_module(get_g_path, "_flexlink_LDPC_Get_G"), "LDPC_Get_G", None)
    encode = getattr(_load_module(encode_path, "_flexlink_LDPC_Encode"), "LDPC_Encode", None)
    if not callable(get_g):
        raise ValueError(f"{get_g_path} has no LDPC_Get_G function")
    if not callable(encode):
        raise ValueError(f"{encode_path} has no LDPC_Encode function")
    return get_g, encode


def g_type(n: int, rate_str: str) -> str:
    """'648', '1/2' -> '648_12'."""
    return f"{n}_{rate_str.replace('/', '').replace(' ', '')}"


def load_g(get_g: Callable, gtype: str) -> np.ndarray:
    try:
        G = get_g(gtype, dtype=np.float32)
    except TypeError:
        G = get_g(gtype)
    return np.asarray(G)


# --------------------------------------------------------------------------- #
# .h5 files
# --------------------------------------------------------------------------- #
def load_bits(path: Path) -> tuple[np.ndarray, dict[str, Any]]:
    import h5py
    if not path.is_file():
        raise FileNotFoundError(f"Data block file not found: {path} - run Attach_CRC.py first")
    with h5py.File(path, "r") as f:
        if "data" not in f:
            raise ValueError(f"{path} has no 'data' dataset")
        bits = np.asarray(f["data"][()]).ravel().astype(np.uint8)
        attrs = dict(f.attrs)
    if not np.isin(bits, (0, 1)).all():
        raise ValueError(f"{path} contains values other than 0 and 1")
    return bits, attrs


def save_code_blk(path: Path, code: np.ndarray, attrs: dict[str, Any]) -> None:
    import h5py
    with h5py.File(path, "w") as f:
        f.create_dataset("data", data=code)
        for k, v in attrs.items():
            f.attrs[k] = v


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="LDPC-encode the Tx data block test vector.")
    ap.add_argument("--params", type=Path, help=f"path to {PARAMS_FILENAME} (default: Tx directory)")
    ap.add_argument("--tx-dir", type=Path, help="Tx directory (default: flexlink_paths.TX)")
    ap.add_argument("--ldpc-dir", type=Path, help="LDPC_Encoder directory (default: flexlink_paths.LDPC_ENCODER)")
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
        ldpc_dir = resolve_dir(args.ldpc_dir, "LDPC_ENCODER", "LDPC_Encoder")
        params_file = args.params or tx_dir / PARAMS_FILENAME
        params = load_params(params_file)

        n = get_positive_int(params, "EncoderBlockSize")
        rate_str, rate = get_code_rate(params)
        data_size = get_positive_int(params, "DataBlockSize")

        k_frac = n * rate
        if k_frac.denominator != 1:
            raise ValueError(f"EncoderBlockSize {n} x EncoderCodeRate {rate_str} is not a whole number of bits")
        k = int(k_frac)
        if data_size % k:
            raise ValueError(f"DataBlockSize {data_size} is not a multiple of K = {k} "
                             f"(N = {n}, rate {rate_str})")

        in_path = tx_dir / ENCODE_SUBDIR / IN_FILE
        bits, in_attrs = load_bits(in_path)
        if bits.size != data_size:
            raise ValueError(f"{in_path.name} has {bits.size} bits but DataBlockSize is {data_size}")
        log.info("Read %d bits from %s", bits.size, in_path)

        get_g, encode = load_encoder(ldpc_dir)
        gtype = g_type(n, rate_str)
        G = load_g(get_g, gtype)
        if G.shape != (k, n):
            raise ValueError(f"LDPC_Get_G('{gtype}') returned G of shape {G.shape}, expected ({k}, {n})")

        m = data_size // k
        code = np.asarray(encode(bits.reshape(m, k), G), dtype=np.uint8).reshape(m, n)
        # G is systematic: the first K bits of each codeword must be the message.
        if not np.array_equal(code[:, :k], bits.reshape(m, k)):
            log.warning("Codewords do not start with the message bits - is G systematic?")
        code = code.ravel()
        log.info("Encoded %d code block(s) with G %s: %d -> %d bits", m, gtype, data_size, code.size)

        out_path = tx_dir / ENCODE_SUBDIR / OUT_FILE
        out_attrs = {
            "EncoderBlockSize": n, "EncoderCodeRate": rate_str, "K": k, "N": n,
            "NumCodeBlocks": m, "DataBlockSize": data_size, "G_Type": gtype,
            "SourceFile": in_path.name,
        }
        if "seed" in in_attrs:
            out_attrs["seed"] = in_attrs["seed"]
        save_code_blk(out_path, code, out_attrs)
        log.info("Code block saved to %s", out_path)

    except (FileNotFoundError, ValueError) as exc:
        log.error("%s", exc)
        return 1
    except Exception:
        log.exception("LDPC encoding failed")
        return 3

    return 0


if __name__ == "__main__":
    sys.exit(main())
