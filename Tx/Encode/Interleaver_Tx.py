#!/usr/bin/env python3
"""
Interleaver_Tx.py - Interleave the Tx LDPC code block test vector.

Steps:
  1. Read Parameters_Tx_payload.json from the Tx directory (flexlink_paths.TX).
  2. Read EncoderBlockSize (code block size CBS: 648, 1296 or 1944) from the JSON.
  3. Load the code block bits from Tx/Encode/Sequence_CodeBlk_Tx.h5
     (written by FEC_LDPC_Tx.py). Its length must be a whole number of CBS-bit blocks.
  4. Interleave each code block with Interleave_LDPC(MatrixToInterleave, CBS) from
     Interleave_LDPC.py in the Interleaver folder (flexlink_paths.INTERLEAVER, or its
     IndexGeneration / TestGeneration subfolders).
  5. Save the interleaved blocks, one after another, to
     Tx/Encode/Sequence_InvCodeBlk_Tx.h5 (the input of Rate_Matching_Tx.py). The file names are
     fixed in this script; nothing is written to the JSON.

Usage:
    python Interleaver_Tx.py
    python Interleaver_Tx.py --params my.json --tx-dir /path/to/Tx --interleaver-dir /path/to/Interleaver

The output .h5 file contains:
    data   (NumCodeBlocks * CBS,) uint8   the interleaved code blocks, one after another
    attrs  EncoderBlockSize, NumCodeBlocks, SourceFile, plus the input file's attributes
           (EncoderCodeRate, K, N, G_Type, seed, ...)
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
IN_FILE = "Sequence_CodeBlk_Tx.h5"      # written by FEC_LDPC_Tx.py
OUT_FILE = "Sequence_InvCodeBlk_Tx.h5"     # read by Rate_Matching_Tx.py
INTERLEAVE_SCRIPT = "Interleave_LDPC.py"
SUPPORTED_CBS = (648, 1296, 1944)

log = logging.getLogger("Interleaver_Tx")


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


def get_block_size(params: dict[str, Any]) -> int:
    parent, value = _find_key(params, "EncoderBlockSize")
    if parent is None:
        raise ValueError(f"EncoderBlockSize not found in {PARAMS_FILENAME}")
    if isinstance(value, bool) or not isinstance(value, int) or value not in SUPPORTED_CBS:
        raise ValueError(f"EncoderBlockSize is {value!r}, expected one of {SUPPORTED_CBS}")
    return value


# --------------------------------------------------------------------------- #
# Interleaver module
# --------------------------------------------------------------------------- #
def load_interleaver(interleaver_dir: Path) -> Callable:
    """Interleave_LDPC() from Interleave_LDPC.py in the Interleaver folder or its subfolders."""
    search = [interleaver_dir, interleaver_dir / "IndexGeneration", interleaver_dir / "TestGeneration"]
    script = next((d / INTERLEAVE_SCRIPT for d in search if (d / INTERLEAVE_SCRIPT).is_file()), None)
    if script is None:
        raise FileNotFoundError(f"{INTERLEAVE_SCRIPT} not found in: " + ", ".join(str(d) for d in search))
    if str(script.parent) not in sys.path:
        sys.path.insert(0, str(script.parent))
    spec = importlib.util.spec_from_file_location("Interleave_LDPC", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    func = getattr(module, "Interleave_LDPC", None)
    if not callable(func):
        raise ValueError(f"{script} has no Interleave_LDPC function")
    log.debug("Using %s", script)
    return func


# --------------------------------------------------------------------------- #
# .h5 files
# --------------------------------------------------------------------------- #
def load_bits(path: Path) -> tuple[np.ndarray, dict[str, Any]]:
    import h5py
    if not path.is_file():
        raise FileNotFoundError(f"Code block file not found: {path} - run FEC_LDPC_Tx.py first")
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
    ap = argparse.ArgumentParser(description="Interleave the Tx LDPC code block test vector.")
    ap.add_argument("--params", type=Path, help=f"path to {PARAMS_FILENAME} (default: Tx directory)")
    ap.add_argument("--tx-dir", type=Path, help="Tx directory (default: flexlink_paths.TX)")
    ap.add_argument("--interleaver-dir", type=Path,
                    help="folder holding Interleave_LDPC.py (default: flexlink_paths.INTERLEAVER)")
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
        interleaver_dir = resolve_dir(args.interleaver_dir, "INTERLEAVER", "Interleaver")
        params_file = args.params or tx_dir / PARAMS_FILENAME
        params = load_params(params_file)

        cbs = get_block_size(params)

        in_path = tx_dir / ENCODE_SUBDIR / IN_FILE
        bits, in_attrs = load_bits(in_path)
        log.info("Read %d bits from %s", bits.size, in_path)
        if bits.size % cbs:
            raise ValueError(f"{in_path.name} has {bits.size} bits, not a multiple of "
                             f"EncoderBlockSize {cbs}")
        if "N" in in_attrs and int(in_attrs["N"]) != cbs:
            raise ValueError(f"{in_path.name} was encoded with N = {int(in_attrs['N'])} "
                             f"but EncoderBlockSize is {cbs} - rerun FEC_LDPC_Tx.py")

        interleave = load_interleaver(interleaver_dir)
        m = bits.size // cbs
        inv = np.asarray(interleave(bits.reshape(m, cbs), cbs)).astype(np.uint8)
        if inv.shape != (m, cbs):
            raise ValueError(f"Interleave_LDPC returned shape {inv.shape}, expected ({m}, {cbs})")
        # A permutation keeps the number of ones in every block.
        if not np.array_equal(inv.sum(axis=1), bits.reshape(m, cbs).sum(axis=1)):
            raise ValueError("Interleaved blocks don't have the same bits as the input - check the interleaver")
        inv = inv.ravel()
        log.info("Interleaved %d code block(s) of %d bits", m, cbs)

        out_path = tx_dir / ENCODE_SUBDIR / OUT_FILE
        out_attrs = dict(in_attrs)
        out_attrs.update({"EncoderBlockSize": cbs, "NumCodeBlocks": m, "SourceFile": in_path.name})
        save_bits(out_path, inv, out_attrs)
        log.info("Interleaved code block saved to %s", out_path)

    except (FileNotFoundError, ValueError) as exc:
        log.error("%s", exc)
        return 1
    except Exception:
        log.exception("Interleaving failed")
        return 3

    return 0


if __name__ == "__main__":
    sys.exit(main())
