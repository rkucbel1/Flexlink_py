#!/usr/bin/env python3
"""
Attach_CRC.py - Append a CRC to the Tx transport block test vector.

Steps:
  1. Read Parameters_Tx_payload.json from the Tx directory (flexlink_paths.TX).
  2. Load the transport block bits from Tx/Encode/Sequence_TransportBlk_Tx.h5
     (written by Generate_TestVector_TransportBlk_Tx.py).
  3. Compute the CRC with GenerateCRC() from CRC_Gen/GenerateCRC.py
     (flexlink_paths.CRC_GEN).
  4. Save transport block + CRC to Tx/Encode/Sequence_DataBlk_Tx.h5
     (the input of FEC_LDPC_Tx.py). The file names are fixed in this script;
     nothing is written to the JSON.

CRC polynomial (Gen24, Gen16, Gen10 or Gen3): taken from Defined/CRC in the JSON;
  --crc-gen on the command line overrides it. 24, "24" or "CRC24" are read as Gen24.

Usage:
    python Attach_CRC.py
    python Attach_CRC.py --crc-gen Gen16
    python Attach_CRC.py --params my.json --tx-dir /path/to/Tx --crc-dir /path/to/CRC_Gen

The output .h5 file contains:
    data   (TransportBlockSize + CRC length,) uint8   transport block bits, then CRC bits
    crc    (CRC length,) uint8                         the CRC bits alone
    attrs  TransportBlockSize, CRCLength, CRC_Gen, SourceFile, seed (if the input had one)
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
IN_FILE = "Sequence_TransportBlk_Tx.h5"      # written by Generate_TestVector_TransportBlk_Tx.py
OUT_FILE = "Sequence_DataBlk_Tx.h5"     # read by FEC_LDPC_Tx.py
CRC_SCRIPT = "GenerateCRC.py"
CRC_FIELD = "CRC"          # Defined/CRC selects the polynomial

log = logging.getLogger("Attach_CRC")


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


def _normalize_crc_gen(value: Any) -> str:
    """Accept 'Gen24', 24, '24' or 'CRC24' and return 'Gen24'."""
    if isinstance(value, bool):
        raise ValueError(f"Defined/CRC is {value!r}, expected e.g. 'Gen24'")
    if isinstance(value, int):
        return f"Gen{value}"
    if isinstance(value, str) and value.strip():
        s = value.strip()
        digits = "".join(ch for ch in s if ch.isdigit())
        if s.lower().startswith("gen") and digits:
            return f"Gen{digits}"
        if digits and s.lower().lstrip("crc_- ") == digits:
            return f"Gen{digits}"
        return s
    raise ValueError(f"Defined/CRC is {value!r}, expected e.g. 'Gen24'")


def crc_gen_name(params: dict[str, Any], explicit: str | None) -> str:
    """CRC polynomial: --crc-gen, else Defined/CRC in the JSON."""
    if explicit:
        return _normalize_crc_gen(explicit)
    defined = params.get("Defined")
    if not isinstance(defined, dict) or CRC_FIELD not in defined:
        raise ValueError(f"Defined/{CRC_FIELD} not set in {PARAMS_FILENAME} - "
                         "set it (e.g. \"Gen24\") or pass --crc-gen")
    name = _normalize_crc_gen(defined[CRC_FIELD])
    log.info("CRC polynomial %s (from Defined/%s)", name, CRC_FIELD)
    return name


# --------------------------------------------------------------------------- #
# CRC
# --------------------------------------------------------------------------- #
def load_generate_crc(crc_dir: Path) -> Callable[[np.ndarray, str], np.ndarray]:
    script = crc_dir / CRC_SCRIPT
    if not script.is_file():
        raise FileNotFoundError(f"{CRC_SCRIPT} not found in {crc_dir}")
    spec = importlib.util.spec_from_file_location("GenerateCRC", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    func = getattr(module, "GenerateCRC", None)
    if not callable(func):
        raise ValueError(f"{script} has no GenerateCRC function")
    return func


def attach_crc(bits: np.ndarray, crc_gen: str, generate_crc: Callable) -> tuple[np.ndarray, np.ndarray]:
    """Returns (bits followed by CRC, CRC alone), both uint8."""
    crc = np.asarray(generate_crc(bits, crc_gen)).ravel().astype(np.uint8)
    out = np.concatenate([bits, crc])
    # Appending the remainder must give an all-zero remainder.
    if np.asarray(generate_crc(out, crc_gen)).any():
        raise ValueError(f"CRC self-check failed for {crc_gen}")
    return out, crc


# --------------------------------------------------------------------------- #
# .h5 files
# --------------------------------------------------------------------------- #
def load_bits(path: Path) -> tuple[np.ndarray, dict[str, Any]]:
    import h5py
    if not path.is_file():
        raise FileNotFoundError(f"Transport block file not found: {path} - "
                                "run Generate_TestVector_TransportBlk_Tx.py first")
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


def save_bits(path: Path, out: np.ndarray, crc: np.ndarray, crc_gen: str,
              tb_size: int, source: str, in_attrs: dict[str, Any]) -> None:
    import h5py
    with h5py.File(path, "w") as f:
        f.create_dataset("data", data=out)
        f.create_dataset("crc", data=crc)
        f.attrs["TransportBlockSize"] = tb_size
        f.attrs["CRCLength"] = crc.size
        f.attrs["CRC_Gen"] = crc_gen
        f.attrs["SourceFile"] = source
        if "seed" in in_attrs:
            f.attrs["seed"] = in_attrs["seed"]


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Append a CRC to the Tx transport block test vector.")
    ap.add_argument("--params", type=Path, help=f"path to {PARAMS_FILENAME} (default: Tx directory)")
    ap.add_argument("--tx-dir", type=Path, help="Tx directory (default: flexlink_paths.TX)")
    ap.add_argument("--crc-dir", type=Path, help="CRC_Gen directory (default: flexlink_paths.CRC_GEN)")
    ap.add_argument("--crc-gen", help="CRC polynomial: Gen24, Gen16, Gen10 or Gen3 (overrides Defined/CRC in the JSON)")
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
        crc_dir = resolve_dir(args.crc_dir, "CRC_GEN", "CRC_Gen")
        params_file = args.params or tx_dir / PARAMS_FILENAME
        params = load_params(params_file)

        in_path = tx_dir / ENCODE_SUBDIR / IN_FILE
        bits, in_attrs = load_bits(in_path)
        log.info("Read %d bits from %s", bits.size, in_path)

        _, tb_size = _find_key(params, "TransportBlockSize")
        if isinstance(tb_size, int) and tb_size != bits.size:
            log.warning("TransportBlockSize in %s is %d but the vector has %d bits",
                        PARAMS_FILENAME, tb_size, bits.size)

        crc_gen = crc_gen_name(params, args.crc_gen)
        generate_crc = load_generate_crc(crc_dir)
        out, crc = attach_crc(bits, crc_gen, generate_crc)
        log.info("CRC %s (%d bits): %s", crc_gen, crc.size, "".join(map(str, crc)))

        out_path = tx_dir / ENCODE_SUBDIR / OUT_FILE
        save_bits(out_path, out, crc, crc_gen, bits.size, in_path.name, in_attrs)
        log.info("Saved %d bits (%d + %d CRC) to %s", out.size, bits.size, crc.size, out_path)

    except (FileNotFoundError, ValueError) as exc:
        log.error("%s", exc)
        return 1
    except Exception:
        log.exception("Attaching CRC failed")
        return 3

    return 0


if __name__ == "__main__":
    sys.exit(main())
