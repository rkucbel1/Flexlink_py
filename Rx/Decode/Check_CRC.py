#!/usr/bin/env python3
"""
Check_CRC.py - Check and remove the CRC from the Rx data block.

Steps:
  1. Read Parameters_Rx_payload.json from this script's Rx folder.
  2. Load the data block bits from the .h5 file named in File_Rx/TestVector_DataBlk_Rx
     (written by FEC_LDPC_Rx.py).
  3. Split it: transport block = all but the last CRClen bits; received CRC = the
     last CRClen bits. CRClen comes from Calculated_Rx and must match the width of
     the CRC polynomial (Defined_Rx/CRC, e.g. "Gen24").
  4. Recalculate the CRC of the transport block with GenerateCRC() from
     CRC_Gen/GenerateCRC.py (flexlink_paths.CRC_GEN) and compare it with the
     received CRC: equal = PASS, different = FAIL.
  5. Save the transport block to the .h5 file named in File_Rx/TestVector_TransportBlk_Rx
     in the Rx folder (written whether the check passes or fails, so it can be inspected).

CRC polynomial: Defined_Rx/CRC in the JSON; --crc-gen overrides it.
24, "24" or "CRC24" are read as Gen24.

Exit status: 0 = CRC PASS, 2 = CRC FAIL, 1 = input/setup error, 3 = unexpected error.

Usage:
    python Check_CRC.py
    python Check_CRC.py --crc-gen Gen16
    python Check_CRC.py --params my.json --crc-dir /path/to/CRC_Gen

The output .h5 file contains:
    data   (TransportBlockSize,) uint8   the transport block bits
    crc    (CRC length,) uint8           the received CRC bits
    attrs  TransportBlockSize, CRCLength, CRC_Gen, CRC_Pass (1/0), ReceivedCRC,
           CalculatedCRC, SourceFile, plus the input file's attributes
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

IN_KEY = "TestVector_DataBlk_Rx"
OUT_KEY = "TestVector_TransportBlk_Rx"
CRC_SCRIPT = "GenerateCRC.py"
CRC_FIELD = "CRC"                                 # Defined_Rx/CRC selects the polynomial
DATASET = "data"

EXIT_PASS, EXIT_ERROR, EXIT_FAIL, EXIT_UNEXPECTED = 0, 1, 2, 3

log = logging.getLogger("Check_CRC")


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


def resolve_crc_dir(explicit: Path | None) -> Path:
    d = explicit or _flexlink_dir("CRC_GEN")
    if d is None:
        raise FileNotFoundError("CRC_Gen directory unknown - pass --crc-dir")
    if not d.is_dir():
        raise FileNotFoundError(f"CRC_Gen directory not found: {d}")
    return d


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


def _normalize_crc_gen(value: Any) -> str:
    """Accept 'Gen24', 24, '24' or 'CRC24' and return 'Gen24' (same rules as Attach_CRC.py)."""
    if isinstance(value, bool):
        raise ValueError(f"{DEFINED_SECTION}/{CRC_FIELD} is {value!r}, expected e.g. 'Gen24'")
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
    raise ValueError(f"{DEFINED_SECTION}/{CRC_FIELD} is {value!r}, expected e.g. 'Gen24'")


def crc_gen_name(params: dict[str, Any], explicit: str | None) -> str:
    """CRC polynomial: --crc-gen, else Defined_Rx/CRC in the JSON."""
    if explicit:
        return _normalize_crc_gen(explicit)
    name = _normalize_crc_gen(get_value(params, DEFINED_SECTION, CRC_FIELD))
    log.info("CRC polynomial %s (from %s/%s)", name, DEFINED_SECTION, CRC_FIELD)
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


def bit_string(bits: np.ndarray) -> str:
    return "".join(map(str, np.asarray(bits).astype(int)))


# --------------------------------------------------------------------------- #
# .h5 files
# --------------------------------------------------------------------------- #
def load_bits(path: Path) -> tuple[np.ndarray, dict[str, Any]]:
    import h5py
    if not path.is_file():
        raise FileNotFoundError(f"Data block file not found: {path} - run FEC_LDPC_Rx.py first")
    with h5py.File(path, "r") as f:
        if DATASET not in f:
            raise ValueError(f"{path} has no '{DATASET}' dataset (found: {', '.join(f.keys()) or 'none'})")
        bits = np.asarray(f[DATASET][()]).ravel()
        attrs = dict(f.attrs)
    if bits.size == 0:
        raise ValueError(f"{path} contains an empty vector")
    if not np.isin(bits, (0, 1)).all():
        raise ValueError(f"{path} contains values other than 0 and 1")
    return bits.astype(np.uint8), attrs


def save_transport_blk(path: Path, tb: np.ndarray, crc: np.ndarray, attrs: dict[str, Any]) -> None:
    import h5py
    with h5py.File(path, "w") as f:
        f.create_dataset(DATASET, data=tb)
        f.create_dataset("crc", data=crc)
        for k, v in attrs.items():
            f.attrs[k] = v


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Check and remove the CRC from the Rx data block.")
    ap.add_argument("--params", type=Path, default=RX_DIR / PARAMS_FILENAME,
                    help=f"path to {PARAMS_FILENAME} (default: the Rx folder)")
    ap.add_argument("--crc-dir", type=Path, help="CRC_Gen directory (default: flexlink_paths.CRC_GEN)")
    ap.add_argument("--crc-gen", help="CRC polynomial: Gen24, Gen16, Gen10 or Gen3 (overrides Defined_Rx/CRC)")
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

        crc_dir = resolve_crc_dir(args.crc_dir)
        params = load_params(args.params)
        crc_gen = crc_gen_name(params, args.crc_gen)
        generate_crc = load_generate_crc(crc_dir)

        crc_len = get_value(params, CALCULATED_SECTION, "CRClen")
        if isinstance(crc_len, bool) or not isinstance(crc_len, int) or crc_len <= 0:
            raise ValueError(f"CRClen is {crc_len!r} in {PARAMS_FILENAME} - run Initialize_Rx.py first")
        width = np.asarray(generate_crc(np.zeros(1, dtype=np.uint8), crc_gen)).size
        if width != crc_len:
            raise ValueError(f"CRClen {crc_len} in {PARAMS_FILENAME} doesn't match the {width}-bit "
                             f"{crc_gen} polynomial - run Initialize_Rx.py")

        in_path = RX_DIR / get_value(params, FILE_SECTION, IN_KEY)
        bits, in_attrs = load_bits(in_path)
        log.info("Read %d bits from %s", bits.size, in_path)
        if bits.size <= crc_len:
            raise ValueError(f"{in_path.name} has {bits.size} bits, not more than CRClen {crc_len}")

        tb, received_crc = bits[:-crc_len], bits[-crc_len:]
        expected_tb = params.get(CALCULATED_SECTION, {}).get("TransportBlockSize")
        if expected_tb and tb.size != expected_tb:
            log.warning("Transport block length %d differs from TransportBlockSize %d in %s",
                        tb.size, expected_tb, PARAMS_FILENAME)

        calculated_crc = np.asarray(generate_crc(tb, crc_gen)).ravel().astype(np.uint8)
        passed = bool(np.array_equal(calculated_crc, received_crc))
        log.info("Received CRC   %s", bit_string(received_crc))
        log.info("Calculated CRC %s", bit_string(calculated_crc))

        out_path = RX_DIR / get_value(params, FILE_SECTION, OUT_KEY)
        out_attrs = dict(in_attrs)
        out_attrs.update({
            "TransportBlockSize": tb.size, "CRCLength": crc_len, "CRC_Gen": crc_gen,
            "CRC_Pass": int(passed), "ReceivedCRC": bit_string(received_crc),
            "CalculatedCRC": bit_string(calculated_crc), "SourceFile": in_path.name,
        })
        save_transport_blk(out_path, tb, received_crc, out_attrs)
        log.info("Saved %d-bit transport block to %s", tb.size, out_path)

    except (FileNotFoundError, ValueError) as exc:
        log.error("%s", exc)
        return EXIT_ERROR
    except Exception:
        log.exception("CRC check failed to run")
        return EXIT_UNEXPECTED

    if passed:
        log.info("CRC check PASSED")
        return EXIT_PASS
    log.error("CRC check FAILED: %d of %d CRC bits differ",
              int((calculated_crc != received_crc).sum()), crc_len)
    return EXIT_FAIL


if __name__ == "__main__":
    sys.exit(main())
