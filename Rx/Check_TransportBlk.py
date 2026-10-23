#!/usr/bin/env python3
"""
Check_TransportBlk.py - Compare the received transport block with the transmitted one.

Steps:
  1. Read Parameters_Rx_payload.json from this script's Rx folder.
  2. Load the Rx transport block from File_Rx/TestVector_TransportBlk_Rx in the Rx
     folder (written by Check_CRC.py) and the Tx transport block from
     Tx/Encode/Sequence_TransportBlk_Tx.h5 (written by Generate_TestVector_TransportBlk_Tx.py).
  3. Compare them bit by bit and report MATCH or MISMATCH, the number of bit
     errors, the bit error rate and the first few error positions. The CRC result
     recorded by Check_CRC.py is reported alongside.

Exit status: 0 = MATCH, 2 = MISMATCH, 1 = input/setup error, 3 = unexpected error.

Usage:
    python Check_TransportBlk.py
    python Check_TransportBlk.py --rx-params my_rx.json --tx-dir /path/to/Tx
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import logging
import sys
from pathlib import Path
from typing import Any

import numpy as np

RX_DIR = Path(__file__).resolve().parent          # the Rx folder
RX_PARAMS_FILENAME = "Parameters_Rx_payload.json"

RX_FILE_SECTION = "File_Rx"
RX_KEY = "TestVector_TransportBlk_Rx"
TX_FILE = "Sequence_TransportBlk_Tx.h5"   # in Tx/Encode
TX_SUBDIR = "Encode"
DATASET = "data"
SHOW_ERRORS = 10                                  # error positions listed in the log

EXIT_MATCH, EXIT_ERROR, EXIT_MISMATCH, EXIT_UNEXPECTED = 0, 1, 2, 3

log = logging.getLogger("Check_TransportBlk")


# --------------------------------------------------------------------------- #
# Folders and JSON
# --------------------------------------------------------------------------- #
def resolve_tx_dir(explicit: Path | None) -> Path:
    d = explicit
    if d is None:
        try:
            import flexlink_paths
            d = Path(flexlink_paths.TX)
        except (ImportError, AttributeError):
            raise FileNotFoundError("Tx directory unknown (flexlink_paths.TX) - pass --tx-dir") from None
    if not d.is_dir():
        raise FileNotFoundError(f"Tx directory not found: {d}")
    return d


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


def file_name(params: dict[str, Any], section: str, key: str, params_name: str) -> str:
    try:
        name = params[section][key]
    except (KeyError, TypeError):
        name = None
    if not name:
        raise ValueError(f"{section}/{key} not set in {params_name}")
    return str(name)


# --------------------------------------------------------------------------- #
# .h5 files
# --------------------------------------------------------------------------- #
def load_bits(path: Path, what: str, hint: str) -> tuple[np.ndarray, dict[str, Any]]:
    import h5py
    if not path.is_file():
        raise FileNotFoundError(f"{what} transport block file not found: {path} - {hint}")
    with h5py.File(path, "r") as f:
        if DATASET not in f:
            raise ValueError(f"{path} has no '{DATASET}' dataset (found: {', '.join(f.keys()) or 'none'})")
        bits = np.asarray(f[DATASET][()]).ravel()
        attrs = dict(f.attrs)
    if not np.isin(bits, (0, 1)).all():
        raise ValueError(f"{path} contains values other than 0 and 1")
    return bits.astype(np.uint8), attrs


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Compare the Rx transport block with the Tx transport block.")
    ap.add_argument("--rx-params", type=Path, default=RX_DIR / RX_PARAMS_FILENAME,
                    help=f"path to {RX_PARAMS_FILENAME} (default: the Rx folder)")
    ap.add_argument("--tx-dir", type=Path, help="Tx directory (default: flexlink_paths.TX)")
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
            raise ValueError("Reading .h5 files needs h5py - install it with: pip install h5py")

        tx_dir = resolve_tx_dir(args.tx_dir)
        rx_params = load_params(args.rx_params)

        rx_path = RX_DIR / file_name(rx_params, RX_FILE_SECTION, RX_KEY, RX_PARAMS_FILENAME)
        tx_path = tx_dir / TX_SUBDIR / TX_FILE
        rx, rx_attrs = load_bits(rx_path, "Rx", "run Check_CRC.py first")
        tx, _ = load_bits(tx_path, "Tx", "run Tx/Encode/Generate_TestVector_TransportBlk_Tx.py first")
        log.info("Rx: %d bits from %s", rx.size, rx_path)
        log.info("Tx: %d bits from %s", tx.size, tx_path)

        if "CRC_Pass" in rx_attrs:
            log.info("CRC check recorded by Check_CRC.py: %s",
                     "PASSED" if int(rx_attrs["CRC_Pass"]) else "FAILED")

        if rx.size != tx.size:
            log.error("MISMATCH: Rx transport block has %d bits, Tx has %d", rx.size, tx.size)
            return EXIT_MISMATCH

        errors = np.flatnonzero(rx != tx)
        if errors.size == 0:
            log.info("MATCH: Rx and Tx transport blocks are identical (%d bits)", rx.size)
            return EXIT_MATCH

        shown = ", ".join(map(str, errors[:SHOW_ERRORS])) + (", ..." if errors.size > SHOW_ERRORS else "")
        log.error("MISMATCH: %d of %d bits differ (BER %.3e), first at bit(s) %s",
                  errors.size, rx.size, errors.size / rx.size, shown)
        return EXIT_MISMATCH

    except (FileNotFoundError, ValueError) as exc:
        log.error("%s", exc)
        return EXIT_ERROR
    except Exception:
        log.exception("Transport block comparison failed to run")
        return EXIT_UNEXPECTED


if __name__ == "__main__":
    sys.exit(main())
