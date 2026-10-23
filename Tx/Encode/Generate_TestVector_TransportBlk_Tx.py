#!/usr/bin/env python3
"""
Generate_TestVector_TransportBlk_Tx.py  (lives in Flexlink_py/Tx/Encode)

Creates the random transport block that starts the Tx test chain:
    Generate_TestVector_TransportBlk_Tx.py -> Attach_CRC.py -> FEC_LDPC_Tx.py
    -> Interleaver_Tx.py -> Rate_Matching_Tx.py

Steps:
  1. Read Calculated/TransportBlockSize from Tx/Parameters_Tx_payload.json
     (written by Initialize_Tx.py).
  2. Generate TransportBlockSize random bits (0 or 1).
  3. Save them to Tx/Encode/Sequence_TransportBlk_Tx.h5. The file name is fixed
     in this script; nothing is written to the JSON.

Rx/Check_TransportBlk.py compares the received transport block against this file.

Usage:
    python Generate_TestVector_TransportBlk_Tx.py                # random bits
    python Generate_TestVector_TransportBlk_Tx.py --seed 1234    # reproducible bits

The .h5 file contains:
    data   (TransportBlockSize,) uint8   the bits, 0 or 1
    attrs  TransportBlockSize, seed
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

# Make flexlink_paths importable when the script is run from anywhere.
for _parent in Path(__file__).resolve().parents:
    if (_parent / "flexlink_paths.py").is_file():
        sys.path.insert(0, str(_parent))
        break
from flexlink_paths import TX  # noqa: E402

PARAMS_FILE = TX / "Parameters_Tx_payload.json"
OUT_FILE = TX / "Encode" / "Sequence_TransportBlk_Tx.h5"   # read by Attach_CRC.py


def transport_block_size(params_file: Path) -> int:
    if not params_file.is_file():
        raise FileNotFoundError(f"Parameter file not found: {params_file}")
    with open(params_file, "r", encoding="utf-8") as fh:
        params = json.load(fh)
    size = params.get("Calculated", {}).get("TransportBlockSize")
    if isinstance(size, bool) or not isinstance(size, int) or size <= 0:
        raise ValueError(f"Calculated/TransportBlockSize is {size!r} in {params_file.name} - "
                         "run Initialize_Tx.py first")
    return size


def generate_bits(length: int, seed: int) -> np.ndarray:
    """`length` random bits, each 0 or 1."""
    return np.random.default_rng(seed).integers(0, 2, size=length, dtype=np.uint8)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Generate the random Tx transport block test sequence.")
    ap.add_argument("--seed", type=int, help="random seed for a reproducible vector (default: random)")
    ap.add_argument("--params", type=Path, default=PARAMS_FILE,
                    help=f"parameter file (default: {PARAMS_FILE})")
    args = ap.parse_args(argv)

    try:
        import h5py
    except ImportError:
        print("Error: saving the vector needs h5py - install it with: pip install h5py", file=sys.stderr)
        return 1

    try:
        length = transport_block_size(args.params)
    except (FileNotFoundError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    seed = args.seed if args.seed is not None else int(np.random.SeedSequence().entropy % 2**63)
    bits = generate_bits(length, seed)

    OUT_FILE.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(OUT_FILE, "w") as f:
        f.create_dataset("data", data=bits)
        f.attrs["TransportBlockSize"] = length
        f.attrs["seed"] = seed

    print(f"Generated {length} bits ({int(bits.sum())} ones, {length - int(bits.sum())} zeros), seed {seed}")
    print(f"Wrote {OUT_FILE}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
