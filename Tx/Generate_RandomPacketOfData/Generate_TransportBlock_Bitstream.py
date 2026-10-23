#!/usr/bin/env python3
"""
Generate_TransportBlock_Bitstream.py

Reads Calculated.AvailableDataBlocks and Calculated.TransportBlockSize from
<Flexlink_py>/Tx/Parameters_Tx_payload.json, generates a random bit vector of
length AvailableDataBlocks x TransportBlockSize with 45 % ones and 55 % zeros,
and writes it to the file named in File.Bitstream_TransportBlks_Tx
(e.g. Bitstream_TransportBlks_Tx.h5) in the same folder as this script.
Paths come from flexlink_paths.py, so it runs from any working directory.

Usage:
    python Generate_TransportBlock_Bitstream.py
    python Generate_TransportBlock_Bitstream.py --json path/to/params.json \
        --out path/to/out.h5 --seed 1234
"""

import argparse
import json
import sys
from pathlib import Path

import h5py
import numpy as np

# Locate flexlink_paths.py by walking up from this script's folder, so the
# script works wherever it sits inside the Flexlink_py project.
for _parent in Path(__file__).resolve().parents:
    if (_parent / "flexlink_paths.py").is_file():
        sys.path.insert(0, str(_parent))
        break
from flexlink_paths import TX  # noqa: E402

DEFAULT_JSON = TX / "Parameters_Tx_payload.json"
OUT_DIR = Path(__file__).resolve().parent      # output name comes from the JSON
OUT_KEY = "Bitstream_TransportBlks_Tx"         # File/<key> = output file name

ONES_RATIO = 0.45  # fraction of '1' bits; the remaining 55 % are '0'


def load_parameters(json_path):
    if not json_path.is_file():
        sys.exit(f"Error: parameter file not found: {json_path}\n"
                 f"Pass the correct location with --json <path>.")
    with open(json_path, "r") as f:
        params = json.load(f)
    try:
        calc = params["Calculated"]
        n_blocks = int(calc["AvailableDataBlocks"])
        tb_size = int(calc["TransportBlockSize"])
    except KeyError as e:
        sys.exit(f"Error: missing key {e} in {json_path}")
    if n_blocks <= 0 or tb_size <= 0:
        sys.exit("Error: AvailableDataBlocks and TransportBlockSize must be > 0")
    name = params.get("File", {}).get(OUT_KEY) if isinstance(params.get("File"), dict) else None
    if not isinstance(name, str) or not name.strip():
        sys.exit(f"Error: File.{OUT_KEY} not found in {json_path}")
    return n_blocks, tb_size, OUT_DIR / name.strip()


def generate_bitstream(length, ones_ratio, rng):
    """Bit vector with exactly round(ones_ratio * length) ones, randomly placed."""
    n_ones = int(round(ones_ratio * length))
    bits = np.zeros(length, dtype=np.uint8)
    bits[:n_ones] = 1
    rng.shuffle(bits)
    return bits


def main():
    parser = argparse.ArgumentParser(description="Generate random transport block bitstream.")
    parser.add_argument("--json", type=Path, default=DEFAULT_JSON,
                        help=f"Parameter file (default: {DEFAULT_JSON})")
    parser.add_argument("--out", type=Path,
                        help=f"Output HDF5 file (default: File.{OUT_KEY} in {OUT_DIR})")
    parser.add_argument("--seed", type=int, default=None,
                        help="Random seed for reproducible output (default: random)")
    args = parser.parse_args()

    n_blocks, tb_size, out_path = load_parameters(args.json)
    out_path = args.out or out_path
    length = n_blocks * tb_size

    rng = np.random.default_rng(args.seed)
    bits = generate_bitstream(length, ONES_RATIO, rng)

    with h5py.File(out_path, "w") as f:
        dset = f.create_dataset("TransportBlock_Bitstream", data=bits)
        dset.attrs["AvailableDataBlocks"] = n_blocks
        dset.attrs["TransportBlockSize"] = tb_size
        dset.attrs["Length"] = length
        dset.attrs["OnesRatio"] = ONES_RATIO
        if args.seed is not None:
            dset.attrs["Seed"] = args.seed

    n_ones = int(bits.sum())
    print(f"AvailableDataBlocks = {n_blocks}, TransportBlockSize = {tb_size}")
    print(f"Wrote {length} bits to {out_path} "
          f"(ones: {n_ones} = {100 * n_ones / length:.2f} %, "
          f"zeros: {length - n_ones} = {100 * (length - n_ones) / length:.2f} %)")


if __name__ == "__main__":
    main()
