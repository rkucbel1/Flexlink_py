#!/usr/bin/env python3
"""
Create_Sequence.py

Creates SequenceToRateMatch.h5 containing a dataset named "sequence"
with 100 random bits (0 or 1).

Usage:
    python Create_Sequence.py            # length 100
    python Create_Sequence.py -n 250     # different length
"""

import argparse
import h5py
import numpy as np

FILE_NAME = "SequenceToRateMatch.h5"
DATASET = "sequence"


def main():
    parser = argparse.ArgumentParser(description="Create the input HDF5 sequence file.")
    parser.add_argument("-n", "--length", type=int, default=100, help="Sequence length (default 100)")
    parser.add_argument("-f", "--file", default=FILE_NAME, help="Output file name")
    args = parser.parse_args()

    seq = np.random.randint(0, 2, size=args.length, dtype=np.uint8)
    with h5py.File(args.file, "w") as f:
        f.create_dataset(DATASET, data=seq)
    print(f"Wrote {len(seq)} random bits to '{args.file}' (dataset '{DATASET}').")


if __name__ == "__main__":
    main()
