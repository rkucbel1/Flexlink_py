#!/usr/bin/env python3
"""
Rate_Match.py

Repetition-based rate matching using HDF5 files.

Input  : dataset "sequence" in SequenceToRateMatch.h5 (any length; the file
         is re-read on every run, so another process may overwrite it).
Param  : Repetition (command line, or prompted if omitted).
Output : dataset "sequence" in SequenceRateMatched.h5, containing the input
         sequence followed by `Repetition` additional copies of it.

Example:
    input = 0 1 0 1 1, Repetition = 3
    output = 0 1 0 1 1 0 1 0 1 1 0 1 0 1 1 0 1 0 1 1

Usage:
    python Rate_Match.py -r 3
    python Rate_Match.py                      (prompts for Repetition)
    python Rate_Match.py -r 3 -i in.h5 -o out.h5
"""

import argparse
import h5py
import numpy as np

INPUT_FILE = "SequenceToRateMatch.h5"
OUTPUT_FILE = "SequenceRateMatched.h5"
DATASET = "sequence"


def rate_match(sequence, repetition):
    """Return `sequence` with `repetition` extra copies appended."""
    if repetition < 0:
        raise ValueError("Repetition must be a non-negative integer.")
    seq = np.asarray(sequence).ravel()
    return np.tile(seq, repetition + 1)


def read_sequence(path, dataset=DATASET):
    """Read the whole sequence from the HDF5 file (length is not assumed)."""
    with h5py.File(path, "r") as f:
        return f[dataset][()]


def write_sequence(path, sequence, dataset=DATASET):
    """Write the sequence to an HDF5 file, replacing any existing file."""
    with h5py.File(path, "w") as f:
        f.create_dataset(dataset, data=sequence)


def main():
    parser = argparse.ArgumentParser(description="Repetition rate matching on HDF5 files.")
    parser.add_argument("-r", "--repetition", type=int, help="Number of times to append the sequence")
    parser.add_argument("-i", "--input", default=INPUT_FILE, help=f"Input file (default {INPUT_FILE})")
    parser.add_argument("-o", "--output", default=OUTPUT_FILE, help=f"Output file (default {OUTPUT_FILE})")
    args = parser.parse_args()

    repetition = args.repetition
    if repetition is None:
        repetition = int(input("Enter Repetition: "))

    sequence = read_sequence(args.input)
    output = rate_match(sequence, repetition)
    write_sequence(args.output, output)

    print(f"Read {len(sequence)} values from '{args.input}'")
    print(f"Repetition: {repetition}")
    print(f"Wrote {len(output)} values to '{args.output}'")


if __name__ == "__main__":
    main()
