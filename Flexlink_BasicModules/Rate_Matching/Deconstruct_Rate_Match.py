#!/usr/bin/env python3
"""
Deconstruct_Rate_Match.py

Reverses the repetition rate matching done by Rate_Match.py.

Input  : dataset "sequence" in SequenceToDeconstruct.h5 (any length; re-read
         on every run).
Param  : Repetition (command line, or prompted if omitted).

Steps:
    1. The sequence length must be divisible by (Repetition + 1), otherwise
       an error is raised.
    2. The sequence is split into (Repetition + 1) equal chunks.
    3. The chunks are summed element by element into one vector.
    4. Each element is divided by (Repetition + 1).

The chunks are not expected to be identical (noisy / soft values), so output
elements may fall anywhere between 0 and 1.

Output : dataset "sequence" (float64) in SequenceDeconstructed.h5.

Example:
    input = 0 1 0 1 1  0 1 1 1 1  0 1 0 0 1 ,  Repetition = 2
    output = 0  1  0.3333  0.6667  1

Usage:
    python Deconstruct_Rate_Match.py -r 3
    python Deconstruct_Rate_Match.py                    (prompts for Repetition)
    python Deconstruct_Rate_Match.py -r 3 -i in.h5 -o out.h5
"""

import argparse
import h5py
import numpy as np

INPUT_FILE = "SequenceToDeconstruct.h5"
OUTPUT_FILE = "SequenceDeconstructed.h5"
DATASET = "sequence"


def deconstruct_rate_match(sequence, repetition):
    """Average the (repetition + 1) chunks of `sequence` element by element."""
    if repetition < 0:
        raise ValueError("Repetition must be a non-negative integer.")

    seq = np.asarray(sequence, dtype=np.float64).ravel()
    n_chunks = repetition + 1

    if seq.size % n_chunks != 0:
        raise ValueError(
            f"Sequence length ({seq.size}) is not divisible by "
            f"Repetition+1 ({n_chunks})."
        )

    chunks = seq.reshape(n_chunks, seq.size // n_chunks)  # one chunk per row
    temp = chunks.sum(axis=0)                             # element-wise sum
    return temp / n_chunks                                # divide by Repetition+1


def read_sequence(path, dataset=DATASET):
    """Read the whole sequence from the HDF5 file (length is not assumed)."""
    with h5py.File(path, "r") as f:
        return f[dataset][()]


def write_sequence(path, sequence, dataset=DATASET):
    """Write the sequence to an HDF5 file, replacing any existing file."""
    with h5py.File(path, "w") as f:
        f.create_dataset(dataset, data=sequence)


def main():
    parser = argparse.ArgumentParser(description="Deconstruct a repetition rate-matched sequence.")
    parser.add_argument("-r", "--repetition", type=int, help="Repetition used when the sequence was rate matched")
    parser.add_argument("-i", "--input", default=INPUT_FILE, help=f"Input file (default {INPUT_FILE})")
    parser.add_argument("-o", "--output", default=OUTPUT_FILE, help=f"Output file (default {OUTPUT_FILE})")
    args = parser.parse_args()

    repetition = args.repetition
    if repetition is None:
        repetition = int(input("Enter Repetition: "))

    sequence = read_sequence(args.input)
    try:
        output = deconstruct_rate_match(sequence, repetition)
    except ValueError as err:
        raise SystemExit(f"Error: {err}")
    write_sequence(args.output, output)

    print(f"Read {len(sequence)} values from '{args.input}'")
    print(f"Repetition: {repetition}  ({repetition + 1} chunks of {len(output)})")
    print(f"Wrote {len(output)} values to '{args.output}'")


if __name__ == "__main__":
    main()
