"""
Test_Scrambler.py

Round-trip test of Scrambler_GC.py:

    Test_Input_2048.h5 -> scrambler_gc -> scrambler_gc -> Test_Output_2048.h5

Scrambling is modulo-2 addition with the Gold code, so scrambling a second
time with the same id must give back the original input.  The script compares
the final output against the input and runs a few extra sanity checks.

Usage:
    python Test_Scrambler.py [-c ID] [-i INPUT] [-o OUTPUT]

Test_Output_2048.h5 contents:
    /SequenceDescrambled   dataset, final output (second pass), uint8 0/1
    /SequenceScrambled     dataset, intermediate result (first pass), uint8 0/1
    /length                dataset, number of elements
    attributes on the file root: id, match (1 = output equals input),
                                 mismatches, all_checks_passed (1/0)

Exit code is 0 if all checks pass, 1 otherwise.
"""

import argparse
import os
import sys

import h5py
import numpy as np

from Scrambler_GC import scrambler_gc

INPUT_FILE = "Test_Input_2048.h5"
OUTPUT_FILE = "Test_Output_2048.h5"
DEFAULT_ID = 130
WRONG_ID_TMP_FILE = "GC_Scrambler_Sequence_wrong_id_tmp.h5"


def read_input(filename):
    if not os.path.isfile(filename):
        sys.exit("Input file not found: %s (run Generate_Test_Inputs.py first)" % filename)
    with h5py.File(filename, "r") as f:
        sequence = f["SequenceToScramble"][:]
        length = int(f["length"][()])
    return sequence, length


def run_test(id, input_file, output_file):
    sequence_in, stored_length = read_input(input_file)
    n = len(sequence_in)

    # Process: scrambler_gc, then scrambler_gc again on its output
    scrambled = scrambler_gc(id, sequence_in)
    sequence_out = scrambler_gc(id, scrambled)

    # Comparison of input and output
    mismatches = int(np.sum(sequence_in != sequence_out))
    match = mismatches == 0

    # Extra checks
    # Wrong id must NOT recover the input (uses a temporary Gold code file so
    # the real GC_Scrambler_Sequence.h5 is left untouched)
    try:
        wrong = scrambler_gc(id + 1, scrambled, h5_file=WRONG_ID_TMP_FILE)
    finally:
        if os.path.isfile(WRONG_ID_TMP_FILE):
            os.remove(WRONG_ID_TMP_FILE)
    wrong_id_differs = not np.array_equal(wrong, sequence_in)

    bits_changed = int(np.sum(sequence_in != scrambled))

    checks = [
        ("Input contains only 0 and 1", bool(np.isin(sequence_in, (0, 1)).all())),
        ("Input length matches length stored in file", stored_length == n),
        ("Scrambled length equals input length", len(scrambled) == n),
        ("Output length equals input length", len(sequence_out) == n),
        ("Scrambling changed the sequence", bits_changed > 0),
        ("Output matches input (%d mismatches)" % mismatches, match),
        ("Wrong id does not recover the input", wrong_id_differs),
    ]
    passed = all(ok for _, ok in checks)

    # Write the output file
    with h5py.File(output_file, "w") as f:  # "w" creates or overwrites
        f.create_dataset("SequenceDescrambled", data=sequence_out)
        f.create_dataset("SequenceScrambled", data=scrambled)
        f.create_dataset("length", data=n)
        f.attrs["id"] = int(id)
        f.attrs["match"] = int(match)
        f.attrs["mismatches"] = mismatches
        f.attrs["all_checks_passed"] = int(passed)

    # Report
    print("Input file     : %s (%d elements, %.1f%% ones)"
          % (input_file, n, 100.0 * float(np.mean(sequence_in))))
    print("Gold code id   : %d" % id)
    print("Bits changed by scrambling: %d (%.1f%%)" % (bits_changed, 100.0 * bits_changed / n))
    for name, ok in checks:
        print("  [%s] %s" % ("PASS" if ok else "FAIL", name))
    print("Output file    : %s" % output_file)
    print("RESULT: %s" % ("PASS" if passed else "FAIL"))
    return passed


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Round-trip test of Scrambler_GC.py")
    parser.add_argument("-c", "--id", type=int, default=DEFAULT_ID,
                        help="Gold code id / c_init (default: %d)" % DEFAULT_ID)
    parser.add_argument("-i", "--input", default=INPUT_FILE,
                        help="input h5 file (default: %s)" % INPUT_FILE)
    parser.add_argument("-o", "--output", default=OUTPUT_FILE,
                        help="output h5 file (default: %s)" % OUTPUT_FILE)
    args = parser.parse_args()

    sys.exit(0 if run_test(args.id, args.input, args.output) else 1)
