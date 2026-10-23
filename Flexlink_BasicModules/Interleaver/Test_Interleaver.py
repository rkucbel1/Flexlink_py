"""Test_Interleaver.py

Round-trip test of the LDPC and BCC interleavers / deinterleavers.

For each test case:
    1. Read  the test input file             (dataset "TestInput")
    2. Interleave   (Interleave_LDPC / Interleave_BCC)
    3. DeInterleave (DeInterleave_LDPC / DeInterleave_BCC)
    4. Write the test output file containing
           "TestOutput"   - the deinterleaved result
           "Interleaved"  - the intermediate interleaved result
    5. Check that TestOutput equals TestInput (and that interleaving
       actually changed the data and preserved the bit count).

Test cases:
    LDPC: Test_Input_648 / 1296 / 1944      -> Test_Output_648 / 1296 / 1944
    BCC : Test_Input_BCC_40 / 140           -> Test_Output_BCC_40 / 140

Usage:
    python Test_Interleaver.py                 # files in the current folder
    python Test_Interleaver.py path/to/dir     # files in that folder

Exit code is 0 if all tests pass, 1 otherwise.
Requires: numpy, h5py, Interleave_LDPC.py, DeInterleave_LDPC.py,
Interleave_BCC.py, DeInterleave_BCC.py and the Interleaver_<CBS>.h5 files
(in the same folder as the scripts). Create the Test_Input files with
Generate_Test_Inputs.py.
"""

import sys
from pathlib import Path

import numpy as np
import h5py

from Interleave_LDPC import Interleave_LDPC
from DeInterleave_LDPC import DeInterleave_LDPC
from Interleave_BCC import Interleave_BCC
from DeInterleave_BCC import DeInterleave_BCC

# (label, length, input name, output name, interleave fn, deinterleave fn)
TESTS = [
    ("LDPC", 648,  "Test_Input_648.h5",  "Test_Output_648.h5",  Interleave_LDPC, DeInterleave_LDPC),
    ("LDPC", 1296, "Test_Input_1296.h5", "Test_Output_1296.h5", Interleave_LDPC, DeInterleave_LDPC),
    ("LDPC", 1944, "Test_Input_1944.h5", "Test_Output_1944.h5", Interleave_LDPC, DeInterleave_LDPC),
    ("BCC",  40,   "Test_Input_BCC_40.h5",  "Test_Output_BCC_40.h5",  Interleave_BCC, DeInterleave_BCC),
    ("BCC",  140,  "Test_Input_BCC_140.h5", "Test_Output_BCC_140.h5", Interleave_BCC, DeInterleave_BCC),
]


def run_test(label, length, in_name, out_name, interleave, deinterleave, data_dir):
    in_path = data_dir / in_name
    out_path = data_dir / out_name

    # 1. Read the test input
    with h5py.File(in_path, "r") as f:
        TestInput = f["TestInput"][()]

    # 2. Interleave, 3. DeInterleave
    Interleaved = interleave(TestInput, length)
    TestOutput = deinterleave(Interleaved, length)

    # 4. Write the output file
    with h5py.File(out_path, "w") as f:
        f.create_dataset("TestOutput", data=TestOutput)
        f.create_dataset("Interleaved", data=Interleaved)
        f.attrs["CBS"] = length

    # 5. Checks
    shape_ok = TestOutput.shape == TestInput.shape
    match = shape_ok and np.array_equal(TestOutput, TestInput)
    changed = not np.array_equal(Interleaved, TestInput)         # interleaver did something
    same_ones = int(Interleaved.sum()) == int(TestInput.sum())   # bits only moved

    with h5py.File(out_path, "a") as f:
        f.attrs["match"] = int(match)

    num_err = int(np.count_nonzero(TestOutput != TestInput)) if shape_ok else -1
    ok = match and changed and same_ones
    print(f"{label:4s} {length:4d}: {'PASS' if ok else 'FAIL'}  "
          f"input{TestInput.shape} -> output{TestOutput.shape}, mismatches={num_err}, "
          f"interleaver changed data={changed}, bit count preserved={same_ones}  "
          f"-> {out_name}")
    return ok


def main():
    data_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(".")
    results = []
    for label, length, in_name, out_name, il, dl in TESTS:
        try:
            results.append(run_test(label, length, in_name, out_name, il, dl, data_dir))
        except (FileNotFoundError, OSError) as e:
            print(f"{label:4s} {length:4d}: FAIL  file problem: {e}")
            results.append(False)

    print("\nALL TESTS PASSED" if all(results) else "\nSOME TESTS FAILED")
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
