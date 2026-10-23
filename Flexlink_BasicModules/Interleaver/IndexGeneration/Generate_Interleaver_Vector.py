"""Generate_Interleaver_Vector.py

Computes the interleaver index vector (InterleaverIndex) for the code block
sizes CBS = 648, 1296 and 1944 and stores each one in its own HDF5 file:

    Interleaver_648.h5, Interleaver_1296.h5, Interleaver_1944.h5

Each file contains one dataset named "InterleaverIndex" (int32, shape (CBS,))
plus the attributes CBS, NumRows, NumCols and index_base.

Procedure (port of the MATLAB code)
-----------------------------------
The indices 0 .. CBS-1 are written down the columns of a 61-row matrix
(unused cells are -1). The rows are then read out, each from left to right,
in a fixed row order (RowOrder), skipping the unused cells.

Index base
----------
MATLAB's last step adds 1 to the vector (1-based indices). By default this
script stores the 0-BASED indices, so they can be used directly in NumPy:

    interleaved = data[InterleaverIndex]

The MATLAB vector is exactly this vector + 1. Use --one-based to store the
MATLAB values instead. The base is saved in the file attribute "index_base".

Usage
-----
    python Generate_Interleaver_Vector.py                 # write into current folder
    python Generate_Interleaver_Vector.py path/to/dir     # write into that folder
    python Generate_Interleaver_Vector.py --one-based     # MATLAB-style 1-based values

Reading a file back:

    import h5py
    with h5py.File("Interleaver_648.h5", "r") as f:
        InterleaverIndex = f["InterleaverIndex"][:]    # int32, shape (648,)

Requires: numpy, h5py
"""

import argparse
import math
from pathlib import Path

import numpy as np
import h5py

CBS_VALUES = (648, 1296, 1944)
NUM_ROWS = 61


def _colon(start, step, stop):
    """MATLAB's start:step:stop (stop is inclusive)."""
    return np.arange(start, stop + 1, step)


def _row_order():
    """RowOrder from the MATLAB code (0-based row numbers)."""
    return np.concatenate([
        _colon(0, 12, 60), _colon(6, 12, 54), _colon(3, 12, 51), _colon(9, 12, 57),
        _colon(1, 12, 49), _colon(11, 12, 59), _colon(2, 12, 50), _colon(10, 12, 58),
        _colon(4, 12, 52), _colon(8, 12, 56), _colon(5, 12, 53), _colon(7, 12, 55),
    ])


def generate_interleaver_vector(CBS, one_based=False):
    """Return the interleaver index vector for code block size CBS.

    Parameters
    ----------
    CBS : int
        Code block size (648, 1296 or 1944 are used here, but any positive
        integer works).
    one_based : bool, optional
        If True, return MATLAB-style 1-based indices (vector + 1).
        Default False (0-based, ready for NumPy indexing).

    Returns
    -------
    InterleaverIndex : np.ndarray of int64, shape (CBS,)
        A permutation of 0 .. CBS-1 (or 1 .. CBS if one_based).
    """
    CBS = int(CBS)
    if CBS < 1:
        raise ValueError(f"CBS must be a positive integer, got {CBS}")

    NumRows = NUM_ROWS
    NumCols = math.ceil(CBS / NumRows)

    # Index matrix filled down the columns; -1 marks unused cells
    IndexMatrix = -np.ones((NumRows, NumCols), dtype=np.int64)
    for Index in range(CBS):
        Row = Index % NumRows
        Column = Index // NumRows
        IndexMatrix[Row, Column] = Index

    RowOrder = _row_order()
    if not np.array_equal(np.sort(RowOrder), np.arange(NumRows)):
        raise RuntimeError("RowOrder is not a permutation of the 61 rows")

    # Read the rows out in RowOrder, left to right, skipping unused cells
    InterleavingIndexVector = np.zeros(CBS, dtype=np.int64)
    Count = 0
    for RowIndex in RowOrder:
        for ColumnIndex in range(NumCols):
            value = IndexMatrix[RowIndex, ColumnIndex]
            if value != -1:
                InterleavingIndexVector[Count] = value
                Count += 1

    if Count != CBS:
        raise RuntimeError(f"CBS={CBS}: collected {Count} indices, expected {CBS}")

    return InterleavingIndexVector + 1 if one_based else InterleavingIndexVector


def save_interleaver_vector(vec, CBS, out_dir, one_based):
    """Write the vector to Interleaver_<CBS>.h5 in out_dir and return the path."""
    path = Path(out_dir) / f"Interleaver_{CBS}.h5"
    with h5py.File(path, "w") as f:
        dset = f.create_dataset("InterleaverIndex", data=vec.astype(np.int32))
        dset.attrs["description"] = (
            "Interleaver index vector; "
            + ("1-based (MATLAB style)" if one_based else "0-based (NumPy style)")
        )
        f.attrs["CBS"] = CBS
        f.attrs["NumRows"] = NUM_ROWS
        f.attrs["NumCols"] = math.ceil(CBS / NUM_ROWS)
        f.attrs["index_base"] = 1 if one_based else 0
    return path


def main():
    parser = argparse.ArgumentParser(description="Generate interleaver index vectors and save them to HDF5.")
    parser.add_argument("out_dir", nargs="?", default=".",
                        help="folder for the output files (default: current folder)")
    parser.add_argument("--one-based", action="store_true",
                        help="store MATLAB-style 1-based indices instead of 0-based")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    base = 1 if args.one_based else 0

    for CBS in CBS_VALUES:
        vec = generate_interleaver_vector(CBS, one_based=args.one_based)

        # Must be a permutation of base .. base+CBS-1
        if not np.array_equal(np.sort(vec), np.arange(base, base + CBS)):
            raise RuntimeError(f"CBS={CBS}: vector is not a permutation")

        path = save_interleaver_vector(vec, CBS, out_dir, args.one_based)

        # Read back and confirm what was written is what was generated
        with h5py.File(path, "r") as f:
            vec_read = f["InterleaverIndex"][:]
        if not np.array_equal(vec_read, vec):
            raise RuntimeError(f"CBS={CBS}: read-back mismatch for {path}")

        print(f"CBS={CBS}: InterleaverIndex {vec_read.shape} {vec_read.dtype}, "
              f"{base}-based -> {path}  OK")


if __name__ == "__main__":
    main()
