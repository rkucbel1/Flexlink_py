"""generate_and_save_all_H.py

Generates the parity-check matrix H for all twelve 802.11n LDPC code types
using generate_h_matrix and stores each one in its own HDF5 file:

    H_648_12.h5, H_648_23.h5, ..., H_1944_56.h5

Each file contains one dataset named "H" (uint8, shape (L, N), values 0/1),
plus attributes describing the code.

Usage
-----
    python generate_and_save_all_H.py                # write into current folder
    python generate_and_save_all_H.py path/to/dir    # write into that folder

Reading a file back:

    import h5py
    with h5py.File("H_648_12.h5", "r") as f:
        H = f["H"][:]          # np.ndarray, uint8, shape (324, 648)

Requires: numpy, h5py   (pip install h5py)
"""

import sys
from pathlib import Path

import numpy as np
import h5py

from generate_h_matrix import generate_h_matrix, VALID_TYPES

# Dataset compression. H is mostly zeros, so it compresses very well.
# For fastest writes/reads use COMPRESSION = {} (no compression) or
# {"compression": "lzf"}.
COMPRESSION = {"compression": "gzip", "compression_opts": 9}

# Code rate for each type suffix
RATE = {"12": (1, 2), "23": (2, 3), "34": (3, 4), "56": (5, 6)}


def save_h_matrix(H, code_type, out_dir):
    """Write H to ``H_<code_type>.h5`` in out_dir and return the path."""
    path = Path(out_dir) / f"H_{code_type}.h5"
    L, N = H.shape
    K = N - L
    with h5py.File(path, "w") as f:
        dset = f.create_dataset("H", data=H, **COMPRESSION)
        dset.attrs["description"] = "802.11n LDPC parity-check matrix H (binary)"
        f.attrs["code_type"] = code_type
        f.attrs["N"] = N
        f.attrs["K"] = K
        f.attrs["L"] = L
        f.attrs["rate"] = K / N
    return path


def main(out_dir="."):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    for code_type in VALID_TYPES:
        H = generate_h_matrix(code_type)
        L, N = H.shape

        # Shape must match the code type: N from the name, L from the rate
        n_name, rate_name = code_type.split("_")
        num, den = RATE[rate_name]
        if N != int(n_name) or L * den != N * (den - num):
            raise RuntimeError(f"{code_type}: unexpected H shape {H.shape}")
        if not np.all((H == 0) | (H == 1)):
            raise RuntimeError(f"{code_type}: H contains values other than 0/1")

        path = save_h_matrix(H, code_type, out_dir)

        # Read back and confirm what was written is what was generated
        with h5py.File(path, "r") as f:
            H_read = f["H"][:]
        if H_read.dtype != H.dtype or not np.array_equal(H_read, H):
            raise RuntimeError(f"{code_type}: read-back mismatch for {path}")

        size_kb = path.stat().st_size / 1024
        print(f"{code_type}: H {H.shape} -> {path}  ({size_kb:.0f} KB)  OK")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
