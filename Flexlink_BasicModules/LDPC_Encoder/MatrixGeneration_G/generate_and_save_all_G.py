"""generate_and_save_all_G.py

Generates the generator matrix G for all twelve 802.11n LDPC code types using
ldpc_generator.generate_g_matrix and stores each one in its own HDF5 file:

    G_648_12.h5, G_648_23.h5, ..., G_1944_56.h5

Each file contains one dataset named "G" (uint8, shape (K, N), values 0/1,
gzip-compressed), plus attributes describing the code.

Usage
-----
    python generate_and_save_all_G.py                # write into current folder
    python generate_and_save_all_G.py path/to/dir    # write into that folder

Reading a file back:

    import h5py
    with h5py.File("G_648_12.h5", "r") as f:
        G = f["G"][:]          # np.ndarray, uint8, shape (324, 648)

Requires: numpy, h5py   (pip install h5py)
"""

import sys
from pathlib import Path

import numpy as np
import h5py

from ldpc_generator import generate_g_matrix

CODE_TYPES = (
    "648_12", "648_23", "648_34", "648_56",
    "1296_12", "1296_23", "1296_34", "1296_56",
    "1944_12", "1944_23", "1944_34", "1944_56",
)


def save_g_matrix(G, code_type, out_dir):
    """Write G to ``G_<code_type>.h5`` in out_dir and return the path."""
    path = Path(out_dir) / f"G_{code_type}.h5"
    K, N = G.shape
    with h5py.File(path, "w") as f:
        dset = f.create_dataset("G", data=G, compression="gzip", compression_opts=9)
        dset.attrs["description"] = "802.11n LDPC generator matrix G = [I_K | A^T], mod 2"
        f.attrs["code_type"] = code_type
        f.attrs["N"] = N
        f.attrs["K"] = K
        f.attrs["rate"] = K / N
    return path


def main(out_dir="."):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    for code_type in CODE_TYPES:
        G, H, col_perm = generate_g_matrix(code_type)
        L, N = H.shape
        K = N - L

        # Refuse to save a G that doesn't match the standard bit ordering
        if not np.array_equal(col_perm, np.arange(N)):
            raise RuntimeError(f"{code_type}: elimination permuted the columns")

        # Every row of G must be a codeword of H
        syndrome = (G.astype(np.int64) @ H.astype(np.int64).T) % 2
        if syndrome.any():
            raise RuntimeError(f"{code_type}: G * H^T != 0")

        path = save_g_matrix(G, code_type, out_dir)

        # Read back and confirm what was written is what was generated
        with h5py.File(path, "r") as f:
            G_read = f["G"][:]
        if G_read.dtype != G.dtype or not np.array_equal(G_read, G):
            raise RuntimeError(f"{code_type}: read-back mismatch for {path}")

        size_kb = path.stat().st_size / 1024
        print(f"{code_type}: G {G.shape} -> {path}  ({size_kb:.0f} KB)  OK")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
