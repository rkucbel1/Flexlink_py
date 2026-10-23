"""LDPC_Get_H.py

Loads the parity-check matrix H for an 802.11n LDPC code from its HDF5 file
(H_<type>.h5, as written by generate_and_save_all_H.py).

Usage
-----
    from LDPC_Get_H import LDPC_Get_H

    H = LDPC_Get_H('648_12')                          # uint8, shape (324, 648)
    H = LDPC_Get_H('648_12', directory='H_matrices')  # look in a specific folder
    H = LDPC_Get_H('648_12', dtype=np.int64)          # ready for (H @ c) % 2

Load H once and reuse it; do not call this for every codeword.

Requires: numpy, h5py
"""

from pathlib import Path

import h5py

VALID_TYPES = (
    "648_12", "648_23", "648_34", "648_56",
    "1296_12", "1296_23", "1296_34", "1296_56",
    "1944_12", "1944_23", "1944_34", "1944_56",
)

_THIS_DIR = Path(__file__).resolve().parent


def LDPC_Get_H(code_type, directory=None, dtype=None):
    """Read H for the given LDPC type from its HDF5 file and return it.

    Parameters
    ----------
    code_type : str
        One of '648_12', '648_23', '648_34', '648_56', '1296_12', '1296_23',
        '1296_34', '1296_56', '1944_12', '1944_23', '1944_34', '1944_56'.
    directory : str or Path, optional
        Folder containing the H_<type>.h5 files. If omitted, these folders
        are searched in order and the first one containing the file is used:
        ./H_matrices, <this script's folder>/H_matrices, ., <this script's folder>.
    dtype : numpy dtype, optional
        Convert H to this dtype (e.g. np.int64). Default: as stored (uint8).

    Returns
    -------
    H : np.ndarray, shape (L, N)
        Writable array of 0/1 values (the parity-check matrix).
    """
    if not isinstance(code_type, str):
        raise TypeError(f"code_type must be a string, got {type(code_type).__name__}")

    key = code_type.strip()
    if key not in VALID_TYPES:
        raise ValueError(
            f"Invalid code type {code_type!r}. Valid types: {', '.join(VALID_TYPES)}"
        )

    filename = f"H_{key}.h5"
    if directory is not None:
        candidates = [Path(directory) / filename]
    else:
        cwd = Path.cwd()
        candidates = [
            cwd / "H_matrices" / filename,
            _THIS_DIR / "H_matrices" / filename,
            cwd / filename,
            _THIS_DIR / filename,
        ]

    path = next((p for p in candidates if p.is_file()), None)
    if path is None:
        searched = "\n  ".join(str(p) for p in candidates)
        raise FileNotFoundError(
            f"Could not find {filename}. Looked in:\n  {searched}\n"
            "Run generate_and_save_all_H.py to create it, or pass directory=..."
        )

    with h5py.File(path, "r") as f:
        H = f["H"][:]

    if dtype is not None:
        H = H.astype(dtype, copy=False)
    return H


if __name__ == "__main__":
    import sys

    for t in sys.argv[1:] or VALID_TYPES:
        H = LDPC_Get_H(t)
        L, N = H.shape
        print(f"{t}: H shape {H.shape} (L={L}, N={N}, K={N - L}), dtype {H.dtype}")
