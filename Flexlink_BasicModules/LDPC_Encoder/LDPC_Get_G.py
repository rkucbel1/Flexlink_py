"""LDPC_Get_G.py

Loads the generator matrix G for an 802.11n LDPC code from its HDF5 file
(G_<type>.h5, as written by generate_and_save_all_G.py).

Usage
-----
    from LDPC_Get_G import LDPC_Get_G

    G = LDPC_Get_G('648_12')                          # uint8, shape (324, 648)
    G = LDPC_Get_G('648_12', directory='G_matrices')  # look in a specific folder
    G = LDPC_Get_G('648_12', dtype=np.int64)          # ready for (msg @ G) % 2

Load G once and reuse it; do not call this for every message.

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


def LDPC_Get_G(code_type, directory=None, dtype=None):
    """Read G for the given LDPC type from its HDF5 file and return it.

    Parameters
    ----------
    code_type : str
        One of '648_12', '648_23', '648_34', '648_56', '1296_12', '1296_23',
        '1296_34', '1296_56', '1944_12', '1944_23', '1944_34', '1944_56'.
    directory : str or Path, optional
        Folder containing the G_<type>.h5 files. If omitted, these folders
        are searched in order and the first one containing the file is used:
        ./G_matrices, <this script's folder>/G_matrices, ., <this script's folder>.
    dtype : numpy dtype, optional
        Convert G to this dtype (e.g. np.int64). Default: as stored (uint8).

    Returns
    -------
    G : np.ndarray, shape (K, N)
        Writable array of 0/1 values, G = [I_K | A^T].
    """
    if not isinstance(code_type, str):
        raise TypeError(f"code_type must be a string, got {type(code_type).__name__}")

    key = code_type.strip()
    if key not in VALID_TYPES:
        raise ValueError(
            f"Invalid code type {code_type!r}. Valid types: {', '.join(VALID_TYPES)}"
        )

    filename = f"G_{key}.h5"
    if directory is not None:
        candidates = [Path(directory) / filename]
    else:
        cwd = Path.cwd()
        candidates = [
            cwd / "G_matrices" / filename,
            _THIS_DIR / "G_matrices" / filename,
            cwd / filename,
            _THIS_DIR / filename,
        ]

    path = next((p for p in candidates if p.is_file()), None)
    if path is None:
        searched = "\n  ".join(str(p) for p in candidates)
        raise FileNotFoundError(
            f"Could not find {filename}. Looked in:\n  {searched}\n"
            "Run generate_and_save_all_G.py to create it, or pass directory=..."
        )

    with h5py.File(path, "r") as f:
        G = f["G"][:]

    if dtype is not None:
        G = G.astype(dtype, copy=False)
    return G


if __name__ == "__main__":
    import sys

    for t in sys.argv[1:] or VALID_TYPES:
        G = LDPC_Get_G(t)
        K, N = G.shape
        print(f"{t}: G shape {G.shape} (K={K}, N={N}), dtype {G.dtype}")
