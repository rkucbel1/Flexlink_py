"""generate_h_matrix.py

Returns the parity-check matrix H for an 802.11n LDPC code. Python port of

    [P, SubmatrixSize] = GetP_LDPC(Type);
    H = CreateParityCheckMatrix(P, SubmatrixSize);

Usage
-----
    from generate_h_matrix import generate_h_matrix

    H = generate_h_matrix('648_56')                   # uint8, shape (108, 648)
    H = generate_h_matrix('648_56', dtype=np.int64)   # for (H @ c) % 2

Command line
------------
    python generate_h_matrix.py 648_56      # prints shape info
    python generate_h_matrix.py             # all twelve types
"""

import sys

import numpy as np

from get_p_ldpc import get_p_ldpc
from create_parity_check_matrix import create_parity_check_matrix

VALID_TYPES = (
    "648_12", "648_23", "648_34", "648_56",
    "1296_12", "1296_23", "1296_34", "1296_56",
    "1944_12", "1944_23", "1944_34", "1944_56",
)


def generate_h_matrix(code_type, dtype=np.uint8):
    """Return the parity-check matrix H for the given LDPC type.

    Parameters
    ----------
    code_type : str
        One of '648_12', '648_23', '648_34', '648_56', '1296_12', '1296_23',
        '1296_34', '1296_56', '1944_12', '1944_23', '1944_34', '1944_56'
        (block length N and rate: 12 = 1/2, 23 = 2/3, 34 = 3/4, 56 = 5/6).
    dtype : numpy dtype, optional
        Output dtype (default ``np.uint8``). Use ``np.int64`` if you will
        multiply H with other arrays, since uint8 products overflow.

    Returns
    -------
    H : np.ndarray, shape (L, N) with L = N - K
        Binary (0/1) parity-check matrix.
    """
    if not isinstance(code_type, str):
        raise TypeError(f"code_type must be a string, got {type(code_type).__name__}")

    key = code_type.strip()
    if key not in VALID_TYPES:
        raise ValueError(
            f"Invalid code type {code_type!r}. Valid types: {', '.join(VALID_TYPES)}"
        )

    P, submatrix_size = get_p_ldpc(key)
    H = create_parity_check_matrix(P, submatrix_size)
    return H if np.dtype(dtype) == H.dtype else H.astype(dtype)


if __name__ == "__main__":
    for t in sys.argv[1:] or VALID_TYPES:
        H = generate_h_matrix(t)
        L, N = H.shape
        print(f"{t}: H shape {H.shape} (L={L}, N={N}, K={N - L}), dtype {H.dtype}, "
              f"ones per row {H.sum(axis=1).min()}-{H.sum(axis=1).max()}")
