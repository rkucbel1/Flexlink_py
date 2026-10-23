"""Python port of UseGaussianElimination / AddRows and the G-matrix script."""

import numpy as np

from get_p_ldpc import get_p_ldpc
from create_parity_check_matrix import create_parity_check_matrix


def add_rows(M, source_row, target_row):
    """Add (mod 2) ``source_row`` of M to ``target_row`` of M, in place."""
    M[target_row, :] ^= M[source_row, :]
    return M


def switch_columns(M, col_a, col_b):
    """Swap two columns of M, in place.

    (SwitchColumns was not included in the MATLAB sources provided; this
    assumes it is a plain column swap.)
    """
    M[:, [col_a, col_b]] = M[:, [col_b, col_a]]
    return M


def use_gaussian_elimination(H):
    """Bring H into the form [A | I_L] (identity on the right).

    Returns
    -------
    H_new : np.ndarray of uint8, shape (L, N)
    col_perm : np.ndarray of int, shape (N,)
        ``H_new`` is a row-equivalent form of ``H[:, col_perm]``. If no
        column swaps were needed this is ``arange(N)``. If it is not the
        identity permutation, the code defined by G is a bit-permuted
        version of the code defined by the original H.
    """
    H_new = np.array(H, dtype=np.uint8)          # copy
    L, N = H_new.shape
    K = N - L
    col_perm = np.arange(N)

    # Already of the form [A | I]? Then nothing to do.
    if np.array_equal(H_new[:, K:], np.eye(L, dtype=np.uint8)):
        return H_new, col_perm

    # ---- Step 1: forward elimination --------------------------------------
    for column in range(K, N):
        r = column - K                      # row of interest (0-based)

        if H_new[r, column] != 1:           # need to pivot
            pivot_ok = False

            # Try to find a 1 in this column in a row below and add it in
            below = np.nonzero(H_new[r + 1:, column])[0]
            if below.size > 0:
                add_rows(H_new, r + 1 + below[0], r)
                pivot_ok = True

            # Otherwise swap columns: find a 1 in the row of interest
            if not pivot_ok:
                ones = np.nonzero(H_new[r, :])[0]
                if ones.size > 0:
                    c = ones[0]
                    switch_columns(H_new, c, column)
                    col_perm[[c, column]] = col_perm[[column, c]]
                    pivot_ok = True

            assert pivot_ok, "The Pivot operation failed."

        # Clear this column in all rows below
        rows_below = r + 1 + np.nonzero(H_new[r + 1:, column])[0]
        H_new[rows_below, :] ^= H_new[r, :]

    # ---- Step 2: backward elimination -------------------------------------
    for column in range(N - 1, K, -1):      # N-1 down to K+1
        r = column - K
        rows_above = np.nonzero(H_new[:r, column])[0]
        H_new[rows_above, :] ^= H_new[r, :]

    return H_new, col_perm


def generate_g_matrix(code_type):
    """Build the generator matrix G = [I_K | A^T] for an 802.11n LDPC code.

    Returns
    -------
    G : np.ndarray of uint8, shape (K, N)
    H : np.ndarray of uint8, shape (L, N)
        The original parity-check matrix.
    col_perm : np.ndarray of int, shape (N,)
        Column permutation applied during elimination (identity if none).
        G is a generator for ``H[:, col_perm]``.
    """
    P, submatrix_size = get_p_ldpc(code_type)
    H = create_parity_check_matrix(P, submatrix_size)
    L, N = H.shape
    K = N - L

    H_modified, col_perm = use_gaussian_elimination(H)
    A = H_modified[:L, :K]
    G = np.hstack([np.eye(K, dtype=np.uint8), A.T])
    return G, H, col_perm


if __name__ == "__main__":
    import time

    for name in ["648_12", "648_23", "648_34", "648_56",
                 "1296_12", "1296_23", "1296_34", "1296_56",
                 "1944_12", "1944_23", "1944_34", "1944_56"]:
        t = time.time()
        G, H, perm = generate_g_matrix(name)
        L, N = H.shape
        K = N - L

        # Every row of G must be a codeword of the (column-permuted) H
        syndrome = (G.astype(np.int64) @ H[:, perm].astype(np.int64).T) % 2
        assert not syndrome.any(), name
        assert G.shape == (K, N)

        swapped = not np.array_equal(perm, np.arange(N))
        print(f"{name}: G {G.shape}, column swaps: {swapped}, "
              f"{time.time() - t:.2f}s  OK")
