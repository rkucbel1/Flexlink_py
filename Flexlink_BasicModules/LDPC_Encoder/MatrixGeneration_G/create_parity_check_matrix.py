"""Python port of CreateParityCheckMatrix.m"""

import numpy as np


def create_parity_check_matrix(M, dimension):
    """Expand a prototype matrix into the full binary parity-check matrix H.

    Parameters
    ----------
    M : array_like of int, shape (mb, nb)
        Prototype matrix. Entries < 0 are all-zero submatrices; entries >= 0
        are cyclic shifts of the identity.
    dimension : int
        Submatrix (circulant) size Z.

    Returns
    -------
    H : np.ndarray of uint8, shape (mb * Z, nb * Z)

    Notes
    -----
    MATLAB's ``circshift(eye(Z), s, 2)`` moves each column ``s`` places to
    the right (wrapping around), so row ``i`` of the submatrix has its 1 in
    column ``(i + s) % Z``. That is what is built here, directly, without
    forming the intermediate identity matrices.
    """
    M = np.asarray(M)
    mb, nb = M.shape
    Z = int(dimension)

    H = np.zeros((mb * Z, nb * Z), dtype=np.uint8)
    idx = np.arange(Z)

    for row in range(mb):
        for col in range(nb):
            shift = M[row, col]
            if shift >= 0:
                H[row * Z + idx, col * Z + (idx + shift) % Z] = 1
    return H


if __name__ == "__main__":
    from get_p_ldpc import get_p_ldpc

    def reference(M, Z):
        """Literal translation of the MATLAB code, for cross-checking."""
        H = np.zeros((M.shape[0] * Z, M.shape[1] * Z), dtype=np.uint8)
        for r in range(M.shape[0]):
            for c in range(M.shape[1]):
                if M[r, c] >= 0:
                    blk = np.roll(np.eye(Z, dtype=np.uint8), M[r, c], axis=1)
                    H[r * Z:(r + 1) * Z, c * Z:(c + 1) * Z] = blk
        return H

    # Tiny hand-checkable case: Z=4, shift 1 -> 1s at (0,1),(1,2),(2,3),(3,0)
    small = create_parity_check_matrix([[1, -1]], 4)
    assert small[:, :4].tolist() == [[0, 1, 0, 0], [0, 0, 1, 0],
                                     [0, 0, 0, 1], [1, 0, 0, 0]]
    assert not small[:, 4:].any()

    for name in ["648_12", "648_23", "648_34", "648_56",
                 "1296_12", "1296_23", "1296_34", "1296_56",
                 "1944_12", "1944_23", "1944_34", "1944_56"]:
        P, Z = get_p_ldpc(name)
        H = create_parity_check_matrix(P, Z)
        assert np.array_equal(H, reference(P, Z)), name
        # every submatrix contributes exactly one 1 per row of H
        assert np.array_equal(H.sum(axis=1), np.repeat((P >= 0).sum(axis=1), Z))
        print(f"{name}: H shape {H.shape}  OK")
