"""LDPC_Decode.py

Python port of the MATLAB LDPC decoder (iterative min-sum belief propagation):

    LDPC_Decode(CodeBlk, H)              ->  DecodedDataBlk
    SISO_SPC_Decoder(r)                  ->  l       (single parity check node)
    SISO_Repetition_Decoder(rx, lx)      ->  rx + lx (bit node)

Usage
-----
    from LDPC_Get_H import LDPC_Get_H
    from LDPC_Decode import LDPC_Decode

    H = LDPC_Get_H('648_12')
    DecodedDataBlk = LDPC_Decode(CodeBlk, H)

Input convention (same as the MATLAB code)
------------------------------------------
CodeBlk holds SOFT beliefs, one per codeword bit:
    positive value -> bit 1,   negative value -> bit 0,   larger |value| = more confident.
For example BPSK with 0 -> -1 and 1 -> +1 plus noise. Hard 0/1 bits are NOT a
valid input (a 0 would mean "no information"); a warning is issued if the input
looks like hard bits. Min-sum is scale invariant, so the amplitude (or LLR
scaling) of the beliefs does not matter.

CodeBlk may be
    * 1-D, shape (N,)    -> one codeword,         DecodedDataBlk has shape (K,)
    * 2-D, shape (M, N)  -> M codewords (rows),   DecodedDataBlk has shape (M, K)
where H has shape (L, N) and K = N - L. DecodedDataBlk is uint8 (0/1) and holds
the first K decoded bits (the message part, since the code is systematic).
"""

import warnings

import numpy as np

# Upper bound on the size of the padded (batch, L, max_row_weight) work arrays.
# Large batches are decoded in chunks so memory stays bounded.
_MAX_WORK_ELEMENTS = 20_000_000


# ---------------------------------------------------------------------------
# Helper functions (direct ports of the two MATLAB sub-functions)
# ---------------------------------------------------------------------------
def SISO_SPC_Decoder(r):
    """Extrinsic beliefs for one single-parity-check node.

    r : 1-D array of beliefs entering the check node (one per connected bit).
    Returns l, same length, where l[x] is computed from all entries of r
    except r[x]:
        sign  = -sign(prod(-r_other))
        mag   = min(abs(r_other))
    This is the literal (slow, O(d^2)) version; LDPC_Decode uses an equivalent
    vectorised implementation.
    """
    r = np.asarray(r, dtype=np.float64)
    l = np.zeros_like(r)
    for x in range(len(r)):
        r_other = np.delete(r, x)
        Sign_Other = -np.sign(np.prod(-r_other))
        Mag_Other = np.min(np.abs(r_other))
        l[x] = Sign_Other * Mag_Other
    return l


def SISO_Repetition_Decoder(rx, lx):
    """Updated intrinsic belief = received belief + sum of extrinsic beliefs."""
    return rx + lx


# ---------------------------------------------------------------------------
# Vectorised decoder
# ---------------------------------------------------------------------------
def _check_node_update(Mpad, deg):
    """Vectorised SISO_SPC_Decoder applied to every check node of every message.

    Mpad : (B, L, dmax) float array; row entries beyond a check node's degree
           are padded with +inf (neutral for both the sign product and the min).
    deg  : (L,) int array of check node degrees (each >= 2).
    Returns l_pad, same shape as Mpad (values at padded positions are ignored).
    """
    s = np.sign(Mpad)                         # padded entries -> +1
    a = np.abs(Mpad)                          # padded entries -> +inf
    zero = (s == 0)

    # --- sign of the product of all *other* entries (exact, handles zeros) ---
    z = zero.sum(axis=2, keepdims=True)       # number of exact zeros per row
    s_nz = np.where(zero, 1.0, s)
    P = s_nz.prod(axis=2, keepdims=True)      # product of the non-zero signs
    others_have_zero = (z - zero) > 0
    prod_others = np.where(others_have_zero, 0.0, P * s_nz)

    # -sign(prod(-r_other)) = -(-1)^(deg-1) * prod_others
    parity = np.where((deg - 1) % 2 == 0, -1.0, 1.0)[None, :, None]
    Sign_Other = parity * prod_others

    # --- min of |other entries|: min1 everywhere except at the argmin ----------
    idx_min = a.argmin(axis=2)[..., None]
    min1 = np.take_along_axis(a, idx_min, axis=2)
    a2 = a.copy()
    np.put_along_axis(a2, idx_min, np.inf, axis=2)
    min2 = a2.min(axis=2, keepdims=True)
    at_min = np.arange(a.shape[2])[None, None, :] == idx_min
    Mag_Other = np.where(at_min, min2, min1)

    return Sign_Other * Mag_Other


def _decode_beliefs(CodeBlk, H, NumberOfIterations):
    """Run the iterations; return the final intrinsic beliefs, shape (B, N)."""
    B, N = CodeBlk.shape
    L = H.shape[0]

    # Edge list of the Tanner graph (row-major order: row by row, ascending col)
    rows, cols = np.nonzero(H)
    deg = np.bincount(rows, minlength=L)
    starts = np.cumsum(deg) - deg
    pos = np.arange(rows.size) - starts[rows]     # position of each edge in its row
    dmax = int(deg.max())

    chunk = max(1, _MAX_WORK_ELEMENTS // (L * dmax))
    out = np.empty((B, N), dtype=np.float64)

    for lo in range(0, B, chunk):
        code = CodeBlk[lo:lo + chunk]
        b = code.shape[0]
        offsets = (np.arange(b) * N)[:, None]

        IntrinsicBeliefs = code
        E = np.zeros((b, rows.size))              # extrinsic beliefs on the edges

        for _ in range(NumberOfIterations):
            # Steps 1 and 4: M = (intrinsic beliefs masked by H) - E
            Mpad = np.full((b, L, dmax), np.inf)
            Mpad[:, rows, pos] = IntrinsicBeliefs[:, cols] - E

            # Steps 2 and 5: every check node
            E = _check_node_update(Mpad, deg)[:, rows, pos]

            # Step 4: column sums of E, then the bit-node update
            Total = np.bincount((cols[None, :] + offsets).ravel(),
                                weights=E.ravel(),
                                minlength=b * N).reshape(b, N)
            IntrinsicBeliefs = SISO_Repetition_Decoder(code, Total)

        out[lo:lo + b] = IntrinsicBeliefs
    return out


def LDPC_Decode(CodeBlk, H, NumberOfIterations=15):
    """Decode soft codeword beliefs with min-sum belief propagation.

    Parameters
    ----------
    CodeBlk : array_like, shape (N,) or (M, N)
        Soft beliefs per codeword bit (positive = bit 1, negative = bit 0).
    H : array_like, shape (L, N)
        Parity-check matrix (0/1), e.g. from LDPC_Get_H.
    NumberOfIterations : int, optional
        Number of belief-propagation iterations (default 15, as in MATLAB).

    Returns
    -------
    DecodedDataBlk : np.ndarray of uint8, shape (K,) or (M, K), K = N - L
        Hard decisions for the first K bits: 1 where the final belief is
        positive, else 0 (a belief of exactly 0 decodes to 0).
    """
    H = np.asarray(H)
    if H.ndim != 2:
        raise ValueError(f"H must be 2-D, got {H.ndim}-D")
    L, N = H.shape
    K = N - L
    if K <= 0:
        raise ValueError(f"H has shape {H.shape}; need more columns than rows")
    H = (H != 0)

    deg = H.sum(axis=1)
    if deg.min() < 2:
        raise ValueError("every row of H must have at least two ones")

    code = np.asarray(CodeBlk)
    if code.ndim not in (1, 2):
        raise ValueError(f"CodeBlk must be 1-D (N,) or 2-D (M, N), got {code.ndim}-D")
    if code.shape[-1] != N:
        raise ValueError(f"CodeBlk has {code.shape[-1]} values per codeword but H expects N = {N}")
    if not np.issubdtype(code.dtype, np.number) and code.dtype != np.bool_:
        raise TypeError(f"CodeBlk must be numeric, got dtype {code.dtype}")
    if int(NumberOfIterations) < 1:
        raise ValueError("NumberOfIterations must be at least 1")

    code = code.astype(np.float64)
    if code.size and np.all((code == 0) | (code == 1)):
        warnings.warn(
            "CodeBlk contains only 0s and 1s. LDPC_Decode expects soft beliefs "
            "(positive = bit 1, negative = bit 0), e.g. 2*bits - 1 plus noise; "
            "hard 0/1 bits will decode poorly.",
            stacklevel=2,
        )

    single = (code.ndim == 1)
    code2d = code[None, :] if single else code

    beliefs = _decode_beliefs(code2d, H, int(NumberOfIterations))
    Decoded = (beliefs[:, :K] > 0).astype(np.uint8)
    return Decoded[0] if single else Decoded
