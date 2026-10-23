"""LDPC_Encode.py

Python port of the MATLAB encoder:

    function CodeBlk = LDPC_Encode(DataBlk, G)
        CodeBlk = mod(DataBlk*G, 2);
    end

Usage
-----
    from LDPC_Get_G import LDPC_Get_G
    from LDPC_Encode import LDPC_Encode

    G = LDPC_Get_G('648_12', dtype=np.float32)   # load once, reuse
    CodeBlk = LDPC_Encode(DataBlk, G)

DataBlk may be
    * 1-D, shape (K,)    -> one message,         CodeBlk has shape (N,)
    * 2-D, shape (M, K)  -> M messages (rows),   CodeBlk has shape (M, N)
where G has shape (K, N). CodeBlk is uint8 with values 0/1. Because G is
systematic ([I_K | A^T]), the first K bits of each codeword are the message.
"""

import numpy as np


def LDPC_Encode(DataBlk, G):
    """Encode message bits with generator matrix G over GF(2).

    Parameters
    ----------
    DataBlk : array_like, shape (K,) or (M, K)
        Message bits (0/1). Each row of a 2-D input is one message.
    G : np.ndarray, shape (K, N)
        Generator matrix with 0/1 entries (any numeric dtype; passing it as
        float32 avoids a conversion on every call, e.g.
        ``LDPC_Get_G(type, dtype=np.float32)``).

    Returns
    -------
    CodeBlk : np.ndarray of uint8, shape (N,) or (M, N)
        ``(DataBlk @ G) mod 2``.
    """
    G = np.asarray(G)
    if G.ndim != 2:
        raise ValueError(f"G must be 2-D, got {G.ndim}-D")
    K, N = G.shape

    data = np.asarray(DataBlk)
    if data.ndim not in (1, 2):
        raise ValueError(f"DataBlk must be 1-D (K,) or 2-D (M, K), got {data.ndim}-D")
    if data.shape[-1] != K:
        raise ValueError(
            f"DataBlk has {data.shape[-1]} bits per message but G expects K = {K}"
        )

    # Messages must be bits. (MATLAB's mod() would silently accept anything.)
    if data.dtype != np.bool_ and not np.all((data == 0) | (data == 1)):
        raise ValueError("DataBlk must contain only 0 and 1")

    # float32 uses fast BLAS matmul and is exact as long as every dot-product
    # (at most K ones) stays below 2**24; otherwise fall back to int64.
    work = np.float32 if K < 2**24 else np.int64
    prod = data.astype(work, copy=False) @ G.astype(work, copy=False)

    # Parity: sums are small non-negative integers, so cast then take the low bit
    return (prod.astype(np.int64) & 1).astype(np.uint8)
