"""
Interleave_BCC.py

Python port of the MATLAB InterleaveBCC() function (interleave direction
only, i.e. direction = 1) for the BCC code path.

The interleaver grids were made by scrambling the numbers 1..N with MATLAB's
randperm(). They are stored here exactly as in the MATLAB code (1-based) and
converted to 0-based indices for NumPy.
"""

import numpy as np

# 1-based index vector for InputLen = 140 (copied from the MATLAB code)
_INV_INDEX_140 = [
    62,  88,  45, 106, 127,  37, 121,  85,  83,  54,
   113,  99,   1,  12,  19,  23, 111,  79,  96,  59,
    97,  94,  55,   7,  31, 112,  89,   5,  34,  47,
   137,   8,  44,  64,  72, 110, 117,  14, 136, 124,
    40,  66,  76,  77,  78,  81,  93,  61, 118, 115,
    65,  11, 125,  73,  38,  25, 134,  50, 123,  20,
    15,  53, 109, 138,  75,   6,  41, 105,  52,  92,
   128,   9,  10,  27,  82,  60,  84,  32,  74,  48,
    91,  43,  56, 135,  67,  28, 100,  57,  29,  17,
   133,  98, 102,  33,  46, 116,   3, 103,  49,   2,
    68,  35, 130, 140, 107,  18, 126,  30,  39,  24,
   132,  22,  42,  95,  70, 131, 101,   4, 104, 120,
   129,  13,  69,  63,  21,  36, 122, 114,  90, 108,
    16,  58, 139,  26, 119,  87,  86,  71,  51,  80,
]

# 1-based index vector for InputLen = 40 (copied from the MATLAB code)
_INV_INDEX_40 = [
    14,  20,  16,   5,   1,  31,  11,  12,  19,   7,
    40,  27,  13,  35,  25,  22,  33,  23,  36,  10,
    34,   4,   6,  17,  38,  24,   3,  39,   9,   2,
    21,   8,  29,  30,  18,  32,  28,  26,  37,  15,
]

# 0-based versions, keyed by InputLen
_INDEX = {
    140: np.array(_INV_INDEX_140, dtype=np.int64) - 1,
    40: np.array(_INV_INDEX_40, dtype=np.int64) - 1,
}

# Sanity check: each vector must be a permutation of 0..N-1
for _n, _idx in _INDEX.items():
    if _idx.size != _n or not np.array_equal(np.sort(_idx), np.arange(_n)):
        raise RuntimeError(f"BCC interleaver vector for length {_n} is not a valid permutation")


def Interleave_BCC(CodeBlkMatrix, InputLen):
    """Interleave code block(s) using the BCC interleaver.

    Equivalent to the MATLAB loop (direction = 1):
        output(1, i) = input(1, InvIndex(1, i))

    Parameters
    ----------
    CodeBlkMatrix : array-like
        Shape (InputLen,) for a single code block, or (N, InputLen) to
        interleave N code blocks at once (each row is interleaved
        independently).
    InputLen : int
        Code block length: 40 or 140.

    Returns
    -------
    InvCodeBlkMatrix : np.ndarray
        Interleaved data, same shape as CodeBlkMatrix.
    """
    if InputLen not in _INDEX:
        raise ValueError(f"InputLen must be 40 or 140, got {InputLen}")

    x = np.asarray(CodeBlkMatrix)
    if x.shape[-1] != InputLen:
        raise ValueError(f"Last dimension of input is {x.shape[-1]}, expected InputLen={InputLen}")

    # output[..., i] = input[..., idx[i]]
    InvCodeBlkMatrix = x[..., _INDEX[InputLen]]
    return InvCodeBlkMatrix


if __name__ == "__main__":
    # Self-test: interleaving a ramp must return the (0-based) index vector
    for n in (40, 140):
        ramp = np.arange(n)
        out = Interleave_BCC(ramp, n)
        ok = np.array_equal(out, _INDEX[n]) and np.array_equal(np.sort(out), ramp)
        print(f"InputLen={n}: {'OK' if ok else 'FAILED'}, first 8 outputs = {out[:8]}")
