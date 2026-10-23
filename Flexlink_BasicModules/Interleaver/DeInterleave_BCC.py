"""
DeInterleave_BCC.py

Python port of the MATLAB InterleaveBCC() function (deinterleave direction
only, i.e. direction ~= 1) for the BCC code path.

The interleaver index vectors (InputLen = 40 and 140) are defined once in
Interleave_BCC.py and reused here, so the two scripts can never get out of
step. Keep both files in the same folder.
"""

import numpy as np

from Interleave_BCC import _INDEX  # 0-based index vectors keyed by InputLen


def DeInterleave_BCC(MatrixToDeInterleave, InputLen):
    """Deinterleave code block(s) using the BCC interleaver.

    Equivalent to the MATLAB loop (direction ~= 1):
        output(1, InvIndex(1, i)) = input(1, i)

    Parameters
    ----------
    MatrixToDeInterleave : array-like
        Shape (InputLen,) for a single code block, or (N, InputLen) to
        deinterleave N code blocks at once (each row independently).
    InputLen : int
        Code block length: 40 or 140.

    Returns
    -------
    CodeBlkMatrix : np.ndarray
        Deinterleaved data, same shape as MatrixToDeInterleave.
    """
    if InputLen not in _INDEX:
        raise ValueError(f"InputLen must be 40 or 140, got {InputLen}")

    x = np.asarray(MatrixToDeInterleave)
    if x.shape[-1] != InputLen:
        raise ValueError(f"Last dimension of input is {x.shape[-1]}, expected InputLen={InputLen}")

    # output[..., idx[i]] = input[..., i]
    CodeBlkMatrix = np.empty_like(x)
    CodeBlkMatrix[..., _INDEX[InputLen]] = x
    return CodeBlkMatrix


if __name__ == "__main__":
    # Self-test: deinterleave(interleave(x)) must return x
    from Interleave_BCC import Interleave_BCC

    rng = np.random.default_rng(0)
    for n in (40, 140):
        data = rng.integers(0, 2, size=(3, n))
        restored = DeInterleave_BCC(Interleave_BCC(data, n), n)
        print(f"InputLen={n}: round trip {'OK' if np.array_equal(restored, data) else 'FAILED'}")
