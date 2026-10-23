"""
Get_Multipath_Filter.py

Python port of the MATLAB function GetMultipathFilter, with power
normalization added.

Generates a random N-tap complex Rayleigh FIR channel whose tap variances
decay exponentially with the RMS delay spread:

    variance[n] = exp(-n * Ts / Trms),   n = 0 .. N-1

Each tap is complex Gaussian with E|h[n]|^2 = variance[n]
(variance[n]/2 in each of the real and imaginary parts).

Normalization (default "expected"):
    "expected" - divide by sqrt(sum(variance)) so the AVERAGE channel
                 power over many realizations is 1. Individual
                 realizations still fade, which preserves realistic
                 fading statistics.
    "realized" - divide by the actual energy of this realization so
                 every channel has exactly unit energy (removes
                 flat-fading power variation).
    None       - no normalization (original MATLAB behaviour, but with
                 the variance split between the real and imaginary parts).
"""

import json

import numpy as np


def get_multipath_filter(sample_rate, delay_spread, n_taps,
                         normalize="expected", rng=None):
    """
    Parameters
    ----------
    sample_rate : float
        Sample rate in Hz.
    delay_spread : float
        RMS delay spread in seconds.
    n_taps : int
        Number of FIR taps.
    normalize : {"expected", "realized", None}
        Power normalization mode (see module docstring).
    rng : numpy.random.Generator or int, optional
        Random generator or seed for reproducible channels.

    Returns
    -------
    fir_taps : ndarray of complex128, shape (n_taps,)
    """
    rng = np.random.default_rng(rng)

    ts = 1.0 / sample_rate                  # sampling period in seconds
    trms = delay_spread                     # spread in seconds
    n = np.arange(n_taps)
    exp_variance = np.exp(-n * ts / trms)

    # Complex Gaussian taps with E|h[n]|^2 = exp_variance[n]
    sigma = np.sqrt(exp_variance / 2.0)
    fir_taps = sigma * (rng.standard_normal(n_taps)
                        + 1j * rng.standard_normal(n_taps))

    if normalize == "expected":
        fir_taps /= np.sqrt(np.sum(exp_variance))
    elif normalize == "realized":
        fir_taps /= np.sqrt(np.sum(np.abs(fir_taps) ** 2))
    elif normalize is not None:
        raise ValueError(f"Unknown normalize mode: {normalize!r}")

    return fir_taps


if __name__ == "__main__":
    with open("Channel_Defects.json", "r") as f:
        cfg = json.load(f)
    s = cfg["Settings"]

    taps = get_multipath_filter(s["SampleRate"], s["DelaySpread"],
                                s["NumberOfTaps"])
    print(f"{len(taps)} taps generated")
    print(np.round(taps[:5], 4))
    print(f"Energy of this realization: {np.sum(np.abs(taps) ** 2):.3f}")
