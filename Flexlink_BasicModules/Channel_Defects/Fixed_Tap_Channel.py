"""
Fixed_Tap_Channel.py

FIR multipath channel (MultipathModel = "FIR").

Python port of the MATLAB calling code:

    FIR_Taps  = GetMultipathFilter(SampleRate, DelaySpread, N);
    TX_Output = filter(FIR_Taps, 1, TX_Output);

Draws a random exponentially decaying N-tap FIR channel from
Get_Multipath_Filter.py and filters the input with it (causal, same length
as the input, like MATLAB filter). The channel is fixed for the whole call
(no Doppler); each call draws a new channel.

Unlike the MATLAB code, the output is NOT rescaled to unit variance. The
taps are normalized to an expected power of 1, so the output power equals
the input power on average, while individual channels can fade.
"""

import json

import numpy as np

from Get_Multipath_Filter import get_multipath_filter


def fixed_tap_channel(sample_rate, delay_spread, n_taps, signal, rng=None,
                      return_taps=False):
    """
    Parameters
    ----------
    sample_rate : float
        Sample rate in Hz.
    delay_spread : float
        RMS delay spread in seconds.
    n_taps : int
        Number of FIR taps.
    signal : array_like, complex, shape (N,)
        Input samples.
    rng : numpy.random.Generator or int, optional
        Random generator or seed for a reproducible channel.
    return_taps : bool
        If True, also return the FIR taps used.

    Returns
    -------
    output : ndarray of complex128, shape (N,)
    taps : ndarray (only if return_taps is True)
    """
    x = np.asarray(signal, dtype=complex).ravel()
    taps = get_multipath_filter(sample_rate, delay_spread, n_taps, rng=rng)

    # Equivalent of MATLAB filter(FIR_Taps, 1, x): causal, same length
    output = np.convolve(x, taps)[:len(x)]

    if return_taps:
        return output, taps
    return output


if __name__ == "__main__":
    with open("Channel_Defects.json", "r") as f:
        cfg = json.load(f)
    s = cfg["Settings"]

    rng = np.random.default_rng(0)
    x = (rng.standard_normal(10000)
         + 1j * rng.standard_normal(10000)) / np.sqrt(2)
    y, taps = fixed_tap_channel(s["SampleRate"], s["DelaySpread"],
                                s["NumberOfTaps"], x, rng=rng,
                                return_taps=True)
    print(f"FIR channel: {len(taps)} taps, input length {len(x)}, "
          f"output length {len(y)}, "
          f"output/input power {np.mean(abs(y)**2) / np.mean(abs(x)**2):.3f}")
