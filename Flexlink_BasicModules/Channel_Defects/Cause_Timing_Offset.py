"""
Cause_Timing_Offset.py

Python port of the MATLAB function Cause_TimingOffset, restricted to
whole-sample (integer) offsets.

MATLAB (1-based):
    InputIndex  = 1:length(input)
    OutputIndex = 1 + SampleDelay : 1 : length(input)
    output      = interp1(InputIndex, input, OutputIndex, 'spline')

With an integer SampleDelay, interpolation at the input points returns the
input samples exactly, so the function reduces to:

    SampleDelay > 0 : drop the first SampleDelay samples
                      (signal arrives EARLIER; output is shorter).
    SampleDelay < 0 : prepend |SampleDelay| samples spline-extrapolated
                      before the first input sample, as MATLAB's
                      interp1(..., 'spline') does
                      (signal is DELAYED; output is longer).
    SampleDelay = 0 : output equals input.

Fractional offsets are rejected with a ValueError.
"""

import json

import numpy as np
from scipy.interpolate import CubicSpline


def cause_timing_offset(signal, sample_delay):
    """
    Parameters
    ----------
    signal : array_like, real or complex, shape (N,)
        Input samples.
    sample_delay : int
        Timing offset in whole samples (may be negative). A float with no
        fractional part (e.g. -1.0) is accepted.

    Returns
    -------
    output : ndarray, shape (N - sample_delay,)
    """
    if float(sample_delay) != int(sample_delay):
        raise ValueError(
            f"sample_delay must be a whole number of samples, got {sample_delay}")
    d = int(sample_delay)

    x = np.asarray(signal).ravel()
    n = len(x)

    if d >= 0:
        return x[d:].copy()

    # Negative offset: extrapolate |d| samples before the first input
    # sample with a not-a-knot cubic spline (matches MATLAB interp1 'spline').
    spline = CubicSpline(np.arange(n), x, bc_type="not-a-knot",
                         extrapolate=True)
    lead = spline(np.arange(d, 0))
    return np.concatenate([lead, x])


if __name__ == "__main__":
    with open("Channel_Defects.json", "r") as f:
        cfg = json.load(f)
    d = cfg["Settings"]["Sample_Offset"]

    rng = np.random.default_rng(0)
    x = rng.standard_normal(16) + 1j * rng.standard_normal(16)
    y = cause_timing_offset(x, d)
    print(f"Sample_Offset = {d}: input length {len(x)}, "
          f"output length {len(y)}")
