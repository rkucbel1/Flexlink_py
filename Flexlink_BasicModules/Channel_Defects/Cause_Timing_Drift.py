"""
Cause_Timing_Drift.py

Python port of the MATLAB function Cause_TimingDrift.

Simulates a sample-clock frequency error by resampling the input with a
step of (1 + Drift_ppm/1e6) samples, using cubic spline interpolation
(equivalent to MATLAB interp1(..., 'spline'), not-a-knot end conditions).

MATLAB (1-based):
    SampleStep  = 1 + Drift_ppm/1e6
    InputIndex  = 1:length(input)
    OutputIndex = 1 + SampleStep : SampleStep : length(input)

Python (0-based) version:
    input index  = 0 .. N-1
    output index = 0, SampleStep, 2*SampleStep, ... <= N-1

    Unlike the MATLAB code, the output starts at the first input sample
    (index 0), so zero drift returns the input unchanged.

Behaviour:
    Drift_ppm > 0 : receiver clock runs slow relative to the signal; the
                    sampling points slip forward and the output is shorter.
    Drift_ppm < 0 : sampling points slip backward; the output is longer
                    in samples per unit of input.

Drift_ppm must be a whole number (integer ppm).
"""

import json
import math

import numpy as np
from scipy.interpolate import CubicSpline


def cause_timing_drift(signal, drift_ppm):
    """
    Parameters
    ----------
    signal : array_like, real or complex, shape (N,)
        Input samples.
    drift_ppm : int
        Sample-clock drift in parts per million (may be negative). A float
        with no fractional part (e.g. 10.0) is accepted.

    Returns
    -------
    output : ndarray
    """
    if float(drift_ppm) != int(drift_ppm):
        raise ValueError(
            f"drift_ppm must be a whole number, got {drift_ppm}")
    drift_ppm = int(drift_ppm)

    x = np.asarray(signal).ravel()
    n = len(x)
    step = 1.0 + drift_ppm / 1e6

    # Sampling points 0, step, 2*step, ... <= N-1 (starts at the first sample)
    n_out = math.floor((n - 1) / step + 1e-10) + 1
    if n == 0:
        return np.empty(0, dtype=x.dtype)
    output_index = step * np.arange(n_out)

    spline = CubicSpline(np.arange(n), x, bc_type="not-a-knot",
                         extrapolate=True)
    return spline(output_index)


if __name__ == "__main__":
    with open("Channel_Defects.json", "r") as f:
        cfg = json.load(f)
    ppm = cfg["Settings"]["Drift_ppm"]

    rng = np.random.default_rng(0)
    x = rng.standard_normal(100000) + 1j * rng.standard_normal(100000)
    y = cause_timing_drift(x, ppm)
    print(f"Drift_ppm = {ppm}: input length {len(x)}, "
          f"output length {len(y)}")
