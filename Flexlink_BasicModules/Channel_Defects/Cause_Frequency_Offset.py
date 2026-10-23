"""
Cause_Frequency_Offset.py

Python port of the MATLAB frequency-offset step:

    OffsetSignal = exp(1i*2*pi*(1:length(TX_Output))*FrequencyOffset/SampleRate);
    TX_Output    = TX_Output .* OffsetSignal;

Multiplies the input by a complex exponential, shifting the whole spectrum
by FrequencyOffset Hz (positive = shifted up).

Unlike the MATLAB code (which starts at n = 1), the rotation starts at
n = 0, so the first sample is unchanged. The difference is a constant phase
of 2*pi*FrequencyOffset/SampleRate (about 0.0003 rad for 2 kHz at 40 MHz).
An optional initial_phase can be given instead.
"""

import json

import numpy as np


def cause_frequency_offset(signal, frequency_offset, sample_rate,
                           initial_phase=0.0):
    """
    Parameters
    ----------
    signal : array_like, complex, shape (N,)
        Input samples.
    frequency_offset : float
        Carrier frequency offset in Hz (may be negative).
    sample_rate : float
        Sample rate in Hz.
    initial_phase : float
        Phase of the rotation at the first sample, in radians.

    Returns
    -------
    output : ndarray of complex128, shape (N,)
    """
    x = np.asarray(signal, dtype=complex).ravel()
    n = np.arange(len(x))
    rotation = np.exp(1j * (2 * np.pi * frequency_offset / sample_rate * n
                            + initial_phase))
    return x * rotation


if __name__ == "__main__":
    with open("Channel_Defects.json", "r") as f:
        cfg = json.load(f)
    s = cfg["Settings"]
    fo, fs = s["FrequencyOffset"], s["SampleRate"]

    # Check: a DC input should come out as a tone at exactly fo
    n = 400000
    y = cause_frequency_offset(np.ones(n), fo, fs)
    phase_step = np.angle(np.mean(y[1:] * np.conj(y[:-1])))
    print(f"Requested offset {fo} Hz, measured "
          f"{phase_step * fs / (2 * np.pi):.3f} Hz")
