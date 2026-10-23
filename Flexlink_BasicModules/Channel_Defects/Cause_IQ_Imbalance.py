"""
Cause_IQ_Imbalance.py

Python port of the MATLAB function IQImbalance.

Applies a phase and amplitude imbalance between the I and Q rails:

    Phase imbalance:      I_t = I + Q*sin(phase_error)
                          Q_t = Q*cos(phase_error)
    Amplitude imbalance:  I_out = I_gain * I_t
                          Q_out = Q_gain * Q_t

A helper, iq_gains_from_db, converts the JSON setting AmplitudeImbalance_dB
into linear gains: I_gain = 10^(dB/20), Q_gain = 1 (Q is the reference).
"""

import json

import numpy as np


def cause_iq_imbalance(signal, iq_phase_error, i_gain, q_gain):
    """
    Parameters
    ----------
    signal : array_like, complex, shape (N,)
        Input samples.
    iq_phase_error : float
        Phase error between I and Q in radians.
    i_gain, q_gain : float
        Linear gains applied to the I and Q rails.

    Returns
    -------
    output : ndarray of complex128, shape (N,)
    """
    x = np.asarray(signal).ravel()
    i = x.real
    q = x.imag

    # Phase imbalance
    i_temp = i + q * np.sin(iq_phase_error)
    q_temp = q * np.cos(iq_phase_error)

    # Amplitude imbalance
    return i_gain * i_temp + 1j * (q_gain * q_temp)


def iq_gains_from_db(amplitude_imbalance_db):
    """Return (i_gain, q_gain) with I/Q gain ratio = amplitude_imbalance_db."""
    return 10 ** (amplitude_imbalance_db / 20), 1.0


if __name__ == "__main__":
    with open("Channel_Defects.json", "r") as f:
        cfg = json.load(f)
    s = cfg["Settings"]

    i_gain, q_gain = iq_gains_from_db(s["AmplitudeImbalance_dB"])
    phase = s["PhaseImbalance"]

    # Measure image rejection with a complex tone
    n = 4096
    k = 100                                   # tone bin
    x = np.exp(2j * np.pi * k * np.arange(n) / n)
    y = cause_iq_imbalance(x, phase, i_gain, q_gain)
    spec = np.abs(np.fft.fft(y)) ** 2
    irr = 10 * np.log10(spec[k] / spec[-k])
    print(f"Phase {phase} rad, amplitude {s['AmplitudeImbalance_dB']} dB "
          f"-> I_gain {i_gain:.5f}, Q_gain {q_gain:.5f}")
    print(f"Image rejection ratio: {irr:.1f} dB")
