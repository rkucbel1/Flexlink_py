"""
Cause_DC_Offset.py

Receiver DC offset (LO leakage / self-mixing in a direct-conversion receiver):
adds a constant complex value to every sample.

    DC     = sqrt(P_ref * 10^(DCOffset_dBc / 10)) * exp(j * phase)
    Output = Input + DC

P_ref is the reference power the level is quoted against (Defect_Model.py passes
the transmitted waveform power, so DCOffset_dBc is relative to the signal, like
the SNR). The phase is random per ray unless given.
"""

import numpy as np


def cause_dc_offset(signal, level_dbc, ref_power, phase=None, rng=None):
    """
    Parameters
    ----------
    signal : array_like, complex, shape (N,)
    level_dbc : float
        DC power relative to ref_power, in dB.
    ref_power : float
        Reference power (linear).
    phase : float, optional
        Phase of the DC term in radians; random when None.
    rng : numpy.random.Generator or int, optional

    Returns
    -------
    output : ndarray of complex128, shape (N,)
    """
    x = np.asarray(signal, dtype=complex).ravel()
    if phase is None:
        phase = np.random.default_rng(rng).uniform(0, 2 * np.pi)
    dc = np.sqrt(ref_power * 10 ** (level_dbc / 10)) * np.exp(1j * phase)
    return x + dc


if __name__ == "__main__":
    x = np.zeros(1000, dtype=complex)
    y = cause_dc_offset(x, -20.0, 1.0, rng=0)
    print(f"DC power {10 * np.log10(np.mean(np.abs(y) ** 2)):.2f} dBc (requested -20 dBc)")
