"""
Add_CW_Interferer.py

Continuous-wave (single tone) interferer at a fixed baseband frequency:

    CW     = sqrt(P_ref * 10^(CW_Level_dBc / 10)) * exp(j * (2 pi f n / Fs + phase))
    Output = Input + CW

f is the tone's offset from the receiver's centre frequency (Hz, may be negative).
P_ref is the reference power (Defect_Model.py passes the transmitted waveform power,
so CW_Level_dBc is relative to the signal, like the SNR). The phase is random per ray
unless given.
"""

import numpy as np


def add_cw_interferer(signal, frequency, level_dbc, ref_power, sample_rate, phase=None, rng=None):
    """
    Parameters
    ----------
    signal : array_like, complex, shape (N,)
    frequency : float
        Tone frequency in Hz relative to the centre frequency.
    level_dbc : float
        Tone power relative to ref_power, in dB.
    ref_power : float
        Reference power (linear).
    sample_rate : float
        Sample rate in Hz.
    phase : float, optional
        Starting phase in radians; random when None.
    rng : numpy.random.Generator or int, optional

    Returns
    -------
    output : ndarray of complex128, shape (N,)
    """
    x = np.asarray(signal, dtype=complex).ravel()
    if phase is None:
        phase = np.random.default_rng(rng).uniform(0, 2 * np.pi)
    n = np.arange(len(x))
    amp = np.sqrt(ref_power * 10 ** (level_dbc / 10))
    return x + amp * np.exp(1j * (2 * np.pi * frequency * n / sample_rate + phase))


if __name__ == "__main__":
    fs, f = 40e6, 1.1e6
    y = add_cw_interferer(np.zeros(400000, dtype=complex), f, -10.0, 1.0, fs, rng=0)
    step = np.angle(np.mean(y[1:] * np.conj(y[:-1])))
    print(f"Tone {step * fs / (2 * np.pi):.1f} Hz (requested {f:.1f}), "
          f"power {10 * np.log10(np.mean(np.abs(y) ** 2)):.2f} dBc (requested -10)")
