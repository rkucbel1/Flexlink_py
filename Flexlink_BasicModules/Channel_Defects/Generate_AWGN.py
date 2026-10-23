"""
Generate_AWGN.py

Python port of the MATLAB function Generate_AWGN.

Returns complex white Gaussian noise (the noise only; the caller adds it to
the signal) whose power is set relative to the measured mean-square power
of the input:

    MeanSquare = mean(|Input|^2)
    NoisePower = MeanSquare / 10^(SNR/10)
    Noise      = sqrt(NoisePower) * sqrt(1/2) * (randn + 1j*randn)

The MATLAB code uses 0.70711 for sqrt(1/2); this version uses the exact
value (difference about 1e-6 in amplitude).
"""

import json

import numpy as np


def generate_awgn(signal, snr_db, rng=None):
    """
    Parameters
    ----------
    signal : array_like, real or complex, shape (N,)
        Signal whose mean-square power sets the noise level.
    snr_db : float
        Desired signal-to-noise ratio in dB.
    rng : numpy.random.Generator or int, optional
        Random generator or seed for reproducible noise.

    Returns
    -------
    noise : ndarray of complex128, shape (N,)
    """
    rng = np.random.default_rng(rng)
    x = np.asarray(signal).ravel()
    n = len(x)

    mean_square = np.mean(np.abs(x) ** 2)
    noise_power = mean_square / (10 ** (snr_db / 10))
    std_noise = np.sqrt(noise_power)

    return std_noise * np.sqrt(0.5) * (rng.standard_normal(n)
                                       + 1j * rng.standard_normal(n))


if __name__ == "__main__":
    with open("Channel_Defects.json", "r") as f:
        cfg = json.load(f)
    snr = cfg["Settings"]["SNR_dB"]

    rng = np.random.default_rng(0)
    x = rng.standard_normal(100000) + 1j * rng.standard_normal(100000)
    noise = generate_awgn(x, snr, rng=rng)
    measured = 10 * np.log10(np.mean(np.abs(x) ** 2)
                             / np.mean(np.abs(noise) ** 2))
    print(f"Requested SNR {snr} dB, measured {measured:.2f} dB")
