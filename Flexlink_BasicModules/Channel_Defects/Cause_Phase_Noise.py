"""
Cause_Phase_Noise.py

Oscillator phase noise from a spectral (PSD) mask.

The mask is given as single-sideband phase noise levels L(f) in dBc/Hz at a
list of offset frequencies, as found on a synthesizer datasheet. Between the
given points the level is interpolated linearly in dB against log-frequency;
below the first point the first level is held, above the last point the last
level is held (noise floor).

Method:
  1. White Gaussian noise is shaped in the frequency domain so that the
     phase process phi[n] has a two-sided PSD equal to L(|f|) (rad^2/Hz).
  2. The signal is multiplied by exp(j*phi[n]).

The integrated RMS phase error over the simulated band
(sample_rate/FFT length up to sample_rate/2) is
    rms = sqrt( 2 * integral L(f) df )
and can be checked with integrated_phase_error_deg().

Each call draws a new, independent phase noise realization.
"""

import json

import numpy as np


def _mask_linear(f, offsets_hz, levels_dbchz):
    """L(f) as a linear ratio (1/Hz) for positive frequencies f."""
    logf = np.log10(np.asarray(offsets_hz, dtype=float))
    lev = np.asarray(levels_dbchz, dtype=float)
    f = np.maximum(np.asarray(f, dtype=float), 1e-12)
    lev_f = np.interp(np.log10(f), logf, lev)       # holds end values outside
    return 10 ** (lev_f / 10)


def integrated_phase_error_deg(offsets_hz, levels_dbchz, f_low, f_high,
                               points=20000):
    """Integrated RMS phase error in degrees between f_low and f_high (Hz)."""
    f = np.logspace(np.log10(f_low), np.log10(f_high), points)
    var = 2 * np.trapezoid(_mask_linear(f, offsets_hz, levels_dbchz), f)
    return np.degrees(np.sqrt(var))


def cause_phase_noise(signal, sample_rate, offsets_hz, levels_dbchz,
                      rng=None, return_phase=False):
    """
    Parameters
    ----------
    signal : array_like, complex, shape (N,)
        Input samples.
    sample_rate : float
        Sample rate in Hz.
    offsets_hz : array_like
        Offset frequencies of the mask points in Hz (increasing, > 0).
    levels_dbchz : array_like
        Phase noise L(f) at those offsets in dBc/Hz.
    rng : numpy.random.Generator or int, optional
        Random generator or seed for reproducible phase noise.
    return_phase : bool
        If True, also return the phase noise phi[n] in radians.

    Returns
    -------
    output : ndarray of complex128, shape (N,)
    phase : ndarray of float, shape (N,) (only if return_phase is True)
    """
    rng = np.random.default_rng(rng)
    offsets_hz = np.asarray(offsets_hz, dtype=float).ravel()
    levels_dbchz = np.asarray(levels_dbchz, dtype=float).ravel()
    if offsets_hz.shape != levels_dbchz.shape:
        raise ValueError("offsets_hz and levels_dbchz must have the same length")
    if np.any(offsets_hz <= 0) or np.any(np.diff(offsets_hz) <= 0):
        raise ValueError("offsets_hz must be positive and increasing")

    x = np.asarray(signal, dtype=complex).ravel()
    n = len(x)
    if n == 0:
        return (x.copy(), np.empty(0)) if return_phase else x.copy()

    # FFT length: power of two, at least 2N so circular wrap-around does
    # not correlate the start and end of the block
    m = 1 << int(np.ceil(np.log2(2 * n)))
    f = np.abs(np.fft.fftfreq(m, d=1 / sample_rate))

    # Shaping filter: white noise (PSD 1/fs) -> PSD L(|f|)
    shaping = np.sqrt(_mask_linear(f, offsets_hz, levels_dbchz) * sample_rate)
    shaping[0] = 0.0                                 # no DC phase term

    white = rng.standard_normal(m)
    phase = np.fft.ifft(np.fft.fft(white) * shaping).real[:n]

    output = x * np.exp(1j * phase)
    if return_phase:
        return output, phase
    return output


if __name__ == "__main__":
    with open("Channel_Defects.json", "r") as f:
        cfg = json.load(f)
    s = cfg["Settings"]
    fs = s["SampleRate"]
    offs, levs = s["PhaseNoise_Offsets_Hz"], s["PhaseNoise_Levels_dBcHz"]

    n = 200000
    y, phi = cause_phase_noise(np.ones(n, dtype=complex), fs, offs, levs,
                               rng=0, return_phase=True)
    m = 1 << int(np.ceil(np.log2(2 * n)))
    expected = integrated_phase_error_deg(offs, levs, fs / m, fs / 2)
    print(f"Integrated RMS phase error: expected {expected:.2f} deg, "
          f"this realization {np.degrees(np.std(phi)):.2f} deg")
    print("Approx. EVM limit from phase noise: "
          f"{20 * np.log10(np.radians(expected)):.1f} dB")
