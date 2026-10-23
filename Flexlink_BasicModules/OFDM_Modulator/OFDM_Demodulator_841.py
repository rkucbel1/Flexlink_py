"""
OFDM_Demodulator_841.py

Python port of a MATLAB OFDM demodulator.

Takes a 20 MHz complex sample sequence (previously decimated from 40 MHz),
removes the cyclic prefix of each OFDM symbol, runs a 1024-point FFT and maps
the FFT bins back onto an 841 x NumOfdmSymbols resource grid.
"""

import numpy as np

NUM_SUBCARRIERS = 841
FFT_SIZE = 1024
CP_LENGTH = 116
OFDM_SYMBOL_LENGTH = CP_LENGTH + FFT_SIZE
ZERO_PAD = 400


def ofdm_demodulator_841(input_sequence, start_time):
    """
    OFDM demodulator.

    Parameters
    ----------
    input_sequence : array_like, 1-D
        Complex samples at 20 MHz.
    start_time : int
        0-based sample index where the first OFDM symbol (its cyclic prefix)
        starts. Must be <= 400 so the last symbol fits in the zero padding.

    Returns
    -------
    numpy.ndarray, shape (841, NumOfdmSymbols)
        Complex resource grid. Row index = subcarrier, column = OFDM symbol.
    """
    # ------------------------------------------------------------------
    # 1. Zero pad to prevent overflow for start_time
    # ------------------------------------------------------------------
    input_sequence = np.asarray(input_sequence, dtype=complex).ravel()
    input_sequence = np.concatenate([input_sequence, np.zeros(ZERO_PAD, dtype=complex)])

    num_ofdm_symbols = len(input_sequence) // OFDM_SYMBOL_LENGTH
    resource_grid = np.zeros((NUM_SUBCARRIERS, num_ofdm_symbols), dtype=complex)

    # Subcarrier <-> FFT bin mapping (0-based)
    pos_index_range = np.arange(0, 421)      # FFT bins 0:420
    neg_index_range = np.arange(604, 1024)   # FFT bins 604:1023
    pos_range = np.arange(420, 841)          # grid rows 420:840
    neg_range = np.arange(0, 420)            # grid rows 0:419

    # ------------------------------------------------------------------
    # 2. OFDM demodulation
    # ------------------------------------------------------------------
    fft_time = int(start_time)
    for i in range(num_ofdm_symbols):
        fft_time += CP_LENGTH
        early_sample = fft_time
        end = early_sample + FFT_SIZE
        if early_sample < 0 or end > len(input_sequence):
            raise IndexError(
                f"OFDM symbol {i} needs samples {early_sample}..{end - 1} but the padded "
                f"input has only {len(input_sequence)}; start_time = {start_time} is out of range.")

        samples = input_sequence[early_sample:end]
        fft_output = np.fft.fft(samples, FFT_SIZE)

        resource_grid[pos_range, i] = fft_output[pos_index_range]
        resource_grid[neg_range, i] = fft_output[neg_index_range]

        fft_time += FFT_SIZE

    # ------------------------------------------------------------------
    # 3. Scaling
    # ------------------------------------------------------------------
    resource_grid = resource_grid / np.sqrt(FFT_SIZE)
    resource_grid = resource_grid / np.sqrt(2)

    return resource_grid


if __name__ == "__main__":
    # Round trip: modulate at 40 MHz, decimate by 2, demodulate at 20 MHz.
    # The 841 subcarriers span about +-8.2 MHz, so plain decimation by 2
    # (20 MHz Nyquist band = +-10 MHz) does not alias.
    from OFDM_Modulator_841 import ofdm_modulator_841

    rng = np.random.default_rng(0)
    num_symbols = 6
    levels = np.array([-3, -1, 1, 3])
    grid = (rng.choice(levels, (NUM_SUBCARRIERS, num_symbols))
            + 1j * rng.choice(levels, (NUM_SUBCARRIERS, num_symbols))) / np.sqrt(10)

    for fs_tx, decim in ((40e6, 2), (20e6, 1)):
        tx = ofdm_modulator_841(grid, fs_tx)[::decim]
        rx = ofdm_demodulator_841(tx, 0)[:, :num_symbols]
        gain = np.vdot(grid, rx) / np.vdot(grid, grid)
        err = rx / gain - grid
        print(f"TX at {fs_tx / 1e6:.0f} MHz -> 20 MHz: symbols out = {rx.shape[1]}, "
              f"gain = {abs(gain):.4f} ({20 * np.log10(abs(gain)):+.2f} dB), "
              f"max error after gain removal = {np.max(np.abs(err)):.2e}")
