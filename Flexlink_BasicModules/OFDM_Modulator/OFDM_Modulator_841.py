"""
OFDM_Modulator_841.py

Python port of a MATLAB OFDM modulator.

Takes an 841 x NumOfdmSymbols resource grid, maps subcarriers onto an IFFT,
adds a cyclic prefix to each OFDM symbol, concatenates the symbols and
scales the output by sqrt(IFFT_Size).
"""

import numpy as np


def ofdm_modulator_841(resource_grid, sample_rate):
    """
    OFDM modulator.

    Parameters
    ----------
    resource_grid : array_like, shape (841, NumOfdmSymbols)
        Complex resource grid, normalized to unit average power.
        Row index = subcarrier, column = OFDM symbol.
        A 1-D array of length 841 is treated as a single OFDM symbol.
    sample_rate : float
        Sample rate in Hz. 20e6 -> 1024-point IFFT, otherwise 2048-point.

    Returns
    -------
    numpy.ndarray, shape (NumOfdmSymbols * OfdmSymbolLength,)
        Complex time-domain output samples.
    """
    # ------------------------------------------------------------------
    # 1a. Definitions
    # ------------------------------------------------------------------
    resource_grid = np.asarray(resource_grid, dtype=complex)
    if resource_grid.ndim == 1:
        resource_grid = resource_grid[:, np.newaxis]

    num_subcarriers, num_ofdm_symbols = resource_grid.shape
    assert num_subcarriers == 841, "Invalid number of subcarriers."

    if sample_rate == 20e6:
        ifft_size = 1024
        cp_length = 116
    else:
        ifft_size = 2048
        cp_length = 232

    ofdm_symbol_length = cp_length + ifft_size

    # ------------------------------------------------------------------
    # 1b. Defined by the resource grid (0-based indices)
    # ------------------------------------------------------------------
    pos_subcarriers = np.arange(420, 841)   # 420:840
    neg_subcarriers = np.arange(0, 420)     # 0:419

    # ------------------------------------------------------------------
    # 1c. Map carrier indices to IFFT input (0-based indices)
    # ------------------------------------------------------------------
    pos_ifft_indices = np.arange(0, 421)    # 0:420
    if ifft_size == 1024:
        neg_ifft_indices = np.arange(604, 1024)    # 604:1023
    else:
        neg_ifft_indices = np.arange(1628, 2048)   # 1628:2047

    # ------------------------------------------------------------------
    # 1d. OFDM modulation
    # ------------------------------------------------------------------
    sample_output = np.zeros(num_ofdm_symbols * ofdm_symbol_length, dtype=complex)
    ifft_input = np.zeros(ifft_size, dtype=complex)
    sample_output_index = 0  # Python uses 0-based indexing

    for i in range(num_ofdm_symbols):
        ofdm_symbol = np.zeros(ofdm_symbol_length, dtype=complex)

        ifft_input[pos_ifft_indices] = resource_grid[pos_subcarriers, i]
        ifft_input[neg_ifft_indices] = resource_grid[neg_subcarriers, i]

        # np.fft.ifft uses the same 1/N normalization as MATLAB's ifft
        ifft_output = np.fft.ifft(ifft_input)

        # Fetch the cyclic prefix and place it at the start of the OFDM symbol
        cyclic_prefix = ifft_output[ifft_size - cp_length:]
        ofdm_symbol[:cp_length] = cyclic_prefix
        ofdm_symbol[cp_length:] = ifft_output

        # Place the OFDM symbol into the output buffer and advance the index
        sample_output[sample_output_index:sample_output_index + ofdm_symbol_length] = ofdm_symbol
        sample_output_index += ofdm_symbol_length

    # ------------------------------------------------------------------
    # 1e. Scaling -> This shouldn't have to be done in actual HW implementation
    #     sqrt(IFFT_Size) undoes the 1/IFFT_Size normalization in ifft.
    # ------------------------------------------------------------------
    sample_output = sample_output * np.sqrt(ifft_size)

    return sample_output


if __name__ == "__main__":
    # Simple self-test with random QPSK symbols
    rng = np.random.default_rng(0)
    num_symbols = 4
    qpsk = (rng.choice([-1, 1], size=(841, num_symbols))
            + 1j * rng.choice([-1, 1], size=(841, num_symbols))) / np.sqrt(2)

    for fs in (20e6, 40e6):
        out = ofdm_modulator_841(qpsk, fs)
        print(f"SampleRate = {fs / 1e6:.0f} MHz -> output length = {len(out)}, "
              f"mean power = {np.mean(np.abs(out) ** 2):.4f}")
