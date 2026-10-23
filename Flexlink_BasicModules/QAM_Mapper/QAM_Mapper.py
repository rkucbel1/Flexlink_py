#!/usr/bin/env python3
"""
IEEE 802.11 constellation mapper (BPSK, QPSK, 16-QAM, 64-QAM).

Follows the Gray-coded mapping tables of IEEE 802.11-2020 (Clause 17.3.5.8,
Tables 17-14..17-17, also used by 802.11n/ac/ax OFDM PHYs):

  * The first bits of each group select the I (real) value, the remaining
    bits select the Q (imaginary) value. First bit is the MSB.
  * Constellation points are multiplied by the normalisation factor K_MOD so
    average symbol power is 1.

    BPSK   : K_MOD = 1
    QPSK   : K_MOD = 1/sqrt(2)
    16-QAM : K_MOD = 1/sqrt(10)
    64-QAM : K_MOD = 1/sqrt(42)

Usage as a module:
    from QAM_Mapper import qam_map
    Output_QAM = qam_map(Bitstream_In, "16QAM")

Usage from the command line:
    python QAM_Mapper.py 64QAM 101100011010...
    python QAM_Mapper.py --selftest
"""

import sys
import numpy as np

# --- 802.11 Gray mapping tables: bit-group (as string, b0 first) -> amplitude ---
_BPSK_TABLE = {"0": -1, "1": 1}

_QAM4_TABLE = {"0": -1, "1": 1}  # per axis for QPSK

_QAM16_TABLE = {"00": -3, "01": -1, "11": 1, "10": 3}  # per axis

_QAM64_TABLE = {
    "000": -7, "001": -5, "011": -3, "010": -1,
    "110": 1, "111": 3, "101": 5, "100": 7,
}  # per axis

# modulation name -> (bits per symbol, bits per axis, axis table, K_MOD)
_MODULATIONS = {
    "BPSK":  (1, None, _BPSK_TABLE,  1.0),
    "QPSK":  (2, 1,    _QAM4_TABLE,  1 / np.sqrt(2)),
    "16QAM": (4, 2,    _QAM16_TABLE, 1 / np.sqrt(10)),
    "64QAM": (6, 3,    _QAM64_TABLE, 1 / np.sqrt(42)),
}


def _build_lut(bits_per_sym, bits_per_axis, table, kmod):
    """Build a lookup array of size 2**bits_per_sym indexed by the integer
    value of the bit group (b0 = MSB). Returns complex constellation points."""
    lut = np.zeros(2 ** bits_per_sym, dtype=complex)
    for idx in range(2 ** bits_per_sym):
        bits = format(idx, "0{}b".format(bits_per_sym))
        if bits_per_axis is None:  # BPSK: real only
            lut[idx] = table[bits] * kmod
        else:
            i_val = table[bits[:bits_per_axis]]
            q_val = table[bits[bits_per_axis:]]
            lut[idx] = (i_val + 1j * q_val) * kmod
    return lut


_LUTS = {name: _build_lut(*params) for name, params in _MODULATIONS.items()}


def qam_map(Bitstream_In, modulation):
    """
    Map a bitstream to 802.11 constellation symbols.

    Parameters
    ----------
    Bitstream_In : array-like of 0/1 (list, tuple, numpy array) or a string
                   such as "101100". Length must be a multiple of the number
                   of bits per symbol.
    modulation   : "BPSK", "QPSK", "16QAM" or "64QAM" (case/dash/space
                   insensitive, e.g. "16-QAM" works).

    Returns
    -------
    Output_QAM : numpy complex array of normalised constellation points.
    """
    name = str(modulation).upper().replace("-", "").replace("_", "").replace(" ", "")
    if name not in _MODULATIONS:
        raise ValueError("Unsupported modulation '{}'. Choose from: {}"
                         .format(modulation, ", ".join(_MODULATIONS)))

    if isinstance(Bitstream_In, str):
        Bitstream_In = [int(c) for c in Bitstream_In.strip() if c in "01"]
    bits = np.asarray(Bitstream_In, dtype=int).ravel()

    if bits.size and not np.all((bits == 0) | (bits == 1)):
        raise ValueError("Bitstream_In must contain only 0s and 1s.")

    nbpsc = _MODULATIONS[name][0]
    if bits.size % nbpsc != 0:
        raise ValueError("Bitstream_In length ({}) must be a multiple of {} for {}."
                         .format(bits.size, nbpsc, name))

    # Group bits and convert each group to an integer index (b0 = MSB)
    groups = bits.reshape(-1, nbpsc)
    weights = 1 << np.arange(nbpsc - 1, -1, -1)
    indices = groups @ weights

    Output_QAM = _LUTS[name][indices]
    return Output_QAM


def _selftest():
    """Check against known values from the 802.11 mapping tables."""
    k2, k10, k42 = 1 / np.sqrt(2), 1 / np.sqrt(10), 1 / np.sqrt(42)

    assert np.allclose(qam_map("01", "BPSK"), [-1, 1])
    assert np.allclose(qam_map("0011", "QPSK"), [(-1 - 1j) * k2, (1 + 1j) * k2])
    assert np.allclose(qam_map("01", "QPSK"), [(-1 + 1j) * k2])
    # 16QAM: b0b1=00 -> -3, b2b3=10 -> +3
    assert np.allclose(qam_map("0010", "16QAM"), [(-3 + 3j) * k10])
    assert np.allclose(qam_map("1101", "16QAM"), [(1 - 1j) * k10])
    # 64QAM: 000 -> -7, 100 -> +7, 010 -> -1, 110 -> +1
    assert np.allclose(qam_map("000100", "64QAM"), [(-7 + 7j) * k42])
    assert np.allclose(qam_map("010110", "64QAM"), [(-1 + 1j) * k42])

    # Unit average power for every modulation
    for name, (nb, *_rest) in _MODULATIONS.items():
        all_bits = [(i >> (nb - 1 - j)) & 1 for i in range(2 ** nb) for j in range(nb)]
        power = np.mean(np.abs(qam_map(all_bits, name)) ** 2)
        assert abs(power - 1) < 1e-12, (name, power)
    print("All self-tests passed.")


if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] == "--selftest":
        _selftest()
    elif len(sys.argv) == 3:
        Bitstream_In = sys.argv[2]
        Output_QAM = qam_map(Bitstream_In, sys.argv[1])
        for sym in Output_QAM:
            print("{:+.4f}{:+.4f}j".format(sym.real, sym.imag))
    else:
        print(__doc__)
