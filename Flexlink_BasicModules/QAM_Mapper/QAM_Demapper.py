#!/usr/bin/env python3
"""
IEEE 802.11 constellation demapper (BPSK, QPSK, 16-QAM, 64-QAM).
Inverse of QAM_Mapper.py (it reuses the same constellation tables).

    Bitstream_Out = QAM_Demapper(Input_QAM, modulation, mode="hard", noise_var=1.0)

Modes
-----
  "hard"   : minimum-distance decision. Returns bits (0/1, int).
  "maxlog" : soft output, max-log-MAP approximation of the LLR (cheap, and
             what most hardware uses). Returns float LLRs.
  "llr"    : soft output, exact log-MAP LLR. Returns float LLRs.

LLR convention
--------------
    LLR_k = ln( P(bit_k = 0 | y) / P(bit_k = 1 | y) )

Positive LLR -> bit 0 is more likely, negative -> bit 1.
Hard bits from LLRs:  bit = (LLR < 0).  (See llr_to_bits().)
Magnitude = confidence. This is the form a Viterbi / LDPC decoder expects
(some decoders use the opposite sign, so check yours).

Noise variance
--------------
noise_var is the TOTAL complex noise variance, sigma^2 = E[|n|^2]
(i.e. sigma^2/2 per real dimension), relative to the unit-average-power
constellation produced by QAM_Mapper. It only scales the LLRs, so the sign
(hard decision) is unaffected, but you need the right value if a decoder
uses the LLR magnitudes. For a given SNR with unit signal power:
noise_var = 10**(-SNR_dB/10).

noise_var may also be an array with one value per input symbol. Use that after
OFDM equalization: for x = Y/H the noise left on each tone is sigma^2/|H|^2, so
faded tones get small (honest) LLRs instead of overconfident ones.

Usage from the command line:
    python QAM_Demapper.py --selftest
"""

import numpy as np

from QAM_Mapper import qam_map, _MODULATIONS, _LUTS

_MODES = ("hard", "maxlog", "llr")


def _normalise_name(modulation):
    name = str(modulation).upper().replace("-", "").replace("_", "").replace(" ", "")
    if name not in _MODULATIONS:
        raise ValueError("Unsupported modulation '{}'. Choose from: {}"
                         .format(modulation, ", ".join(_MODULATIONS)))
    return name


def _logsumexp(a, axis):
    """Numerically stable log(sum(exp(a))) along axis."""
    m = np.max(a, axis=axis, keepdims=True)
    return np.squeeze(m, axis=axis) + np.log(np.sum(np.exp(a - m), axis=axis))


def QAM_Demapper(Input_QAM, modulation, mode="hard", noise_var=1.0):
    """
    Demap received 802.11 constellation symbols to bits or LLRs.

    Parameters
    ----------
    Input_QAM  : array-like of complex symbols (unit average power scaling,
                 i.e. K_MOD already applied, as produced by qam_map).
    modulation : "BPSK", "QPSK", "16QAM" or "64QAM".
    mode       : "hard" (default), "maxlog" or "llr" -- see module docstring.
    noise_var  : total complex noise variance (soft modes only, must be > 0).
                 Scalar, or one value per symbol (same length as Input_QAM).

    Returns
    -------
    Bitstream_Out : 1-D array, length = len(Input_QAM) * bits_per_symbol.
                    int 0/1 for "hard", float LLRs for "maxlog" / "llr".
                    Bit order matches qam_map (first bit of a symbol = MSB).
    """
    name = _normalise_name(modulation)
    mode = str(mode).lower()
    if mode not in _MODES:
        raise ValueError("mode must be one of {}".format(_MODES))
    if mode != "hard" and np.any(np.asarray(noise_var, dtype=float) <= 0):
        raise ValueError("noise_var must be > 0 for soft demapping.")

    nbpsc = _MODULATIONS[name][0]
    lut = _LUTS[name]                      # complex points, index = bit-group integer
    y = np.asarray(Input_QAM, dtype=complex).ravel()

    # Squared distance from every received symbol to every constellation point
    d2 = np.abs(y[:, None] - lut[None, :]) ** 2          # shape (N, 2**nbpsc)

    # Bit table: bit_table[idx, k] = k-th bit (k=0 is MSB) of constellation index idx
    idx = np.arange(2 ** nbpsc)
    bit_table = (idx[:, None] >> (nbpsc - 1 - np.arange(nbpsc))[None, :]) & 1

    if mode == "hard":
        best = np.argmin(d2, axis=1)                     # nearest point
        return bit_table[best].reshape(-1).astype(int)

    # Soft output; noise_var as a column so a per-symbol array broadcasts over points
    nv = np.asarray(noise_var, dtype=float).ravel()
    if nv.size not in (1, y.size):
        raise ValueError("noise_var must be a scalar or have one value per symbol.")
    nv = np.broadcast_to(nv, (y.size,))
    llr = np.empty((y.size, nbpsc), dtype=float)
    for k in range(nbpsc):
        d2_0 = d2[:, bit_table[:, k] == 0]               # points where bit k = 0
        d2_1 = d2[:, bit_table[:, k] == 1]               # points where bit k = 1
        if mode == "maxlog":
            llr[:, k] = (np.min(d2_1, axis=1) - np.min(d2_0, axis=1)) / nv
        else:  # exact log-MAP
            llr[:, k] = (_logsumexp(-d2_0 / nv[:, None], axis=1)
                         - _logsumexp(-d2_1 / nv[:, None], axis=1))
    return llr.reshape(-1)


def llr_to_bits(llr):
    """Hard decision from LLRs (LLR > 0 -> 0, LLR < 0 -> 1)."""
    return (np.asarray(llr) < 0).astype(int)


def _selftest():
    rng = np.random.default_rng(1)
    for name, (nb, *_rest) in _MODULATIONS.items():
        bits = rng.integers(0, 2, nb * 5000)
        tx = qam_map(bits, name)

        # 1) Noiseless round trip, all modes
        assert np.array_equal(QAM_Demapper(tx, name, "hard"), bits), name
        for mode in ("maxlog", "llr"):
            soft = QAM_Demapper(tx, name, mode, noise_var=0.01)
            assert np.array_equal(llr_to_bits(soft), bits), (name, mode)

        # 2) Noisy channel at ~20 dB SNR: hard and LLR decisions should agree
        #    (almost always) and BER should be small
        nv = 10 ** (-20 / 10)
        noise = np.sqrt(nv / 2) * (rng.standard_normal(tx.size)
                                   + 1j * rng.standard_normal(tx.size))
        rx = tx + noise
        hard = QAM_Demapper(rx, name, "hard")
        for mode in ("maxlog", "llr"):
            soft = QAM_Demapper(rx, name, mode, noise_var=nv)
            assert np.mean(llr_to_bits(soft) != hard) < 1e-3, (name, mode)
        assert np.mean(hard != bits) < 0.05, name

        # 3) Max-log should be close to exact LLR at high SNR
        a = QAM_Demapper(rx, name, "maxlog", nv)
        b = QAM_Demapper(rx, name, "llr", nv)
        assert np.corrcoef(a, b)[0, 1] > 0.99, name

    # Per-symbol noise_var: equals scalar results when all values are equal, and
    # a larger noise_var on a symbol shrinks only that symbol's LLRs
    for mode in ("maxlog", "llr"):
        a = QAM_Demapper(rx, name, mode, 0.05)
        b = QAM_Demapper(rx, name, mode, np.full(rx.size, 0.05))
        assert np.allclose(a, b), mode
        nvv = np.full(rx.size, 0.05); nvv[0] = 5.0
        c = QAM_Demapper(rx, name, mode, nvv)
        nb_last = _MODULATIONS[name][0]
        assert np.all(np.abs(c[:nb_last]) < np.abs(a[:nb_last]) + 1e-12), mode
        assert np.allclose(c[nb_last:], a[nb_last:]), mode

    # Sign convention check: BPSK, symbol +1 is bit 1 -> negative LLR
    assert QAM_Demapper([1 + 0j], "BPSK", "llr", 0.5)[0] < 0
    assert QAM_Demapper([-1 + 0j], "BPSK", "llr", 0.5)[0] > 0
    print("All self-tests passed.")


if __name__ == "__main__":
    _selftest()
