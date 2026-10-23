"""
Test_OFDM_Demodulator_841.py

Loads a 40 MSPS OFDM waveform from an HDF5 file
(default: Test_Payload_Waveform_40MSPS.h5), decimates it to 20 MSPS and
runs it through ofdm_demodulator_841.

The output is a resource grid of shape (841, NumOfdmSymbols): one column per
OFDM symbol, one row per subcarrier. Flattening it column by column
(grid.flatten(order="F")) gives the symbols as one continuous stream in the
same order the modulator took them.

Usage:
    python Test_OFDM_Demodulator_841.py [Test_Payload_Waveform_40MSPS.h5]
        [--start-time 0] [--decimator simple|filter]
        [--ref Test_Payload_Grid.h5] [--plot] [--save grid.npy]
"""

import argparse
import sys

import numpy as np

from OFDM_Demodulator_841 import (ofdm_demodulator_841, FFT_SIZE, CP_LENGTH,
                                  OFDM_SYMBOL_LENGTH)


# ----------------------------------------------------------------------
# Loading
# ----------------------------------------------------------------------
def _to_complex(data):
    data = np.asarray(data)
    if np.iscomplexobj(data):
        return data.astype(complex)
    if data.dtype.names:  # compound type, e.g. MATLAB/Octave/h5py complex
        names = {n.lower(): n for n in data.dtype.names}
        re = names.get("real") or names.get("r") or names.get("re")
        im = names.get("imag") or names.get("i") or names.get("im")
        if re and im:
            return data[re].astype(float) + 1j * data[im].astype(float)
    return data.astype(complex)


def load_waveform(path):
    """Find and load the (longest) 1-D complex sample sequence in an HDF5 file."""
    import h5py

    datasets = {}
    with h5py.File(path, "r") as f:
        f.visititems(lambda name, obj: datasets.__setitem__(name, obj[()])
                     if isinstance(obj, h5py.Dataset) else None)

    print(f"Datasets in {path}:")
    for name, val in datasets.items():
        val = np.asarray(val)
        print(f"  {name:30s} shape={val.shape} dtype={val.dtype}")

    # Vectors only: (N,), (1, N) or (N, 1)
    vectors = {n: v for n, v in datasets.items()
               if np.asarray(v).size > 1 and sum(d > 1 for d in np.shape(v)) == 1}

    real = [n for n in vectors if "real" in n.lower() or n.lower().endswith("_re")]
    imag = [n for n in vectors if "imag" in n.lower() or n.lower().endswith("_im")]
    complex_like = [n for n in vectors
                    if np.iscomplexobj(vectors[n]) or np.asarray(vectors[n]).dtype.names]

    if not complex_like and real and imag:
        print(f"Using {real[0]} + j*{imag[0]}")
        return (np.asarray(vectors[real[0]], float).ravel()
                + 1j * np.asarray(vectors[imag[0]], float).ravel())

    pool = complex_like or list(vectors)
    if not pool:
        raise ValueError(f"No 1-D sample sequence found in {path}")
    pick = max(pool, key=lambda n: np.asarray(vectors[n]).size)
    print(f"Using dataset '{pick}'")
    return _to_complex(vectors[pick]).ravel()


# ----------------------------------------------------------------------
# Decimation 40 -> 20 MSPS
# ----------------------------------------------------------------------
def decimate_by_2(x, method="simple"):
    """
    simple : keep every other sample. Exact for a clean simulated waveform,
             since the 841 subcarriers span only about +-8.2 MHz, inside the
             +-10 MHz band of the 20 MSPS output.
    filter : low-pass filter, then keep every other sample
             (scipy.signal.resample_poly; its linear-phase filter is
             delay-compensated, so symbol timing is preserved). Use this for
             captured signals that may have content outside +-10 MHz.
    """
    if method == "simple":
        return x[::2]
    if method == "filter":
        from scipy.signal import resample_poly
        return resample_poly(x, 1, 2)
    raise ValueError(f"Unknown decimator '{method}'")


# ----------------------------------------------------------------------
# Timing check (cyclic-prefix correlation)
# ----------------------------------------------------------------------
def estimate_start_time(x):
    """
    Return the offset (0 .. OFDM_SYMBOL_LENGTH-1) where the cyclic prefix best
    matches the end of its symbol, together with a 0..1 correlation metric.
    Assumes the OFDM symbols follow one another without gaps.
    """
    n_sym = (len(x) - FFT_SIZE) // OFDM_SYMBOL_LENGTH
    if n_sym < 1:
        return None, 0.0
    n = n_sym * OFDM_SYMBOL_LENGTH + CP_LENGTH
    x = x[:n + FFT_SIZE]
    prod = x[:n] * np.conj(x[FFT_SIZE:FFT_SIZE + n])
    energy = 0.5 * (np.abs(x[:n]) ** 2 + np.abs(x[FFT_SIZE:FFT_SIZE + n]) ** 2)
    kernel = np.ones(CP_LENGTH)
    c = np.convolve(prod, kernel, "valid")      # sum over each CP-length window
    e = np.convolve(energy, kernel, "valid")
    m = n_sym * OFDM_SYMBOL_LENGTH
    c = c[:m].reshape(n_sym, OFDM_SYMBOL_LENGTH).sum(axis=0)
    e = e[:m].reshape(n_sym, OFDM_SYMBOL_LENGTH).sum(axis=0)
    metric = np.abs(c) / np.maximum(e, 1e-30)
    best = int(np.argmax(metric))
    return best, float(metric[best])


# ----------------------------------------------------------------------
# Test
# ----------------------------------------------------------------------
def db(x):
    return 10 * np.log10(x)


def test_ofdm_demodulator_841(path="Test_Payload_Waveform_40MSPS.h5", start_time=0,
                              decimator="simple", ref=None, plot=False, save=None):
    x40 = load_waveform(path)
    x20 = decimate_by_2(x40, decimator)

    print("\n--- Input ---")
    print(f"40 MSPS samples           : {x40.size}")
    print(f"20 MSPS samples           : {x20.size}  (decimator: {decimator})")
    print(f"20 MSPS RMS               : {np.sqrt(np.mean(np.abs(x20) ** 2)):.4f}")
    whole, rest = divmod(x20.size, OFDM_SYMBOL_LENGTH)
    print(f"Whole OFDM symbols        : {whole} x {OFDM_SYMBOL_LENGTH} samples"
          + (f" + {rest} leftover samples" if rest else ""))

    est, metric = estimate_start_time(x20)
    if est is not None:
        print(f"CP-correlation timing     : best start_time = {est} (metric {metric:.3f})")
        if metric > 0.5 and est != start_time % OFDM_SYMBOL_LENGTH:
            print(f"WARNING: start_time = {start_time} does not match the CP timing "
                  f"estimate ({est}); try --start-time {est}.")

    grid = ofdm_demodulator_841(x20, start_time)

    nz = grid[np.abs(grid) > 1e-6]
    print("\n--- Demodulator output (resource grid) ---")
    print(f"Shape                     : {grid.shape[0]} subcarriers x {grid.shape[1]} OFDM symbols")
    print("  The last column(s) may be empty: the demodulator pads 400 zeros and")
    print("  demodulates every whole symbol slot in the padded sequence.")
    print(f"Avg power (non-zero REs)  : {np.mean(np.abs(nz) ** 2):.4f}  "
          f"({db(np.mean(np.abs(nz) ** 2)):+.2f} dB)")
    print("  For a unit-power payload sent at 40 MSPS and decimated by 2, expect "
          "0.25 (-6.02 dB) with the current scaling.")

    if ref:
        from Test_OFDM_Modulator_841 import load_resource_grid
        ref_grid = load_resource_grid(ref)
        n = min(ref_grid.shape[1], grid.shape[1])
        rx, tx = grid[:, :n], ref_grid[:, :n]
        gain = np.vdot(tx, rx) / np.vdot(tx, tx)
        err = rx / gain - tx
        evm = np.sqrt(np.mean(np.abs(err) ** 2) / np.mean(np.abs(tx) ** 2))
        print("\n--- Comparison with reference grid ---")
        print(f"Symbols compared          : {n}")
        print(f"Complex gain rx/tx        : {abs(gain):.4f} ({20 * np.log10(abs(gain)):+.2f} dB), "
              f"phase {np.degrees(np.angle(gain)):+.2f} deg")
        print(f"EVM after gain removal    : {100 * evm:.4f} %  ({20 * np.log10(evm + 1e-300):.1f} dB)")

    if plot:
        import matplotlib.pyplot as plt
        n_show = min(3, grid.shape[1])
        fig, axes = plt.subplots(1, n_show, figsize=(4 * n_show, 4), squeeze=False)
        lim = 1.2 * np.max(np.abs(grid[:, :n_show])) if grid.size else 1
        for i in range(n_show):
            ax = axes[0, i]
            ax.plot(grid[:, i].real, grid[:, i].imag, "k.", markersize=4)
            ax.set_title(f"OFDM symbol {i}")
            ax.set_xlabel("real"); ax.set_ylabel("imag")
            ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim)
            ax.set_aspect("equal"); ax.grid(True)
        fig.suptitle("Received constellations before equalization")
        fig.tight_layout()
        plt.show()

    if save:
        np.save(save, grid)
        print(f"\nResource grid saved to {save}")
    return grid


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("path", nargs="?", default="Test_Payload_Waveform_40MSPS.h5")
    p.add_argument("--start-time", type=int, default=0)
    p.add_argument("--decimator", choices=("simple", "filter"), default="simple")
    p.add_argument("--ref", default=None, help="reference grid, e.g. Test_Payload_Grid.h5")
    p.add_argument("--plot", action="store_true")
    p.add_argument("--save", default=None)
    a = p.parse_args()
    try:
        test_ofdm_demodulator_841(a.path, a.start_time, a.decimator, a.ref, a.plot, a.save)
    except ImportError as e:
        sys.exit(f"Missing package: {e.name}. pip install h5py (and scipy/matplotlib "
                 "for --decimator filter / --plot)")
