"""
Test_OFDM_Modulator_841.py

Loads a sample resource grid from an HDF5 file (default: Test_Payload_Grid.h5),
runs it through ofdm_modulator_841 and reports whether the output scaling
looks right.

Usage:
    python Test_OFDM_Modulator_841.py [path/to/Test_Payload_Grid.h5]
                                      [--sample-rate 40e6]
                                      [--save output.npy]

The resource grid is expected to be normalized to unit average power.
"""

import argparse
import sys

import numpy as np

from OFDM_Modulator_841 import ofdm_modulator_841

NUM_SUBCARRIERS = 841


# ----------------------------------------------------------------------
# Loading
# ----------------------------------------------------------------------
def _to_complex(data):
    """Convert an HDF5 dataset's contents to a complex numpy array."""
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


def _orient(grid):
    """Return the grid as (841, NumOfdmSymbols)."""
    if grid.ndim == 1:
        return grid[:, np.newaxis]
    if grid.shape[0] == NUM_SUBCARRIERS:
        return grid
    if grid.shape[1] == NUM_SUBCARRIERS:
        # MATLAB writes column-major, so h5py sees the transpose
        return grid.T
    raise ValueError(f"No dimension of size {NUM_SUBCARRIERS} in shape {grid.shape}")


def load_resource_grid(path):
    """Find and load the resource grid in an HDF5 file."""
    import h5py

    datasets = {}
    with h5py.File(path, "r") as f:
        f.visititems(lambda name, obj: datasets.__setitem__(name, obj[()])
                     if isinstance(obj, h5py.Dataset) else None)

    print(f"Datasets in {path}:")
    for name, val in datasets.items():
        val = np.asarray(val)
        print(f"  {name:30s} shape={val.shape} dtype={val.dtype}")

    # 1) A single dataset with an 841-long dimension
    candidates = [n for n, v in datasets.items()
                  if NUM_SUBCARRIERS in np.shape(v)]
    complex_like = [n for n in candidates
                    if np.iscomplexobj(datasets[n]) or np.asarray(datasets[n]).dtype.names]
    pick = (complex_like or [n for n in candidates if "grid" in n.lower()] or candidates)

    # 2) Separate real / imaginary datasets
    real = [n for n in candidates if "real" in n.lower() or n.lower().endswith("_re")]
    imag = [n for n in candidates if "imag" in n.lower() or n.lower().endswith("_im")]
    if not complex_like and real and imag:
        grid = np.asarray(datasets[real[0]], float) + 1j * np.asarray(datasets[imag[0]], float)
        print(f"Using {real[0]} + j*{imag[0]}")
        return _orient(grid)

    if not pick:
        raise ValueError(f"No dataset with a dimension of {NUM_SUBCARRIERS} found in {path}")
    print(f"Using dataset '{pick[0]}'")
    return _orient(_to_complex(datasets[pick[0]]))


# ----------------------------------------------------------------------
# Analysis
# ----------------------------------------------------------------------
def estimate_modulation_order(grid):
    """Guess M from the number of distinct constellation points."""
    symbols = grid[np.abs(grid) > 1e-9]
    if symbols.size == 0:
        return None, symbols
    scale = np.sqrt(np.mean(np.abs(symbols) ** 2))
    points = np.unique(np.round(symbols / scale, 3))
    n = len(points)
    if np.all(np.abs(symbols.imag) < 1e-9 * scale) and n <= 2:
        return 2, symbols
    for M in (4, 16, 64):
        if n <= M:
            return M, symbols
    return None, symbols


def db(x):
    return 10 * np.log10(x)


def analyze(grid, sample_rate=40e6):
    num_sc, num_sym = grid.shape
    ifft_size = 1024 if sample_rate == 20e6 else 2048

    est_M, symbols = estimate_modulation_order(grid)
    occupancy = symbols.size / grid.size
    Es = np.mean(np.abs(symbols) ** 2)

    print("\n--- Resource grid ---")
    print(f"Shape                     : {num_sc} subcarriers x {num_sym} OFDM symbols")
    print(f"Non-zero resource elements: {symbols.size} of {grid.size} ({100 * occupancy:.1f} %)")
    print(f"Estimated modulation      : M = {est_M}")
    print(f"Avg symbol power Es       : {Es:.4f}")
    print(f"Peak |symbol|             : {np.max(np.abs(symbols)):.4f}")
    dc = grid[420, :]
    if np.any(np.abs(dc) > 1e-9):
        print("Note                      : subcarrier 420 (DC bin) carries data")

    out = ofdm_modulator_841(grid, sample_rate)
    p_avg = np.mean(np.abs(out) ** 2)
    p_peak = np.max(np.abs(out) ** 2)
    peak_iq = max(np.max(np.abs(out.real)), np.max(np.abs(out.imag)))

    print("\n--- Modulator output ---")
    print(f"Sample rate / IFFT size   : {sample_rate / 1e6:.0f} MHz / {ifft_size}")
    print(f"Output samples            : {out.size}")
    print(f"RMS                       : {np.sqrt(p_avg):.4f}")
    print(f"Peak |x|                  : {np.sqrt(p_peak):.4f}")
    print(f"Peak |I| or |Q|           : {peak_iq:.4f}")
    print(f"PAPR                      : {db(p_peak / p_avg):.2f} dB")

    # ------------------------------------------------------------------
    # Scaling assessment
    # ------------------------------------------------------------------
    print("\n--- Scaling assessment ---")
    unit_power = np.isclose(Es, 1.0, rtol=0.05)
    if unit_power:
        print(f"Input is unit power as expected (Es = {Es:.4f}).")
    else:
        print(f"WARNING: input is expected to be unit power but Es = {Es:.4f} "
              f"({db(Es):+.2f} dB).")

    # Output power relative to Es, using the actual number of non-zero
    # subcarriers per OFDM symbol (sqrt(IFFT_Size) undoes the 1/IFFT_Size in ifft)
    used_per_symbol = symbols.size / num_sym
    expected_p = used_per_symbol / ifft_size * Es
    print(f"With {used_per_symbol:.0f} of {ifft_size} IFFT bins used per symbol, the expected "
          f"output power is {used_per_symbol:.0f}/{ifft_size} x Es = {expected_p:.4f} "
          f"(measured {p_avg:.4f}).")

    # DAC view: how much headroom is left if full scale is +-1.0 per rail
    backoff_db = 20 * np.log10(1.0 / peak_iq) if peak_iq > 0 else np.inf
    print(f"\nAgainst a +-1.0 full-scale DAC the peak I/Q is at {peak_iq:.3f} "
          f"({backoff_db:+.1f} dB headroom); RMS sits {db(1.0 / p_avg):.1f} dB below full scale.")
    if peak_iq > 1.0:
        print("The output would clip at +-1.0 full scale; scale down before quantizing.")

    return out, {"M": est_M, "Es": Es, "rms": np.sqrt(p_avg), "peak_iq": peak_iq,
                 "papr_db": db(p_peak / p_avg), "unit_power": unit_power}


def test_ofdm_modulator_841(path="Test_Payload_Grid.h5", sample_rate=40e6, save=None):
    grid = load_resource_grid(path)
    out, stats = analyze(grid, sample_rate)

    # Structural checks
    ifft_size = 1024 if sample_rate == 20e6 else 2048
    cp = 116 if sample_rate == 20e6 else 232
    sym_len = ifft_size + cp
    assert out.size == grid.shape[1] * sym_len, "Unexpected output length"
    for s in range(grid.shape[1]):
        start = s * sym_len
        assert np.allclose(out[start:start + cp], out[start + ifft_size:start + sym_len]), \
            f"Cyclic prefix mismatch in symbol {s}"
    print("\nStructural checks passed (length and cyclic prefixes).")

    if save:
        np.save(save, out)
        print(f"Output saved to {save}")
    return out, stats


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("path", nargs="?", default="Test_Payload_Grid.h5")
    p.add_argument("--sample-rate", type=float, default=40e6)
    p.add_argument("--save", default=None)
    a = p.parse_args()
    try:
        test_ofdm_modulator_841(a.path, a.sample_rate, a.save)
    except ImportError:
        sys.exit("h5py is required: pip install h5py")
