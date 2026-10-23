#!/usr/bin/env python3
"""
Receive_Decimate.py

First receive-chain step for preamble / packet detection:
decimate the 40 Msps input waveform by 2 to 20 Msps.

    Rx_Input  : HDF5 file holding the received waveform (e.g. Test_Preamble.h5)
    Rx_Output : HDF5 file holding the decimated waveform (default: <input>_dec2.h5)

The input layout is auto-detected:
    * a single complex dataset                (e.g. /waveform, complex64/128)
    * separate real I and Q datasets          (names containing 'i'/'q', 'real'/'imag', 're'/'im')
    * one real dataset shaped (N,2) or (2,N)  (columns/rows are I and Q)
    * a single real dataset                   (real-valued signal)
    * h5py compound dtype with 'r'/'i' or 'real'/'imag' fields (MATLAB-style complex)
Use --dataset (and --q-dataset) to pick datasets explicitly if needed.

Filtering:
    --filter fir (default): linear-phase FIR low-pass (Kaiser window), applied
                            zero-phase so the preamble is not shifted in time.
    --filter iir          : 8th-order Chebyshev I, cutoff 0.8*(Fs/2)/2, filtfilt —
                            the same as MATLAB's default decimate(x, 2).

The output file stores the complex waveform as dataset 'Rx_Decimated' (complex64),
plus 'I' and 'Q' real datasets (easy to read in MATLAB with h5read), and
attributes sample_rate_hz, input_sample_rate_hz, decimation_factor, filter.

Usage:
    python Receive_Decimate.py Test_Preamble.h5
    python Receive_Decimate.py Test_Preamble.h5 -o Rx_Dec.h5 --filter iir
    python Receive_Decimate.py Test_Preamble.h5 --dataset /preamble --plot
"""

import argparse
import os
import sys

import h5py
import numpy as np
from scipy import signal

FS_IN_DEFAULT = 40e6   # input sample rate, samples/s
DECIMATION = 2

_I_NAMES = ("i", "real", "re", "in_phase", "inphase", "i_data")
_Q_NAMES = ("q", "imag", "im", "quadrature", "q_data")


# --------------------------------------------------------------------------- #
# Input
# --------------------------------------------------------------------------- #
def _list_datasets(h5):
    out = []
    h5.visititems(lambda name, obj: out.append(name) if isinstance(obj, h5py.Dataset) else None)
    return out


def _leaf(name):
    return name.rsplit("/", 1)[-1].lower()


def _to_complex(arr):
    """Convert a dataset array to a 1-D complex (or real) numpy vector."""
    if arr.dtype.names:  # compound type, e.g. MATLAB complex {'real','imag'} or {'r','i'}
        names = {n.lower(): n for n in arr.dtype.names}
        re_key = names.get("real") or names.get("r") or names.get("re")
        im_key = names.get("imag") or names.get("i") or names.get("im")
        if re_key is None or im_key is None:
            raise ValueError(f"Unrecognised compound dtype fields: {arr.dtype.names}")
        arr = arr[re_key].astype(np.float64) + 1j * arr[im_key].astype(np.float64)

    arr = np.squeeze(np.asarray(arr))
    if np.iscomplexobj(arr):
        if arr.ndim != 1:
            raise ValueError(f"Complex dataset has shape {arr.shape}; expected a vector.")
        return arr.astype(np.complex128)

    if arr.ndim == 1:
        return arr.astype(np.float64)
    if arr.ndim == 2 and 2 in arr.shape:
        if arr.shape[1] == 2:
            return arr[:, 0].astype(np.float64) + 1j * arr[:, 1].astype(np.float64)
        return arr[0, :].astype(np.float64) + 1j * arr[1, :].astype(np.float64)
    raise ValueError(f"Cannot interpret dataset of shape {arr.shape} as a waveform.")


def _read_fs(h5, dset):
    """Look for a sample-rate attribute on the dataset or the file root."""
    for obj in (dset, h5):
        if obj is None:
            continue
        for key, val in obj.attrs.items():
            k = key.lower()
            if k in ("fs", "sample_rate", "sample_rate_hz", "samplerate", "sampling_rate", "fs_hz"):
                try:
                    return float(np.squeeze(val))
                except (TypeError, ValueError):
                    pass
    return None


def read_rx_input(path, dataset=None, q_dataset=None):
    """Return (waveform, sample_rate_or_None, description)."""
    with h5py.File(path, "r") as h5:
        names = _list_datasets(h5)
        if not names:
            raise ValueError(f"No datasets found in {path}")

        # Explicit selection
        if dataset:
            x = _to_complex(h5[dataset][()])
            if q_dataset:
                q = np.squeeze(h5[q_dataset][()]).astype(np.float64)
                x = np.real(x) + 1j * q
                return x, _read_fs(h5, h5[dataset]), f"I='{dataset}', Q='{q_dataset}'"
            return x, _read_fs(h5, h5[dataset]), f"'{dataset}'"

        # Separate I / Q datasets?
        i_name = next((n for n in names if _leaf(n) in _I_NAMES), None)
        q_name = next((n for n in names if _leaf(n) in _Q_NAMES), None)
        if i_name and q_name:
            i = np.squeeze(h5[i_name][()]).astype(np.float64)
            q = np.squeeze(h5[q_name][()]).astype(np.float64)
            return i + 1j * q, _read_fs(h5, h5[i_name]), f"I='{i_name}', Q='{q_name}'"

        # Otherwise: largest dataset that looks like a waveform
        candidates = sorted(names, key=lambda n: h5[n].size, reverse=True)
        for n in candidates:
            if h5[n].size < 2:
                continue
            try:
                x = _to_complex(h5[n][()])
                return x, _read_fs(h5, h5[n]), f"'{n}'"
            except ValueError:
                continue
        raise ValueError(f"No waveform-like dataset found in {path}. Datasets: {names}")


# --------------------------------------------------------------------------- #
# Decimation
# --------------------------------------------------------------------------- #
def design_fir(q=DECIMATION, numtaps=63, atten_db=80.0):
    """Linear-phase low-pass for decimation by q.
    Passband edge 0.8*(Fs_out/2), stopband edge at Fs_out/2."""
    cutoff = 0.9 / q                # normalised to input Nyquist (transition centred)
    width = 0.2 / q                 # 0.8/q .. 1.0/q
    n_kaiser, beta = signal.kaiserord(atten_db, width)
    numtaps = max(numtaps, n_kaiser | 1)  # odd length -> integer group delay
    return signal.firwin(numtaps, cutoff, window=("kaiser", beta))


def decimate(x, q=DECIMATION, filt="fir"):
    """Anti-alias filter (zero-phase) and keep every q-th sample, starting at sample 0."""
    if filt == "iir":
        # Equivalent to MATLAB decimate(x, q): Cheby I, order 8, 0.05 dB ripple, filtfilt
        return signal.decimate(x, q, n=8, ftype="iir", zero_phase=True), "cheby1 order 8, filtfilt (MATLAB default)"
    h = design_fir(q)
    y = signal.filtfilt(h, [1.0], x, padtype="odd", padlen=min(3 * len(h), len(x) - 1))
    return y[::q], f"FIR {len(h)} taps Kaiser, zero-phase"


# --------------------------------------------------------------------------- #
# Output
# --------------------------------------------------------------------------- #
def write_output(path, y, fs_out, fs_in, filt_desc, src):
    with h5py.File(path, "w") as h5:
        d = h5.create_dataset("Rx_Decimated", data=y.astype(np.complex64 if np.iscomplexobj(y) else np.float32))
        h5.create_dataset("I", data=np.real(y).astype(np.float32))
        if np.iscomplexobj(y):
            h5.create_dataset("Q", data=np.imag(y).astype(np.float32))
        for obj in (h5, d):
            obj.attrs["sample_rate_hz"] = fs_out
            obj.attrs["input_sample_rate_hz"] = fs_in
            obj.attrs["decimation_factor"] = DECIMATION
            obj.attrs["filter"] = filt_desc
            obj.attrs["source_file"] = os.path.basename(src)


def plot(x, y, fs_in, fs_out):
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 1, figsize=(10, 7))
    t_in = np.arange(len(x)) / fs_in * 1e6
    t_out = np.arange(len(y)) / fs_out * 1e6
    ax[0].plot(t_in, np.abs(x), label=f"Rx_Input |x| ({fs_in/1e6:g} Msps)", alpha=0.6)
    ax[0].plot(t_out, np.abs(y), ".-", label=f"Decimated |y| ({fs_out/1e6:g} Msps)", ms=3)
    ax[0].set_xlabel("Time (µs)"); ax[0].set_ylabel("Magnitude"); ax[0].legend(); ax[0].grid(True)
    for sig, fs, lbl in ((x, fs_in, "Input"), (y, fs_out, "Decimated")):
        f, p = signal.welch(sig, fs=fs, nperseg=min(256, len(sig)), return_onesided=not np.iscomplexobj(sig))
        f, p = np.fft.fftshift(f), np.fft.fftshift(p)
        ax[1].plot(f / 1e6, 10 * np.log10(p + 1e-20), label=lbl)
    ax[1].set_xlabel("Frequency (MHz)"); ax[1].set_ylabel("PSD (dB/Hz)"); ax[1].legend(); ax[1].grid(True)
    fig.tight_layout()
    plt.show()


# --------------------------------------------------------------------------- #
def main(argv=None):
    p = argparse.ArgumentParser(description="Decimate a 40 Msps Rx waveform by 2 (to 20 Msps).")
    p.add_argument("Rx_Input", nargs="?", default="Test_Preamble.h5", help="input HDF5 file (default: Test_Preamble.h5)")
    p.add_argument("-o", "--output", help="output HDF5 file (default: <input>_dec2.h5)")
    p.add_argument("--dataset", help="dataset path holding the waveform (or the I channel)")
    p.add_argument("--q-dataset", help="dataset path holding the Q channel, if separate")
    p.add_argument("--fs", type=float, help="input sample rate in Hz (overrides file attribute; default 40e6)")
    p.add_argument("--filter", choices=("fir", "iir"), default="fir", help="anti-alias filter type (default fir)")
    p.add_argument("--plot", action="store_true", help="plot input vs decimated waveform and spectra")
    a = p.parse_args(argv)

    x, fs_file, src_desc = read_rx_input(a.Rx_Input, a.dataset, a.q_dataset)
    fs_in = a.fs or fs_file or FS_IN_DEFAULT
    if fs_file and a.fs is None and abs(fs_file - FS_IN_DEFAULT) > 1:
        print(f"Note: file says sample rate is {fs_file/1e6:g} Msps (expected 40 Msps); using file value.")

    if len(x) < 30:
        sys.exit(f"Input too short to filter ({len(x)} samples).")

    y, filt_desc = decimate(x, DECIMATION, a.filter)
    fs_out = fs_in / DECIMATION

    out = a.output or os.path.splitext(a.Rx_Input)[0] + "_dec2.h5"
    write_output(out, y, fs_out, fs_in, filt_desc, a.Rx_Input)

    print(f"Input : {a.Rx_Input} dataset {src_desc}, {len(x)} samples @ {fs_in/1e6:g} Msps, "
          f"{'complex' if np.iscomplexobj(x) else 'real'}")
    print(f"Filter: {filt_desc}")
    print(f"Output: {out} dataset 'Rx_Decimated', {len(y)} samples @ {fs_out/1e6:g} Msps")

    if a.plot:
        plot(x, y, fs_in, fs_out)
    return y


if __name__ == "__main__":
    main()
