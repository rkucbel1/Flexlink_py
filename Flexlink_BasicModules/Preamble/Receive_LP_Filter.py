#!/usr/bin/env python3
"""
Receive_LP_Filter.py

Second receive-chain step for preamble / packet detection:
low-pass filter the decimated (20 Msps) waveform, Fcutoff = 2.75 MHz.

Python equivalent of the MATLAB function:

    function Rx_Output = LP_Filter_Flexlink(Rx_Input)
    h = [...17 taps...];
    Rx_Output = filter(h, 1, Rx_Input);

MATLAB's filter(h, 1, x) is a causal FIR filter with zero initial state, so
the output is the same length as the input and is delayed by (17-1)/2 = 8
samples. scipy.signal.lfilter(h, 1, x) gives exactly the same result.

    Rx_Input  : Test_Preamble_dec2.h5              (output of Receive_Decimate.py)
    Rx_Output : Test_Preamble_dec2_LP_Filtered.h5

Output datasets: 'Rx_Filtered' (complex64), 'I' and 'Q' (float32), with
attributes sample_rate_hz, filter_taps, group_delay_samples, cutoff_hz.

Usage:
    python Receive_LP_Filter.py
    python Receive_LP_Filter.py Test_Preamble_dec2.h5 -o Test_Preamble_dec2_LP_Filtered.h5
"""

import argparse
import os

import h5py
import numpy as np
from scipy import signal

FS_DEFAULT = 20e6       # sample rate after decimation, samples/s
F_CUTOFF = 2.75e6       # Hz

# Filter coefficients from LP_Filter_Flexlink.m
H_LP = np.array([
    -0.0015, -0.0107, -0.0271, -0.0388, -0.0216,  0.0431,
     0.1452,  0.2418,  0.2817,  0.2418,  0.1452,  0.0431,
    -0.0216, -0.0388, -0.0271, -0.0107, -0.0015,
])

DEFAULT_IN = "Test_Preamble_dec2.h5"
DEFAULT_OUT = "Test_Preamble_dec2_LP_Filtered.h5"


def read_rx_input(path, dataset=None):
    """Return (complex waveform, sample rate or None, dataset name)."""
    with h5py.File(path, "r") as h5:
        if dataset is None:
            if "Rx_Decimated" in h5:
                dataset = "Rx_Decimated"
            else:  # fall back to the first complex dataset in the file
                names = []
                h5.visititems(lambda n, o: names.append(n)
                              if isinstance(o, h5py.Dataset) and np.issubdtype(o.dtype, np.complexfloating) else None)
                if not names:
                    raise ValueError(f"No complex dataset found in {path}")
                dataset = names[0]
        d = h5[dataset]
        x = np.squeeze(d[()]).astype(np.complex128)
        fs = d.attrs.get("sample_rate_hz", h5.attrs.get("sample_rate_hz"))
        return x, (float(fs) if fs is not None else None), dataset


# --------------------------------------------------------------------------- #
# DC blocker (ahead of the LP filter and the preamble processing)
#
#   y[n] = x[n] - x[n-1] + alpha * y[n-1],   alpha = 1 - 2^-DC_BLOCKER_SHIFT
#
# A notch at 0 Hz with a -3 dB corner of about (1 - alpha) * Fs / (2 pi) = 3.1 kHz at
# 20 Msps (shift 10). Preamble A has no energy at DC (tones at +-625 kHz and +-1.875 MHz),
# and the payload's DC subcarrier is empty, so nothing wanted is lost. In hardware it is two
# adders and a shift. Without it, a DC offset 10 dB below the noise triggers the detector.
# --------------------------------------------------------------------------- #
DC_BLOCKER_SHIFT = 10


def dc_blocker(x, shift=DC_BLOCKER_SHIFT, settle=256):
    """DC-blocking filter. The state starts as if the input had held its mean over the first
    'settle' samples, so a record that starts with a DC offset has no start-up transient
    (the hardware filter runs continuously and is always settled)."""
    x = np.asarray(x, dtype=np.complex128)
    alpha = 1.0 - 2.0 ** (-shift)
    b, a = [1.0, -1.0], [1.0, -alpha]
    zi = signal.lfilter_zi(b, a) * np.mean(x[:max(1, min(settle, x.size))])
    y, _ = signal.lfilter(b, a, x, zi=zi)
    return y


def lp_filter_flexlink(rx_input):
    """Rx_Output = filter(h, 1, Rx_Input)  — identical to the MATLAB function."""
    return signal.lfilter(H_LP, [1.0], rx_input)


def write_output(path, y, fs, src):
    with h5py.File(path, "w") as h5:
        d = h5.create_dataset("Rx_Filtered", data=y.astype(np.complex64))
        h5.create_dataset("I", data=np.real(y).astype(np.float32))
        h5.create_dataset("Q", data=np.imag(y).astype(np.float32))
        for obj in (h5, d):
            obj.attrs["sample_rate_hz"] = fs
            obj.attrs["cutoff_hz"] = F_CUTOFF
            obj.attrs["filter_taps"] = H_LP
            obj.attrs["group_delay_samples"] = (len(H_LP) - 1) // 2
            obj.attrs["source_file"] = os.path.basename(src)


def main(argv=None):
    p = argparse.ArgumentParser(description="Low-pass filter (2.75 MHz) the 20 Msps decimated Rx waveform.")
    p.add_argument("Rx_Input", nargs="?", default=DEFAULT_IN, help=f"input HDF5 file (default: {DEFAULT_IN})")
    p.add_argument("-o", "--output", default=DEFAULT_OUT, help=f"output HDF5 file (default: {DEFAULT_OUT})")
    p.add_argument("--dataset", help="input dataset name (default: Rx_Decimated)")
    p.add_argument("--plot", action="store_true", help="plot input vs filtered waveform and spectra")
    a = p.parse_args(argv)

    x, fs, dname = read_rx_input(a.Rx_Input, a.dataset)
    fs = fs or FS_DEFAULT

    y = lp_filter_flexlink(x)
    write_output(a.output, y, fs, a.Rx_Input)

    print(f"Input : {a.Rx_Input} dataset '{dname}', {len(x)} samples @ {fs/1e6:g} Msps")
    print(f"Filter: {len(H_LP)}-tap FIR, Fc = {F_CUTOFF/1e6:g} MHz, delay {(len(H_LP)-1)//2} samples")
    print(f"Output: {a.output} datasets 'Rx_Filtered', 'I', 'Q', {len(y)} samples")

    if a.plot:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(10, 7))
        t = np.arange(len(x)) / fs * 1e6
        ax[0].plot(t, np.abs(x), label="Input |x|", alpha=0.6)
        ax[0].plot(t, np.abs(y), label="LP filtered |y|")
        ax[0].set_xlabel("Time (µs)"); ax[0].set_ylabel("Magnitude"); ax[0].legend(); ax[0].grid(True)
        for sig, lbl in ((x, "Input"), (y, "LP filtered")):
            f, pxx = signal.welch(sig, fs=fs, nperseg=min(256, len(sig)), return_onesided=False)
            ax[1].plot(np.fft.fftshift(f) / 1e6, 10 * np.log10(np.fft.fftshift(pxx) + 1e-20), label=lbl)
        w, hresp = signal.freqz(H_LP, worN=1024, whole=True, fs=fs)
        ax[1].plot(np.fft.fftshift(w - fs * (w >= fs / 2)) / 1e6,
                   np.fft.fftshift(20 * np.log10(np.abs(hresp) + 1e-12)), "k--", label="|H(f)| (dB)")
        ax[1].set_xlabel("Frequency (MHz)"); ax[1].set_ylabel("dB"); ax[1].legend(); ax[1].grid(True)
        fig.tight_layout()
        plt.show()
    return y


if __name__ == "__main__":
    main()
