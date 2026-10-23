#!/usr/bin/env python3
"""
Receive_Get_Timing_Offset.py

Fifth receive-chain step: find packet timing by cross-correlating the
20 Msps received signal with a local copy of Preamble B.

Python equivalent of the MATLAB function:

    PeakPosition = Detect_TimingOffset_Flexlink(Signal_Dec)

MATLAB computes, for every input sample i (shift register starts at zero):
    Output(i) = ShiftRegister * fliplr(PreambleB_Local)'
              = sum_{j=1..L} Signal_Dec(i-L+j) * conj(PreambleB_Local(j))
i.e. a matched filter whose output peaks when the LAST sample of Preamble B
has just entered. PeakPosition is the first index of the largest |Output|.
Here it is computed with one convolution (same values as the loop).

Local Preamble B (replaces GeneratePreambleB_Flexlink(20e6)):
    Read from Preamble.h5 in this script's folder (written by
    Construct_Preamble.py): dataset 'Preamble' with attributes SampleRate,
    AgcBurstLength, PreambleALength, PreambleBLength. The Preamble B part is
    cut out using those lengths. If SampleRate is 40 MHz every second sample
    is taken, which is exactly the 20 MHz Preamble B (1140 samples).
    A file holding only Preamble B (a single complex dataset, 1140 or 2280
    samples) is also accepted. Use --preamble-file to point elsewhere.

Input:
    Rx_Input : Test_Preamble_dec2_LP_Filtered.h5 (default, dataset Rx_Filtered)
Output:
    Test_Preamble_Timing_Offset.h5
        attrs  PeakPosition (MATLAB 1-based), PeakIndex (0-based Python),
               PeakValue_real/imag, PeakMagnitude
        datasets Output (complex64 cross-correlation), PreambleB_Local

Usage:
    python Receive_Get_Timing_Offset.py
    python Receive_Get_Timing_Offset.py Test_Preamble_dec2_LP_Filtered.h5 --plot
"""

import argparse
import os

import h5py
import numpy as np
from scipy import signal

FS = 20e6
PREAMBLE_B_LEN_20M = 1140

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PREAMBLE = os.path.join(HERE, "Preamble.h5")
DEFAULT_IN = "Test_Preamble_dec2_LP_Filtered.h5"
DEFAULT_OUT = "Test_Preamble_Timing_Offset.h5"


# --------------------------------------------------------------------------- #
def load_preamble_b(path=DEFAULT_PREAMBLE):
    """Return the 20 Msps Preamble B (1140 complex samples) from Preamble.h5."""
    if not os.path.isfile(path):
        raise FileNotFoundError(f"{path} not found - run Construct_Preamble.py first "
                                f"or pass --preamble-file")
    with h5py.File(path, "r") as h5:
        if "Preamble" in h5 and "PreambleBLength" in h5["Preamble"].attrs:
            d = h5["Preamble"]
            fs = float(d.attrs["SampleRate"])
            start = int(d.attrs["AgcBurstLength"]) + int(d.attrs["PreambleALength"])
            pb = np.asarray(d[()])[start:start + int(d.attrs["PreambleBLength"])]
        else:  # file holding Preamble B only
            names = []
            h5.visititems(lambda n, o: names.append(n)
                          if isinstance(o, h5py.Dataset) and np.issubdtype(o.dtype, np.complexfloating) else None)
            if not names:
                raise ValueError(f"No complex dataset in {path}")
            d = h5[names[0]]
            pb = np.squeeze(d[()])
            fs = float(d.attrs.get("SampleRate", 20e6 if len(pb) == PREAMBLE_B_LEN_20M else 40e6))

    pb = pb.astype(np.complex128)
    if fs == 40e6:
        pb = pb[::2]          # 40 MHz Preamble B sampled every 2nd sample == 20 MHz version
    elif fs != 20e6:
        raise ValueError(f"Unsupported SampleRate {fs} in {path}")
    if len(pb) != PREAMBLE_B_LEN_20M:
        raise ValueError(f"Preamble B has {len(pb)} samples at 20 MHz, expected {PREAMBLE_B_LEN_20M}")
    return pb


def read_complex(path, prefer=("Rx_Filtered", "Rx_Decimated")):
    with h5py.File(path, "r") as h5:
        name = next((n for n in prefer if n in h5), None)
        if name is None:
            names = []
            h5.visititems(lambda n, o: names.append(n)
                          if isinstance(o, h5py.Dataset) and np.issubdtype(o.dtype, np.complexfloating) else None)
            if not names:
                raise ValueError(f"No complex dataset found in {path}")
            name = names[0]
        return np.squeeze(h5[name][()]).astype(np.complex128), name


# --------------------------------------------------------------------------- #
def detect_timing_offset(signal_dec, preamble_b_local):
    """Port of Detect_TimingOffset_Flexlink.

    Returns (PeakPosition, Output, PeakValue). PeakPosition is MATLAB 1-based
    (0 if the input is all zeros, as in MATLAB); PeakPosition-1 is the 0-based
    index of the last Preamble B sample in signal_dec.
    """
    x = np.asarray(signal_dec, dtype=np.complex128)
    h = np.conj(preamble_b_local[::-1])                  # = conj(fliplr(PreambleB))
    output = signal.convolve(x, h, mode="full")[:len(x)]
    mag = np.abs(output)
    if mag.max() == 0:
        return 0, output, 0j
    idx = int(np.argmax(mag))                            # first maximum, like the strict '>' loop
    return idx + 1, output, output[idx]


# --------------------------------------------------------------------------- #
def write_output(path, peak_pos, peak_val, output, pb, src, pre_src):
    with h5py.File(path, "w") as h5:
        h5.attrs["PeakPosition"] = peak_pos                   # MATLAB 1-based
        h5.attrs["PeakIndex"] = peak_pos - 1                  # Python 0-based
        h5.attrs["PeakValue_real"] = float(np.real(peak_val))
        h5.attrs["PeakValue_imag"] = float(np.imag(peak_val))
        h5.attrs["PeakMagnitude"] = float(abs(peak_val))
        h5.attrs["PreambleBLength"] = len(pb)
        h5.attrs["sample_rate_hz"] = FS
        h5.attrs["source_file"] = os.path.basename(src)
        h5.attrs["preamble_file"] = os.path.basename(pre_src)
        h5.create_dataset("Output", data=output.astype(np.complex64))
        h5.create_dataset("PreambleB_Local", data=pb.astype(np.complex64))


def plot(output, peak_pos, pb_len):
    import matplotlib.pyplot as plt
    idx = np.arange(1, len(output) + 1)
    fig, ax = plt.subplots(2, 1, figsize=(10, 7))
    ax[0].plot(idx, np.real(output))
    ax[0].set_title("Result of Crosscorrelation of Preamble B and RX Signal (real part)")
    ax[0].axvline(peak_pos, color="r", ls=":", label=f"PeakPosition = {peak_pos}")
    ax[0].legend()
    lo, hi = max(1, peak_pos - pb_len), min(len(output), peak_pos + 200)
    ax[1].plot(idx[lo - 1:hi], np.abs(output[lo - 1:hi]))
    ax[1].axvline(peak_pos, color="r", ls=":")
    ax[1].axvline(peak_pos - pb_len + 1, color="g", ls="--", label="start of Preamble B")
    ax[1].set_title("|Output| around the peak")
    ax[1].set_xlabel("Sample index (20 Msps, MATLAB 1-based)")
    ax[1].legend()
    for a in ax:
        a.grid(True)
    fig.tight_layout()
    plt.show()


def main(argv=None):
    p = argparse.ArgumentParser(description="Timing offset by cross-correlation with Preamble B.")
    p.add_argument("Rx_Input", nargs="?", default=DEFAULT_IN, help=f"20 Msps waveform (default {DEFAULT_IN})")
    p.add_argument("--preamble-file", default=DEFAULT_PREAMBLE,
                   help="Preamble.h5 from Construct_Preamble.py (default: next to this script)")
    p.add_argument("-o", "--output", default=DEFAULT_OUT, help=f"output HDF5 file (default {DEFAULT_OUT})")
    p.add_argument("--plot", action="store_true", help="plot the cross-correlation (MATLAB figure 34)")
    a = p.parse_args(argv)

    pb = load_preamble_b(a.preamble_file)
    x, dname = read_complex(a.Rx_Input)
    peak_pos, output, peak_val = detect_timing_offset(x, pb)
    write_output(a.output, peak_pos, peak_val, output, pb, a.Rx_Input, a.preamble_file)

    print(f"Input        : {a.Rx_Input} dataset '{dname}', {len(x)} samples")
    print(f"Preamble B   : {a.preamble_file} ({len(pb)} samples @ 20 Msps)")
    print(f"PeakPosition : {peak_pos} (MATLAB 1-based; end of Preamble B)")
    print(f"               Preamble B starts at {peak_pos - len(pb) + 1}, |peak| = {abs(peak_val):.2f}")
    print(f"Output       : {a.output}")
    if a.plot:
        plot(output, peak_pos, len(pb))
    return peak_pos


if __name__ == "__main__":
    main()
