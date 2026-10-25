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

Timing method (--method, or Defined_Rx/TimingMethod when called from Rx/Packet_Detector.py):
    "Full Precision Taps" (default, = MATLAB), "1-bit Taps", "FFT method".
    See the TIMING_METHODS block below.

Input:
    Rx_Input : Test_Preamble_dec2.h5 (default, dataset Rx_Decimated). Use the
               UNFILTERED 20 Msps waveform: the 2.75 MHz LP filter removes about 71%
               of Preamble B's energy. This standalone script does not remove the
               frequency offset, so it gives correct timing only on a zero-CFO test
               vector; Rx/Packet_Detector.py corrects CFO before calling it.
Output:
    Test_Preamble_Timing_Offset.h5
        attrs  PeakPosition (MATLAB 1-based), PeakIndex (0-based Python),
               PeakValue_real/imag, PeakMagnitude
        datasets Output (complex64 cross-correlation), PreambleB_Local

Usage:
    python Receive_Get_Timing_Offset.py
    python Receive_Get_Timing_Offset.py Test_Preamble_dec2.h5 --plot
    python Receive_Get_Timing_Offset.py --method "FFT method" --start-of-packet 199
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
DEFAULT_IN = "Test_Preamble_dec2.h5"
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


def read_complex(path, prefer=("Rx_Decimated", "Rx_Filtered")):
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
# Timing methods (Defined_Rx/TimingMethod in Rx/Parameters_Rx_payload.json)
#
#   "Full Precision Taps" : the MATLAB matched filter - 1140 complex taps, every sample.
#   "1-bit Taps"          : same matched filter, each tap quantised to +-1 +-j, so the
#                           hardware needs only adders (no multipliers).
#   "FFT method"          : frequency-domain search that reuses a 1024-point FFT. A
#                           1024-sample window that starts within +-FFT_MAX_OFFSET of
#                           the Preamble B body start gives a circular correlation whose
#                           peak index is minus the offset (the cyclic prefix makes the
#                           correlation circular). Windows are placed FFT_WINDOW_STEP
#                           apart over the region where Preamble B can be; the window
#                           with the highest peak-to-rms ratio wins.
#
# All three return the same PeakPosition (1-based index of the last Preamble B sample).
# Given the detection index, "1-bit Taps" and "FFT method" search only the hardware window
# (search_bounds); "Full Precision Taps" stays the MATLAB whole-record search.
# --------------------------------------------------------------------------- #
TIMING_METHODS = ("Full Precision Taps", "1-bit Taps", "FFT method")
DEFAULT_TIMING_METHOD = TIMING_METHODS[0]

PB_CP_LEN_20M = 116            # Preamble B cyclic prefix at 20 Msps
PB_BODY_LEN_20M = 1024         # Preamble B body (= FFT size) at 20 Msps
PREAMBLE_A_LEN_20M = 5000      # Preamble A at 20 Msps
FFT_WINDOW_STEP = 800          # window spacing; each window tolerates about +-450 samples
FFT_MAX_OFFSET = 512           # circular correlation indices k >= this are negative offsets
PB_LEN_20M = PB_CP_LEN_20M + PB_BODY_LEN_20M   # 1140
SEARCH_GUARD = 32              # extra samples each side of the search window
DEFAULT_CFO_WINDOW = 2048      # N_CFO of the lag-cascade CFO method
DEFAULT_LPF_DELAY = 8          # StartOfPacket is on the 2.75 MHz-LPF stream (17 taps)


def search_bounds(start_of_packet, cfo_window=DEFAULT_CFO_WINDOW, lpf_delay=DEFAULT_LPF_DELAY):
    """Hardware search window (spec section 4.5). The detection latency (Preamble A start ->
    StartOfPacket) lies in 0 .. L_A - N_CFO, because later triggers are aborted by the
    late-trigger check. Returns (lo, hi) for the 0-based index of the LAST Preamble B sample,
    including SEARCH_GUARD each side. With N_CFO = 2048: StartOfPacket + 3147 .. + 6163."""
    lat_max = PREAMBLE_A_LEN_20M - int(cfo_window)
    end = int(start_of_packet) + PREAMBLE_A_LEN_20M + PB_LEN_20M - 1 - int(lpf_delay)
    return end - lat_max - SEARCH_GUARD, end + SEARCH_GUARD


def normalize_timing_method(method):
    """Accept the TimingMethod string with any case/extra spaces; return the canonical name."""
    if method is None:
        return DEFAULT_TIMING_METHOD
    key = " ".join(str(method).split()).lower()
    for m in TIMING_METHODS:
        if m.lower() == key:
            return m
    raise ValueError(f"TimingMethod {method!r} must be one of {', '.join(TIMING_METHODS)}")


def _matched_filter(x, taps):
    """Output(i) = sum_j x(i-L+j) * conj(taps(j)); peaks when the last tap has entered."""
    h = np.conj(taps[::-1])                              # = conj(fliplr(PreambleB))
    return signal.convolve(x, h, mode="full")[:len(x)]


def _fft_search(x, preamble_b_local, bounds=None):
    """FFT-domain timing search. bounds = (lo, hi) for the last Preamble B sample, or None to
    search the whole record. Returns (Output, chosen 0-based end index or -1, the chosen
    window's circular-correlation magnitudes squared). Output (len(x)) holds, at each
    candidate end index, the circular-correlation value from the window that tested it."""
    n = len(x)
    body = preamble_b_local[PB_CP_LEN_20M:PB_CP_LEN_20M + PB_BODY_LEN_20M]
    ref_spec = np.conj(np.fft.fft(body))                 # stored reference spectrum (ROM)

    # Region where the Preamble B body can start (body start = end - 1023)
    if bounds is None:
        lo, hi = 0, n - PB_BODY_LEN_20M
    else:
        lo, hi = bounds[0] - (PB_BODY_LEN_20M - 1), bounds[1] - (PB_BODY_LEN_20M - 1)
    lo, hi = max(lo, 0), min(hi, n - PB_BODY_LEN_20M)

    output = np.zeros(n, dtype=np.complex128)
    best_score, best_end, best_p = -1.0, -1, None
    if hi < lo:
        return output, best_end, best_p
    # Window starts every FFT_WINDOW_STEP, the first half a step in; each window finds a body
    # start within about +-450 samples, so the windows cover lo .. hi
    count = int(np.ceil((hi - lo + 1) / FFT_WINDOW_STEP))
    starts = [min(lo + FFT_WINDOW_STEP // 2 + j * FFT_WINDOW_STEP, hi) for j in range(count)]
    offs = np.arange(PB_BODY_LEN_20M)
    offs[offs >= FFT_MAX_OFFSET] -= PB_BODY_LEN_20M
    for s in starts:
        cc = np.fft.ifft(np.fft.fft(x[s:s + PB_BODY_LEN_20M]) * ref_spec)
        p = np.abs(cc) ** 2
        k = int(np.argmax(p))
        ends = s + offs + PB_BODY_LEN_20M - 1             # 'last Preamble B sample' per index
        ok = (ends >= 0) & (ends < n)
        keep = ok & (np.abs(cc) > np.abs(output[np.clip(ends, 0, n - 1)]))
        output[ends[keep]] = cc[keep]
        score = p[k] / p.mean() if p.mean() > 0 else 0.0
        if score > best_score and 0 <= ends[k] < n:
            best_score, best_end, best_p = score, int(ends[k]), p
    return output, best_end, best_p


def detect_timing_offset(signal_dec, preamble_b_local, method=DEFAULT_TIMING_METHOD,
                         start_of_packet=None, cfo_window=DEFAULT_CFO_WINDOW,
                         lpf_delay=DEFAULT_LPF_DELAY, return_info=False):
    """Port of Detect_TimingOffset_Flexlink, with a choice of timing method.

    method          : one of TIMING_METHODS (default "Full Precision Taps" = MATLAB).
    start_of_packet : 0-based packet-detection index (on the LP-filtered stream). When given,
                      "1-bit Taps" and "FFT method" search only the hardware window
                      (search_bounds; spec section 4.5). "Full Precision Taps" always searches
                      the whole record, as MATLAB does. None = whole record for all methods.
    cfo_window      : N_CFO of the CFO method used (2048 lag cascade, 4096 FFT); sets the window.
    lpf_delay       : delay of the detection stream relative to signal_dec (8 samples).
    return_info     : also return a dict with SearchStart/SearchEnd (0-based end-index window,
                      -1 if the whole record) and PeakToMean (|peak|^2 / mean |y|^2 over the
                      searched positions; for the FFT method, over the chosen window).

    Returns (PeakPosition, Output, PeakValue[, info]). PeakPosition is MATLAB 1-based
    (0 if nothing is found / the input is all zeros, as in MATLAB); PeakPosition-1 is
    the 0-based index of the last Preamble B sample in signal_dec.
    """
    method = normalize_timing_method(method)
    x = np.asarray(signal_dec, dtype=np.complex128)
    pb = np.asarray(preamble_b_local, dtype=np.complex128)
    windowed = start_of_packet is not None and method != "Full Precision Taps"
    bounds = search_bounds(start_of_packet, cfo_window, lpf_delay) if windowed else None
    info = {"SearchStart": bounds[0] if bounds else -1, "SearchEnd": bounds[1] if bounds else -1,
            "PeakToMean": 0.0}

    def done(pos, out, val):
        return (pos, out, val, info) if return_info else (pos, out, val)

    if method == "FFT method":
        output, idx, p = _fft_search(x, pb, bounds)
        if idx < 0 or np.abs(output).max() == 0:
            return done(0, output, 0j)
        info["PeakToMean"] = float(p.max() / p.mean())
        return done(idx + 1, output, output[idx])

    taps = pb if method == "Full Precision Taps" else np.sign(pb.real) + 1j * np.sign(pb.imag)
    output = _matched_filter(x, taps)
    lo, hi = (max(bounds[0], 0), min(bounds[1], len(x) - 1)) if bounds else (0, len(x) - 1)
    if hi < lo:
        return done(0, output, 0j)
    p = np.abs(output[lo:hi + 1]) ** 2
    if p.max() == 0:
        return done(0, output, 0j)
    idx = lo + int(np.argmax(p))                         # first maximum, like the strict '>' loop
    info["PeakToMean"] = float(p.max() / p.mean())
    return done(idx + 1, output, output[idx])


# --------------------------------------------------------------------------- #
def write_output(path, peak_pos, peak_val, output, pb, src, pre_src, method=DEFAULT_TIMING_METHOD):
    with h5py.File(path, "w") as h5:
        h5.attrs["TimingMethod"] = method
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
    p.add_argument("--method", default=DEFAULT_TIMING_METHOD, help=f"timing method: {', '.join(TIMING_METHODS)}")
    p.add_argument("--start-of-packet", type=int,
                   help="0-based detection index (FFT method window placement; default: search the whole record)")
    p.add_argument("--plot", action="store_true", help="plot the cross-correlation (MATLAB figure 34)")
    a = p.parse_args(argv)

    pb = load_preamble_b(a.preamble_file)
    x, dname = read_complex(a.Rx_Input)
    method = normalize_timing_method(a.method)
    peak_pos, output, peak_val, tinfo = detect_timing_offset(x, pb, method, a.start_of_packet,
                                                             return_info=True)
    write_output(a.output, peak_pos, peak_val, output, pb, a.Rx_Input, a.preamble_file, method)

    print(f"Input        : {a.Rx_Input} dataset '{dname}', {len(x)} samples")
    print(f"Preamble B   : {a.preamble_file} ({len(pb)} samples @ 20 Msps)")
    print(f"Method       : {method}")
    if tinfo["SearchStart"] >= 0:
        print(f"Search window: last Preamble B sample in {tinfo['SearchStart']} .. {tinfo['SearchEnd']} (0-based)")
    print(f"PeakPosition : {peak_pos} (MATLAB 1-based; end of Preamble B)")
    print(f"               Preamble B starts at {peak_pos - len(pb) + 1}, |peak| = {abs(peak_val):.2f}")
    print(f"Peak / mean  : {tinfo['PeakToMean']:.1f}")
    print(f"Output       : {a.output}")
    if a.plot:
        plot(output, peak_pos, len(pb))
    return peak_pos


if __name__ == "__main__":
    main()
