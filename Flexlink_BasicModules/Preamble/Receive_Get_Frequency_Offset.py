#!/usr/bin/env python3
"""
Receive_Get_Frequency_Offset.py

Fourth receive-chain step: estimate the carrier frequency offset from
Preamble A (two tones at 625 kHz and 1.875 MHz, period 32 samples at 20 Msps).

Python equivalent of the MATLAB function:

    [FreqOffset_Course, FreqOffset_Fine] = Detect_FrequencyOffset_Flexlink(DetFlag, RX_Input)

1. Coarse offset (FFT):
   - StartOfPacket = first sample where the detection flag is 1
   - take 4096 samples from there, apply a 512-point Hann taper to the first
     and last 256 samples, FFT, keep bins 0..449 and the last 449 bins
   - the peak in bins 5..251 is the 625 kHz tone (nominal bin 128, i.e. MATLAB
     index 129). Coarse offset = (peak bin - 128) * 20e6 / 4096  (4.88 kHz steps)
     Search range: -600.6 kHz .. +600.6 kHz (bins 5..251), symmetric about the
     nominal tone. (The MATLAB code searched bins 0..178, -625 .. +244 kHz.)
     Preamble A's tones (+-625 kHz, +-1.875 MHz) are 1.25 MHz apart, so no
     window can exceed +-625 kHz; at the edges a neighbouring tone is as strong
     as the wanted one. Bins 5..251 keep a ~25 kHz guard at both ends.
2. Fine offset (autocorrelation):
   - IFFT the filtered spectrum, remove the coarse offset with an NCO
   - lag-32 autocorrelation averaged over 64 samples
   - fine offset = mean of angle(AutoCorr_Est[549:3000]) * 20e6 / (2*pi*32)
     (MATLAB indices 550:3000). Unambiguous range +/-312.5 kHz.
Total offset = coarse + fine.

CFO methods (method= argument, or Defined_Rx/CFOMethod when called from Rx/Packet_Detector.py):
    "FFT"          : the MATLAB method above (default).
    "Lag cascade"  : no FFT. From StartOfPacket, accumulate the lagged products
                     A_l = s_l * sum x(n) x*(n-l) over CASCADE_WINDOW samples for
                     l = 16, 32, 128, 256 (s_16 = -1, the others +1). Preamble A's tones are
                     odd multiples of 625 kHz, so x(n-16) = -x(n) and lag 16 is unambiguous to
                     +-625 kHz; each longer lag refines the estimate and is unambiguous to
                     +-Fs/(2l): +-312.5, +-78.1, +-39.1 kHz. Each step resolves the previous
                     estimate's ambiguity. Coarse = the lag-16 estimate, fine = the
                     correction from the longer lags. About 0.9 kHz rms at -12 dB
                     (FFT method about 6 kHz). The 2048-sample window stays inside Preamble A
                     even when detection is late.

Inputs:
    Rx_Input  : Test_Preamble_dec2_LP_Filtered.h5 (default, dataset Rx_Filtered)
                or Test_Preamble_dec2.h5 (unfiltered, dataset Rx_Decimated)
    --flag-file: Test_Preamble_Detection_Flag.h5 (dataset PacketDetFlag;
                 its 'pad_front' attribute is used to map the index back)
Output:
    Test_Preamble_Frequency_Offset.h5 with attributes FreqOffset_Course,
    FreqOffset_Fine, FreqOffset_Total, StartOfPacket and datasets
    x_n, Y_m, y2_n, y3_n, AutoCorr_Est, Theta, FO.

Usage:
    python Receive_Get_Frequency_Offset.py
    python Receive_Get_Frequency_Offset.py Test_Preamble_dec2.h5 --plot
"""

import argparse
import os

import h5py
import numpy as np
from scipy.signal.windows import hann

FS = 20e6
SECTION_LENGTH = 4096
HANN_LENGTH = 512
DELAY = 32
AVG_LEN = 64
TONE_BIN = 128                 # 625 kHz * 4096 / 20 MHz  (MATLAB index 129)
SEARCH_LO = 5                  # coarse search bins SEARCH_LO .. SEARCH_HI-1 = 5..251
SEARCH_HI = 252                # (+-600.6 kHz around TONE_BIN; MATLAB used Y_m(1:179))
FINE_RANGE = slice(549, 3000)  # MATLAB AutoCorr_Est(550:3000)

CFO_METHODS = ("FFT", "Lag cascade")
DEFAULT_CFO_METHOD = CFO_METHODS[0]
CASCADE_LAGS = (16, 32, 128, 256)
CASCADE_WINDOW = 2048          # N_CFO: samples accumulated after StartOfPacket

DEFAULT_IN = "Test_Preamble_dec2_LP_Filtered.h5"
DEFAULT_FLAG = "Test_Preamble_Detection_Flag.h5"
DEFAULT_OUT = "Test_Preamble_Frequency_Offset.h5"


# --------------------------------------------------------------------------- #
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


def read_flag(path):
    with h5py.File(path, "r") as h5:
        flag = np.squeeze(h5["PacketDetFlag"][()])
        pad_front = int(h5.attrs.get("pad_front", 0))
    return flag, pad_front


# --------------------------------------------------------------------------- #
def detect_frequency_offset(det_flag, rx_input, pad_front=0):
    """Port of Detect_FrequencyOffset_Flexlink. Returns (course, fine, details dict)."""
    hits = np.flatnonzero(det_flag)
    if len(hits) == 0:
        raise ValueError("Detection flag is never set - no packet to process.")
    start = int(hits[0]) - pad_front                 # 0-based StartOfPacket in rx_input
    if start < 0:
        raise ValueError("Packet start falls inside the detector's front padding.")

    # 1a. 4096 samples from the start of packet (zero-pad if the record is too short)
    x_n = rx_input[start:start + SECTION_LENGTH]
    if len(x_n) < SECTION_LENGTH:
        x_n = np.concatenate([x_n, np.zeros(SECTION_LENGTH - len(x_n), complex)])

    # 1b. FFT filter: pass bins 0..449 and the last 449 bins
    h_m = np.zeros(SECTION_LENGTH)
    h_m[:450] = 1.0
    h_m[-449:] = 1.0
    w = hann(HANN_LENGTH, sym=True)                  # = MATLAB hann(512)
    mask = np.concatenate([w[:HANN_LENGTH // 2], np.ones(SECTION_LENGTH - HANN_LENGTH), w[HANN_LENGTH // 2:]])
    y_m = np.fft.fft(x_n * mask) * h_m
    y2_n = np.fft.ifft(y_m)                          # NumSections = 1

    # 1c. Coarse offset from the 625 kHz peak
    y_mag = np.abs(y_m)
    max_bin = SEARCH_LO + int(np.argmax(y_mag[SEARCH_LO:SEARCH_HI]))
    course = (max_bin - TONE_BIN) * FS / SECTION_LENGTH

    # 2a. Remove coarse offset (MATLAB n = 1:length)
    n = np.arange(1, len(y2_n) + 1)
    y3_n = y2_n * np.exp(-1j * 2 * np.pi * n * course / FS)

    # 2b. Lag-32 autocorrelation, 64-sample average (zero initial state)
    y3_d = np.concatenate([np.zeros(DELAY, complex), y3_n])[:len(y3_n)]
    autocorr = np.convolve(y3_n * np.conj(y3_d), np.ones(AVG_LEN))[:len(y3_n)] / AVG_LEN
    theta = np.angle(autocorr[FINE_RANGE])
    fo = theta * FS / (2 * np.pi * DELAY)
    fine = float(np.mean(fo))

    details = dict(StartOfPacket=start, x_n=x_n, Y_m=y_mag, y2_n=y2_n, y3_n=y3_n,
                   AutoCorr_Est=autocorr, Theta=theta, FO=fo, max_bin=max_bin)
    return course, fine, details


def normalize_cfo_method(method):
    """Accept the CFOMethod string with any case/extra spaces; return the canonical name."""
    if method is None:
        return DEFAULT_CFO_METHOD
    key = " ".join(str(method).split()).lower()
    for m in CFO_METHODS:
        if m.lower() == key:
            return m
    raise ValueError(f"CFOMethod {method!r} must be one of {', '.join(CFO_METHODS)}")


def detect_frequency_offset_cascade(det_flag, rx_input, pad_front=0, window=CASCADE_WINDOW,
                                    lags=CASCADE_LAGS):
    """Lag-cascade CFO estimate from Preamble A. Returns (course, fine, details) like
    detect_frequency_offset: course = lag-16 estimate, fine = refinement from the longer lags."""
    hits = np.flatnonzero(det_flag)
    if len(hits) == 0:
        raise ValueError("Detection flag is never set - no packet to process.")
    start = int(hits[0]) - pad_front
    if start < 0:
        raise ValueError("Packet start falls inside the detector's front padding.")

    x = rx_input[start:start + window]
    if len(x) < window:
        x = np.concatenate([x, np.zeros(window - len(x), complex)])

    estimates = []
    f_hat = None
    for lag in lags:
        a = np.sum(x[lag:] * np.conj(x[:-lag]))
        if lag % 32 == 16:                    # half a Preamble A period: x(n-16) = -x(n)
            a = -a
        f_lag = np.angle(a) * FS / (2 * np.pi * lag)
        if f_hat is None:
            f_hat = f_lag
        else:                                 # resolve this lag's ambiguity with the estimate so far
            step = FS / lag
            f_hat = f_lag + np.round((f_hat - f_lag) / step) * step
        estimates.append(f_lag)
    course = float(estimates[0])
    fine = float(f_hat - course)
    details = dict(StartOfPacket=start, x_n=x, Lags=np.array(lags), LagEstimates=np.array(estimates),
                   Window=window)
    return course, fine, details


# --------------------------------------------------------------------------- #
# Tone-structure check
#
# Preamble A is four tones at CFO + (-1.875, -0.625, +0.625, +1.875) MHz. A DC offset or a
# continuous interfering tone also passes the packet detector (it repeats at every lag), but
# it is ONE tone: the lag cascade then places it on one of the four tone positions and the
# other three hold only noise. The check measures, over the CFO window, the power at the four
# tone frequencies and at nine frequencies where Preamble A has none (the noise reference),
# and compares the three WEAKER tones with the noise:
#
#   ToneCheck_dB = 10 log10( (sum of the 4 tone powers - strongest) / mean noise-bin power )
#
# Preamble A: >= 14 dB down to -13 dB SNR (AWGN), >= 12 dB in static ETU at -10 dB.
# DC / CW interference: <= 12.7 dB (1250 trials). Default threshold 13 dB.
# --------------------------------------------------------------------------- #
TONE_OFFSETS = (-1.875e6, -0.625e6, 0.625e6, 1.875e6)
NOISE_REF_OFFSETS = (0.0, 1.25e6, -1.25e6, 0.3125e6, -0.3125e6,
                     1.5625e6, 0.9375e6, -0.9375e6, -1.5625e6)
TONE_CHECK_WINDOW = 2048


def tone_structure_check(rx_input, start, cfo, window=TONE_CHECK_WINDOW):
    """Return (ToneCheck_dB, tone powers, mean noise-reference power) for the samples
    rx_input[start:start+window] (the LP-filtered stream, from StartOfPacket) given the CFO."""
    seg = np.asarray(rx_input[start:start + window], dtype=np.complex128)
    if seg.size < window:
        seg = np.concatenate([seg, np.zeros(window - seg.size, complex)])
    n = np.arange(window)

    def power(f):                                   # single DFT bin at frequency f
        return abs(np.sum(seg * np.exp(-2j * np.pi * f * n / FS))) ** 2

    tones = np.array([power(cfo + f) for f in TONE_OFFSETS])
    noise = float(np.mean([power(cfo + f) for f in NOISE_REF_OFFSETS]))
    weaker = tones.sum() - tones.max()
    if noise <= 0:
        return (float("inf") if weaker > 0 else float("-inf")), tones, noise
    return 10 * np.log10(max(weaker, 1e-300) / noise), tones, noise


def estimate_frequency_offset(det_flag, rx_input, pad_front=0, method=DEFAULT_CFO_METHOD):
    """Dispatch on the CFO method. Returns (course, fine, details)."""
    method = normalize_cfo_method(method)
    if method == "Lag cascade":
        return detect_frequency_offset_cascade(det_flag, rx_input, pad_front)
    return detect_frequency_offset(det_flag, rx_input, pad_front)


# --------------------------------------------------------------------------- #
def write_output(path, course, fine, d, src, flag_src, method=DEFAULT_CFO_METHOD):
    with h5py.File(path, "w") as h5:
        h5.attrs["CFOMethod"] = method
        h5.attrs["FreqOffset_Course"] = course
        h5.attrs["FreqOffset_Fine"] = fine
        h5.attrs["FreqOffset_Total"] = course + fine
        h5.attrs["StartOfPacket"] = d["StartOfPacket"]          # 0-based
        h5.attrs["StartOfPacket_matlab"] = d["StartOfPacket"] + 1
        h5.attrs["sample_rate_hz"] = FS
        h5.attrs["source_file"] = os.path.basename(src)
        h5.attrs["flag_file"] = os.path.basename(flag_src)
        for k in ("x_n", "y2_n", "y3_n", "AutoCorr_Est"):
            if k in d:
                h5.create_dataset(k, data=d[k].astype(np.complex64))
        for k in ("Y_m", "Theta", "FO", "Lags", "LagEstimates"):
            if k in d:
                h5.create_dataset(k, data=d[k])


def plot(d, course, fine):
    import matplotlib.pyplot as plt
    y2, ym, ac, th, fo = d["y2_n"], d["Y_m"], d["AutoCorr_Est"], d["Theta"], d["FO"]
    fig, ax = plt.subplots(5, 1, figsize=(10, 13))
    ax[0].plot(np.real(y2), label="real"); ax[0].plot(np.imag(y2), label="imag")
    ax[0].set_title("PreambleA after FFT filter and then IFFT"); ax[0].legend()
    k = np.arange(1, len(ym) + 1)
    ax[1].stem(k, ym, markerfmt=" ", basefmt=" ")
    ax[1].axvline(TONE_BIN + 1, color="g", ls="--", label="nominal index 129")
    ax[1].axvline(d["max_bin"] + 1, color="r", ls=":", label=f"peak index {d['max_bin'] + 1}")
    ax[1].set_xlim(0, 500)
    ax[1].set_title(f"PreambleA FFT |Y_m| - coarse offset {course/1e3:.2f} kHz"); ax[1].legend()
    ax[2].plot(np.real(ac[:-1]), label="real"); ax[2].plot(np.imag(ac[:-1]), label="imag")
    ax[2].axvspan(FINE_RANGE.start, FINE_RANGE.stop, color="0.9")
    ax[2].set_title("AutoCorr Estimate (shaded = samples used for fine offset)"); ax[2].legend()
    ax[3].plot(np.arange(FINE_RANGE.start + 1, FINE_RANGE.start + len(th) + 1), th)
    ax[3].set_title("Theta from AutoCorr Estimate (rad)")
    ax[4].plot(np.arange(FINE_RANGE.start + 1, FINE_RANGE.start + len(fo) + 1), fo / 1e3)
    ax[4].axhline(fine / 1e3, color="r", ls="--", label=f"mean = {fine/1e3:.3f} kHz")
    ax[4].set_title("Fine Frequency Offset (kHz)"); ax[4].legend()
    for a in ax:
        a.grid(True)
    fig.tight_layout()
    plt.show()


def main(argv=None):
    p = argparse.ArgumentParser(description="Coarse + fine frequency offset from Preamble A.")
    p.add_argument("Rx_Input", nargs="?", default=DEFAULT_IN,
                   help=f"20 Msps waveform (default {DEFAULT_IN}; or Test_Preamble_dec2.h5 for unfiltered)")
    p.add_argument("--flag-file", default=DEFAULT_FLAG, help=f"detection flag file (default {DEFAULT_FLAG})")
    p.add_argument("-o", "--output", default=DEFAULT_OUT, help=f"output HDF5 file (default {DEFAULT_OUT})")
    p.add_argument("--method", default=DEFAULT_CFO_METHOD, help=f"CFO method: {', '.join(CFO_METHODS)}")
    p.add_argument("--plot", action="store_true", help="plot the MATLAB figures 23-27 (FFT method)")
    a = p.parse_args(argv)

    x, dname = read_complex(a.Rx_Input)
    flag, pad_front = read_flag(a.flag_file)
    method = normalize_cfo_method(a.method)
    course, fine, d = estimate_frequency_offset(flag, x, pad_front, method)
    write_output(a.output, course, fine, d, a.Rx_Input, a.flag_file, method)

    print(f"Input      : {a.Rx_Input} dataset '{dname}', {len(x)} samples")
    print(f"Flag file  : {a.flag_file}, StartOfPacket = sample {d['StartOfPacket'] + 1} (MATLAB index)")
    print(f"Method     : {method}")
    if "max_bin" in d:
        print(f"Coarse     : {course/1e3:+9.3f} kHz  (peak at index {d['max_bin'] + 1}, nominal 129)")
    else:
        print(f"Coarse     : {course/1e3:+9.3f} kHz  (lag 16)")
        print("Lags       : " + ", ".join(f"{l}: {f/1e3:+.3f} kHz" for l, f in zip(d["Lags"], d["LagEstimates"])))
    print(f"Fine       : {fine/1e3:+9.3f} kHz")
    print(f"Total      : {(course + fine)/1e3:+9.3f} kHz")
    print(f"Output     : {a.output}")
    if a.plot and method == "FFT":
        plot(d, course, fine)
    elif a.plot:
        print("--plot shows the FFT method's figures only")
    return course, fine


if __name__ == "__main__":
    main()
