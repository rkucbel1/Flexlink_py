#!/usr/bin/env python3
"""
Receive_Packet_Detector.py

Third receive-chain step: delay-and-correlate packet detector on the
20 Msps, low-pass-filtered preamble.

Python equivalent of the MATLAB function:

    PacketDetFlag = Packet_Detector_Flexlink(RX_Input, SA_Len,
                                             Comparison_Ratio_Thresh_Lo,
                                             Comparison_Ratio_Thresh_Hi)

For each sample i (all delay lines start at zero, as in the MATLAB code):
    AutoCorrEst(i)      = (1/SA_Len) * sum_{k=0}^{SA_Len-1} x(i-k) * conj(x(i-k-32))
    Variance_Est(i)     = (1/SA_Len) * sum_{k=0}^{SA_Len-1} |x(i-k)|^2
    Comparison_Ratio(i) = |AutoCorrEst(i)| / Variance_Est(i)
    Flag -> 1 when Comparison_Ratio > Thresh_Hi,
    Flag -> 0 when Comparison_Ratio < Thresh_Lo, otherwise held (hysteresis).
When Variance_Est is 0 (e.g. leading zeros) the ratio is NaN, exactly as in
MATLAB; NaN fails both comparisons, so the flag holds its previous value.

Options (Defined_Rx/PacketDetNormaliser and PacketDetQualifier when called from
Rx/Packet_Detector.py; --normaliser / --qualifier here). The defaults give the MATLAB
behaviour above.
    normaliser "Original" : Comparison_Ratio = |AutoCorrEst| / Variance_Est (MATLAB).
    normaliser "Bounded"  : Comparison_Ratio = |AutoCorrEst| / sqrt(Variance_Est * Variance_Est_D),
                            Variance_Est_D = the same average over x(i-k-32). By Cauchy-Schwarz the
                            ratio never exceeds 1, which removes the false trigger ~SA_Len samples
                            after a strong signal ends. Also blanks the first SA_Len + 32 samples
                            (flag held at 0) while the averaging windows fill.
    qualifier Q           : 0 (or 1) = the flag sets on the first sample above Thresh_Hi (MATLAB).
                            Q > 1 = the flag sets only once Q samples above Thresh_Hi have been
                            counted; a sample below Thresh_Lo resets the count, samples in between
                            hold it. Noise rarely stays high that long, so Thresh_Hi can be lowered
                            (e.g. 0.35 with Q = 128 for -12 dB). Adds Q samples of latency.

The sliding sums are computed with convolutions (same result as the MATLAB
shift-register loop, much faster); only the hysteresis is a sample loop.

Zero padding: --pad-front / --pad-end add zeros before / after the input
(default none). The delay line and averages already start at zero, so no
padding is needed to avoid overrun. Note that zeros at the END make the
ratio jump as the last few samples are averaged against zeros, which can raise
a false flag, and once the windows hold only zeros the ratio is NaN and the
flag freezes. All outputs cover the padded signal; the attribute 'pad_front'
lets you map indices back to the unpadded input.

    Rx_Input  : Test_Preamble_dec2_LP_Filtered.h5   (output of Receive_LP_Filter.py)
    Rx_Output : Test_Preamble_Detection_Flag.h5

Output datasets:
    PacketDetFlag     (int8)     detection flag, 0/1
    Comparison_Ratio  (float64)  |autocorr| / variance
    AutoCorrEst       (complex64)
    Variance_Est      (float64)
    Rx_Input_Padded   (complex64) the (padded) input that was processed, plus I and Q

Usage:
    python Receive_Packet_Detector.py
    python Receive_Packet_Detector.py --sa-len 256 --thresh-lo 0.2 --thresh-hi 0.45 --plot

Defaults for --sa-len, --thresh-lo and --thresh-hi come from
Receive_Packet_Detector.json next to this script; command-line options override them.
"""

import argparse
import json
import os

import h5py
import numpy as np

FS_DEFAULT = 20e6
DELAY = 32                      # autocorrelation lag, samples (Delay32 in MATLAB)
NORMALISERS = ("Original", "Bounded")
DEFAULT_NORMALISER = NORMALISERS[0]
DEFAULT_QUALIFIER = 0
QUALIFIER_MAX = 512             # detection + qualifier + 2048-sample CFO window must fit in Preamble A

# Detector parameter defaults are read from Receive_Packet_Detector.json in the
# same folder as this script. These values are used only if that file is missing.
CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Receive_Packet_Detector.json")
_BUILTIN = {"sa_len": 256, "thresh_lo": 0.2, "thresh_hi": 0.45}


def load_defaults(path=CONFIG_FILE):
    cfg = dict(_BUILTIN)
    if os.path.isfile(path):
        with open(path, "r") as f:
            user = json.load(f)
        # Keys starting with "Comment" (any case) are notes and are ignored
        user = {k: v for k, v in user.items() if not k.lower().startswith("comment")}
        unknown = set(user) - set(cfg)
        if unknown:
            raise ValueError(f"{path}: unknown keys {sorted(unknown)} (expected {sorted(cfg)})")
        cfg.update(user)
    else:
        print(f"Note: {path} not found, using built-in defaults {cfg}")
    return int(cfg["sa_len"]), float(cfg["thresh_lo"]), float(cfg["thresh_hi"])


SA_LEN_DEFAULT, THRESH_LO_DEFAULT, THRESH_HI_DEFAULT = load_defaults()

DEFAULT_IN = "Test_Preamble_dec2_LP_Filtered.h5"
DEFAULT_OUT = "Test_Preamble_Detection_Flag.h5"


def read_rx_input(path, dataset=None):
    """Return (complex waveform, sample rate or None, dataset name)."""
    with h5py.File(path, "r") as h5:
        if dataset is None:
            if "Rx_Filtered" in h5:
                dataset = "Rx_Filtered"
            else:
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


def normalize_normaliser(value):
    """Accept the PacketDetNormaliser string with any case/extra spaces; return the canonical name."""
    if value is None:
        return DEFAULT_NORMALISER
    key = " ".join(str(value).split()).lower()
    for m in NORMALISERS:
        if m.lower() == key:
            return m
    raise ValueError(f"PacketDetNormaliser {value!r} must be one of {', '.join(NORMALISERS)}")


def check_qualifier(value):
    """PacketDetQualifier: whole number of samples, 0..QUALIFIER_MAX (0 = off)."""
    try:
        q = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"PacketDetQualifier {value!r} must be a whole number of samples") from None
    if not q.is_integer() or not 0 <= q <= QUALIFIER_MAX:
        raise ValueError(f"PacketDetQualifier {value!r} must be a whole number 0..{QUALIFIER_MAX}")
    return int(q)


def packet_detector_flexlink(rx_input, sa_len, thresh_lo, thresh_hi, delay=DELAY,
                             normaliser=DEFAULT_NORMALISER, qualifier=DEFAULT_QUALIFIER):
    """Return (PacketDetFlag, Comparison_Ratio, AutoCorrEst, Variance_Est).
    Defaults (normaliser "Original", qualifier 0) reproduce the MATLAB detector exactly."""
    normaliser = normalize_normaliser(normaliser)
    qualifier = check_qualifier(qualifier)
    x = np.asarray(rx_input, dtype=np.complex128)
    n = len(x)

    # Delay32 output: x delayed by 'delay' samples, zeros before the start
    x_d = np.concatenate([np.zeros(delay, dtype=complex), x])[:n]

    # Sliding averages over SA_Len samples (zero initial state)
    win = np.ones(sa_len)
    autocorr = np.convolve(x * np.conj(x_d), win)[:n] / sa_len
    variance = np.convolve(np.abs(x) ** 2, win)[:n] / sa_len

    with np.errstate(divide="ignore", invalid="ignore"):
        if normaliser == "Bounded":
            variance_d = np.concatenate([np.zeros(delay), variance])[:n]   # same average over x(i-k-32)
            ratio = np.abs(autocorr) / np.sqrt(variance * variance_d)
            blank = sa_len + delay                                          # windows still filling
        else:
            ratio = np.abs(autocorr) / variance
            blank = 0

    # Hysteresis detection flag (+ qualifier count when qualifier > 1)
    need = max(qualifier, 1)
    flag = np.zeros(n, dtype=np.int8)
    state, count = 0, 0
    for i, r in enumerate(ratio):
        if i < blank:
            continue
        if r > thresh_hi:
            count += 1
            if count >= need:
                state = 1
        elif r < thresh_lo:
            state, count = 0, 0
        flag[i] = state
    return flag, ratio, autocorr, variance


def write_output(path, flag, ratio, autocorr, variance, x, fs, params, src):
    with h5py.File(path, "w") as h5:
        h5.create_dataset("PacketDetFlag", data=flag)
        h5.create_dataset("Comparison_Ratio", data=ratio)
        h5.create_dataset("AutoCorrEst", data=autocorr.astype(np.complex64))
        h5.create_dataset("Variance_Est", data=variance)
        h5.create_dataset("Rx_Input_Padded", data=x.astype(np.complex64))
        h5.create_dataset("I", data=np.real(x).astype(np.float32))
        h5.create_dataset("Q", data=np.imag(x).astype(np.float32))
        h5.attrs["sample_rate_hz"] = fs
        h5.attrs["source_file"] = os.path.basename(src)
        for k, v in params.items():
            h5.attrs[k] = v


def edges(flag):
    """Indices where the flag rises and falls."""
    d = np.diff(np.concatenate([[0], flag.astype(int)]))
    return np.flatnonzero(d == 1), np.flatnonzero(d == -1)


def plot(x, ratio, flag, thresh_lo, thresh_hi, pad_front):
    import matplotlib.pyplot as plt
    idx = np.arange(1, len(x) + 1)          # 1-based sample index, as in the MATLAB plots
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    ax[0].plot(idx, np.abs(x))
    ax[0].set_title("Packet Detector - Input magnitude")
    ax[0].set_ylabel("|x|")

    ax[1].plot(idx, ratio, label="Comparison Ratio")
    ax[1].axhline(thresh_hi, color="r", ls="--", label=f"Thresh Hi = {thresh_hi:g}")
    ax[1].axhline(thresh_lo, color="g", ls="--", label=f"Thresh Lo = {thresh_lo:g}")
    ax[1].set_title("Packet Detector - Comparison Ratio")
    ax[1].legend(loc="upper right")

    ax[2].plot(idx, flag, drawstyle="steps-post")
    ax[2].set_ylim(-0.1, 1.1)
    ax[2].set_title("Packet Detector - Packet Detect Flag")
    ax[2].set_xlabel("Sample index (20 Msps)")

    for a in ax:
        a.grid(True)
        if pad_front:
            a.axvline(pad_front + 0.5, color="k", lw=0.8, ls=":")
    fig.tight_layout()
    plt.show()


def main(argv=None):
    p = argparse.ArgumentParser(description="Delay-and-correlate packet detector (Packet_Detector_Flexlink).")
    p.add_argument("Rx_Input", nargs="?", default=DEFAULT_IN, help=f"input HDF5 file (default: {DEFAULT_IN})")
    p.add_argument("-o", "--output", default=DEFAULT_OUT, help=f"output HDF5 file (default: {DEFAULT_OUT})")
    p.add_argument("--dataset", help="input dataset name (default: Rx_Filtered)")
    p.add_argument("--sa-len", type=int, default=SA_LEN_DEFAULT, help=f"sliding average length (default {SA_LEN_DEFAULT}, from JSON)")
    p.add_argument("--thresh-lo", type=float, default=THRESH_LO_DEFAULT, help=f"low threshold (default {THRESH_LO_DEFAULT}, from JSON)")
    p.add_argument("--thresh-hi", type=float, default=THRESH_HI_DEFAULT, help=f"high threshold (default {THRESH_HI_DEFAULT}, from JSON)")
    p.add_argument("--normaliser", default=DEFAULT_NORMALISER, help=f"{' or '.join(NORMALISERS)} (default {DEFAULT_NORMALISER})")
    p.add_argument("--qualifier", type=int, default=DEFAULT_QUALIFIER,
                   help=f"samples above Thresh_Hi before the flag sets, 0..{QUALIFIER_MAX} (default 0 = off)")
    p.add_argument("--pad-front", type=int, default=0, help="zeros to add before the input (default 0)")
    p.add_argument("--pad-end", type=int, default=0, help="zeros to add after the input (default 0)")
    p.add_argument("--plot", action="store_true", help="plot comparison ratio and detection flag")
    a = p.parse_args(argv)

    if a.thresh_lo > a.thresh_hi:
        p.error("--thresh-lo must not be greater than --thresh-hi")
    pad_end = a.pad_end

    x0, fs, dname = read_rx_input(a.Rx_Input, a.dataset)
    fs = fs or FS_DEFAULT
    x = np.concatenate([np.zeros(a.pad_front, complex), x0, np.zeros(pad_end, complex)])

    flag, ratio, autocorr, variance = packet_detector_flexlink(x, a.sa_len, a.thresh_lo, a.thresh_hi,
                                                               normaliser=a.normaliser, qualifier=a.qualifier)

    params = dict(Normaliser=normalize_normaliser(a.normaliser), Qualifier=a.qualifier,
                  SA_Len=a.sa_len, Comparison_Ratio_Thresh_Lo=a.thresh_lo,
                  Comparison_Ratio_Thresh_Hi=a.thresh_hi, delay_samples=DELAY,
                  pad_front=a.pad_front, pad_end=pad_end, input_length=len(x0))
    write_output(a.output, flag, ratio, autocorr, variance, x, fs, params, a.Rx_Input)

    rise, fall = edges(flag)
    print(f"Input : {a.Rx_Input} dataset '{dname}', {len(x0)} samples @ {fs/1e6:g} Msps "
          f"(+{a.pad_front} front / +{pad_end} end zeros)")
    print(f"Params: SA_Len={a.sa_len}, Thresh_Lo={a.thresh_lo:g}, Thresh_Hi={a.thresh_hi:g}, delay={DELAY}")
    print(f"Ratio : max {np.nanmax(ratio):.3f}")
    if len(rise):
        for r in rise:
            f_ = fall[fall > r]
            end = f"falls at sample {f_[0] + 1}" if len(f_) else "still set at end of data"
            print(f"Detect: flag rises at sample {r + 1} ({(r - a.pad_front) / fs * 1e6:.2f} µs into input), {end}")
    else:
        print("Detect: no packet detected")
    print(f"Output: {a.output}")

    if a.plot:
        plot(x, ratio, flag, a.thresh_lo, a.thresh_hi, a.pad_front)
    return flag


if __name__ == "__main__":
    main()
