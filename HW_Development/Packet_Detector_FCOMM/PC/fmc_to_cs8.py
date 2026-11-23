"""
fmc_to_cs8.py  (Flexlink_py/HW_Development/Packet_Detector_FCOMM/PC)

Converts an FMCOMMS3 capture (fmc_capture.py: .cs16, 12-bit values) into the int8 .cs8 + .json
that App/FlexlinkProcessCoreBoardCapturedWaveform.py decodes, so the CoreBoard decoder runs on it
unchanged:

    python fmc_to_cs8.py pluto_g50.cs16          # -> cs8/pluto_g50.cs8 + cs8/pluto_g50.json
    cd ~/SiskinRF/Flexlink_py
    python App/FlexlinkProcessCoreBoardCapturedWaveform.py \\
        --capture HW_Development/Packet_Detector_FCOMM/PC/cs8/pluto_g50.cs8 --no-repair

The samples are divided by a power of two, the smallest that keeps the peak within int8
(rounded, so no bias). --no-repair: the ring-lap repair is for the ESP32 CoreBoard's DMA fault
and has nothing to repair here.
"""
import argparse
import json
import os
import sys

import numpy as np


def main():
    ap = argparse.ArgumentParser(description="FMCOMMS3 .cs16 -> int8 .cs8 for the CoreBoard decoder")
    ap.add_argument("capture", help="the .cs16 file of fmc_capture.py")
    ap.add_argument("--outdir", help="output folder (default: cs8/ next to the capture)")
    a = ap.parse_args()

    src = a.capture
    if not src.endswith(".cs16") or not os.path.isfile(src):
        sys.exit(f"{src}: expected an existing .cs16 file")
    meta_in = {}
    if os.path.isfile(src[:-5] + ".json"):
        with open(src[:-5] + ".json") as f:
            meta_in = json.load(f)

    x = np.fromfile(src, dtype="<i2")
    peak = int(np.abs(x).max()) if x.size else 0
    shift = 0
    while (peak + (1 << shift >> 1)) >> shift > 127:
        shift += 1
    y = np.clip((x.astype(np.int32) + ((1 << shift) >> 1 if shift else 0)) >> shift, -128, 127).astype(np.int8)

    outdir = a.outdir or os.path.join(os.path.dirname(os.path.abspath(src)), "cs8")
    os.makedirs(outdir, exist_ok=True)
    base = os.path.splitext(os.path.basename(src))[0]
    out = os.path.join(outdir, base + ".cs8")
    y.tofile(out)

    yy = y.reshape(-1, 2).astype(np.float64)
    meta = {
        "File": base + ".cs8",
        "Format": "int8 interleaved I,Q (cs8)",
        "SampleRate": int(meta_in.get("sample_rate", 40_000_000)),
        "CarrierHz": int(meta_in.get("lo_hz", 0)),
        "Samples": int(yy.shape[0]),
        "Seconds": yy.shape[0] / float(meta_in.get("sample_rate", 40_000_000)),
        "CapturedAt": meta_in.get("time", ""),
        "Board": "Zedboard + AD-FMCOMMS3 (fmc_capture.py)",
        "AGC": 0 if meta_in.get("gain_mode", "manual") == "manual" else 1,
        "Gain": meta_in.get("gain_db"),
        "Bandwidth": int(meta_in.get("rf_bw_hz", 0)),
        "Source": os.path.basename(src),
        "ScaleShift": shift,
        "Peak": int(np.abs(y).max()) if y.size else 0,
        "RmsPerComponent": float(np.sqrt((yy ** 2).mean())) if yy.size else 0.0,
        "ClipFraction": float((np.abs(y) >= 127).mean()) if y.size else 0.0,
        "DC": [float(yy[:, 0].mean()), float(yy[:, 1].mean())] if yy.size else [0.0, 0.0],
        "LostSamples": 0,
        "Overflow": bool(meta_in.get("overflow", False)),
    }
    with open(os.path.join(outdir, base + ".json"), "w") as f:
        json.dump(meta, f, indent=2)
    print(f"{out}: {meta['Samples']} samples, divided by {1 << shift}, peak {meta['Peak']}, "
          f"RMS {meta['RmsPerComponent']:.1f} per component")


if __name__ == "__main__":
    main()
