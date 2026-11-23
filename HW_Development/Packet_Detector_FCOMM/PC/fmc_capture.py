"""
fmc_capture.py  (Flexlink_py/HW_Development/Packet_Detector_FCOMM/PC)

Step 3: capture raw AD9361 samples through the Zedboard (FW/fmc_server.c) and save them.

    python fmc_capture.py --status                      # PING + radio status, nothing changed
    python fmc_capture.py -o pluto_2420.cs16            # 0.5 s at 2420 MHz, 40 Msps, 30 dB manual
    python fmc_capture.py --lo 2420e6 --rate 20e6 --gain-mode slow_attack --seconds 0.2 -o x.cs16

Output: <name>.cs16 (interleaved little-endian int16 I, Q; 12-bit values, sign-extended) and
<name>.json (settings, radio status, capture statistics). Printed: DC offset, RMS and peak in
dBFS, clipped samples, overflow (samples lost on the board).
"""
import argparse
import json
import os
import sys
import time
from dataclasses import asdict

import numpy as np

from fmc_protocol import BOARD_FMCOMMS3, GAIN_MODES, FmcBoard, RadioConfig

FULL_SCALE = 2048.0                     # 12-bit converter


def stats(iq: np.ndarray) -> dict:
    x = iq.astype(np.float64)
    dc = x.mean(axis=0)
    ac = x - dc
    p = (ac ** 2).sum(axis=1).mean()
    peak = np.abs(x).max()
    clipped = int((np.abs(iq) >= 2047).any(axis=1).sum())
    return {
        "dc_i": float(dc[0]), "dc_q": float(dc[1]),
        "rms_dbfs": float(10 * np.log10(p / FULL_SCALE ** 2 + 1e-30)),
        "peak_dbfs": float(20 * np.log10(peak / FULL_SCALE + 1e-30)),
        "clipped": clipped,
        "iq_power_ratio_db": float(10 * np.log10((ac[:, 0] ** 2).mean() / max((ac[:, 1] ** 2).mean(), 1e-30))),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="10.42.0.101")
    ap.add_argument("--status", action="store_true", help="only PING and the radio status")
    ap.add_argument("--lo", type=float, default=2420e6, help="Rx LO in Hz (default 2420e6)")
    ap.add_argument("--rate", type=float, default=40e6, help="sample rate in Hz (default 40e6)")
    ap.add_argument("--bw", type=float, default=18e6, help="RF bandwidth in Hz (default 18e6)")
    ap.add_argument("--gain-mode", choices=GAIN_MODES, default="manual")
    ap.add_argument("--gain", type=int, default=30, help="manual gain in dB (default 30)")
    ap.add_argument("--reinit", action="store_true", help="set the AD9361 up again from reset")
    ap.add_argument("--seconds", type=float, default=0.5)
    ap.add_argument("-n", type=int, help="samples (instead of --seconds)")
    ap.add_argument("-o", "--out", default="fmc_capture.cs16")
    a = ap.parse_args()

    with FmcBoard(a.host) as b:
        info = b.ping()
        print(f"Board: protocol {info.version}, build {info.build}, flags 0x{info.flags:X}, "
              f"buffer {info.max_samples} samples")
        if not info.flags & BOARD_FMCOMMS3:
            sys.exit("This board does not report BOARD_FMCOMMS3 - is fmc_server.elf running?")
        if a.status:
            print(b.radio_status().describe())
            return

        cfg = RadioConfig(lo_hz=int(a.lo), rate_hz=int(a.rate), bw_hz=int(a.bw),
                          gain_mode=GAIN_MODES[a.gain_mode], gain_db=a.gain,
                          flags=1 if a.reinit else 0)
        st = b.radio_config(cfg)
        print(st.describe())
        if not st.ok:
            sys.exit("AD9361 not ready")

        n = a.n if a.n else int(a.seconds * a.rate)
        n = min(n, info.max_samples)
        cap = b.capture(n)
        print(f"Captured {cap.n_samples} samples in {cap.time_us / 1000:.1f} ms "
              f"({cap.n_samples / max(cap.time_us, 1):.2f} Msps)"
              + ("  *** OVERFLOW: samples were lost ***" if cap.overflow else ""))
        t0 = time.time()
        iq = b.read_samples(cap.n_samples)
        dt = time.time() - t0
        print(f"Read {len(iq)} samples in {dt:.1f} s ({iq.nbytes / dt / 1e6:.1f} MB/s)")

    s = stats(iq)
    print(f"DC I {s['dc_i']:.1f} Q {s['dc_q']:.1f} LSB, RMS {s['rms_dbfs']:.1f} dBFS, "
          f"peak {s['peak_dbfs']:.1f} dBFS, clipped {s['clipped']} samples, "
          f"I/Q power ratio {s['iq_power_ratio_db']:.2f} dB")

    out = a.out if a.out.endswith(".cs16") else a.out + ".cs16"
    iq.astype("<i2").tofile(out)
    meta = {"file": os.path.basename(out), "format": "cs16 (interleaved int16 I, Q, little-endian)",
            "sample_rate": cap.radio.rate_hz, "lo_hz": cap.radio.lo_hz, "rf_bw_hz": cap.radio.bw_hz,
            "gain_mode": a.gain_mode, "gain_db": cap.radio.gain_db, "n_samples": int(len(iq)),
            "overflow": cap.overflow, "capture_time_us": cap.time_us, "stats": s,
            "radio_status": asdict(cap.radio), "board_build": info.build,
            "time": time.strftime("%Y-%m-%d %H:%M:%S")}
    with open(out[:-5] + ".json", "w") as f:
        json.dump(meta, f, indent=2)
    print(f"Saved {out} and {out[:-5]}.json")


if __name__ == "__main__":
    main()
