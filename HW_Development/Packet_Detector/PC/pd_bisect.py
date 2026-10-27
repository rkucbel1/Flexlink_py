#!/usr/bin/env python3
"""
pd_bisect.py  (Flexlink_py/HW_Development/Packet_Detector/PC)

Diagnostic for firmware build 7: finds which step makes the sample load / read-back go wrong
(builds 4-6 with the half-band bitstream: about 0.13 % of samples replaced by 0x404, 0x408,
0x624, 0x628; build 3 on the same bitstream, which never touches the detector, is clean).

Build 7 leaves the detector in its power-on reset until the first SET_PARAMS or RUN, as build 3
does. This script then adds one thing at a time and checks the transfer after each:

  A  nothing touched (detector in power-on reset)            -> should be clean, like build 3
  B  the double-precision code (tone_thresh_lin) run 1000x   -> VFP / FPU only, no PL access
  C  detector registers written by REG_WRITE, still in reset -> AXI-Lite writes only
  D  detector released (CTRL = 0)                            -> the PL detector running idle
  E  SET_PARAMS (the firmware's own detector_reset)          -> what builds 4-6 do at boot

Each check loads a pattern (sample k = (k & 0x7FF, k >> 11)), lets the board compare its
buffer with the pattern itself (PATTERN_CHECK: errors that came in on the receive path) and
then reads the buffer back twice (errors only in the read-back: send path).

Power-cycle (or re-run run_board.tcl) before running it, so the detector starts untouched:
    python pd_bisect.py
"""
from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pd_protocol as pr  # noqa: E402
import pd_hw_test as hw  # noqa: E402

REG_READ, REG_WRITE, FPU_TEST, PATTERN_CHECK = 9, 10, 11, 12
PD = 0x43C00000
R_CTRL, R_STATUS, R_PARAM0 = 0x04, 0x08, 0x20


def rd(b, a: int) -> int:
    return struct.unpack("<I", b.call(REG_READ, struct.pack("<I", a)))[0]


def wr(b, a: int, v: int) -> None:
    b.call(REG_WRITE, struct.pack("<II", a, v))


def pattern(n: int) -> np.ndarray:
    k = np.arange(n)
    return np.stack([k & 0x7FF, (k >> 11) & 0x7FF], axis=1).astype(np.int16)


def check(b, pat: np.ndarray, tag: str) -> tuple[int, int]:
    b.load(pat)
    r = b.call(PATTERN_CHECK)
    n_bad, n_list = struct.unpack_from("<II", r)
    lst = np.frombuffer(r[8:8 + 8 * n_list], "<u4").reshape(-1, 2)
    rx_idx = set(lst[:, 0].tolist())
    tx_only = []
    for _ in range(2):
        back = b.read(0, pat.shape[0])
        bad = np.flatnonzero(np.any(back != pat, axis=1))
        tx_only.append(len(set(bad.tolist()) - rx_idx) if n_bad <= 1000 else -1)
    vals = ", ".join(f"0x{v:x}" for v in sorted(set(lst[:, 1].tolist()))[:6])
    tx_txt = "/".join("?" if t < 0 else str(t) for t in tx_only)
    verdict = "clean" if n_bad == 0 and max(tx_only) == 0 else "CORRUPT"
    print(f"  {tag:<44} {verdict:8} received wrong (board check): {n_bad:5}   "
          f"wrong only in read-back 1/2: {tx_txt}" + (f"   values {vals}" if n_bad else ""))
    return n_bad, max(tx_only)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Bisect the sample-transfer corruption (firmware build 7)")
    ap.add_argument("--host", default=hw.DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=pr.DEFAULT_PORT)
    ap.add_argument("--samples", type=int, default=600116)
    a = ap.parse_args(argv)

    b = hw.Board(a.host, a.port, 60)
    info = b.ping()
    if info.build < 7:
        print(f"board firmware build {info.build}: this needs build 7")
        return 2
    ctrl = rd(b, PD + R_CTRL)
    print(f"board build {info.build}; detector CTRL 0x{ctrl:x} "
          + ("(power-on reset: good)" if ctrl == 1 else
             "- NOT the power-on state: power-cycle / re-run run_board.tcl for a clean bisect"))
    pat = pattern(a.samples)
    res = {}
    print("\nstep  what was added before the check")
    b.call(pr.RESET)                              # build 7: does not touch the detector yet
    res["A"] = check(b, pat, "A  nothing (detector untouched)")

    lin, db = struct.unpack("<Ii", b.call(FPU_TEST, struct.pack("<II", 2560, 1000)))
    res["B"] = check(b, pat, f"B  FPU code x1000 (lin {lin}, {db / 256:.1f} dB)")

    p = struct.unpack("<9I", pr.Params().pack())
    wr(b, PD + R_CTRL, 1)
    regs = [p[0], p[1], p[2], p[3], p[4], p[6], p[7], p[8], lin]
    for i, v in enumerate(regs):
        wr(b, PD + R_PARAM0 + 4 * i, v)
    res["C"] = check(b, pat, "C  registers written, held in reset")

    wr(b, PD + R_CTRL, 0)
    res["D"] = check(b, pat, f"D  released (STATUS 0x{rd(b, PD + R_STATUS):x})")

    b.call(pr.SET_PARAMS, pr.Params().pack())
    res["E"] = check(b, pat, "E  SET_PARAMS (firmware detector_reset)")

    first = next((k for k, (rx, tx) in res.items() if rx or tx), None)
    print()
    if first is None:
        print(f"No step corrupts the transfer: build {info.build} is clean.")
    else:
        where = ("receive path / DDR" if res[first][0] else "") + \
                (" and " if res[first][0] and res[first][1] else "") + ("send path" if res[first][1] else "")
        print(f"First corrupt step: {first} ({where}). Send me this output.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
