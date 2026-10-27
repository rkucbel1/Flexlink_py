#!/usr/bin/env python3
"""
pd_load_diag.py  (Flexlink_py/HW_Development/Packet_Detector/PC)

Diagnoses a LOOPBACK FAIL (samples read back from the board's DDR differ from those sent):
where the bad samples are, which values they hold, whether two read-backs agree (memory
really overwritten, or damaged on the way back), whether it depends on load speed or size,
and whether it grows while the board sits idle.

    python pd_load_diag.py                                 # rep3_40.bin, board 192.168.50.10
    python pd_load_diag.py --vector ../TestVectors/rep3.bin
"""
from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pd_protocol as pr  # noqa: E402
import pd_hw_test as hw  # noqa: E402


def ranges(idx: np.ndarray) -> list[tuple[int, int]]:
    """Consecutive runs of indices as (first, last)."""
    if idx.size == 0:
        return []
    cut = np.flatnonzero(np.diff(idx) != 1)
    starts = np.concatenate([[idx[0]], idx[cut + 1]])
    ends = np.concatenate([idx[cut], [idx[-1]]])
    return list(zip(starts.tolist(), ends.tolist()))


def report(tag: str, sent: np.ndarray, back: np.ndarray) -> np.ndarray:
    bad = np.flatnonzero(np.any(back != sent, axis=1))
    if bad.size == 0:
        print(f"  {tag}: OK ({sent.shape[0]} samples)")
        return bad
    rs = ranges(bad)
    words = back[bad].astype(np.int64)
    u32 = (words[:, 0] & 0xFFFF) | ((words[:, 1] & 0xFFFF) << 16)
    vals, cnt = np.unique(u32, return_counts=True)
    top = ", ".join(f"0x{v:08x} x{c}" for v, c in sorted(zip(vals, cnt), key=lambda t: -t[1])[:4])
    print(f"  {tag}: {bad.size} bad samples in {len(rs)} range(s): "
          + ", ".join(f"{a}..{b} (byte 0x{4 * a:06x}, {b - a + 1})" for a, b in rs[:6])
          + (" ..." if len(rs) > 6 else ""))
    print(f"      values read: {top}")
    return bad


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Diagnose sample-buffer read-back errors")
    ap.add_argument("--host", default=hw.DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=pr.DEFAULT_PORT)
    ap.add_argument("--vector", type=Path, default=HERE.parent / "TestVectors" / "rep3_40.bin")
    a = ap.parse_args(argv)
    iq = np.fromfile(a.vector, "<i2").reshape(-1, 2)
    b = hw.Board(a.host, a.port, 60)
    info = b.ping()
    print(f"board build {info.build}; vector {a.vector.name}: {iq.shape[0]} samples")

    print("1. RESET, load at full speed, read back twice")
    b.call(pr.RESET)
    t0 = time.perf_counter()
    b.load(iq)
    print(f"  loaded {iq.nbytes / 1e6:.2f} MB in {time.perf_counter() - t0:.2f} s")
    bad1 = report("read 1", iq, b.read(0, iq.shape[0]))
    bad2 = report("read 2", iq, b.read(0, iq.shape[0]))
    if bad1.size or bad2.size:
        print("  -> the two reads " + ("AGREE: the DDR contents are wrong (overwritten or not written)"
                                       if np.array_equal(bad1, bad2) else
                                       "DIFFER: the data is damaged on the way back (read path)"))

    print("2. wait 5 s with the board idle, read again (does anything keep writing?)")
    time.sleep(5)
    bad3 = report("read 3", iq, b.read(0, iq.shape[0]))
    if bad3.size != bad2.size or not np.array_equal(bad3, bad2):
        print("  -> CHANGED while idle: something on the board writes into the sample buffer by itself")

    print("3. reload slowly (10 ms between 64k-sample messages)")
    b.call(pr.RESET)
    for i in range(0, iq.shape[0], pr.MAX_LOAD_SAMPLES):
        blk = iq[i:i + pr.MAX_LOAD_SAMPLES]
        b.call(pr.LOAD_SAMPLES, struct.pack("<I", i) + pr.iq_to_bytes(blk))
        time.sleep(0.01)
    report("slow load", iq, b.read(0, iq.shape[0]))

    print("4. small loads: first 10000 samples, then samples 0..999 only")
    for n in (10000, 1000):
        b.call(pr.RESET)
        b.load(iq[:n])
        report(f"{n} samples", iq[:n], b.read(0, n))

    print("5. a pattern instead of the vector: sample k = (k & 0x7FF, k >> 11)")
    k = np.arange(iq.shape[0])
    pat = np.stack([k & 0x7FF, (k >> 11) & 0x7FF], axis=1).astype(np.int16)
    b.call(pr.RESET)
    b.load(pat)
    back = b.read(0, pat.shape[0])
    bad = report("pattern", pat, back)
    if bad.size:
        src = back[bad].astype(np.int64)
        where = (src[:, 0] & 0x7FF) | ((src[:, 1] & 0x7FF) << 11)
        print(f"      if the bad words are pattern values, they came from sample(s) {np.unique(where)[:8]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
