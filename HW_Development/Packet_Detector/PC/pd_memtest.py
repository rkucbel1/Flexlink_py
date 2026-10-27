#!/usr/bin/env python3
"""
pd_memtest.py  (Flexlink_py/HW_Development/Packet_Detector/PC)

Board-local DDR test (firmware build 5, MEMTEST command): the board fills a region with a
pattern (each word = its own address, or its inverse), flushes it to DDR, waits, reads it back
from DDR and reports every word that changed. No Ethernet traffic is involved while the data
sits in memory, so this separates "something writes into DDR" from network problems.

    python pd_memtest.py                      # the standard set of regions and waits
    python pd_memtest.py --addr 0x10000000 --bytes 0x1000000 --wait 5000
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

MEMTEST = 8


def run(b, addr: int, nbytes: int, wait_ms: int, hold_reset: bool, inverse: bool) -> None:
    flags = (1 if hold_reset else 0) | (2 if inverse else 0)
    reply = b.call(MEMTEST, struct.pack("<4I", addr, nbytes, wait_ms, flags))
    n_bad, n_list = struct.unpack_from("<II", reply)
    lst = np.frombuffer(reply[8:8 + 8 * n_list], "<u4").reshape(-1, 2)
    tag = (f"0x{addr:08x} + {nbytes / 2**20:5.1f} MB, wait {wait_ms / 1000:4.1f} s, "
           f"{'detector in reset' if hold_reset else 'detector running '}, "
           f"{'~addr' if inverse else ' addr'}")
    if n_bad == 0:
        print(f"  {tag}: OK")
        return
    vals, cnt = np.unique(lst[:, 1], return_counts=True)
    top = ", ".join(f"0x{v:08x} x{c}" for v, c in sorted(zip(vals, cnt), key=lambda t: -t[1])[:6])
    off = lst[:, 0] & 7
    print(f"  {tag}: {n_bad} BAD words ({n_bad / (nbytes / 4) * 100:.3f}%)")
    print(f"      first: " + ", ".join(f"0x{a:08x}=0x{v:08x}" for a, v in lst[:6]))
    print(f"      values: {top}; address mod 8: {dict(zip(*np.unique(off, return_counts=True)))}")
    alias = [(a, v) for a, v in lst if 0x00100000 <= (v if not inverse else ~v & 0xFFFFFFFF) < 0x20000000
             and (v if not inverse else ~v & 0xFFFFFFFF) != a]
    if alias:
        a, v = alias[0]
        print(f"      {len(alias)} listed word(s) hold another address's pattern, e.g. 0x{a:08x} holds 0x{v:08x}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Board-local DDR test")
    ap.add_argument("--host", default=hw.DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=pr.DEFAULT_PORT)
    ap.add_argument("--addr", type=lambda v: int(v, 0))
    ap.add_argument("--bytes", type=lambda v: int(v, 0), default=16 << 20)
    ap.add_argument("--wait", type=int, default=2000, help="ms")
    a = ap.parse_args(argv)
    b = hw.Board(a.host, a.port, 120)
    info = b.ping()
    if info.build < 5:
        print(f"board firmware build {info.build}: MEMTEST needs build 5 (FW/build_firmware.py)")
        return 2
    print(f"board build {info.build}")
    if a.addr is not None:
        run(b, a.addr, a.bytes, a.wait, False, False)
        return 0
    for addr in (0x08000000, 0x10000000, 0x18000000):
        run(b, addr, 16 << 20, 0, False, False)
    run(b, 0x10000000, 16 << 20, 5000, False, False)
    run(b, 0x10000000, 16 << 20, 5000, True, False)
    run(b, 0x10000000, 16 << 20, 5000, False, True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
