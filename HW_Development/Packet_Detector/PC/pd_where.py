#!/usr/bin/env python3
"""
pd_where.py  (Flexlink_py/HW_Development/Packet_Detector/PC)

Diagnostic for firmware build 8: for every sample that arrives wrong on the board, finds the
memory address it passed through on the way in (the lwIP receive buffer, pbuf), from the
board's log of where each received pbuf sat. If the bad words always sit at the same few
addresses, something on the board keeps writing there, and the linker map tells us what.

    python pd_where.py                 # 3 loads of the 600116-sample pattern
"""
from __future__ import annotations

import argparse
import collections
import struct
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pd_protocol as pr  # noqa: E402
import pd_hw_test as hw  # noqa: E402
from pd_bisect import pattern  # noqa: E402

PATTERN_CHECK, SEG_LOG = 12, 13


def one_load(b, pat):
    b.call(pr.RESET)
    b.load(pat)
    r = b.call(PATTERN_CHECK)
    n_bad, n_list = struct.unpack_from("<II", r)
    bad = np.frombuffer(r[8:8 + 8 * n_list], "<u4").reshape(-1, 2)
    r = b.call(SEG_LOG)
    nl, ns = struct.unpack_from("<II", r)
    loads = np.frombuffer(r[8:8 + 12 * nl], "<u4").reshape(-1, 3).astype(np.int64)
    segs = np.frombuffer(r[8 + 12 * nl:8 + 12 * (nl + ns)], "<u4").reshape(-1, 3).astype(np.int64)
    return n_bad, bad, loads, segs


def locate(k, loads, segs):
    """Memory address sample k passed through, its offset in the pbuf payload, the payload length."""
    for off, cnt, pos0 in loads:
        if off <= k < off + cnt:
            pos = pos0 + 4 * (k - off)
            i = int(np.searchsorted(segs[:, 0], pos, side="right")) - 1
            if i >= 0 and segs[i, 0] <= pos < segs[i, 0] + segs[i, 2]:
                return int(segs[i, 1] + pos - segs[i, 0]), int(pos - segs[i, 0]), int(segs[i, 2]), int(segs[i, 1])
            return None
    return None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Where do the bad samples pass through? (firmware build 8)")
    ap.add_argument("--host", default=hw.DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=pr.DEFAULT_PORT)
    ap.add_argument("--samples", type=int, default=600116)
    ap.add_argument("--loads", type=int, default=3)
    a = ap.parse_args(argv)
    b = hw.Board(a.host, a.port, 60)
    info = b.ping()
    if info.build < 8:
        print(f"board firmware build {info.build}: this needs build 8")
        return 2
    pat = pattern(a.samples)
    addr_cnt = collections.Counter()
    in_off = collections.Counter()
    seg_len = collections.Counter()
    vals_at = collections.defaultdict(set)
    all_payload = set()
    lo, hi = None, None
    print(f"board build {info.build}: {a.loads} loads of {a.samples} pattern samples")
    for n in range(a.loads):
        n_bad, bad, loads, segs = one_load(b, pat)
        all_payload.update(segs[:, 1].tolist())
        lo = int(segs[:, 1].min()) if lo is None else min(lo, int(segs[:, 1].min()))
        hi = int((segs[:, 1] + segs[:, 2]).max()) if hi is None else max(hi, int((segs[:, 1] + segs[:, 2]).max()))
        lens = collections.Counter(segs[:, 2].tolist())
        print(f"  load {n + 1}: {n_bad} bad samples, {len(segs)} pbufs received "
              f"(lengths {', '.join(f'{l} x{c}' for l, c in lens.most_common(3))})")
        for k, v in bad:
            loc = locate(int(k), loads, segs)
            if loc is None:
                continue
            addr, o, ln, base = loc
            addr_cnt[addr] += 1
            in_off[o] += 1
            seg_len[ln] += 1
            vals_at[addr].add(int(v))
    print(f"\nreceive pbuf payloads: {len(all_payload)} distinct buffers, 0x{lo:08x} .. 0x{hi:08x}")
    if not addr_cnt:
        print("no bad samples located")
        return 0
    rep = sum(1 for c in addr_cnt.values() if c > 1)
    print(f"bad words at {len(addr_cnt)} distinct addresses, {rep} of them bad in more than one load")
    print("most frequent addresses (address: times bad, values):")
    for addr, c in addr_cnt.most_common(16):
        print(f"  0x{addr:08x}: {c}  {', '.join(f'0x{v:x}' for v in sorted(vals_at[addr]))}")
    print("offset of the bad word inside its pbuf payload (offset: count):")
    print("  " + ", ".join(f"{o}: {c}" for o, c in in_off.most_common(12)))
    print("length of the pbufs that held bad words (length: count):")
    print("  " + ", ".join(f"{l}: {c}" for l, c in seg_len.most_common(6)))
    addrs = np.array(sorted(addr_cnt))
    print(f"bad addresses span 0x{addrs.min():08x} .. 0x{addrs.max():08x}; address mod 64: "
          + ", ".join(f"{m}: {c}" for m, c in collections.Counter((addrs % 64).tolist()).most_common(6)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
