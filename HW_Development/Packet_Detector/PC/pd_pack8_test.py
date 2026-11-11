#!/usr/bin/env python3
"""pd_pack8_test.py -- test the PL int8 unpacker (bitstream 9.1, firmware build 24) with a
CoreBoard capture, and see what the FPGA receiver makes of real CoreBoard samples.

The same int8 I/Q capture (App/CoreBoard/*.cs8, 40 Msps) goes through the receiver twice,
input stage 2 (half-band in the PL):

  A  as today: the PC converts to 12-bit samples (int8 x 16), one sample per DMA word
  B  packed:   the raw int8 bytes, two samples per word, RUN with PACK8 -> pd_unpack8.v
               unpacks them in the PL

pd_unpack8 produces exactly the samples of A, so every record of the two runs (events,
timing, control field, payload, channel estimate, decoder input, transport blocks) must be
identical. Then the transport blocks of the run are summarised per packet: this is also the
first time the FPGA receiver sees real CoreBoard samples (for the PC decoder's results on the
same capture, run App/FlexlinkProcessCoreBoardCapturedWaveform.py --capture <file>).

--dc-shift 10,8,7,6 sweeps the DC blocker (alpha = 1 - 2^-dc_shift at 20 Msps; corner about
20 MHz / (2 pi 2^dc_shift): 10 -> 3 kHz, 8 -> 12 kHz, 7 -> 25 kHz, 6 -> 50 kHz) with the packed
input, and shows per value how many packets decode: the CoreBoard's DC wander (0..40 kHz, in a
noise floor of ~1 LSB) makes the detector trigger in the noise before a packet, which moves the
Preamble B search window too early; a faster DC blocker removes the wander, but also attenuates
the innermost subcarriers (+-19.5 kHz).

Usage (from HW_Development/Packet_Detector/PC):
    python pd_pack8_test.py --host 10.42.0.101
    python pd_pack8_test.py --host 10.42.0.101 --file ../../../App/CoreBoard/EmuSource.cs8 --seconds 0.2
    python pd_pack8_test.py --host 10.42.0.101 --dc-shift 10,9,8,7,6,5

Exit status 0 = identical, 1 = different, 2 = communication or setup error.
"""
from __future__ import annotations

import argparse
import json
import struct
import sys
import time
from collections import OrderedDict
from dataclasses import fields
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pd_protocol as pr  # noqa: E402
from pd_hw_test import Board  # noqa: E402

ROOT = HERE.parents[2]
DEFAULT_FILE = ROOT / "App" / "CoreBoard" / "Segment_newfw_3820ms.cs8"
DEFAULT_PARAMS = HERE.parent / "TestVectors" / "rep3.json"   # the regression vectors' parameters


def run_once(b: Board, params: pr.Params, load: np.ndarray, count: int, flags: int):
    b.call(pr.RESET)
    b.call(pr.SET_PARAMS, params.pack())
    t0 = time.perf_counter()
    b.load(load)
    t1 = time.perf_counter()
    reply = b.call(pr.RUN, struct.pack("<II", count, flags))
    t2 = time.perf_counter()
    return reply, t1 - t0, t2 - t1


def summary(reply: bytes) -> str:
    evs, done, tms, cfs, pls, chs, lqs, tbs = pr.parse_run_reply_tb(reply)
    good_tm = sum(1 for t in tms if t.status == pr.TM_GOOD)
    cf_ok = sum(1 for c in cfs if c.crc_ok)
    return (f"{len(evs)} triggers, {good_tm} with Preamble B timing (of {len(tms)} searches), "
            f"{cf_ok} control fields CRC ok (of {len(cfs)}), {len(tbs)} transport blocks")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="PL int8 unpacker test with a CoreBoard capture.",
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--host", default="192.168.50.10")
    ap.add_argument("--port", type=int, default=pr.DEFAULT_PORT)
    ap.add_argument("--file", type=Path, default=DEFAULT_FILE, help="int8 I/Q capture at 40 Msps (.cs8)")
    ap.add_argument("--start-seconds", type=float, default=0.0)
    ap.add_argument("--seconds", type=float, default=0.0, help="length to use (0 = whole file, max ~0.8 s)")
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--dc-shift", help="sweep: comma-separated dc_shift values (packed input only)")
    ap.add_argument("--chunk-seconds", type=float, default=0.15,
                    help="sweep: RUN in pieces of this length (a RUN reply holds ~900 transport blocks, "
                         "~33 packets); a packet cut by a piece boundary is lost for every value alike")
    ap.add_argument("--params-from", type=Path, default=DEFAULT_PARAMS,
                    help="test-vector .json whose detector parameters to use (default TestVectors/rep3.json; "
                         "pd_protocol.Params defaults if it does not exist)")
    a = ap.parse_args(argv)

    if not a.file.is_file():
        print(f"Error: {a.file} not found", file=sys.stderr)
        return 2
    raw = np.memmap(a.file, dtype=np.int8, mode="r")
    first = 2 * int(a.start_seconds * 40e6)
    n = raw.size // 2 - first // 2
    if a.seconds:
        n = min(n, int(a.seconds * 40e6))
    n -= n % 2                                          # whole words of two samples
    iq8 = np.ascontiguousarray(raw[first:first + 2 * n]).reshape(n, 2)
    iq12 = iq8.astype(np.int16) * 16                    # A: what pd_unpack8 produces
    packed = np.frombuffer(iq8.tobytes(), dtype="<i2").reshape(-1, 2)   # B: the bytes, as LOAD words
    print(f"{a.file.name}: {n} samples ({n / 40e6 * 1e3:.1f} ms at 40 Msps), "
          f"peak {int(np.abs(iq8.astype(np.int16)).max())} of 127")

    try:
        b = Board(a.host, a.port, a.timeout)
        info = b.ping()
        print(f"Zedboard at {a.host}: firmware build {info.build}")
        if info.build < 24 or not info.flags & pr.BOARD_PACK8:
            print("Error: needs firmware build 24 and bitstream 9.1 (pd_unpack8); PING flags "
                  f"0x{info.flags:x}", file=sys.stderr)
            return 2
        words = n // 2 if a.dc_shift else n             # packed: 2 samples per word; A: 1 per word
        if words > info.max_samples:
            print(f"Error: {n} samples need {words} buffer words, the board has {info.max_samples}; "
                  f"use --seconds", file=sys.stderr)
            return 2
        params = pr.Params()
        if a.params_from and a.params_from.is_file():
            meta = json.loads(a.params_from.read_text())
            known = {f.name for f in fields(pr.Params)}
            params = pr.Params(**{k: v for k, v in meta.get("params", {}).items() if k in known})
            print(f"Detector parameters from {a.params_from.name}")
        params.input_stage = 2
        if a.dc_shift:
            print(f"\nDC blocker sweep (packed input; the segment has {n / 40e6 / 5.203e-3:.0f} packet periods):")
            print("  dc_shift  corner    triggers  timing ok  control ok  packets  TB CRC ok")
            step = max(2, int(a.chunk_seconds * 40e6) // 2 * 2)
            if n > step:
                print(f"  (in {-(-n // step)} RUNs of {step / 40e6 * 1e3:.0f} ms)")
            for v in [int(x) for x in a.dc_shift.split(",")]:
                params.dc_shift = v
                evs, tms, cfs, tbs, pk = [], [], [], [], set()
                for k0 in range(0, n, step):
                    blk = packed[k0 // 2:(k0 + step) // 2]
                    rep, _, _ = run_once(b, params, blk, 2 * blk.shape[0], pr.RUN_FLAG_PACK8)
                    e, _, t, c, _, _, _, tb = pr.parse_run_reply_tb(rep)
                    evs += e
                    tms += t
                    cfs += c
                    tbs += tb
                    pk |= {(k0, x.t_trig) for x in tb}
                corner = f"{20e6 / (2 * 3.14159 * 2 ** v) / 1e3:5.1f} kHz" if v else "  off   "
                print(f"  {v:8d}  {corner}  {len(evs):8d}  {sum(t.status == pr.TM_GOOD for t in tms):5d}/{len(tms):<3d}"
                      f"  {sum(c.crc_ok for c in cfs):6d}/{len(cfs):<3d}  {len(pk):7d}  "
                      f"{sum(t.crc_ok for t in tbs)}/{len(tbs)}")
            return 0
        rep_a, la, ra = run_once(b, params, iq12, n, 0)
        print(f"A  12-bit samples:   load {la:.1f} s, run {ra:.2f} s: {summary(rep_a)}")
        rep_b, lb, rb = run_once(b, params, packed, n, pr.RUN_FLAG_PACK8)
        print(f"B  packed int8:      load {lb:.1f} s, run {rb:.2f} s: {summary(rep_b)}")
    except (OSError, pr.ProtocolError) as e:
        print(f"Error: {e}", file=sys.stderr)
        return 2

    same = rep_a == rep_b
    if same:
        print(f"IDENTICAL: all {len(rep_a)} bytes of records match - the PL unpacker gives exactly the "
              f"12-bit samples")
    else:
        pa, pb = pr.parse_run_reply_tb(rep_a), pr.parse_run_reply_tb(rep_b)
        names = ("events", "samples", "timing", "control", "payload", "channel estimate", "decoder input", "TB")
        for name, x, y in zip(names, pa, pb):
            if x != y:
                print(f"DIFFERENT: {name}: A {len(x) if isinstance(x, list) else x}, "
                      f"B {len(y) if isinstance(y, list) else y}")

    # transport blocks per packet (run A)
    tbs = pr.parse_run_reply_tb(rep_a)[7]
    pk: "OrderedDict[int, list]" = OrderedDict()
    for t in tbs:
        pk.setdefault(t.t_trig, []).append(t)
    total = sum(len(v) for v in pk.values())
    ok = sum(t.crc_ok for v in pk.values() for t in v)
    print(f"\nFPGA receiver on the CoreBoard capture: {len(pk)} packet(s) decoded, transport blocks "
          f"{ok}/{total} CRC ok" + (f" (BLER {1 - ok / total:.2e})" if total else ""))
    for t_trig, v in pk.items():
        n_ok = sum(t.crc_ok for t in v)
        iters = max(t.iters for t in v)
        print(f"  t_trig {t_trig:9d} ({t_trig / 20e6 * 1e3:8.3f} ms): {n_ok}/{len(v)} CRC ok, "
              f"LDPC iterations max {iters}")
    return 0 if same else 1


if __name__ == "__main__":
    sys.exit(main())
