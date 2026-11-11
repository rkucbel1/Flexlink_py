#!/usr/bin/env python3
"""pd_replay_test.py -- reproduce the streaming receiver's hang with RUN (firmware build >= 24).

In streaming (build 25/26) the FPGA receiver stopped for good in the 14th packet: STATUS BUSY,
input held, every FIFO empty, sample counter frozen. Streaming differs from RUN in two ways,
and this script feeds the same capture through RUN to test each:

  --pieces N   feed it in RUNs of N datagrams (672 samples each) without a reset in between, as
               the streaming DMA does (64 per transfer); the receiver keeps its state from RUN to
               RUN, so this adds only the pauses. 0 = one RUN (default)
  --zero K:N   zero N datagrams from datagram K (counted from --start-seconds), as streaming
               does for lost datagrams; several, comma separated
  --sweep-pause STEP
               one pause only: the capture in two RUNs, split at datagram K, for K every STEP
               datagrams across --sweep-span; shows in which part of a packet a pause hangs it
  --sweep-zero N
               repeat the run with N zeroed datagrams at every --sweep-step datagrams across
               the first --sweep-span datagrams, and report where the receiver hangs

A hang shows as a RUN error (the DMA does not finish: "DMA timeout", or "detector still
busy"); the script then reads the receiver's STATUS and sample count and stops.

Usage (from HW_Development/Packet_Detector/PC):
    python pd_replay_test.py --host 10.42.0.101                         # one RUN, 150 ms from 2.6 s
    python pd_replay_test.py --host 10.42.0.101 --pieces 64             # streaming's pauses
    python pd_replay_test.py --host 10.42.0.101 --zero 6800:8            # 8 zeroed datagrams
    python pd_replay_test.py --host 10.42.0.101 --sweep-zero 8 --sweep-step 50 --sweep-span 8000
    python pd_replay_test.py --host 10.42.0.101 --start-seconds 2.645 --seconds 0.03 --sweep-pause 10 --sweep-span 700
"""
from __future__ import annotations

import argparse
import json
import struct
import sys
import time
from dataclasses import fields
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pd_protocol as pr  # noqa: E402
from pd_hw_test import Board  # noqa: E402

ROOT = HERE.parents[2]
DEFAULT_FILE = ROOT / "App" / "CoreBoard" / "EmuSource.cs8"
DEFAULT_PARAMS = HERE.parent / "TestVectors" / "rep3.json"
SPP = 672
PD_BASE = 0x43C00000
R_STATUS, R_SAMPLES = 0x08, 0x0C
TB_MAX_PER_RUN = 800                    # ~30 packets: a RUN reply holds ~900 transport blocks


def reg(b: Board, off: int) -> int:
    return struct.unpack("<I", b.call(pr.REG_READ if hasattr(pr, "REG_READ") else 9, struct.pack("<I", PD_BASE + off)))[0]


def replay(b: Board, params: pr.Params, iq8: np.ndarray, pieces: int, zeros: list[tuple[int, int]],
           host: str = "", port: int = 0, split: int = 0):
    """Returns (ok, message, packets {t_trig: [n, crc_ok]}, triggers, timings)."""
    x = iq8.copy()
    for k, n in zeros:
        x[k * SPP:(k + n) * SPP] = 0
    b.call(pr.RESET)
    b.call(pr.SET_PARAMS, params.pack())
    total = x.shape[0]
    step = pieces * SPP if pieces else total
    if not pieces:                                      # RUN-reply limit: ~30 packets per RUN
        step = min(total, int(0.15 * 40e6) // SPP * SPP)
    packets: dict[int, list] = {}
    trig = tims = 0
    bounds = list(range(0, total, step))
    if split:                                           # one pause, at datagram `split`
        bounds = [0, split * SPP]
    for bi, k0 in enumerate(bounds):
        end = bounds[bi + 1] if bi + 1 < len(bounds) else (total if split else k0 + step)
        blk = x[k0:end]
        blk = blk[:blk.shape[0] // 2 * 2]
        words = np.frombuffer(blk.tobytes(), dtype="<i2").reshape(-1, 2)
        b.load(words)
        try:
            rep = b.call(pr.RUN, struct.pack("<II", blk.shape[0], pr.RUN_FLAG_PACK8))
        except (pr.ProtocolError, OSError) as e:
            try:
                b.sock.close()
            except (OSError, AttributeError):
                pass
            time.sleep(6)                                # the board's 5 s DMA limit (build 27)
            b2 = Board(host, port, 30.0)
            st, ns = reg(b2, R_STATUS), reg(b2, R_SAMPLES)
            return False, (f"RUN failed at sample {k0} (datagram {k0 // SPP}, {k0 / 40e3:.2f} ms): {e}; "
                           f"receiver STATUS 0x{st:08x}, samples {ns}"), packets, trig, tims
        evs, _, tms, cfs, _, _, _, tbs = pr.parse_run_reply_tb(rep)
        trig += len(evs)
        tims += sum(t.status == pr.TM_GOOD for t in tms)
        for t in tbs:
            p = packets.setdefault(t.t_trig, [0, 0])
            p[0] += 1
            p[1] += t.crc_ok
    return True, "", packets, trig, tims


def summary(packets) -> str:
    tot = sum(p[0] for p in packets.values())
    ok = sum(p[1] for p in packets.values())
    return f"{len(packets)} packets, transport blocks {ok}/{tot} CRC ok"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Reproduce the streaming receiver's hang with RUN.",
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--host", default="192.168.50.10")
    ap.add_argument("--port", type=int, default=pr.DEFAULT_PORT)
    ap.add_argument("--file", type=Path, default=DEFAULT_FILE)
    ap.add_argument("--start-seconds", type=float, default=2.6)
    ap.add_argument("--seconds", type=float, default=0.15)
    ap.add_argument("--pieces", type=int, default=0, help="datagrams per RUN (0 = one RUN)")
    ap.add_argument("--zero", default="", help="K:N[,K:N...] zero N datagrams from datagram K")
    ap.add_argument("--sweep-zero", type=int, default=0, help="zero this many datagrams at each sweep position")
    ap.add_argument("--sweep-pause", type=int, default=0, help="one pause, moved by this many datagrams")
    ap.add_argument("--sweep-step", type=int, default=50)
    ap.add_argument("--sweep-span", type=int, default=8000, help="datagrams covered by the sweep")
    ap.add_argument("--dc-shift", type=int, default=6)
    ap.add_argument("--params-from", type=Path, default=DEFAULT_PARAMS)
    ap.add_argument("--timeout", type=float, default=60.0)
    a = ap.parse_args(argv)

    raw = np.memmap(a.file, dtype=np.int8, mode="r")
    first = int(a.start_seconds * 40e6) // SPP * SPP
    n = int(a.seconds * 40e6) // SPP * SPP
    iq8 = np.ascontiguousarray(raw[2 * first:2 * (first + n)]).reshape(-1, 2)
    print(f"{a.file.name}: {iq8.shape[0]} samples ({iq8.shape[0] // SPP} datagrams, {iq8.shape[0] / 40e3:.0f} ms) "
          f"from {first / 40e6:.3f} s")

    b = Board(a.host, a.port, a.timeout)
    info = b.ping()
    if info.build < 24 or not info.flags & pr.BOARD_PACK8:
        print("Error: needs firmware build 24+ and bitstream 9.1", file=sys.stderr)
        return 2
    params = pr.Params()
    if a.params_from and a.params_from.is_file():
        meta = json.loads(a.params_from.read_text())
        known = {f.name for f in fields(pr.Params)}
        params = pr.Params(**{k: v for k, v in meta.get("params", {}).items() if k in known})
    params.input_stage = 2
    params.dc_shift = a.dc_shift

    zeros = [tuple(int(v) for v in z.split(":")) for z in a.zero.split(",") if z]
    if a.sweep_pause:
        print(f"Sweep: one pause, at every {a.sweep_pause} datagrams over {a.sweep_span}")
        hangs = []
        for k in range(a.sweep_pause, min(a.sweep_span, iq8.shape[0] // SPP - 1), a.sweep_pause):
            ok, msg, packets, trig, tims = replay(b, params, iq8, 0, zeros, a.host, a.port, split=k)
            print(f"  pause at datagram {k:6d} ({k * SPP / 40e3:7.2f} ms): {'ok  ' if ok else 'HANG'} "
                  f"{summary(packets)}" + ("" if ok else "  <- " + msg), flush=True)
            if not ok:
                hangs.append(k)
                b = Board(a.host, a.port, a.timeout)
        print(f"\nRESULT: {len(hangs)} hang(s)" + (f" at datagrams {hangs}" if hangs else ""))
        return 1 if hangs else 0
    if not a.sweep_zero:
        t0 = time.perf_counter()
        ok, msg, packets, trig, tims = replay(b, params, iq8, a.pieces, zeros, a.host, a.port)
        how = f"in RUNs of {a.pieces} datagrams" if a.pieces else "in one RUN (or 150 ms RUNs)"
        print(f"Fed {how}" + (f", zeroed {zeros}" if zeros else "") + f" ({time.perf_counter() - t0:.1f} s): "
              f"{trig} triggers, {tims} Preamble B timings, {summary(packets)}")
        print("RESULT: " + ("no hang" if ok else "HANG - " + msg))
        return 0 if ok else 1

    print(f"Sweep: {a.sweep_zero} zeroed datagrams at every {a.sweep_step} datagrams over {a.sweep_span}"
          + (f", RUNs of {a.pieces}" if a.pieces else ""))
    hangs = []
    for k in range(0, min(a.sweep_span, iq8.shape[0] // SPP - a.sweep_zero), a.sweep_step):
        ok, msg, packets, trig, tims = replay(b, params, iq8, a.pieces, zeros + [(k, a.sweep_zero)], a.host, a.port)
        mark = "ok  " if ok else "HANG"
        print(f"  zero at datagram {k:6d} ({k * SPP / 40e3:7.2f} ms): {mark} {summary(packets)}"
              + ("" if ok else "  <- " + msg), flush=True)
        if not ok:
            hangs.append(k)
            b = Board(a.host, a.port, a.timeout)       # fresh connection after a failed RUN
    print(f"\nRESULT: {len(hangs)} hang(s)" + (f" at datagrams {hangs}" if hangs else ""))
    return 1 if hangs else 0


if __name__ == "__main__":
    sys.exit(main())
