#!/usr/bin/env python3
"""pd_udp_test.py -- can the Zedboard keep up with the CoreBoard's sample stream?

Firmware build 17 has a UDP receiver for the CoreBoard's ESR2 stream (1376-byte datagrams,
59,524 datagrams/s = ~690 Mbit/s at 40 Msps). This script, for each speed asked for:

  1. UDP_START on the Zedboard (TCP port 5001), then a second without traffic: the main loop's
     idle rate, to turn the rate under load into a CPU-load estimate
  2. streams a capture to it with App/FlexlinkCoreBoardEmulator.py --stream-to (speed x of
     real time), polling UDP_STATS every second
  3. UDP_STOP, then compares what the emulator sent with what the board counted

    mode 0  the board only checks and counts the datagrams
    mode 1  + converts the samples into the sample ring in DDR, cached, no cache maintenance
            (the conversion's CPU cost alone)
    mode 2  + converts into the ring mapped non-cacheable (what a DMA can read directly)
    mode 3  + converts into the cached ring and flushes each datagram (the other DMA-safe way)
    mode 4  + copies the raw int8 bytes into the non-cacheable ring (build 24, for the PL
            unpacker of bitstream 9.1): half the bytes of mode 2, no arithmetic
    (build 21; when the receive path cannot keep up, the board skips conversions instead of
    starving TCP, and reports them as "skipped")

Lost datagrams are split by where they went missing (build 18 reads the GEM's whole statistics
block; the PC's own send counters come from /sys/class/net):
    GEM no-buffer  the Ethernet controller had no free receive buffer: the software did not
                   give buffers back in time (CPU too slow, or the main loop blocked)
    GEM overrun    the controller could not write to memory fast enough
    fcs / symbol / alignment errors   the frame arrived damaged (cable, switch, PHY timing)
    PC tx_dropped / tx_errors         the PC's network card did not send it
    lost (rest)    dropped on the way (switch) or inside lwIP

Usage (from HW_Development/Packet_Detector/PC; the Zedboard runs build 17):
    python pd_udp_test.py --host 10.42.0.50                         # speeds 0.25, 0.5, 1, mode 0
    python pd_udp_test.py --host 10.42.0.50 --mode 1 --speed 1 --seconds 30
    python pd_udp_test.py --host 10.42.0.50 --no-emulator --seconds 10   # a real CoreBoard is
                                        # streaming to the board (started elsewhere)

The PC's own send rate is checked too: if the emulator falls behind (lag), the test says so,
as then the PC, not the board, limits the rate.
"""
from __future__ import annotations

import argparse
import re
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pd_protocol as P  # noqa: E402

ROOT = HERE.parents[2]                                   # Flexlink_py
EMULATOR = ROOT / "App" / "FlexlinkCoreBoardEmulator.py"
DEFAULT_FILE = ROOT / "App" / "CoreBoard" / "Segment_newfw_3820ms.cs8"
MODE_NAMES = {0: "count only", 1: "convert, cached ring, no flush", 2: "convert, non-cacheable ring",
              3: "convert, cached ring + flush", 4: "raw int8 copy for the PL unpacker"}
DGRAM_BYTES = 1376
WIRE_BITS = (DGRAM_BYTES + 42) * 8                       # + UDP/IP/Ethernet headers
REAL_RATE = 40e6 / 672                                   # datagrams/s at 40 Msps


class Board:
    def __init__(self, host: str, port: int):
        self.sock = socket.create_connection((host, port), timeout=10)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.seq = 0
        self.build = 0

    def call(self, cmd: int, payload: bytes = b"") -> bytes:
        self.seq += 1
        P.send_msg(self.sock, cmd, payload, self.seq)
        rcmd, status, seq, data = P.recv_msg(self.sock)
        if status != P.ST_OK:
            raise RuntimeError(f"{P.CMD_NAMES.get(cmd, cmd)} failed (status {status}): "
                               f"{data.decode(errors='replace').strip()}")
        return data

    def ping(self) -> P.BoardInfo:
        info = P.BoardInfo.unpack(self.call(P.PING))
        self.build = info.build
        return info

    def start(self, port: int, mode: int) -> int:
        return int.from_bytes(self.call(P.UDP_START, port.to_bytes(2, "little") + mode.to_bytes(2, "little"))[:4],
                              "little")

    def stats(self) -> P.UdpStats:
        return P.UdpStats.unpack(self.call(P.UDP_STATS))

    def stop(self) -> P.UdpStats:
        return P.UdpStats.unpack(self.call(P.UDP_STOP))

    def gem(self) -> dict:
        d = self.call(P.GEM_STATS)
        vals = struct.unpack(f"<{len(d) // 4}I", d)
        return dict(zip(P.GEM_STAT_NAMES, vals))


def pc_interface(host: str) -> str | None:
    """The PC network interface that leads to host (Linux)."""
    try:
        out = subprocess.run(["ip", "route", "get", host], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    m = re.search(r"\bdev (\S+)", out)
    return m.group(1) if m else None


PC_COUNTERS = ("tx_packets", "tx_dropped", "tx_errors", "tx_fifo_errors", "tx_aborted_errors",
               "tx_carrier_errors", "collisions")


def pc_counters(ifname: str | None) -> dict:
    vals = {}
    if ifname:
        for k in PC_COUNTERS:
            try:
                vals[k] = int(Path(f"/sys/class/net/{ifname}/statistics/{k}").read_text())
            except (OSError, ValueError):
                pass
    return vals


def line(s: P.UdpStats, prev: P.UdpStats | None, t_prev: float | None) -> str:
    if prev is not None and s.elapsed_ms > prev.elapsed_ms:
        dt = (s.elapsed_ms - prev.elapsed_ms) / 1e3
        rate = (s.datagrams - prev.datagrams) / dt
        d_lost = s.lost - prev.lost
    else:
        rate, d_lost = 0.0, 0
    return (f"  {s.elapsed_ms / 1e3:6.1f} s  {rate:8.0f} datagrams/s ({rate * WIRE_BITS / 1e6:4.0f} Mbit/s)  "
            f"lost {s.lost} (+{d_lost})  late {s.late}  dup {s.duplicates}  "
            f"GEM no-buffer {s.gem_no_buffer} overrun {s.gem_overrun}"
            + (f"  SKIPPED {s.skipped}" if s.skipped else "")
            + (f"  POOL EMPTY {s.pool_empty}" if s.pool_empty else ""))


def run_one(board: Board, a, speed: float | None) -> bool:
    port = board.start(a.udp_port, a.mode)
    time.sleep(1.0)
    idle = board.stats()
    idle_rate = idle.loops / max(idle.elapsed_ms, 1) * 1e3          # main-loop iterations/s, idle
    label = f"speed x{speed:g}" if speed is not None else "external stream"
    print(f"\n=== {label}, mode {a.mode} ({MODE_NAMES[a.mode]}), UDP port {port} ===")

    ifname = pc_interface(a.host)
    pc0 = pc_counters(ifname)
    emu = None
    if speed is not None:
        cmd = [sys.executable, str(EMULATOR), "--file", str(a.file), "--stream-to", f"{a.host}:{port}",
               "--seconds", str(a.seconds), "--speed", str(speed), "--sndbuf-mb", "64"]
        if a.drop:
            cmd += ["--drop", str(a.drop)]
        emu = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    prev, base = None, board.stats()
    t_end = time.monotonic() + (a.seconds / speed + 30 if emu else a.seconds)
    while time.monotonic() < t_end:
        time.sleep(1.0)
        s = board.stats()
        print(line(s, prev, None), flush=True)
        prev = s
        if emu is not None and emu.poll() is not None:
            break
    emu_out = ""
    if emu is not None:
        emu_out = emu.communicate(timeout=60)[0]
    time.sleep(0.5)                                     # the last datagrams in flight
    s = board.stop()

    pc1 = pc_counters(ifname)
    gem = board.gem() if board.build >= 18 else {}
    loop_rate = (s.loops - base.loops) / max(s.elapsed_ms - base.elapsed_ms, 1) * 1e3
    cpu = max(0.0, 1 - loop_rate / idle_rate) if idle_rate else float("nan")
    rate = s.datagrams / (s.active_ms / 1e3) if s.active_ms else 0.0
    busy = s.busy_ms / s.active_ms if s.active_ms else 0.0
    print(f"  board: {s.datagrams} datagrams in {s.active_ms / 1e3:.2f} s = {rate:.0f}/s "
          f"({rate * WIRE_BITS / 1e6:.0f} Mbit/s, x{rate / REAL_RATE:.2f} of 40 Msps), "
          f"lost {s.lost}, late {s.late}, duplicates {s.duplicates}, bad {s.bad_size + s.bad_magic}, "
          f"stream restarts {max(0, s.epochs - 1)}")
    if gem:
        big = gem.get("rx_1024_1518", 0)
        errs = {k: v for k, v in gem.items() if k in P.GEM_ERROR_NAMES and v}
        frames = sum(gem.get(k, 0) for k in ("rx_64", "rx_65_127", "rx_128_255", "rx_256_511",
                                              "rx_512_1023", "rx_1024_1518"))
        print(f"  GEM: {frames} frames received ({big} of 1024..1518 bytes), "
              f"{gem['octets_rx_lo'] / 1e6:.0f} MB; errors: "
              + (", ".join(f"{k} {v}" for k, v in errs.items()) if errs else "none"))
        if a.gem_all:
            print("  GEM statistics: " + ", ".join(f"{k} {v}" for k, v in gem.items() if v))
    else:
        print(f"  GEM: {s.gem_frames} frames, {s.gem_no_buffer} dropped (no buffer), {s.gem_overrun} dropped (overrun)")
    if pc0 and pc1:
        d = {k: pc1[k] - pc0.get(k, 0) for k in pc1}
        bad = {k: v for k, v in d.items() if v and k != "tx_packets"}
        print(f"  PC {ifname}: {d.get('tx_packets', 0)} packets sent; "
              + ("drops/errors: " + ", ".join(f"{k} {v}" for k, v in bad.items()) if bad else "no send drops or errors"))
    if s.pool_empty:
        print(f"  lwIP pbuf pool empty {s.pool_empty} times in the receive interrupt (each a lost frame)")
    if s.skipped:
        print(f"  OVERLOAD: {s.skipped} of {s.datagrams} conversions skipped "
              f"({100 * s.skipped / max(s.datagrams, 1):.1f} %) - this mode is too slow at this rate")
    print(f"  CPU: UDP callback {100 * busy:.0f} % of the time; whole receive path ~{100 * cpu:.0f} % "
          f"(main loop {loop_rate:.0f}/s vs {idle_rate:.0f}/s idle)")

    ok = s.lost == 0 and s.gem_no_buffer == 0 and s.gem_overrun == 0 and s.datagrams > 0 and not s.skipped \
        and not s.pool_empty
    if emu is not None:
        m = re.search(r"Last stream: (\d+) datagrams.*?lag max ([\d.]+) ms", emu_out)
        dropped = re.search(r"dropped (\d+)", emu_out.splitlines()[-1] if emu_out else "")
        if not m:
            print("  emulator output:\n" + emu_out)
            return False
        sent, lag = int(m.group(1)), float(m.group(2))
        injected = int(dropped.group(1)) if dropped else 0
        missing = sent - s.datagrams
        print(f"  emulator: {sent} sent ({injected} dropped on purpose), lag max {lag:.0f} ms; "
              f"not counted by the board: {missing}")
        if lag > 50:
            print(f"  NOTE: the PC fell behind by up to {lag:.0f} ms, so it sent in bursts; at this speed "
                  f"the PC limits the test, not the board")
        ok = missing == 0 and s.datagrams > 0 and s.gem_no_buffer == 0 and s.gem_overrun == 0 \
            and s.lost == injected and not s.skipped
        if a.drop:
            print("  check: the board's lost count " + ("matches" if s.lost == injected else "does NOT match")
                  + " the emulator's deliberate drops")
    print("  RESULT: " + ("no datagram lost" if ok else "datagrams lost - see above where"))
    return ok


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Zedboard UDP stream throughput test (firmware build 17).",
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--host", default="192.168.50.10", help="Zedboard address (printed on its UART)")
    ap.add_argument("--tcp-port", type=int, default=P.DEFAULT_PORT)
    ap.add_argument("--udp-port", type=int, default=P.UDP_DEFAULT_PORT)
    ap.add_argument("--mode", default="0",
                    help="0 count only; 1 convert (cached), 2 convert (non-cacheable), 3 convert (cached + flush); "
                         "several comma separated, e.g. 1,2,3")
    ap.add_argument("--speed", default="0.25,0.5,1", help="emulator speeds to try, comma separated")
    ap.add_argument("--seconds", type=float, default=10.0, help="seconds of samples streamed per speed (takes seconds / speed to send)")
    ap.add_argument("--file", type=Path, default=DEFAULT_FILE, help="capture streamed by the emulator")
    ap.add_argument("--drop", type=float, default=0.0, help="emulator drops datagrams on purpose (counter check)")
    ap.add_argument("--no-emulator", action="store_true", help="only count a stream sent from elsewhere")
    ap.add_argument("--gem-all", action="store_true", help="print every non-zero GEM statistics counter")
    a = ap.parse_args(argv)

    if not a.no_emulator and not EMULATOR.is_file():
        print(f"Error: {EMULATOR} not found", file=sys.stderr)
        return 3
    if not a.no_emulator and not a.file.is_file():
        print(f"Error: {a.file} not found (give --file)", file=sys.stderr)
        return 3
    try:
        board = Board(a.host, a.tcp_port)
        info = board.ping()
    except (OSError, RuntimeError) as e:
        print(f"Error: no answer from the Zedboard at {a.host}:{a.tcp_port} ({e})", file=sys.stderr)
        return 3
    print(f"Zedboard at {a.host}: firmware build {info.build}")
    if info.build < 17:
        print("Error: the UDP receiver needs firmware build 17 or later", file=sys.stderr)
        return 3

    speeds = [None] if a.no_emulator else [float(x) for x in a.speed.split(",")]
    modes = [int(m) for m in a.mode.split(",")]
    if any(m not in MODE_NAMES for m in modes) or (info.build < 21 and any(m > 1 for m in modes)) \
            or (info.build < 24 and 4 in modes):
        print("Error: modes are 0..4 (2 and 3 need firmware build 21, 4 needs build 24)", file=sys.stderr)
        return 3
    results = []
    for mode in modes:
        for sp in speeds:
            a.mode = mode
            try:
                results.append((mode, sp, run_one(board, a, sp)))
            except KeyboardInterrupt:
                board.stop()
                print("\nInterrupted")
                return 3
    if len(results) > 1:
        print("\nSummary: " + ", ".join(f"mode {m} " + (f"x{sp:g} " if sp else "") + ("OK" if ok else "LOSS")
                                       for m, sp, ok in results))
    return 0 if all(ok for _, _, ok in results) else 1


if __name__ == "__main__":
    sys.exit(main())
