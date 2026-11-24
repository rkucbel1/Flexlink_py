#!/usr/bin/env python3
"""pd_stream_test.py -- the CoreBoard stream through the Zedboard's FPGA receiver, live.

Firmware build 25 (bitstream 9.1) receives the CoreBoard's ESR2 UDP stream into a ring in DDR
and feeds it to the receiver with the AXI DMA as it arrives (PACK8: the PL unpacks the int8
samples). This script:

  1. sets the receiver's parameters (TestVectors/rep3.json, input stage 2 = 40 Msps,
     dc_shift 6 for the CoreBoard's DC wander)
  2. STREAM_START: the Zedboard opens its UDP port and itself sends the HTTP "start" to the
     sample source, so the stream comes to the Zedboard:
       default        App/FlexlinkCoreBoardEmulator.py, started here on this PC, plays a capture
       --coreboard IP the real CoreBoard (configured from App/Rx_CoreBoard/Parameters_CoreBoard.json)
  3. polls the results (POLL_RESULTS) and prints once a second: stream rate, losses, ring
     level, packets decoded, transport blocks CRC ok
  4. STREAM_STOP (the Zedboard sends the HTTP "stop"), summary

Usage (from HW_Development/Packet_Detector/PC; the PC firewall must let the Zedboard reach the
emulator's HTTP port, e.g. sudo ufw allow from <zedboard IP> to any port 8080 proto tcp):
    python pd_stream_test.py --host 10.42.0.101                      # emulator, EmuSource.cs8
    python pd_stream_test.py --host 10.42.0.101 --seconds 30 --drop 1e-4
    python pd_stream_test.py --host 10.42.0.101 --seconds 30 --ddr-wr-prio=-,16,-,-   # build 28
    python pd_stream_test.py --host 10.42.0.101 --coreboard 10.42.0.133 --seconds 20
                                     # real CoreBoard: start the Pluto transmitter yourself

Expected with the emulator on EmuSource.cs8 (from 2.6 s, where the transmitter started): one
packet per 5.203 ms (192 per second) while the capture has packets, every transport block
CRC ok; the emulator loops the file, so the packets repeat.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import socket
import struct
import subprocess
import sys
import time
import urllib.request
from dataclasses import fields
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pd_protocol as pr  # noqa: E402
from pd_hw_test import Board  # noqa: E402

ROOT = HERE.parents[2]
EMULATOR = ROOT / "App" / "FlexlinkCoreBoardEmulator.py"
DEFAULT_FILE = ROOT / "App" / "Rx_CoreBoard" / "Captures" / "EmuSource.cs8"
CB_SETTINGS = ROOT / "App" / "Rx_CoreBoard" / "Parameters_CoreBoard.json"
DEFAULT_PARAMS = HERE.parent / "TestVectors" / "rep3.json"


def local_ip_towards(host: str) -> str:
    """This PC's address on the way to host."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((host, 9))
        return s.getsockname()[0]
    finally:
        s.close()


def coreboard_start_json(port: int) -> dict:
    """The capture script's 'start' (App/FlexlinkCaptureWithCoreBoard.py), from its settings."""
    cb = json.loads(CB_SETTINGS.read_text()).get("CoreBoard", {})
    cfg = {"op": "start", "port": port, "frequency": int(cb.get("Frequency_MHz", 2440)) * 1_000_000,
           "rate": int(cb.get("SampleRate", 40_000_000)), "agc": int(cb.get("AGC", 0)),
           "bandwidth": int(cb.get("Bandwidth_MHz", 0)) * 1_000_000, "dc_correction": int(cb.get("DC_Correction", 1))}
    if not cfg["agc"]:
        cfg["gain"] = int(cb.get("Gain", 40))
    return cfg


def http_status(host: str, port: int) -> dict:
    req = urllib.request.Request(f"http://{host}:{port}/api/rx", data=b'{"op": "status"}',
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read().decode())


def stall_text(st: pr.StreamStats) -> str:
    sr = st.stall_sr
    why = ("DMA halted" if sr & 1 else "DMA running") + (", DMA error bits 0x%x" % ((sr >> 4) & 7) if sr & 0x70 else "")
    return (f"DMA STALL x{st.stalls}: MM2S SR 0x{sr:08x} ({why}), receiver STATUS 0x{st.stall_status:08x}, "
            f"samples {st.stall_samples}, FIFO levels ev {st.stall_ev} tm {st.stall_tm} cf {st.stall_cf} "
            f"pl {st.stall_pl} ch {st.stall_ch} lq {st.stall_lq} tb {st.stall_tb}; MM2S SR now 0x{st.dma_sr:08x}")


def start_payload(udp_port: int, flags: int, ip: str | None, http_port: int, body: dict | None) -> bytes:
    js = json.dumps(body).encode() if body else b""
    ip32 = struct.unpack("<I", socket.inet_aton(ip))[0] if ip else 0
    return struct.pack("<HHIHH", udp_port, flags, ip32, http_port, len(js)) + js


class Tally:
    def __init__(self):
        self.packets: dict[int, list] = {}         # t_trig -> [blocks, crc ok, max iterations]
        self.triggers = self.timing_ok = self.cf_ok = self.cf = 0
        self.bits_ok = 0

    def add(self, recs):
        evs, _, tms, cfs, _, _, _, tbs = recs
        self.triggers += len(evs)
        self.timing_ok += sum(t.status == pr.TM_GOOD for t in tms)
        self.cf += len(cfs)
        self.cf_ok += sum(c.crc_ok for c in cfs)
        for t in tbs:
            p = self.packets.setdefault(t.t_trig, [0, 0, 0])
            p[0] += 1
            p[1] += t.crc_ok
            p[2] = max(p[2], t.iters)
            self.bits_ok += t.nbits if t.crc_ok else 0

    @property
    def blocks(self) -> tuple[int, int]:
        return sum(p[0] for p in self.packets.values()), sum(p[1] for p in self.packets.values())


DDRC_WR_PRIO = 0xF8006208          # UG585 axi_priority_wr_port0..3, then axi_priority_rd_port0..3
DDRC_RD_PRIO = 0xF8006218          # bits 9:0 = aging counter start value, lower = more priority


class BlockAges:
    """Push mode: how old each transport block is when its push leaves the board - the stream
    position the board had then (its newest datagram) minus the block's trigger, in receiver
    samples (20 Msps). Includes the packet's own air time up to that block; the simulation
    gives that part (CoreBoard vector: first block 1.31 ms, last block up to 4.92 ms after the
    trigger), the rest is the board: ring, FIFO drain, push interval."""
    def __init__(self):
        self.first: dict[int, float] = {}       # t_trig -> age (ms) of its first block
        self.last: dict[int, float] = {}        # t_trig -> age (ms) of its last block
        self.arrive: list[float] = []

    def add(self, slot, tbs, t_now: float):
        if slot is None:                        # the STREAM_STOP reply: no stream position
            return
        now20 = (slot * pr.PUSH_SLOT_SAMPLES) & 0xFFFFFFFF
        for t in tbs:
            d = (now20 - t.t_trig) & 0xFFFFFFFF
            if d >= 1 << 31:
                continue
            ms = d / 20e3
            self.first.setdefault(t.t_trig, ms)
            self.last[t.t_trig] = ms
        if tbs:
            self.arrive.append(t_now)

    @staticmethod
    def _q(v: list[float]) -> str:
        if not v:
            return "-"
        v = sorted(v)
        return (f"median {v[len(v) // 2]:.2f}, 99 % {v[min(len(v) - 1, int(len(v) * 0.99))]:.2f}, "
                f"max {v[-1]:.2f} ms")

    def text(self) -> str:
        gaps = [b - a for a, b in zip(self.arrive, self.arrive[1:])]
        return (f"block age when pushed (after the trigger): first block of a packet {self._q(list(self.first.values()))}; "
                f"last block {self._q(list(self.last.values()))} (simulation, FPGA alone: 1.31 / up to 4.92 ms)"
                + (f"\n  pushes with blocks: {len(self.arrive)}, longest gap between them at the PC "
                   f"{max(gaps) * 1e3:.1f} ms" if gaps else ""))


class PollTimes:
    """How long POLL_RESULTS took and how far apart the polls were (the board keeps about 0.5 s
    of transport blocks, build 29; a longer gap loses blocks)."""
    def __init__(self):
        self.n, self.call_max, self.gap_max, self.slow, self.prev = 0, 0.0, 0.0, 0, None

    def add(self, t_send: float, t_done: float):
        self.n += 1
        self.call_max = max(self.call_max, t_done - t_send)
        if self.prev is not None:
            gap = t_done - self.prev
            self.gap_max = max(self.gap_max, gap)
            self.slow += gap > 0.25
        self.prev = t_done

    def text(self) -> str:
        return (f"polls: {self.n}, longest reply {self.call_max * 1e3:.0f} ms, longest gap {self.gap_max * 1e3:.0f} ms, "
                f"{self.slow} gaps over 250 ms")


GEM_NWCFG, GEM_DMACR = 0xE000B004, 0xE000B010   # Ethernet controller (UG585 net_cfg, dma_cfg)


def gem_text(b: Board) -> tuple[str, int]:
    """DMACR (dma_cfg): bits 4:0 AHB burst length for frame data (1 single, 4 INCR4, 8 INCR8,
    16 INCR16), 9:8 RX packet buffer size, 10 TX packet buffer size, 11 TX checksum offload,
    23:16 RX buffer size in 64-byte units (firmware build 33)."""
    nwcfg = struct.unpack("<I", b.call(9, struct.pack("<I", GEM_NWCFG)))[0]
    dmacr = struct.unpack("<I", b.call(9, struct.pack("<I", GEM_DMACR)))[0]
    return (f"DMACR 0x{dmacr:08x} (burst {dmacr & 0x1F}, RX pkt buffer sel {(dmacr >> 8) & 3}, "
            f"TX pkt buffer sel {(dmacr >> 10) & 1}, TX csum {(dmacr >> 11) & 1}, RX buffers {((dmacr >> 16) & 0xFF) * 64} B), "
            f"NWCFG 0x{nwcfg:08x}"), dmacr


def ddr_regs(b: Board) -> list[int]:
    """The 8 DDR port priority registers (firmware build 28): write ports 0..3, read ports 0..3."""
    return [struct.unpack("<I", b.call(9, struct.pack("<I", DDRC_WR_PRIO + 4 * i)))[0] for i in range(8)]


def ddr_text(v: list[int]) -> str:
    return ("wr " + " ".join(f"{x & 0x3FF:3d}" for x in v[:4]) + "   rd " + " ".join(f"{x & 0x3FF:3d}" for x in v[4:])
            + "   (ports 0 CPU, 1 GEM/central, 2 HP2/3, 3 HP0/1; raw " + " ".join(f"{x:05x}" for x in v) + ")")


def ddr_set(b: Board, base: int, spec: str, cur: list[int]) -> None:
    """spec "P0,P1,P2,P3": new priority (0..1023) per port, "-" keeps it; only bits 9:0 change."""
    parts = spec.split(",")
    if len(parts) != 4:
        raise SystemExit("DDR priorities: give four values P0,P1,P2,P3 (\"-\" keeps one)")
    off = 0 if base == DDRC_WR_PRIO else 4
    for i, t in enumerate(parts):
        t = t.strip()
        if t in ("", "-"):
            continue
        v = int(t, 0)
        if not 0 <= v <= 0x3FF:
            raise SystemExit(f"DDR priority {v} out of range 0..1023")
        b.call(10, struct.pack("<II", base + 4 * i, (cur[off + i] & ~0x3FF) | v))


def ddr_restore(b: Board, v: list[int]) -> None:
    for i, x in enumerate(v):
        b.call(10, struct.pack("<II", DDRC_WR_PRIO + 4 * i, x))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="CoreBoard stream through the Zedboard receiver, live.",
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--host", default="192.168.50.10", help="Zedboard")
    ap.add_argument("--port", type=int, default=pr.DEFAULT_PORT)
    ap.add_argument("--udp-port", type=int, default=pr.UDP_DEFAULT_PORT)
    ap.add_argument("--seconds", type=float, default=10.0, help="stream length")
    ap.add_argument("--coreboard", metavar="IP[:PORT]", help="use the real CoreBoard instead of the emulator")
    ap.add_argument("--file", type=Path, default=DEFAULT_FILE, help="emulator: capture to play")
    ap.add_argument("--start-seconds", type=float, default=2.6, help="emulator: begin this far into the file")
    ap.add_argument("--http-port", type=int, default=8080, help="emulator: its HTTP port on this PC")
    ap.add_argument("--drop", type=float, default=0.0, help="emulator: lose datagrams on purpose")
    ap.add_argument("--emu-pace", type=int, default=8,
                    help="emulator: datagrams sent back to back (default 8; 128 = the old bursts)")
    ap.add_argument("--pace-us", type=int, default=0, metavar="US",
                    help="board sends one TCP segment of results per US microseconds while streaming "
                         "(build 36; multiple of 25, default 100)")
    ap.add_argument("--push", action="store_true",
                    help="the board pushes results as they come (build 39) instead of being polled")
    ap.add_argument("--push-ms", type=float, default=1.0,
                    help="with --push: at most one push per this many ms while there are new results (0.25 .. 7.75)")
    ap.add_argument("--no-pace", action="store_true",
                    help="board sends replies in bursts, not one TCP segment per 100 us (build 35 test)")
    ap.add_argument("--no-pin", action="store_true",
                    help="emulator: do not give it a CPU core of its own (Linux)")
    ap.add_argument("--dc-shift", type=int, default=6, help="receiver DC blocker (6 for the CoreBoard)")
    ap.add_argument("--params-from", type=Path, default=DEFAULT_PARAMS)
    ap.add_argument("--keep-pl", action="store_true", help="also return payload / channel / decoder-input records")
    ap.add_argument("--poll", type=float, default=0.05,
                    help="seconds between POLL_RESULTS (the firmware keeps ~900 transport blocks between polls)")
    ap.add_argument("--ddr-wr-prio", metavar="P0,P1,P2,P3",
                    help="DDR controller write-port priorities for this run (lower = more priority, "
                         "default 1023 each; e.g. \"-,16,-,-\" favours the GEM); restored afterwards")
    ap.add_argument("--ddr-rd-prio", metavar="P0,P1,P2,P3", help="the same for the read ports")
    ap.add_argument("--gem-burst", type=int, choices=(1, 4, 8, 16),
                    help="Ethernet controller DMA burst length for this run (build 33); restored afterwards")
    a = ap.parse_args(argv)

    emu = None
    emu_line = ""
    rx_bytes = 0
    gem_saved = None
    ddr_saved = None
    b = None
    try:
        b = Board(a.host, a.port, 60.0)
        info = b.ping()
        print(f"Zedboard at {a.host}: firmware build {info.build}"
              + (", bitstream 9.2 (NOREC)" if info.flags & pr.BOARD_NOREC else ""))
        if info.build < 25 or not info.flags & pr.BOARD_PACK8:
            print("Error: streaming needs firmware build 25 and bitstream 9.1", file=sys.stderr)
            return 2
        if info.build >= 28:
            ddr_saved = ddr_regs(b)
            if a.ddr_wr_prio or a.ddr_rd_prio:
                if a.ddr_wr_prio:
                    ddr_set(b, DDRC_WR_PRIO, a.ddr_wr_prio, ddr_saved)
                if a.ddr_rd_prio:
                    ddr_set(b, DDRC_RD_PRIO, a.ddr_rd_prio, ddr_saved)
                print(f"DDR port priorities: {ddr_text(ddr_saved)}")
                print(f"           this run: {ddr_text(ddr_regs(b))}")
            else:
                print(f"DDR port priorities: {ddr_text(ddr_saved)}")
                ddr_saved = None                     # nothing to restore
        elif a.ddr_wr_prio or a.ddr_rd_prio:
            print("Error: --ddr-wr-prio / --ddr-rd-prio need firmware build 28", file=sys.stderr)
            return 2
        if info.build >= 33:
            txt, dmacr = gem_text(b)
            print(f"Ethernet controller: {txt}")
            if a.gem_burst and (dmacr & 0x1F) != a.gem_burst:
                gem_saved = dmacr
                b.call(10, struct.pack("<II", GEM_DMACR, (dmacr & ~0x1F) | a.gem_burst))
                print(f"           this run: {gem_text(b)[0]}")
        elif a.gem_burst:
            print("Error: --gem-burst needs firmware build 33", file=sys.stderr)
            return 2
        params = pr.Params()
        if a.params_from and a.params_from.is_file():
            meta = json.loads(a.params_from.read_text())
            known = {f.name for f in fields(pr.Params)}
            params = pr.Params(**{k: v for k, v in meta.get("params", {}).items() if k in known})
        params.input_stage = 2
        params.dc_shift = a.dc_shift
        b.call(pr.SET_PARAMS, params.pack())

        if a.coreboard:
            cb_host, _, cb_p = a.coreboard.partition(":")
            cb_port = int(cb_p or 80)
            st = http_status(cb_host, cb_port)
            print(f"CoreBoard {st.get('product', '?')} at {cb_host}: Ethernet {st.get('ethernet_mbps')} Mbit/s")
            body = coreboard_start_json(a.udp_port)
            src_ip, src_port = cb_host, cb_port
            print(f"  start: {body['frequency'] / 1e6:g} MHz, {body['rate'] / 1e6:g} Msps, "
                  f"{'AGC' if body['agc'] else 'gain ' + str(body.get('gain'))} - start the transmitter now")
        else:
            if not a.file.is_file():
                print(f"Error: {a.file} not found (give --file)", file=sys.stderr)
                return 2
            src_ip, src_port = local_ip_towards(a.host), a.http_port
            cmd = [sys.executable, str(EMULATOR), "--file", str(a.file), "--http-port", str(a.http_port),
                   "--start-seconds", str(a.start_seconds), "--sndbuf-mb", "64", "--pace", str(a.emu_pace)]
            if a.drop:
                cmd += ["--drop", str(a.drop)]
            pin = None
            if not a.no_pin and hasattr(os, "sched_setaffinity") and (os.cpu_count() or 1) >= 4:
                ncpu = os.cpu_count()
                pin = ncpu - 1                       # the emulator alone on the last core, this
                os.sched_setaffinity(0, set(range(ncpu - 1)))   # script (parsing results) elsewhere
            emu = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                   preexec_fn=(lambda: os.sched_setaffinity(0, {pin})) if pin is not None else None)
            for _ in range(50):                      # wait for its HTTP server
                try:
                    http_status("127.0.0.1", a.http_port)
                    break
                except OSError:
                    time.sleep(0.1)
            else:
                print("Error: the emulator did not start:\n" + (emu.stdout.read() if emu.poll() is not None else ""),
                      file=sys.stderr)
                return 2
            body = {"op": "start", "port": a.udp_port}
            print(f"Emulator on this PC ({src_ip}:{src_port}) plays {a.file.name} from {a.start_seconds:g} s, "
                  f"pace {a.emu_pace}" + (f", on CPU {pin}" if pin is not None else ""))

        if a.pace_us and info.build < 36:
            print("Error: --pace-us needs firmware build 36", file=sys.stderr)
            return 2
        pace_units = min(max(round(a.pace_us / 25), 1), 255) if a.pace_us else 0
        if a.push and info.build < 39:
            print("Error: --push needs firmware build 39", file=sys.stderr)
            return 2
        push_units = min(max(round(a.push_ms * 4), 1), 31) if a.push else 0
        flags = ((pr.STREAM_KEEP_PL if a.keep_pl else 0) | (pace_units << 8)
                 | ((pr.STREAM_FLAG_PUSH | (push_units << 2)) if a.push else 0)
                 | (0x80 if a.no_pace else 0))

        def poll_parse(raw: bytes):
            nonlocal rx_bytes
            rx_bytes += len(raw)
            return pr.parse_poll(raw)

        reply = b.call(pr.STREAM_START, start_payload(a.udp_port, flags,
                                                      src_ip, src_port, body))
        if info.build >= 35:
            print("Results paced: " + ("off (bursts)" if a.no_pace else
                                       f"one TCP segment per {25 * pace_units if pace_units else 100} us")
                  + (f"; pushed by the board, at most every {push_units / 4:g} ms" if a.push
                     else f"; polled every {a.poll:g} s"))
        print(f"Streaming to the Zedboard's UDP port {struct.unpack('<I', reply[:4])[0]}, receiver at 40 Msps, "
              f"dc_shift {a.dc_shift}\n")
        print("   time  datagrams/s  lost  late  ring max  resync  packets  per s   TB CRC ok    stalls  HTTP")
        tally = Tally()
        polls = PollTimes()
        t0 = time.monotonic()
        t_print, prev_dg, prev_pk, prev_t = t0 + 1.0, 0, 0, t0
        last_stalls = 0
        st = None
        ages = BlockAges()

        def show(st):
            nonlocal t_print, prev_dg, prev_pk, prev_t, last_stalls
            if st.lost_mask:
                print("  WARNING: record buffers overflowed (" + ", ".join(
                    v for k, v in pr.STREAM_LOST_NAMES.items() if st.lost_mask & k) + ") - "
                      + ("the board could not send them in time" if a.push else
                         "a poll came too late, see polls: in the summary"))
            now = time.monotonic()
            if now >= t_print:
                dt = now - prev_t
                tot, ok = tally.blocks
                npk = len(tally.packets)
                print(f"  {now - t0:5.1f}  {(st.datagrams - prev_dg) / dt:11.0f}  {st.lost:4d}  {st.late_fixed:4d}  "
                      f"{st.ring_max:8d}  {st.resyncs:6d}  {npk:7d}  {(npk - prev_pk) / dt:5.0f}  "
                      f"{ok:6d}/{tot:<6d}  {st.stalls:6d}  {pr.HTTP_STATES.get(st.http_state, '?')} {st.http_code or ''}",
                      flush=True)
                if st.stalls and st.stalls != last_stalls:
                    print("  " + stall_text(st))
                    last_stalls = st.stalls
                if st.http_state == 4:
                    print(f"  HTTP request to {src_ip}:{src_port} failed (lwIP error {st.http_err - (1 << 32) if st.http_err > 1 << 31 else st.http_err}): "
                          f"is the firewall open for the Zedboard?")
                prev_dg, prev_pk, prev_t, t_print = st.datagrams, npk, now, now + 1.0

        def on_push(raw: bytes):
            nonlocal rx_bytes, st
            rx_bytes += len(raw)
            slot, st_new, recs = pr.parse_push(raw)
            tally.add(recs)
            ages.add(slot, recs[7], time.monotonic())
            if st_new is not None:
                st = st_new
                show(st)

        stop_js = json.dumps({"op": "stop"}).encode()
        stop_payload = struct.pack("<H", len(stop_js)) + stop_js
        if a.push:
            b.sock.settimeout(5.0)
            while time.monotonic() - t0 < a.seconds:
                cmd, status, _, raw = pr.recv_msg(b.sock)
                if cmd != pr.STREAM_PUSH:
                    raise pr.ProtocolError(f"unexpected message {pr.CMD_NAMES.get(cmd, cmd)} while streaming")
                on_push(raw)
            b.seq += 1
            pr.send_msg(b.sock, pr.STREAM_STOP, stop_payload, b.seq)
            while True:                     # pushes already on their way, then the reply
                cmd, status, seq, raw = pr.recv_msg(b.sock)
                if cmd == pr.STREAM_PUSH:
                    on_push(raw)
                    continue
                if cmd != pr.STREAM_STOP or seq != b.seq or status != pr.ST_OK:
                    raise pr.ProtocolError(f"STREAM_STOP reply: cmd {cmd} seq {seq} status {status}")
                break
            st, recs = poll_parse(raw)
            ages.add(None, recs[7], time.monotonic())
        else:
            while time.monotonic() - t0 < a.seconds:
                time.sleep(a.poll)
                tp = time.monotonic()
                st, recs = poll_parse(b.call(pr.POLL_RESULTS))
                polls.add(tp, time.monotonic())
                tally.add(recs)
                show(st)
            st, recs = poll_parse(b.call(pr.STREAM_STOP, stop_payload))
        tally.add(recs)
    except KeyboardInterrupt:
        try:
            b.call(pr.STREAM_STOP, struct.pack("<H", 0))
        except Exception:  # noqa: BLE001
            pass
        print("\nInterrupted")
        return 3
    except (OSError, pr.ProtocolError) as e:
        print(f"Error: {e}", file=sys.stderr)
        return 2
    finally:
        if gem_saved is not None and b is not None:
            try:
                b.call(10, struct.pack("<II", GEM_DMACR, gem_saved))
            except Exception as e:  # noqa: BLE001
                print(f"Warning: Ethernet DMA setting not restored ({e}); a reset of the board restores it")
        if ddr_saved is not None and b is not None:
            try:
                ddr_restore(b, ddr_saved)
            except Exception as e:  # noqa: BLE001
                print(f"Warning: DDR priorities not restored ({e}); a reset of the board restores them")
        if emu is not None:
            emu.terminate()
            try:
                emu.wait(5)
            except subprocess.TimeoutExpired:
                emu.kill()
            try:
                lines = [ln for ln in emu.stdout.read().splitlines() if "lag max" in ln]
                emu_line = lines[-1].split("  ", 1)[-1].strip() if lines else ""
            except Exception:  # noqa: BLE001
                emu_line = ""

    tot, ok = tally.blocks
    npk = len(tally.packets)
    secs = st.elapsed_ms / 1e3 if st else a.seconds
    full = sum(1 for p in tally.packets.values() if p[0] == max(q[0] for q in tally.packets.values())) if npk else 0
    print(f"\nStream: {st.datagrams} datagrams in {secs:.1f} s ({st.datagrams / max(secs, 1e-9):.0f}/s), "
          f"{st.lost} lost (zero-filled), {st.late_fixed} late fixed, {st.too_late} too late, "
          f"{st.ring_full} ring full, {st.resyncs} resyncs, {st.bad} bad, {st.duplicates} duplicates")
    # the receiver's count is 32 bits (wraps every 107 s at 40 Msps): unwrap against fed_samples
    took = st.det_samples + ((st.fed_samples - st.det_samples) // (1 << 32)) * (1 << 32) \
        if st.fed_samples >= st.det_samples else st.det_samples
    print(f"  ring: max {st.ring_max} datagrams waiting ({st.ring_max * 672 / 40e3:.1f} ms), {st.dma_chunks} DMA chunks, "
          f"{st.dma_errors} DMA errors; fed {st.fed_samples} samples, receiver took {took}; "
          f"lwIP pool empty {st.pool_empty}, GEM no-buffer {st.gem_no_buffer}, overrun {st.gem_overrun}")
    if a.push:
        print("  " + ages.text())
    print("  " + (f"pushed results" if a.push else polls.text()) + f"; the board sent {rx_bytes / 1e6:.1f} MB of results "
          f"({rx_bytes * 8 / 1e6 / max(secs, 1e-9):.1f} Mbit/s)")
    if emu is not None and emu_line:
        print(f"  emulator: {emu_line}  (lag = how far it fell behind real time; it then sends at line rate)")
    c = getattr(st, "cpu", None)
    if c:
        ms = max(st.elapsed_ms, 1)
        print(f"  board CPU: UDP receive {c['rx_us'] / ms / 10:.1f} % (includes the FIFO drains), "
              f"FIFO drains {c['drain_us'] / ms / 10:.1f} % ({c['drains']} drains, longest {c['drain_max_us']} us, "
              f"{c['pd_reads']} register reads = {c['pd_reads'] / ms * 1e3:.0f}/s), "
              f"polls {c['poll_us'] / ms / 10:.1f} % (longest {c['poll_max_us']} us); "
              f"longest main-loop pass {c['loop_max_us']} us")
    if st.gaps:
        print("  zero-filled gaps (datagram from the stream start, length, ms): " + ", ".join(
            f"{k}+{n} ({k * 672 / 40e3:.2f} ms)" for k, n in st.gaps)
              + "  -> python pd_replay_test.py --host ... --start-seconds <emulator start> --zero "
              + ",".join(f"{k}:{n}" for k, n in st.gaps))
    if st.stalls:
        print("  " + stall_text(st))
    if st.det_status & 0x1F900:
        print(f"  receiver STATUS 0x{st.det_status:08x}: sticky error bits set (see pd_axi.v)")
    print(f"Receiver: {tally.triggers} triggers, {tally.timing_ok} Preamble B timings, control field "
          f"{tally.cf_ok}/{tally.cf} CRC ok, {npk} packets decoded ({full} with all blocks)")
    print(f"  transport blocks {ok}/{tot} CRC ok" + (f" (BLER {1 - ok / tot:.2e})" if tot else "") +
          (f", {tally.bits_ok / max(secs, 1e-9) / 1e6:.2f} Mbit/s of good transport-block data" if tot else ""))
    good = st.lost == 0 and st.ring_full == 0 and st.dma_errors == 0 and st.stalls == 0 and tot > 0 and ok == tot
    print("RESULT: " + ("OK" if good else "see above"))
    return 0 if good else 1


if __name__ == "__main__":
    sys.exit(main())
