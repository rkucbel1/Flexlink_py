#!/usr/bin/env python3
"""
Mavlink_Framing.py  (lives in Flexlink_py/App/Mavlink)

Carries MAVLink frames in the payload transport blocks next to the video stream, and gets them
back at the receiver. Also the MAVLink sources of the transmitter (a .tlog file, UDP, a serial
port) and the outputs of the receiver (UDP to a ground station, a .tlog file).

Transport blocks
----------------
A MAVLink block has the same layout as a video block (Video_Framing.py), with its own magic and
its own block counter:

    magic    8 bits   0xF3 (MAVLink; 0xF2 is video, 0xF1 files)
    seq     16 bits   MAVLink block counter, +1 for every MAVLink block sent, wraps at 65536
    nbytes   8 bits   MAVLink bytes in this block, 1 .. PayloadBytes
    data     8 * PayloadBytes bits; the first nbytes bytes are MAVLink bytes, the rest zeros
    zeros    the rest (TransportBlockSize - 32) % 8 bits

A block holds whole frames, as many as fit; only a frame longer than a block's payload is split
over consecutive blocks. A lost block therefore loses only the frames in it. The video receiver
(VideoUnpacker) skips the 0xF3 blocks as "non-video blocks" and its video block counter is not
disturbed, so a receiver without MAVLink support still plays the video.

Sharing a packet
----------------
MAVLink goes first: each packet carries up to MaxBlocksPerPacket MAVLink blocks (only as many as
there are frames waiting), the rest of the packet is video. Telemetry is a few kbit/s, so it
waits at most one packet and takes little from the video. Frames that waited longer than
MaxAge are dropped: an old position is of no use.

The receiver appends the bytes of the MAVLink blocks in seq order and splits them into frames
(Mavlink_Lite.MavlinkParser). A jump in seq (a MAVLink block lost) drops a partly received frame.

Self-test: python Mavlink_Framing.py --selftest
"""
from __future__ import annotations

import socket
import struct
import sys
import threading
import time
from collections import Counter, deque
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import Mavlink_Lite as ML  # noqa: E402

_VIDEO_TRANSFER = HERE.parents[1] / "Flexlink_BasicModules" / "Video_Transfer"
if str(_VIDEO_TRANSFER) not in sys.path:
    sys.path.insert(0, str(_VIDEO_TRANSFER))
import Video_Framing as VF  # noqa: E402

BLOCK_MAGIC = 0xF3
HEADER_BITS = 32
SEQ_MOD = 1 << 16


def payload_bytes(tb_bits: int) -> int:
    """MAVLink bytes one transport block of tb_bits carries (as for video)."""
    return VF.payload_bytes(tb_bits)


def pack_block(seq: int, data: bytes, tb_bits: int) -> np.ndarray:
    n = payload_bytes(tb_bits)
    if not 0 < len(data) <= n:
        raise ValueError(f"{len(data)} bytes, a MAVLink block of {tb_bits} bits holds 1 to {n}")
    raw = bytes([BLOCK_MAGIC, (seq >> 8) & 0xFF, seq & 0xFF, len(data)]) + data + bytes(n - len(data))
    bits = np.zeros(tb_bits, dtype=np.uint8)
    bits[:8 * len(raw)] = np.unpackbits(np.frombuffer(raw, dtype=np.uint8))
    return bits


# --------------------------------------------------------------------------- #
# Transmitter: frame queue and the packet scheduler
# --------------------------------------------------------------------------- #
class FrameQueue:
    """MAVLink frames waiting for the link (thread safe). A frame being sent in pieces is never
    dropped."""

    def __init__(self, max_age_s: float = 0.5, max_frames: int = 2000):
        self._q: deque[list] = deque()          # [arrival time, frame bytes, bytes already sent]
        self._lock = threading.Lock()
        self.max_age = float(max_age_s)
        self.max_frames = int(max_frames)
        self.frames_in = 0
        self.bytes_in = 0
        self.frames_out = 0
        self.expired = 0

    def put(self, raw: bytes, t: float | None = None) -> None:
        with self._lock:
            self._q.append([time.monotonic() if t is None else t, bytes(raw), 0])
            self.frames_in += 1
            self.bytes_in += len(raw)
            while len(self._q) > self.max_frames and self._q[0][2] == 0:
                self._q.popleft()
                self.expired += 1

    def __len__(self) -> int:
        with self._lock:
            return len(self._q)

    def expire(self, now: float | None = None) -> int:
        now = time.monotonic() if now is None else now
        n = 0
        with self._lock:
            while self._q and self._q[0][2] == 0 and now - self._q[0][0] > self.max_age:
                self._q.popleft()
                n += 1
            self.expired += n
        return n

    def take_block(self, space: int) -> bytes:
        """Bytes for one block of `space` bytes: whole frames that fit, or, when the next frame
        alone is longer than a block, the next piece of it."""
        out = bytearray()
        with self._lock:
            while self._q:
                item = self._q[0]
                rest = len(item[1]) - item[2]
                if rest <= space - len(out):
                    out += item[1][item[2]:]
                    self._q.popleft()
                    self.frames_out += 1
                elif not out:                          # longer than the whole block: one piece
                    out += item[1][item[2]:item[2] + space]
                    item[2] += space
                    break
                else:
                    break
        return bytes(out)


class LinkScheduler:
    """Fills one packet at a time: MAVLink blocks first (up to max_mav_blocks), then video blocks
    from a VF.ByteFifo (or video filler blocks when there is no video). The video blocks are the
    same as VF.VideoPacker's."""

    def __init__(self, tb_bits: int, blocks_per_packet: int, max_mav_blocks: int = 2):
        self.tb_bits = int(tb_bits)
        self.ndb = int(blocks_per_packet)
        self.max_mav = max(0, min(int(max_mav_blocks), self.ndb))
        self.chunk = payload_bytes(self.tb_bits)
        self.vseq = 0
        self.mseq = 0
        self.packets = 0
        self.blocks = 0
        self.video_blocks = 0
        self.video_bytes = 0
        self.mav_blocks = 0
        self.mav_bytes = 0
        self.filler_blocks = 0

    def next_packet(self, video: VF.ByteFifo | None, mav: FrameQueue | None,
                    now: float | None = None) -> np.ndarray:
        out = np.zeros((self.ndb, self.tb_bits), dtype=np.uint8)
        k = 0
        if mav is not None:
            mav.expire(now)
            while k < self.max_mav:
                data = mav.take_block(self.chunk)
                if not data:
                    break
                out[k] = pack_block(self.mseq, data, self.tb_bits)
                self.mseq = (self.mseq + 1) % SEQ_MOD
                self.mav_blocks += 1
                self.mav_bytes += len(data)
                k += 1
        while k < self.ndb:
            data = video.take(self.chunk) if video is not None else b""
            out[k] = VF.pack_block(self.vseq, data, self.tb_bits)
            self.vseq = (self.vseq + 1) % SEQ_MOD
            if data:
                self.video_blocks += 1
                self.video_bytes += len(data)
            else:
                self.filler_blocks += 1
            k += 1
        self.packets += 1
        self.blocks += self.ndb
        return out

    def fill_ratio(self) -> float:
        """Share of the payload bytes carrying video or MAVLink."""
        return (self.video_bytes + self.mav_bytes) / max(1, self.blocks * self.chunk)


# --------------------------------------------------------------------------- #
# Receiver
# --------------------------------------------------------------------------- #
class MavlinkUnpacker:
    """Takes the MAVLink blocks out of received packets and returns the frames."""

    def __init__(self):
        self.expect: int | None = None
        self.parser = ML.MavlinkParser()
        self.blocks = 0            # MAVLink blocks received (CRC passed)
        self.lost = 0              # MAVLink blocks missing from the seq count
        self.frames = 0

    def feed(self, bits: np.ndarray, crc_pass) -> list[ML.Frame]:
        crc_pass = np.asarray(crc_pass).ravel().astype(bool)
        bits = np.asarray(bits, dtype=np.uint8).reshape(crc_pass.size, -1)
        out: list[ML.Frame] = []
        for k in range(crc_pass.size):
            if not crc_pass[k]:
                continue                       # video or MAVLink: a MAVLink loss shows in seq
            raw = np.packbits(bits[k]).tobytes()
            if raw[0] != BLOCK_MAGIC:
                continue
            seq = (raw[1] << 8) | raw[2]
            nb = raw[3]
            if not 0 < nb <= len(raw) - 4:
                continue
            if self.expect is not None and seq != self.expect:
                self.lost += (seq - self.expect) % SEQ_MOD
                self.parser.reset()           # a frame cut by the loss must not be joined
            self.expect = (seq + 1) % SEQ_MOD
            self.blocks += 1
            frames = self.parser.feed(raw[4:4 + nb])
            self.frames += len(frames)
            out.extend(frames)
        return out


# --------------------------------------------------------------------------- #
# Telemetry summary (for the status lines)
# --------------------------------------------------------------------------- #
class TelemetrySummary:
    def __init__(self):
        self.counts: Counter = Counter()
        self.last: dict[str, dict] = {}
        self.text = ""

    def add(self, f: ML.Frame) -> None:
        self.counts[f.name] += 1
        d = f.decode()
        if d is not None:
            self.last[f.name] = d
            if f.name == "STATUSTEXT":
                self.text = f"{ML.MAV_SEVERITY.get(d['severity'], d['severity'])}: {d['text']}"

    def line(self) -> str:
        parts = []
        hb = self.last.get("HEARTBEAT")
        if hb:
            armed = "ARMED" if hb["base_mode"] & ML.MAV_MODE_FLAG_SAFETY_ARMED else "disarmed"
            mode = ML.COPTER_MODES.get(hb["custom_mode"], str(hb["custom_mode"]))
            parts.append(f"{mode} {armed}")
        gp = self.last.get("GLOBAL_POSITION_INT")
        if gp:
            parts.append(f"{gp['lat'] / 1e7:.6f}, {gp['lon'] / 1e7:.6f}, {gp['relative_alt'] / 1e3:5.1f} m, "
                         f"hdg {gp['hdg'] / 100:5.1f}")
        at = self.last.get("ATTITUDE")
        if at:
            parts.append(f"roll {np.degrees(at['roll']):5.1f} pitch {np.degrees(at['pitch']):5.1f}")
        ss = self.last.get("SYS_STATUS")
        if ss:
            parts.append(f"batt {ss['voltage_battery'] / 1e3:.2f} V {ss['battery_remaining']} %")
        if self.text:
            parts.append(f'"{self.text}"')
        return " | ".join(parts) if parts else "(no known messages yet)"


# --------------------------------------------------------------------------- #
# Sources (transmitter side): each puts frames into a FrameQueue from its own thread
# --------------------------------------------------------------------------- #
class _Source:
    kind = "source"

    def __init__(self, queue: FrameQueue):
        self.queue = queue
        self.parser = ML.MavlinkParser()
        self.frames = 0
        self.bytes = 0
        self.first: float | None = None
        self.error: str | None = None
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._guard, daemon=True)

    def start(self) -> "_Source":
        self.thread.start()
        return self

    def _guard(self) -> None:
        try:
            self._run()
        except Exception as e:  # noqa: BLE001  (reported by the worker)
            self.error = f"{type(e).__name__}: {e}"

    def _put(self, raw: bytes) -> None:
        if self.first is None:
            self.first = time.monotonic()
        self.queue.put(raw)
        self.frames += 1
        self.bytes += len(raw)

    def _feed(self, data: bytes) -> None:
        for f in self.parser.feed(data):
            self._put(f.raw)

    def _run(self) -> None:
        raise NotImplementedError

    def close(self) -> None:
        self._stop.set()
        if self.thread.is_alive():
            self.thread.join(timeout=1)


class TlogSource(_Source):
    """Replays a .tlog at its recorded pace, looped."""
    kind = "tlog"

    def __init__(self, path, queue: FrameQueue, loop: bool = True):
        super().__init__(queue)
        self.path = Path(path)
        self.records = ML.read_tlog(self.path)
        if not self.records:
            raise ValueError(f"no MAVLink frames in {self.path}")
        self.loop = loop
        self.loops = 0

    def _run(self) -> None:
        t_first = self.records[0][0]
        span = (self.records[-1][0] - t_first) / 1e6
        gap = max(span / max(1, len(self.records) - 1), 0.01)    # between the last and the first
        t0 = time.monotonic()
        while not self._stop.is_set():
            for t_us, raw in self.records:
                due = t0 + (t_us - t_first) / 1e6
                while not self._stop.is_set():
                    dt = due - time.monotonic()
                    if dt <= 0:
                        break
                    self._stop.wait(min(dt, 0.1))
                if self._stop.is_set():
                    return
                self._put(raw)
            if not self.loop:
                return
            self.loops += 1
            t0 += span + gap


class UdpSource(_Source):
    """Listens for MAVLink datagrams (e.g. mavlink-router, SITL, a companion computer)."""
    kind = "udp"

    def __init__(self, port: int, queue: FrameQueue):
        super().__init__(queue)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(("0.0.0.0", int(port)))
        except OSError as e:
            self.sock.close()
            raise RuntimeError(f"cannot listen on UDP port {port}: {e}") from e
        self.sock.settimeout(0.2)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                d = self.sock.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            self._feed(d)

    def close(self) -> None:
        super().close()
        self.sock.close()


class SerialSource(_Source):
    """Reads MAVLink from a serial port (a flight controller's telemetry port). Needs pyserial."""
    kind = "serial"

    def __init__(self, port: str, baud: int, queue: FrameQueue):
        super().__init__(queue)
        try:
            import serial  # noqa: PLC0415
        except ImportError as e:
            raise RuntimeError("the serial source needs pyserial: pip install pyserial") from e
        self.ser = serial.Serial(port, int(baud), timeout=0.1)

    def _run(self) -> None:
        while not self._stop.is_set():
            d = self.ser.read(4096)
            if d:
                self._feed(d)

    def close(self) -> None:
        super().close()
        self.ser.close()


def tlog_rate(path) -> tuple[float, float, float]:
    """(frames per second, bytes per second, seconds) of a .tlog."""
    rec = ML.read_tlog(path)
    if len(rec) < 2:
        return 0.0, 0.0, 0.0
    span = (rec[-1][0] - rec[0][0]) / 1e6
    total = sum(len(r) for _, r in rec)
    return len(rec) / span, total / span, span


# --------------------------------------------------------------------------- #
# Outputs (receiver side)
# --------------------------------------------------------------------------- #
class UdpSink:
    """Sends every frame as one datagram, e.g. to QGroundControl / Mission Planner (UDP 14550)."""

    def __init__(self, host: str, port: int):
        self.addr = (host, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sent = 0
        self.errors = 0

    def send(self, raw: bytes) -> None:
        try:
            self.sock.sendto(raw, self.addr)
            self.sent += 1
        except OSError:
            self.errors += 1

    def close(self) -> None:
        self.sock.close()


class TlogSink:
    def __init__(self, path):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.f = open(path, "wb")
        self.n = 0

    def write(self, raw: bytes) -> None:
        self.f.write(struct.pack(">Q", int(time.time() * 1e6)) + raw)
        self.n += 1

    def close(self) -> None:
        self.f.close()


def parse_host_port(text: str, default_port: int = 14550) -> tuple[str, int]:
    text = text.strip()
    host, sep, port = text.rpartition(":")
    if not sep:
        return text, default_port
    p = int(port)
    if not 1 <= p <= 65535:
        raise ValueError(f"port {p} out of range")
    return host or "127.0.0.1", p


# --------------------------------------------------------------------------- #
# Self-test
# --------------------------------------------------------------------------- #
def selftest() -> None:
    rng = np.random.default_rng(3)
    enc = ML.MavlinkEncoder()
    frames = []
    for i in range(300):
        frames.append(enc.frame("ATTITUDE", time_boot_ms=i, roll=0.01 * i, pitch=0.2, yaw=1.0))
        if i % 10 == 0:
            frames.append(enc.frame("HEARTBEAT", type=2, autopilot=3, base_mode=217, custom_mode=3,
                                    system_status=4, mavlink_version=3))
            frames.append(enc.frame("STATUSTEXT", severity=6, text=f"message {i} " + "x" * 30))
    ts = VF._fake_ts(300, rng)
    for tb_bits, ndb, max_mav in ((1056, 17, 2), (300, 4, 1), (1596, 27, 3)):
        # 1. clean: every frame and every video byte back, video receiver unaffected
        q = FrameQueue(max_age_s=1e9)
        for fr in frames:
            q.put(fr, t=0.0)
        fifo = VF.ByteFifo()
        for k in range(0, len(ts), 7 * VF.TS_PACKET):
            fifo.put(ts[k:k + 7 * VF.TS_PACKET])
        sch = LinkScheduler(tb_bits, ndb, max_mav)
        packets = []
        while len(q) or fifo.size():
            packets.append(sch.next_packet(fifo, q, now=0.0))
        packets.append(sch.next_packet(fifo, q, now=0.0))
        mu, vu, rs = MavlinkUnpacker(), VF.VideoUnpacker(), VF.TsResync()
        got, vid = [], bytearray()
        for p in packets:
            ok = np.ones(ndb, bool)
            got += [f.raw for f in mu.feed(p, ok)]
            for g, b in vu.feed(p, ok):
                vid += rs.push(b, g)
        assert got == frames, f"{tb_bits}: frames differ"
        assert bytes(vid) == ts, f"{tb_bits}: video differs"
        assert vu.total.gaps == 0 and vu.total.lost == 0 and vu.total.foreign == sch.mav_blocks
        assert mu.parser.bad_crc == 0
        # 2. losses: every frame out is an original one
        mu = MavlinkUnpacker()
        orig = set(frames)
        out = []
        for p in packets:
            out += [f.raw for f in mu.feed(p, rng.random(ndb) > 0.1)]
        assert out and all(o in orig for o in out), "a broken frame got through"
        print(f"TB {tb_bits} bits x {ndb}, up to {max_mav} MAVLink blocks: {len(frames)} frames in "
              f"{sch.mav_blocks} blocks; clean ok; with 10 % block loss {len(out)} frames kept, "
              f"{mu.lost} MAVLink blocks lost, {mu.parser.bad_crc} bad CRCs")
    # 3. expiry
    q = FrameQueue(max_age_s=0.5)
    q.put(frames[0], t=0.0)
    q.put(frames[1], t=1.0)
    assert q.expire(now=1.2) == 1 and len(q) == 1
    print("selftest passed")


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        selftest()
    else:
        print(__doc__)
