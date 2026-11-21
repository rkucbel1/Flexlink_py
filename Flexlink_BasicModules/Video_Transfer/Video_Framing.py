#!/usr/bin/env python3
"""
Video_Framing.py  (lives in Flexlink_py/Flexlink_BasicModules/Video_Transfer)

Carries a live video stream (MPEG-TS bytes from the encoder) in the payload transport blocks,
and puts the byte stream back together at the receiver. Plain NumPy and the standard library,
so it can be tested on its own (python Video_Framing.py --selftest).

Transport blocks
----------------
Every transport block (TransportBlockSize bits; the CRC is added by the transmitter) is

    magic    8 bits   0xF2 (video; File_Framing.py uses 0xF1 for files)
    seq     16 bits   block counter, +1 for every block sent, wraps at 65536
    nbytes   8 bits   stream bytes in this block, 0 .. PayloadBytes (0: a filler block)
    data     8 * PayloadBytes bits, PayloadBytes = (TransportBlockSize - 32) // 8;
                      the first nbytes bytes are stream bytes, the rest zeros
    zeros    the rest (TransportBlockSize - 32) % 8 bits

All fields MSB first, as in File_Framing.py. The transmitter fills each packet from whatever
the encoder has delivered when the packet is due, so a block can be part full or a filler;
the link keeps its packet rate and the latency stays at about one packet.

The receiver appends the bytes of the blocks in seq order. A block whose CRC failed, or a jump
in seq (a block or a whole packet that never arrived), is a gap: the bytes on either side must
not be joined, so the unpacker reports it and TsResync drops the broken TS packet and
re-aligns to the next 188-byte TS packet. The decoder then sees a few missing TS packets and
recovers with the encoder's intra refresh.
"""
from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass, field

import numpy as np

BLOCK_MAGIC = 0xF2
HEADER_BITS = 32          # 8 + 16 + 8
TS_PACKET = 188
TS_SYNC = 0x47
SEQ_MOD = 1 << 16


def payload_bytes(tb_bits: int) -> int:
    """Stream bytes one transport block of tb_bits carries."""
    n = (int(tb_bits) - HEADER_BITS) // 8
    if n < 1:
        raise ValueError(f"TransportBlockSize {tb_bits} is too small for the {HEADER_BITS}-bit block header")
    return min(n, 255)    # nbytes is 8 bits; the largest TransportBlockSize (1596) gives 195


def capacity_bps(tb_bits: int, blocks_per_packet: int, packet_seconds: float) -> float:
    """Stream bits per second the link carries when every block is full."""
    return blocks_per_packet * payload_bytes(tb_bits) * 8 / packet_seconds


# --------------------------------------------------------------------------- #
# Thread-safe byte queue between the network receiver and the packer
# --------------------------------------------------------------------------- #
class ByteFifo:
    """Chunks of bytes in arrival order (e.g. one UDP datagram = 7 TS packets per chunk)."""

    def __init__(self):
        self._q: deque[bytes] = deque()
        self._head = 0                  # bytes of _q[0] already taken
        self._size = 0
        self._lock = threading.Lock()
        self.dropped_chunks = 0
        self.dropped_bytes = 0

    def put(self, data: bytes) -> None:
        if data:
            with self._lock:
                self._q.append(bytes(data))
                self._size += len(data)

    def size(self) -> int:
        with self._lock:
            return self._size

    def take(self, n: int) -> bytes:
        """Up to n bytes from the front."""
        out = bytearray()
        with self._lock:
            while n > 0 and self._q:
                c = self._q[0]
                k = min(n, len(c) - self._head)
                out += c[self._head:self._head + k]
                self._head += k
                n -= k
                if self._head == len(c):
                    self._q.popleft()
                    self._head = 0
            self._size -= len(out)
        return bytes(out)

    def trim(self, max_bytes: int) -> int:
        """Drop whole chunks from the front (never the one being read) until at most max_bytes
        are queued. Keeps TS alignment when every chunk is whole TS packets. Returns the chunks
        dropped."""
        dropped = 0
        with self._lock:
            while self._size > max_bytes and len(self._q) > 1 and self._head == 0:
                c = self._q.popleft()
                self._size -= len(c)
                self.dropped_bytes += len(c)
                dropped += 1
            self.dropped_chunks += dropped
        return dropped


# --------------------------------------------------------------------------- #
# Transmitter
# --------------------------------------------------------------------------- #
def pack_block(seq: int, data: bytes, tb_bits: int) -> np.ndarray:
    n = payload_bytes(tb_bits)
    if len(data) > n:
        raise ValueError(f"{len(data)} bytes, a block of {tb_bits} bits holds {n}")
    raw = bytes([BLOCK_MAGIC, (seq >> 8) & 0xFF, seq & 0xFF, len(data)]) + data + bytes(n - len(data))
    bits = np.zeros(tb_bits, dtype=np.uint8)
    bits[:8 * len(raw)] = np.unpackbits(np.frombuffer(raw, dtype=np.uint8))
    return bits


class VideoPacker:
    """Fills one packet of transport blocks at a time from a ByteFifo."""

    def __init__(self, tb_bits: int, blocks_per_packet: int):
        self.tb_bits = int(tb_bits)
        self.ndb = int(blocks_per_packet)
        self.chunk = payload_bytes(self.tb_bits)
        self.seq = 0
        self.packets = 0
        self.blocks = 0
        self.stream_bytes = 0          # stream bytes sent
        self.filler_blocks = 0

    def next_packet(self, fifo: ByteFifo) -> np.ndarray:
        """(blocks_per_packet, TransportBlockSize) bits; takes what the fifo holds, up to a full packet."""
        out = np.zeros((self.ndb, self.tb_bits), dtype=np.uint8)
        for k in range(self.ndb):
            data = fifo.take(self.chunk)
            out[k] = pack_block(self.seq, data, self.tb_bits)
            self.seq = (self.seq + 1) % SEQ_MOD
            self.stream_bytes += len(data)
            self.filler_blocks += not data
        self.packets += 1
        self.blocks += self.ndb
        return out

    def fill_ratio(self) -> float:
        return self.stream_bytes / max(1, self.blocks * self.chunk)


# --------------------------------------------------------------------------- #
# Receiver
# --------------------------------------------------------------------------- #
@dataclass
class UnpackReport:
    blocks: int = 0            # blocks offered
    crc_fail: int = 0          # dropped: CRC failed
    foreign: int = 0           # CRC passed, but not a video block
    filler: int = 0            # video blocks without stream bytes
    lost: int = 0              # blocks missing from the seq count (never arrived, or CRC failed)
    gaps: int = 0              # places where the stream is broken
    stream_bytes: int = 0


@dataclass
class VideoUnpacker:
    """Turns received transport blocks back into the byte stream, marking the gaps."""
    expect: int | None = None          # seq of the next block
    gap_open: bool = False             # bytes lost since the last piece returned
    total: UnpackReport = field(default_factory=UnpackReport)

    def missing_packet(self) -> None:
        """Tell the unpacker that a packet was detected but not decoded (its blocks are lost)."""
        self.gap_open = True

    def feed(self, bits: np.ndarray, crc_pass) -> list[tuple[bool, bytes]]:
        """bits: one packet (blocks x TransportBlockSize, or flat), crc_pass: one flag per block.
        Returns [(gap_before, bytes), ...] in stream order: gap_before is True when bytes were
        lost just before this piece."""
        crc_pass = np.asarray(crc_pass).ravel().astype(bool)
        n = crc_pass.size
        bits = np.asarray(bits, dtype=np.uint8).reshape(n, -1)
        rep = UnpackReport(blocks=n)
        out: list[tuple[bool, bytes]] = []
        gap = self.gap_open             # a gap at the end of the last packet carries over
        for k in range(n):
            if not crc_pass[k]:
                rep.crc_fail += 1
                gap = True
                continue
            raw = np.packbits(bits[k]).tobytes()
            if raw[0] != BLOCK_MAGIC:
                rep.foreign += 1
                continue
            seq = (raw[1] << 8) | raw[2]
            nb = raw[3]
            if nb > len(raw) - 4:
                rep.foreign += 1
                continue
            if self.expect is not None and seq != self.expect:
                missed = (seq - self.expect) % SEQ_MOD
                rep.lost += missed
                gap = True
            self.expect = (seq + 1) % SEQ_MOD
            if nb == 0:
                rep.filler += 1
                continue
            out.append((gap, raw[4:4 + nb]))
            if gap:
                rep.gaps += 1
            gap = False
            rep.stream_bytes += nb
        self.gap_open = gap
        for name in ("blocks", "crc_fail", "foreign", "filler", "lost", "gaps", "stream_bytes"):
            setattr(self.total, name, getattr(self.total, name) + getattr(rep, name))
        return out


class TsResync:
    """Emits whole 188-byte TS packets. After a gap it drops the broken packet and waits for a
    sync byte that repeats 188 bytes later."""

    def __init__(self):
        self.buf = bytearray()
        self.hunting = True
        self.resyncs = 0
        self.dropped_bytes = 0
        self.ts_packets = 0

    def push(self, data: bytes, gap_before: bool = False) -> bytes:
        if gap_before:
            self.dropped_bytes += len(self.buf)
            self.buf.clear()
            self.hunting = True
        self.buf += data
        out = bytearray()
        while True:
            if self.hunting:
                i = self._find_sync()
                if i is None:
                    keep = max(0, len(self.buf) - TS_PACKET)      # keep enough to finish the search
                    self.dropped_bytes += keep
                    del self.buf[:keep]
                    break
                self.dropped_bytes += i
                del self.buf[:i]
                self.hunting = False
                self.resyncs += 1
            if len(self.buf) < TS_PACKET:
                break
            if self.buf[0] != TS_SYNC:
                self.hunting = True
                continue
            out += self.buf[:TS_PACKET]
            del self.buf[:TS_PACKET]
            self.ts_packets += 1
        return bytes(out)

    def _find_sync(self) -> int | None:
        b = self.buf
        for i in range(len(b)):
            if b[i] != TS_SYNC:
                continue
            if i + TS_PACKET < len(b):
                if b[i + TS_PACKET] == TS_SYNC:
                    return i
            else:
                return None            # not enough bytes yet to confirm this one
        return None


# --------------------------------------------------------------------------- #
# Video out: ffplay and / or a file
# --------------------------------------------------------------------------- #
class TsOutput:
    """Plays whole TS packets with ffplay (low-latency flags) and / or appends them to a file."""

    def __init__(self, play: bool = True, save_path=None, title: str = "Flexlink video"):
        import shutil
        import subprocess
        self._sp = subprocess
        self.file = None
        self.player = None
        self.note = ""
        self.bytes = 0
        if save_path is not None:
            from pathlib import Path
            Path(save_path).parent.mkdir(parents=True, exist_ok=True)
            self.file = open(save_path, "wb")
        if play:
            exe = shutil.which("ffplay")
            if exe:
                self.player = subprocess.Popen(
                    [exe, "-hide_banner", "-loglevel", "error", "-fflags", "nobuffer", "-flags", "low_delay",
                     "-framedrop", "-probesize", "32768", "-analyzeduration", "0",
                     "-window_title", title, "-f", "mpegts", "-i", "-"],
                    stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            else:
                self.note = "ffplay not found (sudo apt install ffmpeg)"

    def write(self, ts: bytes) -> None:
        if not ts:
            return
        self.bytes += len(ts)
        if self.file is not None:
            self.file.write(ts)
        if self.player is not None:
            try:
                self.player.stdin.write(ts)
                self.player.stdin.flush()
            except (BrokenPipeError, OSError):
                self.player = None
                self.note = "ffplay window closed"

    def close(self) -> None:
        if self.file is not None:
            self.file.close()
        if self.player is not None:
            try:
                self.player.stdin.close()
            except OSError:
                pass


# --------------------------------------------------------------------------- #
# Self-test
# --------------------------------------------------------------------------- #
def _fake_ts(n_packets: int, rng) -> bytes:
    pk = rng.integers(0, 256, size=(n_packets, TS_PACKET), dtype=np.uint8)
    pk[pk == TS_SYNC] = 0x46           # sync bytes only where a packet starts
    pk[:, 0] = TS_SYNC
    pk[:, 1] = np.arange(n_packets) >> 8
    pk[:, 2] = np.arange(n_packets) & 0xFF
    return pk.tobytes()


def selftest() -> None:
    rng = np.random.default_rng(1)
    for tb_bits, ndb in ((1596, 27), (1056, 5), (300, 2)):
        ts = _fake_ts(400, rng)
        fifo = ByteFifo()
        for k in range(0, len(ts), 7 * TS_PACKET):
            fifo.put(ts[k:k + 7 * TS_PACKET])
        pk = VideoPacker(tb_bits, ndb)
        packets = []
        while fifo.size():
            packets.append(pk.next_packet(fifo))
        packets.append(pk.next_packet(fifo))          # a filler packet
        # 1. clean: every byte back
        un, rs = VideoUnpacker(), TsResync()
        got = b"".join(rs.push(b, g) for p in packets for g, b in un.feed(p, np.ones(ndb, bool)))
        assert got == ts, "clean round trip differs"
        assert un.total.lost == 0 and un.total.gaps == 0
        # 2. losses: CRC failures and a missing packet; every TS packet out is an original one
        un, rs = VideoUnpacker(), TsResync()
        orig = {ts[i:i + TS_PACKET] for i in range(0, len(ts), TS_PACKET)}
        out = bytearray()
        for i, p in enumerate(packets):
            if i == 3:
                continue                                # whole packet missing
            crc = rng.random(ndb) > 0.05
            for g, b in un.feed(p, crc):
                out += rs.push(b, g)
        outs = [bytes(out[i:i + TS_PACKET]) for i in range(0, len(out), TS_PACKET)]
        assert all(o in orig for o in outs), "a broken TS packet got through"
        assert un.total.lost > 0 and un.total.gaps > 0
        print(f"TB {tb_bits} bits x {ndb}: {payload_bytes(tb_bits)} bytes per block; clean ok; "
              f"with losses {len(outs)} of {len(orig)} TS packets kept, {un.total.lost} blocks lost, "
              f"{rs.resyncs} resyncs, fill {pk.fill_ratio():.2f}")
    print("selftest passed")


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        selftest()
    else:
        print(__doc__)
