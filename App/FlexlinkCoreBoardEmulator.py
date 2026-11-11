#!/usr/bin/env python3
"""FlexlinkCoreBoardEmulator.py -- a PC stand-in for the ESP32-S31 Function-CoreBoard-1 (ESP-SDR).

It answers the same HTTP control API as the CoreBoard's SoapyESPSDR streaming firmware and sends
the same ESR2 UDP datagrams, with the samples taken from a capture file (.cs8). Anything that works
with the real board (FlexlinkCaptureWithCoreBoard.py, the Zedboard firmware) works with this, and
because the samples are a known capture, the decoded results are known in advance.

    HTTP  POST http://<this PC>:<http-port>/api/rx   {"op": "status" | "configure" | "start" | "stop"}
          start {"port": P, ...config}: streams to the address that sent the request, UDP port P
    UDP   1376-byte datagrams: 32-byte header <4sHHIIQQ ("ESR2", version, bits 8, epoch, rate,
          sample position, time_us) + 672 complex int8 samples (I, Q interleaved)

Like the board: every start begins a new epoch at sample position 0, time_us counts from the
emulator's start, and the stream runs until stop (the file is looped). The datagram rate follows
the sample rate (59,524 datagrams/s at 40 Msps); --speed runs it slower or faster.

Usage
-----
    # the emulator (port 80 needs root; any other port is fine for the PC tools and the Zedboard)
    python App/FlexlinkCoreBoardEmulator.py                                   # App/CoreBoard/CoreBoard_Capture.cs8
    python App/FlexlinkCoreBoardEmulator.py --file App/CoreBoard/Segment_newfw_3820ms.cs8 --http-port 8080
    python App/FlexlinkCoreBoardEmulator.py --drop 1e-4 --drop-burst 3 --reorder 1e-4   # network faults
    python App/FlexlinkCoreBoardEmulator.py --speed 0.1                       # 10 x slower than real time

    # check it with the capture script, then decode as usual
    python App/FlexlinkCaptureWithCoreBoard.py --host 127.0.0.1:8080 --seconds 3
    python App/FlexlinkProcessCoreBoardCapturedWaveform.py

    # no HTTP: stream straight to a receiver (e.g. a Zedboard firmware without the HTTP start yet)
    python App/FlexlinkCoreBoardEmulator.py --stream-to 10.42.0.50:4000 --seconds 10

The file's sample rate comes from the .json of the same name (as the capture script writes it);
without one it is taken as 40 Msps, or give --rate (e.g. --rate 20000000 for a 20 Msps segment). A start with another rate is refused, as the samples cannot be re-made.
Configure / start settings (frequency, gain, ...) are checked like the board does and reported
in status, but do not change the samples.

Faults (for testing a receiver): --drop P loses a datagram with probability P (with --drop-burst N,
N in a row), --reorder P swaps a datagram with the next one, --duplicate P sends one twice. They
model the network, so the board counters in status stay 0, as they would on the real board; the
emulator's own counts are under "emulator" in status and in the per-second log.
"""
from __future__ import annotations

import argparse
import gc
import json
import socket
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

APP_DIR = Path(__file__).resolve().parent
DEFAULT_FILE = APP_DIR / "CoreBoard" / "CoreBoard_Capture.cs8"

# ESR2 wire format (as FlexlinkCaptureWithCoreBoard.py)
MAGIC = b"ESR2"
VERSION = 2
BITS = 8
HDR_DTYPE = np.dtype([("magic", "S4"), ("version", "<u2"), ("bits", "<u2"), ("epoch", "<u4"),
                      ("rate", "<u4"), ("sample", "<u8"), ("time_us", "<u8")])
HDR_LEN = HDR_DTYPE.itemsize                   # 32
SPP = 672                                      # complex samples per datagram
PKT_LEN = HDR_LEN + 2 * SPP                    # 1376
RATES = (4_000_000, 8_000_000, 16_000_000, 20_000_000, 40_000_000)
GAIN_MAX = 71
BATCH = 128                                    # datagrams built and paced together (~2.2 ms at 40 Msps)


# ---------- sample source ----------
class Source:
    """The capture file as a memory map of int8 I/Q, read in datagram-sized pieces, looped."""

    def __init__(self, path: Path, rate: int | None, start_s: float):
        if not path.is_file():
            raise SystemExit(f"Error: {path} not found (make one with FlexlinkCaptureWithCoreBoard.py, "
                             f"or give --file)")
        self.path = path
        meta = path.with_suffix(".json")              # the capture script's metadata, same name
        if rate is None:
            rate = 40_000_000
            if meta.is_file():
                try:
                    rate = int(json.loads(meta.read_text()).get("SampleRate", rate))
                except (OSError, ValueError):
                    pass
        if rate not in RATES:
            raise SystemExit(f"Error: sample rate {rate} is not one the board has ({RATES})")
        self.rate = rate
        self.iq = np.memmap(path, dtype=np.int8, mode="r")
        n = self.iq.size // 2
        if n < SPP * BATCH:
            raise SystemExit(f"Error: {path} is too short ({n} samples)")
        self.n_pkts = n // SPP                     # whole datagrams; the rest is not sent
        self.iq = self.iq[:self.n_pkts * 2 * SPP].reshape(self.n_pkts, 2 * SPP)
        self.first = int(start_s * rate) // SPP % self.n_pkts

    def rows(self, k: int, count: int) -> np.ndarray:
        """Payloads of datagrams k .. k+count-1 of the stream (k counted from the start), looped."""
        a = (self.first + k) % self.n_pkts
        b = a + count
        if b <= self.n_pkts:
            return self.iq[a:b]
        return np.concatenate([self.iq[a:], self.iq[:b - self.n_pkts]])


# ---------- the streamer ----------
class Streamer:
    def __init__(self, src: Source, speed: float, faults: dict, sndbuf_mb: int, seed: int, pace: int = 8):
        self.src, self.speed, self.faults = src, speed, faults
        self.pace = max(1, pace)                   # datagrams sent back to back, then wait
        self.rng = np.random.default_rng(seed)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, sndbuf_mb << 20)
        self.thread: threading.Thread | None = None
        self.stop_evt = threading.Event()
        self.lock = threading.Lock()
        self.epoch = 0
        self.boot = time.monotonic()
        self.dest: tuple[str, int] | None = None
        self.stats = self._zero()

    @staticmethod
    def _zero() -> dict:
        return {"datagrams": 0, "dropped": 0, "reordered": 0, "duplicated": 0, "send_errors": 0,
                "max_lag_ms": 0.0, "seconds": 0.0}

    @property
    def streaming(self) -> bool:
        return self.thread is not None and self.thread.is_alive()

    def start(self, dest: tuple[str, int], max_seconds: float = 0.0) -> int:
        self.stop()
        with self.lock:
            self.epoch += 1
            self.dest = dest
            self.stats = self._zero()
        self.stop_evt.clear()
        self.thread = threading.Thread(target=self._run, args=(self.epoch, dest, max_seconds), daemon=True)
        self.thread.start()
        return self.epoch

    def stop(self) -> None:
        if self.thread is not None:
            self.stop_evt.set()
            self.thread.join()
            self.thread = None

    def _run(self, epoch: int, dest: tuple[str, int], max_seconds: float) -> None:
        src, f, rng = self.src, self.faults, self.rng
        rate = src.rate
        pkt_s = SPP / rate / self.speed            # seconds per datagram on the wire
        hdr = np.zeros(BATCH, HDR_DTYPE)
        hdr["magic"], hdr["version"], hdr["bits"] = MAGIC, VERSION, BITS
        hdr["epoch"], hdr["rate"] = epoch, rate
        buf = np.empty((BATCH, PKT_LEN), np.uint8)
        hbytes = buf[:, :HDR_LEN]
        mv = memoryview(buf).cast("B")
        sendto = self.sock.sendto
        pace = self.pace
        t_us0 = int((time.monotonic() - self.boot) * 1e6)
        t0 = time.monotonic()
        k = 0                                      # next datagram of the stream
        burst_left = 0
        max_pkts = int(max_seconds * rate / SPP) if max_seconds > 0 else None
        gc_was = gc.isenabled()
        gc.disable()
        try:
            while not self.stop_evt.is_set():
                if max_pkts is not None and k >= max_pkts:
                    break
                pos = (k + np.arange(BATCH, dtype=np.uint64)) * SPP
                hdr["sample"] = pos
                hdr["time_us"] = t_us0 + (pos * 1_000_000 // rate)
                hbytes[:] = hdr.view(np.uint8).reshape(BATCH, HDR_LEN)
                buf[:, HDR_LEN:] = src.rows(k, BATCH).view(np.uint8)
                order = list(range(BATCH))
                send = [True] * BATCH
                dup = [False] * BATCH
                if f["drop"] > 0 or f["reorder"] > 0 or f["duplicate"] > 0:
                    for i in range(BATCH):
                        if burst_left > 0:
                            send[i] = False
                            burst_left -= 1
                        elif f["drop"] > 0 and rng.random() < f["drop"]:
                            send[i] = False
                            burst_left = f["drop_burst"] - 1
                    if f["reorder"] > 0:
                        for i in np.flatnonzero(rng.random(BATCH - 1) < f["reorder"]):
                            order[i], order[i + 1] = order[i + 1], order[i]
                    if f["duplicate"] > 0:
                        for i in np.flatnonzero(rng.random(BATCH) < f["duplicate"]):
                            dup[i] = True
                # pacing: datagram k is due at t0 + k * pkt_s, checked every self.pace datagrams
                # (before: once per 128-datagram batch, sent back to back at line rate - 1.5 ms
                # bursts at ~1.5 x the stream rate, and longer ones whenever this PC fell behind;
                # the real CoreBoard sends evenly). Short waits spin: sleep() is too coarse.
                lag = 0.0
                errors = sent = 0
                for j, i in enumerate(order):
                    if j % pace == 0:
                        due = t0 + (k + j) * pkt_s
                        now = time.monotonic()
                        lag = max(lag, now - due)
                        if due - now > 0.0005:
                            time.sleep(due - now - 0.0003)
                        while time.monotonic() < due:
                            pass
                    if not send[i]:
                        continue
                    piece = mv[i * PKT_LEN:(i + 1) * PKT_LEN]
                    for _ in range(2 if dup[i] else 1):
                        try:
                            sendto(piece, dest)
                            sent += 1
                        except OSError:            # e.g. ENOBUFS: the PC's send queue is full
                            errors += 1
                k += BATCH
                with self.lock:
                    s = self.stats
                    s["datagrams"] += sent
                    s["dropped"] += send.count(False)
                    s["duplicated"] += sum(dup)
                    s["reordered"] += sum(1 for i, j in enumerate(order) if i != j) // 2
                    s["send_errors"] += errors
                    s["max_lag_ms"] = max(s["max_lag_ms"], 1e3 * lag)
                    s["seconds"] = k * SPP / rate
        finally:
            if gc_was:
                gc.enable()

    def snapshot(self) -> dict:
        with self.lock:
            return dict(self.stats)


# ---------- the board state and HTTP API ----------
class Board:
    def __init__(self, streamer: Streamer, http_port: int):
        self.st = streamer
        self.http_port = http_port
        self.cfg = {"frequency": 2_440_000_000, "rate": streamer.src.rate, "agc": 0, "gain": 40,
                    "bandwidth": 0, "dc_correction": 1}

    def check(self, req: dict) -> dict:
        """Settings from a configure / start request, checked like the board; returns the new config."""
        cfg = dict(self.cfg)
        for key in ("frequency", "rate", "agc", "gain", "bandwidth", "dc_correction"):
            if key in req:
                try:
                    cfg[key] = int(req[key])
                except (TypeError, ValueError):
                    raise ValueError(f"{key} must be an integer") from None
        f = cfg["frequency"]
        if f % 1_000_000 or not 2_300_000_000 <= f <= 2_800_000_000:
            raise ValueError("frequency must be whole MHz in 2300..2800 MHz")
        if cfg["rate"] not in RATES:
            raise ValueError(f"rate must be one of {list(RATES)}")
        if cfg["rate"] != self.st.src.rate:
            raise ValueError(f"emulator: the file {self.st.src.path.name} is at {self.st.src.rate} sps, "
                             f"{cfg['rate']} cannot be streamed")
        if not 0 <= cfg["gain"] <= GAIN_MAX:
            raise ValueError(f"gain must be 0..{GAIN_MAX}")
        bw = cfg["bandwidth"]
        if bw and not 13_000_000 <= bw <= 54_000_000:
            raise ValueError("bandwidth must be 0 or 13..54 MHz")
        cfg["agc"] = 1 if cfg["agc"] else 0
        cfg["dc_correction"] = 1 if cfg["dc_correction"] else 0
        return cfg

    def status(self, host_ip: str) -> dict:
        c = self.cfg
        s = self.st.snapshot()
        return {"product": "ESP-SDR CoreBoard emulator (FlexlinkCoreBoardEmulator.py)", "protocol": VERSION,
                "ip": host_ip, "ethernet_mbps": 1000, "transport": "ethernet",
                "frequency": c["frequency"], "rate": c["rate"], "agc": c["agc"], "gain": c["gain"],
                "gain_max": GAIN_MAX, "bandwidth": c["bandwidth"],
                "effective_bandwidth": c["bandwidth"] or c["rate"], "dc_correction": c["dc_correction"],
                "streaming": self.st.streaming, "epoch": self.st.epoch,
                "destination": f"{self.st.dest[0]}:{self.st.dest[1]}" if self.st.dest else None,
                "capture_drops": 0, "transport_drops": 0, "capture_overruns": 0, "ring_drops": 0,
                "emulator": {"file": str(self.st.src.path), "file_seconds": self.st.src.n_pkts * SPP / c["rate"],
                             "speed": self.st.speed, **s}}

    def handle(self, req: dict, client_ip: str, host_ip: str) -> tuple[int, dict]:
        op = req.get("op")
        try:
            if op == "status":
                return 200, self.status(host_ip)
            if op == "configure":
                self.cfg = self.check(req)
                return 200, {"ok": True, **self.status(host_ip)}
            if op == "start":
                port = req.get("port")
                if not isinstance(port, int) or not 1 <= port <= 65535:
                    raise ValueError("start needs \"port\": the UDP port to stream to (1..65535)")
                self.cfg = self.check(req)
                epoch = self.st.start((client_ip, port))
                log(f"start: epoch {epoch} -> {client_ip}:{port}, {self.cfg['rate'] / 1e6:g} Msps")
                return 200, {"ok": True, "epoch": epoch, "port": port, "destination": client_ip,
                             "rate": self.cfg["rate"]}
            if op == "stop":
                was = self.st.streaming
                self.st.stop()
                if was:
                    s = self.st.snapshot()
                    log(f"stop: {s['datagrams']} datagrams, {s['seconds']:.2f} s of samples")
                return 200, {"ok": True}
            return 400, {"error": f"unknown op {op!r} (status, configure, start, stop)"}
        except ValueError as e:
            return 400, {"error": str(e)}


def make_handler(board: Board):
    class Handler(BaseHTTPRequestHandler):
        def _reply(self, code: int, body: dict) -> None:
            data = json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_POST(self):  # noqa: N802
            if self.path.rstrip("/") != "/api/rx":
                return self._reply(404, {"error": "not found (POST /api/rx)"})
            try:
                n = int(self.headers.get("Content-Length", 0))
                req = json.loads(self.rfile.read(n).decode() or "{}")
                if not isinstance(req, dict):
                    raise ValueError
            except (ValueError, UnicodeDecodeError):
                return self._reply(400, {"error": "body must be a JSON object"})
            code, body = board.handle(req, self.client_address[0], self.connection.getsockname()[0])
            self._reply(code, body)

        def do_GET(self):  # noqa: N802
            if self.path.rstrip("/") in ("", "/api/rx"):
                return self._reply(200, board.status(self.connection.getsockname()[0]))
            self._reply(404, {"error": "not found"})

        def log_message(self, fmt, *args):     # requests are logged by Board.handle
            pass

    return Handler


def log(msg: str) -> None:
    print(time.strftime("%H:%M:%S ") + msg, flush=True)


def report_loop(st: Streamer, stop: threading.Event) -> None:
    last = st.snapshot()
    t_last = time.monotonic()
    while not stop.wait(1.0):
        s = st.snapshot()
        now = time.monotonic()
        if st.streaming and s["datagrams"] >= last["datagrams"]:
            dt = now - t_last
            rate = (s["datagrams"] - last["datagrams"]) / dt
            mbit = rate * (PKT_LEN + 28) * 8 / 1e6
            extra = "".join(f", {k} {s[k]}" for k in ("dropped", "reordered", "duplicated", "send_errors") if s[k])
            log(f"  {s['seconds']:7.2f} s  {rate:8.0f} datagrams/s ({mbit:4.0f} Mbit/s), "
                f"lag max {s['max_lag_ms']:.1f} ms{extra}")
        last, t_last = s, now


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Emulate the ESP32-S31 CoreBoard (ESP-SDR) from a capture file.",
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--file", type=Path, default=DEFAULT_FILE, help="int8 I/Q capture (.cs8) to stream")
    ap.add_argument("--rate", type=int, choices=RATES, help="the file's sample rate (default: from its .json, else 40 Msps)")
    ap.add_argument("--start-seconds", type=float, default=0.0, help="begin this far into the file")
    ap.add_argument("--http-port", type=int, default=8080, help="HTTP control port (the board uses 80; needs root)")
    ap.add_argument("--bind", default="0.0.0.0", help="address to listen on")
    ap.add_argument("--stream-to", metavar="HOST:PORT", help="no HTTP: stream to HOST:PORT at once")
    ap.add_argument("--seconds", type=float, default=0.0, help="with --stream-to: stop after this many seconds (0 = until Ctrl-C)")
    ap.add_argument("--speed", type=float, default=1.0, help="datagram rate relative to real time (default 1)")
    ap.add_argument("--pace", type=int, default=8,
                    help="datagrams sent back to back between pacing checks (default 8, ~135 us at 40 Msps; "
                         "128 = the old behaviour)")
    ap.add_argument("--drop", type=float, default=0.0, help="probability of losing a datagram")
    ap.add_argument("--drop-burst", type=int, default=1, help="datagrams lost in a row per loss")
    ap.add_argument("--reorder", type=float, default=0.0, help="probability of swapping a datagram with the next")
    ap.add_argument("--duplicate", type=float, default=0.0, help="probability of sending a datagram twice")
    ap.add_argument("--seed", type=int, default=1, help="random seed of the faults")
    ap.add_argument("--sndbuf-mb", type=int, default=16, help="UDP send buffer asked for")
    a = ap.parse_args(argv)
    if a.speed <= 0:
        ap.error("--speed must be positive")
    if a.drop_burst < 1:
        ap.error("--drop-burst must be at least 1")

    src = Source(a.file, a.rate, a.start_seconds)
    faults = {"drop": a.drop, "drop_burst": a.drop_burst, "reorder": a.reorder, "duplicate": a.duplicate}
    st = Streamer(src, a.speed, faults, a.sndbuf_mb, a.seed, a.pace)
    rate_note = f"{src.rate / 1e6:g} Msps, {src.n_pkts * SPP / src.rate:.2f} s (looped)"
    fault_note = ", ".join(f"{k} {v:g}" for k, v in faults.items() if v and k != "drop_burst")
    if a.drop and a.drop_burst > 1:
        fault_note += f" (bursts of {a.drop_burst})"
    log(f"Source {src.path} : {rate_note}" + (f", speed x{a.speed:g}" if a.speed != 1 else "")
        + (f", faults: {fault_note}" if fault_note else ""))

    stop = threading.Event()
    rep = threading.Thread(target=report_loop, args=(st, stop), daemon=True)
    rep.start()
    try:
        if a.stream_to:
            host, _, port = a.stream_to.rpartition(":")
            if not host or not port.isdigit():
                ap.error("--stream-to needs HOST:PORT")
            dest = (socket.gethostbyname(host), int(port))
            epoch = st.start(dest, a.seconds)
            log(f"Streaming to {dest[0]}:{dest[1]} (epoch {epoch})" + (f" for {a.seconds:g} s" if a.seconds else
                                                                        ", Ctrl-C to stop"))
            while st.streaming:
                time.sleep(0.2)
        else:
            board = Board(st, a.http_port)
            try:
                srv = ThreadingHTTPServer((a.bind, a.http_port), make_handler(board))
            except PermissionError:
                print(f"Error: port {a.http_port} needs root; use --http-port 8080 (or sudo)", file=sys.stderr)
                return 3
            except OSError as e:
                print(f"Error: cannot listen on {a.bind}:{a.http_port} ({e})", file=sys.stderr)
                return 3
            log(f"Emulated CoreBoard listening on http://{a.bind}:{a.http_port}/api/rx  (Ctrl-C to quit)")
            srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        st.stop()
        stop.set()
        s = st.snapshot()
        if s["datagrams"]:
            log(f"Last stream: {s['datagrams']} datagrams, {s['seconds']:.2f} s of samples, "
                f"lag max {s['max_lag_ms']:.1f} ms"
                + "".join(f", {k} {s[k]}" for k in ("dropped", "reordered", "duplicated", "send_errors") if s[k]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
