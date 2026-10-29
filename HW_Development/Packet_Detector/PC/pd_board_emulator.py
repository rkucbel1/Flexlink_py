#!/usr/bin/env python3
"""
pd_board_emulator.py  (Flexlink_py/HW_Development/Packet_Detector/PC)

Software stand-in for the Zedboard: a TCP server that speaks pd_protocol and runs the
golden reference (pd_reference.py) where the board would run the FPGA detector. Use it to
develop and test the PC side now, and later as the spec for the board firmware: the board
must answer every command the same way.

    python pd_board_emulator.py                    # listens on 0.0.0.0:5001
    python pd_board_emulator.py --port 5002 --verbose
    python pd_hw_test.py TestVectors/<vector>.json --host 127.0.0.1

Deliberate fault injection (to check that pd_hw_test.py catches mismatches):
    --fault tshift     report t_trig one sample late
    --fault cfo        add 100 LSB to cfo_inc
"""
from __future__ import annotations

import argparse
import errno
import socket
import struct
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pd_protocol as pr  # noqa: E402
import pd_reference as ref  # noqa: E402


class Board:
    def __init__(self, max_samples: int, fault: str | None = None):
        self.max_samples = max_samples
        self.fault = fault
        self.params = pr.Params()
        self.reset()

    def reset(self):
        self.buf = np.zeros((0, 2), np.int16)
        self.history = np.zeros((0, 2), np.int16)      # every sample run since RESET
        self.final = set()                              # t_trig of events already reported complete
        self.tm_done = set()                            # t_trig of timing records already reported
        self.cf_done = set()                            # t_trig of control records already reported
        self.debug = None                               # DEBUG_DTYPE records of the last RUN

    # ---- commands -------------------------------------------------------- #
    def load(self, payload: bytes) -> bytes:
        (offset,) = struct.unpack_from("<I", payload)
        iq = pr.bytes_to_iq(payload[4:])
        end = offset + iq.shape[0]
        if end > self.max_samples:
            raise BoardError(pr.ST_NO_SPACE, f"buffer holds {self.max_samples} samples, load ends at {end}")
        if end > self.buf.shape[0]:
            self.buf = np.concatenate([self.buf, np.zeros((end - self.buf.shape[0], 2), np.int16)])
        self.buf[offset:end] = iq
        return struct.pack("<I", self.buf.shape[0])

    def read(self, payload: bytes) -> bytes:
        offset, count = struct.unpack_from("<II", payload)
        if offset + count > self.buf.shape[0] or count * 4 > pr.MAX_PAYLOAD:
            raise BoardError(pr.ST_BAD_ARG, f"read {offset}+{count} outside buffer {self.buf.shape[0]}")
        return pr.iq_to_bytes(self.buf[offset:offset + count])

    def run(self, payload: bytes) -> bytes:
        count, flags = struct.unpack_from("<II", payload)
        count = count or self.buf.shape[0]
        if count > self.buf.shape[0]:
            raise BoardError(pr.ST_BAD_ARG, f"run {count} samples, buffer has {self.buf.shape[0]}")
        div = 2 if self.params.input_stage == 2 else 1  # input samples per detector sample
        if div == 2 and count % 2 and not flags & pr.RUN_FLAG_PL_LOOPBACK:
            raise BoardError(pr.ST_BAD_ARG, "input_stage 2: RUN count must be even (whole 40 Msps sample pairs)")
        if flags & pr.RUN_FLAG_PL_LOOPBACK:          # samples through the PL unchanged
            self.debug = np.ascontiguousarray(self.buf[:count]).view(pr.DEBUG_DTYPE).ravel().copy()
            return struct.pack("<II", 0, count)
        start = self.history.shape[0] // div        # in detector (20 Msps) samples
        self.history = np.concatenate([self.history, self.buf[:count]])
        want_dbg = bool(flags & pr.RUN_FLAG_DEBUG)
        res = ref.detect(self.history, self.params, debug=want_dbg)
        out = []
        for e in res.events:
            if e.t_trig in self.final:
                continue
            if e.status & pr.EV_TRUNCATED:
                if e.t_trig >= start:                   # report once; final version comes later
                    out.append(e)
                continue
            self.final.add(e.t_trig)
            out.append(e)
        if self.fault == "tshift":
            for e in out:
                e.t_trig += 1
        elif self.fault == "cfo":
            for e in out:
                e.cfo_inc += 100
        tm = [r for r in res.timing if r.t_trig not in self.tm_done]
        self.tm_done.update(r.t_trig for r in tm)
        cf = [r for r in res.control if r.t_trig not in self.cf_done]
        self.cf_done.update(r.t_trig for r in cf)
        self.debug = None
        if want_dbg:
            self.debug = np.empty(count // div, pr.DEBUG_DTYPE)
            self.debug["metric"], self.debug["bits"] = res.metric[start:], res.bits[start:]
        return (struct.pack("<II", len(out), count) + b"".join(e.pack() for e in out)
                + struct.pack("<I", len(tm)) + b"".join(r.pack() for r in tm)
                + struct.pack("<I", len(cf)) + b"".join(r.pack() for r in cf))

    def get_debug(self, payload: bytes) -> bytes:
        offset, count = struct.unpack_from("<II", payload)
        if self.debug is None:
            raise BoardError(pr.ST_NO_DEBUG, "no debug capture - RUN with flag bit0 first")
        if offset + count > self.debug.size or count * 4 > pr.MAX_PAYLOAD:
            raise BoardError(pr.ST_BAD_ARG, f"debug {offset}+{count} outside capture {self.debug.size}")
        return self.debug[offset:offset + count].tobytes()

    def handle(self, cmd: int, payload: bytes) -> bytes:
        if cmd == pr.PING:
            return pr.BoardInfo(pr.PROTOCOL_VERSION, 0, self.max_samples, pr.BOARD_EMULATOR | pr.BOARD_TIMING | pr.BOARD_CF | pr.BOARD_REARM).pack()
        if cmd == pr.RESET:
            self.reset()
            return b""
        if cmd == pr.SET_PARAMS:
            p = pr.Params.unpack(payload)
            if not 0 <= p.tone_thresh <= ref.TONE_THRESH_MAX_Q8:
                raise BoardError(pr.ST_BAD_ARG, "tone_thresh must be 0..24 dB (Q8.8 0..6144)")
            if p.input_stage not in (0, 1, 2):
                raise BoardError(pr.ST_BAD_ARG, "input_stage must be 0 (LPF output), 1 (20 Msps input) "
                                                "or 2 (40 Msps input)")
            if not 0 <= p.t_pk <= 0xFFFF:
                raise BoardError(pr.ST_BAD_ARG, "t_pk is 16-bit (Q8.8)")
            self.params = p
            return b""
        if cmd == pr.LOAD_SAMPLES:
            return self.load(payload)
        if cmd == pr.READ_SAMPLES:
            return self.read(payload)
        if cmd == pr.RUN:
            return self.run(payload)
        if cmd == pr.GET_DEBUG:
            return self.get_debug(payload)
        raise BoardError(pr.ST_BAD_CMD, f"unknown command {cmd}")


class BoardError(Exception):
    def __init__(self, status: int, msg: str):
        super().__init__(msg)
        self.status = status


def serve(host: str, port: int, board: Board, verbose: bool) -> None:
    with socket.create_server((host, port), reuse_port=False) as srv:
        print(f"Emulated Zedboard listening on {host}:{port} (Ctrl+C to stop)")
        while True:
            conn, addr = srv.accept()
            print(f"Client {addr[0]}:{addr[1]} connected")
            with conn:
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                try:
                    while True:
                        cmd, _, seq, payload = pr.recv_msg(conn)
                        try:
                            reply, status = board.handle(cmd, payload), pr.ST_OK
                        except BoardError as exc:
                            reply, status = str(exc).encode(), exc.status
                        except Exception as exc:                       # noqa: BLE001
                            reply, status = f"internal error: {exc}".encode(), pr.ST_INTERNAL
                        if verbose:
                            print(f"  {pr.CMD_NAMES.get(cmd, cmd)} seq {seq}: {len(payload)} B in, "
                                  f"{len(reply)} B out, status {status}")
                        pr.send_msg(conn, cmd, reply, seq, status)
                except (ConnectionError, pr.ProtocolError) as exc:
                    print(f"Client disconnected ({exc})")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Emulated Zedboard packet detector (TCP server)")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=pr.DEFAULT_PORT)
    ap.add_argument("--max-samples", type=int, default=64 * 1024 * 1024,
                    help="sample buffer size (the Zedboard's 512 MB DDR holds about 100M)")
    ap.add_argument("--fault", choices=("tshift", "cfo"))
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args(argv)
    try:
        serve(a.host, a.port, Board(a.max_samples, a.fault), a.verbose)
    except KeyboardInterrupt:
        pass
    except OSError as exc:
        if exc.errno == errno.EADDRINUSE:
            print(f"Error: port {a.port} is already in use - probably an emulator is still running "
                  f"(stop it with: pkill -f pd_board_emulator.py), or use --port {a.port + 1}",
                  file=sys.stderr)
            return 1
        raise
    return 0


if __name__ == "__main__":
    sys.exit(main())
