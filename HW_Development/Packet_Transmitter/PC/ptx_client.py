#!/usr/bin/env python3
"""
ptx_client.py  (Flexlink_py/HW_Development/Packet_Transmitter/PC)

PC side of the Pluto transmitter server (FW/ptx_server.c, TCP port 5010 on the Pluto's USB
network address 192.168.2.1):

  1. SET_PARAMS: the Defined sections of Tx/Parameters_Tx_payload.json and
     Tx/ControlSignals/Parameters_Control_Field.json (plus Gap, optional Gains and Radio). The
     Pluto replies with the Calculated sections and BlocksPerPacket / TransportBlockSize, and with
     its number of credits (2: one packet on air, one waiting).
  2. BLOCKS: one packet's transport blocks per message, while a credit is free.
  3. SENT: one per packet the PL has finished; it frees the credit. If the PC is late, the Pluto
     sends nothing until the next packet arrives. The settings stay until the next SET_PARAMS.

    python ptx_client.py ping
    python ptx_client.py send --packets 20                      # JSON files, random blocks
    python ptx_client.py send --forever --run run1              # until Ctrl-C (cycles 4 packets)
    python ptx_client.py send --blocks ../../../Tx/Generate_FileData/<file blocks>.h5
    python ptx_client.py status
    python ptx_client.py stop

--run writes <run>.npz (settings and the transport blocks of every distinct packet) for
ptx_pluto.py check. The reply is checked against the model (ptx_reference): a mismatch means the
server and the PC disagree on the settings.
"""
from __future__ import annotations

import argparse
import itertools
import json
import socket
import struct
import sys
import threading
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = next((p for p in [HERE, *HERE.parents] if (p / "flexlink_paths.py").is_file()), HERE.parents[2])
sys.path.insert(0, str(HERE))
import ptx_reference as R  # noqa: E402

MAGIC = 0x53585450
PING, PONG, SET_PARAMS, PARAMS, BLOCKS, READY, SENT, STATUS, STATUS_R, STOP, ERROR = range(1, 12)
NAMES = {PONG: "PONG", PARAMS: "PARAMS", READY: "READY", SENT: "SENT", STATUS_R: "STATUS", ERROR: "ERROR"}
HDR = struct.Struct("<IHHII")
TX_JSON = ROOT / "Tx" / "Parameters_Tx_payload.json"
CF_JSON = ROOT / "Tx" / "ControlSignals" / "Parameters_Control_Field.json"


class ServerError(RuntimeError):
    pass


class PtxClient:
    def __init__(self, host: str = "192.168.2.1", port: int = 5010, timeout: float = 10.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.timeout = timeout
        self.params: dict | None = None
        self.depth = 0
        self.outstanding: list[int] = []
        self.events: list[tuple[str, int, dict]] = []
        self.sent: list[dict] = []

    # framing
    def _send(self, typ: int, payload: bytes = b"", seq: int = 0) -> None:
        self.sock.sendall(HDR.pack(MAGIC, typ, 0, seq & 0xFFFFFFFF, len(payload)) + payload)

    def _recv_exact(self, n: int) -> bytes:
        b = bytearray()
        while len(b) < n:
            c = self.sock.recv(n - len(b))
            if not c:
                raise ConnectionError("server closed the connection")
            b += c
        return bytes(b)

    def _recv(self) -> tuple[int, int, dict]:
        magic, typ, _, seq, n = HDR.unpack(self._recv_exact(HDR.size))
        if magic != MAGIC:
            raise ServerError(f"bad magic {magic:08x}")
        body = self._recv_exact(n) if n else b""
        msg = json.loads(body.decode()) if body else {}
        self.events.append((NAMES.get(typ, str(typ)), seq, msg))
        if typ == SENT:
            self.sent.append(msg)
            if self.outstanding and self.outstanding[0] == seq:
                self.outstanding.pop(0)
            elif seq in self.outstanding:
                self.outstanding.remove(seq)
        if typ == ERROR:
            raise ServerError(msg.get("error", "error"))
        return typ, seq, msg

    def _wait(self, want: int) -> dict:
        while True:
            typ, _, msg = self._recv()
            if typ == want:
                return msg

    # commands
    def ping(self) -> dict:
        self._send(PING)
        return self._wait(PONG)

    def set_params(self, tx_json: Path = TX_JSON, cf_json: Path | None = CF_JSON, gap: int = 4096,
                   gains: dict | None = None, radio: dict | None = None) -> dict:
        tx = json.loads(Path(tx_json).read_text())["Defined"]
        cf = json.loads(Path(cf_json).read_text())["Defined"] if cf_json is not None and Path(cf_json).is_file() else None
        return self.set_defined(tx, cf, gap, gains, radio)

    def set_defined(self, tx: dict, cf: dict | None = None, gap: int = 4096, gains: dict | None = None,
                    radio: dict | None = None) -> dict:
        """SET_PARAMS with the Defined sections as dicts (the contents of the two JSON files)."""
        msg = {"Tx": tx, "Gap": int(gap)}
        if cf is not None:
            msg["ControlField"] = cf
        if gains:
            msg["Gains"] = gains
        if radio:
            msg["Radio"] = radio
        self._send(SET_PARAMS, json.dumps(msg).encode())
        rep = self._wait(PARAMS)
        if not rep.get("ok"):
            raise ServerError(f"settings rejected: {rep.get('error')}")
        self.depth = int(self._wait(READY).get("depth", 2))
        self.params = rep
        self.outstanding = []
        return rep

    def blocks_payload(self, tbs) -> bytes:
        p = self.params
        tbs = np.asarray(tbs).reshape(-1, p["TransportBlockSize"])
        if tbs.shape[0] != p["BlocksPerPacket"]:
            raise ValueError(f"{tbs.shape[0]} blocks, one packet is {p['BlocksPerPacket']}")
        words = np.concatenate([R.pack_bits(t) for t in tbs]).astype("<u8")
        return struct.pack("<I", tbs.shape[0]) + words.tobytes()

    def send_packet(self, seq: int, tbs) -> None:
        """Waits for a credit, then sends one packet's blocks."""
        while len(self.outstanding) >= self.depth:
            self._recv()
        self._send(BLOCKS, self.blocks_payload(tbs), seq)
        self.outstanding.append(seq)

    def wait_sent(self, timeout: float = 30.0) -> None:
        end = time.monotonic() + timeout
        while self.outstanding:
            if time.monotonic() > end:
                raise TimeoutError(f"{len(self.outstanding)} packet(s) not reported sent")
            self._recv()

    def status(self) -> dict:
        self._send(STATUS)
        return self._wait(STATUS_R)

    def stop(self) -> dict:
        self._send(STOP)
        return self._wait(STATUS_R)

    def close(self) -> None:
        self.sock.close()


def check_reply(rep: dict, cfg: R.TxConfig) -> list[str]:
    """The server's derived values against the model; returns the differences."""
    calc = rep["Tx"]["Calculated"]
    mine = {"AvailableDataBlocks": cfg.ndb, "TransportBlockSize": cfg.tb_size, "ZeroPadding": cfg.zero_pad,
            "NumberOfDataSigs": cfg.data_bits, "CodeWordSize": cfg.cw_size, "DataBlockSize": cfg.k}
    out = [f"{k}: server {calc[k]}, model {v}" for k, v in mine.items() if calc[k] != v]
    msg = "".join(map(str, R.cf_message(cfg)))
    if rep["ControlMessage"] != msg:
        out.append(f"ControlMessage: server {rep['ControlMessage']}, model {msg}")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Client of the Pluto transmitter server")
    ap.add_argument("cmd", choices=("ping", "send", "status", "stop"))
    ap.add_argument("--host", default="192.168.2.1")
    ap.add_argument("--port", type=int, default=5010)
    ap.add_argument("--params", type=Path, default=TX_JSON)
    ap.add_argument("--cf-params", type=Path, default=CF_JSON)
    ap.add_argument("--gap", type=int, default=4096, help="idle samples after each packet")
    ap.add_argument("--packets", type=int, default=4, help="packets to send (distinct random packets)")
    ap.add_argument("--forever", action="store_true", help="cycle the packets until Ctrl-C")
    ap.add_argument("--blocks", type=Path, help="transport-block .h5 instead of random blocks")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--run", help="write <run>.npz for ptx_pluto.py check")
    ap.add_argument("--lo", type=float, help="TX LO, Hz (the server sets it)")
    ap.add_argument("--tx-gain", type=float, help="TX gain, dB (<= 0)")
    ap.add_argument("--bw", type=float, default=18e6, help="RF bandwidth, Hz")
    a = ap.parse_args(argv)

    c = PtxClient(a.host, a.port)
    if a.cmd == "ping":
        print(c.ping())
        return 0
    if a.cmd == "status":
        print(c.status())
        return 0
    if a.cmd == "stop":
        print(c.stop())
        return 0

    cfg = R.TxConfig.from_params(a.params, a.cf_params)
    radio = {k: v for k, v in (("TxLo", a.lo), ("TxGain", a.tx_gain), ("RfBandwidth", a.bw)) if v is not None}
    rep = c.set_params(a.params, a.cf_params, a.gap, radio=radio)
    diff = check_reply(rep, cfg)
    print(f"Settings accepted: {rep['BlocksPerPacket']} blocks of {rep['TransportBlockSize']} bits per packet, "
          f"{rep['PacketSamples']} samples + gap {rep['Gap']}, {c.depth} credits"
          + ("" if not diff else "\n  DIFFERENT FROM THE MODEL: " + "; ".join(diff)))
    if a.blocks:
        import ptx_stream
        pk = ptx_stream.packets_from_bits(cfg, ptx_stream.load_blocks(a.blocks))
    else:
        rng = np.random.default_rng(a.seed)
        pk = [R.random_tbs(cfg, rng) for _ in range(a.packets)]
    if a.run:
        from dataclasses import asdict
        meta = {"cfg": asdict(cfg), "gap": a.gap, "mode": "server", "message": R.cf_message(cfg).tolist(),
                "seed": a.seed}
        np.savez(Path(a.run).with_suffix(".npz"), tbs=np.array(pk, np.uint8), meta=json.dumps(meta))
    seqs = itertools.count()
    t0 = time.monotonic()
    try:
        src = itertools.cycle(enumerate(pk)) if a.forever else enumerate(pk)
        for k, tbs in src:
            c.send_packet(next(seqs), tbs)
            if c.sent and c.sent[-1].get("seq", 0) % 50 == 0:
                s = c.sent[-1]
                print(f"\r  sent {s['sent']}, PL packets {s['pl_packets']}, underflows {s['underflows']}",
                      end="", flush=True)
        c.wait_sent()
    except KeyboardInterrupt:
        print()
    dt = time.monotonic() - t0
    n = len(c.sent)
    bits = n * rep["BlocksPerPacket"] * rep["TransportBlockSize"]
    print(f"{n} packet(s) sent in {dt:.2f} s ({bits / max(dt, 1e-9) / 1e6:.2f} Mbit/s of transport blocks); "
          f"status {c.status()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
