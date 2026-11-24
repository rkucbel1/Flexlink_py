"""
Rx_Mavlink.py  (lives in Flexlink_py/App/Mavlink)

The MAVLink part of the receiver apps, shared by
    App/FlexlinkRxMavlinkWithZedboard_CoreBoard.py   (CoreBoard + Zedboard)
    App/FlexlinkRxMavlinkWithZedboard_FCOMMS3.py     (AD-FMCOMMS3 on the Zedboard)

  settings      App/Mavlink/Parameters_RxMavlink.json (the same file for both apps): telemetry
                window on Start, MAVLink to a ground station (UDP), video to a ground station
                (UDP MPEG-TS), save the MAVLink frames to App/Mavlink/Received.tlog
  gcs_frame     the GUI frame for these settings
  UdpTsSink     the video as UDP datagrams of whole TS packets (up to 7, 1316 bytes)
  TsTee         ffplay / file (Video_Framing.TsOutput) and UdpTsSink as one output
  mavlink_rx    a receiver class (the app's VideoRx with the MAVLink stream added): MAVLink
                blocks -> frames -> telemetry state, UDP to the ground station, .tlog
"""
from __future__ import annotations

import json
import os
import socket
import sys
import tempfile
import time
from collections import deque
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import Mavlink_Framing as MF  # noqa: E402
import Telemetry_Window as TW  # noqa: E402

VF = MF.VF
RXM_JSON = HERE / "Parameters_RxMavlink.json"
RECEIVED_TLOG = HERE / "Received.tlog"
TELEMETRY_PERIOD = 0.2                      # s between telemetry window updates
TS_DATAGRAM = 7 * VF.TS_PACKET              # 1316 bytes, as the Pi's encoder sends

# ---------- Settings ----------
RXM_DEFAULTS = {"ShowTelemetry": 1, "ForwardMavlink": 1, "MavlinkTo": "127.0.0.1:14550", "ForwardVideo": 0,
                "VideoTo": "127.0.0.1:5600", "SaveTlog": 0}
RXM_COMMENTS = {
    "ShowTelemetry_comment": "1: open the telemetry window on Start",
    "ForwardMavlink_comment": "1: send every received MAVLink frame by UDP to MavlinkTo (a ground station)",
    "MavlinkTo_comment": "host:port of the ground station; QGroundControl and Mission Planner listen on 14550",
    "ForwardVideo_comment": "1: send the received MPEG-TS video by UDP to VideoTo",
    "VideoTo_comment": "host:port; QGroundControl: Video source MPEG-TS on this port (default 5600)",
    "SaveTlog_comment": "1: save the received MAVLink frames to App/Mavlink/Received.tlog",
}
RXM_FIELDS = {"-RXM_MAV_TO-": ("MavlinkTo", "MAVLink to"), "-RXM_VID_TO-": ("VideoTo", "Video to")}
RXM_CHECKS = {"-RXM_SHOW-": "ShowTelemetry", "-RXM_FWD_MAV-": "ForwardMavlink", "-RXM_FWD_VID-": "ForwardVideo",
              "-RXM_SAVE-": "SaveTlog"}
RUN_LOCKED = ("-RXM_FWD_MAV-", "-RXM_FWD_VID-", "-RXM_SAVE-")     # cannot change while receiving


def load_rxm() -> dict:
    try:
        data = json.loads(RXM_JSON.read_text(encoding="utf-8"))
    except FileNotFoundError:
        data = {}
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON in {RXM_JSON}: line {e.lineno}, col {e.colno}: {e.msg}")
    sec = data.setdefault("RxMavlink", {})
    for k, v in RXM_DEFAULTS.items():
        sec.setdefault(k, v)
        sec.setdefault(f"{k}_comment", RXM_COMMENTS[f"{k}_comment"])
    return data


def rxm_from_values(values) -> tuple[dict, list[str]]:
    out, errors = {}, []
    for gui_key, (key, label) in RXM_FIELDS.items():
        out[key] = str(values[gui_key]).strip()
        try:
            MF.parse_host_port(out[key])
            if not out[key]:
                raise ValueError("empty")
        except ValueError as e:
            errors.append(f"{label}: host:port ({e})")
    for k, name in RXM_CHECKS.items():
        out[name] = int(bool(values[k]))
    return out, errors


def load_into_window(window) -> None:
    m = load_rxm()["RxMavlink"]
    for gui_key, (key, _label) in RXM_FIELDS.items():
        window[gui_key].update(str(m[key]))
    for k, name in RXM_CHECKS.items():
        window[k].update(bool(int(m[name])))


def save_rxm(rxm: dict) -> dict:
    data = load_rxm()
    data["RxMavlink"].update(rxm)
    fd, tmp = tempfile.mkstemp(dir=RXM_JSON.parent, prefix=RXM_JSON.name, suffix=".tmp")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")
    os.replace(tmp, RXM_JSON)
    return data["RxMavlink"]


def gcs_frame(sg, h2, italic):
    d = RXM_DEFAULTS
    return sg.Frame("Telemetry and ground station (Parameters_RxMavlink.json)", [
        [sg.Checkbox("Open the telemetry window on Start", key="-RXM_SHOW-", default=True),
         sg.Button("Telemetry", key="-TELE_OPEN-", size=10)],
        [sg.Checkbox("MAVLink to", key="-RXM_FWD_MAV-", default=True, size=(12, 1)),
         sg.Input(d["MavlinkTo"], key="-RXM_MAV_TO-", size=20), sg.Text("UDP; QGC listens on 14550", font=italic)],
        [sg.Checkbox("Video to", key="-RXM_FWD_VID-", size=(12, 1)),
         sg.Input(d["VideoTo"], key="-RXM_VID_TO-", size=20),
         sg.Text("UDP MPEG-TS; QGC: Video source MPEG-TS", font=italic)],
        [sg.Checkbox(f"Save the MAVLink frames to App/Mavlink/{RECEIVED_TLOG.name}", key="-RXM_SAVE-")],
    ], font=h2, vertical_alignment="top", expand_x=True)


def started_text(rxm: dict) -> str:
    return ((f"\nMAVLink -> {rxm['MavlinkTo']}" if rxm["ForwardMavlink"] else "")
            + (f"\nvideo -> {rxm['VideoTo']} (MPEG-TS)" if rxm["ForwardVideo"] else ""))


# ---------- Video outputs ----------
class UdpTsSink:
    """The video as UDP datagrams of whole TS packets: collected up to 7 (1316 bytes, as the Pi's
    encoder sends) and sent when full or at flush() (after every result push of the Zedboard, so
    the video waits at most one push)."""

    def __init__(self, host: str, port: int):
        self.addr = (host, int(port))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.buf = bytearray()
        self.bytes = 0
        self.datagrams = 0
        self.errors = 0

    def _send(self, d: bytes) -> None:
        try:
            self.sock.sendto(d, self.addr)
            self.bytes += len(d)
            self.datagrams += 1
        except OSError:
            self.errors += 1

    def write(self, ts: bytes) -> None:
        self.buf += ts
        while len(self.buf) >= TS_DATAGRAM:
            self._send(bytes(self.buf[:TS_DATAGRAM]))
            del self.buf[:TS_DATAGRAM]

    def flush(self) -> None:
        if self.buf:
            self._send(bytes(self.buf))
            self.buf.clear()

    def close(self) -> None:
        self.flush()
        self.sock.close()


class TsTee:
    """ffplay / file (VF.TsOutput) and UDP, for VideoRx (which writes to one output)."""

    def __init__(self, local, udp: UdpTsSink | None):
        self.local, self.udp = local, udp

    @property
    def note(self) -> str:
        return self.local.note if self.local is not None else ""

    def write(self, ts: bytes) -> None:
        if self.local is not None:
            self.local.write(ts)
        if self.udp is not None:
            self.udp.write(ts)

    def flush(self) -> None:
        if self.udp is not None:
            self.udp.flush()

    def close(self) -> None:
        for x in (self.local, self.udp):
            if x is not None:
                x.close()


def video_output(local, rxm: dict) -> TsTee:
    udp = UdpTsSink(*MF.parse_host_port(rxm["VideoTo"], 5600)) if rxm["ForwardVideo"] else None
    return TsTee(local, udp)


# ---------- The receiver class ----------
def mavlink_rx(base):
    """A subclass of the app's VideoRx (base) that also takes the MAVLink stream out of the
    blocks. base must have add(position, cfs, tbs) with tbs = [(t_trig, w, crc_ok, bits), ...],
    stats_line(t, st) and self.out."""

    class MavVideoRx(base):
        def __init__(self, out, rxm: dict):
            super().__init__(out)
            self.mu = MF.MavlinkUnpacker()
            self.tele = TW.TelemetryState()
            self.mav_udp = MF.UdpSink(*MF.parse_host_port(rxm["MavlinkTo"])) if rxm["ForwardMavlink"] else None
            self.tlog = MF.TlogSink(RECEIVED_TLOG) if rxm["SaveTlog"] else None
            self.video_udp = out.udp if isinstance(out, TsTee) else None
            self.mav_window: deque = deque(maxlen=16)        # (t, frames)

        def add(self, position, cfs, tbs) -> None:
            super().add(position, cfs, tbs)
            k = 0
            while k < len(tbs):                          # runs of blocks of the same size, as VideoRx.add
                nb = len(tbs[k][3])
                j = k
                while j < len(tbs) and len(tbs[j][3]) == nb:
                    j += 1
                run = tbs[k:j]
                crc = np.array([r[2] for r in run], bool)
                now = time.monotonic()
                for f in self.mu.feed(np.stack([r[3] for r in run]), crc):
                    self.tele.add(f, now)
                    if self.mav_udp is not None:
                        self.mav_udp.send(f.raw)
                    if self.tlog is not None:
                        self.tlog.write(f.raw)
                k = j
            if isinstance(self.out, TsTee):
                self.out.flush()

        def stats_line(self, t: float, st) -> str:
            line = super().stats_line(t, st)
            self.mav_window.append((t, self.mu.frames))
            (ta, fa), (tb, fb) = self.mav_window[0], self.mav_window[-1]
            rate = (fb - fa) / (tb - ta) if tb > ta else 0.0
            line += (f"\nMAVLink: {self.mu.frames} frames ({rate:.0f}/s) from {self.mu.blocks} blocks, "
                     f"{self.mu.lost} blocks lost, {self.mu.parser.bad_crc} bad frames")
            if self.mav_udp is not None:
                line += f" | {self.mav_udp.sent} sent to {self.mav_udp.addr[0]}:{self.mav_udp.addr[1]}"
            if self.tlog is not None:
                line += f" | {self.tlog.n} saved"
            if self.video_udp is not None:
                line += (f"\nvideo to {self.video_udp.addr[0]}:{self.video_udp.addr[1]}: "
                         f"{self.video_udp.bytes / 1e6:.1f} MB in {self.video_udp.datagrams} datagrams")
            return line

        def close(self) -> None:
            for x in (self.mav_udp, self.tlog):
                if x is not None:
                    x.close()

    return MavVideoRx


# ---------- Main-loop helper for the telemetry window ----------
class TelemetryUi:
    """The telemetry window's part of an app's main loop (sg.read_all_windows)."""

    def __init__(self, sg):
        self.win = TW.TelemetryWindow(sg)
        self.sg = sg
        self.last = None

    def owns(self, win, event) -> bool:
        """True if the event was the telemetry window's (handled here)."""
        if self.win.is_open and win is self.win.window:
            if event == self.sg.WIN_CLOSED:
                self.win.close()
            return True
        return False

    def open(self) -> None:
        self.win.open()
        if self.last is not None:
            self.win.update(self.last)

    def show(self, snap) -> None:
        if snap is not None:
            self.last = snap
            self.win.update(snap)

    def close(self) -> None:
        self.win.close()
