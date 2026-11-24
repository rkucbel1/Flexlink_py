"""FlexlinkRxVideoWithZedboard_CoreBoard.py -- receives the live video of FlexlinkTxVideoWithPluto.py and plays it (GUI).

The mirror of the transmitter app. (For an AD-FMCOMMS3 on the Zedboard instead of the CoreBoard:
FlexlinkRxVideoWithZedboard_FCOMMS3.py.) The radio side is the CoreBoard and the Zedboard receiver (the
Pluto transmits; it cannot decode): the CoreBoard streams 40 Msps to the Zedboard, whose FPGA
receiver decodes the packets and pushes the transport blocks to this PC as they come (firmware
build 39, bitstream 9.1 or later, as pd_stream_test.py --push):

    Pluto  ~~radio~~>  CoreBoard  --UDP samples-->  Zedboard (FPGA receiver)
        --TCP: transport blocks, CRC flags-->  this PC  --unpack, MPEG-TS-->  ffplay

The Zedboard needs no Flexlink parameters: it reads modulation, code and repetition from each
packet's control field. The Transmitter line shows that control field (the transmitter's mode).

  CoreBoard   App/Rx_CoreBoard/Parameters_CoreBoard.json (shared with FlexlinkCaptureWithCoreBoard.py): its
              address, carrier (the Pluto's TxLO_MHz), gain
  Zedboard    App/Rx_CoreBoard/Parameters_RxVideo.json: the board's address and ports, how often it pushes
              results, the DC blocker (6 for the CoreBoard)
  Video       play with ffplay (sudo apt install ffmpeg); optionally also save the received
              stream to App/Rx_CoreBoard/Video/Received.ts (about 3 GB an hour at 7 Mbit/s: off by default)

The blocks are put back together as in Flexlink_BasicModules/Video_Transfer/Video_Framing.py: a
block whose CRC failed, or a block missing from the count, is a gap; the broken TS packet is
dropped and the stream re-aligns, and the decoder recovers with the encoder's intra refresh.

Start saves the settings first (as Save does). Start the transmitter app before or after.

Requires: pip install FreeSimpleGUI; ffplay (ffmpeg) for the picture
Run with: python App/Rx_CoreBoard/FlexlinkRxVideoWithZedboard_CoreBoard.py
"""
from __future__ import annotations

import json
import os
import socket
import struct
import sys
import tempfile
import threading
import time
from collections import deque
from dataclasses import fields
from pathlib import Path

import numpy as np

for _p in Path(__file__).resolve().parents:      # put the project root (flexlink_paths.py) on sys.path
    if (_p / "flexlink_paths.py").is_file():
        if str(_p) not in sys.path:
            sys.path.insert(0, str(_p))
        break
from flexlink_paths import (APP, APP_SIMULATION, APP_RX_COREBOARD, BASIC_MODULES, ROOT,  # noqa: E402
                            PACKET_DETECTOR_PC)
sys.path.insert(0, str(APP_SIMULATION))
import Flexlinksimulation as FS  # noqa: E402  (fonts, theme)

import FreeSimpleGUI as sg  # noqa: E402

PD_PC = PACKET_DETECTOR_PC
sys.path.insert(0, str(PD_PC))
import pd_protocol as pr  # noqa: E402
import pd_stream_test as PS  # noqa: E402  (CoreBoard start body, HTTP status, STREAM_START payload)
from pd_hw_test import Board  # noqa: E402

try:
    import cf_fields  # noqa: E402  (control field -> transmitter mode; needs the simulator project)
except Exception:  # noqa: BLE001
    cf_fields = None

sys.path.insert(0, str(BASIC_MODULES / "Video_Transfer"))
import Video_Framing as VF  # noqa: E402

sg.theme("SystemDefault")
H1, H2 = FS.H1, FS.H2
ITALIC = ("Helvetica", 9, "italic")
RESULTS_FONT = FS.RESULTS_FONT

CB_JSON = APP_RX_COREBOARD / "Parameters_CoreBoard.json"
RXV_JSON = APP_RX_COREBOARD / "Parameters_RxVideo.json"
RECEIVED_TS = APP_RX_COREBOARD / "Video" / "Received.ts"
RX_PARAMS = PD_PC.parent / "TestVectors" / "rep3.json"       # detector thresholds, as pd_stream_test.py

# ---------- Settings ----------
CB_DEFAULTS = {"Address": "10.42.0.133", "HttpPort": 80, "Frequency_MHz": 2440, "SampleRate": 40_000_000,
               "AGC": 0, "Gain": 40, "Bandwidth_MHz": 0, "DC_Correction": 1}
CB_FIELDS = {   # gui key: (json key, type, min, max, label)
    "-CB_ADDR-": ("Address", str, None, None, "CoreBoard Address"),
    "-CB_HTTP-": ("HttpPort", int, 1, 65535, "HTTP Port"),
    "-CB_FREQ-": ("Frequency_MHz", int, 2150, 2800, "Carrier (MHz)"),
    "-CB_GAIN-": ("Gain", int, 0, 127, "Gain (index)"),
    "-CB_BW-": ("Bandwidth_MHz", int, 0, 54, "Analog filter (MHz)"),
}
RXV_DEFAULTS = {"ZedAddress": "10.42.0.101", "ZedPort": pr.DEFAULT_PORT, "ZedUdpPort": pr.UDP_DEFAULT_PORT,
                "Push_ms": 1.0, "DcShift": 6, "Play": 1, "SaveStream": 0}
RXV_COMMENTS = {
    "ZedAddress_comment": "IP address of the Zedboard receiver",
    "ZedPort_comment": "its TCP port for commands and results",
    "ZedUdpPort_comment": "its UDP port for the CoreBoard's sample stream",
    "Push_ms_comment": "the board pushes results at most this often while there are new ones, 0.25 to 7.75 ms",
    "DcShift_comment": "receiver DC blocker: 6 for the CoreBoard",
    "Play_comment": "1: play the received video with ffplay",
    "SaveStream_comment": "1: also save the received stream to App/Rx_CoreBoard/Video/Received.ts (large)",
}
RXV_FIELDS = {
    "-ZED_ADDR-": ("ZedAddress", str, None, None, "Zedboard Address"),
    "-ZED_PORT-": ("ZedPort", int, 1, 65535, "TCP Port"),
    "-ZED_UDP-": ("ZedUdpPort", int, 1024, 65535, "UDP Port (samples)"),
    "-ZED_PUSH-": ("Push_ms", float, 0.25, 7.75, "Push interval (ms)"),
    "-ZED_DC-": ("DcShift", int, 0, 15, "DC shift"),
}
LW = 18


def in_row(label, key, default, size=14, extra=None):
    row = [sg.Text(label, size=(LW, 1), justification="right"),
           sg.Input(str(default), key=key, size=size, justification="right")]
    return row + ([sg.Text(extra, font=ITALIC)] if extra else [])


cb_frame = sg.Frame("CoreBoard (Parameters_CoreBoard.json)", [
    in_row("CoreBoard Address", "-CB_ADDR-", CB_DEFAULTS["Address"], size=16),
    in_row("HTTP Port", "-CB_HTTP-", CB_DEFAULTS["HttpPort"]),
    in_row("Carrier (MHz)", "-CB_FREQ-", CB_DEFAULTS["Frequency_MHz"], extra="= the Pluto's carrier"),
    in_row("Gain (index)", "-CB_GAIN-", CB_DEFAULTS["Gain"], extra="not dB"),
    in_row("Analog filter (MHz)", "-CB_BW-", CB_DEFAULTS["Bandwidth_MHz"], extra="0 = follow the rate"),
    [sg.Text("", size=(LW, 1)), sg.Checkbox("AGC (not recommended)", key="-CB_AGC-")],
    [sg.Text("", size=(LW, 1)), sg.Checkbox("DC correction", key="-CB_DC-", default=True)],
    [sg.Text("40 Msps to the Zedboard", font=ITALIC)],
], font=H2, vertical_alignment="top", expand_x=True)

zed_frame = sg.Frame("Zedboard receiver (Parameters_RxVideo.json)", [
    in_row("Zedboard Address", "-ZED_ADDR-", RXV_DEFAULTS["ZedAddress"], size=16),
    in_row("TCP Port", "-ZED_PORT-", RXV_DEFAULTS["ZedPort"]),
    in_row("UDP Port (samples)", "-ZED_UDP-", RXV_DEFAULTS["ZedUdpPort"]),
    in_row("Push interval (ms)", "-ZED_PUSH-", RXV_DEFAULTS["Push_ms"], extra="0.25 .. 7.75"),
    in_row("DC shift", "-ZED_DC-", RXV_DEFAULTS["DcShift"], extra="6 for the CoreBoard"),
    [sg.Button("Check", key="-CHECK-", size=10), sg.Text("Zedboard firmware, CoreBoard status", font=ITALIC)],
], font=H2, vertical_alignment="top", expand_x=True)

video_frame = sg.Frame("Video", [
    [sg.Checkbox("Play with ffplay", key="-PLAY-", default=True)],
    [sg.Checkbox("Save the received stream", key="-SAVE_TS-")],
    [sg.Text(f"   to App/Rx_CoreBoard/Video/{RECEIVED_TS.name} (about 3 GB an hour at 7 Mbit/s)", font=ITALIC)],
], font=H2, vertical_alignment="top", expand_x=True)

layout = [
    [sg.Text("Flexlink - Video Receiver (CoreBoard + Zedboard)", font=H1, justification="center",
             expand_x=True, pad=(0, 15))],
    [sg.Push(), sg.Column([[cb_frame]], vertical_alignment="top"),
     sg.Column([[zed_frame], [video_frame]], vertical_alignment="top"), sg.Push()],
    [sg.Push(), sg.Text("Transmitter", size=(12, 1), justification="right"),
     sg.Multiline(size=(110, 2), key="-CF-", disabled=True, font=RESULTS_FONT), sg.Push()],
    [sg.Push(), sg.Text("Receiver", size=(12, 1), justification="right"),
     sg.Multiline(size=(110, 10), key="-RX_STATUS-", disabled=True, autoscroll=True, font=RESULTS_FONT),
     sg.Push()],
    [sg.Push(), sg.Button("Save", key="-SAVE-", size=10), sg.Button("Start", key="-START-", size=10),
     sg.Button("Stop", key="-STOP-", size=10, disabled=True), sg.Push()],
    [sg.Text("", key="-STATUS-", size=(110, 1), justification="center")],
]


def write_json_atomic(path: Path, data: dict) -> None:
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=path.name, suffix=".tmp")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")
    os.replace(tmp, path)


def load_section(path: Path, section: str, defaults: dict, comments: dict | None = None) -> dict:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        data = {}
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON in {path}: line {e.lineno}, col {e.colno}: {e.msg}")
    sec = data.setdefault(section, {})
    for k, v in defaults.items():
        sec.setdefault(k, v)
        if comments and f"{k}_comment" in comments:
            sec.setdefault(f"{k}_comment", comments[f"{k}_comment"])
    return data


def parse_fields(values, spec: dict) -> tuple[dict, list[str]]:
    out, errors = {}, []
    for gui_key, (key, typ, lo, hi, label) in spec.items():
        raw = str(values[gui_key]).strip()
        if typ is str:
            out[key] = raw
            if not raw:
                errors.append(f"{label}: empty")
            continue
        try:
            v = typ(float(raw)) if typ is int and float(raw).is_integer() else typ(raw)
            if not lo <= v <= hi:
                raise ValueError
            out[key] = v
        except ValueError:
            errors.append(f"{label}: {'whole ' if typ is int else ''}number from {lo:g} to {hi:g}")
    return out, errors


def settings_from_values(values) -> tuple[dict, dict, list[str]]:
    cb, e1 = parse_fields(values, CB_FIELDS)
    cb["AGC"] = int(bool(values["-CB_AGC-"]))
    cb["DC_Correction"] = int(bool(values["-CB_DC-"]))
    rxv, e2 = parse_fields(values, RXV_FIELDS)
    rxv["Play"] = int(bool(values["-PLAY-"]))
    rxv["SaveStream"] = int(bool(values["-SAVE_TS-"]))
    return cb, rxv, e1 + e2


def load_into_window(window) -> None:
    cb = load_section(CB_JSON, "CoreBoard", CB_DEFAULTS)["CoreBoard"]
    for gui_key, (key, *_r) in CB_FIELDS.items():
        window[gui_key].update(str(cb[key]))
    window["-CB_AGC-"].update(bool(int(cb["AGC"])))
    window["-CB_DC-"].update(bool(int(cb["DC_Correction"])))
    rxv = load_section(RXV_JSON, "RxVideo", RXV_DEFAULTS, RXV_COMMENTS)["RxVideo"]
    for gui_key, (key, *_r) in RXV_FIELDS.items():
        window[gui_key].update(str(rxv[key]))
    window["-PLAY-"].update(bool(int(rxv["Play"])))
    window["-SAVE_TS-"].update(bool(int(rxv["SaveStream"])))


def save_settings(values) -> tuple[dict, dict]:
    """Writes App/Rx_CoreBoard/Parameters_CoreBoard.json (the fields shown; the others are kept) and
    App/Rx_CoreBoard/Parameters_RxVideo.json. Raises ValueError, nothing written."""
    cb, rxv, errors = settings_from_values(values)
    if errors:
        raise ValueError("\n".join(errors))
    cbd = load_section(CB_JSON, "CoreBoard", CB_DEFAULTS)
    cbd["CoreBoard"].update(cb)
    rvd = load_section(RXV_JSON, "RxVideo", RXV_DEFAULTS, RXV_COMMENTS)
    rvd["RxVideo"].update(rxv)
    write_json_atomic(CB_JSON, cbd)
    write_json_atomic(RXV_JSON, rvd)
    return cbd["CoreBoard"], rvd["RxVideo"]


# ---------- Fast record parsing (the transport blocks as NumPy bit arrays) ----------
def parse_records(b: bytes):
    """As pd_protocol.parse_run_reply_tb, but only the control records and the transport blocks,
    and each block's bits as a uint8 array (pd_protocol builds a tuple per bit, too slow for
    several Mbit/s). Returns (control records, [(t_trig, w, crc_ok, bits), ...])."""
    if len(b) < 8:
        return [], []
    n_ev, _ = struct.unpack_from("<II", b)
    off = 8 + pr.EventRecord.FMT.size * n_ev
    cfs, tbs = [], []
    sizes = [pr.TimingRecord.FMT.size, pr.ControlRecord.FMT.size, pr.PayloadRecord.FMT.size,
             pr.PayloadRecord.FMT.size, pr.PayloadRecord.FMT.size]
    for k, size in enumerate(sizes):
        if len(b) < off + 4:
            return cfs, tbs
        (n,) = struct.unpack_from("<I", b, off)
        if k == 1:
            cfs = pr.ControlRecord.unpack_many(b[off + 4:], n)
        off += 4 + size * n
    if len(b) < off + 4:
        return cfs, tbs
    (n_tb,) = struct.unpack_from("<I", b, off)
    off += 4
    hdr = pr.TBRecord.HDR
    for _ in range(n_tb):
        t, w, ok, _it, nb = hdr.unpack_from(b, off)
        off += hdr.size
        nw = (nb + 31) // 32
        words = np.frombuffer(b, dtype="<u4", count=nw, offset=off)
        off += 4 * nw
        bits = np.unpackbits(words.view(np.uint8), bitorder="little")[:nb]   # bit j = word j//32, bit j%32
        tbs.append((t, w, ok, bits))
    return cfs, tbs


def parse_push(payload: bytes):
    flags, _, slot = struct.unpack_from("<IIQ", payload)
    off, st = 16, None
    if flags & 1:
        st = pr.StreamStats.unpack(payload[16:])
        off += st.size
    return slot, st, *parse_records(payload[off:])


def parse_poll(reply: bytes):
    st = pr.StreamStats.unpack(reply)
    return st, *parse_records(reply[st.size:])


# ---------- Receiver state: blocks -> video ----------
class VideoRx:
    def __init__(self, out: VF.TsOutput | None):
        self.unpacker = VF.VideoUnpacker()
        self.resync = VF.TsResync()
        self.out = out
        self.blocks = self.ok = 0
        self.packets = 0
        self.last_trig = None
        self.cf_message = None
        self.cf_text = ""
        self.ages: deque[float] = deque(maxlen=2000)       # ms, first block of each packet
        self.window: deque[tuple[float, int, int, int]] = deque(maxlen=16)   # (t, stream bytes, blocks, ok)

    def add(self, slot, cfs, tbs) -> None:
        for c in cfs:
            if c.crc_ok and c.message != self.cf_message:
                self.cf_message = c.message
                self.cf_text = (cf_fields.describe(c.message) if cf_fields is not None
                                else f"control field message 0x{c.message:014x}")
        if not tbs:
            return
        if slot is not None:
            now20 = (slot * pr.PUSH_SLOT_SAMPLES) & 0xFFFFFFFF
        k = 0
        while k < len(tbs):                         # runs of blocks of the same size
            nb = len(tbs[k][3])
            j = k
            while j < len(tbs) and len(tbs[j][3]) == nb:
                t = tbs[j][0]
                if t != self.last_trig:
                    self.packets += 1
                    self.last_trig = t
                    if slot is not None:
                        d = (now20 - t) & 0xFFFFFFFF
                        if d < 1 << 31:
                            self.ages.append(d / 20e3)
                j += 1
            run = tbs[k:j]
            crc = np.array([r[2] for r in run], bool)
            self.blocks += len(run)
            self.ok += int(crc.sum())
            for gap, data in self.unpacker.feed(np.stack([r[3] for r in run]), crc):
                ts = self.resync.push(data, gap)
                if ts and self.out is not None:
                    self.out.write(ts)
            k = j

    def stats_line(self, t: float, st) -> str:
        u = self.unpacker.total
        self.window.append((t, u.stream_bytes, self.blocks, self.packets))
        (ta, ba, _, pa), (tb, bb, _, pb) = self.window[0], self.window[-1]
        dt = max(tb - ta, 1e-9)
        bler = 1 - self.ok / self.blocks if self.blocks else 0.0
        ages = sorted(self.ages)
        age = f"{ages[len(ages) // 2]:.1f} ms" if ages else "-"
        line = (f"{t:6.1f} s  packets {(pb - pa) / dt:5.0f}/s, blocks {self.ok}/{self.blocks} CRC ok "
                f"(BLER {bler:.1e}), lost {u.lost} | video {(bb - ba) * 8 / dt / 1e3:6.0f} kbit/s, "
                f"{u.gaps} gaps, {self.resync.ts_packets} TS packets, {self.resync.resyncs} resyncs, "
                f"{u.foreign} non-video blocks | block age {age}")
        if st is not None:
            line += (f"\nCoreBoard stream: {st.datagrams} datagrams, {st.lost} lost (zero-filled), "
                     f"ring max {st.ring_max}, resyncs {st.resyncs}, HTTP {pr.HTTP_STATES.get(st.http_state, '?')}"
                     f" {st.http_code or ''}")
            if st.http_state == 4:
                line += " - the Zedboard could not reach the CoreBoard's HTTP port"
        if self.out is not None and self.out.note:
            line += f"\nvideo: {self.out.note}"
        return line


def detector_params(dc_shift: int) -> pr.Params:
    params = pr.Params()
    if RX_PARAMS.is_file():
        meta = json.loads(RX_PARAMS.read_text())
        known = {f.name for f in fields(pr.Params)}
        params = pr.Params(**{k: v for k, v in meta.get("params", {}).items() if k in known})
    params.input_stage = 2                  # 40 Msps in, half-band on the board
    params.dc_shift = int(dc_shift)
    return params


def check(cb: dict, rxv: dict) -> str:
    out = []
    try:
        b = Board(rxv["ZedAddress"], int(rxv["ZedPort"]), 5.0)
        info = b.ping()
        b.sock.close()
        ok = info.build >= 39 and info.flags & pr.BOARD_PACK8
        out.append(f"Zedboard {rxv['ZedAddress']}: firmware build {info.build}"
                   + (", bitstream 9.2 (NOREC)" if info.flags & pr.BOARD_NOREC else "")
                   + ("" if ok else " - needs firmware build 39 and bitstream 9.1 or later (push mode)"))
    except (OSError, pr.ProtocolError) as e:
        out.append(f"Zedboard {rxv['ZedAddress']}:{rxv['ZedPort']}: no answer ({e})")
    try:
        st = PS.http_status(cb["Address"], int(cb["HttpPort"]))
        out.append(f"CoreBoard {cb['Address']}: {st.get('product', '?')}, Ethernet {st.get('ethernet_mbps')} Mbit/s")
    except (OSError, ValueError) as e:
        out.append(f"CoreBoard {cb['Address']}:{cb['HttpPort']}: no answer ({e}) - from this PC; the Zedboard "
                   f"reaches it on its own")
    return "\n".join(out)


# ---------- The session (worker thread) ----------
def rx_worker(post, cb: dict, rxv: dict, stop: threading.Event) -> None:
    """post(kind, **kw): "started" | "stats" | "cf" | "done"."""
    error = None
    b = out = rx = None
    streaming = False
    st = None
    t0 = time.monotonic()
    try:
        out = VF.TsOutput(play=bool(rxv["Play"]), save_path=RECEIVED_TS if rxv["SaveStream"] else None,
                          title="Flexlink received video")
        rx = VideoRx(out)
        b = Board(rxv["ZedAddress"], int(rxv["ZedPort"]), 10.0)
        info = b.ping()
        if info.build < 39 or not info.flags & pr.BOARD_PACK8:
            raise RuntimeError(f"the Zedboard has firmware build {info.build}: push mode needs build 39 "
                               f"and bitstream 9.1 or later")
        b.call(pr.SET_PARAMS, detector_params(rxv["DcShift"]).pack())
        body = PS.coreboard_start_json(int(rxv["ZedUdpPort"]))     # from App/Rx_CoreBoard/Parameters_CoreBoard.json
        push_units = min(max(round(float(rxv["Push_ms"]) * 4), 1), 31)
        flags = pr.STREAM_FLAG_PUSH | (push_units << 2)
        reply = b.call(pr.STREAM_START, PS.start_payload(int(rxv["ZedUdpPort"]), flags, cb["Address"],
                                                          int(cb["HttpPort"]), body))
        streaming = True
        post("started", text=(f"Zedboard {rxv['ZedAddress']}: firmware build {info.build}, streaming on its UDP port "
                              f"{struct.unpack('<I', reply[:4])[0]}; CoreBoard {cb['Address']}: "
                              f"{body['frequency'] / 1e6:g} MHz, {body['rate'] / 1e6:g} Msps, "
                              + ("AGC" if body["agc"] else f"gain {body.get('gain')}")
                              + f"; results pushed at most every {push_units / 4:g} ms"))
        b.sock.settimeout(5.0)
        t0 = time.monotonic()
        t_post = 0.0
        cf_shown = None
        while not stop.is_set():
            cmd, _status, _seq, raw = pr.recv_msg(b.sock)
            if cmd != pr.STREAM_PUSH:
                raise pr.ProtocolError(f"unexpected message {pr.CMD_NAMES.get(cmd, cmd)} while streaming")
            slot, st_new, cfs, tbs = parse_push(raw)
            st = st_new or st
            rx.add(slot, cfs, tbs)
            now = time.monotonic()
            if rx.cf_text != cf_shown:
                cf_shown = rx.cf_text
                post("cf", text=cf_shown)
            if now - t_post > 0.5:
                t_post = now
                post("stats", text=rx.stats_line(now - t0, st))
        # STREAM_STOP: the pushes already on their way, then the reply (the last results)
        stop_js = json.dumps({"op": "stop"}).encode()
        b.seq += 1
        pr.send_msg(b.sock, pr.STREAM_STOP, struct.pack("<H", len(stop_js)) + stop_js, b.seq)
        while True:
            cmd, status, seq, raw = pr.recv_msg(b.sock)
            if cmd == pr.STREAM_PUSH:
                slot, st_new, cfs, tbs = parse_push(raw)
                st = st_new or st
                rx.add(slot, cfs, tbs)
                continue
            if cmd != pr.STREAM_STOP or seq != b.seq or status != pr.ST_OK:
                raise pr.ProtocolError(f"STREAM_STOP reply: cmd {cmd} seq {seq} status {status}")
            break
        streaming = False
        st, cfs, tbs = parse_poll(raw)
        rx.add(None, cfs, tbs)
    except socket.timeout:
        error = "no results from the Zedboard for 5 s"
    except Exception as e:  # noqa: BLE001  (shown in the GUI)
        error = f"{type(e).__name__}: {e}"
    finally:
        if b is not None:
            if streaming:
                try:
                    b.sock.settimeout(3.0)
                    b.call(pr.STREAM_STOP, struct.pack("<H", 0))
                except Exception:  # noqa: BLE001
                    pass
            b.sock.close()
        if out is not None:
            out.close()
    post("done", error=error, summary=rx.stats_line(time.monotonic() - t0, st) if rx is not None else "")


# ---------- Window ----------
def set_running(window, running: bool) -> None:
    window.metadata = running
    for k in ("-SAVE-", "-START-", "-CHECK-"):
        window[k].update(disabled=running)
    window["-STOP-"].update(disabled=not running)


def main():
    window = sg.Window("Flexlink Video Rx", layout, finalize=True, resizable=True)
    window.metadata = False
    stop = threading.Event()
    stats_lines = 0
    try:
        load_into_window(window)
        window["-STATUS-"].update("Loaded Parameters_CoreBoard.json, Parameters_RxVideo.json")
    except (OSError, ValueError, KeyError) as e:
        sg.popup_error(f"Could not load the parameter files:\n{e}")

    def post(kind, **kw):
        window.write_event_value("-RX_EVENT-", {"kind": kind, **kw})

    def append(text):
        cur = window["-RX_STATUS-"].get().rstrip("\n")
        window["-RX_STATUS-"].update(cur + ("\n" if cur.strip() else "") + text)

    while True:
        event, values = window.read()
        if event == sg.WIN_CLOSED:
            if window.metadata:
                stop.set()
                time.sleep(1.0)
            break
        if event == "-SAVE-":
            try:
                save_settings(values)
                window["-STATUS-"].update("Saved Parameters_CoreBoard.json, Parameters_RxVideo.json.")
            except (OSError, ValueError) as e:
                sg.popup_error(f"Nothing was saved:\n{e}", title="Save failed")
            continue
        if event == "-CHECK-":
            cb, rxv, errors = settings_from_values(values)
            if errors:
                sg.popup_error("\n".join(errors), title="Settings")
                continue
            window["-STATUS-"].update("Checking ...")
            window.start_thread(lambda: window.write_event_value("-CHECKED-", check(cb, rxv)), "-THREAD_END-")
            continue
        if event == "-CHECKED-":
            window["-RX_STATUS-"].update(values[event])
            window["-STATUS-"].update("Checked (see Receiver).")
            continue
        if event == "-START-":
            try:
                cb, rxv = save_settings(values)
            except (OSError, ValueError) as e:
                sg.popup_error(f"Not started, nothing was saved:\n{e}", title="Invalid settings")
                continue
            stop.clear()
            stats_lines = 0
            window["-RX_STATUS-"].update("Starting ...")
            window["-CF-"].update("")
            set_running(window, True)
            window.start_thread(lambda: rx_worker(post, cb, rxv, stop), "-THREAD_END-")
            continue
        if event == "-STOP-":
            stop.set()
            window["-STOP-"].update(disabled=True)
            window["-STATUS-"].update("Stopping ...")
            continue
        if event == "-RX_EVENT-":
            ev = values[event]
            if ev["kind"] == "started":
                append(ev["text"])
                stats_lines = 0
                window["-STATUS-"].update("Receiving ... (Stop ends it)")
            elif ev["kind"] == "cf":
                window["-CF-"].update(ev["text"])
            elif ev["kind"] == "stats":
                lines = window["-RX_STATUS-"].get().rstrip("\n").split("\n")
                if stats_lines:
                    lines = lines[:-stats_lines]
                new = ev["text"].split("\n")
                window["-RX_STATUS-"].update("\n".join(lines + new))
                stats_lines = len(new)
            else:                                   # done
                set_running(window, False)
                stats_lines = 0
                if ev["error"]:
                    append(f"ERROR: {ev['error']}")
                    window["-STATUS-"].update("Stopped with an error (see Receiver).")
                else:
                    window["-STATUS-"].update("Stopped.")
                if ev["summary"]:
                    append("Final: " + ev["summary"])
            continue

    window.close()


if __name__ == "__main__":
    main()
