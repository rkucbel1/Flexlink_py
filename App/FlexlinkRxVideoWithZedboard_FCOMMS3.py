"""FlexlinkRxVideoWithZedboard_FCOMMS3.py -- receives the live video of FlexlinkTxVideoWithPluto.py and plays it (GUI).

The mirror of the transmitter app, with an AD-FMCOMMS3 (AD9361) on the Zedboard's FMC connector as
the radio. The Zedboard's FPGA receiver decodes the packets and pushes the transport blocks to this
PC as they come (HW_Development/Packet_Detector_FCOMM: bitstream build_hw_fmcomms3.tcl, firmware
fmc_pd_server.elf). For the CoreBoard as the radio: FlexlinkRxVideoWithZedboard_CoreBoard.py.

    Pluto  ~~radio~~>  FMCOMMS3 --40 Msps--> Zedboard (AD9361 -> DDR ring -> FPGA receiver)
        --TCP: transport blocks, CRC flags-->  this PC  --unpack, MPEG-TS-->  ffplay

The Zedboard needs no Flexlink parameters: it reads modulation, code and repetition from each
packet's control field. The Transmitter line shows that control field (the transmitter's mode).

  Settings    App/Parameters_FMCOMMS3.json: the Zedboard's address and port, the AD9361's carrier
              (the Pluto's TxLO_MHz), sample rate, RF bandwidth, gain control (fast attack by default:
              Flexlink is TDD, so the signal comes in bursts), how often the board pushes
              results, the DC blocker (6: without it, spurs near 0 Hz cause false triggers)
  Video       play with ffplay (sudo apt install ffmpeg); optionally also save the received
              stream to App/Video/Received.ts (about 3 GB an hour at 7 Mbit/s: off by default)

The blocks are put back together as in Flexlink_BasicModules/Video_Transfer/Video_Framing.py: a
block whose CRC failed, or a block missing from the count, is a gap; the broken TS packet is
dropped and the stream re-aligns, and the decoder recovers with the encoder's intra refresh.

Start saves the settings first (as Save does). Start the transmitter app before or after.

Requires: pip install FreeSimpleGUI; ffplay (ffmpeg) for the picture
Run with: python App/FlexlinkRxVideoWithZedboard_FCOMMS3.py
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

APP_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(APP_DIR))
import Flexlinksimulation as FS  # noqa: E402  (fonts, theme)
from flexlink_paths import APP, BASIC_MODULES, ROOT  # noqa: E402

import FreeSimpleGUI as sg  # noqa: E402

PD_PC = ROOT / "HW_Development" / "Packet_Detector" / "PC"
sys.path.insert(0, str(PD_PC))
import pd_protocol as pr  # noqa: E402
from pd_hw_test import Board  # noqa: E402

FMC_PC = ROOT / "HW_Development" / "Packet_Detector_FCOMM" / "PC"
sys.path.insert(0, str(FMC_PC))
import fmc_protocol as fp  # noqa: E402  (RADIO_CONFIG, BOARD_FMCOMMS3)

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

FMC_JSON = APP / "Parameters_FMCOMMS3.json"
RECEIVED_TS = APP / "Video" / "Received.ts"
RX_PARAMS = PD_PC.parent / "TestVectors" / "rep3.json"       # detector thresholds, as pd_stream_test.py
SLOT_SAMPLES_NOTE = "the push header's newest position counts input samples (fmc_pd_server.c)"

# ---------- Settings ----------
FMC_DEFAULTS = {"ZedAddress": "10.42.0.101", "ZedPort": pr.DEFAULT_PORT, "Frequency_MHz": 2420.0,
                "SampleRate": 40_000_000, "RFBandwidth_MHz": 18.0, "GainMode": "fast_attack", "Gain": 50,
                "Port": 0, "Push_ms": 1.0, "DcShift": 6, "Play": 1, "SaveStream": 0}
FMC_COMMENTS = {
    "ZedAddress_comment": "IP address of the Zedboard receiver",
    "ZedPort_comment": "its TCP port for commands and results",
    "Frequency_MHz_comment": "AD9361 receive carrier, MHz, 70 to 6000 (match TxLO_MHz of the Pluto)",
    "SampleRate_comment": "40000000 (receiver input stage 2, proven) or 20000000 (input stage 1)",
    "RFBandwidth_MHz_comment": "AD9361 analog RF bandwidth, MHz (the signal occupies 16.4 MHz)",
    "GainMode_comment": "fast_attack (default: TDD bursts), slow_attack, hybrid or manual (the AD9361's gain control)",
    "Gain_comment": "manual gain only, dB (-3 to 71); 50 over the air at a few metres",
    "Port_comment": "AD9361 Rx input: 0 = A balanced (the FMCOMMS3's RX1A SMA)",
    "Push_ms_comment": "the board pushes results at most this often while there are new ones, 0.25 to 7.75 ms",
    "DcShift_comment": "receiver DC blocker: 6 (0 gives false triggers from spurs near 0 Hz)",
    "Play_comment": "1: play the received video with ffplay",
    "SaveStream_comment": "1: also save the received stream to App/Video/Received.ts (large)",
}
FMC_FIELDS = {   # gui key: (json key, type, min, max, label)
    "-ZED_ADDR-": ("ZedAddress", str, None, None, "Zedboard Address"),
    "-ZED_PORT-": ("ZedPort", int, 1, 65535, "TCP Port"),
    "-ZED_PUSH-": ("Push_ms", float, 0.25, 7.75, "Push interval (ms)"),
    "-ZED_DC-": ("DcShift", int, 0, 15, "DC shift"),
    "-AD_FREQ-": ("Frequency_MHz", float, 70, 6000, "Carrier (MHz)"),
    "-AD_BW-": ("RFBandwidth_MHz", float, 0.2, 56, "RF bandwidth (MHz)"),
    "-AD_GAIN-": ("Gain", int, -3, 71, "Gain (dB)"),
}
RATES = {"40 Msps": 40_000_000, "20 Msps": 20_000_000}
LW = 18


def in_row(label, key, default, size=14, extra=None):
    row = [sg.Text(label, size=(LW, 1), justification="right"),
           sg.Input(str(default), key=key, size=size, justification="right")]
    return row + ([sg.Text(extra, font=ITALIC)] if extra else [])


ad_frame = sg.Frame("FMCOMMS3 (AD9361)", [
    in_row("Carrier (MHz)", "-AD_FREQ-", FMC_DEFAULTS["Frequency_MHz"], extra="= the Pluto's carrier"),
    [sg.Text("Sample rate", size=(LW, 1), justification="right"),
     sg.Combo(list(RATES), default_value="40 Msps", key="-AD_RATE-", readonly=True, size=12),
     sg.Text("40: proven path", font=ITALIC)],
    in_row("RF bandwidth (MHz)", "-AD_BW-", FMC_DEFAULTS["RFBandwidth_MHz"], extra="signal 16.4 MHz"),
    [sg.Text("Gain control", size=(LW, 1), justification="right"),
     sg.Combo(list(fp.GAIN_MODES), default_value="fast_attack", key="-AD_MODE-", readonly=True, size=12)],
    in_row("Gain (dB)", "-AD_GAIN-", FMC_DEFAULTS["Gain"], extra="manual only"),
    [sg.Text("RX1A", font=ITALIC)],
], font=H2, vertical_alignment="top", expand_x=True)

zed_frame = sg.Frame("Zedboard receiver", [
    in_row("Zedboard Address", "-ZED_ADDR-", FMC_DEFAULTS["ZedAddress"], size=16),
    in_row("TCP Port", "-ZED_PORT-", FMC_DEFAULTS["ZedPort"]),
    in_row("Push interval (ms)", "-ZED_PUSH-", FMC_DEFAULTS["Push_ms"], extra="0.25 .. 7.75"),
    in_row("DC shift", "-ZED_DC-", FMC_DEFAULTS["DcShift"], extra="6"),
    [sg.Button("Check", key="-CHECK-", size=10), sg.Text("Zedboard firmware, AD9361 status", font=ITALIC)],
], font=H2, vertical_alignment="top", expand_x=True)

video_frame = sg.Frame("Video", [
    [sg.Checkbox("Play with ffplay", key="-PLAY-", default=True)],
    [sg.Checkbox("Save the received stream", key="-SAVE_TS-")],
    [sg.Text(f"   to App/Video/{RECEIVED_TS.name} (about 3 GB an hour at 7 Mbit/s)", font=ITALIC)],
], font=H2, vertical_alignment="top", expand_x=True)

layout = [
    [sg.Text("Flexlink - Video Receiver (FMCOMMS3 + Zedboard)", font=H1, justification="center",
             expand_x=True, pad=(0, 15))],
    [sg.Push(), sg.Column([[ad_frame]], vertical_alignment="top"),
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


def settings_from_values(values) -> tuple[dict, list[str]]:
    s, errors = parse_fields(values, FMC_FIELDS)
    s["SampleRate"] = RATES.get(values["-AD_RATE-"], 40_000_000)
    s["GainMode"] = values["-AD_MODE-"] if values["-AD_MODE-"] in fp.GAIN_MODES else "fast_attack"
    s["Play"] = int(bool(values["-PLAY-"]))
    s["SaveStream"] = int(bool(values["-SAVE_TS-"]))
    return s, errors


def load_into_window(window) -> None:
    s = load_section(FMC_JSON, "FMCOMMS3", FMC_DEFAULTS, FMC_COMMENTS)["FMCOMMS3"]
    for gui_key, (key, *_r) in FMC_FIELDS.items():
        window[gui_key].update(str(s[key]))
    window["-AD_RATE-"].update(next((k for k, v in RATES.items() if v == int(s["SampleRate"])), "40 Msps"))
    window["-AD_MODE-"].update(s["GainMode"] if s["GainMode"] in fp.GAIN_MODES else "fast_attack")
    window["-PLAY-"].update(bool(int(s["Play"])))
    window["-SAVE_TS-"].update(bool(int(s["SaveStream"])))


def save_settings(values) -> dict:
    """Writes App/Parameters_FMCOMMS3.json (the fields shown; the others are kept).
    Raises ValueError, nothing written."""
    s, errors = settings_from_values(values)
    if errors:
        raise ValueError("\n".join(errors))
    d = load_section(FMC_JSON, "FMCOMMS3", FMC_DEFAULTS, FMC_COMMENTS)
    d["FMCOMMS3"].update(s)
    write_json_atomic(FMC_JSON, d)
    return d["FMCOMMS3"]


def radio_config(s: dict) -> fp.RadioConfig:
    return fp.RadioConfig(lo_hz=int(round(float(s["Frequency_MHz"]) * 1e6)), rate_hz=int(s["SampleRate"]),
                          bw_hz=int(round(float(s["RFBandwidth_MHz"]) * 1e6)),
                          gain_mode=fp.GAIN_MODES[s["GainMode"]], gain_db=int(s["Gain"]), port=int(s["Port"]))


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

    def add(self, now20, cfs, tbs) -> None:
        """now20: the stream position in receiver samples (20 Msps) when the push left the board,
        or None (the STREAM_STOP reply)."""
        for c in cfs:
            if c.crc_ok and c.message != self.cf_message:
                self.cf_message = c.message
                self.cf_text = (cf_fields.describe(c.message) if cf_fields is not None
                                else f"control field message 0x{c.message:014x}")
        if not tbs:
            return
        k = 0
        while k < len(tbs):                         # runs of blocks of the same size
            nb = len(tbs[k][3])
            j = k
            while j < len(tbs) and len(tbs[j][3]) == nb:
                t = tbs[j][0]
                if t != self.last_trig:
                    self.packets += 1
                    self.last_trig = t
                    if now20 is not None:
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
            line += (f"\nFMCOMMS3 ring: {st.datagrams} slots written, {st.lost} overflows (samples lost), "
                     f"{st.ring_full} ring full, ring max {st.ring_max} slots, {st.stalls} stalls")
        if self.out is not None and self.out.note:
            line += f"\nvideo: {self.out.note}"
        return line


def detector_params(dc_shift: int, rate: int) -> pr.Params:
    params = pr.Params()
    if RX_PARAMS.is_file():
        meta = json.loads(RX_PARAMS.read_text())
        known = {f.name for f in fields(pr.Params)}
        params = pr.Params(**{k: v for k, v in meta.get("params", {}).items() if k in known})
    params.input_stage = 2 if rate == 40_000_000 else 1   # 40 Msps: half-band on the board
    params.dc_shift = int(dc_shift)
    return params


def board_ok(info) -> str | None:
    if not info.flags & fp.BOARD_FMCOMMS3:
        return "not the FMCOMMS3 firmware (load fmc_pd_server.elf)"
    if info.flags & pr.BOARD_NO_DETECTOR:
        return "no receiver in the bitstream (the step-4 build_hw_fmcomms3.tcl bitstream is needed)"
    return None


def check(s: dict) -> str:
    try:
        b = Board(s["ZedAddress"], int(s["ZedPort"]), 5.0)
        info = b.ping()
        rs = fp.RadioStatus.unpack(b.call(fp.RADIO_CONFIG))
        b.sock.close()
        bad = board_ok(info)
        return (f"Zedboard {s['ZedAddress']}: fmc_pd_server build {info.build}, flags 0x{info.flags:X}"
                + (f" - {bad}" if bad else "") + "\n" + rs.describe())
    except (OSError, pr.ProtocolError) as e:
        return f"Zedboard {s['ZedAddress']}:{s['ZedPort']}: no answer ({e})"


# ---------- The session (worker thread) ----------
def rx_worker(post, s: dict, stop: threading.Event) -> None:
    """post(kind, **kw): "started" | "stats" | "cf" | "done"."""
    error = None
    b = out = rx = None
    streaming = False
    st = None
    t0 = time.monotonic()
    try:
        out = VF.TsOutput(play=bool(s["Play"]), save_path=RECEIVED_TS if s["SaveStream"] else None,
                          title="Flexlink received video")
        rx = VideoRx(out)
        b = Board(s["ZedAddress"], int(s["ZedPort"]), 10.0)
        info = b.ping()
        bad = board_ok(info)
        if bad:
            raise RuntimeError(f"Zedboard flags 0x{info.flags:X}: {bad}")
        rate = int(s["SampleRate"])
        decim = 2 if rate == 40_000_000 else 1          # input samples per receiver (20 Msps) sample
        rs = fp.RadioStatus.unpack(b.call(fp.RADIO_CONFIG, radio_config(s).pack()))
        if not rs.ok:
            raise RuntimeError("the AD9361 is not set up: " + rs.describe())
        b.call(pr.SET_PARAMS, detector_params(s["DcShift"], rate).pack())
        push_units = min(max(round(float(s["Push_ms"]) * 4), 1), 31)
        flags = pr.STREAM_FLAG_PUSH | (push_units << 2)
        b.call(pr.STREAM_START, struct.pack("<HHIHH", 0, flags, 0, 0, 0))   # only the flags are used
        streaming = True
        post("started", text=(f"Zedboard {s['ZedAddress']}: fmc_pd_server build {info.build}; " + rs.describe()
                              + f"\nreceiver input stage {2 if decim == 2 else 1}, dc_shift {s['DcShift']}; "
                              f"results pushed at most every {push_units / 4:g} ms"))
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
            rx.add((slot // decim) & 0xFFFFFFFF, cfs, tbs)
            now = time.monotonic()
            if rx.cf_text != cf_shown:
                cf_shown = rx.cf_text
                post("cf", text=cf_shown)
            if now - t_post > 0.5:
                t_post = now
                post("stats", text=rx.stats_line(now - t0, st))
        # STREAM_STOP: the pushes already on their way, then the reply (the last results)
        b.seq += 1
        pr.send_msg(b.sock, pr.STREAM_STOP, struct.pack("<H", 0), b.seq)
        while True:
            cmd, status, seq, raw = pr.recv_msg(b.sock)
            if cmd == pr.STREAM_PUSH:
                slot, st_new, cfs, tbs = parse_push(raw)
                st = st_new or st
                rx.add((slot // decim) & 0xFFFFFFFF, cfs, tbs)
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
    window = sg.Window("Flexlink Video Rx (FMCOMMS3)", layout, finalize=True, resizable=True)
    window.metadata = False
    stop = threading.Event()
    stats_lines = 0
    try:
        load_into_window(window)
        window["-STATUS-"].update("Loaded Parameters_FMCOMMS3.json")
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
                window["-STATUS-"].update("Saved Parameters_FMCOMMS3.json.")
            except (OSError, ValueError) as e:
                sg.popup_error(f"Nothing was saved:\n{e}", title="Save failed")
            continue
        if event == "-CHECK-":
            s, errors = settings_from_values(values)
            if errors:
                sg.popup_error("\n".join(errors), title="Settings")
                continue
            window["-STATUS-"].update("Checking ...")
            window.start_thread(lambda: window.write_event_value("-CHECKED-", check(s)), "-THREAD_END-")
            continue
        if event == "-CHECKED-":
            window["-RX_STATUS-"].update(values[event])
            window["-STATUS-"].update("Checked (see Receiver).")
            continue
        if event == "-START-":
            try:
                s = save_settings(values)
            except (OSError, ValueError) as e:
                sg.popup_error(f"Not started, nothing was saved:\n{e}", title="Invalid settings")
                continue
            stop.clear()
            stats_lines = 0
            window["-RX_STATUS-"].update("Starting ...")
            window["-CF-"].update("")
            set_running(window, True)
            window.start_thread(lambda: rx_worker(post, s, stop), "-THREAD_END-")
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
