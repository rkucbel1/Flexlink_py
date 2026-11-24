"""FlexlinkTxMavlinkWithPluto.py -- live video and MAVLink telemetry over the Flexlink transmitter
on an ADALM-Pluto (GUI).

The video app (FlexlinkTxVideoWithPluto.py) with a second stream: MAVLink telemetry. Both share
the transport blocks of every packet:

    Raspberry Pi (ffmpeg + x264) --MPEG-TS over UDP-->  this PC  --transport blocks-->  Pluto
    MAVLink source (.tlog / UDP / serial)  -->  this PC  --^

  MAVLink source  File: a .tlog replayed at its recorded pace and looped (for tests:
                  App/Mavlink/Sim_Telemetry.tlog from App/Mavlink/Generate_Sim_Telemetry.py).
                  UDP: MAVLink datagrams sent to this PC, e.g. by mavlink-router on the Pi with
                  the flight controller on its UART:  mavlink-routerd -e <PC address>:14560 /dev/serial0:921600
                  Serial: a flight controller's telemetry port on this PC (pip install pyserial).
  Sharing         every packet first takes up to "Max blocks per packet" MAVLink blocks (only as
                  many as there are frames waiting), then video. MAVLink frames waiting longer than
                  "Max age" are dropped. Keep the video share below 100 % minus the MAVLink share
                  the plan shows. Block format: App/Mavlink/Mavlink_Framing.py (magic 0xF3); the
                  video receiver skips these blocks, so it plays the video as before.
  Send video      off: MAVLink only (no Pi needed); the rest of every packet is video filler.
  Loopback        no Pluto: the packets are unpacked on this PC (optionally with a share of blocks
                  dropped, as failed CRCs); the video plays with ffplay, the MAVLink frames are sent
                  by UDP to a ground station (default 127.0.0.1:14550: open QGroundControl or
                  Mission Planner on this PC and the simulated vehicle shows on the map) and saved
                  to App/Mavlink/Loopback_Received.tlog.

The link is one way: the ground station sees the telemetry but its commands and parameter
requests do not reach the vehicle (QGroundControl may report that it cannot load parameters).

Start saves the settings first (as Save does): those of the video app and
App/Mavlink/Parameters_Mavlink.json.

Requires: as FlexlinkTxVideoWithPluto.py; pyserial for the serial source only.
Run with: python App/Tx_Pluto/FlexlinkTxMavlinkWithPluto.py
"""
from __future__ import annotations

import json
import threading
import time
from collections import deque
from pathlib import Path
import sys

import numpy as np

APP_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(APP_DIR))
import FlexlinkTxVideoWithPluto as TXV  # noqa: E402  (video settings, the Pi encoder, the video plan)

TXP, VF, FS, sg = TXV.TXP, TXV.VF, TXV.FS, TXV.sg
from flexlink_paths import APP, ROOT  # noqa: E402

MAV_DIR = APP / "Mavlink"
sys.path.insert(0, str(MAV_DIR))
import Mavlink_Lite as ML  # noqa: E402
import Mavlink_Framing as MF  # noqa: E402

H1, H2, ITALIC, RESULTS_FONT = TXV.H1, TXV.H2, TXV.ITALIC, TXV.RESULTS_FONT
MAV_JSON = MAV_DIR / "Parameters_Mavlink.json"
LOOPBACK_TLOG = MAV_DIR / "Loopback_Received.tlog"
LW = TXV.LW

# ---------- MAVLink settings (App/Mavlink/Parameters_Mavlink.json) ----------
MAV_FIELDS = {
    "-MAV_FILE-": ("TlogFile", str, None, None, "MAVLink .tlog file"),
    "-MAV_UDP_PORT-": ("UdpPort", int, 1024, 65535, "MAVLink UDP port"),
    "-MAV_SERIAL-": ("SerialPort", str, None, None, "Serial port"),
    "-MAV_BAUD-": ("SerialBaud", int, 1200, 3000000, "Baud rate"),
    "-MAV_BLOCKS-": ("MaxBlocksPerPacket", int, 1, 64, "Max blocks per packet"),
    "-MAV_AGE-": ("MaxAge_ms", int, 50, 10000, "Max age (ms)"),
    "-MAV_FWD-": ("LoopbackForward", str, None, None, "Loopback: send to"),
}
MAV_SOURCES = {"-MAV_SRC_FILE-": "File", "-MAV_SRC_UDP-": "UDP", "-MAV_SRC_SERIAL-": "Serial"}
MAV_DEFAULTS = {"SendMavlink": 1, "SendVideo": 1, "Source": "File", "TlogFile": "App/Mavlink/Sim_Telemetry.tlog",
                "UdpPort": 14560, "SerialPort": "/dev/ttyUSB0", "SerialBaud": 57600, "MaxBlocksPerPacket": 2,
                "MaxAge_ms": 500, "LoopbackForward": "127.0.0.1:14550", "LoopbackSaveTlog": 1}
MAV_COMMENTS = {
    "SendMavlink_comment": "1: send the MAVLink stream",
    "SendVideo_comment": "1: send the video stream (Parameters_Video.json); 0: MAVLink only, no Pi needed",
    "Source_comment": "File (a .tlog, looped), UDP (datagrams to UdpPort on this PC) or Serial",
    "TlogFile_comment": "File source: a .tlog, relative to the project folder or absolute",
    "UdpPort_comment": "UDP source: port on this PC the MAVLink datagrams are sent to (e.g. by mavlink-router)",
    "SerialPort_comment": "Serial source: the flight controller's telemetry port (needs pyserial)",
    "SerialBaud_comment": "Serial source: baud rate (ArduPilot telemetry ports: 57600 by default)",
    "MaxBlocksPerPacket_comment": "most transport blocks of a packet given to MAVLink (it goes before the video)",
    "MaxAge_ms_comment": "MAVLink frames waiting longer than this are dropped",
    "LoopbackForward_comment": "loopback only: host:port the received frames are sent to (a ground station)",
    "LoopbackSaveTlog_comment": "loopback only: 1 saves the received frames to App/Mavlink/Loopback_Received.tlog",
}

# ---------- Layout: the video app's frames (same keys), rebuilt, and the MAVLink frame ----------
TX_LABEL_W = FS.TX_LABEL_W
tx_param_frame = sg.Frame("Transmitter (Parameters_Tx_payload.json)", [
    FS.combo_row("LDPC Code Block Size", "-TX_LDPC_BLOCK-", [648, 1296, 1944], 1296),
    FS.combo_row("LDPC Encoding Rate", "-TX_LDPC_RATE-", ["1/2", "2/3", "3/4", "5/6"], "5/6"),
    FS.combo_row("Rate Matching (Repetition)", "-TX_REPETITION-", [1, 3, 5, 7, 11, 15, 19, 23, 31], 15),
    FS.combo_row("Payload Field Modulation", "-TX_PAYLOAD_MOD-", ["BPSK", "QPSK", "16QAM", "64QAM"], "QPSK"),
    FS.combo_row("Packet Length (Symbols)", "-TX_PACKET_LEN-", [37, 49, 73, 97, 121], 73),
    FS.combo_row("Reference Symbol Spacing", "-TX_REF_SYM_SPACING-", [1, 3, 6, 12], 3),
    FS.combo_row("Reference Signal Spacing", "-TX_REF_SIG_SPACING-", [3, 6, 12, 24], 3),
    [sg.Text("C_Init", size=(TX_LABEL_W, 1), justification="right"),
     sg.Input("130", key="-TX_C_INIT-", size=12, justification="right", pad=(5, 8))],
    FS.combo_row("Number of Tx Antennas", "-TX_NUM_ANT-", [1], 1, fixed=True),
    [sg.Text("", key="-TX_DERIVED-", font=ITALIC, size=(48, 2))],
], font=H2, vertical_alignment="top")

pluto_frame = sg.Frame("Pluto", [
    TXP.pl_row("Pluto Address", "-PL_ADDR-", TXP.PLUTO_DEFAULTS["Address"], size=16),
    TXP.pl_row("Server Port", "-PL_PORT-", TXP.PLUTO_DEFAULTS["ServerPort"]),
    TXP.pl_row("Carrier (MHz)", "-PL_LO-", TXP.PLUTO_DEFAULTS["TxLO_MHz"]),
    TXP.pl_row("Tx Gain (dB)", "-PL_TXGAIN-", TXP.PLUTO_DEFAULTS["TxGain_dB"], extra="-89.75 to 0"),
    TXP.pl_row("RF Bandwidth (MHz)", "-PL_BW-", TXP.PLUTO_DEFAULTS["RfBandwidth_MHz"]),
    TXP.pl_row("Gap (samples)", "-PL_GAP-", TXP.PLUTO_DEFAULTS["Gap"], extra="idle after each packet"),
    [sg.Text("40 Msps, set by the Pluto's transmitter server", font=ITALIC)],
], font=H2, vertical_alignment="top", expand_x=True)

v_row = TXV.v_row
VD = TXV.VIDEO_DEFAULTS
pi_frame = sg.Frame("Raspberry Pi (encoder)", [
    v_row("Pi Address", "-PI_ADDR-", VD["PiAddress"], size=18, extra="'local': this PC"),
    v_row("Pi User", "-PI_USER-", VD["PiUser"], size=18),
    v_row("PC Address (for the Pi)", "-PC_ADDR-", VD["PcAddress"], size=18),
    v_row("UDP Port", "-UDP_PORT-", VD["UdpPort"]),
    [sg.Text("Max resolution", size=(LW, 1), justification="right"),
     sg.Combo([f"{h}p" for h in TXV.MAX_HEIGHTS], default_value="540p", key="-MAX_HEIGHT-", size=8,
              readonly=True, enable_events=True, background_color="white", text_color="black"),
     sg.Text("Pi 4: 540p", font=ITALIC)],
    v_row("Video share (%)", "-VIDEO_SHARE-", VD["VideoShare_percent"], extra="of the payload rate"),
    v_row("Max queue (ms)", "-MAX_QUEUE-", VD["MaxQueue_ms"], extra="then oldest dropped"),
    [sg.Button("Check Pi", key="-CHECK_PI-", size=10),
     sg.Text("SSH login, ffmpeg, libx264, the video file", font=ITALIC)],
], font=H2, vertical_alignment="top", expand_x=True)

send_frame = sg.Frame("Transmission", [
    [sg.Checkbox("Send video", key="-VIDEO_ON-", default=True, enable_events=True)],
    [sg.Radio("Helheim video on the Pi (looped)", "VSRC", key="-SRC_FILE-", default=True, enable_events=True)],
    [sg.Text("File on the Pi", size=(12, 1), justification="right"),
     sg.Input(VD["PiVideoFile"], key="-PI_FILE-", size=30, enable_events=True)],
    [sg.Radio("Camera on the Pi (not implemented yet)", "VSRC", key="-SRC_CAMERA-", disabled=True)],
    [sg.Checkbox("Loopback on this PC: no Pluto, unpack, play the video,\nsend the MAVLink to a ground station",
                 key="-LOOPBACK-", enable_events=True)],
    v_row("Drop blocks (%)", "-LB_DROP-", VD["LoopbackDrop_percent"], extra="loopback: as failed CRCs"),
], font=H2, vertical_alignment="top", expand_x=True)

MD = MAV_DEFAULTS
MLW = 16


def m_row(label, key, default, size=10, extra=None):
    row = [sg.Text(label, size=(MLW, 1), justification="right"),
           sg.Input(str(default), key=key, size=size, justification="right", enable_events=True)]
    return row + ([sg.Text(extra, font=ITALIC)] if extra else [])


mav_frame = sg.Frame("MAVLink telemetry", [
    [sg.Checkbox("Send MAVLink", key="-MAV_ON-", default=True, enable_events=True)],
    [sg.Radio("File (.tlog, looped)", "MSRC", key="-MAV_SRC_FILE-", default=True, enable_events=True)],
    [sg.Input(MD["TlogFile"], key="-MAV_FILE-", size=34, enable_events=True),
     sg.FileBrowse("Browse", target="-MAV_FILE-", initial_folder=str(MAV_DIR),
                   file_types=(("MAVLink logs", "*.tlog"), ("All files", "*.*")))],
    [sg.Radio("UDP, port", "MSRC", key="-MAV_SRC_UDP-", enable_events=True),
     sg.Input(str(MD["UdpPort"]), key="-MAV_UDP_PORT-", size=8, justification="right", enable_events=True),
     sg.Text("e.g. mavlink-router", font=ITALIC)],
    [sg.Radio("Serial", "MSRC", key="-MAV_SRC_SERIAL-", enable_events=True),
     sg.Input(MD["SerialPort"], key="-MAV_SERIAL-", size=14, enable_events=True),
     sg.Text("baud"), sg.Input(str(MD["SerialBaud"]), key="-MAV_BAUD-", size=8, justification="right",
                               enable_events=True)],
    m_row("Max blocks / packet", "-MAV_BLOCKS-", MD["MaxBlocksPerPacket"], size=6, extra="sent before video"),
    m_row("Max age (ms)", "-MAV_AGE-", MD["MaxAge_ms"], size=6, extra="older frames dropped"),
    m_row("Loopback: send to", "-MAV_FWD-", MD["LoopbackForward"], size=16, extra="ground station"),
    [sg.Checkbox("Loopback: save Loopback_Received.tlog", key="-MAV_SAVE-", default=True)],
], font=H2, vertical_alignment="top", expand_x=True)

layout = [
    [sg.Text("Flexlink - Pluto Video + MAVLink Transmitter", font=H1, justification="center", expand_x=True,
             pad=(0, 15))],
    [sg.Push(), sg.Column([[tx_param_frame]], vertical_alignment="top"),
     sg.Column([[pluto_frame], [pi_frame]], vertical_alignment="top"),
     sg.Column([[send_frame], [mav_frame]], vertical_alignment="top"), sg.Push()],
    [sg.Push(), sg.Text("Link plan", size=(12, 1), justification="right"),
     sg.Multiline(size=(120, 4), key="-PLAN-", disabled=True, font=RESULTS_FONT), sg.Push()],
    [sg.Push(), sg.Text("Transmitter", size=(12, 1), justification="right"),
     sg.Multiline(size=(120, 10), key="-TX_STATUS-", disabled=True, autoscroll=True, font=RESULTS_FONT),
     sg.Push()],
    [sg.Push(), sg.Button("Save", key="-SAVE-", size=10), sg.Button("Start", key="-START-", size=10),
     sg.Button("Stop", key="-STOP-", size=10, disabled=True), sg.Push()],
    [sg.Text("", key="-STATUS-", size=(120, 1), justification="center")],
]


# ---------- Settings ----------
def load_mav() -> dict:
    try:
        data = json.loads(MAV_JSON.read_text(encoding="utf-8"))
    except FileNotFoundError:
        data = {}
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON in {MAV_JSON}: line {e.lineno}, col {e.colno}: {e.msg}")
    sec = data.setdefault("Mavlink", {})
    for k, v in MAV_DEFAULTS.items():
        sec.setdefault(k, v)
        if f"{k}_comment" in MAV_COMMENTS:
            sec.setdefault(f"{k}_comment", MAV_COMMENTS[f"{k}_comment"])
    return data


def tlog_path(text: str) -> Path:
    p = Path(text).expanduser()
    return p if p.is_absolute() else ROOT / p


def mav_from_values(values) -> tuple[dict, list[str]]:
    out, errors = TXV.parse_fields(values, MAV_FIELDS)
    out["SendMavlink"] = int(bool(values["-MAV_ON-"]))
    out["SendVideo"] = int(bool(values["-VIDEO_ON-"]))
    out["Source"] = next((s for k, s in MAV_SOURCES.items() if values.get(k)), "File")
    out["LoopbackSaveTlog"] = int(bool(values["-MAV_SAVE-"]))
    if not (out["SendMavlink"] or out["SendVideo"]):
        errors.append("Nothing to send: tick Send video and / or Send MAVLink")
    if out["SendMavlink"] and out["Source"] == "File" and out.get("TlogFile"):
        if not tlog_path(out["TlogFile"]).is_file():
            errors.append(f"MAVLink .tlog file not found: {tlog_path(out['TlogFile'])} "
                          "(make one with App/Mavlink/Generate_Sim_Telemetry.py)")
    if out.get("LoopbackForward"):
        try:
            MF.parse_host_port(out["LoopbackForward"])
        except ValueError as e:
            errors.append(f"Loopback: send to: host:port ({e})")
    return out, errors


def load_into_window(window) -> None:
    TXV.load_into_window(window)
    m = load_mav()["Mavlink"]
    for gui_key, (key, *_rest) in MAV_FIELDS.items():
        window[gui_key].update(str(m[key]))
    window["-MAV_ON-"].update(bool(int(m["SendMavlink"])))
    window["-VIDEO_ON-"].update(bool(int(m["SendVideo"])))
    window["-MAV_SAVE-"].update(bool(int(m["LoopbackSaveTlog"])))
    for k, s in MAV_SOURCES.items():
        if s == m["Source"]:
            window[k].update(True)


def save_settings(values) -> str:
    """The video app's files (TXV.save_settings) and App/Mavlink/Parameters_Mavlink.json.
    Raises ValueError, nothing written."""
    mav, errors = mav_from_values(values)
    if errors:
        raise ValueError("\n".join(errors))
    msg = TXV.save_settings(values)
    mdata = load_mav()
    mdata["Mavlink"].update(mav)
    TXP.write_json_atomic(MAV_JSON, mdata)
    return msg.rstrip(".") + ", Parameters_Mavlink.json."


# ---------- Link plan ----------
def mav_plan(cfg, plan: dict, mav: dict) -> dict:
    """What the MAVLink stream takes from the link."""
    chunk = MF.payload_bytes(cfg.tb_size)
    pkt_rate = 1.0 / plan["period"]
    nmax = min(int(mav["MaxBlocksPerPacket"]), cfg.ndb)
    out = {"max_blocks": nmax, "max_share": 100.0 * nmax / cfg.ndb,
           "max_bps": nmax * chunk * 8 * pkt_rate, "est_share": None}
    if mav["Source"] == "File":
        try:
            fps, bps, span = MF.tlog_rate(tlog_path(mav["TlogFile"]))
        except (OSError, ValueError):
            fps = bps = span = 0.0
        if fps:
            # frames that arrive within one packet share blocks; at most one block per frame
            blocks_s = min(max(bps / chunk, min(fps, pkt_rate)), nmax * pkt_rate)
            out.update(fps=fps, bps=bps * 8, span=span, est_share=100.0 * blocks_s / (cfg.ndb * pkt_rate))
    return out


def plan_text(cfg, plan: dict, vid: dict, mav: dict) -> str:
    lines = TXV.plan_text(cfg, plan).split("\n")
    if not mav["SendVideo"]:
        lines[1] = "Video: off (the blocks MAVLink does not use are video filler)"
    if mav["SendMavlink"]:
        mp = mav_plan(cfg, plan, mav)
        src = {"File": f"file {Path(mav['TlogFile']).name}", "UDP": f"UDP port {mav['UdpPort']}",
               "Serial": f"serial {mav['SerialPort']} at {mav['SerialBaud']} baud"}[mav["Source"]]
        line = (f"MAVLink: {src}; up to {mp['max_blocks']} of {cfg.ndb} blocks per packet "
                f"({mp['max_share']:.0f} % of the link, {mp['max_bps'] / 1e3:.0f} kbit/s), sent before the video")
        if mp["est_share"] is not None:
            line += (f"\n         the file averages {mp['fps']:.0f} frames/s, {mp['bps'] / 1e3:.1f} kbit/s "
                     f"({mp['span']:.0f} s, looped): about {mp['est_share']:.1f} % of the blocks")
            if mav["SendVideo"] and vid["VideoShare_percent"] + mp["est_share"] > 100:
                line += "  [video share + MAVLink over 100 %: lower the video share]"
        lines.append(line)
    else:
        lines.append("MAVLink: off")
    return "\n".join(lines)


def current_plan(values):
    """(cfg, plan, vid, mav) from the GUI values, or raises ValueError."""
    cfg, plan = TXV.current_plan(values)
    vid, _ = TXV.video_from_values(values)
    mav, errors = mav_from_values(values)
    if errors:
        raise ValueError("; ".join(errors))
    return cfg, plan, vid, mav


# ---------- MAVLink source ----------
def open_source(mav: dict, queue: MF.FrameQueue):
    if mav["Source"] == "File":
        return MF.TlogSource(tlog_path(mav["TlogFile"]), queue, loop=True)
    if mav["Source"] == "UDP":
        return MF.UdpSource(mav["UdpPort"], queue)
    return MF.SerialSource(mav["SerialPort"], mav["SerialBaud"], queue)


# ---------- Loopback sink: video to ffplay, MAVLink to a ground station ----------
class LoopbackSink:
    def __init__(self, drop_percent: float, video: bool, mav: dict | None, play: bool = True):
        self.drop = drop_percent / 100.0
        self.rng = np.random.default_rng()
        self.vu = VF.VideoUnpacker()
        self.resync = VF.TsResync()
        self.out = VF.TsOutput(play=play, save_path=TXV.LOOPBACK_TS, title="Flexlink loopback") if video else None
        self.mu = MF.MavlinkUnpacker() if mav else None
        self.summary = MF.TelemetrySummary()
        self.udp = self.tlog = None
        if mav:
            if mav["LoopbackForward"]:
                self.udp = MF.UdpSink(*MF.parse_host_port(mav["LoopbackForward"]))
            if mav["LoopbackSaveTlog"]:
                self.tlog = MF.TlogSink(LOOPBACK_TLOG)

    def packet(self, tbs: np.ndarray) -> None:
        n = tbs.shape[0]
        crc = self.rng.random(n) >= self.drop if self.drop > 0 else np.ones(n, bool)
        for gap, data in self.vu.feed(tbs, crc):
            ts = self.resync.push(data, gap)
            if self.out is not None:
                self.out.write(ts)
        if self.mu is not None:
            for f in self.mu.feed(tbs, crc):
                self.summary.add(f)
                if self.udp is not None:
                    self.udp.send(f.raw)
                if self.tlog is not None:
                    self.tlog.write(f.raw)

    def close(self) -> None:
        for x in (self.out, self.udp, self.tlog):
            if x is not None:
                x.close()

    def stats(self) -> str:
        t = self.vu.total
        parts = []
        if self.out is not None:
            parts.append(f"loopback video: {t.crc_fail} blocks dropped, {t.gaps} gaps, "
                         f"{self.resync.ts_packets} TS packets out, {self.resync.resyncs} resyncs"
                         + (f"; {self.out.note}" if self.out.note else ""))
        if self.mu is not None:
            dest = f", {self.udp.sent} sent to {self.udp.addr[0]}:{self.udp.addr[1]}" if self.udp else ""
            parts.append(f"loopback MAVLink: {self.mu.frames} frames from {self.mu.blocks} blocks, "
                         f"{self.mu.lost} blocks lost{dest}")
            parts.append(f"  telemetry: {self.summary.line()}")
        return "\n".join(parts)


# ---------- The session (worker thread) ----------
def tx_worker(post, pl: dict, vid: dict, mav: dict, stop: threading.Event, play: bool = True,
              max_seconds: float | None = None) -> None:
    """post(kind, **kw): "started" | "stats" | "log" | "done". Runs until Stop, an error or
    max_seconds (for tests)."""
    error = None
    c = enc = udp = sink = src = None
    fifo = mq = None
    sched = None
    send_video, send_mav = bool(mav["SendVideo"]), bool(mav["SendMavlink"])
    seq = 0
    n_sent = 0
    last = {}
    t0 = time.monotonic()
    try:
        cfg = TXP.tx_config(json.loads(TXP.TX_JSON.read_text())["Defined"])
        plan = TXV.video_plan(cfg, pl["Gap"], vid["VideoShare_percent"], vid["MaxHeight"])
        post("log", text=plan_text(cfg, plan, vid, mav))
        if send_video:
            fifo = VF.ByteFifo()
            udp = TXV.UdpIn(vid["UdpPort"], fifo)
        if send_mav:
            mq = MF.FrameQueue(max_age_s=mav["MaxAge_ms"] / 1e3)
            src = open_source(mav, mq)
        if vid["Loopback"]:
            ndb, tb_bits, period = cfg.ndb, cfg.tb_size, plan["period"]
            sink = LoopbackSink(vid["LoopbackDrop_percent"], send_video, mav if send_mav else None, play)
            post("started", text=f"Loopback on this PC: {ndb} blocks of {tb_bits} bits every {period * 1e3:.3f} ms"
                                 + (f"; video -> {TXV.LOOPBACK_TS}" if send_video else "")
                                 + (f"; MAVLink -> {mav['LoopbackForward'] or '(not sent)'}"
                                    + (f" and {LOOPBACK_TLOG.name}" if mav["LoopbackSaveTlog"] else "")
                                    if send_mav else ""))
        else:
            c, rep = TXP.connect(pl, cfg)
            ndb, tb_bits = rep["BlocksPerPacket"], rep["TransportBlockSize"]
            period = (rep["PacketSamples"] + rep["Gap"]) / TXV.FS_PLUTO
            post("started", text=(f"Pluto {pl['Address']}: {ndb} blocks of {tb_bits} bits per packet, "
                                  f"{rep['PacketSamples']} samples + gap {rep['Gap']}, {c.depth} credits, "
                                  f"carrier {pl['TxLO_MHz']:g} MHz, Tx gain {pl['TxGain_dB']:g} dB"))
        sched = MF.LinkScheduler(tb_bits, ndb, mav["MaxBlocksPerPacket"] if send_mav else 0)
        if send_video:
            enc = TXV.PiEncoder(vid, plan)
            post("log", text=("Encoder: " if not TXV.is_local(vid) else "Encoder (this PC): ")
                 + enc.cmd.split("exec ", 1)[1])
            enc.start()
        if src is not None:
            src.start()
            post("log", text=f"MAVLink source: {mav['Source']}"
                             + (f" {tlog_path(mav['TlogFile'])} ({len(src.records)} frames, looped)"
                                if mav["Source"] == "File" else ""))
        max_queue = int(vid["MaxQueue_ms"] / 1e3 * plan["capacity_bps"] / 8)
        t0 = time.monotonic()
        t_next = t0
        t_post = 0.0
        warned_v = warned_m = False
        rate_win = deque([(t0, 0, 0, 0)], maxlen=8)
        while not stop.is_set():
            if c is not None:                              # one packet per credit
                while len(c.outstanding) >= c.depth:
                    c._recv()
            else:                                          # loopback: the link's packet rate
                t_next += period
                dt = t_next - time.monotonic()
                if dt > 0:
                    time.sleep(dt)
                elif dt < -0.5:                            # far behind (e.g. a stall): do not catch up
                    t_next = time.monotonic()
            if fifo is not None:
                fifo.trim(max_queue)
            tbs = sched.next_packet(fifo, mq)
            if c is not None:
                c.send_packet(seq, tbs)
                del c.events[:]
                if c.sent:
                    n_sent += len(c.sent)
                    last = c.sent[-1]
                    del c.sent[:]
            else:
                sink.packet(tbs)
                n_sent += 1
            seq += 1
            now = time.monotonic()
            if enc is not None and not enc.running():
                tail = "\n".join(enc.lines) or "(no output)"
                raise RuntimeError(f"the encoder stopped:\n{tail}")
            if src is not None and src.error:
                raise RuntimeError(f"the MAVLink source stopped: {src.error}")
            if send_video and not warned_v and udp.first is None and now - t0 > 8:
                warned_v = True
                post("log", text=f"No video from the encoder after 8 s on UDP port {vid['UdpPort']}: check "
                                 f"PC Address ({vid['PcAddress']}) and the firewall. Encoder says: "
                                 + (" | ".join(enc.lines) or "nothing"))
            if src is not None and not warned_m and src.first is None and now - t0 > 5:
                warned_m = True
                post("log", text=f"No MAVLink from the {mav['Source']} source after 5 s"
                                 + (f" (UDP port {mav['UdpPort']}: is the sender pointed at this PC?)"
                                    if mav["Source"] == "UDP" else ""))
            if max_seconds is not None and now - t0 > max_seconds:
                break
            if now - t_post > 0.5:
                t_post = now
                rate_win.append((now, udp.bytes if udp else 0, src.bytes if src else 0, src.frames if src else 0))
                (ta, va, ma, fa), (tb, vb, mb, fb) = rate_win[0], rate_win[-1]
                span = max(tb - ta, 1e-9)
                line = f"{now - t0:6.1f} s  "
                if send_video:
                    q_ms = fifo.size() * 8 / plan["capacity_bps"] * 1e3
                    line += (f"video in {(vb - va) * 8 / span / 1e3:6.0f} kbit/s, queue {q_ms:4.0f} ms, "
                             f"dropped {fifo.dropped_chunks} | ")
                if src is not None:
                    line += (f"MAVLink in {(fb - fa) / span:4.0f} frames/s {(mb - ma) * 8 / span / 1e3:5.1f} kbit/s, "
                             f"queue {len(mq)}, expired {mq.expired} | ")
                line += (f"{seq} packets, {n_sent} on air; blocks: {sched.video_blocks} video, "
                         f"{sched.mav_blocks} MAVLink, {sched.filler_blocks} filler")
                if c is not None:
                    line += f", PL underflows {last.get('underflows', '-')}"
                else:
                    line += "\n" + sink.stats()
                if enc is not None and enc.lines:
                    line += f"\nencoder: {enc.lines[-1]}"
                post("stats", text=line)
    except Exception as e:  # noqa: BLE001  (shown in the GUI)
        error = f"{type(e).__name__}: {e}"
    finally:
        if enc is not None:
            enc.stop()
        if udp is not None:
            udp.close()
        if src is not None:
            src.close()
        final = None
        if c is not None:
            try:
                final = c.stop()
            except Exception as e:  # noqa: BLE001
                error = error or f"stopping: {e}"
            c.close()
        if sink is not None:
            sink.close()
    post("done", error=error, final=final, packets=seq, on_air=n_sent, seconds=time.monotonic() - t0,
         fill=sched.fill_ratio() if sched else 0.0,
         mav_blocks=sched.mav_blocks if sched else 0, mav_frames=mq.frames_out if mq is not None else 0,
         loopback=sink.stats() if sink is not None else "")


# ---------- Window ----------
RUN_LOCKED = ("-SAVE-", "-START-", "-CHECK_PI-", "-LOOPBACK-", "-VIDEO_ON-", "-MAV_ON-", "-MAV_SRC_FILE-",
              "-MAV_SRC_UDP-", "-MAV_SRC_SERIAL-")


def set_running(window, running: bool) -> None:
    window.metadata = running
    for k in RUN_LOCKED:
        window[k].update(disabled=running)
    window["-STOP-"].update(disabled=not running)


def update_plan(window, values) -> None:
    try:
        window["-PLAN-"].update(plan_text(*current_plan(values)))
        window["-TX_DERIVED-"].update(TXP.derived_text(values))
    except Exception as e:  # noqa: BLE001  (a hint only)
        window["-PLAN-"].update(f"(no plan: {e})")


def main():
    window = sg.Window("Flexlink Video + MAVLink Tx with Pluto", layout, finalize=True, resizable=True)
    window.metadata = False
    stop = threading.Event()
    stats_shown = 0
    try:
        load_into_window(window)
        window["-STATUS-"].update("Loaded Parameters_Tx_payload.json, Parameters_Pluto.json, Parameters_Video.json, "
                                  "Parameters_Mavlink.json")
    except (OSError, ValueError, KeyError) as e:
        sg.popup_error(f"Could not load the parameter files:\n{e}")
    values = window.read(timeout=0)[1]
    update_plan(window, values)
    last_values = dict(values)

    def post(kind, **kw):
        window.write_event_value("-TX_EVENT-", {"kind": kind, **kw})

    def append(text):
        cur = window["-TX_STATUS-"].get().rstrip("\n")
        window["-TX_STATUS-"].update(cur + ("\n" if cur.strip() else "") + text)

    while True:
        event, values = window.read(timeout=700)
        if event == sg.WIN_CLOSED:
            if window.metadata:
                stop.set()
                time.sleep(1.0)
            break
        if event == sg.TIMEOUT_EVENT:              # the plan follows the settings (combos send no events)
            if not window.metadata and values != last_values:
                last_values = dict(values)
                update_plan(window, values)
            continue
        if event == "-SAVE-":
            try:
                window["-STATUS-"].update(save_settings(values))
                update_plan(window, values)
            except (OSError, ValueError) as e:
                sg.popup_error(f"Nothing was saved:\n{e}", title="Save failed")
            continue
        if event == "-CHECK_PI-":
            vid, errors = TXV.video_from_values(values)
            if errors:
                sg.popup_error("\n".join(errors), title="Pi settings")
                continue
            window["-STATUS-"].update("Checking the Pi ...")
            window.start_thread(lambda: window.write_event_value("-PI_CHECKED-", TXV.check_pi(vid)), "-THREAD_END-")
            continue
        if event == "-PI_CHECKED-":
            window["-TX_STATUS-"].update(values[event])
            window["-STATUS-"].update("Pi checked (see Transmitter).")
            continue
        if event == "-START-":
            try:
                save_settings(values)
            except (OSError, ValueError) as e:
                sg.popup_error(f"Not started, nothing was saved:\n{e}", title="Invalid settings")
                continue
            pl = TXP.load_pluto()["Pluto"]
            vid = TXV.load_video()["Video"]
            mav = load_mav()["Mavlink"]
            stop.clear()
            stats_shown = 0
            window["-TX_STATUS-"].update("Starting ...")
            set_running(window, True)
            window.start_thread(lambda: tx_worker(post, pl, vid, mav, stop), "-THREAD_END-")
            continue
        if event == "-STOP-":
            stop.set()
            window["-STOP-"].update(disabled=True)
            window["-STATUS-"].update("Stopping ...")
            continue
        if event == "-TX_EVENT-":
            ev = values[event]
            if ev["kind"] == "started":
                append(ev["text"])
                window["-STATUS-"].update("Transmitting ... (Stop ends it)")
            elif ev["kind"] == "log":
                append(ev["text"])
                stats_shown = 0
            elif ev["kind"] == "stats":
                lines = window["-TX_STATUS-"].get().rstrip("\n").split("\n")
                if stats_shown:
                    lines = lines[:-stats_shown]
                window["-TX_STATUS-"].update("\n".join(lines + ev["text"].split("\n")))
                stats_shown = len(ev["text"].split("\n"))
            else:                                   # done
                set_running(window, False)
                stats_shown = 0
                if ev["error"]:
                    append(f"ERROR: {ev['error']}")
                    window["-STATUS-"].update("Stopped with an error (see Transmitter).")
                else:
                    window["-STATUS-"].update(f"Stopped: {ev['on_air']} packet(s) on air in {ev['seconds']:.1f} s.")
                append(f"Stopped: {ev['packets']} packet(s) pushed, {ev['on_air']} on air, payload "
                       f"{100 * ev['fill']:.0f} % used, {ev['mav_frames']} MAVLink frames in {ev['mav_blocks']} blocks"
                       + (f"\n{ev['loopback']}" if ev["loopback"] else "")
                       + ("\ntransmitter switched off" if (ev["final"] or {}).get("stopped") else ""))
            continue
        if event == "-THREAD_END-":
            continue

    window.close()


if __name__ == "__main__":
    main()
