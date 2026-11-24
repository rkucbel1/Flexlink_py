"""FlexlinkTxVideoWithPluto.py -- live video over the Flexlink transmitter on an ADALM-Pluto (GUI).

The same transmitter and Pluto settings as FlexlinkTxWithPluto.py (Tx/Parameters_Tx_payload.json,
App/Tx_Pluto/Parameters_Pluto.json); the transport blocks carry a live H.264 video stream instead of random
blocks or a file:

    Raspberry Pi 4 (ffmpeg + x264, started over SSH)
        --MPEG-TS over UDP-->  this PC  --transport blocks-->  Pluto (FW/ptx_server, PL transmitter)

  Raspberry Pi   where the encoder runs and where the PC sends the video to (App/Tx_Pluto/Parameters_Video.json).
                 The PC starts and stops ffmpeg on the Pi over SSH, so the Pi needs a key login once:
                 ssh-copy-id <user>@<Pi address>. Pi Address "local" runs the encoder on this PC
                 instead (ffmpeg with libx264 needed), for tests without the Pi.
  Transmission   the source: the Helheim video file on the Pi (looped), or the Pi camera (not
                 implemented yet). The encoder settings follow from the link: the transport block
                 payload rate of the Tx settings and the Gap, times the video share, picks the row of
                 the operating table (resolution, frame rate) and the x264 bit rate. x264 as in the
                 operating table: veryfast, zerolatency, no B-frames, intra refresh every 2 s, CBR with
                 a 2-frame buffer; SPS/PPS repeated with every frame, so a receiver started
                 after the transmitter still finds the stream's setup (repeat-headers).
  Loopback       no Pluto: the packets are paced at the link's packet rate, unpacked on this PC
                 (optionally with a share of blocks dropped, as failed CRCs) and played with ffplay;
                 the received stream is also saved to App/Tx_Pluto/Loopback/Loopback_Received.ts. This checks the
                 encoder, the framing and the decoder's recovery without the radio.

Every packet is filled with what the encoder has delivered when the packet is due (block format:
Flexlink_BasicModules/Video_Transfer/Video_Framing.py), so the link keeps its packet rate and the
video waits at most about one packet. If the encoder delivers more than the link carries, the
oldest video is dropped once the queue holds more than "Max queue" milliseconds.

Start saves the settings first (as Save does).

Requires: pip install FreeSimpleGUI; on the Pi: sudo apt install ffmpeg; for loopback: ffplay (ffmpeg)
Run with: python App/Tx_Pluto/FlexlinkTxVideoWithPluto.py
"""
from __future__ import annotations

import json
import shlex
import socket
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

APP_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(APP_DIR))
import FlexlinkTxWithPluto as TXP  # noqa: E402  (Tx and Pluto settings, connect, JSON helpers)

FS, R, ptx_client = TXP.FS, TXP.R, TXP.ptx_client
sg = TXP.sg
from flexlink_paths import APP, APP_TX_PLUTO, BASIC_MODULES  # noqa: E402

sys.path.insert(0, str(BASIC_MODULES / "Video_Transfer"))
import Video_Framing as VF  # noqa: E402

H1, H2, ITALIC, RESULTS_FONT = TXP.H1, TXP.H2, TXP.ITALIC, TXP.RESULTS_FONT
VIDEO_JSON = APP_TX_PLUTO / "Parameters_Video.json"
LOOPBACK_TS = APP_TX_PLUTO / "Loopback" / "Loopback_Received.ts"   # with the Pluto loopback files
FS_PLUTO = TXP.FS_PLUTO                         # 40 Msps
TS_OVERHEAD = 1.05                              # MPEG-TS headers on top of the x264 rate
PID_FILE = "/tmp/flexlink_venc.pid"             # on the Pi: the encoder's process id

# ---------- Operating table (FlexLink H.264 video operating parameters) ----------
# (video rate from, kbit/s; width; height; fps) - the row is the last one whose "from" the rate reaches
LADDER = [(64, 384, 216, 15), (150, 512, 288, 30), (700, 768, 432, 30), (1800, 960, 540, 30),
          (3500, 1280, 720, 30), (7000, 1920, 1080, 30)]
MAX_HEIGHTS = [216, 288, 432, 540, 720, 1080]

# ---------- Video settings (App/Tx_Pluto/Parameters_Video.json) ----------
VIDEO_FIELDS = {
    "-PI_ADDR-": ("PiAddress", str, None, None, "Pi Address"),
    "-PI_USER-": ("PiUser", str, None, None, "Pi User"),
    "-PC_ADDR-": ("PcAddress", str, None, None, "PC Address (for the Pi)"),
    "-UDP_PORT-": ("UdpPort", int, 1024, 65535, "UDP Port"),
    "-VIDEO_SHARE-": ("VideoShare_percent", float, 10.0, 100.0, "Video share (%)"),
    "-MAX_QUEUE-": ("MaxQueue_ms", int, 20, 5000, "Max queue (ms)"),
    "-PI_FILE-": ("PiVideoFile", str, None, None, "Video file on the Pi"),
    "-LB_DROP-": ("LoopbackDrop_percent", float, 0.0, 50.0, "Drop blocks (%)"),
}
VIDEO_DEFAULTS = {"PiAddress": "raspberrypi.local", "PiUser": "rkucbel3", "PcAddress": "10.42.0.1",
                  "UdpPort": 5600, "VideoShare_percent": 85.0, "MaxHeight": 540, "MaxQueue_ms": 300,
                  "Source": "PiFile", "PiVideoFile": "~/Helheim_orig.mov", "Loopback": 0,
                  "LoopbackDrop_percent": 0.0}
VIDEO_COMMENTS = {
    "PiAddress_comment": "IP address or host name of the Raspberry Pi; 'local' runs the encoder on this PC",
    "PiUser_comment": "user on the Pi; the PC logs in with an SSH key (ssh-copy-id user@pi once)",
    "PcAddress_comment": "this PC's address as the Pi sees it (the Pi sends the video there)",
    "UdpPort_comment": "UDP port on this PC for the encoded video (MPEG-TS)",
    "VideoShare_percent_comment": "share of the transport-block payload rate given to the video, %",
    "MaxHeight_comment": "largest resolution row of the operating table to use (Pi 4: 540)",
    "MaxQueue_ms_comment": "video waiting for the link beyond this is dropped (oldest first)",
    "Source_comment": "PiFile (the video file on the Pi, looped) or PiCamera (not implemented yet)",
    "PiVideoFile_comment": "path on the Pi (~/ = the Pi user's home)",
    "Loopback_comment": "1: no Pluto; the packets are unpacked on this PC and played with ffplay",
    "LoopbackDrop_percent_comment": "loopback only: share of blocks dropped as failed CRCs, %",
}

LW = 20                                          # label width

# The transmitter and Pluto frames: the same fields and keys as FlexlinkTxWithPluto.py's, built anew
# (a FreeSimpleGUI element can be placed in one layout only, so that app's frames cannot be reused).
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


def v_row(label, key, default, size=14, extra=None):
    row = [sg.Text(label, size=(LW, 1), justification="right"),
           sg.Input(str(default), key=key, size=size, justification="right", enable_events=True)]
    return row + ([sg.Text(extra, font=ITALIC)] if extra else [])


pi_frame = sg.Frame("Raspberry Pi (encoder)", [
    v_row("Pi Address", "-PI_ADDR-", VIDEO_DEFAULTS["PiAddress"], size=18, extra="'local': this PC"),
    v_row("Pi User", "-PI_USER-", VIDEO_DEFAULTS["PiUser"], size=18),
    v_row("PC Address (for the Pi)", "-PC_ADDR-", VIDEO_DEFAULTS["PcAddress"], size=18),
    v_row("UDP Port", "-UDP_PORT-", VIDEO_DEFAULTS["UdpPort"]),
    [sg.Text("Max resolution", size=(LW, 1), justification="right"),
     sg.Combo([f"{h}p" for h in MAX_HEIGHTS], default_value="540p", key="-MAX_HEIGHT-", size=8,
              readonly=True, enable_events=True, background_color="white", text_color="black"),
     sg.Text("Pi 4: 540p", font=ITALIC)],
    v_row("Video share (%)", "-VIDEO_SHARE-", VIDEO_DEFAULTS["VideoShare_percent"], extra="of the payload rate"),
    v_row("Max queue (ms)", "-MAX_QUEUE-", VIDEO_DEFAULTS["MaxQueue_ms"], extra="then oldest dropped"),
    [sg.Button("Check Pi", key="-CHECK_PI-", size=10),
     sg.Text("SSH login, ffmpeg, libx264, the video file", font=ITALIC)],
], font=H2, vertical_alignment="top", expand_x=True)

send_frame = sg.Frame("Transmission", [
    [sg.Radio("Helheim video on the Pi (looped)", "VSRC", key="-SRC_FILE-", default=True, enable_events=True)],
    [sg.Text("File on the Pi", size=(12, 1), justification="right"),
     sg.Input(VIDEO_DEFAULTS["PiVideoFile"], key="-PI_FILE-", size=30, enable_events=True)],
    [sg.Radio("Camera on the Pi (not implemented yet)", "VSRC", key="-SRC_CAMERA-", disabled=True)],
    [sg.Checkbox("Loopback on this PC: no Pluto, unpack and play with ffplay", key="-LOOPBACK-",
                 enable_events=True)],
    v_row("Drop blocks (%)", "-LB_DROP-", VIDEO_DEFAULTS["LoopbackDrop_percent"],
          extra="loopback: as failed CRCs"),
], font=H2, vertical_alignment="top", expand_x=True)

layout = [
    [sg.Text("Flexlink - Pluto Video Transmitter", font=H1, justification="center", expand_x=True, pad=(0, 15))],
    [sg.Push(), sg.Column([[tx_param_frame]], vertical_alignment="top"),
     sg.Column([[pluto_frame], [pi_frame]], vertical_alignment="top"),
     sg.Column([[send_frame]], vertical_alignment="top"), sg.Push()],
    [sg.Push(), sg.Text("Video plan", size=(12, 1), justification="right"),
     sg.Multiline(size=(110, 3), key="-PLAN-", disabled=True, font=RESULTS_FONT), sg.Push()],
    [sg.Push(), sg.Text("Transmitter", size=(12, 1), justification="right"),
     sg.Multiline(size=(110, 9), key="-TX_STATUS-", disabled=True, autoscroll=True, font=RESULTS_FONT),
     sg.Push()],
    [sg.Push(), sg.Button("Save", key="-SAVE-", size=10), sg.Button("Start", key="-START-", size=10),
     sg.Button("Stop", key="-STOP-", size=10, disabled=True), sg.Push()],
    [sg.Text("", key="-STATUS-", size=(110, 1), justification="center")],
]


# ---------- Settings ----------
def load_video() -> dict:
    try:
        data = json.loads(VIDEO_JSON.read_text(encoding="utf-8"))
    except FileNotFoundError:
        data = {}
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON in {VIDEO_JSON}: line {e.lineno}, col {e.colno}: {e.msg}")
    sec = data.setdefault("Video", {})
    for k, v in VIDEO_DEFAULTS.items():
        sec.setdefault(k, v)
        if f"{k}_comment" in VIDEO_COMMENTS:
            sec.setdefault(f"{k}_comment", VIDEO_COMMENTS[f"{k}_comment"])
    return data


def parse_fields(values, fields: dict) -> tuple[dict, list[str]]:
    out, errors = {}, []
    for gui_key, (key, typ, lo, hi, label) in fields.items():
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


PLUTO_KEYS = ("-PL_ADDR-", "-PL_PORT-", "-PL_LO-", "-PL_TXGAIN-", "-PL_BW-", "-PL_GAP-")


def pluto_from_values(values) -> tuple[dict, list[str]]:
    """The Pluto's radio settings (the fields this app shows) and a list of errors."""
    return parse_fields(values, {k: TXP.PLUTO_FIELDS[k] for k in PLUTO_KEYS})


def video_from_values(values) -> tuple[dict, list[str]]:
    out, errors = parse_fields(values, VIDEO_FIELDS)
    out["MaxHeight"] = int(str(values["-MAX_HEIGHT-"]).rstrip("p"))
    out["Source"] = "PiCamera" if values.get("-SRC_CAMERA-") else "PiFile"
    out["Loopback"] = int(bool(values["-LOOPBACK-"]))
    return out, errors


def load_into_window(window) -> None:
    data = FS.load_json("tx")
    for gui_key, (section, key), kind in FS.LINKS["tx"]:
        v = data.get(section, {}).get(key)
        if v is not None:
            FS.set_gui(window, gui_key, kind, v)
    p = TXP.load_pluto()["Pluto"]
    for gui_key in PLUTO_KEYS:
        window[gui_key].update(str(p[TXP.PLUTO_FIELDS[gui_key][0]]))
    v = load_video()["Video"]
    for gui_key, (key, *_rest) in VIDEO_FIELDS.items():
        window[gui_key].update(str(v[key]))
    window["-MAX_HEIGHT-"].update(f"{int(v['MaxHeight'])}p")
    window["-LOOPBACK-"].update(bool(int(v["Loopback"])))


def save_settings(values) -> str:
    """Parameters_Tx_payload.json (and the shared keys of Parameters_Rx_payload.json, as
    FlexlinkTxWithPluto does), App/Tx_Pluto/Parameters_Pluto.json (the radio fields only) and
    App/Tx_Pluto/Parameters_Video.json. Raises ValueError, nothing written."""
    tx, errors = TXP.tx_from_values(values)
    pl, e2 = pluto_from_values(values)
    vid, e3 = video_from_values(values)
    errors += e2 + e3
    if vid.get("Source") == "PiCamera":
        errors.append("The Pi camera is not implemented yet")
    if errors:
        raise ValueError("\n".join(errors))
    rx = FS.load_json("rx")
    rx_def = rx.setdefault("Defined_Rx", {})
    for key in FS.TX_RX_SHARED:
        if key in tx.get("Defined", {}):
            rx_def[key] = tx["Defined"][key]
    pdata = TXP.load_pluto()
    pdata["Pluto"].update(pl)
    vdata = load_video()
    vdata["Video"].update(vid)
    FS.save_json("tx", tx)
    FS.save_json("rx", rx)
    TXP.write_json_atomic(TXP.PLUTO_JSON, pdata)
    TXP.write_json_atomic(VIDEO_JSON, vdata)
    return "Saved Parameters_Tx_payload.json, Parameters_Rx_payload.json, Parameters_Pluto.json, Parameters_Video.json."


# ---------- Video plan: link rate -> table row -> encoder settings ----------
def video_plan(cfg, gap: int, share_percent: float, max_height: int) -> dict:
    gap16 = (int(gap) + 15) // 16 * 16
    period = (cfg.samples + gap16) / FS_PLUTO
    cap = VF.capacity_bps(cfg.tb_size, cfg.ndb, period)            # stream bits/s, every block full
    video_bps = cap * share_percent / 100.0
    kbps = video_bps / 1e3

    def pick(rows):
        row = rows[0]
        for r in rows:
            if kbps >= r[0]:
                row = r
        return row

    best = pick(LADDER)
    row = pick([r for r in LADDER if r[2] <= max_height] or LADDER[:1])
    x264_kbps = max(16, int(kbps / TS_OVERHEAD))
    _, w, h, fps = row
    return {"period": period, "gap": gap16, "capacity_bps": cap, "video_bps": video_bps,
            "x264_kbps": x264_kbps, "width": w, "height": h, "fps": fps,
            "below_table": kbps < LADDER[0][0], "capped": row != best}


def plan_text(cfg, plan: dict) -> str:
    lines = [f"Link: {cfg.ndb} blocks of {cfg.tb_size} bits every {plan['period'] * 1e3:.3f} ms "
             f"(packet {cfg.samples} + gap {plan['gap']} samples) = "
             f"{cfg.ndb * cfg.tb_size / plan['period'] / 1e6:.3f} Mbit/s of transport blocks, "
             f"{plan['capacity_bps'] / 1e6:.3f} Mbit/s of video payload ({VF.payload_bytes(cfg.tb_size)} bytes per block)",
             f"Video: {plan['video_bps'] / 1e3:.0f} kbit/s -> {plan['width']}x{plan['height']} at {plan['fps']} fps, "
             f"x264 {plan['x264_kbps']} kbit/s (TS headers on top)"
             + ("  [capped by Max resolution]" if plan["capped"] else "")
             + ("  [below the table's lowest row: expect a poor picture]" if plan["below_table"] else "")]
    return "\n".join(lines)


def current_plan(values):
    """(cfg, plan) from the GUI values, or raises ValueError."""
    tx, errors = TXP.tx_from_values(values)
    pl, e2 = pluto_from_values(values)
    vid, e3 = video_from_values(values)
    errors += e2 + e3
    if errors:
        raise ValueError("; ".join(errors))
    cfg = TXP.tx_config(tx["Defined"])
    return cfg, video_plan(cfg, pl["Gap"], vid["VideoShare_percent"], vid["MaxHeight"])


# ---------- The encoder on the Pi (over SSH) ----------
def remote_path(p: str) -> str:
    return '"$HOME"/' + shlex.quote(p[2:]) if p.startswith("~/") else shlex.quote(p)


def is_local(vid: dict) -> bool:
    return vid["PiAddress"].strip().lower() == "local"


def ssh_cmd(vid: dict, remote: str) -> list[str]:
    if is_local(vid):
        return ["sh", "-c", remote]
    return ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", "-o", "StrictHostKeyChecking=accept-new",
            f"{vid['PiUser']}@{vid['PiAddress']}", remote]


def encoder_command(vid: dict, plan: dict) -> str:
    """The shell command that runs ffmpeg + x264 on the Pi (settings of the operating table)."""
    k, fps = plan["x264_kbps"], plan["fps"]
    dest = "127.0.0.1" if is_local(vid) else vid["PcAddress"]
    src = remote_path(vid["PiVideoFile"])
    args = (f"ffmpeg -hide_banner -loglevel warning -nostdin -re -stream_loop -1 -i {src} -an -sn -dn "
            f"-vf fps={fps},scale={plan['width']}:{plan['height']}:flags=bicubic,format=yuv420p "
            f"-c:v libx264 -preset veryfast -tune zerolatency -b:v {k}k -maxrate {k}k -bufsize {max(1, 2 * k // fps)}k "
            f"-x264-params bframes=0:rc-lookahead=0:intra-refresh=1:keyint={2 * fps}:repeat-headers=1 "
            f"-f mpegts -pat_period 1 -sdt_period 1 -muxdelay 0 -flush_packets 1 "
            f"'udp://{dest}:{int(vid['UdpPort'])}?pkt_size=1316'")
    return f"echo $$ > {PID_FILE}; exec {args}"


def check_pi(vid: dict) -> str:
    """One SSH call: login, ffmpeg, libx264, the video file, cores, power and temperature."""
    f = remote_path(vid["PiVideoFile"])
    remote = ("echo login ok on $(hostname); ffmpeg -hide_banner -version 2>&1 | head -1; "
              "if ffmpeg -hide_banner -encoders 2>/dev/null | grep -q libx264; then echo 'libx264: yes'; "
              "else echo 'libx264: MISSING (sudo apt install ffmpeg)'; fi; "
              f"ls -l {f} 2>&1; echo cores: $(nproc); "
              "command -v vcgencmd >/dev/null && vcgencmd get_throttled && vcgencmd measure_temp; true")
    try:
        p = subprocess.run(ssh_cmd(vid, remote), capture_output=True, text=True, timeout=20)
    except subprocess.TimeoutExpired:
        return "No answer from the Pi within 20 s."
    except FileNotFoundError as e:
        return f"Cannot run {e.filename}: install the OpenSSH client."
    out = (p.stdout + p.stderr).strip()
    if p.returncode == 255:
        out += (f"\nSSH login failed. Check the address, then set up a key login once from a terminal:\n"
                f"  ssh-copy-id {vid['PiUser']}@{vid['PiAddress']}")
    return out


class PiEncoder:
    """ffmpeg on the Pi: started over SSH, its warnings kept, stopped by its process id."""

    def __init__(self, vid: dict, plan: dict):
        self.vid = vid
        self.cmd = encoder_command(vid, plan)
        self.lines: deque[str] = deque(maxlen=20)
        self.proc: subprocess.Popen | None = None

    def start(self) -> None:
        self.proc = subprocess.Popen(ssh_cmd(self.vid, self.cmd), stdin=subprocess.DEVNULL,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self) -> None:
        for line in self.proc.stdout:
            if line.strip():
                self.lines.append(line.rstrip())

    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def stop(self) -> None:
        if self.proc is None:
            return
        try:
            subprocess.run(ssh_cmd(self.vid, f"kill $(cat {PID_FILE}) 2>/dev/null; rm -f {PID_FILE}"),
                           capture_output=True, timeout=10)
        except (subprocess.TimeoutExpired, OSError):
            pass
        try:
            self.proc.terminate()
            self.proc.wait(timeout=5)
        except (subprocess.TimeoutExpired, OSError):
            self.proc.kill()


# ---------- Video in (UDP) ----------
class UdpIn:
    """Receives the encoder's datagrams (7 TS packets each) into a ByteFifo."""

    def __init__(self, port: int, fifo: VF.ByteFifo):
        self.fifo = fifo
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 21)
        try:
            self.sock.bind(("0.0.0.0", int(port)))
        except OSError as e:
            self.sock.close()
            raise RuntimeError(f"cannot listen on UDP port {port}: {e}") from e
        self.sock.settimeout(0.2)
        self.bytes = 0
        self.datagrams = 0
        self.first = None                      # time of the first datagram
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                d = self.sock.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            if self.first is None:
                self.first = time.monotonic()
            self.fifo.put(d)
            self.bytes += len(d)
            self.datagrams += 1

    def close(self) -> None:
        self._stop.set()
        self.thread.join(timeout=1)
        self.sock.close()


# ---------- Loopback sink: unpack, resync, ffplay and a file ----------
class LoopbackSink:
    def __init__(self, drop_percent: float, play: bool = True):
        self.unpacker = VF.VideoUnpacker()
        self.resync = VF.TsResync()
        self.drop = drop_percent / 100.0
        self.rng = np.random.default_rng()
        self.out = VF.TsOutput(play=play, save_path=LOOPBACK_TS, title="Flexlink loopback")

    def packet(self, tbs: np.ndarray) -> None:
        crc = self.rng.random(tbs.shape[0]) >= self.drop if self.drop > 0 else np.ones(tbs.shape[0], bool)
        for gap, data in self.unpacker.feed(tbs, crc):
            self.out.write(self.resync.push(data, gap))

    def close(self) -> None:
        self.out.close()

    def stats(self) -> str:
        t = self.unpacker.total
        return (f"loopback: {t.blocks} blocks, {t.crc_fail} dropped, {t.filler} filler, {t.gaps} gaps, "
                f"{self.resync.ts_packets} TS packets out, {self.resync.resyncs} resyncs"
                + (f"; {self.out.note}" if self.out.note else ""))


# ---------- The session (worker thread) ----------
def tx_worker(post, pl: dict, vid: dict, stop: threading.Event, play: bool = True) -> None:
    """post(kind, **kw): "started" | "stats" | "log" | "done". Runs until Stop or an error."""
    error = None
    c = enc = udp = sink = None
    packer = None
    seq = 0
    n_sent = 0
    last = {}
    t0 = time.monotonic()
    try:
        cfg = TXP.tx_config(json.loads(TXP.TX_JSON.read_text())["Defined"])
        plan = video_plan(cfg, pl["Gap"], vid["VideoShare_percent"], vid["MaxHeight"])
        post("log", text=plan_text(cfg, plan))
        fifo = VF.ByteFifo()
        udp = UdpIn(vid["UdpPort"], fifo)
        if vid["Loopback"]:
            ndb, tb_bits, period = cfg.ndb, cfg.tb_size, plan["period"]
            sink = LoopbackSink(vid["LoopbackDrop_percent"], play)
            post("started", text=f"Loopback on this PC: {ndb} blocks of {tb_bits} bits every "
                                 f"{period * 1e3:.3f} ms; received stream -> {LOOPBACK_TS}")
        else:
            c, rep = TXP.connect(pl, cfg)
            ndb, tb_bits = rep["BlocksPerPacket"], rep["TransportBlockSize"]
            period = (rep["PacketSamples"] + rep["Gap"]) / FS_PLUTO
            post("started", text=(f"Pluto {pl['Address']}: {ndb} blocks of {tb_bits} bits per packet, "
                                  f"{rep['PacketSamples']} samples + gap {rep['Gap']}, {c.depth} credits, "
                                  f"carrier {pl['TxLO_MHz']:g} MHz, Tx gain {pl['TxGain_dB']:g} dB"))
        packer = VF.VideoPacker(tb_bits, ndb)
        enc = PiEncoder(vid, plan)
        post("log", text=("Encoder: " if not is_local(vid) else "Encoder (this PC): ") + enc.cmd.split("exec ", 1)[1])
        enc.start()
        max_queue = int(vid["MaxQueue_ms"] / 1e3 * plan["capacity_bps"] / 8)
        t0 = time.monotonic()
        t_next = t0
        t_post = 0.0
        warned = False
        rate_win = deque([(t0, 0)], maxlen=8)
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
            fifo.trim(max_queue)
            tbs = packer.next_packet(fifo)
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
            if not enc.running():
                tail = "\n".join(enc.lines) or "(no output)"
                raise RuntimeError(f"the encoder stopped:\n{tail}")
            if not warned and udp.first is None and now - t0 > 8:
                warned = True
                post("log", text=f"No video from the encoder after 8 s on UDP port {vid['UdpPort']}: check "
                                 f"PC Address ({vid['PcAddress']}) and the firewall. Encoder says: "
                                 + (" | ".join(enc.lines) or "nothing"))
            if now - t_post > 0.5:
                t_post = now
                rate_win.append((now, udp.bytes))
                (ta, ba), (tb, bb) = rate_win[0], rate_win[-1]
                vin = (bb - ba) * 8 / max(tb - ta, 1e-9) / 1e3
                q_ms = fifo.size() * 8 / plan["capacity_bps"] * 1e3
                line = (f"{now - t0:6.1f} s  video in {vin:7.0f} kbit/s, queue {q_ms:5.0f} ms, dropped "
                        f"{fifo.dropped_chunks} datagram(s) | {seq} packets pushed, {n_sent} on air, "
                        f"blocks {100 * packer.fill_ratio():3.0f} % full")
                if c is not None:
                    line += f", PL underflows {last.get('underflows', '-')}"
                else:
                    line += "\n" + sink.stats()
                if enc.lines:
                    line += f"\nencoder: {enc.lines[-1]}"
                post("stats", text=line)
    except Exception as e:  # noqa: BLE001  (shown in the GUI)
        error = f"{type(e).__name__}: {e}"
    finally:
        if enc is not None:
            enc.stop()
        if udp is not None:
            udp.close()
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
         fill=packer.fill_ratio() if packer else 0.0,
         loopback=sink.stats() if sink is not None else "")


# ---------- Window ----------
def set_running(window, running: bool) -> None:
    window.metadata = running
    window["-SAVE-"].update(disabled=running)
    window["-START-"].update(disabled=running)
    window["-STOP-"].update(disabled=not running)
    window["-CHECK_PI-"].update(disabled=running)
    window["-LOOPBACK-"].update(disabled=running)


def update_plan(window, values) -> None:
    try:
        cfg, plan = current_plan(values)
        window["-PLAN-"].update(plan_text(cfg, plan))
        window["-TX_DERIVED-"].update(TXP.derived_text(values))
    except Exception as e:  # noqa: BLE001  (a hint only)
        window["-PLAN-"].update(f"(no plan: {e})")


def main():
    window = sg.Window("Flexlink Video Tx with Pluto", layout, finalize=True, resizable=True)
    window.metadata = False
    stop = threading.Event()
    stats_shown = False
    try:
        load_into_window(window)
        window["-STATUS-"].update("Loaded Parameters_Tx_payload.json, Parameters_Pluto.json, Parameters_Video.json")
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
            vid, errors = video_from_values(values)
            if errors:
                sg.popup_error("\n".join(errors), title="Pi settings")
                continue
            window["-STATUS-"].update("Checking the Pi ...")
            window.start_thread(lambda: window.write_event_value("-PI_CHECKED-", check_pi(vid)), "-THREAD_END-")
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
            vid = load_video()["Video"]
            stop.clear()
            stats_shown = False
            window["-TX_STATUS-"].update("Starting ...")
            set_running(window, True)
            window.start_thread(lambda: tx_worker(post, pl, vid, stop), "-THREAD_END-")
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
                window["-STATUS-"].update("Transmitting video ... (Stop ends it)")
            elif ev["kind"] == "log":
                append(ev["text"])
                stats_shown = False
            elif ev["kind"] == "stats":
                lines = window["-TX_STATUS-"].get().rstrip("\n").split("\n")
                if stats_shown:
                    lines = lines[:-stats_shown]
                window["-TX_STATUS-"].update("\n".join(lines + ev["text"].split("\n")))
                stats_shown = len(ev["text"].split("\n"))
            else:                                   # done
                set_running(window, False)
                stats_shown = False
                if ev["error"]:
                    append(f"ERROR: {ev['error']}")
                    window["-STATUS-"].update("Stopped with an error (see Transmitter).")
                else:
                    window["-STATUS-"].update(f"Stopped: {ev['on_air']} packet(s) on air in {ev['seconds']:.1f} s.")
                append(f"Stopped: {ev['packets']} packet(s) pushed, {ev['on_air']} on air, blocks "
                       f"{100 * ev['fill']:.0f} % full" + (f"; {ev['loopback']}" if ev["loopback"] else "")
                       + ("; transmitter switched off" if (ev["final"] or {}).get("stopped") else ""))
            continue
        if event == "-THREAD_END-":
            continue

    window.close()


if __name__ == "__main__":
    main()
