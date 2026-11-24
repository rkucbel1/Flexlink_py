"""FlexlinkRxMavlinkWithZedboard_CoreBoard.py -- receives the video and the MAVLink telemetry of
FlexlinkTxMavlinkWithPluto.py (GUI).

FlexlinkRxVideoWithZedboard_CoreBoard.py (the same CoreBoard + Zedboard radio side, the same
settings) with the second stream:

    Zedboard --TCP: transport blocks-->  this PC  --video blocks (0xF2)--> MPEG-TS --> ffplay
                                                                                  --> UDP (a ground station)
                                                 --MAVLink blocks (0xF3)--> frames --> telemetry window
                                                                                  --> UDP (a ground station)
                                                                                  --> .tlog (optional)

  Telemetry window   a window of its own (App/Mavlink/Telemetry_Window.py): artificial horizon,
                     compass, mode, position, altitude, speeds, GPS, battery and the vehicle's
                     status messages. Opens on Start (if ticked) or with the Telemetry button.
  MAVLink to         every received frame as one UDP datagram, as a ground station expects:
                     QGroundControl and Mission Planner listen on UDP 14550 and show the vehicle
                     when its HEARTBEAT arrives. The link is one way: the ground station's
                     requests do not reach the vehicle (QGC may say it cannot load parameters).
  Video to           the received MPEG-TS stream as UDP datagrams of up to 7 TS packets (1316
                     bytes). QGroundControl: Application Settings > Video > Source "MPEG-TS",
                     UDP port the same as here (QGC's default 5600). On the PC that also runs the
                     transmitter app, 5600 is taken by the Pi's video: use another port there.
  Save MAVLink       the received frames to App/Mavlink/Received.tlog (opens in QGC's Log Replay)

Settings: those of the video receiver (App/Parameters_CoreBoard.json, App/Parameters_RxVideo.json)
and App/Mavlink/Parameters_RxMavlink.json (shared with the FMCOMMS3 app). Start saves them first
(as Save does). The MAVLink part is App/Mavlink/Rx_Mavlink.py, also used by
FlexlinkRxMavlinkWithZedboard_FCOMMS3.py.

Requires: as FlexlinkRxVideoWithZedboard_CoreBoard.py
Run with: python App/FlexlinkRxMavlinkWithZedboard_CoreBoard.py
"""
from __future__ import annotations

import json
import socket
import struct
import sys
import threading
import time
from pathlib import Path

APP_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(APP_DIR))
import FlexlinkRxVideoWithZedboard_CoreBoard as RX  # noqa: E402  (board protocol, settings, VideoRx)

pr, PS, VF, sg, Board = RX.pr, RX.PS, RX.VF, RX.sg, RX.Board
from flexlink_paths import APP  # noqa: E402

MAV_DIR = APP / "Mavlink"
sys.path.insert(0, str(MAV_DIR))
import Rx_Mavlink as RM  # noqa: E402  (MAVLink receiver part, shared with the FMCOMMS3 app)

H1, H2, ITALIC, RESULTS_FONT = RX.H1, RX.H2, RX.ITALIC, RX.RESULTS_FONT

# ---------- Layout: the video receiver's frames (same keys), rebuilt, and the ground station frame ----------
in_row, CB, RV = RX.in_row, RX.CB_DEFAULTS, RX.RXV_DEFAULTS
LW = RX.LW
cb_frame = sg.Frame("CoreBoard (Parameters_CoreBoard.json)", [
    in_row("CoreBoard Address", "-CB_ADDR-", CB["Address"], size=16),
    in_row("HTTP Port", "-CB_HTTP-", CB["HttpPort"]),
    in_row("Carrier (MHz)", "-CB_FREQ-", CB["Frequency_MHz"], extra="= the Pluto's carrier"),
    in_row("Gain (index)", "-CB_GAIN-", CB["Gain"], extra="not dB"),
    in_row("Analog filter (MHz)", "-CB_BW-", CB["Bandwidth_MHz"], extra="0 = follow the rate"),
    [sg.Text("", size=(LW, 1)), sg.Checkbox("AGC (not recommended)", key="-CB_AGC-")],
    [sg.Text("", size=(LW, 1)), sg.Checkbox("DC correction", key="-CB_DC-", default=True)],
    [sg.Text("40 Msps to the Zedboard", font=ITALIC)],
], font=H2, vertical_alignment="top", expand_x=True)

zed_frame = sg.Frame("Zedboard receiver (Parameters_RxVideo.json)", [
    in_row("Zedboard Address", "-ZED_ADDR-", RV["ZedAddress"], size=16),
    in_row("TCP Port", "-ZED_PORT-", RV["ZedPort"]),
    in_row("UDP Port (samples)", "-ZED_UDP-", RV["ZedUdpPort"]),
    in_row("Push interval (ms)", "-ZED_PUSH-", RV["Push_ms"], extra="0.25 .. 7.75"),
    in_row("DC shift", "-ZED_DC-", RV["DcShift"], extra="6 for the CoreBoard"),
    [sg.Button("Check", key="-CHECK-", size=10), sg.Text("Zedboard firmware, CoreBoard status", font=ITALIC)],
], font=H2, vertical_alignment="top", expand_x=True)

video_frame = sg.Frame("Video", [
    [sg.Checkbox("Play with ffplay", key="-PLAY-", default=True)],
    [sg.Checkbox("Save the received stream", key="-SAVE_TS-")],
    [sg.Text(f"   to App/Video/{RX.RECEIVED_TS.name} (about 3 GB an hour at 7 Mbit/s)", font=ITALIC)],
], font=H2, vertical_alignment="top", expand_x=True)

gcs_frame = RM.gcs_frame(sg, H2, ITALIC)

layout = [
    [sg.Text("Flexlink - Video + MAVLink Receiver (CoreBoard + Zedboard)", font=H1, justification="center",
             expand_x=True, pad=(0, 15))],
    [sg.Push(), sg.Column([[cb_frame]], vertical_alignment="top"),
     sg.Column([[zed_frame], [video_frame]], vertical_alignment="top"),
     sg.Column([[gcs_frame]], vertical_alignment="top"), sg.Push()],
    [sg.Push(), sg.Text("Transmitter", size=(12, 1), justification="right"),
     sg.Multiline(size=(120, 2), key="-CF-", disabled=True, font=RESULTS_FONT), sg.Push()],
    [sg.Push(), sg.Text("Receiver", size=(12, 1), justification="right"),
     sg.Multiline(size=(120, 11), key="-RX_STATUS-", disabled=True, autoscroll=True, font=RESULTS_FONT),
     sg.Push()],
    [sg.Push(), sg.Button("Save", key="-SAVE-", size=10), sg.Button("Start", key="-START-", size=10),
     sg.Button("Stop", key="-STOP-", size=10, disabled=True), sg.Push()],
    [sg.Text("", key="-STATUS-", size=(120, 1), justification="center")],
]


def load_into_window(window) -> None:
    RX.load_into_window(window)
    RM.load_into_window(window)


def save_settings(values) -> tuple[dict, dict, dict]:
    """The video receiver's files (RX.save_settings) and App/Mavlink/Parameters_RxMavlink.json.
    Raises ValueError, nothing written."""
    rxm, errors = RM.rxm_from_values(values)
    _cb, _rxv, e2 = RX.settings_from_values(values)
    if errors or e2:
        raise ValueError("\n".join(errors + e2))
    cb, rxv = RX.save_settings(values)
    return cb, rxv, RM.save_rxm(rxm)


MavVideoRx = RM.mavlink_rx(RX.VideoRx)


# ---------- The session (worker thread): FlexlinkRxVideoWithZedboard_CoreBoard.rx_worker with MAVLink ----------
def rx_worker(post, cb: dict, rxv: dict, rxm: dict, stop: threading.Event) -> None:
    """post(kind, **kw): "started" | "stats" | "cf" | "telemetry" | "done"."""
    error = None
    b = out = rx = None
    streaming = False
    st = None
    t0 = time.monotonic()
    try:
        local = VF.TsOutput(play=bool(rxv["Play"]), save_path=RX.RECEIVED_TS if rxv["SaveStream"] else None,
                            title="Flexlink received video")
        out = RM.video_output(local, rxm)
        rx = MavVideoRx(out, rxm)
        b = Board(rxv["ZedAddress"], int(rxv["ZedPort"]), 10.0)
        info = b.ping()
        if info.build < 39 or not info.flags & pr.BOARD_PACK8:
            raise RuntimeError(f"the Zedboard has firmware build {info.build}: push mode needs build 39 "
                               f"and bitstream 9.1 or later")
        b.call(pr.SET_PARAMS, RX.detector_params(rxv["DcShift"]).pack())
        body = PS.coreboard_start_json(int(rxv["ZedUdpPort"]))     # from App/Parameters_CoreBoard.json
        push_units = min(max(round(float(rxv["Push_ms"]) * 4), 1), 31)
        flags = pr.STREAM_FLAG_PUSH | (push_units << 2)
        reply = b.call(pr.STREAM_START, PS.start_payload(int(rxv["ZedUdpPort"]), flags, cb["Address"],
                                                          int(cb["HttpPort"]), body))
        streaming = True
        post("started", text=(f"Zedboard {rxv['ZedAddress']}: firmware build {info.build}, streaming on its UDP port "
                              f"{struct.unpack('<I', reply[:4])[0]}; CoreBoard {cb['Address']}: "
                              f"{body['frequency'] / 1e6:g} MHz, {body['rate'] / 1e6:g} Msps, "
                              + ("AGC" if body["agc"] else f"gain {body.get('gain')}")
                              + f"; results pushed at most every {push_units / 4:g} ms" + RM.started_text(rxm)))
        b.sock.settimeout(5.0)
        t0 = time.monotonic()
        t_post = t_tele = 0.0
        cf_shown = None
        while not stop.is_set():
            cmd, _status, _seq, raw = pr.recv_msg(b.sock)
            if cmd != pr.STREAM_PUSH:
                raise pr.ProtocolError(f"unexpected message {pr.CMD_NAMES.get(cmd, cmd)} while streaming")
            slot, st_new, cfs, tbs = RX.parse_push(raw)
            st = st_new or st
            rx.add(slot, cfs, tbs)
            now = time.monotonic()
            if rx.cf_text != cf_shown:
                cf_shown = rx.cf_text
                post("cf", text=cf_shown)
            if now - t_tele > RM.TELEMETRY_PERIOD:
                t_tele = now
                post("telemetry", snap=rx.tele.snapshot(now))
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
                slot, st_new, cfs, tbs = RX.parse_push(raw)
                st = st_new or st
                rx.add(slot, cfs, tbs)
                continue
            if cmd != pr.STREAM_STOP or seq != b.seq or status != pr.ST_OK:
                raise pr.ProtocolError(f"STREAM_STOP reply: cmd {cmd} seq {seq} status {status}")
            break
        streaming = False
        st, cfs, tbs = RX.parse_poll(raw)
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
        if rx is not None:
            rx.close()
    post("done", error=error, summary=rx.stats_line(time.monotonic() - t0, st) if rx is not None else "",
         snap=rx.tele.snapshot() if rx is not None else None)


# ---------- Window ----------
def set_running(window, running: bool) -> None:
    window.metadata = running
    for k in ("-SAVE-", "-START-", "-CHECK-") + RM.RUN_LOCKED:
        window[k].update(disabled=running)
    window["-STOP-"].update(disabled=not running)


def main():
    window = sg.Window("Flexlink Video + MAVLink Rx", layout, finalize=True, resizable=True)
    window.metadata = False
    tele = RM.TelemetryUi(sg)
    stop = threading.Event()
    stats_lines = 0
    try:
        load_into_window(window)
        window["-STATUS-"].update("Loaded Parameters_CoreBoard.json, Parameters_RxVideo.json, Parameters_RxMavlink.json")
    except (OSError, ValueError, KeyError) as e:
        sg.popup_error(f"Could not load the parameter files:\n{e}")

    def post(kind, **kw):
        window.write_event_value("-RX_EVENT-", {"kind": kind, **kw})

    def append(text):
        cur = window["-RX_STATUS-"].get().rstrip("\n")
        window["-RX_STATUS-"].update(cur + ("\n" if cur.strip() else "") + text)

    while True:
        win, event, values = sg.read_all_windows(timeout=500)
        if tele.owns(win, event):                  # the telemetry window: only closing it
            continue
        if event == sg.TIMEOUT_EVENT:
            continue
        if event == sg.WIN_CLOSED:                 # the main window
            if window.metadata:
                stop.set()
                time.sleep(1.0)
            break
        if event == "-TELE_OPEN-":
            tele.open()
            continue
        if event == "-SAVE-":
            try:
                save_settings(values)
                window["-STATUS-"].update("Saved Parameters_CoreBoard.json, Parameters_RxVideo.json, "
                                          "Parameters_RxMavlink.json.")
            except (OSError, ValueError) as e:
                sg.popup_error(f"Nothing was saved:\n{e}", title="Save failed")
            continue
        if event == "-CHECK-":
            cb, rxv, errors = RX.settings_from_values(values)
            if errors:
                sg.popup_error("\n".join(errors), title="Settings")
                continue
            window["-STATUS-"].update("Checking ...")
            window.start_thread(lambda: window.write_event_value("-CHECKED-", RX.check(cb, rxv)), "-THREAD_END-")
            continue
        if event == "-CHECKED-":
            window["-RX_STATUS-"].update(values[event])
            window["-STATUS-"].update("Checked (see Receiver).")
            continue
        if event == "-START-":
            try:
                cb, rxv, rxm = save_settings(values)
            except (OSError, ValueError) as e:
                sg.popup_error(f"Not started, nothing was saved:\n{e}", title="Invalid settings")
                continue
            stop.clear()
            stats_lines = 0
            tele.last = None
            window["-RX_STATUS-"].update("Starting ...")
            window["-CF-"].update("")
            set_running(window, True)
            if rxm["ShowTelemetry"]:
                tele.open()
            window.start_thread(lambda: rx_worker(post, cb, rxv, rxm, stop), "-THREAD_END-")
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
            elif ev["kind"] == "telemetry":
                tele.show(ev["snap"])
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
                tele.show(ev["snap"])
                if ev["error"]:
                    append(f"ERROR: {ev['error']}")
                    window["-STATUS-"].update("Stopped with an error (see Receiver).")
                else:
                    window["-STATUS-"].update("Stopped.")
                if ev["summary"]:
                    append("Final: " + ev["summary"])
            continue

    tele.close()
    window.close()


if __name__ == "__main__":
    main()
