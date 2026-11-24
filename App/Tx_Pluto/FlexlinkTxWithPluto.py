"""FlexlinkTxWithPluto.py -- Flexlink transmitter on an ADALM-Pluto (GUI).

The Pluto runs the Flexlink transmitter in its FPGA (HW_Development/Packet_Transmitter) and the
transmitter server FW/ptx_server on its ARM (TCP port 5010). This app sends it the settings and
the transport blocks:

  Tx tab        the transmitter settings (the Defined section of Tx/Parameters_Tx_payload.json,
                same fields as Flexlinksimulation.py's Set Tx tab), the Pluto's radio settings
                (App/Tx_Pluto/Parameters_Pluto.json) and what to send:
                  - Continuous random packets: new random transport blocks in every packet, until
                    Stop
                  - Selected file: the file cut into transport blocks (File_Framing.py, as
                    Run_File_Transfer.py does), sent once or a number of times, or until Stop
                  - Loopback (check box): one random packet, transmitted over and over while the
                    same Pluto captures it on RX1 (TX1 cabled to RX1 through 30-40 dB), then
                    decoded by the Python Rx chain. The results and the plots are on the Received
                    tab.
  Received tab  the loopback results (control field and payload, as on Flexlinksimulation's
                Simulate tab) and the plot selection. The plots are drawn when a loopback run
                finishes, and again with Replot.

Loopback runs, in a background thread:
    Tx/Initialize_Tx.py -> Tx/Run_Tx_Chain.py --no-defects   (random blocks, reference grid)
    Pluto: SET_PARAMS, the packet's blocks, capture at 40 Msps (pyadi-iio)
    capture -> 20 Msps, scaled to the simulator's level -> App/Tx_Pluto/Loopback/Loopback_Capture_Rx.h5
    Rx/Initialize_Rx.py -> Rx/Run_Rx_Chain.py --input <capture> (no defects modelled)

Start saves the settings first (as Save does).

Requires: pip install FreeSimpleGUI pyadi-iio (pyadi-iio and the libiio C library for loopback only)
Run with: python App/Tx_Pluto/FlexlinkTxWithPluto.py
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

import numpy as np
import FreeSimpleGUI as sg

for _p in Path(__file__).resolve().parents:      # put the project root (flexlink_paths.py) on sys.path
    if (_p / "flexlink_paths.py").is_file():
        if str(_p) not in sys.path:
            sys.path.insert(0, str(_p))
        break
from flexlink_paths import (APP, APP_SIMULATION, APP_TX_PLUTO, BASIC_MODULES, CHANNEL_DEFECTS,  # noqa: E402
                            PACKET_TRANSMITTER_PC, ROOT, RX, TX, ray_file_name)
sys.path.insert(0, str(APP_SIMULATION))   # Flexlinksimulation, Flexlink_Plots
import Flexlinksimulation as FS  # noqa: E402  (JSON link, Tx field lists, result parsing)

PTX_PC = PACKET_TRANSMITTER_PC
for _p in (PTX_PC, TX / "Generate_FileData", BASIC_MODULES / "File_Transfer"):
    sys.path.insert(0, str(_p))
import ptx_client  # noqa: E402
import ptx_reference as R  # noqa: E402

sg.theme("SystemDefault")
H1, H2 = FS.H1, FS.H2
ITALIC = ("Helvetica", 9, "italic")
RESULTS_FONT = FS.RESULTS_FONT

TX_JSON = TX / "Parameters_Tx_payload.json"
RX_JSON = RX / "Parameters_Rx_payload.json"
CF_JSON = TX / "ControlSignals" / "Parameters_Control_Field.json"
PLUTO_JSON = APP_TX_PLUTO / "Parameters_Pluto.json"
TEST_FILES_DIR = APP_TX_PLUTO / "Test_Files"     # where Browse starts for "Selected file"
PLUTO_DIR = APP_TX_PLUTO / "Loopback"           # loopback files (recreated on every loopback run)
CAPTURE_40 = PLUTO_DIR / "Loopback_Capture_40MHz.npy"
CAPTURE_RX = PLUTO_DIR / "Loopback_Capture_Rx.h5"
DEFECTS_OFF = PLUTO_DIR / "Channel_Defects_Off.json"
LOOPBACK_LOG = PLUTO_DIR / "Loopback.log"
FS_PLUTO = 40_000_000                           # the PL transmitter runs 40 Msps only
RX_RATE = 20_000_000                            # the Python Rx chain's rate
MAX_TRIES = 3                                   # loopback: packets of the capture tried in turn

# ---------- Pluto settings (App/Tx_Pluto/Parameters_Pluto.json) ----------
# gui key -> (json key, type, min, max, label); numbers are checked on Save / Start
PLUTO_FIELDS = {
    "-PL_ADDR-": ("Address", str, None, None, "Pluto Address"),
    "-PL_PORT-": ("ServerPort", int, 1, 65535, "Server Port"),
    "-PL_LO-": ("TxLO_MHz", float, 70.0, 6000.0, "Carrier (MHz)"),
    "-PL_TXGAIN-": ("TxGain_dB", float, -89.75, 0.0, "Tx Gain (dB)"),
    "-PL_BW-": ("RfBandwidth_MHz", float, 0.2, 56.0, "RF Bandwidth (MHz)"),
    "-PL_GAP-": ("Gap", int, 0, 65535 * 16, "Gap (samples)"),
    "-FILE-": ("File", str, None, None, "File"),
    "-FILE_REPEAT-": ("FileRepeat", int, 0, 100000, "Send the File"),
    "-PL_RXGAIN-": ("RxGain_dB", float, 0.0, 71.0, "Rx Gain (dB)"),
    "-PL_CAPTURE-": ("CaptureSamples", int, 1 << 16, 1 << 24, "Capture (samples)"),
}
PLUTO_DEFAULTS = {"Address": "192.168.2.1", "ServerPort": 5010, "TxLO_MHz": 2400.0, "TxGain_dB": -10.0,
                  "RfBandwidth_MHz": 18.0, "Gap": 20000, "Mode": "Random", "File": "", "FileRepeat": 1,
                  "Loopback": 0, "RxGain_dB": 30.0, "CaptureSamples": 1 << 20}
PLUTO_COMMENTS = {
    "Address_comment": "IP address of the Pluto (USB network: 192.168.2.1)",
    "ServerPort_comment": "TCP port of FW/ptx_server on the Pluto",
    "TxLO_MHz_comment": "carrier, MHz (AD9363 data sheet 325 to 3800; most Plutos tune 70 to 6000)",
    "TxGain_dB_comment": "Tx attenuation as a gain, -89.75 to 0 dB",
    "RfBandwidth_MHz_comment": "analog RF bandwidth, MHz (the signal occupies 16.4 MHz)",
    "Gap_comment": "idle samples after every packet (rounded up to 16, at most 1048560)",
    "Mode_comment": "Random (continuous random packets) or File",
    "FileRepeat_comment": "times the file is sent; 0 = until Stop",
    "Loopback_comment": "1: one random packet looped TX1 -> RX1 on the same Pluto and decoded",
    "RxGain_dB_comment": "loopback only: manual Rx gain, dB",
    "CaptureSamples_comment": "loopback only: samples captured at 40 Msps (lengthened to 4 packet periods if shorter)",
}

# ---------- Tab 1: Tx ----------
TX_LABEL_W = FS.TX_LABEL_W
PL_LABEL_W = 18


def pl_row(label, key, default, size=12, extra=None):
    row = [sg.Text(label, size=(PL_LABEL_W, 1), justification="right"),
           sg.Input(str(default), key=key, size=size, justification="right")]
    return row + ([sg.Text(extra, font=ITALIC)] if extra else [])


# The same fields and option lists as Flexlinksimulation's Set Tx tab (FS.LINKS["tx"])
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
    pl_row("Pluto Address", "-PL_ADDR-", PLUTO_DEFAULTS["Address"], size=16),
    pl_row("Server Port", "-PL_PORT-", PLUTO_DEFAULTS["ServerPort"]),
    pl_row("Carrier (MHz)", "-PL_LO-", PLUTO_DEFAULTS["TxLO_MHz"]),
    pl_row("Tx Gain (dB)", "-PL_TXGAIN-", PLUTO_DEFAULTS["TxGain_dB"], extra="-89.75 to 0"),
    pl_row("RF Bandwidth (MHz)", "-PL_BW-", PLUTO_DEFAULTS["RfBandwidth_MHz"]),
    pl_row("Gap (samples)", "-PL_GAP-", PLUTO_DEFAULTS["Gap"], extra="idle after each packet"),
    [sg.Text("40 Msps, set by the Pluto's transmitter server", font=ITALIC)],
], font=H2, vertical_alignment="top", expand_x=True)

send_frame = sg.Frame("Transmission", [
    [sg.Radio("Continuous random packets (until Stop)", "TXMODE", key="-MODE_RANDOM-", default=True,
              enable_events=True)],
    [sg.Radio("Selected file", "TXMODE", key="-MODE_FILE-", enable_events=True)],
    [sg.Text("File", size=(6, 1), justification="right"),
     sg.Input("", key="-FILE-", size=34), sg.FileBrowse("Browse", key="-FILE_BROWSE-",
                                                         initial_folder=str(TEST_FILES_DIR))],
    [sg.Text("Send the file", size=(PL_LABEL_W - 6, 1), justification="right"),
     sg.Input("1", key="-FILE_REPEAT-", size=6, justification="right"),
     sg.Text("time(s); 0 = until Stop", font=ITALIC)],
], font=H2, vertical_alignment="top", expand_x=True)

loopback_frame = sg.Frame("Loopback", [
    [sg.Checkbox("Loopback test: one random packet, TX1 -> RX1, decoded",
                 key="-LOOPBACK-", enable_events=True)],
    pl_row("Rx Gain (dB)", "-PL_RXGAIN-", PLUTO_DEFAULTS["RxGain_dB"], extra="manual"),
    pl_row("Capture (samples)", "-PL_CAPTURE-", PLUTO_DEFAULTS["CaptureSamples"], extra="at 40 Msps, min 4 packets"),
    [sg.Text("Cable TX1 to RX1 through 30-40 dB of attenuation (or use antennas).\n"
             "Results and plots: Received tab. Needs pyadi-iio and libiio on the PC.", font=ITALIC)],
], font=H2, vertical_alignment="top", expand_x=True)

tx_tab = [
    [sg.Text("Flexlink - Pluto Transmitter", font=H1, justification="center", expand_x=True, pad=(0, 15))],
    [sg.Push(), sg.Column([[tx_param_frame]], vertical_alignment="top"),
     sg.Column([[pluto_frame], [send_frame], [loopback_frame]], vertical_alignment="top"), sg.Push()],
    [sg.Push(), sg.Text("Transmitter", size=(12, 1), justification="right",
                        tooltip="Continuous / file: packets sent, data rate, PL counters. Loopback: its steps"),
     sg.Multiline(size=(110, 7), key="-TX_STATUS-", disabled=True, autoscroll=True, font=RESULTS_FONT),
     sg.Push()],
]

# ---------- Tab 2: Received ----------
packet_frame = sg.Frame("Packet Plots", [
    [sg.Checkbox("Transmitted Packet", key="-PLOT_TX_PACKET-", pad=(5, 6))],
    [sg.Checkbox("Received Packet", key="-PLOT_RX_PACKET-", pad=(5, 6))],
    [sg.Checkbox("PAPR CCDF", key="-PLOT_CCDF-", pad=(5, 6))],
    [sg.Text("Received = the Pluto capture,\nthe packet the Rx chain found", font=ITALIC)],
], font=H2, vertical_alignment="top", expand_x=True)

constellation_frame = sg.Frame("Constellation Plots", [
    [sg.Checkbox("Before Equalization", key="-PLOT_CONST_BEFORE-", pad=(5, 6))],
    [sg.Checkbox("After Equalization", key="-PLOT_CONST_AFTER-", pad=(5, 6))],
    [sg.Text("Symbols to plot"), sg.Input("15", key="-PLOT_SYMBOLS-", size=6, justification="right")],
], font=H2, vertical_alignment="top", expand_x=True)

evm_frame = sg.Frame("EVM Plots", [
    [sg.Checkbox("EVM vs Subcarrier", key="-PLOT_EVM_SC-", pad=(5, 6))],
    [sg.Checkbox("EVM vs OFDM Symbol", key="-PLOT_EVM_SYM-", pad=(5, 6))],
    [sg.Text("After equalization,\nvs the transmitted symbols", font=ITALIC)],
], font=H2, vertical_alignment="top", expand_x=True)

plots_col = sg.Column([
    [sg.Text("")],
    [packet_frame], [constellation_frame], [evm_frame],
    [sg.Button("Replot", key="-REPLOT-", size=10, disabled=True)],
    [sg.Text("Plots are drawn when a loopback\nrun finishes; Replot redraws the\nlast run with "
             "the boxes ticked now", font=ITALIC)],
], vertical_alignment="top")

rx_tab = [
    [plots_col, sg.Push(), sg.Column([
        [sg.Text("")],
        FS.results_row("Pluto Loopback", "-RES_PLUTO-", height=7),
        FS.results_row("Control Field Results", "-RES_CNTL-", height=10),
        FS.results_row("Payload Field Results", "-RES_PAYLOAD-", height=14),
    ]), sg.Push()],
]


# ---------- Settings: load / save ----------
def load_pluto() -> dict:
    """App/Tx_Pluto/Parameters_Pluto.json (the whole file); the defaults if it is not there yet."""
    try:
        data = json.loads(PLUTO_JSON.read_text(encoding="utf-8"))
    except FileNotFoundError:
        data = {}
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON in {PLUTO_JSON}: line {e.lineno}, col {e.colno}: {e.msg}")
    sec = data.setdefault("Pluto", {})
    for k, v in PLUTO_DEFAULTS.items():
        sec.setdefault(k, v)
        if f"{k}_comment" in PLUTO_COMMENTS:
            sec.setdefault(f"{k}_comment", PLUTO_COMMENTS[f"{k}_comment"])
    return data


def write_json_atomic(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=path.name, suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(FS._dumps(data) + "\n")
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


def load_into_window(window) -> None:
    data = FS.load_json("tx")
    for gui_key, (section, key), kind in FS.LINKS["tx"]:
        v = data.get(section, {}).get(key)
        if v is not None:
            FS.set_gui(window, gui_key, kind, v)
    p = load_pluto()["Pluto"]
    for gui_key, (key, *_rest) in PLUTO_FIELDS.items():
        window[gui_key].update(str(p[key]))
    window["-MODE_FILE-" if str(p["Mode"]).lower() == "file" else "-MODE_RANDOM-"].update(True)
    window["-LOOPBACK-"].update(bool(int(p["Loopback"])))


def pluto_from_values(values) -> tuple[dict, list[str]]:
    """The Pluto settings from the GUI and a list of errors."""
    out, errors = {}, []
    for gui_key, (key, typ, lo, hi, label) in PLUTO_FIELDS.items():
        raw = str(values[gui_key]).strip()
        if typ is str:
            out[key] = raw
            continue
        try:
            v = typ(float(raw)) if typ is int and float(raw).is_integer() else typ(raw)
            if not lo <= v <= hi:
                raise ValueError
            out[key] = v
        except ValueError:
            errors.append(f"{label}: {'whole ' if typ is int else ''}number from {lo:g} to {hi:g}")
    if not out.get("Address"):
        errors.append("Pluto Address: enter the Pluto's IP address (192.168.2.1)")
    out["Mode"] = "File" if values["-MODE_FILE-"] else "Random"
    out["Loopback"] = int(bool(values["-LOOPBACK-"]))
    return out, errors


def tx_from_values(values) -> tuple[dict, list[str]]:
    """Parameters_Tx_payload.json with the GUI's Defined values, and a list of errors."""
    errors = FS.validate_int_fields(values, FS.TX_INT_LIMITS)
    data = FS.load_json("tx")
    for gui_key, (section, key), kind in FS.LINKS["tx"]:
        if kind.startswith("ro"):
            continue
        try:
            data.setdefault(section, {})[key] = FS.from_gui(values, gui_key, kind)
        except ValueError:
            errors.append(f"{FS.GUI_LABELS.get(gui_key, key)}: invalid value {values.get(gui_key)!r}")
    if not errors:  # settings the transmitter cannot take (e.g. no code word fits the packet)
        try:
            tx_config(data["Defined"]).check()
        except (AssertionError, KeyError, ValueError) as e:
            errors.append(f"These settings do not make a valid packet: {e or 'check failed'}")
    return data, errors


def tx_config(defined: dict) -> R.TxConfig:
    cfd = json.loads(CF_JSON.read_text())["Defined"] if CF_JSON.is_file() else None
    return R.TxConfig.from_defined(defined, cfd)


def save_settings(values) -> str:
    """Write Parameters_Tx_payload.json (and the shared settings into Parameters_Rx_payload.json,
    as Flexlinksimulation does) and App/Tx_Pluto/Parameters_Pluto.json. Raises ValueError, nothing written."""
    tx, errors = tx_from_values(values)
    pl, errors2 = pluto_from_values(values)
    errors += errors2
    if errors:
        raise ValueError("\n".join(errors))
    rx = FS.load_json("rx")
    rx_def = rx.setdefault("Defined_Rx", {})
    for key in FS.TX_RX_SHARED:
        if key in tx.get("Defined", {}):
            rx_def[key] = tx["Defined"][key]
    pdata = load_pluto()
    pdata["Pluto"].update(pl)
    FS.save_json("tx", tx)
    FS.save_json("rx", rx)
    write_json_atomic(PLUTO_JSON, pdata)
    return "Saved Parameters_Tx_payload.json, Parameters_Rx_payload.json, Parameters_Pluto.json."


def derived_text(values) -> str:
    """One line on what the settings give (from the transmitter model)."""
    try:
        tx, errors = tx_from_values(values)
        if errors:
            return ""
        cfg = tx_config(tx["Defined"])
        ns = cfg.samples
        return (f"{cfg.ndb} transport blocks of {cfg.tb_size} bits per packet,\n"
                f"{ns} samples ({ns / FS_PLUTO * 1e3:.2f} ms at 40 Msps)")
    except Exception:  # noqa: BLE001  (just a hint line)
        return ""


def update_enabled(window, values, running: bool) -> None:
    lb = bool(values["-LOOPBACK-"])
    file_mode = bool(values["-MODE_FILE-"]) and not lb
    for k in ("-MODE_RANDOM-", "-MODE_FILE-"):
        window[k].update(disabled=lb or running)
    for k in ("-FILE-", "-FILE_BROWSE-", "-FILE_REPEAT-"):
        window[k].update(disabled=not file_mode or running)
    for k in ("-PL_RXGAIN-", "-PL_CAPTURE-"):
        window[k].update(disabled=not lb or running)
    window["-LOOPBACK-"].update(disabled=running)


# ---------- Transmission: continuous random packets / file ----------
def radio_settings(pl: dict) -> dict:
    return {"TxLo": pl["TxLO_MHz"] * 1e6, "TxGain": pl["TxGain_dB"], "RfBandwidth": pl["RfBandwidth_MHz"] * 1e6}


def connect(pl: dict, cfg: R.TxConfig) -> tuple[ptx_client.PtxClient, dict]:
    """Connect to the Pluto's server, send SET_PARAMS and check its reply against the model."""
    try:
        c = ptx_client.PtxClient(pl["Address"], int(pl["ServerPort"]))
    except OSError as e:
        raise RuntimeError(f"no transmitter server at {pl['Address']}:{pl['ServerPort']} ({e}). Is "
                           f"ptx_server running on the Pluto? After a power cycle start it with: "
                           f"bash HW_Development/Packet_Transmitter/FW/pluto_start.sh") from e
    try:
        rep = c.set_defined(cfg.defined(), cfg.cf_defined(), int(pl["Gap"]), radio=radio_settings(pl))
        diff = ptx_client.check_reply(rep, cfg)
        if diff:
            raise RuntimeError("the Pluto and the PC disagree on the settings: " + "; ".join(diff))
    except BaseException:
        c.close()
        raise
    return c, rep


def file_source(path: Path, cfg: R.TxConfig, repeat: int, stop: threading.Event):
    """(description, transport blocks) for every packet of the file, `repeat` times (0: forever)."""
    from File_Framing import FileSender
    from Generate_File_Bitstream import default_file_id
    sender = FileSender.from_file(path, cfg.tb_size, cfg.ndb, default_file_id(path))
    plan = sender.packet_plan()
    k = 0
    while not stop.is_set() and (repeat == 0 or k < repeat):
        for i, blocks in enumerate(plan):
            bits = sender.packet_bits(blocks).reshape(cfg.ndb, cfg.tb_size)
            yield (f"file {sender.name}: pass {k + 1}{'' if repeat == 0 else f' of {repeat}'}, "
                   f"packet {i + 1} of {len(plan)} (blocks {blocks[0]}..{blocks[-1]} of {sender.total})"), bits
        k += 1


def random_source(cfg: R.TxConfig, stop: threading.Event):
    rng = np.random.default_rng()
    while not stop.is_set():
        yield "random packets", R.random_tbs(cfg, rng)


def tx_worker(window, pl: dict, stop: threading.Event) -> None:
    """Worker thread: SET_PARAMS, then one packet per credit until the source ends or Stop.
    Posts -TX_EVENT- {"kind": "started" | "stats" | "done", ...}."""
    def post(kind, **kw):
        window.write_event_value("-TX_EVENT-", {"kind": kind, **kw})

    c = None
    t0 = time.monotonic()
    seq = 0
    n_sent = 0                                    # SENT messages of this session
    last = {}
    what = ""
    error = None
    final = None
    try:
        cfg = tx_config(json.loads(TX_JSON.read_text())["Defined"])
        c, rep = connect(pl, cfg)
        bits_per_packet = rep["BlocksPerPacket"] * rep["TransportBlockSize"]
        post("started", text=(f"Pluto {pl['Address']}: {rep['BlocksPerPacket']} blocks of "
                              f"{rep['TransportBlockSize']} bits per packet, {rep['PacketSamples']} samples + "
                              f"gap {rep['Gap']}, {c.depth} credits, carrier {pl['TxLO_MHz']:g} MHz, "
                              f"Tx gain {pl['TxGain_dB']:g} dB"))
        if pl["Mode"] == "File":
            src = file_source(Path(pl["File"]), cfg, int(pl["FileRepeat"]), stop)
        else:
            src = random_source(cfg, stop)
        t_post = 0.0
        t0 = time.monotonic()
        for what, tbs in src:
            if stop.is_set():
                break
            c.send_packet(seq, tbs)
            seq += 1
            del c.events[:]                       # the client keeps every message: drop them
            if c.sent:
                n_sent += len(c.sent)
                last = c.sent[-1]
                del c.sent[:]
            now = time.monotonic()
            if now - t_post > 0.5:
                t_post = now
                post("stats", text=stats_line(what, seq, n_sent, last, now - t0, bits_per_packet))
        else:
            if not stop.is_set():                 # the file is through: wait until it is on air
                c.wait_sent(timeout=30)
                if c.sent:
                    n_sent += len(c.sent)
                    last = c.sent[-1]
    except Exception as e:  # noqa: BLE001  (shown in the GUI)
        error = f"{type(e).__name__}: {e}"
    finally:
        if c is not None:
            try:
                final = c.stop()
            except Exception as e:  # noqa: BLE001
                error = error or f"stopping: {e}"
            c.close()
    dt = time.monotonic() - t0
    post("done", error=error, final=final, packets=seq, on_air=n_sent, seconds=dt, last=last, what=what)


def stats_line(what: str, pushed: int, sent: int, last: dict, seconds: float, bits_per_packet: int) -> str:
    rate = sent * bits_per_packet / max(seconds, 1e-9) / 1e6
    return (f"{what}: {pushed} packet(s) pushed, {sent} on air, {seconds:.1f} s, "
            f"{rate:.2f} Mbit/s of transport blocks; PL packets {last.get('pl_packets', '-')}, "
            f"header errors {last.get('header_errors', '-')}, underflows {last.get('underflows', '-')}")


# ---------- Loopback ----------
def run_script(label: str, cmd: list, ok: set, log) -> tuple[int, str]:
    p = subprocess.run([sys.executable, *map(str, cmd)], capture_output=True, text=True, cwd=ROOT)
    out = p.stdout + p.stderr
    log.write(f"===== {label} (exit {p.returncode})\n{out}\n")
    log.flush()
    if p.returncode not in ok:
        raise StepFailed(label, p.returncode, out)
    return p.returncode, out


class StepFailed(RuntimeError):
    def __init__(self, label, rc, out):
        super().__init__(f"{label} stopped with exit code {rc}")
        self.label, self.rc, self.out = label, rc, out


def tx_blocks(cfg: R.TxConfig) -> np.ndarray:
    """The transport blocks of the Tx chain's last run: (AvailableDataBlocks, TransportBlockSize)."""
    import h5py
    path = TX / "Generate_RandomPacketOfData" / "Bitstream_TransportBlks_Tx.h5"
    with h5py.File(path, "r") as f:
        bits = np.asarray(f["TransportBlock_Bitstream"][()]).ravel().astype(np.int64)
    if bits.size != cfg.ndb * cfg.tb_size:
        raise ValueError(f"{path.name} has {bits.size} bits, the settings give {cfg.ndb} x {cfg.tb_size}: "
                         f"Initialize_Tx and the Tx chain did not run with these settings")
    return bits.reshape(cfg.ndb, cfg.tb_size)


def open_pluto(pl: dict, samples: int):
    """pyadi-iio Pluto for the capture (RX1, manual gain, 40 Msps)."""
    try:
        import adi
    except Exception as e:  # ImportError, or OSError when the libiio C library is missing
        raise RuntimeError(f"loopback needs pyadi-iio and the libiio C library ({e}): "
                           f"sudo apt install libiio0; pip install pyadi-iio 'pylibiio<1'") from e
    sdr = adi.Pluto(f"ip:{pl['Address']}")
    sdr.sample_rate = FS_PLUTO
    sdr.rx_lo = int(pl["TxLO_MHz"] * 1e6)
    sdr.tx_lo = int(pl["TxLO_MHz"] * 1e6)
    sdr.rx_rf_bandwidth = int(pl["RfBandwidth_MHz"] * 1e6)
    sdr.tx_rf_bandwidth = int(pl["RfBandwidth_MHz"] * 1e6)
    sdr.gain_control_mode_chan0 = "manual"
    sdr.rx_hardwaregain_chan0 = float(pl["RxGain_dB"])
    sdr.rx_enabled_channels = [0]
    sdr.rx_buffer_size = int(samples)
    if int(sdr.sample_rate) != FS_PLUTO:
        raise RuntimeError(f"the Pluto runs at {int(sdr.sample_rate)} sps, not {FS_PLUTO}")
    return sdr


def block_power(x: np.ndarray, n: int = 2048) -> np.ndarray:
    m = x.size // n
    return np.mean(np.abs(x[: m * n].reshape(m, n)) ** 2, axis=1)


def burst_power(x: np.ndarray) -> float:
    """Mean power over the blocks that carry the packet (over half the strongest block's power)."""
    p = block_power(x)
    return float(p[p > 0.5 * p.max()].mean()) if p.size and p.max() > 0 else 0.0


def capture_to_rx(x40: np.ndarray, tx_wave: np.ndarray, tx_rate: float) -> tuple[np.ndarray, float]:
    """40 Msps capture -> 20 Msps (low-pass and decimate by 2) at the level Decimate_Payload_Rx.py
    gives the simulator's waveform (a 40 MHz Tx waveform decimated by 2 is multiplied by sqrt(2)).
    Returns the waveform and the gain applied."""
    from scipy.signal import resample_poly
    x20 = resample_poly(x40.astype(np.complex128), 1, 2)
    ref = burst_power(tx_wave) * (2.0 if tx_rate == 2 * RX_RATE else 1.0)
    got = burst_power(x20)
    gain = float(np.sqrt(ref / got)) if got > 0 else 1.0
    return x20 * gain, gain


def find_packets(x20: np.ndarray, packet20: int, gap20: int, blk: int = 128) -> list[int]:
    """Start (20 Msps sample) of every whole packet in the capture: a rise of the power envelope
    after at least half a gap of quiet, followed by a packet's length of signal that fits in the
    capture. Empty when the gap is too short to see the edges."""
    min_off = max(2, int(0.5 * gap20 / blk))
    if gap20 < 4 * blk:
        return []
    p = block_power(x20, blk)
    pdb = 10 * np.log10(p + 1e-12)
    quiet = gap20 / (packet20 + gap20)              # share of the capture that is gap
    lo = np.percentile(pdb, 100 * quiet / 3)        # noise floor: well inside the gaps
    hi = np.percentile(pdb, 100 * (1 - (1 - quiet) / 2))   # signal: the middle of the packets
    if hi - lo < 6:                                 # no on/off pattern
        return []
    on = pdb > (lo + hi) / 2
    n_pkt = packet20 // blk
    starts = []
    for i in range(min_off, on.size - n_pkt + 1):
        if on[i] and not on[i - min_off:i].any() and on[i:i + n_pkt].mean() > 0.7:
            starts.append(i * blk)
    return starts


def write_rx_waveform(path: Path, x: np.ndarray, attrs: dict) -> None:
    import h5py
    path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(path, "w") as f:
        f.create_dataset("data", data=x.astype(np.complex128))
        for k, v in attrs.items():
            f.attrs[k] = v


def defects_off_file() -> Path:
    """A copy of Channel_Defects.json with every defect off: the capture is the channel."""
    d = json.loads((CHANNEL_DEFECTS / "Channel_Defects.json").read_text())
    for k in d.get("Mode", {}):
        if not k.endswith("_comment"):
            d["Mode"][k] = 0
    write_json_atomic(DEFECTS_OFF, d)
    return DEFECTS_OFF


def loopback_worker(window, pl: dict, stop: threading.Event) -> None:
    """Worker thread for one loopback run. Posts -LB_PROGRESS- (text) and -LB_DONE- (result)."""
    def progress(text):
        window.write_event_value("-LB_PROGRESS-", text)

    result = {"ok": False, "pluto": [], "start": time.time()}
    PLUTO_DIR.mkdir(parents=True, exist_ok=True)
    with LOOPBACK_LOG.open("w") as log:
        try:
            # 1. Tx chain: random blocks, the reference waveform and grid (no defects)
            progress("Running Initialize_Tx (1/5) ...")
            run_script("Initialize_Tx", [TX / "Initialize_Tx.py"], {0}, log)
            if stop.is_set():
                raise RuntimeError("stopped")
            progress("Running Run_Tx_Chain (2/5) ...")
            run_script("Run_Tx_Chain", [TX / "Run_Tx_Chain.py", "--no-defects", "--log", FS.TX_LOG], {0}, log)
            cfg = tx_config(json.loads(TX_JSON.read_text())["Defined"])
            tbs = tx_blocks(cfg)
            import Flexlink_Plots
            tx_path = TX / (json.loads(TX_JSON.read_text()).get("File", {}).get("Waveform_Payload_Tx")
                            or "Waveform_Payload_Tx.h5")
            tx_wave, tx_attrs = Flexlink_Plots.load_waveform(tx_path)
            tx_wave = tx_wave.ravel()
            if stop.is_set():
                raise RuntimeError("stopped")

            # 2. Pluto: the packet over and over, capture on RX1
            progress("Pluto: transmitting and capturing (3/5) ...")
            x40, pl_info = pluto_loopback(pl, cfg, tbs, log)
            result["pluto"] += pl_info
            np.save(CAPTURE_40, x40)

            # 3. the capture at 20 Msps; find the whole packets in it (the capture starts anywhere,
            #    usually inside a packet)
            x20, gain = capture_to_rx(x40, tx_wave, float(tx_attrs.get("SampleRate", FS_PLUTO)))
            packet20 = cfg.samples // 2
            gap20 = ((int(pl["Gap"]) + 15) // 16 * 16) // 2
            starts = find_packets(x20, packet20, gap20)
            margin = 1024
            if starts:
                cuts = [(max(0, s - margin), min(x20.size, s + packet20 + margin)) for s in starts[:MAX_TRIES]]
                result["pluto"].append(f"{len(starts)} whole packet(s) in the capture, at 20 Msps samples "
                                       + ", ".join(str(s) for s in starts[:8]) + (" ..." if len(starts) > 8 else ""))
            else:
                cuts = [(0, x20.size)]
                result["pluto"].append("No packet edges found (gap too short or no signal): the Rx chain gets "
                                       "the whole capture")

            # 4. Rx chain on one packet at a time, until one has a good control field
            progress("Running Initialize_Rx (4/5) ...")
            run_script("Initialize_Rx", [RX / "Initialize_Rx.py"], {0}, log)
            defects = defects_off_file()
            for k, (a, b) in enumerate(cuts):
                write_rx_waveform(CAPTURE_RX, x20[a:b], {
                    "SampleRate": RX_RATE, "Source": "Pluto loopback (FlexlinkTxWithPluto.py)",
                    "CaptureRate": FS_PLUTO, "CaptureStart20": a, "CarrierHz": pl["TxLO_MHz"] * 1e6,
                    "TxGain_dB": pl["TxGain_dB"], "RxGain_dB": pl["RxGain_dB"], "LevelGain": gain})
                progress(f"Running Run_Rx_Chain on packet {k + 1} of {len(cuts)} (5/5) ...")
                rc, out = run_script(f"Run_Rx_Chain (packet {k + 1}, samples {a}..{b})",
                                     [RX / "Run_Rx_Chain.py", "--no-model", "--input", CAPTURE_RX,
                                      "--defects-file", defects, "--log", FS.RX_LOG], {0, 1, 2, 4}, log)
                result["pluto"].append(f"Packet {k + 1} (samples {a}..{b}): {FS.RX_STATUS.get(rc, f'exit {rc}')}")
                if rc in (0, 1) or stop.is_set():   # control field good: these are the results
                    break
            result["pluto"].append(f"Rx chain input {CAPTURE_RX.relative_to(ROOT)}: 20 Msps (low-pass, decimated "
                                   f"by 2, scaled x{gain:.3g} to the simulator's level)")
            result["rx_status"] = rc
            try:
                rx_log = FS.RX_LOG.read_text()
            except OSError:
                rx_log = ""
            payload = FS.decode_output(rx_log)
            if rc == 4:
                payload = "No payload decoded: the control field CRC failed, so the packet was dropped."
            result["payload"] = payload if payload else out.rstrip()
            result["control"] = FS.control_output(rx_log)
            result["rate"] = rate_lines(cfg, int(pl["Gap"]), rx_log if rc in (0, 1) else "")
            result["ok"] = True
        except StepFailed as e:
            result.update(failed=e.label, rc=e.rc, tail="\n".join(e.out.rstrip().splitlines()[-25:]))
        except Exception as e:  # noqa: BLE001
            result.update(failed="Loopback", rc=None, tail=f"{type(e).__name__}: {e}")
            log.write(f"===== error\n{type(e).__name__}: {e}\n")
    window.write_event_value("-LB_DONE-", result)


def pluto_loopback(pl: dict, cfg: R.TxConfig, tbs: np.ndarray, log) -> tuple[np.ndarray, list[str]]:
    """Transmit the one packet over and over through the PL and capture it. Returns the capture
    (complex, 12-bit values) and report lines."""
    gap = (int(pl["Gap"]) + 15) // 16 * 16
    samples = int(pl["CaptureSamples"])
    lines = []
    sdr = open_pluto(pl, samples)
    c, rep = connect(pl, cfg)
    period = int(rep["PacketSamples"]) + gap
    if samples < (MAX_TRIES + 1) * period:          # room for MAX_TRIES whole packets
        samples = min((MAX_TRIES + 1) * period, 1 << 24)
        sdr.rx_buffer_size = samples
        lines.append(f"Capture lengthened to {samples} samples ({MAX_TRIES + 1} packet periods)")
    done, err = threading.Event(), []

    def feed():
        seq = 0
        try:
            while not done.is_set():
                c.send_packet(seq, tbs)
                seq += 1
                del c.events[:]
        except Exception as e:  # noqa: BLE001
            err.append(e)

    th = threading.Thread(target=feed, daemon=True)
    th.start()
    try:
        t_end = time.monotonic() + 5
        while len(c.sent) < 2:              # this session's packet is on air
            if err:
                raise err[0]
            if time.monotonic() > t_end:
                raise TimeoutError("the Pluto reports no packet sent after 5 s")
            time.sleep(0.01)
        for _ in range(2):                  # stale buffers
            sdr.rx()
        x = np.asarray(sdr.rx())
    finally:
        done.set()
        th.join(timeout=5)
        try:
            final = c.stop()
        except Exception as e:  # noqa: BLE001
            final = {"error": str(e)}
        c.close()
        try:
            sdr.rx_destroy_buffer()
        except Exception:  # noqa: BLE001
            pass
    if err:
        raise err[0]
    iq = np.stack([x.real, x.imag], axis=1)
    peak = float(np.abs(iq).max())
    rms = float(np.sqrt(np.mean(iq ** 2) * 2))
    burst = float(np.sqrt(burst_power(x)))
    lines.insert(0, f"Pluto {pl['Address']}: {rep['BlocksPerPacket']} blocks of {rep['TransportBlockSize']} bits, "
                    f"{rep['PacketSamples']} samples + gap {gap}, carrier {pl['TxLO_MHz']:g} MHz, "
                    f"Tx gain {pl['TxGain_dB']:g} dB, Rx gain {pl['RxGain_dB']:g} dB")
    lines.append(f"Capture: {x.size} samples at 40 Msps, rms {rms:.0f}, burst rms {burst:.0f}, peak {peak:.0f} "
                 f"of 2048" + ("  CLIPPING: lower the Rx gain (or the Tx gain)" if peak >= 2047 else
                              ("  weak: raise the Rx gain" if burst < 30 else "")))
    lines.append(f"PL after stop: {final}")
    log.write("===== Pluto\n" + "\n".join(lines) + "\n")
    return x, lines


# ---------- Loopback results and plots ----------
def rate_lines(cfg: R.TxConfig, gap: int, rx_log: str) -> list[str]:
    """Data rate from the transport block bits of one packet: during the packet (preamble and
    control field included), over packet + gap (what the Pluto sends continuously), and the part
    in blocks whose CRC passed."""
    import re
    gap = (int(gap) + 15) // 16 * 16
    bits = cfg.ndb * cfg.tb_size
    t_pkt = cfg.samples / FS_PLUTO
    t_cyc = (cfg.samples + gap) / FS_PLUTO
    lines = [f"Data rate: {cfg.ndb} x {cfg.tb_size} = {bits} transport block bits per packet",
             f"  in the packet ({cfg.samples} samples, {t_pkt * 1e3:.3f} ms):  {bits / t_pkt / 1e6:.3f} Mbit/s",
             f"  with the gap ({gap} samples, {t_cyc * 1e3:.3f} ms per packet):  {bits / t_cyc / 1e6:.3f} Mbit/s"]
    m = re.findall(r"CRC passed for (\d+) of (\d+) block", rx_log)
    if m:
        ok, n = map(int, m[-1])
        lines.append(f"  decoded: {ok} of {n} blocks CRC ok -> {ok * cfg.tb_size / t_cyc / 1e6:.3f} Mbit/s with "
                     f"the gap ({ok * cfg.tb_size / t_pkt / 1e6:.3f} in the packet)")
    return lines


def show_loopback_result(window, result) -> None:
    window["-RES_PLUTO-"].update("\n".join(result.get("pluto", [])))
    window["-RES_CNTL-"].update(result.get("control", ""))
    if result["ok"]:
        window["-RES_PAYLOAD-"].update(result["payload"])
        rc = result["rx_status"]
        window["-STATUS-"].update(f"Loopback finished: {FS.RX_STATUS.get(rc, f'exit {rc}')}. "
                                  f"Full output in {LOOPBACK_LOG.relative_to(ROOT)}.")
        try:
            import Flexlink_Plots
            extra = [Flexlink_Plots.evm_summary(RX_JSON, result["start"])]
        except Exception as e:  # noqa: BLE001
            extra = [f"EVM not available: {e}"]
        extra += result.get("rate", [])
        window["-RES_PAYLOAD-"].update(window["-RES_PAYLOAD-"].get().rstrip() + "\n\n" + "\n".join(extra))
        return
    window["-RES_PAYLOAD-"].update("")
    window["-STATUS-"].update(f"Loopback stopped: {result['failed']} failed"
                              + (f" (exit {result['rc']})." if result.get("rc") is not None else "."))
    sg.popup_scrolled(f"{result['failed']} failed:\n\n{result.get('tail', '')}\n\nFull output: {LOOPBACK_LOG}",
                      title="Loopback failed", size=(110, 25), font=RESULTS_FONT)


def make_loopback_plots(values, since: float) -> list[str]:
    """The selected plots of the last loopback run. Returns messages for the status line."""
    try:
        selected, n_symbols = FS.plot_selection(values)
    except (ValueError, OSError) as e:
        return [str(e)]
    if not any(selected.values()):
        return []
    import Flexlink_Plots as P
    msgs = []
    try:
        tx = json.loads(TX_JSON.read_text())
        rxp = json.loads(RX_JSON.read_text())
        if selected["tx_packet"] or selected["ccdf"] or selected["rx_packet"]:
            tx_path = TX / (tx.get("File", {}).get("Waveform_Payload_Tx") or "Waveform_Payload_Tx.h5")
            tx_wave, tx_attrs = P.load_waveform(tx_path)
            tx_wave = tx_wave.ravel()
            rx_wave = rx_attrs = None
            note = ""
            if selected["rx_packet"] or selected["ccdf"]:
                name = rxp.get("File_Rx", {}).get("Waveform_Payload_Packet_Rx") or "Decimated_Payload_Packet_Rx.h5"
                pkt = RX / ray_file_name(name, 1)
                if pkt.is_file() and pkt.stat().st_mtime >= since:
                    rx_wave, rx_attrs = P.load_waveform(pkt)
                    note = "Pluto loopback: the packet the Rx chain found (20 Msps)"
                else:
                    rx_wave, rx_attrs = P.load_waveform(CAPTURE_RX)
                    note = "Pluto loopback: the whole capture (20 Msps), the Rx chain found no packet"
                rx_wave = np.atleast_2d(rx_wave)[0]
            if selected["tx_packet"]:
                P.plot_tx_packet(tx_wave, tx_attrs)
            if selected["rx_packet"]:
                P.plot_rx_packet(rx_wave, rx_attrs, note)
            if selected["ccdf"]:
                start = int(tx_attrs.get("PayloadStart", 0))
                rx_start = int(rx_attrs.get("PayloadStart", 0))
                P.plot_ccdf([("Transmitted", tx_wave[start:]), ("Received (Pluto)", rx_wave[rx_start:])])
    except Exception as e:  # noqa: BLE001
        msgs.append(f"Packet plots: {e}")
    rest = {k: (v if k in ("before", "after", "evm_sc", "evm_sym") else False) for k, v in selected.items()}
    if any(rest.values()):
        try:
            msgs += P.make_plots(rest, n_symbols, RX_JSON, since)
        except Exception as e:  # noqa: BLE001
            msgs.append(f"Plots failed: {e}")
    return msgs


# ---------- Window ----------
def set_running(window, values, running: bool, active_tab: str) -> None:
    window.metadata = running
    window["-SAVE-"].update(disabled=running or active_tab != "-TAB_TX-")
    window["-START-"].update(disabled=running)
    window["-STOP-"].update(disabled=not running)
    update_enabled(window, values, running)


def append(window, key, text) -> None:
    window[key].update(window[key].get().rstrip("\n") + ("\n" if window[key].get().strip() else "") + text)


def main():
    layout = [[sg.TabGroup([[
        sg.Tab("Tx", tx_tab, key="-TAB_TX-"),
        sg.Tab("Received", rx_tab, key="-TAB_RX-"),
    ]], key="-TABS-", enable_events=True, expand_x=True, expand_y=True)],
        [sg.Push(), sg.Button("Save", key="-SAVE-", size=10),
         sg.Button("Start", key="-START-", size=10), sg.Button("Stop", key="-STOP-", size=10, disabled=True),
         sg.Push()],
        [sg.Text("", key="-STATUS-", size=(110, 1), justification="center")]]
    window = sg.Window("Flexlink Tx with Pluto", layout, finalize=True, resizable=True)
    window.metadata = False                    # True while transmitting or a loopback run
    stop = threading.Event()
    last_run = None                            # start time of the last finished loopback run
    stats_line_shown = False

    try:
        load_into_window(window)
        window["-STATUS-"].update("Loaded Parameters_Tx_payload.json and Parameters_Pluto.json")
    except (OSError, ValueError, KeyError) as e:
        sg.popup_error(f"Could not load the parameter files:\n{e}")
    values = window.read(timeout=0)[1]
    update_enabled(window, values, False)
    window["-TX_DERIVED-"].update(derived_text(values))

    while True:
        event, values = window.read()
        if event == sg.WIN_CLOSED:
            if window.metadata:
                stop.set()
            break
        active_tab = values["-TABS-"]
        if event == "-TABS-":
            set_running(window, values, window.metadata, active_tab)
            continue
        if event in ("-LOOPBACK-", "-MODE_RANDOM-", "-MODE_FILE-"):
            update_enabled(window, values, window.metadata)
            continue
        if event == "-SAVE-":
            try:
                window["-STATUS-"].update(save_settings(values))
                window["-TX_DERIVED-"].update(derived_text(values))
            except (OSError, ValueError) as e:
                sg.popup_error(f"Nothing was saved:\n{e}", title="Save failed")
            continue
        if event == "-START-":
            try:
                save_settings(values)
            except (OSError, ValueError) as e:
                sg.popup_error(f"Not started, nothing was saved:\n{e}", title="Invalid settings")
                continue
            pl = load_pluto()["Pluto"]
            stop.clear()
            if pl["Loopback"]:
                try:
                    FS.plot_selection(values)      # a bad symbol count, before a long run
                except (ValueError, OSError) as e:
                    sg.popup_error(str(e), title="Invalid plot settings")
                    continue
                for key in ("-RES_PLUTO-", "-RES_CNTL-", "-RES_PAYLOAD-"):
                    window[key].update("")
                window["-TX_STATUS-"].update("Loopback run started")
                window["-TAB_RX-"].select()        # the Received tab
                active_tab = "-TAB_RX-"
                set_running(window, values, True, active_tab)
                window.start_thread(lambda: loopback_worker(window, pl, stop), "-THREAD_END-")
            else:
                if pl["Mode"] == "File" and not Path(pl["File"]).is_file():
                    sg.popup_error(f"File not found: {pl['File'] or '(none selected)'}", title="Selected file")
                    continue
                window["-TX_STATUS-"].update("Connecting ...")
                stats_line_shown = False
                set_running(window, values, True, active_tab)
                window.start_thread(lambda: tx_worker(window, pl, stop), "-THREAD_END-")
            continue
        if event == "-STOP-":
            stop.set()
            window["-STOP-"].update(disabled=True)
            window["-STATUS-"].update("Stopping ...")
            continue
        if event == "-TX_EVENT-":
            ev = values[event]
            if ev["kind"] == "started":
                window["-TX_STATUS-"].update(ev["text"])
                window["-STATUS-"].update("Transmitting ... (Stop ends it)")
            elif ev["kind"] == "stats":
                lines = window["-TX_STATUS-"].get().rstrip("\n").splitlines()
                if stats_line_shown and lines:
                    lines = lines[:-1]
                window["-TX_STATUS-"].update("\n".join(lines + [ev["text"]]))
                stats_line_shown = True
            else:                                   # done
                set_running(window, values, False, active_tab)
                if ev["error"]:
                    append(window, "-TX_STATUS-", f"ERROR: {ev['error']}")
                    window["-STATUS-"].update("Transmission stopped with an error (see Transmitter).")
                else:
                    window["-STATUS-"].update(f"Transmission finished: {ev['on_air']} packet(s) on air in "
                                              f"{ev['seconds']:.1f} s.")
                last = ev["last"] or {}
                append(window, "-TX_STATUS-",
                       f"Stopped: {ev['packets']} packet(s) pushed, {ev['on_air']} on air ({ev['what']}). "
                       f"PL packet counter {last.get('pl_packets', '-')}, header errors "
                       f"{last.get('header_errors', '-')}, underflows {last.get('underflows', '-')}"
                       + ("; transmitter switched off" if (ev["final"] or {}).get("stopped") else ""))
            continue
        if event == "-LB_PROGRESS-":
            window["-STATUS-"].update(values[event])
            append(window, "-TX_STATUS-", values[event])
            continue
        if event == "-LB_DONE-":
            set_running(window, values, False, active_tab)
            result = values[event]
            show_loopback_result(window, result)
            append(window, "-TX_STATUS-", "\n".join(result.get("pluto", []))
                   + ("" if result["ok"] else f"\nLoopback failed: {result['failed']}"))
            if result["ok"]:
                last_run = result["start"]
                window["-REPLOT-"].update(disabled=False)
                msgs = make_loopback_plots(values, last_run)   # the boxes ticked now
                if msgs:
                    window["-STATUS-"].update(window["-STATUS-"].get() + "  Plots: " + "; ".join(msgs))
            continue
        if event == "-REPLOT-":
            if last_run is not None:
                msgs = make_loopback_plots(values, last_run)
                window["-STATUS-"].update("Replotted." + ("  Plots: " + "; ".join(msgs) if msgs else ""))
            continue
        if event == "-THREAD_END-":
            continue

    if "matplotlib.pyplot" in sys.modules:
        sys.modules["matplotlib.pyplot"].close("all")
    window.close()


if __name__ == "__main__":
    main()
