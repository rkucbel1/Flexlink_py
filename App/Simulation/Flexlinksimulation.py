"""Flexlinksimulation.py -- Flexlink GUI.

Reads and writes the project's JSON parameter files (located via flexlink_paths):
    Flexlink_BasicModules/Channel_Defects/Channel_Defects.json   -> "Set Defects" tab
    Tx/Parameters_Tx_payload.json                                 -> "Set Tx" tab
    Rx/Parameters_Rx_payload.json                                 -> "Set Rx" tab
    App/Simulation/Parameters_HIL.json                                       -> "Set Rx" tab, Receiver frame

Only the keys shown in the GUI are changed on Save; every other key and all the
*_comment entries are kept as they are.

Run (Simulate tab) runs, in a background thread:
    Tx/Initialize_Tx.py -> Tx/Run_Tx_Chain.py -> Rx/Initialize_Rx.py -> Rx/Run_Rx_Chain.py
and shows the Get_Decoded_Bitstream.py output in "Payload Field Results", and the control
field construction (Tx/ControlSignals/Build_Control_Field.log, written by Initialize_Tx.py)
and its decoding in the Rx chain in "Control Field Results". Each chain's full output is
kept in Tx/Run_Tx_Chain.log and Rx/Run_Rx_Chain.log.

With "Hardware in the loop" saved on the Set Rx tab, Run replaces the Rx steps by App/Simulation/Run_HIL.py:
the waveform goes through the Zedboard FPGA receiver, and the decoded transport blocks are
checked against the ones the Tx sent (App/Simulation/HIL/HIL_result.json, App/Simulation/HIL/HIL_results.csv).

Requires: pip install FreeSimpleGUI
Run with: python App/Simulation/Flexlinksimulation.py
"""
import json
import math
import os
import re
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import FreeSimpleGUI as sg

for _p in Path(__file__).resolve().parents:      # put the project root (flexlink_paths.py) on sys.path
    if (_p / "flexlink_paths.py").is_file():
        if str(_p) not in sys.path:
            sys.path.insert(0, str(_p))
        break
from flexlink_paths import APP, APP_SIMULATION, CHANNEL_DEFECTS, ROOT, RX, TX  # noqa: E402

sg.theme("SystemDefault")

H1 = ("Helvetica", 16, "bold")
H2 = ("Helvetica", 12, "bold")
LABEL_W = 20
MAX_RAYS = 4


def num_row(label, key, default):
    """Right-aligned label + numeric input field."""
    return [sg.Text(label, size=(LABEL_W, 1), justification="right"),
            sg.Input(default, key=key, size=10, justification="right")]


# Integer spinner fields with allowed ranges: key -> (label, min, max)
INT_LIMITS = {
    "-NUM_RAYS-": ("Number of Rays", 1, MAX_RAYS),
    "-SNR1-": ("SNR1 (dB)", -30, 70),
    "-SNR2-": ("SNR2 (dB)", -30, 70),
    "-SNR3-": ("SNR3 (dB)", -30, 70),
    "-SNR4-": ("SNR4 (dB)", -30, 70),
    "-SAMPLE_OFFSET-": ("Sample Offset", -200, 200),
    "-DRIFT-": ("Drift (ppm)", -100, 100),
}
SNR_KEYS = ["-SNR1-", "-SNR2-", "-SNR3-", "-SNR4-"]


def int_row(label, key, default, events=False):
    """Label + integer spinner limited to the range in INT_LIMITS."""
    _, lo, hi = INT_LIMITS[key]
    return [sg.Text(label, size=(LABEL_W, 1), justification="right"),
            sg.Spin(list(range(lo, hi + 1)), initial_value=default, key=key,
                    size=9, readonly=False, enable_events=events)]


def ro_list_row(label, key):
    """Label + read-only field showing a list of numbers from the JSON."""
    return [sg.Text(label, size=(LABEL_W, 1), justification="right"),
            sg.Input("", key=key, size=36, readonly=True)]


def fmt_num(x):
    """Compact number: 1000.0 -> 1e3, 250000.0 -> 2.5e5, -100 -> -100."""
    if x and (abs(x) >= 1e3 or abs(x) < 1e-3):
        mant, exp = f"{x:.3e}".split("e")
        return f"{mant.rstrip('0').rstrip('.')}e{int(exp)}"
    return f"{x:g}"


def int_input_row(label, key, default):
    """Label + typed integer field; its range in INT_LIMITS is checked on Save."""
    return [sg.Text(label, size=(LABEL_W, 1), justification="right"),
            sg.Input(str(default), key=key, size=10, justification="right")]


# Typed integer fields on the Tx tab: C_init_Dat seeds the Gold code payload scrambler;
# it is sent in the 8-bit control field C_init, so 0 to 255
TX_INT_LIMITS = {
    "-TX_C_INIT-": ("C_Init", 0, 255),
}


def validate_int_fields(values, limits=INT_LIMITS):
    """Return a list of error messages (empty list means everything is valid)."""
    errors = []
    for key, (label, lo, hi) in limits.items():
        try:
            v = int(str(values[key]).strip())
        except ValueError:
            errors.append(f"{label}: must be an integer between {lo} and {hi}")
            continue
        if not lo <= v <= hi:
            errors.append(f"{label}: must be between {lo} and {hi}")
    return errors


# ---------- Tab 1: Set Defects ----------
defect_names = [
    ("Multipath", "-CB_MULTIPATH-"),
    ("Frequency Offset", "-CB_FREQ-"),
    ("Thermal Noise", "-CB_THERMAL-"),
    ("IQ Imbalance", "-CB_IQ-"),
    ("Timing Offset", "-CB_TOFFSET-"),
    ("Timing Drift", "-CB_TDRIFT-"),
    ("Phase Noise", "-CB_PHASENOISE-"),
    ("DC Offset", "-CB_DCOFFSET-"),
    ("CW Interferer", "-CB_CW-"),
]

checkbox_col = [[sg.Text("Turn On/Off Defects", font=H2)]]
checkbox_col += [[sg.Checkbox(name, key=key, pad=(5, 8))] for name, key in defect_names]

# Radio key -> Settings/MultipathModel value in Channel_Defects.json.
# The delay/power profiles themselves live in Settings/MultipathProfiles.
MP_MODELS = {
    "-MP_FIR-": "FIR",
    "-MP_EPA-": "EPA",
    "-MP_EVA-": "EVA",
    "-MP_ETU-": "ETU",
}
MP_KEYS = list(MP_MODELS)

multipath_frame = sg.Frame("Multipath Model", [
    [sg.Radio("FIR: Small delay w/ no doppler", "MPATH", key="-MP_FIR-", default=True)],
    [sg.Radio("EPA: Low delay w/ doppler", "MPATH", key="-MP_EPA-")],
    [sg.Radio("EVA: Medium delay w/ doppler", "MPATH", key="-MP_EVA-")],
    [sg.Radio("ETU: High delay w/ doppler", "MPATH", key="-MP_ETU-")],
], font=H2, size=(280, 140))

# Terminal velocity that gives the Doppler frequency at each carrier frequency:
# f_D = v * f_c / c  ->  v = f_D * c / f_c. Recomputed when the tab is loaded and saved.
SPEED_OF_LIGHT = 299_792_458.0                          # m/s
CARRIER_FREQS_GHZ = [1, 2, 3, 4, 5, 6, 10, 15, 20, 25]


def velocity_kmh(doppler_hz, carrier_ghz):
    """Terminal velocity (km/h) for a maximum Doppler shift doppler_hz at carrier_ghz."""
    return doppler_hz * SPEED_OF_LIGHT / (carrier_ghz * 1e9) * 3.6


velocity_table = sg.Column(
    [[sg.Text("Carrier (GHz)", size=(12, 1), justification="right", font=("Helvetica", 10, "bold")),
      sg.Text("Velocity (km/h)", size=(14, 1), justification="right", font=("Helvetica", 10, "bold"))]]
    + [[sg.Text(f"{f:g}", size=(12, 1), justification="right", pad=(5, 1)),
        sg.Text("-", key=f"-VEL_{i}-", size=(14, 1), justification="right", pad=(5, 1))]
       for i, f in enumerate(CARRIER_FREQS_GHZ)]
    + [[sg.Text("v = Doppler x c / carrier frequency", font=("Helvetica", 9, "italic"))]],
    pad=((LABEL_W * 8 - 90, 0), (5, 0)))


def update_velocity_table(window, doppler):
    """Fill the velocity column from the Doppler value (Hz), rounded up to a whole km/h;
    '-' if it is not a number."""
    try:
        fd = abs(float(str(doppler).strip()))
    except ValueError:
        fd = None
    for i, f in enumerate(CARRIER_FREQS_GHZ):
        window[f"-VEL_{i}-"].update("-" if fd is None else f"{math.ceil(velocity_kmh(fd, f) - 1e-9)}")


left_col = sg.Column(checkbox_col + [
    [sg.Text("")],
    [multipath_frame],
    num_row("Doppler (Hz)", "-DOPPLER-", "300"),
    [velocity_table],
], vertical_alignment="top")

right_col = sg.Column([
    [sg.Text("Defect Values", font=H2)],
    num_row("Frequency Offset (Hz)", "-FREQ_OFFSET-", "2000"),
    [sg.Text(f"Up to {MAX_RAYS} rays (receive branches) supported",
             font=("Helvetica", 10, "bold"))],
    int_row("Number of Rays", "-NUM_RAYS-", 1, events=True),
    int_input_row("SNR1 (dB)", "-SNR1-", 35),
    int_input_row("SNR2 (dB)", "-SNR2-", 35),
    int_input_row("SNR3 (dB)", "-SNR3-", 35),
    int_input_row("SNR4 (dB)", "-SNR4-", 35),
    [sg.Text("")],
    num_row("Amp. Imbalance (dB)", "-AMP_IMB-", "-0.1"),
    num_row("Phase Imbalance (rad)", "-PHASE_IMB-", "0.00157"),
    int_row("Sample Offset", "-SAMPLE_OFFSET-", -1),
    int_row("Drift (ppm)", "-DRIFT-", 10),
    num_row("DC Offset (dBc)", "-DC_OFFSET-", "-30"),
    num_row("CW Frequency (Hz)", "-CW_FREQ-", "1100000"),
    num_row("CW Level (dBc)", "-CW_LEVEL-", "-20"),
    [sg.Text("")],
    [sg.Text("Phase Noise Mask (read only, set in Channel_Defects.json)",
             font=("Helvetica", 10, "bold"))],
    ro_list_row("Offsets (Hz)", "-PN_OFFSETS-"),
    ro_list_row("Levels (dBc/Hz)", "-PN_LEVELS-"),
], vertical_alignment="top")

defects_tab = [
    [sg.Text("Flexlink - Set Channel and Defect Parameters", font=H1,
             justification="center", expand_x=True, pad=(0, 15))],
    [sg.Push(), left_col, sg.Push(), right_col, sg.Push()],
]

# ---------- Tab 2: Set Tx ----------
GREY = "grey60"
TX_LABEL_W = 28


def combo_row(label, key, options, default, fixed=False):
    """Right-aligned label + dropdown. fixed=True greys it out (read-only);
    editable ones get a white background."""
    colors = {} if fixed else {"background_color": "white", "text_color": "black"}
    return [sg.Text(label, size=(TX_LABEL_W, 1), justification="right",
                    text_color=GREY if fixed else None),
            sg.Combo(options, default_value=default, key=key, size=10,
                     readonly=True, disabled=fixed, pad=(5, 8), **colors)]


# Option lists match the *_comment entries in Parameters_Tx_payload.json.
# Number of Tx Antennas is greyed out (fixed) and shows the file's value.
tx_edit_col = sg.Column([
    combo_row("LDPC Code Block Size", "-TX_LDPC_BLOCK-", [648, 1296, 1944], 1296),
    combo_row("LDPC Encoding Rate", "-TX_LDPC_RATE-", ["1/2", "2/3", "3/4", "5/6"], "5/6"),
    combo_row("Rate Matching (Repetition)", "-TX_REPETITION-",
              [1, 3, 5, 7, 11, 15, 19, 23, 31], 15),
    combo_row("Payload Field Modulation", "-TX_PAYLOAD_MOD-",
              ["BPSK", "QPSK", "16QAM", "64QAM"], "QPSK"),
    combo_row("Packet Length (Symbols)", "-TX_PACKET_LEN-", [37, 49, 73, 97, 121], 73),
    combo_row("Reference Symbol Spacing", "-TX_REF_SYM_SPACING-", [1, 3, 6, 12], 3),
    combo_row("Reference Signal Spacing", "-TX_REF_SIG_SPACING-", [3, 6, 12, 24], 3),
    [sg.Text("C_Init", size=(TX_LABEL_W, 1), justification="right"),
     sg.Input("130", key="-TX_C_INIT-", size=12, justification="right", pad=(5, 8))],
    combo_row("Number of Tx Antennas", "-TX_NUM_ANT-", [1], 1, fixed=True),
], vertical_alignment="top")

tx_tab = [
    [sg.Text("Flexlink - Set Transmitter Parameters", font=H1,
             justification="center", expand_x=True, pad=(0, 15))],
    [sg.Push(), tx_edit_col, sg.Push()],
]

# ---------- Tab 3: Set Rx ----------
RX_LABEL_W = 22
EQ_MODES = ["LMMSE", "LMMSE_LinearTime", "PCA", "Linear", "None"]
TIMING_METHODS = ["Full Precision Taps", "1-bit Taps", "FFT method"]
CFO_METHODS = ["FFT", "Lag cascade"]
DET_NORMALISERS = ["Original", "Bounded"]
DET_QUALIFIERS = [0, 32, 64, 128, 256]


def rx_combo(label, key, options, default):
    return [sg.Text(label, size=(RX_LABEL_W, 1), justification="right"),
            sg.Combo(options, default_value=default, key=key, size=19,
                     readonly=True, pad=(5, 8),
                     background_color="white", text_color="black")]


# Receiver path: the Python Rx chain or hardware in the loop (App/Simulation/Run_HIL.py, Zedboard FPGA).
# Radio key -> HIL/RxPath in App/Simulation/Parameters_HIL.json.
RX_PATHS = {
    "-RX_PATH_SIM-": "Simulation",
    "-RX_PATH_HIL-": "HIL",
}
HIL_RATES = [20, 40]
HIL_DEFAULTS = {"RxPath": "Simulation", "BoardHost": "192.168.50.10", "BoardPort": 5001, "InputRate": 20}
# Rx settings the FPGA does not use (its algorithms are fixed): greyed out with HIL. The detector
# thresholds, DC blocker, tone check and timing peak stay: the FPGA takes those.
RX_SIM_ONLY = ["-RX_NUM_ANT-", "-RX_FFT_START-", "-RX_DET_NORM-", "-RX_CFO-", "-RX_TIMING-",
               "-RX_EQ_MODE-", "-RX_EQ_DOPPLER-", "-RX_DEMAP-"]
HIL_ONLY = ["-HIL_HOST-", "-HIL_RATE-"]

rx_path_frame = sg.Frame("Receiver", [
    [sg.Radio("Simulation: Python Rx chain", "RXPATH", key="-RX_PATH_SIM-", default=True,
              enable_events=True)],
    [sg.Radio("Hardware in the loop: Zedboard FPGA", "RXPATH", key="-RX_PATH_HIL-",
              enable_events=True)],
    [sg.Text("Board Address", size=(RX_LABEL_W, 1), justification="right"),
     sg.Input(HIL_DEFAULTS["BoardHost"], key="-HIL_HOST-", size=21)],
    rx_combo("FPGA Input Rate (Msps)", "-HIL_RATE-", HIL_RATES, 20),
    [sg.Text("HIL takes one waveform (Number of Rays 1) and checks the decoded transport\n"
             "blocks against the Tx. Greyed settings are fixed in the FPGA. 20 Msps: the PC\n"
             "decimates; 40 Msps: the FPGA's half-band does.", font=("Helvetica", 9, "italic"))],
], font=H2)

rx_tab = [
    [sg.Text("Flexlink - Set Receiver Parameters", font=H1,
             justification="center", expand_x=True, pad=(0, 15))],
    [sg.Push(), sg.Column([
        [rx_path_frame],
        [sg.Text("")],
        rx_combo("Number of Rx Antennas", "-RX_NUM_ANT-", [1, 2, 3, 4], 1),
        [sg.Text("FFT Start Time", size=(RX_LABEL_W, 1), justification="right"),
         sg.Input("-12", key="-RX_FFT_START-", size=10, justification="right"),
         sg.Text("Samples into CP")],
        [sg.Text("")],
        [sg.Text("Preamble", font=H2)],
        rx_combo("DC Blocker", "-RX_DCBLOCK-", [1, 0], 1),
        [sg.Text("Packet Det. Thresh Lo", size=(RX_LABEL_W, 1), justification="right"),
         sg.Input("0.2", key="-RX_DET_LO-", size=10, justification="right"),
         sg.Text("flag clears below")],
        [sg.Text("Packet Det. Thresh Hi", size=(RX_LABEL_W, 1), justification="right"),
         sg.Input("0.45", key="-RX_DET_HI-", size=10, justification="right"),
         sg.Text("flag sets above")],
        rx_combo("Packet Det. Normaliser", "-RX_DET_NORM-", DET_NORMALISERS, "Original"),
        rx_combo("Packet Det. Qualifier", "-RX_DET_QUAL-", DET_QUALIFIERS, 0),
        rx_combo("CFO Method", "-RX_CFO-", CFO_METHODS, "FFT"),
        [sg.Text("Tone Check (dB)", size=(RX_LABEL_W, 1), justification="right"),
         sg.Input("13", key="-RX_TONE_CHK-", size=10, justification="right"),
         sg.Text("0 = off; rejects DC / tone interference")],
        rx_combo("Timing Method", "-RX_TIMING-", TIMING_METHODS, "Full Precision Taps"),
        [sg.Text("Timing Peak/Mean Min", size=(RX_LABEL_W, 1), justification="right"),
         sg.Input("18", key="-RX_TIMING_PK-", size=10, justification="right"),
         sg.Text("0 = off; 1-bit / FFT only")],
        [sg.Text("")],
        [sg.Text("Equalizer", font=H2)],
        rx_combo("Equalizer Mode", "-RX_EQ_MODE-", EQ_MODES, "LMMSE"),
        [sg.Text("PCA performs best below 10 dB SNR", font=("Helvetica", 9, "italic"),
                 pad=((RX_LABEL_W * 8, 0), (0, 5)))],
        [sg.Text("Max Doppler (Hz)", size=(RX_LABEL_W, 1), justification="right"),
         sg.Input("300", key="-RX_EQ_DOPPLER-", size=10, justification="right"),
         sg.Text("worst case, for LMMSE / PCA")],
        [sg.Text("")],
        rx_combo("Demapping", "-RX_DEMAP-", ["soft", "hard"], "soft"),
    ]), sg.Push()],
]


# ---------- Tab 4: Simulate (output + plots) ----------
RESULTS_FONT = ("Courier", 9)


def results_row(label, key, height=5):
    return [sg.Text(label, size=(20, 1), justification="right"),
            sg.Multiline(size=(100, height), key=key, disabled=True, autoscroll=True,
                         font=RESULTS_FONT, horizontal_scroll=True, pad=(5, 15))]


# Plot selection (left side). Plots are drawn when a Run finishes, by Flexlink_Plots.py.
constellation_frame = sg.Frame("Constellation Plots", [
    [sg.Checkbox("Before Equalization", key="-PLOT_CONST_BEFORE-", pad=(5, 6))],
    [sg.Checkbox("After Equalization", key="-PLOT_CONST_AFTER-", pad=(5, 6))],
    [sg.Text("Symbols to plot"),
     sg.Input("15", key="-PLOT_SYMBOLS-", size=6, justification="right")],
    [sg.Text("Ray 1 only", font=("Helvetica", 9, "italic"))],
], font=H2, vertical_alignment="top", expand_x=True)

evm_frame = sg.Frame("EVM Plots", [
    [sg.Checkbox("EVM vs Subcarrier", key="-PLOT_EVM_SC-", pad=(5, 6))],
    [sg.Checkbox("EVM vs OFDM Symbol", key="-PLOT_EVM_SYM-", pad=(5, 6))],
    [sg.Text("Ray 1 after equalization,\nvs the transmitted symbols",
             font=("Helvetica", 9, "italic"))],
], font=H2, vertical_alignment="top", expand_x=True)

packet_frame = sg.Frame("Packet Plots", [
    [sg.Checkbox("Transmitted Packet", key="-PLOT_TX_PACKET-", pad=(5, 6))],
    [sg.Checkbox("Received Packet", key="-PLOT_RX_PACKET-", pad=(5, 6))],
    [sg.Checkbox("PAPR CCDF", key="-PLOT_CCDF-", pad=(5, 6))],
    [sg.Text("Received = channel output, ray 1.\nPAPR shown in the plot titles",
             font=("Helvetica", 9, "italic"))],
], font=H2, vertical_alignment="top", expand_x=True)

def sweep_row(label, key, default):
    return [sg.Text(label, size=(17, 1), justification="right"),
            sg.Input(default, key=key, size=6, justification="right")]


sweep_frame = sg.Frame("SNR Sweep", [
    sweep_row("Start SNR (dB)", "-SW_START-", "0"),
    sweep_row("Stop SNR (dB)", "-SW_STOP-", "20"),
    sweep_row("Step (dB)", "-SW_STEP-", "2"),
    sweep_row("Max packets/point", "-SW_MAXPKT-", "50"),
    sweep_row("Target block errors", "-SW_TARGET-", "100"),
    [sg.Button("Run Sweep", key="-SW_RUN-", size=10),
     sg.Button("Stop Sweep", key="-SW_STOP_BTN-", size=10, disabled=True)],
    [sg.Text("Uses the saved Tx/Rx/Defect settings;\nSNR applied to all rays;\nalways the simulated Rx chain",
             font=("Helvetica", 9, "italic"))],
], font=H2, vertical_alignment="top", expand_x=True)

plots_col = sg.Column([
    [sg.Text("")],
    [packet_frame],
    [constellation_frame],
    [evm_frame],
    [sweep_frame],
], vertical_alignment="top")

sim_tab = [
    [plots_col, sg.Push(), sg.Column([
        [sg.Text("")],
        results_row("Control Field Results", "-RES_CNTL-", height=12),
        results_row("Payload Field Results", "-RES_PAYLOAD-", height=16),
    ]), sg.Push()],
]

PLOT_KEYS = {"tx_packet": "-PLOT_TX_PACKET-", "rx_packet": "-PLOT_RX_PACKET-", "ccdf": "-PLOT_CCDF-",
             "before": "-PLOT_CONST_BEFORE-", "after": "-PLOT_CONST_AFTER-",
             "evm_sc": "-PLOT_EVM_SC-", "evm_sym": "-PLOT_EVM_SYM-"}


def plot_selection(values):
    """Selected plots and the symbol count, or raise ValueError with a message."""
    selected = {name: bool(values[key]) for name, key in PLOT_KEYS.items()}
    if not (selected["before"] or selected["after"]):  # only constellations use the count
        return selected, 0
    packet_len = load_json("tx").get("Defined", {}).get("PacketLength")
    try:
        n = int(str(values["-PLOT_SYMBOLS-"]).strip())
    except ValueError:
        n = None
    if n is None or not (1 <= n <= int(packet_len)):
        raise ValueError(f"Symbols to plot must be a whole number from 1 to {packet_len} (PacketLength)")
    return selected, n

# ---------- Run: Tx and Rx chains ----------
TX_LOG = TX / "Run_Tx_Chain.log"
RX_LOG = RX / "Run_Rx_Chain.log"
CONTROL_LOG = TX / "ControlSignals" / "Build_Control_Field.log"   # written by Initialize_Tx.py

# (label, script + arguments, exit codes that let the run continue)
# Run_Rx_Chain: 0 all blocks pass, 1 a block failed, 2 no packet, 4 bad packet (control field
# CRC failed) -> results still shown.
RUN_STEPS = [
    ("Initialize_Tx", [TX / "Initialize_Tx.py"], {0}),
    ("Run_Tx_Chain", [TX / "Run_Tx_Chain.py", "--log", TX_LOG], {0}),
    ("Initialize_Rx", [RX / "Initialize_Rx.py"], {0}),
    # Run_Tx_Chain just made the defects waveform, so the Rx chain must not remake it
    ("Run_Rx_Chain", [RX / "Run_Rx_Chain.py", "--no-model", "--log", RX_LOG], {0, 1, 2, 4}),
]
RX_STATUS = {0: "every CRC passed", 1: "a block failed", 2: "no packet detected or packet aborted",
             4: "bad packet, control field CRC failed"}

# Hardware in the loop: the Tx steps, then Run_HIL instead of the Rx chain.
# Run_HIL: 0 every transport block equal to the Tx, 1 a block missing / failed / different,
# 2 no packet, 4 control field CRC failed -> results shown; 5 board, 6 setup -> run failed.
HIL_DIR = APP_SIMULATION / "HIL"
HIL_LOG = HIL_DIR / "Run_HIL.log"
HIL_RESULT = HIL_DIR / "HIL_result.json"
RUN_STEPS_HIL = RUN_STEPS[:2] + [
    ("Run_HIL", [APP_SIMULATION / "Run_HIL.py", "--log", HIL_LOG], {0, 1, 2, 4}),
]
HIL_STATUS = {0: "every transport block received and equal to the Tx",
              1: "a transport block missing, failed or different from the Tx",
              2: "no packet detected or packet aborted", 4: "bad packet, control field CRC failed"}


def hil_mode():
    """True when the saved Rx path (App/Simulation/Parameters_HIL.json) is hardware in the loop."""
    try:
        return str(load_json("hil").get("HIL", {}).get("RxPath", "")).upper() == "HIL"
    except (OSError, ValueError):
        return False


def hil_problems():
    """Saved settings a HIL run cannot take (the FPGA has one receive path)."""
    problems = []
    try:
        defects = load_json("defects")
    except (OSError, ValueError) as e:
        return [str(e)]
    rays = defects.get("Settings", {}).get("NumberOfRays", 1)
    if any(int(v) for v in defects.get("Mode", {}).values()) and isinstance(rays, int) and rays > 1:
        problems.append(f"Number of Rays is {rays}: hardware in the loop takes one waveform. "
                        "Set Number of Rays to 1 on Set Defects (and Save).")
    return problems


def decode_output(log_text):
    """Output of the last Get_Decoded_Bitstream.py run in a Run_Rx_Chain log."""
    sections = re.split(r"^===== ", log_text, flags=re.M)
    decode = [s for s in sections if s.startswith("Decode")]
    if not decode:
        return None
    return decode[-1].split("\n", 1)[1].rstrip() if "\n" in decode[-1] else ""


_TIMESTAMP = re.compile(r"^\d\d:\d\d:\d\d (INFO|WARNING|ERROR|DEBUG) {1,4}")  # "hh:mm:ss LEVEL   " (level padded to 7)
# Lines of Build_Control_Field.log shown in Control Field Results
_TX_CONTROL_KEEP = re.compile(r"^(ControlMessage|  \w+ +\d+ bits|CRC Gen|BCC |Interleaved and repeated|"
                              r"len\(Sequence\)|Scrambled |BPSK:|Control field:|Error|Bad|.*ERROR)")


def control_output(log_text, hil_lines=None):
    """Control field construction (Build_Control_Field.log) and its decoding (Rx chain log,
    or hil_lines from Run_HIL)."""
    lines = ["Construction (Tx, Initialize_Tx.py):"]
    try:
        tx = CONTROL_LOG.read_text()
    except OSError:
        tx = None
    if tx is None:
        lines.append(f"  {CONTROL_LOG.name} not found - the control field has not been built")
    else:
        failed = re.search(r"^===== Control field step \d+: (\S+) \(exit [1-9]", tx, flags=re.M)
        for ln in tx.splitlines():
            ln = ln[17:] if _TIMESTAMP.match(ln) else ln     # 'hh:mm:ss ' + level padded to 7 + ' '
            if ln.startswith("ControlMessage saved"):
                continue
            if _TX_CONTROL_KEEP.match(ln) or (failed and ln.startswith("  ")):
                lines.append("  " + ln)
        if failed:
            lines.append(f"  stopped at {failed.group(1)}")
    lines.append("")
    if hil_lines is not None:
        lines.append("Reception (hardware in the loop, Run_HIL.py):")
        lines += ["  " + ln for ln in hil_lines] or ["  no result"]
        return "\n".join(lines)
    sections = [sec for sec in re.split(r"^===== ", log_text or "", flags=re.M)
                if sec.startswith("Control field")]
    lines.append("Reception (Rx, Process_Control_Field_Rx.py):")
    if not sections:
        lines.append("  not run (no packet detected, or Use_ControlField is 0)")
    else:
        for ln in sections[-1].split("\n", 1)[-1].splitlines():
            if ln.strip() and not ln.startswith("Wrote") and not re.match(r"^  \w+ +[01]+  -> ", ln):
                lines.append("  " + ln)
    return "\n".join(lines)


def hil_result(since):
    """This run's App/Simulation/HIL/HIL_result.json, or None."""
    try:
        if HIL_RESULT.stat().st_mtime < since:
            return None
        return json.loads(HIL_RESULT.read_text())
    except (OSError, ValueError):
        return None


def run_simulation(window, steps=RUN_STEPS, hil=False):
    """Worker thread: run the steps in order and post the result as a -RUN_DONE- event."""
    result = {"ok": False, "steps": [], "payload": "", "hil": hil}
    start = time.time()
    for i, (label, cmd, ok_codes) in enumerate(steps, start=1):
        window.write_event_value("-RUN_PROGRESS-", f"Running {label} ({i}/{len(steps)}) ...")
        try:
            p = subprocess.run([sys.executable, *map(str, cmd)], capture_output=True,
                               text=True, cwd=ROOT)
            rc, out = p.returncode, p.stdout + p.stderr
        except OSError as e:
            rc, out = -1, str(e)
        result["steps"].append((label, rc, out))
        if rc not in ok_codes:
            result["failed"] = label
            break
    else:
        result["ok"] = True
        rx_rc, rx_out = result["steps"][-1][1], result["steps"][-1][2]
        result["rx_status"] = rx_rc
        if hil:
            res = hil_result(start) or {}
            result["payload"] = "\n".join(res.get("payload_lines", [])) or rx_out.rstrip()
            result["control"] = control_output(None, res.get("control_lines", []))
            window.write_event_value("-RUN_DONE-", result)
            return
        try:
            rx_log = RX_LOG.read_text()
        except OSError:
            rx_log = ""
        payload = decode_output(rx_log)
        if rx_rc == 4:
            payload = "No payload decoded: the control field CRC failed, so the packet was dropped."
        # No decode step (e.g. no packet found): show the Rx chain summary instead
        result["payload"] = payload if payload else rx_out.rstrip()
    if any(label == "Initialize_Tx" for label, _, _ in result["steps"]):
        try:
            rx_log = RX_LOG.read_text() if result.get("ok") else ""
        except OSError:
            rx_log = ""
        result["control"] = control_output(rx_log)
    window.write_event_value("-RUN_DONE-", result)


def set_running(window, running, active_tab, sweep=None):
    """running: a Run or a sweep is in progress. sweep: the sweep folder while sweeping."""
    window.metadata = running
    if sweep is not None or not running:
        window.sweep_dir = sweep
    window["-SAVE-"].update(disabled=running or active_tab == "-TAB_SIM-")
    window["-RUN-"].update(disabled=running or active_tab != "-TAB_SIM-")
    window["-SW_RUN-"].update(disabled=running)
    window["-SW_STOP_BTN-"].update(disabled=not (running and window.sweep_dir))


# ---------- SNR sweep (App/Simulation/Run_SNR_Sweep.py) ----------
SWEEP_SCRIPT = APP_SIMULATION / "Run_SNR_Sweep.py"
SWEEP_FIELDS = {  # key -> (label, type, min, max)
    "-SW_START-": ("Start SNR", float, -30, 70),
    "-SW_STOP-": ("Stop SNR", float, -30, 70),
    "-SW_STEP-": ("Step", float, 0.1, 50),
    "-SW_MAXPKT-": ("Max packets/point", int, 1, 100000),
    "-SW_TARGET-": ("Target block errors", int, 1, 100000),
}
SWEEP_TABLE_HEADER = (f"{'SNR dB':>7} {'pkts':>5} {'missed':>6} {'BLER':>9} {'PER':>9} "
                      f"{'BER':>9} {'EVM dB':>7} {'meas SNR':>8} {'Goodput Mbps':>12} {'time s':>7}")


def sweep_settings(values):
    """Parsed sweep fields, or raise ValueError listing every bad one."""
    out, errors = {}, []
    for key, (label, typ, lo, hi) in SWEEP_FIELDS.items():
        try:
            v = typ(str(values[key]).strip())
            if not lo <= v <= hi:
                raise ValueError
            out[key] = v
        except ValueError:
            errors.append(f"{label}: {'whole ' if typ is int else ''}number from {lo} to {hi}")
    if not errors and out["-SW_STOP-"] < out["-SW_START-"]:
        errors.append("Stop SNR must not be below Start SNR")
    if errors:
        raise ValueError("\n".join(errors))
    return out


def sweep_table_line(row):
    def f(x, fmt):
        if isinstance(x, (int, float)) and x == x:  # x == x is False for NaN
            return format(x, fmt)
        return "-".rjust(int(fmt.split(".")[0]))
    return (f"{f(row['snr_db'], '7.1f')} {row['packets']:>5} {row['missed']:>6} "
            f"{f(row['bler'], '9.2e')} {f(row['per'], '9.2e')} {f(row['ber'], '9.2e')} "
            f"{f(row['evm_db'], '7.1f')} {f(row['measured_snr_db'], '8.1f')} "
            f"{f(row.get('goodput_mbps', float('nan')), '12.3f')} {f(row['seconds'], '7.0f')}")


def run_sweep(window, settings, out_dir):
    """Worker thread: run Run_SNR_Sweep.py and relay its PROGRESS / POINT / DONE lines."""
    cmd = [sys.executable, str(SWEEP_SCRIPT),
           "--start", settings["-SW_START-"], "--stop", settings["-SW_STOP-"],
           "--step", settings["-SW_STEP-"], "--max-packets", settings["-SW_MAXPKT-"],
           "--target-errors", settings["-SW_TARGET-"], "--out-dir", out_dir]
    tail = []
    try:
        p = subprocess.Popen([str(c) for c in cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             text=True, cwd=ROOT, bufsize=1)
        for line in p.stdout:
            line = line.rstrip("\n")
            kind, _, text = line.partition(" ")
            if kind == "PROGRESS":
                window.write_event_value("-SWEEP_PROGRESS-", text)
            elif kind == "POINT":
                window.write_event_value("-SWEEP_POINT-", json.loads(text))
            elif kind != "DONE":
                tail = (tail + [line])[-25:]
        rc = p.wait()
    except OSError as e:
        rc, tail = -1, [str(e)]
    window.write_event_value("-SWEEP_DONE-", {"rc": rc, "tail": "\n".join(tail), "dir": out_dir})


def show_run_result(window, result):
    window["-RES_CNTL-"].update(result.get("control", ""))
    if result["ok"]:
        window["-RES_PAYLOAD-"].update(result["payload"])
        rc = result["rx_status"]
        if result.get("hil"):
            window["-STATUS-"].update(f"HIL run finished: {HIL_STATUS.get(rc, f'exit {rc}')}. "
                                      f"Full output in {TX_LOG.name} and {HIL_LOG.name}; "
                                      f"every run is listed in App/Simulation/HIL/HIL_results.csv.")
            return
        window["-STATUS-"].update(f"Run finished: {RX_STATUS.get(rc, f'exit {rc}')}. "
                                  f"Full output in {TX_LOG.name} and {RX_LOG.name}.")
        return
    label, rc, out = result["steps"][-1]
    window["-RES_PAYLOAD-"].update("")
    window["-STATUS-"].update(f"Run stopped: {label} failed (exit {rc}).")
    tail = "\n".join(out.rstrip().splitlines()[-25:])
    log = {"Run_Tx_Chain": f"\n\nFull output: {TX_LOG}", "Run_Rx_Chain": f"\n\nFull output: {RX_LOG}",
           "Run_HIL": f"\n\nFull output: {HIL_LOG}"}
    sg.popup_scrolled(f"{label} stopped with exit code {rc}:\n\n{tail}{log.get(label, '')}",
                      title="Run failed", size=(110, 25), font=RESULTS_FONT)

# ---------- JSON link ----------
FILES = {
    "defects": CHANNEL_DEFECTS / "Channel_Defects.json",
    "tx": TX / "Parameters_Tx_payload.json",
    "rx": RX / "Parameters_Rx_payload.json",
    "hil": APP_SIMULATION / "Parameters_HIL.json",
}

# file -> [(gui_key, (section, json_key), kind), ...]
# kinds: bool (stored 0/1), int, number (int if whole, else float), float, text,
#        radio (MultipathModel), ro (read-only: loaded into the GUI, never saved)
LINKS = {
    "defects": [
        ("-CB_MULTIPATH-", ("Mode", "Multipath"), "bool"),
        ("-CB_FREQ-", ("Mode", "Freq_Offset"), "bool"),
        ("-CB_THERMAL-", ("Mode", "ThermalNoise"), "bool"),
        ("-CB_IQ-", ("Mode", "IQ_Imbalance"), "bool"),
        ("-CB_TOFFSET-", ("Mode", "TimingOffset"), "bool"),
        ("-CB_TDRIFT-", ("Mode", "TimingDrift"), "bool"),
        ("-CB_PHASENOISE-", ("Mode", "PhaseNoise"), "bool"),
        ("-CB_DCOFFSET-", ("Mode", "DC_Offset"), "bool"),
        ("-CB_CW-", ("Mode", "CW_Interferer"), "bool"),
        ("-NUM_RAYS-", ("Settings", "NumberOfRays"), "int"),
        ("-SNR1-", ("Settings", "SNR1_dB"), "int"),
        ("-SNR2-", ("Settings", "SNR2_dB"), "int"),
        ("-SNR3-", ("Settings", "SNR3_dB"), "int"),
        ("-SNR4-", ("Settings", "SNR4_dB"), "int"),
        ("-FREQ_OFFSET-", ("Settings", "FrequencyOffset"), "number"),
        ("-AMP_IMB-", ("Settings", "AmplitudeImbalance_dB"), "float"),
        ("-PHASE_IMB-", ("Settings", "PhaseImbalance"), "float"),
        ("-SAMPLE_OFFSET-", ("Settings", "Sample_Offset"), "int"),
        ("-DRIFT-", ("Settings", "Drift_ppm"), "int"),
        ("-DC_OFFSET-", ("Settings", "DCOffset_dBc"), "number"),
        ("-CW_FREQ-", ("Settings", "CW_Frequency"), "number"),
        ("-CW_LEVEL-", ("Settings", "CW_Level_dBc"), "number"),
        ("-DOPPLER-", ("Settings", "DopplerFreq"), "number"),
        (MP_KEYS, ("Settings", "MultipathModel"), "radio"),
        ("-PN_OFFSETS-", ("Settings", "PhaseNoise_Offsets_Hz"), "ro_list"),
        ("-PN_LEVELS-", ("Settings", "PhaseNoise_Levels_dBcHz"), "ro_list"),
    ],
    "tx": [
        ("-TX_LDPC_BLOCK-", ("Defined", "EncoderBlockSize"), "int"),
        ("-TX_LDPC_RATE-", ("Defined", "EncoderCodeRate"), "text"),
        ("-TX_REPETITION-", ("Defined", "Repetition"), "int"),
        ("-TX_PAYLOAD_MOD-", ("Defined", "Mod"), "text"),
        ("-TX_REF_SYM_SPACING-", ("Defined", "RefSymbolSpacing"), "int"),
        ("-TX_REF_SIG_SPACING-", ("Defined", "RefSignalSpacing"), "int"),
        ("-TX_NUM_ANT-", ("Defined", "NumberOfAntennas"), "ro"),
        ("-TX_PACKET_LEN-", ("Defined", "PacketLength"), "int"),
        ("-TX_C_INIT-", ("Defined", "C_init_Dat"), "int"),
    ],
    "rx": [
        ("-RX_NUM_ANT-", ("Defined_Rx", "NumberOfAntennas"), "int"),
        ("-RX_TIMING-", ("Defined_Rx", "TimingMethod"), "text"),
        ("-RX_TIMING_PK-", ("Defined_Rx", "TimingPeakThresh"), "number"),
        ("-RX_DCBLOCK-", ("Defined_Rx", "DCBlocker"), "int"),
        ("-RX_TONE_CHK-", ("Defined_Rx", "ToneCheckThresh"), "number"),
        ("-RX_CFO-", ("Defined_Rx", "CFOMethod"), "text"),
        ("-RX_DET_NORM-", ("Defined_Rx", "PacketDetNormaliser"), "text"),
        ("-RX_DET_QUAL-", ("Defined_Rx", "PacketDetQualifier"), "int"),
        ("-RX_DET_LO-", ("Defined_Rx", "PacketDetThreshLo"), "float"),
        ("-RX_DET_HI-", ("Defined_Rx", "PacketDetThreshHi"), "float"),
        ("-RX_FFT_START-", ("Defined_Rx", "FFT_StartTime"), "int"),
        ("-RX_EQ_MODE-", ("Defined_Rx", "EqualizerMode"), "text"),
        ("-RX_EQ_DOPPLER-", ("Defined_Rx", "EqualizerMaxDoppler"), "number"),
        ("-RX_DEMAP-", ("Defined_Rx", "Demapping"), "text"),
    ],
    "hil": [
        (list(RX_PATHS), ("HIL", "RxPath"), "radio"),
        ("-HIL_HOST-", ("HIL", "BoardHost"), "text"),
        ("-HIL_RATE-", ("HIL", "InputRate"), "int"),
    ],
}

# Tx settings the receiver must match. Saving the Tx tab copies these from
# Tx "Defined" to Rx "Defined_Rx" so the two files never disagree.
TX_RX_SHARED = ["EncoderCodeRate", "EncoderBlockSize", "Repetition", "CRC", "Mod",
                "PacketLength", "RefSignalSpacing", "RefSymbolSpacing", "Port",
                "Use_Preamble", "Use_ControlField", "C_init_Dat"]

# Which files each tab's Save button writes
TAB_FILES = {
    "-TAB_DEFECTS-": ["defects"],
    "-TAB_TX-": ["tx"],  # also writes TX_RX_SHARED into the Rx file
    "-TAB_RX-": ["rx", "hil"],
}

GUI_LABELS = {key: name for name, key in defect_names}
GUI_LABELS.update({k: label for k, (label, _, _) in {**INT_LIMITS, **TX_INT_LIMITS}.items()})
GUI_LABELS.update({
    "-FREQ_OFFSET-": "Frequency Offset", "-AMP_IMB-": "Amp. Imbalance",
    "-PHASE_IMB-": "Phase Imbalance", "-DOPPLER-": "Doppler",
    "-DC_OFFSET-": "DC Offset", "-CW_FREQ-": "CW Frequency", "-CW_LEVEL-": "CW Level",
    "-RX_FFT_START-": "FFT Start Time", "-RX_EQ_DOPPLER-": "Max Doppler",
    "-RX_DET_LO-": "Packet Det. Thresh Lo", "-RX_DET_HI-": "Packet Det. Thresh Hi",
    "-RX_TIMING_PK-": "Timing Peak/Mean Min", "-RX_TONE_CHK-": "Tone Check (dB)",
    "-HIL_HOST-": "Board Address", "-HIL_RATE-": "FPGA Input Rate",
})


def radio_map(keys):
    """The {radio key: JSON value} table of a radio group."""
    return RX_PATHS if keys[0] in RX_PATHS else MP_MODELS


def load_json(name):
    path = FILES[name]
    if name == "hil" and not path.is_file():  # first use: the defaults (written on Save)
        return {"HIL": dict(HIL_DEFAULTS)}
    try:
        with path.open(encoding="utf-8") as fh:
            data = json.load(fh)
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON in {path}: line {e.lineno}, col {e.colno}: {e.msg}")
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a JSON object at top level")
    return data


def _dumps(obj, level=0):
    """json.dumps(indent=2), but lists of plain values stay on one line."""
    pad, inner = "  " * level, "  " * (level + 1)
    if isinstance(obj, dict):
        if not obj:
            return "{}"
        items = [f"{inner}{json.dumps(k)}: {_dumps(v, level + 1)}" for k, v in obj.items()]
        return "{\n" + ",\n".join(items) + f"\n{pad}}}"
    if isinstance(obj, list):
        if all(not isinstance(x, (dict, list)) for x in obj):
            return "[" + ", ".join(json.dumps(x) for x in obj) + "]"
        items = [f"{inner}{_dumps(v, level + 1)}" for v in obj]
        return "[\n" + ",\n".join(items) + f"\n{pad}]"
    return json.dumps(obj)


def save_json(name, data):
    """Write atomically, so a crash can't leave a half-written file."""
    path = FILES[name]
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=path.name, suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(_dumps(data) + "\n")
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


def from_gui(values, gui_key, kind):
    """GUI value -> value to store in the JSON."""
    if kind == "radio":
        return radio_map(gui_key)[next(k for k in gui_key if values[k])]
    v = values[gui_key]
    if kind == "bool":
        return int(bool(v))
    if kind == "int":
        return int(str(v).strip())
    if kind == "float":
        return float(str(v).strip())
    if kind == "number":
        f = float(str(v).strip())
        return int(f) if f.is_integer() else f
    return str(v)


def set_gui(window, gui_key, kind, v):
    """JSON value -> GUI element."""
    if kind == "radio":
        table = radio_map(gui_key)
        key = next((k for k, m in table.items() if m.upper() == str(v).upper()), None)
        if key is None:
            raise ValueError(f"{v!r} is not one of {', '.join(table.values())}")
        window[key].update(True)
    elif kind == "bool":
        window[gui_key].update(bool(v))
    else:
        el = window[gui_key]
        if kind == "ro_list":
            el.update(", ".join(fmt_num(x) for x in v) if isinstance(v, list) else str(v))
        elif isinstance(el, sg.Combo) and kind == "ro":
            el.update(values=[v], value=v)  # fixed field: show the file's value
        elif isinstance(el, sg.Combo):
            el.update(value=v)
        elif isinstance(el, sg.Input):
            el.update(str(v))
        else:
            el.update(v)


def load_into_window(window, names):
    """Fill the GUI from the given JSON files. Returns a list of warnings."""
    warnings = []
    for name in names:
        data = load_json(name)
        for gui_key, (section, key), kind in LINKS[name]:
            v = data.get(section, {}).get(key)
            if v is None:  # leave the GUI default if the key is missing/null
                warnings.append(f"{FILES[name].name}: {section}/{key} missing, using GUI default")
                continue
            set_gui(window, gui_key, kind, v)
    update_snr_enabled(window, window["-NUM_RAYS-"].get())
    update_rx_path_enabled(window, window["-RX_PATH_HIL-"].get())
    return warnings


def save_from_window(values, names):
    """Update the given JSON files from the GUI. Nothing is written if any value is bad."""
    errors = []
    pending = {}
    for name in names:
        data = load_json(name)
        for gui_key, (section, key), kind in LINKS[name]:
            if kind.startswith("ro"):  # read-only: never written back
                continue
            try:
                data.setdefault(section, {})[key] = from_gui(values, gui_key, kind)
            except (ValueError, StopIteration):
                label = GUI_LABELS.get(gui_key, key) if isinstance(gui_key, str) else key
                errors.append(f"{label}: invalid value {values.get(gui_key)!r}"
                              if isinstance(gui_key, str) else f"{label}: nothing selected")
        pending[name] = data

    if "tx" in pending:  # mirror the shared Tx settings into the Rx file
        if "rx" not in pending:
            pending["rx"] = load_json("rx")
        tx_def = pending["tx"].get("Defined", {})
        rx_def = pending["rx"].setdefault("Defined_Rx", {})
        for key in TX_RX_SHARED:
            if key in tx_def:
                rx_def[key] = tx_def[key]

    if "rx" in pending:  # packet detector thresholds: 0 <= Lo < Hi <= 1
        rx_def = pending["rx"].get("Defined_Rx", {})
        lo, hi = rx_def.get("PacketDetThreshLo"), rx_def.get("PacketDetThreshHi")
        if isinstance(lo, (int, float)) and isinstance(hi, (int, float)) and not 0 <= lo < hi <= 1:
            errors.append(f"Packet Det. Thresh: need 0 <= Lo < Hi <= 1 (got Lo {lo:g}, Hi {hi:g})")
        tc = rx_def.get("ToneCheckThresh")
        if isinstance(tc, (int, float)) and tc < 0:
            errors.append(f"Tone Check (dB): must be 0 (off) or positive (got {tc:g})")
        pk = rx_def.get("TimingPeakThresh")
        if isinstance(pk, (int, float)) and pk < 0:
            errors.append(f"Timing Peak/Mean Min: must be 0 (off) or positive (got {pk:g})")

    if "hil" in pending:
        hil = pending["hil"].setdefault("HIL", {})
        hil["BoardHost"] = str(hil.get("BoardHost", "")).strip()
        if not hil["BoardHost"]:
            errors.append("Board Address: enter the board's IP address (192.168.50.10)")
        hil.setdefault("BoardPort", HIL_DEFAULTS["BoardPort"])

    if errors:
        raise ValueError("\n".join(errors))
    for name, data in pending.items():
        save_json(name, data)


def consistency_warnings():
    """Cross-file checks run after a save. Returns a list of warning strings."""
    warnings = []
    try:
        defects, rx = load_json("defects"), load_json("rx")
    except (OSError, ValueError):
        return warnings
    any_defect = any(defects.get("Mode", {}).values())
    rays = defects.get("Settings", {}).get("NumberOfRays", 1)
    ants = rx.get("Defined_Rx", {}).get("NumberOfAntennas", 1)
    if any_defect and isinstance(rays, int) and isinstance(ants, int) and ants > rays:
        warnings.append(f"Rx uses {ants} antennas but Channel_Defects models only {rays} ray(s).")
    return warnings


def update_rx_path_enabled(window, hil):
    """HIL: grey out the Rx-chain settings the FPGA does not use; the board settings only with HIL."""
    for key in RX_SIM_ONLY:
        window[key].update(disabled=bool(hil))
    for key in HIL_ONLY:
        window[key].update(disabled=not hil)


def update_snr_enabled(window, n_rays):
    """Grey out SNRk for rays beyond NumberOfRays."""
    try:
        n = int(n_rays)
    except (TypeError, ValueError):
        return
    for k, key in enumerate(SNR_KEYS, start=1):
        window[key].update(disabled=k > n)


# ---------- Window ----------
def main():
    layout = [[sg.TabGroup([[
        sg.Tab("Set Defects", defects_tab, key="-TAB_DEFECTS-"),
        sg.Tab("Set Tx", tx_tab, key="-TAB_TX-"),
        sg.Tab("Set Rx", rx_tab, key="-TAB_RX-"),
        sg.Tab("Simulate", sim_tab, key="-TAB_SIM-"),
    ]], key="-TABS-", enable_events=True, expand_x=True, expand_y=True)],
        # One shared button bar below the tabs, so Save/Run sit at the same
        # x,y position no matter which tab is showing.
        [sg.Push(), sg.Button("Save", key="-SAVE-", size=10),
         sg.Button("Run", key="-RUN-", size=10, disabled=True), sg.Push()],
        [sg.Text("", key="-STATUS-", size=(90, 1), justification="center")]]

    window = sg.Window("Flexlink", layout, finalize=True, resizable=True)
    window.metadata = False  # True while a Run or a sweep is in progress
    window.sweep_dir = None  # sweep folder while a sweep runs (its STOP file stops it)
    window.hil_run = False   # the current / last Run went through the FPGA
    sweep_rows = []          # finished sweep points, for the live plot
    run_start = 0.0  # start time of the current Run (plots skip files older than this)

    try:
        warnings = load_into_window(window, list(FILES))
        update_velocity_table(window, window["-DOPPLER-"].get())
        window["-STATUS-"].update("Loaded Tx, Rx and Channel_Defects parameters"
                                  + (f" ({len(warnings)} missing, see console)" if warnings else ""))
        for w in warnings:
            print(f"Warning: {w}")
    except (OSError, ValueError, KeyError) as e:
        sg.popup_error(f"Could not load the parameter files:\n{e}")

    while True:
        event, values = window.read()
        if event == sg.WIN_CLOSED:
            break
        active_tab = values["-TABS-"]  # e.g. "-TAB_DEFECTS-"
        if event == "-TABS-":
            # Save only on the parameter tabs; Run only on the Simulate tab;
            # neither while a run is in progress (the scripts read the JSON files)
            set_running(window, window.metadata, active_tab)
            if not window.metadata:
                window["-STATUS-"].update("")
            continue
        if event == "-RUN-":
            try:
                plot_selection(values)  # catch a bad symbol count before a long run
            except (ValueError, OSError) as e:
                sg.popup_error(str(e), title="Invalid plot settings")
                continue
            hil = hil_mode()
            if hil:
                problems = hil_problems()
                if problems:
                    sg.popup_error("\n".join(problems), title="Hardware in the loop")
                    continue
            window.hil_run = hil
            set_running(window, True, active_tab)
            run_start = time.time()
            for key in ("-RES_CNTL-", "-RES_PAYLOAD-"):
                window[key].update("")
            steps = RUN_STEPS_HIL if hil else RUN_STEPS
            window.start_thread(lambda: run_simulation(window, steps, hil), "-RUN_THREAD_END-")
            continue
        if event == "-RUN_PROGRESS-":
            window["-STATUS-"].update(values[event])
            continue
        if event == "-RUN_DONE-":
            set_running(window, False, active_tab)
            result = values[event]
            show_run_result(window, result)
            if result["ok"]:  # overall EVM and data rate under the decoder output
                try:
                    import Flexlink_Plots
                    extra = ([Flexlink_Plots.data_rate_summary(run_start)] if result.get("hil") else
                             [Flexlink_Plots.evm_summary(FILES["rx"], run_start),
                              Flexlink_Plots.data_rate_summary(run_start)])
                except Exception as e:
                    extra = [f"EVM / data rate not available: {e}"]
                window["-RES_PAYLOAD-"].update(window["-RES_PAYLOAD-"].get().rstrip()
                                               + "\n\n" + "\n".join(extra))
            # The checkboxes as they are now (they may be changed during the run) decide
            try:
                selected, n_symbols = plot_selection(values)
                msgs = []
            except (ValueError, OSError) as e:
                selected, msgs = {}, [str(e)]
            if result.get("hil"):  # no equalized grid from the FPGA: packet plots only
                rx_only = [k for k in ("before", "after", "evm_sc", "evm_sym") if selected.get(k)]
                for k in rx_only:
                    selected[k] = False
                if rx_only:
                    msgs.append("constellation and EVM plots need the simulated Rx chain")
            if result["ok"] and any(selected.values()):
                try:
                    import Flexlink_Plots
                    msgs += Flexlink_Plots.make_plots(selected, n_symbols, FILES["rx"], run_start)
                except Exception as e:  # e.g. matplotlib missing
                    msgs.append(f"Plots failed: {e}")
            if msgs:
                window["-STATUS-"].update(window["-STATUS-"].get() + "  Plots: " + "; ".join(msgs))
            continue
        if event in ("-RUN_THREAD_END-", "-SWEEP_THREAD_END-"):
            continue
        if event == "-SW_RUN-":
            try:
                settings = sweep_settings(values)
            except ValueError as e:
                sg.popup_error(str(e), title="Invalid sweep settings")
                continue
            out_dir = APP_SIMULATION / "Sweeps" / time.strftime("Sweep_%Y%m%d_%H%M%S")
            sweep_rows = []
            set_running(window, True, active_tab, sweep=out_dir)
            window["-RES_PAYLOAD-"].update(f"SNR sweep -> {out_dir}\n{SWEEP_TABLE_HEADER}\n")
            window.start_thread(lambda: run_sweep(window, settings, out_dir), "-SWEEP_THREAD_END-")
            continue
        if event == "-SW_STOP_BTN-":
            if window.sweep_dir:
                Path(window.sweep_dir).mkdir(parents=True, exist_ok=True)
                (Path(window.sweep_dir) / "STOP").touch()
                window["-STATUS-"].update("Stopping after the current packet ...")
                window["-SW_STOP_BTN-"].update(disabled=True)
            continue
        if event == "-SWEEP_PROGRESS-":
            window["-STATUS-"].update(values[event])
            continue
        if event == "-SWEEP_POINT-":
            sweep_rows.append(values[event])
            window["-RES_PAYLOAD-"].update(window["-RES_PAYLOAD-"].get().rstrip("\n") + "\n"
                                           + sweep_table_line(values[event]))
            try:
                import Flexlink_Plots
                Flexlink_Plots.plot_sweep(sweep_rows, title=Path(window.sweep_dir).name)
            except Exception as e:
                window["-STATUS-"].update(f"Sweep plot failed: {e}")
            continue
        if event == "-SWEEP_DONE-":
            done = values[event]
            set_running(window, False, active_tab)
            if done["rc"] == 0:
                window["-STATUS-"].update(f"Sweep finished: {len(sweep_rows)} point(s), "
                                          f"results in {Path(done['dir']).name}/results.csv")
            else:
                window["-STATUS-"].update(f"Sweep stopped with exit code {done['rc']}.")
                sg.popup_scrolled(f"Run_SNR_Sweep.py stopped with exit code {done['rc']}:\n\n"
                                  f"{done['tail']}\n\nSee {done['dir']}/errors.log",
                                  title="Sweep failed", size=(110, 25), font=RESULTS_FONT)
            continue
        if event == "-NUM_RAYS-":
            update_snr_enabled(window, values["-NUM_RAYS-"])
            continue
        if event in RX_PATHS:
            update_rx_path_enabled(window, values["-RX_PATH_HIL-"])
            continue
        if event == "-SAVE-":
            limits = {"-TAB_DEFECTS-": INT_LIMITS, "-TAB_TX-": TX_INT_LIMITS}.get(active_tab)
            if limits:
                errors = validate_int_fields(values, limits)
                if errors:
                    sg.popup_error("\n".join(errors), title="Invalid values")
                    continue
            names = TAB_FILES[active_tab]
            try:
                save_from_window(values, names)
            except (OSError, ValueError) as e:
                sg.popup_error(f"Nothing was saved:\n{e}", title="Save failed")
                continue
            if active_tab == "-TAB_DEFECTS-":
                update_velocity_table(window, values["-DOPPLER-"])
            written = names + (["rx"] if "tx" in names and "rx" not in names else [])
            msg = "Saved " + ", ".join(FILES[n].name for n in written) + "."
            if active_tab in ("-TAB_TX-", "-TAB_RX-"):
                msg += " Run Initialize_Tx/Initialize_Rx to refresh the calculated values."
            window["-STATUS-"].update(msg)
            warnings = consistency_warnings()
            if warnings:
                sg.popup_ok("\n".join(warnings), title="Check settings")
            continue
        print(f"Event: {event}")

    if "matplotlib.pyplot" in sys.modules:  # close plot windows with the GUI
        sys.modules["matplotlib.pyplot"].close("all")
    window.close()


if __name__ == "__main__":
    main()
