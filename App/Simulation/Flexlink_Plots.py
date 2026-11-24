"""Flexlink_Plots.py -- plots for the Flexlink GUI (Simulate tab).

Each plot opens in its own matplotlib window. The windows are reused on the next
Run, so they update in place instead of piling up.

Constellation plots (receive ray 1 only):
    Before Equalization   Rx/<File_Rx.Grid_Payload_Rx>   with _Ray1  (Demodulate_Waveform_Rx.py)
    After Equalization    Rx/<File_Rx.Grid_Equalized_Rx> with _Ray1  (Equalize_Payload_Grid.py)
Both grids are 'data' (PacketLength, 841) complex, one row per OFDM symbol. The
first N symbols are drawn on top of each other, as in the MATLAB version
    plot(real(ResourceGrid(:,i+1)), imag(ResourceGrid(:,i+1)), 'k.', 'MarkerSize', 10)
but only the Data signals: the signal types come from
Rx/<File_Rx.ResourceGrid_Rx> (written by Initialize_Rx.py), the same
file Equalize_Payload_Grid.py uses.

EVM plots (receive ray 1, after equalization), against the KNOWN transmitted symbols:
    reference  X = Tx/<File.Grid_Payload_Tx>  (Construct_Payload_Grid.py, same layout)
    measured   Y = Rx/<File_Rx.Grid_Equalized_Rx> with _Ray1
    error      e = Y - X on the Data signals of the whole packet
    EVM_rms    = sqrt( mean|e|^2 / mean|X|^2 )   (normalised to the average power of the
                 transmitted Data signals), shown in % and dB (20*log10(EVM_rms)).
    EVM vs Subcarrier: the same ratio per tone k = -420..420, averaged over all symbols.
    EVM vs OFDM Symbol: the same ratio per OFDM symbol, averaged over its Data signals.

Packet plots (real and imag against sample index, as in the MATLAB version):
    Transmitted Packet   Tx/<File.Waveform_Payload_Tx>  (Construct_Payload_Waveform.py), 1-D
    Received Packet      the channel output Defect_Model.py writes,
                         <File_Rx.Waveform_Payload_DefectsModeled_Rx> in Rx/, else Tx/,
                         (NumberOfRays, N); ray 1 is plotted. With no defect switched on
                         the receiver gets the transmitted waveform itself, so that is shown.
    PAPR = max|x|^2 / mean|x|^2 in dB, for the whole packet and for the payload part
    (from the PayloadStart attribute, after the preamble).
    PAPR CCDF            P(instantaneous power > threshold) against the threshold in dB above
                         the average power, over the payload samples (preamble excluded):
                         transmitted, and received ray 1 when defects are modeled.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from flexlink_paths import CHANNEL_DEFECTS, RX, TX, ray_file_name

DEFECTS_FILE = CHANNEL_DEFECTS / "Channel_Defects.json"
TX_PARAMS_FILE = TX / "Parameters_Tx_payload.json"
TX_GRID_KEY, DEFAULT_TX_GRID = "Grid_Payload_Tx", "Grid_Payload_Tx.h5"
CENTER_TONE = 420  # column 420 of the 841 tones is k = 0

TX_WAVE_KEY, DEFAULT_TX_WAVE = "Waveform_Payload_Tx", "Waveform_Payload_Tx.h5"
RX_WAVE_KEY, DEFAULT_RX_WAVE = "Waveform_Payload_DefectsModeled_Rx", "Waveform_Payload_DefectsModeled_Tx.h5"

# plot key -> title
PACKET_PLOTS = {
    "tx_packet": "Transmitted Packet",
    "rx_packet": "Received Packet",
    "ccdf": "PAPR CCDF",
}

# plot key -> title
EVM_PLOTS = {
    "evm_sc": "EVM vs Subcarrier",
    "evm_sym": "EVM vs OFDM Symbol",
}

RESOURCE_GRID_KEY = "ResourceGrid_Rx"
DEFAULT_RESOURCE_GRID = "Resource_Grid_Rx.h5"
DEFAULT_DATA_CODE = 40  # signal_types code for Data, unless the file's attrs say otherwise

PLOT_RAY = 1
AXIS_LIMIT = 2.5

# plot key -> (File_Rx key, default file name, title)
CONSTELLATIONS = {
    "before": ("Grid_Payload_Rx", "Grid_Payload_Rx.h5",
               "Received Constellations Before Equalization"),
    "after": ("Grid_Equalized_Rx", "Grid_Equalized_Rx.h5",
              "Received Constellations After Equalization"),
}


def _plt():
    """Import pyplot on first use, on the Tk backend the GUI already runs."""
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    plt.ion()
    return plt


def load_grid(path: Path) -> np.ndarray:
    import h5py
    with h5py.File(path, "r") as f:
        arr = np.asarray(f["data"][()])
    if arr.dtype.names:  # compound (real, imag) storage
        names = arr.dtype.names
        re_, im_ = ("r", "i") if "r" in names else ("real", "imag")
        arr = arr[re_] + 1j * arr[im_]
    return arr.astype(np.complex128)


def grid_path(rx_params: dict, file_key: str, default: str, ray: int = PLOT_RAY) -> Path:
    name = rx_params.get("File_Rx", {}).get(file_key) or default
    return RX / ray_file_name(name, ray)


def load_data_mask(rx_params: dict) -> np.ndarray:
    """True where the resource grid holds a Data signal, (PacketLength, 841)."""
    import h5py
    name = rx_params.get("File_Rx", {}).get(RESOURCE_GRID_KEY) or DEFAULT_RESOURCE_GRID
    path = RX / name
    if not path.is_file():
        raise FileNotFoundError(f"{path.name} not found in Rx - run Initialize_Rx.py")
    with h5py.File(path, "r") as f:
        types = np.asarray(f["signal_types"][()])
        code = int(f.attrs.get("Data", DEFAULT_DATA_CODE))
    return types == code


def snr_text(rx_params: dict, since: float) -> str:
    """'SNR 20 dB set, 19.8 dB measured' for the plotted ray.

    set:      Settings/SNR<ray>_dB in Channel_Defects.json, when ThermalNoise is on
    measured: SNR_dB attribute that Equalize_Payload_Grid.py writes to the equalized grid
    """
    parts = []
    try:
        d = json.loads(DEFECTS_FILE.read_text())
        if int(d.get("Mode", {}).get("ThermalNoise", 0)):
            parts.append(f"{d['Settings'][f'SNR{PLOT_RAY}_dB']:g} dB set")
        else:
            parts.append("no noise added")
    except (OSError, ValueError, KeyError):
        pass
    try:
        import h5py
        eq = grid_path(rx_params, *CONSTELLATIONS["after"][:2])
        if eq.is_file() and eq.stat().st_mtime >= since:
            with h5py.File(eq, "r") as f:
                if "SNR_dB" in f.attrs:
                    parts.append(f"{float(f.attrs['SNR_dB']):.1f} dB measured")
    except Exception:
        pass
    return ("SNR " + ", ".join(parts)) if parts else ""


def plot_constellation(grid: np.ndarray, data_mask: np.ndarray, n_symbols: int, title: str,
                       snr: str = ""):
    plt = _plt()
    n = min(n_symbols, grid.shape[0])
    if data_mask.shape != grid.shape:
        raise ValueError(f"resource grid {data_mask.shape} does not match {grid.shape}")
    pts = grid[:n][data_mask[:n]]
    fig = plt.figure(title)  # same window every Run
    fig.clf()
    ax = fig.add_subplot()
    ax.plot(pts.real, pts.imag, "k.", markersize=4)
    ax.grid(True)
    ax.set_xlabel("real")
    ax.set_ylabel("imag")
    ax.set_xlim(-AXIS_LIMIT, AXIS_LIMIT)
    ax.set_ylim(-AXIS_LIMIT, AXIS_LIMIT)
    ax.set_aspect("equal")
    ax.set_title(f"{title}\nRay {PLOT_RAY}, Data signals of OFDM symbols 1-{n}"
                 + (f"\n{snr}" if snr else ""), fontsize=10)
    fig.tight_layout()
    fig.canvas.draw_idle()
    plt.show(block=False)
    return fig


def _db(ratio):
    """Power ratio -> dB (EVM_dB = 10 log10(mean|e|^2 / mean|X|^2) = 20 log10(EVM_rms))."""
    with np.errstate(divide="ignore", invalid="ignore"):
        return 10 * np.log10(ratio)


def compute_evm(rx: np.ndarray, tx: np.ndarray, data_mask: np.ndarray) -> dict:
    """EVM of the Data signals of rx against the transmitted tx, overall / per tone / per symbol."""
    if not (rx.shape == tx.shape == data_mask.shape):
        raise ValueError(f"grid shapes differ: Rx {rx.shape}, Tx {tx.shape}, types {data_mask.shape}")
    if not data_mask.any():
        raise ValueError("no Data signals in the resource grid")
    err_p = np.where(data_mask, np.abs(rx - tx) ** 2, 0.0)
    ref_p = np.mean(np.abs(tx[data_mask]) ** 2)  # one reference power for the whole packet
    count_sc, count_sym = data_mask.sum(axis=0), data_mask.sum(axis=1)
    with np.errstate(divide="ignore", invalid="ignore"):
        per_sc = err_p.sum(axis=0) / count_sc / ref_p      # NaN on tones without Data
        per_sym = err_p.sum(axis=1) / count_sym / ref_p    # NaN on symbols without Data
    overall = err_p.sum() / data_mask.sum() / ref_p
    return {"pct": 100 * np.sqrt(overall), "db": float(_db(overall)),
            "per_sc_db": _db(per_sc), "per_sym_db": _db(per_sym), "n": int(data_mask.sum())}


def _evm_figure(title: str, evm: dict, snr: str):
    plt = _plt()
    fig = plt.figure(title)
    fig.clf()
    ax = fig.add_subplot()
    ax.set_title(f"{title}, Ray {PLOT_RAY} after equalization\n"
                 f"EVM {evm['pct']:.2f} % ({evm['db']:.1f} dB) over {evm['n']} Data signals"
                 + (f"\n{snr}" if snr else ""), fontsize=10)
    ax.grid(True)
    ax.set_ylabel("EVM (dB)")
    return plt, fig, ax


def plot_evm_subcarrier(evm: dict, snr: str = ""):
    plt, fig, ax = _evm_figure(EVM_PLOTS["evm_sc"], evm, snr)
    k = np.arange(evm["per_sc_db"].size) - CENTER_TONE
    ax.plot(k, evm["per_sc_db"], "k.", markersize=3)
    ax.axhline(evm["db"], color="r", linewidth=1, label=f"overall {evm['db']:.1f} dB")
    ax.set_xlabel("Subcarrier k")
    ax.set_xlim(k[0], k[-1])
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.canvas.draw_idle()
    plt.show(block=False)
    return fig


def plot_evm_symbol(evm: dict, snr: str = ""):
    plt, fig, ax = _evm_figure(EVM_PLOTS["evm_sym"], evm, snr)
    sym = np.arange(1, evm["per_sym_db"].size + 1)
    ax.plot(sym, evm["per_sym_db"], "k.-", markersize=5, linewidth=0.8)
    ax.axhline(evm["db"], color="r", linewidth=1, label=f"overall {evm['db']:.1f} dB")
    ax.set_xlabel("OFDM symbol")
    ax.set_xlim(0.5, sym[-1] + 0.5)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.canvas.draw_idle()
    plt.show(block=False)
    return fig


def tx_grid_path() -> Path:
    tx_params = json.loads(TX_PARAMS_FILE.read_text())
    return TX / (tx_params.get("File", {}).get(TX_GRID_KEY) or DEFAULT_TX_GRID)


def _fresh(path: Path, since: float) -> bool:
    return path.is_file() and path.stat().st_mtime >= since


def load_evm(rx_params: dict, data_mask: np.ndarray, since: float) -> dict:
    """EVM of this run's equalized ray 1 grid against this run's Tx grid (raises if missing)."""
    eq_path = grid_path(rx_params, *CONSTELLATIONS["after"][:2])
    tx_path = tx_grid_path()
    if not _fresh(eq_path, since):
        raise FileNotFoundError(f"no new {eq_path.name} (ray {PLOT_RAY} found no packet?)")
    if not _fresh(tx_path, since):
        raise FileNotFoundError(f"no new {tx_path.name} from this run's Tx chain")
    return compute_evm(load_grid(eq_path), load_grid(tx_path), data_mask)


def evm_summary(rx_params_file: Path, since: float) -> str:
    """One line for the results box: overall EVM of this run (or why there is none)."""
    try:
        rx_params = json.loads(Path(rx_params_file).read_text())
        evm = load_evm(rx_params, load_data_mask(rx_params), since)
        return (f"EVM (ray {PLOT_RAY} after equalization, vs transmitted symbols): "
                f"{evm['pct']:.2f} % ({evm['db']:.1f} dB) over {evm['n']} Data signals")
    except Exception as e:
        return f"EVM not available: {e}"


def data_rate(since: float) -> dict:
    """Transport block bits of one packet / duration of the whole packet (preamble included).

    bits     = Calculated/TransportBlockSize * Calculated/AvailableDataBlocks (Tx JSON)
    duration = samples in Tx/<File.Waveform_Payload_Tx> / its SampleRate attribute
    """
    import h5py
    tx_params = json.loads(TX_PARAMS_FILE.read_text())
    calc = tx_params.get("Calculated", {})
    bits = int(calc["TransportBlockSize"]) * int(calc["AvailableDataBlocks"])
    path = TX / (tx_params.get("File", {}).get(TX_WAVE_KEY) or DEFAULT_TX_WAVE)
    if not _fresh(path, since):
        raise FileNotFoundError(f"no new {path.name} from this run's Tx chain")
    with h5py.File(path, "r") as f:
        n = int(f["data"].shape[-1])
        fs = float(f.attrs.get("SampleRate", tx_params.get("Defined", {}).get("SampleRate", 0)))
        pre = int(f.attrs.get("PreambleLength", 0))
    if fs <= 0:
        raise ValueError("SampleRate unknown")
    duration = n / fs
    return {"bits": bits, "samples": n, "preamble": pre, "fs": fs, "duration_s": duration,
            "mbps": bits / duration / 1e6}


def data_rate_summary(since: float) -> str:
    """One line for the results box: transport block bit rate over the whole packet."""
    try:
        r = data_rate(since)
        return (f"Data rate: {r['bits']} transport block bits / {r['duration_s'] * 1e6:.1f} us "
                f"= {r['mbps']:.3f} Mbps  (whole packet: {r['samples']} samples at "
                f"{r['fs'] / 1e6:g} MHz, preamble {r['preamble']} samples included)")
    except Exception as e:
        return f"Data rate not available: {e}"


# ---------- Packet (time-domain waveform) plots ----------
def load_waveform(path: Path) -> tuple[np.ndarray, dict]:
    """'data' as complex (1-D, or rays x N) and the file's attrs."""
    import h5py
    with h5py.File(path, "r") as f:
        arr = np.asarray(f["data"][()])
        attrs = dict(f.attrs)
    if arr.dtype.names:
        names = arr.dtype.names
        re_, im_ = ("r", "i") if "r" in names else ("real", "imag")
        arr = arr[re_] + 1j * arr[im_]
    return arr.astype(np.complex128), attrs


def papr_db(x: np.ndarray) -> float:
    p = np.abs(x) ** 2
    return float(10 * np.log10(p.max() / p.mean())) if p.size and p.mean() > 0 else float("nan")


def _papr_text(x: np.ndarray, payload_start: int) -> str:
    text = f"PAPR {papr_db(x):.1f} dB whole packet"
    if 0 < payload_start < x.size:
        text += f", {papr_db(x[payload_start:]):.1f} dB payload"
    return text


def _wave_axes(ax, x: np.ndarray, payload_start: int):
    n = np.arange(1, x.size + 1)
    ax.plot(n, x.real, linewidth=0.5, label="real")
    ax.plot(n, x.imag, linewidth=0.5, label="imag")
    if 0 < payload_start < x.size:
        ax.axvline(payload_start + 1, color="k", linestyle="--", linewidth=0.8, label="payload start")
    ax.set_xlim(1, x.size)
    ax.grid(True)


def plot_tx_packet(x: np.ndarray, attrs: dict):
    plt = _plt()
    title = PACKET_PLOTS["tx_packet"]
    start = int(attrs.get("PayloadStart", 0))
    fig = plt.figure(title)
    fig.clf()
    ax = fig.add_subplot()
    _wave_axes(ax, x, start)
    fs = float(attrs.get("SampleRate", 0))
    ax.set_title(f"{title}\n{x.size} samples" + (f" at {fs / 1e6:g} MHz" if fs else "")
                 + f", {_papr_text(x, start)}", fontsize=10)
    ax.set_xlabel("Sample")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.canvas.draw_idle()
    plt.show(block=False)
    return fig


def plot_rx_packet(x: np.ndarray, attrs: dict, note: str = ""):
    """Received packet of ray PLOT_RAY (x is 1-D)."""
    plt = _plt()
    title = PACKET_PLOTS["rx_packet"]
    start = int(attrs.get("PayloadStart", 0))
    snrs = np.atleast_1d(attrs.get("MeasuredSNR_dB", []))
    i = PLOT_RAY - 1
    snr = f", SNR {float(snrs[i]):.1f} dB" if i < snrs.size and np.isfinite(snrs[i]) else ""
    applied = str(attrs.get("DefectsApplied", "")).replace(",", ", ")
    fig = plt.figure(title)
    fig.clf()
    ax = fig.add_subplot()
    _wave_axes(ax, x, start)
    ax.set_title(f"{title}, Ray {PLOT_RAY}\n{_papr_text(x, start)}{snr}\n"
                 + (note or (f"Defects: {applied}" if applied else "")), fontsize=10)
    ax.set_xlabel("Sample")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.canvas.draw_idle()
    plt.show(block=False)
    return fig


def ccdf(x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Sample-power CCDF: threshold (dB above average power) and P(power > threshold)."""
    p = np.abs(x) ** 2
    p_db = np.sort(10 * np.log10(p[p > 0] / p.mean()))
    prob = 1.0 - np.arange(p_db.size) / p_db.size
    return p_db, prob


def plot_ccdf(curves: list[tuple[str, np.ndarray]]):
    """curves: (label, payload samples). One CCDF line each, log probability axis."""
    plt = _plt()
    title = PACKET_PLOTS["ccdf"]
    fig = plt.figure(title)
    fig.clf()
    ax = fig.add_subplot()
    n_min = None
    for (label, x), style in zip(curves, ("-", "--")):
        t, prob = ccdf(x)
        keep = t >= 0
        ax.semilogy(t[keep], prob[keep], style, linewidth=1.2,
                    label=f"{label} (PAPR {papr_db(x):.1f} dB)")
        n_min = x.size if n_min is None else min(n_min, x.size)
    ax.set_ylim(1.0 / max(n_min or 10, 10), 1)
    ax.set_xlim(left=0)
    ax.grid(True, which="both")
    ax.set_xlabel("Instantaneous power above average (dB)")
    ax.set_ylabel("Probability (power > threshold)")
    ax.set_title(f"{title}, payload samples (preamble excluded)", fontsize=10)
    ax.legend(loc="lower left", fontsize=8)
    fig.tight_layout()
    fig.canvas.draw_idle()
    plt.show(block=False)
    return fig


def received_waveform(rx_params: dict) -> Path:
    """Where Decimate_Payload_Rx.py takes the channel output from: Rx/ first, else Tx/."""
    name = rx_params.get("File_Rx", {}).get(RX_WAVE_KEY) or DEFAULT_RX_WAVE
    return RX / name if (RX / name).is_file() else TX / name


def defects_on() -> bool:
    d = json.loads(DEFECTS_FILE.read_text())
    return any(int(v) for v in d.get("Mode", {}).values())


def make_packet_plots(selected: dict, rx_params: dict, since: float) -> list[str]:
    messages = []
    try:
        tx_params = json.loads(TX_PARAMS_FILE.read_text())
        tx_path = TX / (tx_params.get("File", {}).get(TX_WAVE_KEY) or DEFAULT_TX_WAVE)
        if not _fresh(tx_path, since):
            raise FileNotFoundError(f"no new {tx_path.name} from this run's Tx chain")
        tx_wave, tx_attrs = load_waveform(tx_path)
        tx_wave = tx_wave.ravel()
        start = int(tx_attrs.get("PayloadStart", 0))

        # Received waveform of ray 1: the channel output, or the Tx waveform without defects
        rx_wave = rx_attrs = None
        note = ""
        if selected.get("rx_packet") or selected.get("ccdf"):
            if defects_on():
                path = received_waveform(rx_params)
                if not _fresh(path, since):
                    raise FileNotFoundError(f"no new {path.name} from this run's Defect_Model")
                rays, rx_attrs = load_waveform(path)
                rx_wave = np.atleast_2d(rays)[PLOT_RAY - 1]
            else:
                rx_wave, rx_attrs = tx_wave, tx_attrs
                note = "No defects modeled: the receiver gets the transmitted waveform"

        if selected.get("tx_packet"):
            plot_tx_packet(tx_wave, tx_attrs)
        if selected.get("rx_packet"):
            plot_rx_packet(rx_wave, rx_attrs, note)
        if selected.get("ccdf"):
            curves = [("Transmitted", tx_wave[start:])]
            if not note:  # with no defects the received curve would just repeat the Tx one
                curves.append((f"Received ray {PLOT_RAY}", rx_wave[start:]))
            plot_ccdf(curves)
    except Exception as e:
        messages.append(f"Packet plots: {e}")
    return messages


def make_plots(selected: dict, n_symbols: int, rx_params_file: Path, since: float) -> list[str]:
    """Draw every selected plot.
    selected: {"tx_packet", "rx_packet", "before", "after", "evm_sc", "evm_sym": bool}.

    Files older than `since` (the start of the Run) are skipped, so a ray that found
    no packet this time never shows the grid of an earlier run.
    Returns messages for the status line (empty list = all plotted).
    """
    rx_params = json.loads(Path(rx_params_file).read_text())
    messages = []
    if any(selected.get(k) for k in PACKET_PLOTS):
        messages += make_packet_plots(selected, rx_params, since)
    if not any(selected.get(k) for k in [*CONSTELLATIONS, *EVM_PLOTS]):
        return messages
    try:
        data_mask = load_data_mask(rx_params)
    except Exception as e:
        return [f"Plots: {e}"]
    snr = snr_text(rx_params, since)

    for key, (file_key, default, title) in CONSTELLATIONS.items():
        if not selected.get(key):
            continue
        path = grid_path(rx_params, file_key, default)
        if not _fresh(path, since):
            messages.append(f"{title}: no new {path.name} (ray {PLOT_RAY} found no packet?)")
            continue
        try:
            plot_constellation(load_grid(path), data_mask, n_symbols, title, snr)
        except Exception as e:  # a plot failure must not take the GUI down
            messages.append(f"{title}: {e}")

    if any(selected.get(k) for k in EVM_PLOTS):
        try:
            evm = load_evm(rx_params, data_mask, since)
            if selected.get("evm_sc"):
                plot_evm_subcarrier(evm, snr)
            if selected.get("evm_sym"):
                plot_evm_symbol(evm, snr)
        except Exception as e:
            messages.append(f"EVM: {e}")
    return messages


# ---------- SNR sweep ----------
SWEEP_TITLE = "SNR Sweep"


def draw_sweep(fig, rows: list[dict], title: str = ""):
    """Left: BER, BLER, PER against SNR (log). Right: EVM (dB) against SNR.

    rows: the results.csv rows of Run_SNR_Sweep.py (dicts). Zero error rates cannot be
    shown on a log axis; they are marked at the bottom with a 'v' instead.
    """
    fig.clf()
    ax_err, ax_evm = fig.subplots(1, 2)
    snr = np.array([float(r["snr_db"]) for r in rows])
    floor = None
    for key, label, style in (("ber", "BER", "o-"), ("bler", "BLER", "s-"), ("per", "PER", "^--")):
        y = np.array([float(r[key]) for r in rows])
        pos = y > 0
        if pos.any():
            ax_err.semilogy(snr[pos], y[pos], style, markersize=4, label=label)
            floor = min(floor or 1, y[pos].min())
    zero_ber = [s for s, r in zip(snr, rows) if float(r["ber"]) == 0 and float(r["bler"]) == 0]
    if zero_ber:
        bottom = (floor or 1e-3) / 10
        ax_err.semilogy(zero_ber, [bottom] * len(zero_ber), "kv", markersize=6,
                        label="no errors")
    ax_err.grid(True, which="both")
    ax_err.set_xlabel("SNR (dB)")
    ax_err.set_ylabel("Error rate")
    ax_err.set_title("BER / BLER / PER", fontsize=10)
    if ax_err.lines:
        ax_err.legend(loc="lower left", fontsize=8)

    evm = np.array([float(r["evm_db"]) for r in rows])
    meas = np.array([float(r["measured_snr_db"]) for r in rows])
    ok = np.isfinite(evm)
    if ok.any():
        ax_evm.plot(snr[ok], evm[ok], "o-", markersize=4, label="EVM ray 1")
        ax_evm.plot(snr[ok], -snr[ok], "k:", linewidth=1, label="-SNR (noise only)")
        okm = np.isfinite(meas)
        if okm.any():
            ax_evm.plot(snr[okm], -meas[okm], "x--", markersize=5, label="-measured SNR")
        ax_evm.legend(loc="upper right", fontsize=8)
    else:
        ax_evm.text(0.5, 0.5, "EVM not available\n(no equalized ray 1 grid)",
                    ha="center", va="center", transform=ax_evm.transAxes)
    ax_evm.grid(True)
    ax_evm.set_xlabel("SNR (dB)")
    ax_evm.set_ylabel("EVM (dB)")
    ax_evm.set_title("EVM after equalization", fontsize=10)
    if title:
        fig.suptitle(title, fontsize=10)
    fig.tight_layout()
    return fig


def plot_sweep(rows: list[dict], title: str = ""):
    """Live sweep window for the GUI (same window, redrawn after every point)."""
    plt = _plt()
    fig = plt.figure(SWEEP_TITLE, figsize=(10, 4.5))
    draw_sweep(fig, rows, title)
    fig.canvas.draw_idle()
    plt.show(block=False)
    return fig
