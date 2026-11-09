#!/usr/bin/env python3
"""FlexlinkProcessCoreBoardCapturedWaveform.py -- find and decode the Flexlink packets in a CoreBoard capture.

Input: App/CoreBoard/CoreBoard_Capture.cs8 (+ .json), written by App/FlexlinkCaptureWithCoreBoard.py
(int8 I/Q at 40 Msps, or 20 Msps, from the ESP32-S31 CoreBoard). At 20 Msps no decimation is needed
(the Rx chain's rate); the signal's 16.4 MHz fit, and the analog filter follows the sample rate.

Each decoded packet's preamble is also checked against the known waveform: the summary says if the
sample stream itself is corrupt (the ESP-SDR S31 firmware of Oct 2026 misplaces part of every
2016-sample DMA buffer by one 16128-sample ring lap). That fault is repaired by default: the
misplaced windows are found from their edges (folded over the ring laps around each packet) and
the samples are moved back one lap; samples no lap holds are set to 0. A packet keeps its original
samples if the repair doesn't improve its preamble; --no-repair turns the repair off. The repair is
reliable at 40 Msps (the signal leaves 60 % of the band empty, where the window edges show); at
20 Msps there is too little empty band and it is usually skipped.

Steps, the same path as the Pluto loopback of FlexlinkTxWithPluto.py:
    1. Levels and DC of the capture; the residual DC is removed.
    2. Packet search on the power envelope: every whole packet (rise after quiet, fall before quiet)
       that lies completely inside the capture. Packet and gap lengths are measured, not assumed.
    3. Rx/Initialize_Rx.py once.
    4. Per packet: cut with 1024 samples of margin (at 20 Msps), low-pass and decimate 40 -> 20 Msps,
       scale to the simulator's level -> App/CoreBoard/CoreBoard_Packet_Rx.h5, then
       Rx/Run_Rx_Chain.py --no-model --input <that> (no defects modelled, no Tx comparison: the
       transport blocks come from the other PC). The control field sets the payload settings, so
       this PC's Parameters_Rx_payload.json need not match the transmitter beforehand.
    5. Per packet: CFO, EQ SNR, decision-directed EVM, CRC of every block; file blocks (Selected
       file mode on the transmitter) are collected in App/CoreBoard/FileTransfer_State and the file
       is written to App/CoreBoard/Received_Files once complete (size and SHA-256 checked).

Results: the console, App/CoreBoard/CoreBoard_Results.txt (the same sections as the Received tab of
FlexlinkTxWithPluto.py: capture, control field, payload field), App/CoreBoard/CoreBoard_Packets.csv
(one row per packet) and App/CoreBoard/CoreBoard_Process.log (every Rx chain step's output).

The Rx chain takes a few seconds per packet, so by default the first 20 packets are decoded
(--max-packets 0: all). When the packets carry a file, every packet is decoded until the file is
complete.

Usage
-----
    python App/FlexlinkProcessCoreBoardCapturedWaveform.py
    python App/FlexlinkProcessCoreBoardCapturedWaveform.py --max-packets 0         # every packet
    python App/FlexlinkProcessCoreBoardCapturedWaveform.py --packet 5 --plot       # one packet, plots
    python App/FlexlinkProcessCoreBoardCapturedWaveform.py --find-only             # packet search only
    python App/FlexlinkProcessCoreBoardCapturedWaveform.py --no-repair             # without the ring-lap repair

Exit status: 0 at least one packet decoded with every CRC passing, 1 packets found but none fully
decoded, 2 no packet found, 3 error.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

APP_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(APP_DIR))
sys.path.insert(0, str(APP_DIR.parent))
from flexlink_paths import APP, BASIC_MODULES, CHANNEL_DEFECTS, CONSTRUCT_GRID, ROOT, RX, TX, ray_file_name  # noqa: E402

sys.path.insert(0, str(RX))
sys.path.insert(0, str(BASIC_MODULES / "File_Transfer"))

CB_DIR = APP / "CoreBoard"
CAPTURE = CB_DIR / "CoreBoard_Capture.cs8"
PACKET_RX = CB_DIR / "CoreBoard_Packet_Rx.h5"
DEFECTS_OFF = CB_DIR / "Channel_Defects_Off.json"
PROCESS_LOG = CB_DIR / "CoreBoard_Process.log"
RX_LOG = CB_DIR / "Run_Rx_Chain.log"
RESULTS_TXT = CB_DIR / "CoreBoard_Results.txt"
PACKETS_CSV = CB_DIR / "CoreBoard_Packets.csv"
STATE_DIR = CB_DIR / "FileTransfer_State"
FILES_DIR = CB_DIR / "Received_Files"
RX_JSON = RX / "Parameters_Rx_payload.json"
TX_JSON = TX / "Parameters_Tx_payload.json"
CF_LOG = TX / "ControlSignals" / "Build_Control_Field.log"

FS_IN = 40_000_000                 # CoreBoard capture rate: 40 or 20 Msps (set from the capture's .json)
RATES_OK = (40_000_000, 20_000_000)
RING = 16128                       # ESP-SDR S31 DMA ring, samples (8 descriptors x 2016)
RX_RATE = 20_000_000               # the Python Rx chain's rate
MARGIN20 = 1024                    # samples kept before / after a packet, at 20 Msps (as the loopback)
BLK = 256                          # power-envelope block (6.4 us; 128 at 20 Msps)
FILE_MIN_BLOCKS = 3                # blocks of one file ID before the packets are taken as a file transfer
REF_POWER_20 = 0.8266              # simulator burst power at 20 Msps (Tx PayloadPower 0.4133 x 2)
RX_STATUS = {0: "every CRC passed", 1: "a block failed", 2: "no packet detected or packet aborted",
             3: "Rx chain error", 4: "bad packet, control field CRC failed"}


# ---------- capture ----------
def load_capture(path: Path) -> tuple[np.ndarray, dict]:
    if not path.is_file():
        raise FileNotFoundError(f"{path} not found - run App/FlexlinkCaptureWithCoreBoard.py first")
    raw = np.memmap(path, np.int8, "r")
    if raw.size % 2:
        raw = raw[:-1]
    meta_path = path.with_suffix(".json")
    meta = json.loads(meta_path.read_text()) if meta_path.is_file() else {}
    global FS_IN, BLK
    rate = int(meta.get("SampleRate", 40_000_000))
    if rate not in RATES_OK:
        raise ValueError(f"the capture is at {rate} sps; Flexlink needs 40 or 20 Msps")
    FS_IN = rate
    BLK = int(round(6.4e-6 * FS_IN))
    return raw.reshape(-1, 2), meta


def envelope(iq: np.ndarray, blk: int | None = None, chunk_blocks: int = 1 << 15) -> tuple[np.ndarray, np.ndarray]:
    """Mean power per block (int8 units^2) and mean I/Q per block, in chunks (the capture is large)."""
    blk = blk or BLK
    nb = iq.shape[0] // blk
    p = np.empty(nb)
    m = np.empty((nb, 2))
    for a in range(0, nb, chunk_blocks):
        b = min(nb, a + chunk_blocks)
        x = np.asarray(iq[a * blk:b * blk], np.float32).reshape(b - a, blk, 2)
        p[a:b] = (x ** 2).sum(axis=2).mean(axis=1)
        m[a:b] = x.mean(axis=1)
    return p, m


def find_packets(p: np.ndarray, blk: int | None = None, min_quiet_us: float = 40.0, min_len_us: float = 300.0,
                 thresh_db: float | None = None) -> tuple[list[tuple[int, int]], dict]:
    """Whole packets in the envelope: [(start, end)] in capture samples, and the levels used."""
    blk = blk or BLK
    pdb = 10 * np.log10(p + 1e-6)
    floor = float(np.percentile(pdb, 2))
    # packet level: the typical level of the blocks clearly above the noise. A percentile of the
    # whole capture fails when the packets fill only a small part of it (a short file transfer)
    hi = pdb[pdb > floor + 8]
    us_blk = (blk or BLK) / FS_IN * 1e6
    if hi.size * us_blk >= 1000:                 # at least ~1 ms of strong signal
        sig = float(np.median(hi))
    else:
        sig = float(np.percentile(pdb, 98))
    info = {"floor_db": floor, "signal_db": sig}
    if sig - floor < 6:
        info["reason"] = f"no on/off pattern: the envelope spans only {sig - floor:.1f} dB"
        return [], info
    thr = floor + (sig - floor) / 2 if thresh_db is None else floor + thresh_db
    info["thresh_db"] = thr
    on = pdb > thr
    us = blk / FS_IN * 1e6
    min_quiet, min_len = max(1, int(min_quiet_us / us)), int(min_len_us / us)
    # runs of on / off
    edges = np.flatnonzero(np.diff(on.astype(np.int8))) + 1
    bounds = np.r_[0, edges, on.size]
    runs = [(int(bounds[i]), int(bounds[i + 1]), bool(on[bounds[i]])) for i in range(bounds.size - 1)]
    # close short quiet holes inside a packet
    merged = []
    for r in runs:
        if merged and not r[2] and r[1] - r[0] < min_quiet and r[1] < on.size and merged[-1][2]:
            merged[-1] = (merged[-1][0], r[1], True)
        elif merged and r[2] and merged[-1][2]:
            merged[-1] = (merged[-1][0], r[1], True)
        else:
            merged.append(r)
    pk = []
    for a, b, is_on in merged:
        if not is_on or a == 0 or b >= on.size:          # quiet, or cut by the capture edges
            continue
        if b - a < min_len:
            continue
        pk.append((a * blk, b * blk))
    info["partial"] = sum(1 for a, b, o in merged if o) - len(pk)
    # bursts much shorter than the typical packet (Wi-Fi, a packet cut by lost samples): not decoded
    if pk:
        typ = typical_length([b - a for a, b in pk])
        info["short"] = [(a, b) for a, b in pk if b - a < 0.75 * typ]
        pk = [(a, b) for a, b in pk if b - a >= 0.75 * typ]
    return pk, info


def typical_length(lengths) -> float:
    """The most common burst length (most bursts within +-2 % of it; ties: the longest). Robust
    when Wi-Fi bursts or fragments are as numerous as the Flexlink packets."""
    v = np.sort(np.asarray(lengths, float))
    lo = np.searchsorted(v, v * 0.98, "left")
    hi = np.searchsorted(v, v * 1.02, "right")
    cnt = hi - lo
    best = np.flatnonzero(cnt == cnt.max())[-1]
    return float(np.median(v[lo[best]:hi[best]]))


def describe_packets(pk: list[tuple[int, int]]) -> list[str]:
    if not pk:
        return []
    lens = np.array([b - a for a, b in pk])
    lines = [f"{len(pk)} whole packet(s): length {np.median(lens) / FS_IN * 1e3:.3f} ms "
             f"(~{int(np.median(lens))} samples at {FS_IN / 1e6:g} Msps, envelope estimate +-{BLK})"]
    if len(pk) > 1:
        starts = np.array([a for a, _ in pk])
        period = np.diff(starts)
        med = float(np.median(period))
        regular = period < 1.5 * med
        gaps = np.array([pk[i + 1][0] - pk[i][1] for i in range(len(pk) - 1)])[regular]
        lines.append(f"  packet period {med / FS_IN * 1e3:.3f} ms ({FS_IN / med:.1f} packets/s), gap "
                     f"{np.median(gaps) / FS_IN * 1e6:.0f} us (~{int(np.median(gaps))} samples)")
        missing = int(np.round(period[~regular] / med - 1).sum())
        if missing:
            lines.append(f"  about {missing} packet(s) missing between the ones found (transmitter "
                         f"underflow, lost samples, interference, or packets below the threshold)")
    return lines


def to_complex(iq: np.ndarray, a: int, b: int, dc: np.ndarray) -> np.ndarray:
    """int8 I/Q samples a..b -> complex, DC removed."""
    x = np.asarray(iq[a:b], np.float64)
    return (x[:, 0] - dc[0]) + 1j * (x[:, 1] - dc[1])


def to_rx(x: np.ndarray) -> np.ndarray:
    """complex capture samples -> 20 Msps (40 Msps: low-passed and decimated by 2)."""
    if FS_IN == RX_RATE:
        return x
    from scipy.signal import resample_poly
    return resample_poly(x, 1, FS_IN // RX_RATE)


def settle_lead_in(x: np.ndarray, n_lead: int, span: int = 256) -> np.ndarray:
    """Remove the slow baseline (DC wander, ~0-40 kHz) from the noise before the packet.
    The CoreBoard noise floor is ~1 LSB, so a residual DC that wanders faster than the Rx DC
    blocker follows can dominate it; it autocorrelates like Preamble A and the packet detector
    then triggers early in the noise, which puts the Preamble B search window too early
    ("no Preamble B peak in the search window"). Only the lead-in is touched, never the packet."""
    if n_lead <= span:
        return x
    from scipy.ndimage import uniform_filter1d
    lead = x[:n_lead]
    base = uniform_filter1d(lead.real, span, mode="nearest") + 1j * uniform_filter1d(lead.imag, span, mode="nearest")
    y = x.copy()
    y[:n_lead] = lead - base
    return y


# ---------- ring-lap repair ----------
# The ESP-SDR S31 firmware (Oct 2026) delivers, in a window of ~350-450 samples per ~2016-sample DMA
# region, the samples of one ring lap later: x[t] = y[t + RING]. The windows sit at (nearly) the
# same place in every lap, so the true y[t] is x[t - RING] when t - RING was in the previous lap's
# window too. Window edges are spikes of out-of-band energy (the Flexlink signal is band-limited, a
# splice is not), folded modulo RING over laps inside packets, then refined per lap. Samples that
# no lap holds (a window longer than the one a lap before) are set to 0.
def _hp_energy(x: np.ndarray, fs: float) -> np.ndarray:
    """|high-pass(x)|^2 aligned to x: energy above the Flexlink band (|f| > 10 MHz at 40 Msps,
    > 9.2 MHz at 20 Msps)."""
    from scipy.signal import firwin, lfilter
    cut = 10e6 if fs >= 40e6 else 9.2e6
    ntap = 31 if fs >= 40e6 else 63
    h = firwin(ntap, cut, fs=fs, pass_zero=False)
    y = lfilter(h, 1, x)
    e = np.abs(y) ** 2
    g = (ntap - 1) // 2
    return np.r_[e[g:], np.zeros(g)]


def ring_template(x: np.ndarray, start: int, fs: float, min_len: int | None = None,
                  max_len: int | None = None, rel_thresh: float = 6.0) -> tuple[list[tuple[int, int]], np.ndarray]:
    """Window edges (phase mod RING) from laps of x (complex, absolute position `start` of x[0]).
    Returns [(start_phase, end_phase)] and the folded evidence."""
    sc = fs / 40e6                            # the windows last ~9.6 us: 384 samples at 40 Msps
    min_len = int(120 * sc) if min_len is None else min_len
    max_len = int(700 * sc) if max_len is None else max_len
    nl = x.size // RING
    if nl < 3:
        return [], np.zeros(RING)
    off = (-start) % RING                       # first full lap in absolute phase
    nl = (x.size - off) // RING
    xs = x[off:off + nl * RING]
    e = _hp_energy(xs, fs).reshape(nl, RING)
    p = (np.abs(xs) ** 2).reshape(nl, RING)
    blk = p.reshape(nl, -1, 252).mean(2)     # 64 blocks of 252 per lap
    thr = 0.25 * np.percentile(blk, 90)
    clean = blk.min(1) > thr
    if clean.sum() < 3:
        return [], np.zeros(RING)
    en = e[clean] / (np.median(e[clean], axis=1, keepdims=True) + 1e-12)
    F = np.median(en, axis=0)
    F = np.convolve(F, np.ones(3) / 3, "same")
    # index j is absolute phase j: x[off] sits at a multiple of RING
    # peaks
    pk = []
    for j in np.argsort(F)[::-1]:
        if F[j] < rel_thresh:
            break
        if all(min(abs(j - k), RING - abs(j - k)) > 20 for k in pk):
            pk.append(int(j))
    pk.sort()
    # pair: a start is followed by an end min_len..max_len later; ends may be smeared (lap to lap)
    wins = []
    used = set()
    for i, s in enumerate(pk):
        if s in used:
            continue
        cands = [q for q in pk if q not in used and min_len <= (q - s) % RING <= max_len]
        if cands:
            # the farthest candidate in range is the end (a lap-varying end can make two peaks)
            q = max(cands, key=lambda q: (q - s) % RING)
            for c in cands:
                used.add(c)
            used.add(s)
            wins.append((s, q))
    # an unpaired start (its end varies too much from lap to lap to fold into a peak): keep it as a
    # window with an open end, found per lap (end phase -1 = "search")
    for s in pk:
        if s not in used:
            wins.append((s, -1))
    wins.sort()
    # plausible only as the measured fault: 6..10 windows per lap, one per ~2016-sample DMA region
    if not 6 <= len(wins) <= 10:
        return [], F
    st = np.array([w[0] for w in wins])
    gaps = np.diff(np.r_[st, st[0] + RING])
    if gaps.min() < 1500 or gaps.max() > 2600:
        return [], F
    return wins, F


_HP_TAPS: dict = {}


def _hp_taps(fs: float) -> np.ndarray:
    if fs not in _HP_TAPS:
        from scipy.signal import firwin
        _HP_TAPS[fs] = firwin(31 if fs >= 40e6 else 63, 10e6 if fs >= 40e6 else 9.2e6, fs=fs,
                              pass_zero=False)
    return _HP_TAPS[fs]


def _splice_cost(left: np.ndarray, right: np.ndarray, cands: np.ndarray, h: np.ndarray,
                 half: int = 3) -> np.ndarray:
    """Out-of-band energy right at a junction: left[:c] followed by right[c:], for each candidate
    c (indices into left/right, which are aligned). Smaller = smoother junction."""
    g = h.size // 2
    span = np.arange(-g - half, g + half + 1)
    idx = cands[:, None] + span[None, :]                     # (ncand, L)
    idx = np.clip(idx, 0, left.size - 1)
    z = np.where(idx < cands[:, None], left[idx], right[idx])
    out = np.stack([z[:, k:k + h.size] @ h[::-1] for k in range(2 * half + 1)], axis=1)
    return (np.abs(out) ** 2).sum(1)


def _ring_refine(A: np.ndarray, B: np.ndarray, base: int, s_ph: int, e_ph: int | None, h: np.ndarray,
                 slack_s: int = 40, min_len: int = 150, max_len: int = 700,
                 slack_e: int = 80) -> tuple[int, int]:
    """Per-lap edges of one window. A: the stream as received, B: the stream one lap earlier
    (B[i] = x[i - RING]), both starting at absolute position base. At the true start the received
    stream jumps (A|A rough) while A-then-B joins smoothly; at the true end B-then-A joins smoothly.
    Score = roughness of the received stream - roughness of the repaired junction."""
    def pick(lo, hi, start_edge):
        c = np.arange(max(lo - base, 40), min(hi - base, A.size - 40))
        if c.size == 0:
            return None
        rough = _splice_cost(A, A, c, h)
        smooth = _splice_cost(A, B, c, h) if start_edge else _splice_cost(B, A, c, h)
        return base + int(c[np.argmax(rough - smooth)])
    st = pick(s_ph - slack_s, s_ph + slack_s + 1, True)
    if st is None:
        return s_ph, e_ph
    if e_ph is None:                                   # open end: search the whole plausible range
        en = pick(st + min_len, st + max_len + 1, False)
    else:                                              # near the template end
        en = pick(max(st + min_len, e_ph - slack_e), e_ph + slack_e + 1, False)
    return st, (en if en is not None else (e_ph if e_ph is not None else st))


def ring_repair(x: np.ndarray, start: int, fs: float, wins: list[tuple[int, int]],
           refine_edges: bool = True) -> tuple[np.ndarray, dict]:
    """Repair x (complex; x[0] at absolute position `start`). Needs one lap of history: samples
    of the first lap can't be restored and are zeroed when misplaced. Returns (y, stats)."""
    y = x.copy()
    n = x.size
    h = _hp_taps(fs)
    xb = np.r_[np.zeros(RING, x.dtype), x[:-RING]] if n > RING else np.zeros_like(x)   # one lap earlier
    # per lap and window: [s, t) misplaced (absolute)
    masks = []
    first_lap = start // RING
    last_lap = (start + n) // RING + 1
    for L in range(first_lap, last_lap + 1):
        base = L * RING
        for s_ph, e_ph in wins:
            s_abs = base + s_ph
            open_end = e_ph < 0
            if open_end:
                e_ph = (s_ph + int(400 * fs / 40e6)) % RING
            t_abs = base + e_ph if e_ph > s_ph else base + RING + e_ph
            if t_abs <= start or s_abs >= start + n:
                continue
            if refine_edges and s_abs - 80 >= start + RING and s_abs + 800 <= start + n:
                lo = s_abs - 80
                hi = s_abs + 800
                k = fs / 40e6
                s_r, t_r = _ring_refine(x[lo - start:hi - start], xb[lo - start:hi - start], lo, s_abs,
                                        None if open_end else t_abs, h, slack_s=int(40 * k),
                                        min_len=int(150 * k), max_len=int(700 * k), slack_e=int(80 * k))
                if 0 < t_r - s_r < 900:
                    s_abs, t_abs = s_r, t_r
            masks.append((s_abs, t_abs))
    masks.sort()
    restored = lost = 0
    mk = np.zeros(n + 2 * RING, bool)            # misplaced mask, absolute start-D .. start+n+D
    for s_abs, t_abs in masks:
        a, b = max(s_abs, start - RING), min(t_abs, start + n + RING)
        if b > a:
            mk[a - (start - RING):b - (start - RING)] = True
    for s_abs, t_abs in masks:
        a, b = max(s_abs, start), min(t_abs, start + n)
        if b <= a:
            continue
        idx = np.arange(a, b)
        prev_bad = mk[idx - RING - (start - RING)]
        src = idx - RING - start
        ok = prev_bad & (src >= 0)
        y[idx[ok] - start] = x[src[ok]]
        y[idx[~ok] - start] = 0
        restored += int(ok.sum())
        lost += int((~ok).sum())
    return y, {"windows": len(masks), "restored": restored, "lost": lost,
               "share": (restored + lost) / max(1, n)}


def clear_lead_in(x: np.ndarray, min_run_us: float = 100.0, keep: int | None = None) -> np.ndarray:
    """Zero short bursts before the packet: ring-lap windows in the quiet gap that the repair
    missed would trigger the packet detector early. The packet starts at the first run of signal
    longer than min_run_us; everything more than `keep` samples before it becomes zero."""
    blk = max(16, int(round(1.6e-6 * FS_IN)))
    nb = x.size // blk
    if nb < 8:
        return x
    p = (np.abs(x[:nb * blk]) ** 2).reshape(nb, blk).mean(1)
    floor = np.percentile(p, 1)                       # the cut's quiet margins
    on = p > 30 * max(floor, 1e-9)
    need = int(min_run_us * 1e-6 * FS_IN / blk)
    run = 0
    for i, v in enumerate(on):
        run = run + 1 if v else 0
        if run >= need:
            first = (i - need + 1) * blk
            keep = int(0.3e-6 * FS_IN) if keep is None else keep    # the AGC burst starts the packet
            y = x.copy()
            y[:max(0, first - keep)] = 0
            return y
    return x


_PRE = None


def preamble_ref() -> np.ndarray:
    """AGC burst + Preamble A + Preamble B (Flexlink_BasicModules/Preamble/Preamble.h5) at FS_IN."""
    global _PRE
    if _PRE is None:
        import h5py
        with h5py.File(BASIC_MODULES / "Preamble" / "Preamble.h5", "r") as f:
            pre = np.asarray(f["Preamble"][()]).ravel()
        if pre.dtype.names:
            pre = pre[pre.dtype.names[0]] + 1j * pre[pre.dtype.names[1]]
        if FS_IN != 40_000_000:
            from scipy.signal import resample_poly
            pre = resample_poly(pre, 1, 40_000_000 // FS_IN)
        _PRE = pre
    return _PRE


def stream_check(buf: np.ndarray, buf0: int, s: int) -> dict | None:
    """Sample-stream integrity on the known preamble of the packet starting near s.
    buf: complex samples (DC removed), buf[0] at capture position buf0 (needs one ring lap before
    the packet for the lap test). The ESP-SDR S31 firmware (Oct 2026) puts part of every
    2016-sample DMA buffer one 16128-sample ring lap out of place; those samples then match the
    preamble only when taken 16128 samples earlier. Returns the share of preamble windows that are
    corrupt and the share of those that the one-lap shift explains."""
    try:
        from scipy.signal import fftconvolve
        pre = preamble_ref()
        n = pre.size
        sc = FS_IN / 40e6
        per = int(round(64 * sc))                    # Preamble A period
        lo = max(buf0, s - 8 * BLK)
        hi = min(buf0 + buf.size, s + n + int(8000 * sc))
        x = buf[lo - buf0:hi - buf0]
        t = np.arange(lo, hi)

        def cfo_of(seg):
            return float(np.angle(np.vdot(seg[:-per], seg[per:]))) * FS_IN / (2 * np.pi * per)

        f0 = cfo_of(x[s - lo: s - lo + int(9000 * sc)])          # coarse, from around the edge
        for _ in range(2):                                        # align, then refine on Preamble A
            y = x * np.exp(-2j * np.pi * f0 * t / FS_IN)
            j = int(np.argmax(np.abs(fftconvolve(y, np.conj(pre[::-1]), "valid"))))
            f0 = cfo_of(x[j + int(400 * sc): j + int(10000 * sc)])
        y = x * np.exp(-2j * np.pi * f0 * t / FS_IN)
        a = lo + j
        w = per

        def match(r, v):
            return np.abs(np.vdot(r, v)) / (np.linalg.norm(r) * np.linalg.norm(v) + 1e-9)

        q = np.array([match(pre[i:i + w], y[j + i:j + i + w]) for i in range(0, n - w, w)])
        med = float(np.median(q))                  # typical match: lower when the signal is weak
        tot = q.size
        bad = fixed = 0
        for k in np.flatnonzero(q < 0.5 * med):
            bad += 1
            i = k * w
            b0 = a + i - RING
            if b0 >= buf0:
                z = buf[b0 - buf0:b0 - buf0 + w] * np.exp(-2j * np.pi * f0 * np.arange(b0, b0 + w) / FS_IN)
                if z.size == w and match(pre[i:i + w], z) > 0.75 * med:
                    fixed += 1
        return {"bad": bad / max(1, tot), "lap": fixed / max(1, bad) if bad else 0.0, "cfo_hz": f0,
                "match": med}
    except Exception:  # noqa: BLE001
        return None


def burst_power(x: np.ndarray, n: int = 2048) -> float:
    m = x.size // n
    p = np.mean(np.abs(x[: m * n].reshape(m, n)) ** 2, axis=1)
    return float(p[p > 0.5 * p.max()].mean()) if p.size and p.max() > 0 else 0.0


def reference_power() -> float:
    """Burst power the Rx chain expects at 20 Msps: this PC's Tx waveform level, as the loopback."""
    try:
        import h5py
        name = json.loads(TX_JSON.read_text()).get("File", {}).get("Waveform_Payload_Tx") or "Waveform_Payload_Tx.h5"
        with h5py.File(TX / name, "r") as f:
            x = np.asarray(f["data"][()]).ravel()
            fs = float(np.asarray(f.attrs.get("SampleRate", FS_IN)))
        if x.dtype.names:
            x = x[x.dtype.names[0]] + 1j * x[x.dtype.names[1]]
        return burst_power(x) * (2.0 if fs == 2 * RX_RATE else 1.0)
    except Exception:  # noqa: BLE001  (no Tx run on this PC yet)
        return REF_POWER_20


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
    DEFECTS_OFF.parent.mkdir(parents=True, exist_ok=True)
    DEFECTS_OFF.write_text(json.dumps(d, indent=2) + "\n")
    return DEFECTS_OFF


# ---------- Rx chain ----------
class Log:
    def __init__(self, path: Path):
        self.fh = path.open("w")

    def write(self, text: str) -> None:
        self.fh.write(text if text.endswith("\n") else text + "\n")
        self.fh.flush()


def run_script(label: str, cmd: list, log: Log) -> tuple[int, str]:
    env = dict(os.environ)                     # flexlink_paths importable even without pip install -e .
    env["PYTHONPATH"] = os.pathsep.join(p for p in (str(ROOT), env.get("PYTHONPATH", "")) if p)
    p = subprocess.run([sys.executable, *map(str, cmd)], capture_output=True, text=True, cwd=ROOT, env=env)
    out = p.stdout + p.stderr
    log.write(f"===== {label} (exit {p.returncode})\n{out}")
    return p.returncode, out


def find(pattern: str, text: str):
    m = re.search(pattern, text)
    return m.group(1) if m else None


def num(v) -> float | None:
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def decode_output(log_text: str) -> str | None:
    """Output of the Get_Decoded_Bitstream.py step in a Run_Rx_Chain log (as Flexlinksimulation)."""
    sections = re.split(r"^===== ", log_text, flags=re.M)
    decode = [s for s in sections if s.startswith("Decode")]
    if not decode:
        return None
    return decode[-1].split("\n", 1)[1].rstrip() if "\n" in decode[-1] else ""


def control_output(log_text: str) -> str:
    """Control field decoding of the Rx chain (Process_Control_Field_Rx.py), as the Received tab."""
    sections = [s for s in re.split(r"^===== ", log_text or "", flags=re.M) if s.startswith("Control field")]
    lines = ["Reception (Rx, Process_Control_Field_Rx.py):"]
    if not sections:
        lines.append("  not run (no packet detected, or Use_ControlField is 0)")
    else:
        for ln in sections[-1].split("\n", 1)[-1].splitlines():
            if ln.strip() and not ln.startswith("Wrote") and not re.match(r"^  \w+ +[01]+  -> ", ln):
                lines.append("  " + ln)
        fields = [ln for ln in sections[-1].splitlines() if re.match(r"^  \w+ +[01]+  -> ", ln)]
        if fields:
            lines.append("  Fields:")
            lines += ["  " + ln for ln in fields]
    return "\n".join(lines)


def strip_ts(text: str) -> str:
    return "\n".join(re.sub(r"^\d\d:\d\d:\d\d (INFO|WARNING|ERROR|DEBUG) {1,4}", "", ln) for ln in text.splitlines())


def rx_params() -> dict:
    return json.loads(RX_JSON.read_text())


def evm_decision_directed(since: float) -> dict | None:
    """EVM of the equalized Data signals against the nearest constellation point (the transmitted
    blocks are not known here). Meaningful while the symbol error rate is low."""
    try:
        import h5py
        prm = rx_params()
        mod = str(prm["Defined_Rx"]["Mod"]).upper()
        name = prm.get("File_Rx", {}).get("Grid_Equalized_Rx") or "Grid_Equalized_Rx.h5"
        path = RX / ray_file_name(name, 1)
        if not path.is_file() or path.stat().st_mtime < since:
            return None
        with h5py.File(path, "r") as f:
            g = np.asarray(f["data"][()])
        gname = prm.get("File_Rx", {}).get("ResourceGrid_Rx") or "Resource_Grid_Rx.h5"
        with h5py.File(CONSTRUCT_GRID / gname, "r") as f:
            types = np.asarray(f["signal_types"][()])
            code = int(np.asarray(f.attrs.get("Data", 40)))
        if g.dtype.names:
            g = g[g.dtype.names[0]] + 1j * g[g.dtype.names[1]]
        s = g[types == code]
        m = {"BPSK": 2, "QPSK": 4, "16QAM": 16, "64QAM": 64}.get(mod)
        if m is None or s.size == 0:
            return None
        if m == 2:
            ref = np.sign(s.real) + 0j
        else:
            k = int(np.sqrt(m))
            norm = np.sqrt(2 * (m - 1) / 3)
            def sl(v):
                lvl = np.clip(np.round((v * norm + (k - 1)) / 2), 0, k - 1)
                return (2 * lvl - (k - 1)) / norm
            ref = sl(s.real) + 1j * sl(s.imag)
        e = np.mean(np.abs(s - ref) ** 2) / np.mean(np.abs(ref) ** 2)
        return {"pct": 100 * np.sqrt(e), "db": 10 * np.log10(e), "n": int(s.size), "mod": mod}
    except Exception:  # noqa: BLE001
        return None


def decode_packet(k: int, x20: np.ndarray, attrs: dict, defects: Path, log: Log) -> dict:
    t0 = time.time()
    write_rx_waveform(PACKET_RX, x20, attrs)
    rc, out = run_script(f"Run_Rx_Chain (packet {k})",
                         [RX / "Run_Rx_Chain.py", "--no-model", "--no-compare", "--input", PACKET_RX,
                          "--defects-file", defects, "--log", RX_LOG], log)
    try:
        rx_log = RX_LOG.read_text()
    except OSError:
        rx_log = ""
    log.write(rx_log)
    r = {"rc": rc, "status": RX_STATUS.get(rc, f"exit {rc}"), "since": t0,
         "cfo_khz": find(r"CFO\s+([+-]?\d[\d.]*) kHz", out), "snr_db": find(r"EQ SNR\s+(-?\d[\d.]*) dB", out),
         "cf_crc": find(r"control field: CRC (\w+)", out), "crc": find(r"CRC (\d+/\d+)", out.split("result:")[-1])
         if "result:" in out else None, "blocks": find(r"blocks ([PF0-9 ]+?)\s+bit errors", out),
         "log": rx_log, "stdout": out}
    if rc in (0, 1):
        r["evm"] = evm_decision_directed(t0)
        try:
            calc = rx_params()["Calculated_Rx"]
            r["tb_size"] = int(calc["TransportBlockSize"])
            r["ndb"] = int(calc["AvailableDataBlocks"])
        except (KeyError, ValueError, OSError):
            pass
    return r


def receive_file_blocks() -> tuple[object, dict, list] | None:
    """Add this packet's CRC-passed blocks to the file being received (Rx/Receive_File.py)."""
    import Receive_File as recv
    path = recv.default_blocks_file(RX_JSON)
    if not path.is_file() or recv.is_stale(path, RX_JSON):
        return None
    bits, crc = recv.load_blocks(path)
    return recv.receive(bits, crc, STATE_DIR, FILES_DIR)


# ---------- report ----------
def packet_line(k: int, a: int, b: int, r: dict, extra: str = "") -> str:
    evm = r.get("evm")
    evm_s = f"EVM {evm['pct']:5.2f} % ({evm['db']:5.1f} dB)" if evm else "EVM    -"
    return (f"  {k:3d}  t={a / FS_IN * 1e3:8.2f} ms  {(b - a) / FS_IN * 1e3:5.2f} ms  "
            f"CFO {r.get('cfo_khz') or '-':>8} kHz  SNR {r.get('snr_db') or '-':>5} dB  {evm_s}  "
            f"CF {r.get('cf_crc') or '-':4}  CRC {r.get('crc') or '-':>6}  {r['status']}{extra}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Find and decode the Flexlink packets in a CoreBoard capture.",
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--capture", type=Path, default=CAPTURE, help="int8 I/Q capture at 40 Msps")
    ap.add_argument("--max-packets", type=int, default=20, help="packets to decode (0 = all; default 20)")
    ap.add_argument("--packet", type=int, help="decode only this packet (1 = first whole packet)")
    ap.add_argument("--find-only", action="store_true", help="packet search only, no decoding")
    ap.add_argument("--thresh-db", type=float, help="detection threshold above the noise floor, dB "
                    "(default: halfway between noise and signal)")
    ap.add_argument("--plot", action="store_true", help="plot the last decoded packet (constellations, packet)")
    ap.add_argument("--no-repair", action="store_true",
                    help="don't repair the ESP-SDR ring-lap fault (default: repair when its pattern is found)")
    ap.add_argument("--keep-file-state", action="store_true",
                    help="keep the file blocks received by earlier runs (default: start afresh)")
    a = ap.parse_args(argv)

    report: list[str] = []

    def say(line: str = "") -> None:
        try:
            print(line, flush=True)
        except BrokenPipeError:
            pass
        report.append(line)

    try:
        iq, meta = load_capture(a.capture)
    except (FileNotFoundError, ValueError) as e:
        print(f"Error: {e}", file=sys.stderr)
        return 3
    n = iq.shape[0]

    # 1. capture and levels
    say("CoreBoard capture")
    say(f"  {a.capture.name}: {n} samples at {FS_IN / 1e6:g} Msps ({n / FS_IN:.2f} s)"
        + (f", captured {meta['CapturedAt']}, carrier {meta['CarrierHz'] / 1e6:g} MHz, "
           f"{'AGC' if meta.get('AGC') else 'gain ' + str(meta.get('Gain'))}" if meta else ""))
    if meta.get("LostSamples"):
        say(f"  lost in transfer: {meta['LostSamples']} samples ({meta.get('LostPackets')} datagrams), "
            f"filled with zeros; packets overlapping them will not decode")
    p, mean_blk = envelope(iq)
    dc = mean_blk.mean(axis=0)
    rms = np.sqrt(p.mean() / 2)
    peak = int(meta["Peak"]) if "Peak" in meta else None
    clip = meta.get("ClipFraction")
    say(f"  level: rms {rms:.1f} LSB ({20 * np.log10(rms / 128):.1f} dBFS)"
        + (f", peak {peak} of 127" if peak is not None else "")
        + (f", clipped {100 * clip:.3f} %" if clip is not None else "")
        + f", DC {dc[0]:+.2f}/{dc[1]:+.2f} LSB (removed)"
        + ("   CLIPPING: lower the CoreBoard gain or the Pluto Tx gain" if clip and clip > 1e-4 else ""))

    # 2. packets
    pk, lv = find_packets(p, thresh_db=a.thresh_db)
    say(f"  envelope: noise floor {lv['floor_db'] - 10 * np.log10(2 * 128 ** 2):.1f} dBFS, packets "
        f"{lv['signal_db'] - 10 * np.log10(2 * 128 ** 2):.1f} dBFS "
        f"(packet-to-noise {lv['signal_db'] - lv['floor_db']:.1f} dB)")
    if not pk:
        say(f"  No whole packet found: {lv.get('reason', 'no rise and fall of the power envelope')}. "
            f"Was the transmitter on during the capture, on the same carrier?")
        RESULTS_TXT.parent.mkdir(parents=True, exist_ok=True)
        RESULTS_TXT.write_text("\n".join(report) + "\n")
        return 2
    for ln in describe_packets(pk):
        say("  " + ln)
    if lv.get("short"):
        say(f"  {len(lv['short'])} shorter burst(s) skipped (Wi-Fi or other signals, or packets cut by lost "
            f"samples), at " + ", ".join(f"{a / FS_IN * 1e3:.1f}" for a, _ in lv["short"][:6])
            + (" ..." if len(lv["short"]) > 6 else "") + " ms")
    if a.find_only:
        for i, (s, e) in enumerate(pk[:50], 1):
            say(f"  {i:3d}  start {s:10d}  ({s / FS_IN * 1e3:8.3f} ms)  length {e - s:7d}")
        RESULTS_TXT.write_text("\n".join(report) + "\n")
        return 0

    if a.packet is not None:
        if not 1 <= a.packet <= len(pk):
            print(f"Error: --packet must be 1 to {len(pk)}", file=sys.stderr)
            return 3
        todo = [a.packet]
    else:
        todo = list(range(1, len(pk) + 1))
    limit = len(todo) if a.max_packets == 0 or a.packet else min(a.max_packets, len(todo))

    # 3. Rx chain setup
    CB_DIR.mkdir(parents=True, exist_ok=True)
    log = Log(PROCESS_LOG)
    rc, out = run_script("Initialize_Rx", [RX / "Initialize_Rx.py"], log)
    if rc:
        print(f"Error: Initialize_Rx.py failed (exit {rc}), see {PROCESS_LOG}", file=sys.stderr)
        return 3
    defects = defects_off_file()
    ref = reference_power()
    if not a.keep_file_state:
        from File_Framing import FileReceiver
        FileReceiver(STATE_DIR).reset()

    # 4./5. per packet
    say("")
    say(f"Packets (Rx chain at 20 Msps; first {limit} of {len(pk)}"
        + (", every packet while a file is being received" if a.packet is None else "") + ")")
    say("    #  start           length   CFO              EQ SNR    EVM (decision-directed)      "
        "control   blocks  result")
    rows, results = [], []
    first_good = last_good = None
    file_mode = False
    file_done = None
    real_files: set[int] = set()
    t_start = time.time()
    for idx, k in enumerate(todo):
        if idx >= limit and not (file_mode and file_done is None):
            break
        s, e = pk[k - 1]
        margin = MARGIN20 * FS_IN // RX_RATE
        a40 = max(0, s - margin)
        b40 = min(n, e + margin)
        lo = max(0, a40 - 2 * RING)                  # one ring lap of history for repair and check
        raw = to_complex(iq, lo, b40, dc)
        rep = None
        buf = raw
        if not a.no_repair:
            tlo, thi = max(0, s - 24 * RING), min(n, e + 24 * RING)
            wins, _ = ring_template(to_complex(iq, tlo, thi, dc), tlo, FS_IN)
            if wins:
                buf, rep = ring_repair(raw, lo, FS_IN, wins)
                rep["per_lap"] = len(wins)
        sc_raw = stream_check(raw, lo, s)
        sc_rep = stream_check(buf, lo, s) if rep else None
        if rep and sc_raw and sc_rep and sc_rep["bad"] >= sc_raw["bad"]:
            buf, rep, sc_rep = raw, None, None        # the repair didn't help this packet: keep the original
        cut = buf[a40 - lo:b40 - lo]
        if rep:
            cut = clear_lead_in(cut)
        x20 = to_rx(cut)
        x20 = settle_lead_in(x20, (s - a40) * RX_RATE // FS_IN - 32)
        got = burst_power(x20)
        gain = float(np.sqrt(ref / got)) if got > 0 else 1.0
        x20 = x20 * gain
        r = decode_packet(k, x20, {"SampleRate": RX_RATE, "Source": "ESP32-S31 CoreBoard capture",
                                   "CaptureRate": FS_IN, "CaptureStart": a40,
                                   "CarrierHz": float(meta.get("CarrierHz", 0)), "LevelGain": gain}, defects, log)
        extra = ""
        r["stream"] = sc_raw
        r["stream_rep"] = sc_rep
        r["repair"] = rep
        if r["rc"] in (0, 1):
            last_good = (k, r)
            if first_good is None:
                first_good = (k, r)
            try:
                got_f = receive_file_blocks()
            except Exception as ex:  # noqa: BLE001
                got_f = None
                log.write(f"Receive_File: {ex}")
            if got_f is not None:
                rep, status, written = got_f
                for fid, st in status.items():
                    # a random block starts with the file-block magic byte now and then: a file is
                    # only taken as real once 3 different blocks of it have arrived (or it is complete)
                    if st["received"] >= FILE_MIN_BLOCKS or written:
                        file_mode = True
                        real_files.add(fid)
                    if fid in real_files:
                        extra = f"  file {fid}: {st['received']}/{st['total']} blocks"
                        if st.get("error"):
                            extra += f" ({st['error']})"
                if written:
                    file_done = written[0]
                    extra += f"  -> complete: {file_done.name}"
        results.append((k, s, e, r))
        say(packet_line(k, s, e, r, extra))
        rows.append({"packet": k, "start_sample_40M": s, "length_samples_40M": e - s, "rx_exit": r["rc"],
                     "result": r["status"], "cfo_khz": r.get("cfo_khz"), "eq_snr_db": r.get("snr_db"),
                     "evm_pct": None if not r.get("evm") else round(r["evm"]["pct"], 3),
                     "evm_db": None if not r.get("evm") else round(r["evm"]["db"], 2),
                     "control_crc": r.get("cf_crc"), "crc": r.get("crc"), "blocks": r.get("blocks"),
                     "preamble_corrupt_pct": None if not r.get("stream") else round(100 * r["stream"]["bad"], 1),
                     "preamble_corrupt_after_repair_pct": None if not r.get("stream_rep")
                     else round(100 * r["stream_rep"]["bad"], 1),
                     "repaired_samples": None if not r.get("repair") else r["repair"]["restored"],
                     "zeroed_samples": None if not r.get("repair") else r["repair"]["lost"],
                     "level_gain": round(gain, 5)})

    # summary
    dec = [r for _, _, _, r in results]
    ok_pk = sum(1 for r in dec if r["rc"] == 0)
    some = sum(1 for r in dec if r["rc"] in (0, 1))
    blocks_ok = blocks_all = 0
    for r in dec:
        if r.get("crc"):
            x, y = map(int, r["crc"].split("/"))
            blocks_ok += x
            blocks_all += y
    cfo = [v for v in (num(r.get("cfo_khz")) for r in dec) if v is not None]
    snr = [v for v in (num(r.get("snr_db")) for r in dec) if v is not None]
    evm = [r["evm"]["db"] for r in dec if r.get("evm")]
    say("")
    say("Summary")
    say(f"  {len(dec)} packet(s) decoded in {time.time() - t_start:.0f} s: {ok_pk} with every CRC passing, "
        f"{some - ok_pk} with failed blocks, {len(dec) - some} not decoded"
        + "".join(f" ({sum(1 for r in dec if r['rc'] == c)} {RX_STATUS[c]})" for c in (2, 3, 4)
                    if any(r["rc"] == c for r in dec)))
    sc = [r["stream"] for r in dec if r.get("stream")]
    if sc:
        badp = 100 * float(np.median([v["bad"] for v in sc]))
        lap = [v["lap"] for v in sc if v["bad"] > 0.02]
        if badp > 2:
            say(f"  SAMPLE STREAM CORRUPT: {badp:.0f} % of the preamble samples (median per packet) are out of place"
                + (f"; {100 * float(np.median(lap)):.0f} % of those are the samples of one {RING}-sample DMA ring "
                   f"lap away - the ESP-SDR firmware copies part of each DMA buffer one ring lap out of step"
                   + ("" if any(r.get("repair") for r in dec) else
                      ". Capture at 20 Msps (FlexlinkCaptureWithCoreBoard.py --rate 20000000) for a smaller share"
                      if FS_IN == 40_000_000 else " (seen at 20 Msps too)")
                   if lap and np.median(lap) > 0.5 else "")
                + (" - repaired, see below." if any(r.get("repair") for r in dec)
                   else ". Decoding results above are limited by this, not by the radio link."))
        else:
            say(f"  sample stream: preamble intact ({badp:.1f} % of windows mismatched, noise level)")
    reps = [r["repair"] for r in dec if r.get("repair")]
    if reps:
        after = [r["stream_rep"]["bad"] for r in dec if r.get("stream_rep")]
        say(f"  ring-lap repair: {int(np.median([v['per_lap'] for v in reps]))} windows per ring lap found; "
            f"{100 * np.mean([v['restored'] / max(1, v['restored'] + v['lost']) for v in reps]) * np.mean([v['share'] for v in reps]):.1f} % "
            f"of the samples moved back, "
            f"{100 * np.mean([v['lost'] / max(1, v['restored'] + v['lost']) for v in reps]) * np.mean([v['share'] for v in reps]):.1f} % "
            f"unrecoverable (set to 0)"
            + (f"; preamble after repair: {100 * float(np.median(after)):.1f} % of windows mismatched" if after else ""))
    elif not a.no_repair and sc and np.median([v["bad"] for v in sc]) > 0.02:
        say("  ring-lap repair: the fault's window pattern was not found, nothing repaired")
    if blocks_all:
        say(f"  transport blocks: {blocks_ok} of {blocks_all} CRC ok (BLER {1 - blocks_ok / blocks_all:.2e})")
    if cfo:
        say(f"  CFO {np.median(cfo):+.2f} kHz median ({min(cfo):+.2f} .. {max(cfo):+.2f}) = "
            f"{np.median(cfo) * 1e3 / max(1.0, float(meta.get('CarrierHz', 2.4e9))) * 1e6:+.1f} ppm of the carrier: "
            f"Pluto vs CoreBoard crystals")
    if snr:
        say(f"  EQ SNR {np.median(snr):.1f} dB median ({min(snr):.1f} .. {max(snr):.1f})"
            + (f", EVM {np.median(evm):.1f} dB median" if evm else ""))
    if last_good and last_good[1].get("tb_size"):
        r = last_good[1]
        bits = r["ndb"] * r["tb_size"]
        span = None
        if len(pk) > 1:
            per = np.diff([a for a, _ in pk])
            span = float(np.median(per[per < 1.5 * np.median(per)]))
        say(f"  {r['ndb']} x {r['tb_size']} = {bits} transport block bits per packet"
            + (f"; packet period {span / FS_IN * 1e3:.3f} ms -> {bits / (span / FS_IN) / 1e6:.3f} Mbit/s on air, "
               f"{bits * blocks_ok / max(1, blocks_all) / (span / FS_IN) / 1e6:.3f} Mbit/s with the CRC-ok share"
               if span else ""))
    if file_mode:
        if file_done:
            say(f"  file received: {file_done} ({file_done.stat().st_size} bytes, size and SHA-256 match)")
        else:
            try:
                from File_Framing import FileReceiver
                import Receive_File as recv
                fr = FileReceiver(STATE_DIR)
                for fid in sorted(real_files):
                    st = fr.status(fid)
                    say(f"  file {fid}: {st['received']} of {st['total']} blocks, missing "
                        f"{recv.ranges(st['missing'])} - capture longer or send the file again")
            except Exception:  # noqa: BLE001
                pass

    # the Received-tab style details of the first fully decoded packet (else the first decoded one)
    detail = next(((k, r) for k, _, _, r in results if r["rc"] == 0), first_good)
    if detail is None and results:
        detail = (results[0][0], results[0][3])
    if detail:
        k, r = detail
        say("")
        say(f"Control Field Results (packet {k})")
        say(control_output(r["log"]))
        say("")
        say(f"Payload Field Results (packet {k})")
        out = decode_output(r["log"])
        if r["rc"] == 4:
            out = "No payload decoded: the control field CRC failed, so the packet was dropped."
        say(strip_ts(out) if out else r["stdout"].rstrip())
        if r.get("evm"):
            ev = r["evm"]
            say(f"EVM (ray 1 after equalization, decision-directed, {ev['mod']}): {ev['pct']:.2f} % "
                f"({ev['db']:.1f} dB) over {ev['n']} Data signals")

    RESULTS_TXT.write_text("\n".join(report) + "\n")
    with PACKETS_CSV.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]) if rows else ["packet"])
        w.writeheader()
        w.writerows(rows)
    print(f"\nWrote {RESULTS_TXT.relative_to(ROOT)}, {PACKETS_CSV.relative_to(ROOT)}, "
          f"{PROCESS_LOG.relative_to(ROOT)}")

    if a.plot and last_good:
        k, r = last_good
        try:
            import Flexlink_Plots as P
            msgs = P.make_plots({"before": True, "after": True}, 15, RX_JSON, r["since"])
            pkt = RX / ray_file_name(rx_params().get("File_Rx", {}).get("Waveform_Payload_Packet_Rx")
                                     or "Decimated_Payload_Packet_Rx.h5", 1)
            if pkt.is_file():
                w, at = P.load_waveform(pkt)
                P.plot_rx_packet(np.atleast_2d(w)[0], at, f"CoreBoard: packet {k} as the Rx chain found it (20 Msps)")
            for m in msgs:
                print("Plot:", m)
            import matplotlib.pyplot as plt
            print(f"Plots of packet {k}; close the windows to finish.")
            plt.show(block=True)
        except Exception as ex:  # noqa: BLE001
            print(f"Plots failed: {ex}")

    if ok_pk:
        return 0
    return 1 if results else 2


if __name__ == "__main__":
    sys.exit(main())
