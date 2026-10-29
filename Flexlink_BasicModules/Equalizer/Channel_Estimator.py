"""
Channel_Estimator.py  (lives in Flexlink_py/Flexlink_BasicModules/Equalizer)

Pilot-based channel estimation and one-tap equalization for the FlexLink payload grid.
Used by Rx/Equalize_Payload_Grid.py.

Grid conventions (Construct_Resource_Grid.py):
    signal_types codes  P0 = 10, P1 = 20, DC = 30, Data = 40
    841 tones, k = 0..840, tone = k - 420
    one Ref_P<port> vector repeated in every Symbol_Ref, in increasing k
    only one port's comb is used; the other comb is empty (used for the noise estimate)

Estimators (all are fixed matrices, applied as H = T @ (H_ls @ W.T)):
    AdaptiveChannelEstimator : LMMSE across frequency with a delay window (start and end)
                               chosen per packet from the measured delay profile, then
                               Wiener (or linear) interpolation across time. <- recommended
    ChannelEstimator         : same building blocks with a fixed choice:
                               freq "linear" | "dft" | "lmmse", time "linear" | "wiener".
    FixedFIRChannelEstimator : the FPGA control-field estimator: fixed 8-tap LMMSE across
                               frequency, drift-corrected linear interpolation across time.

Equalizers:
    equalize_for_demapper(Y, H, nv) -> (Y/H, nv/|H|^2)   unbiased symbols + per-tone noise
    equalize(Y, H, nv)              -> MMSE symbols (biased toward 0 on faded tones)

Numerology defaults: N_FFT 1024, CP 116 samples, 19531.25 Hz subcarrier spacing (20 Msps).
Source: pilot_ce.py (validated against the FlexLink Tx/Rx grids, Oct 2026).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

P0, P1, DC, DATA = 10, 20, 30, 40
PORT_CODE = {0: P0, 1: P1}


@dataclass
class GridLayout:
    types: np.ndarray            # (n_sym, n_tone) int8 signal-type map
    tones: np.ndarray            # (n_tone,) tone index, e.g. -420..420
    n_fft: int = 1024            # FFT size: 20 Msps / 19531.25 Hz = 1024
    cp_len: int = 116            # cyclic prefix in samples (5.8 us at 20 Msps)
    scs_hz: float = 19531.25     # subcarrier spacing
    meta: dict = field(default_factory=dict)

    @property
    def n_sym(self):
        return self.types.shape[0]

    @property
    def ref_symbols(self):
        """Indices of reference symbols (any symbol carrying P0 or P1)."""
        return np.where(((self.types == P0) | (self.types == P1)).any(axis=1))[0]

    def pilot_idx(self, sym, port):
        """Column indices of the pilot comb for `port` (0 or 1) in symbol `sym`."""
        return np.where(self.types[sym] == PORT_CODE[port])[0]

    @property
    def t_sym(self):
        """OFDM symbol duration including CP [s]."""
        return (self.n_fft + self.cp_len) / (self.n_fft * self.scs_hz)



# --------------------------------------------------------------------------------------
# Receiver helpers
# --------------------------------------------------------------------------------------
def detect_port(Y: np.ndarray, layout: GridLayout) -> int:
    """Which comb is live: compare received energy on P0 vs P1 tones."""
    rs = layout.ref_symbols
    e0 = np.mean(np.abs(Y[rs][layout.types[rs] == P0]) ** 2)
    e1 = np.mean(np.abs(Y[rs][layout.types[rs] == P1]) ** 2)
    return 0 if e0 >= e1 else 1


def estimate_noise_var(Y: np.ndarray, layout: GridLayout, port: int) -> float:
    """sigma^2 from tones that carry nothing: the inactive pilot comb plus in-band nulls
    in reference symbols. (If the other comb is used by a second Tx antenna, this
    would measure that antenna instead - use only the null tones then.)"""
    rs = layout.ref_symbols
    other = PORT_CODE[1 - port]
    t = layout.types[rs]
    inband = np.abs(layout.tones) < layout.tones.max()           # skip band-edge nulls
    mask = (t == other) | ((t == DC) & inband[None, :])
    return float(np.mean(np.abs(Y[rs][mask]) ** 2))



# --------------------------------------------------------------------------------------
# Frequency-direction filters (each returns a matrix W: H_all_tones = W @ H_ls_pilots)
# --------------------------------------------------------------------------------------
def _uniform_pdp_corr(dk, n_fft, delays):
    """Frequency correlation for equal-power taps at the given (integer) delays."""
    return np.exp(-2j * np.pi * np.multiply.outer(dk, delays) / n_fft).mean(axis=-1)


def w_linear(tones, p_tones):
    """Linear interpolation, flat extrapolation at the edges (== MATLAB interp1 + hold)."""
    W = np.zeros((tones.size, p_tones.size))
    j = np.clip(np.searchsorted(p_tones, tones) - 1, 0, p_tones.size - 2)
    a = np.clip((tones - p_tones[j]) / (p_tones[j + 1] - p_tones[j]), 0.0, 1.0)
    W[np.arange(tones.size), j] = 1 - a
    W[np.arange(tones.size), j + 1] = a
    return W


def w_delay_ls(tones, p_tones, n_fft, delays, reg=1e-3):
    """'DFT-based' estimator generalised to a partial band with gaps: least-squares fit of
    a channel impulse response restricted to `delays` (e.g. the CP), then evaluated on
    every tone. Equivalent to IFFT -> zero taps beyond CP -> FFT, without the edge leakage
    you get from doing literal FFTs on 841-of-1024 tones."""
    Fp = np.exp(-2j * np.pi * np.outer(p_tones, delays) / n_fft)
    Fa = np.exp(-2j * np.pi * np.outer(tones, delays) / n_fft)
    G = Fp.conj().T @ Fp
    G += reg * np.trace(G).real / G.shape[0] * np.eye(G.shape[0])
    return Fa @ np.linalg.solve(G, Fp.conj().T)


def w_lmmse(tones, p_tones, n_fft, delays, noise_var, pdp=None):
    """Wiener / LMMSE: W = R_hp (R_pp + sigma^2 I)^-1.
    Default prior = uniform power over `delays` (the 'robust' design: valid for any channel
    whose delay spread fits in that window). Pass `pdp=(delays_samples, powers_lin)` to use
    a specific profile instead."""
    if pdp is None:
        Rpp = _uniform_pdp_corr(np.subtract.outer(p_tones, p_tones), n_fft, delays)
        Rhp = _uniform_pdp_corr(np.subtract.outer(tones, p_tones), n_fft, delays)
    else:
        d, p = np.asarray(pdp[0], float), np.asarray(pdp[1], float) / np.sum(pdp[1])
        Rpp = np.exp(-2j * np.pi * np.multiply.outer(np.subtract.outer(p_tones, p_tones), d) / n_fft) @ p
        Rhp = np.exp(-2j * np.pi * np.multiply.outer(np.subtract.outer(tones, p_tones), d) / n_fft) @ p
    return Rhp @ np.linalg.inv(Rpp + noise_var * np.eye(p_tones.size))


# --------------------------------------------------------------------------------------
# Time-direction filters (matrix T: H_all_symbols = T @ H_ref_symbols, per tone)
# --------------------------------------------------------------------------------------
def j0(x):
    """Bessel J0 via its integral form (numpy only, accurate to ~1e-12 for |x| < 50)."""
    th = (np.arange(256) + 0.5) * np.pi / 256
    x = np.asarray(x, float)
    return np.cos(np.multiply.outer(x, np.sin(th))).mean(axis=-1)


def t_linear(n_sym, ref_syms):
    syms = np.arange(n_sym)
    j = np.clip(np.searchsorted(ref_syms, syms, side="right") - 1, 0, ref_syms.size - 2)
    a = np.clip((syms - ref_syms[j]) / (ref_syms[j + 1] - ref_syms[j]), 0.0, 1.0)
    T = np.zeros((n_sym, ref_syms.size))
    T[syms, j] = 1 - a
    T[syms, j + 1] = a
    return T


def t_wiener(n_sym, ref_syms, t_sym, fd_hz, noise_var_eff):
    """Time Wiener filter with a Jakes (Clarke) correlation J0(2*pi*fd*dt).
    noise_var_eff is the error variance of the per-symbol frequency estimates."""
    syms = np.arange(n_sym)
    Rpp = j0(2 * np.pi * fd_hz * t_sym * np.subtract.outer(ref_syms, ref_syms))
    Rhp = j0(2 * np.pi * fd_hz * t_sym * np.subtract.outer(syms, ref_syms))
    return Rhp @ np.linalg.inv(Rpp + noise_var_eff * np.eye(ref_syms.size))


# --------------------------------------------------------------------------------------
# Full 2-D estimator
# --------------------------------------------------------------------------------------
class ChannelEstimator:
    """Precomputes the filters once per (layout, port, method, SNR design point);
    `estimate(Y, Xref)` is then two small matrix multiplies per packet."""

    def __init__(self, layout: GridLayout, port: int, freq="lmmse", time="wiener",
                 noise_var=0.01, fd_hz=300.0, delay_window=None, pdp=None, pilot_power=1.0):
        """noise_var   : sigma^2 per tone (estimate_noise_var gives it directly)
        pilot_power : mean |pilot|^2. Ref vectors use the constellation corner, so
                      1.0 (BPSK/QPSK), 1.8 (16QAM), 2.333 (64QAM). LS noise = sigma^2/pilot_power.
        delay_window: (first, last+1) delay in samples the channel may occupy. Default
                      (-4, min(CP, 0.9*N_FFT/S)): with a pilot every S tones only delays up
                      to N_FFT/S samples can be told apart (85 at S=12, 42 at S=24)."""
        self.layout, self.port = layout, port
        self.ref_syms = layout.ref_symbols
        cols = layout.pilot_idx(self.ref_syms[0], port)
        if not all(np.array_equal(cols, layout.pilot_idx(s, port)) for s in self.ref_syms):
            raise NotImplementedError("staggered pilots: build one W per pilot pattern")
        self.cols = cols
        tones, pt = layout.tones.astype(float), layout.tones[cols].astype(float)
        # Default window starts 4 samples early: fractional-sample paths near delay 0
        # (e.g. a 10 ns channel at 20 Msps) leak into 'negative' delay bins because only
        # 841 of 1024 tones are used; excluding them leaves an error floor around -30 dB.
        spacing = int(np.median(np.diff(layout.tones[cols])))
        self.spacing = spacing
        if delay_window is None:
            delay_window = (-4, min(layout.cp_len, int(0.9 * layout.n_fft / spacing)))
        d0, d1 = delay_window
        self.delay_window = delay_window
        delays = np.arange(d0, d1)
        noise_var = noise_var / pilot_power        # noise on the LS estimates

        if freq == "linear":
            self.W = w_linear(tones, pt)
        elif freq == "dft":
            self.W = w_delay_ls(tones, pt, layout.n_fft, delays)
        elif freq == "lmmse":
            self.W = w_lmmse(tones, pt, layout.n_fft, delays, noise_var, pdp)
        else:
            raise ValueError(freq)

        # residual error variance after the frequency filter (for the time Wiener filter)
        nv_eff = noise_var * float(np.mean(np.sum(np.abs(self.W) ** 2, axis=1)))
        if time == "linear":
            self.T = t_linear(layout.n_sym, self.ref_syms)
        elif time == "wiener":
            self.T = t_wiener(layout.n_sym, self.ref_syms, layout.t_sym, fd_hz, nv_eff)
        else:
            raise ValueError(time)

    def estimate(self, Y, Xref):
        Hls = Y[np.ix_(self.ref_syms, self.cols)] / Xref[np.ix_(self.ref_syms, self.cols)]
        Hf = Hls @ self.W.T              # (n_ref, n_tone)   frequency direction
        return self.T @ Hf               # (n_sym, n_tone)   time direction


# --------------------------------------------------------------------------------------
# Adaptive delay window
# --------------------------------------------------------------------------------------
WINDOW_ENDS = (8, 12, 16, 24, 32, 48, 64, 80, 100, 116)   # candidate last-delay (samples)
WINDOW_STARTS = (-4, -8, -12, -16, -24, -32, -48, -64, -80, -100, -116)  # candidate first delay


def delay_profile(Hls, pilot_tones, n_fft, delays):
    """Average power-delay profile seen by the pilots: Hann-windowed inverse DFT of the
    LS estimates at each reference symbol, |.|^2 averaged over reference symbols."""
    w = np.hanning(pilot_tones.size + 2)[1:-1]
    E = np.exp(2j * np.pi * np.outer(pilot_tones, delays) / n_fft)          # (Np, nd)
    h = (Hls * w) @ E / w.sum()                                               # (n_ref, nd)
    return np.mean(np.abs(h) ** 2, axis=0), float(np.sum(w ** 2) / w.sum() ** 2)


def choose_window_end(Hls, pilot_tones, n_fft, noise_var_ls, limit, rel_db=-30.0, margin=4):
    """Last delay (samples) holding channel energy: profile above both rel_db re. its
    peak and 4x the noise floor; plus `margin`, rounded up to a WINDOW_ENDS entry."""
    delays = np.arange(-8, int(limit))
    pdp, g = delay_profile(Hls, pilot_tones, n_fft, delays)
    floor = noise_var_ls * g
    thr = max(pdp.max() * 10 ** (rel_db / 10), 4 * floor)
    above = delays[pdp > thr]
    last = (above.max() if above.size else 0) + margin
    for e in WINDOW_ENDS:
        if e >= last and e <= limit:
            return e
    return int(limit)


def channel_extent(Hls, pilot_tones, n_fft, noise_var_ls, limit, rel_db=-30.0):
    """(first, last) delay in samples holding channel energy: the delay profile above both
    rel_db re. its peak and 4x the noise floor, searched from -limit-8 to limit. `first` is
    negative when timing locked onto a later path (see choose_window)."""
    delays = np.arange(-int(limit) - 8, int(limit))
    pdp, g = delay_profile(Hls, pilot_tones, n_fft, delays)
    thr = max(pdp.max() * 10 ** (rel_db / 10), 4 * noise_var_ls * g)
    above = delays[pdp > thr]
    if not above.size:
        return 0, 0
    return int(above.min()), int(above.max())


def shift_delays(H, tones, n_fft, shift):
    """Delay a channel by `shift` samples in the frequency domain: H(k) exp(-j 2 pi k shift / N).
    A negative shift advances it. Used to move a channel that starts before delay 0 back to 0."""
    return H * np.exp(-2j * np.pi * np.asarray(tones, float) * shift / n_fft)


def choose_window(Hls, pilot_tones, n_fft, noise_var_ls, limit, span, rel_db=-30.0, margin=4):
    """(first, last+1) delays holding channel energy, for the LMMSE window.

    The end is choose_window_end(). The start follows the earliest delay above the same
    threshold, minus `margin`, rounded down to a WINDOW_STARTS entry (never later than -4).
    Paths can sit well before delay 0: the packet detector's timing locks onto the
    strongest path, which in ETU/EVA is often a later one, so earlier paths arrive at
    negative delays (FFT_StartTime -12 keeps up to 12 samples of them free of ISI). With a
    fixed start of -4 the LMMSE filter removed them and the estimate failed (about -5 dB
    error instead of -38 dB). If the window would be longer than the delays the pilots can
    resolve (`span` = 0.9*N_FFT/S), the start falls back to -4."""
    end = choose_window_end(Hls, pilot_tones, n_fft, noise_var_ls, limit, rel_db, margin)
    first = channel_extent(Hls, pilot_tones, n_fft, noise_var_ls, limit, rel_db)[0] - margin
    start = next((s for s in WINDOW_STARTS if s <= first), WINDOW_STARTS[-1])
    start = min(start, -4)
    if end - start > span:
        start = -4
    return int(start), int(end)


class AdaptiveChannelEstimator:
    """LMMSE (freq) + Wiener (time) whose delay window follows the measured channel.

    For every packet: estimate the delay profile from the pilots, pick the window start from
    WINDOW_STARTS and the end from WINDOW_ENDS (end capped at min(CP, 0.9*N_FFT/S), total
    length at 0.9*N_FFT/S), and use a cached ChannelEstimator for (window, SNR design point). Wide windows suit long channels (ETU); short windows
    matter for short channels at sparse pilot spacings (12, 24)."""

    def __init__(self, layout, port, fd_hz=300.0, pilot_power=1.0, snr_grid_db=range(0, 46, 5),
                 time="wiener", freq="lmmse"):
        """fd_hz: worst-case Doppler the time filter is designed for (robust for anything
        slower). Per-packet Doppler estimation was tried and rejected: one packet is too
        short to estimate it reliably, and over-estimates hurt at PacketProfile 12."""
        self.layout, self.port, self.fd_hz, self.pp = layout, port, fd_hz, pilot_power
        self.snr_grid = np.asarray(list(snr_grid_db), float)
        self.time, self.freq = time, freq
        self.ref_syms = layout.ref_symbols
        self.cols = layout.pilot_idx(self.ref_syms[0], port)
        S = int(np.median(np.diff(layout.tones[self.cols])))
        self.span = int(0.9 * layout.n_fft / S)      # delays the pilot comb can resolve
        self.limit = min(layout.cp_len, self.span)
        self._cache = {}
        self.last_window = None

    def _get(self, start, end, snr_db, fd):
        key = (start, end, snr_db, fd)
        if key not in self._cache:
            self._cache[key] = ChannelEstimator(self.layout, self.port, self.freq, self.time,
                                                noise_var=10 ** (-snr_db / 10), fd_hz=fd,
                                                delay_window=(start, end), pilot_power=self.pp)
        return self._cache[key]

    def estimate(self, Y, Xref, noise_var):
        Hls = Y[np.ix_(self.ref_syms, self.cols)] / Xref[np.ix_(self.ref_syms, self.cols)]
        start, end = choose_window(Hls, self.layout.tones[self.cols].astype(float), self.layout.n_fft,
                                   noise_var / self.pp, self.limit, self.span)
        snr = self.snr_grid[np.argmin(np.abs(self.snr_grid + 10 * np.log10(noise_var)))]
        self.last_window = (start, end)
        return self._get(start, end, snr, self.fd_hz).estimate(Y, Xref)


# --------------------------------------------------------------------------------------
# Fixed short FIR (the FPGA control-field estimator)
# --------------------------------------------------------------------------------------
def w_lmmse_banded(tones, p_tones, n_fft, delays, noise_var, taps):
    """LMMSE weights restricted to the `taps` pilots nearest each tone (a banded W): every
    row is the LMMSE solution for its own short pilot window, uniform power over `delays`.
    Tones outside the pilot range use the window at the band edge."""
    W = np.zeros((tones.size, p_tones.size), complex)
    for i, t in enumerate(tones):
        near = np.sort(np.argsort(np.abs(p_tones - t), kind="stable")[:taps])
        p = p_tones[near]
        Rpp = _uniform_pdp_corr(np.subtract.outer(p, p), n_fft, delays)
        Rhp = _uniform_pdp_corr(np.subtract.outer(np.array([t]), p), n_fft, delays)
        W[i, near] = (Rhp @ np.linalg.inv(Rpp + noise_var * np.eye(taps)))[0]
    return W


class FixedFIRChannelEstimator:
    """The control-field estimator of the FPGA receiver (Docs/Specification_ControlField.md,
    HW_Development/Packet_Detector/PC/cf_reference.py), in floating point:

      frequency  every tone from its `taps` nearest pilots, fixed LMMSE weights for a flat
                 delay profile over `delay_window` and a design pilot noise `design_noise`
                 (the same weights for every packet: no window choice, no noise estimate)
      time       between two consecutive reference symbols r0 < r1: the common phase drift
                 phi = angle(sum_pilots Hls(r1) Hls(r0)*) is removed from H(r1), the two are
                 interpolated linearly and the drift put back:
                     H(s) = ((r1 - s) H(r0) + (s - r0) H(r1) e^-j phi) / (r1 - r0)
                            * e^(j phi (s - r0) / (r1 - r0))
                 symbols before the first / after the last reference symbol use the nearest
                 pair (extrapolation).

    delay_window is in the frame of the demodulated grid, i.e. after Demodulate_Waveform_Rx
    removed the FFT_StartTime phase ramp. The FPGA keeps the ramp and uses -8 .. 24 samples
    with FFT_StartTime -12; that is -20 .. 12 here, the default."""

    def __init__(self, layout: GridLayout, port: int, taps=8, delay_window=(-20, 12),
                 design_noise=0.5, pilot_power=1.0):
        self.layout, self.port = layout, port
        self.ref_syms = layout.ref_symbols
        cols = layout.pilot_idx(self.ref_syms[0], port)
        if not all(np.array_equal(cols, layout.pilot_idx(s, port)) for s in self.ref_syms):
            raise NotImplementedError("staggered pilots: build one W per pilot pattern")
        self.cols = cols
        self.delay_window = tuple(delay_window)
        self.W = w_lmmse_banded(layout.tones.astype(float), layout.tones[cols].astype(float),
                                layout.n_fft, np.arange(*delay_window), design_noise / pilot_power, taps)
        self.last_window = self.delay_window
        self.last_drift = None

    def estimate(self, Y, Xref, noise_var=None):
        Hls = Y[np.ix_(self.ref_syms, self.cols)] / Xref[np.ix_(self.ref_syms, self.cols)]
        Hf = Hls @ self.W.T                                   # (n_ref, n_tone)
        rs = self.ref_syms
        H = np.zeros((self.layout.n_sym, Hf.shape[1]), complex)
        drift = []
        for j in range(rs.size - 1):
            drift.append(np.angle(np.sum(Hls[j + 1] * np.conj(Hls[j]))))
        for s in range(self.layout.n_sym):
            j = int(np.clip(np.searchsorted(rs, s, side="right") - 1, 0, rs.size - 2))
            r0, r1, phi = rs[j], rs[j + 1], drift[j]
            a = (s - r0) / (r1 - r0)
            H[s] = ((1 - a) * Hf[j] + a * Hf[j + 1] * np.exp(-1j * phi)) * np.exp(1j * phi * a)
        self.last_drift = drift
        return H


def equalize_for_demapper(Y, H, noise_var, mask=None):
    """Equalized symbols plus per-symbol noise variance, ready for QAM_Demapper soft modes.

    Returns (x, nv): x = Y / H (unbiased, so 16/64-QAM decision regions stay correct) and
    nv = noise_var / |H|^2 (noise left on each tone after equalization). Pass nv as the
    demapper's noise_var array. LLRs from faded tones are then correctly small.
    Equivalent to the unbiased-MMSE / max-log LLR with per-tone SNR |H|^2 / noise_var."""
    g = np.abs(H) ** 2
    g = np.maximum(g, 1e-12)
    x, nv = Y / H, noise_var / g
    if mask is not None:
        return x[mask], nv[mask]
    return x, nv


def equalize(Y, H, noise_var=0.0, mode="mmse"):
    """Per-tone one-tap equalizer. MMSE avoids noise blow-up in deep fades."""
    if mode == "zf":
        return Y / H
    return np.conj(H) * Y / (np.abs(H) ** 2 + noise_var)



# --------------------------------------------------------------------------------------
# Construction from a FlexLink resource grid
# --------------------------------------------------------------------------------------
def layout_from_signal_types(signal_types, codes=None, n_fft=1024, cp_len=116, scs_hz=19531.25):
    """GridLayout from Resource_Grid 'signal_types' (Packet_Len, 841). `codes` maps
    Ref_P0/Ref_P1/DC/Data to the file's numeric codes (defaults 10/20/30/40)."""
    t = np.asarray(signal_types)
    if codes:
        remap = np.full(256, DATA, np.int16)
        for name, ours in (("Ref_P0", P0), ("Ref_P1", P1), ("DC", DC), ("Data", DATA)):
            remap[int(codes.get(name, ours)) & 0xFF] = ours
        t = remap[t.astype(np.int64) & 0xFF]
    return GridLayout(types=t.astype(np.int8), tones=np.arange(-420, 421),
                      n_fft=n_fft, cp_len=cp_len, scs_hz=scs_hz)


def reference_grid(layout: GridLayout, port: int, values: np.ndarray) -> np.ndarray:
    """Known pilots on the grid: `values` (one Ref_P vector) on the port's tones of every
    reference symbol, in increasing k (as Construct_Payload_Grid.py writes them)."""
    X = np.zeros(layout.types.shape, complex)
    for s in layout.ref_symbols:
        idx = layout.pilot_idx(s, port)
        if idx.size != values.size:
            raise ValueError(f"symbol {s} has {idx.size} Ref_P{port} tones but the reference "
                             f"vector has {values.size} values")
        X[s, idx] = values
    return X
