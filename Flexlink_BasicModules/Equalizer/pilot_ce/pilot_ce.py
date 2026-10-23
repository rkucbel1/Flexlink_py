"""
pilot_ce.py - reference-signal generation, channel estimation and equalization
for the comb-pilot OFDM grid described in Grid_Resource_Tx.h5.

Grid conventions (Grid_Resource_Tx.h5 and ResourceGrid_Flexlink.ods):
    signal_types codes: P0 = 10, P1 = 20, DC/null = 30, Data = 40
    tones -420 .. +420 (841 used tones), k = tone + 420
    ReferenceSymbolProfile S in {3, 6, 12, 24}: P0 every S tones ending at tone +419,
        P1 one tone below each P0, nulls at the band edges and around DC
    PacketProfile P in {1, 3, 6, 12}: a reference symbol every P symbols, starting at 0
    one Ref_P0 vector (Ref_P0_<S>_<bits>.h5) is reused in every reference symbol

Pipeline at the receiver
    layout = flexlink_layout(ref_spacing=S, packet_profile=P)
    ref    = load_reference_vector(folder, port=0, spacing=S, bits_per_symbol=Y)
    Xref   = reference_grid(layout, 0, values=ref)        # known pilots on the grid
    nv     = estimate_noise_var(Y, layout, 0)              # sigma^2 from empty comb + nulls
    est    = ChannelEstimator(layout, 0, noise_var=nv, pilot_power=mean(|ref|^2))
    H      = est.estimate(Y, Xref)
    Xhat   = equalize(Y, H, nv)
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

P0, P1, DC, DATA = 10, 20, 30, 40
PORT_CODE = {0: P0, 1: P1}


# --------------------------------------------------------------------------------------
# Layout
# --------------------------------------------------------------------------------------
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


def load_layout(path, **kw) -> GridLayout:
    """Read signal_types / Tone_i from the Tx grid file.
    Uses h5py when available, otherwise the bundled minimal reader."""
    try:
        import h5py
        with h5py.File(path, "r") as f:
            types = f["signal_types"][()]
            tones = f["Tone_i"][()]
            meta = {k: f.attrs[k] for k in f.attrs}
    except ImportError:
        from minih5 import MiniH5
        f = MiniH5(path)
        c = f.children(f.root_ohdr)
        types, tones = f.dataset(c["signal_types"]), f.dataset(c["Tone_i"])
        meta = {k: v.item() for k, v in f.attrs(f.root_ohdr).items()}
    return GridLayout(types=np.asarray(types, np.int8), tones=np.asarray(tones), meta=meta, **kw)


def make_layout(spacing=3, n_sym=72, ref_period=3, n_half=420,
                data_dc=(-6, 8), ref_dc_halfwidth=6, p1_offset=1, p0_offset=2, **kw) -> GridLayout:
    """Synthesize a layout for an arbitrary pilot spacing S.
    With the defaults (S=3) this reproduces the sample file exactly (checked in tests).
    For S != 3 the offsets/null rules are an extrapolation of the S=3 pattern -
    replace with your spec's rule if it differs."""
    tones = np.arange(-n_half, n_half + 1)
    types = np.full((n_sym, tones.size), DATA, np.int8)
    for s in range(n_sym):
        if s % ref_period == 0:
            row = types[s]
            row[np.mod(tones, spacing) == p1_offset % spacing] = P1
            row[np.mod(tones, spacing) == p0_offset % spacing] = P0
            near_dc = (np.abs(tones) <= ref_dc_halfwidth) & (row == DATA)
            row[near_dc] = DC
            row[np.abs(tones) == n_half] = DC
        else:
            types[s, (tones >= data_dc[0]) & (tones <= data_dc[1])] = DC
    return GridLayout(types=types, tones=tones, **kw)


_LAYOUT_FILE = __import__("pathlib").Path(__file__).with_name("flexlink_ref_layouts.npz")


VALID_PACKET_LENS = (37, 49, 73, 97, 121)      # as Construct_Resource_Grid.py


def flexlink_layout(ref_spacing=3, packet_profile=3, n_sym=73, data_dc=(-6, 8), ref_at_end=False,
                    **kw) -> GridLayout:
    """Grid layout for ReferenceSymbolProfile `ref_spacing` (3/6/12/24), PacketProfile
    `packet_profile` (reference symbol every 1/3/6/12 symbols, starting at symbol 0) and
    packet length `n_sym` (37/49/73/97/121).

    The FlexLink lengths are a multiple of 12 plus 1, so the last symbol is always a
    reference symbol; this matches Construct_Resource_Grid.py exactly (verified for
    flexlink_layout(3, 3, 73)). Reference-symbol rows come from ResourceGrid_Flexlink.ods
    (bundled as flexlink_ref_layouts.npz); data symbols use the DC null -6..+8.

    ref_at_end: only for non-FlexLink lengths (older 36/48/72/96/120 studies); forces the
    last symbol to be a reference symbol. Not needed for 37/49/73/97/121."""
    lay = np.load(_LAYOUT_FILE)
    tones, ref_row = lay["tones"], lay[f"S{ref_spacing}"]
    types = np.full((n_sym, tones.size), DATA, np.int8)
    types[:, (tones >= data_dc[0]) & (tones <= data_dc[1])] = DC
    types[::packet_profile] = ref_row
    if ref_at_end:                     # optional extra reference symbol on the last symbol
        types[-1] = ref_row
    return GridLayout(types=types, tones=tones, meta={"ReferenceSymbolProfile": ref_spacing,
                                                      "PacketProfile": packet_profile,
                                                      "n_sym": n_sym, "ref_at_end": ref_at_end}, **kw)


def _read_h5(path):
    try:
        import h5py
        with h5py.File(path, "r") as f:
            return {k: f[k][()] for k in f}, {k: f.attrs[k] for k in f.attrs}
    except ImportError:
        from minih5 import MiniH5
        f = MiniH5(str(path))
        c = f.children(f.root_ohdr)
        return {k: f.dataset(v) for k, v in c.items()}, f.attrs(f.root_ohdr)


def load_reference_vector(folder, port=0, spacing=3, bits_per_symbol=2, layout=None):
    """Read Ref_P<port>_<spacing>_<bits>.h5 (from Generate_Reference_Signal_Vectors.py).
    If `layout` is given, checks the vector length against the number of pilot tones."""
    path = __import__("pathlib").Path(folder) / f"Ref_P{port}_{spacing}_{bits_per_symbol}.h5"
    data, attrs = _read_h5(path)
    vec = np.asarray(data["data"]).astype(complex).ravel()
    if layout is not None:
        n = layout.pilot_idx(layout.ref_symbols[0], port).size
        if n != vec.size:
            raise ValueError(f"{path.name}: {vec.size} values but layout has {n} P{port} tones")
    return vec


# --------------------------------------------------------------------------------------
# Known reference signals
# --------------------------------------------------------------------------------------
def gold_sequence(c_init: int, length: int, nc: int = 1600) -> np.ndarray:
    """Length-31 Gold sequence (3GPP 36.211 sec. 7.2 style). Placeholder pilot source."""
    n = length + nc
    x1 = np.zeros(n + 31, np.uint8)
    x2 = np.zeros(n + 31, np.uint8)
    x1[0] = 1
    x2[:31] = [(c_init >> i) & 1 for i in range(31)]
    for i in range(n):
        x1[i + 31] = x1[i + 3] ^ x1[i]
        x2[i + 31] = x2[i + 3] ^ x2[i + 2] ^ x2[i + 1] ^ x2[i]
    return (x1[nc:nc + length] ^ x2[nc:nc + length]).astype(np.uint8)


def pilot_sequence(n_pilots: int, sym: int, port: int, cell_id: int = 0) -> np.ndarray:
    """Known pilot values for one reference symbol / port, unit power QPSK.

    >>> PLACEHOLDER <<<  The sample .h5 has no IQ values, so this is an LTE-like
    Gold-sequence QPSK. Replace the body with your protocol's actual sequence
    (or read it from a Tx grid that contains values) - nothing else needs to change."""
    c_init = ((sym + 1) * (2 * cell_id + 1) << 10) + 2 * cell_id + port + 1
    c = gold_sequence(c_init % (1 << 31), 2 * n_pilots)
    return ((1 - 2.0 * c[0::2]) + 1j * (1 - 2.0 * c[1::2])) / np.sqrt(2)


def reference_grid(layout: GridLayout, port: int, values: np.ndarray | None = None,
                   cell_id: int = 0, tx_grid: np.ndarray | None = None) -> np.ndarray:
    """Known-pilot grid (n_sym x n_tone), zeros everywhere except the active comb.

    values  : the Ref_P vector (e.g. from load_reference_vector), placed on the pilot
              tones in ascending-tone order in every reference symbol.
    tx_grid : alternatively a Tx resource grid with real IQ values to copy pilots from.
    Neither : placeholder Gold-sequence QPSK (simulation only)."""
    X = np.zeros(layout.types.shape, complex)
    for s in layout.ref_symbols:
        idx = layout.pilot_idx(s, port)
        if values is not None:
            X[s, idx] = values
        elif tx_grid is not None:
            X[s, idx] = tx_grid[s, idx]
        else:
            X[s, idx] = pilot_sequence(idx.size, s, port, cell_id)
    return X


def reference_vector(layout: GridLayout, port: int, **kw):
    """The 'vector of known signals' per reference symbol: {sym: (tone_cols, values)}."""
    X = reference_grid(layout, port, **kw)
    return {int(s): (layout.pilot_idx(s, port), X[s, layout.pilot_idx(s, port)])
            for s in layout.ref_symbols}


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


def ls_pilots(Y, Xref, layout, port):
    """Least-squares estimates at pilot positions: {sym: (cols, H_ls)}."""
    out = {}
    for s in layout.ref_symbols:
        idx = layout.pilot_idx(s, port)
        out[int(s)] = (idx, Y[s, idx] / Xref[s, idx])
    return out


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
    from scipy.special import j0
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


class AdaptiveChannelEstimator:
    """LMMSE (freq) + Wiener (time) whose delay window follows the measured channel.

    For every packet: estimate the delay profile from the pilots, pick the window end from
    WINDOW_ENDS (capped at min(CP, 0.9*N_FFT/S)), and use a cached ChannelEstimator for
    (window, SNR design point). Wide windows suit long channels (ETU); short windows
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
        self.limit = min(layout.cp_len, int(0.9 * layout.n_fft / S))
        self._cache = {}
        self.last_window = None

    def _get(self, end, snr_db, fd):
        key = (end, snr_db, fd)
        if key not in self._cache:
            self._cache[key] = ChannelEstimator(self.layout, self.port, self.freq, self.time,
                                                noise_var=10 ** (-snr_db / 10), fd_hz=fd,
                                                delay_window=(-4, end), pilot_power=self.pp)
        return self._cache[key]

    def estimate(self, Y, Xref, noise_var):
        Hls = Y[np.ix_(self.ref_syms, self.cols)] / Xref[np.ix_(self.ref_syms, self.cols)]
        end = choose_window_end(Hls, self.layout.tones[self.cols].astype(float), self.layout.n_fft,
                                noise_var / self.pp, self.limit)
        snr = self.snr_grid[np.argmin(np.abs(self.snr_grid + 10 * np.log10(noise_var)))]
        self.last_window = (-4, end)
        return self._get(end, snr, self.fd_hz).estimate(Y, Xref)


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
