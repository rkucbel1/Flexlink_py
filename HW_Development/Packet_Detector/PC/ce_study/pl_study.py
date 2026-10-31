"""Payload channel-estimate study (float): the simulator's estimators (LMMSE, LMMSE_LinearTime,
Linear, FIR8 = fixed 8-tap banded LMMSE + drift-corrected linear time) and a genie, on the
payload grid of every pilot pattern, through the time-domain channel model at 20 Msps.

Tx: all-zero LDPC code words, repeated (Repetition + 1 copies, as Rate_Match), interleaved
(Interleave_LDPC tables), scrambled by random bits (statistically the Gold code), QAM mapped,
placed row-major on the data REs; pilots antipodal corner points (Generate_Reference_Signal_
Vectors). OFDM 1024 + CP 116. Rx: perfect timing on the first path, FFT_StartTime -12, ramp
removed (the simulator's frame), estimator, x = Y / H, nv = sigma^2 / |H|^2 (sigma^2 from the
empty comb and nulls, estimate_noise_var), max-log LLRs, copies summed, layered min-sum 10
iterations (the frozen hardware decoder, float). Block error = code word not all zero."""
import sys, time, json, importlib.util
from pathlib import Path
import numpy as np

import os
HERE = Path(__file__).resolve().parent
S = HERE                                                          # results/ go here
BM = Path(os.environ.get("FLEXLINK_MODULES", str(HERE.parents[3] / "Flexlink_BasicModules")))
sys.path.insert(0, str(HERE.parent / "ldpc_study"))
import ldpc_lib as L


class H5:
    @staticmethod
    def read(path, name):
        import h5py
        with h5py.File(str(path), "r") as f:
            return f[name][()]


def load(p, name=None):
    spec = importlib.util.spec_from_file_location(name or p.stem, p)
    m = importlib.util.module_from_spec(spec); sys.modules[name or p.stem] = m; spec.loader.exec_module(m); return m


ce = load(BM / "Equalizer/Channel_Estimator.py")
rg = load(BM / "Construct_Grid/Construct_Resource_Grid.py")
ddc = load(BM / "Channel_Defects/Delay_Doppler_Channel.py")

NFFT, CP, SYM, FST = 1024, 116, 1140, -12
K = np.arange(841) - 420
BINS = (K % NFFT)
ILV = {n: np.asarray(H5.read(BM / f"Interleaver/Interleaver_{n}.h5", "InterleaverIndex")).ravel().astype(int)
       for n in (648, 1296, 1944)}
CORNER = {"BPSK": 1.0, "QPSK": (1 - 1j) / np.sqrt(2), "16QAM": (3 - 3j) / np.sqrt(10), "64QAM": (7 - 7j) / np.sqrt(42)}
PROFILES = {
    "EPA": ([0.0, 3e-08, 7e-08, 9e-08, 1.1e-07, 1.9e-07, 4.1e-07], [0.0, -1.0, -2.0, -3.0, -8.0, -17.2, -20.8]),
    "EVA": ([0.0, 3e-08, 1.5e-07, 3.1e-07, 3.7e-07, 7.1e-07, 1.09e-06, 1.73e-06, 2.51e-06],
            [0.0, -1.5, -1.4, -3.6, -0.6, -9.1, -7.0, -12.0, -16.9]),
    "ETU": ([0.0, 5e-08, 1.2e-07, 2e-07, 2.3e-07, 5e-07, 1.6e-06, 2.3e-06, 5e-06],
            [-1.0, -1.0, -1.0, 0.0, 0.0, 0.0, -3.0, -5.0, -7.0]),
}


class Setup:
    def __init__(self, mod, code, rep, ref_sym, ref_sig, plen=37):
        self.mod, self.rep = mod, rep
        self.code = L.Code(code)
        self.types = rg.Resource_Grid(plen, ref_sym, ref_sig).signal_types
        self.layout = ce.layout_from_signal_types(self.types)
        self.data = self.types == 40
        self.pmask = self.types == 10
        self.nb = L.BITS[mod]
        nbits = int(self.data.sum()) * self.nb
        self.blk = self.code.N * (rep + 1)
        self.ncw = nbits // self.blk
        self.nbits = nbits
        self.plen = plen

    def tx(self, rng):
        scr = rng.integers(0, 2, self.nbits)
        sym = np.asarray(L.qam_map(scr, self.mod)).ravel()
        X = np.zeros(self.types.shape, complex)
        X[self.data] = sym
        pil = rng.integers(0, 2, int(self.pmask.sum()))
        X[self.pmask] = np.where(pil == 1, CORNER[self.mod], -CORNER[self.mod])
        Xref = np.where(self.pmask, X, 0)
        F = np.zeros((self.plen, NFFT), complex)
        F[:, BINS] = X
        t = np.fft.ifft(F, axis=1) * np.sqrt(NFFT)
        wave = np.concatenate([t[:, -CP:], t], axis=1).ravel()
        return wave, X, Xref, scr

    def rx(self, r, delay):
        Y = np.zeros((self.plen, 841), complex)
        for s in range(self.plen):
            a = delay + s * SYM + CP + FST
            Y[s] = np.fft.fft(r[a:a + NFFT])[BINS] / np.sqrt(NFFT)
        return Y * np.exp(-2j * np.pi * K * FST / NFFT)[None, :]

    def decode(self, Y, Hh, nv, scr):
        x, nvs = ce.equalize_for_demapper(Y, Hh, nv)
        llr = np.asarray(L.QAM_Demapper(x[self.data], self.mod, "maxlog", nvs[self.data]), float).ravel()
        llr = np.where(scr == 1, -llr, llr)                         # LLR of the coded bit (0 sent)
        N = self.code.N
        w = llr[:self.ncw * self.blk].reshape(self.ncw, self.rep + 1, N).sum(1)
        c = np.empty_like(w); c[:, ILV[N]] = w                      # de-interleave
        bits, _ = L.decode_layered(self.code, c, 10)
        return bits.any(1)


def estimators(st, Y, Xref, Hgen):
    lay = st.layout
    nv = max(float(ce.estimate_noise_var(Y, lay, 0)), 1e-6)
    pp = float(np.mean(np.abs(CORNER[st.mod]) ** 2))
    out = {"genie": (Hgen, nv)}
    for name, tm in (("LMMSE", "wiener"), ("LMMSE_lin", "linear")):
        key = (name, st.mod, lay.types.tobytes())
        if key not in FIR:
            FIR[key] = ce.AdaptiveChannelEstimator(lay, 0, fd_hz=700, pilot_power=pp, time=tm)
        out[name] = (FIR[key].estimate(Y, Xref, nv), nv)
    for name, taps, win in fir_variants(st):
        tsm = TSMOOTH.get(name.rstrip("N").split("T")[-1] if "T" in name else "", None)
        dn = 0.5
        if name.endswith("N"):                                  # design noise from a ROM class
            dn = next((c for c in NCLASS if c >= nv / pp), NCLASS[-1]) * pp
        key = (name, dn, st.mod, lay.types.tobytes())
        if key not in FIR:
            FIR[key] = SmoothFIR(lay, 0, taps=taps, delay_window=win, design_noise=dn,
                                 pilot_power=pp, smooth=tsm)
        out[name] = (FIR[key].estimate(Y, Xref), nv)
    if SELECT:
        # window classes chosen per packet from the measured delay profile (as the adaptive
        # LMMSE), each a fixed banded FIR set (a ROM per class in hardware)
        est = FIR[("LMMSE_lin", st.mod, lay.types.tobytes())]
        Hls = Y[np.ix_(est.ref_syms, est.cols)] / Xref[np.ix_(est.ref_syms, est.cols)]
        start, end = ce.choose_window(Hls, lay.tones[est.cols].astype(float), lay.n_fft, nv / pp, est.limit, est.span)
        for taps in SELECT:
            cls = next((c for c in CLASSES if c >= end), CLASSES[-1])
            for t in SELT:
                key = ("SEL", taps, cls, t, st.mod, lay.types.tobytes())
                if key not in FIR:
                    FIR[key] = SmoothFIR(lay, 0, taps=taps, delay_window=(-20, cls), pilot_power=pp,
                                         smooth=TSMOOTH.get(t))
                out[f"SEL{taps}" + (f"T{t}" if t else "")] = (FIR[key].estimate(Y, Xref), nv)
    return out


class SmoothFIR(ce.FixedFIRChannelEstimator):
    """FIR8 plus an optional fixed smoothing across reference symbols before the linear time
    interpolation: each frequency-filtered reference row is replaced by the weighted sum of its
    neighbours, phase-aligned by the measured common drift (the same drift the interpolation
    uses). Weights are truncated / renormalised at the packet edges. Hardware: a short FIR on
    the stored reference rows, no adaptation."""

    def __init__(self, *a, smooth=None, **k):
        super().__init__(*a, **k)
        self.smooth = None if smooth is None else np.asarray(smooth, float)

    def estimate(self, Y, Xref, noise_var=None):
        if self.smooth is None or self.ref_syms.size < 3:
            return super().estimate(Y, Xref, noise_var)
        Hls = Y[np.ix_(self.ref_syms, self.cols)] / Xref[np.ix_(self.ref_syms, self.cols)]
        Hf = Hls @ self.W.T
        nr = Hf.shape[0]
        drift = np.array([np.angle(np.sum(Hls[j + 1] * np.conj(Hls[j]))) for j in range(nr - 1)])
        cum = np.concatenate([[0.0], np.cumsum(drift)])          # phase of row j vs row 0
        A = Hf * np.exp(-1j * cum)[:, None]                      # aligned rows
        h = self.smooth; L2 = h.size // 2
        S = np.zeros_like(A); wsum = np.zeros(nr)
        for k, w in enumerate(h):
            o = k - L2
            lo, hi = max(0, -o), min(nr, nr - o)
            S[lo:hi] += w * A[lo + o:hi + o]; wsum[lo:hi] += w
        Hs = (S / wsum[:, None]) * np.exp(1j * cum)[:, None]
        rs = self.ref_syms
        H = np.zeros((self.layout.n_sym, Hf.shape[1]), complex)
        for s in range(self.layout.n_sym):
            j = int(np.clip(np.searchsorted(rs, s, side="right") - 1, 0, rs.size - 2))
            r0, r1, phi = rs[j], rs[j + 1], drift[j]
            a = (s - r0) / (r1 - r0)
            H[s] = ((1 - a) * Hs[j] + a * Hs[j + 1] * np.exp(-1j * phi)) * np.exp(1j * phi * a)
        return H


TSMOOTH = {"3": (1, 2, 1), "5": (1, 2, 2, 2, 1), "9": (1,) * 9}
NCLASS = (0.05, 0.15, 0.5, 1.5)                # design noise classes (x pilot power)
SELT = [""]                                    # smoothing variants of SEL ("" = none)
SELECT = []                                   # tap counts of the window-select FIR variants
CLASSES = (12, 28, 52, 100)


def fir_variants(st):
    sp = int(np.median(np.diff(st.layout.pilot_idx(st.layout.ref_symbols[0], 0))))
    long_end = int(min(100, NFFT // sp - 21))
    allv = [("FIR8", 8, (-20, 12)), ("FIR16", 16, (-20, 12)), ("FIR8L", 8, (-20, long_end)),
            ("FIR16L", 16, (-20, long_end)), ("FIR32L", 32, (-20, long_end)),
            ("FIR8N", 8, (-20, 12)), ("FIR8T3", 8, (-20, 12)), ("FIR8T5", 8, (-20, 12)),
            ("FIR8T9", 8, (-20, 12)), ("FIR8T3N", 8, (-20, 12)), ("FIR8T5N", 8, (-20, 12))]
    return [v for v in allv if not VARIANTS or v[0] in VARIANTS]


VARIANTS = []


FIR = {}


def run(st, snrs, n, channel, seed=1):
    rng = np.random.default_rng(seed)
    res = {}
    for snr in snrs:
        t0 = time.time()
        err = {}; tot = 0
        for _ in range(n):
            wave, X, Xref, scr = st.tx(rng)
            pad = np.concatenate([wave, np.zeros(200)])
            if channel is None:
                r, d = pad, 0
                Hgen = np.ones(X.shape, complex)
            else:
                dl, pw = PROFILES[channel[0]]
                r, info = ddc.delay_doppler_channel(20e6, 0, channel[1], dl, pw, pad, rng=rng, return_info=True)
                d = info["filter_delay"]
                Hgen = genie(st, info, d)
            nv0 = 10 ** (-snr / 10)
            r = r + np.sqrt(nv0 / 2) * (rng.standard_normal(r.size) + 1j * rng.standard_normal(r.size))
            Y = st.rx(r, d)
            for m, (Hh, nv) in estimators(st, Y, Xref, Hgen).items():
                e = st.decode(Y, Hh, nv, scr)
                err[m] = err.get(m, 0) + int(e.sum())
            tot += st.ncw
        res[snr] = {m: v / tot for m, v in err.items()}
        print(f"  {snr:+6.1f} dB: " + ", ".join(f"{m} {v / tot:.3f}" for m, v in err.items())
              + f"  [{time.time() - t0:.0f} s, {tot} cw]", flush=True)
    return res


def genie(st, info, d):
    """Channel per symbol and tone at the middle of each FFT window (frequency response of the
    time-varying taps), in the simulator's frame (ramp removed)."""
    g, h, taps = info["sinc_filters"], info["fading"], info["filter_taps"]
    Hs = np.zeros((st.plen, 841), complex)
    for s in range(st.plen):
        n = d + s * SYM + CP + FST + NFFT // 2
        imp = (h[:, n][:, None] * g).sum(0)                     # taps at time n, delay = taps - d
        tau = taps - d
        Hs[s] = (imp[None, :] * np.exp(-2j * np.pi * np.outer(K, tau) / NFFT)).sum(1)
    return Hs


def snr10(snrs, bl):
    bl = np.maximum(np.asarray(bl, float), 1e-4)
    for i in range(len(snrs) - 1):
        if bl[i] >= 0.1 > bl[i + 1]:
            a, b = np.log10(bl[i]), np.log10(bl[i + 1])
            return float(snrs[i] + (np.log10(0.1) - a) / (b - a) * (snrs[i + 1] - snrs[i]))
    return float("nan")


def sweep(st, channel, n, start, step=0.5, seed=1):
    """Coarse 2 dB steps until the genie is below 50 %, then `step` until every estimator is
    below 1 % (at most 14 points)."""
    s = start
    while True:
        r = run(st, [s], max(4, n // 4), channel, seed=seed + 999)[s]
        if r["genie"] < 0.5 or s > 40: break
        s += 2.0
    snrs = list(np.arange(s - 1.5, s + 12.01, step))
    table = {}
    out_s = []
    for i, x in enumerate(snrs):
        r = run(st, [float(x)], n, channel, seed=seed + i)[float(x)]
        out_s.append(float(x))
        for m, v in r.items(): table.setdefault(m, []).append(v)
        if all(v < 0.01 for v in r.values()) or i >= 13: break
    return {m: snr10(out_s, v) for m, v in table.items()}, out_s, table


if __name__ == "__main__" and len(sys.argv) > 1 and sys.argv[1] == "sweep":
    a = sys.argv[2:]
    mod, code, rep, rsym, rsig = a[0], a[1], int(a[2]), int(a[3]), int(a[4])
    ch = None if a[5] == "AWGN" else (a[5].split("-")[0], float(a[5].split("-")[1]))
    n, start = int(a[6]), float(a[7])
    if len(a) > 8 and a[8] != "-": SELECT[:] = [int(x) for x in a[8].split(",")]
    if len(a) > 9: VARIANTS[:] = a[9].split(",")
    if len(a) > 11: SELT[:] = ["" if x == "0" else x for x in a[11].split(",")]
    st = Setup(mod, code, rep, rsym, rsig)
    t0 = time.time()
    r, ss, tab = sweep(st, ch, n, start, step=float(a[10]) if len(a) > 10 else 0.5)
    line = {"cfg": a[:6], "snr10": r, "snrs": ss, "bler": tab, "s": round(time.time() - t0)}
    print(json.dumps({"cfg": a[:6], "snr10": {k: round(v, 2) for k, v in r.items()}}), flush=True)
    import os
    with open(S / "results" / os.environ.get("PLOUT", "pl_sweep.jsonl"), "a") as f:
        f.write(json.dumps(line) + "\n")
    sys.exit(0)

if __name__ == "__main__":
    a = sys.argv[1:]
    mod, code, rep, rsym, rsig = a[0], a[1], int(a[2]), int(a[3]), int(a[4])
    ch = None if a[5] == "AWGN" else (a[5].split("-")[0], float(a[5].split("-")[1]))
    snrs = [float(x) for x in a[6].split(",")]
    n = int(a[7])
    st = Setup(mod, code, rep, rsym, rsig)
    print(f"{mod} {code} rep {rep} RefSymbolSpacing {rsym} RefSignalSpacing {rsig} {a[5]}: {st.ncw} code words per packet", flush=True)
    r = run(st, snrs, n, ch)
    with open(S / "results" / "pl_results.jsonl", "a") as f:
        f.write(json.dumps({"cfg": a[:6], "res": r}) + "\n")
