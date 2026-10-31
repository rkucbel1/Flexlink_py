#!/usr/bin/env python3
"""
pl_check.py  (Flexlink_py/HW_Development/Packet_Detector/PC)

Checks of the payload bit-true model pl_reference.py:

  tables   CRC24 against the simulator's GenerateCRC; the LDPC encoder used here (systematic,
           parity solved from H) gives code words with zero syndrome
  loop     every modulation x every code (648 / 1296 / 1944, rates 1/2 .. 5/6) x pilot patterns,
           high SNR, through a time-domain channel: every transport block must pass its CRC
  vector   a simulator test vector (pd_make_vector.py output, e.g. TestVectors/
           Waveform_Payload_DefectsModeled_Tx) through pd_reference.detect (re-arm) and
           pl_reference.payload_results: every transport block must pass its CRC
  bler     float (the window-select FIR8 + smoothing of PC/ce_study, max-log, float layered
           decoder) against bit-true, same packets, SNR sweep -> SNR at 10 % block error

    python pl_check.py tables
    python pl_check.py loop
    python pl_check.py ranges     # largest intermediate values and saturation rates
    python pl_check.py vector ../TestVectors/Waveform_Payload_DefectsModeled_Tx.json
    python pl_check.py bler QPSK 648_12 1 3 3 EVA-70 80 [start_dB|-] [bt_lo,bt_hi,bt_bo6,...]

The packet generator follows Tx/Construct_Encoded_Bitstream_Tx.py (CRC24, LDPC, interleave,
repeat, zero padding, Gold scrambling with C_init) and Construct_Payload_Grid.py (row-major data,
Ref_P0 pilots), at 20 Msps (1024 + CP 116). Received samples are quantised to 12 bit (rms
BACKOFF_DB below full scale) as the de-rotator output, then the hardware FFT (cf_reference).
"""
from __future__ import annotations

import importlib.util
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import cf_reference as C                                            # noqa: E402
import pl_reference as P                                            # noqa: E402

ROOT = next((p for p in [HERE, *HERE.parents] if (p / "flexlink_paths.py").is_file()), HERE.parents[2])
BM = Path(os.environ.get("FLEXLINK_MODULES", str(ROOT / "Flexlink_BasicModules")))
sys.path.insert(0, str(BM / "QAM_Mapper"))
from QAM_Mapper import qam_map                                       # noqa: E402

BACKOFF_DB = 12.0
NORM = {"BPSK": 1.0, "QPSK": np.sqrt(2), "16QAM": np.sqrt(10), "64QAM": np.sqrt(42)}


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    sys.modules[name] = m
    spec.loader.exec_module(m)
    return m


# --------------------------------------------------------------------------------------------
# transmitter

class Encoder:
    """Systematic 802.11n LDPC encoder: c = [u, p], H_p p = H_u u (the simulator's G = [I | A^T])."""

    def __init__(self, code: P.Code):
        self.code = code
        H = np.zeros((code.M, code.N), np.uint8)
        for r, idx in enumerate(code.layers):
            for row in range(code.Z):
                H[r * code.Z + row, idx[:, row]] = 1
        self.H = H
        A = np.concatenate([H[:, code.K:], H[:, :code.K]], 1).copy()     # [H_p | H_u]
        M = code.M
        for col in range(M):
            piv = col + np.flatnonzero(A[col:, col])[0]
            if piv != col:
                A[[col, piv]] = A[[piv, col]]
            rows = np.flatnonzero(A[:, col])
            rows = rows[rows != col]
            A[rows] ^= A[col]
        self.A = A[:, M:].astype(np.int64)                               # p = A u

    def __call__(self, u):
        u = np.asarray(u, np.int64)
        return np.concatenate([u, (self.A @ u) % 2])


_ENC: dict = {}


def encoder(t: str) -> Encoder:
    if t not in _ENC:
        _ENC[t] = Encoder(P.code_of(t))
    return _ENC[t]


def crc_bits(tb) -> np.ndarray:
    v = P.crc24(tb)
    return np.array([(v >> (23 - i)) & 1 for i in range(24)], np.int64)


def make_packet(cfg: P.PLConfig, rng):
    """(time-domain payload at 20 Msps, transport blocks (list), grid X)."""
    enc = encoder(cfg.code)
    K, N = enc.code.K, cfg.n
    nblk = P.available_blocks(cfg)
    il = P._T[f"il_{N}"]
    seq, tbs = [], []
    for _ in range(nblk):
        tb = rng.integers(0, 2, K - 24)
        tbs.append(tb)
        c = enc(np.concatenate([tb, crc_bits(tb)]))
        seq.append(np.tile(c[il], cfg.rep + 1))                    # interleave out[i] = in[il[i]]
    nbits = P.data_bits(cfg)
    seq = np.concatenate(seq + [np.zeros(nbits - nblk * N * (cfg.rep + 1), np.int64)])
    seq ^= C.gold(cfg.c_init, nbits)
    types = P.signal_types(cfg)
    X = np.zeros(types.shape, complex)
    X[types == P.DATA] = np.asarray(qam_map(seq.astype(np.uint8), cfg.mod)).ravel()
    b = P._T[f"pilot_bits_{cfg.ref_sig}"]
    if cfg.mod == "BPSK":
        pv = (1 - 2 * b).astype(complex)
    else:
        a = P.OUTER[cfg.mod]
        pv = (2 * b - 1) * (a - 1j * a) / NORM[cfg.mod]
    refs = np.arange(0, cfg.plen, cfg.ref_sym)
    cols = np.arange(cfg.ref_sig - 1, 841, cfg.ref_sig)
    for s in refs:
        X[s, cols] = pv
    F = np.zeros((cfg.plen, 1024), complex)
    F[:, (np.arange(841) - 420) % 1024] = X
    t = np.fft.ifft(F, axis=1) * np.sqrt(1024)
    wave = np.concatenate([t[:, -116:], t], axis=1).ravel()
    return wave, tbs, X


def quantise12(r, backoff_db=BACKOFF_DB, ref_rms=None):
    rms = ref_rms if ref_rms is not None else np.sqrt(np.mean(np.abs(r) ** 2))
    g = 2047 * 10 ** (-backoff_db / 20) / rms
    q = np.round(r * g)
    return np.clip(q.real, -2048, 2047).astype(np.int64), np.clip(q.imag, -2048, 2047).astype(np.int64)


def hw_grid(xr, xi, plen, d0=0):
    """Hardware FFT of each payload symbol (window at CP end - 12), symbol 0 starting at d0."""
    Yr = np.zeros((plen, 841), np.int64)
    Yi = np.zeros((plen, 841), np.int64)
    cols = (np.arange(841) - 420) % 1024
    for s in range(plen):
        a = d0 + s * 1140 + 116 - 12
        fr, fi = C.fft_fixed(xr[a:a + 1024], xi[a:a + 1024])
        Yr[s], Yi[s] = fr[cols], fi[cols]
    return Yr, Yi


def check_blocks(res: P.PLResult, tbs) -> tuple[int, int]:
    """(blocks with crc ok and correct bits, blocks)."""
    good = 0
    for (bits, ok, _), tb in zip(res.blocks, tbs):
        good += int(ok and np.array_equal(bits, tb))
    return good, len(tbs)


# --------------------------------------------------------------------------------------------
# channel

PROFILES = {
    "EPA": ([0.0, 3e-08, 7e-08, 9e-08, 1.1e-07, 1.9e-07, 4.1e-07], [0.0, -1.0, -2.0, -3.0, -8.0, -17.2, -20.8]),
    "EVA": ([0.0, 3e-08, 1.5e-07, 3.1e-07, 3.7e-07, 7.1e-07, 1.09e-06, 1.73e-06, 2.51e-06],
            [0.0, -1.5, -1.4, -3.6, -0.6, -9.1, -7.0, -12.0, -16.9]),
    "ETU": ([0.0, 5e-08, 1.2e-07, 2e-07, 2.3e-07, 5e-07, 1.6e-06, 2.3e-06, 5e-06],
            [-1.0, -1.0, -1.0, 0.0, 0.0, 0.0, -3.0, -5.0, -7.0]),
}
_DDC = None


def through_channel(wave, channel, snr_db, rng):
    """Channel ("AWGN" or "EVA-70" ...) and noise; returns (r, d) with d = first-path delay."""
    global _DDC
    pad = np.concatenate([wave, np.zeros(200)])
    if channel == "AWGN":
        r, d = pad, 0
    else:
        if _DDC is None:
            _DDC = _load(BM / "Channel_Defects" / "Delay_Doppler_Channel.py", "ddc")
        name, fd = channel.split("-")
        dl, pw = PROFILES[name]
        r, info = _DDC.delay_doppler_channel(20e6, 0, float(fd), dl, pw, pad, rng=rng, return_info=True)
        d = info["filter_delay"]
    nv = 10 ** (-snr_db / 10)
    return r + np.sqrt(nv / 2) * (rng.standard_normal(r.size) + 1j * rng.standard_normal(r.size)), d


def cfg_of(mod, code, rep, rsym, rsig, plen=37, c_init=129) -> P.PLConfig:
    n, rate = code.split("_")
    c = P.PLConfig(mod, int(rsym), int(rsig), plen, int(n), rate, int(rep), 0, c_init)
    return P.PLConfig(**{**c.__dict__, "ndb": P.available_blocks(c)})


# --------------------------------------------------------------------------------------------
# checks

def check_tables() -> bool:
    gen = _load(BM / "CRC_Gen" / "GenerateCRC.py", "gcrc")
    rng = np.random.default_rng(5)
    ok = True
    for n in (1, 24, 300, 1056):
        m = rng.integers(0, 2, n)
        ref = np.asarray(gen.GenerateCRC(m, "Gen24")).ravel()
        ok &= np.array_equal(ref, crc_bits(m))
    print(f"CRC24 vs GenerateCRC: {'ok' if ok else 'FAIL'}")
    for t in [f"{n}_{r}" for n in (648, 1296, 1944) for r in P.RATES]:
        e = encoder(t)
        u = rng.integers(0, 2, e.code.K)
        c = e(u)
        good = e.code.syndrome_ok(c[None, :])[0]
        ok &= bool(good)
        print(f"  LDPC {t}: K {e.code.K}, syndrome {'0' if good else 'NOT 0'}")
    return ok


def loop(cfg, channel="AWGN", snr=30.0, seed=1, packets=1, quiet=False):
    rng = np.random.default_rng(seed)
    good = tot = 0
    for _ in range(packets):
        wave, tbs, _ = make_packet(cfg, rng)
        r, d = through_channel(wave, channel, snr, rng)
        xr, xi = quantise12(r)
        res = P.receive_grid(*hw_grid(xr, xi, cfg.plen, d), cfg)
        g, n = check_blocks(res, tbs)
        good += g
        tot += n
    if not quiet:
        print(f"  {cfg.mod:5s} {cfg.code:7s} rep {cfg.rep:2d} {cfg.ref_sym:2d}/{cfg.ref_sig:<2d} {channel:8s} "
              f"{snr:4.0f} dB: {good}/{tot} blocks, class {res.window}{"h" if res.hi else "l"}, k {res.k}, "
              f"iterations {[b[2] for b in res.blocks][:4]}")
    return good, tot


def check_loop() -> bool:
    ok = True
    snr = {"BPSK": 15, "QPSK": 18, "16QAM": 25, "64QAM": 32}
    for mod in P.MODS:
        for code in [f"{n}_{r}" for n in (648, 1296, 1944) for r in P.RATES]:
            g, n = loop(cfg_of(mod, code, 1, 3, 3), "AWGN", snr[mod])
            ok &= g == n and n > 0
    for rsym in (1, 3, 6, 12):
        for rsig in (3, 6, 12, 24):
            g, n = loop(cfg_of("QPSK", "648_12", 1, rsym, rsig), "EPA-5", 25)
            ok &= g == n
    for mod in P.MODS:
        g, n = loop(cfg_of(mod, "1944_34", 1, 3, 3), "ETU-300", snr[mod] + 6)
        ok &= g == n
    g, n = loop(cfg_of("QPSK", "648_12", 31, 3, 3, plen=121), "EVA-70", 10)
    ok &= g == n
    print(f"loop: {'PASS' if ok else 'FAIL'}")
    return ok


def check_vector(path: Path) -> bool:
    import pd_protocol as pr
    import pd_reference as R
    meta = json.loads(path.read_text())
    dec = path.with_name(meta["files"]["decimated"])
    if not dec.is_file():
        print(f"vector: {dec.name} not found (the decimated input; the DC-blocker stream is needed)")
        return False
    iq = np.fromfile(dec, "<i2").reshape(-1, 2).astype(np.int64)
    known = set(pr.Params.__dataclass_fields__)
    p = pr.Params(**{k: v for k, v in meta["params"].items() if k in known})
    p.input_stage, p.rearm = 1, 1
    t0 = time.time()
    res = R.detect(iq, p)
    y = R.dc_blocker(R.as_iq(iq), p.dc_shift)
    out = P.payload_results(y, res.events, res.timing, res.control)
    ok = bool(out)
    for r in out:
        n_ok = sum(r.crc_ok)
        ok &= n_ok == len(r.blocks) and len(r.blocks) > 0
        print(f"  packet t_trig {r.debug['t_trig']}: {r.cfg.mod} {r.cfg.code} rep {r.cfg.rep} "
              f"{r.cfg.ref_sym}/{r.cfg.ref_sig} PacketLength {r.cfg.plen} C_init {r.cfg.c_init}: "
              f"{n_ok}/{len(r.blocks)} CRC ok, class {r.window}, iterations {[b[2] for b in r.blocks]}")
    print(f"vector {path.name}: {len(out)} packets, {'PASS' if ok else 'FAIL'} [{time.time() - t0:.0f} s]")
    return ok


# --------------------------------------------------------------------------------------------
# float vs bit-true

def float_estimate(Y, cfg: P.PLConfig, ce, cache):
    """The float estimator of the study (window-select FIR8 + smoothing), on the float grid Y
    (simulator frame: ramp removed), plus sigma^2."""
    sys.path.insert(0, str(HERE / "ce_study"))
    import pl_study as S                                           # float study code
    types = P.signal_types(cfg)
    lay = ce.layout_from_signal_types(types)
    pp = float(np.abs(complex(P.OUTER[cfg.mod], -P.OUTER[cfg.mod]) / NORM[cfg.mod]) ** 2) \
        if cfg.mod != "BPSK" else 1.0
    nv = max(float(ce.estimate_noise_var(Y, lay, 0)), 1e-9)
    Xref = np.zeros(types.shape, complex)
    b = P._T[f"pilot_bits_{cfg.ref_sig}"]
    pv = (1 - 2 * b).astype(complex) if cfg.mod == "BPSK" else \
        (2 * b - 1) * (P.OUTER[cfg.mod] - 1j * P.OUTER[cfg.mod]) / NORM[cfg.mod]
    for s in np.arange(0, cfg.plen, cfg.ref_sym):
        Xref[s, np.arange(cfg.ref_sig - 1, 841, cfg.ref_sig)] = pv
    key0 = ("lin", lay.types.tobytes(), cfg.mod)
    if key0 not in cache:
        cache[key0] = ce.AdaptiveChannelEstimator(lay, 0, fd_hz=700, pilot_power=pp, time="linear")
    est = cache[key0]
    Hls = Y[np.ix_(est.ref_syms, est.cols)] / Xref[np.ix_(est.ref_syms, est.cols)]
    _, end = ce.choose_window(Hls, lay.tones[est.cols].astype(float), lay.n_fft, nv / pp, est.limit, est.span)
    cls = next((c for c in S.CLASSES if c >= end), S.CLASSES[-1])
    nc = int(P._T[f"n_classes_{cfg.ref_sig}"])
    cls = min(cls, (12, 28, 52, 100)[nc - 1])                      # the classes the hardware has
    # the same SNR mode as the bit-true model (row 0: pilot power >= HI_RATIO x noise)
    hi = np.mean(np.abs(Y[np.ix_(est.ref_syms[:P.NR_EST], est.cols)]) ** 2) >= P.HI_RATIO * nv
    if P.MODE_FORCE is not None:
        hi = P.MODE_FORCE == "hi"
    nz = float(P._T["FIR_NOISE"][int(hi)])
    key = ("sel", cls, hi, lay.types.tobytes(), cfg.mod)
    if key not in cache:
        sm = None if (hi or cfg.ref_sym == 12) else P.SMOOTH[cfg.ref_sym]
        cache[key] = S.SmoothFIR(lay, 0, taps=8, delay_window=(-20, cls), pilot_power=pp,
                                 design_noise=nz * pp, smooth=sm)
    return cache[key].estimate(Y, Xref), nv


def float_decode(Y, H, nv, cfg: P.PLConfig, tbs, ce):
    sys.path.insert(0, str(HERE / "ldpc_study"))
    import ldpc_lib as L
    from QAM_Demapper import QAM_Demapper
    types = P.signal_types(cfg)
    x, nvs = ce.equalize_for_demapper(Y, H, nv)
    m = types == P.DATA
    llr = np.asarray(QAM_Demapper(x[m], cfg.mod, "maxlog", nvs[m]), float).ravel()
    g = C.gold(cfg.c_init, llr.size)
    llr = np.where(g == 1, -llr, llr)
    N, cw = cfg.n, cfg.n * (cfg.rep + 1)
    nblk = len(tbs)
    w = llr[:nblk * cw].reshape(nblk, cfg.rep + 1, N).sum(1)
    c = np.empty_like(w)
    c[:, P._T[f"il_{N}"]] = w
    code = L.Code(cfg.code)
    hard, _ = L.decode_layered(code, c, 10)
    good = 0
    for i, tb in enumerate(tbs):
        msg = hard[i, :code.K]
        good += int(np.array_equal(msg[:code.K - 24], tb) and np.array_equal(msg[code.K - 24:], crc_bits(tb)))
    return good


def bler(mod, code, rep, rsym, rsig, channel, n, start=None, step=1.0, seed=1):
    ce = _load(BM / "Equalizer" / "Channel_Estimator.py", "Channel_Estimator")
    cfg = cfg_of(mod, code, rep, rsym, rsig)
    cache = {}
    K = np.exp(-2j * np.pi * (np.arange(841) - 420) * -12 / 1024)        # remove the FFT_StartTime ramp
    s = start if start is not None else {"BPSK": -6, "QPSK": -3, "16QAM": 3, "64QAM": 8}[mod]
    names = ["float", "bittrue"] + list(EXTRA)
    rows = []
    while True:
        rng = np.random.default_rng([seed, int(round(s * 10)) + 1000])
        err = dict.fromkeys(names, 0)
        tot = 0
        t0 = time.time()
        for _ in range(n):
            wave, tbs, _ = make_packet(cfg, rng)
            r, d = through_channel(wave, channel, s, rng)
            # float: the study's receiver on the unquantised samples
            Yf = np.stack([np.fft.fft(r[d + q * 1140 + 104: d + q * 1140 + 1128]) / 32
                           for q in range(cfg.plen)])[:, (np.arange(841) - 420) % 1024] * K[None, :]
            H, nv = float_estimate(Yf, cfg, ce, cache)
            err["float"] += len(tbs) - float_decode(Yf, H, nv, cfg, tbs, ce)
            # bit-true, and variants: bt_lo / bt_hi (SNR mode forced), bt_bo<dB> (input backoff)
            for name in names[1:]:
                bo = float(name[5:]) if name.startswith("bt_bo") else BACKOFF_DB
                P.MODE_FORCE = {"bt_lo": "lo", "bt_hi": "hi"}.get(name)
                xr, xi = quantise12(r, bo)
                res = P.receive_grid(*hw_grid(xr, xi, cfg.plen, d), cfg)
                P.MODE_FORCE = None
                err[name] += len(tbs) - check_blocks(res, tbs)[0]
            tot += len(tbs)
        rows.append([s] + [err[k] / tot for k in names])
        print(f"  {s:+5.1f} dB: " + ", ".join(f"{k} {err[k] / tot:.3f}" for k in names)
              + f"  [{tot} blocks, {time.time() - t0:.0f} s]", flush=True)
        if all(v < 0.01 for v in rows[-1][1:]) or len(rows) >= 14 or s > 45:
            break
        s += 2 * step if min(rows[-1][1:]) > 0.9 else step
    sn = [r[0] for r in rows]
    out = {"cfg": [mod, code, str(rep), str(rsym), str(rsig), channel], "n": n,
           "snr10": {k: _snr10(sn, [r[1 + i] for r in rows]) for i, k in enumerate(names)},
           "names": names, "rows": rows}
    print(json.dumps({"cfg": out["cfg"], "snr10": {k: round(v, 2) for k, v in out["snr10"].items()}}), flush=True)
    with open(HERE / os.environ.get("PLBLER", "pl_check_bler.jsonl"), "a") as f:
        f.write(json.dumps(out) + "\n")
    return out


EXTRA: list = []          # extra bit-true variants for bler (command line: 9th argument)


def _snr10(snrs, bl):
    bl = np.maximum(np.asarray(bl, float), 1e-4)
    for i in range(len(snrs) - 1):
        if bl[i] >= 0.1 > bl[i + 1]:
            a, b = np.log10(bl[i]), np.log10(bl[i + 1])
            return float(snrs[i] + (np.log10(0.1) - a) / (b - a) * (snrs[i + 1] - snrs[i]))
    return float("nan")


def check_ranges(packets=3) -> bool:
    """Largest magnitudes of the intermediate values and how often each saturates, over
    modulations, channels, SNRs and input backoffs (word-length check of the model's widths)."""
    stats = {}

    def note(name, v, bits=None):
        v = np.abs(np.asarray(v, np.int64))
        m = int(v.max()) if v.size else 0
        st = stats.setdefault(name, [0, 0, 0, bits])
        st[0] = max(st[0], m)
        if bits is not None:
            st[1] += int((v >= (1 << (bits - 1)) - 1).sum())
        st[2] += v.size

    cases = [("QPSK", "648_12", "AWGN", 0), ("QPSK", "648_12", "ETU-300", 30), ("16QAM", "648_12", "EVA-70", 10),
             ("64QAM", "1944_34", "EVA-70", 20), ("64QAM", "648_12", "AWGN", 40), ("BPSK", "648_12", "EPA-5", -3),
             ("64QAM", "1944_34", "ETU-300", 40)]
    for mod, code, ch, snr in cases:
        for bo in (6.0, 12.0, 20.0):
            cfg = cfg_of(mod, code, 1, 3, 3)
            rng = np.random.default_rng(11)
            for _ in range(packets):
                wave, tbs, _ = make_packet(cfg, rng)
                r, d = through_channel(wave, ch, snr, rng)
                xr, xi = quantise12(r, bo)
                Yr, Yi = hw_grid(xr, xi, cfg.plen, d)
                res = P.PLResult(cfg)
                Ypr, Ypi, Hr, Hi, sn, n0 = P.channel(Yr, Yi, cfg, res)
                note("Y (FFT out)", np.concatenate([Yr.ravel(), Yi.ravel()]), 18)
                note("Y'", np.concatenate([Ypr.ravel(), Ypi.ravel()]), P.Y_BITS)
                note("Hf", np.concatenate([a.ravel() for a in res.debug["hf"]]), P.H_BITS)
                note("Sm", np.concatenate([a.ravel() for a in res.debug["sm"]]), P.H_BITS)
                note("H", np.concatenate([Hr.ravel(), Hi.ravel()]), P.H_BITS)
                m = P.signal_types(cfg) == P.DATA
                zr = Ypr[m] * Hr[m] + Ypi[m] * Hi[m]
                e = Hr[m] ** 2 + Hi[m] ** 2
                note("z", zr)
                note("e", e)
                N = P.llr_numerators(zr, Ypi[m] * Hr[m] - Ypr[m] * Hi[m], e, mod).ravel()
                note("N", N)
                mm, sh = P.llr_scale(sn, n0, cfg)
                lf = P._rnd(N * mm, sh)
                note("Lf (before sat)", lf, P.LF_BITS)
                note("N * m", N * mm)
                note("sumN", [sn])
                note("k (packet shift)", [res.k])
                rr = P.decode_soft(P._satm(lf, P.LF_BITS), cfg, res)
                note("Lq", rr.debug["llr_q"], P.LLR_BITS)
    print(f"{'value':18s} {'max |v|':>16s} {'bits':>5s} {'width':>6s} {'saturated':>12s}")
    for k, (m, nsat, n, bits) in stats.items():
        print(f"{k:18s} {m:16d} {m.bit_length() + 1:5d} {str(bits or '-'):>6s} "
              f"{(f'{nsat / max(n, 1):.2e}' if bits else '-'):>12s}")
    return True


def main(argv) -> int:
    if not argv or argv[0] == "tables":
        return 0 if check_tables() else 1
    if argv[0] == "ranges":
        return 0 if check_ranges() else 1
    if argv[0] == "loop":
        return 0 if check_loop() else 1
    if argv[0] == "vector":
        return 0 if check_vector(Path(argv[1])) else 1
    if argv[0] == "bler":
        a = argv[1:]
        if len(a) > 8:
            EXTRA[:] = a[8].split(",")
        bler(a[0], a[1], int(a[2]), int(a[3]), int(a[4]), a[5], int(a[6]),
             float(a[7]) if len(a) > 7 and a[7] != "-" else None)
        return 0
    print(__doc__)
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
