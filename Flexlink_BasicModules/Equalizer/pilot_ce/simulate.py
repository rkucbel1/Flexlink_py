"""
Monte-Carlo comparison of channel estimators on the sample grid (ETU and 10 ns rms channels).
Frequency-domain simulation: the channel is constant within each OFDM symbol (no ICI),
time-varying across symbols with Jakes fading per tap.

    python simulate.py [path/to/ReferenceSignals]
"""
import sys
import time

import numpy as np

import pilot_ce as pc

ETU_DELAYS_NS = np.array([0, 50, 120, 200, 230, 500, 1600, 2300, 5000])
ETU_PWR_DB = np.array([-1, -1, -1, 0, 0, 0, -3, -5, -7])


# 10 ns rms delay spread: exponential profile sampled every 5 ns (essentially flat over 16.4 MHz)
SHORT_DELAYS_NS = np.arange(0, 60, 5.0)
SHORT_PWR_DB = 10 * np.log10(np.exp(-SHORT_DELAYS_NS / 10.0))
PROFILES = {"ETU": (ETU_DELAYS_NS, ETU_PWR_DB), "10 ns rms": (SHORT_DELAYS_NS, SHORT_PWR_DB)}


def etu_channel(layout, fd_hz, rng, n_sin=16, profile="ETU"):
    """H[n_sym, n_tone] for one packet: sum-of-sinusoids Rayleigh fading on each tap."""
    delays_ns, pwr_db = PROFILES[profile]
    p = 10 ** (pwr_db / 10)
    p /= p.sum()
    t = np.arange(layout.n_sym) * layout.t_sym
    f = layout.tones * layout.scs_hz
    H = np.zeros((layout.n_sym, f.size), complex)
    for tau, pw in zip(delays_ns * 1e-9, p):
        alpha = (2 * np.pi * (np.arange(n_sin) + rng.random(n_sin)) / n_sin)   # arrival angles
        phi = rng.uniform(0, 2 * np.pi, n_sin)
        a = np.exp(1j * (2 * np.pi * fd_hz * np.outer(t, np.cos(alpha)) + phi)).sum(1) / np.sqrt(n_sin)
        H += np.sqrt(pw) * np.outer(a, np.exp(-2j * np.pi * f * tau))
    return H


QAM16 = np.array([complex(i, q) for i in (-3, -1, 1, 3) for q in (-3, -1, 1, 3)]) / np.sqrt(10)


def qam16_bits(sym):
    """Hard decision -> 4 bits per symbol (index into QAM16 used as label)."""
    idx = np.argmin(np.abs(sym[:, None] - QAM16[None, :]), axis=1)
    return idx


def run(layout, port, snrs_db, cases, ref_values=None, n_pkt=40, seed=1):
    """cases: list of (profile, doppler_hz)."""
    rng = np.random.default_rng(seed)
    Xref = pc.reference_grid(layout, port, values=ref_values)
    ppow = float(np.mean(np.abs(Xref[Xref != 0]) ** 2))
    data_mask = layout.types == pc.DATA
    snr_design = np.arange(0, 41, 5)                 # LMMSE design points (5 dB grid)

    def make_methods():
        m = {"linear (baseline)": lambda nv: (("linear", "linear"), {}),
             "LMMSE-f + linear-t": lambda nv: (("lmmse", "linear"), {"noise_var": nv}),
             "LMMSE-f + Wiener-t": lambda nv: (("lmmse", "wiener"), {"noise_var": nv, "fd_hz": 300.0})}
        return m

    methods = make_methods()
    adaptive = pc.AdaptiveChannelEstimator(layout, port, fd_hz=300.0, pilot_power=ppow)
    cache = {}

    def get_est(name, nv_design):
        (fm, tm), kw = methods[name](nv_design)
        key = (name, nv_design if "noise_var" in kw else None)
        if key not in cache:
            cache[key] = pc.ChannelEstimator(layout, port, fm, tm, pilot_power=ppow, **kw)
        return cache[key]

    res = {}
    for prof, fd in cases:
        for snr in snrs_db:
            nv = 10 ** (-snr / 10)
            acc = {k: [0.0, 0, 0] for k in list(methods) + ["Adaptive LMMSE + Wiener", "perfect CSI"]}
            nv_err = []
            for _ in range(n_pkt):
                H = etu_channel(layout, fd, rng, profile=prof)
                lab = rng.integers(0, 16, data_mask.sum())
                X = Xref.copy()
                X[data_mask] = QAM16[lab]
                Y = H * X + np.sqrt(nv / 2) * (rng.standard_normal(H.shape) + 1j * rng.standard_normal(H.shape))
                Y[layout.types == pc.DC] = (np.sqrt(nv / 2) * (rng.standard_normal(H.shape)
                                            + 1j * rng.standard_normal(H.shape)))[layout.types == pc.DC]

                p_hat = pc.detect_port(Y, layout)
                assert p_hat == port
                nv_hat = pc.estimate_noise_var(Y, layout, p_hat)
                nv_err.append(10 * np.log10(nv_hat / nv))
                # choose the nearest LMMSE design SNR from the estimated noise
                snr_hat = -10 * np.log10(nv_hat)
                nv_design = 10 ** (-snr_design[np.argmin(np.abs(snr_design - snr_hat))] / 10)

                ests = {k: get_est(k, nv_design).estimate(Y, Xref) for k in methods}
                ests["Adaptive LMMSE + Wiener"] = adaptive.estimate(Y, Xref, nv_hat)
                ests["perfect CSI"] = H
                for k, Hh in ests.items():
                    e = np.mean(np.abs(Hh[data_mask] - H[data_mask]) ** 2)
                    Xh = pc.equalize(Y, Hh, nv_hat)[data_mask]
                    errs = np.sum(qam16_bits(Xh) != lab)  # symbol errors
                    acc[k][0] += e / n_pkt
                    acc[k][1] += errs
                    acc[k][2] += lab.size
            for k, (mse, ne, n) in acc.items():
                res[(prof, fd, snr, k)] = (10 * np.log10(mse) if mse > 0 else -np.inf, ne / n)
            print(f"{prof:9s} fd={fd:>4} Hz SNR={snr:>2} dB  noise-est bias {np.mean(nv_err):+.2f} dB  " +
                  "  ".join(f"{k.split()[0]}:{10*np.log10(acc[k][0]) if acc[k][0]>0 else -99:6.1f}dB/{acc[k][1]/acc[k][2]:.2e}"
                            for k in acc), flush=True)
    return res, list(methods) + ["Adaptive LMMSE + Wiener", "perfect CSI"]


if __name__ == "__main__":
    # usage: python simulate.py [ReferenceSignals folder]   (defaults to placeholder pilots)
    ref_dir = sys.argv[1] if len(sys.argv) > 1 else None
    BITS = 4                                   # 16QAM data, Ref_P0_<S>_4 pilots (power 1.8)
    t0 = time.time()
    snrs = list(range(0, 36, 5))
    plan = {S: [("ETU", 300), ("10 ns rms", 300)] for S in (3, 6, 12, 24)}
    out = {}
    for spacing, cases in plan.items():
        L = pc.flexlink_layout(ref_spacing=spacing, packet_profile=3)
        ref = pc.load_reference_vector(ref_dir, 0, spacing, BITS, layout=L) if ref_dir else None
        print(f"\n=== ReferenceSymbolProfile {spacing}, PacketProfile 3, "
              f"{'Ref_P0_%d_%d' % (spacing, BITS) if ref_dir else 'placeholder pilots'} ===")
        out[spacing] = run(L, port=0, snrs_db=snrs, cases=cases, ref_values=ref)
    np.save("results.npy", {"snrs": snrs, "out": out}, allow_pickle=True)
    print(f"done in {time.time()-t0:.0f}s")
