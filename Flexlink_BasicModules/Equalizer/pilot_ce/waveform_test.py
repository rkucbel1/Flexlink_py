"""
Waveform-level test on a real FlexLink packet.

Grid_Payload_Tx  -> OFDM modulate (N=1024, CP=116, 20 Msps, same tone->IFFT map as
OFDM_Modulator_841) -> time-varying multipath (per-sample Jakes fading, fractional delays)
-> AWGN -> OFDM demodulate with FFT_StartTime (-12) and phase-ramp removal
(as Demodulate_Waveform_Rx) -> channel estimation -> MMSE equalizer -> EVM vs Tx.

Because the channel varies inside each OFDM symbol, this includes inter-carrier
interference and the FFT timing offset, which the grid-level simulation does not.

    python waveform_test.py <Grid_Payload_Tx.h5> <ReferenceSignals dir>
"""
import sys

import numpy as np

import pilot_ce as pc

N, CP, FS = 1024, 116, 20e6
POS_IDX, NEG_IDX = np.arange(0, 421), np.arange(604, 1024)     # tones 0..420, -420..-1


def ofdm_mod(grid):
    """grid (n_sym, 841), columns = tones -420..420 -> time samples (with CP)."""
    out = []
    for row in grid:
        X = np.zeros(N, complex)
        X[POS_IDX] = row[420:]
        X[NEG_IDX] = row[:420]
        x = np.fft.ifft(X) * np.sqrt(N)
        out.append(np.concatenate([x[-CP:], x]))
    return np.concatenate(out)


def ofdm_demod(x, n_sym, start_time=-12, derotate=True):
    tones = np.arange(-420, 421)
    grid = np.zeros((n_sym, 841), complex)
    for i in range(n_sym):
        s = i * (N + CP) + CP + start_time
        X = np.fft.fft(x[s:s + N]) / np.sqrt(N)
        row = np.concatenate([X[NEG_IDX], X[POS_IDX]])
        if derotate and start_time:
            row = row * np.exp(-2j * np.pi * tones * start_time / N)
        grid[i] = row
    return grid


PROFILES = {
    "ETU": (np.array([0, 50, 120, 200, 230, 500, 1600, 2300, 5000]) * 1e-9,
            np.array([-1, -1, -1, 0, 0, 0, -3, -5, -7.0]), 0.0),
    # Flexlink Channeldefects.json "FIR" profile (KFactor 8 -> Rician first tap)
    "Channeldefects FIR": (np.array([0.0, 3e-9, 7e-8, 9e-8, 1.1e-7, 1.9e-7, 4.1e-7]),
                           np.array([0.0, -1.0, -2.0, -3.0, -8.0, -17.2, -20.8]), 8.0),
}


def multipath(x, profile, fd_hz, rng, n_sin=16, half_len=8, return_gains=False):
    """Time-varying tapped delay line; fractional delays via windowed sinc."""
    delays, pdb, K = PROFILES[profile]
    p = 10 ** (pdb / 10)
    p /= p.sum()
    t = np.arange(x.size + 200) / FS
    y = np.zeros(x.size + 200, complex)
    xp = np.concatenate([x, np.zeros(200, complex)])
    gains = []
    for i, (tau, pw) in enumerate(zip(delays, p)):
        ang = 2 * np.pi * (np.arange(n_sin) + rng.random(n_sin)) / n_sin
        ph = rng.uniform(0, 2 * np.pi, n_sin)
        g = np.zeros(t.size, complex)
        for a, b in zip(ang, ph):
            g += np.exp(1j * (2 * np.pi * fd_hz * np.cos(a) * t + b))
        g /= np.sqrt(n_sin)
        if i == 0 and K > 0:                                   # Rician LOS on first tap
            los = np.exp(1j * (2 * np.pi * fd_hz * np.cos(rng.uniform(0, np.pi)) * t + rng.uniform(0, 2 * np.pi)))
            g = np.sqrt(K / (K + 1)) * los + np.sqrt(1 / (K + 1)) * g
        d = tau * FS
        n = np.arange(-half_len, half_len + 1) + int(np.floor(d))
        h = np.sinc(n - d) * np.hanning(2 * half_len + 3)[1:-1]
        delayed = np.zeros_like(xp)
        for nn, hh in zip(n, h):
            if nn >= 0:
                delayed[nn:] += hh * xp[:xp.size - nn]
            else:
                delayed[:nn] += hh * xp[-nn:]
        y += np.sqrt(pw) * g * delayed
        gains.append((tau, np.sqrt(pw) * g))
    if return_gains:
        return y[:x.size], gains
    return y[:x.size]


def true_diag(gains, n_sym, start_time=-12):
    """Exact ICI-free channel per symbol/tone: each tap's gain averaged over the FFT
    window times its delay phase (the receiver's derotation makes the start time drop out)."""
    tones = np.arange(-420, 421)
    H = np.zeros((n_sym, 841), complex)
    for tau, g in gains:
        for i in range(n_sym):
            s0 = i * (N + CP) + CP + start_time
            H[i] += g[s0:s0 + N].mean() * np.exp(-2j * np.pi * tones * tau * FS / N)
    return H


def qpsk_ber(Xh, X, mask):
    a, b = Xh[mask], X[mask]
    return float(np.mean(np.concatenate([np.sign(a.real) != np.sign(b.real), np.sign(a.imag) != np.sign(b.imag)])))


def evm_db(Xh, X, mask):
    return 10 * np.log10(np.mean(np.abs(Xh[mask] - X[mask]) ** 2) / np.mean(np.abs(X[mask]) ** 2))


def run(Xtx, layout, ref, profile, fd, snr_db, rng, n_pkt=6):
    Xref = pc.reference_grid(layout, 0, values=ref)
    dm = layout.types == pc.DATA
    pp = float(np.mean(np.abs(ref) ** 2))
    ests = {
        "Linear f + linear t": pc.ChannelEstimator(layout, 0, "linear", "linear"),
        "Adaptive LMMSE + linear t": pc.AdaptiveChannelEstimator(layout, 0, pilot_power=pp, time="linear"),
        "Adaptive LMMSE + Wiener t": pc.AdaptiveChannelEstimator(layout, 0, pilot_power=pp),
    }
    x = ofdm_mod(Xtx)
    acc = {k: [] for k in list(ests) + ["Perfect CSI"]}
    for _ in range(n_pkt):
        seed = rng.integers(1 << 31)
        y_clean, gains = multipath(x, profile, fd, np.random.default_rng(seed), return_gains=True)
        Yc = ofdm_demod(y_clean, layout.n_sym)
        # genie: same channel, unit grid -> per-tone response averaged over the FFT window
        nv = 10 ** (-snr_db / 10)
        w = np.sqrt(nv / 2) * (rng.standard_normal(x.size) + 1j * rng.standard_normal(x.size))
        Y = Yc + ofdm_demod(w, layout.n_sym)
        nvh = pc.estimate_noise_var(Y, layout, 0)
        for k, e in ests.items():
            H = e.estimate(Y, Xref, nvh) if isinstance(e, pc.AdaptiveChannelEstimator) else e.estimate(Y, Xref)
            Xh = pc.equalize(Y, H, nvh)
            acc[k].append((evm_db(Xh, Xtx, dm), qpsk_ber(Xh, Xtx, dm)))
        Hg = true_diag(gains, layout.n_sym)               # exact ICI-free channel
        Xh = pc.equalize(Y, Hg, nvh)
        acc["Perfect CSI"].append((evm_db(Xh, Xtx, dm), qpsk_ber(Xh, Xtx, dm)))
    return {k: (float(np.mean([a for a, _ in v])), float(np.mean([b for _, b in v]))) for k, v in acc.items()}, nvh


if __name__ == "__main__":
    grid_file = sys.argv[1] if len(sys.argv) > 1 else "Grid_Payload_Tx.h5"
    ref_dir = sys.argv[2] if len(sys.argv) > 2 else "../ReferenceSignals"
    data, attrs = pc._read_h5(grid_file)
    Xtx = np.asarray(data["data"])
    S, PP, n_sym = int(attrs["ReferenceSymbolProfile"]), int(attrs["PacketProfile"]), Xtx.shape[0]
    bits = int(attrs["BitsPerSymbol"])
    layout = pc.flexlink_layout(S, PP, n_sym=n_sym)
    assert np.array_equal(layout.types, np.asarray(data["signal_types"])), "layout mismatch"
    ref = pc.load_reference_vector(ref_dir, 0, S, bits, layout=layout)
    rng = np.random.default_rng(1)
    print(f"Packet: {n_sym} symbols, ref every {PP}, tone spacing {S}, {attrs['Mod']}")
    for profile, fd in (("Channeldefects FIR", 300), ("ETU", 70), ("ETU", 300)):
        for snr in (20, 35):
            r, nvh = run(Xtx, layout, ref, profile, fd, snr, rng)
            print(f"{profile:19s} {fd:3d} Hz  SNR {snr} dB  (noise+ICI est {-10*np.log10(nvh):5.1f} dB)  EVM / BER: " +
                  "  ".join(f"{k}: {v[0]:6.1f} dB / {v[1]:.1e}" for k, v in r.items()), flush=True)
