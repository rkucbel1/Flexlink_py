"""
PCA_Estimator.py  (lives in Flexlink_py/Flexlink_BasicModules/Equalizer/PCA)

Low-rank (principal component) channel estimator for EqualizerMode "PCA" in
Rx/Equalize_Payload_Grid.py. Uses the bases written by Train_PCA_Basis.py
(PCA_Basis_S<RefSignalSpacing>.h5 in this folder).

Per packet
----------
  1. LS estimates at the pilots of every reference symbol: h_p = Y / X_ref.
  1b. Timing re-centering: the bases are trained for channels that start at delay 0, but the
     packet detector locks onto the strongest path, which in ETU/EVA is often a later one,
     so earlier paths arrive at negative delays. The earliest delay with energy
     (Channel_Estimator.channel_extent) gives the shift, applied when the channel starts more
     than RECENTER_TOLERANCE (2) samples early; the pilots are delayed by it
     (a phase ramp across the tones), estimated, and the result is advanced back. A
     constant shift does not change the time direction, so nothing else is affected and the
     trained bases stay valid.
  2. Class (Short / Medium / Long) from the measured delay profile of the re-centered pilots
     (Channel_Estimator.choose_window_end): the smallest class whose MaxDelay covers it.
  3. Fit check: c = U_r^H h_p are the component amplitudes. If the channel matches the
     training models, E|c_i|^2 = P lambda_i + sigma^2, so chi = mean |c_i|^2 / (P lambda_i +
     sigma^2) is about 1 (measured 0.3..1.9 for EPA/EVA/ETU/10 ns). A channel unlike the models
     puts energy into components with tiny lambda_i and chi jumps (5..500 measured). Also
     checked: energy outside the r components, ||h_p||^2 - ||c||^2 - (Np - r) sigma^2, above
     MismatchLimit (-25 dB) of the total. If either test fails, the class's fallback twin
     <Class>U (uniform delay power, the same prior as the LMMSE mode) is used instead, so a
     channel unlike the models is never estimated worse than with LMMSE. Both tests reuse
     c, so they cost r extra multiplies per reference symbol.
  4. Channel power P and LS noise sigma^2 = noise_var / pilot_power; per component gain
         g_i = lambda_i / (lambda_i + sigma^2 / P) / lambda_i = 1 / (lambda_i + sigma^2 / P)
     Components with g_i * lambda_i below MinComponentGain are dropped (they are noise).
  5. Frequency direction, for each reference symbol (two thin products, rank r):
         c      = U_r^H h_p            (r values: projection onto the principal components)
         H_all  = A0 (g * c)           (841 tones)
  6. Time direction: Wiener filter (Jakes, EqualizerMaxDoppler) or linear interpolation,
     the same as the LMMSE mode (Channel_Estimator.t_wiener / t_linear).

Cost per reference symbol: r * (Np + 841) complex multiply-accumulates instead of
841 * Np for the full LMMSE matrix. No matrix inversion per packet: the noise level only
changes the r gains g_i.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
MIN_COMPONENT_GAIN = 0.1      # drop components whose Wiener gain lambda/(lambda+sigma^2) is below this
MISMATCH_LIMIT_DB = -25.0     # channel energy outside the trained components that triggers the fallback
CHI_LIMIT = 3.0               # mean |c_i|^2 / (P lambda_i + sigma^2) above which the fallback is used
RECENTER_TOLERANCE = 2        # samples before delay 0 the bases cover (TimingJitter_samples); re-centre beyond


def basis_file(spacing: int, folder: Path | None = None) -> Path:
    return Path(folder or HERE) / f"PCA_Basis_S{spacing}.h5"


def load_basis(path: Path, port: int) -> list[dict]:
    """Classes from a PCA_Basis_S<S>.h5 file, sorted by MaxDelay."""
    import h5py
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"{path.name} not found in {path.parent} - run "
                                "Flexlink_BasicModules/Equalizer/PCA/Train_PCA_Basis.py once")
    with h5py.File(path, "r") as f:
        names = str(f.attrs["Classes"]).split(",")
        max_delays = np.asarray(f.attrs["MaxDelays"]).ravel()

        def part(name):
            pre = f"{name}_P{port}"
            if f"{pre}_U" not in f:
                return None
            return {"name": name, "U": np.asarray(f[f"{pre}_U"][()]),
                    "lam": np.asarray(f[f"{pre}_lambda"][()]).astype(float),
                    "A0": np.asarray(f[f"{pre}_A0"][()]),
                    "pilot_k": np.asarray(f[f"{pre}_pilot_k"][()]).astype(int)}

        classes = []
        for name, md in zip(names, max_delays):
            c = part(name)
            c["max_delay"], c["fallback"] = int(md), part(name + "U")
            classes.append(c)
    return sorted(classes, key=lambda c: c["max_delay"])


class PCAChannelEstimator:
    """estimate(Y, Xref, noise_var) -> H (n_sym, 841). Same interface as
    Channel_Estimator.AdaptiveChannelEstimator; last_class / last_rank / last_misfit_db
    describe the last packet (last_class ends in 'U' when the fallback was used)."""

    def __init__(self, layout, port, classes: list[dict], ce, fd_hz=300.0, pilot_power=1.0,
                 time="wiener", min_gain=MIN_COMPONENT_GAIN, mismatch_db=MISMATCH_LIMIT_DB,
                 chi_limit=CHI_LIMIT):
        self.layout, self.port, self.ce = layout, port, ce
        self.fd_hz, self.pp, self.time, self.min_gain = fd_hz, pilot_power, time, min_gain
        self.mismatch = 10 ** (mismatch_db / 10)
        self.chi_limit = chi_limit
        self.ref_syms = layout.ref_symbols
        self.cols = layout.pilot_idx(self.ref_syms[0], port)
        if not all(np.array_equal(self.cols, layout.pilot_idx(s, port)) for s in self.ref_syms):
            raise NotImplementedError("staggered pilots are not supported")
        for c in classes + [c["fallback"] for c in classes if c.get("fallback")]:
            if not np.array_equal(c["pilot_k"], self.cols):
                raise ValueError(f"PCA basis class {c['name']} was trained for other pilot positions - "
                                 "rerun Train_PCA_Basis.py")
        self.classes = classes
        S = int(np.median(np.diff(layout.tones[self.cols])))
        self.limit = min(layout.cp_len, int(0.9 * layout.n_fft / S))
        self.last_class = self.last_rank = self.last_window = self.last_misfit_db = None
        self.last_chi = None
        self.last_shift = 0

    def pick_class(self, Hls, nv_ls) -> dict:
        end = self.ce.choose_window_end(Hls, self.layout.tones[self.cols].astype(float),
                                        self.layout.n_fft, nv_ls, self.limit)
        self.last_window = (-4, int(end))
        for c in self.classes:
            if end <= c["max_delay"]:
                return c
        return self.classes[-1]

    def recenter_shift(self, Hls, nv_ls) -> int:
        """Samples to delay the channel by so its earliest path sits at delay >= 0."""
        if not hasattr(self.ce, "channel_extent"):          # older Channel_Estimator.py
            return 0
        first, _ = self.ce.channel_extent(Hls, self.layout.tones[self.cols].astype(float),
                                          self.layout.n_fft, nv_ls, self.limit)
        # the bases already allow +/- TimingJitter_samples (2) of timing error
        return -first if first < -RECENTER_TOLERANCE else 0

    def estimate(self, Y, Xref, noise_var):
        Hls = Y[np.ix_(self.ref_syms, self.cols)] / Xref[np.ix_(self.ref_syms, self.cols)]
        nv_ls = noise_var / self.pp
        shift = self.recenter_shift(Hls, nv_ls)
        self.last_shift = shift
        if shift:
            Hls = self.ce.shift_delays(Hls, self.layout.tones[self.cols], self.layout.n_fft, shift)
        c = self.pick_class(Hls, nv_ls)
        if shift:                                            # report in the packet's own timing
            self.last_window = (self.last_window[0] - shift, self.last_window[1] - shift)
        power = max(float(np.mean(np.abs(Hls) ** 2)) - nv_ls, 0.01 * nv_ls, 1e-12)
        s2 = nv_ls / power                                   # LS noise relative to channel power

        # fit check (see module docstring): does the channel look like the training models?
        n_p, r = self.cols.size, c["lam"].size
        C = Hls @ c["U"].conj()                              # (n_ref, r) component amplitudes
        e_tot = float(np.sum(np.abs(Hls) ** 2))
        out = (e_tot - float(np.sum(np.abs(C) ** 2)) - self.ref_syms.size * (n_p - r) * nv_ls) / max(e_tot, 1e-30)
        chi = float(np.mean(np.abs(C) ** 2 / (power * c["lam"] + nv_ls)))   # ~1 when the model fits
        self.last_misfit_db = float(10 * np.log10(max(out, 1e-6)))
        self.last_chi = chi
        if (out > self.mismatch or chi > self.chi_limit) and c.get("fallback") is not None:
            c = c["fallback"]

        lam = c["lam"]
        keep = lam / (lam + s2) >= self.min_gain
        U, A0, g = c["U"][:, keep], c["A0"][:, keep], 1.0 / (lam[keep] + s2)
        self.last_class, self.last_rank = c["name"], int(keep.sum())

        Hf = ((Hls @ U.conj()) * g) @ A0.T                   # (n_ref, 841)
        if self.time == "linear":
            T = self.ce.t_linear(self.layout.n_sym, self.ref_syms)
        else:
            # error left after the frequency filter: s2 * mean row energy of W = A0 diag(g) U^H
            w_energy = float(np.mean(np.sum(np.abs(A0 * g) ** 2, axis=1)))
            T = self.ce.t_wiener(self.layout.n_sym, self.ref_syms, self.layout.t_sym, self.fd_hz,
                                 s2 * w_energy)
        H = T @ Hf
        if shift:                                            # undo the re-centering
            H = self.ce.shift_delays(H, self.layout.tones, self.layout.n_fft, -shift)
        return H
