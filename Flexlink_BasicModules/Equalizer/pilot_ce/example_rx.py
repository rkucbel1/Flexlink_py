"""Receiver flow for one FlexLink packet with the real Ref_P0 vectors.
Replace `Y` with your received FFT-output grid (n_sym x 841, columns = tones -420..420)."""
import numpy as np
import pilot_ce as pc

REF_DIR = "../ReferenceSignals"      # folder with Ref_P0_<S>_<bits>.h5
S, PP, BITS = 3, 3, 4                # tone spacing, ref symbol every PP symbols, bits/symbol (16QAM)
N_SYM = 73                           # packet length: 37, 49, 73, 97 or 121 (ends with a reference symbol)

layout = pc.flexlink_layout(ref_spacing=S, packet_profile=PP, n_sym=N_SYM)  # = Construct_Resource_Grid
ref = pc.load_reference_vector(REF_DIR, port=0, spacing=S, bits_per_symbol=BITS, layout=layout)
Xref = pc.reference_grid(layout, 0, values=ref)                 # known pilots on the grid
pilot_power = float(np.mean(np.abs(ref) ** 2))                  # 1.8 for 16QAM corner pilots

# Build once; it caches filters per (delay window, SNR design point) and reuses them.
est = pc.AdaptiveChannelEstimator(layout, port=0, fd_hz=300.0, pilot_power=pilot_power)  # fd_hz = worst case

# --- stand-in received grid (2-tap channel + noise) so the script runs ---
rng = np.random.default_rng(0)
H_true = 0.8 * np.exp(1j * 0.3) + 0.3 * np.exp(-2j * np.pi * layout.tones * 20 / layout.n_fft)
Xtx = Xref.copy()
dm = layout.types == pc.DATA
Xtx[dm] = (rng.choice([-3, -1, 1, 3], dm.sum()) + 1j * rng.choice([-3, -1, 1, 3], dm.sum())) / np.sqrt(10)
Y = H_true * Xtx + 0.03 * (rng.standard_normal(Xtx.shape) + 1j * rng.standard_normal(Xtx.shape))

# --- Rx ---
nv = pc.estimate_noise_var(Y, layout, 0)         # sigma^2 from the empty P1 comb + nulls
H = est.estimate(Y, Xref, nv)                    # (N_SYM, 841) channel estimate
Xhat = pc.equalize(Y, H, nv)                     # MMSE-equalized grid
data = Xhat[dm]                                  # equalized data symbols, row-major order

mse = np.mean(np.abs(H[dm] - np.broadcast_to(H_true, H.shape)[dm]) ** 2)
print(f"noise_var={nv:.5f}  window={est.last_window}  channel MSE={10*np.log10(mse):.1f} dB  n_data={data.size}")
