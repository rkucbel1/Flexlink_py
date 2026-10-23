"""Plot results.npy produced by simulate.py -> estimator_comparison.png"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

r = np.load("results.npy", allow_pickle=True).item()
snrs, out = r["snrs"], r["out"]
COL = {"linear (baseline)": "#2a78d6", "LMMSE-f + linear-t": "#1baf7a",
       "LMMSE-f + Wiener-t": "#eda100", "Adaptive LMMSE + Wiener": "#eb6834", "perfect CSI": "#8a8a85"}
LAB = {"linear (baseline)": "Linear f + linear t (baseline)", "LMMSE-f + linear-t": "LMMSE f (CP window) + linear t",
       "LMMSE-f + Wiener-t": "LMMSE f (CP window) + Wiener t",
       "Adaptive LMMSE + Wiener": "Adaptive-window LMMSE f + Wiener t", "perfect CSI": "Perfect CSI"}
spacings = sorted(out)
channels = ["ETU", "10 ns rms"]
plt.rcParams.update({"font.size": 9.5, "axes.edgecolor": "#b5b4ad", "axes.labelcolor": "#3d3d3a",
                     "xtick.color": "#5f5e5a", "ytick.color": "#5f5e5a"})
fig, axes = plt.subplots(4, len(spacings), figsize=(18, 15), sharex=True)
for c, S in enumerate(spacings):
    res, names = out[S]
    for i, prof in enumerate(channels):
        am, ase = axes[2 * i, c], axes[2 * i + 1, c]
        for n in names:
            mse = [res[(prof, 300, s, n)][0] for s in snrs]
            ser = [res[(prof, 300, s, n)][1] for s in snrs]
            ls = "--" if n == "perfect CSI" else "-"
            if n != "perfect CSI":
                am.plot(snrs, mse, ls, color=COL[n], lw=2, marker="o", ms=3.5, label=LAB[n])
            ase.semilogy(snrs, ser, ls, color=COL[n], lw=2, marker="o", ms=3.5, label=LAB[n])
        am.plot(snrs, [-s for s in snrs], ":", color="#b5b4ad", lw=1.5, label="Raw LS at pilots (= −SNR)")
        am.set_ylim(-52, 2)
        ase.set_ylim(1e-3, 1)
        am.set_title(f"Spacing {S}, {prof}, 300 Hz: channel MSE", color="#1f1f1d", fontsize=10.5)
        ase.set_title(f"Spacing {S}, {prof}, 300 Hz: 16-QAM SER", color="#1f1f1d", fontsize=10.5)
    axes[-1, c].set_xlabel("SNR (dB)")
for a in axes.ravel():
    a.grid(True, color="#e6e5df", lw=0.8)
    a.spines[["top", "right"]].set_visible(False)
for i in range(2):
    axes[2 * i, 0].set_ylabel("Channel MSE on data tones (dB)")
    axes[2 * i + 1, 0].set_ylabel("Symbol error rate (uncoded)")
h, l = axes[1, 0].get_legend_handles_labels()
h2, l2 = axes[0, 0].get_legend_handles_labels()
fig.legend(h + [h2[-1]], l + [l2[-1]], loc="lower center", ncol=6, frameon=False, fontsize=10)
fig.suptitle("FlexLink channel estimation with real Ref_P0 vectors (16-QAM, boosted pilots), PacketProfile 3, "
             "N=1024, 20 Msps, CP 116", fontsize=12.5, color="#1f1f1d")
fig.tight_layout(rect=(0, 0.035, 1, 0.97))
fig.savefig("estimator_comparison.png", dpi=110)
