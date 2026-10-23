# pilot_ce: FlexLink reference handling, channel estimation and equalization

## Files
- `pilot_ce.py`: the module.
  - `flexlink_layout(ref_spacing, packet_profile, n_sym)`: grid layout for tone spacing 3/6/12/24, a reference symbol every 1/3/6/12 symbols, and packet length 37/49/73/97/121 (default 73). These lengths always end with a reference symbol, matching `Construct_Resource_Grid.py`. The `ref_at_end` option is only for the earlier studies with lengths 36/48/72/96/120.
  - `load_reference_vector(folder, port, spacing, bits)`: reads `Ref_P{port}_{S}_{bits}.h5` and checks the vector length against the layout.
  - `reference_grid(layout, port, values=ref)`: known pilots placed on the grid.
  - `estimate_noise_var`, `detect_port`: noise level and active-port detection.
  - `AdaptiveChannelEstimator` (recommended) and `ChannelEstimator` (fixed window): the estimators.
  - `equalize`: MMSE equalizer, for EVM and plots.
  - `equalize_for_demapper(Y, H, nv, mask)`: returns `x = Y/H` and the per-tone noise `nv/|H|²`. Use this for `QAM_Demapper` soft modes, passing the per-tone noise as `noise_var`. That needs the patched `QAM_Demapper.py`, which accepts a per-symbol `noise_var` array. On ETU at 300 Hz, a single scalar noise_var loses 0.6–0.7 bits/bit of LLR information, against 0.01–0.09 with per-tone noise.
- `flexlink_ref_layouts.npz`: reference-symbol tone layouts taken from `ResourceGrid_Flexlink.ods`.
- `minih5.py`: pure-Python HDF5 reader, used only if `h5py` is missing.
- `example_rx.py`: the receiver flow for one packet. Set `REF_DIR` to your ReferenceSignals folder.
- `simulate.py <ReferenceSignals dir>`, `plot_results.py`: the grid-level Monte-Carlo comparison that produces `estimator_comparison.png`.
- `waveform_test.py <Grid_Payload_Tx.h5> <ReferenceSignals dir>`: waveform-level test on a real packet. It runs OFDM modulation with the OFDM_Modulator_841 tone map, a time-varying multipath channel (ETU or the Channeldefects.json FIR profile), noise, and demodulation with FFT_StartTime −12 plus phase-ramp removal (as Demodulate_Waveform_Rx). Then it estimates the channel, equalizes, and reports EVM and QPSK BER against a perfectly known channel.

## Verified against the FlexLink Tx/Rx code (Oct 19)
- `flexlink_layout(3, 3, n_sym=73)` is identical to `Grid_Resource_Tx.h5` and `Resource_Grid_Rx.h5` from `Construct_Resource_Grid.py`.
- The `Ref_P0_3_2` pilots from `reference_grid()` match `Grid_Payload_Tx.h5` exactly. The vector is placed in increasing k, which is ascending tone order. The P1 and DC tones are zero.
- `Grid_Payload_Rx.h5` equals `Grid_Payload_Tx.h5` to within 6e-15. Running it through the estimator and equalizer gives an EVM of −86 dB.
- Packet lengths are now 37/49/73/97/121 (a multiple of 12, plus 1), so every packet ends with a reference symbol and `ref_at_end` is no longer needed.
- The Rx starts the FFT 12 samples early (`FFT_StartTime` = −12) and removes the phase ramp. The interference-free delay span is 116 − 12 = 104 samples (5.2 µs) after the timing point, plus 12 samples before it. The channel's delays are only ≥ 0 when the packet detector's timing lands on the first path; see *Timing on a later path* below (corrected 20 Oct 2026).

## Waveform-level results (real 73-symbol QPSK packet, tone spacing 3, reference every 3 symbols)
EVM in dB / uncoded QPSK BER after MMSE equalization. The EVM is averaged over faded tones, so deep fades dominate it.

| Channel | SNR | Linear f + linear t | Adaptive LMMSE + linear t | Adaptive LMMSE + Wiener t | Perfect channel |
|---|---|---|---|---|---|
| Channeldefects FIR, 300 Hz | 20 dB | −12.4 / 7.8e-3 | −13.5 / 5.9e-3 | −13.6 / 5.7e-3 | −13.6 / 5.6e-3 |
| Channeldefects FIR, 300 Hz | 35 dB | −23.2 / 3.5e-4 | −24.1 / 2.8e-4 | −24.3 / 2.8e-4 | −24.1 / 3.2e-4 |
| ETU, 70 Hz | 35 dB | −17.0 / 1.8e-3 | −26.4 / 1.3e-4 | −26.8 / 1.3e-4 | −27.0 / 1.4e-4 |
| ETU, 300 Hz | 20 dB | −12.2 / 8.1e-3 | −13.4 / 6.0e-3 | −13.9 / 5.2e-3 | −14.1 / 4.9e-3 |
| ETU, 300 Hz | 35 dB | −15.5 / 2.2e-3 | −22.6 / 4.1e-4 | −23.0 / 3.6e-4 | −23.3 / 3.3e-4 |

- The adaptive LMMSE estimator is within 0.2–0.7 dB EVM of a perfectly known channel.
- Linear interpolation in both directions is 6–14× worse in BER on ETU at 35 dB.
- With the end reference symbol, linear interpolation in time costs at most 0.5 dB, so it is the FPGA-friendly choice at reference spacing 3.
- At ETU and 300 Hz, inter-carrier interference shows up as a floor of about −33 dB. The noise estimate from the empty tones includes it (32.6 dB measured at 35 dB SNR), which is the right input for the MMSE equalizer.

## Recommended receiver
```python
layout = pc.flexlink_layout(S, PP, n_sym=packet_len)
ref    = pc.load_reference_vector(REF_DIR, 0, S, bits, layout=layout)
Xref   = pc.reference_grid(layout, 0, values=ref)
est    = pc.AdaptiveChannelEstimator(layout, 0, fd_hz=300, pilot_power=np.mean(abs(ref)**2))  # build once
nv     = pc.estimate_noise_var(Y, layout, 0)
H      = est.estimate(Y, Xref, nv)
Xhat   = pc.equalize(Y, H, nv)
```
For each packet, the adaptive estimator measures the delay profile from the pilots and picks a delay window: the end from 8…116 samples, and the start from −4 down to −116 samples when paths arrive before the timing point. It then applies a cached LMMSE filter across frequency and a Wiener filter across time. A long channel (ETU) gets the full CP window. A short channel (10 ns) gets 8 samples, which is worth 8–25 dB of channel MSE over a fixed CP-length window, with the biggest gains at spacings 12 and 24.

## Timing on a later path (found 20 Oct 2026)
The studies above placed the first path at delay 0. In the real chain, `Packet_Detector.py` times the packet from the Preamble B matched-filter peak, which is the **strongest** path. In ETU and EVA that is often a later path of the opening cluster (for example the 500 ns path, 10 samples), so the earlier paths arrive 5–12 samples *before* the timing point. `FFT_StartTime` −12 keeps them free of interference, but the old LMMSE window started at −4 and filtered them out.

- Symptom: with a strong channel and no noise, some packets failed CRC with LMMSE, LMMSE_LinearTime and PCA, while `Linear` (no window) decoded them with no errors (`Rx/Diagnose_Defects.py`, ETU 300 Hz, seeds 14 and 17).
- In a model of the estimator with timing on the strongest path, about half of ETU channels start before −4. Those gave channel-estimate errors around −5 dB instead of −38 dB.
- Fix in `Channel_Estimator.py`: `channel_extent()` finds the first and last delay with energy over −(limit+8)…limit, and `choose_window()` sets the start from `WINDOW_STARTS` (−4 … −116). It falls back to −4 when the window would exceed the resolvable span 0.9·N_FFT/S, so spacings 12 and 24 behave as before.
- Fix in `PCA/PCA_Estimator.py`: the bases are trained for channels starting at delay 0 (±2 samples of jitter), so the pilots are re-centred instead: delayed by the measured early start (a phase ramp across the tones), estimated with the existing basis, then advanced back. No retraining is needed.

Channel-estimate error at spacing 3, timing on the strongest path, 300 Hz Doppler, K = 8, 30–40 packets each (worst packet in brackets):

| Channel, SNR | LMMSE before | LMMSE after | PCA before | PCA after |
|---|---|---|---|---|
| ETU, 35 dB | −43.4 (−0.4) | −44.7 (−43.3) | −39.6 (−2.1) | −45.5 (−43.8) |
| ETU, 20 dB | — | −30.7 (−28.2) | −18.9 (−0.4) | −32.1 (−28.9) |
| EVA, 35 dB | — | −46.8 (−45.2) | −45.6 (−3.5) | −47.0 (−45.0) |
| EPA, 20 dB | — | −37.6 (−34.7) | −39.0 (−35.3) | −38.9 (−35.3) |

— = not measured in this comparison (a frequency-only test gave LMMSE worst packets around −1 dB before the fix at 20 dB).

The wider LMMSE window costs about 1 dB of median error on EPA at 20 dB when timing lands late; PCA has no such cost because it only re-centres beyond 2 samples.

## Findings (16-QAM, boosted pilots, PacketProfile 3, 300 Hz Doppler)
| Spacing | ETU (5 µs) | 10 ns rms |
|---|---|---|
| 3 | −46.8 dB MSE at 35 dB SNR, SER = perfect-CSI | −46.7 dB |
| 6 | −39.9 dB, SER within ~15% of perfect CSI | −46.4 dB |
| 12 | not estimable: about −11.7 dB floor | −45.1 dB |
| 24 | not estimable: about −8 dB floor | −43.1 dB |

With a pilot every S tones and N = 1024, only delays up to 1024/S samples can be told apart: 85 samples (4.3 µs) at S = 12 and 43 samples (2.1 µs) at S = 24. ETU reaches 5 µs (100 samples), so spacings 12 and 24 alias. No estimator can fix that; those profiles only suit short-delay channels.

## Earlier study: reference-symbol spacing in time and packet length
*This study used lengths 36/48/72/96/120, before the spec changed to 37/49/73/97/121. It is the reason the spec now ends every packet with a reference symbol; the "+ ref on last symbol" rows show how the current lengths behave.*

Packet lengths 36/48/72/96/120 are all multiples of 12, so with a reference symbol every PP symbols starting at symbol 0, the last PP − 1 symbols always come after the last reference symbol and must be extrapolated.

Results for tone spacing 3, ETU at 300 Hz, and 16-QAM. Values are channel-estimate error over all data / over the tail symbols, and the symbol error rate (SER) with the estimate vs with perfect channel knowledge:

| Ref. every | Length | SNR 35 dB: error all / tail | SER estimate vs perfect |
|---|---|---|---|
| 1 | 36–120 | −50 … −53 dB / none | equal |
| 3 | 36–120 | −45 … −48 / −39 dB | within 5–8% |
| 6 | 36–120 | −40 … −44 / −33 … −35 dB | within 10–28% |
| 12 | 36 | −22 / −17 dB | **14× worse** (2.9e-2 vs 2.1e-3) |
| 12 | 72 | −33 / −26 dB | 2.2× worse |
| 12 | 120 | −36 / −27 dB | 1.5× worse |
| 12 + ref on last symbol | 36–120 | −40 … −43 / −40 … −42 dB | within 13–26% |

- Spacings 1, 3 and 6 work at every packet length.
- Spacing 12 needs a reference symbol on the last symbol (`ref_at_end=True`), especially for short packets. That costs one more reference symbol, about 2% of data tones at length 36 and under 1% at 120. It also lifts the tail at spacing 6 from about −34 to −44 dB, which may matter for 64-QAM.
- **With a reference symbol at the end of every packet** (now the spec, through lengths 37/49/73/97/121): every symbol is interpolated between two reference symbols, and nothing is extrapolated. With the Wiener time filter, SER stays within about 5–25% of perfect channel knowledge for every spacing, length and Doppler tested. It also makes plain linear interpolation in time usable for spacings 3 and 6 (within about 1.5× of perfect at 300 Hz). At spacing 12 and 300 Hz, linear in time is still about 8× worse, so keep the Wiener filter there.
- The time filter is designed for a fixed worst-case Doppler (`fd_hz`, default 300 Hz). Per-packet Doppler estimation was tried and rejected, because one packet is too short to estimate it reliably.

## What the reference files contain
- `Ref_P0_{S}_{bits}`: one vector, reused in every reference symbol, with length 840/S (280/140/70/35).
- Pilots are two antipodal constellation corners. Mean power is 1.0 for BPSK/QPSK, 1.8 for 16QAM and 2.333 for 64QAM, a pilot boost of +2.6 dB and +3.7 dB. Pass that power as `pilot_power`, because the LS noise is σ²/pilot_power.
- Within one modulation, every spacing uses the same scrambled bit sequence (Gold code, C_init = 129), truncated to its length.

## Assumptions to check
- Vector order and data-symbol DC null: both confirmed by `Construct_Payload_Grid.py` and `Construct_Resource_Grid.py`.
- **PacketProfile 12:** resolved by the 37/49/73/97/121 packet lengths, which end with a reference symbol.
- **Numerology:** at the Rx, N_FFT = 1024 at 20 Msps with CP = 116 samples (5.8 µs) and a 57 µs symbol. The Tx uses a 2048-point IFFT at 40 Msps with CP = 232, and the Rx decimates by 2.
- **FFT timing:** handled per packet since 20 Oct 2026. `Channel_Estimator.AdaptiveChannelEstimator` moves the window start below −4 when the delay profile shows paths before the timing point, and `PCA_Estimator` re-centres the channel instead (see below). `pilot_ce.py` in this folder is the original study module and still uses a fixed −4 start; the Rx chain uses `Channel_Estimator.py`.
- **Simulation model:** frequency-domain simulation with a block-constant channel per symbol (no ICI). At 300 Hz Doppler and a 57 µs symbol, the ICI is about −45 dB.
