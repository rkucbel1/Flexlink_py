Flexlink_py · Rx · as of 23 Oct 2026

# Payload Rx Chain

From the received waveform to the decoded transport blocks. Steps 3 to 9 run in order; each reads the file the step before it wrote. `Rx/Run_Rx_Chain.py` runs them all, for one antenna or several (see *Multiple antennas and diversity combining*).

```
Parameters_Rx_payload.json ──► 2 Initialize_Rx.py ──► Rx/Resource_Grid_Rx.h5  (+ Calculated_Rx)

Waveform_Payload_Tx.h5                    no Mode set in Channel_Defects.json   (Rx/, else Tx/)
Waveform_Payload_DefectsModeled_Tx.h5     any Mode set to 1 (Tx/Defect_Model.py) (Rx/, else Tx/)
  │ 3 Decimate_Payload_Rx.py        40 → 20 MHz, decimation gain removed; with defects keeps
  ▼                                 the first NumberOfAntennas rays (one row each)
Waveform_Payload_Dec_Rx.h5
  │ 4 Packet_Detector.py [--ray N]  packet detection, frequency offset removed, cut at the payload start
  ▼                                 (Flexlink_BasicModules/Preamble; pass-through if Use_Preamble = 0)
Decimated_Payload_Packet_Rx_Ray<N>.h5      from here on every file ends in _Ray1 .. _Ray4 (--ray N, default 1)
  │ 4a Process_Control_Field_Rx.py  Use_ControlField = 1 only, once for all antennas: decode the control
  │                                 field, update Defined_Rx (+ Initialize_Rx.py) if the payload settings
  │                                 changed, cut the control field off every packet file
  │                                 (→ ControlField_Rx.h5; CRC fail = bad packet, the chain stops)
  │ 5 Demodulate_Waveform_Rx.py     OFDM demodulation (FFT_StartTime), phase ramp and demodulator gain removed
  ▼
Grid_Payload_Rx_Ray<N>.h5
  │ 6 Equalize_Payload_Grid.py      channel estimate from the reference signals, x = Y/H, per-signal noise
  ▼                                 (Flexlink_BasicModules/Equalizer, ReferenceSignals/Ref_P<Port>_<RefSignalSpacing>_<bits>.h5;
                                    PCA mode: Equalizer/PCA/PCA_Basis_S<RefSignalSpacing>.h5)
Grid_Equalized_Rx_Ray<N>.h5                steps 4-6 run once per antenna
  │ 6b Combine_Antennas_Rx.py       NumberOfAntennas > 1: maximum-ratio combining of the antennas
  ▼                                 (skipped for one antenna)
Grid_Equalized_Rx_Combined.h5              one antenna: steps 7-9 read the _Ray1 files instead
  │ 7 Get_DataSigs_QAM.py           Data signals (and their noise) picked out of the grid
  ▼
Sequence_DataSigs_QAM_Combined.h5          (or _Ray1, or _RayN with --ray N)
  │ 8 Get_Demapped_Bitstream.py     soft (P(bit = 1), per-signal noise) or hard demapping
  ▼                                 (Flexlink_BasicModules/QAM_Mapper)
Bitstream_Encoded_Rx_Combined.h5
  │ 9 Get_Decoded_Bitstream.py      descramble, rate de-matching, deinterleave, LDPC, CRC
  ▼                                 (Rx/Decode, Flexlink_BasicModules)
Bitstream_TransportBlks_Rx_Combined.h5
```

## What each step reads from the JSON

| Step | Script | Values used |
| --- | --- | --- |
| 2 | Initialize_Rx.py | All of Defined_Rx. Writes Calculated_Rx and Rx/Resource_Grid_Rx.h5 (File_Rx.ResourceGrid_Rx). Run it after every change to Defined_Rx. |
| 3 | Decimate_Payload_Rx.py | Mode from Flexlink_BasicModules/Channel_Defects/Channel_Defects.json. No Mode set: File_Rx.Waveform_Payload_Rx. Any Mode set: File_Rx.Waveform_Payload_DefectsModeled_Rx and NumberOfAntennas. Output File_Rx.Waveform_Payload_Dec_Rx. The input sample rate (20 or 40 MHz) comes from the waveform file's SampleRate attribute. |
| 4 | Packet_Detector.py | Use_Preamble, File_Rx.Waveform_Payload_Dec_Rx (row N with --ray N), File_Rx.Waveform_Payload_Packet_Rx. Detector settings (SA_Len, thresholds) come from Flexlink_BasicModules/Preamble/Receive_Packet_Detector.json, the local Preamble B from Flexlink_BasicModules/Preamble/Preamble.h5. |
| 4a | Process_Control_Field_Rx.py | Use_ControlField, FFT_StartTime, EqualizerMode, EqualizerMaxDoppler; writes RefSymbolSpacing, RefSignalSpacing, PacketLength, Mod, NumberOfTxAntennas, EncoderBlockSize, EncoderCodeRate, Repetition, ClientFlag and C_init_Dat (the payload scrambler id, ControlField C_init) when the control field differs (then runs Initialize_Rx.py, except when only C_init_Dat changed), checks NDB_A against Calculated_Rx.AvailableDataBlocks. The control field format comes from Tx/ControlSignals/Parameters_Control_Field.json (fixed by design). Output File_Rx.ControlField_Rx. |
| 5 | Demodulate_Waveform_Rx.py | PacketLength, FFT_StartTime, File_Rx.Waveform_Payload_Packet_Rx, File_Rx.Grid_Payload_Rx |
| 6 | Equalize_Payload_Grid.py | Mod, Port, RefSignalSpacing, EqualizerMode, EqualizerMaxDoppler, PacketLength and RefSymbolSpacing (checked against the grid), File_Rx.Grid_Payload_Rx, File_Rx.ResourceGrid_Rx, File_Rx.Grid_Equalized_Rx |
| 6b | Combine_Antennas_Rx.py | File_Rx.Grid_Equalized_Rx (the _Ray1 … _RayN files), File_Rx.Waveform_Payload_Dec_Rx (NumberOfAntennas of the run, and the age check). Writes File_Rx.Grid_Equalized_Rx with _Combined. |
| 7 | Get_DataSigs_QAM.py | Mod, PacketLength, RefSignalSpacing and RefSymbolSpacing (checked against the grid), NumberOfDataSigs (checked), File_Rx.Grid_Equalized_Rx, File_Rx.ResourceGrid_Rx, File_Rx.Sequence_DataSigs_QAM |
| 8 | Get_Demapped_Bitstream.py | Mod, Demapping, NumberOfDataSigs (checked), File_Rx.Sequence_DataSigs_QAM, File_Rx.Bitstream_Encoded_Rx |
| 9 | Get_Decoded_Bitstream.py | Repetition, EncoderBlockSize, EncoderCodeRate, CRC, C_init_Dat, AvailableDataBlocks, CodeWordSize, ZeroPadding, TransportBlockSize, ScrambledSequenceSize, File_Rx.Bitstream_Encoded_Rx, File_Rx.Bitstream_TransportBlks_Rx |

## Multiple antennas and diversity combining

When any `Mode` in `Flexlink_BasicModules/Channel_Defects/Channel_Defects.json` is 1, `Tx/Defect_Model.py` writes one waveform per ray, and step 3 keeps the first `NumberOfAntennas` of them (1 to 4; it stops with an error if the file has fewer rays). `Waveform_Payload_Dec_Rx.h5` then holds one row per antenna and records `NumberOfAntennas`. Without defects there is always one antenna.

The easiest way to run everything is

```
python Rx/Run_Rx_Chain.py                # steps 3-9, all antennas, combined, compared with the Tx blocks
python Rx/Run_Rx_Chain.py --per-ray      # also decode every antenna on its own, for comparison
```

When defects are modeled it first reruns `Tx/Defect_Model.py` if `Waveform_Payload_DefectsModeled_Tx.h5` is missing or older than `Channel_Defects.json`, so a changed setting (SNR, model, NumberOfRays …) is always used; `--new-channel` reruns it every time (a new random channel unless RandomSeed is set) and `--no-model` never does. It prints the NumberOfAntennas and defect settings it uses. It runs steps 4 to 6 for every antenna, combines the antennas whose packet was found, and runs steps 7 to 9 once on the result. It prints one line per antenna (frequency offset, equalizer SNR) and the combined CRC result. Exit status: 0 all blocks pass, 1 a block failed, 2 no packet on any antenna, 3 a step stopped with an error (rerun with `-v` or `--log FILE`), 4 bad packet (control field CRC failed). With Use_ControlField 1 it runs step 4 on every antenna first, then step 4a once, then steps 5 and 6 per antenna. Other options: `--eq-mode`, `--max-doppler`, `--no-compare`, `--out-dir`, `--input`, `--dry-run`.

By hand, for NumberOfAntennas = 2:

```
python Rx/Decimate_Payload_Rx.py
python Rx/Packet_Detector.py --ray 1      python Rx/Packet_Detector.py --ray 2
python Rx/Demodulate_Waveform_Rx.py --ray 1     ... --ray 2
python Rx/Equalize_Payload_Grid.py --ray 1      ... --ray 2
python Rx/Combine_Antennas_Rx.py
python Rx/Get_DataSigs_QAM.py
python Rx/Get_Demapped_Bitstream.py
python Rx/Get_Decoded_Bitstream.py --compare
```

**File names.** Steps 4 to 6 write one file per antenna, ending in `_Ray1` to `_Ray4` (`--ray` defaults to 1). Steps 7 to 9 read and write the `_Combined` files when the run has more than one antenna and the `_Ray1` files when it has one; `--ray N` makes them use antenna N alone instead (e.g. to compare it with the combined result). The names in File_Rx stay without a suffix; the steps add it. Every output records `Ray` as an attribute (0 = combined). The resource grid and reference signals are shared.

**Combining (step 6b).** Maximum-ratio combining on the equalized grids. For every resource element, each antenna's equalized symbol x_r = Y_r/H_r is weighted by the inverse of the noise left on it, v_r = σ_r²/|H_r|² (both written by Equalize_Payload_Grid.py):

    x = Σ (x_r / v_r) / Σ (1 / v_r),    v = 1 / Σ (1 / v_r)

A subcarrier faded on one antenna is taken from the others, and v, the noise of the combined symbol, goes to the soft demapper, so the soft values stay correctly scaled for every modulation. This is classic MRC with each antenna's own noise level. An antenna whose packet detection failed is left out; with the default inputs, a `_RayN` file older than `Waveform_Payload_Dec_Rx.h5` is also left out (left over from an earlier run). The output records the antennas used (`Rays`), those skipped, and the median per-signal SNR of every antenna and of the combination.

**Timing and frequency offset** are still found per antenna by Packet_Detector.py. A real multi-antenna receiver would estimate them once from all antennas (shared clock and oscillator); per-antenna estimation is simpler and works because MRC uses each antenna's own channel estimate. Packet_Detector.py stops with an error if `--ray` names an antenna the file does not have.

## Settings that change what a step does

| Setting (Defined_Rx) | Values | Step | Effect |
| --- | --- | --- | --- |
| NumberOfAntennas | 1 (default), 2, 3, 4 | 3, 6b-9 | Only when defects are modeled: how many rays of File_Rx.Waveform_Payload_DefectsModeled_Rx are decimated, one row each. More than 1: the antennas are combined (step 6b) and steps 7-9 use the combined result. |
| Use_ControlField | 1 (default), 0 | 4a | 1: the packet has the 4-symbol control field between the preamble and the payload (Tx Use_ControlField 1). It is decoded on every antenna that found the packet (soft values added across antennas, 15 copies added, Viterbi, CRC Gen10) and sets the payload settings in Defined_Rx. Must match the Tx setting. |
| Use_Preamble | 1 (default), 0 | 4 | 1: detect the packet from the preamble, remove the frequency offset (coarse search ±600 kHz) and cut the waveform at the payload start. 0: pass the waveform through (Tx built with Use_Preamble 0). Must match the Tx setting. |
| FFT_StartTime | samples, e.g. −12 | 5 | Where the FFT window starts relative to the end of the cyclic prefix. −12 starts 12 samples early; the phase ramp this causes is removed. |
| EqualizerMode | LMMSE (default), LMMSE_LinearTime, PCA, Linear, None | 6 | LMMSE: LMMSE across frequency with a delay window chosen per packet (start and end from the measured delay profile, so paths that arrive before the timing point are kept), Wiener filter across time. LMMSE_LinearTime: linear interpolation across time. PCA: low-rank LMMSE across frequency using principal components precomputed from channel models (class Short/Medium/Long chosen per packet; a channel that starts more than 2 samples before the timing point is re-centred first), Wiener filter across time. Linear: linear in both. None: no equalization (loopback checks). |
| EqualizerMaxDoppler | Hz, default 300 | 6 | Worst-case Doppler the Wiener time filter is designed for (LMMSE and PCA). |
| Demapping | soft (default), hard | 8 | soft: P(bit = 1) from max-log LLRs using the noise left on each signal after equalization. hard: bits 0/1. |

## Receiving a file

When the Tx sent a file (Tx/Generate_FileData/Generate_File_Bitstream.py), `Receive_File.py` runs after step 9: it reads the decoded blocks of the last run (TransportBlock_Bitstream and CRC_Pass, _Combined or _Ray1 as step 9 wrote them) and keeps every block whose CRC passed in `Rx/FileTransfer_State/` (written in place, so a packet costs only its own blocks, also for large files). Blocks can arrive in any order, in several Rx runs, and more than once. When all blocks of a file are in, it checks the size and SHA-256 sent by the Tx and writes the file under its own name to `Rx/Received_Files/` (an existing file of that name is not overwritten: `_1`, `_2`, ... are added). If the SHA-256 does not match (a wrong block passed its CRC), the file's blocks are discarded so they are sent again. It does nothing (exit 2) if the blocks file is older than the decimated waveform, i.e. the last Rx run found no packet or a bad control field. `--status` shows what is missing, `--reset` forgets partly received files.

`App/Run_File_Transfer.py --file F` does the whole transfer: for every packet Generate_File_Bitstream.py, Run_Tx_Chain.py --keep-blocks (a new channel per packet when defects are modeled), Run_Rx_Chain.py --no-model and Receive_File.py; then up to `--rounds` (default 8) further rounds that resend only the missing blocks. It ends by comparing the received file byte for byte with F (exit 0 identical, 1 blocks still missing: `--resume` sends only those, also after Ctrl+C). Packets run in parallel: `--workers N` (default: CPU cores − 1, at most 8) gives each worker its own copy of the project in `App/FileTransfer_Workers/` with PYTHONPATH pointing at it, and runs the unchanged scripts there; `--workers 1` runs in the project itself. The time per packet is set by the chain (about 5 s at the settings of 22 Oct 2026), so the number of packets decides how long a file takes: at Repetition 11 a packet carries 615 bytes of the file (1 MB ≈ 1700 packets), at Repetition 3 about 2 kB. Other options: `--seed` (channel seed per packet number, the same with any number of workers), `--no-defects`, `--eq-mode`, `--max-packets`, `--no-init`, `--keep-workers`, `-v`; every step's output goes to `Rx/File_Transfer.log` as it runs.

## Timing and the equalizer delay window

Packet_Detector.py times the packet from the strongest path, which in ETU and EVA is often not the first one, so some paths arrive up to 12 samples before the timing point (FFT_StartTime −12 keeps them free of interference). Since 20 Oct 2026 the LMMSE window start follows the measured delay profile (down to −116 samples) and PCA re-centres the channel, so those paths are estimated instead of filtered out. Before that, about one ETU packet in ten failed with LMMSE or PCA while Linear decoded it. Details and measurements: `Flexlink_BasicModules/Equalizer/pilot_ce/README.md`, section *Timing on a later path*.

## PCA bases (EqualizerMode PCA)

`Flexlink_BasicModules/Equalizer/PCA/Train_PCA_Basis.py` builds `PCA_Basis_S3/6/12/24.h5` in the same folder from `PCA_Training_Profiles.json` (channel models EPA/EVA/ETU grouped into the classes Short ≤ 16, Medium ≤ 64 and Long ≤ 116 samples of delay). Run it once, and again after changing the profiles or the reference-symbol layouts. Step 6 stops with an error if the basis file is missing or was built for other pilot positions.

## Test options

- `Get_DataSigs_QAM.py --unequalized` reads File_Rx.Grid_Payload_Rx instead of the equalized grid (skips step 6). Soft demapping then falls back to one estimated noise value for the packet.
- `Get_Decoded_Bitstream.py --compare` compares the decoded transport blocks with Tx/Generate_RandomPacketOfData/Bitstream_TransportBlks_Tx.h5.
- `Diagnose_Defects.py` runs Tx/Defect_Model.py and the whole chain (through Run_Rx_Chain.py) for each defect alone, none and all, and prints the equalizer SNR per antenna, the SNR after combining, the CRC per block and the bit errors. `--antennas 1 2 4` compares antenna counts on the same channels (the diversity gain); `--per-ray` adds each antenna decoded alone. Channel_Defects.json, the Rx json and the files in Tx/ and Rx/ are not changed (temporary copies and folder). Other options: `--seeds N` or `--seed-list`, `--cases`, `--all-only`, `--all-but`, `--no-baselines`, `--max-doppler`, `--eq-mode`, `--demap-noise-var`, `--keep FOLDER` (keep the files and logs), `--dry-run`.
- For a loopback test, File_Rx.Waveform_Payload_Rx can name the Tx output Waveform_Payload_Tx.h5: step 3 looks for it in Rx/ first, then in Tx/.
- `Decimate_Payload_Rx.py --decimator simple|filter` overrides the 40 → 20 MHz method. The default `auto` keeps every other sample for a clean waveform and low-pass filters first when defects are modeled, so the noise above 10 MHz does not fold into the band. `--defects-file` reads the Mode flags from another Channel_Defects.json.

The scripts in `Rx/Decode/` can also run on their own as a test chain, ending with `Check_TransportBlk.py`, which compares File_Rx.TestVector_TransportBlk_Rx with `Tx/Encode/Sequence_TransportBlk_Tx.h5`. That chain is the counterpart of the `Tx/Encode/` test chain and is separate from the payload chain above, which only borrows their functions in step 9.
