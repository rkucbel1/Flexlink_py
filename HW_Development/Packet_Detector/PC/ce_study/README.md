# Payload channel-estimate study (float)

This study backs the payload channel-estimate choice: window-select FIR8 in frequency plus
fixed smoothing across reference rows in time. The findings are in
`Docs/Specification_PayloadInput.md` §6a.

`pl_study.py` sends a payload grid (all-zero LDPC code words, rate-matched, interleaved,
scrambled, QAM-mapped, with antipodal pilots) through `Delay_Doppler_Channel` at 20 Msps. It
then runs each estimator and decodes with the frozen layered min-sum decoder
(`../ldpc_study/ldpc_lib.py`). The estimators are:
- genie;
- the simulator's LMMSE, with Wiener or linear time interpolation;
- FIR variants;
- window-select (SEL).

Needs numpy and h5py, and `Flexlink_BasicModules` (found from the repo layout, or set with
`FLEXLINK_MODULES`).

    # fixed SNR points:  mod code rep RefSymbolSpacing RefSignalSpacing channel snrs n
    python3 pl_study.py QPSK 648_12 1 3 3 EVA-70 2,3,4 40
    # SNR sweep to 10 % BLER, appends to results/$PLOUT (default pl_sweep.jsonl):
    #   ... channel n start select_taps|- variants step [smoothing for SEL: 0,3,5,9]
    python3 pl_study.py sweep QPSK 648_12 1 1 3 ETU-300 80 -4 8 FIR8 1.0 0,3,5,9

Variant names:

| Name | Meaning |
| --- | --- |
| FIR8, FIR16 | Delay window −20 … 12 |
| …L | Longest alias-free window |
| …N | Design noise from a class of the measured σ² |
| …T3, …T5, …T9 | Smoothing across 3, 5 or 9 reference rows |
| SEL8 | Window chosen per packet from 12 / 28 / 52 / 100 |
| SEL8T5 | SEL8 plus 5-row smoothing |

The configurations that were run are in `configs.txt` (the main matrix; the 12 runs missing from
`results/pl_sweep.jsonl` hit the time limit) and `configs_smoothing.txt`
(`results/pl_sweep2.jsonl`). One QPSK configuration takes 3–10 minutes; 64QAM on ETU takes
about 35.
