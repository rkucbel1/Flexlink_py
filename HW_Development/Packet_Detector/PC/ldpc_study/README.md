# LDPC float-vs-bit-true study

Decides the payload LDPC decoder's algorithm, iterations and widths before any RTL
(`Docs/Specification_PayloadInput.md` §5b). AWGN, all-zero code word through a random
scrambler (symmetrised), the simulator's `qam_map` and max-log `QAM_Demapper`, the 802.11n
codes from `get_p_ldpc`.

    cd HW_Development/Packet_Detector/PC/ldpc_study
    python study.py 648_12 QPSK 2          # SNR for 10 % BLER, every decoder variant (~1-3 min)
    while read l; do python study.py $l >> results/snr10.jsonl; done < configs.txt   # all 32 (~1 h)
    python verify.py 0; python verify.py 1  # chosen formats at the 10 % point and +0.5 dB
    python cscan.py 0;  python cscan.py 1   # LLR step scan per configuration

`ldpc_lib.py`: `Code`, `channel_llr`, `beliefs_sim` + `decode_flood` (identical to the
simulator's decoder on 100/100 code words), `decode_layered` (float, normalised min-sum
α = 0.75), `decode_layered_fixed` (integer; α as m − m>>2), `quantise`.
`FLEXLINK_MODULES` overrides the path to `Flexlink_BasicModules`.
`results/` holds the runs the spec quotes.
