# RTL simulation against the bit-true model

`tb_pd_detect.v` feeds a test vector into `pd_detect_top`. Gaps in `tvalid` are random, of 0 to `GAP` clocks. The testbench checks the following against the bit-true model's expected outputs, which `make_tb_vectors.py` writes:
- **every** LPF output sample;
- **every** debug record (metric and flag bits);
- **every** trigger;
- **every** CFO result: window start, the four lag accumulators, the four CORDIC angles and `cfo_inc`;
- **every** event after the tone check: its CFO fields, tone `num` and `den`, and the abort flag;
- **every** Preamble B timing record (input stages 1 and 2): `n_end`, status (peak check, late trigger, busy), `n_pos`, `p_max`, `p_sum`. Records are matched by `t_trig`, because a BUSY record can come out before the result of an earlier search.
- **every** control record (input stages 1 and 2): CRC result, message, metric, Viterbi scaling, in order. `MAX=213860 ./run_sim.sh` ends the input just after the last control field, so the decoding finishes after the input (checks `busy`).

It obeys `tready`: with fast input the tone check holds the input back, and so does a running timing search (one sample per 5 clocks); the log says for how many clocks. At exactly 5 clocks per sample (`FIXGAP=1 GAP=4`) nothing stalls. Inputs change on the falling clock edge, so the testbench never races the DUT.

`tb_pd_timing.v` tests the Preamble B timing block alone (`./run_sim.sh timing`) on directed cases from `make_tm_vectors.py`: samples exactly on the de-rotator's rounding boundary, the late-trigger and one-search-at-a-time limits exactly and one sample off, full-scale samples, no Preamble B, failed tone checks, silent windows (all P equal), N_CFO 128 / 4096 and T_pk 0 / 60.

`tb_pd_units.v` runs directed unit tests:
- the CORDIC on 22025 vectors: random values plus ±π, the axes, zero, and tiny and full-scale values;
- the cascade on 101000 angle sets, including sets that land exactly on each rounding boundary and one LSB either side;
- the tone check on 160 windows: random full scale, single tones exactly on a bin (the largest DFT values), Preamble-A-like tones, silence, DC, Nyquist, tiny signals; f̂ at 0, ±2²³ and 2²⁴−1; N 128 / 2048 / 4096; thresholds 0, 24 dB, random, and exactly on and one LSB past the decision.

```bash
cd Flexlink_py/HW_Development/Packet_Detector/SIM
source /tools/Xilinx/2025.2/Vivado/settings64.sh      # for xsim
source ~/SiskinRF/Flexlink_py/.venv/bin/activate       # for make_tb_vectors.py (numpy)

./run_sim.sh                                  # rep3 (3 packets), input_stage 1, gaps 0..3, xsim
./run_sim.sh ../TestVectors/rep3.json 0       # input_stage 0: LPF output in, front end bypassed
./run_sim.sh stress 1                         # synthetic full-scale stress vector, 34 triggers
./run_sim.sh cfo 1                            # 40 real packets, CFO ±600 kHz, SNR -6..30 dB: 80 CFO results
./run_sim.sh cfo 2                            # the same packets at 40 Msps: half-band in front
./run_sim.sh highsnr 2                        # 30 packets at +40..+65 dB and noise-free: the power floor
./run_sim.sh units                            # CORDIC, cascade and tone-check unit tests
GAP=0 ./run_sim.sh stress 0                   # one sample per clock (the tone check stalls the input)
FIXGAP=1 GAP=4 ./run_sim.sh stress 1          # exactly 5 clocks per sample = 20 Msps: no stalls
NCFO=4096 GAP=0 ./run_sim.sh cfo 1            # other CFO window length (128..4096)
HOLDOFF=300 ./run_sim.sh                      # short re-arm (effective >= N_CFO): 9 triggers on rep3
SIM=verilator ./run_sim.sh                    # Verilator instead of xsim
```

Each run ends with `PASS: … match the bit-true model`, or `FAIL` and the first 10 mismatches. The exit status is 0 only on PASS. Each run writes its expected-output files to `work/` and its log to `sim.log`; both are throw-away.

The **stress** vector is synthetic and drives every word length to its limit:
- full-scale DC on both rails, and opposite DC on I and Q
- a full-scale square wave, random full-scale samples, and a full-scale Nyquist tone
- silence, which gives `den = 0`
- clipped Preamble-A-like tones
- small noise

It covers saturation of the DC blocker and LPF, the largest products and running sums, a saturating metric (65535) and many triggers.

The **cfo** vector is 40 real packets from `Tx/Waveform_Payload_Tx.h5`, each with its own CFO (±600 kHz) and SNR. Its 4000-sample holdoff re-arms inside Preamble A, so the second window of each packet runs into Preamble B. That gives a wide spread of angles. Building it needs the simulator project and h5py, as `pd_bittrue_check.py` does.

At the end the testbench waits until `busy` is low, then checks the window still open (the model's truncated event, if any). Every window that ends inside the vector gives a CFO result and an event.

Typical xsim run time is about 1 minute for rep3 (300k samples). Use `MAX=20000` for a quick check.

## Control-field receiver (`run_cf.sh`)

`tb_cf_rx.v` runs `RTL/cf_rx.v` on the cases from `make_cf_vectors.py` and compares **every**
FFT output (4 × 841 grid values), **every** channel-filter output (Hf0, H3′ for 841 tones),
**every** soft value after descrambling (2200), the drift angle φ, the Viterbi scaling, the 70
decoded bits, the path metric and the CRC result with `PC/cf_reference.py`. The samples come with
random gaps. Cases: the three rep3 packets, the same with added noise (CRC passes and fails),
×8 clipped (saturation), all zero (only Viterbi ties), random full scale, a full-scale tone on one
bin, and cfo_inc = half the sample rate. `tb_cf_viterbi.v` runs `cf_viterbi` + `cf_crc10` alone
on 400 cases (random, tiny values = ties, noisy valid code words, zeros).

```bash
./run_cf.sh                    # vectors + all cases (xsim if Vivado is sourced, else Verilator)
GAP=0 ./run_cf.sh              # one sample per clock
SIM=verilator NOVEC=1 ./run_cf.sh
```

Ends with one `PASS` line per case; the exit status is 0 only if all pass. Log: `cf_sim.log`.

## Firmware + RTL co-simulation (`cosim/`)

`cosim/build.sh` compiles the board firmware `FW/pd_server.c` against small stand-ins for lwIP
(a real TCP socket) and links it with the Verilated `RTL/pd_axi.v` and an AXI DMA model
(`pl_model.cpp`): register accesses become AXI-Lite transactions, DMA transfers stream through
the RTL clock by clock, with random gaps on MM2S and random `tready` on S2MM. The PC scripts then
talk to it exactly as to the board. Needs Verilator 5.

```bash
cd SIM/cosim && ./build.sh
./obj/fw_cosim 5101 &                                   # the "board" on port 5101
python ../../PC/pd_hw_test.py ../../TestVectors/rep3.json --host 127.0.0.1 --port 5101 --input-stage 1 --debug
python ../../PC/pd_board_emulator.py --port 5102 &      # reference behaviour
python compare_with_emulator.py                         # six sequences, every reply must match
```

Environment for `fw_cosim`: `MM2S_P` / `S2MM_P` (stream duty, default 0.8 / 0.7), `BUSY_TICKS`
(clocks per DMA poll, default 64; 20000 lets the event FIFO fill up), `SEED`.
