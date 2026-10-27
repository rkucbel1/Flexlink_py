# RTL simulation against the bit-true model

`tb_pd_detect.v` feeds a test vector into `pd_detect_top`. Gaps in `tvalid` are random, of 0 to `GAP` clocks. The testbench checks the following against the bit-true model's expected outputs, which `make_tb_vectors.py` writes:
- **every** LPF output sample;
- **every** debug record (metric and flag bits);
- **every** trigger;
- **every** CFO result: window start, the four lag accumulators, the four CORDIC angles and `cfo_inc`.

`tb_pd_units.v` runs directed unit tests:
- the CORDIC on 22025 vectors: random values plus ±π, the axes, zero, and tiny and full-scale values;
- the cascade on 101000 angle sets, including sets that land exactly on each rounding boundary and one LSB either side.

```bash
cd Flexlink_py/HW_Development/Packet_Detector/SIM
source /tools/Xilinx/2025.2/Vivado/settings64.sh      # for xsim
source ~/SiskinRF/Flexlink_py/.venv/bin/activate       # for make_tb_vectors.py (numpy)

./run_sim.sh                                  # rep3 (3 packets), input_stage 1, gaps 0..3, xsim
./run_sim.sh ../TestVectors/rep3.json 0       # input_stage 0: LPF output in, front end bypassed
./run_sim.sh stress 1                         # synthetic full-scale stress vector, 34 triggers
./run_sim.sh cfo 1                            # 40 real packets, CFO ±600 kHz, SNR -6..30 dB: 80 CFO results
./run_sim.sh units                            # CORDIC + cascade unit tests
GAP=0 ./run_sim.sh stress 0                   # one sample per clock
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

Windows that end within the last 32 samples of a vector aren't expected: the RTL lane runs 32 samples behind its input.

Typical xsim run time is about 1 minute for rep3 (300k samples). Use `MAX=20000` for a quick check.
