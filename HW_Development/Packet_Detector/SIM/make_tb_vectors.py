#!/usr/bin/env python3
"""
make_tb_vectors.py  (Flexlink_py/HW_Development/Packet_Detector/SIM)

Exports a test vector (TestVectors/<name>.json + .bin / _dec.bin) as text files for the RTL
testbench, with the bit-true model's expected outputs for every sample:

  work/<name>_s<stage>/cfg.txt       input_stage dc_shift eta_hi_sq eta_lo_sq qual_len
                                     blank_len arm_holdoff n_samples n_trig n_cfo n_cfo_events
                                     tone_lin n_out (detector samples: n_samples / 2 with stage 2)
  work/<name>_s<stage>/in.txt        input samples, "iiii qqqq" (16-bit hex, two's complement)
  work/<name>_s<stage>/exp_lpf.txt   detector input (LPF output), "iii qqq" (12-bit hex)
  work/<name>_s<stage>/exp_dbg.txt   per sample "mmmm bbbb": dbg_metric, dbg_bits (hex)
  work/<name>_s<stage>/exp_trig.txt  t_trig of each trigger (decimal)
  work/<name>_s<stage>/exp_cfo.txt   per completed CFO window (hex): t, A16 re im, A32 re im,
                                     A128 re im, A256 re im (37-bit), ang16 ang32 ang128 ang256,
                                     cfo_inc (32-bit). Every window that ends inside the vector.
  work/<name>_s<stage>/exp_tone.txt  per event, same windows (hex): t, cfo_inc, the four angles,
                                     tone num, tone den (40-bit), fail (0/1)
  work/<name>_s<stage>/exp_open.txt  "n t": windows still open at the end (truncated, 0 or 1)
                                     and the t_trig of the first
  work/<name>_s<stage>/exp_tm.txt    Preamble B timing records (hex, any order): t, n_end, status,
                                     n_pos, p_max, p_sum. cfg.txt then also has t_pk and the
                                     number of records
  work/<name>_s<stage>/exp_cf.txt    control records (count, then hex in order): t, n_end,
                                     status (bit0 CRC ok), shift, metric, message (54 bits)

    python make_tb_vectors.py ../TestVectors/rep3.json --input-stage 1
    python make_tb_vectors.py ../TestVectors/rep3.json --input-stage 0 --max-samples 50000
    python make_tb_vectors.py stress --input-stage 1      # synthetic full-scale stress vector
    python make_tb_vectors.py cfo --n-cfo 4096            # override the CFO window length
    python make_tb_vectors.py cfo --input-stage 1         # 40 real packets, CFO +-600 kHz, SNR -6..30 dB
    python make_tb_vectors.py cfo --input-stage 2         # the same at 40 Msps (half-band decimator)
    python make_tb_vectors.py highsnr --input-stage 2     # 30 packets at +40..+65 dB and noise-free
    python make_tb_vectors.py units                       # CORDIC, cascade, tone-check unit tests
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import fields
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
TV = HERE.parent / "TestVectors"
sys.path.insert(0, str(HERE.parent / "PC"))
import pd_protocol as pr  # noqa: E402
import pd_reference as ref  # noqa: E402


def stress_vector(seed: int = 7) -> np.ndarray:
    """Synthetic 12-bit input that drives every word length to its limit."""
    rng = np.random.default_rng(seed)
    n = 16000
    t = np.arange(n)
    seg = [
        np.full((n, 2), -2048),                                            # full-scale DC (both rails)
        np.where((t // 32) % 2 == 0, 2047, -2048)[:, None].repeat(2, 1),  # square wave, period 64
        rng.integers(-2048, 2048, (n, 2)),                                 # random full scale
        np.zeros((n, 2), np.int64),                                        # silence (den = 0, settling)
        np.stack([np.clip(np.round(3000 * (np.cos(2 * np.pi * t / 32 + np.pi / 4)
                                           + np.cos(2 * np.pi * 3 * t / 32 + 3 * np.pi / 4))), -2048, 2047),
                  np.clip(np.round(3000 * (np.sin(2 * np.pi * t / 32 + np.pi / 4)
                                           + np.sin(2 * np.pi * 3 * t / 32 + 3 * np.pi / 4))), -2048, 2047)],
                 axis=1),                                                  # clipped Preamble-A-like tones
        np.stack([np.full(n, 2047), np.full(n, -2048)], axis=1),          # opposite DC on I and Q
        np.where(t % 2 == 0, 2047, -2048)[:, None] * np.array([[1, -1]]),  # Nyquist, full scale
        rng.integers(-40, 41, (n, 2)),                                     # small noise
    ]
    return np.clip(np.concatenate(seg), -2048, 2047).astype(np.int64)


def cfo_sweep_vector(seed: int = 11, n_packets: int = 40, rate40: bool = False) -> np.ndarray:
    """Real packets (Tx/Waveform_Payload_Tx.h5) back to back, each with its own CFO (uniform
    +-600 kHz) and SNR (uniform -6..30 dB), decimated and quantised as in pd_bittrue_check.
    rate40: the same packets at 40 Msps, quantised before decimation (input stage 2)."""
    import pd_bittrue_check as chk
    rng = np.random.default_rng(seed)
    tx = chk.load_tx(chk.find_root(HERE))
    recs = []
    for _ in range(n_packets):
        x20, _, _, x40 = chk.make_trials(tx, float(rng.uniform(-6, 30)), 1, rng, cfo_max=600e3, return40=True)
        recs.append(chk.quantise(x40 if rate40 else x20)[:, :, 0])
    return np.concatenate(recs).astype(np.int64)


def high_snr_vector(seed: int = 13, per_snr: int = 6, rate40: bool = False) -> np.ndarray:
    """Real packets at very high SNR (+40, +55, +60, +65 dB and noise-free), quantised per
    packet at -12 dBFS: the idle noise between them is ~0.5 LSB or less, the case the power
    floor (pd_reference.E_MIN) is for. rate40: at 40 Msps, quantised before decimation."""
    import pd_bittrue_check as chk
    rng = np.random.default_rng(seed)
    tx = chk.load_tx(chk.find_root(HERE))
    recs = []
    for snr in (40, 55, 60, 65, 300):
        for _ in range(per_snr):
            x20, _, _, x40 = chk.make_trials(tx, snr, 1, rng, cfo_max=600e3, return40=True)
            recs.append(chk.quantise(x40 if rate40 else x20)[:, :, 0])
    return np.concatenate(recs).astype(np.int64)


def cf_waveform(message: int) -> np.ndarray:
    """The 4560-sample control field (20 Msps, complex, unit-ish power) carrying `message`
    (54 bits, bit i = message bit i), built as the Tx does: CRC10, K = 7 code with 6 tail bits,
    BCC interleaver, 15 copies + 100 zero bits, Gold scrambler (130), BPSK, Ref_P0 pilots in
    symbols 0 and 3, 1024-point IFFT, CP 116."""
    import cf_reference as cf
    bits = [(message >> i) & 1 for i in range(54)]
    crc = cf.crc10(bits)
    data = bits + [(crc >> (9 - i)) & 1 for i in range(10)] + [0] * 6
    st, code = 0, []
    for b in data:
        oa, ob = cf._OUT[st][b]
        code += [oa, ob]
        st = (b << 5) | (st >> 1)
    code = np.array(code, np.int64)
    a = code[cf.CF_IDX]                                    # Rx: code[IDX[j]] = A[j]
    tx = np.concatenate([np.tile(a, 15), np.zeros(100, np.int64)]) ^ cf.CF_GOLD
    grid = np.zeros((4, 841), complex)
    grid[cf.CF_DATA_MASK] = 2.0 * tx - 1.0
    grid[0, cf.CF_P0_K] = cf.CF_REF
    grid[3, cf.CF_P0_K] = cf.CF_REF
    out = []
    for s_ in range(4):
        spec = np.zeros(1024, complex)
        spec[(np.arange(841) - 420) % 1024] = grid[s_]
        t = np.fft.ifft(spec) * 32
        out.append(np.r_[t[-cf.CF_CP:], t])
    return np.concatenate(out)


def cfmod_vector(seed: int = 21) -> tuple[np.ndarray, list[str]]:
    """rep3 (decimated input) with each packet's control field replaced, to exercise every
    re-arm decision after a good timing record:
      packet 1  valid CRC, CBS_A_Flag code 11 (no meaning): rejected
      packet 2  noise only: CRC fails, rejected
      packet 3  valid, PacketLength 37 instead of 73: re-arm inside the real payload
    The new control field gets the packet's CFO (from its event) and noise, at the level of
    the samples it replaces."""
    import cf_reference as cf
    meta = json.loads((TV / "rep3.json").read_text())
    known = {f.name for f in fields(pr.Params)}
    p = pr.Params(**{k: v for k, v in meta["params"].items() if k in known})
    p.input_stage, p.rearm = 1, 0
    iq = np.fromfile(TV / meta["files"]["decimated"], "<i2").reshape(-1, 2).astype(np.int64)
    res = ref.detect(iq, p, control=False)
    rng = np.random.default_rng(seed)
    base = cf.control_records(ref.dc_blocker(iq, p.dc_shift), res.events, res.timing)
    if len(base) != 3 or not all(c.crc_ok for c in base):
        raise SystemExit("cfmod: rep3 should give 3 good control fields")
    msg0 = base[0].message

    def with_code(m, pos, w, code):
        for i in range(w):
            bit = (code >> (w - 1 - i)) & 1
            m = (m & ~(1 << (pos + i))) | (bit << (pos + i))
        return m
    plan = [("CBS_A_Flag 11 (no meaning)", with_code(msg0, 10, 2, 0b11)),
            ("noise only (CRC fails)", None),
            ("PacketLength 37", with_code(msg0, 4, 3, 0b011))]
    inc = {e.t_trig: e.cfo_inc for e in res.events}
    out = iq.copy()
    notes = []
    for (what, m), tm in zip(plan, [r for r in res.timing if r.status == pr.TM_GOOD]):
        a, b = tm.n_end + 1, tm.n_end + 1 + cf.CF_LEN
        seg = iq[a:b, 0] + 1j * iq[a:b, 1]
        lvl = np.sqrt(np.mean(np.abs(seg) ** 2))
        noise = (rng.standard_normal(cf.CF_LEN) + 1j * rng.standard_normal(cf.CF_LEN)) / np.sqrt(2)
        if m is None:
            z = lvl * noise
        else:
            w = cf_waveform(m)
            ph = 2 * np.pi * (-(inc[tm.t_trig])) / 2 ** 24 * np.arange(a, b)
            w = w / np.sqrt(np.mean(np.abs(w) ** 2)) * np.exp(1j * ph)
            z = lvl * (0.94 * w + 0.33 * noise)             # about +9 dB
        out[a:b] = np.clip(np.round(np.stack([z.real, z.imag], 1)), -2048, 2047).astype(np.int64)
        notes.append(f"t_trig {tm.t_trig}: {what}")
    return out, notes


def units_vectors(out_root: Path, seed: int = 3) -> Path:
    """CORDIC vectors (random + edge cases) and cascade angle sets (random + exact rounding
    boundaries and one LSB either side) with the model's results."""
    rng = np.random.default_rng(seed)
    m37, lo, hi = (1 << 37) - 1, -(1 << 36), (1 << 36) - 1
    vec = [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1), (-5, 0), (-5, -1), (-5, 1), (1, 1), (-1, -1),
           (hi, 0), (lo, 0), (0, hi), (0, lo), (hi, hi), (lo, lo), (lo, hi), (hi, lo), (lo, -1), (lo, 1),
           (3, -4), ((1 << 22) - 1, 1), (1 << 22, -1), ((1 << 21), (1 << 21)), (-(1 << 23), 7)]
    for _ in range(20000):
        mag = 10 ** rng.uniform(0, 10.8)
        a = rng.uniform(-np.pi, np.pi)
        vec.append((int(np.clip(round(mag * np.cos(a)), lo, hi)), int(np.clip(round(mag * np.sin(a)), lo, hi))))
    for _ in range(2000):                                     # small integers: exact angle edges
        vec.append((int(rng.integers(-40, 41)), int(rng.integers(-40, 41))))
    out = out_root / "units"
    out.mkdir(parents=True, exist_ok=True)
    (out / "cordic.txt").write_text("".join(
        f"{r & m37:010x} {i & m37:010x} {ref.cordic_angle(r, i) & 0xFFFF:04x}\n" for r, i in vec))

    def wrap16(v):
        v &= 0xFFFF
        return v - 0x10000 if v >= 0x8000 else v

    sets = [tuple(int(a) for a in rng.integers(-32768, 32768, 4)) for _ in range(20000)]
    for _ in range(3000):                                     # exact rounding boundaries
        a16 = int(rng.integers(-32768, 32768))
        f16 = 16 * a16
        for d32 in (-1, 0, 1):
            a32 = wrap16(2 * a16 + 32768 + d32)               # 8 a32 = f16 + 2^18 (mod 2^19)
            f32 = 8 * a32 + ((16 * a16 - 8 * a32 + (1 << 18)) >> 19 << 19)
            for d128 in (-1, 0, 1):
                a128 = wrap16((f32 + (1 << 16)) // 2 + d128)  # 2 a128 = f32 + 2^16 (mod 2^17)
                f128 = 2 * a128 + ((f32 - 2 * a128 + (1 << 16)) >> 17 << 17)
                for d256 in (-1, 0, 1):
                    sets.append((a16, a32, a128, wrap16(f128 + (1 << 15) + d256)))
    (out / "cascade.txt").write_text("".join(
        f"{a & 0xFFFF:04x} {b & 0xFFFF:04x} {c & 0xFFFF:04x} {d & 0xFFFF:04x} "
        f"{-ref.cfo_cascade({16: a, 32: b, 128: c, 256: d}) & 0xFFFFFFFF:08x}\n" for a, b, c, d in sets))
    n_tone = tone_vectors(out, rng)
    print(f"{out}: {len(vec)} CORDIC vectors, {len(sets)} cascade angle sets, {n_tone} tone-check windows")
    return out


def tone_vectors(out: Path, rng) -> int:
    """Tone-check windows: header "N cfo_inc lin num den fail" (hex), then N samples "iii qqq".
    Random, full-scale and silent windows, single tones on a bin (largest DFT values), f_hat at
    the wrap points, N = 128 / 2048 / 4096, and thresholds on and just past the decision."""
    m24 = 1 << 24
    lin_max = ref.tone_thresh_lin(ref.TONE_THRESH_MAX_Q8)
    cases = []

    def tone(n, f, amp=2047.0, ph=0.0):
        k = np.arange(n)
        return amp * np.exp(1j * (2 * np.pi * f * k / m24 + ph))

    def q12(z):
        return np.stack([np.clip(np.round(z.real), -2048, 2047), np.clip(np.round(z.imag), -2048, 2047)],
                        axis=1).astype(np.int64)

    for i in range(64):
        n = (128, 2048, 4096, int(rng.integers(128, 4097)))[i % 4]
        f = int(rng.integers(-(1 << 23), 1 << 23))
        if i in (0, 1, 2, 3):
            f = (0, 1 << 23, m24 - 1, -(1 << 23))[i]
        kind = i % 8
        if kind == 0:
            x = rng.integers(-2048, 2048, (n, 2))                          # random full scale
        elif kind == 1:
            m = int(rng.choice(ref.TONE_OFFSETS + ref.NOISE_REF_OFFSETS))
            x = q12(tone(n, f + m, 2047.0 * np.sqrt(2), rng.uniform(0, 6.3)))   # one bin, clipped corners
        elif kind == 2:
            x = q12(sum(tone(n, f + o, 700.0, rng.uniform(0, 6.3)) for o in ref.TONE_OFFSETS)
                    + rng.normal(0, 30, n) + 1j * rng.normal(0, 30, n))    # Preamble-A-like + noise
        elif kind == 3:
            x = np.zeros((n, 2), np.int64)                                # silence: num = den = 0
        elif kind == 4:
            x = np.full((n, 2), -2048) if i % 16 == 4 else np.tile([[2047, -2048]], (n, 1))  # DC
        elif kind == 5:
            x = rng.integers(-1, 2, (n, 2))                               # tiny: no shift
        elif kind == 6:
            x = q12(tone(n, int(rng.integers(0, m24)), 1500.0) + rng.normal(0, 200, n)
                    + 1j * rng.normal(0, 200, n))                         # CW off the bins + noise
        else:
            x = np.where((np.arange(n) % 2 == 0)[:, None], [[2047, 2047]], [[-2048, -2048]])  # Nyquist
        x = np.asarray(x, np.int64)
        num, den, _, _ = ref.tone_check(x[:, 0], x[:, 1], f)
        lins = [0, lin_max, int(rng.integers(1, lin_max + 1))]
        if den > 0 and (num << 16) // den < lin_max:
            b = (num << 16) // den                                        # b * den <= num << 16
            lins += [b, b + 1]
        for lin in lins[(i % 3):(i % 3) + 1] + lins[3:]:
            fail = int(lin != 0 and (num << 16) < lin * den)
            cases.append((n, -f, lin, num, den, fail, x))
    with open(out / "tone.txt", "w") as fh:
        for n, inc, lin, num, den, fail, x in cases:
            fh.write(f"{n:04x} {inc & 0xFFFFFFFF:08x} {lin:06x} {num:010x} {den:010x} {fail:x}\n")
            fh.write("".join(f"{a & 0xFFF:03x} {b & 0xFFF:03x}\n" for a, b in x))
    return len(cases)


def export(vector: Path, stage: int, max_samples: int | None, out_root: Path, holdoff: int | None,
           n_cfo: int | None = None, rearm: int | None = None) -> Path:
    if str(vector) == "units":
        return units_vectors(out_root)
    if str(vector) == "cfo":
        p = pr.Params(arm_holdoff=4000)
        p.input_stage = stage
        if stage == 2:
            iq = cfo_sweep_vector(rate40=True)
        else:
            iq_dec = cfo_sweep_vector()
            iq = iq_dec if stage == 1 else ref.front_end(iq_dec, p.dc_shift)
        vector = Path("cfo.json")
    elif str(vector) == "highsnr":
        p = pr.Params(arm_holdoff=4000)
        p.input_stage = stage
        if stage == 2:
            iq = high_snr_vector(rate40=True)
        else:
            iq_dec = high_snr_vector()
            iq = iq_dec if stage == 1 else ref.front_end(iq_dec, p.dc_shift)
        vector = Path("highsnr.json")
    elif str(vector) == "cfmod":
        if stage != 1:
            raise SystemExit("cfmod: input stage 1 only")
        p = json.loads((TV / "rep3.json").read_text())["params"]
        known = {f.name for f in fields(pr.Params)}
        p = pr.Params(**{k: v for k, v in p.items() if k in known})
        p.input_stage = 1
        iq, notes = cfmod_vector()
        print("cfmod: " + "; ".join(notes))
        vector = Path("cfmod.json")
    elif str(vector) == "stress":
        p = pr.Params(arm_holdoff=500)
        p.input_stage = stage
        # every stage gets the same pattern: straight into the detector (0), into the front end
        # at 20 Msps (1) or into the half-band at 40 Msps (2)
        iq = stress_vector()
        vector = Path("stress.json")
    else:
        meta = json.loads(vector.with_suffix(".json").read_text())
        known = {f.name for f in fields(pr.Params)}
        p = pr.Params(**{k: v for k, v in meta["params"].items() if k in known})
        p.input_stage = stage
        key = {0: "lpf_out", 1: "decimated", 2: "input40"}[stage]
        if key not in meta["files"]:
            raise SystemExit(f"{vector.name} has no {key} file - rebuild it with PC/pd_make_vector.py")
        name = meta["files"][key]
        iq = np.fromfile(vector.with_name(name), dtype="<i2").reshape(-1, 2).astype(np.int64)
    if holdoff is not None:
        p.arm_holdoff = holdoff
    if rearm is not None:
        p.rearm = rearm
    if n_cfo is not None:
        p.n_cfo = n_cfo
    if max_samples:
        iq = iq[:max_samples]

    if stage == 2 and iq.shape[0] % 2:
        iq = iq[:-1]                                   # whole input pairs
    lpf = ref.front_end(ref.halfband(iq) if stage == 2 else iq, p.dc_shift) if stage else iq
    res = ref.detect(iq, p, debug=True)
    trig = [e.t_trig for e in res.events]
    n = lpf.shape[0]
    xi, xq = lpf[:, 0].astype(np.int64), lpf[:, 1].astype(np.int64)
    cfo_lines, tone_lines = [], []
    lin = ref.tone_thresh_lin(p.tone_thresh) if p.tone_thresh > 0 else 0
    for e, x in zip(res.events, res.extra):
        if not e.status & pr.EV_CFO_VALID:
            continue
        acc = ref.cfo_accumulators(xi, xq, e.t_trig, p.n_cfo)
        m37 = (1 << 37) - 1
        vals = [f"{e.t_trig & 0xFFFFFFFF:08x}"]
        for lag in (16, 32, 128, 256):
            vals += [f"{acc[lag][0] & m37:010x}", f"{acc[lag][1] & m37:010x}"]
        vals += [f"{a & 0xFFFF:04x}" for a in (e.ang16, e.ang32, e.ang128, e.ang256)]
        vals.append(f"{e.cfo_inc & 0xFFFFFFFF:08x}")
        cfo_lines.append(" ".join(vals) + "\n")
        m40 = (1 << 40) - 1
        tone_lines.append(f"{e.t_trig & 0xFFFFFFFF:08x} {e.cfo_inc & 0xFFFFFFFF:08x} "
                          + " ".join(f"{a & 0xFFFF:04x}" for a in (e.ang16, e.ang32, e.ang128, e.ang256))
                          + f" {x['tone_num'] & m40:010x} {x['tone_den'] & m40:010x}"
                          + f" {1 if e.status & pr.EV_TONE_FAIL else 0}\n")

    out = out_root / f"{vector.stem}_s{stage}"
    out.mkdir(parents=True, exist_ok=True)
    (out / "cfg.txt").write_text(
        f"{p.input_stage} {p.dc_shift} {p.eta_hi_sq} {p.eta_lo_sq} {p.qual_len} {p.blank_len} "
        f"{p.arm_holdoff} {iq.shape[0]} {len(trig)} {p.n_cfo} {len(cfo_lines)} {lin} {lpf.shape[0]} "
        f"{p.t_pk} {len(res.timing)} {p.rearm}\n")
    (out / "exp_tm.txt").write_text("".join(
        f"{r.t_trig:08x} {r.n_end:08x} {r.status:x} {r.n_pos:04x} {r.p_max:09x} {r.p_sum:012x}\n"
        for r in res.timing))
    (out / "exp_cf.txt").write_text(f"{len(res.control)}\n" + "".join(
        f"{c.t_trig:08x} {c.n_end:08x} {c.status:x} {c.shift:02x} {c.metric & 0xFFFF:04x} {c.message:014x}\n"
        for c in res.control))
    (out / "exp_pl.txt").write_text(f"{len(res.payload)}\n" + "".join(
        f"{r.t_trig:08x} {r.sym:02x} {r.sig_a:08x} {r.sig_b:08x}\n" for r in res.payload))
    u16 = (iq & 0xFFFF)
    np.savetxt(out / "in.txt", u16, fmt="%04x")
    np.savetxt(out / "exp_lpf.txt", lpf & 0xFFF, fmt="%03x")
    np.savetxt(out / "exp_dbg.txt", np.stack([res.metric, res.bits], axis=1).astype(np.int64), fmt="%04x")
    (out / "exp_trig.txt").write_text("".join(f"{t}\n" for t in trig))
    (out / "exp_cfo.txt").write_text("".join(cfo_lines))
    (out / "exp_tone.txt").write_text("".join(tone_lines))
    trunc = [e.t_trig for e in res.events if e.status & pr.EV_TRUNCATED]
    (out / "exp_open.txt").write_text(f"{len(trunc)} {trunc[0] if trunc else 0}\n")
    n_fail = sum(int(l.split()[-1]) for l in tone_lines)
    shown = trig if len(trig) <= 12 else trig[:12] + ["..."]
    good = sum(r.status == pr.TM_GOOD for r in res.timing)
    busy = sum(bool(r.status & pr.TM_BUSY) for r in res.timing)
    print(f"{out}: {iq.shape[0]} samples, input_stage {stage}, {len(trig)} trigger(s) at {shown}, "
          f"{len(cfo_lines)} CFO result(s), {n_fail} failing the tone check, {len(res.timing)} timing "
          f"record(s) ({good} good, {busy} busy), {len(res.control)} control record(s) "
          f"({sum(c.crc_ok for c in res.control)} CRC ok), {len(res.payload)} payload record(s)")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Export a test vector for the RTL testbench")
    ap.add_argument("vector", type=Path,
                    help="TestVectors/<name>.json, or 'stress', 'cfo' (synthetic vectors), 'units' (unit tests)")
    ap.add_argument("--input-stage", type=int, choices=(0, 1, 2), default=1)
    ap.add_argument("--max-samples", type=int)
    ap.add_argument("--holdoff", type=int, help="override arm_holdoff (e.g. small, to get many triggers)")
    ap.add_argument("--n-cfo", type=int, help="override n_cfo (128..4096)")
    ap.add_argument("--rearm", type=int, choices=(0, 1),
                    help="1 = re-arm by the Rx state machine (Params default), 0 = fixed holdoff")
    ap.add_argument("--out", type=Path, default=HERE / "work")
    a = ap.parse_args(argv)
    export(a.vector, a.input_stage, a.max_samples, a.out, a.holdoff, a.n_cfo, a.rearm)
    return 0


if __name__ == "__main__":
    sys.exit(main())
