#!/usr/bin/env python3
"""
pd_make_vector.py  (Flexlink_py/HW_Development/Packet_Detector/PC)

Turns a simulator waveform (.h5) into a test vector for the Zedboard packet detector:

  1. Load dataset 'data' from the .h5 (1-D, or one row per ray: pick --ray).
  2. Add idle noise before and after each packet (spec F2), at 40 Msps, and repeat the
     packet --repeat times so re-arming is exercised. The idle noise power is taken from
     the file's SNR attributes (or --idle-noise-power); continuous defects (DC, CW) are not
     extended into the idle samples.
  3. Decimate to 20 Msps as the simulator does (Rx/Decimate_Payload_Rx.decimate_payload).
  4. Quantise to 12-bit signed I/Q: the whole record is scaled so its rms sits --backoff dB
     below full scale (AGC-normalised radio samples with headroom), rounded and clipped.
  5. Bit-true front end (pd_reference.front_end): integer DC blocker (Defined_Rx/DCBlocker)
     and 17-tap LPF -> 12-bit LPF output, which is what the detector port receives (spec 5.1).
  6. Expected events from the bit-true model (pd_reference.detect), plus a cross-check with
     the simulator's own floating-point chain on the unquantised samples.
  7. Write (int16 I,Q interleaved, little-endian):
       <out>.bin      LPF output       (board input_stage 0: the PC did the front end)
       <out>_dec.bin  decimated input  (board input_stage 1: the board runs the front end)
       <out>_40.bin   40 Msps input    (board input_stage 2: the board also decimates, with its
                      half-band; only for a 40 Msps source). Quantised at 40 Msps with the same
                      backoff, before any decimation.
     and <out>.json (metadata, parameters, expected events). Inputs 0 and 1 give the same events
     ("expected_events"); input 2 has its own ("expected_events_s2"): the half-band is not the
     simulator's resample_poly, so samples and results differ slightly.

Usage
    python pd_make_vector.py                                   # default Tx test file
    python pd_make_vector.py --input ../../../Tx/Waveform_Payload_DefectsModeled_Tx.h5
    python pd_make_vector.py --repeat 4 --idle-pre 4000 --idle-post 4000 --seed 1
    python pd_make_vector.py --backoff 15 --out TestVectors/my_vector
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from pd_protocol import Params, q8, q16  # noqa: E402
import pd_reference as ref  # noqa: E402

PREAMBLE_A_OFFSET = 100        # AGC burst length at 20 Msps
LPF_DELAY = 8
FULL_SCALE = 2047


def find_project_root(start: Path) -> Path:
    for p in [start, *start.parents]:
        if (p / "flexlink_paths.py").is_file():
            return p
    raise FileNotFoundError("Flexlink_py root (flexlink_paths.py) not found above this script - use --project-root")


def load_module(path: Path) -> ModuleType:
    if not path.is_file():
        raise FileNotFoundError(path)
    spec = importlib.util.spec_from_file_location(path.stem, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[path.stem] = mod
    spec.loader.exec_module(mod)
    return mod


def to_complex(arr: np.ndarray) -> np.ndarray:
    if arr.dtype.names:
        names = {n.lower(): n for n in arr.dtype.names}
        re, im = names.get("real") or names.get("r"), names.get("imag") or names.get("i")
        return arr[re].astype(float) + 1j * arr[im].astype(float)
    return arr.astype(np.complex128)


def load_h5(path: Path, ray: int) -> tuple[np.ndarray, dict]:
    import h5py
    with h5py.File(path, "r") as f:
        x = to_complex(np.asarray(f["data"][()]))
        attrs = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in f.attrs.items()}
    if x.ndim == 2:
        if not 1 <= ray <= x.shape[0]:
            raise ValueError(f"--ray {ray}: file has {x.shape[0]} rays")
        x = x[ray - 1]
    for k, v in list(attrs.items()):
        if isinstance(v, bytes):
            attrs[k] = v.decode(errors="replace")
        elif isinstance(v, (np.generic,)):
            attrs[k] = v.item()
    return x.ravel(), attrs


def ray_value(attrs: dict, key: str, ray: int):
    v = attrs.get(key)
    if isinstance(v, list):
        return v[ray - 1] if len(v) >= ray else None
    return v


def idle_noise_power(x: np.ndarray, attrs: dict, ray: int) -> float:
    """Noise power of the record, from its SNR attribute: P_total = P_s + P_n, SNR = P_s / P_n."""
    snr = ray_value(attrs, "MeasuredSNR_dB", ray)
    if snr is None or not np.isfinite(snr):
        snr = ray_value(attrs, "SNR_dB", ray)
    if snr is None or not np.isfinite(float(snr)):
        return 0.0
    return float(np.mean(np.abs(x) ** 2)) / (1 + 10 ** (float(snr) / 10))


def rx_params(root: Path) -> dict:
    p = root / "Rx" / "Parameters_Rx_payload.json"
    try:
        return json.loads(p.read_text()).get("Defined_Rx", {})
    except (OSError, json.JSONDecodeError):
        return {}


def simulator_crosscheck(x_lpf: np.ndarray, mods: dict, p: Params) -> list[dict]:
    """Simulator detector (Bounded, qualifier), lag cascade and tone check on float samples."""
    det, freq = mods["det"], mods["freq"]
    eta_hi, eta_lo = np.sqrt(p.eta_hi_sq / 65536), np.sqrt(p.eta_lo_sq / 65536)
    flag, _, _, _ = det.packet_detector_flexlink(x_lpf, ref.L, eta_lo, eta_hi,
                                                 normaliser="Bounded", qualifier=p.qual_len)
    rises = np.flatnonzero(np.diff(np.concatenate([[0], flag.astype(int)])) == 1)
    out, next_ok = [], 0
    for t in rises:
        if t < next_ok:
            continue
        next_ok = t + p.arm_holdoff
        one = np.zeros(x_lpf.size, np.int8)
        one[t:] = 1
        try:
            coarse, fine, d = freq.detect_frequency_offset_cascade(one, x_lpf, window=p.n_cfo)
            tone, _, _ = freq.tone_structure_check(x_lpf, int(t), coarse + fine, window=p.n_cfo)
            out.append({"t_trig": int(t), "f_hat_hz": float(coarse + fine), "tone_db": float(tone)})
        except ValueError as exc:
            out.append({"t_trig": int(t), "error": str(exc)})
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Build a Zedboard packet-detector test vector from a simulator .h5")
    ap.add_argument("--project-root", type=Path)
    ap.add_argument("--input", type=Path, help="waveform .h5 (default: Tx/Waveform_Payload_DefectsModeled_Tx.h5)")
    ap.add_argument("--ray", type=int, default=1)
    ap.add_argument("--out", type=Path, help="output path without extension (default: TestVectors/<input stem>)")
    ap.add_argument("--idle-pre", type=int, default=3000, help="idle samples before each packet, at 20 Msps")
    ap.add_argument("--idle-post", type=int, default=3000, help="idle samples after each packet, at 20 Msps")
    ap.add_argument("--repeat", type=int, default=1, help="number of packet copies")
    ap.add_argument("--idle-noise-power", type=float, help="idle noise power (input units) instead of the SNR attrs")
    ap.add_argument("--backoff", type=float, default=12.0, help="record rms below 12-bit full scale, dB")
    ap.add_argument("--dc-blocker", type=int, choices=(0, 1), help="default: Defined_Rx/DCBlocker (1)")
    ap.add_argument("--eta-hi", type=float, help="default: Defined_Rx/PacketDetThreshHi")
    ap.add_argument("--eta-lo", type=float, help="default: Defined_Rx/PacketDetThreshLo")
    ap.add_argument("--qualifier", type=int, help="default: Defined_Rx/PacketDetQualifier")
    ap.add_argument("--n-cfo", type=int, default=2048)
    ap.add_argument("--tone-thresh", type=float, help="dB, default Defined_Rx/ToneCheckThresh")
    ap.add_argument("--seed", type=int)
    ap.add_argument("--no-crosscheck", action="store_true", help="skip the simulator cross-check")
    a = ap.parse_args(argv)

    root = a.project_root or find_project_root(HERE)
    in_path = a.input or root / "Tx" / "Waveform_Payload_DefectsModeled_Tx.h5"
    out = a.out or HERE.parent / "TestVectors" / in_path.stem
    rng = np.random.default_rng(a.seed)
    dr = rx_params(root)

    # 1. Load
    x, attrs = load_h5(in_path, a.ray)
    fs = float(attrs.get("SampleRate", 40e6))
    if fs not in (20e6, 40e6):
        raise ValueError(f"SampleRate {fs / 1e6:g} MHz - expected 20 or 40 MHz")
    up = int(fs // 20e6)                                   # 2 at 40 Msps

    # 2. Idle noise and repeats (at the input rate)
    pn = a.idle_noise_power if a.idle_noise_power is not None else idle_noise_power(x, attrs, a.ray)

    def noise(n):
        return np.sqrt(pn / 2) * (rng.standard_normal(n) + 1j * rng.standard_normal(n))

    parts, starts = [], []
    pos = 0
    for _ in range(max(a.repeat, 1)):
        parts.append(noise(a.idle_pre * up))
        pos += a.idle_pre * up
        starts.append(pos)
        parts.append(x)
        pos += x.size
        parts.append(noise(a.idle_post * up))
        pos += a.idle_post * up
    x_in = np.concatenate(parts)

    # 3. Decimate (simulator code)
    dec = load_module(root / "Rx" / "Decimate_Payload_Rx.py")
    lpf = load_module(root / "Flexlink_BasicModules" / "Preamble" / "Receive_LP_Filter.py")
    defects = bool(attrs.get("DefectsApplied")) or pn > 0
    x20, dinfo = dec.decimate_payload(x_in, fs, "filter" if defects else "simple")
    dc_on = a.dc_blocker if a.dc_blocker is not None else int(dr.get("DCBlocker", 1))
    dc_shift = 10 if dc_on else 0

    # 4. Quantise the decimated stream to 12 bits
    rms = float(np.sqrt(np.mean(np.abs(x20) ** 2)))
    scale = FULL_SCALE * 10 ** (-a.backoff / 20) / rms if rms > 0 else 1.0
    iq_f = np.stack([x20.real, x20.imag], axis=1) * scale
    clipped = int(np.count_nonzero(np.abs(iq_f) > FULL_SCALE))
    iq_dec = np.clip(np.round(iq_f), -FULL_SCALE - 1, FULL_SCALE).astype(np.int16)

    # 5. Bit-true front end -> detector input; float version for the simulator cross-check
    iq = ref.front_end(iq_dec, dc_shift).astype(np.int16)
    x_lpf = lpf.lp_filter_flexlink(lpf.dc_blocker(x20) if dc_on else x20)

    # 5. Expected results
    pkt20 = int(np.ceil(x.size / up))
    p = Params(eta_hi_sq=q16((a.eta_hi or float(dr.get("PacketDetThreshHi", 0.35))) ** 2),
               eta_lo_sq=q16((a.eta_lo or float(dr.get("PacketDetThreshLo", 0.2))) ** 2),
               qual_len=int(a.qualifier if a.qualifier is not None else dr.get("PacketDetQualifier", 128)),
               n_cfo=a.n_cfo,
               tone_thresh=q8(a.tone_thresh if a.tone_thresh is not None else float(dr.get("ToneCheckThresh", 13))),
               arm_holdoff=pkt20 + 1024, input_stage=0, dc_shift=dc_shift)
    res = ref.detect(iq, p)
    truth = [{"preamble_a_start": int(s // up) + PREAMBLE_A_OFFSET + LPF_DELAY,
              "packet_end": int((s + x.size) // up) + LPF_DELAY} for s in starts]

    sim = []
    if not a.no_crosscheck:
        mods = {"det": load_module(root / "Flexlink_BasicModules" / "Preamble" / "Receive_Packet_Detector.py"),
                "freq": load_module(root / "Flexlink_BasicModules" / "Preamble" / "Receive_Get_Frequency_Offset.py")}
        sim = simulator_crosscheck(x_lpf, mods, p)

    # 5b. 40 Msps input for input_stage 2: quantise before decimation, same backoff
    iq40, res40, scale40, clipped40 = None, None, None, 0
    if up == 2:
        n40 = x_in.size - x_in.size % 2
        rms40 = float(np.sqrt(np.mean(np.abs(x_in[:n40]) ** 2)))
        scale40 = FULL_SCALE * 10 ** (-a.backoff / 20) / rms40 if rms40 > 0 else 1.0
        f40 = np.stack([x_in[:n40].real, x_in[:n40].imag], axis=1) * scale40
        clipped40 = int(np.count_nonzero(np.abs(f40) > FULL_SCALE))
        iq40 = np.clip(np.round(f40), -FULL_SCALE - 1, FULL_SCALE).astype(np.int16)
        p40 = Params(**{**p.as_dict(), "input_stage": 2})
        res40 = ref.detect(iq40, p40)

    # 6. Write
    out.parent.mkdir(parents=True, exist_ok=True)
    iq.tofile(out.with_suffix(".bin"))
    dec_path = out.with_name(out.name + "_dec.bin")
    iq_dec.tofile(dec_path)
    files = {"lpf_out": out.with_suffix(".bin").name, "decimated": dec_path.name}
    if iq40 is not None:
        p40_path = out.with_name(out.name + "_40.bin")
        iq40.tofile(p40_path)
        files["input40"] = p40_path.name
    meta = {
        "format": "int16 I,Q interleaved, little-endian, 12-bit values, 20 Msps (_40.bin: 40 Msps)",
        "files": files,
        "source": str(in_path.name), "ray": a.ray, "samples": int(iq.shape[0]),
        "sample_rate": 20e6,
        "front_end": {"decimator": "filter" if defects else "simple", "dc_shift": dc_shift,
                      "model": "pd_reference.front_end (bit-true DC blocker + 17-tap LPF)"},
        "quantisation": {"backoff_db": a.backoff, "scale": scale, "clipped_values": clipped},
        "idle": {"pre": a.idle_pre, "post": a.idle_post, "repeat": a.repeat, "noise_power": pn,
                 "note": "idle samples hold noise only; DC and CW defects are not extended into them"},
        "defects": {k: attrs[k] for k in ("DefectsApplied", "FrequencyOffset", "SNR_dB", "MeasuredSNR_dB",
                                          "Sample_Offset", "Drift_ppm", "DCOffset_dBc", "CW_Frequency",
                                          "CW_Level_dBc", "MultipathModel", "RandomSeed") if k in attrs},
        "truth": truth,
        "params": p.as_dict(),
        "expected_events": [{**vars(e), **{k: v for k, v in x.items() if k != "t_trig"}}
                            for e, x in zip(res.events, res.extra)],
        "expected_events_s2": None if res40 is None else
                              [{**vars(e), **{k: v for k, v in x.items() if k != "t_trig"}}
                               for e, x in zip(res40.events, res40.extra)],
        "samples_40": None if iq40 is None else int(iq40.shape[0]),
        "quantisation_40": None if iq40 is None else {"backoff_db": a.backoff, "scale": scale40,
                                                       "clipped_values": clipped40},
        "simulator_crosscheck": sim,
        "seed": a.seed,
    }
    out.with_suffix(".json").write_text(json.dumps(meta, indent=2, default=lambda o: o.item() if isinstance(o, np.generic) else str(o)))

    # Report
    print(f"Input  {in_path.name}: {x.size} samples at {fs / 1e6:g} Msps, ray {a.ray}")
    print(f"Vector {out.with_suffix('.bin')} (+ {dec_path.name}): {iq.shape[0]} samples "
          f"({iq.shape[0] / 20e6 * 1e3:.2f} ms), {a.repeat} packet(s), input rms {-a.backoff:.1f} dBFS, "
          f"{clipped} clipped values")
    print(f"Params {p.describe()}")
    if res40 is not None:
        print(f"40 Msps input {files['input40']}: {iq40.shape[0]} samples, {clipped40} clipped values; "
              f"input_stage 2 events: " + ", ".join(
                  f"t_trig {e.t_trig} f_hat {x['f_hat_hz']:.0f} Hz" if "f_hat_hz" in x else f"t_trig {e.t_trig}"
                  for e, x in zip(res40.events, res40.extra)))
    true_cfo = attrs.get("FrequencyOffset")
    for i, (e, xx) in enumerate(zip(res.events, res.extra)):
        line = f"  ref event {i}: t_trig {e.t_trig}"
        near = [t for t in truth if t["preamble_a_start"] - 300 <= e.t_trig <= t["packet_end"]]
        if near:
            line += f" (latency {e.t_trig - near[0]['preamble_a_start']})"
        else:
            line += " (not on a packet)"
        if "f_hat_hz" in xx:
            line += f", f_hat {xx['f_hat_hz']:.0f} Hz"
            if true_cfo is not None:
                line += f" (true {float(true_cfo):.0f})"
            line += f", ToneCheck {xx['tone_db']:.1f} dB{' FAIL' if e.status & 2 else ''}"
        print(line)
    for i, s in enumerate(sim):
        if "f_hat_hz" in s:
            print(f"  sim event {i}: t_trig {s['t_trig']}, f_hat {s['f_hat_hz']:.0f} Hz, ToneCheck {s['tone_db']:.1f} dB")
        else:
            print(f"  sim event {i}: t_trig {s['t_trig']}, {s.get('error')}")
    print(f"Wrote {out.with_suffix('.json')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
