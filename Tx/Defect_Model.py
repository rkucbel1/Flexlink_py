#!/usr/bin/env python3
"""
Defect_Model.py  (lives in Flexlink_py/Tx)

Runs the transmitted payload waveform through the channel and receiver
defects switched on in Flexlink_BasicModules/Channel_Defects/Channel_Defects.json,
once per received ray (receive branch), and saves all rays in one file.

Steps:
  1. Read Tx/Parameters_Tx_payload.json and Channel_Defects.json.
  2. Load the waveform from Tx/<File.Waveform_Payload_Tx> (written by
     Construct_Payload_Waveform.py): dataset 'data', complex, 1-D.
  3. For each ray k = 1 .. Settings/NumberOfRays, apply every defect whose
     Mode flag is 1, in this order:
        Multipath     FIR model: Fixed_Tap_Channel; EPA/EVA/ETU:
                      Delay_Doppler_Channel. Each ray gets its own random
                      channel realization.
        TimingOffset  Cause_Timing_Offset   (Settings/Sample_Offset)
        TimingDrift   Cause_Timing_Drift    (Settings/Drift_ppm)
        Freq_Offset   Cause_Frequency_Offset (Settings/FrequencyOffset)
        PhaseNoise    Cause_Phase_Noise     (Settings/PhaseNoise_*). One
                      phase noise realization is shared by all rays, as the
                      branches of one receiver share the same oscillator.
        ThermalNoise  Generate_AWGN with Settings/SNR<k>_dB for ray k. Each
                      ray gets independent noise.
        IQ_Imbalance  Cause_IQ_Imbalance    (PhaseImbalance,
                      AmplitudeImbalance_dB)
     Timing offset, drift, frequency offset and IQ imbalance use the same
     settings for every ray; only the SNR and the random parts (multipath,
     noise) differ.
  4. Save all rays to Tx/<File.Waveform_Payload_DefectsModeled_Tx>.

SNR reference (Settings/SNR_Reference):
  "Transmit" (default): noise power is set from the transmitted waveform
      power. The multipath channels have unit average gain, so SNR<k>_dB is
      the AVERAGE SNR and a faded ray gets a lower SNR, as in a real receiver.
  "Received": noise power is set from each ray's own received power, so
      every ray has exactly SNR<k>_dB whatever its fading.

Nothing is written to the JSON files.

Usage:
    python Defect_Model.py
    python Defect_Model.py --seed 1                   # repeatable run
    python Defect_Model.py --input other.h5 --output My_Defects.h5

The output .h5 file contains:
    data   (NumberOfRays, N) complex128   one row per ray
    attrs  all attributes of the input waveform file, plus
           NumberOfRays, DefectsApplied (comma-separated Mode names, in order),
           SNR_dB (per ray, NaN when ThermalNoise is off), SNR_Reference,
           MeasuredSNR_dB (per ray), RayPower (per ray), MultipathModel,
           FrequencyOffset, Sample_Offset, Drift_ppm, PhaseImbalance,
           AmplitudeImbalance_dB, PhaseNoiseRMS_deg, RandomSeed (-1 if none),
           DefectsSourceFile, DefectsSettingsFile.
    PayloadStart and PreambleLength are copied from the input unchanged;
    timing offset, drift and the multipath filter shift where the packet
    actually starts.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

# Make flexlink_paths importable when the script is run from anywhere.
for _parent in Path(__file__).resolve().parents:
    if (_parent / "flexlink_paths.py").is_file():
        sys.path.insert(0, str(_parent))
        break
from flexlink_paths import TX, CHANNEL_DEFECTS  # noqa: E402

# The defect modules import each other by plain name.
sys.path.insert(0, str(CHANNEL_DEFECTS))
from Cause_Frequency_Offset import cause_frequency_offset  # noqa: E402
from Cause_IQ_Imbalance import cause_iq_imbalance, iq_gains_from_db  # noqa: E402
from Cause_Phase_Noise import cause_phase_noise, integrated_phase_error_deg  # noqa: E402
from Cause_Timing_Drift import cause_timing_drift  # noqa: E402
from Cause_Timing_Offset import cause_timing_offset  # noqa: E402
from Delay_Doppler_Channel import delay_doppler_channel  # noqa: E402
from Fixed_Tap_Channel import fixed_tap_channel  # noqa: E402
from Generate_AWGN import generate_awgn  # noqa: E402

PARAMS_FILE = TX / "Parameters_Tx_payload.json"
DEFECTS_FILE = CHANNEL_DEFECTS / "Channel_Defects.json"
INPUT_KEY = "Waveform_Payload_Tx"                    # File/<key> = input (in Tx/)
OUTPUT_KEY = "Waveform_Payload_DefectsModeled_Tx"    # File/<key> = output (in Tx/)
DATASET = "data"

MAX_RAYS = 4
DEFAULT_RAYS = 4
VALID_MODELS = ("FIR", "EPA", "EVA", "ETU")
VALID_SNR_REFERENCES = ("Transmit", "Received")

# Mode name -> processing order
DEFECT_ORDER = ("Multipath", "TimingOffset", "TimingDrift", "Freq_Offset",
                "PhaseNoise", "ThermalNoise", "IQ_Imbalance")


# --------------------------------------------------------------------------- #
# Parameters
# --------------------------------------------------------------------------- #
def load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"Parameter file not found: {path}")
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def file_name(params: dict[str, Any], key: str) -> str:
    """File name stored under File/<key> in the Tx JSON."""
    files = params.get("File")
    name = files.get(key) if isinstance(files, dict) else None
    if not isinstance(name, str) or not name.strip():
        raise KeyError(f"File/{key} not found in {PARAMS_FILE.name}")
    return name.strip()


def enabled_defects(defects: dict[str, Any]) -> list[str]:
    """Mode names set to 1, in processing order."""
    mode = defects.get("Mode")
    if not isinstance(mode, dict):
        raise KeyError(f"Mode not found in {DEFECTS_FILE.name}")
    unknown = [k for k in mode if k not in DEFECT_ORDER]
    if unknown:
        raise KeyError(f"Unknown Mode entries in {DEFECTS_FILE.name}: {unknown}")
    for k, v in mode.items():
        if str(v).strip() not in ("0", "1"):
            raise ValueError(f"Mode/{k} = {v!r} must be 0 or 1")
    return [k for k in DEFECT_ORDER if int(mode.get(k, 0)) == 1]


def setting(s: dict[str, Any], key: str) -> Any:
    if key not in s:
        raise KeyError(f"Settings/{key} not found in {DEFECTS_FILE.name}")
    return s[key]


def number_of_rays(s: dict[str, Any]) -> int:
    n = s.get("NumberOfRays", DEFAULT_RAYS)
    if str(n).strip() not in {str(k) for k in range(1, MAX_RAYS + 1)}:
        raise ValueError(f"Settings/NumberOfRays = {n!r} must be 1 to {MAX_RAYS}")
    return int(n)


def snr_list(s: dict[str, Any], n_rays: int) -> list[float]:
    return [float(setting(s, f"SNR{k}_dB")) for k in range(1, n_rays + 1)]


def snr_reference(s: dict[str, Any]) -> str:
    ref = str(s.get("SNR_Reference", "Transmit")).strip().capitalize()
    if ref not in VALID_SNR_REFERENCES:
        raise ValueError(f"Settings/SNR_Reference = {ref!r} must be one of "
                         f"{VALID_SNR_REFERENCES}")
    return ref


def multipath_model(s: dict[str, Any]) -> str:
    model = str(setting(s, "MultipathModel")).strip().upper()
    if model not in VALID_MODELS:
        raise ValueError(f"Settings/MultipathModel = {model!r} must be one of {VALID_MODELS}")
    if model != "FIR":
        profiles = setting(s, "MultipathProfiles")
        if model not in profiles:
            raise KeyError(f"Settings/MultipathProfiles has no {model} entry")
    return model


# --------------------------------------------------------------------------- #
# Files
# --------------------------------------------------------------------------- #
def to_complex(arr: np.ndarray) -> np.ndarray:
    """Accept native complex or a (real, imag) compound dtype."""
    if arr.dtype.names:
        names = {n.lower(): n for n in arr.dtype.names}
        re = names.get("real") or names.get("r")
        im = names.get("imag") or names.get("i")
        if re and im:
            return arr[re].astype(float) + 1j * arr[im].astype(float)
    return arr.astype(np.complex128)


def load_waveform(path: Path) -> tuple[np.ndarray, dict[str, Any]]:
    import h5py
    if not path.is_file():
        raise FileNotFoundError(f"Waveform not found: {path} - run Construct_Payload_Waveform.py first")
    with h5py.File(path, "r") as f:
        if DATASET not in f:
            raise KeyError(f"{path.name} has no '{DATASET}' dataset (has {list(f)})")
        x = to_complex(np.asarray(f[DATASET][()])).ravel()
        attrs = dict(f.attrs)
    if x.size == 0:
        raise ValueError(f"{path.name} contains an empty waveform")
    return x, attrs


def save_waveform(path: Path, rays: np.ndarray, attrs: dict[str, Any]) -> None:
    import h5py
    with h5py.File(path, "w") as f:
        f.create_dataset(DATASET, data=rays)
        for k, v in attrs.items():
            f.attrs[k] = v


# --------------------------------------------------------------------------- #
# Defect chain
# --------------------------------------------------------------------------- #
def apply_multipath(x: np.ndarray, s: dict[str, Any], fs: float, model: str,
                    rng: np.random.Generator) -> np.ndarray:
    if model == "FIR":
        return fixed_tap_channel(fs, float(setting(s, "DelaySpread")),
                                 int(setting(s, "NumberOfTaps")), x, rng=rng)
    prof = s["MultipathProfiles"][model]
    return delay_doppler_channel(fs, float(setting(s, "KFactor")),
                                 float(setting(s, "DopplerFreq")),
                                 prof["Delays"], prof["Power"], x, rng=rng)


def model_defects(x: np.ndarray, fs: float, defects: dict[str, Any],
                  seed: int | None = None) -> tuple[np.ndarray, dict[str, Any]]:
    """
    Apply the enabled defects to waveform x for every ray.

    Returns (rays, info): rays is (NumberOfRays, N) complex128, info holds
    the per-ray values written to the output attributes.
    """
    s = defects.get("Settings")
    if not isinstance(s, dict):
        raise KeyError(f"Settings not found in {DEFECTS_FILE.name}")
    active = enabled_defects(defects)
    n_rays = number_of_rays(s)
    snrs = snr_list(s, n_rays) if "ThermalNoise" in active else [float("nan")] * n_rays
    snr_ref = snr_reference(s)
    model = multipath_model(s) if "Multipath" in active else "None"

    streams = np.random.SeedSequence(seed).spawn(n_rays + 1)
    ray_rngs = [np.random.default_rng(ss) for ss in streams[:n_rays]]   # one per ray
    pn_rng = np.random.default_rng(streams[n_rays])                      # shared phase noise

    p_tx = float(np.mean(np.abs(x) ** 2))
    shared_phase = None                     # phase noise, built once
    rays: list[np.ndarray] = []
    measured_snr: list[float] = []

    for k in range(n_rays):
        y = x.copy()
        r = ray_rngs[k]
        noise_power = 0.0
        signal_power = 0.0
        for name in active:
            if name == "Multipath":
                y = apply_multipath(y, s, fs, model, r)
            elif name == "TimingOffset":
                y = cause_timing_offset(y, setting(s, "Sample_Offset"))
            elif name == "TimingDrift":
                y = cause_timing_drift(y, setting(s, "Drift_ppm"))
            elif name == "Freq_Offset":
                y = cause_frequency_offset(y, float(setting(s, "FrequencyOffset")), fs)
            elif name == "PhaseNoise":
                if shared_phase is None or shared_phase.size < y.size:
                    _, shared_phase = cause_phase_noise(
                        np.ones(y.size, dtype=complex), fs,
                        setting(s, "PhaseNoise_Offsets_Hz"),
                        setting(s, "PhaseNoise_Levels_dBcHz"),
                        rng=pn_rng, return_phase=True)
                y = y * np.exp(1j * shared_phase[:y.size])
            elif name == "ThermalNoise":
                noise = generate_awgn(y, snrs[k], rng=r)
                signal_power = float(np.mean(np.abs(y) ** 2))
                if snr_ref == "Transmit" and signal_power > 0:
                    noise *= np.sqrt(p_tx / signal_power)
                noise_power = float(np.mean(np.abs(noise) ** 2))
                y = y + noise
            elif name == "IQ_Imbalance":
                i_gain, q_gain = iq_gains_from_db(float(setting(s, "AmplitudeImbalance_dB")))
                y = cause_iq_imbalance(y, float(setting(s, "PhaseImbalance")), i_gain, q_gain)
        rays.append(y)
        if noise_power > 0:
            measured_snr.append(10 * np.log10(max(signal_power, 1e-30) / noise_power))
        else:
            measured_snr.append(float("nan"))

    # Same settings for every ray give equal lengths; pad defensively
    n_out = max(len(y) for y in rays)
    out = np.zeros((n_rays, n_out), dtype=np.complex128)
    for k, y in enumerate(rays):
        out[k, :len(y)] = y

    pn_rms = float("nan")
    if "PhaseNoise" in active:
        pn_rms = float(integrated_phase_error_deg(
            s["PhaseNoise_Offsets_Hz"], s["PhaseNoise_Levels_dBcHz"],
            fs / (1 << int(np.ceil(np.log2(2 * n_out)))), fs / 2))

    info = {
        "NumberOfRays": n_rays,
        "DefectsApplied": ",".join(active) if active else "None",
        "SNR_dB": np.array(snrs),
        "SNR_Reference": snr_ref,
        "MeasuredSNR_dB": np.array(measured_snr),
        "RayPower": np.array([np.mean(np.abs(y) ** 2) for y in rays]),
        "TxPower": p_tx,
        "MultipathModel": model,
        "FrequencyOffset": float(s.get("FrequencyOffset", 0)) if "Freq_Offset" in active else 0.0,
        "Sample_Offset": int(s.get("Sample_Offset", 0)) if "TimingOffset" in active else 0,
        "Drift_ppm": int(s.get("Drift_ppm", 0)) if "TimingDrift" in active else 0,
        "PhaseImbalance": float(s.get("PhaseImbalance", 0)) if "IQ_Imbalance" in active else 0.0,
        "AmplitudeImbalance_dB": (float(s.get("AmplitudeImbalance_dB", 0))
                                  if "IQ_Imbalance" in active else 0.0),
        "PhaseNoiseRMS_deg": pn_rms,
        "RandomSeed": -1 if seed is None else int(seed),
    }
    return out, info


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Apply the Channel_Defects.json defects to the payload waveform.")
    ap.add_argument("--params", type=Path, default=PARAMS_FILE)
    ap.add_argument("--defects", type=Path, default=DEFECTS_FILE)
    ap.add_argument("--input", type=Path, help=f"input waveform .h5 (default: Tx/<File.{INPUT_KEY}>)")
    ap.add_argument("--output", type=Path, help=f"output .h5 (default: Tx/<File.{OUTPUT_KEY}>)")
    ap.add_argument("--seed", type=int,
                    help="random seed for a repeatable run (default: Settings/RandomSeed, else random)")
    a = ap.parse_args(argv)

    try:
        if importlib.util.find_spec("h5py") is None:
            raise ValueError("Reading/writing .h5 files needs h5py - install it with: pip install h5py")

        params = load_json(a.params)
        defects = load_json(a.defects)
        in_path = a.input or TX / file_name(params, INPUT_KEY)
        out_path = a.output or TX / file_name(params, OUTPUT_KEY)

        seed = a.seed
        if seed is None:
            js = defects.get("Settings", {}).get("RandomSeed")
            seed = None if js is None else int(js)

        x, in_attrs = load_waveform(in_path)
        fs = float(in_attrs.get("SampleRate", params.get("Defined", {}).get("SampleRate", 0)))
        if fs <= 0:
            raise KeyError(f"No SampleRate in {in_path.name} or Defined/SampleRate")
        fs_defects = defects.get("Settings", {}).get("SampleRate")
        if fs_defects is not None and float(fs_defects) != fs:
            print(f"Warning: Channel_Defects.json SampleRate {float(fs_defects) / 1e6:.0f} MHz differs "
                  f"from the waveform's {fs / 1e6:.0f} MHz - using {fs / 1e6:.0f} MHz")

        rays, info = model_defects(x, fs, defects, seed)

        out_path.parent.mkdir(parents=True, exist_ok=True)
        save_waveform(out_path, rays, {
            **in_attrs, **info,
            "DefectsSourceFile": in_path.name,
            "DefectsSettingsFile": a.defects.name,
        })

        print(f"Input {in_path.name}: {x.size} samples at {fs / 1e6:.0f} MHz")
        print(f"Defects applied: {info['DefectsApplied'].replace(',', ', ')}")
        if info["MultipathModel"] != "None":
            print(f"Multipath model {info['MultipathModel']} (independent channel per ray)")
        if not np.isnan(info["PhaseNoiseRMS_deg"]):
            print(f"Phase noise {info['PhaseNoiseRMS_deg']:.2f} deg RMS (shared by all rays)")
        for k in range(info["NumberOfRays"]):
            line = f"  Ray {k + 1}: power {info['RayPower'][k]:.4f}"
            if not np.isnan(info["SNR_dB"][k]):
                line += (f", SNR set {info['SNR_dB'][k]:.1f} dB ({info['SNR_Reference']}), "
                         f"measured {info['MeasuredSNR_dB'][k]:.1f} dB")
            print(line)
        print(f"Wrote {out_path}: data {rays.shape[0]} x {rays.shape[1]}")
    except (FileNotFoundError, KeyError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
