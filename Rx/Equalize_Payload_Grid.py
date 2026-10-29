#!/usr/bin/env python3
"""
Equalize_Payload_Grid.py  (lives in Flexlink_py/Rx)

Estimates the channel from the reference signals and equalizes the received payload grid.
Runs between Demodulate_Waveform_Rx.py and Get_DataSigs_QAM.py.

Inputs
------
Rx/Parameters_Rx_payload.json        Defined_Rx: Mod, Port, PacketLength, RefSignalSpacing,
                                     RefSymbolSpacing, EqualizerMode, EqualizerMaxDoppler.
Rx/<File_Rx.Grid_Payload_Rx>         'data' (PacketLength, 841) complex, written by
                                     Demodulate_Waveform_Rx.py.
Construct_Grid/<File_Rx.ResourceGrid_Rx>
                                     'signal_types' (PacketLength, 841): 10=Ref_P0,
                                     20=Ref_P1, 30=DC, 40=Data. Written by Initialize_Rx.py.
ReferenceSignals/Ref_P<Port>_<RefSignalSpacing>_<bits>.h5
                                     'data': the known Ref values for one Symbol_Ref
                                     (bits = 1, 2, 4, 6 for BPSK, QPSK, 16QAM, 64QAM).
Flexlink_BasicModules/Equalizer/Channel_Estimator.py
                                     the estimators and equalizers.
Flexlink_BasicModules/Equalizer/PCA/PCA_Estimator.py, PCA_Basis_S<RefSignalSpacing>.h5
                                     EqualizerMode PCA only.

EqualizerMode (Defined_Rx, or --mode)
-------------------------------------
  LMMSE            LMMSE across frequency, delay window picked per packet from the
                   measured delay profile, Wiener filter across time. Recommended.
  LMMSE_LinearTime Same across frequency, linear interpolation across time
                   (simpler, at most ~0.5 dB worse with RefSymbolSpacing 3 and 6).
  PCA              Low-rank (principal component) LMMSE across frequency using bases
                   precomputed from channel models (Equalizer/PCA/Train_PCA_Basis.py);
                   class Short/Medium/Long picked per packet; Wiener filter across time.
                   Fewer operations than LMMSE; needs PCA_Basis_S<RefSignalSpacing>.h5.
  FIR8             The FPGA control-field estimator: fixed 8-tap LMMSE across frequency
                   (delays -20..12 samples, no per-packet window choice), drift-corrected
                   linear interpolation across time (FixedFIRChannelEstimator). Default for
                   the control field (Process_Control_Field_Rx.py, ControlFieldEqualizerMode).
  Linear           Linear interpolation across frequency and time (baseline).
  None             No equalization: copies the grid (for loopback checks).
EqualizerMaxDoppler (Hz, default 300) is the worst-case Doppler the Wiener time
filter is designed for; it is used by LMMSE and PCA.

Steps
-----
  1. Load the grid, the signal types and the Ref_P<Port> vector.
  2. Noise variance sigma^2 from the tones that carry nothing (the other port's
     comb and the DC tones in reference symbols). It includes inter-carrier
     interference, which is what the equalizer should see.
  3. Channel estimate H for every signal (EqualizerMode).
  4. Equalize: x = Y / H (unbiased), with the noise left on each signal
     nv = sigma^2 / |H|^2. DC signals and the other port's comb are set to 0.

Output
------
Rx/<File_Rx.Grid_Equalized_Rx>   (default name Grid_Equalized_Rx.h5)
    data       (PacketLength, 841) complex128  equalized grid, same layout as
                                               Grid_Payload_Rx.h5 (Get_DataSigs_QAM.py
                                               can read it unchanged)
    noise_var  (PacketLength, 841) float64     noise variance left on each signal
                                               after equalization (for soft demapping)
    channel    (PacketLength, 841) complex128  channel estimate H
    attrs      EqualizerMode, Port, PortDetected, Mod, BitsPerSymbol, RefFile,
               PilotPower, NoiseVar, SNR_dB, DelayWindow, MaxDoppler, PacketLength,
               GridFile, ResourceGridFile; PCA mode also PCA_Class (ends in U when the
               uniform fallback was used), PCA_Rank, PCA_Chi

Nothing is written to the JSON.

Usage
-----
    python Rx/Equalize_Payload_Grid.py
    python Rx/Equalize_Payload_Grid.py --mode LMMSE_LinearTime
    python Rx/Equalize_Payload_Grid.py --grid other.h5 --out My_Eq.h5 --port 1
    python Rx/Equalize_Payload_Grid.py --ray 2      # antenna 2: *_Ray2.h5 in and out (default --ray 1)
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType
from typing import Any

import numpy as np

# Make flexlink_paths importable when the script is run from anywhere.
for _parent in Path(__file__).resolve().parents:
    if (_parent / "flexlink_paths.py").is_file():
        sys.path.insert(0, str(_parent))
        break
from flexlink_paths import ROOT, BASIC_MODULES, CONSTRUCT_GRID, MAX_RAYS, ray_file_name  # noqa: E402

try:
    from flexlink_paths import REFERENCE_SIGNALS  # noqa: E402
except ImportError:
    REFERENCE_SIGNALS = ROOT / "ReferenceSignals"
try:
    from flexlink_paths import EQUALIZER  # noqa: E402
except ImportError:
    EQUALIZER = BASIC_MODULES / "Equalizer"

RX_DIR = Path(__file__).resolve().parent          # the Rx folder
PARAMS_FILE = RX_DIR / "Parameters_Rx_payload.json"
DEFINED_SECTION = "Defined_Rx"
FILE_SECTION = "File_Rx"

GRID_KEY = "Grid_Payload_Rx"            # File_Rx/<key> = input grid (Rx/)
RESOURCE_GRID_KEY = "ResourceGrid_Rx"   # File_Rx/<key> = signal types (Construct_Grid/)
OUTPUT_KEY = "Grid_Equalized_Rx"        # File_Rx/<key> = output (Rx/)
DEFAULT_OUTPUT = "Grid_Equalized_Rx.h5"
ESTIMATOR_SCRIPT = "Channel_Estimator.py"
PCA_SCRIPT = "PCA/PCA_Estimator.py"
DATASET = "data"
TYPES_DATASET = "signal_types"

MODES = ("LMMSE", "LMMSE_LinearTime", "PCA", "FIR8", "Linear", "None")
DEFAULT_MODE = "LMMSE"
DEFAULT_MAX_DOPPLER = 300.0
DEFAULT_CODES = {"Ref_P0": 10, "Ref_P1": 20, "DC": 30, "Data": 40}
MOD_BITS = {"BPSK": 1, "QPSK": 2, "16QAM": 4, "64QAM": 6}
VALID_PORTS = (0, 1)
VALID_SPACINGS = (3, 6, 12, 24)
MIN_NOISE_VAR = 1e-10                   # floor so an ideal loopback still gives finite values


# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------
def load_params(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"Parameter file not found: {path}")
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def get_value(params: dict[str, Any], section: str, key: str, default: Any = None) -> Any:
    try:
        value = params[section][key]
    except (KeyError, TypeError):
        value = None
    if value is None:
        if default is not None:
            return default
        raise KeyError(f"{section}/{key} not found in {PARAMS_FILE.name}")
    return value


def file_name(params: dict[str, Any], key: str, default: str | None = None) -> str:
    try:
        name = get_value(params, FILE_SECTION, key, default)
    except KeyError:
        raise KeyError(f"{FILE_SECTION}/{key} not found in {PARAMS_FILE.name}") from None
    if not isinstance(name, str) or not name.strip():
        raise ValueError(f"{FILE_SECTION}/{key} must be a file name, got {name!r}")
    return name.strip()


def mod_to_bits(value: Any) -> tuple[str, int]:
    key = str(value).strip().upper().replace("-", "").replace("_", "").replace(" ", "")
    if key not in MOD_BITS:
        raise ValueError(f"Mod {value!r} must be one of {list(MOD_BITS)}")
    return key, MOD_BITS[key]


def check_mode(value: Any) -> str:
    for m in MODES:
        if str(value).strip().lower() == m.lower():
            return m
    raise ValueError(f"EqualizerMode {value!r} must be one of {list(MODES)}")


def check_port(value: Any) -> int:
    f = float(value)
    if not f.is_integer() or int(f) not in VALID_PORTS:
        raise ValueError(f"Port {value!r} must be one of {VALID_PORTS}")
    return int(f)


def check_spacing(value: Any) -> int:
    f = float(value)
    if not f.is_integer() or int(f) not in VALID_SPACINGS:
        raise ValueError(f"RefSignalSpacing {value!r} must be one of {VALID_SPACINGS}")
    return int(f)


# ---------------------------------------------------------------------------
# File reading / writing
# ---------------------------------------------------------------------------
def to_complex(arr: np.ndarray) -> np.ndarray:
    """Accept native complex or a (real, imag) compound dtype."""
    if arr.dtype.names and {"r", "i"} <= set(arr.dtype.names):
        return arr["r"] + 1j * arr["i"]
    if arr.dtype.names and {"real", "imag"} <= set(arr.dtype.names):
        return arr["real"] + 1j * arr["imag"]
    return arr.astype(np.complex128)


def load_dataset(path: Path, what: str, hint: str) -> np.ndarray:
    import h5py
    if not path.is_file():
        raise FileNotFoundError(f"{what} not found: {path} - {hint}")
    with h5py.File(path, "r") as f:
        if DATASET not in f:
            raise KeyError(f"{path.name} has no '{DATASET}' dataset (has {list(f)})")
        return to_complex(np.asarray(f[DATASET][()]))


def load_signal_types(path: Path) -> tuple[np.ndarray, dict[str, int], dict[str, Any]]:
    import h5py
    if not path.is_file():
        raise FileNotFoundError(f"Resource grid not found: {path} - run Initialize_Rx.py first")
    with h5py.File(path, "r") as f:
        if TYPES_DATASET not in f:
            raise KeyError(f"{path.name} has no '{TYPES_DATASET}' dataset (has {list(f)})")
        types = np.asarray(f[TYPES_DATASET][()])
        attrs = dict(f.attrs)
    codes = {name: int(attrs.get(name, default)) for name, default in DEFAULT_CODES.items()}
    return types, codes, attrs


def save_equalized(path: Path, datasets: dict[str, np.ndarray], attrs: dict[str, Any]) -> None:
    import h5py
    with h5py.File(path, "w") as f:
        for name, arr in datasets.items():
            f.create_dataset(name, data=arr)
        for k, v in attrs.items():
            f.attrs[k] = v


def load_module(name: str, rel_path: str) -> ModuleType:
    script = Path(EQUALIZER) / rel_path
    if not script.is_file():
        raise FileNotFoundError(f"{rel_path} not found in {EQUALIZER}")
    spec = importlib.util.spec_from_file_location(name, script)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module                         # dataclasses need the module registered
    spec.loader.exec_module(module)
    return module


def load_estimator() -> ModuleType:
    return load_module("Channel_Estimator", ESTIMATOR_SCRIPT)


# ---------------------------------------------------------------------------
# Equalization
# ---------------------------------------------------------------------------
def equalize_payload_grid(Y: np.ndarray, types: np.ndarray, codes: dict[str, int], ref: np.ndarray,
                          port: int, mode: str, max_doppler: float, ce: ModuleType,
                          spacing: int | None = None) -> dict[str, Any]:
    """Channel estimate + equalization. Returns datasets and info for the output file."""
    if Y.shape != types.shape:
        raise ValueError(f"Payload grid is {Y.shape} but the resource grid is {types.shape}. "
                         "Check PacketLength and rerun Initialize_Rx.py / Demodulate_Waveform_Rx.py.")
    layout = ce.layout_from_signal_types(types, codes)
    if layout.ref_symbols.size < 2:
        raise ValueError("The grid needs at least two Symbol_Ref for channel estimation")
    Xref = ce.reference_grid(layout, port, ref)
    pilot_power = float(np.mean(np.abs(ref) ** 2))

    port_detected = int(ce.detect_port(Y, layout))
    nv = max(float(ce.estimate_noise_var(Y, layout, port)), MIN_NOISE_VAR)
    window = pca_class = pca_rank = pca_chi = None

    if mode == "None":
        H = np.ones_like(Y)
    elif mode == "Linear":
        H = ce.ChannelEstimator(layout, port, freq="linear", time="linear").estimate(Y, Xref)
    elif mode == "FIR8":
        est = ce.FixedFIRChannelEstimator(layout, port, pilot_power=pilot_power)
        H = est.estimate(Y, Xref)
        window = est.last_window
    elif mode == "PCA":
        pe = load_module("PCA_Estimator", PCA_SCRIPT)
        if spacing is None:
            spacing = int(np.median(np.diff(layout.pilot_idx(layout.ref_symbols[0], port))))
        classes = pe.load_basis(pe.basis_file(spacing), port)
        est = pe.PCAChannelEstimator(layout, port, classes, ce, fd_hz=max_doppler, pilot_power=pilot_power)
        H = est.estimate(Y, Xref, nv)
        window, pca_class, pca_rank, pca_chi = est.last_window, est.last_class, est.last_rank, est.last_chi
    else:
        time = "wiener" if mode == "LMMSE" else "linear"
        est = ce.AdaptiveChannelEstimator(layout, port, fd_hz=max_doppler, pilot_power=pilot_power,
                                          time=time)
        H = est.estimate(Y, Xref, nv)
        window = est.last_window

    x, nv_sig = ce.equalize_for_demapper(Y, H, nv)
    used = (layout.types == ce.DATA) | (layout.types == ce.PORT_CODE[port])
    x = np.where(used, x, 0)
    nv_sig = np.where(used, np.maximum(nv_sig, MIN_NOISE_VAR), 0.0)

    return {"data": x, "noise_var": nv_sig, "channel": H, "noise": nv, "window": window,
            "pilot_power": pilot_power, "port_detected": port_detected,
            "pca_class": pca_class, "pca_rank": pca_rank, "pca_chi": pca_chi,
            "n_ref": int(layout.ref_symbols.size)}


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Estimate the channel from the reference signals and "
                                             "equalize the received payload grid.")
    ap.add_argument("--params", type=Path, default=PARAMS_FILE)
    ap.add_argument("--grid", type=Path, help=f"payload grid .h5 (default: Rx/<File_Rx.{GRID_KEY}>)")
    ap.add_argument("--resource-grid", type=Path,
                    help=f"resource grid .h5 (default: Construct_Grid/<File_Rx.{RESOURCE_GRID_KEY}>)")
    ap.add_argument("--ref-dir", type=Path, help="folder with Ref_Pz_X_Y.h5 (default: ReferenceSignals/)")
    ap.add_argument("--out", type=Path, help=f"output .h5 (default: Rx/<File_Rx.{OUTPUT_KEY}>)")
    ap.add_argument("--mode", help=f"override Defined_Rx/EqualizerMode ({', '.join(MODES)})")
    ap.add_argument("--mod", help="override Defined_Rx/Mod")
    ap.add_argument("--port", type=int, help="override Defined_Rx/Port")
    ap.add_argument("--max-doppler", type=float, help="override Defined_Rx/EqualizerMaxDoppler (Hz)")
    ap.add_argument("--ray", type=int, choices=range(1, MAX_RAYS + 1), metavar="N", default=1,
                    help="antenna (ray) 1-4, default 1: read and write the _RayN files "
                         "(start with Packet_Detector.py --ray N)")
    a = ap.parse_args(argv)

    try:
        if importlib.util.find_spec("h5py") is None:
            raise ValueError("Reading/writing .h5 files needs h5py - install it with: pip install h5py")

        params = load_params(a.params)
        defined = params.get(DEFINED_SECTION, {})
        if a.mode is None and "EqualizerMode" not in defined:
            print(f"Warning: {DEFINED_SECTION}/EqualizerMode not set - using {DEFAULT_MODE}")
        mode = check_mode(a.mode or defined.get("EqualizerMode") or DEFAULT_MODE)
        max_doppler = float(a.max_doppler or defined.get("EqualizerMaxDoppler") or DEFAULT_MAX_DOPPLER)
        mod, bits = mod_to_bits(a.mod or get_value(params, DEFINED_SECTION, "Mod"))
        port = check_port(a.port if a.port is not None else get_value(params, DEFINED_SECTION, "Port"))
        spacing = check_spacing(get_value(params, DEFINED_SECTION, "RefSignalSpacing"))

        grid_path = a.grid or RX_DIR / ray_file_name(file_name(params, GRID_KEY), a.ray)
        rg_path = a.resource_grid or CONSTRUCT_GRID / file_name(params, RESOURCE_GRID_KEY)
        if a.out is None and OUTPUT_KEY not in params.get(FILE_SECTION, {}):
            print(f"Warning: {FILE_SECTION}/{OUTPUT_KEY} not set - writing {DEFAULT_OUTPUT}")
        out_path = a.out or RX_DIR / ray_file_name(file_name(params, OUTPUT_KEY, DEFAULT_OUTPUT), a.ray)
        ref_path = (a.ref_dir or Path(REFERENCE_SIGNALS)) / f"Ref_P{port}_{spacing}_{bits}.h5"

        Y = load_dataset(grid_path, "Payload grid", "run Demodulate_Waveform_Rx.py first")
        if Y.ndim != 2:
            raise ValueError(f"{grid_path.name}: expected a 2-D grid, got shape {Y.shape}")
        types, codes, rg_attrs = load_signal_types(rg_path)
        ref = load_dataset(ref_path, "Reference signal file",
                           "run ReferenceSignals/Generate_Reference_Signal_Vectors.py").ravel()

        # The resource grid must have been built from the same parameters.
        checks = (("RefSignalSpacing", "ReferenceSymbolProfile"),
                  ("RefSymbolSpacing", "PacketProfile"),
                  ("PacketLength", "Packet_Len"))
        for param_key, grid_attr in checks:
            want = defined.get(param_key)
            have = rg_attrs.get(grid_attr)
            if want is not None and have is not None and int(have) != int(want):
                raise ValueError(f"{a.params.name} has {param_key} {want} but {rg_path.name} was "
                                 f"built with {grid_attr} {int(have)} - run Initialize_Rx.py")

        ce = load_estimator()
        r = equalize_payload_grid(Y, types, codes, ref, port, mode, max_doppler, ce, spacing)

        if r["port_detected"] != port:
            print(f"Warning: Port is {port} but the received energy is on Ref_P{r['port_detected']} - "
                  "check Defined_Rx/Port")

        snr_db = float(-10 * np.log10(r["noise"]))
        attrs = {"EqualizerMode": mode, "Port": port, "PortDetected": r["port_detected"], "Mod": mod,
                 "BitsPerSymbol": bits, "RefFile": ref_path.name, "PilotPower": r["pilot_power"],
                 "NoiseVar": r["noise"], "SNR_dB": snr_db, "MaxDoppler": max_doppler,
                 "PacketLength": Y.shape[0], "GridFile": grid_path.name,
                 "ResourceGridFile": rg_path.name, "Ray": a.ray}
        if r["window"] is not None:
            attrs["DelayWindow"] = np.asarray(r["window"], dtype=np.int64)
        if r["pca_class"] is not None:
            attrs["PCA_Class"] = r["pca_class"]
            attrs["PCA_Rank"] = r["pca_rank"]
            attrs["PCA_Chi"] = r["pca_chi"]
        out_path.parent.mkdir(parents=True, exist_ok=True)
        save_equalized(out_path, {"data": r["data"], "noise_var": r["noise_var"], "channel": r["channel"]},
                       attrs)

        print(f"Grid {Y.shape} from {grid_path.name}, {r['n_ref']} Symbol_Ref, Ref_P{port} from "
              f"{ref_path.name} (pilot power {r['pilot_power']:.3f})")
        print(f"EqualizerMode {mode}" + (f", delay window {r['window'][0]}..{r['window'][1]} samples, "
                                         f"max Doppler {max_doppler:g} Hz" if r["window"] else "")
              + (f", PCA class {r['pca_class']} rank {r['pca_rank']} (fit chi {r['pca_chi']:.2f}"
                 + (", channel unlike the training models - uniform fallback" if r["pca_class"].endswith("U") else "")
                 + ")" if r["pca_class"] else ""))
        print(f"Noise variance {r['noise']:.3e} (SNR ~ {snr_db:.1f} dB, includes ICI)")
        print(f"Wrote {out_path}")
    except (FileNotFoundError, KeyError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
