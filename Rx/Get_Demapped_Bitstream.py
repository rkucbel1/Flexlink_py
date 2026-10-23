#!/usr/bin/env python3
"""
Get_Demapped_Bitstream.py - Demap the received QAM data signals to the encoded bitstream.
Reverse of Tx/Construct_Datasigs_QAM.py.

Steps:
  1. Read Parameters_Rx_payload.json from the Rx directory.
  2. Read Mod (BPSK, QPSK, 16QAM or 64QAM) and Demapping ("soft" or "hard")
     from Defined_Rx.
  3. Load DataSigs_QAM from dataset 'data' of the file named in
     File_Rx/Sequence_DataSigs_QAM in the Rx directory (written by Get_DataSigs_QAM.py),
     and its 'noise_var' dataset if present (one value per symbol, from the equalizer).
  4. Demap every value with QAM_Demapper() from QAM_Mapper/QAM_Demapper.py
     (IEEE 802.11 Gray mapping, first bit = MSB, same tables as qam_map):
       hard : nearest constellation point -> bits 0/1 (uint8)
       soft : max-log LLR -> P(bit = 1) = 1 / (1 + exp(LLR)), 0..1 (float64).
              This is the 0..1 soft-value convention the Rx decode chain
              (Rate_Matching_Rx / FEC_LDPC_Rx) already uses. Noise variance, in order:
                --noise-var        one given value for every symbol (testing)
                'noise_var'        one value per symbol from the equalizer: the noise
                                   left on that tone after dividing by the channel.
                                   Faded tones get small (honest) LLRs. Needs the
                                   QAM_Demapper.py that accepts a per-symbol noise_var.
                estimated          one value for the packet from the distance to the
                                   nearest constellation point (fallback, e.g. after
                                   Get_DataSigs_QAM.py --unequalized).
  5. Check len(Bitstream) == Calculated_Rx/NumberOfDataSigs.
  6. Save it to the file named in File_Rx/Bitstream_Encoded_Rx in the Rx directory
     (or --out). The bits are still scrambled; Get_Decoded_Bitstream.py descrambles.

Usage:
    python Get_Demapped_Bitstream.py                      # mode from Defined_Rx/Demapping
    python Get_Demapped_Bitstream.py --demapping hard     # override the JSON
    python Get_Demapped_Bitstream.py --noise-var 0.01 --out My_Bits.h5
    python Get_Demapped_Bitstream.py --ray 2          # antenna 2: *_Ray2.h5 in and out
    (default: *_Combined.h5 when the run has several antennas, else *_Ray1.h5)

The output .h5 file contains:
    data   (NumberOfDataSigs,) uint8 (hard) or float64 (soft)   scrambled encoded bitstream
    attrs  Mod, BitsPerSymbol, Mode, NumberOfDataSigs, SourceFile, and for soft:
           NoiseVarSource (per-symbol / given / estimated) and NoiseVar (the value, or
           the median of the per-symbol values)
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import logging
import sys
from pathlib import Path
from types import ModuleType
from typing import Any

import numpy as np

# Locate flexlink_paths.py by walking up from this script's folder.
for _parent in Path(__file__).resolve().parents:
    if (_parent / "flexlink_paths.py").is_file():
        sys.path.insert(0, str(_parent))
        break
from flexlink_paths import QAM_MAPPER, MAX_RAYS  # noqa: E402
sys.path.insert(0, str(Path(__file__).resolve().parent))
from Rx_Antennas import antennas_in_run, target_label, target_name  # noqa: E402

RX_DIR = Path(__file__).resolve().parent          # the Rx folder
PARAMS_FILENAME = "Parameters_Rx_payload.json"
DEFINED_SECTION = "Defined_Rx"
CALCULATED_SECTION = "Calculated_Rx"
FILE_SECTION = "File_Rx"

INPUT_FILE_KEY = "Sequence_DataSigs_QAM"       # File_Rx/<key> = input name (Rx directory)
OUTPUT_FILE_KEY = "Bitstream_Encoded_Rx"       # File_Rx/<key> = output name (Rx directory)
DATASET = "data"
NOISE_DATASET = "noise_var"
DEMAPPER_SCRIPT = "QAM_Demapper.py"
DEMAPPING_MODES = ("soft", "hard")
DEFAULT_DEMAPPING = "soft"
MIN_NOISE_VAR = 1e-10
LLR_CLIP = 50.0                                # keeps exp() finite

log = logging.getLogger("Get_Demapped_Bitstream")


# --------------------------------------------------------------------------- #
# JSON
# --------------------------------------------------------------------------- #
def load_params(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"Parameter file not found: {path}")
    try:
        with path.open("r", encoding="utf-8") as fh:
            params = json.load(fh)
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON in {path}: line {exc.lineno}, col {exc.colno}: {exc.msg}") from exc
    if not isinstance(params, dict):
        raise ValueError(f"{path} must contain a JSON object at top level")
    return params


def get_value(params: dict[str, Any], section: str, key: str) -> Any:
    try:
        return params[section][key]
    except (KeyError, TypeError):
        raise ValueError(f"{section}/{key} not set in {PARAMS_FILENAME}") from None


# --------------------------------------------------------------------------- #
# QAM demapper (QAM_Demapper.py, loaded by file path)
# --------------------------------------------------------------------------- #
def load_demapper() -> ModuleType:
    script = QAM_MAPPER / DEMAPPER_SCRIPT
    if not script.is_file():
        raise FileNotFoundError(f"{DEMAPPER_SCRIPT} not found in {QAM_MAPPER}")
    # QAM_Demapper imports QAM_Mapper from its own folder
    if str(QAM_MAPPER) not in sys.path:
        sys.path.insert(0, str(QAM_MAPPER))
    spec = importlib.util.spec_from_file_location("QAM_Demapper", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if not callable(getattr(module, "QAM_Demapper", None)):
        raise ValueError(f"{script} has no QAM_Demapper function")
    return module


def bits_per_symbol(demapper: ModuleType, mod: str) -> tuple[str, int]:
    name = demapper._normalise_name(mod)
    return name, int(demapper._MODULATIONS[name][0])


def estimate_noise_var(demapper: ModuleType, symbols: np.ndarray, mod: str) -> float:
    """Mean |y - nearest constellation point|^2 (decision-directed)."""
    lut = demapper._LUTS[mod]
    d2 = np.abs(symbols[:, None] - lut[None, :]) ** 2
    return float(np.mean(np.min(d2, axis=1)))


# --------------------------------------------------------------------------- #
# .h5 files
# --------------------------------------------------------------------------- #
def load_symbols(path: Path) -> tuple[np.ndarray, np.ndarray | None, dict[str, Any]]:
    """Return (symbols, per-symbol noise variance or None, attrs)."""
    import h5py
    if not path.is_file():
        raise FileNotFoundError(f"Input file not found: {path} - run Get_DataSigs_QAM.py first")
    with h5py.File(path, "r") as f:
        if DATASET not in f:
            raise ValueError(f"{path.name} has no dataset '{DATASET}'")
        symbols = np.asarray(f[DATASET][()]).astype(np.complex128).ravel()
        noise_var = (np.asarray(f[NOISE_DATASET][()], dtype=np.float64).ravel()
                     if NOISE_DATASET in f else None)
        attrs = dict(f.attrs)
    if symbols.size == 0:
        raise ValueError(f"{path} contains an empty vector")
    if noise_var is not None and noise_var.size != symbols.size:
        raise ValueError(f"{path.name}: '{NOISE_DATASET}' has {noise_var.size} values but there are "
                         f"{symbols.size} symbols")
    return symbols, noise_var, attrs


def save_bits(path: Path, bits: np.ndarray, attrs: dict[str, Any]) -> None:
    import h5py
    with h5py.File(path, "w") as f:
        f.create_dataset(DATASET, data=bits)
        for k, v in attrs.items():
            f.attrs[k] = v


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def demap(symbols: np.ndarray, mod: str, demapper: ModuleType, soft: bool = False,
          noise_var: float | np.ndarray | None = None) -> tuple[np.ndarray, float | np.ndarray | None]:
    """Return (Bitstream, noise variance used). Hard: uint8 0/1. Soft: P(bit=1), 0..1.
    noise_var: a scalar, one value per symbol, or None (estimated from the symbols)."""
    if not soft:
        return np.asarray(demapper.QAM_Demapper(symbols, mod, "hard"), dtype=np.uint8), None
    if noise_var is None:
        noise_var = max(estimate_noise_var(demapper, symbols, mod), 1e-6)
    elif np.ndim(noise_var):
        noise_var = np.maximum(np.asarray(noise_var, dtype=np.float64), MIN_NOISE_VAR)
    llr = np.asarray(demapper.QAM_Demapper(symbols, mod, "maxlog", noise_var), dtype=np.float64)
    p1 = 1.0 / (1.0 + np.exp(np.clip(llr, -LLR_CLIP, LLR_CLIP)))   # LLR = ln(P0/P1)
    return p1, noise_var


def demapping_mode(params: dict[str, Any], override: str | None) -> str:
    value = override
    if value is None:
        value = params.get(DEFINED_SECTION, {}).get("Demapping")
        if value is None:
            log.warning("%s/Demapping not set in %s - using %s", DEFINED_SECTION, PARAMS_FILENAME,
                        DEFAULT_DEMAPPING)
            value = DEFAULT_DEMAPPING
    mode = str(value).strip().lower()
    if mode not in DEMAPPING_MODES:
        raise ValueError(f"Demapping {value!r} must be one of {list(DEMAPPING_MODES)}")
    return mode


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Demap the received QAM data signals to bits.")
    ap.add_argument("--input", type=Path,
                    help=f"QAM .h5; a relative name is looked up in the Rx directory "
                         f"(default: File_Rx/{INPUT_FILE_KEY})")
    ap.add_argument("--params", type=Path, help=f"path to {PARAMS_FILENAME} (default: Rx directory)")
    ap.add_argument("--out", type=Path,
                    help=f"output .h5; a relative name is put in the Rx directory "
                         f"(default: File_Rx/{OUTPUT_FILE_KEY})")
    ap.add_argument("--demapping", choices=DEMAPPING_MODES,
                    help="override Defined_Rx/Demapping (soft: P(bit=1) values, hard: bits)")
    ap.add_argument("--noise-var", type=float,
                    help="one total complex noise variance for every symbol in soft mode "
                         "(testing; default: the per-symbol noise_var from the equalizer)")
    ap.add_argument("--ray", type=int, choices=range(1, MAX_RAYS + 1), metavar="N",
                    help="antenna (ray) 1-4: read and write its _RayN files. Default: the _Combined "
                         "files when the run has several antennas (Combine_Antennas_Rx.py), else _Ray1")
    ap.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    return ap.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",
    )
    try:
        if importlib.util.find_spec("h5py") is None:
            raise ValueError("Reading/writing .h5 files needs h5py - install it with: pip install h5py")
        if args.noise_var is not None and not args.noise_var > 0:
            raise ValueError("--noise-var must be > 0")

        params = load_params(args.params or RX_DIR / PARAMS_FILENAME)
        n_ant = antennas_in_run(params)
        in_name = args.input or Path(target_name(get_value(params, FILE_SECTION, INPUT_FILE_KEY), args.ray, n_ant))
        in_path = in_name if in_name.is_absolute() else RX_DIR / in_name
        out_name = args.out or Path(target_name(get_value(params, FILE_SECTION, OUTPUT_FILE_KEY), args.ray, n_ant))
        log.info("Input: %s", target_label(args.ray, n_ant))
        out_path = out_name if out_name.is_absolute() else RX_DIR / out_name

        demapper = load_demapper()
        mod, nbpsc = bits_per_symbol(demapper, get_value(params, DEFINED_SECTION, "Mod"))

        mode = demapping_mode(params, args.demapping)
        soft = mode == "soft"
        symbols, sym_noise_var, in_attrs = load_symbols(in_path)
        if "Mod" in in_attrs and str(in_attrs["Mod"]) != mod:
            log.warning("%s was written for Mod %s but the JSON says %s", in_path.name, in_attrs["Mod"], mod)
        log.info("Read %d symbols from %s", symbols.size, in_path)

        if not soft:
            nv_in, nv_source = None, None
        elif args.noise_var is not None:
            nv_in, nv_source = args.noise_var, "given"
        elif sym_noise_var is not None:
            nv_in, nv_source = sym_noise_var, "per-symbol"
        else:
            nv_in, nv_source = None, "estimated"
            log.warning("%s has no '%s' (unequalized input?) - using one estimated noise value for "
                        "the whole packet", in_path.name, NOISE_DATASET)

        Bitstream, noise_var = demap(symbols, mod, demapper, soft, nv_in)
        log.info("%s %s demapping: %d symbols -> %d %s (%d bits/symbol)", mod, mode, symbols.size,
                 Bitstream.size, "soft values" if soft else "bits", nbpsc)
        nv_summary = None
        if soft:
            nv_summary = float(np.median(noise_var)) if np.ndim(noise_var) else float(noise_var)
            if np.ndim(noise_var):
                log.info("Noise variance per symbol (from the equalizer): median %.3e (SNR ~ %.1f dB), "
                         "10%%-90%% range %.1f to %.1f dB", nv_summary, -10 * np.log10(nv_summary),
                         -10 * np.log10(np.percentile(noise_var, 90)),
                         -10 * np.log10(np.percentile(noise_var, 10)))
            else:
                log.info("Noise variance %.3e (%s), SNR ~ %.1f dB", nv_summary, nv_source,
                         -10 * np.log10(nv_summary))

        n_data_sigs = params.get(CALCULATED_SECTION, {}).get("NumberOfDataSigs")
        if isinstance(n_data_sigs, int) and not isinstance(n_data_sigs, bool):
            if Bitstream.size != n_data_sigs:
                raise ValueError(f"len(Bitstream) = {Bitstream.size} but NumberOfDataSigs = {n_data_sigs}"
                                 " - run Initialize_Rx.py")
            log.info("len(Bitstream) = %d = NumberOfDataSigs", Bitstream.size)
        else:
            log.warning("NumberOfDataSigs not usable in %s - length not checked", PARAMS_FILENAME)

        attrs = {"Mod": mod, "BitsPerSymbol": nbpsc, "Mode": mode,
                 "NumberOfDataSigs": Bitstream.size, "SourceFile": in_path.name,
                 "Ray": args.ray if args.ray is not None else (0 if n_ant > 1 else 1)}
        if soft:
            attrs["NoiseVar"] = nv_summary
            attrs["NoiseVarSource"] = nv_source
        save_bits(out_path, Bitstream, attrs)
        log.info("Bitstream saved to %s", out_path)
    except (FileNotFoundError, ValueError) as exc:
        log.error("%s", exc)
        return 1
    except Exception:
        log.exception("QAM demapping failed")
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
