#!/usr/bin/env python3
"""
Process_Control_Field_Rx.py  (lives in Flexlink_py/Rx)

Step 4a of the Rx chain: decode the control field that sits between the preamble
and the payload, set Defined_Rx from it, and cut it off so the payload starts at
sample 0 again for Demodulate_Waveform_Rx.py.

Runs only when Defined_Rx/Use_ControlField is 1, the default (it must match the Tx setting).

The control field format is fixed by design and read from
Tx/ControlSignals/Parameters_Control_Field.json (PacketLength 4, RefSignalSpacing 3,
RefSymbolSpacing 3, BPSK, BCC K=7 133/171 with 6 tail bits, CRC Gen10, 15 copies,
100 zero-padding bits, Gold code 130, and the ControlField layout).

Steps
-----
For every antenna (the packet files Packet_Detector.py wrote, --inputs):
  1. Take the first PacketLength (4) OFDM symbols and demodulate them with
     demodulate_waveform() from Demodulate_Waveform_Rx.py (FFT_StartTime from Defined_Rx).
  2. Equalize them with their own two reference symbols, using equalize_payload_grid()
     from Equalize_Payload_Grid.py (ReferenceSignals/Ref_P0_3_1.h5) with
     Defined_Rx/ControlFieldEqualizerMode (or --cf-mode):
       FIR8 (default)  the FPGA receiver's estimator: fixed 8-tap LMMSE across frequency,
                       drift-corrected linear interpolation across time, so the simulator
                       and the board decode the control field the same way
       EqualizerMode   use the payload's Defined_Rx/EqualizerMode (or --mode) and
                       EqualizerMaxDoppler, as before
       any EqualizerMode value (LMMSE, LMMSE_LinearTime, PCA, Linear, None)
  3. Soft BPSK values of the 2200 Data signals: LLR = 4 Re(x) / noise_var
     (positive = bit 1, the mapping Construct_ControlSigs_QAM.py uses).
Then, once:
  4. Add the antennas' LLRs (maximum-ratio combining).
  5. Descramble with the Gold code for C_init_Dat, drop the ZeroPadding, add the
     Repetition + 1 copies of the 140-bit block, deinterleave (Interleave_BCC).
  6. Viterbi decode (ConvolutionalEncoder.ViterbiDecoder, soft, EndState 0) to 70 bits,
     check the CRC over the first 64. A failed CRC flags a bad packet (exit 4).
  7. Split the 54 message bits into the ControlField values (parse_control_message and
     CONTROL_FIELD_CODING from Initialize_Control_Field.py). A code with no meaning
     also flags a bad packet.
  8. Compare with Defined_Rx (Tx_Ref_Clk_Cnt and Reserved are not compared; C_init is
     compared with Defined_Rx/C_init_Dat, the payload scrambler id):
       same      -> nothing is written, the chain continues
       different -> the new values are written to Defined_Rx and Initialize_Rx.py is run
                    (not when only C_init_Dat changed: it does not affect Calculated_Rx)
     NDB_A is then checked against Calculated_Rx/AvailableDataBlocks (warning only).
  9. Cut the control field off every packet file (rewritten in place, attribute
     ControlFieldRemoved = 1, so running this step twice on the same file is refused).
 10. Save File_Rx/ControlField_Rx (default ControlField_Rx.h5).

Usage
-----
    python Rx/Process_Control_Field_Rx.py                        # ray 1 packet file
    python Rx/Process_Control_Field_Rx.py --inputs Rx/Decimated_Payload_Packet_Rx_Ray1.h5 \\
                                                   Rx/Decimated_Payload_Packet_Rx_Ray2.h5
    python Rx/Process_Control_Field_Rx.py --no-update   # decode and compare only; nothing written
                                                        # to the JSON or the packet files
    python Rx/Process_Control_Field_Rx.py --cf-mode LMMSE   # the simulator's adaptive LMMSE instead

Exit status: 0 control field decoded (Defined_Rx unchanged or updated), 1 an error,
4 bad packet (control field CRC failed or a field has no valid value).

The output .h5 file contains:
    ControlMessage  (54,)    uint8    decoded message bits
    LLR             (2200,)  float64  combined soft values before descrambling
    attrs           CRC (PASS/FAIL), Rays, EqualizerMode, SNR_dB_Ray<N>, every field's code
                    (<field>) and Rx value (Value_<field>), DefinedRxChanged (names),
                    InitializeRx (0/1)
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path
from types import ModuleType
from typing import Any

import numpy as np

# Make flexlink_paths importable when the script is run from anywhere.
for _parent in Path(__file__).resolve().parents:
    if (_parent / "flexlink_paths.py").is_file():
        sys.path.insert(0, str(_parent))
        break
from flexlink_paths import (ROOT, TX_CONTROL_SIGNALS, CONSTRUCT_CONTROL_FIELD, CONSTRUCT_GRID,  # noqa: E402
                            INTERLEAVER, BCC_ENCODER, CRC_GEN, SCRAMBLER, ray_file_name)

try:
    from flexlink_paths import REFERENCE_SIGNALS  # noqa: E402
except ImportError:
    REFERENCE_SIGNALS = ROOT / "ReferenceSignals"

for _d in (CONSTRUCT_GRID, INTERLEAVER, BCC_ENCODER, CRC_GEN, SCRAMBLER, CONSTRUCT_CONTROL_FIELD):
    if str(_d) not in sys.path:
        sys.path.insert(0, str(_d))

RX_DIR = Path(__file__).resolve().parent
PARAMS_FILE = RX_DIR / "Parameters_Rx_payload.json"
CONTROL_PARAMS_FILE = TX_CONTROL_SIGNALS / "Parameters_Control_Field.json"
PACKET_KEY = ("Waveform_Payload_Packet_Rx", "Decimated_Payload_Packet_Rx.h5")
OUTPUT_KEY = ("ControlField_Rx", "ControlField_Rx.h5")
FS = 20e6
SYMBOL_LENGTH = 1024 + 116                  # 20 MHz OFDM symbol (IFFT + CP)
EXIT_BAD_PACKET = 4
DEFAULT_CF_MODE = "FIR8"                    # Defined_Rx/ControlFieldEqualizerMode default
CF_MODE_FROM_PAYLOAD = "EqualizerMode"      # value meaning: use the payload's EqualizerMode

# ControlField -> Defined_Rx key it sets. Fields not listed (Tx_Ref_Clk_Cnt, Reserved,
# NDB_A) are not compared; NDB_A is checked against Calculated_Rx/AvailableDataBlocks.
# C_init is the payload's Gold code id: it goes to Defined_Rx/C_init_Dat, which
# Get_Decoded_Bitstream.py (step 9) descrambles with. The control field itself is
# always descrambled with its own C_init_Dat (130) from Parameters_Control_Field.json.
FIELD_TO_DEFINED_RX = {
    "RefSymbolSpacing": "RefSymbolSpacing",
    "RefSignalSpacing": "RefSignalSpacing",
    "PacketLength":     "PacketLength",
    "PayloadFieldMod":  "Mod",
    "NoOfTxAntenna":    "NumberOfTxAntennas",
    "CBS_A_Flag":       "EncoderBlockSize",
    "FEC_A_Flag":       "EncoderCodeRate",
    "RM_A_Flag":        "Repetition",
    "ClientFlag":       "ClientFlag",
    "C_init":           "C_init_Dat",
}
# Defined_Rx keys that do not change Calculated_Rx or the resource grid: when only these
# change, they are written to Defined_Rx without running Initialize_Rx.py.
NO_INITIALIZE_KEYS = {"C_init_Dat"}


class BadPacket(Exception):
    """The control field could not be trusted (CRC failed or an invalid field value)."""


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #
def load_module(name: str, path: Path) -> ModuleType:
    if not path.is_file():
        raise FileNotFoundError(f"{path} not found")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def load_json(path: Path) -> dict[str, Any]:
    if not Path(path).is_file():
        raise FileNotFoundError(f"Parameter file not found: {path}")
    return json.loads(Path(path).read_text(encoding="utf-8"))


def save_json(path: Path, params: dict[str, Any]) -> None:
    fd, tmp = tempfile.mkstemp(dir=Path(path).parent, prefix=Path(path).name, suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            json.dump(params, fh, indent=2)
            fh.write("\n")
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


def file_name(params: dict[str, Any], key_default: tuple[str, str]) -> str:
    key, default = key_default
    name = params.get("File_Rx", {}).get(key)
    return name.strip() if isinstance(name, str) and name.strip() else default


def use_controlfield(defined: dict[str, Any]) -> int:
    value = defined.get("Use_ControlField", 1)
    if str(value).strip() not in ("0", "1"):
        raise ValueError(f"Use_ControlField {value!r} must be 0 or 1")
    return int(str(value).strip())


def to_complex(arr: np.ndarray) -> np.ndarray:
    if arr.dtype.names:
        names = {n.lower(): n for n in arr.dtype.names}
        re, im = names.get("real") or names.get("r"), names.get("imag") or names.get("i")
        if re and im:
            return arr[re].astype(float) + 1j * arr[im].astype(float)
    return arr.astype(np.complex128)


# --------------------------------------------------------------------------- #
# The control field
# --------------------------------------------------------------------------- #
class ControlFieldRx:
    """Everything the receiver knows about the control field, from the Tx control JSON."""

    def __init__(self, control_params: dict[str, Any]):
        import h5py
        from Construct_Resource_Grid import Resource_Grid, SignalType

        d, c = control_params["Defined"], control_params["Calculated"]
        self.d, self.c = d, c
        self.widths = control_params["ControlField"]
        self.packet_length = int(d["PacketLength"])
        self.length = self.packet_length * SYMBOL_LENGTH          # samples at 20 MHz
        if d["Mod"] != "BPSK":
            raise ValueError(f"Control field Mod is {d['Mod']}; this step handles BPSK")
        self.port = int(d["Port"])
        grid = Resource_Grid(self.packet_length, d["RefSymbolSpacing"], d["RefSignalSpacing"])
        self.types = grid.signal_types
        self.codes = {"Ref_P0": SignalType.REF_P0.value, "Ref_P1": SignalType.REF_P1.value,
                      "DC": SignalType.DC.value, "Data": SignalType.DATA.value}
        self.spacing = int(d["RefSignalSpacing"])
        n_data = int((self.types == self.codes["Data"]).sum())
        if n_data != c["NumberOfDataSigs"]:
            raise ValueError(f"Control grid has {n_data} Data signals but NumberOfDataSigs is "
                             f"{c['NumberOfDataSigs']} - run Tx/Initialize_Tx.py")

        self.ref_path = Path(REFERENCE_SIGNALS) / f"Ref_P{self.port}_{self.spacing}_1.h5"
        if not self.ref_path.is_file():
            raise FileNotFoundError(f"{self.ref_path} not found - run "
                                    "ReferenceSignals/Generate_Reference_Signal_Vectors.py")
        with h5py.File(self.ref_path, "r") as f:
            self.ref = to_complex(np.asarray(f["data"][()])).ravel()
            ref_c_init = f.attrs.get("C_init")
        if ref_c_init is not None and int(ref_c_init) != int(d["C_init_ID1"]):
            raise ValueError(f"{self.ref_path.name} was made with C_init {int(ref_c_init)}, the control "
                             f"field uses C_init_ID1 {d['C_init_ID1']}")

    # 1-3: one antenna -> LLRs of the Data signals
    def antenna_llr(self, x20: np.ndarray, start_time: int, mode: str, max_doppler: float,
                    demod_mod: ModuleType, eq_mod: ModuleType) -> tuple[np.ndarray, float]:
        if x20.size < self.length:
            raise ValueError(f"Packet has {x20.size} samples, the control field alone is {self.length}")
        Y, _ = demod_mod.demodulate_waveform(x20, start_time, self.packet_length,
                                             demod_mod.load_demodulator())
        r = eq_mod.equalize_payload_grid(Y, self.types, self.codes, self.ref, self.port, mode,
                                         max_doppler, eq_mod.load_estimator(), self.spacing)
        data = self.types == self.codes["Data"]
        x, nv = r["data"][data], r["noise_var"][data]
        return 4.0 * x.real / nv, float(-10 * np.log10(r["noise"]))

    # 5-6: combined LLRs -> 54 message bits, CRC result
    def decode(self, llr: np.ndarray) -> tuple[np.ndarray, bool]:
        from Scrambler_GC import scrambler_gc
        from Interleave_BCC import Interleave_BCC
        from Convolutionalencoder import ViterbiDecoder
        from GenerateCRC import GenerateCRC

        d, c = self.d, self.c
        n = int(c["NumberOfDataSigs"])
        with tempfile.TemporaryDirectory() as tmp:
            gold = np.asarray(scrambler_gc(d["C_init_Dat"], np.zeros(n, dtype=np.uint8),
                                           h5_file=str(Path(tmp) / "GC.h5"))).ravel()
        llr = np.where(gold == 1, -llr, llr)                      # descramble
        cbs, copies = int(c["CodeBlockSize"]), int(c["Repetition"]) + 1
        block = llr[:cbs * copies].reshape(copies, cbs).sum(axis=0)   # drop padding, add copies
        idx = np.asarray(Interleave_BCC(np.arange(cbs), cbs)).ravel()
        code = np.empty(cbs)
        code[idx] = block                                         # deinterleave
        dec = np.asarray(ViterbiDecoder(code, d["ConstraintLength"], d["Polynomials"],
                                        bool(d["TailBiting"]), "SoftNonInverting",
                                        EndState=0)).ravel().astype(np.uint8)
        data_blk = dec[:int(c["DataBlockSize"])]
        msg_size = int(d["ControlMessageSize"])
        crc_ok = not np.asarray(GenerateCRC(data_blk, d["CRC"])).any()
        return data_blk[:msg_size], crc_ok

    # 7: message bits -> {field: (code, Rx value)}
    def fields(self, message: np.ndarray, icf: ModuleType) -> dict[str, tuple[int, Any]]:
        codes = icf.parse_control_message(message, self.widths)
        out = {}
        for name, code in codes.items():
            rule = icf.CONTROL_FIELD_CODING.get(name)
            if rule is None or rule[0] == "Fixed" or rule[2] is None:
                out[name] = (code, code)                          # plain number
                continue
            inverse = {v: k for k, v in rule[2].items()}
            if code not in inverse:
                raise BadPacket(f"ControlField {name} has code {code:0{self.widths[name]}b}, "
                                "which means nothing")
            out[name] = (code, inverse[code])
        return out


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def run(a: argparse.Namespace) -> int:
    import h5py

    params = load_json(a.params)
    defined = params.get("Defined_Rx", {})
    if not use_controlfield(defined):
        print("Use_ControlField 0: no control field expected - nothing to do")
        return 0

    cf = ControlFieldRx(load_json(CONTROL_PARAMS_FILE))
    demod_mod = load_module("Demodulate_Waveform_Rx", RX_DIR / "Demodulate_Waveform_Rx.py")
    eq_mod = load_module("Equalize_Payload_Grid", RX_DIR / "Equalize_Payload_Grid.py")
    icf = load_module("Initialize_Control_Field", CONSTRUCT_CONTROL_FIELD / "Initialize_Control_Field.py")

    start_time = int(defined.get("FFT_StartTime", 0))
    cf_mode = str(a.cf_mode or defined.get("ControlFieldEqualizerMode") or DEFAULT_CF_MODE).strip()
    if cf_mode.lower() == CF_MODE_FROM_PAYLOAD.lower():
        mode = eq_mod.check_mode(a.mode or defined.get("EqualizerMode") or eq_mod.DEFAULT_MODE)
    else:
        mode = eq_mod.check_mode(cf_mode)
    max_doppler = float(a.max_doppler or defined.get("EqualizerMaxDoppler") or eq_mod.DEFAULT_MAX_DOPPLER)

    inputs = a.inputs or [RX_DIR / ray_file_name(file_name(params, PACKET_KEY), 1)]
    packets, llr_sum, snr = [], None, {}
    for path in map(Path, inputs):
        if not path.is_file():
            raise FileNotFoundError(f"{path} not found - run Packet_Detector.py first")
        with h5py.File(path, "r") as f:
            x = to_complex(np.asarray(f["data"][()])).ravel()
            attrs = dict(f.attrs)
        if int(attrs.get("ControlFieldRemoved", 0)):
            raise ValueError(f"{path.name} has already had its control field removed - "
                             "run Packet_Detector.py again")
        if attrs.get("SampleRate") is not None and float(attrs["SampleRate"]) != FS:
            raise ValueError(f"{path.name} is not at 20 MHz")
        ray = int(attrs.get("Ray", len(packets) + 1))
        llr, snr[ray] = cf.antenna_llr(x, start_time, mode, max_doppler, demod_mod, eq_mod)
        llr_sum = llr if llr_sum is None else llr_sum + llr
        packets.append((path, x, attrs, ray))
        print(f"Ray {ray}: control field {cf.packet_length} symbols ({cf.length} samples) from "
              f"{path.name}, EqualizerMode {mode}, SNR ~ {snr[ray]:.1f} dB")

    message, crc_ok = cf.decode(llr_sum)
    rays = [p[3] for p in packets]
    out_path = a.out or RX_DIR / file_name(params, OUTPUT_KEY)
    out_attrs: dict[str, Any] = {"CRC": "PASS" if crc_ok else "FAIL", "Rays": np.array(rays),
                                 "EqualizerMode": mode,
                                 **{f"SNR_dB_Ray{r}": v for r, v in snr.items()}}

    def save() -> None:
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with h5py.File(out_path, "w") as f:
            f.create_dataset("ControlMessage", data=message)
            f.create_dataset("LLR", data=llr_sum)
            for k, v in out_attrs.items():
                f.attrs[k] = v

    print(f"Combined {len(rays)} antenna(s), {cf.c['Repetition'] + 1} copies -> "
          f"Viterbi -> control field CRC {out_attrs['CRC']}")
    if not crc_ok:
        save()
        raise BadPacket("control field CRC failed")

    try:
        fields = cf.fields(message, icf)
    except BadPacket:
        out_attrs["CRC"] = "PASS_INVALID_FIELD"
        save()
        raise
    for name, (code, value) in fields.items():
        out_attrs[name] = code
        out_attrs[f"Value_{name}"] = value if isinstance(value, (int, float)) else str(value)
        print(f"  {name:<17} {code:0{cf.widths[name]}b}  -> {value}")

    # 8. compare with Defined_Rx
    changed = {}
    for name, key in FIELD_TO_DEFINED_RX.items():
        if name in fields and defined.get(key) != fields[name][1]:
            changed[key] = (defined.get(key, "(missing)"), fields[name][1])
    out_attrs["DefinedRxChanged"] = ", ".join(changed) or "none"
    out_attrs["InitializeRx"] = 0

    if not changed:
        print("Defined_Rx already matches the control field - Initialize_Rx.py not needed")
    elif a.no_update:
        for key, (old, new) in changed.items():
            print(f"  Defined_Rx/{key}: {old} -> {new} (not written, --no-update)")
    else:
        for key, (old, new) in changed.items():
            print(f"  Defined_Rx/{key}: {old} -> {new}")
            params["Defined_Rx"][key] = new
        save_json(a.params, params)
        if set(changed) <= NO_INITIALIZE_KEYS:
            print(f"Defined_Rx updated in {Path(a.params).name} - Initialize_Rx.py not needed")
    if changed and not a.no_update and not set(changed) <= NO_INITIALIZE_KEYS:
        print(f"Defined_Rx updated in {Path(a.params).name} - running Initialize_Rx.py")
        p = subprocess.run([sys.executable, str(RX_DIR / "Initialize_Rx.py"), "--params", str(a.params)],
                           capture_output=True, text=True)
        if p.returncode != 0:
            print(p.stdout + p.stderr, file=sys.stderr)
            raise ValueError(f"Initialize_Rx.py stopped with exit {p.returncode}")
        out_attrs["InitializeRx"] = 1
        params = load_json(a.params)
        print("Initialize_Rx.py done")

    ndb = fields.get("NDB_A", (None, None))[1]
    avail = params.get("Calculated_Rx", {}).get("AvailableDataBlocks")
    if ndb is not None and avail is not None and int(ndb) != int(avail):
        print(f"Warning: control field NDB_A = {ndb} but Calculated_Rx/AvailableDataBlocks = {avail}")

    # 9. cut the control field off the packet files
    if not a.no_update:
        for path, x, attrs, ray in packets:
            attrs.update({"ControlFieldRemoved": 1, "ControlLength": cf.length})
            if "PayloadStart" in attrs:
                attrs["ControlStart"] = attrs["PayloadStart"]
                attrs["PayloadStart"] = int(attrs["PayloadStart"]) + cf.length
            with h5py.File(path, "w") as f:
                f.create_dataset("data", data=x[cf.length:])
                for k, v in attrs.items():
                    f.attrs[k] = v
        print(f"Control field ({cf.length} samples) removed from {len(packets)} packet file(s)")

    save()
    print(f"Wrote {out_path}")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Decode the control field and set Defined_Rx from it.")
    ap.add_argument("--params", type=Path, default=PARAMS_FILE)
    ap.add_argument("--inputs", type=Path, nargs="+",
                    help="packet files from Packet_Detector.py, one per antenna (default: ray 1)")
    ap.add_argument("--out", type=Path, help="output .h5 (default: Rx/<File_Rx.ControlField_Rx>)")
    ap.add_argument("--cf-mode", help="override Defined_Rx/ControlFieldEqualizerMode (default FIR8; "
                                      "'EqualizerMode' = the payload's mode)")
    ap.add_argument("--mode", help="override Defined_Rx/EqualizerMode (used only with "
                                   "ControlFieldEqualizerMode EqualizerMode)")
    ap.add_argument("--max-doppler", type=float, help="override Defined_Rx/EqualizerMaxDoppler (Hz)")
    ap.add_argument("--no-update", action="store_true",
                    help="decode and compare only: write neither the JSON nor the packet files")
    a = ap.parse_args(argv)
    try:
        if importlib.util.find_spec("h5py") is None:
            raise ValueError("Reading/writing .h5 files needs h5py - install it with: pip install h5py")
        return run(a)
    except BadPacket as exc:
        print(f"Bad packet: {exc}", file=sys.stderr)
        return EXIT_BAD_PACKET
    except (FileNotFoundError, KeyError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
