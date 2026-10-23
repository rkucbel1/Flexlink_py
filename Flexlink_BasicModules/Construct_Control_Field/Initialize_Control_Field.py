#!/usr/bin/env python3
"""
Initialize_Control_Field.py  (lives in Flexlink_BasicModules/Construct_Control_Field)

Initialize the Flexlink control field: the 4-symbol field (Ref, Data, Data, Ref)
sent between the preamble and the payload.

Steps:
  1. Read Tx/ControlSignals/Parameters_Control_Field.json (flexlink_paths.TX_CONTROL_SIGNALS)
     and check every Defined value against what the control field allows.
  2. Encoder sizes:
         CRClen           = number at the end of CRC              ('Gen10' -> 10)
         DataBlockSize    = ControlMessageSize + CRClen           (54 + 10 = 64)
         TailBits         = ConstraintLength - 1                  (7 - 1 = 6, TailBiting 0)
         EncoderInputSize = DataBlockSize + TailBits              (64 + 6 = 70)
         CodeBlockSize    = EncoderInputSize / EncoderCodeRate    (70 / (1/2) = 140)
     CodeBlockSize must be a length Interleave_BCC supports.
  3. Build Resource_Grid(PacketLength, RefSymbolSpacing, RefSignalSpacing) and count
     Ref_P0, Ref_P1, DC and Data signals. NumberOfDataSigs = Data signals x 1 (BPSK).
  4. Fill the field with as many whole code blocks as fit:
         copies                = NumberOfDataSigs // CodeBlockSize    (2200 // 140 = 15)
         Repetition            = copies - 1                           (extra copies, 14)
         CodeWordSize          = copies x CodeBlockSize               (2100)
         ZeroPadding           = NumberOfDataSigs - CodeWordSize      (100)
         ScrambledSequenceSize = CodeWordSize + ZeroPadding           (= NumberOfDataSigs)
     Stops with an error if not even one code block fits.
  5. Build the ControlMessage (ControlMessageSize = 54 bits) from the payload
     parameters in Tx/Parameters_Tx_payload.json (C_init is Defined/C_init_Dat, the
     payload's scrambler id, 0 to 255; the control field itself is always scrambled
     with its own C_init_Dat 130). The ControlField section of
     Parameters_Control_Field.json lists the fields in transmit order with their
     widths in bits; each field is coded with CONTROL_FIELD_CODING below and
     written MSB first. Stops with an error if the payload's Calculated values are
     out of date (run Initialize_Tx.py first).
  6. Save the grid to File/Grid_ResourceControlField_Tx and the ControlMessage to
     File/Bitstream_ControlBlks_Tx, both in Tx/ControlSignals, then write the
     Calculated section back to the JSON. Defined, File and ControlField are left
     as they are.

Run it after every change to Defined, and after every Initialize_Tx.py run.

Usage:
    python Flexlink_BasicModules/Construct_Control_Field/Initialize_Control_Field.py
    python .../Initialize_Control_Field.py --dry-run -v     # compute only, write nothing
    python .../Initialize_Control_Field.py --params my.json --payload-params other.json

Bitstream_ControlBlks_Tx.h5 contains:
    ControlMessage   (54,) uint8   the control message bits, fields in ControlField order
    attrs            <field> = coded value of each field, Bits_<field> = its width,
                     PayloadParamsFile
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import re
import sys
import tempfile
from fractions import Fraction
from pathlib import Path
from typing import Any

import numpy as np

# Locate flexlink_paths.py by walking up from this script's folder.
for _parent in Path(__file__).resolve().parents:
    if (_parent / "flexlink_paths.py").is_file():
        sys.path.insert(0, str(_parent))
        break
from flexlink_paths import TX, TX_CONTROL_SIGNALS, CONSTRUCT_GRID, INTERLEAVER  # noqa: E402

for _d in (CONSTRUCT_GRID, INTERLEAVER):
    if str(_d) not in sys.path:
        sys.path.insert(0, str(_d))
from Construct_Resource_Grid import Resource_Grid, SignalType  # noqa: E402
from Interleave_BCC import Interleave_BCC                     # noqa: E402

PARAMS_FILENAME = "Parameters_Control_Field.json"
GRID_FILE_KEY = "Grid_ResourceControlField_Tx"

# Allowed values for each Defined key. A key missing from Defined is an error.
ALLOWED = {
    "ControlMessageSize": None,          # any positive integer (checked below)
    "CRC":                ("Gen10",),
    "Encoder":            ("BCC",),
    "ConstraintLength":   (7,),
    "Polynomials":        ([133, 171],),
    "EncoderCodeRate":    ("1/2",),
    "TailBiting":         (0,),
    "Mod":                ("BPSK",),
    "PacketLength":       (4,),
    "RefSignalSpacing":   (3,),
    "RefSymbolSpacing":   (3,),
    "Port":               (0,),
    "C_init_ID1":         None,          # positive integer
    "C_init_Dat":         (130,),        # fixed for every Flexlink control field
    "SampleRate":         None,          # positive integer
}
MOD_BITS = {"BPSK": 1}

PAYLOAD_PARAMS_FILENAME = "Parameters_Tx_payload.json"
MESSAGE_FILE_KEY = "Bitstream_ControlBlks_Tx"
MESSAGE_DATASET = "ControlMessage"

# How each ControlField is coded.
#   (section, key, table): the payload value Tx/Parameters_Tx_payload.json[section][key]
#                          is looked up in table -> integer code
#   (section, key, None) : the payload value itself, as an unsigned binary number
#   ("Fixed", value)     : a constant (not yet taken from anywhere)
# The width of each field (bits) comes from the ControlField section of
# Parameters_Control_Field.json; every code must fit in that width.
CONTROL_FIELD_CODING = {
    "RefSymbolSpacing": ("Defined", "RefSymbolSpacing", {1: 0b00, 3: 0b01, 6: 0b10, 12: 0b11}),
    "RefSignalSpacing": ("Defined", "RefSignalSpacing", {3: 0b00, 6: 0b01, 12: 0b10, 24: 0b11}),
    "PacketLength":     ("Defined", "PacketLength",
                         {4: 0b001, 37: 0b011, 49: 0b111, 73: 0b100, 97: 0b110, 121: 0b101}),
    "PayloadFieldMod":  ("Defined", "Mod", {"BPSK": 0b00, "QPSK": 0b01, "16QAM": 0b10, "64QAM": 0b11}),
    "NoOfTxAntenna":    ("Defined", "NumberOfAntennas", {1: 0b0, 2: 0b1}),
    "CBS_A_Flag":       ("Defined", "EncoderBlockSize", {648: 0b00, 1296: 0b01, 1944: 0b10}),
    "FEC_A_Flag":       ("Defined", "EncoderCodeRate", {"1/2": 0b00, "2/3": 0b01, "3/4": 0b10, "5/6": 0b11}),
    "RM_A_Flag":        ("Defined", "Repetition",
                         {1: 0b0000, 3: 0b0001, 5: 0b0010, 7: 0b0011, 11: 0b0100,
                          15: 0b0101, 19: 0b0110, 23: 0b0111, 31: 0b1000}),
    "NDB_A":            ("Calculated", "AvailableDataBlocks", None),
    "ClientFlag":       ("Defined", "ClientFlag", {"Service": 0b0, "Client": 0b1}),
    "Tx_Ref_Clk_Cnt":   ("Fixed", 0),      # unused for now; later 0 .. 16383
    "C_init":           ("Defined", "C_init_Dat", None),   # payload scrambler id, 0 .. 255
    "Reserved":         ("Fixed", 0),
}

log = logging.getLogger("Initialize_Control_Field")


# --------------------------------------------------------------------------- #
# Parameters
# --------------------------------------------------------------------------- #
def load_params(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"Parameter file not found: {path}")
    try:
        params = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON in {path.name}: line {exc.lineno}, col {exc.colno}: {exc.msg}") from exc
    for section in ("Defined", "File"):
        if not isinstance(params.get(section), dict):
            raise ValueError(f"{path.name} has no '{section}' section")
    return params


def check_defined(defined: dict[str, Any]) -> None:
    errors = []
    for key, allowed in ALLOWED.items():
        if key not in defined:
            errors.append(f"{key} is missing")
            continue
        value = defined[key]
        if allowed is None:
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                errors.append(f"{key} must be a positive integer, got {value!r}")
        elif value not in allowed:
            errors.append(f"{key} must be {' or '.join(map(repr, allowed))}, got {value!r}")
    if errors:
        raise ValueError("Defined values not allowed for the control field:\n  " + "\n  ".join(errors))


def save_params(path: Path, params: dict[str, Any]) -> None:
    """Write JSON atomically so a crash can't leave a half-written file."""
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=path.name, suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            json.dump(params, fh, indent=2)
            fh.write("\n")
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


# --------------------------------------------------------------------------- #
# Calculations
# --------------------------------------------------------------------------- #
def encoder_sizes(d: dict[str, Any]) -> dict[str, int]:
    m = re.fullmatch(r"[A-Za-z_]*(\d+)", str(d["CRC"]).strip())
    crc_len = int(m.group(1))
    data_block = d["ControlMessageSize"] + crc_len
    tail_bits = d["ConstraintLength"] - 1 if d["TailBiting"] == 0 else 0
    enc_in = data_block + tail_bits
    code_block = Fraction(enc_in) / Fraction(d["EncoderCodeRate"])
    if code_block.denominator != 1:
        raise ValueError(f"EncoderInputSize {enc_in} / EncoderCodeRate {d['EncoderCodeRate']} "
                         "is not a whole number of bits")
    code_block = int(code_block)

    try:
        Interleave_BCC(np.zeros(code_block, dtype=np.uint8), code_block)
    except ValueError as exc:
        raise ValueError(f"CodeBlockSize {code_block} has no BCC interleaver ({exc}). "
                         f"Change ControlMessageSize or CRC, or add a permutation of length "
                         f"{code_block} to Interleaver/Interleave_BCC.py") from None

    log.info("CRClen = %d (%s)", crc_len, d["CRC"])
    log.info("DataBlockSize = %d + %d = %d", d["ControlMessageSize"], crc_len, data_block)
    log.info("EncoderInputSize = %d + %d tail bits = %d", data_block, tail_bits, enc_in)
    log.info("CodeBlockSize = %d / (%s) = %d", enc_in, d["EncoderCodeRate"], code_block)
    return {"CRClen": crc_len, "DataBlockSize": data_block, "TailBits": tail_bits,
            "EncoderInputSize": enc_in, "CodeBlockSize": code_block}


def grid_counts(grid: Resource_Grid, mod: str) -> dict[str, int]:
    counts = {
        "NumberOfRefSigs_P0": grid.count_of(SignalType.REF_P0),
        "NumberOfRefSigs_P1": grid.count_of(SignalType.REF_P1),
        "NumberOfDCSigs":     grid.count_of(SignalType.DC),
        "NumberOfDataSigs":   grid.count_of(SignalType.DATA) * MOD_BITS[mod],
    }
    log.info("Grid %s %s: %s", grid.shape, "".join(t.name[0] for t in grid.types), counts)
    return counts


def fill_sizes(n_data_sigs: int, code_block: int) -> dict[str, int]:
    copies = n_data_sigs // code_block
    if copies < 1:
        raise ValueError(f"CodeBlockSize {code_block} is bigger than NumberOfDataSigs {n_data_sigs}: "
                         "the control field can't hold a single code block")
    code_word = copies * code_block
    zero_pad = n_data_sigs - code_word
    log.info("Copies = %d // %d = %d -> Repetition = %d", n_data_sigs, code_block, copies, copies - 1)
    log.info("CodeWordSize = %d x %d = %d, ZeroPadding = %d", copies, code_block, code_word, zero_pad)
    return {"Repetition": copies - 1, "CodeWordSize": code_word, "ZeroPadding": zero_pad,
            "ScrambledSequenceSize": code_word + zero_pad}


def calculate(params: dict[str, Any]) -> tuple[dict[str, Any], Resource_Grid]:
    d = params["Defined"]
    check_defined(d)
    enc = encoder_sizes(d)
    grid = Resource_Grid(d["PacketLength"], d["RefSymbolSpacing"], d["RefSignalSpacing"])
    counts = grid_counts(grid, d["Mod"])
    fill = fill_sizes(counts["NumberOfDataSigs"], enc["CodeBlockSize"])
    if fill["ScrambledSequenceSize"] != counts["NumberOfDataSigs"]:
        raise ValueError("ScrambledSequenceSize does not equal NumberOfDataSigs")

    calc = {
        **enc,
        "Repetition": fill["Repetition"],
        "Repetition_comment": "Extra copies of the code block, the most that fit: "
                              "NumberOfDataSigs // CodeBlockSize - 1",
        "CodeWordSize": fill["CodeWordSize"],
        "ZeroPadding": fill["ZeroPadding"],
        **counts,
        "ScrambledSequenceSize": fill["ScrambledSequenceSize"],
    }
    return calc, grid


# --------------------------------------------------------------------------- #
# Control message
# --------------------------------------------------------------------------- #
def check_payload_current(payload: dict[str, Any]) -> None:
    """Stop if the payload's Calculated values don't match its Defined values."""
    d, c = payload.get("Defined", {}), payload.get("Calculated", {})
    try:
        n, rep = d["EncoderBlockSize"], d["Repetition"]
        expected = {"CodeBlockSize": n, "CodeWordSize": n * (rep + 1),
                    "DataBlockSize": int(n * Fraction(d["EncoderCodeRate"]))}
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{PAYLOAD_PARAMS_FILENAME}: can't check Calculated values ({exc})") from None
    stale = {k: (c.get(k), v) for k, v in expected.items() if c.get(k) != v}
    if stale:
        detail = ", ".join(f"{k} is {old}, expected {new}" for k, (old, new) in stale.items())
        raise ValueError(f"{PAYLOAD_PARAMS_FILENAME} Calculated values are out of date ({detail}) - "
                         "run Tx/Initialize_Tx.py first")


def field_code(name: str, payload: dict[str, Any]) -> tuple[int, Any]:
    """Return (integer code, source value) for one ControlField."""
    if name not in CONTROL_FIELD_CODING:
        raise ValueError(f"ControlField '{name}' has no coding in CONTROL_FIELD_CODING "
                         f"(known: {', '.join(CONTROL_FIELD_CODING)})")
    rule = CONTROL_FIELD_CODING[name]
    if rule[0] == "Fixed":
        return rule[1], rule[1]
    section, key, table = rule
    if key not in payload.get(section, {}):
        raise ValueError(f"ControlField '{name}' needs {section}/{key} in {PAYLOAD_PARAMS_FILENAME}")
    value = payload[section][key]
    if table is None:
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"ControlField '{name}': {section}/{key} = {value!r} is not a "
                             "non-negative integer")
        return value, value
    if value not in table:
        raise ValueError(f"ControlField '{name}': {section}/{key} = {value!r} has no code "
                         f"(allowed: {', '.join(map(str, table))})")
    return table[value], value


def build_control_message(widths: dict[str, int], payload: dict[str, Any],
                          message_size: int) -> tuple[np.ndarray, dict[str, int]]:
    """Concatenate every ControlField, MSB first, into the ControlMessage bit vector.

    widths   ControlField section of Parameters_Control_Field.json: name -> bits, in
             transmit order
    payload  Parameters_Tx_payload.json
    Returns (ControlMessage as uint8 array of message_size bits, name -> code).
    """
    total = sum(widths.values())
    if total != message_size:
        raise ValueError(f"ControlField widths add up to {total} bits, "
                         f"but ControlMessageSize is {message_size}")
    bits, codes = [], {}
    log.info("ControlMessage (%d bits):", message_size)
    for name, width in widths.items():
        if isinstance(width, bool) or not isinstance(width, int) or width <= 0:
            raise ValueError(f"ControlField '{name}' width must be a positive integer, got {width!r}")
        code, value = field_code(name, payload)
        if code >= 1 << width:
            raise ValueError(f"ControlField '{name}' = {value!r} (code {code}) "
                             f"does not fit in {width} bits (0 to {(1 << width) - 1})")
        field = format(code, f"0{width}b")
        bits.extend(int(b) for b in field)
        codes[name] = code
        log.info("  %-17s %2d bits  %-14s <- %s", name, width, field, value)
    return np.array(bits, dtype=np.uint8), codes


def parse_control_message(message: np.ndarray, widths: dict[str, int]) -> dict[str, int]:
    """Inverse of build_control_message: split the bits back into name -> integer code."""
    message = np.asarray(message).ravel()
    if message.size != sum(widths.values()):
        raise ValueError(f"ControlMessage has {message.size} bits, ControlField expects "
                         f"{sum(widths.values())}")
    codes, pos = {}, 0
    for name, width in widths.items():
        codes[name] = int("".join(map(str, message[pos:pos + width].astype(int))), 2)
        pos += width
    return codes


def save_message(path: Path, message: np.ndarray, codes: dict[str, int],
                 widths: dict[str, int], source: Path) -> None:
    import h5py
    with h5py.File(path, "w") as f:
        f.create_dataset(MESSAGE_DATASET, data=message)
        for name, code in codes.items():
            f.attrs[name] = code
            f.attrs[f"Bits_{name}"] = widths[name]
        f.attrs["PayloadParamsFile"] = source.name


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Initialize the Flexlink control field.")
    ap.add_argument("--params", type=Path, default=TX_CONTROL_SIGNALS / PARAMS_FILENAME,
                    help=f"path to {PARAMS_FILENAME} (default: Tx/ControlSignals)")
    ap.add_argument("--payload-params", type=Path, default=TX / PAYLOAD_PARAMS_FILENAME,
                    help=f"path to {PAYLOAD_PARAMS_FILENAME} (default: Tx)")
    ap.add_argument("--dry-run", action="store_true", help="compute only; write neither the JSON nor the .h5")
    ap.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(message)s", datefmt="%H:%M:%S")
    try:
        params = load_params(a.params)
        calc, grid = calculate(params)

        old = params.get("Calculated", {})
        changed = [k for k in calc if not k.endswith("_comment") and old.get(k) != calc[k]]
        for k in changed:
            log.info("  %-22s %s -> %s", k, old.get(k, "(new)"), calc[k])
        dropped = [k for k in old if k not in calc]
        if dropped:
            log.info("  Removed from Calculated: %s", ", ".join(dropped))

        grid_name = params["File"].get(GRID_FILE_KEY)
        if not isinstance(grid_name, str) or not grid_name.strip():
            raise ValueError(f"File/{GRID_FILE_KEY} not found in {a.params.name}")
        grid_path = TX_CONTROL_SIGNALS / grid_name.strip()

        # Control message from the payload parameters
        widths = params.get("ControlField")
        if not isinstance(widths, dict) or not widths:
            raise ValueError(f"{a.params.name} has no ControlField section")
        payload = json.loads(Path(a.payload_params).read_text(encoding="utf-8")) \
            if Path(a.payload_params).is_file() else None
        if payload is None:
            raise FileNotFoundError(f"Payload parameter file not found: {a.payload_params}")
        check_payload_current(payload)
        message, codes = build_control_message(widths, payload,
                                               params["Defined"]["ControlMessageSize"])
        if parse_control_message(message, widths) != codes:
            raise ValueError("ControlMessage does not split back into the same field values")
        msg_name = params["File"].get(MESSAGE_FILE_KEY)
        if not isinstance(msg_name, str) or not msg_name.strip():
            raise ValueError(f"File/{MESSAGE_FILE_KEY} not found in {a.params.name}")
        msg_path = TX_CONTROL_SIGNALS / msg_name.strip()
        log.info("ControlMessage = %s", "".join(map(str, message)))

        if a.dry_run:
            log.info("Dry run: %s, %s and %s not written", a.params.name, grid_path.name, msg_path.name)
            return 0

        try:
            import h5py  # noqa: F401
        except ImportError:
            raise ValueError("Saving the resource grid needs h5py - install it with: pip install h5py") from None
        # Save the grid first: if that fails, the JSON is left untouched.
        grid.save_h5(str(grid_path))
        log.info("Resource grid saved to %s", grid_path)
        save_message(msg_path, message, codes, widths, Path(a.payload_params))
        log.info("ControlMessage saved to %s", msg_path)
        params["Calculated"] = calc
        # Keep the section order Defined, Calculated, File, then any others (ControlField)
        params = {k: params[k] for k in ("Defined", "Calculated", "File")} | \
                 {k: v for k, v in params.items() if k not in ("Defined", "Calculated", "File")}
        save_params(a.params, params)
        log.info("Calculated values saved to %s (%d changed)", a.params, len(changed))
    except (FileNotFoundError, ValueError) as exc:
        log.error("%s", exc)
        return 1
    except Exception:
        log.exception("Control field initialization failed")
        return 3
    log.info("Control field initialized")
    return 0


if __name__ == "__main__":
    sys.exit(main())
