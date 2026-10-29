"""
cf_fields.py  (Flexlink_py/HW_Development/Packet_Detector/PC)

Turns the 54 message bits of a control record (pd_protocol.ControlRecord.message, bit i =
message bit i) into the control-field values, with the simulator's own parser and coding table
(Flexlink_BasicModules/Construct_Control_Field/Initialize_Control_Field.py:
parse_control_message, CONTROL_FIELD_CODING; field widths from
Tx/ControlSignals/Parameters_Control_Field.json). Without the simulator project next to this
folder, decode() returns None.

    import cf_fields
    f = cf_fields.decode(rec.message)      # {"PacketLength": (0b100, 73), ...} or None
    print(cf_fields.describe(rec.message))
    tx = cf_fields.tx_message()            # the message the Tx wrote (needs h5py), or None
"""
from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
_ROOT = next((p for p in [HERE, *HERE.parents] if (p / "flexlink_paths.py").is_file()), None)
_mod = None
_widths = None


def _load():
    global _mod, _widths
    if _mod is not None or _ROOT is None:
        return _mod
    path = _ROOT / "Flexlink_BasicModules" / "Construct_Control_Field" / "Initialize_Control_Field.py"
    js = _ROOT / "Tx" / "ControlSignals" / "Parameters_Control_Field.json"
    if not path.is_file() or not js.is_file():
        return None
    try:
        spec = importlib.util.spec_from_file_location("Initialize_Control_Field", path)
        m = importlib.util.module_from_spec(spec)
        sys.modules["Initialize_Control_Field"] = m
        spec.loader.exec_module(m)
        _widths = json.loads(js.read_text())["ControlField"]
        _mod = m
    except Exception:                                   # simulator not usable here
        _mod = None
    return _mod


def decode(message: int, n_bits: int = 54) -> dict[str, tuple[int, object]] | None:
    """{field: (code, value)}; value is None for a code with no meaning."""
    m = _load()
    if m is None:
        return None
    bits = np.array([(message >> i) & 1 for i in range(n_bits)], np.int64)
    codes = m.parse_control_message(bits, _widths)
    out = {}
    for name, code in codes.items():
        rule = m.CONTROL_FIELD_CODING.get(name)
        if rule is None or rule[0] == "Fixed" or rule[2] is None:
            out[name] = (code, code)
        else:
            inv = {v: k for k, v in rule[2].items()}
            out[name] = (code, inv.get(code))
    return out


def describe(message: int) -> str:
    f = decode(message)
    if f is None:
        return f"message 0x{message:014x} (field decoding needs the simulator project)"
    return ", ".join(f"{k} {v if v is not None else '?' + format(c, 'b')}" for k, (c, v) in f.items())


def tx_message() -> int | None:
    """The 54-bit message of Tx/ControlSignals (Bitstream_ControlBlks_Tx), as an int; None if the
    file or h5py is missing."""
    if _ROOT is None:
        return None
    f = _ROOT / "Tx" / "ControlSignals" / "Bitstream_ControlBlks_Tx.h5"
    try:
        import h5py
        with h5py.File(f, "r") as h:
            bits = np.asarray(h["ControlMessage"][()]).ravel().astype(int)
    except Exception:
        return None
    return sum(int(b) << i for i, b in enumerate(bits[:54]))
