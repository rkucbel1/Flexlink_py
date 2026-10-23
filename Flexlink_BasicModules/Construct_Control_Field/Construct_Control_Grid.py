#!/usr/bin/env python3
"""
Construct_Control_Grid.py  (lives in Flexlink_BasicModules/Construct_Control_Field)

Fill the 4-symbol control field grid (Ref, Data, Data, Ref). The control field
version of Tx/Construct_Payload_Grid.py, and it uses that script's fill functions.

Inputs (all names from Parameters_Control_Field.json, files in Tx/ControlSignals)
------
File/Grid_ResourceControlField_Tx   'signal_types' (4, 841): 10=Ref_P0, 20=Ref_P1, 30=DC,
                                    40=Data (written by Initialize_Control_Field.py)
File/Sequence_ControlSigs_QAM       'data': 2200 BPSK values (Construct_ControlSigs_QAM.py)
ReferenceSignals/Ref_P0_3_1.h5      'data': the 280 Ref_P0 values of one Symbol_Ref,
                                    for RefSignalSpacing 3 and BPSK (1 bit/symbol).
                                    Its C_init attribute must equal Defined/C_init_ID1.

Rules
-----
* Data signals  <- the BPSK values in order: symbol 0 k=0..840, then symbol 1, ...
* Ref_P0        <- Ref_P0_3_1, the same values in symbols 0 and 3, in increasing k.
* Ref_P1 and DC <- 0 (the control field is sent from port 0 only).

Output
------
File/Grid_DataControl_Tx in Tx/ControlSignals:
    data          (4, 841) complex   control field values
    signal_types, symbol_types, k, Tone_i   copied from the resource grid
    attrs: the grid's attrs plus Port, RefSignalSpacing, Mod, BitsPerSymbol,
           C_init_ID1 and the names of the files used.

Usage
-----
    python Flexlink_BasicModules/Construct_Control_Field/Construct_Control_Grid.py
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

# Locate flexlink_paths.py by walking up from this script's folder.
for _parent in Path(__file__).resolve().parents:
    if (_parent / "flexlink_paths.py").is_file():
        sys.path.insert(0, str(_parent))
        break
from flexlink_paths import TX, TX_CONTROL_SIGNALS, REFERENCE_SIGNALS  # noqa: E402

PARAMS_FILENAME = "Parameters_Control_Field.json"
GRID_KEY = "Grid_ResourceControlField_Tx"
DATA_KEY = "Sequence_ControlSigs_QAM"
OUTPUT_KEY = "Grid_DataControl_Tx"


def load_payload_grid_module():
    """Tx/Construct_Payload_Grid.py, loaded by path so its fill functions can be reused."""
    path = TX / "Construct_Payload_Grid.py"
    spec = importlib.util.spec_from_file_location("_Construct_Payload_Grid", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def file_path(params: dict, key: str) -> Path:
    name = params.get("File", {}).get(key)
    if not isinstance(name, str) or not name.strip():
        raise ValueError(f"File/{key} not found in {PARAMS_FILENAME}")
    return TX_CONTROL_SIGNALS / name.strip()


def construct_control_grid(params_path: Path) -> np.ndarray:
    import h5py
    pg = load_payload_grid_module()
    params = json.loads(Path(params_path).read_text(encoding="utf-8"))
    d = params["Defined"]
    port = pg.check_port(d["Port"])
    if port != 0:
        raise ValueError(f"Defined/Port is {port}; the control field is sent from port 0 only")
    spacing = pg.check_spacing(d["RefSignalSpacing"])
    bits = pg.mod_to_bits(d["Mod"])
    grid_path, data_path, out_path = (file_path(params, k) for k in (GRID_KEY, DATA_KEY, OUTPUT_KEY))

    types, codes, grid_attrs, extra = pg.load_grid(grid_path)
    for param_key, grid_attr in (("RefSignalSpacing", "ReferenceSymbolProfile"),
                                 ("RefSymbolSpacing", "PacketProfile"),
                                 ("PacketLength", "Packet_Len")):
        have = grid_attrs.get(grid_attr)
        if have is not None and int(have) != int(d[param_key]):
            raise ValueError(f"{PARAMS_FILENAME} has {param_key} {d[param_key]} but {grid_path.name} "
                             f"was built with {grid_attr} {int(have)} - run Initialize_Control_Field.py")

    # Reference signals: must have been generated with this control field's C_init_ID1
    ref_path = REFERENCE_SIGNALS / f"Ref_P{port}_{spacing}_{bits}.h5"
    if not ref_path.is_file():
        raise FileNotFoundError(f"{ref_path} not found - run ReferenceSignals/Generate_Reference_Signal_Vectors.py")
    with h5py.File(ref_path, "r") as f:
        ref_c_init = f.attrs.get("C_init")
    if ref_c_init is not None and int(ref_c_init) != int(d["C_init_ID1"]):
        raise ValueError(f"{ref_path.name} was generated with C_init {int(ref_c_init)} but the control "
                         f"field's C_init_ID1 is {d['C_init_ID1']} - regenerate the reference signals "
                         "or correct C_init_ID1")

    grid = np.zeros(types.shape, dtype=np.complex128)        # DC and Ref_P1 stay 0
    pg.fill_data(grid, types == codes["Data"], pg.load_values(data_path))
    ref_symbols = pg.fill_ref(grid, types == codes["Ref_P0"], pg.load_values(ref_path), "Ref_P0")

    with h5py.File(out_path, "w") as f:
        f.create_dataset("data", data=grid)
        for name, arr in extra.items():
            f.create_dataset(name, data=arr)
        for k, v in grid_attrs.items():
            f.attrs[k] = v
        f.attrs["Port"] = port
        f.attrs["RefSignalSpacing"] = spacing
        f.attrs["Mod"] = str(d["Mod"])
        f.attrs["BitsPerSymbol"] = bits
        f.attrs["C_init_ID1"] = int(d["C_init_ID1"])
        f.attrs["ResourceGridFile"] = grid_path.name
        f.attrs["DataFile"] = data_path.name
        f.attrs["RefFile"] = ref_path.name

    counts = {name: int((types == code).sum()) for name, code in codes.items()}
    print(f"Port {port}, RefSignalSpacing {spacing}, Mod {d['Mod']} ({bits} bit/symbol)")
    print(f"Grid {types.shape}: {counts}")
    print(f"Data   <- {data_path.name}")
    print(f"Ref_P0 <- {ref_path.name} (C_init {d['C_init_ID1']}), on symbols {ref_symbols}")
    print("Ref_P1 and DC set to 0")
    print(f"Wrote {out_path}")
    return grid


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Fill the control field grid.")
    ap.add_argument("--params", type=Path, default=TX_CONTROL_SIGNALS / PARAMS_FILENAME)
    a = ap.parse_args(argv)
    try:
        construct_control_grid(a.params)
    except (FileNotFoundError, ValueError, KeyError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
