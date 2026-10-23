#!/usr/bin/env python3
"""
Create_ControlField_Waveform.py  (lives in Flexlink_BasicModules/Construct_Control_Field)

Turn the control field grid into its time-domain OFDM waveform: 4 OFDM symbols
(Ref, Data, Data, Ref) that are inserted between the preamble and the payload.

The control field only has to be rebuilt when the Tx parameters change, so its
waveform is made once here and saved; building the packet just reads it.

Steps:
  1. Read Tx/ControlSignals/Parameters_Control_Field.json.
  2. Load the control grid from File/Grid_DataControl_Tx (written by
     Construct_Control_Grid.py): dataset 'data', complex, (PacketLength, 841).
  3. Run every OFDM symbol through ofdm_modulator_841() from
     OFDM_Modulator/OFDM_Modulator_841.py with Defined/SampleRate
     (20 MHz -> 1024-point IFFT, CP 116; 40 MHz -> 2048-point IFFT, CP 232),
     exactly as Tx/Construct_Payload_Waveform.py does for the payload.
  4. Save to File/Waveform_Control_Tx in Tx/ControlSignals.

No scaling is applied: the samples have the power the grid gives them, and that
power is stored (ControlPower) so the step that builds the packet can scale the
control field to match the payload, as it does for the preamble.

Usage:
    python Flexlink_BasicModules/Construct_Control_Field/Create_ControlField_Waveform.py

    # or from Python
    from Create_ControlField_Waveform import create_controlfield_waveform
    waveform, attrs = create_controlfield_waveform()          # also saves the file
    waveform, attrs = create_controlfield_waveform(save=False)

The output .h5 file contains:
    data   (PacketLength * OfdmSymbolLength,) complex128   (4 x 2280 = 9120 at 40 MHz)
    attrs  SampleRate, IfftSize, CpLength, OfdmSymbolLength, NumOfdmSymbols,
           ControlLength (samples), ControlPower (mean |x|^2), Port, Mod,
           C_init_Dat, SourceFile
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

# Locate flexlink_paths.py by walking up from this script's folder.
for _parent in Path(__file__).resolve().parents:
    if (_parent / "flexlink_paths.py").is_file():
        sys.path.insert(0, str(_parent))
        break
from flexlink_paths import TX_CONTROL_SIGNALS, OFDM_MODULATOR  # noqa: E402

if str(OFDM_MODULATOR) not in sys.path:
    sys.path.insert(0, str(OFDM_MODULATOR))

PARAMS_FILENAME = "Parameters_Control_Field.json"
GRID_KEY = "Grid_DataControl_Tx"
OUTPUT_KEY = "Waveform_Control_Tx"
NUM_SUBCARRIERS = 841
OFDM_SIZES = {20e6: (1024, 116), 40e6: (2048, 232)}     # SampleRate -> (IFFT size, CP length)


def _file_path(params: dict[str, Any], key: str) -> Path:
    name = params.get("File", {}).get(key)
    if not isinstance(name, str) or not name.strip():
        raise ValueError(f"File/{key} not found in {PARAMS_FILENAME}")
    return TX_CONTROL_SIGNALS / name.strip()


def create_controlfield_waveform(params_path: Path | None = None,
                                 save: bool = True) -> tuple[np.ndarray, dict[str, Any]]:
    """Return (control field waveform, attributes); save it to File/Waveform_Control_Tx if save."""
    import h5py
    from OFDM_Modulator_841 import ofdm_modulator_841

    params_path = Path(params_path or TX_CONTROL_SIGNALS / PARAMS_FILENAME)
    params = json.loads(params_path.read_text(encoding="utf-8"))
    d = params["Defined"]
    fs = float(d["SampleRate"])
    if fs not in OFDM_SIZES:
        raise ValueError(f"SampleRate {d['SampleRate']} must be 20000000 or 40000000")
    ifft_size, cp_len = OFDM_SIZES[fs]
    sym_len = ifft_size + cp_len

    grid_path = _file_path(params, GRID_KEY)
    if not grid_path.is_file():
        raise FileNotFoundError(f"{grid_path} not found - run Construct_Control_Grid.py first")
    with h5py.File(grid_path, "r") as f:
        grid = np.asarray(f["data"][()]).astype(np.complex128)
        grid_attrs = dict(f.attrs)
    if grid.shape != (d["PacketLength"], NUM_SUBCARRIERS):
        raise ValueError(f"{grid_path.name} is {grid.shape}, expected ({d['PacketLength']}, "
                         f"{NUM_SUBCARRIERS}) - rerun the control field steps")

    waveform = np.concatenate([ofdm_modulator_841(grid[n, :], fs) for n in range(grid.shape[0])])
    if waveform.size != grid.shape[0] * sym_len:
        raise ValueError(f"Control waveform has {waveform.size} samples, expected "
                         f"{grid.shape[0]} x {sym_len}")

    attrs = {"SampleRate": fs, "IfftSize": ifft_size, "CpLength": cp_len,
             "OfdmSymbolLength": sym_len, "NumOfdmSymbols": grid.shape[0],
             "ControlLength": waveform.size,
             "ControlPower": float(np.mean(np.abs(waveform) ** 2)),
             "Port": int(grid_attrs.get("Port", d["Port"])),
             "Mod": str(grid_attrs.get("Mod", d["Mod"])),
             "C_init_Dat": int(d["C_init_Dat"]), "SourceFile": grid_path.name}

    if save:
        out_path = _file_path(params, OUTPUT_KEY)
        with h5py.File(out_path, "w") as f:
            f.create_dataset("data", data=waveform)
            for k, v in attrs.items():
                f.attrs[k] = v
        attrs["OutputFile"] = str(out_path)
    return waveform, attrs


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Build the control field OFDM waveform.")
    ap.add_argument("--params", type=Path, default=TX_CONTROL_SIGNALS / PARAMS_FILENAME)
    a = ap.parse_args(argv)
    try:
        waveform, at = create_controlfield_waveform(a.params)
    except ImportError as exc:
        print(f"Error: {exc} - install h5py with: pip install h5py", file=sys.stderr)
        return 1
    except (FileNotFoundError, KeyError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    print(f"SampleRate {at['SampleRate'] / 1e6:.0f} MHz -> IFFT {at['IfftSize']}, CP {at['CpLength']}, "
          f"{at['OfdmSymbolLength']} samples per OFDM symbol")
    print(f"Control field: {at['NumOfdmSymbols']} symbols x {at['OfdmSymbolLength']} = "
          f"{at['ControlLength']} samples ({at['ControlLength'] / at['SampleRate'] * 1e6:.1f} us), "
          f"RMS {np.sqrt(at['ControlPower']):.4f}, peak |x| {np.max(np.abs(waveform)):.4f}")
    print(f"Wrote {at['OutputFile']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
