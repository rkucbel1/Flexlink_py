"""
Rx_Antennas.py  (lives in Flexlink_py/Rx)

Shared helpers for the multi-antenna Rx chain: how many antennas this run has, and
which files steps 7 to 9 read and write.

    Steps 4-6 run once per antenna and write *_Ray1 .. *_RayN files.
    Combine_Antennas_Rx.py combines the equalized grids into *_Combined.
    Steps 7-9 then work on the *_Combined files when the run has more than one antenna,
    on the *_Ray1 files when it has one, or on *_RayN when --ray N is given.

The number of antennas of a run is the NumberOfAntennas attribute that
Decimate_Payload_Rx.py writes into Waveform_Payload_Dec_Rx.h5 (1 when no defects are
modeled, else Defined_Rx/NumberOfAntennas), so steps 7-9 follow what was actually decimated.
"""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

for _parent in Path(__file__).resolve().parents:
    if (_parent / "flexlink_paths.py").is_file():
        sys.path.insert(0, str(_parent))
        break
from flexlink_paths import ray_file_name  # noqa: E402

RX_DIR = Path(__file__).resolve().parent
COMBINED_TAG = "Combined"
DEC_KEY = "Waveform_Payload_Dec_Rx"


def combined_file_name(name: str) -> str:
    """('Grid_Equalized_Rx.h5') -> 'Grid_Equalized_Rx_Combined.h5'."""
    p = Path(name)
    return f"{p.stem}_{COMBINED_TAG}{p.suffix}"


def antennas_in_run(params: dict[str, Any], rx_dir: Path = RX_DIR) -> int:
    """NumberOfAntennas recorded in the decimated waveform file (1 if absent)."""
    name = params.get("File_Rx", {}).get(DEC_KEY)
    if not name:
        return 1
    path = Path(rx_dir) / name
    if not path.is_file():
        return 1
    import h5py
    with h5py.File(path, "r") as f:
        n = f.attrs.get("NumberOfAntennas")
        if n is None and "data" in f:
            shape = f["data"].shape
            n = shape[0] if len(shape) == 2 else 1
    return int(n or 1)


def target_name(name: str, ray: int | None, n_antennas: int) -> str:
    """File name for steps 7-9: the ray's file with --ray N, else _Combined for a
    multi-antenna run, else _Ray1."""
    if ray is not None:
        return ray_file_name(name, ray)
    return combined_file_name(name) if n_antennas > 1 else ray_file_name(name, 1)


def target_label(ray: int | None, n_antennas: int) -> str:
    if ray is not None:
        return f"ray {ray}"
    return f"{n_antennas} antennas combined" if n_antennas > 1 else "ray 1"
