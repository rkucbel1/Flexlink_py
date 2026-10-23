"""Generate_Test_Inputs.py

Creates the test input files:
    LDPC: Test_Input_648.h5, Test_Input_1296.h5, Test_Input_1944.h5
    BCC : Test_Input_BCC_40.h5, Test_Input_BCC_140.h5
Each file holds one dataset "TestInput" (uint8, shape (NUM_BLOCKS, length)) of
random 0/1 bits, plus the attributes CBS (the block length), NumBlocks and seed.
"""
import sys
from pathlib import Path
import numpy as np
import h5py

LDPC_CBS_VALUES = (648, 1296, 1944)
BCC_LENGTHS = (40, 140)
NUM_BLOCKS = 1    # one code block (one row) per file
SEED = 12345

out_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(".")
out_dir.mkdir(parents=True, exist_ok=True)
rng = np.random.default_rng(SEED)

jobs = [(n, f"Test_Input_{n}.h5") for n in LDPC_CBS_VALUES] + \
       [(n, f"Test_Input_BCC_{n}.h5") for n in BCC_LENGTHS]

for length, name in jobs:
    data = rng.integers(0, 2, size=(NUM_BLOCKS, length), dtype=np.uint8)
    path = out_dir / name
    with h5py.File(path, "w") as f:
        f.create_dataset("TestInput", data=data)
        f.attrs["CBS"] = length
        f.attrs["NumBlocks"] = NUM_BLOCKS
        f.attrs["seed"] = SEED
    with h5py.File(path, "r") as f:
        back = f["TestInput"][:]
    assert np.array_equal(back, data)
    print(f"{path}: shape {back.shape}, dtype {back.dtype}, ones fraction {back.mean():.3f}")
