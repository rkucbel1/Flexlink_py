"""Test_LDPC_Encoder.py

Generates random message bits, encodes them with LDPC_Encode, and saves both
for later use in testing the decoder:

    Test_Encoded_Input.h5    dataset "DataBlk"  uint8, shape (M, K)   message bits
    Test_Encoded_Output.h5   dataset "CodeBlk"  uint8, shape (M, N)   codeword bits

Both are always 2-D with one message / codeword per row, even when M = 1.
Each file also stores attributes: code_type, K, N, num_messages, seed.

Usage
-----
    python Test_LDPC_Encoder.py                       # '648_12', 1 message
    python Test_LDPC_Encoder.py 1944_56               # another code type
    python Test_LDPC_Encoder.py 648_12 --num-messages 100
    python Test_LDPC_Encoder.py 648_12 --seed 1234    # reproducible message
    python Test_LDPC_Encoder.py 648_12 --g-dir G_matrices --out-dir test_data

Reading the files back:

    import h5py
    with h5py.File("Test_Encoded_Input.h5", "r") as f:
        DataBlk = f["DataBlk"][:]
        code_type = f.attrs["code_type"]
    with h5py.File("Test_Encoded_Output.h5", "r") as f:
        CodeBlk = f["CodeBlk"][:]

Requires: numpy, h5py, and the G_<type>.h5 files (generate_and_save_all_G.py).
"""

import argparse
from pathlib import Path

import numpy as np
import h5py

from LDPC_Get_G import LDPC_Get_G, VALID_TYPES
from LDPC_Encode import LDPC_Encode


def write_dataset_file(path, name, data, code_type, K, N, num_messages, seed):
    """Write one array to an HDF5 file along with descriptive attributes."""
    with h5py.File(path, "w") as f:
        f.create_dataset(name, data=data)
        f.attrs["code_type"] = code_type
        f.attrs["K"] = K
        f.attrs["N"] = N
        f.attrs["num_messages"] = num_messages
        f.attrs["seed"] = seed


def main():
    parser = argparse.ArgumentParser(description="Encode random messages and save input/output to HDF5.")
    parser.add_argument("code_type", nargs="?", default="648_12", choices=VALID_TYPES,
                        help="LDPC type (default: 648_12)")
    parser.add_argument("--num-messages", type=int, default=1,
                        help="number of random messages to encode (default: 1)")
    parser.add_argument("--seed", type=int, default=None,
                        help="random seed (default: random; the seed used is saved in the files)")
    parser.add_argument("--g-dir", default=None,
                        help="folder containing the G_<type>.h5 files (default: auto-search)")
    parser.add_argument("--out-dir", default=".",
                        help="folder for the output files (default: current folder)")
    args = parser.parse_args()

    if args.num_messages < 1:
        parser.error("--num-messages must be at least 1")

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Load G once, then generate and encode the messages
    G = LDPC_Get_G(args.code_type, directory=args.g_dir, dtype=np.float32)
    K, N = G.shape

    seed = args.seed if args.seed is not None else int(np.random.SeedSequence().entropy % 2**32)
    rng = np.random.default_rng(seed)
    DataBlk = rng.integers(0, 2, size=(args.num_messages, K), dtype=np.uint8)

    CodeBlk = LDPC_Encode(DataBlk, G)

    # Sanity check: G is systematic, so the first K bits are the message
    assert np.array_equal(CodeBlk[:, :K], DataBlk), "systematic part does not match message"

    in_path = out_dir / "Test_Encoded_Input.h5"
    out_path = out_dir / "Test_Encoded_Output.h5"
    write_dataset_file(in_path, "DataBlk", DataBlk, args.code_type, K, N, args.num_messages, seed)
    write_dataset_file(out_path, "CodeBlk", CodeBlk, args.code_type, K, N, args.num_messages, seed)

    # Read back and confirm what was written
    with h5py.File(in_path, "r") as f:
        assert np.array_equal(f["DataBlk"][:], DataBlk), "DataBlk read-back mismatch"
    with h5py.File(out_path, "r") as f:
        assert np.array_equal(f["CodeBlk"][:], CodeBlk), "CodeBlk read-back mismatch"

    print(f"code type: {args.code_type}   K = {K}, N = {N}, messages = {args.num_messages}, seed = {seed}")
    print(f"wrote {in_path}   DataBlk {DataBlk.shape} {DataBlk.dtype}")
    print(f"wrote {out_path}  CodeBlk {CodeBlk.shape} {CodeBlk.dtype}")
    print("first message, first 20 data bits:    ", DataBlk[0, :20])
    print("first message, first 20 codeword bits:", CodeBlk[0, :20])


if __name__ == "__main__":
    main()
