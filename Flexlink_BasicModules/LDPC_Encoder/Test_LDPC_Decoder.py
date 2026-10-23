"""Test_LDPC_Decoder.py

Decodes the codewords written by Test_LDPC_Encoder.py and checks that the
decoded data equals the original message.

    reads   Test_Encoded_Output.h5   dataset "CodeBlk"          (M, N) bits 0/1
    reads   Test_Encoded_Input.h5    dataset "DataBlk"          (M, K) bits 0/1
    writes  Test_Decoded_Output.h5   dataset "DecodedDataBlk"   (M, K) uint8

The LDPC type is taken from the file attributes (code_type), and H is loaded
with LDPC_Get_H. The decoder needs soft beliefs (positive = bit 1, negative =
bit 0), so the stored bits are mapped to +1 / -1 (BPSK), optionally with added
Gaussian noise (--sigma) to give the decoder real work to do.

The script exits with status 0 if DecodedDataBlk equals DataBlk, 1 otherwise.

Usage
-----
    python Test_LDPC_Decoder.py                          # noise-free
    python Test_LDPC_Decoder.py --sigma 0.7 --seed 5     # noisy channel
    python Test_LDPC_Decoder.py --in-dir test_data --out-dir test_data --h-dir H_matrices

Requires: numpy, h5py, and the H_<type>.h5 files (generate_and_save_all_H.py).
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import h5py

from LDPC_Get_H import LDPC_Get_H
from LDPC_Decode import LDPC_Decode


def _attr_str(value):
    """h5py may return str, bytes or numpy string types for string attributes."""
    if isinstance(value, bytes):
        return value.decode()
    return str(value)


def main():
    parser = argparse.ArgumentParser(description="Decode Test_Encoded_Output.h5 and compare with Test_Encoded_Input.h5.")
    parser.add_argument("--in-dir", default=".",
                        help="folder with Test_Encoded_Output.h5 and Test_Encoded_Input.h5 (default: current folder)")
    parser.add_argument("--out-dir", default=None,
                        help="folder for Test_Decoded_Output.h5 (default: same as --in-dir)")
    parser.add_argument("--h-dir", default=None,
                        help="folder containing the H_<type>.h5 files (default: auto-search)")
    parser.add_argument("--sigma", type=float, default=0.0,
                        help="std. dev. of Gaussian noise added to the +/-1 beliefs (default: 0 = noise-free)")
    parser.add_argument("--seed", type=int, default=None,
                        help="noise seed (default: random; the seed used is saved in the output file)")
    parser.add_argument("--iterations", type=int, default=15,
                        help="decoder iterations (default: 15)")
    args = parser.parse_args()

    if args.sigma < 0:
        parser.error("--sigma must be >= 0")

    in_dir = Path(args.in_dir)
    out_dir = Path(args.out_dir) if args.out_dir is not None else in_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    enc_out_path = in_dir / "Test_Encoded_Output.h5"
    enc_in_path = in_dir / "Test_Encoded_Input.h5"
    dec_out_path = out_dir / "Test_Decoded_Output.h5"

    # ---- read the encoder's files ------------------------------------------
    for p in (enc_out_path, enc_in_path):
        if not p.is_file():
            sys.exit(f"Missing {p}. Run Test_LDPC_Encoder.py first.")

    with h5py.File(enc_out_path, "r") as f:
        CodeBlk = f["CodeBlk"][:]
        code_type = _attr_str(f.attrs["code_type"])
    with h5py.File(enc_in_path, "r") as f:
        DataBlk = f["DataBlk"][:]

    H = LDPC_Get_H(code_type, directory=args.h_dir)
    L, N = H.shape
    K = N - L
    M = CodeBlk.shape[0]
    if CodeBlk.shape != (M, N) or DataBlk.shape != (M, K):
        sys.exit(f"Shape mismatch for code {code_type}: CodeBlk {CodeBlk.shape} "
                 f"(expected ({M}, {N})), DataBlk {DataBlk.shape} (expected ({M}, {K})).")

    # ---- channel: bits -> soft beliefs (1 -> +1, 0 -> -1) plus optional noise
    seed = args.seed if args.seed is not None else int(np.random.SeedSequence().entropy % 2**32)
    rng = np.random.default_rng(seed)
    Beliefs = 2.0 * CodeBlk.astype(np.float64) - 1.0
    if args.sigma > 0:
        Beliefs += args.sigma * rng.standard_normal(Beliefs.shape)

    # ---- decode and compare ------------------------------------------------
    DecodedDataBlk = LDPC_Decode(Beliefs, H, NumberOfIterations=args.iterations)

    mismatch = DecodedDataBlk != DataBlk
    bit_errors = int(mismatch.sum())
    bad_messages = int(mismatch.any(axis=1).sum())
    equal = (bit_errors == 0)
    raw_errors = int(((Beliefs[:, :K] > 0) != DataBlk.astype(bool)).sum())

    # ---- write and verify the decoded file ----------------------------------
    with h5py.File(dec_out_path, "w") as f:
        f.create_dataset("DecodedDataBlk", data=DecodedDataBlk)
        f.attrs["code_type"] = code_type
        f.attrs["K"] = K
        f.attrs["N"] = N
        f.attrs["num_messages"] = M
        f.attrs["sigma"] = args.sigma
        f.attrs["noise_seed"] = seed
        f.attrs["iterations"] = args.iterations
        f.attrs["bit_errors"] = bit_errors
        f.attrs["matches_input"] = int(equal)
    with h5py.File(dec_out_path, "r") as f:
        if not np.array_equal(f["DecodedDataBlk"][:], DecodedDataBlk):
            sys.exit(f"Read-back mismatch for {dec_out_path}")

    print(f"code type: {code_type}   K = {K}, N = {N}, messages = {M}")
    print(f"channel: sigma = {args.sigma}" + (f", noise seed = {seed}" if args.sigma > 0 else " (noise-free)"))
    if args.sigma > 0:
        print(f"hard-decision bit errors before decoding: {raw_errors} of {M * K}")
    print(f"wrote {dec_out_path}  DecodedDataBlk {DecodedDataBlk.shape} {DecodedDataBlk.dtype}")
    if equal:
        print(f"PASS: DecodedDataBlk equals Test_Encoded_Input DataBlk ({M * K} bits, {M} message(s))")
        return 0
    print(f"FAIL: {bit_errors} bit error(s) in {bad_messages} of {M} message(s) "
          f"(first mismatch at message {int(np.argmax(mismatch.any(axis=1)))})")
    return 1


if __name__ == "__main__":
    sys.exit(main())
