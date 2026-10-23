"""
Scrambler_Gold_Code_Generator.py

Python port of the MATLAB function GoldCodeGenerator_Flexlink, using a sliding
31-bit register (constant memory, independent of the number of output bits).
Gold code (scrambling sequence) based on the LTE standard (3GPP TS 36.211, 7.2).

    x1(n+31) = (x1(n+3) + x1(n)) mod 2
    x2(n+31) = (x2(n+3) + x2(n+2) + x2(n+1) + x2(n)) mod 2
    c(n)     = (x1(n+Nc) + x2(n+Nc)) mod 2,   Nc = 1600

Every call writes the generated Sequence and its length to an HDF5 file,
creating it or overwriting any existing file (requires h5py).

Register convention: each register is a 31-bit integer where bit k holds
x(n+k), so bit 0 is the oldest value x(n) (MATLAB's Reg(EndLocation+31)) and
bit 30 is the newest.
"""

import h5py
import numpy as np

SHIFT = 1600  # Nc
H5_FILE = "GC_Scrambler_Sequence.h5"


def write_sequence_h5(sequence, c_init, filename):
    """Write Sequence and its length to an HDF5 file.

    /Sequence   dataset, uint8 array of 0/1
    /length     dataset, number of bits in Sequence
    attribute c_init on the file root, for reference
    """
    with h5py.File(filename, "w") as f:  # "w" creates or overwrites
        f.create_dataset("Sequence", data=sequence)
        f.create_dataset("length", data=len(sequence))
        f.attrs["c_init"] = int(c_init)


def gold_code_generator(c_init=130, number_of_output_bits=1, h5_file=H5_FILE):
    """
    Parameters
    ----------
    c_init : int
        Initial value for the second m-sequence (31-bit).
    number_of_output_bits : int
        Number of scrambling bits to produce.
    h5_file : str
        HDF5 file that Sequence and its length are written to on every call
        (created or overwritten). Default: GC_Scrambler_Sequence.h5

    Returns
    -------
    Sequence : np.ndarray, shape (number_of_output_bits,), dtype=np.uint8
    """
    n_out = int(number_of_output_bits)

    # x1 initial condition: x1(0) = 1, x1(1..30) = 0
    reg1 = 1
    # x2 initial condition: x2(k) = bit k of c_init (31 bits)
    reg2 = int(c_init) & 0x7FFFFFFF

    def step(r1, r2):
        fb1 = (r1 ^ (r1 >> 3)) & 1
        fb2 = (r2 ^ (r2 >> 1) ^ (r2 >> 2) ^ (r2 >> 3)) & 1
        return (r1 >> 1) | (fb1 << 30), (r2 >> 1) | (fb2 << 30)

    # Shift the generator by Nc
    for _ in range(SHIFT):
        reg1, reg2 = step(reg1, reg2)

    # Produce the scrambling bits
    sequence = np.zeros(n_out, dtype=np.uint8)
    for i in range(n_out):
        sequence[i] = (reg1 ^ reg2) & 1
        reg1, reg2 = step(reg1, reg2)

    write_sequence_h5(sequence, c_init, h5_file)

    return sequence


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Generate an LTE Gold code sequence and write it to an HDF5 file."
    )
    parser.add_argument("-c", "--c_init", type=int, default=130,
                        help="initial value c_init (default: 130)")
    parser.add_argument("-n", "--num_bits", type=int, default=64,
                        help="NumberOfOutputBits (default: 64)")
    parser.add_argument("-o", "--output", default=H5_FILE,
                        help="HDF5 output file (default: %s)" % H5_FILE)
    args = parser.parse_args()

    gold_code_generator(args.c_init, args.num_bits, h5_file=args.output)
