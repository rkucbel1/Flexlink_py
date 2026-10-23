# Scrambler_Gold_Code_Generator.py

Python port of the MATLAB function `GoldCodeGenerator_Flexlink`. It generates an LTE Gold code (scrambling sequence, 3GPP TS 36.211, section 7.2) and writes the sequence and its length to an HDF5 file on every call.

## Requirements

- Python 3
- `numpy`
- `h5py`

```
pip install numpy h5py
```

## Running from the terminal

```
python Scrambler_Gold_Code_Generator.py [-c C_INIT] [-n NUM_BITS] [-o OUTPUT]
```

| Option | Long form | Meaning | Default |
|--------|-----------|---------|---------|
| `-c` | `--c_init` | Initial value `c_init` | `130` |
| `-n` | `--num_bits` | `NumberOfOutputBits` | `64` |
| `-o` | `--output` | HDF5 output file | `GC_Scrambler_Sequence_Tx.h5` |

Examples:

```
python Scrambler_Gold_Code_Generator.py                        # c_init=130, 64 bits
python Scrambler_Gold_Code_Generator.py -c 1234 -n 1024        # custom c_init and bit count
python Scrambler_Gold_Code_Generator.py --c_init 1234 --num_bits 1024
python Scrambler_Gold_Code_Generator.py -n 2048 -o my_file.h5  # custom output file
python Scrambler_Gold_Code_Generator.py -h                     # show help
```

Nothing is printed to the terminal; the result is only written to the `.h5` file. Argument errors and a missing `h5py` will still show an error message.

## Using it from Python

```python
from Scrambler_Gold_Code_Generator import gold_code_generator

Sequence = gold_code_generator(c_init=130, number_of_output_bits=1024)
```

Signature:

```python
gold_code_generator(c_init=130, number_of_output_bits=1, h5_file="GC_Scrambler_Sequence_Tx.h5")
```

- Returns `Sequence`, a NumPy `uint8` array of 0s and 1s with `number_of_output_bits` elements.
- **Every call writes to `h5_file`**, creating it or overwriting any existing file.
- `c_init` is masked to 31 bits.

## HDF5 file contents

| Item | Type | Description |
|------|------|-------------|
| `/Sequence` | dataset | The Gold code bits (`uint8`, values 0/1) |
| `/length` | dataset | Number of bits in `Sequence` |
| `c_init` | attribute on the file root | The `c_init` used to generate the sequence |

Reading it in Python:

```python
import h5py

with h5py.File("GC_Scrambler_Sequence_Tx.h5", "r") as f:
    sequence = f["Sequence"][:]
    length = int(f["length"][()])
    c_init = int(f.attrs["c_init"])
```

Reading it in MATLAB:

```matlab
Sequence = h5read('GC_Scrambler_Sequence_Tx.h5', '/Sequence');
Length   = h5read('GC_Scrambler_Sequence_Tx.h5', '/length');
c_init   = h5readatt('GC_Scrambler_Sequence_Tx.h5', '/', 'c_init');
```

## How it works

Two 31-bit LFSRs are used:

- x1(n+31) = (x1(n+3) + x1(n)) mod 2, starting with x1(0) = 1 and the rest 0
- x2(n+31) = (x2(n+3) + x2(n+2) + x2(n+1) + x2(n)) mod 2, starting with x2(k) = bit k of `c_init`
- Output c(n) = (x1(n+1600) + x2(n+1600)) mod 2

The first 1600 steps (Nc) are run and discarded. Each register is held as a single 31-bit integer (a sliding register), so memory use does not depend on `number_of_output_bits`.

## Test value

For `c_init = 130` and 64 bits, the sequence is:

```
1000110011000001100001000000110110011000001010011010001010100001
```

## Related script

`Scrambler_GC.py` imports `gold_code_generator` from this file (keep both in the same folder) and XORs the Gold code with an input bit sequence. Because it calls the generator, each run also rewrites `GC_Scrambler_Sequence_Tx.h5`.
