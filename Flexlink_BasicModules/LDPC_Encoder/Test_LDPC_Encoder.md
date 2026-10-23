# Test_LDPC_Encoder.py and Test_LDPC_Decoder.py

Two test scripts that are meant to be run in order:

1. `Test_LDPC_Encoder.py` generates random message bits, encodes them with `LDPC_Encode`, and saves the message and the codeword to HDF5 files.
2. `Test_LDPC_Decoder.py` reads the codeword, decodes it with `LDPC_Decode`, saves the decoded data to an HDF5 file, and checks that it equals the original message.

## Prerequisites

- Python 3 with `numpy` and `h5py` installed (activate your virtual environment first):

  ```
  pip install numpy h5py
  ```

- These files in the same folder:
  - Matrix generation: `get_p_ldpc.py`, `create_parity_check_matrix.py`, `ldpc_generator.py`, `GenerateGmatrix.py`, `generate_h_matrix.py`
  - Saving and loading matrices: `generate_and_save_all_G.py`, `generate_and_save_all_H.py`, `LDPC_Get_G.py`, `LDPC_Get_H.py`
  - Encoder and decoder: `LDPC_Encode.py`, `LDPC_Decode.py`
  - Test scripts: `Test_LDPC_Encoder.py`, `Test_LDPC_Decoder.py`

- The `G_<type>.h5` and `H_<type>.h5` files, created once with:

  ```
  python generate_and_save_all_G.py G_matrices
  python generate_and_save_all_H.py H_matrices
  ```

## Typical workflow

```
python Test_LDPC_Encoder.py
python Test_LDPC_Decoder.py
```

The encoder writes `Test_Encoded_Input.h5` and `Test_Encoded_Output.h5` to the current folder. The decoder reads them from the current folder and writes `Test_Decoded_Output.h5`. It prints `PASS` if the decoded data equals the original message.

---

# Part 1: Test_LDPC_Encoder.py

## Running

From the folder containing the scripts:

```
python Test_LDPC_Encoder.py
```

By default this encodes one random message with code type `'648_12'` and writes the files to the current folder.

## Output files

- `Test_Encoded_Input.h5` contains the dataset `DataBlk` (the message bits).
- `Test_Encoded_Output.h5` contains the dataset `CodeBlk` (the codeword bits).
- Both datasets are `uint8` with values 0 and 1.
- Each file also stores the attributes `code_type`, `K`, `N`, `num_messages` and `seed`, so the decoder test knows which code produced the data.

## Shapes

- Both datasets are always 2-D, with one message or codeword per row, even for a single message.
- A default run gives `DataBlk` of shape `(1, 324)` and `CodeBlk` of shape `(1, 648)`.
- Use `DataBlk[0]` if your decoder expects a 1-D vector.

## Options

- **Code type:** the first argument, default `'648_12'`. Valid types are `648_12`, `648_23`, `648_34`, `648_56`, `1296_12`, `1296_23`, `1296_34`, `1296_56`, `1944_12`, `1944_23`, `1944_34` and `1944_56`.

  ```
  python Test_LDPC_Encoder.py 1944_56
  ```

- `--num-messages N`: number of random messages to encode (default 1).

  ```
  python Test_LDPC_Encoder.py 648_12 --num-messages 100
  ```

- `--seed S`: random seed, which makes the message reproducible.

  ```
  python Test_LDPC_Encoder.py 648_12 --seed 1234
  ```

- `--g-dir FOLDER`: folder containing the `G_<type>.h5` files. If omitted, the script searches `./G_matrices`, the script's own `G_matrices` folder, the current folder, and the script's folder, in that order.
- `--out-dir FOLDER`: folder for the output files (default: current folder). It is created if it doesn't exist.

  ```
  python Test_LDPC_Encoder.py 648_12 --g-dir G_matrices --out-dir test_data
  ```

Options can be combined.

## Seed

- If you don't give a seed, a random one is chosen.
- The seed used is always saved in both files, so any run can be recreated.
- Running again with the same code type, number of messages and seed gives identical data.

## Checks performed by the script

- The first K bits of each codeword must equal the message, because `G` is systematic (`G = [I | A^T]`).
- Both files are read back after writing and compared with the original data.
- The script stops with an error if either check fails.

The codeword is the K message bits followed by N − K parity bits. This is why the first bits printed at the end of a run are identical for the data and the codeword: the parity bits come after the message bits, so `CodeBlk` is longer than `DataBlk` and the two are not the same.

## Reading the files

```python
import h5py

with h5py.File("Test_Encoded_Input.h5", "r") as f:
    DataBlk = f["DataBlk"][:]
    code_type = f.attrs["code_type"]

with h5py.File("Test_Encoded_Output.h5", "r") as f:
    CodeBlk = f["CodeBlk"][:]
```

---

# Part 2: Test_LDPC_Decoder.py

## What it does

- Reads `Test_Encoded_Output.h5` (dataset `CodeBlk`) and `Test_Encoded_Input.h5` (dataset `DataBlk`).
- Takes the code type from the `code_type` attribute in `Test_Encoded_Output.h5` and loads the matching `H` with `LDPC_Get_H`. You don't pass the type in.
- Converts the codeword bits to soft beliefs (see below), optionally adds noise, and decodes with `LDPC_Decode`.
- Writes the result to `Test_Decoded_Output.h5` (dataset `DecodedDataBlk`).
- Compares `DecodedDataBlk` with `DataBlk` for equality.

## Running

From the folder containing the test files, after running `Test_LDPC_Encoder.py`:

```
python Test_LDPC_Decoder.py
```

By default there is no noise, so this is a basic check that the pipeline recovers the data. For a real test of the decoder, add noise with `--sigma`:

```
python Test_LDPC_Decoder.py --sigma 0.7 --seed 5
```

## Input convention: soft beliefs

`LDPC_Decode` expects soft beliefs, not hard bits:

- A positive value means bit 1 and a negative value means bit 0. A larger magnitude means higher confidence.
- `Test_Encoded_Output.h5` stores hard bits, so the script maps 1 to +1 and 0 to -1 (BPSK) and then adds optional Gaussian noise.
- Hard 0/1 bits passed directly to `LDPC_Decode` are not valid, because a 0 would mean "no information". A warning is issued if the input contains only 0s and 1s.
- The decoder is scale invariant, so only the sign and relative sizes of the beliefs matter.

## Options

- `--sigma S`: standard deviation of the Gaussian noise added to the ±1 beliefs (default 0, noise-free).
- `--seed S`: noise seed. If omitted, a random seed is chosen and saved in the output file.
- `--iterations N`: number of decoder iterations (default 15).
- `--in-dir FOLDER`: folder containing `Test_Encoded_Output.h5` and `Test_Encoded_Input.h5` (default: current folder).
- `--out-dir FOLDER`: folder for `Test_Decoded_Output.h5` (default: same as `--in-dir`).
- `--h-dir FOLDER`: folder containing the `H_<type>.h5` files. If omitted, the script searches `./H_matrices`, the script's own `H_matrices` folder, the current folder, and the script's folder, in that order.

Example with all folders given:

```
python Test_LDPC_Decoder.py --in-dir test_data --out-dir test_data --h-dir H_matrices --sigma 0.7 --seed 5
```

## Sigma and SNR

The script takes `--sigma` directly, not an SNR. With the ±1 mapping, each coded bit has energy Es = 1 and the noise power per dimension is sigma², so:

```
Es/N0 = 1 / (2 * sigma^2)
Eb/N0 = (Es/N0) / R          where R = K/N is the code rate
```

The same sigma is a harder problem for a high-rate code than for a low-rate code, so compare Eb/N0 rather than sigma across code types.

## Output file

`Test_Decoded_Output.h5` contains:

- The dataset `DecodedDataBlk`: `uint8`, shape `(M, K)`, values 0 and 1, one decoded message per row.
- The attributes `code_type`, `K`, `N`, `num_messages`, `sigma`, `noise_seed`, `iterations`, `bit_errors` and `matches_input`.

The script reads the file back after writing to confirm it was saved correctly.

## Result and exit status

- If `DecodedDataBlk` equals `DataBlk` exactly, the script prints `PASS` and exits with status 0.
- Otherwise it prints `FAIL` with the number of bit errors and the number of messages affected, still writes the output file, and exits with status 1.
- When `--sigma` is greater than 0, it also prints the number of hard-decision bit errors before decoding, so you can see how many errors the decoder corrected.
- A missing input file or a shape mismatch between the files prints an error and exits with status 1.

At high noise levels a `FAIL` is a legitimate result, not a bug: the noise has exceeded what the code can correct.

## Reading the decoded file

```python
import h5py

with h5py.File("Test_Decoded_Output.h5", "r") as f:
    DecodedDataBlk = f["DecodedDataBlk"][:]
    matches = f.attrs["matches_input"]
```

## Using the decoder directly

```python
from LDPC_Get_H import LDPC_Get_H
from LDPC_Decode import LDPC_Decode

H = LDPC_Get_H('648_12')                  # load once and reuse
DecodedDataBlk = LDPC_Decode(Beliefs, H)  # Beliefs: (N,) or (M, N), positive = bit 1
```

`LDPC_Decode` returns `uint8` with shape `(K,)` for a 1-D input or `(M, K)` for a 2-D input, holding the first K decoded bits.
