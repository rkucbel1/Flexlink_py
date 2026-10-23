# Resource Grid

The Resource Grid is the time/frequency structure shared by the processes that build, transmit and analyze a packet. It is constructed by `Construct_Resource_Grid.py` and is based on the layouts in `ResourceGrid_Flexlink.ods`.

## 1. Structure

A Resource Grid is a matrix with one row per **Symbol** and one column per **Signal**:

```
Resource_Grid
 └── Packet_Len Symbols            (rows)
      └── 841 Signals per Symbol   (columns, k = 0 .. 840)
```

| Item | Definition |
|---|---|
| `k` | Signal index vector, 0 through 840 (841 elements) |
| `Tone_i` | Tone vector, -420 through 420 (841 elements) |
| Mapping | `Tone_i[k] = k - 420`, so k = 420 is tone 0 |
| Subcarrier spacing | 19531.25 Hz (from the spreadsheet), so tone 420 = 8,203,125 Hz |
| `Packet_Len` | 4, 37, 49, 73, 97 or 121 Symbols |

Each grid element holds a complex signal value (`data[symbol, k]`).

## 2. Inputs that define a grid

```python
Resource_Grid(Packet_Len, PacketProfile, ReferenceSymbolProfile)
```

| Input | Allowed values | Meaning |
|---|---|---|
| `Packet_Len` | 4, 37, 49, 73, 97, 121 | Number of Symbols (4 only with PacketProfile 1 or 3) |
| `PacketProfile` | 1, 3, 6, 12 | Which Symbols are `Symbol_Ref` and which are `Symbol_Data` |
| `ReferenceSymbolProfile` | 3, 6, 12, 24 | Signal layout used by every `Symbol_Ref` |

## 3. Symbol types

There are two Symbol types: `Symbol_Ref` and `Symbol_Data`.

The **PacketProfile** sets the pattern. Each repetition is one `Symbol_Ref` followed by (PacketProfile - 1) `Symbol_Data`, and the pattern repeats until `Packet_Len` is reached. Every allowed `Packet_Len` is a multiple of 12 plus one, so the pattern repeats a whole number of times and the extra last Symbol starts one more repetition: **every grid ends with a `Symbol_Ref`**.

Packet_Len 4 (the control field) is 3 plus one, so it ends with a `Symbol_Ref` only for PacketProfile 1 or 3; PacketProfile 6 and 12 are rejected. With PacketProfile 3 and ReferenceSymbolProfile 3 the grid is Ref, Data, Data, Ref: 560 Ref_P0, 560 Ref_P1, 44 DC and 2,200 Data signals.

| PacketProfile | Pattern (repeats) | Symbol_Ref count |
|---|---|---|
| 1 | Ref, Ref, Ref, ... | Packet_Len |
| 3 | Ref, Data, Data | (Packet_Len - 1) / 3 + 1 |
| 6 | Ref, Data x5 | (Packet_Len - 1) / 6 + 1 |
| 12 | Ref, Data x11 | (Packet_Len - 1) / 12 + 1 |

## 4. Signal types

Every Signal in a Symbol is one of four types. The grid stores them as numeric codes in the `signal_types` matrix, which has the same shape as the data.

| Signal type | Code |
|---|---|
| `Ref_P0` | 10 |
| `Ref_P1` | 20 |
| `DC` | 30 |
| `Data` | 40 |

DC signals always carry the value 0.

## 5. Symbol layouts

### Symbol_Data

All signals are `Data` except k = 414 through 428, which are `DC` (15 DC, 826 Data; tones -6 through +8).

### Symbol_Ref

The layout depends on `ReferenceSymbolProfile` (P = 3, 6, 12 or 24):

- `Ref_P1` is at k = start, start + P, start + 2P, ... up to k = 838.
- `Ref_P0` is at the signal directly above each `Ref_P1` (k + 1).
- `DC` is at the ranges listed below (inclusive).
- All other signals are `Data`.

| Reference profile | First Ref_P1 | DC ranges (k) |
|---|---|---|
| 3 | 1 | 0; 414, 417, 420, 423, 426 (every third); 840 |
| 6 | 4 | 0-3; 414-417; 420-423; 426-429; 840 |
| 12 | 10 | 0-9; 408-417; 420-429; 840 |
| 24 | 22 | 0-21; 408-429; 840 |

Signals per Symbol_Ref (each row sums to 841):

| Reference profile | Ref_P0 | Ref_P1 | DC | Data |
|---|---|---|---|---|
| 3 | 280 | 280 | 7 | 274 |
| 6 | 140 | 140 | 17 | 544 |
| 12 | 70 | 70 | 31 | 670 |
| 24 | 35 | 35 | 45 | 726 |

## 6. Worked example

Packet_Len = 73, PacketProfile = 3, ReferenceSymbolProfile = 3:

- 25 Symbol_Ref (Symbols 0, 3, ..., 72) and 48 Symbol_Data
- Data signals: 25 x 274 + 48 x 826 = 6,850 + 39,648 = **46,498**

## 7. Usage

```python
from Construct_Resource_Grid import Resource_Grid, SignalType
from RG_Get_Number_Of_SigTypes import RG_Get_Number_Of_SigTypes

grid = Resource_Grid(73, 3, 3)

grid.shape                          # (73, 841)
grid.signal_types                   # (73, 841) matrix of codes 10/20/30/40
grid.ref_indices                    # rows that are Symbol_Ref
grid.data_indices                   # rows that are Symbol_Data
grid.count_of(SignalType.DATA)      # 46498
RG_Get_Number_Of_SigTypes('DC')     # 30

grid.set_symbol(1, values)          # 841 values; DC forced to 0
grid.signal_at_tone(1, -320)        # access by tone instead of k
grid.save_h5()                      # writes Resource_Grid.h5
```

## 8. Saved file: Resource_Grid.h5

| Dataset | Shape | Description |
|---|---|---|
| `data` | (Packet_Len, 841) | Complex signal values |
| `signal_types` | (Packet_Len, 841) | Codes 10 / 20 / 30 / 40 |
| `symbol_types` | (Packet_Len,) | `Symbol_Ref` or `Symbol_Data` per Symbol |
| `k` | (841,) | 0 .. 840 |
| `Tone_i` | (841,) | -420 .. 420 |

Root attributes: `Packet_Len`, `PacketProfile`, `ReferenceSymbolProfile`, and the four signal-type codes.

## 9. Files

| File | Purpose |
|---|---|
| `Construct_Resource_Grid.py` | Builds the grid, layouts, access helpers and `save_h5()` |
| `RG_Get_Number_Of_SigTypes.py` | Returns the numeric code for a signal-type name |
| `ResourceGrid_Flexlink.ods` | Source of the Symbol_Ref layouts |

## 10. Open items

- The DC range of the Symbol_Ref layouts differs from Symbol_Data. Profiles 12 and 24 start at k = 408 and profiles 6, 12 and 24 end at k = 429, while Symbol_Data uses 414-428. The layouts follow the spreadsheet exactly. Confirm whether these ends are intended.
- In the spreadsheet, profile 3 marks `Ref_P0` as unfilled and `Data` as blue, while profiles 6, 12 and 24 mark `Ref_P0` as dark green and `Data` as unfilled. The module resolves this by position (`Ref_P0` is always at `Ref_P1` + 1).
