#!/usr/bin/env python3
"""
ptx_stream.py  (Flexlink_py/HW_Development/Packet_Transmitter/PC)

The input stream of the PL transmitter (RTL/ptx_core.v) from Tx/Parameters_Tx_payload.json and
Tx/ControlSignals/Parameters_Control_Field.json: a parameter header with their Defined sections
(the PL derives the Calculated values, the control-field message and the gains), then the
transport blocks of every packet. FW/ptx_server writes exactly these records into the Pluto's TX
DMA; this tool makes them on the PC (tests, other interfaces).

    python ptx_stream.py --out stream.bin                          # JSON + random blocks, 1 packet
    python ptx_stream.py --blocks ../../../Tx/Generate_RandomPacketOfData/Bitstream_TransportBlks_Tx.h5 --out stream.bin
    python ptx_stream.py --packets 0 --out params.bin             # parameters only
    python ptx_stream.py --decode stream.bin                      # print the headers of a stream

stream.bin: little-endian 64-bit words, exactly what goes into the DMA buffer. --blocks takes a
.h5 with dataset TransportBlock_Bitstream (Generate_TransportBlock_Bitstream.py,
Generate_File_Bitstream.py): its bits are cut into AvailableDataBlocks x TransportBlockSize
packets (the last packet zero-filled).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = next((p for p in [HERE, *HERE.parents] if (p / "flexlink_paths.py").is_file()), HERE.parents[2])
sys.path.insert(0, str(HERE))
import ptx_reference as R  # noqa: E402

RATE_TXT = {v: k for k, v in R.RATES.items()}
MOD_TXT = {v: k for k, v in R.MOD_CODE.items()}


def load_blocks(path: Path) -> np.ndarray:
    try:
        import h5py
        with h5py.File(path, "r") as f:
            return np.asarray(f["TransportBlock_Bitstream"][()]).ravel().astype(np.int64)
    except ImportError:
        sys.path.insert(0, str(ROOT / "Flexlink_BasicModules" / "Equalizer" / "pilot_ce"))
        from minih5 import MiniH5
        h = MiniH5(str(path))
        return np.asarray(h.dataset(h.children(h.root_ohdr)["TransportBlock_Bitstream"])).ravel().astype(np.int64)


def packets_from_bits(cfg: R.TxConfig, bits: np.ndarray) -> list[np.ndarray]:
    per = cfg.ndb * cfg.tb_size
    n = max(1, -(-bits.size // per))
    pad = np.zeros(n * per, np.int64)
    pad[:bits.size] = bits
    return [pad[i * per:(i + 1) * per].reshape(cfg.ndb, cfg.tb_size) for i in range(n)]


def decode_header(w: np.ndarray) -> dict:
    """Header words -> fields (inverse of ptx_reference.param_header)."""
    w0, w1, w2, w3, w4, w5 = (int(x) for x in w[:6])
    f = lambda v, lo, n: (v >> lo) & ((1 << n) - 1)          # noqa: E731
    return {"magic_ok": f(w0, 0, 32) == R.MAGIC, "version": f(w0, 32, 8),
            "Mod": MOD_TXT.get(f(w1, 0, 2)), "RefSymbolSpacing": f(w1, 2, 4), "RefSignalSpacing": f(w1, 6, 5),
            "PacketLength": f(w1, 11, 7), "Port": f(w1, 18, 1), "NumberOfAntennas": f(w1, 19, 1) + 1,
            "ClientFlag": "Client" if f(w1, 20, 1) else "Service", "Use_Preamble": f(w1, 21, 1),
            "Use_ControlField": f(w1, 22, 1), "EncoderCodeRate": RATE_TXT[["12", "23", "34", "56"][f(w1, 23, 2)]],
            "EncoderBlockSize": [648, 1296, 1944, None][f(w1, 25, 2)], "Repetition": f(w1, 27, 5),
            "C_init_Dat": f(w1, 32, 8), "Tx_Ref_Clk_Cnt": f(w1, 40, 14), "CRC": ["Gen24", "?", "?", "?"][f(w1, 54, 2)],
            "C_init_ID1": f(w2, 0, 31), "C_init_ID2": f(w2, 31, 31),
            "gain_pre": f(w3, 0, 16), "gain_cf": f(w3, 16, 16), "gain_pl": f(w3, 32, 16), "gap": 16 * f(w3, 48, 16),
            "packets": f(w4, 0, 32), "SampleRate": 1000 * f(w4, 32, 32),
            "ControlField_C_init_ID1": f(w5, 0, 31), "ControlField_C_init_Dat": f(w5, 31, 31)}


def decode_stream(words: np.ndarray) -> list[dict]:
    """Walk a stream record by record (the core's view, without the resync on garbage)."""
    out, i = [], 0
    while i + 6 <= words.size:
        h = decode_header(words[i:i + 6])
        if not h["magic_ok"] or h["version"] != R.VERSION:
            out.append({"word": i, "error": "not a header"})
            i += 1
            continue
        d = {k: h[k] for k in ("Mod", "RefSymbolSpacing", "RefSignalSpacing", "PacketLength", "EncoderBlockSize",
                               "EncoderCodeRate", "Repetition", "C_init_Dat", "Port")}
        try:
            cfg = R.TxConfig.from_defined({**d, "ClientFlag": h["ClientFlag"], "NumberOfAntennas": h["NumberOfAntennas"],
                                           "C_init_ID1": h["C_init_ID1"], "C_init_ID2": h["C_init_ID2"],
                                           "Use_Preamble": h["Use_Preamble"], "Use_ControlField": h["Use_ControlField"],
                                           "Tx_Ref_Clk_Cnt": h["Tx_Ref_Clk_Cnt"], "CRC": h["CRC"],
                                           "SampleRate": h["SampleRate"]},
                                          {"C_init_ID1": h["ControlField_C_init_ID1"],
                                           "C_init_Dat": h["ControlField_C_init_Dat"]})
            cfg.check()
        except (AssertionError, KeyError, TypeError) as exc:
            out.append({"word": i, "error": f"rejected header ({exc})", **h})
            i += 6
            continue
        n = h["packets"] * cfg.ndb * ((cfg.tb_size + 63) // 64)
        out.append({"word": i, **h, "AvailableDataBlocks": cfg.ndb, "TransportBlockSize": cfg.tb_size,
                    "block_words": n})
        i += 6 + n
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="PL transmitter stream from Parameters_Tx_payload.json")
    ap.add_argument("--params", type=Path, default=ROOT / "Tx" / "Parameters_Tx_payload.json")
    ap.add_argument("--cf-params", type=Path, default=ROOT / "Tx" / "ControlSignals" / "Parameters_Control_Field.json")
    ap.add_argument("--blocks", type=Path, help="transport-block .h5 (default: random blocks)")
    ap.add_argument("--packets", type=int, default=1, help="random packets (0: parameters only)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--gap", type=int, default=20000, help="idle samples after each packet")
    ap.add_argument("--out", type=Path, default=Path("stream.bin"))
    ap.add_argument("--decode", type=Path, help="print the records of a stream file and exit")
    a = ap.parse_args(argv)
    if a.decode:
        for r in decode_stream(np.fromfile(a.decode, "<u8")):
            print(r)
        return 0
    cfg = R.TxConfig.from_params(a.params, a.cf_params)
    cfg.check()
    if a.blocks:
        pk = packets_from_bits(cfg, load_blocks(a.blocks))
    else:
        rng = np.random.default_rng(a.seed)
        pk = [R.random_tbs(cfg, rng) for _ in range(a.packets)]
    w = R.record_words(cfg, pk, a.gap).astype("<u8")
    w.tofile(a.out)
    print(f"{a.params.name}: {cfg}")
    print(f"  derived: AvailableDataBlocks {cfg.ndb}, TransportBlockSize {cfg.tb_size}, ZeroPadding {cfg.zero_pad}, "
          f"NumberOfDataSigs {cfg.data_bits}")
    print(f"Wrote {a.out}: header + {len(pk)} packet(s), {w.size} words ({w.nbytes} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
