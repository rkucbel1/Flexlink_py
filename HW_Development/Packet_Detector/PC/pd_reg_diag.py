#!/usr/bin/env python3
"""
pd_reg_diag.py  (Flexlink_py/HW_Development/Packet_Detector/PC)

Diagnostic (firmware build 6, REG_READ / REG_WRITE): shows the AXI DMA and detector registers,
then loads and reads back the sample buffer with the detector held in reset and with it
released, to see which state corrupts the data and whether the DMA or the detector is active
when it should be idle.

    python pd_reg_diag.py
    python pd_reg_diag.py --dma 0x40400000
"""
from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pd_protocol as pr  # noqa: E402
import pd_hw_test as hw  # noqa: E402

REG_READ, REG_WRITE = 9, 10
PD = 0x43C00000
PD_REGS = {"ID": 0x00, "CTRL": 0x04, "STATUS": 0x08, "SAMPLES": 0x0C, "EV_LEVEL": 0x14, "DBG_SENT": 0x1C,
           "INPUT_STAGE": 0x38}
DMA_REGS = {"MM2S_DMACR": 0x00, "MM2S_DMASR": 0x04, "MM2S_SA": 0x18, "MM2S_LENGTH": 0x28,
            "S2MM_DMACR": 0x30, "S2MM_DMASR": 0x34, "S2MM_DA": 0x48, "S2MM_LENGTH": 0x58}


def rd(b, a: int) -> int:
    return struct.unpack("<I", b.call(REG_READ, struct.pack("<I", a)))[0]


def wr(b, a: int, v: int) -> int:
    return struct.unpack("<I", b.call(REG_WRITE, struct.pack("<II", a, v)))[0]


def show(b, dma: int, tag: str) -> None:
    pd = "  ".join(f"{k} 0x{rd(b, PD + o):08x}" for k, o in PD_REGS.items())
    dm = "  ".join(f"{k} 0x{rd(b, dma + o):08x}" for k, o in DMA_REGS.items())
    print(f"  [{tag}] detector: {pd}\n  [{tag}] DMA:      {dm}")


def load_check(b, iq: np.ndarray, tag: str) -> int:
    b.load(iq)
    back = b.read(0, iq.shape[0])
    bad = int(np.count_nonzero(np.any(back != iq, axis=1)))
    print(f"  {tag}: {bad} bad samples of {iq.shape[0]}")
    return bad


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="DMA / detector register diagnostic")
    ap.add_argument("--host", default=hw.DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=pr.DEFAULT_PORT)
    ap.add_argument("--dma", type=lambda v: int(v, 0), default=0x40400000, help="AXI DMA base address")
    ap.add_argument("--vector", type=Path, default=HERE.parent / "TestVectors" / "rep3_40.bin")
    a = ap.parse_args(argv)
    iq = np.fromfile(a.vector, "<i2").reshape(-1, 2)
    b = hw.Board(a.host, a.port, 60)
    info = b.ping()
    if info.build < 6:
        print(f"board firmware build {info.build}: this needs build 6")
        return 2
    print(f"board build {info.build}")
    print("1. registers after boot (no command yet)")
    show(b, a.dma, "boot")
    print("2. RESET (the firmware writes the parameters and releases the detector)")
    b.call(pr.RESET)
    show(b, a.dma, "after RESET")
    print("3. detector held in reset (CTRL = 1), load + read back")
    wr(b, PD + 0x04, 1)
    load_check(b, iq, "held in reset")
    show(b, a.dma, "held")
    print("4. detector released (CTRL = 0), load + read back")
    wr(b, PD + 0x04, 0)
    load_check(b, iq, "released")
    show(b, a.dma, "released")
    print("5. held in reset again, load + read back")
    wr(b, PD + 0x04, 1)
    load_check(b, iq, "held again")
    wr(b, PD + 0x04, 0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
