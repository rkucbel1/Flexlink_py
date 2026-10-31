#!/usr/bin/env python3
"""
chest_mut.py  (Flexlink_py/HW_Development/Packet_Detector/SIM)

Fault injection for the payload channel estimate: each mutant changes one detail of pl_chest.v;
tb_pl_chest must FAIL on every one (Verilator; work/chest_units from make_chest_vectors.py).

    python chest_mut.py
"""
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = HERE.parent / "RTL" / "pl_chest.v"
MUTS = [
    ("mx[14] ? 5'd2 : mx[13] ? 5'd1 : 5'd0", "mx[14] ? 5'd2 : 5'd0", "packet shift k: one bit less"),
    ("v = {y[17], y} + $signed({12'd0, rr});", "v = {y[17], y};", "Y' truncated instead of rounded"),
    ("h5r <= sg4 ? ({y4r[15], y4r} - {y4i[15], y4i}) : ({y4i[15], y4i} - {y4r[15], y4r});",
     "h5r <= sg4 ? ({y4i[15], y4i} - {y4r[15], y4r}) : ({y4r[15], y4r} - {y4i[15], y4i});", "QAM pilot sign"),
    ("q5b <= (init && nz5) ?", "q5b <= (init && f5) ?", "noise sum over the pilots"),
    ("wp_i <= -da_i * $signed({1'b0, d_pr[14:0]});", "wp_i <= da_i * $signed({1'b0, d_pr[14:0]});",
     "delay profile without conj"),
    ("if (pv > (pmax >> 10) && pvn > {8'd0, a44}) last <= cnt[6:0];",
     "if (pv > (pmax >> 9) && pvn > {8'd0, a44}) last <= cnt[6:0];", "peak threshold -27 dB"),
    ("cls_of = (e <= 8'd24) ? 2'd0", "cls_of = (e <= 8'd28) ? 2'd0", "class 0 window end"),
    ("ms_b <= qam ? {33'd0, Np, 6'd0} : {34'd0, Np, 5'd0};", "ms_b <= qam ? {34'd0, Np, 5'd0} : {35'd0, Np, 4'd0};",
     "SNR mode threshold halved"),
    ("fsr <= fm[0] - fm[1] + fm[4] - fm[5];", "fsr <= fm[0] + fm[1] + fm[4] - fm[5];", "FIR real part sign"),
    ("cbase <= pl_coef_base(scode, cls_of(last), hi);",
     "cbase <= pl_coef_base(scode, cls_of(last), 1'b0);", "low-SNR weights always"),
    ("di <= di + dp3 - dp4;", "di <= di + dp3 + dp4;", "drift imaginary part"),
    ("hw_of = ((R == 4'd3 && bb >= 4'd1 && bb <= 4'd3) || (R == 4'd6 && bb == 4'd1)) ? 2'd2 : 2'd1;",
     "hw_of = (R == 4'd6 && bb == 4'd1) ? 2'd2 : 2'd1;", "smoothing weights (1 2 2 2 1) -> 1"),
    ("h_c1 <= cum[(hj + 7'd1) & 7'd15];", "h_c1 <= cum[hj & 7'd15];", "last symbol angle"),
    ("h_c0 + h_p2[30:15] + {15'd0, h_p2[14]};", "h_c0 + h_p2[30:15];", "interpolated angle truncated"),
    ("sm3 <= (sm3 == 2'd2) ? 2'd0 : sm3 + 2'd1;", "sm3 <= sm3 + 2'd1;", "Sm slot counter mod 4"),
    ("row10 <= (row10 == 4'd9) ? 4'd0 : row10 + 4'd1;", "row10 <= row10 + 4'd1;", "A slot counter mod 16"),
    ("ht <= (sym_j > nref - 7'd2) ? R : sym_t;", "ht <= sym_t;", "last symbol t"),
    ("                    w_drop <= 1'b1;\n                    wr_ptr <= w_base;", "                    w_drop <= 1'b1;",
     "rejected packet's symbols kept"),
    ("th = -cum[row & 7'd15];", "th = cum[row & 7'd15];", "row alignment angle sign"),
]
VF = ["../RTL/pl_roms.v", "../RTL/pl_mulseq.v", "../RTL/cf_tdp_ram.v", "../RTL/pd_sdp_ram.v",
      "../RTL/pd_tone_rom.v", "../RTL/pd_cordic.v", "../RTL/pd_fft2.v", "../RTL/cf_roms.v", "tb_pl_chest.v"]


def main() -> int:
    src = SRC.read_text()
    work = HERE / "work" / "chest_mut"
    work.mkdir(parents=True, exist_ok=True)
    caught = 0
    for old, new, name in MUTS:
        if src.count(old) != 1:
            print(f"NOT APPLIED: {name}")
            continue
        (work / "pl_chest.v").write_text(src.replace(old, new))
        b = subprocess.run(["verilator", "--binary", "--timing", "-Wno-fatal", "-Wno-WIDTH", "-Wno-INITIALDLY",
                            "-I../RTL", "--top-module", "tb_pl_chest", "--Mdir", str(work / "obj"), "-o", "tb", "-j", "2",
                            str(work / "pl_chest.v")] + VF, cwd=HERE, capture_output=True, text=True)
        if b.returncode:
            print(f"BUILD FAILED: {name}")
            continue
        try:
            r = subprocess.run([str(work / "obj" / "tb"), "+dir=."], cwd=HERE / "work" / "chest_units",
                               capture_output=True, text=True, timeout=900).stdout
        except subprocess.TimeoutExpired:
            r = "FAIL (timeout)"
        res = next((l for l in r.splitlines() if l.startswith(("PASS", "FAIL"))), "FAIL (no result: hang)")
        if res.startswith("FAIL"):
            caught += 1
            print(f"caught:   {name}")
        else:
            print(f"SURVIVED: {name} ({res})")
    print(f"{caught} of {len(MUTS)} mutants caught")
    return 0 if caught == len(MUTS) else 1


if __name__ == "__main__":
    sys.exit(main())
