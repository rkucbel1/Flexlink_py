#!/usr/bin/env python3
"""
ldpc_mut.py  (Flexlink_py/HW_Development/Packet_Detector/SIM)

Fault injection for the LDPC decoder core: each mutant changes one detail of pl_ldpc_core.v and
tb_pl_ldpc must FAIL on every one (Verilator; work/ldpc_units from make_ldpc_vectors.py).

    python ldpc_mut.py

Not in the list because they cannot change a result (equivalent mutants, each tried and
survived):
- dropping the column-order tie-break of the minimum: two edges tie only when mn1 == mn2, and
  then both get the same magnitude;
- writing the compressed messages on every edge of a layer instead of the first: the same values
  go to the same address;
- APP saturation at -512 instead of -511: a saturated APP gives |q| >= 463, capped at 63 either
  way, so the difference never reaches a message or a hard bit;
- the bank-address carry with b <= s27 instead of b < s27: it moves lane 0 to the next row group
  on every edge of a layer alike, which only renumbers the layer's rows.
Also not in the list: letting P1 run two layers ahead of P2. It would be wrong, but cannot
happen: P2 never stalls, so P1 never gets that far ahead with the decoder's timing.
"""
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = HERE.parent / "RTL" / "pl_ldpc_core.v"
MUTS = [
    ("wire        haz1 = p1_par ? pend0[e1_col] : pend1[e1_col];", "wire        haz1 = 1'b0;",
     "hazard wait removed"),
    ("(y1_b ? rot1(hb_rd1, y1_sb) : {NL{1'b0}})", "{NL{1'b0}}", "syndrome ignores the second edge of a pair"),
    ("u_m1 = mn1[t2_par][t2_g][i] - (mn1[t2_par][t2_g][i] >> 2);",
     "u_m1 = mn1[t2_par][t2_g][i] - (mn1[t2_par][t2_g][i] >> 1);", "alpha 0.5 instead of 0.75 (mn1)"),
    ("u_m2 = mn2[t2_par][t2_g][i] - (mn2[t2_par][t2_g][i] >> 2);",
     "u_m2 = mn2[t2_par][t2_g][i];", "mn2 not scaled"),
    ("t4_L <= rot10(t3_L, inv27(t3_s27));", "t4_L <= rot10(t3_L, t3_s27);", "write-back rotation wrong"),
    ("t4_h <= rot1(t3_h, inv27(t3_s27));", "t4_h <= rot1(t3_h, t3_s27);", "hard-bit rotation wrong"),
    ("(t_sum > 12'sd63 || t_sum < -12'sd63) ? 6'd63", "(t_sum > 12'sd62 || t_sum < -12'sd62) ? 6'd62",
     "magnitude cap 62"),
    ("t_r = s2_it0 ? 7'sd0 :", "t_r = 1'b0 ? 7'sd0 :", "messages of iteration 0 not zero"),
    ("u_mag = (tb_eo == tb_i1[i*5 +: 5]) ? u_m2 : u_m1;", "u_mag = u_m1;", "minimum edge gets mn1"),
    ("mpr[s4_par][s4_g][i] <= mpr[s4_par][s4_g][i] ^ s4_neg[i];", "mpr[s4_par][s4_g][i] <= mpr[s4_par][s4_g][i];",
     "sign product not accumulated"),
    ("if (p2_it == MAXIT) begin", "if (p2_it == MAXIT - 1) begin", "one iteration less"),
    ("if (y2_end && |y_row) y_fail <= 1'b1;", "", "syndrome failure of a row group lost"),
]
VF = ["../RTL/pl_ldpc_rom.v", "../RTL/pl_lutram1.v", "../RTL/cf_tdp_ram.v", "../RTL/pd_sdp_ram.v", "tb_pl_ldpc.v"]


def main() -> int:
    src = SRC.read_text()
    work = HERE / "work" / "ldpc_mut"
    work.mkdir(parents=True, exist_ok=True)
    caught = 0
    for old, new, name in MUTS:
        if src.count(old) != 1:
            print(f"NOT APPLIED: {name}")
            continue
        (work / "pl_ldpc_core.v").write_text(src.replace(old, new))
        b = subprocess.run(["verilator", "--binary", "--timing", "-Wno-fatal", "-Wno-WIDTH", "-Wno-INITIALDLY",
                            "--top-module", "tb_pl_ldpc", "--Mdir", str(work / "obj"), "-o", "tb", "-j", "2",
                            str(work / "pl_ldpc_core.v")] + VF, cwd=HERE, capture_output=True, text=True)
        if b.returncode:
            print(f"BUILD FAILED: {name}")
            continue
        r = subprocess.run([str(work / "obj" / "tb"), "+dir=."], cwd=HERE / "work" / "ldpc_units",
                           capture_output=True, text=True, timeout=900).stdout
        res = next((l for l in r.splitlines() if l.startswith(("PASS", "FAIL"))), "no result")
        if res.startswith("FAIL"):
            caught += 1
            print(f"caught:   {name}")
        else:
            print(f"SURVIVED: {name} ({res})")
    print(f"{caught} of {len(MUTS)} mutants caught")
    return 0 if caught == len(MUTS) else 1


if __name__ == "__main__":
    sys.exit(main())
