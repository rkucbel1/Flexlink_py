#!/usr/bin/env python3
"""
soft_mut.py  (Flexlink_py/HW_Development/Packet_Detector/SIM)

Fault injection for the payload demapper, combiner and decoder stage: each mutant changes one
detail of pl_demap.v, pl_comb.v or pl_dec.v; tb_pl_soft must FAIL on every one (Verilator; work/lq_units from
make_lq_vectors.py).

Left out because they cannot change any output (equivalent): a threshold tie (A z = j e gives the
same D for both levels), the m cap (it only acts with sh = 0, where every LLR with N != 0
saturates anyway), BPSK pairs across packets (every Data RE count per row is even) and the
mask of a packet's first tone (column 0 is DC in every reference row).

    python soft_mut.py
"""
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
RTL = HERE.parent / "RTL"
# (file, original, mutant, name)
MUTS = [
    ("pl_demap.v", "zi <= {pc[31], pc} - {pd[31], pd};", "zi <= {pd[31], pd} - {pc[31], pc};", "z imaginary sign"),
    ("pl_demap.v", "cz_r <= ({{7{zr[32]}}, zr} <<< 5) - ({{7{zr[32]}}, zr} <<< 2);",
     "cz_r <= ({{7{zr[32]}}, zr} <<< 5);", "64QAM C1 = 32"),
    ("pl_demap.v", "az_r <= ({{7{zr[32]}}, zr} <<< 1) + {{7{zr[32]}}, zr};", "az_r <= {{7{zr[32]}}, zr};",
     "16QAM thresholds with A = 1"),
    ("pl_demap.v", "ea49 <= ({{4{e3[39]}}, e3} <<< 4) + {{4{e1[39]}}, e1};", "ea49 <= ({{4{e3[39]}}, e3} <<< 4);",
     "49 e -> 48 e"),
    ("pl_demap.v", "a = lt[1] ? 3'd0 : lt[3] ? 3'd3 : lt[5] ? 3'd4 : 3'd7;",
     "a = lt[1] ? 3'd0 : lt[3] ? 3'd4 : lt[5] ? 3'd3 : 3'd7;", "64QAM bit 2 level set"),
    ("pl_demap.v", "2'd2: den <= {3'd0, s_sumn, 5'd0} + {6'd0, s_sumn, 2'd0};",
     "2'd2: den <= {3'd0, s_sumn, 5'd0};", "16QAM C2 = 32"),
    ("pl_demap.v", "if (den == 56'd0) begin n_m <= 16'hFFFF; n_sh <= 6'd0;",
     "if (den == 56'd0) begin n_m <= 16'hFFFF; n_sh <= 6'd16;", "sumN 0: sh"),
    ("pl_demap.v", "if (u >= 61'sd4095) lsat = 12'sd2047;", "if (u >= 61'sd8191) lsat = 12'sd2047;",
     "LLR not saturated"),
    ("pl_demap.v", "c_md <= n_md; c_sc <= n_sc;", "c_sc <= n_sc;", "modulation kept from the last packet"),
    ("pl_demap.v", "else begin r = (u + 61'sd1) >>> 1; lsat = r[11:0]; end",
     "else begin r = u >>> 1; lsat = r[11:0]; end", "LLR truncated"),
    ("pl_demap.v", "{2'd3, 2'd1}: begin e_n0 <= sn_r2; e_n1 <= sn_i0; end",
     "{2'd3, 2'd1}: begin e_n0 <= sn_i0; e_n1 <= sn_r2; end", "64QAM bit order"),
    ("pl_comb.v", "wire        g0 = ga1[0] ^ ga2[0], g1 = gb1[0] ^ gb2[0];",
     "wire        g0 = ga1[0] ^ ga2[0], g1 = g0;", "Gold second bit"),
    ("pl_comb.v", "st2 = {r[0] ^ r[1] ^ r[2] ^ r[3], r[30:1]};", "st2 = {r[0] ^ r[1] ^ r[3], r[30:1]};",
     "Gold x2 taps"),
    ("pl_comb.v", "acc16 = (s > 17'sd32767) ? 16'sd32767 : (s < -17'sd32767) ? -16'sd32767 : s[15:0];",
     "acc16 = s[15:0];", "copy sum not saturated"),
    ("pl_comb.v", "r = ({a[15], a} + 17'sd4) >>> 3;", "r = {a[15], a} >>> 3;", "Lq truncated"),
    ("pl_comb.v", "lq7 = (r > 14'sd63) ? 7'sd63 : (r < -14'sd63) ? -7'sd63 : r[6:0];",
     "lq7 = (r > 14'sd63) ? 7'sd63 : (r < -14'sd64) ? -7'sd64 : r[6:0];", "Lq -64"),
    ("pl_comb.v", "if ((lf_sop ? 10'd0 : w) + 10'd1 == (lf_sop ? cfg_ndb : ndb)) drop <= 1'b1;",
     "if ((lf_sop ? 10'd0 : w) + 10'd1 == (lf_sop ? cfg_ndb : ndb) + 10'd1) drop <= 1'b1;", "NDB_A + 1 code words"),
    ("pl_comb.v", "q1 <= lq7(r3h1 ? rdb[rb][31:16] : rdb[rb][15:0]);", "q1 <= lq7(r3h1 ? rdb[rb][15:0] : rdb[rb][31:16]);",
     "readout half"),
    ("pl_comb.v", "wire [31:0] sb_n = sb_q + {sa_q[30:0], 1'b0} + {x0[30:0], 1'b0} + x1;",
     "wire [31:0] sb_n = sb_q + {sa_q[30:0], 1'b0} + x0 + x1;", "signature B"),
    ("pl_dec.v", "localparam integer CRC_POLY = 24'h864CFB;", "localparam integer CRC_POLY = 24'h864CFA;", "CRC24 polynomial"),
    ("pl_dec.v", "4'd9: nb <= 11'd1272;", "4'd9: nb <= 11'd1271;", "1944 rate 2/3 block length"),
    ("pl_dec.v", "if (wi < kw) crc <= crc27(crc, c_out_bits);", "if (wi <= kw) crc <= crc27(crc, c_out_bits);",
     "CRC over one word too many"),
    ("pl_dec.v", "msk <= (rem_n > 11'd27) ? {27{1'b1}} : ((27'd1 << rem_n[4:0]) - 27'd1);",
     "msk <= {27{1'b1}};", "transport block not cut at K - 24"),
    ("pl_dec.v", "pk <= {32'd0, pk_new[63:32]}; pk_n <= pk_cnt - 7'd32;", "pk <= {32'd0, pk_new[63:32]}; pk_n <= pk_cnt - 7'd31;",
     "bit packing"),
    ("pl_dec.v", "f_wd <= {crc == 24'd0, 3'd0, r_it, r_w, 3'd0, nb};", "f_wd <= {crc == 24'd0, 4'd0, r_it[2:0], r_w, 3'd0, nb};",
     "iteration count bit 3"),
    ("pl_dec.v", "for (i = 0; i < 27; i = i + 1) c_in_llr[i*7 +: 7] <= gb[i];",
     "for (i = 0; i < 27; i = i + 1) c_in_llr[i*7 +: 7] <= gb[26 - i];", "decoder input order"),
]
TB = ["../RTL/pl_soft_roms.v", "../RTL/cf_tdp_ram.v", "../RTL/pl_ldpc_core.v", "../RTL/pl_ldpc_rom.v",
      "../RTL/pl_lutram1.v", "../RTL/pd_sdp_ram.v", "tb_pl_soft.v"]


def main() -> int:
    work = HERE / "work" / "soft_mut"
    work.mkdir(parents=True, exist_ok=True)
    src = {f: (RTL / f).read_text() for f in ("pl_demap.v", "pl_comb.v", "pl_dec.v")}
    caught = 0
    for f, old, new, name in MUTS:
        if src[f].count(old) != 1:
            print(f"NOT APPLIED: {name}")
            continue
        for g in src:
            (work / g).write_text(src[g].replace(old, new) if g == f else src[g])
        b = subprocess.run(["verilator", "--binary", "--timing", "-Wno-fatal", "-Wno-WIDTH", "-Wno-INITIALDLY",
                            "-I../RTL", "--top-module", "tb_pl_soft", "--Mdir", str(work / "obj"), "-o", "tb", "-j", "2",
                            str(work / "pl_demap.v"), str(work / "pl_comb.v"), str(work / "pl_dec.v")] + TB,
                           cwd=HERE, capture_output=True, text=True)
        if b.returncode:
            print(f"BUILD FAILED: {name}")
            continue
        try:
            r = subprocess.run([str(work / "obj" / "tb"), "+dir=.", "+gap=1"], cwd=HERE / "work" / "lq_units",
                               capture_output=True, text=True, timeout=600).stdout
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
