"""compare_with_emulator.py: the same command sequences on the co-simulated board (port 5101) and
pd_board_emulator.py (port 5102); every reply must match. Start both first:
    ./obj/fw_cosim 5101 &    python ../../PC/pd_board_emulator.py --port 5102 &
Env for fw_cosim: MM2S_P, S2MM_P (stream duty, default 0.8 / 0.7), BUSY_TICKS (clocks per DMA
poll, default 64; 20000 fills the event FIFO), SEED."""
import sys, struct, numpy as np
HERE = __import__("pathlib").Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent.parent / "PC")); sys.path.insert(0, str(HERE.parent))
import pd_protocol as pr, pd_hw_test as hw, make_tb_vectors as mk, pd_reference as ref

def session(port, iq, p, chunks, debug):
    b = hw.Board("127.0.0.1", port, 900)
    b.call(pr.RESET); b.call(pr.SET_PARAMS, p.pack())
    out, i = [], 0
    for c in chunks:
        b.load(iq[i:i + c]); evs, done = b.run(c, debug)
        dbg = b.get_debug(done).tobytes() if debug and done else b""
        out.append(([pr.astuple(e) for e in evs], done, dbg)); i += c
    return out

def case(name, iq, p, chunks, debug=False):
    a = session(5101, iq, p, chunks, debug); e = session(5102, iq, p, chunks, debug)
    n_ev = sum(len(x[0]) for x in e); n_tr = sum(1 for x in e for r in x[0] if r[4] & pr.EV_TRUNCATED)
    diffs = []
    for k, (ra, re_) in enumerate(zip(a, e)):
        for fa, fe in zip(ra[0], re_[0]):
            if fa[:3] + fa[4:] != fe[:3] + fe[4:] or abs(fa[3] - fe[3]) > 2:
                diffs.append((k, fa, fe))
        if len(ra[0]) != len(re_[0]): diffs.append((k, len(ra[0]), len(re_[0])))
        if ra[1:] != re_[1:]: diffs.append((k, "done/debug differ"))
    q8 = [abs(fa[3] - fe[3]) for ra, re_ in zip(a, e) for fa, fe in zip(ra[0], re_[0])]
    print(f"{name}: {len(chunks)} RUNs, {n_ev} events ({n_tr} truncated), ToneCheck max diff "
          f"{max(q8, default=0)} Q8 LSB -> {'SAME' if not diffs else 'DIFFERENT ' + str(diffs[:3])}")
    return not diffs

pr.astuple = __import__("dataclasses").astuple
ok = True
iq = mk.stress_vector()
p = pr.Params(arm_holdoff=1); p.input_stage = 1
ok &= case("stress, holdoff 1, one RUN", iq, p, [iq.shape[0]])
ok &= case("stress, holdoff 1, 7 uneven RUNs, debug", iq, p, [5000, 1, 20000, 33333, 4095, 30000, iq.shape[0] - 92429], True)
p128 = pr.Params(arm_holdoff=1, n_cfo=128); p128.input_stage = 1
ok &= case("stress, N_CFO 128, many events", iq, p128, [64000, 64000])
rep = np.fromfile(HERE.parent.parent / "TestVectors" / "rep3.bin", "<i2").reshape(-1, 2).astype(np.int64)
pr3 = pr.Params(arm_holdoff=95044)
ok &= case("rep3, ends inside a window", rep[:5000], pr3, [5000])
ok &= case("rep3, RUN boundaries inside windows", rep, pr3, [3401, 2000, 46, 1, 100000, 98000, 2000, rep.shape[0] - 205448])
p4k = pr.Params(arm_holdoff=1, n_cfo=4096, tone_thresh=0)
ok &= case("rep3, N_CFO 4096, tone check off", rep, p4k, [rep.shape[0]])
print("ALL SAME" if ok else "DIFFERENCES FOUND")
