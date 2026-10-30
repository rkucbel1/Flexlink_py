"""SNR (Es/N0) for BLER 10 % per decoder variant, AWGN, all-zero code word through a random
scrambler. Variants: sim (flooding 15, averaged probabilities), flood15 LLR-sum, layered float,
layered fixed point."""
import sys, json, time, numpy as np, ldpc_lib as L

def variants():
    v = {"sim": None, "flood15_llr": None}
    for it in (5, 8, 10, 15):
        v[f"lay{it}"] = it
    for q, app, msg in ((5, 7, 5), (6, 8, 6), (7, 9, 7)):
        v[f"fx{q}_{app}_{msg}_it10"] = (q, app, msg, 10)
    return v
V = variants()

def run_point(code, mod, snr, copies, B, seed):
    rng = np.random.default_rng(seed)
    llr = L.channel_llr(code, mod, snr, copies, B, rng)
    s = llr.sum(0)
    res = {}
    res["sim"] = L.decode_flood(code, L.beliefs_sim(llr), 15).any(1).mean()
    res["flood15_llr"] = L.decode_flood(code, -s, 15).any(1).mean()      # belief positive = 1
    for name, it in V.items():
        if name.startswith("lay"):
            res[name] = L.decode_layered(code, s, it)[0].any(1).mean()
        elif name.startswith("fx"):
            q, app, msg, it = it
            scale = (1 << (q - 3)) / max(np.mean(np.abs(s)), 1e-9)          # mean |LLR| = 2^(q-3)
            res[name] = L.decode_layered_fixed(code, L.quantise(s, scale, q), it, app, msg)[0].any(1).mean()
    return res

def snr10(snrs, bl):
    bl = np.maximum(np.array(bl), 1e-4)
    for i in range(len(snrs) - 1):
        if bl[i] >= 0.1 > bl[i + 1]:
            a, b = np.log10(bl[i]), np.log10(bl[i + 1])
            return snrs[i] + (np.log10(0.1) - a) / (b - a) * (snrs[i + 1] - snrs[i])
    return float("nan")

def config(t, mod, copies, B=300):
    code = L.Code(t)
    # bracket with the float layered 15 decoder: find s where BLER crosses 0.5 and 0.01
    s = -10 * np.log10(copies) + {"BPSK": -8, "QPSK": -5, "16QAM": 1, "64QAM": 6}[mod]
    while True:
        r = run_point(code, mod, s, copies, 100, 99)
        if r["lay15"] < 0.6: break
        s += 1.0
    snrs = list(np.arange(s - 1.5, s + 2.51, 0.25))
    table = {k: [] for k in V}
    for i, x in enumerate(snrs):
        r = run_point(code, mod, x, copies, B, 1000 + i)
        for k in V: table[k].append(r[k])
        if all(r[k] == 0 for k in V): snrs = snrs[:i + 1]; break
    out = {k: snr10(snrs, table[k]) for k in V}
    return out

if __name__ == "__main__":
    t, mod, copies = sys.argv[1], sys.argv[2], int(sys.argv[3])
    t0 = time.time()
    r = config(t, mod, copies)
    print(json.dumps({"code": t, "mod": mod, "copies": copies, "snr10": r, "s": round(time.time() - t0)}), flush=True)
