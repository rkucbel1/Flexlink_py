import sys, json, glob, numpy as np, ldpc_lib as L
R=[json.loads(l) for f in sorted(glob.glob('results/snr10.jsonl')) for l in open(f) if l.startswith('{')]
for r in R[int(sys.argv[1])::2]:
    code=L.Code(r['code']); x=r['snr10']['lay10']
    s=L.channel_llr(code,r['mod'],x,r['copies'],600,np.random.default_rng(13)).sum(0)
    out={"float":L.decode_layered(code,s,10)[0].any(1).mean()}
    for c in (0.5,1,2,4,8):
        out[c]=L.decode_layered_fixed(code,L.quantise(s,c,7),10,10,7)[0].any(1).mean()
    print(json.dumps({"cfg":f"{r['code']} {r['mod']} {r['copies']}","b":{str(k):v for k,v in out.items()}}),flush=True)
