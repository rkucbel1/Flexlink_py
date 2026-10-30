import sys, json, glob, numpy as np, ldpc_lib as L
R=[json.loads(l) for f in sorted(glob.glob('results/snr10.jsonl')) for l in open(f) if l.startswith('{')]
part=int(sys.argv[1])
for r in R[part::2]:
    code=L.Code(r['code']); x=r['snr10']['lay10']
    row=[]
    for d in (0.0,0.5):
        s=L.channel_llr(code,r['mod'],x+d,r['copies'],600,np.random.default_rng(11)).sum(0)
        f=L.decode_layered(code,s,10)[0].any(1).mean()
        a=L.decode_layered_fixed(code,L.quantise(s,8,7),10,10,7)[0].any(1).mean()
        b=L.decode_layered_fixed(code,L.quantise(s,4,6),10,8,6)[0].any(1).mean()
        row.append((f,a,b))
    print(json.dumps({"cfg":f"{r['code']} {r['mod']} {r['copies']}","snr":x,"row":row}),flush=True)
