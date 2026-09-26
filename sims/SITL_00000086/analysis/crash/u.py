import pickle, numpy as np, sys
D=pickle.load(open('d.pkl','rb'))
def arr(t,f,filt=None):
    rows=D[t]
    if filt: rows=[r for r in rows if filt(r)]
    return np.array([r['t'] for r in rows]), np.array([r[f] for r in rows],dtype=float)
def show(t,fields,t0,t1,step=0.2,filt=None):
    rows=[r for r in D[t] if t0<=r['t']<=t1 and (filt is None or filt(r))]
    last=-1e9
    print(t,' '.join(f'{f:>8}' for f in fields))
    for r in rows:
        if r['t']-last>=step-1e-6:
            last=r['t']; print(f"{r['t']:6.2f} "+' '.join(f"{r[f]:8.3f}" if isinstance(r[f],float) else f"{r[f]:>8}" for f in fields))
def stats(t,fields,t0,t1,filt=None):
    rows=[r for r in D[t] if t0<=r['t']<=t1 and (filt is None or filt(r))]
    for f in fields:
        a=np.array([r[f] for r in rows],float)
        print(f"{t}.{f} [{t0}-{t1}] mean={a.mean():.3f} min={a.min():.3f} max={a.max():.3f} n={len(a)}")
