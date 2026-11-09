"""FlightGear's dump (fg_dump2.py) to lowi_dem2.npz for the bench: dem2conv.py lowi_dem2.txt"""
import sys, os, re
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eobench as B
E = I = None; G = {}; solid = {}; names = {}
for src in sys.argv[1:]:
    loc = {}; rows_e, rows_i = [], []
    with open(src, errors='replace') as f:
        h = f.readline().split()
        lat0, lon0, half, step, n = float(h[2]), float(h[4]), float(h[6]), float(h[8]), int(h[10])
        for l in f:
            if l[:2] == 'D ':
                a = l.rstrip('\n').split(' ', 3)
                key = a[2] if a[2] == '-' else a[2] + ' ' + a[3]
                if key not in G:
                    g = len(G); G[key] = g
                    if a[2] == '-':
                        names[g] = ['-']; solid[g] = 0
                    else:
                        ns = a[3].split(',')
                        pick = next((x for x in ns if x in B.OPEN), None) or next((x for x in ns if B.PAVED(x)), None) or next((x for x in ns if x in B.ROUGH), None) or ns[0]
                        names[g] = [pick] + [x for x in ns if x != pick]; solid[g] = int(a[2])
                loc[int(a[1])] = G[key]
            else:
                v = l.split()
                if len(v) == 2 * n:
                    rows_e.append(np.array(v[0::2], dtype=np.float32)); rows_i.append(np.array(v[1::2], dtype=np.int32))
    e = np.array(rows_e); ii = np.array(rows_i)
    m = np.full(max(loc) + 2, -1, dtype=np.int32)
    for k, g in loc.items():
        m[k] = g
    ii = m[ii]                                   # (-1 stays -1: the last entry)
    if E is None:
        E = np.full((n, n), np.nan, dtype=np.float32); I = np.full((n, n), -1, dtype=np.int32)
    gap = I < 0
    have = ii >= 0
    # (no scenery there, or known but this one has its material)
    take = gap & have
    E[take] = e[take]; I[take] = ii[take]
    print('%s: %d rows, this one knows %.1f %%; map now %.2f %% empty' % (os.path.basename(src), len(e), 100 * have.mean(), 100 * (I < 0).mean()), flush=True)
I = I.astype(np.int16)
print('%d kinds of ground, %.2f %% without scenery' % (len(names), 100 * (I < 0).mean()))
nm = np.array([names[i][0] for i in range(len(names))]); sd = np.array([solid[i] for i in range(len(names))], dtype=np.int8)
np.savez_compressed(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'lowi_dem2.npz'), e=E, i=I, names=nm, solid=sd, head=np.array([lat0, lon0, half, step, n]))
