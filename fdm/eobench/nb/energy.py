import sys, os, json, io, contextlib, math, random
sys.path.insert(0, '.')
os.environ['FIELD'] = '1'
import multiprocessing as mp
import nbatch as NBt
import nasbench as NB, eobench as B, batch as BT
P = ['position/h-sl-ft', 'velocities/vt-fps', 'velocities/vg-fps', 'velocities/vc-kts', 'attitude/psi-deg', 'attitude/phi-deg', 'ap/eo-phase', 'ap/eo-site-state', 'ap/eo-site-margin-ft', 'ap/eo-site-path-ft', 'ap/eo-reach-nm', 'ap/eo-brake-ratio', 'ap/eo-brake-nom-ratio', 'ap/eo-ld-up', 'ap/eo-site-turn-ft-rad', 'gear/gear-pos-norm', 'ctl/flaperon-deg', 'ap/eo-energy-ft', 'hh/h-wheels-ft', 'position/lat-geod-deg', 'position/long-gc-deg', 'ap/eo-site-on', 'ctl/nacelle-deg', 'atmosphere/sigma', 'velocities/h-dot-fps', 'ap/eo-site-a1-now-rad', 'ap/eo-site-t-dist-ft', 'ap/eo-site-leg', 'ap/eo-site-commit', 'ap/eo-site-x-ft', 'ap/eo-site-y-ft', 'ap/eo-site-sturn-rad']
F0 = NB.NasalFinder
def one(seed):
    rows = []
    class T(F0):
        def update(s):
            F0.update(s)
            f = s.f
            r = {}
            for p in P:
                try:
                    r[p] = f[p]
                except Exception:
                    r[p] = None
            r['t'] = s.t(); r['site'] = s.chosen
            rows.append(r)
    B.Finder = T
    _, lat, lon, hdg, kcas, alt, W, dist, brg = BT.case(seed)
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        out = B.run(lat, lon, hdg, kcas, alt, W=W, quiet=True, every=5.0)
    B.Finder = F0
    log = [l for l in buf.getvalue().split('\n') if 'site:' in l and 'land cover' not in l]
    td = out.get('td')
    json.dump(dict(seed=seed, rows=rows, log=log, td={k: v for k, v in (td or {}).items() if k in ('sink', 'kcas', 'slope', 'clear', 'gear', 'cover', 'chosen', 't', 'vs_site')}, start=dict(hdg=hdg, kcas=kcas, dist=dist, brg=brg)), open('en_%d.json' % seed, 'w'), default=str)
    return seed
if __name__ == '__main__':
    mp.set_start_method('fork')
    with mp.Pool(8) as p:
        print(len(p.map(one, range(1, 41))), 'flights recorded')
