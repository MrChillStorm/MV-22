"""the 40 starts through the real eosite.nas: nbatch.py N [seed0] [case ...]"""
import sys, io, contextlib, multiprocessing as mp, json
sys.path.insert(0, os.path.expanduser('~/git/V-22/fdm/eobench'))
import nasbench as NB
import eobench as B
import batch as BT
import numpy as np
BT.B = B
STATS = {}
import random, math, os
if os.environ.get('FIELD'):
    # (the scenario as FlightGear flies it: over the field, at its weight, any heading)
    def field_case(seed):
        r = random.Random(1000 + seed)
        brg, dist = r.uniform(0, 360), r.uniform(0, 1.5)
        lat = BT.LOWI[0] + dist * math.cos(math.radians(brg)) / 60
        lon = BT.LOWI[1] + dist * math.sin(math.radians(brg)) / 60 / math.cos(math.radians(BT.LOWI[0]))
        hdg, kt = round(r.uniform(0, 360)), round(r.uniform(160, 215))
        if os.environ.get('BENCH_KCAS'):
            kt = float(os.environ['BENCH_KCAS'])
        if os.environ.get('BENCH_AT'):             # (over the field's centre, all starts; a heading: BENCH_AT="260")
            lat, lon, dist = BT.LOWI[0], BT.LOWI[1], 0.0
            hdg = float(os.environ['BENCH_AT']) if float(os.environ['BENCH_AT']) >= 0 else hdg
        return seed, lat, lon, hdg, kt, float(os.environ.get('BENCH_ALT', 6000)), float(os.environ.get('BENCH_W', 38722)), dist, brg
    BT.case = field_case
def one(seed):
    s, lat, lon, hdg, kcas, alt, W, dist, brg = BT.case(seed)
    buf = io.StringIO()
    F = []
    F0 = NB.NasalFinder
    class Rec(F0):
        def __init__(self, *a):
            F0.__init__(self, *a); F.append(self)
    B.Finder = Rec
    try:
        with contextlib.redirect_stdout(buf):
            out = B.run(lat, lon, hdg, kcas, alt, sets=json.loads(os.environ.get('SETS', '{}')), W=W, quiet=True, nac=float(os.environ.get('BENCH_NAC', 0)))
    except Exception as e:
        out = {'error': repr(e)}
    B.Finder = F0
    td = out.get('td')
    txt = buf.getvalue()
    log = [l for l in txt.split('\n') if 'site:' in l and ': field ' not in l]
    v = np.array(F[0].vm_s) * 1000 if F and F[0].vm_s else np.zeros(1)
    cnt = F[0].vm.counts() if F else [0, 0, 0]
    errs = F[0].errs if F else 0
    return seed, (lat, lon, hdg, kcas, alt, W, dist, brg), td, log, out, dict(mean=float(v.mean()), p99=float(np.percentile(v, 99)), mx=float(v.max()), n=len(v), cnt=cnt, errs=errs)
if __name__ == "__main__":
    mp.set_start_method('fork')
    a = sys.argv[1:]
    n = int(a[0]); s0 = int(a[1]) if len(a) > 1 else 1
    seeds = [int(x) for x in a[2:]] or list(range(s0, s0 + n))
    with mp.Pool(int(os.environ.get('NB_POOL', 4))) as p:
        res = p.map(one, seeds)
    ok = 0
    for seed, st, td, log, out_, tm in res:
        lat, lon, hdg, kcas, alt, W, dist, brg = st
        tail = '  [vm %.2f ms mean, %.1f p99, %.1f worst; %d ticks; get %d set %d geo %d%s]' % (tm['mean'], tm['p99'], tm['mx'], tm['n'], *tm['cnt'], ' ERRORS %d' % tm['errs'] if tm['errs'] else '')
        if out_.get('left') is not None:
            print('%4d  %.1f NM on %03.0f, hdg %03d, %3d KCAS, %4d ft, %5d lb: left the terrain data at %.0f s%s' % (seed, dist, brg, hdg, kcas, alt, W, out_['left'], tail)); continue
        if out_.get('diverged') is not None or out_.get('error'):
            print('%4d  start failed (%s)' % (seed, out_.get('error') or 'diverged')); continue
        if td is None:
            print('%4d  %.1f NM on %03.0f, hdg %03d, %3d KCAS, %4d ft, %5d lb: NOT DOWN %s' % (seed, dist, brg, hdg, kcas, alt, W, log[-1:] if log else '')); continue
        wet = td.get('wet')
        good = td['sink_g'] < 12 and td['gear'] > 0.99 and td['clear'] > 1.0 and abs(td['bank']) < 10 and td['pitch'] < 7.3 and td['stop']['roll_clear'] > 0 and not wet
        ok += good
        tail = ('  [pitch %.1f gear %.2f bank %.1f clear %.1f roll %.0f nose %+.1f crab %+.1f stopoff %+.0f space %.1f wet %s rotor %.0f-%.0f]' % (td['pitch'], td['gear'], td['bank'], td['clear'], td['stop']['roll_clear'],
                td.get('nose', 0), td.get('crab', 0), td['stop'].get('off_ft', 0), td['sink'],
                '%d:%s:%.0fkt:%.0fft' % (len(wet['wheels']), wet['cover'], wet['kt'], wet['ft']) if wet else '0', *td.get('nr', (0, 0)))) + tail
        sp = td['stop']['past_end_ft']
        print('%4d  %.1f NM on %03.0f, hdg %03d, %3d KCAS, %4d ft, %5d lb: %-10s %5.1f ft/s %3.0f KCAS tips %+5.1f, on %-20s slope %s, stop %s past the end; %s%s' % (
            seed, dist, brg, hdg, kcas, alt, W, 'INTACT' if good else 'not intact', td['sink_g'], td['kcas'], td['clear'], td['cover'],
            'n/a' if td['slope'] is None else '%+5.1f %%' % (100 * td['slope']), 'n/a' if sp is None else '%+6.0f' % sp,
            (td['chosen'] or 'no site').split(' ')[0] + ' ' + str(len([l for l in log if ' for ' in l or 'chosen' in l or 'none better' in l])) + ' choices'
            + ('' if 'vs_site' not in td else ', down %+.0f ft along its point, %+.0f across' % td['vs_site'][:2]), tail), flush=True)
    valid = sum(1 for r in res if r[4].get('left') is None and r[4].get('diverged') is None and not r[4].get('error'))
    print('%d of %d intact; %d of them stayed on the terrain data' % (ok, len(res), valid))
    nrs = [r[2]['nr'] for r in res if r[2] and 'nr' in r[2]]
    if nrs:
        print('rotor from the cut to the ground: lowest %.0f %%, highest %.0f %%; median of each flight\'s lowest %.0f %%' % (
            min(x[0] for x in nrs), max(x[1] for x in nrs), float(np.median([x[0] for x in nrs]))))
    print('vm: mean of means %.2f ms, worst p99 %.1f, worst tick %.1f' % (np.mean([r[5]['mean'] for r in res]), max(r[5]['p99'] for r in res), max(r[5]['mx'] for r in res)))
