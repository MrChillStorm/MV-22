"""random ENG OUT starts round LOWI on the bench: batch.py N [seed0]"""
import sys, io, contextlib, random, math
sys.path.insert(0, '.')
from multiprocessing import Pool
import eobench as B
buf_lines = lambda log: log
LOWI = (47.26023, 11.34397)
DEM = B.Dem()
def case(seed):
    # (over the valley's floor, the ground under the start below 2300 ft: starts over the slopes, 1300 ft above them,
    # are not the scenario)
    r = random.Random(seed)
    while True:
        brg, dist = r.uniform(0, 360), r.uniform(0, 2.0)
        lat = LOWI[0] + dist * math.cos(math.radians(brg)) / 60
        lon = LOWI[1] + dist * math.sin(math.radians(brg)) / 60 / math.cos(math.radians(LOWI[0]))
        e = DEM.elev_m(lat, lon)
        if e is not None and e * B.M2FT < 2300:
            break
    return seed, lat, lon, round(r.uniform(0, 360)), round(r.uniform(160, 220)), round(r.uniform(5500, 7000)), round(r.uniform(38700, 47500)), dist, brg
def one(seed):
    s, lat, lon, hdg, kcas, alt, W, dist, brg = case(seed)
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            out = B.run(lat, lon, hdg, kcas, alt, W=W, quiet=True)
    except Exception as e:
        out = {'error': repr(e)}
    td = out.get('td')
    log = [l for l in buf.getvalue().split('\n') if 'site:' in l and ': field ' not in l]
    return seed, (lat, lon, hdg, kcas, alt, W, dist, brg), td, log, out
if __name__ == '__main__':
    n = int(sys.argv[1]); s0 = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    with Pool(8) as p:
        res = p.map(one, range(s0, s0 + n))
    ok = 0
    for seed, st, td, log, out_ in res:
        lat, lon, hdg, kcas, alt, W, dist, brg = st
        if out_.get('left') is not None:
            print('%4d  %.1f NM on %03.0f, hdg %03d, %3d KCAS, %4d ft, %5d lb: left the terrain data at %.0f s' % (seed, dist, brg, hdg, kcas, alt, W, out_['left'])); continue
        if out_.get('diverged') is not None or out_.get('error'):
            print('%4d  start failed (%s)' % (seed, out_.get('error') or 'diverged')); continue
        if td is None:
            print('%4d  %.1f NM on %03.0f, hdg %03d, %3d KCAS, %4d ft, %5d lb: NOT DOWN %s' % (seed, dist, brg, hdg, kcas, alt, W,
                  [l for l in buf_lines(log)][-1:] if log else '')); continue
        good = td['sink'] < 12 and td['gear'] > 0.99 and td['clear'] > 1.0 and abs(td['bank']) < 10 and td['pitch'] < 7.3 and td['stop']['roll_clear'] > 0
        ok += good
        sp = td['stop']['past_end_ft']
        print('%4d  %.1f NM on %03.0f, hdg %03d, %3d KCAS, %4d ft, %5d lb: %-10s %5.1f ft/s %3.0f KCAS tips %+5.1f, on %-20s slope %s, stop %s past the end; %s' % (
            seed, dist, brg, hdg, kcas, alt, W, 'INTACT' if good else 'not intact', td['sink'], td['kcas'], td['clear'], td['cover'],
            'n/a' if td['slope'] is None else '%+5.1f %%' % (100 * td['slope']), 'n/a' if sp is None else '%+6.0f' % sp,
            (td['chosen'] or 'no site').split(' ')[0] + ' ' + str(len([l for l in log if ' for ' in l or 'chosen' in l or 'none better' in l])) + ' choices'
            + ('' if 'vs_site' not in td else ', down %+.0f ft along its point, %+.0f across' % td['vs_site'][:2])
            + ('' if not out_.get('m') or out_['m']['choice'] is None else '; margin %+.0f at the choice (path %.1f NM), %s at the final, %s at the flare' % (
                out_['m']['choice'], out_['m']['path'] / 6076.1, *['n/a' if out_['m'][k] is None else '%+.0f' % out_['m'][k] for k in ('final', 'flare')]))), flush=True)
    valid = sum(1 for r in res if r[4].get('left') is None and r[4].get('diverged') is None and not r[4].get('error'))
    print('%d of %d intact; %d of them stayed on the terrain data' % (ok, len(res), valid))
