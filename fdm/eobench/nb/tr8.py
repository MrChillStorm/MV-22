import sys, math
sys.path.insert(0, '.')
import nasbench as NB, eobench as B
F0 = NB.NasalFinder
last = [None, -9, 0, 0, 0]
class T(F0):
    def update(s):
        F0.update(s)
        f = s.f
        st = f['ap/eo-site-state']
        g = lambda p: f[p]
        if st > 0.5 and s.t() - last[1] > 3:
            t = s.t(); h = g('position/h-sl-ft'); e = h + g('velocities/vt-fps')**2 / (2 * 32.174)
            dt = t - last[1]
            print('%5.1f st %d x %6.0f y %6.0f h %5.0f mg %6.0f KCAS %3.0f sink %3.0f esink %4.0f bank %+3.0f flap %4.1f nb %2.0f rq %.2f mid %5.0f over %5.0f gear %.1f nac %2.0f LD %.1f' % (t, st, g('ap/eo-site-x-ft'), g('ap/eo-site-y-ft'), g('hh/h-wheels-ft'), g('ap/eo-site-margin-ft'), f['velocities/vc-kts'], -g('velocities/h-dot-fps'), (last[2] - e) / dt, f['attitude/phi-deg'], g('ctl/flaperon-deg'), g('ap/eo-nb-deg'), g('ap/eo-brake-ratio'), g('ap/eo-site-mid-ft'), 0, g('gear/gear-pos-norm'), g('ctl/nacelle-deg'), g('velocities/vg-fps') / max(1, (last[2] - e) / dt)), flush=True)
            last[1] = t; last[2] = e
B.Finder = T
import os, json, random, batch as BT
def field_case(seed):
    r = random.Random(1000 + seed)
    brg, dist = r.uniform(0, 360), r.uniform(0, 1.5)
    lat = BT.LOWI[0] + dist * math.cos(math.radians(brg)) / 60
    lon = BT.LOWI[1] + dist * math.sin(math.radians(brg)) / 60 / math.cos(math.radians(BT.LOWI[0]))
    return seed, lat, lon, round(r.uniform(0, 360)), round(r.uniform(160, 215)), 6000, 38722, dist, brg
BT.case = field_case
_, lat, lon, hdg, kcas, alt, W, dist, brg = BT.case(int(sys.argv[1]))
out = B.run(lat, lon, hdg, kcas, alt, sets=json.loads(os.environ.get('SETS', '{}')), W=W, quiet=True, T=160.0)
