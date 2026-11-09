import sys, math, os, json, random
sys.path.insert(0, '.')
import nasbench as NB, eobench as B, batch as BT
def field_case(seed):
    r = random.Random(1000 + seed)
    brg, dist = r.uniform(0, 360), r.uniform(0, 1.5)
    lat = BT.LOWI[0] + dist * math.cos(math.radians(brg)) / 60
    lon = BT.LOWI[1] + dist * math.sin(math.radians(brg)) / 60 / math.cos(math.radians(BT.LOWI[0]))
    return seed, lat, lon, round(r.uniform(0, 360)), round(r.uniform(160, 215)), 6000, 38722, dist, brg
BT.case = field_case
F0 = NB.NasalFinder
last = [-9]
class T(F0):
    def update(s):
        F0.update(s)
        f = s.f
        g = lambda p: f[p]
        st = g('ap/eo-site-state')
        if st > 0.5 and s.t() - last[0] > 2.5:
            last[0] = s.t()
            try:
                trk = math.degrees(g('ap/eo-site-track-rad')); cc = g('ap/eo-site-course-rad') * 57.2958
            except Exception:
                trk = cc = 0
            print('%5.1f st %d x %6.0f y %6.0f hdg-crs %+5.0f tx %6.0f ty %6.0f dT %5.0f a1 %.2f a2 %.2f leg %d s %+d turn %+d cm %d R %5.0f Lf %5.0f KCAS %3.0f h %5.0f bank %+3.0f mg %6.0f' % (s.t(), st, g('ap/eo-site-x-ft'), g('ap/eo-site-y-ft'), ((trk - cc + 180) % 360) - 180, g('ap/eo-site-tx-ft'), g('ap/eo-site-ty-ft'), g('ap/eo-site-t-dist-ft'), g('ap/eo-site-a1-now-rad'), g('ap/eo-site-a2-sent-rad'), g('ap/eo-site-leg'), g('ap/eo-site-s'), g('ap/eo-site-turn'), g('ap/eo-site-commit'), g('ap/eo-site-r-ft'), g('ap/eo-site-final-ft'), f['velocities/vc-kts'], g('hh/h-wheels-ft'), f['attitude/phi-deg'], g('ap/eo-site-margin-ft')), flush=True)
B.Finder = T
_, lat, lon, hdg, kcas, alt, W, dist, brg = BT.case(int(sys.argv[1]))
out = B.run(lat, lon, hdg, kcas, alt, sets=json.loads(os.environ.get('SETS', '{}')), W=W, quiet=True, T=160.0)
