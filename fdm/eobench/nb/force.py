import sys
sys.path.insert(0, '.')
import nasbench as NB, eobench as B
hdg_force = float(sys.argv[1])
F0 = NB.NasalFinder
class Fz(F0):
    def update(s):
        F0.update(s)
        if s.f['ap/eo-site-on'] < 0.5:
            s.f['ap/eo-open-on'] = 1; s.f['ap/eo-open-hdg-deg'] = hdg_force
B.Finder = Fz
lat, lon, hdg, kcas, alt = 47.27011, 11.31595, 341.0, 163.0, 5982.0
out = B.run(lat, lon, hdg, kcas, alt, W=38722.0, cut_s=0.5, quiet=True, every=2.0, T=60.0)
rows = out['track']
clear = min(r[5] for r in rows[1:]) if rows else 0
print('forced %03.0f: lowest over the ground %.0f ft; rows:' % (hdg_force, clear))
for r in rows[:14:2]:
    print('   %5.1f %5.0f ft over %5.0f  %5.1f KCAS hdg %5.1f bank %+5.1f sink %5.1f' % (r[0], r[5], r[6], r[7], r[8] % 360, r[9], -r[10]))
