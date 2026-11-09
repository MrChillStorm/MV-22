import sys
sys.path.insert(0, '.')
import nasbench as NB, eobench as B
lat, lon, hdg, kcas, alt = 47.27011, 11.31595, 341.0, 163.0, 5982.0
out = B.run(lat, lon, hdg, kcas, alt, W=38722.0, cut_s=0.5, quiet=True, every=2.0)
for r in out['track'][:16]:
    print('%5.1f ph %d st %d %5.0f ft over %5.0f  %5.1f KCAS hdg %5.1f bank %+5.1f sink %5.1f open %s' % (r[0], r[1], r[2], r[5], r[6], r[7], r[8] % 360, r[9], -r[10], r[12]))
