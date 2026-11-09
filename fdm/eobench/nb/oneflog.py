import sys
sys.path.insert(0, '.')
import nasbench as NB, eobench as B, batch as BT, re
if len(sys.argv) > 2:
    NB.NasalFinder.SCRIPT = sys.argv[2]
    B.TICK_S = float(re.search(r'var TICK_S = ([0-9.]+)', open(sys.argv[2]).read()).group(1))
seed = int(sys.argv[1])
_, lat, lon, hdg, kcas, alt, W, dist, brg = BT.case(seed)
print('start %.5f %.5f hdg %d %d KCAS %d ft %d lb' % (lat, lon, hdg, kcas, alt, W))
B.run(lat, lon, hdg, kcas, alt, W=W, quiet=True)
