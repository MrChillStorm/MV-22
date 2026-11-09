import sys, numpy as np
sys.path.insert(0, '.')
import nasbench as NB, eobench as B, batch as BT
F = []
F0 = NB.NasalFinder
F0.STAGES = True
class Rec(F0):
    def __init__(s, *a):
        F0.__init__(s, *a); F.append(s)
B.Finder = Rec
seed = int(sys.argv[1])
_, lat, lon, hdg, kcas, alt, W, dist, brg = BT.case(seed)
B.run(lat, lon, hdg, kcas, alt, W=W, quiet=True)
v = np.array(F[0].vm_s) * 1000
print('case %d: vm %.2f ms mean, %.1f p99, %.1f worst, ticks over 5 ms %d of %d' % (seed, v.mean(), np.percentile(v, 99), v.max(), (v > 5).sum(), len(v)))
print(F[0].vm.calls('eosite', 'bench_report'))
print(F[0].vm.calls('eosite', 'bench_stages'))
import ctypes as C, vm
g = (C.c_double * 3)(); vm.lib.nb_gc(g)
print('gc: %d collections, %.1f ms total, %.2f ms worst' % (g[2], g[0] * 1000, g[1] * 1000))
