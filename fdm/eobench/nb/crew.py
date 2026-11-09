import sys
sys.path.insert(0, '.')
import nasbench as NB, eobench as B, batch as BT
F0 = NB.NasalFinder
class C(F0):
    n = 0
    def update(s):
        F0.update(s)
        C.n += 1
        if C.n in (150, 170, 190):
            s.vm.call('eosite', 'next'); print('%7.1f  site: (SITE key pressed)' % s.t())
B.Finder = C
_, lat, lon, hdg, kcas, alt, W, dist, brg = BT.case(1)
B.run(lat, lon, hdg, kcas, alt, W=W, quiet=True)
