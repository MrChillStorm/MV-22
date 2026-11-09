"""ENG OUT's finder over open farmland, the line count's worst case: every cell open, level crop, no airport, in place of
LOWI's terrain. Flat and all one cover by construction, not a dump of FlightGear's Kansas: real section roads, creeks and
towns would refuse some of these lines, so what it counts is an upper bound.

    kansas.py BENCH_DIR SCRIPT [T_s]      BENCH_DIR holds nasbench.py and the VM; SCRIPT the eosite.nas to fly"""
import sys, os, statistics
BENCH, SCRIPT = sys.argv[1], sys.argv[2]
T = float(sys.argv[3]) if len(sys.argv) > 3 else 300.0
sys.path.insert(0, BENCH)
import nasbench as N
B = N.B
LAT, LON, ELEV_M = 38.5, -98.5, 550.0


class Flat:
    def elev_m(s, lat, lon):
        return ELEV_M

    def geodinfo(s, lat, lon):
        return ELEV_M, {'names': ['DryCrop'], 'solid': 1}


B.Dem = Flat
N.NasalFinder.SCRIPT = SCRIPT
N.NasalFinder.AIRPORTS = {}
rows = []
u0 = N.NasalFinder.update


def update(s):
    u0(s)
    n = len(s.vm_s)
    if n % 25 == 0:
        rows.append((s.t(), s.vm.call('eosite', 'bench_lines')[0], s.vm.counts()[2], max(s.vm_s[-25:]) * 1000, sum(s.vm_s[-25:]) / 25 * 1000))
        print('t %5.0f  lines %6.0f  geodinfo %7d  tick worst %6.1f ms, mean %5.1f ms' % rows[-1], flush=True)


N.NasalFinder.update = update
out = B.run(LAT, LON, 139.0, 205.0, ELEV_M / 0.3048 + 6000, {}, T=T, quiet=True)
td = out.get('td')
print('errors in the VM: see NASAL lines above' if any('NASAL' in str(r) for r in rows) else 'ran')
if rows:
    print('most lines %.0f; worst tick %.1f ms; geodinfo calls %d' % (max(r[1] for r in rows), max(r[3] for r in rows), rows[-1][2]))
if td:
    print('down: %.1f ft/s, %.0f KCAS, site %s, %+.0f ft past the end' % (td['sink'], td['kcas'], td['chosen'], td['stop']['past_end_ft'] or 0))
