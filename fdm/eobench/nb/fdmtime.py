"""the flight model's own cost: cruise, coupled, engines running; wall time per simulated second over several blocks: fdmtime.py ROOT"""
import sys, time, os
root = sys.argv[1]
os.environ['V22_ROOT'] = root
sys.path.insert(0, os.path.expanduser('~/git/V-22/fdm')); sys.path.insert(0, os.path.expanduser('~/git/V-22/fdm/eobench'))
from benchstart import start
f = start(47.26, 11.34, 90.0, 220.0, 8000.0, 2000.0, 40000.0)
AF = '/instrumentation/mv22b/afcs/'
f[AF + 'cpld'] = 1
for m in ('hdg-hold', 'alt-hold', 'spd-hold'):
    f[AF + m] = 1
f['/autopilot/settings/heading-bug-deg'] = 90.0; f['/autopilot/settings/target-altitude-ft'] = 8000.0; f['/autopilot/settings/target-speed-kt'] = 220.0
for i in range(2400):
    f.run()
best = 1e9; rows = []
for b in range(6):
    t = time.perf_counter()
    for i in range(4800):
        f.run()
    d = (time.perf_counter() - t) / 10.0
    rows.append(d); best = min(best, d)
print('%s: %.1f ms of CPU per simulated second at best, blocks %s' % (root, best * 1000, ' '.join('%.0f' % (r * 1000) for r in rows)))
print('height %.0f ft, %.0f KCAS' % (f['position/h-sl-ft'], f['velocities/vc-kts']))
