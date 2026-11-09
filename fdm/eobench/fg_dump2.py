"""the terrain and land cover round LOWI from the running FlightGear, every 30 m, with the scenery all loaded first"""
import sys, os, time, shutil
sys.argv = ['x']
src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fg_lowi.py')).read().split('import random')[0]
exec(src)
import math
LAT0, LON0 = 47.2602, 11.3440
kl = 111320.0; kc = math.cos(math.radians(LAT0))
pts = [(a, b) for a in (0, -11000, 11000) for b in (0, -11000, 11000)]
for n, (dn, de) in enumerate(pts):
    fg = None
    for _ in range(100):
        try:
            fg = Fg(launch=False); break
        except OSError:
            time.sleep(2)
    for k, v in (('latitude-deg', LAT0 + dn / kl), ('longitude-deg', LON0 + de / (kl * kc)), ('altitude-ft', 12000), ('heading-deg', 90), ('airspeed-kt', 200), ('onground', 'false'), ('airport-id', ''), ('runway', '')):
        fg.put('/sim/presets/' + k, v)
    fg.put('/sim/freeze/master', 0)
    fg.f.write('run reset\r\n'); fg.f.flush()
    time.sleep(5); fg.sock.close(); time.sleep(12)
    for _ in range(100):
        try:
            fg = Fg(launch=False); break
        except OSError:
            time.sleep(2)
    fg.wait_ready()
    fg.put('/sim/freeze/master', 1)
    fg.put('/sim/eodump/load', 0); fg.put('/sim/eodump/load', 1)
    time.sleep(2)
    last, same = None, 0
    for i in range(14):
        fg.put('/sim/eodump/probe', 0); fg.put('/sim/eodump/probe', 1)
        time.sleep(5)
        bad = fg.get('/sim/eodump/probe-bad')
        same = same + 1 if bad == last else 0
        last = bad
        if same >= 3:
            break
    print('place %d (%+d, %+d m): %s of 1089 points without scenery' % (n, dn, de, last), flush=True)
    fg.put('/sim/eodump/file', 'lowi_dem2_%d.txt' % n)
    fg.put('/sim/eodump/go', 0); fg.put('/sim/eodump/go', 1)
    t0 = time.time()
    while time.time() - t0 < 200:
        time.sleep(4)
        if fg.get('/sim/eodump/row') >= 1067:
            break
    fg.put('/sim/eodump/row', 0)
    time.sleep(1)
    fg.put('/sim/freeze/master', 0)
    fg.sock.close()
print('all done', flush=True)
