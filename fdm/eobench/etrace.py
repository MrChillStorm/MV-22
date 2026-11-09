"""a batch case traced: the chosen site, the guidance's state and margin, the S-turn, the bank, the energy, against
where the flare began: etrace.py seed [every]"""
import sys, io, contextlib, math
sys.path.insert(0, '.')
import batch, eobench as B
seed = int(sys.argv[1]); ev = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
s, lat, lon, hdg, kcas, alt, W, dist, brg = batch.case(seed)
print('case %d: %.1f NM on %03.0f, hdg %03d, %d KCAS, %d ft, %d lb' % (seed, dist, brg, hdg, kcas, alt, W))
# (B.run with a hook: patch the print of each row to add the guidance's own)
dem = B.Dem()
f = B.start(lat, lon, hdg, kcas, alt, dem.elev_m(lat, lon) * B.M2FT, W)
f[B.AF + 'cpld'] = 1
f['/autopilot/settings/heading-bug-deg'] = hdg; f['/autopilot/settings/target-altitude-ft'] = alt; f['/autopilot/settings/target-speed-kt'] = kcas
for m in ('hdg-hold', 'alt-hold', 'spd-hold'):
    f[B.AF + m] = 1
dt = f['simulation/dt']; clock = {'t': 0.0}
fd = B.Finder(f, dem, lambda: clock['t'])
print('    t st    margin   path  brake  arc   bank   x       y      wheels  ground  energy  reach | site')
flare = None
for i in range(int(300 / dt)):
    t = i * dt; clock['t'] = t
    e = dem.elev_m(f['position/lat-geod-deg'], f['position/long-gc-deg'])
    if e is None:
        print('left the data'); break
    f['position/terrain-elevation-asl-ft'] = e * B.M2FT
    if abs(t - 3.0) < dt / 2:
        for n in (0, 1): f['/controls/engines/engine[%d]/cutoff' % n] = 1
    if f[B.AF + 'eng-out'] > 0.5 and f['hh/h-wheels-ft'] < 1000 and f['gear/gear-cmd-norm'] < 0.5:
        f['gear/gear-cmd-norm'] = 1.0
    if i % int(round(B.TICK_S / dt)) == 0:
        with contextlib.redirect_stdout(io.StringIO()) as buf:
            fd.update()
        for l in buf.getvalue().split('\n'):
            if ('chosen' in l or ' for ' in l or 'none better' in l): print('       ' + l.strip())
    f.run()
    if flare is None and f['ap/eo-phase'] > 2.5:
        flare = (t, f['ap/eo-site-x-ft'], f['ap/eo-site-y-ft'], f['ap/eo-site-margin-ft'])
        print('FLARE at %.1f s: x %+.0f y %+.0f, margin then %+.0f' % flare)
    if i % int(round(ev / dt)) == 0 and t > 2.5:
        print('%5.1f %2d %+8.0f %6.0f %6.1f %5.1f %+6.1f %+7.0f %+7.0f %6.0f %6.0f %7.0f %5.2f | %s' % (
            t, f['ap/eo-site-state'], f['ap/eo-site-margin-ft'], f['ap/eo-site-path-ft'], f['ap/eo-brake-flap-deg'],
            math.degrees(f['ap/eo-site-arc-rad']), f['attitude/phi-deg'], f['ap/eo-site-x-ft'], f['ap/eo-site-y-ft'], f['hh/h-wheels-ft'],
            f['position/terrain-elevation-asl-ft'], f['ap/eo-energy-ft'], f['ap/eo-reach-nm'], (fd.chosen or '-')[:40]))
    if t > 4 and B.DL.down(f):
        print('DOWN at %.1f s, %.1f ft/s, x %+.0f y %+.0f' % (t, -f['velocities/h-dot-fps'], f['ap/eo-site-x-ft'], f['ap/eo-site-y-ft'])); break
