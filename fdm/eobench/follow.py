"""one line's margin second by second in the flight: follow.py seed lat lon crs"""
import sys, io, contextlib, math
sys.path.insert(0, '.')
import batch, eobench as B
seed, la_, lo_, crs = int(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
s, lat, lon, hdg, kcas, alt, W, dist, brg = batch.case(seed)
dem = B.Dem()
with contextlib.redirect_stdout(io.StringIO()):
    f = B.start(lat, lon, hdg, kcas, alt, dem.elev_m(lat, lon) * B.M2FT, W)
f[B.AF + 'cpld'] = 1
f['/autopilot/settings/heading-bug-deg'] = hdg; f['/autopilot/settings/target-altitude-ft'] = alt; f['/autopilot/settings/target-speed-kt'] = kcas
for m in ('hdg-hold', 'alt-hold', 'spd-hold'):
    f[B.AF + m] = 1
clock = {'t': 0.0}
fd = B.Finder(f, dem, lambda: clock['t'])
dt = f['simulation/dt']
for i in range(int(12 / dt)):
    t = i * dt; clock['t'] = t
    if abs(t - 3.0) < dt / 2:
        for n in (0, 1): f['/controls/engines/engine[%d]/cutoff' % n] = 1
    if i % int(round(B.TICK_S / dt)) == 0:
        with contextlib.redirect_stdout(io.StringIO()): fd.update()
    f['position/terrain-elevation-asl-ft'] = dem.elev_m(f['position/lat-geod-deg'], f['position/long-gc-deg']) * B.M2FT
    if abs(t - 6.0) < 0.001: print('   ORIGIN %s %s ticks %d sampled %s tried %d of %d rough %d of %d lines %d crs300 %s' % (fd.origin, round(t, 2), fd.ticks, fd.count, fd.ti, len(fd.tries), fd.ri, len(fd.rough_tries), len(fd.fields), sorted(set((round(c['lat'], 4), round(c['lon'], 4), c['kind']) for c in fd.fields if c['crs'] == 300.0))[:4]))
    f.run()
    if i % 24 == 1 and 5.7 <= t <= 7.0:
        c = next((c for c in fd.fields if abs(c['lat'] - la_) < 1e-5 and abs(c['lon'] - lo_) < 1e-5 and c['crs'] == crs), None)
        la, lo = f['position/lat-geod-deg'], f['position/long-gc-deg']
        trk = math.atan2(f['velocities/v-east-fps'], f['velocities/v-north-fps']) * B.R2D
        if i % 120 == 1: print('   PROPS', {k: round(f[k], 1) for k in ('ap/eo-site-r-ft', 'ap/eo-site-final-ft', 'ap/eo-site-turn-ft-rad', 'ap/eo-site-r0-ft', 'ap/eo-reach-nm', 'ap/eo-ld-up', 'velocities/vt-fps')})
        if c is None:
            print('%5.1f  not found yet; reach %.2f, trk %03.0f, bank %+.0f, eng-out %d phase %d open %s' % (t, f['ap/eo-reach-nm'], trk % 360, f['attitude/phi-deg'], f[B.AF + 'eng-out'], f['ap/eo-phase'], fd.open)); continue
        pl = fd.plans(la, lo, trk, c)[0]
        print('%5.1f  margin %+6.0f (%s), reach %.2f NM, trk %03.0f, bank %+4.0f, rank margin %s clear %s, chosen %s, open %s, sampling %d, in cands at %s' % (
            t, pl[0], fd.kname(pl), f['ap/eo-reach-nm'], trk % 360, f['attitude/phi-deg'], '%+.0f' % c['margin'] if 'margin' in c else '-', c.get('clear'), fd.chosen, fd.open, fd.sampling,
            next((k for k, x in enumerate(fd.cands) if x is c), None)))
        print('        FDM on %d new %d state %d k %d s %+d side %+d turn %+d phase %d | trk-cmd %.0f phi-cmd %s | p0 cost %.0f p1 %.0f p2 %.0f p3 %.0f' % tuple(
            [f['ap/eo-site-' + n] for n in ('on', 'new', 'state', 'k', 's', 'side', 'turn')] + [f['ap/eo-phase'], f['ap/eo-site-trk-deg'] % 360, '%+.0f' % f['ap/eo-phi-cmd-deg']]
            + [f['ap/eo-site-cost%d-ft' % k] for k in range(4)]))
        print('        eo-fly %.0f phi-ref %+.1f deg  phi %+.1f  p %+.1f deg/s  aileron %+.3f  scas/lat %s  ap-lat %s  hh %s' % (f['ap/eo-fly'], f['ap/phi-ref-rad'] * 57.3, f['attitude/phi-deg'],
              f['velocities/p-rad_sec'] * 57.3, f['fcs/aileron-cmd-norm'], f['scas/lat-calc'] if True else 0, f['ap/lat-on'] if 0 else '-', f['hh/on-new']))
