"""a batch case's candidates during the search: the best few by margin, with their Dubins words: ecands.py seed t1 t2 ..."""
import sys, io, contextlib, math
sys.path.insert(0, '.')
import batch, eobench as B
seed = int(sys.argv[1]); when = [float(a) for a in sys.argv[2:]]
s, lat, lon, hdg, kcas, alt, W, dist, brg = batch.case(seed)
dem = B.Dem()
with contextlib.redirect_stdout(io.StringIO()):
    f = B.start(lat, lon, hdg, kcas, alt, dem.elev_m(lat, lon) * B.M2FT, W)
f[B.AF + 'cpld'] = 1
f['/autopilot/settings/heading-bug-deg'] = hdg; f['/autopilot/settings/target-altitude-ft'] = alt; f['/autopilot/settings/target-speed-kt'] = kcas
for m in ('hdg-hold', 'alt-hold', 'spd-hold'):
    f[B.AF + m] = 1
dt = f['simulation/dt']; clock = {'t': 0.0}
fd = B.Finder(f, dem, lambda: clock['t'])
for i in range(int((max(when) + 0.1) / dt)):
    t = i * dt; clock['t'] = t
    f['position/terrain-elevation-asl-ft'] = dem.elev_m(f['position/lat-geod-deg'], f['position/long-gc-deg']) * B.M2FT
    if abs(t - 3.0) < dt / 2:
        for n in (0, 1): f['/controls/engines/engine[%d]/cutoff' % n] = 1
    if i % int(round(B.TICK_S / dt)) == 0:
        with contextlib.redirect_stdout(io.StringIO()):
            fd.update()
    f.run()
    if any(abs(t - w) < dt / 2 for w in when):
        la, lo = f['position/lat-geod-deg'], f['position/long-gc-deg']
        trk = math.atan2(f['velocities/v-east-fps'], f['velocities/v-north-fps']) * B.R2D
        print('t %.0f: trk %03.0f, reach %.2f NM, %d fields, chosen %s' % (t, trk % 360, f['ap/eo-reach-nm'], len(fd.fields), fd.chosen))
        alls = fd.runways(f['ap/eo-reach-nm'] + 2) + fd.fields
        gs = fd.glide_state()
        rows = []
        for c in alls:
          for pl in fd.plans(la, lo, trk, c):
            c['plan'] = pl; c['margin'] = pl[0]
            cl = fd.path_clear(la, lo, trk, c, gs)
            dub = pl[7]
            dN = (c['lat'] - la) * B.FT_DEG; dE = (c['lon'] - lo) * B.FT_DEG * math.cos(la * B.D2R)
            rows.append((pl[0], '%-5s %-14s crs %03.0f  %4.1f NM on %03.0f  margin %+7.0f  path %6.0f  clear %d  %-14s %s' % (
                c['kind'], c['name'][:14], c['crs'], math.hypot(dN, dE) / 6076, math.degrees(math.atan2(dE, dN)) % 360, pl[0], pl[1], cl, fd.kname(pl),
                'no tangent' if dub is None else 'out %5.0f turn %3.0f straight %5.0f circle %3.0f' % (dub[10], math.degrees(dub[4]), dub[2], math.degrees(dub[5])))))
        for r in sorted(rows, key=lambda r: -r[0])[:10]:
            print('  ' + r[1])
