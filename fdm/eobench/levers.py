"""the best line's margin at a case's time with the final shortened and the circle at the bank limit: levers.py seed t"""
import sys, io, contextlib, math
sys.path.insert(0, '.')
import batch, eobench as B
seed, tw = int(sys.argv[1]), float(sys.argv[2])
s, lat, lon, hdg, kcas, alt, W, dist, brg = batch.case(seed)
dem = B.Dem()
with contextlib.redirect_stdout(io.StringIO()):
    f = B.start(lat, lon, hdg, kcas, alt, dem.elev_m(lat, lon) * B.M2FT, W)
fd = B.Finder(f, dem, lambda: 0.0)
dt = f['simulation/dt']
for i in range(int(tw / dt)):
    if abs(i * dt - 3.0) < dt / 2:
        for n in (0, 1): f['/controls/engines/engine[%d]/cutoff' % n] = 1
    if i % int(round(B.TICK_S / dt)) == 0 and i * dt > 3.0:
        with contextlib.redirect_stdout(io.StringIO()): fd.update()
    f['position/terrain-elevation-asl-ft'] = dem.elev_m(f['position/lat-geod-deg'], f['position/long-gc-deg']) * B.M2FT
    f.run()
# the search finished, from the cut's position (its lines as found by then)
with contextlib.redirect_stdout(io.StringIO()):
    while fd.sampling or fd.heap or fd.rheap:
        fd.update()
la, lo = f['position/lat-geod-deg'], f['position/long-gc-deg']
trk = math.atan2(f['velocities/v-east-fps'], f['velocities/v-north-fps']) * B.R2D
alls = fd.runways(f['ap/eo-reach-nm'] + 2) + fd.fields
Lf0, R44, R55 = f['ap/eo-site-final-ft'], f['ap/eo-site-r-ft'], f['ap/eo-site-r0-ft']
print('case %d at %.0f s: %d lines, the final %.0f ft, circle %.0f ft (44 deg), %.0f (55 deg)' % (seed, tw, len(alls), Lf0, R44, R55))
for R, rn in ((R44, '44'), (R55, '55')):
    for Lj in (Lf0, 4000, 3000, 2000):
        fd._cache = {'ap/eo-site-final-ft': Lj, 'ap/eo-site-r-ft': R}
        best = max(((fd.plans(la, lo, trk, c)[0][0], c) for c in alls), key=lambda t: t[0])
        n_in = sum(1 for c in alls if fd.plans(la, lo, trk, c)[0][0] >= 0)
        fd._cache = None
        print('  circle %s deg, final %5.0f: best %+7.0f (%s %s crs %03.0f), %d lines in reach' % (rn, Lj, best[0], best[1]['kind'], best[1]['name'], best[1]['crs'], n_in))
