"""the lines in reach from the cut, where they are and when the search found them in the flight: inreach.py seed"""
import sys, io, contextlib, math
sys.path.insert(0, '.')
import batch, eobench as B
seed = int(sys.argv[1])
s, lat, lon, hdg, kcas, alt, W, dist, brg = batch.case(seed)
dem = B.Dem()
def fly(T, search_frozen_at=None):
    with contextlib.redirect_stdout(io.StringIO()):
        f = B.start(lat, lon, hdg, kcas, alt, dem.elev_m(lat, lon) * B.M2FT, W)
    clock = {'t': 0.0}
    fd = B.Finder(f, dem, lambda: clock['t'])
    dt = f['simulation/dt']
    for i in range(int(T / dt)):
        clock['t'] = i * dt
        if abs(i * dt - 3.0) < dt / 2:
            for n in (0, 1): f['/controls/engines/engine[%d]/cutoff' % n] = 1
        if i % int(round(B.TICK_S / dt)) == 0:
            with contextlib.redirect_stdout(io.StringIO()): fd.update()
        f['position/terrain-elevation-asl-ft'] = dem.elev_m(f['position/lat-geod-deg'], f['position/long-gc-deg']) * B.M2FT
        f.run()
    return f, fd
f, fd = fly(4.0)
with contextlib.redirect_stdout(io.StringIO()):
    while fd.sampling or fd.heap or fd.rheap:
        fd.update()
la, lo = f['position/lat-geod-deg'], f['position/long-gc-deg']
trk = math.atan2(f['velocities/v-east-fps'], f['velocities/v-north-fps']) * B.R2D
gs = fd.glide_state()
ins = []
for c in fd.runways(6) + fd.fields:
    pl = fd.plans(la, lo, trk, c)[0]
    if pl[0] >= 0:
        c['plan'], c['margin'] = pl, pl[0]
        dN = (c['lat'] - la) * B.FT_DEG; dE = (c['lon'] - lo) * B.FT_DEG * math.cos(la * B.D2R)
        ins.append((pl[0], c, math.hypot(dN, dE) / 6076, math.degrees(math.atan2(dE, dN)) % 360, fd.path_clear(la, lo, trk, c, gs)))
# when the flight's own search found the same lines
f2, fd2 = fly(40.0)
found = {(round(c['lat'], 5), round(c['lon'], 5), c['crs']): c.get('found') for c in fd2.fields}
print('case %d at the cut: trk %03.0f, %d lines in reach' % (seed, trk % 360, len(ins)))
for m, c, d, b, cl in sorted(ins, key=lambda t: -t[0]):
    print('  %+6.0f  %-5s crs %03.0f  %.1f NM on %03.0f  clear %d  %-14s found in flight at %s' % (m, c['kind'], c['crs'], d, b, cl, fd.kname(c['plan']),
          found.get((round(c['lat'], 5), round(c['lon'], 5), c['crs']), 'never')))
