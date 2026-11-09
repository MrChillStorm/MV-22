"""a case's site at the cut with the final joined nearer the point: join.py seed site-name t"""
import sys, io, contextlib, math
sys.path.insert(0, '.')
import batch, eobench as B
from dub_six import six
seed, name, tw = int(sys.argv[1]), sys.argv[2], float(sys.argv[3])
s, lat, lon, hdg, kcas, alt, W, dist, brg = batch.case(seed)
dem = B.Dem()
with contextlib.redirect_stdout(io.StringIO()):
    f = B.start(lat, lon, hdg, kcas, alt, dem.elev_m(lat, lon) * B.M2FT, W)
fd = B.Finder(f, dem, lambda: 0.0)
dt = f['simulation/dt']
for i in range(int(tw / dt)):
    if abs(i * dt - 3.0) < dt / 2:
        for n in (0, 1): f['/controls/engines/engine[%d]/cutoff' % n] = 1
    f.run()
la, lo = f['position/lat-geod-deg'], f['position/long-gc-deg']
trk = math.atan2(f['velocities/v-east-fps'], f['velocities/v-north-fps']) * B.R2D
c = [r for r in fd.runways(6) if r['name'] == name][0]
pl = fd.plan(la, lo, trk, c)
R0, R, Lf, kt = f['ap/eo-site-r0-ft'], f['ap/eo-site-r-ft'], f['ap/eo-site-final-ft'], f['ap/eo-site-turn-ft-rad']
reach = pl[0] + (pl[7][0] + Lf)          # (still air: the reach the plan counts)
cc = c['crs'] * B.D2R
dN = (la - c['lat']) * B.FT_DEG; dE = (lo - c['lon']) * B.FT_DEG * math.cos(la * B.D2R)
x, y = dE * math.sin(cc) + dN * math.cos(cc), dE * math.cos(cc) - dN * math.sin(cc)
psi = math.degrees(B.wrap(trk * B.D2R - cc))
print('%s at %.0f s: x %+.0f y %+.0f psi %+.0f, reach %.0f ft, R0 %.0f R %.0f, the final %.0f' % (name, tw, x, y, psi, reach, R0, R, Lf))
for Lj in (Lf, 4000, 3000, 2000, 1000, 0):
    cst, n, s0 = six(x, y, math.radians(psi), R0, R, Lj, kt)[0]
    print('  joined %5.0f ft before the point: %-6s margin %+7.0f' % (Lj, n, reach - cst - Lj))
