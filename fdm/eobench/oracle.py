"""the bench's finder with the whole search done at once, a moment after the cut: what it could choose from there"""
import sys, math, io, contextlib
sys.path.insert(0, '.')
import eobench as B
lat, lon, hdg, kcas, alt = 47.26023, 11.34397, 139.0, 205.0, 6015.0
if len(sys.argv) >= 6:
    lat, lon, hdg, kcas, alt = (float(x) for x in sys.argv[1:6])
T_EVAL = float(sys.argv[6]) if len(sys.argv) > 6 else 3.6
dem = B.Dem()
f = B.start(lat, lon, hdg, kcas, alt, dem.elev_m(lat, lon) * B.M2FT, 38722.0)
f[B.AF + 'cpld'] = 1
f['/autopilot/settings/heading-bug-deg'] = hdg
for m in ('hdg-hold', 'alt-hold', 'spd-hold'):
    f[B.AF + m] = 1
dt = f['simulation/dt']; clock = {'t': 0.0}
fd = B.Finder(f, dem, lambda: clock['t'])
for i in range(int(T_EVAL / dt)):
    t = i * dt; clock['t'] = t
    f['position/terrain-elevation-asl-ft'] = dem.elev_m(f['position/lat-geod-deg'], f['position/long-gc-deg']) * B.M2FT
    if abs(t - 3.0) < dt / 2:
        for n in (0, 1): f['/controls/engines/engine[%d]/cutoff' % n] = 1
    f.run()
print('at %.1f s: eng-out %d, %.0f ft over %.0f, %.0f KCAS, heading %.0f, reach %.2f NM' % (T_EVAL, f[B.AF + 'eng-out'], f['hh/h-wheels-ft'],
      f['position/terrain-elevation-asl-ft'], f['velocities/vc-kts'], f['attitude/psi-deg'] % 360, f['ap/eo-reach-nm']))
B.MAX_FIELDS = 10000
with contextlib.redirect_stdout(io.StringIO()):
    fd.update()                                   # (on: the origin here)
    while fd.sampling or fd.ti < len(fd.tries):
        fd.update()
    fd.chosen = None
    fd.rank()
fd.chosen = None
print('%d fields; the best 25 by the finder\'s order:' % len(fd.fields))
for c in fd.cands[:25]:
    print('  %-5s %-10s on %03d at %.4f %.4f: margin %+7.0f ft, path %.2f NM, clear %d' % (c['kind'], c['name'], c['crs'], c['lat'], c['lon'],
          c['margin'], c['path'] / 6076.12, c['clear']))
ok = [c for c in fd.cands if c['margin'] >= 0]
print('within reach: %d, of them clear: %d' % (len(ok), sum(c['clear'] for c in ok)))
# where the check fails, for the first within reach
c = next(c for c in fd.cands if c['margin'] >= 0)
gs = fd.glide_state()
la0, lo0 = f['position/lat-geod-deg'], f['position/long-gc-deg']
trk = math.atan2(f['velocities/v-east-fps'], f['velocities/v-north-fps']) * B.R2D
pts = fd.path_pts(la0, lo0, trk, c, gs['r0'])
S, A = pts[-1][2], pts[-1][3]
ce = c['elev']
print('site elev %.0f, glide h0 %.0f, ld %.2f, h per rad %.0f, r0 %.0f; path %.0f ft, turning %.0f deg, plan %s' % (ce, gs['h0'], gs['ld'], gs['hpr'], gs['r0'], S, A * B.R2D,
      [round(x, 2) for x in c['plan']]))
for q in pts[::4] + [pts[-1]]:
    e = fd.elev_at(q[0], q[1])
    hf = gs['h0'] - q[2] / gs['ld'] - q[3] * gs['hpr']
    hb = ce + gs['fl'] + (S - q[2]) / gs['ld'] + (A - q[3]) * gs['hpr']
    need = gs['fl'] * min(1, (S - q[2]) / max(1, c['plan'][6]))
    print('  %.4f %.4f way %6.0f turn %4.0f: ground %5.0f, glide %5.0f, from the site %5.0f -> over %5.0f (needs %3.0f)%s' % (
        q[0], q[1], q[2], q[3] * B.R2D, e if e else -1, hf, hb, min(hf, hb) - (e or 0), need, '  <-- BLOCKED' if e and min(hf, hb) - e < need else ''))
