#!/usr/bin/env python3
"""The level-flight trim's longitudinal stick over the nacelle angle, the calibrated airspeed and the flaperons (held
at each of FLAPERON), at the design gross weight and CG, 1000 ft, inside the conversion corridor (v22data.CPS), for the
augmentation's trim feed-forward (genv22.scas): with it the stick's centre is trimmed level flight through the
conversion at whatever the flaperons are (their law's, or ENG OUT's in its flare), and the augmentation's integrators
only take up the weight's and the CG's differences. Outside the corridor, or where no trim converged, the nearest trim
along the airspeed, then along the nacelle angle, then the nearest flaperons'.

  python3 trimmap.py [--write]      (writes data/trim_ff_v22.json)

With --autonac: the level-flight trim's pitch attitude on a finer grid (5 deg of nacelle, 10 kt), from which the
autopilot's AUTO NAC takes the nacelle angle for an airspeed (genv22.autopilot): data/autonac_v22.json.

With --start: the level-flight trims along AUTO NAC's schedule over the calibrated airspeed, the gross weight (at the
design CG), the density altitude (standard day) and the gear (up, down), for FlightGear's in-air start (genv22.controls:
the lever, the collective, the stick and the pitch attitude the flight model starts with): data/start_trim_v22.json."""
import sys, json
from multiprocessing import Pool
import numpy as np
import harness as H
import v22data as V

NACELLE = [0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0]
FLAPERON = [0.0, 20.0, 40.0, 67.0]
KCAS = [0.0, 20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0, 160.0, 180.0, 200.0, 240.0, 280.0]
OUT = 'data/trim_ff_v22.json'


def inside(nac, kcas):
    (lv, ln), (uv, un) = V.CPS
    return np.interp(kcas, lv, ln) - 0.1 <= nac <= np.interp(kcas, uv, un) + 0.1


def column(nac, kcas_list=KCAS, what=1, flap=None):
    """the trims along the airspeed at one nacelle angle, each from the last: the stick (what 1), the pitch attitude
    (what 'theta') or the stick and the flaperons (what 'level'); flap: the flaperons held there, deg (else their
    law's)"""
    f = H.load()
    if flap is not None:
        f['ctl/flaperon-override'], f['ctl/flaperon-set-deg'] = 1, flap
    out, x0 = {}, None
    for kcas in kcas_list:
        if not inside(nac, kcas):
            continue
        x, r, e = H.trim(f, kcas, alt_ft=1000.0, nacelle=nac, cas=True, x0=x0 if x0 is not None else [2.0, 0, 0, 0, 2.0, 0])
        res = float(np.abs(r).max())
        if res >= 0.05 and x0 is not None:
            # (again from the default guess: from a converged neighbour far off, with the flaperons held at 67 deg and
            # the nacelles at 15, every faster trim failed)
            x, r, e = H.trim(f, kcas, alt_ft=1000.0, nacelle=nac, cas=True, x0=[2.0, 0, 0, 0, 2.0, 0])
            res = float(np.abs(r).max())
        if res < 0.05:
            out[kcas] = {1: float(x[1]), 'theta': float(f['attitude/theta-deg']), 'level': (float(x[1]), float(f['ctl/flaperon-deg']))}[what]
            x0 = x
        print('nacelle %4.1f %5.1f KCAS flaperons %4.1f  lon %6.3f  theta %6.2f  res %.3f' % (
            nac, kcas, f['ctl/flaperon-deg'], x[1], f['attitude/theta-deg'], res), flush=True)
    return nac, out


def flap_column(args):
    nac, flap = args
    return (flap, nac), column(nac, what='level' if flap is None else 1, flap=flap)[1]


def fill(t):
    """holes and the outside of the corridor (nan) in a [nacelle][KCAS] table: the nearest trim along the airspeed,
    then the nearest full row along the nacelle angle"""
    for i in range(len(t)):
        ok = np.where(~np.isnan(t[i]))[0]
        if len(ok):
            for j in range(len(t[i])):
                if np.isnan(t[i, j]):
                    t[i, j] = t[i, ok[np.argmin(np.abs(ok - j))]]
    full = [i for i in range(len(t)) if not np.isnan(t[i]).any()]
    for i in range(len(t)):
        if np.isnan(t[i]).any() and full:
            t[i] = t[min(full, key=lambda m: abs(m - i))]
    return t


def table():
    """the stick at each of FLAPERON, [flaperons][nacelle][KCAS] (where still none, the nearest flaperons'); and level
    flight at the flaperons' own law, its stick and flaperons, [nacelle][KCAS]"""
    with Pool() as pool:
        cols = dict(pool.map(flap_column, [(n, fl) for fl in FLAPERON + [None] for n in NACELLE]))

    def grid(fl, part=None):
        """one [nacelle][KCAS] table of the trims at flaperons fl (None: their law's, part 0 the stick, 1 the flaperons)"""
        at = lambda c, k: (c[k] if part is None else c[k][part]) if k in c else np.nan
        return fill(np.array([[at(cols[(fl, n)], k) for k in KCAS] for n in NACELLE], float))
    tab = np.array([grid(fl, None) for fl in FLAPERON])
    for l in range(len(FLAPERON)):
        if np.isnan(tab[l]).any():
            full = [m for m in range(len(FLAPERON)) if not np.isnan(tab[m]).any()]
            tab[l] = np.where(np.isnan(tab[l]), tab[min(full, key=lambda m: abs(m - l))], tab[l])
    return tab, grid(None, 0), grid(None, 1)


AN_NACELLE = [5.0 * i for i in range(19)]
AN_KCAS = [10.0 * i for i in range(23)]
AN_OUT = 'data/autonac_v22.json'


def autonac_column(nac):
    return column(nac, AN_KCAS, 'theta')


ST_KCAS = [10.0 * i for i in range(23)] + [240.0, 260.0, 280.0]
ST_WEIGHT = [V.W_EMPTY + 3 * V.W_CREW + (60500.0 - V.W_EMPTY - 3 * V.W_CREW) * i / 3 for i in range(4)]   # (empty and crew .. self-deployment)
ST_DALT = [0.0, 6000.0, 12000.0, 18000.0, 24000.0]
ST_OUT = 'data/start_trim_v22.json'


def start_weight(case):
    """the trims at one gear and weight: at each density altitude along the airspeed from the fastest down (airplane
    mode's trims converge from anywhere), each from the last converged one, retried from the one at its airspeed an
    altitude lower: from the hover up, a column whose first trim failed went on from the default guess and failed on
    to airplane mode. Each: the power required (both engines, hp), the collective (deg), the longitudinal stick
    (-1..1), the pitch attitude (deg), the residual (ft/s^2, rad/s^2)"""
    import performance as P
    from genv22 import autonac_schedule
    gear, gw = case
    sk, sn = autonac_schedule()
    f = H.load()
    P.set_weight(f, gw)
    f['gear/gear-cmd-norm'] = gear; f['gear/gear-pos-norm'] = gear
    out, below = {}, {}
    for alt in ST_DALT:
        col, xs, x0 = {}, {}, None
        for kcas in reversed(ST_KCAS):
            nac = float(np.interp(kcas, sk, sn))
            best = None
            for guess in [g for g in (x0, below.get(kcas)) if g is not None] or [[2.0, 0, 0, 0, 2.0, 0]]:
                x, r, e = H.trim(f, kcas, alt_ft=alt, nacelle=nac, cas=True, x0=guess, extra=('drive/power-required-hp',))
                res = float(np.abs(r).max())
                row = (float(e[0]), f['ctl/theta75-deg'], float(x[1]), f['attitude/theta-deg'], res)
                if best is None or res < best[0][4]:
                    best = (row, x)
                if res < 0.05:
                    break
            col[kcas] = best[0]
            if best[0][4] < 0.05:
                x0 = xs[kcas] = best[1]
            print('gear %d %6.0f lb %5.0f ft %5.1f KCAS: %5.0f hp, collective %5.2f, lon %6.3f, pitch %5.2f, res %.3f' % (
                (gear, gw, alt, kcas) + best[0]), flush=True)
        out[alt] = [col[k] for k in ST_KCAS]
        below = xs
    return case, out


def avail_isa(alt, nac):
    """the power available (both engines, hp) at a density altitude on a standard day, at the governed rotor speed
    (drive.py: each engine's rating lapse, and its share of the drive's torque limit)"""
    T = 518.67 - 0.00356616 * alt
    d = (T / 518.67) ** 5.2559
    nr = V.NR_APLN if nac < 1.0 else V.NR_HELO
    return 2 * min(V.P_MRP * min(1.0, d / np.sqrt(T / 518.67)), V.P_XMSN_100 * nr / 100 / 2)


def start_axes(acc, nac):
    """the body's accelerations (udot, wdot) along the rotors' thrust at a nacelle angle (+ more thrust: forward in
    airplane mode, up in the hover) and across it (+ forward in the hover, down in airplane mode), ft/s^2"""
    c, s_ = np.cos(np.radians(nac)), np.sin(np.radians(nac))
    return acc[0] * c - acc[2] * s_, acc[0] * s_ + acc[2] * c


def start_control_power():
    """along the schedule, from the trim at the design weight and CG, sea level, on the held airframe: the longitudinal
    stick's pitching moment and the lateral stick's rolling moment (ft lb per inch of the stick's travel at the
    controls, + nose up, right wing down: the acceleration's derivative times the inertia), the collective's acceleration along the rotors' thrust and the pitch
    attitude's across it (ft/s^2 per deg; start_axes). The in-air start trims the stick, the collective and the pitch
    with them while it holds the airframe (genv22.controls, scas)"""
    from genv22 import autonac_schedule
    from data import bell301 as B
    sk, sn = autonac_schedule()
    f = H.load()
    m_lon, l_lat, a_coll, a_pitch, x0 = [], [], [], [], None
    for kcas in ST_KCAS:
        nac = float(np.interp(kcas, sk, sn))
        x, r, e = H.trim(f, kcas, alt_ft=0.0, nacelle=nac, cas=True, x0=x0 if x0 is not None else [2.0, 0, 0, 0, 2.0, 0])
        x0 = x
        d = []
        for j, h in ((1, 0.02), (2, 0.02), (0, 0.5), (4, 0.5)):
            acc = []
            for sg in (-1, 1):
                xp = x.copy(); xp[j] += sg * h
                acc.append(H.residual(f, xp, kcas, 0.0, 0.0, nacelle=nac, cas=True)[0])
            d.append((acc[1] - acc[0]) / (2 * h))
        m_lon.append(float(d[0][4] * f['inertia/iyy-slugs_ft2'] / B.X_LON))
        l_lat.append(float(d[1][3] * f['inertia/ixx-slugs_ft2'] / B.X_LAT))
        a_coll.append(float(start_axes(d[2], nac)[0])); a_pitch.append(float(start_axes(d[3], nac)[1]))
        print('%5.1f KCAS: %+.0f, %+.0f ft lb per inch; %+.3f, %+.3f ft/s^2 per deg' % (kcas, m_lon[-1], l_lat[-1], a_coll[-1], a_pitch[-1]), flush=True)
    return m_lon, l_lat, a_coll, a_pitch


def start_fill(row, alt):
    """a column's points (power, collective, stick, pitch, residual along ST_KCAS) where no level trim converged, in
    place: between converged ones along the airspeed (the trim failing on its way through, its neighbours both level),
    the line between them; below the slowest converged one (slow, heavy and high: the rotors cannot hold the weight),
    the power available (the lever at its stop) and that trim's collective, stick and pitch. (The power available there
    at first: between converged trims it gave too much, the start at 10,000 ft and 40 kt climbing at 6 ft/s.) Returns
    how many"""
    from genv22 import autonac_schedule
    sk, sn = autonac_schedule()
    ok = [i for i, x in enumerate(row) if x[4] < 0.05]
    n = 0
    for i, x in enumerate(row):
        if x[4] < 0.05 or not ok:
            continue
        lo, hi = [j for j in ok if j < i], [j for j in ok if j > i]
        if lo and hi:
            a, b = row[lo[-1]], row[hi[0]]
            u = (ST_KCAS[i] - ST_KCAS[lo[-1]]) / (ST_KCAS[hi[0]] - ST_KCAS[lo[-1]])
            row[i] = tuple(p + (q - p) * u for p, q in zip(a[:4], b[:4])) + (x[4],)
        else:
            m = row[(lo or hi)[-1 if lo else 0]]
            row[i] = (avail_isa(alt, float(np.interp(ST_KCAS[i], sk, sn))), m[1], m[2], m[3], x[4])
        n += 1
    return n


if __name__ == '__main__' and '--start' in sys.argv:
    from genv22 import autonac_schedule
    sk, sn = autonac_schedule()
    with Pool() as pool:
        cols = dict(pool.map(start_weight, [(g, w) for g in (0, 1) for w in ST_WEIGHT]))
    fill = sum(start_fill(row, a) for c in cols.values() for a, row in c.items())
    print(fill, 'points without a level trim')
    tab = lambda k, g: [[[round(cols[(g, w)][a][i][k], 4) for i in range(len(ST_KCAS))] for a in ST_DALT] for w in ST_WEIGHT]
    json.dump({'kcas': ST_KCAS, 'weight_lb': ST_WEIGHT, 'density_alt_ft': ST_DALT, **dict(zip(('m_lon_ftlb_in', 'l_lat_ftlb_in', 'a_coll_fps2_deg', 'a_pitch_fps2_deg'), [[round(m, 3) for m in ms] for ms in start_control_power()])),
               **{'%s_gear_%s' % (n, gn): tab(k, g) for k, n in enumerate(('power_hp', 'coll_deg', 'lon', 'theta_deg', 'res'))
                  for g, gn in ((0, 'up'), (1, 'down'))},
               'note': 'level-flight trims along AUTO NAC\'s schedule, [weight][density altitude][KCAS]: the power required '
                       '(both engines, hp), the collective (deg), the longitudinal stick (-1..1, + forward), the pitch '
                       'attitude (deg), the trim\'s residual; the gear up and down; the design CG. Where no trim converged '
                       '(residual over 0.05): between converged ones along the airspeed the line between them; below the '
                       'slowest, the power available, standard day, and its trim\'s. Along KCAS at the design weight and CG, sea level: '
                       'm_lon_ftlb_in and l_lat_ftlb_in, the stick\'s pitching and rolling moments, ft lb per inch (+ nose up, right wing '
                       'down); a_coll_fps2_deg and a_pitch_fps2_deg, '
                       'the collective\'s acceleration along the rotors\' thrust and the pitch attitude\'s across it, ft/s^2 per deg '
                       '(trimmap.py --start)'},
              open(ST_OUT, 'w'), indent=1)
    print('wrote', ST_OUT)
elif __name__ == '__main__' and '--autonac' in sys.argv:
    with Pool() as pool:
        cols = dict(pool.map(autonac_column, AN_NACELLE))
    json.dump({'nacelle': AN_NACELLE, 'kcas': AN_KCAS,
               'theta': [[None if k not in cols[n] else round(cols[n][k], 3) for k in AN_KCAS] for n in AN_NACELLE],
               'note': 'level-flight trim pitch attitude, deg (null: no trim, outside the corridor), design weight and CG, '
                       '1000 ft (trimmap.py --autonac)'}, open(AN_OUT, 'w'), indent=1)
    print('wrote', AN_OUT)
elif __name__ == '__main__':
    tab, lev, lev_f = table()
    for name, layer in [('flaperons %.0f deg' % fl, t) for fl, t in zip(FLAPERON, tab)] + [('their law\'s', lev), ('their law\'s flaperons', lev_f)]:
        print(name)
        for n, row in zip(NACELLE, layer):
            print('%4.0f  %s' % (n, ' '.join('%6.3f' % a for a in row)))
    if '--write' in sys.argv:
        r2 = lambda t: [[round(a, 4) for a in row] for row in t]
        json.dump({'flaperon': FLAPERON, 'nacelle': NACELLE, 'kcas': KCAS, 'lon': [r2(t) for t in tab], 'lon_level': r2(lev),
                   'flaperon_level': r2(lev_f),
                   'note': 'level-flight trim longitudinal stick (-1..1, + forward), design weight and CG, 1000 ft (trimmap.py): '
                           'lon [flaperons][nacelle][KCAS] the flaperons held at each; lon_level [nacelle][KCAS] at their '
                           'law\'s, flaperon_level its flaperons, deg'},
                  open(OUT, 'w'), indent=1)
        print('wrote', OUT)
