#!/usr/bin/env python3
"""Both engines out, from the model: the glide with the rotors windmilling (the governor holding their speed with the
collective, genv22.governor; the trims' rotor speed held at the reference), and whether a landing on the wheels is
there to be made.

The glide: at each nacelle angle and calibrated airspeed (within the conversion corridor), the flight path angle at
which the drive needs no power (harness.trim at that angle; bisection on its power required): the glide angle, its
ratio and its descent rate, at the reference loading, 1000 ft. With --table, the airplane mode's glide ratio over the
calibrated airspeed and the gross weight, the gear up and down, for ENG OUT's reach (data/glide_ld_v22.json); with
--flaptable, at the glide's 200 KCAS against the flaperons held (0-40 deg) too, for ENG OUT's spending of a surplus
(data/glide_ld_flap_v22.json).

The landing: the proprotors clear the ground on the wheels only with the nacelles up far enough: the hub (on the shaft
DZ_HUB from the pivot, WL_PIV over the wheels' contact) less the rotor's radius times the cosine of the shaft's angle to
the ground, the nacelle angle plus the pitch attitude. A glide flown to the ground without a flare meets it at its
descent rate; the gear's limit, the military requirement's 12 ft/s free-fall equivalent [US 5,547,148].

    python3 deadstick.py [--table | --flaptable]"""
import math, sys, os, json
from multiprocessing import Pool
import numpy as np
import harness as H
import v22data as U

GEAR_FPS = 12.0
CASES = {0.0: (140, 160, 180, 200, 220), 30.0: (110, 130, 150, 170), 45.0: (90, 110, 130, 150), 60.0: (75, 90, 110, 130),
         75.0: (50, 70, 90, 110)}


def clearance(nac, pitch=0.0):
    """the lower blade tip's height over the ground on the wheels, ft"""
    a = math.radians(nac + pitch)
    return (U.WL_PIV - U.MAIN_GEAR[2] + U.DZ_HUB * math.sin(a)) / 12 - U.R * math.cos(a)


# the glide ratio's table (--table): airplane mode, 1000 ft, KCAS by gross weight (cargo, then fuel under the reference
# fill), the gear up and down
TABLE_KCAS = (160, 180, 200, 220, 240)
TABLE_W = (36000, 42000, 47500, 54000, 60500)


def loaded(W=None, gear=0.0):
    """harness.load at the reference loading, or at W lb (weigh); the gear up (0) or down (1); ENG OUT's mode on, as
    with both engines out (its flaperons: genv22.flap_law)"""
    f = weigh(H.load(), W)
    f['gear/gear-cmd-norm'] = gear
    f['gear/gear-pos-norm'] = gear
    f['/instrumentation/mv22b/afcs/eng-out'] = 1
    return f


def weigh(f, W):
    """the loading f carries changed to W lb: the cargo, below none of it the tanks' reference fill scaled"""
    if W is not None:
        base = U.W_EMPTY + 3 * U.W_CREW
        fuel = U.FUEL_LB_GAL * sum(t[1] * r for t, r in zip(U.TANKS, U.TANKS_REF))
        f['inertia/pointmass-weight-lbs[%d]' % U.CARGO_PM] = max(0.0, W - base - fuel)
        k = min(1.0, (W - base) / fuel)
        for i, (t, r) in enumerate(zip(U.TANKS, U.TANKS_REF)):
            f['propulsion/tank[%d]/contents-lbs' % i] = t[1] * U.FUEL_LB_GAL * r * k
    return f


def power(nac, V, gamma, W=None, gear=0.0, flap=None):
    f = loaded(W, gear)
    if flap is not None:
        f['ctl/flaperon-override'] = 1; f['ctl/flaperon-set-deg'] = flap
    x, r, e = H.trim(f, V, alt_ft=1000.0, gamma=gamma, nacelle=nac, cas=True,
                     extra=('drive/power-required-hp', 'attitude/theta-deg'))
    return float(e[0]), float(e[1]), float(np.abs(r).max())


def glide(args):
    """the flight path angle (deg) of zero power at nac, V KCAS (W lb, the gear): bisection between level flight and
    -40 deg"""
    nac, V, W, gear, flap = (tuple(args) + (None, 0.0, None))[:5]
    lo, hi = -40.0, 0.0                       # power(lo) < 0 < power(hi)
    p_hi, th, res = power(nac, V, hi, W, gear, flap)
    if res > 0.05 or p_hi <= 0:
        return nac, V, None
    for _ in range(9):
        mid = 0.5 * (lo + hi)
        p, th, res = power(nac, V, mid, W, gear, flap)
        if res > 0.05:
            lo = mid
            continue
        if p > 0:
            hi = mid
        else:
            lo = mid
    g = 0.5 * (lo + hi)
    p, th, res = power(nac, V, g, W, gear, flap)
    return nac, V, (g, th, res)


def table():
    """the airplane mode's glide ratio (1 / tan of the zero-power path) at TABLE_KCAS by TABLE_W, the gear up and down"""
    jobs = [(0.0, V, W, gear) for gear in (0.0, 1.0) for W in TABLE_W for V in TABLE_KCAS]
    with Pool(min(8, len(jobs))) as pool:
        res = pool.map(glide, jobs)
    out = {'kcas': list(TABLE_KCAS), 'weight_lb': list(TABLE_W), 'alt_ft': 1000.0, 'nacelle_deg': 0.0}
    it = iter(res)
    for gear in ('up', 'down'):
        rows = []
        for W in TABLE_W:
            row = []
            for V in TABLE_KCAS:
                _, _, r = next(it)
                if r is None or r[2] > 0.02:
                    sys.exit('no glide trim at %g KCAS, %g lb, gear %s' % (V, W, gear))
                row.append(round(1 / math.tan(-math.radians(r[0])), 3))
            rows.append(row)
            print('gear %-4s %6.0f lb: %s' % (gear, W, ' '.join('%5.2f' % x for x in row)), flush=True)
        out['ld_gear_' + gear] = rows
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'glide_ld_v22.json')
    json.dump(out, open(path, 'w'), indent=1)
    print('wrote', path)


FLAP_DEG = (0.0, 10.0, 20.0, 30.0, 40.0)


def flap_table():
    """the airplane mode's glide ratio at 200 KCAS, the glide's speed, by TABLE_W and the flaperons held at FLAP_DEG,
    the gear up and down"""
    jobs = [(0.0, 200.0, W, gear, fl) for gear in (0.0, 1.0) for W in TABLE_W for fl in FLAP_DEG]
    with Pool(min(8, len(jobs))) as pool:
        res = pool.map(glide, jobs)
    out = {'kcas': 200.0, 'weight_lb': list(TABLE_W), 'flap_deg': list(FLAP_DEG), 'alt_ft': 1000.0, 'nacelle_deg': 0.0}
    it = iter(res)
    for gear in ('up', 'down'):
        rows = []
        for W in TABLE_W:
            row = []
            for fl in FLAP_DEG:
                _, _, r = next(it)
                if r is None or r[2] > 0.02:
                    sys.exit('no glide trim at %g lb, flaperons %g, gear %s' % (W, fl, gear))
                row.append(round(1 / math.tan(-math.radians(r[0])), 3))
            rows.append(row)
            print('gear %-4s %6.0f lb: %s' % (gear, W, ' '.join('%5.2f' % x for x in row)), flush=True)
        out['ld_gear_' + gear] = rows
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'glide_ld_flap_v22.json')
    json.dump(out, open(path, 'w'), indent=1)
    print('wrote', path)


# ENG OUT's capability (--captable): the glide's ratio at zero power by the gross weight, the flaperons, the nacelles, the calibrated
# airspeed and the gear, within the conversion corridor. One file; the generator writes it into the flight model and the finder
# alike, so that what the guidance flies and what the planner reckons with cannot differ, and neither is a number from one loading
CAP_W = (36000, 42000, 47500, 54000, 60500)
CAP_FLAP = (0.0, 20.0, 40.0)
CAP_NAC = (0.0, 15.0, 30.0, 45.0, 60.0, 75.0)
CAP_KCAS = (110, 130, 150, 170, 190, 210, 230)   # (under it no steady glide: only the way into one, deadland.py capentry, as flown)


def in_corridor(nac, V):
    """within the conversion corridor's two boundaries at V"""
    (lv, ln), (uv, un) = U.CPS
    return float(np.interp(V, lv, ln)) - 0.5 <= nac <= float(np.interp(V, uv, un)) + 0.5


def glide_fast(args):
    """glide's path angle at zero power by the secant: power against the path is nearly a line, 3-4 trims for bisection's 10"""
    nac, V, W, gear, flap = args
    a, b = -8.0, -20.0
    pa, th, ra = power(nac, V, a, W, gear, flap)
    pb, th, rb = power(nac, V, b, W, gear, flap)
    for _ in range(6):
        if abs(pb - pa) < 1e-6:
            break
        c = b - pb * (b - a) / (pb - pa)
        c = max(-50.0, min(-0.5, c))
        pc, th, rc = power(nac, V, c, W, gear, flap)
        a, pa, b, pb, rb = b, pb, c, pc, rc
        if abs(b - a) < 0.05:
            break
    return b, th, rb


def cap_one(j):
    try:
        r = glide_fast(j)
        if r[2] > 0.03 or r[0] < -49:            # (no trim on the way: bisection, which steps round those)
            r = glide(j)[2]
        return j, r
    except Exception:
        return j, None


def cap_jobs():
    """the most used first, so that a part of the table is of use: the brake's flaperons, the gear up, at every weight; the gear
    down; then the rest. Under 130 KCAS the flaperons are 40 whatever is asked (flap_law), so 40 alone"""
    out = []
    for gear, fl in ((0.0, 40.0), (1.0, 40.0), (0.0, 20.0), (0.0, 0.0), (1.0, 20.0), (1.0, 0.0)):
        for W in CAP_W:
            for V in CAP_KCAS:
                if fl < 40 and V < 130:
                    continue
                out += [(nac, V, W, gear, fl) for nac in CAP_NAC if in_corridor(nac, V)]
    return out


def cap_write(rows, path):
    get = lambda W, g, fl, V, nac, i: (rows.get((nac, V, W, g, fl)) or (None, None))[i]
    out = {'weight_lb': list(CAP_W), 'flap_deg': list(CAP_FLAP), 'kcas': list(CAP_KCAS), 'nacelle_deg': list(CAP_NAC), 'alt_ft': 1000.0,
           'axes': 'weight, gear (up, down), flaperons, kcas, nacelles; null: outside the corridor, no trim or not yet measured'}
    for name, i in (('ld', 0), ('theta_deg', 1)):
        out[name] = [[[[[get(W, g, fl, V, n, i) for n in CAP_NAC] for V in CAP_KCAS] for fl in CAP_FLAP] for g in (0.0, 1.0)] for W in CAP_W]
    json.dump(out, open(path, 'w'), indent=1)


def cap_table():
    """resumable: what is in the file is kept, written back every 8 trims"""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'eo_capability_v22.json')
    rows = {}
    if os.path.exists(path):
        d = json.load(open(path))
        for a, W in enumerate(d['weight_lb']):
            for b, g in enumerate((0.0, 1.0)):
                for c_, fl in enumerate(d['flap_deg']):
                    for e, V in enumerate(d['kcas']):
                        for h, nac in enumerate(d['nacelle_deg']):
                            if d['ld'][a][b][c_][e][h] is not None:
                                rows[(nac, V, W, g, fl)] = (d['ld'][a][b][c_][e][h], d['theta_deg'][a][b][c_][e][h])
    jobs = [j for j in cap_jobs() if j not in rows]
    print(len(jobs), 'trims to go', flush=True)
    with Pool(8, maxtasksperchild=4) as pool:
        for n, (j, r) in enumerate(pool.imap_unordered(cap_one, jobs)):
            ok = r is not None and r[2] <= 0.03 and r[0] < -0.5
            rows[j] = (round(1 / math.tan(-math.radians(r[0])), 3), round(r[1], 1)) if ok else None
            print('%3d/%d %6.0f lb gear %d flap %2.0f %3.0f KCAS nac %2.0f: %s' % (n + 1, len(jobs), j[2], j[3], j[4], j[1], j[0], ('L/D %.2f theta %.1f res %.3f' % (rows[j][0], rows[j][1], r[2])) if ok else 'no trim'), flush=True)
            if n % 8 == 7:
                cap_write(rows, path)
    cap_write(rows, path)
    print('wrote', path)


# The nacelle brake's grid (--brakegrid, data/eo_brake_grid_v22.json): the same zero-power glides with nothing held to the corridor,
# every weight the landings are tested at, and what its law has to watch besides the ratio and the pitch: the stick that is left, the
# collective, the wing's angle, the rotor's flapping and speed (docs/FLIGHT-LAWS.md 5.5). Row by row along the nacelles, each trim
# from the last: up a row the trims stop existing, so two misses end it
BRAKE_W = (36000, 38722, 42000, 47500, 54000, 60500)
BRAKE_NAC = (0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0)
BRAKE_EXTRA = ('drive/power-required-hp', 'attitude/theta-deg', 'aero/alpha-deg', 'alert/flap-deg', 'drive/nr-pct')
BRAKE_COLS = ('ld', 'theta_deg', 'alpha_deg', 'stick', 'coll_deg', 'rotor_flap_deg', 'nr_pct', 'res')


def brake_cell(f, nac, V, x0, jac):
    """the path of zero power at nac, V on the loaded f, by the secant; (columns, x) or None. Most of a row's time went on the search going
    on through cells with no trim in them, so it stops when it can't get a trim near where it's looking; but one that fails is no proof
    on its own (at 54 000 lb and 170 KCAS -20 deg has none and -8 does), so each is tried once closer in first"""
    at = lambda g, x: H.trim(f, V, alt_ft=1000.0, gamma=g, nacelle=nac, cas=True, extra=BRAKE_EXTRA, x0=x, jac=jac, iters=12)
    good = lambda r: float(np.abs(r).max()) <= 0.03
    a = -8.0
    xa, ra, ea = at(a, x0)
    if not good(ra):
        a = -16.0
        jac.clear()
        xa, ra, ea = at(a, x0)
        if not good(ra):
            return None
    b = a - 4.0
    xb, rb, eb = at(b, xa)
    if not good(rb):
        b = a + 3.0
        jac.clear()
        xb, rb, eb = at(b, xa)
        if not good(rb):
            return None
    for _ in range(7):
        if abs(eb[0] - ea[0]) < 1e-6:
            break
        c = max(-50.0, min(-0.5, b - eb[0] * (b - a) / (eb[0] - ea[0])))
        xc, rc, ec = at(c, xb)
        if not good(rc):
            c = 0.5 * (b + c)
            jac.clear()
            xc, rc, ec = at(c, xb)
            if not good(rc):
                return None
        a, ea, b, xb, rb, eb = b, eb, c, xc, rc, ec
        if abs(b - a) < 0.05:
            break
    res = float(np.abs(rb).max())
    if res > 0.03 or not -49 < b < -0.5:
        return None
    return dict(ld=round(1 / math.tan(-math.radians(b)), 3), theta_deg=round(float(eb[1]), 1), alpha_deg=round(float(eb[2]), 1), stick=round(float(xb[1]), 3),
                coll_deg=round(float(xb[0]), 2), rotor_flap_deg=round(float(eb[3]), 2), nr_pct=round(float(eb[4]), 1), res=round(res, 3)), xb


def brake_row(j):
    """one row along the nacelles at W, gear, flaperons and KCAS"""
    W, gear, fl, V = j
    out, x, miss, jac = {}, None, 0, {}
    try:
        f = loaded(W, gear)
        f['ctl/flaperon-override'] = 1; f['ctl/flaperon-set-deg'] = fl
        for nac in BRAKE_NAC:
            r = brake_cell(f, nac, V, x, jac)
            if r is None and x is not None:
                jac.clear()
                r = brake_cell(f, nac, V, None, jac)
            if r is None:
                miss += 1
                if miss == 2 and out:       # (slow and heavy the low nacelles have no trim and the high ones do)
                    break
                continue
            miss = 0
            out[nac], x = r
    except Exception as e:
        print('row', j, repr(e)[:100], flush=True)
    return j, out


def brake_jobs():
    """the brake's own settings first, so that a part of the grid is of use: the speeds it flies, the flaperons out, the gear up"""
    out = []
    for gear, fl in ((0.0, 20.0), (0.0, 40.0), (1.0, 40.0), (0.0, 0.0), (1.0, 20.0), (1.0, 0.0)):
        for V in (150, 170, 130, 190, 110, 210, 230):
            if fl < 40 and V < 130:
                continue
            out += [(W, gear, fl, V) for W in BRAKE_W]
    return out


def brake_write(rows, path):
    cell = lambda W, g, fl, V, n, k: (rows.get((W, g, fl, V)) or {}).get(n, {}).get(k)
    out = {'weight_lb': list(BRAKE_W), 'flap_deg': list(CAP_FLAP), 'kcas': list(CAP_KCAS), 'nacelle_deg': list(BRAKE_NAC), 'alt_ft': 1000.0,
           'axes': 'weight, gear (up, down), flaperons, kcas, nacelles; null: no trim, or past the end of its row. stick: +1 full forward',
           'rows_done': len(rows)}
    for k in BRAKE_COLS:
        out[k] = [[[[[cell(W, g, fl, V, n, k) for n in BRAKE_NAC] for V in CAP_KCAS] for fl in CAP_FLAP] for g in (0.0, 1.0)] for W in BRAKE_W]
    json.dump(out, open(path, 'w'), indent=1)


def brake_grid(n=4):
    """resumable: the rows in the file are kept, the file written after every row"""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'eo_brake_grid_v22.json')
    rows = {}
    if os.path.exists(path):
        d = json.load(open(path))
        for a, W in enumerate(d['weight_lb']):
            for b, g in enumerate((0.0, 1.0)):
                for c_, fl in enumerate(d['flap_deg']):
                    for e, V in enumerate(d['kcas']):
                        cells = {nac: {k: d[k][a][b][c_][e][h] for k in BRAKE_COLS} for h, nac in enumerate(d['nacelle_deg']) if d['ld'][a][b][c_][e][h] is not None}
                        if cells:
                            rows[(W, g, fl, V)] = cells
    jobs = [j for j in brake_jobs() if j not in rows]
    print(len(jobs), 'rows to go', flush=True)
    with Pool(n, maxtasksperchild=2) as pool:
        for i, (j, r) in enumerate(pool.imap_unordered(brake_row, jobs)):
            rows[j] = r
            print('%3d/%d %6.0f lb gear %d flap %2.0f %3.0f KCAS: %s' % (i + 1, len(jobs), j[0], j[1], j[2], j[3], ' '.join('%g:%.2f' % (k, v['ld']) for k, v in r.items()) or 'no trim'), flush=True)
            brake_write(rows, path)
    print('wrote', path)


# The crosswind the pedals can take out (--xwind, data/eo_crosswind_v22.json): at the speed intact landings touch down at, as the flare ends (nacelles 45,
# flaperons 40, the gear down), the sideslip full pedal holds, and the bank it takes: the crosswind whose crab that cancels, V sin(beta)
XW_W = (36000, 38722, 42000, 47500, 54000, 60500)
XW_BETA = (1.5, 3.0, 4.5, 6.0, 7.5)


def xw_one(W):
    sc = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'eo_scatter_v22.json')))
    V = float(np.interp(W, sc['weight_lb'], sc['td_kcas']))
    f = loaded(W, 1.0)
    f['ctl/flaperon-override'] = 1; f['ctl/flaperon-set-deg'] = 40.0
    pts, x, jac = [], None, {}
    for b in XW_BETA:
        x_, r, e = H.trim(f, V, alt_ft=1000.0, gamma=-2.0, nacelle=45.0, cas=True, extra=('attitude/phi-deg',), x0=x, beta=b, jac=jac, iters=16)
        if float(np.abs(r).max()) > 0.05:
            jac.clear()
            x_, r, e = H.trim(f, V, alt_ft=1000.0, gamma=-2.0, nacelle=45.0, cas=True, extra=('attitude/phi-deg',), x0=x, beta=b)
            if float(np.abs(r).max()) > 0.05:
                break
        x = x_
        pts.append((b, float(x_[3]), float(e[0])))
    if len(pts) < 2:
        return W, V, None, None, pts
    bs, ps, bk = [q[0] for q in pts], [abs(q[1]) for q in pts], [q[2] for q in pts]
    k = np.polyfit(ps, bs, 1)
    beta = float(min(np.polyval(k, 1.0), 12.0))
    return W, V, beta, float(np.interp(beta, bs, bk)), pts


def xw_table():
    with Pool(4) as pool:
        rows = pool.map(xw_one, XW_W)
    out = dict(weight_lb=[], kcas=[], beta_full_deg=[], bank_deg=[], limit_kt=[], note='full pedal at the touchdown speed, nacelles 45, flaperons 40, gear down; limit V sin(beta)')
    for W, V, beta, bank, pts in rows:
        print(W, 'lb', round(V), 'KCAS: pedal, bank by sideslip', [(q[0], round(q[1], 2), round(q[2], 1)) for q in pts], 'full pedal', beta, 'bank', bank, flush=True)
        if beta is not None:
            out['weight_lb'].append(W); out['kcas'].append(round(V)); out['beta_full_deg'].append(round(beta, 1)); out['bank_deg'].append(round(bank, 1))
            out['limit_kt'].append(round(V * math.sin(math.radians(beta)), 1))
    json.dump(out, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'eo_crosswind_v22.json'), 'w'), indent=1)
    print(out)


if __name__ == '__main__':
    if '--xwind' in sys.argv:
        xw_table()
        sys.exit()
    if '--brakegrid' in sys.argv:
        brake_grid()
    elif '--captable' in sys.argv:
        cap_table()
        sys.exit()
    if '--flaptable' in sys.argv:
        flap_table()
        sys.exit()
    if '--table' in sys.argv:
        table()
        sys.exit()
    print('the lower blade tip over the ground on the wheels, level: %s' % ', '.join(
        '%g deg %.1f ft' % (n, clearance(n)) for n in (0, 30, 38, 40, 45, 60, 75, 90)))
    n0 = next(n / 10 for n in range(0, 900) if clearance(n / 10) > 0)
    print('clear from %.1f deg of nacelle (level); %.1f deg with 1 ft to spare' % (n0, next(n / 10 for n in range(0, 900) if clearance(n / 10) > 1)))
    jobs = [(n, V) for n, vs in CASES.items() for V in vs]
    with Pool(len(jobs)) as pool:
        res = pool.map(glide, jobs)
    print('%7s %6s %8s %8s %8s %9s %10s' % ('nacelle', 'KCAS', 'path deg', 'ratio', 'pitch', 'sink ft/s', 'clearance'))
    for nac, V, r in res:
        if r is None:
            print('%7.0f %6.0f   no trim' % (nac, V))
            continue
        g, th, res_ = r
        sink = V * 1.6878 * math.sin(-math.radians(g))
        print('%7.0f %6.0f %8.2f %8.2f %8.1f %9.1f %8.1f ft%s' % (nac, V, g, 1 / math.tan(-math.radians(g)), th, sink,
              clearance(nac, th), '' if res_ < 0.02 else '  (residual %.3f)' % res_), flush=True)
