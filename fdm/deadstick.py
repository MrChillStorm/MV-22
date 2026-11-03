#!/usr/bin/env python3
"""Both engines out, from the model: the glide with the rotors windmilling (the governor holding their speed with the
collective, genv22.governor; the trims' rotor speed held at the reference), and whether a landing on the wheels is
there to be made.

The glide: at each nacelle angle and calibrated airspeed (within the conversion corridor), the flight path angle at
which the drive needs no power (harness.trim at that angle; bisection on its power required): the glide angle, its
ratio and its descent rate, at the reference loading, 1000 ft. With --table, the airplane mode's glide ratio over the
calibrated airspeed and the gross weight, the gear up and down, for ENG OUT's reach (data/glide_ld_v22.json).

The landing: the proprotors clear the ground on the wheels only with the nacelles up far enough: the hub (on the shaft
DZ_HUB from the pivot, WL_PIV over the wheels' contact) less the rotor's radius times the cosine of the shaft's angle to
the ground, the nacelle angle plus the pitch attitude. A glide flown to the ground without a flare meets it at its
descent rate; the gear's limit, the military requirement's 12 ft/s free-fall equivalent [US 5,547,148].

    python3 deadstick.py [--table]"""
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


def power(nac, V, gamma, W=None, gear=0.0):
    f = loaded(W, gear)
    x, r, e = H.trim(f, V, alt_ft=1000.0, gamma=gamma, nacelle=nac, cas=True,
                     extra=('drive/power-required-hp', 'attitude/theta-deg'))
    return float(e[0]), float(e[1]), float(np.abs(r).max())


def glide(args):
    """the flight path angle (deg) of zero power at nac, V KCAS (W lb, the gear): bisection between level flight and
    -40 deg"""
    nac, V, W, gear = (tuple(args) + (None, 0.0))[:4]
    lo, hi = -40.0, 0.0                       # power(lo) < 0 < power(hi)
    p_hi, th, res = power(nac, V, hi, W, gear)
    if res > 0.05 or p_hi <= 0:
        return nac, V, None
    for _ in range(9):
        mid = 0.5 * (lo + hi)
        p, th, res = power(nac, V, mid, W, gear)
        if res > 0.05:
            lo = mid
            continue
        if p > 0:
            hi = mid
        else:
            lo = mid
    g = 0.5 * (lo + hi)
    p, th, res = power(nac, V, g, W, gear)
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


if __name__ == '__main__':
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
