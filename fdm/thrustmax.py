#!/usr/bin/env python3
"""The rotors' greatest thrust in the hover (alerts.py: ROTOR RPM), from the model: a hover trim at the reference
loading, 1000 ft out of ground effect, the airframe held and the rotor speed held; the collective raised in steps past
the trim's until the two rotors' thrust stops rising (the blades' stall), averaged over whole revolutions at each. The
thrust the hover needs over the weight (the wing's download) from the trim. And SINK RATE's K_VRS: the V-22's warning in
vertical descent, 800 ft/min [GL], over the vortex ring state's onset there (0.45 vh, Johnson's) in the model's vh at
the reference loading at sea level. And HOVER PWR's table (data/hover_power_v22.json): the power to hover out of ground
effect at the rotor speed it hovers at (104 %) against the weight and the density, from the model's trims
(performance.hover_power; the weights its trims cannot hold, above the ceiling, from the heaviest that did by momentum
theory's 1.5 power).

    python3 thrustmax.py            the greatest thrust, K_VRS
    python3 thrustmax.py table      the hover power table"""
import sys, math
import numpy as np
import harness as H

STEPS_DEG = np.arange(0.0, 14.01, 1.0)


def thrust(f, revs=4):
    w = f['rotor/omega-rad_sec']
    n = int(round(revs * 2 * math.pi / w / f['simulation/dt']))
    for _ in range(n):                                    # settle
        f.run()
    t = 0.0
    for _ in range(n):
        f.run()
        t += f['aero/rotor/r/thrust-lbs'] + f['aero/rotor/l/thrust-lbs']
    return t / n


ALTS = (0, 4000, 8400, 12000, 16000)
GWS = list(range(26000, 56001, 2000))


def _point(args):
    import performance as P
    gw, alt = args
    return gw, alt, P.hover_power(gw, alt, nr_trim=0.0)


def table():
    """the model's power to hover out of ground effect at the rotor speed it hovers at (104 %), ISA, over GWS x ALTS
    (performance.hover_power's trims); a weight its trim cannot hold (above the ceiling) from the heaviest that held,
    by momentum theory's 1.5 power"""
    import os, json
    from multiprocessing import Pool
    here = os.path.dirname(os.path.abspath(__file__))
    with Pool(os.cpu_count() or 4) as pool:
        res = {(gw, a): r for gw, a, r in pool.map(_point, [(gw, a) for a in ALTS for gw in GWS])}
    sigmas, rows = [], []
    for a in ALTS:
        sigmas.append((1 - 6.8756e-6 * a) ** 4.2559)
        ok = [(gw, res[gw, a][0]) for gw in GWS if res[gw, a][2] < 0.05]
        row = []
        for gw in GWS:
            if res[gw, a][2] < 0.05:
                row.append(res[gw, a][0])
            else:
                g0, q0 = max(x for x in ok if x[0] < gw)
                row.append(q0 * (gw / g0) ** 1.5)
        rows.append(row)
        print('%5d ft: %d of %d weights trimmed; available %.0f hp' % (a, len(ok), len(GWS), res[GWS[0], a][1]), flush=True)
    json.dump(dict(gw=GWS, sigma=sigmas, hp=rows, source='thrustmax.py table: performance.hover_power at 104 % rotor speed, ISA'),
              open(os.path.join(here, 'data', 'hover_power_v22.json'), 'w'), indent=1)
    print('data/hover_power_v22.json: %d weights x %d densities' % (len(GWS), len(sigmas)))


if __name__ == '__main__' and sys.argv[1:2] == ['table']:
    table()
elif __name__ == '__main__':
    f = H.load()
    H.trim(f, 0.0, alt_ft=0.0)
    vh = f['aero/rotor/r/vrs/vh-fps']
    print('sea level hover, %.0f lb: vh %.2f ft/s; the onset 0.45 vh %.0f ft/min; K_VRS for 800 ft/min %.3f'
          % (f['inertia/weight-lbs'], vh, 0.45 * vh * 60, 800 / (0.45 * vh * 60)))
    f = H.load()
    x, r, e = H.trim(f, 0.0)
    W, nr, rho = f['inertia/weight-lbs'], f['drive/nr-pct'], f['atmosphere/rho-slugs_ft3']
    t0 = thrust(f)
    print('hover trim: %.0f lb at %.1f %% rotor speed, density %.6f; thrust %.0f lb, %.4f of the weight; collective %.2f deg'
          % (W, nr, rho, t0, t0 / W, x[0]))
    best = (0.0, t0)
    for d in STEPS_DEG[1:]:
        f['gov/int-deg'] = x[0] + d
        t = thrust(f)
        print('  collective +%4.1f deg: thrust %.0f lb (%.3f of the trim\'s)' % (d, t, t / t0), flush=True)
        if t > best[1]:
            best = (d, t)
        elif t < best[1] * 0.995:
            break
    tmax = best[1]
    # at 100 % rotor speed and sea level, as the channel scales it: (Nr / 100)^2 and the density
    t100 = tmax / (nr / 100) ** 2 * (0.0023769 / rho)
    print('greatest thrust %.0f lb at +%.1f deg: %.3f of the hover\'s; %.0f lb at 100 %% rotor speed, sea level'
          % (tmax, best[0], tmax / t0, t100))
    print('the hover\'s thrust over the weight %.4f; the rotor speed it can no longer be held at, here: %.1f %%'
          % (t0 / W, nr * math.sqrt(t0 / tmax)))
