#!/usr/bin/env python3
"""V-22 performance against the published charts [data/v22_flight.py]: the maximum level speed (airplane mode, 84 %
rotor speed, the engines' power or the drive's torque limit, whichever is less) [RC fig. 33c], the hover ceiling out of
ground effect [RC fig. 33a], and through the conversion the airframe's share of the lift and the pitch attitude
[PD figs. 7, 9a]. Standard day.

  python3 performance.py [speed|ceiling|conversion ...]"""
import sys
from multiprocessing import Pool
import numpy as np
import harness as H
from data import v22_flight as VF
import v22data as V

ADJUST_PM = 6             # the test weight adjustment's point mass, at the design CG (genv22.point_masses)


def set_weight(f, gw):
    """the gross weight, by a weight at the design CG (negative to go lighter): the CG stays where it is"""
    f['inertia/pointmass-weight-lbs[%d]' % ADJUST_PM] = gw - V.GW_DESIGN


def available(f):
    """the engines' power available now: each engine's rating lapse and its share of the drive's torque limit"""
    return 2 * min(f['eng/power-available-hp'], f['eng/xmsn-limit-hp'])


def speed_power(gw, alt, speeds):
    f = H.load()
    set_weight(f, gw)
    out, x0 = [], None
    for V_ in speeds:
        x, r, e = H.trim(f, V_, alt_ft=alt, nacelle=0.0, x0=x0, extra=('drive/power-required-hp',))
        x0 = x
        out.append((V_, float(e[0]), available(f), float(np.abs(r).max())))
    return out


def max_speed(gw, alt):
    rows = speed_power(gw, alt, [240, 260, 280, 300, 320, 340])
    ex = [(v_, req - av) for v_, req, av, res in rows if res < 0.05]
    for (v0, d0), (v1, d1) in zip(ex, ex[1:]):
        if d0 <= 0 < d1:
            return v0 + (v1 - v0) * -d0 / (d1 - d0), rows
    return float('nan'), rows


def hover_power(gw, alt, nr_trim=-4.0):
    """at 100 % rotor speed (397 rpm), as the 1986 chart"""
    f = H.load()
    set_weight(f, gw)
    f['/controls/rotor/nr-trim-pct'] = nr_trim
    x, r, e = H.trim(f, 0.0, alt_ft=alt, extra=('drive/power-required-hp',))
    return float(e[0]), available(f), float(np.abs(r).max())


def hover_ceiling(alt):
    """the gross weight at which the power to hover equals the power available"""
    pts = [(gw,) + hover_power(gw, alt) for gw in range(26000, 54001, 2000)]
    d = [(gw, req - av) for gw, req, av, res in pts if res < 0.05]
    for (g0, d0), (g1, d1) in zip(d, d[1:]):
        if d0 <= 0 < d1:
            return g0 + (g1 - g0) * -d0 / (d1 - d0), pts
    return float('nan'), pts


def conversion(nacelle, gw=34000.0):
    f = H.load()
    set_weight(f, gw)
    out, x0 = [], None
    for kcas, test in VF.LIFT_SHARE[nacelle]:
        x, r, e = H.trim(f, kcas, alt_ft=0.0, nacelle=float(nacelle), x0=x0 if x0 is not None else [0.0, 0, 0, 0, 4.0, 0], cas=True,
                         extra=('aero/wing/lift-lbs', 'aero/airframe/FZ-lbs', 'drive/power-required-hp', 'inertia/weight-lbs'))
        x0 = x
        out.append(dict(V=kcas, wing_pct=100 * float(e[0]) / float(e[3]), pitch=f['attitude/theta-deg'], hp=float(e[2]),
                        test_lift=test, test_pitch=float(np.interp(kcas, *zip(*VF.PITCH[nacelle]))) if
                        VF.PITCH[nacelle][0][0] - 2 <= kcas <= VF.PITCH[nacelle][-1][0] + 2 else float('nan'),
                        res=float(np.abs(r).max())))
    return nacelle, out


def _ceiling(alt):
    return alt, hover_ceiling(alt)


if __name__ == '__main__':
    what = sys.argv[1:] or ['speed', 'ceiling', 'conversion']
    with Pool() as pool:
        if 'speed' in what:
            for (gw, alt), (vm, rows) in zip([(42000, 0), (47500, 0), (42000, 17000)],
                                             pool.starmap(max_speed, [(42000, 0), (47500, 0), (42000, 17000)])):
                print('max speed %5.0f lb %5.0f ft: %5.1f KTAS  (%s)' % (gw, alt, vm, ', '.join('%d kt %.0f/%.0f hp' % (a, b, c) for a, b, c, d in rows)))
            print('  test: %s (42 000 lb, KTAS, ft)' % VF.ENVELOPE_42K)
        if 'ceiling' in what:
            for alt, (gw, pts) in pool.map(_ceiling, [0, 4000, 8400, 12000, 16000]):
                print('hover ceiling %5.0f ft: %6.0f lb   (%s)' % (alt, gw, ', '.join('%d lb %.0f/%.0f hp' % (a, b, c) for a, b, c, d in pts)))
            print('  test: %s (lb, ft)' % VF.HOVER_CEILING)
        if 'conversion' in what:
            for nac, rows in pool.map(conversion, [90, 75, 60, 30, 0]):
                for r in rows:
                    print('nacelle %2d %4.0f KCAS  wing lift %5.1f %% (test airframe %5.1f)  pitch %5.1f (test %5.1f)  %6.0f hp  res %.3f' % (
                        nac, r['V'], r['wing_pct'], r['test_lift'], r['pitch'], r['test_pitch'], r['hp'], r['res']))
