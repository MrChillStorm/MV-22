#!/usr/bin/env python3
"""The isolated rotor in hover against the full-scale XV-15 rotor's test [data/xv15_hover.py, NASA TM-86833]: the
right rotor of the XV-15 build, out of ground effect, at the test's tip Mach number (589 rpm at 1000 ft: 0.69), swept
in collective; the left rotor is its mirror image.

  python3 hover_rotor.py          prints CT/sigma, CP/sigma, figure of merit, model and test"""
import math
import numpy as np
import harness as H
from data import xv15_hover as XH
import xv15data as X


def sweep(colls=None):
    f = H.load(xv15=True)
    f['ctl/init'] = 1
    H.freeze(f)
    rows = []
    for c in colls or np.arange(-8.0, 13.1, 1.0):
        acc, ex = H.residual(f, [c, 0, 0, 0, 0, 0], 0.0, settle_s=1.5, revs=3,
                             extra=('aero/rotor/r/thrust-lbs', 'aero/rotor/r/power-hp', 'ctl/r/theta75-deg', 'aero/rotor/r/a1-deg', 'aero/rotor/r/b1-deg'))
        T, P, th, a1, b1 = ex
        rho, W = f['atmosphere/rho-slugs_ft3'], f['rotor/omega-rad_sec']
        A = math.pi * X.R ** 2
        ct = T / (rho * A * (W * X.R) ** 2); cp = P * 550 / (rho * A * (W * X.R) ** 3)
        rows.append(dict(th75=float(th), ct_s=ct / XH.SIGMA, cp_s=cp / XH.SIGMA, fm=ct ** 1.5 / math.sqrt(2) / cp if cp > 0 else 0.0,
                         tilt=float(math.hypot(a1, b1)), mtip=W * X.R / f['atmosphere/a-fps']))
    return rows


def test_cp(ct_s):
    return float(np.interp(ct_s, XH.CT_S, XH.CP_S)) if XH.CT_S[0] <= ct_s <= XH.CT_S[-1] else float('nan')


if __name__ == '__main__':
    for r in sweep():
        tc = test_cp(r['ct_s'])
        print('th75 %5.1f  CT/s %.4f  CP/s %.5f test %.5f (%+5.1f %%)  FM %.3f  tilt %.2f  Mtip %.3f' % (
            r['th75'], r['ct_s'], r['cp_s'], tc, 100 * (r['cp_s'] / tc - 1) if tc == tc else float('nan'), r['fm'], r['tilt'], r['mtip']))
