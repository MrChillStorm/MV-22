#!/usr/bin/env python3
"""Level-flight trims against the XV-15's flight-measured rotor shaft power [data/xv15_flight.py, NASA TM-84222 fig.
20], by nacelle angle and flap setting, at calibrated airspeed: the figure gives no altitudes, so the model is trimmed
at sea level and at 10 000 ft (the range of most of the programme's points) at 13 000 lb.

  python3 sweep.py [set ...]       sets: nac90 nac60 nac30 nac0f20 nac0 (all by default)"""
import sys
import numpy as np
import harness as H
from data import xv15_flight as XF

EXTRA = ('aero/rotor/r/power-hp', 'aero/rotor/l/power-hp', 'ctl/theta75-deg', 'ctl/b1s-deg', 'ctl/elevator-deg',
         'aero/wing/lift-lbs', 'aero/rotor/r/thrust-lbs', 'attitude/theta-deg', 'aero/rotor/r/a1-deg')


def run(name, alt_ft, speeds=None, gear=0.0):
    """gear: its position (the harness's default up; 1 extended, Bell's 4 ft^2 of drag [CR B-35])"""
    nac, flaperon, pts = XF.SETS[name]
    f = H.load(xv15=True)
    f['gear/gear-cmd-norm'] = gear; f['gear/gear-pos-norm'] = gear
    f['ctl/flaperon-override'] = 1
    f['ctl/flaperon-set-deg'] = flaperon
    speeds = speeds or sorted({round(v) for v, p in pts})
    rows, x0 = [], None
    for V in speeds:
        x, r, e = H.trim(f, V, alt_ft=alt_ft, x0=x0, nacelle=nac, cas=True, extra=EXTRA)
        x0 = x
        rows.append(dict(V=V, alt=alt_ft, hp=float(e[0] + e[1]), th75=float(e[2]), b1s=float(e[3]), de=float(e[4]),
                         wing_lift=float(e[5]), T=float(e[6]), pitch=float(e[7]), a1=float(e[8]), x=[float(a) for a in x],
                         res=float(np.abs(r).max())))
    return rows


if __name__ == '__main__':
    names = sys.argv[1:] or list(XF.SETS)
    for name in names:
        pts = XF.SETS[name][2]
        for alt in (0.0, 10000.0):
            for r in run(name, alt):
                near = [p for v, p in pts if abs(v - r['V']) < 3]
                print('%-7s %5.0f ft %5.0f kt  hp %6.0f  test %-12s  th75 %5.1f  lon %6.3f  pitch %5.1f  wing %6.0f  res %.3f' % (
                    name, alt, r['V'], r['hp'], ','.join('%.0f' % p for p in near), r['th75'], r['x'][1], r['pitch'], r['wing_lift'], r['res']))
