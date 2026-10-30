#!/usr/bin/env python3
"""The wing's download in the hover against the flaps: the XV-15 build against its flight tests and the V-22 build
against the 0.658-scale V-22 rotor and wing test [Rosenstein and Clark, ERF 1986, fig. 18: XV-15 at 13 300 lb, h/D
2.5; V-22 OARF data at 67 deg], trimmed out of ground effect at sea level. genv22.DL_K is fitted to the XV-15's.

  python3 download.py             prints download / thrust, model and test"""
import harness as H

# (flap angle deg, download / thrust): XV-15 flight test, the figure's fair curve; V-22 large-scale test
XV15 = [(0, 0.164), (40, 0.130), (60, 0.104), (75, 0.105)]
V22 = [(67, 0.092)]
XV15_FLAPERON = 0.625       # the XV-15's flaperon per degree of flap [TM X-62407 fig. 3.4.1]


def run(xv15, flaps, weight=None):
    f = H.load(xv15=xv15)
    f['ctl/flaperon-override'] = 1
    rows = []
    for fl, test in flaps:
        f['ctl/flaperon-set-deg'] = fl * XV15_FLAPERON if xv15 else fl
        x, r, e = H.trim(f, 0.0, alt_ft=0.0, extra=('aero/wing/download-lbs', 'aero/rotor/r/thrust-lbs', 'aero/rotor/l/thrust-lbs',
                                                     'aero/rotor/r/power-hp', 'aero/rotor/l/power-hp'))
        dl, tr, tl, pr, pl = e
        rows.append(dict(flap=fl, test=test, dl_t=float(dl / (tr + tl)), hp=float(pr + pl), res=float(abs(r).max())))
    return rows


if __name__ == '__main__':
    for name, xv, data in (('XV-15', True, XV15), ('V-22', False, V22)):
        for r in run(xv, data):
            print('%-6s flap %4.0f  DL/T %.3f  test %.3f  rotor power %6.0f hp  res %.3f' % (name, r['flap'], r['dl_t'], r['test'], r['hp'], r['res']))
