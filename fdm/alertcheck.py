#!/usr/bin/env python3
"""The crew alerts against the report's programme (report/mkresults.py's stages): every flight the report flies, with
each alert (alerts.py) logged every 0.1 s of simulated time, to find the ones a case raises that it should not (the
nuisance alerts) and to show that the alert channel leaves the flight model as it was (each stage's results written
to OUT and compared with report/results).

    python3 alertcheck.py OUT [stage ...]       all stages in parallel, or the ones named

Prints, per stage, each alert raised in flight (a trim's frozen evaluations skipped): for how long in all, when first, and the flight then (height above ground, KCAS,
nacelle angle, on the ground); and the extremes in flight of the margins the alerts' thresholds are set against
(margins()). Run genv22.py root/aircraft/mv22b (and the XV-15's, the polar's) first."""
import os, sys, json
from multiprocessing import Pool
import jsbsim

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import alerts as AL
import v22data as U

IDS = [a['id'] for a in AL.table(U)] + ['ground']
EVERY = 48                     # steps: 0.1 s at the harness's 1/480 s
LOG = {}                       # id -> [seconds raised, first sim time, the flight then]
EXT = {}                       # quantity -> [least, greatest]: the margins the alerts' thresholds are set against


def margins(f):
    """in flight (no wheel on the ground): the rotor speed off its reference, %; the gimbals' greater flapping, deg;
    the busier engine's power over its limit (its own or its share of the drive's); the power to hover at this height
    (fdm/displays' estimate) over the power the running engines have, slow (under 60 KEAS) with the nacelles past 60
    deg, and over the power the drive delivers in a steady hover (under 5 kt, under 100 ft/min); the nacelles past the
    conversion corridor's least and short of its most angle for the speed, deg"""
    if f['gear/wow'] > 0.5:
        return {}
    import math
    import numpy as np
    out = {'nr-dev': f['drive/nr-pct'] - f['gov/nr-ref-pct'],
           'flap-deg': math.degrees(max(f['rotor/r/tilt-rad'], f['rotor/l/tilt-rad']))}
    lim = min(f['eng/power-available-hp'], f['eng/xmsn-limit-hp'])
    if lim > 50:
        out['power/limit'] = max(f['eng[0]/power-hp'], f['eng[1]/power-hp']) / lim
    nac, ve = f['ctl/nacelle-deg'], f['velocities/ve-kts']
    (lv, ln), (uv, un) = U.CPS
    out['nac-over-least'] = nac - float(np.interp(ve, lv, ln))
    out['nac-under-most'] = float(np.interp(ve, uv, un)) - nac
    if nac > 60 and ve < 60:
        nrun = max(1, (f['eng[0]/running'] > 0.5) + (f['eng[1]/running'] > 0.5))
        out['hover/available'] = hover_power(f) / (nrun * lim)
    if nac > 80 and f['velocities/vg-fps'] < 5 * 1.6878 and abs(f['velocities/h-dot-fps']) < 100 / 60.0:
        out['hover-estimate/actual'] = hover_power(f, actual=True)          # in a steady hover
    return out


def hover_power(f, actual=False):
    """the power to hover here (Nasal/displays.nas's hover_power with the wheels' own height); actual: over the power
    the drive delivers now (while hovering, about 1)"""
    import math
    W0, P0, R, ZH = 47700.0, U.P_XMSN_100, U.R, (U.WL_PIV + U.DZ_HUB - U.MAIN_GEAR[2]) / 12
    p = P0 * (f['inertia/weight-lbs'] / W0) ** 1.5 / math.sqrt(max(0.2, f['atmosphere/sigma']))
    z = ZH + max(0.0, f['position/h-agl-ft'] - (f['inertia/cg-z-in'] - U.MAIN_GEAR[2]) / 12)
    kge = (1 + 0.3 * (R / z) ** 2) ** -0.5
    p *= 1 - 0.75 + 0.75 * kge
    return p / max(1.0, f['drive/power-required-hp']) if actual else p


class Logged(jsbsim.FGFDMExec):
    def run(self):
        ok = super().run()
        self._n = getattr(self, '_n', 0) + 1
        if self._n % EVERY == 0:
            try:
                vals = [self['alert/' + i] for i in IDS]
            except KeyError:                    # the XV-15's and others without the channel
                return ok
            if self['simulation/integrator/position/translational'] == 0:      # a trim's evaluation, not a flight
                return ok
            for k, x in margins(self).items():
                e = EXT.setdefault(k, [x, x])
                e[0], e[1] = min(e[0], x), max(e[1], x)
            for i, x in zip(IDS, vals):
                if x > 0.5:
                    e = LOG.setdefault(i, [0.0, None, None])
                    e[0] += 0.1
                    if e[1] is None:
                        e[1] = self['simulation/sim-time-sec']
                        e[2] = 'agl %.0f ft, %.0f KCAS, nacelle %.0f, wow %d' % (
                            self['position/h-agl-ft'], self['velocities/vc-kts'], self['ctl/nacelle-deg'], self['gear/wow'])
        return ok


jsbsim.FGFDMExec = Logged
sys.path.insert(0, os.path.join(HERE, 'report'))
import mkresults as MR


def run(args):
    name, out = args
    MR.OUT = out
    LOG.clear(); EXT.clear()
    err = None
    try:
        MR.STAGES[name]()
    except Exception:
        import traceback
        err = traceback.format_exc()
    return name, dict(LOG), dict(EXT), err


if __name__ == '__main__':
    out = os.path.abspath(sys.argv[1])
    os.makedirs(out, exist_ok=True)
    names = sys.argv[2:] or list(MR.STAGES)
    res = {}
    with Pool(min(len(names), os.cpu_count() or 4)) as pool:
        for name, log, ext, err in pool.imap_unordered(run, [(n, out) for n in names]):
            res[name] = dict(alerts=log, margins=ext)
            print('%-22s %s' % (name, 'FAILED\n' + err if err else
                                '; '.join('%s %.1f s from %.1f s (%s)' % (i, *v) for i, v in sorted(log.items())) or '-'), flush=True)
            if ext:
                print('%22s %s' % ('', '  '.join('%s %.3g..%.3g' % (k, *v) for k, v in sorted(ext.items()))), flush=True)
    json.dump(res, open(os.path.join(out, 'alerts.json'), 'w'), indent=1)
