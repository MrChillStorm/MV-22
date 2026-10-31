#!/usr/bin/env python3
"""Run the validation programme of the report into report/results/*.json (the figures, tables and numbers are made
from these by mktables.py and figures.py). Regenerate the harness's models first:

    cd fdm && python3 genv22.py root/aircraft/mv22b && python3 genv22.py root-xv15/aircraft/xv15 --xv15
    python3 genv22.py root-polar/aircraft/mv22b --no-inline
    python3 report/mkresults.py [stage ...]      all stages in parallel, or the ones named

Stages: hover (hover_rotor.py), xv15_<set> for the sets of data/xv15_flight.py and xv15_nac0_gear (sweep.py), download (download.py),
polar (airframe_polar.py), speed, ceiling<alt>, conversion<nacelle> (performance.py), handling_hover,
handling_conversion, start (handling.py), hover_hold_stop, hover_hold_turn, hover_hold_controls (hoverhold.py)."""
import os, sys, json, time
from multiprocessing import Pool
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FDM = os.path.dirname(HERE)
sys.path.insert(0, FDM)
os.chdir(FDM)
OUT = os.path.join(HERE, 'results')
CEILING_ALTS = [0, 4000, 8400, 12000, 16000]
NACELLES = [90, 75, 60, 30, 0]


def plain(x):
    if isinstance(x, dict):
        return {str(k): plain(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [plain(v) for v in x]
    if isinstance(x, np.ndarray):
        return plain(x.tolist())
    if isinstance(x, (np.floating, np.integer)):
        return x.item()
    return x


def dump(name, obj):
    json.dump(plain(obj), open(os.path.join(OUT, name + '.json'), 'w'), indent=1)


def hover():
    import hover_rotor as HR
    dump('hover', HR.sweep())


def xv15(name):
    import sweep
    dump('xv15_' + name, {str(int(a)): sweep.run(name, a) for a in (0.0, 10000.0)})


def xv15_gear():
    """airplane mode, flaps up, with the landing gear extended (its 4 ft^2 of drag)"""
    import sweep
    dump('xv15_nac0_gear', {str(int(a)): sweep.run('nac0', a, gear=1.0) for a in (0.0, 10000.0)})


def download():
    import download as DL
    dump('download', dict(xv15=DL.run(True, DL.XV15), v22=DL.run(False, DL.V22)))


def polar():
    import airframe_polar as AP
    dump('polar', dict(rows=AP.polar(), test=AP.TEST))


def speed():
    import performance as P
    dump('speed', [dict(gw=gw, alt=alt, vmax=vm, rows=rows) for gw, alt in ((42000, 0), (47500, 0), (42000, 17000))
                   for vm, rows in [P.max_speed(gw, alt)]])


def ceiling(alt):
    import performance as P
    gw, pts = P.hover_ceiling(alt)
    dump('ceiling%d' % alt, dict(alt=alt, gw=gw, pts=pts))


def conversion(nac):
    import performance as P
    n, rows = P.conversion(nac)
    dump('conversion%d' % nac, dict(nacelle=n, rows=rows))


def handling(what):
    import handling as HD
    dump('handling_' + what, getattr(HD, what)())


def start():
    import handling as HD
    dump('start', HD.start())


def hover_hold(what):
    """the hover hold (hoverhold.py): its stops from 10, 20 and 39 kt; ADS-33's 360 deg turn in 0, 20 and 35 kt of wind;
    the stick's translational rate and the lever's fly-through"""
    import hoverhold as HH
    def thin(r, keys, step=2):             # every 0.2 s, the series the figures and numbers use
        return {k: (r[k][::step] if isinstance(r[k], list) else r[k]) for k in r if k in keys or not isinstance(r[k], list)}
    if what == 'stop':
        out = [thin(HH.stop(V), ('t', 'n', 'gs', 'theta', 'phi', 'nac', 'dist', 'state')) for V in (10.0, 20.0, 39.0)]
    elif what == 'turn':
        out = [thin(HH.turn(w), ('t', 'n', 'e', 'psi', 'theta', 'phi', 'dist', 'turning'), 5) for w in (0.0, 20.0, 35.0)]
    else:
        keys = ('t', 'gs', 'dist', 'h', 'tcl', 'theta', 'state')
        out = dict(trc=thin(HH.trc(), keys), lever=thin(HH.lever(), keys))
    dump('hover_hold_' + what, out)


STAGES = dict(hover=hover, xv15_nac0_gear=xv15_gear, download=download, polar=polar, speed=speed, start=start,
              handling_hover=lambda: handling('hover'), handling_conversion=lambda: handling('conversion'),
              **{'hover_hold_' + w: (lambda w=w: hover_hold(w)) for w in ('stop', 'turn', 'controls')},
              **{'xv15_' + s: (lambda s=s: xv15(s)) for s in ('nac90', 'nac60', 'nac30', 'nac0f20', 'nac0')},
              **{'ceiling%d' % a: (lambda a=a: ceiling(a)) for a in CEILING_ALTS},
              **{'conversion%d' % n: (lambda n=n: conversion(n)) for n in NACELLES})


def run(name):
    t0 = time.time()
    try:
        STAGES[name]()
        return name, time.time() - t0, None
    except Exception:                            # one stage's failure leaves the others' results
        import traceback
        return name, time.time() - t0, traceback.format_exc()


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    names = sys.argv[1:] or list(STAGES)
    with Pool(min(len(names), os.cpu_count() or 4)) as pool:
        for name, dt, err in pool.imap_unordered(run, names):
            print('%-20s %6.0f s %s' % (name, dt, 'FAILED\n' + err if err else 'ok'), flush=True)
