#!/usr/bin/env python3
"""Run the validation programme of the report into report/results/*.json (the figures, tables and numbers are made
from these by mktables.py and figures.py). Regenerate the harness's models first:

    cd fdm && python3 genv22.py root/aircraft/mv22b && python3 genv22.py root-xv15/aircraft/xv15 --xv15
    python3 genv22.py root-polar/aircraft/mv22b --no-inline
    python3 report/mkresults.py [stage ...]      all stages in parallel, or the ones named

Stages: hover (hover_rotor.py), xv15_<set> for the sets of data/xv15_flight.py and xv15_nac0_gear (sweep.py), download (download.py),
polar (airframe_polar.py), speed, ceiling<alt>, conversion<nacelle> (performance.py), handling_hover,
handling_conversion, start (handling.py), hover_hold_stop, hover_hold_turn, hover_hold_controls (hoverhold.py), autopilot_holds,
autopilot_approach, autopilot_lnav, tf_response, tf_<terrain>_<kt> and tf_jitter_<terrain>_<kt> (aptest.py), freq_hover, freq_cruise
(freqresp.py), derivs (derivs.py), descent_v22, descent_xv15, vrs_encounter (descent.py)."""
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
    """at sea level and 10 000 ft, the airplane-mode sets also at 16 000 ft (where the fastest point was flown)"""
    import sweep
    alts = (0.0, 10000.0, 16000.0) if name in ('nac0', 'nac0f20') else (0.0, 10000.0)
    dump('xv15_' + name, {str(int(a)): sweep.run(name, a) for a in alts})


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


def autopilot(what):
    """the autopilot (aptest.py): the holds and AUTO NAC (an altitude change, a turn, the conversion down and up, a slow
    turn, an engagement off the trim); the approaches (to a hover and landings, the route's, go-arounds); LNAV's turns"""
    import aptest as A
    if what == 'holds':
        out = {n: {k: v[::5] for k, v in getattr(A, n)().items()} for n in ('alt', 'hdg', 'decel', 'accel', 'slow')}
        out['engage'] = A.engage_numbers(A.engage())
    elif what == 'approach':
        r = A.route_approach()
        t, d, vc, gp, route = (np.array(r[k]) for k in ('t', 'd', 'vc', 'gp', 'route'))
        i0, j = int(np.argmax(vc < vc[0] - 2)), int(np.argmax(gp > 0.5))
        rt = dict(A.approach_numbers(r), decel_nm=d[i0] / 6076.1, planner_nm=vc[0] ** 2 / (2 * 1.5 * 3600), join_nm=d[j] / 6076.1, join_route=route[j])
        out = dict(final=[dict(angle=a, land=l, wind=w, **A.approach_numbers(A.approach(a, l, w))) for a, l, w in ((3.0, True, 0.0), (6.0, False, 0.0), (6.0, True, 15.0))],
                   route=rt, ga=[A.ga(w) for w in (6000.0, 2500.0, 'hover')])
    else:
        out = [dict(V=V, direct=d, legs=A.lnav_numbers(A.lnav(V, n, wps=w, direct=d, T=420.0 if w is None else 300.0), d)) for V, n, w, d in A.LNAV_CASES]
    dump('autopilot_' + what, out)


def freq(cond):
    """the XV-15's frequency responses against Tischler's flight tests' (freqresp.py): each pair's model over its
    fitting range, magnitude dB and phase deg, the coherence and the costs"""
    import freqresp as FR
    out = {}
    for pair, r in FR.compare(cond).items():
        m = r['model']
        out[pair] = dict(w=r['w'], db=20 * np.log10(np.abs(m)), deg=np.degrees(np.angle(m)), coherence=r['coherence'], J=r['J'],
                         J_bell=r.get('J_bell'), dmag_db=r['dmag_db'], dphase_deg=r['dphase_deg'], trim_res=r['trim_res'])
    if cond == 'cruise':
        out['derivatives'] = FR.pitch_derivatives()
    dump('freq_' + cond, out)


def derivs():
    """the XV-15's stability derivatives against Bell's [CR table III-1] (derivs.py)"""
    import derivs as D
    d, info = D.derivatives()
    dump('derivs', dict(model=d, bell=D.BELL, **info))


def tf_response():
    """TF's climb's response (aptest.py tf_lag): its step at each speed, the fitted lags, the tracking gain"""
    import aptest as A
    dump('tf_response', A.tf_lag())


def tf(name, V, frame=0.0):
    """terrain following (aptest.py tf_run) over a profile at V KCAS from AUTO NAC's angle, 200 ft set, the medium ride;
    frame: FlightGear's timer up to that late (s)"""
    import aptest as A
    from genv22 import autonac_schedule
    sk, sn = autonac_schedule()
    r = A.tf_run(V, float(np.interp(V, sk, sn)), name, frame=frame)
    t, hw, hd, th = (np.array(r[k]) for k in ('t', 'hw', 'hdot', 'theta'))
    w = t > 5
    acc = np.gradient(hd, t) / 32.174
    out = dict(name=name, V=V, frame=frame, clr=200.0, hw_min=hw[w].min(), hw_max=hw[w].max(), hw_mean=hw[w].mean(), g_min=acc[w].min(), g_max=acc[w].max(),
               theta_range=(th[w].min(), th[w].max()), theta_rms=float(np.std(th[t > 20])), vc_range=(min(r['vc']), max(r['vc'])),
               flyup_s=float((np.array(r['flyup']) > 0.5).sum() * 0.1), cap_fpm=(min(r['cap']) * 60, max(r['cap']) * 60))
    if not frame:
        out.update({k: r[k][::5] for k in ('t', 'y', 'terr', 'hw', 'cmd', 'hdot', 'vc', 'theta', 'tcl')})
    dump('tf_%s%s_%d' % ('jitter_' if frame else '', name, V), out)


def descent(xv15):
    """steady descents through the vortex ring state (descent.py)"""
    import descent as DS
    dump('descent_' + ('xv15' if xv15 else 'v22'), DS.sweep(xv15))


def vrs_encounter():
    """Johnson's V-22 encounter, symmetric and in sideslip (descent.py)"""
    import descent as DS
    dump('vrs_encounter', [DS.encounter(b) for b in (0.0, -15.0)])


TF_CASES = [(n, V) for n in ('flat', 'hills', 'ridge') for V in (120, 150, 200)] + [('ridge', 170)]
TF_JITTER = [(n, V) for n in ('flat', 'ridge') for V in (120, 200)]

STAGES = dict(hover=hover, xv15_nac0_gear=xv15_gear, download=download, polar=polar, speed=speed, start=start,
              handling_hover=lambda: handling('hover'), handling_conversion=lambda: handling('conversion'),
              **{'hover_hold_' + w: (lambda w=w: hover_hold(w)) for w in ('stop', 'turn', 'controls')},
              **{'xv15_' + s: (lambda s=s: xv15(s)) for s in ('nac90', 'nac60', 'nac30', 'nac0f20', 'nac0')},
              **{'ceiling%d' % a: (lambda a=a: ceiling(a)) for a in CEILING_ALTS},
              **{'conversion%d' % n: (lambda n=n: conversion(n)) for n in NACELLES},
              **{'autopilot_' + w: (lambda w=w: autopilot(w)) for w in ('holds', 'approach', 'lnav')}, tf_response=tf_response,
              **{'freq_' + c: (lambda c=c: freq(c)) for c in ('hover', 'cruise')}, derivs=derivs,
              **{'tf_%s_%d' % c: (lambda c=c: tf(c[0], float(c[1]))) for c in TF_CASES},
              **{'tf_jitter_%s_%d' % c: (lambda c=c: tf(c[0], float(c[1]), 0.077)) for c in TF_JITTER},
              descent_v22=lambda: descent(False), descent_xv15=lambda: descent(True), vrs_encounter=vrs_encounter)


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
