#!/usr/bin/env python3
"""SINK RATE's projection ahead (alerts.py: none, from this) and its recovery cue, from the model: how far ahead of the vortex ring
state's onset the warning stands for each projection time, in entries flown as a pilot would (the power lever drawn
back at a steady rate from a hover or slow flight, the augmentation holding the attitude), against the warnings the
same projection gives in the report's own flights (report/mkresults.py's stages: nuisance); and, from the warning,
the recoveries a pilot has, flown: the lever up, the nacelles forward, the nose down.

Each rotor's z = Vz/vh and x = Vx/vh from the flight model, their rates through a TF lag here (started at the first
value, every 0.1 s of flight; a trim's frozen evaluations skipped); the warning evaluated for each projection as the
channel would: either rotor with x' < XM and z' < K_VRS z_N(x'), z' = z + tau min(0, dz/dt), x' = max(0, x + tau px
min(0, dx/dt)), held ON_S. The onset: z <= z_N(x).

    python3 vrswarn.py entries          the lead of each projection over the onset
    python3 vrswarn.py nuisance OUT     the report's stages (results to OUT), the warnings of each projection
    python3 vrswarn.py recovery TAU     the recoveries from the warning at that projection, and from the onset"""
import os, sys, math, json
import numpy as np
import jsbsim

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import vrs as VRS
import alerts as AL

# (projection s, the edgewise velocity projected too)
TAUS = ((0.0, 1), (2.0, 1), (3.0, 1), (4.0, 1), (2.0, 0), (4.0, 0), (6.0, 0), (8.0, 0))
ON_S = 0.5
TF = 1.0
EVERY = 48                              # steps: 0.1 s
XS = [VRS.XM * i / 24 for i in range(25)]
ZNS = [VRS.vrs_points(x)[1] for x in XS]


def zn(x):
    return float(np.interp(x, XS, ZNS))


def sides(f):
    """each rotor's z and x"""
    return [(f['aero/rotor/%s/vrs/z' % s], f['aero/rotor/%s/vrs/x' % s]) for s in 'rl']


def warn(sd, tau, k=AL.K_VRS):
    tau, px = tau if isinstance(tau, tuple) else (tau, 1)
    for z, x, zd, xd in sd:
        zp = z + tau * min(0.0, zd)
        xp = max(0.0, x + tau * px * min(0.0, xd))
        if xp < VRS.XM and zp < k * zn(xp):
            return True
    return False


def onset(sd):
    return any(x < VRS.XM and z <= zn(x) for z, x, _, _ in sd)


class Rates:
    """each rotor's z, x and their rates through a TF lag, every 0.1 s"""
    def __init__(s):
        s.lag = None

    def __call__(s, zx):
        if s.lag is None:
            s.lag = [list(v) for v in zx]
        out = []
        for (z, x), l in zip(zx, s.lag):
            out.append((z, x, (z - l[0]) / TF, (x - l[1]) / TF))
            l[0] += (z - l[0]) * 0.1 / TF; l[1] += (x - l[1]) * 0.1 / TF
        return out


class Tracker:
    """each projection's warning, held ON_S, over a flight sampled every 0.1 s: its time raised in all and its first"""
    def __init__(s):
        s.held = {t: 0.0 for t in TAUS}
        s.total = {t: 0.0 for t in TAUS}
        s.first = {t: None for t in TAUS}

    def step(s, t, sd, ground, frozen=False):
        if frozen:                              # a trim's evaluation, not a flight
            return
        for tau in TAUS:
            w = warn(sd, tau) and not ground
            s.held[tau] = s.held[tau] + 0.1 if w else 0.0
            if s.held[tau] >= ON_S - 1e-9:
                s.total[tau] += 0.1
                if s.first[tau] is None:
                    s.first[tau] = t


# ---------------------------------------------------------------- entries
def entry(V, rate, T=120.0, events=(), stop_at=None):
    """from a trim at V KCAS (AUTO NAC's nacelle angle), the lever drawn back at rate per s from its trim; the flight
    every 0.1 s: t, height, vertical speed (ft/min), KCAS, each projection's tracker, the onset's time. events:
    (t, property, value) from t on (a recovery); stop_at: stop then"""
    import hoverhold as HH
    import genv22
    sk, sn = genv22.autonac_schedule()
    nac = float(np.interp(V, sk, sn))
    f = HH.start(V, nac)
    tcl0 = f['/controls/engines/engine[0]/throttle']
    dt = f['simulation/dt']
    tr = Tracker()
    rates = Rates()
    rows, t_on = [], None
    for i in range(int(T / dt)):
        t = i * dt
        lever = max(0.0, tcl0 - rate * t)
        for ev in events:
            te, prop, val = ev[:3]
            if t >= te:
                if len(ev) > 3 and t >= te + ev[3]:          # a push held ev[3] s, then let go
                    val = 0.0
                if prop == 'lever':
                    lever = val
                else:
                    f[prop] = val
        f['/controls/engines/engine[0]/throttle'] = lever
        f.run()
        if i % EVERY == 0:
            sd = rates(sides(f))
            tr.step(t, sd, f['gear/wow'] > 0.5)
            if t_on is None and onset(sd):
                t_on = t
            rows.append((t, f['position/h-agl-ft'], -f['velocities/v-down-fps'] * 60, f['velocities/vc-kts'],
                         min(z for z, _, _, _ in sd), min(x for _, x, _, _ in sd), warn(sd, 0.0, 1.0)))
            if stop_at is not None and t >= stop_at:
                break
            if f['position/h-agl-ft'] < 50:
                break
    return dict(tracker=tr, onset=t_on, rows=rows, nac=nac, tcl0=tcl0)


ENTRY_SPEEDS = (0.0, 10.0, 20.0, 30.0, 40.0)
ENTRY_RATES = (0.005, 0.01, 0.02, 0.05)


def entry_row(args):
    V, r = args
    e = entry(V, r)
    tr, ton = e['tracker'], e['onset']
    fpm = next((row[2] for row in e['rows'] if row[0] >= ton), float('nan')) if ton is not None else float('nan')
    first = next((row[2] for row in e['rows'] if tr.first[TAUS[0]] is not None and row[0] >= tr.first[TAUS[0]]), float('nan'))
    return V, r, ton, fpm, first, {t: tr.first[t] for t in TAUS}


def entries():
    from multiprocessing import Pool
    print('lever drawn back from the trim at the rate (per s): the onset\'s time and descent rate, the warning\'s descent '
          'rate unprojected; each projection\'s lead over the onset, s')
    print('%5s %6s %8s %8s %8s  ' % ('KCAS', 'rate', 'onset s', 'fpm', 'warn fpm') + '  '.join('%3.0f%s' % (t, ' zx' if px else ' z ') for t, px in TAUS))
    with Pool(os.cpu_count() or 4) as pool:
        for V, r, ton, fpm, wf, first in pool.map(entry_row, [(V, r) for V in ENTRY_SPEEDS for r in ENTRY_RATES]):
            leads = ['%8s' % ('%.1f' % (ton - first[t]) if ton is not None and first[t] is not None else
                              ('none' if first[t] is None else 'no onset')) for t in TAUS]
            print('%5.0f %6.3f %8s %8.0f %8.0f  ' % (V, r, '%.1f' % ton if ton is not None else '-', fpm, wf) + '  '.join(leads), flush=True)


# ---------------------------------------------------------------- nuisance: the report's flights
TRACKERS = []


class Tracked(jsbsim.FGFDMExec):
    def run(self):
        ok = super().run()
        self._n = getattr(self, '_n', 0) + 1
        if self._n % EVERY == 0:
            try:
                zx = sides(self)
            except KeyError:
                return ok
            if self['simulation/integrator/position/translational'] == 0:      # a trim's evaluation, not a flight
                self._rates = Rates()
                return ok
            if not hasattr(self, '_tr'):
                self._tr = Tracker()
                TRACKERS.append(self._tr)
            if not hasattr(self, '_rates'):
                self._rates = Rates()
            self._tr.step(self['simulation/sim-time-sec'], self._rates(zx), self['gear/wow'] > 0.5)
        return ok


def stage(args):
    name, out = args
    jsbsim.FGFDMExec = Tracked
    sys.path.insert(0, os.path.join(HERE, 'report'))
    import mkresults as MR
    MR.OUT = out
    del TRACKERS[:]
    try:
        MR.STAGES[name]()
    except Exception as e:
        return name, None, repr(e)
    tot = {t: sum(tr.total[t] for tr in TRACKERS) for t in TAUS}
    first = {t: min([tr.first[t] for tr in TRACKERS if tr.first[t] is not None], default=None) for t in TAUS}
    return name, (tot, first), None


def nuisance(out):
    from multiprocessing import Pool
    sys.path.insert(0, os.path.join(HERE, 'report'))
    import mkresults as MR
    os.makedirs(out, exist_ok=True)
    names = [n for n in MR.STAGES if not n.startswith('xv15') and n != 'descent_xv15']
    res = {}
    with Pool(os.cpu_count() or 4) as pool:
        for name, r, err in pool.imap_unordered(stage, [(n, out) for n in names]):
            res[name] = r
            if err:
                print('%-22s FAILED %s' % (name, err), flush=True)
            else:
                tot, first = r
                print('%-22s ' % name + '  '.join('%2.0f%s: %5.1f s%s' % (t[0], 'zx' if t[1] else 'z', tot[t], ' from %.1f' % first[t] if first[t] is not None else '')
                                                for t in TAUS if tot[t] > 0) or '%-22s -' % name, flush=True)
    json.dump({k: (v and [{'%g%s' % (t[0], 'zx' if t[1] else 'z'): x for t, x in d.items()} for d in v]) for k, v in res.items()},
              open(os.path.join(out, 'vrswarn.json'), 'w'), indent=1)


# ---------------------------------------------------------------- recovery
# from the warning: the lever held where it is (the descent no longer deepened) unless raised
RECOVERIES = [('lever drawn on back', None),
              ('lever held', []),
              ('lever up', [('lever', 1.0)]),
              ('nacelles 20 fwd', [('nac', -20.0)]),
              ('nose down', [('fcs/elevator-cmd-norm', 0.3, 5.0)]),
              ('nacelles 20 fwd, nose down', [('nac', -20.0), ('fcs/elevator-cmd-norm', 0.3, 5.0)]),
              ('nacelles 20 fwd, lever up', [('nac', -20.0), ('lever', 1.0)])]


def recovery(tau, V=10.0, rate=0.02, at_onset=False):
    """the recoveries from the warning at that projection, or from the onset"""
    tau = (tau, 1) if tau == 0 else (tau, 0)
    base = entry(V, rate)
    tw = base['onset'] if at_onset else base['tracker'].first[tau]
    if tw is None:
        sys.exit('no warning at tau %g' % tau[0])
    if at_onset:
        print('from the onset:')
    print('%.0f KCAS, the lever drawn back at %.3f per s: SINK RATE at %.1f s (tau %g), the onset at %s s' % (
        V, rate, tw, tau[0], '%.1f' % base['onset'] if base['onset'] is not None else '-'))
    print('%-26s %10s %10s %10s %12s %10s' % ('from the warning', 'h lost ft', 'max fpm', 'least z', 'in the onset', 'clear at s'))
    for name, acts in RECOVERIES:
        ev = [] if acts is None else [(tw, 'lever', max(0.0, base['tcl0'] - rate * tw))]
        for a in acts or []:
            prop, val = a[:2]
            if prop == 'nac':
                ev.append((tw, '/controls/flight/nacelle-cmd-deg', base['nac'] + val))
            else:
                ev.append((tw, prop, val) + tuple(a[2:]))
        e = entry(V, rate, T=tw + 40.0, events=ev, stop_at=tw + 40.0)
        rows = [r for r in e['rows'] if r[0] >= tw]
        h0 = rows[0][1]
        lost = h0 - min(r[1] for r in rows)
        maxs = -min(r[2] for r in rows)
        inside = sum(0.1 for r in rows if r[6])
        clear = next((r[0] - tw for r in rows[5:] if not warn_row(r)), None)
        print('%-26s %10.0f %10.0f %10.3f %10.1f s %10s' % (name, lost, maxs, min(r[4] for r in rows), inside,
                                                          '%.1f' % clear if clear is not None else '-'), flush=True)


def warn_row(r):
    """the warning's own boundary (no projection) at a logged row: the more critical rotor's z against its x"""
    return r[5] < VRS.XM and r[4] < AL.K_VRS * zn(r[5])


if __name__ == '__main__':
    if sys.argv[1:2] == ['entries']:
        entries()
    elif sys.argv[1:2] == ['nuisance']:
        nuisance(os.path.abspath(sys.argv[2]))
    elif sys.argv[1:2] == ['recovery']:
        recovery(float(sys.argv[2]))
        recovery(float(sys.argv[2]), at_onset=True)
