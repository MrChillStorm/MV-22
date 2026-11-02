#!/usr/bin/env python3
"""The hover hold (genv22.hover_hold), offline, with the augmentation on: engaged from a trim at a ground speed, the
stop it shows against the stop it makes; the ADS-33E-PRF position hold check (a 360 deg pedal turn in a steady wind,
within a 10 ft circle, attitudes within 18 deg); the stick's translational rate (its rise time, 2.5-5 s, and no
overshoot); the height held through a lever movement.

  python3 hoverhold.py stop [kt ...] | turn [wind_kt] | trc | lever"""
import sys, math, json
import numpy as np
import harness as H
from genv22 import CONTROL_EXECRATE
from data import bell301 as B

R_E = 20925646.0
LOG = [('n', None), ('e', None), ('gs', 'velocities/vg-fps'), ('theta', 'attitude/theta-deg'), ('phi', 'attitude/phi-deg'),
       ('psi', 'attitude/psi-deg'), ('nac', 'ctl/nacelle-deg'), ('h', 'position/h-agl-ft'), ('tcl', '/controls/engines/engine[0]/throttle'),
       ('state', 'hh/state'), ('on', 'hh/on'), ('dist', 'hh/e-ft'), ('nr', 'drive/nr-pct')]


def start(V, nacelle, wind_kt=0.0, wind_from=0.0, alt=1000.0, terrain=H.TERRAIN_FT, gamma=0.0):
    """from a trim at V KCAS heading north (on the flight path angle gamma, deg), 1000 ft over flat ground far below (or
    the terrain at that elevation), the augmentation on, let go"""
    f = H.load()
    x, r, e = H.trim(f, V, alt_ft=alt, nacelle=nacelle, cas=True, extra=('drive/power-required-hp',), terrain=terrain,
                     gamma=gamma)
    avail = min(f['eng/power-available-hp'], f['eng/xmsn-limit-hp'])
    tcl = min(1.0, max(0.0, (e[0] / 2 / avail - 0.17) / 0.83))
    ff0 = f['ctl/theta75-deg'] - f['gov/int-deg']
    f['/controls/engines/engine[0]/throttle'] = tcl
    f['/controls/flight/nacelle-cmd-deg'] = nacelle
    for _ in range(CONTROL_EXECRATE):
        f.run()
    f['gov/int-deg'] = x[0] - (f['ctl/theta75-deg'] - f['gov/int-deg'] - ff0)
    f['ctl/init'] = 0; f['drive/hold-rpm'] = 0; f['ctl/scas-on'] = 1; f['ctl/cps-on'] = 1
    for a in ('elevator', 'aileron', 'rudder'):
        f['fcs/%s-cmd-norm' % a] = 0.0
    for _ in range(CONTROL_EXECRATE):
        f.run()
    H.freeze(f, False)
    f['scas/theta-int-in'] = x[1] * B.X_LON - f['scas/lon-trim-in']; f['scas/phi-int-in'] = x[2] * B.X_LAT
    for ax in ('theta', 'phi', 'psi'):
        f['scas/%s-ref-rad' % ax] = f['attitude/%s-rad' % ax]
    if wind_kt:
        w = wind_kt * 1.6878
        f['atmosphere/wind-north-fps'] = -w * math.cos(math.radians(wind_from))
        f['atmosphere/wind-east-fps'] = -w * math.sin(math.radians(wind_from))
    return f


def run(f, T, events=(), every=0.1):
    """T s, events (t, property, value); the position north and east of the start, ft"""
    dt = f['simulation/dt']
    la0, lo0 = f['position/lat-geod-rad'], f['position/long-gc-rad']
    out = {k: [] for k, _ in LOG}; out['t'] = []
    n_every = int(round(every / dt))
    for i in range(int(T / dt)):
        t = i * dt
        for te, prop, val in events:
            if abs(t - te) < dt / 2:
                f[prop] = val
        f.run()
        if i % n_every == 0:
            out['t'].append(t)
            for k, prop in LOG:
                if k == 'n':
                    out[k].append((f['position/lat-geod-rad'] - la0) * R_E)
                elif k == 'e':
                    out[k].append((f['position/long-gc-rad'] - lo0) * R_E * math.cos(la0))
                else:
                    out[k].append(f[prop] / (1.6878 if k == 'gs' else 1))
    return out


def stop(V, nacelle=None, T=None):
    """engaged at V (KCAS, no wind: the ground speed), held hands off: the stop shown, the stop made"""
    nacelle = nacelle if nacelle is not None else (90.0 if V < 15 else 85.0)
    f = start(V, nacelle)
    vg = f['velocities/vg-fps']
    shown = vg * vg / (2 * f['hh/decel-g'] * 32.174)
    T = T or max(30.0, 8 + 1.5 * V)
    r = run(f, T, [(1.0, 'hh/engage-cmd', 1)])
    i1 = int(1.0 / 0.1)
    n, e = np.array(r['n']), np.array(r['e'])
    n0, e0 = n[i1], e[i1]
    return dict(V=V, nacelle=nacelle, shown_ft=shown, stop_n_ft=float(n[-1] - n0), stop_e_ft=float(e[-1] - e0),
                overshoot_ft=float(max(0.0, (n - n0).max() - (n[-1] - n0))), **r)


def turn(wind_kt=35.0, pedal=-0.38, T=140.0, every=0.1, **gains):
    """ADS-33E-PRF 3.3.11: the hold engaged in the hover, a wind from ahead rising to wind_kt over 30 s, then a 360 deg
    turn on the pedals (pitch and roll free, the lever the hold's), in about 30 s (Moderate agility): the largest
    distance from the point through it, the turn's time, the largest pitch and roll"""
    f = start(0.0, 90.0)
    for k, x in gains.items():
        f[k.replace('__', '/').replace('_', '-')] = x
    dt = f['simulation/dt']
    la0, lo0 = f['position/lat-geod-rad'], f['position/long-gc-rad']
    w = wind_kt * 1.6878
    out = {k: [] for k in ('t', 'n', 'e', 'psi', 'theta', 'phi', 'nac', 'dist', 'h', 'turning')}
    turned, last, t_on, t_off = 0.0, None, None, None
    for i in range(int(T / dt)):
        t = i * dt
        if abs(t - 1.0) < dt / 2:
            f['hh/engage-cmd'] = 1
        f['atmosphere/wind-north-fps'] = -w * min(1.0, max(0.0, (t - 5.0) / 30.0))
        psi = f['attitude/psi-deg']
        if last is not None and t_on is not None and t_off is None:
            turned += ((psi - last + 540) % 360) - 180
        last = psi
        if t_on is None and t >= 60.0:
            t_on = t; f['fcs/rudder-cmd-norm'] = pedal
        if t_on is not None and t_off is None and abs(turned) >= 360.0 - 15.0:
            t_off = t; f['fcs/rudder-cmd-norm'] = 0.0
        f.run()
        if i % int(round(every / dt)) == 0:
            out['t'].append(t)
            out['n'].append((f['position/lat-geod-rad'] - la0) * R_E)
            out['e'].append((f['position/long-gc-rad'] - lo0) * R_E * math.cos(la0))
            for k, prop in (('psi', 'attitude/psi-deg'), ('theta', 'attitude/theta-deg'), ('phi', 'attitude/phi-deg'), ('nac', 'ctl/nacelle-deg'),
                            ('dist', 'hh/e-ft'), ('h', 'position/h-agl-ft')):
                out[k].append(f[prop])
            out['turning'].append(1 if (t_on is not None and t_off is None) else 0)
    return dict(wind_kt=wind_kt, t_on=t_on, t_off=t_off, **out)


def trc(stick=0.25, T=45.0):
    """the stick's ground speed: a step out of the detent from the hold (forward), held 20 s, released"""
    f = start(0.0, 90.0)
    r = run(f, T, [(1.0, 'hh/engage-cmd', 1), (5.0, 'fcs/elevator-cmd-norm', stick), (25.0, 'fcs/elevator-cmd-norm', 0.0)])
    return dict(stick=stick, **r)


def lever(dtcl=-0.06, ramp=4.0, T=40.0):
    """the height held; the pilot moves the lever by dtcl over ramp s (all the while, as a key held down does; down: the
    reference loading hovers near full power) and leaves it: the height change, and the height held after"""
    f = start(0.0, 90.0)
    r0 = run(f, 6.0, [(1.0, 'hh/engage-cmd', 1)])
    tcl = f['/controls/engines/engine[0]/throttle']
    n = int(ramp / 0.1)
    r = run(f, T, [(0.1 * k, '/controls/engines/engine[0]/throttle', tcl + dtcl * (k + 1) / n) for k in range(n)])
    return dict(dtcl=dtcl, ramp=ramp, h0=r0['h'][-1], **r)


if __name__ == '__main__':
    what = sys.argv[1] if len(sys.argv) > 1 else 'stop'
    if what == 'stop':
        for V in [float(a) for a in sys.argv[2:]] or [0.0, 10.0, 20.0, 40.0]:
            r = stop(V)
            t, st = np.array(r['t']), np.array(r['state'])
            hold_t = t[np.argmax(st == 2)] - 1.0 if (st == 2).any() else float('nan')
            print('%4.0f kt: stop shown %6.1f ft, made %6.1f ft (east %5.1f), overshoot %4.1f ft, held after %5.1f s; '
                  'pitch %5.1f..%4.1f roll %5.1f..%4.1f nacelle %5.1f..%5.1f h %6.1f..%6.1f state %s' % (
                      r['V'], r['shown_ft'], r['stop_n_ft'], r['stop_e_ft'], r['overshoot_ft'], hold_t, min(r['theta']), max(r['theta']),
                      min(r['phi']), max(r['phi']), min(r['nac']), max(r['nac']), min(r['h']), max(r['h']), int(st[-1])))
    elif what == 'turn':
        r = turn(float(sys.argv[2]) if len(sys.argv) > 2 else 35.0)
        t, d, tu = np.array(r['t']), np.array(r['dist']), np.array(r['turning'])
        w = (t > r['t_on'] - 1) & (t < r['t_off'] + 15)
        print('wind %2.0f kt: before the turn %4.1f ft, nacelle %4.1f; turn %4.1f s, largest %4.1f ft from the point (10 ft circle: '
              'radius 5), pitch %5.1f..%4.1f roll %5.1f..%4.1f; at the end %4.1f ft' % (
                  r['wind_kt'], d[(t > 50) & (t < 60)].max(), np.array(r['nac'])[(t > 55) & (t < 60)].mean(), r['t_off'] - r['t_on'],
                  d[w].max(), np.array(r['theta'])[w].min(), np.array(r['theta'])[w].max(), np.array(r['phi'])[w].min(), np.array(r['phi'])[w].max(), d[-1]))
    elif what == 'trc':
        r = trc()
        t, gs = np.array(r['t']), np.array(r['gs'])
        i = (t >= 5) & (t < 25)
        vss = gs[(t > 20) & (t < 25)].mean()
        t63 = t[i][np.argmax(gs[i] >= 0.632 * vss)] - 5.0
        print('stick %.2f: %4.1f kt steady, 63 %% at %3.1f s, overshoot %4.1f kt; released: stopped %4.1f s later, %4.1f ft on; pitch %4.1f..%4.1f' % (
            r['stick'], vss, t63, gs[i].max() - vss, t[(t > 25) & (np.array(r['state']) == 2)][0] - 25 if ((t > 25) & (np.array(r['state']) == 2)).any() else -1,
            r['dist'][-1], min(r['theta']), max(r['theta'])))
    elif what == 'lever':
        r = lever()
        t, h, fly = np.array(r['t']), np.array(r['h']), np.array(r['tcl'])
        print('lever %+.2f over %.0f s: %+5.1f ft, then held within %4.1f ft over the last 10 s (lever %.3f -> %.3f)' % (
            r['dtcl'], r['ramp'], h[-1] - r['h0'], np.ptp(h[t > t[-1] - 10]), r['tcl'][0], r['tcl'][-1]))
