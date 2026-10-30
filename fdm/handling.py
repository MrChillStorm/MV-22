#!/usr/bin/env python3
"""The FlightGear aircraft's handling with its augmentation (genv22.scas), offline: the free aircraft from a trim with
the stick let go to its centre (the trim's stick offset taken up by the integrators, the references at the trim's
attitude), then held steps of the controls, as a keyboard gives them (5 % per key press); and a start of the engines
on the ground, cold, with the lever down, to the governed rotor speed.

  python3 handling.py hover|conversion|start"""
import sys, math
import numpy as np
import harness as H
from data import bell301 as B

AX = {'lon': 'fcs/elevator-cmd-norm', 'lat': 'fcs/aileron-cmd-norm', 'ped': 'fcs/rudder-cmd-norm',
      'tcl': '/controls/engines/engine[0]/throttle', 'nac': '/controls/flight/nacelle-cmd-deg'}
LOG = [('theta', 'attitude/theta-deg'), ('phi', 'attitude/phi-deg'), ('psi', 'attitude/psi-deg'), ('kcas', 'velocities/vc-kts'),
       ('h', 'position/h-sl-ft'), ('nacelle', 'ctl/nacelle-deg'), ('nr', 'drive/nr-pct'), ('acah', 'scas/attitude-command'),
       ('lon_in', 'ctl/lon-in'), ('lat_in', 'ctl/lat-in')]


def free(V, nacelle, events, T, every=0.1):
    """from a trim at V (KCAS) and nacelle, 1000 ft, the reference loading, augmentation on; events: (t, axis, value)"""
    f = H.load()
    x, r, e = H.trim(f, V, alt_ft=1000.0, nacelle=nacelle, cas=True, extra=('drive/power-required-hp',))
    avail = min(f['eng/power-available-hp'], f['eng/xmsn-limit-hp'])
    tcl = min(1.0, max(0.0, (e[0] / 2 / avail - 0.17) / 0.83))     # the lever for the trim's power (drive.IDLE 0.17)
    ff0 = f['ctl/theta75-deg'] - f['gov/int-deg']
    f[AX['tcl']] = tcl
    f[AX['nac']] = nacelle
    f.run()
    # the collective stays the trim's: the governor's integral takes back the lever's feed forward
    f['gov/int-deg'] = x[0] - (f['ctl/theta75-deg'] - f['gov/int-deg'] - ff0)
    f['ctl/init'] = 0; f['drive/hold-rpm'] = 0; f['ctl/scas-on'] = 1; f['ctl/cps-on'] = 1
    H.freeze(f, False)
    for a in ('lon', 'lat', 'ped'):
        f[AX[a]] = 0.0
    f.run()
    f['scas/theta-int-in'] = x[1] * B.X_LON - f['scas/lon-trim-in']; f['scas/phi-int-in'] = x[2] * B.X_LAT
    for ax in ('theta', 'phi', 'psi'):
        f['scas/%s-ref-rad' % ax] = f['attitude/%s-rad' % ax]
    dt = f['simulation/dt']
    out = {k: [] for k, _ in LOG}; out['t'] = []
    n_every = int(round(every / dt))
    for i in range(int(T / dt)):
        t = i * dt
        for te, ax, val in events:
            if abs(t - te) < dt / 2:
                f[AX[ax]] = val
        f.run()
        if i % n_every == 0:
            out['t'].append(t)
            for k, p in LOG:
                out[k].append(f[p])
    psi0 = out['psi'][0]
    out['psi'] = [((p - psi0 + 180) % 360) - 180 for p in out['psi']]
    return dict(V0=V, nacelle0=nacelle, events=events, trim=[float(a) for a in x], tcl=tcl, **out)


def hover():
    """one key press of each axis in turn from the hover: stick forward, then right and back, then a pedal"""
    return free(0.0, 90.0, [(4.0, 'lon', 0.05), (16.0, 'lat', 0.05), (26.0, 'lat', 0.0), (30.0, 'ped', 0.05), (36.0, 'ped', 0.0)], 44.0)


def conversion():
    """hands off: the nacelles commanded from 90 to 60 deg at 40 kt, the lever and stick left alone"""
    return free(40.0, 90.0, [(2.0, 'nac', 60.0)], 40.0)


def start(T=110.0, every=0.5):
    """the engines started on the ground, cold, the lever down, as the aircraft's autostart does: the rotor brake off
    and engine 1, engine 2 12 s later; the rotor turns up with the drive and the governor takes it at its reference"""
    import v22data as V
    f = H.load(hold_rpm=False)
    f['eng/init-running'] = 0
    f['/sim/freeze/fuel'] = 0
    f['gear/gear-cmd-norm'] = 1.0; f['gear/gear-pos-norm'] = 1.0
    f[AX['tcl']] = 0.0
    f['ic/h-sl-ft'] = 30; f['ic/terrain-elevation-ft'] = 0; f['ic/h-agl-ft'] = 0.0; f['ic/vt-kts'] = 0
    f.run_ic()
    dt = f['simulation/dt']
    for _ in range(int(2 / dt)):
        f.run()
    out = {k: [] for k in ('t', 'spool0', 'spool1', 'nr', 'gov', 'th75', 'hp')}
    for i in range(int(T / dt)):
        t = i * dt
        if abs(t - 0.0) < dt / 2:
            f['/controls/rotor/brake'] = 0; f['/controls/engines/engine[0]/cutoff'] = 0
        if abs(t - 12.0) < dt / 2:
            f['/controls/engines/engine[1]/cutoff'] = 0
        f.run()
        if i % int(round(every / dt)) == 0:
            for k, p in (('spool0', 'eng[0]/spool'), ('spool1', 'eng[1]/spool'), ('nr', 'drive/nr-pct'), ('gov', 'gov/engaged'),
                         ('th75', 'ctl/theta75-deg')):
                out[k].append(f[p])
            out['hp'].append(f['eng[0]/power-hp'] + f['eng[1]/power-hp'])
            out['t'].append(t)
    return out


if __name__ == '__main__':
    what = sys.argv[1] if len(sys.argv) > 1 else 'hover'
    r = {'hover': hover, 'conversion': conversion, 'start': start}[what]()
    keys = [k for k in r if isinstance(r[k], list) and len(r[k]) == len(r['t']) and k != 't']
    for i in range(0, len(r['t']), 20):
        print('t %5.1f  ' % r['t'][i] + '  '.join('%s %.2f' % (k, r[k][i]) for k in keys))
