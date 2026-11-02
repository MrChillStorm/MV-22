#!/usr/bin/env python3
"""The crew alerts (alerts.py) in the flights meant to raise them, offline: each scenario from a trim, its events, and
every alert's first time and how long it stood (alertcheck.py's other half: there the report's flights must not raise
them). Run genv22.py root/aircraft/mv22b first.

    python3 alertscenarios.py [name ...]"""
import sys, math
from multiprocessing import Pool
import hoverhold as HH
import alerts as AL
import v22data as U

IDS = [a['id'] for a in AL.table(U)]
LEVER, NAC, ELEV, CUT = '/controls/engines/engine[0]/throttle', '/controls/flight/nacelle-cmd-deg', 'fcs/elevator-cmd-norm', \
    '/controls/engines/engine[%d]/cutoff'


def fly(V, nac, T, events, gear=0.0, cargo=None, alt=1000.0, ramp=None, ground=False):
    """from a trim at V KCAS with the nacelles at nac, gear (0 up, 1 down); events (t, property, value); ramp: (t0,
    property, rate per s, floor) a steady change from t0; ground: the terrain at sea level (the trims' is far below).
    Every alert's first time and seconds, and the flight then"""
    f = HH.start(V, nac, alt=alt)
    if ground:
        f['position/terrain-elevation-asl-ft'] = 0.0
    f['gear/gear-cmd-norm'] = gear; f['gear/gear-pos-norm'] = gear
    if cargo is not None:
        f['inertia/pointmass-weight-lbs[%d]' % U.CARGO_PM] = cargo
    dt = f['simulation/dt']
    first, secs, at = {}, {}, {}
    v0 = None
    for i in range(int(T / dt)):
        t = i * dt
        for te, prop, val in events:
            if abs(t - te) < dt / 2:
                f[prop] = val
        if ramp is not None and t >= ramp[0]:
            if v0 is None:
                v0 = f[ramp[1]]
            f[ramp[1]] = max(ramp[3], v0 - ramp[2] * (t - ramp[0])) if ramp[2] > 0 else min(ramp[3], v0 - ramp[2] * (t - ramp[0]))
        f.run()
        if i % 48 == 0:
            for a in IDS:
                if f['alert/' + a] > 0.5:
                    secs[a] = secs.get(a, 0.0) + 0.1
                    if a not in first:
                        first[a] = t
                        at[a] = 'agl %.0f ft, %.0f KCAS, %.0f ft/min, nacelle %.0f, Nr %.1f, flap %.1f deg' % (
                            f['position/h-agl-ft'], f['velocities/vc-kts'], f['velocities/h-dot-fps'] * 60, f['ctl/nacelle-deg'],
                            f['drive/nr-pct'], f['alert/flap-deg'])
            if f['position/h-agl-ft'] < 5:
                break
    return {a: (first[a], secs[a], at[a]) for a in first}


SCENARIOS = {
    # the lever drawn back from a 10 kt hover at 0.02 per s: SINK RATE ahead of the vortex ring state (vrswarn.py)
    'sink-rate': lambda: fly(10.0, 85.0, 40.0, [], ramp=(0.0, LEVER, 0.02, 0.0)),
    # airplane mode at 220 kt, 1500 ft above the ground, the stick pushed a little: PULL UP before the ground
    'pull-up': lambda: fly(220.0, 0.0, 60.0, [(2.0, ELEV, 0.12)], alt=1500.0, ground=True),
    # both engines cut in a hover: DUAL ENG OUT, the rotor speed decaying (ROTOR RPM)
    'rotor-rpm': lambda: fly(0.0, 90.0, 20.0, [(2.0, CUT % 0, 1), (2.0, CUT % 1, 1)]),
    # airplane mode at 200 kt, engine 1 cut, then engine 2: ENG 1 OUT, then DUAL ENG OUT collecting it
    'eng1-out': lambda: fly(200.0, 0.0, 12.0, [(2.0, CUT % 0, 1), (8.0, CUT % 1, 1)]),
    # engine 2 alone: ENG 2 OUT
    'eng2-out': lambda: fly(200.0, 0.0, 8.0, [(2.0, CUT % 1, 1)]),
    # a hover with the stick held hard over: the gimbals' flapping (FLAPPING HIGH, FLAPPING)
    'flapping': lambda: fly(0.0, 90.0, 15.0, [(2.0, 'fcs/aileron-cmd-norm', 1.0)]),
    # descending at 40 kt from 260 ft above the ground, the gear up: WHEELS under 200 ft
    'wheels': lambda: fly(40.0, 80.0, 40.0, [], alt=260.0, ramp=(0.0, LEVER, 0.01, 0.0), ground=True),
    # 52 000 lb at 8000 ft, past its hover ceiling, slowing with the nacelles up: HOVER PWR
    'hover-pwr': lambda: fly(50.0, 80.0, 30.0, [(1.0, ELEV, -0.15)], alt=8000.0,
                             cargo=U.CARGO_REF + 4500.0),
    # airplane mode at 160 kt, the lever to idle and the nose up, the conversion protection off (the nacelles held at
    # 0): AIRSPEED LOW below the corridor's 125 kt
    'airspeed-low': lambda: fly(160.0, 0.0, 60.0, [(1.0, LEVER, 0.0), (1.0, 'ctl/cps-on', 0), (1.0, ELEV, -0.2)]),
}


def run(name):
    return name, SCENARIOS[name]()


if __name__ == '__main__':
    names = sys.argv[1:] or list(SCENARIOS)
    with Pool(len(names)) as pool:
        for name, res in pool.imap(run, names):
            print('%-13s %s' % (name, '\n              '.join('%-12s from %5.1f s for %5.1f s (%s)' % (a, *res[a])
                                                            for a in sorted(res, key=lambda a: res[a][0])) or '-'), flush=True)
