#!/usr/bin/env python3
"""The autopilot (genv22.autopilot), offline, coupled, from trims at the reference loading, 1000 ft: an altitude
change at a held speed, a heading change, a deceleration through the conversion with AUTO NAC at a held altitude,
the acceleration back, a slow turn.

  python3 aptest.py [alt | hdg | decel | accel | slow ...] | engage | instart"""
import sys, math, json, os
import numpy as np
import hoverhold as HH

AF, ST = '/instrumentation/mv22b/afcs/', '/autopilot/settings/'
LOG = [('vc', 'velocities/vc-kts'), ('h', 'position/h-sl-ft'), ('hdot', 'velocities/h-dot-fps'), ('theta', 'attitude/theta-deg'),
       ('phi', 'attitude/phi-deg'), ('psi', 'attitude/psi-deg'), ('nac', 'ctl/nacelle-deg'), ('tcl', '/controls/engines/engine[0]/throttle'),
       ('vref', 'ap/v-ref-kt'), ('on', 'ap/on'), ('nr', 'drive/nr-pct'), ('beta', 'aero/beta-deg'), ('q', 'velocities/q-rad_sec')]


def fly(V, nacelle, modes, T, events=(), every=0.2):
    """from a trim at V KCAS and the nacelle angle, coupled with modes {hdg, alt, spd: target}, auto-nac if 'nac'"""
    f = HH.start(V, nacelle)
    f[ST + 'heading-bug-deg'] = 0.0
    f[ST + 'target-altitude-ft'] = f['position/h-sl-ft']
    f[ST + 'target-speed-kt'] = f['velocities/vc-kts']
    for m in ('hdg', 'alt', 'spd'):
        if m in modes:
            f[AF + m + '-hold'] = 1
    if 'nac' in modes:
        f[AF + 'auto-nac'] = 1
    f[AF + 'cpld'] = 1
    dt = f['simulation/dt']
    out = {k: [] for k, _ in LOG}; out['t'] = []
    for i in range(int(T / dt)):
        t = i * dt
        for te, prop, val in events:
            if abs(t - te) < dt / 2:
                f[prop] = val
        f.run()
        if i % int(round(every / dt)) == 0:
            out['t'].append(t)
            for k, prop in LOG:
                out[k].append(f[prop])
    return out


def summary(name, r, t0=2.0):
    t = np.array(r['t']); i = t >= t0
    g = lambda k: np.array(r[k])[i]
    print('%-6s vc %5.1f..%5.1f (end %5.1f, ref %5.1f) h %6.0f..%6.0f (end %6.0f) pitch %5.1f..%4.1f bank %5.1f..%4.1f heading end %5.1f '
          'nacelle %4.1f..%4.1f lever %.2f..%.2f Nr %5.1f..%5.1f beta %5.1f..%4.1f coupled %d' % (
              name, g('vc').min(), g('vc').max(), g('vc')[-1], g('vref')[-1], g('h').min(), g('h').max(), g('h')[-1], g('theta').min(), g('theta').max(),
              g('phi').min(), g('phi').max(), g('psi')[-1], g('nac').min(), g('nac').max(), g('tcl').min(), g('tcl').max(), g('nr').min(), g('nr').max(),
              g('beta').min(), g('beta').max(), int(g('on')[-1])), flush=True)


def alt():
    return fly(200.0, 0.0, ('alt', 'spd'), 90.0, [(2.0, ST + 'target-altitude-ft', 2000.0)])


def hdg():
    return fly(200.0, 0.0, ('hdg', 'alt', 'spd'), 60.0, [(2.0, ST + 'heading-bug-deg', 90.0)])


def decel():
    return fly(220.0, 0.0, ('alt', 'spd', 'nac'), 140.0, [(2.0, ST + 'target-speed-kt', 100.0)])


def accel():
    return fly(100.0, 80.0, ('alt', 'spd', 'nac'), 120.0, [(2.0, ST + 'target-speed-kt', 220.0)])


def slow():
    return fly(100.0, 80.0, ('hdg', 'alt', 'spd', 'nac'), 90.0, [(2.0, ST + 'target-speed-kt', 50.0), (45.0, ST + 'heading-bug-deg', 90.0)])


def engage(T=40.0, every=0.1):
    """the autopilot engaged off its trim, as the capture flight engaged it in FlightGear after an in-air start at
    120 kt (the lever's 0.65 above the level's 0.54 at its loading): from the trim at 120 kt and 70 deg, 1500 ft, the
    lever at 0.85 and the nose pulled up 1.5 s, let go 9 s, climbing 32 ft/s and 8 kt slow; then HDG, ALT and SPD
    HOLD at 1500 ft and 120 kt, AUTO NAC and CPLD"""
    f = HH.start(120.0, 70.0, alt=1500.0)
    dt, te = f['simulation/dt'], 9.0
    f['/controls/engines/engine[0]/throttle'] = 0.85
    out = {k: [] for k, _ in LOG}; out['t'] = []
    for i in range(int((te + T) / dt)):
        t = i * dt
        f['fcs/elevator-cmd-norm'] = -0.28 if t < 1.5 else 0.0
        if abs(t - te) < dt / 2:
            f[ST + 'heading-bug-deg'] = 0.0; f[ST + 'target-altitude-ft'] = 1500.0; f[ST + 'target-speed-kt'] = 120.0
            for m in ('hdg-hold', 'alt-hold', 'spd-hold', 'auto-nac', 'cpld'):
                f[AF + m] = 1
        f.run()
        if t >= te - 1 and i % int(round(every / dt)) == 0:
            out['t'].append(t - te)
            for k, prop in LOG:
                out[k].append(f[prop])
    return out


def engage_numbers(r):
    """the engagement's pitch: at it, its range after, its largest reversal (at each turn, where its rate changes
    sign, the lesser of the moves into it and out of it: how far it came back); the vertical acceleration's range, g"""
    t, th, hd = (np.array(r[k]) for k in ('t', 'theta', 'hdot'))
    i = t >= 0
    th = th[i]
    d = np.diff(th)
    p = [th[0]] + [th[j + 1] for j in range(len(d) - 1) if d[j] * d[j + 1] < 0] + [th[-1]]
    acc = np.gradient(hd, t)[i] / 32.174
    return dict(theta0=float(th[0]), hdot0=float(hd[i][0]), vc0=float(np.array(r['vc'])[i][0]), theta_lo=float(th.min()), theta_hi=float(th.max()),
                swing=float(max([min(abs(b - a), abs(c - b)) for a, b, c in zip(p, p[1:], p[2:])] or [0.0])), g_lo=float(acc.min()), g_hi=float(acc.max()))


def in_air_start(kcas, gear=1.0, alt=1500.0, cargo=0.0, fuel=0.5, T=30.0, every=0.1, pitch=0.0):
    """FlightGear's in-air start, as the flight model sees it: its loading (by default no cargo, the tanks half full),
    its preset off the ground, the airspeed and altitude set, the nose at pitch (deg) at no angle of attack (FlightGear's
    own start: 0.42 deg up there, climbing on it), the gear down unless raised, the controls centred, the augmentation
    on; T s hands off (the first START_HOLD_S of them the flight model's own trim)"""
    import v22data as V
    f = HH.H.load(hold_rpm=False)
    f['inertia/pointmass-weight-lbs[%d]' % V.CARGO_PM] = cargo
    for i, t in enumerate(V.TANKS):
        f['propulsion/tank[%d]/contents-lbs' % i] = t[1] * V.FUEL_LB_GAL * fuel
    f['gear/gear-cmd-norm'] = gear; f['gear/gear-pos-norm'] = gear
    f['ctl/scas-on'] = 1; f['ctl/cps-on'] = 1
    f['/sim/presets/onground'] = 0
    f['ic/terrain-elevation-ft'] = 0.0; f['ic/h-sl-ft'] = alt; f['ic/vc-kts'] = kcas
    f['ic/psi-true-deg'] = 0.0; f['ic/phi-deg'] = 0.0
    f['ic/alpha-deg'] = 0.0; f['ic/gamma-deg'] = pitch
    f.run_ic()
    dt = f['simulation/dt']
    out = {k: [] for k, _ in LOG}; out['t'] = []
    for i in range(int(T / dt)):
        f.run()
        if i % int(round(every / dt)) == 0:
            out['t'].append(i * dt)
            for k, prop in LOG:
                out[k].append(f[prop])
    out['weight'] = f['inertia/weight-lbs']
    return out


def in_air_numbers(r, t0=6.0):
    """after t0 s (the start's own trim, let go at START_HOLD_S[-1]): the vertical speed's range, ft/s; the height's and
    the airspeed's change to the end; the pitch's range; the lever"""
    t = np.array(r['t']); i = t >= t0
    g = lambda k: np.array(r[k])[i]
    return dict(hdot_lo=float(g('hdot').min()), hdot_hi=float(g('hdot').max()), dh=float(g('h')[-1] - g('h')[0]),
                dv=float(g('vc')[-1] - r['vc'][0]), theta_lo=float(g('theta').min()), theta_hi=float(g('theta').max()),
                lever=float(r['tcl'][-1]), nac=float(r['nac'][-1]), weight=r['weight'])


def bank_limit(kcas):
    """the autopilot's bank limit at an airspeed (genv22.autopilot's table; autopilot.nas's bank_limit)"""
    return float(np.interp(kcas, [40, 80, 150], [15, 20, 25]))


class Gps:
    """FlightGear's GPS in its leg mode, for the harness: the legs between waypoints (ft east, north of the start),
    the leg's true course, the cross-track error (nm, + left of the leg), the bearing to the waypoint; and
    autopilot.nas's turns between legs (every 0.25 s: where an arc at 80 % of the bank limit joins them, the arc
    set for the flight model and the route moved on), mirrored. direct: the first leg flown straight to its
    waypoint, as one activated in flight is"""
    TURNS_S = 0.25

    def __init__(s, f, wps, direct=False):
        s.f, s.wps, s.i, s.direct, s.t_next = f, wps, 1, direct, 0.0
        s.la0, s.lo0 = f['position/lat-geod-rad'], f['position/long-gc-rad']
        f['/autopilot/route-manager/active'] = 1

    def update(s, t):
        f = s.f
        x = (f['position/long-gc-rad'] - s.lo0) * HH.R_E * math.cos(s.la0); y = (f['position/lat-geod-rad'] - s.la0) * HH.R_E
        (ax, ay), (bx, by) = s.wps[s.i - 1], s.wps[s.i]
        crs = math.atan2(bx - ax, by - ay)
        brg = math.atan2(bx - x, by - y)
        f['/instrumentation/gps/wp/leg-true-course-deg'] = math.degrees(crs) % 360
        f['/instrumentation/gps/wp/wp[1]/course-error-nm'] = (-(x - ax) * math.cos(crs) + (y - ay) * math.sin(crs)) / 6076.1
        f['/instrumentation/gps/wp/wp[1]/bearing-true-deg'] = math.degrees(brg) % 360
        # (autopilot.nas: the route's distance still to go to its last point)
        along = math.hypot(bx - x, by - y) if (s.direct and s.i == 1) else max(0.0, (bx - x) * math.sin(crs) + (by - y) * math.cos(crs))
        f['ap/route-togo-ft'] = along + sum(math.hypot(s.wps[j + 1][0] - s.wps[j][0], s.wps[j + 1][1] - s.wps[j][1]) for j in range(s.i, len(s.wps) - 1))
        if t < s.t_next:
            return x, y
        s.t_next = t + s.TURNS_S
        direct = s.direct and s.i == 1 and f['ap/nav'] > 0.5
        f['ap/direct'] = 1 if direct else 0
        if s.i + 1 < len(s.wps):
            if f['ap/nav'] < 0.5 or f['ap/arc-on'] > 0.5:
                return x, y
            c1 = brg if direct else crs
            (cx, cy) = s.wps[s.i + 1]
            c2 = math.atan2(cx - bx, cy - by)
            d = math.degrees((c2 - c1 + math.pi) % (2 * math.pi) - math.pi)
            trk = math.atan2(f['velocities/v-east-fps'], f['velocities/v-north-fps'])
            if not 1 <= abs(d) <= 150 or abs((trk - c1 + math.pi) % (2 * math.pi) - math.pi) > math.radians(15):
                return x, y                                     # (established on the leg first)
            vg = f['velocities/vg-fps']
            phi = 0.8 * bank_limit(f['velocities/vc-kts'])
            r = max(100.0, vg * vg / (32.174 * math.tan(math.radians(phi))))
            k = math.tan(math.radians(abs(d) / 2))
            roll = vg * (phi / 5 / 2 + f['ap/roll-lag-s'] + s.TURNS_S / 2)
            along = (bx - x) * math.sin(c1) + (by - y) * math.cos(c1)
            if along > r * k + roll:
                return x, y
            r_full = vg * vg / (32.174 * math.tan(math.radians(phi / 0.8)))
            r = max(r_full, min(r, (along - roll) / k))
            tt = r * k
            dr = 1 if d > 0 else -1
            ox = bx - tt * math.sin(c1) + r * math.sin(c1 + dr * math.pi / 2)
            oy = by - tt * math.cos(c1) + r * math.cos(c1 + dr * math.pi / 2)
            f['ap/arc-lat-deg'] = math.degrees(s.la0 + oy / HH.R_E)
            f['ap/arc-lon-deg'] = math.degrees(s.lo0 + ox / (HH.R_E * math.cos(s.la0)))
            f['ap/arc-r-ft'] = r; f['ap/arc-dir'] = dr; f['ap/arc-end-deg'] = math.degrees(c2) % 360; f['ap/arc-on'] = 1
            s.i += 1
        elif (bx - x) * math.sin(crs) + (by - y) * math.cos(crs) < 0:
            f['/autopilot/route-manager/active'] = 0
        return x, y


def lnav(V=200.0, nacelle=0.0, T=420.0, wps=None, direct=False):
    """from a trim heading north, a route: 2 nm east of the start, then 5 nm north, east, south (90 deg turns)"""
    f = HH.start(V, nacelle)
    nm = 6076.1
    g = Gps(f, wps or [(0, 0), (2 * nm, 3 * nm), (2 * nm, 8 * nm), (7 * nm, 8 * nm), (7 * nm, 3 * nm)], direct)
    f[ST + 'target-altitude-ft'] = f['position/h-sl-ft']; f[ST + 'target-speed-kt'] = f['velocities/vc-kts']
    for m in ('wypt', 'alt-hold', 'spd-hold', 'auto-nac', 'cpld'):
        f[AF + m] = 1
    dt = f['simulation/dt']
    out = {k: [] for k in ('t', 'x', 'y', 'xte', 'phi', 'h', 'vc', 'leg', 'terr', 'arc')}
    for i in range(int(T / dt)):
        if i % 4 == 0:
            x, y = g.update(i * dt)
        f.run()
        if i % int(round(0.5 / dt)) == 0:
            for k, v in (('t', i * dt), ('x', x), ('y', y), ('xte', f['/instrumentation/gps/wp/wp[1]/course-error-nm'] * nm), ('phi', f['attitude/phi-deg']),
                         ('h', f['position/h-sl-ft']), ('vc', f['velocities/vc-kts']), ('leg', g.i), ('arc', f['ap/arc-on']),
                         ('terr', ((math.degrees(math.atan2(f['velocities/v-east-fps'], f['velocities/v-north-fps'])) - f['/instrumentation/gps/wp/leg-true-course-deg'] + 180) % 360) - 180)):
                out[k].append(v)
    return out


def approach(angle=3.0, land=True, wind_kt=0.0, V=140.0, nacelle=60.0, T=330.0, off_ft=2000.0, dist_nm=6.0, every=0.5):
    """APPR from a trim at V, 1000 ft over ground at sea level: a line dist_nm ahead (north) and off_ft to the right, its
    path at angle; to a hover there, or a landing (APPR twice). The wind from ahead"""
    f = HH.start(V, nacelle)
    f['position/terrain-elevation-asl-ft'] = 0.0
    la0, lo0 = f['position/lat-geod-rad'], f['position/long-gc-rad']
    nm = 6076.1
    f['ap/line-lat-deg'] = math.degrees(la0 + dist_nm * nm / HH.R_E)
    f['ap/line-lon-deg'] = math.degrees(lo0 + off_ft / (HH.R_E * math.cos(la0)))
    f['ap/line-course-deg'] = 0.0; f['ap/line-angle-deg'] = angle; f['ap/line-elev-ft'] = 0.0; f['ap/line-gs'] = 1; f['ap/line-valid'] = 1
    f[ST + 'target-altitude-ft'] = f['position/h-sl-ft']; f[ST + 'target-speed-kt'] = V
    for m in ('alt-hold', 'spd-hold', 'auto-nac', 'appr', 'cpld'):
        f[AF + m] = 1
    f['ap/app-land'] = 1 if land else 0
    if land:
        f['gear/gear-cmd-norm'] = 1.0; f['gear/gear-pos-norm'] = 1.0          # down for the landing
    if wind_kt:
        f['atmosphere/wind-north-fps'] = -wind_kt * 1.6878
    dt = f['simulation/dt']
    keys = ('t', 'd', 'xte', 'dh', 'vc', 'gs', 'nac', 'theta', 'phi', 'state', 'hstate', 'agl', 'hdot', 'n', 'e', 'wow', 'tcl', 'gp')
    out = {k: [] for k in keys}
    for i in range(int(T / dt)):
        f.run()
        if i % int(round(every / dt)) == 0:
            wow = sum(f['gear/unit[%d]/WOW' % j] for j in range(3))
            for k, v in zip(keys, (i * dt, f['ap/app-dist-ft'], f['ap/app-xte-ft'], f['position/h-sl-ft'] - f['ap/app-path-ft'], f['velocities/vc-kts'],
                                   f['velocities/vg-fps'] / 1.6878, f['ctl/nacelle-deg'], f['attitude/theta-deg'], f['attitude/phi-deg'], f['ap/app-state'],
                                   f['hh/state'], f['position/h-agl-ft'], f['velocities/h-dot-fps'], (f['position/lat-geod-rad'] - la0) * HH.R_E - dist_nm * nm,
                                   (f['position/long-gc-rad'] - lo0) * HH.R_E * math.cos(la0) - off_ft, wow, f['/controls/engines/engine[0]/throttle'],
                                   f['ap/gp'])):
                out[k].append(v)
    out['f'] = f
    return out


def go_around(f):
    """GA as autopilot.nas sets it: the approach and the hover hold off; the heading, 1000 ft above (or the ALT knob's
    higher altitude), at least 80 kt, AUTO NAC (the coupling as it was)"""
    for m in ('appr', 'vor-ils', 'hvr-hold'):
        f[AF + m] = 0
    f['ap/app-land'] = 0; f['hh/engage-cmd'] = 0
    f[ST + 'heading-bug-deg'] = f['attitude/psi-deg'] - f['/environment/magnetic-variation-deg']
    h = f['position/h-sl-ft']
    if f[ST + 'target-altitude-ft'] < h + 200:
        f[ST + 'target-altitude-ft'] = round(h / 100) * 100 + 1000
    f[ST + 'target-speed-kt'] = max(f[ST + 'target-speed-kt'], 80)
    for m in ('hdg-hold', 'alt-hold', 'spd-hold', 'auto-nac', 'ga'):
        f[AF + m] = 1


def ga(when, T=90.0):
    """a go-around: from the 3 deg approach at a distance to go (ft), or from its hover (when = 'hover'): the height
    lost, the climb and speed 60 s on, the attitudes"""
    r0 = approach(3.0, False, 0.0, T=1)                   # (the set-up only)
    f = r0['f']
    dt = f['simulation/dt']
    t, done = 0.0, False
    while t < 330:
        f.run(); t += dt
        if not done and ((when == 'hover' and f['hh/state'] == 2 and f['ap/app-state'] == 3) or
                         (when != 'hover' and f['ap/app-state'] == 1 and f['ap/gp'] and f['ap/app-dist-ft'] < when)):
            done, t0, h0 = True, t, f['position/h-agl-ft']
            go_around(f)
            out = {k: [] for k in ('t', 'h', 'vc', 'theta', 'phi', 'nac', 'tcl', 'hdot')}
        if done:
            if abs(f[ST + 'target-altitude-ft'] - f['position/h-sl-ft']) < 100:
                f[AF + 'ga'] = 0                         # (autopilot.nas: GA done within 100 ft of the altitude)
            if int(round(t / dt)) % int(round(0.5 / dt)) == 0:
                for k, prop in (('t', None), ('h', 'position/h-agl-ft'), ('vc', 'velocities/vc-kts'), ('theta', 'attitude/theta-deg'),
                                ('phi', 'attitude/phi-deg'), ('nac', 'ctl/nacelle-deg'), ('tcl', '/controls/engines/engine[0]/throttle'),
                                ('hdot', 'velocities/h-dot-fps')):
                    out[k].append(t - t0 if prop is None else f[prop])
            if t - t0 > T:
                break
    if not done:
        return None
    h = np.array(out['h'])
    return dict(when=when, h0=h0, h_min=float(h.min()), h30=float(h[60]), vc30=out['vc'][60], T=T, h_end=float(h[-1]), vc_end=out['vc'][-1],
                climb_fpm=out['hdot'][-1] * 60, nac_end=out['nac'][-1], tcl=(min(out['tcl']), max(out['tcl'])),
                theta=(min(out['theta']), max(out['theta'])), phi=(min(out['phi']), max(out['phi'])))


def ga_summary(g):
    if g is None:
        return 'GA: never reached'
    return ('GA %s: from %4.0f ft, lowest %4.0f ft; at 30 s %4.0f ft %4.1f kt, at %2.0f s %4.0f ft %4.1f kt climbing %4.0f fpm, nacelle %4.1f, '
            'lever %.2f..%.2f | pitch %5.1f..%4.1f bank %5.1f..%4.1f' % (
                g['when'], g['h0'], g['h_min'], g['h30'], g['vc30'], g['T'], g['h_end'], g['vc_end'], g['climb_fpm'], g['nac_end'],
                g['tcl'][0], g['tcl'][1], g['theta'][0], g['theta'][1], g['phi'][0], g['phi'][1]))


def roll_delay(speeds=(60.0, 80.0, 100.0, 120.0, 150.0, 200.0, 250.0)):
    """the autopilot's roll into a bank as a delay at each speed (genv22.roll_delay): HDG HOLD coupled along AUTO
    NAC's schedule, the bug 170 deg right, the bank reference ramping at its rate limit to the bank limit phi_a and held
    there; over the time a 90 deg turn takes at phi_a, int(tan phi_a - tan phi) dt / tan phi_a, the heading lost
    against a bank reached at once, as a delay; less the ramp's half time (phi_a / 2 at the rate limit), the
    augmentation's own. (Rolling out along a ramp mirrors rolling in, phi_out = phi_a - phi_in, for a linear loop:
    the same delay)"""
    from genv22 import autonac_schedule
    sk, sn = autonac_schedule()
    out = []
    for V in speeds:
        f = HH.start(V, float(np.interp(V, sk, sn)))
        f[ST + 'target-altitude-ft'] = f['position/h-sl-ft']; f[ST + 'target-speed-kt'] = V; f[ST + 'heading-bug-deg'] = 0.0
        for m in ('hdg-hold', 'alt-hold', 'spd-hold', 'auto-nac', 'cpld'):
            f[AF + m] = 1
        dt = f['simulation/dt']
        for i in range(int(3 / dt)):
            f.run()
        f[ST + 'heading-bug-deg'] = (f['attitude/psi-deg'] - f['/environment/magnetic-variation-deg'] + 170) % 360
        pa = math.radians(bank_limit(f['velocities/vc-kts']))
        T90 = (math.pi / 2) * f['velocities/vt-fps'] / (32.174 * math.tan(pa))
        t, ph = [], []
        for i in range(int(T90 / dt)):
            f.run()
            if i % 4 == 0:
                t.append(i * dt); ph.append(f['attitude/phi-rad'])
        tau = float(np.trapezoid(math.tan(pa) - np.tan(np.array(ph)), np.array(t)) / math.tan(pa))
        out.append(dict(kcas=V, bank_deg=math.degrees(pa), tau_eq_s=tau, tau_loop_s=tau - math.degrees(pa) / (2 * f['ap/phi-rate-dps']),
                        overshoot_deg=math.degrees(max(ph) - pa)))
    return out


def rate_per_inch(axis='roll', speeds=(40.0, 60.0, 80.0, 100.0, 120.0, 150.0, 200.0, 250.0), step=0.03):
    """the airframe's roll (or pitch) rate per inch of stick at each speed along AUTO NAC's schedule (genv22.roll_ff,
    pitch_ff): from the trim, the augmentation's loop in that axis off (its gains naught), a small step of stick
    through its direct path (right; aft, a pull-up); the rate 1.5-2.5 s on in roll (the roll mode's 63 % comes at
    0.8-1.0 s), 2-6 s on in pitch (the steady pull-up's: the bare short period rings at about 3 s, its average), rad/s
    per inch"""
    from genv22 import autonac_schedule
    from data import bell301 as B
    sk, sn = autonac_schedule()
    gains, cmd, x, rate = {'roll': (('scas/kp', 'scas/kphi', 'scas/ki-phi'), 'fcs/aileron-cmd-norm', B.X_LAT, 'velocities/p-rad_sec'),
                           'pitch': (('scas/kq', 'scas/ktheta', 'scas/ki-theta'), 'fcs/elevator-cmd-norm', B.X_LON, 'velocities/q-rad_sec')}[axis]
    sign = 1 if axis == 'roll' else -1
    out = []
    for V in speeds:
        f = HH.start(V, float(np.interp(V, sk, sn)))
        for k in gains:
            f[k] = 0.0
        dt = f['simulation/dt']
        for i in range(int(0.5 / dt)):
            f.run()
        inches = step * x * f['scas/stick-ff']
        f[cmd] = sign * step
        t, rr = [], []
        t0, t1 = (1.5, 2.5) if axis == 'roll' else (2.0, 6.0)
        for i in range(int(t1 / dt)):
            f.run()
            if i % 4 == 0:
                t.append(i * dt); rr.append(f[rate])
        t, rr = np.array(t), np.array(rr)
        out.append(dict(kcas=V, per_in=float(rr[t > t0].mean() / inches), peak=float(rr[t < 2.0].max() / max(1e-6, rr[t > t0].mean()))))
    return out


def route_approach(V=120.0, nacelle=70.0, land=True, T=420.0):
    """WYPT on a route whose last point is a landing zone, APPR armed with it as autopilot.nas arms it (its line along
    the last leg in a calm, at 6 deg): from a trim heading north at 1000 ft, legs 3 nm north then 3.6 nm north-east;
    the slowdown and descent along the route, the hover at its end, the landing"""
    f = HH.start(V, nacelle)
    f['position/terrain-elevation-asl-ft'] = 0.0
    nm = 6076.1
    wps = [(0, 0), (0, 3 * nm), (2 * nm, 6 * nm)]
    g = Gps(f, wps, direct=False)
    la0, lo0 = g.la0, g.lo0
    f['ap/line-lat-deg'] = math.degrees(la0 + wps[-1][1] / HH.R_E); f['ap/line-lon-deg'] = math.degrees(lo0 + wps[-1][0] / (HH.R_E * math.cos(la0)))
    f['ap/line-course-deg'] = math.degrees(math.atan2(wps[-1][0] - wps[-2][0], wps[-1][1] - wps[-2][1])) % 360
    f['ap/line-angle-deg'] = 6.0; f['ap/line-elev-ft'] = 0.0; f['ap/line-gs'] = 1; f['ap/line-valid'] = 1
    f[ST + 'target-altitude-ft'] = f['position/h-sl-ft']; f[ST + 'target-speed-kt'] = V
    for m in ('wypt', 'alt-hold', 'spd-hold', 'auto-nac', 'appr', 'cpld'):
        f[AF + m] = 1
    f['ap/app-land'] = 1 if land else 0
    if land:
        f['gear/gear-cmd-norm'] = 1.0; f['gear/gear-pos-norm'] = 1.0
    dt = f['simulation/dt']
    keys = ('t', 'd', 'xte', 'dh', 'vc', 'gs', 'nac', 'theta', 'phi', 'state', 'hstate', 'agl', 'hdot', 'n', 'e', 'wow', 'tcl', 'gp', 'route')
    out = {k: [] for k in keys}
    for i in range(int(T / dt)):
        if i % 4 == 0:
            x, y = g.update(i * dt)
        f.run()
        if i % int(round(0.5 / dt)) == 0:
            wow = sum(f['gear/unit[%d]/WOW' % j] for j in range(3))
            for k, v in zip(keys, (i * dt, f['ap/app-dist-ft'], f['ap/app-xte-ft'], f['position/h-sl-ft'] - f['ap/app-path-ft'], f['velocities/vc-kts'],
                                   f['velocities/vg-fps'] / 1.6878, f['ctl/nacelle-deg'], f['attitude/theta-deg'], f['attitude/phi-deg'], f['ap/app-state'],
                                   f['hh/state'], f['position/h-agl-ft'], f['velocities/h-dot-fps'], y - wps[-1][1], x - wps[-1][0], wow,
                                   f['/controls/engines/engine[0]/throttle'], f['ap/gp'], f['ap/app-route'])):
                out[k].append(v)
    out['f'] = f
    return out


TF_DESC = 2000 / 60.0


def tf_reach(v, t, v0, a1, a2, climb, m, hold=0.25):
    """autopilot.nas tf_reach: the most height (ft) the aircraft gains by t, the reference commanded v now: ramped from
    v0 at a1 and held to the next plan (hold s), then pulled up at a2 to the climb the power makes; the aircraft its
    model m, ((share, time constant s, its vertical speed ft/s) ...), first order lags in parallel, of the reference
    (in closed form along each stretch of it, the reference's speed linear there)"""
    t1 = abs(v - v0) / a1
    segs = [(min(t, t1), v0, a1 if v > v0 else -a1)]
    th = max(t1, hold)
    if t > t1:
        segs.append((min(t, th) - t1, v, 0.0))
    if t > th:
        s, t2 = t - th, max(0.0, climb - v) / a2
        segs.append((min(s, t2), v, a2))
        if s > t2:
            segs.append((s - t2, climb, 0.0))
    out = 0.0
    for w, tau, x in m:
        r = 0.0
        for d, vs, k in segs:
            c, e = x - vs + tau * k, math.exp(-d / tau)
            r += vs * d + k * d * d / 2 - tau * k * d + c * tau * (1 - e)
            x = vs + k * d - tau * k + c * e
        out += w * r
    return out


def tf_need(dh, t, v0, a1, a2, climb, m, hold=0.25):
    """autopilot.nas tf_need: the least vertical speed to command now that still gains dh by t (bisection)"""
    if tf_reach(-TF_DESC, t, v0, a1, a2, climb, m, hold) >= dh:
        return -TF_DESC
    if tf_reach(climb, t, v0, a1, a2, climb, m, hold) < dh:
        return climb
    lo, hi = -TF_DESC, climb
    for _ in range(12):
        mid = (lo + hi) / 2
        if tf_reach(mid, t, v0, a1, a2, climb, m, hold) >= dh:
            hi = mid
        else:
            lo = mid
    return hi


class TfPlan:
    """autopilot.nas tf(), mirrored: every 0.25 s the terrain along the track (curving at its rate) every 1.25 s out
    to 45 s, the track and 150 ft either side, and the last sweep's (4 plans') points at their time to go now; the
    reference's command the most any point needs, for the aircraft's model of it to clear it (by then; a point
    nearer than 1/K that it cannot, K the height's tracking gain, 1/K s later, the command meanwhile not eased)"""
    N, T, W, S, DESC = 36, 45.0, 150.0, 0.25, TF_DESC

    def __init__(s, f, terrain, clr, frame=0.0):
        s.f, s.terrain, s.clr, s.t_next, s.trk, s.rate, s.past = f, terrain, clr, 0.0, None, 0.0, []
        s.frame, s.rng = frame, np.random.default_rng(1)       # (FlightGear's timer: up to a frame late)
        s.t_prev, s.hold = None, s.S

    def update(s, t, x, y):
        f = s.f
        if t < s.t_next:
            return
        s.t_next = t + s.S + s.rng.uniform(0.0, s.frame)
        dt = s.S if s.t_prev is None else t - s.t_prev
        s.t_prev, s.hold = t, max(s.S, dt)
        vn, ve = f['velocities/v-north-fps'], f['velocities/v-east-fps']
        trk = math.atan2(ve, vn)
        if s.trk is not None:
            s.rate += (math.atan2(math.sin(trk - s.trk), math.cos(trk - s.trk)) / dt - s.rate) * 0.5
        s.trk = trk
        vg = math.hypot(vn, ve)
        hw = s.terrain(x, y) + f['hh/h-wheels-ft'] + f['ap/tf-model-ft'] - f['position/h-sl-ft']
        v0 = f['ap/tf-cmd-fps']
        a = f['ap/tf-ride-g'] * 32.174
        climb = max(1.0, f['ap/tf-climb-cap-fps'])
        m = ((f['ap/tf-w'], f['ap/tf-tau1-s'], f['ap/tf-v1-fps']), (1 - f['ap/tf-w'], f['ap/tf-tau2-s'], f['ap/tf-v2-fps']))
        pts, px, py, h = [], x, y, trk
        step = vg * s.T / s.N
        for i in range(1, s.N + 1):
            hm = h + s.rate * s.T / s.N / 2
            px += step * math.sin(hm); py += step * math.cos(hm)
            h += s.rate * s.T / s.N
            pts.append((t + s.T * i / s.N, max(s.terrain(px + side * s.W * math.cos(h), py - side * s.W * math.sin(h)) for side in (-1, 0, 1))))
        s.past = s.past[-int(round(s.T / s.N / s.S)) + 1:] + [pts]
        tr = 1.0 / max(f['ap/tf-kh'], 0.05)
        v = -s.DESC
        for sweep in s.past:
            for ta, e in sweep:
                tt, dh = ta - t, e + s.clr - hw
                hd = s.hold
                if tt > 0 and tf_reach(v, tt, v0, a, a / 2, climb, m, hd) < dh:
                    if tt < tr and tf_reach(climb, tt, v0, a, a / 2, climb, m, hd) < dh:
                        v = max(v, v0)
                        tt += tr
                        if tf_reach(v, tt, v0, a, a / 2, climb, m, hd) >= dh:
                            continue
                    v = tf_need(dh, tt, v0, a, a / 2, climb, m, hd)
        f['ap/tf-hdot-fps'] = max(-s.DESC, min(climb, v))
        f['ap/tf-valid'] = 1


NM = 6076.1
TERRAIN = {
    'flat': lambda x, y: 0.0,
    'hills': lambda x, y: 250.0 * (1 - math.cos(2 * math.pi * max(0.0, y - 1 * NM) / (2 * NM))),
    'ridge': lambda x, y: float(np.interp(y, [0, 2 * NM, 2.5 * NM, 3 * NM, 3.5 * NM], [0, 0, 800, 800, 0])),
}


def tf_run(V=120.0, nacelle=70.0, name='hills', clr=200.0, ride=0.2, T=220.0, frame=0.0):
    """TF over a terrain profile northwards (TERRAIN), from a trim heading north at the clearance over its start:
    SPD HOLD at V, HDG HOLD, AUTO NAC, coupled"""
    terrain = TERRAIN[name]
    f = HH.start(V, nacelle, alt=terrain(0, 0) + clr + 8.0)
    la0, lo0 = f['position/lat-geod-rad'], f['position/long-gc-rad']
    f['position/terrain-elevation-asl-ft'] = terrain(0, 0)
    f[ST + 'target-agl-ft'] = clr; f[ST + 'target-speed-kt'] = V; f[ST + 'heading-bug-deg'] = 0.0
    f['ap/tf-ride-g'] = ride
    for m in ('tf', 'hdg-hold', 'spd-hold', 'auto-nac', 'cpld'):
        f[AF + m] = 1
    plan = TfPlan(f, terrain, clr, frame)
    dt = f['simulation/dt']
    keys = ('t', 'y', 'terr', 'hw', 'cmd', 'hdot', 'vc', 'theta', 'flyup', 'tcl', 'nac', 'cap')
    out = {k: [] for k in keys}
    for i in range(int(T / dt)):
        t = i * dt
        x = (f['position/long-gc-rad'] - lo0) * HH.R_E * math.cos(la0); y = (f['position/lat-geod-rad'] - la0) * HH.R_E
        f['position/terrain-elevation-asl-ft'] = terrain(x, y)
        plan.update(t, x, y)
        f.run()
        if i % int(round(0.1 / dt)) == 0:
            for k, v in zip(keys, (t, y, terrain(x, y), f['hh/h-wheels-ft'], f['ap/tf-cmd-fps'], f['velocities/h-dot-fps'], f['velocities/vc-kts'],
                                   f['attitude/theta-deg'], f['ap/tf-flyup'], f['/controls/engines/engine[0]/throttle'], f['ctl/nacelle-deg'],
                                   f['ap/tf-climb-cap-fps'])):
                out[k].append(v)
    return out


def tf_summary(r, clr):
    hw, hd, t = np.array(r['hw']), np.array(r['hdot']), np.array(r['t'])
    acc = np.gradient(hd, t) / 32.174
    w = t > 5
    return ('clearance %4.0f..%4.0f ft (set %d, mean %4.0f); vertical acceleration %+4.2f..%+4.2f g; vertical speed %+5.0f..%+5.0f fpm; '
            'fly-up %3.1f s; airspeed %4.0f..%4.0f kt; pitch %+5.1f..%+5.1f; lever %.2f..%.2f; climb limit %4.0f..%4.0f fpm' % (
                hw[w].min(), hw[w].max(), clr, hw[w].mean(), acc[w].min(), acc[w].max(), hd[w].min() * 60, hd[w].max() * 60,
                (np.array(r['flyup']) > 0.5).sum() * 0.1, min(r['vc']), max(r['vc']), min(r['theta']), max(r['theta']), min(r['tcl']), max(r['tcl']),
                min(r['cap']) * 60, max(r['cap']) * 60))


def climb_power(speeds=(40.0, 60.0, 80.0, 100.0, 120.0, 135.0, 150.0, 160.0, 170.0, 180.0, 200.0, 250.0), fpm=1000.0):
    """the power a climb takes at each speed along AUTO NAC's schedule (genv22.climb_power): trims level, climbing at
    fpm and climbing at full power (2 x the least of the engines' rating and their share of the drive's torque limit;
    a bisection on the flight path's angle), the same airspeed and nacelle angle; through them P = P_level + b h' + c
    h'^2 (h' ft/s), at the reference weight"""
    import harness as H
    from genv22 import autonac_schedule
    sk, sn = autonac_schedule()
    f = H.load()
    out = []
    for V in speeds:
        nac = float(np.interp(V, sk, sn))
        x0, r0, e0 = H.trim(f, V, alt_ft=1000, nacelle=nac, cas=True)
        pfull = 2 * min(f['eng/power-available-hp'], f['eng/xmsn-limit-hp'])
        vt, W = f['velocities/vt-fps'], f['inertia/weight-lbs']
        g = math.degrees(math.asin(fpm / 60 / vt))
        x1, r1, e1 = H.trim(f, V, alt_ft=1000, nacelle=nac, cas=True, gamma=g, x0=x0)
        h1 = vt * math.sin(math.radians(g))
        lo, hi = 0.0, 1.5 * math.degrees(math.asin(min(0.3, 550 * (pfull - e0[0]) / W / vt)))
        x, res, g2 = x0, 0.0, 0.0
        for _ in range(10):
            g2 = (lo + hi) / 2
            xt, r, e = H.trim(f, V, alt_ft=1000, nacelle=nac, cas=True, gamma=g2, x0=x)
            res = np.abs(r).max()
            if res > 0.02:                       # no trim there: take it as too steep
                hi = g2
                continue
            x = xt
            if abs(e[0] - pfull) < 10:
                break
            lo, hi = (g2, hi) if e[0] < pfull else (lo, g2)
        h2 = vt * math.sin(math.radians(g2))
        b, c = np.linalg.solve([[h1, h1 * h1], [h2, h2 * h2]], [e1[0] - e0[0], pfull - e0[0]])
        out.append(dict(kcas=V, b=float(b), c=float(c), weight=W, p_level=e0[0], p_1000=e1[0], p_full=pfull, climb_fpm=h2 * 60,
                        res=max(res, np.abs(r0).max(), np.abs(r1).max())))
    return out


def tf_lag(speeds=(60.0, 80.0, 100.0, 120.0, 135.0, 150.0, 160.0, 170.0, 180.0, 200.0, 250.0), T=25.0, step=10.0, pm=60.0):
    """the climb's response to TF's command at each speed along AUTO NAC's schedule (genv22.tf_lag), level, SPD HOLD,
    the height's tracking off: a step of step ft/s, y its fraction made. Its delay behind a ramp, int(1 - y) dt (exact
    for any linear response); and the gain K of the height's loop, K A(jw)/(jw), A the response's transform (y's
    derivative's), for pm deg of phase margin: where A's phase is pm - 90 deg, K = w/|A|"""
    from genv22 import autonac_schedule
    sk, sn = autonac_schedule()
    out = []
    for V in speeds:
        f = HH.start(V, float(np.interp(V, sk, sn)), alt=1000)
        f['position/terrain-elevation-asl-ft'] = 0.0
        f[ST + 'target-agl-ft'] = 1000; f[ST + 'target-speed-kt'] = V; f[ST + 'heading-bug-deg'] = 0.0
        for m in ('tf', 'hdg-hold', 'spd-hold', 'auto-nac', 'cpld'):
            f[AF + m] = 1
        f['ap/tf-valid'] = 1; f['ap/tf-hdot-fps'] = 0.0; f['ap/tf-track'] = 0
        dt = f['simulation/dt']
        for i in range(int(4 / dt)):
            f.run()
        f['ap/tf-ride-g'] = 100.0                       # (the command a step)
        h0 = f['velocities/h-dot-fps']
        f['ap/tf-hdot-fps'] = h0 + step
        t, y = [], []
        for i in range(int(T / dt)):
            f.run()
            t.append(i * dt); y.append((f['velocities/h-dot-fps'] - h0) / step)
        t, y = np.array(t), np.array(y)
        # the fit: 1 - y = w exp(-t/tau1) + (1 - w) exp(-t/tau2), least squares (w linear, the lags on a grid); w at
        # least 0, so that y is (1 - w e1 + (w - 1) e2 >= 1 - e1 for w > 1, the slow lag's share negative making an
        # overshoot): the height then rises with the command; tau2 at least 2 tau1, so that the fits are well
        # conditioned between the speeds (free, two lags 14 % apart with shares of +8.3 and -7.3, interpolated
        # towards 200 kt's, made an overshoot it hasn't; a third lag, in series with the quick one for the S-shaped
        # start, fitted closer but with shares of +1.6, +2.8 and -3.4 at 150 kt, and between that and 200 kt's the
        # model ran 87 ft from the aircraft)
        taus, best = np.geomspace(0.1, 40.0, 90), None
        for i1, t1 in enumerate(taus):
            e1 = np.exp(-t / t1)
            for t2 in taus[taus >= 2 * t1]:
                e2 = np.exp(-t / t2)
                dd = e1 - e2
                w = max(0.0, float(np.dot(1 - y - e2, dd) / np.dot(dd, dd)))
                res = float(np.sqrt(np.mean((1 - y - e2 - w * dd) ** 2)))
                if best is None or res < best[0]:
                    best = (res, w, float(t1), float(t2))
        g = np.gradient(y, t)
        w = np.linspace(0.01, 5.0, 2000)
        A = np.array([np.trapezoid(g * np.exp(-1j * wi * t), t) for wi in w])
        ph = np.degrees(np.unwrap(np.angle(A)))
        i = int(np.argmax(ph <= pm - 90.0))
        out.append(dict(kcas=V, lag=float(np.trapezoid(1 - y, t)), k_h=float(w[i] / abs(A[i])), w_c=float(w[i]), peak=float(y.max()),
                        w=best[1], tau1=best[2], tau2=best[3], res=best[0], t=t[::12].tolist(), y=y[::12].tolist()))
    return out


def tf_climb_check(speeds=(60.0, 80.0, 100.0, 120.0, 150.0, 200.0, 250.0), T=40.0):
    """the climb the flight model estimates TF can make (ap/tf-climb-cap-fps), level at each speed along AUTO NAC's
    schedule, against the energy's rate it then makes at full power (the climb and the speed's change, h' + V V'/g):
    TF commanding more than any, SPD HOLD, 10-20 s on"""
    from genv22 import autonac_schedule
    sk, sn = autonac_schedule()
    out = []
    for V in speeds:
        f = HH.start(V, float(np.interp(V, sk, sn)), alt=3000)
        f['position/terrain-elevation-asl-ft'] = 0.0
        f[ST + 'target-agl-ft'] = 1000; f[ST + 'target-speed-kt'] = V; f[ST + 'heading-bug-deg'] = 0.0
        for m in ('tf', 'hdg-hold', 'spd-hold', 'auto-nac', 'cpld'):
            f[AF + m] = 1
        f['ap/tf-valid'] = 1; f['ap/tf-hdot-fps'] = 0.0
        dt = f['simulation/dt']
        for i in range(int(4 / dt)):
            f.run()
        est = f['ap/tf-climb-cap-fps']
        f['ap/tf-hdot-fps'] = 100.0
        ps, hd, vc = [], [], []
        vt0 = f['velocities/vt-fps']
        for i in range(int(T / dt)):
            f.run()
            vt = f['velocities/vt-fps']
            if 10 < i * dt <= 20:                # the lever at its stop: the energy's rate, the speed's share too
                ps.append(f['velocities/h-dot-fps'] + vt * (vt - vt0) / dt / 32.174); hd.append(f['velocities/h-dot-fps']); vc.append(f['velocities/vc-kts'])
            vt0 = vt
        out.append(dict(kcas=V, est_fpm=est * 60, made_fpm=float(np.mean(ps)) * 60, climb_fpm=float(np.mean(hd)) * 60, vc=float(np.mean(vc)),
                        lever=f['/controls/engines/engine[0]/throttle']))
    return out


def approach_numbers(r):
    """an approach's (approach, route_approach) numbers: on the final, the cross-track error and the height off the path
    once on it; the handover, how far past the point it ran (ft), the hover, the touchdown"""
    t, st, d, xte, dh = (np.array(r[k]) for k in ('t', 'state', 'd', 'xte', 'dh'))
    gp = (st == 1) & (np.array(r['gp']) > 0.5) & (d > 1500)        # on the path once it is captured
    on_final = (st == 1) & (d < 15000) & (d > 1500)
    i2 = int(np.argmax(st >= 2)) if (st >= 2).any() else -1
    held = np.where((np.array(r['hstate']) == 2) & (st >= 3))[0]
    wow = np.where(np.array(r['wow']) > 0.5)[0]
    out = dict(xte_max=float(np.abs(xte[on_final]).max()), path=(float(dh[gp].min()), float(dh[gp].max())) if gp.any() else (0.0, 0.0),
               hand_t=float(t[i2]), hand_d=float(d[i2]), hand_vc=r['vc'][i2], taken=bool((st == 3).any()),
               past_ft=float(max(0.0, -d[st >= 1].min())) if (st >= 1).any() else 0.0,
               theta=(min(r['theta']), max(r['theta'])), phi=(min(r['phi']), max(r['phi'])))
    if len(held):
        h0 = held[0]
        out.update(hold_t=float(t[h0]), hold_d=math.hypot(r['n'][h0], r['e'][h0]), hold_h=r['agl'][h0])
    if len(wow):
        w0 = wow[0]
        out.update(down_t=float(t[w0]), down_d=math.hypot(r['n'][w0], r['e'][w0]), sink=-r['hdot'][max(0, w0 - 1)], end_state=r['state'][-1],
                   tcl_end=r['tcl'][-1])
    return out


def approach_summary(r):
    a = approach_numbers(r)
    out = 'final: xte max %4.0f ft, path %+4.0f..%+4.0f ft; handed over at %3.0f s (%4.0f ft to go, %4.1f kt), %3.0f ft past the point at most' % (
        a['xte_max'], a['path'][0], a['path'][1], a['hand_t'], a['hand_d'], a['hand_vc'], a['past_ft'])
    if not a['taken']:
        out += '; NOT TAKEN by the hold'
    if 'hold_t' in a:
        out += '; holding at %3.0f s, %4.1f ft from the point, %3.0f ft up' % (a['hold_t'], a['hold_d'], a['hold_h'])
    if 'down_t' in a:
        out += '; down at %3.0f s, %4.1f ft from the point, sink %3.1f ft/s, end state %d, lever %.2f' % (
            a['down_t'], a['down_d'], a['sink'], a['end_state'], a['tcl_end'])
    out += ' | pitch %5.1f..%4.1f bank %5.1f..%4.1f' % (a['theta'][0], a['theta'][1], a['phi'][0], a['phi'][1])
    return out


def lnav_numbers(r, direct):
    """each leg's (from the second with a direct start): how far past it the turn onto it carried (the cross-track
    error on the far side from the one it closed from, from where the track came within 20 deg of the leg's) and the
    error at its end, ft"""
    xte, leg = np.array(r['xte']), np.array(r['leg'])
    out = []
    for l in range(2 if direct else 1, 5):
        w = np.where(leg == l)[0]
        if len(w):
            near = w[2:][np.abs(np.array(r['terr'])[w[2:]]) < 20]     # (the GPS's next update past the leg's change)
            s0 = np.sign(xte[near[0]]) if len(near) else 0.0
            after = w[w >= near[0]] if len(near) else w
            out.append(dict(leg=l, past=float(max(0.0, (-s0 * xte[after]).max())), end=float(abs(xte[w][-1]))))
    return out


NM_ = 6076.1
# the LNAV routes: 90 deg turns; a direct start 106 deg off the heading, then a 149 deg turn (as from KSFO's 28R)
SHARP = [(0, 0), (4.2 * NM_ * math.sin(math.radians(106)), 4.2 * NM_ * math.cos(math.radians(106)))]
SHARP.append((SHARP[1][0] + 4.5 * NM_ * math.sin(math.radians(255)), SHARP[1][1] + 4.5 * NM_ * math.cos(math.radians(255))))
LNAV_CASES = ((200.0, 0.0, None, False), (100.0, 80.0, None, False), (120.0, 70.0, SHARP, True), (200.0, 0.0, SHARP, True))


if __name__ == '__main__':
    if sys.argv[1:2] == ['approach']:
        for angle, land, wind in ((3.0, True, 0.0), (6.0, False, 0.0), (6.0, True, 15.0)):
            print('%2.0f deg %s wind %2.0f: %s' % (angle, 'land ' if land else 'hover', wind, approach_summary(approach(angle, land, wind))), flush=True)
        sys.exit()
    if sys.argv[1:2] == ['rolldelay']:
        rows = roll_delay()
        for r in rows:
            print('%5.0f kt: bank %4.1f, tau_eq %4.2f s, the augmentation\'s %+5.2f s, overshoot %3.1f deg' % (
                r['kcas'], r['bank_deg'], r['tau_eq_s'], r['tau_loop_s'], r['overshoot_deg']))
        json.dump({'kcas': [r['kcas'] for r in rows], 'tau_loop_s': [round(r['tau_loop_s'], 3) for r in rows],
                   'note': 'aptest.py rolldelay: the augmentation\'s roll delay, s (genv22.roll_delay)'},
                  open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'roll_delay_v22.json'), 'w'), indent=1)
        sys.exit()
    if sys.argv[1:2] in (['rollff'], ['pitchff']):
        axis = sys.argv[1][:-2]
        rows = rate_per_inch(axis)
        for r in rows:
            print('%5.0f kt: %.3f rad/s per inch, its first peak %.2f times that' % (r['kcas'], r['per_in'], r['peak']))
        json.dump({'kcas': [r['kcas'] for r in rows], ('p_per_in' if axis == 'roll' else 'q_per_in'): [round(r['per_in'], 4) for r in rows],
                   'peak_ratio': [round(r['peak'], 3) for r in rows],
                   'note': 'aptest.py %sff: the airframe\'s %s rate per inch of %s stick, rad/s (genv22.%s_ff)' % (
                       axis, axis, 'lateral' if axis == 'roll' else 'aft', axis)},
                  open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', '%s_ff_v22.json' % axis), 'w'), indent=1)
        sys.exit()
    if sys.argv[1:2] == ['ga']:
        for w in (6000.0, 2500.0, 'hover'):
            print(ga_summary(ga(w)), flush=True)
        sys.exit()
    if sys.argv[1:2] == ['climbpower']:
        rows = climb_power()
        for r in rows:
            print('%5.0f kt: level %6.0f hp, 1000 fpm %6.0f, full %6.0f at %5.0f fpm: b %5.1f hp/(ft/s), c %5.2f hp/(ft/s)^2 (residual %.3f)' % (
                r['kcas'], r['p_level'], r['p_1000'], r['p_full'], r['climb_fpm'], r['b'], r['c'], r['res']))
        json.dump({'kcas': [r['kcas'] for r in rows], 'b_hp_fps': [round(r['b'], 3) for r in rows], 'c_hp_fps2': [round(r['c'], 4) for r in rows],
                   'climb_full_fps': [round(r['climb_fpm'] / 60, 3) for r in rows],
                   'weight_lb': rows[0]['weight'],
                   'note': 'aptest.py climbpower: the power a climb takes, P = P_level + b h\' + c h\'^2 (genv22.climb_power)'},
                  open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'climb_power_v22.json'), 'w'), indent=1)
        sys.exit()
    if sys.argv[1:2] == ['tflag']:
        rows = tf_lag()
        for r in rows:
            print('%5.0f kt: the climb %4.2f s behind a ramp (its step\'s peak %4.2f); lags %4.2f of %5.2f s and %5.2f s (rms %.3f); '
                  'the height\'s gain %5.3f /s, crossing over at %4.2f rad/s' % (
                      r['kcas'], r['lag'], r['peak'], r['w'], r['tau1'], r['tau2'], r['res'], r['k_h'], r['w_c']))
        json.dump({'kcas': [r['kcas'] for r in rows], 'w': [round(r['w'], 4) for r in rows], 'tau1_s': [round(r['tau1'], 3) for r in rows],
                   'tau2_s': [round(r['tau2'], 3) for r in rows], 'k_h': [round(r['k_h'], 4) for r in rows], 'lag_s': [round(r['lag'], 3) for r in rows],
                   'note': 'aptest.py tflag: the climb\'s response to TF\'s command, two first order lags in parallel (the quick one\'s share w, '
                           'tau1, tau2 s), the height\'s tracking gain, 1/s, and the delay behind a ramp, s (genv22.tf_lag)'},
                  open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'tf_lag_v22.json'), 'w'), indent=1)
        sys.exit()
    if sys.argv[1:2] == ['tfclimb']:
        for r in tf_climb_check():
            print('%5.0f kt: estimated %5.0f fpm, made %5.0f fpm of energy rate (%+4.0f %%; climbing %5.0f fpm at %5.1f kt), lever %.2f' % (
                r['kcas'], r['est_fpm'], r['made_fpm'], 100 * (r['made_fpm'] / max(r['est_fpm'], 1) - 1), r['climb_fpm'], r['vc'], r['lever']), flush=True)
        sys.exit()
    if sys.argv[1:2] == ['tf']:
        from genv22 import autonac_schedule
        sk, sn = autonac_schedule()
        for name in ('flat', 'hills', 'ridge'):
            for V in (120.0, 150.0, 200.0):
                print('%-5s %3.0f kt: %s' % (name, V, tf_summary(tf_run(V, float(np.interp(V, sk, sn)), name), 200.0)), flush=True)
        sys.exit()
    if sys.argv[1:2] == ['route']:
        r = route_approach()
        t, d, vc, route, gp = (np.array(r[k]) for k in ('t', 'd', 'vc', 'route', 'gp'))
        i0 = np.argmax(vc < vc[0] - 2)
        print('slowing from %3.0f s, %4.2f nm to go (the planner\'s DECEL at %4.2f nm); path joined at %4.2f nm (on the route %d) | %s' % (
            t[i0], d[i0] / 6076.1, (vc[0] ** 2 / (2 * 1.5 * 3600)), d[np.argmax(gp > 0.5)] / 6076.1, route[np.argmax(gp > 0.5)], approach_summary(r)))
        sys.exit()
    if sys.argv[1:2] == ['lnav']:
        for V, n, wps, direct in LNAV_CASES:
            r = lnav(V, n, wps=wps, direct=direct, T=420.0 if wps is None else 300.0)
            parts = ['leg %d: past it %4.0f ft, at its end %4.0f' % (p['leg'], p['past'], p['end']) for p in lnav_numbers(r, direct)]
            print('%3.0f kt: ' % V + '; '.join(parts) + ' | bank %5.1f..%4.1f h %5.0f..%5.0f end leg %d' % (
                min(r['phi']), max(r['phi']), min(r['h']), max(r['h']), r['leg'][-1]))
        sys.exit()
    if sys.argv[1:2] == ['instart']:
        for alt, gear, cargo, fuel in ((1500.0, 1.0, 0.0, 0.5), (1500.0, 0.0, 0.0, 0.5), (10000.0, 0.0, 0.0, 0.5), (1500.0, 0.0, 6000.0, 1.0)):
            for kt in (0.0, 40.0, 80.0, 120.0, 160.0, 200.0, 250.0):
                n = in_air_numbers(in_air_start(kt, gear, alt, cargo, fuel, pitch=0.42))     # (FlightGear's start, measured)
                print('%5.0f ft gear %s %5.0f lb %3.0f kt: lever %.3f nacelles %4.1f | climb %+5.1f..%+5.1f ft/s, height %+5.0f ft, speed %+5.1f kt, '
                      'pitch %+5.1f..%+5.1f' % (alt, 'down' if gear else 'up  ', n['weight'], kt, n['lever'], n['nac'], n['hdot_lo'], n['hdot_hi'],
                                                 n['dh'], n['dv'], n['theta_lo'], n['theta_hi']), flush=True)
        sys.exit()
    if sys.argv[1:2] == ['engage']:
        n = engage_numbers(engage())
        print('engaged at pitch %(theta0)+.1f deg, climbing %(hdot0).1f ft/s, %(vc0).1f kt: the pitch %(theta_lo)+.1f..%(theta_hi)+.1f deg, '
              'its largest reversal %(swing).1f deg; vertical %(g_lo)+.2f..%(g_hi)+.2f g' % n)
        sys.exit()
    for name in sys.argv[1:] or ['alt', 'hdg', 'decel', 'accel', 'slow']:
        summary(name, globals()[name]())
