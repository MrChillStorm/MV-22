#!/usr/bin/env python3
"""Steady descents through the vortex ring state, and Johnson's tiltrotor encounter (vrs.py, genv22.vrs), offline.

Descents: trims in helicopter mode at 1000 ft, out of ground effect, the reference loading, still air, heading into
the motion, at each forward speed from level flight down in steps of 0.05 vh until the trim fails. Each rotor's axial
and edgewise velocity over its own vh (the wake's, genv22.vrs) and its uniform inflow nu = v0 / vh against Johnson's
model at the same point; the aircraft's collective, whose extremes against the descent rate are the heave mode's
stability boundary at a fixed collective (where d(Vz + v)/dVz = 0 at the rotors' thrust). And the uniform inflow's
response to a step of collective in vertical descent just outside and just inside the region's top edge, where the
mass flow passes from momentum theory's to Johnson's.

Encounter: Johnson's V-22 case [TP-2005-213477 table 6, figs. 65-67]: 46 100 lb, trimmed descending at 26.8 kt on a
-30 deg flight path (Vx / vh 0.6, Vz / vh -0.35 at his vh, 65.2 ft/s), then his prescribed collective, a 1 deg step down
and another 1 deg over 10 s, held; the rotor speed held, the augmentation holding the attitude (his autopilot held the
roll rate and the speed). Symmetric, and trimmed in 15 deg of sideslip (his asymmetric trim).

  python3 descent.py [--xv15]         the descents' table
  python3 descent.py encounter        the encounter
"""
import sys, math
import numpy as np
import harness as H
import vrs
from data import bell301 as B
from genv22 import CONTROL_EXECRATE

KT = 1.6878
FPM = 60.0
SPEEDS_KT = (0, 10, 20, 30, 40, 50, 60)
SIDE_KEYS = ('vrs/z', 'vrs/x', 'vrs/vh-fps', 'vrs/in', 'vrs/nu-base', 'vrs/dnu', 'thrust-lbs', 'power-hp')
EXTRA = ['drive/power-required-hp', 'attitude/theta-deg'] + ['aero/rotor/%s/%s' % (s, k) for s in 'rl' for k in SIDE_KEYS] \
    + ['rotor/%s/v0-fps' % s for s in 'rl']


def vh_weight(f, xv15=False, weight=None):
    """the ideal hover induced velocity of half the aircraft's weight, ft/s (the flight tests' scaling)"""
    import v22data, xv15data
    R = (xv15data if xv15 else v22data).R
    return math.sqrt((weight or f['inertia/weight-lbs']) / 2 / (2 * f['atmosphere/rho-slugs_ft3'] * math.pi * R ** 2))


def point(e):
    """the trim's Hann-weighted means: each rotor's z, x, nu and Johnson's nu there"""
    d = dict(zip(EXTRA, e))
    out = dict(power=d['drive/power-required-hp'], theta=d['attitude/theta-deg'])
    for s in 'rl':
        b = 'aero/rotor/%s/' % s
        z, x, vh = d[b + 'vrs/z'], d[b + 'vrs/x'], d[b + 'vrs/vh-fps']
        out[s] = dict(z=z, x=x, vh=vh, nu=d['rotor/%s/v0-fps' % s] / vh, nu_j=vrs.nu(z, x), nu_mom=vrs.upper(z, x),
                      nu_base=d[b + 'vrs/nu-base'], dnu=d[b + 'vrs/dnu'], inside=d[b + 'vrs/in'], thrust=d[b + 'thrust-lbs'],
                      power=d[b + 'power-hp'])
    return out


def sweep(xv15=False, speeds=SPEEDS_KT, alt_ft=1000.0):
    f = H.load(xv15)
    x, r, e = H.trim(f, 0.0, alt_ft=alt_ft, extra=EXTRA)
    vh0 = point(e)['r']['vh']
    vhw = vh_weight(f, xv15)
    step = 0.05 * vh0
    import v22data, xv15data
    out = dict(name='XV-15' if xv15 else 'V-22', weight=f['inertia/weight-lbs'], alt=alt_ft, rho=f['atmosphere/rho-slugs_ft3'],
               R=(xv15data if xv15 else v22data).R, vh_hover=vh0, vh_weight=vhw, speeds=[])
    xl = x
    for V in speeds:
        rows, x0, fails = [], xl, 0
        for i in range(60):
            sink = i * step
            x, r, e = H.trim(f, 0.0, alt_ft=alt_ft, x0=x0, extra=EXTRA, ned=(V * KT, sink), iters=40)
            ok = float(np.abs(r * np.array([1, 1, 1, 10, 10, 10.0])).max()) < 0.02
            if i == 0:
                xl = x
            if not ok:
                fails += 1
                if fails >= 2:
                    break
                continue
            x0 = x
            rows.append(dict(sink_fps=sink, coll=x[0], lon=x[1], lat=x[2], ped=x[3], pitch=x[4], roll=x[5], **point(e)))
        out['speeds'].append(dict(kt=V, rows=rows, boundary=boundary(rows)))
    out['edge'] = edge_response(f, vh0, alt_ft)
    return out


def edge_response(f, vh, alt_ft, sinks=(0.18, 0.19, 0.21, 0.22), step_deg=0.5):
    """the uniform inflow's response to a step of collective in vertical descent either side of the region's top
    edge (D, 0.2 vh): the time to 63 % of its change and the change, the airframe held"""
    out = []
    for zs in sinks:
        x, r, e = H.trim(f, 0.0, alt_ft=alt_ft, extra=EXTRA, ned=(0.0, zs * vh))
        v0, inside = f['rotor/r/v0-fps'], f['aero/rotor/r/vrs/in']
        f['gov/int-deg'] = x[0] + step_deg
        n = int(round(1.0 / f['simulation/dt']))
        v = np.zeros(n)
        for i in range(n):
            f.run()
            v[i] = f['rotor/r/v0-fps']
        dv = v[-1] - v0
        out.append(dict(sink_vh=zs, inside=inside, dv=dv, t63=float(np.argmax((v - v0) / dv >= 0.632) + 1) * f['simulation/dt']))
    return out


def boundary(rows):
    """the trim collective's least value against the descent rate and its greatest past it: the heave mode's neutral
    stability at a fixed collective (the vertex of a parabola through the extreme and its neighbours), as the descent
    rate (ft/s) and the rotors' mean z and x there; within Johnson's region (x < XM)"""
    s = np.array([r['sink_fps'] for r in rows]); c = np.array([r['coll'] for r in rows])
    z = np.array([(r['r']['z'] + r['l']['z']) / 2 for r in rows]); x = np.array([(r['r']['x'] + r['l']['x']) / 2 for r in rows])
    out, i0 = {}, 1
    for name, sign in (('lower', 1), ('upper', -1)):
        for i in range(i0, len(c) - 1):
            if sign * (c[i - 1] - c[i]) > 0 and sign * (c[i + 1] - c[i]) >= 0:
                a, b, _ = np.polyfit(s[i - 1:i + 2], c[i - 1:i + 2], 2)
                sv = float(-b / (2 * a))
                if np.interp(sv, s, x) >= vrs.XM:          # past Johnson's region: momentum theory's, no instability
                    continue
                out[name] = dict(sink_fps=sv, z=float(np.interp(sv, s, z)), x=float(np.interp(sv, s, x)))
                i0 = i + 1
                break
    return out


# ---------------------------------------------------------------- Johnson's encounter
ENC_WEIGHT, ENC_VH = 46100.0, 65.2         # lb, ft/s [TP table 6 and p. 17]
ENC_VX, ENC_VZ = 0.6, -0.35                # over vh


def collective_change(t):
    """[TP fig. 65] deg"""
    return 0.0 if t <= 0 else -1.0 - min(t, 10.0) / 10.0


def encounter(beta_deg=0.0, T=50.0, every=0.1):
    import v22data as V
    f = H.load(False)
    f['inertia/pointmass-weight-lbs[%d]' % V.CARGO_PM] = V.CARGO_REF - (V.GW_DESIGN - ENC_WEIGHT)
    vx, vz = ENC_VX * ENC_VH, ENC_VZ * ENC_VH
    b = math.radians(beta_deg)
    ned = (vx * math.cos(b), -vz, -vx * math.sin(b))
    x, r, e = H.trim(f, 0.0, ned=ned, extra=EXTRA, iters=40)
    # the free aircraft as handling.free has it, but the rotor speed held and the collective prescribed through the
    # governor's integral
    for _ in range(CONTROL_EXECRATE):
        f.run()
    f['ctl/init'] = 0; f['ctl/scas-on'] = 1
    for a in ('fcs/elevator-cmd-norm', 'fcs/aileron-cmd-norm', 'fcs/rudder-cmd-norm'):
        f[a] = 0.0
    for _ in range(CONTROL_EXECRATE):
        f.run()
    H.freeze(f, False)
    f['scas/theta-int-in'] = x[1] * B.X_LON - f['scas/lon-trim-in']; f['scas/phi-int-in'] = x[2] * B.X_LAT
    for ax in ('theta', 'phi', 'psi'):
        f['scas/%s-ref-rad' % ax] = f['attitude/%s-rad' % ax]
    vhw = vh_weight(f, weight=ENC_WEIGHT)
    dt = f['simulation/dt']
    n_every = int(round(every / dt))
    out = dict(beta0=beta_deg, trim=[float(a) for a in x], residual=float(np.abs(r).max()), vh_weight=vhw,
               **{k: [] for k in ('t', 'vz', 'vx', 'nz', 'phi', 'theta', 'beta', 'p', 'r', 'dcoll', 'z_r', 'z_l', 'x_r', 'x_l', 'nu_r',
                                  'nu_l', 'in_r', 'in_l', 'power_r', 'power_l', 'thrust_r', 'thrust_l')})
    for i in range(int(T / dt)):
        t = i * dt
        f['gov/int-deg'] = x[0] + collective_change(t)
        f.run()
        if i % n_every == 0:
            out['t'].append(t)
            out['vz'].append(-f['velocities/v-down-fps'] / ENC_VH)
            out['vx'].append(math.hypot(f['velocities/v-north-fps'], f['velocities/v-east-fps']) / ENC_VH)
            out['nz'].append(-f['accelerations/n-pilot-z-norm'])
            out['phi'].append(f['attitude/phi-deg']); out['theta'].append(f['attitude/theta-deg'])
            out['beta'].append(f['aero/beta-deg'])
            out['p'].append(math.degrees(f['velocities/p-rad_sec'])); out['r'].append(math.degrees(f['velocities/r-rad_sec']))
            out['dcoll'].append((f['ctl/r/theta75-deg'] - f['ctl/l/theta75-deg']) / 2)
            for s in 'rl':
                b = 'aero/rotor/%s/' % s
                vh = f[b + 'vrs/vh-fps']
                for k, prop in (('z', 'vrs/z'), ('x', 'vrs/x'), ('in', 'vrs/in'), ('power', 'power-hp'), ('thrust', 'thrust-lbs')):
                    out[k + '_' + s].append(f[b + prop])
                out['nu_' + s].append(f['rotor/%s/v0-fps' % s] / vh)
    return out


if __name__ == '__main__':
    if 'encounter' in sys.argv:
        for beta in (0.0, -15.0):
            o = encounter(beta)
            t = np.array(o['t']); phi = np.abs(o['phi'])
            off = t[np.argmax(phi > 10)] if phi.max() > 10 else None
            print('beta %.0f: trim residual %.3f, Vz/vh at 0, 10, 20, 30 s: %s, least nz %.2f g, roll past 10 deg at %s s, %.0f deg at most' % (
                beta, o['residual'], ['%.2f' % o['vz'][int(tt / 0.1)] for tt in (0, 10, 20, 30)], min(o['nz']), off, phi.max()))
    else:
        o = sweep('--xv15' in sys.argv)
        print('%s %.0f lb, vh %.1f ft/s (the weight\'s ideal %.1f)' % (o['name'], o['weight'], o['vh_hover'], o['vh_weight']))
        for e in o['edge']:
            print('descending at %.2f vh (%s the region): the uniform inflow %.2f ft/s up for %.1f deg of collective, 63 %% in %.3f s' % (
                e['sink_vh'], 'in' if e['inside'] > 0.5 else 'out of', e['dv'], 0.5, e['t63']))   # edge_response's step
        for sp in o['speeds']:
            print('%3d kt: %d trims to %.0f ft/min; collective least at %s, greatest at %s ft/min' % (
                sp['kt'], len(sp['rows']), sp['rows'][-1]['sink_fps'] * FPM,
                *['%.0f (z %.3f, x %.3f)' % (sp['boundary'][k]['sink_fps'] * FPM, sp['boundary'][k]['z'], sp['boundary'][k]['x'])
                  if k in sp['boundary'] else '-' for k in ('lower', 'upper')]))
            for row in sp['rows']:
                rr = row['r']
                print('   %5.0f ft/min  z %6.3f x %5.3f  nu %.3f  Johnson %.3f  coll %6.2f  P %6.0f hp' % (
                    row['sink_fps'] * FPM, rr['z'], rr['x'], rr['nu'], rr['nu_j'], row['coll'], row['power']))
