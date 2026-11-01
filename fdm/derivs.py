#!/usr/bin/env python3
"""The XV-15 build's stability derivatives against Bell's mathematical model's [CR-114614 table III-1, column (1):
13 000 lb, aft CG, sea level standard, constant rotor speed and collective], in airplane mode at 260 kt and 458 rpm,
flaps 0. Each is the derivative of the held airframe's accelerations (the trims' residual: rotors running at a held
speed, the airframe's integration off, averaged over whole revolutions) by a perturbation of the trim's state, made
non-dimensional on the wing-pylon's reference [CR: S 181 ft^2, c 5.225 ft] and the wing's span (32.17 ft), the rates
as q c / 2V, p b / 2V, r b / 2V, the speed as u / V.

  python3 derivs.py"""
import math
import numpy as np
import harness as H
from data import bell301 as B

V_KT, ALT, NAC, RPM = 260.0, 0.0, 0.0, 458.0
S, C, BSPAN = B.S_WP, B.C_WP, 32.17
# Bell's mathematical model's, airplane mode, 260 kt [CR table III-1 (1)]: signs as printed with their minus
BELL = {'CXu': -1.22, 'CZu': -1.46, 'CMu': 0.887, 'CXa': 0.201, 'CZa': -6.47, 'CMa': -1.32, 'CXq': 0.551, 'CMq': -29.8,
        'CYb': -1.83, 'Clb': -0.318, 'CNb': 0.242, 'CYp': -0.188, 'Clp': -1.34, 'CNp': -0.19, 'Clr': -0.414, 'CNr': -1.22,
        'CL0': 0.314}
CG_AFT = B.CG_AFT          # FS, helicopter mode


def accel(f, x, beta=0.0, rates=(0.0, 0.0, 0.0), dV=0.0, settle_s=1.0, revs=2):
    """the residual's state with a sideslip, body rates and an airspeed change held: body accelerations (ft/s^2, rad/s^2)"""
    coll, lon, lat, ped, th, ph = x
    H.controls(f, coll, lon, lat, ped, NAC)
    f['ic/h-sl-ft'] = ALT; f['ic/vt-kts'] = V_KT + dV; f['ic/psi-true-deg'] = 0.0
    f['ic/phi-deg'] = ph; f['ic/beta-deg'] = beta
    f['ic/gamma-deg'] = 0.0; f['ic/alpha-deg'] = th
    f['ic/terrain-elevation-ft'] = H.TERRAIN_FT
    f['ic/p-rad_sec'], f['ic/q-rad_sec'], f['ic/r-rad_sec'] = rates
    f.run_ic()
    for _ in range(int(settle_s / f['simulation/dt'])):
        f.run()
    n = int(round(revs * 2 * math.pi / f['rotor/omega-rad_sec'] / f['simulation/dt']))
    w = np.sin(np.pi * (np.arange(n) + 0.5) / n) ** 2
    w /= w.sum()
    acc = np.zeros(6)
    for i in range(n):
        f.run()
        acc += w[i] * np.array([f[k] for k in H.ACC])
    return acc


def coefficients(f, acc, V):
    """the accelerations as force and moment coefficients (body axes; Z + down, as Bell's)"""
    qS = 0.5 * f['atmosphere/rho-slugs_ft3'] * V * V * S
    m = f['inertia/weight-lbs'] / 32.174
    ixx, iyy, izz, ixz = (f['inertia/%s-slugs_ft2' % k] for k in ('ixx', 'iyy', 'izz', 'ixz'))
    udot, vdot, wdot, pdot, qdot, rdot = acc
    return dict(CX=m * udot / qS, CY=m * vdot / qS, CZ=m * wdot / qS, Cl=(ixx * pdot - ixz * rdot) / (qS * BSPAN),
                CM=iyy * qdot / (qS * C), CN=(izz * rdot - ixz * pdot) / (qS * BSPAN))


def setup():
    f = H.load(xv15=True)
    f.run_ic()
    # the aft CG: the research equipment and ballast (at the design CG) moved aft, in helicopter mode
    f['/controls/flight/nacelle-cmd-deg'] = 90.0; f['ctl/init'] = 1
    for _ in range(4):
        f.run()
    k = 4
    w4 = f['inertia/pointmass-weight-lbs[%d]' % k]
    x4 = f['inertia/pointmass-location-X-inches[%d]' % k]
    f['inertia/pointmass-location-X-inches[%d]' % k] = x4 + (CG_AFT - f['inertia/cg-x-in']) * f['inertia/weight-lbs'] / w4
    for _ in range(4):
        f.run()
    cg_helo = f['inertia/cg-x-in']
    f['/controls/rotor/nr-trim-pct'] = RPM / 589.0 * 100 - 517.0 / 589.0 * 100
    return f, cg_helo


def derivatives():
    f, cg_helo = setup()
    x, r, e = H.trim(f, V_KT, alt_ft=ALT, nacelle=NAC, cas=False)
    V = f['velocities/vt-fps']
    a0 = accel(f, x)
    c0 = coefficients(f, a0, V)
    d = {}
    h = 1.0
    xa = x.copy(); xa[4] += h; ca = coefficients(f, accel(f, xa), V)
    xb = x.copy(); xb[4] -= h; cb = coefficients(f, accel(f, xb), V)
    for k in ('CX', 'CZ', 'CM'):
        d[k + 'a'] = (ca[k] - cb[k]) / (2 * math.radians(h))
    dv = 5.0
    cu = coefficients(f, accel(f, x, dV=dv), V); cl = coefficients(f, accel(f, x, dV=-dv), V)
    # (the forces' change with the speed, on the trim's dynamic pressure: d(X / q S) / d(u / V))
    for k in ('CX', 'CZ', 'CM'):
        d[k + 'u'] = (cu[k] - cl[k]) / (2 * dv * 1.68781 / V)
    dq = 0.02
    cqp = coefficients(f, accel(f, x, rates=(0.0, dq, 0.0)), V); cqm = coefficients(f, accel(f, x, rates=(0.0, -dq, 0.0)), V)
    for k in ('CX', 'CM'):
        d[k + 'q'] = (cqp[k] - cqm[k]) / (2 * dq * C / (2 * V))
    hb = 2.0
    cbp = coefficients(f, accel(f, x, beta=hb), V); cbm = coefficients(f, accel(f, x, beta=-hb), V)
    for k in ('CY', 'Cl', 'CN'):
        d[k + 'b'] = (cbp[k] - cbm[k]) / (2 * math.radians(hb))
    for ax, i in (('p', 0), ('r', 2)):
        rr = [0.0, 0.0, 0.0]; rr[i] = dq; cp = coefficients(f, accel(f, x, rates=tuple(rr)), V)
        rr[i] = -dq; cm = coefficients(f, accel(f, x, rates=tuple(rr)), V)
        for k in (('CY', 'Cl', 'CN') if ax == 'p' else ('Cl', 'CN')):
            d[k + ax] = (cp[k] - cm[k]) / (2 * dq * BSPAN / (2 * V))
    # (the trim's lift coefficient, the weight's: a check of the reference area)
    d['CL0'] = f['inertia/weight-lbs'] / (0.5 * f['atmosphere/rho-slugs_ft3'] * V * V * S)
    return d, dict(trim_res=float(np.abs(r).max()), alpha=f['aero/alpha-deg'], cg_helo=cg_helo, cg=f['inertia/cg-x-in'],
                   rpm=f['drive/nr-pct'] * 5.89, iyy=f['inertia/iyy-slugs_ft2'])


if __name__ == '__main__':
    d, info = derivatives()
    print('260 kt, sea level, 458 rpm (%.0f), CG %.1f in (%.1f in helicopter mode), alpha %.2f deg, Iyy %.0f, trim residual %.3f' % (
        info['rpm'], info['cg'], info['cg_helo'], info['alpha'], info['iyy'], info['trim_res']))
    for k, b in BELL.items():
        print('%-4s model %+8.3f   Bell %+8.3f   ratio %5.2f' % (k, d[k], b, d[k] / b if b else float('nan')))
