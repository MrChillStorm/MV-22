#!/usr/bin/env python3
"""The proprotors' wake over the wing, the horizontal stabiliser and the fins, from a skewed cylinder of ring vortices
(uniform loading, Heyson & Katzoff NACA TR 1319), as the UH-60 and Bo 105 models' mrwake.py: rings parallel to the
disc, leaving the hub along the skew line at chi from the disc's normal towards where the air carries them (against
the aircraft's motion in the disc's plane), contracting to 0.78 R below the disc in the hover [Landgrebe] and not at
all edgewise. Averaged over each target and normalised by the velocity at the rotor's centre (the model's mean
inflow).

Tabulated for the right rotor (it turns anticlockwise seen from above in helicopter mode) against the wake skew chi,
the direction psi of the aircraft's motion in the disc's plane (0 along the disc's x axis, which is forward in
helicopter mode and down in airplane mode; 90 to the right) and the nacelle angle, which moves the hub with the
nacelle and turns the disc: for each wing strip of both wings, the stabiliser and both fins. The left rotor's are the
same tables for the mirror-image targets (genv22.py).

  python3 rotorwake.py [--xv15] --write      recompute data/rotorwake_v22.json (or _xv15)
"""
import math, os, json, sys
import numpy as np
from scipy.special import ellipk, ellipe
from scipy.integrate import quad_vec

CHI = [0, 10, 20, 30, 45, 60, 70, 80, 90]
PSI = list(range(0, 361, 45))
NACELLE = [0, 30, 60, 75, 90, 97.5]
NSTRIP = 4
HERE = os.path.dirname(os.path.abspath(__file__))
CONTRACTION, LAMBDA_C = 0.22, 5.0


def ring(r, z, R=1.0):
    """axial and radial velocity of a unit-circulation ring (radius R) at radial distance r, axial distance z"""
    a = (R + r) ** 2 + z * z
    m = min(4 * R * r / a, 1 - 1e-12)
    K, E = ellipk(m), ellipe(m)
    d = (R - r) ** 2 + z * z
    uz = (K + (R * R - r * r - z * z) / d * E) / (2 * math.pi * math.sqrt(a))
    ur = z / (2 * math.pi * r * math.sqrt(a)) * (-K + (R * R + r * r + z * z) / d * E) if r > 1e-9 else 0.0
    return uz, ur


def wake_velocity(P, n, s_hat, chi):
    """induced air velocity at P (body axes, rotor radii from the hub) of a semi-infinite tube of unit rings parallel
    to the disc, axis n into the wake, skewed by chi towards s_hat, contracting with the distance s along it"""
    w = math.cos(chi) * n + math.sin(chi) * s_hat
    kc = CONTRACTION * math.cos(chi)

    def vel(s):
        d = P - s * w
        z = float(d @ n)
        rv = d - z * n
        r = float(np.linalg.norm(rv))
        Rs = 1.0 - kc * (1.0 - math.exp(-LAMBDA_C * s))
        uz, ur = ring(r / Rs, z / Rs)
        return (uz * n + (ur * rv / r if r > 1e-9 else 0.0)) / Rs

    pts = [x for x in (float(P @ n) / math.cos(chi) if abs(math.cos(chi)) > 1e-6 else None, float(P @ w)) if x is not None and 0 < x < 60]
    return quad_vec(vel, 0, 60, limit=500, points=pts or None)[0] + quad_vec(vel, 60, np.inf, limit=200)[0]


def frame(nac_deg):
    """wake axis (into the wake), disc x and y axes in body axes (x forward, y right, z down)"""
    b = math.radians(nac_deg)
    return np.array([-math.cos(b), 0.0, math.sin(b)]), np.array([math.sin(b), 0.0, math.cos(b)]), np.array([0.0, 1.0, 0.0])


def hub(U, nac_deg):
    """right hub (FS, BL, WL) at a nacelle angle: the helicopter-mode offset from the pivot turned forward by 90 - nac"""
    f = math.radians(90 - nac_deg)
    dx, dz = -U.DX_HUB, U.DZ_HUB            # forward, up
    fwd = dx * math.cos(f) + dz * math.sin(f)
    up = -dx * math.sin(f) + dz * math.cos(f)
    return U.FS_PIV - fwd, U.BL_PIV, U.WL_PIV + up


def wing_strips(U, n=NSTRIP):
    """the wing's strips (right wing): (BL inboard, BL outboard, BL mid, FS of the quarter chord, WL), in"""
    out = []
    edges = [U.BL_W_ROOT + (U.BL_W_TIP - U.BL_W_ROOT) * k / n for k in range(n + 1)]
    for a, b in zip(edges, edges[1:]):
        y = (a + b) / 2
        fs = U.FS_W_TIP + (U.BL_W_TIP - y) * math.tan(math.radians(-U.SWEEP_W))
        wl = U.WL_W_ROOT + (y - U.BL_W_ROOT) * math.tan(math.radians(U.DIHEDRAL_W))
        out.append((a, b, y, fs, wl))
    return out


def targets(U):
    """{name: [(FS, BL, WL), ...]} sample points of each target (right rotor's view: both wings, stabiliser, fins)"""
    T = {}
    c = U.C_W * 12
    for side, sg in (('r', 1), ('l', -1)):
        for i, (a, b, y, fs, wl) in enumerate(wing_strips(U)):
            T['w%s%d' % (side, i)] = [(fs - 0.25 * c + xc * c, sg * yy, wl) for xc in (0.1, 0.5, 0.9) for yy in (a + 0.25 * (b - a), a + 0.75 * (b - a))]
    T['stab'] = [(U.FS_H, y, U.WL_H) for y in np.linspace(-0.45, 0.45, 5) * U.B_H * 12]
    for side, sg in (('r', 1), ('l', -1)):
        T['fin' + side] = [(U.FS_V, sg * U.BL_V, U.WL_V + z) for z in np.linspace(-0.35, 0.35, 3) * U.B_V * 12]
    return T


def compute(U, name, pts):
    out = []
    for nac in NACELLE:
        hx, hy, hz = hub(U, nac)
        n, xs, ys = frame(nac)
        tab = []
        for chi in CHI:
            row = []
            for psi in PSI:
                p = math.radians(psi)
                s_hat = -(math.cos(p) * xs + math.sin(p) * ys)
                v = np.zeros(3)
                for fs, bl, wl in pts:
                    P = np.array([-(fs - hx), bl - hy, -(wl - hz)]) / 12.0 / U.R
                    v += wake_velocity(P, n, s_hat, math.radians(min(chi, 89.0)))
                # normalised by the velocity at the rotor centre: 1/2 of the unit tube's far-wake velocity, 0.5
                row.append([float(x) for x in v / len(pts) / 0.5])
            tab.append(row)
        out.append(tab)
    return name, out


def _job(args):
    U = __import__(args[0])
    return compute(U, args[1], args[2])


def path(U):
    return os.path.join(HERE, 'data', 'rotorwake_%s.json' % ('xv15' if U.NAME == 'XV-15' else 'v22'))


def load(U):
    return json.load(open(path(U)))


if __name__ == '__main__':
    mod = 'xv15data' if '--xv15' in sys.argv else 'v22data'
    U = __import__(mod)
    if '--write' in sys.argv:
        from multiprocessing import Pool
        T = targets(U)
        with Pool() as pool:
            res = dict(pool.map(_job, [(mod, k, v) for k, v in T.items()]))
        json.dump({'chi': CHI, 'psi': PSI, 'nacelle': NACELLE, 'strips': wing_strips(U), 'k': res}, open(path(U), 'w'))
        print('wrote', path(U))
    d = load(U)
    i90, i0 = d['nacelle'].index(90), d['nacelle'].index(0)
    for k, tab in d['k'].items():
        print('%-5s hover k = %s   airplane, axial k = %s' % (k, np.round(tab[i90][0][0], 2), np.round(tab[i0][0][0], 2)))
