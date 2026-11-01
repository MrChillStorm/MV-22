#!/usr/bin/env python3
"""Rotating modes of the XV-15's metal proprotor blade from its mass and stiffness distribution [data/xv15_blade.py:
NASA's CAMRAD II model of it, TP-2004-212262], as the Bo 105's: the blade a beam from the shaft to the tip with the
masses lumped at the 51 stations and the stiffness constant between them. Flap and lag bending are Euler-Bernoulli
beam elements (cubic Hermite) with the centrifugal tension's geometric stiffness, and for lag the centrifugal
softening; torsion has linear elements on the control system's stiffness at the root (the pitch link through its
horn), with the propeller moment of the blade's pitch inertia. Flap, lag and torsion are uncoupled (no pitch, centre
of gravity or elastic axis offsets). The collective modes: the yoke holds the blade's root (the gimbal does not tilt
for them).

  python3 blademodes.py           the frequencies at the rotor speeds of the fan plots [TX figs. 7.1.1, 7.1.2] and of flight
  python3 blademodes.py --bending the blades' static bending and twist at the XV-15's hover and cruise trims
  python3 blademodes.py --v22     the V-22 blade's torsion against its 1/5-scale model's [data/v22_blade.py]

modes() returns the first flap, lag and torsion modes and the integrals the flight model needs, in ft, slug, s."""
import math
import numpy as np
from scipy.linalg import eigh
from data import xv15_blade as BL
from data import sections as SEC


def blade(gj_scale=1.0):
    """nodes root to tip: r (ft), lumped mass (slug), pitch inertia (slug ft^2); element stiffness (lb ft^2), each
    element the mean of its two stations'"""
    r = np.array(BL.R_R) * BL.R
    L = np.diff(r)
    share = np.concatenate([[L[0] / 2], (L[:-1] + L[1:]) / 2, [L[-1] / 2]])
    m = np.array(BL.MASS) * share
    ix = np.array(BL.I_THETA) * share
    mid = lambda a: (np.array(a)[:-1] + np.array(a)[1:]) / 2
    return r, m, ix, mid(BL.EI_FLAP), mid(BL.EI_LAG), mid(BL.GJ) * gj_scale


def k_control():
    """the control system's stiffness about the blade's pitch axis, ft lb/rad: the pitch link's on its horn's arm
    [TP: KPL on XPH = 0.059 R]"""
    return BL.K_PITCH_LINK * (0.059 * BL.R) ** 2


def tension(r, m, omega):
    """centrifugal tension in each element (lb): the lumped masses outboard of it"""
    mr = m * r
    return np.array([omega ** 2 * mr[j + 1:].sum() for j in range(len(r) - 1)])


def bending(r, m, ei, inplane):
    """stiffness (structural, geometric per unit of Omega^2) and mass matrices, the root clamped; DOFs (w, w') per node"""
    n = len(r)
    K = np.zeros((2 * n, 2 * n)); Kg = np.zeros((2 * n, 2 * n)); Mm = np.zeros((2 * n, 2 * n))
    T = tension(r, m, 1.0)
    for j in range(n - 1):
        L = r[j + 1] - r[j]
        ke = ei[j] / L ** 3 * np.array([[12, 6 * L, -12, 6 * L], [6 * L, 4 * L * L, -6 * L, 2 * L * L],
                                        [-12, -6 * L, 12, -6 * L], [6 * L, 2 * L * L, -6 * L, 4 * L * L]])
        kg = T[j] / (30 * L) * np.array([[36, 3 * L, -36, 3 * L], [3 * L, 4 * L * L, -3 * L, -L * L],
                                         [-36, -3 * L, 36, -3 * L], [3 * L, -L * L, -3 * L, 4 * L * L]])
        idx = [2 * j, 2 * j + 1, 2 * j + 2, 2 * j + 3]
        K[np.ix_(idx, idx)] += ke; Kg[np.ix_(idx, idx)] += kg
    for j in range(n):
        Mm[2 * j, 2 * j] = m[j]
    soft = Mm.copy() if inplane else np.zeros_like(Mm)     # in plane the centrifugal force also pulls the mass outwards
    free = list(range(2, 2 * n))
    sub = lambda A: A[np.ix_(free, free)]
    return sub(K), sub(Kg), sub(Mm), sub(soft)


def torsion(r, ix, gj, k_root):
    n = len(r)
    K = np.zeros((n, n)); Mm = np.diag(ix)
    for j in range(n - 1):
        k = gj[j] / (r[j + 1] - r[j])
        K[j:j + 2, j:j + 2] += k * np.array([[1, -1], [-1, 1]])
    K[0, 0] += k_root
    return K, Mm


def solve(K, M, n=3):
    """lowest n modes of K v = w^2 M v with a diagonal M: the massless DOFs (the slopes) condensed out statically,
    which is exact for them"""
    d = np.diag(M)
    a = np.where(d > 1e-9 * d.max())[0]; b = np.where(d <= 1e-9 * d.max())[0]
    if len(b):
        Kbb_inv_Kba = np.linalg.solve(K[np.ix_(b, b)], K[np.ix_(b, a)])
        Kr = K[np.ix_(a, a)] - K[np.ix_(a, b)] @ Kbb_inv_Kba
    else:
        Kr = K
    w, Va = eigh(Kr, M[np.ix_(a, a)])
    V = np.zeros((len(d), n))
    V[a] = Va[:, :n]
    if len(b):
        V[b] = -Kbb_inv_Kba @ Va[:, :n]
    return np.sqrt(np.maximum(w[:n], 0)), V


def frequencies(omega, n=3, gj_scale=1.0):
    """the collective (root-clamped) flap and lag modes and the torsion modes at omega (rad/s): (w, V) each"""
    r, m, ix, eif, eil, gj = blade(gj_scale)
    Kf, Kgf, Mf, _ = bending(r, m, eif, False)
    Kl, Kgl, Ml, Sl = bending(r, m, eil, True)
    Kt, Mt = torsion(r, ix, gj, k_control())
    return (solve(Kf + omega ** 2 * Kgf, Mf, n), solve(Kl + omega ** 2 * (Kgl - Sl), Ml, n), solve(Kt + omega ** 2 * Mt, Mt, n))


def coupled(theta75_deg, omega, n=4):
    """the collective (root-clamped) bending modes with flap and lag coupled by the blade's pitch: each section's
    principal axes turned by its pitch, the collective at 0.75 R plus the structural twist [TP: TWISTA, 0 at 0.75 R];
    DOFs (w, w', v, v') per node, w out of the plane, v in it. Returns the frequencies (rad/s) and each mode's share
    of kinetic energy out of the plane"""
    r, m, ix, eif, eil, gj = blade()
    th = np.radians(theta75_deg + (np.array(BL.TWIST)[:-1] + np.array(BL.TWIST)[1:]) / 2)
    nn = len(r)
    K = np.zeros((4 * nn, 4 * nn)); Kg = np.zeros_like(K); Mm = np.zeros_like(K); S = np.zeros_like(K)
    T = tension(r, m, 1.0)
    for j in range(nn - 1):
        L = r[j + 1] - r[j]
        c, s = math.cos(th[j]), math.sin(th[j])
        e_ww = eif[j] * c * c + eil[j] * s * s
        e_vv = eif[j] * s * s + eil[j] * c * c
        e_wv = (eil[j] - eif[j]) * s * c
        H = 1 / L ** 3 * np.array([[12, 6 * L, -12, 6 * L], [6 * L, 4 * L * L, -6 * L, 2 * L * L],
                                   [-12, -6 * L, 12, -6 * L], [6 * L, 2 * L * L, -6 * L, 4 * L * L]])
        G = T[j] / (30 * L) * np.array([[36, 3 * L, -36, 3 * L], [3 * L, 4 * L * L, -3 * L, -L * L],
                                        [-36, -3 * L, 36, -3 * L], [3 * L, -L * L, -3 * L, 4 * L * L]])
        w_idx = [4 * j, 4 * j + 1, 4 * j + 4, 4 * j + 5]
        v_idx = [4 * j + 2, 4 * j + 3, 4 * j + 6, 4 * j + 7]
        for (a, b, e) in ((w_idx, w_idx, e_ww), (v_idx, v_idx, e_vv), (w_idx, v_idx, e_wv), (v_idx, w_idx, e_wv)):
            K[np.ix_(a, b)] += e * H
        Kg[np.ix_(w_idx, w_idx)] += G; Kg[np.ix_(v_idx, v_idx)] += G
    for j in range(nn):
        Mm[4 * j, 4 * j] = m[j]; Mm[4 * j + 2, 4 * j + 2] = m[j]; S[4 * j + 2, 4 * j + 2] = m[j]
    free = list(range(4, 4 * nn))
    sub = lambda A: A[np.ix_(free, free)]
    w, V = solve(sub(K) + omega ** 2 * (sub(Kg) - sub(S)), sub(Mm), n)
    Md = np.diag(sub(Mm))
    flap = [float((Md[0::4] * V[0::4, k] ** 2).sum() / (Md * V[:, k] ** 2).sum()) for k in range(n)]
    return w, flap


# ---------------------------------------------------------------- torsion, for the flight model
R_PITCH_BEARING = 0.11      # r/R: the pitch bearing [TP: EPH]; inboard of it the yoke, which does not pitch
ZETA_TORSION = 0.005        # structural damping, of critical [TP: GDAMPT 0.01, the structural damping coefficient g = 2 zeta]


def torsion_blade(bl=BL):
    """the torsion system from the pitch bearing out: nodes (ft), lumped pitch inertia (slug ft^2), element GJ (lb ft^2),
    mass (slug), the CG's offset aft of the elastic axis (ft) at the nodes"""
    r = np.array(bl.R_R) * bl.R
    keep = r >= R_PITCH_BEARING * bl.R - 1e-9
    L = np.diff(r)
    share = np.concatenate([[L[0] / 2], (L[:-1] + L[1:]) / 2, [L[-1] / 2]])
    mid = lambda a: (np.array(a)[:-1] + np.array(a)[1:]) / 2
    ix = np.array(bl.I_THETA) * share
    m = np.array(bl.MASS) * share
    gj = mid(bl.GJ)
    i0 = int(np.argmax(keep))
    return r[i0:], ix[i0:], gj[i0:], m[i0:], np.array(bl.X_CG)[i0:] * bl.R


def k_control_bl(bl=BL):
    return bl.K_PITCH_LINK * (bl.X_PITCH_HORN * bl.R) ** 2


def torsion_mode(omega, bl=BL):
    """the first torsion mode at omega (rad/s): its frequency (rad/s) and shape at the nodes, 1 at the tip; the rigid
    blade's pitch frequency on the control system"""
    r, ix, gj, m, dcg = torsion_blade(bl)
    K, Mt = torsion(r, ix, gj, k_control_bl(bl))
    w, V = solve(K + omega ** 2 * Mt, Mt, 2)
    phi = V[:, 0] / V[-1, 0]
    return w, phi, math.sqrt(k_control_bl(bl) / (ix.sum() * max(omega, 1e-6) ** 2) + 1)


def torsion_constants(seg_r, edges, omega, sections, chord_fn, bl=BL):
    """everything the flight model's torsion mode needs, ft-slug-s, at the aerodynamic segments (radii seg_r, edges):
       It qt'' = sum_j phi_j Mea_j - W^2 sum_j P_j sin(2 theta_j) / 2 + Scg (fc cos psi + fs sin psi + W^2 precone)
                 - Kt qt - (2 zeta sqrt(Kt' It) + Ct rho W) qt'
    Mea_j the segment's moment about the elastic axis (its section's moment at the quarter chord, the normal force at
    the quarter chord's offset aft of it); theta_j its whole pitch, the propeller moment's; P_j the pitch inertia's
    share of the mode lumped to it; Scg the CG's offset times the flap acceleration and the precone's centrifugal force"""
    r, ix, gj, m, dcg = torsion_blade(bl)
    K, Mt = torsion(r, ix, gj, k_control_bl(bl))
    w, phi, rigid = torsion_mode(omega, bl)
    It = float(phi @ Mt @ phi); Kt = float(phi @ K @ phi)
    # each node's pitch inertia to the segment it lies in (the nearest, beyond the aerodynamic blade)
    seg_of = [min(range(len(seg_r)), key=lambda j: 0 if edges[j] <= x < edges[j + 1] else min(abs(x - edges[j]), abs(x - edges[j + 1])))
              for x in r]
    P = [float(sum(ix[i] * phi[i] for i in range(len(r)) if seg_of[i] == j)) for j in range(len(seg_r))]
    phi_seg = [float(np.interp(x, r, phi)) for x in seg_r]
    d_ac = [float(np.interp(x / bl.R, bl.R_R, bl.X_QC)) * bl.R for x in seg_r]      # the quarter chord aft of the elastic axis
    cm0 = []
    for x in seg_r:
        t, cli = SEC.interp_sections(sections, x / bl.R)
        cm0.append(SEC.cm0(cli))
    # aerodynamic damping of the section pitching about its elastic axis, quasi-steady thin airfoil (Theodorsen,
    # C(k) = 1), a semichords aft of mid-chord: -(pi (1/2 - a) - 2 pi (1/4 - a^2)) rho U b^3 theta'
    ct = 0.0
    for j, x in enumerate(seg_r):
        c = chord_fn(x)
        a = 2 * (0.25 - d_ac[j] / c) - 1
        coef = math.pi * (0.5 - a) - 2 * math.pi * (0.25 - a * a)
        ct += coef * (c / 2) ** 3 * x * phi_seg[j] ** 2 * (edges[j + 1] - edges[j])
    return dict(It=It, Kt=Kt, freq=w[0] / omega, freq2=w[1] / omega, rigid=rigid, phi=phi_seg, P=P, d_ac=d_ac, cm0=cm0,
                Scg=float((m * r * dcg * phi).sum()), Ct=ct, zeta=ZETA_TORSION, k_control=k_control_bl(bl), r=list(r), shape=list(phi))


def rigid_pitch(omega):
    """the rigid blade's pitch frequency on the control system, per rev"""
    r, m, ix, eif, eil, gj = blade()
    return math.sqrt(k_control() / (ix.sum() * omega ** 2) + 1)


def static_bending(seg_r, Fn, Ft, omega, precone):
    """the blade's static flap and lag tip deflections (deg, tip / R) under its segments' normal and in-plane loads
    (lb, at radii seg_r), the root held (the collective case) and the precone's centrifugal force along the normal"""
    r, m, ix, eif, eil, gj = blade()
    out = []
    for ei, inplane, F in ((eif, False, Fn), (eil, True, [-x for x in Ft])):
        K, Kg, Mm, S = bending(r, m, ei, inplane)
        load = np.zeros(2 * len(r))
        for x, f in zip(seg_r, F):
            load[2 * int(np.argmin(abs(r - x)))] += f
        if not inplane:
            load[0::2] += -m * omega ** 2 * r * math.sin(precone)
        out.append(math.degrees(np.linalg.solve(K + omega ** 2 * (Kg - S), load[2:])[-2] / BL.R))
    return out


def bending_at_trims():
    """the XV-15's blades' static bending at the hover's and the cruise's trims (blade 0 of the right rotor)"""
    import harness as H, genv22, xv15data as X
    genv22.U = X
    segs, edges = genv22.segments(X.NSEG, X.R_ROOT, X.R)
    rs = [a for a, b in segs]
    props = tuple('aero/rotor/r/b0/s%d/Fn' % j for j in range(len(rs))) + tuple('aero/rotor/r/b0/s%d/Ft' % j for j in range(len(rs)))
    out = {}
    for name, V, alt, nac in (('hover', 0.0, 1000.0, 90.0), ('cruise 180 KCAS', 180.0, 10000.0, 0.0)):
        f = H.load(xv15=True)
        f['gear/gear-cmd-norm'] = 0; f['gear/gear-pos-norm'] = 0
        if nac == 0.0:
            f['ctl/flaperon-override'] = 1; f['ctl/flaperon-set-deg'] = 0.0
        x, r, e = H.trim(f, V, alt_ft=alt, nacelle=nac, cas=True, extra=props)
        out[name] = static_bending(rs, e[:len(rs)], e[len(rs):], f['rotor/omega-rad_sec'], X.PRECONE) + [math.degrees(f['rotor/r/b0/qt'])]
    return out


if __name__ == '__main__' and '--bending' in __import__('sys').argv:
    for name, (fl, lg, tw) in bending_at_trims().items():
        print('%s: elastic coning %+.2f deg beyond the precone, lag %+.2f deg, tip twist %+.2f deg' % (name, fl, lg, tw))
elif __name__ == '__main__' and '--v22' in __import__('sys').argv:
    from data import v22_blade as VB
    w, phi, rigid = torsion_mode(0.0, VB)
    om = 397 * math.pi / 30
    wr, phir, rr = torsion_mode(om, VB)
    print('V-22 blade: first torsion nonrotating %.1f rad/s (WRATS %.1f), at 397 rpm %.2f/rev, rigid pitch %.1f/rev' % (w[0], VB.TORSION_NR, wr[0] / om, rr))
elif __name__ == '__main__':
    r, m, ix, eif, eil, gj = blade()
    print('blade: %.1f lb, pitch inertia %.3f slug ft^2; control system %.0f ft lb/rad' % (m.sum() * 32.174, ix.sum(), k_control()))
    for rpm in (565.0, 458.0, 589.0, 517.0):
        om = rpm * math.pi / 30
        (wf, _), (wl, _), (wt, _) = frequencies(om)
        print('%4.0f rpm: flap %s /rev (%s cpm), lag %s (%s), torsion %s (%s); rigid pitch %.2f /rev' % (
            rpm, ' '.join('%.2f' % (w / om) for w in wf), ' '.join('%.0f' % (w * 30 / math.pi) for w in wf),
            ' '.join('%.2f' % (w / om) for w in wl), ' '.join('%.0f' % (w * 30 / math.pi) for w in wl),
            ' '.join('%.2f' % (w / om) for w in wt), ' '.join('%.0f' % (w * 30 / math.pi) for w in wt), rigid_pitch(om)))
