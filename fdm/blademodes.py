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

modes() returns the first flap, lag and torsion modes and the integrals the flight model needs, in ft, slug, s."""
import math
import numpy as np
from scipy.linalg import eigh
from data import xv15_blade as BL


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


def rigid_pitch(omega):
    """the rigid blade's pitch frequency on the control system, per rev"""
    r, m, ix, eif, eil, gj = blade()
    return math.sqrt(k_control() / (ix.sum() * omega ** 2) + 1)


if __name__ == '__main__':
    r, m, ix, eif, eil, gj = blade()
    print('blade: %.1f lb, pitch inertia %.3f slug ft^2; control system %.0f ft lb/rad' % (m.sum() * 32.174, ix.sum(), k_control()))
    for rpm in (565.0, 458.0, 589.0, 517.0):
        om = rpm * math.pi / 30
        (wf, _), (wl, _), (wt, _) = frequencies(om)
        print('%4.0f rpm: flap %s /rev (%s cpm), lag %s (%s), torsion %s (%s); rigid pitch %.2f /rev' % (
            rpm, ' '.join('%.2f' % (w / om) for w in wf), ' '.join('%.0f' % (w * 30 / math.pi) for w in wf),
            ' '.join('%.2f' % (w / om) for w in wl), ' '.join('%.0f' % (w * 30 / math.pi) for w in wl),
            ' '.join('%.2f' % (w / om) for w in wt), ' '.join('%.0f' % (w * 30 / math.pi) for w in wt), rigid_pitch(om)))
