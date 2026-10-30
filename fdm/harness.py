"""JSBSim harness for the tiltrotor model: load, freeze the airframe, run the rotors, average over revolutions, trim.
(After the UH-60 and Bo 105 models' harnesses.)

  root: the test build's root, root/ for the V-22 (genv22.py root/aircraft/mv22b), root-xv15/ for the XV-15
        (genv22.py root-xv15/aircraft/xv15 --xv15); V22_ROOT overrides it
"""
import os, math
import jsbsim
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
TERRAIN_FT = -10000.0      # trims: terrain far below (out of ground effect)
DT = 1 / 480.0


def load(xv15=False, root=None, dt=DT, hold_rpm=True):
    root = root or os.environ.get('V22_ROOT') or os.path.join(HERE, 'root-xv15' if xv15 else 'root')
    f = jsbsim.FGFDMExec(root)
    f.set_debug_level(0)
    f['/controls/engines/engine[0]/throttle'] = 0.5
    f.set_dt(dt)
    f.load_model('xv15' if xv15 else 'mv22b')
    if not xv15:
        # the reference loading (GW_DESIGN at the design CG), not FlightGear's start (no cargo, the tanks half full)
        import v22data as V
        f['inertia/pointmass-weight-lbs[%d]' % V.CARGO_PM] = V.CARGO_REF
        for i, (t, r) in enumerate(zip(V.TANKS, V.TANKS_REF)):
            f['propulsion/tank[%d]/contents-lbs' % i] = t[1] * V.FUEL_LB_GAL * r
        f['/sim/freeze/fuel'] = 1             # the loading stays put through the trims' and the runs' steps
    # the gear up: the flight tests' configuration in forward flight (in the hover its drag is nil); on the ground
    # (handling.start) it is put down
    f['gear/gear-cmd-norm'] = 0.0
    f['gear/gear-pos-norm'] = 0.0
    f['drive/hold-rpm'] = 1.0 if hold_rpm else 0.0
    f['ctl/scas-on'] = 0
    try:
        f['ctl/cps-on'] = 0           # the trims command the nacelle angle outside the corridor too
    except KeyError:
        pass
    return f


def freeze(f, on=True):
    for k in ('simulation/integrator/rate/rotational', 'simulation/integrator/rate/translational',
              'simulation/integrator/position/rotational', 'simulation/integrator/position/translational'):
        f[k] = 0 if on else 1


def controls(f, coll=0.0, lon=0.0, lat=0.0, ped=0.0, nacelle=None):
    """coll: the collective governor's integral, deg (the trim's collective variable: the rotor speed is held, so the
    governor's proportional part is zero and the lever's feed forward is fixed); lon, lat, ped -1..1 (+ forward,
    right, right)"""
    f['gov/int-deg'] = coll
    f['fcs/elevator-cmd-norm'] = lon; f['fcs/aileron-cmd-norm'] = lat; f['fcs/rudder-cmd-norm'] = -ped
    if nacelle is not None:
        f['/controls/flight/nacelle-cmd-deg'] = nacelle


ACC = ['accelerations/udot-ft_sec2', 'accelerations/vdot-ft_sec2', 'accelerations/wdot-ft_sec2',
       'accelerations/pdot-rad_sec2', 'accelerations/qdot-rad_sec2', 'accelerations/rdot-rad_sec2']


def residual(f, x, V_kt, alt_ft=1000.0, gamma=0.0, settle_s=1.0, revs=2, nacelle=90.0, beta=0.0, extra=(),
             terrain=TERRAIN_FT, cas=False):
    """x = collective (deg), lon, lat, ped, alpha (pitch attitude in the hover), roll; Hann-weighted mean of the body
    accelerations over whole rotor revolutions, and of the extra properties"""
    coll, lon, lat, ped, th, ph = x
    controls(f, coll, lon, lat, ped, nacelle)
    f['ic/h-sl-ft'] = alt_ft
    f['ic/vc-kts' if cas else 'ic/vt-kts'] = V_kt; f['ic/psi-true-deg'] = 0.0
    f['ic/phi-deg'] = ph
    f['ic/beta-deg'] = beta
    if V_kt < 1.0:
        f['ic/theta-deg'] = th
    else:
        f['ic/gamma-deg'] = gamma; f['ic/alpha-deg'] = th
    f['ic/h-sl-ft'] = alt_ft; f['ic/terrain-elevation-ft'] = terrain
    f['ic/p-rad_sec'] = 0; f['ic/q-rad_sec'] = 0; f['ic/r-rad_sec'] = 0
    f.run_ic()
    for _ in range(int(settle_s / f['simulation/dt'])):
        f.run()
    n = int(round(revs * 2 * math.pi / f['rotor/omega-rad_sec'] / f['simulation/dt']))
    w = np.sin(np.pi * (np.arange(n) + 0.5) / n) ** 2
    w /= w.sum()
    acc = np.zeros(6); ex = np.zeros(len(extra))
    for i in range(n):
        f.run()
        acc += w[i] * np.array([f[k] for k in ACC])
        if extra:
            ex += w[i] * np.array([f[k] for k in extra])
    return acc, ex


def trim(f, V_kt, alt_ft=1000.0, gamma=0.0, x0=None, tol=None, iters=24, verbose=False, nacelle=90.0, beta=0.0,
         extra=('drive/power-required-hp',), fix=None, terrain=TERRAIN_FT, cas=False):
    """Newton trim of collective, cyclic (or elevator), pedals, angle of attack and roll for steady straight flight
    (airframe frozen, rotors running at the held speed), bounded, with a backtracking line search. fix: {index: value}
    holds controls (e.g. the pedals in airplane mode). Returns x, residuals, extra means."""
    f['ctl/init'] = 1
    freeze(f)
    lo = np.array([-10.0, -1, -1, -1, -30, -30]); hi = np.array([45.0, 1, 1, 1, 30, 30])
    x = np.array(x0 if x0 is not None else [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], float)
    fix = fix or {}
    for i, val in fix.items():
        x[i] = val
    free = [i for i in range(6) if i not in fix]
    h = np.array([0.2, 0.01, 0.01, 0.01, 0.3, 0.3])
    scale = np.array([1, 1, 1, 10, 10, 10.0])
    cost = lambda r: float(np.abs(r * scale).max())
    kw = dict(nacelle=nacelle, beta=beta, terrain=terrain, cas=cas)
    r0, e0 = residual(f, x, V_kt, alt_ft, gamma, extra=extra, **kw)
    for it in range(iters):
        if verbose:
            print(it, np.round(x, 4), np.round(r0, 4), np.round(e0, 1))
        if cost(r0) < (tol or 0.02):
            break
        J = np.zeros((6, len(free)))
        for jj, j in enumerate(free):
            xp = x.copy(); xp[j] += h[j]
            J[:, jj] = (residual(f, xp, V_kt, alt_ft, gamma, **kw)[0] - r0) / h[j]
        if not (np.all(np.isfinite(J)) and np.all(np.isfinite(r0))):
            break
        dxf = np.linalg.lstsq(J * scale[:, None], -r0 * scale, rcond=None)[0]
        dx = np.zeros(6); dx[free] = dxf
        lim = np.array([4.0, 0.15, 0.1, 0.1, 3, 3])
        k = np.max(np.abs(dx) / lim)
        if k > 1:
            dx /= k
        step = 1.0
        for _ in range(6):
            xn = np.clip(x + step * dx, lo, hi)
            rn, en = residual(f, xn, V_kt, alt_ft, gamma, extra=extra, **kw)
            if cost(rn) < cost(r0):
                break
            step *= 0.5
        x, r0, e0 = xn, rn, en
    return x, r0, e0
