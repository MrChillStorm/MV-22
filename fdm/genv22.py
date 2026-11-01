#!/usr/bin/env python3
"""Generate the tiltrotor JSBSim model: V-22 (default) or, for validation, the XV-15 (--xv15).

  python3 genv22.py OUTDIR [--fg] [--xv15]   writes OUTDIR/mv22b.xml and OUTDIR/Systems/mv22b-*.xml (the XV-15:
                                              xv15.xml, Systems/xv15-*.xml); --fg: into the FlightGear aircraft

The files are named mv22b, as the aircraft (MV-22B/mv22b-set.xml), apart from FlightGear's own V-22 (V22-Osprey,
v22-set.xml and its YASim Systems/v22-yasim.xml).

Proprotors: the two three-bladed rotors are simulated blade by blade. Each rotor's blades are rigid and sit with their
precone on a gimbal, so the disc tilts as a whole: the gimbal's two degrees of freedom are integrated from the blades'
aerodynamic flap moments, the gyroscopic moments of the shaft's rotation and the hub spring, with the blades' pitch-flap
coupling (delta-3). Each blade has NSEG segments of equal annulus area at their own angle of attack and Mach number in
section tables over +-180 deg (data/sections.py), in the air the full kinematics give them: the aircraft's motion, the
nacelle's tilting, the rotor's rotation, the gimbal's tilt and its rate, and the induced inflow (Peters-He finite-state
wake per rotor, with ground effect). The nacelles tilt from 90 deg (helicopter) to 0 (airplane); the rotor's frame and
its hub turn with them. The rotors are geared together by the interconnect shaft: one rotor speed.

Shaft axes of a rotor: x along the disc (forward in helicopter mode, down in airplane mode), y right, z along the shaft
into the wake. The right rotor turns anticlockwise seen from above in helicopter mode (its inboard blade moving aft, up
in airplane mode [CR B-4]; "right rotor clockwise when looking forward" [RC]); the left rotor is its mirror image, and
is computed as the right rotor in the mirror-image aircraft (y, p, r and their accelerations negated going in, the
side force, rolling and yawing moments coming out). Azimuth psi from the disc's -x; a blade at psi lies along
e_r = (-cos psi, sin psi, 0), moves along e_t = (sin psi, cos psi, 0); e_n = (0, 0, -1) is up the shaft.
"""
import math, os, sys, json
import numpy as np
from xmlfun import *
import peters_he as PH
from data import sections as SEC
from data import bell301 as B
from data import gtrs as GTRS
from data import heyson_ground as HG
import rotorwake

U = None                  # the aircraft's data module, set by configure()
G0 = 32.174
R2D = 180 / math.pi
SIDES = (('r', 1), ('l', -1))
# Rates. In FlightGear every property read takes a lock, so a step costs what it reads; the rotors' blade elements and
# gimbals and the drive train run at the FDM rate (480 Hz: the gimbals need it), the rest every few frames: the
# Peters-He inflow with its gains and its forcing (time constants of a tenth of a second and more) and the controls
# (pilot, nacelles, mixing, servos, governor, SCAS) at 120 Hz, the airframe at 60 Hz, the engines and outputs too
INFLOW_EXECRATE = 4
CONTROL_EXECRATE = 4
START_HOLD_S = (0.5, 4.5, 5.5)            # an in-air start's trim (controls()): the rotors; the pitch, rotating; the stick
START_TRIM_TAU_S = 0.25                   # its stick's trim's time constant (scas())
AUTOPILOT_EXECRATE = 8                  # the autopilot's outer loops: 60 Hz
AIRFRAME_EXECRATE = 8
SLOW_EXECRATE = 8
# Peters-He states (j, r): three axisymmetric (the hover's radial distribution) and the first harmonic
PH_COS = [(1, 0), (3, 0), (5, 0), (2, 1)]
PH_SIN = [(2, 1)]
PHI10 = PH.phi(1, 0, 0.5)               # the first shape: the same at every radius (sqrt 3)
assert all(abs(PH.phi(1, 0, x) - PHI10) < 1e-12 for x in (0.05, 0.5, 0.95))
# ground effect on the induced velocity in hover, (1 + GE_K (R/Z)^2)^GE_EXP with Z the hub height [as the UH-60 and Bo 105
# models: the UH-60 flight manual's shape], carried into forward flight with Heyson's ratio [NASA TN D-234]
GE_K, GE_EXP, GE_ZR_MIN = 0.3, -0.5, 0.3
# The wing's download in the hover. The ring-vortex wake gives the strips under the discs about 1.45 times the mean
# inflow and Bell's coefficients at -90 deg [CR Table WIV] hardly change with the flap: the model alone gives 5.6 % of
# the thrust at any flap setting, where the XV-15's flight tests measured 16.4 % with the flaps up, 13 % at 40 deg and
# about 10 % at 60-75 deg [RC fig. 18]. What the strips miss (the wake's acceleration round the wing, the fountain where
# the two wakes meet over the fuselage, the deflected flap's smaller area and vertical drag [RC 6.2]) is a factor on the
# strips' coefficients from 40 deg towards normal incidence, per flap setting, FITTED to the XV-15's flight data
# (download.py); the V-22's large-scale test (9.2 % at 67 deg) checks it.
DL_K = {'F4': 2.9, 'F1': 2.9, 'F2': 2.3, 'F3': 1.85}
KEEP = {'aero/rotor/r/thrust-lbs', 'aero/rotor/l/thrust-lbs', 'aero/rotor/r/torque-ftlb', 'aero/rotor/l/torque-ftlb',
        'aero/rotor/r/power-hp', 'aero/rotor/l/power-hp', 'aero/wing/download-lbs', 'aero/wing/lift-lbs',
        # read by Nasal/v22.nas and the gauges
        'drive/nr-pct', 'eng/power-available-hp', 'eng/xmsn-limit-hp', 'ctl/nacelle-cmd-limited-deg', 'ctl/cps-min-deg',
        'ctl/cps-max-deg', 'gov/theta-deg', 'hh/state', 'hh/e-ft', 'hh/h-ft', 'hh/vN-cmd', 'hh/vE-cmd',
        'ap/tcl-cmd', 'ap/nacelle-sched-deg', 'ap/auto-nac-on', 'ap/lat', 'ap/vert', 'ap/spd', 'ap/coupled', 'ap/nav', 'ap/line', 'ap/app',
        'ap/app-dist-ft', 'ap/app-xte-ft', 'ap/app-path-ft', 'ap/v-sel-kt', 'ap/roll-lag-s', 'ap/app-route',
        'ap/tf-on', 'ap/tf-flyup', 'ap/tf-climb-cap-fps', 'ap/tf-ref-ft', 'ap/tf-model-ft', 'ap/tf-v1-fps',
        'ap/tf-v2-fps', 'ap/tf-w', 'ap/tf-tau1-s', 'ap/tf-tau2-s', 'ap/tf-kh'}
INLINE = True


def prefix():
    """the generated files' name: mv22b for the V-22, xv15 for the validation build"""
    return 'xv15' if U.NAME == 'XV-15' else 'mv22b'


def configure(xv15=False):
    global U
    import v22data, xv15data
    U = xv15data if xv15 else v22data
    import drive
    drive.U = U
    return U


def segments(n, r0, r1):
    """equal annulus area segments: (r_mid, dr) in ft, and the edges"""
    a = [r0 * r0 + (r1 * r1 - r0 * r0) * k / n for k in range(n + 1)]
    edges = [math.sqrt(x) for x in a]
    return [(math.sqrt((edges[k] ** 2 + edges[k + 1] ** 2) / 2), edges[k + 1] - edges[k]) for k in range(n)], edges


def chord(r):
    """blade chord (ft) at r (ft): linear between the root's (at the centre) and the tip's"""
    return U.CHORD_ROOT + (U.CHORD_TIP - U.CHORD_ROOT) * r / U.R


def twist(r):
    return float(np.interp(r / U.R, U.TWIST_R, U.TWIST))


def wrap180(e):
    return IFTHEN(GT(e, v(180)), D(e, v(360)), IFTHEN(LT(e, v(-180)), S(e, v(360)), e))


def autonac_schedule():
    """AUTO NAC's nacelle angle against the calibrated airspeed: at each, the most converted angle whose level-flight
    trim's pitch is at most 4 deg (to 150 kt), rising to 7 at 180 kt, never higher at a higher speed (trimmap.py
    --autonac, at the design weight and CG)"""
    d = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'autonac_v22.json')))
    ns, ks, th = d['nacelle'], d['kcas'], d['theta']
    out = []
    for j, k in enumerate(ks):
        tmax = float(np.interp(k, [0, 150, 180], [4.0, 4.0, 7.0]))
        have = [n for i, n in enumerate(ns) if th[i][j] is not None]
        ok = [n for n in have if th[ns.index(n)][j] <= tmax]
        out.append(min(ok) if ok else max(have))
    for j in range(1, len(out)):
        out[j] = min(out[j], out[j - 1])
    return ks, out


def roll_delay():
    """the autopilot's roll into a bank and out of it as a delay, against the calibrated airspeed: the heading a 90 deg
    turn loses against a bank reached at once, int(tan phi_a - tan phi) dt / tan phi_a, less the half of the bank
    reference's ramp at its rate limit (aptest.py rolldelay: the augmentation's own lag, less what its overshoot gives
    back). Rolling out along a ramp mirrors rolling in for a linear loop, with the same delay. None without the
    measurement"""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'roll_delay_v22.json')
    if not os.path.exists(path):
        return [0.0, 300.0], [0.0, 0.0]
    d = json.load(open(path))
    return d['kcas'], d['tau_loop_s']


def rate_ff(axis):
    """the airframe's roll (pitch) rate per inch of lateral (aft) stick against the calibrated airspeed, along AUTO NAC's
    schedule (aptest.py rollff, pitchff: in pitch the steady pull-up's), for the stick fed forward to a rate the
    autopilot commands; none without it"""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', '%s_ff_v22.json' % axis)
    if not os.path.exists(path):
        return None
    d = json.load(open(path))
    return d['kcas'], d['p_per_in' if axis == 'roll' else 'q_per_in'], d.get('peak_ratio')


def climb_power():
    """the power a climb takes against the calibrated airspeed, along AUTO NAC's schedule: P = P_level + b h' + c h'^2
    (h' ft/s) at the reference weight (aptest.py climbpower: trims level, at 1000 ft/min and at full power); without
    it, the climb's own power at 0.8 of it"""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'climb_power_v22.json')
    if not os.path.exists(path):
        return [0.0, 300.0], [U.GW_DESIGN / 440.0] * 2, [0.0, 0.0], [50.0, 50.0], U.GW_DESIGN
    d = json.load(open(path))
    return d['kcas'], d['b_hp_fps'], d['c_hp_fps2'], d['climb_full_fps'], d['weight_lb']


def tf_lag():
    """the climb's response to TF's command against the calibrated airspeed, along AUTO NAC's schedule (aptest.py
    tflag): two first order lags in parallel fitted to its step, the quick one's share and the two time constants, s;
    and the height's tracking gain, 1/s. Without it a 1 s lag and 0.2 (a loop 5 s slow)"""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'tf_lag_v22.json')
    if not os.path.exists(path):
        return [0.0, 300.0], [1.0, 1.0], [1.0, 1.0], [2.0, 2.0], [0.2, 0.2]
    d = json.load(open(path))
    return d['kcas'], d['w'], d['tau1_s'], d['tau2_s'], d['k_h']


def start_trim():
    """the level-flight trims FlightGear's in-air start takes (trimmap.py --start): along AUTO NAC's schedule over the
    calibrated airspeed, the gross weight and the density altitude, the gear up and down; None without them"""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'start_trim_v22.json')
    return json.load(open(path)) if os.path.exists(path) else None


def periodic(rows, *tabs):
    """tables over -180..180 deg (rows) carried on to -360..360, so that an angle needs no wrapping into them"""
    lo = [(a - 360, i) for i, a in enumerate(rows) if 0 <= a < 180]
    hi = [(a + 360, i) for i, a in enumerate(rows) if -180 < a <= 0]
    idx = lo + list(zip(rows, range(len(rows)))) + hi
    return [a for a, i in idx], [[t[i] for a, i in idx] for t in tabs]


def _odd(xs, ys):
    return [-x for x in reversed(xs[1:])] + list(xs), [-y for y in reversed(ys[1:])] + list(ys)


class Model:
    def __init__(s):
        s.g = Graph()
        s.aero = s.g
        s.ctl = FCS('%s controls' % U.NAME)
        s.rot = FCS('%s rotors' % U.NAME)
        s.air = FCS('%s airframe' % U.NAME)
        s.X, s.Y, s.Z, s.L, s.Mm, s.N = [], [], [], [], [], []
        s.segs, s.edges = segments(U.NSEG, U.R_ROOT, U.R)
        # the blades' first torsion mode, from their mass and stiffness distribution (blademodes.py), where the
        # aircraft's data have one (U.BLADE)
        s.tc = None
        if getattr(U, 'BLADE', None):
            import blademodes
            s.tc = blademodes.torsion_constants([a for a, b in s.segs], s.edges, U.OMEGA_100, U.SECTIONS, chord, U.BLADE)
        s.wake_tab = rotorwake.load(U)
        s.inflow_start = {}

    def add(s, F, Mo):
        for lst, x in zip((s.X, s.Y, s.Z, s.L, s.Mm, s.N), tuple(F) + tuple(Mo)):
            if x is not None:
                lst.append(x)

    # ------------------------------------------------------------ geometry helpers
    def arm_rp(s, fs, bl, wl):
        """body-axis vector (ft) from the aerodynamic reference point to a station (expressions allowed)"""
        e = lambda x: x if isinstance(x, str) else v(x)
        return (Q(D(p('metrics/aero-rp-x-in'), e(fs)), v(12)), Q(D(e(bl), p('metrics/aero-rp-y-in')), v(12)),
                Q(D(p('metrics/aero-rp-z-in'), e(wl)), v(12)))

    def arm_cg(s, fs, bl, wl):
        e = lambda x: x if isinstance(x, str) else v(x)
        return (Q(D(p('inertia/cg-x-in'), e(fs)), v(12)), Q(D(e(bl), p('inertia/cg-y-in')), v(12)),
                Q(D(p('inertia/cg-z-in'), e(wl)), v(12)))

    def point_velocity(s, g, tag, fs, bl, wl, wash=None):
        """velocity relative to the air of a point fixed to the airframe (body axes, ft/s), less the induced air velocity
        wash (air moving with the wake: subtracted)"""
        rx, ry, rz = s.arm_cg(fs, bl, wl)
        Pa, Qa, Ra = p('velocities/p-aero-rad_sec'), p('velocities/q-aero-rad_sec'), p('velocities/r-aero-rad_sec')
        w = wash or (None, None, None)
        comps = (S(p('velocities/u-aero-fps'), D(M(Qa, rz), M(Ra, ry))),
                 S(p('velocities/v-aero-fps'), D(M(Ra, rx), M(Pa, rz))),
                 S(p('velocities/w-aero-fps'), D(M(Pa, ry), M(Qa, rx))))
        return [g.fn('%s/V%s-fps' % (tag, a), c if w[i] is None else D(c, w[i])) for i, (a, c) in enumerate(zip('xyz', comps))]

    # ------------------------------------------------------------ flight controls
    def controls(s):
        """The pilot's controls through the mixing to the rotors and the surfaces. The mixing and its phasing with the
        nacelle angle are the XV-15's [CR B-36 to B-38, data/bell301.py]: the collective lever sets both rotors'
        collective (its slope falling to zero in airplane mode), the lateral stick their differential collective, the
        longitudinal stick their longitudinal cyclic, the pedals their differential longitudinal cyclic (falling with
        the airspeed too), all in blade degrees; the elevator, flaperons and rudders follow the stick and pedals in all
        modes. A collective governor ("beta governing") trims both rotors' collective to hold the rotor speed's
        reference. The stability augmentation adds to the pilot's stick and pedals (scas())."""
        c = s.ctl
        c.declare('/controls/engines/engine[0]/throttle', 0.5)
        c.declare('ctl/init', 0)
        c.declare('/controls/flight/nacelle-cmd-deg', 90.0)
        c.declare('ctl/nacelle-deg', 90.0)
        c.declare('ctl/scas-on', 1)
        for n in ('lon', 'lat', 'ped'):
            c.declare('scas/%s-in' % n, 0.0)
        s.hover_hold()
        s.autopilot()
        c.channel('pilot', execrate=CONTROL_EXECRATE)
        c.fn('ctl/init-fast', IFTHEN(LT(p('simulation/sim-time-sec'), v(1.0)), v(1), MIN(v(1), MAX(v(0), p('ctl/init')))))
        init = GT(p('ctl/init-fast'), v(0.5))
        # FlightGear starting in the air (its preset not on the ground; the test programme's trims are, by default):
        # trimmed level flight at the airspeed from the first step, wherever it starts. The nacelles at AUTO NAC's angle
        # for the airspeed (below); the lever at the level-flight power for the airspeed, the gross weight, the density
        # altitude and the gear (trimmap.py --start, along that schedule), the power available's share of it above
        # flight idle (drive.py's demand); the collective governor's integral, the augmentation's pitch reference and its
        # stick's integral at the trim's collective, pitch attitude and stick (governor(), scas()). The lever fixed at
        # 0.65, 0.11 over the level's at FlightGear's loading at 120 kt, climbed at 26 ft/s and was 8 kt slow 10 s on
        c.declare('/sim/presets/onground', 1)
        aloft = c.fn('ctl/start-aloft', M(init, LT(p('/sim/presets/onground'), v(0.5))), desc='1: starting in the air, trimmed')
        # (and trimmed there as the trims are, the airframe held, its integration off, for 5.5 s. Until 0.5 s held still,
        # while the rotors come to the trim's collective and speed: they start in helicopter mode, the drive at 104 %,
        # the collective without the governor's part and the inflow nil, and before they had at 200 kt they windmilled,
        # 35 700 lb of reverse thrust, -2.8 g, the nose kicked up at 12 deg/s and the aircraft climbed at 9 ft/s. Until
        # 4.5 s its flight path held (JSBSim integrates it inertially) and free to pitch, as a model on a wind tunnel's
        # pivot: the nose turns to the trim's pitch, the angle of attack with it, from FlightGear's own start level (6.5
        # deg at 200 kt and 50 000 lb; held again at 3.5 s it was 0.26 deg short, and sank 33 ft in 30 s). Then held
        # again while the stick is trimmed (scas()). Each integrator as it was once let go)
        t_ = p('simulation/sim-time-sec')
        aloft_any = LT(p('/sim/presets/onground'), v(0.5))
        a_, b_, c_ = START_HOLD_S
        hold = c.fn('ctl/start-hold', M(LT(t_, v(c_)), aloft_any), desc='1: starting in the air, the flight path held')
        c.fn('ctl/start-pitching', M(hold, GT(t_, v(a_)), LT(t_, v(b_))), desc='1: starting in the air, the nose turning to the trim\'s')
        c.fn('ctl/start-stick', M(hold, GT(t_, v(b_))), desc='1: starting in the air, the stick trimmed, the airframe held')
        c.declare('ctl/integrator-rate-translational-held', 0)
        c.fn('ctl/start-release', M(GT(p('ctl/integrator-rate-translational-held'), v(0.5)), D(v(1), hold)),
             desc='1: the step an in-air start lets the airframe go')
        held_rot = M(hold, D(v(1), p('ctl/start-pitching')))
        for k, h in (('rate/rotational', held_rot), ('rate/translational', hold), ('position/rotational', held_rot), ('position/translational', hold)):
            prop, kept = 'simulation/integrator/' + k, 'ctl/integrator-%s' % k.replace('/', '-')
            was = kept + '-held'
            c.declare(kept, 0)
            if k != 'rate/translational':
                c.declare(was, 0)
            c.fn(kept + '-next', IFTHEN(GT(p(was), v(0.5)), p(kept), p(prop)), out=kept)
            c.fn(kept + '-set', IFTHEN(GT(h, v(0.5)), v(0), IFTHEN(GT(p(was), v(0.5)), p(kept), p(prop))), out=prop)
            c.fn(was + '-next', h, out=was)
        # (the pitch and roll accelerations on the held airframe, through a lag of 0.05 s against the rotors' vibration,
        # which the stick is trimmed by: scas())
        for r_ in ('q', 'p'):
            n_ = 'ctl/start-%sdot' % r_
            c.declare(n_, 0.0)
            c.fn(n_ + '-next', S(p(n_), M(D(p('accelerations/%sdot-rad_sec2' % r_), p(n_)), MIN(v(1), Q(p('simulation/channel-dt'), v(0.05))))), out=n_)
        st = start_trim() if U.NAME == 'V-22' else None
        s.start_at = None
        if st:
            def at(key):
                tab = lambda g: T3('velocities/vc-kts', 'atmosphere/density-altitude', 'inertia/weight-lbs', st['kcas'], st['density_alt_ft'], st['weight_lb'],
                                   [[list(r) for r in zip(*st['%s_gear_%s' % (key, g)][k])] for k in range(len(st['weight_lb']))])
                return S(tab('up'), M(p('gear/gear-cmd-norm'), D(tab('down'), tab('up'))))
            at.kcas, at.m_lon, at.l_lat = st['kcas'], st['m_lon_ftlb_in'], st['l_lat_ftlb_in']
            s.start_at = at
            # (held, from 0.5 s, the table's a first guess: the collective, the pitch and the lever trimmed on the airframe
            # as the trims trim them, each stepping a frame toward the value its derivative says levels it, lagged by
            # START_TRIM_TAU_S: the collective on the acceleration along the rotors' thrust (its derivative at sea level,
            # trimmap.py --start, times the density's ratio, the thrust's), the pitch target on the acceleration across
            # it (the nose follows: scas()), the lever at the power the rotors then take, the rotor speed held at its
            # reference meanwhile (drive.py). The table's hover power at 10 000 ft was 8 % over the trim's, between its
            # 6 000 and 12 000 ft by a line where the power curves up to the hover ceiling, and its hover pitch 0.4 deg
            # off FlightGear's loading's: the aircraft climbed at 10 ft/s and drifted 6 kt. The accelerations the forces':
            # the body's, less the turn's part while the nose turns (q w, q u), through a lag of 0.05 s against the
            # rotors' vibration)
            import drive
            tau = START_TRIM_TAU_S
            cdt = p('simulation/channel-dt')
            lag = lambda name, x: c.fn(name + '-next', S(p(name), M(D(x, p(name)), MIN(v(1), Q(cdt, v(0.05))))), out=name)
            # (and with gravity taken at the level flight's attitude, the pitch less the flight path's angle: FlightGear
            # starts the nose 0.42 deg up at no angle of attack, climbing, and trimmed on that path at 200 kt the
            # aircraft went on climbing at 2.5 ft/s on the climb's 180 hp. The pitch reference then less the angle too:
            # scas())
            qb = p('velocities/q-rad_sec')
            th_, gm_ = p('attitude/theta-rad'), p('flight-path/gamma-rad')
            ax_ = S(p('accelerations/udot-ft_sec2'), M(qb, p('velocities/w-fps')), M(v(G0), D(SIN(th_), SIN(D(th_, gm_)))))
            az_ = S(D(p('accelerations/wdot-ft_sec2'), M(qb, p('velocities/u-fps'))), M(v(G0), D(COS(D(th_, gm_)), COS(th_))))
            for n_ in ('ctl/start-a-along', 'ctl/start-a-across', 'ctl/start-power-hp', 'ctl/start-coll-deg', 'ctl/start-theta-deg'):
                c.declare(n_, 0.0)
            lag('ctl/start-a-along', D(M(ax_, p('ctl/cos-nac')), M(az_, p('ctl/sin-nac'))))
            lag('ctl/start-a-across', S(M(ax_, p('ctl/sin-nac')), M(az_, p('ctl/cos-nac'))))
            guess = GT(M(hold, LT(t_, v(a_))), v(0.5))
            # (the power through a lag of 1 s from the table's: at 120 kt and 70 deg it swings 120 hp every 1.5 s at a
            # held collective, the rotors' own, and through 0.05 s the lever was let go 100 hp short, sinking at 1.4
            # ft/s; 1 s leaves a quarter of it)
            c.fn('ctl/start-power-next', IFTHEN(guess, at('power_hp'), S(p('ctl/start-power-hp'), M(D(p('drive/power-required-hp'), p('ctl/start-power-hp')),
                 MIN(v(1), Q(cdt, v(1.0)))))), out='ctl/start-power-hp')
            s_coll = M(T1('velocities/vc-kts', st['kcas'], st['a_coll_fps2_deg']), p('atmosphere/sigma'))
            s_pitch = T1('velocities/vc-kts', st['kcas'], st['a_pitch_fps2_deg'])
            cs, ts = p('ctl/start-coll-deg'), p('ctl/start-theta-deg')
            c.fn('ctl/start-coll-next', IFTHEN(guess, at('coll_deg'), IFTHEN(GT(hold, v(0.5)),
                 D(cs, M(Q(cdt, v(tau)), Q(p('ctl/start-a-along'), s_coll))), cs)), out='ctl/start-coll-deg',
                 desc='an in-air start\'s collective, deg: the table\'s, trimmed')
            c.fn('ctl/start-theta-next', IFTHEN(guess, at('theta_deg'), IFTHEN(GT(p('ctl/start-pitching'), v(0.5)),
                 S(ts, M(Q(cdt, v(tau)), D(D(p('attitude/theta-deg'), Q(p('ctl/start-a-across'), s_pitch)), ts))), ts)), out='ctl/start-theta-deg',
                 desc='an in-air start\'s pitch, deg: the table\'s, trimmed')
            thr = '/controls/engines/engine[0]/throttle'
            avail = M(v(2), MAX(v(1), MIN(p('eng/power-available-hp'), p('eng/xmsn-limit-hp'))))
            lever = lambda pw: MAX(v(0), MIN(v(1), Q(D(Q(pw, avail), v(drive.IDLE)), v(1 - drive.IDLE))))
            c.fn('ctl/tcl-start', IFTHEN(GT(hold, v(0.5)), IFTHEN(guess, lever(at('power_hp')), lever(p('ctl/start-power-hp'))), p(thr)), out=thr)
        tcl = c.fn('ctl/tcl-norm', p('/controls/engines/engine[0]/throttle'), clip=('0', '1'),
                   desc='thrust control lever (collective / power), 0 down .. 1 up (FlightGear: the throttle axis)')
        # JSBSim's rudder is positive left (FlightGear hands it the negated pedal): negated back
        lon = c.fn('ctl/lon-norm', S(p('fcs/elevator-cmd-norm'), p('fcs/pitch-trim-cmd-norm')), clip=('-1', '1'), desc='+ forward')
        lat = c.fn('ctl/lat-norm', S(p('fcs/aileron-cmd-norm'), p('fcs/roll-trim-cmd-norm')), clip=('-1', '1'), desc='+ right')
        ped = c.fn('ctl/ped-norm', NEG(S(p('fcs/rudder-cmd-norm'), p('fcs/yaw-trim-cmd-norm'))), clip=('-1', '1'), desc='+ right')
        # in inches of the XV-15's controls [CR B-6], with the augmentation's inputs
        sc = p('ctl/scas-on')
        c.declare('scas/stick-ff', 1.0)
        c.declare('scas/lon-trim-in', 0.0)
        ff = S(v(1), M(sc, D(p('scas/stick-ff'), v(1))))      # the stick's direct path (scas())
        c.fn('ctl/lon-in', S(M(lon, v(B.X_LON), ff), M(sc, S(p('scas/lon-in'), p('scas/lon-trim-in')))),
             clip=('-%g' % B.X_LON, '%g' % B.X_LON))
        c.fn('ctl/lat-in', S(M(lat, v(B.X_LAT), ff), M(sc, p('scas/lat-in'))), clip=('-%g' % B.X_LAT, '%g' % B.X_LAT))
        c.fn('ctl/ped-in', S(M(ped, v(B.X_PED)), M(sc, p('scas/ped-in'))), clip=('-%g' % B.X_PED, '%g' % B.X_PED))
        c.channel('nacelles', execrate=CONTROL_EXECRATE)
        # the nacelle angle follows its command at the conversion rate; it starts at the command. FlightGear starting it
        # in the air (its preset not on the ground; the test programme's trims are, by default), the command (so the
        # angle) where AUTO NAC's schedule has it for the airspeed, from the first step: v22.nas's own set-up comes with
        # fdm-initialized, which may be after the first second, and a start at 120 kt flew that long with the nacelles
        # vertical, edgewise into the air, and pitched up to 29 deg
        sk, sn = autonac_schedule()
        c.fn('ctl/nacelle-cmd-start', IFTHEN(GT(aloft, v(0.5)), T1('velocities/vc-kts', sk, sn), p('/controls/flight/nacelle-cmd-deg')),
             out='/controls/flight/nacelle-cmd-deg')
        nac = p('ctl/nacelle-deg')
        cmd = MAX(v(U.NACELLE_MIN), MIN(v(U.NACELLE_MAX), p('/controls/flight/nacelle-cmd-deg')))
        if U.CPS:
            # conversion protection [PD fig. 5]: the command held within the corridor's boundaries at the equivalent
            # airspeed (ctl/cps-on; the test programme switches it off)
            c.declare('ctl/cps-on', 1)
            (lv, ln), (uv, un) = U.CPS
            on = GT(p('ctl/cps-on'), v(0.5))
            lo_ = c.fn('ctl/cps-min-deg', IFTHEN(on, T1('velocities/ve-kts', lv, ln), v(U.NACELLE_MIN)))
            hi_ = c.fn('ctl/cps-max-deg', IFTHEN(on, T1('velocities/ve-kts', uv, un), v(U.NACELLE_MAX)))
            cmd = c.fn('ctl/nacelle-cmd-limited-deg', MAX(lo_, MIN(hi_, cmd)))
        # this step's change, and its rate, from the angle before it: the next angle overwrites ctl/nacelle-deg
        step = M(v(U.NACELLE_RATE), p('simulation/channel-dt'))
        dn = c.fn('ctl/nacelle-step-deg', IFTHEN(init, D(cmd, nac), MAX(NEG(step), MIN(step, D(cmd, nac)))))
        c.fn('ctl/nacelle-rate-rad_sec', IFTHEN(init, v(0), Q(dn, M(v(R2D), p('simulation/channel-dt')))))
        c.fn('ctl/nacelle-next', S(nac, dn), out='ctl/nacelle-deg')
        bm = c.fn('ctl/beta-m-deg', D(v(90), nac), desc='Bell\'s mast angle, 0 helicopter .. 90 airplane')
        c.fn('ctl/sin-nac', SIN(M(nac, v(1 / R2D))))
        c.fn('ctl/cos-nac', COS(M(nac, v(1 / R2D))))
        c.channel('mixing', execrate=CONTROL_EXECRATE)
        bmt = lambda ys: T1('ctl/beta-m-deg', B.CTL_BETAM, ys)
        # collective at 0.75 R: Bell's lever schedule is the blade's pitch at the centre with its twist table (40.9 deg
        # root to tip, 6.375 at 0.75 R) [CR B-4, B-38]
        th75 = c.fn('ctl/coll-ff-deg', S(D(bmt(B.TH0_LL), v(40.9 - 6.375)), M(bmt(B.DTH0_DXCOL), v(B.X_COL), tcl)),
                    desc='collective at 0.75 R from the lever, deg')
        # (an in-air start's own while it holds the airframe and trims, controls(): the governor's integral tracks it less
        # the lever's for the release. Summed a step apart, the lever, which follows the power, moved the collective,
        # which moved the power: at 120 kt they swung 100 hp every 1.5 s, and it was let go where they were)
        coll = S(th75, p('gov/theta-deg'))
        if s.start_at:
            coll = IFTHEN(GT(p('ctl/start-hold'), v(0.5)), p('ctl/start-coll-deg'), coll)
        c.fn('ctl/theta75-deg', coll, desc='both rotors\' collective, with the governor\'s')
        c.fn('ctl/dcoll-deg', M(bmt(B.DTH0_DXLT), p('ctl/lat-in')), desc='differential collective, + right rotor down')
        c.fn('ctl/b1s-deg', M(bmt(B.DB1_DXLN), p('ctl/lon-in')), desc='longitudinal cyclic, + forward')
        kts = 'velocities/vc-kts'
        db1 = T2(kts, 'ctl/beta-m-deg', [0, 60, 80, 100], B.CTL_BETAM,
                 [B.DB1_DXPD[0], B.DB1_DXPD[0], B.DB1_DXPD[80], B.DB1_DXPD[100]])
        c.fn('ctl/db1s-deg', M(db1, p('ctl/ped-in')), desc='differential longitudinal cyclic, + right rotor aft (nose right)')
        # per rotor: collective (+ the right rotor's down with right stick), longitudinal cyclic (+ forward)
        for side, m in SIDES:
            c.fn('ctl/%s/theta75-cmd-deg' % side, D(p('ctl/theta75-deg'), M(v(m), p('ctl/dcoll-deg'))))
            c.fn('ctl/%s/b1s-cmd-deg' % side, D(p('ctl/b1s-deg'), M(v(m), p('ctl/db1s-deg'))))
        # surfaces: elevator (+ trailing edge up), rudders (+ trailing edge left: nose right), flaperons' differential
        # (+ right flaperon up)
        c.fn('ctl/elevator-cmd-deg', M(v(-B.DE_DXLN), p('ctl/lon-in'), v(-1)), desc='+ trailing edge up (aft stick)')
        c.fn('ctl/rudder-cmd-deg', M(v(B.DR_DXPD), p('ctl/ped-in')))
        c.fn('ctl/aileron-cmd-deg', M(v(U.FLAPERON_AILERON / B.X_LAT), p('ctl/lat-in')))
        # flaperons' symmetric schedule against airspeed and nacelle angle (estimate; the V-22's is not public): the
        # hover setting with the nacelles up at low speed; through the conversion and in airplane mode at low speed
        # FLAPERON_CONV (40 deg on the V-22: its conversion corridor's lower boundary is the wing's stall "at 40 deg flap"
        # [PD]; Dunford's level-flight charts carry airplane-mode curves with flaps), retracting to 0 between 120 and
        # 200 kt
        fh, fc = U.FLAPERON_HOVER, U.FLAPERON_CONV
        fsch = T2(kts, 'ctl/nacelle-deg', [0, 60, 100, 120, 160, 200], [0, 30, 60, 75, 90],
                  [[fc, fc, fc, fh, fh], [fc, fc, fc, fh, fh], [fc, fc, fc, fc, fc], [fc, fc, fc, fc, fc],
                   [0.5 * fc, 0.5 * fc, 0.5 * fc, 0.5 * fc, 0.5 * fc], [0, 0, 0, 0, 0]])
        c.declare('ctl/flaperon-override', 0)
        c.declare('ctl/flaperon-set-deg', 0.0)
        c.fn('ctl/flaperon-cmd-deg', IFTHEN(GT(p('ctl/flaperon-override'), v(0.5)), p('ctl/flaperon-set-deg'), fsch))
        c.channel('servos', execrate=CONTROL_EXECRATE)
        # swashplate and surface actuators: first-order, 0.05 s (estimate)
        for n in ['r/theta75', 'l/theta75', 'r/b1s', 'l/b1s', 'elevator', 'rudder', 'aileron', 'flaperon']:
            c.lag('ctl/%s-deg' % n, 'ctl/%s-cmd-deg' % n, 0.05)
        s.governor()
        s.scas()

    def governor(s):
        """Collective governor ("beta governing" [PD fig. 2d]; the XV-15's rpm governor acts in all modes [TX 8.2.3]):
        proportional and integral on the rotor speed error, % of 100 %, in deg of collective, within Bell's limits
        [CR table CVIII]; the reference is the schedule's (104 % in helicopter and conversion modes, 84 % in airplane
        mode for the V-22) with the pilot's trim. Started at the trim's value."""
        c = s.ctl
        c.channel('governor', execrate=CONTROL_EXECRATE)
        c.declare('gov/int-deg', 0.0)
        c.declare('gov/kp', 0.6)              # deg per % (Bell's 0.1 deg/rpm at 589 rpm: 0.59)
        c.declare('gov/ki', 0.4)              # deg per % s
        c.declare('/controls/rotor/nr-trim-pct', 0.0)
        init = GT(p('ctl/init-fast'), v(0.5))
        # rotor speed reference: airplane mode's below 1 deg of nacelle angle, rate limited (2 %/s)
        c.declare('gov/nr-ref-pct', U.NR_HELO)
        tgt = S(IFTHEN(LT(p('ctl/nacelle-deg'), v(1.0)), v(U.NR_APLN), v(U.NR_HELO)), p('/controls/rotor/nr-trim-pct'))
        st = M(v(2.0), p('simulation/channel-dt'))
        ref = c.fn('gov/nr-ref-next', IFTHEN(init, tgt, S(p('gov/nr-ref-pct'), MAX(NEG(st), MIN(st, D(tgt, p('gov/nr-ref-pct')))))),
                   out='gov/nr-ref-pct')
        nr = M(v(100 / U.OMEGA_100), p('rotor/omega-rad_sec'))
        # the error is the overspeed: a rotor faster than its reference takes more pitch (more torque), a slower one less
        err = c.fn('gov/err-pct', D(nr, ref))
        lo, hi = B.GOV_LIMITS
        # it engages as the rotor comes up to speed (from 19 to 9 % below its reference: 85 to 95 % in helicopter mode),
        # its integral held at zero below: on a start the blades stay at the lever's flat pitch
        ga = c.fn('gov/engaged', MAX(v(0), MIN(v(1), Q(S(D(nr, ref), v(19)), v(10)))))
        # (starting in the air, the start's collective less the lever's while it holds the airframe: controls())
        held, fixed = p('gov/int-deg'), init
        if s.start_at:
            fixed = MAX(init, GT(p('ctl/start-hold'), v(0.5)))
            held = IFTHEN(GT(p('ctl/start-hold'), v(0.5)), D(p('ctl/start-coll-deg'), p('ctl/coll-ff-deg')), held)
        c.fn('gov/int-next', IFTHEN(fixed, held, IFTHEN(LT(ga, v(0.01)), v(0),
                                                                MAX(v(lo), MIN(v(hi), S(p('gov/int-deg'), M(p('gov/ki'), ga, err, p('simulation/channel-dt'))))))),
             out='gov/int-deg')
        c.fn('gov/theta-deg', M(ga, MAX(v(lo), MIN(v(hi), S(p('gov/int-deg'), M(p('gov/kp'), err))))))

    def scas(s):
        """Stability and control augmentation, for the FlightGear aircraft (the validation runs the bare airframe,
        ctl/scas-on = 0). The V-22's fly-by-wire control laws are not public (it holds attitude in all modes [PD]), nor
        are the XV-15 SCAS's tables used: these are proportional-integral loops through the mixing, in inches of stick
        and pedal, with half the stick's travel and the pedals' full travel as their authority (the V-22 flies by wire). Rate damping in
        the three axes, for a rate time constant of about 0.5 s in the hover (the hover's control power per inch: pitch
        8.7, roll 8.1, yaw 4.8 deg/s^2, from the test programme's pulse responses), and attitude loops in pitch and roll
        of about 1.2 rad/s. A feed-forward of the level-flight trim's stick over the nacelle angle and the airspeed
        (data/trim_ff_v22.json, from trimmap.py at the design weight and CG) centres the stick's trim through the
        conversion, and an integrator (3 s) takes up the rest of its offset. The response types follow a
        common rotorcraft scheme, attitude command at low speed and rate command above it (as Boeing's US patent
        5,596,499), switched without a transient because both use the same reference: the commanded attitude is the
        reference less the stick (with FlightGear's trim) times scas/{theta,phi}-cmd-deg. Below 50 kt (changing over at
        45 slowing, 55 accelerating) with the nacelles above 60 deg, the stick commands an attitude (20 deg of pitch, 25 of bank at full stick; one keyboard step, 1 and
        1.25 deg; its direct path to the controls cut to 30 %, back to full by 60 kt) and the heading is held while the
        pedals rest at their centre; above, the stick commands a rate, the
        reference following the attitude, and the attitude it comes to rest at, back at its centre (within scas/detent),
        is held. On the ground the references follow the aircraft. The gains fall with the dynamic pressure, against the
        surfaces' growing power."""
        c = s.ctl
        c.channel('scas', execrate=CONTROL_EXECRATE)
        c.declare('scas/kq', 13.0)        # in per rad/s
        c.declare('scas/ktheta', 10.0)    # in per rad
        c.declare('scas/ki-theta', 3.0)   # in per rad s
        c.declare('scas/kp', 20.0)        # roll: the hover hold's sideways loop works through it (14 and 10 overshot 67 %)
        c.declare('scas/kphi', 20.0)
        c.declare('scas/ki-phi', 3.0)
        c.declare('scas/kr', 24.0)
        c.declare('scas/kpsi', 10.0)
        c.declare('scas/theta-cmd-deg', 20.0)   # attitude command at full stick
        c.declare('scas/phi-cmd-deg', 25.0)
        c.declare('scas/detent', 0.05)    # stick or pedals within this of their centre: at rest (fraction of travel)
        c.declare('scas/auth-in', 0.5 * B.X_LON)            # stick (pitch, roll), in
        c.declare('scas/auth-ped-in', B.X_PED)              # pedals: the full travel, which the yaw's rate command needs in a
                                                            # tailwind (half of it let a 35 kt one turn the aircraft)
        auth, auth_ped = p('scas/auth-in'), p('scas/auth-ped-in')
        # gain schedule: 1 in the hover, falling with the dynamic pressure against the surfaces' growing power, set on
        # Bell's surfaces at 0.5 at 30 psf (about 95 kt), 0.15 at 170 psf (230 kt). Each axis's corner moves with its
        # surface's power at a dynamic pressure since Ferguson's revision of them [GTRS], so that each loop crosses over
        # where it did: the elevator's (its effectiveness, the tail's dynamic pressure) x0.73, the ailerons' x1.2, the
        # rudders' x0.70
        corner = {'theta': B.TAU_E / (GTRS.TAU_E * GTRS.K_HNU), 'phi': B.CL_DA / GTRS.CL_DA, 'psi': B.TAU_R / GTRS.TAU_R}
        kx = {ax: c.fn('scas/gain-%s' % ax, Q(v(1), S(v(1), Q(p('aero/qbar-psf'), v(30 * f))))) for ax, f in corner.items()}
        # the rate dampers, on the body rates less the rates the autopilot commands while it holds the axis (its
        # reference's ramp: damped, it lagged 10 deg behind a 5 deg/s roll at 200 kt, and the integrator wound up
        # meanwhile overshot the bank by 5 deg and took 20 s to unwind)
        c.fn('scas/q-net', D(p('velocities/q-rad_sec'), M(p('ap/theta-ref-rate'), p('ap/theta-on'))))
        c.fn('scas/p-net', D(p('velocities/p-rad_sec'), M(p('ap/phi-ref-rate'), p('ap/phi-on'))))
        c.washout('scas/q-wo', 'scas/q-net', 5.0)
        c.washout('scas/p-wo', 'scas/p-net', 5.0)
        init = GT(p('ctl/init-fast'), v(0.5))
        air = c.fn('scas/airborne', M(LT(S(*[p('gear/unit[%d]/WOW' % i) for i in range(3)]), v(0.5)), LT(p('ctl/init-fast'), v(0.5))))
        # (with 5 kt of hysteresis about 50: at a speed held near it the response type would change back and forth)
        c.declare('scas/attitude-command', 1)
        vk = p('velocities/vc-kts')
        acah = c.fn('scas/attitude-command-next', M(GT(p('ctl/nacelle-deg'), v(60)), IFTHEN(GT(p('scas/attitude-command'), v(0.5)), LT(vk, v(55)), LT(vk, v(45)))),
                    out='scas/attitude-command', desc='1: attitude command and heading hold (low speed), 0: rate command')
        # the stick's direct path to the controls, cut to 30 % where it commands an attitude, so that the attitude
        # loop sets the response; back to full by 60 kt and with the nacelles below 50 deg
        low = M(MAX(v(0), MIN(v(1), Q(D(v(60), p('velocities/vc-kts')), v(20)))), MAX(v(0), MIN(v(1), Q(D(p('ctl/nacelle-deg'), v(50)), v(20)))))
        hold_on = GT(p('hh/on'), v(0.5))
        c.fn('scas/stick-ff-calc', M(D(v(1), hold_on), D(v(1), M(v(0.7), low))), out='scas/stick-ff')
        # the trim feed-forward: the level-flight trim's stick at the nacelle angle and the airspeed (trimmap.py), so
        # that the stick's centre is trimmed flight through the conversion; carried on past 90 deg (the nacelles tilted
        # aft of vertical, as in a hover stop) along its slope from 75 to 90. Its airspeed is the forward one (equivalent,
        # along the body's x axis): in rearward or sideward flight it is the hover's, where the airspeed's magnitude
        # gave the forward flight's forward stick and pitched the nose down as the aircraft went backwards
        # (that airspeed the autopilot's too, in either build)
        c.fn('scas/ff-kts', M(Q(p('velocities/u-aero-fps'), v(1.6878)), SQ(p('atmosphere/sigma'))), desc='forward airspeed, equivalent')
        tf = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'trim_ff_v22.json')
        if U.NAME == 'V-22' and os.path.exists(tf):
            tm = json.load(open(tf))
            ns, lon_t = list(tm['nacelle']), [list(r) for r in tm['lon']]
            if ns[-1] < U.NACELLE_MAX:
                f = (U.NACELLE_MAX - ns[-1]) / (ns[-1] - ns[-2])
                lon_t.append([b + (b - a) * f for a, b in zip(lon_t[-2], lon_t[-1])]); ns.append(U.NACELLE_MAX)
            c.fn('scas/lon-trim-calc', M(v(B.X_LON), T2('ctl/nacelle-deg', 'scas/ff-kts', ns, tm['kcas'], lon_t)),
                 out='scas/lon-trim-in')
        rest = lambda prop: LT(ABS(p(prop)), p('scas/detent'))
        err = {}
        # (axis, stick at rest, stick with trim, attitude, sign of the command)
        for ax, stick, cmd, att, sg in (('theta', 'fcs/elevator-cmd-norm', 'ctl/lon-norm', 'attitude/theta-rad', -1),
                                        ('phi', 'fcs/aileron-cmd-norm', 'ctl/lat-norm', 'attitude/phi-rad', 1)):
            ref, integ = 'scas/%s-ref-rad' % ax, 'scas/%s-int-in' % ax
            c.declare(ref, 0.0)
            c.declare(integ, 0.0)
            hold = c.fn('scas/%s-hold' % ax, M(air, MAX(acah, rest(stick))), desc='1: the reference held')
            # (the stick and its trim command no attitude while the hover hold or the autopilot holds the axis: their
            # references are the attitudes themselves)
            stk = M(v(sg / R2D), p('scas/%s-cmd-deg' % ax), p(cmd), D(v(1), hold_on), D(v(1), p('ap/%s-on' % ax)))
            # (starting in the air, controls(): the pitch's reference the attitude throughout, so that the attitude's term
            # is nil, and the stick's integral trimmed as the trims trim it, on the airframe's pitch acceleration over the
            # stick's moment (trimmap.py --start), closing in tau = 0.25 s (in 0.1 s, with the rotors' own lag after the
            # stick, it rang). At first the trim's (the table's, the design CG's: the nacelles' with them along the
            # schedule, 5.5 in aft of it the stick held 20 deg/s^2 nose up at 200 kt). While the nose is free, to the
            # acceleration that brings the pitch to the trim's for the weight critically damped at 1/(2 tau) rad/s,
            # the loop around the trim's half as quick: within 0.1 deg of a 6.5 deg turn by 4.5 s. (The augmentation's
            # attitude loop turned it with the integral the table's, the 20 deg/s^2 against it: 1.4 deg short, and let go
            # there at 50 000 lb it climbed at 19 ft/s.) Then held to the pitch acceleration nil. The trim's pitch is the
            # table's, the design CG's: within 0.1 deg of an exact trim's at 6 000 lb of cargo and the tanks full)
            start = bool(s.start_at)
            rnext = IFTHEN(hold_on, p('hh/%s-ref-rad' % ax), IFTHEN(GT(p('ap/%s-on' % ax), v(0.5)), p('ap/%s-ref-rad' % ax),
                           IFTHEN(GT(hold, v(0.5)), p(ref), D(p(att), stk))))
            if start:
                # (let go, the pitch's reference less the flight path's angle: controls())
                let = D(p(att), p('flight-path/gamma-rad')) if ax == 'theta' else p(att)
                rnext = IFTHEN(GT(p('ctl/start-release'), v(0.5)), let, IFTHEN(GT(p('ctl/start-hold'), v(0.5)), p(att), rnext))
            c.fn('scas/%s-ref-next' % ax, rnext, out=ref)
            err[ax] = c.fn('scas/%s-err-rad' % ax, D(p(att), S(p(ref), stk)), desc='attitude less its command')
            run = IFTHEN(GT(hold, v(0.5)), MAX(NEG(auth), MIN(auth, S(p(integ), M(kx[ax], p('scas/ki-' + ax), err[ax], p('simulation/channel-dt'))))),
                         p(integ))
            i0 = v(0)
            if start:
                # (the roll's the same, to the wings level: in FlightGear at 200 kt, the integral at nil, the roll drifted
                # to 1.5 deg while the nose was free, and the augmentation's own integral took 10 s to take it back. Its
                # integral subtracts from the stick: scas/lat-calc)
                tau = START_TRIM_TAU_S
                w = 1 / (2 * tau)
                if ax == 'theta':
                    mt, inertia, acc, rate, tgt, sg = s.start_at.m_lon, 'iyy', 'ctl/start-qdot', 'velocities/q-rad_sec', M(v(1 / R2D), p('ctl/start-theta-deg')), 1
                    guess = D(M(v(B.X_LON), s.start_at('lon')), p('scas/lon-trim-in'))
                else:
                    mt, inertia, acc, rate, tgt, sg = s.start_at.l_lat, 'ixx', 'ctl/start-pdot', 'velocities/p-rad_sec', v(0), -1
                    guess = v(0)
                mom = T1('velocities/vc-kts', s.start_at.kcas, mt)
                want = M(p('ctl/start-pitching'), D(M(v(w * w), D(tgt, p(att))), M(v(2 * w), p(rate))))
                trim = D(p(integ), M(v(sg), Q(p('simulation/channel-dt'), v(tau)), Q(M(D(p(acc), want), p('inertia/%s-slugs_ft2' % inertia)), mom)))
                first = LT(p('simulation/sim-time-sec'), v(START_HOLD_S[0]))
                run = IFTHEN(GT(p('ctl/start-hold'), v(0.5)), IFTHEN(first, guess, trim), run)
                c.fn('scas/%s-int-next' % ax, IFTHEN(GT(p('ctl/start-hold'), v(0.5)), run, IFTHEN(init, i0, run)), out=integ)
            else:
                c.fn('scas/%s-int-next' % ax, IFTHEN(init, i0, run), out=integ)
        # heading hold: at low speed, pedals at rest, in the air
        c.declare('scas/psi-ref-rad', 0.0)
        hh = c.fn('scas/psi-hold', M(air, acah, rest('fcs/rudder-cmd-norm'), D(v(1), p('ap/phi-on'))))
        c.fn('scas/psi-ref-next', IFTHEN(GT(hh, v(0.5)), p('scas/psi-ref-rad'), p('attitude/psi-rad')), out='scas/psi-ref-rad')
        dpsi = D(p('attitude/psi-rad'), p('scas/psi-ref-rad'))
        epsi = c.fn('scas/psi-err-rad', M(hh, ATAN2(SIN(dpsi), COS(dpsi))))
        # signs: + forward stick pitches the nose down, + right stick rolls right, + right pedal yaws right
        # (and the stick the airframe needs for the roll or pitch rate the autopilot commands, fed forward: rate_ff. In
        # pitch the rate its reference moves at and the rate that closes the error in tau (model following: the
        # augmentation's own attitude gain, 1.8 in/rad at 200 kt against the airframe's 0.061 rad/s per inch, closed an
        # error in 9 s and had to build 10 deg of one for a 0.2 g pull-up). The airframe's pitch rate leads its
        # sustained one, its first peak R times it (1.3 at 60 kt, 3.9 at 200, 6.9 at 150 in the conversion): K (T s + 1)
        # with T about R over the short period's frequency (2 rad/s, 2.1 at 200 kt). The inverse is the sustained gain
        # behind a lag of T, and tau the loop's own lag over its stiffness at high frequency, R/tau under 1 by half that
        # frequency: tau = R s. (The bare sustained gain was R times too stiff at the short period: 1.5 s rang at 1 Hz,
        # and at 150 kt it diverged)
        qf = rate_ff('pitch')
        if qf and qf[2]:
            R = [max(1.0, r) for r in qf[2]]
            tau = T1('velocities/vc-kts', qf[0], R)
            lagT = T1('velocities/vc-kts', qf[0], [r / 2.0 for r in R])
            qdes = S(p('ap/theta-ref-rate'), Q(NEG(err['theta']), tau))
            c.declare('scas/ap-pitch-ff', 1)
            c.declare('scas/qff-in', 0.0)
            x = M(qdes, p('ap/theta-on'), p('scas/ap-pitch-ff'), Q(v(1), T1('velocities/vc-kts', qf[0], qf[1])))
            c.fn('scas/qff-next', S(p('scas/qff-in'), M(D(x, p('scas/qff-in')), MIN(v(1), Q(p('simulation/channel-dt'), lagT)))), out='scas/qff-in')
            qff = p('scas/qff-in')
        else:
            qff = v(0)
        c.fn('scas/lon-calc', D(S(M(kx['theta'], S(M(p('scas/kq'), p('scas/q-wo')), M(p('scas/ktheta'), err['theta']))), p('scas/theta-int-in')), qff),
             clip=('-scas/auth-in', 'scas/auth-in'), out='scas/lon-in')
        rf = rate_ff('roll')
        pff = M(p('ap/phi-ref-rate'), p('ap/phi-on'), Q(v(1), T1('velocities/vc-kts', rf[0], rf[1]))) if rf else v(0)
        c.fn('scas/lat-calc', S(NEG(S(M(kx['phi'], S(M(p('scas/kp'), p('scas/p-wo')), M(p('scas/kphi'), err['phi']))), p('scas/phi-int-in'))), pff),
             clip=('-scas/auth-in', 'scas/auth-in'), out='scas/lat-in')
        # yaw: at low speed rate command, direction hold (the pedals command a yaw rate, scas/r-cmd-dps at full travel,
        # held against the wind's weathervaning by an integrator; at rest the heading holds), the response type the
        # hover's control laws use [ADS-33E-PRF; the AH-64D's]; above, the yaw rate damped about a coordinated turn's.
        # A rate damper alone let a steady yaw moment (a tailwind on the fins) turn the aircraft and made the pedals weak
        c.declare('scas/r-cmd-dps', 30.0)
        c.declare('scas/ki-r', 8.0)       # in per rad of yaw (the rate error integrated)
        c.declare('scas/r-int-in', 0.0)
        # Turns are coordinated: the yaw rate held is the one of a coordinated turn at the bank (with the rate command,
        # while the autopilot banks; in rate command always), where a washed-out damper let them slip 20 deg at 50 kt
        coord = c.fn('scas/r-coord-rad_sec', Q(M(v(G0), SIN(p('attitude/phi-rad')), COS(p('attitude/theta-rad'))), MAX(p('velocities/vt-fps'), v(50))))
        rcmd = c.fn('scas/r-cmd-rad_sec', M(S(M(p('ctl/ped-norm'), p('scas/r-cmd-dps'), v(1 / R2D)), M(p('ap/phi-on'), coord)), acah))
        rerr = c.fn('scas/r-err-rad_sec', IFTHEN(GT(acah, v(0.5)), D(p('velocities/r-rad_sec'), rcmd), D(p('velocities/r-rad_sec'), coord)))
        cdt = p('simulation/channel-dt')
        c.fn('scas/r-int-next', IFTHEN(init, v(0), IFTHEN(GT(M(air, acah), v(0.5)),
             MAX(NEG(auth_ped), MIN(auth_ped, D(p('scas/r-int-in'), M(kx['psi'], p('scas/ki-r'), rerr, cdt)))),
             M(p('scas/r-int-in'), MAX(v(0), D(v(1), Q(cdt, v(2))))))), out='scas/r-int-in')
        c.fn('scas/ped-calc', S(NEG(M(kx['psi'], S(M(p('scas/kr'), rerr), M(p('scas/kpsi'), epsi)))), p('scas/r-int-in')),
             clip=('-scas/auth-ped-in', 'scas/auth-ped-in'), out='scas/ped-in')

    def autopilot(s):
        """The autopilot: a flight director whose commands the augmentation flies when it is coupled (CPLD), as the
        V-22's flight director panel and coupling work [Boeing's AUTO NAC patent]. HDG HOLD banks to the heading bug,
        WYPT flies FlightGear's route manager's route (its GPS's legs, turning before each waypoint);
        ALT HOLD climbs or descends to the ALT knob's altitude and holds it; SPD HOLD takes the SPD knob's airspeed,
        towards which a reference moves at no more than 2 kt/s faster, 1.5 kt/s slower. With a speed mode the lever
        and the pitch share the energy as Boeing's total energy control does [Lambregts]: the lever its rate (climb and
        acceleration), the pitch its share between the two; without one the pitch flies the altitude and the lever is
        the pilot's. AUTO NAC sets the nacelles for the speed reference along a schedule from the model's own
        level-flight trims (the most converted nacelle angle that keeps the pitch under 4 deg, 7 at 180 kt:
        trimmap.py --autonac), within the conversion corridor. Coupled, it drives the lever and nacelle command as
        the hover hold does (the pilot's lever flies through; moving the nacelles takes AUTO NAC off); the stick out
        of its detent or the hover hold takes it off. Gains are properties (ap/*)."""
        c = s.ctl
        KT, AF, ST = 1.6878, '/instrumentation/mv22b/afcs/', '/autopilot/settings/'
        for n in ('cpld', 'hdg-hold', 'alt-hold', 'spd-hold', 'auto-nac', 'wypt', 'vor-ils', 'appr', 'hvr-hold', 'tf', 'ga'):
            c.declare(AF + n, 0)
        c.declare(ST + 'target-agl-ft', 0.0)
        for n, x in (('line-valid', 0), ('line-lat-deg', 0.0), ('line-lon-deg', 0.0), ('line-course-deg', 0.0), ('line-gs', 0),
                     ('line-angle-deg', 3.0), ('line-elev-ft', 0.0), ('app-land', 0), ('app-state', 0), ('gp', 0), ('line-cap', 0)):
            c.declare('ap/' + n, x)
        GPS = '/instrumentation/gps/wp/'
        c.declare('/autopilot/route-manager/active', 0)
        c.declare(GPS + 'leg-true-course-deg', 0.0)
        c.declare(GPS + 'wp[1]/course-error-nm', 0.0)
        c.declare(GPS + 'wp[1]/bearing-true-deg', 0.0)
        for n, x in (('tf-hdot-fps', 0.0), ('tf-valid', 0), ('tf-cmd-fps', 0.0), ('tf-was', 0), ('tf-ride-g', 0.2), ('tf-climb-max-fps', 50.0), ('tf-climb-cap-fps', 0.0),
                     ('tf-flyup-g', 0.5), ('tf-band-kt', 20.0), ('tf-ref-ft', 0.0), ('tf-model-ft', 0.0), ('tf-v1-fps', 0.0), ('tf-v2-fps', 0.0), ('tf-track', 1), ('vs-cmd-fps', 0.0), ('vert-accel-g', 0.1)):
            c.declare('ap/' + n, x)
        for n, x in (('direct', 0), ('arc-on', 0), ('route-togo-ft', 0.0), ('arc-lat-deg', 0.0), ('arc-lon-deg', 0.0), ('arc-r-ft', 1000.0), ('arc-dir', 1), ('arc-end-deg', 0.0)):
            c.declare('ap/' + n, x)
        for n in ('heading-bug-deg', 'target-altitude-ft', 'target-speed-kt'):
            c.declare(ST + n, 0.0)
        c.declare('/environment/magnetic-variation-deg', 0.0)
        for n in ('on', 'theta-on', 'phi-on', 'lever-on', 'vert-was', 'lat-was', 'spd-was', 'v-ref-kt', 'v-ref-dot-kts', 'theta-int-rad', 'theta-ref-rad',
                  'phi-ref-rad', 'hdot-hold-fps', 'tcl-int', 'tcl-w', 'fly', 'still-s', 'nac-w', 'vt-prev', 'vdot-fps2', 'dropped', 'nac-dropped'):
            c.declare('ap/' + n, 0.0)
        for n, x in (('hdg-tau-s', 5.0), ('nav-look-s', 20.0), ('kh', 0.1), ('hdot-max-fps', 1500 / 60.0), ('kv', 0.25), ('accel-kts', 2.0), ('decel-kts', 1.5),
                     ('kep', 0.012), ('kei', 0.004), ('klp', 0.8), ('kli', 0.25), ('theta-rate-dps', 3.0), ('phi-rate-dps', 5.0),
                     ('kgp', 0.25), ('app-kt', 120.0), ('hover-ft', 50.0), ('land-fps', 2.0), ('kep-back', 0.04), ('kei-back', 0.01)):
            c.declare('ap/' + n, x)
        c.channel('autopilot', execrate=AUTOPILOT_EXECRATE)
        dt = p('simulation/channel-dt')
        NOT = lambda a: D(v(1), a)
        lim = lambda x, lo, hi: MAX(v(lo), MIN(v(hi), x))
        on = lambda prop: GT(p(prop), v(0.5))
        air, hold = p('scas/airborne'), on('hh/on')
        # the modes (the flight director) and the coupling
        free = M(air, NOT(hold))
        # WYPT: the route manager's route (FlightGear's GPS in its leg mode), in place of the heading while it is active
        nav = c.fn('ap/nav', M(free, on(AF + 'wypt'), on('/autopilot/route-manager/active')))
        # the line (VOR/ILS, APPR): the distance to go to its point along its course, the cross-track error (+ left of it)
        R_E = 20925646.0
        la, lo = p('position/lat-geod-rad'), p('position/long-gc-rad')
        # (the line a great circle, as a localizer's beam: its course here turned from the point's by the meridians'
        # convergence, the longitude between times the sine of the latitude; 0.16 deg, 200 ft, at 12 NM at 37 deg)
        lla, llo = M(p('ap/line-lat-deg'), v(1 / R2D)), M(p('ap/line-lon-deg'), v(1 / R2D))
        dN = M(D(lla, la), v(R_E))
        dE = M(D(llo, lo), v(R_E), COS(M(S(la, lla), v(0.5))))
        lc = S(M(p('ap/line-course-deg'), v(1 / R2D)), M(D(lo, llo), SIN(la)))
        dgo_l = c.fn('ap/line-dist-ft', S(M(dE, SIN(lc)), M(dN, COS(lc))))
        xl = c.fn('ap/app-xte-ft', D(M(dE, COS(lc)), M(dN, SIN(lc))))
        # VOR/ILS and APPR: a course line through a point, from Nasal (autopilot.nas: the ILS's runway threshold, a VOR,
        # the route's landing zone), in place of the route and the heading; APPR also slows down along it to a hover.
        # It arms first (the route or the heading flown meanwhile) and captures within 20 s of flight of the line (its
        # intercept then 45 deg), or at once with neither of those on
        want = M(free, MAX(on(AF + 'vor-ils'), on(AF + 'appr')), on('ap/line-valid'))
        others = MAX(on(AF + 'hdg-hold'), M(on(AF + 'wypt'), on('/autopilot/route-manager/active')))
        near_ = LT(ABS(xl), M(MAX(p('velocities/vg-fps'), v(100)), p('ap/nav-look-s')))
        c.fn('ap/line-cap-next', M(want, MAX(on('ap/line-cap'), near_, NOT(others))), out='ap/line-cap', desc='1: the line captured, 0: armed or off')
        line = c.fn('ap/line', M(want, on('ap/line-cap')))
        # APPR with WYPT's route (autopilot.nas arms it with the route, to its last point): its speed and path along
        # the route's distance still to go until the line has it (ap/route-togo-ft, autopilot.nas)
        rte = c.fn('ap/app-route', M(nav, on(AF + 'appr'), on('ap/line-valid'), NOT(on('ap/line-cap'))))
        app = c.fn('ap/app', MAX(M(line, on(AF + 'appr')), rte))
        dgo = c.fn('ap/app-dist-ft', IFTHEN(GT(rte, v(0.5)), p('ap/route-togo-ft'), dgo_l), desc='the approach\'s distance to go, ft')
        lat = c.fn('ap/lat', MAX(M(free, on(AF + 'hdg-hold')), nav, line))
        # HVR ALT HOLD has the lever (the hover hold's height): the speed and the vertical modes yield to it
        lever_free = M(free, NOT(on(AF + 'hvr-hold')))
        spd = c.fn('ap/spd', M(lever_free, MAX(on(AF + 'spd-hold'), app)))
        # TF: terrain following (autopilot.nas, tf: the vertical speed that clears the terrain ahead at the set clearance)
        tf = c.fn('ap/tf-on', M(lever_free, on(AF + 'tf')))
        vert = c.fn('ap/vert', M(lever_free, MAX(on(AF + 'alt-hold'), spd, on('ap/gp'), tf)))
        stick = MAX(GT(ABS(p('fcs/elevator-cmd-norm')), v(0.1)), GT(ABS(p('fcs/aileron-cmd-norm')), v(0.1)))
        cpl = c.fn('ap/coupled', M(on(AF + 'cpld'), air, p('ctl/scas-on'), NOT(stick)))
        c.fn('ap/dropped-next', IFTHEN(GT(M(on('ap/on'), NOT(cpl), on(AF + 'cpld')), v(0.5)), v(1), p('ap/dropped')), out='ap/dropped',
             desc='1: let go by the stick or on the ground (Nasal clears it)')
        c.fn('ap/on-next', cpl, out=['ap/on', AF + 'cpld'])
        vrise = GT(M(vert, NOT(on('ap/vert-was'))), v(0.5))
        lrise = GT(M(lat, NOT(on('ap/lat-was'))), v(0.5))
        srise = GT(M(spd, NOT(on('ap/spd-was'))), v(0.5))
        # the line's path: its height (the hover's over the point, rising at its angle), its capture within 30 ft
        ang = M(p('ap/line-angle-deg'), v(1 / R2D))
        hp = c.fn('ap/app-path-ft', S(p('ap/line-elev-ft'), p('ap/hover-ft'), M(MAX(v(0), dgo), Q(SIN(ang), COS(ang)))))
        # (joined from below within 30 ft, or from above: past the descent's start it descends to the path)
        c.fn('ap/gp-next', M(MAX(line, rte), on('ap/line-gs'), MAX(on('ap/gp'), GT(p('position/h-sl-ft'), D(hp, v(30))))),
             out='ap/gp')
        # the airspeed's reference and its rate, kt and kt/s
        # (the forward airspeed, equivalent: signed, as the airspeed is not when the aircraft backs in a hover)
        knob = p(ST + 'target-speed-kt')
        vc, vref, gs = p('scas/ff-kts'), p('ap/v-ref-kt'), Q(p('velocities/vg-fps'), v(KT))
        hw = lim(D(vc, gs), -30, 50)
        prof = MAX(v(0), S(Q(SQ(M(v(2), p('ap/decel-kts'), v(KT), MAX(v(0), dgo))), v(KT)), hw))
        vsel = c.fn('ap/v-sel-kt', IFTHEN(GT(app, v(0.5)), MIN(IFTHEN(GT(knob, v(40)), knob, p('ap/app-kt')), prof), knob),
                    desc='the speed selected; on the approach no more than stops the aircraft over the point at the '
                         'deceleration (a ground speed: the headwind, airspeed less ground speed, added)')
        # (its rate from the aircraft's own when the mode engages, changing at no more than ap/vert-accel-g in the
        # energy's units, V dV'/g = 0.1 g: the pitch trading the speed's change for climb asks no more vertical
        # acceleration than ALT HOLD's ramp, 0.32 kt/s^2 at 112 kt. Stepped to 2 kt/s at once, an engagement slowing
        # at 0.7 kt/s 8 kt slow put the pitch from +7.6 to -3.8 deg, and stepped back to 0 on reaching the speed, up
        # again to +3.6; a second, smaller swing after it. Toward the selected speed the rate that stops on it at that
        # change, sqrt(2 j |e|), within the speed's step this frame)
        vt = p('velocities/vt-fps')
        V = MAX(vt, v(100))
        jerk = Q(M(p('ap/vert-accel-g'), v(G0 * G0)), M(V, v(KT)))
        ev = D(vsel, vref)
        rmax = MIN(SQ(M(v(2), jerk, ABS(ev))), Q(ABS(ev), MAX(dt, v(1e-3))))
        rtgt = IFTHEN(GT(ev, v(0)), MIN(p('ap/accel-kts'), rmax), NEG(MIN(p('ap/decel-kts'), rmax)))
        rprev, rj = p('ap/v-ref-dot-kts'), M(jerk, dt)
        vref_dot = c.fn('ap/v-ref-dot-next', IFTHEN(GT(M(spd, NOT(srise)), v(0.5)), S(rprev, MAX(NEG(rj), MIN(rj, D(rtgt, rprev)))),
                                                    M(p('ap/vdot-fps2'), Q(vc, MAX(vt, v(1))))), out='ap/v-ref-dot-kts')
        c.fn('ap/v-ref-next', IFTHEN(GT(M(spd, NOT(srise)), v(0.5)), S(vref, M(vref_dot, dt)), vc), out='ap/v-ref-kt')
        vref_dot = p('ap/v-ref-dot-kts')
        # the flight path's acceleration: the true airspeed's rate, smoothed (the rotors' vibration)
        c.fn('ap/vdot-next', S(p('ap/vdot-fps2'), M(D(Q(D(vt, p('ap/vt-prev')), MAX(dt, v(1e-3))), p('ap/vdot-fps2')), MIN(v(1), Q(dt, v(0.4))))),
             out='ap/vdot-fps2')
        c.fn('ap/vt-prev-next', vt, out='ap/vt-prev')
        # the vertical command: to the selected altitude, or the vertical speed held when the mode engaged; on the
        # path its descent at the ground speed, and its error
        hdot = p('velocities/h-dot-fps')
        c.fn('ap/hdot-hold-next', IFTHEN(vrise, hdot, p('ap/hdot-hold-fps')), out='ap/hdot-hold-fps')
        hmax = p('ap/hdot-max-fps')
        # (never below the hover's height on the radar altimeter: the point's elevation may be the scenery's guess)
        gpc = MAX(NEG(hmax), MIN(hmax, MAX(S(NEG(M(p('velocities/vg-fps'), Q(SIN(ang), COS(ang)))), M(p('ap/kgp'), D(hp, p('position/h-sl-ft')))),
                                           M(p('ap/kgp'), D(p('ap/hover-ft'), p('hh/h-wheels-ft'))))))
        # the energy's sharing blended by the nacelle angle: in airplane mode and early in the conversion (50 deg and
        # below) the lever its rate and the pitch its share between climb and speed [Lambregts]; on the back side, the
        # nacelles past 80 deg, the lever the climb and the pitch (with the nacelles) the speed, as a powered-lift
        # aircraft is flown there (slowing and descending at once, the energy's rule took the power to idle and sank it)
        wb = c.fn('ap/backside', MAX(v(0), MIN(v(1), Q(D(p('ctl/nacelle-deg'), v(50)), v(30)))))
        # TF's: changing at no more than the ride's vertical acceleration (as autopilot.nas plans it, at half that); below
        # half the set clearance, or without terrain data, a climb at the most it allows (below half the clearance at once,
        # at 0.5 g)
        tfr = GT(M(tf, NOT(on('ap/tf-was'))), v(0.5))
        low = LT(p('hh/h-wheels-ft'), M(v(0.5), MAX(v(100), p(ST + 'target-agl-ft'))))
        flyup = c.fn('ap/tf-flyup', M(tf, MAX(low, NOT(on('ap/tf-valid')))), desc='1: TF flying up (too low, or no terrain data)')
        # the climb it can make at full power: the power a climb takes, P = P_level + b h' + c h'^2 (measured, climb_power;
        # with the weight), the level flight's from the present power and energy's climb (h' + V V'/g), the climb whose
        # power is the full; where the pitch flies the path, no more than its 15 deg leave over the angle of attack;
        # smoothed over 2 s, within 3000 ft/min (autopilot.nas plans with it). (One efficiency for all the spare power
        # read 17 % high from a partial climb: the climb's power rises faster than the climb.) No more than the trims'
        # full power climb with the weight: as the power just came back, the climb still lagging it, the level flight's
        # read low and the climb 10 % high
        ck, cb, cc, cf, cw = climb_power()
        # (the tables', measured along AUTO NAC's schedule, looked up at the speed the schedule gives the present
        # nacelle angle, the nearest to the airspeed where it gives it to a range: the airspeed clamped there. In
        # the conversion the nacelles, not 5 kt, change the climb (34 ft/s at 160 kt, 35 deg, 46 at 170, 20 deg); by
        # the airspeed alone, the speed falling in a climb at 170 kt (the nacelles held for the reference's) took
        # the climb the power makes 12 % down as it went, a ridge 16 ft short)
        sks, sns = autonac_schedule()
        sg = np.arange(0.0, 300.5, 0.5)
        sched = np.interp(sg, sks, sns)
        ng = np.arange(0.0, 91.0, 1.0)
        slo = [float(sg[np.argmax(sched <= n + 1e-9)]) if (sched <= n + 1e-9).any() else float(sg[-1]) for n in ng]
        shi = [float(sg[np.nonzero(sched >= n - 1e-9)[0][-1]]) if (sched >= n - 1e-9).any() else 0.0 for n in ng]
        c.fn('ap/tf-config-kt', MAX(T1('ctl/nacelle-deg', list(ng), slo), MIN(T1('ctl/nacelle-deg', list(ng), shi), p('velocities/vc-kts'))),
             desc='the speed AUTO NAC\'s schedule gives the present nacelle angle, the nearest to the airspeed: the climb\'s tables\' (climb_power, tf_lag)')
        pmax = M(v(2), MIN(p('eng/power-available-hp'), p('eng/xmsn-limit-hp')))
        pnow = S(p('eng[0]/power-hp'), p('eng[1]/power-hp'))
        wr = Q(p('inertia/weight-lbs'), v(cw))
        bb, ccf = c.fn('ap/tf-b', M(wr, T1('ap/tf-config-kt', ck, cb))), c.fn('ap/tf-c', M(wr, T1('ap/tf-config-kt', ck, cc)))
        he = S(hdot, Q(M(vt, p('ap/vdot-fps2')), v(G0)))
        plev = D(pnow, S(M(bb, he), M(ccf, SQR(he))))
        dpf = MAX(v(0), D(pmax, plev))
        ps = Q(M(v(2), dpf), S(bb, SQ(S(SQR(bb), M(v(4), MAX(ccf, v(1e-6)), dpf)))))     # the root, stable as c goes to 0
        aoa = p('aero/alpha-rad')
        pcap = M(vt, SIN(MAX(v(0), D(v(15 / R2D), aoa))))
        ps = MIN(ps, Q(T1('ap/tf-config-kt', ck, cf), wr))
        # (the climb at a constant calibrated airspeed: the true one rises as the air thins, d ln V/dh = -1/2 d ln rho/dh
        # = 1/2 (g/(R T) - L/T), and takes V V'/g of the energy's rate; the climb is that over the acceleration factor,
        # 1 + V^2/g 1/2 (g/(R T) - L/T): 5 % at 200 kt, the trims' climbs were at a constant true airspeed)
        tr_ = p('atmosphere/T-R')
        fac = S(v(1), M(Q(SQR(vt), v(G0)), v(0.5), D(Q(v(G0 / 1716.49), tr_), Q(v(0.0035662), tr_))))
        ps = Q(ps, fac)
        cap = MAX(v(0), MIN(p('ap/tf-climb-max-fps'), IFTHEN(LT(wb, v(0.5)), MIN(ps, pcap), ps)))
        ccap = p('ap/tf-climb-cap-fps')
        c.fn('ap/tf-climb-cap-next', S(ccap, M(D(cap, ccap), MIN(v(1), Q(dt, v(2))))), out='ap/tf-climb-cap-fps')
        # TF's path: a reference (its vertical speed the plan's, ramped at the ride's acceleration, and its height),
        # which autopilot.nas plans for in place of the aircraft. (Planned from the aircraft, terrain nearer than its
        # climb's delay could be met only with all or nothing: a relay through a delay, which porpoised over level
        # ground, +-8 ft/s at 200 kt.) The aircraft follows the reference's model of it: its climb's response, two
        # first order lags in parallel fitted to its step (tf_lag: the pitch's quick share and the lever's slow
        # tail, a tenth of it at 200 kt over 10-15 s); autopilot.nas plans the model's height. The height's error
        # closed with a gain for 60 deg of phase margin on the measured response (tf_lag): a linear loop. Where the
        # lever is at its stop and the aircraft falls behind the model, the model and the reference come down to it:
        # the plan sees the height there is (no wind-up)
        lk, lw, lt1, lt2, lkh = tf_lag()
        w1 = c.fn('ap/tf-w', T1('ap/tf-config-kt', lk, lw), desc='the quick lag\'s share of the climb\'s response (tf_lag)')
        tau1 = c.fn('ap/tf-tau1-s', MAX(v(0.05), T1('ap/tf-config-kt', lk, lt1)), desc='the quick lag, s (tf_lag)')
        tau2 = c.fn('ap/tf-tau2-s', MAX(v(0.05), T1('ap/tf-config-kt', lk, lt2)), desc='the slow lag, s (tf_lag)')
        ttgt = IFTHEN(GT(flyup, v(0.5)), ccap, p('ap/tf-hdot-fps'))
        trate = M(IFTHEN(GT(M(tf, low), v(0.5)), MAX(p('ap/tf-flyup-g'), p('ap/tf-ride-g')), p('ap/tf-ride-g')), v(G0), dt)
        tprev = p('ap/tf-cmd-fps')
        run = GT(M(tf, NOT(tfr)), v(0.5))
        vr = c.fn('ap/tf-cmd-next', IFTHEN(run, S(tprev, MAX(NEG(trate), MIN(trate, D(ttgt, tprev)))), hdot), out='ap/tf-cmd-fps')
        h = p('position/h-sl-ft')
        hm, v1, v2 = p('ap/tf-model-ft'), p('ap/tf-v1-fps'), p('ap/tf-v2-fps')
        hmp = S(hm, M(S(M(w1, v1), M(D(v(1), w1), v2)), dt))
        aw = c.fn('ap/tf-aw-ft', M(GT(p('/controls/engines/engine[0]/throttle'), v(0.999)), MAX(v(0), D(hmp, h))))
        c.fn('ap/tf-ref-next', IFTHEN(run, D(S(p('ap/tf-ref-ft'), M(vr, dt)), aw), h), out='ap/tf-ref-ft')
        c.fn('ap/tf-model-next', IFTHEN(run, D(hmp, aw), h), out='ap/tf-model-ft')
        for n, x, tau in (('v1', v1, tau1), ('v2', v2, tau2)):
            xn = S(x, M(D(vr, x), MIN(v(1), Q(dt, tau))))
            c.fn('ap/tf-%s-next' % n, IFTHEN(run, IFTHEN(GT(aw, v(0)), MIN(xn, hdot), xn), hdot), out='ap/tf-%s-fps' % n)
        c.fn('ap/tf-was-next', tf, out='ap/tf-was')
        kh = c.fn('ap/tf-kh', T1('ap/tf-config-kt', lk, lkh), desc='the height\'s tracking gain, 1/s (tf_lag); autopilot.nas closes a height it cannot clear in time over its inverse')
        tfc = S(p('ap/tf-cmd-fps'), M(p('ap/tf-track'), kh, D(p('ap/tf-model-ft'), h)))
        # ALT HOLD's and the vertical speed hold's command changing at no more than ap/vert-accel-g (TF's soft ride),
        # from the vertical speed there is when they take over (engaging, or after the glide path or TF): stepped, an
        # engagement climbing at 26 ft/s 200 ft over the selected altitude asked a 25 ft/s descent at once, the lever
        # went from 0.65 to its stop and the pitch from +8 to -12 deg, -0.37 g. (Capturing, the law's own deceleration
        # is kh h' = 0.1 x 25 = 2.5 ft/s^2, inside it). Not in a go-around, whose climb is wanted at once: eased, it lost
        # 17-23 ft more from 200-370 ft
        vs_tgt = IFTHEN(on(AF + 'alt-hold'), MAX(NEG(hmax), MIN(hmax, M(p('ap/kh'), D(p(ST + 'target-altitude-ft'), p('position/h-sl-ft'))))),
                        p('ap/hdot-hold-fps'))
        vs_prev, vs_rate = p('ap/vs-cmd-fps'), M(p('ap/vert-accel-g'), v(G0), dt)
        own = GT(M(vert, NOT(vrise), NOT(on('ap/gp')), NOT(GT(tf, v(0.5)))), v(0.5))
        vs = c.fn('ap/vs-cmd-next', IFTHEN(own, IFTHEN(on(AF + 'ga'), vs_tgt, S(vs_prev, MAX(NEG(vs_rate), MIN(vs_rate, D(vs_tgt, vs_prev))))),
                                           hdot), out='ap/vs-cmd-fps')
        hdc = c.fn('ap/hdot-cmd-fps', IFTHEN(on('ap/gp'), gpc, IFTHEN(GT(tf, v(0.5)), tfc, vs)))
        vdc = S(M(p('ap/kv'), D(vref, vc), v(KT)), M(vref_dot, v(KT)))
        # (in TF the speed is free within its band: no acceleration asked of it, its own still damping the exchange
        # between climb and speed [Lambregts]; asked again over the band's last 5 kt. On the back side too, the pitch
        # flying the speed there: freed only where the pitch flew the path, at 70 deg a third of it, a climb at full
        # power gave 0.5 ft/s of it to the speed coming back, 12 ft short over a ridge at 120 kt)
        # (The speed's weight in the pitch's share stays 1 there: with 0, the pitch flying the path alone, a shortfall
        # at full power went all into the speed, but the share's damping went with the speed's rate in it: the climb's
        # step overshot 17-43 % at 150-250 kt, 2-19 % with it. A shortfall the anti-windup above takes instead)
        band = p('ap/tf-band-kt')
        edge = MAX(v(0), MIN(v(1), Q(D(ABS(D(vc, vref)), D(band, v(5))), v(5))))
        wspd = D(v(1), M(tf, D(v(1), edge)))
        dv = c.fn('ap/dv-fps', M(spd, Q(M(V, D(M(wspd, vdc), p('ap/vdot-fps2'))), v(G0))), desc='the speed\'s share of the energy rate error')
        dh = D(hdc, hdot)
        e_err = c.fn('ap/e-err-fps', S(dh, M(D(v(1), wb), dv)), desc='the lever\'s error: the energy rate\'s, the climb\'s on the back side, ft/s')
        l_err = c.fn('ap/l-err-fps', D(M(D(v(1), wb), dh), dv),
                     desc='the pitch\'s: the share between climb and speed, the speed\'s on the back side, ft/s')
        # the pitch: integral and proportional on the share, limited and rate limited
        # (the commanded flight path's angle fed forward, where the pitch flies the path: the integrator need not wind up
        # to it, which lagged a changing climb command by 1/kli, 4 s; its state the rest, about the angle of attack)
        gff = M(D(v(1), wb), Q(hdc, V))
        th0 = D(p('attitude/theta-rad'), M(D(v(1), wb), Q(hdot, V)))
        c.fn('ap/theta-int-next', IFTHEN(vrise, th0, IFTHEN(GT(vert, v(0.5)),
             lim(S(p('ap/theta-int-rad'), M(p('ap/kli'), Q(l_err, V), dt)), -15 / R2D, 15 / R2D), th0)), out='ap/theta-int-rad')
        tc = lim(S(p('ap/theta-int-rad'), gff, M(p('ap/klp'), Q(l_err, V))), -10 / R2D, 15 / R2D)
        tr, tmax = p('ap/theta-ref-rad'), M(p('ap/theta-rate-dps'), v(1 / R2D), dt)
        # (its rate, rad/s, for the augmentation's damper: the commanded rate isn't damped)
        c.fn('ap/theta-ref-rate', IFTHEN(GT(M(vert, NOT(vrise)), v(0.5)), Q(MAX(NEG(tmax), MIN(tmax, D(tc, tr))), MAX(dt, v(1e-3))), v(0)))
        c.fn('ap/theta-ref-next', IFTHEN(GT(vert, v(0.5)), IFTHEN(vrise, p('attitude/theta-rad'), S(tr, MAX(NEG(tmax), MIN(tmax, D(tc, tr))))),
             p('attitude/theta-rad')), out='ap/theta-ref-rad', desc='the flight director\'s pitch')
        # the heading: bank to the bug (magnetic) for a turn that closes on it with a time constant (the bank for a
        # heading error growing with the speed), within a limit growing with the speed
        bug = M(S(p(ST + 'heading-bug-deg'), p('/environment/magnetic-variation-deg')), v(1 / R2D))
        dpsi = D(bug, p('attitude/psi-rad'))
        err = c.fn('ap/hdg-err-deg', M(v(R2D), ATAN2(SIN(dpsi), COS(dpsi))))
        bmax = T1('velocities/vc-kts', [40, 80, 150], [15, 20, 25])
        # WYPT: the leg's course with an intercept, the cross-track error over 20 s of flight (it closes on the leg
        # without overshooting it), flown as a ground track; the route's first leg, from wherever it was activated,
        # straight to its waypoint (ap/direct). Between legs an arc (autopilot.nas sets it where an arc of the aircraft's
        # turn radius at 80 % of the bank limit joins them, and moves the route on): its bank fed forward, its radial
        # error closed as the leg's is; rolled out of to wings level, then the next leg's law
        trk = ATAN2(p('velocities/v-east-fps'), p('velocities/v-north-fps'))
        look = M(MAX(vt, v(100)), p('ap/nav-look-s'))
        tc = IFTHEN(on('ap/direct'), M(p(GPS + 'wp[1]/bearing-true-deg'), v(1 / R2D)),
                    S(M(p(GPS + 'leg-true-course-deg'), v(1 / R2D)), ATAN2(M(p(GPS + 'wp[1]/course-error-nm'), v(6076.1)), look)))
        dtrk = D(tc, trk)
        terr = c.fn('ap/trk-err-deg', M(v(R2D), ATAN2(SIN(dtrk), COS(dtrk))))
        vg = MAX(p('velocities/vg-fps'), v(60))
        aN = M(D(M(p('ap/arc-lat-deg'), v(1 / R2D)), la), v(R_E))
        aE = M(D(M(p('ap/arc-lon-deg'), v(1 / R2D)), lo), v(R_E), COS(la))
        adir, ar = p('ap/arc-dir'), MAX(p('ap/arc-r-ft'), v(100))
        atd = D(ATAN2(aE, aN), M(adir, v(math.pi / 2)))               # the arc's track here: the centre abeam
        da = D(S(atd, M(adir, ATAN2(D(SQ(S(SQR(aN), SQR(aE), v(1))), ar), look))), trk)
        aerr = c.fn('ap/arc-err-deg', M(v(R2D), ATAN2(SIN(da), COS(da))))
        aff = c.fn('ap/arc-bank-deg', M(adir, v(R2D), ATAN2(SQR(vg), M(v(G0), ar))), desc='the arc\'s bank, fed forward')
        # The arc ends where the track still has the turn to go that the roll out makes: from the bank reference at
        # its rate limit to wings level, half the ramp's time, and the augmentation's own delay (measured: roll_delay;
        # autopilot.nas leads the arc's start by the same); the roll out flown (2), the leg's law has it
        rk, rt = roll_delay()
        lag = c.fn('ap/roll-lag-s', T1('velocities/vc-kts', rk, rt), desc='the augmentation\'s roll delay (roll_delay), s')
        kpsi = Q(MAX(vt, v(60)), M(v(G0), p('ap/hdg-tau-s')))
        dend = D(M(p('ap/arc-end-deg'), v(1 / R2D)), trk)
        pref = ABS(p('ap/phi-ref-rad'))
        w_out = M(Q(M(v(G0), Q(SIN(pref), COS(pref))), vg), S(Q(M(pref, v(R2D)), M(v(2), p('ap/phi-rate-dps'))), lag))
        ast = p('ap/arc-on')
        ending = M(GT(ast, v(0.5)), LT(ast, v(1.5)), LT(M(adir, ATAN2(SIN(dend), COS(dend))), w_out))
        level = M(GT(ast, v(1.5)), LT(pref, v(0.5 / R2D)))
        arc_st = c.fn('ap/arc-on-next', M(nav, IFTHEN(GT(ending, v(0.5)), v(2), IFTHEN(GT(level, v(0.5)), v(0), ast))), out='ap/arc-on',
                      desc='1: turning on the arc between legs (autopilot.nas starts it), 2: rolling out of it')
        arc = GT(M(GT(arc_st, v(0.5)), LT(arc_st, v(1.5))), v(0.5))
        rolling = GT(arc_st, v(1.5))
        tl = S(lc, ATAN2(xl, look))
        dl_ = D(tl, trk)
        lerr_ = c.fn('ap/line-err-deg', M(v(R2D), ATAN2(SIN(dl_), COS(dl_))))
        bank = IFTHEN(GT(line, v(0.5)), M(kpsi, lerr_), IFTHEN(GT(nav, v(0.5)), IFTHEN(arc, S(aff, M(kpsi, aerr)), IFTHEN(rolling, v(0), M(kpsi, terr))),
                      M(kpsi, err)))
        pc = M(v(1 / R2D), MAX(NEG(bmax), MIN(bmax, bank)))
        pr, pmax = p('ap/phi-ref-rad'), M(p('ap/phi-rate-dps'), v(1 / R2D), dt)
        c.fn('ap/phi-ref-rate', IFTHEN(GT(M(lat, NOT(lrise)), v(0.5)), Q(MAX(NEG(pmax), MIN(pmax, D(pc, pr))), MAX(dt, v(1e-3))), v(0)))
        c.fn('ap/phi-ref-next', IFTHEN(GT(lat, v(0.5)), IFTHEN(lrise, p('attitude/phi-rad'), S(pr, MAX(NEG(pmax), MIN(pmax, D(pc, pr))))),
             p('attitude/phi-rad')), out='ap/phi-ref-rad', desc='the flight director\'s bank')
        c.fn('ap/theta-on-next', M(vert, cpl), out='ap/theta-on')
        c.fn('ap/phi-on-next', M(lat, cpl), out='ap/phi-on')
        for n, x in (('vert-was', vert), ('lat-was', lat), ('spd-was', spd)):
            c.fn('ap/%s-next' % n, x, out='ap/' + n)
        # the lever: the energy rate, integral and proportional, driven while coupled with a speed mode; the pilot's
        # own movement flies through and 1.5 s after it stops the autopilot has it again (as the hover hold's)
        thr = '/controls/engines/engine[0]/throttle'
        drive = c.fn('ap/lever-on-next', M(spd, cpl), out='ap/lever-on')
        dl = ABS(D(p(thr), p('ap/tcl-w')))
        moved, moving = GT(dl, v(0.004)), GT(dl, v(1e-5))
        c.fn('ap/still-next', IFTHEN(moving, v(0), S(p('ap/still-s'), dt)), out='ap/still-s')
        back = GT(M(on('ap/fly'), NOT(moving), GT(p('ap/still-s'), v(1.5))), v(0.5))
        c.fn('ap/fly-next', M(drive, IFTHEN(moved, v(1), IFTHEN(back, v(0), p('ap/fly')))), out='ap/fly')
        flying = GT(M(drive, NOT(p('ap/fly'))), v(0.5))
        c.fn('ap/tcl-int-next', IFTHEN(GT(M(spd, NOT(srise), NOT(back), NOT(p('ap/fly'))), v(0.5)),
             lim(S(p('ap/tcl-int'), M(S(M(D(v(1), wb), p('ap/kei')), M(wb, p('ap/kei-back'))), e_err, dt)), 0, 1), p(thr)), out='ap/tcl-int')
        kep = S(M(D(v(1), wb), p('ap/kep')), M(wb, p('ap/kep-back')))
        tcmd = c.fn('ap/tcl-cmd', lim(S(p('ap/tcl-int'), M(kep, e_err)), 0, 1), desc='the flight director\'s lever')
        c.fn('ap/lever', IFTHEN(GT(p('ap/app-state'), v(4.5)), MAX(v(0), D(p(thr), M(v(0.3), dt))), IFTHEN(flying, tcmd, p(thr))), out=[thr, 'ap/tcl-w'],
             desc='the lever, driven while coupled with a speed mode; down after an approach\'s landing')
        # AUTO NAC: the nacelles for the speed reference, within the corridor; the pilot moving them takes it off
        ncmd = '/controls/flight/nacelle-cmd-deg'
        an = c.fn('ap/auto-nac-on', M(spd, MAX(on(AF + 'auto-nac'), app)))
        sk, sn = autonac_schedule()
        sched = c.fn('ap/nacelle-sched-deg', T1('ap/v-ref-kt', sk, sn))
        own = GT(ABS(D(p(ncmd), p('ap/nac-w'))), v(0.5))
        c.fn('ap/nac-dropped-next', IFTHEN(GT(M(an, own), v(0.5)), v(1), p('ap/nac-dropped')), out='ap/nac-dropped')
        c.fn('ap/auto-nac-next', IFTHEN(GT(M(an, own), v(0.5)), v(0), p(AF + 'auto-nac')), out=AF + 'auto-nac')
        c.fn('ap/nacelle', IFTHEN(GT(M(an, NOT(own)), v(0.5)), MAX(p('ctl/cps-min-deg'), MIN(p('ctl/cps-max-deg'), sched)), p(ncmd)),
             out=[ncmd, 'ap/nac-w'], desc='the nacelle command, driven by AUTO NAC')
        # the approach: 1 flying the line; 2 handing over to the hover hold below 35 kt over the ground within 2000 ft
        # of the point (its point, 50 ft on the radar height); 3 hovering there (HVR CPLD); 4 landing (the height's
        # target down at 2 ft/s), after APPR pressed again or on the way (ap/app-land); 5 on the ground (the modes off,
        # the lever down, then APPR off). The hold refusing or let go by the pilot ends it (APPR off). APPR off: 0
        st, ak = p('ap/app-state'), on(AF + 'appr')
        wow = GT(S(*[p('gear/unit[%d]/WOW' % i) for i in range(3)]), v(0.5))
        is_ = lambda n: GT(M(GT(st, v(n - 0.5)), LT(st, v(n + 0.5))), v(0.5))
        to2 = GT(M(app, LT(gs, v(35)), LT(dgo, v(2000)), GT(p('ctl/nacelle-deg'), v(72))), v(0.5))
        hov = MAX(is_(3), is_(4))
        nst = c.fn('ap/app-state-new', IFTHEN(NOT(ak), v(0), IFTHEN(GT(M(is_(0), app), v(0.5)), v(1), IFTHEN(GT(M(is_(1), to2), v(0.5)), v(2),
                   IFTHEN(GT(M(is_(2), hold), v(0.5)), v(3), IFTHEN(GT(M(is_(2), GT(p('hh/dropped'), v(1.5))), v(0.5)), v(0),
                   IFTHEN(GT(M(hov, wow), v(0.5)), v(5), IFTHEN(GT(M(hov, NOT(hold)), v(0.5)), v(0),
                   IFTHEN(GT(M(is_(3), on('ap/app-land'), GT(p('hh/state'), v(1.5))), v(0.5)), v(4),
                   IFTHEN(GT(M(is_(5), LT(p('/controls/engines/engine[0]/throttle'), v(0.02))), v(0.5)), v(0), st))))))))))
        hand = GT(M(GT(nst, v(1.5)), LT(nst, v(2.5)), is_(1)), v(0.5))
        c.fn('ap/hand-target', IFTHEN(hand, v(1), p('hh/ext')), out='hh/ext')
        c.fn('ap/hand-lat', IFTHEN(hand, M(p('ap/line-lat-deg'), v(1 / R2D)), p('hh/ext-lat-rad')), out='hh/ext-lat-rad')
        c.fn('ap/hand-lon', IFTHEN(hand, M(p('ap/line-lon-deg'), v(1 / R2D)), p('hh/ext-lon-rad')), out='hh/ext-lon-rad')
        c.fn('ap/hand-engage', IFTHEN(hand, v(1), p('hh/engage-cmd')), out='hh/engage-cmd')
        agl = ST + 'target-agl-ft'
        down = GT(M(GT(nst, v(4.5)), LT(st, v(4.5))), v(0.5))
        ended = GT(M(LT(nst, v(0.5)), GT(st, v(0.5))), v(0.5))
        c.fn('ap/hand-agl', IFTHEN(hand, p('ap/hover-ft'), IFTHEN(GT(M(ended, is_(4)), v(0.5)), MAX(v(0), p('hh/h-wheels-ft')),
             IFTHEN(is_(4), MAX(v(-10), D(p(agl), M(p('ap/land-fps'), dt))), p(agl)))), out=agl,
             desc='the hover height handed over; landing, down at 2 ft/s to below the ground (it sinks onto the gear); '
                  'APPR off while landing, the height there')
        c.fn('ap/landed', IFTHEN(down, v(0), p('hh/dropped')), out='hh/dropped')
        for prop in [AF + n for n in ('hvr-hold', 'cpld', 'hdg-hold', 'alt-hold', 'spd-hold', 'auto-nac', 'wypt', 'vor-ils')] + ['hh/engage-cmd']:
            c.fn('ap/landed-%s' % prop.split('/')[-1], IFTHEN(down, v(0), p(prop)), out=prop)
        c.fn('ap/app-hand-hvr', IFTHEN(hand, v(1), IFTHEN(GT(M(ended, GT(st, v(1.5)), NOT(hold)), v(0.5)), v(0), p(AF + 'hvr-hold'))), out=AF + 'hvr-hold',
             desc='HVR ALT HOLD with the handover; off when the approach ends, unless the hover hold still holds')
        for prop in (AF + 'appr', 'ap/app-land'):
            c.fn('ap/ended-%s' % prop.split('/')[-1], IFTHEN(ended, v(0), p(prop)), out=prop)
        c.fn('ap/app-state-next', nst, out='ap/app-state')

    def hover_hold(s):
        """Hover hold (HVR CPLD on the flight control panel, or y), after the AH-64D's modern control laws, Bell's
        automatic hover hold and ADS-33E-PRF's position hold and translational rate command [README]. Engaged at up to
        40 kt over the ground with the nacelles past 70 deg, it sets the stop point, where the present velocity comes
        to rest at 0.08 g, and flies one profile to it (towards the point at no more than sqrt(2 a d), linear within
        its last few tens of feet), then holds it: position, the heading (the augmentation's heading hold) and the
        height (radar below 500 ft, barometric above; HVR ALT HOLD takes the panel's radar height). Fore and aft go
        through the nacelles with the fuselage held level (the V-22 hovers and translates on its nacelles), sideways
        through the bank. The stick out of its detent commands a ground speed (10 kt at half travel, 2 kt with REM
        HVR, the crew's fine positioning) with a first-order response, and on its release the stop point is set
        again; past half travel, 45 kt, below 65 deg of nacelle or on the ground the hold lets go. The hold drives
        the nacelle command and the lever, as a coupled mode drives its controls: the pilot moving the lever flies
        through it, and the lever left alone for 1.5 s holds the new height. Gains are properties (hh/*)."""
        c = s.ctl
        KT, R_E = 1.6878, 20925646.0
        STICK, GS_MAX, GS_DROP, NAC_MIN, NAC_DROP = 0.5, 40.0, 45.0, 70.0, 65.0
        A_MAX, ROLL_MAX, ROLL_RATE, PITCH_RATE, VZ_MAX = 0.2 * G0, 12.0, 15.0, 3.0, 500 / 60.0
        c.declare('hh/engage-cmd', 0)
        c.declare('/instrumentation/mv22b/afcs/hvr-hold', 0)
        c.declare('/instrumentation/mv22b/afcs/rem-hvr', 0)
        c.declare('/autopilot/settings/target-agl-ft', 0.0)
        for n in ('on', 'lat-t-rad', 'lon-t-rad', 'iN', 'iE', 'vf-cmd', 'vr-cmd', 'theta-ref-rad', 'phi-ref-rad', 'h-ref-ft',
                  'alt-on', 'alt-radar', 'alt-fly', 'still-s', 'tcl-int', 'tcl-w', 'dropped', 'nudge-fwd-ft', 'nudge-right-ft',
                  'ext', 'ext-lat-rad', 'ext-lon-rad'):
            c.declare('hh/' + n, 0.0)
        for n, x in (('decel-g', 0.08), ('kp', 0.4), ('kv-fwd', 0.8), ('kv-side', 0.5), ('ki', 0.06), ('i-max-g', 0.1), ('trc-kt', 10.0),
                     ('trc-rem-kt', 2.0), ('trc-tau', 1.2), ('pitch-deg', 0.0), ('kh', 0.3), ('kpz', 0.08), ('kiz', 0.015)):
            c.declare('hh/' + n, x)
        c.channel('hover hold', execrate=CONTROL_EXECRATE)
        dt = p('simulation/channel-dt')
        NOT = lambda a: D(v(1), a)
        lim = lambda x, lo, hi: MAX(v(lo), MIN(v(hi), x))
        air, nac, gs = p('scas/airborne'), p('ctl/nacelle-deg'), Q(p('velocities/vg-fps'), v(KT))
        lon, lat, det = p('fcs/elevator-cmd-norm'), p('fcs/aileron-cmd-norm'), p('scas/detent')
        # engaged, refused, let go
        out = c.fn('hh/stick-out', MAX(GT(ABS(lon), det), GT(ABS(lat), det)), desc='1: the stick out of its detent')
        big = MAX(GT(ABS(lon), v(STICK)), GT(ABS(lat), v(STICK)))
        can = M(air, p('ctl/scas-on'), LT(gs, v(GS_MAX)), GT(nac, v(NAC_MIN)))
        stay = M(air, p('ctl/scas-on'), LT(gs, v(GS_DROP)), GT(nac, v(NAC_DROP)), NOT(big))
        cmd, was = GT(p('hh/engage-cmd'), v(0.5)), GT(p('hh/on'), v(0.5))
        on = c.fn('hh/on-new', M(cmd, IFTHEN(was, stay, can)))
        rise = c.fn('hh/rise', M(on, NOT(was)))
        c.fn('hh/dropped-next', IFTHEN(GT(M(was, NOT(on), cmd), v(0.5)), v(1), IFTHEN(GT(M(cmd, NOT(was), NOT(on)), v(0.5)), v(2),
             IFTHEN(GT(rise, v(0.5)), v(0), p('hh/dropped')))), out='hh/dropped', desc='1: let go, 2: refused (Nasal clears it)')
        c.fn('hh/on-next', on, out=['hh/on', 'hh/engage-cmd'])
        onb = GT(on, v(0.5))
        # the stop point, set on engaging and while the stick flies (the crew's nudges move it, ft forward and right); or
        # a point given on engaging (hh/ext: the autopilot's approach hands over its hover point)
        la, lo = p('position/lat-geod-rad'), p('position/long-gc-rad')
        cl = c.fn('hh/coslat', COS(la))
        vN, vE, vg = p('velocities/v-north-fps'), p('velocities/v-east-fps'), p('velocities/vg-fps')
        cp, sp = c.fn('hh/cpsi', COS(p('attitude/psi-rad'))), c.fn('hh/spsi', SIN(p('attitude/psi-rad')))
        a = c.fn('hh/decel-fps2', M(p('hh/decel-g'), v(G0)))
        k = Q(vg, M(v(2), a))
        again = GT(MAX(rise, M(on, out)), v(0.5))
        nf, nr = p('hh/nudge-fwd-ft'), p('hh/nudge-right-ft')
        ext = GT(c.fn('hh/ext-rise', M(rise, p('hh/ext'))), v(0.5))
        c.fn('hh/lat-t-next', IFTHEN(ext, p('hh/ext-lat-rad'), IFTHEN(again, S(la, Q(M(vN, k), v(R_E))), S(p('hh/lat-t-rad'), Q(D(M(nf, cp), M(nr, sp)), v(R_E))))),
             out='hh/lat-t-rad')
        c.fn('hh/lon-t-next', IFTHEN(ext, p('hh/ext-lon-rad'), IFTHEN(again, S(lo, Q(M(vE, k), M(v(R_E), cl))),
                                     S(p('hh/lon-t-rad'), Q(S(M(nf, sp), M(nr, cp)), M(v(R_E), cl))))), out='hh/lon-t-rad')
        c.fn('hh/ext-next', IFTHEN(GT(rise, v(0.5)), v(0), p('hh/ext')), out='hh/ext')
        c.fn('hh/nudge-clear', v(0), out=['hh/nudge-fwd-ft', 'hh/nudge-right-ft'])
        # the profile to it: velocity commanded towards the point
        eN = c.fn('hh/eN-ft', M(D(p('hh/lat-t-rad'), la), v(R_E)))
        eE = c.fn('hh/eE-ft', M(D(p('hh/lon-t-rad'), lo), v(R_E), cl))
        e = c.fn('hh/e-ft', SQ(S(SQR(eN), SQR(eE), v(0.01))), desc='distance to the hold point')
        kp = p('hh/kp')
        near = c.fn('hh/near', LT(M(kp, e), SQ(M(v(2), a, e))), desc='1: within the linear capture, 0: on the deceleration')
        vp = c.fn('hh/vp-fps', MIN(M(kp, e), SQ(M(v(2), a, e)), v(GS_DROP * KT)))
        # the stick's ground speed, heading axes (forward, right), first order; it starts from the present velocity
        vf = c.fn('hh/vf-fps', S(M(vN, cp), M(vE, sp)))
        vr = c.fn('hh/vr-fps', S(NEG(M(vN, sp)), M(vE, cp)))
        rem = GT(p('/instrumentation/mv22b/afcs/rem-hvr'), v(0.5))
        trc_k = Q(M(IFTHEN(rem, p('hh/trc-rem-kt'), p('hh/trc-kt')), v(KT)), v(STICK))
        f1 = Q(dt, p('hh/trc-tau'))
        for ax, stick, now in (('vf', lon, vf), ('vr', lat, vr)):
            c.fn('hh/%s-cmd-next' % ax, IFTHEN(GT(M(on, out), v(0.5)),
                 S(p('hh/%s-cmd' % ax), M(D(M(lim(stick, -STICK, STICK), trc_k), p('hh/%s-cmd' % ax)), f1)), now), out='hh/%s-cmd' % ax)
        trc = GT(M(on, out), v(0.5))
        vfc, vrc = p('hh/vf-cmd'), p('hh/vr-cmd')
        vcN = c.fn('hh/vN-cmd', IFTHEN(trc, D(M(vfc, cp), M(vrc, sp)), Q(M(vp, eN), e)))
        vcE = c.fn('hh/vE-cmd', IFTHEN(trc, S(M(vfc, sp), M(vrc, cp)), Q(M(vp, eE), e)))
        # the velocity loop: its error and the profile's deceleration fed forward, the integrators in earth axes (they
        # take up the wind, which stays put through a turn), the gains in heading axes: fore and aft through the
        # nacelles is quick, sideways through the bank waits on the roll attitude's loop (about 1.2 rad/s)
        er, ff, ii = {}, {}, {}
        for ax, vc, vv, ee in (('N', vcN, vN, eN), ('E', vcE, vE, eE)):
            er[ax] = c.fn('hh/d%s-fps' % ax, D(vc, vv))
            imax = M(p('hh/i-max-g'), v(G0))
            c.fn('hh/i%s-next' % ax, IFTHEN(GT(M(on, NOT(rise)), v(0.5)), MAX(NEG(imax), MIN(imax, S(p('hh/i' + ax), M(p('hh/ki'), er[ax], dt)))), v(0)),
                 out='hh/i' + ax)
            ff[ax] = c.fn('hh/ff%s-fps2' % ax, IFTHEN(trc, v(0), IFTHEN(GT(near, v(0.5)), NEG(M(kp, vv)), NEG(Q(M(a, ee), e)))))
            ii[ax] = p('hh/i' + ax)
        fwd = lambda x: S(M(x['N'], cp), M(x['E'], sp))
        side = lambda x: S(NEG(M(x['N'], sp)), M(x['E'], cp))
        af = c.fn('hh/af-fps2', lim(S(M(p('hh/kv-fwd'), fwd(er)), fwd(ff), fwd(ii)), -A_MAX, A_MAX), desc='acceleration commanded, forward')
        ar = c.fn('hh/ar-fps2', lim(S(M(p('hh/kv-side'), side(er)), side(ff), side(ii)), -A_MAX, A_MAX), desc='acceleration commanded, right')
        # to the controls: bank (rate limited, from the present one), the fuselage eased level, the nacelles tilted
        rate = lambda r: M(v(r / R2D), dt)
        phi = lim(ATAN2(ar, v(G0)), -ROLL_MAX / R2D, ROLL_MAX / R2D)
        pr = p('hh/phi-ref-rad')
        c.fn('hh/phi-ref-next', IFTHEN(GT(rise, v(0.5)), p('attitude/phi-rad'), S(pr, MAX(NEG(rate(ROLL_RATE)), MIN(rate(ROLL_RATE), D(phi, pr))))),
             out='hh/phi-ref-rad')
        # the nacelles take the fore and aft acceleration; what is past their range (75 deg forward, NACELLE_MAX aft, as
        # a stop in a tailwind needs) the pitch attitude takes, nose up for aft
        nraw = c.fn('hh/nacelle-want-deg', D(v(90), M(v(R2D), ATAN2(af, v(G0)))))
        over = lim(D(nraw, lim(nraw, 75.0, U.NACELLE_MAX)), -10.0, 10.0)
        tr = p('hh/theta-ref-rad')
        c.fn('hh/theta-ref-next', IFTHEN(GT(rise, v(0.5)), p('attitude/theta-rad'),
             S(tr, MAX(NEG(rate(PITCH_RATE)), MIN(rate(PITCH_RATE), D(Q(S(p('hh/pitch-deg'), over), v(R2D)), tr))))), out='hh/theta-ref-rad')
        ncmd = '/controls/flight/nacelle-cmd-deg'
        c.fn('hh/nacelle', IFTHEN(onb, lim(nraw, 75.0, U.NACELLE_MAX), p(ncmd)), out=ncmd, desc='the nacelle command, driven while the hold is on')
        # the height: the lever driven (power: the governor keeps the rotor speed with the collective); the pilot's
        # own movement of it flies through, and 1.5 s after it stops the height there is held
        hvr = GT(p('/instrumentation/mv22b/afcs/hvr-hold'), v(0.5))
        alt = c.fn('hh/alt-on-new', M(air, GT(nac, v(60)), MAX(on, hvr)))
        alt_rise = c.fn('hh/alt-rise', M(alt, NOT(GT(p('hh/alt-on'), v(0.5)))))
        c.fn('hh/alt-on-next', alt, out='hh/alt-on')
        hagl = c.fn('hh/h-wheels-ft', D(p('position/h-agl-ft'), Q(D(p('inertia/cg-z-in'), v(U.MAIN_GEAR[2])), v(12))),
                    desc='the wheels\' height, the radar height the displays and the HVR ALT knob show')
        hsl = p('position/h-sl-ft')
        c.fn('hh/alt-radar-next', MAX(hvr, IFTHEN(GT(alt_rise, v(0.5)), LT(hagl, v(500)), p('hh/alt-radar'))), out='hh/alt-radar',
             desc='1: the radar height held, 0: the barometric altitude')
        h = c.fn('hh/h-ft', IFTHEN(GT(p('hh/alt-radar'), v(0.5)), hagl, hsl))
        thr = '/controls/engines/engine[0]/throttle'
        dl = ABS(D(p(thr), p('hh/tcl-w')))
        moved, moving = GT(dl, v(0.004)), GT(dl, v(1e-5))     # past the hold's own setting; at all
        fly0 = GT(p('hh/alt-fly'), v(0.5))
        c.fn('hh/still-next', IFTHEN(moving, v(0), S(p('hh/still-s'), dt)), out='hh/still-s')
        cap = c.fn('hh/capture', MAX(M(alt_rise, NOT(p('hh/ext-rise'))), M(fly0, NOT(moving), GT(p('hh/still-s'), v(1.5)))),
                   desc='the height captured: on engaging (not a height handed over with a point) or the lever left alone')
        c.fn('hh/alt-fly-next', M(alt, IFTHEN(moved, v(1), IFTHEN(GT(cap, v(0.5)), v(0), p('hh/alt-fly')))), out='hh/alt-fly')
        # the height captured where the present climb or sink stops, at 0.05 g
        vz = p('velocities/h-dot-fps')
        h_stop = c.fn('hh/h-stop-ft', S(h, Q(M(vz, ABS(vz)), v(2 * 0.05 * G0))))
        tgt = '/autopilot/settings/target-agl-ft'
        c.fn('hh/hvr-alt-set', IFTHEN(GT(M(cap, hvr), v(0.5)), MAX(v(0), h_stop), p(tgt)), out=tgt)
        c.fn('hh/h-ref-next', IFTHEN(GT(cap, v(0.5)), h_stop, IFTHEN(hvr, p(tgt), p('hh/h-ref-ft'))), out='hh/h-ref-ft')
        ez = c.fn('hh/vz-err-fps', D(lim(M(p('hh/kh'), D(p('hh/h-ref-ft'), h)), -VZ_MAX, VZ_MAX), vz))
        drive = GT(M(alt, NOT(p('hh/alt-fly'))), v(0.5))
        c.fn('hh/tcl-int-next', IFTHEN(drive, IFTHEN(GT(cap, v(0.5)), p(thr), lim(S(p('hh/tcl-int'), M(p('hh/kiz'), ez, dt)), 0, 1)), p(thr)),
             out='hh/tcl-int')
        c.fn('hh/lever', IFTHEN(drive, lim(S(p('hh/tcl-int'), M(p('hh/kpz'), ez)), 0, 1), p(thr)), out=[thr, 'hh/tcl-w'],
             desc='the lever, driven while the height is held')
        c.fn('hh/state', IFTHEN(NOT(onb), v(0), IFTHEN(trc, v(3), IFTHEN(GT(M(LT(gs, v(1)), LT(e, v(5))), v(0.5)), v(2), v(1)))),
             desc='0 off, 1 closing on the point, 2 holding it, 3 the stick flying')

    # ------------------------------------------------------------ rotor geometry with the nacelle angle
    def hub_station(s, side):
        """the hub's (FS, BL, WL) as expressions of the nacelle angle: the helicopter-mode offset from the pivot turned
        forward by 90 - nacelle angle"""
        m = 1 if side == 'r' else -1
        sn, cn = p('ctl/sin-nac'), p('ctl/cos-nac')       # cos(90 - nac) = sin nac
        # forward = -DX cos f + DZ sin f, up = DX sin f + DZ cos f, f = 90 - nac
        fwd = S(M(v(-U.DX_HUB), sn), M(v(U.DZ_HUB), cn))
        up = S(M(v(U.DX_HUB), cn), M(v(U.DZ_HUB), sn))
        return D(v(U.FS_PIV), fwd), v(m * U.BL_PIV), S(v(U.WL_PIV), up), fwd, up

    def to_s(s, bx, by, bz):
        sn, cn = p('ctl/sin-nac'), p('ctl/cos-nac')
        return S(M(sn, bx), M(cn, bz)), by, S(NEG(M(cn, bx)), M(sn, bz))

    def to_b(s, x, y, z):
        sn, cn = p('ctl/sin-nac'), p('ctl/cos-nac')
        return D(M(sn, x), M(cn, z)), y, S(M(cn, x), M(sn, z))

    # ------------------------------------------------------------ rotor states (FCS, FDM rate)
    def rotor_states(s):
        r = s.rot
        r.declare('rotor/omega-rad_sec', U.OMEGA_100 * U.NR_HELO / 100)
        r.declare('rotor/psi-rad', 0.0)
        for side, m in SIDES:
            for st in ('bc', 'bs', 'bcd', 'bsd', 'v0-fps'):
                r.declare('rotor/%s/%s' % (side, st), 0.0)
            if s.tc:
                for k in range(U.NB):
                    for st in ('qt', 'qtd'):
                        r.declare('rotor/%s/b%d/%s' % (side, k, st), 0.0)
            for st, tg in ((PH_COS, 'c'), (PH_SIN, 's')):
                for i in range(len(st)):
                    r.declare('rotor/%s/ph/%s%d' % (side, tg, i), 0.0)
        dt = p('simulation/dt')
        init = GT(p('ctl/init-fast'), v(0.5))
        r.declare('rotor/accel-hold', 0)
        held = GT(p('rotor/accel-hold'), v(0.5))
        # the gimbals start untilted in the first frames of a run (not after a reset of the initial conditions: the
        # trims keep the rotors' state from one evaluation to the next)
        r.channel('start')
        r.declare('rotor/initialized', 0)
        binit = GT(r.fn('rotor/gimbal-init', LT(p('rotor/initialized'), v(0.5))), v(0.5))
        r.fn('rotor/initialized-next', GT(p('simulation/sim-time-sec'), v(0.02)), out='rotor/initialized')
        for side, m in SIDES:
            pre = 'rotor/%s' % side
            r.channel('gimbal %s' % side)
            # shaft rates (mirror-image frame for the left rotor) with the nacelle's tilting, and accelerations
            Pm, Qm, Rm = M(v(m), p('velocities/p-rad_sec')), S(p('velocities/q-rad_sec'), p('ctl/nacelle-rate-rad_sec')), M(v(m), p('velocities/r-rad_sec'))
            ps, qs, rs = s.to_s(Pm, Qm, Rm)
            ps = r.fn(pre + '/ps', ps); qs = r.fn(pre + '/qs', qs)
            pd, qd, rd = s.to_s(M(v(m), p('accelerations/pdot-rad_sec2')), p('accelerations/qdot-rad_sec2'), M(v(m), p('accelerations/rdot-rad_sec2')))
            pds = r.fn(pre + '/pds', IFTHEN(held, p(pre + '/pds'), pd))
            qds = r.fn(pre + '/qds', IFTHEN(held, p(pre + '/qds'), qd))
            W = p('rotor/omega-rad_sec')
            bc, bs, bcd, bsd = (p(pre + '/' + n) for n in ('bc', 'bs', 'bcd', 'bsd'))
            # gimbal equations in the non-rotating frame (the blades' flap equations projected on cos and sin psi_k):
            #   bc'' + 2 W bs' = 2 / (Nb Ib) (Gc - K bc - C bc') + 2 W ps + qs'
            #   bs'' - 2 W bc' = 2 / (Nb Ib) (Gs - K bs - C bs') - 2 W qs + ps'
            # Gc, Gs the blades' aerodynamic flap moments projected; K the hub spring; a flapping stop beyond FLAP_STOP.
            # The gyroscopic coupling (the tilt's precession at 2 W) is integrated exactly, as a rotation of the rates
            k2 = 2 / (U.NB * U.I_BLADE)
            tilt = r.fn(pre + '/tilt-rad', SQ(S(SQR(bc), SQR(bs), v(1e-12))))
            over = MAX(v(0), D(tilt, v(U.FLAP_STOP)))
            kstop = r.fn(pre + '/k-stop', Q(M(v(30 * U.K_HUB), over), tilt), desc='flapping stop, ft lb/rad of tilt')
            C = 0.02 * 2 * math.sqrt(U.K_HUB * U.NB * U.I_BLADE / 2)          # 2 % of critical on the hub spring
            fc = r.fn(pre + '/fc', S(M(v(k2), D(D(D(p('aero/rotor/%s/Gc' % side), M(v(U.K_HUB), bc)), M(kstop, bc)), M(v(C), bcd))),
                                     M(v(2), W, ps), qds))
            fs_ = r.fn(pre + '/fs', S(M(v(k2), D(D(D(p('aero/rotor/%s/Gs' % side), M(v(U.K_HUB), bs)), M(kstop, bs)), M(v(C), bsd))),
                                      NEG(M(v(2), W, qs)), pds))
            ang = M(v(2), W, dt)
            ca, sa = r.fn(pre + '/rot-c', COS(ang)), r.fn(pre + '/rot-s', SIN(ang))
            # (bcd + i bsd) <- exp(i 2 W dt) (bcd + i bsd + (fc + i fs) dt)
            xc, xs = S(bcd, M(fc, dt)), S(bsd, M(fs_, dt))
            # static tilt at the start: the spring alone against the aerodynamic moment
            r.fn(pre + '/bcd-next', IFTHEN(binit, v(0), D(M(ca, xc), M(sa, xs))), out=pre + '/bcd')
            r.fn(pre + '/bsd-next', IFTHEN(binit, v(0), S(M(sa, xc), M(ca, xs))), out=pre + '/bsd')
            r.fn(pre + '/bc-next', IFTHEN(binit, v(0), S(bc, M(p(pre + '/bcd'), dt))), out=pre + '/bc')
            r.fn(pre + '/bs-next', IFTHEN(binit, v(0), S(bs, M(p(pre + '/bsd'), dt))), out=pre + '/bs')
            if s.tc:
                s.torsion_states(side, fc, fs_, binit)
            s.inflow_states(side)
        r.channel('rotor azimuth')
        psn = S(p('rotor/psi-rad'), M(p('rotor/omega-rad_sec'), p('simulation/dt')))
        r.fn('rotor/psi-next', IFTHEN(GT(psn, v(2 * math.pi)), D(psn, v(2 * math.pi)), psn), out='rotor/psi-rad')

    def torsion_states(s, side, fc, fs_, binit):
        """the blades' first torsion mode (blademodes.torsion_constants), its tip twist qt (rad, nose up +):
            It qt'' = Tq + Scg (fc cos psi + fs sin psi + W^2 precone) - Kt qt - (2 zeta sqrt(Kt It) + Ct rho W) qt'
        Tq the blade elements' moments about the elastic axis and the propeller moment of the blade's whole pitch
        (rotor(), the last frame's); the CG's offset aft of the elastic axis under the gimbal's flap acceleration
        (fc, fs: the gimbal's own, the centrifugal force on its tilt cancelling) and the precone's centrifugal force;
        the structural and control system stiffness, the structure's and the sections' damping. Stepped semi-
        implicitly (31 Hz at 480 Hz); at the start the static twist"""
        r, tc = s.rot, s.tc
        dt, W = p('simulation/dt'), p('rotor/omega-rad_sec')
        cdamp = r.fn('rotor/%s/qt-damp' % side, S(v(2 * tc['zeta'] * math.sqrt(tc['Kt'] * tc['It'])),
                                                  M(v(tc['Ct']), p('atmosphere/rho-slugs_ft3'), W)))
        for k in range(U.NB):
            bk = 'rotor/%s/b%d' % (side, k)
            psi = S(p('rotor/psi-rad'), v(k * 2 * math.pi / U.NB))
            F = r.fn(bk + '/qt-force', S(p('aero/rotor/%s/b%d/Tq' % (side, k)),
                                         M(v(tc['Scg']), S(M(fc, COS(psi)), M(fs_, SIN(psi)), M(v(U.PRECONE), SQR(W))))),
                     desc='torsion mode forcing, ft lb')
            qt, qtd = p(bk + '/qt'), p(bk + '/qtd')
            qtdd = Q(D(F, S(M(v(tc['Kt']), qt), M(cdamp, qtd))), v(tc['It']))
            r.fn(bk + '/qtd-next', IFTHEN(binit, v(0), S(qtd, M(qtdd, dt))), out=bk + '/qtd')
            r.fn(bk + '/qt-next', IFTHEN(binit, Q(F, v(tc['Kt'])), S(qt, M(p(bk + '/qtd'), dt))), clip=('-0.5', '0.5'), out=bk + '/qt')

    def inflow_states(s, side):
        """Peters-He finite-state wake (as the UH-60 and Bo 105 models): M a' + L^-1 V a = tau / 2 per state set, in the
        wake frame; the hub-frame harmonic pairs are turned into it, stepped semi-implicitly, and turned back."""
        r = s.rot
        pre, apre = 'rotor/%s/ph' % side, 'aero/rotor/%s/ph' % side
        r.channel('inflow %s' % side, execrate=INFLOW_EXECRATE)
        s.inflow_start[side] = len(r.channels) - 1
        h = M(MAX(p('rotor/omega-rad_sec'), v(1.0)), p('simulation/channel-dt'))
        cd, sd = p(apre + '/cd'), p(apre + '/sd')
        rot = {1: (cd, sd), 2: (r.fn(pre + '/cd2', D(SQR(cd), SQR(sd))), r.fn(pre + '/sd2', M(v(2), cd, sd)))}
        sets = {'c': PH_COS, 's': PH_SIN}
        pair = {}
        for i, st in enumerate(PH_COS):
            if st[1] > 0 and st in PH_SIN:
                pair[('c', i)] = PH_SIN.index(st); pair[('s', PH_SIN.index(st))] = i

        def wake(tg, i, prop):
            rh = sets[tg][i][1]
            if rh == 0 or (tg, i) not in pair:
                return p(prop % (tg, i))
            cr, sr = rot[rh]; o = 's' if tg == 'c' else 'c'; po = p(prop % (o, pair[(tg, i)]))
            me = p(prop % (tg, i))
            return S(M(me, cr), M(po, sr)) if tg == 'c' else D(M(me, cr), M(po, sr))
        for tg, st in sets.items():
            for i in range(len(st)):
                r.fn(pre + '/w%s%d' % (tg, i), wake(tg, i, pre + '/%s%d'))
                r.fn(pre + '/wtau-%s%d' % (tg, i), wake(tg, i, apre + '/tau-%s%d'))
        for tg, st in sets.items():
            for i, (j, rh) in enumerate(st):
                k_ = Q(h, v(2 / math.pi * PH.H(j, rh)))
                a = lambda i_, kk: p(apre + '/A%s%d-%d' % (tg, i_, kk))
                off = [M(a(i, kk), p(pre + '/w%s%d' % (tg, kk))) for kk in range(len(st)) if kk != i]
                num = S(p(pre + '/w%s%d' % (tg, i)), M(k_, D(M(v(0.5), p(pre + '/wtau-%s%d' % (tg, i))), S(*off) if off else v(0))))
                r.fn(pre + '/n%s%d' % (tg, i), Q(num, S(v(1), M(k_, a(i, i)))))
        for tg, st in sets.items():
            for i, (j, rh) in enumerate(st):
                if rh == 0 or (tg, i) not in pair:
                    r.fn(pre + '/%s%d-next' % (tg, i), p(pre + '/n%s%d' % (tg, i)), out=pre + '/%s%d' % (tg, i))
                else:
                    cr, sr = rot[rh]; o = 's' if tg == 'c' else 'c'; po = p(pre + '/n%s%d' % (o, pair[(tg, i)])); me = p(pre + '/n%s%d' % (tg, i))
                    r.fn(pre + '/%s%d-next' % (tg, i), D(M(me, cr), M(po, sr)) if tg == 'c' else S(M(po, sr), M(me, cr)), out=pre + '/%s%d' % (tg, i))
        r.fn('rotor/%s/v0-out' % side, MAX(v(0), M(v(math.sqrt(3) * U.R), p(pre + '/c0'), p('rotor/omega-rad_sec'))),
             out='rotor/%s/v0-fps' % side, desc='mean inflow, sqrt(3) x the first state, ft/s')

    # ------------------------------------------------------------ proprotor aerodynamics and hub loads
    def rotor(s, side):
        g = s.g
        m = 1 if side == 'r' else -1
        b_ = 'aero/rotor/%s' % side
        st_ = 'rotor/%s' % side
        fs_h, bl_h, wl_h, fwd, up = s.hub_station(side)
        hs = [g.fn(b_ + '/hub-%s-in' % n, e) for n, e in zip(('fs', 'bl', 'wl'), (fs_h, bl_h, wl_h))]
        # hub velocity relative to the air (body axes): the CG's, the airframe's rotation at the hub's arm, the
        # nacelle's tilting about the pivot (rate about +y: nose-up sense, the hub's offset from the pivot)
        rx, ry, rz = s.arm_cg(hs[0], hs[1], hs[2])
        Pa, Qa, Ra = p('velocities/p-aero-rad_sec'), p('velocities/q-aero-rad_sec'), p('velocities/r-aero-rad_sec')
        bn = p('ctl/nacelle-rate-rad_sec')
        ox, oz = Q(fwd, v(12)), NEG(Q(up, v(12)))     # hub from the pivot, body axes (ft)
        vb = (S(p('velocities/u-aero-fps'), D(M(Qa, rz), M(Ra, ry)), M(bn, oz)),
              S(p('velocities/v-aero-fps'), D(M(Ra, rx), M(Pa, rz))),
              S(p('velocities/w-aero-fps'), D(M(Pa, ry), M(Qa, rx)), NEG(M(bn, ox))))
        vm = (vb[0], M(v(m), vb[1]), vb[2])           # the mirror-image aircraft's, for the left rotor
        Vhx, Vhy, Vhz = [g.fn(b_ + '/Vh%s-fps' % a, e) for a, e in zip('xyz', s.to_s(*vm))]
        ps_a, qs_a, rs_a = s.to_s(M(v(m), Pa), S(Qa, bn), M(v(m), Ra))
        ps_a, qs_a, rs_a = g.fn(b_ + '/ps-aero', ps_a), g.fn(b_ + '/qs-aero', qs_a), g.fn(b_ + '/rs-aero', rs_a)
        Vip = g.fn(b_ + '/Vinplane-fps', SQ(S(SQR(Vhx), SQR(Vhy), v(1e-6))))
        v0 = p(st_ + '/v0-fps')
        Vn = g.fn(b_ + '/Vnormal-fps', D(v0, Vhz), 'flow through the disc (into the wake +)')
        g.fn(b_ + '/Vprime-fps', SQ(S(SQR(Vip), SQR(Vn), v(25.0))), 'mass flow velocity (5 ft/s floor)')
        chi = g.fn(b_ + '/chi-rad', ATAN2(Vip, MAX(Vn, v(0.5))), 'wake skew angle')
        g.fn(b_ + '/chi-deg', M(v(R2D), chi))
        ps = ATAN2(Vhy, Vhx)
        g.fn(b_ + '/psi-flow-deg', M(v(R2D), IFTHEN(LT(ps, v(0)), S(ps, v(2 * math.pi)), ps)),
             'direction of the hub\'s motion in the disc\'s plane, from its x axis towards y')
        W = p('rotor/omega-rad_sec')
        g.fn(b_ + '/mu', Q(Vip, MAX(M(W, v(U.R)), v(30))), 'advance ratio')
        # ground effect: hub height over the rotor radius, the wake's angle from the ground's normal
        hh = g.fn(b_ + '/hub-agl-ft', S(p('position/h-agl-ft'), Q(D(hs[2], p('inertia/cg-z-in')), v(12))))
        zr = g.fn(b_ + '/ge-z-r', MAX(v(GE_ZR_MIN), Q(hh, v(U.R))))
        th_, ph_ = p('attitude/theta-rad'), p('attitude/phi-rad')
        dn = s.to_s(NEG(SIN(th_)), M(v(m), SIN(ph_), COS(th_)), M(COS(ph_), COS(th_)))
        wd = g.fn(b_ + '/ge-wake-down', S(M(NEG(Vhx), dn[0]), M(NEG(Vhy), dn[1]), M(Vn, dn[2])))
        w2 = S(SQR(Vip), SQR(Vn))
        g.fn(b_ + '/ge-chi-deg', M(v(R2D), ATAN2(SQ(MAX(v(0), D(w2, SQR(wd)))), wd)), 'wake angle from the ground normal')
        kh = tag('pow', S(v(1), Q(v(GE_K), SQR(zr))), v(GE_EXP))
        g.fn(b_ + '/kge', S(v(1), M(D(kh, v(1)), T2(b_ + '/ge-z-r', b_ + '/ge-chi-deg', HG.ZR, HG.CHI_DEG, HG.RATIO))),
             'induced velocity in / out of ground effect')
        g.fn(b_ + '/vi-fps', M(p(b_ + '/kge'), v0), 'mean induced velocity with ground effect')
        # Peters-He: flow parameters and gain matrices against the wake skew X = tan(chi / 2), and below the forcing:
        # written at the start of the inflow channel, at its rate (ph)
        ph = FCS('ph'); ph.channel('ph')
        WR = g.fn(b_ + '/ph/omega-r-fps', MAX(M(W, v(U.R)), v(30)))
        X = ph.fn(b_ + '/ph/X', Q(SIN(chi), S(v(1), COS(chi))))
        VT = ph.fn(b_ + '/ph/VT', Q(p(b_ + '/Vprime-fps'), WR))
        mu, lm, la = Q(Vip, WR), Q(v0, WR), Q(Vn, WR)
        ph.fn(b_ + '/ph/V', MAX(Q(v(5), WR), Q(S(SQR(mu), M(S(la, lm), la)), VT)))
        Xs = [0.05 * i for i in range(21)]
        for st, tg, kind in ((PH_COS, 'c', 'c'), (PH_SIN, 's', 's')):
            Li = [np.linalg.inv(PH.Lmat(st, x, kind)) for x in Xs]
            for i in range(len(st)):
                for kk in range(len(st)):
                    Vk = VT if st[kk] == (1, 0) else p(b_ + '/ph/V')
                    ph.fn(b_ + '/ph/A%s%d-%d' % (tg, i, kk), M(T1(b_ + '/ph/X', Xs, [float(L_[i, kk]) for L_ in Li]), Vk))
        nw = SQ(S(SQR(Vhx), SQR(Vhy)))
        slow = LT(nw, v(0.5))
        ph.fn(b_ + '/ph/cd', IFTHEN(slow, v(1), Q(Vhx, MAX(nw, v(0.5)))))
        ph.fn(b_ + '/ph/sd', IFTHEN(slow, v(0), NEG(Q(Vhy, MAX(nw, v(0.5))))))
        ph_tau = {('c', i): [] for i in range(len(PH_COS))}; ph_tau.update({('s', i): [] for i in range(len(PH_SIN))})
        rho, a_snd = p('atmosphere/rho-slugs_ft3'), p('atmosphere/a-fps')
        shapes = sorted({x for sts in (PH_COS, PH_SIN) for x in sts})
        bc, bs, bcd, bsd = (p(st_ + '/' + n) for n in ('bc', 'bs', 'bcd', 'bsd'))
        kp = -math.tan(U.DELTA3)              # pitch per flap: delta-3 -15 deg, flap up pitch up
        # controls in this rotor's (mirror-image) frame
        th75 = p('ctl/%s/theta75-deg' % side)
        b1s = p('ctl/%s/b1s-deg' % side)
        Fx, Fy, Fz, Gc, Gs, Qt, T_ = [], [], [], [], [], [], []
        hrx, hry, hrz = s.arm_rp(hs[0], hs[1], hs[2])
        for k in range(U.NB):
            b = b_ + '/b%d' % k
            psi = g.fn(b + '/psi', S(p('rotor/psi-rad'), v(k * 2 * math.pi / U.NB)), 'azimuth of the blade')
            c, sn = g.fn(b + '/cpsi', COS(psi)), g.fn(b + '/spsi', SIN(psi))
            beta = g.fn(b + '/beta-rad', S(v(U.PRECONE), M(bc, c), M(bs, sn)), 'flap: precone and the gimbal\'s tilt')
            gb = D(beta, v(U.PRECONE))
            bdot = g.fn(b + '/betadot', S(M(bcd, c), M(bsd, sn), M(W, D(M(bs, c), M(bc, sn)))))
            cb, sb = g.fn(b + '/cbeta', COS(beta)), g.fn(b + '/sbeta', SIN(beta))
            th0 = g.fn(b + '/theta-deg', S(th75, NEG(M(b1s, sn)), M(v(kp * R2D), gb)), 'pitch at 0.75 R, deg')
            # velocities of a segment at radius r (shaft axes), linear in r:
            #   Vt = Vh.t + r [cos b (W - r_s) - sin b (w.e_r)],  Vn = Vh.n_b + r (b' - w.e_t) + vi cos b
            vht = S(M(Vhx, sn), M(Vhy, c))
            vher = S(NEG(M(Vhx, c)), M(Vhy, sn))
            wer = S(NEG(M(ps_a, c)), M(qs_a, sn))
            wet = S(M(ps_a, sn), M(qs_a, c))
            # induced velocity of each Peters-He shape at the blade, along its normal (x cos b); the first shape's is the
            # same at every radius and goes into the blade's own normal velocity
            trig = {1: (c, sn)}
            tf = lambda tg, rh: v(1) if rh == 0 else trig[rh][0 if tg == 'c' else 1]
            vi_shape, vi10 = {}, None
            for (jj, rh) in shapes:
                lam = S(*[M(p('rotor/%s/ph/%s%d' % (side, tg, i)), tf(tg, rh)) if rh else p('rotor/%s/ph/%s%d' % (side, tg, i))
                          for sts, tg in ((PH_COS, 'c'), (PH_SIN, 's')) for i, st in enumerate(sts) if st == (jj, rh)])
                if (jj, rh) == (1, 0):
                    vi10 = M(v(PHI10), p(b_ + '/kge'), WR, lam, cb)
                else:
                    vi_shape[(jj, rh)] = g.fn(b + '/vi-%d-%d' % (jj, rh), M(p(b_ + '/kge'), WR, lam, cb))
            t0 = g.fn(b + '/Vt0', vht)
            t1 = g.fn(b + '/Vt1', D(M(cb, D(W, rs_a)), M(sb, wer)))
            n0 = g.fn(b + '/Vn0', S(D(NEG(M(sb, vher)), M(cb, Vhz)), vi10))
            n1 = g.fn(b + '/Vn1', D(bdot, wet))
            vr = g.fn(b + '/Vr', S(M(cb, vher), NEG(M(sb, Vhz))))
            FT, FN, DK, MF, QQ, TQ = [], [], [], [], [], []
            qt = p('rotor/%s/b%d/qt' % (side, k)) if s.tc else None
            for j, (rr, dr) in enumerate(s.segs):
                e = b + '/s%d' % j
                tt, cli = SEC.interp_sections(U.SECTIONS, rr / U.R)
                sec = SEC.Section(tt, cli)
                al, mach, cl_tab, cd_tab = sec.tables()
                al, (cl_tab, cd_tab) = periodic(al, cl_tab, cd_tab)
                ch = chord(rr)
                vi = [M(v(PH.phi(jj, rh, rr / U.R)), vi_shape[(jj, rh)]) for (jj, rh) in shapes if (jj, rh) != (1, 0)]
                Vt = g.fn(e + '/Vt', S(t0, M(v(rr), t1)), 'blade velocity along its motion, ft/s')
                Vnn = g.fn(e + '/Vn', S(n0, M(v(rr), n1), *vi), 'blade velocity along its normal (up), ft/s')
                V2 = g.fn(e + '/V2', S(SQR(Vt), SQR(Vnn), v(1e-4)))
                Vv = SQ(V2)
                # the section's pitch: the blade's, its twist and the torsion mode's elastic twist
                thj = S(th0, v(twist(rr))) if not s.tc else S(th0, v(twist(rr)), M(v(R2D * s.tc['phi'][j]), qt))
                g.fn(e + '/alpha-deg', D(thj, M(v(R2D), ATAN2(Vnn, Vt))), 'angle of attack, deg (-360..360)')
                g.fn(e + '/mach', MIN(Q(Vv, a_snd), v(1.0)))
                cl = g.fn(e + '/cl', T2(e + '/alpha-deg', e + '/mach', al, mach, cl_tab))
                cd = g.fn(e + '/cd', S(T2(e + '/alpha-deg', e + '/mach', al, mach, cd_tab), v(U.DCD)))
                r_in, r_out = s.edges[j], s.edges[j + 1]
                fl = max(0.0, min(1.0, (U.TIP_LOSS * U.R - r_in) / (r_out - r_in)))
                Lk = g.fn(e + '/Lk', M(v(0.5 * ch * dr * fl), rho, Vv, cl), 'lift / V')
                Dk = g.fn(e + '/Dk', M(v(0.5 * ch * dr), rho, cd, SQ(S(V2, SQR(vr)))), 'drag / V')
                Ft = g.fn(e + '/Ft', NEG(S(M(Lk, Vnn), M(Dk, Vt))), 'force along the blade motion, lbf')
                Fn = g.fn(e + '/Fn', D(M(Lk, Vt), M(Dk, Vnn)), 'force along the blade normal (up), lbf')
                FT.append(Ft); FN.append(Fn); DK.append(Dk)
                MF.append(M(v(rr), Fn)); QQ.append(M(v(rr), Ft))
                if s.tc:
                    # its moment about the elastic axis in the mode: the section's at the quarter chord and the normal
                    # force there, aft of the axis
                    tc = s.tc
                    TQ.append(M(v(tc['phi'][j] * tc['cm0'][j] * 0.5 * ch * ch * dr), rho, V2))
                    if tc['d_ac'][j]:
                        TQ.append(M(v(-tc['phi'][j] * tc['d_ac'][j]), Fn))
            ft, fn, fr = g.fn(b + '/FT', S(*FT)), g.fn(b + '/FN', S(*FN)), g.fn(b + '/FR', NEG(M(vr, S(*DK))))
            mf = g.fn(b + '/Mflap', S(*MF), 'aerodynamic flap moment about the gimbal, ft lbf (up +)')
            if s.tc:
                # and the propeller moment of the sections' pitch, - W^2 sum P_j sin(2 theta_j) / 2, theta_j the blade's
                # pitch a, the section's twist and its elastic twist phi_j qt: in a and qt, exact in the first and linear
                # in the second (a few degrees), - W^2 (A1 sin 2a + A2 cos 2a + qt (B1 cos 2a - B2 sin 2a))
                tw = [math.radians(twist(rr)) for rr, dr in s.segs]
                P, ph_ = s.tc['P'], s.tc['phi']
                A1 = sum(P[j] * math.cos(2 * tw[j]) for j in range(len(tw))) / 2
                A2 = sum(P[j] * math.sin(2 * tw[j]) for j in range(len(tw))) / 2
                B1 = sum(P[j] * ph_[j] * math.cos(2 * tw[j]) for j in range(len(tw)))
                B2 = sum(P[j] * ph_[j] * math.sin(2 * tw[j]) for j in range(len(tw)))
                a2 = M(v(2 / R2D), th0)
                s2, c2 = SIN(a2), COS(a2)
                prop = M(SQR(W), S(M(v(A1), s2), M(v(A2), c2), M(qt, D(M(v(B1), c2), M(v(B2), s2)))))
                g.fn(b + '/Tq', D(S(*TQ), prop), 'torsion mode\'s generalized moment: the sections\' and the propeller moment, ft lbf')
            qb = g.fn(b + '/Q', M(cb, S(*QQ)), 'aerodynamic moment about the shaft, in the rotation\'s sense, ft lbf')
            # the inflow's forcing: each shape's moment of the normal loads along the shaft
            for (jj, rh) in shapes:
                shp = ph.fn(b + '/Fn-%d-%d' % (jj, rh), M(cb, S(*[M(v(PH.phi(jj, rh, rr / U.R)), p(b + '/s%d/Fn' % j)) for j, (rr, dr) in enumerate(s.segs)])))
                for sts, tg in ((PH_COS, 'c'), (PH_SIN, 's')):
                    for i, st in enumerate(sts):
                        if st == (jj, rh):
                            ph_tau[(tg, i)].append(M(tf(tg, rh), shp))
            # to shaft axes: t = (s, c, 0), n_b = (sb c, -sb s, -cb), d = (-cb c, cb s, -sb)
            Fx.append(S(M(ft, sn), M(fn, sb, c), NEG(M(fr, cb, c))))
            Fy.append(S(M(ft, c), NEG(M(fn, sb, sn)), M(fr, cb, sn)))
            Fz.append(NEG(S(M(fn, cb), M(fr, sb))))
            T_.append(S(M(fn, cb), M(fr, sb)))
            Gc.append(M(mf, c)); Gs.append(M(mf, sn)); Qt.append(qb)
        g.fn(b_ + '/thrust-lbs', S(*T_), 'thrust along the shaft (up the shaft)')
        g.fn(b_ + '/Gc', S(*Gc), 'blades\' flap moments projected on cos psi')
        g.fn(b_ + '/Gs', S(*Gs), 'blades\' flap moments projected on sin psi')
        Qa = g.fn(b_ + '/torque-ftlb', NEG(S(*Qt)), 'aerodynamic torque against the rotation, ft lbf')
        g.fn(b_ + '/power-hp', Q(M(Qa, W), v(550)))
        # Peters-He forcing: tau = (1/2pi or 1/pi) sum of the normal loads x shape / (rho W^2 R^4), over the effective
        # disc B^2 A (tip loss in the momentum balance)
        qn = MAX(M(rho, SQR(MAX(W, v(5))), v(U.R ** 4)), v(1e-6))
        for (tg, i), terms in ph_tau.items():
            rh = (PH_COS if tg == 'c' else PH_SIN)[i][1]
            kb = 1 / U.TIP_LOSS ** 2
            ph.fn(b_ + '/ph/tau-%s%d' % (tg, i), Q(M(v(kb / (2 * math.pi) if rh == 0 else kb / math.pi), S(*terms)), qn))
        s.rot.channels[s.inflow_start[side]][1][0:0] = ph.channels[0][1]
        if getattr(U, 'SPINNER_DRAG', None):
            # the spinner's drag, on the flow through the disc (the hub's motion and the induced velocity): its area
            # along the mast and across it, D = q' (S0 + S1 sin^2 a) against the flow (shaft axes)
            S0, S1 = U.SPINNER_DRAG
            Vp = p(b_ + '/Vprime-fps')
            ks = g.fn(b_ + '/spinner-k', M(v(0.5), rho, S(M(v(S0), Vp), Q(M(v(S1), SQR(Vip)), Vp))))
            Fx.append(NEG(M(ks, Vhx))); Fy.append(NEG(M(ks, Vhy))); Fz.append(M(ks, Vn))
        Fs = [g.fn(b_ + '/F%s-s' % a, S(*x)) for a, x in zip('xyz', (Fx, Fy, Fz))]
        # moments on the airframe in shaft axes: the hub spring's (and its damping and stop) and the drive's reaction
        C = 0.02 * 2 * math.sqrt(U.K_HUB * U.NB * U.I_BLADE / 2)
        kk = S(v(U.K_HUB), p(st_ + '/k-stop'))
        Mxs = NEG(S(M(kk, bs), M(v(C), bsd)))
        Mys = NEG(S(M(kk, bc), M(v(C), bcd)))
        Mzs = p('drive/%s/shaft-torque-ftlb' % side)
        Fb = s.to_b(*Fs)
        Mb = s.to_b(Mxs, Mys, Mzs)
        # back to the aircraft's own frame
        F = [g.fn(b_ + '/F%s-lbs' % a, e) for a, e in zip('XYZ', (Fb[0], M(v(m), Fb[1]), Fb[2]))]
        Mo = [M(v(m), Mb[0]), Mb[1], M(v(m), Mb[2])]
        Mo = [g.fn(b_ + '/M%s-ftlb' % a, S(mo, cr)) for a, mo, cr in zip('LMN', Mo, (D(M(hry, F[2]), M(hrz, F[1])), D(M(hrz, F[0]), M(hrx, F[2])),
                                                                                  D(M(hrx, F[1]), M(hry, F[0]))))]
        s.add(F, Mo)
        # gimbal tilt (deg) for the tests: + forward, + towards the right (the rotor's own frame)
        g.fn(b_ + '/a1-deg', M(v(R2D), bc), 'gimbal tilt, longitudinal: + disc forward (aft blade up)')
        g.fn(b_ + '/b1-deg', M(v(-R2D), bs), 'gimbal tilt, lateral: + disc towards the rotor\'s outboard side... (mirror frame: right)')

    # ------------------------------------------------------------ wakes on the airframe
    def wake(s, g, target):
        """the induced air velocity of both rotors' wakes over a target (body axes, ft/s, air velocity: to be
        subtracted from the target's), from the ring-vortex tables (rotorwake.py). The right rotor's tables are for
        each target; the left rotor's is the right rotor's for the mirror-image target, mirrored."""
        mirror = {'stab': 'stab', 'finr': 'finl', 'finl': 'finr'}
        if target.startswith('w'):
            mirror[target] = 'w' + ('l' if target[1] == 'r' else 'r') + target[2:]
        d = s.wake_tab
        out = [[], [], []]
        for side, m in SIDES:
            b_ = 'aero/rotor/%s' % side
            name = target if side == 'r' else mirror[target]
            tab = d['k'][name]           # [nacelle][chi][psi][xyz]
            for i, a in enumerate('xyz'):
                data = [[[c[i] for c in row] for row in t] for t in tab]
                k = g.fn('aero/wake/%s-%s/k%s' % (target, side, a), T3(b_ + '/chi-deg', b_ + '/psi-flow-deg', 'ctl/nacelle-deg',
                                                                       d['chi'], d['psi'], d['nacelle'], data))
                out[i].append(M(v(m if a == 'y' else 1), k, p(b_ + '/vi-fps')))
        return [S(*o) for o in out]

    # ------------------------------------------------------------ airframe (lower rate)
    def airframe(s):
        s.air.channel('airframe', execrate=AIRFRAME_EXECRATE)
        s.g = ChannelGraph(s.air)
        axes = (s.X, s.Y, s.Z, s.L, s.Mm, s.N)
        s.X, s.Y, s.Z, s.L, s.Mm, s.N = [], [], [], [], [], []
        s.wing()
        s.fuselage()
        s.empennage()
        loads = [s.g.fn('aero/airframe/%s' % n, S(*terms) if terms else v(0))
                 for n, terms in zip(('FX-lbs', 'FY-lbs', 'FZ-lbs', 'ML-ftlb', 'MM-ftlb', 'MN-ftlb'), (s.X, s.Y, s.Z, s.L, s.Mm, s.N))]
        s.X, s.Y, s.Z, s.L, s.Mm, s.N = axes
        s.g = s.aero
        s.add(loads[:3], loads[3:])

    # ------------------------------------------------------------ wing [CR B-12 to B-23, data/bell301.py]
    def wing_tables(s):
        """Bell's wing-pylon lift and drag coefficients over +-180 deg: against the angle of attack (rows), the flap
        index fx (columns: -1 F4, 0 F1, 1 F2, 2 F3) and Bell's mast angle (tables: 0 helicopter, 90 airplane). Past the
        data (+-40 deg; F2 and F3 to -90) the coefficients blend over 10 deg into a flat plate's, normal force CN90
        sin(alpha), CN90 the data's at -90 deg (F2's for F1 and F4, which have none). U.WING_DATA 'gtrs20' adds
        Ferguson's 20/12.5 [data/gtrs.py] at the flap index 0.5; 'gtrs' also takes his F1, F2 and F3: his within +-40
        deg, Bell's beyond (the download's factors are fitted to Bell's), and beyond them the 20/12.5 halfway between F1
        and F2 (as the index's interpolation without it)."""
        al = sorted(set(list(range(-180, 181, 10)) + list(range(-40, 41, 2)) + [-45, 45, -85, -95, 85, 95]))
        cn90 = {(F, bm): B.WIV[('F2' if F in ('F1', 'F4') else F, bm)][0] for F in ('F1', 'F2', 'F3', 'F4') for bm in (0, 90)}
        src = getattr(U, 'WING_DATA', 'bell')

        def pts(xs, ys, keep):
            return [(a, y) for a, y in zip(xs, ys) if y is not None and keep(a)]

        def ferguson(F, bm):
            """Ferguson's within +-40 deg for Bell's setting F, or '20' for his 20/12.5"""
            if F == 'F1':
                return ((GTRS.WP_ALPHA1, GTRS.CL_WP1[0.2] if bm == 90 else GTRS.CL_WP1_HELO),
                        (GTRS.WD_ALPHA1, GTRS.CD_WP1[0.2] if bm == 90 else GTRS.CD_WP1_HELO))
            G = {'20': 'F2', 'F2': 'F3', 'F3': 'F4'}[F]
            return tuple(tuple(map(list, zip(*pts(xs, tab[(G, bm)], lambda a: abs(a) <= 40))))
                         for xs, tab in ((GTRS.WP_ALPHA2, GTRS.CL_WP), (GTRS.WD_ALPHA2, GTRS.CD_WP)))

        def data(F, bm):
            if F == 'F1':
                bell = (B.WI_ALPHA, B.WI_APLN[0.2] if bm == 90 else B.WI_HELO), (B.WIII_ALPHA, B.WIII_APLN[0.2] if bm == 90 else B.WIII_HELO)
            else:
                bell = tuple(tuple(map(list, zip(*pts(xs, tab[(F, bm)], lambda a: True))))
                             for xs, tab in ((B.WII_ALPHA, B.WII), (B.WIV_ALPHA, B.WIV)))
            if src != 'gtrs' or F == 'F4':
                return bell
            return tuple(tuple(map(list, zip(*sorted(pts(xb, yb, lambda a: abs(a) > 40) + pts(xf, yf, lambda a: True)))))
                         for (xb, yb), (xf, yf) in zip(bell, ferguson(F, bm)))

        def ext(xs, ys, a, kind, cn):
            r = math.radians(a)
            plate = cn * math.sin(r) * (math.cos(r) if kind == 'l' else math.sin(r))
            if kind == 'd':
                plate = max(plate, 0.02)
            lo, hi = xs[0], xs[-1]
            if lo <= a <= hi:
                return float(np.interp(a, xs, ys))
            edge = lo if a < lo else hi
            w = min(1.0, abs(a - edge) / 10.0)
            return (1 - w) * float(np.interp(edge, xs, ys)) + w * plate
        sets = ['F4', 'F1', '20', 'F2', 'F3'] if src in ('gtrs20', 'gtrs') else ['F4', 'F1', 'F2', 'F3']
        CL, CD = [], []
        for bm in (0, 90):
            tl, td = [], []
            for a in al:
                rl, rd = [], []
                for F in sets:
                    if F == '20':
                        continue
                    (xl, yl), (xd, yd) = data(F, bm)
                    # the download factor, from 40 deg towards the flow's normal incidence (full from 60 deg)
                    k = 1 + (DL_K[F] - 1) * min(1.0, max(0.0, (abs(a) - 40) / 20.0)) if abs(a) <= 120 else DL_K[F]
                    rl.append(k * ext(xl, yl, a, 'l', cn90[(F, bm)]))
                    rd.append(k * ext(xd, yd, a, 'd', cn90[(F, bm)]))
                if '20' in sets:
                    (xl, yl), (xd, yd) = ferguson('20', bm)
                    inside = -40 <= a <= 40
                    rl.insert(2, float(np.interp(a, xl, yl)) if inside else (rl[1] + rl[2]) / 2)
                    rd.insert(2, float(np.interp(a, xd, yd)) if inside else (rd[1] + rd[2]) / 2)
                tl.append(rl); td.append(rd)
            CL.append(tl); CD.append(td)
        return al, ([-1, 0, 0.5, 1, 2] if '20' in sets else [-1, 0, 1, 2]), [0, 90], CL, CD

    def wing(s):
        """The wing in NSTRIP spanwise strips a side, each in its own flow: the airframe's motion at the strip and both
        rotors' wakes (rotorwake.py), with Bell's wing-pylon coefficients of the whole wing (which carry the pylons and
        the wing's aspect ratio) for the strip's own angle of attack. In the hover the strips under the discs stand in
        the wakes at about -90 deg: the download. Each strip carries its share of the reference area. The flaperons'
        symmetric deflection selects the flap index (interpolating between Bell's flap settings); their differential
        (the ailerons) adds the rolling and yawing moments and the lift loss of GTRS [CR-166536]; the dihedral effect
        is Bell's."""
        g = s.g
        al, fx_cols, bm_tabs, CL, CD = s.wing_tables()
        fdeg = [-17.5, 0.0, 25.0, 47.0] if U.NAME == 'XV-15' else [-17.5, 0.0, 40.0, 67.0]
        fx = g.fn('aero/wing/flap-index', T1('ctl/flaperon-deg', fdeg, [-1, 0, 1, 2]), 'Bell\'s flap setting: -1 F4, 0 F1, 1 F2, 2 F3')
        # compressibility of the clean wing in airplane mode [Table WI, WIII]: increments over Mach 0.2
        ma = [0.2, 0.4, 0.5, 0.6]
        if getattr(U, 'WING_DATA', 'bell') == 'gtrs':
            la, lt, da_, dt = GTRS.WP_ALPHA1, GTRS.CL_WP1, GTRS.WD_ALPHA1, GTRS.CD_WP1       # Ferguson's [tables 4-I, 4-III]
        else:
            la, lt, da_, dt = B.WI_ALPHA, B.WI_APLN, B.WIII_ALPHA, B.WIII_APLN
        dcl = [[lt[mm][i] - lt[0.2][i] for mm in ma] for i in range(len(la))]
        dcd = [[dt[mm][i] - dt[0.2][i] for mm in ma] for i in range(len(da_))]
        wM = g.fn('aero/wing/mach-weight', M(MAX(v(0), D(v(1), ABS(fx))), Q(p('ctl/beta-m-deg'), v(90))))
        rho = p('atmosphere/rho-slugs_ft3')
        strips = rotorwake.wing_strips(U)
        Sstrip = U.S_W / (2 * len(strips))
        if getattr(U, 'WING_DATA', 'bell') == 'gtrs':
            # Ferguson's moment at zero lift by the flap setting and mast angle [table 4-VIII]; the reflex Bell's
            cm0 = T2('aero/wing/flap-index', 'ctl/beta-m-deg', [-1, 0, 0.5, 1, 2], GTRS.CM_WP_BETAM,
                     [[B.CM0['F4']] * len(GTRS.CM_WP_BETAM)] + [GTRS.CM_WP[F] for F in ('F1', 'F2', 'F3', 'F4')])
        else:
            cm0 = T1('aero/wing/flap-index', [-1, 0, 1, 2], [B.CM0['F4'], B.CM0['F1'], B.CM0['F2'], B.CM0['F3']])
        g.fn('aero/wing/cm0', cm0)
        lifts, downs = [], []
        for side, m in SIDES:
            for i, (a, b, y, fs, wl) in enumerate(strips):
                tg = 'aero/wing/%s%d' % (side, i)
                w = s.wake(g, 'w%s%d' % (side, i)) if True else None
                Vx, Vy, Vz = s.point_velocity(g, tg, fs, m * y, wl, w)
                Vn = g.fn(tg + '/V-fps', MAX(SQ(S(SQR(Vx), SQR(Vz))), v(0.01)))
                g.fn(tg + '/alpha-deg', wrap180(S(M(v(R2D), ATAN2(Vz, Vx)), v(U.WING_DALPHA))))
                g.fn(tg + '/mach', Q(Vn, p('atmosphere/a-fps')))
                cl = g.fn(tg + '/cl', S(T3(tg + '/alpha-deg', 'aero/wing/flap-index', 'ctl/beta-m-deg', al, fx_cols, bm_tabs, CL),
                                        M(p('aero/wing/mach-weight'), T2(tg + '/alpha-deg', tg + '/mach', la, ma, dcl))))
                cd = g.fn(tg + '/cd', S(T3(tg + '/alpha-deg', 'aero/wing/flap-index', 'ctl/beta-m-deg', al, fx_cols, bm_tabs, CD),
                                        M(p('aero/wing/mach-weight'), T2(tg + '/alpha-deg', tg + '/mach', da_, ma, dcd))))
                qs = g.fn(tg + '/qS', M(v(0.5 * Sstrip), rho, SQR(Vn)))
                Fx = g.fn(tg + '/Fx-lbs', M(qs, Q(D(M(cl, Vz), M(cd, Vx)), Vn)))
                Fz = g.fn(tg + '/Fz-lbs', M(qs, Q(NEG(S(M(cl, Vx), M(cd, Vz))), Vn)))
                My = M(qs, v(U.C_W), cm0)
                rx, ry, rz = s.arm_rp(fs, m * y, wl)
                s.add((Fx, None, Fz), (M(ry, Fz), S(D(M(rz, Fx), M(rx, Fz)), My), NEG(M(ry, Fx))))
                lifts.append(NEG(Fz))
                if getattr(U, 'PYLON_INTERFERENCE', None) and i == len(strips) - 1:
                    # the drag where the wing meets the pylon, half of both pylons' area here at the outer strip, on the
                    # free stream's dynamic pressure (Ferguson's "wing dynamic pressure in the wake" is the free stream's:
                    # his rotors induce nothing at the wing), along the free stream
                    bm_, dq_ = U.PYLON_INTERFERENCE
                    ua, wa = p('velocities/u-aero-fps'), p('velocities/w-aero-fps')
                    kq = g.fn(tg + '/pylon-k', M(v(0.25), T1('ctl/beta-m-deg', bm_, dq_), rho, SQ(S(SQR(ua), SQR(wa), v(1e-4)))))
                    Fpx, Fpz = g.fn(tg + '/pylon-Fx-lbs', NEG(M(kq, ua))), g.fn(tg + '/pylon-Fz-lbs', NEG(M(kq, wa)))
                    s.add((Fpx, None, Fpz), (M(ry, Fpz), D(M(rz, Fpx), M(rx, Fpz)), NEG(M(ry, Fpx))))
        g.fn('aero/wing/lift-lbs', S(*lifts), 'the wing\'s force up (body axes); in the hover minus its download')
        g.fn('aero/wing/download-lbs', NEG(p('aero/wing/lift-lbs')))
        # ailerons, dihedral effect: on the freestream dynamic pressure and angle of attack. The ailerons as revised
        # against the XV-15's flight tests [GTRS, data/gtrs.py]: their rolling moment (its effectiveness by the flap
        # setting and mast angle, fading from 8 to 25 deg of the wing's angle of attack), their yawing moment and the
        # wing's lift they lose either way; GTRS's flap settings 0/0, 20/12.5, 40/25, 75/47 at the flap index 0, 0.5,
        # 1, 2 (Bell's reflexed setting, -1, has none: the 0/0's)
        fcols, fset = [-1, 0, 0.5, 1, 2], ['F1', 'F1', 'F2', 'F3', 'F4']
        q = p('aero/qbar-psf')
        aw = g.fn('aero/wing/alpha-deg', S(p('aero/alpha-deg'), v(U.WING_DALPHA)))
        fade = T1('aero/wing/alpha-deg', [-B.ALPHA_LDA_ZERO, -8, 8, B.ALPHA_LDA_ZERO], [0, 1, 1, 0])
        kl = T2('aero/wing/flap-index', 'ctl/beta-m-deg', fcols, [0, 90], [list(GTRS.K_LDA[F]) for F in fset])
        clda = g.fn('aero/wing/cl-da', M(v(GTRS.CL_DA), kl, fade), 'rolling moment coefficient per deg of aileron')
        cl_wing = g.fn('aero/wing/CL', Q(p('aero/wing/lift-lbs'), MAX(M(q, v(U.S_W)), v(1))))
        kn0 = T2('aero/wing/flap-index', 'ctl/beta-m-deg', fcols, [0, 30, 90], [GTRS.CN_DA[F][1] for F in fset])
        kn = T2('aero/wing/flap-index', 'ctl/beta-m-deg', fcols, [0, 30, 90], [GTRS.CN_DA[F][2] for F in fset])
        # (+ right flaperon up rolls right: Bell's da = (dl - dr) / 2)
        da = p('ctl/aileron-deg')
        qsb = M(q, v(U.S_W * U.B_W))
        L_a = M(qsb, clda, da)
        N_a = M(qsb, S(kn0, M(kn, MAX(v(-1), MIN(v(2), cl_wing)), clda)), da)
        dL = g.fn('aero/wing/aileron-lift-loss-lbs', M(q, v(U.S_W), T1('aero/wing/flap-index', fcols, [GTRS.CL_DA_LIFT[F] for F in fset]), ABS(da)))
        a_ = p('aero/alpha-rad')
        Fx, Fz = NEG(M(dL, SIN(a_))), M(dL, COS(a_))
        rx, ry, rz = s.arm_rp(U.FS_W_TIP, 0.0, strips[0][4])
        s.add((Fx, None, Fz), (M(ry, Fz), D(M(rz, Fx), M(rx, Fz)), NEG(M(ry, Fx))))
        clb = S(T2('ctl/beta-m-deg', 'aero/mag-beta-deg', B.CLB_BETAM, B.CLB_BETA, B.CLB0),
                M(T2('ctl/beta-m-deg', 'aero/mag-beta-deg', B.CLB_BETAM, B.CLB_BETA, B.CLB_CL), MAX(v(-1), MIN(v(2), cl_wing))))
        L_b = M(qsb, clb, p('aero/beta-rad'))
        s.add((None, None, None), (g.fn('aero/wing/L-ailerons-ftlb', S(L_a, L_b)), None, g.fn('aero/wing/N-ailerons-ftlb', N_a)))

    # ------------------------------------------------------------ fuselage [CR B-10, B-11, A-22, A-23]
    def fuselage(s):
        """Bell's fuselage equations in the local flow, scaled for the V-22 by the length ratio (forces by its square,
        moments by its cube); flying backwards mirrored front to back (drag still opposes the motion); with the parasite
        drag the coefficients miss (DRAG_EXTRA, fitted) and the landing gear's drag."""
        g = s.g
        k2, k3 = U.FUS_SCALE ** 2, U.FUS_SCALE ** 3
        Vx, Vy, Vz = (p('velocities/u-aero-fps'), p('velocities/v-aero-fps'), p('velocities/w-aero-fps'))
        Vxz = g.fn('aero/fus/Vxz-fps', MAX(SQ(S(SQR(Vx), SQR(Vz))), v(0.01)))
        V = g.fn('aero/fus/V-fps', MAX(SQ(S(SQR(Vx), SQR(Vy), SQR(Vz))), v(0.01)))
        g.fn('aero/fus/alpha-deg', M(v(R2D), ATAN2(Vz, ABS(Vx))), 'angle of attack (front-back mirrored)')
        g.fn('aero/fus/beta-deg', M(v(R2D), ATAN2(Vy, Vxz)), 'sideslip')
        A_, Bb = 'aero/fus/alpha-deg', 'aero/fus/beta-deg'
        g.fn('aero/fus/beta-abs-deg', ABS(p(Bb)))
        kf = g.fn('aero/fus/kfwd', MAX(v(-1), MIN(v(1), Q(Vx, v(3.0)))))
        q = g.fn('aero/fus/qbar-psf', M(v(0.5), p('atmosphere/rho-slugs_ft3'), SQR(V)))
        ac = MAX(v(-20), MIN(v(20), p(A_)))
        bc = MAX(v(-20), MIN(v(20), p(Bb)))
        side_on = g.fn('aero/fus/side-on', T1('aero/fus/beta-abs-deg', [0, 50, 70, 90], [0, 0, 1, 1]))
        attached = D(v(1), side_on)
        Dq = g.fn('aero/fus/D-ft2', S(M(attached, v(k2), S(v(B.FUS_D0), M(v(B.FUS_DA2), SQR(ac)), M(v(B.FUS_DB), ABS(bc)))),
                                      M(side_on, v(k2 * B.FUS_D4)), v(U.DRAG_EXTRA)))
        Lq = g.fn('aero/fus/L-ft2', M(attached, kf, v(k2), S(v(B.FUS_L0), M(v(B.FUS_L1), ac))))
        Mq = g.fn('aero/fus/M-ft3', M(attached, kf, v(k3), S(T1(A_, B.FUS_MA_ALPHA, B.FUS_MA), M(v(B.FUS_M2), ABS(bc)))))
        Yq = g.fn('aero/fus/Y-ft2', M(attached, v(k2 * B.FUS_Y1), bc))
        Rq = g.fn('aero/fus/R-ft3', M(attached, kf, v(k3 * B.FUS_L1B), bc))
        Nq = g.fn('aero/fus/N-ft3', M(attached, v(k3 * B.FUS_N1), bc))
        # wind axes (drag along the flow, lift up, side force to the right) to body
        ca, sa = Q(Vx, Vxz), Q(Vz, Vxz)
        cb, sb = Q(Vxz, V), Q(Vy, V)
        c1 = (Q(Vx, V), Q(Vy, V), Q(Vz, V))
        c2 = (NEG(M(ca, sb)), cb, NEG(M(sa, sb)))
        c3 = (sa, v(0), NEG(ca))
        F = [g.fn('aero/fus/F%s-lbs' % a, M(q, S(NEG(M(Dq, c1[i])), M(Yq, c2[i]), M(Lq, c3[i])))) for i, a in enumerate('XYZ')]
        # moments: roll about the flow, pitch about the wind y, yaw about the wind z (Bell's l', M', N' signs: + right
        # wing down, nose up, nose right)
        e2 = (NEG(M(ca, sb)), cb, NEG(M(sa, sb)))
        e3 = (NEG(M(sa, cb)), NEG(sb), M(ca, cb))
        Mw = [g.fn('aero/fus/M%s-wind' % a, M(q, S(M(Rq, c1[i]), M(Mq, e2[i]), M(Nq, e3[i])))) for i, a in enumerate('LMN')]
        rx, ry, rz = s.arm_rp(U.FS_F, 0.0, U.WL_F)
        Mo = (S(Mw[0], D(M(ry, F[2]), M(rz, F[1]))), S(Mw[1], D(M(rz, F[0]), M(rx, F[2]))), S(Mw[2], D(M(rx, F[1]), M(ry, F[0]))))
        Mo = [g.fn('aero/fus/%s-ftlb' % a, e) for a, e in zip('LMN', Mo)]
        s.add(F, Mo)
        # landing gear, extended: drag areas at the wheels [CR B-35]
        gdn = p('gear/gear-pos-norm')
        rho = p('atmosphere/rho-slugs_ft3')
        for name, (fs, bl, wl), da in (('main', U.MAIN_GEAR, U.D_GEAR[0]), ('nose', U.NOSE_GEAR, U.D_GEAR[1])):
            Fg = [g.fn('aero/gear/%s/F%s-lbs' % (name, a), M(v(-0.5 * da), gdn, rho, Vc, V)) for a, Vc in zip('XYZ', (Vx, Vy, Vz))]
            gx, gy, gz = s.arm_rp(fs, 0.0, wl + 15)
            s.add(Fg, (D(M(gy, Fg[2]), M(gz, Fg[1])), D(M(gz, Fg[0]), M(gx, Fg[2])), D(M(gx, Fg[1]), M(gy, Fg[0]))))

    # ------------------------------------------------------------ empennage [CR B-24 to B-34]
    def surface_tables(s, xs, ys, cn90, kind):
        """a stabiliser's coefficient over +-180 deg: the data, blended over 10 deg into a flat plate's past them"""
        al = sorted(set(list(range(-180, 181, 10)) + [x for x in xs if -180 <= x <= 180]))
        out = []
        for a in al:
            r = math.radians(a)
            plate = cn90 * math.sin(r) * (math.cos(r) if kind == 'l' else math.sin(r))
            if kind == 'd':
                plate = max(plate, ys[len(ys) // 2])
            if xs[0] <= a <= xs[-1]:
                out.append(float(np.interp(a, xs, ys)))
            else:
                edge = xs[0] if a < xs[0] else xs[-1]
                w = min(1.0, abs(a - edge) / 10.0)
                out.append((1 - w) * float(np.interp(edge, xs, ys)) + w * plate)
        return al, out

    def tail_wake(s, g):
        """the rotors' wake at the stabiliser and the fins, Ferguson's [GTRS, data/gtrs.py], for the ring-vortex
        tables' at the tail: along the masts a share of the rotors' mean induced velocity against the fuselage's angle of
        attack, the true airspeed and the mast angle (an upwash between the two rotors), at the stabiliser times its
        sideslip factor, each reaching the tail through a lag of the hubs' distance ahead of it over the speed. Returns
        the air's velocity (to be subtracted) in body axes at the stabiliser and at the fins"""
        # (the tables are the XV-15's, against its airspeed: where the wake meets the tail goes with the wake's skew,
        # the airspeed over the rotors' induced velocity, so another aircraft reads them at the XV-15's airspeed for
        # its skew, its own times the ratio of the two's induced velocities in the hover at their design weights)
        vh = lambda gw, r: math.sqrt(gw / 2 / (2 * 0.0023769 * math.pi * r * r))
        kv = vh(GTRS.GW, B.R) / vh(U.GW_DESIGN, U.R)
        g.fn('aero/emp/wake-v-kts', M(v(kv), p('velocities/vtrue-kts')), 'the airspeed at which the XV-15 had this wake skew')
        share = g.fn('aero/emp/wake-share', T3('aero/alpha-deg', 'aero/emp/wake-v-kts', 'ctl/beta-m-deg', GTRS.RH_ALPHA, GTRS.RH_V,
                                               GTRS.RH_BETAM, [GTRS.RH[b] for b in GTRS.RH_BETAM]))
        kb = T2('aero/mag-beta-deg', 'ctl/beta-m-deg', GTRS.KHB_BETA, GTRS.RH_BETAM, GTRS.KHB)
        vi = M(v(0.5), S(p('aero/rotor/r/vi-fps'), p('aero/rotor/l/vi-fps')))
        sn, cn = p('ctl/sin-nac'), p('ctl/cos-nac')
        hub_x = D(v(U.FS_PIV), S(M(v(-U.DX_HUB), sn), M(v(U.DZ_HUB), cn)))       # the hubs' station
        out = []
        for name, fs, target in (('h', U.FS_H, M(share, kb, vi)), ('v', U.FS_V, M(share, vi))):
            st = 'aero/emp/%s/wake-fps' % name
            s.air.declare(st, 0.0)
            tau = Q(Q(D(v(fs), hub_x), v(12)), MAX(p('velocities/vt-fps'), v(10)))
            s.air.fn(st + '-next', S(p(st), M(D(target, p(st)), MIN(v(1), Q(p('simulation/channel-dt'), tau)))), out=st)
            # (along the mast, + down it in helicopter mode: aft and down in body axes)
            out.append((NEG(M(p(st), cn)), None, M(p(st), sn)))
        return out

    def empennage(s):
        """Horizontal stabiliser with its elevator and the two fins with their rudders: Bell's coefficients [CR] in
        each surface's own flow, with both rotors' wakes over it (Ferguson's, tail_wake()) and the wing's downwash at the
        stabiliser; the fins' sideslip times Bell's sidewash factor [Table VIV]. As revised against the XV-15's flight
        tests for NASA's generic tilt-rotor simulation [GTRS: CR-166536, data/gtrs.py]: the downwash, the elevator's and
        rudder's effectiveness (and their Mach number factor), the stabiliser's lift slope with the Mach number, its
        dynamic pressure loss, and the downwash's lag behind the wing. (Bell's 1973 tail, unrevised, made the XV-15
        twice as stiff in pitch for its elevator as Tischler's flight tests found, M_alpha / M_de 0.89 to 0.40, its
        short period 4.8 rad/s to 2.0 [TM-89428]; freqresp.py, derivs.py)"""
        g = s.g
        rho = p('atmosphere/rho-slugs_ft3')
        wake_h, wake_v = s.tail_wake(g)
        # wing downwash at the stabiliser, deg, against the wing's angle of attack, the flap index and Bell's mast angle
        # [GTRS tables 4-V]: its flap settings 0/0, 20/12.5, 40/25, 75/47 at the flap index 0, 0.5, 1, 2 (Bell's 0/0,
        # 40/25, 75/47; his reflexed -17.5 deg, index -1, has none: the 0/0's); and its slope against the wing's angle
        # of attack, for the downwash's lag behind the wing (below)
        fcols = [-1, 0, 0.5, 1, 2]
        fset = ['F1', 'F1', 'F2', 'F3', 'F4']
        rows = GTRS.WV_ALPHA
        eps = [[[GTRS.WV[bm][F][i] for F in fset] for i in range(len(rows))] for bm in GTRS.WV_BETAM]
        g.fn('aero/emp/eps-deg', T3('aero/wing/alpha-deg', 'aero/wing/flap-index', 'ctl/beta-m-deg', rows, fcols, GTRS.WV_BETAM, eps))
        deps = [[list(np.gradient([GTRS.WV[bm][F][i] for i in range(len(rows))], rows)) for F in fset] for bm in GTRS.WV_BETAM]
        deps = [[[deps[k][j][i] for j in range(len(fset))] for i in range(len(rows))] for k in range(len(GTRS.WV_BETAM))]
        deda = T3('aero/wing/alpha-deg', 'aero/wing/flap-index', 'ctl/beta-m-deg', rows, fcols, GTRS.WV_BETAM, deps)
        # stabiliser: lift against its angle of attack and elevator (Table HI outside +-12/8 deg, linear inside with
        # Table HII's slope and the elevator's effectiveness), drag Table HIV
        de_cols = B.HI_DE
        rowsH = sorted(set(list(range(-180, 181, 10)) + B.HI_ALPHA + [-8, -4, 0, 4]))
        CLH = []
        for a in rowsH:
            row = []
            for j, de in enumerate(de_cols):
                col = [B.HI_CL[i][j] for i in range(len(B.HI_ALPHA))]
                if -12 < a < 8:
                    val = B.CLH_ALPHA_LIN * (a - B.TAU_E * de)
                    lo = float(np.interp(-12, B.HI_ALPHA, col)); hi = float(np.interp(8, B.HI_ALPHA, col))
                    val = max(min(val, max(lo, hi) + 0.3), min(lo, hi) - 0.3)
                elif B.HI_ALPHA[0] <= a <= B.HI_ALPHA[-1]:
                    val = float(np.interp(a, B.HI_ALPHA, col))
                else:
                    r = math.radians(a)
                    edge = B.HI_ALPHA[0] if a < B.HI_ALPHA[0] else B.HI_ALPHA[-1]
                    w = min(1.0, abs(a - edge) / 10.0)
                    val = (1 - w) * float(np.interp(edge, B.HI_ALPHA, col)) + w * 1.1 * math.sin(r) * math.cos(r)
                row.append(val)
            CLH.append(row)
        # (the elevator's increment at the revised effectiveness [GTRS p. B-56], times its Mach number factor [table 5-IV];
        # the lift's slope with the Mach number [table 5-II])
        j0 = de_cols.index(0)
        CLH = [[r[j0] + GTRS.TAU_E / B.TAU_E * (x - r[j0]) for x in r] for r in CLH]
        g.fn('aero/emp/h/de-eff-deg', M(p('ctl/elevator-deg'), T1('velocities/mach', GTRS.KE_MACH, GTRS.KE)))
        alD, CDH = s.surface_tables(B.HIV_ALPHA, B.HIV_CD, 0.92, 'd')
        Vx, Vy, Vz = s.point_velocity(g, 'aero/emp/h', U.FS_H, 0.0, U.WL_H, wake_h)
        Vn = g.fn('aero/emp/h/V-fps', MAX(SQ(S(SQR(Vx), SQR(Vz))), v(0.01)))
        # (the downwash's lag behind the wing: it reaches the stabiliser from where the wing's angle of attack was its
        # arm over the speed before, (d eps / d alpha) (l / U) alpha' more angle of attack [GTRS p. A-87, the lift's
        # "wing/body damping", D_WB; its Mach number factor PCPM], the speed at least 35 ft/s)
        arm = Q(D(v(U.FS_H), p('inertia/cg-x-in')), v(12))
        lag = M(v(GTRS.D_WB * R2D), deda, Q(arm, MAX(Vn, v(35))), p('aero/alphadot-rad_sec'),
                T1('velocities/mach', GTRS.PCPM_MACH, [x / GTRS.PCPM[0] for x in GTRS.PCPM]))
        g.fn('aero/emp/h/alpha-deg', wrap180(S(D(S(M(v(R2D), ATAN2(Vz, Vx)), v(U.INC_H)), p('aero/emp/eps-deg')), lag)))
        cl = g.fn('aero/emp/h/cl', M(T2('aero/emp/h/alpha-deg', 'aero/emp/h/de-eff-deg', rowsH, de_cols, CLH),
                                     T1('velocities/mach', GTRS.CLH_MACH, [x / GTRS.CLH_SLOPE[0] for x in GTRS.CLH_SLOPE])))
        cd = g.fn('aero/emp/h/cd', T1('aero/emp/h/alpha-deg', alD, CDH))
        # the force normal to the local flow, which the downwash has turned by eps: lift along the turned normal
        e = M(v(1 / R2D), p('aero/emp/eps-deg'))
        ce, se = COS(e), SIN(e)
        ux, uz = S(M(ce, Vx), NEG(M(se, Vz))), S(M(se, Vx), M(ce, Vz))
        # (its dynamic pressure's loss [GTRS p. A-87, K_HNU]; the rotors' wakes over it are their own, rotorwake.py: not GTRS's
        # eta_H, which maps their rise with the rotors on)
        qs = g.fn('aero/emp/h/qS', M(v(0.5 * U.S_H * GTRS.K_HNU), rho, SQR(Vn)))
        Fx = g.fn('aero/emp/h/Fx-lbs', M(qs, Q(D(M(cl, uz), M(cd, ux)), Vn)))
        Fz = g.fn('aero/emp/h/Fz-lbs', M(qs, Q(NEG(S(M(cl, ux), M(cd, uz))), Vn)))
        rx, ry, rz = s.arm_rp(U.FS_H, 0.0, U.WL_H)
        s.add((Fx, None, Fz), (M(ry, Fz), D(M(rz, Fx), M(rx, Fz)), NEG(M(ry, Fx))))
        # fins: side force (Bell's "lift", + opposing the sideslip) against the fin's sideslip and the rudder
        rowsV = sorted(set(list(range(-180, 181, 10)) + B.VI_BETA))
        CYV = []
        for a in rowsV:
            row = []
            for j in range(len(B.VI_DR)):
                col = [B.VI_CY[i][j] for i in range(len(B.VI_BETA))]
                if abs(a) <= 90:
                    row.append(float(np.interp(a, B.VI_BETA, col)))
                else:
                    r = math.radians(a)
                    row.append(1.0 * math.sin(r) * math.cos(r) * (1 if abs(a) > 100 else (abs(a) - 90) / 10))
            CYV.append(row)
        # (the rudder's increment at the revised effectiveness [GTRS p. B-69], times its Mach number factor [table 5-IV])
        j0 = B.VI_DR.index(0)
        CYV = [[r[j0] + GTRS.TAU_R / B.TAU_R * (x - r[j0]) for x in r] for r in CYV]
        g.fn('aero/emp/dr-eff-deg', M(p('ctl/rudder-deg'), T1('velocities/mach', GTRS.KE_MACH, GTRS.KE)))
        alV, CDV = s.surface_tables(B.VIII_BETA, B.VIII_CD, 0.8, 'd')
        g.fn('aero/emp/sidewash', T3('aero/fus/alpha-deg', 'aero/fus/beta-abs-deg', 'ctl/beta-m-deg', B.SW_ALPHA, B.SW_BETA, B.SW_BETAM, B.SW))
        for side, m in SIDES:
            tg = 'aero/emp/fin%s' % side
            Vx, Vy, Vz = s.point_velocity(g, tg, U.FS_V, m * U.BL_V, U.WL_V, wake_v)
            Vn = g.fn(tg + '/V-fps', MAX(SQ(S(SQR(Vx), SQR(Vy))), v(0.01)))
            g.fn(tg + '/beta-deg', wrap180(M(p('aero/emp/sidewash'), v(R2D), ATAN2(Vy, Vx))))
            cy = g.fn(tg + '/cy', T2(tg + '/beta-deg', 'aero/emp/dr-eff-deg', rowsV, B.VI_DR, CYV))
            cd = g.fn(tg + '/cd', T1(tg + '/beta-deg', alV, CDV))
            qs = g.fn(tg + '/qS', M(v(0.5 * U.S_V), rho, SQR(Vn)))
            Fx = g.fn(tg + '/Fx-lbs', M(qs, Q(D(M(cy, Vy), M(cd, Vx)), Vn)))
            Fy = g.fn(tg + '/Fy-lbs', M(qs, Q(NEG(S(M(cy, Vx), M(cd, Vy))), Vn)))
            rx, ry, rz = s.arm_rp(U.FS_V, m * U.BL_V, U.WL_V)
            s.add((Fx, Fy, None), (NEG(M(rz, Fy)), M(rz, Fx), D(M(rx, Fy), M(ry, Fx))))

    def axes(s):
        def ax(name, terms):
            return '<axis name="%s">%s</axis>' % (name, ''.join('<function>%s</function>' % t for t in terms))
        return '\n'.join((ax('X', s.X), ax('Y', s.Y), ax('Z', s.Z), ax('ROLL', s.L), ax('PITCH', s.Mm), ax('YAW', s.N)))

    # ------------------------------------------------------------ nacelle masses
    def nacelle_masses(s, o):
        """the nacelles' point masses, each nacelle's two halves on its shaft, move with the nacelle angle (JSBSim
        recomputes the CG and the inertia: the halves turn the nacelle's own inertia with it)"""
        sn, cn = p('ctl/sin-nac'), p('ctl/cos-nac')
        for i, dz in nacelle_halves():
            fwd = S(M(v(-U.DX_NAC), sn), M(v(dz), cn))
            up = S(M(v(U.DX_NAC), cn), M(v(dz), sn))
            o.fn('mass/pm%d-x-in' % i, D(v(U.FS_PIV), fwd), out='inertia/pointmass-location-X-inches[%d]' % i)
            o.fn('mass/pm%d-z-in' % i, S(v(U.WL_PIV), up), out='inertia/pointmass-location-Z-inches[%d]' % i)

    # ------------------------------------------------------------ outputs for FlightGear
    def outputs(s):
        o = FCS('%s outputs' % U.NAME)
        o.channel('nacelle masses', execrate=SLOW_EXECRATE)
        s.nacelle_masses(o)
        o.channel('rotors', execrate=SLOW_EXECRATE)
        RPM = 60 / (2 * math.pi)
        W = p('rotor/omega-rad_sec')
        o.fn('out/rotor-rpm', M(v(RPM), W), out=['/rotors/main/rpm', '/rotors/tail/rpm'])
        o.fn('out/rotor-rpm-pct', M(v(100 / U.OMEGA_100), W), out='/rotors/main/rpm-pct')
        wrap = lambda e: IFTHEN(GT(e, v(360)), D(e, v(360)), e)
        for k in range(U.NB):
            o.fn('out/b%d-position' % k, wrap(S(M(v(R2D), p('rotor/psi-rad')), v(360.0 / U.NB * k))), out='/rotors/main/blade[%d]/position-deg' % k)
        o.fn('out/nacelle-deg', p('ctl/nacelle-deg'), out='/sim/model/v22/nacelle-deg')
        for side, m in SIDES:
            o.fn('out/%s/a1' % side, p('aero/rotor/%s/a1-deg' % side), out='/rotors/%s/a1-deg' % side)
            o.fn('out/%s/b1' % side, M(v(m), p('aero/rotor/%s/b1-deg' % side)), out='/rotors/%s/b1-deg' % side)
            o.fn('out/%s/coll' % side, p('ctl/%s/theta75-deg' % side), out='/rotors/%s/collective-deg' % side)
        # the 3-D model's rotors (both turn with blade 0's azimuth and tilt alike): each blade's pitch at 0.6 R, the
        # right rotor's, and the two gimbals' mean tilt (yaw-deg: forward, roll-deg: right side down)
        # (feathered to 90 deg as the wing stows: /sim/model/v22/wingfoldincidence, Nasal/v22.nas)
        o.declare('/sim/model/v22/wingfoldincidence', 0.0)
        h = p('/sim/model/v22/wingfoldincidence')
        for k in range(U.NB):
            th = S(p('aero/rotor/r/b%d/theta-deg' % k), v(twist(0.6 * U.R)))
            o.fn('out/b%d-pitch' % k, S(th, M(h, D(v(90), th))), out='/rotors/main/blade[%d]/incidence-deg' % k)
        o.fn('out/gimbal-fwd', M(v(0.5), S(p('aero/rotor/r/a1-deg'), p('aero/rotor/l/a1-deg'))), out='/rotors/main/yaw-deg')
        o.fn('out/gimbal-right', M(v(-0.5), S(p('aero/rotor/r/b1-deg'), M(v(-1), p('aero/rotor/l/b1-deg')))), out='/rotors/main/roll-deg')
        o.channel('gear', execrate=SLOW_EXECRATE)
        # the landing gear's retraction and extension, 10 s (estimate)
        o.raw('<actuator name="gear/gear-actuator"><input>gear/gear-cmd-norm</input><rate_limit>0.1</rate_limit>'
              '<output>gear/gear-pos-norm</output></actuator>')
        # the wheels' height above the ground (YASim's property, which the rotor wash places itself by), level
        o.fn('out/gear-agl', MAX(v(0), M(v(0.3048), D(p('position/h-agl-ft'), Q(D(p('inertia/cg-z-in'), v(U.MAIN_GEAR[2])), v(12))))),
             out='/position/gear-agl-m')
        o.channel('engine gauges', execrate=SLOW_EXECRATE)
        for n in range(2):
            g, e = 'eng[%d]' % n, '/engines/engine[%d]' % n
            o.fn('out/%s/n1' % g, p(g + '/n1-pct'), out=e + '/n1-pct')
            o.fn('out/%s/rpm' % g, M(v(75), p(g + '/n1-pct')), out=e + '/rpm')
            o.fn('out/%s/torque' % g, p(g + '/torque-pct'), out=e + '/torque-pct')
            o.fn('out/%s/ff' % g, p(g + '/fuel-flow-pph'), out=e + '/fuel-flow-pph')
            # (the multifunction display's fuel flow and the original gauges' converter; the XV-15's fuel taken as 6.7 lb/gal)
            o.fn('out/%s/ff-gph' % g, Q(p(g + '/fuel-flow-pph'), v(getattr(U, 'FUEL_LB_GAL', 6.7))), out=e + '/fuel-flow-gph')
            o.fn('out/%s/run' % g, M(p(g + '/running'), GT(p(g + '/spool'), v(0.5))), out=[e + '/running', e + '/on'])
        o.fn('out/engine-right', p('eng[0]/spool'), out='/sim/model/v22/engine_right')
        o.fn('out/engine-left', p('eng[1]/spool'), out='/sim/model/v22/engine_left')
        o.channel('surfaces', execrate=SLOW_EXECRATE)
        o.fn('out/elevator', Q(p('ctl/elevator-deg'), v(-20)), out='/surface-positions/elevator-pos-norm')
        o.fn('out/rudder', Q(p('ctl/rudder-deg'), v(20)), out='/surface-positions/rudder-pos-norm')
        o.fn('out/flaperon', Q(p('ctl/flaperon-deg'), v(67)), out='/surface-positions/flap-pos-norm')
        o.fn('out/aileron-l', Q(S(p('ctl/flaperon-deg'), p('ctl/aileron-deg')), v(67)), out='/surface-positions/left-aileron-pos-norm')
        o.fn('out/aileron-r', Q(D(p('ctl/flaperon-deg'), p('ctl/aileron-deg')), v(67)), out='/surface-positions/right-aileron-pos-norm')
        return o


# ---------------------------------------------------------------- mass, fuel and gear
def nacelle_own():
    """each nacelle as two halves of its weight on its shaft, either side of its CG: their spacing (in) carries its
    own inertia's difference across the shaft and about it, I_NACELLE[2] - I_NACELLE[0], which turns with the nacelle;
    the rest of its own inertia, in body axes (the same at any nacelle angle), stays in the empty weight's"""
    m = U.W_NACELLE / G0
    d = U.I_NACELLE[2] - U.I_NACELLE[0]
    return 12 * math.sqrt(d / m), (U.I_NACELLE[0], U.I_NACELLE[1] - d, U.I_NACELLE[0])


def nacelle_halves():
    """the nacelle halves' point mass indices and their heights above the nacelle's pivot in helicopter mode (in): the
    rotor ends first in point_masses, the aft ends last"""
    n = len(point_masses()[0])
    a = nacelle_own()[0]
    return ((0, U.DZ_NAC + a), (1, U.DZ_NAC + a), (n - 2, U.DZ_NAC - a), (n - 1, U.DZ_NAC - a))


def point_masses():
    """(name, lb, FS, BL, WL): the nacelles' rotor-end halves first, crew, payload, the nacelles' aft halves last
    (their locations move with the nacelle angle: nacelle_halves), and the fuel"""
    nx, nz = U.FS_PIV + U.DX_NAC, U.WL_PIV + U.DZ_NAC      # helicopter mode (DX_NAC aft +)
    a, h = nacelle_own()[0], U.W_NACELLE / 2
    pms = [('Right nacelle, rotor end', h, nx, U.BL_PIV, nz + a), ('Left nacelle, rotor end', h, nx, -U.BL_PIV, nz + a)]
    if U.NAME == 'XV-15':
        pms += [('Pilot', 200.0, U.EYE[0], U.EYE[1], U.EYE[2] - 25), ('Copilot', 200.0, U.EYE[0], -U.EYE[1], U.EYE[2] - 25),
                ('Research equipment and ballast', 1540.0, U.FS_CG, 0.0, U.WL_CG)]
        fuel = [('Fuel, right', 745.0, U.FS_CG, 60.0, 90.0), ('Fuel, left', 745.0, U.FS_CG, -60.0, 90.0)]
    else:
        fs_c = lambda xm: U.model_to_station(xm, 0, 0)[0]
        wl_floor = U.model_to_station(0, 0, -2.0)[2]
        pms += [('Pilot (right seat)', U.W_CREW, U.EYE[0], U.EYE[1], U.EYE[2] - 28),
                ('Copilot', U.W_CREW, U.EYE[0], -U.EYE[1], U.EYE[2] - 28),
                ('Crew chief', U.W_CREW, fs_c(-4.0), 0.0, wl_floor + 18),
                ('Troops and cargo', U.CARGO_REF, fs_c(-1.0), 0.0, wl_floor + 20)]
        # the tanks at the reference loading: JSBSim tanks (fdm_file)
        fuel = [(n, gal * U.FUEL_LB_GAL * r, x, y, z) for (n, gal, x, y, z), r in zip(U.TANKS, U.TANKS_REF)]
    # the test programme's weight adjustment, at the design CG: zero in the aircraft (performance.py sets it)
    pms.append(('Weight adjustment (tests)', 0.0, U.FS_CG, 0.0, U.WL_CG))
    pms += [('Right nacelle, aft end', h, nx, U.BL_PIV, nz - a), ('Left nacelle, aft end', h, nx, -U.BL_PIV, nz - a)]
    return pms, fuel


def mass_config(cg=None, weight=None, inertia=None):
    """Empty weight, its CG and inertias such that the reference loading (GW_DESIGN) is at the design CG with the
    reference inertias (helicopter mode). For a test configuration (cg, weight given) everything but the nacelles sits
    at the CG."""
    pms, fuel = point_masses()
    # reference inertia: the rest's own about the CG, and the nacelles' (their halves' parallel axes, and the rest of
    # their own)
    nac = pms[:2] + pms[-2:]
    I_ref = list(inertia or U.I_REST[:3])
    own = nacelle_own()[1]
    for n, w, x, y, z in nac:
        m_ = w / G0
        dx, dy, dz = (x - U.FS_CG) / 12, y / 12, (z - U.WL_CG) / 12
        I_ref[0] += m_ * (dy * dy + dz * dz) + own[0] / 2
        I_ref[1] += m_ * (dx * dx + dz * dz) + own[1] / 2
        I_ref[2] += m_ * (dx * dx + dy * dy) + own[2] / 2
    total = weight or U.GW_DESIGN
    cgx, cgz = (cg or (U.FS_CG, 0.0, U.WL_CG))[0], (cg or (U.FS_CG, 0.0, U.WL_CG))[2]
    if cg:
        pms = pms[:2] + [(n, w, cgx, 0.0, cgz) for n, w, x, y, z in pms[2:-2]] + pms[-2:]
        fuel = [(n, w, cgx, 0.0, cgz) for n, w, x, y, z in fuel]
    loads = [(w, x, y, z) for n, w, x, y, z in pms + fuel]
    we = total - sum(l[0] for l in loads)
    ex = (total * cgx - sum(w * x for w, x, y, z in loads)) / we
    ez = (total * cgz - sum(w * z for w, x, y, z in loads)) / we

    def para(w, x, y, z):
        m = w / G0; dx, dy, dz = (x - cgx) / 12, y / 12, (z - cgz) / 12
        return m * (dy * dy + dz * dz), m * (dx * dx + dz * dz), m * (dx * dx + dy * dy), -m * dx * dz
    terms = [para(*l) for l in loads] + [para(we, ex, 0.0, ez)]
    I = tuple(It - sum(t[i] for t in terms) for i, It in enumerate(I_ref + [0.0]))
    return dict(empty=we, cg=(ex, 0.0, ez), I=I, pms=pms, fuel=fuel)


def fdm_file(cg=None, weight=None, inertia=None):
    mc = mass_config(cg, weight, inertia)
    gear = []
    wt = weight or U.GW_DESIGN
    # tricycle gear: about 0.5 ft static deflection, 0.5 of critical damping (estimate)
    for name, (fs, bl, wl), share, steer in (('NOSE', U.NOSE_GEAR, 0.16, 70), ('LEFT_MAIN', (U.MAIN_GEAR[0], -U.MAIN_GEAR[1], U.MAIN_GEAR[2]), 0.42, 0),
                                             ('RIGHT_MAIN', U.MAIN_GEAR, 0.42, 0)):
        k = share * wt / 0.5
        c = 2 * 0.5 * math.sqrt(k * share * wt / G0)
        gear.append('''    <contact type="BOGEY" name="%s">
      <location unit="IN"><x>%.2f</x><y>%.2f</y><z>%.2f</z></location>
      <static_friction>0.8</static_friction><dynamic_friction>0.5</dynamic_friction><rolling_friction>0.02</rolling_friction>
      <spring_coeff unit="LBS/FT">%.0f</spring_coeff><damping_coeff unit="LBS/FT/SEC">%.0f</damping_coeff>
      <max_steer unit="DEG">%d</max_steer><brake_group>%s</brake_group><retractable>1</retractable>
    </contact>''' % (name, fs, bl, wl, k, c, steer, 'NONE' if steer else ('LEFT' if 'LEFT' in name else 'RIGHT')))
    if U.NAME == 'V-22':
        # FlightGear's start: no cargo (the payload dialog's slider: Systems/v22-base.xml), the tanks FUEL_START full;
        # the reference loading (the validation's: harness.load) is GW_DESIGN at the design CG
        pml = [(n, 0.0 if i == U.CARGO_PM else w, x, y, z) for i, (n, w, x, y, z) in enumerate(mc['pms'])]
        tanks = '\n'.join('''    <tank type="FUEL" number="%d">
      <!-- %s -->
      <location unit="IN"><x>%.1f</x><y>%.1f</y><z>%.1f</z></location>
      <capacity unit="LBS">%.1f</capacity>
      <contents unit="LBS">%.1f</contents>
      <type>JP-5</type>
    </tank>''' % (i, n, x, y, z, gal * U.FUEL_LB_GAL, U.FUEL_START * gal * U.FUEL_LB_GAL) for i, (n, gal, x, y, z) in enumerate(U.TANKS))
        propulsion = '  <propulsion>\n    <!-- the engines are the drive system\'s (Systems/%(pre)s-drive.xml), which draws the fuel -->\n' + tanks + '\n  </propulsion>\n'
    else:
        pml = [(n, w, x, y, z) for n, w, x, y, z in mc['pms']] + list(mc['fuel'])
        propulsion = ''
    pms = '\n'.join('''    <pointmass name="%s">
      <weight unit="LBS">%.1f</weight>
      <location unit="IN"><x>%.1f</x><y>%.1f</y><z>%.1f</z></location>
    </pointmass>''' % pm for pm in pml)
    I = mc['I']
    name = U.NAME
    return ('''<?xml version="1.0"?>
<!-- %s: generated by fdm/genv22.py -->
<fdm_config name="%s" version="2.0" release="ALPHA">
  <fileheader>
    <author>generated</author>
    <description>%s tiltrotor: gimballed proprotors blade by blade, from NASA's XV-15 data</description>
  </fileheader>
  <metrics>
    <wingarea unit="FT2">%.1f</wingarea>
    <wingspan unit="FT">%.2f</wingspan>
    <chord unit="FT">%.3f</chord>
    <location name="AERORP" unit="IN"><x>%.2f</x><y>0.0</y><z>%.2f</z></location>
    <location name="EYEPOINT" unit="IN"><x>%.1f</x><y>%.1f</y><z>%.1f</z></location>
    <location name="VRP" unit="IN"><x>%.2f</x><y>0.0</y><z>%.2f</z></location>
  </metrics>
  <mass_balance>
    <ixx unit="SLUG*FT2">%.0f</ixx><iyy unit="SLUG*FT2">%.0f</iyy><izz unit="SLUG*FT2">%.0f</izz><ixz unit="SLUG*FT2">%.0f</ixz>
    <emptywt unit="LBS">%.1f</emptywt>
    <location name="CG" unit="IN"><x>%.2f</x><y>%.2f</y><z>%.2f</z></location>
%s
  </mass_balance>
  <ground_reactions>
%s
  </ground_reactions>
%s  <system file="%%(pre)s-controls"/>
  <system file="%%(pre)s-rotors"/>
  <system file="%%(pre)s-airframe"/>
  <system file="%%(pre)s-drive"/>
  <system file="%%(pre)s-outputs"/>
  <aerodynamics file="Systems/%%(pre)s-aero.xml"/>
</fdm_config>
''' % ((name, name, name, U.S_W, U.B_W, U.C_W, U.FS_CG, U.WL_CG) + tuple(U.EYE) +
       ((U.X0_M * U.IN_PER_M, U.Z0_M * U.IN_PER_M) if U.NAME != 'XV-15' else (0.0, 0.0)) + tuple(I) + (mc['empty'],) +
       tuple(mc['cg']) + (pms, '\n'.join(gear), propulsion))).replace('%(pre)s', prefix())


def build(out, fg=False):
    import drive
    mdl = Model()
    mdl.controls()
    mdl.rotor_states()
    for side, m in SIDES:
        mdl.rotor(side)
    mdl.airframe()
    os.makedirs(os.path.join(out, 'Systems'), exist_ok=True)
    pre = prefix()
    texts = {pre + '-controls.xml': mdl.ctl.xml(), pre + '-rotors.xml': mdl.rot.xml(), pre + '-airframe.xml': mdl.air.xml(),
             pre + '-drive.xml': drive.build(SLOW_EXECRATE).xml(), pre + '-outputs.xml': mdl.outputs().xml(),
             pre + '-aero.xml': '<!-- generated by fdm/genv22.py -->\n<aerodynamics>\n%s\n%s\n</aerodynamics>\n' % (mdl.g.xml(), mdl.axes())}
    if INLINE:
        inline_single_use(texts, extra=[fdm_file()], keep=KEEP)
    if fg and U.NAME != 'XV-15':
        texts[pre + '-displays.xml'] = display_data()
    for name, t in texts.items():
        open(os.path.join(out, 'Systems', name), 'w').write('<?xml version="1.0"?>\n' + t)
    open(os.path.join(out, pre + '.xml'), 'w').write(fdm_file())
    return len(mdl.g.items)


def display_data():
    """what the cockpit's flight display (Nasal/displays.nas) draws from the model's data: the conversion
    protection's boundaries, and the power to hover with its ground effect; and AUTO NAC's schedule, for v22.nas's
    in-air start"""
    from data import v22_flight as VF
    (lv, ln), (uv, un) = U.CPS
    w0, h0 = VF.HOVER_CEILING[0]
    sk, sn = autonac_schedule()
    row = lambda tag, xs: ''.join('<%s>%g</%s>' % (tag, x, tag) for x in xs)
    return ('<!-- generated by fdm/genv22.py: the flight display\'s data (Nasal/displays.nas) -->\n<PropertyList>\n'
            '  <!-- conversion protection: equivalent airspeed (kt) and the least, the most nacelle angle (deg) -->\n'
            '  <cps>\n    <lower>%s%s</lower>\n    <upper>%s%s</upper>\n  </cps>\n'
            '  <!-- airplane mode\'s airspeed limit (KCAS, Mach) at 84 %% rotor speed, and at 100 %%; the high sink rate\n'
            '       warning\'s boundary: KCAS, ft/min -->\n'
            '  <apln><vmo-kt>%g</vmo-kt><mmo>%g</mmo><vmo-100-kt>%g</vmo-100-kt></apln>\n'
            '  <vrs>%s%s</vrs>\n'
            '  <!-- the power to hover out of ground effect at 100 %% rotor speed, standard sea level: the hover ceiling\'s\n'
            '       weight there, which the drive\'s torque limit is fitted to; the rotor\'s radius and the ground effect on\n'
            '       its induced velocity, (1 + k (R/Z)^2)^exp with Z the hub\'s height (above the main wheels\' contact\n'
            '       here, the nacelles up) -->\n'
            '  <hover>\n    <weight-lb>%g</weight-lb>\n    <power-hp>%g</power-hp>\n    <altitude-ft>%g</altitude-ft>\n'
            '    <radius-ft>%.3f</radius-ft>\n    <hub-height-ft>%.2f</hub-height-ft>\n    <ge-k>%g</ge-k>\n'
            '    <ge-exp>%g</ge-exp>\n  </hover>\n'
            '  <!-- AUTO NAC\'s schedule: the nacelle angle (deg) for the calibrated airspeed (kt). An in-air start\'s\n'
            '       nacelles from the flight model\'s first step; v22.nas draws the 3-D model\'s at it before that -->\n'
            '  <autonac>%s%s</autonac>\n</PropertyList>\n'
            % (row('kt', lv), row('nac', ln), row('kt', uv), row('nac', un), U.VMO_APLN, U.MMO_APLN, U.VMO_APLN_100,
               row('kt', U.VRS_SINK[0]), row('sink-fpm', U.VRS_SINK[1]), w0, U.P_XMSN_100, h0, U.R,
               (U.WL_PIV + U.DZ_HUB - U.MAIN_GEAR[2]) / 12, GE_K, GE_EXP, row('kt', sk), row('nac', sn)))


if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    out = args[0] if args else 'out'
    configure('--xv15' in sys.argv)
    if '--no-inline' in sys.argv:
        INLINE = False
    n = build(out, fg='--fg' in sys.argv)
    print('wrote', out, '-', U.NAME, '-', n, 'aerodynamic functions')
