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
wake per rotor, with ground effect, and in descent Johnson's vortex ring state: vrs.py). The nacelles tilt from 90 deg
(helicopter) to 0 (airplane); the rotor's frame and its hub turn with them. The rotors are geared together by the interconnect shaft: one rotor speed.

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
import vrs as VRS
import alerts

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
GEAR_S = 10.0             # the landing gear's retraction and extension, s (estimate)
SLAP_TAU_S = 0.25         # the slap's sound follows the rotors' state through this lag: their 3/rev loads (20 Hz) out
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
EO_SURPLUS_FT = 1000.0                  # ENG OUT's path: the margin left at the entry point (the flare's scatter short)
EO_FALL_KT = 140.0                         # (the fall to flying speed is over by here: capability.py's entries; under it the first turn is the speed's)
EO_STEER_FPS, EO_STEER_DEG = (25.0, 50.0, 100.0, 170.0, 270.0), (70.0, 30.0, 12.0, 6.0, 3.0)
from capability import EO_VG_W, EO_VG_KT, EO_TIP_FT, eo_tip_deg      # matrix of 192 flights each: 170 at 36 000 lb 155 intact against 133, 175 at 38 722 lb 140 against 139, 180 at 42 000 lb 125 against 137
EO_STOP_FPS = 3.0                       # stopped
EO_PED_TOP_FT = 300.0                   # the pedals take part under this, on the final
EO_TIP_SPAN_FT = 42.0                   # the rotor tip from the centreline: the hub's 23 ft and the 19 ft blade
EO_NB_SPAN_FT = 1500.0                  # the height to spare, in reach, it is fully out over
EO_TOUCH_FT = 300.0                     # ENG OUT's path: its two circles overlapping by this still a path, of no leg
KEEP = {'aero/rotor/r/thrust-lbs', 'aero/rotor/l/thrust-lbs', 'aero/rotor/r/torque-ftlb', 'aero/rotor/l/torque-ftlb',
        'aero/rotor/r/power-hp', 'aero/rotor/l/power-hp', 'aero/wing/download-lbs', 'aero/wing/lift-lbs',
        # read by Nasal/v22.nas and the gauges
        'drive/nr-pct', 'eng/power-available-hp', 'eng/xmsn-limit-hp', 'ctl/nacelle-cmd-limited-deg', 'ctl/cps-min-deg',
        'ctl/cps-max-deg', 'gov/theta-deg', 'hh/state', 'hh/e-ft', 'hh/h-ft', 'hh/vN-cmd', 'hh/vE-cmd',
        'ap/tcl-cmd', 'ap/nacelle-sched-deg', 'ap/auto-nac-on', 'ap/lat', 'ap/vert', 'ap/spd', 'ap/coupled', 'ap/nav', 'ap/line', 'ap/app',
        'ap/app-dist-ft', 'ap/app-xte-ft', 'ap/app-path-ft', 'ap/v-sel-kt', 'ap/roll-lag-s', 'ap/app-route',
        'ap/tf-on', 'ap/tf-flyup', 'ap/tf-climb-cap-fps', 'ap/tf-ref-ft', 'ap/tf-model-ft', 'ap/tf-v1-fps',
        'ap/tf-v2-fps', 'ap/tf-w', 'ap/tf-tau1-s', 'ap/tf-tau2-s', 'ap/tf-kh', 'ap/tf-desc-max-fps',
        # ENG OUT's cues (displays.nas) and deadland.py's traces
        'ap/eo-fly', 'ctl/flap-law-deg', 'ap/eo-theta-cmd-deg', 'ap/eo-phi-cmd-deg', 'ap/eo-nac-cmd-deg', 'ap/eo-lever-cmd', 'ap/eo-gate-ft',
        'ap/eo-pull-err-fps', 'ap/eo-hdot-err-fps', 'ap/eo-energy-ft', 'ap/eo-reach-nm', 'ap/eo-reach-s',
        # ENG OUT's landing site (eosite.nas, the displays, deadland.py site tests)
        'ap/eo-site-x-ft', 'ap/eo-site-y-ft', 'ap/eo-site-r-ft', 'ap/eo-site-final-ft', 'ap/eo-site-final-full-ft', 'ap/eo-site-arc-rad', 'ap/eo-site-margin-ft',
        'ap/eo-site-path-ft', 'ap/eo-site-mid-ft', 'ap/eo-nb-deg', 'ap/eo-plan-spend-ft', 'ap/eo-plan-hold-ft', 'ap/eo-glide-plan', 'ap/eo-plan-theta-deg', 'ap/eo-nb-ceil-deg', 'ap/eo-nb-floor-ft', 'ap/eo-nb-used', 'ap/eo-nb-after-kt', 'ap/eo-nb-ld', 'ap/eo-nb-lag-s', 'ap/eo-entry-ft', 'ap/eo-entry-loss-ft', 'ap/eo-entry-way-ft', 'ap/eo-site-fall', 'ap/eo-gear-ft', 'ap/eo-ld-req', 'ap/eo-xw-limit-kt', 'ap/eo-tgo-flare-s', 'ap/eo-decrab-w', 'ap/eo-tgo-s', 'ap/eo-ped-err-rad', 'ap/eo-ped-active', 'ctl/steer-deg', 'ap/eo-zone-short-ft', 'ap/eo-zone-past-ft', 'ap/eo-zone-side-ft', 'ap/eo-zone-rise', 'ap/eo-site-a1-now-rad', 'ap/eo-site-arc-left-rad', 'ap/eo-site-sturn-rad', 'ap/eo-site-trk-deg', 'ap/eo-phi-cmd-deg', 'ap/eo-site-turn-ft-rad', 'ap/eo-site-turn0-ft-rad', 'ap/eo-site-r0-ft', 'ap/eo-site-hac-bank-deg', 'ap/eo-bank-lim-deg', 'ap/eo-brake-nom-ratio', 'ap/eo-h-ft', 'ap/eo-tau-eff-s', 'ap/eo-gnd-rate-fps', 'ap/eo-ld-up',
        # read by descent.py (the others it reads are read twice in the model)
        'aero/rotor/r/vrs/nu-base', 'aero/rotor/l/vrs/nu-base', 'aero/rotor/r/vrs/dnu', 'aero/rotor/l/vrs/dnu'} | alerts.KEEP
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


# AUTO NAC's pitch in level flight against the calibrated airspeed, at most: its schedule's (autonac_schedule), and the
# flaperons' (flap_law), which give the lift at it
AUTONAC_PITCH = ([0.0, 150.0, 180.0], [4.0, 4.0, 7.0])


def autonac_schedule():
    """AUTO NAC's nacelle angle against the calibrated airspeed: at each, the most converted angle whose level-flight
    trim's pitch is at most AUTONAC_PITCH's (4 deg to 150 kt, rising to 7 at 180 kt), never higher at a higher speed
    (trimmap.py --autonac, at the design weight and CG)"""
    d = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'autonac_v22.json')))
    ns, ks, th = d['nacelle'], d['kcas'], d['theta']
    out = []
    for j, k in enumerate(ks):
        tmax = float(np.interp(k, *AUTONAC_PITCH))
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
    the height's tracking gain, 1/s; and the peak of a step down, over the step. Without it a 1 s lag and 0.2 (a loop 5
    s slow), no overshoot"""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'tf_lag_v22.json')
    if not os.path.exists(path):
        return [0.0, 300.0], [1.0, 1.0], [1.0, 1.0], [2.0, 2.0], [0.2, 0.2], [1.0, 1.0]
    d = json.load(open(path))
    return d['kcas'], d['w'], d['tau1_s'], d['tau2_s'], d['k_h'], d.get('peak_down', [1.0] * len(d['kcas']))


def start_trim():
    """the level-flight trims FlightGear's in-air start takes (trimmap.py --start): along AUTO NAC's schedule over the
    calibrated airspeed, the gross weight and the density altitude, the gear up and down; None without them"""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'start_trim_v22.json')
    return json.load(open(path)) if os.path.exists(path) else None


def glide_ld():
    """the airplane mode's glide ratio with the engines out over the calibrated airspeed and the gross weight, the gear
    up and down (deadstick.py --table: the model's zero-power trims, 1000 ft); None without it"""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'glide_ld_v22.json')
    return json.load(open(path)) if os.path.exists(path) else None


def glide_ld_flap():
    """the glide ratio at the glide's 200 KCAS by the gross weight and the flaperons held, the gear up and down
    (deadstick.py --flaptable); None without it"""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'glide_ld_flap_v22.json')
    return json.load(open(path)) if os.path.exists(path) else None


def eo_schedule():
    """ENG OUT's speeds and nacelle brake by the gross weight, from what the model does (capability.py); None without it"""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'eo_capability_schedule_v22.json')
    return json.load(open(path)) if os.path.exists(path) else None


def eo_site():
    """what ENG OUT's landing site must give and the planned turn's cost, over the gross weight (deadland.py site); None
    without it"""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'eo_site_v22.json')
    return json.load(open(path)) if os.path.exists(path) else None


def slap_table():
    """the blade slap's loudness over the rotor's advance ratio and inflow through the tip-path plane, outside and in
    the cockpit, relative to the sound bank's slap loops (rotorsound.py from PyWopWop's map); None without it"""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'slap_v22.json')
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
        c.declare('ap/eo-lon-in', 0.0)          # (ENG OUT's Touchdown task: eng_out)
        c.fn('ctl/lon-in', S(M(lon, v(B.X_LON), ff), M(sc, S(p('scas/lon-in'), p('scas/lon-trim-in'))), p('ap/eo-lon-in')),
             clip=('-%g' % B.X_LON, '%g' % B.X_LON))
        c.declare('ap/eo-ped-in', 0.0)          # (ENG OUT's own, low on the final and on the ground: eng_out_site)
        c.declare('ap/eo-lat-in', 0.0)
        c.fn('ctl/lat-in', S(M(lat, v(B.X_LAT), ff), M(sc, p('scas/lat-in')), p('ap/eo-lat-in')), clip=('-%g' % B.X_LAT, '%g' % B.X_LAT))
        c.fn('ctl/ped-in', S(M(ped, v(B.X_PED)), M(sc, p('scas/ped-in')), p('ap/eo-ped-in')), clip=('-%g' % B.X_PED, '%g' % B.X_PED))
        if U.NAME == 'V-22':
            # (the nose wheel through the pedals, its authority falling with the speed, as a pedal's on other aircraft: all 70 deg to taxi,
            # a few at the landing's. The V-22's own schedule isn't public: estimates)
            c.fn('ctl/steer-deg', M(Q(p('ctl/ped-in'), v(B.X_PED)), T1('velocities/vg-fps', EO_STEER_FPS, EO_STEER_DEG)), out='fcs/steer-pos-deg[0]', desc='the nose wheel, deg')
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
            c.declare('ctl/cps-free', 0)        # (ENG OUT's fall and glide: eng_out)
            (lv, ln), (uv, un) = U.CPS
            on = GT(M(p('ctl/cps-on'), D(v(1), p('ctl/cps-free'))), v(0.5))
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
        if U.NAME == 'V-22' and getattr(U, 'FLAP_LAW', True):
            # the V-22's: the hover's and the conversion's settings (data, above) where the nacelles are up and the speed
            # low, out by 160 kt and below 30 deg of nacelle; elsewhere the least deflection that makes the wing's lift
            # (flap_law), the most of the two
            fsch = MAX(T2(kts, 'ctl/nacelle-deg', [0, 60, 100, 120, 160], [0, 30, 60, 75, 90],
                          [[0, 0, fc, fh, fh], [0, 0, fc, fh, fh], [0, 0, fc, fc, fc], [0, 0, fc, fc, fc], [0, 0, 0, 0, 0]]),
                       s.flap_law())
        else:
            fsch = T2(kts, 'ctl/nacelle-deg', [0, 60, 100, 120, 160, 200], [0, 30, 60, 75, 90],
                      [[fc, fc, fc, fh, fh], [fc, fc, fc, fh, fh], [fc, fc, fc, fc, fc], [fc, fc, fc, fc, fc],
                       [0.5 * fc, 0.5 * fc, 0.5 * fc, 0.5 * fc, 0.5 * fc], [0, 0, 0, 0, 0]])
        c.declare('ctl/flaperon-override', 0)
        c.declare('ctl/flaperon-set-deg', 0.0)
        c.declare('ap/eo-flap-own', 0)          # (ENG OUT's on the wheels: the schedule holds them at 40 under 160 kt)
        c.declare('ap/eo-flap-deg', 0.0)
        c.fn('ctl/flaperon-cmd-deg', IFTHEN(GT(p('ctl/flaperon-override'), v(0.5)), p('ctl/flaperon-set-deg'), IFTHEN(GT(p('ap/eo-flap-own'), v(0.5)), p('ap/eo-flap-deg'), fsch)))
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
        # (its authority below the lever's collective a property, Bell's by default: deadland.py tries wider ones)
        c.declare('gov/lo-deg', lo)
        lo = p('gov/lo-deg')
        # it engages as the rotor comes up to speed (from 19 to 9 % below its reference: 85 to 95 % in helicopter mode),
        # its integral held at zero below: on a start the blades stay at the lever's flat pitch. Once fully engaged it
        # stays so (gov/latched) until the rotor has run down on the ground (a shutdown): in flight a rotor losing speed,
        # both engines out, has its collective taken down to hold it (autorotation), not left at the lever's pitch
        gr = MAX(v(0), MIN(v(1), Q(S(D(nr, ref), v(19)), v(10))))
        latched = c.declare('gov/latched', 0)
        c.fn('gov/latched-next', IFTHEN(GT(gr, v(0.999)), v(1), IFTHEN(GT(M(p('gear/wow'), LT(gr, v(0.001))), v(0.5)), v(0), latched)),
             out='gov/latched')
        ga = c.fn('gov/engaged', MAX(gr, latched))
        # (starting in the air, the start's collective less the lever's while it holds the airframe: controls())
        held, fixed = p('gov/int-deg'), init
        if s.start_at:
            fixed = MAX(init, GT(p('ctl/start-hold'), v(0.5)))
            held = IFTHEN(GT(p('ctl/start-hold'), v(0.5)), D(p('ctl/start-coll-deg'), p('ctl/coll-ff-deg')), held)
        c.fn('gov/int-next', IFTHEN(fixed, held, IFTHEN(LT(ga, v(0.01)), v(0),
                                                                MAX(lo, MIN(v(hi), S(p('gov/int-deg'), M(p('gov/ki'), ga, err, p('simulation/channel-dt'))))))),
             out='gov/int-deg')
        c.fn('gov/theta-deg', M(ga, MAX(lo, MIN(v(hi), S(p('gov/int-deg'), M(p('gov/kp'), err))))))

    def scas(s):
        """Stability and control augmentation, for the FlightGear aircraft (the validation runs the bare airframe,
        ctl/scas-on = 0). The V-22's fly-by-wire control laws are not public (it holds attitude in all modes [PD]), nor
        are the XV-15 SCAS's tables used: these are proportional-integral loops through the mixing, in inches of stick
        and pedal, with half the stick's travel and the pedals' full travel as their authority (the V-22 flies by wire). Rate damping in
        the three axes, for a rate time constant of about 0.5 s in the hover (the hover's control power per inch: pitch
        8.7, roll 8.1, yaw 4.8 deg/s^2, from the test programme's pulse responses), and attitude loops in pitch and roll
        of about 1.2 rad/s. A feed-forward of the level-flight trim's stick over the nacelle angle, the airspeed and the
        flaperons (data/trim_ff_v22.json, from trimmap.py at the design weight and CG) centres the stick's trim through the
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
        c.declare('scas/auth-aft-in', 0.5 * B.X_LON)        # (aft of it: ENG OUT's own pull-out, eng_out)
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
            # (by the flaperons too: indexed by the nacelle angle and the airspeed alone, at the trims' own flaperons, it
            # was a quarter of the stick's travel off in ENG OUT's flare, 40 deg where level flight has few, and the
            # augmentation's half could not hold the nose: it came up to 12 deg, the command at -2)
            tm = json.load(open(tf))
            ns, lon_t = list(tm['nacelle']), [[list(r) for r in layer] for layer in tm['lon']]
            if ns[-1] < U.NACELLE_MAX:
                f = (U.NACELLE_MAX - ns[-1]) / (ns[-1] - ns[-2])
                for layer in lon_t:
                    layer.append([b + (b - a) * f for a, b in zip(layer[-2], layer[-1])])
                ns.append(U.NACELLE_MAX)
            c.fn('scas/lon-trim-calc', M(v(B.X_LON), T3('ctl/nacelle-deg', 'scas/ff-kts', 'ctl/flaperon-deg', ns, tm['kcas'],
                                                       tm['flaperon'], lon_t)),
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
             clip=('-scas/auth-aft-in', 'scas/auth-in'), out='scas/lon-in')
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
        for n in ('cpld', 'hdg-hold', 'alt-hold', 'spd-hold', 'auto-nac', 'wypt', 'vor-ils', 'appr', 'hvr-hold', 'tf', 'ga', 'eng-out'):
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
        # the modes (the flight director) and the coupling; ENG OUT (eng_out) has the controls while it is on, the
        # others waiting
        free = M(air, NOT(hold), NOT(on(AF + 'eng-out')))
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
        # On the approach the reference stops itself over the point: the deceleration that brings its ground speed to
        # rest in the distance to go, v^2 / 2d, up to its own largest rate (the acceleration's). Chasing the profile's
        # speed at the profile's own deceleration instead fell behind for good: the jerk-limited start put the reference
        # a few knots slow, and from there the profile drops faster than its deceleration (by v_aircraft / v_profile),
        # so the gap grew, to 17 kt 250 ft out, and the hover hold took over at the point at 30 kt, ran 190 ft past it
        # and backed up (aptest.py approach). The stop law corrects itself: behind, it asks more
        vg_ref = MAX(v(0), D(vref, hw))
        rstop = c.fn('ap/v-ref-stop-kts', IFTHEN(GT(M(app, GT(dgo, v(1))), v(0.5)),
                                                 MIN(p('ap/accel-kts'), Q(M(SQR(vg_ref), v(KT)), M(v(2), MAX(dgo, v(1))))), v(0)),
                     desc='on the approach, the deceleration that stops the reference over the point, kt/s')
        rtgt = IFTHEN(GT(ev, v(0)), MIN(p('ap/accel-kts'), rmax), NEG(MAX(MIN(p('ap/decel-kts'), rmax), rstop)))
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
        # the conversion the nacelles, not 5 kt, change the climb (46 ft/s at 150 kt, 30 deg, 39 at 160, airplane mode); by
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
        lk, lw, lt1, lt2, lkh, lpd = tf_lag()
        # its descent no more than keeps the aircraft, overshooting as its step down does (tf_lag), out of PULL UP's
        # envelope at the clearance: there, over the ground, h = s PULLUP_S + s^2 / (2 g PULLUP_G) (alerts.py; the
        # alert's height the aircraft's, the clearance the wheels'). The 2000 ft/min it had was that envelope's at 200
        # ft: over the hills at 150 kt, the descent 11 % over it, PULL UP sounded for 1.9 s (the step down's peak 1.115
        # there); with 100 ft set the envelope is 19 ft/s
        kpu = 2 * G0 * alerts.PULLUP_G
        hpu = S(MAX(v(100), p(ST + 'target-agl-ft')), D(p('position/h-agl-ft'), p('hh/h-wheels-ft')))
        spu = M(v(kpu / 2), D(SQ(S(v(alerts.PULLUP_S ** 2), M(v(4 / kpu), hpu))), v(alerts.PULLUP_S)))
        desc = c.fn('ap/tf-desc-max-fps', Q(spu, MAX(v(1), T1('ap/tf-config-kt', lk, lpd))),
                    desc='TF\'s descent limit, ft/s: its overshoot inside PULL UP\'s envelope at the clearance (autopilot.nas plans within it)')
        w1 = c.fn('ap/tf-w', T1('ap/tf-config-kt', lk, lw), desc='the quick lag\'s share of the climb\'s response (tf_lag)')
        tau1 = c.fn('ap/tf-tau1-s', MAX(v(0.05), T1('ap/tf-config-kt', lk, lt1)), desc='the quick lag, s (tf_lag)')
        tau2 = c.fn('ap/tf-tau2-s', MAX(v(0.05), T1('ap/tf-config-kt', lk, lt2)), desc='the slow lag, s (tf_lag)')
        ttgt = IFTHEN(GT(flyup, v(0.5)), ccap, MAX(NEG(desc), p('ap/tf-hdot-fps')))
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
        # (ENG OUT's in place of the modes' while it flies, at its own rate)
        eo = s.eng_out(cpl)
        efly = eo['fly']
        vfly, vrise = MAX(vert, efly), MAX(vrise, eo['rise'])
        tc = IFTHEN(GT(efly, v(0.5)), eo['theta'], tc)
        tr, tmax = p('ap/theta-ref-rad'), M(IFTHEN(GT(efly, v(0.5)), p('ap/eo-theta-rate-dps'), p('ap/theta-rate-dps')), v(1 / R2D), dt)
        # (its rate, rad/s, for the augmentation's damper: the commanded rate isn't damped)
        c.fn('ap/theta-ref-rate', IFTHEN(GT(M(vfly, NOT(vrise)), v(0.5)), Q(MAX(NEG(tmax), MIN(tmax, D(tc, tr))), MAX(dt, v(1e-3))), v(0)))
        c.fn('ap/theta-ref-next', IFTHEN(GT(vfly, v(0.5)), IFTHEN(vrise, p('attitude/theta-rad'), S(tr, MAX(NEG(tmax), MIN(tmax, D(tc, tr))))),
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
        pc = IFTHEN(GT(efly, v(0.5)), eo['phi'], M(v(1 / R2D), MAX(NEG(bmax), MIN(bmax, bank))))
        lfly, lrise = MAX(lat, efly), MAX(lrise, eo['rise'])
        pr, pmax = p('ap/phi-ref-rad'), M(IFTHEN(GT(efly, v(0.5)), p('ap/eo-phi-rate-dps'), p('ap/phi-rate-dps')), v(1 / R2D), dt)
        c.fn('ap/phi-ref-rate', IFTHEN(GT(M(lfly, NOT(lrise)), v(0.5)), Q(MAX(NEG(pmax), MIN(pmax, D(pc, pr))), MAX(dt, v(1e-3))), v(0)))
        c.fn('ap/phi-ref-next', IFTHEN(GT(lfly, v(0.5)), IFTHEN(lrise, p('attitude/phi-rad'), S(pr, MAX(NEG(pmax), MIN(pmax, D(pc, pr))))),
             p('attitude/phi-rad')), out='ap/phi-ref-rad', desc='the flight director\'s bank')
        c.fn('ap/theta-on-next', MAX(M(vert, cpl), efly), out='ap/theta-on')
        c.fn('ap/phi-on-next', MAX(M(lat, cpl), efly), out='ap/phi-on')
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
        c.fn('ap/lever', IFTHEN(GT(eo['drive'], v(0.5)), eo['lever'], IFTHEN(GT(p('ap/app-state'), v(4.5)), MAX(v(0), D(p(thr), M(v(0.3), dt))),
                                IFTHEN(flying, tcmd, p(thr)))), out=[thr, 'ap/tcl-w'],
             desc='the lever, driven while coupled with a speed mode or ENG OUT; down after an approach\'s landing')
        # AUTO NAC: the nacelles for the speed reference, within the corridor; the pilot moving them takes it off
        ncmd = '/controls/flight/nacelle-cmd-deg'
        an = c.fn('ap/auto-nac-on', M(spd, MAX(on(AF + 'auto-nac'), app)))
        sk, sn = autonac_schedule()
        sched = c.fn('ap/nacelle-sched-deg', T1('ap/v-ref-kt', sk, sn))
        own = GT(ABS(D(p(ncmd), p('ap/nac-w'))), v(0.5))
        c.fn('ap/nac-dropped-next', IFTHEN(GT(M(an, own), v(0.5)), v(1), p('ap/nac-dropped')), out='ap/nac-dropped')
        c.fn('ap/auto-nac-next', IFTHEN(GT(M(an, own), v(0.5)), v(0), p(AF + 'auto-nac')), out=AF + 'auto-nac')
        c.fn('ap/nacelle', IFTHEN(GT(efly, v(0.5)), eo['nac'], IFTHEN(GT(M(an, NOT(own)), v(0.5)), MAX(p('ctl/cps-min-deg'), MIN(p('ctl/cps-max-deg'), sched)), p(ncmd))),
             out=[ncmd, 'ap/nac-w'], desc='the nacelle command, driven by AUTO NAC or ENG OUT')
        # the approach: 1 flying the line; 2 handing over to the hover hold below 35 kt over the ground within 2000 ft
        # of the point (its point, 50 ft on the radar height); 3 hovering there (HVR CPLD); 4 landing (the height's
        # target down at 2 ft/s), after APPR pressed again or on the way (ap/app-land), and the gear down and locked (it
        # will not land on the gear up: the hover held till it is down); 5 on the ground (the modes off,
        # the lever down, then APPR off). The hold refusing or let go by the pilot ends it (APPR off). APPR off: 0
        st, ak = p('ap/app-state'), on(AF + 'appr')
        wow = GT(S(*[p('gear/unit[%d]/WOW' % i) for i in range(3)]), v(0.5))
        is_ = lambda n: GT(M(GT(st, v(n - 0.5)), LT(st, v(n + 0.5))), v(0.5))
        to2 = GT(M(app, LT(gs, v(35)), LT(dgo, v(2000)), GT(p('ctl/nacelle-deg'), v(72))), v(0.5))
        hov = MAX(is_(3), is_(4))
        nst = c.fn('ap/app-state-new', IFTHEN(NOT(ak), v(0), IFTHEN(GT(M(is_(0), app), v(0.5)), v(1), IFTHEN(GT(M(is_(1), to2), v(0.5)), v(2),
                   IFTHEN(GT(M(is_(2), hold), v(0.5)), v(3), IFTHEN(GT(M(is_(2), GT(p('hh/dropped'), v(1.5))), v(0.5)), v(0),
                   IFTHEN(GT(M(hov, wow), v(0.5)), v(5), IFTHEN(GT(M(hov, NOT(hold)), v(0.5)), v(0),
                   IFTHEN(GT(M(is_(3), on('ap/app-land'), GT(p('hh/state'), v(1.5)), GT(p('gear/gear-pos-norm'), v(0.99))), v(0.5)), v(4),
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

    def eng_out(s, cpl):
        """ENG OUT: both engines out, the glide and the landing on the wheels as the model flies them best (deadland.py
        auto, its settings swept: deadland.py's docstring). A flight director mode: it comes on when both engines that
        ran stop in the air (or with its key), its commands on the FLT format for the crew to fly; coupled (CPLD) the
        augmentation flies them, as the other modes' (they wait meanwhile: off, the hover hold with them). The lever at
        flat pitch (the governor windmilling the rotors), the nacelles to airplane mode (the conversion protection lets
        them follow the speed). The pitch flies the glide's speed, ap/eo-vg-kt (the most energy for the flare: 180 did
        worse), the speed's error as an acceleration less the flight path's own (as the autopilot's back side), no lower
        than ap/eo-dive-deg and no higher than the wing's stall; the dive's descent limited to what a pull-out at
        ap/eo-pull-g stops by the flare's height. At ap/eo-flare-ft on the wheels' radar height (higher when sinking
        fast), the flare: the pitch on a descent rate closing on a float ap/eo-float-ft over the ground, its time
        constant fitted to the descent at the flare's start and closing to ap/eo-tau-s, never faster than there is, the
        ground's own rate added; the pitch no higher than ap/eo-tail-deg (the fuselage's underside meets the ground at
        TAIL_STRIKE_DEG, 7.3 deg, on the wheels), its attitude held to that (the attitude's error integrated, the limit
        led by the pitch rate); the nacelles commanded up from ap/eo-nac-start-ft to ap/eo-nac-flare-deg, the corridor's
        upper boundary letting them come as the speed falls (the proprotors clear the ground from 38.5 deg, level); the
        flaperons, its own law there, coming down from the flare's start at ap/eo-flap-rate-dps to ap/eo-flap-max-deg.
        Out of pitch (the attitude at its limit and sinking 2 ft/s past the command for 1 s), the settling at
        ap/eo-td-fps, the lever at flat pitch still. On the ground (coupled) the lever down, the attitude held and the
        nacelles coming up to ap/eo-nac-safe-deg, then the mode off. Its flare across the weight, the field's air, the
        wind and the ground's slope: deadland.py flare. The heading held from engaging (the bug's with HDG HOLD), within
        ap/eo-flare-bank-deg in the flare. The gear is the crew's: the mode line asks for it (autopilot.nas). Its
        authority the augmentation's own, half the stick: with the whole the dive was steeper and its pull-out came too
        late. (A nose raised as the rotor speed sagged changed nothing: none.) Phases (ap/eo-phase): 1 diving to the
        glide's speed, 2 gliding, 3 flaring, 4 settling, 5 on the ground."""
        c = s.ctl
        KT, AF = 1.6878, '/instrumentation/mv22b/afcs/'
        # (the glide's speed by the weight, EO_VG_KT: light, at 200 KCAS the glide is well past its best, 20 % of the range at 36 000 lb with the flaperons out.
        # From 42 000 lb 200 stays: at 47 500 lb 185 KCAS did worse, 10 of 48 low starts against 13. ap/eo-vg-set-kt: a test's own)
        sch0 = eo_schedule() if U.NAME == 'V-22' else None
        c.declare('ap/eo-vg-set-kt', 0.0)
        if sch0:
            c.fn('ap/eo-vg-kt', IFTHEN(GT(p('ap/eo-vg-set-kt'), v(1)), p('ap/eo-vg-set-kt'), T1('inertia/weight-lbs', EO_VG_W, EO_VG_KT)), desc='the glide\'s speed by the weight, KCAS')
        else:
            c.declare('ap/eo-vg-kt', 200.0)
        for n, x in (('kv', 0.25), ('kp', 0.8), ('ki', 0.25), ('a-max-g', 0.2), ('dive-deg', -20.0), ('pitch-max-deg', 15.0),
                     ('flare-ft', 250.0), ('float-ft', 10.0), ('tau-s', 3.0), ('kh', 0.3), ('khi', 0.15),
                     ('td-fps', 3.0), ('tail-deg', 6.5), ('nac-flare-deg', 45.0), ('gear-min-ft', 1000.0),
                     ('theta-rate-dps', 4.0), ('flare-bank-deg', 5.0), ('sink-ok-fps', 45.0), ('pull-g', 0.1), ('settle-ft', 300.0), ('kg', 1.0),
                     ('ka', 1.0), ('att-int-max-deg', 5.0), ('lead-s', 0.5), ('bank-deg', 55.0), ('phi-rate-dps', 15.0)):
            c.declare('ap/eo-' + n, x)
        for n in ('phase', 'was', 'fly-was', 'both-was', 'flown', 'int-deg', 'psi-rad', 'cap-s', 'vf-prev', 'vfdot-fps2', 'att-int-deg'):
            c.declare('ap/eo-' + n, 0.0)
        dt = p('simulation/channel-dt')
        NOT = lambda a: D(v(1), a)
        on = lambda prop: GT(p(prop), v(0.5))
        lim = lambda x, lo, hi: MAX(lo, MIN(hi, x))
        thr = '/controls/engines/engine[0]/throttle'
        air = p('scas/airborne')
        wow = GT(S(*[p('gear/unit[%d]/WOW' % i) for i in range(3)]), v(0.5))
        st = p('ap/eo-phase')
        is_ = lambda n: GT(M(GT(st, v(n - 0.5)), LT(st, v(n + 0.5))), v(0.5))
        # on as both engines that ran (the alerts' latches) stop in the air, once: the crew may turn it off; off on
        # the ground (the lever down after its landing, or never in the air)
        ran = lambda e: p('alert/eng%d-ran' % e) if U.NAME == 'V-22' else v(0)      # (the XV-15's build has no alerts)
        both = M(air, ran(0), ran(1), NOT(on('eng[0]/running')), NOT(on('eng[1]/running')))
        fail = GT(M(both, NOT(p('ap/eo-both-was'))), v(0.5))
        # (after a coupled landing it flies on, on the wheels (ap/eo-roll), until the nacelles are up to
        # ap/eo-nac-safe-deg, the proprotors clear of the ground with the nose down: on the ground it let go, the nose
        # came down at once and the nacelles stopped where the corridor had them at touchdown, 37 deg at 169 KEAS, the
        # tips 0.4 ft into the ground. Within a second the corridor lets them up. To 60 deg the tips cleared by 2.6-3.4
        # ft rolling on in the flares the corridor let them, against 1.6-1.8 with 45, the touchdowns the same; 75 no more)
        for n, x in (('roll', 1), ('nac-safe-deg', 60.0)):
            c.declare('ap/eo-' + n, x)
        # (flown coupled down to the ground, ap/eo-flown: CPLD itself drops on the wheels)
        # (and on until the aircraft stops: let go at the nacelles' 60 deg, nothing held it to the line for the rest of the run)
        roll = c.fn('ap/eo-rolling', M(on('ap/eo-roll'), on(AF + 'eng-out'), p('ctl/scas-on'), GT(st, v(4.5)), on('ap/eo-flown'),
                                        MAX(LT(p('ctl/nacelle-deg'), p('ap/eo-nac-safe-deg')), GT(p('velocities/vg-fps'), v(EO_STOP_FPS)))), desc='1: ENG OUT rolling out, until it stops')
        # (the brakes on as the planned run has them, from ap/eo-brake-s after the wheels, with the crew's own on top. Left on when it stops)
        for n, x in (('brake-s', 2.0),):
            c.declare('ap/eo-' + n, x)
        c.declare('ap/eo-wheels-s', 0.0)
        c.declare('/controls/gear/brake-parking', 0)
        c.fn('ap/eo-wheels-next', M(roll, wow, S(p('ap/eo-wheels-s'), p('simulation/channel-dt'))), out='ap/eo-wheels-s')
        brk = GT(p('ap/eo-wheels-s'), p('ap/eo-brake-s'))
        c.fn('ap/eo-brake-l', MAX(p('fcs/left-brake-cmd-norm'), brk), out='fcs/left-brake-cmd-norm')
        c.fn('ap/eo-brake-r', MAX(p('fcs/right-brake-cmd-norm'), brk), out='fcs/right-brake-cmd-norm')
        c.fn('ap/eo-park-next', MAX(p('/controls/gear/brake-parking'), M(GT(st, v(4.5)), on('ap/eo-flown'), on(AF + 'eng-out'), LT(p('velocities/vg-fps'), v(EO_STOP_FPS)))), out='/controls/gear/brake-parking')
        ended = MAX(GT(M(GT(st, v(4.5)), LT(p(thr), v(0.02)), NOT(roll)), v(0.5)), GT(M(wow, LT(st, v(0.5))), v(0.5)))
        c.fn('ap/eo-mode-next', IFTHEN(fail, v(1), IFTHEN(ended, v(0), p(AF + 'eng-out'))), out=AF + 'eng-out')
        eo = on(AF + 'eng-out')
        fly = c.fn('ap/eo-fly', M(eo, air, cpl), desc='1: ENG OUT flown (coupled)')
        eorise = GT(M(eo, NOT(p('ap/eo-was'))), v(0.5))
        frise = GT(M(fly, NOT(p('ap/eo-fly-was'))), v(0.5))
        for prop in [AF + n for n in ('alt-hold', 'spd-hold', 'auto-nac', 'appr', 'vor-ils', 'tf', 'hvr-hold', 'ga')] + ['hh/engage-cmd', 'ap/app-land']:
            c.fn('ap/eo-off-%s' % prop.split('/')[-1], IFTHEN(eorise, v(0), p(prop)), out=prop)
        # the speed: the forward airspeed (equivalent, scas/ff-kts) and its rate, smoothed over 0.4 s (as the autopilot's
        # flight path acceleration; the true airspeed's rate counted the fall from a hover as speed, and the nose went up)
        # the height flown: with a landing site, over its plane (eng_out_site: the ground the finder sampled along it
        # fitted as a line, its elevation at the flare's point and its slope along the landing, + rising) where the
        # ground under the wheels is no nearer than the plane by more than the float's height, the radar's there (a rise
        # the plane does not hold). Over Kansas's fields the ground under the flare rose 10 ft as it began, or fell away
        # 43 ft, and the flare flying the radar height ballooned to 8.6-8.7 deg and touched at 22-32 ft/s
        for n, x in (('site-plane', 0), ('site-elev-ft', 0.0), ('site-slope', 0.0), ('site-end-ft', 0.0), ('plane-below-ft', 15.0)):
            c.declare('ap/eo-' + n, x)
        hr = p('hh/h-wheels-ft')
        # (the aircraft on the site's line before the height: computed after it, the frame a site came in measured the
        # new plane from the old point, at 0,0, 37 million ft off; FlightGear's ENG OUT read -390 000 ft over Kansas and
        # flared at 5900 ft, the nacelles coming up)
        line = s.eng_out_site_line() if (U.NAME == 'V-22' and eo_site()) else None
        if line:
            # (before the flare's point the plane's elevation there, not its slope carried back along the approach: 3 NM
            # out a 1 % slope put the plane 180 ft off, the energy with it, and the flare began 1000 ft late)
            hpl = S(hr, D(p('position/terrain-elevation-asl-ft'), S(p('ap/eo-site-elev-ft'), M(p('ap/eo-site-slope'), MAX(v(0), line[0])))))
            # (the plane only on the site's final and short of its strip's end (ap/eo-site-end-ft past the point), where
            # it is the ground the finder sampled: carried 25 000 ft past the point a 1 % slope put it 270 ft off; and
            # below the radar's by no more than ap/eo-plane-below-ft, the float's and half again: the finder's strip is
            # level within the float of its line where sampled, and Kansas's grounds under the flare lay no more than
            # 11 ft under theirs. A plane wrong by thousands of feet, offline, then landed at 4 ft/s (7 with 10 ft, 12
            # with 20, 19 with 30): a site changed, the scenery's elevation missing, cannot fly the flare)
            planed = GT(M(on('ap/eo-site-on'), on('ap/eo-site-plane'), GT(p('ap/eo-site-state'), v(2.5)),
                          LT(line[0], p('ap/eo-site-end-ft'))), v(0.5))
            h_ = c.fn('ap/eo-h-ft', IFTHEN(planed, MAX(D(hr, p('ap/eo-plane-below-ft')), MIN(hpl, S(hr, p('ap/eo-float-ft')))), hr),
                      desc='ENG OUT\'s height: over the landing site\'s plane on its final, or the wheels\' radar height, ft')
        else:
            planed = v(0)
            h_ = c.fn('ap/eo-h-ft', hr, desc='ENG OUT\'s height: the wheels\' radar height, ft')
        # the ground's rate under the aircraft, ft/s (+ rising): over the site's plane its slope at the speed along its
        # line, past its point; else none known (the radar's ground's own rate, smoothed over 1-6 s, chased each rise and
        # dip of an uneven field: Kansas's second, 4.7 ft/s without it, 9.6-14.7 with)
        c.declare('ap/eo-gnd-follow', 1)
        if line:
            cc_ = line[2]
            along = S(M(p('velocities/v-north-fps'), COS(cc_)), M(p('velocities/v-east-fps'), SIN(cc_)))
            gr = M(planed, p('ap/eo-site-slope'), along, GT(line[0], v(0)))
        else:
            gr = v(0)
        grate = c.fn('ap/eo-gnd-rate-fps', gr, desc='the ground\'s rate under the aircraft, ft/s')
        vc, h, vg = p('scas/ff-kts'), h_, p('ap/eo-vg-kt')
        vfd = p('ap/eo-vfdot-fps2')
        c.fn('ap/eo-vfdot-next', S(vfd, M(D(Q(M(D(vc, p('ap/eo-vf-prev')), v(KT)), MAX(dt, v(1e-3))), vfd), MIN(v(1), Q(dt, v(0.4))))),
             out='ap/eo-vfdot-fps2')
        c.fn('ap/eo-vf-prev-next', vc, out='ap/eo-vf-prev')
        # the phase. The flare from its height, or higher the faster the descent: where the flare's path (closing in tau)
        # meets it, with the time the nose takes to come up to the tail's limit at the pitch's rate
        th = p('attitude/theta-deg')
        sink = MAX(v(0), NEG(p('velocities/h-dot-fps')))
        # the gear asked for at its height, or sooner the faster the descent: the time it takes. At 1000 ft whatever the descent, from a
        # hover at 4100 ft over the field 9 flights of 16 at 47 500 lb met the ground with it still coming down
        c.fn('ap/eo-gear-ft', MAX(p('ap/eo-gear-min-ft'), M(sink, v(GEAR_S))), desc='the height ENG OUT asks for the gear, ft')
        # (up to twice its height: below it the dive's pull-out limits the descent, the pitch)
        # (the handover to the Flare task: no early flare with the wings still banked. The turn's descent read as a fast one, the gate went up to 500 ft in 50 deg of bank onto the
        # line, and the flare took the aircraft mid-turn. The turn ends first, down to the flare's own height)
        c.declare('ap/eo-gate-bank-deg', 15.0)
        c.declare('ap/eo-plan', 1)
        gate_h = MIN(M(v(2), p('ap/eo-flare-ft')), MAX(p('ap/eo-flare-ft'), S(p('ap/eo-float-ft'), M(sink, S(p('ap/eo-tau-s'),
                     Q(MAX(v(0), D(p('ap/eo-tail-deg'), th)), p('ap/eo-theta-rate-dps')))))))
        gate = c.fn('ap/eo-gate-ft', IFTHEN(GT(M(on('ap/eo-plan'), GT(ABS(p('attitude/phi-deg')), p('ap/eo-gate-bank-deg'))), v(0.5)), p('ap/eo-flare-ft'), gate_h), desc='the flare\'s height on the wheels, ft')
        nst = c.fn('ap/eo-phase-new', IFTHEN(NOT(eo), v(0), IFTHEN(GT(M(wow, GT(st, v(0.5))), v(0.5)), v(5), IFTHEN(GT(st, v(4.5)), v(5),
                   IFTHEN(GT(M(LT(st, v(2.5)), LT(h, gate)), v(0.5)), v(3),
                   # (a flare begun over a hill's crest on the way, the ground gone from under it: back to the glide. Held on to, the
                   # nacelles and flaperons out and 5 deg of bank, it hung at 130 KCAS and fell in at 106 ft/s, 7000 ft short)
                   IFTHEN(GT(M(is_(3), GT(h, M(v(2.5), p('ap/eo-flare-ft')))), v(0.5)), v(2),
                   IFTHEN(GT(M(is_(3), GT(p('ap/eo-cap-s'), v(1.0))), v(0.5)), v(4),
                   IFTHEN(GT(M(LT(st, v(2.5)), GT(vc, D(vg, v(10)))), v(0.5)), v(2), IFTHEN(LT(st, v(0.5)), v(1), st)))))))))
        to3 = GT(MAX(M(GT(nst, v(2.5)), LT(st, v(2.5))), M(LT(nst, v(2.5)), GT(st, v(2.5)))), v(0.5))
        flare = GT(st, v(2.5))
        # the pitch: its integral from the attitude on engaging, coupling and flaring
        a_max = M(p('ap/eo-a-max-g'), v(G0))
        # (turning while still diving for the glide's speed, the speed held and the dive left for after the turn: the height
        # kept, and the turn tighter, its radius as the speed's square. Over LOWI at 160 KCAS the nose went to -15 deg with
        # the Nordkette 1 NM ahead, 640 ft spent on speed before the turn away had begun)
        # (the first turn alone, no longer than a half turn takes: held through every later bank, 154 KCAS all the way down, the
        # glide's ratio 6 for 8, the flight that turned away from the Nordkette came down short of everything)
        c.declare('ap/eo-turn-first-deg', 15.0)
        c.declare('ap/eo-turn-first-max-s', 22.0)
        c.declare('ap/eo-turn-first-s', 0.0)
        c.declare('ap/eo-phi-cmd-deg', 0.0)
        # (not with height to spare: held so the glide went at 157 KCAS, its ratio with the brake out 5.3 for 4.5 at the speed)
        c.declare('ap/eo-site-margin-ft', 0.0)
        hold = c.fn('ap/eo-turn-first', GT(M(is_(1), GT(MAX(ABS(p('attitude/phi-deg')), ABS(p('ap/eo-phi-cmd-deg'))), p('ap/eo-turn-first-deg')),
                                             LT(p('ap/eo-turn-first-s'), p('ap/eo-turn-first-max-s')), LT(p('ap/eo-site-margin-ft'), v(EO_SURPLUS_FT))), v(0.5)), desc='turning first, the dive held')
        c.fn('ap/eo-turn-first-next', IFTHEN(is_(1), S(p('ap/eo-turn-first-s'), M(hold, p('simulation/channel-dt'))), v(0)), out='ap/eo-turn-first-s')
        # (after the nacelle brake not back to the glide's speed but to where the flare can start from: back to 200 KCAS it dived
        # for the speed, and with it to the ground before the float was done, 170-190 KCAS, the corridor holding the nacelles
        # low and the tips in the ground. From 170 the flare is 1500 ft shorter, the touchdown 140-150 KCAS)
        c.declare('ap/eo-nb-deg', 0.0)
        c.declare('ap/eo-nb-used', 0)
        # the Glide task, ap/eo-plan: one plan from the end of the entry to the flare's gate, each control one owner, as a glider flies an approach: the pitch the
        # glide's one speed, the drag the path. No speed of the brake's own and none to dive back to after it, and no pull-out law: the nacelles' way in is the plan's.
        # ap/eo-in-ft over the gate they are in and the flaperons alone tune what is left; ap/eo-close-dps how fast the plan takes them in, so the height that takes
        # is planned, ap/eo-plan-a-g the most acceleration the speed may ask of the pitch: it holds the speed, it doesn't dive for it
        for n, x in (('in-ft', 300.0), ('close-dps', 5.6), ('plan-a-g', 0.06), ('flap-out-deg', 20.0), ('plan-kg', 0.5), ('plan-g-max-deg', 6.0)):
            c.declare('ap/eo-' + n, x)
        pl_on = on('ap/eo-plan')
        pl = c.fn('ap/eo-glide-plan', M(pl_on, is_(2), on('ap/eo-site-on'), GT(p('ap/eo-site-state'), v(0.5))), desc='1: the Glide task flying its plan')
        c.declare('ap/eo-site-fall', 1.0)         # (no brake in the fall to flying speed: held at 45 deg from a hover the nose went to +41)
        # (the speeds and the nacelles by the weight, capability.py: the flare wants 170 KCAS at 36 000 lb and 200 from 47 500. With
        # ap/eo-nb-test-deg, deadland.py capturns, the glide's speed held and the nacelles there)
        c.declare('ap/eo-nb-test-deg', -1.0)
        sch = eo_schedule() if U.NAME == 'V-22' else None
        testing = GT(p('ap/eo-nb-test-deg'), v(-0.5))
        by_w = lambda n, x, d: c.fn('ap/eo-' + n, IFTHEN(testing, x, T1('inertia/weight-lbs', sch['weight_lb'], sch[d])), desc=d + ' by the weight (capability.py)') if sch else None
        if sch:
            c.fn('ap/eo-nb-after-kt', IFTHEN(MAX(testing, pl_on), vg, T1('inertia/weight-lbs', sch['weight_lb'], sch['after_kt'])), desc='the speed after the brake (eosite.nas): the glide\'s own')
            c.fn('ap/eo-nb-kt', IFTHEN(MAX(testing, pl_on), vg, T1('inertia/weight-lbs', sch['weight_lb'], sch['nb_kt'])), desc='the speed on the brake: the glide\'s own')
            c.fn('ap/eo-nb-nac-deg', IFTHEN(testing, p('ap/eo-nb-test-deg'), IFTHEN(pl_on, T1('inertia/weight-lbs', sch['gl_weight_lb'], sch['gl_nac_max']), T1('inertia/weight-lbs', sch['weight_lb'], sch['nb_deg']))), desc='the nacelles at their most on the brake, deg')
            c.fn('ap/eo-nb-ld', T1('inertia/weight-lbs', sch['weight_lb'], sch['nb_ld']), desc='the nacelle brake\'s glide ratio by the weight (eosite.nas)')
            c.fn('ap/eo-nb-lag-s', Q(p('ap/eo-nb-nac-deg'), v(U.NACELLE_RATE)), desc='the nacelles\' time up to the brake, s (eosite.nas: nothing yet spent)')
            c.fn('ap/eo-xw-limit-kt', T1('inertia/weight-lbs', sch['xw_weight_lb'], sch['xw_limit_kt']), desc='the crosswind full pedal takes out, kt (eosite.nas)')
            c.fn('ap/eo-ped-beta-rad-in', T1('inertia/weight-lbs', sch['xw_weight_lb'], [x_ / R2D / B.X_PED for x_ in sch['xw_beta_deg']]), desc='the sideslip an inch of pedal holds at the touchdown\'s speed, rad')
            for n, k_ in (('short-ft', 'zone_short_ft'), ('past-ft', 'zone_past_ft'), ('side-ft', 'zone_side_ft'), ('rise', 'zone_rise')):
                c.fn('ap/eo-zone-' + n, T1('inertia/weight-lbs', sch['zone_weight_lb'], sch[k_]), desc='the ground a strip keeps clear by the weight (eosite.nas zone_check)')
        else:
            for n, x in (('nb-after-kt', 200.0), ('nb-kt', 200.0), ('nb-nac-deg', 1.0), ('nb-ld', 1.0)):
                c.declare('ap/eo-' + n, x)
        vg2 = IFTHEN(GT(p('ap/eo-nb-used'), v(0.5)), p('ap/eo-nb-after-kt'), vg)
        vb = D(vg2, M(MIN(v(1), Q(p('ap/eo-nb-deg'), p('ap/eo-nb-nac-deg'))), D(vg2, p('ap/eo-nb-kt'))))
        vgc = c.fn('ap/eo-vg-cmd-kt', IFTHEN(hold, MIN(vg, vc), vb), desc='the speed ENG OUT\'s pitch flies, kt')
        a_lim = IFTHEN(pl, M(p('ap/eo-plan-a-g'), v(G0)), a_max)
        # (faster than the plan the nose comes up as freely as ever; slower it doesn't dive for the speed)
        ea = D(lim(M(p('ap/eo-kv'), D(vgc, vc), v(KT)), NEG(a_max), a_lim), vfd)
        dive, pmax, tail = p('ap/eo-dive-deg'), p('ap/eo-pitch-max-deg'), p('ap/eo-tail-deg')
        # (the flare's path never asks a faster descent than there is: at its height the path's was 60 ft/s, the
        # glide's 44, and the nose went down first; held to the descent the flare began with, a descent easing for a
        # moment out of a dive's pull-out took the nose down to -5 deg at 200 ft. Slower than its path, the attitude held)
        # (ap/eo-tau-blend: the path's time constant fitted at the flare's start to the descent there, the height over
        # the float over the descent, within ap/eo-tau-s..ap/eo-tau-max-s, and closing to ap/eo-tau-s as the float comes:
        # the round-out begun at once, gently. Held at tau-s from the start, the path asked for less descent than there
        # was until it met it, the attitude was held meanwhile, and where the flare began low and fast (the dive guard's
        # swing at its bottom, as FlightGear's Kansas landing: -3.3 deg, 46 ft/s) the pull came at 150 ft at 0.5 g and
        # ballooned. deadland.py flare, the dive guard's swing taken out (sink-ok-fps 50): 6 balloons in 18 flares with
        # it, none without)
        for n, x in (('tau-blend', 1), ('tau-max-s', 8.0), ('flare-u-ft', 1.0), ('flare-tau-s', 3.0)):
            c.declare('ap/eo-' + n, x)
        fu = MAX(v(0), D(h, p('ap/eo-float-ft')))
        # (ap/eo-gnd-follow: where the site's ground falls away, the path and the settling over the ground, not in still
        # space: the descent towards it the aircraft's less the ground's fall, the command's the ground's fall added. Over
        # ground falling 1-2 % the float ran on past the energy. Not where it rises: the rise shortens the float itself,
        # and climbing with it the light aircraft held off and touched at 15-25 ft/s against the ground, 125-136 KCAS.
        # The fall added to a command against still space asked at once for more descent than there was: on 2 % down at
        # 2200 ft the nose went to -4.5 deg, and it touched at 22.5 ft/s)
        gfall = M(on('ap/eo-gnd-follow'), MIN(v(0), grate))
        sink_g = MAX(v(0), S(NEG(p('velocities/h-dot-fps')), gfall))
        c.fn('ap/eo-flare-u-next', IFTHEN(to3, MAX(v(1), fu), p('ap/eo-flare-u-ft')), out='ap/eo-flare-u-ft')
        c.fn('ap/eo-flare-tau-next', IFTHEN(to3, lim(Q(fu, MAX(sink_g, v(1))), p('ap/eo-tau-s'), p('ap/eo-tau-max-s')), p('ap/eo-flare-tau-s')),
             out='ap/eo-flare-tau-s')
        tau_ = IFTHEN(on('ap/eo-tau-blend'), S(p('ap/eo-tau-s'), M(D(p('ap/eo-flare-tau-s'), p('ap/eo-tau-s')),
                                                                   MIN(v(1), Q(fu, p('ap/eo-flare-u-ft'))))), p('ap/eo-tau-s'))
        tau_e = c.fn('ap/eo-tau-eff-s', tau_, desc='the flare\'s path\'s time constant here, s')
        # (the path's descent asked for no further than ap/eo-lead-fps past the descent there is: at its start not the
        # nose down; sinking slower than it, the path taken up again gently, not the attitude held while the speed bleeds.
        # Over sites on falling and uneven ground (deadland.py terrain at 38 700 and 47 500 lb, sea level and 2200 ft): 3
        # ft/s took Kansas's second from 5.9 to 3.7 ft/s and 2 % down from 7.5 to 6.6; 6 no better; 0 the held attitude)
        c.declare('ap/eo-lead-fps', 3.0)
        # (a constant deceleration's path, sqrt(td^2 + 2 a (h - float)), led by its turn a / V, did worse: the descent's
        # integral lagged it 3-5 ft/s and the flares touched at 9.7-16 ft/s, median)
        path_ = MAX(v(0), Q(fu, tau_e))
        hc = S(IFTHEN(GT(st, v(3.5)), NEG(p('ap/eo-td-fps')), NEG(MIN(S(sink_g, p('ap/eo-lead-fps')), path_))), gfall)
        eh = c.fn('ap/eo-hdot-err-fps', D(hc, p('velocities/h-dot-fps')), desc='the flare\'s descent rate command less the aircraft\'s')
        # (diving, the descent limited towards the flare: no faster than the rate a pull-out at ap/eo-pull-g brings to the
        # glide's, ap/eo-sink-ok-fps, ap/eo-settle-ft over the flare's height, the glide settling before it; the nose
        # raised in proportion to the excess, the speed's integral with it. Flared on the descent's rate alone, a dive at
        # 150-200 ft/s from a hover started the flare at 1100-3100 ft and settled there; pulled out by the flare's
        # height, the glide still swung there, 46-54 ft/s)
        # (off the nacelle brake, harder and later: its way down steep to the last, height to lose with a mile to go. At 0.1 g the
        # limit held it to 45-60 ft/s from 1000 ft, 20 s and 4000 ft past the point; at 0.25 g from 100 ft over the flare, 9-9.5
        # ft/s on the wheels and 1000 ft nearer)
        c.declare('ap/eo-nb-pull-g', 0.25)
        c.declare('ap/eo-nb-settle-ft', 100.0)
        nbu = GT(p('ap/eo-nb-used'), v(0.5))
        pullg, settle = IFTHEN(nbu, p('ap/eo-nb-pull-g'), p('ap/eo-pull-g')), IFTHEN(nbu, p('ap/eo-nb-settle-ft'), p('ap/eo-settle-ft'))
        hg = NEG(SQ(S(SQR(p('ap/eo-sink-ok-fps')), M(v(2), pullg, v(G0), MAX(v(0), D(h, S(p('ap/eo-flare-ft'), settle)))))))
        eg = c.fn('ap/eo-pull-err-fps', D(hg, p('velocities/h-dot-fps')), desc='the dive\'s descent past its limit, ft/s')
        # (the pitch that turns the flight path to the limit's at the present angle of attack: the attitude plus the
        # path's change, rad. Its gain 0.3 deg per ft/s, 1.7 times that, overshot: out of a hover the nose came up to
        # +14 deg, the aircraft climbed and slowed, and the speed's dive took it to -18 deg again)
        V_ = MAX(p('velocities/vt-fps'), v(100))
        # (ap/eo-guard-td-s: the limit's excess damped by the climb's acceleration, so it asks the descent to come to the
        # limit at excess / td-s: the attitude plus the path's change alone kept rising while the path lagged the nose,
        # pulled 1.37 g to 30 ft/s, pushed 0.75 g back, the pitch swinging -5..+2 deg into the flare (the jerky glide seen
        # in FlightGear); damped over 2 s the glide holds -2.9 deg at 0.99-1.06 g and the flare begins at 245 ft, 45 ft/s;
        # 0 undamped)
        c.declare('ap/eo-guard-td-s', 2.0)
        hdd = M(v(G0), D(M(p('accelerations/Nz'), COS(p('attitude/theta-rad')), COS(p('attitude/phi-rad'))), v(1)))
        eg_d = D(eg, M(p('ap/eo-guard-td-s'), hdd))
        g_on = GT(MAX(GT(eg, v(0)), M(GT(p('ap/eo-guard-td-s'), v(0)), GT(eg_d, v(0)), GT(eg, v(-3)))), v(0.5))
        # (not the Glide task's: its plan brings the descent in itself; the guard is for the entry and a glide with no site. Left on, the two asked different things of the one pitch)
        guard = IFTHEN(GT(M(g_on, NOT(pl)), v(0.5)), MIN(pmax, S(th, M(p('ap/eo-kg'), v(R2D), Q(eg_d, V_)))), v(-90))
        # (the pitch the speed takes as the nacelles move, from the grid: the Glide task's speed law told what the drag is doing, not left to find it in the speed's error)
        # (and the path's error beside the speed's, as total energy control's pitch has both: as the drag goes the descent the nacelles now give is shallower than the one
        # flown, and the nose comes up for that, not only for the speed it would otherwise gain. Not in a turn: the path there is the turn's)
        if sch:
            ldn = T2('inertia/weight-lbs', 'ctl/nacelle-deg', sch['gl_weight_lb'], sch['gl_nac'], sch['gl_ld'])
            g_ref = NEG(ATAN2(v(1), ldn))
            g_now = ATAN2(p('velocities/h-dot-fps'), MAX(p('velocities/vt-fps'), v(100)))
            g_kb = T1('ap/eo-phi-abs-deg', [10.0, 30.0], [1.0, 0.0])
            c.fn('ap/eo-phi-abs-deg', ABS(p('attitude/phi-deg')))
            g_e = c.fn('ap/eo-plan-g-err-deg', M(pl, g_kb, lim(M(p('ap/eo-plan-kg'), D(g_ref, g_now), v(R2D)), NEG(p('ap/eo-plan-g-max-deg')), p('ap/eo-plan-g-max-deg'))), desc='the path\'s share of the pitch, deg')
            tff = c.fn('ap/eo-plan-theta-deg', S(M(pl_on, is_(2), T2('inertia/weight-lbs', 'ctl/nacelle-deg', sch['gl_weight_lb'], sch['gl_nac'], sch['gl_theta'])), g_e), desc='the pitch the nacelles take at the glide\'s speed, and the path\'s share, deg')
        else:
            tff = v(0)
        ip = p('ap/eo-int-deg')
        # (its integral, while the limit has the pitch, following it: the speed's law takes the pitch back from where
        # the limit leaves it)
        pe = M(p('ap/eo-kp'), Q(ea, v(G0)), v(R2D))
        gl_i = MAX(lim(D(ip, M(p('ap/eo-ki'), Q(ea, v(G0)), v(R2D), dt)), dive, pmax), S(guard, pe))
        # (no more than the pitch at which the wing stalls, the angle of its most lift in Bell's data for the flaperons
        # there (wing_tables, airplane mast: 16 deg F1, 14 F2), but no lower than the dive's. Falling from a hover at
        # 100-150 ft/s, the wing at 40-90 deg, the forward airspeed is the fall seen through the attitude: the speed's
        # law read the pitch's own swing as 40 ft/s^2 of speed and drove the pitch between its limits every 4-5 s, and
        # the dive's guard asked a pull-out the wing could not make; the rotors in the up-flow then pitched the nose up
        # past what the stick holds (to their masts' line with the flow, 90 deg less the nacelles' angle), and from 3000
        # and 4000 ft it fell flat at 160-170 ft/s, the rotor speed lost. With the limit the nose stays down while the
        # wing is stalled, the speed comes and the nacelles with it: from 4000 ft on the wheels at 11.7 ft/s
        # (deadland.py auto), from 4500-8000 at 7.1-7.7 against 7.1-12, from the conversion at 2000 ft at 17 against 41;
        # where the wing flies, the same to the digit, but for a few seconds in a site's turns at 44-55 deg of bank, the
        # wing at 11-12 deg, and in 11 of deadland.py flare's 243, heavy in thin air with the wind behind (by up to 0.9
        # ft/s, 211 intact against 210). The airspeed along the horizon instead of the body's axis held the nose down
        # too, but swung the glide's pitch -5..+5 deg (a shallower path more speed along the horizon, the nose raised
        # for it) and its flares touched at 11 ft/s against 7.3; with it, the limit under the dive's too asked the nose
        # lower in the fall, from 4000 ft 25 ft/s against 9. Its integral no higher, as no higher than
        # ap/eo-pitch-max-deg: held to it less the proportional part, the integral went to -18 deg in a gust's start
        # while the command stayed short of the limit)
        al_, fx_, bm_, CL_, CD_ = s.wing_tables()
        a_st = []
        for x in (0, 1):
            ys = [row[fx_.index(x)] for row in CL_[1]]
            a_st.append(max((y, a_) for a_, y in zip(al_, ys) if -10 <= a_ <= 30)[1])
        stall = c.fn('ap/eo-stall-theta-deg', MAX(dive, S(th, D(T1('aero/wing/flap-index', [0, 1], a_st), p('aero/wing/alpha-deg')))),
                     desc='ENG OUT\'s pitch at the wing\'s stall (no lower than its dive), deg')
        gl_i = MIN(gl_i, D(stall, tff))
        # (its gain ap/eo-khi, 0.15 deg/s per ft/s: with 0.3 the 18 flares of deadland.py flare quick touched at up to
        # 13.8 ft/s, with 0.15 up to 11.3; 0.1 too slow. On the ground, rolling out, held where it touched)
        grd = GT(st, v(4.5))
        # (flown as guidance instead, the deceleration that brings the descent to td-fps at the float, (s^2 - td^2) /
        # 2 (h - float), as the climb's acceleration, held level after it or planned to the speed the tips allow, the
        # light aircraft touched at 1.7-5 ft/s but at 167 KCAS and more, the heavy at 175-193 KCAS with the nacelles too
        # low for the tips: the touchdown's speed, not its descent, is what the corridor binds; the flaperons spend it)
        fl_i = IFTHEN(grd, ip, lim(S(ip, M(p('ap/eo-khi'), eh, dt)), v(-20), tail))
        c.fn('ap/eo-int-next', IFTHEN(GT(MAX(eorise, frise, to3), v(0.5)), th, IFTHEN(flare, fl_i, gl_i)), out='ap/eo-int-deg')
        ip = p('ap/eo-int-deg')
        # (the tail's limit led by the pitch rate: the nose coming up to it, the reference's own overshoot took it 1.9 deg
        # past)
        top = MIN(tail, D(tail, M(p('ap/eo-lead-s'), v(R2D), p('velocities/q-rad_sec'))))
        # the Touchdown task: the mains on, the pitch its own from the attitude they touched at, the nose eased down at ap/eo-derot-dps to where the aircraft rests. The mains
        # sit behind the centre of gravity and their impact throws the nose down at 18-37 deg/s, the wheel on it in 0.1-0.4 s; the augmentation's stick came 0.2 s late, so
        # this has its own: on the pitch's error and its rate, straight to the stick. Down only: the reference never rises, so it can't lift the aircraft off again
        for n, x in (('derot', 1), ('derot-dps', 2.5), ('rest-deg', -0.8), ('derot-kt', 2.0), ('derot-kq', 0.5), ('derot-aft-in', 4.8), ('derot-fwd-in', 1.0), ('derot-ref-deg', 0.0)):
            c.declare('ap/eo-' + n, x)
        touch = GT(M(GT(nst, v(4.5)), LT(st, v(4.5))), v(0.5))
        land = GT(M(on('ap/eo-derot'), GT(MAX(st, nst), v(4.5)), on('ap/eo-flown')), v(0.5))
        d_ref = p('ap/eo-derot-ref-deg')
        c.fn('ap/eo-derot-ref-next', IFTHEN(touch, th, IFTHEN(land, MAX(p('ap/eo-rest-deg'), D(d_ref, M(p('ap/eo-derot-dps'), dt))), th)), out='ap/eo-derot-ref-deg')
        d_on = GT(M(land, GT(d_ref, S(p('ap/eo-rest-deg'), v(0.05)))), v(0.5))
        d_stick = lim(NEG(S(M(p('ap/eo-derot-kt'), D(d_ref, th)), M(p('ap/eo-derot-kq'), D(NEG(p('ap/eo-derot-dps')), M(p('velocities/q-rad_sec'), v(R2D)))))),
                      NEG(p('ap/eo-derot-aft-in')), p('ap/eo-derot-fwd-in'))
        # (the Glide task's stick for the nacelles, from the grid: with them out the stick sat at the augmentation's limit and its trim wound forward, and as they closed the
        # forward trim left behind took the nose to -25 deg at 0.35-0.6 g. Fed in with the nacelles the moment is met as it comes, and the trim has nothing to wind up on)
        c.declare('ap/eo-plan-stick', 1)
        s_ff = M(on('ap/eo-plan-stick'), pl_on, is_(2), T2('inertia/weight-lbs', 'ctl/nacelle-deg', sch['gl_weight_lb'], sch['gl_nac'], [[x * B.X_LON for x in r] for r in sch['gl_stick']])) if sch else v(0)
        # the Flare task's limit on the tail, straight to the stick as the Touchdown's is: the augmentation is slow at this speed by design (its gain a third, its integral 0.1 in/s for 7 deg of error), the nacelles' moment
        # came in faster than it answered with half an inch of stick still unused, and the nose went 1.5-3.5 deg over the limit in the last 2 s
        for n, x in (('tail-kt', 1.0), ('tail-kq', 0.5)):
            c.declare('ap/eo-' + n, x)
        t_over = S(D(th, p('ap/eo-tail-deg')), M(p('ap/eo-tail-kq'), v(R2D), p('velocities/q-rad_sec')))
        t_stick = M(flare, NOT(grd), MAX(v(0), M(p('ap/eo-tail-kt'), t_over)))
        c.fn('ap/eo-lon-next', S(M(d_on, d_stick), s_ff, t_stick), out='ap/eo-lon-in')
        tcd = c.fn('ap/eo-theta-cmd-deg', IFTHEN(land, d_ref, IFTHEN(flare, lim(S(ip, M(p('ap/eo-kh'), eh, NOT(grd))), v(-20), top),
                                                 MIN(MAX(S(lim(D(ip, pe), dive, pmax), tff), guard), stall))),
                   desc='ENG OUT\'s pitch, deg (the flight director\'s)')
        # (flown, the attitude's error integrated into the reference: the attitude, not only the reference, comes to the
        # command, and so to the tail's limit. In the float the nacelles coming up pitched the nose up and the
        # augmentation's own integral, slow at that speed, held it 2-2.5 deg over its reference: at touchdown 1.2-1.5 deg
        # over the limit)
        am = p('ap/eo-att-int-max-deg')
        fly2 = MAX(fly, roll)
        # (not while the stick has no more to give that way: it wound to its limit with the stick against the stop, the reference 5 deg under the command, and as the moment eased the nose went on down through it, 6.8 to 3 deg
        # in 2 s over the ground, and touched 8 KCAS early)
        lon_, slon_ = p('ctl/lon-in'), p('scas/lon-in')
        s_fwd = MAX(GT(lon_, v(0.98 * B.X_LON)), GT(slon_, D(p('scas/auth-in'), v(0.02))))
        s_aft = MAX(LT(lon_, v(-0.98 * B.X_LON)), LT(slon_, D(v(0.02), p('scas/auth-aft-in'))))
        a_err = D(tcd, th)
        a_run = D(v(1), MAX(M(s_fwd, LT(a_err, v(0))), M(s_aft, GT(a_err, v(0)))))
        # (and slower in the flare, from nothing at its start: at ap/eo-ka's 1 per second, under the augmentation's own integral, the pitch swung +-3 deg on a 2.8 s period down to the wheels, and what it held from the glide
        # stepped the reference at the gate. 0.3: the pitch rate's rms in the flare 3.2 to 2.1 deg/s at 60 500 lb)
        for n, x in (('ka-flare', 0.3), ('att-reset', 1)):
            c.declare('ap/eo-' + n, x)
        c.fn('ap/eo-att-int-next', IFTHEN(GT(MAX(NOT(fly2), frise, land, M(on('ap/eo-att-reset'), to3)), v(0.5)), v(0), lim(S(p('ap/eo-att-int-deg'), M(IFTHEN(flare, p('ap/eo-ka-flare'), p('ap/eo-ka')), a_err, a_run, dt)),
             NEG(am), am)), out='ap/eo-att-int-deg')
        tref = S(tcd, p('ap/eo-att-int-deg'))
        # (falling from a hover, the flow from under the wing, the whole stick: the rotors in the up-flow pitch the nose up towards
        # their masts' line, and at 47 500 lb the half stick ran out 5 s in, the nose went to +50 deg, the rotors stopped and it never
        # flew. Not after it: with the whole of it in the dive that went deeper and its pull-out later)
        # (and the whole of it aft while the nose is well under its command: the half stick held aft for 16 s of a fall from a hover, the nose
        # at -49 deg against -20, the dive's limit not its limit at all)
        # the conversion corridor off from the cut to the flare: it keeps the aircraft stable with the rotors under power, and with them
        # windmilling it does the opposite. Falling from a hover held at its lower boundary the nose went to +47-51 deg at 36 000-47 500 lb
        # (once for good, 9456 ft and no flying speed), the gimbals to 18-19 deg; free, at every weight the nose stayed under +2, 170-760 ft less
        # lost, the gimbals at 13-14. Back on for the flare, where off it was no better and from 200 KCAS at 47 500-54 000 lb 4 ft/s harder
        c.declare('ap/eo-cps-free', 1)
        c.declare('ap/eo-cps-free-st', 2.5)
        # (and again where the nacelles' own line in the flare passes it, ap/eo-nac-line-deg: fast, at 173 KCAS, the corridor stopped them at about 32 deg where the tips clear from 35-40.
        # Free through the whole flare they cut the strikes but landed harder, the rise at speed costing lift for too long)
        # (and on the wheels until the aircraft has stopped: back on at the touchdown it drove the nacelles down at their full rate to its boundary for the speed, 39 to 33 deg in a second at 177 KCAS, with the nose coming
        # down, and 15 of 39 flights at 60 500 lb put the tips in the ground rolling. It protects a powered conversion; on the wheels it has nothing to protect)
        hi_raw = T1('velocities/ve-kts', U.CPS[1][0], U.CPS[1][1]) if U.CPS else v(U.NACELLE_MAX)
        late = M(p('ap/eo-nac-go'), MAX(GT(p('ap/eo-nac-line-deg'), hi_raw), GT(p('ctl/nacelle-deg'), hi_raw)))
        c.fn('ap/eo-cps-free-next', M(fly2, p('ap/eo-cps-free'), MAX(LT(st, p('ap/eo-cps-free-st')), M(GT(st, v(2.5)), MAX(GT(st, v(4.5)), late)))), out='ctl/cps-free')
        c.declare('ap/eo-aft-whole', 1)
        c.declare('ap/eo-aft-lead-deg', 5.0)
        c.fn('ap/eo-auth-aft-next', IFTHEN(GT(M(fly, p('ap/eo-aft-whole'), LT(th, D(p('ap/eo-theta-cmd-deg'), p('ap/eo-aft-lead-deg')))), v(0.5)), v(B.X_LON), v(0.5 * B.X_LON)), out='scas/auth-aft-in')
        c.declare('ap/eo-fall-alpha-deg', 45.0)
        # (and with the nacelles out on the Glide task's plan: at 60 500 lb they took more forward stick than the half, the nose stayed at -5 deg against -30, the speed went from 210 to 177 KCAS,
        # and as they closed the stick held forward took the nose to -34 deg, 197 ft/s at 630 ft. The plan has no dive for it to deepen)
        c.fn('ap/eo-auth-next', IFTHEN(GT(MAX(M(fly, GT(p('aero/alpha-deg'), p('ap/eo-fall-alpha-deg'))), M(pl, GT(p('ctl/nacelle-deg'), v(5)))), v(0.5)), v(B.X_LON), v(0.5 * B.X_LON)), out='scas/auth-in')
        # (out of pitch: the attitude, not the command, at the limit: the command reached it at once and the settling
        # began at 100 ft, the nose still coming up)
        c.fn('ap/eo-cap-next', IFTHEN(GT(M(is_(3), GT(th, D(tail, v(0.5))), GT(eh, v(2))), v(0.5)), S(p('ap/eo-cap-s'), dt), v(0)),
             out='ap/eo-cap-s')
        # the bank: with no site, the open course eosite.nas finds over the terrain (ap/eo-open-on: the glide's furthest
        # before the ground comes within the flare's height: over LOWI, held on the heading it had, ENG OUT twice flew
        # into the valley's side), else the heading from engaging, the bug's with HDG HOLD
        c.fn('ap/eo-psi-next', IFTHEN(eorise, p('attitude/psi-rad'), p('ap/eo-psi-rad')), out='ap/eo-psi-rad')
        for n in ('open-on', 'open-hdg-deg'):
            c.declare('ap/eo-' + n, 0)
        bug = M(S(p('/autopilot/settings/heading-bug-deg'), p('/environment/magnetic-variation-deg')), v(1 / R2D))
        dpsi = D(IFTHEN(on('ap/eo-open-on'), M(p('ap/eo-open-hdg-deg'), v(1 / R2D)), IFTHEN(on(AF + 'hdg-hold'), bug, p('ap/eo-psi-rad'))),
                 p('attitude/psi-rad'))
        kpsi = Q(MAX(p('velocities/vt-fps'), v(60)), M(v(G0), p('ap/hdg-tau-s')))
        # (no more than the glide holds at the weight, deadland.py turns: 51-53 deg up to 42 000 lb, 49 at 47 500-54 000,
        # 43 at 60 500; set to 55 the heavy one rolled back to 29, the speed past 219 KCAS)
        d_eo = eo_site()
        held = T1('inertia/weight-lbs', d_eo['weight_lb'], d_eo['bank_held_deg']) if d_eo and 'bank_held_deg' in d_eo else p('ap/eo-bank-deg')
        blim = c.fn('ap/eo-bank-lim-deg', MIN(p('ap/eo-bank-deg'), held), desc='ENG OUT\'s bank limit at the weight, deg')
        # (in the flare by the height, not by the flare's start: the low rotor's tip keeping half of it. At 5 deg from the flare's start, which at
        # 80 ft/s comes at 500 ft, a turn onto the final still 21 deg off its line was cut short and landed 590 ft beside it)
        hs = (0.0, 10.0, 20.0, 30.0, 50.0, 80.0, 120.0)
        tips = [math.degrees(math.asin(max(0.0, min(1.0, (x / 2 - 5.0) / EO_TIP_SPAN_FT)))) for x in hs]
        # (and nothing at the wheels, by the time to go: straight and level when any wheel touches)
        c.fn('ap/eo-tgo-flare-s', Q(p('hh/h-wheels-ft'), MAX(v(5), NEG(p('velocities/h-dot-fps')))), desc='the time to the wheels at this descent, s')
        # (the Final task, in the Glide's bank: under the inner height wings level comes before the line. A turn's descent grows as 1 / cos(bank)^1.5, the gate's contract 60 ft/s from the
        # nominal's 50: 28 deg. At 50 deg down to 230 ft the flare started in the turn at 80 ft/s and touched at 21-24. Rolled out by the height: wide of the line, level, and slower down)
        # (off, 90: at 28 deg, the six weights: landings over 42 ft/s 20 against 8. Rolled out wide of the line it met the ground beside the strip at speed. The turn has to end earlier, not shallower)
        c.declare('ap/eo-final-bank-deg', 90.0)
        c.declare('ap/eo-final-band-ft', 300.0)
        bank_f = S(p('ap/eo-final-bank-deg'), M(D(blim, p('ap/eo-final-bank-deg')), MAX(v(0), MIN(v(1), Q(D(h, S(p('ap/eo-flare-ft'), p('ap/eo-in-ft'))), p('ap/eo-final-band-ft'))))))
        bmax = IFTHEN(flare, MIN(blim, T1('ap/eo-tgo-flare-s', [1.0, 4.0], [0.0, 10.0]), MAX(M(p('ap/eo-flare-bank-deg'), T1('hh/h-wheels-ft', [2.0, 20.0], [0.0, 1.0])), T1('hh/h-wheels-ft', hs, tips))), MIN(M(T1('velocities/vc-kts', [40, 80, 150], [0.6, 0.8, 1]), blim), IFTHEN(pl, bank_f, v(90))))
        # the glide's reach, for the display: the energy height on the wheels at the glide's speed, over the flare's, at
        # the glide's ratio for its speed and the weight (the model's own zero-power trims in airplane mode,
        # deadstick.py --table, 1000 ft), in still air: the gear up down to ap/eo-gear-ft, where the mode asks for it,
        # then coming down for GEAR_S (its drag, so the descent at the glide's speed, growing with it: the two descents'
        # mean), then down; and the time it takes (the descent the ratios give). FlightGear's start, 38 700 lb, glided at
        # 7.47 gear up and 6.10 down (the trims 7.42, 6.18), where the reference loading's 8.44 had put it 5 % too far.
        # Short of the glide flown offline from 3000 ft at 200 KCAS (reachcheck): 4-5 % at the reference loading, 8-12 %
        # at 38 700 lb, about 0.1 NM in its last 1000 ft: the engines' run-down after the cut, and the speed's sag as the
        # gear comes (4-5 kt light), energy the glide's speed leaves out
        # (the energy against the glide's speed at the ground, not here: at a steady calibrated airspeed the true slows
        # as the air thickens, 82 ft of energy height from 3000 ft at 200 KCAS, given to the glide; the ground's density
        # from the standard atmosphere's ratio, (1 - 6.8756e-6 h)^4.2559, over the wheels' height)
        vgt = Q(M(vg, v(KT)), SQ(p('atmosphere/sigma')))
        hsl = p('position/h-sl-ft')
        sig_g = M(p('atmosphere/sigma'), tag('pow', Q(D(v(1), M(v(6.8756e-6), D(hsl, h))), D(v(1), M(v(6.8756e-6), hsl))), v(4.2559)))
        vgg = Q(M(vg2, v(KT)), SQ(sig_g))
        # (less what getting into the glide takes past that, by the speed: from a hover the dive and the rotors' drag spend 900-1600 ft
        # of it, capability.py entry; from 170 KCAS nothing)
        entry = c.fn('ap/eo-entry-ft', T2('inertia/weight-lbs', 'velocities/vc-kts', sch['weight_lb'], sch['entry_kcas'], sch['entry_ft']), desc='the height the fall to flying speed still costs past its way at the glide, ft') if sch else v(0)
        if sch:
            c.fn('ap/eo-entry-loss-ft', T2('inertia/weight-lbs', 'velocities/vc-kts', sch['weight_lb'], sch['entry_kcas'], sch['entry_loss_ft']), desc='the height to the fall\'s end, ft')
            c.fn('ap/eo-entry-way-ft', T2('inertia/weight-lbs', 'velocities/vc-kts', sch['weight_lb'], sch['entry_kcas'], sch['entry_way_ft']), desc='the way to the fall\'s end, ft')
        en = c.fn('ap/eo-energy-ft', D(S(h, Q(D(SQR(p('velocities/vt-fps')), SQR(vgg)), v(2 * G0))), entry),
                  desc='the energy height on the wheels at the glide\'s speed at the ground, ft')
        gl = glide_ld()
        if gl:
            ld_up, ld_dn = [T2('inertia/weight-lbs', 'ap/eo-vg-kt', gl['weight_lb'], gl['kcas'], gl['ld_gear_' + k]) for k in ('up', 'down')]
        else:
            ld_up = ld_dn = T1('ap/eo-vg-kt', [140, 160, 180, 200, 220], [5.65, 6.77, 7.48, 8.44, 7.89])
        fl, gh = p('ap/eo-flare-ft'), p('ap/eo-gear-ft')
        cmd = on('gear/gear-cmd-norm')
        hs = IFTHEN(cmd, en, MIN(en, gh))                       # where the gear starts (has started) to come down
        p0 = IFTHEN(cmd, p('gear/gear-pos-norm'), v(0))         # and from where
        s_up, s_dn = Q(v(1), ld_up), Q(v(1), ld_dn)
        s_t = M(v(0.5), S(S(s_up, M(p0, D(s_dn, s_up))), s_dn))  # the descent per foot while it comes
        x_t = M(vgt, v(GEAR_S), D(v(1), p0))                    # the way it comes in
        a_t = MAX(v(0), D(hs, fl))
        reach = S(M(MAX(v(0), D(en, MAX(hs, fl))), ld_up), MIN(x_t, Q(a_t, s_t)), M(MAX(v(0), D(a_t, M(x_t, s_t))), ld_dn))
        c.fn('ap/eo-reach-nm', M(reach, v(1 / 6076.1)), desc='the glide\'s reach to the flare in still air, NM')
        c.fn('ap/eo-reach-s', Q(reach, vgt), desc='the glide\'s time to the flare, s')
        # the bank: to the landing site when there is one (eng_out_site), else the heading
        hdg_bank = M(kpsi, ATAN2(SIN(dpsi), COS(dpsi)), v(R2D))
        c.fn('ap/eo-ld-up', ld_up, desc='the glide\'s ratio, the gear up, at its speed and the weight (eosite.nas: the reach to each site)')
        # (the guidance's reach to its site from the site's elevation, its plane's at the point, where it is known: the
        # reach above is from the ground under the aircraft, and over LOWI's valley side, 925 ft over the runway, read
        # 1.1 NM short of the runway's; under ground lower than a site it reads long. The height between at the glide's
        # ratio, as eosite.nas ranks them)
        known = GT(M(on('ap/eo-site-on'), on('ap/eo-site-plane')), v(0.5))
        reach_s = S(reach, M(known, D(p('position/terrain-elevation-asl-ft'), p('ap/eo-site-elev-ft')), ld_up))
        # the glide flown as a glider's, its brake part way out for energy both ways: the flaperons at the ratio of the
        # clean glide's that leaves the same share each way, sqrt(r_min), r_min the full flaperons' (deadstick.py
        # --flaptable: 0.60-0.69 of clean by the weight and the gear; sqrt 0.77-0.83, about 15 deg: +21-29 % to
        # stretch, -17-23 % to steepen). A turn's cost in reach is the brake's too: its height per 180 deg grows as the
        # glide's ratio falls, h180 / r, and its way at r LD is h180 LD (deadland.py site's measure, clean)
        gf = glide_ld_flap()
        gp = p('gear/gear-pos-norm')
        if gf:
            rmin = [[row[-1] / row[0] for row in gf[k]] for k in ('ld_gear_up', 'ld_gear_down')]
            r_min = c.fn('ap/eo-brake-min-ratio', S(M(D(v(1), gp), T1('inertia/weight-lbs', gf['weight_lb'], rmin[0])),
                                                    M(gp, T1('inertia/weight-lbs', gf['weight_lb'], rmin[1]))),
                         desc='the full flaperons\' glide ratio, of the clean one')
            r_nom = c.fn('ap/eo-brake-nom-ratio', SQ(r_min), desc='the brake\'s nominal glide ratio, of the clean one (eosite.nas plans at it)')
        else:
            r_min = r_nom = v(1)
        site = s.eng_out_site(reach_s, ld_up, vgt, kpsi, flare, eo, r_nom) if (U.NAME == 'V-22' and eo_site()) else None
        # (wings level while the flow still comes from under the wing: the heading hardly moves then, the turn comes once it flies, and a
        # bank reversed in those seconds, the open course's for the site's, took the fall from a hover nose up to +41 deg, whole stick
        # and all, 1200 ft of height gone)
        phc = c.fn('ap/eo-phi-cmd-deg', M(LT(p('aero/alpha-deg'), p('ap/eo-fall-alpha-deg')), lim(IFTHEN(site['on'], M(site['bank'], v(R2D)), hdg_bank) if site else hdg_bank, NEG(bmax), bmax)),
                   desc='ENG OUT\'s bank, deg')
        # the nacelles (within the corridor) and the lever; on the ground after a coupled landing the lever down
        # (converting at the corridor's lower boundary, as the speed lets the nacelles come down: on AUTO NAC's schedule,
        # the level flight's angle for the speed, the nacelles stayed up, the speed never came, and from every hover and
        # the conversion it struck at 115-200 ft/s)
        lo_, hi_ = (p('ctl/cps-min-deg'), p('ctl/cps-max-deg')) if U.CPS else (v(U.NACELLE_MIN), v(U.NACELLE_MAX))   # (the XV-15's: none)
        # (in the flare from ap/eo-nac-start-ft, kept coming once begun, to ap/eo-nac-flare-deg. From the flare's start,
        # where the corridor let them (below about 190 KCAS: light, or in thin air), the nacelles braked and lifted the
        # aircraft 200 ft up, the descent eased to 15 ft/s, the path then asked for nothing while the speed bled and the
        # descent grew back, and the flare came late and slow: 38 700 lb at 5000 ft and ISA + 20 touched at 39.5 ft/s,
        # 7.4 with them from 100 ft. To 90 deg, floating over ground falling away, their moment took the nose to 22 deg;
        # the tips clear on the wheels from 38.5 deg level, 42 with the nose down)
        # (and earlier the faster the descent, the time they take from 0: off the nacelle brake's steep way down the flare has
        # less height to go, and 15 flights of 40 met the ground at 170-190 KCAS with them at 17 deg, the tips in it)
        for n, x in (('nac-start-ft', 100.0), ('nac-go', 0), ('nac-lead-s', 5.0), ('nac-lead-fps', 25.0)):
            c.declare('ap/eo-' + n, x)
        go = GT(M(flare, MAX(on('ap/eo-nac-go'), LT(h, MAX(p('ap/eo-nac-start-ft'), M(p('ap/eo-nac-lead-s'), MAX(v(0), D(sink, p('ap/eo-nac-lead-fps')))))))), v(0.5))
        c.fn('ap/eo-nac-go-next', go, out='ap/eo-nac-go')
        # (the flaperons ENG OUT's own in the flare: its landing's lift and drag, not the airplane's least-drag law
        # (flap_law takes the more of the two). From the flare's start at ap/eo-flap-rate-dps to ap/eo-flap-max-deg,
        # F2's 40 (F3's 67 adds no lift at the tail's angle and stalls sooner). deadland.py flare, 243 flares, with the
        # damped dive guard, the nacelles' 45 and no cushion: 189 intact against 151 before, median 7.2 ft/s against
        # 8.9, worst 10.2 against 16.5, every one on level or falling ground at every weight, where before the heavy
        # aircraft floated too fast for the nacelles (54 000 lb: 54 intact of 81, against 16); on the tables measured
        # since with the flight state's flaperons, 210, 54 000 lb 61 (deadland.py's docstring). Faster the
        # lift outran the pitch: 3 deg/s left one flare at 27 ft/s, 4-8 deg/s ballooned, set at once 40 deg ballooned
        # every flare to 2.2 g. Braked instead to a speed planned down the flare, no better: the speed stayed over the
        # plan. The flaperons' own rate and limit speed are not public: the emergency's lift is taken over their limit)
        # (the most the flare takes, by the weight: capability.py; ap/eo-flap-max-test-deg: deadland.py capflare's own)
        c.declare('ap/eo-flap-max-test-deg', -1.0)
        if sch:
            c.fn('ap/eo-flap-max-deg', IFTHEN(GT(p('ap/eo-flap-max-test-deg'), v(-0.5)), p('ap/eo-flap-max-test-deg'), T1('inertia/weight-lbs', sch['weight_lb'], sch['flap_max_deg'])),
                 desc='the flaperons the flare goes to, deg')
        else:
            c.declare('ap/eo-flap-max-deg', 40.0)
        for n, x in (('flap-rate-dps', 2.0), ('flap-deg', 0.0)):
            c.declare('ap/eo-' + n, x)
        # (from 0 at the flare's start, the heavy glide's few degrees of AUTO NAC's pitch limit stepping back there: from
        # them, 198 of the 243 intact against 210, the lift coming without the pitch and its drag, the heavy aircraft
        # touching faster: on 1 % up at 54 000 lb 1 intact of 27 against 17, on 1 % down 21 against 17)
        # (the brake: the ratio of the clean glide the path left takes, need / reach, from the site's choice to the
        # flare, between the full flaperons' and none: the margin, against the nominal ratio, by its own measure; the
        # S-turns spend what is over it. Before, a surplus alone, spent by the final's start and the final flown clean:
        # every error after it fell on the flare's point, and over LOWI's north slope the circle's ran 1000 ft short
        # with the brake already in. The deflection from the model's own trims: at 200 KCAS 7.44 to 4.50 from 0 to 40
        # deg at 38 700 lb, the gear up; 6.18 to 3.99 down)
        if gf and site:
            rs = [round(0.40 + 0.05 * i, 2) for i in range(13)]
            def inv(rows):
                return [[float(np.interp(r, [x / row[0] for x in row][::-1], gf['flap_deg'][::-1])) for r in rs] for row in rows]
            # (the energy kept for the final, as a glider keeps its height through the pattern and uses the brakes on final: before it, spent only what the final's own brake couldn't. Spent
            # as soon as it showed the surplus went in the turns, whose cost the plan underrates, and the flights came to the final 1000-2000 ft short, rolling out at 70-160 ft. What the final
            # can spend: its way at the ratio the nacelles give at ap/eo-plan-final-nac over the nominal's, a descent the close can still bring in)
            c.declare('ap/eo-plan-final-nac', 30.0)
            ld_f = T2('inertia/weight-lbs', 'ap/eo-plan-final-nac', sch['gl_weight_lb'], sch['gl_nac'], sch['gl_ld'])
            hold_e = c.fn('ap/eo-plan-hold-ft', M(pl_on, LT(p('ap/eo-site-state'), v(2.5)), p('ap/eo-site-final-ft'), MAX(v(0), D(Q(M(ld_up, r_nom), ld_f), v(1)))), desc='the margin kept for the final, ft')
            m_raw = p('ap/eo-site-margin-ft')
            m_sp = c.fn('ap/eo-plan-spend-ft', MIN(m_raw, MAX(v(0), D(m_raw, hold_e))), desc='the margin to spend here, ft')
            rq = c.fn('ap/eo-brake-ratio', MAX(r_min, MIN(v(1), D(r_nom, Q(m_sp, MAX(reach_s, v(1)))))),
                      desc='the glide\'s ratio, of the clean one, the site\'s path left takes (need / reach)')
            brake = GT(M(fly, LT(st, v(2.5)), on('ap/eo-site-on'), GT(p('ap/eo-site-state'), v(0.5))), v(0.5))
            defl = lambda ratio: S(M(D(v(1), gp), T2('inertia/weight-lbs', ratio, gf['weight_lb'], rs, inv(gf['ld_gear_up']))),
                                   M(gp, T2('inertia/weight-lbs', ratio, gf['weight_lb'], rs, inv(gf['ld_gear_down']))))
            # (no more than its flaperons can bring back to the nominal by the flare's point at their rate: the flare from
            # the nominal's 15 deg as from none, deadland.py flare quick 25 of 27 intact against 24, worst 12.8 ft/s; from
            # 25 deg 12 and 35.4, from 40 15 and 33.1, every 54 000 lb flare broken, the glide arriving at 43 ft/s)
            nom = c.fn('ap/eo-brake-nom-deg', defl('ap/eo-brake-nom-ratio'), desc='the brake\'s nominal deflection, deg')
            # (by the time left to the flare, the height's or the path's, whichever is longer. By the path's alone a high plan
            # took them in on the way to the point, 15 deg by the point with 1800-2000 ft still to lose, and the aircraft
            # flew 12 000 ft on at the nominal's ratio: 40 deg was held for 6 s of a 75 s flight)
            # (the height's at the full brake's descent, the most there is: at the descent of the moment, 40-100 ft/s with the
            # turns and the pull-outs, they were still at 26 deg at the flare, the flare's own 40 came at 220 ft, the speed went
            # early and it fell in at 31 ft/s)
            left = MAX(Q(p('ap/eo-site-path-ft'), MAX(p('velocities/vt-fps'), v(100))),
                       Q(MAX(v(0), D(h, p('ap/eo-flare-ft'))), Q(vgt, M(ld_up, r_min))))
            cap = S(nom, M(p('ap/eo-flap-rate-dps'), left))
            # (and in as the nacelles go out: at 47 500 lb and 170 KCAS 40 deg of flaperon leaves stick for 15 deg of nacelle, none for 45)
            c.declare('ap/eo-nb-line', 0)
            tgt = M(brake, MIN(defl('ap/eo-brake-ratio'), cap), D(v(1), M(p('ap/eo-nb-line'), MIN(v(1), Q(p('ap/eo-nb-deg'), MAX(v(1), p('ap/eo-nb-nac-deg')))))))
            # (the Glide task: with the nacelles out the flaperons from the nominal in by ap/eo-flap-out-deg, for the stick; with them in, the fine tuning that's left)
            nb_ = p('ap/eo-nb-deg')
            tgt = IFTHEN(GT(M(pl, GT(nb_, v(1))), v(0.5)), M(brake, nom, MAX(v(0), D(v(1), Q(nb_, p('ap/eo-flap-out-deg'))))), tgt)
            # (the Glide task's contract with the flare: the nominal's flaperons at the gate, in time at their rate. Short of energy the stretch had them in and the flare started from none, reached only 20 deg by the
            # wheels, and touched at 12-16 ft/s and 151-155 KCAS against 8-10 and 143-146 from the nominal, at 38 722 lb. A shade of reach for the flare's whole lift)
            c.declare('ap/eo-gate-flap-s', 1.0)
            g_left = Q(MAX(v(0), D(h, p('ap/eo-gate-ft'))), MAX(sink, v(10)))
            tgt = IFTHEN(GT(M(pl, LT(g_left, S(Q(nom, p('ap/eo-brake-out-dps')), p('ap/eo-gate-flap-s')))), v(0.5)), MAX(tgt, M(brake, nom)), tgt)
        else:
            tgt = v(0)
        # (deadland.py flare's: the brake held at a deflection into the flare, -1 none)
        c.declare('ap/eo-brake-test-deg', -1.0)
        tgt = IFTHEN(GT(p('ap/eo-brake-test-deg'), v(-0.5)), M(fly, LT(st, v(2.5)), p('ap/eo-brake-test-deg')), tgt)
        c.declare('ap/eo-brake-flap-deg', 0.0)
        c.declare('ap/eo-brake-out-dps', 6.0)
        eb = p('ap/eo-brake-flap-deg')
        # (out faster than in: taken in it sets the flare's start, out it only spends. At 2 deg/s 20 s of a high plan's 30 went
        # on getting there)
        rt = M(IFTHEN(GT(M(pl, GT(p('ap/eo-nb-deg'), v(1))), v(0.5)), p('ap/eo-brake-out-dps'), p('ap/eo-flap-rate-dps')), dt)
        rt_out = M(p('ap/eo-brake-out-dps'), dt)
        c.fn('ap/eo-brake-flap-next', S(eb, MAX(NEG(rt), MIN(rt_out, D(tgt, eb)))), out='ap/eo-brake-flap-deg')
        # (ap/eo-flap-test-deg: deadland.py capflare, the brake's flaperons as a site's approach has them)
        c.declare('ap/eo-flap-test-deg', 0.0)
        # (on the wheels in again at ap/eo-flap-in-dps, none by default: lift off the wing, the weight on the tyres for the brakes and the steering)
        c.declare('ap/eo-flap-in-dps', 0.0)
        flap_up = S(p('ap/eo-flap-deg'), M(p('ap/eo-flap-rate-dps'), dt))
        flap_dn = S(p('ap/eo-flap-deg'), NEG(M(p('ap/eo-flap-in-dps'), dt)))
        own = GT(M(GT(st, v(4.5)), GT(p('ap/eo-flap-in-dps'), v(0.01))), v(0.5))
        c.fn('ap/eo-flap-own-next', own, out='ap/eo-flap-own')
        c.fn('ap/eo-flap-next', IFTHEN(flare, lim(IFTHEN(own, flap_dn, flap_up), v(0), p('ap/eo-flap-max-deg')), MAX(eb, p('ap/eo-flap-test-deg'))),
             out='ap/eo-flap-deg')
        # (on the wheels to the rollout's, ap/eo-nac-safe-deg. ap/eo-nac-flare-deg 45 in the flare: with 60 their moment
        # pitched the nose to 7-8 deg on falling ground, past the tail's limit: 27 quick flares with the flaperons, 21
        # intact with 45 against 18, the worst 9.3 ft/s against 12.2)
        nac_t = IFTHEN(GT(st, v(4.5)), MAX(p('ap/eo-nac-flare-deg'), p('ap/eo-nac-safe-deg')), p('ap/eo-nac-flare-deg'))
        # (the nacelle brake: over a high site the flaperons alone do not spend it. The model's own zero-power trims, flaperons
        # 40 and the gear down (deadstick.py): 4.0 at 200 KCAS, 3.3 with the nacelles at 30 and 150 KCAS, 2.3 at 45 and 150,
        # 2.0 at 130, against the 8000-12 000 ft a short path over a field left past it: the path's own length cannot
        # spend it, the turn's radius a mile, a lap 21 000 ft and more, and nothing between. Same path, same ground: the
        # aircraft only comes down steeper. Out while the flaperons' middle leaves height over, so that they still have
        # both ways to go after it; in over a floor that leaves the speed time to come back before the flare)
        if site:
            c.declare('ap/eo-nb-back', 1.0)
            c.declare('ap/eo-nb-span-ft', EO_NB_SPAN_FT)
            # (the speed back to the flare's takes the height of its energy as well as the glide while it comes: the glide alone left the
            # heavy aircraft, whose flare wants 200 KCAS, diving for the speed at 1200 ft in the last turn, 150 ft/s, pitch -27 deg, and
            # ballooning out of the pull)
            dv = M(MAX(v(0), D(p('ap/eo-nb-after-kt'), p('ap/eo-nb-kt'))), v(KT))
            back = S(M(p('ap/eo-nb-back'), Q(vgt, M(ld_up, r_min)), Q(dv, a_max)), Q(M(vgt, dv), v(G0)))
            # (and what it comes down on the brake while the nacelles close: at 45 deg and glide speed that is 150 ft/s for 5.6 s)
            c.declare('ap/eo-nb-close', 0.0)
            back = S(back, M(p('ap/eo-nb-close'), Q(vgt, p('ap/eo-nb-ld')), p('ap/eo-nb-lag-s')))
            # (the Glide task's: in by the inner height over the gate, closing along a ceiling that falls with the height, the time they take at ap/eo-close-dps at the mean of
            # the descents between; to the planner its middle, the brake's way down at its most above it, the nominal's below)
            h_in = c.fn('ap/eo-in-h-ft', S(p('ap/eo-flare-ft'), p('ap/eo-in-ft')), desc='the height the nacelles are in by, ft')
            span = c.fn('ap/eo-close-span-ft', M(v(0.5), S(Q(vgt, T1('inertia/weight-lbs', sch['gl_weight_lb'], sch['gl_ld_min'])), Q(vgt, M(ld_up, r_nom))), Q(p('ap/eo-nb-nac-deg'), p('ap/eo-close-dps'))), desc='the height the nacelles close over, ft')
            floor = c.fn('ap/eo-nb-floor-ft', IFTHEN(pl_on, S(h_in, span), S(p('ap/eo-flare-ft'), p('ap/eo-nb-settle-ft'), back)), desc='the nacelle brake in by this height, ft')
            # (ap/eo-nb-line: as a glider's airbrakes, the straight line from here to the spot: the glide ratio it takes now, from the margin against the reach
            # and so anew every tick, the nacelles for that ratio from the grid at this weight, none while the nominal glide does. Nothing to spend, nothing
            # opened: the Shuttle's low energy limit comes with it)
            r_raw = D(r_nom, Q(p('ap/eo-site-margin-ft'), MAX(reach_s, v(1))))
            # (the Glide task's line: to the point on the nominal glide ap/eo-in-ft over the gate, not the flare's own: a line to the gate arrives at the brake's descent, too steep to
            # flare. The whole of the ratio: the old line's, of the nominal not the clean glide, read a quarter long and ran long)
            # (less what closing from here will spend past the nominal, the Shuttle's low energy limit: left out, the nacelles stayed out until the margin was gone and 2500 ft under it)
            nb_c = p('ap/eo-nb-deg')
            ld_nb = T2('inertia/weight-lbs', 'ap/eo-nb-deg', sch['gl_weight_lb'], sch['gl_nac'], sch['gl_ld'])
            shut = c.fn('ap/eo-close-cost-ft', M(v(0.5), MAX(v(0), D(Q(vgt, ld_nb), Q(vgt, M(ld_up, r_nom)))), Q(nb_c, p('ap/eo-close-dps')), M(ld_up, r_nom)), desc='what closing the nacelles from here spends past the nominal glide, in way, ft')
            # (and a little in hand, as a glider aims to overshoot its point and brakes to it: spent to the nominal exactly the plan ran a shade short, the flaperons went in to stretch it,
            # and the flare, starting from none, had too little lift: 11-17 ft/s against 8-9 from the nominal's 15 deg)
            c.declare('ap/eo-plan-bias-ft', 300.0)      # (the 5th percentile of the margin at the gate, 38 722 lb, -270 ft)
            r_in = D(r_nom, Q(D(p('ap/eo-plan-spend-ft'), S(shut, p('ap/eo-plan-bias-ft'))), MAX(D(reach_s, M(p('ap/eo-in-ft'), ld_up)), v(500))))
            c.fn('ap/eo-ld-req', IFTHEN(pl_on, M(ld_up, MAX(v(0.05), r_in)), M(ld_up, MAX(v(0.05), Q(r_raw, r_nom)))), desc='the glide ratio to the line\'s end from here')
            nac_r = T2('inertia/weight-lbs', 'ap/eo-ld-req', sch['weight_lb'], sch['nb_ld_axis'], sch['nb_nac_by_ld'])
            by_line = Q(nac_r, MAX(v(1), p('ap/eo-nb-nac-deg')))
            by_span = MAX(v(0), MIN(v(1), Q(p('ap/eo-site-mid-ft'), p('ap/eo-nb-span-ft'))))
            nbs = M(on('ap/eo-site-on'), on('ap/eo-site-commit'), GT(p('ap/eo-site-state'), v(0.5)), IFTHEN(on('ap/eo-nb-line'), by_line, by_span))
            # (the Glide task: the nacelles for the ratio, the gear's share of it, under the ceiling, at their rates)
            nac_by = S(M(D(v(1), gp), T2('inertia/weight-lbs', 'ap/eo-ld-req', sch['gl_weight_lb'], sch['nb_ld_axis'], sch['gl_nac_up'])),
                       M(gp, T2('inertia/weight-lbs', 'ap/eo-ld-req', sch['gl_weight_lb'], sch['nb_ld_axis'], sch['gl_nac_down'])))
            ceil_ = c.fn('ap/eo-nb-ceil-deg', M(p('ap/eo-nb-nac-deg'), MAX(v(0), MIN(v(1), Q(D(h, h_in), MAX(v(1), span))))), desc='the most the nacelles may be out at this height, deg')
            # (only on a path it can measure: on the final's state the path left is the way along the line, and taken thousands of feet off it, past the point, that read nothing,
            # the margin 15 000 ft, and the nacelles went full out with nowhere near the height to spend)
            c.declare('ap/eo-plan-off-ft', 1000.0)
            sure = MAX(LT(p('ap/eo-site-state'), v(2.5)), LT(ABS(p('ap/eo-site-y-ft')), p('ap/eo-plan-off-ft')))
            nb_t = M(fly, is_(2), GT(p('ap/eo-site-fall'), v(0.999)), on('ap/eo-site-on'), on('ap/eo-site-commit'), GT(p('ap/eo-site-state'), v(0.5)), sure, MIN(nac_by, ceil_))
            nb0 = p('ap/eo-nb-deg')
            # (short of energy they come in at their own rate, not the plan's)
            nb_plan = S(nb0, lim(D(nb_t, nb0), NEG(M(IFTHEN(LT(D(p('ap/eo-site-margin-ft'), S(p('ap/eo-close-cost-ft'), p('ap/eo-plan-bias-ft'))), v(0)), v(U.NACELLE_RATE), p('ap/eo-close-dps')), dt)), M(v(U.NACELLE_RATE), dt)))
            nbn = c.fn('ap/eo-nb-next', IFTHEN(GT(p('ap/eo-nb-test-deg'), v(-0.5)), M(fly, p('ap/eo-nb-test-deg')),
                                               IFTHEN(pl_on, nb_plan, M(fly, LT(st, v(2.5)), GT(h, floor), GT(p('ap/eo-site-fall'), v(0.999)), p('ap/eo-nb-nac-deg'), nbs))), out='ap/eo-nb-deg')
            c.declare('ap/eo-site-slow', 0)
            c.fn('ap/eo-nb-used-next', M(eo, MAX(p('ap/eo-nb-used'), GT(nbn, v(5)), M(on('ap/eo-site-on'), on('ap/eo-site-slow')))), out='ap/eo-nb-used')
        # the Flare task's stick: the pitch owns it, and the nacelles have what it leaves (control allocation's clamp: the lower priority takes the remainder). Rising in the float they went on past what the whole
        # forward stick holds, from 188 KCAS at 60 500 lb with the flaperons at 25 deg (the trims say the same: eo_brake_grid_v22.json), the nose went to 9-13 deg against a command of 5 and at worst ballooned to 51 ft
        # and fell in at 32 ft/s, 22 flights of 39. They stop as the stick runs out and come on as the speed gives it back, its moment going with the dynamic pressure. Up only: once risen, never back
        for n, x in (('nac-stick', 1.0), ('nac-stick-band', 0.05)):
            c.declare('ap/eo-' + n, x)
        room = c.fn('ap/eo-nac-room', MAX(v(0), MIN(v(1), Q(D(p('ap/eo-nac-stick'), Q(p('ctl/lon-in'), v(B.X_LON))), p('ap/eo-nac-stick-band')))), desc='the nacelles\' share of their rate in the flare, by the forward stick left')
        nac_now = p('ctl/nacelle-deg')
        in_flare = GT(M(go, LT(st, v(4.5))), v(0.5))
        # (and their timing, the line they arrive on: as late as their rate allows and still have the tips clear on the wheels, at the pitch there is, a second in hand. Freed for a fixed 2.5 s
        # they rose 20 deg from the corridor's 12 at 190 KCAS and touched at 30, the tips in the ground)
        c.declare('ap/eo-nac-tip-deg', eo_tip_deg(EO_TIP_FT))
        c.declare('ap/eo-nac-lead-tgo-s', 1.0)
        c.fn('ap/eo-nac-line-deg', D(D(p('ap/eo-nac-tip-deg'), th), M(v(U.NACELLE_RATE), MAX(v(0), D(p('ap/eo-tgo-flare-s'), p('ap/eo-nac-lead-tgo-s'))))), desc='where the nacelles have to be now to arrive with the tips clear, deg')
        tgt = MAX(nac_now, MIN(nac_t, MAX(hi_raw, p('ap/eo-nac-line-deg'))))
        nac_go = IFTHEN(in_flare, MIN(tgt, S(nac_now, M(room, v(U.NACELLE_RATE), dt))), nac_t)
        c.fn('ap/eo-nac-cmd-deg', MAX(lo_, MIN(hi_, IFTHEN(go, nac_go, p('ap/eo-nb-deg')))),
             desc='ENG OUT\'s nacelle command, within the corridor')
        # (the lever at flat pitch to the ground. The rotors' energy as a cushion, the lever up from 20 ft in the
        # settling, never fired in FlightGear's Kansas landing and offline only a foot up, by chance of the settling's
        # 1 s timer (17 intact of 27 flares with it, 15 without); timed by the height and the descent instead, it jolted
        # the nose down and took the rotor speed from 95 to 85 % in 0.3 s. The touchdown's descent is the flare's)
        lev = c.fn('ap/eo-lever-cmd', IFTHEN(GT(st, v(4.5)), MAX(v(0), D(p(thr), M(v(0.5), dt))), v(0)))
        c.fn('ap/eo-flown-next', IFTHEN(GT(fly, v(0.5)), v(1), IFTHEN(eo, p('ap/eo-flown'), v(0))), out='ap/eo-flown')
        drive_ = MAX(fly, M(GT(st, v(4.5)), p('ap/eo-flown')))
        c.fn('ap/eo-phase-next', nst, out='ap/eo-phase')
        for n, x in (('was', eo), ('fly-was', fly), ('both-was', both)):
            c.fn('ap/eo-%s-next' % n, x, out='ap/eo-' + n)
        return dict(fly=fly2, rise=frise, theta=M(tref, v(1 / R2D)), phi=M(phc, v(1 / R2D)), nac=p('ap/eo-nac-cmd-deg'), lever=lev, drive=drive_)

    def eng_out_site_line(s):
        """ENG OUT's landing site's inputs from Nasal/eosite.nas, and the aircraft from its point, along the line (+
        past it) and across it (+ right): the line a great circle as the approach's (its course here turned by the
        meridians' convergence). Before ENG OUT's height, which flies over the site's plane: [x, y, the course]."""
        c = s.ctl
        for n, x in (('on', 0), ('new', 0), ('lat-deg', 0.0), ('lon-deg', 0.0), ('course-deg', 0.0), ('side', 0), ('state', 0), ('s', 1),
                     ('tha-prev', 0.0)):
            c.declare('ap/eo-site-' + n, x)
        R_E = 20925646.0
        la, lo = p('position/lat-geod-rad'), p('position/long-gc-rad')
        laF, loF = M(p('ap/eo-site-lat-deg'), v(1 / R2D)), M(p('ap/eo-site-lon-deg'), v(1 / R2D))
        dN = M(D(la, laF), v(R_E))
        dE = M(D(lo, loF), v(R_E), COS(M(S(la, laF), v(0.5))))
        cc = c.fn('ap/eo-site-course-rad', S(M(p('ap/eo-site-course-deg'), v(1 / R2D)), M(D(lo, loF), SIN(la))), desc='the line\'s course here, rad')
        x = c.fn('ap/eo-site-x-ft', S(M(dE, SIN(cc)), M(dN, COS(cc))), desc='the aircraft along the landing\'s line from its point, ft')
        y = c.fn('ap/eo-site-y-ft', D(M(dE, COS(cc)), M(dN, SIN(cc))), desc='the aircraft right of the landing\'s line, ft')
        s.eo_line = [x, y, cc]
        return s.eo_line

    def eng_out_site(s, reach, ld_up, vgt, kpsi, flare, eo, r_nom):
        """ENG OUT's landing site: the point where the flare is to begin (ap/eo-site-lat-deg, -lon-deg) on the landing's
        line (ap/eo-site-course-deg, true), from Nasal/eosite.nas (ap/eo-site-on; ap/eo-site-new pulsed for a new one).
        Flown to as the Space Shuttle's terminal area energy management flew to its runway [TAEM]: the leg (a turn, a
        straight; below) to the heading alignment circle, tangent to the final ap/eo-site-final-ft before the point on
        the path's side (ap/eo-site-side's, +1 right), its radius the glide's speed's at ap/eo-hac-frac of ENG OUT's
        bank limit (as WYPT's arcs, room for their corrections: 44 deg, near the least height per heading turned,
        deadland.py site); around it, its bank fed forward; the final. The final: the roll out's own time from the
        circle's bank at the flight director's roll rate and the augmentation's delay, so that one begun as late as the
        circle's end still levels before the gate. The margin: the reach less what the path takes (each straight leg's
        way through the air, its ground speed's share; the circle at its measured height per 180 deg, at the glide's
        straight ratio), + high. High before the circle, S-turns: the leg turned away from the final by acos(L1 / (L1 +
        margin)), the angle that spends the margin by the circle. States (ap/eo-site-state): 1 to the circle, 2 on it, 3
        on the final; the flare flies the final's law. Returns the bank (rad) and whether it flies."""
        c = s.ctl
        c.declare('ap/eo-hac-frac', 0.8)
        c.declare('ap/eo-site-commit', 0)
        c.fn('ap/eo-site-commit-num', p('ap/eo-site-commit'))
        d_ = eo_site()
        on = lambda prop: GT(p(prop), v(0.5))
        TWO_PI = 2 * math.pi
        wrap = lambda a: ATAN2(SIN(a), COS(a))
        def m2(a):                  # to [0, 2 pi) (JSBSim's <mod> is the integers')
            w_ = ATAN2(SIN(a), COS(a))
            return IFTHEN(LT(w_, v(0)), S(w_, v(TWO_PI)), w_)
        x, y, cc = s.eo_line
        # (what is used more than once here has a name: these are strings, and each use of one pastes its whole tree in.
        # The four paths written out so came to 125 000 elements, 95 % of the flight controls' work, every 1/60 s in
        # all flight, ENG OUT or not)
        vgt = c.fn('ap/eo-site-vgt-fps', vgt, desc='the glide\'s true airspeed, ft/s')
        # the circle, the final. Its bank eosite.nas's (ap/eo-site-hac-deg) when it sends one, else ap/eo-hac-frac of
        # the bank limit: a wider circle spends more of the glide per degree turned, the time the turn takes at the
        # glide's own descent (deadland.py turns: at 42 000 lb 1763 ft per 180 deg at 44, 4074 at 20, 5456 at 15),
        # the surplus a short way to a site leaves past what the brake and the S-turns spend (11 000 ft long, before)
        c.declare('ap/eo-site-hac-deg', 0.0)
        phd = c.fn('ap/eo-site-hac-bank-deg', IFTHEN(GT(p('ap/eo-site-hac-deg'), v(0.5)), p('ap/eo-site-hac-deg'),
                                                     M(p('ap/eo-hac-frac'), p('ap/eo-bank-lim-deg'))), desc='the circle\'s bank, deg')
        ph = M(phd, v(1 / R2D))
        R = c.fn('ap/eo-site-r-ft', Q(SQR(vgt), M(v(G0), tag('tan', ph))), desc='the heading alignment circle\'s radius, ft')
        rate, lag = M(p('ap/eo-phi-rate-dps'), v(1 / R2D)), p('ap/roll-lag-s')
        # (the final: the roll out and the speed settled, three of its loop's time constants, 1 / ap/eo-kv. With the
        # roll out's time alone, 1350 ft, the speed was 5 kt down after the circle at the gate, the nose went down for
        # it, the pull out's guard brought it up again, and the flare began at 13 ft/s, held that while the speed bled
        # 56 kt with the nacelles coming up, and touched at 40 ft/s)
        # (shorter when eosite.nas sends one, ap/eo-site-final-cmd-ft: the Shuttle's minimum entry point [TAEM], the way
        # to go cut when the energy is low)
        c.declare('ap/eo-site-final-cmd-ft', 0.0)
        Lf0 = c.fn('ap/eo-site-final-full-ft', M(vgt, S(Q(ph, rate), lag, Q(v(3), p('ap/eo-kv')))), desc='the full final, ft')
        Lf = c.fn('ap/eo-site-final-ft', IFTHEN(GT(p('ap/eo-site-final-cmd-ft'), v(0.5)), MIN(p('ap/eo-site-final-cmd-ft'), Lf0), Lf0), desc='the final, ft')
        st = p('ap/eo-site-state')
        is_ = lambda n: GT(M(GT(st, v(n - 0.5)), LT(st, v(n + 0.5))), v(0.5))
        new = GT(MAX(p('ap/eo-site-new'), M(on('ap/eo-site-on'), eo, LT(st, v(0.5)))), v(0.5))
        sd = p('ap/eo-site-s')      # (the circle's side: the path's, below)
        # the aircraft from the circle's centre (the final's start E abeam it, R to the side): the tangent point it goes
        # round the circle from, the turn from there to E, and from abeam the aircraft
        u, w = S(x, Lf), D(y, M(sd, R))
        # (no way on, out of a hover, the heading: the ground's track there is the drift's)
        trk_ = c.fn('ap/eo-site-track-rad', IFTHEN(GT(p('velocities/vg-fps'), v(60)), ATAN2(p('velocities/v-east-fps'), p('velocities/v-north-fps')), p('attitude/psi-rad')), desc='the track, rad')
        lam = ATAN2(w, u)
        lamE = M(sd, v(-math.pi / 2))
        # (the final's line extended is the circle's tangent at E: from on it the tangent point is E, and its turn 0 or
        # a whole circle by a hair; a turn the roll in's lead would not fly is none: the final's law takes the aircraft
        # onto the line)
        lead_a = c.fn('ap/eo-site-lead-rad', M(Q(M(v(G0), tag('tan', ph)), vgt), S(Q(ph, M(v(2), rate)), lag)), desc='the roll in\'s lead, rad of turn')
        tha = m2(M(sd, D(lamE, lam)))
        tha = c.fn('ap/eo-site-arc-rad', IFTHEN(GT(tha, D(v(TWO_PI), lead_a)), v(0), tha),
                   desc='the circle\'s turn still to go, from abeam the aircraft, rad')
        # (and the turn still to go by the heading, to the final's course the way the circle turns: by where the
        # aircraft is round the circle alone, a site handed over with the aircraft near the final's start but heading
        # across it read the circle flown, its turn unpaid in the margin, and the final began unaligned, banked 54 deg
        # for 20 s, its surplus no longer to be spent: over LOWI 11 300 ft long, past the runway's end; joined on the
        # circle with the heading well off its track, the margin read 7000 ft high and the landing came 11 000 ft short)
        tha_h = m2(M(sd, D(cc, trk_)))
        tha_h = c.fn('ap/eo-site-arc-hdg-rad', IFTHEN(GT(tha_h, D(v(TWO_PI), lead_a)), v(0), tha_h),
                     desc='the circle\'s turn still to go by the heading, rad')
        bk = 'turn_ft_per_180_bank' in d_
        h180 = (T2('inertia/weight-lbs', 'ap/eo-site-hac-bank-deg', d_['weight_lb'], d_['turn_bank_deg'], d_['turn_ft_per_180_bank']) if bk
                else T1('inertia/weight-lbs', d_['weight_lb'], d_['turn_ft_per_180']))
        h180_0 = T2('inertia/weight-lbs', 'ap/eo-bank-lim-deg', d_['weight_lb'], d_['turn_bank_deg'], d_['turn_ft_per_180_bank']) if bk else h180
        # the turn in and out as delays: half the roll's ramp at its rate, and the augmentation's own (as WYPT's arcs)
        dly = c.fn('ap/eo-site-dly-s', S(Q(ph, M(v(2), rate)), lag), desc='the turn in or out as a delay, s')
        w_out = c.fn('ap/eo-site-wout-rad', M(Q(M(v(G0), tag('tan', ph)), vgt), dly), desc='the heading the delay turns through, rad')
        passed = GT(M(LT(p('ap/eo-site-tha-prev'), v(1)), GT(tha, v(TWO_PI - 1))), v(0.5))
        # (a margin a whole turn of the circle would spend: round it again, spiralling down over the site, as the S-turns
        # can spend no more than the leg allows: from 6000 ft 2 NM out the margin was 5.4 NM)
        lap = M(v(2), h180, ld_up)
        # (onto the final unless going round again leaves less over: no more than half a lap high. With a whole lap the test,
        # over LOWI a final began at 5900 ft, 21 000 ft high 2400 ft from its point, with nothing left to spend it on, and
        # it flew on over the field into the rising ground 4 NM past it)
        # (lined up from either side: 22 deg short of the final's course on a circle turning away from it read 338 deg to go, a
        # lap, where a pilot rolls out and takes the final)
        tha_s = c.fn('ap/eo-site-arc-hdg-near-rad', IFTHEN(GT(tha_h, v(TWO_PI - 1)), D(v(TWO_PI), tha_h), tha_h))
        # (and on a committed path, joined already past the final's start: the final's law takes the line from either side, where
        # the circle's, never banking against its turn, held the wings level across it: over LOWI 4300 ft through the line,
        # good energy and the field underneath, into the valley's far side)
        # (only just past it, within 45 deg of the line on the circle: a circle on the far side joined at its forward edge has its 270 deg still to go, past the final's start by x alone. Over LOWI,
        # flying at the slope north of the valley, the final's law dropped the circle 3300 ft off the line and turned along the slope instead of round, and it flew into it)
        inside = M(p('ap/eo-site-commit-num'), GT(x, NEG(Lf)), LT(x, v(0)), GT(tha, v(1.75 * math.pi)))
        to3 = GT(MAX(M(is_(2), MAX(LT(tha, w_out), passed, GT(tha, v(TWO_PI - 1))), MAX(LT(tha_h, w_out), GT(tha_h, v(TWO_PI - 1))), LT(p('ap/eo-site-margin-ft'), M(lap, v(0.5)))),
                     M(is_(2), inside)), v(0.5))
        # each straight leg's way through the air: its ground speed's share (the wind along and across its track)
        wN, wE = p('atmosphere/total-wind-north-fps'), p('atmosphere/total-wind-east-fps')
        def air(trk):
            al, ac = S(M(wN, COS(trk)), M(wE, SIN(trk))), D(M(wE, COS(trk)), M(wN, SIN(trk)))
            return Q(vgt, MAX(v(30), S(al, SQ(MAX(v(1), D(SQR(vgt), SQR(ac)))))))
        air_cc = c.fn('ap/eo-site-air-final', air(cc), desc='the final\'s way through the air per ft over the ground')
        # (falling to flying speed the first turn is nearly free: from a hover it came round 120 deg in 12 s for no more height than
        # the fall takes straight on, the radius and cost of the speed it's at, not the glide's. Planned at the glide's, 3 NM of turning
        # that weren't there hid every field but the one straight ahead, 1.2 NM on and 12 000 ft too close)
        # (over once and for all at its speed: not by the phase, which a glide held under 190 KCAS by the brake never leaves, and at 136 KCAS
        # on the nacelle brake the fall would have read as begun again and the brake gone)
        c.declare('ap/eo-fall-over', 0)
        c.fn('ap/eo-fall-over-next', M(eo, MAX(p('ap/eo-fall-over'), GT(p('velocities/vc-kts'), v(EO_FALL_KT)))), out='ap/eo-fall-over')
        fall = c.fn('ap/eo-site-fall', IFTHEN(on('ap/eo-fall-over'), v(1), MIN(v(1), MAX(v(0.03), SQR(Q(p('velocities/vc-kts'), v(EO_FALL_KT)))))),
                    desc='the first turn\'s radius and cost in the fall, of the glide\'s')
        k_t = c.fn('ap/eo-site-turn-ft-rad', Q(M(h180, ld_up), v(math.pi)),
                   desc='the circle\'s turn\'s cost in the glide\'s still-air way, ft per rad (eosite.nas ranks the sites with it)')
        k_t0 = c.fn('ap/eo-site-turn0-ft-rad', Q(M(h180_0, ld_up, fall), v(math.pi)), desc='the leg\'s turn\'s at the bank limit, ft per rad')
        arc_eq = lambda th: M(th, k_t)
        arc0 = lambda th: M(th, k_t0)        # (the turn onto the leg, at the bank limit: priced at the circle's, 30 % over)
        # (before the circle, the turn from the track onto the leg too, at the circle's cost: from a site behind the
        # turn back, 180 deg, about 1500 ft, and each S-turn's turn back went unpaid, the aircraft arriving low)
        trk = trk_
        air_trk = c.fn('ap/eo-site-air-track', air(trk), desc='straight on, the way through the air per ft over the ground')
        # the leg a Dubins path [Dubins 1957]: a turn at ENG OUT's bank limit from the aircraft's own track, the straight
        # common tangent, the circle onto the final. Four: the circle on either side, the first turn either way (k: 2 for
        # the circle on the left, + 1 for the cross path, the turns opposite). A cross path whose circles overlap flies
        # straight first, until they part: the teardrop. eosite.nas prices each with the terrain under it and sends its
        # own (ap/eo-site-side, ap/eo-site-turn); without it, the cheapest here, kept unless another is cheaper by half
        # a radian's turn (near 180 deg two are alike, and the bank would reverse). Planned with the turn in place on
        # the side the aircraft was on, a site behind and the aircraft heading away read 7000 ft high over LOWI: the
        # turn back carried it 1.5 NM aside, it joined the final 4500 ft off the line, flared 800 ft off it and touched
        # down 10 000 ft long; the teardrop's way there was 3 NM more than the reach
        psi_s = c.fn('ap/eo-site-psi-rad', wrap(D(trk, cc)), desc='the track from the line\'s course, rad')
        sin_psi, cos_psi = c.fn('ap/eo-site-psi-sin', SIN(psi_s)), c.fn('ap/eo-site-psi-cos', COS(psi_s))
        R0 = c.fn('ap/eo-site-r0-ft', Q(M(SQR(vgt), fall), M(v(G0), tag('tan', M(p('ap/eo-bank-lim-deg'), v(1 / R2D))))),
                  desc='the leg\'s turn\'s radius at ENG OUT\'s bank limit, ft')
        def dubins(pre, s1, s2, xa, ya):
            n_ = lambda n, e: c.fn(pre + n, e)
            c1x, c1y = D(xa, M(v(s1), R0, sin_psi)), S(ya, M(v(s1), R0, cos_psi))
            dx_, dy_ = n_('dx-ft', D(NEG(Lf), c1x)), n_('dy-ft', D(M(v(s2), R), c1y))
            rho = n_('rho-ft', D(M(v(s2), R), M(v(s1), R0)))
            d2 = n_('d2-ft2', S(SQR(dx_), SQR(dy_)))
            Lr = n_('l-ft', SQ(MAX(v(0), D(d2, SQR(rho)))))
            phi = n_('phi-rad', D(ATAN2(dy_, dx_), ATAN2(rho, MAX(Lr, v(1)))))
            a1, a2 = n_('a1-raw', m2(M(v(s1), D(phi, psi_s)))), n_('a2-raw', m2(M(v(-s2), phi)))
            a1, a2 = [n_(n, IFTHEN(GT(a_, D(v(TWO_PI), lead_a)), v(0), a_)) for n, a_ in (('a1-rad', a1), ('a2-rad', a2))]
            chi = n_('chi-rad', S(cc, phi))
            # (the circles touching, a turn rolling out onto the circle, a path of no leg: overlapping by a hair it read as none
            # for a frame, and the guidance took another for that frame)
            cost = n_('cost-ft', IFTHEN(GT(S(d2, M(v(2 * EO_TOUCH_FT), ABS(rho))), SQR(rho)), S(M(Lr, air(chi)), arc0(a1), arc_eq(a2)), v(1e9)))
            return [Lr, phi, a1, a2, cost]
        KS = ((1, 1), (1, -1), (-1, -1), (-1, 1))
        paths = []
        for k, (s2, s1) in enumerate(KS):
            d0, s0 = dubins('ap/eo-site-p%d-' % k, s1, s2, x, y), v(0)
            if s1 != s2:
                # (straight on until the circles are R + R0 apart: the far root of |C1 + s u - C2| = R + R0)
                ddx = c.fn('ap/eo-site-p%d-ddx-ft' % k, D(D(x, M(v(s1), R0, sin_psi)), NEG(Lf)))
                ddy = c.fn('ap/eo-site-p%d-ddy-ft' % k, D(S(y, M(v(s1), R0, cos_psi)), M(v(s2), R)))
                ud = c.fn('ap/eo-site-p%d-ud-ft' % k, S(M(cos_psi, ddx), M(sin_psi, ddy)))
                q = c.fn('ap/eo-site-p%d-q-ft2' % k, D(S(SQR(ud), SQR(S(R, R0))), S(SQR(ddx), SQR(ddy))))
                sq = c.fn('ap/eo-site-p%d-out-ft' % k, IFTHEN(GT(q, v(0)), S(D(SQ(MAX(v(0), q)), ud), v(1)), v(0)))
                d1 = dubins('ap/eo-site-p%d-o-' % k, s1, s2, S(x, M(sq, cos_psi)), S(y, M(sq, sin_psi)))
                c1 = c.fn('ap/eo-site-p%d-c1-ft' % k, IFTHEN(GT(sq, v(2)), S(M(sq, air_trk), d1[4]), v(1e9)))
                out_ = c.fn('ap/eo-site-p%d-out' % k, LT(c1, d0[4]))
                d0 = [c.fn('ap/eo-site-p%d-%s-out' % (k, n), IFTHEN(GT(out_, v(0.5)), e1_, e0_))
                      for n, e1_, e0_ in zip(('l-ft', 'phi-rad', 'a1-rad', 'a2-rad', 'cost-ft'), d1[:4] + [c1], d0)]
                s0 = M(out_, sq)
            paths.append(d0 + [s0])
        c.declare('ap/eo-site-k', 0)
        c.declare('ap/eo-site-turn', 0)
        kp = p('ap/eo-site-k')
        def by_k(vals, k_=kp):
            return IFTHEN(LT(k_, v(0.5)), vals[0], IFTHEN(LT(k_, v(1.5)), vals[1], IFTHEN(LT(k_, v(2.5)), vals[2], vals[3])))
        side_, turn_ = p('ap/eo-site-side'), p('ap/eo-site-turn')
        # (a side alone, sent: the cheaper of its two)
        costs = [S(pk[4], M(v(1e9), GT(ABS(side_), v(0.5)), GT(ABS(D(side_, v(s2))), v(0.5)))) for pk, (s2, s1) in zip(paths, KS)]
        costs = [c.fn('ap/eo-site-cost%d-ft' % k, e) for k, e in enumerate(costs)]
        cmin = MIN(*costs)
        best = IFTHEN(LT(costs[0], S(cmin, v(1))), v(0), IFTHEN(LT(costs[1], S(cmin, v(1))), v(1), IFTHEN(LT(costs[2], S(cmin, v(1))), v(2), v(3))))
        own = IFTHEN(new, best, IFTHEN(LT(S(cmin, arc_eq(v(0.5))), by_k(costs)), best, kp))
        k_sent = S(IFTHEN(LT(side_, v(0)), v(2), v(0)), IFTHEN(LT(M(side_, turn_), v(0)), v(1), v(0)))
        cm = on('ap/eo-site-commit')
        sent = GT(M(GT(ABS(side_), v(0.5)), GT(ABS(turn_), v(0.5)), MAX(cm, LT(by_k([pk[4] for pk in paths], k_sent), v(1e8)))), v(0.5))
        kn = c.fn('ap/eo-site-k-next', IFTHEN(GT(MAX(new, is_(1)), v(0.5)), IFTHEN(sent, k_sent, own), kp), out='ap/eo-site-k')
        c.fn('ap/eo-site-s-next', by_k([v(s2) for s2, s1 in KS], kn), out='ap/eo-site-s')
        L1n, phi_, a1_, a2_, cost_, s0_ = [c.fn('ap/eo-site-sel-' + n, by_k([pk[i] for pk in paths]))
                                           for i, n in enumerate(('l-ft', 'phi-rad', 'a1-rad', 'a2-rad', 'cost-ft', 's0-ft'))]
        s1_ = c.fn('ap/eo-site-sel-s1', by_k([v(s1) for s2, s1 in KS]))
        # A path committed (eosite.nas sends it with the site, ap/eo-site-commit): its first turn's way, a straight on first, the
        # point where the leg meets the circle and the circle's turn from there. Flown as a pilot flies it: the turn the
        # way it was planned until the nose is on the point, then to the point. Solved anew each frame the path turned over
        # at its corners: 104 jumps of 3000+ ft on the way to the circle in 40 flights, a lap or more each, the bank
        # following them, and the height went on turns that led nowhere. (No path sent: solved here, as deadland.py flies)
        for n in ('tx-ft', 'ty-ft', 'a2-sent-rad', 'a1-sent-rad', 's0-left-ft', 'leg'):
            c.declare('ap/eo-site-' + n, 0)
        dxT, dyT = c.fn('ap/eo-site-t-dx-ft', D(p('ap/eo-site-tx-ft'), x)), c.fn('ap/eo-site-t-dy-ft', D(p('ap/eo-site-ty-ft'), y))
        dT = c.fn('ap/eo-site-t-dist-ft', SQ(S(SQR(dxT), SQR(dyT))))
        brgT = c.fn('ap/eo-site-t-brg-rad', ATAN2(dyT, dxT))
        chiT = c.fn('ap/eo-site-t-chi-rad', S(cc, brgT))
        a1r = c.fn('ap/eo-site-t-a1-raw', m2(M(p('ap/eo-site-turn'), D(chiT, trk))))
        s0l = p('ap/eo-site-s0-left-ft')
        # (the straight on first read again from where the aircraft is, never shorter than the plan's: sent from a turn, the aircraft rolled out on a track the plan didn't have, and at its end the
        # turn at the bank limit rolled out 1000-1300 ft past the line, with 1400 ft of straight still wanted. In the first turn it stays ended)
        c.declare('ap/eo-site-s0-live', 1)
        s0_run = D(s0l, M(p('velocities/vg-fps'), p('simulation/channel-dt')))
        c.fn('ap/eo-site-s0-left-next', IFTHEN(GT(M(on('ap/eo-site-s0-live'), GT(s0l, v(1))), v(0.5)), MAX(v(0), s0_run, s0_), MAX(v(0), s0_run)), out='ap/eo-site-s0-left-ft')
        pointed = MAX(LT(a1r, lead_a), GT(a1r, D(v(TWO_PI), lead_a)))
        # (and the bearing gone by the nose the short way, far more turn to go than planned: passed, not a lap to come. Flying straight
        # on first it slid past, the turn read a whole circle, -20 000 ft, and the aircraft flew 330 deg of it at 43 deg of bank
        # and came down in the turn)
        passed = GT(a1r, S(p('ap/eo-site-a1-sent-rad'), v(2)))
        c.fn('ap/eo-site-leg-next', IFTHEN(new, v(0), MAX(p('ap/eo-site-leg'), M(MAX(pointed, passed), LT(s0l, v(1))))), out='ap/eo-site-leg')
        a1T = c.fn('ap/eo-site-a1-now-rad', IFTHEN(on('ap/eo-site-leg'), v(0), a1r), desc='the committed path\'s first turn still to go, rad')
        a2T = p('ap/eo-site-a2-sent-rad')
        costT = S(M(dT, air(chiT)), arc0(a1T), arc_eq(a2T), M(s0l, air_trk))
        L1n, phi_, a1_, a2_, cost_, s0_, s1_ = [c.fn('ap/eo-site-fly-' + n, IFTHEN(cm, a_, b_)) for n, a_, b_ in
                                                (('l-ft', dT, L1n), ('phi-rad', brgT, phi_), ('a1-rad', a1T, a1_), ('a2-rad', a2T, a2_), ('cost-ft', costT, cost_),
                                                 ('s0-ft', s0l, s0_), ('s1', p('ap/eo-site-turn'), s1_))]
        # (the circle from here on the side now chosen, the point to it: past the side's choice, as they are read)
        w = c.fn('ap/eo-site-w-ft', D(y, M(sd, R)))
        dd = c.fn('ap/eo-site-dd-ft', SQ(S(SQR(u), SQR(w))))
        lam = c.fn('ap/eo-site-lam-rad', ATAN2(w, u))
        lamT = c.fn('ap/eo-site-lamt-rad', S(lam, M(sd, tag('acos', MIN(v(1), Q(R, MAX(dd, v(1))))))))
        thT = c.fn('ap/eo-site-tht-raw', m2(M(sd, D(lamE, lamT))))
        thT = c.fn('ap/eo-site-tht-rad', IFTHEN(GT(thT, D(v(TWO_PI), lead_a)), v(0), thT))
        L1 = c.fn('ap/eo-site-l1-point-ft', SQ(MAX(v(0), D(SQR(dd), SQR(R)))))
        chi1 = c.fn('ap/eo-site-chi1-point-rad', S(cc, lamT, M(sd, v(math.pi / 2))))
        # (none at all, the aircraft's turn inside the circle: the point to the circle as before, straight on)
        inf = c.fn('ap/eo-site-none', GT(cost_, v(1e8)))
        need1 = c.fn('ap/eo-site-need1-ft', IFTHEN(inf, S(M(L1, air(chi1)), arc0(ABS(wrap(D(chi1, trk)))), arc_eq(thT)), cost_))
        L1, thT = c.fn('ap/eo-site-leg-ft', IFTHEN(inf, L1, L1n)), c.fn('ap/eo-site-circle-rad', IFTHEN(inf, thT, a2_))
        phi_, a1_, s0_ = (c.fn('ap/eo-site-leg-phi-rad', IFTHEN(inf, psi_s, phi_)), c.fn('ap/eo-site-leg-a1-rad', IFTHEN(inf, v(0), a1_)),
                          c.fn('ap/eo-site-leg-s0-ft', IFTHEN(inf, v(0), s0_)))
        chi1 = c.fn('ap/eo-site-chi1-rad', S(cc, phi_))
        # (on the circle nearly lined up, a hair past its end by where it is: a whole lap by the position alone, the margin down
        # 27 000 ft in a step from a perfect place, 9700 ft high half a mile from a field, and the field was given up)
        tha_left = c.fn('ap/eo-site-arc-left-rad', IFTHEN(GT(M(GT(tha, v(TWO_PI - 1)), LT(tha_s, v(1))), v(0.5)), tha_s, MAX(tha, tha_s)),
                        desc='the circle\'s turn still to go, rad')
        # (on the final's state the way to the point, not only along the line: taken 3000-8000 ft off it, joining from inside the circle, the way along read a few thousand feet or
        # nothing, the margin 13 000 ft over, and the brakes went to their full on a surplus that wasn't there)
        # (and the turn still to make onto it: from where that turn rolls out, not from here. Taken 100 deg off the course, the way straight to the point read 2000-4000 ft of margin that wasn't
        # there, the brakes went out in the turn at 1000 ft, 100 ft/s into the flare's gate with 50 deg of bank on, where the turn alone left the height to roll out)
        vg_ = MAX(p('velocities/vg-fps'), v(60))
        dpsi = c.fn('ap/eo-site-dpsi-rad', wrap(D(cc, trk)), desc='the final\'s course from the track, rad')
        rc = c.fn('ap/eo-site-rc-ft', Q(SQR(vg_), M(v(G0), tag('tan', M(p('ap/eo-bank-lim-deg'), v(1 / R2D))))), desc='the turn at the bank limit at this speed, ft')
        c.declare('ap/eo-site-way-turn', 1)
        a3 = M(on('ap/eo-site-way-turn'), ABS(dpsi))
        x3 = S(x, M(rc, SIN(a3)))
        y3 = D(y, M(Q(dpsi, MAX(ABS(dpsi), v(1e-3))), rc, D(v(1), COS(a3))))
        d3 = c.fn('ap/eo-site-final-way-ft', SQ(S(SQR(MAX(v(0), NEG(x3))), SQR(y3))), desc='the way to the point from the turn onto the final\'s end, ft')
        need = IFTHEN(is_(3), S(arc0(a3), M(d3, air_cc)),
                      IFTHEN(is_(2), S(arc_eq(tha_left), M(Lf, air_cc)),
                             S(need1, M(Lf, air_cc))))
        mg = c.fn('ap/eo-site-margin-ft', D(M(reach, r_nom), need),
                  desc='the reach at the brake\'s nominal ratio less what the site\'s path takes, ft (+ high)')
        c.fn('ap/eo-site-mid-ft', D(M(reach, v(0.5), S(r_nom, p('ap/eo-brake-min-ratio'))), need),
             desc='what the path leaves at the flaperons\' middle, between the nominal and the full, ft')
        c.fn('ap/eo-site-path-ft', IFTHEN(is_(3), S(M(a3, rc), d3), IFTHEN(is_(2), S(M(tha_left, R), Lf), S(s0_, M(a1_, R0), L1, M(thT, R), Lf))),
             desc='the path to the site\'s point over the ground, ft')
        # S-turns: against the circle's turn (the turn back to the leg then runs on into the circle's, without a roll
        # reversal: away from the circle's side, with its reversal at 5 deg/s an S-turn ended 50 deg banked the wrong
        # way, and the aircraft rolled through into the circle), by the angle that spends the margin by the circle, no
        # more than the leg left still has room to turn back from: that turn on the circle's radius, and the roll out of
        # it and into the circle as delays
        # (spent over the leg before the turn back's room, not the whole leg: over the whole the turn back's own cost
        # in the margin held it short, and 1600-2000 ft were still there on the final; over its first half, the second
        # left for the margin's errors, a straight-in from 4000 ft overspent and was 6700 ft off the line at the flare)
        room = MAX(v(1), D(L1, S(M(v(2), vgt, dly), M(R, p('ap/eo-site-sturn-rad')))))
        # (none on a committed path: the nacelle brake spends it on the path itself. Sized to the surplus they spiralled round
        # the entry point at the brake's speed, the turn so much tighter than the bearing's swing, and never joined)
        dS = c.fn('ap/eo-site-sturn-rad', IFTHEN(GT(M(is_(1), GT(mg, v(0)), LT(a1_, lead_a), LT(s0_, v(1))), v(0.5)),
                                                 IFTHEN(cm, v(0), MIN(tag('acos', Q(room, S(room, mg))), MAX(v(0), Q(D(L1, M(v(2), vgt, dly)), R)))), v(0)),
                  desc='the S-turn away from the leg, rad')
        look = M(MAX(p('velocities/vt-fps'), v(100)), p('ap/nav-look-s'))
        # (straight on, the teardrop's way out, until the roll in's lead; in the turn its own way, the long way round
        # too when the path's so; on the leg onto its line)
        e1 = c.fn('ap/eo-site-e1-rad', IFTHEN(GT(s0_, M(vgt, dly)), v(0), IFTHEN(GT(a1_, lead_a), M(s1_, MIN(a1_, v(math.pi))), wrap(D(D(chi1, M(sd, dS)), trk)))))
        e2 = c.fn('ap/eo-site-e2-raw', D(S(cc, lam, M(sd, S(v(math.pi / 2), ATAN2(D(dd, R), look)))), trk))
        e2 = c.fn('ap/eo-site-e2-rad', wrap(e2))
        # onto the circle the roll in's lead before it; or, closing on it as a pursuit of the tangent point (which goes
        # round ahead of the aircraft: one held 300 ft outside it, the tangent point 1000-1600 ft on, all the way), within
        # the lead of it and on its track within the lead's turn
        # (a committed path onto the circle only from its leg: passing near the circle in its first turn is not the join)
        # (and at the point, whatever the nose says: that close its bearing swings faster than the aircraft follows, and held to
        # the nose on it, flown 1000 ft from the point on the right heading, it went by and a path 35 000 ft longer was taken)
        to2 = GT(M(is_(1), MAX(M(LT(a1_, lead_a), LT(L1, M(vgt, dly)), LT(s0_, v(1))),
                               M(LT(D(dd, R), M(vgt, dly)), LT(ABS(e2), w_out), MAX(D(v(1), cm), p('ap/eo-site-leg'))),
                               M(cm, LT(dT, M(v(1.5), vgt, dly)), LT(s0_, v(1))))), v(0.5))
        # (the final taken where the turn onto it has to start, from the turn it can make at the speed it is flying: the path's circle reads
        # nothing to go once the aircraft is inside it, and a first turn tighter than the circle puts it there. Over LOWI at 47 500 lb it eased
        # from 48 to 30 deg of bank on the way to a join already gone, took the final 413 ft from the line with 51 deg to turn, where that takes
        # 1450 ft, and crossed it by 1000)
        lead_y = c.fn('ap/eo-site-lead-y-ft', S(M(rc, D(v(1), COS(dpsi))), M(vg_, dly, ABS(SIN(dpsi)))), desc='the distance off the line the turn onto it starts at, ft')
        cap = c.fn('ap/eo-site-capture', GT(M(MAX(is_(1), is_(2)), GT(M(y, dpsi), v(0)), LT(ABS(dpsi), v(2)), LT(ABS(y), lead_y), LT(NEG(x), S(Lf, M(rc, ABS(SIN(dpsi))))), LT(S(x, M(rc, ABS(SIN(dpsi)))), v(0)),
                                              LT(p('ap/eo-site-margin-ft'), M(lap, v(0.5)))), v(0.5)), desc='the turn onto the final starts')
        c.fn('ap/eo-site-tha-next', tha, out='ap/eo-site-tha-prev')
        live = M(on('ap/eo-site-on'), eo)
        c.fn('ap/eo-site-state-next', IFTHEN(LT(live, v(0.5)), v(0), IFTHEN(new, v(1), IFTHEN(cap, v(3), IFTHEN(to2, v(2), IFTHEN(to3, v(3), st))))),
             out='ap/eo-site-state', desc='ENG OUT to its site: 1 to the circle, 2 on it, 3 on the final')
        c.fn('ap/eo-site-new-next', v(0), out='ap/eo-site-new')
        # (the final's own line tracking: on the cruise's 20 s look a line 1000 ft off asked 8 deg back, and of the 17 s left half the error
        # was still there at the wheels. With the heading's lag the two are one second-order loop, damped by half the root of their ratio)
        c.declare('ap/eo-final-look-s', 10.0)
        c.declare('ap/eo-final-tau-s', 5.0)
        look3 = M(MAX(p('velocities/vt-fps'), v(100)), p('ap/eo-final-look-s'))
        kpsi3 = Q(MAX(p('velocities/vt-fps'), v(60)), M(v(G0), p('ap/eo-final-tau-s')))
        e3 = c.fn('ap/eo-site-e3-raw', D(S(cc, ATAN2(NEG(y), look3)), trk))
        e3 = c.fn('ap/eo-site-e3-rad', wrap(e3))
        c.fn('ap/eo-site-trk-deg', M(v(R2D), IFTHEN(GT(MAX(flare, is_(3)), v(0.5)), S(trk, e3), IFTHEN(is_(2), S(trk, e2), S(trk, e1)))),
             desc='the track to the site, deg')
        ff = M(sd, ATAN2(SQR(vg_), M(v(G0), R)))
        # (on the circle never banked against its turn: joined 110 deg off its track, the shorter way round to it was
        # the other way, and the aircraft cut into the circle)
        arc_bank = M(sd, MAX(v(0), M(sd, S(ff, M(kpsi, e2)))))
        # (the first turn at its planned radius whatever the speed: slower, on the nacelle brake, it turned far tighter than the
        # path, the nose came onto the entry point early, and the aircraft arrived there 90 deg across the final, a final
        # of 800-2000 ft, and touched 2000 ft to the side of it on a slope)
        c.declare('ap/eo-site-r0-cmd-ft', 0)
        # (in the fall the heading's own bank: at its planned radius, its way over the ground a third of its speed, it asked 5-12 deg, and the
        # turn was left to the glide, 45 deg at 180 KCAS and 3400 ft of margin gone)
        b1 = IFTHEN(LT(fall, v(0.999)), M(p('ap/eo-bank-lim-deg'), v(1 / R2D)),
                    ATAN2(SQR(vg_), M(v(G0), IFTHEN(GT(p('ap/eo-site-r0-cmd-ft'), v(1)), p('ap/eo-site-r0-cmd-ft'), R0))))
        bank1 = IFTHEN(GT(M(cm, GT(a1_, lead_a)), v(0.5)), MAX(NEG(b1), MIN(b1, M(kpsi, e1))), M(kpsi, e1))
        bank = IFTHEN(GT(MAX(flare, is_(3)), v(0.5)), M(kpsi3, e3), IFTHEN(is_(2), arc_bank, bank1))
        # The pedals, low on the final and through the rollout. On the final they help the bank back to the line, the crab left alone. The crab goes
        # at the last, as a glider pilot takes it, the wheels then holding the direction: from ap/eo-decrab-s before them to ap/eo-decrab-end-s, by the time
        # to go at the descent of the moment, the nose goes onto the track. In 15 kt across, 6-2 s stopped 39 ft off the line, 8-3 s 86 ft, 3-0.5 s 4 ft. The pedals straight to what
        # that sideslip takes (ap/eo-ped-beta-rad-in, deadstick.py --xwind), a PI on the heading round that, rate limited. The roll axis is too slow
        # for them alone: wings commanded level, 8-15 deg went on before it caught up, so the lateral stick the slip's trims take against them.
        # On the ground the same law, the nose wheel with it (controls())
        for n, x_ in (('ped-on', 1.0), ('ped-kp', 10.0), ('ped-ki', 5.0), ('ped-roll', 0.3), ('ped-rate-ips', 3.0), ('decrab-s', 3.0), ('decrab-end-s', 0.5)):
            c.declare('ap/eo-' + n, x_)
        c.declare('ap/eo-ped-int-in', 0.0)
        c.declare('ap/eo-decrab-w', 0.0)
        a_ped = c.fn('ap/eo-ped-active', M(p('ap/eo-ped-on'), live, GT(MAX(flare, is_(3)), v(0.5)), T1('hh/h-wheels-ft', [0.8 * EO_PED_TOP_FT, EO_PED_TOP_FT], [1.0, 0.0])), desc='how far the pedals are in')
        tgo = c.fn('ap/eo-tgo-s', Q(p('hh/h-wheels-ft'), MAX(v(5), NEG(p('velocities/h-dot-fps')))), desc='the time to the wheels at this descent, s')
        w_now = MIN(v(1), MAX(v(0), Q(D(p('ap/eo-decrab-s'), tgo), D(p('ap/eo-decrab-s'), p('ap/eo-decrab-end-s')))))
        wd = c.fn('ap/eo-decrab-next', M(GT(a_ped, v(0.01)), MAX(p('ap/eo-decrab-w'), w_now)), out='ap/eo-decrab-w')
        # (the nose on the track, wherever that is by then: straight when a wheel touches. From there the pedals hold on through the rollout, to the
        # line's course and a little back to it, the weathercock's pull into the wind in the PI's integral, the nose wheel coming in with the speed)
        back = v(3 / R2D)
        on_wheels = GT(S(*[p('gear/unit[%d]/WOW' % i) for i in range(3)]), v(0.5))
        # (and in the air not all the way to the track: ap/eo-decrab-keep of the wind's correction left in, just enough against the drift of the last
        # seconds, nothing else holding the aircraft against the wind by then)
        c.declare('ap/eo-decrab-keep', 0.0)
        wca = wrap(D(S(p('attitude/psi-rad'), p('aero/beta-rad')), trk))
        psi_f = IFTHEN(on_wheels, S(cc, MAX(NEG(back), MIN(back, ATAN2(NEG(y), look3)))), S(trk, M(p('ap/eo-decrab-keep'), wca)))
        e_land = wrap(D(psi_f, p('attitude/psi-rad')))
        e_psi = c.fn('ap/eo-ped-err-rad', S(M(D(v(1), wd), e3), M(wd, e_land)), desc='the nose from where the pedals want it, rad')
        nose_up = LT(p('gear/unit[0]/WOW'), v(0.5))
        ffp = M(wd, nose_up, NEG(Q(wrap(D(S(p('attitude/psi-rad'), p('aero/beta-rad')), psi_f)), p('ap/eo-ped-beta-rad-in'))))
        raw = S(ffp, M(p('ap/eo-ped-kp'), e_psi), p('ap/eo-ped-int-in'))
        c.fn('ap/eo-ped-int-next', M(a_ped, MAX(v(-B.X_PED), MIN(v(B.X_PED), S(p('ap/eo-ped-int-in'), M(p('ap/eo-ped-ki'), e_psi, p('simulation/channel-dt'), LT(ABS(raw), v(B.X_PED))))))), out='ap/eo-ped-int-in')
        pstep = M(p('ap/eo-ped-rate-ips'), p('simulation/channel-dt'))
        tgt = M(a_ped, MAX(v(-B.X_PED), MIN(v(B.X_PED), raw)))
        ped_ = c.fn('ap/eo-ped-next', S(p('ap/eo-ped-in'), MAX(NEG(pstep), MIN(pstep, D(tgt, p('ap/eo-ped-in'))))), out='ap/eo-ped-in')
        # (less as the nacelles come up: the stick's power over the roll grows with them. At full pedal, 45 deg of nacelle and 143 KCAS the trim's 0.3 took
        # it to 9 deg of bank against -4 asked. Estimated: the slip's trims are at nacelles 0)
        # (and in the last seconds the stick straight from the bank's error, the cross control as much as the pedals turn out to need: the roll axis's
        # gains fall with the airspeed, and with 13 deg of error it hardly moved. Until the nose wheel is down, on the mains still free to roll)
        for n, x_ in (('roll-k-in-deg', 0.4), ('roll-d-in-dps', 0.15)):
            c.declare('ap/eo-' + n, x_)
        roll_cl = M(wd, nose_up, MAX(v(-0.5 * B.X_LAT), MIN(v(0.5 * B.X_LAT), D(M(p('ap/eo-roll-k-in-deg'), D(p('ap/eo-phi-cmd-deg'), p('attitude/phi-deg'))), M(p('ap/eo-roll-d-in-dps'), p('velocities/p-rad_sec'), v(R2D))))))
        # (on the wheels the stick into the wind against the pedals, wings held level, as the A320 model's rollout has it: less with the speed, the
        # surfaces' power going)
        for n, x_ in (('ail-ped', 0.5), ('ail-bank', 0.2)):
            c.declare('ap/eo-' + n, x_)
        ail = M(on_wheels, T1('velocities/vc-kts', [80.0, 140.0], [0.1, 1.0]), v(B.X_LAT),
                MAX(v(-0.5), MIN(v(0.5), S(NEG(M(p('ap/eo-ail-ped'), Q(ped_, v(B.X_PED)))), NEG(M(p('ap/eo-ail-bank'), p('attitude/phi-deg')))))))
        # (no stick on the wheels: at 47 500 lb half aft saved 91 ft for 9 deg of pitch past the tail's limit and 2 s off the mains, half forward cost 110 ft)
        c.fn('ap/eo-lat-next', S(roll_cl, M(a_ped, ail), NEG(M(p('ap/eo-ped-roll'), D(v(1), wd), T1('ctl/nacelle-deg', [0.0, 45.0], [1.0, 0.3]), Q(ped_, v(B.X_PED)), v(B.X_LAT), LT(S(*[p('gear/unit[%d]/WOW' % i) for i in range(3)]), v(0.5))))), out='ap/eo-lat-in')
        return dict(on=GT(M(live, GT(st, v(0.5))), v(0.5)), bank=bank)

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
            for st in ('bc', 'bs', 'bcd', 'bsd', 'v0-fps', 'vrs-dv'):
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
        VTu = s.vrs(side, ph, Xs, WR, VT, Vhz, Vip)
        for st, tg, kind in ((PH_COS, 'c', 'c'), (PH_SIN, 's', 's')):
            Li = [np.linalg.inv(PH.Lmat(st, x, kind)) for x in Xs]
            for i in range(len(st)):
                for kk in range(len(st)):
                    Vk = VTu if st[kk] == (1, 0) else p(b_ + '/ph/V')
                    ph.fn(b_ + '/ph/A%s%d-%d' % (tg, i, kk), M(T1(b_ + '/ph/X', Xs, [float(L_[i, kk]) for L_ in Li]), Vk))
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

    def vrs(s, side, ph, Xs, WR, VT, Vhz, Vip):
        """Vortex ring state: Johnson's model of the mean inflow [TP-2005-213477] (vrs.py). Where it departs from
        momentum theory, in descent below 0.2 vh and slower than 0.95 vh edgewise, the uniform wake state's mass flow
        parameter is set for his steady inflow, vh / nu (Vz / vh, Vx / vh), in place of momentum theory's
        sqrt(Vx^2 + (Vz + v)^2) (Vprime). vh is the wake's own: its steady uniform inflow times its mass flow, the
        uniform state's row of L times the forcing, so that its momentum theory is Glauert's in Vz / vh and Vx / vh
        exactly (Vx with the mass flow's 5 ft/s floor) and Johnson's joins it at the region's edges. The wake states keep
        their own dynamics: while his total flow is down through the disc the mass flow is momentum theory's on the
        present inflow, offset to his steady state; his increment for the vortex ring follows with his time lag,
        tau = 0.7 / lambda_h revolutions, lambda_h the thrust's ideal. Each rotor on its own axial and edgewise velocity.
        Returns the uniform state's mass flow parameter."""
        b_, st_, w = 'aero/rotor/%s' % side, 'rotor/%s' % side, 'aero/rotor/%s/vrs/' % side
        cd, sd = p(b_ + '/ph/cd'), p(b_ + '/ph/sd')
        Lc = [PH.Lmat(PH_COS, x, 'c') for x in Xs]
        terms = []
        for kk, (j, rh) in enumerate(PH_COS):
            tau = p(b_ + '/ph/tau-c%d' % kk)
            if rh:          # in the wake's frame, as inflow_states turns it
                tau = S(M(tau, cd), M(p(b_ + '/ph/tau-s%d' % PH_SIN.index((j, rh))), sd))
            terms.append(M(T1(b_ + '/ph/X', Xs, [float(L_[0, kk]) for L_ in Lc]), tau))
        vh = ph.fn(w + 'vh-fps', M(WR, SQ(MAX(v(1e-8), M(v(0.5 * math.sqrt(3)), S(*terms))))),
                   desc='hover induced velocity of the forcing: the steady uniform inflow times its mass flow, ft/s')
        ph.fn(w + 'z', Q(NEG(Vhz), vh), desc='axial velocity (climb +) over vh')
        ph.fn(w + 'x', Q(SQ(S(SQR(Vip), v(25.0))), vh), desc='edgewise velocity over vh, with the mass flow\'s floor')
        ph.fn(w + 's', tag('pow', MAX(v(0), D(v(1), SQR(Q(p(w + 'x'), v(VRS.XM))))), v(0.2)), desc='the tables\' coordinate across (vrs.py)')
        xs, zs = VRS.edge_table()
        inside = ph.fn(w + 'in', M(LT(p(w + 'z'), v(VRS.ZD)), LT(p(w + 'x'), v(VRS.XM)), GT(p(w + 'z'), T1(w + 'x', xs, zs))),
                       desc='in the region where Johnson\'s model departs from momentum theory')
        nb, dn = VRS.tables()
        nub = ph.fn(w + 'nu-base', T2(w + 'z', w + 's', VRS.TAB_Z, VRS.TAB_S, nb), desc='the baseline\'s v / vh')
        dnu = ph.fn(w + 'dnu', T2(w + 'z', w + 's', VRS.TAB_Z, VRS.TAB_S, dn), desc='the vortex ring\'s increment of v / vh')
        dv = p(st_ + '/vrs-dv')
        # Johnson's mass flow v_h / nu; while his total flow is down through the disc, momentum theory's form, on the present
        # inflow, offset to his (the wake's own dynamics, and at the region's edges exactly momentum theory's); past ideal
        # autorotation, where that form has no stable state, his own (the two meet there, with the same slope)
        nu = ph.fn(w + 'nu', S(nub, Q(dv, vh)), desc='Johnson\'s v / vh with the lagged vortex ring')
        Sj = SQ(S(SQR(p(w + 'x')), SQR(S(p(w + 'z'), nu))))
        VTj = IFTHEN(GT(S(p(w + 'z'), nu), v(0)), Q(S(p(b_ + '/Vprime-fps'), M(vh, D(Q(v(1), nu), Sj))), WR), Q(vh, M(WR, nu)))
        VTu = ph.fn(b_ + '/ph/VTu', IFTHEN(inside, VTj, VT), desc='the uniform state\'s mass flow parameter')
        # the lag, 1 / (tau s + 1) stepped semi-implicitly; at a start or a trim's evaluation its steady value
        vhi = SQ(Q(MAX(p(b_ + '/thrust-lbs'), v(1.0)), M(v(2 * math.pi * U.R ** 2), p('atmosphere/rho-slugs_ft3'))))
        k = M(p('simulation/channel-dt'), vhi, v(1 / (VRS.TAU_REV_LH * 2 * math.pi * U.R)))
        tgt = M(inside, vh, dnu)
        ph.fn(st_ + '/vrs-dv-next', IFTHEN(GT(p('ctl/init-fast'), v(0.5)), tgt, Q(S(dv, M(k, tgt)), S(v(1), k))), out=st_ + '/vrs-dv')
        return VTu

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
    def flap_law(s):
        """the flaperons from the flight state, after Bell's wing data (wing_tables, airplane mast): the lift the wing
        makes, as Bell's coefficient at its angle of attack and flap index, brought to 1 g by the load factor, against
        what F1 and F2 (0, 40 deg) can make: their CLmax over ctl/flap-g-margin (1.3: the margin of 1.3 g to it kept in
        flight; the approach's 1.3 Vs, 1.69, asked full flap below 200 KEAS at 47 500 lb, F2's CLmax only a fifth over
        F1's, at 700-1650 hp); the least deflection, interpolated between them, that makes it, and makes it at no more
        than AUTO NAC's pitch in level flight (AUTONAC_PITCH), but in ENG OUT's flare.
        At a given lift the least flap always has the least drag in Bell's data (F1 wherever it reaches it, to CL 1.45),
        so flap is only lift at a lower angle of attack (at 6 deg: 0.73, 1.36, 1.57)"""
        c = s.ctl
        al, fx_cols, bm_tabs, CL, CD = s.wing_tables()
        cl90 = CL[1]
        c.declare('ctl/flap-g-margin', 1.3)
        clt = T2('aero/wing/alpha-deg', 'aero/wing/flap-index', al, fx_cols, cl90)
        need = c.fn('ctl/flap-cl-1g', Q(clt, MAX(v(0.5), MIN(v(2), p('accelerations/Nz')))),
                    desc='the wing\'s lift coefficient (Bell\'s at its angle of attack and flap) brought to 1 g')
        flare = GT(M(GT(p('/instrumentation/mv22b/afcs/eng-out'), v(0.5)), GT(p('ap/eo-phase'), v(2.5)), LT(p('ap/eo-phase'), v(4.5))), v(0.5))
        lims = []
        for x in (0, 1):
            j = fx_cols.index(x)
            ys = [row[j] for row in cl90]
            clmax = max(y for a_, y in zip(al, ys) if -10 <= a_ <= 30)
            lims.append(Q(v(clmax), p('ctl/flap-g-margin')))
        L0, L1 = lims
        # (F3's CLmax is F2's: for the stall's margin it adds nothing, and past F2 the need jumped to it, its trim's tail
        # download asking the more lift that held it there, 67 deg at -2 deg and 11 000 hp at 190 KCAS: F2 the most. In
        # ENG OUT's flare F3 at the tail's angle of attack stalled the wing as the speed fell, 36 to 67 deg in 2 s: the
        # flare's own, ap/eo-flap-deg, there instead)
        fx_m = IFTHEN(LT(need, L0), v(0), IFTHEN(LT(need, L1), Q(D(need, L0), MAX(D(L1, L0), v(0.01))), v(1)))
        # (and the lift at no more than AUTO NAC's pitch in level flight: the wing's angle where the body's angle of
        # attack, the level flight's pitch, is at it, and the deflection that makes the lift there. With the least drag
        # alone the pitch rose 1.5-2.5 deg at each nacelle angle, AUTO NAC held the nacelles 5-20 deg higher at 130-190 kt,
        # and its path took up to 1770 hp more than the earlier schedule's: the least drag for the wing is not the least
        # power for the aircraft. In ENG OUT's glide too: there its flaps cost reach, at 200 KCAS 8.06 against 9.43
        # with the least drag at 60 500 lb, 8.74 against 9.07 at 54 000 (deadstick.py --table), but the flares landed
        # better, 210 of deadland.py's 243 intact against 186, the heavy aircraft 4-5 kt slower (its steeper glide, the
        # flare's path fitted to the descent, floated longer). Not in its flare, whose pitch is its own)
        alim = c.fn('ctl/flap-alpha-lim-deg', S(p('aero/wing/alpha-deg'), D(T1('velocities/vc-kts', *AUTONAC_PITCH), p('aero/alpha-deg'))),
                    desc='the wing\'s angle of attack at AUTO NAC\'s pitch in level flight, deg')
        c0, c1 = [T1('ctl/flap-alpha-lim-deg', al, [row[fx_cols.index(x)] for row in cl90]) for x in (0, 1)]
        fx_p = M(D(v(1), flare), MAX(v(0), MIN(v(1), Q(D(need, c0), MAX(D(c1, c0), v(0.01))))))
        c.fn('ctl/flap-index-law', MAX(fx_m, fx_p), desc='the least of Bell\'s flap indices (0 F1, 1 F2) that makes it')
        on_ = M(LT(S(*[p('gear/unit[%d]/WOW' % i) for i in range(3)]), v(0.5)), GT(p('aero/qbar-psf'), v(20)))
        # (ENG OUT's own, ap/eo-flap-deg, where it is more: its flare's, and its brake before it)
        return c.fn('ctl/flap-law-deg', M(on_, MAX(T1('ctl/flap-index-law', [0, 1], [0, 40]), p('ap/eo-flap-deg'))),
                    desc='the flaperons the lift needs, deg')

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
        # what the rotors carry, for the sound's load grids (fdm/rotorsound.py): their volumes, and their conditions, which
        # FlightGear compares with properties only (a loop playing holds one of its 128 sources, silent or not)
        t_sum = S(p('aero/rotor/r/thrust-lbs'), p('aero/rotor/l/thrust-lbs'))
        o.fn('out/rotor-thrust-lbf', M(v(0.5), t_sum), out='/rotors/main/thrust-lbf', desc='the two rotors\' mean thrust')
        o.fn('out/rotor-thrust-weight', Q(t_sum, p('inertia/weight-lbs')), out='/rotors/main/thrust-over-weight', desc='their thrust over the weight')
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
        sl = slap_table() if U.NAME == 'V-22' else None
        if sl:
            # the blade slap's loudness for the sound (Sounds/v22-sound.xml's slap loops; data/slap_v22.json): PyWopWop's
            # map of it over the advance ratio and the inflow through the tip-path plane, which set where the tip
            # vortices pass the blades. Each rotor's kinematics in the plane normal to its force (PyWopWop's trim puts
            # the tip-path plane there): its advance ratio and upflow; the inflow PyWopWop's at them, scaled with the
            # thrust (the map's levels come from its wake at its own inflow: this model's runs 10-15 % higher at low
            # speed); the vortices' strength with the thrust; the two rotors' sound power added. Lagged so that the
            # blade passages' loads stay out of it
            o.channel('rotor sound', execrate=SLOW_EXECRATE)
            WR = MAX(M(W, v(U.R)), v(30))
            g2 = {'ext': [], 'int': []}
            for side, m in SIDES:
                b_, o_ = 'aero/rotor/%s' % side, 'out/%s/slap' % side
                F = [p(b_ + '/F%s-s' % a) for a in 'xyz']
                Vh = [p(b_ + '/Vh%s-fps' % a) for a in 'xyz']
                Fm = o.fn(o_ + '-F-lbs', MAX(SQ(S(*[SQR(f) for f in F])), v(1.0)), desc='the rotor\'s force')
                vn = o.fn(o_ + '-Vn-fps', Q(S(*[M(a, f) for a, f in zip(Vh, F)]), Fm),
                          desc='the hub\'s velocity along the force: the air\'s through the tip-path plane, into the wake +')
                o.fn(o_ + '-muz-now', NEG(Q(vn, WR)), desc='upflow through the tip-path plane over the tip speed')
                o.fn(o_ + '-mu-now', Q(SQ(MAX(v(0), D(S(*[SQR(a) for a in Vh]), SQR(vn)))), WR))
                muz = o.lag(o_ + '-muz', o_ + '-muz-now', SLAP_TAU_S)
                o.lag(o_ + '-mu', o_ + '-mu-now', SLAP_TAU_S)
                T = o.lag(o_ + '-T-lbs', o_ + '-F-lbs', SLAP_TAU_S)
                l0 = T2(o_ + '-muz', o_ + '-mu', sl['mu_z'], sl['mu'], sl['lam0'])
                o.fn(o_ + '-lam', D(M(l0, Q(T, v(sl['thrust_ref_lbf']))), muz), desc='inflow through the tip-path plane')
                for k in g2:
                    a = o.fn('%s-%s' % (o_, k), M(T2(o_ + '-lam', o_ + '-mu', sl['lam'], sl['mu'], sl[k]),
                                                  Q(T, v(sl['thrust_ref_lbf']))))
                    g2[k].append(SQR(a))
            o.fn('out/slap-ext', SQ(M(v(0.5), S(*g2['ext']))), out='/rotors/main/slap-ext',
                 desc='the slap loops\' amplitude outside, 1 at the bank\'s')
            o.fn('out/slap-int', SQ(M(v(0.5), S(*g2['int']))), out='/rotors/main/slap-int',
                 desc='the slap loops\' amplitude in the cockpit, 1 at the bank\'s')
        o.channel('gear', execrate=SLOW_EXECRATE)
        # the landing gear's retraction and extension in GEAR_S
        o.raw('<actuator name="gear/gear-actuator"><input>gear/gear-cmd-norm</input><rate_limit>%g</rate_limit>'
              '<output>gear/gear-pos-norm</output></actuator>' % (1 / GEAR_S))
        # the wheels' height above the ground (YASim's property, which the rotor wash places itself by), level
        o.fn('out/gear-agl', MAX(v(0), M(v(0.3048), D(p('position/h-agl-ft'), Q(D(p('inertia/cg-z-in'), v(U.MAIN_GEAR[2])), v(12))))),
             out='/position/gear-agl-m')
        # Each leg's touchdown, for its tyres' squeal and its strut's thump (the sound's one-shots: PyWopWop's tyre.py). A
        # wheel meets the runway turning slower than the ground goes by and slides till friction spins it up, losing
        # I (slip / r)^2 / 2: so each wheel's spin, as its rim's speed, spun up on the ground by the contact's sliding friction
        # (0.5) on its share of the strut's force (spring and damping as the contact's; two wheels a leg), rolling at the
        # ground speed, spun down in the air by its bearings and brake pads, a main wheel's brake stopping it (U.TYRES).
        # Then each leg's sink rate and slip (the ground speed less its rims') as its weight comes on, held till it lifts:
        # a wheel that hops touches down still turning and slides less. Its own speed downwards: the aircraft's, with its
        # roll and pitch about the centre of gravity, so a nose that comes down by pitching counts. Every frame: a contact
        # comes in one
        o.channel('touchdown')
        kt = M(p('velocities/vg-fps'), v(1 / 1.6878))                   # the ground speed, kt
        for n, (name, (fs, bl, wl), k, c, steer) in enumerate(struts()):
            g = 'gear/unit[%d]/' % n
            w = U.TYRES['nose' if steer else 'main']
            dx = Q(D(p('inertia/cg-x-in'), v(fs)), v(12.0))            # ahead of the centre of gravity, ft
            dy = Q(D(v(bl), p('inertia/cg-y-in')), v(12.0))            # right of it
            o.fn(g + 'sink-now-fps', S(p('velocities/v-down-fps'), M(p('velocities/p-rad_sec'), dy), NEG(M(p('velocities/q-rad_sec'), dx))),
                 desc='the leg\'s own speed downwards')
            down = GT(p(g + 'WOW'), v(0.5))
            load = MAX(v(0), S(M(v(k), p(g + 'compression-ft')), M(v(c), p(g + 'compression-velocity-fps'))))    # lbf, the leg's
            up = 0.5 * 4.4482216 / w['wheels'] * w['radius_m'] ** 2 / w['inertia_kgm2'] / 0.514444   # kt/s of rim speed a lbf on the leg gives
            spin_down = w['radius_m'] / w['inertia_kgm2'] / 0.514444                                   # kt/s a N m takes away
            torque = v(w['drag_nm'])
            if not steer:
                side = 'left' if 'LEFT' in name else 'right'
                torque = S(torque, M(MAX(p('fcs/%s-brake-cmd-norm' % side), p('/controls/gear/brake-parking')), v(w['brake_nm'])))
            o.fn(g + 'slip-now-kt', MAX(v(0), D(kt, p(g + 'spin-kt'))), desc='the ground speed less the rims\'')
            o.fn(g + 'spin-kt', IFTHEN(down, MIN(kt, S(p(g + 'spin-kt'), M(v(up), load, p('simulation/channel-dt')))),
                                       MAX(v(0), D(p(g + 'spin-kt'), M(v(spin_down), torque, p('simulation/channel-dt'))))),
                 desc='the rims\' speed: spun up by the runway, rolling, spinning down in the air')
            o.fn(g + 'touchdown-sink-fps', IFTHEN(down, p(g + 'touchdown-sink-fps'), p(g + 'sink-now-fps')), desc='as the weight came on, held')
            o.fn(g + 'touchdown-slip-kt', IFTHEN(down, p(g + 'touchdown-slip-kt'), p(g + 'slip-now-kt')), desc='likewise')
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
        if U.NAME == 'V-22':
            # the crew alerts' conditions (alerts.py), for the cockpit's alert manager (Nasal/alerts.nas)
            alerts.channel(o, U, SLOW_EXECRATE, (GE_K, GE_EXP))
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


def struts(weight=None):
    """the gear's contacts, gear/unit[0..2] in order: name, station (FS, BL, WL in), spring (lbf/ft), damping (lbf s/ft),
    steering (deg). Tricycle gear: about 0.5 ft static deflection, 0.5 of critical damping (estimate), at the weight the
    flight model is built for"""
    wt = weight or U.GW_DESIGN
    out = []
    for name, at, share, steer in (('NOSE', U.NOSE_GEAR, 0.16, 70), ('LEFT_MAIN', (U.MAIN_GEAR[0], -U.MAIN_GEAR[1], U.MAIN_GEAR[2]), 0.42, 0),
                                   ('RIGHT_MAIN', U.MAIN_GEAR, 0.42, 0)):
        k = share * wt / 0.5
        out.append((name, at, k, 2 * 0.5 * math.sqrt(k * share * wt / G0), steer))
    return out


def fdm_file(cg=None, weight=None, inertia=None):
    mc = mass_config(cg, weight, inertia)
    gear = []
    for name, (fs, bl, wl), k, c, steer in struts(weight):
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
        texts[pre + '-alerts.xml'] = alerts.data(U)
    for name, t in texts.items():
        open(os.path.join(out, 'Systems', name), 'w').write('<?xml version="1.0"?>\n' + t)
    open(os.path.join(out, pre + '.xml'), 'w').write(fdm_file())
    return len(mdl.g.items)


def display_data():
    """what the cockpit's flight display (Nasal/displays.nas) draws from the model's data: the conversion
    protection's boundaries, the airplane mode's limits, the drive's limit, the tail's strike; and AUTO NAC's schedule,
    for v22.nas's in-air start (the power to hover is the alert channel's, alerts.py)"""
    (lv, ln), (uv, un) = U.CPS
    sk, sn = autonac_schedule()
    row = lambda tag, xs: ''.join('<%s>%g</%s>' % (tag, x, tag) for x in xs)
    return ('<!-- generated by fdm/genv22.py: the flight display\'s data (Nasal/displays.nas) -->\n<PropertyList>\n'
            '  <!-- conversion protection: equivalent airspeed (kt) and the least, the most nacelle angle (deg) -->\n'
            '  <cps>\n    <lower>%s%s</lower>\n    <upper>%s%s</upper>\n  </cps>\n'
            '  <!-- airplane mode\'s airspeed limit (KCAS, Mach) at 84 %% rotor speed, and at 100 %% -->\n'
            '  <apln><vmo-kt>%g</vmo-kt><mmo>%g</mmo><vmo-100-kt>%g</vmo-100-kt></apln>\n'
            '  <!-- the drive\'s torque limit at 100 %% rotor speed (the power bar\'s while the rotors are stopped) -->\n'
            '  <hover>\n    <power-hp>%g</power-hp>\n  </hover>\n'
            '  <!-- AUTO NAC\'s schedule: the nacelle angle (deg) for the calibrated airspeed (kt). An in-air start\'s\n'
            '       nacelles from the flight model\'s first step; v22.nas draws the 3-D model\'s at it before that -->\n'
            '  <autonac>%s%s</autonac>\n'
            '  <!-- the pitch at which the fuselage meets the ground on the wheels (ENG OUT\'s flare), deg -->\n'
            '  <tail-strike-deg>%g</tail-strike-deg>\n%s</PropertyList>\n'
            % (row('kt', lv), row('nac', ln), row('kt', uv), row('nac', un), U.VMO_APLN, U.MMO_APLN, U.VMO_APLN_100,
               U.P_XMSN_100, row('kt', sk), row('nac', sn), U.TAIL_STRIKE_DEG, eo_site_data()))


def eo_site_data():
    """ENG OUT's landing site's needs for Nasal/eosite.nas (deadland.py site, siteguide): over the gross weight (w, lb),
    from the flare's start: the wheels under 50 ft (h50), on the ground (td, at td-kt), the run on the ground to a stop, braked
    (roll-*, by FlightGear's ground: pavement, grass, crop), ft; the reach's error near its end (err-lo, -hi, ft, +
    long: the aircraft short); the guidance's scatter of the flare's start about its point (flare-lo, -hi, ft); the
    turns' cost (deadland.py turns): turn-180, ft of height per 180 deg, a row of the banks turn-bank a weight"""
    d = eo_site() if U.NAME == 'V-22' else None
    if not d:
        return ''
    row = lambda tag, xs: ''.join('<%s>%g</%s>' % (tag, x, tag) for x in xs)
    return ('  <!-- ENG OUT\'s landing site (Nasal/eosite.nas): what the landing takes, by the gross weight -->\n  <eo-site>%s%s%s%s%s%s%s%s'
            '<flare-lo>%g</flare-lo><flare-hi>%g</flare-hi>%s%s</eo-site>\n'
            % (row('w', d['weight_lb']), row('h50', d['gate_to_50ft_ft']), row('td', d['gate_to_touchdown_ft']),
               row('roll-pavement', d['rollout_ft']['pavement']), row('roll-grass', d['rollout_ft']['grass']),
               row('roll-crop', d['rollout_ft']['crop']), row('err-lo', [e[0] for e in d['reach_err_ft']]),
               row('err-hi', [e[1] for e in d['reach_err_ft']]) + row('td-kt', d['touchdown_kt']), d['flare_err_ft'][0], d['flare_err_ft'][1],
               row('turn-bank', d['turn_bank_deg']), row('turn-180', [x for r in d['turn_ft_per_180_bank'] for x in r])))


if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    out = args[0] if args else 'out'
    configure('--xv15' in sys.argv)
    if '--no-inline' in sys.argv:
        INLINE = False
    n = build(out, fg='--fg' in sys.argv)
    print('wrote', out, '-', U.NAME, '-', n, 'aerodynamic functions')
