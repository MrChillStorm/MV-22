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
from data import heyson_ground as HG
import rotorwake

U = None                  # the aircraft's data module, set by configure()
G0 = 32.174
R2D = 180 / math.pi
SIDES = (('r', 1), ('l', -1))
# the rotors, the gimbals and the inflow run at the FDM rate; the airframe, engines and outputs every few frames
AIRFRAME_EXECRATE = 4
SLOW_EXECRATE = 8
# Peters-He states (j, r): three axisymmetric (the hover's radial distribution) and the first harmonic
PH_COS = [(1, 0), (3, 0), (5, 0), (2, 1)]
PH_SIN = [(2, 1)]
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
        'ctl/cps-max-deg', 'gov/theta-deg'}
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
        s.wake_tab = rotorwake.load(U)

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
        c.channel('pilot')
        c.fn('ctl/init-fast', IFTHEN(LT(p('simulation/sim-time-sec'), v(1.0)), v(1), MIN(v(1), MAX(v(0), p('ctl/init')))))
        init = GT(p('ctl/init-fast'), v(0.5))
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
        c.channel('nacelles')
        # the nacelle angle follows its command at the conversion rate; it starts at the command
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
        step = M(v(U.NACELLE_RATE), p('simulation/dt'))
        dn = c.fn('ctl/nacelle-step-deg', IFTHEN(init, D(cmd, nac), MAX(NEG(step), MIN(step, D(cmd, nac)))))
        c.fn('ctl/nacelle-rate-rad_sec', IFTHEN(init, v(0), Q(dn, M(v(R2D), p('simulation/dt')))))
        c.fn('ctl/nacelle-next', S(nac, dn), out='ctl/nacelle-deg')
        bm = c.fn('ctl/beta-m-deg', D(v(90), nac), desc='Bell\'s mast angle, 0 helicopter .. 90 airplane')
        c.fn('ctl/sin-nac', SIN(M(nac, v(1 / R2D))))
        c.fn('ctl/cos-nac', COS(M(nac, v(1 / R2D))))
        c.channel('mixing')
        bmt = lambda ys: T1('ctl/beta-m-deg', B.CTL_BETAM, ys)
        # collective at 0.75 R: Bell's lever schedule is the blade's pitch at the centre with its twist table (40.9 deg
        # root to tip, 6.375 at 0.75 R) [CR B-4, B-38]
        th75 = c.fn('ctl/coll-ff-deg', S(D(bmt(B.TH0_LL), v(40.9 - 6.375)), M(bmt(B.DTH0_DXCOL), v(B.X_COL), tcl)),
                    desc='collective at 0.75 R from the lever, deg')
        c.fn('ctl/theta75-deg', S(th75, p('gov/theta-deg')), desc='both rotors\' collective, with the governor\'s')
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
        c.channel('servos')
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
        c.channel('governor')
        c.declare('gov/int-deg', 0.0)
        c.declare('gov/kp', 0.6)              # deg per % (Bell's 0.1 deg/rpm at 589 rpm: 0.59)
        c.declare('gov/ki', 0.4)              # deg per % s
        c.declare('/controls/rotor/nr-trim-pct', 0.0)
        init = GT(p('ctl/init-fast'), v(0.5))
        # rotor speed reference: airplane mode's below 1 deg of nacelle angle, rate limited (2 %/s)
        c.declare('gov/nr-ref-pct', U.NR_HELO)
        tgt = S(IFTHEN(LT(p('ctl/nacelle-deg'), v(1.0)), v(U.NR_APLN), v(U.NR_HELO)), p('/controls/rotor/nr-trim-pct'))
        st = M(v(2.0), p('simulation/dt'))
        ref = c.fn('gov/nr-ref-next', IFTHEN(init, tgt, S(p('gov/nr-ref-pct'), MAX(NEG(st), MIN(st, D(tgt, p('gov/nr-ref-pct')))))),
                   out='gov/nr-ref-pct')
        nr = M(v(100 / U.OMEGA_100), p('rotor/omega-rad_sec'))
        # the error is the overspeed: a rotor faster than its reference takes more pitch (more torque), a slower one less
        err = c.fn('gov/err-pct', D(nr, ref))
        lo, hi = B.GOV_LIMITS
        # it engages as the rotor comes up to speed (from 19 to 9 % below its reference: 85 to 95 % in helicopter mode),
        # its integral held at zero below: on a start the blades stay at the lever's flat pitch
        ga = c.fn('gov/engaged', MAX(v(0), MIN(v(1), Q(S(D(nr, ref), v(19)), v(10)))))
        c.fn('gov/int-next', IFTHEN(init, p('gov/int-deg'), IFTHEN(LT(ga, v(0.01)), v(0),
                                                                MAX(v(lo), MIN(v(hi), S(p('gov/int-deg'), M(p('gov/ki'), ga, err, p('simulation/dt'))))))),
             out='gov/int-deg')
        c.fn('gov/theta-deg', M(ga, MAX(v(lo), MIN(v(hi), S(p('gov/int-deg'), M(p('gov/kp'), err))))))

    def scas(s):
        """Stability and control augmentation, for the FlightGear aircraft (the validation runs the bare airframe,
        ctl/scas-on = 0). The V-22's fly-by-wire control laws are not public (it holds attitude in all modes [PD]), nor
        are the XV-15 SCAS's tables used: these are proportional-integral loops through the mixing, in inches of stick
        and pedal, with half the stick's and pedals' travel as their authority (the V-22 flies by wire). Rate damping in
        the three axes, for a rate time constant of about 0.5 s in the hover (the hover's control power per inch: pitch
        8.7, roll 8.1, yaw 4.8 deg/s^2, from the test programme's pulse responses), and attitude loops in pitch and roll
        of about 1.2 rad/s. A feed-forward of the level-flight trim's stick over the nacelle angle and the airspeed
        (data/trim_ff_v22.json, from trimmap.py at the design weight and CG) centres the stick's trim through the
        conversion, and an integrator (3 s) takes up the rest of its offset. The response types follow a
        common rotorcraft scheme, attitude command at low speed and rate command above it (as Boeing's US patent
        5,596,499), switched without a transient because both use the same reference: the commanded attitude is the
        reference less the stick (with FlightGear's trim) times scas/{theta,phi}-cmd-deg. Below 50 kt with the nacelles
        above 60 deg, the stick commands an attitude (20 deg of pitch, 25 of bank at full stick; one keyboard step, 1 and
        1.25 deg; its direct path to the controls cut to 30 %, back to full by 60 kt) and the heading is held while the
        pedals rest at their centre; above, the stick commands a rate, the
        reference following the attitude, and the attitude it comes to rest at, back at its centre (within scas/detent),
        is held. On the ground the references follow the aircraft. The gains fall with the dynamic pressure, against the
        surfaces' growing power."""
        c = s.ctl
        c.channel('scas')
        c.declare('scas/kq', 13.0)        # in per rad/s
        c.declare('scas/ktheta', 10.0)    # in per rad
        c.declare('scas/ki-theta', 3.0)   # in per rad s
        c.declare('scas/kp', 14.0)
        c.declare('scas/kphi', 10.0)
        c.declare('scas/ki-phi', 3.0)
        c.declare('scas/kr', 24.0)
        c.declare('scas/kpsi', 10.0)
        c.declare('scas/theta-cmd-deg', 20.0)   # attitude command at full stick
        c.declare('scas/phi-cmd-deg', 25.0)
        c.declare('scas/detent', 0.05)    # stick or pedals within this of their centre: at rest (fraction of travel)
        auth, auth_ped = 0.5 * B.X_LON, 0.5 * B.X_PED
        # gain schedule: 1 in the hover, 0.5 at 30 psf (about 95 kt), 0.15 at 170 psf (230 kt)
        k = c.fn('scas/gain', Q(v(1), S(v(1), Q(p('aero/qbar-psf'), v(30)))))
        c.washout('scas/q-wo', 'velocities/q-rad_sec', 5.0)
        c.washout('scas/p-wo', 'velocities/p-rad_sec', 5.0)
        c.washout('scas/r-wo', 'velocities/r-rad_sec', 3.0)
        init = GT(p('ctl/init-fast'), v(0.5))
        air = c.fn('scas/airborne', M(LT(S(*[p('gear/unit[%d]/WOW' % i) for i in range(3)]), v(0.5)), LT(p('ctl/init-fast'), v(0.5))))
        acah = c.fn('scas/attitude-command', M(LT(p('velocities/vc-kts'), v(50)), GT(p('ctl/nacelle-deg'), v(60))),
                    desc='1: attitude command and heading hold (low speed), 0: rate command')
        # the stick's direct path to the controls, cut to 30 % where it commands an attitude, so that the attitude
        # loop sets the response; back to full by 60 kt and with the nacelles below 50 deg
        low = M(MAX(v(0), MIN(v(1), Q(D(v(60), p('velocities/vc-kts')), v(20)))), MAX(v(0), MIN(v(1), Q(D(p('ctl/nacelle-deg'), v(50)), v(20)))))
        c.fn('scas/stick-ff-calc', D(v(1), M(v(0.7), low)), out='scas/stick-ff')
        # the trim feed-forward: the level-flight trim's stick at the nacelle angle and the airspeed (trimmap.py), so
        # that the stick's centre is trimmed flight through the conversion
        tf = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'trim_ff_v22.json')
        if U.NAME == 'V-22' and os.path.exists(tf):
            tm = json.load(open(tf))
            c.fn('scas/lon-trim-calc', M(v(B.X_LON), T2('ctl/nacelle-deg', 'velocities/vc-kts', tm['nacelle'], tm['kcas'], tm['lon'])),
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
            stk = M(v(sg / R2D), p('scas/%s-cmd-deg' % ax), p(cmd))
            c.fn('scas/%s-ref-next' % ax, IFTHEN(GT(hold, v(0.5)), p(ref), D(p(att), stk)), out=ref)
            err[ax] = c.fn('scas/%s-err-rad' % ax, D(p(att), S(p(ref), stk)), desc='attitude less its command')
            c.fn('scas/%s-int-next' % ax, IFTHEN(init, v(0), IFTHEN(GT(hold, v(0.5)), MAX(v(-auth), MIN(v(auth),
                 S(p(integ), M(k, p('scas/ki-' + ax), err[ax], p('simulation/dt'))))), p(integ))), out=integ)
        # heading hold: at low speed, pedals at rest, in the air
        c.declare('scas/psi-ref-rad', 0.0)
        hh = c.fn('scas/psi-hold', M(air, acah, rest('fcs/rudder-cmd-norm')))
        c.fn('scas/psi-ref-next', IFTHEN(GT(hh, v(0.5)), p('scas/psi-ref-rad'), p('attitude/psi-rad')), out='scas/psi-ref-rad')
        dpsi = D(p('attitude/psi-rad'), p('scas/psi-ref-rad'))
        epsi = c.fn('scas/psi-err-rad', M(hh, ATAN2(SIN(dpsi), COS(dpsi))))
        # signs: + forward stick pitches the nose down, + right stick rolls right, + right pedal yaws right
        c.fn('scas/lon-calc', S(M(k, S(M(p('scas/kq'), p('scas/q-wo')), M(p('scas/ktheta'), err['theta']))), p('scas/theta-int-in')),
             clip=('-%g' % auth, '%g' % auth), out='scas/lon-in')
        c.fn('scas/lat-calc', NEG(S(M(k, S(M(p('scas/kp'), p('scas/p-wo')), M(p('scas/kphi'), err['phi']))), p('scas/phi-int-in'))),
             clip=('-%g' % auth, '%g' % auth), out='scas/lat-in')
        c.fn('scas/ped-calc', NEG(M(k, S(M(p('scas/kr'), p('scas/r-wo')), M(p('scas/kpsi'), epsi)))),
             clip=('-%g' % auth_ped, '%g' % auth_ped), out='scas/ped-in')

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
            s.inflow_states(side)
        r.channel('rotor azimuth')
        psn = S(p('rotor/psi-rad'), M(p('rotor/omega-rad_sec'), p('simulation/dt')))
        r.fn('rotor/psi-next', IFTHEN(GT(psn, v(2 * math.pi)), D(psn, v(2 * math.pi)), psn), out='rotor/psi-rad')

    def inflow_states(s, side):
        """Peters-He finite-state wake (as the UH-60 and Bo 105 models): M a' + L^-1 V a = tau / 2 per state set, in the
        wake frame; the hub-frame harmonic pairs are turned into it, stepped semi-implicitly, and turned back."""
        r = s.rot
        pre, apre = 'rotor/%s/ph' % side, 'aero/rotor/%s/ph' % side
        r.channel('inflow %s' % side)
        h = M(MAX(p('rotor/omega-rad_sec'), v(1.0)), p('simulation/dt'))
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
        # Peters-He: flow parameters and gain matrices against the wake skew X = tan(chi / 2)
        WR = g.fn(b_ + '/ph/omega-r-fps', MAX(M(W, v(U.R)), v(30)))
        X = g.fn(b_ + '/ph/X', Q(SIN(chi), S(v(1), COS(chi))))
        VT = g.fn(b_ + '/ph/VT', Q(p(b_ + '/Vprime-fps'), WR))
        mu, lm, la = Q(Vip, WR), Q(v0, WR), Q(Vn, WR)
        g.fn(b_ + '/ph/V', MAX(Q(v(5), WR), Q(S(SQR(mu), M(S(la, lm), la)), VT)))
        Xs = [0.05 * i for i in range(21)]
        for st, tg, kind in ((PH_COS, 'c', 'c'), (PH_SIN, 's', 's')):
            Li = [np.linalg.inv(PH.Lmat(st, x, kind)) for x in Xs]
            for i in range(len(st)):
                for kk in range(len(st)):
                    Vk = VT if st[kk] == (1, 0) else p(b_ + '/ph/V')
                    g.fn(b_ + '/ph/A%s%d-%d' % (tg, i, kk), M(T1(b_ + '/ph/X', Xs, [float(L_[i, kk]) for L_ in Li]), Vk))
        nw = SQ(S(SQR(Vhx), SQR(Vhy)))
        slow = LT(nw, v(0.5))
        g.fn(b_ + '/ph/cd', IFTHEN(slow, v(1), Q(Vhx, MAX(nw, v(0.5)))))
        g.fn(b_ + '/ph/sd', IFTHEN(slow, v(0), NEG(Q(Vhy, MAX(nw, v(0.5))))))
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
            t0 = g.fn(b + '/Vt0', vht)
            t1 = g.fn(b + '/Vt1', D(M(cb, D(W, rs_a)), M(sb, wer)))
            n0 = g.fn(b + '/Vn0', D(NEG(M(sb, vher)), M(cb, Vhz)))
            n1 = g.fn(b + '/Vn1', D(bdot, wet))
            vr = g.fn(b + '/Vr', S(M(cb, vher), NEG(M(sb, Vhz))))
            # induced velocity of each Peters-He shape at the blade
            trig = {1: (c, sn)}
            tf = lambda tg, rh: v(1) if rh == 0 else trig[rh][0 if tg == 'c' else 1]
            vi_shape = {}
            for (jj, rh) in shapes:
                lam = S(*[M(p('rotor/%s/ph/%s%d' % (side, tg, i)), tf(tg, rh)) if rh else p('rotor/%s/ph/%s%d' % (side, tg, i))
                          for sts, tg in ((PH_COS, 'c'), (PH_SIN, 's')) for i, st in enumerate(sts) if st == (jj, rh)])
                vi_shape[(jj, rh)] = g.fn(b + '/vi-%d-%d' % (jj, rh), M(p(b_ + '/kge'), WR, lam))
            FT, FN, FR, MF, QQ = [], [], [], [], []
            for j, (rr, dr) in enumerate(s.segs):
                e = b + '/s%d' % j
                tt, cli = SEC.interp_sections(U.SECTIONS, rr / U.R)
                sec = SEC.Section(tt, cli)
                al, mach, cl_tab, cd_tab = sec.tables()
                ch = chord(rr)
                vi = S(*[M(v(PH.phi(jj, rh, rr / U.R)), vi_shape[(jj, rh)]) for (jj, rh) in shapes])
                Vt = g.fn(e + '/Vt', S(t0, M(v(rr), t1)), 'blade velocity along its motion, ft/s')
                Vnn = g.fn(e + '/Vn', S(n0, M(v(rr), n1), M(vi, cb)), 'blade velocity along its normal (up), ft/s')
                V2 = g.fn(e + '/V2', S(SQR(Vt), SQR(Vnn), v(1e-4)))
                Vv = g.fn(e + '/V', SQ(V2))
                ar = g.fn(e + '/alpha-raw', D(S(th0, v(twist(rr))), M(v(R2D), ATAN2(Vnn, Vt))))
                g.fn(e + '/alpha-deg', wrap180(ar))
                g.fn(e + '/mach', MIN(Q(Vv, a_snd), v(1.0)))
                cl = g.fn(e + '/cl', T2(e + '/alpha-deg', e + '/mach', al, mach, cl_tab))
                cd = g.fn(e + '/cd', S(T2(e + '/alpha-deg', e + '/mach', al, mach, cd_tab), v(U.DCD)))
                r_in, r_out = s.edges[j], s.edges[j + 1]
                fl = max(0.0, min(1.0, (U.TIP_LOSS * U.R - r_in) / (r_out - r_in)))
                Lk = g.fn(e + '/Lk', M(v(0.5 * ch * dr * fl), rho, Vv, cl), 'lift / V')
                Dk = g.fn(e + '/Dk', M(v(0.5 * ch * dr), rho, cd, SQ(S(V2, SQR(vr)))), 'drag / V')
                Ft = g.fn(e + '/Ft', NEG(S(M(Lk, Vnn), M(Dk, Vt))), 'force along the blade motion, lbf')
                Fn = g.fn(e + '/Fn', D(M(Lk, Vt), M(Dk, Vnn)), 'force along the blade normal (up), lbf')
                FT.append(Ft); FN.append(Fn); FR.append(NEG(M(Dk, vr)))
                MF.append(M(v(rr), Fn)); QQ.append(M(v(rr), Ft))
            ft, fn, fr = g.fn(b + '/FT', S(*FT)), g.fn(b + '/FN', S(*FN)), g.fn(b + '/FR', S(*FR))
            mf = g.fn(b + '/Mflap', S(*MF), 'aerodynamic flap moment about the gimbal, ft lbf (up +)')
            qb = g.fn(b + '/Q', M(cb, S(*QQ)), 'aerodynamic moment about the shaft, in the rotation\'s sense, ft lbf')
            # the inflow's forcing: each shape's moment of the normal loads along the shaft
            for (jj, rh) in shapes:
                shp = g.fn(b + '/Fn-%d-%d' % (jj, rh), M(cb, S(*[M(v(PH.phi(jj, rh, rr / U.R)), p(b + '/s%d/Fn' % j)) for j, (rr, dr) in enumerate(s.segs)])))
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
            g.fn(b_ + '/ph/tau-%s%d' % (tg, i), Q(M(v(kb / (2 * math.pi) if rh == 0 else kb / math.pi), S(*terms)), qn))
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
        sin(alpha), CN90 the data's at -90 deg (F2's for F1 and F4, which have none)."""
        al = sorted(set(list(range(-180, 181, 10)) + list(range(-40, 41, 2)) + [-45, 45, -85, -95, 85, 95]))
        cn90 = {(F, bm): B.WIV[('F2' if F in ('F1', 'F4') else F, bm)][0] for F in ('F1', 'F2', 'F3', 'F4') for bm in (0, 90)}

        def data(F, bm):
            if F == 'F1':
                return (B.WI_ALPHA, B.WI_APLN[0.2] if bm == 90 else B.WI_HELO), (B.WIII_ALPHA, B.WIII_APLN[0.2] if bm == 90 else B.WIII_HELO)
            xs = [a for a, y in zip(B.WII_ALPHA, B.WII[(F, bm)]) if y is not None]
            ys = [y for y in B.WII[(F, bm)] if y is not None]
            xd = [a for a, y in zip(B.WIV_ALPHA, B.WIV[(F, bm)]) if y is not None]
            yd = [y for y in B.WIV[(F, bm)] if y is not None]
            return (xs, ys), (xd, yd)

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
        CL, CD = [], []
        for bm in (0, 90):
            tl, td = [], []
            for a in al:
                rl, rd = [], []
                for F in ('F4', 'F1', 'F2', 'F3'):
                    (xl, yl), (xd, yd) = data(F, bm)
                    # the download factor, from 40 deg towards the flow's normal incidence (full from 60 deg)
                    k = 1 + (DL_K[F] - 1) * min(1.0, max(0.0, (abs(a) - 40) / 20.0)) if abs(a) <= 120 else DL_K[F]
                    rl.append(k * ext(xl, yl, a, 'l', cn90[(F, bm)]))
                    rd.append(k * ext(xd, yd, a, 'd', cn90[(F, bm)]))
                tl.append(rl); td.append(rd)
            CL.append(tl); CD.append(td)
        return al, [-1, 0, 1, 2], [0, 90], CL, CD

    def wing(s):
        """The wing in NSTRIP spanwise strips a side, each in its own flow: the airframe's motion at the strip and both
        rotors' wakes (rotorwake.py), with Bell's wing-pylon coefficients of the whole wing (which carry the pylons and
        the wing's aspect ratio) for the strip's own angle of attack. In the hover the strips under the discs stand in
        the wakes at about -90 deg: the download. Each strip carries its share of the reference area. The flaperons'
        symmetric deflection selects the flap index (interpolating between Bell's flap settings); their differential
        (the ailerons) adds Bell's rolling and yawing moments; the dihedral effect is Bell's."""
        g = s.g
        al, fx_cols, bm_tabs, CL, CD = s.wing_tables()
        fdeg = [-17.5, 0.0, 25.0, 47.0] if U.NAME == 'XV-15' else [-17.5, 0.0, 40.0, 67.0]
        fx = g.fn('aero/wing/flap-index', T1('ctl/flaperon-deg', fdeg, [-1, 0, 1, 2]), 'Bell\'s flap setting: -1 F4, 0 F1, 1 F2, 2 F3')
        # compressibility of the clean wing in airplane mode [Table WI, WIII]: increments over Mach 0.2
        ma = [0.2, 0.4, 0.5, 0.6]
        dcl = [[B.WI_APLN[mm][i] - B.WI_APLN[0.2][i] for mm in ma] for i in range(len(B.WI_ALPHA))]
        dcd = [[B.WIII_APLN[mm][i] - B.WIII_APLN[0.2][i] for mm in ma] for i in range(len(B.WIII_ALPHA))]
        wM = g.fn('aero/wing/mach-weight', M(MAX(v(0), D(v(1), ABS(fx))), Q(p('ctl/beta-m-deg'), v(90))))
        rho = p('atmosphere/rho-slugs_ft3')
        strips = rotorwake.wing_strips(U)
        Sstrip = U.S_W / (2 * len(strips))
        cm0 = T1('aero/wing/flap-index', [-1, 0, 1, 2], [B.CM0['F4'], B.CM0['F1'], B.CM0['F2'], B.CM0['F3']])
        g.fn('aero/wing/cm0', cm0)
        lifts, downs = [], []
        for side, m in SIDES:
            for i, (a, b, y, fs, wl) in enumerate(strips):
                tg = 'aero/wing/%s%d' % (side, i)
                w = s.wake(g, 'w%s%d' % (side, i)) if True else None
                Vx, Vy, Vz = s.point_velocity(g, tg, fs, m * y, wl, w)
                Vn = g.fn(tg + '/V-fps', MAX(SQ(S(SQR(Vx), SQR(Vz))), v(0.01)))
                g.fn(tg + '/alpha-deg', wrap180(S(M(v(R2D), ATAN2(Vz, Vx)), v(U.INC_W))))
                g.fn(tg + '/mach', Q(Vn, p('atmosphere/a-fps')))
                cl = g.fn(tg + '/cl', S(T3(tg + '/alpha-deg', 'aero/wing/flap-index', 'ctl/beta-m-deg', al, fx_cols, bm_tabs, CL),
                                        M(p('aero/wing/mach-weight'), T2(tg + '/alpha-deg', tg + '/mach', B.WI_ALPHA, ma, dcl))))
                cd = g.fn(tg + '/cd', S(T3(tg + '/alpha-deg', 'aero/wing/flap-index', 'ctl/beta-m-deg', al, fx_cols, bm_tabs, CD),
                                        M(p('aero/wing/mach-weight'), T2(tg + '/alpha-deg', tg + '/mach', B.WIII_ALPHA, ma, dcd))))
                qs = g.fn(tg + '/qS', M(v(0.5 * Sstrip), rho, SQR(Vn)))
                Fx = g.fn(tg + '/Fx-lbs', M(qs, Q(D(M(cl, Vz), M(cd, Vx)), Vn)))
                Fz = g.fn(tg + '/Fz-lbs', M(qs, Q(NEG(S(M(cl, Vx), M(cd, Vz))), Vn)))
                My = M(qs, v(U.C_W), cm0)
                rx, ry, rz = s.arm_rp(fs, m * y, wl)
                s.add((Fx, None, Fz), (M(ry, Fz), S(D(M(rz, Fx), M(rx, Fz)), My), NEG(M(ry, Fx))))
                lifts.append(NEG(Fz))
        g.fn('aero/wing/lift-lbs', S(*lifts), 'the wing\'s force up (body axes); in the hover minus its download')
        g.fn('aero/wing/download-lbs', NEG(p('aero/wing/lift-lbs')))
        # ailerons, dihedral effect: on the freestream dynamic pressure and angle of attack
        q = p('aero/qbar-psf')
        aw = g.fn('aero/wing/alpha-deg', S(p('aero/alpha-deg'), v(U.INC_W)))
        fade = T1('aero/wing/alpha-deg', [-B.ALPHA_LDA_ZERO, -8, 8, B.ALPHA_LDA_ZERO], [0, 1, 1, 0])
        kl = T2('aero/wing/flap-index', 'ctl/beta-m-deg', [-1, 0, 1, 2], [0, 90], [list(B.K_LDA[F]) for F in ('F4', 'F1', 'F2', 'F3')])
        clda = g.fn('aero/wing/cl-da', M(v(B.CL_DA), kl, fade), 'rolling moment coefficient per deg of aileron')
        cl_wing = g.fn('aero/wing/CL', Q(p('aero/wing/lift-lbs'), MAX(M(q, v(U.S_W)), v(1))))
        kn0 = T2('aero/wing/flap-index', 'ctl/beta-m-deg', [-1, 0, 1, 2], [0, 30, 90], [B.CN_DA[F][1] for F in ('F4', 'F1', 'F2', 'F3')])
        kn = T2('aero/wing/flap-index', 'ctl/beta-m-deg', [-1, 0, 1, 2], [0, 30, 90], [B.CN_DA[F][2] for F in ('F4', 'F1', 'F2', 'F3')])
        # (+ right flaperon up rolls right: Bell's da = (dl - dr) / 2)
        da = p('ctl/aileron-deg')
        qsb = M(q, v(U.S_W * U.B_W))
        L_a = M(qsb, clda, da)
        N_a = M(qsb, S(kn0, M(kn, MAX(v(-1), MIN(v(2), cl_wing)), clda)), da)
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

    def empennage(s):
        """Horizontal stabiliser with its elevator and the two fins with their rudders: Bell's coefficients [CR] in
        each surface's own flow, with both rotors' wakes over it (rotorwake.py) and the wing's downwash at the
        stabiliser [Table WV]; the fins' sideslip times Bell's sidewash factor [Table VIV]."""
        g = s.g
        rho = p('atmosphere/rho-slugs_ft3')
        # wing downwash at the stabiliser, deg, against the wing's angle of attack, the flap index and Bell's mast angle
        rows = sorted(set(B.WV_APLN_ALPHA) | set(B.WV_HELO_ALPHA))
        eps = [[[float(np.interp(a, B.WV_HELO_ALPHA if bm == 0 else B.WV_APLN_ALPHA, (B.WV_HELO if bm == 0 else B.WV_APLN)[F]))
                 for F in ('F4', 'F1', 'F2', 'F3')] for a in rows] for bm in (0, 90)]
        g.fn('aero/emp/eps-deg', T3('aero/wing/alpha-deg', 'aero/wing/flap-index', 'ctl/beta-m-deg', rows, [-1, 0, 1, 2], [0, 90], eps))
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
        alD, CDH = s.surface_tables(B.HIV_ALPHA, B.HIV_CD, 0.92, 'd')
        Vx, Vy, Vz = s.point_velocity(g, 'aero/emp/h', U.FS_H, 0.0, U.WL_H, s.wake(g, 'stab'))
        Vn = g.fn('aero/emp/h/V-fps', MAX(SQ(S(SQR(Vx), SQR(Vz))), v(0.01)))
        g.fn('aero/emp/h/alpha-deg', wrap180(D(S(M(v(R2D), ATAN2(Vz, Vx)), v(U.INC_H)), p('aero/emp/eps-deg'))))
        cl = g.fn('aero/emp/h/cl', T2('aero/emp/h/alpha-deg', 'ctl/elevator-deg', rowsH, de_cols, CLH))
        cd = g.fn('aero/emp/h/cd', T1('aero/emp/h/alpha-deg', alD, CDH))
        # the force normal to the local flow, which the downwash has turned by eps: lift along the turned normal
        e = M(v(1 / R2D), p('aero/emp/eps-deg'))
        ce, se = COS(e), SIN(e)
        ux, uz = S(M(ce, Vx), NEG(M(se, Vz))), S(M(se, Vx), M(ce, Vz))
        qs = g.fn('aero/emp/h/qS', M(v(0.5 * U.S_H), rho, SQR(Vn)))
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
        alV, CDV = s.surface_tables(B.VIII_BETA, B.VIII_CD, 0.8, 'd')
        g.fn('aero/emp/sidewash', T3('aero/fus/alpha-deg', 'aero/fus/beta-abs-deg', 'ctl/beta-m-deg', B.SW_ALPHA, B.SW_BETA, B.SW_BETAM, B.SW))
        for side, m in SIDES:
            tg = 'aero/emp/fin%s' % side
            Vx, Vy, Vz = s.point_velocity(g, tg, U.FS_V, m * U.BL_V, U.WL_V, s.wake(g, 'fin' + side))
            Vn = g.fn(tg + '/V-fps', MAX(SQ(S(SQR(Vx), SQR(Vy))), v(0.01)))
            g.fn(tg + '/beta-deg', wrap180(M(p('aero/emp/sidewash'), v(R2D), ATAN2(Vy, Vx))))
            cy = g.fn(tg + '/cy', T2(tg + '/beta-deg', 'ctl/rudder-deg', rowsV, B.VI_DR, CYV))
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
        """the nacelles' point masses move with the nacelle angle (JSBSim recomputes the CG and the inertia)"""
        sn, cn = p('ctl/sin-nac'), p('ctl/cos-nac')
        fwd = S(M(v(-U.DX_NAC), sn), M(v(U.DZ_NAC), cn))
        up = S(M(v(U.DX_NAC), cn), M(v(U.DZ_NAC), sn))
        for i in (0, 1):
            o.fn('mass/nacelle%d-x-in' % i, D(v(U.FS_PIV), fwd), out='inertia/pointmass-location-X-inches[%d]' % i)
            o.fn('mass/nacelle%d-z-in' % i, S(v(U.WL_PIV), up), out='inertia/pointmass-location-Z-inches[%d]' % i)

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
def point_masses():
    """(name, lb, FS, BL, WL): the nacelles first (their locations move with the nacelle angle), crew, payload, fuel"""
    nx, nz = U.FS_PIV + U.DX_NAC, U.WL_PIV + U.DZ_NAC      # helicopter mode (DX_NAC aft +)
    pms = [('Right nacelle', U.W_NACELLE, nx, U.BL_PIV, nz), ('Left nacelle', U.W_NACELLE, nx, -U.BL_PIV, nz)]
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
    return pms, fuel


def mass_config(cg=None, weight=None, inertia=None):
    """Empty weight, its CG and inertias such that the reference loading (GW_DESIGN) is at the design CG with the
    reference inertias (helicopter mode). For a test configuration (cg, weight given) everything but the nacelles sits
    at the CG."""
    pms, fuel = point_masses()
    m_ = U.W_NACELLE / G0
    # reference inertia: the rest's own about the CG, and the nacelles' (parallel axes and their own)
    nac = pms[:2]
    I_ref = list(inertia or U.I_REST[:3])
    for n, w, x, y, z in nac:
        dx, dy, dz = (x - U.FS_CG) / 12, y / 12, (z - U.WL_CG) / 12
        I_ref[0] += m_ * (dy * dy + dz * dz) + U.I_NACELLE[0]
        I_ref[1] += m_ * (dx * dx + dz * dz) + U.I_NACELLE[1]
        I_ref[2] += m_ * (dx * dx + dy * dy) + U.I_NACELLE[2]
    total = weight or U.GW_DESIGN
    cgx, cgz = (cg or (U.FS_CG, 0.0, U.WL_CG))[0], (cg or (U.FS_CG, 0.0, U.WL_CG))[2]
    if cg:
        pms = nac + [(n, w, cgx, 0.0, cgz) for n, w, x, y, z in pms[2:]]
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
    for name, t in texts.items():
        open(os.path.join(out, 'Systems', name), 'w').write('<?xml version="1.0"?>\n' + t)
    open(os.path.join(out, pre + '.xml'), 'w').write(fdm_file())
    return len(mdl.g.items)


if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    out = args[0] if args else 'out'
    configure('--xv15' in sys.argv)
    if '--no-inline' in sys.argv:
        INLINE = False
    n = build(out, fg='--fg' in sys.argv)
    print('wrote', out, '-', U.NAME, '-', n, 'aerodynamic functions')
