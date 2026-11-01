"""The XV-15's data as revised against its flight tests for NASA's generic tilt-rotor simulation [GTRS: Ferguson, A
Mathematical Model for Real Time Flight Simulation of a Generic Tilt-Rotor Aircraft, NASA CR-166536 Rev. A (1988),
appendix B]: Bell's 1973 model [CR-114614, data/bell301.py] whose deficiencies the flight tests showed, among them
"questionable input data values ... for elevator, rudder, and aileron effectiveness as well as the Q-loss value at
the respective control surfaces" and inertias "out of date (airframe modifications and flight test instrumentation
weights and locations were not included)" [GTRS sec. I.C, items 11 and 12]. Only what differs from bell301.py, and
what the flight model uses, but for his wing tables, which it does not (genv22's U.WING_DATA tries them)."""

# ---------------------------------------------------------------- mass [GTRS p. B-98, subsystem 9]
GW = 13000.0                     # lb
W_PYLONS = 4200.0                # lb, both (Bell's 3986)
FS_PYLON, WL_PYLON = 291.7, 118.0
FS_CG, WL_CG = 300.0, 81.65      # helicopter mode (Bell's mid CG 296.45)
I_HELO = (52795.0, 21360.0, 66335.0, 1234.0)    # slug ft^2, Ixx, Iyy, Izz, Ixz, helicopter mode (Bell's 42 379,
                                                 # 14 290, 49 520, 0)

# ---------------------------------------------------------------- horizontal stabiliser [GTRS pp. A-85 to A-88, B-56 to B-68]
TAU_E = 0.518                    # elevator effectiveness (Bell's 0.565)
K_HNU = 0.8                      # its dynamic pressure loss multiplier: q_H = K_HNU eta q
D_KE = 0.24                      # the elevator's effectiveness past 15 deg: K_e - D_KE (|de| - 15) / 15
D_WB = 1.0                       # the wing's downwash lag at it (the lift's wing/body damping term)
# the downwash's slope's Mach number factor in that term [table 5-VII]
PCPM_MACH = [0.0, 0.2, 0.4, 0.5, 0.6]
PCPM = [0.0799, 0.0799, 0.0856, 0.0905, 0.0994]
# the elevator's (and rudder's) effectiveness against the Mach number [table 5-IV]
KE_MACH = [0.0, 0.2, 0.4, 0.5, 0.6, 0.7]
KE = [1.0, 1.0, 0.965, 0.95, 0.93, 0.90]
# the stabiliser's lift slope against the Mach number, per deg, the elevator at 0 [table 5-II: 0.155, 0.164, 0.176
# at 2 deg for Mach 0.4, 0.5, 0.6; table 5-I's 0.071 to 0.2]
CLH_MACH = [0.0, 0.2, 0.4, 0.5, 0.6]
CLH_SLOPE = [0.071, 0.071, 0.0775, 0.082, 0.088]
# the dynamic pressure loss with sideslip at the stabiliser and the fins, K_beta [table 5-VI]: eta_S = 1 - (1 - eta) K
KB_BETA = [0, 5, 10, 15, 20, 30, 45, 60]
KB = [1.0, 0.996, 0.985, 0.966, 0.94, 0.866, 0.707, 0.5]

# ---------------------------------------------------------------- wing-pylon lift, drag and moment [GTRS tables 4-I to 4-IV, 4-VIII, 4-IX; pp. B-34 to B-44, B-51]
# Ferguson's flap settings: F1 0/0, F2 20/12.5, F3 40/25, F4 75/47 (Bell's F2 and F3 are 40/25 and 75/47, his F4 the
# reflex), against the wing's angle of attack: the fuselage's less the rotors' induction at low speed [p. A-70], which
# is under 0.1 deg in airplane mode, so in forward flight the fuselage's as Bell's. Bell's 1973 tables less a
# revision: from about -8 deg up to the stall his lift is higher (+0.12 to +0.18 with the flaps up, +0.22 at 40/25 in
# airplane mode, +0.12 at 0/0 in helicopter mode), his drag by about that lift's induced drag, and his maximum lift
# 1.57 for Bell's 1.50. His table 4-IX, the lift slope at zero lift, defined as the difference of two of these tables'
# values, is Bell's tables' exactly (flaps up 0.0799 and 0.057 per deg, airplane and helicopter mode), and not his own
# flaps-up and 20/12.5 tables' (0.105 and 0.09; 0.1025 and 0.08 for its 0.08 and 0.0575); at 40/25 and 75/47, where
# his rise starts above zero lift, his tables and Bell's meet it alike: the lift was raised after the slopes were
# taken, in the revision against the flight tests, for his model. Mast angle 90 airplane, 0 helicopter. Not used: tried
# (genv22 U.WING_DATA 'gtrs20', 'gtrs'), they brought the conversion nearer the flight tests and airplane mode, where the
# rotors' wake hardly reaches the wing, further from them (2 deg of elevator, 8 points of power at 20 deg of flap),
# and the V-22's lift at zero angle of attack to 0.34 for Boeing's 0.18.
WP_ALPHA1 = [-40, -36, -32, -28, -24, -20, -19.5, -16, -15.5, -13, -12, -11, -8, -4, 0, 4, 8, 11, 12, 13, 16, 17, 20,
             24, 28, 32, 36, 40]
# flaps up, Table 4-I: airplane mode at Mach 0.2, 0.4, 0.5, 0.6 (where a Mach number's column is not defined, the
# previous one's), helicopter mode at Mach 0-0.4
CL_WP1 = {0.2: [-.93, -.84, -.84, -.89, -1.00, -1.15, -1.15, -.95, -.91, -.75, -.67, -.59, -.33, -.04, .38, .72, 1.04,
                1.28, 1.37, 1.42, 1.57, 1.57, 1.38, 1.22, 1.20, 1.27, 1.40, 1.46]}
_cl4 = {-20: -.84, -19.5: -.86, -16: -.94, -15.5: -.945, -13: -.85, -12: -.772, -11: -.67, -8: -.37, -4: -.025, 0: .38,
        4: .75, 8: 1.12, 11: 1.41, 12: 1.46, 13: 1.45, 16: 1.32}
_cl5 = {-16: -.675, -15.5: -.680, -13: -.805, -12: -.800, -11: -.78, -8: -.4, -4: -.01, 0: .39, 4: .77, 8: 1.16,
        11: 1.28, 12: 1.27}
_cl6 = {-16: -.49, -15.5: -.49, -13: -.50, -12: -.50, -11: -.49, -8: -.41, -4: -.01, 0: .41, 4: .83, 8: 1.09, 11: 1.12,
        12: 1.12}
CL_WP1[0.4] = [_cl4.get(a, c) for a, c in zip(WP_ALPHA1, CL_WP1[0.2])]
CL_WP1[0.5] = [_cl5.get(a, c) for a, c in zip(WP_ALPHA1, CL_WP1[0.4])]
CL_WP1[0.6] = [_cl6.get(a, c) for a, c in zip(WP_ALPHA1, CL_WP1[0.5])]
CL_WP1_HELO = [-.68, -.58, -.57, -.62, -.72, -.88, -.88, -.73, -.70, -.57, -.50, -.44, -.22, -.06, .30, .55, .80, .98,
               1.05, 1.09, 1.19, 1.17, .98, .80, .78, .86, .98, 1.06]
# flaps up, Table 4-III, as Table 4-I (helicopter mode at Mach 0-0.2)
WD_ALPHA1 = list(range(-40, 41, 4))
CD_WP1 = {0.2: [.575, .505, .425, .327, .230, .150, .089, .042, .025, .0170, .0204, .0418, .072, .118, .171, .247, .354,
                .493, .600, .660, .705]}
_cd4 = {-24: .312, -20: .175, -16: .089, -12: .042, -8: .0250, -4: .0170, 0: .0204, 4: .0418, 8: .072, 12: .128,
        16: .194, 20: .3050, 24: .500}
_cd5 = {-20: .275, -16: .135, -12: .050, -8: .025, -4: .0170, 0: .0204, 4: .0418, 8: .082, 12: .168, 16: .289}
_cd6 = {-16: .240, -12: .110, -8: .052, -4: .040, 0: .042, 4: .062, 8: .127, 12: .268}
CD_WP1[0.4] = [_cd4.get(a, c) for a, c in zip(WD_ALPHA1, CD_WP1[0.2])]
CD_WP1[0.5] = [_cd5.get(a, c) for a, c in zip(WD_ALPHA1, CD_WP1[0.4])]
CD_WP1[0.6] = [_cd6.get(a, c) for a, c in zip(WD_ALPHA1, CD_WP1[0.5])]
CD_WP1_HELO = [.685, .635, .580, .522, .450, .370, .295, .246, .219, .212, .215, .238, .274, .318, .363, .436, .512,
               .580, .642, .698, .748]
# flaps down, Tables 4-II and 4-IV (Mach 0-0.2), by flap setting and mast angle; None where not given
WP_ALPHA2 = [-100, -90, -80, -70, -60, -50, -40, -36, -32, -28, -24, -21.5, -21.0, -20, -19.2, -16, -12, -8, -4, 0, 4,
             8, 11, 12, 13.6, 16, 18.4, 20, 24, 28, 32, 36, 40]
CL_WP = {('F2', 90): [0.0, 0.0, -.45, -.72, -.79, -.80, -.62, -.60, -.60, -.66, -.77, -.84, -.85, -.84, -.81, -.57, -.26,
                      .15, .56, .92, 1.28, 1.54, 1.75, 1.81, 1.88, 1.75, 1.57, 1.46, 1.38, 1.4, 1.5, 1.6, 1.65],
         ('F2', 0): [.15, 0.0, -.28, -.48, -.60, -.62, -.47, -.42, -.40, -.47, -.55, -.60, -.61, -.59, -.54, -.37, -.14,
                     .18, .51, .79, 1.05, 1.28, 1.45, 1.51, 1.54, 1.45, 1.23, 1.10, 1.0, 1.0, 1.1, 1.2, 1.31],
         ('F3', 90): [None, 0.0, -.325, -.520, -.610, -.590, -.410, -.400, -.425, -.515, -.660, -.690, -.680, -.640, -.580,
                      -.320, 0.0, .42, .84, 1.18, 1.46, 1.70, 1.86, 1.92, 1.94, 1.79, 1.62, 1.51, 1.48, 1.54, 1.69, 1.76,
                      1.80],
         ('F3', 0): [None, 0.0, -.245, -.400, -.480, -.420, -.265, -.250, -.260, -.300, -.380, -.440, -.440, -.395, -.360,
                     -.165, .0628, .291, .518, .749, .975, 1.205, 1.380, 1.433, 1.500, 1.400, 1.260, 1.200, 1.15, 1.20,
                     1.32, 1.41, 1.47],
         ('F4', 90): [None, 0.0, -.235, -.385, -.450, -.390, -.240, -.220, -.240, -.275, -.340, -.400, -.400, -.367, -.310,
                      -.048, .272, .69, 1.11, 1.44, 1.66, 1.88, 2.0, 1.99, 1.87, 1.70, 1.53, 1.46, 1.46, 1.54, 1.69,
                      1.78, 1.80],
         ('F4', 0): [None, 0.0, -.190, -.305, -.333, -.220, -.105, -.090, -.095, -.120, -.160, -.210, -.210, -.188, -.140,
                     .040, .268, .6, .92, 1.15, 1.37, 1.59, 1.7, 1.67, 1.55, 1.34, 1.2, 1.14, 1.16, 1.29, 1.38, 1.44,
                     1.48]}
WD_ALPHA2 = [-100, -90, -80, -70, -60, -50] + list(range(-40, 41, 4))
CD_WP = {('F2', 90): [None, None, None, 1.0, .78, .62, .50, .40, .32, .23, .17, .11, .07, .05, .033, .044, .072, .109,
                      .157, .227, .29, .38, .51, .61, .70, .75, .79],
         ('F2', 0): [1.45, 1.57, 1.45, 1.15, .91, .75, .59, .53, .48, .42, .36, .33, .28, .253, .26, .26, .3, .32, .37,
                     .43, .48, .53, .63, .70, .75, .78, .83],
         ('F3', 90): [None, 1.18, 1.10, .93, .705, .565, .430, .335, .245, .180, .130, .090, .065, .058, .076, .106, .141,
                      .186, .243, .322, .404, .528, .630, .710, .764, .805, .865],
         ('F3', 0): [1.33, 1.44, 1.33, 1.12, .91, .75, .54, .468, .405, .352, .310, .282, .263, .253, .267, .307, .345,
                     .394, .453, .537, .589, .630, .690, .748, .800, .845, .888],
         ('F4', 90): [None, 1.145, 1.050, .89, .67, .507, .450, .400, .350, .309, .278, .260, .243, .246, .282, .330, .372,
                      .424, .492, .580, .667, .730, .790, .838, .883, .950, 1.025],
         ('F4', 0): [.88, .90, .878, .822, .740, .640, .550, .525, .500, .480, .462, .450, .440, .445, .485, .536, .580,
                     .636, .706, .778, .814, .839, .880, .920, .955, .985, 1.015]}
# pitching moment at zero lift, Table 4-VIII, against the mast angle
CM_WP_BETAM = [0, 15, 30, 60, 90]
CM_WP = {'F1': [.025, .070, .080, .050, -.025], 'F2': [-.05, -.01, 0.0, -.05, -.11], 'F3': [-.110, -.090, -.060, -.110, -.170],
         'F4': [-.115, -.110, -.080, -.130, -.190]}
# the lift slope at zero lift, per deg, Table 4-IX: helicopter, airplane mode
CL_ALPHA0 = {'F1': (0.057, 0.0799), 'F2': (0.0575, 0.08), 'F3': (0.057, 0.08), 'F4': (0.0563, 0.08)}

# ---------------------------------------------------------------- the wing's wake at the stabiliser [GTRS tables 4-V(a) to (e), pp. B-45 to B-49]
# downwash, deg, against the wing's angle of attack, for the flap settings 0/0, 20/12.5, 40/25, 75/47, at the mast
# angles 0 (helicopter mode), 15, 30, 60, 90 deg (airplane mode); Mach 0-0.2
WV_ALPHA = [-90, -16, -12, -8, -4, 0, 4, 8, 12, 16, 20, 24, 28, 90]
WV_BETAM = [0, 15, 30, 60, 90]
WV_FLAPS = ['F1', 'F2', 'F3', 'F4']           # 0/0, 20/12.5, 40/25, 75/47
_z = lambda *xs: [0, 0] + list(xs) + [0, 0]
WV = {
    0: {'F1': _z(0, 0.09, 1.62, 3.15, 4.68, 6.21, 7.1, 7.5, 7.0, 4.8), 'F2': _z(0.8, 1.6, 3.1, 4.7, 6.2, 7.8, 8.5, 8.6, 7.5, 4.9),
        'F3': _z(1.47, 3.03, 4.59, 6.15, 7.71, 9.27, 9.8, 9.7, 8.0, 5.0), 'F4': _z(1.47, 3.03, 4.59, 6.15, 7.71, 9.27, 9.8, 9.7, 8.0, 5.0)},
    15: {'F1': _z(0, 0, 1.26, 2.80, 4.34, 5.88, 7.1, 7.3, 6.7, 4.1), 'F2': _z(0.4, 1.2, 2.7, 4.3, 6.0, 7.1, 8.7, 8.9, 8.2, 5.3),
         'F3': _z(0.7, 2.4, 4.1, 5.8, 7.5, 9.2, 10.4, 10.8, 9.8, 6.4), 'F4': _z(0.7, 2.4, 4.1, 5.8, 7.5, 9.2, 10.4, 10.8, 9.8, 6.4)},
    30: {'F1': _z(0, 0, 1.18, 2.70, 4.22, 5.74, 7.0, 7.3, 6.7, 4.1), 'F2': _z(0, 0.7, 2.1, 3.6, 5.2, 6.7, 7.9, 8.2, 7.4, 4.8),
         'F3': _z(0, 1.3, 2.9, 4.5, 6.1, 7.7, 8.9, 9.1, 8.1, 5.5), 'F4': _z(0, 1.3, 2.9, 4.5, 6.1, 7.7, 8.9, 9.1, 8.1, 5.5)},
    60: {'F1': _z(0, 0, 1.2, 2.6, 4.0, 5.2, 6.4, 6.8, 6.3, 4.1), 'F2': _z(0, 0.9, 2.25, 3.80, 5.30, 6.80, 7.80, 8.20, 7.40, 4.80),
         'F3': _z(0, 1.78, 3.38, 4.98, 6.58, 8.18, 9.2, 9.5, 8.4, 5.5), 'F4': _z(0, 1.78, 3.38, 4.98, 6.58, 8.18, 9.2, 9.5, 8.4, 5.5)},
    90: {'F1': _z(0, 0.06, 1.32, 2.58, 3.84, 5.10, 5.90, 6.30, 6.00, 4.00), 'F2': _z(0.45, 1.25, 2.60, 4.08, 5.35, 6.60, 7.40, 7.55, 6.70, 4.40),
         'F3': _z(0.95, 2.54, 3.92, 5.40, 6.88, 8.26, 8.90, 8.80, 7.30, 4.80), 'F4': _z(0.95, 2.54, 3.92, 5.40, 6.88, 8.26, 8.90, 8.80, 7.30, 4.80)},
}

# ---------------------------------------------------------------- the rotors' wake at the tail [GTRS pp. A-34 to A-40, B-21 to B-25]
# the velocity along the masts at the horizontal stabiliser, a share of the rotors' mean induced velocity (+ down the
# mast in helicopter mode), against the fuselage's angle of attack (rows, deg) and the true airspeed (columns, kt), at
# the mast angles 0 (helicopter mode), 15, 30 deg; none at 60 and 90 [tables 2-Ia to 2-Id]. Negative: against the
# induced flow, an upwash at the stabiliser, between the two rotors. It reaches the tail through a lag of the hub to
# tail distance over the speed, and the fins see the same share [p. A-39]
RH_ALPHA = [-180, -30, -28, -24, -20, -16, -12, -8, -4, 0, 4, 8, 12, 16, 20, 180]
RH_V = [0, 20, 40, 60, 80, 100, 120, 140]
RH_BETAM = [0, 15, 30, 60, 90]
_rh = lambda *rows: [[0.0] * 8, [0.0] * 8] + [[0.0] + list(r) + [0.0] for r in rows] + [[0.0] * 8, [0.0] * 8]
RH = {0: _rh((-.02, -.06, -.10, -.15, -.12, -.02), (-.05, -.15, -.30, -.60, -.37, -.05), (-.06, -.25, -.50, -.92, -.65, -.06),
             (-.07, -.40, -.70, -1.10, -.85, -.07), (-.07, -.46, -.85, -1.13, -.90, -.08), (-.14, -.46, -.73, -1.05, -.80, -.10),
             (-.0945, -.33, -.623, -.90, -.67, -.09), (-.06, -.23, -.52, -.725, -.57, -.07), (-.0314, -.113, -.392, -.55, -.45, -.03),
             (-.075, -.127, -.290, -.44, -.35, -.07), (-.06, -.10, -.250, -.38, -.27, -.06), (-.04, -.045, -.160, -.20, -.15, -.04)),
      15: _rh((-.05, -.10, -.15, -.25, -.22, -.05), (-.10, -.25, -.40, -.55, -.50, -.10), (-.15, -.40, -.65, -.90, -.75, -.15),
              (-.20, -.50, -.78, -1.05, -.90, -.20), (-.20, -.60, -.85, -1.08, -.95, -.20), (-.16, -.55, -.80, -1.04, -.92, -.16),
              (-.10, -.45, -.75, -.92, -.85, -.10), (-.09, -.30, -.61, -.75, -.65, -.09), (-.10, -.17, -.48, -.56, -.52, -.10),
              (-.08, -.27, -.34, -.36, -.35, -.08), (-.07, -.22, -.25, -.27, -.26, -.07), (-.05, -.15, -.15, -.15, -.15, -.05)),
      30: _rh((-.02, -.03, -.04, -.05, -.05, -.02), (-.04, -.06, -.06, -.10, -.10, -.04), (-.05, -.08, -.14, -.15, -.15, -.05),
              (-.07, -.15, -.20, -.26, -.26, -.07), (-.08, -.20, -.25, -.38, -.38, -.08), (-.08, -.22, -.35, -.44, -.44, -.08),
              (-.08, -.26, -.43, -.48, -.48, -.08), (-.08, -.30, -.45, -.52, -.52, -.08), (-.07, -.30, -.45, -.60, -.60, -.07),
              (-.06, -.24, -.30, -.44, -.44, -.06), (-.06, -.15, -.24, -.28, -.28, -.06), (-.04, -.06, -.10, -.14, -.14, -.04)),
      60: [[0.0] * 8 for _ in range(16)], 90: [[0.0] * 8 for _ in range(16)]}
# its factor against the fuselage's sideslip (rows, deg, either side) and the mast angle (columns) [table 2-II]
KHB_BETA = [0, 4, 8, 12, 16, 20, 24, 28, 32, 180]
KHB = [[1.0, 1.0, 1.0, 1.0, 1.0], [0.9, 0.99, 1.1, 1.0, 1.0], [0.625, 0.96, 1.3, 1.0, 1.0], [0.30, 0.89, 1.4, 1.0, 1.0],
       [0.05, 0.70, 0.8, 1.0, 1.0], [-.17, 0.15, 0.45, 1.0, 1.0], [-.28, 0.0, 0.225, 0.5, 0.5], [-.17, 0.0, 0.07, 0.25, 0.25],
       [0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0]]

# ---------------------------------------------------------------- the pylons' and spinners' drag [GTRS pp. 26, A-75, A-76, B-33, B-55]
# the drag at the wing-pylon intersection, ft^2 of both pylons on the wing's dynamic pressure in the rotors' wakes, "from
# flight-test data ... a correction or addition to the model in order to account for extra drag due to wing-pylon
# interference" [p. 26], against the mast angle [table 4-XV]
PYINT_BETAM = [-5, 0, 15, 30, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90]
PYINT = [13.5, 13.5, 13.5, 13.5, 13.5, 13.4, 13.25, 13.0, 12.0, 10.5, 8.0, 5.7, 3.4, 1.0]
# each spinner's, on its own dynamic pressure (the flow through the disc), ft^2: 1.0 along the mast and 5.5 times the
# square of the sine of its angle of attack [p. A-76, B-33] (Bell's had been implemented wrongly [sec. I.C, item 5])
SPINNER_DQ0, SPINNER_DQ = 1.0, 5.5

# ---------------------------------------------------------------- the ailerons [GTRS pp. A-70, A-73, A-74, B-32, B-53, B-54]
# (the flaperons' differential deflection; the flap settings as WV's)
CL_DA = 0.006                    # rolling moment coefficient per deg, flaps 0, |alpha_W| < 8 deg (Bell's 0.005)
# its effectiveness factor against the mast angle 0 (helicopter mode) and 90 deg (airplane), linear between [table 4-XI]
# (as Bell's, with the 20/12.5 setting added); fading to nothing from 8 to 25 deg of the wing's angle of attack
K_LDA = {'F1': (0.68, 1.00), 'F2': (0.67, 0.88), 'F3': (0.66, 0.73), 'F4': (0.45, 0.34)}
# the yawing moment, Cn_da = K_n0 + K_n C_L Cl_da, against the mast angle 0, 30, 90 deg [tables 4-XIII, 4-XIV]: K_n
# 17.5 times Bell's throughout (his made the ailerons' yaw proverse in cruise, GTRS's slightly adverse)
CN_DA = {'F1': ([0, 30, 90], [.00046, .00092, .00143], [-.61, -.62, -.415]),
         'F2': ([0, 30, 90], [.00046, .001005, .00123], [-.405, -.20, -.30]),
         'F3': ([0, 30, 90], [.00046, .00109, .00103], [-.24, -.09, -.24]),
         'F4': ([0, 30, 90], [-.00003, .00035, .00029], [-.405, -.087, -.275])}
# the wing's lift coefficient lost per deg of aileron, either way [table 4-XII; p. A-70: L = q S C_L - q S C_L_da |da|]
CL_DA_LIFT = {'F1': 0.00316, 'F2': 0.00396, 'F3': 0.00476, 'F4': 0.0}

# ---------------------------------------------------------------- fins [GTRS p. B-69, subsystem 6]
S_V = 20.25                      # ft^2, each of two (Bell's 25.25)
TAU_R = 0.27                     # rudder effectiveness (Bell's 0.385)
D_KR = 0.24                      # past 15 deg of rudder, as D_KE
