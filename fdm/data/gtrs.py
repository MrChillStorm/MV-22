"""The XV-15's data as revised against its flight tests for NASA's generic tilt-rotor simulation [GTRS: Ferguson, A
Mathematical Model for Real Time Flight Simulation of a Generic Tilt-Rotor Aircraft, NASA CR-166536 Rev. A (1988),
appendix B]: Bell's 1973 model [CR-114614, data/bell301.py] whose deficiencies the flight tests showed, among them
"questionable input data values ... for elevator, rudder, and aileron effectiveness as well as the Q-loss value at
the respective control surfaces" and inertias "out of date (airframe modifications and flight test instrumentation
weights and locations were not included)" [GTRS sec. I.C, items 11 and 12]. Only what differs from bell301.py, and
what the flight model uses."""

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
