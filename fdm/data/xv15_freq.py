"""The XV-15's bare-airframe frequency responses identified from flight tests (frequency sweeps, chirp z-transform),
as the transfer functions fitted to them [TT: Tischler, Frequency-Response Identification of XV-15 Tilt-Rotor Aircraft
Dynamics, NASA TM-89428 (1987), table 4.1 (hover) and table 5.1 (cruise, with Bell's nonreal-time simulation's)].

Conditions [TT sec. 2.2]: hover out of ground effect, nacelles 90 deg, gear down, wind under 5 kt, SCAS on in pitch and
roll (the responses are to the surfaces, so the open loop's); cruise at 170 KIAS (180 KCAS), nacelles 0 deg, density
altitude 10 000 ft (the sweeps), 517 rpm. Units: p, q, r deg/s; a_z g (+ down); beta deg (at the CG); delta_a,
delta_e, delta_r deg of the surfaces, delta_c % of the power lever. Signs as NASA's: delta_e + trailing edge down,
delta_a + rolls left (q/delta_e and p/delta_a are negative at high frequency), delta_r + noses right.

Each transfer function: (gain, numerator factors, denominator factors, time delay s), a factor being a first-order
(a: s + a, 0 for s) or a second-order [zeta, w]: s^2 + 2 zeta w s + w^2. RANGE: the frequencies (rad/s) each transfer
function was fitted over, where the identification's coherence was good [TT secs. 4.1-4.2, 5.1-5.2]."""
import numpy as np

HOVER = {
    'r/dr': (0.619, [], [0.102], 0.0210),
    'p/da': (-3.71, [0, -0.107, 0.412], [0.102, 1.23, (-0.418, 0.447)], 0.0313),
    'r/da': (0.344, [-0.345, (0.868, 0.487)], [0.102, 1.23, (-0.418, 0.447)], 0.00900),
    'az/dc': (-0.00980, [0], [0.105], 0.00740),
    'q/de': (-2.66, [0, -0.271, 0.508], [0.105, 1.32, (-0.463, 0.579)], 0.0656),
}
CRUISE = {
    'p/da': (-4.49, [0, (0.313, 1.89)], [0.0630, 1.09, (0.248, 1.58)], 0.0450),
    'beta/dr': (-0.0510, [0.0860, 0.818, 48.0], [0.0630, 1.09, (0.248, 1.58)], 0.0260),
    'q/de': (-7.38, [0.890], [(0.536, 2.021)], 0.00500),
    'az/de': (-0.0230, [-6.70, 7.59], [(0.536, 2.021)], 0.00200),
}
# Bell's nonreal-time simulation at the cruise condition [TT table 5.1]: the model the flight model's XV-15 data are
# from [CR-114614], with the inertias the flight tests found too low in pitch and yaw (the tail's ballast) [TT p. 126]
CRUISE_BELL = {
    'p/da': (-4.76, [0.00810, (0.322, 1.78)], [0.0803, 0.937, (0.275, 1.66)], 0.0),
    'beta/dr': (-0.0688, [0.028, 0.794, 45.8], [0.0803, 0.937, (0.275, 1.66)], 0.0),
    'q/de': (-12.4, [0.0454, 0.823, 6.37], [(0.140, 0.168), (0.505, 2.57), 6.30], 0.0),
    'az/de': (-0.0128, [-0.00940, 0.0352, 6.49, 10.6, -13.8], [(0.140, 0.168), (0.505, 2.57), 6.30], 0.0),
}
RANGE = {
    ('hover', 'p/da'): (0.2, 9.0), ('hover', 'r/dr'): (0.1, 3.0), ('hover', 'r/da'): (0.1, 7.0),
    ('hover', 'az/dc'): (0.1, 3.0), ('hover', 'q/de'): (0.2, 5.0),
    ('cruise', 'p/da'): (0.1, 8.5), ('cruise', 'beta/dr'): (0.1, 3.5), ('cruise', 'q/de'): (0.2, 7.0),
    ('cruise', 'az/de'): (0.3, 10.0),
}


def evaluate(tf, w):
    """the transfer function's complex response at frequencies w (rad/s)"""
    s = 1j * np.asarray(w, float)
    k, num, den, tau = tf

    def fac(f):
        return s * s + 2 * f[0] * f[1] * s + f[1] ** 2 if isinstance(f, tuple) else s + f
    n = np.ones_like(s) * k
    for f in num:
        n = n * fac(f)
    d = np.ones_like(s)
    for f in den:
        d = d * fac(f)
    return n / d * np.exp(-tau * s)
