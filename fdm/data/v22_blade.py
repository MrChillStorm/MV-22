"""The V-22's proprotor blade, for its torsion mode (blademodes.py). Its structure is not published, but the 1/5-scale
aeroelastic model of it is a dynamically scaled one, whose blade was ground vibration tested [WRATS: Ghiringhelli,
Masarati, Mantegazza and Nixon, Multi-Body Analysis of the 1/5 Scale Wind Tunnel Model of the V-22 Tiltrotor, AHS
1999, NTRS 20040086758, table 1: cantilevered, the control chain locked, first torsion 113.35 Hz; to the full size
x 5^-1/2, 50.7 Hz, as the model's rotor speeds are Froude scaled: 888 rpm hover and 742 airplane, the V-22's 397 and
333 x 5^1/2]. So: the XV-15 metal blade's distributions [data/xv15_blade.py] stretched to the V-22's radius,
its mass per foot scaled to the V-22's blade inertia (v22data.I_BLADE, itself an estimate: the pitch inertia's scale
is the mass's times the chord's squared), and its torsional and control system stiffness scaled together so that its
first torsion is the model's (blademodes.py --v22 checks it): 7.7/rev at 100 % rotor speed, against the XV-15 metal
blade's 3.3. The chordwise offsets of its centre of gravity and quarter chord are not published: none. Only its torsion is
modelled: its bending stiffness is not given here."""
from data import xv15_blade as X

R = 228.5 / 12              # ft
NB = 3
CONE = 2.5                  # deg
LAMBDA = R / X.R            # 1.523
MASS_SCALE = 1000.0 / X.I_BLADE_CHECK / LAMBDA ** 3      # per foot: the blades' inertia about the gimbal, 1000 against 103
CHORD_SCALE = 22.0 / 14.0   # the tip chords
STIFFNESS_SCALE = 34.30     # the torsion's and the control system's: the first torsion at WRATS's, 318.5 rad/s nonrotating
TORSION_NR = 2 * 3.141592653589793 * 113.35 / 5 ** 0.5   # rad/s, the target

R_R = X.R_R
TWIST = X.TWIST
MASS = [m * MASS_SCALE for m in X.MASS]
I_THETA = [i * MASS_SCALE * CHORD_SCALE ** 2 for i in X.I_THETA]
GJ = [g * STIFFNESS_SCALE for g in X.GJ]
K_PITCH_LINK = X.K_PITCH_LINK * STIFFNESS_SCALE / LAMBDA ** 2      # on the same horn, stretched
X_PITCH_HORN = X.X_PITCH_HORN
K_P = X.K_P
X_CG = [0.0] * len(R_R)
X_QC = [0.0] * len(R_R)
