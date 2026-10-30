"""Bell XV-15 data for the JSBSim model's validation build (genv22.py --xv15), with sources.

The tiltrotor model is built, and checked, as an XV-15 first: its rotor, airframe and controls are Bell's own
simulation data [CR] and NASA's measurements, and NASA's flight and wind tunnel tests are there to compare with. The
V-22 build (v22data.py) is the same model with the V-22's data.

  [CR]  NASA CR-114614 (Bell, 1973): Model 301 simulation data (data/bell301.py)
  [TX]  NASA TM X-62407 (1975): XV-15 familiarization document
  [FB]  NASA TM-86833 (Felker, Betzina and Signor, 1985): hover test of a full-scale XV-15 rotor (metal blades): table 1,
        twist (fig. 5), thickness (fig. 6), chord (fig. 7), airfoils (fig. 8)
  [MA]  NASA SP-2000-4517 (Maisel, Giulianetti and Dugan, 2000): the history of the XV-15

Frames: Bell's stations [CR] (FS aft, BL right, WL up, in). Nacelle angle 90 in helicopter mode, 0 in airplane mode
(Bell's mast angle beta_M = 90 - nacelle angle).
"""
import math
from data import bell301 as B

NAME = 'XV-15'
DEG = math.pi / 180
G0 = 32.174
IN_PER_M = 39.3701

# ---------------------------------------------------------------- nacelles, conversion [CR B-4, B-5; TX 3.7]
FS_PIV, BL_PIV, WL_PIV = B.FS_SP, B.BL_SP, B.WL_SP
DX_HUB, DZ_HUB = 0.0, B.L_MAST * 12      # hub on the shaft through the pivot, 4.667 ft above it in helicopter mode
NACELLE_MIN, NACELLE_MAX = 0.0, 95.0     # [TX: conversion range 0 to 95 deg]
NACELLE_RATE = 7.5                       # deg/s: the conversion takes about 12 s [CR table CV: 11-15 deg/s at full rate]
CPS = None                  # the XV-15 had no conversion protection

# ---------------------------------------------------------------- proprotors [FB table 1, figs. 5-8; CR B-4]
R = 12.5
NB = 3
CHORD_TIP = 14.0 / 12       # 0.356 m [FB]; solidity 0.089
CHORD_ROOT = 14.0 / 12      # constant over the working blade (the root's flare inboard of 0.25 R is left out)
R_ROOT = 0.1 * R
# twist (deg, relative to 0.75 R), [FB fig. 5] (geometric, traced from the scan; its zero is at 0.765 R)
TWIST_R = [0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0]
TWIST = [x - 0.46 for x in [29.89, 26.81, 23.69, 20.53, 17.48, 14.65, 11.97, 9.46, 7.5, 5.8, 4.31, 3.08, 1.77, 0.46,
                            -0.99, -2.37, -3.67, -5.01, -6.3]]
# sections [FB fig. 8]: 64-935 at 0.09 R, 64-528 at 0.17, 64-118 at 0.51, 64-(1.5)12 at 0.8, 64-208 at the tip
SECTIONS = [(0.09, 0.35, 0.90), (0.17, 0.28, 0.50), (0.51, 0.18, 0.10), (0.8, 0.12, 0.15), (1.0, 0.08, 0.20)]
PRECONE = 2.5 * DEG         # [FB, TX]
DELTA3 = -15.0 * DEG        # [CR, TX]
I_BLADE = B.I_BLADE         # 102.5 slug ft^2 per blade
K_HUB = B.K_HUB / DEG       # 225 ft lb/deg per rotor
FLAP_STOP = B.FLAP_LIMIT * DEG
J_ROTOR = NB * I_BLADE * 1.10
TIP_LOSS = 0.97
NSEG = 8
# rotor speed: 589 rpm in helicopter and conversion modes, 517 rpm in airplane mode, as flown [MA]; the 1975 design
# 565 / 458 rpm [TX]. 100 % is 589 rpm here
OMEGA_100 = 589.0 * 2 * math.pi / 60
NR_HELO, NR_APLN = 100.0, 517.0 / 589.0 * 100
DCD = 0.0

# ---------------------------------------------------------------- flight controls [CR B-36 to B-40, TX 3.4]
FLAPERON_HOVER = 47.0       # the F3 (75/47) setting
FLAPERON_CONV = 25.0        # the F2 (40/25) setting, the XV-15's conversion flaps [CR, TM-84222]
FLAPERON_AILERON = B.DA_DXLT * B.X_LAT
ELEVATOR_RANGE = 20.0
RUDDER_RANGE = 20.0

# ---------------------------------------------------------------- airframe [CR table II, TX 3.3]
S_W, B_W, C_W = B.S_WP, 32.17, B.C_WP
SWEEP_W, DIHEDRAL_W, INC_W = -6.5, 2.0, 3.0
BL_W_ROOT, BL_W_TIP = 30.0, 160.0       # fuselage side, the pylon's inboard side (estimates from the three-view)
FS_W_TIP = 291.17                       # wing-pylon centre of pressure [CR]
WL_W_ROOT = B.WL_WP
S_H, B_H, INC_H = B.S_H, B.B_H, 0.0
FS_H, WL_H = B.FS_H, B.WL_H
S_V, B_V = B.S_V, B.B_V
FS_V, BL_V, WL_V = B.FS_V, B.BL_V, B.WL_V
FUS_SCALE = 1.0
FS_F, WL_F = B.FS_F, B.WL_F
DRAG_EXTRA = 0.0            # FITTED in the test programme (sweep.py)

# ---------------------------------------------------------------- mass [CR table I, B-5]
GW_DESIGN = B.GW
W_NACELLE = B.W_PYLONS / 2
DX_NAC, DZ_NAC = B.FS_PYLON - B.FS_SP, B.WL_PYLON - B.WL_SP     # pylon CG from the pivot in helicopter mode
I_NACELLE = B.I_PYLON
FS_CG, WL_CG = (B.CG_AFT + B.CG_FWD) / 2, B.WL_CG
# inertia without the pylons at the mid CG: table I's less the pylons' (their parallel axes and own)
_m = W_NACELLE / G0
_dy = B.BL_SP / 12
_dx, _dz = (B.FS_PYLON - FS_CG) / 12, (B.WL_PYLON - WL_CG) / 12
I_REST = (B.I_AFT[0] - 2 * (_m * (_dy ** 2 + _dz ** 2) + I_NACELLE[0]),
          (B.I_AFT[1] + B.I_FWD[1]) / 2 - 2 * (_m * (_dx ** 2 + _dz ** 2) + I_NACELLE[1]),
          (B.I_AFT[2] + B.I_FWD[2]) / 2 - 2 * (_m * (_dx ** 2 + _dy ** 2) + I_NACELLE[2]), 0.0)
W_EMPTY = 9570.0

# ---------------------------------------------------------------- engines and drive [CR B-6, B-47, B-49]
# two Lycoming LTC1K-4K (T53 derivatives): 1802 shp 2-minute contingency, 1550 take-off; the cross-shafted drive [MA]
P_MRP = B.HP_RATINGS['takeoff']
P_IRP = B.HP_RATINGS['military']
P_MCP = B.HP_RATINGS['normal']
ENGINE_TAU = 0.6
# the drive's torque limit: 2400 rotor shaft hp at 589 rpm, the conversion corridor's (and, at 517 rpm, the airplane
# mode's 2150 hp) [TM-84222 fig. 20], with 4 % for the gearboxes and accessories at the engines
P_XMSN_100 = 2500.0
J_DRIVE = B.J_DRIVE - 2 * J_ROTOR       # Bell's 824 slug ft^2 for the drive in all, less the rotors
# ---------------------------------------------------------------- landing gear [CR B-5]
NOSE_GEAR = (B.FS_NG, 0.0, B.WL_NG - 6.5)
MAIN_GEAR = (B.FS_MG, B.BL_MG, B.WL_MG - 10.25)
D_GEAR = (B.D_MAIN_GEAR, B.D_NOSE_GEAR)
EYE = (B.FS_PILOT, B.BL_PILOT, B.WL_PILOT)
FLAPERON_SCHEDULE = None
