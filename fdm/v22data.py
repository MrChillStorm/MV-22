"""Bell Boeing V-22 Osprey (MV-22B) data for the JSBSim model, with sources.

The V-22's engineering data are not public. What is: its dimensions, weights, rotor geometry and rotor speeds, the
wing and tail planforms, the rotor and wing hover tests at 0.658 scale and the 1/4-scale rotor's geometry, a few
performance charts. Everything else comes from the XV-15, the V-22's technology demonstrator, through Bell's real-time
simulation data of it [CR] (data/bell301.py), scaled by geometry; each such value is marked XV-15.

  [RC]  Rosenstein and Clark (Boeing Vertol), "Aerodynamic Development of the V-22 Tilt Rotor", 12th ERF 1986, paper 14:
        design characteristics (fig. 4), rotor geometry (fig. 16), wing (section 6.2), download (fig. 18), airframe lift
        and drag (fig. 28), hover ceiling and flight envelope (fig. 33)
  [PD]  Dunford (Bell), "The V-22 Osprey - a significant flight test challenge", ERF 1990 paper II.7.3: control
        system (fig. 2d), conversion corridor (fig. 5), rotor/wing lift sharing (fig. 7), rotor speeds
  [AC]  Acree, NASA/TM-2016-219070: JVX (0.656-scale V-22) and V-22 rotor characteristics (table 1)
  [WJ]  Johnson, "Airloads and wake geometry calculations for an isolated tiltrotor model in a wind tunnel" (TRAM,
        1/4-scale V-22), AHS 2001: airfoil stations, precone, delta-3, chord and twist (fig. 2)
  [CR]  NASA CR-114614 (Bell, 1973): the Model 301 (XV-15) simulation data (data/bell301.py)
  [TX]  NASA TM X-62407 (1975): XV-15 familiarization document
  [FS]  V-22 Osprey fact sheet (NAVAIR PMA-275, about 2001; refs/web/popasmoke-V22factsheet.pdf): weights, fuel
        capacity, crew and troop seats
  [GS]  GlobalSecurity.org, "V-22 fuel system": the MV-22's tanks, their capacities and the transfer sequence
  [GL]  GlobalSecurity.org, "V-22 Osprey" (its operating limits): airspeed in airplane mode, the high sink rate
        warning's boundary (the cockpit's flight display draws them)
  [3D]  the FlightGear V-22's 3-D model (MV-22B/Models/v22.ac and v22.xml): placement of the nacelle pivot, hubs,
        wing, tail and wheels, so that the flight model and the visual model agree

Frames: the stations (in) are the 3-D model's frame (x aft, y right, z up, m) times 39.37: FS = (x + 8.22) 39.37 (FS 0
at the tip of the refuelling probe), BL = y 39.37, WL = (z + 2.95) 39.37 (WL 0 at the bottom of the tyres). The nacelle
angle is the V-22's: 90 deg with the rotor shafts vertical (helicopter), 0 in airplane mode.
"""
import math

NAME = 'V-22'
DEG = math.pi / 180
G0 = 32.174
IN_PER_M = 39.3701
X0_M, Z0_M = 8.22, 2.95


def model_to_station(x, y, z):
    """3-D model metres (x aft, y right, z up) -> (FS, BL, WL) inches"""
    return (x + X0_M) * IN_PER_M, y * IN_PER_M, (z + Z0_M) * IN_PER_M


# ---------------------------------------------------------------- nacelles, conversion
# conversion axis [3D: the nacelles' rotation centre x -1.88, z 0.52, axis y]; hub in helicopter mode [3D: x -1.574,
# y 7.041, z 2.252]: 1.732 m above the axis and 0.306 m aft of it, so in airplane mode the hub is 1.73 m ahead of the
# axis [RC: "the rotor hub centerline is placed 2 ft ahead of the leading edge of the wing at the tip"]. Hub separation
# 46.2 ft [3D] (84.58 ft width with the rotors less their 38 ft diameter: 46.58 ft [RC fig. 4])
FS_PIV, BL_PIV, WL_PIV = model_to_station(-1.88, 7.041, 0.52)
DX_HUB, DZ_HUB = 0.306 * IN_PER_M, 1.732 * IN_PER_M          # hub from the axis in helicopter mode: aft, up (in)
NACELLE_MIN, NACELLE_MAX = 0.0, 97.5                          # deg [PD fig. 5: "7 deg aft of vertical", 97.5]
NACELLE_RATE = 8.0                                            # deg/s, conversion (estimate: about 12 s from 90 to 0)
# conversion protection [PD fig. 5]: the lower boundary's (equivalent airspeed kt, least nacelle angle) and the upper
# boundary's (kt, greatest nacelle angle), traced
CPS = (([0, 35, 70, 93, 107, 125], [75, 75, 60, 40, 20, 0]), ([0, 130, 150, 175, 200], [97.5, 97.5, 60, 30, 0]))
# the operating limits the flight display draws [GL]: airplane mode's airspeed, 280 KCAS or Mach 0.48, whichever is
# less, at 84 % rotor speed, 220 KCAS at 100 %; the high sink rate warning's boundary (the vortex ring state's), a
# descent of 800 ft/min up to 30 KCAS, rising to 1600 at 60 and none past it
VMO_APLN, MMO_APLN, VMO_APLN_100 = 280.0, 0.48, 220.0
# the gear down, KCAS: ASSUMED. No public V-22 figure (a Naval Safety Command account of a gear malfunction speaks of
# keeping below the gear's transition limits without a number); fast military jets' are about 250-300. ENG OUT lowers
# the gear in its 200 KCAS glide
VLE = 250.0
VRS_SINK = ([30, 60], [800, 1600])
# rotor splay: the shafts are splayed 2.5 deg outboard in helicopter mode [RC 6.2]; left out

# ---------------------------------------------------------------- proprotors [RC fig. 4, 16; AC table 1; WJ]
R = 228.5 / 12          # ft (38 ft diameter)
NB = 3
CHORD_TIP = 22.0 / 12   # ft
CHORD_ROOT = 34.3 / 12  # ft, the linear planform extrapolated to the centre; thrust weighted solidity 0.105
R_ROOT = 0.15 * R       # ft, start of the aerodynamic blade (the cuff's fairing inboard of it is left out)
# twist (deg, relative to 0.75 R) [RC fig. 16: 40 deg at the root, zero at 0.75 R, -7.5 at the tip (47.5 deg total);
# WJ fig. 2: the TRAM's]
TWIST_R = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.75, 0.8, 0.9, 1.0]
TWIST = [40.0, 33.5, 26.5, 19.0, 12.5, 8.4, 5.2, 1.8, 0.0, -1.5, -4.5, -7.5]
# sections (r/R, thickness, design lift coefficient): XN28 at 0.2, XN18 0.5, XN12 0.75, XN09 1.0 [RC, WJ]; their
# camber is not published: the XV-15 64-series sections' design lift at the same stations (data/sections.py)
SECTIONS = [(0.2, 0.28, 0.40), (0.5, 0.18, 0.15), (0.75, 0.12, 0.15), (1.0, 0.09, 0.20)]
PRECONE = 2.5 * DEG     # the JVX rotor's (XV-15 hub) [AC]; the V-22's coning flexure has "slightly different" precone
DELTA3 = -15.0 * DEG    # pitch-flap coupling, nominal [WJ table 1]
# the gimbal's hub spring: "the equivalent of a 2 % hinge offset" [RC 6.1]: K = (Nb/2) (3/2) (e/R) Ib Omega^2 at 100 %
I_BLADE = 1000.0        # slug ft^2 per blade about the gimbal: Lock number 3.8, as the XV-15's 3.83 [CR] (estimate)
from data import v22_blade as BLADE      # its torsion mode: the XV-15's blade scaled to WRATS's (data/v22_blade.py)
OMEGA_100 = 397.0 * 2 * math.pi / 60    # rad/s, 100 % rotor speed: 397 rpm, 790 ft/s tip speed [PD, RC]
K_HUB = NB / 2 * 1.5 * 0.02 * I_BLADE * OMEGA_100 ** 2          # ft lb/rad per rotor
FLAP_STOP = 12.0 * DEG  # gimbal flapping limit (the XV-15's [TX]; [RC] designs the wing clearance for 10.5 deg)
J_ROTOR = NB * I_BLADE * 1.10   # slug ft^2 per rotor about the shaft: the blades and 10 % for the hub (estimate)
TIP_LOSS = 0.97
NSEG = 8
# rotor speed schedule, % of 100 %: 104 % (412 rpm) in helicopter and conversion modes, 84 % (333 rpm) in airplane
# mode [RC: 790 ft/s USMC, 820 ft/s USAF, 662 ft/s cruise; PD]
NR_HELO, NR_APLN = 104.0, 84.0
DCD = 0.0               # blanket section drag increment (the sections' own minimum drag is fitted to the XV-15)

# ---------------------------------------------------------------- flight controls
# The V-22's fly-by-wire control laws are not public; [PD fig. 2d] and [RC 4] give their structure: in helicopter mode
# pitch by longitudinal cyclic of both rotors, roll by differential collective (with lateral cyclic in sideward
# flight), yaw by differential longitudinal cyclic; thrust by the thrust control lever, collective pitch with blade
# pitch (beta) governing of the rotor speed; in airplane mode elevators, flaperons and rudders, thrust by the lever
# with beta governing; phased between the two through the conversion. The mixing gains, their phasing with the
# nacelle angle and the collective range are the XV-15's [CR, bell301] in blade degrees, the stick ranges the
# XV-15's in fractions of travel.
COLL75_LEVER_SPAN = None    # (taken from bell301 with the XV-15's twist reference, in genv22)
# flaperons: 0 deg in airplane mode, 67 deg in hover for the least download [RC 6.2, fig. 18]; their schedule against
# nacelle angle and airspeed is not public: 40 deg at 60 deg of nacelle and below 100 kt, 0 deg from 160 kt or
# below 30 deg of nacelle (estimate)
FLAPERON_HOVER = 67.0
FLAPERON_CONV = 40.0        # conversion and low-speed airplane mode setting (estimate after [PD]: the conversion corridor's stall boundary
                            # is given at 40 deg flap)
FLAPERON_AILERON = 15.0     # +- deg of differential flaperon at full lateral stick (estimate; XV-15: 3.93 deg/in x 4.8)
ELEVATOR_RANGE = 20.0       # +- deg (XV-15 [TX])
RUDDER_RANGE = 20.0         # +- deg (XV-15 [TX])

# ---------------------------------------------------------------- airframe [RC fig. 4, 3D]
# wing: 382 ft^2, 45.83 ft span between the rotor centrelines, 100 in chord, -6 deg sweep, 3.5 deg dihedral, 0 deg
# incidence, A821201 section 23 % thick (as the XV-15's 64A223), full-span flaperons
S_W, B_W, C_W = 382.0, 45.83, 100.0 / 12
SWEEP_W, DIHEDRAL_W, INC_W = -6.0, 3.5, 0.0
# the angle added to the airframe's angle of attack to look up the XV-15's wing tables, which are against the
# XV-15's fuselage angle of attack [CR p. A-33]: its wing at 3 deg of incidence lifts nothing at about -3.2 deg; the
# V-22's, at 0 deg with a more cambered section, at about -2.8, from Boeing's airframe polar (C_L 0.18 at zero angle
# of attack, with the tail's -3 deg [RC fig. 28]; the model's 0.22, 0.4 deg of its lift slope more). So none: the
# zero-lift lines within 0.4 deg
WING_DALPHA = 0.0
BL_W_ROOT, BL_W_TIP = 1.25 * IN_PER_M, 6.47 * IN_PER_M       # fuselage side and the nacelle's inboard face [3D]
# quarter chord at the tip: the hub in airplane mode (1.73 m ahead of the axis) is 2 ft ahead of the leading edge
FS_W_TIP = model_to_station(-1.88 - 1.732 + 0.6096 + 0.25 * 2.54, 0, 0)[0]
WL_W_ROOT = model_to_station(0, 0, 0.15)[2]
# horizontal tail: 87.41 ft^2, 17.73 ft span, -3 deg incidence [RC]; quarter chord and height [3D]
S_H, B_H, INC_H = 87.41, 17.73, -3.0
FS_H, WL_H = model_to_station(8.25, 0, -0.46)[0], model_to_station(8.25, 0, -0.46)[2]
# two fins, 131.84 ft^2 in all, 11.17 ft span [RC]; at the stabiliser's tips [3D: BL 2.52 m]; centre of pressure [3D]
S_V, B_V = 131.84 / 2, 11.17
FS_V, BL_V, WL_V = model_to_station(8.35, 2.52, 0.25)
# fuselage: the XV-15's coefficients [CR] scaled by the length ratio, 57.33 ft [RC] / 42.08 ft (XV-15), squared for
# forces and cubed for moments; reference point the fuselage's mid-length on its axis [3D]
FUS_SCALE = 57.33 / 42.08
FS_F, WL_F = model_to_station(0.2, 0, -1.4)[0], model_to_station(0.2, 0, -1.4)[2]
# parasite drag area the XV-15's coefficients miss, ft^2, FITTED to the airframe's drag coefficient 0.067-0.077 at CL
# 0.2-0.55 (on 382 ft^2) at Mach 0.2-0.4 [RC fig. 28], flaps and gear up (airframe_polar.py)
DRAG_EXTRA = 11.5

# ---------------------------------------------------------------- mass [RC fig. 4; Navy fact file]
# MV-22B: empty 33 140 lb; maximum vertical take-off 52 600 lb, rolling 57 000, self-deployment 60 500
GW_DESIGN = 47500.0     # lb, the reference loading (the 1986 design's maximum vertical take-off weight [RC])
W_EMPTY = 33140.0
# each nacelle with its engine, proprotor gearbox and rotor, the mass that tilts: 15 % of the gross weight, as the
# XV-15's pylons (3986 lb of 13 000 [CR]); its CG 0.32 m ahead of and 0.70 m above the axis in helicopter mode (the
# XV-15's 8.3 in and 18 in scaled by 1.52, the rotor diameter ratio) (estimate)
W_NACELLE = 7000.0
DX_NAC, DZ_NAC = -0.32 * IN_PER_M, 0.70 * IN_PER_M
I_NACELLE = (660.0, 3500.0, 3080.0)     # slug ft^2, each about its own CG: about its shaft, the conversion axis and the third
                                        # axis (the XV-15's [CR] x mass ratio x 1.52^2)
# inertia of the rest of the aircraft about the CG: the XV-15's less its pylons [CR table I] scaled by the mass ratio
# and the span (roll) and length (pitch) ratios squared, 45.83 / 32.17 and 57.33 / 42.08 (estimate)
I_REST = (76800.0, 84300.0, 120000.0, 0.0)
# CG at the reference loading: under the hub in helicopter mode, 0.6 m below the wing [3D] (estimate; the V-22's CG
# limits are not public)
FS_CG, WL_CG = model_to_station(-1.55, 0, -0.55)[0], model_to_station(-1.55, 0, -0.55)[2]

# ---------------------------------------------------------------- fuel, crew and payload [FS] [GS]
# the MV-22's five tanks, 1448 US gal in all [FS]: two feed tanks in the wing (88 gal each), two forward sponson tanks
# (478 each) and the right aft sponson tank (316) [GS]: (name, US gal, FS, BL, WL), their places estimated [3D]. The
# transfer drains the aft sponson first, then the forward sponsons, the feed tanks last [GS]
FUEL_LB_GAL = 6.81          # JP-5, the Navy's (JSBSim's density for it)
TANKS = [('Left feed', 88.0) + model_to_station(-1.6, -1.5, 0.15), ('Right feed', 88.0) + model_to_station(-1.6, 1.5, 0.15),
         ('Left forward sponson', 478.0) + model_to_station(-1.3, -1.8, -2.0),
         ('Right forward sponson', 478.0) + model_to_station(-1.3, 1.8, -2.0),
         ('Right aft sponson', 316.0) + model_to_station(1.0, 1.8, -2.0)]
FUEL_START = 0.5            # the FlightGear aircraft starts half full
W_CREW = 220.0              # each of the two pilots and the crew chief [FS: 2 cockpit seats, 1 crew seat]
TROOPS = 24                 # troop seats [FS]; 240 lb each, equipped (estimate)
CARGO_MAX = 20000.0         # lb, internal cargo, the published limit (not in [FS]); the payload dialog's range
CARGO_PM = 5                # the troops-and-cargo point mass's index (genv22.point_masses)
# the reference loading, GW_DESIGN at the design CG: the empty weight, crew, the tanks full but the aft sponson (on the
# right only, the transfer's first: empty, the loading is symmetric) and the rest as cargo
TANKS_REF = [1.0, 1.0, 1.0, 1.0, 0.0]
CARGO_REF = GW_DESIGN - W_EMPTY - 3 * W_CREW - FUEL_LB_GAL * sum(t[1] * r for t, r in zip(TANKS, TANKS_REF))

# ---------------------------------------------------------------- engines and drive
# two Rolls-Royce (Allison) T406-AD-400 / AE 1107C, 6150 shp each [RC, PD], at sea level, standard day; the other
# ratings are the usual turboshaft ratios (estimate): intermediate 0.96, maximum continuous 0.81 of it
P_MRP = 6150.0
P_IRP = 0.96 * P_MRP
P_MCP = 0.81 * P_MRP
ENGINE_TAU = 0.6            # s, power response of the engine and its FADEC to the lever (estimate)
# the drive system's all-engine limit, a torque (power at 100 % rotor speed, falling with it): the hover ceiling's
# flat part from sea level to its knee at 8400 ft [RC fig. 33a] is this limit, so it is the model's power to hover at
# its sea-level 47 700 lb at 100 % (genv22.py root ... ; performance.py prints it): FITTED. [PD fig. 8] puts the limit at
# 1.4 times the power to hover (100 %) and 1.15 at 84 %, as a constant torque does.
P_XMSN_100 = 9100.0         # (9130 with rigid blades)
J_DRIVE = 2 * 60.0          # slug ft^2 at the rotor: the engines' power turbines, gearboxes and the interconnect shaft
                            # (estimate)
# ---------------------------------------------------------------- landing gear [3D]
NOSE_GEAR = model_to_station(-7.25, 0.0, -2.95)
MAIN_GEAR = model_to_station(-0.47, 2.10, -2.95)
D_GEAR = (3.0 * FUS_SCALE ** 2, 1.0 * FUS_SCALE ** 2)   # ft^2 main, nose, extended: the XV-15's [CR] scaled
# the pitch at which the fuselage's underside aft of the main wheels meets the ground, the wheels on it and their struts
# extended: the ramp's lower edge, 3.2 m aft of their contact and 0.41 m above it [3D]
TAIL_STRIKE_DEG = 7.3
EYE = model_to_station(-5.85, 0.57, -0.90)   # pilot's eye, right seat [the original aircraft's view]
