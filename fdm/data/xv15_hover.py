"""Hover performance of the full-scale XV-15 rotor (metal blades), isolated, out of ground effect: NASA TM-86833
(Felker, Betzina and Signor, 1985), fig. 14(c), tip Mach number 0.685-0.695: the report's curve fit through the
wind-corrected data (CP/sigma against CT/sigma, sigma 0.0891), traced from the scan (fdm/digitize.py's calibration,
about +-0.0001 in CP/sigma). Fig. 13(c) gives the figure of merit (0.79 at its peak, CT/sigma 0.12; 0.68 at 0.18) and
fig. 17 CT/sigma against the collective (from the actuator's position, +-1 deg): zero thrust at about -7.5 deg and
CT/sigma 0.18 at 14 deg, a gradient of 0.0085 per deg."""
SIGMA = 0.0891
MTIP = 0.69
CT_S = [0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.10, 0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17, 0.18]
CP_S = [0.00256, 0.00211, 0.00218, 0.00262, 0.00322, 0.00385, 0.00464, 0.00549, 0.00644, 0.00754, 0.00865, 0.00982,
        0.01117, 0.01269, 0.01419, 0.01607, 0.01824, 0.02039, 0.02364]
COLL_ZERO_THRUST = -7.5
DCTS_DCOLL = 0.0085
CTS_MAX_TESTED = 0.181
