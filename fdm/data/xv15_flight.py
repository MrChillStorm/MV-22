"""XV-15 level-flight rotor shaft power from flight test, digitised from NASA TM-84222 (Churchill and Dugan,
"Simulation of the XV-15 tilt rotor research aircraft", 1982), fig. 20 ("XV-15 level-flight performance"): flight
points of rotor shaft horsepower (both rotors) against calibrated airspeed, by nacelle angle and flap angle. The
figure gives neither the gross weight nor the altitudes; its fastest airplane-mode point is 300 KTAS at 16 000 ft, and
the flight envelope of NASA TM-81244 (fig. 15) puts most of the programme's points between 2000 and 10 000 ft density
altitude. Traced on a grid fitted to the figure's ticks (fdm/digitize.py's calibration; about +-15 hp, +-0.5 kt).

    (calibrated airspeed kt, rotor shaft horsepower)
"""
# nacelle 90 deg (helicopter mode), flaps 20 deg; the hover point is at zero airspeed
NAC90_F20 = [(0.0, 1855), (44.7, 1048), (64.9, 1090), (64.9, 1194), (74.9, 1323), (84.1, 1540), (88.0, 1655), (94.3, 1892)]
NAC60_F20 = [(97.1, 1113), (106.9, 1269), (117.0, 1398), (127.1, 1662), (131.9, 1923), (134.3, 2014)]
NAC30_F20 = [(126.3, 1190), (137.5, 1323), (146.6, 1435), (156.1, 1646), (166.6, 1960)]
NAC0_F20 = [(134.3, 945), (156.7, 1171), (161.4, 1218), (167.2, 1365), (180.2, 1606)]
# airplane mode, flaps up; the last point at 300 KTAS, 16 000 ft
NAC0_F0 = [(156.4, 984), (156.4, 1055), (171.3, 1074), (182.0, 1150), (182.0, 1269), (188.5, 1278), (202.6, 1495),
           (202.6, 1555), (211.9, 1662), (222.5, 1831), (226.3, 1923), (232.0, 2143)]
FLAP20_FLAPERON = 12.5      # the XV-15's flaperon at 20 deg of flap [TM X-62407 fig. 3.4.1: 0.625 of the flap]
SETS = {'nac90': (90.0, 12.5, NAC90_F20), 'nac60': (60.0, 12.5, NAC60_F20), 'nac30': (30.0, 12.5, NAC30_F20),
        'nac0f20': (0.0, 12.5, NAC0_F20), 'nac0': (0.0, 0.0, NAC0_F0)}

# the trim elevator, deg trailing edge down, against calibrated airspeed, from the same report's fig. 21 ("XV-15
# longitudinal control versus airspeed"; the sets by nacelle and flap as fig. 20's, other flights; its lines are Bell's
# simulation's): (KCAS, deg), read off a 250 dpi scan against the figure's ticks (about +-0.5 deg, +-2 kt)
TRIM_DE = {'nac90': [(44.4, 2.43), (44.2, 2.03), (54.9, 1.65), (65.1, 3.54), (66.9, 3.54), (75.5, 6.76), (76.9, 6.24),
                     (83.6, 7.59), (93.3, 9.1), (96.7, 10.1)],
           'nac60': [(97.4, 2.08), (106.9, 4.92), (109.5, 6.19), (117.9, 7.32), (117.9, 8.86), (128.3, 9.62), (128.3, 10.97),
                     (132.6, 11.11), (134.7, 11.68), (135.4, 12.97)],
           'nac30': [(127.7, 6.0), (138.1, 7.65), (148.3, 8.68), (158.7, 9.68), (168.8, 10.6)],
           'nac0f20': [(138.6, -0.81), (158.3, 1.08), (162.4, 1.08), (168.0, 1.35), (170.7, 3.24), (179.3, 3.54), (183.4, 3.84),
                       (180.3, 2.11)],
           'nac0': [(160.2, -2.43), (167.6, -1.89), (179.3, -1.84)]}
