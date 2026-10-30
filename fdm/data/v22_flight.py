"""V-22 performance and trim data, digitised on grids fitted to the figures' ticks (+-1 % of weight, +-0.3 deg, +-1 kt).

  [PD]  Dunford (Bell), "The V-22 Osprey - a significant flight test challenge", ERF 1990, paper II.7.3: fig. 7
        rotor/wing lift sharing, fig. 8 power required for level flight, fig. 9a pitch attitude in level flight, by
        nacelle angle. The figures give no weight, altitude or flap setting. Fig. 9a's airplane-mode curve (13.6 deg at
        108 kt falling to 0.2 at 170 kt) needs a lift coefficient of 0.82 at zero angle of attack with [RC fig. 28]'s
        slope, flaps down, and so about 31 500 lb at sea level, below the MV-22B's empty weight (33 140 lb): the
        comparisons run at 34 000 lb, a light aircraft (performance.py)
  [RC]  Rosenstein and Clark (Boeing Vertol), ERF 1986, paper 14: fig. 33a hover ceiling (standard day), fig. 33c
        flight envelope at the 42 000 lb combat weight (standard day), for the 6150 shp engines of the time
"""
# airframe lift, % of the weight, against calibrated airspeed (kt), by nacelle angle [PD fig. 7]
LIFT_SHARE = {90: [(0, -9.7), (20, -1.5), (30, 1.8), (40, 3.6), (50, 4.8), (60, 6.6), (70, 7.9), (82, 7.1)],
              75: [(37, 9.4), (50, 13.0), (60, 15.3), (70, 17.6), (80, 20.2), (90, 22.7), (101, 26.5)],
              60: [(60, 22.7), (70, 29.1), (80, 36.2), (90, 42.1), (100, 47.4), (110, 51.3), (120, 53.3), (126, 53.8)],
              30: [(97, 80.1), (110, 82.7), (130, 84.9), (150, 86.5), (168, 87.0)],
              0: [(116, 94.6), (130, 96.2), (150, 98.0), (170, 99.2), (184, 99.0)]}
# pitch attitude (deg) against calibrated airspeed (kt), by nacelle angle [PD fig. 9a]
PITCH = {90: [(0, 1.5), (20, 0.3), (30, -0.7), (40, -2.0), (50, -4.0), (60, -5.8), (70, -7.8), (80, -9.9), (88, -11.9)],
         75: [(28, 10.8), (40, 9.3), (50, 7.8), (60, 5.9), (70, 3.7), (80, 1.6), (90, -0.4), (100, -2.3), (110, -4.3),
              (120, -6.2), (128, -8.3)],
         60: [(58, 14.9), (70, 11.4), (80, 8.4), (90, 5.5), (100, 2.8), (110, 0.2), (120, -2.4), (130, -4.6), (140, -6.4),
              (150, -7.8)],
         30: [(96, 11.1), (100, 9.5), (110, 6.0), (120, 3.2), (130, 1.0), (140, -1.1), (150, -2.7), (160, -4.2), (170, -5.3),
              (178, -6.2)],
         0: [(108, 13.6), (120, 8.9), (130, 6.4), (140, 4.3), (150, 2.6), (160, 1.4), (170, 0.2), (180, -0.8), (190, -1.5),
             (196, -1.9)]}
# all-engine transmission limit over the power to hover [PD fig. 8]: 1.4 at 100 % rotor speed, about 1.15 at 84 %
# (a constant torque: 1.4 x 0.84 = 1.18); airplane mode, flaps up, reaches the 84 % limit at about 265 KCAS
XMSN_OVER_HOVER_100, XMSN_OVER_HOVER_84, APLN_XMSN_KCAS = 1.4, 1.15, 265.0
# hover ceiling, standard day: (gross weight lb, altitude ft) [RC fig. 33a]: flat to the knee (the transmission limit),
# then the engines' power
HOVER_CEILING = [(47700, 0), (43300, 8400), (31000, 16000)]
# flight envelope at 42 000 lb, standard day: (KTAS, altitude ft) [RC fig. 33c]; the requirement 275 kt at sea level
ENVELOPE_42K = [(274, 0), (321, 17000), (235, 26300)]
