"""Test data in vortex ring state from Johnson's review [W. Johnson, Model for Vortex Ring State Influence on Rotorcraft
Flight Dynamics, NASA/TP-2005-213477, 2005], traced from the NTRS scan rendered at 300 dpi: each figure's frame
calibrates its axes (758 px across), the markers are found by their colour and their centres taken; markers run
together are split by k-means on their pixel count (+-0.005 in the velocities).

Fig. 2: the flight tests' VRS encounters against the rotor's horizontal and vertical velocity over vh (Vx/vh, Vz/vh;
vh the ideal hover induced velocity at the aircraft's weight): the V-22's roll-offs [Kisor et al., AHS 60th Forum, 2004],
the Dauphin D6075's Vz drops and its stabilisations after them [Taghizad et al.]. The markers the figure's own boundary
line crosses (five open diamonds, at Vz/vh -1.28 to -1.46) are classified by eye, the rest by their fill.

Fig. 44: Betzina's tiltrotor rotor in axial descent, rotor only [AHS 57th Forum, 2001; sigma 0.119, twist -41 deg]: the
total inflow (Vz + v)/vh against Vz/vh, v from the rotor's torque and from its thrust (Johnson's reduction)."""

V22_ROLLOFF = [(0.200, -0.500), (0.519, -0.588), (0.695, -0.914), (0.754, -0.681), (0.769, -0.654), (0.772, -0.903),
               (0.833, -0.697), (0.849, -0.944), (0.860, -0.726)]
D6075_DROP = [(0.016, -0.641), (0.038, -0.573), (0.039, -0.673), (0.044, -0.484), (0.121, -0.405), (0.126, -0.553),
              (0.164, -0.385), (0.203, -0.432), (0.342, -0.434), (0.416, -0.381), (0.430, -0.418), (0.455, -0.511),
              (0.473, -0.751), (0.494, -0.345), (0.566, -0.535), (0.578, -0.475), (0.582, -0.919), (0.720, -0.751),
              (0.786, -0.949), (0.824, -0.810)]
D6075_STAB = [(0.044, -1.345), (0.044, -1.236), (0.060, -1.602), (0.082, -1.444), (0.082, -1.365), (0.118, -1.325),
              (0.147, -1.459), (0.165, -1.186), (0.170, -1.424), (0.191, -1.275), (0.203, -1.177), (0.210, -1.320),
              (0.253, -1.236), (0.294, -1.400), (0.313, -1.216), (0.340, -1.028), (0.346, -1.354), (0.346, -1.166),
              (0.369, -1.048), (0.379, -1.187), (0.385, -1.275), (0.407, -1.434), (0.431, -1.384), (0.439, -1.196),
              (0.439, -1.117), (0.468, -1.216), (0.478, -1.048), (0.539, -0.998), (0.539, -0.909), (0.577, -1.156)]
BETZINA_AXIAL_Q = [(-1.702, 1.463), (-1.639, 0.572), (-1.568, 0.730), (-1.437, 0.825), (-1.370, 1.241), (-0.998, 1.087),
                   (-0.954, 1.008), (-0.705, 0.945), (-0.637, 0.984), (-0.515, 1.011)]
BETZINA_AXIAL_T = [(-1.702, 1.423), (-1.636, 0.568), (-1.568, 0.845), (-1.437, 0.861), (-1.370, 1.277), (-1.026, 1.114),
                   (-0.978, 1.035), (-0.697, 0.897), (-0.645, 1.011), (-0.510, 0.940)]
BETZINA_KAPPA = 1.05      # the induced power factor Johnson's model is drawn with against these data

# Fig. 66: Johnson's V-22 encounter, symmetric, calculated (the black curves, traced every 2 s): Vz/vh (vh 65.2 ft/s) and
# the normal acceleration (g) against time (s) from the collective's step
JOHNSON_ENC_T = [2, 4, 6, 8, 10, 12, 14, 16, 18]
JOHNSON_ENC_VZ = [-0.422, -0.497, -0.567, -0.653, -0.790, -1.002, -1.223, -1.373, -1.452]
JOHNSON_ENC_NZ = [0.928, 0.928, 0.919, 0.888, 0.810, 0.746, 0.806, 0.874, 0.918]
