"""Blade sections of the proprotors in C-81's equation form, tabulated over +-180 deg and Mach number.

Neither rotor's section tables are public: the XV-15's metal blades use NACA 64-series sections, 64-935 at 0.09 R,
64-528 at 0.17 R, 64-118 at 0.51 R, 64-(1.5)12 at 0.8 R and 64-208 at the tip [TM-86833 fig. 8]; the V-22's Boeing
XN-series, XN28 at 0.2 R, XN18 at 0.5 R, XN12 at 0.75 R and XN09 at the tip, tested in the Boeing Supersonic Wind
Tunnel [Rosenstein and Clark, ERF 1986 paper 14, fig. 16; Johnson, AHS 2001: "the TRAM blade airfoils are the V-22
airfoils"]. So the sections are built here the way C-81's built-in airfoil equations build the NACA 0012 (the Bo 105
model's tail rotor, data/c81_0012.py in ~/git/Bo105), with the parameters a cambered section of the given thickness and
design lift has:

  lift   about the zero-lift angle alpha0 (deg): slope a(t, M) = A0 (1 - 0.5 (t - 0.09)) / sqrt(1 - M^2) per deg (held
         from M 0.8), the 0012's of C-81 at t = 0.12, M = 0 within 2 %; alpha0 = -0.8 cl_i / A0 for a design lift
         coefficient cl_i (the a = 1.0 mean line's ideal angle is zero and its lift there cl_i; NACA R-824's
         measured 64-series zero-lift angles are about 0.8 of that); maximum lift CLMAX0 (1 + 0.5 cl_i) - DCLMAX_T
         (t - 0.12)^2 times the 0012's Mach dependence of C-81 normalised to its M = 0 value (thick sections stall at
         lower lift, cambered ones higher [NACA R-824]). The 64-series sections of these thicknesses stall from the
         trailing edge [NACA R-824]: the lift curve rounds over into its maximum (a parabola from 0.85 of it) and falls
         gently past it, reaching a flat plate's normal force 1.98 sin(alpha) [Hoerner] STALL_SPAN deg on. (C-81's 0012
         falls to the plate within 6 deg: a negative lift slope as steep as the attached one, which takes away the
         gimbal's aerodynamic damping and lets it diverge at high thrust.) In reversed flow 0.78 at most
  drag   CD0(t) + C-81's 0012 polynomial in the angle from zero lift less its own 0.008; Korn's drag divergence Mach
         number Mdd = KAPPA_A - t - 0.1 |cl| (KAPPA_A 0.87 for conventional sections) with Lock's rise
         20 (M - Mdd + 0.108)^4; past the stall a flat plate's 1.98 sin^2(alpha); in reversed flow 1.5 times the attached
         drag of the angle from the trailing edge
  moment  about the quarter chord, -0.8 cl_i / 4: the a = 1.0 mean line's thin airfoil moment, -cl_i / 4, taken at 0.8
         of it as the zero-lift angle (NACA R-824's 64-series); its own change with the angle of attack and past the
         stall left out. For the blades' torsion (blademodes.py)

The constants marked FITTED are fitted to the full-scale XV-15 rotor's hover test [TM-86833] in the test programme
(hover_rotor.py); the same constants serve the V-22's sections.
"""
import math

MACH = [0.0, 0.3, 0.4, 0.5, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 1.0]
STALL_SPAN = 16.0           # deg from the maximum lift to the flat plate's
ROUND = 0.70                # FITTED: the lift curve leaves its line at this fraction of the maximum
CD_SEP = 0.06                # FITTED: separation drag at the maximum lift, rising with the square of the way to it
CN_PLATE = 1.98
A0 = 0.110                  # lift slope per deg at t = 0.09, M = 0
CLMAX0 = 1.55               # FITTED: maximum lift of a symmetric 12 % section at M = 0 (C-81's 0012: 1.334)
DCLMAX_T = 6.0              # loss of maximum lift with thickness away from 12 %, per (t - 0.12)^2
CD0_12 = 0.0065             # FITTED: minimum drag of a 12 % section (production blade, tripped; 0.0080 with rigid blades)
DCD0_T = 0.06               # rise of the minimum drag with thickness away from 12 %, per (t - 0.12)^2
KAPPA_A = 0.87              # Korn's technology factor for conventional (NACA 6-series) sections


def _mach_shape(M):
    """C-81's 0012 maximum lift against Mach number, over its M = 0 value"""
    M = min(M, 0.85)
    return (1.334 + 0.8334 * M - 4.924 * M ** 2 + 3.853 * M ** 3) / 1.334


class Section:
    def __init__(s, t, cl_i):
        s.t, s.cl_i = t, cl_i
        s.a0 = A0 * (1 - 0.5 * (t - 0.09))
        s.alpha0 = -0.8 * cl_i / s.a0
        s.clmax0 = CLMAX0 * (1 + 0.5 * cl_i) - DCLMAX_T * (t - 0.12) ** 2
        s.cd0 = CD0_12 + DCD0_T * (t - 0.12) ** 2

    def slope(s, M):
        return s.a0 / math.sqrt(1 - min(M, 0.8) ** 2)

    def cl_max(s, M, reversed_flow=False):
        return 0.78 if reversed_flow else s.clmax0 * _mach_shape(M)

    def _attached_cd(s, a, M, cl):
        """drag of the attached section at a deg from zero lift (>= 0), with the drag rise"""
        cd = s.cd0 - 0.00099 * a + 0.000278 * a * a
        mdd = max(0.45, KAPPA_A - s.t - 0.1 * abs(cl))
        if M > mdd - 0.108:
            cd += 20 * (M - mdd + 0.108) ** 4
        return cd

    def coefficients(s, alpha, M):
        """(cl, cd) at alpha (deg, -180..180, from the chord line) and Mach number M"""
        x = alpha - s.alpha0
        x = (x + 180) % 360 - 180
        sgn = -1.0 if x < 0 else 1.0
        a = abs(x)
        k = s.slope(M)
        r = math.radians(a)
        lp, dp = CN_PLATE * math.sin(r) * math.cos(r), CN_PLATE * math.sin(r) ** 2
        rev = a > 90
        b = 180.0 - a if rev else a
        cm_ = s.cl_max(M, rev)
        b1 = ROUND * cm_ / k                         # end of the line
        bs = b1 + 2 * (1 - ROUND) * cm_ / k          # the maximum: a parabola tangent to the line at b1
        f = 1.5 if rev else 1.0
        if b <= b1:
            cl = k * b
            cd = s._attached_cd(b, M, cl) * f
        elif b <= bs:
            u = (b - b1) / (bs - b1)
            cl = ROUND * cm_ + (1 - ROUND) * cm_ * (2 * u - u * u)
            cd = (s._attached_cd(b, M, cl) + CD_SEP * u * u) * f
        elif b <= bs + STALL_SPAN:
            w = (b - bs) / STALL_SPAN
            rp = math.radians(bs + STALL_SPAN)
            lp_e = CN_PLATE * math.sin(rp) * math.cos(rp)
            cl = (1 - w) * cm_ + w * lp_e
            cd0 = (s._attached_cd(bs, M, cm_) + CD_SEP) * f
            cd = (1 - w) * cd0 + w * max(CN_PLATE * math.sin(rp) ** 2, cd0)
        else:
            cl, cd = abs(lp), max(dp, s._attached_cd(min(b, bs), M, cm_))
        if rev:
            cl = -cl
        return sgn * cl, min(2.0, cd)

    def tables(s):
        """(alphas, Mach numbers, cl rows, cd rows): rows over alpha, columns over Mach"""
        al = alphas()
        cl = [[s.coefficients(x, m)[0] for m in MACH] for x in al]
        cd = [[s.coefficients(x, m)[1] for m in MACH] for x in al]
        return al, MACH, cl, cd


def alphas():
    a = list(range(0, 21)) + [22, 24, 26, 28, 30, 35, 40] + list(range(50, 170, 10)) + [170, 175, 180]
    return sorted(set([-x for x in a] + a))


def cm0(cl_i):
    """the section's moment coefficient about its quarter chord (the module's notes)"""
    return -0.8 * cl_i / 4


def interp_sections(stations, r):
    """thickness and design lift at r/R, linear between the listed (r/R, t, cl_i) stations, held beyond them"""
    if r <= stations[0][0]:
        return stations[0][1:]
    for (r0, t0, c0), (r1, t1, c1) in zip(stations, stations[1:]):
        if r <= r1:
            w = (r - r0) / (r1 - r0)
            return t0 + w * (t1 - t0), c0 + w * (c1 - c0)
    return stations[-1][1:]
