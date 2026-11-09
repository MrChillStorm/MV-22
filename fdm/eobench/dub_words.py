"""all Dubins words onto the circle at E for one geometry: dub_words.py x y psi_deg R0 R Lf kt"""
import sys, math
TP = 2 * math.pi
m2 = lambda a: a % TP
def words(x, y, psi, R0, R, Lf, kt, sds=(1, -1)):
    out = []
    for sd in sds:
        C2 = (-Lf, sd * R)
        for s1 in (1, -1):
            C1 = (x - s1 * R0 * math.sin(psi), y + s1 * R0 * math.cos(psi))
            dx, dy = C2[0] - C1[0], C2[1] - C1[1]; d2 = dx * dx + dy * dy
            rho = sd * R - s1 * R0
            if d2 > rho * rho:
                L = math.sqrt(d2 - rho * rho); phi = math.atan2(dy, dx) - math.atan2(rho, L)
                a1, a2 = m2(s1 * (phi - psi)), m2(-sd * phi)
                out.append(('%sS%s' % ('RL'[s1 < 0], 'RL'[sd < 0]), L + kt * (a1 + a2), L, a1, 0, a2))
            if s1 == sd:      # CCC: the middle circle the other way, R0, tangent to both outside
                d = math.sqrt(d2); A, B = 2 * R0, R0 + R
                if abs(A - B) <= d <= A + B:
                    g = math.acos((A * A + d * d - B * B) / (2 * A * d)); base = math.atan2(dy, dx)
                    for sg in (1, -1):
                        Cm = (C1[0] + A * math.cos(base + sg * g), C1[1] + A * math.sin(base + sg * g))
                        u1 = math.atan2(Cm[1] - C1[1], Cm[0] - C1[0]); u2 = math.atan2(C2[1] - Cm[1], C2[0] - Cm[0])
                        h1, h2 = u1 + s1 * math.pi / 2, u2 - s1 * math.pi / 2
                        a1, am, a2 = m2(s1 * (h1 - psi)), m2(-s1 * (h2 - h1)), m2(-s1 * h2)
                        out.append(('%s%s%s' % ('RL'[s1 < 0], 'LR'[s1 < 0], 'RL'[sd < 0]), kt * (a1 + am + a2), 0, a1, am, a2))
    return sorted(out, key=lambda w: w[1])
if __name__ == '__main__':
    x, y, psi, R0, R, Lf, kt = map(float, sys.argv[1:8])
    for w in words(x, y, math.radians(psi), R0, R, Lf, kt):
        print('%-4s cost %6.0f  straight %6.0f  turns %4.0f %4.0f %4.0f deg = %4.0f' % (w[0], w[1], w[2], *[math.degrees(a) for a in w[3:]], math.degrees(sum(w[3:]))))
