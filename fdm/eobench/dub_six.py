"""the six paths (each side's circle, each way of the first turn, now; the cross path after the straight that frees it)
against a fine search over the straight first: dub_six.py x y psi_deg R0 R Lf kt"""
import sys, math
from dub_words import words
TP = 2 * math.pi
def six(x, y, psi, R0, R, Lf, kt):
    out = []
    for sd in (1, -1):
        for w in words(x, y, psi, R0, R, Lf, kt, (sd,)):
            if 'S' in w[0][1]:
                out.append((w[1], w[0], 0.0))
        s1 = -sd                                  # the cross path: straight until the circles part
        C1 = (x - s1 * R0 * math.sin(psi), y + s1 * R0 * math.cos(psi)); C2 = (-Lf, sd * R)
        dx, dy = C1[0] - C2[0], C1[1] - C2[1]; ud = math.cos(psi) * dx + math.sin(psi) * dy
        q = ud * ud - (dx * dx + dy * dy) + (R + R0) ** 2
        if q > 0:
            s0 = -ud + math.sqrt(q) + 1
            if s0 > 1:
                w = [v for v in words(x + s0 * math.cos(psi), y + s0 * math.sin(psi), psi, R0, R, Lf, kt, (sd,)) if v[0] == '%sS%s' % ('RL'[s1 < 0], 'RL'[sd < 0])]
                if w:
                    out.append((s0 + w[0][1], 'S+' + w[0][0], s0))
    return sorted(out)
if __name__ == '__main__':
    x, y, psi, R0, R, Lf, kt = map(float, sys.argv[1:8]); psi = math.radians(psi)
    for c, n, s0 in six(x, y, psi, R0, R, Lf, kt)[:3]:
        print('  %-6s cost %6.0f  straight first %5.0f' % (n, c, s0))
    best = min((s0 + w[1], s0, w[0]) for s0 in range(0, 30001, 100) for w in words(x + s0 * math.cos(psi), y + s0 * math.sin(psi), psi, R0, R, Lf, kt) if 'S' in w[0][1])
    print('  search: cost %6.0f, %s after %d ft straight' % best[:1] + (best[2], best[1]) if False else '  search: cost %6.0f, %s after %d ft straight' % (best[0], best[2], best[1]))
