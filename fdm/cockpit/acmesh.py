"""A small AC3D writer for the cockpit instruments.

Objects are built in the instrument's own frame: d toward the crew, y right, z up (metres), which is FlightGear's
model frame (x aft, y right, z up); AC3D stores it as (d, z, -y). Every face is wound so that it faces the way it is
given (its outward normal), whatever order the points come in.
"""
import math

MATERIALS = ['MATERIAL "DefaultWhite" rgb 1 1 1  amb 1 1 1  emis 0 0 0  spec 0.5 0.5 0.5  shi 64  trans 0',
             'MATERIAL "glass" rgb 0.9 0.9 0.9  amb 0 0 0  emis 0 0 0  spec 0.64 0.64 0.64  shi 50  trans 0.85']
GLASS = 1


class Obj:
    def __init__(s, name, texture, mat=0):
        s.name, s.texture, s.mat = name, texture, mat
        s.V, s.S, s.idx = [], [], {}

    def v(s, p):
        k = tuple(round(c, 7) for c in p)
        if k not in s.idx:
            s.idx[k] = len(s.V); s.V.append(k)
        return s.idx[k]

    def face(s, pts, uvs, normal):
        """A planar polygon (convex, or star-shaped from its first point) facing `normal` (d, y, z)."""
        n = [0.0, 0.0, 0.0]
        for a, b in zip(pts, pts[1:] + pts[:1]):           # Newell's normal
            n[0] += (a[1] - b[1]) * (a[2] + b[2]); n[1] += (a[2] - b[2]) * (a[0] + b[0]); n[2] += (a[0] - b[0]) * (a[1] + b[1])
        if sum(x * y for x, y in zip(n, normal)) < 0:
            pts, uvs = pts[::-1], uvs[::-1]
        s.S.append([(s.v(p), u, w) for p, (u, w) in zip(pts, uvs)])

    def lines(s):
        out = ['OBJECT poly', 'name "%s"' % s.name, 'loc 0 0 0']
        if s.texture:
            out += ['texture "%s"' % s.texture, 'texrep 1 1']
        out += ['crease 40', 'numvert %d' % len(s.V)] + ['%.6f %.6f %.6f' % (d, z, -y) for d, y, z in s.V]
        out.append('numsurf %d' % len(s.S))
        for f in s.S:
            out += ['SURF 0x10', 'mat %d' % s.mat, 'refs %d' % len(f)] + ['%d %.5f %.5f' % r for r in f]
        return out + ['kids 0']


def write(path, objs):
    out = ['AC3Db'] + MATERIALS + ['OBJECT world', 'kids %d' % len(objs)]
    for o in objs:
        out += o.lines()
    open(path, 'w').write('\n'.join(out) + '\n')


# ---------------------------------------------------------------- outlines (y, z), counter-clockwise

def rrect(w, h, r, cy=0.0, cz=0.0, n=4):
    """Rounded rectangle, w x h, corner radius r."""
    r = min(r, w / 2, h / 2); out = []
    for (sy, sz, a0) in ((1, -1, -90), (1, 1, 0), (-1, 1, 90), (-1, -1, 180)):
        oy, oz = cy + sy * (w / 2 - r), cz + sz * (h / 2 - r)
        for k in range(n + 1):
            a = math.radians(a0 + 90 * k / n)
            out.append((oy + r * math.cos(a), oz + r * math.sin(a)))
    return out


def circle(r, cy=0.0, cz=0.0, n=32):
    return [(cy + r * math.cos(2 * math.pi * k / n), cz + r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def ray_hit(poly, cy, cz, a):
    """Where a ray from (cy, cz) at angle a leaves a star-shaped polygon."""
    dy, dz = math.cos(a), math.sin(a); best = None
    for (y0, z0), (y1, z1) in zip(poly, poly[1:] + poly[:1]):
        ey, ez = y1 - y0, z1 - z0; den = dy * ez - dz * ey
        if abs(den) < 1e-12:
            continue
        t = ((y0 - cy) * ez - (z0 - cz) * ey) / den; u = ((y0 - cy) * dz - (z0 - cz) * dy) / den
        if t > 0 and -1e-9 <= u <= 1 + 1e-9 and (best is None or t < best):
            best = t
    return cy + best * dy, cz + best * dz


# ---------------------------------------------------------------- solids

def flat(o, poly, d, uv, normal=1):
    """A flat polygon at depth d facing the crew (normal +1) or away (-1); uv(y, z) -> (u, v)."""
    o.face([(d, y, z) for y, z in poly], [uv(y, z) for y, z in poly], (normal, 0, 0))


def walls(o, poly, d0, d1, uv, outward=True):
    """The side walls of a prism from d0 to d1, facing out of the outline (or into it, for a bore)."""
    n = len(poly)
    cy = sum(p[0] for p in poly) / n; cz = sum(p[1] for p in poly) / n
    for (y0, z0), (y1, z1) in zip(poly, poly[1:] + poly[:1]):
        my, mz = (y0 + y1) / 2 - cy, (z0 + z1) / 2 - cz
        s = 1 if outward else -1
        o.face([(d0, y0, z0), (d0, y1, z1), (d1, y1, z1), (d1, y0, z0)], [uv(y0, z0), uv(y1, z1), uv(y1, z1), uv(y0, z0)],
               (0, s * my, s * mz))


def prism(o, poly, d0, d1, uv_front, uv_side):
    flat(o, poly, d1, uv_front); walls(o, poly, d0, d1, uv_side)


def bezel(o, outer, inner, d, uv, cy=0.0, cz=0.0, n=48):
    """A flat ring between two star-shaped outlines (around (cy, cz)) at depth d, facing the crew."""
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        p = [ray_hit(inner, cy, cz, a0), ray_hit(outer, cy, cz, a0), ray_hit(outer, cy, cz, a1), ray_hit(inner, cy, cz, a1)]
        o.face([(d, y, z) for y, z in p], [uv(y, z) for y, z in p], (1, 0, 0))


def cylinder(o, r, cy, cz, d0, d1, uv_top, uv_side, n=20):
    poly = circle(r, cy, cz, n)
    prism(o, poly, d0, d1, uv_top, uv_side)


def box(o, y0, y1, z0, z1, d0, d1, uv_top, uv_side):
    prism(o, [(y0, z0), (y1, z0), (y1, z1), (y0, z1)], d0, d1, uv_top, uv_side)


def const(u, v):
    return lambda y, z: (u, v)


def planar(y0, y1, z0, z1, u0=0.0, u1=1.0, v0=0.0, v1=1.0):
    """Maps the rectangle y0..y1, z0..z1 onto the texture rectangle u0..u1, v0..v1."""
    return lambda y, z: (u0 + (u1 - u0) * (y - y0) / (y1 - y0), v0 + (v1 - v0) * (z - z0) / (z1 - z0))


def loft(o, rings, uv):
    """A solid through rings of points (d, y, z), each convex and as many as the first, its ends capped."""
    n = len(rings[0])
    cen = [tuple(sum(p[i] for p in r) / n for i in range(3)) for r in rings]
    for ra, rb, ca, cb in zip(rings, rings[1:], cen, cen[1:]):
        mid = tuple((x + y) / 2 for x, y in zip(ca, cb))
        for k in range(n):
            q = [ra[k], ra[(k + 1) % n], rb[(k + 1) % n], rb[k]]
            o.face(q, [uv(0, 0)] * 4, tuple(sum(p[i] for p in q) / 4 - mid[i] for i in range(3)))
    o.face(list(rings[0]), [uv(0, 0)] * n, tuple(x - y for x, y in zip(cen[0], cen[1])))
    o.face(list(rings[-1]), [uv(0, 0)] * n, tuple(x - y for x, y in zip(cen[-1], cen[-2])))


def beam(o, p0, p1, w, uv):
    """A square beam of side w from point p0 to p1 (d, y, z), with an end cap at p1."""
    ax = [b - a for a, b in zip(p0, p1)]; n = math.sqrt(sum(c * c for c in ax)); ax = [c / n for c in ax]
    ref = (1, 0, 0) if abs(ax[0]) < 0.9 else (0, 0, 1)
    u = [ax[1] * ref[2] - ax[2] * ref[1], ax[2] * ref[0] - ax[0] * ref[2], ax[0] * ref[1] - ax[1] * ref[0]]
    nu = math.sqrt(sum(c * c for c in u)); u = [c / nu for c in u]
    v = [ax[1] * u[2] - ax[2] * u[1], ax[2] * u[0] - ax[0] * u[2], ax[0] * u[1] - ax[1] * u[0]]
    corner = lambda p, a, b: tuple(p[i] + w / 2 * (a * u[i] + b * v[i]) for i in range(3))
    sq = [(1, 1), (-1, 1), (-1, -1), (1, -1)]
    for (a0, b0), (a1, b1) in zip(sq, sq[1:] + sq[:1]):
        nrm = tuple((a0 + a1) * u[i] + (b0 + b1) * v[i] for i in range(3))
        o.face([corner(p0, a0, b0), corner(p0, a1, b1), corner(p1, a1, b1), corner(p1, a0, b0)],
               [uv(0, 0)] * 4, nrm)
    o.face([corner(p1, a, b) for a, b in sq], [uv(0, 0)] * 4, tuple(ax))
