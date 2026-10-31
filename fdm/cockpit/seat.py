"""The MV-22B's crew seats: armoured, crashworthy bucket seats (no ejection seat), after public photographs of the
cockpit (refs/web/seat, its SOURCES.txt). A tall shell, wide at the pan and narrowing to its rounded top, its sides
wrapping forward low along the pan and up the back to the shoulders; a head plate at its top with the harness's
guide; the back cushion mesh above and tweed at the small of the back, the pan cushion's front raised; the four-point
harness lying in it, its rotary buckle on the pan; a padded armrest on its left, where each pilot's left forearm rests
behind the thrust control lever (the right-hand seat's on the console, the left-hand seat's on the side wall), so one
model serves both; the bucket on the frame it strokes down in a crash, standing on the floor.

    python3 seat.py      writes MV-22B/Models/Interior/Seat/ (seat.ac, seat.png), placed by all-seats.xml

The seat's frame is FlightGear's (x aft, y right, z up, m) about all-seats.xml's offsets: the pan where the old
seat's was, the back where the view's eye point (Systems/views.xml) has the pilot's head against it.
"""
import os, math, random
from PIL import Image, ImageDraw, ImageFont
import acmesh as A

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', '..', 'MV-22B', 'Models', 'Interior', 'Seat')
TEX = 'seat.png'
TEX_W = 1024

SHELL = (44, 45, 47)          # the photographs' worn grey-black (sRGB, daylight)
HEAD = (30, 31, 33)
MESH = (54, 56, 59)
TWEED = (114, 111, 107)
STRAP = (40, 41, 43)
METAL = (150, 152, 155)
PAD = (36, 36, 38)
FRAME_C = (70, 72, 75)
LABEL = (226, 188, 38)
INK = (16, 16, 16)

PAN_Z = -0.200                # the pan cushion's top at its middle (the old seat's)
BACK_X = -0.098               # the back cushion's face at the pan
RECLINE = 13.0                # deg: the back from the vertical
FLOOR_Z = -2.50 + 1.49        # the cockpit floor (Interior/interior.ac) in the seat's frame
SHELL_W, TOP_W, TOP_H = 0.530, 0.380, 0.950    # the shell's width at the pan and at its top; its top above the pan
SHOULDER = 0.58               # where it starts narrowing, its side walls ending
SHELL_T, RIM_D = 0.030, 0.050                   # the shell's thickness; its rim, forward of the back plate
CUSHION_T = 0.060
PAN_D = 0.420                 # the pan, forward of the back cushion
WING = [(-0.10, None), (0.04, None), (0.07, 0.36), (0.20, 0.26), (0.42, 0.17), (0.552, RIM_D)]
                              # the side walls: (height above the pan, how far forward of the back plate's face their
                              # edge is; None: at the pan's front), up to the shoulders
ARM = (0.240, 0.320, 0.095, 0.065)              # the armrest: its top above the pan, length, width, thickness


# ---------------------------------------------------------------- the texture
class Atlas:
    def __init__(s):
        s.im = Image.new('RGB', (TEX_W, TEX_W), SHELL)
        s.x = s.y = s.row = 0
        s.tiles = {}

    def add(s, key, w, h, draw):
        if key not in s.tiles:
            if s.x + w > TEX_W:
                s.x, s.y, s.row = 0, s.y + s.row + 2, 0
            assert s.y + h <= TEX_W, 'atlas full'
            tile = Image.new('RGB', (w, h))
            draw(ImageDraw.Draw(tile), w, h)
            s.im.paste(tile, (s.x, s.y))
            s.tiles[key] = (s.x / TEX_W, 1 - (s.y + h) / TEX_W, (s.x + w) / TEX_W, 1 - s.y / TEX_W)
            s.x += w + 2; s.row = max(s.row, h)
        return s.tiles[key]


AT = Atlas()


def solid(rgb):
    u0, v0, u1, v1 = AT.add(rgb, 8, 8, lambda d, w, h: d.rectangle((0, 0, w, h), fill=rgb))
    return A.const((u0 + u1) / 2, (v0 + v1) / 2)


def fabric(kind):
    """a cushion's cloth: tweed (flecked) or mesh (a fine weave)"""
    def draw(d, w, h):
        rnd = random.Random(kind)
        base = TWEED if kind == 'tweed' else MESH
        for y in range(h):
            for x in range(w):
                if kind == 'tweed':
                    k = rnd.gauss(0, 14) + (22 if rnd.random() < 0.06 else 0) - (18 if rnd.random() < 0.06 else 0)
                else:
                    k = rnd.gauss(0, 5) - (16 if x % 4 == 0 or y % 4 == 0 else 0)
                d.point((x, y), fill=tuple(max(0, min(255, int(c + k))) for c in base))
    return AT.add(kind, 256, 256, draw)


def label():
    def draw(d, w, h):
        d.rectangle((0, 0, w, h), fill=LABEL)
        f = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial Bold.ttf', 30)
        g = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial Bold.ttf', 15)
        d.text((10, 6), 'WARNING:', font=f, fill=INK)
        d.text((10, 44), 'DO NOT REMOVE BOLT UNLESS SEAT', font=g, fill=INK)
        d.text((10, 62), 'IS RESTED AGAINST UPPER STOP', font=g, fill=INK)
        d.polygon([(w - 40, 14), (w - 14, 26), (w - 40, 38)], fill=INK)
    return AT.add('label', 300, 90, draw)


def rect_uv(tile, a0, a1, b0, b1):
    """maps a0..a1, b0..b1 (whichever two coordinates the caller passes) onto a tile"""
    u0, v0, u1, v1 = tile
    return lambda a, b: (u0 + (u1 - u0) * (a - a0) / (a1 - a0), v0 + (v1 - v0) * (b - b0) / (b1 - b0))


# ---------------------------------------------------------------- frames and solids
class Frame:
    """an object built in a frame of its own: its (d, y, z) at origin + d ed + y ey + z ez in the seat's"""
    def __init__(s, o, origin, ed, ey, ez):
        s.o, s.origin, s.ax = o, origin, (ed, ey, ez)

    def vec(s, v):
        return tuple(sum(v[j] * s.ax[j][i] for j in range(3)) for i in range(3))

    def pt(s, p):
        return tuple(a + b for a, b in zip(s.origin, s.vec(p)))

    def face(s, pts, uvs, normal):
        s.o.face([s.pt(p) for p in pts], uvs, s.vec(normal))


SR, CR = math.sin(math.radians(RECLINE)), math.cos(math.radians(RECLINE))
UP_B, FWD_B = (SR, 0.0, CR), (-CR, 0.0, SR)                     # up the back, forward from it
B0 = (BACK_X + CUSHION_T * CR, 0.0, PAN_Z - CUSHION_T * SR)     # the back plate's face at the pan, behind the cushion


def plate_x(z, d=0.0):
    """where the back plate's face (or a plane d forward of it) is at height z"""
    h = (z - B0[2] - d * FWD_B[2]) / UP_B[2]
    return B0[0] + d * FWD_B[0] + h * UP_B[0]


def back(o):
    """o built in the back's frame: d forward from the back plate's face, y right, z up the back from the pan"""
    return Frame(o, B0, FWD_B, (0.0, 1.0, 0.0), UP_B)


def at_back(d, y, h):
    return tuple(B0[i] + d * FWD_B[i] + y * (0.0, 1.0, 0.0)[i] + h * UP_B[i] for i in range(3))


def closed(o, poly, d0, d1, uv_end, uv_side):
    """a prism capped at both ends"""
    A.prism(o, poly, d0, d1, uv_end, uv_side)
    A.flat(o, poly, d0, uv_end, normal=-1)


def box(o, x0, x1, y0, y1, z0, z1, uv):
    """a box with all six faces, in the seat's frame"""
    closed(o, [(y0, z0), (y1, z0), (y1, z1), (y0, z1)], x0, x1, uv, uv)


def quad(o, pts, uv, normal):
    o.face(list(pts), [uv(0, 0)] * len(pts), normal)


def unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def ribbon(o, pts, normals, w, t, uv, smooth=3):
    """a strap: a band w wide and t thick through pts, lying on the surfaces whose normals (one per point) are given;
    its bends rounded off (Chaikin's corner cutting), each cross-section shared by the pieces either side of it, so
    that it runs on without a seam; capped at its ends"""
    P = [(tuple(p), unit(n)) for p, n in zip(pts, normals)]
    for _ in range(smooth):
        Q = [P[0]]
        for (a, na), (b, nb) in zip(P, P[1:]):
            for f in (0.25, 0.75):
                Q.append((tuple(a[i] + f * (b[i] - a[i]) for i in range(3)), unit(tuple(na[i] + f * (nb[i] - na[i]) for i in range(3)))))
        P = Q + [P[-1]]
    secs = []
    for k, (p, n) in enumerate(P):
        a, b = P[max(0, k - 1)][0], P[min(len(P) - 1, k + 1)][0]
        tng = unit(tuple(y - x for x, y in zip(a, b)))
        side = unit(cross(tng, n)); n = cross(side, tng)
        corner = lambda sa, sb: tuple(p[i] + sa * w / 2 * side[i] + sb * t * n[i] for i in range(3))
        secs.append(([corner(-1, 0), corner(1, 0), corner(1, 1), corner(-1, 1)], n, side, tng))
    for (s0, n0, d0, _), (s1, n1, d1, _) in zip(secs, secs[1:]):
        n, d = unit(tuple(x + y for x, y in zip(n0, n1))), unit(tuple(x + y for x, y in zip(d0, d1)))
        quad(o, [s0[3], s0[2], s1[2], s1[3]], uv, n)
        quad(o, [s0[1], s0[2], s1[2], s1[1]], uv, d)
        quad(o, [s0[0], s0[3], s1[3], s1[0]], uv, tuple(-x for x in d))
    for (s, _, _, tng), sign in ((secs[0], -1), (secs[-1], 1)):
        quad(o, s, uv, tuple(sign * x for x in tng))


# ---------------------------------------------------------------- the seat
def shell_outline(inset=0.0):
    """the back plate (y, height above the pan): from its foot up its sides, narrowing above the shoulders, round
    its rounded top"""
    w, tw, r = SHELL_W / 2 - inset, TOP_W / 2 - inset, 0.07
    top = TOP_H - inset
    right = [(w, -0.09), (w, SHOULDER)]
    for k in range(1, 6):
        f = k / 5
        right.append((w + (tw - w) * f * f, SHOULDER + (top - r - SHOULDER) * f))
    for k in range(1, 6):
        a = math.radians(90 * k / 6)
        right.append((tw - r + r * math.cos(a), top - r + r * math.sin(a)))
    return right + [(-y, z) for y, z in reversed(right)]


def shell(objs):
    o = A.Obj('Seat.shell', TEX); objs.append(o)
    b, p = back(o), solid(SHELL)
    outline, inner = shell_outline(), shell_outline(SHELL_T)
    closed(b, outline, -SHELL_T, 0.0, p, p)
    # its rim, standing forward of the plate round its sides and top
    for a0, a1, c0, c1 in zip(outline, outline[1:], inner, inner[1:]):
        b.face([(RIM_D, *a0), (RIM_D, *a1), (RIM_D, *c1), (RIM_D, *c0)], [p(0, 0)] * 4, (1, 0, 0))
    A.walls(b, outline, 0.0, RIM_D, p)
    A.walls(b, inner, 0.0, RIM_D, p, outward=False)
    # the pan's plate under the cushion, from the back plate's foot to the front
    x0, x1 = BACK_X - PAN_D, plate_x(PAN_Z - 0.10, -SHELL_T)
    zb, zt = PAN_Z - 0.10, PAN_Z - 0.07
    box(o, x0, x1, -SHELL_W / 2 + SHELL_T, SHELL_W / 2 - SHELL_T, zb, zt, p)
    # the side walls: row by row up the back, from the back plate's rear to their front edge
    rows = [(plate_x(PAN_Z + h, -SHELL_T), x0 if f is None else plate_x(PAN_Z + h) - f, PAN_Z + h) for h, f in WING]
    for side in (-1, 1):
        yo, yi = side * SHELL_W / 2, side * (SHELL_W / 2 - SHELL_T)
        for (r0, f0, z0), (r1, f1, z1) in zip(rows, rows[1:]):
            for y_, s_ in ((yo, side), (yi, -side)):
                quad(o, [(r0, y_, z0), (f0, y_, z0), (f1, y_, z1), (r1, y_, z1)], p, (0, s_, 0))
            quad(o, [(f0, yo, z0), (f1, yo, z1), (f1, yi, z1), (f0, yi, z0)], p, unit((z1 - z0, 0, -(f1 - f0)) if f1 != f0 else (-1, 0, 0)))
        r0, f0, z0 = rows[0]
        quad(o, [(r0, yo, z0), (f0, yo, z0), (f0, yi, z0), (r0, yi, z0)], p, (0, 0, -1))
    # the head plate at the top, the harness's slot in it
    h = solid(HEAD)
    closed(b, A.rrect(0.26, 0.17, 0.03, 0.0, 0.83), 0.0, 0.035, h, h)
    b.face([(0.0355, -0.05, 0.765), (0.0355, 0.05, 0.765), (0.0355, 0.05, 0.778), (0.0355, -0.05, 0.778)],
           [solid((10, 10, 11))(0, 0)] * 4, (1, 0, 0))
    return o


def cushions(objs):
    o = A.Obj('Seat.cushions', TEX); objs.append(o)
    b = back(o)
    mesh, tweed = fabric('mesh'), fabric('tweed')
    # the back: mesh above, the small of the back's tweed pad below, thicker
    for z0, z1, t, tile, col in ((0.30, 0.74, CUSHION_T, mesh, MESH), (0.02, 0.30, CUSHION_T + 0.012, tweed, TWEED)):
        A.prism(b, A.rrect(0.40, z1 - z0, 0.025, 0.0, (z0 + z1) / 2), 0.0, t, rect_uv(tile, -0.20, 0.20, z0, z1), solid(col))
    # the pan: its front raised, its top tweed
    x_r, x_f = BACK_X + 0.01, BACK_X - PAN_D + 0.01
    prof = [(x_r, PAN_Z - 0.07), (x_f, PAN_Z - 0.07), (x_f - 0.005, PAN_Z + 0.010), (x_f + 0.025, PAN_Z + 0.028),
            (x_r, PAN_Z - 0.008)]
    w, side = 0.225, solid(TWEED)
    uv = rect_uv(tweed, x_f, x_r, -w, w)
    for y_ in (-w, w):
        o.face([(x, y_, z) for x, z in prof], [side(0, 0)] * len(prof), (0, y_, 0))
    cx, cz = sum(x for x, _ in prof) / len(prof), sum(z for _, z in prof) / len(prof)
    for (xa, za), (xb, zb) in zip(prof, prof[1:] + prof[:1]):
        pts = [(xa, -w, za), (xb, -w, zb), (xb, w, zb), (xa, w, za)]
        o.face(pts, [uv(x, y) for x, y, _ in pts], ((xa + xb) / 2 - cx, 0, (za + zb) / 2 - cz))
    return o


def pan_top(x):
    """the pan cushion's top at x"""
    x_r, x_f = BACK_X + 0.01, BACK_X - PAN_D + 0.035
    return PAN_Z - 0.008 + (0.036 * (x_r - x) / (x_r - x_f))


def harness(objs):
    o = A.Obj('Seat.harness', TEX); objs.append(o)
    s, m = solid(STRAP), solid(METAL)
    W, T = 0.045, 0.004
    bx = BACK_X - 0.27
    buckle = (bx, 0.0, pan_top(bx) + 0.003)
    on_pan = lambda x, y, lift: (x, y, pan_top(x) + lift)
    up, crease = (0.0, 0.0, 1.0), unit(tuple(a + b for a, b in zip(FWD_B, (0.0, 0.0, 1.0))))
    # each strap runs in under the buckle's disc to its middle, ending in it; each a little higher than the one under
    # it where they meet. The lap belts and the crotch strap come up out of the cushion
    for side in (-1, 1):
        # the shoulder strap: out of the head plate's slot, over the cushion's top, down it and over the small of the
        # back's thicker pad, round the crease and across the pan
        ribbon(o, [at_back(0.030, side * 0.03, 0.775), at_back(0.036, side * 0.03, 0.765), at_back(CUSHION_T + 0.002, side * 0.04, 0.745),
                   at_back(CUSHION_T + 0.001, side * 0.045, 0.70), at_back(CUSHION_T + 0.001, side * 0.065, 0.34),
                   at_back(CUSHION_T + 0.014, side * 0.067, 0.30), at_back(CUSHION_T + 0.013, side * 0.075, 0.03),
                   on_pan(BACK_X - 0.012, side * 0.075, 0.003), on_pan(bx + 0.05, side * 0.03, 0.003), on_pan(bx, 0.0, 0.003)],
               [FWD_B] * 7 + [crease, up, up], W, T, s)
        closed(back(o), A.rrect(0.056, 0.03, 0.006, side * 0.062, 0.445), CUSHION_T, CUSHION_T + 0.009, m, m)
        # the lap belt: out of the crease at the pan's rear corner, over the cushion
        ribbon(o, [on_pan(BACK_X + 0.008, side * 0.215, -0.006), on_pan(BACK_X - 0.01, side * 0.205, 0.002),
                   on_pan(BACK_X - 0.08, side * 0.16, 0.002), on_pan(bx + 0.035, side * 0.045, 0.002), on_pan(bx, 0.0, 0.002)],
               [up] * 5, W, T, s)
    ribbon(o, [on_pan(BACK_X - PAN_D + 0.07, 0.0, -0.006), on_pan(BACK_X - PAN_D + 0.085, 0.0, 0.001), on_pan(bx, 0.0, 0.001)],
           [up] * 3, W, T, s)
    # the rotary buckle: a disc, its turning knob on it
    f = Frame(o, buckle, (0, 0, 1), (0, 1, 0), (-1, 0, 0))
    A.cylinder(f, 0.038, 0, 0, 0.0, 0.014, m, m, n=28)
    A.cylinder(f, 0.024, 0, 0, 0.014, 0.020, solid((96, 98, 101)), m, n=24)
    return o


def armrest(objs):
    """on the seat's left, its top ARM[0] above the pan, from behind the back forward"""
    o = A.Obj('Seat.armrest', TEX); objs.append(o)
    top, length, w, t = ARM
    p, f = solid(PAD), solid(FRAME_C)
    yc = -SHELL_W / 2 - w / 2 - 0.004
    xr = at_back(0, 0, top)[0] + 0.02
    zc = PAN_Z + top - t / 2
    ring = lambda x, k: [(x, y, z) for y, z in A.rrect(w * k, t * k, 0.022 * k, yc, zc)]
    A.loft(o, [ring(xr, 1.0), ring(xr - length + 0.02, 1.0), ring(xr - length, 0.8)], p)
    # its post down the shell's side to the pan's foot, a bracket into the shell
    box(o, xr - 0.06, xr - 0.02, yc - 0.015, yc + 0.015, PAN_Z - 0.10, PAN_Z + top - t, f)
    box(o, xr - 0.06, xr - 0.02, yc + 0.015, -SHELL_W / 2, PAN_Z - 0.09, PAN_Z - 0.04, f)
    return o


def placard(objs):
    """the yellow WARNING placard on the left side wall's outer face, low and forward"""
    o = A.Obj('Seat.placard', TEX); objs.append(o)
    x0, x1, z0, z1 = BACK_X - 0.36, BACK_X - 0.22, PAN_Z - 0.095, PAN_Z - 0.053
    y = -SHELL_W / 2 - 0.0006
    uv = rect_uv(label(), x0, x1, z0, z1)                 # read from its left, standing outboard of it: forward to aft
    pts = [(x0, y, z0), (x1, y, z0), (x1, y, z1), (x0, y, z1)]
    o.face(pts, [uv(x, z) for x, _, z in pts], (0, -1, 0))
    return o


def frame(objs):
    """the bucket's frame: two rails up behind the back plate, legs under the pan's front, tubes along the floor"""
    o = A.Obj('Seat.frame', TEX); objs.append(o)
    f = solid(FRAME_C)
    x_rail = at_back(-SHELL_T, 0, 0.62)[0] + 0.03
    x_leg = BACK_X - PAN_D + 0.06
    for side in (-1, 1):
        y = side * 0.17
        box(o, x_rail - 0.02, x_rail + 0.02, y - 0.025, y + 0.025, FLOOR_Z, PAN_Z + 0.64, f)                    # the rail
        box(o, at_back(-SHELL_T, 0, -0.10)[0] - 0.01, x_rail - 0.02, y - 0.015, y + 0.015, PAN_Z - 0.10, PAN_Z - 0.06, f)
        box(o, at_back(-SHELL_T, 0, 0.57)[0] - 0.005, x_rail - 0.02, y - 0.015, y + 0.015, PAN_Z + 0.55, PAN_Z + 0.59, f)
        box(o, x_leg - 0.015, x_leg + 0.015, y - 0.015, y + 0.015, FLOOR_Z, PAN_Z - 0.10, f)                   # front leg
        box(o, x_leg - 0.015, x_rail + 0.02, y - 0.02, y + 0.02, FLOOR_Z, FLOOR_Z + 0.03, f)                    # on the floor
    box(o, x_rail - 0.015, x_rail + 0.015, -0.145, 0.145, FLOOR_Z + 0.25, FLOOR_Z + 0.28, f)
    return o


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    objs = []
    for build in (shell, cushions, harness, armrest, placard, frame):
        build(objs)
    A.write(os.path.join(OUT, 'seat.ac'), objs)
    AT.im.save(os.path.join(OUT, TEX))
    print('%d objects, %d faces' % (len(objs), sum(len(o.S) for o in objs)))
