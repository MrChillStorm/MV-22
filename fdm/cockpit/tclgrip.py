#!/usr/bin/env python3
"""The thrust control lever's grip, carved from three photographs of it (front, top and oblique from aft right;
refs/web/tcl/front.png, top.jpg, iso.png, not included): its mesh and where its switches are, for panel.py.

The grip's frame (m): d up from the base's bottom, y aft, z right. The front view sets the scale: the grip is 0.172 m
wide across its body and head (the cockpit photographs, against the keypads), so 0.117 m tall. The front and top
views are taken as orthographic, the top one rotated in its picture (its front edge 32 deg off the horizontal) and
scaled by the same width; the oblique view's camera (perspective) is fitted so that its outline matches the hull of
the other two. A voxel grid (1.5 mm) is carved by the three outlines; what no outline can decide is sculpted after the
photographs: the palm rest's top falls from the front toward its round rear end, crowned, its edges rounded; aft of
the base it is a cap over an overhang. The hull is smoothed, meshed (surface nets on a 4.5 mm grid, Taubin smoothing)
and its details found by rays from the cameras through their pixels.

    python3 tclgrip.py [--refit] [--check out.png]
        writes data/tcl_grip.json (and a comparison with the photographs); --refit fits the cameras again
        (else it keeps the ones the file has)
"""
import os, sys, json, math
import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage, optimize

HERE = os.path.dirname(os.path.abspath(__file__))
PHOTOS = os.path.join(HERE, '..', '..', 'refs', 'web', 'tcl')
OUT = os.path.join(HERE, 'data', 'tcl_grip.json')
WIDTH = 0.172                   # m: across the body and head
LEFT = -0.055                   # the body's left side in the frame
VOX = 0.0015
GROW = {'front': 2, 'top': 2, 'iso': 6}        # px of tolerance: the oblique close-up's camera is the least sure
MESH_STEP, MESH_SIGMA = 3, 2.0


# ---------------------------------------------------------------- the outlines
def largest(m):
    lab, n = ndimage.label(m)
    sizes = ndimage.sum(m, lab, range(1, n + 1))
    return ndimage.binary_fill_holes(lab == 1 + int(np.argmax(sizes)))


def outline(name):
    f = {'front': 'front.png', 'top': 'top.jpg', 'iso': 'iso.png'}[name]
    im = np.asarray(Image.open(os.path.join(PHOTOS, f)).convert('RGBA')).astype(float)
    rgb, a = im[..., :3], im[..., 3]
    if name == 'top':                          # on a light grey background
        fg = ndimage.binary_closing(rgb.mean(axis=2) < 150, iterations=3)
    else:                                      # cut out
        fg = ndimage.binary_closing((a > 128) & (rgb.min(axis=2) < 235), iterations=2)
    return largest(ndimage.binary_opening(fg, iterations=1))


TOP_RIGHT, TOP_AFT = np.array([0.845, -0.534]), np.array([0.534, 0.845])    # the grip's right and aft, in the top view


def clip_front(m):
    """the top view without the finger switches standing out of the front edge (parts of their own): the body's
    outline, left of the head, clipped at its front face, 21 mm behind the head's front"""
    yy, xx = np.mgrid[0:m.shape[0], 0:m.shape[1]]
    a = xx * TOP_RIGHT[0] + yy * TOP_RIGHT[1]; f = -(xx * TOP_AFT[0] + yy * TOP_AFT[1])
    sc = (a[m].max() - a[m].min()) / WIDTH
    z = (a - a[m].min()) / sc + LEFT
    return m & ~((z < 0.060) & (f > f[m].max() - 0.021 * sc))


MASK = {k: outline(k) for k in ('front', 'top', 'iso')}
MASK['top'] = clip_front(MASK['top'])

Z = np.arange(-0.080, 0.125, VOX); Y = np.arange(-0.075, 0.125, VOX); D = np.arange(-0.005, 0.130, VOX)
GD, GY, GZ = np.meshgrid(D, Y, Z, indexing='ij')
P = np.stack([GD.ravel(), GY.ravel(), GZ.ravel()], axis=1)


# ---------------------------------------------------------------- the cameras
def rot(axis, a):
    a = math.radians(a); c, s = math.cos(a), math.sin(a)
    x, y, z = axis
    return np.array([[c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
                     [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
                     [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)]])


# orthographic views: the picture's right and down directions (d, y, z) before small rotations
BASES = {'front': ((0, 0, -1), (-1, 0, 0)),                                   # looking aft
         'top': ((0, TOP_AFT[0], TOP_RIGHT[0]), (0, TOP_AFT[1], TOP_RIGHT[1]))}  # looking down


def ortho_axes(view, yaw, pitch, roll):
    r, dn = np.array(BASES[view][0], float), np.array(BASES[view][1], float)
    R = rot(np.cross(r, dn), roll) @ rot(r, pitch) @ rot(dn, yaw)
    return R @ r, R @ dn


AIM = np.array([0.060, 0.025, 0.015])          # the oblique camera looks at about the grip's middle


def iso_frame(cam):
    """the oblique camera's eye and axes (d, y, z): azimuth from aft toward the left, elevation, distance"""
    az, el, dist = cam[:3]
    a, e = math.radians(az), math.radians(el)
    eye = AIM + dist * np.array([math.sin(e), math.cos(a) * math.cos(e), -math.sin(a) * math.cos(e)])
    fwd = AIM - eye; fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [1.0, 0, 0]); right /= np.linalg.norm(right)       # (d, y, z): up is +d
    up = np.cross(right, fwd)
    return eye, fwd, right, up


def project(view, cam, pts):
    if view == 'iso':
        du, dv, roll, fov = cam[3:]
        eye, fwd, right, up = iso_frame(cam)
        v = pts - eye
        zc = v @ fwd
        H, W = MASK['iso'].shape
        f = (H / 2) / math.tan(math.radians(fov) / 2)
        x, y = v @ right / zc * f, v @ up / zc * f
        cr, sr = math.cos(math.radians(roll)), math.sin(math.radians(roll))
        x, y = cr * x - sr * y, sr * x + cr * y
        return W / 2 + du + x, H / 2 + dv - y
    yaw, pitch, roll, s, cx, cy = cam
    r, dn = ortho_axes(view, yaw, pitch, roll)
    return cx + s * (pts @ r), cy + s * (pts @ dn)


def inside(view, cam, pts, grow=0):
    m = ndimage.binary_dilation(MASK[view], iterations=grow) if grow else MASK[view]
    u, v = project(view, cam, pts)
    iu, iv = np.round(u).astype(int), np.round(v).astype(int)
    ok = (iu >= 0) & (iu < m.shape[1]) & (iv >= 0) & (iv < m.shape[0])
    out = np.zeros(len(pts), bool)
    out[ok] = m[iv[ok], iu[ok]]
    return out


def carve(cams, views):
    occ = np.ones(len(P), bool)
    for v in views:
        occ &= inside(v, cams[v], P, GROW[v])
    return occ


def silhouette(view, cam, occ):
    m = MASK[view]
    pts = P[occ]
    if len(pts) > 60000:
        pts = pts[np.random.default_rng(0).choice(len(pts), 60000, replace=False)]
    u, v = project(view, cam, pts)
    iu, iv = np.round(u).astype(int), np.round(v).astype(int)
    ok = (iu >= 0) & (iu < m.shape[1]) & (iv >= 0) & (iv < m.shape[0])
    img = np.zeros(m.shape, bool); img[iv[ok], iu[ok]] = True
    return ndimage.binary_fill_holes(ndimage.binary_closing(ndimage.binary_dilation(img, iterations=3), iterations=3))


def iou(a, b):
    return (a & b).sum() / max(1, (a | b).sum())


def fit(view, cam, occ, steps, iters, free, lim):
    """the camera matching the view's outline to the hull's, its free parameters each within lim of their start"""
    p0 = list(cam)
    def full(q):
        p = list(p0)
        for i, x in zip(free, q):
            p[i] = x
        return p
    def loss(q):
        p = full(q)
        return 1 - iou(MASK[view], silhouette(view, p, occ)) + 0.01 * sum(max(0.0, abs(p[i] - p0[i]) - l) ** 2 for i, l in lim.items())
    q, st = [p0[i] for i in free], [steps[i] for i in free]
    for k in range(2):
        simplex = [q] + [[x + (s if i == j else 0) for j, (x, s) in enumerate(zip(q, st))] for i in range(len(q))]
        q = list(optimize.minimize(loss, q, method='Nelder-Mead', options={'maxiter': iters, 'initial_simplex': simplex,
                                                                             'xatol': 1e-3, 'fatol': 1e-4}).x)
        st = [x * 0.4 for x in st]
    p = full(q)
    return p, iou(MASK[view], silhouette(view, p, occ))


def cameras():
    m = MASK['front']; ys, xs = np.nonzero(m)
    s = (xs.max() - xs.min()) / WIDTH
    front = [0.0, 0.0, 0.0, s, xs.max() + LEFT * s, ys.max()]           # its left side at LEFT, its foot at d = 0
    m = MASK['top']; yy, xx = np.nonzero(MASK['top'] | outline('top'))
    a = xx * TOP_RIGHT[0] + yy * TOP_RIGHT[1]; b = xx * TOP_AFT[0] + yy * TOP_AFT[1]
    st = (a.max() - a.min()) / WIDTH
    a0, b0 = a.min() - LEFT * st, b.min() + 0.045 * st                     # its front 45 mm ahead of y = 0
    top = [0.0, 0.0, 0.0, st, TOP_RIGHT[0] * a0 + TOP_AFT[0] * b0, TOP_RIGHT[1] * a0 + TOP_AFT[1] * b0]
    cams = {'front': front, 'top': top}
    occ = carve(cams, ['front', 'top'])
    best = None
    for az in (-170, -140, -110, -80, -50, -20, 20):
        p, q = fit('iso', [az, 30.0, 0.40, 0.0, 0.0, 0.0, 30.0], occ, [20.0, 10.0, 0.08, 60.0, 60.0, 4.0, 0.0], 150,
                   [0, 1, 2, 3, 4, 5], {5: 8.0})
        if best is None or q > best[1]:
            best = (p, q)
    cams['iso'], q = fit('iso', best[0], occ, [6.0, 6.0, 0.06, 30.0, 30.0, 3.0, 8.0], 400, list(range(7)), {5: 8.0, 6: 30.0})
    return cams


# ---------------------------------------------------------------- the solid
def smoothstep(x):
    x = np.clip(x, 0, 1)
    return x * x * (3 - 2 * x)


def rounded_box(y, z, y0, y1, z0, z1, r):
    """inside a box in plan (y, z) with its corners rounded"""
    yc, zc, hy, hz = (y0 + y1) / 2, (z0 + z1) / 2, (y1 - y0) / 2, (z1 - z0) / 2
    qy, qz = np.maximum(abs(y - yc) - (hy - r), 0), np.maximum(abs(z - zc) - (hz - r), 0)
    return (abs(y - yc) <= hy) & (abs(z - zc) <= hz) & (qy * qy + qz * qz <= r * r)


HEAD_LEAN = (18.0, 6.0)        # deg: the head's rear face (the switch plate, near square to the oblique camera) and front


def sculpt(occ, below):
    """within the carve (occ; below: the base's carve, the oblique view's outline with its notch under the head closed,
    whose rays would cut slabs out of the base), what no outline can show, after the photographs: the head a prong, its rear and front faces leaning in
    above the palm rest; the palm rest's top falling toward its round rear end, crowned, its edges rounded and rising
    into the head's foot; aft of the base a cap over an overhang; the base a block"""
    d, y, z = GD, GY, GZ
    base = below & (d <= 0.038) & rounded_box(y, z, -0.030, 0.070, -0.027, 0.083, 0.008)
    rise = np.maximum(d - 0.060, 0)
    head = occ & (z >= 0.040) & (d >= 0.036) & (y <= 0.025 - math.tan(math.radians(HEAD_LEAN[0])) * rise) & \
        (y >= -0.043 + math.tan(math.radians(HEAD_LEAN[1])) * rise)
    plan = occ[np.argmin(abs(D - 0.065))]
    edge = ndimage.distance_transform_edt(plan) * VOX
    R = 0.013
    rim = np.where(edge < R, R - np.sqrt(np.maximum(0, R * R - (R - edge) ** 2)), 0.0)
    top = (0.093 - 0.025 * smoothstep(y / 0.092) - 0.004 * (z / 0.052) ** 2 - rim[None, :, :]
           + 0.022 * smoothstep((z - 0.022) / 0.028) * smoothstep((0.030 - y) / 0.020))          # into the head's foot
    under = 0.024 + 0.021 * smoothstep((y - 0.040) / 0.035)
    body = occ & (z < 0.056) & (d >= under) & (d <= top)
    return base | head | body


def surface_nets(F, iso=0.5):
    """the iso surface of F (inside: F > iso): vertices (index space), quads and each quad's outward axis"""
    nd, ny, nz = F.shape
    ins = F > iso
    corners = [(a, b, c) for a in (0, 1) for b in (0, 1) for c in (0, 1)]
    cv = np.stack([F[a:nd - 1 + a, b:ny - 1 + b, c:nz - 1 + c] for a, b, c in corners], axis=-1)
    ci = cv > iso
    active = ci.any(-1) & ~ci.all(-1)
    edges = [(i, j) for i in range(8) for j in range(i + 1, 8) if sum(abs(p - q) for p, q in zip(corners[i], corners[j])) == 1]
    idx = np.argwhere(active); vals = cv[active]
    acc = np.zeros((len(idx), 3)); cnt = np.zeros(len(idx))
    for i, j in edges:
        fa, fb = vals[:, i], vals[:, j]
        x = (fa > iso) != (fb > iso)
        t = np.where(x, (iso - fa) / np.where(fb - fa == 0, 1, fb - fa), 0)
        pa, pb = np.array(corners[i], float), np.array(corners[j], float)
        acc += x[:, None] * (pa + t[:, None] * (pb - pa)); cnt += x
    verts = idx + acc / cnt[:, None]
    vid = -np.ones(active.shape, int); vid[active] = np.arange(len(idx))
    quads, axes = [], []
    for ax in range(3):
        sl0 = [slice(None)] * 3; sl1 = [slice(None)] * 3
        sl0[ax] = slice(0, -1); sl1[ax] = slice(1, None)
        a, b = ins[tuple(sl0)], ins[tuple(sl1)]
        cross = np.argwhere(a != b)
        sign = np.where(a[tuple(cross.T)], 1, -1)
        o = [k for k in range(3) if k != ax]
        for p, sg in zip(cross, sign):
            cells = []
            for du, dv in ((0, 0), (1, 0), (1, 1), (0, 1)):
                c = list(p); c[o[0]] -= du; c[o[1]] -= dv
                if min(c) < 0 or c[0] >= nd - 1 or c[1] >= ny - 1 or c[2] >= nz - 1:
                    cells = None; break
                cells.append(vid[tuple(c)])
            if cells and min(cells) >= 0:
                quads.append(cells); n = [0, 0, 0]; n[ax] = int(sg); axes.append(n)
    return verts, np.array(quads), np.array(axes)


def taubin(verts, quads, n=30, lam=0.5, mu=-0.53):
    nb = [set() for _ in verts]
    for q in quads:
        for i in range(4):
            a, b = q[i], q[(i + 1) % 4]; nb[a].add(b); nb[b].add(a)
    nb = [np.array(sorted(s)) for s in nb]
    v = verts.copy()
    for _ in range(n):
        for f in (lam, mu):
            v = v + f * (np.array([v[s].mean(axis=0) if len(s) else v[i] for i, s in enumerate(nb)]) - v)
    return v


def mesh(occ):
    F = ndimage.gaussian_filter(occ.reshape(GD.shape).astype(float), MESH_SIGMA)
    verts, quads, axes = surface_nets(np.pad(F[::MESH_STEP, ::MESH_STEP, ::MESH_STEP], 1))
    verts = taubin(verts, quads)
    origin = np.array([D[0], Y[0], Z[0]]) - VOX * MESH_STEP
    return origin + verts * VOX * MESH_STEP, quads, axes, F


# ---------------------------------------------------------------- its details, by rays through the photographs
ISO_PX = {'white button': (553, 78), 'lamp': (601, 95), 'hat': (605, 165), 'screw left': (480, 205),
          'screw right': (665, 215), 'grey switch': (648, 284), 'thumbwheel': (503, 340), 'nacelle label': (528, 287),
          'rocker': (742, 285), 'friction knob': (547, 478)}
FRONT_PX = {'panel': (372, 237), 'button right': (275, 220), 'rocker right': (332, 232), 'rocker left': (402, 232),
            'button left': (467, 225)}


def details(cams, F):
    G = np.gradient(F)
    def at(A, p):
        return ndimage.map_coordinates(A, [[(p[0] - D[0]) / VOX], [(p[1] - Y[0]) / VOX], [(p[2] - Z[0]) / VOX]], order=1)[0]
    def hit(o, dvec):
        for t in np.arange(0.0, 0.8, 0.0004):
            p = o + t * dvec
            if D[0] < p[0] < D[-1] and Y[0] < p[1] < Y[-1] and Z[0] < p[2] < Z[-1] and at(F, p) > 0.5:
                n = -np.array([at(g, p) for g in G])
                return p, n / np.linalg.norm(n)
        raise ValueError('a ray missed the grip')
    def iso_ray(u, v):
        du, dv, roll, fov = cams['iso'][3:]
        eye, fwd, right, up = iso_frame(cams['iso'])
        H, W = MASK['iso'].shape
        f = (H / 2) / math.tan(math.radians(fov) / 2)
        x, y = u - W / 2 - du, -(v - H / 2 - dv)
        cr, sr = math.cos(math.radians(-roll)), math.sin(math.radians(-roll))
        x, y = cr * x - sr * y, sr * x + cr * y
        dw = fwd + x / f * right + y / f * up
        return eye, dw / np.linalg.norm(dw)
    def front_ray(u, v):
        s, cx, cy = cams['front'][3:]
        return np.array([(cy - v) / s, Y[0], (cx - u) / s]), np.array([0.0, 1.0, 0.0])
    out = {}
    for px, ray in ((ISO_PX, iso_ray), (FRONT_PX, front_ray)):
        for name, (u, v) in px.items():
            o, dvec = ray(u, v)
            p, n = hit(o, dvec)
            if name in ('white button', 'grey switch'):        # round in the photograph: facing the camera
                n = n - dvec; n = n / np.linalg.norm(n)
            out[name] = {'p': np.round(p, 4).tolist(), 'n': np.round(n, 3).tolist(), 'view': np.round(-dvec, 3).tolist()}
    # NACELLE is printed just above the thumbwheel, on the body's rounded edge toward the thumb
    t = np.array(out['thumbwheel']['p'])
    p, n = hit(np.array([D[-1], t[1] - 0.004, t[2] - 0.004]), np.array([-1.0, 0.0, 0.0]))   # from above
    out['nacelle label'] = {'p': np.round(p, 4).tolist(), 'n': np.round(n, 3).tolist(), 'view': out['nacelle label']['view']}
    # the friction knob stands on the lobe the ray met: its foot on the lobe's top, found from below
    p = np.array(out['friction knob']['p']) - np.array([0.0, 0.008, 0.0])
    d = 0.002
    while at(F, (d, p[1], p[2])) > 0.5:
        d += 0.0005
    out['friction knob'] = {'p': [round(d, 4), round(p[1], 4), round(p[2], 4)], 'n': [1.0, 0.0, 0.0]}
    return out


def check(path, cams, pts, quads):
    """the mesh, flat shaded, with each camera beside its photograph"""
    tiles = []
    for v, f in (('iso', 'iso.png'), ('front', 'front.png'), ('top', 'top.jpg')):
        ph = Image.open(os.path.join(PHOTOS, f)).convert('RGBA')
        bg = Image.new('RGBA', ph.size, (255, 255, 255, 255)); bg.alpha_composite(ph)
        u, w = project(v, cams[v], pts)
        if v == 'iso':
            depth = np.linalg.norm(pts - iso_frame(cams[v])[0], axis=1)
        else:
            r, dn = ortho_axes(v, *cams[v][:3]); depth = pts @ np.cross(r, dn)
        im = Image.new('RGB', ph.size, 'white'); dr = ImageDraw.Draw(im)
        light = np.array([0.8, 0.45, -0.35]); light /= np.linalg.norm(light)
        for q in quads[np.argsort(-depth[quads].mean(axis=1))]:
            p = pts[q]; n = np.cross(p[2] - p[0], p[3] - p[1]); nn = np.linalg.norm(n)
            c = int(40 + 160 * (0.25 + 0.75 * abs(n @ light) / nn if nn > 0 else 0.5))
            dr.polygon([(u[i], w[i]) for i in q], fill=(c, c, c + 4))
        pair = Image.new('RGB', (ph.width * 2, ph.height), 'white')
        pair.paste(bg.convert('RGB'), (0, 0)); pair.paste(im, (ph.width, 0))
        tiles.append(pair.resize((900, int(900 * ph.height / (2 * ph.width)))))
    side = [90.0, 15.0, 0.45, 0.0, 0.0, 0.0, 30.0]                  # from the left, a little above
    u, w = project('iso', side, pts)
    depth = np.linalg.norm(pts - iso_frame(side)[0], axis=1)
    im = Image.new('RGB', (815, 679), 'white'); dr = ImageDraw.Draw(im)
    light = np.array([0.8, -0.3, -0.5]); light /= np.linalg.norm(light)
    for q in quads[np.argsort(-depth[quads].mean(axis=1))]:
        p = pts[q]; n = np.cross(p[2] - p[0], p[3] - p[1]); nn = np.linalg.norm(n)
        c = int(40 + 160 * (0.25 + 0.75 * abs(n @ light) / nn if nn > 0 else 0.5))
        dr.polygon([(u[i], w[i]) for i in q], fill=(c, c, c + 4))
    tiles.append(im.resize((450, 375)))
    out = Image.new('RGB', (900, sum(t.height for t in tiles)), 'white'); y = 0
    for t in tiles:
        out.paste(t, (0, y)); y += t.height
    out.save(path)


if __name__ == '__main__':
    if '--refit' in sys.argv or not os.path.exists(OUT):
        cams = cameras()
    else:
        cams = json.load(open(OUT))['cameras']
    below = carve(cams, ['front', 'top']) & inside('iso', cams['iso'], P, 12)      # its notch closed
    occ = sculpt(carve(cams, ['front', 'top', 'iso']).reshape(GD.shape), below.reshape(GD.shape)).ravel()
    for v in ('front', 'top', 'iso'):
        print('%-5s outline overlap %.3f' % (v, iou(MASK[v], silhouette(v, cams[v], occ))))
    pts, quads, axes, F = mesh(occ)
    feats = details(cams, F)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump({'note': 'written by fdm/cockpit/tclgrip.py: the TCL grip in its frame (m: d up, y aft, z right)',
               'cameras': {k: [round(float(x), 4) for x in v] for k, v in cams.items()},
               'verts': np.round(pts, 5).tolist(), 'quads': quads.tolist(), 'axes': axes.tolist(), 'details': feats},
              open(OUT, 'w'), separators=(',', ':'))
    print('%d vertices, %d quads; %s' % (len(pts), len(quads), os.path.relpath(OUT, HERE)))
    if '--check' in sys.argv:
        check(sys.argv[sys.argv.index('--check') + 1], cams, pts, quads)
