#!/usr/bin/env python3
"""The MV-22B's instrument panel, glareshield and centre console: the 3-D model, its legends and its animations.

Laid out after public photographs of the MV-22B's cockpit (Wikimedia Commons, US Navy, refs/web/cockpit), its sizes
from the four multifunction displays' 6 in square screens [AT: airforce-technology.com, "four 6in square colour active
matrix liquid crystal multifunction displays, a shared control display unit and a standby flight display"], fitted to
the 3-D model's cockpit (the old panel's frame and width):

- the glareshield, its automatic flight control panel (the coupled modes, CRS / HDG / SPEED / ALT / HVR ALT with their
  readouts and HOLD keys, AUTO NAC, HVR CPLD, REM HVR) and a radio panel at each end;
- the four multifunction displays (MFD 1-4 from the left), each with five keys above the screen, seven on each side,
  C/A and ACK below, the OFF / NIGHT / DAY knob, the VID, BRT and CON rockers;
- in the centre the backup engine and fuel display (a Barco SFD [Aircraft Engineering and Aerospace Technology, 2000])
  and the electronic standby flight instrument, which replaced the standby gauges [DOT&E FY2012 annual report, MV-22],
  the control display unit (CDU) with its seven keys a side and CDU, C/A, STAT, EICAS, ACK, CDU below, and its two
  keypads straight below it in its plane, the centre pedestal as wide as the CDU;
- both pilots' thrust control levers (their grips after photographs) on their quadrants.

Frame: the old panel's (Models/Interior/Panel/all-panels.xml places it): d toward the crew, y right, z up along the
face, metres; the face is d = 0, the multifunction displays' row centred at z = 0.137. The screens are Canvas surfaces
(Nasal/displays.nas), lit through Models/Effects/Lit (the UH-60 model's): their objects carry plain 0..1 texture
coordinates. Every key is its own object, clickable.

    python3 panel.py      writes MV-22B/Models/Interior/Panel/MV22B/ (panel.ac, panel.xml, legends.png, screen.png, rwr.png)
"""
import os, sys, math, json
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import acmesh as A

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', '..', 'MV-22B', 'Models', 'Interior', 'Panel', 'MV22B')
TEX = 'legends.png'
ATLAS_W, ATLAS_H = 4096, 2048

PAINT = (30, 31, 33)          # the panels' near-black grey (sRGB; the photographs' in daylight)
BEZEL = (21, 22, 23)          # the displays' bezels
KEY = (15, 15, 16)            # key caps
LETTER = (232, 232, 228)      # legends
SCREEN_OFF = (6, 7, 9)


def font(px, narrow=False):
    for name in (('Arial Narrow Bold.ttf', 'ARIALNB.TTF') if narrow else ('Arial Bold.ttf', 'arialbd.ttf')):
        try:
            return ImageFont.truetype(name, max(6, int(px)))
        except OSError:
            pass
    sys.exit('Arial Bold is not among the system fonts')


# ---------------------------------------------------------------- the legend atlas
class Atlas:
    """Tiles packed into one ATLAS_W x ATLAS_H texture, each where it lies highest under those before it (a skyline,
    2 px apart); each tile's uv rectangle"""
    def __init__(s):
        s.im = Image.new('RGB', (ATLAS_W, ATLAS_H), PAINT)
        s.sky = np.zeros(ATLAS_W, int)          # each column's first free row
        s.tiles = {}

    def add(s, key, w, h, draw):
        if key in s.tiles:
            return s.tiles[key]
        w, h = int(math.ceil(w)), int(math.ceil(h))
        under = np.lib.stride_tricks.sliding_window_view(s.sky, w + 2).max(axis=1)
        x0 = int(np.argmin(under)); y0 = int(under[x0])
        assert y0 + h + 2 <= ATLAS_H, 'atlas full'
        s.sky[x0:x0 + w + 2] = y0 + h + 2
        x, y = x0 + 1, y0 + 1
        tile = Image.new('RGB', (w, h), PAINT)
        draw(ImageDraw.Draw(tile), w, h)
        s.im.paste(tile, (x, y))
        uv = (x / ATLAS_W, 1 - (y + h) / ATLAS_H, (x + w) / ATLAS_W, 1 - y / ATLAS_H)     # u0, v0, u1, v1
        s.tiles[key] = uv
        return uv


AT = Atlas()


def solid(rgb):
    """a plain colour's uv (the centre of a small tile)"""
    u0, v0, u1, v1 = AT.add(('solid',) + tuple(rgb), 8, 8, lambda d, w, h: d.rectangle((0, 0, w, h), fill=rgb))
    return A.const((u0 + u1) / 2, (v0 + v1) / 2)


def fitted(d, text, w, h, size, narrow=False, fill=LETTER, cx=None, cy=None):
    """text centred at (cx, cy) (default the tile's centre), shrunk to fit w"""
    lines = text.split('\n')
    f = font(size, narrow)
    while size > 6 and max(d.textlength(l, font=f) for l in lines) > w * 0.92:
        size -= 1; f = font(size, narrow)
    lh = size * 1.08
    cx = w / 2 if cx is None else cx; cy = h / 2 if cy is None else cy
    y = cy - lh * len(lines) / 2
    for l in lines:
        d.text((cx, y + lh / 2), l, font=f, fill=fill, anchor='mm')
        y += lh


ARROWS = {'R': '→', 'L': '←', 'D': '↓', 'U': '↑'}
KEY_PX = 6000                 # px per m on key caps


def key_uv(legend, w, h, boxed=False):
    """a key cap's legend tile"""
    def draw(d, W, H):
        d.rectangle((0, 0, W, H), fill=KEY)
        if legend:
            t = ARROWS.get(legend, legend)
            fitted(d, t, W, H, H * (0.62 if t in ARROWS.values() else 0.36 if '\n' not in t else 0.30),
                   narrow=len(t.replace('\n', '')) > 5)
            if boxed:
                m = int(H * 0.16); d.rectangle((W * 0.26, m, W * 0.74, H - m), outline=LETTER, width=max(1, int(H * 0.05)))
    u0, v0, u1, v1 = AT.add(('key', legend, round(w, 4), round(h, 4), boxed), w * KEY_PX, h * KEY_PX, draw)
    return u0, v0, u1, v1


# ---------------------------------------------------------------- objects
OBJS = []
XML = []             # animations and effects
KEYS = []            # (object, unit, key id) for the pick animations


class Tilt:
    """the centre console's frame: the panel's, turned back toward the crew by beta (deg) about the line d = 0,
    z = pivot (the CDU's top edge), so that what is built on the panel's face below the pivot lies on the console"""
    def __init__(s, pivot, beta):
        s.p, s.cb, s.sb = pivot, math.cos(math.radians(beta)), math.sin(math.radians(beta))

    def vec(s, v):
        d, y, z = v
        return (d * s.cb - z * s.sb, y, d * s.sb + z * s.cb)

    def pt(s, p):
        d, y, z = s.vec((p[0], p[1], p[2] - s.p))
        return (d, y, z + s.p)


class Tilted:
    """an object built through a Tilt"""
    def __init__(s, o, t):
        s.o, s.t = o, t

    def face(s, pts, uvs, normal):
        s.o.face([s.t.pt(p) for p in pts], uvs, s.t.vec(normal))


FRAME = None         # the Tilt the objects being made are built through (None: the panel's face)


def obj(name, tex=TEX):
    o = A.Obj(name, tex); OBJS.append(o)
    return Tilted(o, FRAME) if FRAME else o


def tex_rect(uv, y0, y1, z0, z1):
    u0, v0, u1, v1 = uv
    return A.planar(y0, y1, z0, z1, u0, u1, v0, v1)


def key(name, unit, kid, legend, cy, cz, w, h, d0, height=0.006, boxed=False):
    """a key: a box from d0 out by height, its cap the legend; clickable (unit, kid)"""
    o = f = obj(name)
    uv = key_uv(legend, w, h, boxed)
    A.box(f, cy - w / 2, cy + w / 2, cz - h / 2, cz + h / 2, d0, d0 + height, tex_rect(uv, cy - w / 2, cy + w / 2, cz - h / 2, cz + h / 2),
          solid(KEY))
    if unit:
        KEYS.append((name, unit, kid))
    return o


CENTRES = {}         # each knob's axis: the (d, y, z) of its foot and its direction, into the panel


def knob(name, cy, cz, r, d0, height=0.014, legend=None):
    """a round knob with a white index line, clickable by the XML's own bindings (knob_xml)"""
    CENTRES[name] = (FRAME.pt((d0, cy, cz)), FRAME.vec((-1, 0, 0))) if FRAME else ((d0, cy, cz), (-1, 0, 0))
    o = f = obj(name)
    top = solid(KEY)
    A.cylinder(f, r, cy, cz, d0, d0 + height * 0.35, solid(BEZEL), solid(BEZEL), n=24)      # skirt
    A.cylinder(f, r * 0.78, cy, cz, d0 + height * 0.35, d0 + height, top, solid(KEY), n=20)
    A.box(f, cy - r * 0.08, cy + r * 0.08, cz + r * 0.2, cz + r * 0.74, d0 + height, d0 + height + 0.0006, solid(LETTER), solid(LETTER))
    return o


def rocker(name, unit, kid, cy, cz, w, h, d0, legend):
    """a rocker (two halves, up and down)"""
    up = key(name + '-up', unit, kid + '+', '▲', cy, cz + h / 4, w, h / 2, d0, 0.005)
    dn = key(name + '-dn', unit, kid + '-', '▼', cy, cz - h / 4, w, h / 2, d0, 0.005)
    return up, dn


def screen(name, cy, cz, w, h, d):
    """a Canvas surface: its own 0..1 texture coordinates on screen.png (black)"""
    o = obj(name, 'screen.png')
    A.flat(o, [(cy - w / 2, cz - h / 2), (cy + w / 2, cz - h / 2), (cy + w / 2, cz + h / 2), (cy - w / 2, cz + h / 2)], d,
           A.planar(cy - w / 2, cy + w / 2, cz - h / 2, cz + h / 2))
    return o


def face_tile(key_, w, h, px, draw):
    """a printed face: w x h m at px per m"""
    return AT.add(key_, w * px, h * px, lambda d, W, H: draw(d, W, H, W / w))


def plate(name, cy, cz, w, h, d0, d1, uv_front, color=PAINT):
    o = f = obj(name)
    A.box(f, cy - w / 2, cy + w / 2, cz - h / 2, cz + h / 2, d0, d1, uv_front, solid(color))
    return o


# ---------------------------------------------------------------- the multifunction displays
MFD_W, MFD_H, MFD_D = 0.218, 0.216, 0.022            # the bezel; its face at d = MFD_D
SCREEN = 0.1524                                       # 6 in square
MFD_Y = [-0.546, -0.312, 0.312, 0.546]
MFD_Z = 0.137
MFD_CUT = (0.023, 0.016)                              # the bezel's corners cut off: along its sides, along top and foot
SCREW = (44, 45, 47)
FACE_PX = 3000


def mfd_face(d, W, H, k):
    """the bezel's printing: the keys' leader lines, OFF NIGHT DAY, VID, BRT, CON"""
    d.rectangle((0, 0, W, H), fill=BEZEL)
    c = lambda y, z: (W / 2 + y * k, H / 2 - z * k)                # metres from the bezel's centre to px
    s = SCREEN / 2
    lw = max(1, int(0.0006 * k))
    for i in range(7):
        z = 0.0645 - 0.0215 * i
        for side in (-1, 1):
            d.line([c(side * 0.084, z), c(side * (s + 0.002), z - 0.002)], fill=LETTER, width=lw)
    for i in range(5):
        y = -0.052 + 0.026 * i
        d.line([c(y, 0.083), c(y + 0.002, s + 0.002)], fill=LETTER, width=lw)
    f = font(0.0028 * k)
    for t, (y, z) in (('OFF', (-0.081, 0.103)), ('NIGHT', (-0.077, 0.097)), ('DAY', (-0.078, 0.091))):
        d.text(c(y, z), t, font=f, fill=LETTER, anchor='lm')
    for t, (y, z) in (('VID', (0.0935, 0.0915)), ('BRT', (-0.0935, -0.0880)), ('CON', (0.0935, -0.0880))):
        d.text(c(y + 0.009, z), t, font=f, fill=LETTER, anchor='lm')


def mfd(n, cy):
    name = 'MFD%d' % n
    face = face_tile('mfd-face', MFD_W, MFD_H, FACE_PX, mfd_face)
    cz = MFD_Z
    o = obj(name + '.bezel')
    # the bezel: its face printed, its corners cut off back to the panel, where its screws are, a recess for the
    # screen (the face's hole: four strips, the top and the foot's corners cut)
    s = SCREEN / 2 + 0.004
    uvf = tex_rect(face, cy - MFD_W / 2, cy + MFD_W / 2, cz - MFD_H / 2, cz + MFD_H / 2)
    y0, y1, z0, z1 = cy - MFD_W / 2, cy + MFD_W / 2, cz - MFD_H / 2, cz + MFD_H / 2
    cw, ch = MFD_CUT
    A.flat(o, [(y0, cz + s), (y1, cz + s), (y1, z1 - ch), (y1 - cw, z1), (y0 + cw, z1), (y0, z1 - ch)], MFD_D, uvf)
    A.flat(o, [(y0, z0 + ch), (y0 + cw, z0), (y1 - cw, z0), (y1, z0 + ch), (y1, cz - s), (y0, cz - s)], MFD_D, uvf)
    for (a0, a1) in ((y0, cy - s), (cy + s, y1)):
        A.flat(o, [(a0, cz - s), (a1, cz - s), (a1, cz + s), (a0, cz + s)], MFD_D, uvf)
    outline = [(y0 + cw, z0), (y1 - cw, z0), (y1, z0 + ch), (y1, z1 - ch), (y1 - cw, z1), (y0 + cw, z1), (y0, z1 - ch), (y0, z0 + ch)]
    A.walls(o, outline, -0.01, MFD_D, solid(BEZEL))
    r = (cw + ch - math.hypot(cw, ch)) / 2                          # the screws: in the cut corners' middles
    for sy, sz in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        sc = (cy + sy * (MFD_W / 2 - r), cz + sz * (MFD_H / 2 - r))
        A.cylinder(o, 0.0042, *sc, 0.0, 0.0025, solid(SCREW), solid(SCREW), n=16)
        A.cylinder(o, 0.0017, *sc, 0.0025, 0.0027, solid(SCREEN_OFF), solid(SCREEN_OFF), n=10)
    A.walls(o, [(cy - s, cz - s), (cy + s, cz - s), (cy + s, cz + s), (cy - s, cz + s)], 0.010, MFD_D, solid(SCREEN_OFF), outward=False)
    screen(name + '.screen', cy, cz, SCREEN, SCREEN, 0.0105)
    kw, kh = 0.016, 0.0115
    for i in range(7):
        z = cz + 0.0645 - 0.0215 * i
        key('%s.L%d' % (name, i + 1), name, 'L%d' % (i + 1), 'R', cy - 0.0935, z, kw, kh, MFD_D)
        key('%s.R%d' % (name, i + 1), name, 'R%d' % (i + 1), 'L', cy + 0.0935, z, kw, kh, MFD_D)
    for i in range(5):
        key('%s.T%d' % (name, i + 1), name, 'T%d' % (i + 1), 'D', cy - 0.052 + 0.026 * i, cz + 0.0915, kw, kh, MFD_D)
    key(name + '.CA', name, 'CA', 'C/A', cy - 0.048, cz - 0.0935, 0.020, 0.011, MFD_D)
    key(name + '.ACK', name, 'ACK', 'ACK', cy + 0.048, cz - 0.0935, 0.020, 0.011, MFD_D)
    rocker(name + '.BRT', name, 'BRT', cy - 0.0935, cz - 0.0880, 0.011, 0.016, MFD_D, 'BRT')
    rocker(name + '.CON', name, 'CON', cy + 0.0935, cz - 0.0880, 0.011, 0.016, MFD_D, 'CON')
    rocker(name + '.VID', name, 'VID', cy + 0.0935, cz + 0.0915, 0.010, 0.014, MFD_D, 'VID')
    knob(name + '.MODE', cy - 0.0935, cz + 0.0915, 0.0065, MFD_D, 0.010)
    XML.append(mode_knob(name))


def knob_xml(name, prop, deg, step, lo, hi, wrap=False, offset=0.0):
    """a knob turning with prop (deg per unit, from offset), which its clicks and the mouse wheel step by step
    (FlightGear's knob animation: the left half down, the right half up)"""
    (d, y, z), (ad, ay, az) = CENTRES[name]
    return ('  <animation>\n    <type>knob</type>\n    <object-name>%s</object-name>\n    <property>%s</property>\n'
            '    <factor>%g</factor>\n    <offset-deg>%g</offset-deg>\n'
            '    <center>\n      <x-m>%.4f</x-m>\n      <y-m>%.4f</y-m>\n      <z-m>%.4f</z-m>\n    </center>\n'
            '    <axis>\n      <x>%.4f</x>\n      <y>%g</y>\n      <z>%.4f</z>\n    </axis>\n'
            '    <action>\n      <binding>\n        <command>property-adjust</command>\n        <property>%s</property>\n'
            '        <factor>%g</factor>\n        <min>%g</min>\n        <max>%g</max>\n        <wrap>%s</wrap>\n      </binding>\n    </action>\n'
            '  </animation>\n' % (name, prop, deg, offset, d, y, z, ad, ay, az, prop, step, lo, hi, 'true' if wrap else 'false'))


def mode_knob(unit):
    """the OFF / NIGHT / DAY knob: turns with its unit's mode (0 off, 1 night, 2 day)"""
    return knob_xml(unit + '.MODE', '/instrumentation/mv22b/displays/%s/mode' % unit.lower(), 40, 1, 0, 2, offset=-40)


# ---------------------------------------------------------------- the centre: backup engine and fuel display, standby
CENTRE_W = 2 * (abs(MFD_Y[1]) - MFD_W / 2)           # between the inner bezels
SFD_W, ESFI_W = 0.168, 0.150
SFD_Y0 = -CENTRE_W / 2 + 0.010                       # the backup engine and fuel display's left edge
ESFI_Y1 = CENTRE_W / 2 - 0.010                       # the standby instrument's right edge


def small_face(kind, w, h):
    def draw(d, W, H, k):
        d.rectangle((0, 0, W, H), fill=BEZEL)
        c = lambda y, z: (W / 2 + y * k, H / 2 - z * k)
        f = font(0.0028 * k)
        if kind == 'sfd':
            for t, (y, z) in (('SEL', (0.055, 0.040)), ('1', (0.047, 0.028)), ('2', (0.064, 0.028)), ('DIS', (0.055, -0.010)),
                              ('OFF', (0.044, -0.040)), ('BRT', (0.067, -0.040))):
                d.text(c(y, z), t, font=f, fill=LETTER, anchor='mm')
        else:
            for t, (y, z) in (('BARO', (-0.052, -0.050)), ('BRT', (0.052, -0.050))):
                d.text(c(y, z), t, font=f, fill=LETTER, anchor='mm')
    return face_tile(('small', kind), w, h, FACE_PX, draw)


def recessed(o, cy, cz, w, h, sw, sh, scy, scz, dface, uvf):
    """a bezel's face around a screen's recess at (scy, scz)"""
    y0, y1, z0, z1 = cy - w / 2, cy + w / 2, cz - h / 2, cz + h / 2
    a0, a1, b0, b1 = scy - sw / 2, scy + sw / 2, scz - sh / 2, scz + sh / 2
    for (p0, p1, q0, q1) in ((y0, y1, b1, z1), (y0, y1, z0, b0), (y0, a0, b0, b1), (a1, y1, b0, b1)):
        A.flat(o, [(p0, q0), (p1, q0), (p1, q1), (p0, q1)], dface, uvf)
    A.walls(o, [(y0, z0), (y1, z0), (y1, z1), (y0, z1)], -0.01, dface, solid(BEZEL))
    A.walls(o, [(a0, b0), (a1, b0), (a1, b1), (a0, b1)], dface - 0.008, dface, solid(SCREEN_OFF), outward=False)


def centre_top():
    top = MFD_Z + MFD_H / 2
    h = 0.125
    # the backup engine and fuel display (left of centre)
    w, cy, cz = SFD_W, SFD_Y0 + SFD_W / 2, top - h / 2
    o = obj('SFD.bezel')
    sw, sh, scy = 0.106, 0.080, cy - 0.022
    recessed(o, cy, cz, w, h, sw, sh, scy, cz, 0.018, tex_rect(small_face('sfd', w, h), cy - w / 2, cy + w / 2, cz - h / 2, cz + h / 2))
    screen('SFD.screen', scy, cz, sw, sh, 0.0105)
    knob('SFD.SEL', cy + 0.055, cz + 0.022, 0.006, 0.018, 0.010)
    knob('SFD.MODE', cy + 0.055, cz - 0.026, 0.007, 0.018, 0.010)
    XML.append(mode_knob('SFD'))
    # the electronic standby flight instrument (right of centre)
    w, cy = ESFI_W, ESFI_Y1 - ESFI_W / 2
    o = obj('ESFI.bezel')
    sw, sh = 0.098, 0.090
    recessed(o, cy, cz, w, h, sw, sh, cy, cz + 0.008, 0.018, tex_rect(small_face('esfi', w, h), cy - w / 2, cy + w / 2, cz - h / 2, cz + h / 2))
    screen('ESFI.screen', cy, cz + 0.008, sw, sh, 0.0105)
    for i in range(3):
        key('ESFI.L%d' % (i + 1), 'ESFI', 'L%d' % (i + 1), 'R', cy - 0.064, cz + 0.032 - 0.022 * i, 0.012, 0.010, 0.018)
        key('ESFI.R%d' % (i + 1), 'ESFI', 'R%d' % (i + 1), 'L', cy + 0.064, cz + 0.032 - 0.022 * i, 0.012, 0.010, 0.018)
    knob('ESFI.BARO', cy - 0.052, cz - 0.050, 0.0065, 0.018, 0.010)
    knob('ESFI.BRT', cy + 0.052, cz - 0.050, 0.0065, 0.018, 0.010)
    XML.append(baro_knob())
    return top - h


def baro_knob():
    return (knob_xml('ESFI.BARO', '/instrumentation/altimeter/setting-inhg', 600, 0.01, 27.5, 31.5) +
            knob_xml('ESFI.BRT', '/instrumentation/mv22b/displays/esfi/brightness', 270, 0.1, 0.1, 1))


# ---------------------------------------------------------------- the control display unit
CDU_W, CDU_H = 0.300, 0.228
CDU_SW, CDU_SH = 0.213, 0.160
CDU_Y = SFD_Y0 + CDU_W / 2                           # left of centre: its left edge under the backup display's


def cdu_face(d, W, H, k):
    d.rectangle((0, 0, W, H), fill=BEZEL)
    c = lambda y, z: (W / 2 + y * k, H / 2 - z * k)
    lw = max(1, int(0.0006 * k))
    for i in range(7):
        z = 0.063 - 0.0195 * i + 0.010
        for side in (-1, 1):
            d.line([c(side * 0.118, z), c(side * (CDU_SW / 2 + 0.002), z - 0.003)], fill=LETTER, width=lw)
    f = font(0.0028 * k)
    for t, (y, z) in (('BRT', (-0.130, 0.106)), ('OFF', (0.112, 0.108)), ('NIGHT', (0.114, 0.102)), ('DAY', (0.112, 0.096)),
                      ('CAC', (-0.135, -0.078)), ('CAC', (0.135, -0.078))):
        d.text(c(y, z), t, font=f, fill=LETTER, anchor='mm')


def cdu(top):
    cz = top - 0.006 - CDU_H / 2
    o = obj('CDU.bezel')
    face = face_tile('cdu-face', CDU_W, CDU_H, FACE_PX, cdu_face)
    scz = cz + 0.010
    y = CDU_Y
    recessed(o, y, cz, CDU_W, CDU_H, CDU_SW + 0.008, CDU_SH + 0.008, y, scz, 0.020,
             tex_rect(face, y - CDU_W / 2, y + CDU_W / 2, cz - CDU_H / 2, cz + CDU_H / 2))
    screen('CDU.screen', y, scz, CDU_SW, CDU_SH, 0.0125)
    kw, kh = 0.015, 0.0105
    for i in range(7):
        z = scz + 0.063 - 0.0195 * i
        key('CDU.L%d' % (i + 1), 'CDU', 'L%d' % (i + 1), 'R', y - 0.130, z, kw, kh, 0.020)
        key('CDU.R%d' % (i + 1), 'CDU', 'R%d' % (i + 1), 'L', y + 0.130, z, kw, kh, 0.020)
    for i, t in enumerate(['CDU', 'C/A', 'STAT', 'EICAS', 'ACK', 'CDU']):
        key('CDU.B%d' % (i + 1), 'CDU', 'B%d' % (i + 1), t, y - 0.105 + 0.042 * i, cz - CDU_H / 2 + 0.016, 0.026, 0.011, 0.020)
    rocker('CDU.BRT', 'CDU', 'BRT', y - 0.130, cz + CDU_H / 2 - 0.022, 0.011, 0.016, 0.020, 'BRT')
    knob('CDU.MODE', y + 0.130, cz + CDU_H / 2 - 0.020, 0.0065, 0.020, 0.010)
    knob('CDU.CAC1', y - 0.135, cz - CDU_H / 2 + 0.016, 0.0065, 0.020, 0.010)
    knob('CDU.CAC2', y + 0.135, cz - CDU_H / 2 + 0.016, 0.0065, 0.020, 0.010)
    XML.append(mode_knob('CDU'))
    return cz - CDU_H / 2


DIAL = (9, 27, 18)          # the azimuth indicator's green face, unlit
MARK = (226, 222, 200)       # its scale
RING = (12, 12, 13)
CALL_H = 0.124
RWR_C = (-0.002, 0.020)      # the indicator's centre from the panel's
RWR_R = (0.029, 0.025)       # its ring's radius, its face's
RWR_PLATE = (0.066, 0.062)   # its square mounting plate
RWR_TEX, RWR_PX = 'rwr.png', 512                     # its ring and face: their own texture, the ring's width across


def rwr_face():
    R, r = RWR_R
    k = RWR_PX / (2 * R)
    im = Image.new('RGB', (RWR_PX, RWR_PX), RING)
    d = ImageDraw.Draw(im)
    c = lambda y, z: (RWR_PX / 2 + y * k, RWR_PX / 2 - z * k)
    disc = lambda a, **kw: d.ellipse(c(-a, a) + c(a, -a), **kw)
    lw = max(1, int(0.0005 * k))
    disc(r, fill=DIAL)
    for i in range(12):
        a = math.radians(30 * i)
        r0, r1 = (0.62, 0.94) if i % 3 == 0 else (0.80, 0.91)
        d.line([c(r * r0 * math.sin(a), r * r0 * math.cos(a)), c(r * r1 * math.sin(a), r * r1 * math.cos(a))], fill=MARK, width=lw)
    for i in range(16):
        d.arc(c(-0.43 * r, 0.43 * r) + c(0.43 * r, -0.43 * r), 22.5 * i + 5, 22.5 * i + 16, fill=MARK, width=lw)
    disc(0.055 * r, outline=MARK, width=lw)
    return im


def call_panel(top):
    """right of the CDU, under the standby instrument: a small panel with a radar warning receiver's round azimuth
    indicator (by its look, the AN/APR-39's: a green face scaled every 30 deg, a dashed inner ring; its knob at the
    upper right), under it the RADIO CALL placard and a pushbutton. Scaled to the panel: the photographs' is wider"""
    y0, y1 = CDU_Y + CDU_W / 2 + 0.002, ESFI_Y1
    w, h, cy = y1 - y0, CALL_H, (y0 + y1) / 2
    cz = top - 0.006 - h / 2
    ry, rz = cy + RWR_C[0], cz + RWR_C[1]
    R, r = RWR_R

    def draw(d, W, H, k):
        d.rectangle((0, 0, W, H), fill=BEZEL)
        c = lambda y, z: (W / 2 + y * k, H / 2 - z * k)
        lw = max(1, int(0.0005 * k))
        d.rectangle(c(-0.038, -0.020) + c(0.006, -0.040), fill=RING)
        d.text(c(-0.016, -0.026), 'RADIO CALL', font=font(0.0045 * k), fill=LETTER, anchor='mm')
        d.text(c(-0.016, -0.034), '- - - - - -', font=font(0.0040 * k), fill=LETTER, anchor='mm')
        for sy in (-0.035, 0.003):
            d.ellipse(c(sy - 0.0008, -0.0292) + c(sy + 0.0008, -0.0308), outline=LETTER, width=lw)
    face = face_tile('call-face', w, h, FACE_PX, draw)
    o = plate('CALL.panel', cy, cz, w, h, -0.01, 0.016, tex_rect(face, cy - w / 2, cy + w / 2, cz - h / 2, cz + h / 2), color=BEZEL)
    pw, ph = RWR_PLATE
    A.box(o, ry - pw / 2, ry + pw / 2, rz - ph / 2, rz + ph / 2, 0.016, 0.019, solid(BEZEL), solid(BEZEL))
    for sy, sz in ((-1, -1), (1, -1), (-1, 1)):
        A.cylinder(o, 0.0018, ry + sy * (pw / 2 - 0.004), rz + sz * (ph / 2 - 0.004), 0.019, 0.0205, solid(SCREW), solid(SCREW), n=10)
    A.cylinder(o, 0.0022, ry + 0.027, rz - 0.019, 0.019, 0.021, solid(SCREW), solid(SCREW), n=12)
    A.box(o, cy + 0.012, cy + 0.028, cz - 0.034, cz - 0.024, 0.016, 0.021, solid(KEY), solid(KEY))
    # the ring standing off the plate, the face down in it
    f = obj('RWR.face', RWR_TEX)
    uv = A.planar(ry - R, ry + R, rz - R, rz + R)
    A.bezel(f, A.circle(R, ry, rz, 48), A.circle(r, ry, rz, 48), 0.025, uv, ry, rz)
    A.walls(f, A.circle(R, ry, rz, 48), 0.019, 0.025, A.const(0.0, 0.0))
    A.walls(f, A.circle(r, ry, rz, 48), 0.0215, 0.025, A.const(0.0, 0.0), outward=False)
    A.flat(f, A.circle(r, ry, rz, 48), 0.0215, uv)
    knob('RWR.BRIL', ry + 0.0225, rz + 0.0225, 0.0048, 0.019, 0.010)
    A.box(o, cy + 0.012, cy + 0.028, cz - 0.034, cz - 0.024, 0.016, 0.021, solid(KEY), solid(KEY))


# ---------------------------------------------------------------- the thrust control levers
# One for each pilot's left hand, linked: the right-hand seat's on the console's right side, its quadrant's top flush
# with the keypads; the left-hand seat's as far outboard of that seat, its quadrant on the side wall at the same
# height and slope (the interior model has no side console there). The grip, the same in both, is carved from three
# photographs of it by tclgrip.py (data/tcl_grip.json: its mesh and where its switches are, in its frame: d up from
# its base's bottom, y aft, z right): the left hand on the palm rest, its fingers over the front face and its panel of
# two pushbuttons and two ribbed rockers, its thumb on the right side by the head, on the white button, the four-way
# switch and the NACELLE thumbwheel [Vertical magazine, "Flying the V-22": "a spring-loaded, knurled rotary knob ... in
# contact with the pilot's left thumb"]. The lever travels 4 in fore and aft [same], here along the console's slope.
GRIP = (33, 34, 36)
WHEEL = (138, 144, 150)
SWITCH = (184, 184, 178)
BUTTON = (224, 224, 212)
TCL_TRAVEL = 0.1016                    # m
TCL_LEAN = 10.0                        # deg: the grip's up leaning aft of the vertical, its palm rest about level
TCLG = json.load(open(os.path.join(HERE, 'data', 'tcl_grip.json')))
STEM_Y, STEM_Z = 0.020, 0.028          # the lever's stem, under the middle of the grip's base
GRIP_LEFT = -0.055                     # the body's left side
GRIP_BASE_RIGHT = 0.083                # the base's right side
WHEEL_R = 0.012
NACELLE_RATE = math.radians(8.0)                       # rad/s: the flight model's conversion rate
TCL_SIDE_WALL = -1.005                 # m: the left-hand seat's quadrant's outer side, in the side wall
SEAT_Y = 0.540                         # the seats' centres (Interior/Seat/all-seats.xml)
KP_CLEAR = 0.010                       # the grip's left end right of the keypads


def detail(name):
    d = TCLG['details'][name]
    return tuple(d['p']), tuple(d['n'])


def _wheel_centre():
    """the thumbwheel's centre: where the photograph has it, its rim 6 mm out of the surface, its axis up"""
    (d, y, z), (nd, ny, nz) = detail('thumbwheel')
    h = math.hypot(ny, nz)
    return (d, y - ny / h * (WHEEL_R - 0.006), z - nz / h * (WHEEL_R - 0.006))


WHEEL_C = _wheel_centre()


class Frame:
    """an object built in a frame of its own: its (d, y, z) at origin + d ed + y ey + z ez in its parent's"""
    def __init__(s, o, origin, ed, ey, ez):
        s.o, s.origin, s.ax = o, origin, (ed, ey, ez)

    def vec(s, v):
        return tuple(sum(v[j] * s.ax[j][i] for j in range(3)) for i in range(3))

    def pt(s, p):
        return tuple(a + b for a, b in zip(s.origin, s.vec(p)))

    def face(s, pts, uvs, normal):
        s.o.face([s.pt(p) for p in pts], uvs, s.vec(normal))


def unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def on_surface(g, name, along=None):
    """a frame at a detail of the grip: d out of the surface, y along it (along, or level), z up it"""
    p, n = detail(name)
    n = unit(n)
    t = along if along else cross((1.0, 0.0, 0.0), n)
    t = unit(tuple(a - sum(x * y for x, y in zip(t, n)) * b for a, b in zip(t, n)))
    return Frame(g, p, n, t, cross(n, t))


def gear(cy, cz, r, teeth, depth):
    """a knurled wheel's outline (y, z)"""
    return [(cy + (r - depth * (i % 2)) * math.cos(math.pi * i / teeth), cz + (r - depth * (i % 2)) * math.sin(math.pi * i / teeth))
            for i in range(2 * teeth)]


def chaikin(poly, n=2):
    """a closed polygon's corners rounded by n rounds of corner cutting"""
    for _ in range(n):
        out = []
        for p, q in zip(poly, poly[1:] + poly[:1]):
            out += [tuple(0.75 * a + 0.25 * b for a, b in zip(p, q)), tuple(0.25 * a + 0.75 * b for a, b in zip(p, q))]
        poly = out
    return poly


def arrows_face(d, W, H, k):
    d.rectangle((0, 0, W, H), fill=SWITCH)
    c = W / 2
    for a in range(4):
        ca, sa = math.cos(math.pi / 2 * a), math.sin(math.pi / 2 * a)
        tip, base, half = 0.40 * W, 0.22 * W, 0.07 * W
        pt = lambda r, t: (c + r * ca - t * sa, c - (r * sa + t * ca))
        d.polygon([pt(tip, 0), pt(base, half), pt(base, -half)], fill=(60, 60, 58))


def grip_solids(g, w):
    """the grip in its own frame into g, its thumbwheel into w"""
    black, dark = solid(GRIP), solid((50, 50, 52))
    A.box(g, -0.008, 0.008, STEM_Z - 0.012, STEM_Z + 0.012, -0.050, 0.0, dark, dark)       # the stem
    V = TCLG['verts']
    for q, ax in zip(TCLG['quads'], TCLG['axes']):
        g.face([tuple(V[i]) for i in q], [black(0, 0)] * 4, tuple(ax))
    # by the thumb: the white button, a lamp, the four-way switch, the plate's screws, the grey four-way switch,
    # the rocker
    f = on_surface(g, 'white button')
    A.cylinder(f, 0.0085, 0, 0, -0.002, 0.002, black, black, n=24)
    A.cylinder(f, 0.0062, 0, 0, -0.002, 0.0045, solid(BUTTON), solid(BUTTON), n=24)
    A.cylinder(on_surface(g, 'lamp'), 0.003, 0, 0, -0.001, 0.0012, solid((10, 10, 10)), solid((10, 10, 10)), n=12)
    f = on_surface(g, 'hat')
    A.cylinder(f, 0.0125, 0, 0, -0.003, 0.003, black, black, n=28)
    A.cylinder(f, 0.0085, 0, 0, 0.003, 0.010, dark, dark, n=24)
    A.box(f, -0.007, 0.007, -0.0018, 0.0018, 0.010, 0.0115, dark, dark)
    A.box(f, -0.0018, 0.0018, -0.007, 0.007, 0.010, 0.0115, dark, dark)
    for s in ('screw left', 'screw right'):
        A.cylinder(on_surface(g, s), 0.0022, 0, 0, -0.001, 0.0015, dark, dark, n=10)
    arrows = face_tile('tcl-arrows', 0.022, 0.022, 8000, lambda d, W, H, k: arrows_face(d, W, H, k))
    A.cylinder(on_surface(g, 'grey switch'), 0.0105, 0, 0, -0.003, 0.006, tex_rect(arrows, -0.0105, 0.0105, -0.0105, 0.0105),
               solid(SWITCH), n=32)
    A.box(on_surface(g, 'rocker'), -0.004, 0.004, -0.010, 0.010, -0.002, 0.004, dark, dark)
    # NACELLE over the thumbwheel, written across as the photograph has it
    view = TCLG['details']['nacelle label']['view']
    across = cross(tuple(-c for c in view), (1.0, 0.0, 0.0))
    label = face_tile('nacelle-label', 0.032, 0.007, 8000,
                      lambda d, W, H, k: (d.rectangle((0, 0, W, H), fill=GRIP), fitted(d, 'NACELLE', W, H, H * 0.8)))
    A.flat(on_surface(g, 'nacelle label', across), [(-0.016, -0.0035), (0.016, -0.0035), (0.016, 0.0035), (-0.016, 0.0035)],
           0.0006, tex_rect(label, -0.016, 0.016, -0.0035, 0.0035))
    # the friction knob on the base's lobe
    (kd, ky, kz), _ = detail('friction knob')
    A.prism(Frame(g, (kd - 0.002, 0.0, 0.0), (1, 0, 0), (0, 1, 0), (0, 0, 1)), gear(ky, kz, 0.0085, 8, 0.0015), 0.0, 0.014,
            black, black)
    # the front face: the recessed panel, two pushbuttons with their lenses and two ribbed rockers standing out of it
    yf = detail('panel')[0][1]
    fr = lambda z, d: Frame(g, (d, yf - 0.0005, z), (0, -1, 0), (0, 0, 1), (1, 0, 0))   # d out of the face
    A.prism(fr(0.0, 0.0), chaikin([(-0.043, 0.034), (0.042, 0.034), (0.042, 0.075), (-0.043, 0.075)], 1), -0.003, 0.0004, dark, dark)
    for s in ('button left', 'button right'):
        (d, y, z), _ = detail(s)
        A.cylinder(fr(z, d), 0.0075, 0, 0, 0.0, 0.006, dark, dark, n=24)
        A.cylinder(fr(z, d), 0.0050, 0, 0, 0.006, 0.0068, solid((200, 205, 200)), solid((200, 205, 200)), n=20)
    ribs = face_tile('tcl-ribs', 0.014, 0.018, 8000, lambda d, W, H, k: [d.rectangle((0, 0, W, H), fill=SWITCH)] +
                     [d.rectangle((0, i * H / 7, W, i * H / 7 + H / 14), fill=(120, 120, 116)) for i in range(7)])
    for s in ('rocker left', 'rocker right'):
        (d, y, z), _ = detail(s)
        A.box(fr(z, d), -0.007, 0.007, -0.009, 0.009, 0.0, 0.009, tex_rect(ribs, -0.007, 0.007, -0.009, 0.009), solid(SWITCH))
    # the thumbwheel, its axis up the grip
    A.prism(w, gear(WHEEL_C[1], WHEEL_C[2], WHEEL_R, 30, 0.0012), WHEEL_C[0] - 0.005, WHEEL_C[0] + 0.005, solid(WHEEL), solid(WHEEL))


def tcl(seat, kp_top):
    """a thrust control lever, its quadrant beside the keypads (kp_top: their top edge) and the lever's grip on it:
    seat 'R' on the console's right side, 'L' on the left side wall"""
    n = 'TCL.' + seat
    ys = CDU_Y + KP_W + 0.002 + KP_CLEAR - (GRIP_LEFT - STEM_Z)                     # the stem: the grip's left end clear
    y0 = CDU_Y + CDU_W / 2 + 0.006                                                  # the console's side
    if seat == 'L':
        ys, y0 = -SEAT_Y - (SEAT_Y - ys), TCL_SIDE_WALL
    y1 = ys + (GRIP_BASE_RIGHT - STEM_Z) + 0.008                                    # past the base's right side
    zc = kp_top - KP_L / 2
    plate(n + '.quadrant', (y0 + y1) / 2, zc, y1 - y0, KP_L, KP_D - 0.12, KP_D, solid(BEZEL), color=BEZEL)
    slot = obj(n + '.slot')
    A.flat(slot, [(ys - 0.007, zc - 0.061), (ys + 0.007, zc - 0.061), (ys + 0.007, zc + 0.061), (ys - 0.007, zc + 0.061)],
           KP_D + 0.0004, solid(SCREEN_OFF))
    # the grip's frame: up tilted forward from the slope's normal, along it to the right, the thumb's side aft
    a = math.radians(90.0 - PANEL_PITCH - CONSOLE_TILT - TCL_LEAN)                 # forward from the slope's normal
    up, aft, right = (math.cos(a), 0.0, math.sin(a)), (math.sin(a), 0.0, -math.cos(a)), (0.0, 1.0, 0.0)
    stem = (KP_D + 0.022, ys, zc)                                                   # its base clear of the quadrant
    origin = tuple(stem[i] - STEM_Y * aft[i] - STEM_Z * right[i] for i in range(3))
    grip = lambda name: Frame(obj(name), origin, up, aft, right)
    g, w = grip(n + '.grip'), grip(n + '.wheel')
    grip_solids(g, w)
    # spring-centred, turned while the nacelles tilt; its clicks and the mouse wheel step them (v22.set_tilt, the m/M
    # keys': its "tilt" is 0 up, 90 forward)
    centre, axis = FRAME.pt(w.pt(WHEEL_C)), FRAME.vec(w.vec((1, 0, 0)))
    axis = tuple(c / math.sqrt(sum(x * x for x in axis)) for c in axis)
    XML.append('  <animation>\n    <type>knob</type>\n    <object-name>%s.wheel</object-name>\n'
               '    <property>/fdm/jsbsim/ctl/nacelle-rate-rad_sec</property>\n    <factor>%.1f</factor>\n'
               '    <min-deg>-25</min-deg>\n    <max-deg>25</max-deg>\n'
               '    <center>\n      <x-m>%.4f</x-m>\n      <y-m>%.4f</y-m>\n      <z-m>%.4f</z-m>\n    </center>\n'
               '    <axis>\n      <x>%.4f</x>\n      <y>%.4f</y>\n      <z>%.4f</z>\n    </axis>\n'
               '    <increase>\n      <binding>\n        <command>nasal</command>\n        <script>v22.set_tilt(-10)</script>\n'
               '      </binding>\n    </increase>\n'
               '    <decrease>\n      <binding>\n        <command>nasal</command>\n        <script>v22.set_tilt(10)</script>\n'
               '      </binding>\n    </decrease>\n  </animation>\n'
               % ((n, 25.0 / NACELLE_RATE) + centre + axis))
    # the lever: fore and aft along the slope with the thrust control (FlightGear's throttle, 0 aft .. 1 forward)
    ax = FRAME.vec((0, 0, 1))
    XML.append('  <animation>\n    <type>translate</type>\n    <object-name>%s.grip</object-name>\n'
               '    <object-name>%s.wheel</object-name>\n    <property>/controls/engines/engine[0]/throttle</property>\n'
               '    <factor>%.4f</factor>\n    <offset-m>%.4f</offset-m>\n'
               '    <axis>\n      <x>%.4f</x>\n      <y>%.4f</y>\n      <z>%.4f</z>\n    </axis>\n  </animation>\n'
               % ((n, n, TCL_TRAVEL, -TCL_TRAVEL / 2) + ax))


# ---------------------------------------------------------------- the keypads, below the CDU
KP_ROWS = [  # (legend, columns it spans of 7, boxed); None: no key
    [None, '-', '+', 'EMCON', 'INAV', 'ENAV', None],
    ['1', '2', '3', 'COMM', 'FPLN', 'WYPT', 'ECS'],
    ['4', '5', '6', 'IFF', 'MSN', 'W/W', 'LTS'],
    ['7', '8', '9', 'TIMER', 'SEQ↑', 'PG↑', ''],
    ['/', '0', '.', 'EMER', 'SEQ↓', 'PG↓', 'ENTER'],
    ['←', '→', 'DEL', 'CLR', None, 'IDENT', None],
    ['A', 'B', 'C', 'D', 'E', 'F', 'G'],
    ['H', 'I', 'J', 'K', 'L', 'M', 'N'],
    ['O', 'P', 'Q', 'R', 'S', 'T', 'U'],
    ['V', 'W', 'X', 'Y', 'Z', '', '']]
BOXED = {'5', 'E', 'N', 'S', 'W'}                     # the touch-typing marks of the real keypad
KP_W, KP_L = 0.146, 0.212                             # two side by side as wide as the CDU above them
KP_GAP = 0.010                                        # below the CDU
CONSOLE_TILT = 15.0                                   # deg: the CDU and keypads' plane back from the panel's face, down
                                                      # to the level aft section, about as high as the seats' cushions
AFT_L = 0.205                                         # m: the console's level aft section past them, to its square edge
KP_D = 0.014                                          # the base's face: in the CDU's plane, its keys' tops at its face
KP_K = KP_W / 0.166                                   # the key layout's scale (drawn for a 0.166 m keypad)


def kp_face(d, W, H, k):
    d.rectangle((0, 0, W, H), fill=BEZEL)
    for (y, z) in ((-0.064, 0.096), (0.064, 0.096), (-0.064, -0.096), (0.064, -0.096), (-0.064, 0.010), (0.064, 0.010)):
        cx, cy = W / 2 + y * k, H / 2 - z * k; r = 0.0022 * k
        d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=(12, 12, 12))


def keypad(n, cy, z_top):
    """a keypad below the CDU, straight on in its plane (z_top: its top edge)"""
    name = 'KP%d' % n
    face = face_tile('kp-face', KP_W, KP_L, FACE_PX, kp_face)
    plate(name + '.base', cy, z_top - KP_L / 2, KP_W, KP_L, KP_D - 0.012, KP_D,
          tex_rect(face, cy - KP_W / 2, cy + KP_W / 2, z_top - KP_L, z_top), color=BEZEL)
    pitch_z = 0.0193
    for r, row in enumerate(KP_ROWS):
        z = z_top - 0.014 - pitch_z * r
        for c, leg in enumerate(row):
            if leg is None:
                continue
            if r < 6 and c >= 3:          # the wide function keys
                y, w = 0.0738 + 0.0255 * (c - 3), 0.0215
            elif r < 6:
                y, w = 0.0130 + 0.0182 * c, 0.0138
            else:
                y, w = 0.0150 + 0.0225 * c, 0.0158
            y = cy - KP_W / 2 + y * KP_K
            kid = leg.replace('↑', 'UP').replace('↓', 'DN').replace('←', 'LEFT').replace('→', 'RIGHT') or 'BLANK%d%d' % (r, c)
            key('%s.%s' % (name, kid.replace('/', 'SLASH').replace('.', 'DOT').replace('-', 'MINUS').replace('+', 'PLUS')),
                name, kid, leg, y, z, w * KP_K, 0.0118, KP_D, 0.006, boxed=leg in BOXED)


# ---------------------------------------------------------------- the console's aft section, past the keypads
# After the photographs, level from the keypads' foot to the console's square aft edge: on the left the landing gear panel
# (the wheel-knobbed GEAR handle, MUTE, DOWN LOCK ORIDE, the striped EMERG DOWN pull), in the middle a grip lying along
# the section, its head forward (by its markings, FOV, SIZE, POL, MAN and a thumb puck, the FLIR turret's hand control: it
# doesn't move), on the right the FLAP lever (the flight model schedules the flaps: it stays at A, auto) and its
# scale; past the edge the PARK BRAKE handle on its rod (body). Across, the photographs' layout fitted to the console.
CHROME = (118, 120, 123)
RED = (196, 32, 30)
HAZARD = (222, 176, 30)
GLASS = (150, 156, 162)
GEAR_TRAVEL = 0.025                   # m: the gear handle forward with the gear up
BRAKE_TRAVEL = 0.040                  # m: the park brake handle down, released


def closed(o, poly, d0, d1, uv_end, uv_side):
    """a prism capped at both ends: free-standing, seen from all round"""
    A.prism(o, poly, d0, d1, uv_end, uv_side)
    A.flat(o, poly, d0, uv_end, normal=-1)


def screws(o, y0, y1, z0, z1, d, inset=0.005):
    for y, z in ((y0 + inset, z0 + inset), (y1 - inset, z0 + inset), (y1 - inset, z1 - inset), (y0 + inset, z1 - inset)):
        A.cylinder(o, 0.0022, y, z, d, d + 0.0015, solid(SCREW), solid(SCREW), n=10)


def hazard_box(d, x0, y0, x1, y1, b):
    """yellow and black stripes round a black box (px), b wide"""
    im = Image.new('RGB', (int(x1 - x0), int(y1 - y0)), HAZARD)
    di = ImageDraw.Draw(im)
    for x in range(-im.height, im.width, int(2 * b)):
        di.polygon([(x, im.height), (x + b, im.height), (x + b + im.height, 0), (x + im.height, 0)], fill=(10, 10, 10))
    di.rectangle((b, b, im.width - b, im.height - b), fill=(10, 10, 10))
    d._image.paste(im, (int(x0), int(y0)))


def aft_section(z0):
    """the level section aft of the keypads' foot (z0, in its frame: d up, z forward): s aft, y from the console's middle"""
    Z = lambda s: z0 - 0.006 - s
    Y = lambda y: CDU_Y + y
    tex = lambda face, y0, y1, s0, s1: tex_rect(face, Y(y0), Y(y1), Z(s1), Z(s0))

    def panel(name, y0, y1, s0, s1, draw):
        face = face_tile(name, y1 - y0, s1 - s0, FACE_PX,
                         lambda d, W, H, k: (d.rectangle((0, 0, W, H), fill=BEZEL),
                                             draw(d, k, lambda y, s: ((y - y0) * k, (s - s0) * k))))
        o = plate(name + '.panel', Y((y0 + y1) / 2), Z((s0 + s1) / 2), y1 - y0, s1 - s0, -0.002, 0.004,
                  tex(face, y0, y1, s0, s1), color=BEZEL)
        screws(o, Y(y0), Y(y1), Z(s1), Z(s0), 0.004)
        return o

    # the landing gear panel
    def gear_face(d, k, c):
        f = font(0.0030 * k)
        d.rectangle(c(-0.062, 0.020) + c(-0.054, 0.066), fill=SCREEN_OFF)
        d.text(c(-0.058, 0.023), 'GEAR', font=f, fill=LETTER, anchor='mm')
        d.text(c(-0.107, 0.062), 'MUTE', font=f, fill=LETTER, anchor='mm')
        for i, t in enumerate(('DOWN', 'LOCK', 'ORIDE')):
            d.text(c(-0.108, 0.106 + 0.0050 * i), t, font=f, fill=LETTER, anchor='mm')
        d.text(c(-0.108, 0.127), ARROWS['D'], font=font(0.0050 * k), fill=LETTER, anchor='mm')
        hazard_box(d, *c(-0.066, 0.098), *c(-0.031, 0.154), 0.0022 * k)
        for i, t in enumerate(('EMERG', 'DOWN')):
            d.text(c(-0.0485, 0.106 + 0.0050 * i), t, font=f, fill=LETTER, anchor='mm')
        d.ellipse(c(-0.059, 0.119) + c(-0.038, 0.140), fill=(4, 4, 4))
    o = panel('GEAR', -0.124, -0.022, 0.017, 0.159, gear_face)
    A.cylinder(o, 0.0050, Y(-0.090), Z(0.062), 0.004, 0.008, solid(KEY), solid(KEY), n=16)
    A.cylinder(o, 0.0045, Y(-0.089), Z(0.131), 0.004, 0.009, solid(RED), solid(RED), n=16)
    h = obj('GEAR.handle')
    A.beam(h, (0.002, Y(-0.058), Z(0.046)), (0.032, Y(-0.058), Z(0.079)), 0.008, solid(KEY))
    # the wheel, its edges rounded: across, its tread's quarter circles round to its flat sides
    rim = [(math.copysign(0.005 + 0.007 * math.sin(t), s), 0.014 + 0.007 * math.cos(t))
           for s, ts in ((-1, (90, 60, 30, 0)), (1, (0, 30, 60, 90))) for t in map(math.radians, ts)]
    A.loft(Frame(h, (0.032, Y(-0.058), Z(0.079)), (0, 1, 0), (0, 0, 1), (1, 0, 0)),
           [[(x, r * math.cos(a), r * math.sin(a)) for a in (2 * math.pi * k / 24 for k in range(24))] for x, r in rim],
           solid(CHROME))
    up = FRAME.vec((0, 0, 1))
    XML.append('  <animation>\n    <type>translate</type>\n    <object-name>GEAR.handle</object-name>\n'
               '    <property>/controls/gear/gear-down</property>\n    <factor>%.4f</factor>\n    <offset-m>%.4f</offset-m>\n'
               '    <axis>\n      <x>%.4f</x>\n      <y>%.4f</y>\n      <z>%.4f</z>\n    </axis>\n  </animation>\n'
               % ((-GEAR_TRAVEL, GEAR_TRAVEL) + up) + toggle_xml('GEAR.handle', '/controls/gear/gear-down'))

    # the FLIR's hand control
    panel('HCU', -0.018, 0.056, 0.000, 0.173, lambda d, k, c: None)
    g = obj('HCU.grip')
    G = lambda s: Z(s + AFT_L - 0.006 - 0.170)          # the grip aft, its stick's end flush with the aft face
    gy = Y(0.019)
    p, top = solid(GRIP), 0.056
    A.box(g, gy - 0.016, gy + 0.016, G(0.040), G(0.010), 0.004, 0.030, p, p)
    closed(g, A.rrect(0.072, 0.046, 0.012, gy, G(0.023)), 0.028, top, p, p)
    A.cylinder(g, 0.012, gy, G(0.012), top, top + 0.005, solid(KEY), solid(KEY), n=24)
    A.cylinder(g, 0.0085, gy, G(0.012), top + 0.005, top + 0.013, solid(GLASS), solid(GLASS), n=20)
    for sy in (-1, 1):
        A.box(g, gy + sy * 0.026 - 0.0065, gy + sy * 0.026 + 0.0065, G(0.020), G(0.004), top, top + 0.012, solid(SWITCH), solid(SWITCH))
        A.cylinder(g, 0.0055, gy + sy * 0.017, G(0.031), top, top + 0.004, solid(RED), solid(RED), n=16)
    A.cylinder(g, 0.004, gy, G(0.032), top, top + 0.003, solid(KEY), solid(KEY), n=12)
    for t, y, s in (('FOV', -0.024, 0.023), ('SIZE', 0.026, 0.023), ('POL', -0.017, 0.040), ('MAN', 0.0, 0.040)):
        uv = label_uv(t, 0.010, 0.0035)
        a0, a1, b0, b1 = gy + y - 0.005, gy + y + 0.005, G(s) - 0.00175, G(s) + 0.00175
        A.flat(g, [(a0, b0), (a1, b0), (a1, b1), (a0, b1)], top + 0.0002, tex_rect(uv, a0, a1, b0, b1))
    ring = lambda s, r: [(0.038 + r * math.cos(a), gy + r * math.sin(a), G(s)) for a in
                         (2 * math.pi * k / 20 for k in range(20))]
    A.loft(g, [ring(s, r) for s, r in ((0.040, 0.015), (0.050, 0.018), (0.140, 0.0185), (0.160, 0.0175), (0.167, 0.0135),
                                       (0.170, 0.007))], p)

    # the flap lever and its scale
    def flap_face(d, k, c):
        f = font(0.0030 * k)
        d.text(c(0.096, 0.024), 'FLAP', font=f, fill=LETTER, anchor='mm')
        d.rectangle(c(0.075, 0.020) + c(0.083, 0.115), fill=SCREEN_OFF)
        for i, t in enumerate(('A', '0', '10', '20', '40', 'F')):
            d.text(c(0.067, 0.040 + 0.0135 * i), t, font=f, fill=LETTER, anchor='mm')
        d.text(c(0.090, 0.042), '-', font=f, fill=LETTER, anchor='mm')
        d.text(c(0.101, 0.042), 'AUTO', font=f, fill=LETTER, anchor='mm')
        for i, t in enumerate(('0', '10', '20')):
            s = 0.056 + 0.0135 * i
            d.line([c(0.089, s), c(0.101, s)], fill=LETTER, width=max(1, int(0.0006 * k)))
            d.text(c(0.106, s), t, font=f, fill=LETTER, anchor='mm')
        d.rectangle(c(0.112, 0.053) + c(0.1145, 0.084), fill=RED)
        d.rectangle(c(0.061, 0.128) + c(0.126, 0.174), fill=SCREEN_OFF)
    panel('FLAP', 0.058, 0.130, 0.014, 0.180, flap_face)
    v = obj('FLAP.lever')
    A.box(v, Y(0.073), Y(0.085), Z(0.044), Z(0.036), 0.002, 0.036, solid(KEY), solid(KEY))
    closed(v, [(Y(0.071), Z(0.048)), (Y(0.087), Z(0.048)), (Y(0.087), Z(0.032)), (Y(0.071), Z(0.032))], 0.034, 0.046,
           solid(KEY), solid(KEY))


def toggle_xml(name, prop):
    """a click flips prop"""
    return ('  <animation>\n    <type>pick</type>\n    <object-name>%s</object-name>\n    <action>\n      <button>0</button>\n'
            '      <repeatable>false</repeatable>\n      <binding>\n        <command>property-toggle</command>\n'
            '        <property>%s</property>\n      </binding>\n    </action>\n  </animation>\n' % (name, prop))


# ---------------------------------------------------------------- the glareshield
# After the cockpit photographs: the radio panels and the flight control panel are the glareshield, a shelf across
# the panel's top standing out over the displays toward the crew, its sides hardly taller than the radio panels in
# them, its middle taller round the flight control panel; at the panel's ends sheet-metal cheeks run back toward the
# crew flush with the radio panels' ends, their crew-side edges angled out from the radio panels' faces, straight for a
# short stretch, and angled back to the displays at the foot. Each
# radio panel runs from the flight control panel to just past its outer display: from its outer end two square keys,
# the CHAN knob, the display, the BRT knob.
SHELF_D = 0.060                      # the shelf's face, out from the panel's (the displays' bezels at 0.022)
SHELF_M = 0.0035                     # its margin round the panels in it
AFCS_W, AFCS_H = 0.410, 0.090
AFCS_Z0 = MFD_Z + MFD_H / 2 + 0.005  # the flight control panel's foot, over the displays
RADIO_TOP = 0.315                    # the radio housings' top edge, the same along each
RADIO_IN, RADIO_OUT = AFCS_W / 2 + 2 * SHELF_M, 0.6635   # their inner and outer ends, |y|
# Each housing, measured in from its outer end (m): two square keys, then a bevel where its foot slants down from
# under the keys to under the radio unit, the housing there the taller; the radio unit a plate of its own with the
# CHAN knob, the SEL key, the display and the BRT knob.
HOUSING_H = (0.066, 0.049)           # its height under the radio unit and under the keys
BEVEL = (0.168, 0.201)
RADIO_UNIT = (0.201, 0.438, 0.053)   # from, to, height
RADIO_KEYS = (0.075, 0.142)
RADIO_CHAN, RADIO_SEL, RADIO_BRT = 0.226, 0.254, 0.423
RADIO_WIN = (0.273, 0.402, 0.023)    # from, to, height
CHEEK_NEAR = 0.125                   # m: the cheeks at the panel's ends, how near the crew their edge comes
CHEEK_FLAT = (0.085, 0.195)          # m below their top: where that edge runs straight


def afcs_face(d, W, H, k):
    d.rectangle((0, 0, W, H), fill=BEZEL)
    c = lambda y, z: (W / 2 + y * k, H / 2 - z * k)
    f = font(0.0030 * k); fs = font(0.0024 * k)
    for t, (y, z) in AFCS_LABELS:
        d.text(c(y, z), t, font=f, fill=LETTER, anchor='mm')
    for t, (y, z) in (('E\nN\nA\nV', (-0.148, 0.020)), ('I\nN\nA\nV', (-0.148, -0.021))):
        d.multiline_text(c(y, z), t, font=fs, fill=LETTER, anchor='mm', spacing=0)
    # the readouts' window strip
    d.rectangle(c(-0.075, 0.032) + c(0.150, 0.019), fill=(10, 10, 10))


# (y, z) in the panel's own frame (m from its centre); after the photograph's proportions
AFCS_KEYS = [('CPLD-L', 'CPLD', -0.181, 0.000, 0.0155, 0.0145), ('ENAV', '', -0.128, 0.020, 0.016, 0.0145),
             ('VORILS', '', -0.105, 0.020, 0.016, 0.0145), ('TACAN', '', -0.082, 0.020, 0.016, 0.0145),
             ('SRCHPAT', '', -0.128, -0.016, 0.016, 0.0145), ('APPR', '', -0.105, -0.016, 0.016, 0.0145),
             ('WYPT', '', -0.082, -0.016, 0.016, 0.0145),
             ('HDGHOLD', '', -0.031, -0.030, 0.016, 0.0095), ('SPDHOLD', '', 0.004, -0.030, 0.016, 0.0095),
             ('ALTHOLD', '', 0.047, -0.030, 0.016, 0.0095), ('HVRHOLD', '', 0.087, -0.030, 0.016, 0.0095),
             ('AUTONAC', '', 0.127, 0.020, 0.016, 0.0145), ('SPARE', '', 0.152, 0.020, 0.016, 0.0145),
             ('HVRCPLD', '', 0.127, -0.016, 0.016, 0.0145), ('REMHVR', '', 0.152, -0.016, 0.016, 0.0145),
             ('CPLD-R', 'CPLD', 0.180, 0.000, 0.0155, 0.0145)]
AFCS_KNOBS = [('CRS', -0.062, 0.002), ('HDG', -0.026, 0.002), ('SPD', 0.008, 0.002), ('ALT', 0.048, 0.002),
              ('HVRALT', 0.088, 0.002), ('BRT', -0.062, -0.032)]
AFCS_LABELS = [('CPLD', (-0.181, 0.017)), ('VOR/ILS', (-0.105, 0.035)), ('TACAN', (-0.082, 0.035)), ('SRCH\nPAT', (-0.128, -0.000)),
               ('APPR', (-0.105, 0.000)), ('WYPT', (-0.082, 0.000)), ('CRS', (-0.062, 0.039)), ('HDG', (-0.026, 0.039)),
               ('SPEED', (0.008, 0.039)), ('ALT', (0.048, 0.039)), ('HVR ALT', (0.088, 0.039)), ('HOLD', (-0.031, -0.018)),
               ('HOLD', (0.004, -0.018)), ('HOLD', (0.047, -0.018)), ('HOLD', (0.087, -0.018)), ('AUTO\nNAC', (0.127, 0.036)),
               ('HVR\nCPLD', (0.127, 0.000)), ('REM\nHVR', (0.152, 0.000)), ('CPLD', (0.180, 0.017)), ('OFF', (-0.074, -0.040)),
               ('BRT', (-0.049, -0.040))]


def label_uv(text, w, h):
    """a small printed label's uv rectangle"""
    return face_tile(('label', text, w, h), w, h, FACE_PX,
                     lambda d, W, H, k: (d.rectangle((0, 0, W, H), fill=BEZEL), fitted(d, text, W, H, H * 0.75)))


def glareshield():
    """the shelf from the windscreen to its face, the flight control panel and the radio panels in it, the cheeks"""
    o = obj('Glareshield')
    p = solid(PAINT)
    A.box(o, -AFCS_W / 2 - SHELF_M, AFCS_W / 2 + SHELF_M, AFCS_Z0 - SHELF_M, AFCS_Z0 + AFCS_H + SHELF_M, -0.42, SHELF_D, p, p)
    hr, hk = HOUSING_H
    for side in (-1, 1):
        at = lambda s_: side * (RADIO_OUT - s_)
        inner = side * RADIO_IN
        # the radio housing: its full length to the keys' foot, the radio unit's lower part, the bevel between
        A.box(o, *sorted((at(0.0), inner)), RADIO_TOP - hk, RADIO_TOP, -0.42, SHELF_D, p, p)
        A.box(o, *sorted((at(BEVEL[1]), inner)), RADIO_TOP - hr, RADIO_TOP - hk, -0.42, SHELF_D, p, p)
        A.prism(o, [(at(BEVEL[0]), RADIO_TOP - hk), (at(BEVEL[1]), RADIO_TOP - hk), (at(BEVEL[1]), RADIO_TOP - hr)],
                -0.42, SHELF_D, p, p)
        # the cheek at the end: a sheet of metal straight back toward the crew, flush with the radio panel's end; its
        # crew-side edge starts at the radio panel's face at the top, angles toward the crew, runs straight for a
        # short stretch a little above halfway down, and angles back to the displays' bezels at the foot
        z0, z1 = MFD_Z - MFD_H / 2 - 0.010, RADIO_TOP
        top, near, foot = SHELF_D, CHEEK_NEAR, MFD_D
        outline = [(0.0, z0), (foot - BACK, z0), (near - BACK, z1 - CHEEK_FLAT[1]), (near - BACK, z1 - CHEEK_FLAT[0]),
                   (top - BACK, z1), (0.0, z1)]
        f = Frame(o, (BACK, side * (RADIO_OUT + 0.003), 0.0), (0.0, -side, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        A.prism(f, outline, 0.0, 0.003, p, p)
        A.flat(f, outline, 0.0, p, normal=-1)
    # the automatic flight control panel
    cz = AFCS_Z0 + AFCS_H / 2
    face = face_tile('afcs-face', AFCS_W, AFCS_H, FACE_PX, afcs_face)
    plate('AFCS.panel', 0.0, cz, AFCS_W, AFCS_H, SHELF_D, SHELF_D + 0.008,
          tex_rect(face, -AFCS_W / 2, AFCS_W / 2, cz - AFCS_H / 2, cz + AFCS_H / 2), color=BEZEL)
    dk = SHELF_D + 0.008
    for kid, leg, y, z, w, h in AFCS_KEYS:
        key('AFCS.%s' % kid, 'AFCS', kid, leg if leg != 'CPLD' else '', y, cz + z, w, h, dk)
    screen('AFCS.readouts', 0.0375, cz + 0.0255, 0.225, 0.013, dk + 0.0004)
    for kid, y, z in AFCS_KNOBS:
        knob('AFCS.%s.knob' % kid, y, cz + z, 0.0095 if kid != 'BRT' else 0.0075, dk, 0.016)
    for kid, prop, deg, step, lo, hi, wrap in (('CRS', '/instrumentation/nav[0]/radials/selected-deg', 1, 1, 0, 360, True),
                                               ('HDG', '/autopilot/settings/heading-bug-deg', 1, 1, 0, 360, True),
                                               ('SPD', '/autopilot/settings/target-speed-kt', 2, 1, 0, 300, False),
                                               ('ALT', '/autopilot/settings/target-altitude-ft', 0.1, 100, 0, 25000, False),
                                               ('HVRALT', '/autopilot/settings/target-agl-ft', 0.5, 10, 0, 5000, False),
                                               ('BRT', '/instrumentation/mv22b/displays/afcs/brightness', 270, 0.1, 0.1, 1, False)):
        XML.append(knob_xml('AFCS.%s.knob' % kid, prop, deg, step, lo, hi, wrap))
    # the radio units and the keys: s in from the housing's outer end
    a0, a1, ph = RADIO_UNIT
    rz = RADIO_TOP - 0.004 - ph / 2
    dark = solid((12, 12, 13))
    for side, name in ((-1, 'RADIO1'), (1, 'RADIO2')):
        at = lambda s_: side * (RADIO_OUT - s_)
        o = plate(name + '.panel', (at(a0) + at(a1)) / 2, rz, a1 - a0, ph, SHELF_D, SHELF_D + 0.006, solid(BEZEL), color=BEZEL)
        dk = SHELF_D + 0.006
        for s_ in (a0 + 0.006, a1 - 0.006):                                                   # its screws
            for z in (rz - ph / 2 + 0.007, rz + ph / 2 - 0.007):
                A.cylinder(o, 0.0018, at(s_), z, dk, dk + 0.0012, dark, dark, n=8)
        for t, s_ in (('CHAN', RADIO_CHAN), ('BRT', RADIO_BRT)):                          # the knobs' labels
            uv = label_uv(t, 0.016, 0.004)
            A.flat(o, [(at(s_) - 0.008, rz + 0.016), (at(s_) + 0.008, rz + 0.016), (at(s_) + 0.008, rz + 0.020),
                       (at(s_) - 0.008, rz + 0.020)], dk + 0.0003, tex_rect(uv, at(s_) - 0.008, at(s_) + 0.008, rz + 0.016, rz + 0.020))
        knob(name + '.CHAN', at(RADIO_CHAN), rz, 0.0085, dk, 0.014)
        knob(name + '.BRT', at(RADIO_BRT), rz, 0.0065, dk, 0.010)
        key(name + '.SEL', name, 'SEL', 'S\nE\nL', at(RADIO_SEL), rz, 0.010, 0.026, dk, 0.004)
        n = 0 if side < 0 else 1
        XML.append(knob_xml(name + '.CHAN', '/instrumentation/comm[%d]/frequencies/selected-mhz' % n, 40, 0.025, 118, 136.975, True))
        XML.append(knob_xml(name + '.BRT', '/instrumentation/mv22b/displays/%s/brightness' % name.lower(), 270, 0.1, 0.1, 1))
        w0, w1, wh = RADIO_WIN
        screen(name + '.screen', (at(w0) + at(w1)) / 2, rz, w1 - w0, wh, dk + 0.0005)
        for k, s_ in enumerate(RADIO_KEYS):                                                 # on the housing's thin part
            key('%s.K%d' % (name, k + 1), name, 'K%d' % (k + 1), '', at(s_), RADIO_TOP - HOUSING_H[1] / 2, 0.025, 0.025,
                SHELF_D, 0.008)


# ---------------------------------------------------------------- the panel's body and the console
PANEL_PITCH = 30.26          # deg: the panel's face back from the vertical (Models/Interior/Panel/all-panels.xml)
FLOOR = -1.105               # m: the cockpit floor below the panel's origin, at the console's aft end (Interior/interior.ac)
BACK = -0.05                 # the panel's back


def extrude(o, profile, y0, y1, uv):
    """a prism across the panel from y0 to y1 of a convex outline in the (d, z) plane"""
    n = len(profile)
    cd = sum(p[0] for p in profile) / n; cz = sum(p[1] for p in profile) / n
    o.face([(d, y0, z) for d, z in profile], [uv(0, 0)] * n, (0, -1, 0))
    o.face([(d, y1, z) for d, z in profile], [uv(0, 0)] * n, (0, 1, 0))
    for (d0, z0), (d1, z1) in zip(profile, profile[1:] + profile[:1]):
        o.face([(d0, y0, z0), (d1, y0, z1), (d1, y1, z1), (d0, y1, z0)], [uv(0, 0)] * 4, ((d0 + d1) / 2 - cd, 0, (z0 + z1) / 2 - cz))


def body(tilt, kp_foot):
    """the face behind the displays' row; the console: under the CDU and its keypads as wide as they are, from the
    panel's face to the keypads' foot (kp_foot), straight down from there and from the panel's back to the floor; aft
    of it the extension, level to a rounded edge and straight down to the floor, the park brake handle at its aft face;
    the wedge under the call panel"""
    o = obj('Panel.body')
    p = solid(PAINT)
    A.box(o, -RADIO_OUT, RADIO_OUT, MFD_Z - MFD_H / 2 - 0.010, AFCS_Z0, BACK, 0.0, p, p)
    sp, cp = math.sin(math.radians(PANEL_PITCH)), math.cos(math.radians(PANEL_PITCH))
    up, aft = (sp, cp), (cp, -sp)                                       # in (d, z)
    down = lambda q, h: (q[0] - h * up[0], q[1] - h * up[1])
    height = lambda q: q[0] * sp + q[1] * cp - FLOOR                    # above the floor
    top = (BACK, tilt.p)
    s0 = (0.0, tilt.p)
    d0, _, z0 = tilt.pt((0.0, 0.0, kp_foot))
    foot = (d0, z0)
    floor0, floor_top = down(foot, height(foot)), down(top, height(top))
    w = CDU_W / 2 + 0.006
    extrude(o, [top, s0, foot, floor0, floor_top], CDU_Y - w, CDU_Y + w, p)
    # the extension: its aft edge rounded, a quarter circle from the level top to the aft face
    corner = (d0 + AFT_L * aft[0], z0 + AFT_L * aft[1])
    c = (corner[0] - EDGE_R * (aft[0] + up[0]), corner[1] - EDGE_R * (aft[1] + up[1]))
    edge = [(c[0] + EDGE_R * (math.cos(a) * up[0] + math.sin(a) * aft[0]), c[1] + EDGE_R * (math.cos(a) * up[1] + math.sin(a) * aft[1]))
            for a in (math.radians(15 * k) for k in range(7))]
    x = obj('Console.extension')
    extrude(x, [foot] + edge + [down(corner, height(corner)), floor0], CDU_Y - w, CDU_Y + w, p)
    park_brake(x, corner, height(corner), sp, cp)
    d4, _, z4 = tilt.pt((0.0, 0.0, tilt.p - 0.006 - CALL_H))
    extrude(o, [top, s0, (d4, z4), (BACK, z4)], CDU_Y + w, ESFI_Y1, p)


EDGE_R = 0.020                        # m: the console extension's rounded aft edge
BRAKE_Y = 0.115                      # the park brake's rod right of the console's middle
BRAKE_T = (0.075, -0.050, 0.030)      # its T-handle above the console's aft edge, its bracket, the rod aft of the face


def park_brake(o, corner, height, sp, cp):
    """the PARK BRAKE handle: a T on a rod standing up the console's aft face through a bracket, pulled up when set
    (corner: the console's aft edge; height: the edge's above the floor)"""
    hi, lo, off = BRAKE_T
    up, aft = (sp, 0.0, cp), (cp, 0.0, -sp)
    at = lambda h: (corner[0] + off * aft[0] + h * up[0], CDU_Y + BRAKE_Y, corner[1] + off * aft[2] + h * up[2])
    metal = solid(CHROME)
    b = Frame(o, at(lo), up, (0.0, 1.0, 0.0), aft)
    closed(b, A.circle(0.012, n=6), -0.011, 0.011, metal, metal)
    A.box(b, -0.008, 0.008, -off, -0.010, -0.008, 0.008, metal, metal)
    h = obj('BRAKE.handle')
    A.cylinder(Frame(h, at(0.0), up, (0.0, 1.0, 0.0), aft), 0.0045, 0, 0, -height, hi, metal, metal, n=12)
    closed(Frame(h, at(hi), (0.0, 1.0, 0.0), up, aft), A.rrect(0.016, 0.022, 0.006), -0.040, 0.040, solid(KEY), solid(KEY))
    XML.append('  <animation>\n    <type>translate</type>\n    <object-name>BRAKE.handle</object-name>\n'
               '    <property>/controls/gear/brake-parking</property>\n    <factor>%.4f</factor>\n    <offset-m>%.4f</offset-m>\n'
               '    <axis>\n      <x>%.4f</x>\n      <y>%.4f</y>\n      <z>%.4f</z>\n    </axis>\n  </animation>\n'
               % ((BRAKE_TRAVEL, -BRAKE_TRAVEL) + up) + toggle_xml('BRAKE.handle', '/controls/gear/brake-parking'))


# ---------------------------------------------------------------- the model file
def model_xml():
    x = ['<?xml version="1.0" encoding="ISO-8859-1"?>',
         '',
         '<!-- The MV-22B\'s instrument panel, glareshield and console, written by fdm/cockpit/panel.py (see there). The screens',
         '     are Canvas surfaces drawn by Nasal/displays.nas; every key calls displays.key(unit, key). -->',
         '',
         '<PropertyList>',
         '',
         '  <path>panel.ac</path>', '']
    screens = [o.name for o in OBJS if o.name.endswith('.screen') or o.name == 'AFCS.readouts']
    # the screens emit (HDR): their luminance in cd/m2 from Nasal/displays.nas
    for s in screens:
        unit = s.split('.')[0].lower() if s != 'AFCS.readouts' else 'afcs'
        x.append('  <effect>\n    <inherits-from>Aircraft/MV-22B/Models/Effects/Lit/lit</inherits-from>\n    <parameters>\n'
                 '      <lit-level><use>/instrumentation/mv22b/displays/%s/luminance</use></lit-level>\n'
                 '      <lit-reflectance type="float">0.04</lit-reflectance>\n'
                 '      <pbr-metallic type="float">1</pbr-metallic>\n      <pbr-roughness type="float">0.3</pbr-roughness>\n'
                 '    </parameters>\n'
                 '    <object-name>%s</object-name>\n  </effect>\n' % (unit, s))
    # the keys
    for name, unit, kid in KEYS:
        x.append('  <animation>\n    <type>pick</type>\n    <object-name>%s</object-name>\n    <action>\n      <button>0</button>\n'
                 '      <repeatable>false</repeatable>\n      <binding>\n        <command>nasal</command>\n'
                 '        <script>displays.key("%s", "%s")</script>\n      </binding>\n    </action>\n  </animation>\n' % (name, unit, kid))
    x += XML
    x.append('</PropertyList>')
    return '\n'.join(x) + '\n'


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    for i, y in enumerate(MFD_Y):
        mfd(i + 1, y)
    under = centre_top()
    FRAME = Tilt(under - 0.006, CONSOLE_TILT)                         # the console: the CDU, its keypads, the call panel
    top = cdu(under) - KP_GAP
    for n, cy in ((1, CDU_Y - (KP_W + 0.004) / 2), (2, CDU_Y + (KP_W + 0.004) / 2)):
        keypad(n, cy, top)
    call_panel(under)
    tcl('R', top)
    tcl('L', top)
    tilt = FRAME
    sp, cp = math.sin(math.radians(PANEL_PITCH)), math.cos(math.radians(PANEL_PITCH))
    FRAME = Frame(None, tilt.pt((0.0, 0.0, top - KP_L)), (sp, 0.0, cp), (0.0, 1.0, 0.0), (-cp, 0.0, sp))  # level: d up, z forward
    aft_section(0.0)
    FRAME = None
    body(tilt, top - KP_L)
    glareshield()
    A.write(os.path.join(OUT, 'panel.ac'), OBJS)
    AT.im.save(os.path.join(OUT, TEX))
    Image.new('RGB', (4, 4), SCREEN_OFF).save(os.path.join(OUT, 'screen.png'))
    rwr_face().save(os.path.join(OUT, RWR_TEX))
    open(os.path.join(OUT, 'panel.xml'), 'w').write(model_xml())
    print('%d objects, %d keys; the atlas filled to %d px' % (len(OBJS), len(KEYS), AT.sky.max()))
