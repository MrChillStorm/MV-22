#!/usr/bin/env python3
"""The proprotors' motion blur of the blades' own geometry (the method and the blur of the UH-60 and Bo 105 models'
fdm/rotorvis.py).

The 3-D model turned each rotor's three blades with the flight model's azimuth (/rotors/main/blade[0]/position-deg),
tilted them with the gimbal and pitched each blade (/rotors/main/blade[k]/incidence-deg), and laid a blurred-blade
object over them from 60 rpm and a disc of translucent wedges from 150 rpm, the blades staying opaque throughout (the
rotor turns at 333-413 rpm). Here, for each rotor, in Models/Rotor/blur-left and blur-right:

- as the rotor speeds up each blade trails a fan of STRIPS faint wedges over SWEEP deg per rpm, which reaches the next
  blade at 100 % rotor speed: a translucent disc as dense on average as the blades' time-averaged coverage x GAIN, a
  smear at the blade's own position (as the UH-60's and the Bo 105's: the blades pass a point 20 times a second, near
  the eye's flicker fusion, and the display samples the smears as a camera does). The fans lie in the blades' plane
  with their built-in coning (the 3-D model's, 1 deg), from the spinner to the tip, in the blades' colours along the
  span (dark grey, the red tip stripe: the same in every livery); they don't pitch;
- each blade has a translucent copy (t...), pitched on its own blade's properties as the blade and unfolded, that fades
  out as the fan fades in: opaque blade below SOLID_RPM, then the copy fading out to nothing at FADE_RPM, both with the
  rpm and casting partial shadows (Models/Effects/RotorBlur: the HDR pipeline; elsewhere they switch, without shadows);
- each rotor's blur model is a submodel (RotorBlurG, RotorBlurD) that v22.xml turns with its rotor (wing stow, nacelle
  tilt, gimbal and azimuth, its groups LesAiles, LesMoteurs, HeliceGauche/Droite); the blades' pitch and fold axes,
  the hubs' and the spin directions are read from v22.xml and v22.ac. The old blur objects (propblur, propdisc) are
  hidden; the opaque blades only show below SOLID_RPM (they fold, stopped, for the wing's stowing).

In FlightGear's model frame (x aft, y right, z up) the left rotor turns about -z, the right one about +z (v22.xml), and
blade 1 of each points inboard; an AC3D file's frame is (x, z, -y) of it. A rotate animation turns by the right-hand
rule, and the first animation of an object is the outermost.

    python3 rotorvis.py      writes MV-22B/Models/Rotor/ (blur-left.ac/.xml, blur-right.ac/.xml, blur.png),
                             Models/Effects/RotorBlur, Shaders/HDR/rotor_blur*.frag and the rotors' part of v22.xml
"""
import math, os, re, shutil
import xml.etree.ElementTree as ET
from PIL import Image

import v22data as U

HERE = os.path.dirname(os.path.abspath(__file__))
AIRCRAFT = os.path.join(HERE, '..', 'MV-22B')
MODELS = os.path.join(AIRCRAFT, 'Models')
DIR = os.path.join(MODELS, 'Rotor')
BO105 = os.path.expanduser('~/git/Bo105/MBB_Bo_105')         # the blur effect and its shaders, as the Bo 105's

RPM = U.OMEGA_100 * 30 / math.pi        # 100 %: 397 rpm
STRIPS = 24                             # wedges per blade
SWEEP = 360.0 / U.NB / RPM              # deg per rpm: the fan reaches the next blade at 100 % rotor speed
GAIN = 2.5                              # x the blades' time-averaged coverage
IMAGE, SMEAR = 0.5, 2.0                 # the share of it in the blades' smears; their decay, in wedges
SOLID_RPM, FADE_RPM = 0.08 * RPM, 0.62 * RPM      # the opaque blade below; from it to FADE_RPM the copy fades out, the fan in
R_IN = 0.6                              # m: the fans' inner edge, at the spinner (bolG and bolD reach 0.54 m)
PIECES = 16
SIDES = [('G', 'left', 'Gauche'), ('D', 'right', 'Droite')]
PARTS = ['piedpale', 'pale']            # each blade: its root and cuff, and the blade
BEGIN, END = '  <!-- the rotors\' blur: written by fdm/rotorvis.py -->', '  <!-- end of fdm/rotorvis.py\'s rotors -->'


def ac_blocks(path):
    text = open(path, encoding='latin-1').read()
    head, *objs = re.split(r'(?m)^(?=OBJECT poly)', text)
    return head, {re.search(r'^name "([^"]+)"', b, re.M).group(1): b for b in objs}


def vertices(block):
    """an object's vertices in FlightGear's frame and its texture coordinates by vertex"""
    L = block.split('\n')
    loc = next(([float(c) for c in l.split()[1:]] for l in L if l.startswith('loc')), [0.0, 0.0, 0.0])
    i = next(k for k, l in enumerate(L) if l.startswith('numvert')); n = int(L[i].split()[1])
    P = []
    for k in range(n):
        x, y, z = (float(c) for c in L[i + 1 + k].split()[:3])
        P.append((x + loc[0], -(z + loc[2]), y + loc[1]))
    uv = {}
    for m in (re.match(r'^(\d+) (\S+) (\S+)$', l) for l in L[i + n + 1:]):
        if m:
            uv[int(m.group(1))] = (float(m.group(2)), float(m.group(3)))
    return P, uv


def vec(e, tag):
    return tuple(float(e.find('%s/%s' % (tag, a)).text) for a in (('x-m', 'y-m', 'z-m') if tag == 'center' else ('x', 'y', 'z')))


def rotates(root):
    """v22.xml's rotate animations by their (first) object-name"""
    out = {}
    for a in root.iter('animation'):
        if a.findtext('type', '').strip() == 'rotate' and a.find('object-name') is not None:
            out.setdefault(a.find('object-name').text.strip(), []).append(a)
    return out


def geometry():
    """each rotor's hub, spin axis, blade 1's direction, the blades' plane and their pitch and fold animations"""
    root = ET.parse(os.path.join(MODELS, 'v22.xml')).getroot()
    rot = rotates(root)
    head, objs = ac_blocks(os.path.join(MODELS, 'v22.ac'))
    g = {}
    for s, side, name in SIDES:
        hub = next(a for a in rot['Helice' + name] if 'position-deg' in a.findtext('property'))
        hc, spin = vec(hub, 'center'), vec(hub, 'axis')
        pitch = {}
        for k in (1, 2, 3):
            a = rot['PiedPale%s%d' % ('Gauche' if s == 'G' else 'Droit', k)][0]
            pitch[k] = dict(prop=a.findtext('property').strip(), factor=float(a.findtext('factor')),
                            offset=float(a.findtext('offset-deg')), center=vec(a, 'center'), axis=vec(a, 'axis'))
        fold = {}
        for k in (2, 3):
            a = next(a for a in rot['pale%s%d' % (s, k)] if 'float[21]' in a.findtext('property'))
            unfolded = next(float(e.findtext('dep')) for e in a.find('interpolation') if float(e.findtext('ind')) == 0)
            fold[k] = dict(angle=unfolded, center=vec(a, 'center'), axis=vec(a, 'axis'))
        # blade 1's pitch axis: its direction in the rotor plane, the coning, a point on it
        c, ax = pitch[1]['center'], pitch[1]['axis']
        h = math.hypot(ax[0], ax[1])
        e1 = (ax[0] / h, ax[1] / h, 0.0)
        if (c[0] - hc[0]) * e1[0] + (c[1] - hc[1]) * e1[1] < 0:
            e1 = (-e1[0], -e1[1], 0.0)
        cone = math.atan2(abs(ax[2]), h)
        rp = math.hypot(c[0] - hc[0], c[1] - hc[1])
        P, _ = vertices(objs['pale%s1' % s])
        tip = max(math.hypot(p[0] - hc[0], p[1] - hc[1]) for p in P)
        # the azimuth of each blade from blade 1, about the spin axis (deg): from the roots' (unfolded) positions
        az = {}
        for k in (1, 2, 3):
            P, _ = vertices(objs['piedpale%s%d' % (s, k)])
            mx, my = sum(p[0] for p in P) / len(P) - hc[0], sum(p[1] for p in P) / len(P) - hc[1]
            a = math.degrees(math.atan2(e1[0] * my - e1[1] * mx, e1[0] * mx + e1[1] * my)) * spin[2]
            az[k] = 120.0 * round(a / 120.0) % 360
        g[s] = dict(side=side, name=name, hub=hc, spin=spin, e1=e1, cone=cone, zp=c[2], rp=rp, tip=tip,
                    pitch=pitch, fold=fold, az=az)
    return g


def colours(span):
    """the blades' paint along span's pieces: texture.png at blade 1's (the left one's) texture coordinates, linear in
    the radius over the blade and its cuff (the paint is the same in every livery)"""
    head, objs = ac_blocks(os.path.join(MODELS, 'v22.ac'))
    hc = G['G']['hub']
    im = Image.open(os.path.join(MODELS, 'texture.png')).convert('RGB'); W, H = im.size
    fits = []
    for part in PARTS:
        P, uv = vertices(objs['%sG1' % part])
        pts = [(math.hypot(P[k][0] - hc[0], P[k][1] - hc[1]),) + uv[k] for k in uv]
        n = len(pts); mr = sum(p[0] for p in pts) / n
        fit = []
        for j in (1, 2):
            m = sum(p[j] for p in pts) / n
            b = sum((p[0] - mr) * (p[j] - m) for p in pts) / sum((p[0] - mr) ** 2 for p in pts)
            fit.append((m - b * mr, b))
        fits.append((min(p[0] for p in pts), max(p[0] for p in pts), fit))
    out = []
    for r0, r1 in span:
        px = []
        for i in range(8):
            r = r0 + (r1 - r0) * (i + 0.5) / 8
            lo, hi, fit = fits[1] if r >= fits[1][0] else fits[0]
            u, v = (fit[0][0] + fit[0][1] * r, fit[1][0] + fit[1][1] * r)
            px.append(im.getpixel((min(W - 1, max(0, int(u * W))), min(H - 1, max(0, int((1 - v) * H))))))
        out.append(tuple(round(sum(q[c] for q in px) / len(px) / 255.0, 3) for c in range(3)))
    return out


def strip(s, k, j):
    return 'blur%s%d_%d' % (s, k, j)


def to_ac(p):
    return (p[0], p[2], -p[1])


def fan_ac(s, span, rgb):
    """Models/Rotor/blur-<side>.ac: the three blades' fans, all at blade 1 (blur-<side>.xml turns them to their blades
    and back along the rotation), and the blades' translucent copies; blur.png the wedges' colours and opacities"""
    r = G[s]
    hx, hy, _ = r['hub']; e1 = r['e1']; sp = r['spin']
    t1 = (sp[1] * e1[2] - sp[2] * e1[1], sp[2] * e1[0] - sp[0] * e1[2], sp[0] * e1[1] - sp[1] * e1[0])     # spin x e1
    tc = math.tan(r['cone'])

    def point(rr, a):
        """at radius rr, a rad along the rotation from blade 1, in the blades' plane (coned)"""
        return (hx + rr * (e1[0] * math.cos(a) + t1[0] * math.sin(a)), hy + rr * (e1[1] * math.cos(a) + t1[1] * math.sin(a)),
                r['zp'] + (rr - r['rp']) * tc)
    d = math.radians(360.0 / U.NB / (STRIPS - 1))           # their spacing at 100 % rpm; each 2 d wide
    g = [math.exp(-k / SMEAR) for k in range(STRIPS)]
    head, objs = ac_blocks(os.path.join(MODELS, 'v22.ac'))
    mats = [l for l in head.split('\n') if l.startswith('MATERIAL')]
    mats.append('MATERIAL "blur" rgb 1 1 1  amb 1 1 1  emis 0 0 0  spec 0 0 0  shi 0  trans 0')
    blur_mat = len(mats) - 1
    out = []
    for k in (1, 2, 3):
        for j in range(STRIPS):
            w = (1 - IMAGE) * (0.5 if j in (0, STRIPS - 1) else 1.0) + IMAGE * STRIPS * g[j] / sum(g)   # mean 1
            faces = []
            for i, (r0, r1) in enumerate(span):
                chord = (U.CHORD_ROOT + (U.CHORD_TIP - U.CHORD_ROOT) * math.sqrt(r0 * r1) / (U.R * 0.3048)) * 0.3048
                sigma = U.NB * chord / (2 * math.pi * math.sqrt(r0 * r1))
                a = min(0.9, GAIN * sigma / 2 * w)          # two wedges over each point
                faces.append((swatch(rgb[i], a), [point(r0, 0.0), point(r1, 0.0), point(r1, -2 * d), point(r0, -2 * d)]))
            V = [to_ac(pt) for _, pts in faces for pt in pts]
            lines = ['OBJECT poly', 'name "%s"' % strip(s, k, j), 'loc 0 0 0', 'texture "blur.png"', 'texrep 1 1',
                     'numvert %d' % len(V)] + ['%.5f %.5f %.5f' % p for p in V] + ['numsurf %d' % len(faces)]
            for f, (uv, _) in enumerate(faces):
                lines += ['SURF 0x30', 'mat %d' % blur_mat, 'refs 4'] + ['%d %.5f %.5f' % ((4 * f + q,) + uv) for q in range(4)]
            out.append('\n'.join(lines + ['kids 0']))
    for k in (1, 2, 3):
        for part in PARTS:
            b = objs['%s%s%d' % (part, s, k)].rstrip('\n')
            b = b[:b.index('\nkids 0') + len('\nkids 0')]
            b = re.sub(r'(?m)^name "[^"]+"', 'name "t%s%s%d"' % (part, s, k), b)
            out.append(b.replace('texture "texture.png"', 'texture "../texture.png"'))
    text = '\n'.join(['AC3Db'] + mats + ['OBJECT world', 'kids %d' % len(out)] + out) + '\n'
    open(os.path.join(DIR, 'blur-%s.ac' % r['side']), 'w', encoding='latin-1').write(text)


CELLS, SEEN = [], {}
TEX = Image.new('RGBA', (256, 256), (0, 0, 0, 0))


def swatch(rgb, a):
    key = (rgb, int(round(255 * a)))
    if key in SEEN:
        return SEEN[key]
    n = len(CELLS); x, y = 8 * (n % 32), 8 * (n // 32)
    assert n < 1024, 'blur.png is full'
    TEX.paste(tuple(int(round(255 * c)) for c in rgb) + (int(round(255 * a)),), (x, y, x + 8, y + 8))
    CELLS.append(((x + 4) / 256.0, 1 - (y + 4) / 256.0)); SEEN[key] = CELLS[-1]
    return CELLS[-1]


def objects(names, indent='    '):
    return ''.join('%s<object-name>%s</object-name>\n' % (indent, n) for n in names)


def rotate(objs, prop, axis, centre, factor=None, offset=None, comment=None):
    out = '\n  <!-- %s -->\n' % comment if comment else '\n'
    out += '  <animation>\n    <type>rotate</type>\n' + objects(objs)
    if prop:
        out += '    <property>%s</property>\n' % prop
    if factor is not None:
        out += '    <factor> %.6g </factor>\n' % factor
    if offset is not None:
        out += '    <offset-deg> %.6g </offset-deg>\n' % offset
    return out + ('    <center>\n      <x-m> %.6f </x-m>\n      <y-m> %.6f </y-m>\n      <z-m> %.6f </z-m>\n    </center>\n'
                  '    <axis>\n      <x> %g </x>\n      <y> %g </y>\n      <z> %g </z>\n    </axis>\n  </animation>\n'
                  % (tuple(centre) + tuple(axis)))


def select(objs, cond, comment=None):
    out = '\n  <!-- %s -->\n' % comment if comment else '\n'
    return out + '  <animation>\n    <type>select</type>\n' + objects(objs) + '    <condition>\n%s    </condition>\n  </animation>\n' % cond


def fade(objs, f, t, rough, shadow, comment):
    """the rotor-blur effect on objs, faded from f to t rpm (of /rotors/main/rpm), its paint's roughness rough (the
    transparent shader's default is a gloss that mirrors the sky)"""
    return ('\n  <!-- %s -->\n  <effect>\n    <inherits-from>../Effects/RotorBlur/rotor-blur</inherits-from>\n'
            '    <parameters>\n      <fade-from type="float"> %g </fade-from>\n      <fade-to type="float"> %g </fade-to>\n'
            '      <pbr-metallic type="float"> 0 </pbr-metallic>\n      <pbr-roughness type="float"> %g </pbr-roughness>\n'
            '      <shadow-gain type="float"> %g </shadow-gain>\n    </parameters>\n' % (comment, f, t, rough, shadow) +
            objects(objs) + '  </effect>\n')


def rpm(r):
    return '<property>rotors/main/rpm</property><value> %g </value>' % r


def blur_xml(s):
    r = G[s]
    strips = [strip(s, k, j) for k in (1, 2, 3) for j in range(STRIPS)]
    copies = ['t%s%s%d' % (part, s, k) for k in (1, 2, 3) for part in PARTS]
    x = ('<?xml version="1.0" encoding="ISO-8859-1"?>\n\n<!-- The %s rotor\'s motion blur, written by fdm/rotorvis.py (see there): '
         'v22.xml turns it with\n     the rotor (RotorBlur%s) -->\n\n<PropertyList>\n\n  <path>blur-%s.ac</path>\n' % (r['side'], s, r['side']))
    x += fade(strips, SOLID_RPM, FADE_RPM, 1.0, 2.0, 'the fans fade in with the rpm; their shadow x 2: two wedges over each point')
    x += fade(copies, FADE_RPM, SOLID_RPM, 0.5, 1.0, 'the blades\' copies fade out')
    x += select(copies, '      <and>\n        <not><less-than>%s</less-than></not>\n        <less-than>%s</less-than>\n      </and>\n'
                % (rpm(SOLID_RPM), rpm(FADE_RPM)), 'the copies between the opaque blades and the fans')
    x += select(strips, '      <not><less-than>%s</less-than></not>\n' % rpm(SOLID_RPM), 'and the fans')
    # the copies: pitched as their blades (v22.xml's PiedPale animations), then unfolded (the fold's angle at 0)
    for k in (1, 2, 3):
        p = r['pitch'][k]
        objs = ['t%s%s%d' % (part, s, k) for part in PARTS]
        x += rotate(objs, p['prop'], p['axis'], p['center'], factor=p['factor'], offset=p['offset'],
                    comment='blade %d\'s copy pitched as the blade' % k if k == 1 else None)
    for k in (2, 3):
        f = r['fold'][k]
        x += rotate(['tpale%s%d' % (s, k)], None, f['axis'], f['center'], offset=f['angle'],
                    comment='blades 2 and 3 unfolded (v22.ac has them folded)' if k == 2 else None)
    # the fans: to their blades, each wedge back along the rotation, narrowed with the rpm
    for k in (2, 3):
        x += rotate([strip(s, k, j) for j in range(STRIPS)], None, r['spin'], r['hub'], offset=r['az'][k],
                    comment='blade %d\'s fan at its blade, %.0f deg on from blade 1 along the rotation' % (k, r['az'][k]))
    for j in range(1, STRIPS):
        x += rotate([strip(s, k, j) for k in (1, 2, 3)], 'rotors/main/rpm', r['spin'], r['hub'], factor=-SWEEP * j / (STRIPS - 1),
                    comment='wedge j trails its blade by j / %d of the fan' % (STRIPS - 1) if j == 1 else None)
    x += ('\n  <!-- the wedges as wide as twice their spacing at any rpm (blur-%s.ac: at 100 %%), across blade 1 (x) -->\n'
          '  <animation>\n    <type>scale</type>\n' % r['side'] + objects(strips) +
          '    <property>rotors/main/rpm</property>\n    <x-factor> %.7f </x-factor>\n    <x-offset> 0 </x-offset>\n'
          '    <x-min> 0.02 </x-min>\n    <x-max> 1 </x-max>\n    <y-factor> 0 </y-factor>\n    <y-offset> 1 </y-offset>\n'
          '    <z-factor> 0 </z-factor>\n    <z-offset> 1 </z-offset>\n'
          '    <center>\n      <x-m> %.6f </x-m>\n      <y-m> %.6f </y-m>\n      <z-m> %.6f </z-m>\n    </center>\n  </animation>\n'
          % ((1.0 / RPM,) + tuple(r['hub'])))
    open(os.path.join(DIR, 'blur-%s.xml' % r['side']), 'w', encoding='latin-1').write(x + '\n</PropertyList>\n')


def install():
    """v22.xml: the blur submodels in the rotors' groups, the opaque blades only below SOLID_RPM, the old blur hidden"""
    p = os.path.join(MODELS, 'v22.xml'); t = open(p, encoding='latin-1').read()
    blades = ['%s%s%d' % (part, s, k) for s, _, _ in SIDES for k in (1, 2, 3) for part in PARTS]
    block = [BEGIN]
    for s, side, _ in SIDES:
        block.append('  <model>\n    <name>RotorBlur%s</name>\n    <path>Rotor/blur-%s.xml</path>\n  </model>' % (s, side))
    block.append(select(blades, '      <less-than>%s</less-than>\n' % rpm(SOLID_RPM),
                        'the opaque blades below %.0f rpm (Rotor/blur-*.xml fade them out above)' % SOLID_RPM).rstrip('\n'))
    block.append(select(['propdisc%s' % s for s, _, _ in SIDES] + ['propblur%s' % s for s, _, _ in SIDES],
                        '      <property>sim/model/v22/old-rotor-blur</property>\n',
                        'the original blur and disc: not shown').rstrip('\n'))
    block.append(END)
    block = '\n'.join(block) + '\n'
    if BEGIN in t:
        t = t[:t.index(BEGIN)] + block + t[t.index(END) + len(END) + 1:]
    else:
        i = t.index('  <!-- Rotation des ailes -->')
        t = t[:i] + block + '\n' + t[i:]
    # the submodels in the groups that turn the rotors
    for group, s in [('LesAiles', 'G'), ('LesAiles', 'D'), ('LesMoteurs', 'G'), ('LesMoteurs', 'D'),
                     ('HeliceGauche', 'G'), ('HeliceDroite', 'D')]:
        m = re.search(r'(<name>%s</name>\n)((?:    <object-name>[^<]+</object-name>\n)+)' % group, t)
        if '<object-name>RotorBlur%s</object-name>' % s not in m.group(2):
            t = t[:m.end()] + '    <object-name>RotorBlur%s</object-name>\n' % s + t[m.end():]
    open(p, 'w', encoding='latin-1').write(t)


def effect():
    """Models/Effects/RotorBlur and its shaders, the Bo 105's with this aircraft's paths"""
    os.makedirs(os.path.join(MODELS, 'Effects', 'RotorBlur'), exist_ok=True)
    os.makedirs(os.path.join(AIRCRAFT, 'Shaders', 'HDR'), exist_ok=True)
    e = open(os.path.join(BO105, 'Models', 'Effects', 'RotorBlur', 'rotor-blur.eff'), encoding='latin-1').read()
    e = e.replace('Aircraft/MBB_Bo_105/', 'Aircraft/MV-22B/').replace('(fdm/rotorvis.py, after the UH-60 model\'s)',
                                                                       '(fdm/rotorvis.py, as the Bo 105 and UH-60 models\')')
    open(os.path.join(MODELS, 'Effects', 'RotorBlur', 'rotor-blur.eff'), 'w', encoding='latin-1').write(e)
    for f in ('rotor_blur.frag', 'rotor_blur_shadow.frag'):
        shutil.copy(os.path.join(BO105, 'Shaders', 'HDR', f), os.path.join(AIRCRAFT, 'Shaders', 'HDR', f))


if __name__ == '__main__':
    os.makedirs(DIR, exist_ok=True)
    G = geometry()
    tip = G['G']['tip']
    span = [(R_IN * (tip / R_IN) ** (i / PIECES), R_IN * (tip / R_IN) ** ((i + 1) / PIECES)) for i in range(PIECES)]
    rgb = colours(span)
    for s, _, _ in SIDES:
        fan_ac(s, span, rgb)
        blur_xml(s)
    TEX.save(os.path.join(DIR, 'blur.png'))
    effect()
    install()
    for s, _, _ in SIDES:
        r = G[s]
        print('%s rotor: hub %.3f %.3f, spin %s, blade 1 along %.3f %.3f, cone %.2f deg, tip %.3f m, blades at %s deg' % (
            r['side'], r['hub'][0], r['hub'][1], r['spin'], r['e1'][0], r['e1'][1], math.degrees(r['cone']), r['tip'], r['az']))
    print('colours along the span:', ' '.join('%.2f/%.2f/%.2f' % c for c in rgb))
    print('%d wedges per blade over %.0f deg at %.0f rpm, fan from %.0f rpm, blade gone at %.0f' % (STRIPS, 360 / U.NB, RPM, SOLID_RPM, FADE_RPM))
