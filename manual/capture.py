#!/usr/bin/env python3
"""The flight manual's pictures of the displays (disp/*.png), taken from FlightGear itself: the aircraft flown over
telnet through the situations the manual shows, the cockpit's camera set square to each screen (its corners from the
panel's model, Models/Interior/Panel/MV22B/panel.ac, through its placements), a white frame drawn round the canvas's
edge for a first screenshot to show exactly where its corners are, and a second without it warped flat from them to
the canvas's own size. Run it when a format changes; the build uses the pictures as they are.

    python3 capture.py [name ...]      all, or the pictures named (FlightGear started here unless one is listening)

(Two ways of taking the canvas itself failed in this FlightGear: a Canvas window draws everything grey, a test
canvas's red, green and blue as 0.3, 0.59 and 0.11 of white; the web server's /screenshot?canvasindex= cannot encode
the canvases' floating point texture, R11F_G11F_B10F, and FlightGear went down at the request.)"""
import os, sys, re, glob, math, time, socket, subprocess, tempfile, shutil
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(HERE, 'disp')
FG = '/Applications/FlightGear.app/Contents/MacOS/FlightGear'
FG_ROOT = os.path.expanduser('~/git/fgdata')
D, J, AF, ST, RM = '/instrumentation/mv22b/displays/', '/fdm/jsbsim/', '/instrumentation/mv22b/afcs/', '/autopilot/settings/', \
    '/autopilot/route-manager/'
SHOTS = tempfile.mkdtemp(prefix='mv22b-capture-')
NOTICE_S = 4.0                 # FlightGear's "Screenshot written to ..." shows for 3 s after each one: waited out
SIZE = {'MFD1': (512, 512), 'MFD2': (512, 512), 'MFD3': (512, 512), 'MFD4': (512, 512), 'CDU': (640, 480), 'SFD': (512, 384),
        'ESFI': (512, 470)}
W, H = 1600, 1000              # the window, and so the screenshot, px
FOV = 20.0                     # the camera's horizontal field of view, deg
DIST = 0.9                     # m from the screen: 700-800 px across a 6 in display


# ---------------------------------------------------------------- where each screen is, and the camera square to it
def screens():
    """each screen's corners in the cockpit view's axes (m right, up, back): panel.ac's object <unit>.screen (AC3D's
    x, y up, z: the model's x aft, z up, -y), placed by all-panels.xml (offsets and pitch) in the aircraft's model,
    whose axes the view's are (right = y, up = z, back = x)"""
    ac = open(os.path.join(ROOT, 'MV-22B', 'Models', 'Interior', 'Panel', 'MV22B', 'panel.ac')).read()
    xml = open(os.path.join(ROOT, 'MV-22B', 'Models', 'Interior', 'Panel', 'all-panels.xml')).read()
    off = {k: float(re.search(r'<%s>\s*([-0-9.]+)\s*</%s>' % (k, k), xml).group(1)) for k in ('x-m', 'y-m', 'z-m', 'pitch-deg')}
    th = math.radians(off['pitch-deg'])
    R = np.array([[math.cos(th), 0, math.sin(th)], [0, 1, 0], [-math.sin(th), 0, math.cos(th)]])     # nose up +
    out = {}
    for unit in SIZE:
        m = re.search(r'name "%s\.screen"\n(?:.*\n)*?numvert 4\n((?:.*\n){4})' % re.escape(unit), ac)
        pts = [tuple(map(float, l.split())) for l in m.group(1).strip().split('\n')]
        model = [R @ np.array([x, -z, y]) + np.array([off['x-m'], off['y-m'], off['z-m']]) for x, y, z in pts]
        out[unit] = [np.array([p[1], p[2], p[0]]) for p in model]                           # view: right, up, back
    return out


def poses():
    """the camera for each screen: DIST along its normal (toward the crew) from its centre, looking back along it; and
    where its corners will be in the screenshot (px), for finding the frame"""
    out = {}
    for unit, c in screens().items():
        ctr = sum(c) / 4
        n = np.cross(c[1] - c[0], c[2] - c[1]); n /= np.linalg.norm(n)
        if n[2] < 0:                                       # toward the crew: aft (view's back, +z)
            n = -n
        eye = ctr + DIST * n
        L = -n
        pitch, hdg = math.degrees(math.asin(L[1])), math.degrees(math.atan2(-L[0], -L[2]))
        f = L; r = np.cross(f, [0, 1, 0]); r /= np.linalg.norm(r); u = np.cross(r, f)
        k = (W / 2) / math.tan(math.radians(FOV / 2))
        px = [(W / 2 + k * np.dot(p - eye, r) / np.dot(p - eye, f), H / 2 - k * np.dot(p - eye, u) / np.dot(p - eye, f)) for p in c]
        out[unit] = ((tuple(eye), hdg, pitch), px)
    return out


def frame_corners(a, b, guess, margin=60):
    """the white frame's outer corners (top left, top right, bottom right, bottom left), px: the pixels white in the
    framed screenshot a and not in b, without it, near where the screen should be (the keys' legends round the CDU are
    white in both), their extremes along the diagonals"""
    xs, ys = [p[0] for p in guess], [p[1] for p in guess]
    x0, x1 = max(0, int(min(xs)) - margin), min(a.shape[1], int(max(xs)) + margin)
    y0, y1 = max(0, int(min(ys)) - margin), min(a.shape[0], int(max(ys)) + margin)
    sa, sb = a[y0:y1, x0:x1].astype(int), b[y0:y1, x0:x1].astype(int)
    yy, xx = np.nonzero((sa.min(2) > 200) & (np.abs(sa - sb).sum(2) > 150))
    if len(xx) < 100:
        sys.exit('no frame found near %s' % guess)
    xx, yy = xx + x0, yy + y0
    pick = lambda v: (xx[np.argmin(v)], yy[np.argmin(v)])
    return [pick(xx + yy), pick(-(xx - yy)), pick(-(xx + yy)), pick(xx - yy)]


def flat(a, corners, w, h):
    """the screenshot warped so that the corners go to the canvas's (0, 0), (w, 0), (w, h), (0, h)"""
    dst = [(0, 0), (w, 0), (w, h), (0, h)]
    A, b = [], []
    for (X, Y), (x, y) in zip(corners, dst):            # output (x, y) to input (X, Y): Pillow's perspective
        A.append([x, y, 1, 0, 0, 0, -X * x, -X * y]); b.append(X)
        A.append([0, 0, 0, x, y, 1, -Y * x, -Y * y]); b.append(Y)
    coef = np.linalg.solve(np.array(A, float), np.array(b, float))
    return Image.fromarray(a).transform((w, h), Image.PERSPECTIVE, tuple(coef), Image.BICUBIC)


POSES = poses()


class Fg:
    def __init__(s):
        s.proc = None
        try:
            s.sock = socket.create_connection(('localhost', 5401), timeout=5)
        except OSError:
            s.start()
        s.f = s.sock.makefile('rw')
        s.f.write('data\r\n'); s.f.flush()
        s.put('/sim/paths/screenshot-dir', SHOTS)              # (a FlightGear already running: this run's folder)

    def start(s):
        """in the air west of KSFO, over the bay towards the peninsula's hills, 120 kt"""
        args = [FG, '--fg-root=' + FG_ROOT, '--fg-aircraft=' + ROOT, '--aircraft=mv22b', '--lat=37.615', '--lon=-122.385',
                '--heading=275', '--altitude=1500', '--vc=120', '--wind=0@0', '--timeofday=noon', '--telnet=5401',
                '--compositor=Compositor/HDR/hdr', '--ignore-autosave', '--geometry=%dx%d' % (W, H),
                '--prop:/sim/paths/screenshot-dir=' + SHOTS]
        for attempt in range(2):                 # (after a crash on exit FlightGear may hold its first start on a dialog)
            s.proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            for _ in range(60):
                try:
                    s.sock = socket.create_connection(('localhost', 5401), timeout=5); return
                except OSError:
                    time.sleep(2)
            s.proc.kill()
        sys.exit('FlightGear did not start')

    def get(s, p):
        s.f.write('get ' + p + '\r\n'); s.f.flush()
        v = s.f.readline().strip()
        try:
            return float(v)
        except ValueError:
            return v

    def put(s, p, v):
        s.f.write('set %s %s\r\n' % (p, v)); s.f.flush()

    def wait_ready(s):
        t0 = time.time()
        while s.get('/sim/signals/fdm-initialized') != 'true':
            if 'scenery' in str(s.get('/sim/startup/splash-progress-text')) and time.time() - t0 > 25:
                s.put('/sim/sceneryloaded-override', 'true')      # (WS3 may never say the tile loaded)
            time.sleep(2)
        time.sleep(8)
        s.put('/controls/electric/battery-switch', 1)
        s.put('/sim/current-view/view-number', 0)

    def view(s, pose):
        """the cockpit view at an eye point (view axes: m right, up, back) looking along heading (left +) and pitch
        (up +), deg"""
        (x, y, z), hdg, pitch = pose
        for k, v in (('x-offset-m', x), ('y-offset-m', y), ('z-offset-m', z), ('heading-offset-deg', hdg), ('goal-heading-offset-deg', hdg),
                     ('pitch-offset-deg', pitch), ('goal-pitch-offset-deg', pitch), ('roll-offset-deg', 0), ('field-of-view', FOV)):
            s.put('/sim/current-view/' + k, v)

    def shot(s):
        """a screenshot (the notice of the one before waited out), as an array"""
        before = set(glob.glob(os.path.join(SHOTS, '*.png')))
        s.f.write('run screen-capture\r\n'); s.f.flush()
        for _ in range(40):
            time.sleep(0.25)
            new = set(glob.glob(os.path.join(SHOTS, '*.png'))) - before
            if new:
                time.sleep(0.8)
                f = new.pop()
                a = np.asarray(Image.open(f).convert('RGB'))
                os.remove(f)
                time.sleep(NOTICE_S)
                return a
        sys.exit('no screenshot')

    def capture(s, unit, name, page=''):
        """the unit's screen (on the page given), flat at the canvas's size, to disp/<name>.png"""
        if page:
            s.put(D + 'show', unit + ' ' + page)
        s.view(POSES[unit][0])
        s.put(D + 'show', unit + ' FRAME')
        time.sleep(1.2)
        framed = s.shot()
        s.put(D + 'show', unit + ' NOFRAME')
        time.sleep(0.5)
        img = s.shot()
        corners = frame_corners(framed, img, POSES[unit][1])
        w, h = SIZE[unit]
        flat(img, corners, w, h).save(os.path.join(OUT, name + '.png'))
        print('%-16s %s %s' % (name, unit, page), flush=True)

    def fly(s, sec, every=10, what=()):
        t0 = time.time()
        while time.time() - t0 < sec:
            time.sleep(min(every, sec - (time.time() - t0)))
            print('   %5.0f s: %5.1f kt, %5.0f ft, %4.0f ft above the ground, nacelles %4.1f %s' % (
                time.time() - t0, s.get('/instrumentation/airspeed-indicator/indicated-speed-kt'), s.get('/position/altitude-ft'),
                s.get('/position/altitude-agl-ft'), s.get(J + 'ctl/nacelle-deg'), ' '.join(str(s.get(p)) for p in what)), flush=True)


def route(fg, *pts):
    fg.put(RM + 'input', '@clear'); time.sleep(0.5)
    for lat, lon in pts:
        fg.put(RM + 'input', '%f,%f' % (lon, lat)); time.sleep(0.5)
    fg.f.write('run activate-flightplan\r\n'); fg.f.flush(); time.sleep(1)


def main(want):
    os.makedirs(OUT, exist_ok=True)
    fg = Fg()
    fg.wait_ready()
    try:
        flight(fg, want)
    except StopIteration:
        pass
    fg.sock.close()
    shutil.rmtree(SHOTS, ignore_errors=True)
    print('done; FlightGear left running' if fg.proc is None else 'done; FlightGear (started here) left running: quit it from its menu')


def flight(fg, want):
    left = set(want)

    def take(unit, name, page=''):
        if want and name not in want:
            return
        fg.capture(unit, name, page)
        left.discard(name)
        if want and not left:                  # the pictures asked for taken: the rest of the flight not needed
            raise StopIteration
    # coupled at 120 kt, a route over the peninsula's ridge to the coast and a landing zone south of it (drawn on the
    # map, not flown)
    hdg = round(fg.get('/orientation/heading-magnetic-deg'))
    fg.put(ST + 'heading-bug-deg', hdg); fg.put(ST + 'target-altitude-ft', 1500); fg.put(ST + 'target-speed-kt', 120)
    fg.put(ST + 'target-agl-ft', 300)
    for m in ('hdg-hold', 'alt-hold', 'spd-hold', 'auto-nac', 'cpld'):
        fg.put(AF + m, 1)
    fg.put('/controls/gear/gear-down', 0)
    route(fg, (37.632, -122.470), (37.600, -122.510), (37.515, -122.500))
    fg.fly(20)
    take('MFD1', 'flt_conversion')
    take('MFD3', 'nav')
    take('MFD2', 'eng')
    take('MFD4', 'fuel', 'FUEL')
    take('CDU', 'cdu_eicas', 'EICAS')
    take('CDU', 'cdu_stat', 'STAT')
    take('CDU', 'cdu_pad', 'CDU')
    take('SFD', 'sfd')
    take('ESFI', 'esfi')
    # terrain following over the peninsula's ridge, at 300 ft
    fg.put(AF + 'press', 'TF')
    fg.fly(40, what=(J + 'ap/tf-on',))
    take('MFD1', 'flt_tf')
    take('MFD3', 'nav_tf')
    fg.put(AF + 'press', 'TF')
    # airplane mode, out over the sea
    fg.put(ST + 'target-speed-kt', 230); fg.put(ST + 'target-altitude-ft', 2500)
    fg.fly(70)
    take('MFD1', 'flt_airplane')
    # slowed down to a hover 150 ft over the sea (under 200 ft the format's radar height gauge), held
    fg.put(ST + 'target-speed-kt', 30); fg.put(ST + 'target-altitude-ft', 150)
    fg.fly(60)
    fg.put('/controls/gear/gear-down', 1)                    # (below 150 kt by then)
    fg.fly(100)
    fg.put(J + 'hh/engage-cmd', 1)
    fg.fly(30, what=(J + 'hh/state',))
    take('MFD1', 'flt_hover')
    take('MFD3', 'nav_hover')


if __name__ == '__main__':
    main(set(sys.argv[1:]))
