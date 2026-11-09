"""(fg_lowi.py, from fg_random.py) ENG OUT's landing site finder in FlightGear (eosite.nas + genv22 eng_out_site): an in-air start over the south bay at
6000 ft and 200 KCAS heading for KSFO, coupled; both engines cut; nothing steered from here: ENG OUT and the finder
choose and fly; the gear when the mode asks, the brakes after touchdown. Logged: the site chosen, its margin, where the
flare began against its point and line, the touchdown; NAV and FLT pictures. (Below: the earlier test's docstring.)
ENG OUT in FlightGear: an in-air start on KSFO 28R's extended centreline over the bay, 8 NM out, 3000 ft, 200 kt in
airplane mode, still air; coupled ALT/SPD/HDG HOLD, the heading bug steered onto the centreline (the crew's job: ENG
OUT holds the bug's heading). Both engines cut where the glide's reach (ap/eo-reach-nm) plus the flare's run lands the
aircraft RUN_AIM ft past the threshold; ENG OUT comes on by itself, CPLD flies it, the gear lowered when the mode line
asks (below ap/eo-gear-ft), the brakes after touchdown. A second telnet connection logs at 10 Hz (pipelined gets) and
steers; the main one takes the flight and nav formats' pictures (glide, flare) to the scratchpad. FlightGear on its
own port (5411: the ASK21 runs use 5401), launched here once, waited for (it needs the user's click)."""
import sys, os, time, math, json, socket, subprocess, threading, shutil, tempfile
sys.path.insert(0, os.path.expanduser('~/git/V-22/manual'))
sys.path.insert(0, os.path.expanduser('~/git/V-22/fdm'))
sys.argv = ['capture.py']
import capture as C
import v22data as U

S = (os.environ.get('EOBENCH_OUT') or os.path.join(tempfile.gettempdir(), 'eobench'))      # (EOBENCH_OUT: where the flights' logs go)
os.makedirs(S, exist_ok=True)
PORT = 5411
J, AF, ST = C.J, C.AF, C.ST
FT_NM = 6076.1
THR = (37.61351918, -122.35716907)          # 28R's threshold (apt.dat), its displacement 103.94 m
END = (37.62872250, -122.39342127)          # 10L's
DISP_FT = 103.94 / 0.3048
RUN_NM = 0.6                                # the flare's run, gate to touchdown (deadland.py auto: 0.5-0.75)
RUN_AIM = DISP_FT + 2000.0                  # the touchdown aimed for, ft past the threshold


def course(a, b):
    la, lb = math.radians(a[0]), math.radians(b[0])
    dl = math.radians(b[1] - a[1])
    return math.degrees(math.atan2(math.sin(dl) * math.cos(lb), math.cos(la) * math.sin(lb) - math.sin(la) * math.cos(lb) * math.cos(dl))) % 360


CRS = course(THR, END)
LEN_FT = math.hypot((END[1] - THR[1]) * math.cos(math.radians(THR[0])), END[0] - THR[0]) * 60 * FT_NM


def offset(lat, lon):
    """along the runway's course from the threshold (ft, + past it) and across it (ft, + right)"""
    dx = (lon - THR[1]) * math.cos(math.radians(THR[0])) * 60 * FT_NM
    dy = (lat - THR[0]) * 60 * FT_NM
    c = math.radians(CRS)
    return dx * math.sin(c) + dy * math.cos(c), dx * math.cos(c) - dy * math.sin(c)


def start_point(nm):
    c = math.radians(CRS + 180)
    return (THR[0] + nm * math.cos(c) / 60, THR[1] + nm * math.sin(c) / 60 / math.cos(math.radians(THR[0])))


class Fg(C.Fg):
    def __init__(s, launch=True):
        s.proc = None
        try:
            s.sock = socket.create_connection(('localhost', PORT), timeout=5)
        except OSError:
            if not launch:
                raise
            s.start()
        s.sock.settimeout(60)
        s.f = s.sock.makefile('rw')
        s.f.write('data\r\n'); s.f.flush()
        s.put('/sim/paths/screenshot-dir', C.SHOTS)

    def start(s):
        lat, lon = 37.548, -122.268
        args = [C.FG, '--fg-root=' + C.FG_ROOT, '--fg-aircraft=' + C.ROOT, '--aircraft=mv22b', '--lat=%.6f' % lat, '--lon=%.6f' % lon,
                '--heading=315', '--altitude=6000', '--vc=200', '--wind=0@0', '--timeofday=noon', '--telnet=%d' % PORT,
                '--compositor=Compositor/HDR/hdr', '--ignore-autosave', '--geometry=%dx%d' % (C.W, C.H),
                '--prop:/sim/paths/screenshot-dir=' + C.SHOTS]
        s.proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print('FlightGear launched (pid %d): waiting for the click' % s.proc.pid, flush=True)
        for _ in range(300):                     # 10 minutes: one launch, never a second
            try:
                s.sock = socket.create_connection(('localhost', PORT), timeout=5); return
            except OSError:
                time.sleep(2)
        sys.exit('FlightGear did not open its telnet port in 10 minutes (left running)')

    def many(s, paths):
        s.f.write(''.join('get %s\r\n' % p for p in paths)); s.f.flush()
        out = []
        for _ in paths:
            v = s.f.readline().strip()
            try:
                out.append(float(v))
            except ValueError:
                out.append({'true': 1.0, 'false': 0.0}.get(v, float('nan')))
        return out



SITE = '/instrumentation/mv22b/eosite/'
LOG = [('hw', '/position/gear-agl-m'), ('kcas', J + 'velocities/vc-kts'), ('gs', '/velocities/groundspeed-kt'), ('hdot', J + 'velocities/h-dot-fps'),
       ('theta', J + 'attitude/theta-deg'), ('phi', J + 'attitude/phi-deg'), ('ph', J + 'ap/eo-phase'), ('eo', AF + 'eng-out'),
       ('st', J + 'ap/eo-site-state'), ('on', J + 'ap/eo-site-on'), ('x', J + 'ap/eo-site-x-ft'), ('y', J + 'ap/eo-site-y-ft'),
       ('margin', J + 'ap/eo-site-margin-ft'), ('sturn', J + 'ap/eo-site-sturn-rad'), ('reach', J + 'ap/eo-reach-nm'),
       ('n', SITE + 'n'), ('name', SITE + 'name'), ('nmargin', SITE + 'margin-nm'), ('path', SITE + 'path-nm'),
       ('gear', J + 'gear/gear-pos-norm'), ('w0', J + 'gear/unit[0]/WOW'), ('w1', J + 'gear/unit[1]/WOW'), ('w2', J + 'gear/unit[2]/WOW'),
       ('lat', '/position/latitude-deg'), ('lon', '/position/longitude-deg'),
       ('nac', J + 'ctl/nacelle-deg'), ('eoh', J + 'ap/eo-h-ft'), ('tau', J + 'ap/eo-tau-eff-s'), ('grate', J + 'ap/eo-gnd-rate-fps'),
       ('plane', J + 'ap/eo-site-plane'), ('selev', J + 'ap/eo-site-elev-ft'), ('slope', J + 'ap/eo-site-slope'), ('send', J + 'ap/eo-site-end-ft'),
       ('terr', '/position/ground-elev-ft'), ('rolling', J + 'ap/eo-rolling'), ('nz', J + 'accelerations/Nz'), ('nr', J + 'drive/nr-pct'),
       ('oc', J + 'ap/eo-open-on'), ('och', J + 'ap/eo-open-hdg-deg'), ('thcmd', J + 'ap/eo-theta-cmd-deg'), ('agl', '/position/altitude-agl-ft'), ('alt', '/position/altitude-ft'), ('gearft', J + 'ap/eo-gear-ft')]
rows, ev, stop = [], {}, threading.Event()


def logger():
    g = Fg(launch=False)
    paths = [p for _, p in LOG]
    t0, last, last_st, cut = time.time(), -10.0, None, False
    prev = None
    FAST = ('agl', 'hdot', 'w0', 'w1', 'w2', 'theta', 'phi', 'nz', 'kcas', 'nac', 'x', 'y', 'gs', 'nr')    # (near the ground, these alone: 40 gets a sample came to 2 Hz)
    while not stop.is_set():
        fast = prev is not None and isinstance(prev['agl'], float) and prev['agl'] < 80 and 'td' not in ev and ev.get('cut')
        keys = [(k, p_) for k, p_ in LOG if not fast or k in FAST]
        g.f.write(''.join('get %s\r\n' % p_ for _, p_ in keys)); g.f.flush()
        r = dict(prev) if fast else {}
        for k, _ in keys:
            v = g.f.readline().strip()
            try:
                r[k] = float(v)
            except ValueError:
                r[k] = {'true': 1.0, 'false': 0.0}.get(v, v)
        r['t'] = time.time() - t0
        rows.append(r)
        prev = r
        if ev.get('cut') and not cut:
            cut = True
        num = lambda k: r[k] if isinstance(r[k], float) else float('nan')
        # (the rotor speed from the cut to the ground: reported, not judged. docs/FLIGHT-LAWS.md, section 10)
        if cut and 'td' not in ev:
            nr = ev.setdefault('nr', dict(lo=999.0, hi=0.0))
            if isinstance(r.get('nr'), float):
                nr['lo'], nr['hi'] = min(nr['lo'], r['nr']), max(nr['hi'], r['nr'])
        if cut and 'gear' not in ev and num('eo') > 0.5 and num('hw') * 3.2808 < num('gearft') and num('gear') < 0.99:
            g.put('/controls/gear/gear-down', 1); ev['gear'] = r['t']
            print('GEAR down at %.0f ft' % (num('hw') * 3.2808), flush=True)
        if r['st'] != last_st:
            print('%6.1f s site state %s: %s, %s sites, x %.0f y %.0f, margin %.0f ft (Nasal %+.2f NM), path %s NM, %.0f ft' % (
                r['t'], r['st'], r['name'], r['n'], num('x'), num('y'), num('margin'), num('nmargin'), r['path'], num('hw') * 3.2808), flush=True)
            last_st = r['st']
            print('        plane %s, elevation %s ft, slope %s, strip end %s ft; ground here %.0f ft' % (r['plane'], r['selev'], r['slope'], r['send'], num('terr')), flush=True)
        if cut and 'flare' not in ev and num('ph') > 2.5:
            ev['flare'] = dict(r)
            print('FLARE at x %+.0f ft, y %+.0f ft, bank %.1f, %.0f KCAS, %s; %.0f ft (eo %.0f), sink %.1f ft/s, pitch %.1f, nacelles %.1f' % (
                num('x'), num('y'), num('phi'), num('kcas'), r['name'], num('hw') * 3.2808, num('eoh'), -num('hdot'), num('theta'), num('nac')), flush=True)
        if cut and 'td' not in ev and num('w0') + num('w1') + num('w2') > 0.5:
            ev['td'] = dict(r)
            print('TOUCHDOWN sink %.1f ft/s, %.0f kt GS, %.0f KCAS, pitch %.1f, nacelles %.1f, x %+.0f y %+.0f, rotor %.0f-%.0f %%' % (
                -num('hdot'), num('gs'), num('kcas'), num('theta'), num('nac'), num('x'), num('y'), ev['nr']['lo'], ev['nr']['hi']), flush=True)
        if cut and 'td' not in ev and num('agl') < 4 and num('w0') + num('w1') + num('w2') < 0.5:
            ev['td'] = dict(r); ev['impact'] = dict(r)
            print('IMPACT (no wheel on the ground) sink %.1f ft/s, %.0f kt GS, pitch %.1f, bank %.1f, ground %.0f ft, gear %.2f, rotor %.0f-%.0f %%' % (
                -num('hdot'), num('gs'), num('theta'), num('phi'), num('terr'), num('gear'), ev['nr']['lo'], ev['nr']['hi']), flush=True)
            break
        if 'td' in ev and 'brakes' not in ev and r['t'] - ev['td']['t'] > 2:
            g.put('/controls/gear/brake-left', 1); g.put('/controls/gear/brake-right', 1); ev['brakes'] = 1
        if 'td' in ev and num('agl') < -20:
            print('IN THE WATER x %+.0f y %+.0f, strip end %.0f ft past the point, %.0f KCAS' % (num('x'), num('y'), num('send'), num('kcas')), flush=True)
            break
        if 'td' in ev and num('gs') < 2 and r['t'] - ev['td']['t'] > 5:
            print('STOPPED x %+.0f y %+.0f' % (num('x'), num('y')), flush=True)
            break
        if r['t'] - last > 5:
            last = r['t']
            print('%6.1f s ph %s st %s %5.0f ft AGL (%5.0f MSL, ground %5.0f) %5.1f KCAS hdg-bank %5.1f margin %6.0f reach %.2f %s' % (
                r['t'], r['ph'], r['st'], num('agl'), num('alt'), num('terr'), num('kcas'), num('phi'), num('margin'), num('reach'), r['name']), flush=True)
        time.sleep(0.05)
    g.sock.close()



def connect():
    for _ in range(200):
        try:
            return Fg(launch=False)
        except OSError:
            time.sleep(2)
    sys.exit('no FlightGear on %d' % PORT)



import random
seed = int(os.environ['SEED']) if os.environ.get('SEED') else random.randrange(100000)    # (sys.argv is capture.py's, set above)
rnd = random.Random(seed)
LOWI = (47.26023, 11.34397)                  # between the runway ends (apt.dat: 08 47.25883 11.33091, 26 47.26162 11.35702), 1906 ft
brg, dist = rnd.uniform(0, 360), 0.0          # (over the airport itself)
LAT = LOWI[0] + dist * math.cos(math.radians(brg)) / 60
LON = LOWI[1] + dist * math.sin(math.radians(brg)) / 60 / math.cos(math.radians(LOWI[0]))
HDG, KCAS, ALT = round(rnd.uniform(0, 360)), round(rnd.uniform(160, 220)), 6000.0
CUT_S = 3                                    # (cut over the field, the modes just engaged)
# (HOVER=<ft above the runway>: a trimmed hover over the field's centre, on HOVER_HDG, HVR CPLD holding it HOVER_S s before the cut)
HOVER = float(os.environ.get('HOVER', 0))
if HOVER:
    HDG, KCAS, ALT, CUT_S = int(os.environ.get('HOVER_HDG', 260)), 0, 1906.0 + HOVER, float(os.environ.get('HOVER_S', 20))
# (BENCH_SEED=<n>: the start of the bench sweep's flight n, nbatch3.py's field_case, so that a flight here lines up with that
# flight there; WEIGHT=<lb>: loaded as the bench loads it, deadstick.weigh, then SETTLE_S s on the holds before the cut, since
# the start's trim is the default loading's)
WEIGHT, BSEED = float(os.environ.get('WEIGHT', 0)), os.environ.get('BENCH_SEED')
if BSEED:
    seed = int(BSEED)
    r_ = random.Random(1000 + seed)
    brg, dist = r_.uniform(0, 360), r_.uniform(0, 1.5)
    LAT = LOWI[0] + dist * math.cos(math.radians(brg)) / 60
    LON = LOWI[1] + dist * math.sin(math.radians(brg)) / 60 / math.cos(math.radians(LOWI[0]))
    HDG, KCAS = round(r_.uniform(0, 360)), round(r_.uniform(160, 215))
SETTLE_S = float(os.environ.get('SETTLE_S', 15)) if WEIGHT else 0.0
# (a reset flies on for about 25 s before the cut, the script's waits: started that far back along the heading, so that the
# engines quit over the field, not 1.25 NM on from it at the foot of the Nordkette)
LEAD_S = 25 + SETTLE_S
_nm = KCAS * 1.2 / 3600 * LEAD_S             # (the true airspeed at 6000 ft, about 1.2 times the calibrated)
RLAT = LAT - _nm * math.cos(math.radians(HDG)) / 60
RLON = LON - _nm * math.sin(math.radians(HDG)) / 60 / math.cos(math.radians(LOWI[0]))
print('seed %d: %.2f NM from LOWI on %03.0f deg, heading %03d, %d KCAS, %d ft MSL; both engines cut after %d s' % (seed, dist, brg, HDG, KCAS, ALT, CUT_S), flush=True)


def load(g, W):
    """deadstick.weigh in the running FlightGear: the cargo, below none of it the tanks' reference fill scaled"""
    base = U.W_EMPTY + 3 * U.W_CREW
    fuel = U.FUEL_LB_GAL * sum(t[1] * r for t, r in zip(U.TANKS, U.TANKS_REF))
    g.put(J + 'inertia/pointmass-weight-lbs[%d]' % U.CARGO_PM, max(0.0, W - base - fuel))
    k = min(1.0, (W - base) / fuel)
    for i, (t, r) in enumerate(zip(U.TANKS, U.TANKS_REF)):
        g.put('/consumables/fuel/tank[%d]/level-gal_us' % i, t[1] * r * k)


class FgLowi(Fg):
    def start(s):
        args = [C.FG, '--fg-root=' + C.FG_ROOT, '--fg-aircraft=' + C.ROOT, '--aircraft=mv22b', '--lat=%.6f' % LAT, '--lon=%.6f' % LON,
                '--heading=%d' % HDG, '--altitude=%d' % ALT, '--vc=%d' % KCAS, '--wind=0@0', '--disable-real-weather-fetch', '--timeofday=noon', '--telnet=%d' % PORT,
                '--compositor=Compositor/HDR/hdr', '--ignore-autosave', '--geometry=%dx%d' % (C.W, C.H),
                '--prop:/sim/paths/screenshot-dir=' + C.SHOTS]
        s.proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print('FlightGear launched (pid %d): waiting for the click' % s.proc.pid, flush=True)
        for _ in range(300):
            try:
                s.sock = socket.create_connection(('localhost', PORT), timeout=5); return
            except OSError:
                time.sleep(2)
        sys.exit('FlightGear did not open its telnet port in 10 minutes (left running)')


def to_start(fg):
    """the start, by a reset: RLAT/RLON, so that the engines quit over the field after the reset's waits"""
    for k, v in (('latitude-deg', RLAT), ('longitude-deg', RLON), ('altitude-ft', ALT), ('heading-deg', HDG),
                 ('airspeed-kt', KCAS), ('onground', 'false'), ('airport-id', ''), ('runway', '')):
        fg.put('/sim/presets/' + k, v)
    # (still air, as the offline bench: real weather off, no wind; unfrozen)
    fg.put('/environment/realwx/enabled', 0)
    for w in ('/environment/config/presets/wind-speed-kt', '/environment/wind-speed-kt'):
        fg.put(w, 0)
    fg.put('/sim/freeze/master', 0)
    fg.f.write('run reset\r\n'); fg.f.flush()
    print('reset in the running FlightGear', flush=True)
    time.sleep(5); fg.sock.close(); time.sleep(10)
    return connect()


try:
    fg = to_start(Fg(launch=False))
except OSError:
    # (a fresh launch flies on while FlightGear loads: the first flight so started began 1.1 NM WNW of its start, heading
    # into the Nordkette, and no version of the finder found a site from there. So the start by a reset here too)
    fg = FgLowi(launch=True)
    fg.wait_ready()
    fg = to_start(fg)
fg.wait_ready()
fg.put('/sim/freeze/master', 0)
if WEIGHT:
    load(fg, WEIGHT)
if os.environ.get('TRACE'):                  # (the finder's inputs and outputs to a file: Nasal/eotrace.nas)
    fg.put('/sim/eotrace/file', 'eotrace_%d.txt' % seed)
    fg.put('/sim/eotrace/load', 1)
    print('recording', flush=True)
print('wind %s kt from %s' % (fg.get('/environment/wind-speed-kt'), fg.get('/environment/wind-from-heading-deg')), flush=True)
print('started: %.0f ft MSL, %.0f ft over the ground, %.0f KCAS, nacelles %.0f, %.0f lb' % (fg.get('/position/altitude-ft'), fg.get('/position/altitude-agl-ft'),
      fg.get(J + 'velocities/vc-kts'), fg.get(J + 'ctl/nacelle-deg'), fg.get(J + 'inertia/weight-lbs')), flush=True)
fg.put('/controls/gear/gear-down', 0)
hdg = round(fg.get('/orientation/heading-magnetic-deg'))
fg.put(ST + 'heading-bug-deg', hdg); fg.put(ST + 'target-altitude-ft', round(fg.get('/position/altitude-ft'))); fg.put(ST + 'target-speed-kt', KCAS)
if HOVER:
    fg.put(AF + 'cpld', 1); fg.put(J + 'hh/engage-cmd', 1)
    print('HVR CPLD: hover hold on %s' % fg.get(J + 'hh/on'), flush=True)
else:
    for m in ('hdg-hold', 'alt-hold', 'spd-hold', 'cpld'):
        fg.put(AF + m, 1)
th = threading.Thread(target=logger, daemon=True)
th.start()
time.sleep(CUT_S + SETTLE_S)
print('CUT at %.0f ft MSL, %.0f KCAS, climb %.1f ft/s, pitch %.1f, %.0f lb' % (fg.get('/position/altitude-ft'), fg.get(J + 'velocities/vc-kts'),
      fg.get(J + 'velocities/h-dot-fps'), fg.get(J + 'attitude/theta-deg'), fg.get(J + 'inertia/weight-lbs')), flush=True)
fg.put('/controls/engines/engine[0]/cutoff', 1); fg.put('/controls/engines/engine[1]/cutoff', 1)
ev['cut'] = True
fg.sock.close()
th.join(timeout=900)
stop.set()
json.dump(dict(seed=seed, weight=WEIGHT, bench=bool(BSEED), start=dict(lat=LAT, lon=LON, hdg=HDG, kcas=KCAS, alt=ALT, cut_s=CUT_S), rows=rows,
               ev={k: v for k, v in ev.items() if isinstance(v, dict)}),
          open(os.path.join(S, 'fg_b%d_%d.json' % (seed, WEIGHT) if BSEED else 'fg_lowi_%d%s.json' % (seed, '_%d' % WEIGHT if WEIGHT else '')), 'w'))
print('done; FlightGear left running', flush=True)
