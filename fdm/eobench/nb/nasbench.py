"""the real eosite.nas in the bench: B.Finder's place taken by the embedded Nasal VM"""
import sys, os, json, re
import xml.etree.ElementTree as ET
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.expanduser('~/git/V-22/fdm/eobench'))
import eobench as B
from vm import VM
JSB = '/fdm/jsbsim/'
WIND = [float(x) for x in os.environ.get('BENCH_WIND', '0 0').split()]     # kt, from deg
DATA = '/instrumentation/mv22b/displays/data/eo-site/'
AIRCRAFT = os.path.expanduser('~/git/V-22/MV-22B/')
MAP = {'/position/latitude-deg': 'position/lat-geod-deg', '/position/longitude-deg': 'position/long-gc-deg',
       '/velocities/speed-north-fps': 'velocities/v-north-fps', '/velocities/speed-east-fps': 'velocities/v-east-fps',
       '/position/ground-elev-ft': 'position/terrain-elevation-asl-ft'}


def eo_tree():
    t = ET.parse(AIRCRAFT + 'Systems/mv22b-displays.xml').getroot().find('eo-site')
    d = {}
    for e in t:
        d.setdefault(e.tag, []).append(float(e.text))
    return d


class NasalFinder:
    SCRIPT = os.environ.get('EO_SCRIPT') or AIRCRAFT + 'Nasal/eosite.nas'
    AIRPORTS = None
    STAGES = False

    def __init__(s, f, dem, t):
        s.f, s.dem, s.t = f, dem, t
        s.store, s.vm_s, s.errs = {}, [], 0
        s.tree = eo_tree()
        s.vm = VM(s.get, s.setp, s.geo, s.log)
        path = os.path.join(HERE, 'data_%d.nas' % os.getpid())
        with open(path, 'w') as o:
            o.write(s.nasal_data())
        s.vm.load(path, 'prelude_')
        os.remove(path)
        s.vm.load(s.SCRIPT, 'eosite')
        s.vm.load(os.path.join(HERE, 'helper.nas'), 'eosite')
        if s.STAGES:
            s.vm.load(os.path.join(HERE, 'stages.nas'), 'eosite')
        s.vm.call('eosite', 'load')

    def nasal_data(s):
        q = lambda xs: '[' + ','.join('{v:%r,getValue:func me.v}' % x for x in xs) + ']'
        tree = ','.join('"%s":%s' % (k, q(v)) for k, v in s.tree.items())
        apts = B.LOWI if s.AIRPORTS is None else s.AIRPORTS
        rw = ','.join('"%s":{lat:%r,lon:%r,heading:%r,length:%r,threshold:%r,surface:%d}' % (k, r['lat'], r['lon'], r['heading'], r['length'], r['threshold'], r['surface']) for k, r in apts.items())
        return ('globals.props = {globals: {getNode: func(p) {var t = {%s}; return {getChildren: func(n) { return contains(t, n) ? t[n] : []; }}; }}};\n'
                'globals.findAirportsWithinRange = func(r) { return [{id: "%s"}]; };\n'
                'globals.airportinfo = func(id) { return {runways: {%s}}; };\n' % (tree, 'LOWI' if s.AIRPORTS is None else 'NONE', rw)) if apts else (
                'globals.props = {globals: {getNode: func(p) {var t = {%s}; return {getChildren: func(n) { return contains(t, n) ? t[n] : []; }}; }}};\n'
                'globals.findAirportsWithinRange = func(r) { return []; };\nglobals.airportinfo = func(id) nil;\n' % tree)

    def get(s, p):
        if p.startswith(DATA):
            return s.tree[p[len(DATA):]][0]
        if p == '/sim/time/elapsed-sec':
            return s.t()
        if p == '/environment/wind-speed-kt':
            return WIND[0]
        if p == '/environment/wind-from-heading-deg':
            return WIND[1]
        if p.startswith(JSB):
            p = p[len(JSB):]
        try:
            return s.f[MAP.get(p, p)]
        except Exception:
            return s.store.get(p)

    def setp(s, p, x):
        if p.startswith(JSB):
            s.f[p[len(JSB):]] = x
        else:
            s.store[p] = x

    def geo(s, lat, lon):
        return s.dem.geodinfo(lat, lon)

    def log(s, m):
        if m.startswith('NASAL'):
            s.errs += 1
        print('%7.1f  site: %s' % (s.t(), m.replace('MV-22B ENG OUT site: ', '')), flush=True)

    def update(s):
        if WIND[0]:
            a = WIND[1] * 3.14159265 / 180; w = WIND[0] * 1.6878
            import math
            s.f['atmosphere/wind-north-fps'] = -w * math.cos(a); s.f['atmosphere/wind-east-fps'] = -w * math.sin(a)
        _, t = s.vm.call('eosite', 'update')
        s.vm_s.append(t)

    @property
    def chosen(s):
        return s.vm.calls('eosite', 'bench_key')

    @property
    def open(s):
        return s.f['ap/eo-open-hdg-deg'] if s.f['ap/eo-open-on'] > 0.5 else None

    def find(s, k):
        r = s.vm.calls('eosite', 'bench_site')
        if r is None:
            return None
        a = r.split('|')
        # (the strip's end moved on by the open ground the finder found past it: past that, nothing is known)
        e = [float(x) for x in a[4:8]]
        sp = float(a[8]) if len(a) > 8 else 0.0
        e[2], e[3] = B.coord_move(e[2], e[3], float(a[3]), sp * B.FT2M)
        return dict(kind=a[0], lat=float(a[1]), lon=float(a[2]), crs=float(a[3]), ends=e)


B.Finder = NasalFinder
B.TICK_S = float(re.search(r'var TICK_S = ([0-9.]+)', open(NasalFinder.SCRIPT).read()).group(1))

if os.environ.get('BENCH_TICK'):
    B.TICK_S = float(os.environ['BENCH_TICK'])

if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if '=' not in a]
    if len(args) >= 5:
        lat, lon, hdg, kcas, alt = (float(x) for x in args[:5])
    else:
        lat, lon, hdg, kcas, alt = 47.26023, 11.34397, 139.0, 205.0, 6015.0
    B.run(lat, lon, hdg, kcas, alt, {}, quiet=True)
