"""the real eosite.nas fed a FlightGear flight's recorded inputs, tick by tick; its rankings against FlightGear's
    retrace.py TRACE fg|dem     fg: FlightGear's own terrain answers; dem: the bench's terrain"""
import sys, os, re, collections
sys.path.insert(0, '.')
import nasbench as NB, eobench as B
from vm import VM
path, mode = sys.argv[1], sys.argv[2]
ticks, geo, cur, base = [], {}, None, {}
rankF = []
for l in open(path, errors='replace'):
    k = l[:2]
    if k == 'T ':
        cur = {'t': float(l.split()[1]), 'n': {}}; ticks.append(cur)
    elif k == 'N ':
        a = l.rstrip('\n').split(' ')
        if len(a) == 3:
            try:
                (cur['n'] if cur is not None else base)[a[1]] = float(a[2])
            except ValueError:
                pass
    elif k == 'G ':
        a = l.split()
        if len(a) < 4 or (a[3] != 'nil' and len(a) < 6):
            continue
        key = (round(float(a[1]), 6), round(float(a[2]), 6))
        geo[key] = None if a[3] == 'nil' else (float(a[3]), {'names': [a[4]], 'solid': int(a[5])} if a[4] != '-' else None)
    elif k == 'R ':
        rankF.append({'h': l.split(), 'c': []})
    elif k == 'C ' and rankF:
        rankF[-1]['c'].append(l.split())
print('%s: %d ticks, %.0f s, %d terrain answers, %d rankings' % (os.path.basename(path), len(ticks), ticks[-1]['t'] - ticks[0]['t'], len(geo), len(rankF)))
dem = B.Dem()
latest = dict(base); latest['/instrumentation/mv22b/afcs/eng-out'] = 1.0
store, misses = {}, [0, 0]
OUT = path + '.replay_' + mode
tree = NB.eo_tree()
now = {'t': 0.0}
def get(p):
    if p.startswith(NB.DATA):
        return tree[p[len(NB.DATA):]][0]
    if p == '/sim/eotrace/path':
        return OUT
    if p == '/sim/time/elapsed-sec':
        return now['t']
    if p == '/instrumentation/mv22b/afcs/eng-out':
        return 1.0
    return latest.get(p)
TRF = open(OUT, 'w')
def setp(p, x):
    if p == '/bench/trace':
        TRF.write(x); return
    store[p] = x
def gd(lat, lon):
    if mode == 'dem':
        return dem.geodinfo(lat, lon)
    k = (round(lat, 6), round(lon, 6))
    misses[1] += 1
    if k in geo:
        return geo[k]
    misses[0] += 1
    return dem.geodinfo(lat, lon)
msgs = []
v = VM(get, setp, gd, lambda m: msgs.append(m))
class D:
    AIRPORTS = None; tree = NB.eo_tree()
p_ = 'data_rt_%d.nas' % os.getpid()
open(p_, 'w').write(NB.NasalFinder.nasal_data(D()) + 'globals.io = {open: func(p, m) { return p; }, write: func(f, s) { setprop("/bench/trace", s); }, flush: func(f) nil};\n')
v.load(p_, 'prelude_'); os.remove(p_)
v.load(NB.NasalFinder.SCRIPT, 'eosite')
try:
    v.load(os.path.expanduser('~/git/V-22/fdm/eobench/nb/eotrace.nas'), 'eosite')
except RuntimeError:
    print(msgs[-3:]); raise
v.call('eosite', 'load')
for t in ticks:
    latest.update(t['n']); now['t'] = t['t']
    v.call('eosite', 'update')
err = [m for m in msgs if 'NASAL' in m]
print('replay done; terrain answers not in the recording %d of %d; errors %d %s' % (misses[0], misses[1], len(err), err[:1]))
rankB = []
TRF.close()
for l in open(OUT, errors='replace'):
    if l[:2] == 'R ':
        rankB.append({'h': l.split(), 'c': []})
    elif l[:2] == 'C ' and rankB:
        rankB[-1]['c'].append(l.split())
print('rankings: FlightGear %d, replay %d' % (len(rankF), len(rankB)))
same = first = None; ident = 0; chosen_same = 0
key = lambda c: tuple(c[1:5])
for i, (a, b) in enumerate(zip(rankF, rankB)):
    ca, cb = a['h'][7], b['h'][7]
    sa, sb = [key(c) for c in a['c']], [key(c) for c in b['c']]
    chosen_same += ca == cb
    if sa == sb and ca == cb:
        ident += 1
    elif first is None:
        first = i
        print('first difference at ranking %d, t = %s (%.0f s after on): lines %s/%s, chosen %s / %s' % (i, a['h'][1], float(a['h'][1]) - ticks[0]['t'], a['h'][3], b['h'][3], ca, cb))
        print('   in FlightGear only: %s' % [' '.join(x) for x in sa if x not in sb][:3])
        print('   in the replay only: %s' % [' '.join(x) for x in sb if x not in sa][:3])
n = min(len(rankF), len(rankB))
print('identical rankings %d of %d; same site chosen %d of %d' % (ident, n, chosen_same, n))
md = [abs(float(x[10]) - float(y[10])) for a, b in zip(rankF, rankB) for x in a['c'] for y in b['c'] if key(x) == key(y)]
if md:
    md.sort(); print('margins of the same lines: median difference %.1f ft, 99%% %.1f, worst %.1f (%d pairs)' % (md[len(md)//2], md[int(len(md)*.99)], md[-1], len(md)))
