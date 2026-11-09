import re, sys
def rd(fn):
    d = {}
    for l in open(fn):
        m = re.match(r'\s*(\d+)\s+\d', l)
        if not m: continue
        s = int(m.group(1))
        r = dict(left='left the terrain' in l, intact=': INTACT' in l)
        a = re.search(r'down ([+-]\d+) ft along', l); p = re.search(r'stop\s+([+-]?\d+) past', l)
        k = re.search(r': (?:INTACT|not intact)\s+([\d.]+) ft/s', l); w = re.search(r'past the end; (\S+)', l)
        r.update(along=int(a.group(1)) if a else None, past=int(p.group(1)) if p else None, sink=float(k.group(1)) if k else None, site=w.group(1) if w else None)
        d[s] = r
    return d
def stats(d):
    v = [r for r in d.values() if not r['left']]
    tgt = [r for r in v if r['along'] is not None]
    lng = [(s, r['past']) for s, r in d.items() if r['past'] is not None and r['past'] > 2000]
    sht = [(s, r['along']) for s, r in d.items() if r['along'] is not None and r['along'] < 0]
    on = [r for r in tgt if (r['past'] is None or r['past'] <= 2000) and r['along'] >= 0]
    strip = sum(1 for r in v if r['intact'] and (r['past'] is None or r['past'] <= 0))
    return 'on strip %d, intact %d/%d, on target %d/%d, long %s, short %s, left %s' % (strip, sum(r['intact'] for r in v), len(v), len(on), len(tgt), lng, sht, [s for s, r in d.items() if r['left']])
a = rd(sys.argv[1]); print(sys.argv[1].split('/')[-1], stats(a))
if len(sys.argv) > 2:
    b = rd(sys.argv[2]); print(sys.argv[2].split('/')[-1], stats(b))
    for s in sorted(set(a) | set(b)):
        x, y = a.get(s), b.get(s)
        if x and y and (x['intact'] != y['intact'] or x['left'] != y['left'] or abs((x['along'] or 0) - (y['along'] or 0)) > 3000):
            print('  %2d  %s  ->  %s' % (s, {k: x[k] for k in ('intact', 'left', 'sink', 'along', 'past', 'site')}, {k: y[k] for k in ('intact', 'left', 'sink', 'along', 'past', 'site')}))
