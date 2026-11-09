import re, sys
def rd(f):
    t = dict(n=0, left=0, ok=0, strip=0, soft=0, hard=0, slope=0, water=0)
    bad = []
    for l in open(f):
        if not re.match(r'\s*\d+\s+\d', l): continue
        t['n'] += 1
        if 'left the terrain' in l: t['left'] += 1; continue
        m = re.search(r': (INTACT|not intact)\s+([\d.]+) ft/s .*?on (\S+)\s+slope\s+(\S+)', l)
        if not m: continue
        ok = m.group(1) == 'INTACT'; s = float(m.group(2))
        sl = abs(float(m.group(4))) if m.group(4) != 'n/a' else 0
        p = re.search(r'stop\s+([+-]?\d+) past', l)
        t['ok'] += ok; t['strip'] += ok and (p is None or int(p.group(1)) <= 0)
        t['hard'] += 12 <= s; hit = sl >= 10 or s >= 25
        t['slope'] += hit
        t['water'] += m.group(3) in ('Lake', 'Water', 'Stream', 'Canal', 'River')
        if hit: bad.append(int(l.split()[0]))
    return t, bad
for f in sys.argv[1:]:
    t, bad = rd(f); print('%-16s %s  slope hits %s' % (f, t, bad))
