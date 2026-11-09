import sys, json, math, os, tempfile
sys.path.insert(0, '.')
import nasbench as NB, eobench as B
s = int(sys.argv[1])
SP = (os.environ.get('EOBENCH_OUT') or os.path.join(tempfile.gettempdir(), 'eobench'))      # (EOBENCH_OUT: fg_lowi.py's logs)
d = json.load(open(SP + '/fg_lowi_%d.json' % s)); r = d['rows']
n = lambda x: x if isinstance(x, float) else float('nan')
x = r[0]
cut = next(q for q in r if n(q['eo']) > 0.5)['t'] - x['t']
out = B.run(x['lat'], x['lon'], d['start']['hdg'], x['kcas'], x['alt'], W=38722.0, cut_s=cut, quiet=True, every=1.0, T=110.0)
json.dump(dict(track=out['track'], cut=cut, td=out.get('td'), left=out.get('left')), open('rp4_%d.json' % s, 'w'), default=str)
