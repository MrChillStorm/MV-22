"""ENG OUT's glide turning steadily at a bank: the height per 180 deg of heading, by the weight and the bank (deadland.py
site_turn's measure at banks other than the circle's 44): turn_bank.py"""
import sys, io, contextlib, json, os, tempfile
sys.path.insert(0, os.path.expanduser('~/git/V-22/fdm'))
from multiprocessing import Pool
import deadland as DL
OUT = (os.environ.get('EOBENCH_OUT') or os.path.join(tempfile.gettempdir(), 'eobench'))      # (EOBENCH_OUT: where the results go)
os.makedirs(OUT, exist_ok=True)
BANKS = (15, 20, 25, 30, 35, 40, 44, 50, 55)
def run(args):
    W, bank = args
    with contextlib.redirect_stdout(io.StringIO()):
        f = DL.site_start(W)
    f['ap/eo-bank-deg'] = bank
    dt = f['simulation/dt']
    turned, last, h0, t0, v = 0.0, None, None, None, []
    for i in range(int(56 / dt)):
        t = i * dt
        if abs(t - 1.0) < dt / 2:
            for e in (0, 1):
                f['/controls/engines/engine[%d]/cutoff' % e] = 1
        if t > 14:
            f['/instrumentation/mv22b/afcs/hdg-hold'] = 1
            f['/autopilot/settings/heading-bug-deg'] = (f['attitude/psi-deg'] + 90 - f['/environment/magnetic-variation-deg']) % 360
        if t > 25:
            if h0 is None:
                h0, t0 = f['/position/gear-agl-m'] / 0.3048, t
            turned += abs((f['attitude/psi-deg'] - last + 540) % 360 - 180)
            v.append((f['velocities/vc-kts'], abs(f['attitude/phi-deg'])))
        last = f['attitude/psi-deg']
        f.run()
    dh = h0 - f['/position/gear-agl-m'] / 0.3048
    return dict(W=W, bank=bank, ft180=dh / turned * 180, rate=turned / (t - t0), kcas=sum(x[0] for x in v) / len(v), phi=sum(x[1] for x in v) / len(v),
                sink=dh / (t - t0))
if __name__ == '__main__':
    jobs = [(W, b) for W in DL.SITE_W for b in BANKS]
    with Pool(8) as p:
        res = p.map(run, jobs)
    print('weight  bank  flown   KCAS  deg/s  sink ft/s  ft per 180 deg')
    for r in res:
        print('%6.0f  %4.0f  %5.1f  %5.1f  %5.2f  %6.1f  %7.0f' % (r['W'], r['bank'], r['phi'], r['kcas'], r['rate'], r['sink'], r['ft180']))
    json.dump(res, open(os.path.join(OUT, 'turn_bank.json'), 'w'), indent=1)
