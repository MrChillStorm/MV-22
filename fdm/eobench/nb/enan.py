import json, glob, math, re
import numpy as np
G = 32.174
fl = []
for p in sorted(glob.glob('en_*.json'), key=lambda x: int(re.findall(r'\d+', x)[0])):
    fl.append(json.load(open(p)))
def f(r, k):
    v = r.get(k)
    return float('nan') if v is None else float(v)
segs = []     # dE, air way, turn, brake ratio, ld, turn cost, gear, kcas, bank
mt = []
for F in fl:
    R = [r for r in F['rows'] if f(r, 'ap/eo-phase') >= 1]
    td = F['td']
    good = bool(td) and td.get('sink', 99) < 12 and float(td.get('clear', 0)) > 1 and float(td.get('gear', 0)) > .99
    # one-second segments in the glide
    g = [r for r in R if 1 <= f(r, 'ap/eo-phase') < 2.5 and f(r, 'ctl/nacelle-deg') < 2]
    for a, b in zip(g[0::5], g[5::5]):
        dt = b['t'] - a['t']
        if dt <= 0 or dt > 1.5: continue
        va, vb = f(a, 'velocities/vt-fps'), f(b, 'velocities/vt-fps')
        dE = (f(b, 'position/h-sl-ft') + vb * vb / (2 * G)) - (f(a, 'position/h-sl-ft') + va * va / (2 * G))
        way = (va + vb) / 2 * dt
        dp = (f(b, 'attitude/psi-deg') - f(a, 'attitude/psi-deg') + 180) % 360 - 180
        segs.append((dE, way, abs(math.radians(dp)), f(b, 'ap/eo-brake-ratio'), f(b, 'ap/eo-ld-up'), f(b, 'ap/eo-site-turn-ft-rad'), f(b, 'gear/gear-pos-norm'), f(b, 'velocities/vc-kts'), abs(f(b, 'attitude/phi-deg')), f(b, 'ap/eo-brake-nom-ratio'), f(b, 'ap/eo-site-margin-ft'), f(b, 'ctl/flaperon-deg')))
    # the margin through the flight
    site, start = None, None
    steps = []
    prev = None
    for r in R:
        on = f(r, 'ap/eo-site-on') > .5
        if not on or f(r, 'ap/eo-site-state') < .5:
            prev = None; continue
        m = f(r, 'ap/eo-site-margin-ft')
        if prev is not None:
            steps.append((r['t'], m - prev[0], r['site'] != prev[1], f(r, 'ap/eo-site-state')))
        prev = (m, r['site'])
    mt.append(dict(seed=F['seed'], good=good, steps=steps, first=next((f(r, 'ap/eo-site-margin-ft') for r in R if f(r, 'ap/eo-site-state') > .5), None), last=next((f(r, 'ap/eo-site-margin-ft') for r in reversed(R) if f(r, 'ap/eo-site-state') > .5), None), td=td, n=len(steps), changes=len([l for l in F['log'] if ' for ' in l or 'chosen' in l])))
S = np.array(segs)
print('%d one-second segments of glide, nacelles forward, gear up unless said' % len(S))
def fit(mask, name):
    if mask.sum() < 30:
        print('  %-44s %d segments: too few' % (name, mask.sum())); return
    A = np.c_[S[mask, 1], S[mask, 2]]
    c = np.linalg.lstsq(A, -S[mask, 0], rcond=None)[0]
    print('  %-44s %4d seg: L/D %.2f, %4.0f ft per rad turned | plan: ld %.2f x ratio %.2f = %.2f, %4.0f ft per rad' % (name, mask.sum(), 1 / c[0], c[1], np.nanmean(S[mask, 4]), np.nanmean(S[mask, 3]), np.nanmean(S[mask, 4] * S[mask, 3]), np.nanmean(S[mask, 5] / S[mask, 4])))
up = S[:, 6] < .05; fast = S[:, 7] > 185
fit(up & fast, 'all, gear up, 185+ KCAS')
fit(up & fast & (S[:, 3] > .97), 'brake clean (ratio > .97)')
fit(up & fast & (S[:, 3] <= .97) & (S[:, 3] > .85), 'brake part out (.85-.97)')
fit(up & fast & (S[:, 3] <= .85), 'brake well out')
fit(up & ~fast, 'gear up, under 185 KCAS (turn first)')
fit(~up, 'gear moving or down')
st = (S[:, 8] < 5) & up & fast
print('  straight, wings within 5 deg: L/D %.2f  (%d seg)' % (S[st, 1].sum() / -S[st, 0].sum(), st.sum()))
tn = (S[:, 8] > 35) & up & fast
print('  turning, 35+ deg: energy lost %.0f ft per s, against %.0f straight' % (-S[tn, 0].mean(), -S[st, 0].mean()))
neg = S[:, 10] < 0
print('  brake ratio while the margin is negative: mean %.3f, clean (> .97) %.0f %% of the time; nominal %.3f; flaperons mean %.1f deg' % (np.nanmean(S[neg, 3]), 100 * np.mean(S[neg, 3] > .97), np.nanmean(S[neg, 9]), np.nanmean(S[neg, 11])))
print()
print('margin through the flight (ft), by outcome:')
for gd in (True, False):
    q = [m for m in mt if m['good'] == gd]
    drift = [sum(s[1] for s in m['steps'] if not s[2] and abs(s[1]) < 3000) for m in q]
    jump_same = [sum(s[1] for s in m['steps'] if not s[2] and abs(s[1]) >= 3000) for m in q]
    jump_site = [sum(s[1] for s in m['steps'] if s[2]) for m in q]
    nj = [len([s for s in m['steps'] if abs(s[1]) >= 3000]) for m in q]
    print('  %-9s %2d flights: first margin median %6.0f; drift on a site %+6.0f; steps on the same site %+6.0f; steps at changes %+6.0f; steps over 3000 ft median %d; changes median %d' % ('landed' if gd else 'failed', len(q), np.median([m['first'] for m in q if m['first'] is not None]), np.median(drift), np.median(jump_same), np.median(jump_site), np.median(nj), np.median([m['changes'] for m in q])))
print()
print('failed flights:')
for m in mt:
    if m['good']: continue
    td = m['td'] or {}
    print('  %2d first margin %7.0f last %7.0f  changes %3d, big steps %2d; down %s ft/s %s KCAS slope %s, site %s' % (m['seed'], m['first'] or 0, m['last'] or 0, m['changes'], len([s for s in m['steps'] if abs(s[1]) >= 3000]), td.get('sink') and round(td['sink'], 1), td.get('kcas') and round(td['kcas']), td.get('slope') and round(100 * td['slope']), str(td.get('chosen'))[:18]))
