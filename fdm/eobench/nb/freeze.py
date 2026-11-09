"""a recorded moment held still: the finder's search run out, then every line it has, and why"""
import sys, os
sys.path.insert(0, '.')
import nasbench as NB, eobench as B
from vm import VM
path, at_s = sys.argv[1], float(sys.argv[2])
ticks, cur, base = [], None, {}
for l in open(path, errors='replace'):
    if l[:2] == 'T ':
        cur = {'t': float(l.split()[1]), 'n': {}}; ticks.append(cur)
    elif l[:2] == 'N ':
        a = l.rstrip('\n').split(' ')
        if len(a) == 3:
            try:
                (cur['n'] if cur is not None else base)[a[1]] = float(a[2])
            except ValueError:
                pass
t0 = ticks[0]['t']
latest = dict(base)
for t in ticks:
    latest.update(t['n'])
    if t['t'] - t0 >= at_s:
        break
hold = dict(latest); now = {'t': t['t']}
print('held at %.1f s after ENG OUT: lat %.5f lon %.5f, ground %.0f, wheels %.0f ft, reach %.2f NM, track from speeds' % (t['t'] - t0, hold['/position/latitude-deg'], hold['/position/longitude-deg'], hold['/position/ground-elev-ft'], hold.get('/fdm/jsbsim/hh/h-wheels-ft', 0), hold['/fdm/jsbsim/ap/eo-reach-nm']))
tree = NB.eo_tree(); dem = B.Dem(); store = {}
def get(p):
    if p.startswith(NB.DATA):
        return tree[p[len(NB.DATA):]][0]
    if p == '/sim/time/elapsed-sec':
        return now['t']
    if p == '/instrumentation/mv22b/afcs/eng-out':
        return 1.0
    if p.endswith('ap/eo-site-state') or p.endswith('ap/eo-phase'):
        return 0.0
    return hold.get(p)
msgs = []
v = VM(get, lambda p, x: store.__setitem__(p, x), lambda a, b: dem.geodinfo(a, b), lambda m: msgs.append(m))
class D:
    AIRPORTS = None; tree = NB.eo_tree()
p_ = 'data_fz_%d.nas' % os.getpid()
open(p_, 'w').write(NB.NasalFinder.nasal_data(D()))
v.load(p_, 'prelude_'); os.remove(p_)
v.load(NB.NasalFinder.SCRIPT, 'eosite')
open('fz.nas', 'w').write('''
var why = func(c, gs) {
    var n = path_pts(c, gs.r0);
    if (!n) return "no path";
    var St = PS[n - 1]; var At = PA[n - 1];
    var ce = contains(c, "elev") ? c.elev : 0;
    var Lf = c.plan[6]; Lf = Lf > 1 ? Lf : 1;
    var stretch = c.margin < 0 and c.margin + c.plan[8] >= 0;
    var within = c.margin >= 0 or stretch;
    var ld = stretch ? gs.ld / S.rn : gs.ld; var hpr = stretch ? gs.hpr * S.rn : gs.hpr;
    reads = 1e9;
    for (var q = 0; q < n; q += 1) {
        var e = elev_at(PN[q], PE[q]);
        if (e == nil) return sprintf("ground unknown at point %d of %d", q, n);
        var s = PS[q]; var a = PA[q];
        var hp = gs.h0 - s / ld - a * hpr;
        if (within) { var h2 = ce + gs.fl + (St - s) / ld + (At - a) * hpr; if (h2 < hp) hp = h2; }
        var w = (St - s) / Lf;
        if (hp - e < gs.fl * (w < 1 ? w : 1)) return sprintf("point %d of %d: plan height %.0f, ground %.0f at %.5f %.5f", q, n, hp, e, PN[q], PE[q]);
    }
    return "clear";
}
var dump = func {
    weigh(); read_state();
    var gs = glide_state(); GS = gs;
    reads = 1e9;
    var rows = []; var tiers = [0, 0, 0, 0, 0]; var clr = [0, 0, 0, 0, 0];
    var dryn = 0; var wet = 0;
    foreach (var c; lines) {
        if (c.kind != "RWY" and !contains(c, "dry")) c.dry = dry_strip(c);
        if (c.kind != "RWY" and !c.dry) { wet += 1; continue; }
        c.opts = energy_plans(c); c.plan = c.opts[0]; c.margin = c.plan[0]; c.path = c.plan[1];
        var t = tier(c); tiers[t] += 1;
        var ok = 0; var pl0 = nil;
        foreach (var pl; c.opts) { c.plan = pl; c.margin = pl[0]; if (path_clear(c, gs)) { ok = 1; pl0 = pl; break; } }
        if (pl0 == nil) { c.plan = c.opts[0]; c.margin = c.plan[0]; } else c.plan = pl0;
        c.margin = c.plan[0]; t = tier(c);
        if (ok) clr[t] += 1;
        append(rows, [t * 1e7 + c.margin, c, ok]);
    }
    rows = sort(rows, func(a, b) { return b[0] - a[0]; });
    var s = sprintf("%d lines, %d wet; by tier 0-4: %d %d %d %d %d; clear: %d %d %d %d %d; floor %.0f, h0 %.0f, reach %.0f, rn %.2f, sampling %d, tries left %d\\n", size(lines), wet, tiers[0], tiers[1], tiers[2], tiers[3], tiers[4], clr[0], clr[1], clr[2], clr[3], clr[4], floor_, gs.h0, S.reach0, S.rn, sampling, size(oq.k) - oq.h + size(rq.k) - rq.h);
    var n = 0; var nr = 0;
    foreach (var r; rows) {
        var c = r[1];
        var isr = c.kind == "RWY";
        if (isr) nr += 1;
        if ((n >= 12 and !isr) or (isr and nr > 8)) continue;
        n += 1;
        s ~= sprintf("  %-20s crs %3d elev %5.0f at %.4f %.4f tier %d %s margin %7.0f path %5.2f NM %-34s | %s\\n", name_of(c), c.crs, contains(c, "elev") ? c.elev : 0, c.lat, c.lon, tier(c), r[2] ? "CLEAR" : "block", c.margin, c.path / 6076.12, kname(c.plan), r[2] ? "" : why(c, gs));
    }
    print(s);
    return 0;
}
''')
v.load('fz.nas', 'eosite')
v.call('eosite', 'load')
for i in range(420):
    now['t'] += 0.2
    v.call('eosite', 'update')
    if store.get('/fdm/jsbsim/ap/eo-site-new') == 99: pass
msgs.clear()
v.call('eosite', 'dump')
print('\n'.join(msgs))
