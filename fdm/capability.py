#!/usr/bin/env python3
"""ENG OUT's capability by the gross weight, from what the model does: the schedule the guidance (genv22.py) and the finder
(Nasal/eosite.nas, through the flight model's properties) both read, so that neither holds a number of its own from one loading.

From deadstick.py --captable (data/eo_capability_v22.json: the zero-power glide by weight, gear, flaperons, airspeed and nacelles)
and deadland.py capturns and capflare (data/eo_capability_turns_v22.json, _flare_: the turns and flares as flown):

    flap_max_deg, after_kt
               the flare, from deadland.py capflare at a field's height: the most flaperons, and with them the lowest speed, from which every
               flare up to VG_KT + 10 is clean (the descent, the tips, the pitch inside their limits). The speed is the one held after the
               nacelle brake. There is a cliff: heavy, 10 KCAS too fast with 40 deg, their moment takes the nose past what the stick holds, the
               float balloons and the speed runs out (54 000 lb, 200 KCAS: 23 ft/s at 133 KCAS, for 9 ft/s at 167 with 30 deg). No clean
               column, the flaperons cleanest at the glide's own speed. flap_max_deg itself: flap_index(), from the table and the stick at the wheels
    nb_kt, nb_deg, nb_ld
               the nacelle brake: of the speeds and nacelles flown in the turns, the steepest straight glide, and its ratio (the
               flaperons out, the gear up). A weight's trim the solver missed, from its neighbours in weight

    xw_weight_lb, xw_limit_kt
               the crosswind full pedal takes out at the touchdown's speed (deadstick.py --xwind)

    zone_weight_lb, zone_short_ft, zone_past_ft, zone_side_ft, zone_rise
               the ground a strip has to keep clear, by where the intact landings came down (eobench/scatter.py), and the most a slope
               may rise there: what the gear's limit leaves over the ninetieth percentile's sink at the touchdown's speed

    entry_kcas, entry_ft, entry_loss_ft, entry_way_ft
               what getting into the glide costs past the energy it has and the way it covers, by the speed the engines stop at (deadland.py
               capentry, a hover to 170 KCAS, the nacelles the conversion's there): the energy height lost, less the way covered at the
               glide's ratio. From a hover 1300-2500 ft of energy simply goes, into the rotors' drag and the dive

    python3 capability.py     writes data/eo_capability_schedule_v22.json
"""
import json, math, os
import numpy as np
import v22data as U

HERE = os.path.dirname(os.path.abspath(__file__))
VG_KT = 200.0                   # ap/eo-vg-kt
FLARE_SINK = 10.0               # ft/s at the wheels: the gear's 12 less the scatter the flares show
FLARE_TIPS = 1.5                # ft
FLARE_PITCH = 7.0               # deg: the tail's 7.3 on the wheels
FLARE_STICK = 0.95              # of forward stick at the wheels, the nacelles up for the tips: the rest for the pitch's swing in the flare, 0.2-0.3 in rms of 4.8
EO_TIP_FT = 2.0                 # ft: the tips over the ground on the wheels that ENG OUT's nacelles aim for (the bench's limit 1, less what the pitch's swing takes)
NB_LD_AXIS = (1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0)
NB_STICK = 0.9                  # of forward stick: what the nacelle brake leaves for the pull-out
GEAR_FPS = 12.0                 # ft/s: the gear's limit, against the ground
# the nacelles' way down the Glide task's ratios: the flaperons from their nominal, GL_FLAP_NOM, in by GL_FLAP_OUT deg of nacelle, for the forward stick they'd take from them
GL_NAC = tuple(float(n) for n in range(0, 50, 5))
GL_FLAP_NOM, GL_FLAP_OUT = 15.0, 20.0
GL_BAND = 0.03                  # of the best glide: the speed may give this up, for no more than the reach's own scatter
ALT0 = 9000.0                   # ft: where the entries started


def data(name):
    return json.load(open(os.path.join(HERE, 'data', name)))


def glide(cap, W, V, nac):
    """the glide's ratio at W, flaperons 40, the gear up; a trim missing there from the weights either side, None if outside them"""
    i, j, k = cap['kcas'].index(V), cap['nacelle_deg'].index(nac), cap['flap_deg'].index(40.0)
    w = [(x, cap['ld'][n][0][k][i][j]) for n, x in enumerate(cap['weight_lb']) if cap['ld'][n][0][k][i][j] is not None]
    if len(w) < 2 or not w[0][0] <= W <= w[-1][0]:
        return None
    return float(np.interp(W, [x[0] for x in w], [x[1] for x in w]))


def flare(fl, a):
    """the flaperons and the speed at weight index a"""
    clean = lambda r: r and r[0] <= FLARE_SINK and r[2] >= FLARE_TIPS and r[4] <= FLARE_PITCH
    kts = fl['kcas']
    for j, flap in sorted(enumerate(fl['flap_deg']), key=lambda x: -x[1]):
        v = None
        for i in range(len(kts) - 1, -1, -1):
            if kts[i] > VG_KT + 10 or not clean(fl['flare'][a][i][j]):
                if kts[i] <= VG_KT + 10:
                    break
                continue
            v = kts[i]
        if v is not None and v <= VG_KT:
            return flap, v
    over = lambda r: max(r[0] / FLARE_SINK, FLARE_TIPS / max(r[2], 0.1), r[4] / FLARE_PITCH)
    i = kts.index(VG_KT)
    return min((over(fl['flare'][a][i][j]), fl['flap_deg'][j]) for j in range(len(fl['flap_deg'])))[1], VG_KT


def tas(kcas, h):
    """ft/s, the standard atmosphere"""
    return kcas * 1.68781 / math.sqrt((1 - 6.8756e-6 * h) ** 4.2559)


def entry(ent, ld):
    """by weight and starting speed: ft of energy height, its height and its way on to the fall's end; ld: the glide's ratio by weight.
    Two nacelle angles at a speed averaged; a start measured to no glide from the weights either side"""
    kts = sorted({e[0] for e in ent['start_kcas_nacelles']})
    tables = [[], [], []]
    for a, W in enumerate(ent['weight_lb']):
        rows = [[], [], []]
        for v in kts:
            x = []
            for e, r in zip(ent['start_kcas_nacelles'], ent['entry'][a]):
                if e[0] == v and r:
                    lost = r[0] + (tas(v, ALT0) ** 2 - tas(VG_KT - 5, ALT0 - r[0]) ** 2) / (2 * 32.174)
                    x.append((lost - r[1] / ld[a], r[0], r[1]))
            for k in range(3):
                rows[k].append(sum(q[k] for q in x) / len(x) if x else None)
        for k in range(3):
            tables[k].append(rows[k])
    Ws = np.array(ent['weight_lb'], dtype=float)
    out = []
    for t in tables:
        t = np.array(t, dtype=float)
        for j in range(t.shape[1]):
            ok = ~np.isnan(t[:, j])
            t[:, j] = np.interp(Ws, Ws[ok], t[ok, j])
        out.append([[max(0, round(x)) for x in r] + [0] for r in t])
    return kts + [VG_KT], out[0], out[1], out[2]


def vg_index():
    """The glide's one speed by the weight, indexed from what the model does, no speed chosen by hand. The top of the glide's curve is flat: GL_BAND of its best takes in 20-30 KCAS. So not
    the peak: the lowest speed within GL_BAND of the best glide (data/glide_ld_v22.json) that the flare lands cleanly from (data/eo_capability_flare_v22.json: under FLARE_SINK, the tips and
    pitch inside their limits, at any flaperons). Every knot over that buys no way, and costs a wider turn and a longer float: 209 KCAS at 54 000 lb, the peak, landed at 8.8 deg of pitch
    where 200, 2 % down the curve, lands clean. Where none is clean, the flare's least bad. Between the weights the two tables give, interpolated. Between the flare table's rows too, at the flaperons the
    stick allows there (flare_ok): taken to the next row up, 38 722 lb flew 180 KCAS with its edge at 176.7, touched at 145 against 141, the nose at 3.9 deg against 5.7, and 12-13 ft/s where 1 % of rising
    ground adds 3-4 (at 175-178 KCAS the three light weights' 12-20 ft/s flights 10 to 3, intact 103 to 108 of 120)"""
    gl, fl = data('glide_ld_v22.json'), data('eo_capability_flare_v22.json')
    ks = gl['kcas']
    def low(row):
        c = np.polyfit(ks, row, 4)
        kk = np.linspace(ks[0], ks[-1], 401)
        i = int(np.argmax(row))
        fine = np.interp(kk, ks, row) if not 0 < i < len(ks) - 1 else np.polyval(np.polyfit(ks[i - 1:i + 2], row[i - 1:i + 2], 2), kk)
        return float(kk[int(np.argmax(fine >= (1 - GL_BAND) * fine.max()))])
    clean = lambda r: r and r[0] <= FLARE_SINK and r[2] >= FLARE_TIPS and r[4] <= FLARE_PITCH
    miss = lambda r: max(0.0, r[0] - FLARE_SINK) + max(0.0, FLARE_TIPS - r[2]) * 2 + max(0.0, r[4] - FLARE_PITCH) * 2 if r else 1e9
    W, V = [], []
    for a, w in enumerate(fl['weight_lb']):
        vb = low(gl['ld_gear_up'][gl['weight_lb'].index(w)])
        ok = [k for i, k in enumerate(fl['kcas']) if any(clean(fl['flare'][a][i][j]) for j in range(len(fl['flap_deg'])))]
        if ok:
            up = [k for k in ok if k >= vb - 2]
            v = min(up) if up else max(ok)
            v = next((float(x) for x in np.arange(vb, v, 0.1) if flare_ok(a, float(x))), v)
        else:
            v = float(min((min(miss(fl['flare'][a][i][j]) for j in range(len(fl['flap_deg']))), k) for i, k in enumerate(fl['kcas']))[1])
        W.append(float(w))
        V.append(round(float(v), 1))
    return tuple(W), tuple(float(x) for x in np.maximum.accumulate(V))


def eo_tip_deg(ft):
    """the nacelles and the pitch together at which the low blade tip clears the ground by ft on the wheels, deg (deadstick.clearance's geometry)"""
    lo, hi = 0.0, math.pi / 2
    for _ in range(40):
        a = 0.5 * (lo + hi)
        if (U.WL_PIV - U.MAIN_GEAR[2] + U.DZ_HUB * math.sin(a)) / 12 - U.R * math.cos(a) < ft:
            lo = a
        else:
            hi = a
    return math.degrees(hi)


def flare_stick(W, flap, V, nac):
    """The forward stick the trims ask at the wheels (data/eo_brake_grid_v22.json, the gear down), of the whole: past the end of a row, where the trims stop because the stick has run out, along its slope"""
    g = data('eo_brake_grid_v22.json')
    wi = g['weight_lb'].index(W)
    def along(row, x, ax):
        pts = [(a, b) for a, b in zip(ax, row) if b is not None]
        if len(pts) < 2:
            return None
        xs, ys = [q[0] for q in pts], [q[1] for q in pts]
        return float(np.interp(x, xs, ys)) if x <= xs[-1] else ys[-1] + (ys[-1] - ys[-2]) / (xs[-1] - xs[-2]) * (x - xs[-1])
    per = [along([along(g['stick'][wi][1][fi][ki], nac, g['nacelle_deg']) for ki in range(len(g['kcas']))], V, g['kcas']) for fi in range(len(g['flap_deg']))]
    return along(per, flap, g['flap_deg'])


def flare_ok(a, V):
    """Clean from a speed between the flare table's rows, at weight index a: the two rows either side read across, at flaperons the stick has at the wheels"""
    fl = data('eo_capability_flare_v22.json')
    ks = fl['kcas']
    i = max(0, min(len(ks) - 2, int(np.searchsorted(ks, V, side='right')) - 1))
    t = (V - ks[i]) / (ks[i + 1] - ks[i])
    tip = eo_tip_deg(EO_TIP_FT)
    for j, f in enumerate(fl['flap_deg']):
        p, q = fl['flare'][a][i][j], fl['flare'][a][i + 1][j]
        if p and q:
            r = [x + t * (y - x) for x, y in zip(p, q)]
            if r[0] <= FLARE_SINK and r[2] >= FLARE_TIPS and r[4] <= FLARE_PITCH and (flare_stick(fl['weight_lb'][a], f, r[1], tip - r[4]) or 0) <= FLARE_STICK:
                return True
    return False


EO_VG_W, EO_VG_KT = vg_index()


def flap_index(a, V):
    """The flaperons for the flare from the indexed speed at weight index a: the pair is one choice. At the table's speed nearest V, the most flaperons that keep a step in hand and leave the stick
    for the tips. More lift, a lower touchdown, so the most; but each takes stick from the nacelles, and past a weight's cliff they don't arrive. A step in hand: the next flaperons up still inside the tips
    and the pitch, 54 000 lb from 200 KCAS reading clean at 30 deg and off it at 35 (flown, 30 deg 25 of 40 intact, 25 deg 33). The stick: what the trims ask at the wheels, with the nacelles the tips want at
    the table's pitch, under FLARE_STICK; the table's straight flares have it to the last inch, the flown ones don't (60 500 lb 0.90 at 15 deg, 24; 47 500 lb 1.07 at 40 deg, 6). None inside, the least bad"""
    fl = data('eo_capability_flare_v22.json')
    i = min(range(len(fl['kcas'])), key=lambda n: abs(fl['kcas'][n] - V))
    rows = [(fl['flare'][a][i][j], fl['flap_deg'][j]) for j in range(len(fl['flap_deg'])) if fl['flare'][a][i][j]]
    inside = lambda r: r[2] >= FLARE_TIPS and r[4] <= FLARE_PITCH
    tip = eo_tip_deg(EO_TIP_FT)
    fit = [(r, f) for n, (r, f) in enumerate(rows) if n + 1 == len(rows) or inside(rows[n + 1][0])] or rows
    fit = [(r, f) for r, f in fit if (flare_stick(fl['weight_lb'][a], f, r[1], tip - r[4]) or 0) <= FLARE_STICK] or fit[:1]
    good = [f for r, f in fit if inside(r)]
    return max(good) if good else min((max(0.0, r[0] - FLARE_SINK) + 2 * max(0.0, r[4] - FLARE_PITCH) + 2 * max(0.0, FLARE_TIPS - r[2]), f) for r, f in fit)[1]


def glide_alloc(grid):
    """The Glide task's drag, at each weight's glide speed: the nacelles for a glide ratio, the pitch that speed takes at each, from the brake grid (corridor free).
    Measured cells only: the row ends where the grid's does or the forward stick passes NB_STICK"""
    ks = grid['kcas']
    def at(tab, wi, gi, fi, V, nac):
        ax = grid['nacelle_deg']
        rows = []
        for k in range(len(ks)):
            pts = [(ax[n], tab[wi][gi][fi][k][n]) for n in range(len(ax)) if tab[wi][gi][fi][k][n] is not None]
            if pts and pts[0][0] <= nac <= pts[-1][0]:
                rows.append((ks[k], float(np.interp(nac, [q[0] for q in pts], [q[1] for q in pts]))))
        # (only between speeds the grid has: read from the row under it, 45 deg passed at 210 KCAS and 60 500 lb, where the stick runs out at 200, and the nose went to -60 deg as they closed)
        if not rows or not rows[0][0] <= V <= rows[-1][0]:
            return None
        return float(np.interp(V, [q[0] for q in rows], [q[1] for q in rows]))
    out = dict(gl_weight_lb=list(grid['weight_lb']), gl_nac=list(GL_NAC), gl_nac_up=[], gl_nac_down=[], gl_theta=[], gl_stick=[], gl_ld=[], gl_ld_min=[], gl_nac_max=[])
    f0, f20 = grid['flap_deg'].index(0.0), grid['flap_deg'].index(20.0)
    for wi, W in enumerate(grid['weight_lb']):
        V = float(np.interp(W, EO_VG_W, EO_VG_KT))
        def cell(tab, gi, nac):
            a = at(tab, wi, gi, f0, V, nac)
            f = GL_FLAP_NOM * max(0.0, 1 - nac / GL_FLAP_OUT) / 20.0
            if a is None or f <= 0:
                return a
            b = at(tab, wi, gi, f20, V, nac)
            return None if b is None else a + f * (b - a)
        ok = [n for n in GL_NAC if cell(grid['ld'], 0, n) is not None and cell(grid['stick'], 0, n) < NB_STICK]
        top = ok[-1]
        out['gl_nac_max'].append(top)
        th = [cell(grid['theta_deg'], 0, n) if n <= top else cell(grid['theta_deg'], 0, top) for n in GL_NAC]
        out['gl_theta'].append([round(x - th[0], 2) for x in th])
        st_ = [cell(grid['stick'], 0, n) if n <= top else cell(grid['stick'], 0, top) for n in GL_NAC]
        out['gl_stick'].append([round(x - st_[0], 3) for x in st_])
        ld_ = [cell(grid['ld'], 0, n) if n <= top else cell(grid['ld'], 0, top) for n in GL_NAC]
        out['gl_ld'].append([round(x, 2) for x in ld_])
        for gi, k in ((0, 'gl_nac_up'), (1, 'gl_nac_down')):
            pts = sorted((cell(grid['ld'], gi, n), n) for n in ok if cell(grid['ld'], gi, n) is not None)
            out[k].append([round(float(np.interp(x, [q[0] for q in pts], [q[1] for q in pts])), 1) for x in NB_LD_AXIS])
            if gi == 1:
                out['gl_ld_min'].append(round(pts[0][0], 2))
    return out


def schedule():
    cap, tur, fl, grid = data('eo_capability_v22.json'), data('eo_capability_turns_v22.json'), data('eo_capability_flare_v22.json'), data('eo_brake_grid_v22.json')
    out = dict(weight_lb=cap['weight_lb'], flap_max_deg=[], after_kt=[], nb_kt=[], nb_deg=[], nb_ld=[], nbg_kt=[], nbg_deg=[], nbg_ld=[], nb_ld_axis=list(NB_LD_AXIS), nb_nac_by_ld=[])
    low = 0.0
    for a, W in enumerate(cap['weight_lb']):
        flap, v = flare(fl, a)
        low = max(low, v)
        out['after_kt'].append(low)
        out['flap_max_deg'].append(flap_index(a, EO_VG_KT[a]))
        # (the brake from the grid, the corridor free: the flaperons in, for the forward stick they'd take from the nacelles, at the speed the flare wants or
        # the grid's next under it, so there is none to get back after it: the steepest there with stick to spare)
        wi, ki = grid['weight_lb'].index(W), max(i for i, k in enumerate(grid['kcas']) if k <= low)
        fi = grid['flap_deg'].index(0.0)
        best = min((grid['ld'][wi][0][fi][ki][n], grid['nacelle_deg'][n]) for n in range(len(grid['nacelle_deg']))
                   if grid['ld'][wi][0][fi][ki][n] is not None and grid['stick'][wi][0][fi][ki][n] < NB_STICK)
        # (for the straight-line law, ap/eo-nb-line, under test. The default stays the turns' steepest, at 150 KCAS: at five weights it was no worse)
        for k, x in zip(('nbg_ld', 'nbg_kt', 'nbg_deg'), (best[0], grid['kcas'][ki], best[1])):
            out[k].append(round(x, 2))
        ld, V, nac = min((glide(cap, W, V, nac), V, nac) for V in tur['kcas'] for nac in tur['nacelle_deg']
                         if V in cap['kcas'] and nac in cap['nacelle_deg'] and glide(cap, W, V, nac) is not None)
        for k, x in zip(('nb_ld', 'nb_kt', 'nb_deg'), (round(ld, 2), V, nac)):
            out[k].append(x)
        # (and its way there: the nacelles for a glide ratio, at that speed, the steepest stopping it)
        pts = sorted((grid['ld'][wi][0][fi][ki][n], grid['nacelle_deg'][n]) for n in range(len(grid['nacelle_deg'])) if grid['ld'][wi][0][fi][ki][n] is not None and grid['nacelle_deg'][n] <= best[1])
        out['nb_nac_by_ld'].append([round(float(np.interp(x, [q[0] for q in pts], [q[1] for q in pts])), 1) for x in NB_LD_AXIS])
    out.update(glide_alloc(grid))
    out['vg_weight_lb'], out['vg_kt'] = list(EO_VG_W), list(EO_VG_KT)
    gl = json.load(open(os.path.join(HERE, 'data', 'glide_ld_v22.json')))
    ld = [float(np.interp(VG_KT, gl['kcas'], np.array(gl['ld_gear_up'])[int(np.argmin(abs(np.array(gl['weight_lb']) - W)))])) for W in cap['weight_lb']]
    out['entry_kcas'], out['entry_ft'], out['entry_loss_ft'], out['entry_way_ft'] = entry(data('eo_capability_entry_v22.json'), ld)
    xw = data('eo_crosswind_v22.json')
    out['xw_weight_lb'], out['xw_limit_kt'], out['xw_beta_deg'] = xw['weight_lb'], xw['limit_kt'], xw['beta_full_deg']
    sc, site = data('eo_scatter_v22.json'), data('eo_site_v22.json')
    out['zone_weight_lb'], out['zone_short_ft'], out['zone_past_ft'], out['zone_side_ft'] = sc['weight_lb'], sc['short_ft'], sc['past_ft'], sc['side_ft']
    td = lambda W: float(np.interp(W, site['weight_lb'], site['touchdown_kt'])) * 1.6878
    out['zone_rise'] = [round((GEAR_FPS - s_) / td(W), 4) for W, s_ in zip(sc['weight_lb'], sc['sink_p90_fps'])]
    return out


if __name__ == '__main__':
    s = schedule()
    json.dump(s, open(os.path.join(HERE, 'data', 'eo_capability_schedule_v22.json'), 'w'), indent=1)
    for i, W in enumerate(s['weight_lb']):
        print('%6d lb: flare to %2.0f deg of flaperons; after the brake %3.0f KCAS; brake %3.0f KCAS, nacelles %2.0f, glide %.2f' % (
            W, s['flap_max_deg'][i], s['after_kt'][i], s['nb_kt'][i], s['nb_deg'][i], s['nb_ld'][i]))
    print('entry past the energy, by KCAS', s['entry_kcas'])
    for W, r in zip(s['weight_lb'], s['entry_ft']):
        print('%6d lb:' % W, r)
