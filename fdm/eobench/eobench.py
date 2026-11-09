"""ENG OUT's landing site finder (MV-22B/Nasal/eosite.nas, ported line for line) flown offline with the flight model
over FlightGear's own terrain and land cover round LOWI (lowi_dem.txt: geodinfo every 75 m, dumped from FlightGear):
the FlightGear scenario's start, the engines cut, the finder ticking every 0.2 s as the Nasal timer does.

    eobench.py [lat lon hdg kcas alt_msl] [setting=value ...]     default: seed 47632's start (over LOWI, 139, 205 KCAS)"""
import sys, os, math, json, functools, heapq
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.expanduser('~/git/V-22/fdm'))
sys.path.insert(0, HERE)
import deadland as DL
from benchstart import start

D2R, R2D, FT2M, M2FT = math.pi / 180, 180 / math.pi, 0.3048, 1 / 0.3048
FT_DEG = 60 * 6076.12
CELL_M, PER_TICK, TRY_PER_TICK, TICK_S, RANK_TICKS, HEADINGS, MAX_FIELDS, SEARCH_NM = 150, 120, 16, 0.2, 5, 12, 24, 8
DOWN_MAX, RISE_MAX, FALL_K, SITE_NEAR_FT = 0.02, 0.005, 10, 6076.1
OPEN = dict(Airport='grass', AirportKeep='grass', Grass='grass', GrassCover='grass', Grassland='grass', DryLake='grass',
            grass_rwy='grass', dirt_rwy='grass', lakebed_taxiway='grass',
            MixedCrop='crop', IrrCrop='crop', DryCrop='crop', Crop='crop', CropGrass='crop', CropGrassCover='crop',
            MixedCropPastureCover='crop', DryCropPastureCover='crop', IrrCropPastureCover='crop')
KS = ((1, 1), (1, -1), (-1, -1), (-1, 1))       # the paths: (the circle's side, the first turn's way), +1 right
KIND_RANK = {'RWY': 0, 'pavement': 0, 'grass': 1, 'crop': 2, 'rough': 3}
# (an airfield's pavement, FlightGear's pa_ and pc_ materials: the runways' markings and the taxiways' and aprons'
# pa_tiedown, along LOWI's runway on both sides; left out, every line along or across the field met it and was
# refused, and the field was its two runway ends alone, its grass none)
PAVED = lambda n: n.startswith('pa_') or n.startswith('pc_')
# the shorter finals tried: deadland.py finals, in 15 kt from all sides at 38 730 and 54 000 lb, from 800 ft 16 of 16 intact,
# worst 8.7 ft/s; from 400 ft 15, worst 18.6, still rolling out at the flare
FINALS = (4000.0, 3000.0, 2000.0, 1200.0, 800.0)
TOP = 12          # the lines whose paths the terrain is checked under each ranking, the best by all else
# (no field open and level in reach: level ground under woods, scrub or natural cover, flown as a field, ranked after
# the fields; over LOWI the open course and the short fields came down on 6-117 % slopes instead)
ROUGH = {'MixedForestCover', 'DeciduousBroadCover', 'EvergreenBroadCover', 'EvergreenForest', 'DeciduousForest', 'MixedForest',
         'CropWoodCover', 'ShrubCover', 'Scrub', 'HerbTundraCover', 'HerbTundra', 'NaturalCrop', 'Heath', 'Sclerophyllous'}
HEAD_KT, HOLD_S = 5, 10
AF = '/instrumentation/mv22b/afcs/'


def nround(x):                       # Nasal's math.round: halves away from zero
    return int(math.copysign(math.floor(abs(x) + 0.5), x))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def m2pi(a):
    w = wrap(a)
    return w + 2 * math.pi if w < 0 else w


class Dem:
    """FlightGear's geodinfo every 75 m round LOWI: the elevation (bilinear) and the land cover (nearest)"""
    def __init__(s, path=os.path.join(HERE, 'lowi_dem.txt')):
        s.fine = False
        z = os.path.join(HERE, 'lowi_dem2.npz')
        if os.path.exists(z) and not os.environ.get('OLD_DEM'):
            # (every 30 m, the scenery all loaded, fg_dump2.py: lowi_dem.txt had a fifth of it empty and land cover at 75 m, and
            # the finder fed it chose FlightGear's site in 15 of 47 rankings)
            d = np.load(z)
            s.lat0, s.lon0, s.half, s.step = (float(x) for x in d['head'][:4]); s.n = int(d['head'][4])
            s.e, s.idx, s.names, s.solids = d['e'].astype(float), d['i'], [str(x) for x in d['names']], d['solid']
            s.e[s.idx < 0] = np.nan
            s.kl, s.kc, s.fine = 111320.0, math.cos(s.lat0 * D2R), True
            return
        L = open(path).read().split('\n')
        h = L[0].split()
        s.lat0, s.lon0, s.half, s.step, s.n = float(h[2]), float(h[4]), float(h[6]), float(h[8]), int(h[10])
        rows = [l.split() for l in L[1:] if l.strip()]
        s.e = np.array([[float(r[3 * c]) for c in range(s.n)] for r in rows])
        s.name = [[r[3 * c + 1] for c in range(s.n)] for r in rows]
        s.solid = [[int(r[3 * c + 2]) for c in range(s.n)] for r in rows]
        s.kl, s.kc = 111320.0, math.cos(s.lat0 * D2R)

    def rc(s, lat, lon):
        return ((lat - s.lat0) * s.kl + s.half) / s.step, ((lon - s.lon0) * s.kl * s.kc + s.half) / s.step

    def elev_m(s, lat, lon):
        if lat != lat or lon != lon:
            return None
        r, c = s.rc(lat, lon)
        r0, c0 = int(math.floor(r)), int(math.floor(c))
        if r0 < 0 or c0 < 0 or r0 + 1 >= s.n or c0 + 1 >= s.n:
            return None
        # (plain floats, the same sums in the same order: numpy's scalars took 5 % of a flight here)
        e = s.e.item
        q00, q01, q10, q11 = e(r0, c0), e(r0, c0 + 1), e(r0 + 1, c0), e(r0 + 1, c0 + 1)
        if q00 != q00 or q01 != q01 or q10 != q10 or q11 != q11:
            v = s.e[nround(r) if nround(r) < s.n else s.n - 1, nround(c) if nround(c) < s.n else s.n - 1]
            return None if np.isnan(v) else float(v)
        fr, fc = r - r0, c - c0
        return q00 * (1 - fr) * (1 - fc) + q01 * (1 - fr) * fc + q10 * fr * (1 - fc) + q11 * fr * fc

    def geodinfo(s, lat, lon):
        e = s.elev_m(lat, lon)
        if e is None:
            return None
        r, c = s.rc(lat, lon)
        r, c = min(s.n - 1, max(0, nround(r))), min(s.n - 1, max(0, nround(c)))
        if s.fine:
            i = int(s.idx[r, c])
            return e, (None if i < 0 or s.names[i] == '-' else {'names': [s.names[i]], 'solid': int(s.solids[i])})
        nm = s.name[r][c]
        return e, (None if nm == '-' else {'names': [nm], 'solid': s.solid[r][c]})


# LOWI's runways (apt.dat), as FlightGear's airportinfo gives them: the end, its true heading, length and displaced
# threshold (m), the surface (1 asphalt)
def _crs(a, b):
    la, lb, dl = a[0] * D2R, b[0] * D2R, (b[1] - a[1]) * D2R
    return math.degrees(math.atan2(math.sin(dl) * math.cos(lb), math.cos(la) * math.sin(lb) - math.sin(la) * math.cos(lb) * math.cos(dl))) % 360


E08, E26 = (47.25882780, 11.33091400), (47.26162380, 11.35702000)
LEN_M = math.hypot((E26[1] - E08[1]) * math.cos(E08[0] * D2R), E26[0] - E08[0]) * FT_DEG * FT2M
LOWI = {'08': dict(lat=E08[0], lon=E08[1], heading=_crs(E08, E26), length=LEN_M, threshold=103.02, surface=1),
        '26': dict(lat=E26[0], lon=E26[1], heading=_crs(E26, E08), length=LEN_M, threshold=0.0, surface=1)}


def coord_move(lat, lon, crs_deg, d_m):
    """geo.Coord.apply_course_distance, over these few km flat"""
    d = d_m * M2FT
    lat2 = lat + d * math.cos(crs_deg * D2R) / FT_DEG
    return lat2, lon + d * math.sin(crs_deg * D2R) / (FT_DEG * math.cos(lat * D2R))


class Finder:
    def __init__(s, f, dem, t):
        s.f, s.dem, s.t = f, dem, t
        d = json.load(open(os.path.expanduser('~/git/V-22/fdm/data/eo_site_v22.json')))
        s.W = d['weight_lb']
        s.TAB = {'h50': d['gate_to_50ft_ft'], 'td': d['gate_to_touchdown_ft'], 'roll-pavement': d['rollout_ft']['pavement'],
                 'roll-grass': d['rollout_ft']['grass'], 'roll-crop': d['rollout_ft']['crop'],
                 'err-lo': [e[0] for e in d['reach_err_ft']], 'err-hi': [e[1] for e in d['reach_err_ft']], 'td-kt': d['touchdown_kt']}
        s.FLARE = d['flare_err_ft']
        s.TB, s.TH = d['turn_bank_deg'], d['turn_ft_per_180_bank']     # (the turn's height per 180 deg by the bank, deadland.py turns)
        s.was, s.ticks, s.sampling, s._cache = 0, 0, 1, None
        s.reset()

    # ---- the properties
    def num(s, p, dflt=0.0):
        m = {'/position/latitude-deg': 'position/lat-geod-deg', '/position/longitude-deg': 'position/long-gc-deg',
             '/velocities/speed-north-fps': 'velocities/v-north-fps', '/velocities/speed-east-fps': 'velocities/v-east-fps',
             '/position/ground-elev-ft': 'position/terrain-elevation-asl-ft'}
        if p == '/sim/time/elapsed-sec':
            return s.t()
        if p.startswith('/environment/wind'):
            return 0.0
        try:
            return s.f[m.get(p, p)]
        except Exception:
            return dflt

    def setp(s, p, x):
        s.f[p] = x

    def log(s, msg):
        print('%7.1f  site: %s' % (s.t(), msg), flush=True)

    def at(s, t, w):
        return float(np.interp(w, s.W, s.TAB[t]))

    def needs(s, ground, head=0.0, slope=0.0):
        w = s.num('inertia/weight-lbs')
        vt = s.at('td-kt', w)
        ks = 1 + FALL_K * max(0, -slope)
        kf = max(0.2, 1 - head / ((200 + vt) / 2)) * ks
        kr = max(0.2, 1 - head / vt) ** 2 * ks
        return [s.FLARE[0] - max(0, s.at('err-hi', w)), s.FLARE[1] + max(0, -s.at('err-lo', w)), s.at('h50', w) * kf, s.at('td', w) * kf,
                s.at('roll-' + ('crop' if ground == 'rough' else ground), w) * kr]

    def head(s, crs):
        return 0.0

    # ---- the path and its margin
    def cnum(s, p, dflt=0.0):
        if s._cache is None:
            return s.num(p, dflt)
        if p not in s._cache:
            s._cache[p] = s.num(p, dflt)
        return s._cache[p]

    def hac_default(s):
        return s.cnum('ap/eo-hac-frac', 0.8) * s.cnum('ap/eo-bank-lim-deg', 44 / 0.8)

    def turn_cost(s, bank):
        # the turn's cost in the glide's way per radian at the bank, the weight's row of the table, as the guidance's
        w = s.cnum('inertia/weight-lbs')
        row = [float(np.interp(w, s.W, [r[k] for r in s.TH])) for k in range(len(s.TB))]
        return float(np.interp(bank, s.TB, row)) * s.cnum('ap/eo-ld-up', 7) / math.pi

    def plans(s, lat, lon, trk, c, hac=None, fin=0.0):
        num = s.cnum
        wN, wE = num('atmosphere/total-wind-north-fps'), num('atmosphere/total-wind-east-fps')
        va = max(100, num('velocities/vt-fps'))
        def air(t):
            al = wN * math.cos(t) + wE * math.sin(t); ac = wE * math.cos(t) - wN * math.sin(t)
            return va / max(30, al + math.sqrt(max(1, va * va - ac * ac)))
        # (the circle at its bank: the radius, the final's roll out and the cost from the guidance's own at its present)
        phc = num('ap/eo-site-hac-bank-deg', 44); hb = s.hac_default() if hac is None else hac
        rate = num('ap/eo-phi-rate-dps', 15) * D2R
        vgt = math.sqrt(max(1.0, num('ap/eo-site-r-ft', 4000) * 32.174 * math.tan(phc * D2R)))
        R = vgt * vgt / (32.174 * math.tan(hb * D2R))
        Lf = num('ap/eo-site-final-full-ft', 5000) + vgt * (hb - phc) * D2R / rate
        if fin > 0:                                  # (the minimum entry point's: the final cut, as sent)
            Lf = min(fin, Lf)
        kt, kt0 = s.turn_cost(hb), num('ap/eo-site-turn0-ft-rad', 4000)
        below = num('/position/ground-elev-ft')
        se = c['elev'] if 'elev' in c else s.dem.elev_m(c['lat'], c['lon'])
        if se is not None and 'elev' not in c:
            se *= M2FT
        # (at the brake's nominal ratio, as the guidance's margin: the glide flown with it part way out; retracted, the
        # clean glide's more, the slack)
        rn = num('ap/eo-brake-nom-ratio', 1)
        reach = (num('ap/eo-reach-nm') * 6076.12 + (0 if se is None else (below - se) * num('ap/eo-ld-up', 7))) * rn
        slack = reach * (1 / rn - 1)
        ph = hb * D2R
        vt = max(100, num('velocities/vt-fps'))
        lead = 32.174 * math.tan(ph) / vt * (ph / (2 * rate) + num('ap/roll-lag-s'))
        dN = (lat - c['lat']) * FT_DEG
        dE = (lon - c['lon']) * FT_DEG * math.cos(lat * D2R)
        cc = c['crs'] * D2R
        x = dE * math.sin(cc) + dN * math.cos(cc)
        y = dE * math.cos(cc) - dN * math.sin(cc)
        sg = -1 if y < 0 else 1
        u = x + Lf
        w = y - sg * R
        d = math.sqrt(u * u + w * w)
        lam = math.atan2(w, u)
        lamT = lam + sg * math.acos(min(1, R / max(d, 1)))
        thT = m2pi(sg * (-sg * math.pi / 2 - lamT))
        if thT > 2 * math.pi - lead:
            thT = 0
        L1 = math.sqrt(max(0, d * d - R * R))
        # (the leg a Dubins path, as the guidance's: the circle on either side, the first turn either way, a cross
        # path straight on first while its circles overlap)
        psi = wrap(trk * D2R - cc); R0 = num('ap/eo-site-r0-ft', 2000)
        def dub(s1, s2, xa, ya):
            c1x, c1y = xa - s1 * R0 * math.sin(psi), ya + s1 * R0 * math.cos(psi)
            dx, dy = -Lf - c1x, s2 * R - c1y
            rho = s2 * R - s1 * R0
            d2 = dx * dx + dy * dy
            if d2 <= rho * rho:
                return None
            L = math.sqrt(d2 - rho * rho)
            phi = math.atan2(dy, dx) - math.atan2(rho, max(L, 1))
            a1 = m2pi(s1 * (phi - psi)); a1 = 0 if a1 > 2 * math.pi - lead else a1
            a2 = m2pi(-s2 * phi); a2 = 0 if a2 > 2 * math.pi - lead else a2
            return (L * air(cc + phi) + kt0 * a1 + kt * a2, L, phi, a1, a2)
        out = []
        for k, (s2, s1) in enumerate(KS):
            d = dub(s1, s2, x, y); s0 = 0.0
            if s1 != s2:
                ddx, ddy = x - s1 * R0 * math.sin(psi) + Lf, y + s1 * R0 * math.cos(psi) - s2 * R
                ud = math.cos(psi) * ddx + math.sin(psi) * ddy
                q = ud * ud + (R + R0) ** 2 - (ddx * ddx + ddy * ddy)
                sq = -ud + math.sqrt(q) + 1 if q > 0 else 0
                if sq > 2:
                    d1 = dub(s1, s2, x + sq * math.cos(psi), y + sq * math.sin(psi))
                    if d1 is not None and (d is None or sq * air(trk * D2R) + d1[0] < d[0]):
                        d = (sq * air(trk * D2R) + d1[0],) + d1[1:]; s0 = sq
            if d is None:
                continue
            cost, L, phi, a1, a2 = d
            geo = (cost, s1, L, phi, a1, a2, x + s0 * math.cos(psi), y + s0 * math.sin(psi), psi, R0, s0, k)
            # (what the guidance can spend on it: the brake, full from the nominal, the reach's 1 - r_nom; the S-turns
            # on its leg, the most of room (1 / cos dS - 1) and their turns, the room what the guidance gives them; a
            # lap of the circle)
            vd = vgt * (ph / (2 * rate) + num('ap/roll-lag-s'))
            st = max([0.0] + [max(0.0, L - 2 * vd - R * ds) * (1 / math.cos(ds) - 1) + 2 * ds * kt0 for ds in (0.2, 0.4, 0.6, 0.8, 1.0, 1.2)
                              if L - 2 * vd - R * ds > 0])
            out.append([reach - (cost + Lf * air(cc)), s0 + R0 * a1 + L + R * a2 + Lf, s2, lamT, a2, R, Lf, geo, slack, hb, fin, reach * (1 - rn) + st, 2 * math.pi * kt])
        if out:
            return sorted(out, key=lambda o: -o[0])
        turn0 = abs(wrap(cc + lamT + sg * math.pi / 2 - trk * D2R))
        return [[reach - (L1 * air(cc + lamT + sg * math.pi / 2) + kt0 * turn0 + kt * thT + Lf * air(cc)), L1 + R * thT + Lf, sg, lamT, thT, R, Lf, None, slack, hb, fin, reach * (1 - rn), 2 * math.pi * kt]]

    def fits(s, pl, margin=None):
        # the surplus over the floor no more than the guidance can spend (a lap taking its share first)
        m = (pl[0] if margin is None else margin) - s.floor
        return (m - pl[12] if m > pl[12] else m) <= pl[11]

    def plan_tier(s, pl, margin=None):
        # 4: within reach at the brake's nominal by the floor, what is over it spendable; 3: clean by the floor, the brake
        # in (a stretch); 2: clean, under the floor; 1: high, more over the floor than can be spent, the landing long by
        # about what is left; 0: short. (Reaching the field before flying past it: a final cut from a loop to a straight-in
        # read 22 700 ft high 1.3 NM from LOWI, ranked over the fields as a runway, and ran 27 000 ft on into the town)
        m = pl[0] if margin is None else margin
        if m >= s.floor:
            return 4 if s.fits(pl, m) else 1
        return 3 if m + pl[8] >= s.floor else 2 if m + pl[8] >= 0 else 0

    def energy_plans(s, lat, lon, trk, c):
        # (the options for the energy, the best tier first. High, the circle wider: the shallower banks of the turn table,
        # the tightest that fits: the circle spends only where it turns, and wider than the way to the site its tangents
        # come and go: over LOWI at 15 deg, 15 600 ft across a field 2 NM out, the path flown lost its tangent, read 95 000
        # ft short, and the S-turns, their room the leg less the circle's, had none. Low, the Shuttle's minimum entry point:
        # the final cut, the longest of FINALS that fits, the flare no worse for it, deadland.py finals)
        base = s.plans(lat, lon, trk, c)
        extra = []
        t0 = s.plan_tier(base[0])
        if t0 == 1:
            h0 = s.hac_default()
            for b in sorted((b for b in s.TB if b < h0 - 0.5), reverse=True):
                pl = s.plans(lat, lon, trk, c, b)
                if pl[0][0] < s.floor:
                    break
                if s.plan_tier(pl[0]) == 4:
                    extra = pl
                    break
        elif t0 < 4:
            for Lj in FINALS:
                pl = s.plans(lat, lon, trk, c, None, Lj)
                if s.plan_tier(pl[0]) == 4:
                    extra = pl
                    break
                if s.plan_tier(pl[0]) > s.plan_tier((extra or base)[0]):
                    extra = pl
        return sorted(extra + base, key=lambda o: (-s.plan_tier(o), -o[0]))

    def plan(s, lat, lon, trk, c):
        return s.plans(lat, lon, trk, c)[0]

    # ---- the terrain under the path
    def elev_at(s, lat, lon):
        if s.origin is not None:
            c = s.grid.get(s.ckey((lon - s.origin[1]) * FT_DEG * math.cos(s.origin[0] * D2R) / (CELL_M * M2FT),
                                  (lat - s.origin[0]) * FT_DEG / (CELL_M * M2FT)))
            if c is not None:
                return c[1]
        e = s.dem.elev_m(lat, lon)
        return None if e is None else e * M2FT

    def glide_state(s):
        num = s.num
        vt = num('velocities/vt-fps')
        vg = num('ap/eo-vg-kt', 200) * 1.6878 / math.sqrt(max(0.2, num('atmosphere/sigma', 1)))
        fl = num('ap/eo-flare-ft', 250)
        ld = num('ap/eo-reach-nm') * 6076.12 / max(100, num('ap/eo-energy-ft') - fl)
        v_ = max(100, vt)
        rn = num('ap/eo-brake-nom-ratio', 1)        # (the glide flown with the brake at its nominal ratio)
        return dict(h0=num('/position/ground-elev-ft') + num('hh/h-wheels-ft') + min(0, (vt * vt - vg * vg) / (2 * 32.174)),
                    ld=max(1, ld * rn), hpr=num('ap/eo-site-turn-ft-rad', 4000) / max(1, num('ap/eo-ld-up', 7) * rn), fl=fl,
                    r0=v_ * v_ / (32.174 * math.tan(num('ap/eo-bank-deg', 55) * D2R)))

    STEP_FT = CELL_M * M2FT

    def path_pts(s, lat0, lon0, trk, c, r0, flown=None):
        sg, lamT, thT, R, Lf = c['plan'][2:7]
        if flown is not None:                     # (round its circle from here, or its final, as the guidance flies them)
            sg, R, Lf = s.num('ap/eo-site-s', sg), s.num('ap/eo-site-r-ft', R), s.num('ap/eo-site-final-ft', Lf)
            lamT = math.atan2(s.num('ap/eo-site-y-ft') - sg * R, s.num('ap/eo-site-x-ft') + Lf)
            thT = 0 if flown > 2.5 else s.num('ap/eo-site-arc-rad')
        cc = c['crs'] * D2R; k = math.cos(lat0 * D2R)
        Pn = (c['lat'] - lat0) * FT_DEG; Pe = (c['lon'] - lon0) * FT_DEG * k
        site = lambda x, y: [Pn + x * math.cos(cc) - y * math.sin(cc), Pe + x * math.sin(cc) + y * math.cos(cc)]
        out = []; st = {'sc': 0.0, 'ac': 0.0}
        add = lambda p: out.append([lat0 + p[0] / FT_DEG, lon0 + p[1] / (FT_DEG * k), st['sc'], st['ac']])
        def line(a, b):
            d = math.hypot(b[0] - a[0], b[1] - a[1])
            n = int(d / s.STEP_FT) + 1
            for i in range(1, n + 1):
                st['sc'] += d / n; add([a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n])
        dub = c['plan'][7] if flown is None and len(c['plan']) > 7 else None
        if dub is not None:                       # (the Dubins path: its turn, the tangent, the circle, the final)
            _, s1, L, phi, a1, a2, xa, ya, psi, R0, s0, _ = dub
            C1 = (xa - s1 * R0 * math.sin(psi), ya + s1 * R0 * math.cos(psi)); C2 = (-Lf, sg * R)
            q = [0.0, 0.0]
            if s0 > 0:
                line(q, site(xa, ya)); q = site(xa, ya)
            n = int(a1 * R0 / s.STEP_FT) + 1
            for i in range(1, n + 1):
                hh = psi + s1 * a1 * i / n
                st['ac'] += a1 / n; q = site(C1[0] + s1 * R0 * math.sin(hh), C1[1] - s1 * R0 * math.cos(hh)); add(q)
            line(q, site(C2[0] + sg * R * math.sin(phi), C2[1] - sg * R * math.cos(phi)))
            n = int(a2 * R / s.STEP_FT) + 1
            for i in range(1, n + 1):
                hh = phi + sg * a2 * i / n
                st['ac'] += a2 / n; add(site(C2[0] + sg * R * math.sin(hh), C2[1] - sg * R * math.cos(hh)))
            line(site(-Lf, 0), site(0, 0))
            return out
        h = trk * D2R; hl = cc + lamT + sg * math.pi / 2
        dt = 0.0 if flown is not None else wrap(hl - h); sgn = -1 if dt < 0 else 1
        cen = [sgn * r0 * -math.sin(h), sgn * r0 * math.cos(h)]
        n = int(abs(dt) * r0 / s.STEP_FT) + 1
        p = [0.0, 0.0]
        for i in range(1, n + 1):
            hh = h + dt * i / n
            p = [cen[0] + sgn * r0 * math.sin(hh), cen[1] - sgn * r0 * math.cos(hh)]
            st['ac'] += abs(dt) / n; add(p)
        tp = site(-Lf + R * math.cos(lamT), sg * R + R * math.sin(lamT))
        if flown is None or flown <= 2.5:
            line(p, tp)
        if flown is not None and flown > 2.5:
            p = [0.0, 0.0]
        n = int(thT * R / s.STEP_FT) + 1
        for i in range(1, n + 1):
            l = lamT + sg * thT * i / n
            st['ac'] += thT / n; add(site(-Lf + R * math.cos(l), sg * R + R * math.sin(l)))
        line(p if flown is not None and flown > 2.5 else site(-Lf, 0), site(0, 0))
        return out

    def path_clear(s, lat0, lon0, trk, c, gs, flown=None):
        pts = s.path_pts(lat0, lon0, trk, c, gs['r0'], flown)
        if not pts:
            return 1
        S, A = pts[-1][2], pts[-1][3]
        ce = c['elev'] if 'elev' in c else s.elev_at(c['lat'], c['lon'])
        if ce is None:
            return 0
        Lf = c['plan'][6]
        for q in pts:
            e = s.elev_at(q[0], q[1])
            if e is None:                         # (ground not known: the path not known clear)
                return 0
            hf = gs['h0'] - q[2] / gs['ld'] - q[3] * gs['hpr']
            hp = min(hf, ce + gs['fl'] + (S - q[2]) / gs['ld'] + (A - q[3]) * gs['hpr']) if c['margin'] >= 0 else hf
            if hp - e < gs['fl'] * min(1, (S - q[2]) / max(1, Lf)):
                return 0 if c['margin'] >= 0 else int(e <= ce + gs['fl'])
        return 1

    def open_course(s, lat0, lon0, trk, gs):
        best, bd = None, -1; k = math.cos(lat0 * D2R); h = trk * D2R
        for j in range(24):
            hc = j * 15 * D2R
            dt = wrap(hc - h); sgn = -1 if dt < 0 else 1
            cen = [sgn * gs['r0'] * -math.sin(h), sgn * gs['r0'] * math.cos(h)]
            # (the turn onto it, then straight; the height kept over the ground along the glide, summed, till it comes
            # within the flare's height: the longest glide alone took it over the terrace south of LOWI, 700-900 ft up)
            n = int(abs(dt) * gs['r0'] / s.STEP_FT) + 1
            d, blocked = 0.0, False
            for i in range(1, n + 1):
                hh = h + dt * i / n
                q = [cen[0] + sgn * gs['r0'] * math.sin(hh), cen[1] - sgn * gs['r0'] * math.cos(hh)]
                e = s.elev_at(lat0 + q[0] / FT_DEG, lon0 + q[1] / (FT_DEG * k))
                hf = gs['h0'] - abs(dt) * i / n * gs['hpr']
                if e is None or hf - e < gs['fl']:
                    blocked = True; break
                d += (hf - (e if e is not None else hf)) * s.STEP_FT
            p = [cen[0] + sgn * gs['r0'] * math.sin(hc), cen[1] - sgn * gs['r0'] * math.cos(hc)]
            sc = 0.0; a = abs(dt)
            for i in range(400 if not blocked else 0):
                hf = gs['h0'] - sc / gs['ld'] - a * gs['hpr']
                e = s.elev_at(lat0 + p[0] / FT_DEG, lon0 + p[1] / (FT_DEG * k))
                if e is None or hf - e < gs['fl']:      # (the known ground's end too)
                    break
                d += (hf - (e if e is not None else hf)) * s.STEP_FT; sc += s.STEP_FT
                p = [p[0] + s.STEP_FT * math.cos(hc), p[1] + s.STEP_FT * math.sin(hc)]
            if best is None or d > bd:
                bd = d; best = j * 15
        return best

    # ---- runways
    def runways(s, range_nm):
        out = []
        for rid, r in LOWI.items():
            ground = 'pavement' if r['surface'] in (1, 2) else 'grass'
            nd = s.needs(ground, s.head(r['heading']))
            lda = (r['length'] - r['threshold']) * M2FT
            if lda < nd[1] - nd[0] + nd[4]:
                continue
            t0 = coord_move(r['lat'], r['lon'], r['heading'], r['threshold'])
            e = coord_move(t0[0], t0[1], r['heading'], lda * FT2M)
            # (its touchdown anywhere along it the length left still takes: the aim point slid on by a cell at a time)
            lda0 = lda
            for k in range(int(lda0 * FT2M / CELL_M) + 1):
                lda = lda0 - k * CELL_M * M2FT
                if lda < nd[1] - nd[0] + nd[4]:
                    break
                t = coord_move(t0[0], t0[1], r['heading'], k * CELL_M)
                back = (nd[3] + nd[0]) * FT2M
                fpt = coord_move(t[0], t[1], r['heading'] + 180 if back > 0 else r['heading'], abs(back))
                et, ee = s.dem.elev_m(*t), s.dem.elev_m(*e)
                c = dict(kind='RWY', name='LOWI ' + rid + ('' if k == 0 else ' +%d m' % (k * CELL_M)), lat=fpt[0], lon=fpt[1],
                         crs=r['heading'], head=s.head(r['heading']), ends=[t[0], t[1], e[0], e[1]], len=lda, end=back * M2FT + lda)
                if et is not None and ee is not None:
                    c['slope'] = (ee - et) * M2FT / lda
                    if c['slope'] < -DOWN_MAX or c['slope'] > RISE_MAX:
                        continue
                    ns = s.needs(ground, s.head(r['heading']), c['slope'])
                    if lda < ns[1] - ns[0] + ns[4]:
                        continue
                    c['elev'] = et * M2FT - c['slope'] * back * M2FT
                out.append(c)
        return out

    # ---- open ground
    def cell_ll(s, i, j):
        return [s.origin[0] + j * CELL_M * M2FT / FT_DEG, s.origin[1] + i * CELL_M * M2FT / (FT_DEG * math.cos(s.origin[0] * D2R))]

    def ckey(s, i, j):
        return '%d %d' % (nround(i) + 0, nround(j) + 0)

    def classify(s, g):
        m = g[1]
        if m is None:
            s.seen['(none)'] = s.seen.get('(none)', 0) + 1; return ''
        if m['names']:
            s.seen[m['names'][0]] = s.seen.get(m['names'][0], 0) + 1
        if not m['solid']:
            return ''
        for n in m['names']:
            if n in OPEN:
                return OPEN[n]
            if PAVED(n):
                return 'pavement'
        for n in m['names']:
            if n in ROUGH:
                return 'rough'
        return ''

    def sample(s, i, j, again=0):
        k = s.ckey(i, j)
        if k in s.grid:
            return
        ll = s.cell_ll(i, j)
        g = s.dem.geodinfo(ll[0], ll[1])
        s.count['sampled'] += 1
        if g is None:
            s.count['bare'] += 1
            if not again:
                s.retry.append([i, j])
            return
        ground = s.classify(g)
        s.grid[k] = [ground, g[0] * M2FT]
        if ground != '':
            heapq.heappush(s.rheap if ground == 'rough' else s.heap, (-s.promise(ll, g[0] * M2FT), s.count['sampled'], i, j))
            s.count['open'] += ground != 'rough'

    def promise(s, ll, elev):
        # (tried in the order of what a line there could leave, its best course's: tried nearest first, the near cells came first, which an
        # aligned approach reaches only round a whole circle, and over LOWI the fields 1.8 NM out that were within reach
        # at the engines' cut were tried 10 s later, out of it)
        lat, lon = s.cnum('/position/latitude-deg'), s.cnum('/position/longitude-deg')
        trk = math.atan2(s.cnum('/velocities/speed-east-fps'), s.cnum('/velocities/speed-north-fps')) * R2D
        return max(s.plans(lat, lon, trk, dict(lat=ll[0], lon=ll[1], crs=hk * 360 / HEADINGS, elev=elev))[0][0] for hk in range(HEADINGS))

    def next_cell(s, rc):
        if s.queue is None:
            s.queue, s.qi = s.search_order(rc), 0
        while s.qi < len(s.queue):
            ij = s.queue[s.qi]; s.qi += 1
            if s.ckey(*ij) not in s.grid:
                return ij
        return None

    def search_order(s, rc):
        # (the cells within the search's radius in the order of what a straight-in to each could leave, from where the
        # engines quit: the way there and the turn to its bearing at the turns' measured cost, a whole circle more where
        # the final's start is nearer than the turn onto it takes (2 R0). Only the order the land cover is read in, the
        # choice the plans' own; ring by ring from the aircraft the fields 1.8 NM out that were within reach at the cut
        # were read 6 s later, their margin gone at 950 ft/s, the aircraft flying away from them)
        num = s.cnum
        lat, lon = num('/position/latitude-deg'), num('/position/longitude-deg')
        trk = math.atan2(num('/velocities/speed-east-fps'), num('/velocities/speed-north-fps'))
        reach, kt = num('ap/eo-reach-nm') * 6076.12 * num('ap/eo-brake-nom-ratio', 1), num('ap/eo-site-turn-ft-rad', 4000)
        Lf, R0 = num('ap/eo-site-final-full-ft', 5000), num('ap/eo-site-r0-ft', 2000)
        k = CELL_M * M2FT
        out = []
        for i in range(-rc, rc + 1):
            for j in range(-rc, rc + 1):
                if i * i + j * j > rc * rc:
                    continue
                dN, dE = (s.origin[0] - lat) * FT_DEG + j * k, (s.origin[1] - lon) * FT_DEG * math.cos(lat * D2R) + i * k
                d = math.hypot(dN, dE)
                turn = abs(wrap(math.atan2(dE, dN) - trk)) + (2 * math.pi if d < Lf + 2 * R0 else 0)
                out.append((d + kt * turn - reach, i, j))
        out.sort()
        return [[i, j] for _, i, j in out]

    def strip(s, i, j, h, ln):
        n = int(math.ceil(ln * FT2M / CELL_M))
        si, co = math.sin(h * D2R), math.cos(h * D2R)
        el = []; ground = 'pavement'
        for k in range(n + 1):
            c = s.grid.get(s.ckey(i + k * si, j + k * co))
            if c is None and s.budget > 0:
                s.budget -= 1
                s.sample(nround(i + k * si), nround(j + k * co))
                c = s.grid.get(s.ckey(i + k * si, j + k * co))
            if c is None:
                return '?'
            if c[0] == '':
                return None
            if KIND_RANK[c[0]] > KIND_RANK[ground]:      # (the strip's worst ground)
                ground = c[0]
            el.append(c[1])
        step = CELL_M * M2FT; mx = n / 2; my = sum(el) / (n + 1)
        sxy = sum((k - mx) * (el[k] - my) for k in range(n + 1)); sxx = sum((k - mx) ** 2 for k in range(n + 1))
        sl = sxy / sxx / step
        if sl > RISE_MAX or sl < -DOWN_MAX:
            return None
        e0 = my - sl * mx * step
        fl = s.num('ap/eo-float-ft', 10)
        for k in range(n + 1):
            if abs(el[k] - (e0 + sl * k * step)) > fl:
                return None
        return [ground, e0, sl]

    def try_cell(s, ij):
        again = 0
        for hk in range(HEADINGS):
            h = hk * 360 / HEADINGS
            nd = s.needs('crop', s.head(h))
            ln = nd[1] - nd[0] + nd[3] - nd[2] + nd[4]
            g = s.strip(ij[0], ij[1], h, ln)
            if g is None:
                continue
            if g == '?':
                again = 1; continue
            s0 = s.cell_ll(ij[0], ij[1])
            pl = g; g = pl[0]
            if pl[2] < 0:
                nf = s.needs('crop', s.head(h), pl[2])
                ln2 = nf[1] - nf[0] + nf[3] - nf[2] + nf[4]
                g2 = s.strip(ij[0], ij[1], h, ln2)
                if g2 is None:
                    continue
                if g2 == '?':
                    again = 1; continue
                pl = g2; g = pl[0]; ln = ln2
            nd = s.needs(g, s.head(h), pl[2])
            back = (nd[0] + nd[2]) * FT2M
            fpt = coord_move(s0[0], s0[1], h + 180 if back > 0 else h, abs(back))
            e = coord_move(s0[0], s0[1], h, ln * FT2M)
            nfd = dict(kind=g, name='FIELD', lat=fpt[0], lon=fpt[1], crs=h, head=s.head(h), ends=[s0[0], s0[1], e[0], e[1]], len=ln,
                       elev=pl[1] - pl[2] * back * M2FT, slope=pl[2], end=back * M2FT + ln)
            nfd['found'] = s.t(); s.fields.append(nfd); s.count['lines'] = s.count.get('lines', 0) + 1
        return again

    # ---- ranking and choice
    def key_of(s, c):
        return '%s %.5f %.5f %d' % (c['name'], c['lat'], c['lon'], c['crs'])

    def tier(s, c):
        return s.plan_tier(c['plan'], c['margin'])

    def order(s, a, b):
        if a['clear'] != b['clear']:
            return -1 if a['clear'] else 1
        ta, tb = s.tier(a), s.tier(b)
        if ta != tb:
            return tb - ta
        if ta == 0:
            return (b['margin'] + b['plan'][8]) - (a['margin'] + a['plan'][8])
        if ta == 1:
            return a['margin'] - b['margin']      # (high: the least)
        ka, kb = KIND_RANK[a['kind']], KIND_RANK[b['kind']]
        if ka != kb:
            return ka - kb
        ha, hb = min(a['head'], HEAD_KT * 2), min(b['head'], HEAD_KT * 2)
        if abs(ha - hb) > HEAD_KT:
            return hb - ha
        ra, rb = 'slope' in a and a['slope'] > 0.001, 'slope' in b and b['slope'] > 0.001
        if ra != rb:
            return 1 if ra else -1
        fa_, fb_ = a['plan'][10] > 0, b['plan'][10] > 0
        if fa_ != fb_:
            return 1 if fa_ else -1                 # (a full final before a cut one)
        # (robust, the least surplus; a stretch, the most it keeps)
        return a['margin'] - b['margin'] if ta == 4 else b['margin'] - a['margin']

    def order_cat(s, a, b):
        if a['clear'] != b['clear']:
            return -1 if a['clear'] else 1
        ta, tb = s.tier(a), s.tier(b)
        if ta != tb:
            return tb - ta
        if ta <= 1:
            return 0                             # (both short or both high: only by how much, the margin)
        ka, kb = KIND_RANK[a['kind']], KIND_RANK[b['kind']]
        if ka != kb:
            return ka - kb
        ha, hb = min(a['head'], HEAD_KT * 2), min(b['head'], HEAD_KT * 2)
        if abs(ha - hb) > HEAD_KT:
            return hb - ha
        ra, rb = 'slope' in a and a['slope'] > 0.001, 'slope' in b and b['slope'] > 0.001
        if ra != rb:
            return 1 if ra else -1
        return 0

    @staticmethod
    def kname(pl):
        g = pl[7]
        return 'none' if g is None else '%s%s%s, circle at %.0f deg%s' % ('straight, ' if g[10] > 0 else '', 'R' if g[1] > 0 else 'L', 'R' if pl[2] > 0 else 'L', pl[9],
                                                                          ', final %.0f ft' % pl[10] if pl[10] > 0 else '')

    def send_path(s, pl):
        g = pl[7]
        s.setp('ap/eo-site-side', pl[2]); s.setp('ap/eo-site-turn', 0 if g is None else g[1]); s.setp('ap/eo-site-hac-deg', pl[9]); s.setp('ap/eo-site-final-cmd-ft', pl[10])
        s.k_sent = None if g is None else g[11]
        s.hac_sent, s.fin_sent = pl[9], pl[10]

    def send(s, c):
        if c is None:
            s.setp('ap/eo-site-on', 0); s.setp('ap/eo-site-side', 0); s.setp('ap/eo-site-turn', 0); s.setp('ap/eo-site-hac-deg', 0); s.setp('ap/eo-site-final-cmd-ft', 0); s.k_sent = None; return
        s.send_path(c['plan'])
        s.setp('ap/eo-site-lat-deg', c['lat']); s.setp('ap/eo-site-lon-deg', c['lon']); s.setp('ap/eo-site-course-deg', c['crs'])
        planed = 'slope' in c and 'elev' in c
        s.setp('ap/eo-site-plane', planed)
        if planed:
            s.setp('ap/eo-site-elev-ft', c['elev']); s.setp('ap/eo-site-slope', c['slope']); s.setp('ap/eo-site-end-ft', c['end'])
        s.setp('ap/eo-site-on', 1); s.setp('ap/eo-site-new', 1)

    def find(s, k):
        for c in s.cands:
            if s.key_of(c) == k:
                return c
        return None

    def rank(s):
        num = s.num
        lat, lon = num('/position/latitude-deg'), num('/position/longitude-deg')
        trk = math.atan2(num('/velocities/speed-east-fps'), num('/velocities/speed-north-fps')) * R2D
        alls = s.runways(num('ap/eo-reach-nm') + 2) + s.fields
        gs = s.glide_state()
        st = num('ap/eo-site-state')
        ck = s.chosen
        w = num('inertia/weight-lbs')
        # (the floor of a margin within reach: the flare's measured scatter short and the reach's error low, deadland.py)
        s.floor = -s.FLARE[0] + max(0, -s.at('err-lo', w))
        s._cache = {}
        for c in alls:
            c['opts'] = s.energy_plans(lat, lon, trk, c)
            c['plan'], c['clear'] = c['opts'][0], 1
            c['margin'], c['path'] = c['plan'][0], c['plan'][1]
        alls.sort(key=functools.cmp_to_key(s.order))
        s._cache = None
        # (the terrain looked under down the ranking until TOP lines are clear, no more than 4 TOP of them; those not
        # looked under not known clear)
        checked = good = 0
        for c in alls:
            if (good >= TOP or checked >= 4 * TOP) and (ck is None or s.key_of(c) != ck):
                c['clear'] = 0; continue
            checked += 1
            opts = []
            for pl in c['opts']:
                c['margin'], c['plan'] = pl[0], pl
                opts.append((pl, s.path_clear(lat, lon, trk, c, gs)))
            pick = next((o for o in opts if o[1]), opts[0])
            if ck is not None and s.key_of(c) == ck and 0.5 < st < 1.5:
                # (the path flown kept but for one within reach and clear: one short is no way there; swapped for a
                # path 33 000 ft short, its glide ending over level ground, when the one flown's clearance flickered)
                fl = next((o for o in opts if o[0][7] is not None and o[0][7][11] == s.k_sent and abs(o[0][9] - s.hac_sent) < 0.1 and abs(o[0][10] - s.fin_sent) < 1), None)
                better = pick[1] and pick[0][0] >= 0 and (fl is None or not fl[1] or fl[0][0] < 0
                                                         or pick[0][0] > fl[0][0] + 0.5 * num('ap/eo-site-turn-ft-rad', 4000))
                if fl is not None and not better:
                    pick = fl
                elif better and pick[0][7] is not None and pick[0][7][11] != s.k_sent:
                    s.log('%s %s: path %s for %s (margin %.0f for %.0f%s)' % (c['kind'], c['name'], s.kname(pick[0]), 'none' if fl is None else s.kname(fl[0]),
                          pick[0][0], -1e9 if fl is None else fl[0][0], '' if fl is None or fl[1] else ', its path meets the ground'))
                    s.send_path(pick[0])
            c['plan'], c['clear'] = pick
            c['margin'], c['path'] = c['plan'][0], c['plan'][1]
            good += c['clear']
        cur = None if s.chosen is None else s.find(s.chosen)          # (in the last ranking's list, as eosite.nas)
        if cur is not None and st > 0.5:
            cur['margin'] = num('ap/eo-site-margin-ft'); cur['path'] = num('ap/eo-site-path-ft')
        if cur is not None and st > 1.5:                               # (judged along what the guidance flies)
            cur['clear'] = s.path_clear(lat, lon, trk, cur, gs, st)
        s.cands = sorted(alls, key=functools.cmp_to_key(s.order))
        if s.none:
            s.setp('ap/eo-open-on', 0); return
        # the best from here, every second: clear and within reach; none yet, while the search goes on nothing
        # (the open course meanwhile), after it the least short clear one
        # (a high one, as the least short, only after it: taken at the first ranking, a second after the engines quit, the
        # aircraft was on its final in seconds, past changing, and landed 25 000 ft on)
        best = next((c for c in s.cands if c['clear'] and s.tier(c) >= (1 if not s.sampling else 2)), None)
        if best is None and not s.sampling:
            best = next((c for c in s.cands if c['clear']), None)
        short = 0 if cur is None else -s.needs('pavement' if cur['kind'] == 'RWY' else cur['kind'])[0]
        now = s.t()
        if not s.picked:
            if cur is None:
                if best is not None:
                    s.chosen = s.key_of(best); s.send(best); s.changed = now
                    s.log('chosen %s %s, margin %.0f ft, path %.2f NM (%s)%s' % (best['kind'], best['name'], best['margin'], best['path'] / 6076.12, s.kname(best['plan']),
                                                                           ' (none within reach: the least short)' if s.tier(best) == 0 else ' (high)' if s.tier(best) == 1 else ' (a stretch: the brake in)' if s.tier(best) < 4 else ''))
            elif st < 2.5 or s.tier(cur) == 1:           # (on its final too while it stays high)
                fails = not cur['clear'] or (cur['margin'] + cur['plan'][8] < -short and (s.sampling or (best is not None and s.tier(best) >= 1)))
                # (given up for none only before its circle and only for its path meeting the ground: short, it is still
                # the way down onto open ground; dropped short on the circle, 900-1150 ft over LOWI's north slope and
                # the search still going, ENG OUT came down on 30-88 % slopes beside the runway)
                if best is None or s.key_of(best) == s.chosen:
                    if fails and (best is None or s.key_of(best) == s.chosen) and not cur['clear'] and st < 1.5:
                        s.log('%s %s: %s; none better yet' % (cur['kind'], cur['name'], 'its path meets the ground' if not cur['clear'] else 'out of reach'))
                        s.chosen = None; s.send(None); s.changed = now
                elif fails or (s.order_cat(best, cur) < 0 and now - s.changed > HOLD_S):
                    s.log('%s %s (margin %.0f) for %s %s (margin %.0f, %s)' % (best['kind'], best['name'], best['margin'], cur['kind'], cur['name'],
                                                                         cur['margin'], 'its path meets the ground' if not cur['clear'] else
                                                                         'out of reach' if cur['margin'] + cur['plan'][8] < -short else 'better'))
                    s.chosen = s.key_of(best); s.send(best); s.changed = now
        if s.chosen is None:
            oc = s.open_course(lat, lon, trk, gs)
            s.setp('ap/eo-open-on', int(oc is not None))
            if oc is not None:
                s.setp('ap/eo-open-hdg-deg', oc)
            s.open = oc
        else:
            s.setp('ap/eo-open-on', 0); s.open = None
        if int(now) % 5 == 0 and cur is not None:
            s.log('%s %s: margin %.0f ft here, %.0f the flight model\'s; path %.2f NM, state %d, clear %d' % (
                cur['kind'], cur['name'], cur['margin'], num('ap/eo-site-margin-ft'), cur['path'] / 6076.12, st, cur['clear']))

    def reset(s):
        s.grid, s.origin, s.ring, s.retry, s.tries, s.ti, s.fields, s.cands = {}, None, [0, 0], [], [], 0, [], []
        s.chosen, s.picked, s.none, s.sampling, s.seen, s.count, s.changed = None, 0, 0, 1, {}, dict(sampled=0, bare=0, open=0), -100
        s.centre, s.open, s.rough_tries, s.ri, s.k_sent = [0, 0], None, [], 0, None
        s.heap, s.rheap, s.later, s.floor, s.queue, s.qi, s.budget, s.hac_sent, s.fin_sent = [], [], [], 1000.0, None, 0, 0, 0.0, 0.0
        s.send(None); s.setp('ap/eo-open-on', 0)

    def tries_tick(s):
        # the tries: the best promise first, open ground before rough unless rough promises more within reach; the
        # cells their lines still wait on read from the tick's budget, those it ran out of again a second later
        if s.ticks % RANK_TICKS == 0:
            for e in s.later:
                heapq.heappush(s.rheap if s.grid[s.ckey(e[2], e[3])][0] == 'rough' else s.heap, e)
            s.later = []
        k = 0
        while k < TRY_PER_TICK and (s.heap or s.rheap):
            ho, hr = (-s.heap[0][0] if s.heap else None), (-s.rheap[0][0] if s.rheap else None)
            q = s.heap if ho is not None and (hr is None or ho >= s.floor or ho >= hr) else s.rheap
            e = heapq.heappop(q)
            if s.try_cell([e[2], e[3]]):
                s.later.append(e)
            k += 1

    def update(s):
        num = s.num
        if not num(AF + 'eng-out'):
            if s.was:
                s.reset()
            s.was = 0
            return
        if s.was and num('ap/eo-phase') > 2.5:
            return
        if not s.was:
            s.reset()
            s.origin = [num('/position/latitude-deg'), num('/position/longitude-deg')]
            s.centre = [0, 0]
            s.log('on: reach %.1f NM, the land cover searched from here' % num('ap/eo-reach-nm'))
        s.was = 1
        s._cache = {}                                                   # (the aircraft's state read once a tick)
        s.floor = -s.FLARE[0] + max(0, -s.at('err-lo', num('inertia/weight-lbs')))
        rc = int(min(SEARCH_NM, num('ap/eo-reach-nm') + 1) * 1852 / CELL_M)
        s.budget = PER_TICK
        s.tries_tick()
        while s.budget > 0:
            s.budget -= 1
            ij = s.next_cell(rc)
            if ij is None:
                if not s.retry:
                    s.sampling = 0; break
                ij = s.retry.pop(0)
                s.sample(ij[0], ij[1], 1)
            else:
                s.sample(ij[0], ij[1])
        s._cache = None
        s.ticks += 1
        if s.ticks % RANK_TICKS == 0:
            s.rank()


def nose_wheel(f, td, crs):
    """the moment the nose wheel comes down: the wings and the track then, the direction set from there on"""
    if 'nw' not in td and f['gear/unit[0]/WOW'] > 0.5:
        trk = math.atan2(f['velocities/v-east-fps'], f['velocities/v-north-fps']) * R2D
        w = lambda a: (a + 180) % 360 - 180
        td['nw'] = dict(bank=f['attitude/phi-deg'], trk=w(trk - crs) if crs is not None else 0.0, nose=w(f['attitude/psi-deg'] - crs) if crs is not None else 0.0, kt=f['velocities/vg-fps'] / 1.6878)


# (built-up ground: FlightGear draws buildings on Town and SubUrban, and Construction, Industrial and Transport are structures and roads.
# A touchdown or a rollout there is no landing, however soft: the model has no buildings to hit)
BUILT = {'Town', 'SubUrban', 'Urban', 'BuiltUpCover', 'Construction', 'Industrial', 'Transport'}


def wheels_wet(f, dem, wet, dt, built=None):
    """water under a wheel with weight on it. FlightGear's water carries the aircraft like ground and the model feels nothing, so only
    the land cover can tell: each wheel's own place against it, the wheels in the order of the gear's units. wet: the wheels, the
    cover, the speed it was first met at, the way run with a wheel in it, ft"""
    U = DL.DS.U
    psi, la, lo = f['attitude/psi-rad'], f['position/lat-geod-deg'], f['position/long-gc-deg']
    cx, cy = f['inertia/cg-x-in'], f['inertia/cg-y-in']
    fs, bl, _ = U.MAIN_GEAR
    any_ = False
    for i, (x, y) in enumerate(((U.NOSE_GEAR[0], U.NOSE_GEAR[1]), (fs, -bl), (fs, bl))):
        if f['gear/unit[%d]/WOW' % i] < 0.5:
            continue
        fw, rt = (cx - x) / 12, (y - cy) / 12
        n, e = fw * math.cos(psi) - rt * math.sin(psi), fw * math.sin(psi) + rt * math.cos(psi)
        g = dem.geodinfo(la + n / FT_DEG, lo + e / (FT_DEG * math.cos(la * D2R)))
        if g and g[1] and not g[1]['solid']:
            any_ = True
            wet['wheels'].add(i)
            wet['cover'] = wet['cover'] or g[1]['names'][0]
            wet['kt'] = wet['kt'] or f['velocities/vg-fps'] / 1.6878
        if built is not None and g and g[1] and g[1]['names'][0] in BUILT:
            built['wheels'].add(i); built['any'] = True
            built['cover'] = built['cover'] or g[1]['names'][0]
            built['kt'] = built['kt'] or f['velocities/vg-fps'] / 1.6878
    if any_:
        wet['ft'] += f['velocities/vg-fps'] * dt
    if built is not None and built.pop('any', False):
        built['ft'] += f['velocities/vg-fps'] * dt


def run(lat, lon, hdg, kcas, alt, sets={}, W=38722.0, cut_s=3.0, T=300.0, quiet=False, every=2.0, nac=0.0):
    dem = Dem()
    g0 = dem.elev_m(lat, lon) * M2FT
    f = start(lat, lon, hdg, kcas, alt, g0, W, nac)
    for k, x in sets.items():
        f[k if '/' in k else 'ap/eo-' + k] = x
    f[AF + 'cpld'] = 1
    f['/autopilot/settings/heading-bug-deg'] = hdg; f['/autopilot/settings/target-altitude-ft'] = alt
    f['/autopilot/settings/target-speed-kt'] = kcas
    if kcas <= 35 and not os.environ.get('BENCH_NOHH'):      # (the hover hold's, as HVR CPLD: up to 40 kt over the ground)
        f['hh/engage-cmd'] = 1
    else:
        for m in ('hdg-hold', 'alt-hold', 'spd-hold'):
            f[AF + m] = 1
    dt = f['simulation/dt']
    clock = {'t': 0.0}
    fd = Finder(f, dem, lambda: clock['t'])
    tick_every = int(round(TICK_S / dt))
    out = dict(start=(lat, lon, hdg, kcas, alt), track=[])
    nr = DL.NR0
    for i in range(int(T / dt)):
        t = i * dt; clock['t'] = t
        if f['position/lat-geod-deg'] != f['position/lat-geod-deg']:
            print('DIVERGED at %.1f s' % t); out['diverged'] = t; return out
        e = dem.elev_m(f['position/lat-geod-deg'], f['position/long-gc-deg'])
        if e is None:
            print('LEFT the terrain data at %.1f s, %.0f ft over its last ground' % (t, f['hh/h-wheels-ft'])); out['left'] = t; return out
        f['position/terrain-elevation-asl-ft'] = e * M2FT
        if abs(t - cut_s) < dt / 2:
            for n in (0, 1):
                f['/controls/engines/engine[%d]/cutoff' % n] = 1
        if f[AF + 'eng-out'] > 0.5 and f['hh/h-wheels-ft'] < f['ap/eo-gear-ft'] and f['gear/gear-cmd-norm'] < 0.5:
            f['gear/gear-cmd-norm'] = 1.0
        if i % tick_every == 0:
            ch = fd.chosen
            fd.update()
            if fd.chosen != ch and fd.chosen is not None:          # (the margin's drift: at the last choice, the final, the flare)
                out['m'] = dict(choice=None, choice_t=t, final=None, flare=None)
        f.run()
        m = out.get('m')
        if m is not None:
            if m['choice'] is None and f['ap/eo-site-state'] > 0.5:
                m['choice'] = f['ap/eo-site-margin-ft']; m['path'] = f['ap/eo-site-path-ft']
            if m['final'] is None and f['ap/eo-site-state'] > 2.5:
                m['final'] = f['ap/eo-site-margin-ft']
            if m['flare'] is None and f['ap/eo-phase'] > 2.5:
                m['flare'] = f['ap/eo-site-margin-ft']; m['flare_x'] = f['ap/eo-site-x-ft']
        if i % int(round(every / dt)) == 0:
            row = (t, f['ap/eo-phase'], f['ap/eo-site-state'], f['position/lat-geod-deg'], f['position/long-gc-deg'], f['hh/h-wheels-ft'],
                   f['position/terrain-elevation-asl-ft'], f['velocities/vc-kts'], f['attitude/psi-deg'], f['attitude/phi-deg'],
                   f['velocities/h-dot-fps'], f['ap/eo-site-on'], fd.open if fd.chosen is None else -1)
            out['track'].append(row)
            if not quiet:
                print('%6.1f ph %d st %d  %.5f %.5f  %5.0f ft over %5.0f  %5.1f KCAS hdg %5.1f bank %+5.1f  %+6.1f ft/s  site %d open %s' % (
                    row[0], row[1], row[2], row[3], row[4], row[5], row[6], row[7], row[8] % 360, row[9], row[10], row[11], row[12]), flush=True)
        if t > cut_s:
            nr = DL.rotor_range(f, nr)
        if t > cut_s + 1 and f['hh/h-wheels-ft'] < 200 and DL.down(f):
            lat_, lon_ = f['position/lat-geod-deg'], f['position/long-gc-deg']
            gi = dem.geodinfo(lat_, lon_)
            # the ground's slope where it came down, along the track
            trk = math.atan2(f['velocities/v-east-fps'], f['velocities/v-north-fps']) * R2D
            a, b = coord_move(lat_, lon_, trk, -100), coord_move(lat_, lon_, trk, 100)
            ea, eb = dem.elev_m(*a), dem.elev_m(*b)
            out['td'] = dict(t=t, sink=-f['velocities/h-dot-fps'], kcas=f['velocities/vc-kts'], gs=f['velocities/vg-fps'] / 1.6878,
                             pitch=f['attitude/theta-deg'], bank=f['attitude/phi-deg'], nac=f['ctl/nacelle-deg'], gear=f['gear/gear-pos-norm'],
                             lat=lat_, lon=lon_, ground_ft=f['position/terrain-elevation-asl-ft'], cover=gi[1]['names'][0] if gi and gi[1] else None,
                             slope=None if ea is None or eb is None else (eb - ea) / 200.0,
                             clear=__import__('deadstick').clearance(f['ctl/nacelle-deg'], f['attitude/theta-deg']),
                             site=f['ap/eo-site-on'], chosen=fd.chosen, nr=nr)
            td = out['td']
            # (the descent against the ground, what the gear meets: the rise of the ground at the speed along it added, as deadland.py's flare has it)
            td['sink_g'] = td['sink'] + (td['slope'] or 0.0) * f['velocities/vg-fps']
            # (the low rotor's tip, banked: down by the hub's arm from the wheel on the ground. The clearance alone read a landing at 9 deg of bank as level)
            U_ = DL.DS.U
            td['clear'] -= (U_.BL_PIV - abs(U_.MAIN_GEAR[1])) / 12 * abs(math.sin(td['bank'] / R2D))
            # (the nose at the wheels: the direction is set there. Against the track, the crab, and against the line, what the run follows)
            psi_ = f['attitude/psi-deg']
            td['crab'] = (psi_ - trk + 180) % 360 - 180
            ok = td['sink_g'] < 12 and td['gear'] > 0.99 and td['clear'] > 1.0 and abs(td['bank']) < 10 and td['pitch'] < 7.3
            wet = dict(wheels=set(), cover=None, kt=None, ft=0.0)
            built = dict(wheels=set(), cover=None, kt=None, ft=0.0)
            # rolled out, braked from 2 s: where it stops against the chosen strip's end, the ground on the way
            c = fd.find(fd.chosen) if fd.chosen else None
            t1, covers, roll_clear, n1 = t, set(), 99.0, 0
            wheels_wet(f, dem, wet, 0.0, built)
            crs_ = c['crs'] if c is not None else None
            nose_wheel(f, td, crs_)
            while f['velocities/vg-fps'] > 2 * 1.6878 and t1 - t < 60:
                t1 += dt; clock['t'] = t1
                e = dem.elev_m(f['position/lat-geod-deg'], f['position/long-gc-deg'])
                f['position/terrain-elevation-asl-ft'] = (e if e is not None else 0.0) * M2FT
                if t1 - t > 2 and not os.environ.get('BENCH_FDM_BRAKES'):      # (builds before ENG OUT's own brakes)
                    f['fcs/left-brake-cmd-norm'] = f['fcs/right-brake-cmd-norm'] = 1.0
                f.run()
                n1 += 1
                nose_wheel(f, td, crs_)
                # (on the wheels still? the pitch at its most, and any time off them again, a float or a bounce)
                ro = td.setdefault('rollout', dict(pitch=-99.0, air_s=0.0, bank=0.0))
                ro['pitch'] = max(ro['pitch'], f['attitude/theta-deg']); ro['bank'] = max(ro['bank'], abs(f['attitude/phi-deg']))
                # (the nose coming down on the mains: how fast, how long after them, and what the nose wheel meets)
                ro['q_min'] = min(ro.get('q_min', 0.0), f['velocities/q-rad_sec'] * R2D)
                if 'nose_s' not in ro and f['gear/unit[0]/WOW'] > 0.5:
                    ro['nose_s'] = t1 - t
                    ro['nose_q'] = f['velocities/q-rad_sec'] * R2D
                    ro['nose_kcas'] = f['velocities/vc-kts']
                if hasattr(fd, 'roll_tick'):
                    fd.roll_tick()
                if f['gear/unit[1]/WOW'] + f['gear/unit[2]/WOW'] < 0.5:
                    ro['air_s'] += dt
                if n1 % 6 == 0:
                    wheels_wet(f, dem, wet, 6 * dt, built)
                if int(t1 / dt) % 240 == 0:
                    g_ = dem.geodinfo(f['position/lat-geod-deg'], f['position/long-gc-deg'])
                    covers.add(g_[1]['names'][0] if g_ and g_[1] else '?')
                    roll_clear = min(roll_clear, __import__('deadstick').clearance(f['ctl/nacelle-deg'], f['attitude/theta-deg']))
            sl, so = f['position/lat-geod-deg'], f['position/long-gc-deg']
            past = None
            if c is not None:
                e1 = c['ends'][2:4]; cc = c['crs'] * D2R
                dN, dE = (sl - e1[0]) * FT_DEG, (so - e1[1]) * FT_DEG * math.cos(sl * D2R)
                past = dN * math.cos(cc) + dE * math.sin(cc)
            td['stop'] = dict(lat=sl, lon=so, past_end_ft=past, covers=sorted(covers), roll_clear=roll_clear)
            td['wet'] = dict(wet, wheels=sorted(wet['wheels'])) if wet['wheels'] else None
            td['built'] = dict(built, wheels=sorted(built['wheels'])) if built['wheels'] else None
            ok = ok and td['wet'] is None and td['built'] is None
            if c is not None:
                cc = c['crs'] * D2R
                dN, dE = (lat_ - c['lat']) * FT_DEG, (lon_ - c['lon']) * FT_DEG * math.cos(lat_ * D2R)
                td['vs_site'] = (dN * math.cos(cc) + dE * math.sin(cc), dE * math.cos(cc) - dN * math.sin(cc), c['kind'], c['crs'])
                td['nose'] = (psi_ - c['crs'] + 180) % 360 - 180
                sN, sE = (sl - c['lat']) * FT_DEG, (so - c['lon']) * FT_DEG * math.cos(lat_ * D2R)
                td['stop']['off_ft'] = sE * math.cos(cc) - sN * math.sin(cc)
            print('  stopped %.0f s later at %.5f %.5f, %s the strip\'s end; over %s; tips rolling %+.1f ft' % (
                t1 - t, sl, so, 'n/a' if past is None else '%+.0f ft past' % past, ', '.join(sorted(covers)), roll_clear), flush=True)
            print('DOWN at %.1f s: %.1f ft/s, %.0f KCAS, pitch %.1f, bank %.1f, nacelles %.1f (tips %+.1f ft), gear %.0f %%; at %.5f %.5f, '
                  'ground %.0f ft (%s), slope %s; site %s: %s; rotor %.0f-%.0f %%' % (
                      t, td['sink'], td['kcas'], td['pitch'], td['bank'], td['nac'], td['clear'], 100 * td['gear'], lat_, lon_, td['ground_ft'],
                      td['cover'], 'n/a' if td['slope'] is None else '%+.1f %%' % (100 * td['slope']), td['chosen'],
                      'INTACT' if ok else 'NOT INTACT', *nr), flush=True)
            return out
    print('not down in %.0f s' % T)
    return out


if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if '=' not in a]
    sets = dict((kv.split('=')[0], float(kv.split('=')[1])) for kv in sys.argv[1:] if '=' in kv)
    if len(args) >= 5:
        lat, lon, hdg, kcas, alt = (float(x) for x in args[:5])
    else:
        lat, lon, hdg, kcas, alt = 47.26023, 11.34397, 139.0, 205.0, 6015.0
    run(lat, lon, hdg, kcas, alt, sets)
