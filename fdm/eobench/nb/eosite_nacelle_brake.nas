# ENG OUT's landing site: with both engines out (ENG OUT on), the places the glide can reach and land on, priced and
# ranked continually, the best handed to the flight model, which flies to it coupled (CPLD) or shows it to fly (fdm/
# genv22.py eng_out_site: a heading alignment circle and a continuous brake, as the Space Shuttle's terminal
# area energy management).
#
# The candidates are lines: a landing along a course from a point. The runways of the airports in reach (FlightGear's
# airport data), their aim points slid along them while the length left holds the landing; and strips anywhere on open
# ground from FlightGear's own land cover (geodinfo): pavement, grass, crops, an airfield's grass beside its runway, and
# after those level ground under natural cover (rough), each cell tried on HEADINGS courses. A strip must hold the
# landing the flight model makes at this weight (Systems/mv22b-displays.xml's eo-site, from fdm/deadland.py): the wheels
# under 50 ft to the stop, braked, with the scatter of where the flare begins; level within the float's height over the
# line fitted to it, rising no more than RISE_MAX and falling no more than DOWN_MAX. The chosen one's plane goes to the
# flight model with it, which flares over the plane rather than the ground under it on the final (ap/eo-h-ft).
#
# The way to a line: the four Dubins paths (a turn at ENG OUT's bank, a tangent, the circle, the final), a cross path
# with overlapping circles flying straight on first, at the turns' measured cost (deadland.py turns) and the glide
# flown with the brake part way out; the margin is what that leaves of the reach. What a line is worth (plan_tier):
# 4 within reach and spendable (the flaperons, a lap), 3 a stretch (the brake in), NB (2.5) high, spent by the nacelle
# brake as well (genv22 ap/eo-nb-deg: the same path flown steeper), 2 clean, under the flare's scatter, 1 too high to
# spend, 0 short. Every plan too high, a tight one (tight_search: the turns at 170 KCAS, 2.8 fits, 2.7 stretches) or a
# holding turn. A line short or high tries a shorter final (the Shuttle's minimum entry point) or a
# wider circle. Then runways before pavement before grass before crops before rough, into the wind, level or falling.
#
# The search reads the land cover in the order of what a straight-in to each block could leave from where the engines
# quit (from the aircraft outward, the fields in reach at the cut were read after they had gone out of it). Every line
# is priced in turn; each second the best of them are planned in full and the ground under their paths looked at, and
# the best clear one flown. A site is changed for a better while the aircraft is before its circle, and given up when
# its path meets the ground. Ground not known is not clear. None clear, the course with the most height kept over the
# ground (ap/eo-open-hdg-deg). Ctrl-L or the NAV format's SITE key steps through them, and to none.
#
# The costs, measured on the VM alone: four paths 14 us, a line's price 30 us, a sort with a Nasal comparison 4.5 us a
# line; so no string keys for the cells, no allocation in the pricing, one pass for the best lines, the reads and tries
# budgeted per tick. (At 10 Hz with the budgets halved the tries ran out of reads half way down their strips and waited
# a second: over the 40 starts round LOWI 28 landed intact for 33, and a ranking spread over the ticks no better.)
#
# What it cannot see: objects on open ground (pylons, fences, buildings and trees placed in it), the ground's state
# (soft, ploughed), anything under the land cover's resolution. Ditching is not modelled (FlightGear's water is solid
# to the flight model).

var JSB = "/fdm/jsbsim/";
var AFCS = "/instrumentation/mv22b/afcs/";
var DATA = "/instrumentation/mv22b/displays/data/eo-site/";
var OUT = "/instrumentation/mv22b/eosite/";
var D2R = math.pi / 180;
var R2D = 180 / math.pi;
var PI = math.pi;
var TWO_PI = 2 * math.pi;
var FT2M = 0.3048;
var M2FT = 1 / 0.3048;
var G0 = 32.174;
var FT_DEG = 60 * 6076.12;       # ft per degree of latitude
var CELL_M = 150;                # the land cover sampled every, m
var CELL_FT = CELL_M * M2FT;
var BLOCK = 4;                   # the search's order by blocks of this many cells a side
var TICK_S = 0.2;
var RANK_TICKS = 5;              # ranked every second
var PER_TICK = 1500;             # land cover reads per tick: the search is done in 2-3 s (a read 20 us, a strip 30 us, a price 30 us)
var TRY_PER_TICK = 200;           # open cells tried as strips' starts per tick (16: a field in reach at the cut found 15 s later, or never)
var PRICE_PER_TICK = 1000;        # lines priced per tick: raised together, or the lines queue here unranked
var RANK_READS = 400;            # land cover reads a ranking may spend under the paths it looks at
var MAX_LINES = 1500;            # over it the better half kept (open farmland: 12 lines a cell)
var HEADINGS = 12;               # the strips' courses, 30 deg apart
# The room wanted past a strip's end. Over 64 landings round LOWI the touchdown came within -1650..+1550 ft of its plan
# one time in ten each way, the strips sized for +389: 15 of them stopped past their strips, and in FlightGear one, on
# a field beside the Inn, ran off the end into the river
var SPARE_FT = 1200;
var SPARE_CELLS = 6;             # looked at past the end
var MAX_SHOWN = 16;
var SEARCH_NM = 8;               # the land cover searched no further (the sampling's cost)
var TOP = 12;                    # the lines whose paths are looked under each ranking, the best by all else
# the shorter finals tried: deadland.py finals, in 15 kt from all sides at 38 730 and 54 000 lb, from 800 ft 16 of 16
# intact, worst 8.7 ft/s; from 400 ft 15, worst 18.6, still rolling out at the flare
var FINALS = [4000, 3000, 2000, 1200, 800];
# A field's grade along the landing (fdm/deadland.py flare, the descent against the ground): falling, no more than FAA
# AC 150/5300-13B's longitudinal grade for small aircraft, the float lengthened (5.5-8.1 ft/s on 2 % down at 38 700 and
# 47 500 lb); rising, the float cut short and the touchdown fast, the corridor holding the nacelles low: at 47 500 lb on
# 0.5-2 % up 174-187 KCAS, the nacelles at 16-32 deg, the proprotors in the ground
var DOWN_MAX = 0.02;
var RISE_MAX = 0.005;
var CROSS_FT = 25;               # the ground a cell to the side of a strip within this of it: 5 %
var FALL_K = 10;                 # falling, the flare's way and the run longer by this per unit of fall (the flare's 6-23 % on 2 %)
var OPEN = {Airport: 2, AirportKeep: 2, Grass: 2, GrassCover: 2, Grassland: 2, DryLake: 2,
            grass_rwy: 2, dirt_rwy: 2, lakebed_taxiway: 2,
            MixedCrop: 3, IrrCrop: 3, DryCrop: 3, Crop: 3, CropGrass: 3, CropGrassCover: 3,
            MixedCropPastureCover: 3, DryCropPastureCover: 3, IrrCropPastureCover: 3};
# (no field open and level in reach: level ground under woods, scrub or natural cover, flown as a field, ranked after
# the fields; over LOWI the open course and the short fields came down on 6-117 % slopes instead)
var ROUGH = {MixedForestCover: 1, DeciduousBroadCover: 1, EvergreenBroadCover: 1, EvergreenForest: 1, DeciduousForest: 1,
             MixedForest: 1, CropWoodCover: 1, ShrubCover: 1, Scrub: 1, HerbTundraCover: 1, HerbTundra: 1, NaturalCrop: 1,
             Heath: 1, Sclerophyllous: 1};
var KIND_NAME = ["", "pavement", "grass", "crop", "rough"];      # the cells' ground, worse the higher
var KIND_RANK = {RWY: 0, pavement: 0, grass: 1, crop: 2, rough: 3};
var HEAD_KT = 5;                 # winds along the landing within this alike for the ranking
var HOLD_S = 10;                 # after a change of site, no other for this long (the flight model's margin settling)
# The holding turn. The path is the shortest there is, its length the geometry's, so the height over it is left over: from
# over the field the plans go from 10 000 ft high to 12 000 ft short with nothing between, a turn-round being 20 000 ft. So
# while every plan is too high none is taken: a steady turn over the lowest ground beside it, the plans looked at each
# second, until one fits, and that one flown. It burns any amount and brings the aircraft round through every heading:
# the Shuttle's circle, the path shaped to the energy, not the energy left to the path
var HOLD_MIN_FT = 1800;          # under this no more waiting: the best there is
var HOLD_LEAD = 50;              # deg ahead of the track, the turn held at the bank limit
var hold_dir = 0;                # the way it is turning, +1 right, 0 not holding
var hold_until = -1;
var HOLD_BURN_FPS = 110;         # energy spent in the turn with the nacelle brake out, flown 90-130 ft/s (66 without it)
var HOLD_TIMES = [4, 8, 12, 18, 26];
# a landing at the end of a hold, looked for before it starts: for each time and way round, where the aircraft will be and
# with what height, and whether a plan from there fits and clears the ground: [seconds, way, the line], or nil. Held on with
# nothing at its end, a field 12 000 ft high went out of reach within a second of the turn, and it came down short
var hold_search = func(gs, cl) {
    var lat = S.lat; var lon = S.lon; var trk = S.trk; var r0 = S.reach0; var k = S.k;
    var bank = num(JSB ~ "ap/eo-bank-lim-deg", 50) * D2R;
    var w = G0 * math.tan(bank) / S.vt; var R = S.vt / w;
    var found = nil;
    foreach (var T; HOLD_TIMES) {
        foreach (var sd; [1, -1]) {
            var h = trk + sd * w * T;
            var n = R * sd * (math.sin(h) - math.sin(trk)); var e = -R * sd * (math.cos(h) - math.cos(trk));
            # (the turn itself over ground it can clear)
            var ok = 1; var hw = gs.hw;
            for (var q = 1; q <= 6 and ok; q += 1) {
                var hq = trk + sd * w * T * q / 6;
                var e_ = elev_at(lat + R * sd * (math.sin(hq) - math.sin(trk)) / FT_DEG, lon - R * sd * (math.cos(hq) - math.cos(trk)) / (FT_DEG * k));
                if (e_ == nil or hw - HOLD_BURN_FPS * T * q / 6 - e_ < 2 * gs.fl) ok = 0;
            }
            if (!ok) continue;
            # (the energy then reckoned from the speed after the brake, ap/eo-nb-after-kt, as the flight model will)
            S.lat = lat + n / FT_DEG; S.lon = lon + e / (FT_DEG * k); S.trk = h; S.reach0 = r0 + (S.after_ft - HOLD_BURN_FPS * T) * S.ldu;
            var g2 = {h0: gs.h0 - HOLD_BURN_FPS * T, hw: hw - HOLD_BURN_FPS * T, ld: gs.ld, hpr: gs.hpr, hpr0: gs.hpr0, fl: gs.fl, r0: gs.r0};
            foreach (var c; cl) {
                if (contains(c, "dry") and !c.dry) continue;
                var k_ = geom(c, S.hb0, 0);
                if (k_ < 0 or tier_k(k_) < 3) continue;
                var pl = plans(c);
                var tp = plan_tier(pl[0]);
                if (tp < 3) continue;
                var sp = c.plan; var sm = c.margin;
                c.plan = pl[0]; c.margin = pl[0][0];
                var cl_ = path_clear(c, g2);
                c.plan = sp; c.margin = sm;
                if (cl_) { found = [T, sd, c]; break; }
            }
            S.lat = lat; S.lon = lon; S.trk = trk; S.reach0 = r0;
            if (found != nil) return found;
        }
    }
    return nil;
}
var hold_side = func(gs) {
    var r = gs.r0; var best = 0; var be = 1e9;
    foreach (var sd; [-1, 1]) {
        var h = S.trk + sd * PI / 2;
        var e = elev_at(S.lat + r * math.cos(h) / FT_DEG, S.lon + r * math.sin(h) / (FT_DEG * S.k));
        var e2 = elev_at(S.lat + 2 * r * math.cos(h) / FT_DEG, S.lon + 2 * r * math.sin(h) / (FT_DEG * S.k));
        if (e == nil or e2 == nil) continue;
        if (e + e2 < be) { be = e + e2; best = sd; }
    }
    return best;
}
var GIVE_UP_FT = 800;            # a site past its circle whose path meets the ground given up over this height
var COMMIT_FT = 1000;            # under this nothing changes: a change on a final at 250 ft threw the guidance back to a new path
var TOUCH_FT = 300;
var KS2 = [1, 1, -1, -1];        # the paths: the circle's side, +1 right
var KS1 = [1, -1, -1, 1];        # and the first turn's way

var num = func(p, d = 0) { var x = getprop(p); x == nil ? d : x; };
var log = func(s) logprint(LOG_INFO, "MV-22B ENG OUT site: ", s);
var wrap = func(a) { return math.atan2(math.sin(a), math.cos(a)); }
var mx = func(a, b) { return a > b ? a : b; }    # (math.max and math.min are Nasal, 4 times the cost)
var mn = func(a, b) { return a < b ? a : b; }
var ab = func(a) { return a < 0 ? -a : a; }
# a point d_m from (lat, lon) on crs deg, in the flat frame the flight model's guidance flies
var move = func(lat, lon, crs, d_m) {
    var d = d_m * M2FT;
    return [lat + d * math.cos(crs * D2R) / FT_DEG, lon + d * math.sin(crs * D2R) / (FT_DEG * math.cos(lat * D2R))];
}
var elev_m = func(lat, lon) { var g = geodinfo(lat, lon); return g == nil ? nil : g[0]; }

# ---------------------------------------------------------------- the landing's needs at this weight
var W = [];
var TAB = {};
var FLARE = [0, 0];
var TB = [];            # the turn table's banks, deg
var TB_DOWN = [];       # and down from the steepest
var TH = [];            # its ft per 180 deg, a row a weight
var load = func {
    var n = props.globals.getNode(DATA);
    if (n == nil) return 0;
    W = [];
    foreach (var c; n.getChildren("w")) append(W, c.getValue());
    foreach (var t; ["h50", "td", "roll-pavement", "roll-grass", "roll-crop", "err-lo", "err-hi", "td-kt"]) {
        TAB[t] = [];
        foreach (var c; n.getChildren(t)) append(TAB[t], c.getValue());
    }
    FLARE = [num(DATA ~ "flare-lo"), num(DATA ~ "flare-hi")];
    TB = [];
    foreach (var c; n.getChildren("turn-bank")) append(TB, c.getValue());
    TB_DOWN = [];
    for (var i = size(TB) - 1; i >= 0; i -= 1) append(TB_DOWN, TB[i]);
    TH = [];
    foreach (var c; n.getChildren("turn-180")) append(TH, c.getValue());
    return size(W) > 1 and size(TH) == size(W) * size(TB);
}
var at = func(t, w) {
    var ys = TAB[t];
    if (w <= W[0]) return ys[0];
    for (var i = 1; i < size(W); i += 1)
        if (w <= W[i]) return ys[i - 1] + (ys[i] - ys[i - 1]) * (w - W[i - 1]) / (W[i] - W[i - 1]);
    return ys[size(ys) - 1];
}
var NW = {};            # the needs at this weight
var TROW = [];          # the turn table's row at this weight
var HD = [];            # the headwind along each of the strips' courses, kt (- tail)
var LEN = [];           # and the strip each takes on level ground, ft
var floor_ = 1000;      # the floor of a margin within reach: the flare's measured scatter short and the reach's error low
# the headwind along a course, kt (- tail)
var head = func(crs) { return num("/environment/wind-speed-kt") * math.cos((num("/environment/wind-from-heading-deg") - crs) * D2R); }
var weigh = func {
    var w = num(JSB ~ "inertia/weight-lbs");
    var hi = at("err-hi", w); var lo = at("err-lo", w);
    NW = {vt: at("td-kt", w), lo: FLARE[0] - mx(0, hi), hi: FLARE[1] + mx(0, -lo), h50: at("h50", w), td: at("td", w),
          pavement: at("roll-pavement", w), grass: at("roll-grass", w), crop: at("roll-crop", w), rough: at("roll-crop", w)};
    floor_ = -FLARE[0] + mx(0, -lo);
    var nb = size(TB); var nw = size(W);
    TROW = setsize([], nb);
    var a = 0; var f = 0;
    if (w > W[0]) {
        for (var i = 1; i < nw; i += 1) {
            a = i - 1;
            if (w <= W[i]) { f = (w - W[a]) / (W[i] - W[a]); break; }
            f = 1;
        }
    }
    var b = a + 1 < nw ? a + 1 : a;
    for (var k = 0; k < nb; k += 1) TROW[k] = TH[a * nb + k] + (TH[b * nb + k] - TH[a * nb + k]) * f;
    HD = setsize([], HEADINGS); LEN = setsize([], HEADINGS);
    for (var k = 0; k < HEADINGS; k += 1) {
        HD[k] = head(k * 360 / HEADINGS);
        var nd = needs("crop", HD[k]);        # (the longer run: the ground known only along the strip)
        LEN[k] = nd[1] - nd[0] + nd[3] - nd[2] + nd[4];
    }
}
# [lo, hi, h50, td, roll], ft: the flare's start within lo..hi of its point (the guidance's scatter widened by the
# reach's error: + long, the aircraft arriving low and short), the wheels under 50 ft and on the ground past the
# flare's start, the run braked to a stop on this ground; into a headwind of hd kt (- tail): the flare's way over the
# ground by its ground speed's share (from the glide's 200 KCAS to the touchdown's), the run by its square (from the
# touchdown's); on ground falling along the landing (slope < 0) both longer by FALL_K
var needs = func(ground, hd = 0, slope = 0) {
    var ks = 1 + FALL_K * (slope < 0 ? -slope : 0);
    var a = 1 - hd / ((200 + NW.vt) / 2);
    var kf = (a > 0.2 ? a : 0.2) * ks;
    var b = 1 - hd / NW.vt;
    b = b > 0.2 ? b : 0.2;
    return [NW.lo, NW.hi, NW.h50 * kf, NW.td * kf, NW[ground] * b * b * ks];
}

# ---------------------------------------------------------------- the aircraft's state, read once a tick
var S = {lat: 0, lon: 0, trk: 0, k: 1, wN: 0, wE: 0, va: 100, windy: 0, phc: 44, rate: 0.26, vgt: 300, Lf0: 5000, kt0: 4000, ktc: 4000,
         below: 0, rn: 1, ldu: 7, reach0: 0, lag: 0, vt: 100, R0: 2000, hb0: 44, kt: 4000, now: 0, adv: 0};
var turn_cost = func(bank) {
    # the turn's cost in the glide's way per radian at the bank: the weight's row of the table, as the guidance's
    var n = size(TB); var y = TROW[0];
    if (bank >= TB[n - 1]) y = TROW[n - 1];
    elsif (bank > TB[0])
        for (var i = 1; i < n; i += 1)
            if (bank <= TB[i]) { y = TROW[i - 1] + (TROW[i] - TROW[i - 1]) * (bank - TB[i - 1]) / (TB[i] - TB[i - 1]); break; }
    return y * S.ldu / PI;
}
var read_state = func {
    S.lat = num("/position/latitude-deg"); S.lon = num("/position/longitude-deg");
    S.trk = math.atan2(num("/velocities/speed-east-fps"), num("/velocities/speed-north-fps"));
    S.k = math.cos(S.lat * D2R);
    S.wN = num(JSB ~ "atmosphere/total-wind-north-fps"); S.wE = num(JSB ~ "atmosphere/total-wind-east-fps");
    S.windy = S.wN * S.wN + S.wE * S.wE > 0.01;
    S.vt = mx(100, num(JSB ~ "velocities/vt-fps")); S.va = S.vt;
    # (the circle at its bank: the radius, the final's roll out and the cost from the guidance's own at its present)
    S.phc = num(JSB ~ "ap/eo-site-hac-bank-deg", 44);
    S.rate = num(JSB ~ "ap/eo-phi-rate-dps", 15) * D2R;
    S.vgt = math.sqrt(mx(1, num(JSB ~ "ap/eo-site-r-ft", 4000) * G0 * math.tan(S.phc * D2R)));
    S.Lf0 = num(JSB ~ "ap/eo-site-final-full-ft", 5000);
    S.kt0 = num(JSB ~ "ap/eo-site-turn0-ft-rad", 4000);
    S.ktc = num(JSB ~ "ap/eo-site-turn-ft-rad", 4000);
    S.below = num("/position/ground-elev-ft");
    S.agl = num(JSB ~ "hh/h-wheels-ft");
    var va = num(JSB ~ "ap/eo-nb-after-kt", 170) / mx(1, num(JSB ~ "ap/eo-vg-kt", 200));
    var vgs = num(JSB ~ "ap/eo-vg-kt", 200) * 1.6878;
    S.after_ft = num(JSB ~ "ap/eo-nb-used") > 0.5 ? 0 : vgs * vgs * (1 - va * va) / (2 * G0);
    S.nb_floor = num(JSB ~ "ap/eo-nb-floor-ft", 1e9);
    S.rn = num(JSB ~ "ap/eo-brake-nom-ratio", 1);
    S.rmin = num(JSB ~ "ap/eo-brake-min-ratio", 1);
    S.ldu = num(JSB ~ "ap/eo-ld-up", 7);
    S.reach0 = num(JSB ~ "ap/eo-reach-nm") * 6076.12;
    S.lag = num(JSB ~ "ap/roll-lag-s");
    S.R0 = num(JSB ~ "ap/eo-site-r0-ft", 2000);
    S.hb0 = num(JSB ~ "ap/eo-hac-frac", 0.8) * num(JSB ~ "ap/eo-bank-lim-deg", 55);
    S.kt = turn_cost(S.hb0);
    S.adv = S.vt * (num(JSB ~ "ap/eo-bank-lim-deg", 50) * D2R / (2 * S.rate) + S.lag);
    S.now = num("/sim/time/elapsed-sec");
}

# ---------------------------------------------------------------- the paths to a line and their margins
# (each straight leg's way through the air, as the flight model's: its ground speed's share in the wind along and
# across it; left out, the finder's margins were the flight model's less 1700 ft in 14 kt)
var air = func(t) {
    if (!S.windy) return 1;
    var cs = math.cos(t); var sn = math.sin(t);
    var al = S.wN * cs + S.wE * sn; var ac = S.wE * cs - S.wN * sn;
    var q = S.va * S.va - ac * ac;
    var b = al + math.sqrt(q > 1 ? q : 1);
    return S.va / (b > 30 ? b : 30);
}
# the last geom's: the circle's radius, the final, the turn's cost, the circle's bank, the reach, the slack the brake holds
var gR = 0; var gLf = 0; var gkt = 0; var gph = 0; var greach = 0; var gslack = 0; var glead = 0;
var gcc = 0; var gpsi = 0; var gsp = 0; var gcp = 0;
var dC = 0; var dL = 0; var dPhi = 0; var dA1 = 0; var dA2 = 0;       # dub's
# by path: found, margin, way over the ground, tangent, its course, first turn, circle's turn, straight on first, where the turn starts, cost
var OV = [0, 0, 0, 0]; var OM = [0, 0, 0, 0]; var OP = [0, 0, 0, 0]; var OL = [0, 0, 0, 0]; var OPH = [0, 0, 0, 0];
var OA1 = [0, 0, 0, 0]; var OA2 = [0, 0, 0, 0]; var OS0 = [0, 0, 0, 0]; var OXA = [0, 0, 0, 0]; var OYA = [0, 0, 0, 0]; var OC = [0, 0, 0, 0];
var fbM = 0; var fbP = 0; var fbS = 1; var fbLam = 0; var fbTh = 0;    # none of the four: the point to the circle
# the path from (xa, ya) with the first turn s1 and the circle on side s2: 1 if there is one, dC its cost in reach
var gR0 = 0;
var dub = func(s1, s2, xa, ya) {
    var R0 = gR0;
    var dx = -gLf - (xa - s1 * R0 * gsp);
    var dy = s2 * gR - (ya + s1 * R0 * gcp);
    var rho = s2 * gR - s1 * R0;
    var d2 = dx * dx + dy * dy;
    if (d2 + 2 * TOUCH_FT * ab(rho) <= rho * rho) return 0;      # (touching, a path of no leg: genv22 EO_TOUCH_FT)
    var L = d2 > rho * rho ? math.sqrt(d2 - rho * rho) : 0;
    var phi = math.atan2(dy, dx) - math.atan2(rho, L > 1 ? L : 1);
    var a1 = s1 * (phi - gpsi);
    a1 -= TWO_PI * math.floor(a1 / TWO_PI);
    if (a1 > TWO_PI - glead) a1 = 0;
    var a2 = -s2 * phi;
    a2 -= TWO_PI * math.floor(a2 / TWO_PI);
    if (a2 > TWO_PI - glead) a2 = 0;
    dC = L * air(gcc + phi) + S.kt0 * a1 + gkt * a2;
    dL = L; dPhi = phi; dA1 = a1; dA2 = a2;
    return 1;
}
# the four paths to c with the circle at hb deg and the final no longer than fin (0: the full one); the best's index, -1 if none
var geom = func(c, hb, fin) {
    var rate = S.rate; var vgt = S.vgt;
    var R0 = hb >= TIGHT_HB ? S.R0 * TIGHT_R0 : S.R0; gR0 = R0;
    var ph = hb * D2R; var tn = math.tan(ph);
    gph = ph;
    gR = vgt * vgt / (G0 * tn);
    gLf = S.Lf0 + vgt * (hb - S.phc) * D2R / rate;
    if (fin > 0 and fin < gLf) gLf = fin;
    gkt = hb == S.hb0 ? S.kt : turn_cost(hb);
    # (the reach the flight model gives is from the ground under the aircraft; to the site, from the site's elevation:
    # over LOWI's valley side, 925 ft over the runway, the runway's read 1.1 NM short of its own. At the brake's nominal
    # ratio, as the guidance's margin; retracted, the clean glide's more, the slack)
    var se = contains(c, "elev") ? c.elev : elev_at(c.lat, c.lon);
    greach = (S.reach0 + (se == nil ? 0 : (S.below - se) * S.ldu)) * S.rn;
    if (hb >= TIGHT_HB) greach += S.after_ft * S.ldu * S.rn;      # (as the guidance will read it then, from the speed after)
    gslack = greach * (1 / S.rn - 1);
    glead = G0 * tn / S.vt * (ph / (2 * rate) + S.lag);
    var dN = (S.lat - c.lat) * FT_DEG;
    var dE = (S.lon - c.lon) * FT_DEG * S.k;
    var cc = c.crs * D2R; gcc = cc;
    var si = math.sin(cc); var co = math.cos(cc);
    var x = dE * si + dN * co; var y = dE * co - dN * si;
    gpsi = wrap(S.trk - cc); gsp = math.sin(gpsi); gcp = math.cos(gpsi);
    var Lair = gLf * air(cc); var airt = air(S.trk);
    # (from where it can be turning: the bank takes its time, straight on meanwhile. Planned from here, a path that only
    # existed for an instant turn was chosen, its leg none, and gone a frame later: the guidance fell back on another,
    # 29 000 ft longer, and good sites were given up for that, again and again)
    var adv = S.adv; var cadv = adv * airt;
    x += adv * gcp; y += adv * gsp;
    var best = -1; var bm = -1e18;
    for (var k = 0; k < 4; k += 1) {
        var s2 = KS2[k]; var s1 = KS1[k];
        var ok = dub(s1, s2, x, y);
        var s0 = adv; var cost = dC + cadv; var L = dL; var phi = dPhi; var a1 = dA1; var a2 = dA2;
        if (s1 != s2) {
            # (a cross path whose circles overlap: straight on first, to where they part)
            var ddx = x - s1 * R0 * gsp + gLf; var ddy = y + s1 * R0 * gcp - s2 * gR;
            var ud = gcp * ddx + gsp * ddy;
            var rr = gR + R0;
            var q = ud * ud + rr * rr - (ddx * ddx + ddy * ddy);
            var sq = q > 0 ? -ud + math.sqrt(q) + 1 : 0;
            if (sq > 2 and dub(s1, s2, x + sq * gcp, y + sq * gsp) and (!ok or sq * airt + dC + cadv < cost)) {
                ok = 1; s0 = sq + adv; cost = sq * airt + dC + cadv; L = dL; phi = dPhi; a1 = dA1; a2 = dA2;
            }
        }
        OV[k] = ok;
        if (!ok) continue;
        var m = greach - (cost + Lair);
        OM[k] = m; OC[k] = cost; OL[k] = L; OPH[k] = phi; OA1[k] = a1; OA2[k] = a2; OS0[k] = s0;
        OXA[k] = x + (s0 - adv) * gcp; OYA[k] = y + (s0 - adv) * gsp;
        OP[k] = s0 + R0 * a1 + L + gR * a2 + gLf;
        if (m > bm) { bm = m; best = k; }
    }
    if (best < 0) {
        var sg = y < 0 ? -1 : 1;
        var u = x + gLf; var w = y - sg * gR;
        var d = math.sqrt(u * u + w * w);
        var lamT = math.atan2(w, u) + sg * math.acos(mn(1, gR / (d > 1 ? d : 1)));
        var thT = sg * (-sg * PI / 2 - lamT);
        thT -= TWO_PI * math.floor(thT / TWO_PI);
        if (thT > TWO_PI - glead) thT = 0;
        var q = d * d - gR * gR;
        var L1 = math.sqrt(q > 0 ? q : 0);
        var turn0 = ab(wrap(cc + lamT + sg * PI / 2 - S.trk));
        fbM = greach - (L1 * air(cc + lamT + sg * PI / 2) + S.kt0 * turn0 + gkt * thT + Lair);
        fbP = L1 + gR * thT + gLf; fbS = sg; fbLam = lamT; fbTh = thT;
    }
    return best;
}
# what the nacelle brake can spend on a path P, in reach at the nominal: flown at NB_LD for all of it, or for the height there is
# over its floor, the speed back after that (genv22 ap/eo-nb-floor-ft)
var NB_LD = 2.6;                 # its glide's ratio, the gear coming: 2.3 down (fdm/deadstick.py, 45 deg, 150 KCAS, flaperons 40), 2.0-2.3 flown
# The last resort, every plan too high: the turns tight, at 170 KCAS, the speed the guidance flies after the brake, the radius 2500
# ft for 4800, a turn's height the same at any speed (fdm/deadland.py turns at 150-200 KCAS: 1400-1900 ft per 180 deg at 42-55
# deg). So not a lap, which still takes all the height there is, but the way round to a field the wide turn could not get to,
# over a valley's side. Offered everywhere they took flights off stretches that were landing on their marks. Sent as the
# circle at the bank that gives the radius at the glide's speed; ap/eo-site-slow holds the speed to it
var TIGHT_HB = 59;
var TIGHT_R0 = 0.72;             # (170 / 200)^2 of the first turn's radius
var NB_LAG_FT = 2000;            # the way flown while the nacelles come up and the speed back, nothing yet spent
var spend_nb = func(P) {
    var hb = S.agl - S.nb_floor;
    if (hb <= 0) return 0;
    var d = hb * NB_LD;
    d = (d < P ? d : P) - NB_LAG_FT;
    return d > 0 ? d * (S.ldu * S.rn / NB_LD - 1) : 0;
}
# 4: within reach at the brake's nominal by the floor, what is over it spendable; 3: clean by the floor, the brake in (a
# stretch); 2: clean, under the floor; 1: high, more over the floor than can be spent, the landing long by about what is
# left; 0: short. (Reaching the field before flying past it: a final cut from a loop to a straight-in read 22 700 ft
# high 1.3 NM from LOWI, ranked over the fields as a runway, and ran 27 000 ft on into the town)
# 2.5 (NB): high, what is over spent by the nacelle brake as well: after a stretch, which has landed where it was pointed every
# time, and before all else: the steep way down is for when nothing fits
var NB = 2.5;
var tier_of = func(m, slack, sp, lap, spn = 0) {
    if (m >= floor_) {
        var q = m - floor_;
        var r = q > lap ? q - lap : q;
        return r <= sp ? 4 : r <= sp + spn ? NB : 1;
    }
    return m + slack >= floor_ ? 3 : m + slack >= 0 ? 2 : 0;
}
# (a lap's cost, the last geom's: none for the tight turns, whose lap would read a field straight under the aircraft as fitting)
var glap = func { return gR0 < S.R0 ? 1e9 : TWO_PI * gkt; }
var tight_tier = func(t) { return t == 4 ? 2.8 : t == 3 ? 2.7 : t; }
var tier_k = func(k) {       # of the last geom's path k
    var sp = greach * (1 - S.rn);
    var t = k < 0 ? tier_of(fbM, gslack, sp, glap()) : tier_of(OM[k], gslack, sp, glap(), spend_nb(OP[k]));
    return gR0 < S.R0 ? tight_tier(t) : t;
}
var margin_k = func(k) { return k < 0 ? fbM : OM[k]; }
# no water on a strip nor past its end, read every DRY_STEP_FT along it: the cells are 150 m apart, and the Inn runs narrower.
# Once a strip, when it first reaches the ranking
var DRY_STEP_FT = 130;
var DRY_PAST_FT = 1200;
var dry_strip = func(c) {
    var la = c.ends[0]; var lo = c.ends[1];
    var cs = math.cos(c.crs * D2R); var sn = math.sin(c.crs * D2R); var k = math.cos(la * D2R);
    var all = c.len + DRY_PAST_FT;
    for (var d = 0; d <= all; d += DRY_STEP_FT) {
        var g = geodinfo(la + d * cs / FT_DEG, lo + d * sn / (FT_DEG * k));
        if (g != nil and g[1] != nil and !g[1].solid) return 0;
    }
    return 1;
}
# a line's place among the lines, the lower the better, from its best plan: the ranking's order (order) as a number
var line_key = func(c, t, m, slack, fin) {
    if (t == 0) return 4e10 - (m + slack);
    if (t == 1) return 3e10 + m;
    if (t == NB) return (4 - t) * 1e10 + m;      # (by what it has to spend alone: it lands the nearer its mark the less)
    var hq = int((HEAD_KT * 2 - mn(c.head, HEAD_KT * 2)) / HEAD_KT);
    var sub = (((c.spare < SPARE_FT) * 4 + KIND_RANK[c.kind]) * 8 + (hq < 7 ? hq : 7)) * 4 + (contains(c, "slope") and c.slope > 0.001) * 2 + (fin > 0);
    return (4 - t) * 1e10 + sub * 1e7 + (t == 4 ? m : -m);
}
# priced: what its plans could come to from the two ends of the options, nothing kept but the key; energy_plans finds
# the one that fits for the few ranked. (Walking the options here, most lines over open ground took 6 sets of paths)
var price = func(c) {
    if (contains(c, "dry") and !c.dry) { c.key = 2e18; return; }
    var k = geom(c, S.hb0, 0);
    var m = margin_k(k); var t = tier_k(k); var sl = gslack; var fin = 0;
    if (t == 1 or t == NB) {
        # (high: fits on the widest circle; or is under the floor there and some circle between may: that one not yet proved,
        # after those that are. Priced as fitting, over LOWI straight-ins 16 000 ft high took the ranking's places, and
        # the landings that did fit were never looked at)
        if (TB[0] < S.hb0 - 0.5) {
            k = geom(c, TB[0], 0);
            if (tier_k(k) == 4) t = 4;
            elsif (margin_k(k) < floor_) t = 2;
        }

    } elsif (t < 4) {
        # (low: the same with the shortest final; still short there, no other will do)
        k = geom(c, S.hb0, FINALS[size(FINALS) - 1]);
        var tj = tier_k(k);
        if (tj == 1) tj = 2;           # (high there, some final between may fit: not yet proved)
        if (tj > t) { t = tj; m = margin_k(k); sl = gslack; fin = 1; }
    }
    c.key = line_key(c, t, m, sl, fin);
}
# the plans to c, the most margin first: [margin, way, the circle's side, lamT, the circle's turn, R, Lf, the path, slack,
# the circle's bank, the final cut to, spendable, a lap's cost]; the path [cost, first turn's way, tangent, its course,
# first turn, circle's turn, x, y, psi, R0, straight on first, k], nil for the point to the circle
var plans = func(c, hb = nil, fin = 0) {
    var b = hb == nil ? S.hb0 : hb;
    var best = geom(c, b, fin);
    var sp0 = greach * (1 - S.rn); var lap = glap();
    if (best < 0) return [[fbM, fbP, fbS, fbLam, fbTh, gR, gLf, nil, gslack, b, fin, sp0, lap, 0]];
    var out = [];
    for (var k = 0; k < 4; k += 1) {
        if (!OV[k]) continue;
        var o = [OM[k], OP[k], KS2[k], 0, OA2[k], gR, gLf,
                 [OC[k], KS1[k], OL[k], OPH[k], OA1[k], OA2[k], OXA[k], OYA[k], gpsi, gR0, OS0[k], k],
                 gslack, b, fin, sp0, lap, spend_nb(OP[k])];
        var i = size(out);
        append(out, o);
        while (i > 0 and out[i - 1][0] < o[0]) { out[i] = out[i - 1]; i -= 1; }
        out[i] = o;
    }
    return out;
}
var plan_tier = func(pl, margin = nil) {
    var t = tier_of(margin == nil ? pl[0] : margin, pl[8], pl[11], pl[12], pl[13]);
    return pl[9] >= TIGHT_HB ? tight_tier(t) : t;
}
# (the options for the energy, the best tier first. High, the circle wider: the shallower banks of the turn table, the
# tightest that fits: the circle spends only where it turns, and wider than the way to the site its tangents come and
# go: over LOWI at 15 deg, 15 600 ft across a field 2 NM out, the path flown lost its tangent, read 95 000 ft short, and
# the S-turns, their room the leg less the circle's, had none. Low, the Shuttle's minimum entry point: the final cut, the
# longest of FINALS that fits, the flare no worse for it, deadland.py finals)
var energy_plans = func(c) {
    var base = plans(c);
    var extra = [];
    var t0 = plan_tier(base[0]);
    if (t0 == 1 or t0 == NB) {
        foreach (var b; TB_DOWN) {
            if (b >= S.hb0 - 0.5) continue;
            var pl = plans(c, b);
            if (pl[0][0] < floor_) break;
            if (plan_tier(pl[0]) == 4) { extra = pl; break; }
        }
    } elsif (t0 < 4) {
        foreach (var Lj; FINALS) {
            var pl = plans(c, nil, Lj);
            var tj = plan_tier(pl[0]);
            if (tj == 4) { extra = pl; break; }
            if (tj > plan_tier(size(extra) ? extra[0] : base[0])) extra = pl;
        }
    }
    return sort(extra ~ base, func(a, b) {
        var ta = plan_tier(a); var tb = plan_tier(b);
        return ta != tb ? tb - ta : b[0] - a[0];
    });
}

# ---------------------------------------------------------------- the terrain under the path
# A site's planned path against the ground under it: the lowest the guidance flies it (from the site back: the site's
# plane at the flare's height, the way at the glide's ratio and the turns at their cost, no higher than the aircraft's
# glide from here) no less than the flare's height over the ground (lower, ENG OUT would flare there, over that
# ground), easing to none at the flare's point over the final. Out of reach, it comes down where its glide meets that
# height: short of the site at the site's own level, or blocked. (Without it the finder ranked by energy alone, as over
# flat ground: over LOWI its circle to runway 08, 2.7 km across, swung over the Nordkette's slope, and ENG OUT flew
# into it)
var reads = 0;          # the land cover reads left to whatever looks at the ground
# the ground under a point, ft, from its cell; a cell not read yet read now while the reads last; nil not known
var elev_at = func(lat, lon) {
    var i = math.round((lon - origin[1]) * KI); var j = math.round((lat - origin[0]) * KJ);
    var k = (i + OFF) * KEY + j + OFF;
    var v = grid[k];
    if (v == nil) {
        if (reads <= 0) return nil;
        reads -= 1;
        sample(i, j, 1);
        v = grid[k];
        if (v == nil) return nil;
    }
    return v - int(v / KMUL) * KMUL - EOFF;
}
# the glide's state for the paths: the wheels' height above sea level less the speed the glide must still gain (diving
# for it the aircraft comes down to its energy); the glide's ratio over the reach (the gear's part in it, the brake at
# its nominal); the height a radian of turning takes; the flare's height; the radius of the turn onto the leg at ENG
# OUT's bank
var glide_state = func {
    var vt = num(JSB ~ "velocities/vt-fps");
    # (after the brake or on a tight plan the speed the guidance holds then)
    var vg = num(JSB ~ (num(JSB ~ "ap/eo-nb-used") > 0.5 ? "ap/eo-nb-after-kt" : "ap/eo-vg-kt"), 200) * 1.6878 / math.sqrt(mx(0.2, num(JSB ~ "atmosphere/sigma", 1)));
    var fl = num(JSB ~ "ap/eo-flare-ft", 250);
    var ld = num(JSB ~ "ap/eo-reach-nm") * 6076.12 / mx(100, num(JSB ~ "ap/eo-energy-ft") - fl);
    var v_ = mx(100, vt);
    var hw = S.below + num(JSB ~ "hh/h-wheels-ft");
    return {h0: hw + mn(0, (vt * vt - vg * vg) / (2 * G0)), hw: hw,
            ld: mx(1, ld * S.rn), hpr: S.ktc / mx(1, S.ldu * S.rn), hpr0: S.kt0 / mx(1, S.ldu), fl: fl,
            r0: v_ * v_ / (G0 * math.tan(num(JSB ~ "ap/eo-bank-lim-deg", 50) * D2R))};
}
# the path's points, every cell's width, in shared buffers: lat, lon, the straight way, the turning (rad): the turns' way
# not counted as way, their cost per radian holding the glide round them
var PN = setsize([], 1024); var PE = setsize([], 1024); var PS = setsize([], 1024); var PA = setsize([], 1024);
var pn = 0; var psc = 0; var pac = 0; var pPn = 0; var pPe = 0; var pco = 1; var psi_ = 0; var pkk = 1;
var padd = func(n, e) {
    if (pn >= size(PN)) { var m = size(PN) * 2; PN = setsize(PN, m); PE = setsize(PE, m); PS = setsize(PS, m); PA = setsize(PA, m); }
    PN[pn] = S.lat + n / FT_DEG; PE[pn] = S.lon + e / (FT_DEG * pkk); PS[pn] = psc; PA[pn] = pac;
    pn += 1;
}
var pline = func(an, ae, bn, be) {
    var d = math.sqrt((bn - an) * (bn - an) + (be - ae) * (be - ae));
    var n = int(d / CELL_FT) + 1;
    for (var i = 1; i <= n; i += 1) { psc += d / n; padd(an + (bn - an) * i / n, ae + (be - ae) * i / n); }
}
var pxn = func(x, y) { return pPn + x * pco - y * psi_; }      # the line's frame to north and east of the aircraft
var pxe = func(x, y) { return pPe + x * psi_ + y * pco; }
var parc = func(cx, cy, R, s, h0, a) {         # from h0 round a rad on the circle at (cx, cy), the way s
    var n = int(a * R / CELL_FT) + 1;
    for (var i = 1; i <= n; i += 1) {
        var hh = h0 + s * a * i / n;
        pac += a / n;
        var x = cx + s * R * math.sin(hh); var y = cy - s * R * math.cos(hh);
        padd(pxn(x, y), pxe(x, y));
    }
}
var path_pts = func(c, r0, flown = nil) {
    var pl = c.plan;
    var sg = pl[2]; var lamT = pl[3]; var thT = pl[4]; var R = pl[5]; var Lf = pl[6];
    if (flown != nil) {          # (round its circle from here, or its final, as the guidance flies them)
        sg = num(JSB ~ "ap/eo-site-s", sg); R = num(JSB ~ "ap/eo-site-r-ft", R); Lf = num(JSB ~ "ap/eo-site-final-ft", Lf);
        lamT = math.atan2(num(JSB ~ "ap/eo-site-y-ft") - sg * R, num(JSB ~ "ap/eo-site-x-ft") + Lf);
        thT = flown > 2.5 ? 0 : num(JSB ~ "ap/eo-site-arc-left-rad");
    }
    var cc = c.crs * D2R;
    pkk = S.k; pco = math.cos(cc); psi_ = math.sin(cc);
    pPn = (c.lat - S.lat) * FT_DEG; pPe = (c.lon - S.lon) * FT_DEG * pkk;
    pn = 0; psc = 0; pac = 0;
    var dub_ = flown == nil ? pl[7] : nil;
    if (dub_ != nil) {           # (the Dubins path: its turn, the tangent, the circle, the final)
        var s1 = dub_[1]; var phi = dub_[3]; var a1 = dub_[4]; var a2 = dub_[5]; var xa = dub_[6]; var ya = dub_[7];
        var psi = dub_[8]; var R0 = dub_[9]; var s0 = dub_[10];
        var c1x = xa - s1 * R0 * math.sin(psi); var c1y = ya + s1 * R0 * math.cos(psi);
        var qn = 0; var qe = 0;
        if (s0 > 0) { qn = pxn(xa, ya); qe = pxe(xa, ya); pline(0, 0, qn, qe); }
        parc(c1x, c1y, R0, s1, psi, a1);
        if (pn) { qn = (PN[pn - 1] - S.lat) * FT_DEG; qe = (PE[pn - 1] - S.lon) * FT_DEG * pkk; }
        var tx = -Lf + sg * R * math.sin(phi); var ty = sg * R - sg * R * math.cos(phi);
        pline(qn, qe, pxn(tx, ty), pxe(tx, ty));
        parc(-Lf, sg * R, R, sg, phi, a2);
        pline(pxn(-Lf, 0), pxe(-Lf, 0), pxn(0, 0), pxe(0, 0));
        return pn;
    }
    var h = S.trk; var hl = cc + lamT + sg * PI / 2;
    var dt = flown != nil ? 0 : wrap(hl - h);
    var sgn = dt < 0 ? -1 : 1;
    var cn = sgn * r0 * -math.sin(h); var ce = sgn * r0 * math.cos(h);
    var n = int(ab(dt) * r0 / CELL_FT) + 1;
    var p0 = 0; var p1 = 0;
    for (var i = 1; i <= n; i += 1) {
        var hh = h + dt * i / n;
        p0 = cn + sgn * r0 * math.sin(hh); p1 = ce - sgn * r0 * math.cos(hh);
        pac += ab(dt) / n; padd(p0, p1);
    }
    var tx = -Lf + R * math.cos(lamT); var ty = sg * R + R * math.sin(lamT);
    if (flown == nil or flown <= 2.5) pline(p0, p1, pxn(tx, ty), pxe(tx, ty));
    if (flown != nil and flown > 2.5) { p0 = 0; p1 = 0; }
    n = int(thT * R / CELL_FT) + 1;
    for (var i = 1; i <= n; i += 1) {
        var l = lamT + sg * thT * i / n;
        pac += thT / n;
        var x = -Lf + R * math.cos(l); var y = sg * R + R * math.sin(l);
        padd(pxn(x, y), pxe(x, y));
    }
    if (flown != nil and flown > 2.5) pline(p0, p1, pxn(0, 0), pxe(0, 0));
    else pline(pxn(-Lf, 0), pxe(-Lf, 0), pxn(0, 0), pxe(0, 0));
    return pn;
}
var path_clear = func(c, gs, flown = nil) {
    var n = path_pts(c, gs.r0, flown);
    if (!n) return 1;
    var St = PS[n - 1]; var At = PA[n - 1];
    var ce = contains(c, "elev") ? c.elev : elev_at(c.lat, c.lon);
    if (ce == nil) return 0;
    var Lf = c.plan[6]; Lf = Lf > 1 ? Lf : 1;
    # (a stretch, short at the brake's nominal and within reach clean, is flown all the way, the brake in: looked at along
    # all of it at the clean glide. Taken as short, the look stopped where the nominal glide first met the valley floor, read
    # clear, and the slope further along the path, 3000 ft over that glide, was never seen)
    var stretch = c.margin < 0 and c.margin + c.plan[8] >= 0;
    var within = c.margin >= 0 or stretch;
    var ld = stretch ? gs.ld / S.rn : gs.ld; var hpr = stretch ? gs.hpr * S.rn : gs.hpr; var fl = gs.fl; var h0 = gs.h0;
    var surplus = h0 - St / ld - At * hpr - (ce + fl);
    if (surplus < 0) surplus = 0;
    var nb = S.agl > S.nb_floor; var Rp = c.plan[5];
    # (on its way down the flare can begin at up to twice its height, the faster the descent: at 250 ft, clear by this measure, a
    # hill's crest set one off 2 miles from the field)
    var flc = nb and surplus > 0 ? 2 * fl : fl;
    for (var q = 0; q < n; q += 1) {
        var e = elev_at(PN[q], PE[q]);
        if (e == nil) return 0;                  # (ground not known: the path not known clear)
        var s = PS[q]; var a = PA[q];
        # (within reach, the surplus spent along the way, the height coming down evenly to the site's: taken as spent at once,
        # the lowest it could fly, a path 9700 ft high half a mile from a field read as meeting the ground, and it was left
        # for a field 11 700 ft out of reach)
        var hp = h0 - s / ld - a * hpr;
        # (with the nacelle brake to spend it, as it goes: down at its ratio first, until it meets the glide up from the site)
        if (within and surplus > 0) {
            if (nb) {
                # (round the turns by the way too, not their cost per radian: slow on the brake a wide circle takes three times as
                # long, 2100 ft a radian for 780, and one read clear met the Nordkette 6000 ft from its field at 76 ft/s)
                var h1 = h0 - (s + a * Rp) / NB_LD; var h2 = ce + fl + (St - s) / ld + (At - a) * hpr;
                hp = h1 > h2 ? h1 : h2;
            } else hp -= surplus * (St > 1 ? s / St : 1);
        }
        var w = (St - s) / Lf;
        if (hp - e < flc * (w < 1 ? w : 1)) return within ? 0 : (e <= ce + fl);
    }
    return 1;
}
# On its final: the glide from here on along the track to where it comes down, past the flare's point too, the landing
# held from there on the site's strip. (Looked at only as far as the point, a final 21 000 ft high read clear over the
# field and on up the valley until the slope was under it)
var line_clear = func(c, gs) {
    var cs = math.cos(S.trk); var sn = math.sin(S.trk); var x0 = num(JSB ~ "ap/eo-site-x-ft");
    var ld = gs.ld * S.rmin / S.rn;           # (the brake in)
    var need = NW.td + NW[c.kind == "RWY" ? "pavement" : c.kind];
    # (the nacelle brake's way down first, while there is height over its floor: at the flaperons' ratio alone a final 2500 ft
    # high read as running on past its strip, every second, and the point hopped along the field until the brake was in)
    var hb = S.agl - S.nb_floor; hb = hb > 0 ? hb : 0;
    var dn = hb * NB_LD;
    for (var d = CELL_FT; d < 80000; d += CELL_FT) {
        var e = elev_at(S.lat + d * cs / FT_DEG, S.lon + d * sn / (FT_DEG * S.k));
        if (e == nil) return 0;
        if (gs.hw - (d < dn ? d / NB_LD : hb + (d - dn) / ld) - e < gs.fl) return x0 + d + need <= c.end + NW.hi;
    }
    return 1;
}
# no site clear: the course whose glide (the turn onto it, then straight) keeps the most height over the ground, summed,
# till it comes within the flare's height (the longest glide alone took it over the terrace south of LOWI, 700-900 ft
# up), deg true; the courses walked out together, so that the reads this ranking has reach as far on each. From the
# height there is and the speed there is: the turn is flown first, the dive for the glide's speed after it (genv22
# ap/eo-vg-cmd-kt). Planned with the dive already spent and the turns at the glide's speed, at 160 KCAS 1 NM from the
# Nordkette the way back read as blocked as the way on, and it flew on and into the slope
var oc_last = nil;
var open_course = func(gs) {
    var n = 24; var k = S.k; var h = S.trk; var r0 = gs.r0; var h0 = gs.hw; var hpr = gs.hpr0;
    var d = setsize([], n); var pN = setsize([], n); var pE = setsize([], n); var sc = setsize([], n); var turn = setsize([], n); var live = setsize([], n);
    for (var j = 0; j < n; j += 1) {
        var hc = j * 15 * D2R;
        var dt = wrap(hc - h); var sgn = dt < 0 ? -1 : 1;
        var cn = sgn * r0 * -math.sin(h); var ce = sgn * r0 * math.cos(h);
        var m = int(ab(dt) * r0 / CELL_FT) + 1;
        d[j] = 0; live[j] = 1; sc[j] = 0; turn[j] = ab(dt);
        for (var i = 1; i <= m; i += 1) {
            var hh = h + dt * i / m;
            var e = elev_at(S.lat + (cn + sgn * r0 * math.sin(hh)) / FT_DEG, S.lon + (ce - sgn * r0 * math.cos(hh)) / (FT_DEG * k));
            var hf = h0 - ab(dt) * i / m * hpr;
            if (e == nil or hf - e < gs.fl) { live[j] = 0; break; }
            d[j] += (hf - e) * CELL_FT;
        }
        pN[j] = cn + sgn * r0 * math.sin(hc); pE[j] = ce - sgn * r0 * math.cos(hc);
    }
    for (var i = 0; i < 400; i += 1) {
        var any = 0;
        for (var j = 0; j < n; j += 1) {
            if (!live[j]) continue;
            var hc = j * 15 * D2R;
            var hf = h0 - sc[j] / gs.ld - turn[j] * hpr;
            var e = elev_at(S.lat + pN[j] / FT_DEG, S.lon + pE[j] / (FT_DEG * k));
            if (e == nil or hf - e < gs.fl) { live[j] = 0; continue; }       # (the known ground's end too)
            any = 1;
            d[j] += (hf - e) * CELL_FT; sc[j] += CELL_FT;
            pN[j] += CELL_FT * math.cos(hc); pE[j] += CELL_FT * math.sin(hc);
        }
        if (!any) break;
    }
    # (none known, none: with the reads spent every course scored nothing and the first, north, was flown, at the Nordkette.
    # Alike, the least turn)
    var best = nil; var bd = 0; var bt = 9;
    for (var j = 0; j < n; j += 1) {
        if (d[j] > bd * 1.02 or (d[j] > bd * 0.98 and d[j] > 0 and turn[j] < bt)) { bd = d[j]; bt = turn[j]; best = j * 15; }
    }
    # (the course taken kept unless another is well ahead: re-chosen each second it went from a turn back to straight on)
    if (oc_last != nil and d[int(oc_last / 15)] >= 0.7 * bd and d[int(oc_last / 15)] > 0) best = oc_last;
    oc_last = best;
    return best;
}

# ---------------------------------------------------------------- the lines
var lines = [];         # all the lines found
var fresh = [];         # those not priced yet
var fi = 0;
var pi_ = 0;            # the pricing's place among them
var add_line = func(c) {
    c.key = 1e18; c.clear = 0; c.plan = nil; c.margin = -1e9; c.path = 0;
    append(lines, c); append(fresh, c);
}
var trim_lines = func {
    # (the better half kept, by a sample of the keys: sorting them all took 7 ms; never the one flown, nor those not
    # priced yet)
    var ks = [];
    for (var i = 0; i < size(lines); i += 16) if (lines[i].key < 1e18) append(ks, lines[i].key);
    if (size(ks) < 2) return;
    ks = sort(ks, func(a, b) { return a < b ? -1 : a > b ? 1 : 0; });
    var cut = ks[int(size(ks) / 2)];
    var out = [];
    foreach (var c; lines) if (c.key <= cut or c.key >= 1e18 or c == chosen) append(out, c);
    lines = out; pi_ = 0;
}
var price_tick = func {
    var n = PRICE_PER_TICK;
    while (n > 0 and fi < size(fresh)) { price(fresh[fi]); fi += 1; n -= 1; }
    if (fi >= size(fresh)) { fresh = []; fi = 0; }
    var L = size(lines);
    # (all of them each ranking if they are few; each in its turn if they are many)
    var share = int(L / RANK_TICKS) + 1;
    if (share < n) n = share;
    for (; n > 0 and L > 0; n -= 1) {
        if (pi_ >= L) pi_ = 0;
        price(lines[pi_]); pi_ += 1;
    }
    if (L > MAX_LINES) trim_lines();
}

# ---------------------------------------------------------------- runways
# Each runway end of the airports within range whose length past its displaced threshold holds the landing: the flare's
# point placed for the scatter's earliest touchdown at the threshold (the approach below 50 ft over the airport's own
# clear ground), and on along it by a cell at a time while the length left takes the landing. One end a tick; an end
# whose scenery is not there yet waits and is tried again.
var rw_todo = [];       # [airport, runway id]
var rw_wait = [];
var runways = func(range_nm) {
    var near = findAirportsWithinRange(range_nm);
    if (near == nil) return;
    foreach (var a; near) {
        var apt = airportinfo(a.id);
        if (apt == nil) continue;
        foreach (var id; keys(apt.runways)) append(rw_todo, [a.id, id]);
    }
}
var runway_end = func(aid, id) {
    var r = airportinfo(aid).runways[id];
    if (r.surface == 13 or r.surface == 15) return;                  # water; transparent
    var ground = (r.surface == 1 or r.surface == 2) ? "pavement" : "grass";
    var hd = head(r.heading);
    var nd = needs(ground, hd);
    var lda0 = (r.length - r.threshold) * M2FT;
    var t0 = move(r.lat, r.lon, r.heading, r.threshold);
    var e = move(t0[0], t0[1], r.heading, lda0 * FT2M);
    var ee = elev_m(e[0], e[1]);
    var et0 = elev_m(t0[0], t0[1]);
    if (ee == nil or et0 == nil) { append(rw_wait, [aid, id]); return; }
    var n = int(lda0 * FT2M / CELL_M);
    for (var k = 0; k <= n; k += 1) {
        var lda = lda0 - k * CELL_FT;
        if (lda < nd[1] - nd[0] + nd[4]) break;
        var t = move(t0[0], t0[1], r.heading, k * CELL_M);
        var back = (nd[3] + nd[0]) * FT2M;
        var f = move(t[0], t[1], back > 0 ? r.heading + 180 : r.heading, ab(back));
        var et = k ? elev_m(t[0], t[1]) : et0;
        if (et == nil) continue;
        # (its plane from the scenery's elevations at the aim and the end; past DOWN_MAX and RISE_MAX left out, falling
        # the longer landing held)
        var slope = (ee - et) * M2FT / lda;
        if (slope < -DOWN_MAX or slope > RISE_MAX) continue;
        var ns = needs(ground, hd, slope);
        if (lda < ns[1] - ns[0] + ns[4]) continue;
        add_line({kind: "RWY", name: aid ~ " " ~ id ~ (k ? sprintf(" +%d m", k * CELL_M) : ""), lat: f[0], lon: f[1], crs: r.heading, head: hd,
                  ends: [t[0], t[1], e[0], e[1]], len: lda, end: back * M2FT + lda, slope: slope, elev: et * M2FT - slope * back * M2FT,
                  spare: lda - (ns[1] - ns[0] + ns[4])});
    }
}

# ---------------------------------------------------------------- open ground
# The cells: a number a cell in one hash, its ground's code and elevation in one number (keys and values with no
# strings, nothing for the collector)
var OFF = 32768; var KEY = 65536; var KMUL = 100000; var EOFF = 2000;
var grid = {};
var bare = {};          # cells whose scenery was not there, and when
var origin = nil;       # the grid's (0, 0): [lat, lon]
var KI = 1; var KJ = 1; # degrees to cells
var retry = [];         # cells whose scenery was not there yet
var oq = {k: [], p: [], h: 0};     # open cells to try as strips' starts: their key, what they promise, the queue's head
var rq = {k: [], p: [], h: 0};     # and rough ones
var later = []; var laterp = [];
var budget = 0;         # the land cover reads left this tick
var seen = {};          # the land cover's names met, and how often (logged)
var count = {sampled: 0, bare: 0, open: 0, lines: 0};
var classify = func(g) {
    var m = g[1];
    if (m == nil) { seen["(none)"] = (contains(seen, "(none)") ? seen["(none)"] : 0) + 1; return 0; }
    if (size(m.names)) seen[m.names[0]] = (contains(seen, m.names[0]) ? seen[m.names[0]] : 0) + 1;
    if (!m.solid) return 0;
    # (an airfield's pavement, FlightGear's pa_ and pc_ materials: the runways' markings and the taxiways' and aprons'
    # pa_tiedown, along LOWI's runway on both sides; left out, every line along or across the field met it and was
    # refused, and the field was its two runway ends alone, its grass none)
    foreach (var n; m.names) {
        if (contains(OPEN, n)) return OPEN[n];
        var pre = substr(n, 0, 3);
        if (pre == "pa_" or pre == "pc_") return 1;
    }
    foreach (var n; m.names) if (contains(ROUGH, n)) return 4;
    return 0;
}
# (tried in the order of what a line there could leave)
var promise = func(lat, lon, elev) {
    var dN = (lat - S.lat) * FT_DEG; var dE = (lon - S.lon) * FT_DEG * S.k;
    var d = math.sqrt(dN * dN + dE * dE);
    var turn = ab(wrap(math.atan2(dE, dN) - S.trk)) + (d < S.Lf0 + 2 * S.R0 ? TWO_PI : 0);
    return (S.reach0 + (S.below - elev) * S.ldu) * S.rn - d - S.ktc * turn;
}
var sample = func(i, j, again = 0) {
    var k = (i + OFF) * KEY + j + OFF;
    if (grid[k] != nil) return;
    var b = bare[k];
    if (b != nil and S.now - b < 5) return;
    var lat = origin[0] + j / KJ; var lon = origin[1] + i / KI;
    var g = geodinfo(lat, lon);
    count.sampled += 1;
    if (g == nil) {                               # (the scenery not there yet: once more after the search)
        count.bare += 1;
        bare[k] = S.now;
        if (!again) append(retry, k);
        return;
    }
    var kd = classify(g);
    var e = g[0] * M2FT;
    grid[k] = kd * KMUL + e + EOFF;
    if (kd) {
        var q = kd == 4 ? rq : oq;
        append(q.k, k); append(q.p, promise(lat, lon, e));
        if (kd != 4) count.open += 1;
    }
}
# (the cells within the search's radius by blocks, in the order of what a straight-in to each could leave, from where the
# engines quit: the way there and the turn to its bearing at the turns' measured cost, a whole circle more where the
# final's start is nearer than the turn onto it takes (2 R0). Only the order the land cover is read in, the choice the
# plans' own; ring by ring from the aircraft the fields 1.8 NM out that were within reach at the cut were read 6 s
# later, their margin gone at 950 ft/s, the aircraft flying away from them)
var blocks = nil; var bq = 0; var bc = 0; var ni = 0; var nj = 0;
var search_trk = 0;
var search_order = func(rc) {
    var reach = S.reach0 * S.rn;
    var out = [];
    var oN = (origin[0] - S.lat) * FT_DEG; var oE = (origin[1] - S.lon) * FT_DEG * S.k;
    search_trk = S.trk;
    var nb = int(rc / BLOCK) + 1;
    var lim = (rc + BLOCK) * (rc + BLOCK);
    for (var bi = -nb; bi <= nb; bi += 1) {
        for (var bj = -nb; bj <= nb; bj += 1) {
            var i = bi * BLOCK; var j = bj * BLOCK;
            if (i * i + j * j > lim) continue;
            var dN = oN + j * CELL_FT; var dE = oE + i * CELL_FT;
            var d = math.sqrt(dN * dN + dE * dE);
            var turn = ab(wrap(math.atan2(dE, dN) - S.trk)) + (d < S.Lf0 + 2 * S.R0 ? TWO_PI : 0);
            append(out, [d + S.ktc * turn - reach, i, j]);
        }
    }
    return sort(out, func(a, b) { return a[0] - b[0]; });
}
var next_cell = func(rc) {
    # (again after a big turn, from where the aircraft has got to: ordered on the course toward the Nordkette, the search was
    # still reading that slope 15 s after the turn back, and the valley's fields came too late)
    if (blocks != nil and ab(wrap(S.trk - search_trk)) > 1) blocks = nil;
    if (blocks == nil) { blocks = search_order(rc); bq = 0; bc = 0; }
    var n = size(blocks); var nn = BLOCK * BLOCK; var lim = rc * rc;
    while (bq < n) {
        var b = blocks[bq];
        while (bc < nn) {
            var r = int(bc / BLOCK);
            var i = b[1] + r - BLOCK / 2; var j = b[2] + bc - r * BLOCK - BLOCK / 2;
            bc += 1;
            if (i * i + j * j <= lim and grid[(i + OFF) * KEY + j + OFF] == nil) { ni = i; nj = j; return 1; }
        }
        bq += 1; bc = 0;
    }
    return 0;
}
# a strip from cell (i, j) along course h (deg) over len ft: open all along, level within the float's height over the
# line fitted to it, rising no more than RISE_MAX nor falling more than DOWN_MAX along h: 1, with its ground (the worst
# of it), the line's elevation at its start, its slope (+ rising); 0 if not, 2 if not all read yet
var sK = 0; var sE0 = 0; var sSL = 0;
var EL = setsize([], 128);
var strip = func(i, j, h, len) {
    var n = int(math.ceil(len * FT2M / CELL_M));
    if (n + 1 > size(EL)) EL = setsize(EL, n + 1);
    var si = math.sin(h * D2R); var co = math.cos(h * D2R);
    # (its ends first: further apart than the slope's limits and the float's height at each end allow, no line fits, and
    # over a hillside that is most courses, for two cells read of a dozen)
    var a = grid[(i + OFF) * KEY + j + OFF];
    var b = grid[(math.round(i + n * si) + OFF) * KEY + math.round(j + n * co) + OFF];
    if (a != nil and b != nil) {
        var de = b - int(b / KMUL) * KMUL - a + int(a / KMUL) * KMUL;
        var run = n * CELL_FT; var fl2 = 2 * S.fl;
        if (de > RISE_MAX * run + fl2 or de < -DOWN_MAX * run - fl2) return 0;
    }
    var kd = 1; var sum = 0;
    for (var k = 0; k <= n; k += 1) {
        var ci = math.round(i + k * si); var cj = math.round(j + k * co);
        var key = (ci + OFF) * KEY + cj + OFF;
        var v = grid[key];
        if (v == nil and budget > 0) {
            budget -= 1;
            sample(ci, cj);
            v = grid[key];
        }
        if (v == nil) return 2;
        var c = int(v / KMUL);
        if (!c) return 0;
        if (c > kd) kd = c;
        var e = v - c * KMUL - EOFF;
        EL[k] = e; sum += e;
    }
    # the line fitted (least squares) to the elevations along, ft per ft
    var mxk = n / 2; var my = sum / (n + 1); var sxy = 0; var sxx = 0;
    for (var k = 0; k <= n; k += 1) { var dk = k - mxk; sxy += dk * (EL[k] - my); sxx += dk * dk; }
    var sl = sxy / sxx / CELL_FT;
    if (sl > RISE_MAX or sl < -DOWN_MAX) return 0;
    var e0 = my - sl * mxk * CELL_FT;
    var fl = S.fl;
    for (var k = 0; k <= n; k += 1) if (ab(EL[k] - (e0 + sl * k * CELL_FT)) > fl) return 0;
    # (and level across it, a cell to each side at its ends and middle: only along it, a strip following the contour of a
    # mountainside read as flat, and sites were taken on the valley's walls)
    foreach (var k; [0, int(n / 2), n]) {
        var line = e0 + sl * k * CELL_FT;
        foreach (var sd; [-1, 1]) {
            var ci = math.round(i + k * si + sd * co); var cj = math.round(j + k * co - sd * si);
            var key = (ci + OFF) * KEY + cj + OFF;
            var v = grid[key];
            if (v == nil and budget > 0) {
                budget -= 1;
                sample(ci, cj);
                v = grid[key];
            }
            if (v == nil) return 2;
            if (ab(v - int(v / KMUL) * KMUL - EOFF - line) > CROSS_FT) return 0;
        }
    }
    sK = kd; sE0 = e0; sSL = sl;
    return 1;
}
# the open ground on along the strip's line past its end, ft: open and within the float's height of the line
var spare_past = func(i, j, h, len, e0, sl) {
    var n = int(math.ceil(len * FT2M / CELL_M));
    var si = math.sin(h * D2R); var co = math.cos(h * D2R);
    var fl = S.fl; var good = 0;
    for (var k = n + 1; k <= n + SPARE_CELLS; k += 1) {
        var ci = math.round(i + k * si); var cj = math.round(j + k * co);
        var key = (ci + OFF) * KEY + cj + OFF;
        var v = grid[key];
        if (v == nil and budget > 0) {
            budget -= 1;
            sample(ci, cj);
            v = grid[key];
        }
        if (v == nil) break;
        var c = int(v / KMUL);
        if (!c or ab(v - c * KMUL - EOFF - (e0 + sl * k * CELL_FT)) > fl) break;
        good += 1;
    }
    return good * CELL_FT;
}
# (every course that holds a strip from the cell, not only the first: over LOWI the open land west of the runway was
# taken on 060 and 090 alone, landing back towards the aircraft, and 270, which it could have reached, was never tried)
var try_cell = func(i, j) {
    var again = 0;
    var lat0 = origin[0] + j / KJ; var lon0 = origin[1] + i / KI;
    for (var hk = 0; hk < HEADINGS; hk += 1) {
        var h = hk * 360 / HEADINGS; var hd = HD[hk];
        var len = LEN[hk];
        var r = strip(i, j, h, len);
        if (!r) continue;
        if (r == 2) { again = 1; continue; }  # (tried again when the cells along it are read, while they are)
        if (sSL < 0) {                        # (falling: the longer landing it needs, along the same line)
            var nf = needs("crop", hd, sSL);
            var len2 = nf[1] - nf[0] + nf[3] - nf[2] + nf[4];
            r = strip(i, j, h, len2);
            if (!r) continue;
            if (r == 2) { again = 1; continue; }
            len = len2;
        }
        var g = KIND_NAME[sK]; var e0 = sE0; var sl = sSL;
        var more = spare_past(i, j, h, len, e0, sl);
        var nd = needs(g, hd, sl);
        var back = (nd[0] + nd[2]) * FT2M;    # the flare's point: the strip's start where the wheels come under 50 ft early
        var f = move(lat0, lon0, back > 0 ? h + 180 : h, ab(back));
        var e = move(lat0, lon0, h, len * FT2M);
        # (the plane at the flare's point: the strip's line carried back to it)
        add_line({kind: g, name: "FIELD", lat: f[0], lon: f[1], crs: h, head: hd, ends: [lat0, lon0, e[0], e[1]], len: len,
                  elev: e0 - sl * back * M2FT, slope: sl, end: back * M2FT + len + more, spare: more});
        count.lines += 1;
    }
    return again;
}
# the tries: in the order read, open ground before rough unless rough promises more within reach; the cells their lines
# still wait on read from the tick's budget, those it ran out of again a second later
var tries_tick = func {
    if (math.mod(ticks, RANK_TICKS) == 0) {
        forindex (var n; later) {
            var q = int(grid[later[n]] / KMUL) == 4 ? rq : oq;
            append(q.k, later[n]); append(q.p, laterp[n]);
        }
        later = []; laterp = [];
    }
    if (size(fresh) - fi > 4 * PRICE_PER_TICK) return;          # (the pricing behind: open farmland)
    var most = size(lines) > MAX_LINES / 2 ? 2 : TRY_PER_TICK;  # (lines enough: no more than the trimming would drop)
    for (var n = 0; n < most; n += 1) {
        var ho = oq.h < size(oq.k); var hr = rq.h < size(rq.k);
        if (!ho and !hr) break;
        var q = (ho and (!hr or oq.p[oq.h] >= floor_ or oq.p[oq.h] >= rq.p[rq.h])) ? oq : rq;
        var k = q.k[q.h]; var p = q.p[q.h];
        q.h += 1;
        var a = int(k / KEY);
        if (try_cell(a - OFF, k - a * KEY - OFF)) { append(later, k); append(laterp, p); }
    }
}

# ---------------------------------------------------------------- ranking and choice
var cands = [];         # the lines of the last ranking, in its order
var chosen = nil;       # the line flown
var picked = 0;         # chosen by the crew (no automatic change)
var changed = -100;     # when the site last changed, s
var none = 0;           # the crew chose none
var k_sent = nil; var hac_sent = 0; var fin_sent = 0;
var sent_pl = nil;      # the plan committed
var path_t = -100;      # when
var PATH_HOLD_S = 5;    # no other path sooner than this but for the ground
# the committed path as it stands, from where the aircraft is on it
var flown_plan = func(c) {
    var pl = sent_pl; var g = pl[7];
    var psi = wrap(S.trk - c.crs * D2R);
    var s0 = num(JSB ~ "ap/eo-site-s0-left-ft");
    var x = num(JSB ~ "ap/eo-site-x-ft") + s0 * math.cos(psi); var y = num(JSB ~ "ap/eo-site-y-ft") + s0 * math.sin(psi);
    var a1 = num(JSB ~ "ap/eo-site-a1-now-rad");
    return [num(JSB ~ "ap/eo-site-margin-ft"), num(JSB ~ "ap/eo-site-path-ft"), pl[2], 0, g[5], pl[5], pl[6],
            [0, g[1], 0, g[3], a1, g[5], x, y, psi, g[9], s0, g[11]], pl[8], pl[9], pl[10], pl[11], pl[12], spend_nb(num(JSB ~ "ap/eo-site-path-ft"))];
}
var GS = nil;           # the last ranking's glide state
var name_of = func(c) { return c.kind ~ " " ~ c.name; }
var tier = func(c) { return plan_tier(c.plan, c.margin); }
var alike = func(a, b) {       # by the room past the end, kind, wind and slope: 0, or which is better
    var pa = a.spare >= SPARE_FT; var pb = b.spare >= SPARE_FT;
    if (pa != pb) return pa ? -1 : 1;
    var ka = KIND_RANK[a.kind]; var kb = KIND_RANK[b.kind];
    if (ka != kb) return ka - kb;
    # (into the wind before downwind: a tailwind lengthens the float and the run, and the field chosen in Kansas was
    # landed on with 13 kt behind; within HEAD_KT alike)
    var ha = mn(a.head, HEAD_KT * 2); var hb = mn(b.head, HEAD_KT * 2);
    if (ab(ha - hb) > HEAD_KT) return hb - ha;
    # (then level or falling before rising: rising, the touchdown faster, the nacelles lower)
    var ra = contains(a, "slope") and a.slope > 0.001; var rb = contains(b, "slope") and b.slope > 0.001;
    if (ra != rb) return ra ? 1 : -1;
    return 0;
}
var order = func(a, b) {
    if (a.clear != b.clear) return a.clear ? -1 : 1;
    var ta = tier(a); var tb = tier(b);
    if (ta != tb) return tb - ta;
    if (ta == 0) return (b.margin + b.plan[8]) - (a.margin + a.plan[8]);
    if (ta == 1 or ta == NB) return a.margin - b.margin;      # (high: the least)
    var r = alike(a, b);
    if (r != 0) return r;
    var fa = a.plan[10] > 0; var fb = b.plan[10] > 0;
    if (fa != fb) return fa ? 1 : -1;             # (a full final before a cut one)
    # (robust, the least surplus; a stretch, the most it keeps)
    return ta == 4 ? a.margin - b.margin : b.margin - a.margin;
}
var order_cat = func(a, b) {
    if (a.clear != b.clear) return a.clear ? -1 : 1;
    var ta = tier(a); var tb = tier(b);
    if (ta != tb) return tb - ta;
    if (ta <= 1 or ta == NB) return 0;            # (both short or both high: only by how much, the margin)
    return alike(a, b);
}
var kname = func(pl) {
    var g = pl[7];
    if (g == nil) return "none";
    return sprintf("%s%s%s, circle at %.0f deg%s", g[10] > S.adv + 1 ? "straight, " : "", g[1] > 0 ? "R" : "L", pl[2] > 0 ? "R" : "L", pl[9],
                   pl[10] > 0 ? sprintf(", final %.0f ft", pl[10]) : "");
}
var send_path = func(pl) {
    var g = pl[7];
    setprop(JSB ~ "ap/eo-site-side", pl[2]);
    setprop(JSB ~ "ap/eo-site-turn", g == nil ? 0 : g[1]);
    setprop(JSB ~ "ap/eo-site-hac-deg", pl[9]);
    setprop(JSB ~ "ap/eo-site-final-cmd-ft", pl[10]);
    setprop(JSB ~ "ap/eo-site-slow", pl[9] >= TIGHT_HB);
    setprop(JSB ~ "ap/eo-site-r0-cmd-ft", g != nil and pl[9] >= TIGHT_HB ? g[9] : 0);
    k_sent = g == nil ? nil : g[11];
    hac_sent = pl[9]; fin_sent = pl[10];
    # (the path committed: the flight model flies these, solving nothing: genv22 ap/eo-site-commit)
    if (g != nil) {
        var sg = pl[2]; var R = pl[5]; var phi = g[3];
        var s0 = g[10] - S.adv;
        setprop(JSB ~ "ap/eo-site-tx-ft", -pl[6] + sg * R * math.sin(phi));
        setprop(JSB ~ "ap/eo-site-ty-ft", sg * R - sg * R * math.cos(phi));
        setprop(JSB ~ "ap/eo-site-a2-sent-rad", g[5]);
        setprop(JSB ~ "ap/eo-site-s0-left-ft", s0 > 1 ? s0 : 0);
        setprop(JSB ~ "ap/eo-site-leg", 0);
    }
    setprop(JSB ~ "ap/eo-site-commit", g != nil);
    sent_pl = pl; path_t = S.now;
}
var send = func(c) {
    if (c == nil) {
        setprop(JSB ~ "ap/eo-site-on", 0);
        setprop(JSB ~ "ap/eo-site-side", 0); setprop(JSB ~ "ap/eo-site-turn", 0);
        setprop(JSB ~ "ap/eo-site-hac-deg", 0); setprop(JSB ~ "ap/eo-site-final-cmd-ft", 0);
        setprop(JSB ~ "ap/eo-site-commit", 0);
        k_sent = nil; sent_pl = nil;
        return;
    }
    send_path(c.plan);
    setprop(JSB ~ "ap/eo-site-lat-deg", c.lat);
    setprop(JSB ~ "ap/eo-site-lon-deg", c.lon);
    setprop(JSB ~ "ap/eo-site-course-deg", c.crs);
    var planed = contains(c, "slope") and contains(c, "elev");
    setprop(JSB ~ "ap/eo-site-plane", planed);
    # (the plane's elevation at the point, its slope, where its strip ends past the point, ft: the flight model flies
    # over it on the final, short of the end)
    if (planed) {
        setprop(JSB ~ "ap/eo-site-elev-ft", c.elev); setprop(JSB ~ "ap/eo-site-slope", c.slope); setprop(JSB ~ "ap/eo-site-end-ft", c.end);
    }
    setprop(JSB ~ "ap/eo-site-on", 1);
    setprop(JSB ~ "ap/eo-site-new", 1);
}
# the best lines by their keys, one pass; the one flown with them
var select_top = func {
    var K = 4 * TOP; var top = []; var worst = 1e300; var n = 0;
    foreach (var c; lines) {
        var k = c.key;
        if (k >= 1e18 or (n >= K and k >= worst)) continue;
        if (n < K) { append(top, c); n += 1; }
        else top[n - 1] = c;
        var i = n - 1;
        while (i > 0 and top[i - 1].key > k) { top[i] = top[i - 1]; i -= 1; }
        top[i] = c;
        if (n >= K) worst = top[n - 1].key;
    }
    if (chosen != nil) {
        var there = 0;
        foreach (var c; top) if (c == chosen) there = 1;
        if (!there) append(top, chosen);
    }
    return top;
}
# every plan too high: a line a tight plan fits or stretches to, its ground clear, the most kept first; nil if none. (A pass over
# the lines at the shortest final first: 14 us each, and it runs only until one is flown)
var tight_search = func(gs) {
    var cand = [];
    foreach (var c; lines) {
        if (c.key >= 1e18 or (contains(c, "dry") and !c.dry)) continue;
        var k = geom(c, TIGHT_HB, FINALS[size(FINALS) - 1]);
        if (k < 0) continue;
        var t = tier_k(k);
        if (t == 2.8 or t == 2.7) append(cand, [c, OM[k]]);
    }
    cand = sort(cand, func(a, b) { return b[1] - a[1]; });
    var tried = 0;
    var gt = {h0: gs.h0 + S.after_ft, hw: gs.hw, ld: gs.ld, hpr: gs.hpr, hpr0: gs.hpr0, fl: gs.fl, r0: gs.r0 * TIGHT_R0};
    foreach (var q; cand) {
        var c = q[0];
        if (tried >= 4 * TOP) break;
        if (c.kind != "RWY" and !contains(c, "dry")) c.dry = dry_strip(c);
        if (c.kind != "RWY" and !c.dry) continue;
        tried += 1;
        var sp = contains(c, "plan") ? c.plan : nil; var sm = contains(c, "margin") ? c.margin : 0;
        foreach (var Lj; [0, 3000, 800]) {
            foreach (var pl; plans(c, TIGHT_HB, Lj)) {
                if (plan_tier(pl) < 2.7) continue;
                c.plan = pl; c.margin = pl[0];
                if (path_clear(c, gt)) { c.path = pl[1]; c.clear = 1; return c; }
            }
        }
        c.plan = sp; c.margin = sm;
    }
    return nil;
}
var rank = func {
    weigh();
    var gs = glide_state(); GS = gs;
    var st = num(JSB ~ "ap/eo-site-state");
    var top = select_top();
    # (planned in full and the terrain looked under down the ranking until TOP lines are clear; those not reached are
    # not in it)
    var good = 0; var done = []; var swapped = 0;
    var low = num(JSB ~ "hh/h-wheels-ft") < COMMIT_FT;
    reads = RANK_READS;
    foreach (var c; top) {
        var mine = c == chosen;
        if (good >= TOP and !mine) { c.clear = 0; continue; }
        if (c.kind != "RWY" and !contains(c, "dry")) c.dry = dry_strip(c);
        if (c.kind != "RWY" and !c.dry and !mine) { c.key = 2e18; c.clear = 0; continue; }
        c.opts = energy_plans(c);
        c.plan = c.opts[0]; c.clear = 1;
        c.margin = c.plan[0]; c.path = c.plan[1];
        append(done, c);
        if (mine) reads = 1e9;                  # (the one flown never for want of a read)
        var pick = nil; var pclear = 0; var fl = nil; var flclear = 0;
        var flying = mine and st > 0.5 and st < 1.5 and sent_pl != nil and sent_pl[7] != nil;
        if (flying) {
            fl = flown_plan(c);
            c.margin = fl[0]; c.plan = fl;
            flclear = path_clear(c, gs);
        }
        foreach (var pl; c.opts) {
            c.margin = pl[0]; c.plan = pl;
            if (path_clear(c, gs)) { pick = pl; pclear = 1; break; }
        }
        if (pick == nil) pick = c.opts[0];
        if (flying) {
            # (the path flown kept, but for the ground or one that keeps half a lap more: changing it costs turns of its own)
            # (and never for one out of reach: a warning on the path flown "put right" with one 31 000 ft short)
            var better = pclear and pick[0] + pick[8] >= 0 and low == 0 and (!flclear or (pick[0] > fl[0] + pick[12] / 2 and S.now - path_t > PATH_HOLD_S));
            if (better) {
                log(sprintf("%s: path %s for %s (margin %.0f for %.0f%s)", name_of(c), kname(pick), kname(fl), pick[0], fl[0], flclear ? "" : ", its path meets the ground"));
                send_path(pick);
                setprop(JSB ~ "ap/eo-site-new", 1);
                swapped = 1;
            } else { pick = fl; pclear = flclear; }
        }
        c.plan = pick; c.clear = pclear;
        c.margin = pick[0]; c.path = pick[1];
        good += pclear and plan_tier(pick) >= 2;     # (one too high or short no reason to stop looking)
        if (mine) reads = RANK_READS;
    }
    # (the one flown: the flight model's own margin and way, its plan from where the aircraft is on it)
    # (not the second its path was changed: it still reads the old path's, and a path 33 000 ft short, just put right,
    # gave the site up as out of reach, and each change set off the next)
    var cur = chosen;
    if (cur != nil and st > 0.5 and !swapped) {
        cur.margin = num(JSB ~ "ap/eo-site-margin-ft"); cur.path = num(JSB ~ "ap/eo-site-path-ft");
    }
    if (cur != nil and st > 1.5) {               # (judged along what the guidance flies)
        reads = 1e9;
        cur.clear = path_clear(cur, gs, st);
        # (running on past the strip, read every second, no reason to change every second: two fields took turns for 8 s, high on
        # their final, each past the other's end)
        if (st > 2.5 and cur.clear and num(JSB ~ "hh/h-wheels-ft") > COMMIT_FT and S.now - changed > HOLD_S) cur.clear = line_clear(cur, gs);
        reads = RANK_READS;
    }
    cands = sort(done, order);
    if (none) { setprop(JSB ~ "ap/eo-open-on", 0); return; }
    # the best from here, every second: clear and within reach; none yet, while the search goes on nothing (the open
    # course meanwhile), after it the least short clear one. (A high one, as the least short, only after it: taken at
    # the first ranking, a second after the engines quit, the aircraft was on its final in seconds, past changing, and
    # landed 25 000 ft on)
    var best = nil;
    # (the steep way down not while the search still has fields to find: taken 4 s in, a stretch that had landed on its mark every
    # time was found a second too late)
    foreach (var c; cands) { var t_ = tier(c); if (c.clear and (sampling ? t_ >= 2 and t_ != NB : t_ >= 1)) { best = c; break; } }
    # (the one flown no way down: anything clear, high too, and the search still going)
    var stuck = cur != nil and !cur.clear;
    if (best == nil and (!sampling or stuck)) foreach (var c; cands) if (c.clear and c != chosen) { best = c; break; }
    # (only a plan too high to be had: tight, or hold, if a landing can be seen at its end. Not on the circle or final: those are flown)
    var hwh = num(JSB ~ "hh/h-wheels-ft");
    reads = RANK_READS * 4;
    if (!picked and !hold_dir and best != nil and tier(best) == 1 and hwh > HOLD_MIN_FT and (cur == nil or (st < 1.5 and tier(cur) <= 1))) {
        var tc = tight_search(gs);
        if (tc != nil) best = tc;
    }
    reads = RANK_READS;
    if (hold_dir and S.now < hold_until and hwh > HOLD_MIN_FT) {
        if (best == nil or tier(best) != 4) best = nil;
    } else {
        hold_dir = 0;
        if (!picked and best != nil and tier(best) == 1 and hwh > HOLD_MIN_FT and (cur == nil or (st < 1.5 and tier(cur) <= 1))) {
            var f = hold_search(gs, lines);
            if (f != nil) {
                hold_dir = f[1]; hold_until = S.now + f[0];
                log(sprintf("holding %s for %d s, every plan too high: then %s", hold_dir > 0 ? "right" : "left", f[0], name_of(f[2])));
                if (cur != nil) { chosen = nil; send(nil); cur = nil; }
                best = nil;
            }
        }
    }
    var short = cur == nil ? 0 : -needs(cur.kind == "RWY" ? "pavement" : cur.kind)[0];
    var now = S.now;
    if (!picked and (cur == nil or num(JSB ~ "hh/h-wheels-ft") > COMMIT_FT)) {
        if (cur == nil) {
            if (best != nil) {
                chosen = best; send(best); changed = now;
                var t = tier(best);
                log(sprintf("chosen %s, margin %.0f ft, path %.2f NM (%s)%s", name_of(best), best.margin, best.path / 6076.12, kname(best.plan),
                            t == 0 ? " (none within reach: the least short)" : t == 1 ? " (high)" : t == NB ? " (high: the nacelle brake)" : t == 2.8 or t == 2.7 ? " (the turns tight)" : t < 4 ? " (a stretch: the brake in)" : ""));
            }
        } elsif (st < 2.5 or tier(cur) == 1 or stuck) {   # (on its final too while it stays high, or leads nowhere)
            var low = cur.margin + cur.plan[8] < -short;
            var fails = !cur.clear or (low and (sampling or (best != nil and tier(best) >= 1)));
            # (one within reach never left for one out of it: too high over a valley's floor is the better trouble. A final 11 000
            # ft high half a mile from a field was left for one 10 000 ft short, and that was the slope)
            if (best != nil and cur.margin + cur.plan[8] >= 0 and tier(best) == 0) best = nil;
            # (given up for none only before its circle and only for its path meeting the ground: short, it is still
            # the way down onto open ground; dropped short on the circle, 900-1150 ft over LOWI's north slope and the
            # search still going, ENG OUT came down on 30-88 % slopes beside the runway)
            if (best == nil or best == chosen) {
                # (past the circle only with height left to do something else with)
                if (!cur.clear and cur.margin + cur.plan[8] < 0 and (st < 1.5 or num(JSB ~ "hh/h-wheels-ft") > GIVE_UP_FT)) {
                    log(name_of(cur) ~ ": its path meets the ground; none better yet");
                    chosen = nil; send(nil); changed = now;
                }
            # (a tight plan kept but for its own path: left at 16 s for a wide one that read better, the aircraft slow and out of its
            # way, it ran out of height on a forest's edge)
            } elsif (fails or (order_cat(best, cur) < 0 and now - changed > HOLD_S and (st < 1.5 or tier(cur) <= 1) and cur.plan[9] < TIGHT_HB)) {
                # (merely better only before the circle: on it, past S-turns, a site near its energy is kept. Over LOWI
                # one at 1001 ft was dropped there for one at 2719, the final began 1900 ft high and with nothing left to
                # spend ran off its field into the Inn)
                log(sprintf("%s (margin %.0f) for %s (margin %.0f, %s)", name_of(best), best.margin, name_of(cur), cur.margin,
                            !cur.clear ? "its path meets the ground" : low ? "out of reach" : "better"));
                chosen = best; send(best); changed = now;
            }
        }
    }
    setprop(JSB ~ "ap/eo-nb-hold", hold_dir != 0);
    # none chosen: the holding turn, or the open course for ENG OUT (genv22 ap/eo-open-hdg-deg)
    if (chosen == nil) {
        reads = RANK_READS;
        var oc = hold_dir ? math.mod(S.trk * R2D + hold_dir * HOLD_LEAD + 720, 360) : open_course(gs);
        setprop(JSB ~ "ap/eo-open-on", oc != nil);
        if (oc != nil) setprop(JSB ~ "ap/eo-open-hdg-deg", oc);
    } else setprop(JSB ~ "ap/eo-open-on", 0);
    reads = 0;
    var sec = int(now);
    if (math.mod(sec, 10) == 0) {
        var names = [];
        foreach (var n; keys(seen)) append(names, n ~ " " ~ seen[n]);
        log(sprintf("land cover: %d read (%d without scenery), %d open, %d lines, %d tries to go; tick %.1f ms mean, %.1f worst; %s",
                    count.sampled, count.bare, count.open, size(lines), size(oq.k) - oq.h + size(rq.k) - rq.h,
                    tick_n ? tick_sum / tick_n * 1000 : 0, tick_max * 1000, string.join(", ", names)));
        tick_sum = 0; tick_n = 0; tick_max = 0;
    }
    if (cur != nil and math.mod(sec, 5) == 0) {
        log(sprintf("%s: margin %.0f ft here, %.0f the flight model's; path %.2f NM, state %d, clear %d",
                    name_of(cur), cur.margin, num(JSB ~ "ap/eo-site-margin-ft"), cur.path / 6076.12, st, cur.clear));
    }
}
# the next candidate (Ctrl-L, the NAV format's SITE key): those within reach (or all, if none is) in their order,
# then none (ENG OUT's heading), then the first again; not once flaring
var next = func {
    if (!num(AFCS ~ "eng-out") or num(JSB ~ "ap/eo-phase") > 2.5) return;
    var feas = [];
    foreach (var c; cands) if (c.margin >= 0) append(feas, c);
    if (!size(feas)) feas = cands;
    if (!size(feas)) return;
    var i = -1;
    forindex (var k; feas) if (chosen != nil and feas[k] == chosen) i = k;
    picked = 1;
    if (!none and i + 1 >= size(feas)) {
        none = 1; chosen = nil; send(nil);
        log("crew chose none");
        return;
    }
    var c = feas[none ? 0 : i + 1];
    none = 0; chosen = c; send(c);
    log("crew chose " ~ name_of(c));
}

# ---------------------------------------------------------------- the displays' copy
var publish = func {
    var n = mn(MAX_SHOWN, size(cands));
    setprop(OUT ~ "n", n);
    for (var i = 0; i < n; i += 1) {
        var c = cands[i];
        var p = OUT ~ "cand[" ~ i ~ "]/";
        setprop(p ~ "lat0", c.ends[0]); setprop(p ~ "lon0", c.ends[1]);
        setprop(p ~ "lat1", c.ends[2]); setprop(p ~ "lon1", c.ends[3]);
        setprop(p ~ "kind", c.kind);
        setprop(p ~ "reach", c.margin >= 0);
        setprop(p ~ "chosen", c == chosen);
    }
    var cur = chosen;
    setprop(OUT ~ "on", cur != nil);
    if (cur == nil) return;
    setprop(OUT ~ "name", cur.kind == "RWY" ? "RWY " ~ cur.name : "FIELD " ~ string.uc(cur.kind));
    setprop(OUT ~ "path-nm", cur.path / 6076.12);
    setprop(OUT ~ "margin-nm", cur.margin / 6076.12);
    # the planned path for the map, about 40 points of it: from the aircraft, what is left of the flight model's own
    # once on its circle or final
    var st = num(JSB ~ "ap/eo-site-state");
    var m = path_pts(cur, GS.r0, st > 1.5 ? st : nil);
    var step = int(m / 40) + 1;
    var k = 1;
    setprop(OUT ~ "path/pt[0]/lat", S.lat); setprop(OUT ~ "path/pt[0]/lon", S.lon);
    for (var q = 0; q < m; q += step) {
        setprop(OUT ~ "path/pt[" ~ k ~ "]/lat", PN[q]); setprop(OUT ~ "path/pt[" ~ k ~ "]/lon", PE[q]);
        k += 1;
    }
    setprop(OUT ~ "path/pt[" ~ k ~ "]/lat", cur.lat); setprop(OUT ~ "path/pt[" ~ k ~ "]/lon", cur.lon);
    setprop(OUT ~ "path/n", k + 1);
}

# ---------------------------------------------------------------- the loop
var was = 0;
var ticks = 0;
var sampling = 1;       # the search still going
var tick_sum = 0; var tick_n = 0; var tick_max = 0;
var reset = func {
    grid = {}; bare = {}; origin = nil; retry = []; lines = []; fresh = []; fi = 0; pi_ = 0; cands = [];
    oq = {k: [], p: [], h: 0}; rq = {k: [], p: [], h: 0}; later = []; laterp = [];
    blocks = nil; rw_todo = []; rw_wait = []; oc_last = nil; hold_dir = 0; hold_until = -1;
    chosen = nil; picked = 0; none = 0; sampling = 1; seen = {}; count = {sampled: 0, bare: 0, open: 0, lines: 0}; changed = -100;
    k_sent = nil; hac_sent = 0; fin_sent = 0; tick_sum = 0; tick_n = 0; tick_max = 0;
    setprop(OUT ~ "n", 0); setprop(OUT ~ "on", 0);
    send(nil); setprop(JSB ~ "ap/eo-open-on", 0);
}
var update = func {
    # (the site kept through the flare and the landing: the flight model flies its line there; found and ranked only
    # before)
    if (!num(AFCS ~ "eng-out")) {
        if (was) reset();
        was = 0;
        return;
    }
    if (was and num(JSB ~ "ap/eo-phase") > 2.5) return;
    var t0 = systime();
    if (!was) {
        reset();
        if (!size(W) and !load()) return;
        weigh();
    }
    read_state();
    if (!was) {
        origin = [S.lat, S.lon];
        KI = FT_DEG * math.cos(S.lat * D2R) / CELL_FT; KJ = FT_DEG / CELL_FT;
        S.fl = num(JSB ~ "ap/eo-float-ft", 10);
        runways(S.reach0 / 6076.12 + 2);
        log(sprintf("on: reach %.1f NM, %d runway ends in range", S.reach0 / 6076.12, size(rw_todo)));
        # (a course at once: ten seconds from the slope, the first ranking's second was a tenth of them)
        GS = glide_state();
        reads = RANK_READS;
        var oc = open_course(GS);
        reads = 0;
        setprop(JSB ~ "ap/eo-open-on", oc != nil);
        if (oc != nil) setprop(JSB ~ "ap/eo-open-hdg-deg", oc);
    }
    was = 1;
    S.fl = num(JSB ~ "ap/eo-float-ft", 10);
    var rc = int(mn(SEARCH_NM, S.reach0 / 6076.12 + 1) * 1852 / CELL_M);
    if (size(rw_todo)) {
        var r = rw_todo[0];
        rw_todo = subvec(rw_todo, 1);
        runway_end(r[0], r[1]);
    }
    budget = PER_TICK;
    tries_tick();
    while (budget > 0) {
        budget -= 1;
        if (next_cell(rc)) sample(ni, nj);
        else {
            if (!size(retry)) { sampling = 0; break; }
            var k = retry[0];
            retry = subvec(retry, 1);
            var a = int(k / KEY);
            sample(a - OFF, k - a * KEY - OFF, 1);
        }
    }
    price_tick();
    if (hold_dir and chosen == nil) setprop(JSB ~ "ap/eo-open-hdg-deg", math.mod(S.trk * R2D + hold_dir * HOLD_LEAD + 720, 360));
    ticks += 1;
    if (math.mod(ticks, RANK_TICKS) == 0) {
        rank(); publish();
        if (size(rw_wait)) { rw_todo = rw_todo ~ rw_wait; rw_wait = []; }
    }
    var dt = systime() - t0;
    tick_sum += dt; tick_n += 1;
    if (dt > tick_max) tick_max = dt;
}
var timer = nil;         # kept: a timer nothing refers to is collected, and stops
setlistener("/sim/signals/fdm-initialized", func(n) {
    if (!n.getValue() or timer != nil) return;
    load();
    timer = maketimer(TICK_S, update);
    timer.start();
}, 1, 0);
