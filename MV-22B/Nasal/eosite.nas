# ENG OUT's landing site: with both engines out (ENG OUT on), the places the glide can reach and land on, ranked, the
# best handed to the flight model, which flies to it coupled (CPLD) or shows it to fly (fdm/genv22.py eng_out_site: a
# heading alignment circle and S-turns, as the Space Shuttle's terminal area energy management).
#
# The candidates: the runways of the airports in reach (FlightGear's airport data), and open, flat ground from
# FlightGear's own land cover (geodinfo): arable land, pasture, natural grassland, airport ground, dry lakebeds (WS3's
# CORINE Land Cover classes 124, 211, 212, 231, 241, 321, and WS2's names for them), sampled every CELL_M and tried as
# strips on HEADINGS courses. A site must hold the landing the flight model makes at this weight (Systems/
# mv22b-displays.xml's eo-site, from fdm/deadland.py site and siteguide): the wheels under 50 ft to the stop, braked,
# with the scatter of where the flare begins (the guidance's, the reach's); a field level within the float's height
# over the line fitted to it, rising no more than RISE_MAX and falling no more than DOWN_MAX along the landing. The
# chosen one's plane (that line, a runway's from its ends) goes to the flight model with it, which flares over the
# plane rather than over the ground under it on the final, short of the strip's end (ap/eo-h-ft). Ranked by what each leaves of the glide's energy (the
# flight model's reach less the path its guidance would fly there, the turns at their measured cost: the flight
# model's own formulas): those within reach first, those within SITE_NEAR_FT of the energy before larger surpluses
# (S-turns spend no more than their leg allows, and those land long), runways before grass before crops, into the
# wind before downwind, level or falling before rising, the least surplus first.
#
# The best is chosen when ENG OUT comes on, and changed while it is out of reach and another is not (before the
# circle); Ctrl-L or the NAV format's SITE key steps through them, and to none (ENG OUT's heading then).
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
var FT2M = 0.3048;
var M2FT = 1 / 0.3048;
var FT_DEG = 60 * 6076.12;       # ft per degree of latitude
var CELL_M = 150;                # the land cover sampled every, m
var PER_TICK = 30;               # land cover samples per tick
var TRY_PER_TICK = 4;            # open cells tried as strips' starts per tick
var TICK_S = 0.2;
var RANK_TICKS = 5;              # ranked every second
var HEADINGS = 12;               # the strips' courses, 30 deg apart
var MAX_FIELDS = 12;
var MAX_SHOWN = 16;
var SEARCH_NM = 8;               # the land cover searched no further (the sampling's cost)
# A field's grade along the landing (fdm/deadland.py flare, the descent against the ground): falling, no more than FAA
# AC 150/5300-13B's longitudinal grade for small aircraft, the float lengthened (5.5-8.1 ft/s on 2 % down at 38 700 and
# 47 500 lb); rising, the float cut short and the touchdown fast, the corridor holding the nacelles low: at 47 500 lb on
# 0.5-2 % up 174-187 KCAS, the nacelles at 16-32 deg, the proprotors in the ground
var DOWN_MAX = 0.02;
var RISE_MAX = 0.005;
var FALL_K = 10;                 # falling, the flare's way and the run longer by this per unit of fall (the flare's 6-23 % on 2 %)
var SITE_NEAR_FT = 6076.1;       # the guidance's scatter measured for sites within this of the energy (deadland.py)
var OPEN = {Airport: "grass", AirportKeep: "grass", Grass: "grass", GrassCover: "grass", Grassland: "grass", DryLake: "grass",
            grass_rwy: "grass", dirt_rwy: "grass", lakebed_taxiway: "grass",
            MixedCrop: "crop", IrrCrop: "crop", DryCrop: "crop", Crop: "crop", CropGrass: "crop", CropGrassCover: "crop",
            MixedCropPastureCover: "crop", DryCropPastureCover: "crop", IrrCropPastureCover: "crop"};
var KIND_RANK = {RWY: 0, grass: 1, crop: 2};
var HEAD_KT = 5;                 # winds along the landing within this alike for the ranking
var HOLD_S = 10;                 # after a change of site, no other for this long (the flight model's margin settling)

var num = func(p, d = 0) { var x = getprop(p); x == nil ? d : x; };
var log = func(s) logprint(LOG_INFO, "MV-22B ENG OUT site: ", s);
var wrap = func(a) { return math.atan2(math.sin(a), math.cos(a)); }
var m2pi = func(a) { var w = wrap(a); w < 0 ? w + 2 * math.pi : w; };

# ---------------------------------------------------------------- the landing's needs at this weight
var W = [];
var TAB = {};
var FLARE = [0, 0];
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
    return size(W) > 1;
}
var at = func(t, w) {
    var ys = TAB[t];
    if (w <= W[0]) return ys[0];
    for (var i = 1; i < size(W); i += 1)
        if (w <= W[i]) return ys[i - 1] + (ys[i] - ys[i - 1]) * (w - W[i - 1]) / (W[i] - W[i - 1]);
    return ys[size(ys) - 1];
}
# [lo, hi, h50, td, roll], ft: the flare's start within lo..hi of its point (the guidance's scatter widened by the
# reach's error: + long, the aircraft arriving low and short), the wheels under 50 ft and on the ground past the
# flare's start, the run braked to a stop on this ground (pavement, grass, crop); into a headwind of head kt (- tail):
# the flare's way over the ground by its ground speed's share (from the glide's 200 KCAS to the touchdown's), the run
# by its square (from the touchdown's); on ground falling along the landing (slope < 0) both longer by FALL_K
var needs = func(ground, head = 0, slope = 0) {
    var w = num(JSB ~ "inertia/weight-lbs");
    var vt = at("td-kt", w);
    var ks = 1 + FALL_K * math.max(0, -slope);
    var kf = math.max(0.2, 1 - head / ((200 + vt) / 2)) * ks;
    var kr = math.pow(math.max(0.2, 1 - head / vt), 2) * ks;
    return [FLARE[0] - math.max(0, at("err-hi", w)), FLARE[1] + math.max(0, -at("err-lo", w)), at("h50", w) * kf, at("td", w) * kf,
            at("roll-" ~ ground, w) * kr];
}
# the headwind along a course, kt (- tail)
var head = func(crs) { return num("/environment/wind-speed-kt") * math.cos((num("/environment/wind-from-heading-deg") - crs) * D2R); }

# ---------------------------------------------------------------- the path to a site and its margin
# The flight model's guidance's geometry (genv22 eng_out_site), here to rank the sites: the leg to the circle's
# tangent, the turn onto it and round it at their cost (ap/eo-site-turn-ft-rad), the final; the margin the reach
# leaves (ft, + high) and the way over the ground (ft). The wind left out: the flight model's own counts it for the one
# chosen.
var plan = func(lat, lon, trk, s_) {
    # (each straight leg's way through the air, as the flight model's: its ground speed's share in the wind along and
    # across it; left out, the finder's margins were the flight model's less 1700 ft in 14 kt, and it changed sites
    # three times in 8 s)
    var wN = num(JSB ~ "atmosphere/total-wind-north-fps"); var wE = num(JSB ~ "atmosphere/total-wind-east-fps");
    var va = math.max(100, num(JSB ~ "velocities/vt-fps"));
    var air = func(t) {
        var al = wN * math.cos(t) + wE * math.sin(t); var ac = wE * math.cos(t) - wN * math.sin(t);
        return va / math.max(30, al + math.sqrt(math.max(1, va * va - ac * ac)));
    };
    var R = num(JSB ~ "ap/eo-site-r-ft", 4000);
    var Lf = num(JSB ~ "ap/eo-site-final-ft", 5000);
    var kt = num(JSB ~ "ap/eo-site-turn-ft-rad", 4000);
    var reach = num(JSB ~ "ap/eo-reach-nm") * 6076.12;
    var ph = num(JSB ~ "ap/eo-hac-frac", 0.8) * num(JSB ~ "ap/eo-bank-deg", 55) * D2R;
    var rate = num(JSB ~ "ap/eo-phi-rate-dps", 15) * D2R;
    var vt = math.max(100, num(JSB ~ "velocities/vt-fps"));
    var lead = 32.174 * math.tan(ph) / vt * (ph / (2 * rate) + num(JSB ~ "ap/roll-lag-s"));
    var dN = (lat - s_.lat) * FT_DEG;
    var dE = (lon - s_.lon) * FT_DEG * math.cos(lat * D2R);
    var c = s_.crs * D2R;
    var x = dE * math.sin(c) + dN * math.cos(c);
    var y = dE * math.cos(c) - dN * math.sin(c);
    var s = y < 0 ? -1 : 1;
    var u = x + Lf;
    var w = y - s * R;
    var d = math.sqrt(u * u + w * w);
    var lam = math.atan2(w, u);
    var lamT = lam + s * math.acos(math.min(1, R / math.max(d, 1)));
    var thT = m2pi(s * (-s * math.pi / 2 - lamT));
    if (thT > 2 * math.pi - lead) thT = 0;
    var L1 = math.sqrt(math.max(0, d * d - R * R));
    var turn0 = math.abs(wrap(c + lamT + s * math.pi / 2 - trk * D2R));
    return [reach - (L1 * air(c + lamT + s * math.pi / 2) + kt * (thT + turn0) + Lf * air(c)), L1 + R * thT + Lf, s, lamT, thT, R, Lf];
}

# ---------------------------------------------------------------- runways
# Each runway end of the airports within range whose length past its displaced threshold holds the landing: the
# flare's point placed for the scatter's earliest touchdown at the threshold (the approach below 50 ft over the
# airport's own clear ground)
var runways = func(range_nm) {
    var out = [];
    var near = findAirportsWithinRange(range_nm);
    if (near == nil) return out;
    foreach (var a; near) {
        var apt = airportinfo(a.id);
        if (apt == nil) continue;
        foreach (var id; keys(apt.runways)) {
            var r = apt.runways[id];
            if (r.surface == 13 or r.surface == 15) continue;                  # water; transparent
            var ground = (r.surface == 1 or r.surface == 2) ? "pavement" : "grass";
            var nd = needs(ground, head(r.heading));
            var lda = (r.length - r.threshold) * M2FT;
            if (lda < nd[1] - nd[0] + nd[4]) continue;
            var t = geo.Coord.new().set_latlon(r.lat, r.lon);
            t.apply_course_distance(r.heading, r.threshold);
            var back = (nd[3] + nd[0]) * FT2M;
            var f = geo.Coord.new(t);
            f.apply_course_distance(back > 0 ? r.heading + 180 : r.heading, math.abs(back));
            var e = geo.Coord.new(t);
            e.apply_course_distance(r.heading, lda * FT2M);
            # (its plane from the scenery's elevations at the threshold and the end; past DOWN_MAX and RISE_MAX left out,
            # falling the longer landing held)
            var et = geo.elevation(t.lat(), t.lon()); var ee = geo.elevation(e.lat(), e.lon());
            var c = {kind: "RWY", name: a.id ~ " " ~ id, lat: f.lat(), lon: f.lon(), crs: r.heading, head: head(r.heading),
                     ends: [t.lat(), t.lon(), e.lat(), e.lon()], len: lda, end: back * M2FT + lda};
            if (et != nil and ee != nil) {
                c.slope = (ee - et) * M2FT / lda;
                if (c.slope < -DOWN_MAX or c.slope > RISE_MAX) continue;
                var ns = needs(ground, head(r.heading), c.slope);
                if (lda < ns[1] - ns[0] + ns[4]) continue;
                c.elev = et * M2FT - c.slope * back * M2FT;
            }
            append(out, c);
        }
    }
    return out;
}

# ---------------------------------------------------------------- open ground
var grid = {};          # "i j": [the ground ("grass", "crop"; "" not open), its elevation, ft]
var origin = nil;       # the grid's (0, 0): [lat, lon]
var ring = [0, 0];      # the sampling's ring and place in it, about the search's centre
var centre = [0, 0];    # the search's centre, in cells
var retry = [];         # cells whose scenery was not there yet
var tries = [];         # open cells to try as strips' starts
var ti = 0;
var fields = [];
var cell_ll = func(i, j) {
    return [origin[0] + j * CELL_M * M2FT / FT_DEG, origin[1] + i * CELL_M * M2FT / (FT_DEG * math.cos(origin[0] * D2R))];
}
var ckey = func(i, j) { return (math.round(i) + 0) ~ " " ~ (math.round(j) + 0); }     # (+ 0: a rounded -0 keys as 0)
var seen = {};          # the land cover's names met, and how often (logged)
var count = {sampled: 0, bare: 0, open: 0};
var classify = func(g) {
    var m = g[1];
    if (m == nil) { seen["(none)"] = (contains(seen, "(none)") ? seen["(none)"] : 0) + 1; return ""; }
    if (size(m.names)) seen[m.names[0]] = (contains(seen, m.names[0]) ? seen[m.names[0]] : 0) + 1;
    if (!m.solid) return "";
    foreach (var n; m.names) if (contains(OPEN, n)) return OPEN[n];
    return "";
}
var sample = func(i, j, again = 0) {
    var k = ckey(i, j);
    if (contains(grid, k)) return;
    var ll = cell_ll(i, j);
    var g = geodinfo(ll[0], ll[1]);
    count.sampled += 1;
    if (g == nil) {                               # (the scenery not there yet: once more after the rings)
        count.bare += 1;
        if (!again) append(retry, [i, j]);
        return;
    }
    var ground = classify(g);
    grid[k] = [ground, g[0] * M2FT];
    if (ground != "") { append(tries, [i, j]); count.open += 1; }
}
# the next cell of the square rings about the centre, out to the search's radius
var next_cell = func(rc) {
    var k = ring[0];
    if (k > rc) return nil;
    var n = k == 0 ? 1 : 8 * k;
    var m = ring[1];
    ring[1] += 1;
    if (ring[1] >= n) { ring[0] += 1; ring[1] = 0; }
    if (k == 0) return [centre[0], centre[1]];
    var side = int(m / (2 * k));
    var p = math.mod(m, 2 * k);
    var ij = side == 0 ? [-k + p, -k] : side == 1 ? [k, -k + p] : side == 2 ? [k - p, k] : [-k, k - p];
    return [centre[0] + ij[0], centre[1] + ij[1]];
}
# a strip from cell (i, j) along course h (deg) over len ft: open all along, level within the float's height over the
# line fitted to it, rising no more than RISE_MAX nor falling more than DOWN_MAX along h; [its ground (crop if any of
# it is), the line's elevation at its start, its slope (+ rising)]; nil if not, "?" if not all sampled yet
var strip = func(i, j, h, len) {
    var n = int(math.ceil(len * FT2M / CELL_M));
    var si = math.sin(h * D2R); var co = math.cos(h * D2R);
    var el = []; var ground = "grass";
    for (var k = 0; k <= n; k += 1) {
        var c = grid[ckey(i + k * si, j + k * co)];
        if (c == nil) return "?";
        if (c[0] == "") return nil;
        if (c[0] == "crop") ground = "crop";
        append(el, c[1]);
    }
    # the line fitted (least squares) to the elevations along, ft per ft
    var step = CELL_M * M2FT; var mx = n / 2; var my = 0;
    foreach (var e; el) my += e;
    my /= n + 1;
    var sxy = 0; var sxx = 0;
    for (var k = 0; k <= n; k += 1) { sxy += (k - mx) * (el[k] - my); sxx += (k - mx) * (k - mx); }
    var sl = sxy / sxx / step;
    if (sl > RISE_MAX or sl < -DOWN_MAX) return nil;
    var e0 = my - sl * mx * step;
    var fl = num(JSB ~ "ap/eo-float-ft", 10);
    for (var k = 0; k <= n; k += 1)
        if (math.abs(el[k] - (e0 + sl * k * step)) > fl) return nil;
    return [ground, e0, sl];
}
var try_cell = func(ij) {
    if (size(fields) >= MAX_FIELDS) return;
    for (var hk = 0; hk < HEADINGS; hk += 1) {
        var h = hk * 360 / HEADINGS;
        var nd = needs("crop", head(h));           # (the longer run: the ground known only along the strip)
        var len = nd[1] - nd[0] + nd[3] - nd[2] + nd[4];
        var g = strip(ij[0], ij[1], h, len);
        if (g == nil) continue;
        if (typeof(g) == "scalar") {                            # (tried again when the cells along it are sampled, while they are)
            if (sampling) append(tries, ij);
            return;
        }
        var s0 = cell_ll(ij[0], ij[1]);
        var dup = 0;
        foreach (var f; fields)
            if (math.abs(geo.normdeg180(f.crs - h)) < 45 and math.abs(f.ends[0] - s0[0]) * FT_DEG < 3 * CELL_M * M2FT
                and math.abs(f.ends[1] - s0[1]) * FT_DEG * math.cos(s0[0] * D2R) < 3 * CELL_M * M2FT) dup = 1;
        if (dup) continue;
        var pl = g; g = pl[0];
        if (pl[2] < 0) {                                        # (falling: the longer landing it needs, along the same line)
            var nf = needs("crop", head(h), pl[2]);
            var len2 = nf[1] - nf[0] + nf[3] - nf[2] + nf[4];
            var g2 = strip(ij[0], ij[1], h, len2);
            if (g2 == nil) continue;
            if (typeof(g2) == "scalar") {
                if (sampling) append(tries, ij);
                return;
            }
            pl = g2; g = pl[0]; len = len2;
        }
        nd = needs(g, head(h), pl[2]);
        var st = geo.Coord.new().set_latlon(s0[0], s0[1]);
        var f = geo.Coord.new(st);
        var back = (nd[0] + nd[2]) * FT2M;             # the flare's point: the strip's start where the wheels come under 50 ft early
        f.apply_course_distance(back > 0 ? h + 180 : h, math.abs(back));
        var e = geo.Coord.new(st);
        e.apply_course_distance(h, len * FT2M);
        # (the plane at the flare's point: the strip's line carried back to it)
        append(fields, {kind: g, name: "FIELD", lat: f.lat(), lon: f.lon(), crs: h, head: head(h), ends: [s0[0], s0[1], e.lat(), e.lon()], len: len,
                        elev: pl[1] - pl[2] * back * M2FT, slope: pl[2], end: back * M2FT + len});
        log(sprintf("field %s, %.0f ft on %03d at %.4f %.4f, %+.1f %% along", g, len, h, s0[0], s0[1], pl[2] * 100));
        return;
    }
}

# ---------------------------------------------------------------- ranking and choice
var cands = [];         # the ranked candidates: the site's hash with its plan
var chosen = nil;       # the chosen one's key
var picked = 0;         # chosen by the crew (no automatic change)
var changed = -100;     # when the site last changed, s
var none = 0;           # the crew chose none
var key_of = func(c) { return sprintf("%s %.5f %.5f %d", c.name, c.lat, c.lon, c.crs); }
var order = func(a, b) {
    var fa = a.margin >= 0; var fb = b.margin >= 0;
    if (fa != fb) return fa ? -1 : 1;
    var na = a.margin <= SITE_NEAR_FT; var nb = b.margin <= SITE_NEAR_FT;
    if (fa and na != nb) return na ? -1 : 1;
    var ka = KIND_RANK[a.kind]; var kb = KIND_RANK[b.kind];
    if (ka != kb) return ka - kb;
    # (into the wind before downwind: a tailwind lengthens the float and the run, and the field chosen in Kansas was
    # landed on with 13 kt behind; within HEAD_KT alike, the least surplus)
    var ha = math.min(a.head, HEAD_KT * 2); var hb = math.min(b.head, HEAD_KT * 2);
    if (math.abs(ha - hb) > HEAD_KT) return hb - ha;
    # (then level or falling before rising: rising, the touchdown faster, the nacelles lower)
    var ra = contains(a, "slope") and a.slope > 0.001; var rb = contains(b, "slope") and b.slope > 0.001;
    if (ra != rb) return ra ? 1 : -1;
    return fa ? a.margin - b.margin : b.margin - a.margin;
}
var send = func(c) {
    if (c == nil) {
        setprop(JSB ~ "ap/eo-site-on", 0);
        return;
    }
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
var find = func(k) { foreach (var c; cands) if (key_of(c) == k) return c; return nil; }
var rank = func {
    var lat = num("/position/latitude-deg"); var lon = num("/position/longitude-deg");
    var trk = math.atan2(num("/velocities/speed-east-fps"), num("/velocities/speed-north-fps")) * R2D;
    var all = runways(num(JSB ~ "ap/eo-reach-nm") + 2) ~ fields;
    foreach (var c; all) {
        var pl = plan(lat, lon, trk, c);
        c.margin = pl[0]; c.path = pl[1]; c.plan = pl;
    }
    # (the one flown: the flight model's own margin and way, its plan from where the aircraft is on it: this one's
    # geometry is the leg's, and on the final it read a whole circle more)
    var st = num(JSB ~ "ap/eo-site-state");
    var cur = chosen == nil ? nil : find(chosen);
    if (cur != nil and st > 0.5) {
        cur.margin = num(JSB ~ "ap/eo-site-margin-ft"); cur.path = num(JSB ~ "ap/eo-site-path-ft");
    }
    cands = sort(all, order);
    if (none) return;
    var best = size(cands) and cands[0].margin >= 0 ? cands[0] : nil;
    # (out of reach by more than where the flare may begin short of its point, the scatter's: KSFO 01L was left at
    # -39 ft for its neighbour; and not once on the circle)
    var short = cur == nil ? 0 : -needs(cur.kind == "RWY" ? "pavement" : cur.kind)[0];
    if (cur == nil and !picked and best != nil) {
        chosen = key_of(best); send(best);
        log(sprintf("chosen %s %s, margin %.0f ft, path %.2f NM", best.kind, best.name, best.margin, best.path / 6076.12));
    } elsif (cur != nil and cur.margin < -short and !picked and st < 1.5 and best != nil and key_of(best) != chosen
             and num("/sim/time/elapsed-sec") - changed > HOLD_S) {
        log(sprintf("%s out of reach (%.0f ft): %s %s, margin %.0f ft", cur.name, cur.margin, best.kind, best.name, best.margin));
        chosen = key_of(best); send(best); changed = num("/sim/time/elapsed-sec");
    } elsif (cur != nil and !picked and st < 1.5 and best != nil and key_of(best) != chosen and order(best, cur) < 0
             and (cur.margin > SITE_NEAR_FT and best.margin <= SITE_NEAR_FT or best.head - cur.head > HEAD_KT)
             and num("/sim/time/elapsed-sec") - changed > HOLD_S) {
        # (a better one found while the search goes on, before the circle: one near its energy for one far above it,
        # or one more into the wind; the first choice, made on the first field found, had 2.3 NM to spend)
        log(sprintf("%s %s (margin %.0f ft, head %.0f kt) for %s %s (margin %.0f ft, head %.0f kt)", best.kind, best.name, best.margin, best.head,
                    cur.kind, cur.name, cur.margin, cur.head));
        chosen = key_of(best); send(best); changed = num("/sim/time/elapsed-sec");
    }
    if (math.mod(int(num("/sim/time/elapsed-sec")), 10) == 0) {
        var names = [];
        foreach (var n; keys(seen)) append(names, n ~ " " ~ seen[n]);
        log(sprintf("land cover: %d sampled (%d without scenery), %d open, %d tried, %d fields, ring %d; %s",
                    count.sampled, count.bare, count.open, ti, size(fields), ring[0], string.join(", ", names)));
    }
    if (cur != nil and math.mod(int(num("/sim/time/elapsed-sec")), 5) == 0) {
        log(sprintf("%s %s: margin %.0f ft here, %.0f the flight model's; path %.2f NM, state %d",
                    cur.kind, cur.name, cur.margin, num(JSB ~ "ap/eo-site-margin-ft"), cur.path / 6076.12, st));
    }
}
# the next candidate (Ctrl-L, the NAV format's SITE key): those within reach (or all, if none is) in their order,
# then none (ENG OUT's heading), then the first again; not once flaring
var next = func {
    if (!num(AFCS ~ "eng-out") or num(JSB ~ "ap/eo-phase") > 2.5) return;
    var feas = [];
    foreach (var c; cands) if (c.margin >= 0) append(feas, c);
    if (!size(feas)) feas = cands;
    var i = -1;
    forindex (var k; feas) if (chosen != nil and key_of(feas[k]) == chosen) i = k;
    picked = 1;
    if (!none and i + 1 >= size(feas)) {
        none = 1; chosen = nil; send(nil);
        log("crew chose none");
        return;
    }
    var c = feas[none ? 0 : i + 1];
    if (c == nil) return;
    none = 0; chosen = key_of(c); send(c);
    log("crew chose " ~ c.kind ~ " " ~ c.name);
}

# ---------------------------------------------------------------- the displays' copy
var publish = func {
    var n = math.min(MAX_SHOWN, size(cands));
    setprop(OUT ~ "n", n);
    for (var i = 0; i < n; i += 1) {
        var c = cands[i];
        var p = OUT ~ "cand[" ~ i ~ "]/";
        setprop(p ~ "lat0", c.ends[0]); setprop(p ~ "lon0", c.ends[1]);
        setprop(p ~ "lat1", c.ends[2]); setprop(p ~ "lon1", c.ends[3]);
        setprop(p ~ "kind", c.kind);
        setprop(p ~ "reach", c.margin >= 0);
        setprop(p ~ "chosen", chosen != nil and key_of(c) == chosen);
    }
    var cur = chosen == nil ? nil : find(chosen);
    setprop(OUT ~ "on", cur != nil);
    if (cur == nil) return;
    setprop(OUT ~ "name", cur.kind == "RWY" ? "RWY " ~ cur.name : cur.kind == "crop" ? "FIELD CROP" : "FIELD GRASS");
    setprop(OUT ~ "path-nm", cur.path / 6076.12);
    setprop(OUT ~ "margin-nm", cur.margin / 6076.12);
    # the planned path for the map: the leg to the circle's tangent point, round it, the final, to the flare's point
    var pl = cur.plan; var s = pl[2]; var lamT = pl[3]; var thT = pl[4]; var R = pl[5]; var Lf = pl[6];
    var c_ = cur.crs * D2R;
    var pt = func(x, y) {
        var dn = x * math.cos(c_) - y * math.sin(c_); var de = x * math.sin(c_) + y * math.cos(c_);
        [cur.lat + dn / FT_DEG, cur.lon + de / (FT_DEG * math.cos(cur.lat * D2R))];
    };
    var pts = [[num("/position/latitude-deg"), num("/position/longitude-deg")]];
    # (flown: what is left of the flight model's own plan, round the circle from abeam the aircraft on its side, or
    # the final)
    var st = num(JSB ~ "ap/eo-site-state");
    if (st > 1.5) {
        s = num(JSB ~ "ap/eo-site-s", s); R = num(JSB ~ "ap/eo-site-r-ft", R); Lf = num(JSB ~ "ap/eo-site-final-ft", Lf);
        lamT = math.atan2(num(JSB ~ "ap/eo-site-y-ft") - s * R, num(JSB ~ "ap/eo-site-x-ft") + Lf);
        thT = st > 2.5 ? 0 : num(JSB ~ "ap/eo-site-arc-rad");
    }
    if (st < 2.5) {
        var steps = int(math.max(1, thT / (10 * D2R)));
        for (var k = 0; k <= steps; k += 1) {
            var l = lamT + s * thT * k / steps;
            append(pts, pt(-Lf + R * math.cos(l), s * R + R * math.sin(l)));
        }
    }
    append(pts, pt(0, 0));
    setprop(OUT ~ "path/n", size(pts));
    forindex (var k; pts) { setprop(OUT ~ "path/pt[" ~ k ~ "]/lat", pts[k][0]); setprop(OUT ~ "path/pt[" ~ k ~ "]/lon", pts[k][1]); }
}

# ---------------------------------------------------------------- the loop
var was = 0;
var ticks = 0;
var sampling = 1;       # the rings still being sampled
var reset = func {
    grid = {}; origin = nil; ring = [0, 0]; retry = []; tries = []; ti = 0; fields = []; cands = [];
    chosen = nil; picked = 0; none = 0; sampling = 1; seen = {}; count = {sampled: 0, bare: 0, open: 0}; changed = -100;
    setprop(OUT ~ "n", 0); setprop(OUT ~ "on", 0);
    send(nil);
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
    if (!was) {
        reset();
        if (!size(W) and !load()) return;
        # the search about the point a third of the reach ahead along the track (the sites behind cost the turn back)
        var lat = num("/position/latitude-deg"); var lon = num("/position/longitude-deg");
        origin = [lat, lon];
        var trk = math.atan2(num("/velocities/speed-east-fps"), num("/velocities/speed-north-fps"));
        var ahead = math.min(SEARCH_NM, num(JSB ~ "ap/eo-reach-nm")) / 3 * 1852;
        centre = [math.round(ahead * math.sin(trk) / CELL_M), math.round(ahead * math.cos(trk) / CELL_M)];
        log(sprintf("on: reach %.1f NM, the land cover searched about %.1f NM ahead", num(JSB ~ "ap/eo-reach-nm"), ahead / 1852));
    }
    was = 1;
    var rc = int(math.min(SEARCH_NM, num(JSB ~ "ap/eo-reach-nm") + 1) * 1852 / CELL_M);
    for (var k = 0; k < PER_TICK; k += 1) {
        var ij = next_cell(rc);
        if (ij == nil) {
            if (!size(retry)) { sampling = 0; break; }
            ij = retry[0]; retry = subvec(retry, 1);
            sample(ij[0], ij[1], 1);
        } else sample(ij[0], ij[1]);
    }
    for (var k = 0; k < TRY_PER_TICK and ti < size(tries); k += 1) { try_cell(tries[ti]); ti += 1; }
    ticks += 1;
    if (math.mod(ticks, RANK_TICKS) == 0) { rank(); publish(); }
}
var timer = nil;         # kept: a timer nothing refers to is collected, and stops
setlistener("/sim/signals/fdm-initialized", func(n) {
    if (!n.getValue() or timer != nil) return;
    load();
    timer = maketimer(TICK_S, update);
    timer.start();
}, 1, 0);
