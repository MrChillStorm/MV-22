# The autopilot's approach line and its keys. The flight model's autopilot (fdm/genv22.py, autopilot) flies VOR/ILS
# and APPR along a course line through a point, APPR with a path at an angle down to a hover 50 ft over the point;
# here the line is made from:
#   - the NAV 1 radio's ILS: the localizer's centreline, the glide slope's angle and siting (the path is the glide
#     slope's own, which crosses the threshold at about 50 ft: the hover is over the threshold);
#   - the NAV 1 radio's VOR (VOR/ILS only): the station, on the CRS knob's course;
#   - the route manager's route (APPR only): its destination runway (as an ILS's, at 3 deg) or, with none, its last
#     waypoint as a landing zone, into the wind (along the last leg in a calm), at 6 deg.
# WYPT's turns between the route's legs are made here too (turns, below).
# WYPT flies a route's whole plan, as the NAV format draws it: APPR is armed with the route, to a hover at its last
# point (the landing zone), its speed and path along the route's distance still to go (turns: ap/route-togo-ft).
# The keys: VOR/ILS on and off; APPR on (to the hover), again (and land), again (off); GA (go around: the approach and
# the hover hold off, the heading held, 1000 ft above or the ALT knob's altitude, at least 80 kt, AUTO NAC); ENGOUT
# (AFCS/press only: no key on the panel) ENG OUT on and off, which both engines stopping in the air turn on.

var JSB = "/fdm/jsbsim/";
var AFCS = "/instrumentation/mv22b/afcs/";
var ST = "/autopilot/settings/";
var NAV = "/instrumentation/nav[0]/";
var RM = "/autopilot/route-manager/";
var GPSW = "/instrumentation/gps/wp/";

var val = func(path, d = 0) { var x = getprop(path); x == nil ? d : x; }

var label = "";             # the line's name on the mode annunciator: ILS 28R, VOR SFO, KSFO 28R, LZ WP3
var source = "";            # what it was made from (a new one is made when this changes: the radio retuned, the CRS knob)
var alert = "";             # APPR or VOR/ILS refused, for the annunciator
var elev_at = nil;          # the point whose elevation the scenery didn't have yet: asked again until it does

var set_line = func(lat, lon, crs, elev_ft, gs, ang, name, from) {
    setprop(JSB ~ "ap/line-lat-deg", lat);
    setprop(JSB ~ "ap/line-lon-deg", lon);
    setprop(JSB ~ "ap/line-course-deg", math.mod(crs + 360, 360));
    setprop(JSB ~ "ap/line-elev-ft", elev_ft);
    setprop(JSB ~ "ap/line-gs", gs);
    setprop(JSB ~ "ap/line-angle-deg", ang);
    setprop(JSB ~ "ap/line-valid", 1);
    label = name;
    source = from;
    return 1;
}

var elevation_ft = func(lat, lon, d_m) {
    var e = geo.elevation(lat, lon);
    elev_at = e == nil ? [lat, lon] : nil;
    return (e != nil ? e : d_m) * M2FT;
}

# c moved d metres along crs (back along it when d < 0)
var moved = func(c, crs, d) {
    var m = geo.Coord.new(c);
    m.apply_course_distance(d < 0 ? crs + 180 : crs, math.abs(d));
    return m;
}

# the navaids of a type on freq (MHz) within nm, nearest first
var by_freq = func(freq, type, nm = 30) {
    var out = [];
    foreach (var n; findNavaidsWithinRange(nm, type)) {
        var f = n.frequency;
        if (f == nil) continue;
        if (f > 1000) f /= 100;                                # kept in 10 kHz
        if (math.abs(f - freq) < 0.005) append(out, n);
    }
    return out;
}

# the runway with its ILS on freq, among the airports within 30 nm (airportinfo's, which carry their runways)
var ils_runway = func(freq) {
    foreach (var a; findAirportsWithinRange(30)) {
        var apt = airportinfo(a.id);
        if (apt != nil) foreach (var id; keys(apt.runways)) {
            var r = apt.runways[id];
            var f = r.ils_frequency_mhz;
            if (f != nil and math.abs(f - freq) < 0.005) return [apt, r];
        }
    }
    return nil;
}

# the glide slope on freq by the threshold: its angle (FlightGear keeps it as angle * 100000 + course) and its
# distance along the runway past the threshold; without one, 3 deg from the aiming point 1000 ft in
var glide = func(freq, thr, hdg) {
    if (freq != nil)
        foreach (var n; by_freq(freq, "gs")) {
            var c = geo.Coord.new().set_latlon(n.lat, n.lon);
            var d = thr.distance_to(c);
            if (d > 4000) continue;
            var ang = (n.course != nil and n.course > 1000) ? int(n.course / 1000) / 100 : 3;
            return [ang > 1 and ang < 10 ? ang : 3, d * math.cos((thr.course_to(c) - hdg) * D2R)];
        }
    return [3, 1000 * FT2M];
}

# a runway's line: the point where the glide path is at the hover's height over the runway (the hover over it)
var runway_line = func(apt, r, freq, name, from) {
    var thr = geo.Coord.new().set_latlon(r.lat, r.lon);
    thr.apply_course_distance(r.heading, r.threshold);         # past a displaced threshold
    var g = glide(freq, thr, r.heading);
    var pt = moved(thr, r.heading, g[1] - val(JSB ~ "ap/hover-ft", 50) * FT2M / math.tan(g[0] * D2R));
    return set_line(pt.lat(), pt.lon(), r.heading, elevation_ft(pt.lat(), pt.lon(), apt.elevation), 1, g[0], name, from);
}

# the runway most into the wind (270 in a calm), the longer and wider the better (as glide_slope_tunnel.nas)
var best_runway = func(apt) {
    var w = val("/environment/wind-speed-kt") > 3 ? val("/environment/wind-from-heading-deg") : 270;
    var best = nil; var score = -1;
    foreach (var id; keys(apt.runways)) {
        var r = apt.runways[id];
        var dev = math.abs(geo.normdeg180(w - r.heading)) + 1e-6;
        var s = (0.01 * r.length + 0.01 * r.width) / dev;
        if (s > score) { score = s; best = r; }
    }
    return best;
}

# the ILS on NAV 1's frequency: the localizer's centreline (the radio's course, true, at its antenna), the glide
# slope's origin (its antenna abeam on the centreline) and angle (FlightGear keeps it as angle * 100000 + course); a
# localizer alone, its runway's aiming point (1000 ft past the threshold, 3 deg)
var ils_line = func(gs) {
    var freq = val(NAV ~ "frequencies/selected-mhz");
    var loc = by_freq(freq, "ils");
    if (!size(loc)) loc = by_freq(freq, "loc");
    if (!size(loc)) return 0;
    var n = loc[0];
    var crs = val(NAV ~ "radials/target-radial-deg");
    var ref = geo.Coord.new().set_latlon(n.lat, n.lon);
    var g = by_freq(freq, "gs");
    var c = size(g) ? geo.Coord.new().set_latlon(g[0].lat, g[0].lon) : nil;
    if (c != nil and ref.distance_to(c) < 6000) {
        var along = ref.distance_to(c) * math.cos((ref.course_to(c) - crs) * D2R);
        var ang = (g[0].course != nil and g[0].course > 1000) ? int(g[0].course / 1000) / 100 : 3;
        if (ang < 1 or ang > 10) ang = 3;
        var pt = moved(ref, crs, along - val(JSB ~ "ap/hover-ft", 50) * FT2M / math.tan(ang * D2R));
        crs = pt.course_to(ref);                               # the beam's course there (a great circle's turns)
        return set_line(pt.lat(), pt.lon(), crs, elevation_ft(pt.lat(), pt.lon(), g[0].elevation), gs, ang, "ILS " ~ n.id,
                        sprintf("ILS %.2f", freq));
    }
    var ar = ils_runway(freq);
    if (ar == nil) return 0;
    if (!runway_line(ar[0], ar[1], nil, "LOC " ~ n.id, sprintf("ILS %.2f", freq))) return 0;
    if (!gs) setprop(JSB ~ "ap/line-gs", 0);
    return 1;
}

var vor_line = func {
    var freq = val(NAV ~ "frequencies/selected-mhz");
    var n = by_freq(freq, "vor", 200);
    if (!size(n)) return 0;
    var crs = val(NAV ~ "radials/selected-deg");
    elev_at = nil;
    return set_line(n[0].lat, n[0].lon, crs + val("/environment/magnetic-variation-deg"), n[0].elevation * M2FT, 0, 3,
                    "VOR " ~ n[0].id, sprintf("VOR %.2f %d", freq, math.round(crs)));
}

var route_line = func {
    if (!val(RM ~ "active")) return 0;
    var fp = flightplan();
    if (fp == nil or fp.getPlanSize() < 1) return 0;
    var apt = fp.destination != nil ? airportinfo(fp.destination.id) : nil;
    if (apt != nil) {
        var r = fp.destination_runway;
        if (r == nil) r = best_runway(apt);
        if (r != nil)
            return runway_line(apt, r, r.ils_frequency_mhz, apt.id ~ " " ~ r.id, "RWY " ~ apt.id ~ " " ~ r.id);
    }
    # the last waypoint, a landing zone
    var w = fp.getWP(fp.getPlanSize() - 1);
    if (w == nil) return 0;
    var crs = val("/environment/wind-speed-kt") >= 5 ? val("/environment/wind-from-heading-deg")
            : w.leg_bearing != nil ? w.leg_bearing
            : geo.aircraft_position().course_to(geo.Coord.new().set_latlon(w.wp_lat, w.wp_lon));
    return set_line(w.wp_lat, w.wp_lon, crs, elevation_ft(w.wp_lat, w.wp_lon, 0), 1, 6, "LZ " ~ w.wp_name,
                    sprintf("LZ %s %.5f %.5f", w.wp_name, w.wp_lat, w.wp_lon));
}

# the line for APPR (an ILS tuned and received, else the route) or VOR/ILS (the radio's ILS or VOR); 1 when made
var make_line = func(appr) {
    var ok = 0;
    var radio = val(NAV ~ "in-range");
    call(func {
        if (radio and val(NAV ~ "nav-loc")) ok = ils_line(appr or val(NAV ~ "has-gs"));
        elsif (radio and !appr) ok = vor_line();
        if (!ok and appr) ok = route_line();
    }, [], nil, nil, var err = []);
    if (size(err)) logprint(LOG_WARN, "MV-22B autopilot: ", err[0], " (", err[1], ":", err[2], ")");
    if (ok) logprint(LOG_INFO, sprintf("MV-22B autopilot: %s line %.6f %.6f course %.1f elevation %.0f ft path %d at %.2f deg",
                                       label, val(JSB ~ "ap/line-lat-deg"), val(JSB ~ "ap/line-lon-deg"), val(JSB ~ "ap/line-course-deg"),
                                       val(JSB ~ "ap/line-elev-ft"), val(JSB ~ "ap/line-gs"), val(JSB ~ "ap/line-angle-deg")));
    else logprint(LOG_INFO, "MV-22B autopilot: no line (", appr ? "APPR" : "VOR/ILS", ")");
    return ok;
}

var refused = func(what) {
    alert = what ~ " UNAVAIL";
    settimer(func { alert = ""; }, 5);
}

var vorils = func {
    if (val(AFCS ~ "vor-ils")) return setprop(AFCS ~ "vor-ils", 0);
    if (!make_line(0)) return refused("VOR/ILS");
    setprop(AFCS ~ "appr", 0);
    setprop(AFCS ~ "vor-ils", 1);
}

var declined = "";          # the route whose approach the pilot turned off (not armed again for it)
var route_key = func {
    var last = RM ~ "route/wp[" ~ (int(val(RM ~ "route/num")) - 1) ~ "]/";
    sprintf("%d %.5f %.5f", val(RM ~ "route/num"), val(last ~ "latitude-deg"), val(last ~ "longitude-deg"));
}
var arm_route = func {
    if (!val(AFCS ~ "wypt") or !val(RM ~ "active") or val(AFCS ~ "appr") or val(AFCS ~ "vor-ils") or declined == route_key()) return;
    var ok = 0;
    call(func { ok = route_line(); }, [], nil, nil, var err = []);
    if (!ok) return;
    setprop(JSB ~ "ap/app-land", 0);
    setprop(AFCS ~ "appr", 1);
    logprint(LOG_INFO, "MV-22B autopilot: APPR armed with the route, to ", label);
}

var appr = func {
    if (!val(AFCS ~ "appr")) {
        if (!make_line(1)) return refused("APPR");
        setprop(JSB ~ "ap/app-land", 0);
        setprop(AFCS ~ "vor-ils", 0);
        setprop(AFCS ~ "appr", 1);
    } elsif (!val(JSB ~ "ap/app-land") and val(JSB ~ "ap/app-state") < 3.5)
        setprop(JSB ~ "ap/app-land", 1);                       # and land
    else {
        setprop(AFCS ~ "appr", 0);
        if (val(AFCS ~ "wypt") and val(RM ~ "active")) declined = route_key();
    }
}

var ga = func {
    var h = val("/instrumentation/altimeter/indicated-altitude-ft");
    foreach (var m; ["appr", "vor-ils", "hvr-hold"]) setprop(AFCS ~ m, 0);
    setprop(JSB ~ "ap/app-land", 0);
    setprop(JSB ~ "hh/engage-cmd", 0);
    setprop(ST ~ "heading-bug-deg", math.round(val("/orientation/heading-magnetic-deg")));
    if (val(ST ~ "target-altitude-ft") < h + 200) setprop(ST ~ "target-altitude-ft", math.round(h / 100) * 100 + 1000);
    setprop(ST ~ "target-speed-kt", math.max(80, val(ST ~ "target-speed-kt")));
    foreach (var m; ["hdg-hold", "alt-hold", "spd-hold", "auto-nac"]) setprop(AFCS ~ m, 1);
    setprop(AFCS ~ "ga", 1);
}

# the mode annunciator's lateral, vertical and approach words (displays.nas, afcs_modes)
var STATES = ["", "", " HOVER", " HOVER", " LAND", " DOWN"];
# ENG OUT's phases (the flight model's eng_out); the gear asked for while up below the mode's gear height or flaring
var EO_PHASES = ["", " DIVE", " GLIDE", " FLARE", " LAND", " DOWN"];
var annunciate = func(s) {
    if (val(AFCS ~ "eng-out")) {
        var ph = math.max(0, math.min(5, int(val(JSB ~ "ap/eo-phase"))));
        var t = "ENG OUT" ~ EO_PHASES[ph] ~ (ph == 2 ? sprintf(" %d", val(JSB ~ "ap/eo-vg-kt")) : "");
        if (ph < 5 and val(JSB ~ "gear/gear-pos-norm") < 0.99
            and (ph >= 3 or val("/position/gear-agl-m") * M2FT < val(JSB ~ "ap/eo-gear-ft")))
            t ~= " GEAR";
        append(s, t);
    }
    var line = val(JSB ~ "ap/line"); var want = val(AFCS ~ "vor-ils") or val(AFCS ~ "appr");
    if (want and val(JSB ~ "ap/line-valid") and !val(JSB ~ "hh/on"))
        append(s, label ~ (line ? "" : " ARM"));
    if (val(JSB ~ "ap/gp")) append(s, sprintf("GP %.1f", val(JSB ~ "ap/line-angle-deg")));
    elsif (line and val(JSB ~ "ap/line-gs")) append(s, "GP ARM");
    if (val(AFCS ~ "appr")) {
        var st = math.max(0, math.min(5, int(val(JSB ~ "ap/app-state"))));
        var t = "APPR" ~ STATES[st];
        # the landing armed, but the gear up: the hover held till it is down
        if (st < 4 and val(JSB ~ "ap/app-land"))
            t ~= st < 2 ? " LAND" : st == 3 and val(JSB ~ "gear/gear-pos-norm") < 0.99 ? " GEAR" : " LAND ARM";
        if (st == 1) t ~= sprintf(" %.1f", val(JSB ~ "ap/app-dist-ft") / 6076.1);
        append(s, t);
    }
    if (val(AFCS ~ "ga")) {
        if (!val(AFCS ~ "alt-hold") or math.abs(val(ST ~ "target-altitude-ft") - val("/instrumentation/altimeter/indicated-altitude-ft")) < 100)
            setprop(AFCS ~ "ga", 0);
        else
            append(s, "GA");
    }
    if (alert != "") append(s, alert);
}

# the line kept up to date while a mode wants it (a new one when the radio or the CRS knob changes); off with them
var key_of = func(appr) {
    if (substr(source, 0, 3) == "RWY" or substr(source, 0, 2) == "LZ") return source;     # the route's: kept
    var radio = val(NAV ~ "in-range");
    var f = val(NAV ~ "frequencies/selected-mhz");
    if (radio and val(NAV ~ "nav-loc")) return sprintf("ILS %.2f", f);
    if (radio and !appr) return sprintf("VOR %.2f %d", f, math.round(val(NAV ~ "radials/selected-deg")));
    return source;          # the route's: kept as it was made
}
var update = func {
    var appr = val(AFCS ~ "appr");
    if (!appr and !val(AFCS ~ "vor-ils")) {
        if (val(JSB ~ "ap/line-valid")) setprop(JSB ~ "ap/line-valid", 0);
        source = ""; label = "";
        return;
    }
    if (val(JSB ~ "ap/app-state") < 1.5 and key_of(appr) != source) make_line(appr);
    if (elev_at != nil) {
        var e = geo.elevation(elev_at[0], elev_at[1]);
        if (e != nil) { setprop(JSB ~ "ap/line-elev-ft", e * M2FT); elev_at = nil; }
    }
}

# ---------------------------------------------------------------- WYPT's turns between legs
# Where an arc of the aircraft's turn radius (at 80 % of the bank limit the flight model's autopilot allows at this
# speed) joins the leg flown and the next, less half the roll into it: the arc's centre and radius for the flight
# model, which flies it, and the route moved on to the next leg. (FlightGear's own fly-by turned at a set rate and
# overshot the next leg in the larger turns.) Up to 150 deg, once on the leg; established too close for it, a tighter
# arc that fits, down to the full bank limit's. Past 150 deg the waypoint is flown over. The route's first leg, from
# wherever it was activated, is flown straight to its waypoint (ap/direct).
var bank_limit = func(kcas) {
    kcas < 40 ? 15 : kcas < 80 ? 15 + (kcas - 40) / 8 : kcas < 150 ? 20 + (kcas - 80) / 14 : 25;
}
var wp_coord = func(i) {
    geo.Coord.new().set_latlon(val(RM ~ "route/wp[" ~ i ~ "]/latitude-deg"), val(RM ~ "route/wp[" ~ i ~ "]/longitude-deg"));
}
var TURNS_S = 0.25;
var turns = func {
    var nav = val(JSB ~ "ap/nav");
    var cur = int(val(RM ~ "current-wp"));
    var n = int(val(RM ~ "route/num"));
    var direct = (nav and cur == 0) ? 1 : 0;
    if (val(JSB ~ "ap/direct") != direct) setprop(JSB ~ "ap/direct", direct);
    arm_route();
    # the route's distance still to go to its last point: to the active waypoint along its leg, then the legs on
    if (nav and cur >= 0 and cur < n) {
        var pos = geo.aircraft_position();
        var a = wp_coord(cur);
        var togo = math.max(0, pos.distance_to(a) * math.cos((pos.course_to(a) - val(GPSW ~ "leg-true-course-deg")) * D2R));
        if (direct) togo = pos.distance_to(a);
        for (var i = cur; i + 1 < n; i += 1) togo += wp_coord(i).distance_to(wp_coord(i + 1));
        setprop(JSB ~ "ap/route-togo-ft", togo * M2FT);
    }
    if (!nav or val(JSB ~ "ap/arc-on")) return;
    if (cur < 0 or cur + 1 >= n) return;
    var a = wp_coord(cur);
    var c1 = direct ? val(GPSW ~ "wp[1]/bearing-true-deg") : val(GPSW ~ "leg-true-course-deg");
    var c2 = a.course_to(wp_coord(cur + 1));
    var d = geo.normdeg180(c2 - c1);
    if (math.abs(d) < 1 or math.abs(d) > 150) return;
    var trk = math.atan2(val("/velocities/speed-east-fps"), val("/velocities/speed-north-fps")) * R2D;
    if (math.abs(geo.normdeg180(trk - c1)) > 15) return;                    # established on the leg first
    var vg = val("/velocities/groundspeed-kt") * 1.6878;
    var phi = 0.8 * bank_limit(val("/instrumentation/airspeed-indicator/indicated-speed-kt"));
    var r = math.max(100, vg * vg / (32.174 * math.tan(phi * D2R)));        # ft
    var k = math.tan(math.abs(d) / 2 * D2R);
    # the roll into it as a delay: half its ramp at 5 deg/s, the augmentation's own (measured), half this period
    var roll = vg * (phi / 5 / 2 + val(JSB ~ "ap/roll-lag-s") + TURNS_S / 2);
    var pos = geo.aircraft_position();
    var along = pos.distance_to(a) * M2FT * math.cos((pos.course_to(a) - c1) * D2R);
    if (along > r * k + roll) return;
    var r_full = vg * vg / (32.174 * math.tan(phi / 0.8 * D2R));            # at the full bank limit
    r = math.max(r_full, math.min(r, (along - roll) / k));
    var t = r * k;                                                          # the tangent point, ft before the waypoint
    var dir = d > 0 ? 1 : -1;
    var ctr = moved(moved(a, c1, -t * FT2M), c1 + 90 * dir, r * FT2M);
    setprop(JSB ~ "ap/arc-lat-deg", ctr.lat());
    setprop(JSB ~ "ap/arc-lon-deg", ctr.lon());
    setprop(JSB ~ "ap/arc-r-ft", r);
    setprop(JSB ~ "ap/arc-dir", dir);
    setprop(JSB ~ "ap/arc-end-deg", c2);
    setprop(JSB ~ "ap/arc-on", 1);
    setprop(RM ~ "current-wp", cur + 1);
}

# ---------------------------------------------------------------- TF: terrain following
# The terrain ahead along the ground track (curving at the present turn rate), every 1.25 s of flight out to 45 s
# (time to clear 1000 ft of rise at the conversion's least climb, 24 ft/s at 150 kt, with its pull-up; 30 s left a
# ridge 55 ft short there; at 250 kt, 22 ft/s, about 930 ft), the highest of the track and 150 ft either side of it,
# with the last sweep's points (the 4 plans before, at their time to go now: 0.25 s apart between them, where one
# sweep's alone missed a ridge's edge by up to 1.25 s): the heights the wheels must clear, the terrain and the set
# clearance (the HVR ALT knob, at least 100 ft). Planned for the flight model's reference path, which the aircraft
# follows as its model of it does (ap/tf-model-ft: two lags fitted to its climb's response): a vertical speed v
# commanded now is reached by a ramp from the reference's at the ride's acceleration (the flight model's rate limit)
# and held to the next plan, and the most height it leaves the aircraft by t is with a pull-up after that at half the
# ride's acceleration (the margin) up to the climb its power makes (the flight model's): for each point the least v
# that still clears it so (by bisection: that height rises with v), the command the largest of them. It descends into
# a valley and pulls up as late as half the ride allows; within 2000 ft/min of descent. No terrain data: the flight
# model climbs (TF FAIL).
var TF_N = 36; var TF_T = 45; var TF_W = 150;
var TF_RIDES = [["SOFT", 0.1], ["MED", 0.2], ["HARD", 0.3]];
var TF_DESC = 2000 / 60;
var tf_climb = 0;                                         # ft/s: the climb the aircraft can make (ap/tf-climb-cap-fps)
var TF_S = 0.25;
var tf_trk = nil; var tf_rate = 0;
var tf_past = [];                                         # the sweep's plans' points: [the time they are met, s; ft]
var tf_last = nil; var tf_hold = TF_S;                    # the last plan's time, s; the time to the next (as the last)

# the most height (ft) the aircraft gains by t (s), the reference commanded v now: ramped from v0 at a1 and held to the
# next plan (the timer's interval: up to a frame late, a hold of 0.25 s had the plan short of its own path, and a
# climb at once), then pulled up at a2 to the climb limit; the aircraft as its model (tf_m: [share, time constant s, its
# vertical speed ft/s] per lag) follows it, in closed form along each stretch of it (the reference's speed linear there)
var tf_m = [];
var tf_reach = func(v, t, v0, a1, a2) {
    var t1 = math.abs(v - v0) / a1;
    var segs = [[math.min(t, t1), v0, v > v0 ? a1 : -a1]];
    var th = math.max(t1, tf_hold);
    if (t > t1) append(segs, [math.min(t, th) - t1, v, 0]);
    if (t > th) {
        var s = t - th; var t2 = math.max(0, tf_climb - v) / a2;
        append(segs, [math.min(s, t2), v, a2]);
        if (s > t2) append(segs, [s - t2, tf_climb, 0]);
    }
    var out = 0;
    foreach (var m; tf_m) {
        var tau = m[1]; var x = m[2]; var r = 0;
        foreach (var g; segs) {
            var d = g[0]; var vs = g[1]; var k = g[2];
            var c = x - vs + tau * k; var e = math.exp(-d / tau);
            r += vs * d + k * d * d / 2 - tau * k * d + c * tau * (1 - e);
            x = vs + k * d - tau * k + c * e;
        }
        out += m[0] * r;
    }
    return out;
}

# the least vertical speed (ft/s) to command now that still gains dh (ft) by t
var tf_need = func(dh, t, v0, a1, a2) {
    if (tf_reach(-TF_DESC, t, v0, a1, a2) >= dh) return -TF_DESC;
    if (tf_reach(tf_climb, t, v0, a1, a2) < dh) return tf_climb;
    var lo = -TF_DESC; var hi = tf_climb;
    for (var k = 0; k < 12; k += 1) {
        var m = (lo + hi) / 2;
        if (tf_reach(m, t, v0, a1, a2) >= dh) hi = m; else lo = m;
    }
    return hi;
}

var tf = func {
    var vn = val("/velocities/speed-north-fps"); var ve = val("/velocities/speed-east-fps");
    var trk = math.atan2(ve, vn);
    var now = val("/sim/time/elapsed-sec");
    var dt = tf_last == nil ? TF_S : math.max(1e-3, now - tf_last);
    tf_last = now;
    if (tf_trk != nil) tf_rate += (math.atan2(math.sin(trk - tf_trk), math.cos(trk - tf_trk)) / dt - tf_rate) * 0.5;
    tf_trk = trk;
    if (!val(AFCS ~ "tf")) { tf_past = []; return; }
    tf_hold = math.max(TF_S, dt);
    var vg = math.sqrt(vn * vn + ve * ve);
    var pos = geo.aircraft_position();
    # the model's wheels, ft above sea level
    var hw = val("/position/ground-elev-ft") + val(JSB ~ "hh/h-wheels-ft") + val(JSB ~ "ap/tf-model-ft") - val("/position/altitude-ft");
    var v0 = val(JSB ~ "ap/tf-cmd-fps");
    var clr = math.max(100, val(ST ~ "target-agl-ft"));
    var a = val(JSB ~ "ap/tf-ride-g") * 32.174;
    tf_climb = math.max(1, val(JSB ~ "ap/tf-climb-cap-fps"));
    var tr = 1 / math.max(0.05, val(JSB ~ "ap/tf-kh"));
    var w = val(JSB ~ "ap/tf-w");
    tf_m = [[w, val(JSB ~ "ap/tf-tau1-s", 1), val(JSB ~ "ap/tf-v1-fps")], [1 - w, val(JSB ~ "ap/tf-tau2-s", 1), val(JSB ~ "ap/tf-v2-fps")]];
    var ok = 1; var pts = [];
    var p = geo.Coord.new(pos); var h = trk; var step = vg * TF_T / TF_N * FT2M;
    for (var i = 1; i <= TF_N; i += 1) {
        p.apply_course_distance((h + tf_rate * TF_T / TF_N / 2) * R2D, step);      # along the arc, its heading at mid-step
        h += tf_rate * TF_T / TF_N;
        var e = -1e9;
        foreach (var side; [-1, 0, 1]) {
            var q = side ? moved(p, h * R2D + 90 * side, TF_W * FT2M) : p;
            var z = geo.elevation(q.lat(), q.lon());
            if (z == nil) { ok = 0; continue; }
            e = math.max(e, z * M2FT);
        }
        if (e > -1e8) append(pts, [now + TF_T * i / TF_N, e]);
    }
    append(tf_past, pts);
    if (size(tf_past) > TF_T / TF_N / TF_S) tf_past = subvec(tf_past, 1);
    # (a point it cannot clear by then, nearer than the flight model's tracking closes a height error, 1/K (tr): one
    # of those, closed over that, the command meanwhile not eased, which would only take it lower. Climbing at once
    # for it, as much as the power makes, the plan took a tenth of a foot under the clearance over the sea for a
    # climb, and porpoised; further, that climb is the plan's to make, and it does)
    var v = -TF_DESC;
    foreach (var sw; tf_past) {
        foreach (var q; sw) {
            var t = q[0] - now; var dh = q[1] + clr - hw;
            if (t <= 0 or tf_reach(v, t, v0, a, a / 2) >= dh) continue;
            if (t < tr and tf_reach(tf_climb, t, v0, a, a / 2) < dh) {
                v = math.max(v, v0);
                t += tr;
                if (tf_reach(v, t, v0, a, a / 2) >= dh) continue;
            }
            v = tf_need(dh, t, v0, a, a / 2);
        }
    }
    setprop(JSB ~ "ap/tf-hdot-fps", math.max(-TF_DESC, math.min(tf_climb, v)));
    setprop(JSB ~ "ap/tf-valid", ok);
}

# TF on (above 40 kt over the ground; the clearance at least 100 ft, 200 if the knob is lower) and off (the altitude
# held then, with ALT HOLD on)
var tf_key = func {
    if (val(AFCS ~ "tf")) {
        setprop(AFCS ~ "tf", 0);
        setprop(ST ~ "target-altitude-ft", math.round(val("/instrumentation/altimeter/indicated-altitude-ft") / 10) * 10);
        return;
    }
    if (val("/velocities/groundspeed-kt") < 40 or !val(JSB ~ "scas/airborne")) return refused("TF");
    if (val(ST ~ "target-agl-ft") < 100) setprop(ST ~ "target-agl-ft", 200);
    setprop(AFCS ~ "appr", 0);
    setprop(AFCS ~ "tf", 1);
}

var tf_ride = func(i) {
    i = math.max(1, math.min(3, i));
    setprop(JSB ~ "ap/tf-ride-g", TF_RIDES[i - 1][1]);
}

var tf_annunciate = func(s) {
    if (!val(AFCS ~ "tf")) return;
    if (val(JSB ~ "ap/tf-flyup") and val(JSB ~ "ap/tf-valid")) return append(s, "FLY UP");
    if (!val(JSB ~ "ap/tf-valid")) return append(s, "TF FAIL");
    var g = val(JSB ~ "ap/tf-ride-g");
    var ride = g < 0.15 ? "SOFT" : g < 0.25 ? "MED" : "HARD";
    append(s, sprintf("TF %d %s", math.max(100, val(ST ~ "target-agl-ft")), ride));
}

# AFCS/press: a key's name (displays.key's, as the panel's: APPR, VORILS, GA, HDGHOLD ...) pressed from outside the
# cockpit (a hardware panel through a protocol, a script over telnet)
setlistener(AFCS ~ "press", func(n) {
    var k = n.getValue();
    if (k == nil or k == "") return;
    n.setValue("");
    displays.key("AFCS", k);
}, 0, 0);

var timer = nil;
var turn_timer = nil;
var tf_timer = nil;
setlistener("/sim/signals/fdm-initialized", func {
    if (timer == nil) { timer = maketimer(2, update); timer.start(); }
    if (turn_timer == nil) { turn_timer = maketimer(TURNS_S, turns); turn_timer.start(); }
    if (tf_timer == nil) { tf_timer = maketimer(TF_S, tf); tf_timer.start(); }
    # the GPS flies over its waypoints; the turns before them are made here
    setprop("/instrumentation/gps/config/enable-fly-by", 0);
}, 1, 0);
