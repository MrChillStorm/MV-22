# MV-22B displays: the Canvas pages of the four multifunction displays (MFD 1-4 from the left), the control display
# unit (CDU), the backup engine and fuel display (SFD), the electronic standby flight instrument (ESFI), the automatic
# flight control panel's readouts and the glareshield's radio panels, on the screens of the panel's 3-D model
# (Models/Interior/Panel/MV22B, written by fdm/cockpit/panel.py), whose keys call key(unit, key).
#
# The V-22's display formats are not public in detail; these follow its published layout (four 6 in square
# multifunction displays with flight, navigation and system formats, the CDU's EICAS and status pages, a standby
# flight display) with the flight model's own quantities: nacelle angle and the conversion protection's limits,
# torque, rotor speed, the drive's torque limit, the five tanks. The flight format goes past the V-22's own, after
# human-factors studies of display design (see the Flight object): the limits it draws come from the flight model's
# data through Systems/mv22b-displays.xml (fdm/genv22.py).
#
# Each display's luminance (cd/m2, /instrumentation/mv22b/displays/<unit>/luminance) is what the screen's lit effect
# emits. Its photocell follows the light outside, here the sun's elevation (night with the sun 6 deg below the horizon,
# full day 10 deg above, logarithmically between); the OFF / NIGHT / DAY knob (<unit>/mode: 0, 1, 2) picks the range,
# NIGHT a fixed night vision goggle level, DAY the photocell's from dusk to full sun. The units without the knob (the
# standby instrument, the flight control panel's readouts, the radios) follow the photocell over both ranges. The BRT
# rocker or knob scales each (<unit>/brightness, 0.1-1); the levels are in /instrumentation/mv22b/displays/levels. The
# displays need power: the battery or an engine running.

var D2R = math.pi / 180;
var FONT = "LiberationFonts/LiberationSansNarrow-Bold.ttf";
var MONO = "LiberationFonts/LiberationMono-Bold.ttf";
var WHITE = [0.95, 0.95, 0.95];
var GREEN = [0.25, 0.95, 0.35];
var CYAN = [0.30, 0.90, 1.00];
var MAGENTA = [1.00, 0.35, 1.00];
var AMBER = [1.00, 0.72, 0.10];
var RED = [1.00, 0.20, 0.20];
var GREY = [0.55, 0.55, 0.55];
var SKY = [0.16, 0.40, 0.78];
var GROUND = [0.44, 0.28, 0.13];
var LEVELS = {night: 6, dusk: 30, day: 8000};      # cd/m2 at full brightness: NIGHT, DAY at dusk and in full sun

# a property's value, d when it has none (Nasal's "or" would take a zero for none too); the nodes are kept, so that a
# path is looked up once, not on every update
var NODES = {};
var num = func(p, d = 0) {
    var n = NODES[p];
    if (n == nil) n = NODES[p] = props.globals.getNode(p, 1);
    var v = n.getValue();
    return v == nil ? d : v;
}

# Canvas writes only when the value changes. Each write is a property set (a lookup, a lock and its listeners, a text's
# layout), and most of what a format draws holds from one update to the next; the elements a format keeps to update
# get their own setters that skip the value they already have (cache_writes walks a format's members once).
var cached = func(e) {
    if (e["_w"] != nil) return;
    e._w = {};
    if (isa(e, canvas.Text)) {
        var st = e.setText;
        e.setText = func(s) { if (me._w["t"] != s) { me._w["t"] = s; call(st, [s], me); } return me; };
    }
    var tr = e.setTranslation;
    e.setTranslation = func {
        var a = size(arg) == 1 ? arg[0] : arg;
        var w = me._w;
        if (w["x"] != a[0] or w["y"] != a[1]) { w["x"] = a[0]; w["y"] = a[1]; call(tr, [a[0], a[1]], me); }
        return me;
    };
    var ro = e.setRotation;
    e.setRotation = func(r) { if (me._w["r"] != r) { me._w["r"] = r; call(ro, [r], me); } return me; };
    var vi = e.setVisible;
    e.setVisible = func(v) { v = v ? 1 : 0; if (me._w["v"] != v) { me._w["v"] = v; call(vi, [v], me); } return me; };
    if (isa(e, canvas.Text) or isa(e, canvas.Path)) {
        var co = e.setColor; var cf = e.setColorFill;
        e.setColor = func { colour(me, "c", co, arg) };
        e.setColorFill = func { colour(me, "f", cf, arg) };
    }
}

# a colour given as one vector (the same one: the constants above) is cached, as separate components it is not
var colour = func(e, k, f, a) {
    if (size(a) == 1 and typeof(a[0]) == "vector") {
        if (e._w[k] == a[0]) return e;
        e._w[k] = a[0];
    } else e._w[k] = nil;
    call(f, a, e);
    return e;
}

# a path drawn again only when what it shows has changed (key: its inputs as shown, a number or a string)
var changed = func(e, key) {
    if (e["_k"] == key) return 0;
    e._k = key;
    return 1;
}

# a path of a fixed shape (n lines of k points) whose coordinates are rewritten in place: no nodes made and removed
var fixed_path = func(p, n, k) {
    var cmds = []; var coords = [];
    for (var l = 0; l < n; l += 1)
        for (var i = 0; i < k; i += 1) { append(cmds, i ? p.VG_LINE_TO_ABS : p.VG_MOVE_TO_ABS); append(coords, 0, 0); }
    p.setData(cmds, coords);
    return p._node.getChildren("coord");
}

var cache_writes = func(x, depth = 0) {
    if (depth > 5) return;
    if (typeof(x) == "vector") { foreach (var y; x) cache_writes(y, depth + 1); return; }
    if (typeof(x) != "hash") return;
    if (isa(x, canvas.Element)) return cached(x);
    if (isa(x, props.Node) or isa(x, canvas.Canvas)) return;
    foreach (var k; keys(x)) if (k != "parents") cache_writes(x[k], depth + 1);
}

var JSB = "/fdm/jsbsim/";

# ---------------------------------------------------------------- drawing helpers
var txt = func(g, s, x, y, size, color = nil, align = "center-center", font = nil) {
    return g.createChild("text").setFont(font == nil ? FONT : font).setFontSize(size, 1).setColor(color == nil ? WHITE : color)
            .setAlignment(align).setTranslation(x, y).setText(s);
}

var line = func(g, pts, color = nil, w = 2) {
    var p = g.createChild("path").moveTo(pts[0][0], pts[0][1]);
    for (var i = 1; i < size(pts); i += 1) p.lineTo(pts[i][0], pts[i][1]);
    return p.setColor(color == nil ? WHITE : color).setStrokeLineWidth(w);
}

var box = func(g, x, y, w, h, color = nil, fill = nil, lw = 2) {
    var p = g.createChild("path").rect(x, y, w, h).setColor(color == nil ? WHITE : color).setStrokeLineWidth(lw);
    if (fill != nil) p.setColorFill(fill);
    return p;
}

var clip = func(e, x0, y0, x1, y1) {
    e.set("clip", sprintf("rect(%d, %d, %d, %d)", y0, x1, y1, x0));
    return e;
}

# a moving tape: ticks every `step` units, labels every `label`, px per unit; returns the group to translate
var tape = func(g, x, w, lo, hi, step, label, px, cy, side, fmt) {
    var t = g.createChild("group");
    for (var v = lo; v <= hi; v += step) {
        var y = cy - v * px;
        var big = math.mod(v, label) == 0;
        var x0 = side < 0 ? x + w : x;
        line(t, [[x0, y], [x0 - side * (big ? 12 : 7), y]], WHITE, 2);
        if (big) txt(t, sprintf(fmt, v), side < 0 ? x + w - 16 : x + 16, y, 20, WHITE, side < 0 ? "right-center" : "left-center");
    }
    return t;
}

# the key labels beside a bezel's keys: top T1..T5, left L1..L7, right R1..R7 (on a 512 screen: the keys are 26 mm
# apart along the top, 21.5 mm down the sides, 3360 px per m: fdm/cockpit/panel.py), in a group of their own; backed:
# on a dark ground, to show over a format's own drawing
var labels = func(g, top, left, right, backed = 0) {
    var l = g.createChild("group");
    var put = func(s, x, y, fs, align) {
        if (backed) {
            var w = size(s) * fs * 0.55 + 8;
            var x0 = align == "left-center" ? x - 4 : align == "right-center" ? x - w + 4 : x - w / 2;
            box(l, x0, y - fs / 2 - 4, w, fs + 8, [0, 0, 0, 0.85], [0, 0, 0, 0.85], 1);
        }
        txt(l, s, x, y, fs, CYAN, align);
    }
    for (var i = 0; i < size(top); i += 1)
        if (top[i] != "") put(top[i], 256 + (i - 2) * 87.4, 14, 18, "center-center");
    for (var i = 0; i < size(left); i += 1)
        if (left[i] != "") put(left[i], 6, 39 + 72.2 * i, 17, "left-center");
    for (var i = 0; i < size(right); i += 1)
        if (right[i] != "") put(right[i], 506, 39 + 72.2 * i, 17, "right-center");
    return l;
}

# ---------------------------------------------------------------- the flight model's data for the flight format
# Systems/mv22b-displays.xml, written by fdm/genv22.py from the model's data: the conversion protection's boundaries
# and the power to hover (read into /instrumentation/mv22b/displays/data at the start)
var DATA = "/instrumentation/mv22b/displays/data/";
var SITE = "/instrumentation/mv22b/eosite/";      # ENG OUT's landing sites (eosite.nas)
var SITES_SHOWN = 16;
var R2D = 180 / math.pi;
var KT = 0.592484;                   # kt per ft/s

var column = func(path, tag) {
    var out = [];
    foreach (var c; props.globals.getNode(path).getChildren(tag)) append(out, c.getValue());
    return out;
}

var interp = func(x, xs, ys) {
    if (x <= xs[0]) return ys[0];
    for (var i = 1; i < size(xs); i += 1)
        if (x <= xs[i]) return ys[i - 1] + (ys[i] - ys[i - 1]) * (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
    return ys[-1];
}

var clamp = func(x, lo, hi) { math.max(lo, math.min(hi, x)) };
var mix = func(a, b, f) { [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f] };

# the conversion protection: its least and most nacelle angle at an equivalent airspeed, and the other way round, the
# least and most airspeed its boundaries allow the nacelles' angle (the indicated airspeed is taken for the
# equivalent: they differ by under 2 % below 10 000 ft and 200 kt)
var corridor = {
    load: func {
        me.lv = column(DATA ~ "cps/lower", "kt"); me.ln = column(DATA ~ "cps/lower", "nac");
        me.uv = column(DATA ~ "cps/upper", "kt"); me.un = column(DATA ~ "cps/upper", "nac");
    },
    nac_min: func(v) { interp(v, me.lv, me.ln) },
    nac_max: func(v) { interp(v, me.uv, me.un) },
    # where a falling boundary (vs, ns) crosses the angle n: the first crossing (v_min) or the last (v_max)
    cross: func(n, vs, ns, last) {
        var out = nil;
        for (var i = 1; i < size(vs); i += 1)
            if (ns[i - 1] > ns[i] and n <= ns[i - 1] and n >= ns[i]) {
                var v = vs[i - 1] + (vs[i] - vs[i - 1]) * (ns[i - 1] - n) / (ns[i - 1] - ns[i]);
                if (!last) return v;
                out = v;
            }
        return out;
    },
    v_min: func(n) { n >= me.ln[0] ? 0 : n <= me.ln[-1] ? me.lv[-1] : me.cross(n, me.lv, me.ln, 0) },
    v_max: func(n) { n > me.un[0] ? 0 : n <= me.un[-1] ? me.uv[-1] : me.cross(n, me.uv, me.un, 1) },
    # in airplane mode the operating limit instead: at the low rotor speed its KCAS or Mach, whichever is less
    v_apln: func(ias) {
        var a = DATA ~ "apln/";
        if (num(JSB ~ "drive/nr-pct") > 94) return num(a ~ "vmo-100-kt");
        return math.min(num(a ~ "vmo-kt"), ias * num(a ~ "mmo") / math.max(0.05, num("/velocities/mach")));
    },
};

# a moving tape with its ticks inside its box (x0..x1): side -1 on its right edge (the airspeed), 1 on its left (the
# altitude); returns the group to translate by the value times px
var tape_in = func(g, x0, x1, lo, hi, step, label, px, side) {
    var t = g.createChild("group");
    var xe = side < 0 ? x1 : x0;
    for (var v = lo; v <= hi; v += step) {
        var y = CY - v * px; var big = math.mod(v, label) == 0;
        line(t, [[xe, y], [xe + side * (big ? 10 : 6), y]], WHITE, 1.5);
        if (big) txt(t, sprintf("%d", v), xe + side * 14, y, 15, WHITE, side < 0 ? "right-center" : "left-center");
    }
    return t;
}

var lowpass = func(y, x, dt, tau) { y + (x - y) * math.min(1, dt / tau) };

# ---------------------------------------------------------------- the flight format
# One format for every regime, after the human-factors studies the README lists: the flight path marker with its
# acceleration chevron at the centre; 10 s trend vectors on the tapes; the limits where the eye already is (the
# conversion corridor's airspeeds for the nacelles' angle on the speed tape, the vortex ring state's sink rate for
# the airspeed on the vertical speed scale, the margin to the first power limit in one bar with the power to hover in
# and out of ground effect); a ground grid in perspective; the corridor itself, where the aircraft is in it and where
# it is going. Slowing below 30 kt with the nacelles past 75 deg it turns to the hover: the flight path marker gives
# way to the drift seen from above as a target to fly to: the stop ball on a line from the centre, opposite where the
# ground velocity will be in 3 s (drifting aft it is ahead, the stick goes towards it, and it comes back to the centre
# as the drift is about to stop), the hover point, the radar height on a stretched tape; below 200 ft the tape (and its
# trend) goes and a green gauge under the readout drains to its foot as the height goes to nothing. Linear: the time
# the eye sees it take to empty is the time to the ground (a square-root scale showed twice it). Past 60 deg of bank or 25 deg
# of pitch the clutter goes and an arrow shows the way to roll level. With ENG OUT on (both engines out: the flight
# model's eng_out) it flies that: the guidance cue its pitch and bank, the speed bug its glide's speed, the corridor's
# target its nacelles, the lever's way its lever; the flare's height on the altitude tape, the tail's strike on the
# pitch ladder in the flare, and in place of the configuration the glide's reach along the track (to the flare, from
# the energy height at the model's own glide ratio, the wind's ground speed with it).
var AX0 = 96; var AY0 = 46; var AX1 = 416; var AY1 = 330;      # the attitude's window
var CX = 256; var CY = 178; var K = 6.0;                       # its centre (the aircraft's waterline), px per deg
var GROUND_BAND = [0.43, 0.27, 0.12];
var RAD_PX = 2.6;                                              # the radar tape: px per ft
var RAD_FT = 200;                                              # ft: below it the radar height's gauge in its place,
var GAUGE_PX = AY1 - CY;                                       # filling the tape under the readout at 200 ft
var RAD_BAR = [0.25, 0.95, 0.35, 0.4];                         # the radar height's bar: RAD ALT's green, the scale seen through
var GRID = [0.62, 0.45, 0.28];
var GRID_HALF = 6;                       # the ground grid: lines each side of the aircraft, each way
var GRID_S = [-1, -0.6, -0.35, -0.2, -0.1, 0, 0.1, 0.2, 0.35, 0.6, 1];     # its points along a line, x its length
var HOVER_IN = 30; var HOVER_OUT = 40;                         # kt ground speed into and out of the hover format
var RING = 62;                                                 # px: 10 kt, or 60 ft, from the hover display's centre

var Flight = {
    new: func(g) {
        var m = {parents: [Flight], g: g, t0: systime(), ias0: 0, dias: 0, dtas: 0, tas0: 0, af: 0, ar: 0, vf0: 0, vr0: 0,
                 hover: 0, mark: nil, n: 0};
        # ---- the attitude: sky, ground and its grid, the horizon, the pitch ladder
        var win = clip(g.createChild("group"), AX0, AY0, AX1, AY1);
        m.att = win.createChild("group").setCenter(CX, CY);
        m.pitch = m.att.createChild("group");
        box(m.pitch, CX - 700, CY - 1000, 1400, 1000, SKY, SKY, 1);
        box(m.pitch, CX - 700, CY, 1400, 1000, GROUND, GROUND, 1);
        m.grid = m.pitch.createChild("path").setColor(GRID).setStrokeLineWidth(1);
        m.gridc = fixed_path(m.grid, 2 * (2 * GRID_HALF + 1), size(GRID_S));
        line(m.pitch, [[CX - 700, CY], [CX + 700, CY]], WHITE, 2);
        m.ladder = m.pitch.createChild("group");
        for (var p = -80; p <= 80; p += 5) {
            if (p == 0) continue;
            var y = CY - p * K; var w = math.mod(p, 10) == 0 ? 34 : 16;
            line(m.ladder, [[CX - w, y], [CX + w, y]], WHITE, 2);
            if (math.mod(p, 10) == 0) {
                txt(m.ladder, sprintf("%d", math.abs(p)), CX - w - 14, y, 15);
                txt(m.ladder, sprintf("%d", math.abs(p)), CX + w + 14, y, 15);
            }
        }
        m.ladder_h = m.pitch.createChild("group");                    # the hover's: short, unlabelled
        foreach (var p; [-10, -5, 5, 10])
            line(m.ladder_h, [[CX - (math.mod(p, 10) == 0 ? 16 : 8), CY - p * K], [CX + (math.mod(p, 10) == 0 ? 16 : 8), CY - p * K]], GREY, 2);
        # the bank scale, its pointer
        foreach (var a; [-60, -45, -30, -20, -10, 10, 20, 30, 45, 60]) {
            var r1 = math.mod(a, 30) == 0 ? 129 : 124; var s = math.sin(a * D2R); var c = math.cos(a * D2R);
            line(g, [[CX + 118 * s, CY - 118 * c], [CX + r1 * s, CY - r1 * c]], WHITE, 2);
        }
        line(g, [[CX, CY - 117], [CX - 7, CY - 129], [CX + 7, CY - 129], [CX, CY - 117]], WHITE, 2).setColorFill(WHITE);
        m.roll = g.createChild("group").setCenter(CX, CY);
        line(m.roll, [[CX, CY - 116], [CX - 7, CY - 104], [CX + 7, CY - 104], [CX, CY - 116]], WHITE, 2).setColorFill(WHITE);
        # the aircraft's waterline, small: the flight path is the symbol to fly
        line(g, [[CX - 16, CY], [CX - 7, CY], [CX - 3, CY + 5], [CX, CY], [CX + 3, CY + 5], [CX + 7, CY], [CX + 16, CY]], AMBER, 3);
        # the flight path marker and its acceleration chevron (flight path acceleration in g, as degrees)
        m.fpv = g.createChild("group");
        m.fpv.createChild("path").circle(8, 0, 0).setColor(GREEN).setStrokeLineWidth(2.5);
        line(m.fpv, [[-24, 0], [-8, 0]], GREEN, 2.5); line(m.fpv, [[8, 0], [24, 0]], GREEN, 2.5); line(m.fpv, [[0, -8], [0, -16]], GREEN, 2.5);
        m.chev = line(m.fpv, [[-38, -6], [-30, 0], [-38, 6]], GREEN, 2.5);
        # the flight director's guidance cue: fly the waterline into it (its pitch and bank commands)
        m.cue = g.createChild("group");
        m.cue.createChild("path").circle(9, 0, 0).setColor(MAGENTA).setStrokeLineWidth(3);
        line(m.cue, [[-15, 0], [-9, 0]], MAGENTA, 3); line(m.cue, [[9, 0], [15, 0]], MAGENTA, 3);
        # ENG OUT's flare: the pitch at which the fuselage meets the ground on the wheels (the waterline on it, the tail
        # strikes), on the ladder
        var ts = num(DATA ~ "tail-strike-deg", 7.3);
        m.tail = m.pitch.createChild("group");
        line(m.tail, [[CX - 70, CY - ts * K], [CX - 22, CY - ts * K]], RED, 2.5);
        line(m.tail, [[CX + 22, CY - ts * K], [CX + 70, CY - ts * K]], RED, 2.5);
        txt(m.tail, "TAIL", CX + 92, CY - ts * K, 13, RED);
        m.tail.hide();
        # ---- the hover: the stop ball on its line (a target: opposite the ground velocity 3 s on), the hover point
        m.hov = g.createChild("group").setTranslation(CX, CY + 46);
        for (var a = 0; a < 360; a += 12) {
            var a0 = a * D2R; var a1 = (a + 6) * D2R;
            line(m.hov, [[RING * math.cos(a0), RING * math.sin(a0)], [RING * math.cos(a1), RING * math.sin(a1)]], WHITE, 1.5);
        }
        m.hov.createChild("path").circle(RING / 2, 0, 0).setColor(GREY).setStrokeLineWidth(1);
        line(m.hov, [[-7, 0], [7, 0]], AMBER, 2.5); line(m.hov, [[0, -9], [0, 6]], AMBER, 2.5);
        m.vel = m.hov.createChild("path").setColor(WHITE).setStrokeLineWidth(3);
        m.ball = m.hov.createChild("path").circle(6, 0, 0).setColor(GREEN).setStrokeLineWidth(2.5);
        m.hpt = m.hov.createChild("group");
        box(m.hpt, -9, -9, 18, 18, MAGENTA, nil, 2.5);
        m.hptf = box(m.hpt, -6, -6, 12, 12, MAGENTA, MAGENTA, 1);
        m.hptxt = txt(m.hpt, "", 14, 0, 13, MAGENTA, "left-center");
        txt(m.hov, "10 KT", RING + 4, -4, 12, WHITE, "left-center");
        # ---- the airspeed tape: the corridor's limits for the nacelles' angle red, the trend, the selected speed
        var sw = clip(g.createChild("group"), 44, AY0, 92, AY1);
        box(sw, 44, AY0, 48, AY1 - AY0, [0.07, 0.07, 0.08], [0.07, 0.07, 0.08], 1);
        m.spd = tape_in(sw, 44, 92, 0, 400, 10, 20, 2.2, -1);
        m.vlim = sw.createChild("path").setColor(RED).setColorFill(RED).setStrokeLineWidth(1);
        m.sbug = line(g, [[92, 0], [101, -7], [101, 7], [92, 0]], CYAN, 2);
        m.strend = g.createChild("path").setColor(GREEN).setStrokeLineWidth(2.5);
        box(g, 42, CY - 15, 51, 30, WHITE, [0, 0, 0], 2);
        m.spdtxt = txt(g, "0", 88, CY, 22, WHITE, "right-center", MONO);
        m.ssel = txt(g, "", 68, AY0 - 9, 15, CYAN);
        m.gs = txt(g, "", 68, AY1 + 11, 15);
        # ---- the altitude tape: baro, in the hover the radar height stretched; the ground rising on it
        var aw = clip(g.createChild("group"), 420, AY0, 468, AY1);
        box(aw, 420, AY0, 48, AY1 - AY0, [0.07, 0.07, 0.08], [0.07, 0.07, 0.08], 1);
        m.gnd = aw.createChild("path").setColor(AMBER).setColorFill(GROUND_BAND).setStrokeLineWidth(2);
        m.altb = tape_in(aw, 420, 468, -2000, 30000, 100, 200, 0.32, 1);
        m.altr = tape_in(aw, 420, 468, 0, 400, 10, 50, RAD_PX, 1);
        m.abug = line(g, [[420, 0], [411, -7], [411, 7], [420, 0]], CYAN, 2);
        m.fbug = g.createChild("group");                                  # ENG OUT's flare height
        line(m.fbug, [[420, 0], [411, -7], [411, 7], [420, 0]], MAGENTA, 2).setColorFill(MAGENTA);
        txt(m.fbug, "FLR", 408, 0, 11, MAGENTA, "right-center");
        m.fbug.hide();
        m.atrend = g.createChild("path").setColor(GREEN).setStrokeLineWidth(2.5);
        box(g, 419, CY - 15, 58, 30, WHITE, [0, 0, 0], 2);
        m.alttxt = txt(g, "0", 424, CY, 21, WHITE, "left-center", MONO);
        m.asel = txt(g, "", 444, AY0 - 9, 15, CYAN);
        m.altlow = txt(g, "", 444, AY1 + 11, 15, GREEN);
        # ---- the vertical speed: its scale square-rooted, SINK RATE's descent rate
        var vx = 488;
        line(g, [[vx, m.vsy(2000)], [vx, m.vsy(-2000)]], [0.3, 0.3, 0.32], 2);
        foreach (var f; [-2000, -1000, -500, 0, 500, 1000, 2000]) line(g, [[vx - 4, m.vsy(f)], [vx + 4, m.vsy(f)]], WHITE, 1.5);
        foreach (var f; [-2000, -1000, 1000, 2000]) txt(g, sprintf("%d", math.abs(f) / 1000), vx + 12, m.vsy(f), 12, GREY);
        m.vsbar = g.createChild("path").setStrokeLineWidth(6);                     # the vertical speed, from naught
        m.vrslim = g.createChild("path").setColor(RED).setStrokeLineWidth(2.5);    # the boundary's sink
        m.vrstxt = txt(g, "VRS", vx, m.vsy(-2000) + 13, 12, RED);
        m.vstxt = txt(g, "", vx, AY0 - 9, 13);
        # ---- the heading arc: a card turning under a fixed index, the ground track, the selected heading, the wind
        var hx = CX; var hy = 432; var hr = 66;
        m.hx = hx; m.hy = hy;
        var cw = clip(g.createChild("group"), hx - hr - 12, hy - hr - 10, hx + hr + 12, hy - 18);
        m.card = cw.createChild("group").setCenter(hx, hy);
        for (var h = 0; h < 360; h += 5) {
            var a = (h - 90) * D2R; var r1 = hr - (math.mod(h, 10) == 0 ? 9 : 5);
            line(m.card, [[hx + hr * math.cos(a), hy + hr * math.sin(a)], [hx + r1 * math.cos(a), hy + r1 * math.sin(a)]], WHITE, 1.5);
            if (math.mod(h, 30) == 0) {
                var s = h == 0 ? "N" : h == 90 ? "E" : h == 180 ? "S" : h == 270 ? "W" : sprintf("%d", h / 10);
                txt(m.card, s, hx + (hr - 20) * math.cos(a), hy + (hr - 20) * math.sin(a), 13).setRotation(h * D2R);
            }
        }
        m.trk = cw.createChild("group").setCenter(hx, hy);
        line(m.trk, [[hx, hy - hr - 6], [hx + 5, hy - hr], [hx, hy - hr + 6], [hx - 5, hy - hr], [hx, hy - hr - 6]], GREEN, 2);
        m.hbug = cw.createChild("group").setCenter(hx, hy);
        box(m.hbug, hx - 5, hy - hr - 8, 10, 8, CYAN, nil, 2);
        box(g, hx - 21, hy - hr - 24, 42, 20, WHITE, [0, 0, 0], 2);
        m.hdgtxt = txt(g, "000", hx, hy - hr - 14, 16, WHITE, "center-center", MONO);
        m.wind = g.createChild("group").setTranslation(hx + 52, hy - hr - 14);
        line(m.wind, [[0, -8], [0, 8]], WHITE, 2); line(m.wind, [[-4, -3], [0, -8], [4, -3]], WHITE, 2);
        m.windtxt = txt(g, "", hx + 64, hy - hr - 14, 13, WHITE, "left-center");
        # ---- the conversion corridor: airspeed across, nacelle angle up; the aircraft in it, its 10 s trend; the
        # angles allowed at the selected speed
        var x0 = 22; var y0 = 372; var w = 118; var h = 106;
        m.cvx = func(v) { x0 + 6 + v / 220 * (w - 12) };
        m.cny = func(n) { y0 + h - 8 - n / 100 * (h - 18) };
        box(g, x0, y0, w, h, [0.07, 0.07, 0.08], [0.07, 0.07, 0.08], 1);
        var allowed = g.createChild("path").setColorFill([0.20, 0.22, 0.25]).setColor([0.20, 0.22, 0.25]);
        allowed.moveTo(m.cvx(corridor.lv[0]), m.cny(corridor.ln[0]));
        for (var i = 1; i < size(corridor.lv); i += 1) allowed.lineTo(m.cvx(corridor.lv[i]), m.cny(corridor.ln[i]));
        for (var i = size(corridor.uv) - 1; i >= 0; i -= 1) allowed.lineTo(m.cvx(corridor.uv[i]), m.cny(corridor.un[i]));
        allowed.close();
        foreach (var b; [[corridor.lv, corridor.ln], [corridor.uv, corridor.un]]) {
            var pts = [];
            for (var i = 0; i < size(b[0]); i += 1) append(pts, [m.cvx(b[0][i]), m.cny(b[1][i])]);
            line(g, pts, RED, 2);
        }
        foreach (var n; [0, 30, 60, 90]) txt(g, sprintf("%d", n), x0 + 3, m.cny(n), 10, GREY, "left-center");
        foreach (var v; [100, 200]) txt(g, sprintf("%d", v), m.cvx(v), y0 + h - 3, 10, GREY);
        m.cband = g.createChild("path").setColor(CYAN).setStrokeLineWidth(3);
        m.ctrend = g.createChild("path").setColor(GREEN).setStrokeLineWidth(2);
        m.cdot = g.createChild("path").circle(5, 0, 0).setColor(WHITE).setColorFill(WHITE);
        m.ncue = g.createChild("path").circle(6, 0, 0).setColor(MAGENTA).setStrokeLineWidth(2.5);   # AUTO NAC's target
        m.nactxt = txt(g, "NAC 90", x0 + w / 2, y0 - 11, 17);
        # ---- power: the margin to the first limit (the transmission's torque or the engines' power), the power to
        # hover in and out of ground effect, the rotor's speed
        var px = 428; var py = 372; var pw = 26; var ph = 106;
        m.py = func(pct) { py + ph - clamp(pct, 0, 110) / 110 * ph };
        box(g, px, py, pw, ph, GREY, nil, 1.5);
        m.pbar = g.createChild("path").setStrokeLineWidth(1);
        line(g, [[px - 4, m.py(100)], [px + pw + 4, m.py(100)]], RED, 2.5);
        line(g, [[px - 4, m.py(90)], [px + pw + 4, m.py(90)]], AMBER, 1.5);
        m.oge = g.createChild("group"); line(m.oge, [[px - 8, 0], [px, 0]], CYAN, 2.5); txt(m.oge, "OGE", px - 10, -5, 11, CYAN, "right-center");
        m.ige = g.createChild("group"); line(m.ige, [[px - 8, 0], [px, 0]], CYAN, 2.5); txt(m.ige, "IGE", px - 10, 6, 11, CYAN, "right-center");
        m.pwrtxt = txt(g, "", px + pw / 2, py - 11, 16);
        m.lcue = txt(g, "", px - 22, py + ph / 2, 15, MAGENTA, "center-center");    # the lever's way, flown uncoupled
        txt(g, "NR", px + pw + 26, py + ph / 2 - 8, 11, GREY);
        m.nrtxt = txt(g, "", px + pw + 26, py + ph / 2 + 7, 17, WHITE, "center-center", MONO);
        # ---- the modes, the configuration, the recovery arrow
        m.modes = txt(g, "", CX, 22, 16, GREEN);
        m.alert = txt(g, "", CX, 40, 16, AMBER);
        # a time-critical warning (Nasal/alerts.nas: SINK RATE), red under the aircraft in the attitude's window
        m.tc = g.createChild("group");
        box(m.tc, CX - 105, 238, 210, 62, RED, [0, 0, 0, 0.85], 2);
        m.tctxt = txt(m.tc, "", CX, 257, 30, RED);
        m.tccue = txt(m.tc, "", CX, 286, 18, WHITE);           # its recovery, where the model shows one
        m.tc.hide();
        m.config = txt(g, "", CX, 497, 14);
        m.recover = g.createChild("path").setColor(RED).setColorFill(RED);
        return m;
    },
    vsy: func(fpm) { CY - (fpm < 0 ? -1 : 1) * math.sqrt(math.min(2000, math.abs(fpm)) / 2000) * 128 },
    key: func(k) {
        if (k == "R7") me.mark = [num("/position/latitude-deg"), num("/position/longitude-deg")];
    },
    update: func {
        var t = systime(); var dt = clamp(t - me.t0, 0.01, 0.5); me.t0 = t;
        var pitch = num("/orientation/pitch-deg"); var roll = num("/orientation/roll-deg");
        var hdg = num("/orientation/heading-magnetic-deg", num("/orientation/heading-deg"));
        var thdg = num("/orientation/heading-deg");
        var ias = math.max(0, num("/instrumentation/airspeed-indicator/indicated-speed-kt"));
        var tas = num(JSB ~ "velocities/vtrue-kts");
        var alt = num("/instrumentation/altimeter/indicated-altitude-ft");
        var agl = num("/position/gear-agl-m") / 0.3048;                        # the radar height: 0 on the wheels
        var vs = num("/velocities/vertical-speed-fps") * 60;
        var vn = num("/velocities/speed-north-fps") * KT; var ve = num("/velocities/speed-east-fps") * KT;
        var gs = math.sqrt(vn * vn + ve * ve);
        var vf = vn * math.cos(thdg * D2R) + ve * math.sin(thdg * D2R);           # ground velocity, kt: forward, right
        var vr = -vn * math.sin(thdg * D2R) + ve * math.cos(thdg * D2R);
        var nac = num(JSB ~ "ctl/nacelle-deg", 90);
        # rates, filtered: airspeed (kt/s) for its trend, true airspeed for the flight path's acceleration, the
        # ground velocity's for the stop ball
        me.dias = lowpass(me.dias, (ias - me.ias0) / dt, dt, 0.8); me.ias0 = ias;
        me.dtas = lowpass(me.dtas, (tas - me.tas0) / dt, dt, 0.5); me.tas0 = tas;
        me.af = lowpass(me.af, (vf - me.vf0) / dt, dt, 0.5); me.vf0 = vf;
        me.ar = lowpass(me.ar, (vr - me.vr0) / dt, dt, 0.5); me.vr0 = vr;
        var hold = num(JSB ~ "hh/on"); var hstate = num(JSB ~ "hh/state");
        if (me.hover and !hold and (gs > HOVER_OUT or nac < 70)) { me.hover = 0; me.mark = nil; }
        elsif (!me.hover and ((gs < HOVER_IN and nac > 75) or hold)) me.hover = 1;
        var unusual = math.abs(roll) > 60 or pitch > 25 or pitch < -25;
        # ---- the attitude
        me.att.setRotation(-roll * D2R);
        me.pitch.setTranslation(0, pitch * K);
        me.roll.setRotation(-roll * D2R);
        me.ladder.setVisible(!me.hover); me.ladder_h.setVisible(me.hover);
        me.n += 1;
        if (math.mod(me.n, 2) == 0) me.draw_grid(num("/position/altitude-agl-ft"), thdg, unusual);
        # the flight path marker: the ground-relative velocity's angles off the aircraft's axes
        var u = num(JSB ~ "velocities/u-fps"); var v = num(JSB ~ "velocities/v-fps"); var w = num(JSB ~ "velocities/w-fps");
        var show_fpv = !me.hover and !unusual and gs > 25 and u > 10;
        me.fpv.setVisible(show_fpv);
        if (show_fpv) {
            var fx = clamp(math.atan2(v, u) * R2D * K, AX0 - CX + 30, AX1 - CX - 30);
            var fy = clamp(math.atan2(w, u) * R2D * K, AY0 - CY + 20, AY1 - CY - 20);
            me.fpv.setTranslation(CX + fx, CY + fy);
            me.chev.setTranslation(0, -clamp(me.dtas / KT / 32.17 * R2D * K, -60, 60));
        }
        # the hover: the line and the stop ball opposite the velocity, now and 3 s ahead (a target), the hover point
        me.hov.setVisible(me.hover and !unusual);
        if (me.hover) {
            var sc = RING / 10;
            # one vector for both, its length held within 1.4 rings (14 kt) and its direction kept
            var bx = -(vr + me.ar * 3) * sc; var by = (vf + me.af * 3) * sc;
            var len = math.sqrt(bx * bx + by * by); var f = len > RING * 1.4 ? RING * 1.4 / len : 1;
            bx = math.round(bx * f); by = math.round(by * f);
            if (changed(me.vel, bx * 1000 + by)) me.vel.reset().moveTo(0, 0).lineTo(bx, by);
            me.ball.setTranslation(bx, by);
            # the hover hold's point (filled once it holds it), or where the aircraft last stopped
            if (hold) me.mark = [num(JSB ~ "hh/lat-t-rad") * R2D, num(JSB ~ "hh/lon-t-rad") * R2D];
            elsif (me.mark == nil and gs < 1.5 and agl > 3) me.mark = [num("/position/latitude-deg"), num("/position/longitude-deg")];
            me.hpt.setVisible(me.mark != nil);
            me.hptf.setVisible(hold and hstate == 2);
            if (me.mark != nil) {
                var dn = (me.mark[0] - num("/position/latitude-deg")) * 364567;
                var de = (me.mark[1] - num("/position/longitude-deg")) * 364567 * math.cos(me.mark[0] * D2R);
                var pf = dn * math.cos(thdg * D2R) + de * math.sin(thdg * D2R);
                var pr = -dn * math.sin(thdg * D2R) + de * math.cos(thdg * D2R);
                var d = math.sqrt(pf * pf + pr * pr); var k = d > 60 ? 60 / d : 1;
                if (d > 600 and !hold) me.mark = nil;         # left behind: the next stop sets another
                me.hpt.setTranslation(pr * k * RING / 60, -pf * k * RING / 60);
                me.hptxt.setText(sprintf("%d FT", math.round(d)));
            }
        }
        # the recovery arrow: roll toward level, the shorter way
        var s = roll > 0 ? -1 : 1;                    # roll left when banked right
        if (changed(me.recover, unusual ? s : 0)) {
            me.recover.reset();
            if (unusual) me.recover.moveTo(CX - s * 40, 92).lineTo(CX + s * 20, 92).lineTo(CX + s * 20, 78).lineTo(CX + s * 50, 102)
                      .lineTo(CX + s * 20, 126).lineTo(CX + s * 20, 112).lineTo(CX - s * 40, 112).close();
        }
        # ---- the airspeed
        me.spd.setTranslation(0, ias * 2.2);
        me.spdtxt.setText(sprintf("%d", ias));
        me.gs.setText(sprintf("GS %d", math.round(gs)));
        var sy = func(kt) { CY - (kt - ias) * 2.2 };
        var vmin = corridor.v_min(nac); var vmax = nac > 0.5 ? corridor.v_max(nac) : corridor.v_apln(ias);
        var bands = [];
        foreach (var r; [[0, vmin], [vmax, 450]])
            if (r[1] > r[0] + 0.5 and sy(r[0]) > AY0 and sy(r[1]) < AY1)
                append(bands, math.round(clamp(sy(r[1]), AY0, AY1)), math.round(clamp(sy(r[0]), AY0, AY1)));
        if (changed(me.vlim, size(bands) ? sprintf("%d %d %d %d", bands[0], bands[1], size(bands) > 2 ? bands[2] : 0, size(bands) > 2 ? bands[3] : 0) : "")) {
            me.vlim.reset();
            for (var b = 0; b < size(bands); b += 2) me.vlim.rect(84, bands[b], 8, bands[b + 1] - bands[b]);
        }
        # (ENG OUT's glide speed in its place, diving and gliding: magenta, the flight director's)
        var eo = num(AFCS ~ "eng-out"); var eph = eo ? num(JSB ~ "ap/eo-phase") : 0;
        var eglide = eo and eph < 2.5;
        var ssel = eglide ? num(JSB ~ "ap/eo-vg-kt") : eo ? 0 : num("/autopilot/settings/target-speed-kt");
        me.sbug.setTranslation(0, clamp(sy(ssel), AY0 + 8, AY1 - 8)).setColor(eglide ? MAGENTA : CYAN);
        me.ssel.setText((me.hover and !eglide) or ssel <= 0 ? "" : sprintf("%d", ssel)).setColor(eglide ? MAGENTA : CYAN);
        me.sbug.setVisible((!me.hover or eglide) and ssel > 0);
        var tr = clamp(me.dias * 10, -60, 60);
        if (math.abs(tr) <= 1) tr = 0;
        if (changed(me.strend, math.round(tr * 2.2))) {
            me.strend.reset();
            if (tr != 0) me.strend.moveTo(95, CY).lineTo(95, sy(ias + tr)).moveTo(91, sy(ias + tr) + (tr > 0 ? 6 : -6))
                             .lineTo(95, sy(ias + tr)).lineTo(99, sy(ias + tr) + (tr > 0 ? 6 : -6));
        }
        # ---- the altitude: baro, or the radar height in the hover near the ground
        var rad = me.hover and agl < 300;
        var hv = rad ? agl : alt; var px = rad ? RAD_PX : 0.32;
        var ay = func(ft) { CY - (ft - hv) * px };
        var gauge = rad and agl < RAD_FT;                 # below 200 ft: the readout and the gauge alone
        me.altb.setVisible(!rad); me.altr.setVisible(rad and !gauge);
        if (rad) me.altr.setTranslation(0, agl * RAD_PX); else me.altb.setTranslation(0, alt * 0.32);
        me.alttxt.setText(sprintf("%d", rad ? math.round(agl) : math.round(alt / 10) * 10));
        me.alttxt.setColor(rad ? GREEN : WHITE);
        var g0 = math.round(ay(rad ? 0 : alt - agl));
        # the ground on the baro tape (brown, below its line); below 200 ft, in place of the radar tape, the height as
        # a gauge under the readout: full at 200 ft, draining to its foot
        var gh = gauge ? math.round(GAUGE_PX * math.max(0, agl) / RAD_FT) : -1;
        if (changed(me.gnd, rad ? "r" ~ gh : g0 < AY1 ? math.max(AY0, g0) : -1)) {
            me.gnd.reset();
            if (rad) {
                me.gnd.setColor(RAD_BAR).setColorFill(RAD_BAR);
                if (gh > 0) me.gnd.rect(420, AY1 - gh, 48, gh);
            } else {
                me.gnd.setColor(AMBER).setColorFill(GROUND_BAND);
                if (g0 < AY1) me.gnd.rect(420, math.max(AY0, g0), 48, AY1 - math.max(AY0, g0));
            }
        }
        var asel = num("/autopilot/settings/target-altitude-ft");
        me.abug.setVisible(!rad and asel > 0 and !eo); me.abug.setTranslation(0, clamp(ay(asel), AY0 + 8, AY1 - 8));
        var fy = eglide ? ay((rad ? 0 : alt - agl) + num(JSB ~ "ap/eo-gate-ft")) : -1;
        me.fbug.setVisible(fy > AY0 and fy < AY1);
        if (fy > AY0 and fy < AY1) me.fbug.setTranslation(0, fy);
        me.asel.setText(rad ? "RAD ALT" : asel > 0 ? sprintf("%d", asel) : ""); me.asel.setColor(rad ? GREEN : CYAN);
        me.altlow.setText(rad ? sprintf("MSL %d", math.round(alt / 10) * 10) : agl < 5000 ? sprintf("R %d", math.round(agl)) : "");
        me.altlow.setColor(rad ? GREY : GREEN);
        var th = clamp(vs / 6, -400 / px, 400 / px);
        var ty = !gauge and math.abs(th * px) > 2 ? math.round(ay(hv + th)) : 0;
        if (changed(me.atrend, ty)) {
            me.atrend.reset();
            if (ty) me.atrend.moveTo(416, CY).lineTo(416, ty);
        }
        # ---- the vertical speed as a bar from naught, its colour the sink's closeness to the vortex ring state's
        # boundary at this airspeed (marked): green to half its sink, amber by 0.8 of it, red at it
        # SINK RATE's own descent rate at this edgewise speed and weight (fdm/alerts.py: half the vortex ring state's
        # onset in the more critical rotor's hover induced velocity), the bar red at it
        var lim = num(JSB ~ "alert/vrs/sink-fpm", 1e5);
        if (lim > 9e4) lim = nil;
        var r = lim == nil ? 0 : -vs / lim;
        var vy = math.round(me.vsy(vs));
        if (changed(me.vsbar, sprintf("%d %d", vy, math.round(clamp(r, 0.5, 1) * 40)))) {
            me.vsbar.reset().setColor(r <= 0.5 ? GREEN : r < 0.8 ? mix(GREEN, AMBER, (r - 0.5) / 0.3) : r < 1 ? mix(AMBER, RED, (r - 0.8) / 0.2) : RED);
            if (math.abs(vy - CY) >= 1) me.vsbar.moveTo(488, CY).lineTo(488, vy);
        }
        if (changed(me.vrslim, lim == nil ? -1 : math.round(me.vsy(-lim)))) {
            me.vrslim.reset();
            if (lim != nil) me.vrslim.moveTo(481, me.vsy(-lim)).lineTo(495, me.vsy(-lim));
        }
        me.vrstxt.setVisible(lim != nil);
        me.vstxt.setText(sprintf("%+d", math.round(vs / 10) * 10));
        me.vstxt.setColor(r >= 1 ? RED : r >= 0.8 ? AMBER : WHITE);
        # ---- heading, track, the selected heading, the wind
        me.card.setRotation(-hdg * D2R);
        me.hdgtxt.setText(sprintf("%03d", math.mod(math.round(hdg) + 359, 360) + 1));
        var trk = gs > 3 ? math.atan2(ve, vn) * R2D - thdg : 0;
        me.trk.setRotation(trk * D2R); me.trk.setVisible(gs > 3);
        me.hbug.setRotation((num("/autopilot/settings/heading-bug-deg") - hdg) * D2R);
        var wkt = num("/environment/wind-speed-kt");
        me.wind.setRotation((num("/environment/wind-from-heading-deg") - thdg + 180) * D2R).setVisible(wkt > 2);
        me.windtxt.setText(wkt > 2 ? sprintf("%03d/%d", math.mod(math.round(num("/environment/wind-from-heading-deg")) + 359, 360) + 1, math.round(wkt)) : "");
        # ---- the conversion corridor
        var nrate = num(JSB ~ "ctl/nacelle-rate-rad_sec") * R2D;
        me.cdot.setTranslation(me.cvx(math.min(ias, 220)), me.cny(nac));
        var c0 = [me.cvx(math.min(ias, 220)), me.cny(nac)];
        var c1 = [me.cvx(clamp(ias + me.dias * 10, 0, 220)), me.cny(clamp(nac + nrate * 10, 0, 97.5))];
        if (changed(me.ctrend, sprintf("%d %d %d %d", c0[0], c0[1], c1[0], c1[1])))
            me.ctrend.reset().moveTo(c0[0], c0[1]).lineTo(c1[0], c1[1]);
        if (changed(me.cband, !me.hover and ssel > 0 ? ssel : -1)) {
            me.cband.reset();
            if (!me.hover and ssel > 0)
                me.cband.moveTo(me.cvx(ssel), me.cny(corridor.nac_min(ssel))).lineTo(me.cvx(ssel), me.cny(corridor.nac_max(ssel)));
        }
        me.nactxt.setText(sprintf("NAC %d", math.round(nac)));
        # ---- power, per engine: the busier engine's against its share of the transmission's torque limit or its own
        # power available, whichever is less; with the rotor stopped the transmission's at 100 % rotor speed, the
        # engines' not yet one. The power to hover is shared by the engines running
        var p = math.max(num(JSB ~ "eng[0]/power-hp"), num(JSB ~ "eng[1]/power-hp"));
        var nrun = math.max(1, (num(JSB ~ "eng[0]/running") > 0.5) + (num(JSB ~ "eng[1]/running") > 0.5));
        var xm = num(JSB ~ "eng/xmsn-limit-hp"); var av = num(JSB ~ "eng/power-available-hp");
        if (xm < 50) xm = num(DATA ~ "hover/power-hp") / 2;
        var lim_p = av > 50 ? math.min(xm, av) : xm;
        var pct = p / lim_p * 100;
        if (changed(me.pbar, math.round(me.py(pct)))) me.pbar.reset().rect(431, me.py(pct), 20, me.py(0) - me.py(pct) - 1);
        me.pbar.setColorFill(pct > 100 ? RED : pct > 90 ? AMBER : GREEN).setColor(pct > 100 ? RED : pct > 90 ? AMBER : GREEN);
        me.pwrtxt.setText(sprintf("%d%% %s", math.round(pct), av > 50 and av < xm ? "ENG" : "TQ"));
        me.pwrtxt.setColor(pct > 100 ? RED : pct > 90 ? AMBER : WHITE);
        var vtol = nac > 60;
        me.oge.setVisible(vtol); me.ige.setVisible(vtol);
        if (vtol) {
            # the model's own power to hover (fdm/alerts.py, its trims), out of ground effect and with the wheels 10 ft up
            me.oge.setTranslation(0, me.py(num(JSB ~ "alert/hover-oge-hp") / nrun / lim_p * 100));
            me.ige.setTranslation(0, me.py(num(JSB ~ "alert/hover-ige-hp") / nrun / lim_p * 100));
        }
        me.nrtxt.setText(sprintf("%d", num(JSB ~ "drive/nr-pct")));
        # ---- the modes and the configuration
        var am = afcs_modes();
        me.modes.setText(me.hover and !hold ? "HOVER" ~ (am == "" ? "" : "  " ~ am) : am);
        me.modes.setColor(num(JSB ~ "ap/coupled") or hold ? GREEN : WHITE);
        me.alert.setText(alerts.flight_line());
        var tc = alerts.critical_line();
        me.tc.setVisible(tc[0] != "");
        if (tc[0] != "") { me.tctxt.setText(tc[0]); me.tccue.setText(tc[1]); }
        # ---- the flight director: the guidance cue, AUTO NAC's target, the lever's way
        # (ENG OUT's in the air: its pitch and bank, its nacelles at the airspeed, its lever; in the hover's format too)
        var fd_v = num(JSB ~ "ap/vert"); var fd_l = num(JSB ~ "ap/lat");
        var efd = eo and eph < 4.5;
        me.cue.setVisible((fd_v or fd_l or efd) and !unusual and (!me.hover or efd));
        if (efd)
            me.cue.setTranslation(CX + clamp((num(JSB ~ "ap/eo-phi-cmd-deg") - roll) * 3, -90, 90),
                                  CY - clamp((num(JSB ~ "ap/eo-theta-cmd-deg") - pitch) * K, -110, 110));
        elsif (fd_v or fd_l) {
            var dth = fd_v ? num(JSB ~ "ap/theta-ref-rad") * R2D - pitch : 0;
            var dph = fd_l ? num(JSB ~ "ap/phi-ref-rad") * R2D - roll : 0;
            me.cue.setTranslation(CX + clamp(dph * 3, -90, 90), CY - clamp(dth * K, -110, 110));
        }
        me.tail.setVisible(eo and eph > 2.5);
        var spd_on = num(JSB ~ "ap/spd");
        me.ncue.setVisible(spd_on or efd);
        if (efd) me.ncue.setTranslation(me.cvx(math.min(ias, 220)), me.cny(num(JSB ~ "ap/eo-nac-cmd-deg")));
        elsif (spd_on) me.ncue.setTranslation(me.cvx(math.min(num(JSB ~ "ap/v-ref-kt"), 220)), me.cny(num(JSB ~ "ap/nacelle-sched-deg")));
        var dl = efd and !num(JSB ~ "ap/eo-fly") ? num(JSB ~ "ap/eo-lever-cmd") - num("/controls/engines/engine[0]/throttle")
               : spd_on and !num(JSB ~ "ap/lever-on") ? num(JSB ~ "ap/tcl-cmd") - num("/controls/engines/engine[0]/throttle") : 0;
        me.lcue.setText(dl > 0.02 ? "TCL UP" : dl < -0.02 ? "TCL DN" : "");
        var gear = num(JSB ~ "gear/gear-pos-norm", 1);
        var gtxt = gear > 0.99 ? "GEAR DN" : gear < 0.01 ? "GEAR UP" : "GEAR TRANS";
        if (efd) {
            # the reach along the track: the still air's at the present ground speed over the true airspeed
            var reach = num(JSB ~ "ap/eo-reach-nm") * (tas > 30 ? gs / tas : 1);
            # with a landing site (eosite.nas): its name, the way to it and the energy's margin, NM (+ high)
            var st = !eglide ? "FLARE" : nac >= 1 ? "CONVERTING" : sprintf("REACH %.1f NM", reach);
            if (eglide and nac < 1 and num(SITE ~ "on"))
                st = sprintf("%s %.1f NM %+.1f", getprop(SITE ~ "name"), num(SITE ~ "path-nm"), num(SITE ~ "margin-nm"));
            me.config.setText(sprintf("%s   %s", st, gtxt));
            me.config.setColor(AMBER);
        } else {
            me.config.setText(sprintf("%s   FLAP %d   %s%s", gtxt, num(JSB ~ "ctl/flaperon-deg"),
                                      num(JSB ~ "scas/attitude-command") > 0.5 ? "ATT CMD" : "RATE CMD",
                                      num("/controls/rotor/brake") > 0.05 ? "   ROTOR BRAKE" : ""));
            me.config.setColor(WHITE);
        }
    },
    # the ground grid: lines north-south and east-west on the ground, spaced by the height, seen from the aircraft:
    # depression below the horizon and azimuth off the heading, in degrees (the pitch ladder's scale)
    draw_grid: func(agl, thdg, unusual) {
        me.grid.setVisible(!unusual and agl >= 5);
        if (unusual or agl < 5) return;
        var h = math.max(agl, 20);
        var D = h < 400 ? 100 : h < 2000 ? 500 : h < 8000 ? 2000 : 8000;
        var lat = num("/position/latitude-deg"); var lon = num("/position/longitude-deg");
        var n0 = math.mod(lat * 364567, D); var e0 = math.mod(lon * 364567 * math.cos(lat * D2R), D);
        var c = math.cos(thdg * D2R); var sn = math.sin(thdg * D2R);
        var near = h * 0.7; var far = math.min(h * 40, D * 14);
        var gc = me.gridc; var n = 0;
        # points behind the near limit drawn at it, below the window's clip
        for (var dir = 0; dir < 2; dir += 1)
            for (var j = -GRID_HALF; j <= GRID_HALF; j += 1) {
                var off = j * D - (dir ? e0 : n0);
                foreach (var a; GRID_S) {
                    var along = a * far;
                    var dn = dir ? along : off; var de = dir ? off : along;
                    var f = math.max(near, dn * c + de * sn); var r = -dn * sn + de * c;
                    gc[n].setDoubleValue(CX + math.atan2(r, f) * R2D * K);
                    gc[n + 1].setDoubleValue(CY + math.atan2(h, f) * R2D * K);
                    n += 2;
                }
            }
    },
};

# ---------------------------------------------------------------- the navigation format
# A moving map, track up (north up on a key), after the studies the README lists: the terrain coloured by its clearance
# below the height the aircraft will have when it gets there (60 s ahead at most; below 90 kt the helicopter's tighter
# bands); the route, its waypoints and the landing zone (the route's last point) with the approach into the wind; the
# conversion planner's points on the route, where to begin slowing down (DECEL) and converting (CONV) to arrive in a
# hover at the landing zone at a comfortable deceleration; the path 30 and 60 s ahead in the turn; traffic; the fuel's
# endurance and range; under it the profile along the track: the terrain, the aircraft's path, the selected altitude.
# Below 100 ft above the ground the terrain is measured from 100 ft up, so the flat ground around a hover or a landing
# stays quiet and only what rises above it is coloured; where the scenery has no elevation yet the map is dim magenta.
# Keys: R1 / R2 the range, R3 track or north up, R4 the terrain.
var NM_DEG = 60.0;                       # NM per degree of latitude
var MAP_X = 256; var MAP_Y = 330; var MAP_R = 290;   # the aircraft, the outer range arc's radius (px)
var MAP_CLIP = [16, 40, 496, 372];
var RANGES = [1, 2, 5, 10, 20, 40, 80];  # NM to the outer arc
var CELL = 10;                           # px: the terrain's sample spacing
var SAMPLES = 45;                        # elevations looked up per update (10 Hz: the map's sweep about 2 s)
var ROWS = 4;                            # rows of the terrain drawn per update, into the hidden one of two sets
var SIDE = 62;                           # deg: the map's sector each side of the track
var NEXT = [1, 2, 3, 0]; var SQ_X = [0, 1, 1, 0]; var SQ_Y = [0, 0, 1, 1];      # a square's corners, clockwise
var DECEL_KTS = 1.5;                     # kt/s: the planner's deceleration (about 0.08 g)
var CONV_KT = 150;                       # the planner's speed to begin the conversion at
var TERRAIN = [[0.60, 0.12, 0.12], [0.55, 0.40, 0.08], [0.10, 0.30, 0.14], [0.22, 0.06, 0.22]];   # red, amber, green, no data
var FLOOR_AGL = 100;                     # ft: the terrain's reference height near the ground
var TRAFFIC_MAX = 12; var WPT_MAX = 16;

var Nav = {
    new: func(g) {
        var m = {parents: [Nav], range_i: 3, north: 0, terrain: 1, cells: [], elev: [], prof: [], idx: 0, rate: 0,
                 trk0: nil, t0: systime(), eo_r: -1};
        # the terrain's sample points: a grid over the map, those in its sector or a little outside it (gi: the grid
        # to the samples, -1 where there is none)
        m.nx = int((MAP_CLIP[2] - MAP_CLIP[0]) / CELL) + 1; m.ny = int((MAP_CLIP[3] - MAP_CLIP[1]) / CELL) + 1;
        m.gi = [];
        var near = 1.5 * CELL;
        for (var j = 0; j < m.ny; j += 1)
            for (var i = 0; i < m.nx; i += 1) {
                var x = MAP_CLIP[0] + i * CELL; var y = MAP_CLIP[1] + j * CELL;
                var dx = x - MAP_X; var dy = y - MAP_Y; var r = math.sqrt(dx * dx + dy * dy);
                var sx = (dx < 0 ? -1 : 1) * math.sin(SIDE * D2R); var sy = -math.cos(SIDE * D2R);
                var inside = r <= MAP_R + near and (r < near or math.abs(math.atan2(dx, -dy)) * R2D <= SIDE
                                                     or math.abs(dx * sy - dy * sx) <= near);
                append(m.gi, inside ? size(m.cells) : -1);
                if (inside) { append(m.cells, [x, y, dx, dy]); append(m.elev, nil); }
            }
        setsize(m.prof, 41);
        var map = clip(g.createChild("group"), MAP_CLIP[0], MAP_CLIP[1], MAP_CLIP[2], MAP_CLIP[3]);
        # the terrain's colours painted over each other (green, amber, red, then no data), twice: one set shown while the
        # other is drawn a few rows an update
        m.sets = []; m.front = 0; m.drow = -1;
        for (var n = 0; n < 2; n += 1) {
            var set = setsize([], size(TERRAIN));
            var sg = map.createChild("group");
            foreach (var b; [2, 1, 0, 3])
                set[b] = sg.createChild("path").setColorFill(TERRAIN[b]).setColor(TERRAIN[b]).setStrokeLineWidth(1);
            if (n == 1) sg.hide();
            append(m.sets, {g: sg, p: set});
        }
        # the sector's edge: black outside it, under everything else
        var mask = map.createChild("path").setColorFill(0, 0, 0).setColor(0, 0, 0).setStrokeLineWidth(1);
        var L = MAP_CLIP[0]; var T = MAP_CLIP[1]; var R = MAP_CLIP[2]; var B = MAP_CLIP[3];
        var ys = MAP_Y - (MAP_X - L) / math.sin(SIDE * D2R) * math.cos(SIDE * D2R);
        mask.moveTo(L, ys).lineTo(MAP_X, MAP_Y).lineTo(R, ys).lineTo(R, B).lineTo(L, B).close();
        var a0 = math.atan2(MAP_X - L, math.sqrt(MAP_R * MAP_R - (MAP_X - L) * (MAP_X - L)));
        mask.moveTo(L, T);
        for (var i = 0; i <= 40; i += 1) {
            var a = -a0 + 2 * a0 * i / 40;
            mask.lineTo(MAP_X + MAP_R * math.sin(a), MAP_Y - MAP_R * math.cos(a));
        }
        mask.lineTo(R, T).close();
        # the range arcs, the compass card on the outer one
        m.arcs = map.createChild("path").setColor(GREY).setStrokeLineWidth(1.5);
        foreach (var f; [1, 0.5]) {
            for (var a = -SIDE; a <= SIDE; a += 2) {
                var px = MAP_X + MAP_R * f * math.sin(a * D2R); var py = MAP_Y - MAP_R * f * math.cos(a * D2R);
                if (a == -SIDE) m.arcs.moveTo(px, py); else m.arcs.lineTo(px, py);
            }
        }
        m.rlab = [txt(map, "", MAP_X - MAP_R * 0.5 * math.sin(62 * D2R) - 4, MAP_Y - MAP_R * 0.5 * math.cos(62 * D2R) - 10, 13, GREY),
                  txt(map, "", MAP_X - MAP_R * math.sin(62 * D2R) + 22, MAP_Y - MAP_R * math.cos(62 * D2R) - 4, 13, GREY)];
        var cardclip = clip(g.createChild("group"), MAP_CLIP[0], MAP_CLIP[1], MAP_CLIP[2], MAP_Y - MAP_R * 0.5);
        m.card = cardclip.createChild("group").setCenter(MAP_X, MAP_Y);
        for (var h = 0; h < 360; h += 5) {
            var a = h * D2R; var r1 = MAP_R - (math.mod(h, 10) == 0 ? 10 : 5);
            line(m.card, [[MAP_X + MAP_R * math.sin(a), MAP_Y - MAP_R * math.cos(a)], [MAP_X + r1 * math.sin(a), MAP_Y - r1 * math.cos(a)]], WHITE, 1.5);
            if (math.mod(h, 30) == 0) {
                var s = h == 0 ? "N" : h == 90 ? "E" : h == 180 ? "S" : h == 270 ? "W" : sprintf("%d", h / 10);
                txt(m.card, s, MAP_X + (MAP_R - 20) * math.sin(a), MAP_Y - (MAP_R - 20) * math.cos(a), 14).setRotation(a);
            }
        }
        # the route, its waypoints, the landing zone and its approach into the wind, the planner's points
        m.route = map.createChild("path").setColor(MAGENTA).setStrokeLineWidth(2.5);
        m.wpts = [];
        for (var i = 0; i < WPT_MAX; i += 1) {
            var w = map.createChild("group");
            line(w, [[0, -7], [7, 0], [0, 7], [-7, 0], [0, -7]], MAGENTA, 2);
            append(m.wpts, {g: w, t: txt(w, "", 11, 0, 13, MAGENTA, "left-center")});
        }
        m.lz = map.createChild("group");
        m.lz.createChild("path").circle(9, 0, 0).setColor(WHITE).setStrokeLineWidth(2);
        line(m.lz, [[-6, 0], [6, 0]], WHITE, 2); line(m.lz, [[0, -6], [0, 6]], WHITE, 2);
        m.lzt = txt(m.lz, "", 14, -2, 13, WHITE, "left-center");
        m.hold = map.createChild("group");           # the hover hold's point
        box(m.hold, -8, -8, 16, 16, MAGENTA, nil, 2.5);
        m.holdf = box(m.hold, -5, -5, 10, 10, MAGENTA, MAGENTA, 1);
        m.appl = map.createChild("path").setColor(MAGENTA).setStrokeLineWidth(2).setStrokeDashArray([12, 7]);   # the approach line
        m.applc = fixed_path(m.appl, 1, 2);
        m.appp = map.createChild("group");
        m.appp.createChild("path").circle(7, 0, 0).setColor(MAGENTA).setStrokeLineWidth(2.5);
        m.appt = txt(m.appp, "", 12, 12, 13, MAGENTA, "left-center");
        m.wind_app = map.createChild("path").setColor(CYAN).setStrokeLineWidth(2);
        m.wind_txt = txt(map, "", 0, 0, 12, CYAN, "right-center");
        m.plan = [];
        foreach (var p; [["DECEL", AMBER], ["CONV", GREEN], ["HOVER", CYAN]]) {
            var pg = map.createChild("group");
            pg.createChild("path").circle(5, 0, 0).setColorFill(p[1]).setColor(p[1]);
            txt(pg, p[0], -10, 0, 13, p[1], "right-center");
            append(m.plan, pg);
        }
        # traffic
        m.traffic = [];
        for (var i = 0; i < TRAFFIC_MAX; i += 1) {
            var t = map.createChild("group");
            var d = t.createChild("path").moveTo(0, -7).lineTo(7, 0).lineTo(0, 7).lineTo(-7, 0).close();
            var tt = txt(t, "", 0, -15, 12, CYAN);
            var ar = t.createChild("path").setStrokeLineWidth(1.8);
            append(m.traffic, {g: t, d: d, t: tt, a: ar});
        }
        # ENG OUT's glide reach: the flare's circle, drifting with the wind
        m.reach = map.createChild("path").setColor(AMBER).setStrokeLineWidth(2).setStrokeDashArray([10, 6]);
        m.reacht = txt(map, "GLIDE", 0, 0, 12, AMBER);
        m.reacht.hide();
        # ENG OUT's landing sites (eosite.nas): each a strip, runways white, fields green, out of reach grey, the chosen
        # one magenta with its planned path dashed
        m.sites = [];
        for (var i = 0; i < SITES_SHOWN; i += 1) append(m.sites, map.createChild("path").setStrokeLineWidth(4).hide());
        m.sitepath = map.createChild("path").setColor(MAGENTA).setStrokeLineWidth(2).setStrokeDashArray([8, 5]);
        m.sitet = txt(map, "", 0, 0, 12, MAGENTA);
        m.sitet.hide();
        # the path ahead, the aircraft
        m.path = map.createChild("path").setColor(WHITE).setStrokeLineWidth(2);
        m.pathc = fixed_path(m.path, 1, 21);
        m.dots = [];
        for (var i = 0; i < 2; i += 1) append(m.dots, map.createChild("path").circle(3, 0, 0).setColor(WHITE).setColorFill(WHITE));
        m.acft = g.createChild("path").moveTo(MAP_X, MAP_Y - 13).lineTo(MAP_X + 9, MAP_Y + 9).lineTo(MAP_X, MAP_Y + 4)
                  .lineTo(MAP_X - 9, MAP_Y + 9).close().setColor(WHITE).setColorFill(WHITE).setCenter(MAP_X, MAP_Y);
        # the wind; the top strip; the fuel
        m.wind = g.createChild("group").setTranslation(64, 300);
        line(m.wind, [[0, -14], [0, 14]], WHITE, 2.5); line(m.wind, [[-5, -6], [0, -14], [5, -6]], WHITE, 2.5);
        m.windtxt = txt(g, "", 64, 326, 13);
        box(g, MAP_X - 25, 6, 50, 22, WHITE, [0, 0, 0], 2);
        m.trktxt = txt(g, "000", MAP_X, 17, 16, WHITE, "center-center", MONO);
        m.trklab = txt(g, "TRK", MAP_X - 31, 17, 12, GREY, "right-center");
        m.wpt = txt(g, "", 22, 14, 15, MAGENTA, "left-center");
        m.wptd = txt(g, "", 22, 32, 14, WHITE, "left-center");
        m.rng = txt(g, "", 490, 14, 15, CYAN, "right-center");
        m.mode = txt(g, "", 490, 32, 13, GREY, "right-center");
        m.now = txt(g, "", MAP_X, 56, 15, AMBER);
        m.fuel = txt(g, "", 490, MAP_Y + 4, 13, GREEN, "right-center");
        m.fuelr = txt(g, "", 490, MAP_Y + 22, 13, GREEN, "right-center");
        # ---- the profile along the track
        var vx0 = 40; var vx1 = 492; var vy0 = 384; var vy1 = 486;
        m.v = [vx0, vx1, vy0, vy1];
        box(g, vx0, vy0, vx1 - vx0, vy1 - vy0, [0.25, 0.25, 0.27], [0.04, 0.04, 0.05], 1);
        m.vterr = g.createChild("path").setColorFill([0.36, 0.25, 0.13]).setColor([0.36, 0.25, 0.13]);
        m.vterrc = fixed_path(m.vterr, 1, 43);
        m.vsel = g.createChild("path").setColor(CYAN).setStrokeLineWidth(1.5);
        m.vlz = g.createChild("path").setColor(WHITE).setStrokeLineWidth(2);
        m.vfpa = g.createChild("path").setColor(GREEN).setStrokeLineWidth(2.5);
        m.vglide = g.createChild("path").setColor(AMBER).setStrokeLineWidth(2).setStrokeDashArray([10, 6]);
        m.vacft = g.createChild("path").setColor(WHITE).setColorFill(WHITE);
        m.vscale = [txt(g, "", vx0 - 3, 0, 11, GREY, "right-center"), txt(g, "", vx0 - 3, 0, 11, GREY, "right-center")];
        txt(g, "PROFILE ALONG TRACK", vx1 - 8, vy0 + 10, 11, GREY, "right-center");
        return m;
    },
    key: func(k) {
        if (k == "R1") me.range_i = math.min(size(RANGES) - 1, me.range_i + 1);
        elsif (k == "R2") me.range_i = math.max(0, me.range_i - 1);
        elsif (k == "R3") me.north = !me.north;
        elsif (k == "R4") me.terrain = !me.terrain;
        elsif (k == "R5") eosite.next();
        if (k == "R1" or k == "R2" or k == "R3") {                         # the cells are elsewhere now: a new sweep
            foreach (var set; me.sets) { foreach (var p; set.p) p.reset(); set.g.hide(); }
            me.idx = 0; me.drow = -1;
        }
    },
    update: func {
        var t = systime(); var dt = clamp(t - me.t0, 0.01, 0.5); me.t0 = t;
        var lat = num("/position/latitude-deg"); var lon = num("/position/longitude-deg");
        var coslat = math.cos(lat * D2R);
        var alt = num("/position/altitude-ft");                                   # true: the terrain is too
        var vs = num("/velocities/vertical-speed-fps") * 60;
        var floor = alt - num("/position/gear-agl-m") / 0.3048 + FLOOR_AGL;
        var vn = num("/velocities/speed-north-fps") * KT; var ve = num("/velocities/speed-east-fps") * KT;
        var gs = math.sqrt(vn * vn + ve * ve);
        var thdg = num("/orientation/heading-deg");
        var trk = gs > 20 ? math.atan2(ve, vn) * R2D : thdg;
        var magvar = num("/environment/magnetic-variation-deg");
        var up = me.north ? 0 : trk;
        var rng = RANGES[me.range_i]; var k = MAP_R / rng;
        var cu = math.cos(up * D2R); var su = math.sin(up * D2R);
        # world (NM east, north of the aircraft) to the map, and back
        var to_map = func(de, dn) { [MAP_X + (de * cu - dn * su) * k, MAP_Y - (de * su + dn * cu) * k] };
        var of = func(la, lo) { [(lo - lon) * NM_DEG * coslat, (la - lat) * NM_DEG] };
        # ---- the terrain and the profile: a few elevations each update; the colours when the sweep is through
        for (var i = 0; i < SAMPLES; i += 1) {
            if (me.idx >= size(me.cells) + 41) {
                me.idx = 0;
                if (me.drow < 0) me.start_terrain(alt, vs, gs, k, floor);
            }
            if (me.idx < size(me.cells)) {
                var c = me.cells[me.idx];
                var a = c[2] / k; var b = -c[3] / k;               # NM right, ahead on the map
                var de = a * cu + b * su; var dn = -a * su + b * cu;
                var e = geo.elevation(lat + dn / NM_DEG, lon + de / (NM_DEG * coslat));
                me.elev[me.idx] = e == nil ? nil : e / 0.3048;
            } else {
                var j = me.idx - size(me.cells);                     # the profile: along the track
                var d = rng * j / 40;
                var e = geo.elevation(lat + d * math.cos(trk * D2R) / NM_DEG, lon + d * math.sin(trk * D2R) / (NM_DEG * coslat));
                me.prof[j] = e == nil ? 0 : e / 0.3048;
            }
            me.idx += 1;
        }
        if (me.drow >= 0) me.draw_rows();
        me.sets[me.front].g.setVisible(me.terrain);
        # ---- the card, the range labels, the aircraft
        me.card.setRotation(-(up - magvar) * D2R);
        me.acft.setRotation((me.north ? trk : 0) * D2R);
        me.rlab[0].setText(sprintf("%g", rng / 2)); me.rlab[1].setText(sprintf("%g", rng));
        # ---- the route: the aircraft to the active waypoint and on; the landing zone at its end
        me.route.reset();
        foreach (var w; me.wpts) w.g.hide();
        me.lz.hide(); me.wind_app.reset(); me.wind_txt.setText("");
        foreach (var p; me.plan) p.hide();
        var now = "";
        var n = num("/autopilot/route-manager/route/num");
        var cur = num("/autopilot/route-manager/current-wp");
        var legs = [];                                     # [east, north] from the aircraft: the remaining route
        if (num("/autopilot/route-manager/active") and n > 0 and cur < n) {
            append(legs, [0, 0]);
            for (var i = cur; i < n; i += 1) {
                var wp = "/autopilot/route-manager/route/wp[" ~ i ~ "]/";
                append(legs, of(num(wp ~ "latitude-deg"), num(wp ~ "longitude-deg")));
            }
            var pt = to_map(legs[0][0], legs[0][1]); me.route.moveTo(pt[0], pt[1]);
            for (var i = 1; i < size(legs); i += 1) {
                pt = to_map(legs[i][0], legs[i][1]); me.route.lineTo(pt[0], pt[1]);
                if (i < size(legs) - 1 and i - 1 < WPT_MAX) {
                    me.wpts[i - 1].g.setTranslation(pt[0], pt[1]).show();
                    me.wpts[i - 1].t.setText(getprop("/autopilot/route-manager/route/wp[" ~ (cur + i - 1) ~ "]/id") or "");
                }
            }
            # the landing zone, the approach into the wind
            me.lz.setTranslation(pt[0], pt[1]).show();
            me.lzt.setText(getprop("/autopilot/route-manager/route/wp[" ~ (n - 1) ~ "]/id") or "LZ");
            var wf = num("/environment/wind-from-heading-deg");
            if (num("/environment/wind-speed-kt") > 3) {
                var a = (wf + 180 - up) * D2R;                 # the approach comes from downwind
                for (var s = 14; s < 70; s += 10)
                    me.wind_app.moveTo(pt[0] + s * math.sin(a), pt[1] - s * math.cos(a)).lineTo(pt[0] + (s + 5) * math.sin(a), pt[1] - (s + 5) * math.cos(a));
                me.wind_txt.setTranslation(pt[0] + 74 * math.sin(a) + 8, pt[1] - 74 * math.cos(a)).setText(sprintf("INTO WIND %03d", math.mod(math.round(wf - magvar) + 359, 360) + 1));
            }
            # the planner: back along the route from the landing zone
            var togo = 0;
            for (var i = 1; i < size(legs); i += 1)
                togo += math.sqrt(math.pow(legs[i][0] - legs[i - 1][0], 2) + math.pow(legs[i][1] - legs[i - 1][1], 2));
            if (gs > 40) {
                var d_dec = gs * gs / (2 * DECEL_KTS * 3600);
                var d_conv = math.pow(math.min(gs, CONV_KT), 2) / (2 * DECEL_KTS * 3600);
                var marks = [gs > CONV_KT ? d_dec : nil, d_conv, 0.05];
                for (var j = 0; j < 3; j += 1) {
                    if (marks[j] == nil) continue;
                    if (marks[j] >= togo) { if (j < 2 and now == "") now = j == 0 ? "DECEL NOW" : "CONVERT NOW"; continue; }
                    var p = me.back(legs, marks[j]);
                    var q = to_map(p[0], p[1]);
                    me.plan[j].setTranslation(q[0], q[1]).show();
                }
            }
            var wid = getprop("/autopilot/route-manager/route/wp[" ~ cur ~ "]/id") or "";
            var d0 = math.sqrt(math.pow(legs[1][0], 2) + math.pow(legs[1][1], 2));
            me.wpt.setText(wid);
            var ete = gs > 10 ? d0 / gs * 60 : 0;
            me.wptd.setText(gs > 10 ? sprintf("%.1f NM  %d:%02d", d0, int(ete), math.mod(int(ete * 60), 60)) : sprintf("%.1f NM", d0));
        } else {
            me.wpt.setText(""); me.wptd.setText("");
        }
        me.now.setText(now);
        # ---- the hover hold's point
        var hold = num(JSB ~ "hh/on");
        me.hold.setVisible(hold);
        if (hold) {
            var o = of(num(JSB ~ "hh/lat-t-rad") * R2D, num(JSB ~ "hh/lon-t-rad") * R2D);
            var q = to_map(o[0], o[1]);
            me.hold.setTranslation(q[0], q[1]);
            me.holdf.setVisible(num(JSB ~ "hh/state") == 2);
        }
        # ---- the approach line (VOR/ILS, APPR, autopilot.nas): its course into its point from 15 NM out, and the
        # point (the hover's, until the hover hold has it)
        var lv = num(JSB ~ "ap/line-valid") and !hold;
        me.appl.setVisible(lv); me.appp.setVisible(lv);
        if (lv) {
            var o = of(num(JSB ~ "ap/line-lat-deg"), num(JSB ~ "ap/line-lon-deg"));
            var c = num(JSB ~ "ap/line-course-deg") * D2R;
            var q = to_map(o[0], o[1]); var r = to_map(o[0] - 15 * math.sin(c), o[1] - 15 * math.cos(c));
            me.applc[0].setDoubleValue(r[0]); me.applc[1].setDoubleValue(r[1]); me.applc[2].setDoubleValue(q[0]); me.applc[3].setDoubleValue(q[1]);
            me.appp.setTranslation(q[0], q[1]);
            me.appt.setText(size(legs) ? "" : autopilot.label);          # (the route names its own points)
        }
        # ---- the path ahead: the turn 60 s on, dots at 30 and 60 s
        var hd = gs > 20 ? trk : thdg;
        if (me.trk0 != nil) {
            var dpsi = math.mod(hd - me.trk0 + 540, 360) - 180;
            me.rate = lowpass(me.rate, dpsi / dt, dt, 1.5);
        }
        me.trk0 = hd;
        me.path.setVisible(gs > 15); me.dots[0].setVisible(gs > 15); me.dots[1].setVisible(gs > 15);
        if (gs > 15) {
            var w = me.rate * D2R; var v = gs / 3600;
            var rot = (me.north ? trk : 0) * D2R;
            var n = 0;
            for (var s = 0; s <= 60; s += 3) {
                var a = math.abs(w) < 1e-4 ? 0 : v / w * (1 - math.cos(w * s));
                var b = math.abs(w) < 1e-4 ? v * s : v / w * math.sin(w * s);
                var x = MAP_X + (a * math.cos(rot) + b * math.sin(rot)) * k; var y = MAP_Y - (-a * math.sin(rot) + b * math.cos(rot)) * k;
                me.pathc[n].setDoubleValue(x); me.pathc[n + 1].setDoubleValue(y); n += 2;
                if (s == 30 or s == 60) me.dots[s == 30 ? 0 : 1].setTranslation(x, y);
            }
        }
        # ---- ENG OUT: the glide's reach to the flare (the flight model's, in still air) about where the wind takes the
        # air the aircraft glides in over the glide's time
        me.reach.reset(); me.reacht.hide();
        me.eo_r = -1;
        if (num(AFCS ~ "eng-out") and num(JSB ~ "ap/eo-phase") < 2.5 and num(JSB ~ "ctl/nacelle-deg") < 1) {
            var r = num(JSB ~ "ap/eo-reach-nm"); var ts = num(JSB ~ "ap/eo-reach-s");
            var wkt = num("/environment/wind-speed-kt"); var wto = (num("/environment/wind-from-heading-deg") + 180) * D2R;
            var ce = wkt * ts / 3600 * math.sin(wto); var cn = wkt * ts / 3600 * math.cos(wto);
            for (var i = 0; i <= 48; i += 1) {
                var a = i / 48 * 2 * math.pi;
                var q = to_map(ce + r * math.sin(a), cn + r * math.cos(a));
                if (i == 0) me.reach.moveTo(q[0], q[1]); else me.reach.lineTo(q[0], q[1]);
            }
            var q = to_map(ce + r * math.sin(up * D2R), cn + r * math.cos(up * D2R));
            me.reacht.setTranslation(q[0], q[1] - 10).show();
            # (along the track, for the profile: the circle's edge ahead)
            var at = (ce * math.sin(trk * D2R) + cn * math.cos(trk * D2R));
            var ac = -ce * math.cos(trk * D2R) + cn * math.sin(trk * D2R);
            me.eo_r = at + math.sqrt(math.max(0, r * r - ac * ac));
        }
        # ---- ENG OUT's landing sites and the planned path to the chosen one
        var ns = num(AFCS ~ "eng-out") ? math.min(SITES_SHOWN, int(num(SITE ~ "n"))) : 0;
        forindex (var i; me.sites) {
            var p = me.sites[i];
            if (i >= ns) { p.hide(); continue; }
            var b = SITE ~ "cand[" ~ i ~ "]/";
            var q0 = of(num(b ~ "lat0"), num(b ~ "lon0")); var q1 = of(num(b ~ "lat1"), num(b ~ "lon1"));
            var a0 = to_map(q0[0], q0[1]); var a1 = to_map(q1[0], q1[1]);
            p.reset().moveTo(a0[0], a0[1]).lineTo(a1[0], a1[1]);
            # (the chosen on top: a runway's two ends are one strip, and 19L's covered 01R's)
            var ch = num(b ~ "chosen");
            p.setColor(ch ? MAGENTA : !num(b ~ "reach") ? GREY : getprop(b ~ "kind") == "RWY" ? WHITE : GREEN);
            p.setInt("z-index", ch ? 1 : 0);
            p.show();
        }
        me.sitepath.reset(); me.sitet.hide();
        if (ns and num(SITE ~ "on")) {
            var np = int(num(SITE ~ "path/n"));
            var last = nil;
            for (var j = 0; j < np; j += 1) {
                var q = of(num(SITE ~ "path/pt[" ~ j ~ "]/lat"), num(SITE ~ "path/pt[" ~ j ~ "]/lon"));
                last = to_map(q[0], q[1]);
                if (j == 0) me.sitepath.moveTo(last[0], last[1]); else me.sitepath.lineTo(last[0], last[1]);
            }
            if (last != nil) me.sitet.setText(getprop(SITE ~ "name")).setTranslation(last[0], last[1] + 14).show();
        }
        # ---- traffic
        var shown = 0;
        foreach (var kind; ["aircraft", "multiplayer"]) {
            foreach (var a; props.globals.getNode("/ai/models", 1).getChildren(kind)) {
                if (shown >= TRAFFIC_MAX) break;
                if (!(a.getValue("valid") or 0)) continue;
                var o = of(a.getValue("position/latitude-deg") or 0, a.getValue("position/longitude-deg") or 0);
                var q = to_map(o[0], o[1]);
                if (math.sqrt(math.pow(q[0] - MAP_X, 2) + math.pow(q[1] - MAP_Y, 2)) > MAP_R) continue;
                var rel = ((a.getValue("position/altitude-ft") or 0) - num("/position/altitude-ft")) / 100;
                var near = math.abs(rel) < 10 and math.sqrt(o[0] * o[0] + o[1] * o[1]) < 2;
                var col = near ? AMBER : CYAN;
                var tr = me.traffic[shown];
                tr.g.setTranslation(q[0], q[1]).show();
                tr.d.setColor(col).setColorFill(col);
                tr.t.setText(sprintf("%+03d", math.round(rel))).setColor(col).setTranslation(0, rel > 0 ? -15 : 15);
                var tvs = (a.getValue("velocities/vertical-speed-fps") or 0) * 60;
                tr.a.reset().setColor(col);
                if (math.abs(tvs) > 500) {
                    var sgn = tvs > 0 ? -1 : 1;
                    tr.a.moveTo(11, -5 * sgn).lineTo(11, 5 * sgn).moveTo(8, 2 * sgn).lineTo(11, 5 * sgn).lineTo(14, 2 * sgn);
                }
                shown += 1;
            }
        }
        for (var i = shown; i < TRAFFIC_MAX; i += 1) me.traffic[i].g.hide();
        # ---- the wind, the strips, the fuel
        var wf = num("/environment/wind-from-heading-deg"); var ws = num("/environment/wind-speed-kt");
        me.wind.setRotation((wf + 180 - up) * D2R).setVisible(ws > 2);
        me.windtxt.setText(ws > 2 ? sprintf("%03d/%d", math.mod(math.round(wf - magvar) + 359, 360) + 1, math.round(ws)) : "");
        me.trktxt.setText(sprintf("%03d", math.mod(math.round(trk - magvar) + 359, 360) + 1));
        me.trklab.setText(me.north ? "TRK  N UP" : "TRK");
        me.rng.setText(sprintf("%g NM", rng));
        me.mode.setText((me.north ? "N UP" : "TRK UP") ~ (me.terrain ? "  TERR" : ""));
        var flow = num("/engines/engine[0]/fuel-flow-pph") + num("/engines/engine[1]/fuel-flow-pph");
        var fuel = num("/consumables/fuel/total-fuel-lbs");
        if (flow > 50) {
            var h = (fuel - flow / 3) / flow;                  # to a 20 min reserve
            me.fuel.setText(sprintf("FUEL %d:%02d", int(math.max(0, h)), math.mod(int(math.max(0, h) * 60), 60)));
            me.fuelr.setText(gs > 30 ? sprintf("%d NM AT RES", math.max(0, h) * gs) : "");
            var low = h < 0.33;
            me.fuel.setColor(low ? AMBER : GREEN); me.fuelr.setColor(low ? AMBER : GREEN);
        } else {
            me.fuel.setText(""); me.fuelr.setText("");
        }
        # ---- the profile
        me.draw_profile(alt, vs, gs, rng, legs);
    },
    # the point d NM back along the route (legs: [east, north] from the aircraft) from its end
    back: func(legs, d) {
        for (var i = size(legs) - 1; i > 0; i -= 1) {
            var a = legs[i - 1]; var b = legs[i];
            var L = math.sqrt(math.pow(b[0] - a[0], 2) + math.pow(b[1] - a[1], 2));
            if (d <= L) { var f = L > 0 ? d / L : 0; return [b[0] + (a[0] - b[0]) * f, b[1] + (a[1] - b[1]) * f]; }
            d -= L;
        }
        return legs[0];
    },
    # the terrain's clearance at each sample point when a sweep is through, and whether it has an elevation there (the
    # neighbours' mean, so that the edges of the scenery's tiles come out smooth too)
    start_terrain: func(alt, vs, gs, k, floor) {
        me.bands = gs > 90 ? [100, 500, 1500, 0.5] : [0, 100, 300, 0.5];      # the last: no data
        var clr = setsize([], size(me.cells));
        for (var i = 0; i < size(me.cells); i += 1) {
            var e = me.elev[i];
            if (e == nil) continue;
            var c = me.cells[i];
            var dist = math.sqrt(c[2] * c[2] + c[3] * c[3]) / k;
            clr[i] = math.max(floor, alt + vs * math.min(60, dist / math.max(gs, 30) * 3600) / 60) - e;
        }
        var nx = me.nx; var gi = me.gi;
        var known = setsize([], size(me.cells));
        for (var j = 0; j < me.ny; j += 1)
            for (var i = 0; i < nx; i += 1) {
                var q = gi[j * nx + i];
                if (q < 0) continue;
                var n = 0; var sum = 0;
                for (var jj = math.max(0, j - 1); jj <= math.min(me.ny - 1, j + 1); jj += 1)
                    for (var ii = math.max(0, i - 1); ii <= math.min(nx - 1, i + 1); ii += 1) {
                        var r = gi[jj * nx + ii];
                        if (r >= 0) { n += 1; if (clr[r] != nil) sum += 1; }
                    }
                known[q] = sum / n;
            }
        me.clr = clr; me.known = known; me.drow = 0;
        foreach (var p; me.sets[1 - me.front].p) p.reset();
    },
    # each square of four sample points: the part of it below each band's clearance, the edges interpolated between the
    # corners (marching squares), whole squares merged along the rows; the sets swapped when the last row is drawn
    draw_rows: func {
        var nx = me.nx; var gi = me.gi; var clr = me.clr; var bands = me.bands;
        var paths = me.sets[1 - me.front].p;
        for (var j = me.drow; j < math.min(me.drow + ROWS, me.ny - 1); j += 1) {
            var y = MAP_CLIP[1] + j * CELL;
            var run = [-1, -1, -1, -1];
            for (var i = 0; i < nx; i += 1) {
                var x = MAP_CLIP[0] + i * CELL;
                var vv = nil;
                if (i < nx - 1) {
                    var q = [gi[j * nx + i], gi[j * nx + i + 1], gi[(j + 1) * nx + i + 1], gi[(j + 1) * nx + i]];
                    if (q[0] >= 0 and q[1] >= 0 and q[2] >= 0 and q[3] >= 0) {
                        var v = [clr[q[0]], clr[q[1]], clr[q[2]], clr[q[3]]];
                        var n = 0; var sum = 0;
                        foreach (var c; v) if (c != nil) { n += 1; sum += c; }
                        var fill = n ? sum / n : 1e6;          # no elevation: the square's mean clearance
                        vv = [[], [], [], [me.known[q[0]], me.known[q[1]], me.known[q[2]], me.known[q[3]]]];
                        foreach (var c; v) {
                            var w = c == nil ? fill : c;
                            append(vv[0], w); append(vv[1], w); append(vv[2], w);
                        }
                    }
                }
                for (var b = 0; b < 4; b += 1) {
                    var t = bands[b];
                    var n = 0;
                    if (vv != nil) foreach (var c; vv[b]) if (c < t) n += 1;
                    if (n == 4) { if (run[b] < 0) run[b] = x; continue; }
                    if (run[b] >= 0) { paths[b].rect(run[b], y, x - run[b], CELL); run[b] = -1; }
                    if (n > 0) me.square(paths[b], x, y, vv[b], t);
                }
            }
        }
        me.drow += ROWS;
        if (me.drow >= me.ny - 1) {
            me.sets[me.front].g.hide();
            me.front = 1 - me.front; me.drow = -1;
        }
    },
    # the part of a square (corners clockwise from the top left) where v < t
    square: func(p, x, y, v, t) {
        var first = 1;
        for (var c = 0; c < 4; c += 1) {
            var d = NEXT[c];
            var pts = [];
            if (v[c] < t) append(pts, [x + SQ_X[c] * CELL, y + SQ_Y[c] * CELL]);
            if ((v[c] < t) != (v[d] < t)) {
                var f = (t - v[c]) / (v[d] - v[c]);
                append(pts, [x + (SQ_X[c] + (SQ_X[d] - SQ_X[c]) * f) * CELL, y + (SQ_Y[c] + (SQ_Y[d] - SQ_Y[c]) * f) * CELL]);
            }
            foreach (var pt; pts) {
                if (first) p.moveTo(pt[0], pt[1]); else p.lineTo(pt[0], pt[1]);
                first = 0;
            }
        }
        p.close();
    },
    draw_profile: func(alt, vs, gs, rng, legs) {
        var v = me.v; var top = alt + 1000;
        foreach (var e; me.prof) if (e != nil) top = math.max(top, e + 500);
        top = math.ceil(top / 1000) * 1000;
        var vx = func(d) { v[0] + 6 + d / rng * (v[1] - v[0] - 12) };
        var vy = func(ft) { v[3] - 4 - clamp(ft, 0, top) / top * (v[3] - v[2] - 10) };
        var tc = me.vterrc;
        tc[0].setDoubleValue(vx(0)); tc[1].setDoubleValue(vy(0));
        for (var j = 0; j <= 40; j += 1) {
            tc[2 * j + 2].setDoubleValue(vx(rng * j / 40)); tc[2 * j + 3].setDoubleValue(vy(me.prof[j] == nil ? 0 : me.prof[j]));
        }
        tc[84].setDoubleValue(vx(rng)); tc[85].setDoubleValue(vy(0));
        var sel = num("/autopilot/settings/target-altitude-ft");
        me.vsel.reset();
        if (sel > 0) me.vsel.moveTo(vx(0), vy(sel)).lineTo(vx(rng), vy(sel));
        me.vacft.reset().moveTo(vx(0) - 6, vy(alt) - 5).lineTo(vx(0) + 7, vy(alt)).lineTo(vx(0) - 6, vy(alt) + 5).close();
        var gam = gs > 10 ? vs / (gs * 101.27) : 0;          # the path: ft per ft of ground
        # (ended where it leaves the profile's height scale: drawn on to 0.6 of the range and clamped there, a steep
        # descent's line bent along the bottom and read shallower than it was)
        var pd = rng * 0.6;
        if (gam < 0 and alt + gam * pd * 6076 < 0) pd = alt / (-gam * 6076);
        elsif (gam > 0 and alt + gam * pd * 6076 > top) pd = (top - alt) / (gam * 6076);
        me.vfpa.reset().moveTo(vx(0), vy(alt)).lineTo(vx(pd), vy(alt + gam * pd * 6076));
        # ENG OUT's glide, to the flare's height over the ground here, at the reach ahead
        me.vglide.reset();
        if (me.eo_r > 0) {
            var fl = alt - num("/position/gear-agl-m") / 0.3048 + num(JSB ~ "ap/eo-flare-ft");
            var d = math.min(me.eo_r, rng); var z = alt + (fl - alt) * d / me.eo_r;
            me.vglide.moveTo(vx(0), vy(alt)).lineTo(vx(d), vy(z));
            if (d == me.eo_r) me.vglide.moveTo(vx(d) - 6, vy(z)).lineTo(vx(d) + 6, vy(z));
        }
        me.vlz.reset();
        if (size(legs) > 1) {
            var togo = 0;
            for (var i = 1; i < size(legs); i += 1)
                togo += math.sqrt(math.pow(legs[i][0] - legs[i - 1][0], 2) + math.pow(legs[i][1] - legs[i - 1][1], 2));
            if (togo < rng) {
                var lze = geo.elevation(num("/autopilot/route-manager/route/wp[" ~ (num("/autopilot/route-manager/route/num") - 1) ~ "]/latitude-deg"),
                                        num("/autopilot/route-manager/route/wp[" ~ (num("/autopilot/route-manager/route/num") - 1) ~ "]/longitude-deg"));
                var ly = vy(lze == nil ? 0 : lze / 0.3048);
                me.vlz.moveTo(vx(togo) - 7, ly).lineTo(vx(togo) + 7, ly).moveTo(vx(togo), ly).lineTo(vx(togo), ly - 12);
            }
        }
        me.vscale[0].setTranslation(v[0] - 3, vy(top / 2)).setText(sprintf("%gK", top / 2000));
        me.vscale[1].setTranslation(v[0] - 3, vy(top)).setText(sprintf("%gK", top / 1000));
    },
};

# ---------------------------------------------------------------- the engine and drive format
var bar = func(g, x, y0, h, w, color) {
    return g.createChild("path").rect(x, y0 - h, w, h).setColorFill(color).setColor(color);
}

# an alert window (Nasal/alerts.nas), the ENG format's and the CDU EICAS page's: warnings red, cautions and let-go
# autopilot modes amber, advisories green, the latest first in each; a new one (not yet acknowledged) in reverse video;
# the last line counts what does not fit
var ALERT_COLOUR = {warning: RED, caution: AMBER, autopilot: AMBER, advisory: GREEN};
var ALERT_SHORT = {warning: "WARN", caution: "CAUT", autopilot: "CAUT", advisory: "ADVY"};
var BLACK = [0, 0, 0];

var AlertWindow = {
    new: func(g, x, y, w, rows, lh, fs) {
        var m = {parents: [AlertWindow], rows: []};
        box(g, x, y, w, rows * lh + 8, GREY, nil, 1);
        for (var i = 0; i < rows; i += 1) {
            var ry = y + 4 + lh * i;
            append(m.rows, {back: box(g, x + 4, ry, w - 8, lh, BLACK, BLACK, 0),
                            text: txt(g, "", x + 16, ry + lh / 2 + 1, fs, WHITE, "left-center")});
        }
        return m;
    },
    update: func {
        var list = alerts.standing();
        var rows = size(me.rows);
        var n = size(list) > rows ? rows - 1 : size(list);
        forindex (var i; me.rows) {
            var r = me.rows[i];
            if (i < n) {
                var a = list[i]; var c = ALERT_COLOUR[a[1]];
                r.back.setColorFill(a[2] ? c : BLACK);
                r.text.setText(a[0]).setColor(a[2] ? BLACK : c);
            } elsif (i == n and size(list) > rows) {
                var more = {}; var order = [];
                for (var j = n; j < size(list); j += 1) {
                    var k = ALERT_SHORT[list[j][1]];
                    if (!contains(more, k)) { more[k] = 0; append(order, k); }
                    more[k] += 1;
                }
                var s = "";
                foreach (var k; order) s ~= sprintf("+%d %s  ", more[k], k);
                r.back.setColorFill(BLACK);
                r.text.setText(s).setColor(WHITE);
            } else {
                r.back.setColorFill(BLACK);
                r.text.setText("");
            }
        }
    },
};

# the engines' torque bars (0-120 %, the limit's line amber) and readouts either side, the rotor speed and the drive's
# power between: the ENG format's and the CDU EICAS page's. x1, x2: the engines' columns, xm the middle's; y0 the top,
# h the bars' height; brake: the rotor brake under the middle
var Gauges = {
    new: func(g, x1, x2, xm, y0, h, brake) {
        var m = {parents: [Gauges], h: h};
        m.cols = [];
        foreach (var e; [0, 1]) {
            var x = e == 0 ? x1 : x2;
            var c = {};
            txt(g, sprintf("ENG %d", e + 1), x, y0 + 16, 20, WHITE);
            box(g, x - 22, y0 + 32, 44, h, GREY, nil, 2);
            # the engine's limit (fdm/alerts.py: its own power or its share of the drive's, whichever is less),
            # moved to it
            c.lim = line(g, [[x - 28, 0], [x + 28, 0]], AMBER, 2);
            c.bar = bar(g, x - 18, y0 + 30 + h, 0, 36, GREEN);
            c.y = y0 + 30 + h;
            var y = y0 + 32 + h;
            c.tq = txt(g, "0", x, y + 18, 22, GREEN, "center-center", MONO);
            txt(g, "TQ %", x, y + 38, 16, GREY);
            c.ng = txt(g, "NG 0", x, y + 62, 18, WHITE, "center-center", MONO);
            c.shp = txt(g, "SHP 0", x, y + 84, 18, WHITE, "center-center", MONO);
            c.ff = txt(g, "FF 0", x, y + 106, 18, WHITE, "center-center", MONO);
            c.state = txt(g, "", x, y + 128, 18, AMBER, "center-center");
            c.x = x;
            append(m.cols, c);
        }
        txt(g, "NR", xm, y0 + 36, 20, GREY);
        m.nr = txt(g, "0", xm, y0 + 64, 32, GREEN, "center-center", MONO);
        txt(g, "% REF", xm, y0 + 90, 16, GREY);
        m.ref = txt(g, "0", xm, y0 + 108, 18, CYAN, "center-center", MONO);
        txt(g, "DRIVE", xm, y0 + 138, 18, GREY);
        m.req = txt(g, "0", xm, y0 + 158, 18, WHITE, "center-center", MONO);
        txt(g, "LIMIT", xm, y0 + 182, 16, GREY);
        m.lim = txt(g, "0", xm, y0 + 202, 18, WHITE, "center-center", MONO);
        m.brake = brake ? txt(g, "", xm, y0 + 234, 18, AMBER) : nil;
        return m;
    },
    update: func {
        forindex (var e; me.cols) {
            var c = me.cols[e];
            var tq = num("/engines/engine[" ~ e ~ "]/torque-pct");
            var h = math.min(1.2, math.max(0, tq / 100)) / 1.2 * (me.h - 4);
            c.bar.reset().rect(c.x - 18, c.y - h, 36, h);
            var lp = num(JSB ~ "alert/limit-pct", 100);
            c.lim.setTranslation(0, c.y + 2 - math.min(1.2, lp / 100) / 1.2 * me.h);
            c.tq.setText(sprintf("%d", tq)).setColor(tq >= lp - 1 ? AMBER : GREEN);
            c.ng.setText(sprintf("NG %d", num("/engines/engine[" ~ e ~ "]/n1-pct")));
            c.shp.setText(sprintf("SHP %d", num(JSB ~ "eng[" ~ e ~ "]/power-hp")));
            c.ff.setText(sprintf("FF %d", num("/engines/engine[" ~ e ~ "]/fuel-flow-pph")));
            var run = num(JSB ~ "eng[" ~ e ~ "]/running"); var sp = num(JSB ~ "eng[" ~ e ~ "]/spool");
            c.state.setText(run and sp < 0.99 ? "START" : !run and sp > 0.01 ? "SPOOL DN" : !run ? "OFF" : "");
        }
        me.nr.setText(sprintf("%d", num(JSB ~ "drive/nr-pct")));
        me.ref.setText(sprintf("%d", num(JSB ~ "gov/nr-ref-pct")));
        me.req.setText(sprintf("%d HP", num(JSB ~ "drive/power-required-hp")));
        # the drive's limit: each running engine's share of it (fdm/drive.py) times the engines running
        var nrun = math.max(1, (num(JSB ~ "eng[0]/running") > 0.5) + (num(JSB ~ "eng[1]/running") > 0.5));
        me.lim.setText(sprintf("%d HP", nrun * num(JSB ~ "eng/xmsn-limit-hp")));
        if (me.brake != nil) me.brake.setText(num("/controls/rotor/brake") > 0.05 ? "ROTOR BRAKE ON" : "");
    },
};

var Engines = {
    new: func(g) {
        var m = {parents: [Engines]};
        txt(g, "ENGINES AND DRIVE", 256, 42, 20);
        m.alerts = AlertWindow.new(g, 24, 56, 464, 6, 22, 20);
        m.gauges = Gauges.new(g, 120, 392, 256, 202, 140, 1);
        return m;
    },
    update: func {
        me.alerts.update();
        me.gauges.update();
    },
};

# ---------------------------------------------------------------- the fuel format
var TANKS = [["L FEED", 120, 200], ["R FEED", 392, 200], ["L FWD SPN", 120, 320], ["R FWD SPN", 392, 320], ["R AFT SPN", 392, 420]];

var Fuel = {
    new: func(g) {
        var m = {parents: [Fuel]};
        txt(g, "FUEL", 256, 52, 22);
        m.q = [];
        forindex (var i; TANKS) {
            var t = TANKS[i];
            box(g, t[1] - 70, t[2] - 30, 140, 64, GREY, nil, 2);
            txt(g, t[0], t[1], t[2] - 16, 16, GREY);
            append(m.q, txt(g, "0", t[1], t[2] + 12, 22, GREEN, "center-center", MONO));
        }
        txt(g, "TOTAL", 256, 120, 18, GREY);
        m.total = txt(g, "0", 256, 146, 30, GREEN, "center-center", MONO);
        m.flow = txt(g, "", 120, 420, 18, WHITE, "center-center", MONO);
        m.endur = txt(g, "", 120, 446, 18, WHITE, "center-center", MONO);
        return m;
    },
    update: func {
        var total = 0;
        forindex (var i; me.q) {
            var l = num("/consumables/fuel/tank[" ~ i ~ "]/level-lbs");
            total += l;
            me.q[i].setText(sprintf("%d", l));
        }
        me.total.setText(sprintf("%d LB", total)).setColor(total < 1000 ? AMBER : GREEN);
        var ff = num("/engines/engine[0]/fuel-flow-pph") + num("/engines/engine[1]/fuel-flow-pph");
        me.flow.setText(sprintf("FLOW %d PPH", ff));
        me.endur.setText(ff > 1 ? sprintf("ENDUR %d:%02d", int(total / ff), int(math.mod(total / ff * 60, 60))) : "");
    },
};

# ---------------------------------------------------------------- the multifunction display
var MFD_PAGES = ["FLT", "NAV", "ENG", "FUEL"];
var ON_DEMAND = {FLT: 1, NAV: 1};
var PERIOD = {FLT: 1, NAV: 2, ENG: 4, FUEL: 4};     # 20 Hz ticks per update: the flight format 20 Hz, the map 10, the systems 5          # the formats whose key labels show only for a while after a key is pressed
var LABEL_TIME = 5;                # s

var Mfd = {
    new: func(name, page) {
        var m = {parents: [Mfd], name: name};
        m.c = canvas.new({"name": name, "size": [512, 512], "view": [512, 512], "mipmapping": 1});
        m.c.addPlacement({"node": name ~ ".screen"});
        m.c.setColorBackground(0, 0, 0, 1);
        m.pages = {};
        m.groups = {};
        m.labs = {};
        foreach (var p; MFD_PAGES) {
            var g = m.c.createGroup();
            m.groups[p] = g;
            m.pages[p] = (p == "FLT" ? Flight : p == "NAV" ? Nav : p == "ENG" ? Engines : Fuel).new(g);
            m.labs[p] = labels(g, MFD_PAGES, [], p == "FLT" ? ["", "", "", "", "", "", "HVR MARK"]
                                                 : p == "NAV" ? ["RNG +", "RNG -", "N/TRK UP", "TERR", "SITE"] : [], contains(ON_DEMAND, p));
            g.hide();
        }
        m.touched = -100;
        m.page = "";
        m.tick = name[3] - `0`;              # MFD1..4: the slower formats' updates staggered
        m.show(page);
        cache_writes(m);
        return m;
    },
    show: func(p) {
        if (me.page != "") me.groups[me.page].hide();
        me.page = p;
        me.groups[p].show();
        me.pages[p].update();
    },
    key: func(k) {
        me.touched = systime();
        if (k == "ACK") alerts.acknowledge();
        elsif (k == "CA") me.show("ENG");                   # the caution/advisory key: the format with the alert window
        elsif (k[0] == `T`) {
            var i = num_key(k) - 1;
            if (i < size(MFD_PAGES)) me.show(MFD_PAGES[i]);
        } elsif (me.page == "FLT" or me.page == "NAV") me.pages[me.page].key(k);
    },
    update: func {
        if (contains(ON_DEMAND, me.page)) me.labs[me.page].setVisible(systime() - me.touched < LABEL_TIME);
        me.tick += 1;
        if (math.mod(me.tick, PERIOD[me.page]) == 0) me.pages[me.page].update();
    },
};

# the number after a key's or an entry's first letter ("T3": 3, "H270": 270); 0 if there are other characters
var num_key = func(k) {
    var v = 0;
    for (var i = 1; i < size(k); i += 1) {
        if (k[i] < `0` or k[i] > `9`) return 0;
        v = v * 10 + k[i] - `0`;
    }
    return v;
}

# ---------------------------------------------------------------- the control display unit: EICAS, status, the scratchpad
var Cdu = {
    new: func {
        var m = {parents: [Cdu]};
        m.c = canvas.new({"name": "CDU", "size": [640, 480], "view": [640, 480], "mipmapping": 1});
        m.c.addPlacement({"node": "CDU.screen"});
        m.c.setColorBackground(0, 0, 0, 1);
        var g = m.c.createGroup();
        m.title = txt(g, "", 320, 26, 24, WHITE);
        line(g, [[20, 44], [620, 44]], GREY, 2);
        m.text = g.createChild("group");
        m.lines = [];
        for (var i = 0; i < 12; i += 1) append(m.lines, txt(m.text, "", 40, 70 + 30 * i, 22, GREEN, "left-center", MONO));
        # the EICAS page, the one the CDU shows unless another is called up: the engines and the drive on the left (as
        # the ENG format), the alerts on the right, under them the fuel and the configuration, each amber while its
        # alert stands
        m.eicas = g.createChild("group");
        m.gauges = Gauges.new(m.eicas, 80, 260, 170, 48, 150, 0);
        m.alerts = AlertWindow.new(m.eicas, 336, 52, 284, 11, 24, 20);
        m.status = [];
        foreach (var r; ["FUEL", "NAC", "GEAR", "OAT"]) {
            var y = 344 + 22 * size(m.status);
            txt(m.eicas, r, 340, y, 16, GREY, "left-center");
            append(m.status, txt(m.eicas, "", 392, y, 18, GREEN, "left-center", MONO));
        }
        line(g, [[20, 436], [620, 436]], GREY, 2);
        m.pad = txt(g, "", 30, 458, 24, WHITE, "left-center", MONO);
        m.scratch = "";
        m.page = "EICAS";
        m.tick = 0;
        return m;
    },
    eicas_update: func {
        me.gauges.update();
        me.alerts.update();
        var amber = func(id) num(JSB ~ "alert/" ~ id) > 0.5 ? AMBER : GREEN;
        var fuel = num("/consumables/fuel/total-fuel-lbs");
        var ff = num("/engines/engine[0]/fuel-flow-pph") + num("/engines/engine[1]/fuel-flow-pph");
        me.status[0].setText(sprintf("%5d LB", fuel) ~ (ff > 1 ? sprintf("  %d:%02d", int(fuel / ff), int(math.mod(fuel / ff * 60, 60))) : ""))
                    .setColor(amber("fuel-low"));
        me.status[1].setText(sprintf("%4.1f  %d-%d", num(JSB ~ "ctl/nacelle-deg"), num(JSB ~ "ctl/cps-min-deg"),
                                     math.min(97.5, num(JSB ~ "ctl/cps-max-deg", 97.5)))).setColor(amber("conv-prot"));
        var gear = num(JSB ~ "gear/gear-pos-norm", 1);
        me.status[2].setText(sprintf("%-5s  FLAP %d", gear > 0.99 ? "DN" : gear < 0.01 ? "UP" : "TRANS", num(JSB ~ "ctl/flaperon-deg")))
                    .setColor(amber("gear-down"));
        me.status[3].setText(sprintf("%d C  GW %d", num("/environment/temperature-degc"), num(JSB ~ "inertia/weight-lbs")));
    },
    key: func(k) {
        if (k == "B4") me.page = "EICAS";
        elsif (k == "B3") me.page = "STAT";
        elsif (k == "B2") me.page = "C/A";
        elsif (k == "B5") alerts.acknowledge();
        elsif (k == "B1" or k == "B6") me.page = "CDU";
        me.update();
    },
    type: func(k) {
        if (k == "CLR") me.scratch = "";
        elsif (k == "DEL") me.scratch = size(me.scratch) ? substr(me.scratch, 0, size(me.scratch) - 1) : "";
        elsif (k == "ENTER") me.enter();
        elsif (size(k) == 1 or k == "SLASH" or k == "DOT" or k == "MINUS" or k == "PLUS") {
            var c = k == "SLASH" ? "/" : k == "DOT" ? "." : k == "MINUS" ? "-" : k == "PLUS" ? "+" : k;
            if (size(me.scratch) < 24) me.scratch ~= c;
        }
        me.pad.setText(me.scratch);
    },
    # ENTER: a number on the scratchpad sets the heading bug (H123), the course (C123), the altimeter (B2992), TF's
    # clearance (T300, 100-1000 ft) or ride (R1 soft, R2 medium, R3 hard), else nothing
    enter: func {
        var s = me.scratch;
        if (size(s) > 1) {
            var v = num_key(s);
            if (s[0] == `H`) setprop("/autopilot/settings/heading-bug-deg", math.mod(v, 360));
            elsif (s[0] == `C`) setprop("/instrumentation/nav[0]/radials/selected-deg", math.mod(v, 360));
            elsif (s[0] == `B`) setprop("/instrumentation/altimeter/setting-inhg", v / 100);
            elsif (s[0] == `T` and v) setprop("/autopilot/settings/target-agl-ft", math.max(100, math.min(1000, v)));
            elsif (s[0] == `R` and v) autopilot.tf_ride(v);
        }
        me.scratch = "";
    },
    # run with the fast units (20 Hz): the EICAS page at 10 Hz, the text pages at 4 Hz; a key's own call at once
    update: func(now = 0) {
        me.tick += 1;
        var eicas = me.page == "EICAS";
        me.eicas.setVisible(eicas);
        me.text.setVisible(!eicas);
        if (eicas) {
            me.title.setText("EICAS");
            if (now or !math.mod(me.tick, 2)) me.eicas_update();
            return;
        }
        if (!now and math.mod(me.tick, 5)) return;
        var rows = [];
        if (me.page == "C/A") {
            # the alerts cleared, the latest first, and how long ago
            me.title.setText("C/A HISTORY");
            var t = num("/sim/time/elapsed-sec");
            foreach (var h; alerts.history) {
                var ago = int(t - h[3]);
                append(rows, [sprintf("  %-18s %3d:%02d AGO", h[0], int(ago / 60), math.mod(ago, 60)), ALERT_COLOUR[h[1]]]);
            }
            if (!size(rows)) append(rows, ["NONE CLEARED", GREY]);
        } elsif (me.page == "STAT") {
            me.title.setText("STATUS");
            var w = num(JSB ~ "inertia/weight-lbs"); var f = num("/consumables/fuel/total-fuel-lbs");
            append(rows, [sprintf("GROSS WEIGHT  %6d LB", w), GREEN]);
            append(rows, [sprintf("FUEL          %6d LB", f), GREEN]);
            append(rows, [sprintf("CARGO         %6d LB", num(JSB ~ "inertia/pointmass-weight-lbs[5]")), GREEN]);
            append(rows, [sprintf("CG            %6.1f IN", num(JSB ~ "inertia/cg-x-in")), GREEN]);
            append(rows, [sprintf("OAT           %6d C", num("/environment/temperature-degc")), GREEN]);
            append(rows, [sprintf("DENSITY ALT   %6d FT", num("/environment/density-altitude-ft")), GREEN]);
            append(rows, [sprintf("NACELLE       %6.1f DEG", num(JSB ~ "ctl/nacelle-deg")), GREEN]);
            append(rows, [sprintf("CONV CORRIDOR %3d-%3d DEG", num(JSB ~ "ctl/cps-min-deg"), math.min(97.5, num(JSB ~ "ctl/cps-max-deg", 97.5))), GREEN]);
        } else {
            me.title.setText("CDU");
            foreach (var t; ["EICAS: ENGINES, ALERTS, FUEL, CONFIG", "C/A: ALERTS CLEARED", "ACK: ACKNOWLEDGE", "STAT: WEIGHT, FUEL, CG", "", "SCRATCHPAD ENTRIES (ENTER):",
                             " H123  HEADING BUG", " C123  NAV 1 COURSE", " B2992 ALTIMETER, IN HG",
                             " T300  TF CLEARANCE, FT", " R1-3  TF RIDE SOFT, MED, HARD"])
                append(rows, [t, GREEN]);
        }
        forindex (var i; me.lines) {
            if (i < size(rows)) me.lines[i].setText(rows[i][0]).setColor(rows[i][1]);
            else me.lines[i].setText("");
        }
    },
};

# ---------------------------------------------------------------- the backup engine and fuel display
var Sfd = {
    new: func {
        var m = {parents: [Sfd]};
        m.c = canvas.new({"name": "SFD", "size": [512, 384], "view": [512, 384], "mipmapping": 1});
        m.c.addPlacement({"node": "SFD.screen"});
        m.c.setColorBackground(0, 0, 0, 1);
        var g = m.c.createGroup();
        txt(g, "1", 190, 30, 26, GREY); txt(g, "2", 360, 30, 26, GREY);
        m.rows = [];
        foreach (var r; ["TQ %", "NG %", "FF PPH"]) {
            var y = 80 + 60 * size(m.rows);
            txt(g, r, 30, y, 24, GREY, "left-center");
            append(m.rows, [txt(g, "0", 190, y, 34, GREEN, "center-center", MONO), txt(g, "0", 360, y, 34, GREEN, "center-center", MONO)]);
        }
        txt(g, "NR %", 30, 260, 24, GREY, "left-center");
        m.nr = txt(g, "0", 275, 260, 34, GREEN, "center-center", MONO);
        txt(g, "FUEL", 30, 330, 24, GREY, "left-center");
        m.fuel = txt(g, "0", 275, 330, 34, GREEN, "center-center", MONO);
        return m;
    },
    update: func {
        for (var e = 0; e < 2; e += 1) {
            me.rows[0][e].setText(sprintf("%d", num("/engines/engine[" ~ e ~ "]/torque-pct")));
            me.rows[1][e].setText(sprintf("%d", num("/engines/engine[" ~ e ~ "]/n1-pct")));
            me.rows[2][e].setText(sprintf("%d", num("/engines/engine[" ~ e ~ "]/fuel-flow-pph")));
        }
        me.nr.setText(sprintf("%d", num(JSB ~ "drive/nr-pct")));
        me.fuel.setText(sprintf("%d", num("/consumables/fuel/total-fuel-lbs")));
    },
};

# ---------------------------------------------------------------- the electronic standby flight instrument
var Esfi = {
    new: func {
        var m = {parents: [Esfi]};
        m.c = canvas.new({"name": "ESFI", "size": [512, 470], "view": [512, 470], "mipmapping": 1});
        m.c.addPlacement({"node": "ESFI.screen"});
        m.c.setColorBackground(0, 0, 0, 1);
        var g = m.c.createGroup();
        var cx = 256; var cy = 235;
        var win = clip(g.createChild("group"), 110, 60, 402, 410);
        m.att = win.createChild("group").setCenter(cx, cy);
        m.pitch = m.att.createChild("group");
        box(m.pitch, cx - 600, cy - 900, 1200, 900, SKY, SKY, 1);
        box(m.pitch, cx - 600, cy, 1200, 900, GROUND, GROUND, 1);
        for (var p = -30; p <= 30; p += 10) {
            if (p == 0) continue;
            line(m.pitch, [[cx - 30, cy - p * 5], [cx + 30, cy - p * 5]], WHITE, 2);
        }
        line(m.pitch, [[cx - 600, cy], [cx + 600, cy]], WHITE, 2);
        line(g, [[cx - 70, cy], [cx - 26, cy], [cx - 14, cy + 12], [cx, cy], [cx + 14, cy + 12], [cx + 26, cy], [cx + 70, cy]], AMBER, 5);
        box(g, 10, cy - 24, 96, 48, WHITE, [0, 0, 0], 2);
        m.ias = txt(g, "0", 98, cy, 34, WHITE, "right-center", MONO);
        box(g, 404, cy - 24, 104, 48, WHITE, [0, 0, 0], 2);
        m.alt = txt(g, "0", 500, cy, 30, WHITE, "right-center", MONO);
        m.hdg = txt(g, "000", cx, 30, 30, WHITE, "center-center", MONO);
        m.baro = txt(g, "29.92", 456, 440, 24, CYAN, "center-center", MONO);
        m.k = 5;
        return m;
    },
    update: func {
        me.att.setRotation(-num("/orientation/roll-deg") * D2R);
        me.pitch.setTranslation(0, num("/orientation/pitch-deg") * me.k);
        me.ias.setText(sprintf("%d", math.max(0, num("/instrumentation/airspeed-indicator/indicated-speed-kt"))));
        me.alt.setText(sprintf("%d", math.round(num("/instrumentation/altimeter/indicated-altitude-ft") / 10) * 10));
        var hdg = num("/orientation/heading-magnetic-deg", num("/orientation/heading-deg"));
        me.hdg.setText(sprintf("%03d", math.mod(math.round(hdg) + 359, 360) + 1));
        me.baro.setText(sprintf("%.2f", num("/instrumentation/altimeter/setting-inhg", 29.92)));
    },
};

# ---------------------------------------------------------------- the automatic flight control panel and the radio panels
var AFCS = "/instrumentation/mv22b/afcs/";

var HOLD_STATE = ["", " DECEL", " HOLD", " TRC"];
var afcs_modes = func {
    var s = [];
    if (num(AFCS ~ "cpld")) append(s, "CPLD");
    # (the hover hold flies all of it while it is on: the other modes, still selected, not shown; listed, they ran off
    # both ends of the line)
    var hover = num(JSB ~ "hh/on");
    # the lateral mode: the approach line once captured (the route or the heading until then), the vertical: its path
    if (!hover and !num(JSB ~ "ap/line")) {
        if (num(JSB ~ "ap/nav")) append(s, "WYPT " ~ (getprop("/instrumentation/gps/wp/wp[1]/ID") or ""));
        elsif (num(AFCS ~ "wypt")) append(s, "WYPT (NO ROUTE)");
        if (num(AFCS ~ "hdg-hold") and !num(JSB ~ "ap/nav"))
            append(s, sprintf("HDG %03d", math.mod(math.round(num("/autopilot/settings/heading-bug-deg")) + 359, 360) + 1));
    }
    autopilot.annunciate(s);
    if (!hover) {
        autopilot.tf_annunciate(s);
        if (num(AFCS ~ "alt-hold") and !num(JSB ~ "ap/gp") and !num(JSB ~ "ap/tf-on"))
            append(s, sprintf("ALT %d", math.round(num("/autopilot/settings/target-altitude-ft"))));
        if (num(JSB ~ "ap/app")) append(s, sprintf("SPD %d", math.round(num(JSB ~ "ap/v-sel-kt"))));
        elsif (num(AFCS ~ "spd-hold")) append(s, sprintf("SPD %d", math.round(num("/autopilot/settings/target-speed-kt"))));
        if (num(AFCS ~ "auto-nac")) append(s, num(JSB ~ "ap/auto-nac-on") ? "AUTO NAC" : "AUTO NAC (SPD)");
    }
    # the hover hold (HVR CPLD) and its height (HVR ALT HOLD, or with HVR CPLD)
    if (num(JSB ~ "hh/on"))
        append(s, "HVR CPLD" ~ HOLD_STATE[num(JSB ~ "hh/state")] ~ (num(AFCS ~ "rem-hvr") ? " REM" : ""));
    if (num(JSB ~ "hh/alt-on"))
        append(s, (num(JSB ~ "hh/alt-radar") ? "RAD ALT " : "ALT ") ~ math.round(num(JSB ~ "hh/h-ref-ft")));
    var out = "";
    foreach (var t; s) out ~= (out == "" ? "" : "  ") ~ t;
    return out;
}

var Readouts = {
    new: func {
        var m = {parents: [Readouts]};
        m.c = canvas.new({"name": "AFCS", "size": [1024, 64], "view": [1024, 64], "mipmapping": 1});
        m.c.addPlacement({"node": "AFCS.readouts"});
        m.c.setColorBackground(0, 0, 0, 1);
        var g = m.c.createGroup();
        # the knobs' places along the strip (fdm/cockpit/panel.py: -0.075 .. 0.150 m)
        m.t = [];
        foreach (var y; [-0.062, -0.026, 0.008, 0.048, 0.088])
            append(m.t, txt(g, "", (y + 0.075) / 0.225 * 1024, 34, 44, GREEN, "center-center", MONO));
        return m;
    },
    update: func {
        me.t[0].setText(sprintf("%03d", num("/instrumentation/nav[0]/radials/selected-deg")));
        me.t[1].setText(sprintf("%03d", num("/autopilot/settings/heading-bug-deg")));
        me.t[2].setText(sprintf("%3d", num("/autopilot/settings/target-speed-kt")));
        me.t[3].setText(sprintf("%5d", num("/autopilot/settings/target-altitude-ft")));
        me.t[4].setText(sprintf("%3d", num("/autopilot/settings/target-agl-ft")));
    },
};

var Radio = {
    new: func(n) {
        var m = {parents: [Radio], n: n};
        m.c = canvas.new({"name": "RADIO" ~ (n + 1), "size": [512, 92], "view": [512, 92], "mipmapping": 1});
        m.c.addPlacement({"node": "RADIO" ~ (n + 1) ~ ".screen"});
        m.c.setColorBackground(0, 0, 0, 1);
        var g = m.c.createGroup();
        m.t = txt(g, "", 256, 48, 52, GREEN, "center-center", MONO);
        return m;
    },
    update: func {
        me.t.setText(sprintf("COM%d %7.3f", me.n + 1, num("/instrumentation/comm[" ~ me.n ~ "]/frequencies/selected-mhz")));
    },
};

# ---------------------------------------------------------------- the master alert capsules
# At each end of the glareshield (fdm/cockpit/panel.py), MASTER WARN over MASTER CAUT, each half lit while
# Nasal/alerts.nas has a new alert of its level, at the photocell's level (MIL-STD-411F: dimmed at night); dark
# otherwise, the legends not to be seen; pressed, everything standing is acknowledged
var UNLIT = [0.06, 0.06, 0.06];
var Master = {
    new: func(name) {
        var m = {parents: [Master], name: string.lc(name)};
        m.c = canvas.new({"name": name, "size": [128, 128], "view": [128, 128], "mipmapping": 1});
        m.c.addPlacement({"node": name ~ ".screen"});
        m.c.setColorBackground(0, 0, 0, 1);
        var g = m.c.createGroup();
        line(g, [[8, 64], [120, 64]], UNLIT, 3);
        m.warn = [txt(g, "MASTER", 64, 24, 26, UNLIT), txt(g, "WARN", 64, 48, 30, UNLIT)];
        m.caut = [txt(g, "MASTER", 64, 84, 26, UNLIT), txt(g, "CAUT", 64, 108, 30, UNLIT)];
        return m;
    },
    update: func {
        var w = num("/instrumentation/mv22b/alerts/master-warning"); var c = num("/instrumentation/mv22b/alerts/master-caution");
        foreach (var t; me.warn) t.setColor(w ? RED : UNLIT);
        foreach (var t; me.caut) t.setColor(c ? AMBER : UNLIT);
        setprop(BASE ~ me.name ~ "/luminance", (w or c) and powered() ? between(level("night"), level("day"), daylight()) : 0);
    },
};

# ---------------------------------------------------------------- the units, their keys, brightness and updates
var units = {};
var cdu = nil;
var BASE = "/instrumentation/mv22b/displays/";

# the flight manual's pictures (manual/capture.py: the cockpit's camera square to each screen, the picture warped
# flat from where a white frame round the canvas's edge shows its corners): BASE/show, a unit and the format or page
# to show on it ("MFD4 FUEL", "CDU STAT"), or FRAME / NOFRAME to draw the frame or take it away
var canvases = {};
var frames = {};
setlistener(BASE ~ "show", func(n) {
    var w = split(" ", n.getValue() or "");
    if (size(w) < 2 or !contains(canvases, w[0])) return;
    var u = w[0];
    if (w[1] == "FRAME" or w[1] == "NOFRAME") {
        if (contains(frames, u)) { frames[u].del(); delete(frames, u); }
        if (w[1] == "NOFRAME") return;
        var c = canvases[u];
        frames[u] = c.createGroup();
        frames[u].createChild("path").rect(0, 0, c.get("size[0]"), c.get("size[1]")).setColor(1, 1, 1).setStrokeLineWidth(10);
    } elsif (contains(units, u)) units[u].show(w[1]);
    elsif (u == "CDU") cdu.page = w[1];
}, 0, 0);

var powered = func num("/controls/electric/battery-switch") or num(JSB ~ "eng[0]/running") or num(JSB ~ "eng[1]/running");

# the photocell: 0 at night (the sun 6 deg below the horizon) to 1 in daylight (10 deg above)
var daylight = func { math.max(0, math.min(1, (96 - num("/sim/time/sun-angle-rad", 0) / D2R) / 16)); };

var level = func(k) { num(BASE ~ "levels/" ~ k, LEVELS[k]); };
var between = func(lo, hi, f) { lo * math.pow(hi / lo, f); };

var AUTO = {esfi: 1, afcs: 1, radio1: 1, radio2: 1};      # no mode knob: the photocell over night and day

var luminance = func(u, f) {
    var mode = contains(AUTO, u) ? -1 : int(num(BASE ~ u ~ "/mode", 2));
    var l = !powered() or mode == 0 ? 0
          : mode == 1 ? level("night")
          : mode == 2 ? between(level("dusk"), level("day"), f)
          : between(level("night"), level("day"), f);
    setprop(BASE ~ u ~ "/luminance", l * num(BASE ~ u ~ "/brightness", 1));
}

var key = func(unit, k) {
    if (unit == "MASTER") return alerts.acknowledge();
    if (unit == "CDU") {
        if (k == "BRT+" or k == "BRT-") brightness("cdu", k);
        elsif (cdu != nil) cdu.key(k);
        return;
    }
    if (substr(unit, 0, 2) == "KP") {
        if (cdu != nil) cdu.type(k);
        return;
    }
    if (unit == "AFCS") {
        if (k == "HVRCPLD") return setprop(JSB ~ "hh/engage-cmd", !num(JSB ~ "hh/on"));
        if (k == "VORILS") return autopilot.vorils();
        if (k == "APPR") return autopilot.appr();
        if (k == "GA") return autopilot.ga();
        if (k == "TF") return autopilot.tf_key();
        if (k == "ENGOUT") return setprop(AFCS ~ "eng-out", !num(AFCS ~ "eng-out"));
        if (k == "HVRHOLD" and !num(AFCS ~ "hvr-hold"))
            setprop("/autopilot/settings/target-agl-ft", math.max(0, math.round(num("/position/gear-agl-m") / 0.3048)));
        # the holds take the present heading, altitude or speed onto their knobs, which then change them
        if (k == "HDGHOLD" and !num(AFCS ~ "hdg-hold"))
            setprop("/autopilot/settings/heading-bug-deg", math.round(num("/orientation/heading-magnetic-deg")));
        if (k == "ALTHOLD" and !num(AFCS ~ "alt-hold"))
            setprop("/autopilot/settings/target-altitude-ft", math.round(num("/instrumentation/altimeter/indicated-altitude-ft") / 10) * 10);
        if (k == "SPDHOLD" and !num(AFCS ~ "spd-hold"))
            setprop("/autopilot/settings/target-speed-kt", math.round(num("/instrumentation/airspeed-indicator/indicated-speed-kt")));
        var t = {HDGHOLD: "hdg-hold", SPDHOLD: "spd-hold", ALTHOLD: "alt-hold", HVRHOLD: "hvr-hold", AUTONAC: "auto-nac",
                 REMHVR: "rem-hvr", WYPT: "wypt", "CPLD-L": "cpld", "CPLD-R": "cpld"};
        if (contains(t, k)) setprop(AFCS ~ t[k], !num(AFCS ~ t[k]));
        return;
    }
    if (k == "BRT+" or k == "BRT-") return brightness(string.lc(unit), k);
    if (contains(units, unit)) units[unit].key(k);
}

var brightness = func(u, k) {
    var b = num(BASE ~ u ~ "/brightness", 1) + (k == "BRT+" ? 0.1 : -0.1);
    setprop(BASE ~ u ~ "/brightness", math.max(0.1, math.min(1, b)));
}

var ALL = ["mfd1", "mfd2", "mfd3", "mfd4", "cdu", "sfd", "esfi", "afcs", "radio1", "radio2"];
var fast = [];
var slow = [];
var PROFILE = "/instrumentation/mv22b/displays/profile/";

var init = func {
    foreach (var k; keys(LEVELS))
        if (getprop(BASE ~ "levels/" ~ k) == nil) setprop(BASE ~ "levels/" ~ k, LEVELS[k]);
    foreach (var u; ALL)
        if (getprop(BASE ~ u ~ "/brightness") == nil) setprop(BASE ~ u ~ "/brightness", 1);
    # the knobs NIGHT with the sun down, DAY otherwise, once the start's time of day has set the sun
    settimer(func {
        var night = daylight() < 0.25;
        foreach (var u; ALL)
            if (!contains(AUTO, u) and getprop(BASE ~ u ~ "/mode") == nil) setprop(BASE ~ u ~ "/mode", night ? 1 : 2);
    }, 2);
    io.read_properties(getprop("/sim/aircraft-dir") ~ "/Systems/mv22b-displays.xml", DATA);
    corridor.load();
    foreach (var s; [["MFD1", "FLT"], ["MFD2", "ENG"], ["MFD3", "NAV"], ["MFD4", "FLT"]])
        units[s[0]] = Mfd.new(s[0], s[1]);
    cdu = Cdu.new();
    var sfd = Sfd.new(); var esfi = Esfi.new(); var ro = Readouts.new();
    canvases = {MFD1: units["MFD1"].c, MFD2: units["MFD2"].c, MFD3: units["MFD3"].c, MFD4: units["MFD4"].c, CDU: cdu.c, SFD: sfd.c,
                ESFI: esfi.c};
    var masters = [Master.new("MASTER1"), Master.new("MASTER2")];
    fast = [units["MFD1"], units["MFD2"], units["MFD3"], units["MFD4"], esfi, cdu] ~ masters;
    slow = [sfd, ro, Radio.new(0), Radio.new(1)];
    foreach (var u; [esfi, cdu] ~ masters ~ slow) cache_writes(u);
    # PROFILE/enabled: each unit's update time, ms (a 2 s average), in PROFILE/<n> (fast: 0-7, slow: 8-11)
    var prof = props.globals.getNode(PROFILE, 1);
    var timed = func(us, i0) {
        if (!prof.getValue("enabled")) { foreach (var u; us) u.update(); return; }
        forindex (var i; us) {
            var t = systime(); us[i].update();
            var n = prof.getNode("unit[" ~ (i0 + i) ~ "]", 1);
            n.setDoubleValue(lowpass(n.getValue() or 0, (systime() - t) * 1000, 0.05, 2));
        }
    }
    var tf = maketimer(0.05, func { timed(fast, 0); });
    var ts = maketimer(0.25, func {
        timed(slow, 8);
        var f = daylight();
        foreach (var u; ALL) luminance(u, f);
    });
    tf.start(); ts.start();
}

var listener = setlistener("/sim/signals/fdm-initialized", func {
    removelistener(listener);
    init();
});
