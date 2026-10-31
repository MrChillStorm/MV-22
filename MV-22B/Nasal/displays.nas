# MV-22B displays: the Canvas pages of the four multifunction displays (MFD 1-4 from the left), the control display
# unit (CDU), the backup engine and fuel display (SFD), the electronic standby flight instrument (ESFI), the automatic
# flight control panel's readouts and the glareshield's radio panels, on the screens of the panel's 3-D model
# (Models/Interior/Panel/MV22B, written by fdm/cockpit/panel.py), whose keys call key(unit, key).
#
# The V-22's display formats are not public in detail; these follow its published layout (four 6 in square
# multifunction displays with flight, navigation and system formats, the CDU's EICAS and status pages, a standby
# flight display) with the flight model's own quantities: nacelle angle and the conversion protection's limits,
# torque, rotor speed, the drive's torque limit, the five tanks.
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

# a property's value, d when it has none (Nasal's "or" would take a zero for none too)
var num = func(p, d = 0) {
    var v = getprop(p);
    return v == nil ? d : v;
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
# apart along the top, 21.5 mm down the sides, 3360 px per m: fdm/cockpit/panel.py)
var labels = func(g, top, left, right) {
    for (var i = 0; i < size(top); i += 1)
        if (top[i] != "") txt(g, top[i], 256 + (i - 2) * 87.4, 14, 18, CYAN);
    for (var i = 0; i < size(left); i += 1)
        if (left[i] != "") txt(g, left[i], 6, 39 + 72.2 * i, 17, CYAN, "left-center");
    for (var i = 0; i < size(right); i += 1)
        if (right[i] != "") txt(g, right[i], 506, 39 + 72.2 * i, 17, CYAN, "right-center");
}

# ---------------------------------------------------------------- the flight format
var Flight = {
    new: func(g) {
        var m = {parents: [Flight]};
        m.g = g;
        var cx = 256; var cy = 230;
        m.cx = cx; m.cy = cy; m.k = 5.0;                     # px per deg of pitch
        # the attitude
        var win = clip(g.createChild("group"), 118, 92, 394, 368);
        m.att = win.createChild("group").setCenter(cx, cy);
        m.pitch = m.att.createChild("group");
        box(m.pitch, cx - 600, cy - 900, 1200, 900, SKY, SKY, 1);
        box(m.pitch, cx - 600, cy, 1200, 900, GROUND, GROUND, 1);
        line(m.pitch, [[cx - 600, cy], [cx + 600, cy]], WHITE, 2);
        for (var p = -60; p <= 60; p += 5) {
            if (p == 0) continue;
            var y = cy - p * m.k; var w = math.mod(p, 10) == 0 ? 46 : 22;
            line(m.pitch, [[cx - w, y], [cx + w, y]], WHITE, 2);
            if (math.mod(p, 10) == 0) {
                txt(m.pitch, sprintf("%d", math.abs(p)), cx - w - 16, y, 16);
                txt(m.pitch, sprintf("%d", math.abs(p)), cx + w + 16, y, 16);
            }
        }
        # the roll scale and its pointer
        foreach (var a; [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]) {
            var r0 = 128; var r1 = math.mod(a, 30) == 0 ? 142 : 136;
            var s = math.sin(a * D2R); var c = math.cos(a * D2R);
            line(g, [[cx + r0 * s, cy - r0 * c], [cx + r1 * s, cy - r1 * c]], WHITE, 2);
        }
        m.roll = g.createChild("group").setCenter(cx, cy);
        line(m.roll, [[cx, cy - 126], [cx - 8, cy - 112], [cx + 8, cy - 112], [cx, cy - 126]], WHITE, 2);
        # the aircraft symbol
        line(g, [[cx - 60, cy], [cx - 22, cy], [cx - 12, cy + 10], [cx, cy], [cx + 12, cy + 10], [cx + 22, cy], [cx + 60, cy]], AMBER, 4);
        # the airspeed tape (KCAS) and its readout
        var sw = clip(g.createChild("group"), 30, 92, 110, 368);
        box(g, 30, 92, 80, 276, GREY, nil, 1);
        m.spd = tape(sw, 30, 80, 0, 400, 10, 20, 3.0, cy, -1, "%d");
        box(g, 34, cy - 16, 72, 32, WHITE, [0, 0, 0], 2);
        m.spdtxt = txt(g, "0", 100, cy, 24, WHITE, "right-center", MONO);
        m.gs = txt(g, "GS 0", 70, 384, 18, GREEN);
        # the altitude tape and its readout
        var aw = clip(g.createChild("group"), 402, 92, 482, 368);
        box(g, 402, 92, 80, 276, GREY, nil, 1);
        m.alt = tape(aw, 402, 80, -1000, 30000, 100, 500, 0.30, cy, 1, "%d");
        box(g, 404, cy - 16, 92, 32, WHITE, [0, 0, 0], 2);
        m.alttxt = txt(g, "0", 492, cy, 22, WHITE, "right-center", MONO);
        m.vs = txt(g, "VS 0", 442, 384, 18, GREEN);
        m.ralt = txt(g, "", cx, cy + 112, 22, GREEN, "center-center", MONO);
        # the heading tape
        var hw = clip(g.createChild("group"), 118, 38, 394, 74);
        m.hdg = hw.createChild("group");
        for (var h = -40; h <= 400; h += 5) {
            var x = cx + h * 4;
            line(m.hdg, [[x, 74], [x, math.mod(h, 10) == 0 ? 62 : 67]], WHITE, 2);
            if (math.mod(h, 30) == 0) {
                var v = math.mod(h + 360, 360) / 10;
                var s = v == 0 ? "N" : v == 9 ? "E" : v == 18 ? "S" : v == 27 ? "W" : sprintf("%d", v);
                txt(m.hdg, s, x, 50, 18);
            }
        }
        box(g, cx - 26, 30, 52, 26, WHITE, [0, 0, 0], 2);
        m.hdgtxt = txt(g, "000", cx, 43, 20, WHITE, "center-center", MONO);
        # the AFCS modes
        m.modes = txt(g, "", cx, 84, 16, GREEN);
        # the nacelle angle against the conversion protection's limits (an arc, 0 at the right, 90 up)
        var nx = 70; var ny = 486; var nr = 46;
        m.nx = nx; m.ny = ny; m.nr = nr;
        for (var a = 0; a <= 90; a += 15)
            line(g, [[nx + nr * math.cos(a * D2R), ny - nr * math.sin(a * D2R)], [nx + (nr - 8) * math.cos(a * D2R), ny - (nr - 8) * math.sin(a * D2R)]], WHITE, 2);
        m.nac = line(g, [[nx, ny], [nx + nr - 4, ny]], GREEN, 3).setCenter(nx, ny);
        m.cpsmin = line(g, [[nx + nr, ny], [nx + nr + 10, ny]], AMBER, 4).setCenter(nx, ny);
        m.cpsmax = line(g, [[nx + nr, ny], [nx + nr + 10, ny]], AMBER, 4).setCenter(nx, ny);
        m.nactxt = txt(g, "NAC 90", nx + 6, ny - 60, 20, GREEN, "left-center");
        # power and rotor speed, flaperons, gear, the augmentation's mode
        m.tq = txt(g, "TQ 0 0", 210, 418, 20, GREEN, "left-center", MONO);
        m.nr = txt(g, "NR 0", 210, 444, 20, GREEN, "left-center", MONO);
        m.flap = txt(g, "FLAP 0", 210, 470, 18, WHITE, "left-center");
        m.gear = txt(g, "GEAR DN", 330, 470, 18, WHITE, "left-center");
        m.scas = txt(g, "", 330, 444, 18, CYAN, "left-center");
        m.brake = txt(g, "", 330, 418, 18, AMBER, "left-center");
        return m;
    },
    update: func {
        var pitch = num("/orientation/pitch-deg"); var roll = num("/orientation/roll-deg");
        me.att.setRotation(-roll * D2R);
        me.pitch.setTranslation(0, pitch * me.k);
        me.roll.setRotation(-roll * D2R);
        var ias = math.max(0, num("/instrumentation/airspeed-indicator/indicated-speed-kt"));
        me.spd.setTranslation(0, ias * 3.0);
        me.spdtxt.setText(sprintf("%d", ias));
        me.gs.setText(sprintf("GS %d", num("/velocities/groundspeed-kt")));
        var alt = num("/instrumentation/altimeter/indicated-altitude-ft");
        me.alt.setTranslation(0, alt * 0.30);
        me.alttxt.setText(sprintf("%d", math.round(alt / 10) * 10));
        me.vs.setText(sprintf("VS %+d", math.round(num("/velocities/vertical-speed-fps") * 6) * 10));
        var agl = num("/position/altitude-agl-ft");
        me.ralt.setText(agl < 5000 ? sprintf("R %d", math.round(agl)) : "");
        var hdg = num("/orientation/heading-magnetic-deg", num("/orientation/heading-deg"));
        me.hdg.setTranslation(-hdg * 4, 0);
        me.hdgtxt.setText(sprintf("%03d", math.mod(math.round(hdg) + 359, 360) + 1));
        me.modes.setText(afcs_modes());
        var nac = num(JSB ~ "ctl/nacelle-deg", 90);
        me.nac.setRotation(-nac * D2R);
        me.cpsmin.setRotation(-num(JSB ~ "ctl/cps-min-deg", 0) * D2R);
        me.cpsmax.setRotation(-math.min(97.5, num(JSB ~ "ctl/cps-max-deg", 97.5)) * D2R);
        me.nactxt.setText(sprintf("NAC %d", math.round(nac)));
        me.tq.setText(sprintf("TQ %3d %3d", num("/engines/engine[0]/torque-pct"), num("/engines/engine[1]/torque-pct")));
        me.nr.setText(sprintf("NR %3d", num(JSB ~ "drive/nr-pct")));
        me.flap.setText(sprintf("FLAP %d", num(JSB ~ "ctl/flaperon-deg")));
        var gear = num(JSB ~ "gear/gear-pos-norm", 1);
        me.gear.setText(gear > 0.99 ? "GEAR DN" : gear < 0.01 ? "GEAR UP" : "GEAR TRANS");
        me.scas.setText(num(JSB ~ "scas/attitude-command") > 0.5 ? "ATT CMD" : "RATE CMD");
        me.brake.setText(num("/controls/rotor/brake") > 0.05 ? "ROTOR BRAKE" : "");
    },
};

# ---------------------------------------------------------------- the navigation format (a horizontal situation display)
var Nav = {
    new: func(g) {
        var m = {parents: [Nav]};
        var cx = 256; var cy = 290; var r = 170;
        m.cx = cx; m.cy = cy;
        m.rose = g.createChild("group").setCenter(cx, cy);
        for (var a = 0; a < 360; a += 5) {
            var s = math.sin(a * D2R); var c = math.cos(a * D2R);
            var r1 = math.mod(a, 10) == 0 ? r - 14 : r - 8;
            line(m.rose, [[cx + r * s, cy - r * c], [cx + r1 * s, cy - r1 * c]], WHITE, 2);
            if (math.mod(a, 30) == 0) {
                var v = a / 10;
                var t = txt(m.rose, v == 0 ? "N" : v == 9 ? "E" : v == 18 ? "S" : v == 27 ? "W" : sprintf("%d", v), cx + (r - 30) * s, cy - (r - 30) * c, 20);
                t.setRotation(a * D2R);
            }
        }
        # the course pointer and deviation bar, the heading bug, the bearing pointer
        m.crs = m.rose.createChild("group").setCenter(cx, cy);
        line(m.crs, [[cx, cy - r + 16], [cx, cy - 60]], MAGENTA, 3);
        line(m.crs, [[cx - 9, cy - r + 30], [cx, cy - r + 16], [cx + 9, cy - r + 30]], MAGENTA, 3);
        line(m.crs, [[cx, cy + 60], [cx, cy + r - 16]], MAGENTA, 3);
        foreach (var d; [-2, -1, 1, 2]) m.crs.createChild("path").circle(4, cx + d * 30, cy).setColor(WHITE).setStrokeLineWidth(2);
        m.dev = line(m.crs, [[cx, cy - 56], [cx, cy + 56]], MAGENTA, 3);
        m.bug = line(m.rose, [[cx - 10, cy - r - 2], [cx - 10, cy - r - 12], [cx + 10, cy - r - 12], [cx + 10, cy - r - 2]], CYAN, 3).setCenter(cx, cy);
        m.brg = line(m.rose, [[cx, cy - r + 20], [cx, cy + r - 20]], GREEN, 2).setCenter(cx, cy);
        line(g, [[cx, cy - r - 16], [cx, cy - r + 4]], WHITE, 3);              # the lubber line
        line(g, [[cx - 16, cy], [cx + 16, cy]], AMBER, 3);
        line(g, [[cx, cy - 18], [cx, cy + 22]], AMBER, 3);
        # the data
        m.hdgtxt = txt(g, "HDG 000", 70, 50, 20, WHITE, "left-center", MONO);
        m.crstxt = txt(g, "CRS 000", 70, 76, 18, MAGENTA, "left-center", MONO);
        m.bugtxt = txt(g, "SEL 000", 70, 100, 18, CYAN, "left-center", MONO);
        m.wpt = txt(g, "", 442, 50, 20, GREEN, "right-center", MONO);
        m.dist = txt(g, "", 442, 76, 18, GREEN, "right-center", MONO);
        m.nav = txt(g, "", 442, 100, 18, MAGENTA, "right-center", MONO);
        m.gs = txt(g, "GS 0", 70, 486, 18, WHITE, "left-center", MONO);
        m.tas = txt(g, "TAS 0", 70, 464, 18, WHITE, "left-center", MONO);
        m.wind = txt(g, "", 442, 486, 18, WHITE, "right-center", MONO);
        m.windarrow = line(g, [[440, 430], [440, 460], [434, 450], [440, 460], [446, 450]], WHITE, 2).setCenter(440, 445);
        m.r = r;
        return m;
    },
    update: func {
        var hdg = num("/orientation/heading-magnetic-deg", num("/orientation/heading-deg"));
        me.rose.setRotation(-hdg * D2R);
        var crs = num("/instrumentation/nav[0]/radials/selected-deg");
        me.crs.setRotation(crs * D2R);
        var dev = num("/instrumentation/nav[0]/heading-needle-deflection-norm");
        me.dev.setTranslation(dev * 60, 0);
        var bug = num("/autopilot/settings/heading-bug-deg");
        me.bug.setRotation(bug * D2R);
        var brg = getprop("/autopilot/route-manager/wp/bearing-deg");
        if (brg != nil and num("/autopilot/route-manager/active") > 0) {
            me.brg.show(); me.brg.setRotation((brg - num("/environment/magnetic-variation-deg")) * D2R);
            me.wpt.setText(getprop("/autopilot/route-manager/wp/id") or "");
            me.dist.setText(sprintf("%.1f NM", num("/autopilot/route-manager/wp/dist")));
        } else {
            me.brg.hide(); me.wpt.setText(""); me.dist.setText("");
        }
        me.nav.setText(num("/instrumentation/nav[0]/in-range") ? (getprop("/instrumentation/nav[0]/nav-id") or "") : "");
        me.hdgtxt.setText(sprintf("HDG %03d", math.mod(math.round(hdg) + 359, 360) + 1));
        me.crstxt.setText(sprintf("CRS %03d", crs));
        me.bugtxt.setText(sprintf("SEL %03d", bug));
        me.gs.setText(sprintf("GS %d", num("/velocities/groundspeed-kt")));
        me.tas.setText(sprintf("TAS %d", num("/velocities/airspeed-kt")));
        var wd = num("/environment/wind-from-heading-deg"); var ws = num("/environment/wind-speed-kt");
        me.wind.setText(sprintf("%03d/%d", wd, ws));
        me.windarrow.setRotation((wd + 180 - num("/orientation/heading-deg")) * D2R);
    },
};

# ---------------------------------------------------------------- the engine and drive format
var bar = func(g, x, y0, h, w, color) {
    return g.createChild("path").rect(x, y0 - h, w, h).setColorFill(color).setColor(color);
}

var Engines = {
    new: func(g) {
        var m = {parents: [Engines]};
        txt(g, "ENGINES AND DRIVE", 256, 52, 22);
        m.cols = [];
        foreach (var e; [0, 1]) {
            var x = e == 0 ? 120 : 392;
            var c = {};
            txt(g, sprintf("ENG %d", e + 1), x, 86, 20, WHITE);
            box(g, x - 22, 112, 44, 220, GREY, nil, 2);                     # torque, 0..120 %
            line(g, [[x - 28, 332 - 220 * 100 / 120], [x + 28, 332 - 220 * 100 / 120]], AMBER, 2);
            c.bar = bar(g, x - 18, 330, 0, 36, GREEN);
            c.tq = txt(g, "0", x, 352, 22, GREEN, "center-center", MONO);
            txt(g, "TQ %", x, 374, 16, GREY);
            c.ng = txt(g, "NG 0", x, 404, 18, WHITE, "center-center", MONO);
            c.shp = txt(g, "SHP 0", x, 428, 18, WHITE, "center-center", MONO);
            c.ff = txt(g, "FF 0", x, 452, 18, WHITE, "center-center", MONO);
            c.state = txt(g, "", x, 476, 18, AMBER, "center-center");
            c.x = x;
            append(m.cols, c);
        }
        txt(g, "NR", 256, 120, 20, GREY);
        m.nr = txt(g, "0", 256, 150, 34, GREEN, "center-center", MONO);
        txt(g, "% REF", 256, 178, 16, GREY);
        m.ref = txt(g, "0", 256, 198, 18, CYAN, "center-center", MONO);
        txt(g, "DRIVE", 256, 240, 18, GREY);
        m.req = txt(g, "0", 256, 264, 18, WHITE, "center-center", MONO);
        txt(g, "LIMIT", 256, 292, 16, GREY);
        m.lim = txt(g, "0", 256, 314, 18, WHITE, "center-center", MONO);
        m.brake = txt(g, "", 256, 352, 18, AMBER);
        return m;
    },
    update: func {
        forindex (var e; me.cols) {
            var c = me.cols[e];
            var tq = num("/engines/engine[" ~ e ~ "]/torque-pct");
            var h = math.min(1.2, math.max(0, tq / 100)) / 1.2 * 216;
            c.bar.reset().rect(c.x - 18, 330 - h, 36, h);
            c.tq.setText(sprintf("%d", tq)).setColor(tq > 100 ? AMBER : GREEN);
            c.ng.setText(sprintf("NG %d", num("/engines/engine[" ~ e ~ "]/n1-pct")));
            c.shp.setText(sprintf("SHP %d", num(JSB ~ "eng[" ~ e ~ "]/power-hp")));
            c.ff.setText(sprintf("FF %d", num("/engines/engine[" ~ e ~ "]/fuel-flow-pph")));
            var run = num(JSB ~ "eng[" ~ e ~ "]/running"); var sp = num(JSB ~ "eng[" ~ e ~ "]/spool");
            c.state.setText(run and sp < 0.99 ? "START" : !run and sp > 0.01 ? "SPOOL DN" : !run ? "OFF" : "");
        }
        me.nr.setText(sprintf("%d", num(JSB ~ "drive/nr-pct")));
        me.ref.setText(sprintf("%d", num(JSB ~ "gov/nr-ref-pct")));
        me.req.setText(sprintf("%d HP", num(JSB ~ "drive/power-required-hp")));
        me.lim.setText(sprintf("%d HP", 2 * num(JSB ~ "eng/xmsn-limit-hp")));
        me.brake.setText(num("/controls/rotor/brake") > 0.05 ? "ROTOR BRAKE ON" : "");
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

var Mfd = {
    new: func(name, page) {
        var m = {parents: [Mfd], name: name};
        m.c = canvas.new({"name": name, "size": [512, 512], "view": [512, 512], "mipmapping": 1});
        m.c.addPlacement({"node": name ~ ".screen"});
        m.c.setColorBackground(0, 0, 0, 1);
        m.pages = {};
        m.groups = {};
        foreach (var p; MFD_PAGES) {
            var g = m.c.createGroup();
            m.groups[p] = g;
            m.pages[p] = (p == "FLT" ? Flight : p == "NAV" ? Nav : p == "ENG" ? Engines : Fuel).new(g);
            labels(g, MFD_PAGES, [], []);
            g.hide();
        }
        m.page = "";
        m.show(page);
        return m;
    },
    show: func(p) {
        if (me.page != "") me.groups[me.page].hide();
        me.page = p;
        me.groups[p].show();
        me.pages[p].update();
    },
    key: func(k) {
        if (k[0] == `T`) {
            var i = num_key(k) - 1;
            if (i < size(MFD_PAGES)) me.show(MFD_PAGES[i]);
        }
    },
    update: func me.pages[me.page].update(),
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
var cautions = func {
    var out = [];
    var jsb = func(p) num(JSB ~ p);
    for (var e = 0; e < 2; e += 1) {
        var other = 1 - e;
        if (!jsb("eng[" ~ e ~ "]/running") and jsb("eng[" ~ other ~ "]/running")) append(out, [sprintf("ENG %d OUT", e + 1), RED]);
    }
    var total = num("/consumables/fuel/total-fuel-lbs");
    if (total < 1000) append(out, ["FUEL LOW", AMBER]);
    if (jsb("drive/nr-pct") > 5 and math.abs(jsb("drive/nr-pct") - jsb("gov/nr-ref-pct")) > 3) append(out, ["NR", AMBER]);
    if (num("/engines/engine[0]/torque-pct") > 100 or num("/engines/engine[1]/torque-pct") > 100) append(out, ["TORQUE", AMBER]);
    if (math.abs(num("/controls/flight/nacelle-cmd-deg") - jsb("ctl/nacelle-cmd-limited-deg")) > 1) append(out, ["CONV PROT", AMBER]);
    if (num("/controls/rotor/brake") > 0.05) append(out, ["ROTOR BRAKE", GREEN]);
    if (num("/position/altitude-agl-ft") > 50 and jsb("gear/gear-pos-norm") > 0.99 and num("/velocities/airspeed-kt") > 150) append(out, ["GEAR DOWN", AMBER]);
    if (!num("/controls/electric/battery-switch")) append(out, ["BATTERY OFF", GREEN]);
    return out;
}

var Cdu = {
    new: func {
        var m = {parents: [Cdu]};
        m.c = canvas.new({"name": "CDU", "size": [640, 480], "view": [640, 480], "mipmapping": 1});
        m.c.addPlacement({"node": "CDU.screen"});
        m.c.setColorBackground(0, 0, 0, 1);
        var g = m.c.createGroup();
        m.title = txt(g, "", 320, 26, 24, WHITE);
        line(g, [[20, 44], [620, 44]], GREY, 2);
        m.lines = [];
        for (var i = 0; i < 12; i += 1) append(m.lines, txt(g, "", 40, 70 + 30 * i, 22, GREEN, "left-center", MONO));
        line(g, [[20, 436], [620, 436]], GREY, 2);
        m.pad = txt(g, "", 30, 458, 24, WHITE, "left-center", MONO);
        m.scratch = "";
        m.page = "EICAS";
        return m;
    },
    key: func(k) {
        if (k == "B4") me.page = "EICAS";
        elsif (k == "B3") me.page = "STAT";
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
    # ENTER: a number on the scratchpad sets the heading bug (H123), the course (C123), the altimeter (B2992), else nothing
    enter: func {
        var s = me.scratch;
        if (size(s) > 1) {
            var v = num_key(s);
            if (s[0] == `H`) setprop("/autopilot/settings/heading-bug-deg", math.mod(v, 360));
            elsif (s[0] == `C`) setprop("/instrumentation/nav[0]/radials/selected-deg", math.mod(v, 360));
            elsif (s[0] == `B`) setprop("/instrumentation/altimeter/setting-inhg", v / 100);
        }
        me.scratch = "";
    },
    update: func {
        var rows = [];
        if (me.page == "EICAS") {
            me.title.setText("EICAS");
            foreach (var c; cautions()) append(rows, c);
            if (!size(rows)) append(rows, ["NO CAUTIONS", GREY]);
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
            foreach (var t; ["EICAS: CAUTIONS AND ADVISORIES", "STAT: WEIGHT, FUEL, CG", "", "SCRATCHPAD ENTRIES (ENTER):",
                             " H123  HEADING BUG", " C123  NAV 1 COURSE", " B2992 ALTIMETER, IN HG"])
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

var afcs_modes = func {
    var s = [];
    foreach (var m; [["hdg-hold", "HDG"], ["spd-hold", "SPD"], ["alt-hold", "ALT"], ["hvr-hold", "HVR ALT"], ["auto-nac", "AUTO NAC"],
                     ["cpld", "CPLD"]])
        if (num(AFCS ~ m[0])) append(s, m[1]);
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

# ---------------------------------------------------------------- the units, their keys, brightness and updates
var units = {};
var cdu = nil;
var BASE = "/instrumentation/mv22b/displays/";

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
        var t = {HDGHOLD: "hdg-hold", SPDHOLD: "spd-hold", ALTHOLD: "alt-hold", HVRHOLD: "hvr-hold", AUTONAC: "auto-nac",
                 "CPLD-L": "cpld", "CPLD-R": "cpld"};
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
    foreach (var s; [["MFD1", "FLT"], ["MFD2", "ENG"], ["MFD3", "NAV"], ["MFD4", "FLT"]])
        units[s[0]] = Mfd.new(s[0], s[1]);
    cdu = Cdu.new();
    var sfd = Sfd.new(); var esfi = Esfi.new(); var ro = Readouts.new();
    fast = [units["MFD1"], units["MFD2"], units["MFD3"], units["MFD4"], esfi];
    slow = [cdu, sfd, ro, Radio.new(0), Radio.new(1)];
    var tf = maketimer(0.05, func { foreach (var u; fast) u.update(); });
    var ts = maketimer(0.25, func {
        foreach (var u; slow) u.update();
        var f = daylight();
        foreach (var u; ALL) luminance(u, f);
    });
    tf.start(); ts.start();
}

var listener = setlistener("/sim/signals/fdm-initialized", func {
    removelistener(listener);
    init();
});
