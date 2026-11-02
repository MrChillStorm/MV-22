# The crew alerts (EICAS): the alert manager. The alerts, their levels and their order of priority are fdm/alerts.py's
# (Systems/mv22b-alerts.xml, which fdm/genv22.py --fg writes): the flight model sets alert/<id> while each stands, and
# alert/ground on the ground with the rotors carrying under half the weight; the autopilot's let-go modes come as its
# flags (hh/dropped, ap/dropped, ap/nac-dropped), read and cleared here.
#
# An alert is listed while it stands (Nasal/displays.nas: the engine format's window, the CDU's EICAS page) and is new
# from its onset till ACK (any MFD's or the CDU's) or either master capsule (the glareshield's) acknowledges it. A new
# warning lights MASTER WARN, sounds the warning tone and says its voice, again every CYCLE_S (a time-critical one, as
# SINK RATE, says its own voice instead, every CRITICAL_S while it stands, acknowledged or not, and shows on the flight
# format: AC 25.1322-1 6.b and appendix 2, 2.d(5)); a new caution lights
# MASTER CAUT and chimes once (MIL-STD-411F 4.1.3.8: the masters lit till reset or the condition clears, their audio
# cancelled within 10 s). A let-go autopilot mode is listed with its own tone till acknowledged, the mode engaged again
# or AURAL_S. On the ground (alert/ground) nothing is new: alerts are listed only, and one still standing when the
# aircraft lifts off is new then. The voice is FlightGear's own (sim/sound/voices/voice[4], Systems/v22-base.xml).

var JSB = "/fdm/jsbsim/";
var OUT = "/instrumentation/mv22b/alerts/";
var VOICE = "/sim/sound/voices/voice[4]/text";
var AURAL_S = 10;        # s: the masters' audio and the autopilot tone cancelled after (MIL-STD-411F 4.1.3.8)
var CYCLE_S = 4;         # s: a warning's tone and voice again
var CRITICAL_S = 2.5;    # s: a time-critical warning's voice again, while it stands
var TONE_S = 1.2;        # s: the warning tone's burst (Sounds/alerts/warning.wav, 1 s, played once) and a pause
var CHIME_S = 1.2;       # s: the caution chime (Sounds/alerts/caution.wav)
var EVENT_SHOW_S = 5;    # s: a let-go mode on the flight format
var ADVISORY_EVENT_S = 10;    # s: an advisory from the autopilot (HVR CPLD UNAVAIL) listed
var HISTORY = 20;
var RANK = {warning: 0, caution: 1, autopilot: 1, advisory: 2};

var table = [];          # {id, level, text, voice, critical, on, acked, new, since, said}
var kinds = [];          # the autopilot's events: {flag, value, text, level, mode}
var events = [];         # standing events: {text, level, mode, since, acked, quiet}
var history = [];        # cleared: [text, level, from, to], the latest first
var warn_since = -100;   # the latest new warning's onset
var said = -1;           # the warning cycle whose voice has been said
var last_event = ["", -100];
var masters = [0, 0];    # as last set: warning, caution
var timer = nil;         # kept: a timer nothing refers to is collected, and stops

# each alert's onset and clearing, the masters, the voice and the acknowledgements in FlightGear's log
var log = func(s) logprint(LOG_INFO, "MV-22B alerts: ", s);

var num = func(p) { var x = getprop(p); x == nil ? 0 : x };
var now = func num("/sim/time/elapsed-sec");

var forget = func(text, level, from) {
    history = [[text, level, from, now()]] ~ history;
    if (size(history) > HISTORY) history = subvec(history, 0, HISTORY);
}

var chime = func {
    setprop(OUT ~ "aural/caution", 1);
    settimer(func setprop(OUT ~ "aural/caution", 0), CHIME_S);
}

# ACK, either master capsule: every alert standing acknowledged, the masters out, their sounds stopped; the
# let-go modes off the list
var acknowledge = func {
    log("acknowledged");
    foreach (var a; table) if (a.on) a.acked = 1;
    foreach (var e; events) forget(e.text, e.level, e.since);
    events = [];
    warn_since = -100;
    update();
}

var update = func {
    var t = now();
    var ground = num(JSB ~ "alert/ground") > 0.5;
    var chimes = 0;
    foreach (var a; table) {
        var on = num(JSB ~ "alert/" ~ a.id) > 0.5;
        if (on and !a.on) { a.on = 1; a.acked = 0; a.since = t; a.said = -100; log(a.text ~ (ground ? " (on the ground)" : "")); }
        elsif (!on and a.on) { a.on = 0; forget(a.text, a.level, a.since); log(a.text ~ " cleared"); }
        var was = a.new;
        a.new = a.on and !a.acked and !ground and a.level != "advisory";
        if (a.new and !was) {
            if (a.level == "warning") { warn_since = t; said = -1; }
            else chimes = 1;
        }
    }
    if (chimes) chime();
    # the autopilot's let-go modes: each flag read once and cleared
    var seen = {};
    foreach (var k; kinds) {
        var f = num(JSB ~ k.flag);
        if (f == k.value) {
            append(events, {text: k.text, level: k.level, mode: k.mode, since: t, quiet: num(JSB ~ "gear/wow") > 0.5});
            last_event = [k.text, t];
            log(k.text);
        }
        if (f) seen[k.flag] = 1;
    }
    foreach (var f; keys(seen)) setprop(JSB ~ f, 0);
    var keep = [];
    foreach (var e; events) {
        if (num(JSB ~ e.mode) > 0.5 or (e.level == "advisory" and t - e.since > ADVISORY_EVENT_S)) forget(e.text, e.level, e.since);
        else append(keep, e);
    }
    events = keep;
    # the masters, and the warning's tone and voice: TONE_S of tone, then the highest new warning's voice, every
    # CYCLE_S, for AURAL_S from the latest new warning
    var mw = 0; var mc = 0; var voice = nil; var wtone = 0;
    foreach (var a; table) {
        if (!a.new) continue;
        if (a.level == "warning") {
            mw = 1;
            if (!a.critical) { wtone = 1; if (voice == nil) voice = a.voice; }
        } else mc = 1;
    }
    # a time-critical warning: its own voice (no master tone) every CRITICAL_S while it stands, acknowledged or not;
    # the first standing (the table's order) only
    foreach (var a; table) {
        if (!a.critical or !a.on) continue;
        if (t - a.said >= CRITICAL_S) { a.said = t; setprop(VOICE, a.voice); log("said \"" ~ a.voice ~ "\""); }
        break;
    }
    if (mw != masters[0] or mc != masters[1]) log(sprintf("master warning %d, caution %d", mw, mc));
    masters = [mw, mc];
    setprop(OUT ~ "master-warning", mw);
    setprop(OUT ~ "master-caution", mc);
    var w = t - warn_since;
    var tone = wtone and w < AURAL_S and math.mod(w, CYCLE_S) < TONE_S;
    if (tone and !num(OUT ~ "aural/warning")) {
        # the onset eased in over 0.25 s (MIL-STD-1472H: not at full level in its first 0.2 s)
        setprop(OUT ~ "aural/warning-gain", 0.3);
        interpolate(OUT ~ "aural/warning-gain", 1.0, 0.25);
    }
    setprop(OUT ~ "aural/warning", tone);
    var cycle = int(w / CYCLE_S);
    if (wtone and w < AURAL_S and math.mod(w, CYCLE_S) >= TONE_S and cycle != said) {
        said = cycle;
        if (voice != nil and voice != "") { setprop(VOICE, voice); log("said \"" ~ voice ~ "\""); }
    }
    if (!wtone) said = -1;
    var ap = 0;
    foreach (var e; events) if (e.level == "autopilot" and !e.quiet and t - e.since < AURAL_S) ap = 1;
    setprop(OUT ~ "aural/autopilot", ap);
}

# what the displays list: [text, level, new], warnings first, then cautions and the let-go modes, then advisories,
# the latest first within each
var standing = func {
    var out = [];
    foreach (var a; table) if (a.on) append(out, [a.text, a.level, a.new, a.since]);
    foreach (var e; events) append(out, [e.text, e.level, !e.quiet, e.since]);
    return sort(out, func(a, b) RANK[a[1]] != RANK[b[1]] ? RANK[a[1]] - RANK[b[1]] : b[3] - a[3]);
}

# the flight format's time-critical warning: the first standing and its recovery cue, ["", ""] for none
var critical_line = func {
    foreach (var a; table) if (a.critical and a.on) return [a.text, a.cue == nil ? "" : a.cue];
    return ["", ""];
}

# the flight format's line: the latest let-go mode for EVENT_SHOW_S
var flight_line = func now() - last_event[1] < EVENT_SHOW_S ? last_event[0] : "";

var init = func {
    var d = props.globals.getNode("/sim/mv22b-alerts", 1);
    io.read_properties(getprop("/sim/aircraft-dir") ~ "/Systems/mv22b-alerts.xml", d);
    foreach (var n; d.getChildren("alert"))
        append(table, {id: n.getValue("id"), level: n.getValue("level"), text: n.getValue("text"), voice: n.getValue("voice"),
                       critical: n.getValue("critical") != nil, cue: n.getValue("cue"), on: 0, acked: 0, new: 0, since: 0,
                       said: -100});
    foreach (var n; d.getChildren("event"))
        append(kinds, {flag: n.getValue("flag"), value: n.getValue("value") + 0, text: n.getValue("text"), level: n.getValue("level"),
                       mode: n.getValue("mode")});
    foreach (var k; ["master-warning", "master-caution", "aural/warning", "aural/caution", "aural/autopilot"]) setprop(OUT ~ k, 0);
    setprop(OUT ~ "aural/warning-gain", 1);
    # OUT/acknowledge set: as ACK (a key binding's or a script's way)
    setlistener(OUT ~ "acknowledge", func(n) { if (n.getValue()) { n.setValue(0); acknowledge(); } }, 0, 0);
    timer = maketimer(0.1, update);
    timer.start();
}

var listener = setlistener("/sim/signals/fdm-initialized", func {
    removelistener(listener);
    init();
});
