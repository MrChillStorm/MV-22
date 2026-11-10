# The radar warning receiver: which emitters the azimuth indicator on the console hears, and how much each matters.
# The Rwr unit in Nasal/displays.nas draws them on the indicator's face.
#
# The V-22 carries the AN/APR-39A(V)2 [FAS]: alphanumeric symbols by bearing, with synthetic speech. How its family's
# indicator behaves is in the Army's operators' manuals [CH-47, AH-64] and a unit's brief [BRIEF]. A symbol's distance from
# the centre shows lethality, by the emitter's condition or mode and not by range, the most lethal nearest the centre.
# Search radars, and fire control radars in search mode, are strobes at the edge at their bearing. At most the 7 highest
# priority show, of the 10 tracked; an emitter not identified is U; a new threat is in boldface; one that drops out, or
# whose lock breaks, is ghosted for 10 s as though dotted. In a dense environment the + flashes and it says so. What the
# symbols mean is classified [AH-64], so the codes here are generic: a type's number or letters, U for an unknown.
#
# [FAS]   https://man.fas.org/dod-101/sys/ac/equip/an-apr-39.htm
# [CH-47] TM 1-1520-240-10, 4-1-2: https://ch-47helicopters.tpub.com/TM-1-1520-240-10/css/TM-1-1520-240-10_200.htm
# [AH-64] TM 1-1520-238-10, 4.38: https://apachehelicopter.tpub.com/TM-1-1520-238-10/css/TM-1-1520-238-10_349.htm
# [BRIEF] https://www.slideserve.com/fletcher-hampton/717th-medical-company-an-apr-39a-v-1-radar-warning-detector
#
# The modes here: a radar on with no lock is searching, a lock is tracking. An AI contact tells neither, so a weapon's
# radar among them inside its lethal range is taken as acquiring: a symbol outside the ring, so that its type shows.
#
# What it hears is decided as the FlightGear F-16's radar system decides it, whose threat weights and emitter table
# these are: a contact's radar on, this aircraft inside its radar's field, no ground between; or its lock on us, which
# OPRF aircraft send as the first 4 characters of the md5 of the callsign they have locked in generic string[6]; their
# generic int[2] at 1 is a radar on standby. Unlike there, only contacts that can warn are looked at (not the airline
# traffic), the multiplayer properties that come after a contact does are still read, a lock from a model not in the
# table shows as an unknown, the bearing is kept true so the display turns with the aircraft between looks, and the
# radar horizon is 4/3 earth's.
#
# Missiles: the OPRF aircraft tell the others of each weapon in flight over multiplayer (emesary bridge 18, the F-16's
# ArmamentInFlightNotification), so a launch by any of them shows here. A missile burning within PLUME_NM, or one with
# its own radar on us at any range, is a track of its own, M, the first to be shown; a launcher, or one illuminating us
# for a semi-active missile, flashes a circle on its symbol. This only listens: it sends nothing, and takes no damage.
# Unlike the F-16 it has that one bridge, and only reads what it uses of a message.
#
# The set speaks as its symbols appear, in the manuals' full format: its class, its type, the clock, the mode ("SA, SA-8
# twelve o'clock tracking" [AH-64]); at power on, "APR 39 power up". A missile is time-critical: said the moment
# word of it comes, whatever else stands, and again every alerts.CRITICAL_S while it flies, as the crew alerts' own
# time-critical warnings are. Any other track is said once and each time it gets worse, SPEAK_S apart, and gives way to a
# time-critical crew alert. The voice is FlightGear's own (sim/sound/voices/voice[5], Systems/v22-base.xml): a queue and
# a voice apart from the crew alerts' (Nasal/alerts.nas), so a missile call need not wait for theirs. It is in the crew's
# headsets, so it is heard in the cockpit views only: from an outside view the set goes on (what it has said is said) but
# nothing is spoken. (Its level is FlightGear's: its speech plays at a fixed volume, by the master volume alone.)
#
# After the F-16's Nasal/rwr.nas, damage.nas, ArmamentNotification.nas, GeoBridgedTransmitter.nas,
# radar/radar-system.nas, radar/apg-68.nas and radar/radar-system-database.nas
# (https://github.com/NikolaiVChr/f16), GPL 2.0 or later:
#   Copyright (C) 2023 Erik Hofman and the F-16's authors (its authors.txt)
#   the radar system: Alexis Bory, Richard Harrison, Fabien Barber, Nikolai V. Chr, Justin Nicholson, Colin Geniet
#   the notifications and their bridges: Richard Harrison
#   the emitter table: inspired from pinto's MiG-21bis mp_db.nas

var TICK = 0.1;          # s between steps
var SCAN = 1.0;          # s between looks at one contact
var RANK = 0.5;          # s between sorting the threats
var SWEEP = 30;          # s: /ai/models read again at least this often, as well as when a model comes or goes
var MAX_NM = 150;        # the F-16's; low down the radar horizon comes first
var LOST = 2.5;          # s without an intercept: it has dropped out
var GHOST_S = 10;        # s a threat that dropped out, or whose lock broke, stays ghosted [CH-47]
var HOLD = LOST + GHOST_S;
var NEW = 5;             # s a new track is in boldface
var SHOWN = 7;           # the highest priority signals on the indicator [CH-47]
var TRACKS = 10;         # those it tracks [BRIEF]: more is the dense environment that degrades it
var GROUND_M = 300;      # m short of a surface emitter a ray may meet the ground: it stands on it
var PLUME_NM = 2.25;     # a motor burning is seen this far (the F-16's missile launch warning)
var ALARM = 3;           # s a launcher's or illuminator's symbol flashes its circle after the last word of it
var SPEAK_S = 3;         # s between calls, about the time one takes
var VOICE = "/sim/sound/voices/voice[5]/text";
var INTERNAL = props.globals.getNode("/sim/current-view/internal", 1);
# a call, spoken only where the headsets are heard: in a cockpit view
var speak = func(text) { if (INTERNAL.getBoolValue()) setprop(VOICE, text); }
var MODES = ["", "searching", "acquisition", "tracking", "launch", "inbound"];        # the call's last word, by a track's level
var CLOCK = ["twelve", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten", "eleven"];
var OUT = "/instrumentation/mv22b/rwr/";

# ---------------------------------------------------------------- what radiates
# kinds: the field its radar covers (deg each side of the nose, 180 all round), the speed above which it is airborne
# (kt; 0 never), its threat before range, its class as said, whether its type is said too
var FIGHTER = {fov: 60, air: 60, base: 0.30, say: "fighter", named: 1};
var HELO = {fov: 60, air: 10, base: 0.30, say: "helicopter", named: 1};
var AWACS = {fov: 180, air: 60, base: 0, say: "radar", named: 0};
var WEAPON = {fov: 180, air: 0, base: 0.30, say: "S A", named: 1};
var GUN = {fov: 180, air: 0, base: 0.30, say: "gun", named: 0};
var SHIP = {fov: 180, air: 0, base: 0.30, say: "ship", named: 0};
var SEARCH = {fov: 180, air: 0, base: 0, say: "radar", named: 0};
var UNKNOWN = {kind: {fov: 180, air: 60, base: 0, say: "unknown", named: 0}, code: "U", kill: 0, said: "unknown, "};

var spell = func(code) {         # a number as it is, letters one by one
    if (num(code) != nil) return code;
    var s = "";
    for (var i = 0; i < size(code); i += 1) s ~= substr(code, i, 1) ~ " ";
    return s;
}

var EMITTERS = {};
var emitters = func(kind, code, kill, models) {      # kill: its weapon's lethal range, NM (0: it has none)
    foreach (var m; models) EMITTERS[m] = {kind: kind, code: code, kill: kill, said: kind.say ~ ", " ~ (kind.named ? spell(code) ~ ", " : "")};
}
emitters(FIGHTER, "14", 80, ["f-14b", "F-14D"]);
emitters(FIGHTER, "15", 50, ["F-15", "F-15C", "F-15D", "F-15C_Eagle", "F-15J_ADTW", "F-15DJ_ADTW"]);
emitters(FIGHTER, "16", 50, ["F-16", "f-16", "f16", "F-16CJ", "F-16A-MLU", "f16-Wingman", "Mig-28"]);
emitters(FIGHTER, "18", 50, ["F-18", "FA-18", "f18", "FA-18C_Hornet", "FA-18D_Hornet", "FA-18E_CVW5"]);
emitters(FIGHTER, "21", 50, ["MiG-21bis", "MiG-21MF-75", "MiG-21Bison"]);
emitters(FIGHTER, "22", 50, ["F-22-Raptor"]);
emitters(FIGHTER, "23", 50, ["MiG-23ML", "MiG-23MLD"]);
emitters(FIGHTER, "25", 50, ["MiG-25"]);
emitters(FIGHTER, "27", 50, ["SU-27", "J-11A"]);
emitters(FIGHTER, "29", 50, ["MiG-29", "Mig-29"]);
emitters(FIGHTER, "34", 50, ["daVinci_SU-34", "Su-34", "SU-37"]);
emitters(FIGHTER, "35", 50, ["F-35A", "F-35B", "F-35C", "daVinci_F-35A", "daVinci_F-35B"]);
emitters(FIGHTER, "37", 50, ["JA-37", "JA37-Viggen", "JA37Di-Viggen"]);
emitters(FIGHTER, "37", 15, ["AJ37-Viggen", "AJS37-Viggen"]);
emitters(FIGHTER, "39", 50, ["JAS-39C_Gripen", "gripen"]);
emitters(FIGHTER, "57", 50, ["T-50"]);
emitters(FIGHTER, "A4", 50, ["a4f", "A-4K"]);
emitters(FIGHTER, "A6", 50, ["A-6E", "ea-6b"]);
emitters(FIGHTER, "AJ", 50, ["alphajet"]);
emitters(FIGHTER, "AV", 50, ["AV-8B"]);
emitters(FIGHTER, "B1", 50, ["B-1B"]);
emitters(FIGHTER, "EF", 50, ["Typhoon", "EF2000"]);
emitters(FIGHTER, "ET", 50, ["super-etendard"]);
emitters(FIGHTER, "F1", 50, ["F-111C", "daVinci_F-111G"]);
emitters(FIGHTER, "F4", 50, ["F-4", "F-4C", "F-4D", "F-4E", "F-4EJ", "F-4EJ_ADTW", "F-4F", "F-4J", "F4J", "F-4N", "F-4S",
                             "FGR2", "FGR2-Phantom", "QF-4E"]);
emitters(FIGHTER, "F5", 50, ["F-5", "F-5E", "F-5E-TigerII", "F-5ENinja"]);
emitters(FIGHTER, "HA", 50, ["Harrier-GR1", "Harrier-GR3", "Harrier-GR5", "Harrier-GR9", "Harrier-FRS2"]);
emitters(FIGHTER, "HK", 50, ["BAE_Hawk_T2"]);
emitters(FIGHTER, "LC", 50, ["L-159"]);
emitters(FIGHTER, "M1", 50, ["Mirage_F1"]);
emitters(FIGHTER, "M2", 50, ["m2000-5", "m2000-5B", "m2000-5_lowpoly"]);
emitters(FIGHTER, "RF", 50, ["brsq"]);
emitters(FIGHTER, "SF", 50, ["TF-104G"]);
emitters(FIGHTER, "SU", 50, ["Su-15"]);
emitters(FIGHTER, "TO", 50, ["Tornado-GR4a", "Tornado-IDS", "Tornado-ECR", "Tornado-F3", "Tornado-ADV"]);
emitters(FIGHTER, "TS", 50, ["f-20A", "f-20C", "f-20prototype", "f-20bmw", "f-20-dutchdemo"]);
emitters(FIGHTER, "U", 50, ["ufo", "F-23C_BlackWidow-II"]);
emitters(FIGHTER, "Y1", 50, ["Yak-130"]);
emitters(HELO, "8", 15, ["Mil-Mi-8"]);
emitters(HELO, "24", 15, ["mi24"]);
emitters(HELO, "50", 15, ["ka50"]);
emitters(AWACS, "S", 0, ["A-50", "E-2C", "E-2C-Hawkeye", "E-3", "E-3R", "E3B", "E-7A", "EC-137D", "EC-137R", "onox-awacs"]);
emitters(SEARCH, "S", 0, ["gci", "tower"]);
emitters(WEAPON, "2", 35, ["S-75"]);
emitters(WEAPON, "3", 18, ["SA-3"]);
emitters(WEAPON, "5", 150, ["s-200"]);
emitters(WEAPON, "6", 15, ["SA-6"]);
emitters(WEAPON, "17", 35, ["buk-m1-2", "buk-m2"]);
emitters(WEAPON, "20", 80, ["s-300"]);
emitters(GUN, "AA", 7.5, ["ZSU-23-4M"]);
emitters(WEAPON, "P", 45, ["MIM104D"]);
emitters(SHIP, "SH", 80, ["fleet", "frigate", "missile_frigate", "USS-LakeChamplain", "USS-NORMANDY", "USS-OliverPerry",
                            "USS-SanAntonio"]);
emitters(SHIP, "SH", 50, ["mp-clemenceau", "mp-eisenhower", "mp-nimitz", "mp-truman", "mp-vinson"]);

var SECOND_SEATS = {"f-14b-bs": 1, "f15-bs": 1, "m2000-5B-backseat": 1, "hunter": 1};     # never a contact of their own
var NOT_CONTACTS = {ballistic: 1, munition: 1};

# ---------------------------------------------------------------- the line of sight
var in_view = func(own, there, dist, airborne) {
    # the radar horizon: 4.12 km per root metre of either height
    if (dist > 4120 * (math.sqrt(math.max(0, own.alt())) + math.sqrt(math.max(0, there.alt())))) return 0;
    if (own.alt() > 8900 and there.alt() > 8900) return 1;           # both over the highest ground
    var hit = get_cart_ground_intersection({x: own.x(), y: own.y(), z: own.z()},
                                           {x: there.x() - own.x(), y: there.y() - own.y(), z: there.z() - own.z()});
    if (hit == nil) return 1;
    var ground = geo.Coord.new().set_latlon(hit.lat, hit.lon, hit.elevation);
    return own.direct_distance_to(ground) > dist - (airborne ? 1 : GROUND_M);
}

# ---------------------------------------------------------------- a contact, and its track
var Contact = {
    new: func(node, raw, model) {
        var c = {parents: [Contact], raw: raw, e: EMITTERS[model], spike: nil, standby: nil,
                 callsign: node.getValue("callsign") or "",
                 scanned: -1e9, seen: -1e9, first: 0, unlocked: -1e9, alarm: -1e9, lock: 0, acquiring: 0, airborne: 0, lethal: 0, told: 0, order: 0, say: "",
                 bearing: 0, range: nil, ranged: 0, closure: 0, p: 0, code: ""};
        c.lat = node.getNode("position/latitude-deg", 1);
        c.lon = node.getNode("position/longitude-deg", 1);
        c.alt = node.getNode("position/altitude-ft", 1);
        c.heading = node.getNode("orientation/true-heading-deg", 1);
        c.speed = node.getNode("velocities/true-airspeed-kt", 1);
        if (node.getName() == "multiplayer") {        # made here: they come with its first packets, not with it
            c.spike = node.getNode("sim/multiplay/generic/string[6]", 1);
            c.standby = node.getNode("sim/multiplay/generic/int[2]", 1);
        }
        return c;
    },
    ghost: func(now) { now - me.seen > LOST or now - me.unlocked < GHOST_S },
    level: func { me.lock ? 3 : me.acquiring ? 2 : 1 },         # searching, acquiring, tracking
    tier: func(now) { now - me.unlocked < GHOST_S ? 3 : me.level() },     # where it is drawn: a broken lock's ghost stays
    scan: func(now, own) {
        me.scanned = now;
        var lock = me.spike != nil and me.spike.getValue() == mark;
        if (me.lock and !lock) { me.lock = 0; me.unlocked = now; me.told = math.min(me.told, 2); }     # a new lock is said again
        if (!lock and (me.e == nil or (me.standby != nil and me.standby.getValue() == 1))) return;
        var lat = me.lat.getValue(); var lon = me.lon.getValue(); var alt = me.alt.getValue();
        if (lat == nil or lon == nil or alt == nil) return;
        var there = geo.Coord.new().set_latlon(lat, lon, alt * FT2M);
        var e = me.e == nil ? UNKNOWN : me.e;
        var dist = own.direct_distance_to(there);
        var range = dist * M2NM;
        var bearing = own.course_to(there);
        var off = math.abs(geo.normdeg180(bearing + 180 - (me.heading.getValue() or 0)));     # us off its nose
        var airborne = e.kind.air > 0 and (me.speed.getValue() or 0) > e.kind.air;
        if (!lock and (range > MAX_NM or off > e.kind.fov or !in_view(own, there, dist, airborne))) return;

        if (now - me.seen > HOLD) { me.first = now; me.range = nil; me.told = 0; }        # a new track
        if (me.range != nil and now > me.ranged) me.closure = (me.range - range) / (now - me.ranged) * 3600;
        me.seen = now; me.ranged = now; me.range = range;
        me.bearing = bearing; me.lock = lock; me.airborne = airborne; me.code = e.code; me.say = e.said;
        # the threat: the F-16's weights, the rate of closure from the range between looks
        var zone = e.kill > 0 ? math.max(0, 1 - range / e.kill) : 0;
        var aspect = e.kind.fov < 180 ? (180 - off) / 180 : 1;
        me.p = math.min(1, math.max(0.05, e.kind.base * aspect + 0.6 * zone + (lock ? 0.3 : 0) + math.max(0, me.closure) / 5000));
        me.lethal = lock;
        me.acquiring = me.spike == nil and e.kill > 0 and range < e.kill;       # an AI contact tells no mode
        me.order = me.level() + me.p / 2;       # by mode, as the manual has it, the threat within a mode
    },
};

# ---------------------------------------------------------------- missiles
var Missile = {
    code: "M", lethal: 1, airborne: 0, alarm: -1e9, say: "missile, ",
    new: func(now) { return {parents: [Missile], first: now, seen: now, bearing: 0, p: 0, lock: 0, said: -1e9}; },
    ghost: func(now) { now - me.seen > LOST },
    level: func { me.lock ? 5 : 4 },
    tier: func(now) { me.level() },
};
var missiles = {};       # by launcher's callsign and the weapon's number
var senders = {};        # the multiplayer contacts, by callsign

# A message as it comes: where the weapon is, what it is doing, whom it is after. Only these are read: its speed,
# heading and pitch are stepped over, and its position is left as it came until it matters.
var IN_FLIGHT = 21;      # the notification's type id
var MOVE = 2; var DESTROY = 3;             # its Kind: a weapon flying, one gone
var NOT_MISSILES = {114: 1, 116: 1};       # its SecondaryKind: an ejected pilot, a flare (damage.nas's warheads 93, 95, + 21)
var coord_size = emesary.TransferCoord.LatLonLength * 2 + emesary.TransferCoord.AltLength;
var in_flight = emesary.Notification.new("ArmamentInFlightNotification", "none", IN_FLIGHT);
var fields = [
    {setValue: func(v, root, pos) { in_flight.message = v; in_flight.at = pos; return {pos: pos + coord_size}; }},
    {setValue: func(v, root, pos) { var d = emesary.TransferByte.decode(v, pos); in_flight.Kind = d.value; return d; }},
    {setValue: func(v, root, pos) { var d = emesary.TransferByte.decode(v, pos); in_flight.SecondaryKind = d.value; return d; }},
    {setValue: func(v, root, pos) { return {pos: pos + 3}; }},
    {setValue: func(v, root, pos) { var d = emesary.TransferString.decode(v, pos); in_flight.RemoteCallsign = d.value; return d; }},
    {setValue: func(v, root, pos) { var d = emesary.TransferByte.decode(v, pos); in_flight.Flags = d.value; return d; }},
    {setValue: func(v, root, pos) { var d = emesary.TransferByte.decode(v, pos); in_flight.UniqueIdentity = d.value; return d; }},
];
in_flight.bridgeProperties = func { fields };

var heard = func(n) {
    var OK = emesary.Transmitter.ReceiptStatus_OK;
    if (n.NotificationType != in_flight.NotificationType or !n.FromIncomingBridge) return emesary.Transmitter.ReceiptStatus_NotProcessed;
    if (contains(NOT_MISSILES, n.SecondaryKind) or !displays.powered()) return OK;
    var key = n.Callsign ~ "/" ~ n.UniqueIdentity;
    if (n.Kind == DESTROY) { delete(missiles, key); return OK; }
    var source = senders["cs" ~ n.Callsign];              # none: a pilot ignored
    if (n.Kind != MOVE or source == nil) return OK;
    var ours = n.RemoteCallsign == target;
    var radar = ours and bits.test(n.Flags, 0);
    var burning = bits.test(n.Flags, 1);
    var now = elapsed.getValue();
    if (ours and bits.test(n.Flags, 2)) source.alarm = now;   # its launcher's illuminator on us
    if (!radar and !burning) return OK;
    var own = geo.Coord.new().set_latlon(own_lat.getValue(), own_lon.getValue(), own_alt.getValue() * FT2M);
    var at = emesary.TransferCoord.decode(n.message, n.at).value;
    if (!radar and own.direct_distance_to(at) * M2NM > PLUME_NM) return OK;
    var m = missiles[key];
    var fresh = m == nil;
    if (fresh) { m = missiles[key] = Missile.new(now); source.alarm = now; }
    m.seen = now; m.bearing = own.course_to(at); m.lock = radar; m.p = 1; m.order = m.level() + 1;
    if (fresh) rank(now);        # shown and said now, not at the next ranking
    return OK;
}

# ---------------------------------------------------------------- the scan
var contacts = {};       # by node path, every contact of /ai/models
var watched = [];        # those that can warn: a known radar, or a multiplayer pilot who may lock us
var shown = [];          # the tracks on the indicator, highest priority first (displays.nas reads this)
var mark = "";           # our callsign as a lock on us comes
var target = "";         # and as a missile after us names it
var degraded = 0;        # more signals than it can track: the + flashes (displays.nas)
var powered = 0;
var cursor = 0;
var spoke = -1e9;
var swept = -1e9;
var ranked = -1e9;
var dirty = 1;
var models = nil; var own_lat = nil; var own_lon = nil; var own_alt = nil; var own_hdg = nil; var elapsed = nil;
var out_count = nil; var out = [];
var timer = nil;         # kept: a timer nothing refers to is collected, and stops

var model_name = func(raw) {
    var m = string.truncateAt(io.basename(raw), ".xml");
    foreach (var s; ["-model", "-anim"])
        if (size(m) > size(s) and substr(m, size(m) - size(s)) == s) m = substr(m, 0, size(m) - size(s));
    return m;
}

var emesary_callsign = func(callsign) {       # its first 7 characters, those a message can carry
    var s = "";
    for (var i = 0; i < math.min(size(callsign), 7); i += 1) {
        var c = emesary.TransferString.getalphanumericchar(substr(callsign, i, 1));
        if (c != nil) s ~= c;
    }
    return s;
}

var sweep = func {
    var callsign = getprop("/sim/multiplay/callsign") or "";
    mark = substr(md5(callsign), 0, 4);
    target = emesary_callsign(callsign);
    var alive = {};
    foreach (var n; models.getChildren()) {
        if (!n.getValue("valid") or contains(NOT_CONTACTS, n.getName()) or n.getValue("controls/invisible")
            or n.getNode("impact") != nil or n.getNode("missile") != nil) continue;
        var raw = n.getValue("sim/model/path") or "";
        var path = n.getPath();
        alive[path] = 1;
        if (contains(contacts, path) and contacts[path].raw == raw) continue;
        var model = model_name(raw);
        if (contains(SECOND_SEATS, model)) delete(alive, path);
        else contacts[path] = Contact.new(n, raw, model);
    }
    foreach (var p; keys(contacts)) if (!contains(alive, p)) delete(contacts, p);
    watched = [];
    senders = {};
    foreach (var p; keys(alive)) {
        var c = contacts[p];
        if (c.e != nil or c.spike != nil) append(watched, c);
        if (c.spike != nil) senders["cs" ~ c.callsign] = c;
    }
    cursor = 0;
}

var publish = func(now) {
    out_count.setIntValue(size(shown));
    var hdg = own_hdg.getValue();
    forindex (var i; shown) {
        var t = shown[i]; var o = out[i];
        o.code.setValue(t.code);
        o.bearing.setDoubleValue(geo.normdeg(t.bearing - hdg));
        o.lethal.setBoolValue(t.lethal);
        o.level.setIntValue(t.tier(now));
        o.lock.setBoolValue(t.lock);
        o.airborne.setBoolValue(t.airborne);
        o.ghost.setBoolValue(t.ghost(now));
    }
}

var announce = func(t, level) {
    var o = math.mod(int(geo.normdeg(t.bearing - own_hdg.getValue()) / 30 + 0.5), 12);
    speak(t.say ~ CLOCK[o] ~ " o clock " ~ MODES[level]);
}

var say = func(now) {
    var waits = now - spoke < SPEAK_S or alerts.critical_line()[0] != "";
    foreach (var t; shown) {         # the missiles first: they rank above any radar
        if (t.ghost(now)) continue;
        var level = t.level();
        if (level >= 4) {
            if (now - t.said < alerts.CRITICAL_S) continue;
            t.said = now;
        } elsif (waits or level <= t.told) continue;
        else t.told = level;
        spoke = now;
        announce(t, level);
        return;
    }
}

var rank = func(now) {
    ranked = now;
    var live = [];
    foreach (var c; watched) if (now - c.seen < HOLD) append(live, c);
    foreach (var k; keys(missiles)) {
        if (now - missiles[k].seen < HOLD) append(live, missiles[k]);
        else delete(missiles, k);
    }
    live = sort(live, func(a, b) { a.order > b.order ? -1 : a.order < b.order ? 1 : 0 });
    shown = subvec(live, 0, math.min(size(live), SHOWN));
    if ((size(live) > TRACKS) != degraded) {
        degraded = !degraded;
        spoke = now;
        speak("threat detection " ~ (degraded ? "degraded" : "restored"));
    }
    publish(now);
    say(now);
}

var step = func {
    var now = elapsed.getValue();
    if (!displays.powered()) {
        powered = 0; degraded = 0;
        if (size(shown)) { shown = []; missiles = {}; publish(now); }
        return;
    }
    if (!powered) { powered = 1; spoke = now; speak("A P R 39 power up"); }
    if (dirty or now - swept >= SWEEP) { sweep(); swept = now; dirty = 0; }
    var n = size(watched);
    if (n) {
        var own = geo.Coord.new().set_latlon(own_lat.getValue(), own_lon.getValue(), own_alt.getValue() * FT2M);
        for (var i = int(n * TICK / SCAN) + 1; i > 0; i -= 1) {       # the scan's work spread over its steps
            cursor = cursor + 1 < n ? cursor + 1 : 0;
            if (now - watched[cursor].scanned >= SCAN) watched[cursor].scan(now, own);
        }
    }
    if (now - ranked >= RANK) rank(now);
}

var init = func {
    models = props.globals.getNode("/ai/models", 1);
    own_lat = props.globals.getNode("/position/latitude-deg");
    own_lon = props.globals.getNode("/position/longitude-deg");
    own_alt = props.globals.getNode("/position/altitude-ft");
    own_hdg = props.globals.getNode("/orientation/heading-deg");
    elapsed = props.globals.getNode("/sim/time/elapsed-sec");
    out_count = props.globals.getNode(OUT ~ "count", 1);
    for (var i = 0; i < SHOWN; i += 1) {
        var t = props.globals.getNode(OUT ~ "track[" ~ i ~ "]", 1);
        append(out, {code: t.getNode("code", 1), bearing: t.getNode("relative-bearing-deg", 1), lethal: t.getNode("lethal", 1), level: t.getNode("level", 1),
                     lock: t.getNode("lock", 1), airborne: t.getNode("airborne", 1), ghost: t.getNode("ghost", 1)});
    }
    setlistener("/ai/models/model-added", func { dirty = 1; }, 0, 0);
    setlistener("/ai/models/model-removed", func { dirty = 1; }, 0, 0);
    timer = maketimer(TICK, step);
    timer.start();
}

# a message comes from anyone, and a recipient that throws is taken off the transmitter for good: the warnings with it
var recipient = emesary.Recipient.new("RWR");
recipient.Receive = func(n) {
    var r = call(heard, [n], nil, nil, var err = []);
    if (size(err)) { logprint(LOG_WARN, "RWR: a bad message from ", n["Callsign"], ": ", err[0]); return emesary.Transmitter.ReceiptStatus_NotProcessed; }
    return r;
}
emesary.GlobalTransmitter.Register(recipient);
emesary_mp_bridge.IncomingMPBridge.startMPBridge([in_flight], 18);

var listener = setlistener("/sim/signals/fdm-initialized", func {
    removelistener(listener);
    init();
});
