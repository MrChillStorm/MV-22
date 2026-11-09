# (TEMPORARY, not for commit: every input and output of eosite.nas to a file, for replaying it on the bench)
var _p = getprop("/sim/eotrace/path");
var TR = io.open(_p != nil ? _p : getprop("/sim/fg-home") ~ "/Export/" ~ getprop("/sim/eotrace/file"), "w");
var trw = func(s) { io.write(TR, s); };
var _geo = globals.geodinfo;
var geodinfo = func(lat, lon) {
    var g = _geo(lat, lon);
    if (g == nil) trw(sprintf("G %.9f %.9f nil\n", lat, lon));
    else {
        var m = g[1];
        trw(sprintf("G %.9f %.9f %.6f %s %d\n", lat, lon, g[0], (m == nil or !size(m.names)) ? "-" : m.names[0], m == nil ? 0 : m.solid));
    }
    return g;
};
var _num = num;
num = func(p, d = 0) {
    var x = getprop(p);
    var v = x == nil ? d : x;
    if (x != nil) trw(sprintf("N %s %.12g\n", p, v));
    return v;
};
var _read = read_state;
read_state = func { trw(sprintf("T %.4f\n", getprop("/sim/time/elapsed-sec"))); _read(); };
var _send = send_path;
send_path = func(pl) { trw(sprintf("P path side %d k %s hac %.2f fin %.0f\n", pl[2], pl[7] == nil ? "nil" : pl[7][11], pl[9], pl[10])); _send(pl); };
var _rank = rank;
rank = func {
    _rank();
    trw(sprintf("R %.4f lines %d cands %d chosen %s floor %.1f\n", S.now, size(lines), size(cands), chosen == nil ? "none" : sprintf("%s|%.5f|%.5f|%d", chosen.kind, chosen.lat, chosen.lon, chosen.crs), floor_));
    foreach (var c; cands) trw(sprintf("C %s %.5f %.5f %d tier %d clear %d margin %.1f path %.1f\n", c.kind, c.lat, c.lon, c.crs, tier(c), c.clear, c.margin, c.path));
    io.flush(TR);
};
