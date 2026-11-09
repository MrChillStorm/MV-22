# (TEMPORARY, not for commit: the scenery's terrain and land cover round LOWI to $FG_HOME/Export/lowi_dem2.txt)
var LAT0 = 47.2602; var LON0 = 11.3440; var HALF_M = 16000; var STEP_M = 30;
var N = int(2 * HALF_M / STEP_M) + 1;
var kl = 111320.0; var kc = math.cos(LAT0 * math.pi / 180);
var probe = func {
    var n = 0; var bad = 0;
    for (var a = 0; a <= 32; a += 1) for (var b = 0; b <= 32; b += 1) {
        var lat = LAT0 - HALF_M / kl + a * 2 * HALF_M / 32 / kl;
        var lon = LON0 - HALF_M / (kl * kc) + b * 2 * HALF_M / 32 / (kl * kc);
        var g = geodinfo(lat, lon);
        n += 1;
        if (g == nil or g[1] == nil) bad += 1;
    }
    setprop("/sim/eodump/probe-n", n); setprop("/sim/eodump/probe-bad", bad); setprop("/sim/eodump/probe-done", getprop("/sim/time/elapsed-sec"));
};
setlistener("/sim/eodump/probe", func(n) { if (n.getValue()) probe(); }, 0, 0);
var f = nil; var row = 0; var dict = {}; var nd = 0;
var tick = func {
    var t0 = systime();
    while (row < N and systime() - t0 < 0.06) {
        var lat = LAT0 - HALF_M / kl + row * STEP_M / kl;
        var parts = [];
        for (var col = 0; col < N; col += 1) {
            var lon = LON0 - HALF_M / (kl * kc) + col * STEP_M / (kl * kc);
            var g = geodinfo(lat, lon);
            if (g == nil) { append(parts, "nan -1"); continue; }
            var m = g[1]; var key = "-";
            if (m != nil) key = (m.solid ? "1 " : "0 ") ~ string.join(",", m.names);
            if (!contains(dict, key)) { dict[key] = nd; io.write(f, sprintf("D %d %s\n", nd, key)); nd += 1; }
            append(parts, sprintf("%.2f %d", g[0], dict[key]));
        }
        io.write(f, string.join(" ", parts) ~ "\n");
        row += 1;
    }
    setprop("/sim/eodump/row", row);
    if (row >= N) { io.close(f); timer.stop(); setprop("/sim/eodump/done", 1); }
};
var timer = maketimer(0.02, tick);
setlistener("/sim/eodump/go", func(n) {
    if (!n.getValue()) return;
    f = io.open(getprop("/sim/fg-home") ~ "/Export/" ~ getprop("/sim/eodump/file"), "w");
    io.write(f, sprintf("# lat0 %.6f lon0 %.6f half_m %d step_m %d n %d\n", LAT0, LON0, HALF_M, STEP_M, N));
    timer.start();
}, 0, 0);
