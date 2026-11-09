# (TEMPORARY, not for commit: the scenery's terrain and land cover round LOWI to $FG_HOME/Export/lowi_dem.txt, for an
# offline copy of eosite.nas's search; loaded by eosite.nas's debug hook)
var LAT0 = 47.2602; var LON0 = 11.3440; var HALF_M = 12000; var STEP_M = 75;
var N = int(2 * HALF_M / STEP_M) + 1;
var path = getprop("/sim/fg-home") ~ "/Export/lowi_dem.txt";
var f = io.open(path, "w");
io.write(f, sprintf("# lat0 %.6f lon0 %.6f half_m %d step_m %d n %d\n", LAT0, LON0, HALF_M, STEP_M, N));
var kl = 111320.0; var kc = math.cos(LAT0 * math.pi / 180);
var row = 0;
var tick = func {
    for (var k = 0; k < 3 and row < N; k += 1) {
        var lat = LAT0 - HALF_M / kl + row * STEP_M / kl;
        var line = "";
        for (var col = 0; col < N; col += 1) {
            var lon = LON0 - HALF_M / (kl * kc) + col * STEP_M / (kl * kc);
            var g = geodinfo(lat, lon);
            if (g == nil) line ~= "nan - 0 ";
            else {
                var m = g[1];
                line ~= sprintf("%.2f %s %d ", g[0], (m == nil or !size(m.names)) ? "-" : m.names[0], m == nil ? 0 : m.solid);
            }
        }
        io.write(f, line ~ "\n");
        row += 1;
    }
    setprop("/sim/eodump/row", row);
    if (row >= N) { io.close(f); timer.stop(); setprop("/sim/eodump/done", 1); print("EODUMP done ", path); }
};
var timer = maketimer(0.1, tick);
timer.start();
