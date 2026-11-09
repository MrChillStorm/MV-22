# the bench's look at the module
var bench_key = func { return chosen == nil ? nil : sprintf("%s %.5f %.5f %d", name_of(chosen), chosen.lat, chosen.lon, chosen.crs); }
var bench_site = func {
    if (chosen == nil) return nil;
    var c = chosen;
    return sprintf("%s|%.7f|%.7f|%.3f|%.7f|%.7f|%.7f|%.7f|%.1f", c.kind, c.lat, c.lon, c.crs, c.ends[0], c.ends[1], c.ends[2], c.ends[3], contains(c, "spare") and c.kind != "RWY" ? c.spare : 0);
}
var bench_lines = func { return size(lines); }
var bench_sampling = func { return sampling; }
# the costs of the hot functions
var bench_report = func {
    var c = size(lines) ? lines[0] : nil;
    if (c == nil) return "no lines";
    var t0 = systime();
    for (var i = 0; i < 2000; i += 1) geom(c, S.hb0, 0);
    var t1 = systime();
    for (var i = 0; i < 2000; i += 1) price(c);
    var t2 = systime();
    return sprintf("geom %.1f us, price %.1f us, %d lines", (t1 - t0) / 2000 * 1e6, (t2 - t1) / 2000 * 1e6, size(lines));
}

