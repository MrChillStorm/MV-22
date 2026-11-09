# each stage's time on the VM alone
var ST = {};
var wrapst = func(name, f) {
    ST[name] = {sum: 0, n: 0, mx: 0};
    var e = ST[name];
    return func {
        var t = vmtime();
        var r = call(f, arg);
        var d = vmtime() - t;
        e.sum += d; e.n += 1;
        if (d > e.mx) e.mx = d;
        return r;
    };
}
tries_tick = wrapst("tries", tries_tick); price_tick = wrapst("price_tick", price_tick); rank = wrapst("rank", rank);
publish = wrapst("publish", publish);
search_order = wrapst("search_order", search_order); open_course = wrapst("open_course", open_course); trim_lines = wrapst("trim", trim_lines);
sample = wrapst("sample", sample); energy_plans = wrapst("energy_plans", energy_plans); path_clear = wrapst("path_clear", path_clear);
var bench_stages = func {
    var s = "";
    foreach (var k; sort(keys(ST), cmp)) { var e = ST[k]; s ~= sprintf("%-13s %7d calls %8.1f ms total %7.3f mean %7.2f worst\n", k, e.n, e.sum * 1000, e.n ? e.sum / e.n * 1000 : 0, e.mx * 1000); }
    return s;
}
