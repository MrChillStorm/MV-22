/* The Nasal VM embedded for the bench: FlightGear's getprop, setprop, geodinfo and logging answered by Python callbacks,
   modules laid out as FlightGear lays them (a hash per file in the globals), a call's time with the callbacks' taken out. */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <mach/mach_time.h>
#include "nasal.h"
#include "data.h"

typedef int  (*get_cb)(const char*, double*, char*, int);
typedef void (*set_cb)(const char*, double, const char*);
typedef int  (*geo_cb)(double, double, double*, char*, int, int*);
typedef void (*log_cb)(const char*);
static get_cb cb_get; static set_cb cb_set; static geo_cb cb_geo; static log_cb cb_log;
static naContext ctx; static naRef G;
static double tick_s; static double t_cb;
static long n_get, n_set, n_geo;
static double now_s(void) { return mach_absolute_time() * tick_s; }

static const char* cstr(naContext c, naRef r) { naRef s = naStringValue(c, r); return naIsNil(s) ? "" : naStr_data(s); }
static naRef f_getprop(naContext c, naRef me, int argc, naRef* a) {
    double x = 0, t = now_s(); char buf[256]; int k; n_get++;
    k = cb_get(cstr(c, a[0]), &x, buf, sizeof buf);
    t_cb += now_s() - t;
    if (k == 1) return naNum(x);
    if (k == 2) { naRef s = naNewString(c); return naStr_fromdata(s, buf, strlen(buf)); }
    return naNil();
}
static naRef f_setprop(naContext c, naRef me, int argc, naRef* a) {
    double t = now_s(); n_set++;
    naRef v = a[argc - 1];
    if (naIsNum(v)) cb_set(cstr(c, a[0]), v.num, NULL); else cb_set(cstr(c, a[0]), 0, cstr(c, v));
    t_cb += now_s() - t;
    return naNum(1);
}
static naRef f_geodinfo(naContext c, naRef me, int argc, naRef* a) {
    double e = 0, t = now_s(); char nm[128]; int solid = 0, k; n_geo++;
    k = cb_geo(naNumValue(a[0]).num, naNumValue(a[1]).num, &e, nm, sizeof nm, &solid);
    t_cb += now_s() - t;
    if (k == 0) return naNil();
    naRef v = naNewVector(c), m = naNil();
    naVec_append(v, naNum(e));
    if (k == 1) {
        naRef h = naNewHash(c), names = naNewVector(c), s = naNewString(c);
        s = naStr_fromdata(s, nm, strlen(nm));
        naVec_append(names, s);
        naAddSym(c, h, "names", names); naAddSym(c, h, "solid", naNum(solid));
        m = h;
    }
    naVec_append(v, m);
    return v;
}
static naRef f_print(naContext c, naRef me, int argc, naRef* a) {
    char buf[4096]; int n = 0; double t = now_s();
    for (int i = 0; i < argc && n < 4000; i++) n += snprintf(buf + n, sizeof buf - n, "%s", cstr(c, a[i]));
    cb_log(buf); t_cb += now_s() - t;
    return naNil();
}
static naRef f_logprint(naContext c, naRef me, int argc, naRef* a) { return f_print(c, me, argc - 1, a + 1); }
static naRef f_systime(naContext c, naRef me, int argc, naRef* a) { return naNum(now_s()); }
static naRef f_vmtime(naContext c, naRef me, int argc, naRef* a) { return naNum(now_s() - t_cb); }

static void report(const char* where) {
    if (!naGetError(ctx)) return;
    char buf[1024]; int n = snprintf(buf, sizeof buf, "NASAL ERROR %s: %s at %s line %d", where, naGetError(ctx), naStr_data(naGetSourceFile(ctx, 0)), naGetLine(ctx, 0));
    for (int i = 1; i < naStackDepth(ctx) && n < 900; i++) n += snprintf(buf + n, sizeof buf - n, "\n   called from %s line %d", naStr_data(naGetSourceFile(ctx, i)), naGetLine(ctx, i));
    cb_log(buf);
}
#define NASTR(s) naStr_fromdata(naNewString(ctx), (s), strlen((s)))
static void addf(const char* n, naCFunction f) { naAddSym(ctx, G, (char*)n, naNewFunc(ctx, naNewCCode(ctx, f))); }

int nb_init(get_cb g, set_cb s, geo_cb e, log_cb l) {
    mach_timebase_info_data_t tb; mach_timebase_info(&tb); tick_s = (double)tb.numer / tb.denom * 1e-9;
    cb_get = g; cb_set = s; cb_geo = e; cb_log = l; t_cb = 0; n_get = n_set = n_geo = 0;
    ctx = naNewContext();
    G = naInit_std(ctx); naSave(ctx, G);
    naAddSym(ctx, G, "math", naInit_math(ctx)); naAddSym(ctx, G, "bits", naInit_bits(ctx)); naAddSym(ctx, G, "io", naInit_io(ctx));
    naAddSym(ctx, G, "utf8", naInit_utf8(ctx)); naAddSym(ctx, G, "unix", naInit_unix(ctx)); naAddSym(ctx, G, "thread", naInit_thread(ctx));
    naAddSym(ctx, G, "string", naInit_string(ctx));
    addf("getprop", f_getprop); addf("setprop", f_setprop); addf("geodinfo", f_geodinfo); addf("print", f_print);
    addf("logprint", f_logprint); addf("systime", f_systime); addf("vmtime", f_vmtime);
    naAddSym(ctx, G, "globals", G);
    return 1;
}
/* the file into module's hash (made if new), as FlightGear's loadModule: its top level with the hash as its locals */
int nb_load(const char* path, const char* module) {
    FILE* f = fopen(path, "rb"); if (!f) { cb_log(path); return 0; }
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    char* buf = malloc(n); fread(buf, 1, n, f); fclose(f);
    int err = 0;
    naRef code = naParseCode(ctx, NASTR(path), 1, buf, n, &err);
    free(buf);
    if (naIsNil(code)) { char b[512]; snprintf(b, sizeof b, "NASAL PARSE ERROR %s line %d: %s", path, err, naGetError(ctx)); cb_log(b); return 0; }
    naRef key = NASTR(module), h;
    if (!naHash_get(G, key, &h)) { h = naNewHash(ctx); naHash_set(G, key, h); }
    naRef fn = naBindFunction(ctx, code, G);
    naCall(ctx, fn, 0, NULL, naNil(), h);
    report(path);
    return naGetError(ctx) == NULL;
}
static naRef lookup(const char* module, const char* name) {
    naRef h, v = naNil();
    if (naHash_get(G, NASTR(module), &h)) naHash_get(h, NASTR(name), &v);
    return v;
}
/* module.name(), its time in the VM alone, s (the callbacks' left out); returns the number it gave, or 0 */
double nb_call(const char* module, const char* name, double* vm_s) {
    naRef fn = lookup(module, name);
    double t0 = now_s(), c0 = t_cb;
    naRef r = naCall(ctx, fn, 0, NULL, naNil(), naNil());
    if (vm_s) *vm_s = now_s() - t0 - (t_cb - c0);
    report(name);
    return naIsNum(r) ? r.num : 0;
}
/* module.name() as a string into out */
int nb_calls(const char* module, const char* name, char* out, int n) {
    naRef fn = lookup(module, name);
    naRef r = naCall(ctx, fn, 0, NULL, naNil(), naNil());
    report(name);
    if (naIsNil(r)) { out[0] = 0; return 0; }
    snprintf(out, n, "%s", cstr(ctx, r));
    return 1;
}
void nb_counts(long* out) { out[0] = n_get; out[1] = n_set; out[2] = n_geo; }

extern double nb_gc_s, nb_gc_max; extern long nb_gc_n;
void nb_gc(double* o) { o[0] = nb_gc_s; o[1] = nb_gc_max; o[2] = nb_gc_n; }
