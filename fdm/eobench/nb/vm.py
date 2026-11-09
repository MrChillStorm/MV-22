"""the embedded Nasal VM: loads FlightGear's own math, string, geo over the bench's prelude"""
import ctypes as C, os
HERE = os.path.dirname(os.path.abspath(__file__))
FGDATA = os.path.expanduser('~/git/fgdata/Nasal/')
GET = C.CFUNCTYPE(C.c_int, C.c_char_p, C.POINTER(C.c_double), C.c_void_p, C.c_int)
SET = C.CFUNCTYPE(None, C.c_char_p, C.c_double, C.c_char_p)
GEO = C.CFUNCTYPE(C.c_int, C.c_double, C.c_double, C.POINTER(C.c_double), C.c_void_p, C.c_int, C.POINTER(C.c_int))
LOG = C.CFUNCTYPE(None, C.c_char_p)
lib = C.CDLL(os.path.join(HERE, 'libnb.dylib'))
lib.nb_call.restype = C.c_double
lib.nb_call.argtypes = [C.c_char_p, C.c_char_p, C.POINTER(C.c_double)]


class VM:
    def __init__(s, get, setp, geo, log):
        def g(p, x, buf, n):
            v = get(p.decode())
            if v is None:
                return 0
            if isinstance(v, str):
                C.memmove(buf, v.encode() + b'\0', len(v) + 1); return 2
            x[0] = float(v); return 1
        def st(p, x, t):
            setp(p.decode(), x if t is None else t.decode())
        def e(lat, lon, el, nm, n, solid):
            r = geo(lat, lon)
            if r is None:
                return 0
            el[0] = r[0]
            if r[1] is None:
                return 2
            b = r[1]['names'][0].encode() + b'\0'
            C.memmove(nm, b, len(b)); solid[0] = int(r[1]['solid']); return 1
        s.cbs = (GET(g), SET(st), GEO(e), LOG(lambda m: log(m.decode())))
        lib.nb_init(*s.cbs)
        s.load(os.path.join(HERE, 'prelude.nas'), 'prelude_')
        s.call('prelude_', 'nil') if False else None
        for m in ('math', 'string', 'geo'):
            s.load(FGDATA + m + '.nas', m)

    def load(s, path, module):
        if not lib.nb_load(path.encode(), module.encode()):
            raise RuntimeError('load failed: ' + path)

    def call(s, module, name):
        t = C.c_double()
        r = lib.nb_call(module.encode(), name.encode(), C.byref(t))
        return r, t.value

    def calls(s, module, name):
        b = C.create_string_buffer(8192)
        return b.value.decode() if lib.nb_calls(module.encode(), name.encode(), b, 8192) else None

    def counts(s):
        a = (C.c_long * 3)()
        lib.nb_counts(a)
        return list(a)
