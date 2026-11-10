#!/usr/bin/env python3
"""The cockpit's noise in Nasal, for the crew alerts' tones' automatic volume: MV-22B/Nasal/noise.nas, written from the
installed Sounds/v22-sound.xml (run it again after any change to the sound file, and after alertsound.py).

noise.level() is the A-weighted level of the bank sounds the cockpit plays at that moment, dB re full scale at volume 1 (the
sound file's scale: a full-scale sample is FS_SPL dB re 20 uPa, rotorsound.py's scale_pass): every cockpit loop's condition,
volume and pitch as SimGear reads them (soundcheck.py's), translated to Nasal, each file's A-weighted mean square by the pitch it
plays at (its spectrum moved by the pitch). The touchdowns' one-shots are left out, and the original sounds (doors, flaps).

Once a second noise.nas sets the tones' gain (/sim/model/v22/sound/tone-gain; alertsound.py's entries multiply by it) so that
their onset stands TONE_OVER_DB over the noise as played: the FAA's flight deck handbook (DOT/FAA/AM-24/23, ch. 4, from
AC 25.1322-1 appendix 2) asks an aural alert's onset 20 to 30 dB over the ambient, and automatic volume control. The noise is
taken no lower than AMBIENT_SPL, a talker's normal vocal effort (ANSI S3.5-1997: 62.35 dB re 20 uPa), the crew's own speech:
no measurement of the MV-22B's cockpit with the engines off (fans, air conditioning, its APU) was found, and without a floor
the tones would fall silent with the engines. Each by what FlightGear plays, measured on its own output (MEASURED): the bank
sounds' output against level() (the listener's centring) and the caution chime's at gain 1. The aircraft's sounds go with the
effects volume, the tones with the avionics volume (both with the master volume), so those are divided out. A tone that would
need more than gain 1 plays at 1.

The voices (alerts.nas, rwr.nas) are FlightGear's text-to-speech, which plays every message at volume 1: it reads no volume
(flitevoice.cxx; only an external Festival server would), and only the master volume scales it. So they are left at that.

    python3 cockpitnoise.py
"""
import json, os, re, sys

import numpy as np
import scipy.io.wavfile as wav
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
SOUNDS = os.path.join(HERE, '..', 'MV-22B', 'Sounds')
OUT = os.path.join(HERE, '..', 'MV-22B', 'Nasal', 'noise.nas')
sys.path.insert(0, HERE)
import rotorsound as R

INTERNAL = 'sim/current-view/internal'
P_LOG2 = np.arange(-5.0, 1.0001, 0.25)              # the pitches a file's level is tabled at, log2: 1/32 to 2
EDGES = np.geomspace(5.0, 24000.0, 241)
CTR = np.sqrt(EDGES[:-1] * EDGES[1:])
TONE_OVER_DB = 20.0
AMBIENT_SPL = 62.35
# FlightGear's own output, recorded (OpenAL's wave backend) on 2026-11-11 with every sound slider at 1, A-weighted, dB re the
# output's full scale: the ground idle at 104 % (15 s) less level() at its logged state; the caution chime's loudest 125 ms at
# gain 1 and its factor then, 1 (the voices: the RWR's -33.2 over its message, the crew alerts' -27.2, at any volume)
MEASURED = dict(air_out_db=-3.34, caution_out_db=-10.0)


def a_weight(f):
    f2 = f * f
    ra = 12194.0 ** 2 * f2 * f2 / ((f2 + 20.6 ** 2) * np.sqrt((f2 + 107.7 ** 2) * (f2 + 737.9 ** 2)) * (f2 + 12194.0 ** 2))
    return ra ** 2 * 10 ** (2.0 / 10)


def pitch_table(path):
    """the file's A-weighted mean square, dB re its full scale, at the pitches 2^P_LOG2"""
    fs, y = wav.read(path)
    y = y.astype(float) / 32768.0
    if y.ndim > 1:
        y = y.mean(axis=1)
    P = np.abs(np.fft.rfft(y)) ** 2
    f = np.fft.rfftfreq(len(y), 1 / fs)
    P = P / P.sum() * np.mean(y ** 2)
    idx = np.digitize(f, EDGES) - 1
    ok = (idx >= 0) & (idx < len(CTR))
    sp = np.bincount(idx[ok], P[ok], minlength=len(CTR))
    return [round(float(10 * np.log10(max(np.sum(sp * a_weight(CTR * 2 ** k)), 1e-30))), 2) for k in P_LOG2]


class Nasal:
    """a cockpit entry's condition, volume and pitch as Nasal expressions; properties read once into p[i]"""
    def __init__(self):
        self.props = []

    def prop(self, e):
        k = e.text.strip().lstrip('/')
        if k == INTERNAL:
            return '1'
        if k not in self.props:
            self.props.append(k)
        return 'p[%d]' % self.props.index(k)

    def expr(self, e):
        t = e.tag
        k = [c for c in e if c.tag not in ('clipMin', 'clipMax')]
        if t == 'value':
            return repr(float(e.text))
        if t == 'property':
            return self.prop(e)
        if t == 'sum':
            return '(' + ' + '.join(self.expr(c) for c in k) + ')'
        if t == 'difference':
            return '(' + ' - '.join(self.expr(c) for c in k) + ')'
        if t == 'product':
            return '(' + ' * '.join(self.expr(c) for c in k) + ')'
        if t == 'div':
            return '_div(%s, %s)' % (self.expr(k[0]), self.expr(k[1]))
        if t == 'sqrt':
            return 'math.sqrt(_pos(%s))' % self.expr(k[0])
        if t == 'abs':
            return 'math.abs(%s)' % self.expr(k[0])
        if t == 'pow':
            return 'math.pow(_pos(%s), %s)' % (self.expr(k[0]), self.expr(k[1]))
        if t == 'max':
            return 'math.max(%s, %s)' % (self.expr(k[0]), self.expr(k[1]))
        if t == 'clip':
            return '_clip(%s, %r, %r)' % (self.expr(k[0]), float(e.findtext('clipMin')), float(e.findtext('clipMax')))
        raise ValueError('expression: ' + t)

    def cond(self, e):
        t = e.tag
        if t in ('and', 'or'):
            return '(' + (' %s ' % t).join(self.cond(c) for c in e) + ')'
        if t == 'not':
            return '!' + self.cond(list(e)[0])
        if t == 'property':
            v = self.prop(e)
            return '1' if v == '1' else '(%s > 0.5)' % v
        a, b = list(e)
        op = {'greater-than': '>', 'less-than': '<'}[t]
        return '(%s %s %s)' % (self.expr(a), op, self.expr(b))

    def pitch(self, x):
        """SimGear's: each block's factor x property within <min> and <max> (a max of 0: none) multiplied, the offsets (1.0 where
        none is given; an expression block adds none) added, the sum held within 0.01 and 2.0 (soundcheck.sg_pitch)"""
        terms, off = [], 0.0
        for pt in x.findall('pitch'):
            ex = pt.find('expression')
            if ex is not None:
                terms.append(self.expr(list(ex)[0]))
                continue
            q = '(%s * %r)' % (self.prop(pt.find('property')), float(pt.findtext('factor') or 1.0))
            lo, hi = float(pt.findtext('min') or 0.0), float(pt.findtext('max') or 0.0)
            terms.append('_clip(%s, %r, %r)' % (q, lo, hi) if hi else 'math.max(%s, %r)' % (q, lo))
            off += float(pt.findtext('offset') or 1.0)
        if not terms:
            return '1'
        return '_clip(%r + %s, 0.01, 2.0)' % (off, ' * '.join(terms))


def translate():
    """the cockpit loops' lines of level(), their files and the files' pitch tables"""
    t = open(os.path.join(SOUNDS, 'v22-sound.xml'), encoding='latin-1').read()
    i, j = t.index(R.BEGIN), t.index(R.END)
    E = [x for x in ET.fromstring('<fx>' + t[i + len(R.BEGIN):j] + '</fx>') if (x.findtext('name') or '').startswith('pww-')]
    N = Nasal()
    files, body, skipped = [], [], 0
    for x in E:
        blk = ET.tostring(x, encoding='unicode')
        if R.view_of(blk) != 'cockpit' or x.tag == 'touchdown':
            skipped += 1
            continue
        fn = x.findtext('path')
        if fn not in files:
            files.append(fn)
        c = N.cond(list(x.find('condition'))[0])
        v = N.expr(list(x.find('volume').find('expression'))[0])
        body.append((c, v, files.index(fn), N.pitch(x), x.findtext('name')))
    tabs = [pitch_table(os.path.join(SOUNDS, fn)) for fn in files]
    return N, body, files, tabs, skipped


def main():
    N, body, files, tabs, skipped = translate()
    import alertsound as AS
    fs_spl = 20 * np.log10(1 / 2e-5) - R.SCALE_GAINS['cockpit']
    m = MEASURED
    on = AS.onsets()
    tone_out = m['caution_out_db'] - on['caution'] + min(on.values())     # every tone's onset at gain 1, its factor brought it to
    head = ('# The cockpit\'s noise, for the crew alerts\' tones\' automatic volume (Sounds/v22-sound.xml\'s alert entries). Written by\n'
            '# fdm/cockpitnoise.py from Sounds/v22-sound.xml: do not edit. level(): the A-weighted level of the bank sounds the cockpit\n'
            '# plays now, dB re full scale at volume 1 (a full-scale sample is FS_SPL dB re 20 uPa), %d loops of %d files\n\n' % (len(body), len(files)))
    nas = [head,
           'var FS_SPL = %.2f;\n' % fs_spl,
           'var TONE_OVER_DB = %g;       # an aural alert\'s onset over the noise (AC 25.1322-1 appendix 2, 2.d(2)(b): 20 to 30 dB)\n' % TONE_OVER_DB,
           'var AMBIENT_DB = %.2f;      # the noise taken no lower: a talker\'s normal vocal effort, re full scale (ANSI S3.5-1997: %.2f dB SPL)\n'
           % (AMBIENT_SPL - fs_spl, AMBIENT_SPL),
           'var AIR_OUT_DB = %.2f;        # FlightGear\'s output of the bank sounds against level(), all sliders at 1 (measured)\n' % m['air_out_db'],
           'var TONE_OUT_DB = %.2f;      # a tone\'s onset at gain 1 in the output (the caution chime measured, %.2f; each tone brought to %.2f)\n\n'
           % (tone_out, m['caution_out_db'], min(on.values())),
           'var P0 = %g; var DP = %g; var NP = %d;\n' % (P_LOG2[0], P_LOG2[1] - P_LOG2[0], len(P_LOG2)),
           'var PROPS = [\n' + ',\n'.join('    "/%s"' % k for k in N.props) + '\n];\n\n',
           'var _clip = func(x, lo, hi) { return x < lo ? lo : (x > hi ? hi : x); }\n'
           'var _pos = func(x) { return x > 0 ? x : 0; }\n'
           'var _div = func(a, b) { return b != 0 ? a / b : 0; }\n'
           '# a file\'s A-weighted mean square at the pitch it plays at (a, its table in dB, by log2 of the pitch)\n'
           'var _a = func(a, pitch) {\n'
           '    var x = (math.ln(pitch) / math.ln(2) - P0) / DP;\n'
           '    var k = int(_clip(x, 0, NP - 2));\n'
           '    var f = _clip(x - k, 0, 1);\n'
           '    return math.pow(10, 0.1 * (a[k] + f * (a[k + 1] - a[k])));\n'
           '}\n\n'
           '# each loop\'s mean square: a function of its own (one for all of them is past what Nasal\'s bytecode can hold)\n'
           'var E = [];\n',
           ''.join('append(E, func(p) { if (%s) { var v = _clip(%s, 0, 1); return v * v * _a([%s], %s); } return 0; });   # %s\n'
                   % (c, v, ', '.join('%g' % q for q in tabs[f]), pt, nm) for c, v, f, pt, nm in body),
           '\nvar level = func {\n'
           '    var p = [];\n'
           '    foreach (var k; PROPS) append(p, num(getprop(k)) or 0);\n'
           '    var ms = 0;\n'
           '    foreach (var e; E) ms += e(p);\n'
           '    return ms > 1e-30 ? 10 * math.ln(ms) / math.ln(10) : -300;\n'
           '}\n\n'
           '# the tones\' gain that puts their onset TONE_OVER_DB over the cockpit\'s noise as played (the effects volume; the noise no\n'
           '# lower than AMBIENT_DB), by the avionics volume they play at\n'
           'var _vol = func(p) { var k = getprop(p); return k == nil ? 1 : _clip(k, 1e-3, 1); }\n'
           'var tone_gain = func(n) {\n'
           '    var db = math.max(n, AMBIENT_DB) + AIR_OUT_DB + 20 * math.ln(_vol("/sim/sound/effects/volume")) / math.ln(10) + TONE_OVER_DB;\n'
           '    return _clip(math.pow(10, (db - TONE_OUT_DB) / 20) / _vol("/sim/sound/avionics/volume"), 0, 1);\n'
           '}\n\n'
           '# once a second: the noise and the tones\' gain\n'
           'var OUT = "/sim/model/v22/sound/";\n'
           'setprop(OUT ~ "tone-gain", 1);         # until the first update: the tones at their own level, not silent\n'
           'var update = func {\n'
           '    var n = level();\n'
           '    setprop(OUT ~ "cockpit-noise-db", n);\n'
           '    setprop(OUT ~ "tone-gain", tone_gain(n));\n'
           '}\n'
           'var timer = maketimer(1.0, update);\n'
           'setlistener("/sim/signals/fdm-initialized", func { update(); timer.start() }, 1, 0);\n']
    open(OUT, 'w').write(''.join(nas))
    print('%s: %d cockpit loops, %d files, %d properties (%d left out); %.0f kB' % (
        OUT, len(body), len(files), len(N.props), skipped, os.path.getsize(OUT) / 1024))
    return N, body, files, tabs


def check(states_json):
    """the translation against soundcheck's own reading of the file (rotorsound's mixer): level() in Python on trimmed states"""
    import math as _m
    N, body, files, tabs, _ = translate()

    class M:
        sqrt, pow, ln = staticmethod(_m.sqrt), staticmethod(_m.pow), staticmethod(_m.log)
        abs, max = staticmethod(abs), staticmethod(max)
    env = dict(math=M, _clip=lambda x, lo, hi: lo if x < lo else (hi if x > hi else x), _pos=lambda x: max(x, 0.0),
               _div=lambda a, b: a / b if b != 0 else 0.0)

    def _a(i, pitch):
        x = (_m.log(pitch, 2) - P_LOG2[0]) / (P_LOG2[1] - P_LOG2[0])
        k = int(min(max(x, 0), len(P_LOG2) - 2)); f = min(max(x - k, 0), 1)
        return 10 ** (0.1 * (tabs[i][k] + f * (tabs[i][k + 1] - tabs[i][k])))
    env['_a'] = _a
    lines = [(compile(c.replace('!', ' not '), 'c', 'eval'), compile(v, 'v', 'eval'), int(f), compile(pt, 'p', 'eval'), nm)
             for c, v, f, pt, nm in body]
    return env, lines, N.props


if __name__ == '__main__':
    main()

