#!/usr/bin/env python3
"""The proprotors' sound from two banks of PyWopWop, a rotor noise code (its mv22b preset: the rotor data
of v22data.py, the XN12 section of data/sections.py, the blade's frequencies from WRATS, the airplane mode's trims of
this flight model).

PyWopWop computes a rotor's noise from its blades' loads (thickness and loading noise by Farassat's Formulation 1A,
the blade-vortex interaction of a prescribed tip-vortex wake, broadband self-noise and turbulence ingestion) and
synthesises seamless loops per flight condition, each heard outside and in the cockpit, and a FlightGear <fx> section
that cross-fades them with airspeed, rate of descent and height and pitches them with rotors/main/rpm. The helicopter
mode's bank: hover out of and in ground effect, 40, 80 and 120 kt level, a climb, the approach descent with blade slap
and a steep descent, the rotors carrying all the weight with the nacelles vertical, at 104 %. The airplane mode's:
180, 220 and 260 kt level, a climb and a descent, the wing carrying the weight and the rotors pulling the drag with
the nacelles down, at 84 %. Between them (the conversion is not modelled) the helicopter mode's loops fade out and
the airplane mode's in as the nacelles come down from NAC_FULL to NAC_GONE deg (a mixing choice: there is no data
for the sound in between). Each bank's loops share one gain (outside, cockpit): the airplane mode's are put back at
the helicopter mode's, so that the two modes' loudness is the physics'.

The blade slap is the flight model's, not the bank's conditions': where the tip vortices pass the blades is set by
the rotor's advance ratio and the inflow through its tip-path plane, so PyWopWop's map of the slap's level over them
(its tools/slap_map.py, slap_map.json in the bank) becomes data/slap_v22.json, relative to the bank's slap loops at
SLAP_REF's state, and genv22.py looks each rotor's state up in it (/rotors/main/slap-ext and -int). Only SLAP_REF's
slap loops play, at that gain: in a flare, a decelerating turn or a descent at any attitude or nacelle angle.

The cockpit's wind is PyWopWop's too (its wind.py): the windscreen's turbulent boundary layer through the cabin wall,
loops at sea level cross-faded by the equivalent sea level airspeed and scaled for the altitude from FlightGear's
equivalent airspeed and the air's density and temperature.

    python3 rotorsound.py ~/git/PyWopWop/out/mv22b ~/git/PyWopWop/out/mv22b_airplane ~/git/PyWopWop/out/mv22b_wind
        copies the loops the banks' FlightGear sections play to MV-22B/Sounds/pywopwop/, writes the sections into
        Sounds/v22-sound.xml (between BEGIN and END; the original rotor sound, v22_flap.wav, and the earlier wind
        taken out) and the slap table to data/slap_v22.json (then rebuild with genv22.py)

Make the banks with `pywopwop bank mv22b -o out/mv22b`, `pywopwop bank mv22b --mode airplane -o out/mv22b_airplane`
and `pywopwop wind mv22b -o out/mv22b_wind --gain-from out/mv22b`, and the map with `python tools/slap_map.py mv22b
-o out/mv22b/slap_map.json` in the PyWopWop checkout.
"""
import json, os, re, shutil, sys

import numpy as np
import scipy.io.wavfile as wav
from scipy.interpolate import griddata

HERE = os.path.dirname(os.path.abspath(__file__))
SOUNDS = os.path.join(HERE, '..', 'MV-22B', 'Sounds')
SUBDIR = 'pywopwop'
NAC = 'sim/model/v22/nacelle-deg'
NAC_FULL, NAC_GONE = 75.0, 45.0
SLAP_REF = 'bvi'                 # the bank's condition whose slap loops play (the approach descent)
SLAP_VIEW = {'side': ('ext', 'side'), 'cockpit': ('int', 'cockpit')}   # loop's position: the table's, the map's
SLAP_FLOOR_DB = -60.0            # no slap: where the map has none, and outside it
PEAK_DBFS = -0.5                 # a louder slap than the loop's: the loop raised up to this peak, the gain lowered
BEGIN = '    <!-- the proprotors, helicopter and airplane modes: written by fdm/rotorsound.py from PyWopWop banks -->'
END = '    <!-- end of fdm/rotorsound.py\'s proprotors -->'
# The engines' sound at the banks' scale (their loudness against the rotors' and the wind's the physics'): turboshaft
# engines' octave sound power, Levine and DeFelice's (NASA 19790002686, figs. 14-16, after Grande and Brown and Sikorsky
# data): the normalised casing and exhaust spectra (dB) plus 10 log10(shaft hp), re 1e-13 W, the overall level following
# 10 log10(hp) + 108 dB. The engine's power (flight model) sets it: amplitude as the square root of the power
OCT_HZ = [31.5, 63, 125, 250, 500, 1000, 2000, 4000, 8000]
CASING_DB = [93, 98, 99, 99.4, 100, 98, 95, 94, 98.5]          # fig. 15, composite (31.5 Hz: its first point)
EXHAUST_DB = [None, 103, 105, 106, 105, 100, 99, 95, 99]       # fig. 16, composite
ENGINE_LOOPS = (('engine', 'v22_turbine_out.wav', CASING_DB, 1.03), ('exhaust', 'v22_exh.wav', EXHAUST_DB, 1.0))
ENGINE_COCKPIT_M = 7.7          # the nacelle's pivot to the right seat's eye (3-D model)
CABIN_KG_M2 = 8.0               # the cabin wall PyWopWop's cockpit loops come through: its mass law, leakage at -30 dB
SIDES_ENG = (('right', 0, 'engine_right', 7.04), ('left', 1, 'engine_left', -7.04))
# the original aircraft's other sounds (actuators, gear, tyres, doors, flares, rain on the windscreen): no data for their
# levels, so their author's balance with the original engine loops (volume 0.25) is kept and carried to the banks'
# scale as those loops' (the engines' factors above against 0.25, geometric means), outside and in the cockpit. The
# alerts' tones are fdm/alertsound.py's
OLD_SOUNDS = ('wingrotation', 'bladefold', 'flap', 'enginetilt', 'rumble', 'squeal', 'squeal1', 'squeal2', 'gear',
              'gear-lock', 'intense ground contact', 'open door', 'Door closed', 'Steps door', 'Steps closed', 'Bay door',
              'Bay close', 'Flares', 'rain')
OLD_ENGINE_VOLUME = 0.25
OLD_TAG = '      <!-- at the banks\' scale (fdm/rotorsound.py) -->\n'


def rescale_old(t, k):
    """OLD_SOUNDS at the banks' scale: one more <volume> (an expression, last: FlightGear skips property entries after
    one), the outside's factor, the cockpit's in the cockpit"""
    ko = np.sqrt(k['engine', 'outside'] * k['exhaust', 'outside']) / OLD_ENGINE_VOLUME
    kc = np.sqrt(k['engine', 'cockpit'] * k['exhaust', 'cockpit']) / OLD_ENGINE_VOLUME
    vol = ('      <volume><expression><sum><value>%.6g</value><product><property>sim/current-view/internal</property>'
           '<value>%.6g</value></product></sum></expression></volume>\n' % (ko, kc - ko))
    t = re.sub(re.escape(OLD_TAG) + r'.*\n', '', t)
    for name in OLD_SOUNDS:
        i = t.find('<name>%s</name>' % name)
        if i < 0:
            sys.exit('no sound %s in the sound file' % name)
        tag = re.findall(r'\n    <([A-Za-z_-]+)>\n', t[:i])[-1]          # the sound's own element, and its end
        j = t.index('\n    </%s>' % tag, i) + 1
        t = t[:j] + OLD_TAG + vol + t[j:]
    return t, ko, kc

def ramp(prop, a, b):
    """an expression going linearly from 0 at a to 1 at b, clipped to 0..1 (PyWopWop's bank.py _ramp: a <volume>'s own
    factor, offset, min and max can't make a fade in FlightGear)"""
    f = 1.0 / (b - a)
    return ('<clip><sum><product><property>%s</property><value>%.6g</value></product><value>%.6g</value></sum>'
            '<clipMin>0</clipMin><clipMax>1</clipMax></clip>' % (prop, f, -a * f))


def slap_table(bank):
    """the map's slap level (dBA) over (mu, lambda), relative to SLAP_REF's state, as amplitudes on a grid for
    genv22.py; and the largest of them per position"""
    m = json.load(open(os.path.join(bank, 'manifest.json')))
    d = json.load(open(os.path.join(bank, 'slap_map.json')))
    ref = next(c for c in m['conditions'] if c['name'] == SLAP_REF)
    pts = [q for q in d['points'] if 'error' not in q]
    vc = np.array([(q['airspeed_kt'], q['climb_fpm'] / 100.0) for q in pts])
    at_ref = lambda vals: float(griddata(vc, vals, [(ref['airspeed_kt'], ref['climb_fpm'] / 100.0)], 'linear')[0])
    mu = np.array([q['mu'] for q in pts])
    lam = np.array([q['lam'] for q in pts])
    muz = np.array([q['mu_z'] for q in pts])
    mu_g = np.round(np.arange(0.0, mu.max() + 0.0101, 0.01), 3)
    lam_g = np.round(np.arange(np.floor(lam.min() / 0.0025) * 0.0025 - 0.01, lam.max() + 0.0126, 0.0025), 4)
    muz_g = np.round(np.arange(np.floor(muz.min() / 0.005) * 0.005, muz.max() + 0.0051, 0.005), 4)
    out = dict(source='PyWopWop slap map (%s, %s quality, %g s per state) of the bank %s, its %s loops at %g kt, '
                      '%+g ft/min' % (d['aircraft'], d['quality'], d['duration'], os.path.basename(os.path.normpath(bank)),
                                      SLAP_REF, ref['airspeed_kt'], ref['climb_fpm']),
               mu=mu_g.tolist(), lam=lam_g.tolist(), mu_z=muz_g.tolist(),
               thrust_ref_lbf=round(at_ref([q['thrust_N'] for q in pts]) / 4.448222, 1), peak={})
    # PyWopWop's mean induced inflow over the advance ratio and the upflow through the tip-path plane (at the map's
    # thrust): the flight model takes the map's inflow at its own kinematics, since the map's levels come from
    # PyWopWop's wake at PyWopWop's inflow (beyond the map: the nearest)
    MU, MUZ = np.meshgrid(mu_g, muz_g)
    l0 = np.array([q['lambda0'] for q in pts])
    g = griddata(np.c_[mu, muz], l0, (MU, MUZ), 'linear')
    out['lam0'] = np.round(np.where(np.isnan(g), griddata(np.c_[mu, muz], l0, (MU, MUZ), 'nearest'), g), 5).tolist()
    MU, LAM = np.meshgrid(mu_g, lam_g)
    for view, (key, obs) in SLAP_VIEW.items():
        L = np.array([q['levels'][obs]['slap']['dba'] for q in pts])
        L = np.where(L < -100, np.nan, L)                     # no slap at all (the hover)
        rel = L - at_ref(np.nan_to_num(L, nan=L[np.isfinite(L)].min() - 60))
        rel = np.where(np.isfinite(rel), np.maximum(rel, SLAP_FLOOR_DB), SLAP_FLOOR_DB)
        g = griddata(np.c_[mu, lam], rel, (MU, LAM), 'linear')
        g = np.where(np.isnan(g), griddata(np.c_[mu, lam], rel, (MU, LAM), 'nearest'), g)    # past its edge: the nearest
        g[0, :] = g[-1, :] = SLAP_FLOOR_DB                    # beyond the map's inflow: the wake far from the disc
        g[:, 0] = SLAP_FLOOR_DB                               # the hover
        amp = 10 ** (g / 20)
        out[key] = np.round(amp, 4).tolist()
        out['peak'][key] = float(amp.max())
    return out


def section(bank, boost=None, helicopter=None):
    """the bank's <rotor> entries, the loops in SUBDIR. The helicopter mode's: faded out with the nacelles, but for the
    slap: only SLAP_REF's loops, at the flight model's gain (over boost: the loop raised by it). The airplane mode's
    (helicopter: the helicopter mode's bank): faded in with the nacelles, at the helicopter mode's bank gains. The
    files they play"""
    x = open(os.path.join(bank, 'pywopwop-sound.xml'), encoding='utf-8').read()
    body = x[x.index('<fx>') + len('<fx>'):x.index('</fx>')].lstrip('\n').rstrip()
    body = body.replace('<path>Sounds/pywopwop/', '<path>%s/' % SUBDIR)
    if helicopter:
        g_air = json.load(open(os.path.join(bank, 'manifest.json')))['gain_db']
        g_hel = json.load(open(os.path.join(helicopter, 'manifest.json')))['gain_db']
        to_hel = {g: 10 ** ((g_hel[g] - g_air[g]) / 20) for g in g_air}
    fade = ramp(NAC, NAC_FULL, NAC_GONE) if helicopter else ramp(NAC, NAC_GONE, NAC_FULL)
    blocks = []
    for blk in re.findall(r'\t*<rotor>.*?</rotor>', body, re.S):
        name = re.search(r'<name>pww-(.+?)</name>', blk).group(1)
        cond, view, src = re.match(r'(.+)-(side|cockpit)-(rotor|slap|swish|tail|mix)$', name).groups()
        if helicopter:
            g = '<value>%.6g</value>' % to_hel['cockpit' if view == 'cockpit' else 'exterior']
            blk = re.sub(r'(\t+)(<volume><expression><product>\n)', lambda mm: mm.group(0) + mm.group(1) + '\t' + fade + '\n'
                         + mm.group(1) + '\t' + g + '\n', blk)
        elif src == 'slap':
            if cond != SLAP_REF:
                continue
            key = SLAP_VIEW[view][0]
            rpm = re.search(r'\t*(<clip><sum><product><property>rotors/main/rpm</property>.*?</clip>)', blk).group(1)
            gain = ('<clip><product><property>/rotors/main/slap-%s</property><value>%.6g</value></product>'
                    '<clipMin>0</clipMin><clipMax>1</clipMax></clip>' % (key, 1.0 / boost[name]))
            blk = re.sub(r'(\t*)<volume>.*?</volume>', lambda mm: mm.group(1) + '<volume><expression><product>\n'
                         + mm.group(1) + '\t' + gain + '\n' + mm.group(1) + '\t' + rpm + '\n' + mm.group(1)
                         + '</product></expression></volume>', blk, flags=re.S)
        else:
            blk = re.sub(r'(\t+)(<volume><expression><product>\n)', lambda mm: mm.group(0) + mm.group(1) + '\t' + fade + '\n', blk)
        blocks.append(blk)
    body = '\n'.join(blocks).replace('\t', '  ')
    files = re.findall(r'<path>%s/([^<]+)</path>' % SUBDIR, body)
    return body, files


def a_weight_db(f):
    """IEC 61672 A-weighting, dB"""
    f2 = np.asarray(f, float) ** 2
    ra = 12194.0 ** 2 * f2 ** 2 / ((f2 + 20.6 ** 2) * np.sqrt((f2 + 107.7 ** 2) * (f2 + 737.9 ** 2)) * (f2 + 12194.0 ** 2))
    return 20 * np.log10(np.maximum(ra, 1e-30)) + 2.0


def engine_level(spectrum, hp, cockpit):
    """an engine source's A-weighted level (dB re 20 uPa): at 100 m in the free field, or in the cockpit (ENGINE_COCKPIT_M
    away, through the cabin wall's mass law with its leakage floor)"""
    tot = 0.0
    for fc, n in zip(OCT_HZ, spectrum):
        if n is None:
            continue
        pwl = n + 10 * np.log10(hp) - 10.0                          # re 1e-12 W
        r = ENGINE_COCKPIT_M if cockpit else 100.0
        spl = pwl - 10 * np.log10(4 * np.pi * r ** 2)
        if cockpit:
            spl += 10 * np.log10(max(10 ** (-max(20 * np.log10(CABIN_KG_M2 * fc) - 47.0, 0.0) / 10), 1e-3))
        tot += 10 ** ((spl + a_weight_db(fc)) / 10)
    return 10 * np.log10(tot)


def file_level(path):
    """a WAV's A-weighted rms, dB re its full scale"""
    fs, y = wav.read(path)
    y = y.astype(np.float64) / (128.0 if y.dtype == np.uint8 else 32768.0)
    if y.dtype == np.uint8:
        y -= 1.0
    Y = np.fft.rfft(y - y.mean())
    w = 10 ** (a_weight_db(np.maximum(np.fft.rfftfreq(len(y), 1 / fs), 1e-3)) / 10)
    return 10 * np.log10(2 * np.sum(np.abs(Y) ** 2 * w) / len(y) ** 2)


def engine_section(helicopter):
    """the engines' loops (casing: the turbine's whine; exhaust) and start, each engine outside (at its nacelle, its
    level at 100 m) and in the cockpit, at the banks' gains, following the engine's shaft power"""
    sys.path.insert(0, HERE)
    import v22data as V
    gains = json.load(open(os.path.join(helicopter, 'manifest.json')))['gain_db']
    full = {v: 20 * np.log10(1.0 / 10 ** (gains['cockpit' if v == 'cockpit' else 'exterior'] / 20) / 2e-5)
            for v in ('outside', 'cockpit')}
    k = {}
    for name, fn, spec, _ in ENGINE_LOOPS + (('start', 'v22_turbine_strt_out.wav', CASING_DB, None),):
        lf = file_level(os.path.join(SOUNDS, fn))
        for view in ('outside', 'cockpit'):
            k[name, view] = 10 ** ((engine_level(spec, V.P_MRP, view == 'cockpit') - full[view] - lf) / 20)
            if k[name, view] > 1:
                sys.exit('%s would need a volume of %.2f in the %s: louder files needed' % (fn, k[name, view], view))
    out = []
    for side, n, spool, y in SIDES_ENG:
        for view in ('outside', 'cockpit'):
            cond = ('<property>sim/current-view/internal</property>' if view == 'cockpit'
                    else '<not><property>sim/current-view/internal</property></not>')
            where = ('' if view == 'cockpit' else '      <position><x>-1.88</x><y>%.2f</y><z>0.52</z></position>\n'
                     '      <reference-dist>100</reference-dist>\n      <max-dist>4000</max-dist>\n' % -y)
            on = '<and>%s<greater-than><property>/sim/model/v22/%s</property><value>0.01</value></greater-than></and>' % (cond, spool)
            for name, fn, _, pf in ENGINE_LOOPS:
                power = ('<sqrt><clip><div><property>/fdm/jsbsim/eng[%d]/power-hp</property><value>%.6g</value></div>'
                         '<clipMin>0</clipMin><clipMax>2</clipMax></clip></sqrt>' % (n, V.P_MRP))
                out.append('    <engine>\n      <name>%s-%s-%s</name>\n      <mode>looped</mode>\n      <path>%s</path>\n'
                           '      <condition>%s</condition>\n      <volume><expression><product><value>%.6g</value>%s'
                           '</product></expression></volume>\n      <pitch><property>/sim/model/v22/%s</property>'
                           '<factor>%g</factor><max>2.0</max></pitch>\n%s    </engine>' % (
                               side, name, view, fn, on, k[name, view], power, spool, pf, where))
        # the start: one sound for both views (a sound played once starts again when its condition comes true: with the
        # view in it, at every change of view), its volume the view's; at the nacelle, its level the 100 m one outside.
        # Only while a started engine spools up (drive.py: 0 to 1 over 30 s; the recording is 15 s): not at an in-air
        # start, where it runs already, nor at a shutdown
        ko, kc = k['start', 'outside'], k['start', 'cockpit']
        out.append('    <engine>\n      <name>%s-start</name>\n      <mode>once</mode>\n      <path>v22_turbine_strt_out.wav</path>\n'
                   '      <condition><and><greater-than><property>/fdm/jsbsim/eng[%d]/running</property><value>0.5</value></greater-than>'
                   '<greater-than><property>/sim/model/v22/%s</property><value>0.01</value></greater-than>'
                   '<less-than><property>/sim/model/v22/%s</property><value>0.99</value></less-than></and></condition>\n'
                   '      <volume><expression><sum><value>%.6g</value><product><property>sim/current-view/internal</property>'
                   '<value>%.6g</value></product></sum></expression></volume>\n'
                   '      <position><x>-1.88</x><y>%.2f</y><z>0.52</z></position>\n      <reference-dist>100</reference-dist>\n'
                   '      <max-dist>4000</max-dist>\n    </engine>' % (side, n, spool, spool, ko, kc - ko, -y))
    note = ('    <!-- the engines: casing and exhaust at Levine and DeFelice\'s turboshaft levels (fdm/rotorsound.py: %s) -->'
            % ', '.join('%s %s %.0f dBA' % (nm, v, engine_level(sp, V.P_MRP, v == 'cockpit'))
                        for nm, _, sp, _ in ENGINE_LOOPS for v in ('outside', 'cockpit')))
    return note + '\n' + '\n'.join(out), k


def wind_section(wind, helicopter):
    """the wind bank's <wind> entries (the cockpit's wind, wind.py of PyWopWop), the loops in SUBDIR; its gain must be
    the helicopter mode's bank's cockpit gain (pywopwop wind --gain-from), so that the wind against the rotors is the
    physics'"""
    g_w = json.load(open(os.path.join(wind, 'manifest.json')))['gain_db']['cockpit']
    g_h = json.load(open(os.path.join(helicopter, 'manifest.json')))['gain_db']['cockpit']
    if abs(g_w - g_h) > 0.01:
        sys.exit('the wind bank\'s cockpit gain %+.2f dB is not the helicopter bank\'s %+.2f: make it with --gain-from' % (g_w, g_h))
    x = open(os.path.join(wind, 'pywopwop-wind.xml'), encoding='utf-8').read()
    body = x[x.index('<fx>') + len('<fx>'):x.index('</fx>')].lstrip('\n').rstrip()
    body = body.replace('<path>Sounds/pywopwop/', '<path>%s/' % SUBDIR).replace('\t', '  ')
    return body, re.findall(r'<path>%s/([^<]+)</path>' % SUBDIR, body)


def install(bank, airplane, wind):
    tab = slap_table(bank)
    json.dump(tab, open(os.path.join(HERE, 'data', 'slap_v22.json'), 'w'))
    # the slap loops raised where the map has louder slap than theirs, up to PEAK_DBFS (FlightGear's volume stops at 1)
    boost, raised = {}, {}
    for view, (key, _) in SLAP_VIEW.items():
        fn = '%s_%s_slap.wav' % (SLAP_REF, view)
        fs, y = wav.read(os.path.join(bank, fn))
        y = y.astype(np.float64) / 32768.0
        b = max(1.0, min(tab['peak'][key], 10 ** (PEAK_DBFS / 20) / np.abs(y).max()))
        boost['%s-%s-slap' % (SLAP_REF, view)] = b
        raised[fn] = (fs, np.clip(np.round(y * b * 32767), -32768, 32767).astype(np.int16), b, tab['peak'][key])
    body, files = section(bank, boost)
    body_a, files_a = section(airplane, helicopter=bank)
    body_w, files_w = wind_section(wind, bank)
    body_e, k_eng = engine_section(bank)
    out = os.path.join(SOUNDS, SUBDIR)
    if os.path.isdir(out):
        shutil.rmtree(out)
    os.makedirs(out)
    for src, fns in ((bank, files), (airplane, files_a), (wind, files_w)):
        for fn in fns:
            if fn in raised:
                wav.write(os.path.join(out, fn), raised[fn][0], raised[fn][1])
            else:
                shutil.copy(os.path.join(src, fn), os.path.join(out, fn))
    body, files = body + '\n' + body_a + '\n' + body_w + '\n' + body_e, files + files_a + files_w
    p = os.path.join(SOUNDS, 'v22-sound.xml')
    t = open(p, encoding='latin-1').read()
    block = BEGIN + '\n' + body + '\n' + END + '\n'
    if BEGIN in t:
        t = t[:t.index(BEGIN)] + block + t[t.index(END) + len(END) + 1:]
    else:
        at = t.index('    <!-- outside -->\n    <!-- rotor -->')
        t = t[:at] + block + '\n' + t[at:]
    # the original rotor sound (v22_flap.wav): the airplane mode's bank in its place; the earlier wind (a noise loop
    # of fgdata's wind sample with the Bo 105's curves): the wind bank's
    for rx in [r'    <!-- outside -->\n    <!-- rotor -->\n    <rotor>\n      <name>rotor</name>.*?</rotor>\n',
               r'    <!-- wind: the Bo 105\'s .*?</wind>\n\n'] + [
               r'    <engine>\n      <name>%s-%s</name>.*?</engine>\n' % (s, n) for s in ('left', 'right')
               for n in ('engine-start', 'engine-loop', 'exhaust-loop')]:
        m = re.search(rx, t, re.S)
        if m:
            t = t[:m.start()] + t[m.end():]
    t, ko, kc = rescale_old(t, k_eng)
    open(p, 'w', encoding='latin-1').write(t)
    mb = sum(os.path.getsize(os.path.join(out, fn)) for fn in files) / 2 ** 20
    print('%d loops (%.0f MB) in %s, %d sounds in %s' % (len(files), mb, out, body.count('<rotor>') + body.count('<wind>')
                                                       + body.count('<engine>'), p))
    print('engines\' volume factors at %.0f shp: %s' % (6150, ', '.join('%s %s %.3f' % (a, b, v) for (a, b), v in k_eng.items())))
    print('the original sounds at the banks\' scale: x %.3f (%+.1f dB) outside, x %.3f (%+.1f dB) in the cockpit' % (
        ko, 20 * np.log10(ko), kc, 20 * np.log10(kc)))
    for fn, (_, _, b, pk) in raised.items():
        print('%s: the map\'s loudest slap %+.1f dB over the loop\'s, the loop raised %+.1f dB%s' % (
            fn, 20 * np.log10(pk), 20 * np.log10(b), '' if b >= pk else ' (the rest clipped at full volume)'))
    print('data/slap_v22.json: %d x %d (lambda x mu), thrust reference %.0f lbf' % (len(tab['lam']), len(tab['mu']), tab['thrust_ref_lbf']))


if __name__ == '__main__':
    install(*[os.path.expanduser(a) for a in sys.argv[1:4]])
