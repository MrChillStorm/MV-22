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

The blade slap's level is the flight model's: where the tip vortices pass the blades is set by the rotor's advance ratio
and the inflow through its tip-path plane, so PyWopWop's map of the slap's level over them (its tools/slap_map.py,
slap_map.json in the bank) becomes data/slap_v22.json, relative to the bank's slap loops at SLAP_REF's state, and genv22.py
looks each rotor's state up in it (/rotors/main/slap-ext and -int): in a flare, a decelerating turn or a descent at any
attitude or nacelle angle. Its sound is the bank's: every condition's slap loops, cross-faded by airspeed and vertical
speed as PyWopWop's own section has them, each over its own state's level in the map, so a trimmed state plays the bank's
loop as rendered.

The cockpit's wind is PyWopWop's too (its wind.py): the windscreen's turbulent boundary layer through the cabin wall,
loops at sea level cross-faded by the equivalent sea level airspeed and scaled for the altitude from FlightGear's
equivalent airspeed and the air's density and temperature.

The engines' are PyWopWop's as well (its engine.py, from the AE 1107C's cycle and the flight model's Ng and fuel flow:
compressor, core, power turbine and starter, lit and cold, heard outside and in the cockpit, and the gearbox), one bank
with the nacelles up and one with them down, faded into each other with the nacelles as the rotors' are, at the same
gains: their loudness against the rotors and the wind is the physics'.

The tyres' are PyWopWop's too (its tyre.py, at the rotor bank's gains): each leg's two tyres rolling, by its own rolling speed
and load (/gear/gear[n]/rollspeed-ms, compression-norm), and its touchdown, a squeal and a strut's thump, once, by the sink
rate and slip the flight model latches as its weight comes on (genv22.py's 'touchdown' channel). They replace the original
rumble and squeals.

    python3 rotorsound.py ~/git/PyWopWop/out/mv22b ~/git/PyWopWop/out/mv22b_airplane AIRPLANE_LOAD HELICOPTER_LOAD WIND ENGINES \\
        ENGINES_AIRPLANE TYRES TOUCHDOWN
        copies the loops the banks' FlightGear sections play to MV-22B/Sounds/pywopwop/, writes the sections into
        Sounds/v22-sound.xml (between BEGIN and END; the original rotor sound, v22_flap.wav, the earlier wind and the
        original tyres taken out) and the slap table to data/slap_v22.json (then rebuild with genv22.py)

    python3 rotorsound.py --pitch
        only the installed section's pitch blocks rewritten as the install writes them (pitches()), nothing else: for when the
        banks it was made from are no longer on disk. Then check it with soundcheck.py

    python3 rotorsound.py --np-fade
        only the installed section's power turbine loops given their level by the turbine's speed (np_fade(), NP_EXPONENT)
        and their pitch blocks as the install writes them, nothing else: likewise without the banks

    python3 rotorsound.py --scale
        only the installed loops and the original sounds put on the cockpit's scale (scale_pass(), rescale_old() at SCALE_GAINS),
        nothing else: likewise without the banks

    python3 rotorsound.py --runup RUNUP
        the installed section given the rotors' run-up on the ground (runup_section()) from RUNUP, made with `pywopwop bank mv22b
        --mode helicopter_load --runup --conditions he_ige_1 runup_20 ... runup_100 -o RUNUP`: its he_ige_1 sets the installed
        loops' scale, as their banks are gone. The grid's in-ground-effect loops as grid_section writes them now, the pitch blocks as
        the install does; the run-up's loops copied to MV-22B/Sounds/pywopwop/. A full install takes the run-up from the
        helicopter bank instead (`pywopwop bank mv22b --runup --update -o out/mv22b`)

Make the banks with `pywopwop bank mv22b -o out/mv22b` (its slap and gains), `pywopwop bank mv22b --mode airplane_load -o
AIRPLANE_LOAD`, `pywopwop bank mv22b --mode helicopter_load -o HELICOPTER_LOAD`
and `pywopwop wind mv22b -o out/mv22b_wind --gain-from out/mv22b`, the map with `python tools/slap_map.py mv22b
-o out/mv22b/slap_map.json --climbs=-4000:1500:250` (down to 4,000 ft/min: past the default 3,000 the steep descents' inflow
leaves the map, and the slap with it; about a day at 4 workers, `--append` resumes), and the engines' with `pywopwop engine-bank mv22b -o ENGINES --like out/mv22b --duration 5`
and `pywopwop engine-bank mv22b -o ENGINES_AIRPLANE --nacelle 0 --like out/mv22b_airplane --duration 5` (5 s: at the
default 15 s the two come to 660 MB; but the gearbox's loop, the cockpit's loudest sound, is heard repeating at 5 s: its two
files from the same commands at `--duration 15`, put in place of the 5 s ones) in the PyWopWop checkout. The tyres' (TYRES, TOUCHDOWN) with
tyre.make_tyre_bank and make_touchdown_bank from fdm/v22data.py's TYRES, the struts and contacts of genv22.struts() and the
rotor bank's gains.
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
# The rotors by speed and load, from the flight model: a rotor's sound goes with what it is carrying as much as with its speed, so
# each mode has a grid of loops (pywopwop bank --mode airplane_load, helicopter_load) blended live: airplane mode by the two rotors'
# mean thrust, lbf, from windmilling (the flight model's -470) to high power; helicopter mode by their thrust over the weight, the
# bank's load factors at HOVER_T_W, what a hover takes with the wing's download. Hover is the grid's zero speed; in ground effect its
# own row, from one rotor radius down. The slap is apart, set by the flight model
# (genv22.py's outputs: properties, since a loop's condition can compare properties only. Each grid loop plays only inside its
# window of nacelle angle, airspeed and load: a looped sound holds one of FlightGear's 128 sources while its condition is true,
# however silent, and all 31 a view at once left little for the rest)
THRUST_P, T_OVER_W_P = '/rotors/main/thrust-lbf', '/rotors/main/thrust-over-weight'
THRUST = '<property>%s</property>' % THRUST_P
T_OVER_W = '<property>%s</property>' % T_OVER_W_P
HOVER_T_W = 1.10
AGL_P, SPD_P, VS_P = 'position/altitude-agl-ft', 'velocities/airspeed-kt', 'velocities/vertical-speed-fps'
AGL = '<property>%s</property>' % AGL_P
# Each loop by altitude: speed and load are PyWopWop's, the loops faded between; the air's density is an expression here, a power of
# the density ratio for each loop and place (soundalt.py: PyWopWop's own levels at the same calibrated speed and load, over the air
# that loop can be flown in), so no more loops are needed for it. Not one power for all: a light rotor falls with the density, a rotor
# pulling hard at speed gets louder with height, at the same calibrated speed its true speed and tip Mach number rising
SIGMA = '<div><property>/environment/density-slugft3</property><value>0.00237689</value></div>'
ALT_LAW = json.load(open(os.path.join(HERE, 'data', 'sound_altitude_v22.json')))
IGE_FT = (28.6, 9.5)            # all out of ground effect, all in it
OLD_SOUNDS = ('wingrotation', 'bladefold', 'flap', 'enginetilt', 'gear',
              'gear-lock', 'intense ground contact', 'open door', 'Door closed', 'Steps door', 'Steps closed', 'Bay door',
              'Bay close', 'Flares', 'rain')
OLD_ENGINE_VOLUME = 0.25
# the original tyres (a rumble while any wheel is down, three squeals by the sink rate and airspeed): the tyre banks' in their place
OLD_TYRES = ('rumble', 'squeal', 'squeal1', 'squeal2')
# The power turbine's loops are rendered at 100 % of its speed and play by Ng, pitched by its own speed (the rotors'):
# their level goes with that speed too, Smith and Bushell's turbine noise (ANOPP section 8.9, eq. 4: its power the tip
# Mach number cubed times the gas generator's flow), so the amplitude with (NR / 100)^1.5, off where that is 60 dB down.
# Otherwise an engine running with the rotor brake on (drive.py: cutoffs opened without the autostart, which releases it
# first) plays them in full while the turbine stands, and a start plays them in full while the rotors are still coming
# up. PyWopWop's engine writer has it (engine.py's NP_EXPONENT); banks written before it get it here
NP_EXPONENT = 1.5
NP_GATE_PCT = 100.0 * 10.0 ** (-60.0 / (20.0 * NP_EXPONENT))       # 1 %
OLD_TAG = '      <!-- at the banks\' scale (fdm/rotorsound.py) -->\n'
PITCH_MIN, PITCH_MAX = 0.01, 2.0    # the least and the most pitch SimGear plays (sample.hxx's set_pitch holds it there)
ENTRY_TAGS = ('rotor', 'engine', 'wind', 'tyre', 'touchdown')     # the banks' entries in the section


def gates(blk, prop):
    """the least and the most of prop an entry's condition lets it play at (None: no bound). Only an <and> of comparisons
    bounds it: anything under a <not> or an <or> is passed over"""
    c = re.search(r'<condition>(.*?)</condition>', blk, re.S)
    c = re.sub(r'<not>.*?</not>', '', c.group(1), flags=re.S) if c else ''
    if '<or>' in c:
        return None, None
    q = r'<property>/?%s</property><value>([^<]+)</value>' % re.escape(prop.lstrip('/'))
    lo = [float(v) for v in re.findall('<greater-than>%s</greater-than>' % q, c)]
    hi = [float(v) for v in re.findall('<less-than>%s</less-than>' % q, c)]
    return (max(lo) if lo else None), (min(hi) if hi else None)


def pitches(body):
    """the banks' pitch blocks as SimGear has to have them. SimGear plays a <pitch> at its <offset> plus its factor times its
    property (that within <min> and <max>), and the offset is 1.0 where none is given (xmlsound.cxx; a <volume>'s is 0): the
    banks' sections stated none, so every loop played an octave up, flat at SimGear's cap of 2.0. Each states 0 here. And a
    loop's min and max may hold its pitch only where its condition doesn't let it play: otherwise they are its condition's
    bounds on the pitch's property, or SimGear's own limits where it has none. A rotor loop plays at any rotor speed over its
    gate (its volume doesn't fade out above its own speed): the banks' 1.25 left the airplane mode's loops (at 84 %) flat over
    105 %, and they are heard through the conversion at 104 %, NR going 3 % either way before its caution (alerts.py); and their
    min of 0.2 stood a little over the gate (82 rpm, 0.2 of 412 being 82.4). The power turbine's loops play by Ng whatever the
    rotor's speed, so with the rotor brake on (drive.py) they were held at 0.05 while the turbine stood. A pitch given by an
    <expression> adds no offset: left as it is"""
    def entry(m):
        blk = m.group(0)

        def one(p):
            s = p.group(0)
            if '<expression>' in s:
                return s
            prop = re.search(r'<property>([^<]+)</property>', s)
            f = re.search(r'<factor>([^<]+)</factor>', s)
            f = float(f.group(1)) if f else 1.0
            lo, hi = gates(blk, prop.group(1)) if prop else (None, None)
            mn = re.search(r'<min>([^<]+)</min>', s)
            if mn and float(mn.group(1)) > (PITCH_MIN if lo is None else lo * f) * (1 + 1e-3):
                s = s.replace(mn.group(0), '<min>%.6g</min>' % (PITCH_MIN if lo is None else lo * f))
            mx = re.search(r'<max>([^<]+)</max>', s)
            if mx and float(mx.group(1)) < min(PITCH_MAX, PITCH_MAX if hi is None else hi * f) * (1 - 1e-3):
                s = s.replace(mx.group(0), '<max>%.6g</max>' % min(PITCH_MAX, PITCH_MAX if hi is None else hi * f))
            if '<offset>' not in s:
                s = s.replace('</pitch>', '<offset>0</offset></pitch>')
            return s
        return re.sub(r'<pitch>.*?</pitch>', one, blk, flags=re.S)
    return re.sub(r'<(%s)>.*?</\1>' % '|'.join(ENTRY_TAGS), entry, body, flags=re.S)


def np_fade(body):
    """the power turbine's loops (pww-...-turbine-lit-...) with their level by the turbine's speed, the property their pitch
    follows: (speed / 100)^NP_EXPONENT in the volume and over NP_GATE_PCT in the condition. Loops that have it already
    (PyWopWop's writer gives it) are left as they are"""
    def entry(m):
        blk = m.group(0)
        if not re.search(r'<name>pww-[^<]*-turbine-lit-[^<]*</name>', blk):
            return blk
        name = re.search(r'<name>(.+?)</name>', blk).group(1)
        prop = re.search(r'<pitch><property>([^<]+)</property>', blk)
        if not prop:
            sys.exit('%s: no pitch property for its speed' % name)
        q = '<property>%s</property>' % prop.group(1)
        if re.search(r'<pow><div>%s<value>100</value></div>' % re.escape(q), blk):
            return blk
        if '<volume><expression><product>' not in blk or '<condition><and>' not in blk:
            sys.exit('%s: not the volume or condition this expects' % name)
        fade = '<pow><div>%s<value>100</value></div><value>%g</value></pow>' % (q, NP_EXPONENT)
        gate = '<greater-than>%s<value>%.3g</value></greater-than>' % (q, NP_GATE_PCT)
        blk = blk.replace('<volume><expression><product>', '<volume><expression><product>' + fade, 1)
        return blk.replace('<condition><and>', '<condition><and>' + gate, 1)
    return re.sub(r'<engine>.*?</engine>', entry, body, flags=re.S)


def section_only(step, what):
    """the installed section rewritten by step (a function of its text), the rest of the file as it is; what: the blocks
    to count, a regular expression"""
    p = os.path.join(SOUNDS, 'v22-sound.xml')
    t = open(p, encoding='latin-1').read()
    if BEGIN not in t or END not in t:
        sys.exit('no section of fdm/rotorsound.py in %s' % p)
    i, j = t.index(BEGIN), t.index(END)
    old, new = t[i:j], step(t[i:j])
    open(p, 'w', encoding='latin-1').write(t[:i] + new + t[j:])
    a, b = re.findall(what, old, re.S), re.findall(what, new, re.S)
    print('%s: %d of %d rewritten' % (p, sum(x != y for x, y in zip(a, b)), len(b)))


def pitch_only():
    """the installed section's pitch blocks rewritten by pitches(), the rest of the file as it is"""
    section_only(pitches, r'<pitch>.*?</pitch>')


def np_fade_only():
    """the installed section's power turbine loops faded by their speed (np_fade()), their pitch blocks as the install
    writes them (pitches()), the rest of the file as it is"""
    section_only(lambda s: pitches(np_fade(s)), r'<engine>\s*<name>pww-[^<]*-turbine-lit-.*?</engine>')


def rescale_old(t, k, to_cockpit=1.0):
    """OLD_SOUNDS at the banks' scale: one more <volume> (an expression, last: FlightGear skips property entries after
    one), the outside's factor (brought to the cockpit's scale by to_cockpit, as the outside's loops are: scale_pass), the
    cockpit's in the cockpit"""
    ko = to_cockpit * np.sqrt(k['engine', 'outside'] * k['exhaust', 'outside']) / OLD_ENGINE_VOLUME
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

# Scale. Every sound plays at its level on one scale, the same in the cockpit and outside, so that the views keep their difference and
# nothing is set by ear (as the Alouette III's and the X-57's sounds are). A bank writes each view at its own gain, its loudest sample at
# full scale (PyWopWop's peak_dbfs); the cockpit's gain is the lower, its heavy hover in ground effect the loudest sample of all (134.1 dB
# re 20 uPa), so the cockpit's gain is the scale: the outside's loops come down to it in their volume, and the original sounds' outside
# factor with them (rescale_old). A full-scale sample at volume 1 is then 134.1 dB re 20 uPa in either view. The cockpit is the open ear,
# as the cockpit recordings it is fitted to have it: under the crew's HGU-84/P (Gentex's data sheet, ANSI S12.6: 13 dB at 125 Hz to 44 at
# 6.3 kHz) it would be about 17 dB(A) quieter and duller; not modelled.
# The installed files were raised on 2026-11-11 to bring the aircraft up to the Alouette III's level by ear (LOUD_LOG beside them: up to
# 17.6 dB in the cockpit and 7.1 outside, each as far as its peak allowed). They keep the raise, for their resolution, and their volume
# carries it back. SCALE_LOG keeps the factor each entry's volume has, so that the pass can be run again
SCALE_GAINS = {'cockpit': -40.071, 'exterior': -32.790}     # the helicopter bank's gain_db, which every installed loop is at
LOUD_LOG = 'loudness.json'
SCALE_LOG = 'scale.json'


def view_of(blk):
    """'cockpit', 'outside' or None, by the entry's condition"""
    c = re.search(r'<condition>(.*?)</condition>', blk, re.S)
    c = c.group(1).replace('\n', '').replace(' ', '') if c else ''
    inside = '<property>sim/current-view/internal</property>'
    return 'outside' if '<not>%s</not>' % inside in c else ('cockpit' if inside in c else None)


def scale_pass(t, gains=None):
    """the section's bank loops in t on the cockpit's scale: in each volume, last in a product, the outside's gain against the cockpit's
    and the file's raise (LOUD_LOG) taken back; returns the new text"""
    gains = gains or SCALE_GAINS
    to_cockpit = 10 ** ((gains['cockpit'] - gains['exterior']) / 20)
    out = os.path.join(SOUNDS, SUBDIR)
    raised = json.load(open(os.path.join(out, LOUD_LOG))) if os.path.exists(os.path.join(out, LOUD_LOG)) else {}
    log_p = os.path.join(out, SCALE_LOG)
    have = json.load(open(log_p)) if os.path.exists(log_p) else {}
    count = {'cockpit': 0, 'outside': 0}

    def entry(m):
        blk, name = m.group(0), m.group(2)
        view = view_of(blk)
        if view is None:
            sys.exit('%s: no view in its condition' % name)
        fn = os.path.basename(re.search(r'<path>([^<]+)</path>', blk).group(1))
        k = '%.6g' % (10 ** (-raised.get(fn, 0.0) / 20) * (to_cockpit if view == 'outside' else 1.0))
        tail = '<value>%s</value></product></expression></volume>'
        if name in have and tail % have[name] in blk:
            blk = blk.replace(tail % have[name], tail % k, 1)
        else:
            blk, n = re.subn(r'<volume><expression>(.*?)</expression></volume>',
                             lambda q: '<volume><expression><product>%s' % q.group(1) + tail % k, blk, count=1, flags=re.S)
            if n != 1:
                sys.exit('%s: not the volume this expects' % name)
        have[name] = k
        count[view] += 1
        return blk
    i, j = t.index(BEGIN), t.index(END)
    sec = re.sub(r'\n    <(%s)>\n      <name>(pww-[^<]+)</name>.*?\n    </\1>' % '|'.join(ENTRY_TAGS), entry, t[i:j], flags=re.S)
    json.dump(have, open(log_p, 'w'), indent=0, sort_keys=True)
    print('on the cockpit\'s scale (a full-scale sample at volume 1: %.1f dB re 20 uPa): %d cockpit loops, %d outside (x %.4f, %+.2f dB)'
          % (20 * np.log10(1 / 2e-5) - gains['cockpit'], count['cockpit'], count['outside'], to_cockpit, 20 * np.log10(to_cockpit)))
    return t[:i] + sec + t[j:]


def scale_only():
    """the installed loops and the original sounds on the cockpit's scale (scale_pass, rescale_old at SCALE_GAINS), the rest as it is"""
    p = os.path.join(SOUNDS, 'v22-sound.xml')
    t = open(p, encoding='latin-1').read()
    to = 10 ** ((SCALE_GAINS['cockpit'] - SCALE_GAINS['exterior']) / 20)
    t, ko, kc = rescale_old(t, engine_scale(SCALE_GAINS), to)
    t = scale_pass(t)
    open(p, 'w', encoding='latin-1').write(t)
    print('the original sounds: x %.4f outside, x %.4f in the cockpit' % (ko, kc))


def ramp(prop, a, b):
    """an expression going linearly from 0 at a to 1 at b, clipped to 0..1 (PyWopWop's bank.py _ramp: a <volume>'s own
    factor, offset, min and max can't make a fade in FlightGear)"""
    f = 1.0 / (b - a)
    return ('<clip><sum><product><property>%s</property><value>%.6g</value></product><value>%.6g</value></sum>'
            '<clipMin>0</clipMin><clipMax>1</clipMax></clip>' % (prop, f, -a * f))


def ramp_x(x, a, b):
    """ramp on an expression"""
    f = 1.0 / (b - a)
    return '<clip><sum><product>%s<value>%.6g</value></product><value>%.6g</value></sum><clipMin>0</clipMin><clipMax>1</clipMax></clip>' % (x, f, -a * f)


def sigma_isa(alt_m):
    """the standard atmosphere's density ratio under the tropopause"""
    return (1.0 - 2.25577e-5 * alt_m) ** 4.25588


def altitude(k, top_m=None):
    """the level by the air's density, relative to sea level: its power k, no further than the altitude it was measured to"""
    x = SIGMA if top_m is None else '<max>%s<value>%.4g</value></max>' % (SIGMA, sigma_isa(top_m))
    return '<pow>%s<value>%.4g</value></pow>' % (x, k)


def engine_power(name):
    """an engine loop's power of the density ratio (soundalt.py): by its layer, place and Ng; none for the gearbox, its torque's in any air"""
    m = re.search(r'-(compressor|core|turbine|starter)-(?:lit|cold|starter)-([\d.]+)-(side|cockpit)$', name)
    if not m:
        return None
    law = ALT_LAW['engines'][m.group(1)]
    return float(np.interp(float(m.group(2)), law['ng'], law[m.group(3)]))


def power_fade(*parts):
    """fades of loops that don't add in phase: their powers cross-fade, so the amplitude is the root of the product. Linear
    amplitudes left a grid's cell centre 6 dB under its corners, a dip at every speed and load between two loops"""
    return '<sqrt><product>%s</product></sqrt>' % ''.join(parts)


def inside(prop, lo=None, hi=None):
    """condition terms: prop over lo and under hi (either may be None)"""
    t = []
    if lo is not None:
        t.append('<greater-than><property>%s</property><value>%.6g</value></greater-than>' % (prop, lo))
    if hi is not None:
        t.append('<less-than><property>%s</property><value>%.6g</value></less-than>' % (prop, hi))
    return t


def window_terms(prop, axis, i):
    """the condition terms of point i's cross-fade over prop (windows): true wherever its fade isn't zero"""
    return inside(prop, axis[i - 1] if i else None, axis[i + 1] if i < len(axis) - 1 else None)


def with_condition(blk, terms):
    """a sound entry with more terms in its condition's <and>"""
    i = blk.index('<condition>')
    j = blk.index('</and>', i)
    k = blk.rfind('\n', i, j) + 1
    ind = blk[k:j]
    return blk[:k] + ''.join(ind + '\t' + x + '\n' for x in terms) + blk[k:]


def windows(x, axis, i):
    """the fades that make point i of an axis's cross-fade over the expression x: flat past its ends"""
    out = []
    if i:
        out.append(ramp_x(x, axis[i - 1], axis[i]))
    if i < len(axis) - 1:
        out.append(ramp_x(x, axis[i + 1], axis[i]))
    return out


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


def slap_levels(bank):
    """the slap map's level (dBA) at an airspeed and vertical speed relative to SLAP_REF's state, by view, as slap_table reads it"""
    m = json.load(open(os.path.join(bank, 'manifest.json')))
    d = json.load(open(os.path.join(bank, 'slap_map.json')))
    ref = next(c for c in m['conditions'] if c['name'] == SLAP_REF)
    pts = [q for q in d['points'] if 'error' not in q]
    vc = np.array([(q['airspeed_kt'], q['climb_fpm'] / 100.0) for q in pts])

    def rel(view, kt, fpm):
        L = np.array([q['levels'][SLAP_VIEW[view][1]]['slap']['dba'] for q in pts])
        L = np.where(L < -100, L[L > -100].min() - 60, L)
        at = lambda x, y: float(griddata(vc, L, [(x, y / 100.0)], 'linear')[0])
        return at(kt, fpm) - at(ref['airspeed_kt'], ref['climb_fpm'])
    return rel


def slap_section(bank, airplane, tab):
    """the banks' blade slap: every condition's slap loops, in both views, cross-faded as PyWopWop's own section has them
    (the level-flight loops by airspeed, the climbs and descents by the vertical speed within their airspeed windows), each
    playing only inside its window. The helicopter mode's level is the flight model's (/rotors/main/slap-ext, -int: its map
    over the advance ratio and the inflow, relative to SLAP_REF's state): each loop over its own state's level in that map, so
    that in a trimmed state the bank's loops play as rendered, and in a flare or a turn at the map's level; raised in the file
    where the map has louder slap than its state, up to PEAK_DBFS. Faded out with the nacelles. The airplane mode's as its bank
    has them (the map has none), at the helicopter bank's gains, faded in with the nacelles. The entries and, for each file,
    its bank and the factor it is raised by"""
    rel = slap_levels(bank)
    g_ref = json.load(open(os.path.join(bank, 'manifest.json')))['gain_db']
    blocks, files = [], {}
    for src_bank, air in ((bank, False), (airplane, True)):
        x = open(os.path.join(src_bank, 'pywopwop-sound.xml'), encoding='utf-8').read()
        body = x[x.index('<fx>') + len('<fx>'):x.index('</fx>')]
        m = json.load(open(os.path.join(src_bank, 'manifest.json')))
        conds = {c['name']: c for c in m['conditions']}
        level = sorted([c['airspeed_kt'] for c in m['conditions'] if abs(c['climb_fpm']) < 1.0 and c['ground_clearance_m'] is None
                        and not c.get('ground_run')])
        fade = ramp(NAC, NAC_FULL, NAC_GONE) if air else ramp(NAC, NAC_GONE, NAC_FULL)
        for blk in re.findall(r'\t*<rotor>.*?</rotor>', body, re.S):
            name = re.search(r'<name>pww-(.+?)</name>', blk).group(1)
            cname, view, src = re.match(r'(.+)-(side|cockpit|cockpit_open)-(rotor|slap|swish|tail|mix)$', name).groups()
            if src != 'slap' or view not in SLAP_VIEW:
                continue
            c = conds[cname]
            fn = re.search(r'<path>Sounds/pywopwop/(.+?)</path>', blk).group(1)
            blk = blk.replace('<path>Sounds/pywopwop/', '<path>%s/' % SUBDIR)
            key = SLAP_VIEW[view][0]
            # (its window: the nacelles, and the airspeed and vertical speed its fades leave it any volume in)
            cond = inside(NAC, hi=NAC_FULL) if air else inside(NAC, lo=NAC_GONE)
            if abs(c['climb_fpm']) < 1.0:
                cond += window_terms(SPD_P, level, level.index(c['airspeed_kt']))
            else:
                cond += inside(SPD_P, c['airspeed_kt'] - 40.0 if c['airspeed_kt'] > 40.0 else None, c['airspeed_kt'] + 50.0)
                cond += inside(VS_P, lo=5.0) if c['climb_fpm'] > 0 else inside(VS_P, hi=-5.0)
            fs, y = wav.read(os.path.join(src_bank, fn))
            room = 10 ** (PEAK_DBFS / 20) / (np.abs(y.astype(np.float64)).max() / 32768.0)
            parts = [fade, altitude(ALT_LAW['slap'][view])]
            if air:
                g = 10 ** ((g_ref['cockpit' if view == 'cockpit' else 'exterior'] - m['gain_db']['cockpit' if view == 'cockpit' else 'exterior']) / 20)
                g *= sigma_isa(m['altitude_m']) ** -ALT_LAW['slap'][view]
                b = max(1.0, min(g, room))
                parts.append('<value>%.6g</value>' % (g / b))
            else:
                a = 10 ** (rel(view, c['airspeed_kt'], c['climb_fpm']) / 20)
                b = max(1.0, min(tab['peak'][key] / a, room))
                parts.append('<clip><product><property>/rotors/main/slap-%s</property><value>%.6g</value></product>'
                             '<clipMin>0</clipMin><clipMax>1</clipMax></clip>' % (key, 1.0 / (a * b)))
            blk = re.sub(r'(\t+)(<volume><expression><product>\n)', lambda mm: mm.group(0) + ''.join(mm.group(1) + '\t' + p + '\n' for p in parts), blk)
            blk = with_condition(blk, cond)
            blocks.append(blk)
            files[fn] = (src_bank, b)
    return '\n'.join(blocks).replace('\t', '  '), files


def grid_section(bank, reference, template, tcond, airplane):
    """a load grid's loops, each its rotor and swish as one (same volume, pitch and place, so nothing is lost, and the swish, the
    blade passage's own noise, stays locked to the rotor's thump), blended by airspeed and load with the nacelles' fade, at the
    reference bank's gains. The sound entries, and for each file its factor (over 1: the file itself raised, FlightGear's
    volume stopping at 1)"""
    # (the entry's mode, condition, pitch and place from a bank of the same mode: template's tcond; a grid has no section of its own)
    x = open(os.path.join(template, 'pywopwop-sound.xml'), encoding='utf-8').read()
    m = json.load(open(os.path.join(bank, 'manifest.json')))
    g_ref = json.load(open(os.path.join(reference, 'manifest.json')))['gain_db']
    to_ref = {g: 10 ** ((g_ref[g] - m['gain_db'][g]) / 20) for g in m['gain_db']}
    by = {re.search(r'<name>pww-(.+?)</name>', e).group(1): e for e in re.findall(r'\t*<rotor>.*?</rotor>', x, re.S)}
    fade = ramp(NAC, NAC_FULL, NAC_GONE) if airplane else ramp(NAC, NAC_GONE, NAC_FULL)     # (airplane: all of it with the nacelles down)
    spd = '<property>velocities/airspeed-kt</property>'
    conds = [c for c in m['conditions']]
    air = [c for c in conds if c['ground_clearance_m'] is None]
    kts = sorted({c['airspeed_kt'] for c in air})
    if airplane:
        loads, load_x = sorted({c['thrust_n'] / 4.448222 for c in air}), THRUST
        at = lambda c: c['thrust_n'] / 4.448222
    else:
        loads, load_x = sorted({c['load_factor'] * HOVER_T_W for c in air}), T_OVER_W
        at = lambda c: c['load_factor'] * HOVER_T_W
    ige = ramp_x(AGL, *IGE_FT)
    blocks, files = [], {}
    for view in ('side', 'cockpit'):
        for c in conds:
            name = c['name']
            blk = by['%s-%s-rotor' % (tcond, view)]
            blk = re.sub(r'<name>.*?</name>', '<name>pww-%s-%s-blades</name>' % (name, view), blk)
            blk = re.sub(r'<path>.*?</path>', '<path>%s/%s_%s_blades.wav</path>' % (SUBDIR, name, view), blk)
            rpm = re.search(r'(<clip><sum><product><property>rotors/main/rpm</property>.*?</clip>)', blk).group(1)
            # (the power from sea level, the loop in its bank's air: that air's level taken out. Raised in the file if louder than full
            # volume at the top of its air)
            law = ALT_LAW['blades'][name]
            g = to_ref['cockpit' if view == 'cockpit' else 'exterior'] * sigma_isa(m['altitude_m']) ** -law[view]
            raised = max(1.0, g, g * sigma_isa(law['top_m']) ** law[view])
            parts = [rpm, altitude(law[view], law['top_m'])]
            if g / raised < 1:
                parts.append('<value>%.6g</value>' % (g / raised))
            cross = [fade] + windows(load_x, loads, loads.index(at(c)))
            cond = inside(NAC, hi=NAC_FULL) if airplane else inside(NAC, lo=NAC_GONE)
            cond += window_terms(THRUST_P if airplane else T_OVER_W_P, loads, loads.index(at(c)))
            if c['ground_clearance_m'] is None:
                cross += windows(spd, kts, kts.index(c['airspeed_kt']))
                cond += window_terms(SPD_P, kts, kts.index(c['airspeed_kt']))
                if not airplane and c['airspeed_kt'] == 0:
                    cross.append(ramp_x(AGL, *IGE_FT[::-1]))
                    cond += inside(AGL_P, lo=IGE_FT[1])
            else:
                cross += [ramp_x(spd, kts[1], kts[0]), ige]
                cond += inside(SPD_P, hi=kts[1]) + inside(AGL_P, hi=IGE_FT[0])
                # (heard at the distance it was rendered at, on the ground beside the aircraft: the template's is its hover's, 100 m)
                blk = re.sub(r'<reference-dist>[^<]*</reference-dist>', '<reference-dist>%g</reference-dist>'
                             % (c.get('listener_distance_m') or 50.0), blk)
            parts.append(power_fade(*cross))
            if not airplane and c['ground_clearance_m'] is not None:
                parts.append(GROUND_IGE % ground_share(min(loads)))
            blk = with_condition(blk, cond)
            blk = re.sub(r'<volume>.*?</volume>', lambda _: '<volume><expression><product>' + ''.join(parts) + '</product></expression></volume>', blk, flags=re.S)
            blocks.append(blk)
            files['%s_%s_blades.wav' % (name, view)] = raised
    return '\n'.join(blocks).replace('\t', '  '), files


def _blades_float(bank, fn):
    """a bank's rotor and swish loops as one, the run-up's with RUNUP_EQ_DB for its view"""
    fs, r = wav.read(os.path.join(bank, fn.replace('_blades.wav', '_rotor.wav')))
    fs2, w = wav.read(os.path.join(bank, fn.replace('_blades.wav', '_swish.wav')))
    if fs != fs2 or len(r) != len(w):
        sys.exit('%s: its rotor and swish loops differ in length' % fn)
    y = r.astype(np.float64) + w.astype(np.float64)
    if fn.startswith('runup_'):
        y = loop_eq(y, fs, RUNUP_EQ_DB['cockpit' if '_cockpit_' in fn else 'side'])
    return fs, y


def loop_eq(y, fs, table):
    """a seamless loop through a gain curve ([[Hz, dB], ...], in log frequency, its end values held beyond it), applied to its
    spectrum: the loop is periodic, so it stays seamless"""
    t = np.asarray(table, float)
    Y = np.fft.rfft(y)
    f = np.fft.rfftfreq(len(y), 1.0 / fs)
    g = np.interp(np.log(np.maximum(f, 1.0)), np.log(t[:, 0]), t[:, 1])
    return np.fft.irfft(Y * 10 ** (g / 20), len(y))


def blades(bank, fn, k=1.0):
    """a bank's rotor and swish loops as one (fn its ..._blades.wav): their sum, as the bank's mix less the slap, times k"""
    fs, y = _blades_float(bank, fn)
    y = y * k
    pk = np.abs(y).max()
    if pk > 32767:              # (louder than full scale at the reference gain: fitted to it, that loop alone that much quieter)
        y *= 32767 / pk
        print('%s: %.1f dB over full scale at the banks\' gain, played that much under it' % (fn, 20 * np.log10(pk / 32767)))
    return fs, np.round(y).astype(np.int16)


# The rotors on the ground. The helicopter grid's lowest row carries 0.66 of the weight (0.6 g at the hover's thrust over the weight),
# held below it: on the ground at flat pitch (the thrust nearly nil) the rotors played as if carrying two thirds of the aircraft,
# which buried the engines' start. An outside start recording (PyWopWop's refs/recordings/mv22b_start) is broadband from 250 Hz
# to 8 kHz at 85 % NR, its lows 20-34 dB under its mids; the installed sound's rose 22-27 dB over them. So on the ground (JSBSim's
# gear/wow) PyWopWop's run-up plays: the rotors on the gear at flat pitch (bank --runup, the preset's flat_pitch_deg: the flight
# model's collective with the lever down) at 20 to 100 % of their speed, cross-faded by it with flat tops at its points (the
# governor holds 104 % on one) as the Alouette III's are (al3sound.py), the slowest from nothing, out where it is 60 dB under the
# loudest. Against the grid's in-ground-effect loops by the thrust over the weight: a rotor's loading noise goes with its thrust
# (Gutin), so the grid by power (|T/W| / its lowest row)^2 and the run-up by what that leaves; a lever raised on the ground brings
# the grid in before the wheels leave. Off the ground the grid alone, as before. The run-up has no altitude law (soundalt.py has
# none for it): its levels are sea level's
WOW_P = '/fdm/jsbsim/gear/wow'
RUNUP_FLAT = 0.01         # the run-up's flat tops' half width, of the rotors' nominal speed
RUNUP_HEARD = 1e-3        # -60 dB: where the slowest run-up loop stops
RUNUP_PEAK_DBFS = -1.0    # the run-up's files' peaks
# The run-up's lows fitted to two recordings, by view (dB, in log frequency, held beyond): at flat pitch the rotors' lowest harmonics
# are their in-plane thickness noise (the run-up's unweighted level stayed put when the mirrored inflow moved its thrust to the
# flight model's), and both recordings have far less of them. The cockpit's from one recorded on the ground at governed speed
# (PyWopWop's refs/recordings/mv22b_cockpit2, 102-104 % NR): FlightGear's cockpit at 104 % had 63-100 Hz 17 dB, 125 Hz 10 and 160 Hz
# 8 over it, each against its 1 kHz octave, and 200 Hz up within a dB or so. The outside's from the start recorded outside
# (refs/recordings/mv22b_start, at 85 % NR): 125 Hz 12.6 dB, 160 Hz 6.7 and 200 Hz 1.6 over it, the same way; under 125 Hz
# that camera's own roll-off is unknown, so the cockpit's 17 dB is taken there. Taken down only
RUNUP_EQ_DB = {'cockpit': [[50.0, -17.0], [100.0, -17.0], [125.0, -10.0], [160.0, -8.0], [200.0, 0.0]],
               'side': [[50.0, -17.0], [100.0, -17.0], [125.0, -12.6], [160.0, -6.7], [200.0, -1.6], [250.0, 0.0]]}
GROUND_IGE = ('<difference><value>1</value><product><property>%s</property><difference><value>1</value>%%s</difference>'
              '</product></difference>' % WOW_P)


def ground_share(low):
    """the grid's share of the rotors' amplitude on the ground: the thrust over the weight against the grid's lowest row, 0 to 1"""
    return ('<clip><product><abs>%s</abs><value>%.6g</value></product><clipMin>0</clipMin><clipMax>1</clipMax></clip>'
            % (T_OVER_W, 1.0 / low))


def tents(nodes, flat, lo0=None):
    """fades over sorted grid points (as the Alouette III's al3sound.py has them): for each, the (from, to) of its rising and
    falling ramps (None at an end) and its open interval. With flat tops of half width flat[i]: within it only point i plays,
    and the ramps either side move with it so neighbours still add to one. lo0: where the first point's fade starts (None: it
    holds to the bottom)"""
    out = []
    for i, x in enumerate(nodes):
        up = dn = None
        a, b = -1e30, 1e30
        if i > 0:
            up = (nodes[i - 1] + flat[i - 1], x - flat[i])
            a = up[0]
        elif lo0 is not None:
            up = (lo0, x - flat[i])
            a = lo0
        if i < len(nodes) - 1:
            dn = (nodes[i + 1] - flat[i + 1], x + flat[i])
            b = dn[0]
        out.append((up, dn, a, b))
    return out


def runup_section(bank, k, low):
    """the bank's run-up loops on the ground, each its rotor and swish as one (as the grid's), by the rotors' speed and, against
    the grid's in-ground-effect loops, by their thrust; k: {'side': ..., 'cockpit': ...}, the factor to the reference bank's
    scale; low: the grid's lowest row of thrust over the weight. The sound entries, and for each file its factor (over 1: the
    file itself raised)"""
    m = json.load(open(os.path.join(bank, 'manifest.json')))
    runup = sorted([c for c in m['conditions'] if c.get('ground_run')], key=lambda c: c['rpm_percent'])
    if not runup:
        return '', {}
    nom = next(r['nominal_rpm'] for r in m['rotors'].values() if r['role'] == 'main')
    lv = {(f['condition'], f['observer'], f['source']): f['dba'] for f in m['files']}
    nodes = [c['rpm_percent'] / 100.0 * nom for c in runup]
    rpm = 'rotors/main/rpm'
    share = '<difference><value>1</value><pow>%s<value>2</value></pow></difference>' % ground_share(low)
    blocks, files = [], {}
    for view in ('side', 'cockpit'):
        db = {c['name']: 10 * np.log10(sum(10 ** (lv[(c['name'], view, s)] / 10) for s in ('rotor', 'swish')
                                           if (c['name'], view, s) in lv)) for c in runup}
        top = max(db.values())
        for i, ((up, dn, a, b), c) in enumerate(zip(tents(nodes, [RUNUP_FLAT * nom] * len(nodes), lo0=0.0), runup)):
            name = c['name']
            cross = [ramp(NAC, NAC_GONE, NAC_FULL), share] + ([ramp(rpm, *dn)] if dn else [])
            parts = []
            if i == 0:
                # (the slowest from nothing, its amplitude the speed's share of its own, out where it is 60 dB under the loudest)
                parts.append(ramp(rpm, *up))
                a = max(nodes[0] * RUNUP_HEARD * 10 ** ((top - db[name]) / 20), 0.01 * nodes[0])
            else:
                cross.append(ramp(rpm, *up))
            parts.append(power_fade(*cross))
            # (each file raised to RUNUP_PEAK_DBFS, its volume taking the rest: the slow loops lie far under the bank's loudest)
            pk = np.abs(_blades_float(bank, '%s_%s_blades.wav' % (name, view))[1]).max()
            g = k[view]
            raised = max(1.0, g, 10 ** (RUNUP_PEAK_DBFS / 20) * 32767 / max(pk, 1.0))
            if g / raised < 1:
                parts.append('<value>%.6g</value>' % (g / raised))
            cond = [('<property>sim/current-view/internal</property>' if view == 'cockpit'
                     else '<not><property>sim/current-view/internal</property></not>'), '<property>%s</property>' % WOW_P]
            cond += inside(NAC, lo=NAC_GONE) + inside(T_OVER_W_P, lo=-low, hi=low) + inside(rpm, lo=a, hi=b if b < 1e29 else None)
            ref = nodes[i]
            hi = b if b < 1e29 else 2.0 * ref
            blk = ('    <rotor>\n      <name>pww-%s-%s-blades</name>\n      <mode>looped</mode>\n'
                   '      <path>%s/%s_%s_blades.wav</path>\n      <condition>\n        <and>\n%s        </and>\n      </condition>\n'
                   '      <volume><expression><product>%s</product></expression></volume>\n'
                   '      <pitch><property>%s</property><factor>%.6g</factor><min>%.4g</min><max>%.4g</max><offset>0</offset></pitch>\n'
                   % (name, view, SUBDIR, name, view, ''.join('          %s\n' % x for x in cond), ''.join(parts),
                      rpm, 1.0 / ref, a / ref, min(hi / ref, PITCH_MAX)))
            if view == 'side':
                blk += '      <reference-dist>%g</reference-dist>\n      <max-dist>4000</max-dist>\n' % (c.get('listener_distance_m') or 50.0)
            blocks.append(blk + '    </rotor>\n')
            files['%s_%s_blades.wav' % (name, view)] = raised
    return ''.join(blocks).rstrip('\n'), files


def ground_ige(body, low, dist):
    """the installed grid's in-ground-effect loops as grid_section writes them now: on the ground by the thrust (GROUND_IGE),
    and outside at the distance they were rendered at"""
    def entry(m):
        blk = m.group(0)
        if not re.search(r'<name>pww-he_ige_\d+-(side|cockpit)-blades</name>', blk):
            return blk
        if WOW_P not in blk:
            blk = blk.replace('</product></expression></volume>', GROUND_IGE % ground_share(low) + '</product></expression></volume>', 1)
        return re.sub(r'<reference-dist>[^<]*</reference-dist>', '<reference-dist>%g</reference-dist>' % dist, blk)
    return re.sub(r'<rotor>.*?</rotor>', entry, body, flags=re.S)


def runup_only(bank):
    """the installed section with bank's run-up (bank --mode helicopter_load --runup, with the grid's he_ige_1 rendered beside it
    at the same gains: the installed loops' banks are gone, so their scale is that loop's level as installed over its level in
    bank), the grid's in-ground-effect loops as grid_section writes them now and the pitch blocks as the install writes them, the
    rest of the file as it is. The run-up's loops written to SUBDIR"""
    import soundcheck as S
    import xml.etree.ElementTree as ET
    m = json.load(open(os.path.join(bank, 'manifest.json')))
    cref = next((c for c in m['conditions'] if c['name'] == 'he_ige_1'), None)
    if cref is None or not any(f['condition'] == 'he_ige_1' for f in m['files']):
        sys.exit('%s: no he_ige_1 to take the installed scale from' % bank)
    p = os.path.join(SOUNDS, 'v22-sound.xml')
    t = open(p, encoding='latin-1').read()
    if BEGIN not in t or END not in t:
        sys.exit('no section of fdm/rotorsound.py in %s' % p)
    i, j = t.index(BEGIN), t.index(END)
    sec = re.sub(r'\n?[ \t]*<rotor>\s*<name>pww-runup_[^<]*</name>.*?</rotor>', '', t[i:j], flags=re.S)
    # the grid's lowest row of thrust over the weight: the lowest bound its loops' windows have
    low = min(float(v) for v in re.findall(r'<greater-than><property>%s</property><value>([^<]+)</value>'
                                            % re.escape(T_OVER_W_P), sec))
    nom = next(r['nominal_rpm'] for r in m['rotors'].values() if r['role'] == 'main')
    k = {}
    for view in ('side', 'cockpit'):
        blk = re.search(r'<rotor>\s*<name>pww-he_ige_1-%s-blades</name>.*?</rotor>' % view, sec, re.S).group(0)
        x = ET.fromstring(blk)
        P = {S.key(e): np.zeros(1) for e in x.iter('property')}
        # (its own state: helicopter mode, sea level, its speed and load, on the ground's edge, airborne)
        P.update({NAC: np.full(1, 90.0), 'environment/density-slugft3': np.full(1, 0.00237689), 'rotors/main/rpm': np.full(1, nom),
                  T_OVER_W_P.lstrip('/'): np.full(1, cref['load_factor'] * HOVER_T_W), WOW_P.lstrip('/'): np.zeros(1)})
        V = float(np.ravel(S.val(list(x.find('volume').find('expression'))[0], P))[0])
        _, y = wav.read(os.path.join(SOUNDS, x.findtext('path')))
        _, r = wav.read(os.path.join(bank, 'he_ige_1_%s_rotor.wav' % view))
        _, w = wav.read(os.path.join(bank, 'he_ige_1_%s_swish.wav' % view))
        z = r.astype(np.float64) + w.astype(np.float64)
        k[view] = float(np.sqrt(np.mean(y.astype(np.float64) ** 2)) * V / np.sqrt(np.mean(z ** 2)))
        print('%s: the installed scale over %s\'s, from he_ige_1: %+.2f dB (its octaves 63 Hz-4 kHz: %s)' % (
            view, bank, 20 * np.log10(k[view]), ' '.join('%+.1f' % d for d in octave_ratio_db(y.astype(np.float64) * V, z))))
    sec = ground_ige(sec, low, cref.get('listener_distance_m') or 50.0)
    body, files = runup_section(bank, k, low)
    sec = pitches(sec.rstrip('\n') + '\n' + body + '\n')
    for fn, kf in files.items():
        wav.write(os.path.join(SOUNDS, SUBDIR, fn), *blades(bank, fn, kf))
    open(p, 'w', encoding='latin-1').write(t[:i] + sec + t[j:])
    print('%s: %d run-up loops from %s, the grid\'s in-ground-effect loops on the ground by the thrust (lowest row %g)'
          % (p, len(files), bank, low))


def octave_ratio_db(a, b, fs=48000.0, oct_=(63, 125, 250, 500, 1000, 2000, 4000)):
    """a's octave band levels over b's, dB"""
    fa, A = np.fft.rfftfreq(len(a), 1 / fs), np.abs(np.fft.rfft(a)) ** 2
    fb, B = np.fft.rfftfreq(len(b), 1 / fs), np.abs(np.fft.rfft(b)) ** 2
    band = lambda f, X, o: X[(f >= o / 2 ** 0.5) & (f < o * 2 ** 0.5)].sum() / len(f)
    return [10 * np.log10(band(fa, A, o) / band(fb, B, o)) for o in oct_]


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


def engine_scale(gains):
    """the original engine recordings' factors at the banks' scale (gains: the helicopter bank's gain_db, by view), from Levine and
    DeFelice's levels: no longer played, they are what carries the original aircraft's other sounds to it (rescale_old)"""
    sys.path.insert(0, HERE)
    import v22data as V
    full ={v: 20 * np.log10(1.0 / 10 ** (gains['cockpit' if v == 'cockpit' else 'exterior'] / 20) / 2e-5)
            for v in ('outside', 'cockpit')}
    k = {}
    for name, fn, spec, _ in ENGINE_LOOPS:
        lf = file_level(os.path.join(SOUNDS, fn))
        for view in ('outside', 'cockpit'):
            k[name, view] = 10 ** ((engine_level(spec, V.P_MRP, view == 'cockpit') - full[view] - lf) / 20)
    return k


def engine_section(bank, helicopter=None):
    """the engine bank's <engine> entries, the loops in SUBDIR. The nacelles-up bank's: faded out with the nacelles; the
    nacelles-down bank's (helicopter: the rotors' helicopter bank): faded in, at the helicopter bank's gains. Each only while
    it can be heard. The files they play"""
    x = open(os.path.join(bank, 'pywopwop-engines.xml'), encoding='utf-8').read()
    body = x[x.index('<fx>') + len('<fx>'):x.index('</fx>')].lstrip('\n').rstrip()
    body = body.replace('<path>Sounds/pywopwop/', '<path>%s/' % SUBDIR)
    fade = ramp(NAC, NAC_FULL, NAC_GONE) if helicopter else ramp(NAC, NAC_GONE, NAC_FULL)
    heard = ('<less-than><property>%s</property><value>%g</value></less-than>' % (NAC, NAC_FULL) if helicopter
             else '<greater-than><property>%s</property><value>%g</value></greater-than>' % (NAC, NAC_GONE))
    if helicopter:
        g_air = json.load(open(os.path.join(bank, 'manifest.json')))['gain_db']
        g_hel = json.load(open(os.path.join(helicopter, 'manifest.json')))['gain_db']
        to_hel = {g: 10 ** ((g_hel[g] - g_air[g]) / 20) for g in g_air}
    blocks = []
    for blk in re.findall(r'\t*<engine>.*?</engine>', body, re.S):
        name = re.search(r'<name>(.+?)</name>', blk).group(1)
        g = ''
        if helicopter:
            g = '<value>%.6g</value>' % to_hel['cockpit' if name.endswith('-cockpit') else 'exterior']
        if '<volume><expression><product>' not in blk or '<condition><and>' not in blk:
            sys.exit('%s: not the volume or condition this expects' % name)
        k = engine_power(name)
        alt = altitude(k) if k is not None else ''
        blk = blk.replace('<volume><expression><product>', '<volume><expression><product>' + power_fade(fade) + g + alt, 1)
        blk = blk.replace('<condition><and>', '<condition><and>' + heard, 1)
        blocks.append(blk)
    body = '\n'.join(blocks).replace('\t', '  ')
    return body, re.findall(r'<path>%s/([^<]+)</path>' % SUBDIR, body)


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


def gear_section(gear, xml, reference):
    """a tyre bank's FlightGear section (PyWopWop's tyre.py: the rolling loops, or the touchdowns' one-shots), the files it
    plays; made at the reference bank's gains"""
    meta = 'touchdown.json' if os.path.exists(os.path.join(gear, 'touchdown.json')) else 'manifest.json'
    g = json.load(open(os.path.join(gear, meta)))['gain_db']
    g_ref = json.load(open(os.path.join(reference, 'manifest.json')))['gain_db']
    for k in g_ref:
        if abs(g[k] - g_ref[k]) > 0.01:
            sys.exit('%s: its %s gain %+.2f dB is not the rotor bank\'s %+.2f' % (gear, k, g[k], g_ref[k]))
    x = open(os.path.join(gear, xml), encoding='utf-8').read()
    body = x[x.index('<fx>') + len('<fx>'):x.index('</fx>')].lstrip('\n').rstrip()
    body = body.replace('<path>Sounds/pywopwop/', '<path>%s/' % SUBDIR).replace('\t', '  ')
    return body, re.findall(r'<path>%s/([^<]+)</path>' % SUBDIR, body)


def install(bank, airplane, ap_grid, he_grid, wind, engines, engines_air, tyres, touchdown):
    tab = slap_table(bank)
    json.dump(tab, open(os.path.join(HERE, 'data', 'slap_v22.json'), 'w'))
    body, slaps = slap_section(bank, airplane, tab)
    body_a, grid_a = grid_section(ap_grid, bank, airplane, 'airplane_220', True)
    body_h, grid_h = grid_section(he_grid, bank, bank, 'hover', False)
    # (the helicopter bank's own run-up on the ground, if it has one: pywopwop bank mv22b --runup --update -o BANK)
    low = min(c['load_factor'] for c in json.load(open(os.path.join(he_grid, 'manifest.json')))['conditions']
              if c['ground_clearance_m'] is None) * HOVER_T_W
    body_r, grid_r = runup_section(bank, {'side': 1.0, 'cockpit': 1.0}, low)
    if not grid_r:
        print('no run-up in %s: the rotors on the ground play the grid\'s lowest row (bank --runup --update adds it)' % bank)
    body_w, files_w = wind_section(wind, bank)
    gains = json.load(open(os.path.join(bank, 'manifest.json')))['gain_db']
    k_eng = engine_scale(gains)
    body_e, files_e = engine_section(engines)
    body_ea, files_ea = engine_section(engines_air, helicopter=bank)
    body_t, files_t = gear_section(tyres, 'pywopwop-tyres.xml', bank)
    body_d, files_d = gear_section(touchdown, 'pywopwop-touchdown.xml', bank)
    out = os.path.join(SOUNDS, SUBDIR)
    if os.path.isdir(out):
        shutil.rmtree(out)
    os.makedirs(out)
    for src, grid in ((ap_grid, grid_a), (he_grid, grid_h), (bank, grid_r)):
        for fn, k in grid.items():
            wav.write(os.path.join(out, fn), *blades(src, fn, k))
    for fn, (src, b) in slaps.items():
        if b > 1.0:
            fs, y = wav.read(os.path.join(src, fn))
            wav.write(os.path.join(out, fn), fs, np.clip(np.round(y.astype(np.float64) * b), -32768, 32767).astype(np.int16))
        else:
            shutil.copy(os.path.join(src, fn), os.path.join(out, fn))
    for src, fns in ((wind, files_w), (engines, files_e), (engines_air, files_ea), (tyres, files_t), (touchdown, files_d)):
        for fn in fns:
            shutil.copy(os.path.join(src, fn), os.path.join(out, fn))
    body = pitches(np_fade('\n'.join([x for x in (body, body_a, body_h, body_r, body_w, body_e, body_ea, body_t, body_d) if x])))
    files = list(slaps) + list(grid_a) + list(grid_h) + list(grid_r) + files_w + files_e + files_ea + files_t + files_d
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
               for n in ('engine-start', 'engine-loop', 'exhaust-loop')] + [
               r'    <(rumble|squeal)>\n      <name>%s</name>.*?</\1>\n\n?' % n for n in OLD_TYRES]:
        m = re.search(rx, t, re.S)
        if m:
            t = t[:m.start()] + t[m.end():]
    t, ko, kc = rescale_old(t, k_eng, 10 ** ((gains['cockpit'] - gains['exterior']) / 20))
    t = scale_pass(t, gains)        # (the loops just written: the logs went with the old folder)
    open(p, 'w', encoding='latin-1').write(t)
    mb = sum(os.path.getsize(os.path.join(out, fn)) for fn in files) / 2 ** 20
    print('%d loops (%.0f MB) in %s, %d sounds in %s' % (len(files), mb, out, sum(body.count('<%s>' % k) for k in
                                                       ('rotor', 'wind', 'engine', 'tyre', 'touchdown')), p))
    print('the original sounds at the banks\' scale: x %.3f (%+.1f dB) outside, x %.3f (%+.1f dB) in the cockpit' % (
        ko, 20 * np.log10(ko), kc, 20 * np.log10(kc)))
    for fn, (src, b) in sorted(slaps.items()):
        print('%s: raised %+.1f dB in the file' % (fn, 20 * np.log10(b)))
    print('data/slap_v22.json: %d x %d (lambda x mu), thrust reference %.0f lbf' % (len(tab['lam']), len(tab['mu']), tab['thrust_ref_lbf']))


if __name__ == '__main__':
    if sys.argv[1:] == ['--pitch']:
        pitch_only()
    elif sys.argv[1:] == ['--np-fade']:
        np_fade_only()
    elif len(sys.argv) == 3 and sys.argv[1] == '--runup':
        runup_only(os.path.expanduser(sys.argv[2]))
    elif sys.argv[1:] == ['--scale']:
        scale_only()
    else:
        install(*[os.path.expanduser(a) for a in sys.argv[1:10]])
