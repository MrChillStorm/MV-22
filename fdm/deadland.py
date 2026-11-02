#!/usr/bin/env python3
"""Both engines out: flying the windmilling glide (deadstick.py) and landing from it on the wheels, offline, with the
governor's authority below the lever's collective (gov/lo-deg, Bell's XV-15 -6 deg [CR-114614 Table CVIII]: the V-22's
is not public) made wider as an assumption, to find how much the glide and the landing need.

The pilot: the stick on the pitch attitude (KT per deg of its error, KQ per deg/s of the pitch rate, full travel), the
augmentation under it; the lever at flat pitch from the failure (the governor holding the rotor speed, windmilling).

  authority: from each glide's trim, 3000 ft up, the attitude held 40 s: the rotor speed's least, the pitch's spread,
  the stick's time at its stop, at each authority
  landing: from the glide 800 ft over flat ground at sea level, the gear down: at the flare height (the wheels') the
  pitch raised by the flare's angle at FLARE_DPS; at the cushion height the lever up (the collective the lever adds,
  from the rotors' speed). At the wheels' first touch: the descent rate, the ground speed, the pitch, the blade tips'
  clearance (deadstick.clearance at the nacelle angle plus the pitch), the rotor speed. Intact: the descent under the
  gear's GEAR_FPS and the tips clear by CLEAR_FT; the safest of those the least descent rate, then ground speed. With
  the tips not clear the proprotors strike: survivable under SURVIVE_FPS, MIL-STD-1290A's rotorcraft crash criterion
  (42 ft/s vertical, the high percentile of survivable accidents [NASA NTRS 20060020751]).

    python3 deadland.py authority
    python3 deadland.py landing LO [nacelle ...]       LO: the governor's authority below the lever, deg
    python3 deadland.py hover                          from a hover at HOVER_FT: the least height for a survivable one
    python3 deadland.py auto [setting=value ...]       ENG OUT (genv22.eng_out) flying it, coupled, from STARTS; the
                                                       settings ap/eo-* (or a property path) changed

The authority's answer (2026-11-02): wider makes it worse. Past Bell's -6 deg the governor takes the collective on
down as the rotor slows, the twisted blades' outer parts stall negatively and brake it, and the rotor speed falls the
further (to nothing by -20 deg), its control of the pitch with it: at 45-75 deg of nacelle the nose goes down with the
stick at its stop. With -6 deg the rotor holds 68-88 %; at 75 deg and 70 KCAS the glide is flown, the stick off its
stop.
The landing's (Bell's -6 deg, from 800 ft): none intact. Survivable only in airplane mode: the glide at 160 KCAS,
the nose up 15 deg from 200 ft, on the wheels at 7.7 ft/s and 132 kt, the proprotors 5.8 ft into the ground. At 75
deg the best touches at 81 ft/s, in helicopter mode (the autorotation 7500-9000 ft/min) at 95 ft/s. The lever's
cushion adds nothing in airplane mode: Bell's lever has no collective there (its gradient nil at the mast's 90 deg).
From a hover (hover): survivable only from 6000 ft (19.6-32.7 ft/s, the flare from 200 ft), 47.8 ft/s at best from
5000, against the 1600 ft below which DOT&E found it unlikely: the conversion costs about 2400 ft (the corridor holds
the nacelles till the speed comes, the rotor speed sags to a quarter in it, its pitch control with it), the pull out
of the dive and the glide's setting up with this pilot another 1000 ft or more.
Those touchdowns at 13 deg of pitch would also have struck the tail: the fuselage meets the ground at 7.3 deg on the
wheels (v22data.TAIL_STRIKE_DEG).

ENG OUT (auto, 2026-11-02; genv22.eng_out, its settings swept). Intact, at the reference loading over flat ground at
sea level, from: cruise (250 KCAS, 3000 ft), airplane mode at 200 and 180 KCAS from 1500 ft, the conversion (60 deg,
110 KCAS) from 4000 ft, a hover from 5500-6000 ft and up: on the wheels at 7.4-11 ft/s and 161-164 kt over the
ground, the pitch 6.0-6.5 deg, the nacelles 43-47 deg (the tips 3.5-4.4 ft clear), the flare 0.5-0.75 NM from its
height to the wheels. No slower: held off at 6 deg the wing carries the aircraft to about 150 kt (prototypes, full
flap worse: its drag). Not survivable from 160 KCAS at 800 ft (57 ft/s), the conversion at 2000 ft (61), a hover at
3000 ft (89); from a hover at 4000-5000 ft it swings with the pull out's timing at the flare, 36-70 ft/s. What
mattered: the flare's float with the nacelles coming up (from the glide's trim, the airplane mode's flare struck the
proprotors and the tail); the pitch's limit held on the attitude (the augmentation alone let the nacelles' moment take
it 1.2-1.5 deg over); the glide at 200 KCAS (180 worse); the pull out of the dive limited 300 ft over the flare. The
augmentation's own authority, half the stick: with the whole the dive steepened and the pull out came too late.
Converting on AUTO NAC's schedule kept the nacelles up and the speed never came."""
import sys, math, itertools
from multiprocessing import Pool
import hoverhold as HH
import deadstick as DS

KT, KQ = 0.1, 0.05
FLARE_DPS = 4.0
CLEAR_FT = 1.0
SURVIVE_FPS = 42.0
LEVER = '/controls/engines/engine[0]/throttle'
# the glides to start from at each nacelle angle: KCAS and the zero-power flight path deadstick.py's trims gave (at the
# reference loading; those with a residual over 0.02 only a start, the flight settling from it); in helicopter mode
# level flight, the autorotation setting itself up after the failure
GLIDES = {0.0: [(160, -8.40), (200, -6.76)], 45.0: [(110, -7.46), (130, -15.20)], 60.0: [(90, -11.37), (110, -11.29)],
          75.0: [(70, -25.12), (110, -13.48)], 90.0: [(40, 0.0), (60, 0.0)]}
AUTHORITY = (-6.0, -10.0, -15.0, -20.0, -25.0, -30.0)
FLARE_FT = (100.0, 200.0, 300.0, 400.0)
FLARE_DEG = (5.0, 10.0, 15.0, 20.0)
CUSHION_FT = (0.0, 15.0, 40.0)


def start(nac, V, gamma, lo, alt, terrain):
    f = HH.start(V, nac, alt=alt, terrain=terrain, gamma=gamma)
    f['gov/lo-deg'] = lo
    f['gear/gear-cmd-norm'] = 1.0; f['gear/gear-pos-norm'] = 1.0
    for e in (0, 1):
        f['/controls/engines/engine[%d]/cutoff' % e] = 1
    f[LEVER] = 0.0
    return f


def stick(f, ref_deg):
    th, q = f['attitude/theta-deg'], math.degrees(f['velocities/q-rad_sec'])
    s = max(-1.0, min(1.0, -(KT * (ref_deg - th) - KQ * q)))
    f['fcs/elevator-cmd-norm'] = s
    return s


def glide(args):
    nac, V, gamma, lo = args
    f = start(nac, V, gamma, lo, 3000.0, HH.H.TERRAIN_FT)
    th0 = f['attitude/theta-deg']
    dt = f['simulation/dt']
    nr_min, th_lo, th_hi, sat, n = 999.0, 99.0, -99.0, 0, 0
    for i in range(int(40 / dt)):
        s = stick(f, th0)
        f.run()
        if i % 48 == 0 and i * dt > 5:
            nr_min = min(nr_min, f['drive/nr-pct']); th = f['attitude/theta-deg']
            th_lo, th_hi = min(th_lo, th), max(th_hi, th)
            sat += abs(s) > 0.99; n += 1
    return args, dict(nr_min=nr_min, pitch=(th_lo - th0, th_hi - th0), sat=sat / max(1, n), sink=-f['velocities/h-dot-fps'],
                      kcas=f['velocities/vc-kts'], gov=f['gov/int-deg'])


def land(args):
    nac, V, gamma, lo, flare_ft, flare_deg, cushion_ft = args
    f = start(nac, V, gamma, lo, 800.0, 0.0)
    th0 = f['attitude/theta-deg']
    ref = th0
    dt = f['simulation/dt']
    flaring = False
    for i in range(int(120 / dt)):
        wheels = f['/position/gear-agl-m'] / 0.3048
        if wheels <= flare_ft:
            flaring = True
        if flaring:
            ref = min(th0 + flare_deg, ref + FLARE_DPS * dt)
        stick(f, ref)
        if cushion_ft and wheels <= cushion_ft:
            f[LEVER] = 1.0
        f.run()
        if sum(f['gear/unit[%d]/WOW' % j] for j in range(3)) > 0:
            pitch = f['attitude/theta-deg']
            return dict(args=args, sink=-f['velocities/h-dot-fps'], gs=f['velocities/vg-fps'] / 1.6878, pitch=pitch,
                        clear=DS.clearance(f['ctl/nacelle-deg'], pitch), nr=f['drive/nr-pct'], t=i * dt)
    return dict(args=args, sink=None)


# from a hover: the heights, the dive's pitch, the glide's speed and pitch per knot off it, the flare (the sweep's best)
HOVER_FT = (1600.0, 3000.0, 3500.0, 4000.0, 4500.0, 5000.0, 6000.0, 8000.0)
DIVE_DEG = (-10.0, -20.0, -30.0)
GLIDE_KCAS, GLIDE_PITCH, KV = 160.0, 0.2, 0.2      # (the pitch: deadstick.py's airplane mode glide trim there)
FLARE = ((200.0, 15.0), (300.0, 15.0))


def from_hover(args):
    """both engines out in a hover: after a second, the lever down, the nacelles commanded to 0 (the conversion
    protection letting them follow the speed), the nose down to the dive's pitch till GLIDE_KCAS, then the glide's
    pitch with KV deg per kt over it (the pull out); at the flare height the nose up by the flare's angle. The
    touchdown, and the height at which the nacelles reached 0"""
    h0, dive, (flare_ft, flare_deg) = args
    f = HH.start(0.0, 90.0, alt=h0, terrain=0.0)
    f['gear/gear-cmd-norm'] = 1.0; f['gear/gear-pos-norm'] = 1.0
    dt = f['simulation/dt']
    gliding = False
    ref, flare_from, h_apln, nr_min = f['attitude/theta-deg'], None, None, 999.0
    for i in range(int(180 / dt)):
        t = i * dt
        if abs(t - 1.0) < dt / 2:
            for e in (0, 1):
                f['/controls/engines/engine[%d]/cutoff' % e] = 1
        if t >= 2.0:
            f[LEVER] = 0.0
            f['/controls/flight/nacelle-cmd-deg'] = 0.0
            wheels = f['/position/gear-agl-m'] / 0.3048
            vc = f['velocities/vc-kts']
            if flare_from is None and wheels <= flare_ft:
                flare_from = f['attitude/theta-deg']
            if flare_from is not None:
                ref = min(flare_from + flare_deg, ref + FLARE_DPS * dt)
            elif not gliding and vc < GLIDE_KCAS - 10:
                ref = dive
            else:
                gliding = True
                ref = max(-30.0, min(15.0, GLIDE_PITCH + KV * (vc - GLIDE_KCAS)))
            stick(f, ref)
            if h_apln is None and f['ctl/nacelle-deg'] < 0.5:
                h_apln = wheels
            nr_min = min(nr_min, f['drive/nr-pct'])
        f.run()
        if t > 3 and sum(f['gear/unit[%d]/WOW' % j] for j in range(3)) > 0:
            pitch = f['attitude/theta-deg']
            return dict(args=args, sink=-f['velocities/h-dot-fps'], gs=f['velocities/vg-fps'] / 1.6878, pitch=pitch,
                        clear=DS.clearance(f['ctl/nacelle-deg'], pitch), nr=f['drive/nr-pct'], nac=f['ctl/nacelle-deg'],
                        h_apln=h_apln, nr_min=nr_min, t=t)
    return dict(args=args, sink=None)


# ENG OUT flown (genv22.eng_out, coupled): both engines cut a second in, the mode coming on with the failure; the
# crew's part the gear, down below the mode's gear height (or at once below it). The starts: (name, KCAS, nacelle,
# height ft, flight path deg)
STARTS = (('cruise', 250.0, 0.0, 3000.0, 0.0), ('airplane', 200.0, 0.0, 1500.0, 0.0), ('airplane 180', 180.0, 0.0, 1500.0, 0.0),
          ('airplane low', 160.0, 0.0, 800.0, 0.0), ('conversion', 110.0, 60.0, 2000.0, 0.0), ('conversion high', 110.0, 60.0, 4000.0, 0.0)) + \
         tuple(('hover %d' % h, 0.0, 90.0, float(h), 0.0) for h in (3000, 4000, 4500, 5000, 5500, 6000, 7000, 8000))
TAIL_DEG = DS.U.TAIL_STRIKE_DEG      # the fuselage's underside on the ground, the wheels on it


def auto(args):
    """one start (STARTS) with the mode's settings (ap/eo-*) changed by sets: the touchdown, the phases' heights"""
    (name, V, nac, h0, gamma), sets = args
    f = HH.start(V, nac, alt=h0, terrain=0.0, gamma=gamma)
    for k, x in sets.items():
        f[k if '/' in k else 'ap/eo-' + k] = x
    f['/instrumentation/mv22b/afcs/cpld'] = 1
    dt = f['simulation/dt']
    hts, last, nr_min, run_ft = {}, None, 999.0, 0.0
    for i in range(int(400 / dt)):
        t = i * dt
        if abs(t - 1.0) < dt / 2:
            for e in (0, 1):
                f['/controls/engines/engine[%d]/cutoff' % e] = 1
        wheels = f['/position/gear-agl-m'] / 0.3048
        if wheels < f['ap/eo-gear-ft'] and f['gear/gear-cmd-norm'] < 0.5:
            f['gear/gear-cmd-norm'] = 1.0
        ph = int(f['ap/eo-phase'])
        if ph != last:
            hts[ph] = (round(t, 1), round(wheels))
            last = ph
        if t > 2:
            nr_min = min(nr_min, f['drive/nr-pct'])
        if ph >= 3:
            run_ft += f['velocities/vg-fps'] * dt
        f.run()
        if t > 3 and sum(f['gear/unit[%d]/WOW' % j] for j in range(3)) > 0:
            pitch = f['attitude/theta-deg']
            return dict(args=args, sink=-f['velocities/h-dot-fps'], gs=f['velocities/vg-fps'] / 1.6878, pitch=pitch,
                        clear=DS.clearance(f['ctl/nacelle-deg'], pitch), nr=f['drive/nr-pct'], nac=f['ctl/nacelle-deg'],
                        nr_min=nr_min, t=t, phases=hts, gear=f['gear/gear-pos-norm'], on=f['ap/eo-fly'], run_nm=run_ft / 6076.1)
    return dict(args=args, sink=None, phases=hts)


def verdict(r):
    if r['sink'] is None:
        return 'not down'
    if r['sink'] < DS.GEAR_FPS and r['clear'] > CLEAR_FT and r['pitch'] < TAIL_DEG:
        return 'intact'
    return 'survivable' if r['sink'] < SURVIVE_FPS else 'not survivable'


def report(r):
    name = r['args'][0][0]
    ph = ' '.join('%d@%.0fs/%dft' % (k, *v) for k, v in sorted(r['phases'].items()))
    if r['sink'] is None:
        return '%-12s not down in 400 s; phases %s' % (name, ph)
    return ('%-12s %-14s sink %5.1f ft/s, %3.0f kt, pitch %5.1f, nacelles %4.1f (tips %+5.1f ft), Nr %5.1f (least %5.1f), '
            '%5.1f s, the flare %.2f NM; phases %s' % (name, verdict(r), r['sink'], r['gs'], r['pitch'], r['nac'], r['clear'], r['nr'],
                                                     r['nr_min'], r['t'], r['run_nm'], ph))


def intact(r):
    return r['sink'] is not None and r['sink'] < DS.GEAR_FPS and r['clear'] > CLEAR_FT


if __name__ == '__main__':
    if sys.argv[1:2] == ['authority']:
        jobs = [(n, V, g, lo) for n in (45.0, 60.0, 75.0) for V, g in GLIDES[n] for lo in AUTHORITY]
        with Pool() as pool:
            res = pool.map(glide, jobs)
        print('the glide held 40 s, the pilot holding the attitude: authority, the least rotor speed, the pitch\'s spread, '
              'the stick at its stop, then the descent and speed')
        for (n, V, g, lo), r in res:
            print('nacelle %2.0f %3.0f KCAS, authority %4.0f deg: Nr least %5.1f, pitch %+5.1f..%+5.1f, stick at its stop %3.0f %%, '
                  'at 40 s %5.1f ft/s %5.1f KCAS, governor %6.2f' % (n, V, lo, r['nr_min'], *r['pitch'], 100 * r['sat'], r['sink'],
                                                                     r['kcas'], r['gov']), flush=True)
    elif sys.argv[1:2] == ['hover']:
        jobs = list(itertools.product(HOVER_FT, DIVE_DEG, FLARE))
        with Pool() as pool:
            res = pool.map(from_hover, jobs)
        print('both engines out in a hover: the dive, the conversion, the glide at %.0f KCAS, the flare' % GLIDE_KCAS)
        for r in res:
            h0, dive, (fh, fd) = r['args']
            if r['sink'] is None:
                print('%5.0f ft, dive %3.0f deg, flare at %3.0f ft: not down in 180 s' % (h0, dive, fh))
                continue
            print('%5.0f ft, dive %3.0f deg, flare at %3.0f ft: on the ground after %5.1f s at %5.1f ft/s%s, %3.0f kt, pitch %5.1f, '
                  'nacelles %4.1f (tips %+5.1f ft), Nr %5.1f (least %5.1f); airplane mode %s' % (
                      h0, dive, fh, r['t'], r['sink'], ' (survivable)' if r['sink'] < SURVIVE_FPS else '', r['gs'], r['pitch'],
                      r['nac'], r['clear'], r['nr'], r['nr_min'],
                      'at %.0f ft' % r['h_apln'] if r['h_apln'] is not None else 'not reached'), flush=True)
    elif sys.argv[1:2] == ['auto']:
        sets = dict((kv.split('=')[0], float(kv.split('=')[1])) for kv in sys.argv[2:])
        with Pool() as pool:
            res = pool.map(auto, [(st, sets) for st in STARTS])
        print('ENG OUT coupled, both engines cut at 1 s%s' % (': ' + ' '.join(sys.argv[2:]) if sets else ''))
        for r in res:
            print(report(r), flush=True)
    elif sys.argv[1:2] == ['landing']:
        lo = float(sys.argv[2])
        nacs = [float(a) for a in sys.argv[3:]] or sorted(GLIDES)
        jobs = [(n, V, g, lo, fh, fd, ch) for n in nacs for V, g in GLIDES[n]
                for fh, fd, ch in itertools.product(FLARE_FT, FLARE_DEG, CUSHION_FT)]
        with Pool() as pool:
            res = pool.map(land, jobs)
        for n in nacs:
            rs = [r for r in res if r['args'][0] == n and r['sink'] is not None]
            ok = sorted([r for r in rs if intact(r)], key=lambda r: (r['sink'], r['gs']))
            print('nacelle %.0f, authority %.0f deg: %d landings, %d intact, %d survivable' % (
                n, lo, len(rs), len(ok), sum(1 for r in rs if r['sink'] < SURVIVE_FPS)), flush=True)
            for r in (ok[:5] if ok else sorted(rs, key=lambda r: r['sink'])[:3]):
                _, V, g, _, fh, fd, ch = r['args']
                print('  %3.0f KCAS, flare at %3.0f ft by %2.0f deg, cushion %2.0f ft: sink %5.1f ft/s, %3.0f kt over the ground, '
                      'pitch %5.1f, tips clear %5.1f ft, Nr %5.1f' % (V, fh, fd, ch, r['sink'], r['gs'], r['pitch'], r['clear'],
                                                                     r['nr']), flush=True)
