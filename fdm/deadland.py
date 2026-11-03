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
    python3 deadland.py site                           what ENG OUT's landing site must give (SITE_W, SURFACES):
                                                       -> data/eo_site_v22.json
    python3 deadland.py siteguide                      ENG OUT flown to sites set by hand (SITE_CASES): where the
                                                       flare began against the planned point -> eo_site_v22.json
    python3 deadland.py terrain                        ENG OUT's straight-in to a site over uneven ground
                                                       (TERRAIN): the flare on the radar height and on the site's plane
    python3 deadland.py flare [quick] [setting=value ...]   ENG OUT's flare on its own, from a straight glide, across
                                                       the weight, the field's air, the wind and the ground's slope
                                                       (FLARE_*; quick: in still air from 2000 ft)

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
Converting on AUTO NAC's schedule kept the nacelles up and the speed never came.

The flare across the weight, the field's air, the wind and the slope (flare, 2026-11-03, after FlightGear's landings in
Kansas ballooned and dropped in at 23-32 ft/s). The flare tuned above, over 243 flares (38 700-54 000 lb; sea level,
2200 ft at ISA + 15, 5000 ft at ISA + 20; 13 kt behind to 15 ahead; 1 % up, level, 1 % down), the descent against the
ground and the tips rolled on: 61 intact, 13.3 ft/s the median, 40.3 the worst, 9 balloons. Density altitude and
weight did it, the steady wind nothing. Where the corridor let them (light, thin air), the nacelles came up at the
flare's start, braked and lifted the aircraft 200 ft up, the path then asked for nothing while the speed bled and the
descent grew back, and the flare came late and slow; where the flare began at the bottom of the dive guard's swing,
the attitude was held until the path met the descent and the pull at 150 ft ballooned; on the wheels ENG OUT let go,
the nose came down and the nacelles stayed where the corridor had them at touchdown, the tips in the ground from about
165 KEAS. Now: the path's time constant fitted to the descent at the flare's start, the path judged against the ground
(a site's plane falling away followed down) and led by 3 ft/s, the nacelles from 100 ft to 60 deg, the flare's
integral halved, and rolled out on the wheels until the nacelles reach 45 deg: 151 intact, 8.9 ft/s the median, 16.5
the worst, no balloons, none past survivable. 38 700 lb all 81; 47 500 lb all 54 on level and falling ground (5.5-11.3
ft/s), on 1 % up the touchdown at 174-187 KCAS with the nacelles held low and the tips in the ground (the finder
refuses rising ground past 0.5 %); 54 000 lb 16 of 81, the touchdown at 172-179 KCAS, the corridor's 25-33 deg too
few for the tips. At the reference loading (auto) 5.3-5.6 ft/s at 169 kt where the old flare made 7.4-7.8 at 161, the
tips 1.1-1.4 ft clear rolling (2.3 before): softer, and faster, the corridor's nacelles fewer. Earlier slope results
here read the descent in still space: 2.3 ft/s harder on 1 % down, softer on 1 % up, than the gear meets.
ENG OUT's own flare (2026-11-03): the flaperons from the flare's start at 2 deg/s to 40 (faster ballooned), the
nacelles to 45 deg (60 pitched the nose past the tail's limit on falling ground), the lever at flat pitch to the ground
(the cushion gone), on the wheels the nacelles to 60; the flaperons otherwise from the flight state (genv22.flap_law),
AUTO NAC's pitch limit with them in the glide. Over the 243: 210 intact, 7.7 ft/s the median, 10.6 the worst, no
balloons, none past survivable; 38 700 lb all 81 (5.8-9.5 ft/s); 47 500 lb all 54 on level and falling ground, 14 of
27 on 1 % up; 54 000 lb all 27 on level ground, 17 of 27 on 1 % down (the tail, the nose at 7.3-7.6 deg), 17 of 27 on
1 % up (the others at 173-179 KCAS, the nacelles 25-33 deg). With the glide on the least drag instead, its reach 17 %
longer at 60 500 lb (9.43 against 8.06 at 200 KCAS) and 4 % at 54 000: 186, the heavy glide shallower and the flare
fitted to it floating 1.5 s less, 4-5 kt faster on the wheels. With the flare's flaperons from the glide's few degrees
instead of 0: 198, the lift there without the pitch and its drag (1 % up at 54 000 lb 1 intact of 27). At the
reference loading (auto) 7.1-7.6 ft/s at 159 kt, the tips 2.9 ft clear rolling; from a hover intact from 4500 ft;
lower, the nose pitches up to 50 deg against the command's -20 as the nacelles come down, and the rotor speed is lost."""
import sys, os, math, json, itertools
from multiprocessing import Pool
import harness as H
import hoverhold as HH
import deadstick as DS

KT, KQ = 0.1, 0.05
FLARE_DPS = 4.0
CLEAR_FT = 1.0
SURVIVE_FPS = 42.0
ROLL_S = 8.0            # rolled on after touchdown, the brakes on from 2 s: the tips' least clearance as the nose comes down
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
# ENG OUT's landing site (site): the ground the landing needs and the reach's error near its end, from a glide at
# 200 KCAS from 3000 ft over flat ground at sea level (both engines cut at 1 s, the gear down at the mode's height, the
# brakes full 2 s after touchdown), at the weights deadstick.py's glide table spans; on pavement and on FlightGear's
# grass and crop ground, as its groundreactions.cxx hands a material to JSBSim (the friction factor; the rolling
# friction / 0.02; the bumpiness: Grass, Airport 0.7, 0.1, 0.1; DryCrop, IrrCrop, MixedCrop 0.9, 0.1, 0.6-0.7). And
# the planned turn's cost: the bank held at HAC_FRAC of ENG OUT's limit (ap/eo-bank-deg: as WYPT's arcs, room left for
# their corrections), the height per 180 deg of heading
SITE_W = DS.TABLE_W
SURFACES = {'pavement': (1.0, 1.0, 0.0), 'grass': (0.7, 0.1 / 0.02, 0.1), 'crop': (0.9, 0.1 / 0.02, 0.7)}
HAC_FRAC = 0.8
# ENG OUT's site guidance (siteguide, genv22.eng_out_site): from a trimmed 200 KCAS start heading north at FlightGear's
# start loading, 38 730 lb (no cargo, the tanks half full), both engines cut at 1 s, the site set at 3 s: start height
# (ft), the site's point (where the flare is to begin) north and east of the start (NM), its course (deg true), the
# wind from the north (kt). The flare's start against the point, over the cases that began within SITE_NEAR_FT of the
# energy the site takes (larger surpluses than S-turns spend land long: a straight-in from 4000 ft 1.8 NM high flared
# 1.3 NM past the point, the circle's turns are whole): the guidance's own scatter, the ground a site must add
SITE_CASES = {'straight 3 NM': (3000, 3.0, 0.0, 0, 0), 'base 180': (5000, 2.0, 1.0, 180, 0), 'behind': (6500, -1.0, 0.0, 0, 0),
              'high 6000': (6000, 2.0, 0.0, 0, 0), 'headwind 20': (3000, 3.0, 0.0, 0, 20), 'cross base': (5000, 2.0, -1.0, 90, 0),
              'straight 4000': (4000, 3.0, 0.0, 0, 0), 'side 30': (4000, 3.0, 1.5, 330, 0)}
SITE_W_FG = 38730.0
SITE_NEAR_FT = 6076.1
# ground profiles under the landing (terrain): elevation (ft) against the way along the site's line from the flare's
# point (ft, + on), level before it; Kansas's from FlightGear's flights there (2026-11-03: the wheels' radar height and
# the integrated descent), the first rising as the flare began, the third falling away under it
def _interp(xs, ys):
    return lambda x: ys[0] if x <= xs[0] else ys[-1] if x >= xs[-1] else next(
        ys[i - 1] + (ys[i] - ys[i - 1]) * (x - xs[i - 1]) / (xs[i] - xs[i - 1]) for i in range(1, len(xs)) if x <= xs[i])
TERRAIN = {'flat': lambda x: 0.0, 'up 1 %': lambda x: 0.01 * max(0.0, x), 'up 2 %': lambda x: 0.02 * max(0.0, x),
           'down 1 %': lambda x: -0.01 * max(0.0, x), 'down 2 %': lambda x: -0.02 * max(0.0, x),
           'bumps 10 ft': lambda x: 10 * math.sin(2 * math.pi * x / 1200) if x > 0 else 0.0,
           'Kansas 1': _interp([466, 927, 1353, 1794, 2171, 2628, 3025, 3400, 3723, 4058, 4357],
                               [0, 1.1, 1.8, 1.3, 0.2, 10.0, 8.5, 13.5, 10.7, 7.8, 10.8]),
           'Kansas 3': _interp([-131, 288, 721, 1162, 1557, 1917, 2328, 2716, 3042, 3393, 3683, 4013],
                               [0, -4.1, -16.7, -8.6, -11.7, -14.1, -20.3, -30.1, -31.6, -36.2, -43.0, -41.5])}
# the strip the finder would have sampled for the plane: from where the wheels may first come under 50 ft to the stop
# (eosite.nas's needs at FlightGear's start loading: the scatter's -804 ft, 1966 ft to 50 ft, 4929 ft long), every 150 m
STRIP_FT = (-804 + 1966, -804 + 1966 + 4929)


def auto(args):
    """one start (STARTS) with the mode's settings (ap/eo-*) changed by sets: the touchdown, the phases' heights"""
    (name, V, nac, h0, gamma), sets = args
    f = HH.start(V, nac, alt=h0, terrain=0.0, gamma=gamma)
    for k, x in sets.items():
        f[k if '/' in k else 'ap/eo-' + k] = x
    f['/instrumentation/mv22b/afcs/cpld'] = 1
    dt = f['simulation/dt']
    hts, last, nr_min, run_ft, td = {}, None, 999.0, 0.0, None
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
        wow = t > 3 and sum(f['gear/unit[%d]/WOW' % j] for j in range(3)) > 0
        if wow and td is None:
            pitch = f['attitude/theta-deg']
            td = dict(args=args, sink=-f['velocities/h-dot-fps'], gs=f['velocities/vg-fps'] / 1.6878, pitch=pitch,
                      clear=DS.clearance(f['ctl/nacelle-deg'], pitch), nr=f['drive/nr-pct'], nac=f['ctl/nacelle-deg'],
                      nr_min=nr_min, t=t, phases=hts, gear=f['gear/gear-pos-norm'], on=f['ap/eo-fly'], run_nm=run_ft / 6076.1,
                      roll_clear=99.0)
        if td is not None:
            roll_on(f, td, t, wow)
            if t - td['t'] > ROLL_S:
                return td
    return td or dict(args=args, sink=None, phases=hts)


def roll_on(f, r, t, wow):
    """after touchdown (r['t']): the brakes on from 2 s; the tips' least clearance on the wheels (r['roll_clear'])"""
    if t - r['t'] > 2:
        f['/controls/gear/brake-left'] = f['/controls/gear/brake-right'] = 1.0
    if wow:
        r['roll_clear'] = min(r['roll_clear'], DS.clearance(f['ctl/nacelle-deg'], f['attitude/theta-deg']))


H_LOAD = H.load


def site_start(W):
    H.load = lambda *a, **k: DS.weigh(H_LOAD(*a, **k), W)          # (HH.start's trim at this weight)
    f = HH.start(200.0, 0.0, alt=3000.0, terrain=0.0)
    f['/instrumentation/mv22b/afcs/cpld'] = 1
    return f


def site_landing(args):
    """the distances past the flare's start (the gate): the wheels under 50 ft, on the ground, stopped; and the reach's
    error (ft, + long) in the last NM before the gate"""
    W, surface = args
    f = site_start(W)
    for k, x in zip(('static-friction-factor', 'rolling_friction-factor', 'bumpiness'), SURFACES[surface]):
        f['ground/' + k] = x
    dt = f['simulation/dt']
    x, ev, reach = 0.0, {}, []
    for i in range(int(300 / dt)):
        t = i * dt
        if abs(t - 1.0) < dt / 2:
            for e in (0, 1):
                f['/controls/engines/engine[%d]/cutoff' % e] = 1
        h = f['/position/gear-agl-m'] / 0.3048
        if h < f['ap/eo-gear-ft'] and f['gear/gear-cmd-norm'] < 0.5:
            f['gear/gear-cmd-norm'] = 1.0
        if 'gate' not in ev:
            if f['ap/eo-phase'] > 2.5:
                ev['gate'] = x
            elif t > 5 and i % 30 == 0:
                reach.append((x, f['ap/eo-reach-nm'] * 6076.1))
        elif 'h50' not in ev and h < 50:
            ev['h50'] = x
        wow = sum(f['gear/unit[%d]/WOW' % j] for j in range(3)) > 0
        if 'gate' in ev and 'td' not in ev and wow:
            ev['td'], ev['t_td'], ev['kt'] = x, t, f['velocities/vg-fps'] / 1.6878
        if 'td' in ev and t > ev['t_td'] + 2:
            f['fcs/left-brake-cmd-norm'] = 1.0
            f['fcs/right-brake-cmd-norm'] = 1.0
        if 'td' in ev and f['velocities/vg-fps'] < 1.0:
            ev['stop'] = x
            break
        x += f['velocities/vg-fps'] * dt
        f.run()
    g = ev['gate']
    err = [r - (g - x0) for x0, r in reach if g - x0 < 6076.1]
    return dict(W=W, surface=surface, h50=ev['h50'] - g, td=ev['td'] - g, stop=ev['stop'] - g, kt=ev['kt'],
                err=(min(err), max(err)))


def site_turn(W):
    """the height per 180 deg of heading in the planned turn at the glide's speed"""
    f = site_start(W)
    f['ap/eo-bank-deg'] = HAC_FRAC * f['ap/eo-bank-deg']
    dt = f['simulation/dt']
    turned, last, h0 = 0.0, None, None
    for i in range(int(56 / dt)):
        t = i * dt
        if abs(t - 1.0) < dt / 2:
            for e in (0, 1):
                f['/controls/engines/engine[%d]/cutoff' % e] = 1
        if t > 14:
            f['/instrumentation/mv22b/afcs/hdg-hold'] = 1
            f['/autopilot/settings/heading-bug-deg'] = (f['attitude/psi-deg'] + 90 - f['/environment/magnetic-variation-deg']) % 360
        if t > 25:
            if h0 is None:
                h0, t0 = f['/position/gear-agl-m'] / 0.3048, t
            turned += abs((f['attitude/psi-deg'] - last + 540) % 360 - 180)
        last = f['attitude/psi-deg']
        f.run()
    dh = h0 - f['/position/gear-agl-m'] / 0.3048
    return dict(W=W, bank=f['ap/eo-bank-deg'], ft180=dh / turned * 180, kcas=f['velocities/vc-kts'], rate=turned / (t - t0))


def site():
    jobs = [('land', (W, s)) for W in SITE_W for s in SURFACES] + [('turn', W) for W in SITE_W]
    with Pool(8) as pool:
        res = pool.map(site_job, jobs)
    land_ = {(r['W'], r['surface']): r for r in res if 'surface' in r}
    turn_ = {r['W']: r for r in res if 'ft180' in r}
    pav = [land_[(W, 'pavement')] for W in SITE_W]
    out = {'weight_lb': list(SITE_W), 'hac_frac': HAC_FRAC, 'hac_bank_deg': turn_[SITE_W[0]]['bank'],
           'turn_ft_per_180': [round(turn_[W]['ft180']) for W in SITE_W],
           'gate_to_50ft_ft': [round(r['h50']) for r in pav], 'gate_to_touchdown_ft': [round(r['td']) for r in pav],
           'touchdown_kt': [round(r['kt'], 1) for r in pav],
           'reach_err_ft': [[round(r['err'][0]), round(r['err'][1])] for r in pav],
           'rollout_ft': {s: [round(land_[(W, s)]['stop'] - land_[(W, s)]['td']) for W in SITE_W] for s in SURFACES}}
    for W in SITE_W:
        r, t = land_[(W, 'pavement')], turn_[W]
        print('%6.0f lb: the turn at %.0f deg %4.0f ft per 180 deg (%.1f deg/s); past the gate 50 ft at %4.0f ft, down at %4.0f ft '
              '(%3.0f kt), stopped at %4.0f ft; on the ground %s; the reach in the last NM %+.0f..%+.0f ft' % (
                  W, t['bank'], t['ft180'], t['rate'], r['h50'], r['td'], r['kt'], r['stop'],
                  ', '.join('%s %4.0f ft' % (s, land_[(W, s)]['stop'] - land_[(W, s)]['td']) for s in SURFACES), *r['err']), flush=True)
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'eo_site_v22.json')
    json.dump(out, open(path, 'w'), indent=1)
    print('wrote', path)


def site_guide(name):
    """one SITE_CASES flight: the margin when the site is set, the flare's start and the touchdown against the point
    and the line"""
    h0, n_nm, e_nm, crs, wind = SITE_CASES[name]
    H.load = lambda *a, **k: DS.weigh(H_LOAD(*a, **k), SITE_W_FG)
    f = HH.start(200.0, 0.0, alt=float(h0), terrain=0.0, wind_kt=wind, wind_from=0.0)
    f['/instrumentation/mv22b/afcs/cpld'] = 1
    lat0, lon0 = f['position/lat-geod-deg'], f['position/long-gc-deg']
    f['ap/eo-site-lat-deg'] = lat0 + n_nm / 60.0
    f['ap/eo-site-lon-deg'] = lon0 + e_nm / 60.0 / math.cos(math.radians(lat0))
    f['ap/eo-site-course-deg'] = crs
    dt = f['simulation/dt']
    r = dict(name=name)
    for i in range(int(400 / dt)):
        t = i * dt
        if abs(t - 1.0) < dt / 2:
            for e in (0, 1):
                f['/controls/engines/engine[%d]/cutoff' % e] = 1
        if abs(t - 3.0) < dt / 2:
            f['ap/eo-site-on'] = 1
        if 'margin' not in r and t > 3.1:
            r['margin'] = f['ap/eo-site-margin-ft']
        h = f['/position/gear-agl-m'] / 0.3048
        if h < f['ap/eo-gear-ft'] and f['gear/gear-cmd-norm'] < 0.5:
            f['gear/gear-cmd-norm'] = 1.0
        if 'gate' not in r and f['ap/eo-phase'] > 2.5:
            r['gate'] = (f['ap/eo-site-x-ft'], f['ap/eo-site-y-ft'], f['attitude/phi-deg'])
        f.run()
        if t > 3 and sum(f['gear/unit[%d]/WOW' % j] for j in range(3)) > 0:
            r['td'] = (f['ap/eo-site-x-ft'], f['ap/eo-site-y-ft'], -f['velocities/h-dot-fps'], f['velocities/vg-fps'] / 1.6878)
            break
    return r


def terrain_run(args):
    """the straight-in from 3000 ft at 200 KCAS over a TERRAIN profile, the flare on the radar height or on the plane
    fitted to the strip"""
    name, plane = args[:2]
    sets, W, elev, dT = (tuple(args[2:]) + ({}, SITE_W_FG, 0.0, 0.0)[len(args) - 2:])[:4]
    g = lambda x, g0=TERRAIN[name]: elev + g0(x)
    H.load = lambda *a, **k: DS.weigh(H_LOAD(*a, **k), W)
    f = HH.start(200.0, 0.0, alt=3000.0 + elev, terrain=elev)
    f['atmosphere/delta-T'] = dT * 1.8
    f['/instrumentation/mv22b/afcs/cpld'] = 1
    lat0, lon0 = f['position/lat-geod-deg'], f['position/long-gc-deg']
    f['ap/eo-site-lat-deg'], f['ap/eo-site-lon-deg'], f['ap/eo-site-course-deg'] = lat0 + 3.0 / 60.0, lon0, 0.0
    f['ap/eo-site-on'] = 1
    for k, x in sets.items():
        f[k if '/' in k else 'ap/eo-' + k] = x
    if plane:
        xs = [STRIP_FT[0] + i * 492.1 for i in range(int((STRIP_FT[1] - STRIP_FT[0]) / 492.1) + 1)]
        ys = [g(x) for x in xs]
        n, mx, my = len(xs), sum(xs) / len(xs), sum(ys) / len(ys)
        sl = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
        f['ap/eo-site-plane'], f['ap/eo-site-elev-ft'], f['ap/eo-site-slope'] = 1, my - sl * mx, sl
        f['ap/eo-site-end-ft'] = STRIP_FT[1]
    dt = f['simulation/dt']
    pmax, td, flare = -99.0, None, None
    for i in range(int(300 / dt)):
        t = i * dt
        if abs(t - 1.0) < dt / 2:
            for e in (0, 1):
                f['/controls/engines/engine[%d]/cutoff' % e] = 1
        if t > 2:
            f['position/terrain-elevation-asl-ft'] = g(f['ap/eo-site-x-ft'])
        if f['/position/gear-agl-m'] / 0.3048 < f['ap/eo-gear-ft'] and f['gear/gear-cmd-norm'] < 0.5:
            f['gear/gear-cmd-norm'] = 1.0
        if f['ap/eo-phase'] > 2.5:
            pmax = max(pmax, f['attitude/theta-deg'])
            if flare is None:
                flare = f['ap/eo-site-x-ft']
        # (the descent against the ground: the profile's rate at the speed along it added)
        x_ = f['ap/eo-site-x-ft']
        rise = (g(x_ + 1.0) - g(x_ - 1.0)) / 2.0 * f['velocities/v-north-fps'] if t > 2 else 0.0
        last = (-f['velocities/h-dot-fps'] + rise, f['attitude/theta-deg'], f['velocities/vg-fps'] / 1.6878, x_)
        f.run()
        if t > 3 and sum(f['gear/unit[%d]/WOW' % j] for j in range(3)) > 0:
            td = last
            break
    return name, plane, flare, pmax, td


def terrain_runs(sets={}):
    jobs = [(n, p, sets) for n in TERRAIN for p in (False, True)]
    with Pool(8) as pool:
        res = pool.map(terrain_run, jobs)
    for name, plane, flare, pmax, td in res:
        print('%-12s %-6s the flare from %+6.0f ft, its pitch up to %4.1f; down %s' % (
            name, 'plane' if plane else 'radar', flare if flare is not None else float('nan'), pmax,
            '%5.1f ft/s, pitch %4.1f, %3.0f kt over the ground, %+5.0f ft' % td if td else 'not'), flush=True)


def site_guides():
    with Pool(min(8, len(SITE_CASES))) as pool:
        res = pool.map(site_guide, list(SITE_CASES))
    near = []
    for r in res:
        g, td = r.get('gate'), r.get('td')
        ok = td is not None and td[2] < DS.GEAR_FPS
        print('%-14s margin %+6.0f ft: the flare %s; down %s%s' % (
            r['name'], r['margin'], g and '%+6.0f ft along, %+5.0f across, banked %5.1f' % g,
            td and '%+6.0f along, %+5.0f across, %4.1f ft/s, %3.0f kt' % td, '' if ok else '  (not intact)'), flush=True)
        if ok and abs(r['margin']) <= SITE_NEAR_FT:
            near.append(g[0])
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'eo_site_v22.json')
    d = json.load(open(path))
    d['flare_err_ft'] = [round(min(near)), round(max(near))]
    d['flare_err_cases'] = [[r['name'], round(r['margin']), r.get('gate') and round(r['gate'][0])] for r in res]
    json.dump(d, open(path, 'w'), indent=1)
    print('the flare\'s start %+.0f..%+.0f ft from the point over %d cases within %.0f ft of the site\'s energy; wrote %s' % (
        min(near), max(near), len(near), SITE_NEAR_FT, path))


def site_job(j):
    return site_landing(j[1]) if j[0] == 'land' else site_turn(j[1])


# ENG OUT's flare on its own (flare): from a straight glide at 200 KCAS FLARE_H0 over the field (no site: the heading
# held, north), the engines cut at 1 s; the weights, the field's air (its elevation, ISA + deg C), a steady wind along
# the landing (kt, + head), the ground's slope from where the flare begins (+ rising). The glide's starts set where the
# dive's pull-out guard (its swing about ap/eo-sink-ok-fps) meets the flare's height
FLARE_W = (38730.0, 47500.0, 54000.0)
FLARE_AIRS = ((0.0, 0.0), (2200.0, 15.0), (5000.0, 20.0))
FLARE_WINDS = (-13.0, 0.0, 15.0)
FLARE_SLOPES = (0.0, 0.01, -0.01)
FLARE_H0 = (1800.0, 2000.0, 2200.0)


def flare_run(args):
    """one flare: its start (height, descent, KCAS); its pitch's peak and the height there; the balloon (the rise over
    its lowest point before touchdown); the most normal load before touchdown; the touchdown; rolled on (roll_on)"""
    W, elev, dT, wind, slope, h0, sets = args
    H.load = lambda *a, **k: DS.weigh(H_LOAD(*a, **k), W)
    f = HH.start(200.0, 0.0, alt=h0 + elev, terrain=elev, wind_kt=abs(wind), wind_from=0.0 if wind > 0 else 180.0)
    f['atmosphere/delta-T'] = dT * 1.8
    for k, x in sets.items():
        f[k if '/' in k else 'ap/eo-' + k] = x
    f['/instrumentation/mv22b/afcs/cpld'] = 1
    dt = f['simulation/dt']
    d, d_f, r, td = 0.0, None, dict(args=args, sink=None), None
    hmin, bal, pk, nz = 1e9, 0.0, (-99.0, 0.0), 0.0
    for i in range(int(300 / dt)):
        t = i * dt
        if abs(t - 1.0) < dt / 2:
            for e in (0, 1):
                f['/controls/engines/engine[%d]/cutoff' % e] = 1
        d += f['velocities/v-north-fps'] * dt
        if t > 2:
            f['position/terrain-elevation-asl-ft'] = elev + (slope * max(0.0, d - d_f) if d_f is not None else 0.0)
        hw = f['/position/gear-agl-m'] / 0.3048
        if hw < f['ap/eo-gear-ft'] and f['gear/gear-cmd-norm'] < 0.5:
            f['gear/gear-cmd-norm'] = 1.0
        if f['ap/eo-phase'] > 2.5 and td is None:
            if d_f is None:
                d_f, t_f = d, t
                r['flare'] = (hw, -f['velocities/h-dot-fps'], f['velocities/vc-kts'])
            if f['attitude/theta-deg'] > pk[0]:
                pk = (f['attitude/theta-deg'], hw)
            hmin = min(hmin, hw)
            bal = max(bal, hw - hmin)
            nz = max(nz, f['accelerations/Nz'])
        f.run()
        wow = t > 3 and sum(f['gear/unit[%d]/WOW' % j] for j in range(3)) > 0
        if wow and td is None:
            pitch = f['attitude/theta-deg']
            # (the descent against the ground, what the gear meets: the ground's own rate, its slope at the speed along it,
            # added; in still space it read 2.3 ft/s softer on 1 % up, 2.3 harder on 1 % down)
            rise = slope * f['velocities/v-north-fps'] if d_f is not None and d > d_f else 0.0
            r.update(sink=-f['velocities/h-dot-fps'] + rise, sink_space=-f['velocities/h-dot-fps'], gs=f['velocities/vg-fps'] / 1.6878,
                     kcas=f['velocities/vc-kts'], pitch=pitch,
                     clear=DS.clearance(f['ctl/nacelle-deg'], pitch), nac=f['ctl/nacelle-deg'], run=d - (d_f or d),
                     t_fl=t - (t_f if d_f is not None else t), t=t, roll_clear=99.0)
            td = r
        if td is not None:
            roll_on(f, r, t, wow)
            if t - r['t'] > ROLL_S:
                break
    r.update(peak=pk, balloon=bal, nz=nz)
    return r


def flare_runs(quick, sets):
    if quick:
        jobs = [(W, e, dT, 0.0, sl, 2000.0, sets) for W in FLARE_W for e, dT in FLARE_AIRS for sl in FLARE_SLOPES]
    else:
        jobs = [(W, e, dT, w, sl, h0, sets) for W in FLARE_W for e, dT in FLARE_AIRS for w in FLARE_WINDS for sl in FLARE_SLOPES
                for h0 in FLARE_H0]
    print('ENG OUT\'s flare from a straight glide at 200 KCAS%s' % (': ' + ' '.join('%s=%g' % kv for kv in sets.items()) if sets else ''))
    res = []
    with Pool(8) as pool:
        for r in pool.imap(flare_run, jobs):
            W, elev, dT, wind, slope, h0, _ = r['args']
            s = '%5.1fk lb %4.0f ft ISA%+3.0f %+4.0f kt %+3.0f %% %4.0f ft' % (W / 1000, elev, dT, wind, slope * 100, h0)
            if r.get('flare'):
                s += ': the flare at %3.0f ft %4.1f ft/s %3.0f KCAS' % r['flare']
            s += ', its pitch up to %4.1f at %3.0f ft, balloon %4.1f ft, %.2f g' % (*r['peak'], r['balloon'], r['nz'])
            if r['sink'] is not None:
                s += '; down %5.1f ft/s %3.0f KCAS, pitch %4.1f, nacelles %4.1f (tips %+5.1f ft, rolling %+5.1f), %5.0f ft, %4.1f s: %s' % (
                    r['sink'], r['kcas'], r['pitch'], r['nac'], r['clear'], r['roll_clear'], r['run'], r['t_fl'], verdict(r))
            print(s, flush=True)
            res.append(r)
    sinks = sorted(r['sink'] for r in res if r['sink'] is not None)
    print('%d flares: %d intact; down at %.1f ft/s median, %.1f at worst; balloons over 5 ft %d; up to %.2f g; tips rolling down to %+.1f ft' % (
        len(res), sum(verdict(r) == 'intact' for r in res), sinks[len(sinks) // 2], sinks[-1], sum(r['balloon'] > 5 for r in res),
        max(r['nz'] for r in res), min(r.get('roll_clear', 99.0) for r in res)))
    for W in FLARE_W:
        rs = [r for r in res if r['args'][0] == W]
        print('  %5.1fk lb: %d of %d intact' % (W / 1000, sum(verdict(r) == 'intact' for r in rs), len(rs)))


def verdict(r):
    if r['sink'] is None:
        return 'not down'
    if r['sink'] < DS.GEAR_FPS and r['clear'] > CLEAR_FT and r['pitch'] < TAIL_DEG and r.get('roll_clear', 99.0) > 0:
        return 'intact'
    return 'survivable' if r['sink'] < SURVIVE_FPS else 'not survivable'


def report(r):
    name = r['args'][0][0]
    ph = ' '.join('%d@%.0fs/%dft' % (k, *v) for k, v in sorted(r['phases'].items()))
    if r['sink'] is None:
        return '%-12s not down in 400 s; phases %s' % (name, ph)
    return ('%-12s %-14s sink %5.1f ft/s, %3.0f kt, pitch %5.1f, nacelles %4.1f (tips %+5.1f ft, rolling %+5.1f), Nr %5.1f '
            '(least %5.1f), %5.1f s, the flare %.2f NM; phases %s' % (name, verdict(r), r['sink'], r['gs'], r['pitch'], r['nac'], r['clear'],
                                                                    r['roll_clear'], r['nr'], r['nr_min'], r['t'], r['run_nm'], ph))


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
    elif sys.argv[1:2] == ['site']:
        site()
    elif sys.argv[1:2] == ['siteguide']:
        site_guides()
    elif sys.argv[1:2] == ['terrain']:
        terrain_runs(dict((kv.split('=')[0], float(kv.split('=')[1])) for kv in sys.argv[2:]))
    elif sys.argv[1:2] == ['flare']:
        quick = sys.argv[2:3] == ['quick']
        flare_runs(quick, dict((kv.split('=')[0], float(kv.split('=')[1])) for kv in sys.argv[3 if quick else 2:]))
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
