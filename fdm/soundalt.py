#!/usr/bin/env python3
"""The sounds' levels by altitude, from PyWopWop: its own levels of each load grid's loop (fdm/rotorsound.py) and each engine layer at
the same calibrated speed and load in thinner air, as a power of the density ratio each, for the sound file's <expression>: speed and
load are the loops', the air the expression's. No one power fits them all: the blades of a light rotor fall with the density, but at
the same calibrated speed the true speed and the tip Mach number rise, and a rotor pulling hard at speed gets louder with height. So a
power per loop, over the altitudes that loop can be flown at: its trim reached, its Mach number under the airplane mode's limit.

    python3 soundalt.py ~/git/PyWopWop [draft]
        writes data/sound_altitude_v22.json (quality draft by default: a difference of levels, not the loops' own sound)"""
import json, math, os, sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.expanduser(sys.argv[1]) if len(sys.argv) > 1 else '')
sys.path.insert(0, HERE)
import v22data as U
from pywopwop.aero import AeroOptions
from pywopwop.atmosphere import Atmosphere
from pywopwop.model import Observer, STANDARD_OBSERVERS, load_aircraft
from pywopwop.operating import operate
from pywopwop.synth import Quality, Variability, render
from pywopwop import bank as BK
from pywopwop import engine as EN

AC = 'mv22b'
AIRPLANE_M = (0.0, 1500.0, 3000.0, 4500.0, 6000.0)
HELICOPTER_M = (0.0, 1000.0, 2000.0, 3000.0)
ENGINE_M = (0.0, 1500.0, 3000.0, 4500.0, 6000.0)
ENGINE_NG = (65.0, 72.0, 80.0, 87.5, 92.0, 96.0, 100.0, 104.0)
STARTER_NG = (5.0, 24.0, 55.0)
QUALITY = sys.argv[2] if len(sys.argv) > 2 else 'draft'
LOOP_S = 2.0
OUT = os.path.join(HERE, 'data', 'sound_altitude_v22.json')


def sigma(alt):
    return float(Atmosphere(alt).rho / Atmosphere(0.0).rho)


def conditions():
    return BK.load_conditions('airplane_load') + BK.load_conditions('helicopter_load')


def flyable(c, alt):
    """under the airplane mode's Mach limit: at its calibrated speed the true speed goes up with height"""
    if c.nacelle_deg > 0:
        return True
    return c.airspeed_kt / math.sqrt(sigma(alt)) * 0.514444 / float(Atmosphere(alt).c) <= U.MMO_APLN


def blades_level(j):
    name, alt = j
    ac = load_aircraft(AC)
    c = next(x for x in conditions() if x.name == name)
    q = Quality.get(QUALITY)
    try:
        fc = BK.flight_condition(ac, c, alt)
        st = operate(ac, fc, AeroOptions(n_radial=q.n_radial, n_azimuth=q.n_azimuth))
        out = {}
        for o in (STANDARD_OBSERVERS['side'], Observer('cockpit', position=tuple(ac.cockpit))):
            cabin = ac.cabin if o.position is not None else None
            lv = render(ac, fc, o, fs=48000.0, duration=LOOP_S, quality=q, cabin=cabin, state=st, variability=Variability(seed=8),
                        pitch_headroom=1.25, environment='open').levels()
            out[o.name] = dict(blades=10 * math.log10(sum(10 ** (lv[k]['dba'] / 10) for k in ('layer/rotor', 'layer/swish') if k in lv)),
                               slap=float(lv['layer/slap']['dba']) if 'layer/slap' in lv else None)
        return name, alt, out
    except ValueError:                  # (no trim: more than the rotor can carry in that air)
        return name, alt, None


def power(alts, levels):
    """the power of the density ratio the amplitude goes as, by least squares through sea level's end of the data"""
    s = np.array([20 * math.log10(sigma(a)) for a in alts])
    return float(np.polyfit(s, np.array(levels), 1)[0])


def engines():
    ac = load_aircraft(AC)
    inst = EN.load_installations(AC, 90.0)[0]
    rows = {}
    for ng in ENGINE_NG:
        for view in EN._views(ac, 50.0):
            lv = []
            for alt in ENGINE_M:
                r = EN.render_engine(inst, EN.EngineCondition(ng), view, 48000.0, 1.0, 1, 'open', cabin=ac.cabin, pitch_headroom=1.3,
                                     hub_height=ac.hub_height_m, atm=Atmosphere(alt))
                lv.append({k.split('/')[-1]: float(v['dba']) for k, v in r.levels().items()})
            for layer in ('compressor', 'core', 'turbine'):
                rows.setdefault(layer, {}).setdefault(view.name, []).append(round(power(ENGINE_M, [x[layer] for x in lv]), 3))
    # (the starter on its own, up to where it cuts out; a start at a high field the only time it's heard away from sea level)
    for ng in STARTER_NG:
        for view in EN._views(ac, 50.0):
            lv = []
            for alt in ENGINE_M:
                r = EN.render_engine(inst, EN.EngineCondition(ng, lit=False, starter=True), view, 48000.0, 1.0, 1, 'open', cabin=ac.cabin,
                                     pitch_headroom=1.3, hub_height=ac.hub_height_m, atm=Atmosphere(alt))
                lv.append(float(r.levels()['layer/starter']['dba']))
            rows.setdefault('starter', {}).setdefault(view.name, []).append(round(power(ENGINE_M, lv), 3))
    return {k: dict(ng=list(STARTER_NG if k == 'starter' else ENGINE_NG), **v) for k, v in rows.items()}


def main():
    jobs = [(c.name, a) for c in conditions() for a in (AIRPLANE_M if c.nacelle_deg == 0 else HELICOPTER_M) if flyable(c, a)]
    print(len(jobs), 'loops and altitudes', flush=True)
    with Pool(3) as p:
        res = []
        for x in p.imap_unordered(blades_level, jobs):
            res.append(x)
            print(x[0], x[1], 'no trim' if x[2] is None else {k: round(v['blades'], 1) for k, v in x[2].items()}, flush=True)
    out = dict(source='PyWopWop, %s quality: each loop at the same calibrated speed and load over the altitudes it can be flown at' % QUALITY,
               blades={}, slap={'side': [], 'cockpit': []})
    for c in conditions():
        rows = sorted((a, o) for n, a, o in res if n == c.name and o)
        out['blades'][c.name] = dict(top_m=rows[-1][0] if rows else 0.0)
        for view in ('side', 'cockpit'):
            out['blades'][c.name][view] = round(power([a for a, _ in rows], [o[view]['blades'] for _, o in rows]), 3) if len(rows) > 1 else 0.0
            sl = [(a, o[view]['slap']) for a, o in rows if o[view]['slap'] is not None and o[view]['slap'] > 0]
            if c.nacelle_deg > 0 and len(sl) > 1:
                out['slap'][view].append(power([a for a, _ in sl], [x for _, x in sl]))
    out['slap'] = {k: round(float(np.mean(v)), 3) if v else 0.0 for k, v in out['slap'].items()}
    out['engines'] = engines()
    json.dump(out, open(OUT, 'w'), indent=1)
    print('wrote', OUT)


if __name__ == '__main__':
    main()
