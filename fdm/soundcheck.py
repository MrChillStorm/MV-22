#!/usr/bin/env python3
"""The banks' loops in MV-22B/Sounds/v22-sound.xml (fdm/rotorsound.py writes them) as SimGear plays their pitch:
  - every bank loop's <pitch> states its <offset>: SimGear plays a pitch block at its offset (1.0 where none is given) plus
    its factor times its property within <min> and <max>, multiplying the blocks' terms and adding their offsets, and holds
    the sum within 0.01 and 2.0 (simgear/sound/xmlsound.cxx, sample.hxx);
  - over random states of the flight model, every loop plays at its own pitch (its speed over the one it was rendered at:
    the factor times the property) wherever it can be heard, and none would need more than SimGear's 2.0;
  - that the random states reach every pitched loop, so that the check covers them all.
A loop is heard where its condition holds and its volume is within 60 dB of the most it reaches over the states.

    python3 soundcheck.py [FILE]        (FILE: another sound file than the aircraft's)
"""
import json
import os
import sys
import numpy as np
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(HERE, '..', 'MV-22B', 'Sounds', 'v22-sound.xml')
HEARD = 1e-3
RPM_100 = 397.0                 # 100 % rotor speed (v22data.OMEGA_100)
NR_TOP = 110.0                  # the highest rotor speed the states take, %: past the NR caution's 3 % either side of 104 %
fails = []


def check(name, ok, detail=''):
    print('%-4s %s%s' % ('ok' if ok else 'FAIL', name, ('  ' + detail) if detail else ''))
    if not ok:
        fails.append(name)


def key(e):
    return e.text.strip().lstrip('/')


def val(e, P):
    """an <expression>'s value, as SimGear's SGExpression has it"""
    t = e.tag
    if t == 'value':
        return float(e.text)
    if t == 'property':
        return P[key(e)]
    k = [c for c in e if c.tag not in ('clipMin', 'clipMax')]
    if t == 'sum':
        return sum(val(c, P) for c in k)
    if t == 'difference':
        r = val(k[0], P)
        for c in k[1:]:
            r = r - val(c, P)
        return r
    if t == 'product':
        r = 1.0
        for c in k:
            r = r * val(c, P)
        return r
    if t == 'div':
        return val(k[0], P) / val(k[1], P)
    if t == 'sqrt':
        return np.sqrt(np.maximum(val(k[0], P), 0.0))
    if t == 'pow':
        return np.power(np.maximum(val(k[0], P), 0.0), val(k[1], P))
    if t == 'max':
        return np.maximum(val(k[0], P), val(k[1], P))
    if t == 'clip':
        return np.clip(val(k[0], P), float(e.findtext('clipMin')), float(e.findtext('clipMax')))
    raise ValueError('expression: ' + t)


def test(e, P):
    """a <condition>'s truth"""
    t = e.tag
    if t == 'and':
        r = True
        for c in e:
            r = r & test(c, P)
        return r
    if t == 'or':
        r = False
        for c in e:
            r = r | test(c, P)
        return r
    if t == 'not':
        return ~np.asarray(test(list(e)[0], P), bool)
    if t == 'property':
        return P[key(e)] > 0.5
    a, b = list(e)
    if t == 'greater-than':
        return val(a, P) > val(b, P)
    if t == 'less-than':
        return val(a, P) < val(b, P)
    raise ValueError('condition: ' + t)


def sg_pitch(x, P, n):
    """the pitch SimGear plays a loop at, and its own. SimGear multiplies its <pitch> blocks' factor x property, each within
    its <min> and <max> (a max of 0 being none), and adds their offsets, 1.0 where none is given; an <expression> block
    adds no offset. Then it holds the sum within 0.01 and 2.0"""
    sg, own, offset = np.ones(n), np.ones(n), 0.0
    for pt in x.findall('pitch'):
        ex = pt.find('expression')
        if ex is not None:
            v = np.broadcast_to(val(list(ex)[0], P), (n,))
            sg, own = sg * v, own * v
            continue
        q = np.broadcast_to(P[key(pt.find('property'))] * float(pt.findtext('factor') or 1.0), (n,))
        lo, hi = float(pt.findtext('min') or 0.0), float(pt.findtext('max') or 0.0)
        c = np.maximum(q, lo)
        sg, own = sg * (np.minimum(c, hi) if hi else c), own * q
        offset += float(pt.findtext('offset') or 1.0)
    return np.clip(offset + sg, 0.01, 2.0), own


def states(E, n=40000, seed=3):
    """random states of what the loops' conditions, volumes and pitches read, half of each at the banks' grid points"""
    r = np.random.default_rng(seed)
    pick = lambda grid, lo, hi: np.where(r.random(n) < 0.5, r.choice(grid, n), r.uniform(lo, hi, n))
    P = {'sim/current-view/internal': r.integers(0, 2, n).astype(float),
         'sim/model/v22/nacelle-deg': pick([0, 30, 45, 60, 75, 90, 97.5], 0, 97.5)}
    nr = pick([0, 5, 20, 50, 84, 104], 0, NR_TOP)
    P['fdm/jsbsim/drive/nr-pct'] = nr
    P['rotors/main/rpm'] = nr * RPM_100 / 100.0
    for e in (0, 1):
        q = 'fdm/jsbsim/eng[%d]/' % e
        P[q + 'n1-pct'] = pick([0, 3, 5, 8, 12, 17, 20, 24, 30, 45, 65, 80, 92, 100, 104], 0, 106)
        P[q + 'running'] = r.integers(0, 2, n).astype(float)
        # (the starter turns while a running engine comes up to 62 %: drive.py)
        P[q + 'cranking'] = ((r.random(n) < 0.5) & (P[q + 'running'] > 0.5) & (P[q + 'n1-pct'] < 62)).astype(float)
        P[q + 'fuel-flow-pph'] = r.uniform(0, 3000, n)
        P[q + 'torque-pct'] = r.uniform(-10, 110, n)
    P['velocities/airspeed-kt'] = pick([0, 40, 80, 120, 140, 180, 220, 260], 0, 300)
    P['velocities/equivalent-kt'] = P['velocities/airspeed-kt'] * r.uniform(0.6, 1.0, n)
    P['velocities/vertical-speed-fps'] = pick([0, -5, 5, -15, -25, -40], -70, 40)
    P['position/altitude-agl-ft'] = pick([0, 5, 9.5, 20, 28.6], 0, 3000)
    P['rotors/main/thrust-lbf'] = pick([-472.1, 899.2, 2472.9, 4945.8], -1500, 7000)
    P['rotors/main/thrust-over-weight'] = pick([0.66, 1.1, 1.43], 0.3, 1.8)
    peak = json.load(open(os.path.join(HERE, 'data', 'slap_v22.json')))['peak']
    P['rotors/main/slap-ext'] = r.uniform(0, peak['ext'], n)
    P['rotors/main/slap-int'] = r.uniform(0, peak['int'], n)
    P['environment/density-slugft3'] = r.uniform(0.0008, 0.00255, n)
    P['environment/temperature-degc'] = r.uniform(-40, 45, n)
    for x in E:
        for el in x.iter('property'):
            P.setdefault(key(el), np.zeros(n))
    return P


def main(path=XML):
    E = list(ET.parse(path).getroot().find('fx'))
    bank = [x for x in E if (x.findtext('name') or '').startswith('pww-')]
    nooff = [x.findtext('name') for x in bank
             if any(pt.find('expression') is None and pt.find('offset') is None for pt in x.findall('pitch'))]
    blocks = sum(len(x.findall('pitch')) for x in bank)
    check('every bank loop states its pitch offset (SimGear adds 1.0 otherwise)', not nooff,
          '%d of %d loops with %d pitch blocks lack it, e.g. %s' % (len(nooff), len(bank), blocks, nooff[:2]) if nooff
          else '%d pitch blocks in %d loops' % (blocks, len(bank)))
    other = [x.findtext('name') for x in E if x not in bank and any(pt.find('expression') is None and pt.find('offset') is None
                                                                       for pt in x.findall('pitch'))]
    print('     (hand-written sounds with no offset, SimGear adding 1.0 as their authors may have meant: %s)' % ', '.join(other))

    pitched = [x for x in bank if x.findall('pitch')]
    n = 40000
    P = states(E, n)
    bad, over, unheard = {}, {}, []
    for x in pitched:
        c = x.find('condition')
        C = np.broadcast_to(test(list(c)[0], P), (n,)) if c is not None else np.ones(n, bool)
        V = np.broadcast_to(val(list(x.find('volume').find('expression'))[0], P), (n,))
        top = V[C].max() if C.any() else 0.0
        heard = C & (V >= HEARD * top) & (top > 0)
        if not heard.any():
            unheard.append(x.findtext('name'))
            continue
        sg, own = sg_pitch(x, P, n)
        h = heard & (own >= 0.01)                   # (under SimGear's floor nothing can play it, the loop nearly at rest)
        err = np.abs(sg / np.maximum(own, 1e-9) - 1.0)
        if (err[h] > 1e-3).any():
            k = np.where(h)[0][np.argmax(err[h])]
            prop = key(x.find('pitch').find('property')) if x.find('pitch').find('property') is not None else ''
            bad[x.findtext('name')] = (sg[k], own[k], prop, P[prop][k] if prop else 0.0)
        if own[heard].max() > 2.0:
            over[x.findtext('name')] = float(own[heard].max())
    worst = sorted(bad.items(), key=lambda kv: -abs(kv[1][0] / kv[1][1] - 1))[:3]
    check('over %d random states every loop heard plays at its own pitch' % n, not bad,
          '%d of %d loops off it, e.g. %s' % (len(bad), len(pitched), '; '.join(
              '%s at %.3g (its own %.3g, %s %.4g)' % (k, *v) for k, v in worst)) if bad else '%d loops' % len(pitched))
    check('no loop heard where its own pitch needs more than SimGear\'s 2.0', not over,
          '%d: %s' % (len(over), sorted(over.items())[:3]) if over else '')
    check('the states reach every pitched loop', not unheard, '%d never heard: %s' % (len(unheard), unheard[:4]) if unheard else '')
    if fails:
        sys.exit('%d failed' % len(fails))


if __name__ == '__main__':
    main(*sys.argv[1:2])
