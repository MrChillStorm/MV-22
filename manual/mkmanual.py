#!/usr/bin/env python3
"""The flight manual's numbers (tab/numbers.tex, as \\M... macros) and tables (tab/tab_*.tex), from the aircraft's own
data, so that the manual follows the aircraft: the flight model's (fdm/v22data.py, fdm/data/*.json and the report's
results, fdm/report/results/*.json), the key bindings (MV-22B/Systems/v22-keyboard.xml) and the flight control panel
(fdm/cockpit/panel.py's AFCS_KEYS)."""
import os, sys, re, ast, json
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FDM = os.path.join(ROOT, 'fdm')
sys.path.insert(0, FDM)
import v22data as V

TAB = os.path.join(HERE, 'tab')
NUM = {}


def num(name, value, fmt='%.0f'):
    assert re.fullmatch('M[A-Za-z]+', name), name
    s = value if isinstance(value, str) else (fmt % value)
    NUM[name] = s


def grouped(x):
    return '{:,}'.format(int(round(x))).replace(',', '\\,')


def write(name, body):
    open(os.path.join(TAB, 'tab_%s.tex' % name), 'w').write(body)


def tabular(cols, head, rows):
    return ('\\begin{tabular}{@{}%s@{}}\n\\toprule\n%s \\\\\n\\midrule\n' % (cols, head) + ''.join(r + ' \\\\\n' for r in rows) +
            '\\bottomrule\n\\end{tabular}\n')


def tex(s):
    return s.replace('{', '\\{').replace('}', '\\}').replace('&', '\\&').replace('%', '\\%').replace('#', '\\#').replace('_', '\\_').replace('~', '\\textasciitilde{}')


# ---------------------------------------------------------------- the aircraft: weights, engines, drive, conversion
def aircraft():
    fuel_gal = sum(t[1] for t in V.TANKS)
    fuel_lb = fuel_gal * V.FUEL_LB_GAL
    num('MWEmpty', grouped(V.W_EMPTY)); num('MWRef', grouped(V.GW_DESIGN))
    num('MWStart', grouped(V.W_EMPTY + 3 * V.W_CREW + V.FUEL_START * fuel_lb))
    num('MFuelGal', grouped(fuel_gal)); num('MFuelLb', grouped(round(fuel_lb, -1)))
    num('MCargoMax', grouped(V.CARGO_MAX)); num('MTroops', V.TROOPS)
    num('MRotorDia', 2 * V.R); num('MSpan', V.B_W, '%.1f')
    num('MPmrp', grouped(V.P_MRP)); num('MPirp', grouped(V.P_IRP)); num('MPmcp', grouped(V.P_MCP))
    num('MPxmsn', grouped(V.P_XMSN_100)); num('MPxmsnApln', grouped(V.P_XMSN_100 * V.NR_APLN / 100))
    num('MNrHelo', V.NR_HELO); num('MNrApln', V.NR_APLN)
    num('MNacMin', V.NACELLE_MIN); num('MNacMax', V.NACELLE_MAX, '%.1f'); num('MNacRate', V.NACELLE_RATE)
    num('MVmo', V.VMO_APLN); num('MMmo', V.MMO_APLN, '%.2f'); num('MVmoHundred', V.VMO_APLN_100)
    (kv, sk) = V.VRS_SINK
    num('MVrsLoKt', kv[0]); num('MVrsLoSink', grouped(sk[0])); num('MVrsHiKt', kv[1]); num('MVrsHiSink', grouped(sk[1]))
    rows = ['%s & %d & %.0f' % (n, g, g * V.FUEL_LB_GAL) for n, g, *_ in V.TANKS]
    rows.append('\\midrule total & %d & %.0f' % (fuel_gal, fuel_lb))
    write('tanks', tabular('lrr', 'Tank & US gal & lb', rows))
    # the conversion protection: the least and the most nacelle angle against equivalent airspeed
    (lv, ln), (uv, un) = V.CPS
    speeds = sorted(set(lv) | set(uv))
    import numpy as np
    rows = ['%d & %.0f & %.1f' % (k, np.interp(k, lv, ln), np.interp(k, uv, un)) for k in speeds]
    write('cps', tabular('rrr', 'KEAS & least nacelle, deg & most nacelle, deg', rows))


# ---------------------------------------------------------------- the keys and the flight control panel
FG_KEYS = [('Page Up / Page Down', 'Thrust control lever up / down (or the throttle axis)'),
           ('arrow keys, keypad', 'Stick (elevator and aileron axes)'),
           ('keypad 0 / Enter', 'Pedals (rudder axis)'),
           ('g / G', 'Landing gear up / down'),
           ('B', 'Parking brake')]


def keys():
    t = ET.parse(os.path.join(ROOT, 'MV-22B', 'Systems', 'v22-keyboard.xml')).getroot()
    rows = []
    for k in t.iter('key'):
        name, desc = k.findtext('name'), k.findtext('desc')
        if name and desc:
            rows.append('\\texttt{%s} & %s' % (tex(name), tex(desc)))
    rows += ['\\texttt{%s} & %s' % (tex(a), tex(b)) for a, b in FG_KEYS]
    write('keys', tabular('lp{0.68\\textwidth}', 'Key & Function', rows))


PANEL = {'CPLD-L': 'Couples the flight director to the augmentation (either key)', 'CPLD-R': None,
         'ENAV': 'Not installed', 'TACAN': 'Not installed', 'SRCHPAT': 'Not installed',
         'VORILS': 'VOR/ILS: NAV 1\'s localizer and glide slope, or its VOR', 'APPR': 'Approach to a hover; again: land; again: off',
         'WYPT': 'The route manager\'s route', 'HDGHOLD': 'Heading hold (HDG knob)', 'SPDHOLD': 'Speed hold (SPEED knob)',
         'ALTHOLD': 'Altitude hold (ALT knob)', 'HVRHOLD': 'Hover height hold, HVR ALT HOLD (HVR ALT knob)',
         'AUTONAC': 'Automatic nacelles', 'TF': 'Terrain following', 'HVRCPLD': 'Hover hold', 'REMHVR': 'Remote hover: the stick\'s '
         'translation slowed to 2 kt'}
LEGEND = {'CPLD-L': 'CPLD', 'VORILS': 'VOR/ILS', 'SRCHPAT': 'SRCH PAT', 'HDGHOLD': 'HDG HOLD', 'SPDHOLD': 'SPD HOLD', 'ALTHOLD': 'ALT HOLD',
          'HVRHOLD': 'HVR HOLD', 'AUTONAC': 'AUTO NAC', 'HVRCPLD': 'HVR CPLD', 'REMHVR': 'REM HVR'}


def panel():
    src = open(os.path.join(FDM, 'cockpit', 'panel.py')).read()
    def literal(name):
        m = re.search(r'^%s = (\[.*?\])\n(?=\S)' % name, src, re.S | re.M)
        return ast.literal_eval(m.group(1))
    rows = []
    for kid, *_ in literal('AFCS_KEYS'):
        if PANEL.get(kid):
            rows.append('%s & %s' % (LEGEND.get(kid, kid), tex(PANEL[kid])))
    knobs = {'CRS': 'NAV 1\'s course', 'HDG': 'Heading bug', 'SPD': 'Speed, KIAS', 'ALT': 'Altitude, ft', 'HVRALT': 'Hover and TF height '
             'above the ground, ft', 'BRT': 'Panel brightness'}
    for kid, *_ in literal('AFCS_KNOBS'):
        rows.append('%s knob & %s' % ({'SPD': 'SPEED', 'HVRALT': 'HVR ALT'}.get(kid, kid), tex(knobs[kid])))
    write('panel', tabular('lp{0.62\\textwidth}', 'Key or knob & Function', rows))


# ---------------------------------------------------------------- performance, from the report's results
def performance():
    res = os.path.join(FDM, 'report', 'results')
    sp = json.load(open(os.path.join(res, 'speed.json')))
    rows = ['%s & %s & %.0f' % (grouped(r['gw']), grouped(r['alt']), r['vmax']) for r in sp]
    write('vmax', tabular('rrr', 'Gross weight, lb & Altitude, ft & KTAS', rows))
    rows = []
    for a in (0, 4000, 8400, 12000, 16000):
        c = json.load(open(os.path.join(res, 'ceiling%d.json' % a)))
        rows.append('%s & %s' % (grouped(a), grouped(round(c['gw'], -1))))
    write('ceiling', tabular('rr', 'Pressure altitude, ft & Gross weight, lb', rows))
    cp = json.load(open(os.path.join(FDM, 'data', 'climb_power_v22.json')))
    num('MClimbWeight', grouped(cp['weight_lb']))
    i = min(range(len(cp['kcas'])), key=lambda j: cp['climb_full_fps'][j])
    num('MClimbLeast', grouped(round(60 * cp['climb_full_fps'][i], -1))); num('MClimbLeastKt', cp['kcas'][i])
    # TF's plan: its horizon (MV-22B/Nasal/autopilot.nas)
    nas = open(os.path.join(ROOT, 'MV-22B', 'Nasal', 'autopilot.nas')).read()
    T = float(re.search(r'var TF_T = ([0-9.]+);', nas).group(1))
    num('MTfHorizon', T)
    # the rise it clears in that time at the least climb, c, its pull-up at half the medium ride's 0.2 g: c T - c^2 / (2 a2)
    c, a2 = cp['climb_full_fps'][i], 0.5 * 0.2 * 32.174
    num('MTfRise', grouped(round(c * T - c * c / (2 * a2), -1)))


# ---------------------------------------------------------------- level flight along AUTO NAC's schedule (cached)
TRIMS = os.path.join(HERE, 'data', 'level_trims.json')
TRIM_KCAS = (40, 60, 80, 100, 120, 135, 150, 160, 170, 180, 200, 220, 250, 270)


def trims():
    """level-flight trims at the reference loading, 1000 ft, along AUTO NAC's schedule: the power required and the
    pitch attitude (mkmanual.py --trims writes data/level_trims.json; the build reads it)"""
    import numpy as np
    import harness as H
    from genv22 import autonac_schedule
    sk, sn = autonac_schedule()
    f = H.load()
    rows = []
    for V in TRIM_KCAS:
        nac = float(np.interp(V, sk, sn))
        x, r, e = H.trim(f, float(V), alt_ft=1000, nacelle=nac, cas=True, extra=('drive/power-required-hp',))
        rows.append(dict(kcas=V, nacelle=nac, power_hp=e[0], pitch_deg=f['attitude/theta-deg'], res=float(np.abs(r).max()),
                         weight=f['inertia/weight-lbs']))
        print('%3d kt: nacelle %4.1f, power %5.0f hp, pitch %4.1f deg (residual %.3f)' % (V, nac, e[0], rows[-1]['pitch_deg'], rows[-1]['res']), flush=True)
    os.makedirs(os.path.dirname(TRIMS), exist_ok=True)
    json.dump(rows, open(TRIMS, 'w'), indent=1)


def level():
    rows = json.load(open(TRIMS))
    th = 1 - 6.8756e-6 * 1000                            # one engine's rating at the trims' 1000 ft: delta / sqrt(theta)
    one = V.P_MRP * th ** 5.2559 / th ** 0.5
    num('MPoneThousand', grouped(round(one, -1)))
    write('level', tabular('rrrrl', 'KCAS & nacelle, deg & power, hp & pitch, deg & one engine',
                           ['%d & %.0f & %s & %.1f & %s' % (r['kcas'], r['nacelle'], grouped(r['power_hp']), r['pitch_deg'],
                                                            'enough' if r['power_hp'] < one else 'not enough') for r in rows]))
    p = {r['kcas']: r for r in rows}
    num('MPitchTwoHundred', p[200]['pitch_deg'], '%.0f'); num('MPitchTwoFifty', p[250]['pitch_deg'], '%.0f')
    num('MPitchTwoSeventy', p[270]['pitch_deg'], '%.0f')
    ok = [r['kcas'] for r in rows if r['power_hp'] < one]
    num('MOeiLo', min(ok)); num('MOeiHoverPower', grouped(rows[0]['power_hp']))


if __name__ == '__main__':
    if sys.argv[1:] == ['--trims']:
        trims()
        sys.exit()
    os.makedirs(TAB, exist_ok=True)
    for fn in (aircraft, keys, panel, performance, level):
        fn()
    open(os.path.join(TAB, 'numbers.tex'), 'w').write(''.join('\\newcommand{\\%s}{%s}\n' % kv for kv in sorted(NUM.items())))
    print('%d numbers' % len(NUM))
