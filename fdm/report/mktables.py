#!/usr/bin/env python3
"""The report's tables (tab/tab_*.tex) and the numbers quoted in its text (tab/numbers.tex, as \\N... macros), from
report/results/*.json (mkresults.py) and the data they are compared with."""
import os, sys, json, math, re
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FDM = os.path.dirname(HERE)
sys.path.insert(0, FDM)
import v22data as V
import xv15data as X
from data import xv15_hover as XH, xv15_flight as XF, v22_flight as VF, bell301 as B
TAB = os.path.join(HERE, 'tab')
NUM = {}
NACELLES = [90, 75, 60, 30, 0]
CEILING_ALTS = [0, 4000, 8400, 12000, 16000]


def R(name):
    return json.load(open(os.path.join(HERE, 'results', name + '.json')))


def num(name, value, fmt='%.1f'):
    assert re.fullmatch('[A-Za-z]+', name), name
    s = (fmt % value) if not isinstance(value, str) else value
    NUM[name] = '$-$' + s[1:] if s.startswith('-') else s


def write(name, body):
    open(os.path.join(TAB, 'tab_%s.tex' % name), 'w').write(body)


def f(x, fmt='%+.1f'):
    return '--' if x is None or (isinstance(x, float) and math.isnan(x)) else (fmt % x).replace('-', '$-$')


def tabular(cols, head, rows, foot=''):
    return ('\\begin{tabular}{@{}%s@{}}\n\\toprule\n%s \\\\\n\\midrule\n' % (cols, head) +
            ''.join(r + ' \\\\\n' for r in rows) + foot + '\\bottomrule\n\\end{tabular}\n')


# ---------------------------------------------------------------- the generated model
def model_table():
    root = os.path.join(FDM, 'root', 'aircraft', 'mv22b', 'Systems')
    files = [('mv22b-aero.xml', 'the aerodynamic functions: blade segments of both rotors, gimbal moments, wing strips in the wakes, fuselage, empennage; the six axes', 'function'),
             ('mv22b-rotors.xml', 'gimbals, rotor azimuth, Peters--He wake states of each rotor with their gains and forcing', 'fcs'),
             ('mv22b-airframe.xml', 'ring-vortex wake at the wing, stabiliser and fins; ground effect', 'fcs'),
             ('mv22b-drive.xml', 'engines, torque limit, interconnected drive and rotor speed, rotor brake, fuel', 'fcs'),
             ('mv22b-controls.xml', 'pilot controls, mixing, nacelles and conversion protection, flaperons, governor, augmentation, hover hold, servos', 'fcs'),
             ('mv22b-outputs.xml', 'properties for the 3-D model and the instruments', 'fcs')]
    rows, nfun, nfcs = [], 0, 0
    for fn, what, kind in files:
        s = open(os.path.join(root, fn)).read()
        if kind == 'function':
            n = len(re.findall(r'<function name=', s)); nfun += n
        else:
            n = len(re.findall(r'<(?:fcs_function|pure_gain|summer|lag_filter|integrator|switch|kinematic|actuator|'
                               r'washout_filter|lead_lag_filter|second_order_filter|deadband|aerosurface_scale|distributor)\b', s)); nfcs += n
        rows.append('\\texttt{%s} & %s & %d' % (fn.replace('mv22b-', '-'), what, n))
    num('NnFunc', '%d' % nfun); num('NnFcs', '%d' % nfcs)
    write('model', tabular('>{\\raggedright\\arraybackslash}p{2.15cm}>{\\raggedright\\arraybackslash}X r', 'File & Content & $n$', rows)
          .replace('\\begin{tabular}', '\\begin{tabularx}{\\columnwidth}').replace('\\end{tabular}', '\\end{tabularx}'))


def params_table():
    sol = lambda U: U.NB * (U.CHORD_ROOT + U.CHORD_TIP) / 2 / (math.pi * U.R) if hasattr(U, 'CHORD_ROOT') else float('nan')
    rpm = lambda U: U.OMEGA_100 * 30 / math.pi
    rows = [('Rotor radius, ft', '%.2f' % V.R, '%.2f' % X.R),
            ('Blades per rotor', '%d' % V.NB, '%d' % X.NB),
            ('Precone, deg', '%.2f' % math.degrees(V.PRECONE), '%.2f' % math.degrees(X.PRECONE)),
            ('$\\delta_3$, deg', '%.0f' % math.degrees(V.DELTA3), '%.0f' % math.degrees(X.DELTA3)),
            ('Chord root (extrapolated) / tip, in', '%.1f / %.1f' % (12 * V.CHORD_ROOT, 12 * V.CHORD_TIP), '%.1f / %.1f' % (12 * X.CHORD_ROOT, 12 * X.CHORD_TIP)),
            ('Rotor speed 100\\,\\%, rpm', '%.0f' % rpm(V), '%.0f' % rpm(X)),
            ('Rotor speed, helicopter/airplane, \\%', '%.0f / %.0f' % (V.NR_HELO, V.NR_APLN), '%.0f / %.0f' % (X.NR_HELO, X.NR_APLN)),
            ('Wing area, ft$^2$; span, ft', '%.0f; %.2f' % (V.S_W, V.B_W), '%.0f; %.2f' % (X.S_W, X.B_W)),
            ('Engines, shp each', '%.0f' % V.P_MRP, '%.0f' % X.P_MRP),
            ('Drive torque limit at 100\\,\\%, hp', '%.0f (fitted)' % V.P_XMSN_100, '%.0f' % X.P_XMSN_100),
            ('Nacelle angle range, deg; rate, deg/s', '%.0f--%.1f; %.0f' % (V.NACELLE_MIN, V.NACELLE_MAX, V.NACELLE_RATE),
             '%.0f--%.1f; %.0f' % (X.NACELLE_MIN, X.NACELLE_MAX, X.NACELLE_RATE)),
            ('Reference gross weight, lb', '%.0f' % V.GW_DESIGN, '%.0f' % X.GW_DESIGN),
            ('Empty weight, lb', '%.0f' % V.W_EMPTY, '--'),
            ('Fuel, US gal (tanks)', '%.0f (%d)' % (sum(t[1] for t in V.TANKS), len(V.TANKS)), '--')]
    write('params', tabular('@{}l r r', 'Item & V-22 & XV-15', [' & '.join(r) for r in rows]))


# ---------------------------------------------------------------- the XV-15 rotor in hover
def hover_table():
    rows = R('hover')
    imax = max(range(len(rows)), key=lambda i: rows[i]['ct_s'])
    num('NHthStall', rows[imax]['th75'], '%.0f')
    rows = rows[:imax + 1]                     # up to the rotor's maximum thrust (past it the blades stall: Section 5)
    out, dcp, dfm = [], [], []
    for r in rows:
        cts = r['ct_s']
        if not (0.075 <= cts <= XH.CT_S[-1]):
            continue
        cpt = float(np.interp(cts, XH.CT_S, XH.CP_S))
        ct = cts * XH.SIGMA; cp = cpt * XH.SIGMA
        fmt = ct ** 1.5 / math.sqrt(2) / cp
        d = 100 * (r['cp_s'] / cpt - 1); dcp.append(d); dfm.append(r['fm'] - fmt)
        out.append('%.1f & %.3f & %.5f & %.5f & %s & %.3f & %.3f' % (r['th75'], cts, r['cp_s'], cpt, f(d), r['fm'], fmt))
    write('hover', tabular('rrrrrrr', '$\\theta_{75}$ & $C_T/\\sigma$ & \\multicolumn{2}{c}{$C_P/\\sigma$, model / test} & $\\Delta$\\,\\% & '
                           '\\multicolumn{2}{c}{FM, model / test}', out))
    num('NHcpMin', min(dcp)); num('NHcpMax', max(dcp))
    num('NHctMax', max(r['ct_s'] for r in rows), '%.3f'); num('NHctMaxTest', XH.CTS_MAX_TESTED, '%.3f')
    # low thrust: below the table's range
    low = [100 * (r['cp_s'] / float(np.interp(r['ct_s'], XH.CT_S, XH.CP_S)) - 1) for r in rows if 0.02 <= r['ct_s'] < 0.075]
    num('NHcpLowMin', min(low), '%.0f'); num('NHcpLowMax', max(low), '%.0f')
    # the collective gradient, model against the test's
    th = [r['th75'] for r in rows if 0.02 <= r['ct_s'] <= 0.16]; cs = [r['ct_s'] for r in rows if 0.02 <= r['ct_s'] <= 0.16]
    g = np.polyfit(th, cs, 1)
    num('NHgrad', g[0], '%.4f'); num('NHgradTest', XH.DCTS_DCOLL, '%.4f'); num('NHzero', -g[1] / g[0])
    num('NHgradPct', 100 * (g[0] / XH.DCTS_DCOLL - 1), '%.0f')
    fm = max(rows, key=lambda r: r['fm'])
    num('NHfmMod', fm['fm'], '%.2f'); num('NHfmCt', fm['ct_s'], '%.2f')


# ---------------------------------------------------------------- the XV-15 in level flight
SETNAMES = [('nac90', '90$^\\circ$, flaps 20$^\\circ$'), ('nac60', '60$^\\circ$, flaps 20$^\\circ$'), ('nac30', '30$^\\circ$, flaps 20$^\\circ$'),
            ('nac0f20', '0$^\\circ$, flaps 20$^\\circ$'), ('nac0', '0$^\\circ$, flaps up')]


def xv15_power(name, data=None):
    """(V, test hp, model hp at sea level, at 10 000 ft) at each test point (data: the test's set, else name's)"""
    res = R('xv15_' + name)
    out = []
    for v, p in XF.SETS[data or name][2]:
        m = {}
        for alt in ('0', '10000'):
            rows = [r for r in res[alt] if r['res'] < 0.05]
            Vs = [r['V'] for r in rows]; hp = [r['hp'] for r in rows]
            m[alt] = float(np.interp(v, Vs, hp)) if rows and Vs[0] - 1 <= v <= Vs[-1] + 1 else float('nan')
        out.append((v, p, m['0'], m['10000']))
    return out


def xv15_table():
    out = []
    for name, lab in SETNAMES:
        pts = xv15_power(name)
        d0 = [100 * (m0 / p - 1) for v, p, m0, m1 in pts if m0 == m0]
        d1 = [100 * (m1 / p - 1) for v, p, m0, m1 in pts if m1 == m1]
        out.append('%s & %.0f--%.0f & %s to %s & %s to %s' % (lab, pts[0][0], pts[-1][0], f(min(d0), '%+.0f'), f(max(d0), '%+.0f'),
                                                          f(min(d1), '%+.0f'), f(max(d1), '%+.0f')))
        if name == 'nac90':
            # without the hover point: in forward flight, at 45 and 94 kt
            fwd = [(v, p, m0) for v, p, m0, m1 in pts if v > 1]
            num('NXhelLo', 100 * (fwd[0][2] / fwd[0][1] - 1), '%+.0f'); num('NXhelHi', 100 * (fwd[-1][2] / fwd[-1][1] - 1), '%+.0f')
            num('NXhover', 100 * (pts[0][2] / pts[0][1] - 1), '%+.0f')
        if name in ('nac30', 'nac0f20', 'nac60'):
            k = {'nac30': 'c', 'nac0f20': 'f', 'nac60': 's'}[name]
            num('NX%sMin' % k, min(d0), '%+.0f'); num('NX%sMax' % k, max(d0), '%+.0f')
            num('NX%sHiMin' % k, min(d1), '%+.0f'); num('NX%sHiMax' % k, max(d1), '%+.0f')
        if name == 'nac0':
            num('NXaMin', min(d1), '%+.0f'); num('NXaMax', max(d1), '%+.0f')
            num('NXaSlMin', min(d0), '%+.0f'); num('NXaSlMax', max(d0), '%+.0f')
    pts = xv15_power('nac0_gear', 'nac0')
    d0 = [100 * (m0 / p - 1) for v, p, m0, m1 in pts if m0 == m0]; d1 = [100 * (m1 / p - 1) for v, p, m0, m1 in pts if m1 == m1]
    out.append('0$^\\circ$, flaps up, gear down & %.0f--%.0f & %s to %s & %s to %s' % (pts[0][0], pts[-1][0], f(min(d0), '%+.0f'), f(max(d0), '%+.0f'),
                                                                                f(min(d1), '%+.0f'), f(max(d1), '%+.0f')))
    num('NXgMin', min(d1), '%+.0f'); num('NXgMax', max(d1), '%+.0f')
    write('xv15', tabular('lrrr', 'Nacelle, flaps & KCAS & sea level, \\% & 10\\,000\\,ft, \\%', out))


def download_table():
    res = R('download')
    out = []
    for name, rows in (('XV-15', res['xv15']), ('V-22', res['v22'])):
        for r in rows:
            out.append('%s & %.0f & %.1f & %.1f & %s' % (name, r['flap'], 100 * r['dl_t'], 100 * r['test'], f(100 * (r['dl_t'] - r['test']))))
    write('download', tabular('lrrrr', 'Aircraft & flap, deg & model, \\% & test, \\% & $\\Delta$', out))
    v = res['v22'][0]
    num('NDlV', 100 * v['dl_t']); num('NDlVtest', 100 * v['test'])
    x = res['xv15']
    num('NDlXmax', max(abs(100 * (r['dl_t'] - r['test'])) for r in x))


def polar_table():
    res = R('polar')
    rows = res['rows']
    CL = [r['CL'] for r in rows]; CD = [r['CD'] for r in rows]
    out = []
    for cl_t, cd_t in res['test']:
        cd_m = float(np.interp(cl_t, CL, CD)) if CL[0] <= cl_t <= CL[-1] else float('nan')
        out.append('%.2f & %s & %.4f & %s' % (cl_t, f(cd_m, '%.4f'), cd_t, f(1e4 * (cd_m - cd_t), '%+.0f')))
    write('polar', tabular('rrrr', '$C_L$ & $C_D$ model & $C_D$ test & $\\Delta$, counts', out))
    slope = np.polyfit([r['alpha'] for r in rows], CL, 1)
    num('NPslope', slope[0], '%.3f'); num('NPclzero', slope[1], '%.2f')
    num('NPcdTwo', float(np.interp(0.2, CL, CD)), '%.4f')


def perf_table():
    sp = R('speed')
    out = []
    for r in sp:
        test = dict(VF.ENVELOPE_42K).get if r['gw'] == 42000 else None
        t = {0: 274, 17000: 321}.get(r['alt']) if r['gw'] == 42000 else None
        out.append('maximum level speed, %.0f\\,lb, %s\\,ft & %.0f\\,KTAS & %s' % (r['gw'], '{:,}'.format(r['alt']).replace(',', '\\,'), r['vmax'],
                                                                              '%d\\,KTAS' % t if t else '--'))
    ce = [R('ceiling%d' % a) for a in CEILING_ALTS]
    tests = dict((alt, gw) for gw, alt in VF.HOVER_CEILING)
    for c in ce:
        t = tests.get(c['alt'])
        out.append('hover ceiling OGE at %s\\,ft & %.0f\\,lb & %s' % ('{:,}'.format(c['alt']).replace(',', '\\,'), c['gw'],
                                                                     ('%s\\,lb' % '{:,}'.format(t).replace(',', '\\,')) if t else '--'))
    write('perf', tabular('lrr', 'Quantity & model & test~\\cite{rc}', out))
    v = {(r['gw'], r['alt']): r['vmax'] for r in sp}
    num('NVsl', v[(42000, 0)], '%.0f'); num('NVhi', v[(42000, 17000)], '%.0f'); num('NVmax', v[(47500, 0)], '%.0f')
    g = {c['alt']: c['gw'] for c in ce}
    num('NCsl', '{:,}'.format(round(g[0], -1)).replace(',', '\\,')); num('NCmid', '{:,}'.format(round(g[8400], -1)).replace(',', '\\,'))
    num('NChi', '{:,}'.format(round(g[16000], -1)).replace(',', '\\,'))
    num('NCmidPct', 100 * (g[8400] / 43300 - 1), '%+.0f'); num('NChiPct', 100 * (g[16000] / 31000 - 1), '%+.0f')


def conversion_table():
    out = []
    allp, alll = [], []
    for n in NACELLES:
        rows = [r for r in R('conversion%d' % n)['rows'] if r['res'] < 0.05]
        dl = [r['wing_pct'] - r['test_lift'] for r in rows]
        dp = [r['pitch'] - r['test_pitch'] for r in rows if r['test_pitch'] == r['test_pitch']]
        alll += dl; allp += dp
        if n == 30:
            num('NCvWingLo', min(r['wing_pct'] for r in rows), '%.0f'); num('NCvWingHi', max(r['wing_pct'] for r in rows), '%.0f')
            num('NCvTestLo', min(r['test_lift'] for r in rows), '%.0f'); num('NCvTestHi', max(r['test_lift'] for r in rows), '%.0f')
        out.append('%d & %.0f--%.0f & %s to %s & %s to %s' % (n, rows[0]['V'], rows[-1]['V'], f(min(dl), '%+.0f'), f(max(dl), '%+.0f'),
                                                              f(min(dp)), f(max(dp))))
    write('conversion', tabular('rrrr', 'Nacelle, deg & KCAS & wing lift, \\% of $W$ & pitch, deg', out))
    num('NCvLiftRms', math.sqrt(np.mean(np.square(alll))), '%.0f'); num('NCvPitchRms', math.sqrt(np.mean(np.square(allp))))


def handling_numbers():
    h = R('handling_hover')
    t = np.array(h['t']); th = np.array(h['theta']); ph = np.array(h['phi']); ps = np.array(h['psi'])
    th0 = th[0]
    i = (t > 10) & (t < 16)
    num('NHdTheta', th[i].mean() - th0)
    i = (t > 22) & (t < 26)
    num('NHdPhi', ph[i].mean())
    i = (t > 32) & (t < 36)
    num('NHdYaw', abs(np.polyfit(t[i], ps[i], 1)[0]))
    c = R('handling_conversion')
    tc = np.array(c['t']); thc = np.array(c['theta'])
    num('NCthMin', thc.min()); num('NCthMax', thc.max()); num('NCvEnd', c['kcas'][-1], '%.0f')
    tm = json.load(open(os.path.join(FDM, 'data', 'trim_ff_v22.json')))
    num('NTmAft', -100 * min(min(r) for r in tm['lon']), '%.0f'); num('NTmFwd', 100 * max(max(r) for r in tm['lon']), '%.0f')
    s = R('start')
    ts = np.array(s['t']); nr = np.array(s['nr'])
    num('NStNr', ts[np.argmax(nr > 103.5)], '%.0f')


def hover_hold_numbers():
    st = R('hover_hold_stop')
    num('NHhStopErr', max(abs(r['stop_n_ft'] - r['shown_ft']) for r in st))
    num('NHhStopPitch', max(max(abs(a) for a in r['theta']) for r in st))
    r = st[-1]
    t, sta = np.array(r['t']), np.array(r['state'])
    num('NHhStopV', r['V'], '%.0f'); num('NHhStopD', r['shown_ft'], '%.0f'); num('NHhStopT', t[np.argmax(sta == 2)] - 1.0, '%.0f')
    for r in R('hover_hold_turn'):
        t, d, tu = np.array(r['t']), np.array(r['dist']), np.array(r['turning'])
        w = (t > r['t_on']) & (t < r['t_off'] + 10)
        key = {0: 'Calm', 20: 'Twenty', 35: 'Gale'}[int(r['wind_kt'])]
        num('NHhTurn' + key, d[w].max())
        num('NHhTurnAtt' + key, max(np.abs(r['theta'])[w].max(), np.abs(r['phi'])[w].max()))
        num('NHhTurnT' + key, r['t_off'] - r['t_on'], '%.0f')
    c = R('hover_hold_controls')
    tr = c['trc']
    t, gs = np.array(tr['t']), np.array(tr['gs'])
    i = (t >= 5) & (t < 25)
    vss = gs[(t > 20) & (t < 25)].mean()
    num('NHhTrcV', vss); num('NHhTrcT', t[i][np.argmax(gs[i] >= 0.632 * vss)] - 5.0); num('NHhTrcOver', max(0.0, gs[i].max() - vss))
    lv = c['lever']
    t, h = np.array(lv['t']), np.array(lv['h'])
    num('NHhLeverDh', abs(h[-1] - lv['h0']), '%.0f'); num('NHhLeverHold', max(0.1, np.ptp(h[t > t[-1] - 10])))


if __name__ == '__main__':
    os.makedirs(TAB, exist_ok=True)
    for fn in (model_table, params_table, hover_table, xv15_table, download_table, polar_table, perf_table, conversion_table, handling_numbers,
               hover_hold_numbers):
        fn()
    open(os.path.join(TAB, 'numbers.tex'), 'w').write(''.join('\\newcommand{\\%s}{%s}\n' % kv for kv in sorted(NUM.items())))
    print('%d numbers' % len(NUM))
