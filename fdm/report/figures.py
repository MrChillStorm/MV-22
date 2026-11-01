"""Figures for the report (vector PDF) from report/results/*.json.  python3 report/figures.py"""
import sys, os, json, math
HERE = os.path.dirname(os.path.abspath(__file__))
FDM = os.path.dirname(HERE)
sys.path.insert(0, FDM)
import numpy as np
import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
import matplotlib.ticker
import v22data as V
from data import xv15_hover as XH, xv15_flight as XF, v22_flight as VF

plt.rcParams.update({'font.family': 'serif', 'font.serif': ['STIX Two Text', 'STIXGeneral', 'DejaVu Serif'], 'mathtext.fontset': 'stix',
                     'font.size': 8.5, 'axes.labelsize': 8.5, 'axes.titlesize': 9, 'legend.fontsize': 7.5, 'xtick.labelsize': 7.5,
                     'ytick.labelsize': 7.5, 'axes.linewidth': 0.6, 'lines.linewidth': 1.1, 'grid.linewidth': 0.3, 'grid.alpha': 0.5,
                     'axes.grid': True, 'figure.dpi': 150, 'savefig.bbox': 'tight', 'savefig.pad_inches': 0.02,
                     'axes.spines.top': False, 'axes.spines.right': False})
C_MOD, C_TEST, C_REF = '#b2182b', '#222222', '#7a7a7a'
NAC_C = {90: '#b2182b', 75: '#ef8a62', 60: '#67a9cf', 30: '#2166ac', 0: '#1b7837'}
W1, W2 = 3.35, 6.9          # single / double column width (in)
OUT = os.path.join(HERE, 'fig')
RES = os.path.join(HERE, 'results')
NACELLES = [90, 75, 60, 30, 0]
CEILING_ALTS = [0, 4000, 8400, 12000, 16000]


def res(name):
    return json.load(open(os.path.join(RES, name + '.json')))


def save(fig, name):
    fig.savefig(os.path.join(OUT, name + '.pdf')); plt.close(fig)


def tag(ax, s):
    ax.set_title('(%s)' % s, loc='left', fontsize=8.5, pad=3)


def test_pts(ax, data, label='test', color=C_TEST, marker='o'):
    x, y = zip(*data)
    ax.plot(x, y, marker, mfc='none', mec=color, ms=3.6, mew=0.8, ls='none', label=label)


def hover():
    rows = res('hover')
    cts = [r['ct_s'] for r in rows]
    fig, axs = plt.subplots(1, 3, figsize=(W2, 2.0))
    ax = axs[0]
    ax.plot(XH.CT_S, XH.CP_S, '-', color=C_TEST, lw=0.9, label='test (curve fit)')
    ax.plot(cts, [r['cp_s'] for r in rows], 'o-', color=C_MOD, ms=2.5, label='model')
    ax.set_xlabel(r'$C_T/\sigma$'); ax.set_ylabel(r'$C_P/\sigma$'); ax.legend(frameon=False, loc='upper left'); tag(ax, 'a')
    ax = axs[1]
    fm_t = [(c * XH.SIGMA) ** 1.5 / math.sqrt(2) / (p * XH.SIGMA) for c, p in zip(XH.CT_S, XH.CP_S) if c > 0]
    ax.plot([c for c in XH.CT_S if c > 0], fm_t, '-', color=C_TEST, lw=0.9, label='test')
    ax.plot(cts, [r['fm'] for r in rows], 'o-', color=C_MOD, ms=2.5, label='model')
    ax.set_xlabel(r'$C_T/\sigma$'); ax.set_ylabel('figure of merit'); ax.set_ylim(0, 0.9); tag(ax, 'b')
    ax = axs[2]
    th = np.linspace(-8, 15, 10)
    ax.plot(th, XH.DCTS_DCOLL * (th - XH.COLL_ZERO_THRUST), '--', color=C_TEST, lw=0.9, label='test (actuator)')
    ax.plot([r['th75'] for r in rows], cts, 'o-', color=C_MOD, ms=2.5, label=r'model ($\theta_{75}$)')
    ax.set_xlabel('collective, deg'); ax.set_ylabel(r'$C_T/\sigma$'); ax.legend(frameon=False, loc='lower right', fontsize=6.5); tag(ax, 'c')
    fig.tight_layout(w_pad=1.0)
    save(fig, 'hover')


SETS = [('nac90', 'nacelle 90$^\\circ$, flaps 20$^\\circ$'), ('nac60', 'nacelle 60$^\\circ$, flaps 20$^\\circ$'),
        ('nac30', 'nacelle 30$^\\circ$, flaps 20$^\\circ$'), ('nac0f20', 'airplane mode, flaps 20$^\\circ$'), ('nac0', 'airplane mode, flaps up')]


# the model's altitudes in the XV-15's comparisons: 16 000 ft only in airplane mode, where its fastest point was flown
ALTS = (('0', '-', 'model, sea level'), ('10000', '--', 'model, 10 000 ft'), ('16000', '-.', 'model, 16 000 ft'))


def xv15():
    fig, axs = plt.subplots(1, 5, figsize=(W2, 1.95), sharey=True)
    for i, (ax, (name, lab)) in enumerate(zip(axs, SETS)):
        r = res('xv15_' + name)
        test_pts(ax, XF.SETS[name][2], 'flight test')
        for alt, ls, l in ALTS:
            rows = [x for x in r.get(alt, []) if x['res'] < 0.05]
            if rows:
                ax.plot([x['V'] for x in rows], [x['hp'] for x in rows], ls, color=C_MOD, lw=1.0, label=l)
        if name == 'nac0':
            rg = res('xv15_nac0_gear')
            rows = [x for x in rg['10000'] if x['res'] < 0.05]
            ax.plot([x['V'] for x in rows], [x['hp'] for x in rows], ':', color=C_REF, lw=1.0, label='gear down, 10 000 ft')
            # only the lines (a)'s legend does not explain
            extra = [h for h in ax.get_lines() if h.get_label() in (ALTS[2][2], 'gear down, 10 000 ft')]
            ax.legend(extra, [h.get_label() for h in extra], frameon=False, loc='upper left', fontsize=6.0)
        ax.set_title('(%s) %s' % ('abcde'[i], lab), loc='left', fontsize=7.2, pad=3)
        ax.set_xlabel('KCAS')
        if i == 0:
            ax.set_ylabel('rotor shaft power, hp')
    axs[0].legend(frameon=False, loc='upper center', fontsize=6.5)
    fig.tight_layout(w_pad=0.5)
    save(fig, 'xv15')


def xv15_trim():
    """the trim elevator against the flight test's [TM-84222 fig. 21]"""
    fig, axs = plt.subplots(1, 5, figsize=(W2, 1.95), sharey=True)
    for i, (ax, (name, lab)) in enumerate(zip(axs, SETS)):
        r = res('xv15_' + name)
        test_pts(ax, XF.TRIM_DE[name], 'flight test')
        for alt, ls, l in ALTS:
            rows = [x for x in r.get(alt, []) if x['res'] < 0.05]
            if rows:
                ax.plot([x['V'] for x in rows], [-x['de'] for x in rows], ls, color=C_MOD, lw=1.0, label=l)
        ax.axhline(0, color=C_REF, lw=0.5)
        ax.set_title('(%s) %s' % ('abcde'[i], lab), loc='left', fontsize=7.2, pad=3)
        ax.set_xlabel('KCAS')
        if i == 0:
            ax.set_ylabel('elevator, deg trailing edge down')
    h, l = axs[0].get_legend_handles_labels()
    h16 = [x for x in axs[-1].get_lines() if x.get_label() == ALTS[2][2]]     # (the 16 000 ft line, airplane mode's)
    axs[0].legend(h + h16[:1], l + [x.get_label() for x in h16[:1]], frameon=False, loc='upper left', fontsize=6.3)
    fig.tight_layout(w_pad=0.5)
    save(fig, 'xv15_trim')


def download_polar():
    fig, axs = plt.subplots(1, 2, figsize=(W1 * 2.0, 1.9))
    d = res('download')
    ax = axs[0]
    test_pts(ax, [(r['flap'], 100 * r['test']) for r in d['xv15']], 'XV-15 flight test')
    ax.plot([r['flap'] for r in d['xv15']], [100 * r['dl_t'] for r in d['xv15']], 'o-', color=C_MOD, ms=2.5, label='XV-15 model')
    test_pts(ax, [(r['flap'], 100 * r['test']) for r in d['v22']], 'V-22, 0.658 scale', color=C_REF, marker='s')
    ax.plot([r['flap'] for r in d['v22']], [100 * r['dl_t'] for r in d['v22']], 's', color=C_MOD, ms=3.5, label='V-22 model')
    ax.set_xlabel('flap, deg'); ax.set_ylabel('download, % of thrust'); ax.legend(frameon=False, fontsize=6.5); tag(ax, 'a')
    p = res('polar')
    ax = axs[1]
    test_pts(ax, [(cd, cl) for cl, cd in p['test']], 'Boeing estimate')
    ax.plot([r['CD'] for r in p['rows']], [r['CL'] for r in p['rows']], 'o-', color=C_MOD, ms=2.5, label='model')
    ax.set_xlabel('$C_D$'); ax.set_ylabel('$C_L$'); ax.legend(frameon=False, loc='lower right'); tag(ax, 'b')
    fig.tight_layout(w_pad=1.2)
    save(fig, 'download_polar')


def perf():
    fig, axs = plt.subplots(1, 2, figsize=(W2, 2.1))
    ax = axs[0]
    for r, ls in zip(res('speed'), ('-', ':', '--')):
        rows = [x for x in r['rows'] if x[3] < 0.05]
        lab = '%.0f lb, %s ft' % (r['gw'], '{:,}'.format(r['alt']).replace(',', ' '))
        ax.plot([x[0] for x in rows], [x[1] for x in rows], ls, color=C_MOD, label='required, ' + lab)
        ax.plot([x[0] for x in rows], [x[2] for x in rows], ls, color=C_REF, lw=0.8)
        ax.plot([r['vmax']], [np.interp(r['vmax'], [x[0] for x in rows], [x[1] for x in rows])], 'o', color=C_MOD, ms=3)
    for v, alt in ((274, 0), (321, 17000)):
        ax.axvline(v, color=C_TEST, lw=0.6, ls=':')
    ax.text(274, ax.get_ylim()[1] * 0.98, ' 274', fontsize=6.5, va='top'); ax.text(321, ax.get_ylim()[1] * 0.98, ' 321', fontsize=6.5, va='top')
    ax.set_xlabel('KTAS'); ax.set_ylabel('power, hp (grey: available)'); ax.legend(frameon=False, fontsize=6.3, loc='upper left'); tag(ax, 'a')
    ax = axs[1]
    ce = [res('ceiling%d' % a) for a in CEILING_ALTS]
    ax.plot([c['gw'] / 1000 for c in ce], [c['alt'] / 1000 for c in ce], 'o-', color=C_MOD, ms=3, label='model')
    test_pts(ax, [(gw / 1000, a / 1000) for gw, a in VF.HOVER_CEILING], 'Boeing chart')
    ax.set_xlabel('gross weight, 1000 lb'); ax.set_ylabel('hover ceiling OGE, 1000 ft'); ax.legend(frameon=False); tag(ax, 'b')
    fig.tight_layout(w_pad=1.2)
    save(fig, 'perf')


def conversion():
    fig, axs = plt.subplots(1, 2, figsize=(W2, 2.2))
    for n in NACELLES:
        rows = [r for r in res('conversion%d' % n)['rows'] if r['res'] < 0.05]
        c = NAC_C[n]
        axs[0].plot([r['V'] for r in rows], [r['wing_pct'] for r in rows], '-', color=c, label='%d$^\\circ$' % n)
        test_pts(axs[0], VF.LIFT_SHARE[n], None, color=c)
        axs[1].plot([r['V'] for r in rows], [r['pitch'] for r in rows], '-', color=c)
        test_pts(axs[1], VF.PITCH[n], None, color=c)
    axs[0].set_xlabel('KCAS'); axs[0].set_ylabel('airframe lift, % of weight'); axs[0].legend(frameon=False, title='nacelle', fontsize=6.5, title_fontsize=6.5); tag(axs[0], 'a')
    axs[1].set_xlabel('KCAS'); axs[1].set_ylabel('pitch attitude, deg'); tag(axs[1], 'b')
    fig.tight_layout(w_pad=1.2)
    save(fig, 'conversion')


def handling():
    fig, axs = plt.subplots(2, 2, figsize=(W2, 3.7))
    h = res('handling_hover')
    ax = axs[0, 0]
    ax.plot(h['t'], [a - h['theta'][0] for a in h['theta']], '-', color=C_MOD, label=r'pitch $\Delta\theta$')
    ax.plot(h['t'], h['phi'], '-', color='#2166ac', label=r'roll $\phi$')
    ax.plot(h['t'], h['psi'], '-', color='#1b7837', label=r'heading $\Delta\psi$')
    for t, a, v in h['events']:
        ax.axvline(t, color=C_REF, lw=0.6, ls=':')
        ax.text(t, 2.9, ' %s %+.2f' % (a, v), fontsize=6, rotation=90, va='top')
    ax.set_xlabel('time, s'); ax.set_ylabel('deg'); ax.set_ylim(-10, 3); ax.legend(frameon=False, fontsize=6.3, loc='lower left'); tag(ax, 'a')
    c = res('handling_conversion')
    ax = axs[0, 1]
    ax.plot(c['t'], c['theta'], '-', color=C_MOD, label=r'pitch $\theta$, deg')
    ax.plot(c['t'], c['nacelle'], '-', color='#ef8a62', label='nacelle, deg')
    ax.plot(c['t'], c['kcas'], '-', color='#2166ac', label='KCAS')
    ax.set_xlabel('time, s'); ax.legend(frameon=False, fontsize=6.3, loc='center right'); tag(ax, 'b')
    ax = axs[1, 0]
    tm = json.load(open(os.path.join(FDM, 'data', 'trim_ff_v22.json')))
    (lv, ln), (uv, un) = V.CPS
    for n, row in zip(tm['nacelle'], tm['lon']):
        if n not in NAC_C and n not in (15, 45):
            continue
        ks = [k for k in tm['kcas'] if np.interp(k, lv, ln) - 0.1 <= n <= np.interp(k, uv, un) + 0.1]
        ys = [y for k, y in zip(tm['kcas'], row) if k in ks]
        ax.plot(ks, ys, 'o-', ms=2, color=NAC_C.get(n, C_REF), label='%d$^\\circ$' % n)
    ax.set_xlabel('KCAS'); ax.set_ylabel('trim stick, + forward'); ax.legend(frameon=False, ncol=4, fontsize=6, title='nacelle', title_fontsize=6); tag(ax, 'c')
    s = res('start')
    ax = axs[1, 1]
    ax.plot(s['t'], s['nr'], '-', color=C_MOD, label='rotor speed, %')
    ax.plot(s['t'], [100 * a for a in s['spool0']], '-', color='#2166ac', label='engine 1, % of idle')
    ax.plot(s['t'], [100 * a for a in s['spool1']], '--', color='#2166ac', label='engine 2')
    ax.plot(s['t'], [100 * a for a in s['gov']], '-', color='#1b7837', lw=0.8, label='governor engaged, %')
    ax.set_xlabel('time, s'); ax.legend(frameon=False, fontsize=6.3, loc='lower right'); tag(ax, 'd')
    fig.tight_layout(w_pad=1.2, h_pad=0.8)
    save(fig, 'handling')


def hover_hold():
    fig, axs = plt.subplots(2, 2, figsize=(W2, 3.7))
    ax = axs[0, 0]
    for r, c in zip(res('hover_hold_stop'), ('#2166ac', '#1b7837', C_MOD)):
        t = [a - 1.0 for a in r['t']]
        ax.plot(t, r['gs'], '-', color=c, label='%.0f kt' % r['V'])
        ax.plot(t, r['theta'], ':', color=c, lw=0.8)
        n0 = r['n'][int(round(1.0 / 0.2))]
        ax.plot(t, [10 * (a - n0) / r['shown_ft'] for a in r['n']], '--', color=c, lw=0.8)
    ax.axhline(10, color=C_REF, lw=0.5, ls='-')
    ax.set_xlim(-1, 40); ax.set_ylim(-3, 42)
    ax.set_xlabel('time from engaging, s'); ax.set_ylabel('ground speed, kt; pitch, deg (dotted)')
    ax.text(30, 11, 'distance / shown stop (dashed), x10', fontsize=6, ha='center')
    ax.legend(frameon=False, fontsize=6.3, loc='upper right'); tag(ax, 'a')
    ax = axs[0, 1]
    th = np.linspace(0, 2 * np.pi, 100)
    ax.plot(5 * np.cos(th), 5 * np.sin(th), '-', color=C_REF, lw=0.8, label='10 ft circle')
    for r, c in zip(res('hover_hold_turn'), ('#2166ac', '#1b7837', C_MOD)):
        w = [i for i, t in enumerate(r['t']) if r['t_on'] <= t <= r['t_off'] + 10]
        n0, e0 = np.array(r['n'])[w[0]], np.array(r['e'])[w[0]]
        ax.plot([r['e'][i] - e0 for i in w], [r['n'][i] - n0 for i in w], '-', color=c, lw=0.9, label='%.0f kt wind' % r['wind_kt'])
    ax.set_aspect('equal'); ax.set_xlabel('east, ft'); ax.set_ylabel('north, ft (wind from north)')
    ax.legend(frameon=False, fontsize=6.3, loc='upper right'); tag(ax, 'b')
    c = res('hover_hold_controls')
    ax = axs[1, 0]
    tr = c['trc']
    ax.plot(tr['t'], tr['gs'], '-', color=C_MOD, label='ground speed, kt')
    ax.plot(tr['t'], tr['theta'], '-', color='#2166ac', lw=0.8, label='pitch, deg')
    ax.axvline(5, color=C_REF, lw=0.6, ls=':'); ax.axvline(25, color=C_REF, lw=0.6, ls=':')
    ax.text(5, 6.5, ' stick +0.25', fontsize=6); ax.text(25, 6.5, ' released', fontsize=6)
    ax.set_xlabel('time, s'); ax.set_ylim(-2, 7); ax.legend(frameon=False, fontsize=6.3, loc='upper right', bbox_to_anchor=(1.0, 0.9)); tag(ax, 'c')
    ax = axs[1, 1]
    lv = c['lever']
    ax.plot(lv['t'], [a - lv['h0'] for a in lv['h']], '-', color=C_MOD, label='height change, ft')
    ax.plot(lv['t'], [100 * (a - lv['tcl'][0]) for a in lv['tcl']], '-', color='#2166ac', lw=0.8, label='lever, % from its start')
    ax.axvspan(0, 4, color='#eeeeee', lw=0); ax.text(2, 1.5, 'lever\nmoved', fontsize=6, ha='center')
    ax.set_xlabel('time, s'); ax.legend(frameon=False, fontsize=6.3, loc='lower right'); tag(ax, 'd')
    fig.tight_layout(w_pad=1.2, h_pad=0.8)
    save(fig, 'hover_hold')


def tf():
    nm = 6076.1
    sp_c = {120: '#2166ac', 150: '#1b7837', 170: '#ef8a62', 200: C_MOD}
    fig, axs = plt.subplots(2, 2, figsize=(W2, 3.9))
    for ax, name, speeds, lab in ((axs[0, 0], 'hills', (120, 150, 200), 'a'), (axs[0, 1], 'ridge', (120, 150, 170, 200), 'b')):
        r0 = res('tf_%s_%d' % (name, speeds[0]))
        x0, t0 = np.array(r0['y']) / nm, np.array(r0['terr'])
        ax.fill_between(x0, 0, t0, color='#dddddd', lw=0, label='terrain')
        ax.plot(x0, t0 + 200, '-', color=C_REF, lw=0.6, label='terrain + 200 ft')
        for V in speeds:
            r = res('tf_%s_%d' % (name, V))
            ax.plot(np.array(r['y']) / nm, np.array(r['terr']) + np.array(r['hw']), '-', color=sp_c[V], lw=0.9, label='%d kt' % V)
        ax.set_xlim(0, 6 if name == 'hills' else 4.5); ax.set_ylim(0, 1300 if name == 'ridge' else 1050)
        ax.set_xlabel('distance, nm'); ax.set_ylabel('wheels above sea level, ft')
        ax.legend(frameon=False, fontsize=6.0, loc='upper left', ncol=2); tag(ax, lab)
    ax = axs[1, 0]
    rp = {r['kcas']: r for r in res('tf_response')}
    for V, c in ((60.0, '#762a83'), (120.0, '#2166ac'), (150.0, '#1b7837'), (200.0, C_MOD)):
        r = rp[V]
        t = np.array(r['t'])
        ax.plot(t, r['y'], '-', color=c, lw=0.9, label='%.0f kt' % V)
        w = r['w']
        ax.plot(t, 1 - w * np.exp(-t / r['tau1']) - (1 - w) * np.exp(-t / r['tau2']), '--', color=c, lw=0.7)
    ax.set_xlim(0, 15); ax.set_ylim(0, 1.3)
    ax.set_xlabel('time from the step, s'); ax.set_ylabel('vertical speed / its command')
    ax.text(14.5, 0.12, 'measured (solid), model (dashed)', fontsize=6, ha='right')
    ax.legend(frameon=False, fontsize=6.3, loc='lower right', bbox_to_anchor=(1.0, 0.18)); tag(ax, 'c')
    ax = axs[1, 1]
    for V in (120, 150, 170, 200):
        r = res('tf_ridge_%d' % V)
        ax.plot(np.array(r['y']) / nm, r['hw'], '-', color=sp_c[V], lw=0.9, label='%d kt' % V)
    ax.axhline(200, color=C_REF, lw=0.6)
    ax.axvline(2.5, color=C_REF, lw=0.5, ls=':'); ax.text(2.505, 330, 'ridge edge', fontsize=6)
    ax.set_xlim(2.2, 2.8); ax.set_ylim(150, 450)
    ax.set_xlabel('distance, nm'); ax.set_ylabel('clearance of the wheels, ft')
    ax.legend(frameon=False, fontsize=6.3, loc='upper right'); tag(ax, 'd')
    fig.tight_layout(w_pad=1.2, h_pad=0.8)
    save(fig, 'tf')


def freq():
    """the XV-15's frequency responses: the flight tests' identified transfer functions over their fitting ranges, Bell's
    simulation's in cruise, the model's"""
    from data import xv15_freq as XQ
    fig, axs = plt.subplots(4, 5, figsize=(W2, 5.6))
    lab = {'p/da': '$p/\\delta_a$', 'r/da': '$r/\\delta_a$', 'r/dr': '$r/\\delta_r$', 'q/de': '$q/\\delta_e$', 'az/dc': '$a_z/\\delta_c$',
           'beta/dr': '$\\beta/\\delta_r$', 'az/de': '$a_z/\\delta_e$'}
    for row, cond in ((0, 'hover'), (2, 'cruise')):
        r = res('freq_' + cond)
        r.pop('derivatives', None)
        ref = XQ.HOVER if cond == 'hover' else XQ.CRUISE
        for j, pair in enumerate(r):
            m = r[pair]
            lo, hi = XQ.RANGE[(cond, pair)]
            wd = np.logspace(math.log10(lo), math.log10(hi), 200)
            Hf = XQ.evaluate(ref[pair], wd)
            # (phases unwrapped alike: each curve kept within 180 deg of the flight's at the same frequency)
            ph_f = np.degrees(np.unwrap(np.angle(Hf)))
            ph_m = np.array(m['deg'])
            ph_m = ph_m + 360 * np.round((np.interp(m['w'], wd, ph_f) - ph_m) / 360)
            a, b = axs[row, j], axs[row + 1, j]
            a.semilogx(wd, 20 * np.log10(abs(Hf)), '-', color=C_TEST, label='flight')
            b.semilogx(wd, ph_f, '-', color=C_TEST)
            if cond == 'cruise':
                Hb = XQ.evaluate(XQ.CRUISE_BELL[pair], wd)
                ph_b = np.degrees(np.unwrap(np.angle(Hb)))
                ph_b = ph_b + 360 * np.round((ph_f - ph_b) / 360)
                a.semilogx(wd, 20 * np.log10(abs(Hb)), '--', color=C_REF, lw=0.9, label="Bell's simulation")
                b.semilogx(wd, ph_b, '--', color=C_REF, lw=0.9)
            a.semilogx(m['w'], m['db'], 'o-', color=C_MOD, ms=2.2, lw=0.9, label='model')
            b.semilogx(m['w'], ph_m, 'o-', color=C_MOD, ms=2.2, lw=0.9)
            a.set_title('%s %s, $J$ %.0f' % (cond, lab[pair], m['J']), loc='left', fontsize=7.2, pad=3)
            b.set_xlabel('rad/s', fontsize=7)
            ticks = [t for t in ((0.1, 0.3, 1, 3, 10) if lo < 0.15 else (0.2, 0.5, 1, 2, 5, 10)) if lo * 0.99 <= t <= hi * 1.01]
            for ax in (a, b):
                ax.set_xticks(ticks); ax.set_xticklabels(['%g' % t for t in ticks])
                ax.xaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
            if j == 0:
                a.set_ylabel('dB'); b.set_ylabel('deg')
    axs[2, 4].axis('off'); axs[3, 4].axis('off')
    h, l = axs[2, 0].get_legend_handles_labels()
    axs[2, 4].legend(h, l, frameon=False, loc='center', fontsize=7)
    fig.tight_layout(w_pad=0.4, h_pad=0.5)
    save(fig, 'freq')


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    for fn in (hover, xv15, xv15_trim, download_polar, perf, conversion, handling, hover_hold, tf, freq):
        fn()
