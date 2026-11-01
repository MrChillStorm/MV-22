#!/usr/bin/env python3
"""The flight manual's charts (fig/*.pdf) from the aircraft's own data: the conversion corridor and AUTO NAC's schedule
(fdm/v22data.py, fdm/data/autonac_v22.json), the hover ceiling (fdm/report/results), the climb at full power
(fdm/data/climb_power_v22.json), the vortex ring state's boundary"""
import os, sys, json, glob
HERE = os.path.dirname(os.path.abspath(__file__))
FDM = os.path.join(os.path.dirname(HERE), 'fdm')
sys.path.insert(0, FDM)
import numpy as np
import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
import v22data as V

plt.rcParams.update({'font.family': 'serif', 'font.serif': ['STIX Two Text', 'STIXGeneral', 'DejaVu Serif'], 'mathtext.fontset': 'stix',
                     'font.size': 8.5, 'axes.labelsize': 8.5, 'legend.fontsize': 7.5, 'xtick.labelsize': 7.5, 'ytick.labelsize': 7.5,
                     'axes.linewidth': 0.6, 'lines.linewidth': 1.2, 'grid.linewidth': 0.3, 'grid.alpha': 0.5, 'axes.grid': True,
                     'savefig.bbox': 'tight', 'savefig.pad_inches': 0.02, 'axes.spines.top': False, 'axes.spines.right': False})
OUT = os.path.join(HERE, 'fig')
C1, C2, C3 = '#b2182b', '#2166ac', '#1b7837'


def save(fig, name):
    fig.savefig(os.path.join(OUT, name + '.pdf')); plt.close(fig)


def corridor():
    (lv, ln), (uv, un) = V.CPS
    k = np.linspace(0, 300, 601)
    lo, hi = np.interp(k, lv, ln), np.interp(k, uv, un)
    fig, ax = plt.subplots(figsize=(5.2, 3.0))
    ax.fill_between(k, lo, hi, color='#e8eef6', lw=0, label='conversion corridor')
    ax.plot(k, lo, '-', color=C2, lw=1.0); ax.plot(k, hi, '-', color=C2, lw=1.0)
    from genv22 import autonac_schedule
    sk, sn = autonac_schedule()
    ax.plot(sk, sn, '-', color=C1, lw=1.4, label='AUTO NAC schedule')
    ax.axvline(V.VMO_APLN, color='#7a7a7a', lw=0.8, ls='--'); ax.text(V.VMO_APLN - 3, 60, '%g KCAS (84 %% $N_R$)' % V.VMO_APLN, rotation=90, fontsize=6.5, ha='right')
    ax.axvline(V.VMO_APLN_100, color='#7a7a7a', lw=0.8, ls=':'); ax.text(V.VMO_APLN_100 - 3, 60, '%g KCAS at 100 %% $N_R$' % V.VMO_APLN_100, rotation=90, fontsize=6.5, ha='right')
    ax.set_xlim(0, 300); ax.set_ylim(-2, 100)
    ax.set_xlabel('equivalent airspeed, kt'); ax.set_ylabel('nacelle angle, deg')
    ax.legend(frameon=False, loc='lower left')
    save(fig, 'corridor')


def perf():
    res = os.path.join(FDM, 'report', 'results')
    fig, axs = plt.subplots(1, 3, figsize=(6.9, 2.3))
    ax = axs[0]
    alts = (0, 4000, 8400, 12000, 16000)
    gw = [json.load(open(os.path.join(res, 'ceiling%d.json' % a)))['gw'] / 1000 for a in alts]
    ax.plot(gw, [a / 1000 for a in alts], 'o-', color=C1, ms=3)
    ax.set_xlabel('gross weight, 1000 lb'); ax.set_ylabel('pressure altitude, 1000 ft'); ax.set_title('(a) hover ceiling OGE', loc='left', fontsize=8.5)
    ax = axs[1]
    cp = json.load(open(os.path.join(FDM, 'data', 'climb_power_v22.json')))
    ax.plot(cp['kcas'], [60 * x for x in cp['climb_full_fps']], 'o-', color=C2, ms=3)
    ax.set_xlabel('KCAS (AUTO NAC)'); ax.set_ylabel('rate of climb, ft/min'); ax.set_title('(b) full-power climb, 1000 ft', loc='left', fontsize=8.5)
    ax.set_ylim(0, None)
    ax = axs[2]
    kv, sk = V.VRS_SINK
    k = np.array([0] + list(kv) + [kv[-1] + 1e-3, 90])
    s = np.array([sk[0]] + list(sk) + [5000, 5000])
    ax.fill_between(k, s, 5000, color='#f4dede', lw=0)
    ax.plot(k[:-2], s[:-2], '-', color=C1); ax.plot([kv[-1], kv[-1]], [sk[-1], 3000], '-', color=C1)
    ax.text(15, 2300, 'high sink rate:\nvortex ring state', fontsize=6.5)
    ax.set_xlim(0, 90); ax.set_ylim(0, 3000)
    ax.set_xlabel('KCAS'); ax.set_ylabel('rate of descent, ft/min'); ax.set_title('(c) high sink rate boundary', loc='left', fontsize=8.5)
    fig.tight_layout(w_pad=1.0)
    save(fig, 'perf')


# ---------------------------------------------------------------- the displays (disp/*.png, capture.py), with callouts
# each picture's callouts: (x, y) px on the display's canvas and what is there, numbered in order; the manual's
# figure lists them (tab/callouts_<picture>.tex), so a callout and its words stay together
FLT = [(14, 18, 'AFCS mode line: the modes, green when coupled'),
       (24, 100, 'airspeed tape, KIAS, with its 10\\,s trend; the red bands: the conversion corridor\'s limits for the nacelle angle'),
       (14, 178, 'airspeed'),
       (256, 200, 'flight path marker'),
       (222, 160, 'acceleration chevron: above the marker accelerating, below slowing'),
       (422, 152, 'altitude, with its tape and trend'),
       (500, 120, 'vertical speed scale; the bar coloured by the high sink rate limit'),
       (210, 352, 'heading'),
       (24, 341, 'ground speed'),
       (395, 341, 'radar altitude'),
       (24, 425, 'nacelle angle in the conversion corridor: where the aircraft is (circle) and its limits'),
       (466, 400, 'power: torque against the first limit; OGE and IGE: the power to hover out of and in ground effect'),
       (500, 470, 'rotor speed'),
       (256, 482, 'gear, flaperons and the augmentation\'s response type')]
CALLOUTS = {
    'flt_conversion': FLT, 'flt_airplane': FLT,
    'flt_hover': [(14, 18, 'AFCS mode line: the hover hold\'s state (DECEL, HOLD, TRC) and height hold'),
                  (228, 196, 'the hover point: the hold\'s stop, filled once it holds it; its distance'),
                  (282, 238, 'the drift ball, where the aircraft is drifting 3\\,s on, to fly the stick towards'),
                  (338, 190, 'the ring at 10\\,kt of drift'),
                  (422, 152, 'radar height'),
                  (444, 300, 'below 200\\,ft: the ground closing, a bar that drains as the height goes'),
                  (395, 341, 'barometric altitude'),
                  (500, 285, 'the high sink rate limit for the airspeed'),
                  (14, 341, 'ground speed'),
                  (24, 425, 'nacelle angle in the conversion corridor'),
                  (466, 400, 'power, with the power to hover out of and in ground effect'),
                  (256, 482, 'the augmentation\'s response type: attitude command')],
    'nav': [(100, 14, 'next waypoint: distance and time to it'),
            (256, 40, 'track'),
            (395, 34, 'range; track or north up; terrain shown'),
            (240, 196, 'the route'),
            (52, 262, 'the landing zone, the route\'s last point, with its approach into the wind'),
            (106, 214, 'the conversion planner: where to begin converting (CONV) and slowing (DECEL)'),
            (165, 282, 'terrain coloured by its clearance below the aircraft\'s height when it gets there'),
            (250, 100, 'no elevation yet from the scenery'),
            (238, 290, 'the path 30 and 60\\,s ahead'),
            (400, 335, 'fuel: endurance and range at the reserve'),
            (250, 470, 'the profile along the track: terrain, the path, the selected altitude')],
    'eng': [(70, 290, 'each engine\'s torque against its limit (the line)'),
            (70, 352, 'torque, \\%'),
            (256, 128, 'rotor speed and its reference'),
            (256, 255, 'the drive\'s power and its limit'),
            (40, 428, 'gas generator speed, shaft power, fuel flow'),
            (40, 14, 'the formats\' keys')],
    'fuel': [(170, 146, 'total fuel'),
             (30, 200, 'the five tanks'),
             (30, 432, 'fuel flow and endurance')],
    'esfi': [(256, 18, 'heading'),
             (30, 200, 'airspeed'),
             (256, 160, 'attitude'),
             (480, 200, 'altitude'),
             (400, 440, 'altimeter setting')],
}


def displays():
    import matplotlib.image as mpimg
    for png in sorted(glob.glob(os.path.join(HERE, 'disp', '*.png'))):
        name = os.path.splitext(os.path.basename(png))[0]
        img = mpimg.imread(png)
        h, w = img.shape[:2]
        fig = plt.figure(figsize=(w / 160, h / 160))
        ax = fig.add_axes([0, 0, 1, 1])
        ax.imshow(img, interpolation='none'); ax.set_axis_off()     # (embedded at its own pixels)
        calls = CALLOUTS.get(name, [])
        for i, (x, y, _) in enumerate(calls, 1):
            ax.annotate(str(i), (x, y), ha='center', va='center', fontsize=6.5, fontweight='bold', color='black',
                        bbox=dict(boxstyle='circle,pad=0.18', fc='#ffd400', ec='black', lw=0.5))
        fig.savefig(os.path.join(OUT, 'disp_' + name + '.pdf'), dpi=300); plt.close(fig)
        os.makedirs(os.path.join(HERE, 'tab'), exist_ok=True)
        open(os.path.join(HERE, 'tab', 'callouts_%s.tex' % name), 'w').write(
            '\\begin{enumerate}[label=\\textbf{\\arabic*},leftmargin=*,itemsep=0pt]\n' +
            ''.join('\\item %s\n' % t for _, _, t in calls) + '\\end{enumerate}\n' if calls else '')


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    corridor(); perf(); displays()
