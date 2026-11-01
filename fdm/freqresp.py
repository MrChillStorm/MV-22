#!/usr/bin/env python3
"""The XV-15's bare-airframe frequency responses against the flight tests' [TM-89428 tables 4.1 and 5.1,
data/xv15_freq.py]: the XV-15 build trimmed at Tischler's conditions (hover out of ground effect at 13 000 lb,
nacelles 90 deg, gear down; cruise at 180 KCAS, 10 000 ft, nacelles 0 deg, flaps up, 517 rpm), then flown through a
logarithmic sweep of one control at the trim's power, the rotor speed governed, a pilot's attitude hold (and in cruise
his speed hold by the attitude) keeping it at the trim as the flight tests' pilots did. As in the flight tests the
input is the surface's own deflection (the power lever's in heave), so that the pilot's corrections are part of the
input and the response is the open loop's [TT sec. 3.2]. The frequency response is the cross-spectrum over the
input's auto-spectrum, smoothed over 1/12 decade, with its coherence.

The mismatch with a flight transfer function over its fitting range is Tischler's cost, magnitude (dB) and phase
(deg) errors weighted 1 to 0.01745 (1 dB as 7.6 deg), averaged over 20 frequencies spaced evenly in log and times 20:
under 100 or so a model is held to match well [TT sec. 3.3.2].

  python3 freqresp.py [hover | cruise] [--plot out.png] [--derivs]"""
import sys, math
import numpy as np
import harness as H
from data import bell301 as B
from data import xv15_freq as X
import xv15data as XV

# the pilot's attitude hold, stick (-1..1) per deg and per deg/s, and the yaw damper per deg/s (gains), which keep the
# aircraft at the trim through the sweep as the flight tests' pilots did: in the hover from its control sensitivity,
# about 53 deg/s^2 of pitch and 70 of roll per unit of stick [TT table 4.1 x the gearing], for a loop of about 1.5 rad/s;
# in cruise about 150 and 85 [TT table 5.1 x the gearing], for about 1 rad/s; and in cruise the speed held by the pitch
# attitude, deg per kt, in about 15 s, under the fitting ranges (left alone, the cruise sweeps rolled to 50 deg and
# ended 35 kt fast and 5000 ft low; with the attitude held, the speed, neutral at the trim's power on the power curve's
# flat bottom, wandered to 150 or 215 kt)
# (flaperon: in cruise flaps up, as the XV-15 cruised with its nacelles locked down [TM-84222 fig. 21]; in the hover the
# build's own, 47 deg, its 75/47 setting)
COND = {'hover': dict(V=0.0, alt=0.0, nacelle=90.0, gear=1.0, flaperon=None, gains=(0.03, 0.03, 0.01, 0.0)),
        'cruise': dict(V=180.0, alt=10000.0, nacelle=0.0, gear=0.0, flaperon=0.0, gains=(0.03, 0.02, 0.005, 0.2))}
W0, W1, T = 0.08, 20.0, 240.0           # the sweep, rad/s, s
# the sweeps' amplitudes, of the control's travel (about the flight tests' 10-20 %)
AMP = {'lat': 0.08, 'lon': 0.06, 'ped': 0.08, 'col': 0.03}
# each response: the swept control, the surface it is measured by (its sign to NASA's), the output and its scale
PAIRS = {'hover': {'p/da': ('lat', 'aileron', 'p'), 'r/da': ('lat', 'aileron', 'r'), 'r/dr': ('ped', 'rudder', 'r'),
                   'q/de': ('lon', 'elevator', 'q'), 'az/dc': ('col', 'lever', 'az')},
         'cruise': {'p/da': ('lat', 'aileron', 'p'), 'beta/dr': ('ped', 'rudder', 'beta'), 'q/de': ('lon', 'elevator', 'q'),
                    'az/de': ('lon', 'elevator', 'az')}}
# the model's surfaces in NASA's signs: elevator + trailing edge down (the model's + up), aileron + rolling left (the
# model's differential + right), rudder + nose right (the model's too); the power lever in %
SURFACE = {'elevator': ('ctl/elevator-deg', -1.0), 'aileron': ('ctl/aileron-deg', -1.0), 'rudder': ('ctl/rudder-deg', 1.0),
           'lever': ('/controls/engines/engine[0]/throttle', 100.0)}
OUT = {'p': ('velocities/p-rad_sec', 180 / math.pi), 'q': ('velocities/q-rad_sec', 180 / math.pi),
       'r': ('velocities/r-rad_sec', 180 / math.pi), 'beta': ('aero/beta-deg', 1.0), 'az': ('accelerations/Nz', -1.0)}


def setup(c, root=None):
    """the XV-15 build with the condition's gear and flaperons"""
    f = H.load(xv15=True, root=root)
    f['gear/gear-cmd-norm'] = c['gear']; f['gear/gear-pos-norm'] = c['gear']
    if c['flaperon'] is not None:
        f['ctl/flaperon-override'] = 1; f['ctl/flaperon-set-deg'] = c['flaperon']
    return f


def sweep(cond, ctl, root=None):
    """trimmed at the condition, flown through the sweep of one control: t, the surfaces' (and the lever's)
    deflections and the outputs, each its own signal"""
    c = COND[cond]
    K_ATT, K_RATE, K_YAW, K_V = c['gains']
    f = setup(c, root)
    x, res, ex = H.trim(f, c['V'], alt_ft=c['alt'], nacelle=c['nacelle'], cas=True)
    coll, lon, lat, ped = x[:4]
    th0, ph0, vc0 = f['attitude/theta-deg'], f['attitude/phi-deg'], f['velocities/vc-kts']
    # the lever for the trim's power (drive.IDLE 0.17), the collective the trim's: the governor's integral takes back
    # the lever's feed forward (as handling.free)
    avail = min(f['eng/power-available-hp'], f['eng/xmsn-limit-hp'])
    lever0 = min(1.0, max(0.0, (ex[0] / 2 / avail - 0.17) / 0.83))
    ff0 = f['ctl/theta75-deg'] - f['gov/int-deg']
    f['/controls/engines/engine[0]/throttle'] = lever0
    for _ in range(8):
        f.run()
    coll = x[0] - (f['ctl/theta75-deg'] - f['gov/int-deg'] - ff0)
    H.controls(f, coll, lon, lat, ped)
    f['drive/hold-rpm'] = 0.0
    H.freeze(f, False)
    f['ctl/init'] = 0
    dt = f['simulation/dt']
    n = int(T / dt)
    t = np.arange(n) * dt
    k = math.log(W1 / W0) / T
    d = AMP[ctl] * np.sin(W0 * (np.exp(k * t) - 1) / k) * np.minimum(1, np.minimum(t / 3.0, (T - t) / 3.0))
    sig = {s: np.zeros(n) for s in list(SURFACE) + list(OUT)}
    for i in range(n):
        q, p, r = (math.degrees(f['velocities/%s-rad_sec' % a]) for a in 'qpr')
        lo = lon + K_ATT * (f['attitude/theta-deg'] - th0 - K_V * (f['velocities/vc-kts'] - vc0)) + K_RATE * q + (d[i] if ctl == 'lon' else 0)
        la = lat - K_ATT * (f['attitude/phi-deg'] - ph0) - K_RATE * p + (d[i] if ctl == 'lat' else 0)
        pe = ped - K_YAW * r + (d[i] if ctl == 'ped' else 0)
        # (the sticks and pedals only: the governor's integral is its own from the release)
        f['fcs/elevator-cmd-norm'] = lo; f['fcs/aileron-cmd-norm'] = la; f['fcs/rudder-cmd-norm'] = -pe
        f['/controls/engines/engine[0]/throttle'] = lever0 + (d[i] if ctl == 'col' else 0)
        f.run()
        for s, (prop, g) in list(SURFACE.items()) + list(OUT.items()):
            sig[s][i] = g * f[prop]
    return t, sig, float(np.abs(res).max())


def response(t, x, y, w):
    """the frequency response y/x and its coherence at w (rad/s), from the whole record (Hann window), smoothed over
    1/12 decade"""
    win = np.hanning(len(t))
    Xf = np.fft.rfft((x - x.mean()) * win); Yf = np.fft.rfft((y - y.mean()) * win)
    wf = 2 * math.pi * np.fft.rfftfreq(len(t), t[1] - t[0])
    Sxy, Sxx, Syy = np.conj(Xf) * Yf, np.abs(Xf) ** 2, np.abs(Yf) ** 2
    Hs, coh = [], []
    for wi in w:
        m = (wf > wi * 10 ** (-1 / 24)) & (wf < wi * 10 ** (1 / 24))
        Hs.append(Sxy[m].sum() / Sxx[m].sum())
        coh.append(abs(Sxy[m].sum()) ** 2 / (Sxx[m].sum() * Syy[m].sum()))
    return np.array(Hs), np.array(coh)


def cost(Hm, Ht, wrap=True):
    """Tischler's mismatch cost over the frequencies given (20 of them, log spaced, in the fitting range)"""
    dm = 20 * np.log10(np.abs(Hm)) - 20 * np.log10(np.abs(Ht))
    dp = np.degrees(np.angle(Hm / Ht))
    return float(20 / len(dm) * np.sum(dm ** 2 + 0.01745 * dp ** 2)), float(np.mean(dm)), float(np.mean(np.abs(dp)))


def compare(cond, root=None):
    """each pair's model response against the flight's (and, in cruise, Bell's simulation's) over the fitting range:
    {pair: dict(w, model, coherence, flight, bell, J, dmag_db, dphase_deg)}"""
    runs, out = {}, {}
    flight = X.HOVER if cond == 'hover' else X.CRUISE
    for pair, (ctl, surf, o) in PAIRS[cond].items():
        if ctl not in runs:
            runs[ctl] = sweep(cond, ctl, root)
        t, sig, res = runs[ctl]
        lo, hi = X.RANGE[(cond, pair)]
        w = np.logspace(math.log10(lo), math.log10(hi), 20)
        Hm, coh = response(t, sig[surf], sig[o], w)
        Ht = X.evaluate(flight[pair], w)
        J, dm, dp = cost(Hm, Ht)
        r = dict(w=w, model=Hm, coherence=coh, flight=Ht, J=J, dmag_db=dm, dphase_deg=dp, trim_res=res)
        if cond == 'cruise':
            r['bell'] = X.evaluate(X.CRUISE_BELL[pair], w)
            r['J_bell'] = cost(r['bell'], Ht)[0]
        out[pair] = r
    return out


def pitch_derivatives(root=None):
    """the short period's derivatives in cruise: the model's from its held airframe's residual (M_alpha 1/s^2, Z_w 1/s,
    M_de deg/s^2 per deg of elevator), the flight's from Tischler's q/de [TT table 5.1]: its zero is -Z_w, its
    denominator s^2 - (Z_w + M_q) s + Z_w M_q - M_alpha (M_alpha-dot in M_q), its gain M_de"""
    c = COND['cruise']
    f = setup(c, root)
    x, res, ex = H.trim(f, c['V'], alt_ft=c['alt'], nacelle=c['nacelle'], cas=True)
    V = f['velocities/vt-fps']
    iyy, cgx, cgz = f['inertia/iyy-slugs_ft2'], f['inertia/cg-x-in'], f['inertia/cg-z-in']
    qSc = f['aero/qbar-psf'] * B.S_WP * B.C_WP
    forces = ('aero/rotor/r/FX-lbs', 'aero/rotor/r/FZ-lbs', 'aero/rotor/l/FX-lbs', 'aero/rotor/l/FZ-lbs', 'aero/emp/h/Fx-lbs', 'aero/emp/h/Fz-lbs')
    kw = dict(alt_ft=c['alt'], nacelle=c['nacelle'], cas=True, extra=('ctl/elevator-deg',) + forces)
    d = {}
    for i, h in ((4, 1.0), (1, 0.02)):
        xa = x.copy(); xa[i] += h; xb = x.copy(); xb[i] -= h
        (aa, ea), (ab, eb) = H.residual(f, xa, c['V'], **kw), H.residual(f, xb, c['V'], **kw)
        if i == 4:
            d['M_alpha'] = (aa[4] - ab[4]) / math.radians(2 * h); d['Z_w'] = (aa[2] - ab[2]) / math.radians(2 * h) / V
            # its parts, the pitching moment coefficient per rad: the rotors' hub forces (the hubs in airplane mode ahead of
            # the pivot by the mast), the stabiliser's, and the rest (wing, fuselage, fins, the hubs' moments)
            dF = (ea[1:] - eb[1:]) / math.radians(2 * h)
            hx, hz = XV.FS_PIV - XV.DZ_HUB, XV.WL_PIV
            arm = lambda fs, wl, Fx, Fz: (fs - cgx) / 12 * Fz + (cgz - wl) / 12 * Fx     # nose up +, x forward, z down
            rot = arm(hx, hz, dF[0], dF[1]) + arm(hx, hz, dF[2], dF[3])
            tail = arm(XV.FS_H, XV.WL_H, dF[4], dF[5])
            tot = d['M_alpha'] * iyy
            d.update(CMa=tot / qSc, CMa_rotors=rot / qSc, CMa_tail=tail / qSc, CMa_rest=(tot - rot - tail) / qSc)
        else:
            d['M_de'] = abs(math.degrees(aa[4] - ab[4]) / (ea[0] - eb[0]))
    k, num, den, tau = X.CRUISE['q/de']
    z, (zeta, w) = num[0], den[0]
    Mq = -2 * zeta * w + z
    d.update(flight_M_alpha=-z * Mq - w * w, flight_Z_w=-z, flight_M_de=-k, alpha=f['aero/alpha-deg'], cg=f['inertia/cg-x-in'],
             iyy=f['inertia/iyy-slugs_ft2'], trim_res=float(np.abs(res).max()))
    return d


if __name__ == '__main__':
    if '--derivs' in sys.argv:
        print(pitch_derivatives())
    conds = [a for a in sys.argv[1:] if a in COND] or list(COND)
    for cond in conds:
        res = compare(cond)
        for pair, r in res.items():
            print('%-6s %-8s J %6.0f (Bell\'s %s)  magnitude %+5.1f dB  phase %5.1f deg  coherence %.2f..%.2f' % (
                cond, pair, r['J'], '%.0f' % r['J_bell'] if 'J_bell' in r else '--', r['dmag_db'], r['dphase_deg'],
                r['coherence'].min(), r['coherence'].max()), flush=True)
        if '--plot' in sys.argv:
            import matplotlib; matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(2, len(res), figsize=(3.2 * len(res), 5.5), squeeze=False)
            for j, (pair, r) in enumerate(res.items()):
                lo, hi = X.RANGE[(cond, pair)]
                wd = np.logspace(math.log10(lo), math.log10(hi), 200)
                Hf = X.evaluate((X.HOVER if cond == 'hover' else X.CRUISE)[pair], wd)
                ax[0, j].semilogx(wd, 20 * np.log10(abs(Hf)), 'k', label='flight')
                ax[1, j].semilogx(wd, np.degrees(np.angle(Hf)), 'k')
                if cond == 'cruise':
                    Hb = X.evaluate(X.CRUISE_BELL[pair], wd)
                    ax[0, j].semilogx(wd, 20 * np.log10(abs(Hb)), 'k--', lw=0.8, label='Bell sim')
                    ax[1, j].semilogx(wd, np.degrees(np.angle(Hb)), 'k--', lw=0.8)
                ax[0, j].semilogx(r['w'], 20 * np.log10(abs(r['model'])), 'C3o-', ms=3, label='model')
                ax[1, j].semilogx(r['w'], np.degrees(np.angle(r['model'])), 'C3o-', ms=3)
                ax[0, j].set_title('%s %s (J %.0f)' % (cond, pair, r['J']), fontsize=9)
                ax[1, j].set_xlabel('rad/s')
            ax[0, 0].legend(fontsize=7); ax[0, 0].set_ylabel('dB'); ax[1, 0].set_ylabel('deg')
            fig.tight_layout()
            out = sys.argv[sys.argv.index('--plot') + 1]
            fig.savefig(out.replace('.png', '_%s.png' % cond) if len(conds) > 1 else out, dpi=100)
