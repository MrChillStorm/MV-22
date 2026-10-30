#!/usr/bin/env python3
"""The V-22 airframe's lift and drag in airplane mode against Boeing's full-scale estimate [Rosenstein and Clark,
ERF 1986, fig. 28: lift coefficient 0.18 at zero angle of attack, about 0.093 per deg; minimum drag coefficient
0.067 near CL 0.3, 0.077 at 0.55, on 382 ft^2, Mach 0.2-0.4]: the airframe's loads alone (aero/airframe/*, which the
test build keeps named with --no-inline), with the rotors at zero thrust, flaps and gear up, at 198 kt (Mach 0.3) and 3000 ft.
v22data.DRAG_EXTRA is fitted to the drag at CL 0.2-0.6.

  python3 genv22.py root-polar/aircraft/mv22b --no-inline && python3 airframe_polar.py"""
import math, os
import harness as H

ROOT = os.path.join(H.HERE, 'root-polar')
TEST = [(0.18, 0.068), (0.30, 0.067), (0.55, 0.077), (1.0, 0.115)]     # (CL, CD), traced


def polar(alphas=(-2, 0, 2, 4, 6, 8), gov=10.8):
    f = H.load(root=ROOT)
    f['ctl/flaperon-override'] = 1          # flaps up, as the estimate is
    f['ctl/flaperon-set-deg'] = 0.0
    f['ctl/init'] = 1
    H.freeze(f)
    H.controls(f, gov, 0, 0, 0, 0)        # about zero thrust at 198 kt in airplane mode
    out = []
    for a in alphas:
        f['ic/h-sl-ft'] = 3000; f['ic/vt-kts'] = 198; f['ic/alpha-deg'] = a; f['ic/gamma-deg'] = 0
        f['ic/terrain-elevation-ft'] = H.TERRAIN_FT
        f.run_ic()
        for _ in range(300):
            f.run()
        q, S = f['aero/qbar-psf'], 382.0
        X, Z = f['aero/airframe/FX-lbs'], f['aero/airframe/FZ-lbs']
        al = math.radians(f['aero/alpha-deg'])
        L, D = -Z * math.cos(al) + X * math.sin(al), -X * math.cos(al) - Z * math.sin(al)
        out.append(dict(alpha=a, CL=L / (q * S), CD=D / (q * S), thrust=f['aero/rotor/r/thrust-lbs']))
    return out


if __name__ == '__main__':
    for r in polar():
        print('alpha %4.1f  CL %.3f  CD %.4f  (rotor thrust %5.0f lb)' % (r['alpha'], r['CL'], r['CD'], r['thrust']))
    print('test (CL, CD):', TEST)
