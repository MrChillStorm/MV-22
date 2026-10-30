#!/usr/bin/env python3
"""The level-flight trim's longitudinal stick over the nacelle angle and the calibrated airspeed, at the design gross
weight and CG, 1000 ft, inside the conversion corridor (v22data.CPS), for the augmentation's trim feed-forward
(genv22.scas): with it the stick's centre is trimmed level flight through the conversion, and the augmentation's
integrators only take up the weight's and the CG's differences. Outside the corridor, the nearest trim inside it.

  python3 trimmap.py [--write]      (writes data/trim_ff_v22.json)"""
import sys, json
from multiprocessing import Pool
import numpy as np
import harness as H
import v22data as V

NACELLE = [0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0]
KCAS = [0.0, 20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0, 160.0, 180.0, 200.0, 240.0, 280.0]
OUT = 'data/trim_ff_v22.json'


def inside(nac, kcas):
    (lv, ln), (uv, un) = V.CPS
    return np.interp(kcas, lv, ln) - 0.1 <= nac <= np.interp(kcas, uv, un) + 0.1


def column(nac):
    """the trims along the airspeed at one nacelle angle, each from the last"""
    f = H.load()
    out, x0 = {}, None
    for kcas in KCAS:
        if not inside(nac, kcas):
            continue
        x, r, e = H.trim(f, kcas, alt_ft=1000.0, nacelle=nac, cas=True, x0=x0 if x0 is not None else [2.0, 0, 0, 0, 2.0, 0])
        res = float(np.abs(r).max())
        if res < 0.05:
            out[kcas] = float(x[1])
            x0 = x
        print('nacelle %4.1f %5.1f KCAS  lon %6.3f  theta %6.2f  res %.3f' % (nac, kcas, x[1], f['attitude/theta-deg'], res), flush=True)
    return nac, out


def table():
    with Pool() as pool:
        cols = dict(pool.map(column, NACELLE))
    # holes and the outside of the corridor: the nearest trim along the airspeed, then along the nacelle angle
    tab = np.full((len(NACELLE), len(KCAS)), np.nan)
    for i, n in enumerate(NACELLE):
        for j, k in enumerate(KCAS):
            if k in cols[n]:
                tab[i, j] = cols[n][k]
    for i in range(len(NACELLE)):
        ok = np.where(~np.isnan(tab[i]))[0]
        if len(ok):
            for j in range(len(KCAS)):
                if np.isnan(tab[i, j]):
                    tab[i, j] = tab[i, ok[np.argmin(np.abs(ok - j))]]
    return tab


if __name__ == '__main__':
    tab = table()
    for n, row in zip(NACELLE, tab):
        print('%4.0f  %s' % (n, ' '.join('%6.3f' % a for a in row)))
    if '--write' in sys.argv:
        json.dump({'nacelle': NACELLE, 'kcas': KCAS, 'lon': [[round(a, 4) for a in row] for row in tab],
                   'note': 'level-flight trim longitudinal stick (-1..1, + forward), design weight and CG, 1000 ft (trimmap.py)'},
                  open(OUT, 'w'), indent=1)
        print('wrote', OUT)
