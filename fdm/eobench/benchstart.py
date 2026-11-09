import sys, math, os
sys.path.insert(0, os.path.expanduser('~/git/V-22/fdm'))
import hoverhold as HH, harness as H, deadstick as DS
def start(lat, lon, hdg, kcas, alt_msl, ground_ft, W=38723.0, nac=0.0):
    """hoverhold.start trimmed at (lat, lon) on heading hdg (harness.IC_*), at W lb"""
    L0 = H.load
    H.load = lambda *a, **k: DS.weigh(L0(*a, **k), W)
    H.IC_PSI_DEG, H.IC_LATLON = hdg, (lat, lon)
    try:
        f = HH.start(kcas, nac, alt=alt_msl, terrain=ground_ft)
    finally:
        H.load = L0
        H.IC_PSI_DEG, H.IC_LATLON = 0.0, None
    return f
