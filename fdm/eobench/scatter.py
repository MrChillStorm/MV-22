"""where the landings of the flown bench runs actually came down, by weight: data/eo_scatter_v22.json, the size of the ground a strip has
to keep clear (eosite.nas zone_check, through capability.py and the flight model's properties, as all ENG OUT's numbers by the weight)

    python3 scatter.py RESULT_FILES...      the output of nbatch.py

Of the landings that were intact, so that a flight that ran out of energy miles short doesn't size every strip; the fifth and ninety-fifth
percentiles (at the tenth and ninetieth the strips' own length already holds them, and the landings that matter are the others): short_ft before the strip's start, past_ft past its end at the stop, side_ft off its line (the wheels' half track added),
sink_p90_fps what the wheels meet on level ground. A weight with under MIN_N landings has no row, the weights either side standing for it: from
the 40 of a field sweep alone are no sample of the slow starts"""
import collections, json, os, re, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
FDM = os.path.dirname(HERE)
sys.path.insert(0, FDM)
import v22data as U

MIN_N = 60
LINE = re.compile(r'\s*\d+\s+.*?(\d+) lb: INTACT\s+(-?[\d.]+) ft/s\s+(\d+) KCAS tips\s+[+-][\d.]+, on \S+\s+slope\s+\S+\s*%?, stop\s+(\S+) past the end; \S+ \d+ choices, '
                  r'down ([+-]\d+) ft along its point, ([+-]\d+) across')


def main(paths):
    site = json.load(open(os.path.join(FDM, 'data', 'eo_site_v22.json')))
    rows = collections.defaultdict(list)
    for p in paths:
        for l in open(p):
            m = LINE.match(l)
            if m:
                W, sink, kt, stop, along, across = m.groups()
                rows[int(W)].append((float(sink), None if stop == 'n/a' else float(stop), float(along), abs(float(across)), float(kt)))
    Ws = site['weight_lb']
    out = dict(weight_lb=[], n=[], short_ft=[], past_ft=[], side_ft=[], sink_p90_fps=[], td_kcas=[],
               note='p5 and p95 of the intact landings (eobench/scatter.py); a weight with too few has no row')
    for W in sorted(rows):
        r = rows[W]
        if len(r) < MIN_N:
            print('%6d lb, %3d landings: too few' % (W, len(r)))
            continue
        # (the flare's point is this far before the strip's start: eosite.nas try_cell, needs())
        back = float(np.interp(W, Ws, site['gate_to_50ft_ft'])) + site['flare_err_ft'][0] - max(0.0, float(np.interp(W, Ws, [e[1] for e in site['reach_err_ft']])))
        stops = [x[1] for x in r if x[1] is not None]
        out['weight_lb'].append(W); out['n'].append(len(r))
        out['short_ft'].append(round(max(0.0, -float(np.percentile([x[2] - back for x in r], 5)))))
        out['past_ft'].append(round(max(0.0, float(np.percentile(stops, 95)))))
        out['side_ft'].append(round(float(np.percentile([x[3] for x in r], 95)) + abs(U.MAIN_GEAR[1]) / 12))
        out['sink_p90_fps'].append(round(float(np.percentile([x[0] for x in r], 90)), 1))
        out['td_kcas'].append(round(float(np.median([x[4] for x in r]))))
    path = os.path.join(FDM, 'data', 'eo_scatter_v22.json')
    json.dump(out, open(path, 'w'), indent=1)
    for i, W in enumerate(out['weight_lb']):
        print('%6d lb, %3d landings: short %5d ft, past %5d ft, side %4d ft, sink p90 %.1f ft/s' % (W, out['n'][i], out['short_ft'][i], out['past_ft'][i], out['side_ft'][i], out['sink_p90_fps'][i]))
    print('wrote', path)


if __name__ == '__main__':
    main(sys.argv[1:])
