"""Helpers for digitising scanned plots: the plot frame, its tick marks, and circle markers by template matching.

    ax = Axes(img)                      # frame from the longest dark lines
    yt = ax.yticks(); xt = ax.xticks()  # tick positions (pixels), to be paired with their labels
    pts = circles(img, r=8, box=...)    # marker centres (pixels)
"""
import numpy as np
from PIL import Image
from scipy.signal import fftconvolve
from scipy.ndimage import maximum_filter


def load(path, thresh=140):
    a = np.asarray(Image.open(path).convert('L'), float)
    return a, a < thresh


def runs(line):
    """longest run of True in a 1-D boolean array: (length, start)"""
    best, start, cur, s0 = 0, 0, 0, 0
    for i, x in enumerate(line):
        if x:
            if cur == 0:
                s0 = i
            cur += 1
            if cur > best:
                best, start = cur, s0
        else:
            cur = 0
    return best, start


class Axes:
    def __init__(s, dark, box=None):
        """box = (x0, y0, x1, y1) to search in; the left axis is the column with the longest vertical dark run, the
        bottom axis the row with the longest horizontal run"""
        s.dark = dark
        H, W = dark.shape
        x0, y0, x1, y1 = box or (0, 0, W, H)
        s.box = (x0, y0, x1, y1)
        cols = [(runs(dark[y0:y1, x])[0], x) for x in range(x0, x1)]
        s.left = max(cols)[1]
        rows = [(runs(dark[y, x0:x1])[0], y) for y in range(y0, y1)]
        s.bottom = max(rows)[1]
        n, st = runs(dark[y0:y1, s.left]); s.top = y0 + st
        n, st = runs(dark[s.bottom, x0:x1]); s.right = x0 + st + n - 1

    def yticks(s, length=(4, 16), side='in'):
        """rows where a short dark segment sticks out of the left axis (inward by default)"""
        d = s.dark
        out = []
        for y in range(s.top, s.bottom + 1):
            seg = d[y, s.left + 2:s.left + 2 + length[1]] if side == 'in' else d[y, s.left - 1 - length[1]:s.left - 1][::-1]
            n = 0
            while n < len(seg) and seg[n]:
                n += 1
            if length[0] <= n < length[1]:
                out.append(y)
        return _cluster(out)

    def xticks(s, length=(4, 16), side='in'):
        d = s.dark
        out = []
        for x in range(s.left, s.right + 1):
            seg = d[s.bottom - 2 - length[1]:s.bottom - 1, x][::-1] if side == 'in' else d[s.bottom + 2:s.bottom + 2 + length[1], x]
            n = 0
            while n < len(seg) and seg[n]:
                n += 1
            if length[0] <= n < length[1]:
                out.append(x)
        return _cluster(out)


def _cluster(v, gap=3):
    out, grp = [], []
    for x in v:
        if grp and x - grp[-1] > gap:
            out.append(sum(grp) / len(grp)); grp = []
        grp.append(x)
    if grp:
        out.append(sum(grp) / len(grp))
    return out


def calib(pix, vals):
    """linear map pixel -> value from tick pixels and their labels (least squares)"""
    A = np.c_[pix, np.ones(len(pix))]
    k, c = np.linalg.lstsq(A, np.array(vals, float), rcond=None)[0]
    return lambda p: k * p + c


def circles(dark, r, box=None, width=2.5, min_score=0.55, exclude=()):
    """centres of open circle markers of radius r (pixels): correlation with a ring, less a filled interior"""
    yy, xx = np.mgrid[-r - 4:r + 5, -r - 4:r + 5]
    rr = np.hypot(xx, yy)
    ring = (np.abs(rr - r) < width / 2 + 0.5).astype(float)
    inner = (rr < r - width).astype(float)
    k = ring / ring.sum() - 0.8 * inner / max(inner.sum(), 1)
    score = fftconvolve(dark.astype(float), k[::-1, ::-1], mode='same')
    peak = (score == maximum_filter(score, size=2 * r)) & (score > min_score)
    ys, xs = np.nonzero(peak)
    out = []
    for y, x in zip(ys, xs):
        if box and not (box[0] <= x <= box[2] and box[1] <= y <= box[3]):
            continue
        if any(e[0] <= x <= e[2] and e[1] <= y <= e[3] for e in exclude):
            continue
        out.append((float(x), float(y), float(score[y, x])))
    return sorted(out)


def trace(dark, x0, y0, x1, window=6):
    """follow a curve from pixel (x0, y0) to column x1: in each column the centre of the dark run nearest the previous
    row (within window pixels); returns [(x, y)]"""
    out, y = [], float(y0)
    step = 1 if x1 >= x0 else -1
    for x in range(x0, x1 + step, step):
        col = dark[:, x]
        lo, hi = int(max(0, y - window)), int(min(len(col), y + window + 1))
        ys = [i for i in range(lo, hi) if col[i]]
        if not ys:
            continue
        runs, cur = [], [ys[0]]
        for i in ys[1:]:
            if i == cur[-1] + 1:
                cur.append(i)
            else:
                runs.append(cur); cur = [i]
        runs.append(cur)
        best = min(runs, key=lambda r: abs((r[0] + r[-1]) / 2 - y))
        y = (best[0] + best[-1]) / 2
        out.append((x, y))
    return out
