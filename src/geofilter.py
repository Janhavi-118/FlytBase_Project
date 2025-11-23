
import numpy as np

def seg_seg_closest_points(a0, a1, b0, b1):
    EPS = 1e-9
    u = a1 - a0
    v = b1 - b0
    w0 = a0 - b0
    a = np.dot(u,u)
    b = np.dot(u,v)
    c = np.dot(v,v)
    d = np.dot(u,w0)
    e = np.dot(v,w0)
    D = a*c - b*b
    if D < EPS:
        # parallel: fallback to endpoint projections
        cand = []
        for pt in [a0, a1]:
            ub = np.dot(v, pt - b0) / (c + 1e-9)
            ub = float(np.clip(ub,0.0,1.0))
            pb = b0 + v*ub
            cand.append((np.linalg.norm(pt-pb), pt, pb, 0.0, ub))
        for pt in [b0, b1]:
            ua = np.dot(u, pt - a0) / (a + 1e-9)
            ua = float(np.clip(ua,0.0,1.0))
            pa = a0 + u*ua
            cand.append((np.linalg.norm(pa-pt), pa, pt, ua, 0.0))
        d, pa, pb, ua, ub = min(cand, key=lambda x: x[0])
        return float(d), float(ua), float(ub), pa, pb
    sc = (b*e - c*d) / D
    tc = (a*e - b*d) / D
    sc = np.clip(sc, 0.0, 1.0)
    tc = np.clip(tc, 0.0, 1.0)
    pa = a0 + sc * u
    pb = b0 + tc * v
    return float(np.linalg.norm(pa - pb)), float(sc), float(tc), pa, pb

def aabb_inflate(seg, margin):
    p0, p1 = seg['p0'], seg['p1']
    lo = np.minimum(p0, p1) - margin
    hi = np.maximum(p0, p1) + margin
    return lo, hi

def time_windows_overlap(sa, sb):
    return not (sa['t1'] < sb['t0'] or sb['t1'] < sa['t0'])
