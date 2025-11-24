
import numpy as np
from lib.geofilter import seg_seg_closest_points, aabb_inflate, time_windows_overlap
from lib.trajectory import position_at_time

def geometric_prefilter(segsA, segsB, safety_dist):
    candidates = []
    for i, sa in enumerate(segsA):
        for j, sb in enumerate(segsB):
            if not time_windows_overlap(sa, sb):
                continue
            loA, hiA = aabb_inflate(sa, safety_dist)
            loB, hiB = aabb_inflate(sb, safety_dist)
            if np.any(hiA < loB) or np.any(hiB < loA):
                continue
            dmin, ua, ub, pa, pb = seg_seg_closest_points(sa['p0'], sa['p1'], sb['p0'], sb['p1'])
            if dmin <= safety_dist + 1e-9:
                candidates.append((i, j, float(dmin), ua, ub))
    return candidates

def time_sample_confirm(segsA, segsB, iA, iB, ua, ub, safety_dist, dt=0.1):
    sA = segsA[iA]; sB = segsB[iB]
    t0 = max(sA['t0'], sB['t0'])
    t1 = min(sA['t1'], sB['t1'])
    if t1 < t0:
        return None
    times = np.arange(t0, t1 + 1e-9, dt)
    for t in times:
        pA = position_at_time(segsA, t)
        pB = position_at_time(segsB, t)
        d = np.linalg.norm(pA - pB)
        if d <= safety_dist + 1e-9:
            return {
                'time': float(t),
                'distance': float(d),
                'position': [float(x) for x in 0.5*(pA + pB)],
                'segA': iA,
                'segB': iB
            }
    return None

def simple_deconflict_pipeline(segsA, segsB, safety_dist=2.0, dt=0.1):
    conflicts = []
    candidates = geometric_prefilter(segsA, segsB, safety_dist)
    if not candidates:
        return conflicts
    for (iA, iB, dmin, ua, ub) in candidates:
        conf = time_sample_confirm(segsA, segsB, iA, iB, ua, ub, safety_dist, dt=dt)
        if conf is not None:
            conflicts.append(conf)
    return conflicts
