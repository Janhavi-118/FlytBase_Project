
import numpy as np
from lib.geofilter import seg_seg_closest_points, aabb_inflate, time_windows_overlap
from lib.trajectory import position_at_time

import collections

def grid_cell(pos, cell_size):
    return tuple((pos // cell_size).astype(int))

def geometric_prefilter_grid(segsA, segsB, safety_dist, cell_size=1.0):
    grid = collections.defaultdict(list)

    def covered_cells(seg):
        lo, hi = aabb_inflate(seg, safety_dist)
        lo_cell = grid_cell(lo, cell_size)
        hi_cell = grid_cell(hi, cell_size)
        cells = []
        for x in range(lo_cell[0], hi_cell[0] + 1):
            for y in range(lo_cell[1], hi_cell[1] + 1):
                for z in range(lo_cell[2], hi_cell[2] + 1):
                    cells.append((x, y, z))
        return cells

    for i, seg in enumerate(segsA):
        for cell in covered_cells(seg):
            grid[cell].append(('A', i))
    for j, seg in enumerate(segsB):
        for cell in covered_cells(seg):
            grid[cell].append(('B', j))

    candidates = []
    tested_pairs = set()

    for cell, segs in grid.items():
        A_indices = [idx for sset, idx in segs if sset == 'A']
        B_indices = [idx for sset, idx in segs if sset == 'B']

        for i in A_indices:
            sa = segsA[i]
            for j in B_indices:
                if (i, j) in tested_pairs:
                    continue
                tested_pairs.add((i, j))
                sb = segsB[j]

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
    candidates = geometric_prefilter_grid(segsA, segsB, safety_dist)
    if not candidates:
        return conflicts
    for (iA, iB, dmin, ua, ub) in candidates:
        conf = time_sample_confirm(segsA, segsB, iA, iB, ua, ub, safety_dist, dt=dt)
        if conf is not None:
            conflicts.append(conf)
    return conflicts
