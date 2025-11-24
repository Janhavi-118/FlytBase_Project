
import numpy as np

def segments_from_waypoints(waypoints, t_start, speed):
    segs = []
    t = float(t_start)
    for i in range(len(waypoints)-1):
        p0 = np.array(waypoints[i], dtype=float)
        p1 = np.array(waypoints[i+1], dtype=float)
        vec = p1 - p0
        L = np.linalg.norm(vec)
        if L < 1e-6:
            continue
        dt = L / speed
        segs.append({
            "p0": p0,
            "p1": p1,
            "t0": t,
            "t1": t + dt,
            "length": L,
            "dir": vec / L
        })
        t += dt
    return segs

def position_at_time(segs, t):
    if not segs:
        return None
    if t <= segs[0]['t0']:
        return segs[0]['p0'].copy()
    for s in segs:
        if s['t0'] <= t <= s['t1']:
            ratio = (t - s['t0']) / max(1e-9, (s['t1'] - s['t0']))
            return s['p0'] + s['dir'] * s['length'] * ratio
    return segs[-1]['p1'].copy()
