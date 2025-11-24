import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from pathlib import Path

def static_plot(ax, segs, label, color):
    pts = [segs[0]['p0']] + [s['p1'] for s in segs]
    pts = np.array(pts)
    ax.plot(pts[:,0], pts[:,1], pts[:,2], '-', color=color, label=label)
    ax.scatter(pts[:,0], pts[:,1], pts[:,2], color=color, s=20)

def make_animation(filename, all_trajs, conflicts, t_start, t_end, dt=0.1, fps=15):
    times = np.arange(t_start, t_end + 1e-9, dt)

    fig = plt.figure(figsize=(9, 6))
    ax = fig.add_subplot(111, projection='3d')

    # static trajectories
    for (id_, segs, color) in all_trajs:
        static_plot(ax, segs, id_, color)

    # set axis limits
    pts = []
    for (_, segs, _) in all_trajs:
        for s in segs:
            pts.append(s['p0'])
            pts.append(s['p1'])
    pts = np.vstack(pts)
    pad = max(10.0, float(np.ptp(pts[:,0])) * 0.1)
    ax.set_xlim(np.min(pts[:,0]) - pad, np.max(pts[:,0]) + pad)
    ax.set_ylim(np.min(pts[:,1]) - pad, np.max(pts[:,1]) + pad)
    ax.set_zlim(np.min(pts[:,2]) - 5, np.max(pts[:,2]) + 5)

    # moving drone markers
    markers = {}
    for (id_, segs, color) in all_trajs:
        m, = ax.plot([], [], [], 'o', color=color, markersize=6)
        markers[id_] = m

    # conflict markers
    conflict_artists = []
    for c in conflicts:
        cm, = ax.plot([], [], [], 'o', color='red', markersize=9, alpha=0.9)
        conflict_artists.append((c, cm))

    def update(frame):
        t = times[frame]

        # update drone positions
        for (id_, segs, color) in all_trajs:
            pos = None
            for s in segs:
                if s['t0'] <= t <= s['t1']:
                    ratio = (t - s['t0']) / (s['t1'] - s['t0']) if s['t1'] > s['t0'] else 0
                    pos = s['p0'] + s['dir'] * s['length'] * ratio
                    break
            if pos is None:
                pos = segs[-1]['p1']

            markers[id_].set_data([pos[0]], [pos[1]])
            markers[id_].set_3d_properties([pos[2]])

        # update conflict markers
        for (c, art) in conflict_artists:
            tconf = c['time']
            if t >= tconf - dt/2:
                pos = np.array(c['position'], dtype=float)
                art.set_data([pos[0]], [pos[1]])
                art.set_3d_properties([pos[2]])
            else:
                art.set_data([], [])
                art.set_3d_properties([])
                
        artists = list(markers.values())
        for (_, art) in conflict_artists:
            artists.append(art)
        return artists

    anim = animation.FuncAnimation(fig, update, frames=len(times), interval=1000/fps)

    try:
        anim.save(filename, fps=fps, dpi=120)
    except Exception as e:
        gif = Path(filename).with_suffix(".gif")
        print("MP4 writer missing, saving GIF:", gif)
        anim.save(str(gif), writer="pillow", fps=fps, dpi=120)

    plt.close(fig)
