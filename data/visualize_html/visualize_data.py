#!/usr/bin/env python3
"""
tools/visualize_dataset.py

Simple interactive 3D viewer for scenario JSON files.
Usage:
    python tools/visualize_dataset.py data/scenario_multi_weird_02.json

This script:
 - reads the scenario JSON (list of drones with waypoints)
 - plots each drone trajectory as a 3D polyline
 - shows waypoints as scatter points and labels them
 - opens an interactive matplotlib window (rotate/zoom with mouse)

Optional: set --plotly to write an interactive HTML file (good if you cannot open GUI).
"""

import json
import argparse
from pathlib import Path
import numpy as np

# Matplotlib interactive viewer
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (needed for projection)

# Optional Plotly exporter
try:
    import plotly.graph_objects as go
    _HAS_PLOTLY = True
except Exception:
    _HAS_PLOTLY = False

ASSIGNMENT_PDF = "/mnt/data/FlytBase Robotics Assignment 2025.pdf"  # local reference path

def load_scenario(path):
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"scenario file not found: {path}")
    return json.loads(p.read_text())

def plot_matplotlib(scen, title=None, show_legend=True, waypoint_markersize=30):
    drones = scen["drones"]
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection="3d")
    cmap = plt.get_cmap("tab10")

    all_pts = []
    for i, d in enumerate(drones):
        waypoints = np.array(d["waypoints"], dtype=float)
        if waypoints.size == 0:
            continue
        all_pts.append(waypoints)
        color = cmap(i % 10)
        # plot polyline
        ax.plot(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], '-', color=color, label=d.get("id", f"drone_{i}"))
        # plot waypoints
        ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], color=color, s=waypoint_markersize)
        # annotate first and last points
        ax.text(waypoints[0,0], waypoints[0,1], waypoints[0,2], f"{d.get('id','')}:start", fontsize=8)
        ax.text(waypoints[-1,0], waypoints[-1,1], waypoints[-1,2], f"{d.get('id','')}:end", fontsize=8)

    # Auto scale axes to include all pts with padding
    if all_pts:
        pts = np.vstack(all_pts)
        xpad = (pts[:,0].max() - pts[:,0].min()) * 0.1 + 1.0
        ypad = (pts[:,1].max() - pts[:,1].min()) * 0.1 + 1.0
        zpad = (pts[:,2].max() - pts[:,2].min()) * 0.1 + 1.0
        ax.set_xlim(pts[:,0].min() - xpad, pts[:,0].max() + xpad)
        ax.set_ylim(pts[:,1].min() - ypad, pts[:,1].max() + ypad)
        ax.set_zlim(pts[:,2].min() - zpad, pts[:,2].max() + zpad)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    t = title if title else scen.get("scenario_id", "scenario")
    ax.set_title(f"{t} — interactive view\n(Assignment: {ASSIGNMENT_PDF})")
    if show_legend:
        ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1.0))
    plt.tight_layout()
    plt.show()


def export_plotly_html(scen, out_html):
    if not _HAS_PLOTLY:
        raise RuntimeError("plotly is not installed. Install with `pip install plotly` to use HTML export.")
    drones = scen["drones"]
    fig = go.Figure()
    colors = ["blue","orange","green","red","purple","brown","pink","gray","olive","cyan"]
    for i, d in enumerate(drones):
        w = np.array(d["waypoints"], dtype=float)
        if w.size == 0:
            continue
        color = colors[i % len(colors)]
        # line
        fig.add_trace(go.Scatter3d(x=w[:,0], y=w[:,1], z=w[:,2],
                                   mode='lines+markers',
                                   marker=dict(size=3),
                                   line=dict(width=3, color=color),
                                   name=d.get("id", f"drone_{i}")))
        # start/end labels
        fig.add_trace(go.Scatter3d(x=[w[0,0]], y=[w[0,1]], z=[w[0,2]],
                                   mode='markers+text', marker=dict(size=4),
                                   text=[f"{d.get('id','')}:start"], textposition="top center",
                                   showlegend=False))
        fig.add_trace(go.Scatter3d(x=[w[-1,0]], y=[w[-1,1]], z=[w[-1,2]],
                                   mode='markers+text', marker=dict(size=4),
                                   text=[f"{d.get('id','')}:end"], textposition="top center",
                                   showlegend=False))
    fig.update_layout(scene=dict(xaxis_title='X (m)', yaxis_title='Y (m)', zaxis_title='Z (m)'),
                      title=f"{scen.get('scenario_id','scenario')} (Assignment: {ASSIGNMENT_PDF})",
                      margin=dict(l=0, r=0, b=0, t=50))
    fig.write_html(out_html)
    print("Wrote HTML interactive view to:", out_html)


def main():
    p = argparse.ArgumentParser(description="Visualize scenario dataset (3D)")
    p.add_argument("scenario", help="path to scenario JSON (in data/)")
    p.add_argument("--plotly-out", help="write interactive HTML instead of opening GUI", default=None)
    args = p.parse_args()

    scen = load_scenario(args.scenario)

    if args.plotly_out:
        export_plotly_html(scen, args.plotly_out)
    else:
        plot_matplotlib(scen, title=scen.get("scenario_id", None))


if __name__ == "__main__":
    main()
