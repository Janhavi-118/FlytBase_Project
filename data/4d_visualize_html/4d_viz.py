import json
import numpy as np
import plotly.graph_objects as go
from pathlib import Path
import sys
import os

# Ensure project root is added for imports
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from src.all_check import run_all_vs_all, prepare_animation_inputs, load_scenario
from lib.trajectory import position_at_time
from plotly.colors import qualitative


def plot_interactive_4d_with_all_check(scen_path, dt=0.2):
    scen = load_scenario(scen_path)
    drones, results = run_all_vs_all(scen)
    all_trajs, conflicts = prepare_animation_inputs(drones, results)

    print(f"Detected {len(conflicts)} collisions:")
    for c in conflicts:
        print(f"Time: {c['time']}, Position: {c['position']}, Label: {c['label']}")

    colors = qualitative.Plotly
    t_start = min(s["segs"][0]["t0"] for s in drones if s["segs"])
    t_end = max(s["segs"][-1]["t1"] for s in drones if s["segs"])

    print(f"Animation time span: {t_start} to {t_end}")
    if conflicts:
        print(f"Collision times min/max: {min(c['time'] for c in conflicts)} to {max(c['time'] for c in conflicts)}")

    times = np.arange(t_start, t_end + dt / 2, dt)
    positions = {}
    for d in drones:
        arr = []
        segs = d["segs"]
        for t in times:
            arr.append(position_at_time(segs, t))
        positions[d["id"]] = np.array(arr)

    fig = go.Figure()

    # Add static drone paths and waypoints
    for i, d in enumerate(drones):
        color = colors[i % len(colors)]
        waypoints = np.array([seg["p0"] for seg in d["segs"]] + [d["segs"][-1]["p1"]])

        fig.add_trace(go.Scatter3d(
            x=waypoints[:, 0], y=waypoints[:, 1], z=waypoints[:, 2],
            mode="lines+markers",
            line=dict(width=2, color=color),
            marker=dict(size=4, color=color),
            name=f"{d['id']} path/waypoints"
        ))

        fig.add_trace(go.Scatter3d(
            x=[positions[d["id"]][0, 0]],
            y=[positions[d["id"]][0, 1]],
            z=[positions[d["id"]][0, 2]],
            mode="markers",
            marker=dict(size=9, color=color),
            name=f"{d['id']} moving",
            showlegend=False
        ))

    # Add collision markers as static traces initially with zero opacity
    num_drone_traces = 2 * len(drones)
    for collision in conflicts:
        pos = collision["position"]
        fig.add_trace(go.Scatter3d(
            x=[pos[0]], y=[pos[1]], z=[pos[2]],
            mode="markers",
            marker=dict(size=30, color="yellow", symbol="diamond", opacity=0),
            name=f"Collision {collision['label']}",
            hoverinfo="text",
            hovertext=f"Collision at {collision['time']:.3f}s",
            showlegend=True,
        ))

    frames = []
    num_collisions = len(conflicts)

    for frame_index, t in enumerate(times):
        frame_data = []

        # Drone moving markers only
        for i, d in enumerate(drones):
            p = positions[d["id"]][frame_index]
            color = colors[i % len(colors)]
            frame_data.append(go.Scatter3d(
                x=[p[0]], y=[p[1]], z=[p[2]],
                mode="markers",
                marker=dict(size=9, color=color),
                showlegend=False,
            ))

        # Set opacity of collision markers based on collision time relative to current frame time
        for ci, c in enumerate(conflicts):
            opacity = 1.0 if t >= c["time"] else 0.0
            pos = c["position"]
            frame_data.append(go.Scatter3d(
                x=[pos[0]], y=[pos[1]], z=[pos[2]],
                mode="markers",
                marker=dict(size=15, color="black", symbol="diamond", opacity=opacity),
                showlegend=False,
                hoverinfo="text",
                hovertext=f"Collision {c['label']} at {c['time']:.3f}s"
            ))

        # Trace indices to update — moving drone markers and collision markers after drone traces
        traces_to_update = [i * 2 + 1 for i in range(len(drones))] + \
                           list(range(num_drone_traces, num_drone_traces + num_collisions))

        frames.append(go.Frame(data=frame_data, name=f"{t:.2f}", traces=traces_to_update))

    fig.frames = frames

    fig.update_layout(
        title=f"4D Trajectory Viewer with Collisions - {Path(scen_path).stem}",
        scene=dict(
            xaxis_title="X", yaxis_title="Y", zaxis_title="Z",
            aspectmode="data",
            dragmode="orbit",
            camera=dict(eye=dict(x=1.2, y=1.2, z=1.2))  # Setting initial camera position
        ),
        sliders=[{
            "steps": [{"args": [[f"{time:.2f}"], {"frame": {"duration": 0, "redraw": True}}],
                       "label": f"{time:.2f}", "method": "animate"} for time in times],
            "transition": {"duration": 0},
            "x": 0.1, "len": 0.8
        }],
        updatemenus=[{
            "type": "buttons",
            "buttons": [
                {"label": "Play", "method": "animate", "args": [None, {"frame": {"duration": 50, "redraw": True}, "fromcurrent": True}]},
                {"label": "Pause", "method": "animate", "args": [[None], {"frame": {"duration": 0, "redraw": False}, "mode": "immediate"}]}
            ]
        }]
    )

    out_dir = Path(project_root) / "data/4d_visualize_html/4d_random_vis"
    out_dir.mkdir(exist_ok=True)

    out_html = out_dir / f"{Path(scen_path).stem}_4d_viz.html"
    fig.write_html(out_html)
    print("Interactive 4D viewer with collisions saved to:", out_html)


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python -m data.4d_visualize_html.4d_viz path/to/scenario.json")
        sys.exit(0)
    plot_interactive_4d_with_all_check(sys.argv[1])
