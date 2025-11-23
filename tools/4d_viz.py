import json
import numpy as np
import plotly.graph_objects as go
from pathlib import Path
from src.trajectory import segments_from_waypoints, position_at_time

def load_scenario(path):
    return json.loads(Path(path).read_text())

def sample_positions(drones, t_start, t_end, dt):
    times = np.arange(t_start, t_end + 1e-9, dt)
    positions = {}
    for d in drones:
        segs = d["segs"]
        arr = []
        for t in times:
            arr.append(position_at_time(segs, t))
        positions[d["id"]] = np.array(arr)
    return times, positions

def build_drones(scen):
    drones = []
    speed = scen.get("speed_mps",5.0)
    for d in scen["drones"]:
        segs = segments_from_waypoints(d["waypoints"], d.get("t_start",0.0), speed)
        drones.append({"id":d["id"], "segs":segs})
    return drones

def plot_interactive_4d(scen_path, dt=0.2):
    scen = load_scenario(scen_path)
    drones = build_drones(scen)

    t_start = min(s["t0"] for d in drones for s in d["segs"])
    t_end   = max(s["t1"] for d in drones for s in d["segs"])

    times, positions = sample_positions(drones, t_start, t_end, dt)

    fig = go.Figure()

    # Add static trajectories
    for i, d in enumerate(drones):
        pts = np.array([s["p0"] for s in d["segs"]] + [d["segs"][-1]["p1"]])
        fig.add_trace(go.Scatter3d(
            x=pts[:,0], y=pts[:,1], z=pts[:,2],
            mode="lines",
            line=dict(width=4),
            name=f"{d['id']}"
        ))

    # Add dynamic markers (one per drone)
    for i, d in enumerate(drones):
        fig.add_trace(go.Scatter3d(
            x=[positions[d["id"]][0,0]],
            y=[positions[d["id"]][0,1]],
            z=[positions[d["id"]][0,2]],
            mode="markers",
            marker=dict(size=6),
            name=f"{d['id']}_marker",
            showlegend=False
        ))

    frames = []
    for k, t in enumerate(times):
        frame_data = []
        for d in drones:
            p = positions[d["id"]][k]
            frame_data.append(dict(
                x=[p[0]], y=[p[1]], z=[p[2]]
            ))
        frames.append(go.Frame(data=frame_data, name=f"{t:.2f}"))

    fig.frames = frames

    fig.update_layout(
        title=f"4D Trajectory Viewer - {scen.get('scenario_id','scenario')}",
        scene=dict(
            xaxis_title='X', yaxis_title='Y', zaxis_title='Z',
            aspectmode='data'
        ),
        sliders=[{
            "steps":[{"args":[[f"{t:.2f}"], {"frame":{"duration":0, "redraw":True}}],
                      "label":f"{t:.2f}",
                      "method":"animate"} for t in times],
            "transition":{"duration":0},
            "x":0.1, "len":0.8
        }],
        updatemenus=[{
            "type":"buttons",
            "buttons":[
                {"label":"Play", "method":"animate",
                 "args":[None, {"frame":{"duration":50,"redraw":True},
                                "fromcurrent":True}]},
                {"label":"Pause", "method":"animate",
                 "args":[[None], {"frame":{"duration":0,"redraw":False},
                                  "mode":"immediate"}]}
            ]
        }]
    )

    out_html = Path(scen_path).with_suffix(".html")
    fig.write_html(out_html)
    print("Wrote interactive 4D viewer to:", out_html)


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python tools/plotly_4d_viewer.py data/scenario.json")
    else:
        plot_interactive_4d(sys.argv[1])
