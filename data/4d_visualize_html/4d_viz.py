import json
import numpy as np
import plotly.graph_objects as go
from pathlib import Path
from src.trajectory import segments_from_waypoints, position_at_time
from plotly.colors import qualitative

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
    speed = scen.get("speed_mps", 5.0)
    for d in scen["drones"]:
        segs = segments_from_waypoints(d["waypoints"], d.get("t_start", 0.0), speed)
        drones.append({"id": d["id"], "segs": segs, "waypoints": np.array(d["waypoints"])})
    return drones

def plot_interactive_4d(scen_path, dt=0.2):
    scen = load_scenario(scen_path)
    drones = build_drones(scen)
    
    print("Loaded drones:", [d['id'] for d in drones])
    
    colors = qualitative.Plotly
    
    t_start = min(s["t0"] for d in drones for s in d["segs"])
    t_end = max(s["t1"] for d in drones for s in d["segs"])
    
    times, positions = sample_positions(drones, t_start, t_end, dt)
    
    fig = go.Figure()
    
    # Plot static trajectories and waypoints
    for i, d in enumerate(drones):
        color = colors[i % len(colors)]
        waypoints = d["waypoints"]
        
        # Trajectory line
        fig.add_trace(go.Scatter3d(
            x=waypoints[:, 0], y=waypoints[:, 1], z=waypoints[:, 2],
            mode="lines",
            line=dict(width=2, color=color),
            name=f"{d['id']} path",
            visible=True
        ))
        
        # Waypoint markers
        fig.add_trace(go.Scatter3d(
            x=waypoints[:, 0], y=waypoints[:, 1], z=waypoints[:, 2],
            mode="markers",
            marker=dict(size=4, color=color),
            name=f"{d['id']} waypoints",
            visible=True
        ))
        
        # Initial moving marker for animation
        fig.add_trace(go.Scatter3d(
            x=[positions[d["id"]][0, 0]],
            y=[positions[d["id"]][0, 1]],
            z=[positions[d["id"]][0, 2]],
            mode="markers",
            marker=dict(size=9, color=color),
            name=f"{d['id']} moving",
            showlegend=False,
            visible=True
        ))
    
    frames = []
    for k, t in enumerate(times):
        frame_data = []
        for i, d in enumerate(drones):
            color = colors[i % len(colors)]
            p = positions[d["id"]][k]
            frame_data.append(go.Scatter3d(
                x=[p[0]], y=[p[1]], z=[p[2]],
                mode="markers",
                marker=dict(size=9, color=color),
                showlegend=False
            ))
        # Specify which traces are updated by each frame: indices of moving markers (every 3rd trace starting at 2)
        traces_to_update = [i*3 + 2 for i in range(len(drones))]
        frames.append(go.Frame(data=frame_data, name=f"{t:.2f}", traces=traces_to_update))
    
    fig.frames = frames
    
    fig.update_layout(
        title=f"4D Trajectory Viewer - {scen.get('scenario_id', 'scenario')}",
        scene=dict(
            xaxis_title='X', yaxis_title='Y', zaxis_title='Z',
            aspectmode='data',
            dragmode='orbit'
        ),
        sliders=[{
            "steps": [{"args": [[f"{t:.2f}"], {"frame": {"duration": 0, "redraw": True}}],
                       "label": f"{t:.2f}",
                       "method": "animate"} for t in times],
            "transition": {"duration": 0},
            "x": 0.1, "len": 0.8
        }],
        updatemenus=[{
            "type": "buttons",
            "buttons": [
                {"label": "Play", "method": "animate",
                 "args": [None, {"frame": {"duration": 50, "redraw": True},
                                 "fromcurrent": True}]},
                {"label": "Pause", "method": "animate",
                 "args": [[None], {"frame": {"duration": 0, "redraw": False},
                                   "mode": "immediate"}]}
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
