import json, sys
from pathlib import Path
from src.trajectory import segments_from_waypoints
from src.collision_check import simple_deconflict_pipeline
from src.visualize import make_animation

def load_scenario(path):
    with open(path, "r") as f:
        return json.load(f)

def build_segments(scen):
    speed = 5.0
    drones = []
    for d in scen["drones"]:
        segs = segments_from_waypoints(d["waypoints"], d.get("t_start", 0.0), speed)
        drones.append({"id": d["id"], "segs": segs})
    return drones

def run_all_vs_all(scen):
    drones = build_segments(scen)
    safety = 2.0

    results = []
    N = len(drones)

    for i in range(N):
        for j in range(i+1, N):
            A = drones[i]
            B = drones[j]

            conflicts = simple_deconflict_pipeline(
                A["segs"], B["segs"], safety_dist=safety
            )

            results.append({
                "pair": f"{A['id']} - {B['id']}",
                "conflicts": conflicts,
                "A": A,
                "B": B
            })

    return drones, results

def pretty_print(results, scenario_id):
    print(f"\n=== ALL-vs-ALL Collision Report ({scenario_id}) ===")
    any_conf = False
    for r in results:
        if r["conflicts"]:
            any_conf = True
            print(f"{r['pair']}: {len(r['conflicts'])} conflict(s)")
            for c in r["conflicts"]:
                print("   ", c)
        else:
            print(f"{r['pair']}: CLEAR")
    if not any_conf:
        print("\nNo collisions between any drone pairs.")

def prepare_animation_inputs(drones, results):
    cmap = ["C0","C1","C2","C3","C4","C5","C6","C7","C8"]
    all_trajs = [(d["id"], d["segs"], cmap[i % len(cmap)]) for i, d in enumerate(drones)]

    conflicts = []
    for r in results:
        pair = r["pair"]
        for c in r["conflicts"]:
            if "time" in c and "position" in c:
                conflicts.append({
                    "time": c["time"],
                    "position": c["position"],
                    "label": pair
                })

    conflicts.sort(key=lambda x: x["time"])
    return all_trajs, conflicts

def run(scenario_path):
    scen = load_scenario(scenario_path)
    scenario_id = scen.get("scenario_id", Path(scenario_path).stem)

    drones, results = run_all_vs_all(scen)
    pretty_print(results, scenario_id)

    all_trajs, conflicts = prepare_animation_inputs(drones, results)

    t_start = min(s["segs"][0]["t0"] for s in drones if s["segs"])
    t_end   = max(s["segs"][-1]["t1"] for s in drones if s["segs"])

    out_name = f"output_{scenario_id}_all.mp4"
    print("Generating animation:", out_name)
    make_animation(out_name, all_trajs, conflicts, t_start, t_end)
    print("Saved:", out_name)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python -m src.all_vs_all_check data/<scenario>.json")
        sys.exit(0)
    run(sys.argv[1])
