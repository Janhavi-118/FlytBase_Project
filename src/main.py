
import json
from src.trajectory import segments_from_waypoints
from src.collision_check import simple_deconflict_pipeline
from src.visualize import make_animation

def load_scenario(path):
    with open(path,'r') as f:
        return json.load(f)

def run_scenario(path, dt=0.1):
    scen = load_scenario(path)
    speed = scen.get('speed_mps', 5.0)
    safety = scen.get('safety_distance_m', 5.0)
    drones = []
    for d in scen['drones']:
        segs = segments_from_waypoints(d['waypoints'], d.get('t_start',0.0), speed)
        drones.append({'id': d['id'], 'segs': segs})
    primary = next((d for d in drones if d['id']=='primary'), drones[0])
    others = [d for d in drones if d['id'] != primary['id']]
    all_conflicts = []
    for od in others:
        conflicts = simple_deconflict_pipeline(primary['segs'], od['segs'], safety_dist=safety, dt=dt)
        for c in conflicts:
            c['other'] = od['id']
        all_conflicts.extend(conflicts)
    if not all_conflicts:
        print(f"RESULT ({scen.get('scenario_id','unknown')}): CLEAR")
    else:
        print(f"RESULT ({scen.get('scenario_id','unknown')}): CONFLICTS FOUND")
        for c in all_conflicts:
            print(c)
    # animation
    cmap = ['C0','C1','C2','C3','C4']
    all_trajs = [(d['id'], d['segs'], cmap[i % len(cmap)]) for i,d in enumerate(drones)]
    t_start = min(s['t0'] for d in drones for s in d['segs'])
    t_end = max(s['t1'] for d in drones for s in d['segs'])
    out_name = f"output_{scen['scenario_id']}.mp4"
    print('Rendering:', out_name)
    make_animation(out_name, all_trajs, all_conflicts, t_start, t_end, dt=dt)
if __name__=='__main__':
    import sys
    if len(sys.argv)<2:
        print('Usage: python -m src.main data/scenario_headon.json')
    else:
        run_scenario(sys.argv[1])
