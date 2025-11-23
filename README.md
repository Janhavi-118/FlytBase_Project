# UAV Strategic Deconfliction in Shared Airspace

This project implements a modular system for detecting potential conflicts between multiple UAVs operating in a shared airspace. The focus is on **strategic deconfliction**: identifying collision risks early using only planned trajectories, without relying on onboard reactive avoidance.

The system supports both simple and highly irregular trajectories, including straight segments, squiggles, spirals, semicircles, and arbitrary polyline paths.

## Key Features

The pipeline integrates:
- **Geometric proximity filtering** for efficient candidate selection
- **Analytic closest-approach computation** for exact conflict detection
- **Multi-pair conflict evaluation** in both primary-vs-others and all-vs-all modes
- **3D visualization** with persistent conflict markers and time-based animation
- **Modular architecture** that remains transparent, deterministic, and interpretable

## Motivation and Approach

Conflict detection in multi-UAV environments often suffers from two extremes:

1. **Naive sampling**: misses conflicts unless timestep is unrealistically small
2. **Heavyweight simulation**: unnecessary when only strategic (pre-flight) trajectories are available

This project positions itself between the two:

- Trajectories are treated as **piecewise-linear segments**
- A fast **geometric filter** identifies segment pairs that may come within a safety radius
- A mathematically exact **analytic closest-approach routine** computes minimum separation between two moving UAVs over their respective segment times
- A scenario can be evaluated either **relative to a primary drone**, or in **all-vs-all mode** for full swarm deconfliction

This architecture remains **transparent, deterministic, and interpretable**—important qualities for safety-critical systems.

## Project Structure

```
.
├── src/
│   ├── trajectory.py          # Trajectory segmentation + position interpolation
│   ├── collision_check.py     # Geometric filter + analytic closest-approach logic
│   ├── geofilter.py           # Geometric utility functions
│   ├── visualize.py           # 3D visualization with persistent conflict markers
│   ├── main.py                # Primary-vs-others deconfliction pipeline
│   └── all_check.py           # Full swarm pairwise conflict evaluation
│
├── data/
│   ├── logic_checks
|   |   └──  ... logic test cases
│   ├── random_scenarios
|   |   └──  ... random test cases
│   └── visualize_html
|       └──  ... html visualizations of datasets
│
├── logic_checks_primary_drone/ # Video results of logic checks
├── results_all_drones/         # Video results of random data
├── requirements.txt
├── Dockerfile
└── README.md
```

## Scenario Format

Each scenario JSON file defines:

```json
{
  "scenario_id": "example",
  "speed_mps": 4.0,
  "safety_distance_m": 2.0,
  "drones": [
    {
      "id": "drone1",
      "t_start": 0.0,
      "waypoints": [[x, y, z], [x2, y2, z2], ...]
    },
    {
      "id": "drone2",
      "t_start": 0.0,
      "waypoints": [[x, y, z], [x2, y2, z2], ...]
    }
  ]
}
```

**Key details:**
- Waypoints implicitly form straight segments with constant speed
- `t_start` defines when the UAV begins its first segment
- No dynamics model is required—this is pre-flight strategic evaluation
- All distances are in meters, speeds in m/s, times in seconds

## How Conflict Detection Works

### Step 1: Segment Construction

Each UAV's waypoints are converted into timed segments:
- Segment: p₀ → p₁
- Duration: distance / speed

### Step 2: Geometric Proximity Filter

Every pair of segments is checked:
- If their bounding boxes + safety radius do **not** overlap → skip
- Else → candidate for analytic evaluation

This avoids O(N² × segment²) overhead.

### Step 3: Analytic Closest-Approach Evaluation

For each candidate pair:
1. Extract relative position and velocity
2. Compute the time of minimum separation exactly
3. If separation < safety_distance → **conflict recorded**

### Step 4: Aggregation & Reporting

Conflicts are reported with:

```json
{
  "time": t_collision,
  "position": [x, y, z],
  "distance": d_min,
  "pair": "drone_id_A - drone_id_B"
}
```

## Usage

### Prerequisites

```bash
python >= 3.9
pip install numpy matplotlib
```

### Running Without Docker

#### Primary vs Others Mode

```bash
python -m src.main data/random_scenarios/<scenario>.json
```

This evaluates the first drone (primary) against all others.

#### All-vs-All Mode (Full Swarm)

```bash
python -m src.all_check data/random_scenarios/<scenario>.json
```

This evaluates all drones pairwise. **Output includes:**
- Printed list of all conflicts with details
- 3D animation file (`output_<scenario>_all.mp4` or GIF fallback)

#### Interactive Dataset Viewer

```bash
python data/visualize_html/visualize_data.py data/random_scenarios/<scenario>.json
```

Shows raw trajectories (no collision detection). Allows rotation, zoom, and inspection of path complexity.

### Running with Docker

#### Build the Docker Image

```bash
docker build -t uav-deconflict:latest .
```

#### Run the Container

```bash
docker run --rm -it \
    -v "D:/FlytBase_Project:/home/appuser/workspace" \
    -w /home/appuser/workspace \
    uav-deconflict:latest
```

Replace `D:/FlytBase_Project` with your project directory path.

#### Inside the Container

1. Run `main.py` to check for conflicts of only primary drone with other drones:

```bash
python -m src.main data/random_scenarios/<scenario>.json
```

2. Run `all_check.py` to check for conflicts between all drones:

```bash
python -m src.all_check data/random_scenarios/<scenario>.json
```

3. To test the dataset visualizer:

```bash
python data/visualize_html/visualize_data.py data/random_scenarios/<scenario>.json
```

## Visualization Tools

### A. Strategic Visualization (Animation)

- Shows UAV trajectories
- Shows moving markers for each drone
- Shows persistent conflict markers from collision time onward
- Annotates each marker with pair ID and time
- Generated automatically via `src.all_check` or `src.main`

### B. Static 3D Dataset Viewer

Useful for inspecting raw trajectories without animation or collision logic:

```bash
python data/visualize_html/visualize_data.py data/random_scenarios/<scenario>.json
```

Interactive matplotlib plot—rotate, zoom, and inspect path complexity freely.

## Creating New Scenarios

A valid scenario JSON requires:
- List of drones (each with `id`, `t_start`, `waypoints`)
- Shared `speed_mps` and `safety_distance_m` thresholds
- Unique `scenario_id`

Example:

```json
{
  "scenario_id": "my_scenario",
  "speed_mps": 5.0,
  "safety_distance_m": 3.0,
  "drones": [
    {
      "id": "alpha",
      "t_start": 0.0,
      "waypoints": [[0, 0, 10], [10, 10, 10], [20, 0, 10]]
    },
    {
      "id": "bravo",
      "t_start": 2.0,
      "waypoints": [[5, 5, 10], [15, 5, 10]]
    }
  ]
}
```

## Output Interpretation

### Console Output Example

```
RESULT my_scenario: CONFLICTS FOUND
Pair: alpha - bravo
  Time: 4.5s, Position: [12.5, 7.5, 10.0], Distance: 1.2m
```

**Interpretation:**
- Drones "alpha" and "bravo" will violate the 3.0m safety distance at t=4.5s
- Closest approach point is [12.5, 7.5, 10.0]
- Minimum separation distance is 1.2m (less than 3.0m threshold)

### No Conflicts

```
RESULT my_scenario: CLEAR
No collisions between any drone pairs.
```

All drones maintain safe separation throughout their missions.

## Troubleshooting

### Import Errors

Ensure the `src/` directory structure is preserved and you're running from the project root:

```bash
cd /path/to/FlytBase_Project
python -m src.main data/random_scenarios/scenario.json
```

### Missing Dependencies

```bash
pip install --upgrade numpy matplotlib
```

### Docker Permission Issues (WSL2/Windows)

If you encounter permission errors:

```bash
docker exec -it -u root <container_id> bash
apt-get update
apt-get install -y git python3-pip
```

### Animation Not Generating

Ensure matplotlib backend supports animation. On headless systems, use:

```bash
export MPLBACKEND=Agg
python -m src.all_check data/random_scenarios/<scenario>.json
```

## Design Rationale

- **Modularity**: Each component (trajectory, collision, visualization) is independent and testable
- **Determinism**: No randomness; results are reproducible across runs
- **Interpretability**: Conflicts are reported with exact times, positions, and distances
- **Scalability**: Geometric filtering ensures O(n) behavior in non-adversarial scenarios
- **Safety**: Exact analytic computation avoids sampling artifacts

