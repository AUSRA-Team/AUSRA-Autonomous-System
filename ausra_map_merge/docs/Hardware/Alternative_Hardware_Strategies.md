# AUSRA Hardware Map Merge — Alternative Alignment Strategies
**Document:** `Alternative_Hardware_Strategies.md`  
**Purpose:** Reference for future deployments, operational scaling, and advanced
alignment requirements beyond the baseline tape-measure SOP.

---

## Overview

The baseline deployment uses a tape-measure physical reference frame (see
`AUSRA_Hardware_Map_Merge_SOP.md`). That approach is fast and requires no
additional hardware, but it has limitations: it requires remeasurement when
robots move, is sensitive to yaw alignment errors, and does not scale gracefully
to large or complex environments.

The strategies documented here represent the natural upgrade path. Each one
integrates into the same `ausra_map_merge` architecture by providing a more
reliable or automated source of `robot_offset_x` and `robot_offset_y` values.

---

## Strategy 1 — Fixed Deployment Stations

### Concept

Fabricate or tape permanent, labelled floor pads at pre-calibrated positions.
Robots always start from their designated pads. Offsets are measured once during
commissioning and hardcoded permanently.

### How It Integrates

`ROBOT_SPAWN_POSES` in `map_merge.launch.py` is set once during commissioning
and never changes unless hardware is physically relocated. Daily operation
requires no measurement — operators place robots on marked pads and launch.

```python
# Set once at commissioning. Never changes unless pads are physically moved.
ROBOT_SPAWN_POSES = {
    'ausra_1': {'x': 0.0,  'y': 0.0},   # Pad A — origin pad
    'ausra_2': {'x': 3.45, 'y': 0.0},   # Pad B — measured at commissioning
    'ausra_3': {'x': 1.20, 'y': 2.80},  # Pad C — measured at commissioning
}
```

### Pad Design Recommendations

- Use adhesive rubber bumpers (4 points) to physically locate the robot wheels/tracks.
- Cut a directional notch or arrow into each pad pointing in the +X direction.
- Label each pad with the robot's name and offset values for field reference.
- For indoor permanent installations: use epoxy floor paint markers.

### Yaw Alignment

Build the yaw into the pad. A physical arrow or notch cut precisely parallel to
the X axis tape means correct yaw is achieved by correctly seating the robot —
no additional alignment step needed.

### Pros

- Zero remeasurement after commissioning
- Robot placement takes under 1 minute per robot
- Yaw alignment is enforced mechanically, not procedurally
- Highly repeatable across operators and sessions
- No additional software, hardware, or ROS nodes

### Cons

- Initial commissioning measurement is still required (same tape procedure as SOP)
- Requires physical infrastructure (pads) that cannot be moved ad-hoc
- Does not work for environments where robot start positions vary by mission

### When to Adopt

Immediately after the tape-measure strategy is validated and the operational
area is considered stable. This is the recommended upgrade from the baseline SOP
for any deployment running more than a few sessions per week.

---

## Strategy 2 — ArUco / Fiducial Marker Initialisation

### Concept

Place ArUco markers at known global positions in the environment. At startup,
each robot detects the nearest marker with its camera, computes its global pose
through the known marker position, and an initialisation node writes the correct
`robot_offset_x` and `robot_offset_y` before launching the expansion node.
Eliminates human measurement entirely after one-time marker placement.

### System Components

| Component | Role |
|---|---|
| ArUco markers (printed, laminated) | Physical global position anchors |
| Robot RGB camera (already on AUSRA hardware) | Marker detection |
| `ros2_aruco` or OpenCV-based detection node | Computes pose from marker |
| `ausra_pose_initialiser` node (new, ~100 lines) | Reads detected pose, writes `robot_offset_x/y` |

### Architecture

```
At startup:
  Camera detects ArUco marker (known global position)
        │
        ▼
  Detect marker pose → compute robot's global (x, y, yaw)
        │
        ▼
  ausra_pose_initialiser writes:
    robot_offset_x = detected_x
    robot_offset_y = detected_y
        │
        ▼
  map_expansion_node launches with correct offsets
  map_merge proceeds normally
```

### Marker Placement Rules

- Place one marker per major room entry zone, mounted at a fixed height.
- Record the global `(x, y)` of each marker relative to the physical origin.
- Store these in a `markers.yaml` config file:

```yaml
# config/markers.yaml
markers:
  - id: 0
    global_x: 0.0
    global_y: 0.0
    global_yaw: 0.0
  - id: 1
    global_x: 5.0
    global_y: 2.5
    global_yaw: 0.0
```

### Key Advantage Over Tape Measure

Yaw is detected automatically from the marker transform. The robot does not
need to be manually aligned — the software knows its orientation from the
detected marker pose.

### Pros

- Eliminates daily measurement entirely
- Provides yaw automatically (no alignment step)
- Works in dynamic environments where robot start positions vary
- Scales to any number of robots without additional configuration
- One-time setup cost only

### Cons

- Requires marker detection node development (~1–2 weeks)
- Camera must have clear line of sight to at least one marker at startup
- Marker placement must be calibrated to global frame (one-time but careful)
- Adds a dependency on camera calibration quality

### When to Adopt

For deployments in environments where robots do not always start from the same
position, or where operational tempo (many sessions per day) makes daily
measurement impractical. Also the correct solution for environments where a
human operator is not always available to physically align robots.

---

## Strategy 3 — Ultra-Wideband (UWB) Positioning

### Concept

Install UWB anchor beacons at known positions around the operating environment.
Each robot carries a UWB tag. At startup (or continuously), the tag resolves a
centimetre-accurate global position, which is read by a ROS node and passed
directly into `robot_offset_x` and `robot_offset_y`.

### System Components

| Component | Typical Product | Role |
|---|---|---|
| UWB anchors (×4 minimum) | Decawave DWM1001, Pozyx | Fixed global position references |
| UWB tag (×1 per robot) | Decawave DWM1001 | Robot position sensor |
| UWB ROS driver | `ros2-uwb` community packages | Publishes `geometry_msgs/PoseWithCovariance` |
| `ausra_pose_initialiser` node | Custom, ~50 lines | Reads UWB pose, writes offsets |

### Key Properties

- **Update rate:** 10–100 Hz continuous position updates
- **Accuracy:** 5–10 cm typical indoors
- **Works without line-of-sight** (unlike cameras / ArUco)
- **Works in darkness, dust, smoke** — purely RF-based

### Architecture

```
UWB tag on robot ──────► UWB anchors
                               │
                               ▼
               UWB driver publishes /ausra_X/uwb_pose
                               │
                               ▼
               ausra_pose_initialiser reads pose at startup
               writes robot_offset_x, robot_offset_y
                               │
                               ▼
               map_expansion_node launches with correct offsets
```

### Pros

- Centimetre-accurate, fully automatic
- Continuous updates — can correct drift if integrated with slam_toolbox
- No line-of-sight requirement
- Works in any lighting condition
- Highest robustness of all strategies

### Cons

- Upfront hardware cost (anchors + tags, ~$200–500 per robot)
- Anchor installation and calibration required
- Adds hardware dependency to each robot
- UWB ROS 2 driver maturity varies — may require custom integration

### When to Adopt

For permanent installations in complex environments (warehouses, multi-room
buildings), or when the operational requirement is fully autonomous robot
deployment with no human alignment procedure. Also appropriate when the
environment includes areas where camera-based detection (ArUco) is unreliable.

---

## Strategy 4 — Inter-Robot LiDAR Scan Matching at Boot

### Concept

Robot 1 starts at the global origin and maps briefly. Robot 2 is placed within
LiDAR scanning range of Robot 1. Before beginning autonomous operation, Robot 2
runs a one-shot ICP (Iterative Closest Point) scan match against Robot 1's
early map to compute its own offset relative to Robot 1. No external hardware
needed.

### Requirements

- Both robots within ~5 m of each other at startup
- Robot 1 must have mapped a sufficient area (>30 seconds of SLAM) before
  Robot 2 attempts the scan match
- A `scan_match_initialiser` node (new, ~200 lines) to perform the ICP operation

### Limitations

- Strong proximity constraint (robots must start near each other)
- Fails in featureless environments (long empty corridors, open warehouses)
- Requires careful timing during launch sequence
- ICP convergence is not guaranteed — may need multiple attempts

### When to Adopt

When no external hardware is available or installable, and robot start positions
are always within close range of each other. Primarily useful for research
settings, not recommended for production operational environments.

---

## Comparison Table

| Strategy | Setup Time | Daily Ops Time | Hardware Cost | Code Needed | Yaw Automated | Scales to N Robots |
|---|---|---|---|---|---|---|
| **Tape Measure (current SOP)** | 30 min | 15–30 min | None | None | No | Moderate |
| **Fixed Deployment Stations** | 2–4 hours | < 2 min | Low (tape/pads) | None | Partially | Good |
| **ArUco Fiducials** | 4–8 hours | 0 min | Low (print markers) | Medium (~100 lines) | Yes | Excellent |
| **UWB Positioning** | 1–2 days | 0 min | High ($200–500/robot) | Medium (~50 lines) | Yes | Excellent |
| **LiDAR Scan Matching** | 4–8 hours | 5 min | None | High (~200 lines) | Yes | Limited |

---

## Recommended Upgrade Path

```
Phase 1 (Now):       Tape Measure SOP
                     → Validate core pipeline on hardware

Phase 2 (Month 1):   Fixed Deployment Stations
                     → Eliminate daily measurement overhead
                     → Recommended for any deployment > 5 sessions/week

Phase 3 (Month 2+):  ArUco Markers
                     → Eliminate human alignment entirely
                     → Enable dynamic robot positioning by mission

Phase 4 (Advanced):  UWB Positioning
                     → Fully autonomous, no human procedure
                     → Required for 24/7 unattended operation
```

---

## Notes on Integration

All strategies listed here are drop-in replacements for the tape-measure step
only. The `ausra_map_merge` package architecture does not change:

- `map_expansion_node.cpp` — unchanged
- `map_merge_params.yaml` — `init_pose_*` stays at `0.0`
- `map_merge.launch.py` — `ROBOT_SPAWN_POSES` values are populated
  automatically (by detection node) instead of manually

The only new code required is the initialisation node that reads from the
position source (camera, UWB, scan match) and writes the values into
`ROBOT_SPAWN_POSES` before the expansion nodes launch.
