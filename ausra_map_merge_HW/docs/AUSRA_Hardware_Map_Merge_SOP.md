# AUSRA Hardware Map Merge — Standard Operating Procedure
**Document:** `AUSRA_Hardware_Map_Merge_SOP.md`  
**Strategy:** Physical World Reference Frame (Tape Measure)  
**Applies to:** All physical AUSRA robot deployments using `ausra_map_merge`  
**Prerequisite:** `ausra_map_merge` package validated and working in simulation

---

## 1. Core Concept

In simulation, Gazebo provides exact spawn coordinates automatically. On physical
hardware, each robot's SLAM node starts at local `(0, 0)` wherever it is powered
on. There are no global coordinates until we create them manually.

This procedure establishes a **physical world coordinate system** by designating
one fixed point in the room as the global origin `(0, 0)`. All robot starting
positions are then measured relative to that origin with a tape measure. Those
measurements become the `robot_offset_x` and `robot_offset_y` parameters in
`map_merge.launch.py`, which tell each `map_expansion_node` how to translate its
robot's local SLAM frame onto the shared global canvas.

```
Physical room                        map_expansion_node math
─────────────────────────────────    ──────────────────────────────────────────
[ORIGIN mark] ── 3.45 m ──► [R2]     robot_offset_x = 3.45
                                      global_origin_x = local_origin_x + 3.45
                                      offset_x = (global_origin_x - (-25.0)) / 0.05
```

> **The `init_pose_*` values in `map_merge_params.yaml` always remain `0.0`.
> Do not change them. The expansion node does all spatial work.**

---

## 2. Equipment Required

| Item | Purpose |
|---|---|
| Steel tape measure (≥ 10 m) | Primary measurement |
| Second tape measure or rigid ruler | Cross-check / verification |
| Coloured floor tape — 2 colours | Marking origin and axis |
| Laser level or long straightedge (≥ 2 m) | Yaw axis alignment |
| Permanent marker | Labelling floor marks |
| Deployment notebook + pen | Recording measurements |
| Two operators minimum | One measures, one records and cross-checks |

---

## 3. Phase 1 — Establish the Global Origin

### 3.1 Select the Origin Point

Choose a fixed, permanent, unambiguous physical feature as `(0, 0)`.

**Good choices**
- Inner corner where two walls meet at the floor
- Centre of a load-bearing column at floor level
- A permanent anchor bolt or floor stud

**Bad choices**
- Furniture — it moves
- Doorways — obstructed during operation
- Open floor with no nearby reference — impossible to recover next session

> **Rule:** The origin must be recoverable without measurement. If the robots are
> repositioned and setup must be repeated tomorrow, you must find the exact same
> point in under 30 seconds without guesswork.

### 3.2 Mark and Label the Origin

Place a cross of **Colour A** floor tape at the origin.
Label it clearly: `AUSRA ORIGIN (0, 0)`.

### 3.3 Define and Mark the Global X Axis

The X axis tape line is the physical embodiment of the coordinate system.
Every robot's forward direction must align to it.

1. Run a strip of **Colour B** floor tape from the origin along the longer wall,
   extending at least 5 m in the positive direction.
2. Label the far end: `+X DIRECTION`.

> **This is the most critical step in the procedure.**
> The expansion node applies only a translation offset — it has no rotation term.
> Any robot placed at an angle relative to this line will produce a
> rotated/sheared local map that cannot be corrected in software after the fact.

---

## 4. Phase 2 — Robot Placement and Measurement

Repeat steps 4.1 through 4.5 for **each robot**, one at a time.

### 4.1 Select and Mark the Robot Starting Position

Choose a starting position that is:
- On flat, level floor
- Clear of obstacles (≥ 0.5 m clearance in all directions)
- Within the 50 m × 50 m canvas (all positions within 25 m of origin)
- At least 0.5 m from the origin to avoid map-origin ambiguity

Place a small cross of floor tape at the selected position.
Label it: `ROBOT_N START` (e.g., `ROBOT_2 START`).

### 4.2 Measure robot_offset_x

```
Origin (0,0) ─────────────────────────────── +X tape line
                           │
                           │ (drop a perpendicular from robot position to X axis)
                           │
                       [Robot Start]

                ◄────────────────────────────►
                        robot_offset_x
```

1. Stretch the tape from the origin along the X axis tape line.
2. Drop a perpendicular from the robot start mark to the X axis tape.
3. Read the tape at the perpendicular intersection. This is `robot_offset_x` in metres.

### 4.3 Measure robot_offset_y

1. From the perpendicular intersection point on the X axis, measure straight
   across to the robot start mark.
2. That distance is `robot_offset_y` in metres.

**Sign convention**

| Direction | Sign |
|---|---|
| Along +X tape direction | Positive X |
| Away from base wall (into room) | Positive Y |
| Opposite to +X tape | Negative X |
| Behind base wall | Negative Y |

### 4.4 Cross-Check (Mandatory)

The second operator independently re-measures both values without looking at
the first operator's reading. If results differ by more than **2 cm** (= less
than one canvas cell at 0.05 m/cell), re-measure both until agreement. Record
the agreed values.

### 4.5 Yaw Alignment (CRITICAL — DO NOT SKIP)

> This step prevents the most common and hardest-to-diagnose hardware failure:
> the "rotated map" bug. A robot placed 5° off axis will produce a merged map
> where walls diverge as exploration progresses. There is no software fix.

1. Place the robot at its start position mark.
2. Lay the laser level or long straightedge along the X axis floor tape.
3. Sight down the robot's centreline from behind.
4. Physically rotate the robot until its centreline is visually parallel to the
   laser / tape line.
5. For confirmation: place a second straightedge along the robot's centreline.
   It must run parallel to the X axis tape over a 2 m length with no visible
   angular gap.
6. Record alignment quality: **Good** / **Acceptable** / **Poor**.
   If Poor: redo until at least Acceptable.

---

## 5. Phase 3 — Configuration Update

### 5.1 Update `map_merge.launch.py`

Edit the `ROBOT_SPAWN_POSES` dictionary at the top of the launch file:

```python
# map_merge.launch.py
# Update before every session where robot positions have changed.

ROBOT_SPAWN_POSES = {
    'ausra_1': {'x': 0.0,  'y': 0.0},   # Robot 1 IS at the physical origin
    'ausra_2': {'x': 3.45, 'y': 0.0},   # Measured: 3.45 m along X axis
    'ausra_3': {'x': 1.20, 'y': 2.80},  # Measured: 1.20 m X, 2.80 m Y
}
```

The launch file automatically passes these into each expansion node:

```python
for robot_name, spawn in ROBOT_SPAWN_POSES.items():
    expansion_node = Node(
        parameters=[{
            'robot_offset_x': spawn['x'],  # ← fed from tape measurement
            'robot_offset_y': spawn['y'],  # ← fed from tape measurement
        }],
    )
```

> **Recommended:** Place Robot 1 at the physical origin `(0, 0)`. This gives
> you a zero-offset anchor robot, which simplifies visual debugging in RViz.

### 5.2 Verify `map_merge_params.yaml`

Open the YAML. Confirm every robot's `init_pose_*` is `0.0`. Do not change
these to the measured tape values.

```yaml
# map_merge_params.yaml

/ausra_1/map_merge/init_pose_x:   0.0
/ausra_1/map_merge/init_pose_y:   0.0
/ausra_1/map_merge/init_pose_yaw: 0.0

/ausra_2/map_merge/init_pose_x:   0.0   # ← MUST be 0.0, NOT 3.45
/ausra_2/map_merge/init_pose_y:   0.0   # ← MUST be 0.0
/ausra_2/map_merge/init_pose_yaw: 0.0
```

> **The single most common misconfiguration:** setting `init_pose_x` to the
> spawn coordinate here AND having it in the expansion node. This applies
> the offset twice (double-shift), producing a badly misaligned merged map
> with no error message. The expansion node applies the offset; the merger
> must apply zero.

### 5.3 Record the Session Configuration

```
────────────────────────────────────────────────
AUSRA Deployment Log
────────────────────────────────────────────────
Date:
Environment / Room:
Origin landmark:

Robot    | robot_offset_x | robot_offset_y | Yaw quality
---------|----------------|----------------|-------------
ausra_1  |     0.00 m     |     0.00 m     |
ausra_2  |                |                |
ausra_3  |                |                |

Validated by (2 operators):  _____________  _____________
────────────────────────────────────────────────
```

---

## 6. Phase 4 — Launch Sequence

```
1. Power on all robots. Confirm all are at their marked positions with
   correct yaw alignment.

2. Launch all robot base stacks (SLAM, sensors, nav2) on each robot.

3. Confirm SLAM maps are publishing:
      ros2 topic hz /ausra_1/map
      ros2 topic hz /ausra_2/map
   Expected: ~1 Hz per robot.

4. Launch the map merge stack:
      ros2 launch ausra_map_merge map_merge.launch.py

   NOTE: The heartbeat timer in map_expansion_node publishes a valid
   all-Unknown canvas immediately at startup. The merger will not
   segfault even if SLAM is not yet active. However, launch SLAM first
   so the merged map is meaningful from the first frame.

5. Open RViz. Add the /map_merged topic.
   Proceed to Phase 5 validation before allowing robots to explore.
```

---

## 7. Phase 5 — Validation Before Exploration

### 7.1 Visual Sanity Check (30 seconds)

- Robot 1's environment appears near the canvas centre (assuming Robot 1 at origin).
- Robot 2's environment appears at approximately the correct offset distance and direction.
- No visible rotation or shear between the two maps in any shared-view area.

### 7.2 Known-Distance Validation (if environments overlap)

If the two robots can see a common wall or feature from their starting positions:

1. In the merged RViz view, use the **Measure** tool.
2. Click the same wall corner as detected by Robot 1 and then by Robot 2.
3. **Acceptable result:** distance < 5 cm (1 canvas cell).
4. **Fail result:** distance > 10 cm → recheck `robot_offset_x/y` values and yaw alignment.

### 7.3 Diagnosing a Rotated Map

If maps appear offset but walls run in non-parallel directions between robots:
- This is a **yaw alignment failure**.
- Power down, physically re-align the offending robot to the X axis tape.
- There is no parameter to fix this post-launch.

---

## 8. Troubleshooting Reference

| Symptom | Most Likely Cause | Fix |
|---|---|---|
| Robot 2's map stacked on Robot 1 at origin | `robot_offset_x/y` not updated; still `0.0` | Update `ROBOT_SPAWN_POSES` in launch file |
| Robot 2's map offset by **twice** the correct amount | `init_pose_*` in YAML set to spawn coordinates | Set all `init_pose_*` back to `0.0` |
| Maps translationally correct but rotationally wrong | Yaw alignment error at placement | Physically re-align robot, relaunch |
| Map merger segfaults | SLAM not publishing before merger launched | Heartbeat timer should prevent this; confirm SLAM is running first |
| `Canvas overflow` warnings in terminal log | Robot explored beyond 25 m from origin | Increase `canvas_width/height` OR reposition origin closer to exploration area |
| Merged map looks correct but drifts over time | slam_toolbox loop closure changed local origin | Canvas math compensates for this automatically; if drift persists, check `robot_offset_x/y` sign conventions |

---

## 9. Session Teardown and Persistence

1. Take a screenshot of `/map_merged` in RViz and save to the deployment log.
2. Note any alignment quality observations.
3. **Preserve all floor tape marks.** Permanent marks allow the next session to
   start in under 5 minutes with no re-measurement.
4. If robot positions will not change, no action is needed before next session.
5. If a new robot is added to the fleet, measure its start position using the
   existing origin marks and add it to `ROBOT_SPAWN_POSES` and the YAML.
