# AUSRA Hardware Map Merge — Missing Strategy Evaluation
# AMCL Localization Against a Prior Reference Map
**Document:** `Missing_Hardware_Strategy_Evaluation.md`  
**Strategy:** Adaptive Monte Carlo Localization (AMCL) with a pre-built static map  
**Identified by:** Peer review of existing hardware deployment roadmap  
**Integrates with:** `ausra_map_merge` — identical `map_expansion_node` pipeline

---

## 1. Executive Summary

This document evaluates a deployment strategy that was absent from the original
`Alternative_Hardware_Strategies.md` roadmap: using **AMCL (Adaptive Monte Carlo
Localization)** with a pre-built reference map to automatically determine each
robot's global starting pose at boot time.

This approach requires **zero additional hardware** beyond what the AUSRA robots
already carry (2D LiDAR) and leverages `nav2_amcl`, a package the system already
depends on for autonomous navigation. It provides automated `(x, y, yaw)`
localization within a known environment, feeding directly into the
`map_expansion_node`'s `robot_offset_x` and `robot_offset_y` parameters.

---

## 2. Core Concept

### The Problem (Recap)

On physical hardware, each robot's `slam_toolbox` initializes at local `(0, 0)`
wherever it is powered on. The `map_expansion_node` needs `robot_offset_x` and
`robot_offset_y` to translate this local frame onto the shared global canvas.
All current strategies focus on *measuring or detecting* this offset externally.

### The AMCL Approach

Instead of measuring the robot's position relative to a physical origin, we let
the robot **figure out where it is** by matching its live LiDAR scan against a
pre-existing map of the environment.

```
┌─────────────────────────────────────────────────────────────────────────┐
│  ONE-TIME SETUP (done once per environment)                            │
│                                                                        │
│  1. Drive one robot through the entire operational area.               │
│  2. Save the resulting SLAM map as a static .pgm + .yaml file.        │
│     (ros2 run nav2_map_server map_saver_cli -f reference_map)         │
│  3. This map defines the global coordinate system permanently.        │
│     Its origin IS the global origin — no tape, no markers.            │
└─────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────┐
│  EVERY DEPLOYMENT (fully automatic per robot)                          │
│                                                                        │
│  1. Robot boots. LiDAR begins scanning.                                │
│  2. nav2_amcl loads the reference map and begins particle filtering.   │
│  3. Within 5–15 seconds, particles converge on the robot's true       │
│     global (x, y, yaw) within the reference map frame.                │
│  4. ausra_amcl_initialiser node reads the converged pose.             │
│  5. Writes robot_offset_x, robot_offset_y (and optionally yaw)       │
│     into the map_expansion_node parameters.                            │
│  6. map_expansion_node launches with correct global alignment.        │
│  7. map_merge proceeds normally — no human intervention needed.       │
└─────────────────────────────────────────────────────────────────────────┘
```

### Why This Works

AMCL is a probabilistic localization algorithm that maintains a cloud of
"particles," each representing a hypothesis about the robot's pose. As the
robot's LiDAR scan is compared against the reference map, unlikely hypotheses
are discarded and likely ones are reinforced. After sufficient scan matches
(typically 5–15 seconds of stationary scanning or brief rotation), the particle
cloud converges to a tight cluster around the true pose.

The key insight: **the reference map's coordinate frame IS the global frame.**
Whatever origin `map_saver` used when saving the map becomes the permanent
global origin for all future deployments. No physical origin markers, no tape
measures, no fiducial markers — the map itself is the ground truth.

---

## 3. Integration with `map_expansion_node`

### Offset Injection — Identical to All Other Strategies

The AMCL-derived pose integrates identically to how the tape measure, ArUco, or
UWB strategies feed into the system. Only the *source* of the offset values
changes:

```python
# map_merge.launch.py — no structural changes needed
# ROBOT_SPAWN_POSES is populated dynamically by the AMCL initialiser
# instead of being hardcoded or read from tape measurements.

ROBOT_SPAWN_POSES = {
    'ausra_1': {'x': amcl_x_1, 'y': amcl_y_1},  # ← from AMCL convergence
    'ausra_2': {'x': amcl_x_2, 'y': amcl_y_2},  # ← from AMCL convergence
}
```

### Spatial Math — Unchanged

The `map_expansion_node` receives `robot_offset_x` and `robot_offset_y` exactly
as before. The canvas math remains:

```cpp
// map_expansion_node.cpp — NO CHANGES REQUIRED

// Step 1: local SLAM origin → global coordinates
const double global_origin_x = local_origin_x + robot_offset_x_;  // AMCL-derived
const double global_origin_y = local_origin_y + robot_offset_y_;  // AMCL-derived

// Step 2: global coordinates → canvas pixel offset
const int offset_x = static_cast<int>(
  std::round((global_origin_x - canvas_origin_x_) / canvas_resolution_));
const int offset_y = static_cast<int>(
  std::round((global_origin_y - canvas_origin_y_) / canvas_resolution_));
```

### YAML Configuration — Unchanged

```yaml
# map_merge_params.yaml — ALL init_pose values remain 0.0
# The expansion node handles all spatial alignment, not the merger.

/ausra_1/map_merge/init_pose_x: 0.0
/ausra_1/map_merge/init_pose_y: 0.0
/ausra_1/map_merge/init_pose_yaw: 0.0
```

### Yaw Handling — Optional Extension

AMCL provides full `(x, y, yaw)` pose. The current `map_expansion_node` only
applies translational offsets. If yaw correction is desired, a small extension
would be needed:

```cpp
// OPTIONAL FUTURE EXTENSION — NOT REQUIRED FOR INITIAL DEPLOYMENT
// If robot_offset_yaw is added as a parameter, apply rotation before translation.
// This would eliminate the manual yaw alignment step entirely.
//
// For now, the physical yaw alignment procedure (SOP Section 4.5) is still
// required. However, AMCL's yaw output can be used as a VERIFICATION tool
// to detect misalignment before exploration begins:
//
//   if (std::abs(amcl_yaw) > 5.0 * M_PI / 180.0) {
//     RCLCPP_WARN("Robot yaw misalignment detected: %.1f degrees", amcl_yaw_deg);
//   }
```

---

## 4. System Components

| Component | Source | Role | Development Effort |
|---|---|---|---|
| **Reference map file** (`.pgm` + `.yaml`) | Generated once via `map_saver_cli` | Defines global coordinate frame | One 10-minute mapping run |
| **`nav2_amcl`** | Already installed (`nav2` dependency) | Particle-filter localization against reference map | Zero — existing package |
| **`nav2_map_server`** | Already installed (`nav2` dependency) | Serves reference map to AMCL | Zero — existing package |
| **`ausra_amcl_initialiser`** (new node) | Custom ROS 2 node, ~80 lines Python | Waits for AMCL convergence, extracts pose, writes `robot_offset_x/y` | 1–2 days development |
| **Robot LiDAR** | Already on AUSRA hardware | Provides scan data for AMCL matching | Zero — existing hardware |

### New Node: `ausra_amcl_initialiser`

```python
# Pseudocode — ausra_amcl_initialiser node (~80 lines)

class AmclInitialiser(Node):
    def __init__(self):
        # Subscribe to AMCL's pose output
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/ausra_X/amcl_pose',
            self.pose_callback
        )
        self.converged = False

    def pose_callback(self, msg):
        # Check if particle cloud has converged
        # (covariance diagonal elements below threshold)
        cov = msg.pose.covariance
        xy_variance = cov[0] + cov[7]  # xx + yy diagonal

        if xy_variance < CONVERGENCE_THRESHOLD and not self.converged:
            self.converged = True

            # Extract global pose
            robot_offset_x = msg.pose.pose.position.x
            robot_offset_y = msg.pose.pose.position.y
            robot_offset_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

            self.get_logger().info(
                f"AMCL converged: offset=({robot_offset_x:.3f}, "
                f"{robot_offset_y:.3f}), yaw={robot_offset_yaw:.1f}°"
            )

            # Write to map_expansion_node via parameter service
            # OR: publish to a topic that the launch system reads
            self.set_expansion_node_params(robot_offset_x, robot_offset_y)
```

---

## 5. Convergence Considerations

### Initial Pose Hint (Recommended)

AMCL converges faster and more reliably with an approximate initial pose hint.
This does NOT need to be accurate — a rough region is sufficient:

```bash
# Optional: publish approximate pose to speed convergence (±2 m tolerance)
ros2 topic pub --once /ausra_1/initialpose \
  geometry_msgs/PoseWithCovarianceStamped \
  "{pose: {pose: {position: {x: 3.0, y: 0.0}}, covariance: [4.0, ...]}}"
```

Alternatively, the robot can perform a slow 360° rotation in place at startup.
This guarantees full scan coverage and forces AMCL convergence even without an
initial hint, typically within 10–20 seconds.

### Convergence Failure Detection

The initialiser node must handle the case where AMCL fails to converge
(featureless environment, changed layout since reference map was built):

```
Convergence timeout (30 seconds):
  → Log ERROR with diagnostic info
  → Fall back to manual offset entry (tape measure SOP)
  → Do NOT launch expansion node with incorrect values
```

### Reference Map Staleness

The reference map must be updated whenever the environment layout changes
significantly (walls moved, large furniture rearranged). Minor changes
(chairs, boxes) do not typically affect AMCL convergence because particle
filtering is robust to partial occlusion.

**Recommended practice:** Re-map the environment at the start of each
deployment campaign (e.g., weekly or when moving to a new site). This takes
under 10 minutes.

---

## 6. Pros / Cons — Comparative Analysis

### vs. Tape Measure SOP (Current Baseline)

| Dimension | Tape Measure SOP | AMCL Prior Map |
|---|---|---|
| **Hardware required** | Tape, floor tape, laser level | None beyond existing LiDAR |
| **Daily setup time** | 15–30 min per session | 0 min (fully automatic) |
| **Yaw alignment** | Manual, error-prone, no software fix | Detected automatically (verification or correction) |
| **Accuracy** | ±2 cm (human-dependent) | ±3–5 cm (algorithm-dependent) |
| **Operator skill** | Two trained operators required | Zero operators (unattended boot) |
| **Failure mode** | Silent — bad measurement produces bad map | Detectable — covariance threshold flags non-convergence |
| **Scalability** | O(N) time per robot | O(1) — all robots self-localize in parallel |
| **Environment dependency** | Works anywhere | Requires sufficient LiDAR features (walls, obstacles) |
| **One-time setup** | Mark origin + axis | One mapping run (10 min) |

### vs. ArUco Fiducial Markers (Strategy 3 in Alternatives doc)

| Dimension | ArUco Fiducials | AMCL Prior Map |
|---|---|---|
| **Additional hardware** | Printed markers + camera calibration | None |
| **Sensor dependency** | RGB camera (lighting-sensitive) | LiDAR (lighting-independent) |
| **Line-of-sight** | Required to at least one marker | Not required — uses 360° scan |
| **Works in darkness** | No | Yes |
| **Works in dust/smoke** | No | Partially (LiDAR penetrates light particulate) |
| **Yaw detection** | Yes (from marker transform) | Yes (from particle convergence) |
| **Development effort** | ~100 lines + marker calibration | ~80 lines + one mapping run |
| **Physical infrastructure** | Markers must be placed, calibrated, maintained | None — the map file is the infrastructure |
| **Failure detection** | Marker not visible → explicit failure | Covariance threshold → explicit failure |
| **Dynamic start positions** | Yes (if marker is visible) | Yes (anywhere in mapped area) |
| **Multi-room / large area** | Needs markers distributed everywhere | Single map file covers entire area |

---

## 7. Recommended Position in Upgrade Path

```
Phase 1 (Now):           Tape Measure SOP
                         → Validate core pipeline on hardware

Phase 2 (Month 1):       Fixed Deployment Stations
                         → Eliminate daily measurement overhead

Phase 2.5 (Month 1–2):   ★ AMCL Prior Map ← THIS STRATEGY
                         → Zero additional hardware cost
                         → Automated localization using existing LiDAR
                         → Natural stepping stone before ArUco investment

Phase 3 (Month 2+):      ArUco Markers
                         → For environments where prior map is impractical
                         → (frequently changing layouts, outdoor transitions)

Phase 4 (Advanced):       UWB Positioning
                         → Fully autonomous, 24/7 unattended operation
```

### Why AMCL Slots Before ArUco

AMCL should be evaluated **before** investing in ArUco because:

1. **Zero hardware cost** — ArUco requires printed markers, placement, calibration,
   and camera verification. AMCL uses existing LiDAR.
2. **Zero physical infrastructure** — No markers to damage, obscure, or recalibrate.
   The reference map is a file on disk.
3. **Lighting-independent** — ArUco fails in poor lighting. AMCL works in complete
   darkness, which is critical for warehouse/tunnel deployments.
4. **Already in the dependency tree** — `nav2_amcl` is a standard `nav2` component
   that the AUSRA system already depends on for navigation. No new packages needed.
5. **Full-area coverage** — One reference map covers the entire operational area.
   ArUco requires markers distributed throughout every zone where a robot might start.

### When ArUco is Still Preferable

- Environments that change layout frequently (making the reference map stale)
- Outdoor or semi-outdoor areas where LiDAR features are sparse
- When camera-based pose provides higher accuracy than LiDAR AMCL in specific geometries

---

## 8. Updated Comparison Table (All 6 Strategies)

| Strategy | Setup Time | Daily Ops Time | HW Cost | Code Needed | Yaw Auto | Scales | Lighting Independent |
|---|---|---|---|---|---|---|---|
| **Tape Measure** | 30 min | 15–30 min | None | None | No | Moderate | Yes |
| **Fixed Stations** | 2–4 hrs | < 2 min | Low | None | Partial | Good | Yes |
| **★ AMCL Prior Map** | **10 min** | **0 min** | **None** | **~80 lines** | **Yes** | **Excellent** | **Yes** |
| **ArUco Fiducials** | 4–8 hrs | 0 min | Low | ~100 lines | Yes | Excellent | No |
| **UWB Positioning** | 1–2 days | 0 min | High | ~50 lines | Yes | Excellent | Yes |
| **LiDAR Scan Match** | 4–8 hrs | 5 min | None | ~200 lines | Yes | Limited | Yes |

---

## 9. Risks and Mitigations

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Reference map becomes stale (furniture moved) | Medium | Low–Medium | Re-map weekly or on layout change (10 min) |
| AMCL fails to converge (symmetric/featureless room) | Low | High | Detect via covariance timeout; fall back to tape SOP |
| Multiple convergence hypotheses (symmetric layout) | Low | High | Provide rough initial pose hint; add asymmetric landmarks |
| LiDAR sensor failure at boot | Very Low | High | Node detects no scan data and aborts with clear error |
| Reference map frame doesn't match canvas origin | Medium | Medium | Align map_saver origin with canvas_origin (-25, -25) during initial mapping, or compute the transform offset once |

---

## 10. Integration Notes

This strategy is a **drop-in replacement** for the tape-measure step only.
The `ausra_map_merge` package architecture does not change:

- **`map_expansion_node.cpp`** — Unchanged. Receives `robot_offset_x/y` as before.
- **`map_merge_params.yaml`** — `init_pose_*` stays at `0.0`.
- **`map_merge.launch.py`** — `ROBOT_SPAWN_POSES` values are populated by the
  AMCL initialiser node instead of being hardcoded from tape measurements.

The only new components are:
1. A reference map file (`.pgm` + `.yaml`) — generated once.
2. An `ausra_amcl_initialiser` node (~80 lines Python) — reads converged AMCL
   pose, writes offset parameters, then exits.
3. Launch file modifications to start AMCL and the initialiser before the
   expansion nodes.
