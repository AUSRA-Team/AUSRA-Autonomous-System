# AUSRA ArUco Dynamic Initialization Workflow
**Document:** `ArUco_Dynamic_Initialization_Workflow.md`  
**Purpose:** Details the workflow for allowing robots to begin SLAM mapping immediately and dynamically calculate their global canvas offsets once an ArUco marker is detected.

---

## 1. Core Architecture: The "Lighthouse" Principle

In this deployment strategy, ArUco markers act as fixed global anchors (lighthouses). 
* Markers are placed at known global coordinates in the physical room (e.g., Marker 42 is at Global X=5.0, Y=2.0).
* Multiple robots can use the **same marker** to localize. The ROS 2 ArUco detection node calculates the geometric transform from the robot's camera to the marker. 
* By knowing where the marker is, and how far the robot is from the marker, the robot determines its own global position.

---

## 2. Operational Workflow (Map First, Locate Later)

This workflow allows for rapid deployment where robots do not need to be placed directly in front of a marker to begin operations.

### Step 1: Power On & Alignment (CRITICAL)
Because the `map_expansion_node` performs pure translation (shifting X and Y pixels) without rotation, the local SLAM map must be generated parallel to the global room axes.
* Power on the robots anywhere in the arena.
* **Mandatory:** The physical robot must be facing exactly parallel to the room's designated X-axis when `slam_toolbox` is launched.

### Step 2: Begin Local Mapping
* Launch the standard `slam_toolbox` and sensor stack on the robot.
* The robot begins building its local map, starting at its own arbitrary `(0,0)`.
* The robot begins exploring the environment.

### Step 3: Marker Detection & TF Resolution
* While exploring, the robot's camera enters the line-of-sight of an ArUco marker.
* The `ros2_aruco` node detects the marker and publishes the relative pose.
* The custom `ausra_pose_initialiser` script triggers.

### Step 4: The Backwards Math Calculation
The initialization script calculates the `robot_offset_x` and `robot_offset_y` required for the map merger. Because the robot has moved since it started mapping, the script uses the TF tree to calculate the offset of the *local map origin*, not just the current robot position.

1. **Get Marker Global Pose:** Looks up the detected marker ID in `markers.yaml` (e.g., Global X=10, Y=10).
2. **Get Robot Global Pose:** Subtracts the camera-to-marker distance to find the robot's current global position (e.g., Global X=8, Y=10).
3. **Get Robot Local Pose:** Looks up the robot's current position within its local SLAM map using the `map -> base_link` TF (e.g., Robot has driven Local X=3, Y=0).
4. **Calculate Map Origin Offset:** Subtracts the local pose from the global pose to find where the local `(0,0)` lives in the global frame.
   * `Offset X = Global X (8) - Local X (3) = 5.0`
   * `Offset Y = Global Y (10) - Local Y (0) = 10.0`

### Step 5: Launching the Merge Pipeline
* The `ausra_pose_initialiser` script automatically passes the calculated `offset_x=5.0` and `offset_y=10.0` to `map_merge.launch.py`.
* The `map_expansion_node` launches, takes the currently active local SLAM map, shifts all pixels by the calculated offset, and publishes the aligned canvas to the central `multirobot_map_merge` node.