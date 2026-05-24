# ArUco Pipeline Integration Steps
**Document:** `ArUco_Pipeline_Integration_Steps.md`
**Package:** `ausra_map_merge_HW` (existing `ament_cmake` C++ package)
**Goal:** Inject the ArUco Python initialisation pipeline without breaking
the existing `map_expansion_node` C++ executable.

---

## ⚠️ Read This First — Critical Bug in the Guide

Before touching any file, there is a **silent correctness bug** in the
`ArUco_Fiducials_Implementation_Guide.md` that must be patched first.

**The problem:**

`ausra_pose_initialiser.py` calls the ROS 2 `SetParameters` service on the
running `map_expansion_node` to write the detected `robot_offset_x/y`.
This correctly updates the node's declared parameter on the ROS 2 parameter
server. However, `map_expansion_node.cpp` **caches** those values into C++
member variables at construction time only:

```cpp
// map_expansion_node.cpp — constructor, line 127–128
robot_offset_x_ = this->get_parameter("robot_offset_x").as_double();
robot_offset_y_ = this->get_parameter("robot_offset_y").as_double();
```

There is **no `on_set_parameters_callback`** in the node. Once the node is
running, updating `robot_offset_x` via `SetParameters` changes the value
on the parameter server, but `robot_offset_x_` (the member variable used
in every `mapCallback` computation) **never changes**. The expansion node
silently continues computing offsets from `(0.0, 0.0)` with no error.

**Section 1 of this document patches the C++ node to fix this before any
Python code is added.**

---

## Table of Contents

1. [Patch `map_expansion_node.cpp` — Dynamic Parameter Callback](#1-patch-map_expansion_nodecpp--dynamic-parameter-callback)
2. [CMakeLists.txt — Hybrid C++/Python Setup](#2-cmakeliststxt--hybrid-cpython-setup)
3. [package.xml — Python Dependencies](#3-packagexml--python-dependencies)
4. [Directory Structure — Terminal Commands](#4-directory-structure--terminal-commands)
5. [Python Scripts — Hardware-Corrected Code](#5-python-scripts--hardware-corrected-code)
6. [Config Files](#6-config-files)
7. [Launch Files](#7-launch-files)
8. [Workflow Summary — SetParameters Chain](#8-workflow-summary--setparameters-chain)
9. [Topic and Frame Corrections vs the Guide](#9-topic-and-frame-corrections-vs-the-guide)

---

## 1. Patch `map_expansion_node.cpp` — Dynamic Parameter Callback

This is the **only change** to the C++ source file. Add two things:

1. A `rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr`
   member variable.
2. An `add_on_set_parameters_callback` call in the constructor that
   updates `robot_offset_x_` / `robot_offset_y_` and clears the canvas
   when the ArUco initialiser pushes new values.

### 1.1 New Member Variable

Add this declaration to the member variables block at the bottom of the
class, alongside the existing ROS 2 interfaces:

```cpp
// ── Dynamic parameter update handle (ArUco integration) ──────────────────
// Allows ausra_pose_initialiser.py to write robot_offset_x/y at runtime
// via the SetParameters service. Without this, SetParameters updates only
// the ROS 2 parameter server — the cached member variables never change.
rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    param_callback_handle_;
```

### 1.2 Register the Callback (in the Constructor)

Add this block immediately after the two `robot_offset_*_ = ...` lines
(after the existing parameter reads, before the canvas pre-allocation):

```cpp
// ── Dynamic parameter update callback (ArUco integration) ─────────────────
// When ausra_pose_initialiser.py calls SetParameters with detected
// robot_offset_x/y values, this callback:
//   1. Updates the member variables used in mapCallback math.
//   2. Clears the canvas — cells stamped at offset (0,0) are now wrong
//      and must be erased before the new offset is applied.
// The mutex guarantees publishCanvas() does not read a partially-cleared
// canvas during the reset.
param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& params)
    -> rcl_interfaces::msg::SetParametersResult
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        bool offset_changed = false;
        for (const auto& param : params) {
            if (param.get_name() == "robot_offset_x") {
                robot_offset_x_ = param.as_double();
                offset_changed = true;
                RCLCPP_INFO(this->get_logger(),
                    "[ArUco] robot_offset_x updated to %.4f",
                    robot_offset_x_);
            } else if (param.get_name() == "robot_offset_y") {
                robot_offset_y_ = param.as_double();
                offset_changed = true;
                RCLCPP_INFO(this->get_logger(),
                    "[ArUco] robot_offset_y updated to %.4f",
                    robot_offset_y_);
            }
        }

        if (offset_changed) {
            // Reset canvas so stale cells (stamped at old offset) are cleared.
            // The next mapCallback will repopulate at the correct positions.
            std::lock_guard<std::mutex> lock(canvas_mutex_);
            std::fill(canvas_data_.begin(), canvas_data_.end(),
                      static_cast<int8_t>(-1));
            last_written_indices_.clear();
            RCLCPP_INFO(this->get_logger(),
                "[ArUco] Canvas cleared — stale cells from offset (0,0) removed. "
                "New offset will be applied on next SLAM frame.");
        }

        return result;
    });

RCLCPP_INFO(this->get_logger(),
    "Dynamic parameter callback registered. "
    "robot_offset_x/y can be updated at runtime via SetParameters.");
```

### 1.3 Required Header

Add this include at the top of `map_expansion_node.cpp` if not already
present (it is part of `rclcpp`):

```cpp
#include <rcl_interfaces/msg/set_parameters_result.hpp>
```

---

## 2. CMakeLists.txt — Hybrid C++/Python Setup

Replace the full `CMakeLists.txt` with this version. The only additions vs
the original are:
- `find_package(ament_cmake_python REQUIRED)` — enables Python script install
- `install(PROGRAMS ...)` — installs `.py` scripts as executable ROS 2 nodes

```cmake
cmake_minimum_required(VERSION 3.8)
project(ausra_map_merge_HW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake       REQUIRED)
find_package(ament_cmake_python REQUIRED)   # ← NEW: enables Python script install
find_package(rclcpp            REQUIRED)
find_package(nav_msgs          REQUIRED)

# ── C++ Executable ────────────────────────────────────────────────────────────
add_executable(map_expansion_node src/map_expansion_node.cpp)

ament_target_dependencies(map_expansion_node
  rclcpp
  nav_msgs
)

# ── Install C++ executable ────────────────────────────────────────────────────
install(TARGETS
  map_expansion_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install Python scripts as ROS 2 executable nodes ─────────────────────────
# Each script must have:
#   1. #!/usr/bin/env python3  as the first line
#   2. Execute permission (chmod +x) — install(PROGRAMS) sets this automatically
#
# After install, nodes are runnable as:
#   ros2 run ausra_map_merge_HW aruco_detector_node.py
#   ros2 run ausra_map_merge_HW ausra_pose_initialiser.py
install(PROGRAMS
  scripts/aruco_detector_node.py
  scripts/ausra_pose_initialiser.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install launch, config, docs ──────────────────────────────────────────────
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs
)

ament_package()
```

---

## 3. package.xml — Python Dependencies

Replace the full `package.xml` with this version. New additions are marked:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
            schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ausra_map_merge_HW</name>
  <version>1.0.0</version>
  <description>
    Hardware deployment variant of ausra_map_merge for physical AUSRA robots.
    Includes C++ map_expansion_node (heartbeat timer) and Python ArUco
    initialisation pipeline (aruco_detector_node + ausra_pose_initialiser).
  </description>
  <maintainer email="ausra@team.local">AUSRA Team</maintainer>
  <license>Apache-2.0</license>

  <!-- Build tools -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>  <!-- NEW -->

  <!-- C++ node dependencies -->
  <depend>rclcpp</depend>
  <depend>nav_msgs</depend>

  <!-- Python node dependencies (NEW) -->
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>rcl_interfaces</depend>

  <!-- System Python packages (NEW) -->
  <!-- Install with: pip3 install opencv-contrib-python numpy pyyaml      -->
  <!-- opencv-contrib-python includes the aruco module                    -->
  <!-- NOTE: scipy is NOT required — this package uses pure-numpy         -->
  <!--       quaternion conversion in aruco_detector_node.py              -->
  <exec_depend>python3-opencv</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>python3-yaml</exec_depend>

  <!-- Runtime dependencies -->
  <exec_depend>multirobot_map_merge</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Install System Python Packages

Run this once on the robot's onboard computer before building:

```bash
# Install opencv with aruco module (contrib version required)
pip3 install opencv-contrib-python numpy pyyaml

# Verify ArUco is available
python3 -c "import cv2; print(cv2.aruco.DICT_4X4_50); print('ArUco OK')"
```

---

## 4. Directory Structure — Terminal Commands

Run these from the root of your ROS 2 workspace:

```bash
cd ~/ros2_ws/src/ausra_map_merge_HW

# Create the scripts directory for Python nodes
mkdir -p scripts

# Create the ArUco config files (populated in Section 6)
touch config/aruco_markers.yaml
touch config/camera_calibration.yaml

# Create the Python node files (populated in Section 5)
touch scripts/aruco_detector_node.py
touch scripts/ausra_pose_initialiser.py

# Set execute permissions on Python scripts
# (install(PROGRAMS) in CMake handles this at install time,
#  but set it now so you can run them directly during testing)
chmod +x scripts/aruco_detector_node.py
chmod +x scripts/ausra_pose_initialiser.py

# Create the new launch files (populated in Section 7)
touch launch/aruco_init.launch.py
touch launch/map_merge_hw_aruco.launch.py
```

Final structure after these commands:

```
ausra_map_merge_HW/
├── CMakeLists.txt                       ← updated (Section 2)
├── package.xml                          ← updated (Section 3)
├── src/
│   └── map_expansion_node.cpp           ← patched (Section 1)
├── scripts/                             ← NEW
│   ├── aruco_detector_node.py           ← NEW (Section 5.1)
│   └── ausra_pose_initialiser.py        ← NEW (Section 5.2)
├── launch/
│   ├── map_merge_hw.launch.py           ← existing (single-robot baseline test)
│   ├── aruco_init.launch.py             ← NEW (Section 7.1)
│   └── map_merge_hw_aruco.launch.py     ← NEW (Section 7.2)
├── config/
│   ├── map_merge_HW_params.yaml         ← existing, unchanged
│   ├── aruco_markers.yaml               ← NEW (Section 6.1)
│   └── camera_calibration.yaml          ← NEW (Section 6.2)
└── docs/
    ├── Hardware_Baseline_Testing_Plan.md
    ├── AUSRA_Hardware_Map_Merge_SOP.md
    └── Alternative_Hardware_Strategies.md
```

---

## 5. Python Scripts — Hardware-Corrected Code

### 5.1 `scripts/aruco_detector_node.py`

**Corrections vs the guide:**
- Camera topics are **parameters** (not hardcoded with robot name prefix) because
  the hardware SLAM stack has no namespace — the OAK camera publishes to
  `/oak_camera/image_raw`, not `/{robot_name}/oak_camera/image_raw`.
- Camera optical frame is parameterised to match the hardware TF convention
  (`ausrabot_camera_link_optical`, not `{robot_name}_camera_link_optical`).
- **`scipy` removed** — replaced with a pure `math`/`numpy` quaternion
  conversion. This removes an undeclared system dependency that would cause
  a silent import error at runtime.

```python
#!/usr/bin/env python3
"""
aruco_detector_node.py
Detects ArUco markers from the OAK camera and publishes marker poses.

HARDWARE TOPIC NOTE:
  The AUSRA hardware stack runs in the global namespace (no robot prefix).
  Camera topics are therefore /oak_camera/image_raw and
  /oak_camera/camera_info — NOT /{robot_name}/oak_camera/image_raw.
  Configure via the 'image_topic' and 'camera_info_topic' parameters.

Subscribes:
  image_topic       (default: /oak_camera/image_raw)
  camera_info_topic (default: /oak_camera/camera_info)

Publishes:
  /{robot_name}/aruco/detected_marker  (geometry_msgs/PoseStamped)
  /{robot_name}/aruco/marker_id        (std_msgs/Int32)
"""

import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from cv_bridge import CvBridge


def rotation_matrix_to_quaternion(R):
    """
    Convert a 3×3 rotation matrix to a quaternion [x, y, z, w].
    Pure math/numpy implementation — no scipy required.
    Uses Shepperd's method for numerical stability.
    """
    trace = R[0][0] + R[1][1] + R[2][2]
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2][1] - R[1][2]) * s
        y = (R[0][2] - R[2][0]) * s
        z = (R[1][0] - R[0][1]) * s
    elif (R[0][0] > R[1][1]) and (R[0][0] > R[2][2]):
        s = 2.0 * math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2])
        w = (R[2][1] - R[1][2]) / s
        x = 0.25 * s
        y = (R[0][1] + R[1][0]) / s
        z = (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s = 2.0 * math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2])
        w = (R[0][2] - R[2][0]) / s
        x = (R[0][1] + R[1][0]) / s
        y = 0.25 * s
        z = (R[1][2] + R[2][1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1])
        w = (R[1][0] - R[0][1]) / s
        x = (R[0][2] + R[2][0]) / s
        y = (R[1][2] + R[2][1]) / s
        z = 0.25 * s
    return x, y, z, w


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter('robot_name',               'ausra_1')
        self.declare_parameter('dictionary',               'DICT_4X4_50')
        self.declare_parameter('marker_size_m',             0.15)
        self.declare_parameter('max_detection_distance_m',  3.0)

        # Hardware-specific: camera topics are global (no robot namespace prefix)
        # because hardware_full_stack.launch.py applies no top-level namespace.
        # Override these parameters if your camera driver uses different names.
        self.declare_parameter('image_topic',
                               '/oak_camera/image_raw')
        self.declare_parameter('camera_info_topic',
                               '/oak_camera/camera_info')

        # Hardware TF frame for the camera optical axis.
        # Based on AUSRA hardware convention (ausrabot_* prefix from URDF).
        # Verify with: ros2 run tf2_tools view_frames.py, then check camera link.
        self.declare_parameter('camera_optical_frame',
                               'ausrabot_camera_link_optical')

        self.robot_name     = self.get_parameter('robot_name').value
        marker_size         = self.get_parameter('marker_size_m').value
        dict_name           = self.get_parameter('dictionary').value
        self.max_dist       = self.get_parameter('max_detection_distance_m').value
        image_topic         = self.get_parameter('image_topic').value
        camera_info_topic   = self.get_parameter('camera_info_topic').value
        self.optical_frame  = self.get_parameter('camera_optical_frame').value

        # ── ArUco detector setup ────────────────────────────────────────────
        dict_id = getattr(cv2.aruco, dict_name)
        aruco_dict     = cv2.aruco.getPredefinedDictionary(dict_id)
        aruco_params   = cv2.aruco.DetectorParameters()
        self.detector  = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        self.marker_size = marker_size
        self.bridge    = CvBridge()

        # Camera intrinsics (populated from CameraInfo topic)
        self.camera_matrix = None
        self.dist_coeffs   = None

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(
            Image, image_topic,
            self.image_callback, 10)
        self.create_subscription(
            CameraInfo, camera_info_topic,
            self.camera_info_callback, 10)

        # ── Publishers ──────────────────────────────────────────────────────
        # Output topics are namespaced by robot name for multi-robot support.
        self.pose_pub = self.create_publisher(
            PoseStamped,
            f'/{self.robot_name}/aruco/detected_marker', 10)
        self.id_pub = self.create_publisher(
            Int32,
            f'/{self.robot_name}/aruco/marker_id', 10)

        self.get_logger().info(
            f'ArUco detector ready:\n'
            f'  robot      : {self.robot_name}\n'
            f'  dictionary : {dict_name}\n'
            f'  marker size: {marker_size} m\n'
            f'  image topic: {image_topic}\n'
            f'  info topic : {camera_info_topic}\n'
            f'  camera frame: {self.optical_frame}')

    def camera_info_callback(self, msg: CameraInfo):
        """Cache camera intrinsics from CameraInfo — fires once, then ignored."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs   = np.array(msg.d)
            self.get_logger().info(
                f'Camera intrinsics received from {msg.header.frame_id}:\n'
                f'  fx={self.camera_matrix[0,0]:.1f}  '
                f'fy={self.camera_matrix[1,1]:.1f}  '
                f'cx={self.camera_matrix[0,2]:.1f}  '
                f'cy={self.camera_matrix[1,2]:.1f}')

    def image_callback(self, msg: Image):
        """Detect ArUco markers and publish the first valid marker's pose."""
        if self.camera_matrix is None:
            # Throttled warning so it doesn't flood before CameraInfo arrives
            self.get_logger().warn(
                'Waiting for camera intrinsics from camera_info topic...',
                throttle_duration_sec=5.0)
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        corners, ids, _ = self.detector.detectMarkers(frame)

        if ids is None or len(ids) == 0:
            return

        for i, marker_id in enumerate(ids.flatten()):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corners[i]], self.marker_size,
                self.camera_matrix, self.dist_coeffs)

            distance = float(np.linalg.norm(tvec[0][0]))
            if distance > self.max_dist:
                self.get_logger().debug(
                    f'Marker {marker_id} at {distance:.2f}m exceeds '
                    f'max_detection_distance ({self.max_dist}m). Skipping.')
                continue

            # Publish marker ID
            id_msg = Int32()
            id_msg.data = int(marker_id)
            self.id_pub.publish(id_msg)

            # Build pose from tvec and rvec
            rot_matrix, _ = cv2.Rodrigues(rvec[0][0])
            qx, qy, qz, qw = rotation_matrix_to_quaternion(rot_matrix)

            pose_msg = PoseStamped()
            pose_msg.header.stamp    = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.optical_frame

            pose_msg.pose.position.x = float(tvec[0][0][0])
            pose_msg.pose.position.y = float(tvec[0][0][1])
            pose_msg.pose.position.z = float(tvec[0][0][2])

            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw

            self.pose_pub.publish(pose_msg)
            self.get_logger().debug(
                f'Marker {marker_id} detected at distance {distance:.3f} m')
            break  # Publish only the first valid marker per frame


def main():
    rclpy.init()
    rclpy.spin(ArucoDetectorNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

### 5.2 `scripts/ausra_pose_initialiser.py`

**Preserved from the guide as-is.** The only corrections are:
- The `SetParameters` service path `/map_expansion_{robot_name}/set_parameters`
  matches the node name `map_expansion_ausra_1` used in the hardware launch file.
- A `rclpy.shutdown()` call is added as a cleaner alternative to `SystemExit`.

```python
#!/usr/bin/env python3
"""
ausra_pose_initialiser.py

Reads ArUco marker detection, looks up the marker's known global position
from aruco_markers.yaml, computes the robot's global (x, y), and writes
robot_offset_x / robot_offset_y into the map_expansion_node via the
ROS 2 SetParameters service.

This node runs ONCE at boot, collects N detections for averaging, writes
the offsets, and then shuts itself down.

PARAMETER SERVICE PATH:
  /map_expansion_{robot_name}/set_parameters
  e.g. /map_expansion_ausra_1/set_parameters
  This matches the node name 'map_expansion_ausra_1' set in the launch file.

PREREQUISITE:
  map_expansion_node.cpp must have the dynamic parameter callback installed
  (see Section 1 of ArUco_Pipeline_Integration_Steps.md). Without that patch,
  this service call succeeds silently but the C++ node ignores the new values.
"""

import math
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


class PoseInitialiser(Node):
    def __init__(self):
        super().__init__('ausra_pose_initialiser')

        self.declare_parameter('robot_name',          'ausra_1')
        self.declare_parameter('markers_config',       '')
        self.declare_parameter('convergence_samples',  5)

        self.robot_name       = self.get_parameter('robot_name').value
        config_path           = self.get_parameter('markers_config').value
        self.required_samples = self.get_parameter('convergence_samples').value

        if not config_path:
            self.get_logger().error(
                'markers_config parameter is empty. '
                'Pass the full path to aruco_markers.yaml.')
            raise RuntimeError('markers_config not set')

        # Load marker registry from YAML
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        self.markers = {m['id']: m for m in config['markers']}
        self.get_logger().info(
            f'Loaded {len(self.markers)} markers from {config_path}')

        # Detection state
        self.current_marker_id = None
        self.samples           = []
        self.done              = False

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            Int32, f'/{self.robot_name}/aruco/marker_id',
            self.id_callback, 10)
        self.create_subscription(
            PoseStamped, f'/{self.robot_name}/aruco/detected_marker',
            self.pose_callback, 10)

        # ── SetParameters service client ─────────────────────────────────────
        # Target: the map_expansion_node running for this robot.
        # Node name in launch file: map_expansion_{robot_name}
        service_path = f'/map_expansion_{self.robot_name}/set_parameters'
        self.param_client = self.create_client(SetParameters, service_path)
        self.get_logger().info(
            f'Pose initialiser ready for {self.robot_name}.\n'
            f'  Marker service : {service_path}\n'
            f'  Samples needed : {self.required_samples}\n'
            f'  Waiting for ArUco detections...')

    def id_callback(self, msg: Int32):
        self.current_marker_id = msg.data

    def pose_callback(self, msg: PoseStamped):
        if self.done or self.current_marker_id is None:
            return

        marker_id = self.current_marker_id
        if marker_id not in self.markers:
            self.get_logger().warn(
                f'Marker ID {marker_id} not found in aruco_markers.yaml. '
                f'Known IDs: {list(self.markers.keys())}. Ignoring.')
            return

        marker = self.markers[marker_id]

        # Camera optical frame coordinates:
        #   Z = depth (forward from camera to marker)
        #   X = lateral (positive = right)
        # We project the 3D offset onto the ground plane for 2D alignment.
        dx = msg.pose.position.x   # lateral camera-frame offset
        dz = msg.pose.position.z   # depth (camera-to-marker distance)

        # Transform camera-frame relative offset to global frame using
        # the marker's known global yaw (direction the marker faces).
        # robot_pos = marker_global - (camera_to_marker vector in global frame)
        marker_yaw = marker['global_yaw']
        robot_x = (marker['global_x']
                   - (dz * math.cos(marker_yaw) - dx * math.sin(marker_yaw)))
        robot_y = (marker['global_y']
                   - (dz * math.sin(marker_yaw) + dx * math.cos(marker_yaw)))

        self.samples.append((robot_x, robot_y))
        self.get_logger().info(
            f'  Sample {len(self.samples)}/{self.required_samples}: '
            f'marker={marker_id}  '
            f'robot=({robot_x:.4f}, {robot_y:.4f})')

        if len(self.samples) >= self.required_samples:
            self.finalize()

    def finalize(self):
        """Average all collected samples and push offsets to map_expansion_node."""
        self.done = True
        avg_x = sum(s[0] for s in self.samples) / len(self.samples)
        avg_y = sum(s[1] for s in self.samples) / len(self.samples)

        self.get_logger().info(
            f'\n'
            f'  ══════════════════════════════════════════\n'
            f'  CONVERGED after {len(self.samples)} samples:\n'
            f'    robot_offset_x = {avg_x:.4f} m\n'
            f'    robot_offset_y = {avg_y:.4f} m\n'
            f'  ══════════════════════════════════════════')

        if not self.param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f'SetParameters service not available at '
                f'/map_expansion_{self.robot_name}/set_parameters\n'
                f'Ensure map_expansion_node is running before this node.')
            return

        req = SetParameters.Request()
        req.parameters = [
            Parameter(
                name='robot_offset_x',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=avg_x)),
            Parameter(
                name='robot_offset_y',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=avg_y)),
        ]
        future = self.param_client.call_async(req)
        future.add_done_callback(self._on_params_set)

    def _on_params_set(self, future):
        try:
            result = future.result()
            for r in result.results:
                if not r.successful:
                    self.get_logger().error(
                        f'SetParameters failed: {r.reason}')
                    return
            self.get_logger().info(
                'robot_offset_x/y written to map_expansion_node successfully.\n'
                'The C++ node has updated its canvas offset and cleared stale cells.\n'
                'Pose initialiser shutting down.')
        except Exception as e:
            self.get_logger().error(f'SetParameters call raised: {e}')
            return

        # Clean shutdown
        rclpy.shutdown()


def main():
    rclpy.init()
    node = PoseInitialiser()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 6. Config Files

### 6.1 `config/aruco_markers.yaml`

Commission once. Measure each marker's global `(x, y, yaw)` from the
physical origin using the same tape-measure procedure as the baseline SOP.
These values never change unless markers are physically moved.

```yaml
# config/aruco_markers.yaml
# ──────────────────────────────────────────────────────────────────────────
# Global positions of all ArUco markers in the physical environment.
# Measured ONCE during commissioning relative to the physical origin mark.
# Coordinate system: same as AUSRA_Hardware_Map_Merge_SOP.md Section 3.
#
# global_yaw: angle in RADIANS that the marker faces.
#   0.0       = marker faces +X (toward the +X tape line)
#   1.5708    = marker faces +Y (90° counterclockwise from +X)
#   3.1416    = marker faces -X
#   -1.5708   = marker faces -Y
# ──────────────────────────────────────────────────────────────────────────

aruco_config:
  dictionary:               DICT_4X4_50   # Must match aruco_detector_node param
  marker_size_m:            0.15          # Physical side length of printed marker
  convergence_samples:      5             # Detections averaged before writing offset
  max_detection_distance_m: 3.0          # Reject detections beyond this range

markers:
  - id: 0
    global_x:   0.0
    global_y:   0.0
    global_yaw: 0.0
    description: "Origin wall — northwest corner of Room A"

  - id: 1
    global_x:   5.0
    global_y:   0.0
    global_yaw: 0.0
    description: "East wall — Room A"

  - id: 2
    global_x:   0.0
    global_y:   4.0
    global_yaw: 1.5708
    description: "South corridor entry — faces +Y"

  # ── Template for additional markers ────────────────────────────────────
  # - id: 3
  #   global_x:   2.5
  #   global_y:   3.0
  #   global_yaw: 0.0
  #   description: ""
```

### 6.2 `config/camera_calibration.yaml`

Replace the placeholder values below with output from the ROS 2 camera
calibration tool (see Section 7.3 of the ArUco guide for the calibration
command). The values below are structural placeholders only.

```yaml
# config/camera_calibration.yaml
# ──────────────────────────────────────────────────────────────────────────
# OAK camera intrinsic calibration.
# Generated by: ros2 run camera_calibration cameracalibrator
#
# IMPORTANT: The values below (615.0 focal length, zero distortion) are
# PLACEHOLDERS. Replace with real values from your OAK camera calibration
# before using ArUco detection in production.
# ──────────────────────────────────────────────────────────────────────────

image_width:  640
image_height: 480

camera_matrix:
  rows: 3
  cols: 3
  data: [615.0,   0.0, 320.0,
           0.0, 615.0, 240.0,
           0.0,   0.0,   1.0]

distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0, 0.0, 0.0, 0.0, 0.0]   # ← REPLACE WITH REAL VALUES

camera_name: oak_camera
distortion_model: plumb_bob
```

> **Note:** `aruco_detector_node.py` reads intrinsics from the live
> `/oak_camera/camera_info` topic at runtime (published by the OAK driver),
> not from this YAML file. This YAML is retained for reference and for
> offline debugging. The `camera_calibration cameracalibrator` tool writes
> the values into the camera driver which publishes them on `camera_info`.

---

## 7. Launch Files

### 7.1 `launch/aruco_init.launch.py`

Launches the detector and initialiser for one robot. Used standalone for
single-robot testing or included by the full stack launch.

```python
#!/usr/bin/env python3
"""
aruco_init.launch.py
Launches ArUco detection and pose initialisation for one robot.

Usage (single robot):
  ros2 launch ausra_map_merge_HW aruco_init.launch.py robot_name:=ausra_1

Usage (included from map_merge_hw_aruco.launch.py):
  Launched per robot via IncludeLaunchDescription with robot_name argument.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share     = get_package_share_directory('ausra_map_merge_HW')
    markers_config = os.path.join(pkg_share, 'config', 'aruco_markers.yaml')

    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='ausra_1',
        description='Robot namespace prefix (e.g. ausra_1)')

    # ── ArUco Detector ─────────────────────────────────────────────────────
    # Runs continuously. Publishes detections at camera framerate.
    # Stops publishing when no marker is visible.
    detector_node = Node(
        package='ausra_map_merge_HW',
        executable='aruco_detector_node.py',
        name='aruco_detector',
        namespace='',
        parameters=[{
            'robot_name':               LaunchConfiguration('robot_name'),
            'dictionary':               'DICT_4X4_50',
            'marker_size_m':             0.15,
            'max_detection_distance_m':  3.0,
            # Hardware topics — global namespace (no robot prefix in HW stack)
            'image_topic':             '/oak_camera/image_raw',
            'camera_info_topic':       '/oak_camera/camera_info',
            # Hardware camera frame — verify with: ros2 run tf2_tools view_frames.py
            'camera_optical_frame':    'ausrabot_camera_link_optical',
        }],
        output='screen',
    )

    # ── Pose Initialiser ───────────────────────────────────────────────────
    # Runs ONCE. Collects N detections, averages them, calls SetParameters
    # on map_expansion_node, then shuts itself down.
    initialiser_node = Node(
        package='ausra_map_merge_HW',
        executable='ausra_pose_initialiser.py',
        name='pose_initialiser',
        namespace='',
        parameters=[{
            'robot_name':          LaunchConfiguration('robot_name'),
            'markers_config':      markers_config,
            'convergence_samples': 5,
        }],
        output='screen',
    )

    return LaunchDescription([
        robot_name_arg,
        LogInfo(msg='[ArUco] Detector and pose initialiser launching...'),
        detector_node,
        initialiser_node,
    ])
```

### 7.2 `launch/map_merge_hw_aruco.launch.py`

Full stack: ArUco init → expansion node → central merge. This replaces
manual offset entry in `ROBOT_HW_CONFIG`.

```python
#!/usr/bin/env python3
"""
map_merge_hw_aruco.launch.py
Full hardware map merge with automatic ArUco initialisation.

Sequence:
  T+0s    ArUco detector and pose initialiser launch for each robot.
  T+0s    map_expansion_node launches with offset (0.0, 0.0).
          Its heartbeat timer immediately publishes all-Unknown canvas.
  T+0–15s Pose initialiser collects 5 ArUco detections, averages them,
          calls SetParameters to update robot_offset_x/y on the C++ node.
          The C++ node updates its member variables and clears the canvas.
  T+15s+  Subsequent SLAM frames are stamped at the correct global position.
  T+20s   Central map_merge node starts.

NOTE:
  The expansion node is intentionally started at (0.0, 0.0) and updated
  dynamically. It must have the parameter callback patch installed (Section 1
  of ArUco_Pipeline_Integration_Steps.md) for SetParameters to take effect.
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# ── Configure fleet here ──────────────────────────────────────────────────────
# For single-robot test: include only 'ausra_1' and keep the phantom.
# For multi-robot: list all active robots; remove phantom entry.
ACTIVE_ROBOTS   = ['ausra_1']
PHANTOM_ROBOTS  = ['ausra_2']   # Remove once 2nd physical robot is added

CANVAS_WIDTH      = 1000
CANVAS_HEIGHT     = 1000
CANVAS_RESOLUTION = 0.05
CANVAS_ORIGIN_X   = -25.0
CANVAS_ORIGIN_Y   = -25.0
# ─────────────────────────────────────────────────────────────────────────────


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share   = get_package_share_directory('ausra_map_merge_HW')
    params_file = os.path.join(pkg_share, 'config', 'map_merge_HW_params.yaml')
    aruco_launch = os.path.join(pkg_share, 'launch', 'aruco_init.launch.py')

    ld.add_action(LogInfo(msg=(
        '\n'
        '╔═══════════════════════════════════════════════════════════════╗\n'
        '║      AUSRA Hardware Map Merge — ArUco Auto-Init Mode         ║\n'
        '╠═══════════════════════════════════════════════════════════════╣\n'
        '║  Step 1: ArUco detector + pose initialiser launch per robot  ║\n'
        '║  Step 2: Expansion nodes launch at (0,0), await ArUco update ║\n'
        '║  Step 3: Initialiser writes offset → C++ node updates canvas ║\n'
        '║  Step 4: Central merger launches after 20s                   ║\n'
        '╚═══════════════════════════════════════════════════════════════╝'
    )))

    # ── Stage 1: ArUco initialisation per active robot (immediate) ──────────
    for robot_name in ACTIVE_ROBOTS:
        aruco_init = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(aruco_launch),
            launch_arguments={'robot_name': robot_name}.items(),
        )
        ld.add_action(aruco_init)

    # ── Stage 2: Expansion nodes (immediate — heartbeat starts right away) ───
    # Launched at (0,0). The pose initialiser will update the offsets
    # dynamically once ArUco convergence is achieved.
    for robot_name in ACTIVE_ROBOTS:
        expansion_node = Node(
            package='ausra_map_merge_HW',
            executable='map_expansion_node',
            name=f'map_expansion_{robot_name}',
            namespace='',
            parameters=[{
                'input_topic':        f'/{robot_name}/map'
                                       if robot_name != 'ausra_1'
                                       else '/map',   # hardware: no namespace
                'output_topic':       f'/{robot_name}/map_fixed',
                'robot_offset_x':      0.0,   # Updated by pose_initialiser
                'robot_offset_y':      0.0,   # Updated by pose_initialiser
                'canvas_width':        CANVAS_WIDTH,
                'canvas_height':       CANVAS_HEIGHT,
                'canvas_resolution':   CANVAS_RESOLUTION,
                'canvas_origin_x':     CANVAS_ORIGIN_X,
                'canvas_origin_y':     CANVAS_ORIGIN_Y,
                'publish_rate_hz':     1.0,
            }],
            output='screen',
        )
        ld.add_action(expansion_node)

    # ── Stage 2b: Phantom expansion nodes (prevent single-map segfault) ─────
    for robot_name in PHANTOM_ROBOTS:
        phantom_node = Node(
            package='ausra_map_merge_HW',
            executable='map_expansion_node',
            name=f'map_expansion_{robot_name}_phantom',
            namespace='',
            parameters=[{
                'input_topic':      f'/phantom_map_never_published_{robot_name}',
                'output_topic':     f'/{robot_name}/map_fixed',
                'robot_offset_x':   0.0,
                'robot_offset_y':   0.0,
                'canvas_width':     CANVAS_WIDTH,
                'canvas_height':    CANVAS_HEIGHT,
                'canvas_resolution': CANVAS_RESOLUTION,
                'canvas_origin_x':  CANVAS_ORIGIN_X,
                'canvas_origin_y':  CANVAS_ORIGIN_Y,
                'publish_rate_hz':  1.0,
            }],
            output='screen',
        )
        ld.add_action(phantom_node)

    # ── Stage 3: Central map merge (delayed to allow ArUco convergence) ──────
    map_merge_node = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg='[ArUco] Starting multirobot_map_merge...'),
            Node(
                package='multirobot_map_merge',
                executable='map_merge',
                name='map_merge',
                namespace='',
                parameters=[params_file],
                output='screen',
            ),
        ]
    )
    ld.add_action(map_merge_node)

    return ld
```

---

## 8. Workflow Summary — SetParameters Chain

This is the complete data flow from camera pixel to canvas alignment.

```
──────────────────────────────────────────────────────────────────────────
HARDWARE → DETECTION → COMPUTATION → PARAMETER SERVICE → C++ UPDATE
──────────────────────────────────────────────────────────────────────────

1. OAK camera publishes:
   /oak_camera/image_raw  (sensor_msgs/Image)
   /oak_camera/camera_info  (sensor_msgs/CameraInfo)

2. aruco_detector_node.py:
   - Reads camera_info to get intrinsic matrix K and distortion coeffs d.
   - On each image frame, runs cv2.aruco.ArucoDetector.detectMarkers().
   - If a marker is visible within max_detection_distance_m:
       Calls cv2.aruco.estimatePoseSingleMarkers() → rvec, tvec
       Publishes:
         /ausra_1/aruco/marker_id       (std_msgs/Int32)
         /ausra_1/aruco/detected_marker (geometry_msgs/PoseStamped)
           - position: camera-frame XYZ from camera to marker
           - orientation: quaternion of marker relative to camera

3. ausra_pose_initialiser.py:
   - Subscribes to both topics above.
   - For each detection:
       Looks up marker['global_x'], marker['global_y'], marker['global_yaw']
       from aruco_markers.yaml.
       Computes:
         dx = pose.position.x  (camera-frame lateral offset)
         dz = pose.position.z  (camera-frame depth = forward distance)
         robot_x = marker_global_x
                   - (dz * cos(marker_yaw) - dx * sin(marker_yaw))
         robot_y = marker_global_y
                   - (dz * sin(marker_yaw) + dx * cos(marker_yaw))
       Appends (robot_x, robot_y) to samples list.
   - After convergence_samples detections:
       avg_x = mean(samples_x)
       avg_y = mean(samples_y)
       Calls: /map_expansion_ausra_1/set_parameters
         robot_offset_x = avg_x
         robot_offset_y = avg_y

4. map_expansion_node.cpp — parameter callback fires:
   - Updates robot_offset_x_ and robot_offset_y_ member variables.
   - Acquires canvas_mutex_.
   - Clears canvas_data_ (removes all cells stamped at the old (0,0) offset).
   - Clears last_written_indices_.
   - Releases mutex.
   - Logs: "[ArUco] Canvas cleared — new offset applied."

5. Next slam_toolbox map message arrives at mapCallback():
   - Reads updated robot_offset_x_ (e.g. 3.4194).
   - Computes: global_origin_x = local_origin_x + 3.4194
   - Computes: canvas_offset_x = (global_origin_x - (-25.0)) / 0.05
   - Stamps cells at globally-correct canvas positions.
   - Heartbeat timer publishes the now-correct canvas to /ausra_1/map_fixed.

6. multirobot_map_merge overlays /ausra_1/map_fixed + /ausra_2/map_fixed
   → publishes /map_merged with correct spatial alignment.

──────────────────────────────────────────────────────────────────────────
init_pose_* in map_merge_HW_params.yaml: 0.0 throughout — never changed.
──────────────────────────────────────────────────────────────────────────
```

---

## 9. Topic and Frame Corrections vs the Guide

The following deviations from `ArUco_Fiducials_Implementation_Guide.md`
are applied in the code above and must not be reverted.

| Item | Guide (incorrect for hardware) | This document (corrected) | Reason |
|---|---|---|---|
| Camera image topic | `/{robot_name}/oak_camera/image_raw` | `/oak_camera/image_raw` (via parameter) | Hardware stack has no namespace |
| Camera info topic | `/{robot_name}/oak_camera/camera_info` | `/oak_camera/camera_info` (via parameter) | Hardware stack has no namespace |
| Camera optical frame | `{robot_name}_camera_link_optical` | `ausrabot_camera_link_optical` (via parameter) | Hardware uses `ausrabot_*` TF prefix |
| Quaternion conversion | `from scipy.spatial.transform import Rotation` | Pure `math`/`numpy` Shepperd method | scipy not declared as dependency; would fail silently at import |
| Expansion node package | `ausra_map_merge` | `ausra_map_merge_HW` | Guide references old package; executable lives here |
| C++ parameter update | Not implemented (silent failure) | `add_on_set_parameters_callback` in constructor | Without this, `SetParameters` has no effect on the canvas math |

---

## Verification Commands (After Build)

```bash
# 1. Confirm Python scripts are installed and executable
ros2 run ausra_map_merge_HW aruco_detector_node.py --ros-args -p robot_name:=ausra_1

# 2. Confirm SetParameters service is visible once expansion node runs
ros2 service list | grep map_expansion

# 3. Confirm ArUco topics appear when detector is running
ros2 topic list | grep aruco

# 4. Manually verify marker detection (standalone, no SLAM needed)
ros2 launch ausra_map_merge_HW aruco_init.launch.py robot_name:=ausra_1

# 5. Check that offset was applied to the C++ node
ros2 param get /map_expansion_ausra_1 robot_offset_x
# Expected after convergence: the measured distance value, not 0.0
```
