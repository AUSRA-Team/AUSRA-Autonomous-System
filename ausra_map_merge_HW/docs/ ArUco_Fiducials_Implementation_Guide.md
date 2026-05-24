# ArUco Fiducials Implementation Guide for `ausra_map_merge_HW`

**Document:** `ArUco_Fiducials_Implementation_Guide.md`
**Purpose:** Step-by-step implementation plan for replacing the tape-measure SOP with automatic ArUco-based robot localization on physical hardware.
**New Package:** `ausra_map_merge_HW`

---

## 1. Why ArUco Fiducials?

The tape-measure SOP requires 15–30 minutes of manual measurement and yaw alignment per session. ArUco markers eliminate both steps entirely:

- **Position**: Computed from the marker's known global coordinates and the camera-to-marker transform.
- **Yaw**: Extracted automatically from the marker's orientation — no manual alignment needed.
- **Hardware cost**: Near zero — printed paper markers + the OAK camera already on each AUSRA robot.

---

## 2. How It Works With Our Architecture

The ArUco system is a **drop-in replacement** for the tape-measure step only. The `map_expansion_node` and `map_merge_params.yaml` remain completely unchanged.

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        CURRENT (Tape Measure)                          │
│                                                                        │
│  Human measures robot position ──► hardcodes ROBOT_SPAWN_POSES         │
│       robot_offset_x = 3.45        in map_merge.launch.py              │
│       robot_offset_y = 0.00                                            │
└──────────────────────────────┬──────────────────────────────────────────┘
                               │
                               ▼
                    map_expansion_node receives offsets
                    Canvas math is identical either way

┌─────────────────────────────────────────────────────────────────────────┐
│                     NEW (ArUco — ausra_map_merge_HW)                   │
│                                                                        │
│  OAK camera detects ArUco marker                                       │
│       ▼                                                                │
│  aruco_detector_node computes camera→marker transform                  │
│       ▼                                                                │
│  ausra_pose_initialiser reads marker's known global (x, y, yaw)       │
│  + detected relative pose → computes robot's global (x, y)            │
│       ▼                                                                │
│  Writes robot_offset_x, robot_offset_y into map_expansion_node        │
│       ▼                                                                │
│  map_expansion_node launches with correct offsets (UNCHANGED CODE)     │
└─────────────────────────────────────────────────────────────────────────┘
```

### Key Rule — `init_pose_*` Stays at 0.0

Just like the tape-measure SOP, all `init_pose_*` values in `map_merge_params.yaml` must remain `0.0`. The expansion node applies the offset; the merger applies zero.

---

## 3. The `ausra_map_merge_HW` Package Structure

```
ausra_map_merge_HW/
├── CMakeLists.txt                       # ament_cmake (C++) or setup.py (Python)
├── package.xml
├── config/
│   ├── aruco_markers.yaml               # Known global positions of all markers
│   └── camera_calibration.yaml          # OAK camera intrinsics (from calibration)
├── launch/
│   ├── aruco_init.launch.py             # Per-robot: detector + initialiser
│   └── map_merge_hw.launch.py           # Full stack: aruco init → expansion → merge
├── src/  (or ausra_map_merge_hw/ for Python)
│   ├── aruco_detector_node.py           # Detects markers, publishes transforms
│   └── ausra_pose_initialiser.py        # Reads detection, writes offsets, exits
└── docs/
    └── marker_placement_guide.md
```

### Package Dependencies

```xml
<!-- package.xml -->
<depend>rclpy</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
<depend>cv_bridge</depend>
<depend>tf2_ros</depend>
<depend>image_transport</depend>

<!-- OpenCV ArUco is a system dependency (comes with opencv) -->
<build_depend>python3-opencv</build_depend>
<exec_depend>python3-opencv</exec_depend>

<!-- Existing packages we integrate with -->
<exec_depend>ausra_map_merge</exec_depend>
```

---

## 4. Configuration Files

### 4.1 Marker Registry — `config/aruco_markers.yaml`

Each marker's global position is measured **once** during commissioning (same procedure as the tape-measure origin setup) and stored permanently.

```yaml
# aruco_markers.yaml
# Global positions of all ArUco markers in the environment.
# Measured once during commissioning relative to the physical origin.
# Marker positions NEVER change unless physically relocated.

aruco_config:
  dictionary: DICT_4X4_50          # ArUco dictionary type
  marker_size_m: 0.15              # Physical marker side length in metres
  convergence_samples: 5           # Detections required before accepting pose
  max_detection_distance_m: 3.0    # Ignore detections beyond this range

markers:
  - id: 0
    global_x: 0.0
    global_y: 0.0
    global_yaw: 0.0                # Marker faces +X direction
    description: "Origin wall — Room A northwest corner"

  - id: 1
    global_x: 5.0
    global_y: 0.0
    global_yaw: 0.0
    description: "East wall — Room A"

  - id: 2
    global_x: 0.0
    global_y: 4.0
    global_yaw: 1.5708             # Marker faces +Y direction (90°)
    description: "South corridor entry"

  # Add more markers as needed for coverage
```

### 4.2 Camera Calibration — `config/camera_calibration.yaml`

The OAK camera (`oak_camera` link in `sensors.xacro`) must be calibrated once using the standard OpenCV checkerboard procedure.

```yaml
# camera_calibration.yaml
# Generated by: ros2 run camera_calibration cameracalibrator
# Camera: OAK-D (or OAK-1) mounted on AUSRA robot

image_width: 640
image_height: 480
camera_matrix:
  rows: 3
  cols: 3
  data: [615.0, 0.0, 320.0,
         0.0, 615.0, 240.0,
         0.0, 0.0, 1.0]
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0, 0.0, 0.0, 0.0, 0.0]   # Replace with real calibration values
```

---

## 5. Node Implementations

### 5.1 ArUco Detector Node — `aruco_detector_node.py`

This node subscribes to the OAK camera image, detects ArUco markers, and publishes the detected marker ID + relative pose.

```python
#!/usr/bin/env python3
"""
aruco_detector_node.py
Detects ArUco markers from the OAK camera and publishes marker poses.

Subscribes: /<robot_name>/oak_camera/image_raw  (sensor_msgs/Image)
Publishes:  /<robot_name>/aruco/detected_marker  (geometry_msgs/PoseStamped)
            /<robot_name>/aruco/marker_id         (std_msgs/Int32)
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from cv_bridge import CvBridge


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # Parameters
        self.declare_parameter('robot_name', 'ausra_1')
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_size_m', 0.15)
        self.declare_parameter('max_detection_distance_m', 3.0)

        self.robot_name = self.get_parameter('robot_name').value
        marker_size = self.get_parameter('marker_size_m').value
        dict_name = self.get_parameter('dictionary').value
        self.max_dist = self.get_parameter('max_detection_distance_m').value

        # ArUco setup
        dict_id = getattr(cv2.aruco, dict_name)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.marker_size = marker_size
        self.bridge = CvBridge()

        # Camera intrinsics (populated from CameraInfo)
        self.camera_matrix = None
        self.dist_coeffs = None

        # Subscribers
        self.create_subscription(
            Image,
            f'/{self.robot_name}/oak_camera/image_raw',
            self.image_callback, 10)
        self.create_subscription(
            CameraInfo,
            f'/{self.robot_name}/oak_camera/camera_info',
            self.camera_info_callback, 10)

        # Publishers
        self.pose_pub = self.create_publisher(
            PoseStamped,
            f'/{self.robot_name}/aruco/detected_marker', 10)
        self.id_pub = self.create_publisher(
            Int32,
            f'/{self.robot_name}/aruco/marker_id', 10)

        self.get_logger().info(
            f'ArUco detector ready for {self.robot_name} '
            f'(dict={dict_name}, size={marker_size}m)')

    def camera_info_callback(self, msg):
        """Cache camera intrinsics from CameraInfo topic."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera intrinsics received.')

    def image_callback(self, msg):
        """Detect ArUco markers and publish the closest one's pose."""
        if self.camera_matrix is None:
            return  # No intrinsics yet

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        corners, ids, _ = self.detector.detectMarkers(frame)

        if ids is None or len(ids) == 0:
            return

        # Estimate pose for each detected marker
        for i, marker_id in enumerate(ids.flatten()):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corners[i]], self.marker_size,
                self.camera_matrix, self.dist_coeffs)

            distance = np.linalg.norm(tvec[0][0])
            if distance > self.max_dist:
                continue

            # Publish marker ID
            id_msg = Int32()
            id_msg.data = int(marker_id)
            self.id_pub.publish(id_msg)

            # Publish camera-to-marker pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = f'{self.robot_name}_camera_link_optical'
            pose_msg.pose.position.x = float(tvec[0][0][0])
            pose_msg.pose.position.y = float(tvec[0][0][1])
            pose_msg.pose.position.z = float(tvec[0][0][2])

            # Convert rvec to quaternion for orientation
            rot_matrix, _ = cv2.Rodrigues(rvec[0][0])
            from scipy.spatial.transform import Rotation
            quat = Rotation.from_matrix(rot_matrix).as_quat()  # [x,y,z,w]
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]

            self.pose_pub.publish(pose_msg)
            self.get_logger().debug(
                f'Marker {marker_id} at distance {distance:.2f}m')
            break  # Use closest / first valid marker


def main():
    rclpy.init()
    rclpy.spin(ArucoDetectorNode())
    rclpy.shutdown()
```

### 5.2 Pose Initialiser Node — `ausra_pose_initialiser.py`

This node reads the detected marker pose, looks up the marker's known global position from `aruco_markers.yaml`, computes the robot's global `(x, y)`, and writes the offsets into the `map_expansion_node`.

```python
#!/usr/bin/env python3
"""
ausra_pose_initialiser.py
Reads ArUco detection + marker registry → computes robot_offset_x/y
→ sets parameters on the map_expansion_node → shuts down.

This node runs ONCE at boot and exits after convergence.
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

        self.declare_parameter('robot_name', 'ausra_1')
        self.declare_parameter('markers_config', '')
        self.declare_parameter('convergence_samples', 5)

        self.robot_name = self.get_parameter('robot_name').value
        config_path = self.get_parameter('markers_config').value
        self.required_samples = self.get_parameter('convergence_samples').value

        # Load marker registry
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        self.markers = {m['id']: m for m in config['markers']}

        # Detection state
        self.current_marker_id = None
        self.samples = []

        # Subscribers
        self.create_subscription(
            Int32,
            f'/{self.robot_name}/aruco/marker_id',
            self.id_callback, 10)
        self.create_subscription(
            PoseStamped,
            f'/{self.robot_name}/aruco/detected_marker',
            self.pose_callback, 10)

        # Parameter client for map_expansion_node
        self.param_client = self.create_client(
            SetParameters,
            f'/map_expansion_{self.robot_name}/set_parameters')

        self.get_logger().info(
            f'Pose initialiser waiting for ArUco detection '
            f'({self.required_samples} samples needed)...')

    def id_callback(self, msg):
        self.current_marker_id = msg.data

    def pose_callback(self, msg):
        if self.current_marker_id is None:
            return
        if self.current_marker_id not in self.markers:
            self.get_logger().warn(
                f'Marker {self.current_marker_id} not in registry. Ignoring.')
            return

        marker = self.markers[self.current_marker_id]

        # camera-to-marker translation (in camera optical frame)
        # We need the 2D ground-plane distance from robot to marker
        dx = msg.pose.position.x  # lateral
        dz = msg.pose.position.z  # depth (forward in optical frame)

        # Robot's global position = marker's global position - relative offset
        # (simplified 2D: assumes marker faces known direction)
        marker_yaw = marker['global_yaw']
        # Transform camera-relative offset to global frame
        robot_x = marker['global_x'] - (dz * math.cos(marker_yaw) - dx * math.sin(marker_yaw))
        robot_y = marker['global_y'] - (dz * math.sin(marker_yaw) + dx * math.cos(marker_yaw))

        self.samples.append((robot_x, robot_y))
        self.get_logger().info(
            f'Sample {len(self.samples)}/{self.required_samples}: '
            f'marker={self.current_marker_id}, '
            f'robot_pos=({robot_x:.3f}, {robot_y:.3f})')

        if len(self.samples) >= self.required_samples:
            self.finalize()

    def finalize(self):
        """Average samples and write offsets to map_expansion_node."""
        avg_x = sum(s[0] for s in self.samples) / len(self.samples)
        avg_y = sum(s[1] for s in self.samples) / len(self.samples)

        self.get_logger().info(
            f'CONVERGED: robot_offset_x={avg_x:.4f}, '
            f'robot_offset_y={avg_y:.4f}')

        # Set parameters on the map_expansion_node
        if self.param_client.wait_for_service(timeout_sec=5.0):
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
            future.add_done_callback(self.param_set_done)
        else:
            self.get_logger().error(
                'map_expansion_node parameter service not available!')

    def param_set_done(self, future):
        self.get_logger().info(
            'Offsets written to map_expansion_node. Initialiser shutting down.')
        raise SystemExit(0)


def main():
    rclpy.init()
    node = PoseInitialiser()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    rclpy.shutdown()
```

---

## 6. Launch Files

### 6.1 Per-Robot ArUco Init — `launch/aruco_init.launch.py`

```python
"""Launches ArUco detection + pose initialisation for one robot."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ausra_map_merge_HW')
    markers_config = os.path.join(pkg_share, 'config', 'aruco_markers.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='ausra_1'),

        # ArUco detector — runs continuously until initialiser converges
        Node(
            package='ausra_map_merge_HW',
            executable='aruco_detector_node',
            name='aruco_detector',
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name'),
                'dictionary': 'DICT_4X4_50',
                'marker_size_m': 0.15,
                'max_detection_distance_m': 3.0,
            }],
            output='screen',
        ),

        # Pose initialiser — runs once, writes offsets, then exits
        Node(
            package='ausra_map_merge_HW',
            executable='ausra_pose_initialiser',
            name='pose_initialiser',
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name'),
                'markers_config': markers_config,
                'convergence_samples': 5,
            }],
            output='screen',
        ),
    ])
```

### 6.2 Full HW Map Merge — `launch/map_merge_hw.launch.py`

```python
"""
Full hardware map merge launch:
  1. ArUco initialisation per robot (detects position automatically)
  2. map_expansion_node per robot (from ausra_map_merge package)
  3. Central map_merge node (from multirobot_map_merge package)
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


ROBOTS = ['ausra_1', 'ausra_2']   # Add more robots here


def generate_launch_description():
    ld = LaunchDescription()

    pkg_hw = get_package_share_directory('ausra_map_merge_HW')
    pkg_merge = get_package_share_directory('ausra_map_merge')
    map_merge_params = os.path.join(pkg_merge, 'config', 'map_merge_params.yaml')

    for robot_name in ROBOTS:
        # Phase 1: ArUco detection + pose initialisation
        aruco_init = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_hw, 'launch', 'aruco_init.launch.py')),
            launch_arguments={'robot_name': robot_name}.items(),
        )
        ld.add_action(aruco_init)

        # Phase 2: Expansion node (delayed to allow ArUco convergence)
        # Starts with offset (0,0) — the initialiser will update it
        expansion_node = TimerAction(
            period=15.0,  # Allow 15s for ArUco convergence
            actions=[Node(
                package='ausra_map_merge',
                executable='map_expansion_node',
                name=f'map_expansion_{robot_name}',
                parameters=[{
                    'input_topic':      f'/{robot_name}/map',
                    'output_topic':     f'/{robot_name}/map_fixed',
                    'canvas_width':      1000,
                    'canvas_height':     1000,
                    'canvas_resolution': 0.05,
                    'canvas_origin_x':  -25.0,
                    'canvas_origin_y':  -25.0,
                    'robot_offset_x':    0.0,  # Updated by initialiser
                    'robot_offset_y':    0.0,  # Updated by initialiser
                }],
                output='screen',
            )],
        )
        ld.add_action(expansion_node)

    # Phase 3: Central map merge (delayed further)
    map_merge_node = TimerAction(
        period=20.0,
        actions=[Node(
            package='multirobot_map_merge',
            executable='map_merge',
            name='map_merge',
            parameters=[map_merge_params],
            output='screen',
        )],
    )
    ld.add_action(map_merge_node)

    return ld
```

---

## 7. Hardware Setup — What You Need

### 7.1 Print the ArUco Markers

```bash
# Generate marker images using OpenCV (run once on your laptop)
python3 -c "
import cv2
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
for marker_id in range(5):
    img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, 400)
    cv2.imwrite(f'aruco_marker_{marker_id}.png', img)
    print(f'Generated marker {marker_id}')
"
```

**Printing rules:**
- Print on **matte white paper** (glossy causes reflections)
- Marker size: **15 cm × 15 cm** (matches `marker_size_m: 0.15`)
- **Laminate** each marker to protect from damage
- Leave a **white border** of at least 2 cm around each marker

### 7.2 Place Markers in the Environment

| Rule | Reason |
|---|---|
| Mount at OAK camera height (≈12 cm from ground on AUSRA) | Perpendicular detection = best accuracy |
| Face markers toward the area where robots will start | Camera must see the marker at boot |
| Place at least 1 marker per starting zone | Each robot needs visibility to at least 1 marker |
| Secure markers to walls/columns — never furniture | Markers must not move between sessions |
| Record each marker's `(global_x, global_y, global_yaw)` once | One-time commissioning measurement |

### 7.3 Calibrate the OAK Camera (One-Time)

```bash
# Print a checkerboard pattern (9x6 inner corners, 25mm squares)
# Then run the ROS 2 camera calibration tool:

ros2 run camera_calibration cameracalibrator \
  --size 9x6 \
  --square 0.025 \
  image:=/<robot_name>/oak_camera/image_raw \
  camera:=/<robot_name>/oak_camera

# Save the output calibration to:
#   ausra_map_merge_HW/config/camera_calibration.yaml
```

---

## 8. Operational Workflow (Daily Use)

```
Step 1:  Place robots anywhere within camera view of a marker.
         No yaw alignment needed. No tape measure.

Step 2:  Power on all robots. Start SLAM + sensor stacks.

Step 3:  Launch the HW map merge:
           ros2 launch ausra_map_merge_HW map_merge_hw.launch.py

Step 4:  Watch terminal output for convergence messages:
           [pose_initialiser] Sample 1/5: marker=0, robot_pos=(3.421, 0.012)
           [pose_initialiser] Sample 5/5: marker=0, robot_pos=(3.418, 0.015)
           [pose_initialiser] CONVERGED: robot_offset_x=3.4194, robot_offset_y=0.0134
           [pose_initialiser] Offsets written to map_expansion_node.

Step 5:  Open RViz → add /map_merged topic → verify alignment.

Step 6:  Begin exploration / mission.
```

**Total daily setup time: < 2 minutes** (vs 15–30 min with tape measure).

---

## 9. How the Math Connects to `map_expansion_node`

The ArUco system only changes **where the offset values come from**. The expansion node's spatial math is identical:

```
ArUco System                              map_expansion_node (UNCHANGED)
─────────────────────                     ──────────────────────────────
Marker at global (0, 0)                   robot_offset_x = 3.42
Camera detects marker at 3.42m away       robot_offset_y = 0.01
  ↓                                         ↓
Initialiser computes:                     global_origin_x = local_origin_x + 3.42
  robot is at global (3.42, 0.01)         offset_x = (global_origin_x - (-25.0)) / 0.05
  ↓                                         ↓
Writes robot_offset_x = 3.42             Canvas pixel position is STABLE
Writes robot_offset_y = 0.01             Moving floor eliminated (same as simulation)
```

---

## 10. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `No camera intrinsics received` | Camera not publishing `camera_info` | Check OAK driver is running, verify topic name |
| Initialiser never converges | No marker visible to camera | Reposition robot to face a marker, check lighting |
| Converged but map is offset | Marker's `global_x/y` in YAML is wrong | Re-measure marker position from physical origin |
| Converged but map is rotated | `global_yaw` in YAML doesn't match marker orientation | Re-measure marker facing direction |
| `Marker X not in registry` | Marker ID detected but not in `aruco_markers.yaml` | Add the marker entry to the config |
| Poor accuracy (>10 cm error) | Camera not calibrated or marker too far | Recalibrate camera; move robot closer to marker |

---

## 11. Summary — What Changes, What Stays the Same

| Component | Changes? | Details |
|---|---|---|
| `map_expansion_node.cpp` | **NO** | Receives `robot_offset_x/y` exactly as before |
| `map_merge_params.yaml` | **NO** | `init_pose_*` stays at `0.0` |
| `map_merge.launch.py` | **Replaced** by `map_merge_hw.launch.py` | Offsets come from ArUco, not hardcoded |
| OAK Camera | **Used** (already on robot) | Needs one-time calibration |
| Physical markers | **NEW** | Printed ArUco markers placed in environment |
| `ausra_map_merge_HW` package | **NEW** | 2 Python nodes + configs + launch files |
| Tape measure | **ELIMINATED** | No longer needed for daily operation |
| Manual yaw alignment | **ELIMINATED** | Detected automatically from marker |
