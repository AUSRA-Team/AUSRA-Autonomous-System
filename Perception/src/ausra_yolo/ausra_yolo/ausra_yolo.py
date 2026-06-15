import os
import traceback
import math
import numpy as np
import rclpy
import tf2_ros
import tf2_geometry_msgs
from rclpy.node import Node
from rclpy.duration import Duration as RclpyDuration
from builtin_interfaces.msg import Duration as MsgDuration


from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion, Pose
from visualization_msgs.msg import Marker, MarkerArray
from message_filters import Subscriber, ApproximateTimeSynchronizer
from ultralytics import YOLO


class AusraYoloNode(Node):

    # ──────────────────────────────────────────────────────────────────────────
    # Lifecycle
    # ──────────────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('ausra_yolo_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('robot_name', 'ausra_1')
        self.declare_parameter('model_path', '~/ausra_full_system/src/AUSRA-Autonomous-System/'
                                              'Perception/weights/best.engine')
        self.declare_parameter('confidence',      0.7)
        self.declare_parameter('image_size',      640)
        self.declare_parameter('use_gpu',         True)
        self.declare_parameter('use_sim',         True)
        self.declare_parameter('dedup_radius_m',  0.5)   # min distance between unique victims

        self._read_parameters()

        # ── YOLO model ────────────────────────────────────────────────────────
        self.get_logger().info(f'Loading YOLO model from: {self.model_path}')
        self.model = YOLO(self.model_path)
        self.get_logger().info('YOLO model loaded successfully')

        # ── Utilities ─────────────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── Cached / stateful data ─────────────────────────────────────────────
        self.camera_info     = None   # CameraInfo msg — populated on first receipt
        self._marker_counter = 0      # monotonic RViz marker ID (never resets)

        # Known victim positions in map frame: list of (x, y) tuples.
        # Used to suppress duplicate detections of the same victim.
        self._known_victims: list[tuple[float, float]] = []

        # ── TF2 ───────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscribers / Publishers ───────────────────────────────────────────
        self._create_subscribers()
        self._create_publishers()

        self.get_logger().info(
            f'[{self.robot_name}] Node ready  |  '
            f'RGB → {self.rgb_topic}  |  '
            f'Depth → {self.depth_topic}'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Setup helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _read_parameters(self):
        """Pull all ROS parameters into instance attributes."""
        self.robot_name      = self.get_parameter('robot_name').value
        self.model_path      = os.path.expanduser(self.get_parameter('model_path').value)
        self.confidence      = self.get_parameter('confidence').value
        self.image_size      = self.get_parameter('image_size').value
        self.use_gpu         = self.get_parameter('use_gpu').value
        self.use_sim         = self.get_parameter('use_sim').value
        self._dedup_radius_m = self.get_parameter('dedup_radius_m').value

        # Sync / TF tuning: simulation needs more tolerance than real hardware
        self._sync_slop       = 0.10 if self.use_sim else 0.05
        self._sync_queue_size = 25   if self.use_sim else 10
        self._tf_timeout_s    = 0.30 if self.use_sim else 0.20
        self._depth_max_m     = 15.0 if self.use_sim else 8.0

        # Topic names — built entirely from robot_name so they stay in sync
        ns = f'/{self.robot_name}'
        if self.use_sim:
            self.rgb_topic         = f'{ns}/{self.robot_name}_rgb_camera/image_raw'
            self.depth_topic       = f'{ns}/{self.robot_name}_depth_camera/depth/image_raw'
            self.camera_info_topic = f'{ns}/{self.robot_name}_depth_camera/depth/camera_info'
        else:
            # OAK-D Lite — adjust to match your exact DepthAI launch file
            self.rgb_topic         = f'{ns}/camera/rgb/image_raw'
            self.depth_topic       = f'{ns}/camera/stereo/image_raw'
            self.camera_info_topic = f'{ns}/camera/rgb/camera_info'

    def _create_subscribers(self):
        # CameraInfo: needed once to cache the K matrix; kept alive for frame_id
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_cb,
            10,
        )

        # Synchronised RGB + Depth pair
        self.rgb_sub   = Subscriber(self, Image, self.rgb_topic)
        self.depth_sub = Subscriber(self, Image, self.depth_topic)

        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=self._sync_queue_size,
            slop=self._sync_slop,
        )
        self.ts.registerCallback(self._sync_callback)

    def _create_publishers(self):
        yolo_ns   = f'/{self.robot_name}/yolo'
        robot_ns  = f'/{self.robot_name}'

        self.detection_pub   = self.create_publisher(Detection2DArray, f'{yolo_ns}/detections',   10)
        self.debug_image_pub = self.create_publisher(Image,            f'{yolo_ns}/debug_image',  10)
        self.marker_pub      = self.create_publisher(MarkerArray,      f'{robot_ns}/victim_markers',  10)
        self.waypoint_pub    = self.create_publisher(PoseStamped,      f'{robot_ns}/victim_waypoint', 10)

    # ──────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────────

    def _camera_info_cb(self, msg: CameraInfo):
        """Cache intrinsics on first receipt and log once so we know it arrived."""
        if self.camera_info is None:
            K = msg.k
            self.get_logger().info(
                f'CameraInfo received — '
                f'fx={K[0]:.1f}  fy={K[4]:.1f}  '
                f'cx={K[2]:.1f}  cy={K[5]:.1f}  '
                f'frame_id="{msg.header.frame_id}"'
            )
        self.camera_info = msg

    def _sync_callback(self, rgb_msg: Image, depth_msg: Image):
        """
        Main pipeline — fires when a matched (RGB, Depth) pair arrives.

        Steps
        ─────
        1. Decode both images into OpenCV / NumPy
        2. Run YOLO tracking on the RGB frame
        3. For each detection:
             a. Depth lookup at centroid           → Z
             b. Back-project (u, v, Z)             → camera-frame (Xc, Yc, Zc)
             c. TF2 transform                      → map-frame PointStamped
             d. Deduplication check
             e. Publish RViz marker + Nav2 waypoint
        4. Publish Detection2DArray + annotated debug image
        """
        if self.camera_info is None:
            self.get_logger().warn(
                'CameraInfo not yet received — skipping frame.',
                throttle_duration_sec=5.0,
            )
            return

        try:
            # ── 1. Decode ─────────────────────────────────────────────────────
            cv_image  = self.bridge.imgmsg_to_cv2(rgb_msg,   desired_encoding='bgr8')
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

            depth_m = self._to_depth_metres(depth_raw, depth_msg.encoding)
            if depth_m is None:
                return

            # ── 2. YOLO inference ─────────────────────────────────────────────
            device  = 0 if self.use_gpu else 'cpu'
            results = self.model.track(
                cv_image,
                conf=self.confidence,
                imgsz=self.image_size,
                persist=True,
                tracker='bytetrack.yaml',
                verbose=False,
                device=device,
                half=self.use_gpu,   # FP16 only on GPU
            )

            # ── 3. Per-detection 3-D localisation ────────────────────────────
            detection_array        = Detection2DArray()
            detection_array.header = rgb_msg.header

            # Cache K matrix outside the loop — it's the same for every box
            K          = self.camera_info.k
            fx, fy     = K[0], K[4]
            cx, cy     = K[2], K[5]
            cam_frame = f'{self.robot_name}_camera_link_optical'

            marker_array = MarkerArray()

            for box in results[0].boxes:

                # Build and store the 2D detection regardless of depth success
                detection = self._build_detection(box)
                detection_array.detections.append(detection)

                # Early exit — non-victims need no 3D processing whatsoever
                label = detection.results[0].hypothesis.class_id
                if label != 'Victim':
                    continue    # ← jumps straight to next box

                # ── a. Depth lookup ───────────────────────────────────────────
                # xywh gives (centre_x, centre_y, width, height)
                u = int(box.xywh[0][0].item())
                v = int(box.xywh[0][1].item())
                Z = self._sample_depth(depth_m, u, v)
                if Z is None:
                    self.get_logger().debug(
                        f'No valid depth at ({u},{v}) for track {detection.id} — skipping 3D.'
                    )
                    continue

                # ── b. Back-project to camera frame ───────────────────────────
                Xc = (u - cx) * Z / fx
                Yc = (v - cy) * Z / fy
                Zc = Z

                # ── c. TF2: camera frame → map ────────────────────────────────
                point_map = self._transform_to_map(Xc, Yc, Zc, rgb_msg.header, cam_frame)
                if point_map is None:
                    continue
        

                # ── d. Deduplication ──────────────────────────────────────────
                mx, my = point_map.point.x, point_map.point.y
                if self._is_known_victim(mx, my):
                    continue

                # New unique victim — register it
                self._known_victims.append((mx, my))
                self.get_logger().info(
                    f'New victim [{label}] registered at map '
                    f'({mx:.2f}, {my:.2f}, {point_map.point.z:.2f})  '
                    f'depth={Z:.2f} m  —  total found: {len(self._known_victims)}'
                )

                # ── e. Publish 3D outputs ─────────────────────────────────────   
                marker = self._build_marker(point_map, label)
                marker_array.markers.append(marker)
                robot_pose = self._get_robot_pose()
                self._publish_waypoint(point_map, robot_pose)

            # ── 4. Publish 2D outputs ──────────────────────────────────────────
            self.detection_pub.publish(detection_array)

            if marker_array.markers:
                self.marker_pub.publish(marker_array)

            annotated        = results[0].plot()
            debug_msg        = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            debug_msg.header = rgb_msg.header
            self.debug_image_pub.publish(debug_msg)

            self.get_logger().debug(
                f'Published {len(detection_array.detections)} detections | '
                f'Victims found so far: {len(self._known_victims)}'
            )

        except Exception:
            self.get_logger().error(f'Error in _sync_callback:\n{traceback.format_exc()}')

    # ──────────────────────────────────────────────────────────────────────────
    # Helper methods
    # ──────────────────────────────────────────────────────────────────────────

    def _to_depth_metres(self, raw: np.ndarray, encoding: str) -> np.ndarray | None:
        """
        Normalise a raw depth image to float32 metres.

          32FC1          → Gazebo (already metres, float32)
          16UC1 / mono16 → OAK-D Lite and some Gazebo plugins (millimetres, uint16)
        """
        if encoding == '32FC1':
            return raw.astype(np.float32)
        elif encoding in ('16UC1', 'mono16'):
            return raw.astype(np.float32) / 1000.0
        else:
            self.get_logger().error(
                f'Unsupported depth encoding "{encoding}" — expected 32FC1 or 16UC1.',
                throttle_duration_sec=10.0,
            )
            return None

    def _sample_depth(self, depth_m: np.ndarray, u: int, v: int) -> float | None:
        """
        Robust depth at pixel (u, v): median of a valid 7×7 patch.
        Returns None when no valid readings exist within the patch.
        """
        h, w  = depth_m.shape
        u     = int(np.clip(u, 0, w - 1))
        v     = int(np.clip(v, 0, h - 1))
        patch = depth_m[max(0, v - 3):v + 4, max(0, u - 3):u + 4]
        valid = patch[np.isfinite(patch) & (patch > 0.3) & (patch < self._depth_max_m)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _build_detection(self, box) -> Detection2D:
        """Convert a single YOLO box into a Detection2D message."""
        detection    = Detection2D()
        detection.id = str(int(box.id[0].item())) if box.id is not None else 'unknown'

        bx, by, bw, bh = box.xywh[0].cpu().numpy()
        detection.bbox.center.position.x = float(bx)
        detection.bbox.center.position.y = float(by)
        detection.bbox.size_x            = float(bw)
        detection.bbox.size_y            = float(bh)

        hyp                     = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(self.model.names[int(box.cls[0])])
        hyp.hypothesis.score    = float(box.conf[0])
        detection.results.append(hyp)

        return detection

    def _transform_to_map(
        self,
        Xc: float, Yc: float, Zc: float,
        header,
        cam_frame: str,
    ) -> PointStamped | None:
        """
        Transform a 3-D point from the camera optical frame into the map frame.

        Uses the image capture timestamp so the TF lookup matches the robot pose
        at the exact moment the frame was taken — not 'now'.
        Returns None on any TF failure (tree not ready, SLAM lost, etc.).
        """
        point_cam              = PointStamped()
        point_cam.header.stamp = header.stamp
        point_cam.header.frame_id = cam_frame
        point_cam.point.x      = float(Xc)
        point_cam.point.y      = float(Yc)
        point_cam.point.z      = float(Zc)

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                cam_frame,
                rclpy.time.Time.from_msg(header.stamp),
                RclpyDuration(seconds=self._tf_timeout_s),   # use tuned timeout
            )
            return tf2_geometry_msgs.do_transform_point(point_cam, transform)

        except Exception as e:
            self.get_logger().debug(f'TF2 transform failed (normal on startup): {e}')
            return None

    def _is_known_victim(self, x: float, y: float) -> bool:
        """
        Return True if (x, y) is within dedup_radius_m of any already-registered
        victim. Squared-distance comparison avoids a sqrt per candidate.
        """
        r2 = self._dedup_radius_m ** 2
        return any((x - vx) ** 2 + (y - vy) ** 2 < r2
                   for vx, vy in self._known_victims)
    
    def _get_robot_pose(self):
        """Get current robot position in map frame."""
        try:
            tf = self.tf_buffer.lookup_transform(
                f'{self.robot_name}_map', # ausra_1/map
                f'{self.robot_name}_base_link', # 'ausra_1_base_link'
                rclpy.time.Time(),        # latest available
                RclpyDuration(seconds=0.1)
            )
            pose = Pose()
            pose.position.x = tf.transform.translation.x
            pose.position.y = tf.transform.translation.y
            return pose
        except Exception:
            return None

    def _build_marker(self, point_map: PointStamped, label: str) -> Marker:
        """
        Build a red sphere Marker at the victim's map-frame position.

        marker.id is driven by a monotonic node-level counter so IDs never
        collide across restarts or across detections in the same frame.
        """
        self._marker_counter += 1

        marker                    = Marker()
        marker.header             = point_map.header   # already stamped to 'map'
        marker.ns                 = 'ausra_victims'
        marker.id                 = self._marker_counter
        marker.type               = Marker.SPHERE
        marker.action             = Marker.ADD

        marker.pose.position.x = point_map.point.x
        marker.pose.position.y = point_map.point.y
        marker.pose.position.z = 0.0

        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0   # must be > 0 or RViz renders nothing

        # Persist indefinitely — victim stays on the map until the node restarts.
        # Use MsgDuration(sec=0, nanosec=0) which means "never expire" in RViz.
        marker.lifetime = MsgDuration(sec=0, nanosec=0)

        return marker

    def _publish_waypoint(self, point_map: PointStamped, robot_pose=None) -> None:
        """Publish the victim position as a PoseStamped for downstream consumers.""" 
        waypoint                    = PoseStamped()
        waypoint.header             = point_map.header
        waypoint.header.frame_id    = 'map'

        # Force Z to floor level — planners work in 2D
        waypoint.pose.position.x    = point_map.point.x
        waypoint.pose.position.y    = point_map.point.y
        waypoint.pose.position.z    = 0.0               # ← flatten to ground

        # Face the robot toward the victim using the robot's current XY position
        # If you don't have robot pose yet, identity is the fallback
        if robot_pose is not None:
            dx = point_map.point.x - robot_pose.position.x
            dy = point_map.point.y - robot_pose.position.y
            yaw = math.atan2(dy, dx)                    # angle toward victim

            # Convert yaw to quaternion (rotation around Z axis only)
            waypoint.pose.orientation.x = 0.0
            waypoint.pose.orientation.y = 0.0
            waypoint.pose.orientation.z = math.sin(yaw / 2.0)
            waypoint.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            waypoint.pose.orientation.w = 1.0           # fallback

        self.waypoint_pub.publish(waypoint)


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = AusraYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()