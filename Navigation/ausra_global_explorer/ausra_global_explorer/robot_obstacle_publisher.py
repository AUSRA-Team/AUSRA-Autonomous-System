#!/usr/bin/env python3
# Copyright 2024 AUSRA Team — Apache-2.0
"""
robot_obstacle_publisher.py
============================
Fleet-level node: makes every robot "visible" to every other robot's costmap.

For N robots in the swarm, this node:
  1. Reads each robot's position from TF (map → {robot}_robot_footprint)
  2. Generates a PointCloud2 disk of radius `robot_obstacle_radius_m` around it
  3. Publishes the COMBINED disk of all OTHER robots to each robot's
     `/{robot}/neighbor_obstacles` topic

This feeds three Nav2 layers simultaneously:
  • Local costmap  obstacle_layer   → DWB avoids live robot positions
  • Global costmap obstacle_layer   → A* plans routes that don't cross swarm
  • Collision Monitor               → Hard-stops if another robot enters footprint

Publish rate: 10 Hz (configurable)
Frame: map (global frame shared by all robots)
"""

import math
import struct
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener, TransformException


class RobotObstaclePublisher(Node):
    """Publishes swarm-mate positions as PointCloud2 obstacle halos."""

    def __init__(self):
        super().__init__('robot_obstacle_publisher')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('robot_names', 'ausra_1,ausra_2,ausra_3')
        self.declare_parameter('robot_obstacle_radius_m', 0.45)   # halo disk radius
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('ring_points', 20)   # points per ring in the disk
        # NOTE: use_sim_time is already declared by the Node base class — do NOT redeclare

        names_str = self.get_parameter('robot_names').value
        self._robot_names = [r.strip() for r in names_str.split(',') if r.strip()]
        self._radius = self.get_parameter('robot_obstacle_radius_m').value
        self._ring_pts = self.get_parameter('ring_points').value

        # ── TF ────────────────────────────────────────────────────────────────
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        # ── Publishers (one per robot) ─────────────────────────────────────────
        # Each robot gets a PointCloud2 of all OTHER robots' positions.
        self._pubs = {
            robot: self.create_publisher(
                PointCloud2, f'/{robot}/neighbor_obstacles', 10
            )
            for robot in self._robot_names
        }

        # ── Timer ─────────────────────────────────────────────────────────────
        rate = self.get_parameter('publish_rate_hz').value
        self._timer = self.create_timer(1.0 / rate, self._publish_cb)

        self.get_logger().info(
            f'[RobotObstaclePublisher] Swarm: {self._robot_names} | '
            f'halo_radius={self._radius:.2f}m | rate={rate:.1f}Hz'
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _get_pose(self, robot_name, target_frame):
        """Return (x, y) of robot in target_frame, or None if TF unavailable."""
        try:
            tf = self._tf_buf.lookup_transform(
                target_frame,
                f'{robot_name}_robot_footprint',
                Time(),               # latest available transform
            )
            return tf.transform.translation.x, tf.transform.translation.y
        except TransformException:
            return None

    def _build_pointcloud(self, robot_center_poses, stamp, frame_id):
        """
        Build a single PointCloud2 from a list of (cx, cy) obstacle centres.
        Each centre is represented by two concentric rings + centre point.
        Frame: frame_id.
        """
        pts = []
        for (cx, cy) in robot_center_poses:
            # Outer ring at full radius
            for i in range(self._ring_pts):
                a = 2.0 * math.pi * i / self._ring_pts
                pts.append((cx + self._radius * math.cos(a),
                             cy + self._radius * math.sin(a),
                             0.5))   # z = mid-body height
            # Inner ring at half radius
            for i in range(self._ring_pts):
                a = 2.0 * math.pi * i / self._ring_pts
                r2 = self._radius * 0.5
                pts.append((cx + r2 * math.cos(a),
                             cy + r2 * math.sin(a),
                             0.5))
            # Centre point
            pts.append((cx, cy, 0.5))

        if not pts:
            return None

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = len(pts)
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12        # 3 × float32
        msg.row_step = 12 * len(pts)
        msg.is_dense = True

        raw = bytearray()
        for x, y, z in pts:
            raw += struct.pack('fff', float(x), float(y), float(z))
        msg.data = bytes(raw)
        return msg

    # ── Timer callback ────────────────────────────────────────────────────────

    def _publish_cb(self):
        now = self.get_clock().now().to_msg()

        # For each target robot, publish all OTHER robots relative to its map frame
        for target_robot in self._robot_names:
            # Target robot's map frame is {target_robot}_map
            target_map_frame = f'{target_robot}_map'

            other_poses = []
            for other_robot in self._robot_names:
                if other_robot == target_robot:
                    continue

                pos = self._get_pose(other_robot, target_map_frame)
                if pos is not None:
                    other_poses.append(pos)

            if not other_poses:
                continue

            cloud = self._build_pointcloud(other_poses, now, target_map_frame)
            if cloud is not None:
                self._pubs[target_robot].publish(cloud)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = RobotObstaclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
