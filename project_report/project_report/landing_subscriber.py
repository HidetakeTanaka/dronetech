#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Vector3
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Bool
from typing import Optional


class LandingSubscriber(Node):
    """
    Subscribe: ArUco marker detections (PoseStamped or MarkerArray).
    Publish: filtered relative position error (Vector3) + visibility flag.

    Frame convention (assumed body-like):
      x forward (+), y right (+), z down (+).
    Use sign_x/y/z params to flip if the detector uses another convention.
    """

    def __init__(self):
        super().__init__('landing_subscriber')

        # --- Parameters (all relative topics by default) ---
        self.declare_parameter('marker_topic', 'eolab/precision_landing/marker_pose')
        self.declare_parameter('marker_array_topic', 'detected_aruco_markers')
        self.declare_parameter('marker_id', -1)           # -1 = use first available marker
        self.declare_parameter('visible_timeout', 0.6)    # seconds without msg → invisible
        self.declare_parameter('alpha', 0.3)              # low-pass 0..1 (higher = faster)
        self.declare_parameter('sign_x', 1.0)             # flip axes if needed
        self.declare_parameter('sign_y', 1.0)
        self.declare_parameter('sign_z', 1.0)

        self.marker_topic: str = self.get_parameter('marker_topic').value
        self.marker_array_topic: str = self.get_parameter('marker_array_topic').value
        self.marker_id: int = int(self.get_parameter('marker_id').value)
        self.visible_timeout: float = float(self.get_parameter('visible_timeout').value)
        self.alpha: float = float(self.get_parameter('alpha').value)
        self.sign_x: float = float(self.get_parameter('sign_x').value)
        self.sign_y: float = float(self.get_parameter('sign_y').value)
        self.sign_z: float = float(self.get_parameter('sign_z').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Publishers (relative topics, so they live under the node namespace) ---
        self.err_pub = self.create_publisher(Vector3, 'eolab/precision_landing/error', 10)
        self.vis_pub = self.create_publisher(Bool, 'eolab/precision_landing/visible', 10)

        # --- Subscribers (support both PoseStamped and MarkerArray) ---
        self.sub_pose = self.create_subscription(PoseStamped, self.marker_topic, self.pose_cb, qos)
        self.sub_arr = self.create_subscription(MarkerArray, self.marker_array_topic, self.marker_array_cb, qos)

        # --- State ---
        self.last_seen_time: Optional[float] = None
        self.err_filtered = Vector3()

        # Visibility watchdog (20 Hz)
        self.timer = self.create_timer(0.05, self.visible_watchdog)

        self.get_logger().info(
            f'[landing_subscriber] subscribed to "{self.marker_topic}" and "{self.marker_array_topic}" '
            f'→ publishing error & visibility'
        )

    # --- Callbacks ---
    def pose_cb(self, msg: PoseStamped):
        ex = msg.pose.position.x * self.sign_x
        ey = msg.pose.position.y * self.sign_y
        ez = msg.pose.position.z * self.sign_z
        self.update_error(ex, ey, ez)

    def marker_array_cb(self, msg: MarkerArray):
        if not msg.markers:
            return
        marker = None
        if self.marker_id >= 0:
            marker = next((m for m in msg.markers if m.id == self.marker_id), None)
        if marker is None:
            marker = msg.markers[0]

        ex = marker.pose.position.x * self.sign_x
        ey = marker.pose.position.y * self.sign_y
        ez = marker.pose.position.z * self.sign_z
        self.update_error(ex, ey, ez)

    # --- Helpers ---
    def update_error(self, ex: float, ey: float, ez: float):
        a = self.alpha
        self.err_filtered.x = a * ex + (1 - a) * self.err_filtered.x
        self.err_filtered.y = a * ey + (1 - a) * self.err_filtered.y
        self.err_filtered.z = a * ez + (1 - a) * self.err_filtered.z
        self.err_pub.publish(self.err_filtered)
        self.last_seen_time = self.get_clock().now().nanoseconds * 1e-9
        self.vis_pub.publish(Bool(data=True))

    def visible_watchdog(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        visible = (
            self.last_seen_time is not None
            and (now - self.last_seen_time) <= self.visible_timeout
        )
        self.vis_pub.publish(Bool(data=visible))

    self.debug_timer = self.create_timer(1.0, self.debug_tick)

    def debug_tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        vis = (self.last_seen_time is not None) and ((now - self.last_seen_time) <= self.visible_timeout)
        self.get_logger().info(f'visible={vis}  err=({self.err_filtered.x:.2f},{self.err_filtered.y:.2f},{self.err_filtered.z:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = LandingSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
