#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Bool


class LandingSubscriber(Node):
    """
    Subscribe: marker pose -> Publish: filtered position error & visibility flag.
    Assumed frame (Body-ish): x forward(+), y right(+), z down(+).
    You can flip signs via parameters if your publisher uses another convention.
    """

    def __init__(self):
        super().__init__('landing_subscriber')

        # --- Parameters ---
        self.declare_parameter('marker_topic', '/eolab/precision_landing/marker_pose')
        self.declare_parameter('visible_timeout', 0.6)   # [s] no message -> invisible
        self.declare_parameter('alpha', 0.3)             # low-pass factor 0..1 (higher = quicker)
        # sign flips if needed (e.g., camera optical frame → body frame調整など)
        self.declare_parameter('sign_x', 1.0)  # +1 or -1
        self.declare_parameter('sign_y', 1.0)
        self.declare_parameter('sign_z', 1.0)

        self.marker_topic: str = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.visible_timeout: float = self.get_parameter('visible_timeout').get_parameter_value().double_value
        self.alpha: float = float(self.get_parameter('alpha').get_parameter_value().double_value)
        self.sign_x: float = float(self.get_parameter('sign_x').get_parameter_value().double_value)
        self.sign_y: float = float(self.get_parameter('sign_y').get_parameter_value().double_value)
        self.sign_z: float = float(self.get_parameter('sign_z').get_parameter_value().double_value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.err_pub = self.create_publisher(Vector3, '/eolab/precision_landing/error', 10)
        self.vis_pub = self.create_publisher(Bool,   '/eolab/precision_landing/visible', 10)

        # Subscriber (PoseStamped)
        self.sub = self.create_subscription(PoseStamped, self.marker_topic, self.pose_cb, qos)

        # State
        self.last_seen_time: Optional[float] = None
        self.err_filtered = Vector3()

        # Visibility watchdog
        self.timer = self.create_timer(0.05, self.visible_watchdog)  # 20 Hz

        self.get_logger().info(
            f'[landing_subscriber] listening {self.marker_topic} → publish /eolab/precision_landing/error, /visible'
        )

    def pose_cb(self, msg: PoseStamped):
        # Raw pose (marker relative to drone)
        ex = msg.pose.position.x * self.sign_x
        ey = msg.pose.position.y * self.sign_y
        ez = msg.pose.position.z * self.sign_z

        # Low-pass filtering
        a = self.alpha
        self.err_filtered.x = a * ex + (1 - a) * self.err_filtered.x
        self.err_filtered.y = a * ey + (1 - a) * self.err_filtered.y
        self.err_filtered.z = a * ez + (1 - a) * self.err_filtered.z

        # Publish error
        self.err_pub.publish(self.err_filtered)

        # Update visibility
        self.last_seen_time = self.get_clock().now().nanoseconds * 1e-9
        self.vis_pub.publish(Bool(data=True))

    def visible_watchdog(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        visible = False
        if self.last_seen_time is not None and (now - self.last_seen_time) <= self.visible_timeout:
            visible = True
        self.vis_pub.publish(Bool(data=visible))


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
