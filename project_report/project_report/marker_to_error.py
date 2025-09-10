#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import MarkerArray

def clamp(v, lo, hi): return max(lo, min(hi, v))

class MarkerToError(Node):
    """
    A simple bridge node that listens to
      /protoflyer/detected_aruco_markers (MarkerArray),
    estimates the error vector from the closest (or first) marker,
    and publishes:
      /protoflyer/eolab/precision_landing/visible (Bool)
      /protoflyer/eolab/precision_landing/error   (Vector3)

    The sign convention may differ depending on the environment,
    so it can be inverted via parameters.
    """
    def __init__(self):
        super().__init__('marker_to_error')

        # Input/output topics (can be remapped with --ros-args -r)
        self.sub_topic = self.declare_parameter(
            'markers_topic', '/protoflyer/detected_aruco_markers'
        ).get_parameter_value().string_value

        self.pub_visible_topic = self.declare_parameter(
            'visible_topic', '/protoflyer/eolab/precision_landing/visible'
        ).get_parameter_value().string_value

        self.pub_error_topic = self.declare_parameter(
            'error_topic', '/protoflyer/eolab/precision_landing/error'
        ).get_parameter_value().string_value

        # Sign inversion (to compensate camera vs control coordinate differences)
        self.flip_x = bool(self.declare_parameter('flip_x', False).value)
        self.flip_y = bool(self.declare_parameter('flip_y', False).value)
        self.flip_z = bool(self.declare_parameter('flip_z', False).value)

        # Scale factors (useful for pixel→meter conversion; keep 1.0 if already in meters)
        self.scale_x = float(self.declare_parameter('scale_x', 1.0).value)
        self.scale_y = float(self.declare_parameter('scale_y', 1.0).value)
        self.scale_z = float(self.declare_parameter('scale_z', 1.0).value)

        # Publishers
        self.pub_visible = self.create_publisher(Bool, self.pub_visible_topic, 10)
        self.pub_error   = self.create_publisher(Vector3, self.pub_error_topic, 10)

        # Subscriber
        self.sub = self.create_subscription(MarkerArray, self.sub_topic, self.cb, 10)

        self.get_logger().info(f'[marker_to_error] sub: {self.sub_topic}')
        self.get_logger().info(f'[marker_to_error] pub: visible→{self.pub_visible_topic} | error→{self.pub_error_topic}')

    def cb(self, msg: MarkerArray):
        visible = Bool()
        err = Vector3()

        if len(msg.markers) == 0:
            visible.data = False
            self.pub_visible.publish(visible)
            # For error, we could keep the previous value,
            # but here we output zero
            err.x = 0.0; err.y = 0.0; err.z = 0.0
            self.pub_error.publish(err)
            return

        # Here we simply take the first marker
        m = msg.markers[0]
        # Marker.pose is likely in camera coordinates [m] (environment-dependent)
        x = m.pose.position.x * (-1.0 if self.flip_x else 1.0) * self.scale_x
        y = m.pose.position.y * (-1.0 if self.flip_y else 1.0) * self.scale_y
        z = m.pose.position.z * (-1.0 if self.flip_z else 1.0) * self.scale_z

        # Controller expectation:
        #   err.x = forward/backward (+ forward)
        #   err.y = left/right (+ right)
        #   err.z = altitude error (+ higher)
        err.x = float(x)
        err.y = float(y)
        err.z = float(z)

        visible.data = True
        self.pub_visible.publish(visible)
        self.pub_error.publish(err)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerToError()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
