#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import MarkerArray
from collections import deque
import math


class MarkerToError(Node):
    """
    Subscribe to an ArUco marker topic (MarkerArray),
    publish a boolean flag (visible) and a simple error vector (Vector3).

    Published error Vector3 semantics (controller想定):
      - error.x : lateral (right+ / left-)
      - error.z : forward/back (forward+ / back-)
      - error.y : altitude (upper+ / lower-)  ※Used for LAND determination
    """

    def __init__(self):
        super().__init__('marker_to_error')

        # ---- Parameters ----
        self.declare_parameter('markers_topic', '/protoflyer/detected_aruco_markers')
        self.declare_parameter('visible_topic', '/protoflyer/eolab/precision_landing/visible')
        self.declare_parameter('error_topic',   '/protoflyer/eolab/precision_landing/error')

        # axis & scale
        self.declare_parameter('scale', 1.0)
        self.declare_parameter('flip_x', False)
        self.declare_parameter('flip_y', False)
        self.declare_parameter('flip_z', False)

        # Important: specify which axis to use for "forward/backward" and "altitude" (accomodates detector frame differences)
        # e.g. For a typical camera_optical_frame... -> use forward_axis='z', altitude_axis='y'
        #     if the detector uses y: forward/backward, z:up/down... -> use forward_axis='y', altitude_axis='z'
        self.declare_parameter('forward_axis',  'z')   # 'y' or 'z'
        self.declare_parameter('altitude_axis', 'y')   # 'y' or 'z'

        # Marker selection
        self.declare_parameter('target_id',   -1)      # -1: Arbitrary / Select nearest neighbor
        self.declare_parameter('prefer_nearest', True) # True: Select nearest neighbor(minimum | forward |)

        # Smoothing(noise reduction)
        self.declare_parameter('smooth_n', 5)          # Moving average window. 0/1 disable

        # ---- Read parameters ----
        self.markers_topic = self.get_parameter('markers_topic').get_parameter_value().string_value
        self.visible_topic = self.get_parameter('visible_topic').get_parameter_value().string_value
        self.error_topic   = self.get_parameter('error_topic').get_parameter_value().string_value

        self.scale  = float(self.get_parameter('scale').value)
        self.flip_x = bool(self.get_parameter('flip_x').value)
        self.flip_y = bool(self.get_parameter('flip_y').value)
        self.flip_z = bool(self.get_parameter('flip_z').value)

        self.forward_axis  = self.get_parameter('forward_axis').get_parameter_value().string_value.lower()
        self.altitude_axis = self.get_parameter('altitude_axis').get_parameter_value().string_value.lower()

        self.target_id       = int(self.get_parameter('target_id').value)
        self.prefer_nearest  = bool(self.get_parameter('prefer_nearest').value)
        self.smooth_n        = max(0, int(self.get_parameter('smooth_n').value))

        if self.forward_axis == self.altitude_axis:
            self.get_logger().warn(
                f"[marker_to_error] forward_axis ({self.forward_axis}) and altitude_axis "
                f"({self.altitude_axis}) are the SAME. Consider switching one of them."
            )

        # ---- QoS ----
        sub_qos = QoSProfile(depth=10)
        sub_qos.reliability = QoSReliabilityPolicy.RELIABLE
        sub_qos.durability  = QoSDurabilityPolicy.VOLATILE

        pub_qos = QoSProfile(depth=10)
        pub_qos.reliability = QoSReliabilityPolicy.RELIABLE
        pub_qos.durability  = QoSDurabilityPolicy.VOLATILE

        # ---- Pub/Sub ----
        self.vis_pub = self.create_publisher(Bool, self.visible_topic, pub_qos)
        self.err_pub = self.create_publisher(Vector3, self.error_topic, pub_qos)
        self.sub     = self.create_subscription(MarkerArray, self.markers_topic, self.cb_markers, sub_qos)

        # smoothing buffers
        self.buf_x = deque(maxlen=self.smooth_n if self.smooth_n >= 2 else 1)
        self.buf_y = deque(maxlen=self.smooth_n if self.smooth_n >= 2 else 1)
        self.buf_z = deque(maxlen=self.smooth_n if self.smooth_n >= 2 else 1)

        self._last_log = self.get_clock().now()

        self.get_logger().info(
            f"[marker_to_error] sub: {self.markers_topic}\n"
            f"[marker_to_error] pub: visible→{self.visible_topic}, error→{self.error_topic}\n"
            f"[marker_to_error] axes: forward={self.forward_axis}, altitude={self.altitude_axis} "
            f"(plane uses x & forward); scale={self.scale:.3f}, flips(x,y,z)=({self.flip_x},{self.flip_y},{self.flip_z}); "
            f"smooth_n={self.smooth_n}, target_id={self.target_id}, prefer_nearest={self.prefer_nearest}"
        )

    # ---- Helpers ----
    def _select_marker(self, markers):
        """Prefer marker by id; otherwise choose nearest in |forward| axis if prefer_nearest; else first."""
        if not markers:
            return None

        if self.target_id >= 0:
            for m in markers:
                if m.id == self.target_id:
                    return m
            return None  # target specified but not found

        if self.prefer_nearest:
            # choose with minimum absolute forward distance
            def forward_value(m):
                if self.forward_axis == 'z':
                    return float(m.pose.position.z)
                else:
                    return float(m.pose.position.y)
            return min(markers, key=lambda m: abs(forward_value(m)))

        return markers[0]

    def _apply_flips_scale(self, x, y, z):
        if self.flip_x: x = -x
        if self.flip_y: y = -y
        if self.flip_z: z = -z
        return (x * self.scale, y * self.scale, z * self.scale)

    def _smooth(self, x, y, z):
        self.buf_x.append(x); self.buf_y.append(y); self.buf_z.append(z)
        if len(self.buf_x) == 0:
            return x, y, z
        return (sum(self.buf_x)/len(self.buf_x),
                sum(self.buf_y)/len(self.buf_y),
                sum(self.buf_z)/len(self.buf_z))

    # ---- Callback ----
    def cb_markers(self, msg: MarkerArray):
        visible = len(msg.markers) > 0
        out_x = out_y = out_z = 0.0

        if visible:
            m = self._select_marker(msg.markers)
            if m is None:
                visible = False
            else:
                # raw camera-frame positions
                rx = float(m.pose.position.x)
                ry = float(m.pose.position.y)
                rz = float(m.pose.position.z)

                # flips & scale
                rx, ry, rz = self._apply_flips_scale(rx, ry, rz)

                # map to controller error axes:
                #   plane lateral  -> error.x  (x)
                #   plane forward  -> error.z  (forward_axis)
                #   altitude       -> error.y  (altitude_axis)
                out_x = rx
                out_z = rz if self.forward_axis == 'z' else ry
                out_y = ry if self.altitude_axis == 'y' else rz

                # optional smoothing
                if self.smooth_n >= 2:
                    out_x, out_y, out_z = self._smooth(out_x, out_y, out_z)

        # publish
        self.vis_pub.publish(Bool(data=visible))
        self.err_pub.publish(Vector3(x=out_x, y=out_y, z=out_z))

        # 1Hz log
        now = self.get_clock().now()
        if (now - self._last_log).nanoseconds * 1e-9 > 1.0:
            self.get_logger().info(f"visible={visible} error=({out_x:.3f}, {out_y:.3f}, {out_z:.3f})")
            self._last_log = now


def main():
    rclpy.init()
    node = MarkerToError()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
