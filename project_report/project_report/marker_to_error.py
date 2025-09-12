#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import MarkerArray


class MarkerToError(Node):
    """
    Bridge from ArUco MarkerArray to:
      - visible: std_msgs/Bool
      - error:   geometry_msgs/Vector3

    Controller-side semantics:
      error.x : lateral (right + / left -)
      error.z : forward/back (forward + / back -)
      error.y : altitude cue (up + / down -)

    Improvements vs. previous version:
      - Heartbeat timer publishes at a fixed rate (even if detector is irregular).
      - 'stale_timeout_s' forces visible=false if no detections arrive in time.
      - Optional outlier/jump rejection to reduce jitter close to landing.
    """

    def __init__(self):
        super().__init__('marker_to_error')

        # ---- Parameters ----
        self.declare_parameter('markers_topic', '/protoflyer/detected_aruco_markers')
        self.declare_parameter('visible_topic', '/protoflyer/eolab/precision_landing/visible')
        self.declare_parameter('error_topic',   '/protoflyer/eolab/precision_landing/error')

        # Axis, scale, flips
        self.declare_parameter('scale', 1.0)
        self.declare_parameter('flip_x', False)
        self.declare_parameter('flip_y', False)
        self.declare_parameter('flip_z', False)
        self.declare_parameter('forward_axis',  'z')  # 'y' or 'z'
        self.declare_parameter('altitude_axis', 'y')  # 'y' or 'z'

        # Selection
        self.declare_parameter('target_id', -1)        # -1: accept any ID
        self.declare_parameter('prefer_nearest', True) # used when target_id==-1

        # Smoothing / Robustness
        self.declare_parameter('smooth_n', 5)          # 0/1 disables smoothing
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('stale_timeout_s', 0.60)
        self.declare_parameter('jump_reject_m', 0.60)  # reject single-frame jumps > this (0 disables)
        self.declare_parameter('range_cap_m', 6.0)     # clamp extreme values

        # ---- Read parameters ----
        gp = self.get_parameter
        self.markers_topic = gp('markers_topic').get_parameter_value().string_value
        self.visible_topic = gp('visible_topic').get_parameter_value().string_value
        self.error_topic   = gp('error_topic').get_parameter_value().string_value

        self.scale  = float(gp('scale').value)
        self.flip_x = bool(gp('flip_x').value)
        self.flip_y = bool(gp('flip_y').value)
        self.flip_z = bool(gp('flip_z').value)

        self.forward_axis  = gp('forward_axis').get_parameter_value().string_value.lower()
        self.altitude_axis = gp('altitude_axis').get_parameter_value().string_value.lower()

        self.target_id      = int(gp('target_id').value)
        self.prefer_nearest = bool(gp('prefer_nearest').value)
        self.smooth_n       = max(0, int(gp('smooth_n').value))

        self.publish_rate_hz = max(5.0, float(gp('publish_rate_hz').value))
        self.stale_timeout_s = max(0.1, float(gp('stale_timeout_s').value))
        self.jump_reject_m   = max(0.0, float(gp('jump_reject_m').value))
        self.range_cap_m     = max(0.1, float(gp('range_cap_m').value))

        if self.forward_axis == self.altitude_axis:
            self.get_logger().warn(
                f"[marker_to_error] forward_axis ({self.forward_axis}) and altitude_axis "
                f"({self.altitude_axis}) are identical. Verify detector axes."
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

        # Smoothing buffers
        win = self.smooth_n if self.smooth_n >= 2 else 1
        self.buf_x = deque(maxlen=win)
        self.buf_y = deque(maxlen=win)
        self.buf_z = deque(maxlen=win)

        # Internal state for heartbeat publishing
        self._last_rx_t = self.get_clock().now()
        self._have_valid = False
        self._ox = 0.0; self._oy = 0.0; self._oz = 0.0
        self._prev_ox = None; self._prev_oy = None; self._prev_oz = None

        # Heartbeat timer
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._heartbeat_publish)

        self._last_log = self.get_clock().now()
        self.get_logger().info(
            f"[marker_to_error] sub: {self.markers_topic}\n"
            f"[marker_to_error] pub: visible→{self.visible_topic}, error→{self.error_topic}\n"
            f"[marker_to_error] axes: forward={self.forward_axis}, altitude={self.altitude_axis}; "
            f"scale={self.scale:.3f}, flips(x,y,z)=({self.flip_x},{self.flip_y},{self.flip_z}); "
            f"smooth_n={self.smooth_n}, target_id={self.target_id}, prefer_nearest={self.prefer_nearest}; "
            f"rate={self.publish_rate_hz:.1f}Hz, stale_timeout={self.stale_timeout_s:.2f}s"
        )

    # ---- Helpers ----
    def _apply_flips_scale(self, x, y, z):
        if self.flip_x: x = -x
        if self.flip_y: y = -y
        if self.flip_z: z = -z
        # Clamp absurd ranges to avoid blowing up the controller
        x = max(-self.range_cap_m, min(self.range_cap_m, x * self.scale))
        y = max(-self.range_cap_m, min(self.range_cap_m, y * self.scale))
        z = max(-self.range_cap_m, min(self.range_cap_m, z * self.scale))
        return x, y, z

    def _plane_error_sq(self, m):
        """Squared distance in (lateral x, forward) plane after flips/scale (for ranking)."""
        try:
            rx = float(m.pose.position.x)
            ry = float(m.pose.position.y)
            rz = float(m.pose.position.z)
        except Exception:
            return float('inf')
        if not (math.isfinite(rx) and math.isfinite(ry) and math.isfinite(rz)):
            return float('inf')
        rx, ry, rz = self._apply_flips_scale(rx, ry, rz)
        fwd = rz if self.forward_axis == 'z' else ry
        return rx * rx + fwd * fwd

    def _select_marker(self, markers):
        if not markers:
            return None
        if self.target_id >= 0:
            for m in markers:
                if int(m.id) == self.target_id:
                    return m
            return None
        if self.prefer_nearest:
            return min(markers, key=self._plane_error_sq)
        return markers[0]

    def _smooth(self, x, y, z):
        self.buf_x.append(x); self.buf_y.append(y); self.buf_z.append(z)
        n = len(self.buf_x) or 1
        return (sum(self.buf_x)/n, sum(self.buf_y)/n, sum(self.buf_z)/n)

    # ---- Callback ----
    def cb_markers(self, msg: MarkerArray):
        if len(msg.markers) == 0:
            # Keep last but do not mark as fresh; heartbeat will expire it
            return

        m = self._select_marker(msg.markers)
        if m is None:
            return

        rx = float(m.pose.position.x)
        ry = float(m.pose.position.y)
        rz = float(m.pose.position.z)
        if not (math.isfinite(rx) and math.isfinite(ry) and math.isfinite(rz)):
            return

        rx, ry, rz = self._apply_flips_scale(rx, ry, rz)

        # mapping to controller axes
        ox = rx
        oz = rz if self.forward_axis == 'z' else ry
        oy = ry if self.altitude_axis == 'y' else rz

        # outlier / jump rejection
        if self.jump_reject_m > 0.0 and self._prev_ox is not None:
            if (abs(ox - self._prev_ox) > self.jump_reject_m or
                abs(oy - self._prev_oy) > self.jump_reject_m or
                abs(oz - self._prev_oz) > self.jump_reject_m):
                # Ignore a single crazy spike
                return

        # smoothing
        if self.smooth_n >= 2:
            ox, oy, oz = self._smooth(ox, oy, oz)

        self._ox, self._oy, self._oz = ox, oy, oz
        self._prev_ox, self._prev_oy, self._prev_oz = ox, oy, oz
        self._have_valid = True
        self._last_rx_t = self.get_clock().now()

    # ---- Heartbeat publisher (fixed rate) ----
    def _heartbeat_publish(self):
        now = self.get_clock().now()
        stale = (now - self._last_rx_t).nanoseconds * 1e-9 > self.stale_timeout_s

        visible = (self._have_valid and not stale)
        if stale:
            # decay error to zeros when stale
            self._ox *= 0.0; self._oy *= 0.0; self._oz *= 0.0

        self.vis_pub.publish(Bool(data=visible))
        self.err_pub.publish(Vector3(x=float(self._ox), y=float(self._oy), z=float(self._oz)))

        # 1 Hz log
        if (now - self._last_log).nanoseconds * 1e-9 > 1.0:
            self.get_logger().info(f"visible={visible} error=({self._ox:.3f}, {self._oy:.3f}, {self._oz:.3f})")
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
