#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from collections import deque

from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from px4_msgs.msg import ManualControlSetpoint


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class PrecisionLandingController(Node):
    """
    Convert visual marker tracking errors into PX4 ManualControlSetpoint commands.

    Modes:
      - mapping_mode = "sm":
          WAIT → SEARCH → ALIGN → DESCEND → FINE_ALIGN → LAND
      - mapping_mode = "direct":
          direct P mapping from error to pitch/roll/throttle.

    ManualControlSetpoint fields used: pitch, roll, yaw, throttle ∈ [-1..1]
    """

    def __init__(self):
        super().__init__('landing_controller')

        # ---- Parameters ----
        self.declare_parameter('px4_topic', '/protoflyer/fmu/in/manual_control_input')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('mapping_mode', 'sm')  # 'sm' or 'direct'

        # Control gains
        self.declare_parameter('kp_xy', 0.7)          # pitch/roll per meter
        self.declare_parameter('kp_z',  0.10)         # throttle delta per meter

        # Limits
        self.declare_parameter('max_tilt_cmd', 0.4)   # |pitch|,|roll| <=
        self.declare_parameter('max_yaw_rate', 0.3)   # |yaw| <=

        # Throttle (0..1) -> mapped to [-1..1] at publish
        self.declare_parameter('hover_throttle',    0.55)
        self.declare_parameter('descend_throttle',  0.50)
        self.declare_parameter('land_throttle_min', 0.40)

        # Thresholds [m]
        self.declare_parameter('coarse_align_xy', 0.20)
        self.declare_parameter('fine_align_xy',   0.08)
        self.declare_parameter('land_z_thresh',   0.12)

        # Dwells [s]
        self.declare_parameter('coarse_dwell', 0.6)
        self.declare_parameter('fine_dwell',   0.6)

        # Search / robustness
        self.declare_parameter('search_yaw_rate', 0.2)
        self.declare_parameter('lost_timeout',    0.8)

        # Direct mapping helpers
        self.declare_parameter('deadzone_xy', 0.01)
        self.declare_parameter('deadzone_z',  0.02)
        self.declare_parameter('limit_xy',    0.3)
        self.declare_parameter('limit_z_low', 0.35)
        self.declare_parameter('limit_z_high',0.65)

        # Visible dwell to enter ALIGN / LAND
        self.declare_parameter('align_enter_visible_dwell', 0.3)
        self.declare_parameter('land_enter_visible_dwell',  0.3)

        # ---- Read parameters ----
        self.px4_topic   = self.get_parameter('px4_topic').get_parameter_value().string_value
        self.rate_hz     = float(self.get_parameter('rate_hz').value)
        self.mapping_mode= self.get_parameter('mapping_mode').get_parameter_value().string_value

        self.kp_xy   = float(self.get_parameter('kp_xy').value)
        self.kp_z    = float(self.get_parameter('kp_z').value)
        self.max_tilt= float(self.get_parameter('max_tilt_cmd').value)
        self.max_yaw = float(self.get_parameter('max_yaw_rate').value)

        self.hover_thr    = float(self.get_parameter('hover_throttle').value)
        self.descend_thr  = float(self.get_parameter('descend_throttle').value)
        self.land_thr_min = float(self.get_parameter('land_throttle_min').value)

        self.coarse_xy = float(self.get_parameter('coarse_align_xy').value)
        self.fine_xy   = float(self.get_parameter('fine_align_xy').value)
        self.land_z    = float(self.get_parameter('land_z_thresh').value)

        self.coarse_dwell = float(self.get_parameter('coarse_dwell').value)
        self.fine_dwell   = float(self.get_parameter('fine_dwell').value)

        self.search_yaw_rate = float(self.get_parameter('search_yaw_rate').value)
        self.lost_timeout    = float(self.get_parameter('lost_timeout').value)

        self.dead_xy  = float(self.get_parameter('deadzone_xy').value)
        self.dead_z   = float(self.get_parameter('deadzone_z').value)
        self.lim_xy   = float(self.get_parameter('limit_xy').value)
        self.lim_z_lo = float(self.get_parameter('limit_z_low').value)
        self.lim_z_hi = float(self.get_parameter('limit_z_high').value)

        self.align_dwell = float(self.get_parameter('align_enter_visible_dwell').value)
        self.land_dwell  = float(self.get_parameter('land_enter_visible_dwell').value)

        # ---- I/O ----
        self.err_sub = self.create_subscription(Vector3, '/eolab/precision_landing/error', self.err_cb, 10)
        self.vis_sub = self.create_subscription(Bool,   '/eolab/precision_landing/visible', self.vis_cb, 10)
        self.pub     = self.create_publisher(ManualControlSetpoint, self.px4_topic, 10)

        # Start/stop autonomous mode
        self.srv = self.create_service(SetBool, '/eolab/landing_controller/start', self.start_srv)

        # ---- State machine ----
        self.STATE_WAIT    = 0
        self.STATE_SEARCH  = 1
        self.STATE_ALIGN   = 2
        self.STATE_DESCEND = 3
        self.STATE_FINE    = 4
        self.STATE_LAND    = 5

        self.state = self.STATE_WAIT
        self.state_enter_time = self.stamp_sec()

        # Inputs / status
        self.err = Vector3()
        self.visible = False
        self.visible_since = None         # None = invisible, value= time of first visibility
        self.last_visible_time = 0.0

        # Snapshots for log
        self.last_err = Vector3()
        self.last_cmd = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'throttle': 0.10}

        # Engagement
        self.engaged = False

        # Timer
        self.dt = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(self.dt, self.loop)

        # Throttled logger
        self._last_log_time = self.get_clock().now()

        self.get_logger().info(f"LandingController up: mode={self.mapping_mode} pub→{self.px4_topic}")

    # --- Utils ---
    def stamp_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, s):
        if s != self.state:
            self.get_logger().info(f'[SM] {self.state_name(self.state)} -> {self.state_name(s)}')
            self.state = s
            self.state_enter_time = self.stamp_sec()

    def state_name(self, s):
        return ['WAIT', 'SEARCH', 'ALIGN', 'DESCEND', 'FINE_ALIGN', 'LAND'][s]

    # --- Callbacks ---
    def err_cb(self, msg: Vector3):
        self.err = msg
        self.last_err = msg

    def vis_cb(self, msg: Bool):
        now = self.stamp_sec()
        self.visible = bool(msg.data)
        if self.visible:
            if self.visible_since is None:
                self.visible_since = now   # the moment it became visible
            self.last_visible_time = now   # updates every time
        else:
            self.visible_since = None

    def start_srv(self, req: SetBool.Request, res: SetBool.Response):
        self.engaged = bool(req.data)
        if self.engaged:
            if self.mapping_mode == 'sm':
                now = self.stamp_sec()
                vis_ok = (self.visible and
                          (self.visible_since is not None) and
                          ((now - self.visible_since) >= self.align_dwell))
                self.set_state(self.STATE_ALIGN if vis_ok else self.STATE_SEARCH)
            else:
                self.set_state(self.STATE_WAIT)
            res.success, res.message = True, 'Autonomous mode engaged.'
        else:
            self.set_state(self.STATE_WAIT)
            res.success, res.message = True, 'Autonomous mode disengaged.'
        return res

    # --- Publish helper ---
    def publish_mcp(self, cmd_x, cmd_y, cmd_z, cmd_r):
        msg = ManualControlSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        msg.pitch = float(clamp(cmd_x, -1.0, 1.0))
        msg.roll  = float(clamp(cmd_y, -1.0, 1.0))
        msg.yaw   = float(clamp(cmd_r, -1.0, 1.0))

        # throttle: 0..1 -> -1..1
        throttle_px4 = 2.0 * float(clamp(cmd_z, 0.0, 1.0)) - 1.0
        msg.throttle = float(clamp(throttle_px4, -1.0, 1.0))

        msg.valid = True
        msg.data_source = 1
        msg.flaps = 0.0
        msg.aux1 = msg.aux2 = msg.aux3 = msg.aux4 = msg.aux5 = msg.aux6 = 0.0
        msg.buttons = 0

        eps = 1e-3
        msg.sticks_moving = (abs(msg.pitch) > eps or
                             abs(msg.roll)  > eps or
                             abs(msg.yaw)   > eps or
                             abs(msg.throttle) > eps)

        self.pub.publish(msg)

        # snapshot
        self.last_cmd['pitch'] = msg.pitch
        self.last_cmd['roll'] = msg.roll
        self.last_cmd['yaw'] = msg.yaw
        self.last_cmd['throttle'] = msg.throttle

    # --- Main control loop ---
    def loop(self):
        now = self.stamp_sec()
        lost = (now - self.last_visible_time) > self.lost_timeout

        # Disengaged → hover
        if not self.engaged:
            self.publish_mcp(0.0, 0.0, self.hover_thr, 0.0)
            return

        # defaults
        cmd_x = 0.0   # pitch
        cmd_y = 0.0   # roll
        cmd_r = 0.0   # yaw
        cmd_z = self.hover_thr

        if self.mapping_mode == 'direct':
            # Hover if not visible
            if (not self.visible) or lost:
                self.publish_mcp(0.0, 0.0, self.hover_thr, 0.0)
                return

            # Unify： pitch←err.y, roll←err.x
            cmd_x = clamp(self.kp_xy * ( self.err.y), -self.lim_xy, self.lim_xy)
            cmd_y = clamp(self.kp_xy * ( self.err.x), -self.lim_xy, self.lim_xy)

            # Height is controlled by P based on hover: flip_z is handled by the bridge
            cmd_z = self.hover_thr - self.kp_z * self.err.z

            # deadzone / clamp
            if abs(cmd_x) < self.dead_xy: cmd_x = 0.0
            if abs(cmd_y) < self.dead_xy: cmd_y = 0.0
            if abs(cmd_z - self.hover_thr) < self.dead_z: cmd_z = self.hover_thr
            cmd_z = clamp(cmd_z, self.lim_z_lo, self.lim_z_hi)

        else:
            # ---- State machine ----
            if self.state == self.STATE_WAIT:
                pass

            elif self.state == self.STATE_SEARCH:
                cmd_r = clamp(self.search_yaw_rate, -self.max_yaw, self.max_yaw)
                vis_ok = (self.visible and
                          (self.visible_since is not None) and
                          ((now - self.visible_since) >= self.align_dwell))
                if vis_ok and not lost:
                    self.set_state(self.STATE_ALIGN)

            elif self.state == self.STATE_ALIGN:
                if lost or not self.visible:
                    cmd_z = self.hover_thr
                    self.set_state(self.STATE_SEARCH)
                else:
                    # pitch←err.y / roll←err.x
                    cmd_x = clamp(self.kp_xy * ( self.err.y), -self.max_tilt, self.max_tilt)
                    cmd_y = clamp(self.kp_xy * ( self.err.x), -self.max_tilt, self.max_tilt)
                    if (abs(self.err.x) < self.coarse_xy) and (abs(self.err.y) < self.coarse_xy):
                        if (now - self.state_enter_time) > self.coarse_dwell:
                            self.set_state(self.STATE_DESCEND)

            elif self.state == self.STATE_DESCEND:
                if lost or not self.visible:
                    cmd_z = self.hover_thr
                    self.set_state(self.STATE_SEARCH)
                else:
                    cmd_x = clamp(self.kp_xy * ( self.err.y), -self.max_tilt, self.max_tilt)
                    cmd_y = clamp(self.kp_xy * ( self.err.x), -self.max_tilt, self.max_tilt)
                    cmd_z = self.descend_thr
                    if (abs(self.err.x) < self.fine_xy) and (abs(self.err.y) < self.fine_xy):
                        if (now - self.state_enter_time) > self.fine_dwell or (abs(self.err.z) < self.land_z):
                            self.set_state(self.STATE_FINE)

            elif self.state == self.STATE_FINE:
                if lost or not self.visible:
                    cmd_z = self.hover_thr
                    self.set_state(self.STATE_SEARCH)
                else:
                    cmd_x = clamp(self.kp_xy * ( self.err.y), -self.max_tilt, self.max_tilt)
                    cmd_y = clamp(self.kp_xy * ( self.err.x), -self.max_tilt, self.max_tilt)
                    cmd_z = max(self.land_thr_min, self.descend_thr - 0.03)

                    land_vis_ok = (self.visible and
                                   (self.visible_since is not None) and
                                   ((now - self.visible_since) >= self.land_dwell))
                    if land_vis_ok and (abs(self.err.x) < self.fine_xy) and (abs(self.err.y) < self.fine_xy) and (abs(self.err.z) < self.land_z):
                        self.set_state(self.STATE_LAND)

            elif self.state == self.STATE_LAND:
                if lost or not self.visible:
                    cmd_z = self.hover_thr
                    self.set_state(self.STATE_SEARCH)
                else:
                    cmd_x *= 0.5
                    cmd_y *= 0.5
                    cmd_z = self.land_thr_min

        # publish
        self.publish_mcp(cmd_x, cmd_y, cmd_z, cmd_r)

        # 1Hz Log
        now_ros = self.get_clock().now()
        if (now_ros - self._last_log_time).nanoseconds * 1e-9 > 1.0:
            self.get_logger().info(
                f"state={self.state_name(self.state)} vis={self.visible} "
                f"err=({self.last_err.x:.2f},{self.last_err.y:.2f},{self.last_err.z:.2f}) "
                f"cmd(roll,pitch,yaw,thr)="
                f"({self.last_cmd['roll']:.2f},{self.last_cmd['pitch']:.2f},"
                f"{self.last_cmd['yaw']:.2f},{self.last_cmd['throttle']:.2f})"
            )
            self._last_log_time = now_ros


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLandingController()
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
