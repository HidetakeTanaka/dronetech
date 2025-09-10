#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

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
          Autonomous landing state machine:
          WAIT → SEARCH → ALIGN → DESCEND → FINE_ALIGN → LAND
      - mapping_mode = "direct":
          Direct proportional (P) mapping from error to pitch/roll/throttle.
          Useful for bring-up and piping tests.

    ManualControlSetpoint fields used:
      msg.pitch, msg.roll, msg.yaw, msg.throttle ∈ [-1..1]
    """

    def __init__(self):
        super().__init__('landing_controller')

        # ---- Parameters ----
        # Match your running PX4-ROS bridge; override via:
        #   --ros-args -p px4_topic:=/protoflyer/fmu/in/manual_control_input
        self.declare_parameter('px4_topic', '/protoflyer/fmu/in/manual_control_input')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('mapping_mode', 'sm')  # 'sm' or 'direct'

        # Control gains (position error → attitude/throttle)
        self.declare_parameter('kp_xy', 0.7)          # pitch/roll command per meter
        self.declare_parameter('kp_z',  0.10)         # throttle delta per meter (around hover)

        # Command limits
        self.declare_parameter('max_tilt_cmd', 0.4)   # saturation for pitch/roll
        self.declare_parameter('max_yaw_rate', 0.3)   # saturation for yaw

        # Throttle (controller scale 0..1; mapped to PX4 [-1..1] at publish)
        self.declare_parameter('hover_throttle',    0.55)
        self.declare_parameter('descend_throttle',  0.50)
        self.declare_parameter('land_throttle_min', 0.40)

        # Alignment thresholds [m]
        self.declare_parameter('coarse_align_xy', 0.20)
        self.declare_parameter('fine_align_xy',   0.08)
        self.declare_parameter('land_z_thresh',   0.12)

        # Dwell times [s]
        self.declare_parameter('coarse_dwell', 0.6)
        self.declare_parameter('fine_dwell',   0.6)

        # Search behavior
        self.declare_parameter('search_yaw_rate', 0.2)
        self.declare_parameter('lost_timeout',    0.8)

        # Direct mapping helpers (deadzones & safety clamping)
        self.declare_parameter('deadzone_xy', 0.01)
        self.declare_parameter('deadzone_z',  0.02)
        self.declare_parameter('limit_xy',    0.3)
        self.declare_parameter('limit_z_low', 0.35)
        self.declare_parameter('limit_z_high',0.65)

        # ---- Read parameters ----
        self.px4_topic = self.get_parameter('px4_topic').get_parameter_value().string_value
        self.rate_hz   = float(self.get_parameter('rate_hz').get_parameter_value().double_value)
        self.mapping_mode = self.get_parameter('mapping_mode').get_parameter_value().string_value

        self.kp_xy = float(self.get_parameter('kp_xy').get_parameter_value().double_value)
        self.kp_z  = float(self.get_parameter('kp_z').get_parameter_value().double_value)

        self.max_tilt = float(self.get_parameter('max_tilt_cmd').get_parameter_value().double_value)
        self.max_yaw  = float(self.get_parameter('max_yaw_rate').get_parameter_value().double_value)

        self.hover_thr    = float(self.get_parameter('hover_throttle').get_parameter_value().double_value)
        self.descend_thr  = float(self.get_parameter('descend_throttle').get_parameter_value().double_value)
        self.land_thr_min = float(self.get_parameter('land_throttle_min').get_parameter_value().double_value)

        self.coarse_xy = float(self.get_parameter('coarse_align_xy').get_parameter_value().double_value)
        self.fine_xy   = float(self.get_parameter('fine_align_xy').get_parameter_value().double_value)
        self.land_z    = float(self.get_parameter('land_z_thresh').get_parameter_value().double_value)

        self.coarse_dwell = float(self.get_parameter('coarse_dwell').get_parameter_value().double_value)
        self.fine_dwell   = float(self.get_parameter('fine_dwell').get_parameter_value().double_value)

        self.search_yaw_rate = float(self.get_parameter('search_yaw_rate').get_parameter_value().double_value)
        self.lost_timeout    = float(self.get_parameter('lost_timeout').get_parameter_value().double_value)

        self.dead_xy  = float(self.get_parameter('deadzone_xy').get_parameter_value().double_value)
        self.dead_z   = float(self.get_parameter('deadzone_z').get_parameter_value().double_value)
        self.lim_xy   = float(self.get_parameter('limit_xy').get_parameter_value().double_value)
        self.lim_z_lo = float(self.get_parameter('limit_z_low').get_parameter_value().double_value)
        self.lim_z_hi = float(self.get_parameter('limit_z_high').get_parameter_value().double_value)

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

        # Inputs
        self.err = Vector3()
        self.visible = False
        self.last_visible_time = 0.0

        # Engagement flag (toggled by service)
        self.engaged = False

        # Control loop timer
        self.dt = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(self.dt, self.loop)

        self.get_logger().info(
            f'[landing_controller] mode={self.mapping_mode} | pub→ {self.px4_topic} | service /eolab/landing_controller/start'
        )

    # --- Callbacks & helpers ---

    def stamp_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, s):
        if s != self.state:
            self.get_logger().info(f'[SM] {self.state_name(self.state)} -> {self.state_name(s)}')
            self.state = s
            self.state_enter_time = self.stamp_sec()

    def state_name(self, s):
        return ['WAIT', 'SEARCH', 'ALIGN', 'DESCEND', 'FINE_ALIGN', 'LAND'][s]

    def err_cb(self, msg: Vector3):
        self.err = msg
        self.get_logger().info(f'[DBG] err=({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})')

    def vis_cb(self, msg: Bool):
        self.visible = bool(msg.data)
        if self.visible:
            self.last_visible_time = self.stamp_sec()
        self.get_logger().info(f'[DBG] visible={self.visible}')

    def start_srv(self, req: SetBool.Request, res: SetBool.Response):
        self.engaged = bool(req.data)
        if self.engaged:
            if self.mapping_mode == 'sm':
                self.set_state(self.STATE_SEARCH)
            else:
                # Direct mode doesn't use states; set a known baseline anyway
                self.set_state(self.STATE_WAIT)
            res.success = True
            res.message = 'Autonomous mode engaged.'
        else:
            self.set_state(self.STATE_WAIT)
            res.success = True
            res.message = 'Autonomous mode disengaged.'
        return res

    # --- Publish helper (pitch, roll, throttle[0..1], yaw) ---

    def publish_mcp(self, cmd_x, cmd_y, cmd_z, cmd_r):
        """
        Publish a ManualControlSetpoint given controller commands:
          cmd_x: pitch  [-1..1]
          cmd_y: roll   [-1..1]
          cmd_z: throttle [0..1] (will be mapped to [-1..1])
          cmd_r: yaw    [-1..1]
        """
        msg = ManualControlSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # Map to PX4 fields [-1..1]
        msg.pitch = float(clamp(cmd_x, -1.0, 1.0))   # ← pitch
        msg.roll  = float(clamp(cmd_y, -1.0, 1.0))   # ← roll
        msg.yaw   = float(clamp(cmd_r, -1.0, 1.0))   # ← yaw
        
        # Map throttle 0..1 → -1..1
        throttle_px4 = 2.0 * float(clamp(cmd_z, 0.0, 1.0)) - 1.0
        msg.throttle = float(clamp(throttle_px4, -1.0, 1.0))

        # Extended fields expected by your PX4 bridge layout
        msg.valid = True
        msg.data_source = 1
        msg.flaps = 0.0
        msg.aux1 = msg.aux2 = msg.aux3 = msg.aux4 = msg.aux5 = msg.aux6 = 0.0
        msg.buttons = 0

        # Simple sticks_moving heuristic
        eps = 1e-3
        msg.sticks_moving = (abs(msg.pitch) > eps or
                            abs(msg.roll)  > eps or
                            abs(msg.yaw)   > eps or
                            abs(msg.throttle) > eps)

        self.pub.publish(msg)

    # --- Control loop ---

    def loop(self):
        now = self.stamp_sec()

        # Hover-like behavior when disengaged
        if not self.engaged:
            self.publish_mcp(0.0, 0.0, self.hover_thr, 0.0)
            return

        # Defaults (hover reference)
        cmd_x = 0.0   # pitch
        cmd_y = 0.0   # roll
        cmd_r = 0.0   # yaw
        cmd_z = self.hover_thr  # throttle in [0..1] (mapped to [-1..1] on publish)

        lost = (now - self.last_visible_time) > self.lost_timeout

        if self.mapping_mode == 'direct':
            # ---- Direct conversion (bring-up) ----
            if not self.visible or lost:
                self.publish_mcp(0.0, 0.0, self.hover_thr, 0.0)
                return

            # P-control (flip signs here if your frame requires it)
            cmd_x = clamp(self.kp_xy * self.err.x, -self.lim_xy, self.lim_xy)  # pitch  ← err.x
            cmd_y = clamp(self.kp_xy * self.err.y, -self.lim_xy, self.lim_xy)  # roll   ← err.y

            # Throttle tracks z-error around hover throttle
            cmd_z = self.hover_thr - self.kp_z * self.err.z

            # Deadzones & clamping
            if abs(cmd_x) < self.dead_xy: cmd_x = 0.0
            if abs(cmd_y) < self.dead_xy: cmd_y = 0.0
            if abs(cmd_z - self.hover_thr) < self.dead_z: cmd_z = self.hover_thr
            cmd_z = clamp(cmd_z, self.lim_z_lo, self.lim_z_hi)

        else:
            # ---- State machine behavior ----
            if self.state == self.STATE_WAIT:
                pass

            elif self.state == self.STATE_SEARCH:
                cmd_r = clamp(self.search_yaw_rate, -self.max_yaw, self.max_yaw)
                if self.visible and not lost:
                    self.set_state(self.STATE_ALIGN)

            elif self.state == self.STATE_ALIGN:
                cmd_x = clamp(self.kp_xy * self.err.x, -self.max_tilt, self.max_tilt)
                cmd_y = clamp(self.kp_xy * self.err.y, -self.max_tilt, self.max_tilt)
                if lost:
                    self.set_state(self.STATE_SEARCH)
                else:
                    if (abs(self.err.x) < self.coarse_xy) and (abs(self.err.y) < self.coarse_xy):
                        if (now - self.state_enter_time) > self.coarse_dwell:
                            self.set_state(self.STATE_DESCEND)

            elif self.state == self.STATE_DESCEND:
                cmd_x = clamp(self.kp_xy * self.err.x, -self.max_tilt, self.max_tilt)
                cmd_y = clamp(self.kp_xy * self.err.y, -self.max_tilt, self.max_tilt)
                cmd_z = self.descend_thr
                if lost:
                    cmd_z = self.hover_thr
                    self.set_state(self.STATE_SEARCH)
                else:
                    if (abs(self.err.x) < self.fine_xy) and (abs(self.err.y) < self.fine_xy):
                        if (now - self.state_enter_time) > self.fine_dwell or (abs(self.err.z) < self.land_z):
                            self.set_state(self.STATE_FINE)

            elif self.state == self.STATE_FINE:
                cmd_x = clamp(self.kp_xy * self.err.x, -self.max_tilt, self.max_tilt)
                cmd_y = clamp(self.kp_xy * self.err.y, -self.max_tilt, self.max_tilt)
                cmd_z = max(self.land_thr_min, self.descend_thr - 0.03)
                if lost:
                    cmd_z = self.hover_thr
                    self.set_state(self.STATE_SEARCH)
                else:
                    if (abs(self.err.x) < self.fine_xy) and (abs(self.err.y) < self.fine_xy) and (abs(self.err.z) < self.land_z):
                        self.set_state(self.STATE_LAND)

            elif self.state == self.STATE_LAND:
                cmd_x *= 0.5
                cmd_y *= 0.5
                cmd_z = self.land_thr_min

        # Publish using the helper (ensures correct MSG fields only)
        self.publish_mcp(cmd_x, cmd_y, cmd_z, cmd_r)


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
