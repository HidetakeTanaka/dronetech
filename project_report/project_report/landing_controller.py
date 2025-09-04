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
    State machine that translates marker errors into PX4 ManualControlSetpoint.
    States: WAIT → SEARCH → ALIGN → DESCEND → FINE_ALIGN → LAND
    Internal command semantics:
      cmd_x = pitch  (forward/back, + pushes nose down, moves forward)
      cmd_y = roll   (left/right, + tips right wing down, moves right)
      cmd_r = yaw    (turn rate, + rotates clockwise seen from top)
      cmd_z = throttle in [0..1] (0=min thrust, 1=max thrust) → mapped to PX4 [-1..1]
    """

    def __init__(self):
        super().__init__('landing_controller')

        # ---- Parameters ----
        self.declare_parameter('px4_topic', '/fmu/manual_control_setpoint/in')
        self.declare_parameter('rate_hz', 50.0)

        # Control gains
        self.declare_parameter('kp_xy', 0.7)          # roll/pitch command per meter
        self.declare_parameter('kp_z',  0.0)          # not used (we handle altitude via throttle)

        # command limits
        self.declare_parameter('max_tilt_cmd', 0.4)   # saturation for roll/pitch
        self.declare_parameter('max_yaw_rate', 0.3)   # saturation for yaw

        # Throttle values (controller's 0..1 scale; mapped to PX4 -1..1 on publish)
        self.declare_parameter('hover_throttle',   0.55)
        self.declare_parameter('descend_throttle', 0.50)
        self.declare_parameter('land_throttle_min',0.40)

        # Alignment thresholds [m]
        self.declare_parameter('coarse_align_xy', 0.20)
        self.declare_parameter('fine_align_xy',   0.08)
        self.declare_parameter('land_z_thresh',   0.12)

        # Dwell times [s]
        self.declare_parameter('coarse_dwell', 0.6)
        self.declare_parameter('fine_dwell',   0.6)

        # Search settings
        self.declare_parameter('search_yaw_rate', 0.2)
        self.declare_parameter('lost_timeout',    0.8)

        # Read parameters
        self.px4_topic = self.get_parameter('px4_topic').get_parameter_value().string_value
        self.rate_hz = float(self.get_parameter('rate_hz').get_parameter_value().double_value)

        self.kp_xy = float(self.get_parameter('kp_xy').get_parameter_value().double_value)
        self.kp_z  = float(self.get_parameter('kp_z').get_parameter_value().double_value)

        self.max_tilt = float(self.get_parameter('max_tilt_cmd').get_parameter_value().double_value)
        self.max_yaw  = float(self.get_parameter('max_yaw_rate').get_parameter_value().double_value)

        self.hover_thr   = float(self.get_parameter('hover_throttle').get_parameter_value().double_value)
        self.descend_thr = float(self.get_parameter('descend_throttle').get_parameter_value().double_value)
        self.land_thr_min= float(self.get_parameter('land_throttle_min').get_parameter_value().double_value)

        self.coarse_xy = float(self.get_parameter('coarse_align_xy').get_parameter_value().double_value)
        self.fine_xy   = float(self.get_parameter('fine_align_xy').get_parameter_value().double_value)
        self.land_z    = float(self.get_parameter('land_z_thresh').get_parameter_value().double_value)

        self.coarse_dwell = float(self.get_parameter('coarse_dwell').get_parameter_value().double_value)
        self.fine_dwell   = float(self.get_parameter('fine_dwell').get_parameter_value().double_value)

        self.search_yaw_rate = float(self.get_parameter('search_yaw_rate').get_parameter_value().double_value)
        self.lost_timeout    = float(self.get_parameter('lost_timeout').get_parameter_value().double_value)

        # ---- I/O ----
        self.err_sub = self.create_subscription(Vector3, '/eolab/precision_landing/error', self.err_cb, 10)
        self.vis_sub = self.create_subscription(Bool,   '/eolab/precision_landing/visible', self.vis_cb, 10)
        self.pub     = self.create_publisher(ManualControlSetpoint, self.px4_topic, 10)

        # Service to start/stop autonomous
        self.srv = self.create_service(SetBool, '/eolab/landing_controller/start', self.start_srv)

        # ---- State machine ----
        self.STATE_WAIT   = 0
        self.STATE_SEARCH = 1
        self.STATE_ALIGN  = 2
        self.STATE_DESCEND= 3
        self.STATE_FINE   = 4
        self.STATE_LAND   = 5

        self.state = self.STATE_WAIT
        self.state_enter_time = self.stamp_sec()

        # inputs
        self.err = Vector3()
        self.visible = False
        self.last_visible_time = 0.0

        # control loop
        self.dt = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(self.dt, self.loop)

        self.get_logger().info(
            f'[landing_controller] publishing to {self.px4_topic} | call /eolab/landing_controller/start to engage'
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
        return ['WAIT','SEARCH','ALIGN','DESCEND','FINE_ALIGN','LAND'][s]

    def err_cb(self, msg: Vector3):
        self.err = msg

    def vis_cb(self, msg: Bool):
        self.visible = bool(msg.data)
        if self.visible:
            self.last_visible_time = self.stamp_sec()

    def start_srv(self, req: SetBool.Request, res: SetBool.Response):
        if req.data:
            self.set_state(self.STATE_SEARCH)
            res.success = True
            res.message = 'Autonomous mode engaged.'
        else:
            self.set_state(self.STATE_WAIT)
            res.success = True
            res.message = 'Autonomous mode disengaged.'
        return res

    # --- Control loop ---

    def loop(self):
        now = self.stamp_sec()

        # default commands (hover)
        cmd_x = 0.0   # pitch
        cmd_y = 0.0   # roll
        cmd_r = 0.0   # yaw
        cmd_z = self.hover_thr  # throttle in [0..1] (will be remapped to PX4 [-1..1])

        lost = (now - self.last_visible_time) > self.lost_timeout

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

        # Publish ManualControlSetpoint (PX4 expects roll/pitch/yaw/throttle in [-1..1])
        msg = ManualControlSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # Map our internal commands to PX4 fields:
        msg.pitch = float(clamp(cmd_x, -1.0, 1.0))
        msg.roll  = float(clamp(cmd_y, -1.0, 1.0))
        msg.yaw   = float(clamp(cmd_r, -1.0, 1.0))

        # Map 0..1 → -1..1 for throttle
        throttle_px4 = 2.0 * float(clamp(cmd_z, 0.0, 1.0)) - 1.0
        msg.throttle = float(clamp(throttle_px4, -1.0, 1.0))

        self.pub.publish(msg)


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
