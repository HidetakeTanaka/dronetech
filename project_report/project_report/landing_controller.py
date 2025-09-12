#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from px4_msgs.msg import ManualControlSetpoint, VehicleControlMode


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

    Axes (as used by your bridge):
      - lateral roll  ← +err.x
      - lateral pitch ← -err.z
      - altitude axis ←  err.y

    PX4 sticks range: roll/pitch/yaw/throttle ∈ [-1..1]
    """

    def __init__(self):
        super().__init__('landing_controller')

        # ---- Parameters ----
        self.declare_parameter('px4_topic', '/protoflyer/fmu/in/manual_control_input')
        self.declare_parameter('vcm_topic', '/protoflyer/fmu/out/vehicle_control_mode')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('mapping_mode', 'sm')  # 'sm' or 'direct'

        # Ownership / acceptance
        self.declare_parameter('exclusive_mode', True)   # True: always publish to own sticks
        self.declare_parameter('idle_hover', True)       # in exclusive mode, send hover when not engaged
        self.declare_parameter('data_source', 2)         # 2 = MAVLINK_0

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
        self.declare_parameter('land_z_thresh',   0.12)   # altitude axis (err.y) threshold

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
        self.declare_parameter('align_enter_visible_dwell', 0.15)
        self.declare_parameter('land_enter_visible_dwell',  0.3)

        # SEARCH pattern + guided homing
        self.declare_parameter('search_mode', 'orbit')   # 'yaw_only'|'orbit'|'spiral'
        self.declare_parameter('search_orbit_tilt', 0.12)
        self.declare_parameter('search_orbit_hz', 0.18)
        self.declare_parameter('search_spiral_tilt_min', 0.06)
        self.declare_parameter('search_spiral_tilt_max', 0.18)
        self.declare_parameter('search_spiral_ramp_s', 6.0)
        self.declare_parameter('search_homing_gain', 0.9)   # 0..1
        self.declare_parameter('search_yaw_kp', 0.6)

        # ---- Read parameters ----
        self.px4_topic   = self.get_parameter('px4_topic').get_parameter_value().string_value
        self.vcm_topic   = self.get_parameter('vcm_topic').get_parameter_value().string_value
        self.rate_hz     = float(self.get_parameter('rate_hz').value)
        self.mapping_mode= self.get_parameter('mapping_mode').get_parameter_value().string_value

        self.exclusive   = bool(self.get_parameter('exclusive_mode').value)
        self.idle_hover  = bool(self.get_parameter('idle_hover').value)
        self.data_source = int(self.get_parameter('data_source').value)

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

        self.search_mode = self.get_parameter('search_mode').get_parameter_value().string_value
        self.search_orbit_tilt = float(self.get_parameter('search_orbit_tilt').value)
        self.search_orbit_hz   = float(self.get_parameter('search_orbit_hz').value)
        self.search_spiral_tilt_min = float(self.get_parameter('search_spiral_tilt_min').value)
        self.search_spiral_tilt_max = float(self.get_parameter('search_spiral_tilt_max').value)
        self.search_spiral_ramp_s   = float(self.get_parameter('search_spiral_ramp_s').value)
        self.search_home_gain  = float(self.get_parameter('search_homing_gain').value)
        self.search_yaw_kp     = float(self.get_parameter('search_yaw_kp').value)

        # ---- I/O ----
        self.err_sub = self.create_subscription(Vector3, '/eolab/precision_landing/error', self.err_cb, 10)
        self.vis_sub = self.create_subscription(Bool,   '/eolab/precision_landing/visible', self.vis_cb, 10)
        self.pub     = self.create_publisher(ManualControlSetpoint, self.px4_topic, 10)

        # VehicleControlMode (BEST_EFFORT to match PX4)
        vcm_qos = QoSProfile(depth=1)
        vcm_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        vcm_qos.durability  = QoSDurabilityPolicy.VOLATILE
        self.manual_ok = False
        self.vcm_sub = self.create_subscription(VehicleControlMode, self.vcm_topic, self._vcm_cb, vcm_qos)

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
        self.visible_since = None         # None = invisible, value = time of first visibility
        self.last_visible_time = 0.0

        # Snapshots for log
        self.last_err = Vector3()
        self.last_cmd = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'throttle': 0.10}

        # Engagement
        self.engaged = False

        # Timer
        self.dt = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(self.dt, self.loop)

        # Throttled loggers
        self._last_log_time = self.get_clock().now()
        self._t0 = self.stamp_sec()
        self._last_vcm_warn = self._t0

        # Progress trackers for notch-down
        self._z_last = None
        self._no_progress_since = None

        self.get_logger().info(f"LandingController up: mode={self.mapping_mode} pub→{self.px4_topic}")

    # --- Utils ---
    def stamp_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, s):
        if s != self.state:
            self.get_logger().info(f'[SM] {self.state_name(self.state)} -> {self.state_name(s)}')
            self.state = s
            self.state_enter_time = self.stamp_sec()
            # reset progress trackers each state entry
            self._z_last = None
            self._no_progress_since = None

    def state_name(self, s):
        return ['WAIT', 'SEARCH', 'ALIGN', 'DESCEND', 'FINE_ALIGN', 'LAND'][s]

    # --- Callbacks ---
    def _vcm_cb(self, m: VehicleControlMode):
        self.manual_ok = (bool(m.flag_control_manual_enabled)
                          and not bool(m.flag_control_auto_enabled)
                          and not bool(m.flag_control_offboard_enabled))

    def err_cb(self, msg: Vector3):
        self.err = msg
        self.last_err = msg

    def vis_cb(self, msg: Bool):
        now = self.stamp_sec()
        self.visible = bool(msg.data)
        if self.visible:
            if self.visible_since is None:
                self.visible_since = now
            self.last_visible_time = now
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

        # Stick mapping (PX4 expects [-1..1] here)
        msg.pitch = float(clamp(cmd_x, -1.0, 1.0))   # forward/back
        msg.roll  = float(clamp(cmd_y, -1.0, 1.0))   # left/right
        msg.yaw   = float(clamp(cmd_r, -1.0, 1.0))

        # 0..1 internal throttle -> [-1..1] PX4 stick
        throttle_px4 = 2.0 * float(clamp(cmd_z, 0.0, 1.0)) - 1.0
        msg.throttle = float(clamp(throttle_px4, -1.0, 1.0))

        # Hints for PX4 acceptance / ownership
        msg.valid = True
        msg.data_source = self.data_source

        # Exclusive: always claim "moving" to keep ownership.
        # Non-exclusive: only when actual outputs are non-zero (small eps).
        moving_eps = 1e-3
        moving = (
            abs(msg.pitch) > moving_eps or
            abs(msg.roll)  > moving_eps or
            abs(msg.yaw)   > moving_eps or
            abs(msg.throttle) > moving_eps
        )
        msg.sticks_moving = True if self.exclusive else moving

        msg.flaps = 0.0
        msg.aux1 = msg.aux2 = msg.aux3 = msg.aux4 = msg.aux5 = msg.aux6 = 0.0
        msg.buttons = 0

        self.pub.publish(msg)

        # snapshot for logging
        self.last_cmd['pitch'] = msg.pitch
        self.last_cmd['roll'] = msg.roll
        self.last_cmd['yaw'] = msg.yaw
        self.last_cmd['throttle'] = msg.throttle

    # --- Main control loop ---
    def loop(self):
        now = self.stamp_sec()
        lost = (now - self.last_visible_time) > self.lost_timeout

        # Manual acceptance gate:
        #  - exclusive_mode: keep sending hover to assert ownership even if manual_ok is false
        #  - non-exclusive : do not publish to let QGC/RC own the sticks until mode is correct
        if not self.manual_ok:
            if self.exclusive and self.idle_hover:
                self.publish_mcp(0.0, 0.0, self.hover_thr, 0.0)
            if (now - self._t0) > 2.0 and (now - self._last_vcm_warn) > 1.0:
                self.get_logger().warn('PX4 is not in MANUAL/POSCTL (manual_ok=false).')
                self._last_vcm_warn = now
            if not self.exclusive:
                return  # in non-exclusive, do nothing until manual_ok
            # in exclusive, continue (we already published hover) to keep asserting ownership
            return

        # Not engaged:
        #  - exclusive_mode with idle_hover: publish hover to keep ownership
        #  - otherwise: do nothing (QGC can fly)
        if not self.engaged:
            if self.exclusive and self.idle_hover:
                self.publish_mcp(0.0, 0.0, self.hover_thr, 0.0)
            return

        # defaults
        cmd_x = 0.0   # pitch
        cmd_y = 0.0   # roll
        cmd_r = 0.0   # yaw
        cmd_z = self.hover_thr

        if self.mapping_mode == 'direct':
            if (not self.visible) or lost:
                self.publish_mcp(0.0, 0.0, self.hover_thr, 0.0)
                return

            # Plane alignment: pitch <- -err.z, roll <- +err.x
            cmd_x = clamp(self.kp_xy * (-self.err.z), -self.lim_xy, self.lim_xy)
            cmd_y = clamp(self.kp_xy * ( self.err.x), -self.lim_xy, self.lim_xy)

            # Altitude control around hover (altitude axis = err.y)
            cmd_z = self.hover_thr - self.kp_z * self.err.y

            if abs(cmd_x) < self.dead_xy: cmd_x = 0.0
            if abs(cmd_y) < self.dead_xy: cmd_y = 0.0
            if abs(cmd_z - self.hover_thr) < self.dead_z: cmd_z = self.hover_thr
            cmd_z = clamp(cmd_z, self.lim_z_lo, self.lim_z_hi)

        else:
            # ---- State machine ----
            if self.state == self.STATE_WAIT:
                pass

            elif self.state == self.STATE_SEARCH:
                t = max(0.0, now - self.state_enter_time)

                if self.visible and (self.visible_since is not None):
                    # guided homing while still in SEARCH
                    cmd_x = clamp(self.search_home_gain * self.kp_xy * (-self.err.z), -self.max_tilt, self.max_tilt)
                    cmd_y = clamp(self.search_home_gain * self.kp_xy * ( self.err.x), -self.max_tilt, self.max_tilt)
                    cmd_r = clamp(self.search_yaw_kp * (-self.err.x), -self.max_yaw, self.max_yaw)

                    vis_ok = (now - self.visible_since) >= self.align_dwell
                    if vis_ok and not lost:
                        self.set_state(self.STATE_ALIGN)

                else:
                    cmd_r = clamp(self.search_yaw_rate, -self.max_yaw, self.max_yaw)
                    mode = self.search_mode if self.search_mode in ('orbit','spiral','yaw_only') else 'orbit'

                    if mode == 'orbit':
                        amp = clamp(self.search_orbit_tilt, 0.0, self.max_tilt)
                        hz  = max(0.01, float(self.search_orbit_hz))
                        ang = 2.0 * math.pi * hz * t
                        cmd_x = -amp * math.sin(ang)      # pitch
                        cmd_y =  amp * math.cos(ang)      # roll

                    elif mode == 'spiral':
                        # simple outward ramp
                        k = min(1.0, t / max(0.01, self.search_spiral_ramp_s))
                        a0, a1 = self.search_spiral_tilt_min, self.search_spiral_tilt_max
                        amp = clamp(a0 + (a1 - a0) * k, 0.0, self.max_tilt)
                        hz  = max(0.01, float(self.search_orbit_hz))
                        ang = 2.0 * math.pi * hz * t
                        cmd_x = -amp * math.sin(ang)
                        cmd_y =  amp * math.cos(ang)

                    else:  # yaw_only
                        cmd_x = 0.0
                        cmd_y = 0.0

            elif self.state == self.STATE_ALIGN:
                if lost or not self.visible:
                    cmd_z = self.hover_thr
                    self.set_state(self.STATE_SEARCH)
                else:
                    cmd_x = clamp(self.kp_xy * (-self.err.z), -self.max_tilt, self.max_tilt)
                    cmd_y = clamp(self.kp_xy * ( self.err.x), -self.max_tilt, self.max_tilt)
                    if (abs(self.err.x) < self.coarse_xy) and (abs(self.err.z) < self.coarse_xy):
                        if (now - self.state_enter_time) > self.coarse_dwell:
                            self.set_state(self.STATE_DESCEND)

            elif self.state == self.STATE_DESCEND:
                if lost or not self.visible:
                    cmd_z = self.hover_thr
                    self.set_state(self.STATE_SEARCH)
                else:
                    cmd_x = clamp(self.kp_xy * (-self.err.z), -self.max_tilt, self.max_tilt)
                    cmd_y = clamp(self.kp_xy * ( self.err.x), -self.max_tilt, self.max_tilt)
                    cmd_z = self.descend_thr

                    # proceed to FINE when xy is tight or altitude close
                    if (abs(self.err.x) < self.fine_xy) and (abs(self.err.z) < self.fine_xy):
                        if (now - self.state_enter_time) > self.fine_dwell or (abs(self.err.y) < self.land_z):
                            self.set_state(self.STATE_FINE)

                    # notch-down if altitude progress stalls (altitude axis = err.y)
                    if self._z_last is None:
                        self._z_last = float(self.err.y)
                        self._no_progress_since = now
                    else:
                        if abs(float(self.err.y) - self._z_last) < 0.01:          # < 1 cm progress
                            if (now - self._no_progress_since) > 0.8:             # 0.8 s stall
                                self.descend_thr = max(self.land_thr_min, self.descend_thr - 0.02)
                                cmd_z = self.descend_thr
                                self._no_progress_since = now
                        else:
                            self._z_last = float(self.err.y)
                            self._no_progress_since = now

            elif self.state == self.STATE_FINE:
                if lost or not self.visible:
                    cmd_z = self.hover_thr
                    self.set_state(self.STATE_SEARCH)
                else:
                    cmd_x = clamp(self.kp_xy * (-self.err.z), -self.max_tilt, self.max_tilt)
                    cmd_y = clamp(self.kp_xy * ( self.err.x), -self.max_tilt, self.max_tilt)
                    cmd_z = max(self.land_thr_min, self.descend_thr - 0.03)

                    land_vis_ok = (self.visible and
                                   (self.visible_since is not None) and
                                   ((now - self.visible_since) >= self.land_dwell))

                    if land_vis_ok and (abs(self.err.x) < self.fine_xy) and (abs(self.err.z) < self.fine_xy) and (abs(self.err.y) < self.land_z):
                        self.set_state(self.STATE_LAND)

            elif self.state == self.STATE_LAND:
                if lost or not self.visible:
                    cmd_z = self.hover_thr
                    self.set_state(self.STATE_SEARCH)
                else:
                    # soften lateral commands during final touchdown
                    cmd_x = clamp(self.kp_xy * (-self.err.z), -self.max_tilt, self.max_tilt) * 0.5
                    cmd_y = clamp(self.kp_xy * ( self.err.x), -self.max_tilt, self.max_tilt) * 0.5
                    cmd_z = self.land_thr_min

                    # final push if stuck near ground (watch altitude axis = err.y)
                    if self._z_last is None:
                        self._z_last = float(self.err.y)
                        self._no_progress_since = now
                    else:
                        if abs(float(self.err.y) - self._z_last) < 0.008:         # < 8 mm
                            if (now - self._no_progress_since) > 0.6:             # 0.6 s stall
                                self.land_thr_min = max(0.26, self.land_thr_min - 0.02)
                                cmd_z = self.land_thr_min
                                self._no_progress_since = now
                        else:
                            self._z_last = float(self.err.y)
                            self._no_progress_since = now

        # publish
        self.publish_mcp(cmd_x, cmd_y, cmd_z, cmd_r)

        # 1 Hz log
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
