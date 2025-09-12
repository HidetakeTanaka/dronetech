#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
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
    Precision landing controller for PX4 using ArUco marker feedback.

    Coordinate convention (matches your current bridge):
      roll  <- +err.x
      pitch <- -err.z
      altitude cue <- err.y

    FSM: WAIT → SEARCH → ALIGN → DESCEND → FINE_ALIGN → LAND
    """

    def __init__(self):
        super().__init__('landing_controller')

        # --- Parameters ---
        # Topics
        self.declare_parameter('px4_topic', '/protoflyer/fmu/in/manual_control_input')
        self.declare_parameter('vcm_topic', '/protoflyer/fmu/out/vehicle_control_mode')

        # Rates / mode
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('mapping_mode', 'sm')  # 'sm' | 'direct'

        # Gains
        self.declare_parameter('kp_xy', 1.4)
        self.declare_parameter('kp_z', 0.1)  # (used only in 'direct' mode)

        # Limits
        self.declare_parameter('max_tilt_cmd', 0.75)  # |pitch|,|roll|
        self.declare_parameter('max_yaw_rate', 0.8)

        # Throttle (internal 0..1, converted to [-1..1] at publish)
        self.declare_parameter('hover_throttle', 0.50)
        self.declare_parameter('descend_throttle', 0.34)
        self.declare_parameter('land_throttle_min', 0.22)

        # Thresholds [m] & dwells [s]
        self.declare_parameter('coarse_align_xy', 0.30)
        self.declare_parameter('fine_align_xy', 0.12)
        self.declare_parameter('land_z_thresh', 0.16)
        self.declare_parameter('align_enter_visible_dwell', 0.15)
        self.declare_parameter('coarse_dwell', 0.30)
        self.declare_parameter('fine_dwell', 0.30)

        # Robustness
        self.declare_parameter('lost_timeout', 1.2)        # visibility timeout
        self.declare_parameter('lost_grace_s', 1.5)        # LAND grace window if lost
        self.declare_parameter('land_push_on_loss', True)  # keep pushing if lost in LAND

        # Search behavior
        self.declare_parameter('search_mode', 'orbit')     # 'orbit'|'spiral'|'yaw_only'
        self.declare_parameter('search_orbit_hz', 0.08)
        self.declare_parameter('search_orbit_amp', 0.45)   # big lateral sweep
        self.declare_parameter('search_yaw_rate', 0.8)
        self.declare_parameter('search_spiral_amp_min', 0.18)
        self.declare_parameter('search_spiral_amp_max', 0.50)
        self.declare_parameter('search_spiral_ramp_s', 8.0)

        # PX4 adoption hints
        self.declare_parameter('data_source', 2)           # MAVLINK_0
        self.declare_parameter('idle_hover', True)         # publish hover when disengaged

        # --- Read parameters ---
        gp = self.get_parameter
        self.px4_topic = gp('px4_topic').get_parameter_value().string_value
        self.vcm_topic = gp('vcm_topic').get_parameter_value().string_value

        self.rate_hz  = float(gp('rate_hz').value)
        self.mode     = gp('mapping_mode').get_parameter_value().string_value

        self.kp_xy    = float(gp('kp_xy').value)
        self.kp_z     = float(gp('kp_z').value)
        self.max_tilt = float(gp('max_tilt_cmd').value)
        self.max_yaw  = float(gp('max_yaw_rate').value)

        self.hover_thr    = float(gp('hover_throttle').value)
        self.descend_thr  = float(gp('descend_throttle').value)
        self.land_thr_min = float(gp('land_throttle_min').value)

        self.coarse_xy   = float(gp('coarse_align_xy').value)
        self.fine_xy     = float(gp('fine_align_xy').value)
        self.land_z      = float(gp('land_z_thresh').value)
        self.align_dwell = float(gp('align_enter_visible_dwell').value)
        self.coarse_dwell= float(gp('coarse_dwell').value)
        self.fine_dwell  = float(gp('fine_dwell').value)

        self.lost_timeout   = float(gp('lost_timeout').value)
        self.lost_grace_s   = float(gp('lost_grace_s').value)
        self.land_push_loss = bool(gp('land_push_on_loss').value)

        self.search_mode     = gp('search_mode').get_parameter_value().string_value
        self.search_orbit_hz = float(gp('search_orbit_hz').value)
        self.search_orbit_amp= float(gp('search_orbit_amp').value)
        self.search_yaw_rate = float(gp('search_yaw_rate').value)
        self.search_spiral_amp_min = float(gp('search_spiral_amp_min').value)
        self.search_spiral_amp_max = float(gp('search_spiral_amp_max').value)
        self.search_spiral_ramp_s  = float(gp('search_spiral_ramp_s').value)

        self.data_source = int(gp('data_source').value)
        self.idle_hover  = bool(gp('idle_hover').value)

        # --- I/O ---
        self.err_sub = self.create_subscription(Vector3, '/eolab/precision_landing/error', self.err_cb, 10)
        self.vis_sub = self.create_subscription(Bool,   '/eolab/precision_landing/visible', self.vis_cb, 10)

        # VehicleControlMode with BEST_EFFORT to match PX4
        vcm_qos = QoSProfile(depth=1)
        vcm_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        vcm_qos.durability  = QoSDurabilityPolicy.VOLATILE
        self.vcm_sub = self.create_subscription(VehicleControlMode, self.vcm_topic, self.vcm_cb, vcm_qos)

        # MCP publisher (BEST_EFFORT is fine)
        pub_qos = QoSProfile(depth=1)
        pub_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        pub_qos.durability  = QoSDurabilityPolicy.VOLATILE
        self.pub = self.create_publisher(ManualControlSetpoint, self.px4_topic, pub_qos)

        self.srv = self.create_service(SetBool, '/eolab/landing_controller/start', self.start_srv)

        # --- FSM ---
        self.STATE_WAIT, self.STATE_SEARCH, self.STATE_ALIGN, self.STATE_DESCEND, self.STATE_FINE, self.STATE_LAND = range(6)
        self.state = self.STATE_WAIT
        self.state_enter_time = 0.0

        # Status
        self.err = Vector3()
        self.visible = False
        self.visible_since = None
        self.last_visible_time = 0.0
        self.manual_ok = False
        self.engaged = False

        # Helpers
        self._t0 = self.stamp_sec()
        self._last_log_time = self.get_clock().now()
        self._alt_last = None
        self._no_progress_since = None
        self._land_loss_since = None

        self.last_cmd = {'roll':0.0,'pitch':0.0,'yaw':0.0,'throttle':0.0}

        self.dt = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(self.dt, self.loop)

        self.get_logger().info("LandingController initialized")

    # --- Utils ---
    def stamp_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, s):
        if s != self.state:
            self.get_logger().info(f"[SM] {self.state_name(self.state)} -> {self.state_name(s)}")
            self.state = s
            self.state_enter_time = self.stamp_sec()
            self._alt_last = None
            self._no_progress_since = None
            if s == self.STATE_LAND:
                self._land_loss_since = None

    def state_name(self, s):
        return ['WAIT','SEARCH','ALIGN','DESCEND','FINE_ALIGN','LAND'][s]

    # --- Callbacks ---
    def vcm_cb(self, m: VehicleControlMode):
        # Require MANUAL/POSCTL (manual true, auto false, offboard false)
        self.manual_ok = (bool(m.flag_control_manual_enabled)
                          and not bool(m.flag_control_auto_enabled)
                          and not bool(m.flag_control_offboard_enabled))

    def err_cb(self, msg: Vector3):
        self.err = msg

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
            vis_ok = self.visible and (self.visible_since is not None) and \
                     ((self.stamp_sec() - self.visible_since) >= self.align_dwell)
            self.set_state(self.STATE_ALIGN if vis_ok else self.STATE_SEARCH)
            res.success, res.message = True, 'Autonomous mode engaged.'
        else:
            self.set_state(self.STATE_WAIT)
            res.success, res.message = True, 'Autonomous mode disengaged.'
        return res

    # --- Publish helper (all floats!) ---
    def publish_mcp(self, pitch, roll, throttle_01, yaw):
        msg = ManualControlSetpoint()
        t_us = int(self.get_clock().now().nanoseconds / 1000)
        msg.timestamp = t_us
        msg.timestamp_sample = t_us

        msg.pitch = float(clamp(pitch, -1.0, 1.0))
        msg.roll  = float(clamp(roll,  -1.0, 1.0))
        msg.yaw   = float(clamp(yaw,   -1.0, 1.0))

        thr_px4 = 2.0 * float(clamp(throttle_01, 0.0, 1.0)) - 1.0
        msg.throttle = float(clamp(thr_px4, -1.0, 1.0))

        msg.valid = True
        msg.data_source = int(self.data_source)
        msg.sticks_moving = True

        msg.flaps = 0.0
        msg.aux1 = msg.aux2 = msg.aux3 = msg.aux4 = msg.aux5 = msg.aux6 = 0.0
        msg.buttons = 0

        self.pub.publish(msg)
        self.last_cmd.update({'pitch':msg.pitch,'roll':msg.roll,'yaw':msg.yaw,'throttle':msg.throttle})

    # --- Main loop ---
    def loop(self):
        now = self.stamp_sec()
        lost = (now - self.last_visible_time) > self.lost_timeout

        # Require PX4 to accept manual sticks
        if not self.manual_ok:
            if self.idle_hover:
                self.publish_mcp(0.0, 0.0, self.hover_thr, 0.0)
            return

        if not self.engaged:
            if self.idle_hover:
                self.publish_mcp(0.0, 0.0, self.hover_thr, 0.0)
            return

        # Defaults
        pitch = 0.0; roll = 0.0; yaw = 0.0; thr = self.hover_thr

        # --- LAND (robust: do not bounce back to SEARCH) ---
        if self.state == self.STATE_LAND:
            if self.visible:
                pitch = clamp(self.kp_xy * (-self.err.z), -self.max_tilt, self.max_tilt) * 0.5
                roll  = clamp(self.kp_xy * ( self.err.x), -self.max_tilt, self.max_tilt) * 0.5
                thr   = self.land_thr_min
                self._land_loss_since = None
            else:
                if self._land_loss_since is None:
                    self._land_loss_since = now
                # Keep pushing for a grace window even if marker is lost
                if self.land_push_loss and (now - self._land_loss_since) < self.lost_grace_s:
                    pitch = 0.0; roll = 0.0; thr = self.land_thr_min
                else:
                    pitch = 0.0; roll = 0.0; thr = self.land_thr_min
            self.publish_mcp(pitch, roll, thr, yaw)
            self._log_1hz()
            return

        # --- SEARCH ---
        if self.state == self.STATE_SEARCH:
            t = max(0.0, now - self.state_enter_time)
            yaw = clamp(self.search_yaw_rate, -self.max_yaw, self.max_yaw)

            if self.search_mode == 'yaw_only':
                pitch = 0.0; roll = 0.0
            else:
                ang = 2.0 * math.pi * max(0.01, self.search_orbit_hz) * t
                if self.search_mode == 'orbit':
                    amp = clamp(self.search_orbit_amp, 0.0, self.max_tilt)
                else:  # spiral
                    k = min(1.0, t / max(0.01, self.search_spiral_ramp_s))
                    raw = self.search_spiral_amp_min + (self.search_spiral_amp_max - self.search_spiral_amp_min) * k
                    amp = clamp(raw, 0.0, self.max_tilt)
                pitch = -amp * math.sin(ang)
                roll  =  amp * math.cos(ang)

            vis_ok = (self.visible and (self.visible_since is not None) and
                      ((now - self.visible_since) >= self.align_dwell))
            if vis_ok and not lost:
                self.set_state(self.STATE_ALIGN)

        # --- ALIGN ---
        elif self.state == self.STATE_ALIGN:
            if lost or not self.visible:
                self.set_state(self.STATE_SEARCH)
            else:
                pitch = clamp(self.kp_xy * (-self.err.z), -self.max_tilt, self.max_tilt)
                roll  = clamp(self.kp_xy * ( self.err.x), -self.max_tilt, self.max_tilt)
                thr   = self.hover_thr

                if (abs(self.err.x) < self.coarse_xy) and (abs(self.err.z) < self.coarse_xy):
                    if (now - self.state_enter_time) > self.coarse_dwell:
                        self.set_state(self.STATE_DESCEND)

        # --- DESCEND ---
        elif self.state == self.STATE_DESCEND:
            if lost or not self.visible:
                self.set_state(self.STATE_SEARCH)
            else:
                pitch = clamp(self.kp_xy * (-self.err.z), -self.max_tilt, self.max_tilt)
                roll  = clamp(self.kp_xy * ( self.err.x), -self.max_tilt, self.max_tilt)
                thr   = self.descend_thr

                # Notch-down if altitude stalls (use err.y as altitude cue)
                if self._alt_last is None:
                    self._alt_last = float(self.err.y); self._no_progress_since = now
                else:
                    if abs(float(self.err.y) - self._alt_last) < 0.01:  # < 1 cm progress
                        if (now - self._no_progress_since) > 0.8:       # 0.8 s stall
                            self.descend_thr = max(self.land_thr_min, self.descend_thr - 0.02)
                            thr = self.descend_thr
                            self._no_progress_since = now
                    else:
                        self._alt_last = float(self.err.y)
                        self._no_progress_since = now

                # Go FINE when centered or close in altitude
                if (abs(self.err.x) < self.fine_xy and abs(self.err.z) < self.fine_xy) \
                   or (abs(self.err.y) < self.land_z):
                    if (now - self.state_enter_time) > self.fine_dwell:
                        self.set_state(self.STATE_FINE)

        # --- FINE_ALIGN (small corrections, prepare to LAND) ---
        elif self.state == self.STATE_FINE:
            if lost or not self.visible:
                self.set_state(self.STATE_SEARCH)
            else:
                pitch = clamp(self.kp_xy * (-self.err.z), -self.max_tilt, self.max_tilt)
                roll  = clamp(self.kp_xy * ( self.err.x), -self.max_tilt, self.max_tilt)
                thr   = max(self.land_thr_min, self.descend_thr - 0.03)

                # Enter LAND when tight on XY and close enough in Z
                if (abs(self.err.x) < self.fine_xy) and (abs(self.err.z) < self.fine_xy) and \
                   (abs(self.err.y) < self.land_z):
                    self.set_state(self.STATE_LAND)

        # Publish
        self.publish_mcp(pitch, roll, thr, yaw)
        self._log_1hz()

    def _log_1hz(self):
        now_ros = self.get_clock().now()
        if (now_ros - self._last_log_time).nanoseconds * 1e-9 > 1.0:
            self.get_logger().info(
                f"state={self.state_name(self.state)} vis={self.visible} "
                f"err=({float(self.err.x):.2f},{float(self.err.y):.2f},{float(self.err.z):.2f}) "
                f"cmd(roll,pitch,yaw,thr)=({self.last_cmd['roll']:.2f},"
                f"{self.last_cmd['pitch']:.2f},{self.last_cmd['yaw']:.2f},"
                f"{self.last_cmd['throttle']:.2f})"
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
