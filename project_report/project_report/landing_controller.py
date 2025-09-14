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

from rclpy.exceptions import ParameterAlreadyDeclaredException, ParameterNotDeclaredException
from px4_msgs.msg import VehicleCommand


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
    def _declare_if_needed(self, name, default):
        try:
            self.declare_parameter(name, default)
        except ParameterAlreadyDeclaredException:
            # Ignore if already declared
            pass

    def _pf(self, name: str, default: float) -> float:
        self._declare_if_needed(name, default)
        return float(self.get_parameter(name).value)

    def _pb(self, name: str, default: bool) -> bool:
        self._declare_if_needed(name, default)
        return bool(self.get_parameter(name).value)

    def _pi(self, name: str, default: int) -> int:
        self._declare_if_needed(name, default)
        return int(self.get_parameter(name).value)

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
        self.declare_parameter('search_orbit_amp', 0.45)   # large lateral sweep
        self.declare_parameter('search_yaw_rate', 0.8)
        self.declare_parameter('search_spiral_amp_min', 0.18)
        self.declare_parameter('search_spiral_amp_max', 0.50)
        self.declare_parameter('search_spiral_ramp_s', 8.0)

        # PX4 adoption hints
        self.declare_parameter('data_source', 2)           # MAVLINK_0
        self.declare_parameter('idle_hover', True)         # publish hover when disengaged

        self.declare_parameter('search_timeout_s', 60.0)   # Search timeout (seconds)
        self.declare_parameter('safe_action', 'rtl')       # 'rtl' | 'hover' | 'disengage'

        # --- Read parameters ---
        gp = self.get_parameter
        self.px4_topic = gp('px4_topic').get_parameter_value().string_value
        self.vcm_topic = gp('vcm_topic').get_parameter_value().string_value

        self.rate_hz  = float(gp('rate_hz').value)
        self.mode     = gp('mapping_mode').get_parameter_value().string_value

        self.search_timeout_s = float(self.get_parameter('search_timeout_s').value)
        self.safe_action      = self.get_parameter('safe_action').get_parameter_value().string_value

        # self.yaw_freeze_xy = float(self.get_parameter_or('yaw_freeze_xy', 0.08))
        self.yaw_freeze_xy   = self._pf('yaw_freeze_xy', 0.08)
        self.kp_xy           = self._pf('kp_xy', 1.2)
        self.kp_yaw          = self._pf('kp_yaw', 0.6)
        self.kp_z            = float(gp('kp_z').value)
        self.max_yaw         = float(gp('max_yaw_rate').value)

        self.max_tilt        = self._pf('max_tilt_cmd', 0.60)
        self.max_yaw_rate    = self._pf('max_yaw_rate', 0.25)
        self.hover_thr       = self._pf('hover_throttle', 0.55)
        self.descend_thr     = self._pf('descend_throttle', 0.47)
        self.land_thr_min    = self._pf('land_throttle_min', 0.40)

        self.coarse_xy   = float(gp('coarse_align_xy').value)

        # Normalize naming differences (support both CLI/legacy names)
        self.coarse_align_xy = self._pf('coarse_align_xy', 0.30)
        self.fine_xy         = self._pf('fine_align_xy', 0.12)
        self.land_z          = self._pf('land_z_thresh', 0.16)

        self.align_dwell     = self._pf('align_enter_visible_dwell', 0.10)
        self.coarse_dwell    = self._pf('coarse_dwell', 0.25)
        self.fine_dwell      = self._pf('fine_dwell', 0.25)

        self.lost_timeout    = self._pf('lost_timeout', 1.0)
        self.lost_grace_s    = float(gp('lost_grace_s').value)
        self.land_push_loss  = bool(gp('land_push_on_loss').value)

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
        self.last_visible_time = None
        self.manual_ok = False
        self.engaged = False

        self.engage_time    = None
        self.ever_visible   = False
        self.safe_triggered = False

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

        self._vis_streak = 0
        self._no_vis_streak = 0
        self.visible_locked = False  # True after N consecutive visible frames
        self.lost_hard = False       # True after M consecutive lost frames
        self.visible_N = 3           # e.g., 3 frames
        self.lost_M = 6              # e.g., 6 frames

        self.flip_x_cmd      = self._pb('flip_x_cmd', False)
        self.flip_z_cmd      = self._pb('flip_z_cmd', False)
        self.search_mode_orbit = True  # if you later branch on mode
        # Toggle Z gate via parameter (default OFF)
        self.use_z_gate = self._pb('use_z_gate', False)

        self.homing_boost   = self._pf('homing_boost', 1.3)   # 1.0–1.6
        self.homing_boost_s = self._pf('homing_boost_s', 0.8) # seconds
        self.min_lateral_cmd = self._pf('min_lateral_cmd', 0.10)  # minimum lateral cmd magnitude (stick domain -1..1)

        # VehicleCommand publisher (RELIABLE delivery)
        cmd_qos = QoSProfile(depth=5)
        cmd_qos.reliability = QoSReliabilityPolicy.RELIABLE
        cmd_qos.durability  = QoSDurabilityPolicy.VOLATILE
        # Parametrize topic to avoid namespace mismatch
        self.declare_parameter('cmd_topic', '/protoflyer/fmu/in/vehicle_command')
        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.cmd_pub = self.create_publisher(VehicleCommand, self.cmd_topic, cmd_qos)

        self._safe_cmd_until = None
        self._safe_cmd_last  = 0.0
        self._safe_cmd_rate  = 10.0
        self._safe_fallback_after = 2.0  # fallback to LAND if RTL not accepted in 2s

    def _param(self, name, default):
        try:
            self.declare_parameter(name, default)
        except Exception:
            pass
        v = self.get_parameter(name).value
        return default if v is None else v

    def _param_any(self, names, default):
        """Return the first set parameter among names (alias support)."""
        for n in names:
            try:
                self.declare_parameter(n, default)
            except Exception:
                pass
            v = self.get_parameter(n).value
            if v is not None:
                return v
        return default

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
        v = bool(msg.data)
        self.visible = v
        if v:
            self.ever_visible = True  # mark true once seen at least once
            if self.visible_since is None:
                self.visible_since = now  # start time of consecutive visibility
            self.last_visible_time = now  # last time we saw the marker (for lost timeout)
        else:
            self.visible_since = None     # reset consecutive visibility

    def start_srv(self, req: SetBool.Request, res: SetBool.Response):
        self.engaged = bool(req.data)
        if self.engaged:
            self.engage_time = self.stamp_sec()  # start timeout tracking
            self.safe_triggered = False
            vis_ok = self.visible and (self.visible_since is not None) and \
                     ((self.stamp_sec() - self.visible_since) >= self.align_dwell)
            self.set_state(self.STATE_ALIGN if vis_ok else self.STATE_SEARCH)
            res.success, res.message = True, 'Autonomous mode engaged.'
        else:
            self.engage_time = None
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

    def send_vehicle_command(self, command: int, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        t_us = int(self.get_clock().now().nanoseconds / 1000)
        msg.timestamp = t_us
        msg.param1, msg.param2, msg.param3 = float(param1), float(param2), float(param3)
        msg.param4, msg.param5, msg.param6, msg.param7 = float(param4), float(param5), float(param6), float(param7)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def trigger_safe_mode(self, reason:str):
        if self.safe_triggered:
            return
        self.safe_triggered = True
        self.get_logger().warn(f"[SAFE] Triggered ({reason}). Action={self.safe_action}")

        if self.safe_action.lower() == 'rtl':
            # MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
            MAV_CMD_NAV_RETURN_TO_LAUNCH = getattr(VehicleCommand, 'VEHICLE_CMD_NAV_RETURN_TO_LAUNCH', 20)
            self.send_vehicle_command(MAV_CMD_NAV_RETURN_TO_LAUNCH)
            # Resend at 10 Hz for 1.5 s
            now = self.stamp_sec()
            self._safe_cmd_until = now + 1.5
            self._safe_cmd_last  = 0.0
        elif self.safe_action.lower() == 'hover':
            # Do not interfere: stop sending and let PX4 hold current mode
            pass
        elif self.safe_action.lower() == 'disengage':
            self.engaged = False  # disable controller

        # While SAFE, stop sending MCP sticks
        self.idle_hover = False

    # Helper: landing corridor check
    def in_landing_corridor(self):
        now = self.stamp_sec()
        xy_ok  = (abs(float(self.err.x)) < self.fine_xy) and (abs(float(self.err.z)) < self.fine_xy)
        vis_ok = self.visible and (self.visible_since is not None) and ((now - self.visible_since) >= self.fine_dwell)
        if self.use_z_gate:
            z_ok = abs(float(self.err.y)) < self.land_z
            return xy_ok and vis_ok and z_ok
        else:
            return xy_ok and vis_ok

    def lateral_cmd_vec(self, ex: float, ez: float, use_boost: bool = True):
        """
        Assign roll/pitch toward error vector (ex, ez):
        - Preserve direction via vector normalization
        - Initial boost to speed up homing
        - Minimum vector magnitude floor to avoid deadband
        - Saturate the vector to max_tilt_cmd for safety
        """
        # Base gain (apply boost during first few hundred ms)
        k = self.kp_xy
        if use_boost and (self.stamp_sec() - self.state_enter_time) < self.homing_boost_s:
            k *= self.homing_boost

        # Pre-saturation targets (axis-wise P)
        ux = k * ex         # → roll
        uz = k * (-ez)      # → pitch (forward = +pitch)

        mag = math.hypot(ux, uz)
        if mag < 1e-6:
            return 0.0, 0.0

        # Vector min-magnitude floor (ensure actuation when r is large)
        r = math.hypot(ex, ez)
        if r > self.fine_xy and mag < self.min_lateral_cmd:
            scale = self.min_lateral_cmd / mag
            ux *= scale; uz *= scale
            mag = self.min_lateral_cmd

        # Vector saturation (preserve direction while clamping to max_tilt_cmd)
        max_mag = self.max_tilt
        if mag > max_mag:
            scale = max_mag / mag
            ux *= scale; uz *= scale

        # Final clamp (safety)
        roll  = clamp(ux, -self.max_tilt, self.max_tilt)
        pitch = clamp(uz, -self.max_tilt, self.max_tilt)
        return roll, pitch

    def _is_auto_enabled(self) -> bool:
        # Check whether AUTO is enabled (based on last VehicleControlMode)
        try:
            return bool(self._last_vcm.flag_control_auto_enabled)
        except Exception:
            return False

    def vcm_cb(self, m: VehicleControlMode):
        self._last_vcm = m
        self.manual_ok = (bool(m.flag_control_manual_enabled)
                        and not bool(m.flag_control_auto_enabled)
                        and not bool(m.flag_control_offboard_enabled))

    # --- Main loop ---
    def loop(self):
        now = self.stamp_sec()

        ex = float(self.err.x)
        ez = float(self.err.z)

        # lost = (now - self.last_visible_time) > self.lost_timeout
        lost = (self.last_visible_time is None) or ((now - self.last_visible_time) > self.lost_timeout)

        # SAFE: timeout since engage & never seen
        if (not self.safe_triggered) and self.engaged and (self.engage_time is not None):
            if (now - self.engage_time) >= self.search_timeout_s and (not self.ever_visible):
                self.trigger_safe_mode("search timeout without detection")

        if self.safe_triggered:
            # Resend RTL at 10 Hz within the window
            if self._safe_cmd_until is not None and now < self._safe_cmd_until:
                if (now - self._safe_cmd_last) > (1.0 / self._safe_cmd_rate):
                    self.send_vehicle_command(getattr(VehicleCommand,'VEHICLE_CMD_NAV_RETURN_TO_LAUNCH',20))
                    self._safe_cmd_last = now

            # Acceptance check: once AUTO is enabled, stop resending
            if self.manual_ok is False or self._is_auto_enabled():
                return

            # If still not AUTO after 2 s, fallback to AUTO.LAND (indoor/home-invalid fallback)
            if (self._safe_cmd_until is not None) and (now > self._safe_cmd_until + self._safe_fallback_after):
                self.get_logger().warn("[SAFE] RTL not accepted → fallback to AUTO.LAND")
                self.send_vehicle_command(getattr(VehicleCommand,'VEHICLE_CMD_NAV_LAND',21))
                return
            return

        # Require PX4 to accept manual sticks
        if not self.manual_ok:
            if self.safe_triggered:
                return
            if self.idle_hover:
                self.publish_mcp(0.0, 0.0, self.hover_thr, 0.0)
            return

        if not self.engaged:
            if self.safe_triggered:
                return
            if self.idle_hover:
                self.publish_mcp(0.0, 0.0, self.hover_thr, 0.0)
            return

        # Defaults
        pitch = 0.0; roll = 0.0; yaw = 0.0; thr = self.hover_thr

        # --- LAND (robust: do not bounce back to SEARCH) ---
        if self.state == self.STATE_LAND:
            if self.visible:
                # roll, pitch from vector control (half gains previously are now handled via lateral_cmd_vec / params)
                roll, pitch = self.lateral_cmd_vec(ex, ez, use_boost=True)
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
        elif self.state == self.STATE_SEARCH:
            now = self.stamp_sec()
            if self.visible:
                # When visible: stop search and home (lateral P + yaw control)
                roll, pitch = self.lateral_cmd_vec(ex, ez, use_boost=True)

                # Gentle yaw based on bearing
                bearing = math.atan2(float(self.err.x), float(self.err.z))
                r = math.hypot(float(self.err.x), float(self.err.z))
                yaw = 0.0 if r < self.yaw_freeze_xy else clamp(self.kp_yaw * bearing, -self.max_yaw_rate, self.max_yaw_rate)
                # If visible continuously for a while → ALIGN
                if (self.visible_since is not None) and ((now - self.visible_since) >= self.align_dwell):
                    self.set_state(self.STATE_ALIGN)
            else:
                # Small-orbit search
                ang = 2.0 * math.pi * max(0.01, self.search_orbit_hz) * (now - self.state_enter_time)
                amp = min(self.max_tilt, self.search_orbit_amp)
                pitch = -amp * math.sin(ang)
                roll  =  amp * math.cos(ang)
                yaw   = self.search_yaw_rate

        # --- ALIGN ---
        elif self.state == self.STATE_ALIGN:
            if lost or not self.visible:
                self.set_state(self.STATE_SEARCH)
            else:
                ex = float(self.err.x)
                ez = float(self.err.z)
                r  = math.hypot(ex, ez)

                # Lateral control
                roll, pitch = self.lateral_cmd_vec(ex, ez, use_boost=True)

                thr   = self.hover_thr

                # Yaw: freeze to 0 inside coarse_align_xy.
                # Outside, scale by r to avoid sudden full yaw command.
                r_freeze = self.coarse_align_xy  # capture radius
                if r < r_freeze:
                    yaw = 0.0
                else:
                    bearing = math.atan2(ex, ez)
                    scale   = min(1.0, max(0.0, (r - r_freeze) / r_freeze))  # 0..1
                    yaw     = clamp(self.kp_yaw * scale * bearing,
                                    -self.max_yaw_rate, self.max_yaw_rate)

                if (r < self.coarse_align_xy) and ((now - self.state_enter_time) > self.coarse_dwell):
                    self.set_state(self.STATE_DESCEND)

        # --- DESCEND ---
        elif self.state == self.STATE_DESCEND:
            if lost or not self.visible:
                self.set_state(self.STATE_SEARCH)
            else:
                ex = float(self.err.x)
                ez = float(self.err.z)
                r  = math.hypot(ex, ez)

                # Lateral control
                roll, pitch = self.lateral_cmd_vec(ex, ez, use_boost=True)

                # Yaw: same rule as ALIGN (freeze inside capture radius; scale outside)
                r_freeze = self.coarse_align_xy
                if r < r_freeze:
                    yaw = 0.0
                else:
                    bearing = math.atan2(ex, ez)
                    scale   = min(1.0, max(0.0, (r - r_freeze) / r_freeze))
                    yaw     = clamp(self.kp_yaw * scale * bearing,
                                    -self.max_yaw_rate, self.max_yaw_rate)

                # Glide-slope throttle: more descent as r shrinks
                r_gate = max(1e-3, self.coarse_align_xy)
                desc_allow = max(0.0, min(1.0, 1.0 - (r / r_gate)))  # 0..1
                thr = self.hover_thr - (self.hover_thr - self.descend_thr) * desc_allow

                # Notch-down if altitude stalls (recompute thr after updates)
                if self._alt_last is None:
                    self._alt_last = float(self.err.y); self._no_progress_since = now
                else:
                    if abs(float(self.err.y) - self._alt_last) < 0.01:  # <1 cm
                        if (now - self._no_progress_since) > 0.8:
                            self.descend_thr = max(self.land_thr_min, self.descend_thr - 0.02)
                            thr = self.hover_thr - (self.hover_thr - self.descend_thr) * desc_allow
                            self._no_progress_since = now
                    else:
                        self._alt_last = float(self.err.y)
                        self._no_progress_since = now

                # Go FINE when XY is tight (with dwell)
                xy_tight = (abs(float(self.err.x)) < self.fine_xy) and (abs(float(self.err.z)) < self.fine_xy)
                if xy_tight and ((now - self.state_enter_time) > self.fine_dwell):
                    self.set_state(self.STATE_FINE)

        # --- FINE_ALIGN (small corrections, prepare to LAND) ---
        elif self.state == self.STATE_FINE:
            if lost or not self.visible:
                self.set_state(self.STATE_SEARCH)
            else:
                ex = float(self.err.x)
                ez = float(self.err.z)
                r  = math.hypot(ex, ez)

                # Lateral control: +pitch when marker ahead (pitch = -ez with current convention)
                roll, pitch = self.lateral_cmd_vec(ex, ez, use_boost=True)

                # Yaw: freeze near center to prevent sideways drift while yawing
                if r < self.yaw_freeze_xy:
                    yaw = 0.0
                else:
                    yaw = clamp(self.kp_yaw * math.atan2(ex, ez),
                                -self.max_yaw_rate, self.max_yaw_rate)

                # Glide slope: descend more as XY radius shrinks
                r_gate = max(1e-3, self.fine_xy)
                desc_allow = max(0.0, min(1.0, 1.0 - (r / r_gate)))  # 0..1

                # Target throttle biased to descend (lower as we approach LAND)
                target = max(self.land_thr_min, self.descend_thr - 0.03)
                thr = self.hover_thr - (self.hover_thr - target) * desc_allow

                # If very close, push below hover band to ensure descent
                if r < (0.5 * self.fine_xy):
                    thr = min(thr, target - 0.02)   # e.g., descend=0.45 → 0.43

                # "Never hover" safety margin (avoid stalling near hover)
                thr = min(thr, self.hover_thr - 0.02)

                # Close-range fallback: near ground visibility can flicker → push a bit lower
                try:
                    if abs(float(self.err.y)) < (self.land_z * 0.6):
                        thr = min(thr, self.descend_thr - 0.04)
                except Exception:
                    pass

                # Clamp throttle to [0, 1]
                thr = clamp(thr, 0.0, 1.0)

                # If corridor satisfied (visible dwell + XY ≤ fine (+ optional Z)), go LAND
                if self.in_landing_corridor():
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
