# Section 3 – Drone Programming

> **Submission status (today)**
> We are still integrating and tuning the autonomous precision landing. If it does not reliably touch down by **22:00** today, we will submit this as an **incomplete prototype**: design, rationale, partial results, limitations, and next steps are documented below.

---

## 3.1 Overview

This section explains the software implementation of **precision landing on an ArUco marker** using **ROS 2 ↔ PX4**. We describe the topic pipeline (detector → bridge → controller), the **finite-state machine (FSM)** controller, and two key design choices added during integration:

1. **Guided-search** (home toward a just-visible marker),
2. **Exclusive stick ownership** (our node publishes ManualControlSetpoint continuously so PX4 selects our inputs over QGC/RC).

We conclude with results, known limitations, and concrete improvements.

---

## 3.2 Architecture (ROS 2 / PX4)

**Nodes & topics**

* **Detector** `eolab_precision_landing/detector`

  * Sub: `/protoflyer/image`, `/protoflyer/camera_info`
  * Pub: `/protoflyer/detected_aruco_markers` (\~30 Hz), `/protoflyer/aruco_debug_image`

* **Bridge** `project_report/marker_to_error`

  * Sub: `/protoflyer/detected_aruco_markers`
  * Pub:
    `/protoflyer/eolab/precision_landing/visible` *(std\_msgs/Bool)*,
    `/protoflyer/eolab/precision_landing/error` *(geometry\_msgs/Vector3, meters)*
  * Effective params: `scale=0.04`, `flip_z=true`, `forward_axis="z"`, `altitude_axis="y"`, `smooth_n=3`

* **Controller** `project_report/landing_controller`

  * Sub: `/protoflyer/eolab/precision_landing/{visible,error}`, `/protoflyer/fmu/out/vehicle_control_mode`
  * Pub: `/protoflyer/fmu/in/manual_control_input` *(px4\_msgs/ManualControlSetpoint)*
  * Srv: `/eolab/landing_controller/start` *(std\_srvs/SetBool)*

**Axis mapping (after the bridge)**

* Lateral: `roll  ← +err.x`, `pitch ← -err.z`
* Altitude: `throttle` is controlled around hover using `err.y` (positive = “too high”).

---

## 3.3 Mission logic (FSM)

`WAIT → SEARCH → ALIGN → DESCEND → FINE_ALIGN → LAND`

* **SEARCH** – Yaw scan + small **orbit/spiral**. If marker becomes visible, **guided-homing** (proportional roll/pitch toward the marker) runs immediately; once visible for `align_dwell` seconds, transition to ALIGN.
* **ALIGN** – Lateral P control using `kp_xy`, clipped by `max_tilt_cmd` until within `coarse_align_xy`.
* **DESCEND** – Apply `descend_throttle` (below hover) while maintaining lateral centering; if within `fine_align_xy` (or close in altitude), go to FINE\_ALIGN.
* **FINE\_ALIGN** – Tight lateral bounds, slightly lower throttle.
* **LAND** – Use `land_throttle_min` (floor); lateral commands softened. A **notch-down** routine reduces throttle further if altitude progress stalls near ground.

Safety: if the marker is lost longer than `lost_timeout`, the controller reverts to **SEARCH** and commands **hover**.

---

## 3.4 Implementation highlights

### A) Exclusive stick ownership (PX4 acceptance)

To prevent QGC/RC from overriding descent, the controller **always publishes** (even when not engaged, it can send hover if `idle_hover=true`) with:

* `data_source = MAVLINK_0 (2)`
* `sticks_moving = true`
* High rate: `rate_hz = 75–100`

This reliably makes `/fmu/out/manual_control_setpoint` follow our `/fmu/in/manual_control_input`.

**Snippet (publishing sticks)**

```py
# 0..1 -> [-1..1] mapping for PX4 throttle stick
throttle_px4 = 2.0 * clamp(cmd_z, 0.0, 1.0) - 1.0

msg = ManualControlSetpoint()
msg.pitch    = clamp(cmd_x, -1.0, 1.0)
msg.roll     = clamp(cmd_y, -1.0, 1.0)
msg.yaw      = clamp(cmd_r, -1.0, 1.0)
msg.throttle = clamp(throttle_px4, -1.0, 1.0)

msg.valid = True
msg.data_source = 2          # MAVLINK_0
msg.sticks_moving = True     # keep ownership
pub.publish(msg)
```

### B) Guided-search (snappier acquisition)

During **SEARCH**, if the marker is even briefly visible, we immediately bias roll/pitch toward the marker (plus mild yaw centering). This reduces “spinning in place”.

**Snippet (SEARCH state)**

```py
if visible:
    cmd_x = clamp(kp_xy * homing_gain * (-err.z), -max_tilt, max_tilt)
    cmd_y = clamp(kp_xy * homing_gain * ( err.x), -max_tilt, max_tilt)
    cmd_r = clamp(search_yaw_kp * (-err.x), -max_yaw, max_yaw)
    if visible_for >= align_dwell:
        transition(ALIGN)
else:
    # orbit scan
    ang = 2*pi*orbit_hz*t
    cmd_x = -orbit_tilt * sin(ang)
    cmd_y =  orbit_tilt * cos(ang)
    cmd_r = clamp(search_yaw_rate, -max_yaw, max_yaw)
```

### C) Stall-aware descent near ground

If altitude (from `err.y`) does not improve for a short dwell, we **notch down** `descend_throttle` (never below `land_throttle_min`) to ensure continued descent.

**Snippet (DESCEND → notch-down)**

```py
cmd_z = descend_throttle
if abs(err.y - last_alt_err) < 0.01 and stalled_for > 0.8:  # <1 cm in 0.8 s
    descend_throttle = max(land_throttle_min, descend_throttle - 0.02)
    cmd_z = descend_throttle
```

---

## 3.5 Key parameters (typical working ranges)

* **Rates & ownership:** `rate_hz=75–100`, `exclusive_mode=true`, `idle_hover=true`, `data_source=2`
* **Lateral:** `kp_xy=1.2–1.6`, `max_tilt_cmd=0.7–0.8`
* **Throttle (internal 0..1 → PX4 \[-1..1]):**
  `hover_throttle≈0.48–0.55`, `descend_throttle≈0.30–0.36`, `land_throttle_min≈0.20–0.24`
* **Thresholds & dwells:**
  `coarse_align_xy≈0.30`, `fine_align_xy≈0.12–0.16`, `land_z_thresh≈0.16–0.18`,
  `align_enter_visible_dwell≈0.15–0.25`, `coarse_dwell≈0.3`, `fine_dwell≈0.3`
* **Search & homing:** `search_mode=orbit`, `search_orbit_tilt≈0.16`, `search_orbit_hz≈0.20`,
  `search_yaw_rate≈0.6`, `search_homing_gain≈0.9`, `search_yaw_kp≈0.6`
* **Bridge:** `scale=0.04`, `flip_z=true`, `forward_axis="z"`, `altitude_axis="y"`, `smooth_n=3`

---

## 3.6 How to run (reproducible steps)

```bash
# 1) PX4/Gazebo with ArUco world
ros2 launch eolab_bringup start.launch.py world:=aruco

# 2) ArUco detector
ros2 run eolab_precision_landing detector --ros-args -r __ns:=/protoflyer

# 3) Vision→error bridge (fast response)
ros2 run project_report marker_to_error --ros-args \
  -p markers_topic:=/protoflyer/detected_aruco_markers \
  -p visible_topic:=/protoflyer/eolab/precision_landing/visible \
  -p error_topic:=/protoflyer/eolab/precision_landing/error \
  -p scale:=0.04 -p flip_z:=true \
  -p 'forward_axis:="z"' -p 'altitude_axis:="y"' \
  -p smooth_n:=3

# 4) Controller (exclusive ownership + guided-search)
ros2 run project_report landing_controller --ros-args \
  -p exclusive_mode:=true -p idle_hover:=true -p data_source:=2 \
  -p rate_hz:=100 \
  -p mapping_mode:=sm \
  -p search_mode:=orbit -p search_orbit_tilt:=0.16 -p search_orbit_hz:=0.20 \
  -p search_yaw_rate:=0.6 -p search_homing_gain:=0.9 -p search_yaw_kp:=0.6 \
  -p kp_xy:=1.4 -p max_tilt_cmd:=0.7 \
  -p align_enter_visible_dwell:=0.2 -p coarse_dwell:=0.3 -p fine_dwell:=0.3 \
  -p descend_throttle:=0.34 -p land_throttle_min:=0.22 \
  -p fine_align_xy:=0.12 -p land_z_thresh:=0.16 \
  -r /eolab/precision_landing/visible:=/protoflyer/eolab/precision_landing/visible \
  -r /eolab/precision_landing/error:=/protoflyer/eolab/precision_landing/error

# 5) Engage autonomy
ros2 service call /eolab/landing_controller/start std_srvs/srv/SetBool "{data: true}"
```

**Quick verification (separate terminals)**

```bash
# Our output (should be non-zero; negative throttle in DESCEND/LAND)
ros2 topic echo /protoflyer/fmu/in/manual_control_input | egrep 'roll:|pitch:|yaw:|throttle:'

# PX4 adoption (should follow /in)
ros2 topic echo /protoflyer/fmu/out/manual_control_setpoint | egrep 'roll:|pitch:|yaw:|throttle:'

# Flight mode flags (Manual/POSCTL expected)
timeout 2s ros2 topic echo /protoflyer/fmu/out/vehicle_control_mode \
| egrep 'flag_control_manual_enabled|flag_control_auto_enabled|flag_control_offboard_enabled'

# Altitude trend (z should move steadily during descent)
ros2 topic echo /protoflyer/fmu/out/vehicle_local_position | egrep 'z:|vz:'
```

---

## 3.7 Results (current)

* **Detection & bridging** stable (debug overlay + MarkerArray consistent).
* **Guided-search** shortens time to center; ALIGN/DESCEND transitions behave as intended.
* With **exclusive mode**, PX4 **accepts** our ManualControlSetpoint; QGC/RC no longer cancels descent.
* **Touchdown** is *intermittent* and depends on hover/descend/land throttle calibration and visibility near ground.

---

## 3.8 Limitations

* **Altitude calibration:** fixed `hover_throttle` can be off by vehicle/condition; small offsets slow or prevent descent.
* **FoV at low altitude:** marker may drop out of frame, causing oscillatory SEARCH/ALIGN transitions.
* **Aggressiveness vs noise:** larger `kp_xy`/tilt speeds up centering but can amplify jitter on noisy detections.

---

## 3.9 Improvements (planned)

* **Auto-hover calibration** (pre-flight sweep to estimate true hover).
* **Land detector integration** (`/vehicle_land_detected`) to cut outputs at touchdown.
* **Blended altitude P-term** (use `kp_z` around hover on top of fixed descend).
* **Adaptive smoothing** in the bridge (`smooth_n` 1–3) based on detection jitter.
* **Robust visibility gating** at very low altitude (require small dwell before LAND).

---

## 3.10 Troubleshooting (common pitfalls)

* `/fmu/out/manual_control_setpoint` stuck at zeros → PX4 not adopting inputs.

  * Ensure **Manual/POSCTL** (manual=true, auto/offboard=false).
  * Disable **QGC Joystick**; set **`COM_RC_OVERRIDE=1`**.
  * Run controller with `exclusive_mode=true, rate_hz≥75`.
* Marker briefly visible but no transition → increase `align_enter_visible_dwell` to `0.25–0.35`.
* Descent stalls near ground → lower `land_throttle_min` by 0.02 steps; confirm notch-down logs.

---
