# Filling rig.yaml (safety limits and robot setup)

`robot_control` refuses to start while any of these is null and names each one. `ros2 run usrig_qc rig_status` lists the ones still null (26 of 27 today). Values come from
your measurements and judgement; `rig_measure` (read-only, never moves the robot) helps with the
ones that can be measured. Units are in the key names.

| Key | What it is | How to get it |
|---|---|---|
| `robot.ip` | UR7e controller IP | pendant: Settings, System, Network |
| `robot.tcp_setting` | active TCP offset `[x, y, z, rx, ry, rz]` (m, rotvec) | pendant: Installation, General, TCP |
| `robot.payload_kg`, `robot.payload_cog_m` | payload mass and CoG `[cx, cy, cz]` (flange frame) as set on the controller | pendant: Installation, General, Payload (probe + holder + camera; the pendant's payload estimation can measure it). robot_control refuses to start if the controller differs (R4). Check with `rig_measure force-tilt` |
| `probe.tcp_already_includes_probe` | is that TCP the probe face? | must be `true` (D4): rotations pivot about the TCP; robot_control refuses otherwise |
| `limits.workspace_box_base_m.min/max` | box the probe tip must stay in, base frame | `rig_measure box --write`: touch the extremes of the allowed volume; add your own margin |
| `limits.tilt_reference_axis_base` | axis the tilt cone is measured from | `rig_measure tilt-axis --write` with the probe square to the phantom |
| `limits.tilt_cone_deg` | max angle between probe axis and that reference | your choice for the protocol |
| `limits.max_contact_force_n` | hard ceiling on \|F\|, FAULT above | phantom / probe limits |
| `limits.force_band_n` | `[low, high]` allowed force setpoint along the probe axis | protocol (QC also uses it) |
| `limits.force_setpoint_default_n` | setpoint after contact | inside the band |
| `limits.contact_detect_threshold_n` | descent stops when probe-axis force exceeds this | above the noise from `rig_measure force-noise` |
| `limits.contact_lost_threshold_n`, `contact_lost_time_s` | in contact, force below this for this long = contact lost (hold, event) | below the band low, above the noise; time long enough to ignore brief dips |
| `limits.phantom_box_m.pmin/pmax` | extra box in the phantom frame (z up), enforced once registered | `rig_measure box` around the phantom after registration, converted to the phantom frame |
| `limits.local_tilt_cone_deg` | cone around each waypoint's probe axis | protocol (inside the global cone) |
| `motion.transit_safe_height_mm` | phantom-frame z for transit between waypoints | above the phantom's highest point + margin |
| `limits.max_linear_speed_mm_s`, `max_angular_speed_deg_s` | command speed caps | protocol; keymap speeds must be below them |
| `limits.max_linear_accel_mm_s2`, `max_angular_accel_deg_s2` | command acceleration caps | protocol |
| `limits.stop_decel_mm_s2` | deceleration of speedStop on FAULT / stop | higher than the accel caps |
| `limits.deadman_timeout_s` | no teleop input for this long: zero velocity; also the controller watchdog period | e.g. a few control periods to a fraction of a second |
| `motion.transit_speed_mm_s`, `descent_speed_mm_s` | approach / descent speeds | below the speed cap; descent slow enough to stop on contact |
| `motion.descent_max_travel_mm` | give up descending after this travel | from the approach offset (50 mm) plus tolerance |
| `motion.retract_distance_mm` | retract along the probe axis | clears the phantom |
| `rtde.force_mode_limits` | 6 values for forceMode type 2: max speed along the compliant probe axis (m/s), allowed deviation on the other axes (m, rad) | protocol; check behaviour in the URSim dry run |
| `rtde.force_mode_damping` | optional (0-1); null = controller default | |

Before starting control: put the UR7e in **Remote Control** (pendant, top right), power on and release
brakes. robot_control checks Remote Control, the TCP and the payload itself and refuses on any mismatch.
