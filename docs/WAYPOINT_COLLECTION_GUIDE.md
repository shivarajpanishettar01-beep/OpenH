# Waypoint and path collection guide

Teaching, verifying and executing preset waypoints and Stage 1 paths. Written from the code; the
tools have **not yet run on the rig** (they need the UR7e, micrus and a registered phantom). Every
terminal: `cd ~/OpenH && source scripts/env.sh`.

All motion goes through `robot_control` services (`/robot/move_to_pose`, `/robot/descend_to_contact`,
`/robot/run_path`, `/robot/retract`, `/robot/set_teleop`): the same authority, limits and FAULT logic as
teleoperation. Waypoint tools never talk to the robot directly.

## 0. Known configuration first
- `rig.yaml`: limits, `robot.tcp_setting` (controller TCP at the probe face), `probe.type`,
  `robot.payload_kg` / `payload_cog_m`, `motion.transit_safe_height_mm`, `limits.phantom_box_m`,
  `limits.local_tilt_cone_deg`, `waypoint_verify_min_ncc`, `registration_max_residual_mm`
  (`ros2 run usrig_robot rig_setup` for the robot and limit values).
- `session.yaml`: `session.phantom_id`. Waypoints, paths and plans go to `~/data/waypoints`, `~/data/paths`,
  `~/data/plans` (`DATA_LAYOUT.md`).
- Robot and micrus running:
  ```bash
  ros2 launch usrig_bringup collect.launch.py session_id:=<SESSION_ID> control:=true teleop:=true
  ```

## 1. Register the phantom (each session, or whenever it moved)
Touch the 4 fiducials of `phantoms/<id>.yaml` with the probe tip (or the calibrated pointer):
```bash
ros2 run usrig_episode register_phantom ~/data/sessions/<SESSION_ID>
```
Refused if the RMS residual exceeds `registration_max_residual_mm`. Writes `T_base_phantom` to
`session.json`. Waypoints are stored in the **phantom frame**, so a new registration does not invalidate
them. Once registered, `robot_control` also enforces `limits.phantom_box_m` (and refuses all motion if
that box is null).

## 2. Teach
Drive the probe to the view with teleop (`TELEOPERATION_GUIDE.md`), then:
```bash
ros2 run usrig_presets waypoint_teach ~/data/sessions/<SESSION_ID>
```
For each waypoint: name, Enter to capture, organ, view name, 3-5 paraphrased instructions. Stored in
`~/data/waypoints/<phantom_id>.yaml`:

| Field | Meaning |
|---|---|
| `name` | stable identifier (re-teaching the same name replaces it) |
| `position_m`, `quaternion_xyzw` | probe-tip pose in the phantom frame |
| `approach_position_m`, `approach_quaternion_xyzw` | `approach_offset_mm` back along the probe axis |
| `force_n` | probe-axis force at capture (descent setpoint at execution) |
| `preset` | micrus settings at capture |
| `reference_snapshot` | ultrasound image used by verification |
| `instructions`, `organ`, `view` | language labels |
| `taught_config` | `tcp_setting`, `probe_type`, `payload_kg`, `payload_cog_m` at capture |
| `verified`, `verified_config`, `verify` | result of the last verification |

A taught waypoint has `verified: false` and **cannot be executed**.

## 3. Verify (makes a waypoint executable)
```bash
ros2 run usrig_presets waypoint_verify ~/data/sessions/<SESSION_ID> [--only NAME ...]
```
For each waypoint: retract → move to the approach pose (phantom frame, via the safe height) → zero F/T →
descend to contact at `force_n` → snapshot → NCC against the reference. `verified: true` only if
NCC ≥ `waypoint_verify_min_ncc`; the tip position error is reported. On success `verified_config` records
the current TCP, probe type and payload.

**Executable** = `verified: true` **and** `verified_config` equal to the current `rig.yaml`.
`episode_manager` refuses any other waypoint (Stage 0 approach, Stage 1 named path waypoints, Stage 2
targets) with the reason, e.g. "verified with a different tcp_setting: verify it again". After a probe
remount, TCP or payload change, run `waypoint_verify` again.

## 4. Paths (Stage 1)
```bash
ros2 run usrig_presets path_editor line  NAME --through WP1 WP2 [WP3 ...] [--instruction "..."]
ros2 run usrig_presets path_editor fan   NAME --at WP --axis tilt|rock --deg 10 --steps 5
ros2 run usrig_presets path_editor sweep NAME --at WP --along x|y --mm 40
ros2 run usrig_presets path_editor list
ros2 run usrig_presets path_editor show  NAME     # nav_msgs/Path on /usrig/path_preview (RViz, frame phantom)
```
Review the ordered poses in RViz before use. Named entries must be executable waypoints; fan / sweep
entries are poses derived from one.

## 5. Plans (Stage 2)
```bash
ros2 run usrig_presets plan_generator generate --waypoints ~/data/waypoints/<phantom>.yaml --rows 400 --seed 1 --operators op1 op2     # -> ~/data/plans/plan_<phantom>_400rows_seed1.csv
ros2 run usrig_presets plan_generator lock --plan ~/data/plans/plan_<phantom>_400rows_seed1.csv --session ~/data/sessions/<SESSION_ID>
```
Only verified waypoints become targets. The lock stores the plan's SHA-256 in `session.json`;
`episode_manager` refuses a plan that is not locked or changed since.

## 6. Execute
Pre-session check (required for real sessions, see `DATA_COLLECTION_GUIDE.md`), then:
```bash
ros2 run usrig_episode episode_manager --ros-args -p session_id:=<SESSION_ID> -p stage:=1 -p robot_control:=true -p operator_id:=op1
```
Before anything moves, `robot_control` validates the **whole** motion:
- `move_to_pose`: every leg (up to the safe height, across, down) sampled every 1/20 of its length:
  inside the base box, the phantom box and the global tilt cone. A move that starts outside (after a FAULT
  recovery) may pass outside on the way but must end inside.
- `run_path`: the constant-speed path sampled every 50 ms; speed in (0, max linear speed].
A refused motion moves nothing and reports the first bad pose ("planned pose 7/40 at [...] mm (base) is
outside the limits ['envelope']: refused before moving"). The limits are enforced again on every control
tick during execution.

Stage 1 per path: approach → recording → free-air hold → zero F/T → descend (force-regulated contact) →
`run_path` (force along the probe axis, constant speed tangentially) → end → retract → QC.
Stage 2 per plan row: approach + start offset → recording → contact → teleop (+ planned perturbation) →
outcome key → retract → QC (pass: next row, fail: redo).

Free-space motion (approach, transit, retract) and force-regulated contact (descend, path, teleop in
contact) are separate motions; a path needs contact first and refuses otherwise.

## 7. Stop, cancel, recover
| Action | Effect |
|---|---|
| Esc (discard) during a Stage 1 path | episode_manager retracts; the path ends `preempted`; episode discarded |
| P | operator pause: the path holds its place on the path (time stops), resumes on P |
| `set_teleop true` during a path | explicit operator takeover; the path ends `preempted` |
| retract service | not subject to the pre-check (refused only in FAULT); cancels the running motion |
| contact lost during a path | path aborted, force mode stopped, robot holds; J = slow recontact |
| FAULT | robot stops; nothing resumes until K; the path is **not** resumed |
| episode_manager restart | resumes at the first incomplete plan row (Stage 2) after you start it; nothing moves by itself |

Every motion ends with a `motion` event (ok, timeout, preempted, FAULT, refused) in the episode bag.
Planned (path / waypoint poses), commanded (`/robot/cmd_executed`) and measured (`/robot/tcp_pose`) motion
are separate topics in the bag.
