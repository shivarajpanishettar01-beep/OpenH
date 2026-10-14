# Real-robot checks: see each mode move the arm

One short check per collection mode, run on the UR7e. You watch the pose change live and each check prints
the pose before / after and PASS / FAIL. These tools were tested against the simulated robot
(`test_mode_checks.py`, `test_freedrive_check.py`); **they have not run on the UR7e yet**.

| Mode | Package | Check | What moves the arm |
|---|---|---|---|
| Teleoperation | `usrig_teleop` | `teleop_check` | robot_control, from `/teleop/twist` (the same path as the keyboard) |
| Presets | `usrig_presets` | `preset_check` | robot_control, from `/robot/move_to_pose` |
| Freedrive | `usrig_freedrive` | `freedrive_check` | your hand (UR freedrive, pedal held); software never commands |
| any | `usrig_robot` | `pose_monitor` | nothing (read-only live view) |

## Before any check
- `rig.yaml` limits filled (robot_control refuses otherwise), Remote Control on, payload and TCP set.
- Arm in free air, well above the table, nothing in reach. **Pendant speed slider low** (10-25 %), e-stop in hand.
- Every check that moves the arm asks you to type `yes` first. `--yes` skips that; use it only in simulation.
- The check tools refuse distances above 50 mm / 20 deg. robot_control's own limits (speed, box, cone) apply on top.

Every terminal: `cd ~/OpenH && source scripts/env.sh`.

## Terminal 1: robot and live pose
```bash
ros2 launch usrig_bringup robot.launch.py control:=true
```
```bash
ros2 run usrig_robot pose_monitor
```
`pose_monitor` shows the TCP pose (base and phantom frame), speed, mode / source / expert / safety flags,
probe-axis force, and the **change from a reference** in the base frame and the probe frame
(x along the array, y across, z into tissue). Press Enter to reset the reference before each test.

## 1. Teleoperation (`usrig_teleop teleop_check`)
Stop `keyboard_teleop` first (the check refuses while another node publishes `/teleop/twist`).
```bash
ros2 run usrig_teleop teleop_check --axis slide_along --speed 5 --seconds 2 --back
```
Moves +10 mm along probe x at 5 mm/s, then back. Repeat for each axis:

| `--axis` | Key | Unit of `--speed` | Expect in pose_monitor (probe frame) |
|---|---|---|---|
| `slide_along` (vx) | W / S | mm/s | probe d[x] changes, y z ~0 |
| `slide_across` (vy) | A / D | mm/s | probe d[y] |
| `beam` (vz, free air only) | - | mm/s | probe d[z] (+ = toward the tissue) |
| `tilt` (wx) | R / F | deg/s | rot about probe x; TCP position ~unchanged |
| `rock` (wy) | Q / E | deg/s | rot about probe y; TCP position ~unchanged |
| `rotate` (wz) | Z / C | deg/s | rot about probe z; TCP position ~unchanged |

Negative `--speed` goes the other way. The report checks:
- the direction sign;
- the amount (within 30 % of speed × time);
- the other axes (< 20 %);
- for rotations, that the TCP stays within 2 mm. This proves rotation is about the probe face, i.e. the TCP is set correctly (D4).
- that no limit flag was raised.

A sign FAIL means the keymap / probe frame convention is wrong: stop and report it.

The same thing by hand, with raw ROS commands (watch `pose_monitor`):
```bash
ros2 service call /robot/set_teleop std_srvs/srv/SetBool "{data: true}"
```
```bash
ros2 topic pub -r 100 -t 200 /teleop/twist geometry_msgs/msg/TwistStamped "{header: {frame_id: probe}, twist: {linear: {x: 0.005}}}"
```
(200 messages at 100 Hz = 2 s at 5 mm/s → 10 mm. When publishing stops, the deadman stops the arm.)
```bash
ros2 service call /robot/set_teleop std_srvs/srv/SetBool "{data: false}"
```

## 2. Presets (`usrig_presets preset_check`)
Relative move, then back (probe frame by default; `--frame base` for base axes):
```bash
ros2 run usrig_presets preset_check offset --dx 10 --back
```
```bash
ros2 run usrig_presets preset_check offset --rz 5 --back
```
Save the current pose as a test preset, move away (offset, teleop or the pendant), go back to it:
```bash
ros2 run usrig_presets preset_check save home
```
```bash
ros2 run usrig_presets preset_check go home
```
```bash
ros2 run usrig_presets preset_check list
```
Reports the pose before / target / after, and PASS if move_to_pose succeeded and the TCP ended within 1 mm /
1 deg of the target. A target outside the box or cone is refused before anything moves. Test presets are
stored in `<waypoint_dir>/base_presets.yaml` with the TCP / probe / payload they were saved with; `go` refuses
after a change. They are for testing and transit, not collection waypoints.

To see the raw service call for an offset without moving:
```bash
ros2 run usrig_presets preset_check offset --dx 10 --print-only
```
Collection waypoints (after `register_phantom` and `waypoint_verify`; robot.launch.py started with that
session's `session_json`) go to the approach pose via the safe height:
```bash
ros2 run usrig_presets preset_check waypoint <NAME> --session ~/data/<SESSION_ID>
```

## 3. Freedrive (`usrig_freedrive freedrive_check`)
Stop robot_control (`robot.launch.py` **without** `control:=true`). Remote Control can stay on. Then:
```bash
ros2 run usrig_freedrive freedrive_check
```
Guided, four steps, each started with Enter:
1. Pedal released, hands off: the arm must be labelled IDLE and still (< 0.5 mm).
2. Hold the pedal, move the probe ~5 cm and back by the handle, release the pedal, then press Enter. It must be labelled FREEDRIVE / kinesthetic, with **no motion labelled otherwise**. Motion without the label means the wrong pedal input or `freedrive.input`.
3. Released again: locked and IDLE.
4. Pedal held, hand off the handle (keep it close): drift < 2 mm in 3 s. More drift means the payload / CoG is wrong. Press `s` to skip this step.

The software never commands the arm in this check.

## Recording the results
Put the PASS / FAIL and the printed numbers into `first_motion_checklist.md`'s result table (sections C-D for
teleop and presets) and `FREEDRIVE_COLLECTION_GUIDE.md` section 7.
