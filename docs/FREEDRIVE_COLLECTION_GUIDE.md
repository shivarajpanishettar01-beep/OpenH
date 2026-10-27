# Freedrive (kinesthetic) collection guide

You guide the UR7e by hand while the rig records ultrasound, wrist RGB-D, 500 Hz robot state and events.
Decided 13 Oct 2026: **foot pedal** on a UR configurable input set to "Freedrive", **handle above the
F/T sensor**, freedrive episodes in the **same dataset** with their own label. Written from the code; the
pedal path has **not run on the UR7e yet** (checklist at the end).

## How it works
- **Our software never commands the arm in this mode.** The UR controller does freedrive (gravity
  compensation) while the pedal is held and locks the arm when it is released. `robot_control` does not run;
  `episode_manager` refuses freedrive collection if it does. The robot's own safety configuration is the only
  safety layer, so set it (section 1).
- **Labels come from what the controller reports.** `robot_state_cpp` reads `actual_digital_input_bits` and
  `robot_mode` with every RTDE sample. A sample is `mode FREEDRIVE`, `action_source kinesthetic (5)`,
  `action_is_expert true` only while the pedal input is high, the robot is RUNNING, and no fresh
  `robot_control` status exists. Otherwise it is IDLE / none / not expert. The force setpoint is NaN: nobody
  regulates force.
- **No fake command.** Without `robot_control`, `/robot/cmd_executed` carries NaN twists at 125 Hz
  (`frame_id: none`), an explicit "no command" marker.
- **The action is measured motion.** The dataset action is the incremental TCP pose
  `[dx, dy, dz, drx, dry, drz]` from the measured trajectory, the same definition as for teleop episodes.
  `observation.meta.command_valid` is false on freedrive steps, and their `commanded_twist` /
  `force_setpoint_n` are 0 and must be ignored.

## 1. One-time setup (you, on the robot)
1. **Pedal**: a hold-to-run (normally open) foot pedal wired to a configurable digital input (0-7) of the
   control box, per the UR control box I/O wiring.
2. **Pendant** → Installation → I/O Setup → that configurable input → action **Freedrive**.
3. `rig.yaml`: `freedrive.input: <0-7>` (the same input).
4. **Handle** clamped on the wrist link, between the last joint and the tool flange F/T sensor, so the
   hand's force does not pass through the sensor. Do not hold the probe or its holder.
   `session.yaml freedrive.grip: wrist_handle` (use `probe` for any session where you hold the probe: QC then
   marks the force check n/a for those episodes).
5. **Payload** (pendant, Installation → Payload) = probe + holder + camera + handle. Freedrive uses it for
   gravity compensation: a wrong payload makes the arm drift up or down when you let go. `robot_control`'s
   payload check does not run in this mode, so check it with the read-only sweep:
   ```bash
   ros2 run usrig_robot rig_measure force-tilt
   ```
6. **UR safety configuration** (pendant, Installation → Safety; your values): TCP speed and force limits, and
   safety planes around the phantom table. In freedrive these are the only limits. The rig's box / tilt cone
   are not enforced (nothing is commanded); QC only reports speed afterwards.

The freedrive input works in **Remote Control** mode as well as Local (user, 14 Oct 2026), so the robot can
stay in Remote Control between teleop / preset sessions and freedrive sessions. `robot_control` must still
be stopped for freedrive collection (it would otherwise label and hold the arm); the rig only reads RTDE.

Before the first session, run the guided check (section 7):
```bash
ros2 run usrig_freedrive freedrive_check
```

## 2. Each session
```bash
ros2 run usrig_qc presession_check --stage 0 --session ~/data/sessions/<SESSION_ID> --planned-hours <h> --mb-per-s <measured>
```
(no `--control`: nothing is commanded). Then:
```bash
ros2 launch usrig_bringup collect.launch.py session_id:=<SESSION_ID> session_json:=$HOME/data/sessions/<SESSION_ID>/session.json control:=false teleop:=true input:=freedrive episode_manager:=true
```
```bash
ros2 run usrig_qc status_console
```
`input:=freedrive` starts the keyboard for commands only (no motion keys) and `episode_manager` with
`collection_mode: freedrive`. To give every episode an instruction / target (Mode A), leave out
`episode_manager:=true` and start it yourself:
```bash
ros2 run usrig_episode episode_manager --ros-args -p session_id:=<SESSION_ID> -p collection_mode:=freedrive -p adhoc_json:=fields.json
```
with e.g. `fields.json` = `{"instruction": "Find the long-axis view of the vessel", "target": "liver_long"}`.

Zero the F/T sensor in free air at the start of the session from the pendant (a one-line program with
`zero_ftsensor()`). The free-air window at the start of every episode also records the offset.

## 3. One episode
| Step | You | Rig |
|---|---|---|
| 0 | set the task in the console (Recording box: **target** + **instruction**, Set task); it stays for the next episodes until changed | Start is refused without a task (Open-H needs task + target per episode) |
| 1 | probe in free air, pedal released, press **B** | recording starts after the readiness gate |
| 2 | keep still ~1.5 s (`free_air_hold_s`) | `free_air` phase: F/T offset window |
| 3 | | `start` event (episode t0) |
| 4 | hold the pedal, guide the probe onto the phantom and scan; release the pedal to rest (the arm locks) | samples labelled FREEDRIVE / kinesthetic while the pedal is held |
| 5 | **Space** view reached; **O / U / I** failure onset / recovery start / end; **1 / 2 / 3** outcome | events |
| 6 | lift off, press **B** (or **Esc** to discard) | `end`, bag closed, `episode.json` (`type: kinesthetic`, `collection_mode: freedrive`, `grip`, `force_label_valid`), automatic annotation, QC |

`status_console` shows `robot: FREEDRIVE source kinesthetic expert True` while the pedal is held and IDLE
when released. If it never shows FREEDRIVE, the pedal input or `freedrive.input` is wrong.

## 4. QC (additional checks for freedrive episodes)
| Check | Pass when |
|---|---|
| `freedrive_engaged` | at least one sample in the episode is kinesthetic (the pedal was used) |
| `freedrive_speed` | peak TCP speed (20 ms windows) ≤ `limits.max_linear_speed_mm_s` (unset while null) |
| `force_band` | as for robot episodes; **n/a** when `force_label_valid` is false (grip on the probe) |
| `control_timing` | n/a (no robot_control) |
| `unlabelled_motion` | the arm did not move (> 5 mm/s on > 1 % of the window) while labelled IDLE: freedrive by the **pendant button** is not seen by robot_state, so use the pedal |
| `free_air_window` | also measures this episode's F/T offset (mean / std over the free-air hold); exported as `observation.meta.ft_free_air_offset` |
All other checks (ultrasound gaps, robot sample gaps, camera, events) are the same as for robot episodes.

## 5. Export
Raw force stays as recorded; the per-episode free-air offset is exported beside it, and every 500 Hz F/T sample is
kept in `meta/force_torque_full/episode_NNNNNN.parquet` (the step data has F/T at the 30 fps step rate).
`stage_bags.py` includes `kinesthetic` episodes whatever their stage, in the **train** split (after success
episodes, before recovery / failure); `--exclude-kinesthetic` leaves them out. Per step:
| Field | Freedrive value |
|---|---|
| `action` | incremental TCP pose from the measured trajectory (as all episodes) |
| `observation.meta.action_source` | 5 (kinesthetic) while the pedal is held, 4 (none) at rest |
| `observation.meta.action_is_expert` | true while the pedal is held |
| `observation.meta.command_valid` | false |
| `observation.meta.commanded_twist`, `force_setpoint_n` | 0 (ignore: `command_valid` false) |
| `observation.meta.force_torque` | measured contact force (valid with the wrist handle) |
Verified end to end on a synthetic freedrive episode by `python export/pipeline_check.py run`.

## 6. Things to keep in mind for learning
- **Force comes from your hand, not the force controller.** A policy trained on freedrive data and run with
  `robot_control` (force-regulated probe axis) sees a different force behaviour. `action_source` lets
  training weight or separate the two.
- **Your hand may appear in the wrist camera** near the handle: check the D415 view once and note it.
- **Freedrive dynamics** (friction, inertia) make motion less smooth than teleop; QC's speed check flags
  fast moves only.
- The free-air window gives an F/T offset per episode; the export does not subtract it yet (open decision 18).

## 7. First run on the UR7e (checklist)
The first three items are what `freedrive_check` measures (steps 1-4: released = IDLE and still, guided =
kinesthetic with no unlabelled motion, released again = locked, pedal held hands off = drift < 2 mm).
Watch the pose live in another terminal with `ros2 run usrig_robot pose_monitor`.
- [ ] Pedal released: `status_console` shows IDLE; the arm cannot be moved by hand.
- [ ] Pedal held: FREEDRIVE / kinesthetic / expert True; the arm moves by hand; released mid-motion: it locks.
- [ ] Let go of the handle mid-air with the pedal held: the arm stays (payload right) and does not drift.
- [ ] Push slowly toward a safety plane: the robot stops with a protective stop at the plane (UR safety).
- [ ] With the probe in contact and the hand on the wrist handle, the probe-axis force matches the expected
      contact force; touching the probe holder visibly changes it (confirms the grip matters).
- [ ] One test episode: `qc.json` has `freedrive_engaged` pass, `robot_sample_gaps` 0, `micrus_frame_gaps` 0.
- [ ] Export that episode (`DATA_COLLECTION_GUIDE.md` section 11) and check `command_valid` is false and the
      action follows the motion.
