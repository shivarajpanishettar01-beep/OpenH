# Data collection guide

Session setup, recording, the episode lifecycle, QC and export. The commands are written from the code.
Recording and crash recovery were run in simulation (`P4_results.md`); a real session has **not** run
yet. Every terminal starts with `cd ~/OpenH && source scripts/env.sh`.

Where every file goes: `DATA_LAYOUT.md`. Related: `TELEOPERATION_GUIDE.md` (keys, enable, stop), `WAYPOINT_COLLECTION_GUIDE.md` (waypoints,
paths, plans), `FREEDRIVE_COLLECTION_GUIDE.md` (guiding the arm by hand with the freedrive pedal),
`first_motion_checklist.md` (before the first real session).

## 1. Power-on
1. Power the UR7e and release brakes on the pendant (you, not software). Check the controller TCP and
   payload match `rig.yaml robot.tcp_setting` / `payload_kg` / `payload_cog_m`; put it in Remote Control.
2. Plug the MicrUs scanner and the D415 (USB 3 port, strain relief on the arm cable).
3. Check the D415 enumerates on USB 3:
   ```bash
   rs-enumerate-devices -s
   ```

## 2. Bring up (terminal 1)
```bash
ros2 launch usrig_bringup collect.launch.py session_id:=<SESSION_ID> session_json:=$HOME/data/sessions/<SESSION_ID>/session.json control:=true teleop:=true
```
`control:=true` starts `robot_control`, which refuses to start while any `rig.yaml` limit is null, without
Remote Control, or with a TCP / payload mismatch. Simulation: `sim:=true camera:=false` (micrus
`--simulate`; data goes to `session.sim_root`, never exported).

## 3. Pre-session check (terminal 2): required
```bash
ros2 run usrig_qc presession_check --stage <0|1|2> --session ~/data/sessions/<SESSION_ID> --control --planned-hours <h> --mb-per-s <measured>
```
One row per check: micrus command / licence / USB, every stream's rate and stamp offset, clock (no sim
time), D415 exposure / white balance locked and USB 3, data root, stage gate, `rig.yaml` limits, plan
lock, free disk vs the planned hours. PASS needs every row PASS or N/A (UNSET counts as not passed).

The result is stored in `session.json` (`presession.stage<N>`). **episode_manager refuses a real session**
without a PASS for that stage, run with `--control` if the robot will be commanded, younger than
`session.presession_max_age_h` (12 h). Fix what failed and run it again.

## 4. Calibrations (when needed)
```bash
ros2 run usrig_episode register_phantom ~/data/sessions/<SESSION_ID>
```
```bash
ros2 run usrig_episode rig_static_tf --ros-args -p session_json:=$HOME/data/sessions/<SESSION_ID>/session.json
```
Hand-eye (after touching the camera mount):
```bash
ros2 run usrig_episode handeye_calib ~/data/sessions/<SESSION_ID>
```
Full list and order: `calibration.md`. Waypoints need re-verification after a TCP / probe / payload change.

## 5. Collect (terminal 3)
```bash
ros2 run usrig_episode episode_manager --ros-args -p session_id:=<SESSION_ID> -p robot_control:=true -p operator_id:=op1
```
Operator status (headless, used for collection):
```bash
ros2 run usrig_qc status_console
```
Dashboard (demos; ~1.3 CPU cores; Start/stop and Next ask for confirmation because they can move the robot):
```bash
ros2 run usrig_dashboard dashboard --ros-args -p session_id:=<SESSION_ID>
```
Keys: `TELEOPERATION_GUIDE.md`. Progress against the 10 h commitment:
```bash
ros2 run usrig_qc collection_progress
```

## 6. Episode lifecycle
| Step | Who | What is written |
|---|---|---|
| select | operator (B in Stage 0) or the stage / plan row | `.ep_NNNN.partial.json` (crash record) |
| approach (Stages 1-2, Stage 0 with `approach_waypoint`) | episode_manager → `move_to_pose` | `motion` event |
| start recording | recorder: `ros2 bag record -s mcap` → **readiness gate** (bag subscribed, every ready topic produced a message, within `ready_timeout_s`) → `/micrus/record true` | bag dir `ep_NNNN/`, micrus HDF5 |
| free-air hold, zero F/T, descend | episode_manager → `robot_control` | `phase` events (`free_air`, `zero_ft`, `contact`) |
| `start` event | episode_manager | `start` (the episode's t0) |
| demonstration | teleop / path / perturbation | all recorded topics |
| outcome | operator keys 1/2/3, Esc = discard; Stage 0: B again or `stage0.max_duration_s` | `outcome` event |
| `end`, stop recording | episode_manager: `end` event → SIGINT the bag → `/micrus/record false` | bag closed |
| finalize | episode_manager | `task.txt`, `episode.json`, `annotation.json` (automatic phases / gestures / g_t) |
| QC | `episode_qc` automatically | `qc.json`; Stage 2: pass → next row, fail → redo |
| retract | episode_manager | `motion` event |

An episode is complete only with `episode.json` written after the bag closed. If episode_manager dies
mid-episode: the bag gets SIGINT (PR_SET_PDEATHSIG) and closes cleanly, `record_guard` stops a micrus
recording left on, and on the next start the episode is marked `discard` / `interrupted` (with its HDF5
path). Numbering continues; nothing is overwritten (verified with SIGKILL, `P4_results.md`).

## 7. What is recorded
| Signal | Topic | Required for a valid episode |
|---|---|---|
| Ultrasound B-mode (header stamp = acquisition time on the PC clock) | `/micrus/bmode` | **yes** (readiness gate, QC gaps / fps) |
| Ultrasound settings, frame index | `/micrus/frame_info` | **yes** (QC settings constant, gaps) |
| Lossless ultrasound frames | micrus HDF5 (`/micrus/record`) | **yes** (path in `episode.json`) |
| Robot state 500 Hz: q, qd, TCP base / phantom, wrench, force setpoint, mode, flags, action source, expert flag | `/robot/state` (+ `/joint_states`, `/robot/tcp_pose`, `/robot/wrench`, same stamp) | **yes** (readiness gate, QC rate / sample gaps) |
| Executed command (after the safety filter), probe frame | `/robot/cmd_executed` | yes when robot_control runs |
| Requested teleop command, probe frame | `/teleop/twist` | when teleop runs |
| Raw keys, operator commands | `/operator/command` | audit trail |
| Episode, phase, motion, control-timing events | `/episode/events` | **yes** (QC: start, end, outcome) |
| Wrist RGB and aligned 16-bit depth + camera info | `/wrist/d415/...` | **yes** when the camera is enabled (readiness gate, QC rate / stamp offset / USB / valid depth) |
| Live QC status | `/qc/status` | optional |
| Calibration transforms | `/tf_static`, `session.json` (registration, hand-eye), `rig.yaml` provenance | session level |
| Beamformed data | `/micrus/beamformed` | open decision 4 |

Timestamps: every header stamp is the PC system clock (ns), taken at acquisition (robot: once per RTDE
sample; camera: hardware time mapped to the host clock, NOT VERIFIED until P1b). Ultrasound and camera
offsets against the robot are measured by `tools/temporal_calib.py` (P9c); until then
`calibration.*_time_offset_s` are placeholders. QC checks the gaps it can measure; it does not assume
synchronization from the stamps alone.

## 8. QC
`qc.json` per episode; status per check: pass | fail | n/a | unset (threshold null). An episode passes only
if every check is pass or n/a. Checks: `micrus_fps`, `micrus_frame_gaps`, `settings_constant`,
`robot_rate`, `robot_sample_gaps`, `us_robot_stamp_gap`, `coupling`, `force_band`, `free_air_window`,
`expert_flag`, `events`, `control_timing`, `speed_tracking` (Stage 1), `failure_labels` (Stage 2),
`free_disk`, `wrist_rate`, `wrist_stamp_offset`, `wrist_usb`, `valid_depth`.
Re-run on any episode:
```bash
ros2 run usrig_qc episode_qc ~/data/sessions/<SESSION_ID>/ep_0001
```

## 9. End of session
1. Ctrl+C the episode manager, then the launch (micrus freezes and powers down on SIGINT).
2. Retract and park the robot from the pendant.
3. Review in the dashboard's Review tab, or the `qc.json` files.

## 10. Backup
```bash
rsync -a --info=progress2 ~/data/sessions/<SESSION_ID> ~/data/micrus/<SESSION_ID> <BACKUP_DEST>/
```
or everything at once (sessions, micrus recordings, waypoints, paths, plans, presets, phantoms, calibration):
```bash
rsync -a --info=progress2 ~/data/ <BACKUP_DEST>/data/
```
Verify the copy before deleting anything from the rig disk. Raw bags and HDF5 stay the master copy;
the exported dataset is derived from them.

## 11. Export (Open-H, LeRobot v3.0)
Only episodes with QC pass, not discarded, not simulated (`sim: true` is refused), and not synthetic are
exported. Stage 0 / Stage 1 need `--include-stage0` / `--include-stage1` (open decision 6). Test sessions
are whole held-out sessions.
```bash
python export/stage_bags.py --sessions ~/data/sessions/S1 ~/data/sessions/S2 --test-sessions ~/data/sessions/S3 -o ~/datasets/staging
```
```bash
cd export && python "$BAG_TO_LEROBOT" -c lerobot_rig.yaml -o ~/datasets/us_v21 ~/datasets/staging
```
In the `openh-export` env (Python 3.12, LeRobot 0.6.0):
```bash
~/miniconda3/envs/openh-export/bin/python export/openh_export.py --v21 ~/datasets/us_v21 --order ~/datasets/staging_order.json --out ~/datasets/iitmandi_us
```
Needs `probe.type` and `calibration.prob_cali_mtx` in `rig.yaml`. Action = incremental TCP pose
`[dx, dy, dz, drx, dry, drz]` (base frame, rotation vector; `--action-frame tool` for the TCP frame); the
executed probe-frame twist is kept as `observation.meta.commanded_twist`, with
`observation.meta.command_valid` false where nothing was commanded (freedrive, robot_control off: twist
and setpoint are 0 there); wrist depth is a lossless 16-bit PNG in mm. Freedrive episodes (type
`kinesthetic`) go to the train split; `--exclude-kinesthetic` on `stage_bags.py` leaves them out. Then Open-H's `validate_formatting.py` (not in this repo).

The whole chain is checked on synthetic episodes with known ground truth (no hardware, ~25 s):
```bash
python export/pipeline_check.py run
```
