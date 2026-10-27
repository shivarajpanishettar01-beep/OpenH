# Data collection guide

Session setup, recording, the episode lifecycle, QC and export. The commands are written from the code.
Recording and crash recovery were run in simulation (`P4_results.md`); a real session has **not** run
yet. Every terminal starts with `cd ~/OpenH && source scripts/env.sh`.

Where every file goes: `DATA_LAYOUT.md`. Related: `TELEOPERATION_GUIDE.md` (keys, enable, stop), `WAYPOINT_COLLECTION_GUIDE.md` (waypoints,
paths, plans), `FREEDRIVE_COLLECTION_GUIDE.md` (guiding the arm by hand with the freedrive pedal),
`first_motion_checklist.md` (before the first real session), `OPERATOR_CONSOLE_GUIDE.md` (the collection window:
every command below has a button there, and Pause holds the robot while recording continues).

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
ros2 run usrig_episode handeye_calib --session ~/data/sessions/<SESSION_ID>
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
Schedule to a deadline (remaining hours per phantom -> collection days, one phantom per day; yield and the
recorded share of rig time are measured from your sessions after 10 episodes, assumptions until then):
```bash
ros2 run usrig_qc collection_plan --by 2026-12-01 --hours-per-day 2.5 --goal vascular_4vein=1.5 central_line_gen2=1.0
```
(`--goal` for an intermediate milestone; without it the session.yaml targets.) Both count exactly the episodes
`stage_bags` exports (one rule, `usrig_common/export_rules.py`: Stage 0 / 1 only with `--include-stage0` /
`--include-stage1`, freedrive at any stage unless `--exclude-kinesthetic`), each as `duration_s - paused_s`;
the dataset card counts the same time (in-episode, unpaused steps). Pass the same options as to `stage_bags`.

## 6. What one episode is
One episode = one attempt at one target view: from the probe in free air (the free-air window) to the view
acquired and held (success), or to the end of the recovery / a declared failure. Aim for the proposal's 20-30 s
(`targets.episode_s`; QC reports `episode_length`, the console's elapsed time turns amber past 30 s); the outcome,
not the clock, ends an attempt. Do not record long takes to split later: every episode needs its own task text,
outcome, labels and QC, and there is no splitting tool. Pause for interruptions; Discard what you would cut out.
Freedrive and free teleop (Stage 0): set the **task** (target + instruction) in the console before Start; Stage 0
with a task is recorded as `teleop` (exported), without one as a `shakedown` test (not exported).

## 6b. Episode lifecycle
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
`wrist_rate`, `wrist_stamp_offset`, `wrist_usb`, `valid_depth`, `stream_coverage` (every stream covers the
window), `bag_closed`, `unlabelled_motion`; information only: `episode_length`, `free_disk` (the disk floor is
checked before Start). While thresholds are null (unset = fail) no episode counts as kept; the data is recorded
anyway. Once they are set, re-check whole sessions and write the result into episode.json:
```bash
ros2 run usrig_qc episode_qc ~/data/sessions/<SESSION_ID> --update
```

## 8b. Labelling and label agreement (expert annotation QA)
`auto_annotate` writes the automated layers to `<ep>/annotation.json` after every episode. Annotators
accept or correct them in the `annotator`; each annotator's work goes to `<ep>/annotation.<annotator>.json`
only (own segments, narration, episode labels, QA decisions on other entries), so several annotators can
work at the same time and `annotation.json` is never rewritten by hand:
```bash
ros2 run usrig_dashboard annotator ~/data/sessions/S1 --annotator dr_a --role expert
```
Accepting an automated segment (QA `accepted`) counts as that annotator's rating; rejecting it removes it
from the dataset. Where labels overlap, the export uses expert > model_assisted > trainee > automated and
writes the winner's provenance per step (`observation.meta.{status,phase,gesture}_source`); an expert
rejection beats a trainee acceptance.

Double-rate 10-20 % of the episodes for agreement: the second annotator works blind (other raters'
labels hidden; automated ones visible):
```bash
ros2 run usrig_qc label_agreement ~/data/sessions/S1 --pick 0.15          # which episodes need a second rater
ros2 run usrig_dashboard annotator ~/data/sessions/S1 --annotator dr_b --role expert --blind
ros2 run usrig_qc label_agreement ~/data/sessions/S1 ~/data/sessions/S2
```
Cohen's kappa (two raters) or Fleiss' kappa (more) for phases, g_t intervals and gestures (sampled at 10 Hz),
and for failure class / outcome / motion quality (quadratic-weighted) from each annotator's own rating.
Unreviewed automated labels and QA-rejected labels are ignored. The acceptance threshold is yours.

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
exported. Stage 0 / Stage 1 need `--include-stage0` / `--include-stage1` (open decision 6); freedrive
episodes are kept whatever their stage. Test sessions are whole held-out sessions. A bag without messages
on a topic the converter needs (e.g. no camera) is skipped and listed (`SKIPPED`, and `skipped` in the
order file) instead of stopping the conversion; `--depth none` if the export runs without depth.
```bash
python export/stage_bags.py --sessions ~/data/sessions/S1 ~/data/sessions/S2 --test-sessions ~/data/sessions/S3 -o ~/datasets/staging
```
```bash
cd export && python run_bag_to_lerobot.py -c lerobot_rig.yaml -o ~/datasets/us_v21 ~/datasets/staging
```
`run_bag_to_lerobot.py` runs `$BAG_TO_LEROBOT` unchanged, but lets rosbags read the RealSense `Metadata`
definition (it has a CRLF line ending; calling the converter directly stops on every bag with
`/wrist/d415/color/metadata`).
In the `openh-export` env (Python 3.12, LeRobot 0.6.0):
```bash
~/miniconda3/envs/openh-export/bin/python export/openh_export.py --v21 ~/datasets/us_v21 --order ~/datasets/staging_order.json --out ~/datasets/iitmandi_us
```
Before writing anything the export checks, and refuses with the full list of problems: the v2.1 dataset holds
exactly the staged episodes in staging order and was converted with `export/lerobot_rig.yaml` (hash in
`micrus_sources.jsonl`); every episode's measured time offsets (`episode.json time_offsets_s`, from its capture
PC) equal the `time_offset_s` the converter aligned the videos with (convert episodes from capture PCs with
different offsets separately); every failure / recovery episode has a failure class.
Calibrations are exported per session: each session's snapshot in `session.json` (`calibration.prob_cali_mtx`,
`calibration.wrist_cam_cali_mtx`, or the older top-level `wrist_cam_cali_mtx`); the exporting PC's `rig.yaml`
only where a session has none, recorded as such in `meta/calibration/*.json` (`per_session` source),
`provenance.jsonl` and the dataset card. `wrist_cali_mtx` is T_flange_camera (D415 colour optical frame in the
flange frame), not the TCP. Needs `probe.type` in `rig.yaml` and a `prob_cali_mtx` for every session. Action = incremental TCP pose
`[dx, dy, dz, drx, dry, drz]` (base frame, rotation vector; `--action-frame tool` for the TCP frame); the
executed probe-frame twist is kept as `observation.meta.commanded_twist`, with
`observation.meta.command_valid` false where nothing was commanded (freedrive, robot_control off: twist
and setpoint are 0 there); wrist depth is a lossless 16-bit PNG in mm. The converter keeps the whole bag, so
steps are flagged: `observation.meta.in_episode` (false on the pre-roll: readiness, free-air hold, descent, and
after the end event), `observation.meta.paused` (operator pause) and `observation.meta.action_valid` (false on
those and on the last step, whose action is a zero placeholder): train on `action_valid` steps. Freedrive episodes (type
`kinesthetic`) go to the train split; `--exclude-kinesthetic` on `stage_bags.py` leaves them out.

Dataset card and checksums (every number from the data: hours per phantom against the targets, episode types,
modes, operators, label provenance, calibration present / missing, capture PCs, code commits, known
limitations):
```bash
~/miniconda3/envs/openh-export/bin/python export/release_card.py --dataset ~/datasets/iitmandi_us
```
writes `README.md` and `MANIFEST.sha256` into the dataset; `--check` verifies the manifest after copying or
uploading. Then Open-H's `validate_formatting.py` (not in this repo).

The whole chain is checked on synthetic episodes with known ground truth (no hardware, ~25 s):
```bash
python export/pipeline_check.py run
```
