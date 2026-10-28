# NOT VERIFIED

Everything below is written but has not been checked on this rig. Run so far: micrus simulator (P1),
robot state on URSim (P2, read-only), recorder in simulation (P4), export end to end on synthetic
episodes (`pipeline_check.py`). Nothing has run on the UR7e, the real scanner or the D415. Test levels:
`TEST_AND_VERIFICATION.md`.

## Robot
- Lab URScript: orientation format (rotvec vs RPY), wrench frame, `fz` / `start`, `force_mode`,
  watchdog, loop rate (P0 item 3). Path A backend's `orientation_format` stays null until known.
- Controller TCP setting and whether it already includes the probe (`probe.tcp_already_includes_probe`).
- `RtdeReader` on URSim (P2, docs/P2_results.md): no repeats, identical stamps on all derived
  messages, but URSim runs at 0.968x real time and ~2 samples/s are missed by polling.
  Real-controller rate / jitter still NOT VERIFIED.
- `RtdeReader`: one stamp per new `getTimestamp()`; values are read after the timestamp, so on a
  heavily loaded PC a read can straddle two samples. Check repeated controller times and jitter (P2).
- `actual_TCP_force` frame (base frame assumed, per UR docs) and the sign of `forceMode`'s wrench
  along tool +z (into tissue).
- `forceMode` + `speedL` combined each cycle (path B): behaviour on the robot (URSim run skipped).
- Payload check (R4): reading `payload` / `payload_cog` through a restricted RTDEReceiveInterface works on
  URSim 5.26.1 (13 Oct 2026); on the UR7e's PolyScope version: NOT VERIFIED. `payload_cog` frame assumed to
  be the flange frame as on the pendant.
- Control-tick jitter on the capture PC (R8) and whether `speedl_time_ticks` > 1 is needed.
- ur_rtde `setWatchdog(1/deadman)` + `kickWatchdog()` every tick: stop behaviour when the PC stalls
  (real arm, checklist D). `getTCPOffset` vs `rig.yaml robot.tcp_setting` check.
- SCHED_FIFO for the RTDE reader / control loop: refused on the dev PC (`ulimit -r` = 0); fallback
  verified (warning, normal scheduling). With the rtprio limit set: NOT VERIFIED.
- Path A: whether the lab URScript accepts absolute targets at 125 Hz; it has no watchdog unless added.
- `zero_ft` over the path A protocol is not defined.
- URSim address (`robot.ursim_ip`).

## micrus
- ~~SIGINT via `ros2 launch` Ctrl+C reaching micrus~~: verified with the simulator (docs/P1_results.md);
  re-check on the real scanner.
- `micrus --version` output format (episode.json `micrus_version`).
- `/micrus/snapshot` `message`: assumed to be a path whose `.png` sibling is the image.
- `frame_info` `image.pixel_spacing_mm` scalar vs pair (dashboard handles both).
- Rate on the real scanner (27.35 Hz measured with the simulator through our launch, P1).

## Wrist camera
- realsense2_camera 4.58.4 launch-argument names: checked with `--show-args` (dev PC). Still
  NOT VERIFIED: the node parameter names in `rs_params` (`rgb_camera.exposure`,
  `rgb_camera.white_balance`, `*.global_time_enabled`), the topic names under `/wrist/d415/...`,
  the metadata keys `camera_autolock` reads, and the exposure units. Needs the camera on USB 3.
- Metadata topic and frame-number field (frame gap detection is n/a until confirmed).
- Header stamps = hardware capture time on the host clock (P1b measurement).
- USB serial string in sysfs vs camera serial (`usrig_qc/usb.py`).
- Minimum depth distance vs camera-to-phantom distance at contact.
- `easy_handeye2` on Humble: not checked; `handeye_calib` uses OpenCV directly.

## TPV camera (third-person view, added 25 Oct 2026)
- No second camera yet: `tpv_camera.enabled: false`, serial / exposure / white balance null. Launch,
  recording, QC (`tpv_rate`, `stream_coverage`, presession rows, `bag_check --expect tpv`) and export
  (`observation.images.tpv_camera`, `observation.meta.tpv_cali_mtx`, `meta/calibration/tpv_cali_mtx.json`,
  TPV intrinsics) were tested on synthetic bags and the export pipeline check only.
- rs_launch.py argument `enable_depth` (colour-only TPV) in realsense2_camera 4.58.4.
- Two RealSense cameras on the capture PC at once (USB bandwidth, wrist USB 3 + TPV colour 640x480@30).
- `T_base_cam` of the calibration rig is in the same UR base frame as OpenH's `/robot/tcp_pose`
  (both the controller's base frame; check once with a probe-tip touch seen by the TPV camera).
- The dataset key `observation.images.tpv_camera`: confirm against the earlier Open-H datasets' scene camera name.
- Since lerobot_rig.yaml lists the TPV image, `stage_bags` skips bags without TPV frames (reported).

## Freedrive (kinesthetic) collection
- Pedal on a configurable input set to "Freedrive": the input bit in `actual_digital_input_bits` (8 + input)
  and freedrive behaviour on the UR7e (works in Remote Control mode as well: user, 14 Oct 2026).
  `freedrive_check` (usrig_freedrive) tests the labels, locking and drift on the arm; run it first.
- Robot data content on the real UR7e (16 Oct): `bag_check` now checks that pose, joints and force are READ
  (F/T live, not all zero / constant; TCP not at the origin; derived topics equal `/robot/state`), and
  `robot_state_cpp` drops a sample with a missing RTDE field instead of publishing zeros. Only checked on
  synthetic bags and the 14 Oct URSim bag (force exactly 0 there: URSim has no F/T sensor). First real
  check: test 1.5 with the move / tilt / press protocol.
- `robot_state_cpp` with the new RTDE outputs (`actual_digital_input_bits`, `robot_mode`): runs on URSim
  5.26.1 (13 Oct 2026, read-only: 475 Hz, IDLE labels, NaN marker at 120 Hz); the FREEDRIVE label itself
  (pedal held) is tested only as pure logic (`test_labels.py`).
- Gravity compensation with the probe + camera + handle payload; hand in the wrist camera view; F/T reading
  with the handle above the sensor.

## Collection gaps closed 18 Oct 2026
- Console task (target + instruction) -> episode_manager -> episode.json / task.txt: unit-tested, not run live.
- `meta/force_torque_full/` (500 Hz F/T sidecar) and `observation.meta.ft_free_air_offset`: pipeline check only;
  whether Open-H's validator accepts the extra meta folder is not checked (run it on the first sample).
- `unlabelled_motion` threshold (5 mm/s, 1 % of the window) chosen as a QC heuristic, not measured on the arm.
- UR freedrive with the hand on the handle above the F/T sensor (does the arm move? does the hand force stay out
  of actual_TCP_force?) and the frame of actual_TCP_force: check in the first freedrive session.

## Audit fixes of 16-17 Oct 2026 (docs/COLLECTION_READINESS_AUDIT.md)
- Protective-stop / not-RUNNING detection (`getSafetyMode`, `robot_mode`), FAULT on a refused speedL / forceMode,
  and `reuploadScript` on acknowledge: checked against the kinematic fake only. Check on the UR7e: trigger a
  protective stop (pendant) during teleop -> FAULT, labels NONE; unlock on the pendant -> ack -> teleop again.
  RTDE `safety_mode` is deprecated in favour of `safety_status_bits` (PolyScope 5.4+): confirm getSafetyMode
  still reports a protective stop on this controller.
- Live T_base_phantom update of robot_state: verified on URSim (read-only, 17 Oct); register_phantom's push
  and the Start check are not yet run against a real registration.
- Stage 2 loop changes (cross-session progress, redo pass, next row shown, N before every row): unit-tested,
  not run end to end (needs robot_control and taught waypoints).
- One ultrasound frame missing from the bag (present in the micrus HDF5) in 2 of 4 sim episodes on this PC
  (recorder at normal priority). Re-check on the capture PC with rtprio set.

## Operator console (14 Oct 2026)
- Checked under ROS 2 Humble (15 Oct): tests with a scripted ROS side, an offscreen render (Stage 2, episode 7
  waiting for an outcome), and the real `ConsoleRos` fed by a fake publisher (status / control / state received,
  buttons gated), and against the real episode_manager in simulation (15 Oct: micrus --simulate, qc_monitor,
  rig_static_tf, a stand-in /robot/state; Stage 0 without robot_control: Start, View reached, Pause, Resume,
  Success, Stop -> bag events, episode.json pauses, QC, back to idle). Not yet with robot_control or the real
  arm (no robot on the dev PC), nor Stages 1-2 / freedrive from the console. In that run one of 597 ultrasound
  frames was missing from the bag but present in the micrus HDF5 (recorder at normal priority: no rtprio limit on
  the dev PC); QC flagged it. Check on the capture PC with docs/capture_pc_setup.md applied. CPU use on the capture PC with the scanner,
  D415 and UR7e running (data_integrity.md runner, with and without the console); every robot action from the
  console on the real arm; the terminal emulator used by "Launch collection".
- `/episode/status` with TRANSIENT_LOCAL durability under Fast DDS (`config/fastdds_rig.xml`).

## Teleop
- SpaceMouse: spacenav driver Joy axis order, scaling (`full_scale`) and the axis signs in `keymap.yaml spacemouse.axes`.
- Contact-loss thresholds on the real phantom; recontact behaviour in real contact.

## Policy (GR00T-H) runs (added 27 Oct 2026)
- robot_control `PolicyRun` (/robot/set_policy, /policy/target_pose, /policy/info): tested against the kinematic
  fake only (`test_control_sim.py -k policy`): following, force-regulated probe axis, speed caps, input timeout,
  refused targets (limits, jump, frame), stop / operator takeover / FAULT records, labels POLICY / never expert.
  No policy node exists yet; nothing ran in URSim or on the UR7e.
- `rig.yaml policy.*` values are null (set_policy refuses until they are chosen).
- episode.json `policy` / `policy_runs` and the export's provenance `policy`, `policy_steps` and card line:
  unit test + synthetic pipeline check (ep_0002: policy, then failure, then teleop recovery).
- The target is the probe-tip pose in the base frame (the dataset's action) on the assumption that the controller
  TCP is the probe face (`design_requirements`); a policy trained on an export with another tip offset would be off
  by that offset.

## Recording
- Loss budget on the capture PC (docs/data_integrity.md): dev-PC numbers are with the micrus
  simulator using ~4 cores; real scanner, D415 and UR7e not measured together yet.
- `rosbag2_storage_mcap` writer options keys (`compression: Zstd`, `compressionLevel`, `chunkSize`)
  in the Humble plugin; plugin not installed yet.
- Bag recorder node name `rosbag2_recorder` (readiness gate looks for it).
- MCAP bag readability after the manager is killed with SIGKILL (bag gets SIGINT via PDEATHSIG).
- rosbags reading Humble MCAP bags with the custom `usrig_interfaces` types.

## Export
- Verified on synthetic episodes (`export/pipeline_check.py`, 13 Oct 2026): bag -> v2.1 -> Open-H fields ->
  v3.0 -> `LeRobotDataset` reload, actions / frames / depth / labels against ground truth. Real bags: 5 freedrive
  episodes from the lab recorder (`~/calib/us_wrist_bag`, 26 Oct; progress.md "Real-hardware pilot") exported and
  reloaded; not yet bags written by OpenH's episode_manager, and no ground truth for the real ones.
- Wrist hand-eye on the pilot episodes: assumes the camera bracket did not move between the 19 Oct calibration and
  the 26 Oct recording (one D415, moved from the TPV mount to the wrist on 19 Oct).
- Action angles centred on roll = 180 deg (`openh_fields.EULER_CENTER`): matches a probe pointing down; the published
  reference keeps roll in [-180, 180] with jumps. Confirm Open-H / GR00T accept angles outside [-pi, pi].
- `rosbags` (bag_to_lerobot, QC) rejects `sensor_msgs/JointState` with empty `velocity` / `effort` (CDR size
  mismatch). robot_state_cpp fills them; check the first real bag with `bag_to_lerobot.py --list-topics` and
  `--dry-run`.
- `bag_to_lerobot` on these bags (`--list-topics`, `--dry-run`), bool field `action_is_expert`
  through `fields`, relative `custom_msgs` paths (resolved from the current directory: run from `export/`).
- LeRobot 0.6.0: converter found at `lerobot.scripts.convert_dataset_v21_to_v30` (flags --repo-id, --root,
  --push-to-hub=false checked); still NOT VERIFIED on our data:
  whether it accepts the added string / bool columns.
- Open-H field names and README template vs the current Open-H guide; `validate_formatting.py`
  is not in this repo.
- GR00T `modality.json` schema vs the GR00T release used.

- Open-H export (proposal fields): 16-bit PNG `wrist_depth` image features through LeRobot 0.6.0's
  v2.1 -> v3.0 converter and loader; full pipeline bag -> bag_to_lerobot -> openh_export not yet run.
- `temporal_calib.py --aruco` on real colour frames (marker dictionary / size are inputs).

## Phantom / plan
- Fiducial file format `phantoms/<id>.yaml` (`fiducials_mm`), staging method names and
  disturbance magnitudes in `failure_catalog.yaml` (proposed placeholders).

## Staged tests (TESTING_STEPS.md)
- test1-test4 launch files ran in simulation only (test 3: refusal path only, no D415 here).
- `temporal_calib.py` on a real compress test (hand-driven in freedrive or robot-driven): not run yet.
- Ultrasound frames missing from the bag under heavy load (12 of 555 in a simulated test 4 episode, micrus
  simulator writing HDF5 + MP4 at ~4.5 cores; the HDF5 keeps every frame): measure on the capture PC.
