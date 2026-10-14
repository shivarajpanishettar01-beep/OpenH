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

## Freedrive (kinesthetic) collection
- Pedal on a configurable input set to "Freedrive": the input bit in `actual_digital_input_bits` (8 + input)
  and freedrive behaviour on the UR7e (works in Remote Control mode as well: user, 14 Oct 2026).
  `freedrive_check` (usrig_freedrive) tests the labels, locking and drift on the arm; run it first.
- `robot_state_cpp` with the new RTDE outputs (`actual_digital_input_bits`, `robot_mode`): runs on URSim
  5.26.1 (13 Oct 2026, read-only: 475 Hz, IDLE labels, NaN marker at 120 Hz); the FREEDRIVE label itself
  (pedal held) is tested only as pure logic (`test_labels.py`).
- Gravity compensation with the probe + camera + handle payload; hand in the wrist camera view; F/T reading
  with the handle above the sensor.

## Teleop
- SpaceMouse: spacenav driver Joy axis order, scaling (`full_scale`) and the axis signs in `keymap.yaml spacemouse.axes`.
- Contact-loss thresholds on the real phantom; recontact behaviour in real contact.

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
  v3.0 -> `LeRobotDataset` reload, actions / frames / depth / labels against ground truth. Real bags: not yet.
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
