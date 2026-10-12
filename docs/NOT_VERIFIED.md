# NOT VERIFIED

Everything below is written but has not been checked on this rig. Nothing in this repo has run
against micrus, URSim, the UR7e or the D415 yet. Unit tests cover pure logic only.

## Robot
- Lab URScript: orientation format (rotvec vs RPY), wrench frame, `fz` / `start`, `force_mode`,
  watchdog, loop rate (P0 item 3). Path A backend's `orientation_format` stays null until known.
- Controller TCP setting and whether it already includes the probe (`probe.tcp_already_includes_probe`).
- `RtdeReader`: one stamp per new `getTimestamp()`; values are read after the timestamp, so on a
  heavily loaded PC a read can straddle two samples. Check repeated controller times and jitter (P2).
- `actual_TCP_force` frame (base frame assumed, per UR docs) and the sign of `forceMode`'s wrench
  along tool +z (into tissue).
- `forceMode` + `speedL` combined each cycle (path B): behaviour in URSim and on the robot.
- Path A: whether the lab URScript accepts absolute targets at 125 Hz; it has no watchdog unless added.
- `zero_ft` over the path A protocol is not defined.
- URSim address (`robot.ursim_ip`).

## micrus
- `micrus serve --ros2` from launch: SIGINT reaching micrus via `ros2 launch` Ctrl+C (terminal
  sends SIGINT to the process group, launch sends another) and the power-down message (P1 check).
- `micrus --version` output format (episode.json `micrus_version`).
- `/micrus/snapshot` `message`: assumed to be a path whose `.png` sibling is the image.
- `frame_info` `image.pixel_spacing_mm` scalar vs pair (dashboard handles both).
- Rate on the real scanner (27.4 fps measured only in simulation).

## Wrist camera
- realsense2_camera version, every `rs_launch.py` argument name in `session.yaml`
  (`rgb_camera.color_profile`, `global_time_enabled`, `rgb_camera.exposure`, ...), and the topic
  names under `/wrist/d415/...`.
- Metadata topic and frame-number field (frame gap detection is n/a until confirmed).
- Header stamps = hardware capture time on the host clock (P1b measurement).
- USB serial string in sysfs vs camera serial (`usrig_qc/usb.py`).
- Minimum depth distance vs camera-to-phantom distance at contact.
- `easy_handeye2` on Humble: not checked; `handeye_calib` uses OpenCV directly.

## Recording
- `rosbag2_storage_mcap` writer options keys (`compression: Zstd`, `compressionLevel`, `chunkSize`)
  in the Humble plugin; plugin not installed yet.
- Bag recorder node name `rosbag2_recorder` (readiness gate looks for it).
- MCAP bag readability after the manager is killed with SIGKILL (bag gets SIGINT via PDEATHSIG).
- rosbags reading Humble MCAP bags with the custom `usrig_interfaces` types.

## Export
- `bag_to_lerobot` on these bags (`--list-topics`, `--dry-run`), bool field `action_is_expert`
  through `fields`, relative `custom_msgs` paths (resolved from the current directory: run from `export/`).
- LeRobot 0.6.0: presence and flags of `lerobot.datasets.v30.convert_dataset_v21_to_v30`, and
  whether it accepts the added string / bool columns.
- Open-H field names and README template vs the current Open-H guide; `validate_formatting.py`
  is not in this repo.
- GR00T `modality.json` schema vs the GR00T release used.

## Phantom / plan
- Fiducial file format `phantoms/<id>.yaml` (`fiducials_mm`), staging method names and
  disturbance magnitudes in `failure_catalog.yaml` (proposed placeholders).
