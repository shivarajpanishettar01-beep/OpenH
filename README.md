# Robotic-ultrasound data-collection rig

UR7e + Telemed probe (micrus) + wrist RealSense D415, one Ubuntu 22.04 / ROS 2 Humble machine,
one clock. Records one MCAP bag per episode (ultrasound, robot, camera, events) plus the micrus
HDF5, and exports LeRobot v2.1 + `modality.json` (GR00T-H) and LeRobot v3.0 (Open-H v2).

## Layout
| Path | Contents |
|---|---|
| `src/usrig_interfaces` | `RobotState`, `EpisodeEvent` msgs; `MoveToPose`, `DescendToContact`, `InjectPerturbation`, `RunPath` srvs |
| `src/usrig_common` | config + null-limit refusal, geometry (no FK), constant-speed paths, Kabsch, stage gates, legacy packet parser |
| `src/usrig_robot_cpp` | `robot_state_cpp`: lossless 500 Hz RTDE state (ur_client_library), the recorded robot data |
| `src/usrig_robot` | `robot_control` (only node that moves the robot; safety envelope, controller watchdog, FAULT/ack, services), Python `robot_state` fallback, `rig_measure` |
| `src/usrig_teleop` | `keyboard_teleop` (evdev, ramped probe-frame twist at 100 Hz) |
| `src/usrig_episode` | `episode_manager` (stages 0/1/2, crash-safe recorder, resume), `plan_generator`, `register_phantom`, `waypoint_teach/verify`, `path_editor`, `handeye_calib`, `rig_static_tf` |
| `src/usrig_qc` | `qc_monitor` (/qc/status), `episode_qc` (from bag stamps), `presession_check`, `stream_check` |
| `src/usrig_dashboard` | dashboard (demos only), `annotator` (offline expert labelling) |
| `src/usrig_bringup` | `config/session.yaml`, `config/rig.yaml` (all safety limits null), `config/keymap.yaml`; launch files |
| `export/` | `lerobot_rig.yaml` (bag_to_lerobot config), `stage_bags.py`, `depth_export.py`, `openh_export.py` |
| `tools/temporal_calib.py` | ultrasound / D415 latency from a sinusoidal compress test |
| `docs/` | P0 survey, NOT VERIFIED list, open decisions, collection-day commands |

## Setup
```bash
scripts/setup_env.sh
```
```bash
scripts/build.sh
```
```bash
scripts/test.sh
```
System packages (sudo): `ros-humble-rosbag2-storage-mcap ros-humble-realsense2-camera ros-humble-plotjuggler-ros`.
The Open-H export uses a second env (`scripts/setup_env.sh --with-export`, Python 3.12, LeRobot 0.6.0).

## Safety rules built into the code
- `robot_control`, `keyboard_teleop`, `episode_manager` (stages 1-2) refuse to start while any
  limit in `rig.yaml` is null and name each one.
- `robot_control` refuses until `robot.control_backend` is chosen (path A `urscript` / path B `rtde`).
- Test motion in URSim first (`sim:=true`). The real robot moves only after the limits are filled
  and the go is given.
- micrus is never modified; it is driven only through its CLI and ROS 2 interface.

## Status
Code base only. Nothing has run against micrus, URSim, the robot or the camera yet: see
`docs/NOT_VERIFIED.md` and `docs/OPEN_DECISIONS.md`. Next step is the P1 check with
`micrus --simulate`.
