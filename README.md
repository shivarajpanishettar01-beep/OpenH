# Robotic-ultrasound data-collection rig

UR7e + Telemed probe (micrus) + wrist RealSense D415, one Ubuntu 22.04 / ROS 2 Humble machine,
one clock. Records one MCAP bag per episode (ultrasound, robot, camera, events) plus the micrus
HDF5, and exports LeRobot v3.0 for the Open-H Embodiment V2 contribution (via LeRobot v2.1 from
`bag_to_lerobot`).

## Layout
Three collection modes, one package each (operator tools and checks). All of them share one motion
authority (`robot_control`, which freedrive never uses), one recorder and one episode lifecycle
(`episode_manager`).

| Path | Contents |
|---|---|
| `src/usrig_interfaces` | `RobotState`, `EpisodeEvent`, `ControlStatus` msgs; `MoveToPose`, `DescendToContact`, `InjectPerturbation`, `RunPath` srvs |
| `src/usrig_common` | config + null-limit refusal, geometry (no FK), constant-speed paths, Kabsch, stage gates, CLI tool node, motion-check helpers |
| `src/usrig_robot_cpp` | `robot_state_cpp`: lossless 500 Hz RTDE state (ur_client_library), the recorded robot data |
| `src/usrig_robot` | `robot_control` (only node that moves the robot; safety envelope, controller watchdog, FAULT/ack, services), Python `robot_state` fallback, `rig_setup` (guided entry of the limits), `rig_measure`, `pose_monitor` (live pose and its change) |
| `src/usrig_teleop` | **teleoperation mode**: `keyboard_teleop` (Ctrl hold-to-move, commands), `spacemouse_teleop` (proportional 6-DoF, hold-to-move), fail-safe on device loss; `teleop_check` |
| `src/usrig_presets` | **preset mode**: `waypoint_teach/verify`, `path_editor` (Stage 1), `plan_generator` + `failure_catalog.yaml` (Stage 2); `preset_check` (offset / save / go / waypoint moves) |
| `src/usrig_freedrive` | **freedrive mode**: sample labelling rule (`labels.py`, mirrored in `robot_state_cpp`); `freedrive_check` (guided, never commands) |
| `src/usrig_episode` | `episode_manager` (the collection lifecycle for every mode: stages 0/1/2 and freedrive, crash-safe recorder, resume), `register_phantom`, `handeye_calib`, `rig_static_tf` |
| `src/usrig_qc` | `rig_status` (what is set up / connected), `bag_check` (test recordings), `qc_monitor`, `episode_qc` (from bag stamps), `presession_check`, `stream_check`, `status_console` (headless operator view), `auto_annotate`, `collection_progress`, `camera_autolock` |
| `src/usrig_dashboard` | dashboard (demos only), `annotator` (offline expert labelling) |
| `src/usrig_bringup` | `config/session.yaml`, `config/rig.yaml` (all safety limits null), `config/keymap.yaml`; launch files incl. the staged tests `test1_robot` ... `test4_full_system` |
| `export/` | `lerobot_rig.yaml` (bag_to_lerobot config), `stage_bags.py`, `depth_export.py`, `openh_export.py` |
| `tools/temporal_calib.py` | ultrasound / D415 latency from a sinusoidal compress test |
| `docs/` | capture-PC setup, calibration, rig limits, collection-day commands, data integrity, proposal alignment, P0-P2 results, NOT VERIFIED list, open decisions |

## Capture PC setup

The capture PC runs everything: micrus (ultrasound), the UR7e connection, the wrist D415 and the
recorder, all on its own clock. Requirements: Ubuntu 22.04, ROS 2 Humble (`/opt/ros/humble`),
Miniconda, a USB 3 port for the D415, a network link to the UR7e, and a data disk.
Full details and troubleshooting: [`docs/capture_pc_setup.md`](docs/capture_pc_setup.md).

**1. System packages** (sudo):
```bash
sudo apt update && sudo apt install -y ros-humble-ur-client-library ros-humble-realsense2-camera ros-humble-rosbag2-storage-mcap ros-humble-plotjuggler-ros ros-humble-spacenav spacenavd
```

**2. Clone, create the conda env, build, test:**
```bash
git clone https://github.com/shivarajpanishettar01-beep/OpenH.git ~/OpenH && cd ~/OpenH
```
```bash
scripts/setup_env.sh
```
```bash
scripts/build.sh && scripts/test.sh
```
`setup_env.sh` creates the `openh` env (Python 3.10, matching Humble). All Python here runs in it.
After the build, `ls install/usrig_robot_cpp/lib/usrig_robot_cpp/` must list `robot_state_cpp`
(the lossless robot-state node); if it is missing, step 1 was not completed.

**3. In every new terminal:**
```bash
source ~/OpenH/scripts/env.sh
```
It activates `openh`, sources ROS 2 Humble and this workspace. RealSense tools such as
`rs-enumerate-devices` only work after it.

**4. One-time machine settings** (sudo, then log out and back in):
```bash
sudo usermod -aG input $USER
```
```bash
echo "$USER - rtprio 95" | sudo tee /etc/security/limits.d/90-usrig-rtprio.conf
```
The first lets `keyboard_teleop` read the keyboard; the second allows real-time priority for the
robot-state reader, control loop and recorder (check `ulimit -r` prints 95). Without it they run
at normal priority and log a warning.

**5. micrus on this machine:** install the micrus SDK (conda, Python 3.10) or AppImage and its USB
udev rule, run `micrus machine-id`, install the licence file for this machine, then set
`micrus.command`, `micrus.licence_file` and `micrus.usb_vendor_id` in
`src/usrig_bringup/config/session.yaml`. Check the simulator:
```bash
ros2 launch usrig_bringup micrus.launch.py session_id:=setup_check sim:=true
```

**6. Wrist D415:** plug it into a USB 3 port on the PC (not a hub or dock) with a SuperSpeed cable:
```bash
rs-enumerate-devices | grep -E "Serial Number|Usb Type"
```
`Usb Type` must be `3.x`. Put the serial in `session.yaml` (`wrist_camera.serial`), then lock exposure
and white balance from auto: start the camera with `auto:=true`, then let the tool sample and write them:
```bash
ros2 launch usrig_bringup wrist_camera.launch.py auto:=true
```
```bash
ros2 run usrig_qc camera_autolock --write
```

**7. UR7e:** set `robot.ip` in `src/usrig_bringup/config/rig.yaml`, put the robot in **Remote Control**
on the pendant, and record its active TCP offset in `robot.tcp_setting`. Check the read-only connection:
```bash
ros2 run usrig_robot rig_measure pose
```

**8. Safety limits:** every limit in `rig.yaml` is `null` until filled from your measurements and
protocol (see [`docs/rig_limits.md`](docs/rig_limits.md)). Enter them with `ros2 run usrig_robot rig_setup`
(it measures the box, tilt axis, payload and force noise, and checks the values against each other).
`robot_control` refuses to start while any is null and names each one.
Then [`docs/first_motion_checklist.md`](docs/first_motion_checklist.md) on the real arm (no URSim step: decided 13 Oct 2026).

**9. Before each session:**
```bash
ros2 run usrig_qc presession_check --stage 0 --session ~/data/sessions/<SESSION_ID> --control
```
Collection runs headless; the operator view is `ros2 run usrig_qc status_console`. The dashboard
is for demos only. Bring-up in four staged tests (robot, ultrasound + robot, camera + robot, everything), each
with its own launch file: [`docs/TESTING_STEPS.md`](docs/TESTING_STEPS.md). What is set up and connected: `ros2 run usrig_qc rig_status`. All data goes under `~/data` (simulation: `~/data_sim`), one folder per kind:
[`docs/DATA_LAYOUT.md`](docs/DATA_LAYOUT.md). Before collecting, check each mode on the arm and watch the pose change:
[`docs/REAL_ROBOT_CHECKS.md`](docs/REAL_ROBOT_CHECKS.md). Guides: [`docs/DATA_COLLECTION_GUIDE.md`](docs/DATA_COLLECTION_GUIDE.md),
[`docs/TELEOPERATION_GUIDE.md`](docs/TELEOPERATION_GUIDE.md), [`docs/WAYPOINT_COLLECTION_GUIDE.md`](docs/WAYPOINT_COLLECTION_GUIDE.md),
[`docs/FREEDRIVE_COLLECTION_GUIDE.md`](docs/FREEDRIVE_COLLECTION_GUIDE.md) (guide the arm by hand with a foot pedal).
`episode_manager` refuses a real session without a passing pre-session check for that stage.

**Open-H export** (any PC with the recorded data): a second env with Python 3.12 and LeRobot 0.6.0,
`scripts/setup_env.sh --with-export`, then `export/stage_bags.py` and `export/openh_export.py`.
The whole export chain is checked on synthetic episodes with `python export/pipeline_check.py run`.

## Safety rules built into the code
- `robot_control`, `keyboard_teleop` and `episode_manager` (stages 1-2) refuse to start while any
  limit in `rig.yaml` is null, and name each one.
- Control is path B (ur_rtde `forceMode` + `speedL`) with a watchdog on the robot controller: if this
  PC stops sending, the UR7e stops by itself. `robot_control` also refuses to start if the robot is
  not in Remote Control or its active TCP differs from `rig.yaml robot.tcp_setting`.
- Every move and path is checked against the box(es) and tilt cone before it starts; waypoints run only
  if verified with the current TCP / probe / payload.
- The real robot moves only after the limits are filled and the go is given. Simulated and synthetic
  runs are stored separately and never exported with real data.
- micrus is never modified; it is driven only through its CLI and ROS 2 interface.

## Status
Builds from a fresh clone (11 packages); 147 tests (`bash scripts/test.sh`). Verified so far: micrus simulator bringup
(27.35 Hz, 0 frame gaps in 5 min, clean power-down), robot state on URSim, recording and crash recovery
in simulation, data integrity under load on the development PC (C++ robot_state: 0 of ~31,000 samples
lost per minute), and the export chain end to end on synthetic episodes. Not yet run: the D415 on USB 3,
the real scanner, and the real UR7e. Audit and evidence: [`docs/IMPLEMENTATION_AUDIT.md`](docs/IMPLEMENTATION_AUDIT.md),
[`docs/TEST_AND_VERIFICATION.md`](docs/TEST_AND_VERIFICATION.md). Full status: [`docs/progress.md`](docs/progress.md). Before the first real
motion: [`docs/first_motion_checklist.md`](docs/first_motion_checklist.md). See also
[`docs/NOT_VERIFIED.md`](docs/NOT_VERIFIED.md) and [`docs/OPEN_DECISIONS.md`](docs/OPEN_DECISIONS.md).

## License
Code: MIT (see `LICENSE`). This covers the rig code in this repository only. micrus is a separate,
licensed package that is not included here, and datasets produced with the rig are released under
their own licence (OpenMDW-1.1 for the Open-H contribution).
