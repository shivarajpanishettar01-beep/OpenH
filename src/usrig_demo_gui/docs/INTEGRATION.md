# Where real data enters each panel

All panels read one `DashboardState` (`usrig_demo_gui/models.py`). A data source builds it in
`snapshot()`, which the GUI calls on its timer (30 Hz by default). To connect the backend, only
`sources/ros_source.py` (or a new source with the same interface) has to change.

`ros_source.py` is a **draft**: it has been tested with fake message objects (see
`test_ros_snapshot_mapping_without_ros`) but not yet against the rig. The collection guard was run
on a live ROS 2 graph with a stand-in `episode_manager` (16 Oct 2026): paused with no status, for a
real session and again after it, live for a sim session and once it left. It is strictly read-only:
it creates subscriptions only, no publishers, services or parameters on other nodes.

Demo only: the one subscription that always exists is `/episode/status`. While `episode_manager` is
in the ROS graph and its status does not say `session.sim: true`, all other subscriptions are
destroyed and the window shows a paused page (`collection_guard.py`), so the GUI adds no load during
dataset collection. For the first 2 s after start (DDS discovery) it is paused as well.

Start it with:

```bash
ros2 run usrig_demo_gui demo_gui --source ros
```

## Panel by panel

The recording rows below apply only when `show_recording_panel: true` is set in `demo_gui.yaml`
(hidden by default so the force/torque chart gets the full column).

| Panel / field | DashboardState field | Source on the rig | Notes |
|---|---|---|---|
| Header: System | `system` | derived in `ros_source.snapshot()` | Disconnected = no `/robot/state` and no `/micrus/bmode` for 1 s; Degraded = robot FAULT, any safety flag, or any node row warn/down; Running = robot not IDLE or recording; else Idle |
| Header: Contact | `contact` | `/robot/state` wrench, force along tool z | needs `contact_force_threshold_n`; Unknown while null |
| Header: Plane | `plane` | none yet | always Unknown until a plane estimator publishes something |
| Header: Task phase, Outcome | `task.phase`, `task.outcome` | `/episode/events` via `EpisodeTracker` | mapping in `phase_map` / `outcome_map`; phase names checked against episode_manager and robot_control (a robot_control path run is the Sweep) |
| Robot state: 3D view, joints | `robot.joints_rad` | `/robot/state` `q` | the view does FK itself, only for drawing |
| Robot state: trajectory map (flange, probe tip, wrist camera) | `robot.T_flange`, `T_tip`, `T_camera` | `/robot/tcp_pose` (fallback `tcp_base` in `/robot/state`) + `rig.yaml` `robot.tcp_setting`, `probe.tcp_already_includes_probe`, `probe.tip_offset_flange_m` + `/tf_static` `flange -> wrist_camera` (fallback `calibration.wrist_cam_cali_mtx`) | all `null` in the repo today: those paths show "not calibrated" until set (see `frames.py`) |
| Robot state: trajectory phase markers | `task.phase` | `/episode/events` | run starts at the last rest (stationary, no episode) |
| Robot state: TCP position / rotation | `robot.tcp_pos_m`, `robot.tcp_rotvec` | `/robot/state` `tcp_base` | controller value, base frame |
| Robot state: mode, safety flags | `robot.mode`, `robot.safety_flags` | `/robot/state` `mode`, `safety_flags` | constants from `RobotState.msg` |
| B-mode image | `ultrasound.image` | `/micrus/bmode` (mono8) | best effort, keep last 1 (as the dashboard) |
| B-mode pixel spacing, frame index | `ultrasound.pixel_mm`, `frame_index` | `/micrus/frame_info` `image.pixel_spacing_mm`, `frame_index` | scalar or pair both handled (NOT VERIFIED in repo docs) |
| B-mode gain / depth / frequency | `ultrasound.gain`, `depth_mm`, `freq_mhz` | `/micrus/frame_info` `settings.gain`, `settings.depth_mm`, `settings.frequency_hz` (Hz, shown in MHz) | keys checked against `micrus --simulate` (15 Oct 2026); the depth ruler uses the imaged depth (image height x pixel spacing). Shows "—" when missing |
| B-mode frame rate | `ultrasound.fps` | arrival rate measured in the GUI | |
| Wrist RGB | `rgb` | `/wrist/d415/color/image_raw` | rgb8/bgr8 |
| Wrist depth | `depth` | `/wrist/d415/aligned_depth_to_color/image_raw` | uint16 mm -> metres; 0 = invalid (drawn black) |
| Force / torque chart | `ft.t`, `ft.values` | `/robot/state` `wrench`, decimated 500 -> 125 Hz | wrench is in the **base** frame in `RobotState.msg`; rotated into the tool frame with `tcp_base` |
| Episode / scan metrics: contact force | `metrics.items[0]` | as above, absolute value | colour uses `force_band_n` when set |
| Episode / scan metrics: probe-tissue angle | | needs phantom registration (`tcp_phantom` is identity until then) | n/a |
| Episode / scan metrics: in-plane score | | no estimator yet | n/a |
| Episode / scan metrics: speckle SNR | | computed in the GUI on a fixed deep ROI of the B-mode image | relative indicator only |
| Episode / scan metrics: CNR | | needs a vessel ROI / segmentation | n/a |
| Episode / scan metrics: image quality | `metrics.image_quality` | `/qc/status` `coupling_ok` (qc_monitor), else `coupling_floor` | Unknown while `rig.yaml qc.coupling_floor` is null |
| Episode / task | `task` | `/episode/events` | progress = elapsed / `episode_cap_s` |
| Recording state | `recording.status` | `/episode/events` start/end + `rosbag2_recorder` node presence | Finalizing for 2 s after `end` |
| Episode, session, output path | `recording.*` | `episode_id` = `<session>_ep_NNNN` (episode_manager) | path = `/episode/status` `session.session_dir` + `ep_NNNN/` (sim sessions: under `~/data_sim`); fallback `data_root/sessions/<session>/ep_NNNN/` |
| Data warnings | `recording.warnings` | `/qc/status`: frame gaps, valid depth fraction, USB speed / reconnects, force band, free disk | |
| ROS 2 nodes | `nodes` | `get_node_names_and_namespaces()` + `/qc/status.topics[*].hz`; rates of `/qc/status` itself and `/robot/control_status` (counted raw, not decoded) measured in the GUI | node and topic list in `demo_gui.yaml` |

## Things to verify on the capture PC

0. Trajectory map: `rig_static_tf` running (it publishes `flange -> wrist_camera` once hand-eye is
   done; frame names are its `flange_frame` / `camera_frame` parameters, set in `frames:` here),
   and `robot.tcp_setting` / `probe.*` filled in `rig.yaml`. Check with
   `ros2 topic echo /tf_static --once` and `ros2 topic echo /robot/tcp_pose --once`.

1. `ros2 node list` names match `nodes[*].node` in `demo_gui.yaml` (especially the RealSense node
   name and the recorder: the repo's dashboard expects `rosbag2_recorder`).
2. `ros2 topic list` names match `topics:` (D415 topics depend on `camera_namespace` / `camera_name`).
3. ~~Keys inside `/micrus/frame_info` `settings`~~: checked with `micrus --simulate` (15 Oct 2026).
4. Sign of the force along the probe axis when pressing on the phantom (the GUI shows the absolute
   value for the contact force metric, the chart shows signed tool-frame components).
5. ~~What Stage 1 path runs publish~~: robot_control publishes `search` with `path_length_m`, mapped to Sweep.
6. CPU: the GUI subscribes to `/robot/state` at 500 Hz in Python, like the existing dashboard. If the
   capture PC is busy, run the demo GUI on a second PC on the same ROS domain. `gui_nice` in the
   config already lowers the GUI's CPU priority (default 10, as in `usrig_dashboard`).

## Adding another source

Write a class with `start()`, `stop()` and `snapshot() -> DashboardState` (see `sources/base.py`),
then add it to the `--source` choices in `app.py`. A bag-replay source, for example, can reuse
`EpisodeTracker` and the same image/wrench conversions as `ros_source.py`.
