# P0 survey (read-only), 12 October 2026

Nothing was started, connected to, or moved. Sources: the files in this folder before the
rebuild, `ros2 topic list`, `lsusb`, `df`, `dpkg`. The old `ur10/` and `ur10_ws/` scripts were
removed from the repo after the rebuild; a copy is in `~/OpenH_legacy_ur10_2026-10-12.tar.gz`.

## 1. Map of what existed

| Component | Found |
|---|---|
| Launch files | none (only Python scripts run by hand) |
| Robot driver | `ur10_launch.py`: TCP **server** on port 51680; the URScript on the robot connects as client, sends 18-value state packets, receives `(x, y, z, o1, o2, o3, fz, start)` |
| Teleop | none |
| Cameras | none |
| Force/torque | UR built-in F/T only, as values 7-12 of the URScript state packet |
| Recorders | none |
| URScript program | **not in the folder** (needs to be copied from the robot) |
| ur_robot_driver / MoveIt | not installed |

## 2. Live topics

With nothing running: only `/parameter_events` and `/rosout`. No robot, scanner or camera
topics were live, so no rates or QoS could be measured. **NOT VERIFIED**: every topic of the
real system (re-run P0 item 2 with the rig powered).

The legacy node, when it runs, publishes `robot_states` (String, no header), `tcp_pose`,
`tcp_wrench`, `joint_states`, `ee_pose`, `probe_pose`, `rcm_pose` (all stamped with
`get_clock().now()` after parsing, i.e. arrival time, defect 4). Its `/joint_states` would
collide with the new `robot_state` node: do not run both.

## 3. URScript

Not available. What the PC side implies (NOT VERIFIED until the URScript is read):

- **Orientation format is inconsistent on the PC side.** The newest `ur10_launch.py` parses
  values 4-6 as roll/pitch/yaw (`R.from_euler('xyz', ...)`; the rotation-vector parse is
  commented out). The newest `ur10_controller.py` converts the measured quaternion to a
  **rotation vector** and sends that as the command. One of the two is wrong; only the URScript
  (`get_actual_tcp_pose()` vs `rotvec2rpy`) can say which.
- Wrench frame, use of `fz` / `start`, `force_mode`, command watchdog, loop rate: unknown.

## 4. RTDE

Not attempted (robot IP unknown, and connecting needs your go-ahead). `ur_rtde` is installed in
the `openh` conda env. Controller TCP setting: unknown (record it in `rig.yaml robot.tcp_setting`).

## 5. Which legacy file was current

| File | Lines | Modified | Status |
|---|---|---|---|
| `ur10/scripts/new/ur10_launch.py` | 889 | 8 Oct 17:59 | newest launch |
| `ur10/scripts/ur10_launch.py` | 858 | older | previous version (same parser, no RPY comments) |
| `ur10/scripts/ur10_launch_old.py` | 208 | older | old |
| `ur10_ws/src/controller/scripts/ur10_launch.py` | 317 | 7 Oct | older |
| `ur10_ws/src/controller/scripts/ur10_launch_OLD.py` | 66 | 7 Oct | oldest |
| `ur10/scripts/new/ur10_controller.py` | 263 | 8 Oct 18:10 | newest controller |
| `ur10/scripts/ur10_controller.py` | 95 | older | |
| `ur10_ws/src/controller/scripts/ur10_controller.py` | 42 | 7 Oct | truncated version |
| `utils.py` (2 copies) | 161 | identical | UR10e DH (wrong for UR7e), prints in hot path |

**Safety finding:** the newest `ur10_controller.py` publishes the *measured* TCP pose back as a
command with `start=1` and `fz=1.0` on **every** `/tcp_pose` message, despite its docstring
saying it does not. Running it with the URScript closes a loop
UR7e → tcp_pose → controller → robot_controller → UR7e at the state rate (commands pile up,
defect 5). Do not run it.

Reused in the new code: the buffered `[...]` packet parser (`usrig_common/legacy_packet.py`,
without prints) and the protocol (path A backend). Nothing else.

## 6. Disk

`/` (`/dev/sdb2`): 234 GB, **39 GB free (83 % used)**. `~/data` does not exist yet;
`~/MicrUs` is on the same disk. **This is below the 50 GB QC threshold before a single episode
is recorded.** At ~46 MB/s raw for the D415 alone (~165 GB/h before compression), a session needs
a dedicated data disk. Your decision.

## 7. D415

- librealsense2 and `realsense2_camera`: **not installed**.
- No RealSense on USB right now (`lsusb` shows only an Intel Bluetooth device).
- Firmware, USB 3.x enumeration, mount and camera-to-phantom distance: **NOT VERIFIED**. Please
  send photos / measurements of the mount; with the probe in contact, the expected distance must
  be compared with the ~0.3 m minimum depth distance (section 2.4).

## Other packages missing (install with apt, needs sudo)

```bash
sudo apt install ros-humble-rosbag2-storage-mcap ros-humble-realsense2-camera ros-humble-plotjuggler-ros
```

(`ros-humble-realsense2-camera` pulls `librealsense2`; check `rs-enumerate-devices` afterwards.)

## micrus

`~/.local/bin/micrus` → conda env `micrus-us` (Python 3.10.21), micrus 0.2.0. `~/MicrUs` holds
earlier snapshots (`snapshot_*.png/.json/_raw.npz`) and one `cine_*.h5`. Licence file location
and USB vendor id: not checked (set `micrus.licence_file`, `micrus.usb_vendor_id`).
