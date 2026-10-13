# Setting up the capture PC

This repo is built and unit-tested on the development PC. Data capture runs on a separate
Ubuntu 22.04 + ROS 2 Humble PC with a real USB 3 link to the D415 (the development PC only
reaches USB 2.1 with the available cable/ports: see "P1b status" below).

## 1. System packages (sudo)
```bash
sudo apt update && sudo apt install -y ros-humble-realsense2-camera ros-humble-rosbag2-storage-mcap ros-humble-plotjuggler-ros
```
RealSense tools (`rs-enumerate-devices`, ...) live in `/opt/ros/humble/bin`: run
`source /opt/ros/humble/setup.bash` (or `source scripts/env.sh`) before using them.

## 2. Repo and conda env
```bash
git clone <this repo> ~/OpenH && cd ~/OpenH
```
```bash
scripts/setup_env.sh
```
```bash
scripts/build.sh && scripts/test.sh
```

## 3. Per-machine items
- **micrus**: SDK conda install (Python 3.10) or AppImage, the USB udev rule, and a licence for
  this machine (`micrus machine-id`, install the `.lic`). Set `micrus.command`,
  `micrus.licence_file`, `micrus.usb_vendor_id` in `session.yaml`.
- **bag_to_lerobot**: set `BAG_TO_LEROBOT` if it is not at `~/US SDK/micrus_capture/tools/bag_to_lerobot/`.
- **keyboard_teleop**: add the operator account to the `input` group (`sudo usermod -aG input $USER`, log out and in).
- **Disk**: a data disk with room for the planned session (QC requires > 50 GB free).
- **Clock**: NTP on; every stamp is this PC's system clock.

## 4. D415 on USB 3 (start of P1b)
```bash
rs-enumerate-devices | grep -E "Serial Number|Firmware Version|Usb Type"
```
`Usb Type Descriptor` must be `3.x`. If it shows `2.1`: use the camera's own or a
SuperSpeed-rated USB-C cable, plug in quickly and firmly, use a port on the PC (not a USB 2 hub,
dock or monitor). Update `wrist_camera.serial` in `session.yaml` if it is a different camera.

## 5. Lock exposure and white balance from auto (as requested)
Rig lit as for collection, camera looking at the phantom:
```bash
ros2 launch usrig_bringup wrist_camera.launch.py auto:=true
```
In a second terminal:
```bash
ros2 run usrig_qc camera_autolock --settle-s 5 --sample-s 5 --write
```
Check `ros2 param list /wrist/d415` for the real exposure / white-balance / global-time parameter
names and fix `wrist_camera.rs_params` if they differ; check the exposure units printed by the tool.
Then restart without `auto:=true` and continue P1b (`stream_check --camera --duration 600`, the 60 s
MCAP test, depth at contact, hand-eye).

## P1b status on the development PC (13 Oct 2026)
- D415 serial 211422060364, firmware 5.17.0.10; librealsense2 2.58.4, realsense2_camera 4.58.4,
  rosbag2_storage_mcap 0.15.17.
- USB link 2.1 / 480 Mb/s via a USB 2 hub and then directly on a PC port: P1b not run (no streaming).
- rs_launch.py arguments checked with `--show-args`: colour exposure, white balance and global time
  are not launch arguments in 4.58.4; they now go through `config_file` (`rs_params`).
