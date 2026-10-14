# Setting up the capture PC

This repo is built and unit-tested on the development PC. Data capture runs on a separate
Ubuntu 22.04 + ROS 2 Humble PC with a real USB 3 link to the D415 (the development PC only
reaches USB 2.1 with the available cable/ports: see "P1b status" below).

## 1. System packages (sudo)
```bash
sudo apt update && sudo apt install -y ros-humble-realsense2-camera ros-humble-rosbag2-storage-mcap ros-humble-plotjuggler-ros ros-humble-ur-client-library ros-humble-spacenav spacenavd
```
RealSense tools (`rs-enumerate-devices`, ...) live in `/opt/ros/humble/bin`: run
`source /opt/ros/humble/setup.bash` (or `source scripts/env.sh`) before using them.

## 2. Repo and conda env
```bash
git clone https://github.com/shivarajpanishettar01-beep/OpenH.git ~/OpenH && cd ~/OpenH
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
- **Real-time priority** for the RTDE reader and the 125 Hz control loop (`rig.yaml robot.rt_priority`,
  SCHED_FIFO). Without it they run at normal priority and log a warning. Allow it once (sudo), then log
  out and in, and check `ulimit -r` prints 95:
  ```bash
  echo "$USER - rtprio 95" | sudo tee /etc/security/limits.d/90-usrig-rtprio.conf
  ```
- **Robot (path B)**: the UR7e must be in **Remote Control** mode for ur_rtde to upload its control
  script; the lab URScript program must not be running. Record the controller's active TCP offset in
  `rig.yaml robot.tcp_setting`: robot_control refuses to start if the controller's TCP differs.
- **Data roots**: real sessions go to `session.root` (`~/data`); anything launched with `sim:=true`
  goes to `session.sim_root` (`~/data_sim`), is tagged `sim: true`, and the export tools refuse it.

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

## 6. After setup
Per-PC settings go in `~/.config/usrig/machine.yaml` (outside the repository; allowed keys:
`usrig_common/config.py MACHINE_KEYS`), e.g. the data disk and this PC's micrus licence:
```yaml
session:
  session: {root: /mnt/data/usrig, sim_root: /mnt/data/usrig_sim}
  micrus: {licence_file: /path/to/this_pc.lic, usb_vendor_id: "xxxx"}
rig:
  calibration: {ultrasound_time_offset_s: <measured on this PC>, camera_time_offset_s: <measured on this PC>}
```
On the capture PC the camera stays enabled (the shared default). In every terminal: `cd ~/OpenH && source scripts/env.sh`. Then:
```bash
ros2 run usrig_qc rig_status
```
It lists, per staged test, what is still to set or to connect. Fill the robot and limit values with
`ros2 run usrig_robot rig_setup`, then follow `docs/TESTING_STEPS.md` (test 1 robot, test 2 ultrasound + robot,
test 3 camera + robot, test 4 everything). Check the export environment once with
`python export/pipeline_check.py run` (needs `scripts/setup_env.sh --with-export`).

## P1b status on the development PC (13 Oct 2026)
- D415 serial 211422060364, firmware 5.17.0.10; librealsense2 2.58.4, realsense2_camera 4.58.4,
  rosbag2_storage_mcap 0.15.17.
- USB link 2.1 / 480 Mb/s via a USB 2 hub and then directly on a PC port: P1b not run (no streaming).
- rs_launch.py arguments checked with `--show-args`: colour exposure, white balance and global time
  are not launch arguments in 4.58.4; they now go through `config_file` (`rs_params`).
