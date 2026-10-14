# Staged rig tests: step by step

Four tests, each building on the one before. Each has its own launch file and ends with a check
you can read: a PASS / FAIL table from `bag_check`, or an episode with `qc.json`.

| Test | Launch file | Hardware | Calibration in this test |
|---|---|---|---|
| 1 Robot: topics, poses, data | `test1_robot.launch.py` | UR7e (LAN) | TCP setting, payload (pendant) |
| 2 Ultrasound + robot | `test2_micrus_robot.launch.py` | + MicrUs scanner, probe | pointer pivot, phantom registration, ultrasound temporal offset |
| 3 Camera + robot | `test3_camera_robot.launch.py` | + D415 (USB 3) | exposure lock, hand-eye, camera temporal offset |
| 4 Everything | `test4_full_system.launch.py` | all | pre-session check, Stage 0 episodes |

**Two PCs** (plan of 14 Oct 2026): tests 1 and 2 on this development PC, test 3 (camera) and the final
collection on the camera / capture PC. Both pull the same repository. Settings that differ per PC live outside
it, in `~/.config/usrig/machine.yaml`, and only these keys are allowed there: data roots, micrus command /
licence / USB id, camera on/off, recording and robot real-time priority, ultrasound and camera time offsets.
Safety limits are refused there and always come from the shared `rig.yaml`. This PC's file switches the camera
off, so `qc_monitor`, the recorder and the pre-session check do not expect it. `rig_status` prints the active
machine settings.
- The micrus licence is tied to the machine (`micrus machine-id`): the capture PC needs its own.
- The ultrasound time offset (test 2) depends on the PC's USB and CPU: measure it again on the capture PC and
  keep each PC's value in its own `machine.yaml` (`rig: {calibration: {ultrasound_time_offset_s: ...}}`).
- Robot values (IP, TCP, payload, limits), the pointer calibration and the phantom files are the same for both
  PCs. They go in the shared files: commit and push them here, then pull on the capture PC.

**Every terminal**:
```bash
cd ~/OpenH && source scripts/env.sh
```
Where files go: `docs/DATA_LAYOUT.md`. Test recordings go to `~/data/checks/<test>_<date_time>/`, never into
sessions, and are never exported. Simulated runs (`sim:=true`) go to `~/data_sim`.

What has run where (14 Oct 2026, development PC):
- Test 1 ran in simulation (URSim): its recording passed `bag_check`, with 0 robot samples lost.
- Test 2 ran in simulation (URSim + the micrus simulator): recordings passed with 0 robot samples lost and 0
  ultrasound frame gaps. One run right after URSim had booted lost robot samples in the bag; see the note at
  the end of test 2.
- Test 4 ran in simulation with `camera:=false`: a Stage 0 episode was recorded with its files, its HDF5 and QC.
- Test 3 needs the D415 and has only been checked to refuse while the camera settings are unset.

None of them has run on the UR7e, the real scanner or the camera yet.

---

## Step 0 (before every test): what is set up, connected and running
```bash
ros2 run usrig_qc rig_status
```
Read-only, works with nothing connected. Per test it lists what is ready (`ok`), what to fill in the config
(`SET`, with the key and where the value comes from), what is not connected (`CONNECT`), and rig nodes that are
already running from an earlier launch. The last lines say which tests can start.

## Troubleshooting
| You see | Cause | Fix |
|---|---|---|
| `Package 'usrig_bringup' not found` | the terminal has not loaded the workspace | `cd ~/OpenH && source scripts/env.sh` (every new terminal) |
| `The message type 'usrig_interfaces/msg/RobotState' is invalid` (`ros2 topic hz/echo`) | same: only `/opt/ros/humble` is sourced, so the rig's own message types are unknown | `source scripts/env.sh` in that terminal too |
| `robot: NOT STARTED - no robot address` | `rig.yaml robot.ip` is null | connect the robot and set `robot.ip`, or `robot_ip:=<ip>` on the launch line |
| `robot <ip>:30004 (RTDE) not reachable within 3 s` | wrong IP, cable, PC not on the robot subnet, controller off | `ping <ip>`; PC address on the same subnet; controller booted |
| nodes / topics you did not start in `ros2 node list` / `ros2 topic list` | a launch from another terminal is still running (a topic also appears when only a subscriber exists, e.g. `qc_monitor` lists `/robot/state` and `/micrus/bmode` even without a robot) | `rig_status` shows them under "running now"; stop them with Ctrl+C in their terminal |
| `process has died ... exit code 2` right after start | that node refused to start and printed why on the line above | read the line above; fix the config value it names |

---

## Test 1: robot connection, topics, saving poses, recorded data

**Before**
- PC on the robot's LAN; `rig.yaml robot.ip` set. Check `ping <robot ip>` answers.
- On the pendant: robot powered on, brakes released. Enter the robot values (IP, TCP from Installation >
  General > TCP, probe type; the payload is read from the controller with `m`):
  ```bash
  ros2 run usrig_robot rig_setup --section robot
  ```
  Saved poses store the TCP / payload / probe type, so set them **before** saving poses.
- Nothing is commanded in this test; the limits can still be null.

**1.1 Start the robot (terminal 1)**
```bash
ros2 launch usrig_bringup test1_robot.launch.py
```
Expect, every second, `RTDE 500 Hz ... samples lost 0`. If the robot is not reachable the node stops within
3 s and says so (check the IP / cable / that the controller is on).

**1.2 Topics and nodes (terminal 2)**
```bash
ros2 node list
```
Expect `/robot_state` and `/rig_static_tf`.
```bash
ros2 topic list
```
Expect `/robot/state`, `/joint_states`, `/robot/tcp_pose`, `/robot/wrench`, `/robot/cmd_executed`, `/tf_static`.
```bash
ros2 topic hz /robot/state
```
Expect about 500 Hz.
```bash
ros2 topic echo --once /robot/tcp_pose
```
Expect the position in metres and the orientation as a quaternion. Compare with the pendant (Move tab, Base
view). The pendant shows mm and a rotation vector.
```bash
ros2 run usrig_qc stream_check --robot --duration 60
```
Expect 500 Hz ±1 %, 0 repeated controller times.

**1.3 Watch the pose live (terminal 3)**
```bash
ros2 run usrig_robot pose_monitor
```
Press Enter to set the reference. Move the arm with the pendant jog, the pendant freedrive button or your
freedrive pedal: the change shows in the base and probe frames. With the pedal, mode FREEDRIVE / source
kinesthetic must appear while it is held (needs `freedrive.input` in `rig.yaml`).

**1.4 Save poses (terminal 2)**
Move the arm to a pose (pendant or freedrive), then:
```bash
ros2 run usrig_presets preset_check save home
```
Repeat for each pose (`save above_phantom`, `save scan_start`, ...). Then:
```bash
ros2 run usrig_presets preset_check list
```
They are in `~/data/presets/base_presets.yaml`. Move the arm away and run `list` again: the
"mm from here" column changes. Going back to a pose by software (`preset_check go home`) needs robot_control
and the limits (test 4 / `REAL_ROBOT_CHECKS.md`).

**1.5 Record and check the data**
Stop terminal 1 (Ctrl+C), start it again with recording:
```bash
ros2 launch usrig_bringup test1_robot.launch.py record:=true
```
Move the arm by hand / pendant for about 30 s, then Ctrl+C. The launch printed the folder; check it:
```bash
ros2 run usrig_qc bag_check ~/data/checks/test1_robot_<date_time> --expect robot
```
**Pass**: rate 500 Hz (controller clock), 0 samples lost, `/joint_states`, `/robot/tcp_pose`, `/robot/wrench`
with the same count as `/robot/state`, and "TCP moved ... mm" matching what you did.

**1.6 Measure the workspace while you are here** (read-only; the robot connected, you move it by pendant or
freedrive). `m` measures the box from corners you touch with the probe tip, the tilt axis with the probe square to
the phantom, and the free-air F/T noise for the contact thresholds; the other values you type:
```bash
ros2 run usrig_robot rig_setup --section workspace force
```
`ros2 run usrig_robot rig_setup --review` shows what is still null and checks the values against each other.

**1.7 Optional: freedrive check** (pedal set up)
```bash
ros2 run usrig_freedrive freedrive_check
```

---

## Test 2: ultrasound (micrus) + robot, with calibration

**Before**: test 1 passed. MicrUs connected, probe mounted on the arm. In `session.yaml`, set the micrus scan
settings and `micrus.command`, `micrus.licence_file`, and `session.phantom_id` (one of `phantoms:`). Pick a
session id for the calibration session, e.g. `2026-10-20_vascular_4vein_op1_s01`, below `<ID>`.

**2.1 Start ultrasound + robot (terminal 1)**
```bash
ros2 launch usrig_bringup test2_micrus_robot.launch.py session_id:=<ID> session_json:=$HOME/data/sessions/<ID>/session.json
```
(The session.json is created by the first calibration tool that writes it; the robot node uses it for the
phantom frame once a registration exists.)

**2.2 Topics (terminal 2)**
```bash
ros2 topic hz /micrus/bmode
```
Expect about 27 fps (`micrus.expected_fps`).
```bash
ros2 run usrig_qc stream_check --micrus --duration 60
```
Expect 0 frame_index gaps.
```bash
ros2 run usrig_qc status_console
```
Shows the live rates of both streams.

**2.3 Calibrations** (`docs/calibration.md` has the details; all tools only read the robot)
1. Pointer tip (once per pointer):
   ```bash
   ros2 run usrig_robot pivot_calib --tool pointer
   ```
   12-20 poses, tip in the divot, tilt 20-40 deg. Accept the residual, then run again with `--write`.
2. Phantom registration (each session, and whenever the phantom moved). Needs
   `~/data/phantoms/<phantom_id>.yaml` with `fiducials_mm` and `rig.yaml registration_max_residual_mm`:
   ```bash
   ros2 run usrig_episode register_phantom ~/data/sessions/<ID> --tool pointer
   ```
   Restart terminal 1 afterwards so the robot node publishes the pose in the phantom frame.
3. Ultrasound temporal offset: record while compressing the phantom periodically (about 0.5 Hz, 5-10 mm,
   20-30 s). By hand with the freedrive pedal is fine (the tool cross-correlates the robot's probe-axis
   position with the image). Stop terminal 1 and start it with `record:=true`:
   ```bash
   ros2 launch usrig_bringup test2_micrus_robot.launch.py session_id:=<ID> session_json:=$HOME/data/sessions/<ID>/session.json record:=true
   ```
   After Ctrl+C:
   ```bash
   python tools/temporal_calib.py ~/data/checks/test2_micrus_robot_<date_time> --us-rows 150 380 --session ~/data/sessions/<ID>
   ```
   (`--us-rows`: an image band containing a bright structure that moves with the compression.) Put the
   reported offset into this PC's `~/.config/usrig/machine.yaml` as
   `rig: {calibration: {ultrasound_time_offset_s: <value>}}`. It depends on the PC; the capture PC measures its own.
4. Ultrasound image to probe (`prob_cali_mtx`): pending the lab's calibration file.

**2.4 Check the recording**
```bash
ros2 run usrig_qc bag_check ~/data/checks/test2_micrus_robot_<date_time> --expect robot micrus
```
**Pass**: robot as in test 1; `/micrus/bmode` in the fps band; 0 frame_index gaps; settings constant; a robot
sample within 10 ms on both sides of every ultrasound frame. Ultrasound files (HDF5, snapshots) are in
`~/data/micrus/<ID>/`.

If robot samples are lost in the bag while the robot node logs `samples lost 0`, the PC was overloaded
(seen once in simulation right after URSim booted): close other programs and record again. On the capture PC
follow `capture_pc_setup.md` (real-time priority).

---

## Test 3: wrist camera + robot, with calibration

**Before**: test 1 passed. D415 on a **USB 3** port (`rs-enumerate-devices -s` shows USB 3.x), its serial in
`session.yaml wrist_camera.serial`, the calibration board in `wrist_camera.calib_board`.

**3.1 Lock the exposure first** (the launch refuses to start while exposure / white balance are null):
```bash
ros2 launch usrig_bringup test3_camera_robot.launch.py auto:=true
```
Point the camera at the phantom under the session lighting, then in terminal 2:
```bash
ros2 run usrig_qc camera_autolock --write
```
Stop terminal 1.

**3.2 Start camera + robot (terminal 1)**
```bash
ros2 launch usrig_bringup test3_camera_robot.launch.py
```

**3.3 Topics (terminal 2)**
```bash
ros2 topic hz /wrist/d415/color/image_raw
```
Expect `wrist_camera.fps` (30).
```bash
ros2 run usrig_qc stream_check --camera --duration 60
```
Expect both streams at 30 Hz, stamp offset in ms.
```bash
ros2 run usrig_qc stream_check --camera --duration 10 --depth-roi 200 140 440 340
```
Run with the probe in contact: the depth at contact distance must be valid (open decision 9).
View the images with `ros2 run rqt_image_view rqt_image_view`.

**3.4 Calibrations**
1. Hand-eye (after every change to the camera mount): board fixed on the table.
   ```bash
   ros2 run usrig_episode handeye_calib --session ~/data/sessions/<ID>
   ```
   12-20 poses (whole board visible, tilt 20-40 deg, rotate, vary the distance), `q` to solve. Accept, then
   store it:
   ```bash
   ros2 run usrig_episode handeye_calib --resolve ~/data/calibration/handeye_<date_time> --write-rig
   ```
   Restart terminal 1: `/tf_static` now has `flange -> wrist_camera`:
   ```bash
   ros2 run tf2_ros tf2_echo flange wrist_camera
   ```
2. Camera temporal offset: as in 2.3 step 3 with an ArUco marker fixed in view, recording with
   `record:=true`, then:
   ```bash
   python tools/temporal_calib.py ~/data/checks/test3_camera_robot_<date_time> --aruco DICT_4X4_50 --marker-mm 40 --session ~/data/sessions/<ID>
   ```
   Put the offset into `rig.yaml calibration.camera_time_offset_s`.

**3.5 Check a recording**
```bash
ros2 launch usrig_bringup test3_camera_robot.launch.py record:=true
```
Move the arm about 30 s, then Ctrl+C:
```bash
ros2 run usrig_qc bag_check ~/data/checks/test3_camera_robot_<date_time> --expect robot camera
```
**Pass**: robot as in test 1; colour and depth at 30 Hz; camera_info present; depth 16UC1.

---

## Test 4: everything together (final check)

**Before**: tests 1-3 passed, calibrations stored. For motion with robot_control: all `rig.yaml` values robot_control needs (27 null today) and the keymap speeds, entered with `ros2 run usrig_robot rig_setup`
and the keymap speeds filled (`rig_limits.md`), and `first_motion_checklist.md` sections B-D done. Without
them, run test 4 with `control:=false` and freedrive or the pendant.

**4.1 Pre-session check (terminal 2, with the launch from 4.2 running)**
```bash
ros2 run usrig_qc presession_check --stage 0 --session ~/data/sessions/<ID> --planned-hours 1
```
Add `--control` if robot_control runs, and `--no-camera` if you run without the camera (`camera:=false`). Every
row must be PASS or N/A. `episode_manager` refuses a real session without this PASS (12 h validity), and refuses
to record the camera on a `--no-camera` PASS.

The disk row needs the data rate of an episode. Once an episode exists, the check measures it from the latest
episode itself. Before the first episode it uses a conservative estimate from your latest test 2 and test 3
recordings (the row says which). `--mb-per-s <rate>` overrides it.

**4.2 Start everything (terminal 1)**
Teleop (robot_control, keyboard):
```bash
ros2 launch usrig_bringup test4_full_system.launch.py session_id:=<ID> session_json:=$HOME/data/sessions/<ID>/session.json control:=true
```
Freedrive (pedal, nothing commanded):
```bash
ros2 launch usrig_bringup test4_full_system.launch.py session_id:=<ID> session_json:=$HOME/data/sessions/<ID>/session.json input:=freedrive
```
Start the launch first, then run 4.1 (it measures the live streams). If 4.1 failed, stop the launch, fix
it, and start again.

**4.3 Operator view (terminal 3)**
```bash
ros2 run usrig_qc status_console
```

**4.4 Record 3-5 Stage 0 episodes**
Press **B** to start: recording starts, a 1.5 s free-air hold, then `start`. With control, it descends to
contact and teleop is on. With freedrive, guide with the pedal. Then **Space** (view reached), **1** / **2**
/ **3** (outcome), **B** to stop. Each episode is in `~/data/sessions/<ID>/ep_NNNN/` with `qc.json`.
```bash
ros2 run usrig_qc episode_qc ~/data/sessions/<ID>/ep_0001
```
```bash
ros2 run usrig_qc bag_check ~/data/sessions/<ID>/ep_0001 --expect robot micrus camera
```
```bash
ros2 run usrig_qc collection_progress
```

**4.5 Motion checks** (control:=true only; `REAL_ROBOT_CHECKS.md`)
```bash
ros2 run usrig_teleop teleop_check --axis slide_along --speed 5 --seconds 2 --back
```
```bash
ros2 run usrig_presets preset_check go home
```

**4.6 Export check**
```bash
python export/pipeline_check.py run
```
This checks the export chain on synthetic data. Exporting the real Stage 0 episodes also needs `probe.type`
and `prob_cali_mtx` (`DATA_COLLECTION_GUIDE.md` section 11).

**Pass**: pre-session PASS; every episode has `episode.json` + `qc.json` with QC pass, except checks whose
thresholds you have not set yet, which show `unset`; `bag_check` PASS on the episodes; HDF5 files in
`~/data/micrus/<ID>/`.

---

## Simulation dry run of the launch files (no hardware)
With URSim (`scripts/ursim.sh`) and the micrus simulator:
```bash
ros2 launch usrig_bringup test1_robot.launch.py sim:=true record:=true
```
```bash
ros2 launch usrig_bringup test2_micrus_robot.launch.py session_id:=sim_test sim:=true record:=true
```
```bash
ros2 launch usrig_bringup test4_full_system.launch.py session_id:=sim_test sim:=true camera:=false
```
Data goes to `~/data_sim`. URSim runs slower than real time, so `bag_check` judges the robot rate on the
controller clock.
