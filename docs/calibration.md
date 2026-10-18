# Calibration on the rig

All results are 4x4 rigid transforms in metres, stored in `src/usrig_bringup/config/rig.yaml`
with a provenance record (date, method, residual, number of poses, raw-capture folder). Raw
captures go to `<root>/calibration` (`~/data/calibration`; simulation: `~/data_sim/calibration`), so every result can be
re-solved or audited. Redo a calibration whenever its mount is touched.

| Calibration | Tool | Stored as | Used by |
|---|---|---|---|
| Active TCP of the controller | pendant (Installation > TCP) | `robot.tcp_setting` | robot_control (refuses on mismatch), all frame conversions |
| Calibration-pointer tip (arm) | `ros2 run usrig_robot pivot_calib --tool pointer --write` | `pointer.tip_offset_flange_m` + `pointer.tip_calibration` | `register_phantom --tool pointer` |
| Wrist D415 to flange (hand-eye) | `ros2 run usrig_episode handeye_calib --write-rig` | `calibration.wrist_cam_cali_mtx` (T_flange_camera) + `wrist_cam_cali_info` | `rig_static_tf` (/tf_static), Open-H export (`wrist_cali_mtx`) |
| D415 intrinsics | factory, from `camera_info` in every bag | `meta/calibration/camera_intrinsics.json` at export | export |
| Ultrasound image to probe | **pending** (the lab's calibration file) | `calibration.prob_cali_mtx` | Open-H export |
| Phantom registration | `ros2 run usrig_episode register_phantom <session> --tool pointer` | `session.json phantom_registration` | robot_state (tcp_phantom), waypoints, paths |
| Temporal offsets | `tools/temporal_calib.py` | `calibration.ultrasound_time_offset_s`, `camera_time_offset_s` | export |

Order: TCP setting -> pointer pivot -> hand-eye -> phantom registration (each session) -> temporal.

## Pointer pivot calibration (arm)
1. Mount the calibration pointer; set `robot.tcp_setting` to the active TCP shown on the pendant.
2. Fix a divot (a small cone) on the table. Freedrive the arm so the pointer tip sits in the divot.
3. `ros2 run usrig_robot pivot_calib --tool pointer` and press Enter at 12-20 poses, keeping the tip
   in the divot while tilting the pointer 20-40 deg in several directions and rotating it about its axis.
4. `q` solves. Accept when the residual rms is small for your purpose (sub-millimetre is typical with a
   good divot) and the orientation spread is above ~40 deg; then run again with `--write`.
   The tool reads the robot only; it never commands motion.

## Phantom fiducial files (one per phantom, before registration)
`register_phantom` needs `<data root>/phantoms/<phantom_id>.yaml` for the session's phantom
(`vascular_4vein.yaml`, `central_line_gen2.yaml`). Neither exists yet. The coordinates must come from the
phantom's drawing / datasheet or your own measurement of marked points; nothing in the rig proposes them, and
fiducials with null or missing values are refused:
```yaml
# <root>/phantoms/vascular_4vein.yaml   (phantom frame, millimetres; at least 4 points, not all in one line)
fiducials_mm:
  F1: [null, null, null]
  F2: [null, null, null]
  F3: [null, null, null]
  F4: [null, null, null]
```
Use points the probe tip or the calibrated pointer can touch reproducibly (corner marks, divots). The phantom
frame you choose here is the frame of every waypoint and of `tcp_phantom` in the data.

## Wrist-camera hand-eye
1. Set `wrist_camera.calib_board` in `session.yaml` (checkerboard: `squares_x`, `squares_y`, `square_mm`;
   ChArUco also `marker_mm`, `dictionary`). Measure the printed square size with calipers.
2. Fix the board flat on the table; start the camera (locked exposure) and robot state:
   `ros2 launch usrig_bringup wrist_camera.launch.py` and `ros2 launch usrig_bringup robot.launch.py`.
3. `ros2 run usrig_episode handeye_calib` and capture 12-20 poses (whole board visible, tilt 20-40 deg,
   rotate about the camera axis, vary the distance). Each capture prints corners, reprojection error
   and distance.
4. `q` solves with all OpenCV methods and keeps the lowest residual. Accept, then store with
   `--write-rig` (re-solve the same captures without the robot: `handeye_calib --resolve <capture dir> --write-rig`).

## Ultrasound image to probe (pending)
When the lab's calibration file is available, it is added as `calibration.prob_cali_mtx` with its
provenance (source file, method, residual); the format and frame direction will be confirmed then.
