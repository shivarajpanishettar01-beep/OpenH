# Collection day (draft: commands are written, not yet run on the rig)

Every terminal starts with `cd ~/OpenH && source scripts/env.sh`.

## Power-on
1. Power the UR7e, release brakes on the pendant (you, not software). Check the controller TCP
   matches `rig.yaml robot.tcp_setting`.
2. Plug the MicrUs scanner and the D415 (USB 3 port, strain relief on the arm cable).
3. Check the D415 enumerates on USB 3:
   ```bash
   rs-enumerate-devices -s
   ```

## Bring up (terminal 1)
```bash
ros2 launch usrig_bringup collect.launch.py session_id:=<SESSION_ID> session_json:=$HOME/data/<SESSION_ID>/session.json control:=true teleop:=true
```
(`control:=true` refuses to start while any `rig.yaml` limit is null.)

## Pre-session check (terminal 2)
```bash
ros2 run usrig_qc presession_check --stage <0|1|2> --session ~/data/<SESSION_ID> --control --planned-hours <h> --mb-per-s <measured>
```
Proceed only on PASS.

## Calibrations (when needed)
```bash
ros2 run usrig_episode register_phantom ~/data/<SESSION_ID>
```
```bash
ros2 run usrig_episode rig_static_tf --ros-args -p session_json:=$HOME/data/<SESSION_ID>/session.json
```
Hand-eye (after touching the camera mount):
```bash
ros2 run usrig_episode handeye_calib ~/data/<SESSION_ID>
```

## Collect (terminal 3)
```bash
ros2 run usrig_episode episode_manager --ros-args -p session_id:=<SESSION_ID> -p robot_control:=true -p operator_id:=op1
```
Operator status (headless, used for collection):
```bash
ros2 run usrig_qc status_console
```
Dashboard: demos only (it costs ~1.3 CPU cores):
```bash
ros2 run usrig_dashboard dashboard --ros-args -p session_id:=<SESSION_ID>
```
Keys: W/S A/D Q/E R/F Z/C move, T/G force, Shift fine, Space view reached, 1/2/3 outcome,
Esc discard, N next, P hold, K acknowledge fault, B start/stop (Stage 0),
O view lost (failure), U recovery start, I recovery end (view reacquired).

Progress against the 10 h commitment:
```bash
ros2 run usrig_qc collection_progress
```

## End of session
1. Ctrl+C the episode manager, then the launch (micrus freezes and powers down on SIGINT).
2. Retract and park the robot from the pendant.
3. Review in the dashboard's Review tab; re-run QC on any episode:
   ```bash
   ros2 run usrig_qc episode_qc ~/data/<SESSION_ID>/ep_0001
   ```

## Backup
```bash
rsync -a --info=progress2 ~/data/<SESSION_ID> ~/MicrUs/<SESSION_ID> <BACKUP_DEST>/
```
Verify the copy before deleting anything from the rig disk.
