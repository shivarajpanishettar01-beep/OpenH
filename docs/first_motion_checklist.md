# First motion on the real UR7e: checklist

Purpose: prove every safety layer on the real arm **before any data is collected**. No recording, no
phantom until section E. Run it once on the capture PC, and again whenever `rig.yaml` limits, the
probe mount, the TCP or the control code change. Record the result of every line (bottom table).

Two people: the **operator** (keyboard, terminal) and the **safety person** (hand on the teach-pendant
E-stop the whole time, says "stop" and presses it at any doubt). Anyone can stop the test.

Every terminal: `source ~/OpenH/scripts/env.sh`.

## A. Before powering the arm (all must be true)
- [ ] Software checks pass on the capture PC: `bash scripts/test.sh` and `python export/pipeline_check.py run`.
      (The URSim dry run was skipped by decision on 13 Oct 2026: sections C-D are the first motion of this
      control code on any robot, so run them slowly, one item at a time.)
- [ ] Every limit in `rig.yaml` is filled from your measurements and protocol (`docs/rig_limits.md`), entered with
      `ros2 run usrig_robot rig_setup`; `rig_setup --review` shows no ERROR.
      For this first session use **low speed and acceleration caps** (your choice) and a small force band.
- [ ] `keymap.yaml` speeds and `force_step_n` are set and below the `rig.yaml` caps (keyboard_teleop refuses otherwise).
- [ ] A **dedicated operator keyboard** is set in `keymap.yaml device:` (its /dev/input/by-id path): evdev reads
      the device regardless of window focus, so no other keyboard may drive the robot.
- [ ] **UR safety configuration on the pendant** (Installation > Safety) is set by you and is equal to or
      tighter than `rig.yaml`: TCP speed, TCP force, momentum/power, and a safety plane or boundary around
      the phantom table. This layer works even if our software fails.
- [ ] Probe and wrist camera mounted and tightened; cables strain-relieved along the arm, no snag points.
- [ ] Workspace clear of people and objects inside the robot reach; phantom **not** on the table yet.
- [ ] E-stop on the pendant tested (press, release, recover) before starting.

## B. Read-only checks (robot powered, brakes released, nothing commanded)
- [ ] `rig.yaml robot.ip` set; robot in **Remote Control** mode; lab URScript program not running.
- [ ] Pose and connection:
      ```bash
      ros2 run usrig_robot rig_measure pose
      ```
      TCP pose equals the pendant (Move tab, Base) to display precision.
- [ ] Active TCP on the pendant (Installation > TCP) equals `rig.yaml robot.tcp_setting`;
      `probe.tcp_already_includes_probe` is correct.
- [ ] State stream (P2 on the real controller):
      ```bash
      ros2 launch usrig_bringup robot.launch.py
      ```
      ```bash
      ros2 run usrig_qc stream_check --robot --duration 60
      ```
      Expect 500 Hz +-1 %, 0 repeated controller times; robot_state_cpp log: `samples lost 0`.
- [ ] F/T noise in free air for the contact threshold:
      ```bash
      ros2 run usrig_robot rig_measure force-noise --seconds 10
      ```
- [ ] Payload on the pendant (Installation > Payload) equals `robot.payload_kg` / `payload_cog_m`, and the
      free-air force does not change with tilt (R4): freedrive to 4-5 tilts across the cone, Enter at each:
      ```bash
      ros2 run usrig_robot rig_measure force-tilt
      ```
      Probe-axis force spread should be close to the force-noise peak-to-peak; much larger = payload / CoG wrong.

Stop the launch (Ctrl+C) before section C.

## C. Start control in free air (arm well above the table)
- [ ] Pendant **speed slider low** (e.g. 10-25 %) for every first try; raise it only after the item passed.
- [ ] Start robot state + control and the teleop:
      ```bash
      ros2 launch usrig_bringup robot.launch.py control:=true
      ```
      ```bash
      ros2 run usrig_teleop keyboard_teleop
      ```
      ```bash
      ros2 run usrig_qc status_console
      ```
      robot_control must log the controller TCP offset, the controller payload, `controller watchdog: min ... Hz`,
      and no refusal. Every 10 s it checks the control-tick intervals and warns `control ticks: {...}` if any
      tick came later than the speedL lifetime (R8); note p99 / max / late for the capture PC.
- [ ] Live pose in its own terminal (Enter resets the reference): `ros2 run usrig_robot pose_monitor`.
- [ ] One-axis checks with the pose change and PASS / FAIL printed (`REAL_ROBOT_CHECKS.md`), each axis, small
      speed, before the keyboard:
      ```bash
      ros2 run usrig_teleop teleop_check --axis slide_along --speed 5 --seconds 2 --back
      ```
      ```bash
      ros2 run usrig_presets preset_check offset --dx 10 --back
      ```
- [ ] Teleop on (holding still until enabled):
      ```bash
      ros2 service call /robot/set_teleop std_srvs/srv/SetBool "{data: true}"
      ```
- [ ] Each axis, short key presses, fine mode (Shift) first: W/S, A/D (slide), Q/E (rock), R/F (tilt),
      Z/C (rotate). Motion direction matches the probe frame (x along the array, y across, z into tissue).
- [ ] Two keys together (diagonal) move smoothly; releasing all keys ramps to a stop.
- [ ] **Hold-to-move**: motion keys without Ctrl held do nothing; releasing Ctrl mid-motion ramps to a stop.
- [ ] SpaceMouse (if used, `input:=spacemouse`): each cap axis moves the matching probe axis with the right sign
      (fix `keymap.yaml spacemouse.axes` otherwise); releasing the left button stops; stopping `spacenav_node`
      stops the robot (deadman); cap push changes the force setpoint only in contact.

## D. Every safety layer trips (free air)
After each FAULT: robot stopped, status shows FAULT, nothing moves until acknowledged with **K**.
- [ ] **Keyboard unplugged**: hold a key, unplug the operator keyboard: motion stops within the deadman
      timeout (keyboard_teleop stops publishing); plug it back in with no key held: no motion.
- [ ] **Deadman**: hold a key, then stop keyboard_teleop (Ctrl+C in its terminal). Motion stops within
      `limits.deadman_timeout_s`; status shows flag `deadman`. Restart keyboard_teleop.
- [ ] **Workspace box (soft)**: teleop slowly toward one face of the box: motion along that axis stops
      at the face (flag `envelope`), other axes still move. Repeat for each face you can reach safely.
- [ ] **Workspace box (hard)**: with control stopped, move the TCP just outside the box with the pendant;
      start control: it must go to FAULT immediately. Acknowledge (K): only motion back inside is allowed.
- [ ] **Tilt cone**: tilt (R/F, Q/E) toward the cone limit: the tilting motion stops at the cone (flag `tilt`),
      rotation about the probe axis (Z/C) still works.
- [ ] **Speed cap**: hold a slide key without fine mode: measured TCP speed (pendant / `/robot/tcp_pose`)
      never exceeds `max_linear_speed_mm_s`.
- [ ] **Pre-motion refusal**: ask for a target outside the box; it is refused at once and nothing moves
      (copy the current pose from `rig_measure pose`, raise z above the box top):
      ```bash
      ros2 service call /robot/move_to_pose usrig_interfaces/srv/MoveToPose "{target: {header: {frame_id: base}, pose: {position: {x: <x>, y: <y>, z: <above box>}, orientation: {x: <qx>, y: <qy>, z: <qz>, w: <qw>}}}, speed_mm_s: 10.0}"
      ```
      Reply `success: false`, message `... refused before moving`.
- [ ] **Protective stop from software**:
      ```bash
      ros2 service call /robot/protective_stop std_srvs/srv/Trigger
      ```
      Robot goes to protective stop; recover it on the pendant; acknowledge (K).
- [ ] **Controller watchdog** (PC side dies): while sliding slowly, kill robot_control hard:
      ```bash
      pkill -KILL -f usrig_robot/lib/usrig_robot/robot_control
      ```
      The UR7e stops by itself within about one deadman period (ur_rtde watchdog). Note the stop time.
- [ ] **Pendant E-stop** during slow teleop: stops; recover; robot_control reports a fault / stale state.
- [ ] **Stale state**: stop the robot launch while teleop is idle; restart: no motion until teleop is
      enabled again.

## E. First contact (phantom on the table, low force band)
- [ ] Phantom placed and fixed; gel applied. Approach above it with teleop, probe axis down.
- [ ] Zero the F/T sensor in free air:
      ```bash
      ros2 service call /robot/zero_ft std_srvs/srv/Trigger
      ```
- [ ] Slow descent until contact:
      ```bash
      ros2 service call /robot/descend_to_contact usrig_interfaces/srv/DescendToContact "{force_setpoint_n: 0.0, max_travel_mm: 0.0}"
      ```
      Stops at the contact threshold; status shows the force setpoint; the probe **presses into** the
      phantom, it does not pull away (**force-mode sign check**: if it pulls away, stop and report).
- [ ] Force setpoint nudges (T/G) change the measured probe-axis force within `force_band_n`.
- [ ] **Force ceiling**: for this test only, set a temporary low `max_contact_force_n` (your value, just
      above the band), restart control, descend and raise the setpoint (T): FAULT when the ceiling is
      exceeded; robot stops. Restore the real value afterwards.
- [ ] Teleop in contact: slide and rock while the force stays in the band; release keys: holds position and force.
- [ ] **Contact loss**: slide slowly off the phantom edge (or lift it away): `contact_lost` event, force mode
      stops, the probe holds; **J** (recontact) descends slowly back into contact.
- [ ] Retract:
      ```bash
      ros2 service call /robot/retract std_srvs/srv/Trigger
      ```

## F. Recording check (still no real session)
- [ ] With micrus (real scanner), camera and robot running, run `presession_check --stage 0 --control`
      for a throw-away session (episode_manager refuses a real session without a fresh PASS), then record
      one Stage 0 test episode (`collect.launch.py ... episode_manager:=true control:=true teleop:=true`),
      press B to start and stop. Check its `qc.json`: `robot_sample_gaps` 0, `micrus_frame_gaps` 0,
      `wrist_rate` ok, events present.

## Result table (fill in)
| Item | Pass / fail | Measured value / note | Who | Date |
|---|---|---|---|---|
| B pose vs pendant | | | | |
| B 500 Hz, samples lost | | | | |
| D deadman stop time | | | | |
| D box soft / hard | | | | |
| D tilt cone | | | | |
| D speed cap (max measured) | | | | |
| D protective stop | | | | |
| D watchdog stop time | | | | |
| D E-stop | | | | |
| E force-mode sign | | | | |
| E force ceiling FAULT | | | | |
| F test episode QC | | | | |

Any failure: stop, do not collect, write down what happened (time, command, status line), fix it with a
test that reproduces it (`scripts/test.sh`), then repeat the section from its first item.
