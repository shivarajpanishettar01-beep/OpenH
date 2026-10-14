# Progress so far (as of 13 October 2026)

Robotic-ultrasound data-collection rig for the IIT Mandi Open-H Embodiment V2 contribution
(UR7e + Telemed probe via micrus + wrist RealSense D415, ROS 2 Humble, one PC, one clock).
Repository: https://github.com/shivarajpanishettar01-beep/OpenH (public, MIT).
GR00T-H is out of scope for now (separate task).

## Phase status
| Phase | What | Status | Evidence |
|---|---|---|---|
| P0 | Survey of the old UR code and the machine | done | `P0_survey.md` |
| P1 | micrus bringup (simulator) | **passed** | 27.35 Hz, 0 frame gaps in 5 min, clean power-down on Ctrl+C (`P1_results.md`) |
| P1b | Wrist D415 bringup | **blocked on hardware** | dev PC only gives USB 2.1; runs on the capture PC (`capture_pc_setup.md`) |
| P2 | robot_state | URSim done; real UR7e pending | pose matches pendant; URSim runs at 0.968x real time so its 500 Hz check is not meaningful (`P2_results.md`) |
| P3 | Control + teleop | **code done, mocked tests pass** | path B; URSim run skipped (decision 13 Oct); first motion on the real arm needs the 22 limits |
| P4 | Episode recorder + manager | **passed in simulation** | 5 episodes incl. SIGKILL mid-recording; crash recovery verified (`P4_results.md`) |
| P5 | Registration, waypoints, paths, plan | code done, not run | waypoints executable only if verified with the current TCP / probe / payload; moves and paths pre-checked; needs robot motion |
| P6 | QC + pre-session check | done, tested on bags | per-episode QC on real MCAP bags; checks every source for gaps |
| P7 | Dashboard | done (demos only) | runs stably on PySide6 6.8.3; collection is headless |
| P8 | Export (Open-H, LeRobot v3.0) | **verified end to end on synthetic episodes** | `pipeline_check.py`, 3 export defects fixed; real episodes pending |
| P9 | Real-hardware verification, calibration, Stages 0-2 | not started | after the first-motion checklist |

## Done
**Code base** (11 ROS 2 packages, 126 tests, fresh clone builds and passes in a clean shell)
- Mode packages (14 Oct): `usrig_teleop`, `usrig_presets` (waypoints / paths / plans moved from usrig_episode),
  `usrig_freedrive`; a check per mode that prints the pose change on the real arm (`teleop_check`,
  `preset_check`, `freedrive_check`) and `pose_monitor` (`REAL_ROBOT_CHECKS.md`). Freedrive works in Remote
  Control (user, 14 Oct).
- Freedrive (kinesthetic) collection (13 Oct): pedal on a UR "Freedrive" input, per-sample labels from the
  controller's inputs, `collection_mode:=freedrive`, QC and export support (`FREEDRIVE_COLLECTION_GUIDE.md`).
- Audit against the master prompt (13 Oct): `IMPLEMENTATION_AUDIT.md`, evidence in `TEST_AND_VERIFICATION.md`;
  guides `TELEOPERATION_GUIDE.md`, `WAYPOINT_COLLECTION_GUIDE.md`, `DATA_COLLECTION_GUIDE.md`.
- micrus launch from config, SIGINT power-down; nothing in micrus modified.
- `robot_state_cpp` (C++, ur_client_library 2.15, blocking reads): **0 robot samples lost** in every
  load test and episode; ~6 % CPU (the Python version lost 7-30 %).
- `robot_control` (path B: ur_rtde `forceMode` + `speedL`): safety envelope (box, tilt cone, speed,
  acceleration, force ceiling), deadman, FAULT + acknowledge, **controller-side watchdog**, refuses to
  start with null limits, without Remote Control, or with a TCP mismatch. Real-time priority when allowed.
- Keyboard teleop (evdev, ramped probe-frame twist); keys for outcome, view reached, failure onset
  (O), recovery start (U), recovery end (I).
- `episode_manager`: Stages 0/1/2, crash-safe recorder (MCAP + micrus HDF5, readiness gate, record
  guard), resume, stage gates, simulated data kept apart and never exported.
- QC: live monitor, per-episode QC from the bag stamps, pre-session check, headless `status_console`,
  `collection_progress` (hours per phantom vs 6 h / 4 h, failure/recovery share vs 20 %).
- Annotation: automatic phases / gestures / failure-recovery status per episode; offline
  `annotator` for expert / trainee labels with provenance and QA status.
- Export following the proposal: 6-DoF incremental action, per-step status / phase / gesture,
  lossless 16-bit wrist depth, calibration files, contiguous splits.
- Tools: `rig_measure` (read-only box / tilt axis / F/T noise), hand-eye calibration, phantom
  registration, waypoint teach / verify, path editor, plan generator, temporal calibration (ultrasound
  keypoint + ArUco, drift start vs end of session), camera auto-then-lock.
- Calibration (`calibration.md`): pointer pivot calibration (arm) and wrist-camera hand-eye
  (checkerboard or ChArUco) with raw captures saved, re-solve, flange frame, provenance in rig.yaml;
  tested on synthetic data (pivot within 0.5 mm at 0.2 mm noise; rendered board within 2 mm / 0.5 deg).

**Control logic verified in a kinematic simulator** (`src/usrig_robot/test/test_control_sim.py`,
test-only limits; rig.yaml unchanged): move to pose, unreachable target -> timeout, probe-frame teleop
directions incl. diagonal, deadman stop, hard envelope FAULT -> acknowledge -> inward-only recovery,
force-ceiling FAULT (acknowledge refused while violated), descend to contact, constant-speed path
(speed error < 10 %), path cancelled by retract. Bugs found and fixed by these tests and the review:
- robot_control crashed on start (ControlStatus message shadowed by the status class);
- motion services had no timeout: an unreachable target blocked the service forever;
- a retract that cancelled a path was itself cancelled by the path's cleanup (probe stayed in contact);
- keyboard loss kept the last motion command publishing at 100 Hz, so the deadman never tripped.
Every motion now publishes a `motion` event (ok / timeout / preempted / FAULT) into the episode bag.
- R4: robot_control refuses to start if the controller payload / CoG differs from `robot.payload_kg` /
  `payload_cog_m`; `rig_measure force-tilt` measures free-air force vs tilt (read-only).
- R8: control-tick intervals reported every 10 s (log; `control_timing` event in the bag while recording;
  per-episode QC check `control_timing`); speedL lifetime configurable (`robot.speedl_time_ticks`, default 1 tick).

**Data integrity** (`data_integrity.md`)
- Loss measured at every stage under load. Fixed: Python robot reader (replaced by C++), shallow
  publisher queues, a CPU-heavy QC monitor, readiness subscriptions that decoded every image, a GUI
  crash (PySide6 6.12 + pyqtgraph 0.14) and a GUI that did not exit.
- The micrus HDF5 keeps every ultrasound frame even when the bag misses one.

**Decisions taken**
- Control path B (13 Oct). URSim control dry run skipped (13 Oct): motion checks on the real arm in free air.
- GUI for demos only; collection headless.
- Action = incremental TCP pose (proposal); wrist depth exported losslessly.
- Code licence MIT; repo public.

## What's left (as of 13 October 2026)
All code that can be written and tested without the rig is done: the control review items that need no
hardware (R2-R10, D1-D11) are implemented and tested. Everything left needs your values, the capture PC or the
real robot. In order:

**1. Your inputs (blocking the first motion)**

| Item | Where | Owner |
|---|---|---|
| Safety limits: 22 values incl. payload mass / CoG (`rig_limits.md`) | `rig.yaml` | you |
| Teleop speeds per axis, fine mode (D6) | `keymap.yaml` | you |
| Failure-mode mapping to the four classes; staging methods and magnitudes | `failure_catalog.yaml` | you |
| Gate values `gate01.*`, `gate12.*` | `session.yaml` | you |
| Open decisions 1 (10 h: what counts), 4 (beamformed in bags), 5, 6, 9, 13-17 | `OPEN_DECISIONS.md` | you |
| micrus licence file path, USB ids for the pre-session check | `session.yaml` | you |
| Freedrive: foot pedal wired to a configurable input + "Freedrive" action on the pendant, `freedrive.input`, wrist handle | robot, `rig.yaml` | you |

**2. ~~URSim control dry run~~** skipped by decision (13 Oct): its items are checked on the real arm in free
air, at low speed, in `first_motion_checklist.md` C-D.

**3. Capture PC**

| Item | Owner |
|---|---|
| Setup per `capture_pc_setup.md`: rtprio limit, data disk > 50 GB, micrus licence | you + me |
| P1b: wrist D415 on USB 3 (rates, stamps, depth at contact distance; decision 9 on the mount) | me |
| P1 on the real scanner (rate, SIGINT power-down) | me |
| Loss budget with the scanner, D415 and UR7e together (`data_integrity.md`) | me |

**4. Real UR7e** (needs 1-3 and your go-ahead), `first_motion_checklist.md`

| Item | Control review ref. |
|---|---|
| Read-only P2 on the controller: 500 Hz, 0 samples lost, TCP vs pendant | - |
| Payload on the pendant + `rig_measure force-tilt` free-air sweep; `rig_measure force-noise` | R4 |
| Free-air control: each teleop axis, hold-to-move, deadman, FAULT / acknowledge; SpaceMouse axis signs and scaling | D1, D2 |
| Control-tick jitter and teleop latency on the capture PC; choose `speedl_time_ticks` and `qc.max_late_tick_frac` | R8 |
| Contact tests at low force on the phantom: force step, slide, rock / tilt about the probe face, contact loss at the edge, forceMode sign | R1, R2 |

**5. Calibration on the rig** (`calibration.md`, tools ready): pointer pivot, wrist-camera hand-eye, phantom
registration (fiducial file per phantom), temporal offsets (ultrasound + ArUco), image-to-probe `prob_cali_mtx`
(needs the lab's calibration file). Owner: you + me.

**6. Plan per phantom (P5)**: teach and verify waypoints, paths, generate and lock the plan. Owner: you + me.

**7. Collection and release**

| Item | Owner |
|---|---|
| Stage 0 episode end to end through export on real data (the chain is verified on synthetic data) | me |
| Stages 1-2 collection (`DATA_COLLECTION_GUIDE.md`), hours per phantom vs 6 h / 4 h, failure / recovery share 20 % | you + me |
| Export on real episodes through LeRobot 0.6.0 + Open-H `validate_formatting.py` | me |
| Release extras: robot description (URDF / USD / DH), vessel masks on a subset, model-assisted labelling | open |

Deadline from the proposal: **representative sample by 1 December 2026**, validated data by 31 January 2027.

Control review before the teleoperation spec: `control_review.md` (10 findings, 11 decisions).

See also: `NOT_VERIFIED.md` (everything not yet checked on hardware), `OPEN_DECISIONS.md`,
`proposal_alignment.md`, `first_motion_checklist.md`, `DATA_COLLECTION_GUIDE.md`.
