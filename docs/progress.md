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
| P3 | Control + teleop | **code done, not run** | path B chosen; needs the 20 limits for the URSim dry run |
| P4 | Episode recorder + manager | **passed in simulation** | 5 episodes incl. SIGKILL mid-recording; crash recovery verified (`P4_results.md`) |
| P5 | Registration, waypoints, paths, plan | code done, not run | needs robot motion; plan needs failure mapping + magnitudes |
| P6 | QC + pre-session check | done, tested on bags | per-episode QC on real MCAP bags; checks every source for gaps |
| P7 | Dashboard | done (demos only) | runs stably on PySide6 6.8.3; collection is headless |
| P8 | Export (Open-H, LeRobot v3.0) | code done, not run end to end | needs real episodes and the LeRobot 0.6.0 env |
| P9 | Real-hardware verification, calibration, Stages 0-2 | not started | after the first-motion checklist |

## Done
**Code base** (9 ROS 2 packages, 46 unit tests, fresh clone builds and passes in a clean shell)
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

**Data integrity** (`data_integrity.md`)
- Loss measured at every stage under load. Fixed: Python robot reader (replaced by C++), shallow
  publisher queues, a CPU-heavy QC monitor, readiness subscriptions that decoded every image, a GUI
  crash (PySide6 6.12 + pyqtgraph 0.14) and a GUI that did not exit.
- The micrus HDF5 keeps every ultrasound frame even when the bag misses one.

**Decisions taken**
- Control path B (13 Oct). URSim only for the control dry run and later for demos.
- GUI for demos only; collection headless.
- Action = incremental TCP pose (proposal); wrist depth exported losslessly.
- Code licence MIT; repo public.

## Not done / needed next
| Item | Needs | Owner |
|---|---|---|
| Safety limits (20 values in `rig.yaml`) + keymap speeds | your measurements and safety protocol (`rig_limits.md`) | you |
| Failure-mode mapping to the four classes; staging magnitudes (`failure_catalog.yaml`) | your failure-mode work | you |
| URSim control dry run | the limits | then me |
| Capture PC setup, P1b camera on USB 3, micrus licence, rtprio, data disk (> 50 GB) | the capture PC | you + me |
| Real UR7e: read-only P2, then `first_motion_checklist.md` | capture PC + limits + your go-ahead | you + me |
| Ultrasound image-to-probe calibration `prob_cali_mtx` | the lab's calibration file (pending) | you, then me |
| Calibrations on the rig: pointer pivot, hand-eye, phantom registration (fiducial files per phantom), temporal offsets (tools ready, `calibration.md`) | real hardware | you + me |
| Waypoints, paths and a locked plan per phantom (P5) | robot motion | you + me |
| Export end to end on real episodes + Open-H `validate_formatting.py` | recorded episodes, LeRobot 0.6.0 env | me |
| Release extras: robot description (URDF / USD / DH), vessel masks on a subset, model-assisted labelling | later | open |

Deadline from the proposal: **representative sample by 1 December 2026**, validated data by 31 January 2027.

See also: `NOT_VERIFIED.md` (everything not yet checked on hardware), `OPEN_DECISIONS.md`,
`proposal_alignment.md`, `first_motion_checklist.md`, `collection_day.md`.
