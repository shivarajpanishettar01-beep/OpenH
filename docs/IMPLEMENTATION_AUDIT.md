# Implementation audit (13 October 2026)

Audit of the checkout at `/home/shiv/OpenH` against the master prompt (teleoperation + preset
waypoints + recording + Open-H export). Every state below is backed by a command that was run or a
file and test named in the row; nothing has run on the UR7e, the real micrus scanner or the D415.

**Local vs public repository.** The local `main` holds the commits of this audit on top of
`origin/main` (github.com/shivarajpanishettar01-beep/OpenH); they are not pushed yet. No other
differences (`git status` clean before the audit, `git diff origin/main` = the audit commits).

**Decision taken during the audit:** the URSim control dry run is **skipped** (user, 13 Oct 2026): the PC
talks to the real controller over the LAN exactly as to URSim, and URSim cannot test force. Motion checks
happen on the real arm in free air (`first_motion_checklist.md` section C) at low speed.

## State vocabulary
| State | Meaning |
|---|---|
| unit | pure logic tested (`scripts/test.sh`) |
| mocked | integration test against a mocked interface: kinematic fake robot, fake `rtde_control`, fake evdev / spacenav devices, synthetic bags |
| built | compiles / installs and starts |
| sim | run against a simulator: micrus `--simulate`, URSim (read-only), synthetic episodes through the real export chain |
| hardware | verified on the real rig: **none yet** |
| partial / not implemented / blocked | as written; the blocker is named |

## Component table
| Component | Existing implementation | Evidence | Missing work | Verification method | State |
|---|---|---|---|---|---|
| Robot state | `usrig_robot_cpp/robot_state_cpp` (ur_client_library, 500 Hz, one stamp per RTDE sample → `/robot/state`, `/joint_states`, `/robot/tcp_pose`, `/robot/wrench`); Python fallback `robot_state.py` | `P2_results.md` (URSim: pose = pendant), `data_integrity.md` (0 of ~31,000 samples lost per minute under load) | 500 Hz rate and jitter on the real UR7e; `actual_TCP_force` frame | `stream_check --robot` on the UR7e (checklist B) | built, sim; hardware blocked (UR7e) |
| Robot control | `robot_control.py` (single motion authority, one `Motion` at a time), `safety.py` (box, phantom box, tilt cones, speed / accel clamp, force ceiling), `backends.py` (path B: `forceMode` + `speedL`, controller watchdog, TCP + payload checks) | `test_control_sim.py` (25 tests), `test_safety.py` (10), `test_backend.py` (7) | contact behaviour and `forceMode` sign (R1), watchdog stop time, tick jitter on the capture PC | checklist C-E on the real arm | unit, mocked; URSim control skipped by decision; hardware blocked (limits null) |
| Keyboard teleoperation | `keyboard_teleop.py` (evdev, hold-Ctrl enable, ramp, fine mode, device-loss failsafe, SYN_DROPPED resync, speeds checked vs `rig.yaml`), `spacemouse_teleop.py`; `robot_control` deadman, one-publisher guard | `test_ramp.py`, `test_keyboard_failsafe.py`, `test_spacemouse.py`, teleop tests in `test_control_sim.py` | key directions and SpaceMouse signs on the real arm; keymap speeds (null) | checklist C-D | unit, mocked; hardware blocked |
| Waypoint system | `waypoints.py` teach / verify (NCC vs reference snapshot), `path_editor.py`, `plan_generator.py` (+ SHA-256 plan lock), `episode_manager` Stage 0/1/2 execution through `robot_control` services | `test_waypoint_validity.py`, `test_plan.py`, path / move tests in `test_control_sim.py` | teach / verify never run (need robot + micrus snapshot); fiducial files per phantom | `WAYPOINT_COLLECTION_GUIDE.md` on the rig | unit, mocked; hardware blocked |
| Ultrasound acquisition | micrus (closed package) via `micrus.launch.py`; `/micrus/bmode`, `/micrus/frame_info`, `/micrus/record` (lossless HDF5), `/micrus/snapshot` | `P1_results.md` (simulator: 27.35 Hz, 0 gaps in 5 min, SIGINT power-down) | real scanner rate, licence, USB | P1 on the capture PC | sim; hardware blocked (scanner) |
| RGB-D camera | `wrist_camera.launch.py` (realsense2_camera 4.58.4, aligned depth, locked exposure via `camera_autolock`) | launch args checked with `--show-args`; D415 enumerated on the dev PC at USB 2.1 only | streams, stamps, metadata keys, depth at contact distance | P1b on the capture PC (USB 3) | built; hardware blocked (USB 3) |
| Episode recorder | `episode_manager.py` + `recorder.py` (one MCAP per episode, readiness gate, micrus HDF5, `record_guard`, crash recovery), `episode_qc.py`, `presession_check.py` | `P4_results.md` (5 sim episodes incl. SIGKILL), `test_episode_qc.py`, `test_common.py::test_presession_result_required_for_collection` | run with the camera and robot_control on real hardware | checklist F, then a Stage 0 session | sim; hardware blocked |
| Dataset exporter | `stage_bags.py` → micrus `bag_to_lerobot.py` (v2.1) → `openh_export.py` (Open-H fields, incremental TCP action, 16-bit depth, calibration, splits) → LeRobot 0.6.0 v2.1→v3.0 converter | `export/pipeline_check.py`: synthetic episodes end to end, reloaded with `LeRobotDataset`, all checks pass (3 episodes incl. freedrive); `test_export_fields.py` | real episodes; Open-H `validate_formatting.py` (not in this repo) | `pipeline_check.py run`, then real Stage 0 episodes | unit, sim (synthetic); real data blocked |
| Freedrive (kinesthetic) collection | pedal on a UR configurable input ("Freedrive"); `robot_state_cpp` / `labels.py` label samples FREEDRIVE / kinesthetic / expert from `actual_digital_input_bits` + `robot_mode`; `episode_manager collection_mode:=freedrive`; QC `freedrive_engaged`, `freedrive_speed`; export `command_valid` | `test_labels.py` (3), freedrive tests in `test_episode_qc.py` (3), synthetic freedrive episode in `pipeline_check.py`; `robot_state_cpp` read-only on URSim | pedal, gravity compensation and labels on the UR7e | `FREEDRIVE_COLLECTION_GUIDE.md` section 7 | unit, mocked, sim (synthetic); hardware blocked |
| Mode packages and real-robot checks (14 Oct) | `usrig_teleop` (teleop_check), `usrig_presets` (waypoints, paths, plans moved from usrig_episode; preset_check), `usrig_freedrive` (labels.py moved from usrig_robot; freedrive_check), `usrig_robot pose_monitor`; `REAL_ROBOT_CHECKS.md` | `test_mode_checks.py` (8: the checks over real ROS services against robot_control on the fake robot), `test_freedrive_check.py` (4), `test_motion_check`, `test_pose_monitor_render` | running them on the UR7e | `REAL_ROBOT_CHECKS.md` | mocked; hardware blocked |
| Operator interface | `status_console` (headless, used for collection); `dashboard.py` (PySide6, demos) | `test_stream_state.py`; offscreen start, 8 s, no errors | real streams | collection day | unit, built |

## Defects found and fixed in this audit
| # | Defect | Why it mattered | Fix | Test | Commit |
|---|---|---|---|---|---|
| 1 | `scripts/test.sh` imported the **installed copies** of the packages; after a non-symlink build, edits were not tested | Green tests on stale code | `PYTHONPATH` puts `src/` first | the target-validation test failed as expected once fixed | 658098d |
| 2 | Move / path targets were not checked before motion: a target outside the box or cone ran until the filter clipped it and the move timed out | Motion started toward a pose the safety envelope forbids | `safety.check_targets` + `interpolate_poses`; `move_to_pose` legs and `run_path` sampled and refused before moving (recovery moves may start outside) | `test_target_outside_envelope_refused_before_moving`, `test_run_path_outside_envelope_refused_before_moving`, `test_check_targets_and_recovery_tolerance` | 658098d |
| 3 | Episodes could run **unverified** waypoints, and nothing invalidated a waypoint after a TCP / probe / payload change | Silent reuse of a pose taught with another tool | `taught_config` / `verified_config` fingerprint; `episode_manager` refuses unverified or stale waypoints | `test_waypoint_validity.py` (4) | 658098d |
| 4 | `openh-export` env had no `scipy`: `openh_export.py` could not start | Export had never run | installed; `environment_export.yml` | `pipeline_check.py` | 7feb1c8 |
| 5 | `force_setpoint_n` written as a 1-element list: **LeRobot could not load the dataset** | Every real export would have failed at load time | written as a scalar (LeRobot's schema for shape [1]) | `pipeline_check.py` | 7feb1c8 |
| 6 | LeRobot's v2.1→v3.0 converter **drops** `meta/calibration/`, `annotations.jsonl`, `README.md`, `modality.json` and custom `info.json` keys | Released data would have lost its calibration and expert labels | `carry_meta` copies them back and checks they arrived | `pipeline_check.py` (5 meta checks) | 7feb1c8 |
| 7 | `presession_check` printed PASS / FAIL but nothing enforced it | Collection could start with a failed readiness check | result stored in `session.json`; `episode_manager` refuses real sessions without a fresh PASS for that stage (and `--control` when commanding) | `test_presession_result_required_for_collection` | a2dee60 |
| 8 | Dashboard showed the last image, robot mode and rates after a stream stopped; motion-starting buttons were one click | A frozen "TELEOP" or image looks live | live / STALE / NO DATA per stream, rates drop to 0, robot mode red when stale; confirmation on Start/stop and Next | `test_stream_state.py` | a2dee60 |
| 9 | Unused imports (pyflakes) | hygiene | removed | pyflakes clean | 658098d |

Teleop acceptance tests added (c51d69a): stale robot state → FAULT and acknowledge does not resume teleop;
an over-limit request is clamped and the executed command differs from the requested one; keyboard twists
are ignored while a path runs and `set_teleop` is an explicit takeover; motion services are refused in FAULT.

## Findings not changed (documented)
- `rosbags` (used by `bag_to_lerobot` and QC) rejects a `sensor_msgs/JointState` whose `velocity` / `effort`
  arrays are empty (CDR size mismatch). `robot_state_cpp` fills all three, which is why the P4 bags convert;
  any other `/joint_states` publisher must do the same (`NOT_VERIFIED.md`).
- LeRobot 0.6.0's `LeRobotDataset` has native depth options (`depth_encoder`, `depth_output_unit`). The current
  16-bit PNG depth loads and round-trips losslessly (pipeline check); switching is open decision 17.
- `torchcodec` in `openh-export` cannot load (no FFmpeg shared libraries); the check uses `video_backend='pyav'`.
- The keymap has no probe-z key: in free air, motion along the probe axis comes only from the services
  (descend, retract, move). In contact, z is force-regulated (setpoint keys T/G). By design (D5).
- Single motion authority: `robot_control` is the only node with an RTDE control connection; teleop,
  waypoints, paths and perturbations are `Motion` objects in one slot. `set_teleop` refuses with more than
  one `/teleop/twist` publisher. The dashboard sends only episode commands, protective stop and zero F/T.
- Units: `rig.yaml` is in mm, deg, N; `safety.Limits.from_rig` converts to m, rad (unit tests). Every
  `/teleop/twist` and `/robot/cmd_executed` is in the probe frame (`frame_id: probe`), m/s and rad/s.
  Robot state and the base-frame twist sent to `speedL` are in the UR base frame.

## Readiness decision
**Software tests pass; simulation remains incomplete.** Build from a fresh clone (11 packages) and 144 tests pass; the
export chain is verified on synthetic data; recording and crash recovery were verified with the micrus
simulator and URSim state (P4). Teleoperation and path execution are verified only against the kinematic
mock, because the URSim control run was skipped by decision. The next level, a controlled hardware trial,
is **not yet eligible**: `robot_control` refuses to start while any of its 28 required `rig.yaml` values is null (27 today) (by
design), and the capture PC is not set up. Details and evidence: `TEST_AND_VERIFICATION.md`.
