# Test and verification

What was run, how, and what it showed (13 October 2026, development PC, Ubuntu 22.04, ROS 2 Humble,
conda env `openh` Python 3.10; export env `openh-export` Python 3.12 + LeRobot 0.6.0). **No test in this
file ran on the UR7e, the real micrus scanner or the D415.** Earlier phase results: `P1_results.md`,
`P2_results.md`, `P4_results.md`, `data_integrity.md`.

## Reproduce
```bash
cd ~/OpenH && bash scripts/build.sh                 # colcon, symlink install, openh Python
```
```bash
bash scripts/test.sh                                # unit + mocked integration, tests the src/ tree
```
```bash
source scripts/env.sh && python export/pipeline_check.py run     # synthetic end-to-end export
```
```bash
/usr/bin/python3 -m pyflakes src export tools       # static check
```
Clean-room build as done for this report (fresh clone, empty environment):
```bash
git clone ~/OpenH /tmp/clean && cd /tmp/clean && env -i HOME=$HOME USER=$USER PATH=$HOME/miniconda3/bin:/usr/bin:/bin bash scripts/build.sh
```

## Build and static checks
| Command | Result | Evidence / note |
|---|---|---|
| fresh clone + `scripts/build.sh` in an empty environment | exit 0, 9 packages, 30 s | warnings only: CMake "cannot generate a safe runtime search path" (conda vs system libraries, harmless) |
| same clone + `scripts/test.sh` | 83 passed (before this audit's additions) | |
| `scripts/test.sh` (final) | **126 passed**, ~95 s | 21 files; fresh clone of 14 Oct: 11 packages build, all pass |
| `pyflakes src export tools` | clean (after removing 6 unused imports) | |
| `ros2 launch usrig_bringup {robot,micrus,wrist_camera,collect}.launch.py --show-args` | all parse; arguments listed | |
| `dashboard` offscreen (`QT_QPA_PLATFORM=offscreen`, no topics) | ran 8 s, no exception | |

Defect in the baseline: `scripts/test.sh` imported the installed copies; after a non-symlink build it
tested stale code. Fixed: it now puts `src/` first on `PYTHONPATH` (`IMPLEMENTATION_AUDIT.md` #1).

## Test inventory by level
| Level | Files (tests) | What they cover |
|---|---|---|
| Unit | `test_common` (6), `test_safety` (10), `test_annotation` (6), `test_calibration` (4), `test_plan` (4), `test_waypoint_validity` (4), `test_rig_measure` (2), `test_ramp` (4), `test_export_fields` (7), `test_stream_state` (4), `test_sim_separation` (8) | config parsing and null-limit refusal, units, transforms, safety envelope and target pre-check, annotation / g_t labels, calibration solvers, plan balance, waypoint executability, key ramp and hold-to-move, export action maths, stale display, sim / synthetic separation, pre-session gate |
| Mocked integration | `test_control_sim` (25: kinematic fake robot + fake F/T), `test_backend` (7: fake `rtde_control` / `rtde_receive`), `test_keyboard_failsafe` (2: fake evdev), `test_spacemouse` (2: fake Joy), `test_episode_qc` (5: real MCAP bags written by the test), `test_calibration_tools` (4: rendered board images) | control authority, FAULT / acknowledge, deadman, stale state, timeouts, cancellation, contact loss, path / move pre-validation, TCP / payload / watchdog setup, keyboard loss, QC on bag stamps |
| Simulation | `P1` micrus `--simulate`; `P2` robot_state on URSim (read-only); `P4` recorder + manager with micrus sim + URSim state, 5 episodes incl. SIGKILL; `pipeline_check.py` synthetic episodes through the real export chain | |
| Real hardware | none | blocked: limits null, capture PC not set up |

## Safety-critical behaviour: required tests and their level
| Requirement | Test | Level | Result |
|---|---|---|---|
| refuses to start with any limit null | `test_require_limits_names_every_null`, `test_null_limit_refuses` | unit | pass |
| refuses TCP / payload mismatch | `test_tcp_mismatch_refuses`, `test_payload_mismatch_refuses` | mocked | pass |
| TCP must be the probe face | `test_design_requires_tcp_at_probe_face` | unit | pass |
| valid teleop command accepted, probe-frame directions | `test_teleop_probe_frame_directions_and_deadman` | mocked | pass |
| over-limit command clamped; executed ≠ requested | `test_over_limit_request_is_clamped_and_executed_differs_from_requested` | mocked | pass |
| key release → zero; hold-to-move | `test_ramps_up_in_ramp_time_and_back_to_zero`, `test_enable_key_required` | unit | pass |
| keyboard lost → publishing stops → deadman | `test_device_loss_stops_twists_and_reconnects`, deadman part of the teleop test | mocked | pass |
| robot state stale → FAULT; ack does not resume | `test_stale_robot_state_faults_teleop_and_ack_does_not_resume` | mocked | pass |
| controller watchdog set and kicked | `test_watchdog_set_from_deadman_and_kicked` | mocked | pass (the real stop: checklist D) |
| hard envelope → FAULT, ack, inward-only recovery | `test_hard_envelope_fault_ack_and_recover_inward_only` | mocked | pass |
| force ceiling → FAULT, ack refused while violated | `test_force_ceiling_faults` | mocked | pass |
| motion refused while faulted | `test_motion_services_refused_while_faulted` | mocked | pass |
| move / path outside limits refused before moving | `test_target_outside_envelope_refused_before_moving`, `test_run_path_outside_envelope_refused_before_moving` | mocked | pass |
| stalled move times out instead of hanging | `test_stalled_move_times_out_instead_of_hanging` | mocked | pass |
| teleop cannot conflict with a path | `test_teleop_twists_ignored_while_a_path_runs_and_set_teleop_takes_over`, `test_two_twist_publishers_refuse_teleop` | mocked | pass |
| path cancel, retract wins | `test_descend_then_path_then_cancel` | mocked | pass |
| contact loss detected, path aborted | `test_contact_loss_is_detected`, `test_path_aborts_on_contact_loss` | mocked | pass |
| unverified / stale waypoint refused | `test_waypoint_validity.py` | unit | pass |
| phantom-frame moves via safe height, phantom box, local cone | `test_phantom_*` | mocked | pass |
| collection refused without a passing pre-session check | `test_presession_result_required_for_collection` | unit | pass |
| simulated / synthetic data never exported with real data | `test_stage_bags_refuses_sim_sessions`, `test_synthetic_pipeline_check_episodes_never_mix` | unit | pass |

## Export end to end (synthetic, `export/pipeline_check.py`)
Two synthetic episodes (success, recovery-mid), 3.84 s each, every recorded topic with known values:
TCP trajectory, executed twist, force setpoint, expert flag, ultrasound frames whose pixel value encodes
the frame index (27.3 Hz), 16-bit depth whose value encodes the frame index (30 Hz), failure / recovery
intervals. Chain: `stage_bags --pipeline-check` → `bag_to_lerobot` (v2.1, 25 fps) → `openh_export
--pipeline-check` → LeRobot 0.6.0 v2.1→v3.0 converter → `LeRobotDataset(..., video_backend='pyav')`.

| Check | Result |
|---|---|
| codebase v3.0, 2 episodes, 192 frames, splits `train 0:1`, `recovery 1:2` | pass |
| incremental TCP action vs the action recomputed from the ground-truth trajectory | max error 4.2e-8 |
| executed twist (`observation.meta.commanded_twist`) | max error 1.1e-9 |
| ultrasound frame selection (sample-and-hold, -20 ms offset) and pixels after H.264 | max 1 grey level |
| ultrasound host stamps, depth (lossless, exact), force setpoint, expert flag, g_t status per step | 0 mismatches |
| meta: calibration files, annotations, README, synthetic marker survive the v3.0 conversion | pass |
| `delta_timestamps` query (action history) | pass |
| sensitivity: wrong ultrasound offset in the truth file | 108 failures reported (the check is not vacuous) |

Found and fixed by this check (none had run before): missing `scipy` in the export env; a
1-element-list column that LeRobot could not load; the converter dropping calibration, annotations,
README, modality and custom keys.

Not covered: real episodes, Open-H `validate_formatting.py` (not in this repo), GR00T `modality.json`
against a GR00T release, `torchcodec` decoding (FFmpeg shared libraries missing in `openh-export`; pyav used).

## Freedrive (kinesthetic) collection
| Check | Level | Result |
|---|---|---|
| labelling rule (fresh control status wins, stale never carried forward, pedal bit + RUNNING) | unit `test_labels.py` | pass |
| QC: pedal used, hand speed vs limit, force check n/a for a probe grip | mocked `test_episode_qc.py` (bags written by the test) | pass |
| export: synthetic freedrive episode in `pipeline_check.py`: train split, `command_valid` false, action from measured motion, expert true, finite action stats, dataset loads | sim (synthetic) | pass (37 checks, 3 episodes) |
| `robot_state_cpp` with the new RTDE outputs against URSim (read-only, 10 s) | sim | 475 Hz, IDLE / none / NaN setpoint, NaN marker 120 Hz, 0 samples lost |
Found by the export check: LeRobot's v3.0 conversion turns NaN into null and the loader then fails, so
"no command" is a boolean `command_valid` in the dataset (NaN stays in the bags).

## URSim
URSim e-series 5.26.1 (UR5e model) runs in Docker on the dev PC. During this audit: Remote Control enabled
from its pendant (browser VNC), powered on, `payload` / `payload_cog` read through a restricted
RTDEReceiveInterface (the R4 payload check's call works on PolyScope 5.26), powered off again. The
control dry run in URSim was **skipped by decision** (user, 13 Oct 2026); its items move to the real arm
in free air (`first_motion_checklist.md` C-D).

## Not verified (summary; full list `NOT_VERIFIED.md`)
Real UR7e: state rate, `forceMode` sign and contact behaviour, watchdog stop time, tick jitter,
payload / tilt sweep, key and SpaceMouse directions. micrus real scanner. D415 streams, stamps, metadata,
exposure lock. Recording with all three together on the capture PC. Export on real episodes.

## Readiness decision
**Software tests pass; simulation remains incomplete.** See `IMPLEMENTATION_AUDIT.md`. A controlled
hardware trial becomes eligible when: the 22 limits and keymap speeds are filled, the capture PC is set up
(USB 3, rtprio, data disk, micrus licence), `presession_check` passes, and the operator starts
`first_motion_checklist.md` section B.
