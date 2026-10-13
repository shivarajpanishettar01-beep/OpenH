# Control and system-design review: teleoperation and waypoint presets (13 October 2026)

Scope: `src/usrig_robot/usrig_robot/{robot_control,safety,backends}.py`, `src/usrig_teleop`,
`src/usrig_episode/usrig_episode/{episode_manager,waypoints}.py`, read in full this session.
Purpose: state what the current design assumes before the teleoperation specification arrives,
so the real-arm work is not built blindly. Evidence labels: [Measured] (run this session),
[Derived] (from the code), [Hypothesis], [Literature].

## 1. What the design assumes today

| Area | Assumption in the code | Where |
|---|---|---|
| Control law | Hybrid force/motion: force regulated along the probe axis by UR `forceMode` (type 2), the other 5 DoF commanded as a velocity with `speedL` every 8 ms (125 Hz) | `backends.py:117-129`, `robot_control.py:458` |
| Teleop command | Probe-frame twist from the keyboard at 100 Hz, ramped over `ramp_s`; probe z velocity is zeroed in contact, the operator nudges the force setpoint instead | `robot_control.py:83-93`, `keyboard_teleop.py:102,176-187` |
| Rotation pivot | Angular velocity is about the active TCP; rock / tilt pivot about the probe face only if the TCP is the probe face | `robot_control.py:93`, `geometry.twist_probe_to_base` |
| Safety | Measured-state limits (box, tilt cone, \|F\|, speed) -> FAULT; command filter (speed, accel, box, tilt) clips; deadman on twist arrival; controller watchdog | `safety.py:76-136`, `robot_control.py:426-458`, `backends.py:99-115` |
| Preset waypoint | Waypoint (phantom frame) + planned offset -> approach pose 50 mm back along the probe axis -> straight-line move -> zero F/T -> descend to the contact threshold -> hold -> teleop | `episode_manager.py:480-487,551-563` |
| Transit | Straight Cartesian line from the current pose, position and orientation each P-controlled and clamped separately | `robot_control.py:96-114` |
| Stage 1 path | Constant speed along waypoints, probe axis force-regulated, tangential P correction | `robot_control.py:142-177` |
| Perturbation | translate / rotate in the probe frame, or `lift` = force-setpoint change | `robot_control.py:180-206` |

## 2. Findings

| ID | Sev. | path:line | Issue | Consequence | Fix | Test |
|---|---|---|---|---|---|---|
| R1 | critical | `backends.py:117-129` | `forceMode` + `speedL` each cycle, force sign and behaviour in contact never run. **URSim cannot validate it: it does not simulate contact forces.** | Unknown contact behaviour on the first real contact | Staged contact tests on the real arm at low force (section 4, step 4); force-step and slide tests logged | Real-arm Gate B tests; cannot be a CI test |
| R2 | critical | `robot_control.py:83-93,458` | No contact-loss detection. In force mode with no reaction force the controller keeps moving along the probe axis at the force-mode speed limit until something stops it (box, phantom, FAULT) | Probe can drop into the phantom edge / table after slipping off; FM-02 lift-off staging is unbounded | Detect `|F_axis| < threshold` for N ms while in contact -> event `contact_lost` -> hold position (stop force mode) or slow bounded re-approach; your choice (D8) | `test_contact_loss_is_detected` (xfail today: **12 mm travel in 1.5 s, no event** [Measured, simulator]) |
| R3 | important | `robot_control.py:204-205` | `lift` perturbation sets the force setpoint without clamping | Setpoint went to **-6.0 N** [Measured, simulator]: negative force = pulling away; outside the band that FM-03 must never leave | Clamp to `[0, band_high]`; make "lift-off" an explicit, bounded retract instead of a negative force | `test_lift_perturbation_stays_in_force_band` (xfail today) |
| R4 | important | `safety.py:84`, `robot_control.py:485-495` | Force ceiling uses \|F\| from the controller after one `zero_ft` in free air; no check that the controller's **payload / CoG** matches the mounted probe + camera | If payload is wrong, the apparent force changes with tilt [Hypothesis, standard UR behaviour]: false FAULTs or a masked over-force | Add `payload_kg`, `payload_cog_m` to rig.yaml; refuse to start on mismatch (RTDE `getPayload`, `getPayloadCog` are readable); measure free-air force vs tilt (`rig_measure force-noise` at several tilts) | Unit test on the check; real-arm free-air tilt sweep |
| R5 | important | `robot_control.py:96-114`, `episode_manager.py:481,510` | Transit between episodes = retract along the probe axis, then a straight line to the next approach pose; no via-point at a safe height | Moving across the phantom between waypoints can clip it if the retract height is smaller than the phantom's relief [Derived] | Transit via a safe-height plane in the phantom frame: up, across, down | Kinematic-sim test: path from waypoint A to B never goes below the safe plane |
| R6 | important | `robot_control.py:83-87`, `keyboard_teleop.py:102,176-187` | The keyboard publishes at 100 Hz even with no key held, so the deadman only catches a dead process or link, not an absent operator. There is no hold-to-move enable | A key typed into any window (evdev ignores focus) or a dropped object on the keyboard moves the robot while teleop is on | Hold-to-move enable (a held key, foot pedal or SpaceMouse button), decided in the teleop spec (D2) | Sim test: twist with enable released -> zero motion |
| R7 | important | `robot_control.py:96-114` | Transit position and orientation are P-controlled and clamped independently | They finish at different times; with large rotations the probe sweeps sideways; not a straight, coordinated move [Derived] | Synchronised trapezoidal profile (both finish together) | Sim test: orientation error proportional to position error along the move |
| R8 | minor | `backends.py:129`, `robot_control.py:420-424` | `speedL(..., time=dt)` with dt = 8 ms: a late tick (Python, no real-time priority) lets the command expire and the arm decelerates until the next one | Jerky motion under load; uneven actions in the dataset [Hypothesis, not measured] | Measure tick jitter on the capture PC; if needed `time = 2-3 dt` (the watchdog still stops a dead loop) | Log tick intervals; `cmd_executed` jerk per episode |
| R9 | minor | `robot_control.py:300-301` | `hold` is toggled by the operator (P) and set by services | Operator and episode manager can disagree on hold [Derived] | Separate operator pause from service hold | Sim test |
| R10 | minor | `safety.py:80-83` | Workspace box is axis-aligned in the base frame; tilt cone is relative to a fixed base axis | Moving the phantom or scanning a curved surface (neck phantom) needs re-measured limits; the cone does not follow the surface normal | Box and tilt reference in the phantom frame after registration (D9, D10) | Unit tests on the frame change |

Not found as problems: deadman on link loss, controller watchdog, FAULT -> acknowledge -> inward-only
recovery, timeouts, cancellation, motion events, probe-frame direction convention (all tested in the
kinematic simulator, `src/usrig_robot/test/test_control_sim.py`, 6 passing [Measured]).

## 3. Decisions for the teleoperation specification (recommended default in bold)

| # | Decision | Options | Why it matters |
|---|---|---|---|
| D1 | Input device | **SpaceMouse (proportional 6-DoF) for collection**, keyboard as fallback / Stage 0 | Keyboard actions are discrete ramps: near-identical, bang-bang demonstrations are a known weakness for imitation learning |
| D2 | Enable / deadman | **Hold-to-move enable** (SpaceMouse button or foot pedal) | Closes R6 |
| D3 | Command frame | **Probe frame** (current) or base frame | Matches how sonographers think (slide, rock, tilt) |
| D4 | Rotation pivot | **Probe face (TCP = probe face)** | Rock / tilt about the contact point; otherwise the probe slides across the skin |
| D5 | Probe-axis control | **Force-regulated with setpoint nudges** (current) or operator position control | Defines compress / decompress and the force band |
| D6 | Per-axis speeds, fine mode | your values, below rig.yaml caps | Demonstration quality and safety |
| D7 | Preset waypoint role | **Automatic approach + descend, operator takes over in contact** (current) or operator does the approach | Stage 2 relies on automatic setup |
| D8 | Contact-loss behaviour | **Hold position, event `contact_lost`, operator re-establishes** or bounded slow re-approach | Closes R2 |
| D9 | Workspace box frame | **Phantom frame after registration** or base frame | Phantom placement changes between sessions |
| D10 | Tilt reference | **Waypoint's probe axis (local surface normal)** or fixed vertical | Curved phantoms (central-line neck) |
| D11 | Transit between waypoints | **Via a safe-height plane** | Closes R5 |

## 4. What to verify before capturing training data (order)
1. URSim dry run with your limits: kinematics, envelope, deadman, watchdog, cancel / timeout (URSim cannot test force).
2. Real arm, free air (`first_motion_checklist.md` C-D) plus **payload check and free-air force vs tilt** (R4).
3. Measure control-loop tick jitter and teleop latency (key / button -> TCP motion) on the capture PC (R8).
4. **Contact tests on the phantom at low force** (R1): force step response (setpoint T/G), force tracking during a slow slide, rock and tilt about the probe face, contact loss at the phantom edge (after R2 is fixed), phantom stiffness.
5. One Stage 0 episode end to end through export (actions, timestamps, frames, depth).
