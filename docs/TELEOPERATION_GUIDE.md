# Teleoperation guide

Keyboard (or SpaceMouse) teleoperation of the probe on the UR7e. Written from the code; **not yet run on
the real arm** (see `first_motion_checklist.md` sections C-D for the first run). Every terminal starts with
`cd ~/OpenH && source scripts/env.sh`.

## How the command path works
```
keyboard (evdev) -> keyboard_teleop -> /teleop/twist (probe frame, 100 Hz)      requested
                                        |
                              robot_control (the only node that commands the UR7e)
                                safety filter: speed, accel, box(es), tilt cones
                                        |
             speedL / forceMode (ur_rtde, 125 Hz) -> UR7e   and   /robot/cmd_executed    executed
/robot/state (500 Hz, measured) carries mode, safety flags, action source, expert flag
```
Requested (`/teleop/twist`), executed (`/robot/cmd_executed`, after the filter) and measured
(`/robot/state`, `/robot/tcp_pose`) are separate topics and are all recorded. Raw key events go to
`/operator/command` for the audit trail; they are never the action.

## Setup (once)
1. `rig.yaml`: every limit filled (`rig_limits.md`). `robot_control` and `keyboard_teleop` refuse to start otherwise.
2. `keymap.yaml`: `speeds.*` and `force_step_n` (your values, below the `rig.yaml` caps; checked at start).
   Both with `ros2 run usrig_robot rig_setup` (sections `speeds`, `force`, `motion`, `keyboard`).
3. `keymap.yaml device:` = the `/dev/input/by-id/...-event-kbd` path of a **dedicated operator keyboard**.
   evdev reads the device regardless of window focus: with `null` the first keyboard found is used, and keys
   typed into any window would reach the robot.
4. User in the `input` group (evdev access).
5. Controller in **Remote Control**, TCP = probe face (`probe.tcp_already_includes_probe: true`), payload set.

## Start
```bash
ros2 launch usrig_bringup robot.launch.py control:=true
```
```bash
ros2 run usrig_teleop keyboard_teleop
```
```bash
ros2 run usrig_qc status_console
```
Enable (the robot holds still until enabled; episode_manager does this itself in Stages 0 and 2):
```bash
ros2 service call /robot/set_teleop std_srvs/srv/SetBool "{data: true}"
```
Disable (robot holds position, teleop input ignored):
```bash
ros2 service call /robot/set_teleop std_srvs/srv/SetBool "{data: false}"
```

## Keys (`keymap.yaml`)
Probe frame: x along the array, y across it, z along the beam into tissue.

| Key | Action |
|---|---|
| **Ctrl (held)** | enable: motion keys act only while Ctrl is held (D2 hold-to-move) |
| W / S | slide along the array (+x / -x) |
| A / D | slide across the array (+y / -y) |
| Q / E | rock: in-plane rotation about probe y |
| R / F | tilt: out-of-plane rotation about probe x |
| Z / C | rotate about the beam axis (probe z) |
| Shift (held) | fine mode (`fine_scale`, default 0.25) |
| T / G | force setpoint up / down by `force_step_n` (Shift: a quarter) within `force_band_n` |
| P | operator pause / resume (independent of the episode flow's hold) |
| K | acknowledge a FAULT |
| J | recontact after `contact_lost` (slow bounded descent) |
| B | Stage 0: start / stop an episode |
| Space | view reached |
| 1 / 2 / 3 | outcome success / recovered / fail |
| Esc | discard the episode |
| N | next plan row |
| O / U / I | failure onset / recovery start / recovery end (g_t labels) |

There is no probe-z key. In free air, motion along the probe axis comes from the services (descend,
retract, move). In contact the probe axis is force-regulated: T / G change the force, not the position.
Several motion keys at once combine (diagonal). Key auto-repeat is ignored. Releasing all keys ramps to
zero over `ramp_s` (0.25 s).

## Stop behaviour
| Situation | What happens |
|---|---|
| keys released | ramp to zero; the robot holds position (in contact: holds force) |
| Ctrl released | same as keys released |
| keyboard unplugged / read error | all keys released, one zero twist, publishing stops; robot_control's deadman stops the robot; keyboard_teleop reconnects and reads which keys are physically held: no motion unless Ctrl and a motion key are held |
| keyboard_teleop dies | no twist within `deadman_timeout_s`: zero velocity, flag `deadman` |
| robot_control dies or stalls | the controller watchdog (ur_rtde, min rate = 1 / deadman timeout) stops the UR7e |
| robot state older than `max_state_age_s` while moving | FAULT |
| measured state outside box / cone, force above ceiling, speed above limit x margin | FAULT |
| command would leave box / cone, or exceeds speed / accel | clipped (flag set), no FAULT |
| a path or approach is running | teleop twists are ignored; `set_teleop true` is an explicit takeover (the path ends `preempted`) |

After a FAULT nothing moves until **K**. The acknowledgement is refused while force or speed is still
violated. Outside the box or cone it is accepted, but only motion back inside is allowed. The previous
motion is **never** restarted: teleop must be enabled again.

## Status indicators
`status_console` (refreshed once per second; a status not updated for 0.5 s (control) or 2 s (QC) is marked NO UPDATE): robot mode (IDLE MOVING DESCENT TELEOP PERTURB RETRACT FAULT),
safety flags (envelope, tilt, force_max, deadman, speed), action source and expert flag, force setpoint,
episode / recording state, QC summary. A flag shown during teleop means your command was **limited**:
- `speed` / accel: you asked for more than the cap; the executed command is clamped;
- `envelope` / `tilt`: that direction is blocked at the limit, others still move;
- `deadman`: no input is arriving.
The dashboard (demos) shows the same, and marks every stream live / STALE / NO DATA.

## SpaceMouse (optional, D1)
```bash
ros2 launch usrig_bringup collect.launch.py session_id:=<id> control:=true teleop:=true input:=spacemouse
```
Left button held = enable, right button = fine; the cap moves the probe proportionally; pushing / pulling
the cap nudges the force setpoint. The keyboard still sends commands but no twist (`set_teleop` refuses if
two nodes publish `/teleop/twist`). Axis indices and signs (`keymap.yaml spacemouse.axes`) are **NOT
VERIFIED**: check each axis in free air.

## Troubleshooting
| Symptom | Cause / fix |
|---|---|
| `keyboard_teleop` exits: `keymap.yaml speeds ... null` | fill `speeds.*`, below `rig.yaml` caps |
| `no keyboard found in /dev/input` | user not in the `input` group, or `device:` path wrong |
| keys do nothing | Ctrl not held; teleop not enabled (`set_teleop`); operator pause (P); FAULT (K); a path is running |
| `set_teleop` refused: `N nodes publish /teleop/twist` | run one input device only |
| motion stops at a face, flag `envelope` | box limit; move along another axis or back inside |
| tilt stops, flag `tilt` | global or local tilt cone; rotation about the beam (Z / C) still works |
| `robot_control` refuses at start | it names the reason: null limits, Remote Control off, TCP or payload mismatch, TCP not at the probe face |
| FAULT `robot state stale` | robot_state_cpp stopped or the network dropped; check the launch and cable, then K |
| robot slows in jerks | late control ticks (log `control ticks: {...}`); see `robot.speedl_time_ticks` (R8) |
