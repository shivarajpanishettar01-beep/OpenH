# Operator console guide

The operator console is the window used during data collection. It shows the session and the current
episode (number, date, start time, elapsed and paused time, type, target, instruction, plan row), states
in one line what the rig is waiting for, and gives buttons for every collection command: start and stop of
an episode, pause and resume, outcomes, failure and recovery marks, notes, teleoperation and safety
controls, preset waypoints and paths, freedrive status, and session set-up. The demo dashboard
(`dashboard:=true`, with ultrasound and camera images) is unchanged and remains for demonstrations.
Written from the code and checked offscreen with scripted states; **not yet run on the rig**.

## Starting it

With a collection already running:
```bash
ros2 run usrig_dashboard operator_console
```
Or together with a collection: `ros2 launch usrig_bringup collect.launch.py ... console:=true`.
The console can also start the collection itself from its Session tab (below). It runs at a lower CPU
priority (`nice 10`) and refreshes at 4 Hz.

## Design rules, and why they matter

The console was built so that it can stay open while data are recorded, which the image dashboard
could not (it took more than one core and, before the C++ robot reader, cost a quarter of the robot
samples; `data_integrity.md`). Four rules follow from the rig's safety and data-integrity requirements.

**It is never the motion input.** Probe motion stays on the dedicated keyboard or the SpaceMouse with
hold-to-move. The console's buttons send exactly the commands the keyboard sends (`/operator/command`)
or call `robot_control` services, so every limit, the deadman and the FAULT logic apply unchanged. Every
button that can move the arm (start in a robot stage, Next row, go to approach, descend, retract, run a
path, verify a waypoint, recontact) asks for confirmation first.

**It is operated with the mouse only.** `keyboard_teleop` reads the operator keyboard through evdev,
whatever window has focus. If a console button had keyboard focus, pressing Space on the operator
keyboard would both mark "view reached" through evdev and press the focused button. Buttons therefore
never take focus, there are no keyboard shortcuts, and text fields take focus only when clicked. Type
notes and waypoint fields on the PC's own keyboard, never on the operator keyboard.

**It shows no images and decodes little.** The 500 Hz robot stream is read raw and only one message in
25 is decoded (20 Hz). Ultrasound is viewed on the scanner screen.

**It is not the parent of capture processes.** A collection started from the Session tab runs in its own
terminal, detached (`setsid`), so closing or crashing the console stops nothing.

## The window

**Header.** Session id (with a SIM badge for simulated sessions), stage, collection mode, operator,
phantom and whether `robot_control` runs; a live date and clock; the recording indicator (red
`● REC · Ep 7 · 0:23`, amber `PAUSED`, blue while starting or finishing, grey when idle); *Acknowledge
fault* and *PROTECTIVE STOP*. The protective stop asks `robot_control` to stop the UR7e; the pendant
emergency stop remains the primary stop. Without `robot_control` (freedrive collection, `control:=false`) the button
is grey: the console cannot stop the arm then, only the pendant can.

**Prompt line.** One sentence saying what the rig expects next, for example "Ready: probe in free air,
then Start episode" or "Recording: when done press Success / Recovered / Fail (or Discard)". It turns red
on a FAULT or when `episode_manager` is not running.

**Episode.** The current episode's number, identifier, date, start time, elapsed and paused time, state,
type and target, instruction, plan row, marks set so far and outcome, then a line on the previous episode
with its QC result.

**Recording.** *Start episode* / *Stop episode* (the B key), *Pause*, *Resume*, *Success*, *Recovered*,
*Fail*, *Discard*, *View reached*, *Failure onset*, *Recovery start*, *Recovery end*, *Next row*, a note
field and the gel state. A button is enabled only when `episode_manager` would accept the command at that
moment; hovering over a disabled button shows why (for example "only while recording", or "not used now:
this stage starts episodes itself" for Start in Stage 2).

**Session progress.** Episodes so far by outcome, QC failures, kept recording time (QC-passed, not
discarded, without paused time), and in Stage 2 a progress bar over the plan rows.

**Robot.** Mode, action source and expert label (as recorded), safety flags, force setpoint, measured
probe-axis force against the force band, probe tip in the phantom frame, the teleoperation input (keys
held, or LOST when the keyboard or SpaceMouse dropped out) and which status streams are live.

**Teleop and safety.** *Teleop ON* / *Teleop OFF (hold)* (`/robot/set_teleop`), *Operator hold (P)*,
*Force +* / *Force −* (the keyboard's `force_step_n`), *Recontact*, *Zero F/T (free air)* and *Retract*.

**Console log.** Every command the console sent and the result of every service call.

**Tabs.**
- *Presets*: the phantom registration state; the taught waypoints with their force, verification, NCC and
  whether they may run with the current probe, TCP and payload (`executable_problem`); *Go to approach*,
  *Descend to contact* (at the waypoint's taught force), *Retract*, *Verify selected*; a path list with
  speed and *Run path* (needs contact first); and *Teach a waypoint at the current probe pose* (name,
  organ, view, 3–5 instructions). Teaching and verification use the same functions as `waypoint_teach`
  and `waypoint_verify`. Manual motion is disabled while an episode is in progress, because
  `episode_manager` then owns the motion.
- *Freedrive*: whether the pedal is held (arm free) or released (arm locked), the labels being recorded,
  the share of kinesthetic samples in the current episode, and a warning if `robot_control` is running
  (it must not run during freedrive collection).
- *Session*: collection mode (teleoperation Stage 0, preset paths Stage 1, planned episodes Stage 2,
  freedrive, or set-up only for teaching and verifying), teleop input, operator, phantom, session id
  (*New id* follows `session.yaml id_pattern`), simulation, wrist camera and *Planned hours* (needed by the
  disk row of the check); *Pre-session check*, *Rig status*, *Launch collection (new terminal)* and *Copy
  command*. The full launch command is shown before anything starts. Order: Launch collection, then
  Pre-session check (it measures the running streams), then Start: episode_manager waits for a passing
  check (the prompt says why Start is refused) instead of exiting. In Stage 2 the prompt shows the next plan
  row (type, target, failure mode, staging method) before N starts it.
- *Events*: the episode event stream (phases, motions, pauses, outcomes) with local times.
- *Health*: per-topic rate, stamp age and largest gap from `qc_monitor`, disk and USB state, and which
  nodes are running, including whether the bag recorder is active.

## Start, stop and pause

**Start and stop** are the episode's recording boundaries, owned by `episode_manager`: Start opens a new
bag and micrus recording, waits until every required stream is in the bag, holds in free air for the
F/T window and descends when the stage needs it; Stop ends the bag and micrus, writes `episode.json`,
and runs QC and automatic annotation. In Stages 1 and 2 the episodes start themselves and end with an
outcome; Start / Stop is used in Stage 0 and freedrive.

**Pause does not stop the bag.** Each episode is a bag plus a micrus HDF5 that keeps every ultrasound
frame. A hole in the bag would no longer line up with the HDF5, and the episode QC would fail it on
sample gaps. Pause therefore keeps recording and instead (1) makes the robot hold still (operator hold;
teleop input is ignored and the held samples are labelled source *none*, not expert), (2) writes a
`pause` event into the bag at the start and end of the span, and (3) stores the spans in `episode.json`
as `pauses` (seconds from the start), `pauses_ns` and `paused_s`. A pause still open at Stop is closed
there and the hold released. The time caps (`stage0.max_duration_s`, `stage2.cap_s`) do not count paused
time, so a pause never ends an episode. Pause is not offered in Stage 1: a path runs to a fixed time budget,
so use Discard instead. In freedrive collection there is no robot hold: release the pedal while paused.

**Stop asks for an outcome.** QC fails an episode without an outcome event, so Stop on an episode with no
outcome asks for Success, Recovered or Fail first (or stops without one, or cancels). The B key alone does
not ask: set the outcome with 1 / 2 / 3 before B.

Whether the exporter should drop paused spans or keep them as non-expert steps is an open decision
(`OPEN_DECISIONS.md` item 19); the bag and `episode.json` hold what either choice needs.

## Topics added for the console
- `/episode/status` (`std_msgs/String`, JSON, 2 Hz, latched): session, state (`idle`, `starting`,
  `preparing`, `recording`, `finishing`), the commands `episode_manager` is waiting for, the current
  episode's details, the previous episode, session counts and plan progress
  (`usrig_episode/episode_status.py`).
- `/episode/events` kind `pause` with `{"state": "paused" | "resumed"}`.
- `/operator/command` `pause`, `resume`, `pause_toggle`.

## Not verified yet
CPU use of the console on the capture PC with the scanner, camera and UR7e running (measure with the
`data_integrity.md` runner, as for the dashboard); the terminal emulator on the capture PC
(`gnome-terminal`, `xterm` or `x-terminal-emulator`; without one the command is copied to the clipboard);
and every robot action from the console on the real arm (`first_motion_checklist.md` first).
