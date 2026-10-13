# P4 results: episode recorder end to end, in simulation (13 October 2026)

Setup: dev PC; micrus `--simulate`, robot_state_cpp on URSim (UR5e model, 0.96x real time),
qc_monitor, episode_manager Stage 0 with robot_control off (limits are null, nothing moves),
wrist camera disabled (no USB 3 here). Operator input on /operator/command, as keyboard_teleop sends it.
5 episodes; episode 3: episode_manager killed with SIGKILL while recording; then restarted.

## Workflow (final run, session p4_sim4)
| Episode | Result |
|---|---|
| 1, 2, 4 | outcome success, view_reached recorded, bag + micrus HDF5 + qc.json + annotation.json written |
| 5 | outcome fail, same files |
| 3 (SIGKILL) | record_guard stopped the micrus recording ~2 s after the kill (HDF5 saved); the bag closed cleanly (metadata written, 15,608 messages in run 1); on restart the episode was marked `discard` / `interrupted` with its HDF5 path; numbering continued at 4 |
Also observed: a forgotten recording is ended by `stage0.max_duration_s`; commands sent while the
manager is starting (~7 s) or idle are ignored.

## Data (final run)
| Episode | US frames in bag | US gaps | micrus fps | robot samples lost | micrus HDF5 frames |
|---|---|---|---|---|---|
| 1 | 368 | 0 | 27.41 | 0 | 372 |
| 2 | 359 | 0 | 27.41 | 0 | 308* |
| 4 | 282 | 6 | 26.69 | 0 | 289 (index span in bag 288) |
| 5 | 367 | 2 | 27.35 | 0 | 376 (index span 369) |
*HDF5 of episode 2 vs 3 ordering per micrus log; every HDF5 covers its whole recording.

The micrus HDF5 keeps every frame, including those missing from the bag. Remaining QC failures are
expected here: robot_rate (URSim clock 0.96x), coupling / force_band (thresholds null), free_disk (< 50 GB).

## Fixed during P4
- Crash record: the in-progress episode.json is rewritten once the HDF5 path is known, so an
  interrupted episode keeps it.
- Recorder readiness subscriptions were deserializing every B-mode image and every 500 Hz robot
  sample inside episode_manager for the whole episode; now raw and removed once the bag is ready.
  Ultrasound gaps per episode went from 2-18 to 0-6.

## Not covered by P4 in simulation
Stage 2 plan resume (needs robot_control, i.e. the limits), the wrist camera in the bag (P1b on the
capture PC), real scanner and UR7e timing.
