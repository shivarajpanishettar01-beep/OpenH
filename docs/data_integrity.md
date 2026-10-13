# Data integrity: measured loss and what to do about it (13 October 2026)

## Where data can be lost
| Stage | Mechanism | Detected by |
|---|---|---|
| Sensor -> driver | robot_state polls RTDE and misses samples when its thread is late; micrus / RealSense drop frames under load | controller_time gaps; micrus frame_index; RealSense metadata frame_number |
| Publisher -> recorder | queue overwritten before the recorder reads it (micrus: keep-last **2**, not configurable; robot: now 500) | same counters, compared with the source logs |
| Recorder -> disk | recorder or disk too slow | per-episode QC on the bag |
| GUI | competes for CPU; must never be a reliable subscriber or a parent of capture processes | node panel; QC |

Every episode's QC now checks all three sources for gaps (`micrus_frame_gaps`, `robot_sample_gaps`,
`wrist_rate` incl. frame numbers); a gap fails the episode, so loss can never pass silently.
The micrus HDF5 recorded per episode keeps every raw ultrasound frame, independent of the bag.

## Measurement (dev PC, 8 cores, micrus simulator + robot_state on URSim + qc_monitor + MCAP recorder, 60 s each)
| Condition | Robot samples lost | US frames lost | Lost in the recorder itself | GUI CPU |
|---|---|---|---|---|
| no GUI (2 runs) | 6.6 % / 11.3 % | 1 / 15 | ~0 | - |
| GUI, default DDS (2 runs) | 25.5 % / 29.8 % | 33 / 73 | ~0 | 82-94 % |
| GUI, Fast DDS profile (2 runs) | 23.9 % / 34.5 % | 12 / 45 | ~0 | 88-112 % |

Before the fixes below, derived robot topics lost ~55 messages per minute in the recorder and the
dashboard crashed (PySide6 6.12 + pyqtgraph 0.14, `none_dealloc`, reproduced without rig code).

Caveats: the micrus simulator alone used 4.2-4.6 cores, URSim runs in Docker on the same PC at
0.968x real time, and a PDF viewer used ~1 core in one run. The capture PC must be measured again.

## Fixed (in the code)
- robot_state: RTDE reading separated from message building / publishing (queue, overflow counter).
- Robot topics: publisher queue depth 10 -> 500 (1 s); recorder-side loss of derived topics went from
  ~55/min to <= 3 (bag start/stop edges).
- qc_monitor: raw subscriptions, decodes 1 in 25 robot / 1 in 3 B-mode / 1 in 10 depth (was ~50 % CPU, now ~30-40 %).
- Recorder: `chrt -f recording.rt_priority` when allowed.
- Dashboard: PySide6 pinned to 6.8.3 (no crash in 90 s stress + all runs); exits on Ctrl+C / SIGTERM
  (it used to stay alive and pile up); runs at nice 10; nodes panel (running / MISSING / recording).
- QC: robot controller-time gaps and RealSense frame-number gaps per episode.

## Still needed (recommendations, in order)
1. **robot_state in C++.** Even without the GUI the Python reader loses 7-11 % of samples under load;
   the recorder loses ~none. A C++ node on `ur_client_library` (blocking `getDataPackage`, no polling)
   removes the main loss. Needs `sudo apt install ros-humble-ur-client-library`.
2. **CPU isolation on the capture PC**: rtprio limit (docs/capture_pc_setup.md); pin micrus,
   robot_state and the recorder to their own cores and the GUI to others (`taskset`); keep browsers /
   PDF viewers closed during sessions.
3. **GUI cost**: the dashboard uses about one core at 30 Hz. Showing B-mode at 15 Hz in the GUI (the bag
   still gets every frame) roughly halves that; it is a setting (`refresh_hz`), your choice.
4. **Measure on the capture PC** with the real scanner, camera and UR7e using the same runner
   (5 x 60 s with and without GUI) before Stage 0.
