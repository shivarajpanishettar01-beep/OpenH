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

## C++ robot_state (usrig_robot_cpp, ur_client_library 2.15.2) - same load, 60 s runs
| robot_state | GUI | Robot samples lost | US frames lost | robot_state CPU | GUI CPU |
|---|---|---|---|---|---|
| Python (2 runs) | no | 6.6 % / 11.3 % | 1 / 15 | ~65 % | - |
| Python (2 runs) | yes | 25.5 % / 29.8 % | 33 / 73 | ~65 % | 82-94 % |
| **C++ (2 runs)** | no | **0 / 0** of ~31,000 | 2 / 0 | **~6 %** | - |
| **C++ (2 runs)** | yes | **0 / 0** of ~30,600 | 4 / 0 | ~6 % | 124-138 % |

Key detail: in ur_client_library 2.15 the default background-read mode returns only the *latest*
package (it would drop samples like the Python poller). robot_state_cpp uses `start(false)` +
`getDataPackageBlocking()`, which reads every packet in order from the TCP stream.
robot.launch.py uses the C++ node when built and falls back to Python with a warning.

## GUI only for demos
Collection runs headless: keyboard teleop (evdev), episode_manager, recorder, qc_monitor, and
`ros2 run usrig_qc status_console` (terminal view: episode / recording state, robot mode and
faults, QC and per-topic rates, recent events; low-rate topics only). The dashboard (1.2-1.4
cores here) is started only for demos (`dashboard:=true`); the annotator runs offline.

## Still needed (recommendations, in order)
1. ~~robot_state in C++~~: done (above).
2. **CPU isolation on the capture PC**: rtprio limit (docs/capture_pc_setup.md); pin micrus,
   robot_state and the recorder to their own cores and the GUI to others (`taskset`); keep browsers /
   PDF viewers closed during sessions.
3. **GUI cost**: the dashboard uses about one core at 30 Hz. Showing B-mode at 15 Hz in the GUI (the bag
   still gets every frame) roughly halves that; it is a setting (`refresh_hz`), your choice.
4. **Measure on the capture PC** with the real scanner, camera and UR7e using the same runner
   (5 x 60 s with and without GUI) before Stage 0.
