# P1 results: micrus bringup with the simulator (13 October 2026)

Command built by `usrig_bringup/launch/micrus.launch.py` from `session.yaml` (sim:=true):

    micrus serve --ros2 --lines 128 --depth 40 --pixel-mm 0.1 --out ~/MicrUs/p1_sim \
      --ros2-name micrus --ros2-frame micrus_probe --simulate --sim-probe 49 --sim-pid 0x3447

Simulator: MPRO-L40S (code 49), firmware 2019.10.24, 128 lines x 710 samples, programmed 27.4 fps.
Topics: `/micrus/bmode`, `/micrus/beamformed`, `/micrus/frame_info`. `/micrus/bmode` QoS: reliable,
volatile. Image 407 x 411 px at 0.1 mm, origin (-20.3, 0) mm.

| Check | Result | Pass |
|---|---|---|
| `/micrus/bmode` rate (5 min, `stream_check --micrus --duration 300`) | 27.35 Hz, 8,225 frames | yes (~27 Hz) |
| `frame_index` gaps over 5 min | 0 gaps in 8,224 frames; settings constant (1 distinct) | yes |
| Stamps | 0 non-increasing; median interval 36.4 ms; std 8.4 ms; max 720 ms (once) | see note |
| Stamp minus subscriber clock | median -12.0 ms, p99 -9.2 ms (stamp = last scan line, before arrival) | info |
| Clean shutdown, Ctrl+C on `ros2 launch` | launch forwards SIGINT, micrus logs `FREEZE` then `Device powered OFF`, exits cleanly in 1.0 s | yes |
| Shutdown by SIGTERM to micrus | `FREEZE`, `Device powered OFF`, finished cleanly | yes |

Notes
- One 720 ms interval between consecutive frames with no `frame_index` gap: micrus paused
  acquisition once (its own 5 s reports ranged 26.9-31.4 fps, consistent with a stall and
  catch-up). Load average was ~5 on 8 cores. Re-check on the real scanner (P9b, 30 min).
- First 5 min run showed 6 `frame_index` gaps: caused by the checker itself (deserializing every
  B-mode image in Python fell behind micrus's keep-last-2 queue). A frame_info-only listener saw
  0 gaps in 3,290 frames; `stream_check` now reads image stamps from raw CDR, and the repeat run
  had 0 gaps. Consequence for the rig: any slow *reliable* subscriber to micrus can lose frames
  (publisher depth 2). The bag recorder is C++; its drop rate is checked per episode by QC.
- `ros2 launch` runs under /usr/bin/python3 (the `ros2` script's shebang), not the openh env;
  nodes started by it use the openh Python.
