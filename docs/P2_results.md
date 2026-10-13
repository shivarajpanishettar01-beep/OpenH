# P2 results: robot_state on URSim (13 October 2026)

Setup: URSim e-series 5.26.1 in Docker on the development PC; dashboard reports model `UR5`
(the model actually loaded is to be confirmed: UR7e or UR5e). Robot POWER_OFF, stationary,
safety NORMAL, no program running. `ros2 launch usrig_bringup robot.launch.py sim:=true`
(read-only RTDE receive at 127.0.0.1, 500 Hz requested).

| Check | Result | Pass |
|---|---|---|
| Rate by our stamps (60 s) | **481.3 Hz** (28,878 samples) | **no** (500 ±1 % = 495-505) |
| URSim controller clock vs wall clock | **0.968** (58.08 s controller time in 60.00 s) | - |
| Rate in controller time | 497.2 samples per controller second | yes (within 1 %) |
| Repeated `controller_time_s` | 0 | yes |
| Controller steps > 3 ms (missed samples) | 117 in 58 s (~2.0/s); bare ur_rtde loop without ROS: 39 in 30 s (~1.3/s) | see notes |
| Same stamp on /robot/state, /joint_states, /robot/tcp_pose, /robot/wrench | 28,876 / 28,876 / 28,875 identical | yes |
| Our stamp interval | median 2.058 ms, p99 2.560 ms, max 10.8 ms | info |
| stamp - controller time | drift 33.2 ms per controller second (URSim slower than real time); after removing the drift: abs p50 3.05 ms, p99 9.37 ms | info (sim) |
| robot_state CPU | ~60 % of one core | info |
| TCP pose vs teach pendant | read below; waiting for the pendant reading | pending |

Pose at the time of reading (base frame, active TCP):
X -143.97 mm, Y -435.62 mm, Z 202.03 mm; RX -0.0012, RY 3.1163, RZ 0.0389 rad (rotation vector);
joints -91.71, -98.96, -126.22, -46.29, 91.39, -1.78 deg.

Interpretation
- The rate check fails on URSim because the simulator's controller is not real time on this PC:
  it produces 2 ms controller steps at 0.968x wall speed. Measured in controller time the stream is
  497 samples/s with no repeats. The 500 Hz +-1 % check is only meaningful on the real controller
  (P2 stationary check on the robot, which needs your go-ahead).
- Missed samples: robot_state polls `getTimestamp()` and misses a sample when two arrive within
  one poll / scheduling delay. The bare loop shows that most misses come from polling plus URSim
  timing; the ROS node adds ~0.7/s on top. Re-measure on the real controller; if the misses stay,
  move the reader to ur_rtde's own waitPeriod-paced loop or to C++.
- The jitter numbers on URSim are dominated by the simulator's own timing and are not a measure of
  the real rig.
