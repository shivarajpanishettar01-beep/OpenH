# <dataset name>: robotic ultrasound on a phantom (UR7e + Telemed probe)

Fill every field before release. Field list follows the Open-H v2 submission template
(check it against the current Open-H guide; NOT VERIFIED here).

## Summary
- Task: ultrasound scanning of <phantom> to reach target views from natural-language instructions
- Robot: Universal Robots UR7e, controller TCP = <tcp setting>; probe <probe type>
- Episodes: <n> (<hours> h); success / recovery / failure: <counts>
- Operators: <n>; sessions: <n>; held-out test sessions: <ids>

## Sensors and rates
| stream | source | rate | notes |
|---|---|---|---|
| observation.images.ultrasound | micrus /micrus/bmode | ~27 fps | stamp = last scan line; offset <measured> s |
| observation.images.wrist_rgb | RealSense D415 | 30 fps | exposure / WB locked |
| observation.state | RTDE 500 Hz | resampled to 25 fps | joints + TCP pose |
| observation.meta.force_torque | UR controller | 500 Hz | base frame |

## Actions
Executed probe-frame TCP twist (after the safety envelope) + force setpoint along the probe axis.
`observation.meta.action_is_expert` is false during transit and injected perturbations.

## Splits
See `info.json` `splits`: train, recovery, failure, test (whole held-out sessions).

## Failure modes
Catalog IDs FM-01, 02, 05, 06, 07, 12 staged two ways each; FM-03 never beyond the force band;
FM-04, 08-11 never staged.

## Calibration
prob_cali_mtx, wrist_cam_cali_mtx (eye-in-hand ChArUco, residual <x> mm), temporal offsets.

## Known limitations
<list>
