"""Read-only RTDE sampling thread (ur_rtde RTDEReceiveInterface).

Each new controller sample gets exactly one system-clock stamp (time.time_ns()), taken as soon
as the sample is seen. All messages derived from it reuse that stamp.
"""
import threading
import time
from dataclasses import dataclass, field

import numpy as np


@dataclass
class RobotSample:
    stamp_ns: int
    controller_time_s: float
    q: np.ndarray
    qd: np.ndarray
    tcp_pose: np.ndarray        # [x, y, z, rx, ry, rz], rotation vector, base frame
    tcp_speed: np.ndarray       # [vx, vy, vz, wx, wy, wz], base frame
    wrench: np.ndarray          # actual_TCP_force, as reported by the controller
    di_bits: int = None         # actual_digital_input_bits (freedrive pedal on a configurable input)
    robot_mode: int = None      # RTDE robot_mode (7 = RUNNING)


@dataclass
class ReaderStats:
    samples: int = 0
    repeated_controller_time: int = 0
    controller_gaps: int = 0          # controller time step > 1.5 periods
    stamp_minus_ctrl: list = field(default_factory=list)


class RtdeReader:
    """Polls RTDEReceiveInterface for new samples and calls ``on_sample(sample)``."""

    def __init__(self, ip, frequency_hz, on_sample, poll_s=0.0002, rt_priority=0, log=print):
        self.ip = ip
        self.freq = float(frequency_hz)
        self.on_sample = on_sample
        self.poll_s = poll_s
        self.stats = ReaderStats()
        self._stats_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self.rtde_r = None
        self.rt_priority = int(rt_priority or 0)
        self.log = log

    def start(self):
        import rtde_receive
        try:
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.ip, self.freq, [], False, False, self.rt_priority)
        except RuntimeError as e:
            if not self.rt_priority:
                raise
            self.log(f'RTDE receive with rt_priority {self.rt_priority} failed ({e}); retrying with 0')
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.ip, self.freq)
        self._thread = threading.Thread(target=self._run, name='rtde_reader', daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.rtde_r is not None:
            try:
                self.rtde_r.disconnect()
            except Exception:
                pass

    def take_stats(self) -> ReaderStats:
        with self._stats_lock:
            s, self.stats = self.stats, ReaderStats()
        return s

    def _run(self):
        from usrig_common.config import set_thread_rt_priority
        ok, msg = set_thread_rt_priority(self.rt_priority)
        self.log(f'RTDE reader thread: {msg}')
        r = self.rtde_r
        period = 1.0 / self.freq
        last_ctrl = None
        while not self._stop.is_set():
            ctrl_t = r.getTimestamp()
            if ctrl_t == last_ctrl:
                time.sleep(self.poll_s)
                continue
            stamp_ns = time.time_ns()
            sample = RobotSample(
                stamp_ns=stamp_ns,
                controller_time_s=float(ctrl_t),
                q=np.asarray(r.getActualQ(), float),
                qd=np.asarray(r.getActualQd(), float),
                tcp_pose=np.asarray(r.getActualTCPPose(), float),
                tcp_speed=np.asarray(r.getActualTCPSpeed(), float),
                wrench=np.asarray(r.getActualTCPForce(), float),
                di_bits=int(r.getActualDigitalInputBits()),
                robot_mode=int(r.getRobotMode()),
            )
            with self._stats_lock:
                st = self.stats
                st.samples += 1
                if last_ctrl is not None:
                    dt = ctrl_t - last_ctrl
                    if dt <= 0:
                        st.repeated_controller_time += 1
                    elif dt > 1.5 * period:
                        st.controller_gaps += 1
                st.stamp_minus_ctrl.append(stamp_ns * 1e-9 - ctrl_t)
            last_ctrl = ctrl_t
            self.on_sample(sample)
