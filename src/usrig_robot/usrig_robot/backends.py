"""The two ways robot_control can drive the UR7e. Which one is used is rig.yaml
``robot.control_backend`` (null until decided at the P3 STOP):

* ``rtde``     path B: ur_rtde RTDEControlInterface, speedL + forceMode along the probe axis.
               Cannot run while the lab URScript program is running on the robot.
* ``urscript`` path A: the lab URScript connects to this PC (TCP client -> port 51680) and
               receives absolute pose targets ``(x, y, z, o1, o2, o3, fz, start)`` at a fixed rate,
               latest command only. Needs a command watchdog inside the URScript (not added
               here: editing the URScript is the user's decision).

Both receive the same per-tick request from robot_control: a base-frame twist that has already
passed the safety filter, plus whether force regulation along the probe axis is on.
"""
import socket
import threading
import time

import numpy as np

from usrig_common import geometry
from usrig_common.legacy_packet import PacketParser, format_command


class Backend:
    name = 'none'

    def connect(self):
        raise NotImplementedError

    def close(self):
        pass

    def command(self, T_tcp, v_base, w_base, accel, force_on, force_setpoint_n, dt):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def zero_ft(self):
        raise NotImplementedError

    def protective_stop(self):
        self.stop()


class RtdeBackend(Backend):
    """Path B. NOT VERIFIED on this rig: test every call in URSim first."""
    name = 'rtde'

    def __init__(self, ip, frequency_hz, force_limits, force_damping=None):
        self.ip = ip
        self.freq = frequency_hz
        self.force_limits = list(force_limits)   # rig.yaml rtde.force_mode_limits [6]
        self.force_damping = force_damping
        self.c = None
        self.force_active = False

    def connect(self):
        import rtde_control
        self.c = rtde_control.RTDEControlInterface(self.ip, self.freq)
        if self.force_damping is not None:
            self.c.forceModeSetDamping(float(self.force_damping))

    def close(self):
        if self.c is not None:
            try:
                self.stop()
                self.c.stopScript()
            finally:
                self.c.disconnect()

    def command(self, T_tcp, v_base, w_base, accel, force_on, force_setpoint_n, dt):
        if force_on:
            task_frame = geometry.ur_from_pose(T_tcp)
            # selection: compliant along tool z only; wrench along tool +z (into tissue).
            # Sign convention of forceMode wrench vs probe direction: NOT VERIFIED (check in URSim).
            self.c.forceMode(task_frame, [0, 0, 1, 0, 0, 0],
                             [0.0, 0.0, float(force_setpoint_n), 0.0, 0.0, 0.0],
                             2, self.force_limits)
            self.force_active = True
        elif self.force_active:
            self.c.forceModeStop()
            self.force_active = False
        self.c.speedL([*map(float, v_base), *map(float, w_base)], float(accel), float(dt))

    def stop(self):
        if self.c is None:
            return
        if self.force_active:
            self.c.forceModeStop()
            self.force_active = False
        self.c.speedStop(2.0)

    def zero_ft(self):
        self.c.zeroFtSensor()

    def protective_stop(self):
        self.c.triggerProtectiveStop()


class UrscriptBackend(Backend):
    """Path A. Server socket for the lab URScript. A fixed-rate sender thread always sends the
    latest target only; commands are never queued. Incoming state packets are parsed and
    discarded (state comes from RTDE) so the socket buffer never fills.
    """
    name = 'urscript'

    def __init__(self, port, rate_hz, orientation_format, lookahead_s, host='0.0.0.0'):
        if orientation_format not in ('rotvec', 'rpy'):
            raise ValueError("urscript.orientation_format must be 'rotvec' or 'rpy' (see P0 report)")
        self.host, self.port = host, int(port)
        self.period = 1.0 / float(rate_hz)
        self.orientation_format = orientation_format
        self.lookahead_s = float(lookahead_s)
        self.parser = PacketParser()
        self._latest = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.conn = None
        self.sock = None

    def connect(self, accept_timeout_s=60.0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.listen(1)
        self.sock.settimeout(accept_timeout_s)
        self.conn, _ = self.sock.accept()
        self.conn.settimeout(0.0)
        threading.Thread(target=self._sender, name='urscript_sender', daemon=True).start()

    def close(self):
        self._stop.set()
        for s in (self.conn, self.sock):
            try:
                s.close()
            except Exception:
                pass

    def _encode(self, T, fz, start):
        if self.orientation_format == 'rotvec':
            o = geometry.ur_from_pose(T)[3:]
        else:
            from scipy.spatial.transform import Rotation
            o = Rotation.from_matrix(T[:3, :3]).as_euler('xyz')
        p = T[:3, 3]
        return format_command(p[0], p[1], p[2], o[0], o[1], o[2], fz, start)

    def _sender(self):
        nxt = time.monotonic()
        while not self._stop.is_set():
            with self._lock:
                pkt = self._latest
            try:
                try:
                    data = self.conn.recv(65536)
                    if data:
                        self.parser.feed(data)
                except BlockingIOError:
                    pass
                if pkt is not None:
                    self.conn.sendall(pkt)
            except OSError:
                self._stop.set()
                break
            nxt += self.period
            time.sleep(max(0.0, nxt - time.monotonic()))

    def command(self, T_tcp, v_base, w_base, accel, force_on, force_setpoint_n, dt):
        from scipy.spatial.transform import Rotation
        T = T_tcp.copy()
        T[:3, 3] = T_tcp[:3, 3] + np.asarray(v_base) * self.lookahead_s
        T[:3, :3] = Rotation.from_rotvec(np.asarray(w_base) * self.lookahead_s).as_matrix() @ T_tcp[:3, :3]
        fz = float(force_setpoint_n) if force_on else 0.0
        with self._lock:
            self._latest = self._encode(T, fz, 1)

    def stop(self):
        with self._lock:
            self._latest = None   # stop sending; the URScript watchdog must stop the arm

    def zero_ft(self):
        raise NotImplementedError('zero_ft over the lab URScript protocol is not defined (P0)')


def make_backend(rig):
    from usrig_common.config import get_path
    kind = get_path(rig, 'robot.control_backend')
    if kind == 'rtde':
        return RtdeBackend(get_path(rig, 'robot.ip'), get_path(rig, 'robot.control_rate_hz') or 125.0,
                           get_path(rig, 'rtde.force_mode_limits'),
                           get_path(rig, 'rtde.force_mode_damping'))
    if kind == 'urscript':
        return UrscriptBackend(get_path(rig, 'urscript.port'), get_path(rig, 'robot.control_rate_hz') or 125.0,
                               get_path(rig, 'urscript.orientation_format'),
                               get_path(rig, 'urscript.lookahead_s'))
    raise ValueError(f"robot.control_backend must be 'rtde' or 'urscript' (got {kind!r})")
