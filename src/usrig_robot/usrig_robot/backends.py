"""The two ways robot_control can drive the UR7e. Which one is used is rig.yaml
``robot.control_backend``. Decided at the P3 STOP (13 Oct 2026): path B, ``rtde``.

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

    def kick(self):
        """Called every control tick, also when nothing is commanded (controller-side deadman)."""

    def command(self, T_tcp, v_base, w_base, accel, force_on, force_setpoint_n, dt):
        """Returns False when the controller refused the command (robot_control FAULTs)."""
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def ensure_running(self):
        """Before resuming after a FAULT: make sure the controller side can take commands again."""
        return True

    def zero_ft(self):
        raise NotImplementedError

    def protective_stop(self):
        self.stop()


class TcpMismatchError(RuntimeError):
    pass


class PayloadMismatchError(RuntimeError):
    pass


# Payload check tolerances: rig.yaml holds the values typed on the pendant, so they should match closely.
PAYLOAD_TOL_KG = 0.01
PAYLOAD_COG_TOL_M = 0.001


class RtdeBackend(Backend):
    """Path B (chosen 13 Oct 2026): ur_rtde RTDEControlInterface, speedL + forceMode along the
    probe axis. Needs the robot in Remote Control; the lab URScript cannot run at the same time.

    Controller-side deadman: setWatchdog(1 / deadman_timeout_s). The control loop kicks it every
    tick; if this process stalls or dies, the controller stops the robot by itself.
    NOT VERIFIED on the rig: test every call in the URSim dry run first.
    """
    name = 'rtde'

    def __init__(self, ip, frequency_hz, force_limits, force_damping=None, watchdog_min_hz=None,
                 stop_decel=2.0, expected_tcp=None, expected_payload=None, speedl_time_ticks=1.0,
                 rt_priority=0, log=print):
        self.ip = ip
        self.freq = frequency_hz
        self.force_limits = list(force_limits)   # rig.yaml rtde.force_mode_limits [6]
        self.force_damping = force_damping
        self.watchdog_min_hz = watchdog_min_hz
        self.stop_decel = float(stop_decel)
        self.expected_tcp = expected_tcp
        self.expected_payload = expected_payload   # (mass_kg, [cx, cy, cz] m) or None
        self.speedl_time_ticks = float(speedl_time_ticks or 1.0)
        self.rt_priority = int(rt_priority or 0)
        self.log = log
        self.c = None
        self.force_active = False
        self.tcp_offset = None
        self.payload = None

    def connect(self):
        import rtde_control
        try:
            self.c = rtde_control.RTDEControlInterface(self.ip, self.freq, rtde_control.RTDEControlInterface.FLAG_UPLOAD_SCRIPT,
                                                       50002, self.rt_priority)
        except RuntimeError as e:
            if not self.rt_priority:
                raise
            self.log(f'RTDE control with rt_priority {self.rt_priority} failed ({e}); retrying with 0')
            self.c = rtde_control.RTDEControlInterface(self.ip, self.freq)
        self.tcp_offset = [float(x) for x in self.c.getTCPOffset()]
        self.log(f'controller active TCP offset: {[round(x, 5) for x in self.tcp_offset]}')
        if self.expected_tcp is not None:
            d = np.abs(np.asarray(self.tcp_offset) - np.asarray(self.expected_tcp, float))
            if d[:3].max() > 1e-4 or d[3:].max() > 1e-3:
                self.close()
                raise TcpMismatchError(f'controller TCP {self.tcp_offset} differs from rig.yaml robot.tcp_setting '
                                       f'{self.expected_tcp}: refusing (probe offset could be applied twice)')
        self.payload = self._read_payload()
        self.log(f'controller payload: {self.payload[0]:.3f} kg, CoG {[round(x, 4) for x in self.payload[1]]} m')
        if self.expected_payload is not None:
            m, cog = float(self.expected_payload[0]), np.asarray(self.expected_payload[1], float)
            if abs(self.payload[0] - m) > PAYLOAD_TOL_KG or np.abs(np.asarray(self.payload[1]) - cog).max() > PAYLOAD_COG_TOL_M:
                self.close()
                raise PayloadMismatchError(f'controller payload {self.payload[0]:.3f} kg / CoG {self.payload[1]} differs '
                                           f'from rig.yaml robot.payload_kg / payload_cog_m {m} / {cog.tolist()}: '
                                           'refusing (force readings would change with tilt)')
        if self.force_damping is not None:
            self.c.forceModeSetDamping(float(self.force_damping))
        if self.watchdog_min_hz:
            if not self.c.setWatchdog(float(self.watchdog_min_hz)):
                raise RuntimeError('setWatchdog failed')
            self.log(f'controller watchdog: min {self.watchdog_min_hz:.1f} Hz')

    def _read_payload(self):
        """(mass_kg, [cx, cy, cz]) as set on the controller. RTDEControlInterface cannot read it, so a
        short-lived receive connection is used."""
        import rtde_receive
        r = rtde_receive.RTDEReceiveInterface(self.ip, -1.0, ['payload', 'payload_cog'])
        try:
            return float(r.getPayload()), [float(x) for x in r.getPayloadCog()]
        finally:
            r.disconnect()

    def close(self):
        if self.c is not None:
            try:
                self.stop()
                self.c.stopScript()
            finally:
                self.c.disconnect()
                self.c = None

    def kick(self):
        if self.c is not None and self.watchdog_min_hz:
            self.c.kickWatchdog()

    def command(self, T_tcp, v_base, w_base, accel, force_on, force_setpoint_n, dt):
        if force_on:
            task_frame = geometry.ur_from_pose(T_tcp)
            # selection: compliant along tool z only; wrench along tool +z (into tissue).
            # Sign convention of forceMode wrench vs probe direction: NOT VERIFIED (check in URSim).
            if self.c.forceMode(task_frame, [0, 0, 1, 0, 0, 0],
                                [0.0, 0.0, float(force_setpoint_n), 0.0, 0.0, 0.0],
                                2, self.force_limits) is False:
                return False
            self.force_active = True
        elif self.force_active:
            self.c.forceModeStop()
            self.force_active = False
        # R8: the command lives speedl_time_ticks x dt, so one late tick does not decelerate the arm;
        # the controller watchdog still stops a dead loop.
        return self.c.speedL([*map(float, v_base), *map(float, w_base)], float(accel),
                             float(dt) * self.speedl_time_ticks) is not False

    def ensure_running(self):
        """A protective / safeguard stop ends the ur_rtde control script on the controller: upload it again."""
        if self.c is None:
            return False
        if self.c.isProgramRunning():
            return True
        self.log('RTDE control script not running on the controller: uploading it again')
        self.force_active = False
        return bool(self.c.reuploadScript())

    def stop(self):
        if self.c is None:
            return
        if self.force_active:
            self.c.forceModeStop()
            self.force_active = False
        self.c.speedStop(self.stop_decel)

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


def _payload(rig):
    from usrig_common.config import get_path
    m, cog = get_path(rig, 'robot.payload_kg'), get_path(rig, 'robot.payload_cog_m')
    return None if m is None or cog is None else (float(m), [float(x) for x in cog])


def make_backend(rig, log=print, ip=None):
    from usrig_common.config import get_path
    kind = get_path(rig, 'robot.control_backend')
    if kind == 'rtde':
        deadman = get_path(rig, 'limits.deadman_timeout_s')
        return RtdeBackend(ip or get_path(rig, 'robot.ip'), get_path(rig, 'robot.control_rate_hz') or 125.0,
                           get_path(rig, 'rtde.force_mode_limits'),
                           get_path(rig, 'rtde.force_mode_damping'),
                           watchdog_min_hz=(1.0 / float(deadman)) if deadman else None,
                           stop_decel=float(get_path(rig, 'limits.stop_decel_mm_s2')) / 1000.0,
                           expected_tcp=get_path(rig, 'robot.tcp_setting'),
                           expected_payload=_payload(rig),
                           speedl_time_ticks=get_path(rig, 'robot.speedl_time_ticks') or 1.0,
                           rt_priority=get_path(rig, 'robot.rt_priority') or 0, log=log)
    if kind == 'urscript':
        return UrscriptBackend(get_path(rig, 'urscript.port'), get_path(rig, 'robot.control_rate_hz') or 125.0,
                               get_path(rig, 'urscript.orientation_format'),
                               get_path(rig, 'urscript.lookahead_s'))
    raise ValueError(f"robot.control_backend must be 'rtde' or 'urscript' (got {kind!r})")
