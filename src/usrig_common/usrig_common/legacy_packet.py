"""Buffered parser for the lab URScript socket protocol (path A only).

Ported from the lab's newest ur10_launch.py (robot_states_callback + process_robot_packet):
collect bytes, extract complete ``[...]`` packets, require 18 values, drop malformed packets.
Unlike the original it never prints in the hot path; it counts drops instead.

State packet: [x, y, z, o1, o2, o3, fx, fy, fz, tx, ty, tz, q1..q6]
(o1..o3 is a rotation vector or roll/pitch/yaw depending on the URScript; see P0 report.)
"""
EXPECTED_VALUES = 18
MAX_GARBAGE = 4096


class PacketParser:

    def __init__(self, expected_values=EXPECTED_VALUES):
        self.expected = expected_values
        self.buf = ''
        self.good = 0
        self.bad = 0

    def feed(self, data: bytes):
        """Add received bytes; return a list of complete, valid packets (lists of floats)."""
        self.buf += data.decode('utf-8', errors='ignore')
        out = []
        while True:
            start = self.buf.find('[')
            if start == -1:
                if len(self.buf) > MAX_GARBAGE:
                    self.buf = ''
                return out
            end = self.buf.find(']', start)
            if end == -1:
                if start > 0:
                    self.buf = self.buf[start:]
                return out
            packet = self.buf[start + 1:end]
            self.buf = self.buf[end + 1:]
            values = self._parse(packet)
            if values is not None:
                out.append(values)

    def _parse(self, body):
        fields = [f.strip() for f in body.strip().split(',')]
        if len(fields) != self.expected or any(f == '' for f in fields):
            self.bad += 1
            return None
        try:
            values = [float(f) for f in fields]
        except ValueError:
            self.bad += 1
            return None
        self.good += 1
        return values


def format_command(x, y, z, o1, o2, o3, fz, start) -> bytes:
    """Command tuple the lab URScript expects: (x, y, z, o1, o2, o3, fz, start)."""
    return ('(' + ','.join(f'{v:.6f}' for v in (x, y, z, o1, o2, o3, fz)) + f',{int(start)})').encode()
