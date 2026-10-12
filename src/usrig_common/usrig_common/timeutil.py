"""System-clock stamping. Every rig message is stamped with time.time_ns() at capture."""
import time

from builtin_interfaces.msg import Time


def now_ns() -> int:
    return time.time_ns()


def stamp_from_ns(ns: int) -> Time:
    t = Time()
    t.sec = int(ns // 1_000_000_000)
    t.nanosec = int(ns % 1_000_000_000)
    return t


def ns_from_stamp(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def now_stamp() -> Time:
    return stamp_from_ns(time.time_ns())
