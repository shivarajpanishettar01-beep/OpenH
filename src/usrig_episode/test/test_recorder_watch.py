"""The recorder's readiness subscriptions must not kill episode_manager's MultiThreadedExecutor.

Found live (17 Oct 2026): episode_manager exited at an episode start with InvalidHandle "cannot use Destroyable
because destruction was requested": _unwatch destroyed raw subscriptions that the 4-thread executor was serving
(/robot/state at 500 Hz). Reproduced in isolation within 2-26 watch / unwatch cycles."""
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')


def test_watch_unwatch_cycles_keep_the_executor_alive():
    from rclpy.executors import MultiThreadedExecutor
    from rosidl_runtime_py.utilities import get_message
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from usrig_episode.recorder import EpisodeRecorder
    rclpy.init()
    try:
        pub_n = rclpy.create_node('fast_pub')
        pub = pub_n.create_publisher(String, '/fast_ready', 10)
        pub_n.create_timer(0.0005, lambda: pub.publish(String(data='x')))
        node = rclpy.create_node('episode_manager_test')
        ex = MultiThreadedExecutor(num_threads=4)
        ex.add_node(node)
        ex.add_node(pub_n)
        errors = []

        def spin():
            try:
                ex.spin()
            except Exception as e:  # noqa: BLE001
                errors.append(repr(e))
        th = threading.Thread(target=spin, daemon=True)
        th.start()
        time.sleep(0.5)                                              # discovery
        r = EpisodeRecorder.__new__(EpisodeRecorder)
        r.node, r._subs, r._typed, r._watcher, r._n_watch = node, [], set(), None, 0
        r._seen, r._seen_lock = {}, threading.Lock()
        r._get_message = get_message
        r._qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        seen = 0
        for _ in range(150):
            assert r._watch('/fast_ready')
            time.sleep(0.02)
            with r._seen_lock:
                seen += '/fast_ready' in r._seen
            r._unwatch()
            if errors or not th.is_alive():
                break
        assert not errors and th.is_alive(), errors
        assert seen > 50                                             # the readiness watch still sees messages
        ex.shutdown()
    finally:
        rclpy.shutdown()
