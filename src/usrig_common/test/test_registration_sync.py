"""push / matches against a live node holding T_base_phantom (as robot_state does), and rig_static_tf reload."""
import json
import threading

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_push_and_match_a_running_registration(ros):
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter
    from usrig_common import registration_sync as rs
    holder = rclpy.create_node('fake_robot_state')
    holder.declare_parameter('T_base_phantom', Parameter.Type.DOUBLE_ARRAY)
    client = rclpy.create_node('registration_client')
    ex = MultiThreadedExecutor()
    ex.add_node(holder)
    ex.add_node(client)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        T = np.eye(4)
        T[:3, 3] = [0.4, -0.1, 0.02]
        assert rs.matches(client, T, target='/fake_robot_state')[0] is False      # none yet
        ok, msg = rs.push(client, T, target='/fake_robot_state')
        assert ok, msg
        assert rs.matches(client, T, target='/fake_robot_state')[0] is True
        T2 = T.copy()
        T2[0, 3] += 0.001
        assert rs.matches(client, T2, target='/fake_robot_state')[0] is False
        assert rs.matches(client, T, target='/nobody', timeout_s=0.3)[0] is None
    finally:
        ex.shutdown()
        holder.destroy_node()
        client.destroy_node()


def test_rig_static_tf_republishes_on_reload(tmp_path):
    from std_msgs.msg import String
    sj = tmp_path / 'session.json'
    sj.write_text(json.dumps({}))
    rclpy.init(args=['--ros-args', '-p', f'session_json:={sj}'])
    try:
        from usrig_episode.static_tf import RigStaticTf
        n = RigStaticTf()
        assert [t.child_frame_id for t in n.tfs] == [] or 'phantom' not in [t.child_frame_id for t in n.tfs]
        T = np.eye(4)
        T[:3, 3] = [0.5, 0.0, 0.1]
        sj.write_text(json.dumps({'phantom_registration': {'T_base_phantom': T.tolist()}}))
        n._on_cmd(String(data=json.dumps({'cmd': 'reload_registration'})))
        ph = [t for t in n.tfs if t.child_frame_id == 'phantom']
        assert ph and ph[0].transform.translation.x == pytest.approx(0.5)
        n.destroy_node()
    finally:
        rclpy.shutdown()
