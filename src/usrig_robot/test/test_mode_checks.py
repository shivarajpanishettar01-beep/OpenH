"""teleop_check and preset_check end to end against robot_control on the kinematic fake robot.

Real ROS plumbing: robot_control is spun in an executor and serves its services; /robot/state is published
from the fake robot; the check tools run as on the rig (ToolNode, /robot/set_teleop, /teleop/twist,
/robot/move_to_pose) with --yes. MOCKED integration, not a hardware test.
"""
import threading

import numpy as np
import pytest
from rclpy.executors import MultiThreadedExecutor

import test_control_sim
from usrig_common import geometry
from usrig_common.timeutil import stamp_from_ns
from usrig_interfaces.msg import RobotState

sim = test_control_sim.sim          # the kinematic fake robot fixture


@pytest.fixture
def rig_up(sim, tmp_path):
    from usrig_common.tool_node import ToolNode
    nodes = []

    def make(p0=(0.4, 0.0, 0.2)):
        node, robot = sim(p0=p0)
        pub_node = node.create_publisher(RobotState, '/robot/state', 50)

        def publish():
            s = robot.get_latest()
            fsp, mode, flags, src, expert = node.status.snapshot()
            m = RobotState()
            m.header.stamp = stamp_from_ns(s.stamp_ns)
            geometry.fill_pose_msg(m.tcp_base, geometry.pose_from_ur(s.tcp_pose))
            m.tcp_phantom = m.tcp_base
            m.wrench = s.wrench.tolist()
            m.force_setpoint_n, m.mode, m.safety_flags = float(fsp), mode, flags
            m.action_source, m.action_is_expert = src, bool(expert)
            pub_node.publish(m)
        node.create_timer(0.005, publish)
        ex = MultiThreadedExecutor(num_threads=4)
        ex.add_node(node)
        threading.Thread(target=ex.spin, daemon=True).start()
        tool = ToolNode('check_under_test')
        tool.scfg['session']['root'] = str(tmp_path)          # presets go to <tmp>/presets
        nodes.append((ex, node, tool))
        tool.state(5.0)
        return node, robot, tool
    yield make
    for ex, node, tool in nodes:
        tool.destroy_node()
        ex.shutdown()
        node.destroy_node()


def test_teleop_check_slide_passes_and_moves_along_probe_x(rig_up):
    from usrig_teleop import teleop_check
    node, robot, tool = rig_up()
    p0 = robot.T[:3, 3].copy()
    assert teleop_check.main(['--axis', 'slide_along', '--speed', '10', '--seconds', '1', '--yes'], node=tool)
    d = (robot.T[:3, 3] - p0) * 1000
    assert 7.0 < d[0] < 11.0 and abs(d[1]) < 0.5                  # probe x = base x for the fake's probe-down pose
    assert node.hold                                               # teleop disabled again at the end


def test_teleop_check_rotation_pivots_at_probe_face_and_back(rig_up):
    from usrig_teleop import teleop_check
    node, robot, tool = rig_up()
    T0 = robot.T.copy()
    assert teleop_check.main(['--axis', 'rotate', '--speed', '10', '--seconds', '1', '--back', '--yes'], node=tool)
    d = (robot.T[:3, 3] - T0[:3, 3]) * 1000
    assert np.linalg.norm(d) < 0.5


def test_teleop_check_reports_limit_hit_as_fail(rig_up):
    from usrig_teleop import teleop_check
    node, robot, tool = rig_up()
    assert not teleop_check.main(['--axis', 'vx', '--speed', '80', '--seconds', '0.5', '--yes'], node=tool)


def test_teleop_check_refuses_with_another_twist_publisher(rig_up):
    from geometry_msgs.msg import TwistStamped
    from usrig_teleop import teleop_check
    node, robot, tool = rig_up()
    other = node.create_publisher(TwistStamped, '/teleop/twist', 10)  # e.g. keyboard_teleop running
    p0 = robot.T[:3, 3].copy()
    with pytest.raises(RuntimeError, match='already publish /teleop/twist'):
        teleop_check.main(['--axis', 'vx', '--speed', '5', '--seconds', '1', '--yes'], node=tool)
    assert np.linalg.norm(robot.T[:3, 3] - p0) < 1e-4
    node.destroy_publisher(other)


def test_teleop_check_not_confirmed_moves_nothing(rig_up):
    from usrig_teleop import teleop_check
    node, robot, tool = rig_up()
    p0 = robot.T[:3, 3].copy()
    assert not teleop_check.main(['--axis', 'vx', '--speed', '5', '--seconds', '1'], node=tool, ask=lambda _: 'no')
    assert np.linalg.norm(robot.T[:3, 3] - p0) < 1e-4


def test_preset_check_offset_and_back(rig_up):
    from usrig_presets import preset_check
    node, robot, tool = rig_up()
    p0 = robot.T[:3, 3].copy()
    assert preset_check.main(['offset', '--dy', '10', '--rz', '5', '--back', '--yes'], node=tool)
    assert np.linalg.norm(robot.T[:3, 3] - p0) < 0.001


def test_preset_check_save_go(rig_up):
    from usrig_presets import preset_check
    node, robot, tool = rig_up()
    assert preset_check.main(['save', 'home'], node=tool)
    p0 = robot.T[:3, 3].copy()
    assert preset_check.main(['offset', '--dx', '15', '--frame', 'base', '--yes'], node=tool)
    assert preset_check.main(['go', 'home', '--yes'], node=tool)
    assert np.linalg.norm(robot.T[:3, 3] - p0) < 0.001
    tool.rig['probe']['type'] = 'another probe'                    # remounted: preset no longer usable
    with pytest.raises(SystemExit, match='different probe_type'):
        preset_check.main(['go', 'home', '--yes'], node=tool)


def test_preset_check_outside_box_refused_and_reported(rig_up):
    from usrig_presets import preset_check
    node, robot, tool = rig_up(p0=(0.58, 0.0, 0.2))                 # box x max 0.6
    p0 = robot.T[:3, 3].copy()
    assert not preset_check.main(['offset', '--dx', '40', '--frame', 'base', '--yes'], node=tool)
    assert np.linalg.norm(robot.T[:3, 3] - p0) < 1e-4
