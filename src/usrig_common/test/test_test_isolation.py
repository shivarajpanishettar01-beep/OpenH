"""The test suite must never talk to a running rig: its sim nodes publish /teleop/twist, /operator/command."""
import os


def test_tests_run_on_their_own_ros_domain_and_localhost():
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0')
