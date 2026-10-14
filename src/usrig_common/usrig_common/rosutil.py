"""Small ROS process helpers."""
import signal


def ignore_sigint():
    """Call first thing in a node's cleanup. ros2 launch and the terminal both send SIGINT on Ctrl+C; a second
    one arriving while rclpy tears down its signal handler kills the process (exit -2, 'process has died')
    in the middle of its cleanup (robot_control: stopping the arm). Once shutdown has started, ignore it."""
    try:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    except ValueError:            # not the main thread
        pass
