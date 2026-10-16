"""The running robot_state's phantom registration (parameter T_base_phantom, row-major 4x4) vs session.json.

robot_state (C++ or Python) computes /robot/state.tcp_phantom from T_base_phantom. A registration made while the
launch runs is pushed with push(); episode_manager refuses Start while matches() is False, so episodes never
record the base pose as tcp_phantom. Service calls wait on the future: the node must be spun elsewhere
(executor thread), or pass spin=True.
"""
import time

import numpy as np

NODE = '/robot_state'
PARAM = 'T_base_phantom'


def flat(T):
    return [float(x) for x in np.asarray(T, float).reshape(16)]


def _wait(node, fut, timeout_s, spin):
    if spin:
        import rclpy
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_s)
    t0 = time.monotonic()
    while not fut.done() and time.monotonic() - t0 < timeout_s:
        time.sleep(0.01)
    return fut.result() if fut.done() else None


def push(node, T, timeout_s=3.0, spin=False, target=NODE):
    """(ok, message): set T_base_phantom on the running robot_state."""
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    cli = node.create_client(SetParameters, f'{target}/set_parameters')
    try:
        if not cli.wait_for_service(timeout_sec=timeout_s):
            return False, f'{target} not running: the registration is used when the launch starts it'
        req = SetParameters.Request(parameters=[Parameter(name=PARAM, value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=flat(T)))])
        r = _wait(node, cli.call_async(req), timeout_s, spin)
        if r is None:
            return False, f'{target}/set_parameters timed out'
        res = r.results[0]
        return bool(res.successful), res.reason or 'updated'
    finally:
        node.destroy_client(cli)


def current(node, timeout_s=2.0, spin=False, target=NODE):
    """The running robot_state's T_base_phantom as 16 floats ([] = none); None if it cannot be asked."""
    from rcl_interfaces.srv import GetParameters
    cli = node.create_client(GetParameters, f'{target}/get_parameters')
    try:
        if not cli.wait_for_service(timeout_sec=timeout_s):
            return None
        r = _wait(node, cli.call_async(GetParameters.Request(names=[PARAM])), timeout_s, spin)
        if r is None:
            return None
        return list(r.values[0].double_array_value) if r.values else []     # declared but unset: no registration
    finally:
        node.destroy_client(cli)


def matches(node, T, timeout_s=2.0, spin=False, tol=1e-9, target=NODE):
    """(ok, message): the running robot_state uses registration T. ok None = robot_state cannot be asked."""
    cur = current(node, timeout_s, spin, target)
    if cur is None:
        return None, f'{target} does not answer (not running?)'
    if not cur:
        return False, 'robot_state has no phantom registration (started before register_phantom)'
    d = float(np.max(np.abs(np.asarray(cur) - np.asarray(flat(T)))))
    return (d <= tol), ('ok' if d <= tol else f'robot_state uses another registration (max difference {d:.3g})')
