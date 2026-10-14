"""Loading session.yaml / rig.yaml / keymap.yaml and refusing to move with unset limits.

Every safety limit in rig.yaml stays ``null`` until it is filled from measurements.
Code that can move the robot calls :func:`require_limits` at start-up; it raises with the
dotted names of every limit that is still ``null``.
"""
import os
from pathlib import Path

import yaml


# Limits that must all be set before any node may command motion (dotted paths in rig.yaml).
MOTION_LIMITS = (
    'robot.ip',
    'robot.control_backend',
    'limits.max_contact_force_n',
    'limits.force_band_n',
    'limits.tilt_cone_deg',
    'limits.tilt_reference_axis_base',
    'limits.local_tilt_cone_deg',
    'limits.workspace_box_base_m.min',
    'limits.workspace_box_base_m.max',
    'limits.max_linear_speed_mm_s',
    'limits.max_angular_speed_deg_s',
    'limits.max_linear_accel_mm_s2',
    'limits.max_angular_accel_deg_s2',
    'limits.stop_decel_mm_s2',
    'robot.tcp_setting',
    'robot.payload_kg',
    'robot.payload_cog_m',
    'limits.deadman_timeout_s',
    'limits.contact_detect_threshold_n',
    'limits.contact_lost_threshold_n',
    'limits.contact_lost_time_s',
    'limits.force_setpoint_default_n',
    'motion.transit_speed_mm_s',
    'motion.descent_speed_mm_s',
    'motion.descent_max_travel_mm',
    'motion.retract_distance_mm',
    'probe.tcp_already_includes_probe',
    'rtde.force_mode_limits',          # RtdeBackend needs all 6; null used to crash robot_control at start
)


class UnsetLimitError(RuntimeError):
    """Raised when a node that can move the robot finds a limit still set to null."""

    def __init__(self, missing, source):
        self.missing = list(missing)
        super().__init__(
            f'Refusing to start: {len(self.missing)} limit(s) in {source} are null: '
            + ', '.join(self.missing)
            + '. Fill them from measurements first.')


def default_config_dir() -> Path:
    """Directory holding session.yaml, rig.yaml and keymap.yaml.

    $USRIG_CONFIG_DIR wins; otherwise the installed usrig_bringup share directory.
    """
    env = os.environ.get('USRIG_CONFIG_DIR')
    if env:
        return Path(env).expanduser()
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('usrig_bringup')) / 'config'


def load_yaml(path) -> dict:
    with open(Path(path).expanduser(), 'r') as f:
        return yaml.safe_load(f) or {}


# Settings that differ between PCs (data disk, scanner licence, camera present, PC-dependent latencies). Only
# these may be set in the machine file; anything else there (e.g. a safety limit) is refused, so no value
# that matters for safety can hide outside the shared rig.yaml.
MACHINE_KEYS = {
    'session': ('session.root', 'session.sim_root', 'micrus.command', 'micrus.licence_file', 'micrus.usb_vendor_id',
                'wrist_camera.enabled', 'recording.include_wrist_camera', 'recording.rt_priority'),
    'rig': ('robot.rt_priority', 'calibration.ultrasound_time_offset_s', 'calibration.camera_time_offset_s'),
}


class MachineConfigError(RuntimeError):
    pass


def machine_config_path() -> Path:
    """Per-PC settings outside the repository: $USRIG_MACHINE_CONFIG or ~/.config/usrig/machine.yaml."""
    return Path(os.environ.get('USRIG_MACHINE_CONFIG', '~/.config/usrig/machine.yaml')).expanduser()


def _flatten(d, prefix=''):
    out = {}
    for k, v in (d or {}).items():
        key = f'{prefix}{k}'
        if isinstance(v, dict) and v:
            out.update(_flatten(v, key + '.'))
        else:
            out[key] = v
    return out


def machine_overrides(name: str) -> dict:
    """{dotted key: value} of the machine file for one config (session / rig); {} if there is no file."""
    p = machine_config_path()
    if not p.exists():
        return {}
    data = load_yaml(p)
    unknown = set(data) - set(MACHINE_KEYS)
    if unknown:
        raise MachineConfigError(f'{p}: unknown sections {sorted(unknown)} (allowed: {sorted(MACHINE_KEYS)})')
    flat = _flatten(data.get(name) or {})
    bad = sorted(set(flat) - set(MACHINE_KEYS.get(name, ())))
    if bad:
        raise MachineConfigError(f'{p}: {bad} may not be set per machine (only {list(MACHINE_KEYS.get(name, ()))}); '
                                 f'set them in the shared {name}.yaml')
    return flat


def load_config(name: str, config_dir=None, machine=True) -> dict:
    """Load ``<config_dir>/<name>.yaml`` (name: session, rig, keymap), then the per-machine settings of
    ~/.config/usrig/machine.yaml on top (session and rig only, MACHINE_KEYS only). machine=False: the shared
    file alone (tools that edit it)."""
    d = Path(config_dir) if config_dir else default_config_dir()
    cfg = load_yaml(d / f'{name}.yaml')
    if machine:
        for dotted, value in machine_overrides(name).items():
            node = cfg
            *parents, last = dotted.split('.')
            for k in parents:
                node = node.setdefault(k, {})
            node[last] = value
    return cfg


def get_path(cfg: dict, dotted: str, default=None):
    cur = cfg
    for part in dotted.split('.'):
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur[part]
    return cur


def _is_unset(value) -> bool:
    if value is None:
        return True
    if isinstance(value, (list, tuple)):
        return len(value) == 0 or any(_is_unset(v) for v in value)
    if isinstance(value, dict):
        return any(_is_unset(v) for v in value.values())
    return False


def find_unset(cfg: dict, keys) -> list:
    """Dotted keys whose value is missing, null, or contains a null."""
    return [k for k in keys if _is_unset(get_path(cfg, k))]


def find_all_nulls(cfg, prefix='') -> list:
    """Every leaf in a nested dict/list that is null (for pre-session reports)."""
    out = []
    if isinstance(cfg, dict):
        for k, v in cfg.items():
            out += find_all_nulls(v, f'{prefix}{k}.')
    elif isinstance(cfg, list):
        for i, v in enumerate(cfg):
            out += find_all_nulls(v, f'{prefix}{i}.')
    elif cfg is None:
        out.append(prefix.rstrip('.'))
    return out


def set_thread_rt_priority(priority):
    """SCHED_FIFO for the calling thread. Returns (ok, message); never raises."""
    import os
    if not priority:
        return False, 'rt_priority 0: normal scheduling'
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(int(priority)))
        return True, f'SCHED_FIFO priority {priority}'
    except (PermissionError, OSError) as e:
        return False, (f'could not set SCHED_FIFO {priority} ({e}); running with normal scheduling. '
                       'Allow it with an rtprio limit (docs/capture_pc_setup.md)')


def require_limits(rig_cfg: dict, keys=MOTION_LIMITS, source='rig.yaml'):
    missing = find_unset(rig_cfg, keys)
    if missing:
        raise UnsetLimitError(missing, source)


def write_yaml_key(path, key, value):
    """Replace the value of the first `key:` line in a YAML file, keeping every comment.

    Lists, nested lists (4x4 matrices) and dicts are written in flow style on that one line.
    Used by the calibration and measurement tools to fill rig.yaml / session.yaml.
    """
    import re
    p = Path(path)
    text = p.read_text()
    flow = yaml.safe_dump(value, default_flow_style=True, width=10**6).strip()
    if flow.endswith('...'):
        flow = flow[:-3].strip()
    pat = re.compile(rf'^(\s*{re.escape(key)}:\s*)([^#\n]*?)(\s*(#.*)?)$', re.M)
    new, n = pat.subn(lambda m: m.group(1) + flow + (('   ' + m.group(4)) if m.group(4) else ''), text, count=1)
    if n != 1:
        raise KeyError(f'{key} not found in {path}')
    p.write_text(new)


def _split_comment(rest):
    """('value', '# comment' or '') for the text after 'key:' on one line."""
    import re
    m = re.search(r'(^|\s)#', rest)
    if not m:
        return rest.strip(), ''
    return rest[:m.start()].strip(), rest[m.start():].strip()


def write_yaml_path(path, dotted, value):
    """Set the value of a nested key given by its full dotted path (e.g. limits.workspace_box_base_m.min),
    keeping every comment and every other line as it is. Raises KeyError if the path is not in the file.
    Unlike write_yaml_key, keys with the same name in different sections (min / max / input) are never confused."""
    import re
    p = Path(path)
    lines = p.read_text().split('\n')
    target = dotted.split('.')
    flow = yaml.safe_dump(value, default_flow_style=True, width=10**6).strip()
    if flow.endswith('...'):
        flow = flow[:-3].strip()
    stack = []
    for i, line in enumerate(lines):
        if line.lstrip().startswith('#'):
            continue
        m = re.match(r'^(\s*)([A-Za-z0-9_]+):(.*)$', line)
        if not m:
            continue
        ind = len(m.group(1))
        while stack and stack[-1][0] >= ind:
            stack.pop()
        keys = [k for _, k in stack] + [m.group(2)]
        if keys == target:
            _, comment = _split_comment(m.group(3))
            new = f'{m.group(1)}{m.group(2)}: {flow}'
            if comment:
                new = new.ljust(max(len(new) + 3, 36)) + comment
            lines[i] = new
            p.write_text('\n'.join(lines))
            return
        stack.append((ind, m.group(2)))
    raise KeyError(f'{dotted} not found in {path}')


def expand(path: str, **fmt) -> Path:
    return Path(os.path.expandvars(str(path).format(**fmt))).expanduser()
