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
    'limits.workspace_box_base_m.min',
    'limits.workspace_box_base_m.max',
    'limits.max_linear_speed_mm_s',
    'limits.max_angular_speed_deg_s',
    'limits.max_linear_accel_mm_s2',
    'limits.max_angular_accel_deg_s2',
    'limits.deadman_timeout_s',
    'limits.contact_detect_threshold_n',
    'limits.force_setpoint_default_n',
    'motion.transit_speed_mm_s',
    'motion.descent_speed_mm_s',
    'motion.descent_max_travel_mm',
    'motion.retract_distance_mm',
    'probe.tcp_already_includes_probe',
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


def load_config(name: str, config_dir=None) -> dict:
    """Load ``<config_dir>/<name>.yaml`` (name: session, rig, keymap)."""
    d = Path(config_dir) if config_dir else default_config_dir()
    return load_yaml(d / f'{name}.yaml')


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


def require_limits(rig_cfg: dict, keys=MOTION_LIMITS, source='rig.yaml'):
    missing = find_unset(rig_cfg, keys)
    if missing:
        raise UnsetLimitError(missing, source)


def expand(path: str, **fmt) -> Path:
    return Path(os.path.expandvars(str(path).format(**fmt))).expanduser()
