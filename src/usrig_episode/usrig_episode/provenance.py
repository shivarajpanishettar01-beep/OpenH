"""Provenance for episode.json: git commit, micrus version, RealSense versions."""
import os
import subprocess
from functools import lru_cache
from pathlib import Path


def _run(cmd, timeout=10):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.stdout.strip() if r.returncode == 0 else None
    except (OSError, subprocess.TimeoutExpired):
        return None


@lru_cache(maxsize=None)
def rig_git_commit():
    repo = os.environ.get('USRIG_REPO')
    if not repo:
        return None
    commit = _run(['git', '-C', repo, 'rev-parse', 'HEAD'])
    dirty = _run(['git', '-C', repo, 'status', '--porcelain'])
    return f'{commit}{"-dirty" if dirty else ""}' if commit else None


@lru_cache(maxsize=None)
def micrus_version(micrus_cmd='micrus'):
    out = _run([os.path.expanduser(micrus_cmd), '--version'])
    return out.splitlines()[-1] if out else None


@lru_cache(maxsize=None)
def deb_version(pkg):
    return _run(['dpkg-query', '-W', '-f=${Version}', pkg])


def realsense_versions():
    return {'librealsense2': deb_version('librealsense2') or deb_version('ros-humble-librealsense2'),
            'realsense2_camera': deb_version('ros-humble-realsense2-camera')}


@lru_cache(maxsize=None)
def camera_firmware(serial):
    """Firmware from `rs-enumerate-devices -s` (NOT VERIFIED output format)."""
    out = _run(['rs-enumerate-devices', '-s'])
    if not out or not serial:
        return None
    for line in out.splitlines():
        if str(serial) in line:
            parts = line.split()
            for p in parts:
                if p.count('.') == 3 and p.replace('.', '').isdigit():
                    return p
    return None


def free_disk_gb(path):
    import shutil
    p = Path(path).expanduser()
    while not p.exists():
        p = p.parent
    return shutil.disk_usage(p).free / 1e9
