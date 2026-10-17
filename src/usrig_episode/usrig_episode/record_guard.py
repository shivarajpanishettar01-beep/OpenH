"""Stops a micrus recording left running by a dead episode_manager.

episode_manager starts this detached. It polls the manager's PID; when the manager is gone and
the lock file still exists (micrus recording was on), it calls /micrus/record false and removes
the lock. The bag recorder dies with the manager by itself (PR_SET_PDEATHSIG -> SIGINT).
"""
import argparse
import os
import subprocess
import time
from pathlib import Path


def stop_micrus_recording(service='/micrus/record', timeout=20):
    # coreutils timeout: the call ends by itself even if this process is killed (ros2 service call waits forever
    # for a service that is gone; orphaned calls used to pile up after a SIGKILLed episode_manager)
    return subprocess.run(['timeout', str(int(timeout)), 'ros2', 'service', 'call', service, 'std_srvs/srv/SetBool',
                           '{data: false}'], capture_output=True, text=True, timeout=timeout + 5)


def alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--parent-pid', type=int, required=True)
    ap.add_argument('--lock', required=True)
    ap.add_argument('--service', default='/micrus/record')
    a = ap.parse_args()
    lock = Path(a.lock)
    while alive(a.parent_pid):
        time.sleep(0.5)
    if lock.exists():
        try:
            r = stop_micrus_recording(a.service)
            (lock.parent / 'record_guard.log').write_text(
                f'{time.time_ns()} manager {a.parent_pid} died while recording; '
                f'/micrus/record false -> rc={r.returncode}\n{r.stdout}\n{r.stderr}\n')
        finally:
            lock.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
