"""Operator console logic without Qt or ROS (unit-tested): what each button may do in the current state,
the operator prompt, session ids and the collection launch commands.

The rules mirror what the nodes accept, so a button is never enabled for an action the rig would refuse,
and the operator sees why it is disabled.
"""
import datetime
import shlex
from pathlib import Path

MODES = ['IDLE', 'MOVING', 'DESCENT', 'TELEOP', 'PERTURB', 'RETRACT', 'FAULT', 'FREEDRIVE']
SOURCES = ['teleop', 'scripted', 'transit', 'perturbation', 'none', 'kinesthetic']
FLAGS = {1: 'envelope', 2: 'tilt cone', 4: 'force max', 8: 'deadman', 16: 'speed'}

STATE_TEXT = {'idle': 'Idle', 'starting': 'Starting (approach, bag readiness)', 'preparing': 'Preparing (free air, contact)',
              'recording': 'Recording', 'finishing': 'Finishing (stopping bag, QC)'}

# Collection modes offered on the Session tab -> collect.launch.py arguments.
COLLECTION_MODES = {
    'teleop':    {'label': 'Teleoperation (Stage 0)', 'control': True, 'teleop': True, 'episode_manager': True,
                  'stage': 0, 'input': None},
    'preset_s1': {'label': 'Preset paths (Stage 1)', 'control': True, 'teleop': True, 'episode_manager': True,
                  'stage': 1, 'input': None},
    'preset_s2': {'label': 'Planned episodes (Stage 2)', 'control': True, 'teleop': True, 'episode_manager': True,
                  'stage': 2, 'input': None},
    'freedrive': {'label': 'Freedrive by hand (pedal)', 'control': False, 'teleop': True, 'episode_manager': True,
                  'stage': 0, 'input': 'freedrive'},
    'setup':     {'label': 'Setup only: teach / verify waypoints (no episodes)', 'control': True, 'teleop': True,
                  'episode_manager': False, 'stage': None, 'input': None},
}


def fmt_s(s):
    """Seconds -> 'm:ss' or 'h:mm:ss'; None -> '-'."""
    if s is None:
        return '-'
    s = int(round(float(s)))
    h, rem = divmod(max(s, 0), 3600)
    m, sec = divmod(rem, 60)
    return f'{h}:{m:02d}:{sec:02d}' if h else f'{m}:{sec:02d}'


def flag_names(flags):
    return [n for b, n in FLAGS.items() if int(flags or 0) & b]


def mode_name(mode):
    return MODES[mode] if mode is not None and 0 <= int(mode) < len(MODES) else '-'


def source_name(src):
    return SOURCES[src] if src is not None and 0 <= int(src) < len(SOURCES) else '-'


class Ctx:
    """What the console knows right now (each field None when unknown / stale)."""

    def __init__(self, status=None, robot_mode=None, control_running=False, state_live=False):
        self.status = status or {}
        self.robot_mode = robot_mode            # from /robot/control_status or /robot/state (decimated)
        self.control_running = bool(control_running)
        self.state_live = bool(state_live)      # /robot/state arriving

    @property
    def manager(self):
        return bool(self.status)

    @property
    def ep_state(self):
        return self.status.get('state') if self.status else None

    @property
    def waiting(self):
        return set(self.status.get('waiting_for') or [])

    @property
    def episode(self):
        return self.status.get('episode') or None

    @property
    def paused(self):
        return bool(self.episode and self.episode.get('paused'))

    @property
    def fault(self):
        return self.robot_mode == MODES.index('FAULT')

    @property
    def session(self):
        return self.status.get('session') or {}


def allowed(action, c: Ctx):
    """(enabled, reason) for a console action."""
    no_mgr = (False, 'episode_manager is not running')
    if action == 'start_stop':
        if not c.manager:
            return no_mgr
        return ((True, '') if 'start_stop' in c.waiting else
                (False, 'not used now: this stage starts episodes itself' if c.ep_state == 'idle'
                 else 'waiting for something else (see the prompt)'))
    if action in ('outcome', 'discard', 'next'):
        if not c.manager:
            return no_mgr
        return (True, '') if action in c.waiting else (False, 'not expected now (see the prompt)')
    if action in ('pause', 'resume'):
        if not c.manager:
            return no_mgr
        if c.ep_state != 'recording':
            return False, 'only while recording'
        if action == 'pause' and c.session.get('stage') == 1 and c.session.get('collection_mode') == 'robot':
            return False, 'Stage 1 paths run to a fixed time budget: use Discard'
        if action == 'pause' and c.paused:
            return False, 'already paused'
        if action == 'resume' and not c.paused:
            return False, 'not paused'
        return True, ''
    if action in ('onset', 'recovery_start', 'recovery_end', 'view_reached'):
        if not c.manager:
            return no_mgr
        return (True, '') if c.ep_state == 'recording' else (False, 'only while recording')
    if action in ('note', 'gel'):
        if not c.manager:
            return no_mgr
        return (True, '') if c.episode else (False, 'no episode in progress')
    if action == 'protective_stop':
        return (True, '') if c.control_running else (False, 'robot_control is not running (use the pendant)')
    if action == 'ack_fault':
        if not c.control_running:
            return False, 'robot_control is not running'
        return (True, '') if c.fault else (False, 'no fault')
    if action in ('teleop_on', 'teleop_off', 'force_up', 'force_down', 'recontact', 'hold', 'zero_ft'):
        if not c.control_running:
            return False, 'robot_control is not running'
        if c.fault:
            return False, 'robot in FAULT: acknowledge first'
        return True, ''
    if action in ('goto', 'descend', 'retract', 'run_path', 'verify'):
        if not c.control_running:
            return False, 'robot_control is not running'
        if c.fault:
            return False, 'robot in FAULT: acknowledge first'
        if c.ep_state not in (None, 'idle'):
            return False, 'an episode is in progress: episode_manager owns the motion'
        return True, ''
    if action == 'teach':
        if not c.state_live:
            return False, 'no live /robot/state'
        if c.ep_state not in (None, 'idle'):
            return False, 'an episode is in progress'
        return True, ''
    return False, f'unknown action {action}'


def prompt(c: Ctx):
    """One line telling the operator what the rig waits for."""
    if not c.manager:
        return 'episode_manager not running: start a collection on the Session tab'
    if c.fault:
        return 'Robot in FAULT: check the cause, then Acknowledge fault'
    if c.session.get('stage_error'):
        return f'STOPPED: {c.session["stage_error"]}. Fix the cause, then restart the collection'
    st, w = c.ep_state, c.waiting
    if st == 'idle' and c.session.get('presession_problem'):
        return f'Blocked: {c.session["presession_problem"]}. Run the Pre-session check (Session tab), then Start'
    if c.paused:
        if c.session.get('robot_control'):
            return 'PAUSED: recording continues, robot holds. Resume to go on'
        # episode_manager sends the hold only with robot_control (freedrive: the arm is guided by hand)
        return 'PAUSED: recording continues, robot NOT held (no robot_control): release the pedal, keep the probe still. Resume to go on'
    if st == 'idle' and 'start_stop' in w and c.session.get('collection_mode') == 'freedrive' and not c.session.get('task'):
        return 'Set the task (target + instruction, Recording box) before Start: freedrive episodes need one'
    if st == 'idle' and 'start_stop' in w:
        return 'Ready: probe in free air, then Start episode'
    if st == 'idle' and 'next' in w:
        nr = (c.status.get('plan') or {}).get('next_row')
        if nr:
            stage = (f' · {nr["failure_mode"]} / {nr["staging_method"]}: stage it' if nr.get('failure_mode') else '')
            return f'Next: row {nr["row"]} {nr["type"]} · {nr["target"]}{stage}, then Next row'
        return 'Episode done: check the result, then Next row'
    if st == 'recording' and 'outcome' in w and 'start_stop' in w:
        return 'Recording: scan, set an outcome if needed, then Stop episode'
    if st == 'recording' and 'outcome' in w:
        return 'Recording: when done press Success / Recovered / Fail (or Discard)'
    if st == 'recording':
        return 'Recording: scripted motion running (Discard to abort)'
    return {'starting': 'Starting: moving to the approach pose / waiting for the bag',
            'preparing': 'Preparing: free-air hold, zero F/T, descending to contact',
            'finishing': 'Finishing: stopping the bag, running QC'}.get(st, STATE_TEXT.get(st, st or '-'))


def start_stop_label(c: Ctx):
    return 'Stop episode' if c.ep_state in ('recording', 'preparing', 'starting') else 'Start episode'


def make_session_id(pattern, phantom_id, operator_id, existing, date=None):
    """session.yaml id_pattern with the next free n for this date / phantom / operator."""
    date = date or datetime.date.today().strftime('%Y%m%d')
    n = 1
    while True:
        sid = pattern.format(date=date, phantom_id=phantom_id or 'phantom', operator_id=operator_id or 'op', n=n)
        if sid not in existing:
            return sid
        n += 1


def launch_command(mode, session_id, *, session_json, sim=False, camera=True, input_device='keyboard',
                   operator_id='', phantom_id=''):
    """argv for `ros2 launch usrig_bringup collect.launch.py ...` for a collection mode."""
    m = COLLECTION_MODES[mode]
    args = {'session_id': session_id, 'session_json': session_json, 'sim': sim, 'camera': camera,
            'control': m['control'], 'teleop': m['teleop'], 'episode_manager': m['episode_manager'],
            'input': m['input'] or input_device}
    if m['stage'] is not None:
        args['stage'] = m['stage']
    if operator_id:
        args['operator_id'] = operator_id
    if phantom_id:
        args['phantom_id'] = phantom_id
    fmt = lambda v: str(v).lower() if isinstance(v, bool) else str(v)  # noqa: E731
    return ['ros2', 'launch', 'usrig_bringup', 'collect.launch.py'] + [f'{k}:={fmt(v)}' for k, v in args.items()]


def presession_command(stage, session_dir, *, control, sim=False, camera=True, planned_hours=None):
    cmd = ['ros2', 'run', 'usrig_qc', 'presession_check', '--stage', str(stage), '--session', str(session_dir)]
    if planned_hours:
        cmd += ['--planned-hours', f'{float(planned_hours):g}']      # without it the disk row is UNSET (= fail)
    if control:
        cmd.append('--control')
    if sim:
        cmd.append('--sim')
    if not camera:
        cmd.append('--no-camera')
    return cmd


def terminal_command(argv, env_script, title='usrig', which=None):
    """Run argv in a new terminal window, detached from the console (the console must never be the parent of
    capture processes: closing it must not stop a recording). Returns argv for subprocess, or None if no
    terminal emulator is installed (the console then offers the command to copy)."""
    import shutil
    which = which or shutil.which
    inner = f'source {shlex.quote(str(env_script))} && {shlex.join(argv)}; echo; echo "[exited: $?]"; exec bash'
    if which('gnome-terminal'):
        term = ['gnome-terminal', f'--title={title}', '--', 'bash', '-lc', inner]
    elif which('xterm'):
        term = ['xterm', '-T', title, '-e', 'bash', '-lc', inner]
    elif which('x-terminal-emulator'):
        term = ['x-terminal-emulator', '-e', 'bash', '-lc', inner]
    else:
        return None
    return ['setsid', '-f'] + term


def env_script_path(repo_hint=None):
    """scripts/env.sh of this checkout (src/usrig_dashboard/usrig_dashboard -> repo root)."""
    if repo_hint:
        return Path(repo_hint) / 'scripts' / 'env.sh'
    here = Path(__file__).resolve()
    for p in here.parents:
        if (p / 'scripts' / 'env.sh').exists():
            return p / 'scripts' / 'env.sh'
    return Path('~/OpenH/scripts/env.sh').expanduser()


def waypoint_rows(waypoints, executable_problem=None, rig=None):
    """Display rows for the waypoint table: name, view, force, verified, NCC, status / reason."""
    rows = []
    for w in waypoints:
        v = w.get('verify') or {}
        problem = executable_problem(w, rig) if executable_problem and rig is not None else None
        rows.append({'name': w.get('name'), 'view': ' / '.join(x for x in (w.get('organ'), w.get('view')) if x),
                     'force_n': None if w.get('force_n') is None else round(abs(float(w['force_n'])), 2),
                     'verified': bool(w.get('verified')),
                     'ncc': None if v.get('ncc') is None else round(float(v['ncc']), 3),
                     'status': 'ready' if problem is None else problem})
    return rows
