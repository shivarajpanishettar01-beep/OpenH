"""Operator console rules (no Qt, no ROS)."""
from usrig_dashboard import console_model as cm

REC_S2 = {'state': 'recording', 'waiting_for': ['discard', 'outcome'],
          'episode': {'number': 7, 'paused': False}, 'session': {'robot_control': True}}
IDLE_S0 = {'state': 'idle', 'waiting_for': ['start_stop'], 'episode': None, 'session': {'robot_control': True}}


def ok(action, ctx):
    return cm.allowed(action, ctx)[0]


def test_episode_buttons_follow_what_episode_manager_waits_for():
    c = cm.Ctx(REC_S2, robot_mode=3, control_running=True)
    assert ok('outcome', c) and ok('discard', c) and ok('pause', c) and ok('onset', c)
    assert not ok('start_stop', c) and not ok('next', c) and not ok('resume', c)
    c = cm.Ctx(IDLE_S0, robot_mode=0, control_running=True)
    assert ok('start_stop', c) and cm.start_stop_label(c) == 'Start episode'
    assert not ok('outcome', c) and not ok('pause', c) and not ok('note', c)


def test_no_pause_in_stage1_paths():
    st = dict(REC_S2, session={'robot_control': True, 'stage': 1, 'collection_mode': 'robot'})
    ok_, why = cm.allowed('pause', cm.Ctx(st, robot_mode=1, control_running=True))
    assert not ok_ and 'Discard' in why


def test_pause_resume_toggle_and_prompt():
    st = dict(REC_S2, episode={'number': 7, 'paused': True})
    c = cm.Ctx(st, robot_mode=3, control_running=True)
    assert not ok('pause', c) and ok('resume', c)
    assert cm.prompt(c).startswith('PAUSED')


def test_pause_prompt_says_the_robot_holds_only_with_robot_control():
    """episode_manager sends the hold only with robot_control; without it (freedrive) nothing holds the arm."""
    st = dict(REC_S2, episode={'number': 7, 'paused': True})
    assert 'robot holds' in cm.prompt(cm.Ctx(st, robot_mode=3, control_running=True))
    st = dict(st, session=dict(st['session'], robot_control=False, collection_mode='freedrive'))
    p = cm.prompt(cm.Ctx(st, robot_mode=None, control_running=False))
    assert p.startswith('PAUSED') and 'robot holds' not in p and 'not held' in p.lower()


def test_manual_motion_only_when_idle_and_not_faulted():
    c = cm.Ctx(REC_S2, robot_mode=3, control_running=True)
    assert not ok('goto', c) and 'episode is in progress' in cm.allowed('goto', c)[1]
    c = cm.Ctx(IDLE_S0, robot_mode=0, control_running=True)
    assert ok('goto', c) and ok('verify', c) and ok('run_path', c)
    c = cm.Ctx(IDLE_S0, robot_mode=6, control_running=True)            # FAULT
    assert not ok('goto', c) and not ok('teleop_on', c) and ok('ack_fault', c)
    assert cm.prompt(c).startswith('Robot in FAULT')
    c = cm.Ctx(IDLE_S0, robot_mode=7, control_running=False)           # freedrive: no robot_control
    assert not ok('goto', c) and not ok('protective_stop', c) and not ok('ack_fault', c)
    assert cm.Ctx(None, control_running=True).manager is False
    assert ok('goto', cm.Ctx(None, robot_mode=0, control_running=True))   # setup mode: no episode_manager


def test_no_manager_disables_episode_actions():
    c = cm.Ctx(None)
    for a in ('start_stop', 'outcome', 'pause', 'onset', 'note'):
        assert not ok(a, c)
    assert 'episode_manager not running' in cm.prompt(c)


def test_session_id_and_commands():
    sid = cm.make_session_id('{date}_{phantom_id}_{operator_id}_s{n:02d}', 'ph', 'op1',
                             {'20261014_ph_op1_s01'}, date='20261014')
    assert sid == '20261014_ph_op1_s02'
    cmd = cm.launch_command('freedrive', sid, session_json='/d/s.json', input_device='keyboard')
    assert cmd[:4] == ['ros2', 'launch', 'usrig_bringup', 'collect.launch.py']
    assert 'control:=false' in cmd and 'input:=freedrive' in cmd and 'stage:=0' in cmd
    cmd = cm.launch_command('setup', sid, session_json='/d/s.json')
    assert 'episode_manager:=false' in cmd and not any(a.startswith('stage:=') for a in cmd)
    cmd = cm.launch_command('preset_s2', sid, session_json='/d/s.json', input_device='spacemouse', sim=True)
    assert 'stage:=2' in cmd and 'input:=spacemouse' in cmd and 'sim:=true' in cmd and 'control:=true' in cmd
    pc = cm.presession_command(2, '/d', control=True, camera=False)
    assert pc[-2:] == ['--control', '--no-camera']


def test_terminal_is_detached_and_absent_terminal_is_reported():
    t = cm.terminal_command(['ros2', 'launch', 'x'], '/repo/scripts/env.sh',
                            which=lambda n: '/usr/bin/gnome-terminal' if n == 'gnome-terminal' else None)
    assert t[:2] == ['setsid', '-f'] and 'gnome-terminal' in t and 'source /repo/scripts/env.sh' in t[-1]
    assert cm.terminal_command(['x'], '/e', which=lambda n: None) is None


def test_formatting():
    assert cm.fmt_s(None) == '-' and cm.fmt_s(5) == '0:05' and cm.fmt_s(3725) == '1:02:05'
    assert cm.flag_names(9) == ['envelope', 'deadman']
    assert cm.mode_name(6) == 'FAULT' and cm.source_name(5) == 'kinesthetic'
    rows = cm.waypoint_rows([{'name': 'a', 'organ': 'vein', 'view': 'long', 'force_n': -3.214, 'verified': True,
                              'verify': {'ncc': 0.81234}}], lambda w, r: None, {})
    assert rows[0] == {'name': 'a', 'view': 'vein / long', 'force_n': 3.21, 'verified': True, 'ncc': 0.812,
                       'status': 'ready'}


def test_presession_command_passes_planned_hours():
    """Without --planned-hours the disk row is UNSET and UNSET fails: the console's check never passed."""
    pc = cm.presession_command(0, '/d', control=False, planned_hours=2.5)
    assert pc[pc.index('--planned-hours') + 1] == '2.5'


def test_prompt_names_the_presession_problem():
    st = dict(IDLE_S0, session=dict(IDLE_S0['session'], presession_problem='no pre-session check for stage 0'))
    p = cm.prompt(cm.Ctx(st, robot_mode=0, control_running=False))
    assert 'pre-session' in p and 'Start' in p


def test_prompt_shows_the_next_plan_row_before_it_starts():
    """Failure mode and staging method were never shown: manual staging (gel removed, ...) was guesswork."""
    st = dict(IDLE_S0, waiting_for=['next'], plan={'rows': 40, 'next_row': {
        'row': 12, 'type': 'failure', 'target': 'IJV_long', 'failure_mode': 'FM-03', 'staging_method': 'gel_removed'}})
    p = cm.prompt(cm.Ctx(st, robot_mode=0, control_running=True))
    assert 'row 12' in p and 'FM-03' in p and 'gel_removed' in p
