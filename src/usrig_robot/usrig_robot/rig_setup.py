"""rig_setup: fill the values robot_control and keyboard teleop need, one by one, with checks.

  ros2 run usrig_robot rig_setup                      # every item, in order
  ros2 run usrig_robot rig_setup --section force      # robot | workspace | speeds | force | motion | keyboard
  ros2 run usrig_robot rig_setup --review             # no questions: show what is null and check what is set
  add --ip <controller> to measure from another address than rig.yaml robot.ip

For each item it shows the meaning, the unit, where the value comes from and the current value. You type
the value (Enter keeps the current one). Where the robot can measure something, `m` does it, read-only:
the workspace box from corners you touch, the tilt reference axis, the payload set on the controller, the
F/T noise in free air. **It never proposes or invents a safety value**: the numbers are yours.

Before writing, every value is checked against the others (force band inside the ceiling, key speeds below
the caps, thresholds above the measured noise, local cone inside the global one, ...). Errors block the
write; warnings are shown. The previous rig.yaml / keymap.yaml and a record of the changes go to
<root>/calibration/rig_setup_<date_time>/, then the values are written with every comment kept.
"""
import argparse
import json
import shutil
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from usrig_common import config

UR7E_MAX_PAYLOAD_KG = 7.5
CONTROL_PERIOD_S = 0.008


@dataclass
class Item:
    section: str
    file: str            # rig | keymap
    key: str             # dotted path in that file
    kind: str            # str float bool vec3 vec6 pair float_or_null
    unit: str
    meaning: str
    source: str
    measure: str = ''    # box_min box_max tilt_axis payload_kg payload_cog force_noise


ITEMS = [
    # ---- robot
    Item('robot', 'rig', 'robot.ip', 'str', '', 'UR7e controller address on the LAN', 'pendant: Settings > System > Network'),
    Item('robot', 'rig', 'robot.tcp_setting', 'vec6', 'm, rad', 'active TCP offset [x y z rx ry rz]; must be the PROBE FACE',
         'pendant: Installation > General > TCP (mm there: enter metres here)'),
    Item('robot', 'rig', 'probe.tcp_already_includes_probe', 'bool', '', 'is that TCP the probe face? robot_control needs yes (D4)',
         'yes if the TCP above points at the probe face'),
    Item('robot', 'rig', 'probe.type', 'str', '', 'probe model', 'e.g. MPRO-L40S'),
    Item('robot', 'rig', 'robot.payload_kg', 'float', 'kg', 'probe + holder + camera + handle mass, as set on the controller',
         'pendant: Installation > General > Payload (m reads it)', 'payload_kg'),
    Item('robot', 'rig', 'robot.payload_cog_m', 'vec3', 'm', 'payload centre of gravity [cx cy cz], flange frame',
         'pendant: Payload (m reads it)', 'payload_cog'),
    # ---- workspace
    Item('workspace', 'rig', 'limits.workspace_box_base_m.min', 'vec3', 'm', 'workspace box corner [x y z] min, base frame',
         'm: touch the extremes of the allowed volume with the probe tip; add your own margin', 'box_min'),
    Item('workspace', 'rig', 'limits.workspace_box_base_m.max', 'vec3', 'm', 'workspace box corner [x y z] max, base frame',
         'set together with min by m', 'box_max'),
    Item('workspace', 'rig', 'limits.tilt_reference_axis_base', 'vec3', 'unit vector', 'axis the tilt cone is measured from',
         'm: hold the probe square to the phantom', 'tilt_axis'),
    Item('workspace', 'rig', 'limits.tilt_cone_deg', 'float', 'deg', 'max angle between the probe axis and that reference',
         'your protocol'),
    Item('workspace', 'rig', 'limits.local_tilt_cone_deg', 'float', 'deg', 'cone around each waypoint axis (inside the global cone)',
         'your protocol'),
    # ---- speeds
    Item('speeds', 'rig', 'limits.max_linear_speed_mm_s', 'float', 'mm/s', 'cap on every commanded TCP speed', 'your protocol (start low)'),
    Item('speeds', 'rig', 'limits.max_angular_speed_deg_s', 'float', 'deg/s', 'cap on every commanded rotation speed', 'your protocol'),
    Item('speeds', 'rig', 'limits.max_linear_accel_mm_s2', 'float', 'mm/s2', 'cap on commanded acceleration', 'your protocol'),
    Item('speeds', 'rig', 'limits.max_angular_accel_deg_s2', 'float', 'deg/s2', 'cap on commanded angular acceleration', 'your protocol'),
    Item('speeds', 'rig', 'limits.stop_decel_mm_s2', 'float', 'mm/s2', 'deceleration of speedStop on FAULT / stop',
         'higher than the acceleration cap'),
    Item('speeds', 'rig', 'limits.deadman_timeout_s', 'float', 's', 'no teleop input this long -> zero velocity; also the controller watchdog',
         'a few control periods (8 ms) to a fraction of a second'),
    # ---- force
    Item('force', 'rig', 'limits.contact_detect_threshold_n', 'float', 'N', 'descent stops when the probe-axis force exceeds this',
         'above the free-air noise (m measures it)', 'force_noise'),
    Item('force', 'rig', 'limits.contact_lost_threshold_n', 'float', 'N', 'in contact: force below this ...', 'below the band low, above the noise'),
    Item('force', 'rig', 'limits.contact_lost_time_s', 'float', 's', '... for this long = contact lost (hold, event)', 'long enough to ignore brief dips'),
    Item('force', 'rig', 'limits.force_band_n', 'pair', 'N', '[low high] allowed force setpoint along the probe axis', 'your protocol'),
    Item('force', 'rig', 'limits.force_setpoint_default_n', 'float', 'N', 'setpoint after contact', 'inside the band'),
    Item('force', 'rig', 'limits.max_contact_force_n', 'float', 'N', 'hard ceiling on |F|: FAULT and stop above', 'phantom / probe limits'),
    Item('force', 'rig', 'rtde.force_mode_limits', 'vec6', 'm/s, m, rad',
         'forceMode type 2: [x y z rx ry rz]; on the force axis (z) max speed m/s, elsewhere allowed deviation',
         'your protocol; z below the linear speed cap'),
    Item('force', 'rig', 'rtde.force_mode_damping', 'float_or_null', '0-1', 'forceMode damping (null = controller default)', 'optional'),
    # ---- motion
    Item('motion', 'rig', 'motion.transit_speed_mm_s', 'float', 'mm/s', 'approach / transit speed', 'below the linear speed cap'),
    Item('motion', 'rig', 'motion.descent_speed_mm_s', 'float', 'mm/s', 'descent to contact', 'slow enough to stop on contact'),
    Item('motion', 'rig', 'motion.descent_max_travel_mm', 'float', 'mm', 'give up descending after this', 'approach offset (50 mm) + tolerance'),
    Item('motion', 'rig', 'motion.retract_distance_mm', 'float', 'mm', 'retract along the probe axis', 'clears the phantom'),
    Item('motion', 'rig', 'motion.transit_safe_height_mm', 'float_or_null', 'mm', 'phantom-frame height for moves between waypoints',
         'above the phantom + margin (needed for phantom-frame moves)'),
    # ---- policy (GR00T-H runs)
    Item('policy', 'rig', 'policy.input_timeout_s', 'float', 's', 'no policy target this long -> zero velocity',
         'a few policy periods (a 10 Hz policy: 0.3 s)'),
    Item('policy', 'rig', 'policy.max_target_jump_mm', 'float', 'mm', 'refuse a policy target this far from the probe',
         'a few times the distance the policy moves per step'),
    Item('policy', 'rig', 'policy.max_target_jump_deg', 'float', 'deg', 'same for rotation', 'a few times the rotation per step'),
    Item('policy', 'rig', 'policy.max_linear_speed_mm_s', 'float_or_null', 'mm/s', 'policy speed cap (null = the global cap)',
         'start lower than teleop'),
    Item('policy', 'rig', 'policy.max_angular_speed_deg_s', 'float_or_null', 'deg/s', 'policy rotation cap (null = the global cap)',
         'start lower than teleop'),
    # ---- keyboard
    Item('keyboard', 'keymap', 'speeds.slide_along_mm_s', 'float', 'mm/s', 'W/S slide along the array', 'below the linear speed cap'),
    Item('keyboard', 'keymap', 'speeds.slide_across_mm_s', 'float', 'mm/s', 'A/D slide across the array', 'below the linear speed cap'),
    Item('keyboard', 'keymap', 'speeds.rock_deg_s', 'float', 'deg/s', 'Q/E rock (about probe y)', 'below the angular speed cap'),
    Item('keyboard', 'keymap', 'speeds.tilt_deg_s', 'float', 'deg/s', 'R/F tilt (about probe x)', 'below the angular speed cap'),
    Item('keyboard', 'keymap', 'speeds.rotate_deg_s', 'float', 'deg/s', 'Z/C rotate (about the beam)', 'below the angular speed cap'),
    Item('keyboard', 'keymap', 'force_step_n', 'float', 'N', 'T/G force setpoint step', 'smaller than the force band width'),
]
SECTIONS = ['robot', 'workspace', 'speeds', 'force', 'motion', 'policy', 'keyboard']


# ------------------------------------------------------------------------------- parsing
def parse(kind, text):
    t = text.strip()
    if t.lower() == 'null':
        if kind in ('float_or_null', 'str'):
            return None
        raise ValueError('this value is required (null not allowed here)')
    if kind == 'str':
        return t
    if kind == 'bool':
        if t.lower() in ('y', 'yes', 'true', '1'):
            return True
        if t.lower() in ('n', 'no', 'false', '0'):
            return False
        raise ValueError('answer yes or no')
    if kind in ('float', 'float_or_null'):
        return float(t)
    n = {'vec3': 3, 'vec6': 6, 'pair': 2}[kind]
    parts = [x for x in t.replace('[', ' ').replace(']', ' ').replace(',', ' ').split() if x]
    if len(parts) != n:
        raise ValueError(f'{n} numbers needed, got {len(parts)}')
    return [float(x) for x in parts]


# ------------------------------------------------------------------------------- checks
def validate(rig, km, noise=None):
    """[(level, message)] with level ERROR or WARN; only values that are set are checked."""
    g = lambda k: config.get_path(rig, k)  # noqa: E731
    out = []

    def err(m):
        out.append(('ERROR', m))

    def warn(m):
        out.append(('WARN', m))

    tcp = g('robot.tcp_setting')
    if tcp is not None and max(abs(x) for x in tcp[:3]) > 0.5:
        err('robot.tcp_setting: offset > 0.5 m: entered in mm? (metres and radians here)')
    if g('probe.tcp_already_includes_probe') is False:
        err('probe.tcp_already_includes_probe must be yes: set the controller TCP to the probe face (robot_control refuses otherwise)')
    pk = g('robot.payload_kg')
    if pk is not None and not (0 < pk <= UR7E_MAX_PAYLOAD_KG):
        err(f'robot.payload_kg {pk}: must be in (0, {UR7E_MAX_PAYLOAD_KG}] kg (UR7e)')
    cog = g('robot.payload_cog_m')
    if cog is not None and max(abs(x) for x in cog) > 0.3:
        warn('robot.payload_cog_m: more than 0.3 m from the flange: entered in mm?')
    lo, hi = g('limits.workspace_box_base_m.min'), g('limits.workspace_box_base_m.max')
    if lo is not None and hi is not None:
        if any(a >= b for a, b in zip(lo, hi)):
            err('workspace box: every min must be below its max')
        elif max(b - a for a, b in zip(lo, hi)) > 2.0:
            warn('workspace box larger than 2 m on an axis: entered in mm? (metres here)')
    ax = g('limits.tilt_reference_axis_base')
    if ax is not None and abs(np.linalg.norm(ax) - 1.0) > 0.05:
        warn('limits.tilt_reference_axis_base is not a unit vector (it is normalised when used)')
    cone, local = g('limits.tilt_cone_deg'), g('limits.local_tilt_cone_deg')
    for k, v in (('tilt_cone_deg', cone), ('local_tilt_cone_deg', local)):
        if v is not None and not (0 < v < 90):
            err(f'limits.{k} must be in (0, 90) deg')
    if cone is not None and local is not None and local > cone:
        err('limits.local_tilt_cone_deg must not exceed limits.tilt_cone_deg')
    vmax, wmax = g('limits.max_linear_speed_mm_s'), g('limits.max_angular_speed_deg_s')
    amax, decel = g('limits.max_linear_accel_mm_s2'), g('limits.stop_decel_mm_s2')
    for k in ('max_linear_speed_mm_s', 'max_angular_speed_deg_s', 'max_linear_accel_mm_s2', 'max_angular_accel_deg_s2',
              'stop_decel_mm_s2', 'deadman_timeout_s', 'contact_lost_time_s', 'max_contact_force_n'):
        v = g(f'limits.{k}')
        if v is not None and v <= 0:
            err(f'limits.{k} must be > 0')
    if vmax is not None and vmax > 250:
        warn(f'limits.max_linear_speed_mm_s {vmax}: fast for scanning (UR collaborative TCP speed is 250 mm/s by default)')
    if amax is not None and decel is not None and decel < amax:
        warn('limits.stop_decel_mm_s2 below the acceleration cap: stops take longer than speed-ups')
    dm = g('limits.deadman_timeout_s')
    if dm is not None and dm < 2 * CONTROL_PERIOD_S:
        err(f'limits.deadman_timeout_s {dm}: shorter than two control periods ({2 * CONTROL_PERIOD_S} s)')
    if dm is not None and dm > 1.0:
        warn(f'limits.deadman_timeout_s {dm} s: the arm keeps moving this long after input stops')
    band, fsp, fmax = g('limits.force_band_n'), g('limits.force_setpoint_default_n'), g('limits.max_contact_force_n')
    if band is not None:
        if not (0 <= band[0] < band[1]):
            err('limits.force_band_n: need 0 <= low < high')
        if fmax is not None and band[1] >= fmax:
            err('limits.force_band_n high must be below limits.max_contact_force_n (else a normal setpoint FAULTs)')
        if fsp is not None and not (band[0] <= fsp <= band[1]):
            err('limits.force_setpoint_default_n must be inside limits.force_band_n')
    cd, cl = g('limits.contact_detect_threshold_n'), g('limits.contact_lost_threshold_n')
    if cd is not None and band is not None and cd >= band[1]:
        warn('limits.contact_detect_threshold_n at or above the band high: descent would press hard before stopping')
    if cl is not None and band is not None and cl >= band[0] and band[0] > 0:
        warn('limits.contact_lost_threshold_n at or above the band low: a normal low setpoint may read as contact lost')
    if cd is not None and cl is not None and cl > cd:
        warn('limits.contact_lost_threshold_n above contact_detect_threshold_n (no hysteresis)')
    if noise is not None:
        for k, v in (('contact_detect_threshold_n', cd), ('contact_lost_threshold_n', cl)):
            if v is not None and v < 3 * noise['axis_std_n']:
                warn(f'limits.{k} {v} N is below 3x the measured free-air noise ({noise["axis_std_n"]:.3f} N std)')
    fml = g('rtde.force_mode_limits')
    if fml is not None:
        if any(x <= 0 for x in fml):
            err('rtde.force_mode_limits: all six must be > 0')
        elif vmax is not None and fml[2] * 1000 > vmax:
            warn('rtde.force_mode_limits z (m/s) above the linear speed cap: force mode may move faster than the cap')
    damp = g('rtde.force_mode_damping')
    if damp is not None and not (0 <= damp <= 1):
        err('rtde.force_mode_damping must be in [0, 1]')
    for k in ('transit_speed_mm_s', 'descent_speed_mm_s'):
        v = g(f'motion.{k}')
        if v is not None and v <= 0:
            err(f'motion.{k} must be > 0')
        if v is not None and vmax is not None and v > vmax:
            warn(f'motion.{k} above the linear speed cap (it is capped to {vmax})')
    trav, appr = g('motion.descent_max_travel_mm'), g('motion.approach_offset_mm')
    if trav is not None and appr is not None and trav < appr:
        warn(f'motion.descent_max_travel_mm {trav} below the approach offset {appr} mm: descent may give up before the surface')
    for k in ('slide_along_mm_s', 'slide_across_mm_s'):
        v = config.get_path(km, f'speeds.{k}')
        if v is not None and vmax is not None and v > vmax:
            err(f'keymap speeds.{k} {v} above limits.max_linear_speed_mm_s {vmax} (keyboard_teleop refuses)')
    for k in ('rock_deg_s', 'tilt_deg_s', 'rotate_deg_s'):
        v = config.get_path(km, f'speeds.{k}')
        if v is not None and wmax is not None and v > wmax:
            err(f'keymap speeds.{k} {v} above limits.max_angular_speed_deg_s {wmax} (keyboard_teleop refuses)')
    step = config.get_path(km, 'force_step_n')
    if step is not None and band is not None and step >= band[1] - band[0]:
        warn('keymap force_step_n is not smaller than the force band width')
    return out


# ------------------------------------------------------------------------------- session
class Setup:

    def __init__(self, config_dir=None, ip=None, ask=input, connect=None):
        d = Path(config_dir) if config_dir else config.default_config_dir()
        # the real files (the installed config is a symlink into src/ with scripts/build.sh)
        self.files = {'rig': (d / 'rig.yaml').resolve(), 'keymap': (d / 'keymap.yaml').resolve()}
        if not config_dir and not all('/src/usrig_bringup/config/' in str(f) for f in self.files.values()):
            print('WARNING: these are installed copies, not src/usrig_bringup/config: a rebuild would overwrite '
                  'your values. Build with scripts/build.sh (symlink install) or pass --config-dir.')
        self.cfg = {k: config.load_yaml(p) for k, p in self.files.items()}
        self.orig = json.loads(json.dumps(self.cfg))
        self.ip, self.ask = ip, ask
        self._connect = connect
        self.r = None
        self.noise = None
        self.measured = {}

    def robot(self):
        if self.r is None:
            ip = self.ip or config.get_path(self.cfg['rig'], 'robot.ip')
            if not ip:
                raise RuntimeError('no robot address: set robot.ip first (section robot) or give --ip')
            if self._connect is None:
                from .rig_measure import connect
                self._connect = connect
            self.r = self._connect(ip)
        return self.r

    def set(self, item, value):
        cfg = self.cfg[item.file]
        node = cfg
        parts = item.key.split('.')
        for k in parts[:-1]:
            node = node.setdefault(k, {})
        node[parts[-1]] = value

    def measure(self, item):
        from . import rig_measure as rm
        r = self.robot()
        if item.measure in ('box_min', 'box_max'):
            lo, hi = rm.measure_box(r, self.ask)
            lo, hi = [round(float(x), 4) for x in lo], [round(float(x), 4) for x in hi]
            self.measured['workspace_box'] = {'min': lo, 'max': hi}
            self.set(next(i for i in ITEMS if i.measure == 'box_min'), lo)
            self.set(next(i for i in ITEMS if i.measure == 'box_max'), hi)
            print('  min and max both set (no margin added: edit them to add yours)')
            return lo if item.measure == 'box_min' else hi
        if item.measure == 'tilt_axis':
            z = [round(float(x), 4) for x in rm.measure_tilt_axis(r, self.ask)]
            self.measured['tilt_axis'] = z
            return z
        if item.measure in ('payload_kg', 'payload_cog'):
            kg, cog = float(r.getPayload()), [round(float(x), 4) for x in r.getPayloadCog()]
            self.measured['payload'] = {'kg': kg, 'cog_m': cog}
            print(f'  controller payload: {kg:.3f} kg, CoG {cog} m (what is set on the pendant)')
            self.set(next(i for i in ITEMS if i.measure == 'payload_kg'), round(kg, 3))
            self.set(next(i for i in ITEMS if i.measure == 'payload_cog'), cog)
            return round(kg, 3) if item.measure == 'payload_kg' else cog
        if item.measure == 'force_noise':
            self.noise = rm.measure_force_noise(r, 10.0, self.ask)
            self.measured['force_noise'] = self.noise
            print('  measured only: now type the threshold you choose')
            return None
        raise ValueError(f'no measurement for {item.key}')

    def run_item(self, item, i, n):
        cur = config.get_path(self.cfg[item.file], item.key)
        print(f'\n[{item.section} {i}/{n}] {item.key}' + (f'  ({item.unit})' if item.unit else ''))
        print(f'  {item.meaning}\n  from: {item.source}\n  now:  {cur}')
        opts = 'Enter = keep' + (', m = measure' if item.measure else '') + ', null, s = skip section, q = finish'
        while True:
            t = self.ask(f'  value ({opts}): ').strip()
            if t == '':
                return 'next'
            if t.lower() in ('q', 's'):
                return {'q': 'quit', 's': 'skip'}[t.lower()]
            if t.lower() == 'm' and item.measure:
                try:
                    v = self.measure(item)
                except (RuntimeError, ValueError, OSError) as e:
                    print(f'  could not measure: {e}')
                    continue
                if v is None:
                    continue
                self.set(item, v)
                print(f'  set {item.key} = {v}')
                return 'next'
            try:
                v = parse(item.kind, t)
            except ValueError as e:
                print(f'  {e}')
                continue
            self.set(item, v)
            return 'next'

    def changes(self):
        out = []
        for it in ITEMS:
            a = config.get_path(self.orig[it.file], it.key)
            b = config.get_path(self.cfg[it.file], it.key)
            if a != b:
                out.append((it, a, b))
        return out

    def write(self, ch, sim=False):
        from usrig_common.session import data_dir
        scfg = config.load_config('session', str(self.files['rig'].parent))
        rec = data_dir(scfg, 'calibration', sim) / f'rig_setup_{time.strftime("%Y%m%d_%H%M%S")}'
        rec.mkdir(parents=True, exist_ok=True)
        for k, p in self.files.items():
            shutil.copy2(p, rec / f'{p.name}.before')
        (rec / 'changes.json').write_text(json.dumps({
            'changes': [{'file': it.file, 'key': it.key, 'before': a, 'after': b} for it, a, b in ch],
            'measurements': self.measured, 'time': time.strftime('%Y-%m-%d %H:%M:%S')}, indent=2, default=float))
        for it, _, b in ch:
            config.write_yaml_path(self.files[it.file], it.key, b)
        for it, _, b in ch:                                  # read back what was written
            got = config.get_path(config.load_yaml(self.files[it.file]), it.key)
            if got != b:
                raise RuntimeError(f'{it.key}: wrote {b}, file now has {got}')
        return rec


def print_review(rig, km, noise=None):
    miss = config.find_unset(rig, config.MOTION_LIMITS)
    kmiss = [k for k, v in (km.get('speeds') or {}).items() if v is None] + \
        (['force_step_n'] if km.get('force_step_n') is None else [])
    print(f'\nrobot_control needs {len(config.MOTION_LIMITS)} rig.yaml values: {len(miss)} still null'
          + (f': {", ".join(miss)}' if miss else ''))
    print(f'keyboard teleop needs keymap.yaml speeds + force_step_n: {len(kmiss)} still null'
          + (f': {", ".join(kmiss)}' if kmiss else ''))
    if not km.get('device') and not km.get('allow_any_keyboard'):
        print('keyboard teleop also needs keymap.yaml device: the operator keyboard '
              '(ls /dev/input/by-id/ | grep event-kbd); with null it refuses to move the arm')
    issues = validate(rig, km, noise)
    for lvl, m in issues:
        print(f'  {lvl:5s} {m}')
    if not issues:
        print('  checks: no problems among the values that are set')
    return issues


def main(argv=None, ask=input, connect=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--section', choices=SECTIONS, nargs='+')
    ap.add_argument('--review', action='store_true', help='only show what is null and check what is set')
    ap.add_argument('--ip')
    ap.add_argument('--sim', action='store_true', help='keep the change record under session.sim_root')
    ap.add_argument('--config-dir')
    a = ap.parse_args(argv)
    s = Setup(a.config_dir, a.ip, ask, connect)
    print(f'rig.yaml:    {s.files["rig"]}\nkeymap.yaml: {s.files["keymap"]}')
    if a.review:
        issues = print_review(s.cfg['rig'], s.cfg['keymap'])
        return 1 if any(lvl == 'ERROR' for lvl, _ in issues) else 0
    print('Type each value (yours: nothing is suggested). Enter keeps the current value.')
    for sec in a.section or SECTIONS:
        items = [it for it in ITEMS if it.section == sec]
        print(f'\n========== {sec} ==========')
        stop = False
        for i, it in enumerate(items, 1):
            res = s.run_item(it, i, len(items))
            if res == 'skip':
                break
            if res == 'quit':
                stop = True
                break
        if stop:
            break
    try:
        if s.r is not None:
            s.r.disconnect()
    except Exception:  # noqa: BLE001
        pass
    ch = s.changes()
    issues = print_review(s.cfg['rig'], s.cfg['keymap'], s.noise)
    if not ch:
        print('\nnothing changed')
        return 0
    print('\nchanges:')
    for it, a_, b in ch:
        print(f'  {it.file}.yaml {it.key}: {a_} -> {b}')
    if any(lvl == 'ERROR' for lvl, _ in issues):
        print('\nNOT WRITTEN: fix the ERROR lines (run rig_setup again; your answers are not kept)')
        return 1
    if ask('\nwrite these values? type yes: ').strip().lower() != 'yes':
        print('not written')
        return 1
    rec = s.write(ch, a.sim)
    print(f'written. Previous files and the change record: {rec}')
    print('Next: ros2 run usrig_qc rig_status')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
