"""collection_plan: how many collection days are needed to reach the hours, and a day-by-day schedule.

  ros2 run usrig_qc collection_plan --by 2026-12-01 --hours-per-day 2.5 [--days mon tue wed thu fri]
      [--goal vascular_4vein=1.5 central_line_gen2=1.0] [--start 2026-10-20] [--blocked 2026-11-05 ...]
      [--yield 0.8] [--recorded-fraction 0.4] [--json plan.json]

Remaining recorded hours per phantom = goal (default: session.yaml `phantoms.<id>.target_hours`; --goal for an
intermediate milestone such as the 1 Dec sample) minus what collection_progress counts (real, kept, QC-passed
episodes). Turning recorded hours into hours at the rig needs two rates:
  yield              kept episodes / all real episodes (discards, QC failures)
  recorded fraction  recorded episode time / time at the rig in a session (setup, approach, retract, QC)
Both are MEASURED from your real sessions once there are at least 10 episodes; until then the defaults (or
your --yield / --recorded-fraction) are used and printed as ASSUMPTIONS. One phantom per day (one registration
per session); days go to the phantom with the most work left. Never touches the robot or the data.
"""
import argparse
import datetime as dt
import json
import math
from pathlib import Path

from usrig_common import config
from usrig_common.session import data_root, episode_files

from .collection_progress import progress

DEFAULT_YIELD, DEFAULT_RECORDED_FRACTION = 0.8, 0.4      # assumptions until measured (printed as such)
DAYS = ['mon', 'tue', 'wed', 'thu', 'fri', 'sat', 'sun']
MIN_EPISODES_TO_MEASURE = 10


def measured_rates(root):
    """(yield, recorded_fraction, n_episodes) from real episodes, or (None, None, n) when too few."""
    by_session, n_all, n_kept = {}, 0, 0
    for f in episode_files(root):
        e = json.loads(f.read_text())
        if e.get('sim') or e.get('synthetic_pipeline_check'):
            continue
        n_all += 1
        kept = e.get('outcome') != 'discard' and (e.get('qc') or {}).get('pass') is True
        n_kept += kept
        s = by_session.setdefault(f.parent.parent.name, {'t0': [], 't1': [], 'rec': 0.0})
        if e.get('t_start_ns') is not None and e.get('t_end_ns') is not None:
            s['t0'].append(e['t_start_ns'])
            s['t1'].append(e['t_end_ns'])
            if kept:
                s['rec'] += (e['t_end_ns'] - e['t_start_ns']) * 1e-9
    if n_all < MIN_EPISODES_TO_MEASURE:
        return None, None, n_all
    span = sum((max(s['t1']) - min(s['t0'])) * 1e-9 for s in by_session.values() if s['t0'])
    rec = sum(s['rec'] for s in by_session.values())
    return n_kept / n_all, (rec / span if span > 0 else None), n_all


def available_days(start, by, weekdays, blocked):
    days, d = [], start
    while d < by:
        if DAYS[d.weekday()] in weekdays and d not in blocked:
            days.append(d)
        d += dt.timedelta(days=1)
    return days


def plan(prog, goals, days, hours_per_day, yld, rec_frac, episode_s, fr_target):
    """Pure planning: remaining work per phantom, capacity, and a schedule (one phantom per day)."""
    remaining = {p: max(0.0, goals[p] - prog['phantoms'].get(p, {}).get('hours', 0.0)) for p in goals}
    rig_hours = {p: h / yld / rec_frac for p, h in remaining.items()}
    need = sum(rig_hours.values())
    cap = len(days) * hours_per_day
    per_day_rec = hours_per_day * rec_frac * yld                     # recorded (kept) hours per day
    left = dict(remaining)
    schedule = []
    for d in days:
        p = max(left, key=lambda k: left[k]) if left else None
        if p is None or left[p] <= 1e-9:
            break
        rec = min(per_day_rec, left[p])
        left[p] -= rec
        n_ep = math.ceil(rec * 3600 / episode_s)
        schedule.append({'date': d.isoformat(), 'phantom': p, 'recorded_h': rec, 'episodes': n_ep,
                         'failure_recovery_episodes': math.ceil(n_ep * fr_target)})
    unscheduled = {p: h for p, h in left.items() if h > 1e-9}
    return {'remaining_recorded_h': remaining, 'rig_hours_needed': rig_hours, 'rig_hours_total': need,
            'capacity_h': cap, 'days': len(days), 'on_track': need <= cap + 1e-9,
            'hours_per_day_needed': need / len(days) if days else float('inf'),
            'schedule': schedule, 'unscheduled_recorded_h': unscheduled}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--by', required=True, help='deadline date YYYY-MM-DD (not included)')
    ap.add_argument('--hours-per-day', type=float, required=True, help='hours at the rig on a collection day')
    ap.add_argument('--days', nargs='+', default=['mon', 'tue', 'wed', 'thu', 'fri'], choices=DAYS)
    ap.add_argument('--start', help='first possible day (default today)')
    ap.add_argument('--blocked', nargs='*', default=[], help='dates without collection')
    ap.add_argument('--goal', nargs='*', default=[], help='phantom=hours (default: session.yaml target_hours)')
    ap.add_argument('--yield', dest='yld', type=float, help=f'kept / all episodes (default {DEFAULT_YIELD} until measured)')
    ap.add_argument('--recorded-fraction', type=float,
                    help=f'recorded / rig time (default {DEFAULT_RECORDED_FRACTION} until measured)')
    ap.add_argument('--config-dir')
    ap.add_argument('--json')
    a = ap.parse_args(argv)
    scfg = config.load_config('session', a.config_dir)
    root = data_root(scfg, False)
    prog = progress(scfg, root)
    goals = {p: float((v or {}).get('target_hours') or 0) for p, v in (scfg.get('phantoms') or {}).items()}
    for g in a.goal:
        k, v = g.split('=')
        if k not in goals:
            raise SystemExit(f'unknown phantom {k} (session.yaml phantoms: {sorted(goals)})')
        goals[k] = float(v)
    m_yld, m_rec, n = measured_rates(root)
    yld = a.yld or m_yld or DEFAULT_YIELD
    rec = a.recorded_fraction or m_rec or DEFAULT_RECORDED_FRACTION
    src_y = 'given' if a.yld else ('MEASURED' if m_yld else 'ASSUMPTION')
    src_r = 'given' if a.recorded_fraction else ('MEASURED' if m_rec else 'ASSUMPTION')
    lo, hi = (scfg.get('targets') or {}).get('episode_s') or [20, 30]
    fr = float((scfg.get('targets') or {}).get('failure_recovery_fraction') or 0.2)
    start = dt.date.fromisoformat(a.start) if a.start else dt.date.today()
    by = dt.date.fromisoformat(a.by)
    days = available_days(start, by, set(a.days), {dt.date.fromisoformat(x) for x in a.blocked})
    r = plan(prog, goals, days, a.hours_per_day, yld, rec, (lo + hi) / 2.0, fr)

    print(f'data: {root}  ({n} real episodes so far)')
    print(f'yield {yld:.2f} [{src_y}]   recorded fraction {rec:.2f} [{src_r}]   episode {(lo + hi) / 2:g} s   '
          f'failure/recovery share {fr:.0%}')
    if 'ASSUMPTION' in (src_y, src_r):
        print(f'  assumptions are replaced by measured values after {MIN_EPISODES_TO_MEASURE} real episodes')
    print('\nphantom              done h   goal h   recorded h left   rig h needed')
    for p in goals:
        done = prog['phantoms'].get(p, {}).get('hours', 0.0)
        print(f'{p:20s} {done:6.2f}   {goals[p]:6.2f}   {r["remaining_recorded_h"][p]:15.2f}   '
              f'{r["rig_hours_needed"][p]:12.1f}')
    print(f'\n{r["days"]} collection days from {start} to {by} at {a.hours_per_day:g} h = {r["capacity_h"]:.1f} rig h; '
          f'needed {r["rig_hours_total"]:.1f} rig h -> '
          + ('ON TRACK' if r['on_track'] else f'SHORT by {r["rig_hours_total"] - r["capacity_h"]:.1f} h '
             f'(needs {r["hours_per_day_needed"]:.1f} h per day, or more days)'))
    if r['schedule']:
        print('\ndate        phantom              recorded   episodes (failure/recovery)')
        for s in r['schedule']:
            print(f'{s["date"]}  {s["phantom"]:20s} {s["recorded_h"] * 60:6.0f} min   {s["episodes"]:4d} '
                  f'({s["failure_recovery_episodes"]})')
    if r['unscheduled_recorded_h']:
        print(f'\nnot fitting before {by}: ' + ', '.join(f'{p} {h:.2f} h' for p, h in r['unscheduled_recorded_h'].items()))
    if a.json:
        Path(a.json).write_text(json.dumps({**r, 'yield': yld, 'yield_source': src_y, 'recorded_fraction': rec,
                                            'recorded_fraction_source': src_r, 'goals': goals}, indent=2))


if __name__ == '__main__':
    main()

