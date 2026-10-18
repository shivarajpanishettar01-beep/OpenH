"""Which episodes the export takes and how much recorded time each counts: one rule for stage_bags,
collection_progress / collection_plan and (through the exported flags) the release card. Pure Python."""

FAILURE_RECOVERY_TYPES = ('recovery-start', 'recovery-mid', 'failure')
# reasons an episode is not exported; the first three never count anywhere, the stage / type ones are choices
NEVER = ('sim', 'synthetic')
REASONS = ('sim', 'synthetic', 'discard', 'qc_fail', 'stage0', 'stage1', 'kinesthetic')


def exportable(ep, include_stage0=False, include_stage1=False, include_kinesthetic=True, synthetic=False):
    """(True, '') or (False, reason in REASONS). Defaults = stage_bags defaults: Stage 0 / 1 left out (open
    decision 6), freedrive (kinesthetic) episodes kept whatever their stage. synthetic: pipeline-check runs only."""
    if ep.get('sim'):
        return False, 'sim'
    if bool(ep.get('synthetic_pipeline_check')) != bool(synthetic):
        return False, 'synthetic'
    if ep.get('outcome') == 'discard':
        return False, 'discard'
    if not (ep.get('qc') or {}).get('pass'):
        return False, 'qc_fail'
    if ep.get('type') == 'kinesthetic':
        return (True, '') if include_kinesthetic else (False, 'kinesthetic')
    # Stage 0 with an operator-set task is teleop collection (type 'teleop'); without one it is a shakedown test
    if ep.get('stage') == 0 and not include_stage0 and ep.get('type') != 'teleop':
        return False, 'stage0'
    if ep.get('stage') == 1 and not include_stage1:
        return False, 'stage1'
    return True, ''


def recorded_s(ep):
    """Episode time that counts: start to end event minus operator pauses (duration_s - paused_s)."""
    d = ep.get('duration_s')
    if d is None and ep.get('t_start_ns') is not None and ep.get('t_end_ns') is not None:
        d = (ep['t_end_ns'] - ep['t_start_ns']) * 1e-9
    return max(0.0, float(d or 0.0) - float(ep.get('paused_s') or 0.0))
