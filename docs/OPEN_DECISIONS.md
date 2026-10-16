# Open decisions (recorded, not blocking)

1. **10 h commitment**: 10 h of recorded episode data? Do autonomous rollouts count? 400 × 25 s
   ≈ 2.8 h; 10 h ≈ 1,440 episodes. Decides whether episodes are chained in contact.
2. ~~Control path A or B~~: **decided 13 Oct 2026: path B** (ur_rtde forceMode + speedL, controller-side
   watchdog). The URSim control dry run was **skipped** (13 Oct 2026): motion is checked on the real arm in
   free air (`first_motion_checklist.md` C-D). URSim is never part of data collection.
3. ~~Action form~~: proposal fixes it: incremental TCP pose [dx..drz] (rotation vector);
   base vs tool frame selectable at export (`--action-frame`, default base). Was: absolute or relative TCP. Bags hold both (TCP pose 500 Hz and the
   executed twist); `export/lerobot_rig.yaml` currently exports the executed probe-frame twist +
   force setpoint.
4. **`/micrus/beamformed` in the bags** (`session.yaml recording.record_beamformed: null`).
5. **FM-07 staging**: does the phantom have dense inserts for shadowing?
6. **Stage 1 data** in the training / released datasets, or pipeline check only
   (`stage_bags.py --include-stage1`).
7. **Depth**: committed in the proposal as `observation.images.wrist_depth`; exported losslessly as
   16-bit PNG (large). Was: does Open-H accept 16-bit depth? Depth is exported
   losslessly to `depth/*.npz` (the converter would reduce it to 8 bits).
8. **Gate values**: `gate01.*`, `gate12.*` in `session.yaml`.
9. **D415 mount**: move it back or accept colour only if depth is mostly invalid at contact.
10. **Disk**: 39 GB free on `/`, below the 50 GB QC floor. A data disk is needed (P0).
11. **Expert flag vs scripted motion**: section 2.3 sets `action_is_expert=true` for
    `scripted_task`, but the QC table says "no expert=true during scripted motion". The QC
    implements 2.3 (expert only for teleop and scripted_task; never for transit / perturbation).
12. **Staging methods and disturbance magnitudes** in `failure_catalog.yaml` (placeholders).
13. **Teleop axes**: rock = rotation about probe y (in-plane), tilt = about probe x (out of
    plane); swap in `keymap.yaml` if the lab uses the opposite convention.
14. **Fans in Stage 1**: path speed counts rotation as the motion of a point
    `motion.path_rotation_radius_mm` (50 mm) from the tip; confirm or change.
15. **Envelope behaviour**: commands that would leave the box / tilt cone are clipped (flag
    set, no FAULT); an actual violation is a FAULT that needs acknowledgement, after which only
    motion back inside is allowed. Confirm this is what you want.
16. **Late control ticks per episode** (`rig.yaml qc.max_late_tick_frac`, null): share of ticks later than the
    speedL lifetime that an episode may have before QC fails it. Choose after measuring jitter on the capture PC.
17. **Depth encoding in LeRobot 0.6.0**: the export writes `observation.images.wrist_depth` as 16-bit PNG
    images (mm), verified lossless through the v3.0 conversion and reload (`pipeline_check.py`). LeRobot 0.6.0
    also has native depth options (`depth_encoder`, `depth_output_unit`) that may suit Open-H better; check
    with the Open-H guide before the first release.
18. **F/T offset in freedrive episodes**: no `zero_ft` service without robot_control. Each episode records a
    free-air window (`free_air` phase); subtract its mean force in the export, or rely on zeroing on the
    pendant at the start of the session? (Applies to robot episodes too if the offset drifts.)
19. **Paused spans in the export** (operator console Pause, 14 Oct 2026): recording continues while paused (a bag
    hole would not line up with the micrus HDF5 and QC would fail the episode); the robot holds, samples are
    labelled not expert, and `episode.json` has `pauses` / `paused_s`. Drop those spans in `openh_export.py`, or
    keep them as non-expert steps? Not implemented in the exporter yet.
20. **FM id -> failure class** (proposal section 6): `failure_catalog.yaml` has no class field, so
    `annotation.FM_FAILURE_CLASS` holds a PROPOSED mapping: FM-01 coupling loss and FM-07 shadowing ->
    observation_degradation; FM-02 lift-off and FM-03 force -> safety_relevant_interaction_state; FM-05 out of
    plane and FM-06 wrong view -> target_view_loss; FM-12 wrong instruction -> task_performance_failure.
    `auto_annotate` writes it as an automated label on failure / recovery episodes; a `failure_class` key on a
    catalog entry overrides the table; an annotator's label wins. Confirm with the failure-mode / safety protocol.
