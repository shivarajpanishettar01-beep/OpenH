# Open decisions (recorded, not blocking)

1. **10 h commitment**: 10 h of recorded episode data? Do autonomous rollouts count? 400 × 25 s
   ≈ 2.8 h; 10 h ≈ 1,440 episodes. Decides whether episodes are chained in contact.
2. ~~Control path A or B~~: **decided 13 Oct 2026: path B** (ur_rtde forceMode + speedL, controller-side
   watchdog). URSim is used for a one-time control dry run before the first real motion and later as a
   GUI view only; it is never part of data collection.
3. **Action form for Open-H**: absolute or relative TCP. Bags hold both (TCP pose 500 Hz and the
   executed twist); `export/lerobot_rig.yaml` currently exports the executed probe-frame twist +
   force setpoint.
4. **`/micrus/beamformed` in the bags** (`session.yaml recording.record_beamformed: null`).
5. **FM-07 staging**: does the phantom have dense inserts for shadowing?
6. **Stage 1 data** in the training / released datasets, or pipeline check only
   (`stage_bags.py --include-stage1`).
7. **Depth**: does GR00T-H use wrist depth; does Open-H accept 16-bit depth? Depth is exported
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
