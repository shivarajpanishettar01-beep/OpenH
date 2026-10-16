# Alignment with the submitted Open-H V2 proposal (IIT Mandi)

Source: `openh_v2 (5).pdf`. GR00T-H is out of scope for now (separate task).

| # | Proposal | Implementation | Status |
|---|---|---|---|
| 1 | Four failure classes (task-performance failure, observation degradation, target/view loss, safety-relevant interaction state) | `usrig_common/annotation.py FAILURE_CLASSES`; per-episode `failure_class` label, filled automatically from the plan FM id (`FM_FAILURE_CLASS`, automated provenance), corrected in the annotator; exported as `observation.meta.failure_class`; the export stops on a failure / recovery episode without a class | mapping done but **PROPOSED** (open decision 20): confirm with your failure-mode / safety protocol |
| 2 | Per-step flag g_t in {nominal, failure, recovery}, raised at view loss, held until reacquired | keys **O** (onset), **U** (recovery start), **I** (recovery end); injected perturbations also set onset. `auto_annotate` writes status intervals; export writes `observation.meta.status` per step | done, unit- and bag-tested |
| 3 | Recovery from seeded degraded states | plan start offsets + perturbations | classes from the FM id (item 1, PROPOSED mapping) |
| 4 | Action = incremental TCP pose [dx..drz], rotation vector | `openh_fields.delta_actions` (base or tool frame, `--action-frame`); raw executed twist and force setpoint kept as `observation.meta.commanded_twist` / `force_setpoint_n` | done, unit-tested |
| 5 | `observation.images.wrist_camera` and `observation.images.wrist_depth` | colour renamed in `export/lerobot_rig.yaml`; depth added losslessly as 16-bit PNG (mm) per step | done; LeRobot 0.6.0 handling of 16-bit PNG images NOT VERIFIED |
| 6 | `wrist_cali_mtx`, `meta/calibration/camera_intrinsics.json` | `observation.meta.wrist_cali_mtx`; `meta/calibration/{camera_intrinsics,wrist_cali_mtx,prob_cali_mtx}.json` from the bag's camera_info and calibrations | done |
| 7 | Vascular phantom ~6 h, central-line phantom ~4 h, 10 h total, ~20 % failure/recovery, 20-30 s episodes | `session.yaml phantoms/targets`; episode_manager accepts only those phantom ids; `ros2 run usrig_qc collection_progress` | done |
| 8 | phase -> step -> task -> gesture (slide, tilt/fan, rock, rotate, compress/decompress), motion quality 1-5, intent, outcome, narration, segmentation masks, provenance (expert/trainee/model-assisted/automated) + QA status | `<ep>/annotation.json` (schema + validator in `annotation.py`); automated phases, gestures, status, task, outcome, failure class after every episode; experts accept / correct in `usrig_dashboard annotator` (one `annotation.<annotator>.json` each, `--blind` for double rating); the export resolves overlaps (QA-rejected dropped, expert > model_assisted > trainee > automated) and writes the label source per step; per episode in `meta/annotations.jsonl` | done |
| 9 | Temporal offsets: ultrasound via tracked keypoint vs EE height (sinusoid fit); wrist camera via ArUco; re-checked at session start and end | `tools/temporal_calib.py`: `--aruco`, cross-correlation + sinusoid fit, two bags -> drift, `--session` stores the result | done, known-lag tests; not run on real data |
| 10 | Ultrasound is the reference clock | export aligns on the ultrasound stream | unchanged |

Size note: lossless depth at 640x480 is roughly 0.2-0.4 MB per step as 16-bit PNG, i.e. on the
order of 100-300 GB for 10 h at 25 fps (estimate, to be measured). Use `--depth none` for drafts.
