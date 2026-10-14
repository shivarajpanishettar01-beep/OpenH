# Where the data goes

Everything the rig writes lives under **one root**: `~/data` for real collection (`session.root`) and
`~/data_sim` for simulation (`session.sim_root`). Both have the same folders, so simulated data can never
land in a real folder. The layout is defined once in `usrig_common/session.py` (`LAYOUT`, `data_dir`); no
tool has its own path setting.

```
~/data/                                      (~/data_sim/ has the same folders)
├── sessions/<session_id>/
│   ├── session.json                         registration, gates, pre-session result, plan lock, calibration refs
│   ├── plan_progress.json                   Stage 2 rows done / redo
│   ├── .ep_NNNN.record.log                  bag recorder log
│   └── ep_NNNN/
│       ├── ep_NNNN_0.mcap, metadata.yaml    the episode bag (all recorded topics)
│       ├── episode.json                     labels, outcome, micrus HDF5 path, provenance
│       ├── qc.json                          per-episode QC
│       ├── annotation.json                  phases, gestures, g_t (automatic + expert)
│       └── task.txt                         instruction
├── micrus/<session_id>/                     micrus --out: lossless HDF5 recordings, snapshots
├── waypoints/<phantom_id>.yaml              collection waypoints (phantom frame)
├── waypoints/<phantom_id>/<name>.png        each waypoint's reference ultrasound snapshot
├── paths/<phantom_id>.yaml                  Stage 1 paths (path_editor)
├── plans/plan_<phantom>_<rows>rows_seed<seed>.csv   Stage 2 plans (plan_generator)
├── presets/base_presets.yaml                test / transit poses, base frame (preset_check save)
├── phantoms/<phantom_id>.yaml               fiducials for register_phantom
├── calibration/                             handeye_<date>/ (images + poses), pivot_<tool>_<date>.json
└── checks/                                  test recordings (test1-test3 launch files, record:=true) and
                                             pipeline checks on synthetic data; never exported
```

| Written by | Goes to |
|---|---|
| `episode_manager` (bags, episode / QC / annotation files) | `sessions/<session_id>/` |
| micrus (`micrus.launch.py`, from `session_id`) | `micrus/<session_id>/` |
| `waypoint_teach`, `waypoint_verify` | `waypoints/` (real or sim by the session's `sim` flag) |
| `path_editor` | `paths/` (`--sim` for simulation) |
| `plan_generator generate` | `plans/` unless `-o` is given (`--sim`) |
| `preset_check save` | `presets/` (`--sim`) |
| `register_phantom` | reads `phantoms/`, writes `sessions/<id>/session.json` |
| `handeye_calib`, `pivot_calib` | `calibration/` (handeye: by the session's `sim` flag; `--sim`) |
| `presession_check`, `temporal_calib` | `sessions/<id>/session.json` |
| `test1_robot` / `test2_micrus_robot` / `test3_camera_robot` launch with `record:=true` | `checks/<test>_<date_time>/` |
| `export/pipeline_check.py` | `~/data_sim/checks/pipeline_check/` |
| `rig_measure --write`, `handeye --write-rig`, `pivot_calib --write`, `camera_autolock --write` | the config files in `src/usrig_bringup/config/` (values, not data) |
| exports (`stage_bags`, `bag_to_lerobot`, `openh_export`) | where you point `-o` / `--out` (e.g. `~/datasets`): derived data, rebuilt from `~/data` |

Backup: `rsync -a ~/data/ <BACKUP_DEST>/data/` copies everything. The disk check in QC covers the micrus
recordings too, because they are on the same root.

Data from before 14 Oct 2026 (sessions directly in the root, micrus recordings in `~/MicrUs`, `~/MicrUs_sim`):
```bash
python scripts/migrate_data_layout.py
```
shows the moves; add `--apply` to do them (it moves, never deletes or overwrites, and rewrites the HDF5 paths
in the moved episodes). Run on 14 Oct 2026 for the simulated P4 sessions. Other files in `~/MicrUs` (your own
micrus captures) are not touched.
