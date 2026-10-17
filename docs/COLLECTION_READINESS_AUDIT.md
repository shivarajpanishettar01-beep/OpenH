# Collection readiness audit (16 Oct 2026)

Run: us-eng `code-audit depth=deep` with five independent read-only auditors (recording, control and
teleop, presets and stages, export and labels, operator workflow), then a us-lab `pre-mortem` of the
data-collection plan and an evidence pass in which every critical / safety claim was re-read at the cited
line. Code at commit 630f4b0 (main).

## Status, 17 Oct 2026: P0, P1 and P2 fixed in software (see docs/REPORT_2026-10-17.md)
Every item below was fixed with a regression test that fails on the previous code (283 tests pass; export pipeline
check passes). Not yet verified on the real UR7e: protective-stop detection, registration push, Stage 2 loop.
Found and fixed during the fixes: the test suite ran on the rig's ROS domain; the recorder's readiness
subscriptions could crash episode_manager at Start (InvalidHandle); orphaned micrus-stop calls; a console
outcome race. Still open (minor): see the report, "Not done".

## Verdict: FIX FIRST

The code base collects and records data end to end in simulation, but it is **not yet efficient or safe
enough for real collection**. Seven issues must be fixed before the first real episode: two safety
issues, label errors that put fault data in the expert set, a QC bug that fails every camera episode
of the target length, and a console start-up loop that blocks every session. Seven more must be fixed
before Stage 2 plans run, and eight more before the validated release. None of them needs hardware to
fix; all can be tested in simulation.

The schedule's critical path is **hardware bring-up, not code**. As of today: RTDE is not enabled; 27 rig.yaml
values are null; no calibration has been done; the capture PC and the D415 are untested; this PC has 27 GB free.

Tags: **[V]** = re-read at the line by the lead in the evidence pass; **[A]** = reported by an auditor
with quoted evidence, not re-read by the lead; **[A+run]** = the auditor also reproduced it with a
scratch run.

## P0: before the first real episode

| # | Area | Finding | Where | Fix |
|---|---|---|---|---|
| 1 | Safety | **PROTECTIVE STOP in the console is refused while any other console action runs** (Run path, Go to, Verify, Descend). The button stays enabled; the only trace is a log line. | `operator_console.py:161-165` (`run_async`: `if self.busy: ... refused`), `:300-301` (`do_pstop` → `trigger` → `run_async`), `:1014-1015` (not in the busy list) **[V]** | Call protective_stop on its own thread, bypassing `busy`; never refuse it. Regression test with `busy` set. |
| 2 | Labels | **During a FAULT the last motion's labels persist**: samples keep `action_source` TELEOP / SCRIPTED and `action_is_expert=True`. `/robot/cmd_executed` also stops during FAULT (no NaN marker either). The deadman zero is labelled expert too. | `robot_control.py:466-474` (`_fault` sets mode and flags only) **[V]**; `:603-606` early return without `cmd_executed` **[V]**; deadman `:89-101` **[A]** | In `_fault` and stale / deadman ticks: source NONE, expert False; publish a zero or NaN `cmd_executed` every tick. |
| 3 | Safety / labels | **A controller protective stop or a not-RUNNING robot is never detected.** `robot_mode` is read but unused; the `speedL` / `forceMode` return values are ignored; there is no `isProgramRunning` / `reuploadScript`. | `backends.py:150-162` **[V]**; grep for `isProgramRunning\|reuploadScript\|getSafetyMode` finds nothing **[V]**; `rtde_reader.py:99` **[V]** | FAULT on robot_mode != RUNNING, on a safety stop, or on a False return; ack re-uploads the control script. Test with a fake backend. |
| 4 | Safety | **`/robot/inject_perturbation` runs without contact**: the probe goes into force mode in free air and is driven down along z until the force setpoint or the hard box. | `robot_control.py:762-773` (no contact / mode check) **[V]**; reproduced in sim **[A+run]** | Refuse unless the probe is in contact during an active or held teleop. |
| 5 | QC | **The depth rate check counts every depth message in the bag but divides by the start-to-end window.** The bag also holds the readiness tail, the 1.5 s free-air hold, the descent and the stop. A 25 s window with 1.7 s or more of extra recording reads at least 6.8 % fast, which fails the ±5 % band. **Every camera episode shorter than about 35 s fails QC**, and the Stage 0→1 and 1→2 gates can then never pass. The tests miss it (the camera is disabled in all QC tests). | `episode_qc.py:49-53` (`counts` = whole bag), `:233-235` (`hz = (n_all - 1) / ((t1 - t0) ...)`) **[V]** | Count only depth messages inside [t0, t1]. Also add per-stream coverage (first − t0, t1 − last) so a stream that dies mid-episode fails (R2). |
| 6 | Workflow | **A real session cannot start from the console.** The console's pre-session check never passes `--planned-hours`; the disk row is then UNSET, and UNSET counts as a fail. Separately, episode_manager exits at start-up when no passing check exists, but the check needs the launch's streams running. Every new session therefore needs launch → exit → check → Ctrl+C → relaunch. | `console_model.py:214-222` **[V]**; `presession_check.py:51, 242-244` **[V]**; `episode_manager.py:152-157` **[V]** | Add planned hours in the Session tab. Move the pre-session gate to the first Start, re-reading session.json, so the node stays up and reports the reason. |
| 7 | Data | **`tcp_phantom` is stale after registration.** `T_base_phantom` is read once as a launch parameter. Registering during a running session (the documented flow) leaves `/robot/state.tcp_phantom` equal to the base pose until relaunch, while robot_control already enforces the real phantom box. | `robot_state_cpp.cpp:185-189`, `robot.launch.py:49-61` **[A]** | Reload in robot_state_cpp on the same `reload_registration` command, or refuse Start until the registration is live. Publish a `phantom_registered` flag. |

## P1: before Stage 1/2 plan collection (wasted rig time otherwise)

| # | Finding | Where | Fix |
|---|---|---|---|
| 8 | **Plan progress is per session**: each new session (new id daily) restarts the plan at row 0. Early rows are duplicated and late rows are never reached. | `episode_manager.py:653` (`self.sdir / 'plan_progress.json'`) **[V]** | Key progress by plan SHA under `<root>/plans/`, or rebuild it from all sessions' `episode.json` `plan_row`. |
| 9 | **The plan mix is confounded and off target.** Target and failure mode come from two cyclic counters, so with 4 or 6 targets each staging method lands on one target. The type mix is hard-coded at 15 % failure + 30 % recovery (45 %), against the 20 % in session.yaml. | `plan_generator.py:37-38, 78, 92` **[V]** (the auditor cited lines 334/375/389, which don't exist); confounding reproduced **[A+run]** | Seeded shuffle stratified on (type, target); read the fractions from session.yaml; per-phantom allowed failure modes (FM-07 on the imaging-only phantom). |
| 10 | **The 1→2 gate blocks forever** after any single kept Stage 1 episode with a failed rate or gap check. No tool can exclude it afterwards. | `gates.py:53-61` **[V]** | Judge on the latest N episodes or a pass ratio; add a logged "exclude from gate" review. |
| 11 | **A finished episode can be lost.** A micrus stop error raises out of `_finish` before episode.json is written; the stage thread dies while the console still shows idle. Any non-Abort exception does the same. | `recorder.py:239-247`, `episode_manager.py:423, 498-509` **[A]** | Write episode.json first; catch per episode; publish `stage_dead` in `/episode/status`. |
| 12 | **Free disk space is a QC check on the already-recorded episode**, so good episodes are discarded below 50 GB. This PC has 27 GB. | `episode_qc.py:219-222` **[V]** | Check before Start; report free disk as info in QC. |
| 13 | **Operator dead time on every episode.** Bag stop, micrus stop, full QC (decodes every frame) and auto_annotate all run before `_safe_retract`, with the probe still in contact. | `episode_manager.py:409-453` **[A]** (the live sim run measured Stop → idle at 2.1 s for a 9 s episode without the camera) | Write episode.json → retract → QC and annotation in the background. |
| 14 | The next row's failure mode and staging method are not shown before N starts motion; the row's `operator_id` overrides the session operator; redo rows are not retried. | `episode_manager.py:666, 668, 713`; `episode_status.py:116-125` **[A]** | Show the next row and wait for "staged, go"; record the real operator; do a second pass over redo rows. |

## P2: before the validated release (31 Jan)

| # | Finding | Where | Fix |
|---|---|---|---|
| 15 | **The export ignores label provenance and QA.** Later entries overwrite earlier ones, and auto_annotate appends the automated entries after the human ones, so **automated labels override expert labels**, including QA-rejected ones. | `openh_fields.py:50` **[V]**; `annotation.py:125-128, 132-137` **[V]**; `auto_annotate.py:45` **[V]** | A resolver: drop rejected entries; priority expert > model_assisted > trainee > automated; write the label source per step. |
| 16 | The annotator saves the whole annotation.json, so concurrent raters overwrite each other. Raters see each other's segments, which biases kappa. Accepting an automated segment yields no agreement data. | `annotator.py:274-276, 354`; `label_agreement.py:164` **[A]** | One file per annotator; blind mode; count "accepted" as a rating; double-rate 10-20 %. |
| 17 | `failure_class` is never filled automatically (empty for every failure / recovery episode unless labelled by hand). | `auto_annotate.py:36-48`, `openh_export.py:263` **[A]** | Map plan FM id → class (failure_catalog.yaml); the export fails when a class is missing. |
| 18 | The export doesn't cross-check the parquet `episode_index` against the stage index or count. Video frames use one placeholder offset instead of each episode's measured offset, and the mismatch warning fires once per run. | `openh_export.py:136-149, 356-362` **[A]** | Assert index and count; make an offset mismatch an error. |
| 19 | Calibrations are exported from the exporting PC's rig.yaml / the first episode, not per session. The wrist matrix is T_flange_camera but labelled "w.r.t. end-effector". episode_manager overwrites the session's hand-eye matrix with rig.yaml on every start. | `openh_export.py:192, 245-253, 346-348`; `episode_manager.py:146` **[A]** | Snapshot calibrations per session; name frames exactly. |
| 20 | **Hour counting disagrees between tools**: progress counts every stage with paused time and pre-roll; stage_bags drops stages 0 and 1; the card counts steps. collection_plan can say ON TRACK while the export falls short. | `collection_progress.py:31-43`, `stage_bags.py:48-54` **[A]** | One `exportable(ep)` predicate used everywhere; count `duration_s − paused_s`. |
| 21 | Freedrive episodes recorded in a Stage 1 session are dropped. One episode without camera topics stops the whole conversion. | `stage_bags.py:48-54`; `bag_to_lerobot.py:317-319` **[A]** | Fix the filter order; skip and report. |
| 22 | Pre-roll, paused spans and the zero last action stay in the exported steps without a flag. | `stage_bags.py:46-56`, `openh_fields.py:227-240` **[A]** | `observation.meta.paused` / `action_valid`; or trim to [start, end]. |

Also reported, minor or unverified [A]:
- The onset key overwrites a disturbance onset (R6).
- The 20-30 s length is not enforced (R11).
- The demo GUI does not pause during test recordings (I9).
- Both PCs use `ROS_DOMAIN_ID=0` on the robot LAN (I10).
- No hours-vs-target in the console (I11).
- The "Robot may move" confirmation appears on every Start (M12).
- The phantom combo silently preselects a phantom; waypoints use session.yaml `phantom_id`, not the active session (I5, I6).
- [Hypothesis] Open-H may require lossless or 1-channel ultrasound; it is now h264 CRF 23 with grey copied into 3 channels.

Found correct by the auditors (no action): the pivot, Kabsch and hand-eye math; the force sign; unit conversions; the C++ quaternion and inverse; Python and C++ label rules matching; the deadman / watchdog / ack paths; the demo GUI has no publishers.

## Pre-mortem: "It is 1 Dec / 31 Jan and the dataset failed"

Volume: 10 recorded hours ≈ 1,440 episodes of 25 s; ≈ 31 rig hours at the planner's assumptions (yield 0.8,
recorded fraction 0.4) [`collection_plan`, run 16 Oct]. Likelihoods are not estimated (no base rate).

| Failure story | Impact | Early warning (signal · threshold · date) | Mitigate now | If triggered |
|---|---|---|---|---|
| Bring-up drags (RTDE, 27 limits, calibrations, first-motion checklist) and real collection starts in December | 1 Dec sample missed | test 1 `bag_check` PASS on the real arm · by **23 Oct**; `rig_setup --review` 0 null · by **30 Oct** | Do P0 fixes now in sim; enable RTDE; book rig days | Collect **freedrive** first (no limits needed) for the sample; teleop and presets later |
| Capture PC / D415 / disk not ready | Camera fields missing; QC fails; data lost | test 3 `bag_check` PASS + ≥ 300 GB free on the capture PC · by **6 Nov** | Fix P0-5 first; set up rtprio (`capture_pc_setup.md`) | Sample without depth (`--depth none`), camera added afterwards |
| Throughput far below plan (1,440 episodes) | Jan target missed | `collection_plan` after 10 real episodes: measured yield < 0.6 or recorded fraction < 0.3 · first collection week | P1-13 (QC after retract), P0-6 (one launch per session), one confirmation per session | Longer days on one phantom; freedrive / Stage 0 as the main source; drop Stage 1 repeats |
| Data looks fine but is wrong (fault samples labelled expert, stale `tcp_phantom`, offsets, labels overridden) | Invalid release, found late | Weekly sample export + `release_card` + manual review of 5 episodes · from **14 Nov** | P0-2, P0-7, P2-15 | Re-export (raw data kept); never relabel by hand at scale |
| Failure / recovery share or diversity not reached; failure mode confounded with target | Dataset fails the proposal's mix | `collection_progress` failure / recovery share < 15 % or a failure mode on only 1 target · **21 Nov** | P1-9 before generating plans | Targeted Stage 2 days per failure mode |
| Labelling bottleneck: 1,440 episodes need expert labels and agreement | Not "validated" by 31 Jan | Minutes of labelling per episode on the first 20 > 3 · **20 Nov** | P2-15/16/17: auto-labels + accept, double-rate 10-20 % | Validate a stratified subset; release the rest as auto-labelled with provenance |
| **Science:** the dataset is valid but not useful for learning (mostly static holds, kinesthetic actions not commandable, little view or approach diversity) | Weak contribution; poor GR00T-H fine-tuning | Release card action statistics (fraction of near-zero actions, per-gesture counts) on the sample · **1 Dec** | Plan a gesture / target / start-offset mix; flag paused and pre-roll steps | Re-weight the remaining collection toward the under-represented gestures and targets |
| External: Open-H format or validator expectations differ (lossless ultrasound, fields) | Rework at submission | Run Open-H's validator / guide on the first sample export · by **14 Nov** | Ask the Open-H team for the validator now | Adapt the export (raw data kept) |

**Robust actions** (they help in most stories): fix P0 in simulation now; get test 1 on the real arm this
week; collect the first 10 real episodes (freedrive or Stage 0) as early as possible to replace the
planner's assumptions with measured rates; export and review a sample every week from mid-November
instead of once at the deadline.

**Next check:** 23 Oct: test 1 on the real arm (`bag_check` all PASS, including F/T live) and the P0 list status.

## How to collect efficiently (summary)
1. Fix P0 (about a week of software in simulation, each fix with a regression test).
2. Bring-up order: RTDE → test 1 → test 2 → freedrive pedal → first 10 freedrive / Stage 0 episodes, which give measured rates.
3. Then the limits (`rig_setup`), the first-motion checklist, keyboard teleop, calibrations and registration, waypoints. Fix P1 before Stage 1/2.
4. One phantom per day (the planner does this); one launch per session; QC after retract.
5. Labels: auto-annotate, then experts accept or correct with the P2 resolver; double-rate 10-20 % for kappa.
6. Export a sample weekly from mid-November and read the release card.

Coverage: all 12 packages, export/, and the external bag_to_lerobot.py were read by at least one auditor.
Not inspected: the demo GUI widgets / kinematics, micrus internals, ur_rtde internals, Open-H's validator.
