# H2 training experiments: handoff (2026-11-14)

Read first: `~/humanoidx/PROJECT.md` (project map), `EXPERIMENTS.md` (what each experiment
changes and why), `EVAL_HANDOFF.md` (the Gazebo eval suite). This file covers the experiment
runs of 2026-11-10 to 2026-11-14: what ran, what came out, what is waiting on the user, and how
to run the next one.

## State right now

- Nothing is running. hermes's queue finished 2026-11-14 02:12 (Exp5b last). Every run has a
  Gazebo eval. No waiters are running on the laptop, and the sleep inhibitor is stopped.
- Code: top repo `9a07dbf`, `unitree_rl_lab` submodule `c994c35`, on both the laptop and
  hermes's shared checkout.
- Docs not yet updated with these results (only the user asks for them):
  - the `EXPERIMENTS.md` file-table statuses (Exp1, 2, 7, 10, 11, 5a and 5b still say
    "written");
  - `EXPERIMENTS.md` §4 "Record of runs" (empty);
  - `PROJECT.md` §6.8.
- Scripts and saved wandb histories are in `~/humanoidx/h2_pipeline/` (laptop only, not in git).
  See "Tools" below.

## Baseline history

The tracking reward is exp(−err²/σ²); σ_lin and σ_ang are its widths for speed and turn rate.
Each tracking σ is both paid (the reward the robot gets) and scoring (what the curricula judge);
the baseline needs paid = scoring.

| Version | Change | wandb | Gazebo eval |
| --- | --- | --- | --- |
| v1 | σ_ang 0.5, turn-curriculum bar 0.8. Turn range stuck at 0.1 | `r3yy2pa3` | `misc-policies/baseline_seed42_v2-16800/eval/default_20261110-012025` |
| v2 = Exp12 | σ_ang 1.0 | `crjat5kz` (trained on misc) | `misc-policies/exp12_ang_std_1_seed42-19999/eval/default_20261110-122507` |
| **v3 = Exp14 (current)** | σ_ang 0.5 paid and scoring, turn bar 0.4 (`threshold` param on `ang_vel_cmd_levels_fixed_std`) | `vfwoqdqw` | `misc-policies/exp14_ang_std_05_bar_04_seed42-19999/eval/default_20261111-183653` |

All `misc-policies` paths are under `~/humanoidx/`. Every experiment from Exp6-rerun on ran
against v3, for 20k iterations, on hermes, with seed 42.

## Results

### Gazebo

Suite `default`, hash `7d04666ca16b`, 3 repeats, final checkpoint unless marked. Produced by
`h2_pipeline/table_evals.py`. Steady-state RMS errors; lower is better except push survival.

| Run | fwd err | side err | turn err | heading ° | stand drift m | pitch ° | push surv | falls |
|---|---|---|---|---|---|---|---|---|
| 09-17-v2 (Sept 17 policy, `q3gjl7b5`) | 0.114 | 0.134 | 0.279 | 5.2 | 0.077 | 3.4 | 0.93 | 1 |
| Baseline v1 (16.8k) | 0.098 | 0.093 | 0.251 | 8.4 | 0.051 | 1.9 | 0.89 | 1 |
| Baseline v2 = Exp12 | 0.091 | 0.077 | 0.250 | 7.1 | 0.054 | 2.0 | 0.94 | 1 |
| Exp6 zero axes, vs v2 (`okblyzcb`) | 0.101 | 0.091 | 0.287 | 8.6 | 0.042 | 1.7 | 1.00 | 0 |
| **Baseline v3 = Exp14 (current)** | 0.091 | 0.082 | 0.175 | 3.3 | 0.051 | 2.4 | 0.90 | 1 |
| Exp11 alive 2.0, 17k (`nvo85y0i`) | 0.089 | 0.086 | 0.191 | 4.2 | 0.048 | 1.6 | 1.00 | 0 |
| Exp6 zero axes, rerun (`0aqlhamh`) | 0.096 | 0.094 | 0.187 | 4.6 | 0.043 | 1.5 | 1.00 | 0 |
| Exp1 gait reward (`mk95ciuc`) | 0.106 | 0.103 | 0.285 | 7.0 | 0.063 | 3.4 | 1.00 | 0 |
| Exp2 gait reward + phase (`g6a8xbby`) | 0.120 | 0.168 | 0.310 | 7.5 | 0.067 | 2.7 | 0.82 | 2 |
| Exp5 σ curriculum, broken (`k379ec06`) | 0.056 | 0.057 | 0.338 | 10.6 | 0.048 | 2.4 | 1.00 | 0 |
| Exp7 domain randomization (`pn9k8m4n`) | 0.090 | 0.097 | 0.186 | 3.8 | 0.047 | 1.6 | 1.00 | 0 |
| Exp10 pose (`361aauqy`) | 0.095 | 0.083 | 0.188 | 3.0 | 0.046 | 2.5 | 1.00 | 0 |
| Exp5a σ curriculum, all-resets scoring (`3zkrigi8`) | 0.064 | 0.056 | 0.293 | 8.9 | 0.051 | 2.6 | 1.00 | 0 |
| Exp5b all curricula, all-resets scoring (`k8sn5vox`) | 0.067 | 0.057 | 0.281 | 9.1 | 0.046 | 2.2 | 1.00 | 0 |
| Exp12 at 11.5k (mid-run) | 0.097 | 0.096 | 0.300 | 9.3 | 0.059 | 2.0 | 1.00 | 0 |
| Exp6 (vs v2) at 11.5k (mid-run) | 0.117 | 0.085 | 0.298 | 9.2 | 0.051 | 1.7 | 1.00 | 0 |

Turn rate achieved, rad/s. The commanded rate is in the header. half_square turns in place right
after walking.

| Run | spin +1.0 | spin −1.0 | half_square +0.5 | walk +0.4 | walk −0.4 | walk +0.3 | strafe −0.6 |
|---|---|---|---|---|---|---|---|
| 09-17-v2 | +0.90 | −0.94 | +0.38 | +0.32 | −0.37 | +0.18 | −0.47 |
| Baseline v1 (16.8k) | +0.39 | −0.33 | +0.04 | +0.25 | −0.28 | +0.17 | −0.10 |
| Baseline v2 = Exp12 | +0.71 | −0.11 | +0.17 | +0.31 | −0.30 | +0.23 | −0.28 |
| Exp6 zero axes, vs v2 | +0.15 | −0.19 | −0.00 | +0.32 | −0.32 | +0.24 | −0.33 |
| **Baseline v3 = Exp14 (current)** | +0.90 | −0.93 | +0.31 | +0.34 | −0.41 | +0.23 | −0.58 |
| Exp11 alive 2.0 (17k) | +0.89 | −0.88 | +0.15 | +0.38 | −0.37 | +0.26 | −0.60 |
| Exp6 zero axes, rerun | +0.91 | −0.93 | +0.15 | +0.34 | −0.42 | +0.23 | −0.54 |
| Exp1 gait reward | +0.85 | −0.79 | +0.14 | +0.30 | −0.38 | +0.21 | −0.41 |
| Exp2 gait reward + phase | +0.62 | −0.66 | +0.32 | +0.31 | −0.27 | +0.25 | −0.41 |
| Exp5 σ curriculum (broken) | +0.00 | −0.01 | +0.01 | +0.18 | −0.21 | +0.13 | −0.37 |
| Exp7 domain randomization | +0.90 | −0.88 | +0.23 | +0.38 | −0.37 | +0.30 | −0.50 |
| Exp10 pose | +0.96 | −0.96 | +0.32 | +0.37 | −0.39 | +0.29 | −0.58 |
| Exp5a | +0.02 | −0.03 | +0.15 | +0.32 | −0.30 | +0.20 | −0.37 |
| Exp5b | +0.29 | −0.27 | +0.07 | +0.29 | −0.31 | +0.21 | −0.38 |
| Exp12 at 11.5k (mid-run) | +0.27 | −0.13 | +0.03 | +0.29 | −0.33 | +0.19 | −0.11 |
| Exp6 (vs v2) at 11.5k (mid-run) | +0.23 | −0.15 | +0.03 | +0.28 | −0.27 | +0.22 | −0.28 |

### Training errors at 19–20k

From wandb `Metrics/base_velocity/error_vel_xy` and `error_vel_yaw`.

| Run | xy | yaw | Note |
|---|---|---|---|
| Baseline v3 | 0.412 | 0.738 | full-length episodes 5.5k, turn range maxed 6.5k, speed range maxed 8.2k |
| Exp11 | 0.410 | 0.742 | milestones 1–2k sooner, same plateau; both level off ~17k |
| Exp6 rerun | 0.400 | 0.748 | its errors include easier zeroed commands |
| Exp1 | 0.474 | 0.994 | errors stall after 9k; yaw rises after 17k |
| Exp2 | 0.513 | 1.138 | |
| Exp5 | 0.307 | 1.471 | curricula froze, then burst (below) |
| Exp7 | 0.438 | 0.768 | |
| Exp10 | 0.374 | 0.700 | best overall |
| Exp5a | 0.322 | 1.197 | final σ_lin 0.25, σ_ang 0.30 |
| Exp5b | 0.326 | 1.212 | curves ~identical to 5a |

### Decisions made

- **Exp6** (zero axes):
  - vs v2, it failed the agreed fold criteria and was not folded.
  - The rerun on v3 was a normal experiment: about baseline, except half_square 0.15.
- **Exp11** (alive 2.0): not folded. It was judged only on training speed. The lead is gone
  after ~15k, both runs plateau at ~17k, and half_square regressed (0.15).
- **Exp1, 2, 5, 7, 10, 5a, 5b**: experiments, reported, never folded (user's rule).
- **Exp10** is the best experiment (best training errors, spins 0.96, heading 3.0°). Exp7 is
  close. Exp1, 2 and 5 are worse.
- The current baseline beats 09-17-v2 on every tracking metric; spins are equal. 09-17-v2 is
  better only on half_square (0.38 vs 0.31).

## Findings worth knowing

1. **Curriculum check-step flaw.**
   - Every curriculum acts only when `env.common_step_counter % env.max_episode_length == 0`:
     every 1000 steps, about 41.7 iterations.
   - Isaac Lab computes curricula in `_reset_idx`, so a check scores only robots resetting on
     that exact step.
   - With `init_at_random_ep_len=True` and no falls, each robot resets on a fixed phase. About 4
     robots then decide every check (Poisson 4.1; none 1.7% of the time).
   - In Exp5 all four curricula froze from 6.6k to 15.9k. One robot fell at 15833, and the
     checks at 15874, 15916 and 15958 each fired (σ_ang 0.45 → 0.30, later 0.25).
   - **The fix:** `_ScoreSinceLastCheck` and three `*_all_resets` classes in
     `mdp/curriculums.py`. They score every robot reset since the last check. The old
     functions are unchanged.
   - The fix works: in 5a the σ curricula moved gradually.
   - The speed and turn curricula did not stall in 5a even with the old scoring, and 5b (new
     scoring on them too) trained the same.
   - The user's earlier plan was "if the baseline curriculum changes, everything reruns". The
     data so far says the command curricula don't need the change; the user hasn't decided.
   - Milestone iterations in all runs carry this sampling noise.
2. **Tightening σ_lin helps; tightening σ_ang kills in-place turning.**
   - 5a/5b: forward/sideways error −30% in Gazebo and training.
   - Spins fell to 0–0.3 rad/s and heading error tripled, whether σ_ang shrank in a late burst
     (Exp5) or gradually from 5.8k (5a).
   - Reason: turning at half of a 1.0 rad/s command earns 37% of the turn reward at σ 0.5, but
     only 6% at σ 0.3. Partial progress pays nothing.
   - The 5b-vs-5a spin gap (0.29 vs 0.02) is likely seed noise: their training yaw errors are
     equal.
3. **In-place turning from standing at ±0.5 stalls**, in Isaac, for the baseline and Exp6
   alike.
   - Measured with `isaac_turn_check.py`: 100% stalled at ±0.5 from standing, while ±1.0 spins
     at ~0.86–0.89.
   - Likely cause: a robot standing still under 0.5 rad/s still earns exp(−1) ≈ 37% at σ_ang
     0.5. Zeroing axes (Exp6) did not help.
   - The Gazebo spin test only commands ±1.0, so it doesn't see this.
4. **half_square may be noisy.**
   - Per-second detail (`spin_detail.py`): all policies start at ~0.4 rad/s. The baseline
     mostly sustains it; Exp6 and Exp11 decay to ~0 by second 3.
   - The baseline's single-seed 0.31 may be partly luck. A second baseline seed would tell.
5. **Mid-run checkpoints are misleading for in-place turning.** Exp12 spun 0.27 at 11.5k and
   0.71 at 20k. Compare final checkpoints, or same-iteration ones.
6. **The plateau.** The baseline levels off at ~17k: yaw floor ~0.74, xy flat from ~15k. The
   user's rule when shortening runs: plateau + 2k, minimum 12k.

## Waiting on the user (proposed, not approved; do not start)

- **Exp5c:** σ_lin curriculum only, σ_ang fixed at 0.5. It would subclass Exp5a and drop the
  `track_ang_vel_z_std` curriculum term. Aim: 5a's linear tracking with the baseline's turning.
  This is the recommended next run.
- **Add ±0.5 spins to the Gazebo spin test** (`unitree_gz_eval/suites/default.yaml`; the user
  suggested it).
  - This changes the suite hash, so the baseline must be re-evaluated. Update `REF` in
    `h2_pipeline/after_run.sh` afterwards.
  - Older evals stay comparable only on the unchanged tests.
- **Whether to adopt all-resets scoring in the baseline curricula** and rerun everything.
- **A second baseline seed**, to size the noise (half_square, spins).
- Unwritten ideas:
  - Exp13: filtered yaw reward;
  - heading-mode experiment (see memory `heading-mode-experiment-idea`);
  - Exp4 heading 0/100 is written but has not been run.

## How to run the next experiment

1. **Write it on the laptop.**
   - Add `robots/h2/experiments/expN_<name>.py` (under
     `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/`).
   - Subclass `baseline.EnvCfg` or another experiment's. `PlayEnvCfg` calls
     `baseline.apply_play_settings(self)`.
   - Register the task in `experiments/__init__.py`, and add rows to the `EXPERIMENTS.md` file
     table and plan table.
   - Never modify or delete old experiment code.
2. **The user commits and pushes** (submodule and top repo). Never commit or push yourself.
3. **Sync hermes:**
   `ssh kparan@hermes.cim.mcgill.ca 'cd ~/humanoidx/unitree-humanoidx && git pull --ff-only -q && git -C unitree_rl_lab pull --ff-only -q'`.
   A git-lfs post-merge warning there is harmless.
4. **Smoke test on hermes:**
   `source ~/miniconda3/etc/profile.d/conda.sh; conda activate unitree_rl_lab; cd ~/humanoidx/unitree-humanoidx/unitree_rl_lab; ./unitree_rl_lab.sh -t --task Unitree-H2-<Suffix> --num_envs 64 --max_iterations 5 --run_name smoke`.
5. **Queue it:**
   - Edit the `for spec in ...` line in hermes `~/humanoidx/h2_queue.sh`
     (`<TaskSuffix>:<run_name>` pairs; 20k iterations, wandb).
   - Note the line count of `~/humanoidx/h2_queue_status.txt`.
   - Start: `tmux new-session -d -s h2-queue "bash ~/humanoidx/h2_queue.sh"`.
   - ~0.84–1 s/iteration on hermes, so a 20k run takes about 5–5.5 h.
6. **Auto-eval on the laptop:**
   - Run `bash ~/humanoidx/h2_pipeline/after_run.sh <TaskSuffix> <run_name> <line count>` in
     the background.
   - It polls hermes every 5 min. When the run exits it exports on bikini, copies the policy
     to `~/humanoidx/misc-policies/<run>_seed42-<iter>/`, runs the headless Gazebo eval, and
     compares against the v3 eval.
   - The laptop must be on the McGill VPN and awake. To keep it awake, run in the background:
     `systemd-inhibit --what=sleep:handle-lid-switch --mode=block --who="H2 pipeline" --why="unattended eval" gnome-session-inhibit --app-id h2-pipeline --reason "unattended eval" --inhibit suspend sleep infinity`.
7. **Training curves:**
   - Find the run ID with wandb `query_wandb_tool` (runs, order `-created_at`).
   - Fetch `get_run_history_tool` with `samples=400` and these keys:
     - `Curriculum/lin_vel_cmd_levels`, `Curriculum/ang_vel_cmd_levels`
     - `Curriculum/track_lin_vel_xy_std`, `Curriculum/track_ang_vel_z_std` (σ runs only)
     - `Train/mean_episode_length`
     - `Metrics/base_velocity/error_vel_xy`, `Metrics/base_velocity/error_vel_yaw`
     - `Episode_Reward/track_lin_vel_xy`, `Episode_Reward/track_ang_vel_z`
     - `Episode_Termination/time_out`, `Episode_Termination/bad_orientation`,
       `Episode_Termination/base_height`
   - The result is too large to inline. Copy the saved tool-result file into
     `h2_pipeline/wandb_history/hist_<exp>_<id>.json`, then
     `python3 train_curves.py wandb_history/hist_baseline_vfwoqdqw.json wandb_history/hist_<exp>_<id>.json`.
8. **Report:** add a row to `ROWS` in `table_evals.py`; report Gazebo plus training against the
   baseline.

## Tools (`~/humanoidx/h2_pipeline/`)

| Script | What it does |
| --- | --- |
| `after_run.sh <suffix> <run> <lines>` | Waits for a hermes queue entry to exit, then `eval_auto.sh` + `compare_eval.py` vs the v3 eval (`REF`) |
| `eval_auto.sh <log folder> <run name> [iter]` | CPU ONNX export on bikini (`export_onnx_cpu.py`), copy to `misc-policies`, then `eval_local.sh`. Log folder = task name lowercased, e.g. `unitree_h2_exp5a_stdcurriculumallresets` |
| `eval_local.sh <policy dir>` | Headless Gazebo eval in its own process group, then `analyze.py`. Prints `EVAL_DIR=` |
| `eval_run.sh <policy dir>` | Same eval with the Gazebo GUI |
| `compare_eval.py <ref eval> <new eval> [exp6\|exp11]` | Overall metrics + per-segment turn rates, ref vs new. Skips aborted tests |
| `table_evals.py` | The two big tables above. Rows are hard-coded |
| `train_curves.py <hist.json>...` | Milestones and 1k-iteration bins, side by side |
| `spin_detail.py <eval dir>...` | Per-second turn rate in spin and half_square |
| `isaac_turn_check.py <name>=<policy.pt>...` | Isaac check: 32 robots, fixed spin commands from standing and walking. Run on hermes in the `unitree_rl_lab` env; a copy is in hermes `/tmp/kparan_isaac_check/` |
| `test_curr.py` + `stub/` | Torch-only stub test of the all-resets curricula (no Isaac). Run on hermes; the laptop has no torch |
| `wait_ckpt.sh` | Waits for a given checkpoint. Its early-exit grep is hard-coded to Exp11 (`Alive2 exit=`) |

Run `compare_eval.py` and `table_evals.py` from
`unitree_ros2_gazebo/unitree_gz_eval/analysis` with
`env -u PYTHONPATH uv run python <script>`; they import `analyze`.

## Machines and gotchas

- **hermes** (`kparan@hermes.cim.mcgill.ca`, RTX 4090) trains. tmux session `h2-queue`.
- **bikini** does the CPU export. Both share the NFS home `/home/robot/kparan/humanoidx`.
- **Network:** both are reachable only on the McGill network or VPN. Off-network, the waiters
  loop silently. Before re-running the evals after reconnecting, kill the waiters so you don't
  get several Gazebo evals at once.
- **wandb:** entity `kieranparanjpe-mcgill-university`, project `humanoidx-isaaclab`. Runs show
  "crashed" because rsl_rl never calls finish; this is normal.
- **Gazebo:** stop a stack by its process group (the scripts do this), never with a broad
  `pkill`.
- **Aborted eval tests:** a test can abort when its reset check fails, e.g. Exp2's third
  `stand`, after a push-test fall. It has no `file` in `meta.json`.
- **Isaac on misc/hermes** always runs headless.
- **Housekeeping, not done:**
  - stale stashes on bikini;
  - `/tmp/kparan_isaac_check` on bikini and hermes;
  - `*_smoke` run folders in hermes `unitree_rl_lab/logs/rsl_rl/`.

## User working rules

- The user commits and pushes. Edits go on the laptop checkout. Stage by name.
- Explain before editing; edit only when asked.
- Check in before state-changing commands on misc.
- Experiments are reported, not folded, unless the user says otherwise.
- Say σ_lin/σ_ang and paid/scoring, never bare "std".
- No emojis; concise.
