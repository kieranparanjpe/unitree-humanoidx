# H2 velocity training: baseline and experiment plan

The stripped-down baseline that every H2 training experiment is compared against, the
experiments queued against it, and how the baseline differs from the current H2 config (the
09-17-v2 policy) and Unitree's G1 config. Decided 2026-11-07.

Formulas, file locations and the meaning of each setting are in `TRAINING_SPEC.md`. Config paths
below are relative to `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/`.

Notation: "a → b" is a curriculum, starting at a and growing to the limit b. c = (cx, cy, cz) is
the command: forward and sideways speed (m/s) and turn rate (rad/s). Reward weights are per
second; each policy step adds weight × value × 0.02.

---

## 1. Baseline

The baseline needs no new functions; every term below already exists in Isaac Lab or
`mdp/`. It is written as `robots/h2/experiments/baseline.py` (section 2).

### Commands

| Setting | Baseline |
| --- | --- |
| Resample | every 1–8 s (uniform, per robot) |
| Forward cx | [-0.1, 0.1] → [-0.5, 1.0] |
| Sideways cy | [-0.1, 0.1] → [-0.5, 0.5] |
| Turn cz | [-0.1, 0.1] → [-1.0, 1.0] |
| Heading mode | 50% of robots (`rel_heading_envs = 0.5`). In heading mode cz = clip(0.5 × wrap(target heading - heading), current turn range), so the turn curriculum also limits steering |
| Standing | 5% of robots, c = 0 (`rel_standing_envs = 0.05`) |
| Zero-snap | kept, and checked every step: whenever the command's size sqrt(cx² + cy² + cz²) is ≤ 0.1 it is set to 0 |

Each axis is drawn uniformly and independently; no axis is ever exactly 0 except through the
zero-snap (which zeroes all three).

Since 2026-11-09 the snap runs every step, after heading mode has set that step's cz
(`UniformLevelVelocityCommand`, `mdp/commands/velocity_command.py`), so a command is never
non-zero with size ≤ 0.1. A heading-mode robot with no linear command turns toward its target
until cz drops to 0.1 (about 11° away), then stands. Before that the snap ran only when a command
was drawn, using a cz that heading mode overwrote on the next step, so a snapped heading-mode
robot was told to turn in place instead of stand. 09-17-v2 was trained that way, and
unitree_rl_mjlab still works that way. (From 2026-11-07 to 2026-11-09 a snapped robot was
instead put in standing mode until its next draw.)

### Curriculum

- Speed (`lin_vel_cmd_levels_fixed_std`): kept, same rule as `lin_vel_cmd_levels`. Once per
  episode length (1000 steps), if the average `track_lin_vel_xy` reward per second of the
  resetting robots, computed at a fixed std of 0.5, is above 0.8 × its weight, cx and cy widen by
  0.1 on each side, capped at their limits.
- Turn (`ang_vel_cmd_levels_fixed_std`): **added**. Same rule using `track_ang_vel_z`, widening
  cz.
- The fixed std of 0.5 is the baseline's tracking std, so in the baseline the score is exactly the
  reward paid. It differs only in experiment 5, where the paid std shrinks; scoring at 0.5 keeps
  the ranges growing as in the baseline.
- Terrain level: kept but meaningless (the ground is flat everywhere).

Early in training all three ranges are ±0.1, so about 52% of non-heading-mode draws are
zero-snapped. A heading-mode robot is never snapped while it is still turning at the full 0.1, and
once it faces its target it is snapped whenever its linear command is ≤ 0.1 (about 79% of draws).
This differs from 09-17-v2, where cz was ±1 from the start.

### Observations

Same as the current H2 config except **no `gait_phase`**: 102 numbers per step × 5 steps of
history = 510. Noise and scales unchanged. The deploy side (robot and Gazebo `isaac_policy`)
builds its observation list from the exported `deploy.yaml`, so it follows automatically.

### Rewards

| Term | Weight | Value |
| --- | --- | --- |
| track_lin_vel_xy | 1.0 | exp(-((cx - vx)² + (cy - vy)²) / 0.25), velocity in the heading frame |
| track_ang_vel_z | 0.5 | exp(-(cz - ωz)² / 0.25) |
| alive | 1.0 | 1 while not fallen |
| is_terminated | -200 | 1 on the step of a fall (costs 4 once) |
| base_linear_velocity | -2.0 | vz² |
| base_angular_velocity | -0.5 | ωx² + ωy² |
| flat_orientation_l2 | -5.0 | gx² + gy² |
| base_height | -10 | (z - 1.0)² |
| joint_vel | -0.001 | Σ q̇² over 31 joints |
| joint_acc | -2.5e-7 | Σ q̈² |
| action_rate | -0.05 | Σ (a - a_previous)² |
| dof_pos_limits | -5.0 | Σ distance past the soft joint limits |
| energy | -2e-5 | Σ \|q̇\| × \|τ\| |
| joint_deviation_arms | -0.1 | Σ \|q - q_default\| over shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw |
| joint_deviation_waist | -1.0 | Σ \|q - q_default\| over waist yaw/roll/pitch |
| joint_deviation_hips | -1.0 | Σ \|q - q_default\| over hip roll, hip yaw |
| joint_deviation_head | -0.1 | Σ \|q - q_default\| over head pitch, head yaw |
| feet_clearance | -1.0 | if \|c\| > 0.1: Σ over feet \|foot height - 0.1\| × horizontal foot speed |
| feet_slide | -0.25 | if \|c\| > 0.1: Σ over feet foot speed while in contact |
| feet_contact_forces | -0.0002 | Σ over feet max(0, contact force - 500 N) |
| undesired_contacts | -1.0 | number of non-ankle bodies with contact force > 1 N |

Tracking: both kernels use the G1/H1 std of 0.5 (divisor 0.25). Linear tracking covers two
commands at weight 1.0 and turn tracking one at 0.5, so each command axis is worth 0.5. Both
terms are the `_with_fixed_std_sum` versions in `mdp/rewards.py`: the stock reward, plus each
robot's episode total at a fixed std of 0.5, which the speed and turn curricula score on.

The `joint_deviation_*` terms are Isaac Lab's `joint_deviation_l1` with G1's joint groups and
weights, plus a head group (G1 has no head joints).

Removed compared with the current H2 config: `pose` (`variable_posture`, +1.0) and `gait`
(`feet_gait`, +0.5).

Maximum positive reward: 1.0 + 0.5 + 1.0 = 2.5 per second.

### Unchanged from the current H2 config

- Timing: 0.005 s physics, 50 Hz policy, 20 s episodes, 4096 robots, flat ground.
- Robot model, motor gains and limits, action scale 0.25, default pose.
- Falls: episode ends below 0.2 m pelvis height or above 70° tilt.
- Randomization: friction 0.3–1.0 on all bodies, torso mass -1 to +3 kg (a payload on the upper
  body, not general mass randomization), reset position ±0.5 m and any heading, joints reset to
  exactly the default pose, pushes every 5–6 s (horizontal ±0.5 m/s per axis, vertical
  ±0.4 m/s, roll and pitch rate ±0.52 rad/s, yaw rate ±0.78 rad/s). The event terms come from
  Unitree's G1/H1 template.
- Removed: `base_external_force_torque`. Its force and torque ranges were 0, so it did nothing
  and training is unchanged. `reset_robot_joints` stays: it is what returns the joints to the
  default pose at reset (Isaac Lab does not), even though its velocity range has no effect.
- PPO settings and network sizes.

### Reward balance

Survival stays as in the current H2 config: alive 1.0 with a -200 fall penalty, and
feet_clearance as a penalty (zero when the feet are still). No weight budgets or normalisation.

Removing `pose` and `gait`, or adding terms in experiments, shifts how much staying alive is
worth relative to the fall penalty. Instead of correcting for that, check these for every run
(already logged by Isaac Lab) and flag a run whose result might come from a survival shift
rather than the change being tested:

- mean episode length
- share of terminations that are falls (`Episode_Termination/*`)
- net reward per second: positive `Episode_Reward/*` totals minus penalty totals

If an experiment noticeably moves these, the fall penalty can be made proportional to the
maximum positive reward (one line in the config).

---

## 2. Experiments

### Where the configs are

`robots/h2/experiments/` (under `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/`):

| File | Task | Status |
| --- | --- | --- |
| `baseline.py` | `Unitree-H2-Baseline` | written; self-contained (does not import `../velocity_env_cfg.py`) |
| `exp1_gait_reward.py` | `Unitree-H2-Exp1-GaitReward` | written |
| `exp2_gait_reward_phase.py` | `Unitree-H2-Exp2-GaitRewardPhase` | written |
| `exp4_heading_0.py` | `Unitree-H2-Exp4-Heading0` | written |
| `exp4_heading_100.py` | `Unitree-H2-Exp4-Heading100` | written |
| `exp5_tracking_std_curriculum.py` | `Unitree-H2-Exp5-TrackingStdCurriculum` | written |
| `exp7_domain_randomization.py` | `Unitree-H2-Exp7-DomainRandomization` | written |
| `exp10_pose.py` | `Unitree-H2-Exp10-Pose` | written |
| `exp11_alive_2.py` | `Unitree-H2-Exp11-Alive2` | written |
| `exp12_ang_std_1.py` | `Unitree-H2-Exp12-AngStd1` | written |

Each experiment subclasses `baseline.EnvCfg` and changes only its own setting. Terms that are off
in the baseline but used by several experiments (`gait`, `gait_phase`) are listed in
`baseline.py` as `None`, with a function that builds them; experiment 7's randomization terms and
experiment 10's `pose` term are used only there, so they are defined in their own files. Tasks are registered in
`experiments/__init__.py`. The
existing `Unitree-H2-Velocity` task's config file (09-17-v2's config) is unchanged, but the
zero-snap fix (section 1) is in shared code, so retraining it would no longer exactly reproduce
09-17-v2.

Train on misc (inside tmux, so it survives logging out):

```bash
tmux new -s h2-baseline
conda activate unitree_rl_lab
cd ~/kieran/unitree-humanoidx/unitree_rl_lab
./unitree_rl_lab.sh -t --task Unitree-H2-Baseline --max_iterations 20000 \
    --logger wandb --log_project_name humanoidx-isaaclab --run_name baseline_seed42 \
    2>&1 | tee baseline_seed42.log
```

`-t` runs `scripts/rsl_rl/train.py --headless`. Detach with `Ctrl-b d`, reattach with
`tmux attach -t h2-baseline`. Before a first real run of a new config, a quick check:
`--num_envs 64 --max_iterations 5` without the logger flags.

Logs go to `logs/rsl_rl/<task name lowercased with underscores>/`. Note: train.py copies only the file
defining the task's class into `params/`, so for an experiment the baseline file is not copied;
`params/env.yaml` still records the full resolved config.

### Plan

Each run changes one thing against the baseline, except experiment 7, which adds several kinds of
randomization at once. Every policy is evaluated the same way: the
Gazebo eval (`unitree_gz_eval`, default suite, 3 repeats), compared on the `aggregate` run's
`overall/...` metrics in wandb (see `unitree_ros2_gazebo/README.md`).

| # | Change | New code | Deploy change |
| --- | --- | --- | --- |
| 1 | + gait reward (`feet_gait`, 0.5), no gait phase observation. This is the G1 default | none | none |
| 2 | + gait reward + gait phase observation. This is the current H2 gait setup | none | automatic via `deploy.yaml` |
| 3 | + gait reward + gait phase, with the gait clock reset when walking starts | `gait_phase`, `feet_gait`, C++ `gait_phase` | C++ change |
| 4 | Heading-mode share 0% and 100% (baseline 50%) | none | none |
| 5 | Tracking std curriculum: each tracking term's std 0.5 → 0.05 in steps of 0.05, linear and turn separately | `tracking_std_levels` in `mdp/curriculums.py`; the speed and turn curricula already score at a fixed std (baseline), so they don't stall | none |
| 6 | Each command axis set to exactly 0 with some probability | command function | none |
| 7 | Domain randomization, all in one run: 0–15 ms action delay, redrawn every policy step; link masses ×0.9–1.1 plus a ±5 cm torso centre-of-mass offset; starting pose ±0.1 rad and joint velocity ±0.1 rad/s per joint. Motor gains not touched. Details below | delayed action term, `mdp/actions.py`; the rest are existing event functions | none |
| 8 | Unused reward terms, one at a time: `stand_still`, `feet_stumble`, `feet_too_near`, `air_time_variance_penalty`, `joint_mirror`, ... (all in `mdp/rewards.py`) | none | none |
| 9 | Survival design: no fall penalty with a larger alive bonus, vs. the baseline | none | none |
| 10 | Pose: the four `joint_deviation_*` penalties replaced by one `pose` penalty (`variable_posture_penalty`, -1.0), the 09-17-v2 `pose` term turned into a penalty | `variable_posture_penalty` in `mdp/rewards.py` | none |
| 11 | Alive 2.0 (baseline 1.0) | none | none |
| 12 | σ_ang = 1.0 for `track_ang_vel_z`, paid and scoring (baseline 0.5 for both) | none | none |
| later | Pose from an animation | large | — |

Gait experiments run in the order 1, 2, 3. Experiments 12 and then 11 run first, right after the
baseline. If one works, it is added to the baseline before the next experiment runs.

Notes:

- **Gait reward without the phase (1).** `feet_gait` computes its own clock from time since
  reset, whether or not the policy observes it. Out of sync, keeping both feet down scores about
  1.1 per step against about 1.01 for a proper walk at 55% stance; only an in-sync walk reaches
  2. Without the clock the reward may mostly act as a mild push to keep feet on the ground.
- **Heading mode (4).** With heading mode, drift is corrected by the command generator rather
  than the policy, and constant turn rates (how `/cmd_vel` drives the deployed robot) are never
  held. 100% is the 09-17-v2 setting. Heading-mode robots are snapped on their actual turn
  rate, so the share of standing robots differs between the three runs.
- **Domain randomization (7).** The baseline randomizes little: friction, a torso payload,
  start position and pushes. Nothing covers mass or centre of mass across the body, latency, or
  the starting joint state. Experiment 7 adds all three in a single run, so its result is for
  the set as a whole, not for each part. Motor gains are deliberately left alone. More
  randomization usually lowers sim tracking scores, so judge this run mainly on the Gazebo
  eval. Details in the next subsection.
- **Tracking std curriculum (5).** Small tracking errors currently cost almost nothing:
  exp(-0.1² / 0.25) = 0.96. A tighter std makes small drift matter, but too tight early gives no
  signal, hence the curriculum. On each 1000-step check, a tracking term's std drops by 0.05 if
  the reward it actually paid averaged above 0.8 × its weight; it never loosens. A reward above
  0.8 needs an error below about 0.47 × std, so the std stops near twice the robot's typical
  error, and its final value (`Curriculum/track_lin_vel_xy_std`, `Curriculum/track_ang_vel_z_std`
  in wandb) measures how precise the robot got. Risk: once the std is tight, a newly unlocked
  speed can start where the reward is nearly flat (an error of 2 × std pays 0.02).
- **Pose (10).** The `joint_deviation_*` penalties apply one weight per joint group at every
  speed, so they penalise the hips and knees for doing what walking requires. `pose` costs
  1 - exp(-mean over all joints of ((q - q_default) / std)²) at weight -1.0, with each joint's
  std picked from one of three tables by commanded speed |(cx, cy)| + |cz|: 0.05 rad for every
  joint when standing (below 0.1), wider when walking, wider again from 1.5. Hip pitch and knee
  get 0.5 rad while walking; waist roll and pitch stay at 0.1 at every speed to keep the torso
  upright. Tables are copied from the 09-17-v2 config (`../velocity_env_cfg.py`), where the same
  term is a +1.0 bonus. As a penalty it ranks poses the same way but, like the terms it replaces,
  is 0 at the default pose, so it doesn't make staying alive worth more. A single term can't
  match the four weights it replaces (-0.1 to -1, each on an unbounded sum); -1.0 caps it at
  1 per second. After both runs, compare the baseline's `Episode_Reward/joint_deviation_*` total
  with this run's `Episode_Reward/pose`.
- **Alive 2.0 (11).** At iteration 3000 of the first baseline run, episodes averaged 530 of 1000
  steps against 873 for 09-17-v2, and the reward while up was about -1.4 per second (penalties
  above alive plus tracking), so only the fall penalty made staying up worth it. Most of the gap
  to 09-17-v2 was its `gait` bonus (about +0.5 per second). Alive 2.0 adds 1 per second while up;
  once robots stop falling it is the same constant for every policy. If it learns noticeably
  faster (episode length, the iteration where the speed limits first widen) and is no worse
  otherwise, including the Gazebo eval, the baseline's alive becomes 2.0 before the other
  experiments run.
- **σ_ang 1.0 (12).** Paid σ_ang is the `std` the yaw reward uses; scoring σ_ang is the
  `fixed_std` the turn-rate limit curriculum scores with. In the baseline both are 0.5, and they
  are always equal there. The turn-rate limit widens only when the yaw reward at scoring σ_ang
  averages above 0.8. With the same yaw term, 09-10 settled at 0.60, and in the first baseline run
  the limit had not widened by iteration 4500. 09-10's per-step yaw error (about 0.385 rad/s)
  would score about 0.85 at σ_ang = 1.0. It works if `Curriculum/ang_vel_cmd_levels` rises with
  linear tracking, survival and the Gazebo eval no worse. Then the baseline's paid and scoring
  σ_ang both become 1.0, and experiment 5's σ_ang curriculum starts from 1.0.

### Experiment 7: domain randomization

One task, `Unitree-H2-Exp7-DomainRandomization` (`exp7_domain_randomization.py`): the baseline
plus all three parts below, together. Motor gains (PD stiffness and damping) are not randomized.
"Per robot" means each of the 4096 robots draws its own value.

| Part | Randomized | Drawn | Range | Implementation |
| --- | --- | --- | --- | --- |
| Delay | Delay before a new joint target reaches the motors | per robot, every policy step | 0–15 ms (0–3 physics steps, uniform) | `DelayedJointPositionActionCfg` (`mdp/actions.py`) replaces the action term |
| Mass | Mass of every link except the torso; torso centre of mass | per robot, once at startup | ×0.9–1.1 per link; ±5 cm per axis | `randomize_rigid_body_mass` (scale; inertia scales with it), `randomize_rigid_body_com` |
| Start state | Starting joint angles and joint velocities | per robot, every reset | default ± 0.1 rad; ± 0.1 rad/s; per joint | `reset_joints_by_offset` replaces the baseline's `reset_joints_by_scale` with scale 1 |

**Delay.** Training is fully synchronous: PhysX applies the PD (implicit, no delay) and the
new target is used from the first physics step after the policy acts. The deployed stack is not:
in Gazebo the first policy output after a reset lands 2–7 ms after it, torque timing between
nodes is asynchronous, and identical eval runs diverge (`EVAL_HANDOFF.md`, `PROJECT.md` §6.9).
The delayed action term keeps the previous joint target for the first d physics steps of each
policy step, then switches to the new one. d is redrawn per robot every policy step, so each
target lands at a random point in its 20 ms window. Targets never arrive out of order. d is at
most 3, so every target is used for at least one physics step. At 4, a target delayed by a full
step followed by a zero delay would never be used, which happens to 1 in 25 targets.

- Sensor delay is not a separate setting. For a feedback loop what matters is the total time
  from reading the state to the motor acting on the result, so all of it is put on the action
  side.
- 0–15 ms covers the Gazebo measurement (2–7 ms) and the published humanoid ranges of up to
  10 ms, but not the 10–50 ms some other work uses (`PROJECT.md` §6.9). The term keeps only one
  previous target, so it can never exceed one policy step (20 ms).
- Not done with Isaac Lab's `DelayedPDActuatorCfg` (which the repo's `UnitreeActuator` builds
  on). It delays only by replacing the implicit PD with an explicit one computed at 200 Hz, which
  would be a second change in the same run and changes how the light joints (wrists) respond. It
  also draws its delay only at reset. The repo has no motor models for H2's motors, so its
  torque-speed and friction effects wouldn't apply either.
- Not covered: in Gazebo the PD itself works from joint readings 1–3 ms old, because it runs in
  a ROS node. That is Gazebo-only (the real motor driver runs its PD next to the motor), and a
  policy can't compensate for it. It's what makes the wrists shake while standing.
- Success here is mainly a smaller spread between repeats of the Gazebo eval, not only a better
  mean.

**Mass.** `randomize_rigid_body_mass` resets the bodies it is given to their default mass
before applying its change. Including the torso would erase the baseline's -1 to +3 kg torso
payload, so the torso is left out of the scaling and keeps the payload. The centre-of-mass range
is unitree_rl_mjlab's `base_com`, which its H2 config applies to `torso_link`. Both are drawn
once, at startup. The centre-of-mass event adds to the current value, so it must not run at reset.

**Start state.** Offsets rather than the 0.5–1.5 scale Isaac Lab uses for quadrupeds (this
repo's G1 and H1 configs set that scale to 1, i.e. off):

- A scale leaves every joint whose default is 0 at exactly 0. That's the waist, hip roll and
  yaw, ankle roll, wrists, head and most of the shoulder joints.
- On the legs a scale changes leg length by several cm. The pelvis is still placed at 1.03 m, so
  the feet start inside the ground (up to about 2 cm) or above it (up to about 7 cm).
- ±0.1 rad on every joint keeps the feet within about 1 cm below and 3 cm above their default
  height (H2.urdf: thigh 0.41 m, shin 0.50 m). Values are clamped to the soft joint limits.
- Joint velocities start at ±0.1 rad/s per joint (an offset on the default of 0) instead of 0.

The Gazebo eval always starts from the exact default pose, so this part may show little there.
It is about recovering from off-nominal poses after pushes and at hardware start-up.

What to watch, beyond the usual checks under "Reward balance":

- Tracking rewards should drop a little. A large drop, or fall terminations rising, means the
  ranges are too wide for this reward setup rather than that randomization doesn't help.
- The delay can make the policy stiffer or more hesitant. `action_rate` and `joint_acc` show it.
- If the run is clearly worse, split it into one run per part to find which one is responsible.

---

## 3. Comparison: baseline vs. current H2 (09-17-v2) vs. G1 (29 dof)

G1 is `robots/g1/29dof/velocity_env_cfg.py`; current H2 is `robots/h2/velocity_env_cfg.py`.

| Area | Baseline | Current H2 | G1 |
| --- | --- | --- | --- |
| Resample | 1–8 s | 3–8 s | 10 s |
| cx limit | [-0.5, 1.0] | [-0.5, 1.0] | [-0.5, 1.0] |
| cy limit | ±0.5 | ±0.3 | ±0.3 |
| cz | ±0.1 → ±1.0 (curriculum) | ±1.0 from the start | ±0.1 throughout training (no turn curriculum; ±0.2 only in play mode) |
| Heading mode | 50% | 100% | off (constant turn rates) |
| Standing | 5% | 5% | 2% |
| Zero-snap | yes, every step | yes, but heading mode turned snapped robots in place (trained before the fix) | no |
| Gait phase observation | no | yes (0.8 s period, zero when standing) | no (commented out) |
| History | 5 | 5 | 5 |
| track_lin_vel_xy | 1.0, divisor 0.25 | 1.0, divisor 0.25 | 1.0, divisor 0.25 |
| track_ang_vel_z | 0.5, divisor 0.25 | 1.0, divisor 0.5 | 0.5, divisor 0.25 |
| alive | 1.0 | 1.0 | 0.15 |
| Fall penalty | -200 | -200 | none |
| Pose | G1 joint_deviation groups + head | `variable_posture` +1.0 | joint_deviation: arms -0.1, waist -1, hip roll/yaw -1 |
| gait | none | +0.5 | +0.5 |
| Feet clearance | penalty -1.0, zero when feet are still | same | bonus +1.0, exp form, pays 1 when feet are still |
| feet_slide | -0.25, only while moving | same | -0.2, always |
| feet_contact_forces | -0.0002 | same | none |
| base_angular_velocity | -0.5 | -0.5 | -0.05 |
| base_height target | 1.0 m | 1.0 m | 0.78 m (robot-specific) |
| Other penalties | lin_vel_z -2, joint_vel -0.001, joint_acc -2.5e-7, action_rate -0.05, dof_pos_limits -5, energy -2e-5, flat_orientation -5, undesired_contacts -1 | same | same |
| Tilt ending | 70° | 70° | 0.8 rad (about 46°) |
| Pushes | every 5–6 s, all six axes | same | every 5 s, horizontal only, ±0.5 m/s |
| Friction, torso mass | 0.3–1.0, -1 to +3 kg | same | same |

Why survival differs from G1: G1 has no fall penalty, but most of its positive terms pay out for
standing still (tracking at zero command, the exp-form feet clearance bonus), so falling forfeits
a lot of future reward. H2 replaced the clearance bonus with mjlab's penalty, which pays nothing
for standing; with alive at 0.15 and no fall penalty, run `yhyw3t6n` learned to fall
immediately. H2 therefore uses mjlab's explicit fall penalty (-200, added in commit `2afa903`)
with alive at 1.0. The reasoning is recorded in the comments above `alive` in the H2 config.

---

## 4. Record of runs

| Run name | Training run (wandb) | Change vs. baseline | Why | Eval (aggregate run) | Notes |
| --- | --- | --- | --- | --- | --- |
| | | | | | |
