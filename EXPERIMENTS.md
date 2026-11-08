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
| Zero-snap | kept: a drawn command with size sqrt(cx² + cy² + cz²) ≤ 0.1 is set to 0, and the robot stands until its next draw |

Each axis is drawn uniformly and independently; no axis is ever exactly 0 except through the
zero-snap (which zeroes all three).

A snapped robot is put in standing mode (since 2026-11-07, in `UniformLevelVelocityCommand`,
`mdp/commands/velocity_command.py`). Before that, heading mode rewrote cz on the next step, so a
snapped heading-mode robot was told to turn in place instead of stand. 09-17-v2 was trained that
way, and unitree_rl_mjlab still works that way.

### Curriculum

- Speed (`lin_vel_cmd_levels`): kept. Once per episode length (1000 steps), if the average
  `track_lin_vel_xy` reward per second of the resetting robots is above 0.8 × its weight, cx and
  cy widen by 0.1 on each side, capped at their limits.
- Turn (`ang_vel_cmd_levels`): **added**. Same rule using `track_ang_vel_z`, widening cz.
- Terrain level: kept but meaningless (the ground is flat everywhere).

Early in training all three ranges are ±0.1, so about 52% of draws are zero-snapped; with the 5%
standing share, about 55% of robots stand until the first widening, whatever the heading-mode
share. This differs from 09-17-v2, where cz was ±1 from the start.

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
commands at weight 1.0 and turn tracking one at 0.5, so each command axis is worth 0.5.

The `joint_deviation_*` terms are Isaac Lab's `joint_deviation_l1` with G1's joint groups and
weights, plus a head group (G1 has no head joints).

Removed compared with the current H2 config: `pose` (`variable_posture`, +1.0) and `gait`
(`feet_gait`, +0.5).

Maximum positive reward: 1.0 + 0.5 + 1.0 = 2.5 per second.

### Unchanged from the current H2 config

- Timing: 0.005 s physics, 50 Hz policy, 20 s episodes, 4096 robots, flat ground.
- Robot model, motor gains and limits, action scale 0.25, default pose.
- Falls: episode ends below 0.2 m pelvis height or above 70° tilt.
- Randomization: friction 0.3–1.0, torso mass -1 to +3 kg, reset position ±0.5 m and any
  heading, pushes every 5–6 s (horizontal ±0.5 m/s per axis, vertical ±0.4 m/s, roll and pitch
  rate ±0.52 rad/s, yaw rate ±0.78 rad/s).
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

Each experiment subclasses `baseline.EnvCfg` and changes only its own setting. Terms that are off
in the baseline but used by experiments (`gait`, `gait_phase`) are listed in `baseline.py` as
`None`, with a function that builds them. Tasks are registered in `experiments/__init__.py`. The
existing `Unitree-H2-Velocity` task's config file (09-17-v2's config) is unchanged, but the
zero-snap fix (section 1) is in shared code, so retraining it would no longer exactly reproduce
09-17-v2.

Train with `python scripts/rsl_rl/train.py --task Unitree-H2-Baseline --headless`. Logs go to
`logs/rsl_rl/<task name lowercased with underscores>/`. Note: train.py copies only the file
defining the task's class into `params/`, so for an experiment the baseline file is not copied;
`params/env.yaml` still records the full resolved config.

### Plan

Each run changes one thing against the baseline. Every policy is evaluated the same way: the
Gazebo eval (`unitree_gz_eval`, default suite, 3 repeats), compared on the `aggregate` run's
`overall/...` metrics in wandb (see `unitree_ros2_gazebo/README.md`).

| # | Change | New code | Deploy change |
| --- | --- | --- | --- |
| 1 | + gait reward (`feet_gait`, 0.5), no gait phase observation. This is the G1 default | none | none |
| 2 | + gait reward + gait phase observation. This is the current H2 gait setup | none | automatic via `deploy.yaml` |
| 3 | + gait reward + gait phase, with the gait clock reset when walking starts | `gait_phase`, `feet_gait`, C++ `gait_phase` | C++ change |
| 4 | Heading-mode share 0% and 100% (baseline 50%) | none | none |
| 5 | Tracking std curriculum 0.5 → 0.05, linear and turn together | new curriculum function; the speed and turn curricula's 0.8 threshold must use a fixed std or they stall | none |
| 6 | Each command axis set to exactly 0 with some probability | command function | none |
| 7 | Randomized delay (action and/or sensor) | depends on motor model (currently physics-engine PD, no delay) | none |
| 8 | Unused reward terms, one at a time: `stand_still`, `feet_stumble`, `feet_too_near`, `air_time_variance_penalty`, `joint_mirror`, ... (all in `mdp/rewards.py`) | none | none |
| 9 | Survival design: no fall penalty with a larger alive bonus, vs. the baseline | none | none |
| later | Pose from an animation | large | — |

Gait experiments run in the order 1, 2, 3.

Notes:

- **Gait reward without the phase (1).** `feet_gait` computes its own clock from time since
  reset, whether or not the policy observes it. Out of sync, keeping both feet down scores about
  1.1 per step against about 1.01 for a proper walk at 55% stance; only an in-sync walk reaches
  2. Without the clock the reward may mostly act as a mild push to keep feet on the ground.
- **Heading mode (4).** With heading mode, drift is corrected by the command generator rather
  than the policy, and constant turn rates (how `/cmd_vel` drives the deployed robot) are never
  held. 100% is the 09-17-v2 setting. Snapped robots stand in every heading setting, so the
  share of standing robots is the same in all three runs.
- **Tracking std curriculum (5).** Small tracking errors currently cost almost nothing:
  exp(-0.1² / 0.25) = 0.96. A tighter std makes small drift matter, but too tight early gives no
  signal, hence the curriculum.

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
| Zero-snap | yes; snapped robots stand | yes, but heading mode turned snapped robots in place (trained before the fix) | no |
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
