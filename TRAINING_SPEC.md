# H2 velocity training: everything that defines it

What decides how an H2 walking policy is trained, where each piece is set, and the exact
formulas. Written from the code as of 2026-11-06, for task `Unitree-H2-Velocity`; the 09-17-v2
policy was trained with exactly this configuration (its saved `params/velocity_env_cfg.py` is
byte-identical to the repo file).

Paths are relative to `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/` unless they start
with `unitree_rl_lab/` or `Isaac Lab`. "Isaac Lab" means the Isaac Lab 2.3.0 library (the
version `unitree_rl_lab/README.md` pins), which is not in this repo.

Contents:
1. [The files](#1-the-files)
2. [Timing](#2-timing)
3. [Robot model and motors](#3-robot-model-and-motors)
4. [Actions: what the policy outputs](#4-actions-what-the-policy-outputs)
5. [Commands: what the robot is asked to do](#5-commands-what-the-robot-is-asked-to-do)
6. [Observations: what the policy sees](#6-observations-what-the-policy-sees)
7. [Rewards](#7-rewards)
8. [Episode endings](#8-episode-endings)
9. [Randomization and resets](#9-randomization-and-resets)
10. [Curriculum: what changes as training progresses](#10-curriculum-what-changes-as-training-progresses)
11. [Ground and sensors](#11-ground-and-sensors)
12. [Learning algorithm](#12-learning-algorithm)
13. [What must match the deployed robot](#13-what-must-match-the-deployed-robot)
14. [Running experiments](#14-running-experiments)

---

## 1. The files

| What | File | Defines |
| --- | --- | --- |
| The task | `tasks/locomotion/robots/h2/velocity_env_cfg.py` | Nearly everything: ground, sensors, randomization (`EventCfg`), commands (`CommandsCfg`), actions (`ActionsCfg`), observations (`ObservationsCfg`), rewards (`RewardsCfg`), episode endings (`TerminationsCfg`), curriculum (`CurriculumCfg`), timing (`RobotEnvCfg.__post_init__`) |
| Task name | `tasks/locomotion/robots/h2/__init__.py` | Registers `Unitree-H2-Velocity` and points it at the files above |
| Robot | `assets/robots/unitree.py` (`UNITREE_H2_CFG`, line ~731) | Robot model file, spawn height, default joint angles, motor stiffness/damping/limits, soft joint-limit factor |
| Unitree reward functions | `tasks/locomotion/mdp/rewards.py` | `energy`, `variable_posture` (pose), `feet_gait`, `feet_clearance_penalty`, `feet_slide_when_moving`, plus unused alternatives |
| Unitree observation functions | `tasks/locomotion/mdp/observations.py` | `gait_phase` |
| Unitree command | `tasks/locomotion/mdp/commands/velocity_command.py` | Snapping small commands to zero |
| Unitree curriculum | `tasks/locomotion/mdp/curriculums.py` | `lin_vel_cmd_levels` (speed range growth) |
| Isaac Lab functions | Isaac Lab `envs/mdp/rewards.py`, `observations.py`, `terminations.py`, `events.py`, `commands/velocity_command.py`; `isaaclab_tasks/.../locomotion/velocity/mdp/rewards.py` and `curriculums.py` | Every other reward, observation, ending, randomization, the heading controller, the terrain curriculum |
| Isaac Lab bookkeeping | Isaac Lab `managers/reward_manager.py`, `managers/observation_manager.py`, `envs/mdp/actions/joint_actions.py` | Rewards scaled by time step; order of noise and scaling; history stacking; how actions become joint targets |
| Learning algorithm | `tasks/locomotion/agents/rsl_rl_ppo_cfg.py` | PPO settings, network sizes |
| Launch | `unitree_rl_lab/scripts/rsl_rl/train.py` | Command-line overrides (e.g. `--max_iterations`) |
| Deploy export | `utils/export_deploy_cfg.py` | Writes `params/deploy.yaml`, which the robot / Gazebo stack reads |
| Deployed observations | `unitree_rl_lab/deploy/include/isaaclab/envs/mdp/observations/observations.h` | The C++ copy of each observation; must match training (section 13) |

Each training run saves the resolved configuration in its log folder under `params/`
(`env.yaml`, `agent.yaml`, `velocity_env_cfg.py`, `deploy.yaml`). Those are the record of what a
policy was actually trained with.

## 2. Timing

| Quantity | Value | Where |
| --- | --- | --- |
| Physics step | 0.005 s (200 Hz) | `sim.dt` |
| Physics steps per policy step | 4 | `decimation` |
| Policy step $\Delta t$ | 0.02 s (50 Hz) | `sim.dt × decimation` |
| Episode length | 20 s = 1000 policy steps | `episode_length_s` |
| Parallel robots | 4096 | `RobotSceneCfg(num_envs=4096)` |

## 3. Robot model and motors

`assets/robots/unitree.py`, `UNITREE_H2_CFG`.

- Model: `H2/H2_dae.usd`. 31 joints, all controlled by the policy (legs, waist, arms, wrists,
  head).
- Spawn: pelvis at 1.03 m. Default joint angles $q^0$ (rad): hip pitch −0.25, knee 0.5, ankle
  pitch −0.25, shoulder pitch 0.35, elbow 0.87, left shoulder roll 0.18, right shoulder roll
  −0.18, all others 0.
- Soft joint limits $[\ell_j, h_j]$: the middle 90% of each joint's range
  (`soft_joint_pos_limit_factor = 0.9`, the Unitree default, not overridden for H2). Used by the
  joint-limit penalty.
- Motors (each a spring-damper simulated inside the physics engine, with no delay):

| Group | Joints | Stiffness $k_p$ | Damping $k_d$ | Torque limit (N·m) |
| --- | --- | --- | --- | --- |
| legs | hip pitch/roll/yaw, knee | 200 | 4 | 360 |
| ankle_roll | ankle roll | 40 | 2 | 19 |
| ankle_pitch | ankle pitch | 40 | 2 | 66.88 |
| waist | waist yaw/roll/pitch | 150 | 3 | 120 / 180 / 180 |
| arms | shoulder pitch/roll/yaw, elbow, wrist roll | 40 | 2 | 130 (shoulder pitch), 60 (rest) |
| wrist_pitch_yaw | wrist pitch, yaw | 20 | 1 | 10 |
| head | head pitch, yaw | 20 | 1 | 50 (gains are a guess; see the file) |

All groups also have armature 0.01 and a velocity limit (see the file).

## 4. Actions: what the policy outputs

`ActionsCfg` and Isaac Lab `joint_actions.py`. The policy outputs one number $a_j$ per joint
(31 numbers). Each becomes a target angle

$$
q^{\text{target}}_j = q^0_j + 0.25\, a_j
$$

and the motor applies, every physics step,

$$
\tau_j = \operatorname{clip}\!\left( k_{p,j}\,\big(q^{\text{target}}_j - q_j\big) - k_{d,j}\,\dot q_j,\; -\tau^{\max}_j,\; \tau^{\max}_j \right)
$$

$a_j$ is not clipped. The target is held for the 4 physics steps of a policy step.

## 5. Commands: what the robot is asked to do

`CommandsCfg`, Isaac Lab `commands/velocity_command.py`, Unitree `commands/velocity_command.py`.

A command is $c = (c_x, c_y, c_z)$: forward speed (m/s), sideways speed (m/s) and turn rate
(rad/s), in the robot's own frame. Write $\lVert c \rVert = \sqrt{c_x^2 + c_y^2 + c_z^2}$.

**Every 3–8 s** (a new random time for each robot, uniform in $[3, 8]$), each robot draws:

$$
\begin{aligned}
c_x &\sim \mathcal{U}(X), &\quad X &\text{ starts at } [-0.1,\ 0.1] \text{ and grows to } [-0.5,\ 1.0] \text{ (section 10)} \\
c_y &\sim \mathcal{U}(Y), & Y &\text{ starts at } [-0.1,\ 0.1] \text{ and grows to } [-0.3,\ 0.3] \\
c_z &\sim \mathcal{U}(-1.0,\ 1.0) \\
\psi^* &\sim \mathcal{U}(-\pi,\ \pi) && \text{target heading} \\
\text{heading mode} &= \text{yes with probability } 1.0 && (\texttt{rel\_heading\_envs}) \\
\text{standing mode} &= \text{yes with probability } 0.05 && (\texttt{rel\_standing\_envs})
\end{aligned}
$$

($\mathcal{U}(a, b)$ means uniformly random between $a$ and $b$.) Then, if
$\lVert c \rVert \le 0.1$, it sets $c = (0, 0, 0)$ (`zero_command_threshold`).

**Every policy step**, with $\psi$ the robot's current heading:

$$
\text{heading mode:}\quad c_z = \operatorname{clip}\!\big( 0.5 \cdot \operatorname{wrap}(\psi^* - \psi),\; -1.0,\; 1.0 \big)
\qquad\qquad
\text{standing mode:}\quad c = (0, 0, 0)
$$

where $\operatorname{wrap}$ maps an angle into $[-\pi, \pi]$.

Consequences worth knowing:
- Because every robot is in heading mode, the drawn $c_z$ is always overwritten. The turn
  command is a steering signal that shrinks to 0 as the robot reaches $\psi^*$. A constant turn
  rate is never held for more than about 1 s: the full 1 rad/s only lasts while the heading
  error is above 2 rad.
- The zero-snap uses the drawn $c_z$ before it is overwritten. A snapped command still gets a
  steering $c_z$ on the next step, so in practice it only zeroes $c_x$ and $c_y$. (That is how
  09-17-v2 was trained. Since 2026-11-09 the snap is checked every step on the command actually
  given, so a retrain behaves differently; see `EXPERIMENTS.md`.)
- On the deployed robot, the turn rate comes straight from `/cmd_vel`; nothing steers to a
  heading.

## 6. Observations: what the policy sees

`ObservationsCfg`, Isaac Lab `observations.py` and `observation_manager.py`, Unitree
`observations.py`.

Each term is computed, then random noise is added, then it is multiplied by its scale (so the
noise is scaled too). Noise is uniform in the range shown, drawn independently for every number
at every step.

**Policy input** (in this order):

| # | Term | What it is | Size | Scale | Noise (before scaling) |
| --- | --- | --- | --- | --- | --- |
| 1 | base_ang_vel | Pelvis rotation rate $\omega = (\omega_x, \omega_y, \omega_z)$, pelvis frame (rad/s) | 3 | 0.2 | ±0.2 |
| 2 | projected_gravity | Unit vector of gravity in the pelvis frame, $g$; equals $(0, 0, -1)$ when upright | 3 | 1 | ±0.05 |
| 3 | velocity_commands | The command $c$ from section 5 | 3 | 1 | none |
| 4 | joint_pos_rel | $q_j - q^0_j$ for all 31 joints (rad) | 31 | 1 | ±0.01 |
| 5 | joint_vel_rel | $\dot q_j$ for all 31 joints (rad/s) | 31 | 0.05 | ±1.5 |
| 6 | last_action | The policy's previous output $a_j$ (unscaled) | 31 | 1 | none |
| 7 | gait_phase | $\big(\sin 2\pi\phi,\ \cos 2\pi\phi\big)$ if $\lVert c \rVert > 0.1$, else $(0, 0)$; $\phi$ defined below | 2 | 1 | none |

The gait clock, with $t$ the time since the episode started:

$$
\phi = \frac{t \bmod 0.8}{0.8} \in [0, 1)
$$

One step is 104 numbers. **History**: each term keeps its last 5 steps, oldest first, and the
terms are joined in the order above: term 1's five steps (15 numbers), then term 2's five, and
so on. The policy sees $104 \times 5 = 520$ numbers.

**Critic input** (used only during training, to judge how good a state is; never deployed): the
same terms with no noise, plus `base_lin_vel` (pelvis velocity, pelvis frame, 3 numbers) at the
front. $107 \times 5 = 535$ numbers.

## 7. Rewards

`RewardsCfg`, plus the function files in section 1.

### How the total is formed

Every policy step, each term $i$ produces a number $r_i$ and has a weight $w_i$. The reward for
that step is

$$
R = \sum_i w_i \, r_i \, \Delta t, \qquad \Delta t = 0.02\ \text{s}
$$

So weights are per second: a term with $w = 1$ and $r = 1$ for a whole episode adds
$1 \times 20 = 20$ to the episode's total.

### Symbols

| Symbol | Meaning |
| --- | --- |
| $c = (c_x, c_y, c_z)$ | Command (section 5); $\lVert c \rVert = \sqrt{c_x^2 + c_y^2 + c_z^2}$ |
| $v = (v_x, v_y, v_z)$ | Pelvis velocity, pelvis frame |
| $\tilde v = (\tilde v_x, \tilde v_y)$ | Pelvis velocity in a frame turned with the robot's heading but kept level (pitch and roll don't affect it) |
| $\omega = (\omega_x, \omega_y, \omega_z)$ | Pelvis rotation rate, pelvis frame |
| $g = (g_x, g_y, g_z)$ | Gravity unit vector in the pelvis frame |
| $z$ | Pelvis height above the ground |
| $q_j,\ \dot q_j,\ \ddot q_j$ | Angle, speed, acceleration of joint $j$ ($j = 1 \dots 31$) |
| $q^0_j$ | Default angle of joint $j$ (section 3) |
| $[\ell_j, h_j]$ | Soft limits of joint $j$ (section 3) |
| $\tau_j$ | Torque applied at joint $j$ |
| $a_j,\ a'_j$ | This step's action and the previous step's action for joint $j$ |
| $i \in \{L, R\}$ | The two feet: the `left/right_ankle_pitch_link` bodies |
| $p_{i,z}$ | Height of foot $i$ (world) |
| $u_i$ | Horizontal speed of foot $i$ (world) |
| $F_i$ | Largest contact force on foot $i$ over the last 3 physics steps |
| $\phi$ | Gait clock (section 6) |
| $\mathbb{1}[\cdot]$ | 1 if the condition is true, 0 otherwise |

### Tracking

**track_lin_vel_xy**, $w = +1.0$ (isaaclab_tasks `track_lin_vel_xy_yaw_frame_exp`)

$$
r = \exp\!\left( -\frac{(c_x - \tilde v_x)^2 + (c_y - \tilde v_y)^2}{0.25} \right)
$$

**track_ang_vel_z**, $w = +1.0$ (Isaac Lab `track_ang_vel_z_exp`)

$$
r = \exp\!\left( -\frac{(c_z - \omega_z)^2}{0.5} \right)
$$

### Survival

**alive**, $w = +1.0$ (Isaac Lab `is_alive`). $r = 1$ unless the episode ended by falling this
step (running out of time doesn't count as falling).

**is_terminated**, $w = -200$ (Isaac Lab `is_terminated`). $r = 1$ on the step the episode ends by
falling (section 8), else 0.

### Pelvis

**base_linear_velocity**, $w = -2.0$ (Isaac Lab `lin_vel_z_l2`)

$$
r = v_z^2
$$

**base_angular_velocity**, $w = -0.5$ (Isaac Lab `ang_vel_xy_l2`)

$$
r = \omega_x^2 + \omega_y^2
$$

**flat_orientation_l2**, $w = -5.0$ (Isaac Lab `flat_orientation_l2`). Equal to the sine squared
of the tilt angle.

$$
r = g_x^2 + g_y^2
$$

**base_height**, $w = -10$ (Isaac Lab `base_height_l2`)

$$
r = (z - 1.0)^2
$$

### Joints

**joint_vel**, $w = -0.001$ (Isaac Lab `joint_vel_l2`)

$$
r = \sum_{j=1}^{31} \dot q_j^{\,2}
$$

**joint_acc**, $w = -2.5 \times 10^{-7}$ (Isaac Lab `joint_acc_l2`)

$$
r = \sum_{j=1}^{31} \ddot q_j^{\,2}
$$

**action_rate**, $w = -0.05$ (Isaac Lab `action_rate_l2`)

$$
r = \sum_{j=1}^{31} \big(a_j - a'_j\big)^2
$$

**dof_pos_limits**, $w = -5.0$ (Isaac Lab `joint_pos_limits`). How far each joint is past its soft
limit, summed.

$$
r = \sum_{j=1}^{31} \Big[ \max(0,\ \ell_j - q_j) + \max(0,\ q_j - h_j) \Big]
$$

**energy**, $w = -2 \times 10^{-5}$ (Unitree `energy`)

$$
r = \sum_{j=1}^{31} \lvert \dot q_j \rvert \, \lvert \tau_j \rvert
$$

**pose**, $w = +1.0$ (Unitree `variable_posture`). Rewards staying near the default pose, with a
per-joint tolerance $\sigma_j$ that depends on how fast the robot is asked to move:

$$
r = \exp\!\left( -\frac{1}{31} \sum_{j=1}^{31} \frac{\big(q_j - q^0_j\big)^2}{\sigma_j^2} \right),
\qquad
s = \sqrt{c_x^2 + c_y^2} + \lvert c_z \rvert
$$

$\sigma_j$ comes from the "standing" table if $s < 0.1$ (0.05 for every joint), the "walking"
table if $0.1 \le s < 1.5$, and the "running" table if $s \ge 1.5$. The per-joint tables are in
`RewardsCfg.pose`. A small $\sigma_j$ means the joint must stay put; a large one lets it move.

### Feet

**gait**, $w = +0.5$ (Unitree `feet_gait`). Each foot has a phase: $\phi_L = \phi$ and
$\phi_R = (\phi + 0.5) \bmod 1$. Foot $i$ should be on the ground when $\phi_i < 0.55$ and in
the air otherwise. The reward counts the feet doing what their phase says (0, 1 or 2), only
while moving:

$$
r = \mathbb{1}\big[\lVert c \rVert > 0.1\big] \sum_{i \in \{L,R\}} \mathbb{1}\Big[ \big(\phi_i < 0.55\big) = \big(\text{foot } i \text{ touching}\big) \Big]
$$

**feet_slide**, $w = -0.25$ (Unitree `feet_slide_when_moving`, wrapping isaaclab_tasks
`feet_slide`). Horizontal foot speed while that foot is on the ground:

$$
r = \mathbb{1}\big[\lVert c \rVert > 0.1\big] \sum_{i \in \{L,R\}} u_i \; \mathbb{1}\big[F_i > 1\ \text{N}\big]
$$

**feet_clearance**, $w = -1.0$ (Unitree `feet_clearance_penalty`). A foot moving sideways at any
height other than 0.1 m costs; zero when the feet are still:

$$
r = \mathbb{1}\big[\lVert c \rVert > 0.1\big] \sum_{i \in \{L,R\}} \lvert p_{i,z} - 0.1 \rvert \; u_i
$$

**feet_contact_forces**, $w = -0.0002$ (Isaac Lab `contact_forces`). Contact force above 500 N:

$$
r = \sum_{i \in \{L,R\}} \max\big(0,\ F_i - 500\big)
$$

### Other

**undesired_contacts**, $w = -1.0$ (Isaac Lab `undesired_contacts`). The number of bodies whose
name doesn't contain "ankle" that have a contact force above 1 N over the last 3 physics steps,
self-contact included:

$$
r = \sum_{b\ \text{not an ankle}} \mathbb{1}\big[F_b > 1\ \text{N}\big]
$$

### Unused alternatives

Already written in Unitree `rewards.py` but not used: `stand_still`, `orientation_l2`, `upward`,
`joint_position_penalty`, `feet_stumble`, `feet_height_body`, `foot_clearance_reward`,
`feet_too_near`, `feet_contact_without_cmd`, `air_time_variance_penalty`, `joint_mirror`. The
comments in `RewardsCfg` record why several current weights were chosen and which runs failed
with other values. Read them before changing `alive`, `is_terminated`, `flat_orientation_l2` or
`feet_clearance`.

## 8. Episode endings

`TerminationsCfg`, Isaac Lab `terminations.py`.

| Ending | Condition | Counts as falling (−200 penalty)? |
| --- | --- | --- |
| time_out | 1000 steps (20 s) reached | No |
| base_height | $z < 0.2$ m | Yes |
| bad_orientation | $\arccos(-g_z) > 70^\circ$ (the tilt angle) | Yes |

There is deliberately no ending for the torso touching something; the comment explains why
(self-contact set it off).

## 9. Randomization and resets

`EventCfg`, Isaac Lab `events.py`.

| Event | When | What it does |
| --- | --- | --- |
| physics_material | once, at start | Each body's friction (static and sliding) $\sim \mathcal{U}(0.3, 1.0)$, bounce 0. The ground has friction 1.0 and the two are multiplied, so the effective friction is 0.3–1.0 |
| add_base_mass | once, at start | Adds $\mathcal{U}(-1, 3)$ kg to `torso_link` |
| base_external_force_torque | each reset | Force and torque ranges are 0: **does nothing** |
| reset_base | each reset | Pelvis at its start position + $\mathcal{U}(-0.5, 0.5)$ m in $x$ and $y$; heading $\sim \mathcal{U}(-3.14, 3.14)$; all velocities 0 |
| reset_robot_joints | each reset | Angles: $q_j = q^0_j \cdot \mathcal{U}(1.0, 1.0) = q^0_j$. Speeds: $\dot q_j = 0 \cdot \mathcal{U}(-1, 1) = 0$, because the default speed is 0, so the velocity range has **no effect** |
| push_robot | every $\mathcal{U}(5, 6)$ s, per robot | Adds to the pelvis velocity, each drawn separately: $x, y \sim \mathcal{U}(-0.5, 0.5)$ m/s, $z \sim \mathcal{U}(-0.4, 0.4)$ m/s, roll and pitch rate $\sim \mathcal{U}(-0.52, 0.52)$ rad/s, yaw rate $\sim \mathcal{U}(-0.78, 0.78)$ rad/s. Largest horizontal push: $\sqrt{0.5^2 + 0.5^2} \approx 0.71$ m/s |

Not randomized at all: motor stiffness and damping, motor delay, sensor delay, joint friction,
centre-of-mass position, ground shape.

## 10. Curriculum: what changes as training progresses

`CurriculumCfg`.

**Speed range** (Unitree `lin_vel_cmd_levels`). Every 1000 steps (one episode length), for the
robots resetting at that step, it computes the average `track_lin_vel_xy` reward per second:

$$
\bar r = \frac{\text{mean over those robots of the episode's total } w\, r\, \Delta t}{20\ \text{s}}
$$

If $\bar r > 0.8$, both ranges widen by 0.1 m/s on each side, capped at the limits:

$$
X \leftarrow \big[\max(X_{\text{lo}} - 0.1,\ -0.5),\ \min(X_{\text{hi}} + 0.1,\ 1.0)\big],
\qquad
Y \leftarrow \big[\max(Y_{\text{lo}} - 0.1,\ -0.3),\ \min(Y_{\text{hi}} + 0.1,\ 0.3)\big]
$$

They never shrink. With a constant error $e$, $\exp(-e^2/0.25) > 0.8$ means $e < 0.24$ m/s, so
the rule is roughly "widen once tracking is within about 0.24 m/s". In the 09-17-v2 run the cap
was reached at iteration 4,250 of 32,000. The turn-rate range has no curriculum:
`ang_vel_cmd_levels` exists in the file but is not used.

**Terrain level** (isaaclab_tasks `terrain_levels_vel`). At reset, a robot moves to a harder tile
if it walked more than 4 m from where it started, or to an easier one if it walked less than
$\tfrac{1}{2}\sqrt{c_x^2 + c_y^2} \times 20$ s. **The ground is flat everywhere** (the only tile
type is `flat`), so this changes nothing except which flat tile the robot starts on. The
`Curriculum/terrain_levels` number in wandb is meaningless for this task.

## 11. Ground and sensors

`RobotSceneCfg`.

- Ground: a 9 × 21 grid of 8 m × 8 m tiles, all flat (`MeshPlaneTerrainCfg`), friction 1.0.
- Contact sensor on every body, keeping 3 physics steps of history and tracking time in the air
  and on the ground. Used by the gait, feet and undesired-contact rewards.
- Height scanner (a grid of downward rays under the pelvis): defined, but **no observation or
  reward reads it**.

## 12. Learning algorithm

`tasks/locomotion/agents/rsl_rl_ppo_cfg.py` (PPO, from the `rsl_rl` library).

| Setting | Value |
| --- | --- |
| Network (policy and critic, separate) | 3 layers of 512, 256, 128, ELU activation |
| Starting action noise | 1.0 (standard deviation; each joint has its own, learned during training) |
| Steps collected per robot per iteration | 24 (so $4096 \times 24 = 98{,}304$ per iteration) |
| Passes over each batch / batches | 5 / 4 |
| Learning rate | $10^{-3}$, adjusted to keep the policy change per update near a KL divergence of 0.01 |
| Discount $\gamma$ / advantage smoothing $\lambda$ | 0.99 / 0.95 |
| Clip / value-loss weight / entropy weight | 0.2 / 1.0 / 0.01 |
| Max gradient norm | 1.0 |
| Iterations | 50,000 in the file; 09-17-v2 used 32,000 (`--max_iterations`) |
| Seed | 42 |

## 13. What must match the deployed robot

The Gazebo and real-robot stacks run the exported policy using `params/deploy.yaml` and the C++
observation code. Changing any of these means the deploy side must change too:

- Observation terms, their order, scales, history length (5) and history layout.
- `gait_phase`: period (0.8), the $\lVert c \rVert > 0.1$ gate, and the 3-component command
  norm, which `observations.h` must reproduce.
- Joint set and order (`joint_ids_map`), default angles, action scale (0.25), motor gains.
- Command ranges: `deploy.yaml` exports `limit_ranges`, which `policy_node` clamps `/cmd_vel` to.

Rewards, endings, randomization, the curriculum and PPO settings do not affect the deploy side.

## 14. Running experiments

The categories above are the things you can vary independently:

| Category | Section | Deploy change needed? |
| --- | --- | --- |
| Rewards: which terms, weights, parameters | 7 | No |
| Commands: ranges, resample time, heading-mode share, standing share, zero-snap | 5 | Only if ranges change (deploy clamp) |
| Curriculum: speed growth rule, turn-rate curriculum | 10 | No |
| Randomization and pushes | 9 | No |
| Episode endings | 8 | No |
| Observations: terms, noise, scales, history | 6 | **Yes** |
| Actions and motors: action scale, gains, delay | 3, 4 | **Yes** (gains and scale are exported) |
| Learning algorithm | 12 | No |
| Ground | 11 | No |

A workable routine:
1. **One idea per run.** Change one category, or one coherent set of related values, against a
   fixed baseline (currently 09-17-v2's configuration). Several changes at once can't be
   attributed.
2. **Name and record it.** Give the run a short name saying what changed, and keep a table: run
   name, wandb training run, the change, the reason, the eval result. The saved `params/` folder
   records what was trained, but not why.
3. **Evaluate every run the same way.** Use the Gazebo eval (`unitree_gz_eval`, default suite,
   3 repeats) and compare the `aggregate` runs in wandb on the `overall/...` metrics.
4. **Know what the eval tests that training doesn't.** The suite holds constant turn rates for
   3–4 s and pushes the standing robot at 1.0 m/s. Training never held a constant turn rate for
   long (section 5) and pushes at most about 0.71 m/s (section 9). Expect those tests to separate
   policies that change commands or pushes.
