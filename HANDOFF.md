# Handoff: Isaac Lab → ROS2/Gazebo sim2sim for H2 (Unitree)

`isaac-to-ros2-gazebo-plan.md` holds the original architecture rationale (`## 2`, `## 8`) and
is still worth reading for *why* the node is manifest-driven. Its `## 9` validation checklist is
now largely superseded — see Open items. This file records what was built, the two root causes
of the long-running failures, and what is left.

Build and run instructions live in `unitree_ros2_gazebo/README.md`, not here.

## Status: working

An Isaac Lab-trained H2 policy loads from its `deploy.yaml` manifest and **stands and walks** in
ROS 2 Jazzy + Gazebo Harmonic.

| | result | when |
| --- | --- | --- |
| Standing (no command) | upright for the whole run, `proj_grav_b` → `[0,0,-1.000]`, actions never saturate | verified 2026-09-16 |
| Reset from a fully collapsed robot | all 31 joints snap to `default_joint_pos`, robot stands | verified 2026-09-16 |
| Mid-run `~/reset` on a standing robot | `success=True`, still standing afterwards | verified 2026-09-16 |
| Walking (`linear.x = 0.5`) | ≈0.41 m/s with `unitree_isaac_policy`, ≈0.36 m/s with the bridge | earlier session, not re-measured |
| Turning (`angular.z = 0.2`) | ~0.03 rad/s achieved — poor, see Open items | earlier session |

The rows above describe the checkpoint the stack was brought up on. A later H2 retrain
(2026-09-16, the first with a `gait_phase` observation) **regressed**: unchanged yaw tracking,
marching in place at zero command, and a leftward veer walking straight. All three are
training-side — see *H2 retrain config*. The deploy path reproduces faithfully what the policy
learned; it is not implicated.

## Root cause 1: `deploy.yaml` is not uniformly ordered

From `unitree_rl_lab/.../utils/export_deploy_cfg.py:30-36`:

```python
stiffness = np.zeros(len(joint_sdk_names))
stiffness[joint_ids_map] = asset.data.default_joint_stiffness[0]   # scattered → SDK motor order
damping[joint_ids_map]   = asset.data.default_joint_damping[0]     # scattered → SDK motor order
cfg["default_joint_pos"] = asset.data.default_joint_pos[0]         # raw → Isaac env order
```

`stiffness` and `damping` are the **only** arrays in SDK motor order. `default_joint_pos`, the
action `scale`/`offset`/`clip`, and the observation scales all come straight out of Isaac-order
tensors. Reading them all positionally gives every joint some other joint's gains:

```
right_knee_joint       kp= 40 → 200      waist_yaw_joint        kp=200 → 150
right_hip_roll_joint   kp= 40 → 200      left_ankle_roll_joint  kp=150 →  40
```

A knee at kp=40 cannot hold up a 75 kg robot. This is why H2 collapsed *even under a plain PD
hold with no policy in the loop*. Both stacks now re-index through `joint_ids_map`:
`load_gains()` in `unitree_isaac_policy`'s `include/unitree_isaac_policy/pd.hpp`,
`load_dynamics()` in the bridge. Verified against `UNITREE_H2_CFG`'s actuator groups by joint
name: 0 mismatches across all 31 joints, where the old code had 12.

**Why it hid:** `default_joint_pos` *is* in the order everyone assumed, so the spawn pose looked
perfect and appeared to validate the ordering. If you add a robot, check this first — it is a
ten-line script.

## Root cause 2: one ECM write per simulation step, or it is silently discarded

A `JointPositionReset` write lands correctly **on its own**. Issue anything else before the
next physics step — a `set_pose`, or a second `/world/<w>/control/state` call — and the joint
write is thrown away while every service still returns success. Measured, same robot, same
collapsed start:

| writes issued before the next step | resulting max &#124;q − default&#124; |
| --- | --- |
| joints only | 3.025 → **0.007 rad** |
| joints, then base-velocity (2 `control/state` calls) | 0.809 rad |
| joints + `set_pose` | 1.013 rad |
| joints+velocity combined in one call, then `set_pose` | 1.157 rad |

`_reset()` therefore writes base pose and velocity, steps once, writes the joints, steps again,
and only then un-pauses.

**Why it hid for the entire project:** the old implementation batched all three writes, so its
joint reset had **never** worked. Nothing noticed, because the world used to start paused and
the robot never left the pose `initial_value` gave it — so there was nothing for the reset to
correct and a no-op looked identical to success. Its own log said so every run (`pose error
before reset 0.0000 rad`) and that read as healthy. A previous version of this document claimed
"verified: a fully collapsed robot snaps back to 1.2e-4 rad" — **that was not reproducible and
the mechanism says it could not have been true.**

If you change the write sequence, test it against a robot that has genuinely fallen over. A
robot already in the right pose cannot tell you anything.

## Where the code lives

`unitree_ros2_gazebo/` (submodule of the parent repo) — five ROS2 packages:

- **`unitree_gz_description/`** (Python) — `generate_urdf.py` turns a bare URDF + `deploy.yaml`
  into a Gazebo/`ros2_control`-ready URDF and controller config. `robots/h2.yaml` holds the only
  per-robot data that isn't a path and isn't derivable: `imu_link`, `joint_sdk_names`,
  `spawn_height`. Depends on nothing but `python3-yaml`, deliberately — both policy packages
  import it, and the hardware path has no Gazebo.
- **`unitree_gz_bringup/`** (Python) — `launch/sim.launch.py`, `launch_utils.py`,
  `worlds/default.sdf`, `gz_reset_node.py` (the reset), `gz_ecm.py` (the ECM writes as plain
  functions).
- **`unitree_isaac_policy/`** (C++) — `policy_node` + `pd_node`. **The stack to use.**
- **`unitree_policy_bridge/`** (C++) — `policy_bridge_node`, the older single-node
  re-implementation. Legacy; being retired.
- **`unitree_keyboard_teleop/`** (C++) — `keyboard_controller`, optional WASD/arrow teleop onto
  `/cmd_vel`.

Dependency direction, which matters — `launch_utils.py` used to sit in `unitree_gz_description`
and include `sim.launch.py` from `unitree_gz_bringup`, making the two mutually dependent:

```
unitree_gz_description  ->  (nothing internal)
unitree_gz_bringup      ->  unitree_gz_description
unitree_isaac_policy    ->  unitree_gz_description, unitree_gz_bringup
unitree_policy_bridge   ->  unitree_gz_description, unitree_gz_bringup
unitree_keyboard_teleop ->  (nothing internal)
```

`~/humanoidx/jazzy_ws/` (outside this repo) is the colcon workspace; the repo is symlinked into
`src/`.

### Why `unitree_isaac_policy` over `unitree_policy_bridge`

`policy_node` reuses `ManagerBasedRLEnv`, `ObservationManager`, `ActionManager`, `OrtRunner` and
the mdp term registries from `unitree_rl_lab/deploy` **unchanged**. Only two substitutions: a
`RosArticulation` feeding state from ROS topics instead of a DDS `LowState`, and a
`velocity_commands` term reading `/cmd_vel` instead of the handheld joystick. Observation parity
with the DDS-hardware and `unitree_mujoco` paths is therefore structural, not something to test.
The bridge re-implements the terms in an if-else chain covering only the six H2 uses.

**Reference code being mirrored** (read before touching the term registry):
`unitree_rl_lab/deploy/include/isaaclab/manager/{observation_manager.h,manager_term_cfg.h,action_manager.h}`,
`.../envs/mdp/observations/observations.h`, `.../unitree_articulation.h`, and
`unitree_rl_lab/source/.../assets/robots/unitree.py`'s `UNITREE_H2_CFG`.

`unitree_rl_lab/deploy/` and `unitree_rl_mjlab/deploy/` are indexed as their own
codebase-memory projects — the indexer's skip-list drops any directory named `deploy`.

## Design decisions that still hold

- **No env vars, no assumption this repo lives inside the parent repo.** Every path
  (`urdf_path`, `policy_onnx_path`) is an explicit `ros2 launch` argument; `deploy.yaml` is
  derived as the sibling `params/` dir. External roots are CMake cache variables with editable
  placeholders, by design.
- **Effort interface + self-computed PD**, not `gz_ros2_control` position mode. Mirrors what
  real H2 firmware and Isaac's implicit-PD actuator model do.
- **The PD runs at 1000 Hz, decoupled from the 50 Hz policy step.** `policy_node` emits a joint
  position target; `pd_node` converts it to torque on its own timer. Isaac solves its PD *inside*
  the physics solver, which makes the damping term unconditionally stable; an explicit PD held
  across a whole 20 ms control period is not, and the limit is set by the lightest joint (H2's
  wrists pair kd=1.0 with ~1e-3 kg·m²). See Open items — sound reasoning, still unproven.
- **The control loop runs on simulation time.** `rclcpp::create_timer` on the node clock, not
  `create_wall_timer` (which ignores `use_sim_time`). The launch bridges `/clock`; without it
  nothing works. No sim time passes before Gazebo is up, so no policy step happens early.
  `gz_reset_node` and `keyboard_controller` are the deliberate exceptions — both must tick in
  wall time regardless of what the simulator is doing.
- **The start pose comes from `initial_value`**, a nested `<param name="initial_value">` on each
  joint's position state interface, which `gz_ros2_control` applies as a teleport at spawn. It
  **must** be a nested element; `ros2_control` silently ignores it as an attribute.
- **IMU is a plain gz-sim sensor** bridged via `ros_gz_bridge`, not a `gz_ros2_control`
  hardware-interface sensor. `IMU sensor 'imu_sensor' not found in hardware_info` is expected.
  `imu_in_pelvis` has `rpy="0 0 0"`, so its frame is aligned with the pelvis.
- **Custom world file**, because gz's stock `empty.sdf` doesn't load `gz-sim-imu-system` and an
  IMU sensor silently never produces data on it.
- **No stand-up/settle phase.** The reset establishes the trained start state directly, so the
  policy runs from the first step, as Isaac does. On hardware, reaching the default pose is the
  controller FSM's job upstream of these nodes.
- **Safety guard** in `pd_node`: refuses any non-finite or >1e6 N·m torque, publishes zero
  effort and stops loudly. Keep it.

## `policy_node`'s reset state machine

Three phases, one enum, three transitions:

```
ctor  --wait_for_reset param-->  AwaitingReset | ResetTriggered
any   --/policy_reset (zeroes both message counters)-->  ResetTriggered
ResetTriggered --both counters > 0, calls env_->reset()-->  Running
```

`env_->reset()` fills every observation term's history `history_length` deep (5, for all six of
H2's terms) with the *current* state and zeroes `last_action` — so it must run on post-teleport
data. That deferral is the whole point of the counter reset: at the instant `/policy_reset`
arrives the cached state is still pre-teleport.

This replaced nine coordinating members with three. Note that `ManagerBasedRLEnv`'s constructor
seeds the history from whatever state the articulation holds at construction (all zeros), so a
path that never calls `env_->reset()` runs its first five steps on a fabricated history — which
is what the hardware path used to do.

## The reset primitive (`gz_reset_node.py` + `gz_ecm.py`)

Gazebo Harmonic has no `/gazebo/reset_world` — those are Gazebo **Classic** APIs and none of
them exist here. The equivalent of Classic's `set_model_configuration` is a direct ECM write:

- `/world/<w>/state` (`Empty` → `SerializedStepMap`) reads the ECM; `/world/<w>/control/state`
  (`WorldControlState` → `Boolean`) writes arbitrary components.
- Joints: `JointPositionReset` / `JointVelocityReset`, payload = serialized `gz.msgs.Double_V`.
  Confirmed against `VectorDoubleSerializer` in gz-sim's `components/Serialization.hh`.
- Base: `/world/<w>/set_pose` for pose, `WorldLinearVelocityCmd` / `WorldAngularVelocityCmd` for
  velocity. These have no explicit serializer, so their payload is the **ASCII** form of a
  `math::Vector3d` (`b"0 0 0"`), not protobuf. One-shot, not a pin — verified.
- Component type ids are `gz::common::hash64(type_name)`, i.e. FNV-1a/64. C++ callers use
  `components::JointPositionReset::typeId` instead. `verify_type_ids()` re-derives it in Python
  and checks it against `Name`/`Pose`/`ParentEntity` at startup. **Note this check is weaker
  than it looks** — it proves the hash function matches for components that happen to be
  present, not that `JointPositionReset` is being consumed. It did not catch root cause 2.

**The world runs from the start (`gz sim -r`), and the robot is allowed to fall.** Nothing
commands it until the policy engages; `gz_reset_node` waits `settle_seconds` (default 3.0) for
it to come to rest, then teleports it back. Starting the world paused is **not** an option:
controller activation is serviced by `controller_manager`'s update loop, which under
`gz_ros2_control` only runs on simulation steps, so a paused world never finishes activating and
the spawners time out.

An earlier design started paused and stepped the world forward in bursts ("nudging") to get the
controllers up without letting the robot sag. That cost ~90 lines — a closed-loop step burst and
a retried un-pause, because gz queues world-control requests and each queued `multi_step`
re-pauses the world as it drains, silently undoing an un-pause sent alongside it. All of it was
removable once the joint reset actually worked: the node went from 282 code lines to 98 plus
106 shared in `gz_ecm.py`.

`reset: {all: true}` also works but tears down `gz_ros2_control` (controllers unload,
`/joint_states` stops permanently). `reset: {model_only: true}` returns success and does
nothing. Don't use either.

**One gz trap that still bites:** a `gz.transport` subscription on the same node breaks its
blocking service requests — subscribing to gz's `/stats` made every `/world/<w>/control` and
`/world/<w>/state` request time out. Don't add one.

## H2 retrain config (2026-09-16) and the `env->command` side channel

Two things changed together: H2's Isaac Lab reward/command config, and one shared header in
`unitree_rl_lab/deploy`. They are coupled — the config change is only correct if the deploy
change ships with it. Written but **not yet trained or run**.

### Why the 2026-09-16 checkpoint regressed

- **Yaw was never trained wide.** `ranges.ang_vel_z` sat at ±0.1 for the whole run.
  `limit_ranges` is only a ceiling the curriculum grows toward, and the sole registered command
  curriculum, `lin_vel_cmd_levels`, advances `lin_vel_x`/`lin_vel_y` and nothing else
  (`unitree_rl_lab/.../locomotion/mdp/curriculums.py:26-35`). An `ang_vel_cmd_levels` sits right
  beside it, registered by no robot. Meanwhile `export_deploy_cfg` exports **`limit_ranges`**, so
  `deploy.yaml` advertised ±0.5 and `policy_node` clamped `/cmd_vel` to a range 5× wider than
  training ever sampled. Widening `limit_ranges` alone — which an earlier session did, believing
  it was the fix — changes nothing about training and *widens* the train/deploy gap.
- **Marching in place at zero command.** `gait_phase` was an ungated clock, so the observation
  vector was periodic with no command given, and nothing in the reward set opposed a periodic
  output: `feet_gait` gates *itself* off below `‖cmd‖ = 0.1`, `track_lin_vel_xy` is satisfied by
  marching (net velocity ≈ 0), and `rel_standing_envs` was 0.02 so the regime was barely trained.
  `feet_contact_without_cmd` exists in `rewards.py` and is registered by no robot; mjlab's
  equivalent, `stand_still`, is what makes its H2 stand.
- **`feet_clearance` at weight 20 was a bonus, not a penalty.** `foot_clearance_reward` is
  `exp(-Σ(z−h)²·tanh(2|v_xy|)/std)` — it *peaks* when the feet are still. At weight 20 it paid
  ~20/step for standing, and pivoting the feet near the ground cost more than `track_ang_vel_z`
  (max 0.5) could pay back, so turning in place was net-negative.
- **Veering** is the weakest of the three diagnoses. Nothing observes heading
  (`heading_command=False`, no yaw term in either obs group), so straight walking is open-loop in
  yaw and any gait asymmetry integrates freely. What tipped it is likely `flat_orientation_l2`
  −5.0 → −1.0 together with `ang_vel_xy_l2` −0.05 → −0.5: a static tilt got cheap at the same
  time as correcting one got expensive. Not addressed below.

### Changes made

All in `robots/h2/velocity_env_cfg.py` unless noted. Ported from `unitree_rl_mjlab`'s H2 velocity
task, which does not exhibit any of the three symptoms.

| | from | to |
| --- | --- | --- |
| `ranges.ang_vel_z` / `limit_ranges.ang_vel_z` | ±0.1 / ±0.5 | **±1.0 / ±1.0** |
| `resampling_time_range`, `rel_standing_envs` | (10,10) s, 0.02 | (3,8) s, 0.05 |
| `gait_phase`, both obs groups | `{period: 0.8}` | + `command_name`, `command_threshold: 0.1` |
| `track_ang_vel_z` | weight 0.5, std √0.25 | weight 1.0, std √0.5 |
| `feet_slide` | `mdp.feet_slide`, −0.2 | `mdp.feet_slide_when_moving`, −0.25 |
| `feet_clearance` | `foot_clearance_reward`, **+20.0** | `feet_clearance_penalty`, **−1.0** |

New in `locomotion/mdp/`: `gait_phase` gained optional `command_name`/`command_threshold`
(default `None` = ungated, so H1 is unchanged); `rewards.py` gained `feet_clearance_penalty`
(mjlab's linear `Σ|z−h|·|v_xy|`, gated) and `feet_slide_when_moving` (a gate wrapping upstream
`feet_slide` rather than reimplementing it). Pre-existing `foot_clearance_reward` and
`feet_slide` are untouched, so g1/h1/go2/b2 are unaffected.

Observation width is unchanged (still 5×104), so no ONNX or launch change. Rewards,
terminations, events and curricula are **not exported to `deploy.yaml` at all** — only the yaw
range and the new `gait_phase` params cross into deploy.

Deliberately not done, so the retrain stays interpretable: mjlab's `stand_still` and
`variable_posture` (both deploy-invisible, they are rewards), and its reward economy —
`is_terminated` −200 in place of an `alive` bonus, orientation penalties moved from root to
`torso_link`, `bad_orientation` loosened to 70°.

### Run `yhyw3t6n` collapsed: the survival economy

The first retrain with the changes above (wandb `kieranparanjpe-mcgill-university/isaaclab/yhyw3t6n`,
log_dir `2026-09-16_15-27-10`) **died at ~iteration 1,200 and ran dead for 14,000 more**:

| | |
| --- | --- |
| `Train/mean_episode_length` | exactly 3.00, unchanged for 14k iterations (3 steps = 0.06 s) |
| `Episode_Termination/base_contact` | 1.0 — every episode; `time_out` 0 |
| `Train/mean_reward` vs episode length | **−1.32 → +0.026** as length went **9.57 → 3** |
| `Policy/mean_noise_std` | 0.937 → 0.026; `Loss/learning_rate` floored at 1e-5 from iter 50 |

Reward rising as episodes shorten is the signature: the policy learned that **ending the episode
immediately beat playing it**. Cause: `foot_clearance_reward` is `20·exp(−…)`, which peaks when
the feet are still, so at weight 20 it was quietly paying ~+20/step (~+400/episode) for merely
existing — the only thing making survival worthwhile. Replacing it with a −1.0 penalty was right
in shape and removed that subsidy, leaving `alive` at +0.003/step against a measured −0.138/step
cost of exploring. With no penalty for terminating, suicide was optimal and PPO found it.

mjlab's survival economy has three legs; this config had none of them:

| | mjlab | rl_lab H2 as-run |
| --- | --- | --- |
| termination penalty | `is_terminated` −200 | none |
| dense positive for standing | `variable_posture` +1.0/step | none (only `joint_deviation` penalties) |
| terminations | `time_out`, `bad_orientation` 70° | + `base_contact` (1 N torso), + `base_height`, `bad_orientation` **45.8°** |

Fixes applied:

- `alive` 0.15 → **5.0**. Restores the dense positive at a sane magnitude: +100/episode for
  surviving against ~+50 for tracking perfectly, where the accidental subsidy was +400 against
  the same +50. A constant cannot distort *how* the robot moves, unlike the foot-velocity term
  it replaces.
- **`is_terminated` −200** added (mjlab's value; dt-scaled that is −4 per event).
- **`base_contact` removed.** 3 steps is far too fast to have fallen — free-fall from 1.03 m is
  ~450 ms. The scene's `contact_forces` sensor spans `Robot/.*` with an empty
  `filter_prim_paths_expr` and the articulation sets `enabled_self_collisions=True`, so
  `illegal_contact` on `torso_link` fires on **self**-contact; an arm at ~19 rad/s reaches its
  own torso almost instantly. Ground-only filtering would need a filtered `ContactSensor` plus a
  termination reading `force_matrix_w` — machinery this repo has nowhere else — so matching
  mjlab's termination set was the cheaper correct answer.
- `bad_orientation` 0.8 rad → **70°** (mjlab's). 45.8° ends the episode on a recoverable lean,
  which is survivable when termination is free but not once it costs −4.

**Convention confirmed empirically from this run:** IsaacLab scales every reward term by
`step_dt`. `alive` at 0.15 logged `Episode_Reward/alive` = 0.0003 = 0.15 × 0.02 × 2 ÷ 20, which
also confirms `is_terminated` fires exactly once (2 alive steps + 1 terminating step = length 3).

### The deploy-side half: `env->command`

Gating `gait_phase` in Python is only correct if C++ gates identically. A checkpoint trained with
the gate, run against an ungated clock, marches at zero command anyway — the exact bug the gate
removes, reintroduced at deploy.

But a C++ observation term is handed only `(env, params)` (`manager_term_cfg.h:16`), and the
command is produced *inside* whichever `velocity_commands` term is registered — three
implementations reading three different sources: the stock joystick one
(`observations.h:111`), `keyboard_velocity_commands` in g1_29dof's `State_RLBase.cpp:11`, and
`policy_node`'s `/cmd_vel` lambda. None of it lands anywhere `gait_phase` can reach.

As shipped: a `std::vector<float> command` on `ManagerBasedRLEnv`, written by all three
`velocity_commands` implementations, read by `gait_phase`. This follows the existing idiom —
`env->global_phase` already exists solely as scratch state for `gait_phase`.

**Backward compatible.** The gate sits behind `params["command_name"].IsDefined()`, and older
`deploy.yaml`s don't carry that key (2026-09-16 has `params: {period: 0.8}`; 09-11 has no
`gait_phase` term at all). Both run byte-identically against the new code, and a registered term
a `deploy.yaml` never names is never called — `_prepare_group_terms` iterates the yaml, not the
registry.

**Known brittle, see Open items.** The invariant *"every `velocity_commands` implementation must
also write `env->command`"* is unenforced and invisible at the point where a fourth one would be
written. Miss it and `env->command` stays `{0,0,0}`, `gait_phase` gates off permanently, and the
robot stands frozen with no error. Secondary: `gait_phase` reads what `velocity_commands` wrote
earlier in the same `compute_group` loop, correct only because `deploy.yaml` emits terms in
config order.

The better shape, designed but **not implemented**: have `ObservationManager` record each term's
raw output by name — a `name` field on `ObservationTermCfg` (already computed as `term_name` in
`_prepare_group_terms`), a `last_raw_` map, and a `last(name)` accessor returning a pointer — and
have `gait_phase` pull from it. That removes the write discipline entirely (any
`velocity_commands` implementation works, including unwritten ones) and turns the ordering
dependency into a loud throw instead of a stale read. Costs three call sites in
`ObservationManager` — `compute_group`, `reset`, and the construction-time call in
`_prepare_group_terms`, which all invoke `term.func` — and keys by term name alone, which
collides only if a `deploy.yaml` ever carried two observation groups. Deploy runs the single flat
group, so that is theoretical but is a real assumption.

## Open items

1. **Yaw tracking is poor** — ~0.03 rad/s achieved for 0.2 rad/s commanded, versus ~85% for
   forward walking, and unchanged by the 2026-09-16 retrain. Diagnosed training-side (see *H2
   retrain config*): yaw was only ever sampled at ±0.1 while `deploy.yaml` advertised ±0.5, and
   `feet_clearance` at weight 20 made turning in place net-negative. Config fixes are written and
   **untrained**. The command does reach the observation vector correctly — the `cmd=[...]` diag
   field confirms it — so this was never a plumbing bug.
   **Cheapest confirmation before spending a retrain:** run `play.py` on the 09-16 checkpoint and
   watch an env whose command is near zero. Marching there too ⇒ diagnosis holds. Standing in
   Isaac but marching in Gazebo ⇒ the cause is on the deploy path instead and the above is wrong.
2. **The PD-rate change is unproven.** Decoupling the PD from the policy rate is verifiably
   running and the wrist-inertia argument is sound, but it did not produce the improvement
   predicted — the gain ordering was the real bug. Don't cite it as proven.
3. **Walking and turning have not been re-measured** since the reset fix, the `Phase` rework, or
   the move to `unitree_isaac_policy`. The numbers in Status are inherited from an earlier
   session. Standing is confirmed; distance-over-time is not.
4. **`unitree_policy_bridge` has not been re-tested** against any of this session's changes, and
   its reset handling differs from `policy_node`'s. If it is staying, it needs a pass; if it is
   going, delete it.
5. **`pd_node` does not clear its target on reset**, where `policy_bridge_node` does. During the
   re-seed window it keeps driving toward the pre-teleport target. Harmless at current rates —
   the window is 1–2 ms and the world is paused across the teleport — but mitigated by timing
   rather than by design.
6. **Observation parity against the C++ deploy reference** (plan doc `## 9`, last item) is now
   moot for `unitree_isaac_policy` — it *is* that code. It remains undone for the bridge.
7. **Diagnostic logging is in `policy_node`**, throttled to once/sec. `cmd=[...]` in particular
   is what distinguishes "command not arriving" from "policy tracks poorly". Keep it.
8. **`env->command` is a brittle side channel.** Every `velocity_commands` implementation has to
   remember to write it; one that forgets leaves `gait_phase` gated off forever and the robot
   standing frozen, with no error. The pull-based replacement is designed in *H2 retrain config*
   and deliberately not built yet. Until it is, a new `velocity_commands` implementation is a
   code change with an invisible requirement attached.
9. **Possible permanent self-contact at H2's default pose.** In `yhyw3t6n`,
   `Episode_Reward/undesired_contacts` sat at exactly one non-ankle body in contact, from spawn.
   With `enabled_self_collisions=True` and an unfiltered contact sensor, that is most likely an
   arm resting on the torso in the default pose — which would mean `undesired_contacts` charges
   a constant −0.02/step tax the policy can never avoid. **Watch that metric on the next run:**
   if it stays pinned at exactly one body, the default pose or the collision geometry needs a
   look. mjlab avoids this class of problem with a dedicated self-collision sensor and a
   `force_threshold` of 10 N, not 1 N.
10. **Remaining deviations from mjlab, deliberately not changed** (each is a candidate if the
   next run still misbehaves): `joint_deviation_*` penalties where mjlab has a `variable_posture`
   bonus; `reset_robot_joints` velocity range ±1.0 rad/s where mjlab resets to zero; starting
   `lin_vel_x` at ±0.1 where mjlab's stage 0 is (−0.5, 1.0), so early training is turn-heavy;
   `base_height_l2` −10 and `dof_pos_limits` −5, neither of which mjlab has at those values.
11. **Nothing checks Python↔C++ observation parity.** `export_deploy_cfg` carries a term's *name
   and params* into `deploy.yaml` and nothing else; the math is hand-written twice. A new term
   name fails loudly at startup (`Observation term 'X' is not registered`), but a changed
   implementation — `gait_phase`'s gate is exactly this — diverges silently and costs a training
   run to discover. The fix is a parity test: record an observation trace from `play.py`, feed
   the same inputs through the C++ `ObservationManager`, diff the vectors. Does not exist.

## Method notes for the next agent

- **Kill stale processes before trusting any result.** Leftover `policy_node` processes survive
  a killed launch, and a second policy publishing `/joint_targets` makes the robot flail and
  fall — which is *exactly* the signature of a broken reset. One run was misdiagnosed this way.
  `ps aux | grep "[p]olicy_node"` before believing a failure. A surviving `gz sim` server is
  just as bad: the next launch reports `Controller already loaded` / `Failed to configure
  controller` and comes up with no joints in the ECM.
- **`pkill -f <pattern>` matches your own shell's command line.** If your command text contains
  the pattern you kill yourself: exit 1, no output. Keep kill patterns in a separate script
  **and** never run that script in the same call as a command mentioning them — writing the
  script and running it must be two separate steps.
- **Background processes are reaped between tool calls.** Any measurement of commanded behaviour
  must have the publish *and* the sampling in one call. Use `setsid` + `kill -- -$PGID` to take
  down a whole launch tree.
- **Measure, don't infer.** Wrong conclusions in this project have come from reading logs and
  reasoning instead of instrumenting: "400 nudge steps" (requests, not executions), "base
  velocity can't be reset" (it can), "the teleport can't recover a collapsed robot" (it can —
  the joint write was being discarded), and a startup timeline that didn't match what the user
  saw. Where a number can be printed, print it.
- **Measure at the right moment.** A reset measured 2 s after un-pausing looks like a failed
  reset, because the robot has moved since. The old before/after pose-error diagnostic sampled
  "after" 1.5 s post-un-pause with the policy actively balancing, so it reported the policy's own
  displacement as evidence of a failed teleport. It has been removed.
- **A diagnostic that cannot fail is not a diagnostic.** Both safeguards around root cause 2 —
  the pose-error check and `verify_type_ids()` — were blind to it for the same reason: neither
  was ever exercised on a robot that had actually moved.
- `gz topic -i -t <topic>` for real publisher/subscriber counts; `gz topic -e -t /stats` for RTF
  and the `paused` flag; `gz service -is <service>` for request/response types.

## Convention: adding a new robot

Add `unitree_gz_description/robots/<name>.yaml` (`imu_link`, `joint_sdk_names`, `spawn_height` —
no paths), then launch with `robot:=<name> urdf_path:=... policy_onnx_path:=...`. No code changes
for a robot whose `deploy.yaml` uses a term set the stack already supports.

A new **observation term type** needs code only for the bridge (a branch in
`observation_term_fn()`); `unitree_isaac_policy` inherits everything upstream registers in
`observations.h`, `gait_phase` included.

Changing what an **existing** term computes is the dangerous case: `deploy.yaml` carries only the
term's name and params, so the two implementations have to be kept in agreement by hand and a
mismatch is silent. `gait_phase`'s command gate is the current example — Python in
`locomotion/mdp/observations.py`, C++ in `observations.h`, with a comment on each pointing at the
other. See Open items 8 and 11.

**Before running it, check gain/joint-order parity** (root cause 1) — assert each joint's
`stiffness`/`damping` against the actuator groups in its training config by name.
