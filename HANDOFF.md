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

## Open items

1. **Yaw tracking is poor** — ~0.03 rad/s achieved for 0.2 rad/s commanded, versus ~85% for
   forward walking. The command reaches the observation vector correctly (the `cmd=[...]` field
   in the diag log confirms this), so this is not a plumbing bug. Most likely a property of this
   training run: `h2-isaac-training-notes.md` records that H2's Isaac config was copied from G1
   and is materially less developed than H1's (no `gait_phase` observation, weaker reward
   weights), and the trained yaw range is only ±0.2 rad/s. **Next step:** check whether it turns
   this weakly in Isaac Lab too. If so the fix is training-side.
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

**Before running it, check gain/joint-order parity** (root cause 1) — assert each joint's
`stiffness`/`damping` against the actuator groups in its training config by name.
