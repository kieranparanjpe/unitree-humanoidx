# Handoff: Isaac Lab → ROS2/Gazebo sim2sim for H2 (Unitree)

Read `isaac-to-ros2-gazebo-plan.md` first for the original architecture rationale (`## 2`,
`## 8`, `## 9`). This file records what was built, the root cause of the long-running failure,
and what is left.

## Status: working

An Isaac Lab-trained H2 policy loads from its `deploy.yaml` manifest and **stands and walks**
in ROS 2 Jazzy + Gazebo Harmonic. Measured on the known-good checkpoint:

| | result |
| --- | --- |
| Standing (no command) | base height 1.033 m held for 90s, max &#124;ang_vel&#124; 0.38 rad/s |
| Walking (`linear.x = 0.5`) | 12.5 m in 30 s ≈ 0.425 m/s, upright throughout |
| Turning (`angular.z = 0.2`) | ~0.03 rad/s achieved — poor, see Open items |

Build and run instructions live in `unitree_ros2_gazebo/README.md`, not here.

## The root cause (read this before touching anything)

**`deploy.yaml` is not uniformly ordered.** From
`unitree_rl_lab/.../utils/export_deploy_cfg.py:30-36`:

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
hold with no policy in the loop*, and why the policy then saturated — it was reading a robot
already on its way to the floor. `load_dynamics()` in `policy_bridge_node.cpp` now gathers them
back through `joint_ids_map`. Verified against `UNITREE_H2_CFG`'s actuator groups by joint
name: 0 mismatches across all 31 joints, where the old code had 12.

**Why it hid for so long:** `default_joint_pos` *is* in the order everyone assumed, so the
spawn pose looked perfect and appeared to validate the ordering. This was the plan doc's `## 9`
first checklist item ("joint name/order parity"), and it had never been done. If you add a
robot, do that check first — it is a ten-line script.

## Where the code lives

`unitree_ros2_gazebo/` (submodule of the parent repo) — three ROS2 packages:

- **`unitree_gz_description/`** (Python) — `generate_urdf.py` turns a bare URDF +
  `deploy.yaml` into a Gazebo/`ros2_control`-ready URDF and controller config.
  `robots/h2.yaml` holds the only per-robot data that isn't a path and isn't derivable:
  `imu_link`, `joint_sdk_names`, `spawn_height`.
- **`unitree_gz_bringup/`** (Python) — `launch/sim.launch.py`, `worlds/default.sdf`, and
  `gz_reset_node.py`.
- **`unitree_policy_bridge/`** (C++) — `src/policy_bridge_node.cpp`, the manifest-driven
  policy node. Ported term-for-term from `unitree_rl_lab`'s C++ hardware deploy path.

`~/jazzy_ws/` (sibling to the parent repo, not inside it) is the colcon workspace; the three
packages are symlinked into `~/jazzy_ws/src/`.

**Reference code being mirrored** (read before changing the term registry):
`unitree_rl_lab/deploy/include/isaaclab/manager/{observation_manager.h,manager_term_cfg.h,action_manager.h}`,
`.../envs/mdp/actions/joint_actions.h`, `.../unitree_articulation.h`,
`deploy/robots/h1/src/State_RLBase.cpp` (H2 has no deploy reference of its own), and
`unitree_rl_lab/source/.../assets/robots/unitree.py`'s `UNITREE_H2_CFG`.

## Design decisions that still hold

- **No env vars, no assumption this repo lives inside the parent repo.** Every path
  (`urdf_path`, `policy_onnx_path`) is an explicit `ros2 launch` argument; `deploy.yaml` is
  derived as the sibling `params/` dir. `unitree_policy_bridge/CMakeLists.txt` has a
  hand-edited `ONNXRUNTIME_ROOT` placeholder (or pass `-DONNXRUNTIME_ROOT=`), by design.
- **Effort interface + self-computed PD**, not `gz_ros2_control` position mode. Mirrors what
  real H2 firmware does and what Isaac's implicit-PD actuator model does.
- **The PD runs at 1000 Hz, decoupled from the 50 Hz policy step.** `run_policy_step()`
  produces a joint position target; `run_pd_step()` converts it to torque on its own timer
  (`pd_update_rate_hz`, default 1000). Isaac solves its PD *inside* the physics solver, which
  makes the damping term unconditionally stable; an explicit PD held across a whole 20 ms
  control period is not, and the limit is set by the lightest joint (H2's wrists pair kd=1.0
  with ~1e-3 kg·m²). See Open items — this is sound reasoning but was not the bug.
- **The control loop runs on simulation time.** `rclcpp::create_timer` on the node clock, not
  `create_wall_timer` (which ignores `use_sim_time` entirely). The launch bridges `/clock`;
  without it nothing works. This also removes the old startup race and the stall-divergence
  mechanism — no sim time passes before Gazebo is up, so no policy step happens.
- **The start pose comes from `initial_value`**, a nested `<param name="initial_value">` on
  each joint's position state interface. `gz_ros2_control` turns it into a `JointPositionReset`
  component at spawn. It **must** be a nested element; `ros2_control` silently ignores it as an
  attribute — which is why an earlier attempt concluded, wrongly, that it doesn't work.
- **IMU is a plain gz-sim sensor** bridged via `ros_gz_bridge`, not a `gz_ros2_control`
  hardware-interface sensor. The `IMU sensor 'imu_sensor' not found in hardware_info` log line
  is expected. `imu_in_pelvis` has `rpy="0 0 0"`, so its frame is aligned with the pelvis.
- **Custom world file**, because gz's stock `empty.sdf` doesn't load `gz-sim-imu-system` and an
  IMU sensor silently never produces data on it.
- **No stand-up/settle phase.** The reset establishes the trained start state directly, so the
  policy runs from the first step, as Isaac does. A settle phase is redundant in simulation and
  was actively harmful — it spent 250 steps driving a PD toward a pose the robot was already
  in. On hardware, reaching the default pose is the controller FSM's job upstream of this node;
  `wait_for_reset` (bridge parameter, default `false`) is what makes simulation wait for the
  teleport instead.
- **Safety guard** (`publish_effort_checked()`): refuses any non-finite or >1e6 N·m torque.
  Keep it.

## The reset primitive (`gz_reset_node.py`)

Gazebo Harmonic has no `/gazebo/reset_world` — those are Gazebo **Classic** APIs and none of
them exist here. The equivalent of Classic's `set_model_configuration` is a direct ECM write,
which is what this node does:

- `/world/<w>/state` (`Empty` → `SerializedStepMap`) reads the ECM; `/world/<w>/control/state`
  (`WorldControlState` → `Boolean`) writes arbitrary components.
- Joints: `JointPositionReset` / `JointVelocityReset`, payload = serialized `gz.msgs.Double_V`.
- Base: `/world/<w>/set_pose` for pose, `WorldLinearVelocityCmd` / `WorldAngularVelocityCmd`
  for velocity. These have no explicit serializer, so their payload is the **ASCII** form of a
  `math::Vector3d` (`b"0 0 0"`), not protobuf. They are one-shot, not a pin — verified.
- Component type ids are `gz::common::hash64(type_name)`, i.e. FNV-1a/64 (`components/Factory.hh:254`).
  C++ callers use `components::JointPositionReset::typeId` instead. The node re-derives it in
  Python and **verifies it against `Name`/`Pose`/`ParentEntity` at startup**, because the
  failure mode is otherwise silent: an unrecognised type id writes a component nothing reads
  and the service still returns success.

Verified: a fully collapsed robot snaps back to 1.2e-4 rad joint error and 1e-11 m base pose.
Also exposed as `ros2 service call /gz_reset_node/reset std_srvs/srv/Trigger` for episode
resets — confirmed working mid-run on a standing robot.

`reset: {all: true}` also works but tears down `gz_ros2_control` (controllers unload,
`/joint_states` stops permanently). `reset: {model_only: true}` returns success and does
nothing. Don't use either.

### Two Gazebo traps this node works around

1. **`multi_step` batches with a following un-pause.** The server drains queued world-control
   requests in one pass, and `multi_step`'s "pause once the steps are done" is applied last, so
   an un-pause sent alongside a pending step burst is silently undone. The world then sits
   paused with `/clock` still republishing one instant, which stops every `use_sim_time` timer.
   `_start_resume()` retries and requires several consecutive confirmations that time is
   advancing.
2. **A `gz.transport` subscription on the same node breaks its blocking service requests.**
   Subscribing to gz's `/stats` made every `/world/<w>/control` and `/world/<w>/state` request
   time out. Step feedback is taken from ROS `/clock` instead.

The startup nudge is closed-loop against `/clock`: it won't request another burst until the
previous one has actually executed. Counting steps *requested* over-counted by 5-10x (queued
requests drain at the server's own pace), which previously let the robot fall for 0.4 s before
the reset instead of 0.07 s.

## Open items

1. **Yaw tracking is poor** — ~0.03 rad/s achieved for 0.2 rad/s commanded, versus ~85% for
   forward walking. The command reaches the observation vector correctly (the `cmd=[...]` field
   in the diag log confirms this), so this is not a plumbing bug. Most likely a property of
   this training run: `h2-isaac-training-notes.md` records that H2's Isaac config was copied
   from G1 and is materially less developed than H1's (no `gait_phase` observation, weaker
   reward weights), and the trained yaw range is only ±0.2 rad/s. **Next step:** check whether
   it turns this weakly in Isaac Lab too. If so the fix is training-side, per the plan already
   in the training notes.
2. **The PD-rate change is unproven.** Decoupling the PD from the policy rate is verifiably
   running (`pd_steps=1000` per policy second, efforts at ~918 Hz) and the wrist-inertia
   argument for it is sound, but it did **not** produce the improvement predicted — the gain
   ordering was the real bug. It may be fixing a real problem that simply wasn't this one.
   Don't cite it as proven.
3. **The observation-parity check still hasn't been done** (plan doc `## 9`, last item): diff
   `unitree_policy_bridge`'s computed observation vector against the C++ deploy reference's
   output given the same synthetic state. Now finally worth doing, since the initial state is
   correct and the robot walks — it would catch residual scale/clip/history/ordering errors
   that current behaviour is not sensitive enough to reveal.
4. **The mid-run stall and `last_action` runaway have not been re-tested.** Both should be gone
   with the sim-time control loop (the old wall timer free-ran on frozen observations), but
   nobody has tried to reproduce a stall since.
5. **Base velocity is only reset via the ECM path**, so a `~/reset` on a fast-moving robot is
   the least-tested route. Startup is safe because the world starts paused.
6. **Diagnostic logging is still in `policy_bridge_node.cpp`**, throttled to once/sec. It has
   earned its place repeatedly; `cmd=[...]` in particular is what distinguishes "command not
   arriving" from "policy tracks poorly". Next person's call whether to keep it.

## Method notes for the next agent

- **Background processes are reaped between tool calls.** A `ros2 launch ... &` in one call and
  an inspection in the next will show it dead, and worse, a `ros2 topic pub &` from a previous
  call is gone — so any measurement of commanded behaviour must have the publish *and* the
  sampling in **one** call. Several early measurements this session were garbage for exactly
  this reason.
- **`pkill -f <pattern>` matches your own shell's command line.** If your command text contains
  the pattern, you kill yourself and the tool call returns exit 1 with no output. Keep kill
  patterns in a separate script and never in the same call as a command mentioning them.
- **Measure, don't infer.** Three separate wrong conclusions this session came from reading
  logs and reasoning instead of instrumenting: "400 nudge steps" (requests, not executions),
  "base velocity can't be reset" (it can), and a startup timeline that didn't match what the
  user saw. Where a number can be printed, print it.
- **Measure at the right moment.** A reset measured 2 s after un-pausing looks like a failed
  reset, because the robot has already fallen back down. Pause, write, step 1-2, then read.
- `gz topic -i -t <topic>` for real publisher/subscriber counts; `gz topic -e -t /stats` for
  RTF and the `paused` flag; `gz service -is <service>` for request/response types.

## Convention: adding a new robot

Add `unitree_gz_description/robots/<name>.yaml` (`imu_link`, `joint_sdk_names`,
`spawn_height` — no paths), then launch with `robot:=<name> urdf_path:=... policy_onnx_path:=...`.
No code changes for a robot whose `deploy.yaml` uses the same term set as H2. A new observation
term type needs a branch in `observation_term_fn()`.

**Before running it, check gain/joint-order parity** (see root cause above) — assert each
joint's `stiffness`/`damping` against the actuator groups in its training config by name.
