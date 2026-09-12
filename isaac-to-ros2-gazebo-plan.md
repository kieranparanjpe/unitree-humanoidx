# Isaac → ROS2 + Gazebo Sim2Sim Plan

Goal: take a policy trained in Isaac Lab (`unitree_rl_lab`) and run it in a ROS2 + Gazebo
simulation, using the same obs/action contract the policy was trained with. This is a new
integration in this project — none of the four Unitree repos we've surveyed ship this
prebuilt, for any robot.

## 1. Architecture

Two things need to exist, and they're mostly independent of which robot:

```
[URDF + <ros2_control> tag + gazebo_ros2_control plugin]
        |
   Gazebo physics
        |
 controller_manager  --(joint_state_broadcaster)--> /joint_states
        |                                                 |
 effort/position controller <--(command topic)--  [policy inference node]
                                                           |
                                                  ONNX policy (.onnx export
                                                  from the Isaac Lab checkpoint)
```

- The **description + `ros2_control` + Gazebo plugin** side is robot-specific asset work
  (URDF, joint list, plugin config) but not "AI" work — it's the same shape for every robot.
- The **policy inference node** should NOT be hand-written per robot. Isaac Lab already has
  a generic, data-driven mechanism for exactly this problem (see §2) — the node should be a
  thin ROS2 wrapper around that same mechanism: a small named-term registry plus a
  manifest-driven composer, not a hardcoded obs vector per robot.

Note: this Gazebo path does **not** need `unitree_ros2`/DDS/`unitree_hg` at all — those are
only relevant if we later want the *same* policy node to also drive real hardware, via a
custom `ros2_control` hardware interface that translates to DDS. That's a follow-on, not a
prerequisite for sim2sim.

## 2. The observation/action contract mechanism (already solved — reuse it)

This is the key finding that should drive the node's design. Isaac Lab's own MuJoCo/real-
hardware sim2sim deploy path does not hand-maintain two copies (Python training config vs.
C++ runtime) in sync. Instead:

1. **A generic C++ runtime mirrors Isaac Lab's Python manager architecture term-for-term.**
   `unitree_rl_lab/deploy/include/isaaclab/manager/observation_manager.h` and
   `action_manager.h` reimplement Isaac Lab's `ObservationManager`/`ActionManager`. Each MDP
   term (`base_ang_vel`, `projected_gravity`, `joint_pos_rel`, `velocity_commands`, etc.) is
   hand-implemented **once** in
   `deploy/include/isaaclab/envs/mdp/observations/observations.h` and self-registers into a
   string-keyed lookup table via a `REGISTER_OBSERVATION(name)` macro — a name→function
   registry mirroring Python's `mdp.<func>` module. This is per-term-type work, not
   per-robot work.

2. **A YAML manifest is auto-exported from the live training env, not hand-written.**
   `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/utils/export_deploy_cfg.py` runs
   automatically at the end of every training run (`scripts/rsl_rl/train.py:196`). It
   introspects the *actual instantiated* `env.observation_manager`/`env.action_manager`
   objects — term names, order, scale, clip, history_length, resolved joint gains/default
   pose, command ranges — and writes `logs/.../params/deploy.yaml`. Because it reads live
   objects instead of re-deriving from config classes, it cannot drift out of sync with
   training.

3. **The joint-order mapping is computed, not memorized.** `deploy.yaml`'s `joint_ids_map`
   comes from `resolve_matching_names(asset.data.joint_names, joint_sdk_names,
   preserve_order=True)` — matching Isaac Lab's internal joint order against a
   `joint_sdk_names` list declared once per robot in its asset config
   (`source/.../assets/robots/unitree.py`).

4. **At runtime, the C++ side loads the YAML and dispatches by name** — no per-robot C++
   code beyond pointing a `policy_dir` at the right log dir
   (see `deploy/robots/h1/config/config.yaml`'s `Velocity.policy_dir`).

**This already exists for H2, with zero extra Python-side work:**
- `UNITREE_H2_CFG.joint_sdk_names` is already defined in `unitree.py`, explicitly sourced
  from `unitree_sdk2`'s `H2JointIndex` enum (`example/h2/low_level/h2_ankle_swing_example.cpp`).
- `unitree_rl_lab/logs/rsl_rl/unitree_h2_velocity/2026-09-10_22-04-06/params/deploy.yaml` is
  already sitting in the repo, fully populated: H2's `joint_ids_map`, per-joint
  stiffness/damping, default pose, and the complete observation/action term spec.

**Design implication for the ROS2 node:** port the term registry (the small set of
functions in `observations.h`/the action equivalent) into the node's language, and have the
node parse `deploy.yaml` directly to build its observation/action pipeline — the same node
then works for H1, H2, or any future Isaac Lab robot, with the manifest as the only
per-robot input. This replaces the earlier idea of "port obs-construction logic from
`State_RLBase.h`" — don't port the *specific* H1 logic, port the *generic mechanism*.

**Open gap — mjlab has no equivalent manifest.** `unitree_rl_mjlab`'s exporter
(`mjlab/rl/exporter_utils.py`) only embeds flat metadata in the `.onnx` (joint names, gains,
default pose, action scale, an `observation_names` list) — it does not capture per-term
scale/clip/history the way `deploy.yaml` does, and mjlab's H2 policy has a structurally
different contract anyway (98-dim single-frame obs, 29 actions, includes a `phase` term —
see prior analysis). If we ever want the same ROS2 node to run mjlab-trained checkpoints
too, we'd need either (a) an mjlab-side exporter that produces a `deploy.yaml`-equivalent
manifest, or (b) a separate adapter that reads the onnx metadata into the node's internal
contract representation. Not required for the H1/H2 Isaac Lab path — flag as future work
only if mjlab checkpoints need to be deployed this way.

## 3. What already exists today, per component

| Component | H1 | H2 |
|---|---|---|
| Bare URDF (no ros2_control/Gazebo wiring) | `unitree_ros/robots/h1_description` | `unitree_ros/robots/h2_description` (confirmed present, same bare shape) |
| ROS1 Gazebo package (`unitree_gazebo`) | Generic, quadruped-era, low-level-only; robot-agnostic — **not confirmed to even support humanoids**, and it's ROS1 (catkin) regardless | Same — no advantage either way |
| `ros2_control` + `gazebo_ros2_control` wiring | **Does not exist** — must be authored | **Does not exist** — must be authored |
| Real-hardware DDS message set | `unitree_hg` (`LowCmd`/`LowState`, `MotorCmd[35]`), used by H1 in `unitree_ros2` | Same `unitree_hg` message set, used by H2 in `unitree_sdk2`/`unitree_sdk2_python` C++/Python examples — **not exampled in `unitree_ros2` itself**, but mechanically identical (31 motors fit in the same 35-slot array) |
| C++ ONNX deploy reference (joint mapping, PD gains, obs assembly) | `unitree_rl_lab/deploy/robots/h1/main.cpp` + `unitree_rl_lab/deploy/robots/h1_2/main.cpp` — **working reference** | **Does not exist** — no `deploy/robots/h2` in `unitree_rl_lab` or `unitree_rl_mjlab` |
| Isaac Lab / mjlab training config (joint names, gains, default pose, obs/action spec) | Exists per-robot in both repos | `unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/h2/velocity_env_cfg.py`, `unitree_rl_mjlab/src/assets/robots/unitree_h2/h2_constants.py`, `unitree_rl_mjlab/src/tasks/velocity/config/h2` — all present |
| Auto-exported `deploy.yaml` manifest (joint_ids_map, gains, obs/action term spec) | Exists per H1 training run (e.g. `deploy/robots/g1_29dof/.../deploy.yaml` checked in as a worked example for G1) | **Already exists**: `unitree_rl_lab/logs/rsl_rl/unitree_h2_velocity/2026-09-10_22-04-06/params/deploy.yaml` |

**Key takeaway:** the Gazebo/`ros2_control` gap is identical for H1 and H2 — nobody has
built it for either. H1's advantage is narrower than "everything is set up": it has a
**working C++ deploy reference** (`deploy/robots/h1`) to copy the *pattern* from, and a
documented DDS path. H2 has the training-side config, `joint_sdk_names`, and the exported
`deploy.yaml` manifest all already in place — it's missing only the deploy reference
implementation and the Gazebo/`ros2_control` wiring, same as H1.

## 4. H1 path (reference case)

1. Add `<ros2_control>` + `gazebo_ros2_control` (or `gz_ros2_control`, depending on Gazebo
   version) tags to `h1_description`'s URDF/xacro. Declare position/velocity/effort
   interfaces per joint.
2. Write a `ros2_control` controller YAML (`joint_state_broadcaster` +
   effort/position controller) and a bring-up launch file that spawns Gazebo, the
   controller manager, and the controllers.
3. Write the policy node as a **manifest-driven composer**: port the small term registry
   (mirroring `deploy/include/isaaclab/envs/mdp/observations/observations.h` and the action
   equivalent) into the node's language, then have it parse H1's `deploy.yaml` to build the
   observation vector / joint mapping / PD gains — not a hardcoded H1-specific
   implementation. The only change from the existing C++ deploy pattern is the transport
   (ROS2 topics instead of the DDS/SDK channel objects).
4. Validate: joint order, action scale, control decimation, and default pose against the
   Isaac Lab H1 env config — this is a direct diff against a known-working reference, not
   guesswork.

Because the node is manifest-driven, this is the template for every other robot, H2
included — step 3 does not need to be redone, only re-pointed at a different `deploy.yaml`.

## 5. H2 gap: what's unknown, and how to resolve each unknown

| Unknown | How to resolve it | Template / source to copy from |
|---|---|---|
| Joint order used by the trained policy | **Already resolved** — captured in `deploy.yaml`'s term order and `joint_ids_map` | `unitree_rl_lab/logs/rsl_rl/unitree_h2_velocity/2026-09-10_22-04-06/params/deploy.yaml` |
| PD gains (kp/kd), effort limits, action scale | **Already resolved** — numeric values in `deploy.yaml`, sourced from the live training env | Same `deploy.yaml`; also `unitree_rl_mjlab/src/assets/robots/unitree_h2/h2_constants.py:39-100` for the mjlab-side equivalents |
| Default/home joint pose (obs zero-reference) | **Already resolved** — in `deploy.yaml`'s `default_joint_pos` | Same `deploy.yaml`; `h2_constants.py:107-119` (`HOME_KEYFRAME`) for mjlab |
| Mapping between training joint order and the real motor index order | **Already resolved** — `deploy.yaml`'s `joint_ids_map`, computed by name-matching against `UNITREE_H2_CFG.joint_sdk_names` | `unitree.py`'s `UNITREE_H2_CFG.joint_sdk_names` (sourced from `H2JointIndex` in `unitree_sdk2`'s `example/h2/low_level/h2_ankle_swing_example.cpp`) — **only exercised once a real/DDS backend is added, not for pure Gazebo sim** |
| Whether the H2 URDF's joint names match the MJCF/USD names used in training | Direct diff — both already exist, nothing to derive experimentally | `unitree_ros/robots/h2_description` (URDF) vs `h2_constants.py`'s `h2.xml` (MJCF) vs `unitree_model/H2/H2_dae.usd` |
| No working C++ deploy reference for H2 (unlike H1) | Write `deploy/robots/h2/main.cpp` following the H1/H1_2 pattern — now largely mechanical since `deploy.yaml` already supplies every per-robot value | `unitree_rl_lab/deploy/robots/h1/main.cpp`, `deploy/robots/h1_2/main.cpp` as templates |
| Whether the `mode_pr`/`mode_machine` low-level-control handshake behaves the same for H2 as H1 over DDS | Only relevant once a real-hardware/DDS backend is added later; the handshake logic is already demonstrated | `h2_ankle_swing_example.cpp`'s `LowCommandWriter()` (sets `mode_pr`/`mode_machine`, mirrored from `LowState_`) |
| Whether `unitree_ros2` needs any change to carry H2 traffic | Almost certainly none — `unitree_hg`'s `LowCmd`/`LowState` already has a 35-motor array and H2 uses 31; same topics (`rt/lowcmd`, `rt/lowstate`, `rt/secondary_imu`) as H1 | Verify by publishing/subscribing `unitree_hg` messages from a ROS2 node against a real or simulated H2 once available — this is the one item that's genuinely unverified rather than just "look it up" |

## 6. Milestones

1. Get the H1 Gazebo path working end-to-end (URDF wiring, controller config, launch).
2. Build the **manifest-driven policy node**: port the term registry + a `deploy.yaml`
   parser as generic, robot-agnostic code — this is the piece that pays off for every robot,
   not just H1. Validate it against H1's `deploy.yaml` first since there's a working C++
   reference to cross-check against.
3. Port `deploy/robots/h2` in `unitree_rl_lab`/`unitree_rl_mjlab` (C++, DDS/SDK-based) —
   independent of ROS2/Gazebo, useful on its own for real-hardware/mujoco-only deploy, and a
   good cross-check for the manifest-driven node's H2 output.
4. Repeat the Gazebo URDF/`ros2_control` wiring for H2 (`h2_description`), and point the
   same policy node from step 2 at H2's already-existing `deploy.yaml`.
5. (Follow-on, not required for sim2sim) Add a custom `ros2_control` hardware interface
   that speaks `unitree_hg` DDS, so the same policy node can also drive real H2 hardware —
   this is where the `H2JointIndex`/`joint_ids_map` mapping and `mode_pr`/`mode_machine`
   handshake actually get used.

## 8. ROS2 + Gazebo package structure (decided 2026-09-12)

Target stack: **ROS2 Jazzy + Gazebo Harmonic**, confirmed installed on this machine as
`gz sim` 8.15.0. The bridge plugin is `gz_ros2_control` (the Gazebo Sim / "gz" generation),
**not** `gazebo_ros2_control`, which targets EOL Gazebo Classic and isn't relevant here.

**Finding: `unitree_ros`'s description packages can't be reused directly.**
`unitree_ros/robots/h1_description/package.xml` is `format="2"` with
`<buildtool_depend>catkin</buildtool_depend>` and a catkin `CMakeLists.txt` — a ROS1 package,
not buildable in a ROS2/colcon workspace. `h2_description` isn't even a package (no
`package.xml`/`CMakeLists.txt` at all, just loose URDF + meshes). New ament_cmake
description packages must be authored from scratch; they can reference the existing mesh
files, but not the packages themselves.

**New repo: `unitree_ros2_gazebo`**, added as a submodule and symlinked whole into a sibling
`jazzy_ws/src/` (jazzy_ws lives next to this repo, not inside it). It holds three separate
colcon packages:

- **`unitree_gz_description`** — a generic `generate_urdf.py` that takes a bare URDF path and
  a `deploy.yaml` path, and programmatically injects the `<ros2_control>` tag (one
  `<joint>` block per motor, sourced from `deploy.yaml`'s joint list) plus the
  `gz_ros2_control` plugin block. Per-robot input is a small `robots/<name>.yaml` pointer
  file (`urdf_path`, `deploy_yaml_path`, `mesh_path`) — nothing else. This mirrors Isaac
  Lab's own pattern of pointing an asset config at a path rather than hand-authoring
  per-robot xacro/boilerplate; adding a new robot later means adding one pointer YAML, not a
  new package. Scope right now is H2 only — the mechanism is written generic, but no other
  robot's pointer file needs to exist yet.
- **`unitree_gz_bringup`** — launch files (`robot:=h2` arg): spawns Gazebo, spawns the robot
  entity, spawns `joint_state_broadcaster` + a raw effort passthrough controller.
- **`unitree_policy_bridge`** — the manifest-driven policy node from §2 (deploy.yaml parser +
  term registry + ONNX inference), robot-agnostic.

**Control interface decision: effort, with PD computed in `unitree_policy_bridge` itself**,
not Gazebo's own position-controller PID loop. Verified from the actual H1 deploy reference
(`unitree_rl_lab/deploy/robots/h1/src/State_RLBase.cpp:30`): each control step it writes only
`.q()` (target position) to the motor command — the policy's action space is position
targets, and the PD-to-torque conversion happens downstream (onboard motor firmware on real
hardware; Isaac Lab's implicit-PD actuator model in training). Reproducing that PD math
ourselves in `unitree_policy_bridge`, using `deploy.yaml`'s `kp`/`kd` directly, matches both
of those exactly and is bit-for-bit checkable against the H1 C++ reference (per the §7
checklist's last item) — relying on Gazebo's own internal PID loop would not have that
guarantee, since its timing/discretization isn't tied to Isaac Lab's actuator model. Use
`forward_command_controller`/`effort_controllers` (raw torque passthrough) as the
`ros2_control` controller, not `joint_trajectory_controller` or anything PID-based.

**Dependency audit (2026-09-12): no new apt packages needed.** Already installed on this
machine: `ros-jazzy-ros-gz-sim`/`ros-jazzy-ros-gz-bridge`, `ros-jazzy-gz-ros2-control`
(1.2.20), `ros-jazzy-ros2-control`/`ros-jazzy-ros2-controllers`/`ros-jazzy-controller-manager`,
`ros-jazzy-effort-controllers`/`ros-jazzy-forward-command-controller`,
`ros-jazzy-joint-state-broadcaster`. ONNX Runtime 1.22.0 is already vendored at
`unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0/` — `unitree_policy_bridge`
should link against that copy rather than fetching its own.

## 9. Validation checklist (applies to both H1 and H2)

- [ ] Joint name/order parity between training config and URDF/`ros2_control` joint list
- [ ] Same PD gains (or equivalent effort-mode math) as training
- [ ] Same action scale/clipping as training
- [ ] Same control decimation (policy step rate vs. physics step rate) as training
- [ ] Same default/home pose used as the observation's zero-reference
- [ ] Same gravity/angular-velocity frame convention (base frame vs. world frame) in the
      observation construction
- [ ] The manifest-driven node's computed observation vector matches the C++ deploy
      reference's output bit-for-bit given the same simulated state (a strong, cheap
      correctness check before ever running the policy for real)
