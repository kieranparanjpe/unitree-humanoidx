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
- The **policy inference node** is new code regardless of robot: subscribe to
  `/joint_states` + IMU + a velocity-command topic, build the observation vector in the
  exact order/scale the Isaac Lab env used, run the ONNX policy at the trained control
  rate, publish joint targets (doing the PD math yourself if using an effort controller,
  same as the existing hardware deploy code does).

Note: this Gazebo path does **not** need `unitree_ros2`/DDS/`unitree_hg` at all — those are
only relevant if we later want the *same* policy node to also drive real hardware, via a
custom `ros2_control` hardware interface that translates to DDS. That's a follow-on, not a
prerequisite for sim2sim.

## 2. What already exists today, per component

| Component | H1 | H2 |
|---|---|---|
| Bare URDF (no ros2_control/Gazebo wiring) | `unitree_ros/robots/h1_description` | `unitree_ros/robots/h2_description` (confirmed present, same bare shape) |
| ROS1 Gazebo package (`unitree_gazebo`) | Generic, quadruped-era, low-level-only; robot-agnostic — **not confirmed to even support humanoids**, and it's ROS1 (catkin) regardless | Same — no advantage either way |
| `ros2_control` + `gazebo_ros2_control` wiring | **Does not exist** — must be authored | **Does not exist** — must be authored |
| Real-hardware DDS message set | `unitree_hg` (`LowCmd`/`LowState`, `MotorCmd[35]`), used by H1 in `unitree_ros2` | Same `unitree_hg` message set, used by H2 in `unitree_sdk2`/`unitree_sdk2_python` C++/Python examples — **not exampled in `unitree_ros2` itself**, but mechanically identical (31 motors fit in the same 35-slot array) |
| C++ ONNX deploy reference (joint mapping, PD gains, obs assembly) | `unitree_rl_lab/deploy/robots/h1/main.cpp` + `unitree_rl_lab/deploy/robots/h1_2/main.cpp` — **working reference** | **Does not exist** — no `deploy/robots/h2` in `unitree_rl_lab` or `unitree_rl_mjlab` |
| Isaac Lab / mjlab training config (joint names, gains, default pose, obs/action spec) | Exists per-robot in both repos | `unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/h2/velocity_env_cfg.py`, `unitree_rl_mjlab/src/assets/robots/unitree_h2/h2_constants.py`, `unitree_rl_mjlab/src/tasks/velocity/config/h2` — all present |

**Key takeaway:** the Gazebo/`ros2_control` gap is identical for H1 and H2 — nobody has
built it for either. H1's advantage is narrower than "everything is set up": it has a
**working C++ deploy reference** (`deploy/robots/h1`) to copy the obs/action/gain logic
from, and a documented DDS path. H2 has the training-side config fully specified, but no
deploy reference yet.

## 3. H1 path (reference case)

1. Add `<ros2_control>` + `gazebo_ros2_control` (or `gz_ros2_control`, depending on Gazebo
   version) tags to `h1_description`'s URDF/xacro. Declare position/velocity/effort
   interfaces per joint.
2. Write a `ros2_control` controller YAML (`joint_state_broadcaster` +
   effort/position controller) and a bring-up launch file that spawns Gazebo, the
   controller manager, and the controllers.
3. Write the policy node. Port the obs-construction, action-scaling, and PD-gain logic
   directly from `unitree_rl_lab/deploy/include/FSM/State_RLBase.h` and
   `deploy/robots/h1/main.cpp` — joint order, default pose, and gains are already solved
   there; the only change is the transport (ROS2 topics instead of the DDS/SDK channel
   objects).
4. Validate: joint order, action scale, control decimation, and default pose against the
   Isaac Lab H1 env config — this is a direct diff against a known-working reference, not
   guesswork.

This is the template for every other robot, H2 included.

## 4. H2 gap: what's unknown, and how to resolve each unknown

| Unknown | How to resolve it | Template / source to copy from |
|---|---|---|
| Joint order used by the trained policy | Read directly from the training config — no ambiguity | `unitree_rl_mjlab/src/assets/robots/unitree_h2/h2_constants.py` (actuator groups, regex joint names), `unitree_rl_lab/.../robots/h2/velocity_env_cfg.py` |
| PD gains (kp/kd), effort limits, action scale | Already numeric constants in the mjlab config; carries over 1:1 into the policy node's PD math | `h2_constants.py:39-100` (per-group stiffness/damping/effort_limit), `h2_constants.py:187-195` (action scale formula) |
| Default/home joint pose (obs zero-reference) | Already defined | `h2_constants.py:107-119` (`HOME_KEYFRAME`) |
| Mapping between training joint order and the real motor index order | Build a static name→index permutation array once, by matching joint names between the two known lists | Training order: files above. Motor order: `H2JointIndex` enum in `unitree_sdk2`'s `example/h2/low_level/h2_ankle_swing_example.cpp` (31 motors, 0–30) — **only needed once we wire a real/DDS backend, not for pure Gazebo sim** |
| Whether the H2 URDF's joint names match the MJCF/USD names used in training | Direct diff — both already exist, nothing to derive experimentally | `unitree_ros/robots/h2_description` (URDF) vs `h2_constants.py`'s `h2.xml` (MJCF) vs `unitree_model/H2/H2_dae.usd` |
| No working C++ deploy reference for H2 (unlike H1) | Write `deploy/robots/h2/main.cpp` following the H1/H1_2 pattern, populated with H2's gains/joint list instead of H1's — mechanical port, not new design | `unitree_rl_lab/deploy/robots/h1/main.cpp`, `deploy/robots/h1_2/main.cpp` as templates |
| Whether the `mode_pr`/`mode_machine` low-level-control handshake behaves the same for H2 as H1 over DDS | Only relevant once a real-hardware/DDS backend is added later; the handshake logic is already demonstrated | `h2_ankle_swing_example.cpp`'s `LowCommandWriter()` (sets `mode_pr`/`mode_machine`, mirrored from `LowState_`) |
| Whether `unitree_ros2` needs any change to carry H2 traffic | Almost certainly none — `unitree_hg`'s `LowCmd`/`LowState` already has a 35-motor array and H2 uses 31; same topics (`rt/lowcmd`, `rt/lowstate`, `rt/secondary_imu`) as H1 | Verify by publishing/subscribing `unitree_hg` messages from a ROS2 node against a real or simulated H2 once available — this is the one item that's genuinely unverified rather than just "look it up" |

## 5. Milestones

1. Get the H1 Gazebo path working end-to-end (URDF wiring, controller config, policy
   node, launch) — this de-risks the generic (non-robot-specific) parts of the pipeline.
2. Port `deploy/robots/h2` in `unitree_rl_lab`/`unitree_rl_mjlab` (C++, DDS/SDK-based) —
   independent of ROS2/Gazebo, but produces the validated joint-mapping/gain logic to
   reuse in step 3, and is useful on its own for real-hardware or mujoco-only deploy.
3. Repeat the H1 Gazebo steps for H2, substituting `h2_description`'s URDF and the
   config/gains from `h2_constants.py` / `velocity_env_cfg.py`.
4. (Follow-on, not required for sim2sim) Add a custom `ros2_control` hardware interface
   that speaks `unitree_hg` DDS, so the same policy node built in step 3 can also drive
   real H2 hardware — this is where the `H2JointIndex` mapping and `mode_pr`/`mode_machine`
   handshake actually get used.

## 6. Validation checklist (applies to both H1 and H2)

- [ ] Joint name/order parity between training config and URDF/`ros2_control` joint list
- [ ] Same PD gains (or equivalent effort-mode math) as training
- [ ] Same action scale/clipping as training
- [ ] Same control decimation (policy step rate vs. physics step rate) as training
- [ ] Same default/home pose used as the observation's zero-reference
- [ ] Same gravity/angular-velocity frame convention (base frame vs. world frame) in the
      observation construction
