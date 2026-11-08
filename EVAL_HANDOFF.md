# Gazebo evaluation harness — handoff (2026-11-05)

Read `~/humanoidx/PROJECT.md` first (project map). This file covers only the Gazebo evaluation
work: a fixed test suite that scores any H2 policy from simulator ground truth, so reward and
observation experiments can be compared.

## User requirements (all agreed with the user; don't relitigate)

- **One suite for every policy.**
  - The commands published on `/cmd_vel` are fixed in the suite file and never depend on the
    policy.
  - Grading uses the published command, never `deploy.yaml`. An old policy is graded the same
    way and simply scores badly.
- **The eval is independent of the existing stack.**
  - Nothing in the policy or sim stack may depend on `unitree_gz_eval`.
  - Eval-only additions to existing packages are opt-in and off by default (`instrumentation`).
- **The policy code must keep running on the real H2.** No ros2_control rewrite: no H2
  ros2_control hardware interface exists, checked 2026-11-05.
- **Ground truth comes from the simulator** (link poses, contact forces), never from sensors.
- **Tests:** several short tests with a reset between each. ~0.5 s lead, ≤1 s pause between
  direction changes, ~4 s per direction.
- **Analysis is plain Python + wandb in its own uv venv,** not coupled to ROS.

## How to run

```bash
source /opt/ros/jazzy/setup.bash && source ~/humanoidx/jazzy_ws/install/setup.bash
ros2 launch unitree_gz_eval eval.launch.py \
  policy_onnx_path:=$HOME/Downloads/onnx_from_misc/09-17-v2/exported/policy.onnx \
  urdf_path:=$HOME/humanoidx/unitree-humanoidx/unitree_ros/robots/h2_description/H2.urdf
# optional: suite:=<name or abs path>  output_dir:=...  headless:=false
# output: <policy_dir>/eval/<suite>_<timestamp>/  (meta.json, suite.yaml, NN_<test>_r<k>.npz)

cd ~/humanoidx/unitree-humanoidx/unitree_ros2_gazebo/unitree_gz_eval/analysis
env -u PYTHONPATH uv run analyze.py <eval_dir> [--no-wandb]   # -> <eval_dir>/results.json
env -u PYTHONPATH uv run --group dev pytest -q test_metrics.py
```

- `env -u PYTHONPATH` is required: the shell profile sources ROS, whose `PYTHONPATH` breaks
  the uv venv (pytest tries to load `launch_testing` and fails with "no module lark").
- The launch shuts itself down when the runner exits. Exit code 0 is not proof of success:
  read `meta.json` (`finished`, each test's `status`).
- **wandb: never tested.** No CLI login exists on this laptop. The user has to run
  `! cd …/analysis && env -u PYTHONPATH uv run wandb login` once. The key exists only in the
  MCP config in `~/.claude.json`; don't copy it out.
  - Target: entity `kieranparanjpe-mcgill-university`, project `humanoidx-gz-eval`.
  - Run name `<policy>__<suite>`, grouped by suite hash.

## Files

**New package `unitree_ros2_gazebo/unitree_gz_eval/`** (ament_python; it depends on the
existing packages, nothing depends on it):

- `suites/default.yaml`: the 12 tests (156 s of sim time per repeat), `repeats: 3`, fall limits
  (pelvis < 0.5 m or tilt > 70°), reset-check limits. Any edit changes the suite hash.
- `unitree_gz_eval/suite.py`: loads and validates the suite (rejects unknown keys); builds each
  test's absolute schedule (segments + push times); the hash is the sha256 of the file.
- `unitree_gz_eval/runner_node.py` (~360 lines). For each repeat × test:
  1. Calls `/gz_reset_node/reset`.
  2. Checks the reset using only post-reset poses: the first one must be at the origin and
     upright, and the pelvis must not move much over the next 0.1 s. Aborts the test if not.
  3. Publishes the scheduled `/cmd_vel` at 50 Hz on sim time, fires pushes, ends the test early
     on a fall.
  4. Saves `.npz` and rewrites `meta.json`.

  Callbacks only store messages; conversion to arrays happens once per test in `_save`. It keeps
  only the model (= pelvis) pose, the torso and feet poses relative to the model, joint
  velocity/effort decimated to 200 Hz, and foot vertical force.
- `unitree_gz_eval/gz_push.py`: a push is a one-physics-step `ExternalWorldWrenchCmd` on the
  pelvis link with force = mass·dv / step (an impulse). It's deliberately separate from
  bringup's `gz_ecm.py`.
- `launch/eval.launch.py`: includes `unitree_isaac_policy all.launch.py` with
  `instrumentation:=true`, starts a `ros_gz_bridge` for poses and contacts, then the runner.
  There's no rosbag: it was 1.1 GB per run, mostly ros2_control diagnostics, and the user chose
  to drop it.
- `analysis/` (not installed; uv project, `package = false`):
  - `metrics.py`: all metrics. The previous session explained each one to the user in full;
    re-derive from the code if needed.
  - `analyze.py`: per-repeat metrics → mean / std / standard error per test; the overall score
    is computed per repeat and then averaged; time series averaged over repeats for wandb.
  - `test_metrics.py`: 6 synthetic tests.

**Changes to existing code** (`unitree_ros2_gazebo` is *not tracked by git*; there is no diff
history, so be careful):

- **`unitree_gz_description/.../generate_urdf.py`**
  - `build_urdf(..., instrumentation=False)`. When on, it adds a PosePublisher plugin (200 Hz,
    `/model/<robot>/pose`, Pose_V) and contact sensors on `foot_links`
    (`/contact/<link>`). Output is byte-identical when off (verified).
  - `IMU_UPDATE_RATE_HZ` is now 1000 (was 500), so each joint sample has an IMU sample with the
    same stamp.
- **`unitree_gz_description/robots/h2.yaml`**: added `foot_links` and `torso_link` (eval only).
- **`unitree_gz_bringup/launch/sim.launch.py`**: new `instrumentation` (default false) and
  `headless` (default false, gives `gz sim -s`) args. `launch_utils.py` forwards both.
- **`unitree_gz_bringup/unitree_gz_bringup/gz_ecm.py` + `gz_reset_node.py`: reset bug fixed**
  (details below). `HANDOFF.md`'s ECM section was corrected.
- **`unitree_isaac_policy/src/policy_node.hpp`**: new parameter `step_on_state_stamp` (default
  false).
  - When true, the timer isn't created. `maybe_step()`, called from both the joint and the IMU
    callback, runs `on_step()` once both sensors' stamps reach the next 20 ms boundary.
  - The first boundary is set once both sensors have data after a reset, so steps are aligned
    to the reset.
  - `policy.launch.py` sets it to true (sim only).
- **`unitree_isaac_policy/src/pd_node.cpp`** (2026-11-06): on `/policy_reset` it switches its
  target to `default_joint_pos` and publishes a fresh effort at once. Before, it kept the
  pre-reset target.
- **`unitree_gz_bringup/.../gz_reset_node.py`** (2026-11-06): publishes `/policy_reset` inside
  `_reset()`, after the teleport steps and before un-pausing, and waits up to 1 s for it to be
  acknowledged. Before, it published after un-pausing.
- **`unitree_rl_lab/deploy/.../observations.h`**: the `gait_phase` one-step-lead fix from the
  earlier session. Uncommitted in the `unitree_rl_lab` submodule; every eval run used it.
- **`PROJECT.md` §6.9**: action item to train with latency/async randomisation (with
  literature).

Nothing has been committed. Commit only when the user asks.

## Findings that cost time (verified; don't re-discover)

- **Reset never removed momentum.** `gz_ecm.write_base_velocity_zero` wrote
  `WorldLinear/AngularVelocityCmd`, and gz-sim 8's Physics never reads those.
  - The fix uses model `LinearVelocityCmd`/`AngularVelocityCmd`. Physics zeroes them but leaves
    them in place, which holds the base at zero velocity, so they're removed in the
    joint-reset request.
  - `EntityComponentManager::SetState` skips an **empty-payload** component *before* checking
    `remove`, so the removal needs a dummy payload.
  - Verified: a reset while walking or mid-fall now leaves the robot at ~0 velocity.
- **Pushes:** `ExternalWorldWrenchCmd` (one step, then cleared) works. Velocity commands can't
  be used as one-shots (see above). The measured whole-body velocity change is ≈ dv (±10%).
- **The pelvis frame is the model frame** (identity over 20k samples), so the model pose is the
  pelvis pose.
- **Names:** `ros_gz_bridge` rewrites gz `::` to `/` in frame ids (`h2/left_ankle_pitch_link`).
- **Contact sensors** publish at 1 kHz regardless of `update_rate`. Contact wrenches carry
  force, and the summed foot Fz while standing ≈ m·g.
- **The Python runner drops messages:** it received ~580/1000 joint, ~500/1000 contact and
  183/200 pose messages per second. The metrics use timestamps, so this is tolerated.
- **Determinism investigation:**
  - No RNG or noise anywhere in the stack.
  - Gazebo alone is deterministic: post-reset trajectories match to 1e-14.
  - With the policy, runs diverge (mm at 10 ms, cm at 1 s) because the node loop is
    asynchronous with Gazebo.
  - `step_on_state_stamp` fixed the policy-step phase: the first output is now 2–7 ms after
    every reset, versus 3–22 before.
  - Runs still diverge: torque application timing (`pd_node` → Gazebo) remains asynchronous.
  - The user's decision: **accept the asynchrony, run repeats and aggregate**, and add latency
    randomisation to training (`PROJECT.md` action item). Lockstep and ros2_control were
    rejected.
- **`stand` aborted by the reset check (fixed 2026-11-06).** Measured over 22 resets:
  - The teleport is accurate. The first pose after a reset is within 1 mm of the origin, with
    yaw within ±0.1° (worst case 1°).
  - The check used to judge the pose 0.1 s later, with the policy running. The reset puts the
    pelvis at 1.030 m, but H2 stands at 1.019 m in Gazebo, so it drops 11 mm, lands at ~45 ms
    and settles by ~90 ms. The policy turns it 0.4–2.7° while doing so. The ±4° failures were the
    tail of this.
  - Metrics use heading relative to each test's start, so this offset doesn't bias them.
  - Fix: the pose is judged on the first post-reset sample, and speed over the next 0.1 s. Only
    the runner changed; the suite and its hash did not. The yaw at the check is now within ±0.3°.
  - Also found and fixed: the stale `pd_node` target after a fall (`HANDOFF.md` open item 5).

## Open issues, in priority order

1. **The wrist roll joints oscillate continuously in Gazebo** (measured 2026-11-06 while
   standing quietly).
   - They oscillate at ~200 Hz, saturated at the URDF's 18.7 rad/s velocity limit (15.5 rad/s
     rms, 31 N·m rms). Shoulder yaw and wrist yaw shake along with them. Left ankle roll is at
     5 rad/s rms, with peaks of 32 rad/s.
   - Likely cause: the explicit 1 kHz PD (kp 40, kd 2) is unstable on these light links. Isaac's
     PD is implicit, and its joints carry 0.01 kg·m² of armature.
   - The policy sees wrist velocity × 0.05 = ±0.93, where training noise is ±0.075. This is a
     candidate cause of the wide, off-balance stance that only shows up in Gazebo.
   - Not investigated further.
2. **The held effort is applied during the teleport step itself.**
   - The effort controller keeps applying the last command during the paused step that
     teleports the joints, and that command was computed for the pre-reset state.
   - Light joints therefore end the teleport moving: wrist roll at 18.7 rad/s (partly the
     oscillation above), and ankle roll at ~20 rad/s after falls.
   - Fixing it needs zero effort during that step. That means telling `pd_node` before the
     teleport, separately from the policy, which must re-seed after it.
3. **gz world-control steps don't run when they are requested.**
   - In the 2026-11-06 bags, the world stays frozen through the first `_step_once` sleep, and the
     step only runs when the next request arrives. So the base and joint writes land in the same
     step, contrary to `_reset()`'s comment.
   - The teleport result is still correct (measured). Not investigated.
4. **The real-time factor dropped to ~0.55–0.72** (14 min for 3 repeats) from ~0.8. Suspects
   are the IMU at 1 kHz and `step_on_state_stamp`. Neither is measured.
5. **`analyze.py` `print_summary` shows `ok` for tests that were aborted**: it only counts
   falls.
6. **`joint_acc_rms` / `joint_jerk_rms` depend on the input sample rate.** They went from 4.4e3
   to 1.9e4 when joints were decimated to 200 Hz. Redefine them so they don't depend on the
   input rate.
7. **`stride_length` is wrong:** it pairs consecutive touchdowns of the same foot even across
   different steady windows (e.g. across a stop), which inflates it.
8. **Overall means** of metrics that exist only in some tests (`ratio_wz`, `stand_*`) are means
   over whichever tests have them. Interpret per test.
9. **`gz_reset_node` prints a harmless `RCLError` traceback** on launch shutdown.
10. **`PROJECT.md` has no "Gazebo evaluation" section yet.** Add one pointing here once the
   above settles.
11. **Older eval folders** under `~/Downloads/onnx_from_misc/09-17-v2/eval/` (`…201446`,
   `…210028`, `…213022`) use earlier `.npz` layouts that the current analysis can't read. The
   user may want them deleted. Ask first.

**Baseline, 09-17-v2** (latest run, mean ± standard error over 3 repeats; stand excluded):
- steady tracking error: forward 0.069 ± 0.004 m/s, sideways 0.060 ± 0.001 m/s, turn
  0.077 ± 0.001 rad/s
- response time 0.61 ± 0.01 s
- heading drift 6.0 ± 0.4 °/m
- stance width 0.437 ± 0.001 m
- foot slip 1.3 ± 0.1 cm/step
- cost of transport 5.4 ± 0.2
- push test fell in 2 of 3 repeats

## Method notes

- **Kill stacks by process group, never with broad `pkill`** (auto mode denies it, and it can
  match your own shell). Pattern used:

  ```bash
  setsid bash -c "echo \$\$ > $S/pgid; exec ros2 launch …" > log 2>&1 &
  …; kill -INT -- -$(cat $S/pgid); sleep 5; kill -9 -- -$(cat $S/pgid)
  ```

  Then check `ps -eo cmd | grep -cE "[g]z sim|[p]olicy_node"` is 0.
- **An interrupted or rejected tool call can still have launched a stack.** Afterwards, check
  for stray `gz sim` processes. Two worlds publishing on the same topics silently corrupted one
  investigation.
- **Long runs:** use Bash `run_in_background` with a script that waits for
  `eval_runner-N]: process has` and then kills the group.
- **Exact timing work:** pose samples come every 5 ms. To align on reset steps, temporarily set
  `GROUND_TRUTH_RATE_HZ = 1000` in `generate_urdf.py`, and **revert it** afterwards (it's
  currently 200).
- **Probe scripts** from this session are in the session scratchpad
  (`/tmp/claude-1000/-home-kieran-humanoidx/1e7b880d-…/scratchpad/`: `det_probe.py`,
  `det_compare.py`, `e_run.sh`, `run_full.sh`). They're temporary and may be gone.
- **Reset probes (2026-11-06)**, same caveat, in
  `/tmp/claude-1000/-home-kieran-humanoidx/27bf44ca-f984-4be4-8293-2a1c1c66144a/scratchpad/`.
  - `run_probe2.sh` runs the stack and `ros2 bag record` with `probe2.py` (standing and
    after-fall resets). `run_eval_probe.sh` runs the real runner on a stand + pushes suite.
  - `reset_sig.py` gives the per-reset torque and yaw, `fine_timeline.py` the per-message
    timeline, and `steady_joints.py` the joint oscillation table.
  - The `ros2 bag record` process sometimes ignores the first SIGINT; check that it exited.

## Working with this user

- **Answer questions; don't start implementing when asked a question.** Explain, propose, and
  wait for an explicit go-ahead.
- **Plain language with concrete quantities.** No jargon, no vagueness. Keep responses short
  when asked.
- **Keep code minimal.** No speculative features, no workarounds for bugs: fix the root cause,
  or stop and report it.
- **Investigate and measure; don't speculate.** Say clearly what is verified and what isn't.
