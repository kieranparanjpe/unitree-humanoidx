# H2 Isaac Lab training — session notes (2026-09-11)

## Playing a trained policy with specific weights
`unitree_rl_lab.sh -p --task <task> --checkpoint <path/to/model_N.pt>` or `--load_run <run_folder>`
picks a specific checkpoint; omitting both auto-loads the latest run's latest checkpoint.
(`play.py:88-91`, `cli_args.py:30-32`)

## Bugs found & fixed in unitree_rl_lab/scripts/rsl_rl/play.py

1. **Segfault on `-p` without `--headless`**
   `[xcb] Unknown sequence number...` / `You called XInitThreads...` / segfault in `AppLauncher.__init__`.
   - Root cause: Kit doesn't vendor its own libX11/libxcb — it dynamically links the host's.
     This machine's Ubuntu 20.04 ships libxcb 1.14 / libX11 1.6.9 (~2020), while the NVIDIA
     driver is 575.57.08 (2025-era). Attaching Kit's own GLX/Vulkan context to the live,
     already-compositing desktop X session (GNOME/Mutter on Xorg :1, vt2) races two threads
     on that old libxcb and it aborts. Ubuntu 20.04 is also already outside Isaac Sim 4.5's
     supported OS list (22.04/24.04).
   - Fix: pass `--headless` (unsets DISPLAY, drops to EGL offscreen, never touches libX11/xcb).
     Add `--video` to still see the rollout as an mp4 instead of a live window.
   - Not resolved: an actual live GUI viewer would need a dedicated, non-shared X server, or
     upgrading the host to 22.04.

2. **`ModuleNotFoundError: isaaclab.utils.pretrained_checkpoint`**
   `play.py:59` imported from the old location. Installed IsaacLab (v2.3.2) moved this function
   to `isaaclab_rl.utils.pretrained_checkpoint`. Fixed the import. This is a top-level import,
   so it broke every `-p` invocation regardless of flags.

3. **`--video` overwrote the previous recording every run**
   `RecordVideo`'s `name_prefix` defaulted to the hardcoded `"rl-video"`. Fixed by setting
   `name_prefix=f"rl-video-{time.strftime('%Y%m%d-%H%M%S')}"` in `play.py`'s `video_kwargs`.

## H2 asset verified
`unitree_model/H2/H2_dae.usd` (+ `configuration/*.usd` layers) is wired correctly: 31 actuated
joints + 2 fixed hand joints, `ArticulationRootAPI` at `/H2/pelvis`, all names match
`UNITREE_H2_CFG` in `unitree.py`. Confirmed by opening the stage with `pxr` (needed Isaac
Sim's bundled `kit/python/bin/python3` + `omni.usd.libs` extension on
`PYTHONPATH`/`LD_LIBRARY_PATH` — plain `python3` and the conda env don't have `pxr`).
Raw grep on the binary usd gives false negatives (compressed sections) — don't trust that method.

Two pre-flagged (in code comments), still-unverified tuning guesses, not wiring bugs:
- `base_height` reward target = `1.0` (`h2/velocity_env_cfg.py:304`)
- head joint PD stiffness/damping (`unitree.py:833-840`)

## Why mjlab's H2 policy trains faster / looks better than Isaac's
- **Not** more domain randomization in Isaac — mjlab's is if anything richer (6-axis push vs
  Isaac's x/y-only; plus encoder-bias + CoM-jitter randomization Isaac's H2 lacks). Isaac H2's
  `base_external_force_torque` event is currently a no-op (`force_range=(0,0)`).
- **Not** harder terrain — Isaac's `COBBLESTONE_ROAD_CFG` only defines one sub-terrain type
  (`"flat"`), so despite the name/curriculum it's flat ground.
- Real drivers: mjlab gives the policy an explicit gait-`phase` observation (Isaac's G1/H2 have
  this commented out); mjlab's reward set is richer (`variable_posture` per-joint-mode pose
  reward, `stand_still`, `self_collision_cost`); MuJoCo has materially lower per-step overhead
  than PhysX/Isaac Sim generally.
- PPO hyperparameters are identical between the two — not the source of the gap.

## H2 (Isaac) was copied from G1, not H1
`h2/velocity_env_cfg.py` is a near line-for-line copy of `g1/29dof/velocity_env_cfg.py` — same
scene/event/observation boilerplate, same commented-out `gait_phase`/`height_scanner` lines,
same 19-term reward set (only body-name selectors and `base_height` target differ).
`h1/velocity_env_cfg.py` is meaningfully more developed: **active** `gait_phase` observation,
extra `feet_contact_forces` reward + `base_contact` termination, different weights
(`base_angular_velocity=-0.5`, `flat_orientation_l2=-1.0`, `feet_clearance=20.0`).
In mjlab, by contrast, G1/H1_2/H2 all share one factory (`make_velocity_env_cfg`) — no such
divergence exists there.

## Plan to improve H2's Isaac policy (not started yet)
1. **Cheap, same-framework wins first** — port H1(isaac)'s choices into H2: enable the
   `gait_phase` observation, adopt H1's `base_angular_velocity` / `flat_orientation_l2` /
   `feet_clearance` weights, add `feet_contact_forces` reward + `base_contact` termination.
2. **Then, higher-effort mjlab ports**, only if still needed: `variable_posture` per-joint pose
   reward, `self_collision_cost`, `stand_still`, `soft_landing`, `encoder_bias`/`body_com_offset`
   randomization — none of these have an IsaacLab `mdp` equivalent yet; each needs
   reimplementing against IsaacLab's sensor/asset/event API.
