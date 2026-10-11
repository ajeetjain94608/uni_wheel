# Uni Wheel — RL Self-Balancing Reaction-Wheel Unicycle

A single-ground-wheel balancing robot: one motor drives the ground-contact wheel
(pitch balance, same principle as a classic 2-wheel balance bot), and a second
motor spins a reaction/flywheel (roll balance — the axis a 2-wheel bot doesn't
need to worry about, but a unicycle does).

Project structure mirrors [ShawnHymel/reinforcement-learning-for-robotics](https://github.com/ShawnHymel/reinforcement-learning-for-robotics):
train the balance policy in MuJoCo with PPO, export it, run it on the real
robot's ESP32. Mechanical design lives in OnShape (not FreeCAD, as in the
reference repo) and is pulled into the sim pipeline with
[`onshape-to-robot`](https://onshape-to-robot.readthedocs.io/).

## Hardware

- Compute: ESP32-WROOM
- IMU: MPU6050
- Motors (×2, one per axis): DFRobot FIT1034 — 2804 hollow-shaft BLDC gimbal
  motor with a built-in AS5600 magnetic encoder
- Motor drivers (×2): SimpleFOC Mini
- Mechanical design: OnShape, [`Uni Wheel V2`](https://cad.onshape.com/documents/8d580a9c25331d8779ad11df/w/cedc02ad8d136dd6be906e0c/e/798769d6907ec11080a60c0a)

See `docs/feasibility_analysis.md` for a first-pass physical sanity check (torque
margins, reaction-wheel momentum saturation) computed from the real OnShape mass
properties before any sim work started.

## Project layout

```
workspace/
  mechanical/
    onshape/uniwheel/       # onshape-to-robot config + generated MJCF + meshes
  software/
    01-test-model-in-mujoco/    # sanity-check the generated model, hand-tuned PID
    02-balance-bot-with-ppo/    # gym env (2 actuators: wheel torque, flywheel torque) + PPO training
    03-sim-to-real/             # ESP32 firmware: IMU+encoder read, FOC command, trained policy inference
    04-domain-randomization/    # randomize mass/friction/motor params during training for robustness
    05-command-conditioning/    # steer/drive-command-conditioned policy
    06-remote-control/          # ESP32 web UI for manual RC
    rl/                         # shared PPO trainer + policy-to-C exporter
docs/
  feasibility_analysis.md
Dockerfile.bare / Dockerfile.cpu   # (to add) same containerized MuJoCo/SB3 dev env as the reference repo
```

## Status

- [x] Pulled real mass/inertia numbers from OnShape API, ran a first feasibility
      pass (`docs/feasibility_analysis.md`)
- [x] CAD → MJCF: `onshape-to-robot` hit a hard limitation (see
      `workspace/mechanical/onshape/uniwheel/EXPORT_NOTES.md`); switched to
      OnShape's native URDF export → MuJoCo (`fusestatic`) → hand-finished MJCF.
      Working model: `workspace/mechanical/urdf/uniwheel/uniwheel.xml`
      (0.394 kg total, verified against the OnShape mass-properties API; see
      `CONVERSION_NOTES.md` next to it). Loads, settles under gravity, and
      both actuators drive/react as expected.
- [x] Stage 01: model balances under a hand-tuned PID (`workspace/software/01-test-model-in-mujoco/`).
      First pass wrongly reported an 18mm CG offset — that was a bug in how
      MuJoCo's `fusestatic` merges bodies, not a real design issue (CG is
      actually centered to <2mm, confirmed against OnShape's own numbers).
      See `STAGE01_FINDINGS.md` and `workspace/mechanical/urdf/uniwheel/
      CONVERSION_NOTES.md` for the corrected story. Real finding that survived:
      pitch and roll each balance well alone, but not reliably together
      (genuine gyroscopic coupling) — good motivation for stage 02.
- [x] Stage 02 wired up and smoke-tested (`workspace/software/02-balance-bot-with-ppo/`,
      `workspace/software/rl/`): `UniWheelEnv` gym env (2 actuators, both wheel speeds
      in the observation — see that folder's README for why), reference repo's PPO
      trainer copied in unmodified. A 5-iteration smoke test confirmed the whole
      env/trainer/reward pipeline end-to-end (mean return climbed 52→69, checkpointing
      and best-model saving both worked) before committing to a real training run.
- [x] Real training run in progress in the background (38% through at last
      check, mean return climbing steadily: 430 → 6,187 → 9,936). Evaluated an
      early checkpoint against stage 01's harder disturbance scenarios — solid
      within its training distribution (survives velocity kicks 3x beyond what
      it was trained on), doesn't generalize to starting pre-tilted (untested
      scenario, not in `reset()` — confirmed the reference repo has the same
      gap in every stage, and decided not to fix it: the real robot never
      starts pre-tilted in practice).
- [x] Stage 03 firmware scaffolded (`workspace/software/03-sim-to-real/`):
      sensor_test → static_inference_test → uniwheel_bot, built on
      [macolin007/mcdgf](https://github.com/macolin007/mcdgf) (identical
      hardware stack, real pin scheme + SimpleFOC setup) crossed with the
      reference RL repo's control-loop shape. `actor.h` still needs exporting
      once training finishes; IMU axis mapping and motor directions are
      flagged placeholders needing empirical calibration on real hardware.
- [x] Stage 04 prepared (`workspace/software/04-domain-randomization/`):
      `uniwheel_env_dr.py` (obs/motor noise, action delay, pushes, mass/friction/
      motor-gain randomization, wheel-only ridge torque — deliberately not
      applied to the reaction wheel, it never touches the ground) and a phased
      training notebook that **warm-starts from stage 02's checkpoint**
      (verified this actually works, not just assumed) rather than redoing
      stage 02's work like the reference repo's own DR notebook does. Waiting
      on stage 02 to finish before running it.
- [x] Stage 02 training complete. Best mean return **9,970.5** (430 → 6,187 → 9,093 →
      9,783 → 9,936 → 9,970 over the run). Independently verified with a full
      10,000-step rollout (bypassing a bug found in the shared `evaluate()` helper —
      `RecordEpisodeStatistics` doesn't forward custom attributes, silently truncating
      eval episodes at 2,048 steps instead of 10,000, understating return ~5x — noted
      in `rl/README.md`, fixed in stage 04's notebook): ~9,940–9,970, matching training.
      Checkpoint wired in as stage 04's warm-start source and confirmed working there
      too (fixed instance re-evaluates at ~9,940–9,953).
- [x] Stage 04 (domain randomization) training complete, all 6 phases. `nbconvert`
      crashed in its final export cell (`onnx`/`onnxscript` were never installed) —
      training itself finished cleanly regardless (confirmed via checkpoint files,
      not assumed). Installed the missing deps and finished the export by hand:
      verified the checkpoint with real rollouts (~9,900 return, full-length
      episodes), exported to ONNX → C, and verified the C header is numerically
      exact against the PyTorch model (matched to 6 decimals on a test input) before
      copying it into stage 03. Real finding along the way: the policy balances but
      drifts ~0.4-0.5m over a 50s episode (position penalty too weak relative to
      pitch/roll) — decided not to patch stage 04 for this, since stage 05's
      `cmd_vel=0` case already trains proper station-keeping as part of the same
      policy that also handles driving on command.
- [x] Stage 05 (command-conditioning) training complete, all 10 phases (retrained
      from scratch — see that folder's README for why). Notebook finished cleanly
      this time (no crash, since `onnx`/`onnxscript` were already installed).
      Independently verified before exporting: **station-keeping** (`cmd_vel=0`)
      holds within 0.06-0.3m over 25s under full domain randomization; **velocity
      tracking** drives in the commanded direction but settles at a stable ~38%
      of the commanded speed (0.095 vs 0.25 m/s target) — a real, confirmed
      steady-state undershoot, not a ramp-up artifact — usable as direction
      control, not precise speed control, until `sigma_vel` is tightened in a
      future pass. C header verified numerically exact against PyTorch (6
      decimals). `actor.h` copied into stage 03, **replacing stage 04's** — this
      policy supersedes it (balances + holds position + drives on command, as
      one policy); `uniwheel_bot.ino` updated for the 7-input observation with a
      `cmd_vel` placeholder for stage 06 to wire up.
- [x] Stage 06 written (`workspace/software/06-remote-control/`): WiFi AP +
      WebSocket + web UI, adapted from the reference repo's `balance_bot_rc.ino`
      (that layer is hardware-agnostic, reused close to as-is). One real
      difference: a single vertical slider sending `{vel}`, not a 2D thumbstick
      sending `{vel, yaw}` — this robot has no yaw actuator, so a steer axis
      would silently do nothing. `uniwheel_rc/uniwheel_bot_rc.ino` combines this
      with stage 03's firmware; `vel_slider_test/` tests the WiFi/UI layer in
      isolation first. Uses stage 05's already-verified `actor.h` directly, no
      new export needed. Also added `sim_remote_control.ipynb`: the same
      arrow-key drive control but in the MuJoCo viewer (no hardware needed) —
      loads the real stage-05 checkpoint and runs the actual trained policy,
      not a raw motor pass-through. Validated for real (this dev session did
      have a usable display after all): ran with the actual GLFW viewer open,
      held position at `cmd_vel=0`, then drove forward cleanly once `cmd_vel`
      was raised (x position climbing to +0.86m over 12s), no tipping.
- [x] Found and root-caused a real hardware ceiling on roll-disturbance
      recovery — not a training gap. `sim_remote_control.ipynb` testing found
      roll recovers from much smaller disturbances than pitch (~0.40N vs
      ~0.90N sustained). A full training-continuation attempt aimed at closing
      that gap (roll-only, guaranteed, sustained disturbances every episode -
      see `05-command-conditioning/README.md`'s "Phase 11" sections for the
      full trial-and-error) produced **zero improvement**, which prompted
      checking the actual physics before spending more compute: this robot's
      real center of mass sits 113.2mm above the ground contact (confirmed via
      MuJoCo's `subtree_com`, correcting a stale 65.6mm figure in
      `docs/feasibility_analysis.md` that predated the final kinematic tree),
      and the FIT1034 motor's only *verified* torque spec (DFRobot doesn't
      publish a stall/max-torque figure) is its rated 0.03N·m. Those two
      numbers fix a hard critical angle: **past ≈3.92° of roll tilt, gravity's
      torque exceeds the reaction wheel's maximum possible torque, full stop,
      regardless of policy quality.** Matches the data almost exactly (0.40N
      survives at 2.73° peak roll, 0.45N fails and blows straight past the
      ceiling). Decision: keep stage 05's checkpoint (phase 10) as final -
      it already performs at close to this hardware's real limit. Closing this
      gap further would need a mechanical change (stronger motor, lower CG),
      not more training.
- [ ] Hardware bring-up: sensor_test → static_inference_test → uniwheel_bot →
      uniwheel_rc, in that order (blocked on hardware arriving). All software
      stages (01-06) are otherwise complete.

## A note on the OnShape API key

This project was bootstrapped with an OnShape API key pasted directly into chat
during setup. Treat that key pair as burned — rotate it at
https://dev-portal.onshape.com/keys. Credentials belong in `.env` (already
gitignored here), never in chat or in a committed file.
