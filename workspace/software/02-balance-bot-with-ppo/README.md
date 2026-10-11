# 02 — Balance bot with PPO

Gymnasium env (`uniwheel_env.py`, class `UniWheelEnv`) wrapping the MuJoCo model, trained
with the shared PPO trainer in `../rl/ppo_trainer.py` (same trainer the reference repo
uses — it's environment-agnostic, no changes needed).

Two things differ from the reference repo's 2-wheel bot, both driven directly by stage 01:

- **2 actuators, 2 tracked axes** (`wheel_motor` for pitch, `reaction_wheel_motor` for
  roll) instead of 1 axis / 2 differential motors.
- **Both wheel speeds are in the observation**, not just pitch/roll/rates. Stage 01 found
  a hand-tuned decoupled PID can't handle combined pitch+roll disturbances (real
  gyroscopic coupling between the two spinning wheels) — a policy that can at least see
  wheel speed has the information needed to manage that; a policy that can't, structurally
  can't learn to.

## Files

- `uniwheel_env.py` — the gym env
- `env_test.ipynb` — sanity check (spaces, `check_env`, random policy — expected to fall
  quickly, see the module docstring)
- `train_with_ppo.ipynb` — configure `PPOConfig`, build the vectorized env, train

## Status

Smoke-tested (5 PPO iterations, 40,960 total steps): trainer/env/reward wiring all
confirmed working end-to-end, mean episodic return climbed 52 → 66 → 68 → 69 across
those 5 iterations, checkpointing and best-model saving both confirmed. That's nowhere
near enough training to expect a balanced policy — `train_with_ppo.ipynb` is configured
for a real run (`NUM_ENVS * STEPS_PER_ENV` — 12M steps by default); scale to what your
hardware/patience allows.

**GPU (GTX 1650) measured, not assumed — it doesn't help here.** This model is a tiny
64-unit MLP driving a cheap 4-body MuJoCo sim; `cuda=True` measured *slower* than CPU
(14.9s vs 9.8s on the same 5-iteration test) since PCIe transfer overhead for such small
tensors outweighs anything the GPU saves. The actual lever is `NUM_ENVS`: 4→12 (this
machine's core count) measured ~1.5x more steps/sec for free. True multiprocessing
(`AsyncVectorEnv`) was also tried — only ~10% faster than `SyncVectorEnv` at the same
env count, and needs a small shim around the shared trainer's render-only `envs.envs[0]`
access — not worth it for that little gain, so the notebook stays on `SyncVectorEnv`.

## Evaluation of first training run (12% through, still running in background)

Checked the `best_model.cleanrl_model` checkpoint against two disturbance types:
- **In-distribution** (velocity kicks from upright, matching `reset()`): survives up to
  ±1.5 rad/s on both axes at once — 3x beyond the ±0.5 rad/s it was trained on.
- **Out-of-distribution** (starting from an already-tilted angle, the same scenarios
  stage 01's hand-tuned PID was tested against): falls almost immediately in most cases
  (worse than the hand-tuned PID on these specific cases) — `reset()` never shows the
  policy a pre-tilted starting state, only upright + a velocity kick, so this is
  genuinely unseen territory for it, not a training failure.
- Checked the reference repo's own domain-randomization env (`04-domain-randomization/
  balance_bot_env_dr.py`) — it has the identical limitation (randomizes mass/friction/
  motor-gain/action-delay, never initial tilt), in every stage, so this isn't something
  we're missing relative to reference.
- **Decision: not worth fixing.** The real robot never starts from a meaningful tilt in
  practice (always upright, only ever gets velocity disturbances) — so the
  out-of-distribution result above, while real, doesn't reflect a scenario this robot
  will actually encounter. Training continues on the current `reset()` as-is.

## Training complete

Best mean return: **9,970.5** (climbed 430 → 6,187 → 9,093 → 9,783 → 9,936 → 9,970 over the
full run). Checkpoint: `runs/UniWheel-v0__uniwheel-ppo__42__1788678088/best_model.cleanrl_model`.

Independently verified with a full 10,000-step rollout (bypassing a bug in the shared
`evaluate()` helper - see `../rl/README.md`): ~9,940-9,970 return, both deterministic and
stochastic action selection. Ready to use as stage 04's warm-start checkpoint.
