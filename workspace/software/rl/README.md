# rl — shared training utilities

Copied from the reference repo essentially unmodified (both are environment-agnostic):

- `ppo_trainer.py` — the PPO trainer (`PPOConfig`, `Agent`, `train()`, `evaluate()`,
  `export_actor_onnx()`, `export_tb_plots_as_csv()`). Takes any Gymnasium env; nothing
  in here is specific to the 2-wheel bot it was written for or the uni wheel bot using
  it now.
- `onnx_actor_to_c.py` — converts a trained actor's exported `.onnx` into a plain C
  header for ESP32 inference with no library dependency (stage 03).

Dependencies installed for this: `torch` (CPU wheel), `gymnasium`, `tensorboardX`.

## Status

Smoke-tested against `UniWheelEnv` (see `../02-balance-bot-with-ppo/README.md`) — a
short training run, checkpointing, and best-model saving all confirmed working
end-to-end before committing to a real training run.

## Known quirk in `evaluate()`

`max_steps = getattr(eval_envs.envs[0], 'max_steps', config.num_steps)` silently falls back to
`config.num_steps` (2048) instead of the env's real `max_steps` (10000 for `UniWheelEnv`) —
`gym.wrappers.RecordEpisodeStatistics` doesn't forward custom attributes through to the wrapped
env, so the `getattr` always misses and hits the fallback. Effect: `evaluate()` truncates each
eval episode at 2048 steps instead of letting it run the full episode length, understating the
return by ~5x for an env with `max_steps=10000` (confirmed exactly: stage 02's final eval
reported 2,040.6 instead of the ~9,940-9,970 a full-length rollout actually gets). Not a policy
problem - re-run a rollout manually at the real `max_steps` if `evaluate()`'s number looks
suspiciously low, rather than trusting it blindly.
