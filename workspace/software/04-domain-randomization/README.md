# 04 — Domain randomization

`uniwheel_env_dr.py` — `UniWheelEnv` extended with a `DomainRandomConfig`, layered on the
identical observation/action/reward structure as stage 02's env. `train_with_ppo_dr.ipynb` —
a phased curriculum, same pattern as the reference repo (each phase continues training the
previous phase's agent, randomization added incrementally rather than all at once).

## Deliberately deviates from the reference repo's structure
Their notebook redoes a "balance only" → "add position/yaw penalty" curriculum from scratch
before ever touching domain randomization, because their base training used a stripped-down
reward. Ours (`02-balance-bot-with-ppo/uniwheel_env.py`) already trains with position/yaw
penalties on by default, so `train_with_ppo_dr.ipynb` **warm-starts from stage 02's finished
`best_model` checkpoint** instead — Phase 1 is just loading those weights and confirming they
still work in this new env class, not re-deriving what stage 02 already did. Verified this
warm-start actually works (not just assumed): loaded stage 02's in-progress checkpoint into an
`Agent` built against `uniwheel_env_dr.py`'s spaces and ran a real `evaluate()` call — loads
cleanly, balances the full 500-step episode cap with no DR enabled, as expected.

## One physically-motivated deviation in the randomization itself
"Ridge torque" (random axle disturbance modeling a tire ridge hitting the ground) is applied
**only to the `wheel` joint, never `reaction_wheel`** — the reaction wheel is fully enclosed
and never contacts the ground (see `docs/feasibility_analysis.md`), so it has no tire to hit a
ridge with. The reference repo's version (a 2-wheel bot where both wheels touch the ground)
applies it to both; ours would be training against a disturbance that can't physically occur.

## Phases (after the stage-02 warm-start)
1. Observation noise + action delay (IMU noise, I2C/motor response latency)
2. Motor noise + random pushes
3. Mass + friction randomization
4. Motor gain randomization (battery sag, part-to-part motor variance)
5. Ridge torque (wheel only)

## Status: prepared, not yet run
Waiting on stage 02's training run to finish (it's the warm-start source). Once it's done:
1. Fill in `STAGE02_CHECKPOINT` in `train_with_ppo_dr.ipynb` with the printed `best_model_path`.
2. Run it the same way as stage 02 (`jupyter nbconvert --to notebook --execute --inplace`).

Ends with the same `actor.onnx` → `actor.h` export path stage 03 expects.

## Training complete (all 6 phases)

`jupyter nbconvert --execute --inplace` crashed in its final export cell — `onnx`/`onnxscript`
were never installed, so `export_actor_onnx()` raised `ModuleNotFoundError`. Because nbconvert
only writes the `.ipynb` file back on a clean finish, the crash meant the notebook's own file
still shows its pre-run state and never recorded any output, even though **all 6 phases of
actual training completed successfully** (confirmed via checkpoint file timestamps and content,
not just assumed from the missing notebook output).

Installed the missing deps and finished the export manually from the real checkpoint on disk:
- Independently verified both `best_model` (mean ~9,900, all 5 test seeds surviving the full
  10,000-step episode) and the phase's `_final` checkpoint (~9,760) with real rollouts before
  choosing which to export.
- Exported `best_model` → `actor.onnx` → `actor.h`.
- **Verified the C conversion is numerically exact**, not just assumed: compiled `actor.h`
  standalone, ran it on the same test observation as `static_inference_test.ino`'s, and compared
  against the actual PyTorch model's output — matched to 6 decimal places (1.185565 / -0.397036
  both sides).
- Copied `actor.h` into `../03-sim-to-real/uniwheel_bot/` and `../03-sim-to-real/
  static_inference_test/`, ready for hardware bring-up once it arrives.

Net result: stage 04 is genuinely done, with a policy that balances (drifts ~0.4-0.5m/episode,
see the earlier section on this) and handles obs/motor noise, action delay, pushes, mass/
friction/motor-gain variation, and ridge torque - just doesn't hold position, which is what
stage 05 is for.
