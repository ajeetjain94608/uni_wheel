# 05 — Command conditioning

`uniwheel_env_cmd.py` — extends `04-domain-randomization/uniwheel_env_cmd.py` with a commanded
forward/back velocity (`cmd_vel`) added to the observation and reward. `train_with_ppo_cmd.ipynb`
— 10-phase curriculum: balance → position/rate penalties → small velocity commands → full
velocity range → mid-episode resampling → the same 5 domain-randomization phases as stage 04.

## No yaw command — this isn't an oversight
The reference repo's `balance_bot_env_cmd.py` commands both forward velocity *and* yaw rate,
because its differential-drive wheels can actually steer. Neither of our actuators (`wheel_motor`:
axis Y, `reaction_wheel_motor`: axis X) has any torque component about the yaw axis — there's
nothing to condition a yaw command on, same reasoning as stage 04's `yaw_penalty_coef=0.0`. Roll
is likewise never commanded — it's always regulated to zero (`roll_penalty_coef`), since there's
no operational reason to want a nonzero roll the way a mobile robot legitimately wants a nonzero
yaw rate to turn.

## Retrains from scratch — not a choice, a constraint
Unlike stage 04 (identical observation space to stage 02, so warm-startable), adding `cmd_vel`
changes the observation size (6 → 7), which changes the actor/critic's input layer shape. A
stage-04 checkpoint's weights cannot load into this env's `Agent` — first-layer shape mismatch.
The reference repo's own `train_with_ppo_cmd.ipynb` retrains from scratch for the identical
reason (its own cmd env adds 2 observation dims, not 1, for the same architectural reason).

## Validated before writing the full notebook
Ran the actual `UniWheelEnv` + `PPOConfig` + `train()` integration through a real (if short)
training loop — confirmed `obs_size=(7,)`, `action_size=(2,)`, no crashes — before writing out
all 10 phases, same as every other stage's env in this project.

## Status: prepared, not yet run
Waiting on stage 04 to finish (not as a dependency this time — this notebook trains from
scratch, so it could in principle start anytime; holding off just so the machine isn't running
two long training jobs at once).

## Training complete (all 10 phases)

Notebook completed cleanly this time (no crash, unlike stage 04 — `onnx`/`onnxscript` were
already installed from fixing that). Converted `actor.onnx` → `actor.h` with the standalone
script (never part of the notebook itself, same as stage 04): handled the newer external-data
ONNX format (`actor.onnx` + `actor.onnx.data`) with no issues.

Per-phase trajectory (`best` / `final` mean_return_10ep) — declining ceiling from Phase 5
onward is expected (each phase stacks harder randomization on a harder task, not a worse
policy), and `best ≈ final` in every phase means no phase handed forward a checkpoint worse
than its own peak:

| phase | best | final |
|---|---|---|
| 1 (balance) | 9,985 | 9,977 |
| 2 (position/rate) | 19,975 | 19,975 |
| 3 (small vel cmd) | 19,959 | 19,848 |
| 4 (full vel range) | 19,855 | 19,855 |
| 5 (resampling) | 18,728 | 17,586 |
| 6 (obs noise/delay) | 17,012 | 16,473 |
| 7 (motor noise/pushes) | 16,570 | 16,570 |
| 8 (mass/friction) | 17,927 | 17,105 |
| 9 (motor gain) | 13,738 | 12,868 |
| 10 (ridge torque) | see checkpoint below | |

## Independently verified before exporting (not just trusted training logs)

- **Station-keeping** (`cmd_vel=0`): 3 seeds, all survived 5,000/5,000 steps, drift 0.06-0.30m
  over 25s — solid, holds up under the full stack of domain randomization.
- **Velocity tracking** (`cmd_vel=0.5`, target 0.25 m/s): drives in the correct direction, but
  **settles at a stable ~0.095 m/s steady state, not the commanded 0.25 m/s** — confirmed this
  isn't a slow ramp-up (checked out to 3,000 steps, velocity is flat from ~step 500 onward).
  Real, working direction control; not accurate speed control. Likely cause: `sigma_vel=0.1`'s
  Gaussian tracking reward is forgiving enough (~0.79 reward even at this size of error) that
  the policy settled for "close enough, safely" over pushing harder toward the exact target.
  Worth tightening `sigma_vel` in a future pass if precise speed matters; fine as direction-only
  control for now (e.g. stage 06's remote control).
- **C header numerically verified**: compiled `actor.h` standalone, compared against the actual
  PyTorch model on the same test input — matched to 6 decimal places (1.503294 / -0.176048).

`actor.h` copied into `../03-sim-to-real/uniwheel_bot/` and `.../static_inference_test/`,
**replacing stage 04's** — this policy supersedes it (balances, holds position, *and* drives on
command, as one policy). `uniwheel_bot.ino` updated for the 7-input observation and a `cmd_vel`
placeholder (currently hardcoded to 0 = hold position) for stage 06 to wire up.

## Confirmed limitation: roll recovers from a much smaller disturbance than pitch

Found while testing `06-remote-control/sim_remote_control.ipynb` against a lateral
push (roll-inducing) vs a fore-aft push (pitch-inducing), same policy, same push
mechanism as training's `push_x`/`push_y` (see `04-domain-randomization/
uniwheel_env_dr.py`), swept up in magnitude on each axis independently:

| axis  | recovers cleanly up to | fails at | vs. training's 0.3N push range |
|-------|------------------------|----------|----------------------------------|
| pitch (wheel motor)      | 0.90N | 0.95N | ~3x   |
| roll (reaction wheel)    | 0.40N | 0.45N | ~1.3x |

Checked whether this is a hardware/actuator ceiling before concluding anything: the
two actuators are physically near-identical (same ~0.115kg rotor mass, same gear=0.03,
near-identical inertia tensors), so no asymmetry there. Logged actual control signal
use right at each axis's own failure boundary: at pitch's boundary (0.90N), `wheel_ctrl`
saturates to 1.000 (full torque, no headroom left). At roll's boundary (0.40N),
`rw_ctrl` peaks at only 0.348 - the reaction wheel is leaving roughly 65% of its torque
budget unused even when about to tip.

**Original conclusion (since corrected below): thought this was a training-exposure gap,
not a physical limit.** Domain randomization only ever pushed up to
`push_force_max_n=0.3N` (both axes, same range), so the policy never needed to react to
a larger roll disturbance - the 65%-unused-torque reading at the 0.40N boundary looked
like real headroom being left on the table. **This turned out to be the wrong
conclusion** - see "Phase 11 outcome: this was a hardware ceiling, not a training gap"
below, which found the missing piece: 0.348 was never "idle headroom" at all, it was
roughly what's actually needed to arrest a ~2.7° deviation given the real physics - the
apparent slack was an artifact of not yet having done the torque-vs-gravity calculation.

## Continuation training: phase 11 (roll-only, sustained pushes)

Follow-up to the "Confirmed limitation" finding above. Two design decisions worth recording:

1. **Sustained pushes, not just bigger ones.** Training's push mechanic re-rolled a fresh
   random force *every single 0.005s timestep* - so raising `push_force_max_n` alone would
   have made each of those instants punchier without ever teaching the policy to handle a
   real, held shove (which is what actually broke it in testing). `uniwheel_env_cmd.py` now
   supports `push_hold_steps_min`/`push_hold_steps_max` so a triggered push holds its force for
   multiple consecutive steps - defaults (1/1) keep the old single-step behavior byte-for-byte
   (verified before changing anything else).
2. **Roll-only, not both axes.** Originally planned to raise the shared `push_force_max_n` for
   both axes at once, since pitch and roll shared one knob - but pitch already recovers fine
   (~0.90-0.95N) and doesn't need more practice. Added `push_force_max_n_x`/`push_force_max_n_y`
   per-axis overrides so phase 11 leaves pitch's exposure at its original 0.3N and pushes only
   `push_y` (roll) up to 0.7N - all the extra training signal goes toward the axis that actually
   needs it. (A first attempt trained both axes together for ~5 minutes before this was caught
   and corrected - killed before any checkpoint was saved, no wasted result.)

Both changes smoke-tested (a few-iteration run with a real warm-start from phase-10, plus a
direct check that push_x stays capped at 0.3N while push_y reaches 0.7N) before committing to
the full ~1h20m run. `train_with_ppo_cmd_continuation.ipynb` warm-starts from phase-10's
checkpoint as `uniwheel-ppo-cmd-phase-11`. Will re-run the same disturbance sweep from the
"Confirmed limitation" section once it finishes, before touching `actor.h`.

**One more correction before this run stuck**: the push mechanic fires `push_x` and `push_y`
together on every trigger, so even capping pitch's magnitude at its old 0.3N still meant every
roll-recovery attempt was also fighting a simultaneous (if small) pitch disturbance - not a
clean roll-only exercise. Set `push_force_max_n_x=0.0` to remove pitch pushes entirely for this
phase (ridge torque on the wheel axle is unaffected and keeps some pitch practice alive
regardless). Caught and fixed before the second attempt's first checkpoint saved.

**Third correction**: `push_prob` alone left pushes to chance, so a short or early-terminating
episode (common right now, before the policy adapts to 0.7N) could complete without ever
encountering one - wasted training signal. Added `push_min_per_episode` to `DomainRandomConfig`:
schedules a guaranteed number of pushes at reset(), spread to random steps across the episode
(one per roughly-equal segment, so they land throughout, not clustered at the start) rather than
relying on a per-step coin flip. Phase 11 now guarantees 5 roll pushes every single episode.
Verified directly (a short test episode showed exactly 5 push events at the scheduled steps,
re-randomized on the next reset) and smoke-tested through a real warm-started train() call
before committing to the full run.

**Fourth correction**: `ridge_prob` (a wheel-axle-only, pitch-related disturbance carried over
unchanged from phase-10) fires independently every single step. Measured directly: over a
10000-step episode it fired ~480 times, affecting ~504 steps - vastly outnumbering the 5
guaranteed roll pushes (156 steps). This was genuinely diluting the phase's roll-only focus with
a much more frequent pitch-axle disturbance. Turned off entirely for phase 11
(`ridge_prob=0.0`) - verified directly (0 ridge-affected steps, ~184 roll-push-affected steps)
before relaunching. Phase-10's checkpoint already has ridge-torque robustness from its own
dedicated phase; one short continuation phase without it shouldn't meaningfully erode that.

## Phase 11 outcome: this was a hardware ceiling, not a training gap

Phase 11 (above) was executed correctly - verified roll-only, verified guaranteed-per-episode,
verified ridge-torque-off, smoke-tested before every launch - and produced **no improvement**.
The resulting checkpoint fails at the identical 0.45N boundary as phase-10, and in a direct
side-by-side trace at that exact force, is if anything *less* reactive (`rw_ctrl` stays flat
around -0.05 as roll grows to -12°, vs phase-10's `rw_ctrl` climbing to +0.137 over the same
window) - not evidence of a bug, but the signature of PPO fine-tuning around an existing
policy rather than discovering a new strategy, when there may not have been a better strategy
to discover in the first place.

That prompted checking the actual physics before spending more compute chasing it further:

- Total mass 0.395kg, overall center of mass 11.3cm above the ground contact point (both
  numbers direct from the compiled MJCF, not estimated).
- Gravity's destabilizing torque at roll angle θ (inverted-pendulum instability - once
  tilted, gravity pushes it further over): `M*g*h*sin(θ) = 0.438*sin(θ) N.m`
- Reaction wheel's max torque in the sim: `gear=0.03 * ctrl_max=1 = 0.03 N.m`
- **Critical angle where gravity's torque equals the reaction wheel's max torque:
  `sin(θ_c) = 0.03/0.438 = 0.0684` → `θ_c ≈ 3.92°`.** Past this angle, no torque the
  actuator can ever produce is enough to counter gravity, regardless of training - this
  matches the data almost exactly: the 0.40N push that survived peaked at 2.73° (just under
  the ceiling), the 0.45N push that failed blew straight past it and never came back.

Verified the `gear=0.03` figure against the real hardware before trusting this calculation
(DFRobot's own wiki/product pages for FIT1034/2804, https://wiki.dfrobot.com/fit1034/ and
https://www.dfrobot.com/product-3005.html - confirmed as the exact part ordered for this
build): rated voltage 12V, rated current 0.4A, max current 1A, rated torque ≈0.03N.m
(matches `gear=0.03` exactly), 220KV, confirmed direct-drive with no gearbox. **DFRobot does
not publish a stall/max-torque figure.** `feasibility_analysis.md`'s existing "~0.075N.m at
1A" estimate assumes torque scales linearly with current from the rated point
(`0.03N.m/0.4A * 1A`) - a real but unverified extrapolation. Cross-checked independently via
the motor's own KV rating (`Kt = 1/(KV_rad/s/V) ≈ 0.043N.m/A`), which disagrees with the
naive-scaling estimate by ~1.7x, showing genuine uncertainty in what the true max torque is.
Even the most optimistic of the three (0.075N.m → θ_c≈9.85°) falls well short of the 12°+
excursions phase 11's target disturbances (0.45-0.7N) were producing - so regardless of which
torque estimate is closest to reality, that specific training target was very likely
unreachable on this hardware from the start.

**Decision: keep phase-10 as the final policy.** `actor.h` (already phase-10's export, see
"Training complete" above) is unchanged. The roll-recovery envelope this robot's actual
motor can support is small (~4-10° depending on which torque estimate is used) - phase-10
already handles disturbances within that range (verified: clean recovery at 0.40N, peak
roll 2.73°). Chasing recovery from larger disturbances than that is a hardware question
(a stronger motor, or a mechanical redesign lowering the center of mass to reduce `h` in
the formula above), not a training one - out of scope here unless revisited later.
