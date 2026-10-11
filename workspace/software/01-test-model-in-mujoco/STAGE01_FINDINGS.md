# Stage 01 findings

Goal was: load `uniwheel.xml`, hand-tune a simple PID, confirm the model *can*
balance before touching RL.

## Correction first: the earlier version of this file was wrong

The first pass reported an "~18mm CG offset from the wheel axis" as a real
mechanical finding, with a controller built around compensating for it. **The
user checked OnShape's own mass properties directly and called this out as
wrong — correctly.** The cause: MuJoCo's `fusestatic` compiler option computes
the wrong merged center-of-mass/inertia for a merge this large (got total mass
right, position/orientation wrong). Full technical detail and the independent
re-derivation that fixed it are in
`../../mechanical/urdf/uniwheel/CONVERSION_NOTES.md`. The real number: the
whole-robot CG sits **1.5mm / -3.8mm** from the wheel axle — essentially
centered, matching OnShape to <0.1mm. Everything below is re-tuned against the
corrected model.

## What's actually true

**Pitch (`wheel_motor`) works very well on its own.** With the CG genuinely
centered, a plain PD (`KP=30, KD=1`, no CG-compensation term needed) converges
to under 1° from *any* single-axis pitch disturbance tested, up to 20°.

**Roll (`reaction_wheel_motor`) also works very well in isolation.** Tested
down to `KP=1, KD=2` recovering cleanly from a 5° roll disturbance with pitch
control switched off entirely.

**The two together, as independent per-axis PID loops, do not combine
cleanly — and this time it's real gyroscopic coupling, not a design flaw.**
With both loops active (`KP_pitch=30` in particular), roll fails to recover
from even a small (5°) disturbance, regardless of what roll gains are tried.
Isolating the effect: turning pitch control off restores roll's clean
recovery immediately. The explanation that fits: correcting pitch means
actively accelerating/decelerating the ground wheel, and a wheel with
meaningful angular momentum precesses when the chassis itself rotates about a
different axis — exactly the gyroscopic coupling a decoupled per-axis PID
can't see, because each loop only looks at its own axis's angle and rate.

Validated result, `KP_pitch=30, KD_pitch=1, KP_roll=10, KD_roll=1`:

| initial disturbance | outcome |
|---|---|
| none, or pitch alone (5°–20°) | recovers to <1° every time |
| roll alone (5°–20°) | **falls** (with pitch loop active) |
| combined pitch+roll | falls in most combinations tried, recovers in a couple by coincidence |

**Left this as the stage-01 result on purpose**, same conclusion as before but
now for the right reason: this is exactly the gap a learned (PPO) policy,
trained on the full coupled dynamics rather than two decoupled loops, is
expected to close. Not a blocker for stage 02.

## Validated (not just simulated once and hoped)
- Sign/direction of both actuators checked independently before combining.
- Gains grid-searched, not hand-guessed (multiple sweeps — see the session).
- The gyroscopic-coupling explanation was tested directly (toggling pitch
  control on/off with roll gains held fixed), not just inferred.

## Re-validated against a newer OnShape export

The model was rebuilt from a later `Export as URDF` (see
`../../mechanical/urdf/uniwheel/CONVERSION_NOTES.md` for what changed — small
mass increase, much tighter chassis collision box, CG offset confirmed again
independently). Same gains, re-checked: pitch alone still recovers cleanly for
small-to-moderate disturbances, roll alone still works in isolation, and the
combined-loop gyroscopic coupling limitation is still there. Pitch's clean
range shrank a little (now solid to ~10°, not 20°) with the updated mass
distribution — a reminder that these gains are tied to a specific model
snapshot and should be re-checked (not assumed) after any further CAD change,
same as everything else in this file.

## Carried over from `CONVERSION_NOTES.md`, still open
- Chassis collision box is a generous AABB, not a tight fit.
- IMU site position is still a placeholder (`0 0 0.02` in chassis frame).
- Joint damping/frictionloss (`0.001`/`0.001`) are guesses, not measured
  bearing friction.
