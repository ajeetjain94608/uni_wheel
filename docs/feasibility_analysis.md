# Feasibility Analysis — Uni Wheel V2 (reaction-wheel unicycle)

Numbers below are pulled directly from the live OnShape document (`Uni Wheel V2`,
document `8d580a9c25331d8779ad11df`) via the Onshape mass-properties API, and from
the DFRobot FIT1034 (2804 hollow-shaft BLDC gimbal motor + built-in AS5600 encoder)
datasheet. This is a first-pass, hand-calculated check, meant to catch a fatal
mismatch (or confirm there isn't one) before investing in the MuJoCo model and
firmware. The MuJoCo model (stage `01-test-model-in-mujoco`) will let us verify
this dynamically instead of with a static hand calc.

## 1. What the CAD actually is

The assembly is **two nominally-identical motor+disc modules**, bolted to a shared
PLA chassis, plus an ESP32 + MPU6050 + battery + two SimpleFOC Mini FOC driver
boards:

| Module | Role | Contents |
|---|---|---|
| **lower** | Pitch axis — drives the ground wheel | 2804 BLDC motor (`FIT1034`) + "fly wheel" disc **with a rubber tire ring bonded on** (this is the actual ground-contact wheel) + 626ZZ bearing + shaft + MPU6050 + ESP32 dev kit |
| **upper** | Roll axis — pure reaction/momentum wheel | Same 2804 BLDC motor + bare "fly wheel" disc (no tire) + 626ZZ bearing + shaft + battery + buck converter |

Both motors are driven via their own **SimpleFOC Mini** FOC controller board, commanded
by the ESP32 (which also reads MPU6050 + the two AS5600 encoders built into the FIT1034
motors). This is a sound and coherent architecture — confirms FIT1034 = the same 2804
gimbal motor already modeled in the CAD, used twice.

## 2. Measured mass properties (from OnShape, not estimated)

| Part | Mass | Notes |
|---|---|---|
| Fly wheel disc (PLA) ×1 | 45.66 g | spin-axis inertia **6.049e-5 kg·m²** (OnShape principal inertia) |
| 2804 motor, full (stator+rotor+bearing) | 67.81 g | axial inertia 1.040e-5 kg·m² |
| — of which rotor only | 31.49 g | axial inertia 6.032e-6 kg·m² (spins with the wheel) |
| — of which stator | ≈36.3 g | fixed to chassis |
| Shaft | 1.01 g | |
| Rubber tire ring | 1.03 g | ground wheel only |
| Battery | 31.25 g | |
| **Whole robot (as modeled)** | **399.6 g** | CG height ≈ 65.6 mm above the model's z=0 (assumed ≈ ground contact plane — to be confirmed once the kinematic tree is built) |

**Confirmed, and revised**: now that the kinematic tree exists (the working MJCF in
`workspace/mechanical/urdf/uniwheel/`), the real CG height above the ground contact plane
is **113.2 mm**, not 65.6mm - read directly from MuJoCo's own `subtree_com`, not
estimated. Total mass also settled at 394.7g (close to the 399.6g CAD estimate - the small
difference isn't the story here, the CG height is). This matters: see section 4 below,
where this height is the deciding factor in the reaction wheel's actual disturbance-
recovery ceiling.

**Per rotating wheel module** (disc + rotor + shaft): mass ≈ 78.2 g,
spin-axis inertia **I ≈ 6.65e-5 kg·m²** (disc + rotor summed, coaxial).
Both wheel modules are essentially identical, since it's the same disc and motor
reused for both roll and pitch.

## 3. FIT1034 motor (DFRobot, confirmed = the CAD's 2804 BLDC + AS5600)

Verified directly against DFRobot's own pages for this exact part
(https://wiki.dfrobot.com/fit1034/, https://www.dfrobot.com/product-3005.html - confirmed
as the exact part ordered for this build):

- No-load speed: 2600 RPM @ 12 V (≈ 272 rad/s)
- Rated voltage/current: 12V / 0.4A. Max current: 1A.
- Rated torque: ≈0.03 N·m (300 g·cm) @ 0.4 A rated - this is the *only* torque figure
  DFRobot actually publishes. **No stall/max-torque figure is published anywhere.**
- Max torque at 1A is therefore an estimate, and a genuinely uncertain one - two different
  extrapolation methods disagree by ~1.7x:
  - Naive linear scaling from the rated point: `0.03N·m/0.4A * 1A ≈ 0.075 N·m`
  - From the motor's own KV=220 rating (`Kt = 1/(KV_rad/s/V)`): `Kt ≈ 0.043 N·m/A` →
    `≈0.043 N·m` at 1A
  - Treat **0.03 N·m (the one number DFRobot actually states) as the only fully
    defensible figure** for anything safety- or capability-critical; 0.043-0.075 N·m is
    a plausible but unverified range for brief transients.
- Direct drive, no gearbox, on both axes (confirmed on both pages)

## 4. Roll axis (reaction wheel) — the one you specifically asked about

**Correction (this section originally used a stale CoM height, understating the real
constraint - see below):** the static torque margin was originally reported as
"comfortable" using `h=0.0656m`. Recomputed directly from the current, corrected MJCF
(`workspace/mechanical/urdf/uniwheel/uniwheel.xml`) via MuJoCo's own `subtree_com`
(not estimated - read straight from the compiled model): **h = 0.1132m**, total mass
0.3947kg - both roughly 1.7x the figures this section was originally built on, most
likely because the original calculation predates the mass/CoM fixes documented in
`workspace/mechanical/urdf/uniwheel/CONVERSION_NOTES.md`. This changes the conclusion
substantially - static torque margin is *not* comfortable, it's the binding constraint:

Required holding torque at tilt θ: τ = M·g·h·sin(θ), with **M=0.3947 kg, h=0.1132 m**
(current, verified values):

| θ | τ required | vs. 0.03N·m (rated, only verified figure) |
|---|---|---|
| 2° | 0.0153 N·m | 51% of budget |
| **3.92°** | **0.0300 N·m** | **100% - the critical angle** |
| 10° | 0.0761 N·m | 254% - impossible |
| 20° | 0.1500 N·m | 500% - impossible |
| 30° | 0.2192 N·m | 731% - impossible |

**Past ≈3.92° of roll tilt, gravity's torque exceeds the reaction wheel's maximum
possible torque (using the only verified spec, 0.03N·m) - full stop, regardless of
control strategy.** Using the more optimistic (but unverified) 0.075N·m estimate pushes
this out to ≈9.85°, still nowhere near the 30° figure this section originally treated as
comfortably within margin. See `workspace/software/05-command-conditioning/README.md`'s
"Phase 11 outcome" section for how this was discovered (an RL continuation-training
experiment aimed at extending roll disturbance recovery produced no improvement, which
led to this recalculation - confirming it's a hardware ceiling, not a training gap).

**The real constraint is reaction-wheel momentum saturation — this is the thing to
design around:**

A reaction wheel doesn't drive against the ground, so it can't "walk out from under"
a sustained tip like the pitch wheel can — it can only *store* momentum, and it has a
hard ceiling (motor no-load speed).

- Max flywheel angular velocity: ω_max ≈ 272 rad/s (2600 RPM)
- Max stored angular momentum: L_max = I·ω_max = 6.65e-5 × 272 ≈ **0.0181 kg·m²/s**
- For a *transient* disturbance (a bump, a push), this is plenty — it absorbs the
  impulse and unwinds it back to near-zero speed once the disturbance clears.
- For a *sustained* bias torque (uneven CG, a slight floor slope, a lopsided battery
  wire), the wheel has to keep accelerating to hold the tilt, and **will run out of
  RPM in a bounded time**: t_sat = L_max / τ_bias. E.g. a small 5 mN·m steady bias
  saturates the wheel in ≈3.6 seconds.

**This is the #1 thing to design for, not raw torque.** Options, roughly in order of
how the reference repo's own robot would map to this:
1. Get the mechanical CG as close to zero roll-bias as possible (this is a real,
   testable CAD check — worth doing before machining).
2. Add a desaturation behavior to the controller: periodically let the body lean
   slightly to bleed the wheel back toward zero speed (the equivalent of what the
   reference repo's PPO policy would learn on its own if the training environment
   models wheel-speed limits and includes it in the reward/observation — this should
   be modeled explicitly in the MuJoCo env, not just tacked onto firmware later).
3. This also argues for including flywheel speed in the RL observation space and
   penalizing wheel-speed saturation in the reward, so the trained policy learns to
   manage momentum rather than just angle — worth deciding before stage
   `02-balance-bot-with-ppo` is written.

## 5. Pitch axis (ground wheel) — secondary risk, likely fine

Direct-drive (no gearbox) at ~52 mm wheel radius gives a max rim force of
τ_max/r ≈ 0.075/00.052 ≈ 1.4 N. For a 400 g robot this is enough for normal balance
correction (sub-1g accelerations), though very fast/large recoveries could be
torque-limited. Gimbal-style BLDC motors (which the FIT1034 is) are specifically
low-cogging, so low-speed smoothness near zero — usually the failure mode for
direct-drive wheels — should be less of a problem here than with a generic BLDC.
Worth validating in sim once real friction/backlash numbers are in the MJCF.

## 5b. Update from stage 01 (real simulation, not just this hand calc)

Once the actual MJCF was built and PID-tested (see
`workspace/software/01-test-model-in-mujoco/STAGE01_FINDINGS.md`), the CG
offset originally reported here turned out to be a bug in the conversion
pipeline, not a real property of the design — the whole-robot CG actually
sits ~1.5mm/-3.8mm from the wheel axle, matching OnShape's own mass properties
to well under a millimeter (see `workspace/mechanical/urdf/uniwheel/
CONVERSION_NOTES.md` for the full story). The static torque-margin numbers
above hold as originally computed. The one thing stage 01 *did* turn up that's
real: a simple decoupled PID on each axis handles pitch or roll disturbances
individually just fine, but the two together expose genuine gyroscopic
coupling between the two spinning wheels — motivation for RL, not a hardware
concern.

## 6. Bottom line

Nothing here rules the design out — the motor has ample static torque on both
axes. The one substantive physical risk is **reaction-wheel saturation under any
sustained roll bias**, which is a controller/CG design problem, not a "wrong motor"
problem. Next step: get real numbers out of the MuJoCo model (stage 01) to replace
the "τ∝I" and "h≈65.6mm-from-z=0" assumptions above with values read straight out of
the compiled model.
