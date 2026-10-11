# URDF → MJCF conversion notes

## Update: rebuilt from a newer OnShape export (2025-09-05, later same day)

Re-ran this whole pipeline against a fresh `Export as URDF` (213 links/212
joints vs. 182/181 before — same 2 continuous joints, same parent/child
pattern, so no structural surprises). Notable differences from the rebuild:

- **New export had ~75 degenerate/zero-eigenvalue `<inertia>` blocks** (mostly
  sub-microgram mesh-decomposition slivers — screws/standoffs OnShape split
  into tiny fragments). MuJoCo's URDF importer rejects a non-positive-definite
  inertia tensor outright (`inertia must have positive eigenvalues`). Fixed by
  eigen-decomposing every link's tensor and flooring any eigenvalue below
  `1e-11` before it reaches MuJoCo — harmless given the masses involved, but
  worth knowing this can happen on any re-export, not just this one.
- Total mass moved from 0.394323 kg → **0.394710 kg** (+0.4g) — a real, if
  tiny, change from whatever was edited in OnShape.
- **Chassis collision box got much narrower** (X-extent AABB: 211mm → 50mm) —
  the earlier box was likely catching some part that stuck out unrealistically
  far in X; that's gone now. Worth remembering this box is regenerated from
  the mesh AABB fresh each time, not hand-tuned, so re-running this pipeline
  after any CAD change will pick up geometry fixes automatically.
- CG-vs-wheel-axle offset re-checked with the same independent method: still
  ~0.1mm / -5mm (X/Y) — consistent with the previous export, confirms the
  earlier correction wasn't a fluke of one specific export.
- PID gains re-validated against the new model: same qualitative story
  (pitch and roll each work well alone, combined has real gyroscopic coupling)
  — see `workspace/software/01-test-model-in-mujoco/STAGE01_FINDINGS.md`.


Source: OnShape's own **Export as URDF** (not `onshape-to-robot` — that tool hit a
hard limitation, see `workspace/mechanical/onshape/uniwheel/EXPORT_NOTES.md` for
why it was abandoned). OnShape's built-in exporter correctly walks mates at any
nesting depth, so it picked up both motor-internal `REVOLUTE` mates without any
naming convention or CAD restructuring needed.

`uni_wheel_v2.urdf` here is the exported file, patched in two small ways (both
already applied):
1. Stripped the `package://uni_wheel_v2/meshes/` prefix from every `<mesh
   filename=...>` (MuJoCo doesn't resolve ROS package URIs).
2. Added an explicit `<mujoco><compiler .../></mujoco>` block, and a
   `world` → `root` **floating** joint. Without it, MuJoCo's URDF importer
   welds a URDF's root link rigidly to the world (fine for a static arm, wrong
   for a robot that needs to fall over) — this makes the chassis a free body.

## ⚠️ Correction: `fusestatic` computes the wrong merged CoM/inertia — do not trust it

**This is the important part of these notes.** The first pass through this
pipeline used `mujoco.MjModel.from_xml_path(..., fusestatic=True)` to collapse
the 182 individually-modeled links down to the 3 that actually move, then
`mj_saveLastXML` to get the merged body's mass properties. That produced a
chassis center-of-mass **18mm off the wheel axis** — which made it into
`docs/feasibility_analysis.md` and `workspace/software/01-test-model-in-mujoco/
STAGE01_FINDINGS.md` as a real design finding, complete with a controller
change (a wheel-speed damping term) built to compensate for it.

**It was wrong.** The user pushed back, having checked OnShape's own assembly
mass properties directly (centroid ~0.2mm/0.46mm off the origin — essentially
centered). That prompted a from-scratch, independent re-derivation:

1. Loaded the same URDF with `fusestatic="false"` — every one of the 182 links
   stays its own body, connected by `fixed` joints (MuJoCo still solves them
   rigidly, but doesn't merge/recompute a combined inertial the way
   `fusestatic` does).
2. Manually summed each link's mass-weighted world position to get the true
   merged center of mass, and combined each link's own inertia tensor (rotated
   into world axes) via the parallel-axis theorem to get the true merged
   inertia tensor, all in Python/numpy — a completely independent computation
   path from whatever `fusestatic` does internally.
3. Chassis CoM by this method: **(0.53mm, -0.35mm, 65.3mm)** — matches
   OnShape's own reported centroid (after weighting in the two wheels' small
   contributions) to **better than 0.1mm** on X and Y. `fusestatic`'s answer
   (18.3mm, -6.0mm, 42.2mm) was wrong on all three axes; only the **total
   mass** happened to come out right (0.1635 kg either way).
4. The two wheel bodies were *also* wrong via `fusestatic`, though less
   dramatically (~1-2mm position error) — so this isn't a chassis-specific
   fluke, it's `fusestatic` itself, generally, on a merge this size (~170
   links for the chassis, ~10-15 for each wheel — error seems to scale with
   how much gets merged).
5. Second bug caught in the same pass: my first attempt at applying the
   independently-computed fix used the merged inertia tensor **in world axes**
   for every body's `<inertial>` tag. That's only valid for the chassis, whose
   own body frame happens to be identity (≡ world) at the reference pose — for
   `wheel` and `reaction_wheel`, whose body frames are rotated relative to
   world, this silently misaligned their principal axes and broke their
   dynamics (symptom: roll became unstable at *any* gain, even alone). Fixed by
   rotating the merged tensor into each body's own local frame
   (`R_body.T @ I_world @ R_body`) before decomposing it into
   `pos`/`quat`/`diaginertia` — obvious in hindsight, easy to miss.

**Takeaway for next time:** `fusestatic` is convenient for collapsing the
kinematic tree, but **don't trust its computed `<inertial>` values** for a
merge this large — recompute them independently (mass-weighted CoM + parallel
axis, in the *target body's own frame*, not world) and cross-check the total
against a source-of-truth number (OnShape's own mass-properties API, in this
case) before building anything on top of it. `sum(model.body_mass)` matching
is necessary but nowhere near sufficient to trust the rest.

## How `uniwheel.xml` was actually built (current, corrected version)

1. Loaded the patched URDF with `fusestatic="true"` **only** to get the
   collapsed body/joint/geom structure (3 moving bodies, 2 hinges) — not its
   `<inertial>` values.
2. Separately computed the correct mass/CoM/inertia tensor for all three
   bodies as described above (`fusestatic="false"`, manual parallel-axis sum,
   each expressed in its own body's local frame) and used those instead.
3. Identified which rotor is which by checking world-frame joint axes at
   qpos0: one spins about world **Y** near the chassis origin (**`wheel`** —
   the ground/drive wheel, pitch axis, matches the reference repo's own wheel
   axis convention) and the other spins about world **X**, offset **+131.7 mm**
   in Z (**`reaction_wheel`** — roll axis, sits above the chassis).
4. Renamed bodies/joints (`root`→`chassis`, `revolute_1`→`wheel_joint`,
   `revolute_1_1`→`reaction_wheel_joint`).
5. Marked all ~300 imported mesh geoms **visual-only**
   (`contype="0" conaffinity="0" group="2"`) — mass is carried entirely by the
   explicit `<inertial>` tags, not by these geoms. Added clean primitive
   collision geoms instead, measured from the compiled mesh vertex data:
   - `wheel`: cylinder, radius **0.0475 m**, ~35.5 mm thick.
   - `chassis`: a coarse box from the merged body's AABB — generous on
     purpose, re-check against the actual render before trusting it for
     anything but "does it fall through the floor."
   - `reaction_wheel`: no collision geom — fully enclosed, never contacts
     anything.
6. Friction: no rubber tire (both wheels identical, bare PLA) — one
   `<default><geom friction="0.4 0.005 0.0001"/></default>` standing in for
   contact grip, reference-repo style, lower than their 0.8 (tuned for rubber).
7. Actuators (`<motor>`, gear=0.03 → ±0.03 N·m at `ctrl=±1`, the FIT1034's
   rated torque) and sensors (IMU accel/gyro/orientation + both joints'
   pos/vel).
8. Start height: chassis `pos` = `0 0 0.0475` so the wheel touches the ground
   at `qpos0`.

## Validated, not just assumed

- Loads cleanly, total mass 0.394323 kg.
- Overall robot CG now sits **1.5mm / -3.8mm** from the wheel axle (X/Y) —
  consistent with OnShape's own near-zero centroid, not the earlier wrong
  18mm figure.
- 10s free-fall sim with zero control: no NaNs, settles rather than exploding.
- See `workspace/software/01-test-model-in-mujoco/STAGE01_FINDINGS.md` for the
  (corrected) PID validation results.

## What's still approximate
- Chassis collision box (generous AABB, not a tight fit)
- IMU site position (placeholder, `0 0 0.02` in chassis frame)
- Joint damping/frictionloss (`0.001`/`0.001` — a guess, not measured bearing
  friction)
- Actuator gear (rated torque, not the ~0.075 N·m transient max)
