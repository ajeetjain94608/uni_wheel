# onshape-to-robot export notes (abandoned path — kept for the lessons learned)

**This path was abandoned.** `onshape-to-robot`'s hard requirement — mates must
live in the flat top-level assembly, named `dof_*` — proved awkward for a
design where the real joints are the motors' own internal rotor/stator mates.
OnShape's own **Export as URDF** feature has no such limitation (it walks mates
at any nesting depth) and worked on the first real attempt. The working model
is at `workspace/mechanical/urdf/uniwheel/uniwheel.xml` — see
`CONVERSION_NOTES.md` next to it for how it was built. Config/generated output
from this abandoned path have been deleted; the diagnosis below is kept only
because the naming-convention gotcha is a useful thing to know if you ever do
reach for `onshape-to-robot` again.

## Correction (superseding the "mark subassemblies Flexible" advice below)

That advice was wrong — you made the change, still 0 DOF. I read
`onshape_to_robot/assembly.py` directly instead of guessing further. The actual
rule (`process_mates()` / `feature_mating_two_occurrences()`):

1. **Only mates that exist directly in the top-level assembly's own `features`
   list are ever considered.** Subassembly-internal mates are never read, at any
   nesting depth, flexible or not — this is a hard architectural limit of the
   tool, not a setting.
2. **A mate only becomes a robot joint if its name starts with `dof_`.** A
   `REVOLUTE`-type mate named e.g. `Revolute 1` is otherwise just... ignored (not
   even merged as fixed — it's invisible to the DOF pass entirely, which is why
   you got 0 DOF with the mates present and correctly typed).
3. Bonus: if any underscore-separated token in the name (after stripping `dof_`)
   is literally `wheel` or `continuous`, the joint is exported as **CONTINUOUS**
   (free-spinning, no angle limit) instead of a limited `REVOLUTE` — which is what
   you want for both wheels.

### The actual fix, in OnShape

Your `Revolute 1` mates currently live *inside* `uni wheel assembly lower` and
`uni wheel assembly upper` — invisible to the exporter regardless of flexible
state. You need two mates that exist **directly in the top-level `Uni Wheel V2`
assembly tab** (you can still select the shaft/disc faces even though they're
nested several levels down — Onshape lets you drill into the tree and pick
geometry from the top assembly context):

1. In the **`Uni Wheel V2`** tab (not a subassembly tab), create a Revolute mate
   between the ground-wheel disc/shaft and the lower body plate. Name it
   **`dof_wheel`** (→ joint `wheel`, auto-CONTINUOUS since `wheel` is a token).
2. Create a second Revolute mate between the reaction-flywheel disc/shaft and the
   upper body plate. Name it **`dof_reaction_wheel`** (→ joint `reaction_wheel`,
   also auto-CONTINUOUS).
3. The old internal `Revolute 1` mates inside the two subassemblies can stay —
   they're inert for export purposes either way, just don't rely on them.

Then re-run:
```
source .env && python3 -m onshape_to_robot.export workspace/mechanical/onshape/uniwheel
```
Log should say `Found total 2 degrees of freedom` with `+ Found DOF: wheel
(continuous)` and `+ Found DOF: reaction_wheel (continuous)`.

## Friction / no rubber tire

Per your latest message: both wheels are identical, no rubber tire modeled. We're
following the reference repo's approach exactly — a single `<default><geom
friction="..."/></default>` in the MJCF standing in for contact grip, rather than
modeling tire compliance. The reference repo uses `0.8 0.005 0.0001`
(sliding/torsional/rolling) as "an estimate for rubber on a floor/desk"; since
yours is bare PLA-on-floor, not rubber, I'll use a lower sliding-friction
estimate (~0.3–0.4) once the model's in, and it's trivially tunable from there.

## Status (superseded parts of the original note below kept for the mass/geometry
findings, which are still accurate — only the DOF diagnosis was wrong)

Mass matches OnShape's own mass-properties API exactly (0.399556 kg) and the
`ignore` list correctly strips decorative electronics/bearing geometry without
losing their mass contribution. Only the joint/DOF issue above needs the CAD fix.
