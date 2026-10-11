# 01 — Test model in MuJoCo

Load `workspace/mechanical/urdf/uniwheel/uniwheel.xml` (see
`CONVERSION_NOTES.md` next to it for how it was built from OnShape's URDF
export), confirm it stands under gravity, and hand-tune a simple PID (`wheel_motor`
torque from pitch error, `reaction_wheel_motor` torque from roll error) to check
the model *can* balance before any RL is involved. This is also where we replace
the remaining approximations flagged in `CONVERSION_NOTES.md` (chassis collision
box, IMU site position, joint damping/frictionloss) with values read/tuned
against the actual model, and update `docs/feasibility_analysis.md`'s hand-calc
assumptions (CG height, spin inertia) with numbers read directly from it.
