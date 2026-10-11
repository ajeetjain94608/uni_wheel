# 03 — Sim to real

Two references were used here, for different reasons:

- **[macolin007/mcdgf](https://github.com/macolin007/mcdgf)** — a two-wheel
  self-balancing bot using the *identical* hardware stack (ESP32, SimpleFOC Mini x2,
  AS5600 x2, MPU6050). Its `main.cpp` is where the pin scheme, dual-I2C-bus setup
  (AS5600's I2C address is fixed — two of them can't share a bus), and SimpleFOC
  driver/motor init all come from. **Important finding from checking it**: SimpleFOC
  Mini has no onboard MCU or command protocol of its own — it's a bare gate-driver
  breakout. The *host* (ESP32) has to run the full SimpleFOC control loop for both
  motors itself. mcdgf's own `main.cpp` does this in one plain sequential `loop()`,
  no dual-core/FreeRTOS task split needed — simpler than initially assumed.
- **The reference RL repo's `03-sim-to-real/balance_bot.ino`** — for the *loop shape*
  only (complementary filter → build obs → `actor_forward()` → clamp → command
  motors → tip-detect/recover). Its actual hardware code (M5Unified, a commercial
  "BALA2" I2C board) doesn't apply to bare ESP32 + discrete components, so none of
  that was reused — only the control-flow pattern.

## Files, in bring-up order

1. **`sensor_test/`** — verify IMU + both encoders read sensibly *before* touching
   motor control. Also where you empirically determine the IMU axis mapping and
   sign (can't be known from simulation — depends on physical mounting).
2. **`static_inference_test/`** — verify the trained policy's C export runs
   correctly on-device, in isolation from sensors/motors. `actor.h` (generated
   via `export_actor_onnx()` in `rl/ppo_trainer.py` + `rl/onnx_actor_to_c.py`)
   is already in this folder and in `uniwheel_bot/`, wired for stage 05's
   7-input policy.
3. **`motor_spin_test/`** — isolate motor control from the policy: verify
   voltage-mode FOC spins the drive wheel cleanly before trusting it in the
   closed loop. See "Motor control" below.
4. **`uniwheel_bot/`** — the full closed loop.

## Libraries (Arduino IDE Library Manager)
- `Simple FOC` (askuric)
- `MPU6050_tockn`

## Motor control: voltage mode, confirmed good

`uniwheel_bot.ino` drives both motors with SimpleFOC's voltage-mode torque
control (`TorqueControlType::voltage` + `MotionControlType::torque` +
`motor.move(volts)`) — no current sensing. This was checked in isolation two
ways before trusting it in the full closed loop: `motor_spin_test.ino`
(angle-mode control, but same voltage-mode FOC underneath) and
`../reaction_wheel_unicycle/` (a full balance loop, same voltage-mode torque
control, proven-working third-party reference). Both confirmed the
driver/motor/pin setup itself spins cleanly — so that part of the config is
not in question.

What *was* wrong: `uniwheel_bot.ino`'s main loop used to busy-wait its entire
body — including `motor_wheel.loopFOC()`/`motor_reaction_wheel.loopFOC()`,
SimpleFOC's commutation update — down to the policy's control rate
(`TIMESTEP` = 5ms / 200Hz). That starves voltage-mode FOC, which needs to run
much faster than 200Hz to stay smooth, and is almost certainly what the
`MOTOR_DIR_REACTION_WHEEL` comment in the file describes as a "starting-
torque/cogging symptom" (that wheel needing a hand-nudge before it would spin
under motor power). `reaction_wheel_unicycle.ino` never throttles `loopFOC()`
at all — it runs every iteration as fast as the hardware allows, and only
gates the control-law update behind a loop counter. `uniwheel_bot.ino` now
does the same: both encoders' `.update()` + both motors' `.loopFOC()` run
unthrottled every iteration, and only the sensor-fusion/policy/new-target
block is gated to `TIMESTEP` via a non-blocking elapsed-time check. Worth
re-testing the reaction wheel's cold-start behavior now that FOC isn't being
throttled — this may well have fixed it outright.

## What's not done yet
- **End-to-end validation of the full closed loop** (balance, not just motor
  spin) hasn't happened on this hardware yet — `uniwheel_bot.ino` is believed
  ready to try, not yet confirmed holding balance for real.
- **`VOLTAGE_LIMIT = 6.0f`** is mcdgf's proven-working value for their 2804
  motor + SimpleFOC Mini + 12V supply (same motor family as our FIT1034),
  used as a conservative starting point — worth revisiting once you can tune
  it against real balance behavior (too low and the policy can't produce
  enough corrective torque; too high risks overcurrent on a stall).
- **Reaction wheel's own `initFOC()` status and direction sign are still only
  hand-spin-inferred**, not powered-confirmed (see the comment above
  `MOTOR_DIR_REACTION_WHEEL` in `uniwheel_bot.ino`) — re-test now that FOC
  runs unthrottled, since the previous inconclusive powered test may have
  been a symptom of the throttling itself rather than a real direction issue.
