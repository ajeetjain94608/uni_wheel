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
   correctly on-device, in isolation from sensors/motors. Needs `actor.h`, which
   doesn't exist yet — see below.
3. **`uniwheel_bot/`** — the full closed loop.

## Libraries (Arduino IDE Library Manager)
- `Simple FOC` (askuric)
- `MPU6050_tockn`

## What's not done yet
- **`actor.h` doesn't exist.** Once training (stage 02) reaches a checkpoint you're
  happy with: export it with `export_actor_onnx()` in `rl/ppo_trainer.py`, then
  `python3 rl/onnx_actor_to_c.py <path>.onnx actor.h`, then copy the result into
  `static_inference_test/` and `uniwheel_bot/`.
- **Calibration constants in `uniwheel_bot.ino` are unfilled placeholders**, marked
  clearly in the file — IMU axis/sign mapping and motor spin direction. These are
  real unknowns until measured on your actual hardware via `sensor_test.ino`, not
  defaults safe to leave as-is. Getting them wrong means the robot actively fights
  itself rather than balancing, not just "works a bit worse."
- **Pin scheme is mcdgf's, not yet confirmed against your actual wiring.** Check it
  matches before flashing.
- **`VOLTAGE_LIMIT = 6.0f`** is mcdgf's proven-working value for their 2804 motor +
  SimpleFOC Mini + 12V supply (same motor family as our FIT1034), used as a
  conservative starting point since there's no hardware here to tune it against —
  worth revisiting once you can actually test it.
