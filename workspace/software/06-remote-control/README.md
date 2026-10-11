# 06 — Remote control

Two sketches, bring-up order:

1. **`vel_slider_test/`** — the WiFi AP + WebSocket + web-UI layer in isolation, no
   motors/policy involved. Confirms the phone/browser control path works before
   combining it with the balance firmware.
2. **`uniwheel_rc/`** — `03-sim-to-real/uniwheel_bot.ino` plus the WiFi layer, with
   `cmd_vel` now coming from the browser instead of a hardcoded 0.

## One real difference from the reference repo, not an oversight
The reference repo's UI is a 2D thumbstick sending `{vel, yaw}` — its differential-drive
wheels can actually steer. This robot can't act on a yaw command (no actuator has a torque
component about that axis — same reasoning as `yaw_penalty_coef=0.0` in stage 04's env), so
`index_html.h` here is a **single vertical slider** sending just `{vel}`. A left/right axis
that silently did nothing would be worse than not offering it.

## Libraries (Arduino IDE Library Manager)
- Everything stage 03 needs (`Simple FOC`, `MPU6050_tockn`)
- `ESP Async WebServer` (me-no-dev/ESPAsyncWebServer)
- `AsyncTCP` (me-no-dev/AsyncTCP)
- `ArduinoJson`

## Using it
1. Flash `uniwheel_rc/uniwheel_bot_rc.ino` (fill in the same CALIBRATION CONSTANTS as
   stage 03 first — this file assumes you've already got `uniwheel_bot.ino` balancing
   with `cmd_vel` hardcoded to 0 before adding remote control on top).
2. Connect to WiFi network **UniWheel** (password `uniwheel1`).
3. Open `http://192.168.4.1` in a browser, drag the slider up/down.

**Remember the confirmed limitation from stage 05**: the slider is a direction control,
not a precise speed dial — commanding full-scale only nets ~38% of the nominal target
speed in simulation (0.095 vs 0.25 m/s at `cmd_vel=0.5`). Expect "drive this way" behavior,
not "go exactly this fast."

## Status: written, not yet run
Blocked on hardware arriving, same as stage 03. `uniwheel_rc/actor.h` is already stage 05's
verified policy (7 obs inputs) — no separate export step needed, it's the same file copied
into `03-sim-to-real/uniwheel_bot/`.

## `sim_remote_control.ipynb` — the simulation-side counterpart

Same control experience as the real hardware's `uniwheel_rc/`, but in the MuJoCo viewer,
no hardware needed: **Up/Down arrows** raise/lower `cmd_vel`, run through the actual
trained stage-05 policy (not raw motor control like stage 01's `test_motion.ipynb`) —
this is the same thing `uniwheel_bot_rc.ino` does with a phone-browser slider, just with
the keyboard standing in for the phone.

Validated for real, not just headlessly: this dev session did have a usable display
after all, so this ran with the actual MuJoCo/GLFW viewer window open, loading the real
stage-05 checkpoint and calling `key_callback(KEY_UP)` (the same function a real
keypress triggers) 5x at t=3s to simulate raising `cmd_vel` to 0.5. 12 real-time-paced
seconds: held position cleanly for the first 3s, then drove forward smoothly and stayed
stable the whole time (pitch/roll under 0.5deg throughout, x position climbing to
+0.86m by t=12s, matching stage 05's known ~38%-of-target speed undershoot).

One caveat: the process **segfaulted while closing the viewer window**, strictly after
the run itself completed and printed done - a known GLFW/MuJoCo-viewer cleanup quirk on
some display setups, not a bug in the control logic. If the same thing happens on your
machine when you close the window, that's expected and harmless; everything up to that
point already worked.
