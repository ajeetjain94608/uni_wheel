/**
 * Static test for the actor network forward pass. Verifies the C header exported
 * by rl/onnx_actor_to_c.py runs correctly on the ESP32 and check inference speed,
 * before it's ever wired to real sensors/motors.
 *
 * Adapted near-verbatim from the reference repo's version - this part is fully
 * hardware-agnostic, it just calls actor_forward(). test_obs's size changed twice:
 * 4 -> 6 (added roll/roll_rate/reaction_wheel_vel, stage 02/04), then 6 -> 7 (added
 * cmd_vel, stage 05 - this is the version currently in actor.h, since stage 05's
 * policy supersedes stage 04's: it balances *and* holds position *and* drives on
 * command, all as one policy - see workspace/software/05-command-conditioning/README.md).
 *
 * Setup:
 *   1. Once training finishes (workspace/software/02-balance-bot-with-ppo/), export
 *      the best checkpoint: export_actor_onnx() in rl/ppo_trainer.py -> .onnx, then
 *      `python3 rl/onnx_actor_to_c.py <path>.onnx actor.h`
 *   2. Copy the generated actor.h into this sketch folder
 *   3. Upload to ESP32, open Serial Monitor at 115200 baud
 *
 * Sanity check once it prints: action values should be small and finite (roughly
 * in [-1, 1] before any clamping) for this near-zero test input - NaN or huge
 * values mean something went wrong in the ONNX->C conversion, not the sketch.
 */

#include <math.h>

#include "actor.h"

// [pitch, roll, pitch_rate, roll_rate, wheel_vel, reaction_wheel_vel, cmd_vel]
const float test_obs[ACTOR_OBS_SIZE] = {0.2, 0.1, -0.5, 0.3, 1.0, -1.0, 0.5};

void setup() {
  unsigned long timestamp;
  float action[ACTOR_ACTION_SIZE];

  Serial.begin(115200);
  while (!Serial);
  Serial.println("Static inference test");
  Serial.printf("ACTOR_OBS_SIZE=%d ACTOR_ACTION_SIZE=%d\n", ACTOR_OBS_SIZE, ACTOR_ACTION_SIZE);

  timestamp = micros();
  actor_forward(test_obs, action);

  Serial.print("Inference time (us): ");
  Serial.println(micros() - timestamp);
  Serial.print("Action: [");
  for (int i = 0; i < ACTOR_ACTION_SIZE; i++) {
    Serial.print(action[i], 4);
    if (i < ACTOR_ACTION_SIZE - 1) {
      Serial.print(", ");
    }
  }
  Serial.println("]");
}

void loop() {
  // Do nothing
}
