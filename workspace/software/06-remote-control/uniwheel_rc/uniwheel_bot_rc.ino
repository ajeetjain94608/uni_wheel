/**
 * uniwheel_bot.ino (stage 03) + a WiFi remote control layer on top: drive the
 * commanded velocity (cmd_vel) from a phone/browser instead of the hardcoded 0.
 *
 * The WiFi AP / AsyncWebServer / WebSocket / dual-core command-sharing layer is
 * adapted from the reference repo's balance_bot_rc.ino - that part is fully
 * hardware-agnostic and reused close to as-is. One real difference: only a
 * {vel} command, not {vel, yaw} - this robot has no yaw actuator (same reasoning
 * as workspace/software/04-domain-randomization/uniwheel_env_dr.py's
 * yaw_penalty_coef=0.0), so there's no yaw command to send, and index_html.h's
 * UI is a single vertical slider, not a 2D thumbstick, to not offer a control
 * that silently does nothing.
 *
 * Hardware plumbing (I2C buses, SimpleFOC setup, pin scheme) is adapted from
 * macolin007/mcdgf (github.com/macolin007/mcdgf) - a two-wheel balance bot using
 * the identical hardware stack (ESP32 + SimpleFOC Mini x2 + AS5600 x2 + MPU6050).
 *
 * ============================================================================
 * BEFORE USING THIS FILE:
 *   1. Run sensor_test.ino (03-sim-to-real/) first and confirm wiring/readings
 *      look sane, and get uniwheel_bot.ino balancing with cmd_vel hardcoded to 0
 *      before adding remote control on top - don't debug both at once.
 *   2. Fill in the CALIBRATION CONSTANTS section below - the axis mapping, signs,
 *      and motor directions CANNOT be known from simulation alone; they depend on
 *      how your IMU is physically mounted and which way each motor spins for a
 *      positive command. Get these wrong and the robot will actively fight
 *      itself instead of balancing - verify each one by hand before trusting it.
 *   3. actor.h here is already copied from stage 05's trained policy (7 obs
 *      inputs, includes cmd_vel) - see workspace/software/05-command-conditioning/
 *      README.md. Verified against a real rollout: holds position within ~0.06-0.3m
 *      over 25s when cmd_vel=0, and drives in the commanded direction when nonzero -
 *      but noticeably undershoots the commanded *speed* (settles around 0.095 m/s for
 *      a cmd_vel=0.5 -> 0.25 m/s target, not just a slow ramp-up). Treat the slider as
 *      a direction control, not a precise speed dial, until that's improved.
 * ============================================================================
 *
 * Libraries needed (Arduino IDE Library Manager): Simple FOC (askuric),
 * MPU6050_tockn, ESP Async WebServer (me-no-dev/ESPAsyncWebServer),
 * AsyncTCP (me-no-dev/AsyncTCP), ArduinoJson
 */

#include <math.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

#include "actor.h"
#include "index_html.h"

// ============================================================================
// CALIBRATION CONSTANTS - fill these in from sensor_test.ino before trusting
// this sketch. Every one of these is a real unknown until measured on your
// actual hardware, not a default you can leave as-is.
// ============================================================================

// Which MPU6050 axis is pitch (rotates the ground wheel forward/back) and which
// is roll (rotates the reaction wheel side to side)? Tilt the robot each way by
// hand while running sensor_test.ino and see which of angleX/angleY moves.
// Set these to +1 or -1 depending on which axis maps to which AND which sign
// direction correctly reports "leaning forward = positive pitch" (matching the
// simulation's convention, since that's what the policy was trained on).
#define IMU_PITCH_FROM_X 1     // 1 if angleX/gyroX is pitch, 0 if it's angleY/gyroY instead
#define IMU_PITCH_SIGN 1.0f    // +1 or -1, verify by hand
#define IMU_ROLL_SIGN 1.0f     // +1 or -1, verify by hand

// Motor spin direction: does a positive voltage command spin the wheel the
// direction the simulation expects (matching the "wheel_joint"/"reaction_wheel_
// joint" axis convention in uniwheel.xml)? Flip to -1 if the robot visibly fights
// itself (overcorrects wildly) rather than converging when tipped gently by hand.
const int MOTOR_DIR_WHEEL = 1;
const int MOTOR_DIR_REACTION_WHEEL = 1;

// ============================================================================
// WiFi / remote control settings
// ============================================================================
static const char* AP_SSID     = "UniWheel";
static const char* AP_PASSWORD = "uniwheel1";  // WPA2 requires >= 8 chars
static const IPAddress AP_IP      (192, 168, 4, 1);
static const IPAddress AP_GATEWAY (192, 168, 4, 1);
static const IPAddress AP_SUBNET  (255, 255, 255, 0);

const float CMD_VEL_DEADZONE = 0.0f;  // ignore |cmd_vel| below this (tune once on hardware)
const float CMD_VEL_FACTOR = 1.0f;    // flip to -1 if "forward" on the slider drives backward

// ============================================================================
// Pin scheme (from macolin007/mcdgf's pin-table.xlsx - confirm against your own
// wiring, don't assume it matches without checking)
// ============================================================================
#define M0_I2C_SDA 21  // ground wheel AS5600 + MPU6050 (shared bus)
#define M0_I2C_SCL 22
#define M1_I2C_SDA 19  // reaction wheel AS5600 (separate bus - AS5600's I2C
#define M1_I2C_SCL 18  // address is fixed, can't share a bus with another AS5600)

// ============================================================================
// Settings (match training - see workspace/software/02-balance-bot-with-ppo/
// uniwheel_env.py and workspace/mechanical/urdf/uniwheel/uniwheel.xml)
// ============================================================================
const float COMP_ALPHA = 0.98f;      // complementary filter weight - MUST match uniwheel_env.py
const float TIMESTEP = 0.005f;       // sim timestep (sec) - MUST match uniwheel.xml <option timestep>
const float TIP_THRESHOLD = 0.524f;  // radians (30 deg) - MUST match uniwheel_env.py's tip_threshold_deg
const float VOLTAGE_LIMIT = 6.0f;    // volts - conservative default from the mcdgf reference build
const float MOTOR_PHASE_RESISTANCE = 2.3f;  // ohms - FIT1034 datasheet value (not mcdgf's 2804 motor's 2.9)
const int MOTOR_POLE_PAIRS = 7;      // FIT1034 datasheet value
const unsigned long RESET_TIME_MS = 1000;

// ============================================================================
// Hardware objects
// ============================================================================
TwoWire I2C_0 = TwoWire(0);
TwoWire I2C_1 = TwoWire(1);

MagneticSensorI2C sensor_wheel = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor_reaction_wheel = MagneticSensorI2C(AS5600_I2C);
MPU6050 mpu6050(I2C_0);

BLDCMotor motor_wheel = BLDCMotor(MOTOR_POLE_PAIRS);
// Phase order (A/C) corrected from (25,32,33,12) per hardware verification.
BLDCDriver3PWM driver_wheel = BLDCDriver3PWM(33, 32, 25, 12);

BLDCMotor motor_reaction_wheel = BLDCMotor(MOTOR_POLE_PAIRS);
// Phase order (A/C) corrected from (14,27,26,13) per hardware verification.
BLDCDriver3PWM driver_reaction_wheel = BLDCDriver3PWM(26, 27, 14, 13);

// ============================================================================
// State
// ============================================================================
float pitch = 0.0f;
float roll = 0.0f;
bool tipped = false;

// Shared command state: written by the Core 0 WebSocket callback, read by
// Core 1's control loop. Only cmd_vel - no cmd_yaw, this robot has no yaw
// actuator (see the file header).
static volatile float g_cmd_vel = 0.0f;
static portMUX_TYPE g_cmd_mux = portMUX_INITIALIZER_UNLOCKED;

inline void setCommand(float vel) {
  portENTER_CRITICAL(&g_cmd_mux);
  g_cmd_vel = vel;
  portEXIT_CRITICAL(&g_cmd_mux);
}

inline float getCommand() {
  portENTER_CRITICAL(&g_cmd_mux);
  float vel = g_cmd_vel;
  portEXIT_CRITICAL(&g_cmd_mux);
  return vel;
}

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void onWsEvent(AsyncWebSocket* server,
               AsyncWebSocketClient* client,
               AwsEventType type,
               void* arg,
               uint8_t* data,
               size_t len)
{
  if (type == WS_EVT_CONNECT) {
    Serial.printf("[WS] Client #%u connected\n", client->id());

  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] Client #%u disconnected - zeroing command\n", client->id());
    setCommand(0.0f);

  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len
        && info->opcode == WS_TEXT) {
      char buf[64];
      size_t copy_len = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
      memcpy(buf, data, copy_len);
      buf[copy_len] = '\0';

      StaticJsonDocument<64> doc;
      if (deserializeJson(doc, buf) == DeserializationError::Ok) {
        float vel = doc["vel"] | 0.0f;
        vel = constrain(vel, -1.0f, 1.0f);
        setCommand(vel);
      }
    }
  }
}

// Runs on Core 0: WiFi AP + web/WebSocket server. Core 1 (loop()) stays free
// to run the balance control loop at a fixed timestep, undisturbed by WiFi.
void serverTask(void* pvParameters) {
  Serial.println("[Core 0] Starting WiFi AP...");

  WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.printf("[Core 0] AP started: SSID=%s  IP=%s\n",
                AP_SSID, WiFi.softAPIP().toString().c_str());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });
  server.onNotFound([](AsyncWebServerRequest* request) {
    request->send(404, "text/plain", "Not found");
  });

  server.begin();
  Serial.println("[Core 0] HTTP + WebSocket server running");

  for (;;) {
    ws.cleanupClients();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void sensors_init() {
  I2C_0.begin(M0_I2C_SDA, M0_I2C_SCL, 100000);
  sensor_wheel.init(&I2C_0);

  I2C_1.begin(M1_I2C_SDA, M1_I2C_SCL, 100000);
  sensor_reaction_wheel.init(&I2C_1);

  mpu6050.begin();
  Serial.println("Calibrating gyro - keep the robot still...");
  mpu6050.calcGyroOffsets(true);
}

void motor_init(BLDCMotor& motor, BLDCDriver3PWM& driver, MagneticSensorI2C& sensor) {
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = 12;
  driver.voltage_limit = VOLTAGE_LIMIT;
  driver.init();
  motor.phase_resistance = MOTOR_PHASE_RESISTANCE;
  motor.torque_controller = TorqueControlType::voltage;
  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.voltage_limit = VOLTAGE_LIMIT;
  motor.useMonitoring(Serial);
  motor.init();
  motor.initFOC();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Uni wheel balance bot");

  sensors_init();
  motor_init(motor_wheel, driver_wheel, sensor_wheel);
  motor_init(motor_reaction_wheel, driver_reaction_wheel, sensor_reaction_wheel);

  // Start the WiFi/WebSocket server on Core 0 - leaves Core 1 (loop()) free to
  // hold the fixed-timestep control loop without WiFi stack jitter interfering.
  xTaskCreatePinnedToCore(
    serverTask, "serverTask", 8192, nullptr, 1, nullptr, 0  // Core 0
  );

  Serial.printf("Starting in %lu ms, place robot upright\n", RESET_TIME_MS);
  delay(RESET_TIME_MS);
  Serial.println("Running...");
}

void loop() {
  unsigned long step_start = micros();

  mpu6050.update();
  sensor_wheel.update();
  sensor_reaction_wheel.update();
  motor_wheel.loopFOC();
  motor_reaction_wheel.loopFOC();

  // Raw IMU: MPU6050_tockn reports accel in g's (ratio - no unit conversion
  // needed for atan2) and gyro in deg/s (DOES need conversion to rad/s to match
  // what the policy was trained on).
  float accX = mpu6050.getAccX();
  float accY = mpu6050.getAccY();
  float accZ = mpu6050.getAccZ();
  float pitch_rate = IMU_PITCH_SIGN * (IMU_PITCH_FROM_X ? mpu6050.getGyroX() : mpu6050.getGyroY())
                      * (M_PI / 180.0f);
  float roll_rate = IMU_ROLL_SIGN * (IMU_PITCH_FROM_X ? mpu6050.getGyroY() : mpu6050.getGyroX())
                      * (M_PI / 180.0f);

  // Same complementary filter as uniwheel_env.py's _get_obs() - keep these
  // formulas in sync if that ever changes, or the policy will see a
  // systematically different signal than it was trained on.
  float accel_pitch = IMU_PITCH_SIGN * -atan2f(accX, accZ);
  float accel_roll = IMU_ROLL_SIGN * atan2f(accY, accZ);
  pitch = COMP_ALPHA * (pitch + pitch_rate * TIMESTEP) + (1.0f - COMP_ALPHA) * accel_pitch;
  roll = COMP_ALPHA * (roll + roll_rate * TIMESTEP) + (1.0f - COMP_ALPHA) * accel_roll;

  float wheel_vel = MOTOR_DIR_WHEEL * sensor_wheel.getVelocity();
  float reaction_wheel_vel = MOTOR_DIR_REACTION_WHEEL * sensor_reaction_wheel.getVelocity();

  if (fabsf(pitch) > TIP_THRESHOLD || fabsf(roll) > TIP_THRESHOLD) {
    tipped = true;
  }

  if (!tipped) {
    // Read the latest command from the WebSocket handler (Core 0). Deadzone
    // and factor are for hardware tuning, not simulation behavior - the
    // trained policy itself scales this to vel_max=0.5 m/s.
    float cmd_vel = getCommand();
    cmd_vel = fabsf(cmd_vel) < CMD_VEL_DEADZONE ? 0.0f : cmd_vel;
    cmd_vel = constrain(cmd_vel * CMD_VEL_FACTOR, -1.0f, 1.0f);

    // Observation order MUST match uniwheel_env_cmd.py's _get_obs() exactly:
    // [pitch, roll, pitch_rate, roll_rate, wheel_vel, reaction_wheel_vel, cmd_vel]
    float obs[ACTOR_OBS_SIZE] = {pitch, roll, pitch_rate, roll_rate, wheel_vel,
                                  reaction_wheel_vel, cmd_vel};
    float action[ACTOR_ACTION_SIZE];
    actor_forward(obs, action);

    action[0] = constrain(action[0], -1.0f, 1.0f);
    action[1] = constrain(action[1], -1.0f, 1.0f);

    // Voltage-mode torque control: target is a voltage, not a literal torque or
    // angle. Scaling by VOLTAGE_LIMIT maps the policy's [-1,1] action directly
    // onto the driver's configured voltage ceiling.
    motor_wheel.move(MOTOR_DIR_WHEEL * action[0] * VOLTAGE_LIMIT);
    motor_reaction_wheel.move(MOTOR_DIR_REACTION_WHEEL * action[1] * VOLTAGE_LIMIT);
  } else {
    motor_wheel.move(0);
    motor_reaction_wheel.move(0);

    // Wait for someone to set it upright again before resuming
    if (fabsf(pitch) < 0.1f && fabsf(roll) < 0.1f) {
      tipped = false;
      Serial.printf("Untipped! Starting in %lu ms\n", RESET_TIME_MS);
      delay(RESET_TIME_MS);
      Serial.println("Running...");
    }
  }

  // Pace to TIMESTEP (matches the sim's control rate - the policy was trained
  // expecting this timing, running faster or slower changes its effective gains)
  while (micros() - step_start < (unsigned long)(TIMESTEP * 1e6f));
}
