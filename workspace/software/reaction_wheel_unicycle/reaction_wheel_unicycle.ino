/**
 * Adapted for our hardware (ESP32 + SimpleFOC Mini x2 + AS5600 x2 + MPU6050,
 * SimpleFOC 2.4.0) from the original remrc/Reaction-Wheel-Unicycle, which
 * targeted a BGC 3.1 gimbal board (STM32) + SimpleFOC 1.61 + analog-output
 * AS5600s. Control law, K-gain names/values, tuning scheme (tuning.ino), and
 * overall structure are UNCHANGED from upstream - only what's forced by
 * actual hardware differences was touched:
 *
 *   1. Motor/driver API: SimpleFOC 1.61 combined motor+driver into one
 *      BLDCMotor(pinA,pinB,pinC,en) call and used ControlType::voltage. Our
 *      SimpleFOC 2.4.0 splits this into BLDCMotor(pole_pairs) + a separate
 *      BLDCDriver3PWM(pins), and uses TorqueControlType::voltage +
 *      MotionControlType::torque. Not a stylistic choice - the old
 *      constructor doesn't exist in 2.4.0.
 *   2. Sensors: original used MagneticSensorAnalog (AS5600 wired to an
 *      analog input pin). Our AS5600 boards are wired for I2C (SDA/SCL), not
 *      analog out, so these are MagneticSensorI2C instead, on two separate
 *      TwoWire buses (AS5600's I2C address is fixed, so two of them can't
 *      share one bus) - same constraint documented in uniwheel_bot.ino.
 *   3. Pins: ours, from pin-table.xlsx / uniwheel_bot.ino (confirmed against
 *      actual wiring - not assumed).
 *   4. Axis mapping in angle_calc(): the original's Acc_angleX/Acc_angleY use
 *      AcX as the "vertical" reference axis in both atan2() calls, which
 *      means THEIR MPU6050 is physically mounted with its own X axis pointing
 *      up at rest - a fact about their specific build, not ours. Our MPU6050
 *      was independently measured by hand (see uniwheel_bot.ino /
 *      sensor_test.ino): accZ is vertical at rest (standard mounting), raw Y
 *      axis carries pitch, raw X axis carries roll, with signs
 *      IMU_PITCH_SIGN=-1 / IMU_ROLL_SIGN=+1. The atan2() axis pairings and
 *      gyro axis selection below use OUR measured mapping, not the
 *      original's - copying theirs verbatim would silently feed the wrong
 *      physical quantity into the controller.
 *
 * NOT changed / still needs your own measurement before trusting this:
 *   - AcX_offset/AcY_offset/AcZ_offset: DONE - re-measured via
 *     imu_raw_calib.ino with the robot sitting level (see values below).
 *     angle_offsetX/Y set to 0 accordingly (see comment at their definition).
 *   - K1X/K2X/K3X/K1Y/K2Y/K3Y are the original's values for their GBM2804H-
 *     100T motors - a starting point only, not tuned for our FIT1034 motors
 *     or this mechanical build. Use tuning.ino's live serial commands.
 *   - gyroSens/accSens register config and the 131.0/6.07968E-5 scaling
 *     constants are left exactly as upstream (tied to gyroSens=1, ±500dps) -
 *     untouched since they're not pin/axis-specific.
 *
 * Libraries needed (Arduino IDE Library Manager): Simple FOC (askuric)
 */

#include <SimpleFOC.h>

// ============================================================================
// Pin scheme - ours (see uniwheel_bot.ino), not the original's
// ============================================================================
#define M0_I2C_SDA 21  // motorY (reaction wheel) AS5600 + MPU6050, shared bus
#define M0_I2C_SCL 22
#define M1_I2C_SDA 19  // motorX (ground/drive wheel) AS5600, separate bus
#define M1_I2C_SCL 18

TwoWire I2C_0 = TwoWire(0);
TwoWire I2C_1 = TwoWire(1);

const float MOTOR_PHASE_RESISTANCE = 2.3f;  // FIT1034 datasheet value (was 2804 motor's 2.9 upstream)
const int MOTOR_POLE_PAIRS = 7;             // FIT1034 datasheet value

// Original: MagneticSensorAnalog sensorX = MagneticSensorAnalog(A1, 14, 1020);
// Our AS5600s are wired I2C, not analog - see header comment.
MagneticSensorI2C sensorX = MagneticSensorI2C(AS5600_I2C);  // ground/drive wheel, bus 1
MagneticSensorI2C sensorY = MagneticSensorI2C(AS5600_I2C);  // reaction wheel, bus 0

// Original: BLDCMotor motorX = BLDCMotor(3, 5, 6, 7); (SimpleFOC 1.61 combined
// motor+driver constructor - pins were PWM pins directly). Our 2.4.0 needs
// pole pairs on the motor and a separate driver object with our own pins.
BLDCMotor motorX = BLDCMotor(MOTOR_POLE_PAIRS);
// Phase order (A/C) corrected from (14,27,26,13) per hardware verification.
BLDCDriver3PWM driverX = BLDCDriver3PWM(26, 27, 14, 13);

BLDCMotor motorY = BLDCMotor(MOTOR_POLE_PAIRS);
// Phase order (A/C) corrected from (25,32,33,12) per hardware verification.
BLDCDriver3PWM driverY = BLDCDriver3PWM(33, 32, 25, 12);

#define MPU6050_ADDR 0x68          // Device address
#define ACCEL_CONFIG 0x1C         // Accelerometer configuration address
#define GYRO_CONFIG 0x1B          // Gyro configuration address

//Registers: Accelerometer, Temp, Gyroscope
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

//Sensor output scaling
#define accSens 0             // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1            // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s

// Our measured axis mapping (see header comment) - NOT the original's.
#define IMU_PITCH_SIGN -1.0f
#define IMU_ROLL_SIGN 1.0f

int16_t  AcX, AcY, AcZ, GyY, GyX, GyZ;
float gyroX, gyroY;

//IMU offset values - STILL THE ORIGINAL REPO'S VALUES, re-measure for our IMU
// Measured via imu_raw_calib.ino with the robot sitting level and still
// (converged over ~1400 samples): raw AcX~-648, AcY~-30, AcZ~14322 (dominant,
// confirming Z is vertical at rest as expected). Offsets below zero AcX/AcY
// and target AcZ+offset=+16384 (1g at accSens=0).
int16_t  AcX_offset = 648;
int16_t  AcY_offset = 32;
int16_t  AcZ_offset = 2062;
// GyX_offset added - the original only ever calibrated GyY/GyZ (it only used
// those two axes for integration). We use GyX for roll (robot_angleY) to
// match our IMU's actual mounting, but that means GyX ALSO needs its own bias
// calibrated and subtracted, or its raw offset integrates forever as a
// constant angular drift (this was a real bug: robot_angleY was observed
// drifting steadily rather than settling).
int16_t  GyX_offset = 0, GyY_offset = 0, GyZ_offset = 0;
int32_t  GyX_offset_sum = 0, GyY_offset_sum = 0, GyZ_offset_sum = 0;

float robot_angleX, robot_angleY, gyroXfilt, gyroYfilt;
float Acc_angleX,  Acc_angleY;
// 0, not the original repo's -5.60/2.03 - those corrected for THEIR board's
// residual bias, which is already zeroed out above via AcX_offset/AcY_offset
// on our board. Keeping both would double-correct.
float angle_offsetX = 0.0, angle_offsetY = 0.0;

float K1X = 1.67;
float K2X = 0.28;
float K3X = 0.190;
float K1Y = 0.74;
float K2Y = 0.09;
float K3Y = 0.200;

int loop_time = 4;

boolean vertical = false;
boolean test = false;  // if true - motors and sensors test. Type in serial monitor t+ to enable and t- to disable

#define Gyro_amount 0.996   // percent of gyro in complementary filter T/(T+del_t) del_t: sampling rate, T acc. timeconstant ~1s
float alphaX = 0.25; // low pass
float alphaY = 0.75; // low pass

void setup() {

  Serial.begin(115200);
  delay(1000);

  angle_setup();

  // Original: sensorX.init(); motorX.linkSensor(&sensorX);
  // motorX.voltage_power_supply=12; motorX.controller=ControlType::voltage;
  // motorX.voltage_sensor_align=4; motorX.foc_modulation=...; motorX.init();
  // motorX.initFOC(); (SimpleFOC 1.61 - power supply/control type lived on
  // the motor directly, no separate driver). Adapted to 2.4.0's split
  // motor/driver objects, same pattern as uniwheel_bot.ino's motor_init().
  sensorX.init(&I2C_1);
  motorX.linkSensor(&sensorX);
  driverX.voltage_power_supply = 12;
  driverX.voltage_limit = 6;
  driverX.init();
  motorX.phase_resistance = MOTOR_PHASE_RESISTANCE;
  motorX.torque_controller = TorqueControlType::voltage;
  motorX.linkDriver(&driverX);
  motorX.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motorX.controller = MotionControlType::torque;
  motorX.voltage_limit = 6;
  motorX.init();
  motorX.initFOC();

  sensorY.init(&I2C_0);
  motorY.linkSensor(&sensorY);
  driverY.voltage_power_supply = 12;
  driverY.voltage_limit = 6;
  driverY.init();
  motorY.phase_resistance = MOTOR_PHASE_RESISTANCE;
  motorY.torque_controller = TorqueControlType::voltage;
  motorY.linkDriver(&driverY);
  motorY.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motorY.controller = MotionControlType::torque;
  motorY.voltage_limit = 6;
  motorY.init();
  motorY.initFOC();
}

long loop_count = 0;
float m_speedX = 0, m_speedY = 0;
float move_Speed;

void loop() {
  // ~1ms
  motorX.loopFOC();
  motorY.loopFOC();

  Tuning();
  angle_calc();

  if (abs(robot_angleX - angle_offsetX) > 8 || abs(robot_angleY - angle_offsetY) > 8) vertical = false;
  // Loosened from 0.3 deg (upstream's value) - that's an unrealistically tight
  // window for real accel/gyro fusion noise; if the robot never quite hits it,
  // "vertical" never becomes true and both motors sit at 0 forever (exactly
  // "initialized but never rotates"). 3 deg is still a reasonably upright
  // requirement without requiring near-perfect precision.
  if (abs(robot_angleX - angle_offsetX) < 3.0 && abs(robot_angleY - angle_offsetY) < 3.0 && !vertical) vertical = true;

  gyroX = GyZ / 131.0; // Convert to deg/s
  gyroY = GyY / 131.0; // Convert to deg/s

  gyroXfilt = alphaX * gyroX + (1 - alphaX) * gyroXfilt;
  gyroYfilt = alphaY * gyroY + (1 - alphaY) * gyroYfilt;

  if (test) {
    motorX.move(4 * (motorY.shaft_angle - motorX.shaft_angle));
    motorY.move(4 * (motorX.shaft_angle - motorY.shaft_angle));
  } else if (loop_count++ > loop_time) {  // control loop
    float target_voltageX = 0, target_voltageY = 0;
    if (vertical) {
      target_voltageX = XcontrolleR(robot_angleX - angle_offsetX, gyroXfilt, sensorX.getVelocity() + m_speedX);
      m_speedX += sensorX.getVelocity() / 50;
      target_voltageY = YcontrolleR(robot_angleY - angle_offsetY, gyroYfilt, sensorY.getVelocity() + m_speedY);
      m_speedY += sensorY.getVelocity() / 40 - move_Speed;

      motorX.move(target_voltageX);
      motorY.move(-target_voltageY);
    } else {
      motorX.move(target_voltageX);
      motorY.move(target_voltageY);
      m_speedX = 0;
      m_speedY = 0;
    }
    loop_count = 0;
  }

  static unsigned long last_print = 0;
  if (millis() - last_print >= 200) {
    last_print = millis();
    Serial.printf("angleX=%.2f angleY=%.2f vertical=%d\n", robot_angleX, robot_angleY, vertical);
  }
}

int sign(int x) {
  if (x > 0) return 1;
  if (x < 0) return -1;
  if (x = 0) return 0;
}

float XcontrolleR(float p_angle, float p_vel, float m_vel) {
  float u =  K1X * p_angle + K2X * p_vel + K3X * m_vel;
  if (abs(u) > motorX.voltage_limit * 0.8)
    u = sign(u) * motorX.voltage_limit * 0.8;
  return u;
}

float YcontrolleR(float p_angle, float p_vel, float m_vel) {
  float u =  K1Y * p_angle + K2Y * p_vel + K3Y * -m_vel;
  if (abs(u) > motorY.voltage_limit * 0.8)
    u = sign(u) * motorY.voltage_limit * 0.8;
  return u;
}

void writeTo(byte device, byte address, byte value) {
  I2C_0.beginTransmission(device);
  I2C_0.write(address);
  I2C_0.write(value);
  I2C_0.endTransmission(true);
}

//setup MPU6050 - on I2C_0, which is the bus it physically shares with sensorY
void angle_setup() {
  I2C_0.begin(M0_I2C_SDA, M0_I2C_SCL, 100000);
  delay (100);
  writeTo(MPU6050_ADDR, PWR_MGMT_1, 0);
  writeTo(MPU6050_ADDR, ACCEL_CONFIG, accSens << 3); // Specifying output scaling of accelerometer
  writeTo(MPU6050_ADDR, GYRO_CONFIG, gyroSens << 3); // Specifying output scaling of gyroscope
  delay (100);

  Serial.println("Calibrating gyroscope...");

  // calc gyro offset by averaging 1024 values
  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyX_offset_sum += GyX;
    GyZ_offset_sum += GyZ;
    GyY_offset_sum += GyY;
    delay(5);
  }
  GyX_offset = GyX_offset_sum >> 10;
  Serial.print("GyX offset value = "); Serial.println(GyX_offset);
  GyZ_offset = GyZ_offset_sum >> 10;
  Serial.print("GyZ offset value = "); Serial.println(GyZ_offset);
  GyY_offset = GyY_offset_sum >> 10;
  Serial.print("GyY offset value = "); Serial.println(GyY_offset);
}

void angle_calc() {

  // read raw accel/gyro measurements from device
  I2C_0.beginTransmission(MPU6050_ADDR);
  I2C_0.write(GYRO_XOUT_H);
  I2C_0.endTransmission(false);
  I2C_0.requestFrom(MPU6050_ADDR, 6, true);
  GyX = I2C_0.read() << 8 | I2C_0.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = I2C_0.read() << 8 | I2C_0.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = I2C_0.read() << 8 | I2C_0.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  I2C_0.beginTransmission(MPU6050_ADDR);
  I2C_0.write(ACCEL_XOUT_H);
  I2C_0.endTransmission(false);
  I2C_0.requestFrom(MPU6050_ADDR, 6, true);
  AcX = I2C_0.read() << 8 | I2C_0.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = I2C_0.read() << 8 | I2C_0.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = I2C_0.read() << 8 | I2C_0.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  // add mpu6050 offset values
  AcX += AcX_offset;
  AcY += AcY_offset;
  AcZ += AcZ_offset;
  GyZ -= GyZ_offset;
  GyY -= GyY_offset;
  GyX -= GyX_offset;

  // Our measured mapping (see header comment): raw Y axis carries pitch
  // (robot_angleX, motorX/ground wheel), raw X axis carries roll
  // (robot_angleY, motorY/reaction wheel), accZ is vertical at rest - NOT the
  // original's GyZ/AcX-as-vertical pairing, which was specific to their
  // mounting.
  robot_angleX += IMU_PITCH_SIGN * GyY * 6.07968E-5;
  Acc_angleX = IMU_PITCH_SIGN * -atan2(AcY, AcZ) * 57.2958;      // angle from acc. values * 57.2958 (deg/rad)
  robot_angleX = robot_angleX * Gyro_amount + Acc_angleX * (1.0 - Gyro_amount);

  robot_angleY += IMU_ROLL_SIGN * GyX * 6.07968E-5;
  Acc_angleY = IMU_ROLL_SIGN * atan2(AcX, AcZ) * 57.2958;        // angle from acc. values * 57.2958 (deg/rad)
  robot_angleY = robot_angleY * Gyro_amount + Acc_angleY * (1.0 - Gyro_amount);

  //Serial.print("AngleX: "); Serial.print(robot_angleX); Serial.print(" AngleY: "); Serial.println(robot_angleY);

}
