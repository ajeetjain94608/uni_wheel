/**
 * Sensor bring-up test: verify the IMU and both AS5600 encoders read sensibly
 * BEFORE wiring in motor control or the trained policy. Run this first.
 *
 * Hardware / pin scheme taken from macolin007/mcdgf (github.com/macolin007/mcdgf) -
 * a two-wheel self-balancing bot using the exact same hardware stack (ESP32,
 * SimpleFOC Mini x2, AS5600 x2, MPU6050). Confirm your actual wiring matches this
 * before trusting the readings:
 *
 *   AS5600 #0 (reaction wheel) + MPU6050, shared I2C bus 0:  SDA=21  SCL=22
 *   AS5600 #1 (drive wheel), separate I2C bus 1:              SDA=19  SCL=18
 *
 * (Matches the macolin007/mcdgf reference's bus-0 pins. An earlier version of
 * this file had bus 0 swapped to SDA=22/SCL=21 based on a wiring report that
 * turned out to be for a since-changed physical connection - re-verify
 * against your own build rather than trusting either value blindly.)
 *
 * (Motor driver pins aren't used in this sketch - see uniwheel_bot/ for those.)
 *
 * Libraries needed (Arduino IDE Library Manager):
 *   - Simple FOC (askuric)
 *   - MPU6050_tockn
 *
 * Expected behavior / things to actually verify, not assume:
 *   - With the robot sitting still, angleX/angleY should read near 0 and settle
 *     (not drift) after the gyro calibration at boot.
 *   - IMPORTANT - figure out the axis mapping empirically, don't guess it from
 *     this code: tilt the robot nose-up/down about the drive wheel's axle -
 *     whichever of angleX/angleY moves is "pitch". Then lean it sideways about
 *     the reaction wheel's axis - whichever moves (should be the other one) is
 *     "roll". This depends on how the MPU6050 board is physically mounted and
 *     can't be known from this code alone - write down which is which before
 *     writing uniwheel_bot.ino, and check the sign (does leaning one way give a
 *     positive or negative angle?) since sign matters as much as which axis.
 *   - Spinning either wheel by hand should show its encoder angle changing
 *     smoothly (no jumps, no stuck-at-zero) and velocity tracking direction
 *     correctly (spin one way -> positive, other way -> negative).
 */

#include <MPU6050_tockn.h>
#include <Wire.h>
#include <SimpleFOC.h>

// Pin definitions (see header comment)
#define M0_I2C_SDA 21  // reaction wheel AS5600 + MPU6050 (shared bus)
#define M0_I2C_SCL 22
#define M1_I2C_SDA 19  // drive wheel AS5600 (separate bus)
#define M1_I2C_SCL 18

TwoWire I2C_0 = TwoWire(0);
TwoWire I2C_1 = TwoWire(1);

MagneticSensorI2C sensor_reaction_wheel = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor_drive_wheel = MagneticSensorI2C(AS5600_I2C);
MPU6050 mpu6050(I2C_0);

unsigned long last_print = 0;
const unsigned long PRINT_INTERVAL_MS = 200;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("\n=== Sensor test ===");

  I2C_0.begin(M0_I2C_SDA, M0_I2C_SCL, 100000);
  sensor_reaction_wheel.init(&I2C_0);

  I2C_1.begin(M1_I2C_SDA, M1_I2C_SCL, 100000);
  sensor_drive_wheel.init(&I2C_1);

  mpu6050.begin();
  Serial.println("Calibrating gyro - keep the robot still...");
  mpu6050.calcGyroOffsets(true);  // prints progress, takes a few seconds
  Serial.println("Ready.\n");
}

void loop() {
  mpu6050.update();
  sensor_reaction_wheel.update();
  sensor_drive_wheel.update();

  if (millis() - last_print >= PRINT_INTERVAL_MS) {
    last_print = millis();
    // Printed as angleX/angleY, not pitch/roll - see the header comment: which
    // is which, and their sign, needs to be verified by hand before uniwheel_bot.ino
    Serial.printf(
      "IMU: angleX=%.2f angleY=%.2f gyroX=%.2f gyroY=%.2f gyroZ=%.2f  |  "
      "reaction_wheel: angle=%.3f vel=%.2f  |  drive_wheel: angle=%.3f vel=%.2f\n",
      mpu6050.getAngleX(), mpu6050.getAngleY(),
      mpu6050.getGyroX(), mpu6050.getGyroY(), mpu6050.getGyroZ(),
      sensor_reaction_wheel.getAngle(), sensor_reaction_wheel.getVelocity(),
      sensor_drive_wheel.getAngle(), sensor_drive_wheel.getVelocity()
    );
  }
}
