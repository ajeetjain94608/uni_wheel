/**
 * One-shot helper: print raw (pre-offset) AcX/AcY/AcZ from the MPU6050, plus
 * a running average, so AcX_offset/AcY_offset/AcZ_offset in
 * reaction_wheel_unicycle.ino can be re-zeroed for OUR IMU chip/mounting
 * instead of the upstream repo's placeholder values.
 *
 * No motors involved - just raw I2C register reads, same registers/config as
 * reaction_wheel_unicycle.ino's angle_setup()/angle_calc(), reused verbatim.
 *
 * Usage: sit the robot level/upright and still, flash this, watch the
 * running average settle (a few seconds), then read off the reported offsets
 * to paste into reaction_wheel_unicycle.ino.
 */

#include <Wire.h>

#define M0_I2C_SDA 21  // same bus as MPU6050 in reaction_wheel_unicycle.ino (ESP32 default pins)
#define M0_I2C_SCL 22

TwoWire I2C_0 = TwoWire(0);

#define MPU6050_ADDR 0x68
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define ACCEL_XOUT_H 0x3B
#define PWR_MGMT_1 0x6B
#define accSens 0   // matches reaction_wheel_unicycle.ino - 0 = +/-2g, 16384 LSB/g
#define gyroSens 1  // matches reaction_wheel_unicycle.ino

int16_t AcX, AcY, AcZ;
double AcX_sum = 0, AcY_sum = 0, AcZ_sum = 0;
long n = 0;

void writeTo(byte device, byte address, byte value) {
  I2C_0.beginTransmission(device);
  I2C_0.write(address);
  I2C_0.write(value);
  I2C_0.endTransmission(true);
}

void read_accel() {
  I2C_0.beginTransmission(MPU6050_ADDR);
  I2C_0.write(ACCEL_XOUT_H);
  I2C_0.endTransmission(false);
  I2C_0.requestFrom(MPU6050_ADDR, 6, true);
  AcX = I2C_0.read() << 8 | I2C_0.read();
  AcY = I2C_0.read() << 8 | I2C_0.read();
  AcZ = I2C_0.read() << 8 | I2C_0.read();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(500);

  I2C_0.begin(M0_I2C_SDA, M0_I2C_SCL, 100000);
  delay(100);
  writeTo(MPU6050_ADDR, PWR_MGMT_1, 0);
  writeTo(MPU6050_ADDR, ACCEL_CONFIG, accSens << 3);
  writeTo(MPU6050_ADDR, GYRO_CONFIG, gyroSens << 3);
  delay(100);

  Serial.println("\n=== raw accel calibration ===");
  Serial.println("Sit the robot level/upright and still. Watch the AVG line settle.");
  Serial.println("At rest, accZ should read ~+16384 (1g), accX/accY should read ~0.");
  Serial.println("Offsets needed = target - avg (accZ target=+16384, accX/accY target=0).\n");
}

void loop() {
  read_accel();
  n++;
  AcX_sum += AcX;
  AcY_sum += AcY;
  AcZ_sum += AcZ;

  static unsigned long last_print = 0;
  if (millis() - last_print >= 200) {
    last_print = millis();
    double avgX = AcX_sum / n, avgY = AcY_sum / n, avgZ = AcZ_sum / n;
    Serial.printf("raw: AcX=%d AcY=%d AcZ=%d  |  AVG(n=%ld): AcX=%.1f AcY=%.1f AcZ=%.1f  |  "
                  "suggested offsets: AcX_offset=%.0f AcY_offset=%.0f AcZ_offset=%.0f\n",
                  AcX, AcY, AcZ, n, avgX, avgY, avgZ,
                  -avgX, -avgY, 16384.0 - avgZ);
  }
  delay(5);
}
