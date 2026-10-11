/**
*
* Position/angle motion control example
* Steps:
* 1) Configure the motor and magnetic sensor
* 2) Run the code
* 3) Set the target angle (in radians) from serial terminal
*
*/
#include <SimpleFOC.h> // Be sure to load this library

// Driver pins (26,27,14,13) are the DRIVE WHEEL motor (see uniwheel_bot.ino) -
// its encoder is on bus 1 (SDA=19, SCL=18), NOT the ESP32's default I2C pins
// (21/22, which is physically the REACTION wheel's encoder). sensor.init()
// with no argument defaults to the global Wire object on the default pins -
// that would silently commutate this motor using a different motor's shaft
// position entirely. Explicit bus below fixes that.
#define M1_I2C_SDA 19
#define M1_I2C_SCL 18
TwoWire I2C_1 = TwoWire(1);

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7); // The number of pole pairs of 2804 and 2208 motors is 7, and the number of pole pairs of 4015 motors is 11
BLDCDriver3PWM driver = BLDCDriver3PWM(26, 27, 14, 13); // drive wheel, phase order verified

float target_angle = 0; //Define the angle variable, which can also be torque or speed.
//Instantiate the commander
Commander command = Commander(Serial); //Send command via the serial port
void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }

void setup() { //Initialization

// Initialize magnetic sensor hardware, on the drive wheel's actual bus
I2C_1.begin(M1_I2C_SDA, M1_I2C_SCL, 100000);
sensor.init(&I2C_1);
// Link the motor to the sensor
motor.linkSensor(&sensor); //Motor to sensor connection

// Driver configuration
// Power supply voltage [V]
driver.voltage_power_supply = 12; //Power supply voltage connected to the driver
driver.init(); //Driver initialization
// Link the motor and the driver
motor.linkDriver(&driver); // Connect the motor to the driver

// Choose FOC modulation (optional)
motor.foc_modulation = FOCModulationType::SpaceVectorPWM; // Use PWM drive

// Set the motion control loop to be used
motor.controller = MotionControlType::angle; // Angle mode

// Velocity PI controller parameters
motor.PID_velocity.P = 0.05f; // Velocity P value. This value should not be too large, otherwise it will cause jitter.
motor.PID_velocity.I = 0.02; // A larger value will slow down the response.
motor.PID_velocity.D = 0;
// Maximum voltage to be set to the motor
motor.voltage_limit = 6; // Limit the maximum voltage. This value is generally half the supply voltage.

// Velocity low-pass filtering time constant
// The lower the less filtered value
motor.LPF_velocity.Tf = 0.01f; // Filtering

// Angle P controller
motor.P_angle.P = 20; // P value for position PID control
// Maximum velocity of the position control
motor.velocity_limit = 15; // Maximum velocity limit, radians/second

// Use monitoring with serial
Serial.begin(115200); // Open the serial port
// Comment out if not needed
motor.useMonitoring(Serial); //

// Initialize motor
motor.init(); // Initialize the motor
// Align sensor and start FOC
motor.initFOC(); // Align sensor and start FOC

// Add target command T
command.add('T', doTarget, "target angle"); // Send the position via the serial port T command. For example, T6.28 means the motor rotates 6.28 radians, or 1 revolution.

Serial.println(F("Motor ready."));
Serial.println(F("Set the target angle using serial terminal:"));
_delay(1000);
}

void loop() {
motor.loopFOC(); // Start, apply power

motor.move(target_angle); // Move to target position

command.run(); // Monitor input commands
}
