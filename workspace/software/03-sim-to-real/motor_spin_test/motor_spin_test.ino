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
// RETUNED - the original P=0.05/I=0.02 here (inherited from a generic SimpleFOC
// tutorial template, not tuned for this motor) was the cause of "needs a manual
// push to start": with gains that low, the controller can't generate enough
// corrective voltage near zero speed to break static friction/cogging on this
// motor, so it sits there until a hand-push gets the rotor past stiction, then
// the small gain barely sustains motion. These values are SimpleFOC's own
// documented velocity-loop defaults (P=0.5, I=10 - see docs.simplefoc.com/
// velocity_loop) blended with the community's tested starting point for
// 2804/2808-class gimbal motors specifically (P=0.2, I=20 - same motor family
// as our FIT1034). Still a starting point, not a guaranteed fit for this exact
// mechanical load - if it oscillates/buzzes, back P off in ~20% steps; if it's
// still sluggish to start, raise I first (it's what overcomes stiction here).
// For a load-specific tune, use the Ziegler-Nichols procedure instead:
// docs.simplefoc.com/zn_tuning_pid (set I=D=0, raise P until sustained
// oscillation at gain Kc/period Tc, then P=0.45*Kc, I=0.54*Kc/Tc for PI).
motor.PID_velocity.P = 0.3f;
motor.PID_velocity.I = 15.0f;
motor.PID_velocity.D = 0;
// Maximum voltage to be set to the motor
motor.voltage_limit = 6; // Limit the maximum voltage. This value is generally half the supply voltage.

// Velocity low-pass filtering time constant - unchanged, already in
// SimpleFOC's own recommended 0.005-0.01s range (docs.simplefoc.com/velocity_loop)
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
