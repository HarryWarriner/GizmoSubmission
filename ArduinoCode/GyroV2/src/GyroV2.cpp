#ifndef PIO_UNIT_TESTING
// MPU-6050 Accelerometer + Gyro

// Bluetooth module connected to digital pins 2,3
// I2C bus on A4, A5
// Servo on pin 0
#include <Arduino.h>
#include "debug.h"
// #include "i2c_helper.h"
// #include "mpu6050.h"
#include "mpu_bno.h"
#include "motors.h"
#include "joystick.h"
#include "brain.h"
// #include "memcheck.h"
// #include <SoftwareSerial.h>

// Bluetooth transmitter, used optionally
// SoftwareSerial BTSerial(2, 3); // RX | TX

unsigned long lastDebugTime = 0;
#define debugInterval 1000000L
unsigned long loopCount = 0;
unsigned long lastSensorReadTime = 0;
#define sensorReadInterval (1000000 / FREQ)
#define tenseconds 10000000L
bool debug = false;
bool motor = true;
extern float gx, gy, gz;

Movement movementData;
JoystickPosition joystickPosition;
bool hasMPU6050 = true;
bool issetup = false;
void setup()
{
  if (issetup)
    return;
  issetup = true;
  delay(1);
  Serial.begin(115200);
  delay(1);
  debug_printf("Setup started...\n");
  // Clear serial port of any buffered data
  while (Serial.available())
  {
    // debug_printf("%c", Serial.read());
    debug_printf(".");
  }

  // Initialize motors
  motor_init();
  //
  // Initialize i2c library
  //
  // i2c_init();
  //
  // Setup Gyro
  //
  // hasMPU6050 = mpu6050_init();
  mpu_init();
  //
  // Initialize brain
  //
  brain_init();

  debug_printf("Setup ok.\n");
}

void loop()
{
  unsigned long imuTestStartTime;
  loopCount++;
  // Read Serial input
  if (Serial.available() > 0)
  {
    char cmd = Serial.read();
    switch (cmd)
    {
    case 'J': // joystick coordinates
      joystickPosition = read_joystick_position(debug);
      break;
    case 'D': // toggle debug
      debug = !debug;
      debug_printf("debug set to %d\n", debug);
      break;
    case 'M': // control motors
      motor = !motor;
      debug_printf("motor set to %d\n", motor);
      break;
    case 'q': // rotate left
      if (joystickPosition.rotation > -1.0)
        joystickPosition.rotation -= 0.1;

      debug_printf("joystick at %f, %f, R=%f\n", joystickPosition.x, joystickPosition.y, joystickPosition.rotation);
      break;
    case 'e': // rotate right
      if (joystickPosition.rotation < 1.0)
        joystickPosition.rotation += 0.1;

      debug_printf("joystick at %f, %f, R=%f\n", joystickPosition.x, joystickPosition.y, joystickPosition.rotation);
      break;
    // WASD
    case 'a':
      if (joystickPosition.x > -1.0)
        joystickPosition.x += -0.1;
      debug_printf("joystick at %f, %f, R=%f\n", joystickPosition.x, joystickPosition.y, joystickPosition.rotation);
      break;
    case 'd':
      if (joystickPosition.x < 1.0)
        joystickPosition.x += +0.1;
      debug_printf("joystick at %f, %f, R=%f\n", joystickPosition.x, joystickPosition.y, joystickPosition.rotation);
      break;
    case 's':
      if (joystickPosition.y > -1.0)
        joystickPosition.y += -0.1;
      debug_printf("joystick at %f, %f, R=%f\n", joystickPosition.x, joystickPosition.y, joystickPosition.rotation);
      break;
    case 'w':
      if (joystickPosition.y < 1.0)
        joystickPosition.y += 0.1;
      debug_printf("joystick at %f, %f, R=%f\n", joystickPosition.x, joystickPosition.y, joystickPosition.rotation);
      break;
      // IJKL - shift calibration centre
    case 'j':
      if (joystickPosition.shiftx > -1.0)
        joystickPosition.shiftx += -0.1;
      debug_printf("joystick SHIFT at %f, %f\n", joystickPosition.shiftx, joystickPosition.shifty);
      break;
    case 'l':
      if (joystickPosition.shiftx < 1.0)
        joystickPosition.shiftx += +0.1;
      debug_printf("joystick SHIFT at %f, %f\n", joystickPosition.shiftx, joystickPosition.shifty);
      break;
    case 'k':
      if (joystickPosition.shifty > -1.0)
        joystickPosition.shifty += -0.1;
      debug_printf("joystick SHIFT at %f, %f\n", joystickPosition.shiftx, joystickPosition.shifty);
      break;
    case 'i':
      if (joystickPosition.shifty < 1.0)
        joystickPosition.shifty += 0.1;
      debug_printf("joystick SHIFT at %f, %f\n", joystickPosition.shiftx, joystickPosition.shifty);
      break;

    case 't':
      joystickPosition.x = 0;
      joystickPosition.y = 0;
      joystickPosition.shiftx = 0;
      joystickPosition.shifty = 0;
      joystickPosition.rotation = 0;
      debug_printf("joystick at %f, %f, R=%f\n", joystickPosition.x, joystickPosition.y, joystickPosition.rotation);
      break;
    case 'I':
      // gx = 0;
      // gy = 0;
      // gz = 0;
      debug_printf("imu testing starting for 10 seconds\n");
      imuTestStartTime = micros();
      while ((imuTestStartTime + tenseconds) > micros()) // 10 seconds
      {
        // SensorData sensorData = mpu6050_read_sensor_data();
        SensorData sensorData = mpu_read_sensor_data();
        brain_imu_update(sensorData, true);
        delay(1000 / FREQ); // delay is ms, not microseconds
      }
      debug_printf("imu testing done\n");
      break;

    case 'c': // Reset gyro to 0
    {
      SensorData sensorData = mpu_read_sensor_data();
      brain_imu_setpos(sensorData);
    }
    break;
    case '7': // brain algorithm 1
      brain_algorithm(1);
      break;
    case '8': // brain algorithm 2
      brain_algorithm(2);
      break;
    case '9': // brain algorithm 3
      brain_algorithm(3);
      break;

    // Motor Tests

    case '1': 
      debug_printf("Spinning motor X\n");
      spin_motor('1');
      break;
    case '2':
      debug_printf("Spinning motor Y\n");
      spin_motor('2');
      break;
    case '3':
      debug_printf("Spinning motor Z\n");
      spin_motor('3');
      break;
    case '4':
      debug_printf("Spinning all motors\n");
      spin_motor('A');
      break;
    case '5':
      debug_printf("Spinning all motors BACK\n");
      spin_motor('B');
      break;

    default: // Terminal input error 
      debug_printf("unknown command %c\n", cmd);
      debug_printf(" wasd for joystick movement D for debug, M for motors, t to center joystick\n");
    }
  }
  //
  // Time specific lines
  //
  unsigned long currentTime = micros();
  if ((currentTime - lastDebugTime) > debugInterval)
  {
    if (debug)
    {
      // debug_printf("DEBUG:: 1 second took %lu micros, for %lu iterations, or %lu micros per tick\n",
      //              currentTime - lastDebugTime,
      //              loopCount,
      //              (currentTime - lastDebugTime) / loopCount);
      print_movement(movementData);
    }
    lastDebugTime = currentTime;
    loopCount = 0;
  }

  if ((currentTime - lastSensorReadTime >= sensorReadInterval))
  {
    lastSensorReadTime = currentTime;
    SensorData sensorData;
    if (hasMPU6050)
      sensorData = mpu_read_sensor_data();
    else if (debug)
      debug_printf("!MPU!");
    movementData = brain_update(sensorData, joystickPosition, false);
  }
  if (motor && hasMPU6050)
  {
    motor_move_if_needed(movementData, debug);
  }
}
#endif