#ifndef PIO_UNIT_TESTING
#include <Arduino.h>
#else
#include <stdio.h>
#endif
#include <math.h>
#include "debug.h"
#include "brain.h"
// #include "mpu6050.h"
#include "joystick.h"

// Gyro absolute positions
float gx = 0, gy = 0, gz = 0;
int algorithm = 2;
// const double speed_max = 90.0;
// const unsigned long stepIntervalMin = 1000;
// const unsigned long stepIntervalMax = 20000;
// const unsigned long stepIntervalMax = 5000;
void brain_init()
{
  gx = 0;
  gy = 0;
  gz = 0;
}

#ifdef PIO_UNIT_TESTING
double constrain(double v, double min_value, double max_value)
{
  if (v < min_value)
    return min_value;
  if (v > max_value)
    return max_value;
  return v;
}
#endif

// Map the data across a scale
double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
// Brain algorithm for speed control (P of the PID controller)
void brain_algorithm(int a)
{
  algorithm = a;
  debug_printf("alg = %d\n", a);
}

double max_angle = 100.0; // Max angle 
double calculate_speed(double angle)
{
  double sign = angle < 0 ? -1 : 1;
  // normalise angle
  angle = fabs(angle);
  angle = angle > max_angle ? max_angle : angle;
  angle = angle / max_angle;
  double speed = angle * sign;
  switch (algorithm)
  {
  case 1: // Case 1, linear speed to angle
    speed = angle * sign;
    break;
  case 2: // Case 2, speed = square root of the angle
    speed = pow(angle, 1.0 / 2.0) * sign;
    break;
  case 3: // Case 3, speed = angle squared
    speed = angle * angle * sign;
    break;
  default:
    break;
  }
  // debug_printf("%3.3f\n", speed);
  if (speed > 0.006)
    return speed;
  if (speed < -0.006)
    return speed;
  return 0;
}
double base_h = 0;
double base_r = 0;
double base_p = 0;
void brain_imu_setpos(SensorData sensorData)
{
  base_h = sensorData.h;
  base_r = sensorData.r;
  base_p = sensorData.p;
}
void brain_imu_update(SensorData sensorData, bool debug)
{
  if (debug)
  {
    debug_printf("SensorGyro: h,p,r::%.2f, %.2f, %.2f\n", sensorData.h, sensorData.p, sensorData.r);
  }
}
Movement brain_update(SensorData sensorData, JoystickPosition joystick, bool debug)
{
  Movement movementResult;
  brain_imu_update(sensorData, debug);
  gx = sensorData.r - base_r + joystick.shiftx * 2;
  gy = sensorData.p - base_p + joystick.shifty * 2;
  gz = sensorData.h - base_h;
  // setpoint caluclation
  double movementX = -calculate_speed(gx) + joystick.x;      // -1.0 -> 1.0
  double movementY = calculate_speed(gy) + joystick.y;       // -1.0 -> 1.0
  double rotation = calculate_speed(gz) + joystick.rotation; // 0-100.
  // double movementX = -sensorData.r/100.0 + joystick.x;           // -1.0 -> 1.0
  // double movementY = sensorData.p/100.0 + joystick.y;            // -1.0 -> 1.0
  // double rotation = sensorData.h/100.0 + joystick.x * max_angle; // 0-100.

  rotation = 0; // joystick.rotation;// * max_angle;
  double scale = -2.1;

  // Vector Decomposition for the motors
  movementResult.speedX = (-0.33333333 * movementX + 0.57735027 * movementY + 0.33333333 * rotation) * scale;
  movementResult.speedY = (-0.33333333 * movementX - 0.57735027 * movementY + 0.33333333 * rotation) * scale;
  movementResult.speedZ = (0.66666667 * movementX + 0 * movementY + 0.33333333 * rotation) * scale;

  return movementResult;
}
