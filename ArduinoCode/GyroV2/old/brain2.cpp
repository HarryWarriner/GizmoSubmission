#if BRAIN2

#ifndef PIO_UNIT_TESTING
#include <Arduino.h>
#else
#include <stdio.h>
#endif
#include <math.h>
#include "sensorfusion.h"
#include "debug.h"
#include "brain.h"
#include "mpu6050.h"
#include "joystick.h"

// Gyro absolute positions
float gx = 0, gy = 0, gz = 0;

// const double speed_max = 90.0;
// const unsigned long stepIntervalMin = 1000;
// const unsigned long stepIntervalMax = 20000;
// const unsigned long stepIntervalMax = 5000;
void brain_init()
{
  gx = 0;
  gy = 0;
  gz = 0;
  sensfusionInit();
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

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#define  max_angle 20
double calculate_speed(double angle)
{
  double sign = angle < 0 ? -1 : 1;
  // normalise angle
  angle = fabs(angle);
  angle = angle > max_angle ? max_angle : angle;
  angle = angle / max_angle;

  // calculate speed from angle - slow down when close to max angle
  // double speed = fabs(angle * angle * angle) * sign;
  // calculate speed from angle - speed up when close to min angle
  // double speed = fabs(angle) * sign;
  double speed = pow(fabs(angle), 1.0 / 2.0) * sign;

  return speed;
}

void brain_imu_update(SensorData sensorData, bool debug)
{
  sensfusionUpdateQ(sensorData.gyrX, sensorData.gyrY, sensorData.gyrZ,
                    sensorData.accX, sensorData.accY, sensorData.accZ, 1.0 / FREQ);
  // sensfusionUpdateQ(sensorData.gyrX, sensorData.gyrY, sensorData.gyrZ,
  //                   sensorData.accX, sensorData.accY, sensorData.accZ, 0, 0, 0, 1.0 / FREQ);
  sensfusionGetEulerRPY(&gx, &gy, &gz);
  // ROLL, PITCH, YAW
  // ROLL - X AXIS ROTATION
  // PITCH - Y AXIS ROTATION
  // YAW - Z AXIS ROTATION
  //
  if (debug)
    debug_printf("gx,y,z = %.2f, %.2f, %.2f\n", gx, gy, gz);
}

Movement brain_update(SensorData sensorData, JoystickPosition joystick, bool debug)
{
  Movement movementResult;
  if (!sensorData.ignoreSensorData)
  {
    brain_imu_update(sensorData, debug);
  }
  // Calculate vector to recover from current position to 0,0

  // Update movement variables
  double movementY = calculate_speed(gx + joystick.x * max_angle);
  double movementX = calculate_speed(gy + joystick.y * max_angle);

  double rotation = joystick.rotation * 3.14159;
  double scale = 1.1;
  movementResult.speedX = (-0.33333333 * movementX + 0.57735027 * movementY + 0.33333333 * rotation) * scale;
  movementResult.speedY = (-0.33333333 * movementX - 0.57735027 * movementY + 0.33333333 * rotation) * scale;
  movementResult.speedZ = (0.66666667 * movementX + 0 * movementY + 0.33333333 * rotation) * scale;

  return movementResult;
}
#endif