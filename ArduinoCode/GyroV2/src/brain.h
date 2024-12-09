#ifndef _BRAIN_H
#define _BRAIN_H
#include "joystick.h"
// #include "mpu6050.h"
#include "mpu_bno.h"
#include "joystick.h"
// #include "mpu6050.h"
double mapDouble(double x, double in_min, double in_max, double out_min, double out_max);
void brain_init();
void brain_imu_setpos(SensorData sensorData);
void brain_algorithm(int algorithm);
//
#define FREQ 30.0 // sample freq in Hz
typedef struct
{
  double speedX;
  double speedY;
  double speedZ;
} Movement;

Movement brain_update(SensorData sensorData, JoystickPosition joystick, bool debug);
void brain_imu_update(SensorData sensorData, bool debug);

void print_movement(Movement &movement);

#endif