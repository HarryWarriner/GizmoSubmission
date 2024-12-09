#ifndef PIO_UNIT_TESTING
#include <Arduino.h>
#include <Wire.h>
#include "brain.h"
#include "BNO055_support.h"

// This structure contains the details of the BNO055 device that is connected. (Updated after initialization)
struct bno055_t myBNO;
struct bno055_euler myEulerData; // Structure to hold the Euler data

void mpu_init()
{
  debug_printf("before mpu\n");
  Wire.begin();
  // Initialization of the BNO055
  BNO_Init(&myBNO); // Assigning the structure to hold information about the device
  debug_printf("2\n");
  // Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  debug_printf("after mpu\n");
}

SensorData mpu_read_sensor_data()
{
  bno055_read_euler_hrp(&myEulerData);
  SensorData data;
  data.h = float(myEulerData.h) / 16.00;
  data.r = float(myEulerData.r) / 16.00;
  data.p = float(myEulerData.p) / 16.00;
  return data;
}
#endif