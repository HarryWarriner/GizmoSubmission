#ifndef _MPU_BNO_H
#define _MPU_BNO_H

typedef struct
{
  float h;
  float r;
  float p;
} SensorData;

extern void mpu_init();
extern SensorData mpu_read_sensor_data();

#endif