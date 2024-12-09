#ifndef _MPU6050_H
#define _MPU6050_H

#define FREQ 30.0 // sample freq in Hz
// #define FREQ 60.0 // sample freq in Hz

typedef struct
{
  bool ignoreSensorData;
  double gyrX;
  double gyrY;
  double gyrZ;
  double accX;
  double accY;
  double accZ;
} SensorData;

extern bool mpu6050_init();
extern SensorData mpu6050_read_sensor_data();

typedef struct
{
  double gyrXOffs;
  double gyrYOffs;
  double gyrZOffs;
} CalibrationData;
CalibrationData calibrate();
extern double gyrXoffs, gyrYoffs, gyrZoffs;
#endif
