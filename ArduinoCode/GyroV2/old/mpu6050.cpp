#ifndef PIO_UNIT_TESTING
#include <Arduino.h>
#endif
#include "debug.h"
#include "mpu6050.h"
#include "i2c_helper.h"

// Constants
#define MPU6050_I2C_ADDRESS 0x68

#define MPU6050_WHO_AM_I 0x75
#define MPU6050_DEVICE_ID 0x70 // Expected device ID (could be 0x68 or 0x69) MPU 6500 not renaming the values because im lazy

// Forward function definitions
bool checkMPU6050Connection();

// Local variables

// global angle, gyro derived
// double gSensitivity = 65.5; // for 500 deg/s, check data sheet
#define gSensitivity  65.5
// double gyrXoffs = -281.00, gyrYoffs = 18.00, gyrZoffs = -83.00;
double gyrXoffs = -13, gyrYoffs = -91, gyrZoffs = 21.00;

// // Global variables
// double gyrX = 0, gyrY = 0, gyrZ = 0;
// double accX = 0, accY = 0, accZ = 0;

// // for testing only
// void set_sensor_data(double gX, double gY, double gZ, double aX, double aY, double aZ)
// {
//     gyrX = gX;
//     gyrY = gY;
//     gyrZ = gZ;
//     accX = aX;
//     accY = aY;
//     accZ = aZ;
// }
#ifndef PIO_UNIT_TESTING

bool mpu6050_init()
{
  double sample_div;
  // Check MPU6050 connection
  if (!checkMPU6050Connection())
  {
    debug_printf("MPU6050 not connected!\n");
    return false;
  }
  else
  {
    debug_printf("MPU6050 connected successfully.\n");
  }

  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x6b, 0x00);
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1a, 0x01);
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1b, 0x08);

  sample_div = 1000 / FREQ - 1;
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x19, (uint8_t)sample_div);

  calibrate();
  debug_printf("mpu init done\n");
  return true;
}

SensorData mpu6050_read_sensor_data()
{
  SensorData sensorData;

  uint8_t i2cData[14];
  uint8_t error;
  // read imu data
  error = i2c_read(MPU6050_I2C_ADDRESS, 0x3b, i2cData, sizeof(i2cData)); // 14);
  if (error != 0)
  {
    debug_printf("I2C read error in mpu6050_read_sensor_data: %u\n", (unsigned int)error);
    return sensorData; // this will have zeros in it.
  }

  // assemble 16 bit sensor data
  sensorData.accX = ((i2cData[0] << 8) | i2cData[1]);
  sensorData.accY = ((i2cData[2] << 8) | i2cData[3]);
  sensorData.accZ = ((i2cData[4] << 8) | i2cData[5]);

  sensorData.gyrX = (((i2cData[8] << 8) | i2cData[9]) - gyrXoffs) / gSensitivity;
  sensorData.gyrY = (((i2cData[10] << 8) | i2cData[11]) - gyrYoffs) / gSensitivity;
  sensorData.gyrZ = (((i2cData[12] << 8) | i2cData[13]) - gyrZoffs) / gSensitivity;

  return sensorData;
}

bool checkMPU6050Connection()
{
  uint8_t deviceID;
  int error = i2c_read(MPU6050_I2C_ADDRESS, MPU6050_WHO_AM_I, &deviceID, 1);
  if (error != 0)
  {
    debug_printf("I2C read error: %u\n", (unsigned int)error);
    return false;
  }
  // this crashes - don't run!
  // debug_printf("MPU6050 Device ID: 0x%02X, Expected: 0x%02X\n", (unsigned int)deviceID, MPU6050_DEVICE_ID);
  // Serial.println(deviceID, HEX);
  // Serial.println(MPU6050_DEVICE_ID);

  if (deviceID != MPU6050_DEVICE_ID)
  {
    debug_printf("MPU6050 initialization failed: Incorrect device ID\n");
    return false;
  }
  return true;
}

CalibrationData calibrate()
{
  int x;
  long xSum = 0, ySum = 0, zSum = 0;
  uint8_t i2cData[6];
  int num = 500;
  uint8_t error;
  debug_printf("calibrating gyro... This takes %d seconds\n", (num * 10) / 1000);

  for (x = 0; x < num; x++)
  {
    error = i2c_read(MPU6050_I2C_ADDRESS, 0x43, i2cData, sizeof(i2cData)); // 6);
    if (error != 0)
    {
      debug_printf("I2C read error in calibrate: %u\n", (unsigned int)error);
      return {0, 0, 0};
    }

    xSum += ((i2cData[0] << 8) | i2cData[1]);
    ySum += ((i2cData[2] << 8) | i2cData[3]);
    zSum += ((i2cData[4] << 8) | i2cData[5]);
    delay(10); // pause for 10 MILLIseconds
  }
  gyrXoffs = xSum / num;
  gyrYoffs = ySum / num;
  gyrZoffs = zSum / num;

  debug_printf("Calibration result: %f, %f, %f\n", gyrXoffs, gyrYoffs, gyrZoffs);
  CalibrationData data = {gyrXoffs, gyrYoffs, gyrZoffs};
  return data;
}

#endif
