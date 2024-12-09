#ifndef PIO_UNIT_TESTING
#include <Arduino.h>
#include <Wire.h>
#include "debug.h"
#include "i2c_helper.h"

void i2c_init()
{
  Wire.begin();
}
// ---- I2C routines

int i2c_read(int addr, int start, uint8_t *buffer, int size)
{
  int i, n;

  Wire.beginTransmission(addr);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false); // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(addr, size, true);
  i = 0;
  while (Wire.available() && i < size)
  {
    buffer[i++] = Wire.read();
  }
  if (i != size)
    return (-11);

  return (0); // return : no error
}

int i2c_write(int addr, int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(addr);
  n = Wire.write(start); // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size); // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0); // return : no error
}

int i2c_write_reg(int addr, int reg, uint8_t data)
{
  int error;

  error = i2c_write(addr, reg, &data, 1);
  return (error);
}
#endif