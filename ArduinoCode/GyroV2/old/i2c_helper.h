#ifndef _I2C_HELPER_H
#define _I2C_HELPER_H

#ifndef PIO_UNIT_TESTING
void i2c_init();
int i2c_read(int addr, int start, uint8_t *buffer, int size);
int i2c_write(int addr, int start, const uint8_t *pData, int size);
int i2c_write_reg(int addr, int reg, uint8_t data);
#endif
#endif