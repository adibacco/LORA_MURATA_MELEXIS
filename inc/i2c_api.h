#ifndef _I2C_API_
#define _I2C_API_

int i2c_start(I2C_HandleTypeDef *i2c_dev);
int i2c_stop(I2C_HandleTypeDef *i2c_dev);

int i2c_read(I2C_HandleTypeDef *i2c_dev, int address, char *data, int length, int stop);
int i2c_write(I2C_HandleTypeDef *i2c_dev, int address, const char *data, int length, int stop); 
int i2c_byte_read(I2C_HandleTypeDef *i2c_dev, int last);
int i2c_byte_write(I2C_HandleTypeDef *i2c_dev, int data);

void i2c_reset(I2C_HandleTypeDef *i2c_dev);

#endif
