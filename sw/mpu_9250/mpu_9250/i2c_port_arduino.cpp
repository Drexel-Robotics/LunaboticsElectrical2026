#include <Wire.h>
#include "i2c_port.h"

int i2c_write(uint8_t dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    Wire.beginTransmission(dev);
    Wire.write(reg);
    for (uint16_t i = 0; i < len; i++)
        Wire.write(data[i]);
    return Wire.endTransmission();
}

int i2c_read(uint8_t dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    Wire.beginTransmission(dev);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(dev, len);
    for (uint16_t i = 0; i < len; i++)
        data[i] = Wire.read();
    return 0;
}