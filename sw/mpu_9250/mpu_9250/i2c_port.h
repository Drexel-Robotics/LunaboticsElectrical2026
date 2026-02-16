#pragma once
#include <stdint.h>

int i2c_write(uint8_t dev, uint8_t reg, uint8_t *data, uint16_t len);
int i2c_read(uint8_t dev, uint8_t reg, uint8_t *data, uint16_t len);