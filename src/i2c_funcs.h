#pragma once

#include <stdint.h>
#include <p32_defs.h>

// These functions are subsitutes for those found in MPIDE's plib

namespace i2c_funcs {

void OpenI2C(volatile p32_i2c& i2c, uint32_t config1, uint32_t config2);
void StopI2C(volatile p32_i2c& i2c);
char MasterWriteI2C(volatile p32_i2c& i2c, uint8_t data_out);
void StartI2C(volatile p32_i2c& i2c);
void IdleI2C(volatile p32_i2c& i2c);
char DataRdyI2C(volatile p32_i2c& i2c);
size_t MastergetsI2C(volatile p32_i2c& i2c, size_t length, uint8_t * rdptr, uint32_t data_wait);

const uint32_t I2C_EN = 1 << 15;

}