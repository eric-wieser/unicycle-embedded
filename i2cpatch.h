#pragma once

#include <stdint.h>

namespace i2c_patch {
	void OpenI2C1(unsigned int config1,unsigned int config2);
	void StopI2C1(void);
	char MasterWriteI2C1(unsigned char data_out);
	void StartI2C1(void);
	void IdleI2C1(void);
	void StopI2C1(void);
	char DataRdyI2C1(void);
	unsigned int MastergetsI2C1(unsigned int length, unsigned char * rdptr, unsigned int i2c1_data_wait);

	const uint32_t I2C_EN = 1 << 15;
};