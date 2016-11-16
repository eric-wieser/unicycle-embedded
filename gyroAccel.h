#pragma once

#include <stdint.h>
#include <stddef.h>

void gyroAccelSetup();

void accelWrite(uint8_t, uint8_t);
void accelRead(uint8_t, uint8_t *, size_t);
void accelRead(float &, float &, float &);
void gyroWrite(uint8_t, uint8_t);
void gyroRead(uint8_t, uint8_t *, size_t);
void gyroRead(float &, float &, float &);
