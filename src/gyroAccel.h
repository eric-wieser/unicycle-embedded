#pragma once

#include <stdint.h>
#include <stddef.h>

void gyroAccelSetup();

void accelRead(float &x, float &y, float &z);
void gyroRead(float &x, float &y, float &z);
