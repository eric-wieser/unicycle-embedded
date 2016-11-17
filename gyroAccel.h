#pragma once

#include <stdint.h>
#include <stddef.h>

void gyroAccelSetup();

void accelRead(float &, float &, float &);
void gyroRead(float &, float &, float &);
