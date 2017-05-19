#pragma once

#include <stdint.h>
#include <stddef.h>

#include <vector3.h>

void gyroAccelSetup();

geometry::Vector3<float> accelRead();
geometry::Vector3<float> gyroRead();
geometry::Vector3<float> gyroCalibrate(int N = 20);
