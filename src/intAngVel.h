#pragma once

#include <quat.h>
#include <euler.h>
#include <vector3.h>

void intAngVel(geometry::quat& q,
               geometry::Vector3<float> &w0,
               const geometry::Vector3<float> &w,
               geometry::euler_angle &orient,
               geometry::euler_angle &dorient);

extern const float dt;                 // time step in seconds
