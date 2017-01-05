#pragma once

#include "quat.h"

struct euler_angle {
	float phi;   // roll - supposedly, based on older code
	float theta; // pitch
	float psi;   // yaw
};

void intAngVel(quat& q,
               float *w0,
               const float *w,
               euler_angle &orient,
               euler_angle &dorient);

extern const float dt;                 // time step in seconds
