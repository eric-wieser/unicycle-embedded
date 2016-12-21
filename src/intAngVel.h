#pragma once

void intAngVel(const float *w,
               float &phi, float &theta, float &psi,
               float &dphi, float &dtheta, float &dpsi);

extern const float dt;                 // time step in seconds
