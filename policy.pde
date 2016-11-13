// This file implements the policy
// Aleksi Tukiainen, 2016-05-20

#include "policy.h"
float policyTurntable(float v1, float v2, float v3, float v4, float v5, float v6, float v7, float v8, float v9, float v10)
{
  float param[10] = {2.56325, -0.02692, 0.02074, 0.06003, -0.00312, 1.61079, -2.93646, 11.57684, -0.05651, 0.53147};
  float bias = 0.13994;
  return v1*param[0]+v2*param[1]+v3*param[2]+v4*param[3]+v5*param[4]+v6*param[5]+v7*param[6]+v8*param[7]+v9*param[8]+v10*param[9] + bias;
}


float policyWheel(float v1, float v2, float v3, float v4, float v5, float v6, float v7, float v8, float v9, float v10)
{
  float param[10] = {0.04465, -0.01305, 0.02915, 0.43688, 0.00171, -0.56873, -0.62321, 0.15515, -0.01179, 1.00638};
  float bias = 0.02970;
  return v1*param[0]+v2*param[1]+v3*param[2]+v4*param[3]+v5*param[4]+v6*param[5]+v7*param[6]+v8*param[7]+v9*param[8]+v10*param[9] + bias;

}
