#pragma once

#include <quat.h>
#include <euler.h>
#include <vector3.h>

/**
 * (2,1,3) euler angles (aka YXZ). This matches the order of the "joints" on the
 * robot:
 *   Y is the axis of the wheel
 *   X is "hinging" on the wheel tangent with the ground
 *   Z is spinning the robot on an imaginary turntable
 * This is very important, as the visualizer assumes this, and if the joints do
 * not align, we end up with our software singularities not matching the
 * hardware ones. Also, we rely on this alignment to compute the wheel angle.
 */
typedef geometry::euler_angles<213> joint_angles;

void intAngVel(geometry::quat& q,
               geometry::Vector3<float> &w0,
               const geometry::Vector3<float> &w,
               joint_angles &orient,
               joint_angles &dorient);

extern const float dt;                 // time step in seconds
