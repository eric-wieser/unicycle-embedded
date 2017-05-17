// compute new orientation and angular velocity in world Euler angle
// representaion from body angular velocities. The function stores the world
// orientation in unit quaternion q and body angular velocity in vector w0.
//
// Carl Edward Rasmussen, 2011-09-28
// Aleksi Tukiainen, 2016-05-20

//TODO: Cleanup extra bits commented out

#include "intAngVel.h"

using namespace geometry;

quat integrate_quat_fast(const quat &q0, const Vector3<float> &w, float dt) {
  return (1 + 0.5*quat(w)*dt) * q0;
}

quat integrate_quat_aleksi(const quat &q0, const Vector3<float> &w, float dt) {
  // This was previously used for a large timesteps. It is not clear where it
  // came from. It seems to be the above with an extra high-order term
  quat p(1 + (w[0]*dt/2.0)*(w[1]*dt/2.0)*(w[2]*dt/2.0),

         (w[0]*dt/2.0) - (w[1]*dt/2.0)*(w[2]*dt/2.0),
         (w[1]*dt/2.0) + (w[2]*dt/2.0)*(w[0]*dt/2.0),
         (w[2]*dt/2.0) - (w[0]*dt/2.0)*(w[1]*dt/2.0));
  return p * q0;
}

void intAngVel(quat& q,
               Vector3<float> &w0,
               const Vector3<float> &w,
               joint_angles &orient,
               joint_angles &dorient)
{

  // extract Euler angles after integrating with mean angular velocity
  q = integrate_quat_aleksi(q, (w + w0) / 2.0, dt);
  q.normalize();
  orient = q;

  // extract Euler angles after small timestep
  float dt_small = dt/10;
  quat q1 = integrate_quat_fast(q, w, dt_small);
  q1.normalize();

  // approximate instantaneous Euler velocities
  joint_angles e1 = q1;
  dorient.phi   = (e1.phi   - orient.phi)/dt_small;
  dorient.theta = (e1.theta - orient.theta)/dt_small;
  dorient.psi   = (e1.psi   - orient.psi)/dt_small;

  // save speeds for next call
  w0 = w;
}
