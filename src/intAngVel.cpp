// compute new orientation and angular velocity in world Euler angle
// representaion from body angular velocities. The function stores the world
// orientation in unit quaternion q and body angular velocity in vector w0.
//
// Carl Edward Rasmussen, 2011-09-28
// Aleksi Tukiainen, 2016-05-20

//TODO: Cleanup extra bits commented out

#include "intAngVel.h"

using namespace geometry;

/**
 * @brief      Integrate from a starting quaternion, applying an angular
 *             velocity
 *
 * @param[in]  q0    The initial quaternion, q(t)
 * @param[in]  w     The angular velocity
 * @param[in]  dt    The timestep to integrate over
 *
 * @return     The quaternion at q(t + dt)
 */
quat integrate_quat(const quat &q0, const Vector3<float> &w, float dt) {
  // from integrating $\dot{q} = 0.5 \omega q$
  return exp(0.5*quat(w)*dt) * q0;
}

quat integrate_quat_fast(const quat &q0, const Vector3<float> &w, float dt) {
  // as above, approximating `exp(x)` as `1 + x`. This was previously used for
  // small timesteps
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
  // normalizing is not strictly necessary but numerical error buildup happens
  // otherwise

  // extract Euler angles after integrating with mean angular velocity
  q = integrate_quat(q, (w + w0) / 2.0, dt);
  q.normalize();
  orient = q;

  // extract Euler angles after small timestep
  float dt_small = dt/10;
  quat q1 = integrate_quat(q, w, dt_small);
  q1.normalize();

  // approximate instantaneous Euler velocities
  joint_angles e1 = q1;
  dorient.phi   = (e1.phi   - orient.phi)/dt_small;
  dorient.theta = (e1.theta - orient.theta)/dt_small;
  dorient.psi   = (e1.psi   - orient.psi)/dt_small;

  // save speeds for next call
  w0 = w;
}
