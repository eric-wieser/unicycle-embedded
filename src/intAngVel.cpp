// compute new orientation and angular velocity in world Euler angle
// representaion from body angular velocities. The function stores the world
// orientation in unit quaternion q and body angular velocity in vector w0.
//
// Carl Edward Rasmussen, 2011-09-28
// Aleksi Tukiainen, 2016-05-20

//TODO: Cleanup extra bits commented out

#include "intAngVel.h"

using namespace geometry;

void intAngVel(quat& q,
               Vector3<float> &w0,
               const Vector3<float> &w,
               euler_angle &orient,
               euler_angle &dorient)
{
  Vector3<float> w_mean = (w + w0) / 2.0;

  //Here below is the conversion from local angular velocities into a position quaternion.
  // Here we have the area under the speed curve - the position!
  quat p(1 + (w_mean[0]*dt/2.0)*(w_mean[1]*dt/2.0)*(w_mean[2]*dt/2.0),
         (w_mean[0]*dt/2.0) - (w_mean[1]*dt/2.0)*(w_mean[2]*dt/2.0),
         (w_mean[1]*dt/2.0) + (w_mean[2]*dt/2.0)*(w_mean[0]*dt/2.0),
         (w_mean[2]*dt/2.0) - (w_mean[0]*dt/2.0)*(w_mean[1]*dt/2.0));

  q = p*q;
  q.normalize();               // new unit quaternion = previous position q + local integration p

  quat p1( 1, w[0]/2*dt/10, w[1]/2*dt/10, w[2]/2*dt/10); // quaternion for the instantaneous position change
  quat q1 = p1*q;
  q1.normalize();       // unit quaternion to represent the next position after a tiny timestep forward

  // extract Euler angles (currently in 1,2,3)
  orient = q.euler();

  // extract Euler angles after small timestep (currently in 1,2,3)
  euler_angle e1 = q1.euler();
  dorient.phi   = (e1.phi   - orient.phi)/(dt/10);
  dorient.theta = (e1.theta - orient.theta)/(dt/10);
  dorient.psi   = (e1.psi   - orient.psi)/(dt/10); // instantaneous Euler velocities

  // save speeds for next call
  w0 = w;
}
