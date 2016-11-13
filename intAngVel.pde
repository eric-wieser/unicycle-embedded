// compute new orientation and angular velocity in world Euler angle
// representaion from body angular velocities. The function stores the world
// orientation in unit quaternion q and body angular velocity in vector w0.
//
// Carl Edward Rasmussen, 2011-09-28
// Aleksi Tukiainen, 2016-05-20

//TODO: Cleanup extra bits commented out

#include "intAngVel.h"
#include "quat.h"                      // quaternion library
extern const float dt;                 // time step in seconds

void intAngVel(float *w, float &phi, float &theta, float &psi,
               float &dphi, float &dtheta, float &dpsi)
{
  static quat q(1, 0, 0, 0); //we first define q as an identity quaternion with no rotation
  static float w0[3] = {0.0, 0.0, 0.0}; //what is w0 for? - It's for keeping track of the velocity before!
  float phi1, theta1, psi1;
  //Here below is the conversion from local angular velocities into a position quaternion.
  // Here we have the area under the speed curve - the position!
  quat p( 1+((w0[0]+w[0])*dt/4.0)*((w0[1]+w[1])*dt/4.0)*((w0[2]+w[2])*dt/4.0),
          (w0[0]+w[0])*dt/4.0 - ((w0[1]+w[1])*dt/4.0)*((w0[2]+w[2])*dt/4.0),
          (w0[1]+w[1])*dt/4.0 + ((w0[0]+w[0])*dt/4.0)*((w0[2]+w[2])*dt/4.0),
          (w0[2]+w[2])*dt/4.0 - ((w0[0]+w[0])*dt/4.0)*((w0[1]+w[1])*dt/4.0));

  q = p*q;
  q.normalize();               // new unit quaternion = previous position q + local integration p

  quat p1( 1, w[0]/2*dt/10, w[1]/2*dt/10, w[2]/2*dt/10); // quaternion for the instantaneous position change
  quat q1 = p1*q;
  q1.normalize();       // unit quaternion to represent the next position after a tiny timestep forward

  q.euler(phi, theta, psi);             // extract Euler angles (currently in 1,2,3)
  q1.euler(phi1, theta1, psi1);         // extract Euler angles after small timestep (currently in 1,2,3)
  dphi = (phi1-phi)/(dt/10);
  dtheta = (theta1-theta)/(dt/10);
  dpsi = (psi1-psi)/(dt/10); // instantaneous Euler velocities
  for (int i=0; i<3; i++) w0[i] = w[i]; // save speeds for next call
}
