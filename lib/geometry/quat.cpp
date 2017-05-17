// provide simple manipulations of quaternions: conjugate, multiply and extract
// Euler angles (using 123, or pitch, roll, yaw convention). See J. Diebel:
// Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors
// https://www.astro.rug.nl/software/kapteyn/_downloads/attitude.pdf
//
// Carl Edward Rasmussen, 2011-09-28
// Aleksi Tukiainen, 2016-05-20

// The quaternion quat(x,y,z,w) represents x + y i + z j + w k
// Note: The names of the variables are usually ordered as (w,x,y,z)
// That representation is written in comments

#include <math.h>
#include "quat.h"

namespace geometry {

// a vector is equivalent to a quaternion with no real part
quat::quat(Vector3<float> v) : quat(0, v.x, v.y, v.z) {}

// a scalar is a quaternion with only a real part
quat::quat(float k) : quat(k, 0, 0, 0) {}

quat::quat(float xx, float yy, float zz, float ww)
  : x(xx), y(yy), z(zz), w(ww)
{
}

//quat::quat(float ww, float xx, float yy, float zz)
//  : w(ww), x(xx), y(yy), z(zz)
//{
//}

void quat::normalize()
{
  *this /= sqrt(x*x+y*y+z*z+w*w);
}

quat quat::conj() const // conjugate unit quaternion
{
  return quat(x, -y, -z, -w);
}

//quat quat::conj() // conjugate unit quaternion
//{
//  return quat(w, -x, -y, -z);
//}

// basic arithmetic operators
quat& quat::operator+= (quat q) {
  x += q.x;
  y += q.y;
  z += q.z;
  w += q.w;
  return *this;
}

quat& quat::operator-= (quat q) {
  x -= q.x;
  y -= q.y;
  z -= q.z;
  w -= q.w;
  return *this;
}

quat& quat::operator*= (float f) {
  x *= f;
  y *= f;
  z *= f;
  w *= f;
  return *this;
}

quat& quat::operator/= (float f) {
  x /= f;
  y /= f;
  z /= f;
  w /= f;
  return *this;
}

// quaternion multiplication
quat quat::operator* (const quat &q) const
{

  // This function seems right: see p. 14 on J. Diebel's paper

  return quat(q.x*x - q.y*y - q.z*z - q.w*w,
              q.x*y + q.y*x + q.z*w - q.w*z,
              q.x*z - q.y*w + q.z*x + q.w*y,
              q.x*w + q.y*z - q.z*y + q.w*x);

}

//quat quat::operator* (const quat &q) const
//{
//
//  // This function seems right: see p. 14 on J. Diebel's paper
//
//  return quat(w*q.w - x*q.x - y*q.y - z*q.z,
//              x*q.w + w*q.x + z*q.y - y*q.z,
//              y*q.w - z*q.x + w*q.y + x*q.z,
//              z*q.w + y*q.x - x*q.y + w*q.z);
//
//}

//From p.12

euler_angle quat::euler() const
{
  euler_angle e;
  e.phi   = atan2(2*z*w+2*x*y, w*w-z*z-y*y+x*x);
  e.theta = -asin(2*y*w-2*x*z);
  e.psi   = atan2(2*y*z+2*x*w, y*y+x*x-w*w-z*z);
  return e;
}

//void quat::euler(float &phi, float &theta, float &psi)
//{
//  phi = atan2(2*y*z+2*w*x, z*z-y*y-x*x+w*w);
//  theta = -asin(2*x*z-2*w*y);
//  psi = atan2(2*x*y+2*w*z, x*x+w*w-z*z-y*y);
//}

}
