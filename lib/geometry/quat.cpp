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

float quat::norm() const {
  return sqrt(x*x+y*y+z*z+w*w);
}

void quat::normalize() {
  *this /= norm();
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


//void quat::euler(float &phi, float &theta, float &psi)
//{
//  phi = atan2(2*y*z+2*w*x, z*z-y*y-x*x+w*w);
//  theta = -asin(2*x*z-2*w*y);
//  psi = atan2(2*x*y+2*w*z, x*x+w*w-z*z-y*y);
//}

Vector3<float> quat::v() const {
  return Vector3<float>(y, z, w);
}

quat quat::between(Vector3<float> a, Vector3<float> b) {
  float len_ab = sqrt(a.squaredNorm() * b.squaredNorm());
  auto scalar = dot(a, b) / len_ab;
  auto vec = cross(a, b) / len_ab;
  auto double_rotation = quat(scalar, vec);

  // halfway between identity and what we calculated
  auto q = (1 + double_rotation);
  q.normalize();
  return q;
}

// https://math.stackexchange.com/q/1030737/1896
quat exp(const quat &q) {
  float n = q.y*q.y + q.z*q.z + q.w*q.w;
  return ::exp(q.x) * quat(
    cos(n),
    sin(n) * q.v() / n
  );
}

}
