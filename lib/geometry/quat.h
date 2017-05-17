#pragma once

#include "vector3.h"

namespace geometry {

class quat
{
public:
  float x, y, z, w;
  quat() {};
  quat(float xx, float yy, float zz, float ww);
  quat(Vector3<float> v);
  quat(float k);
  quat(float k, Vector3<float> v);

  void normalize();
  quat conj() const;
  quat operator* (const quat &q) const;
  quat& operator*= (float f);
  quat& operator/= (float f);
  quat& operator+= (quat q);
  quat& operator-= (quat q);
  float norm() const;

  Vector3<float> v() const;
};

inline quat operator +(quat q1, const quat& q2) { return q1 += q2; }
inline quat operator -(quat q1, const quat& q2) { return q1 -= q2; }
inline quat operator *(quat q1, float f) { return q1 *= f; }
inline quat operator *(float f, quat q1) { return q1 *= f; }
inline quat operator /(quat q1, float f) { return q1 /= f; }

quat exp(const quat &q);
}
