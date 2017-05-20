#pragma once

#include "vector3.h"

namespace geometry {

class quat
{
public:
  float x, y, z, w;
  quat() {};
  constexpr quat(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
  constexpr quat(float k, Vector3<float> v) : quat(k, v.x, v.y, v.z) {}
  constexpr quat(Vector3<float> v)          : quat(0, v.x, v.y, v.z) {}
  constexpr quat(float k)                   : quat(k, 0, 0, 0) {}

  void normalize();
  quat conj() const;
  quat operator* (const quat &q) const;
  quat& operator*= (float f);
  quat& operator/= (float f);
  quat& operator+= (quat q);
  quat& operator-= (quat q);
  float norm() const;

  Vector3<float> v() const;

  static quat between(Vector3<float> a, Vector3<float> b);
};

inline quat operator +(quat q1, const quat& q2) { return q1 += q2; }
inline quat operator -(quat q1, const quat& q2) { return q1 -= q2; }
inline quat operator *(quat q1, float f) { return q1 *= f; }
inline quat operator *(float f, quat q1) { return q1 *= f; }
inline quat operator /(quat q1, float f) { return q1 /= f; }

quat exp(const quat &q);
}
