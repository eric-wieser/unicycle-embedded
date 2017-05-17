#pragma once

#include "euler.h"
#include "vector3.h"

namespace geometry {

class quat
{
public:
  float x, y, z, w;
  quat(float xx, float yy, float zz, float ww);
  quat(Vector3<float> v);
  quat(float k);
  void normalize();
  quat conj() const;
  quat operator* (const quat &q) const;
  quat operator* (float f) const;
  quat operator/ (float f) const;
  euler_angle euler() const;
};

}
