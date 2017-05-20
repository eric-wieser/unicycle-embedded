#pragma once

#include "quat.h"

namespace geometry {

template<int order>
struct euler_angles {
  // the meaning of these varies depending on the order
  float phi;
  float theta;
  float psi;
  euler_angles() {}
  constexpr euler_angles(float phi, float theta, float psi)
    : phi(phi), theta(theta), psi(psi) {}
  euler_angles(const quat &q);

  operator quat() const;

  static quat remove_psi(const quat &q);
};


template<>
euler_angles<123>::euler_angles(const quat &q);

template<>
euler_angles<213>::euler_angles(const quat &q);

template<>
euler_angles<213>::operator quat() const;


}