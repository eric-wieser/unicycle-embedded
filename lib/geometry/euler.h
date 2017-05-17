#pragma once


namespace geometry {

class quat;

template<int order>
struct euler_angles {
  // the meaning of these varies depending on the order
  float phi;
  float theta;
  float psi;
  euler_angles() {}
  euler_angles(float phi, float theta, float psi)
    : phi(phi), theta(theta), psi(psi) {}
  euler_angles(const quat &q);
};


template<>
euler_angles<123>::euler_angles(const quat &q);

template<>
euler_angles<213>::euler_angles(const quat &q);

}