#include "euler.h"
#include "quat.h"
#include "math.h"

using namespace geometry;

// https://www.astro.rug.nl/software/kapteyn/_downloads/attitude.pdf#page=24
template<>
euler_angles<123>::euler_angles(const quat &q) {
  float x = q.x, y = q.y, z = q.z, w = q.w;

  phi   = atan2(2*z*w + 2*x*y, w*w -z*z -y*y +x*x);
  theta = -asin(2*y*w - 2*x*z);
  psi   = atan2(2*y*z + 2*x*w, y*y +x*x -w*w -z*z);
}

// https://www.astro.rug.nl/software/kapteyn/_downloads/attitude.pdf#page=28
template<>
euler_angles<213>::euler_angles(const quat &q) {
  float x = q.x, y = q.y, z = q.z, w = q.w;

  phi   = atan2(-2*w*y + 2*x*z, x*x -y*y -z*z +w*w);
  theta = asin(  2*z*w + 2*x*y);
  psi   = atan2(-2*y*z + 2*x*w, x*x -y*y +z*z -w*w);
}

template<>
euler_angles<213>::operator quat() const {
	return
		quat(cos(phi   / 2), 0,              sin(phi   / 2), 0) *
		quat(cos(theta / 2), sin(theta / 2), 0,              0) *
		quat(cos(psi   / 2), 0,              0,              sin(psi / 2));
}