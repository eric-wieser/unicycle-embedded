#ifndef QUAT
#define QUAT

class quat
{
public:
  float x, y, z, w;
  quat(float xx, float yy, float zz, float ww);
  void normalize();
  quat conj() const;
  quat operator* (const quat &q) const;
  void euler(float &phi, float &theta, float &psi) const;
};

#endif
