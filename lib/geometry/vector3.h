#pragma once

#include <math.h>

namespace geometry {

template<typename T>
class Vector3{
public:
  T x;
  T y;
  T z;

  // Assume that the xyz fields are contiguous
  T operator [](int i) const { return (&x)[i];}
  T & operator [](int i) { return (&x)[i];}

  Vector3() {}
  constexpr Vector3(T x, T y, T z) : x(x), y(y), z(z) {}

  static constexpr Vector3<T> Zero() {
    return Vector3<T>(0, 0, 0);
  }

  // conversion to other scalar types
  template<typename U>
  constexpr operator Vector3<U>() {
    return Vector3<U>(x, y, z);
  }

  // basic arithmetic
  template<typename U>
  Vector3<T>& operator +=(Vector3<U> v) {
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
  }
  template<typename U>
  Vector3<T>& operator -=(Vector3<U> v) {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
  }
  template<typename U>
  Vector3<T>& operator *=(U f) {
    x *= f;
    y *= f;
    z *= f;
    return *this;
  }
  template<typename U>
  Vector3<T>& operator /=(U f) {
    x /= f;
    y /= f;
    z /= f;
    return *this;
  }

  // other operations
  T squaredNorm() const {
    return x*x + y*y + z*z;
  }

  void normalize() {
    *this /= sqrt(squaredNorm());
  }

  Vector3<T> normalized() const{
    Vector3<T> res = *this;
    res.normalize();
    return res;
  }
};

template<typename T>
T dot(Vector3<T> a, Vector3<T> b) {
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

template<typename T>
Vector3<T> cross(Vector3<T> a, Vector3<T> b) {
  return Vector3<T>(
    a.y*b.z - b.y*a.z,
    a.z*b.x - b.z*a.x,
    a.x*b.y - b.x*a.y
  );
}


template<typename T>
T declval() noexcept;

template<typename T, typename U,
         typename R = decltype(declval<T>() + declval<U>())>
inline Vector3<R> operator +(Vector3<T> a, const Vector3<U> &b) {
  return static_cast<Vector3<R>>(a) += b;
}
template<typename T, typename U,
         typename R = decltype(declval<T>() - declval<U>())>
inline Vector3<R> operator -(Vector3<T> a, const Vector3<U> &b) {
  return static_cast<Vector3<R>>(a) -= b;
}
template<typename T, typename U,
         typename R = decltype(declval<T>() * declval<U>())>
inline Vector3<R> operator *(Vector3<T> a, const U &f) {
  return static_cast<Vector3<R>>(a) *= f;
}
template<typename T, typename U,
         typename R = decltype(declval<T>() / declval<U>())>
inline Vector3<R> operator /(Vector3<T> a, const U &b) {
  return a /= b;
}

template<typename T, typename U>
inline auto operator *(T f, Vector3<U> a) {
  return a * f;
}

}
