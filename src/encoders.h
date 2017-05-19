#pragma once

#include <stdint.h>

// poor-man's #include <type_traits>
namespace std {
  template<typename T> struct make_signed;
  template<> struct make_signed<uint8_t> { using type = int8_t; };
  template<> struct make_signed<uint16_t> { using type = int16_t; };
  template<> struct make_signed<uint32_t> { using type = int32_t; };
}

//! helper class that encapsulates wraparound subtraction. Note in particular
//! that the data cannot be used at all without performing subtraction.
template<typename T>
class wrapping {
private:
  T val;
public:
  typedef typename std::make_signed<T>::type diff_type;
  constexpr wrapping(T val=0) : val(val) {}
  constexpr diff_type operator -(wrapping<T> b) const {
    return val - b.val;
  }
  constexpr wrapping<T> operator -() const {
    return -val;
  }
};

void setupEncoders();
void resetEncoders();

wrapping<uint16_t> getTTangle();
wrapping<uint16_t> getWangle();
