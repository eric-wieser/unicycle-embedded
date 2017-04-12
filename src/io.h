#pragma once

#include <p32_defs.h>
#include "chipkit_patch.h"

namespace io {

  namespace {
    //! Helper function for when an invalid argument is supplied
    template<typename T>
    T failed(const char*) { while(1); }
  }

  /**
   * @name Devices
   * These variables just turn the long `#define`d names into C++ references of
   * the approapriate types
   * @{ */
  // Output compare
  static constexpr p32_oc& oc1 = *reinterpret_cast<p32_oc*>(_OCMP1_BASE_ADDRESS);
  static constexpr p32_oc& oc2 = *reinterpret_cast<p32_oc*>(_OCMP2_BASE_ADDRESS);
  static constexpr p32_oc& oc3 = *reinterpret_cast<p32_oc*>(_OCMP3_BASE_ADDRESS);
  static constexpr p32_oc& oc4 = *reinterpret_cast<p32_oc*>(_OCMP4_BASE_ADDRESS);

  // Type A timers
  static constexpr p32_timer& tmr1 = *reinterpret_cast<p32_timer*>(_TMR1_BASE_ADDRESS);

  // Type B timers
  static constexpr p32_timer& tmr2 = *reinterpret_cast<p32_timer*>(_TMR2_BASE_ADDRESS);
  static constexpr p32_timer& tmr3 = *reinterpret_cast<p32_timer*>(_TMR3_BASE_ADDRESS);
  static constexpr p32_timer& tmr4 = *reinterpret_cast<p32_timer*>(_TMR4_BASE_ADDRESS);
  static constexpr p32_timer& tmr5 = *reinterpret_cast<p32_timer*>(_TMR5_BASE_ADDRESS);

  // I2C
  static constexpr p32_i2c& i2c1 = *reinterpret_cast<p32_i2c*>(_I2C1_BASE_ADDRESS);
  static constexpr p32_i2c& i2c2 = *reinterpret_cast<p32_i2c*>(_I2C1_BASE_ADDRESS);

  // change notifier
  static constexpr p32_cn& cn = *reinterpret_cast<p32_cn*>(_CN_BASE_ADDRESS);
  /** @} */

  /**
   * @name Helpers
   * These functions allow conversion between devices and relevant
   * configuation needed to use them elsewhere. These prevent a device having
   * to be referred to by name in more than one place.
   *
   * These should all be evaluated at compile time!
   * @{ */
  //! Get the interrupt bit number for a given timer
  constexpr int irq_for(const p32_timer& tmr) {
    return &tmr == &tmr1 ? _TIMER_1_IRQ :
           &tmr == &tmr2 ? _TIMER_2_IRQ :
           &tmr == &tmr3 ? _TIMER_3_IRQ :
           &tmr == &tmr4 ? _TIMER_4_IRQ :
           &tmr == &tmr5 ? _TIMER_5_IRQ :
           failed<int>("Timer does not exist");
  }
  //! Get the interrupt bit number for the change notice hardware
  constexpr int irq_for(const p32_cn& cn) {
    return _CHANGE_NOTICE_IRQ;
  }

  //! Get the interrupt vector number for a given timer
  constexpr int vector_for(const p32_timer& tmr) {
    return &tmr == &tmr1 ? _TIMER_1_VECTOR :
           &tmr == &tmr2 ? _TIMER_2_VECTOR :
           &tmr == &tmr3 ? _TIMER_3_VECTOR :
           &tmr == &tmr4 ? _TIMER_4_VECTOR :
           &tmr == &tmr5 ? _TIMER_5_VECTOR :
           failed<int>("Timer does not exist");
  }
  //! Get the interrupt vector number for the change notice hardare
  constexpr int vector_for(const p32_cn& cn) {
    return _CHANGE_NOTICE_VECTOR;
  }

  //! Get the timer connected to a given CK (clock) pin
  constexpr p32_timer& timer_for(uint8_t pin) {
    return pin == 4  ? tmr1 :
           pin == 22 ? tmr2 :
           pin == 23 ? tmr4 :
           pin == 11 ? tmr5 :
           failed<p32_timer&>("Timer does not exist");
  }

  //! Get the output compare (PWM) connected to a given pin
  constexpr p32_oc& oc_for(uint8_t pin) {
    return pin == 3 ? oc1 :
           pin == 5 ? oc2 :
           pin == 6 ? oc3 :
           pin == 9 ? oc4 :
           failed<p32_oc&>("Output compare does not exist");
  }

  /** @} */

  /**
   * @name Compile-time Helpers
   * Like the above functions, but with arguments re-expresed as template
   * parameters to force errors at compile time. These make it impossible to
   * choose an invalid pin.
   * @{
   */
  template <uint8_t pin>
  p32_timer& timer_for() {
    constexpr auto& res = timer_for(pin);
    return res;
  }
  template <uint8_t pin>
  p32_oc& oc_for() {
    constexpr auto& res = oc_for(pin);
    return res;
  }
  /** @} */
}

#undef _return_constexpr
