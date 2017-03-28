#pragma once

#include <p32_defs.h>
#include "chipkit_patch.h"

// Declaring these static means that there is a copy of each variable in every
// file. However, this isn't a problem, and allows the compiler to optimize
// accesses to them.

namespace io {
  // Output compare
  static p32_oc& oc1 = *reinterpret_cast<p32_oc*>(_OCMP1_BASE_ADDRESS);
  static p32_oc& oc2 = *reinterpret_cast<p32_oc*>(_OCMP2_BASE_ADDRESS);
  static p32_oc& oc3 = *reinterpret_cast<p32_oc*>(_OCMP3_BASE_ADDRESS);
  static p32_oc& oc4 = *reinterpret_cast<p32_oc*>(_OCMP4_BASE_ADDRESS);

  // Type A timers
  static p32_timer& tmr1 = *reinterpret_cast<p32_timer*>(_TMR1_BASE_ADDRESS);
  static p32_timer& tmr2 = *reinterpret_cast<p32_timer*>(_TMR2_BASE_ADDRESS);

  // Type B timers
  static p32_timer& tmr3 = *reinterpret_cast<p32_timer*>(_TMR3_BASE_ADDRESS);
  static p32_timer& tmr4 = *reinterpret_cast<p32_timer*>(_TMR4_BASE_ADDRESS);

  // use I2C1
  static p32_i2c& i2c1 = *reinterpret_cast<p32_i2c*>(_I2C1_BASE_ADDRESS);

  //change notifier
  static p32_cn& cn = *reinterpret_cast<p32_cn*>(_CN_BASE_ADDRESS);

  // object to interrupt number conversion
  inline int irq_for(const p32_timer& tmr) {
    if (&tmr == &tmr1) return _TIMER_1_IRQ;
    if (&tmr == &tmr2) return _TIMER_2_IRQ;
    if (&tmr == &tmr3) return _TIMER_3_IRQ;
    if (&tmr == &tmr4) return _TIMER_4_IRQ;
    while(1);
  }
  inline int irq_for(const p32_cn& cn) {
    return _CHANGE_NOTICE_IRQ;
  }

  // object to vector number conversion
  inline int vector_for(const p32_timer& tmr) {
    if (&tmr == &tmr1) return _TIMER_1_VECTOR;
    if (&tmr == &tmr2) return _TIMER_2_VECTOR;
    if (&tmr == &tmr3) return _TIMER_3_VECTOR;
    if (&tmr == &tmr4) return _TIMER_4_VECTOR;
    while(1);
  }
  inline int vector_for(const p32_cn& cn) {
    return _CHANGE_NOTICE_VECTOR;
  }
}
