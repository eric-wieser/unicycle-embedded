#pragma once

#include <Arduino.h>
#include "io.h"

/*
 * See section 14 of the PIC32 Family Reference Manual for details of timer
 * operations.
 *
 * This code only works for type A timers, which is why the constant names start
 * with TA.
 */

class CallbackTimer {
private:
    p32_timer& _tmr;
    const int _vector;
    uint16_t _period;
public:
    const int irq;
    /**
     * Create from timer hardware. The timer must be of type A
     */
    constexpr CallbackTimer(p32_timer& tmr)
        : _tmr(&tmr == &io::tmr1 ? tmr :
               io::failed<p32_timer&>("Must be a type A timer")),
          _vector(io::vector_for(tmr)),
          irq(io::irq_for(tmr)),
          _period(0xffff)
    { }

    /** Configure the timer */
    void setup() {
        _tmr.tmxCon.reg = TACON_SRC_INT | TACON_PS_256;
        _tmr.tmxTmr.reg = 0;
        _tmr.tmxPr.reg = _period;
    }

    /** Start the timer. The first callback should fire immediately */
    void start() {
        _tmr.tmxTmr.reg = _period;
        _tmr.tmxCon.set = TACON_ON;
        setIntEnable(irq);
    }

    /** Stop and reset the timer. Callbacks will stop firing */
    void stop() {
        _tmr.tmxCon.clr = TACON_ON;
        clearIntEnable(irq);
    }

    /**
     * Set the function to run when the timer hits its target.
     * This function should be marked __attribute__((interrupt)), and should
     * start with a call to clearIntFlag(timer.irq);
     */
    isrFunc attach(isrFunc f) {
        isrFunc old = setIntVector(_vector, f);
        setIntPriority(_vector, 2, 0);
        return old;
    }

    /** Set the period, in seconds */
    inline void setPeriod(float p) {
        // clock divisor is 256 back in setup
        _tmr.tmxPr.reg = _period = static_cast<uint16_t>(p * F_CPU / 256);
    }
};
