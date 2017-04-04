/*!
\file encoders.cpp

\rst

The robot has an encoder on each motor. These go via some circuitry on the
board that convert them into two lines - a tick, and a direction.

We count the ticks using the builtin hardware timers, but in order to deal with
the direction reversing, we have to monitor the direction pin. We use the
"change notifier" hardware to fire an interrupt whenever these pins change, and
correct the sign accordingly.

Each encoder is a `Maxon 201937`_, "Encoder MR, Type M, 512 CPT, 2 Channels,
with Line Driver".

.. _`Maxon 201937`: http://www.maxonmotor.com/maxon/view/product/201937

\endrst
*/

#include <Arduino.h>
#include <io.h>
#include <pins.h>
#include <messaging.h>
#include <gpio.h>

// static variables
namespace {

  // change notifier
  p32_cn& cn = io::cn;

  // the timers that the encoders are wired to
  class RollingTimer {
  private:
    volatile int last_sign;
    p32_timer& tmr;

  public:
    gpio::Pin dir_pin;
    RollingTimer(p32_timer& tmr, uint8_t pin)
      : tmr(tmr), last_sign(1), dir_pin(pin) {}

    void setup() {
      tmr.tmxCon.reg = TBCON_SRC_EXT | TBCON_PS_1;
      tmr.tmxTmr.reg = 0;
      tmr.tmxPr.reg = 0xffff;
      tmr.tmxCon.set = TBCON_ON;
    }

    void update(bool negative) {
      // if the timer changed direction, replace it with its negative
      const int sign = negative ? -1 : 1;
      if (last_sign != sign) {
        tmr.tmxTmr.reg = (short int) -tmr.tmxTmr.reg;
      }
      last_sign = sign;
    }

    inline int16_t read() {
      return last_sign * static_cast<int16_t>(tmr.tmxTmr.reg);
    }

    void reset() {
      // must only be called with CN disabled
      last_sign = 1;
      tmr.tmxTmr.reg = 0;
    }
  };

  RollingTimer tt_timer(io::tmr3, pins::TT_DIR);
  RollingTimer w_timer(io::tmr4, pins::W_DIR);

  void __attribute__((interrupt)) handleEncoderSignChange(void) {
    // This interrupt handler is for keeping track of the TMR3&4 directions, flagged if direction changes
    // They are counters to keep track of the two motor angles, so if direction changes,
    // the counters need to count down rather than up

    gpio::CachedReader reader;
    bool w_neg = reader.read(w_timer.dir_pin);
    bool tt_neg = reader.read(tt_timer.dir_pin);

    // only reenable the interrupt after we've read the values
    clearIntFlag(_CHANGE_NOTICE_IRQ);

    w_timer.update(w_neg);
    tt_timer.update(tt_neg);
  }
}
//! Initialize the hardware required by the encoders
void setupEncoders() {
  // configure the change notice to watch the encoder pins
  cn.cnCon.clr = 0xFFFF;
  cn.cnCon.reg = CNCON_ON | CNCON_IDLE_RUN;
  cn.cnEn.reg = digitalPinToCN(w_timer.dir_pin)
              | digitalPinToCN(tt_timer.dir_pin);
  cn.cnPue.reg = 0;

  clearIntFlag(io::irq_for(cn));
  setIntVector(io::vector_for(cn), handleEncoderSignChange);
  setIntPriority(io::vector_for(cn), 2, 0); //should this be priority 2?
  setIntEnable(io::irq_for(cn));

  // start the encoder timers
  w_timer.setup();
  tt_timer.setup();
}

//! Reset the counts of the encoders
void resetEncoders() {
  clearIntEnable(_CHANGE_NOTICE_IRQ);
  w_timer.reset();
  tt_timer.reset();
  setIntEnable(_CHANGE_NOTICE_IRQ);
}

//! Get the angle of the turntable, in encoder ticks
int16_t getTTangle() {
  return tt_timer.read();
}

//! Get the angle of the wheel, in encoder ticks
int16_t getWangle() {
  return w_timer.read();
}
