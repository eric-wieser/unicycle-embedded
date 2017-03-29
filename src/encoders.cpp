#include <Arduino.h>
#include <io.h>
#include <pins.h>
#include <messaging.h>
#include <gpio.h>

// pin definitions
const uint8_t TT_DIR_PIN = 39;
const uint8_t W_DIR_PIN = 47;

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

void setupEncoders() {
  // configure the change notice to watch the encoder pins
  cn.cnCon.clr = 0xFFFF;
  cn.cnCon.reg = CNCON_ON | CNCON_IDLE_RUN;
  cn.cnEn.reg = digitalPinToCN(TT_DIR_PIN) | digitalPinToCN(W_DIR_PIN);
  cn.cnPue.reg = 0;

  clearIntFlag(io::irq_for(cn));
  setIntVector(io::vector_for(cn), handleEncoderSignChange);
  setIntPriority(io::vector_for(cn), 2, 0); //should this be priority 2?
  setIntEnable(io::irq_for(cn));

  // start the encoder timers
  w_timer.setup();
  tt_timer.setup();
}

void resetEncoders() {
  clearIntEnable(_CHANGE_NOTICE_IRQ);
  w_timer.reset();
  tt_timer.reset();
  setIntEnable(_CHANGE_NOTICE_IRQ);
}

int16_t getTTangle() {
  return tt_timer.read();
}

int16_t getWangle() {
  return w_timer.read();
}
