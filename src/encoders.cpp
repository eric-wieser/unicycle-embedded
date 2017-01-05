#include <Arduino.h>
#include <p32_defs.h>
#include "chipkit_patch.h"

// pin definitions
const uint8_t TT_DIR_PIN = 39;
const uint8_t W_DIR_PIN = 47;

// static variables
namespace {
  // Type B timers
  volatile p32_timer& tmr3 = *reinterpret_cast<volatile p32_timer*>(_TMR3_BASE_ADDRESS);
  volatile p32_timer& tmr4 = *reinterpret_cast<volatile p32_timer*>(_TMR4_BASE_ADDRESS);

  //change notifier
  volatile p32_cn& cn = *reinterpret_cast<volatile p32_cn*>(_CN_BASE_ADDRESS);

  volatile int TMR3sign = 1;             // sign of the value in TMR3
  volatile int TMR4sign = 1;             // sign of the value in TMR4

  void __attribute__((interrupt)) handleEncoderSignChange(void) {
    // This interrupt handler is for keeping track of the TMR3&4 directions, flagged if direction changes
    // They are counters to keep track of the two motor angles, so if direction changes,
    // the counters need to count down rather than up

    // convert pin numbers to ports - should be optimized out
    const uint8_t w_port = digitalPinToPort(W_DIR_PIN);
    const uint8_t w_mask = digitalPinToBitMask(W_DIR_PIN);
    const uint8_t tt_port = digitalPinToPort(TT_DIR_PIN);
    const uint8_t tt_mask = digitalPinToBitMask(TT_DIR_PIN);

    // timing is everything, so do just one read if possible
    bool wNeg, ttNeg;
    if(w_port == tt_port) {
      const uint8_t reading = portRegisters(w_port)->port.reg;
      wNeg = reading & w_mask;
      ttNeg = reading & tt_mask;
    }
    else {
      wNeg = portRegisters(w_port)->port.reg & w_mask;
      ttNeg = portRegisters(tt_mask)->port.reg & tt_mask;
    }

    clearIntFlag(_CHANGE_NOTICE_IRQ);

    const int ttSign = ttNeg ? -1 : 1;
    const int wSign  = wNeg ? -1 : 1;

    if (TMR3sign != ttSign) {
      tmr3.tmxTmr.reg = (short int) -tmr3.tmxTmr.reg;
    }
    TMR3sign = ttSign;

    if (TMR4sign != wSign) {
      tmr4.tmxTmr.reg = (short int) -tmr4.tmxTmr.reg;
    }
    TMR4sign = wSign;
  }
}

void setupEncoders() {
  // configure the change notice to watch the encoder pins
  cn.cnCon.clr = 0xFFFF;
  cn.cnCon.reg = CNCON_ON | CNCON_IDLE_RUN;
  cn.cnEn.reg = digitalPinToCN(TT_DIR_PIN) | digitalPinToCN(W_DIR_PIN);
  cn.cnPue.reg = 0;

  clearIntFlag(_CHANGE_NOTICE_IRQ);
  setIntVector(_CHANGE_NOTICE_VECTOR, handleEncoderSignChange);
  setIntPriority(_CHANGE_NOTICE_VECTOR, 2, 0); //should this be priority 2?
  setIntEnable(_CHANGE_NOTICE_IRQ);

  // start the encoder timers
  // T3 external source is the pulse from the turntable
  tmr3.tmxCon.reg = TBCON_SRC_EXT | TBCON_PS_1;
  tmr3.tmxTmr.reg = 0;
  tmr3.tmxPr.reg = 0xffff;
  tmr3.tmxCon.set = TBCON_ON;

  // T4 external source is the pulse from the wheel
  tmr4.tmxCon.reg = TBCON_SRC_EXT | TBCON_PS_1;
  tmr4.tmxTmr.reg = 0;
  tmr4.tmxPr.reg = 0xffff;
  tmr4.tmxCon.set = TBCON_ON;
}

int16_t getTTangle() {
  return TMR3sign * static_cast<int16_t>(tmr3.tmxTmr.reg);
}

int16_t getWangle() {
  return TMR4sign * static_cast<int16_t>(tmr4.tmxTmr.reg);
}
