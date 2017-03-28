#include <Arduino.h>
#include <io.h>
#include <messaging.h>

// pin definitions
const uint8_t TT_DIR_PIN = 39;
const uint8_t W_DIR_PIN = 47;

// static variables
namespace {
  // Type B timers
  p32_timer& tt_tmr = io::tmr3;
  p32_timer& w_tmr = io::tmr4;

  // change notifier
  p32_cn& cn = io::cn;

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

    clearIntFlag(io::irq_for(cn));

    const int ttSign = ttNeg ? -1 : 1;
    const int wSign  = wNeg ? -1 : 1;

    if (TMR3sign != ttSign) {
      tt_tmr.tmxTmr.reg = (short int) -tt_tmr.tmxTmr.reg;
    }
    TMR3sign = ttSign;

    if (TMR4sign != wSign) {
      w_tmr.tmxTmr.reg = (short int) -w_tmr.tmxTmr.reg;
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

  clearIntFlag(io::irq_for(cn));
  setIntVector(io::vector_for(cn), handleEncoderSignChange);
  setIntPriority(io::vector_for(cn), 2, 0); //should this be priority 2?
  setIntEnable(io::irq_for(cn));

  // start the encoder timers
  // T3 external source is the pulse from the turntable
  tt_tmr.tmxCon.reg = TBCON_SRC_EXT | TBCON_PS_1;
  tt_tmr.tmxTmr.reg = 0;
  tt_tmr.tmxPr.reg = 0xffff;
  tt_tmr.tmxCon.set = TBCON_ON;

  // T4 external source is the pulse from the wheel
  w_tmr.tmxCon.reg = TBCON_SRC_EXT | TBCON_PS_1;
  w_tmr.tmxTmr.reg = 0;
  w_tmr.tmxPr.reg = 0xffff;
  w_tmr.tmxCon.set = TBCON_ON;
}

void resetEncoders() {
  // clearIntEnable(io::irq_for(cn));
  // // TMR3sign = 1;
  // // TMR4sign = 1;
  // // w_tmr.tmxTmr.reg = 0;
  // // tt_tmr.tmxTmr.reg = 0;
  // // setIntEnable(io::irq_for(cn));
  // // 
  // setupEncoders();
}

int16_t getTTangle() {
  return TMR3sign * static_cast<int16_t>(tt_tmr.tmxTmr.reg);
}

int16_t getWangle() {
  return TMR4sign * static_cast<int16_t>(w_tmr.tmxTmr.reg);
}
