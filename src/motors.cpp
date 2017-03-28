#include <Arduino.h>
#include <p32_defs.h>
#include "io.h"

// static variables
namespace {
  //output compare
  p32_oc& tmr_tt_fwd = io::oc1;
  p32_oc& tmr_tt_rev = io::oc2;
  p32_oc& tmr_w_fwd = io::oc3;
  p32_oc& tmr_w_rev = io::oc4;

  // timer used for pwm. Note the code below requires it to be "type B"
  p32_timer& motor_tmr = io::tmr2;

  void __attribute__((interrupt)) handlePWMTimer(void) {
    // The timer 2 is currently not in use (doesn't do anything)
    clearIntFlag(io::irq_for(motor_tmr));
  }
}

// function definitions for setting the motor duty cycle
// note that the timers count only to 0xffff, despite the OC being 32 bits
void setMotorTurntable(float cmd) {
  float mag = abs(cmd);
  if(mag > 1) mag = 1;
  uint32_t duty = round(motor_tmr.tmxPr.reg * mag);

  if(cmd < 0) {
    tmr_tt_fwd.ocxRs.reg = 0x0000;
    tmr_tt_rev.ocxRs.reg = duty;
  } else {
    tmr_tt_fwd.ocxRs.reg = duty;
    tmr_tt_rev.ocxRs.reg = 0x0000;
  }
}

void setMotorWheel(float cmd) {
  float mag = abs(cmd);
  if(mag > 1) mag = 1;
  uint32_t duty = round(motor_tmr.tmxPr.reg * mag);

  if(cmd < 0) {
    tmr_w_fwd.ocxRs.reg = 0x0000;
    tmr_w_rev.ocxRs.reg = duty;
  } else {
    tmr_w_fwd.ocxRs.reg = duty;
    tmr_w_rev.ocxRs.reg = 0x0000;
  }
}

void setupMotors() {
  p32_oc* ocs[] = {&tmr_tt_fwd, &tmr_tt_rev, &tmr_w_fwd, &tmr_w_rev};

  for(int i = 0; i < 4; i++) {
    // Turn off the OC when performing the setup
    ocs[i]->ocxCon.reg = 0;

    // Set the primary and secondary compare registers
    ocs[i]->ocxR.reg   = 0x0000;
    ocs[i]->ocxRs.reg  = 0x0000;

    // configure for PWM mode
    ocs[i]->ocxCon.reg = OCCON_SRC_TIMER2 | OCCON_PWM_FAULT_DISABLE;
  }

  // Set period of corresponding timer
  motor_tmr.tmxPr.reg = 0xFFFF;

  // Configure Timer2 interrupt. Note that in PWM mode, the
  // corresponding source timer interrupt flag is asserted.
  // OC interrupt is not generated in PWM mode.
  clearIntFlag(io::irq_for(motor_tmr));
  setIntVector(io::vector_for(motor_tmr), handlePWMTimer);
  setIntPriority(io::vector_for(motor_tmr), 7, 0);
  setIntEnable(io::irq_for(motor_tmr));

  // enable the timer
  motor_tmr.tmxCon.set = TBCON_ON;

  // Enable output compares
  for(int i = 0; i < 4; i++) {
    ocs[i]->ocxCon.set = OCCON_ON;
  }
}

void beep(float freq, int duration){
  static int tot_dur = 0;

  // calculate the period
  uint32_t per = static_cast<uint32_t>(getPeripheralClock() / freq);
  if(per > 0xffff)
    return;

  motor_tmr.tmxPr.reg = per;

  // spin the turntable one way or the other, trying to minimize net movement
  if(tot_dur <= 0) {
    tmr_tt_fwd.ocxRs.reg = 0x0000;
    tmr_tt_rev.ocxRs.reg = 0x1000;
    tot_dur += duration;
  }
  else{
    tmr_tt_fwd.ocxRs.reg = 0x1000;
    tmr_tt_rev.ocxRs.reg = 0x0000;
    tot_dur -= duration;
  }

  // let the beep complete
  delay(duration);

  // turn everything off, reset the period
  tmr_tt_fwd.ocxRs.reg = 0;
  tmr_tt_rev.ocxRs.reg = 0;
  motor_tmr.tmxPr.reg = 0xffff;
}
