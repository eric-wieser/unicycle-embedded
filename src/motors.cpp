#include <Arduino.h>
#include <p32_defs.h>
#include "io.h"

#include "io.h"

  // timer used for pwm. Note the code below requires it to be "type B"
namespace {
  p32_timer& motor_tmr = io::tmr2;
}

class Timer2Motor {
private:
  p32_oc& oc_fwd;
  p32_oc& oc_rev;

  static void __attribute__((interrupt)) handlePWMTimer(void) {
    // The timer 2 is currently not in use (doesn't do anything)
    clearIntFlag(io::irq_for(motor_tmr));
  }

public:

  static void setup_timer() {
    // Set period of corresponding timer
    motor_tmr.tmxPr.reg = 0xFFFF;

    // Configure Timer2 interrupt. Note that in PWM mode, the
    // corresponding source timer interrupt flag is asserted.
    // OC interrupt is not generated in PWM mode.
    clearIntFlag(io::irq_for(motor_tmr));
    setIntVector(io::vector_for(motor_tmr), Timer2Motor::handlePWMTimer);
    setIntPriority(io::vector_for(motor_tmr), 7, 0);
    setIntEnable(io::irq_for(motor_tmr));

    // enable the timer
    motor_tmr.tmxCon.set = TBCON_ON;
  }

  Timer2Motor(p32_oc& fwd, p32_oc& rev) : oc_fwd(fwd), oc_rev(rev) { }

  void setup() {
    p32_oc* ocs[] = {&oc_fwd, &oc_rev};

    for(auto &oc : ocs) {
      // Turn off the OC when performing the setup
      oc->ocxCon.reg = 0;

      // Set the primary and secondary compare registers
      oc->ocxR.reg   = 0x0000;
      oc->ocxRs.reg  = 0x0000;

      // configure for PWM mode
      oc->ocxCon.reg = OCCON_SRC_TIMER2 | OCCON_PWM_FAULT_DISABLE;
    }

  }

  void set(float value) {
    float mag = abs(value);
    if (mag > 1) mag = 1;
    uint32_t duty = round(motor_tmr.tmxPr.reg * mag);

    if (value < 0) {
      oc_fwd.ocxRs.reg = 0x0000;
      oc_rev.ocxRs.reg = duty;
    } else {
      oc_fwd.ocxRs.reg = duty;
      oc_rev.ocxRs.reg = 0x0000;
    }
  }

  void enable() {
    oc_fwd.ocxCon.set = OCCON_ON;
    oc_rev.ocxCon.set = OCCON_ON;
  }
};

// static variables
namespace {
  Timer2Motor motor_turntable(io::oc1, io::oc2);
  Timer2Motor motor_wheel(io::oc3, io::oc4);
}

// function definitions for setting the motor duty cycle
// note that the timers count only to 0xffff, despite the OC being 32 bits
void setMotorTurntable(float cmd) {
  motor_turntable.set(cmd);
}

void setMotorWheel(float cmd) {
  motor_wheel.set(cmd);
}

void setupMotors() {
  motor_wheel.setup();
  motor_turntable.setup();
  Timer2Motor::setup_timer();
  motor_wheel.enable();
  motor_turntable.enable();
}

void beep(float freq, int duration){
  using namespace io;
  static int tot_dur = 0;

  // calculate the period
  uint32_t per = static_cast<uint32_t>(getPeripheralClock() / freq);
  if(per > 0xffff)
    return;

  const uint32_t old_per = motor_tmr.tmxPr.reg;
  motor_tmr.tmxPr.reg = per;

  // spin the turntable one way or the other, trying to minimize net movement
  if(tot_dur <= 0) {
    oc1.ocxRs.reg = 0x0000;
    oc2.ocxRs.reg = 0x1000;
    tot_dur += duration;
  }
  else{
    oc1.ocxRs.reg = 0x1000;
    oc2.ocxRs.reg = 0x0000;
    tot_dur -= duration;
  }

  // let the beep complete
  delay(duration);

  // turn everything off, reset the period
  oc1.ocxRs.reg = 0;
  oc2.ocxRs.reg = 0;
  motor_tmr.tmxPr.reg = old_per;
}
