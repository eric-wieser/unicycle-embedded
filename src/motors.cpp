#include <Arduino.h>
#include <p32_defs.h>

// static variables
namespace {
  //output compare
  volatile p32_oc& oc1 = *reinterpret_cast<volatile p32_oc*>(_OCMP1_BASE_ADDRESS);
  volatile p32_oc& oc2 = *reinterpret_cast<volatile p32_oc*>(_OCMP2_BASE_ADDRESS);
  volatile p32_oc& oc3 = *reinterpret_cast<volatile p32_oc*>(_OCMP3_BASE_ADDRESS);
  volatile p32_oc& oc4 = *reinterpret_cast<volatile p32_oc*>(_OCMP4_BASE_ADDRESS);

  // timer used for pwm
  volatile p32_timer& tmr2 = *reinterpret_cast<volatile p32_timer*>(_TMR2_BASE_ADDRESS);

  void __attribute__((interrupt)) handlePWMTimer(void) {
    // The timer 2 is currently not in use (doesn't do anything)
    clearIntFlag(_TIMER_2_IRQ);
  }
}

// function definitions for setting the motor duty cycle
// note that the timers count only to 0xffff, despite the OC being 32 bits
void setMotorTurntable(float cmd) {
  float mag = abs(cmd);
  if(mag > 1) mag = 1;
  uint32_t duty = round(tmr2.tmxPr.reg * mag);

  if(cmd < 0) {
    oc1.ocxRs.reg = 0x0000;
    oc2.ocxRs.reg = duty;
  } else {
    oc1.ocxRs.reg = duty;
    oc2.ocxRs.reg = 0x0000;
  }
}

void setMotorWheel(float cmd) {
  float mag = abs(cmd);
  if(mag > 1) mag = 1;
  uint32_t duty = round(tmr2.tmxPr.reg * mag);

  if(cmd < 0) {
    oc3.ocxRs.reg = 0x0000;
    oc4.ocxRs.reg = duty;
  } else {
    oc3.ocxRs.reg = duty;
    oc4.ocxRs.reg = 0x0000;
  }
}

void setupMotors() {
  volatile p32_oc* ocs[] = {&oc1, &oc2, &oc3, &oc4};

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
  tmr2.tmxPr.reg = 0xFFFF;

  // Configure Timer2 interrupt. Note that in PWM mode, the
  // corresponding source timer interrupt flag is asserted.
  // OC interrupt is not generated in PWM mode.
  clearIntFlag(_TIMER_2_IRQ);
  setIntVector(_TIMER_2_VECTOR, handlePWMTimer);
  setIntPriority(_TIMER_2_VECTOR, 7, 0);
  setIntEnable(_TIMER_2_IRQ);

  // enable TMR2
  tmr2.tmxCon.set = TBCON_ON;

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

  tmr2.tmxPr.reg = per;

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
  tmr2.tmxPr.reg = 0xffff;
}
