/**
 * This might seem too simple for its own file, but it makes it easier if we
 * change the polarity of the pin, and want a uniform wait duration for
 * detecting presses
 */
#include "button.h"

#include <Arduino.h>

#include "io.h"
#include "pins.h"

namespace button {
  namespace {
    uint32_t last_changed = 0;
    bool last_pressed = 0;
  }

  void setup() {
    pinMode(pins::SWITCH, INPUT_PULLUP);
  }

  /**
   * Determine if the button is pressed, with debouncing
   *
   * This detects the first edge instanltly, but delays subsequent edges to
   * occur at least DEBOUNCE_TIME milliseconds later.
   */
  bool isPressed() {
    uint32_t now = millis();
    bool pressed = digitalRead(pins::SWITCH);
    uint32_t debounce_time = last_pressed ? PRESS_DEBOUNCE_TIME : RELEASE_DEBOUNCE_TIME;

    if (last_changed == 0 || (now > last_changed + debounce_time && pressed != last_pressed)) {
      last_pressed = pressed;
      last_changed = now;
    }
    return last_pressed;
  }
}
