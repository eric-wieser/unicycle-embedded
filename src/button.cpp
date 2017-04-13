/**
 * This might seem too simple for its own file, but it makes it easier if we
 * change the polarity of the pin, and want a uniform wait duration for
 * detecting presses
 */

#include <Arduino.h>

#include "io.h"
#include "pins.h"

namespace button {
  void setup() {
    pinMode(pins::SWITCH, INPUT_PULLUP);
  }

  bool isPressed() {
    return digitalRead(pins::SWITCH);
  }

  void awaitPress() {
    while (!isPressed());
    delay(50);
    while (isPressed());
  }
}
