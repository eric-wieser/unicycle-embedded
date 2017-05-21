#pragma once

#include <stdint.h>

namespace button {
	// time after a press before a release can occur
	constexpr uint32_t PRESS_DEBOUNCE_TIME = 50;
	// time after a release before a press can occur
	constexpr uint32_t RELEASE_DEBOUNCE_TIME = 150;
	void setup();
	bool isPressed();
}
