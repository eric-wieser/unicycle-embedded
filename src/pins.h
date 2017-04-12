#pragma once


/* These are mostly for documentation, and so we know which
pins are already in use
*/
namespace pins {
	// encoders
	static const uint8_t TT_DIR = 39;
	static const uint8_t W_DIR = 47;
	static const uint8_t TMR3_CLK = 22;
	static const uint8_t TMR4_CLK = 23;

	// sensor(s)
	static const uint8_t I2C1_SDA = 20;
	static const uint8_t I2C1_SCL = 21;

	// motors
	static const uint8_t OC1_PWM = 3;
	static const uint8_t OC2_PWM = 5;
	static const uint8_t OC3_PWM = 6;
	static const uint8_t OC4_PWM = 9;

	// human interaction
	static const uint8_t SWITCH = 43;
	static const uint8_t LED    = PIN_LED1;
	static const uint8_t USB_RX = 0;
	static const uint8_t USB_TX = 1;
}
