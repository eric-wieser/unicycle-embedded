#pragma once


/*
 * These are mostly for documentation, and so we know which
 * pins are already in use.
*/
namespace pins
{
	/** @name Encoders */
	static const uint8_t TT_DIR = 39;
	static const uint8_t W_DIR  = 47;
	static const uint8_t TT_CLK = 22;
	static const uint8_t W_CLK  = 23;
	/** @} */

	/** @name I2C Sensors */
	static const uint8_t IMU_SCL = 21;
	static const uint8_t IMU_SDA = 20;
	/** @} */

	/** @name Motors */
	static const uint8_t TT_FWD = 3;
	static const uint8_t TT_REV = 5;
	static const uint8_t W_FWD  = 6;
	static const uint8_t W_REV  = 9;
	/** @} */

	/** @name Human interaction */
	static const uint8_t SWITCH = 43; //!< Input for the limit switch
	static const uint8_t LED    = PIN_LED1;
	static const uint8_t USB_RX = 0;
	static const uint8_t USB_TX = 1;
	/** @} */
}
