Sensors
=======

The robot has a combined accelerometer and gyro board that is `sold by
Sparkfun`. The accelerometer is an ADXL345_, and the gyroscope is an `ITG-3200`_.

Both of these sensors use the I2C protocol - in fact, they share a bus.
Unfortunately, the builtin arduino Wire_ interface that implements this protocol
does not appear to work on our microcontroller board.

As a result, we have to work in terms of lower-level apis, which are described in
the `I2C Helpers`_ section

.. doxygenfile:: gyroAccel.cpp

I2C helpers
-----------

.. _`sold by Sparkfun`: https://www.sparkfun.com/products/10121
.. _ADXL345: https://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf
.. _`ITG-3200`: https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf
.. _Wire: https://www.arduino.cc/en/reference/wire
