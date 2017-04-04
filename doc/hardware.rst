.. default-domain:: cpp

Hardware
========

The controller for the robot is a `chipKIT Max32`_, which is an
arduino-compatible board with a PIC32 microcontroller on board. The specific
microcontroller on the board is a PIC32MX795F512_, which is relevant when
looking for specific hardware features such as timers and PWM.

Power comes from a 7.4V Li-ion battery.

.. _flashing:

Flashing a program
------------------

The trace under ``JP5`` has been deliberately cut on the board. This jumper
connects the ``DTR`` line of the RS232 interface (from the FTDI232RQ_ chip that
tunnels RS232 over USB), to the reset pin of the microcontroller. The flash
programmer normally uses this line to send the chip into the bootloader, so its
code can be changed.

Unfortunately, this line is also unconditionally asserted when plugged into a
computer, making it impossible to attach to a long-running program. The fact
that this is a problem at all is a design flaw in the Max32 - were the ``CTS``
line used instead, there would be no problem.

To reprogram the board then, ``JP5`` must be temporarily closed. This could be
done with a jumper, but this requires that the top layer of circuit board be
removed. In practive, this is best done using a screwdriver to short the two
pins.

.. danger::
   Do not reprogram the board while the battery is attached! Shorting ``JP5``
   is clumsy, and has a risk of shorting other parts of the board. Your USB port
   is `probably protected`__ against this, but the battery may
   do unspeakably bad things.

   .. __: https://superuser.com/questions/1040824/are-usb-ports-in-laptops-protected-against-short-circuits


.. _`chipKIT Max32`: http://store.digilentinc.com/chipkit-max32-microcontroller-board-with-mega-r3-headers/
.. _PIC32MX795F512: http://ww1.microchip.com/downloads/en/DeviceDoc/61156G.pdf

.. _FTDI232RQ: http://www.ftdichip.com/Support/Documents/DataSheets/ICs/DS_FT232R.pdf

Motors
------

.. doxygenfile:: motors.cpp

Sensors
-------


Inertial
~~~~~~~~

.. doxygenfile:: gyroAccel.cpp

Encoders
~~~~~~~~

.. doxygenfile:: encoders.cpp
