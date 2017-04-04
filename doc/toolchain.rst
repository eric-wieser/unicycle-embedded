Toolchain
=========

PlatformIO
----------

To build the code, you'll need to install PlatformIO_. This is used as a
a replacement for the Arduino IDE, which has a number of shortcomings:
 * Poor dependency management - using other people's code is tricky, and
   specifying versions of their code is even harder
 * No command-line interface - downloading code requires clumsy clicking.

The "Core" version of PlatformIO is sufficient, as that gives you the command
line tools.

Once installed, open a terminal and type ``pio run``. This should install all
the libraries we depend on, and compile the code.

To upload the program, run ``pio run -t upload``. See the notes about
:ref:`flashing` for hardware-related issues with this opperation.


Protobuf
--------

To send messages from the robot to the PC, we use protobuf_.

When building the code with ``pio run``, the command ``protoc --nanopb_out ...``
will be executed. This requires that ``protoc``, the protobuf compiler, is on
the path, and that the nanopb_ plugin is also installed.



Documentation
-------------

The HTML version of this documentation is generated with Sphinx_. To generate
the help from C++ comments, both Breathe_ and Doxygen_ are required.

Once everything is installed, typing `make html` in the documentation directory
should rebuild the documentation.

However, this is not necessary - the documentation will be automatically built
by `Read the Docs`_ whenever this repository is pushed to.


.. _Sphinx: https://www.sphinx-doc.org
.. _Breathe: https://breathe.readthedocs.io
.. _Doxygen: https:// www.doxygen.org/
.. _PlatformIO: https://platformio.org/
.. _protobuf: https://developers.google.com/protocol-buffers/
.. _nanopb: https://jpa.kapsi.fi/nanopb/
.. _`Read the Docs`: https://readthedocs.org/
