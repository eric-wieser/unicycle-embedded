Yaw-actuated unicycle - Embedded code
=====================================

Getting started
---------------

This project makes use of [PlatformIO](http://platformio.org/) to compile, upload, and dependency-manage.

For this, it is sufficient to [install PlatformIO core](http://docs.platformio.org/en/latest/installation.html).

`protobuf` is also used here. The `protoc` binary along with the `nanopb` plugin for the arduino can be downloaded [from here](https://koti.kapsi.fi/~jpa/nanopb/download/)

The installation process looks something like this:

* Install python 2.7
* `pip2 install platformio --user`
  * Check this worked by running `platformio`
  * If not, you may need to add a folder to your path. On windows, this folder is `%APPDATA%\Python\Scripts`
* Download and extract the `nanopb` binary build from above, and add `generated-bin/` to your path
  * Check this works by running `protoc` on the command line

From there:

* `platformio run` to compile the code
* `platformio run --target upload` to upload the code
