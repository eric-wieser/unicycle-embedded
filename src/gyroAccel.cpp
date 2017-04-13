/*!
\file gyroAccel.cpp

\rst

  The robot has a combined accelerometer and gyro board that is `sold by
  Sparkfun`_. The accelerometer is an ADXL345_, and the gyroscope is an
  `ITG-3200`_.

  Both of these sensors use the I2C protocol - in fact, they share a bus.
  Unfortunately, the builtin arduino Wire_ interface that implements this
  protocol does not appear to work on our microcontroller board.

  .. _`sold by Sparkfun`: https://www.sparkfun.com/products/10121
  .. _ADXL345: https://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf
  .. _`ITG-3200`: https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf
  .. _Wire: https://www.arduino.cc/en/reference/wire

\endrst
*/
#include "gyroAccel.h"
#include "i2c_funcs.h"

#include <Arduino.h>

using namespace i2c_funcs;

#include "io.h"
#include "pins.h"

//! Internal helpers
namespace {

  // choose which I2C pins to use
  constexpr p32_i2c& i2c = io::i2c_for(pins::IMU_SCL, pins::IMU_SDA);

  //! Write one data byte to a specified register at I2C address
  void I2CWrite(uint8_t addr, uint8_t reg, uint8_t data)
  {
    StartI2C(i2c); IdleI2C(i2c);             // send start condition
    MasterWriteI2C(i2c, addr); IdleI2C(i2c); // I2C write address
    MasterWriteI2C(i2c, reg); IdleI2C(i2c);  // send register
    MasterWriteI2C(i2c, data); IdleI2C(i2c); // send data
    StopI2C(i2c); IdleI2C(i2c);              // send stop condition
  }

  //! Read length bytes to data array from I2C addr, starting at specified reg
  void I2CRead(uint8_t addr, uint8_t reg, uint8_t *data, size_t length)
  {
    StartI2C(i2c); IdleI2C(i2c);                 // send start condition
    MasterWriteI2C(i2c, addr); IdleI2C(i2c);     // I2C write address
    MasterWriteI2C(i2c, reg); IdleI2C(i2c);      // send start register
    StartI2C(i2c); IdleI2C(i2c);                 // send start condition
    MasterWriteI2C(i2c, addr | 1); IdleI2C(i2c); // I2C read address
    MastergetsI2C(i2c, length, data, 2000);       // get data
    IdleI2C(i2c);
    StopI2C(i2c); IdleI2C(i2c);    // send stop condition
  }

  // write one data byte to specified accelerometer register
  void accelWrite(uint8_t reg, uint8_t data)
  {
    I2CWrite(0xa6, reg, data);
  }
  void accelRead(uint8_t reg, uint8_t *data, size_t length)
  {
    I2CRead(0xa6, reg, data, length);
  }


  // write one data byte to specified gyro register
  void gyroWrite(uint8_t reg, uint8_t data)
  {
    I2CWrite(0xd0, reg, data);          // gyro is at I2C address 0xd0
  }
  void gyroRead(uint8_t reg, uint8_t *data, size_t length)
  {
    I2CRead(0xd0, reg, data, length);
  }
}

//! Initialize the connection to the accelerometer and gyro
void gyroAccelSetup()
{
  OpenI2C(i2c, I2C_EN, 0x062); // 400 KHz

  gyroWrite(0x3e, 0x80);  // Reset to defaults
  gyroWrite(0x16, 0x19);  // DLPF_CFG = 1 (188 Hz LP), FS_SEL = 3

  accelWrite(0x31, 0x0f); // data format, 13 bits, left justified, +/-16g range
  accelWrite(0x2c, 0x0b); // measurement rate, 200 Hz
  accelWrite(0x2d, 0x08); // power ctrl, enable measurements

  // wait for gyro to get ready (setup)
  delay(1500);
}

/**
 * \brief Read the accelerometer
 *
 * \param[out]  x,y,z  The acceleration in m s^-2, in the coordinate
 *                     system printed on the board
 */
void accelRead(float &x, float &y, float &z)
{
  float c = 2048.0 / 9.82;            // conversion factor from 13 bit to m/s^2
  uint8_t s[6];                 // need 6 bytes
  accelRead(0x32, s, 6);          // I2C addr 0xa6 and regs begin at 0x32

  x = (short int) -(s[4] + 256*s[5])/c; // assemble short int (16 bit), rescale (0 is LSB, 1 is MSB)
  y = (short int) (s[0] + 256*s[1])/c;  // to meters per second squared and
  z = (short int) -(s[2] + 256*s[3])/c; // return as floats for each direction

}
/**
 * \brief Read the gyroscope
 *
 * \param[out]  x,y,z  The angular velocity in rad s^-2, in the
 *                     coordinate system printed on the board
 */
void gyroRead(float &x, float &y, float &z)
{
  static float x0, y0, z0;                 // offset angular velocities
  static bool first = true;
  float c = 1.0/14.375/57.2958;            // sensor resolution, radians per second
  uint8_t s[6];                      // need 6 bytes

  if (first) {
    first = false;
    for (int i = 0; i < 20; i++) {
      gyroRead(0x1d, s, 6);            // I2C addr 0xd0 and regs begin at 0x1d

      x0 += (short int) -(256*s[4] + s[5])*c; // assemble short int (16 bit), rescale (0 is MSB, 1 is LSB)
      y0 += (short int) (256*s[0] + s[1])*c;  // to radians per second and return as
      z0 += (short int) -(256*s[2] + s[3])*c; // floats for each direction


      /*x0 += (short int)(256*s[0] + s[1])*c; // assemble short int (16 bit), rescale (0 is MSB, 1 is LSB)
      y0 += (short int)(256*s[2] + s[3])*c; // to radians per second and return as
      z0 += (short int)(256*s[4] + s[5])*c; // floats for each direction*/
      delay(5);
    }
    x0 /= 20;
    y0 /= 20;
    z0 /= 20;
  }
  I2CRead(0xd0, 0x1d, s, 6);               // I2C addr 0xd0 and regs begin at 0x1d
  x = (short int) -(256*s[4] + s[5])*c - x0; // assemble short int (16 bit), rescale
  y = (short int) (256*s[0] + s[1])*c - y0;  // to radians per second and return as
  z = (short int) -(256*s[2] + s[3])*c - z0; // floats for each direction
}
