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

using namespace geometry;

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

  // in internal units, stored as float for extra precision
  Vector3<float> gyro_offset;

  const Vector3<float> acc_down = Vector3<float>(1.304, 0.038, 10.12).normalized();
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
 * \brief Convert from the chip coordinate frame to the robot frame
 *
 * The robot frame has 
 *    x pointing forwards (the side with the microcontroller)
 *    z pointing left
 *    y pointing up
 */
template<typename T>
Vector3<T> chipToRobotFrame(Vector3<T> v_chip) {
  Vector3<float> v_frame;

  v_frame.x = -v_chip.z;
  v_frame.y =  v_chip.x;
  v_frame.z = -v_chip.y;

  return v_frame;
}

/**
 * \brief Convert from the raw chip reading to radians per second, in the robot
 *        frame
 */
template<typename T>
Vector3<float> gyroRawToSI(Vector3<T> raw) {
  // LSB/(deg/s) and (rad/s) / lsb, respectively
  const float LSB_S_PER_DEG = 14.375;  // from datasheet
  const float RAD_PER_LSB_S = (M_PI / 180) / LSB_S_PER_DEG;
  return chipToRobotFrame(raw * RAD_PER_LSB_S);
}

/**
 * \brief Convert from the raw chip reading to meters per second squared, in the
 *        robot frame
 */
template<typename T>
Vector3<float> accRawToSI(Vector3<T> raw) {
  const float SI_PER_LSB = 9.82 / 2048.0;  // conversion factor from 13 bit to m/s^2
  return chipToRobotFrame(raw * SI_PER_LSB);
}

//! Read the raw values of the accelerometer, in internal frame and units
Vector3<int16_t> accelReadRaw()
{
  uint8_t s[6];
  accelRead(0x32, s, 6);

  // little-endian
  Vector3<int16_t> acc_i;
  acc_i.x = s[0] | (s[1] << 8);
  acc_i.y = s[2] | (s[3] << 8);
  acc_i.z = s[4] | (s[5] << 8);
  return acc_i;
}

//! Get the acceleration in m s^-2, in the robot frame
Vector3<float> accelRead()
{
  return accRawToSI(accelReadRaw());
}

//! Read the raw values of the gyroscope, without subtracting initial values
Vector3<int16_t> gyroReadRaw()
{
    uint8_t s[6];
    gyroRead(0x1D, s, 6);  // GYRO_XOUT_H - GYRO_ZOUT_L

    // big-endian
    Vector3<int16_t> omega_i;
    omega_i.x = (s[0] << 8) | s[1];
    omega_i.y = (s[2] << 8) | s[3];
    omega_i.z = (s[4] << 8) | s[5];
    return omega_i;
}


//! calibrate the offset for the gyro. Returns the stdev of each component
Vector3<float> gyroCalibrate(int N) {

  // compute running sum and sum of squares - all of raw values
  Vector3<int32_t> sum_lsb  = Vector3<int32_t>::Zero();
  Vector3<int32_t> sum_lsb2 = Vector3<int32_t>::Zero();
  for (int i = 0; i < N; i++) {
    delay(5);
    auto lsb = gyroReadRaw();

    sum_lsb += lsb;
    for (int i = 0; i < 3; i++) {
      sum_lsb2[i] += lsb[i] * lsb[i];
    }
  }

  // compute first and second moments
  Vector3<float> mean_lsb = sum_lsb; mean_lsb /= N;
  Vector3<float> std_lsb;
  for (int i = 0; i < 3; i++) {
    std_lsb[i] = sqrt(N*sum_lsb2[i] - sum_lsb[i]*sum_lsb[i]) / N;
  }

  gyro_offset = mean_lsb;
  // convert to real units
  return gyroRawToSI(std_lsb);
}


//! Get the angular velocity in the robot frame
Vector3<float> gyroRead()
{
  // read in the coordinate frame of the sensor chip
  return gyroRawToSI(gyroReadRaw() - gyro_offset);
}

//! Get the robot orientation based on the accelerometer reading. Only accurate
//! when static
quat accelOrient(Vector3<float> acc) {
  quat q = quat::between(acc, acc_down);
  return euler_angles<213>::remove_psi(quat::between(acc, acc_down));
}
quat accelOrient() {
  return accelOrient(accelRead());
}
