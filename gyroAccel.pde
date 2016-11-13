// Provide interface to gyro and accelerometer via I2C serial 
//
// Carl Edward Rasmussen, 2011-09-22
// Aleksi Tukiainen, 2016-05-20

#include "gyroAccel.h"

// write one data byte to specified accelerometer register

void accelWrite(unsigned char reg, unsigned char data)
{
  I2CWrite(0xa6, reg, data); 
}


// read the accelerometer and return accel in m/s^2

void accelRead(float &x, float &y, float &z)
{
  float c = 2048.0 / 9.82;            // conversion factor from 13 bit to m/s^2
  unsigned char s[6];                 // need 6 bytes
  I2CRead(0xa6, 0x32, s, 6);          // I2C addr 0xa6 and regs begin at 0x32
  
  x = (short int) -(s[4] + 256*s[5])/c; // assemble short int (16 bit), rescale (0 is LSB, 1 is MSB)
  y = (short int) (s[0] + 256*s[1])/c;  // to meters per second squared and
  z = (short int) -(s[2] + 256*s[3])/c; // return as floats for each direction

}

// write one data byte to specified gyro register

void gyroWrite(unsigned char reg, unsigned char data)
{
  I2CWrite(0xd0, reg, data);          // gyro is at I2C address 0xd0
}

// read the rate gyros, and return angular speeds in radians per second

void gyroRead(float &x, float &y, float &z)
{
  static float x0, y0, z0;                 // offset angular velocities
  static boolean first = true;
  float c = 1.0/14.375/57.2958;            // sensor resolution, radians per second
  unsigned char s[6];                      // need 6 bytes

  if (first) {
    first = false;
    for (int i=0; i<20; i++) {
      I2CRead(0xd0, 0x1d, s, 6);            // I2C addr 0xd0 and regs begin at 0x1d

      x0 += (short int) -(256*s[4] + s[5])*c; // assemble short int (16 bit), rescale (0 is MSB, 1 is LSB)
      y0 += (short int) (256*s[0] + s[1])*c;  // to radians per second and return as
      z0 += (short int) -(256*s[2] + s[3])*c; // floats for each direction

      
      /*x0 += (short int)(256*s[0] + s[1])*c; // assemble short int (16 bit), rescale (0 is MSB, 1 is LSB)
      y0 += (short int)(256*s[2] + s[3])*c; // to radians per second and return as
      z0 += (short int)(256*s[4] + s[5])*c; // floats for each direction*/
      delay(5);
    }
    x0 /= 20; y0 /= 20; z0 /= 20;
  }
  I2CRead(0xd0, 0x1d, s, 6);               // I2C addr 0xd0 and regs begin at 0x1d
  x = (short int) -(256*s[4] + s[5])*c - x0; // assemble short int (16 bit), rescale 
  y = (short int) (256*s[0] + s[1])*c - y0;  // to radians per second and return as
  z = (short int) -(256*s[2] + s[3])*c - z0; // floats for each direction
  

}


// write one data byte to a specified register at I2C address

void I2CWrite(unsigned char addr, unsigned char reg, unsigned char data)
{
  StartI2C1(); IdleI2C1();           // send start condition
  MasterWriteI2C1(addr); IdleI2C1(); // I2C write address
  MasterWriteI2C1(reg); IdleI2C1();  // send register  
  MasterWriteI2C1(data); IdleI2C1(); // send data 
  StopI2C1(); IdleI2C1();            // send stop condition
}  


// read length bytes to data array from I2C addr, starting at specified reg

void I2CRead(unsigned char addr, unsigned char reg, unsigned char *data, int length)
{
  StartI2C1(); IdleI2C1();               // send start condition
  MasterWriteI2C1(addr); IdleI2C1();     // I2C write address
  MasterWriteI2C1(reg); IdleI2C1();      // send start register
  StartI2C1(); IdleI2C1();               // send start condition
  MasterWriteI2C1(addr | 1); IdleI2C1(); // I2C read address
  MastergetsI2C1(length, data, 2000);    // get data
  IdleI2C1(); StopI2C1(); IdleI2C1();    // send stop condition
}
