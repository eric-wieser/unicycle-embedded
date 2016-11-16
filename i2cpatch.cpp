#include "i2cpatch.h"
#include <xc.h>

// taken from http://www.moderncontroltechnology.com/docs/peggy_robot/EET/SourceCode/Archives/Robots.net.H4/Controller/P4011.1/HMI/pmc/i2c/
// license is probably prohibitive, so we should kill this

using namespace i2c_patch;

void i2c_patch::OpenI2C1(unsigned int config1,unsigned int config2)
{
  I2C1BRG = config2;
  I2C1CON = config1;
}

void i2c_patch::StopI2C1(void)
{
	I2C1CONbits.PEN = 1;	/* initiate Stop on SDA and SCL pins */
}

char i2c_patch::MasterWriteI2C1(unsigned char data_out)
{
  I2C1TRN = data_out;

  if(I2C1STATbits.IWCOL)        /* If write collision occurs,return -1 */
    return -1;
  else
    return 0;
}
void i2c_patch::StartI2C1(void)
{
  I2C1CONbits.SEN = 1; /* initiate Start on SDA and SCL pins */
}

void i2c_patch::IdleI2C1(void)
{
  /* Wait until I2C Bus is Inactive */
  while(I2C1CONbits.SEN || I2C1CONbits.PEN || I2C1CONbits.RCEN || I2C1CONbits.ACKEN || I2C1STATbits.TRSTAT);
}

char i2c_patch::DataRdyI2C1(void)
{
  return I2C1STATbits.RBF;
}

unsigned int i2c_patch::MastergetsI2C1(unsigned int length, unsigned char * rdptr, unsigned int i2c1_data_wait)
{
  int wait = 0;
  while(length)                    /* Receive the number of bytes specified by length */
  {
    I2C1CONbits.RCEN = 1;
    while(!DataRdyI2C1())
    {
      if(wait < i2c1_data_wait)
        wait++ ;
      else
        return(length);          /* Time out, return number of byte/word to be read */
    }
    wait = 0;
    *rdptr = I2C1RCV;             /* save byte received */
    rdptr++;
    length--;
    if(length == 0)              /* If last char, generate NACK sequence */
    {
      I2C1CONbits.ACKDT = 1;
      I2C1CONbits.ACKEN = 1;
    }
    else                         /* For other chars,generate ACK sequence */
    {
      I2C1CONbits.ACKDT = 0;
      I2C1CONbits.ACKEN = 1;
    }
    while(I2C1CONbits.ACKEN == 1);    /* Wait till ACK/NACK sequence is over */
  }
  return 0;    /* return status that number of bytes specified by length was received */
}