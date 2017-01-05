#include "i2c_funcs.h"

// ported from http://www.moderncontroltechnology.com/docs/peggy_robot/EET/SourceCode/Archives/Robots.net.H4/Controller/P4011.1/HMI/pmc/i2c/

namespace i2c_funcs {

void OpenI2C(volatile p32_i2c& i2c, uint32_t config1, uint32_t config2)
{
  i2c.ixBrg.reg = config2;
  i2c.ixCon.reg = config1;
}

void StopI2C(volatile p32_i2c& i2c)
{
	i2c.i2cCon.PEN = 1;	/* initiate Stop on SDA and SCL pins */
}

char MasterWriteI2C(volatile p32_i2c& i2c, uint8_t data_out)
{
  i2c.ixTrn.reg = data_out;

  if(i2c.i2cStat.IWCOL) /* write collision */
    return -1;
  else
    return 0;
}
void StartI2C(volatile p32_i2c& i2c)
{
  i2c.i2cCon.SEN = 1; /* initiate Start on SDA and SCL pins */
}

void IdleI2C(volatile p32_i2c& i2c)
{
  /* Wait until I2C Bus is Inactive */
  while(i2c.i2cCon.SEN || i2c.i2cCon.PEN || i2c.i2cCon.RCEN || i2c.i2cCon.ACKEN || I2C1STATbits.TRSTAT);
}

char DataRdyI2C(volatile p32_i2c& i2c)
{
  return i2c.i2cStat.RBF;
}

size_t MastergetsI2C(volatile p32_i2c& i2c, size_t length, uint8_t * rdptr, uint32_t data_wait)
{
  uint32_t wait = 0;
  while(length)                    /* Receive the number of bytes specified by length */
  {
    i2c.i2cCon.RCEN = 1;
    while(!DataRdyI2C(i2c))
    {
      if(wait < data_wait)
        wait++;
      else
        return(length);          /* Time out, return number of byte/word to be read */
    }
    wait = 0;
    *rdptr = i2c.ixRcv.reg;             /* save byte received */
    rdptr++;
    length--;
    if(length == 0)              /* If last char, generate NACK sequence */
    {
      i2c.i2cCon.ACKDT = 1;
      i2c.i2cCon.ACKEN = 1;
    }
    else                         /* For other chars,generate ACK sequence */
    {
      i2c.i2cCon.ACKDT = 0;
      i2c.i2cCon.ACKEN = 1;
    }
    while(i2c.i2cCon.ACKEN == 1);    /* Wait till ACK/NACK sequence is over */
  }
  return 0;    /* return status that number of bytes specified by length was received */
}

}