#include "LCD.h"

void delayms1(unsigned int nCount)
{
  unsigned long t;
  	t = nCount * 4800;  //1ms/unit at 48Mhz
  	while(t--);

}

void delayms2(unsigned int nCount)
{
  unsigned long t;
	t = nCount * 720;   // mostly for i2c
	while(t--);
}
