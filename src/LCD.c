/*
==========================================================================
    ElectroDacus Solar BMS "SBMS4080" Nokia5110 LCD driver.
    This is modified version based Ahmed version from my.st.com website.

    Copyright (C) 2014  <Dacian Todea>
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
===========================================================================
*/

#include "stm32f0xx.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
#include "font.h"
#include "LCD.h"

//Define the LCD Operation function
void LCD5110_LCD_write_byte(unsigned char dat,unsigned char LCD5110_MOde);
void LCD5110_LCD_delay_ms(unsigned int t);

GPIO_InitTypeDef        GPIO_InitStructure;

//Define the hardware operation function
void LCD5110_GPIO_Config(void);
void LCD5110_SCK(unsigned char temp);
void LCD5110_MO(unsigned char temp);
void LCD5110_CS(unsigned char temp);
void LCD5110_DC(unsigned char temp);



void LCD5110_init()
{
	LCD5110_GPIO_Config();

	LCD5110_DC(1);//LCD_DC = 1;
	LCD5110_MO(1);//SPI_MO = 1;
	LCD5110_SCK(1);//SPI_SCK = 1;
	LCD5110_CS(1);//SPI_CS = 1;


	LCD5110_LCD_delay_ms(70);


	LCD5110_LCD_write_byte(0x21,0);  //21
	LCD5110_LCD_write_byte(0xC2,0);  //C2
	LCD5110_LCD_write_byte(0x06,0);  //06
	LCD5110_LCD_write_byte(0x13,0);  //13
	LCD5110_LCD_write_byte(0x20,0); //20
	LCD5110_clear();
	LCD5110_LCD_write_byte(0x0C,0); // 0X0D positive
}

void LCD5110_LCD_write_byte(unsigned char dat,unsigned char mode)
{
	unsigned char i;

	LCD5110_CS(0);//SPI_CS = 0;

	if (0 == mode)
		LCD5110_DC(0);//LCD_DC = 0;
	else
		LCD5110_DC(1);//LCD_DC = 1;

	for(i=0;i<8;i++)
	{
		LCD5110_MO(dat & 0x80);//SPI_MO = dat & 0x80;
		dat = dat<<1;
		LCD5110_SCK(0);//SPI_SCK = 0;
		LCD5110_SCK(1);//SPI_SCK = 1;
	}

	LCD5110_CS(1);//SPI_CS = 1;

}



void LCD5110_bar(unsigned int val2,unsigned int val3)
{
	unsigned char val1;
		unsigned char ch = 0x7E,ch2 = 0x42;

		if (val2>val3){return;}

		LCD5110_LCD_write_byte(ch,1);
	 for(val1=0;val1<val2;val1++)
		{
			LCD5110_LCD_write_byte(ch,1);
		}
	 for(val1=val2;val1<val3;val1++)
	 {
	 	LCD5110_LCD_write_byte(ch2,1);
	 }
	 LCD5110_LCD_write_byte(ch,1);

}





void LCD5110_write_char(unsigned char c)
{
	unsigned char line;
	unsigned char ch = 0;

	c = c - 12;

	for(line=0;line<6;line++)
	{
		ch = font6_8[c][line];
		LCD5110_LCD_write_byte(ch,1);
		
	}
}
void LCD5110_write_char_neg(unsigned char c)
{
	unsigned char line;
	unsigned char ch = 0;

	c = c - 12;

	for(line=0;line<6;line++)
	{
		ch = ~font6_8[c][line];
		LCD5110_LCD_write_byte(ch,1);
		
	}
}



void LCD5110_write_string(char *s)
{
	unsigned char ch;
  	while(*s!='\0')
	{
		ch = *s;
		LCD5110_write_char(ch);
		s++;
	}
}

void LCD5110_write_string_neg(char *s)
{
	unsigned char ch;
  	while(*s!='\0')
	{
		ch = *s;
		LCD5110_write_char_neg(ch);
		s++;
	}
}


void LCD5110_clear()
{
	unsigned char i,j;
	for(i=0;i<6;i++)
		for(j=0;j<84;j++)
			LCD5110_LCD_write_byte(0,1);
}

void LCD5110_set_XY(unsigned char X,unsigned char Y)
{
	unsigned char x;
	x = 6*X;

	LCD5110_LCD_write_byte(0x40|Y,0);
	LCD5110_LCD_write_byte(0x80|x,0);
}

void LCD5110_write_Dec(unsigned int dig,unsigned int dec,unsigned int b)
{

	unsigned char datas[3];
    unsigned int i;

	if(dec!=0)
	{
	for (i=0;i<(5-(dig+1));i++)
	LCD5110_write_char(0x20);
	}
	else
	{
	for (i=0;i<(5-(dig));i++)
	{
	LCD5110_write_char(0x20);
	}
	}


	if (dig >= 4)
	{
	datas[0] = b/1000;
	b = b - datas[0]*1000;
	datas[0]+=12;
	LCD5110_write_char(datas[0]);
	if (dec==3)
	{
	LCD5110_write_char(46);
	}
	}




	if (dig >= 3)
		{
		datas[1] = b/100;
		b = b - datas[1]*100;
		datas[1]+=12;
		LCD5110_write_char(datas[1]);
		if (dec==2)
		{
		LCD5110_write_char(46);
		}
		}



	if (dig >= 2)
		{
		datas[2] = b/10;
		b = b - datas[2]*10;
		datas[2]+=12;
		LCD5110_write_char(datas[2]);
		if (dec==1)
		{
		LCD5110_write_char(46);
		}
		}



	datas[3] = b;

	datas[3]+=12;

	LCD5110_write_char(datas[3]);

}

void LCD5110_write_Dec_neg(unsigned int dig,unsigned int dec,unsigned int b)
{

	unsigned char datas[3];
	unsigned int i;

		if(dec!=0)
		{
		for (i=0;i<(5-(dig+1));i++)
		LCD5110_write_char_neg(0x20);
		}
		else
		{
		for (i=0;i<(5-(dig));i++)
		{
		LCD5110_write_char_neg(0x20);
		}
		}


	if (dig >= 4)
	{
	datas[0] = b/1000;
	b = b - datas[0]*1000;
	datas[0]+=12;
	LCD5110_write_char_neg(datas[0]);
	if (dec==3)
	{
	LCD5110_write_char_neg(46);
	}
	}

	if (dig >= 3)
		{
		datas[1] = b/100;
		b = b - datas[1]*100;
		datas[1]+=12;
		LCD5110_write_char_neg(datas[1]);
		if (dec==2)
		{
		LCD5110_write_char_neg(46);
		}
		}

	if (dig >= 2)
		{
		datas[2] = b/10;
		b = b - datas[2]*10;
		datas[2]+=12;
		LCD5110_write_char_neg(datas[2]);
		if (dec==1)
		{
		LCD5110_write_char_neg(46);
		}
		}
	datas[3] = b;

	datas[3]+=12;

	LCD5110_write_char_neg(datas[3]);

}

void LCD5110_write_Dec1(unsigned int dig,unsigned int dec,unsigned int b)
{

	unsigned char datas[3];


	if (dig >= 4)
	{
	datas[0] = b/1000;
	b = b - datas[0]*1000;
	datas[0]+=12;
	LCD5110_write_char(datas[0]);
	if (dec==3)
	{
	LCD5110_write_char(46);
	}
	}




	if (dig >= 3)
		{
		datas[1] = b/100;
		b = b - datas[1]*100;
		datas[1]+=12;
		LCD5110_write_char(datas[1]);
		if (dec==2)
		{
		LCD5110_write_char(46);
		}
		}



	if (dig >= 2)
		{
		datas[2] = b/10;
		b = b - datas[2]*10;
		datas[2]+=12;
		LCD5110_write_char(datas[2]);
		if (dec==1)
		{
		LCD5110_write_char(46);
		}
		}



	datas[3] = b;

	datas[3]+=12;

	LCD5110_write_char(datas[3]);

}

void LCD5110_write_Dec1_neg(unsigned int dig,unsigned int dec,unsigned int b)
{

	unsigned char datas[3];



	if (dig >= 4)
	{
	datas[0] = b/1000;
	b = b - datas[0]*1000;
	datas[0]+=12;
	LCD5110_write_char_neg(datas[0]);
	if (dec==3)
	{
	LCD5110_write_char_neg(46);
	}
	}

	if (dig >= 3)
		{
		datas[1] = b/100;
		b = b - datas[1]*100;
		datas[1]+=12;
		LCD5110_write_char_neg(datas[1]);
		if (dec==2)
		{
		LCD5110_write_char_neg(46);
		}
		}

	if (dig >= 2)
		{
		datas[2] = b/10;
		b = b - datas[2]*10;
		datas[2]+=12;
		LCD5110_write_char_neg(datas[2]);
		if (dec==1)
		{
		LCD5110_write_char_neg(46);
		}
		}
	datas[3] = b;

	datas[3]+=12;

	LCD5110_write_char_neg(datas[3]);

}


void LCD5110_LCD_delay_ms(unsigned int nCount)
{
  unsigned long t;
	t = nCount * 4800;
	while(t--);
}




void LCD5110_GPIO_Config()
{

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);


  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);


  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOF, &GPIO_InitStructure);




}

void LCD5110_CS(unsigned char temp)
{
	if (temp) GPIO_SetBits(GPIOF,GPIO_Pin_1);
	else GPIO_ResetBits(GPIOF,GPIO_Pin_1);
}


void LCD5110_DC(unsigned char temp)
{
	if (temp) GPIO_SetBits(GPIOF,GPIO_Pin_0);
	else GPIO_ResetBits(GPIOF,GPIO_Pin_0);
}

void LCD5110_MO(unsigned char temp)
{
	if (temp) GPIO_SetBits(GPIOB,GPIO_Pin_9);
	else GPIO_ResetBits(GPIOB,GPIO_Pin_9);
}

void LCD5110_SCK(unsigned char temp)
{
	if (temp) GPIO_SetBits(GPIOB,GPIO_Pin_8);
	else GPIO_ResetBits(GPIOB,GPIO_Pin_8);
}
