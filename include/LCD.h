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

void LCD5110_init(void);

void LCD5110_write_char(unsigned char c);

void LCD5110_write_char_neg(unsigned char c);

void LCD5110_clear(void);

void LCD5110_set_XY(unsigned char X,unsigned char Y);

void LCD5110_write_string(char *s);

void LCD5110_write_string_neg(char *s);

void LCD5110_LCD_write_byte(unsigned char dat,unsigned char mode);

void LCD5110_write_Dec(unsigned int dig,unsigned int dec,unsigned int b);

void LCD5110_write_Dec_neg(unsigned int dig,unsigned int dec,unsigned int b);

void LCD5110_write_Dec1(unsigned int dig,unsigned int dec,unsigned int b);

void LCD5110_write_Dec1_neg(unsigned int dig,unsigned int dec,unsigned int b);

void LCD5110_bar(unsigned int val2,unsigned int val3);

void LCD5110_GPIO_Config();
