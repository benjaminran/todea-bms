/*
==========================================================================
    ElectroDacus Solar BMS "SBMS4080" main file.
    v0.9c
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
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include <stdio.h>
#include "LCD.h"
#include "BMS1.h"
#include "tsl_user.h"
#include "tsl.h"
#include "stm32f0xx_pwr.h"
#include "stm32f0xx_rtc.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_misc.h"
#include "stm32f0xx_syscfg.h"
#include "stm32f0xx_tim.h"
#include "stm320518_eval_i2c_ee.h"
#include "bmsmenu.h"
#include "tsl_touchkey.h"
#include "stm32f0xx_usart.h"


// Select to use or not the Low-Power mode and RTC
#define USE_LPM 1 // 0=No, 1=Yes
#define SBMS4080 1 //0=1616 1=4080


// Private defines

// LEDs definition
#define LED1_OFF    {GPIOB->BSRR = (uint32_t)((uint32_t)1 << PIN_14);}
#define LED1_ON     {GPIOB->BSRR = (uint32_t)((uint32_t)1 << (PIN_14 + 16));}
#define LED2_OFF    {GPIOB->BSRR = (uint32_t)((uint32_t)1 << PIN_15);}
#define LED2_ON     {GPIOB->BSRR = (uint32_t)((uint32_t)1 << (PIN_15 + 16));}

// Ext GPIO's
#define EXT1_ON    {GPIOA->BSRR = (uint32_t)((uint32_t)1 << PIN_11);}
#define EXT1_OFF     {GPIOA->BSRR = (uint32_t)((uint32_t)1 << (PIN_11 + 16));}
#define EXT2_ON    {GPIOA->BSRR = (uint32_t)((uint32_t)1 << PIN_12);}
#define EXT2_OFF     {GPIOA->BSRR = (uint32_t)((uint32_t)1 << (PIN_12 + 16));}

// I2C
#define RAMStart       0x80
#define ROMUStart      0x50
#define ROMSize        88
#define RAMSize        44
#define ROMUSize       8

uint8_t ROMt[4];
uint8_t ROMrx[ROMSize];
uint8_t ROMtemp[ROMSize];
uint8_t RAMrx[RAMSize];
uint8_t ROMUrx[ROMUSize];
uint16_t ROMdec[50];
uint16_t ROMdec2[3];
char* ROMunit[50];
uint8_t tempL;
uint8_t tempH;
uint8_t d[50][2];
unsigned int td[5],bk[3],Serial[3],EXT[4];


volatile uint16_t NumData = 0;
uint32_t  sEETimeout;


// Private macros
#define TEST_TKEY(NB) (MyTKeys[(NB)].p_Data->StateId == TSL_STATEID_DETECT)


// Private variables
//GPIO_InitTypeDef        GPIO_InitStructure;
TSL_tTick_ms_T ECS_last_tick; // Hold the last time value for ECS
TSL_tTick_sec_T debug_tick;
uint32_t process_sensor;

// Private functions prototype

void Init_Std_GPIOs(void);
void TSL_Action1(void);
void Time1 (unsigned int r);
void ProcessSensors(void);
void LoadONOFF(void);

void ROMv1bd(unsigned int pp,unsigned int loop22);
void ROMunitbd(unsigned int pp,unsigned int loop22);
void ROMv1db(unsigned int pp,unsigned int loop22);
void ROMunitdb(unsigned int pp,unsigned int loop22,unsigned int timeu);

void ROMconvertdb(void);
void ROMconvertbd(void);
void ROMreadall(void);
void RAMreadall(void);
void ROMread(uint8_t ROMStart);
void ROMwrite(uint8_t ROMaddress,uint8_t Tx1);
void RAMwrite(uint8_t RAMaddress,uint8_t Tx1);
void valueset(void);
void scrollhelp(void);
void mainmenu1(void);
void monitoring(void);
void monitor(void);
void parameterset(void);
void deviceset(void);
void diagnostics(void);
void about(void);
void timeanddate(void);
void valueset2(void);
void backlight(void);
void valueset3(void);
void bk1(void);
void dataaquisition(void);
void bktimer(void);
void USART_Config(void);
void timestamp1(void);
void datestamp1(void);
void USARTMenu(void);
void valueset4(void);
void onesecondcounter(void);
void uSDMenu(void);
void EXT1ANDEXT2(void);
void UsartDataHeader(void);
void counterR(void);
void externalIO(void);
void valueset5(void);
void externalTEMPI(void);
void lk1(void);
void unlock();
void lockKtimer();

#if USE_LPM > 0
void RTC_Config(void);
void SYSCLKConfig_STOP(void);
void RTC_AlarmConfig(void);
void LowPower(void);
void Init_Reset_GPIOs(void);
void Init_LPM_GPIOs(void);
#endif


GPIO_InitTypeDef GPIO_InitStructure;
RTC_InitTypeDef RTC_InitStructure;
RTC_TimeTypeDef RTC_TimeStructure;
RTC_DateTypeDef RTC_DateStructure;
RTC_TimeTypeDef RTC_TimeStampStructure;
RTC_DateTypeDef RTC_TimeStampDateStructure;

unsigned int b1,b2,b3,b4,b5,b6,c1,c2,c22,c222,c3,c33,c333,c3333,c6,c7,cs7,c8,c8cal,c9,c9cal,c9calx50,c9calx500,ITEMP,ETEMP,ETEMPLimit,c10,sj,j,BOK,BMENU,BLoad,BBack,TOK,SOK,z,nr1,i3,i4,i5,i6,scrolllenght,tim1,key1,update1,step1,i2c,dir1,dir2,dir3;
unsigned int bc1,bc2,bc3,bc4,sc1,sc2,sc3,sc4,sc5,gain,SolarCurrent,BatteryCurrent,LoadCurrent,BatteryVoltage,BatteryVoltage2,buttondir,buttondirm,swtimer1,swtimer2,swtimer3,swtimer4,timer3600,bk11,bk22;
unsigned long BatteryW,SolarW,LoadW,SOC1;
unsigned long long BatterySOC,BattCapacity,BatteryWh,BatterykWh2,BatteryuWh,BatteryuWh2,SolaruAh,SolarmAh,LoaduAh,LoadmAh,SolarWh,SolarkWh2,SolaruWh,SolaruWh2,LoadWh,LoaduWh,BatteryuAh,BatterymAh;
unsigned int zz1,zz2,zz3,zz4,zz5,filter1,filter2,filter3,release1,release2,refresh1,DFET,CFET,lockkey,lockkey1,lockkey2,UVlock,OVlock,nrcells,Cdirection,rangeswitch;
unsigned int BV[8][4],BVmin[2],BVmax[2],BVdelta,CB[8],EF[15];

char timestamp[8],datestamp[8];
long i8;
char *helptext;
char chr[192];
char* direction;

__IO uint32_t subsec = 0;

extern const TSL_TouchKey_T MyTKeys[];
extern TSL_ObjectGroup_T MyObjGroup;

//constants

static float const v1=0.117216117;

#if SBMS4080>0
static unsigned int const PVgain=60;  //MAX4080S
static float const PVshunt=0.00052;
static float const Battshunt=0.00028;
#endif





//============================================================================
//                             Main routine.
//============================================================================

int main(void)
{


  delayms1(2000);  //delay 2s to be able to enter debug or program the device *do not remove this*

  Init_Std_GPIOs();

  TSL_user_Init();

  LCD5110_init();

  LCD5110_set_XY(0,0);
  LCD5110_write_string_neg("SBMS 4080 v03c");
  delayms1(200);
  LCD5110_clear();


  EXT1_OFF;       //just a temporary timing test.
  delayms1(2);
  EXT1_ON;
  delayms1(3);
  EXT1_OFF;
  delayms1(4);
  EXT1_ON;


  // Low-Power mode configuration

#if USE_LPM > 0

      RTC_Config();
#else
 // while(TSL_tim_CheckDelay_sec(1, &debug_tick) == TSL_STATUS_BUSY);
#endif

  i2c=0;
  LCD5110_set_XY(0,1);
  LCD5110_write_string_neg("BeforeRAMread");
  RAMreadall();
  LCD5110_set_XY(0,2);
  LCD5110_write_string_neg("BeforeRAMwrite");
  RAMwrite(0x85,32); //set current amplifier gain x50 (00) x5 (16) x500 (32) x500 (48)
  LCD5110_set_XY(0,3);
  LCD5110_write_string_neg("BeforeROM");
  ROMread(0x00);
  ROMreadall(); //read the entire EEPROM
  LCD5110_set_XY(0,4);
  LCD5110_write_string_neg("ROM-OK");



  //-----------------------------------------------------------------------------
  j=5;
  b1=0;
  b2=0;
  b3=0;
  b4=0;
  c2=25;
  key1=0;
  c33=0;
  c3=1;
  i6=0;
  update1=1;
  step1=1;
  buttondir=0;
  swtimer1=3601;
  swtimer3=0;
  swtimer4=3601;
  filter1=0;
  filter2=0;
  filter3=0;
  ROMdec2[0]=5;
  ROMdec2[1]=3;
  ROMdec2[2]=2;
  BattCapacity=(100*1000000);
  BatterySOC=BattCapacity*1.5;  //set the initial SOC at 50% after a full charge the SOC will be correct.
  refresh1=1;
  lockkey=0;
  lockkey1=0;
  lockkey2=0;
  zz4=6;
  cs7=0;
  c7=0;


  LCD5110_set_XY(0,5);
  LCD5110_write_string_neg("Calibration.");

  for (sj=0;sj<10;sj++)     //PV & Batt-x500 current calibration (get the ADC offset)
  {
	  RAMreadall();
	  c8=((RAMrx[37]*256)+RAMrx[36]);  //PV current
	  c9=(RAMrx[15]*256)+RAMrx[14];  //battery current
	  cs7=c8+cs7;
	  c7=c9+c7;
   	  delayms1(50);

  }

  c8cal=cs7/10;
  c9calx500=c7/10;

  c7=0;
  RAMwrite(0x85,0); //set current amplifier gain x50 (00) x5 (16) x500 (32) x500 (48)
  LCD5110_set_XY(0,5);
  LCD5110_write_string_neg("Calibration..");

  for (sj=0;sj<10;sj++)     //Batt-x50 current calibration (get the ADC offset)
   {
 	  RAMreadall();
 	  c9=(RAMrx[15]*256)+RAMrx[14];  //battery current
 	  c7=c9+c7;
      delayms1(50);
   }

  c9calx50=c7/10;

  LCD5110_set_XY(0,5);
  LCD5110_write_string_neg("Calibration...");


  ROMconvertbd();

  bk[0]=2; //set backlight to max
  bk[1]=1; //default backlight off set at 1 minute
  bk1();
  bk[2]=1;  //lockkey set at 1min
  lk1();

  EXT[0]=1;
  EXT[1]=90;
  EXT[2]=1;
  EXT[3]=30;

  Serial[0]=0; //disable USART datalog at power ON
  Serial[1]=96; //USART baud rate set default at 9600 (baud set at 10000 seems to work better) also value is /100
  Serial[2]=30;   //data log interval set default at 30s.
  swtimer2=0;

  BattCapacity=(ROMdec[2]*1000000);
  BatterySOC=BattCapacity*1.5;  //set the initial SOC at 50% after a full charge the SOC will be correct.

  USART_Config();

//============================================================================
//                             Main loop
//============================================================================

  for (;;)
  {

   bktimer();
   lockKtimer();
   dataaquisition();
   onesecondcounter();


if (key1==0)
{
    TSL_Action1();
	ProcessSensors();
}

if (key1!=0)
{
 bk1();
 lk1();
}


if ((zz3==1)&&(lockkey==1)&&(zz4<6))
	{
	zz4++;
	zz3=0;
	i5=0;
	LCD5110_set_XY(13,3);
	LCD5110_write_Dec1(1,0,zz4);
	}

if (zz4==5){update1=1;zz4=6;LCD5110_clear();}

if ((key1!=0)&&(lockkey==1))
{
update1=0;
if (zz4>=6)
{
zz4=0;
lockkey1=0;
lockkey2=0;
}
unlock();
key1=0;
c3=c3333;
c2=c222;
BOK=0;
BBack=0;
BMENU=0;
}



if (zz1==1 && (c2==5||c2==8)&&((zz4==6)||lockkey!=1))
{
	zz1=0;
	if(c2==5){Time1(5);}
	if(c2==8){Time1(0);}
}


if (update1==1)
{
   RAMreadall();
}

    if((key1==1)||(update1==1))
{
    update1=0;
    key1=0;
	mainmenu1();    //c2>=25;
    valueset();     //c2=6;
    monitoring();   //c2=1;
    parameterset(); //c2=2;
    deviceset();    //c2=3;
    diagnostics();  //c2=4;
    about();        //c2=5;
    valueset2();    //c2=7;
    timeanddate();  //c2=8;
    valueset3();    //c2=13;
    backlight();    //c2=9;
    valueset4();    //c2=14;
    USARTMenu();    //c2=10;
    uSDMenu();      //c2=11;
    EXT1ANDEXT2();  //c2=12;
    valueset5();    //c2=16
    counterR();     //c2=15;

}


    if (i6>=3)
    {
    i6=0;
    scrollhelp();
    }
    i6++;
  }



}

//========================================================================================
// Initializes GPIOs not related to Touch-Sensing or LCD (LEDs, EXT1 and EXT2 pins, ...)
//========================================================================================

void Init_Std_GPIOs(void)
{

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	  /* Configure PB14 and PB15 output open drain mode for back light */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	  	  /* Configure PA11 and PA12 output not open drain mode for EXT1 and EXT2 lines */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

}


void externalIO(void)
{
if (EXT[0]==1)
{
if (CFET==1){EXT1_ON;}
else {EXT1_OFF;}
}
if (EXT[0]==2)
{
if (EXT[1]<=SOC1){EXT1_ON;}
else {EXT1_OFF;}
}

if (EXT[2]==1)
{
if (DFET==1){EXT2_ON;}
else {EXT2_OFF;}
}
if (EXT[2]==2)
{
if (EXT[3]>=SOC1){EXT2_ON;}
else {EXT2_OFF;}
}
if (EXT[0]==0){EXT1_OFF;}
if (EXT[2]==0){EXT2_OFF;}
CFET=0;
DFET=0;
}

void externalTEMPI(void)
{

if ((ETEMP>ROMdec[28])||(ETEMP<ROMdec[29]))
{
ETEMPLimit=1;
tempH=((RAMrx[6]) & 0x01);
    if(tempH!=0)
        {
        tempL=(((RAMrx[7])&0xBF)+64);
    	RAMwrite(0x87,tempL); //uC manual control
    	LCD5110_set_XY(3,0);
    	LCD5110_write_string("Load&PV OFF");
    	tempL=(RAMrx[6] & 0xFC);
    	RAMwrite(0x86,tempL);//load and PV is OFF
    	// need to do also charge OFF
        delayms1(300);
        }

}
if (ETEMPLimit==1)
{
if (((ETEMP+20)<=ROMdec[28])&&(ETEMP>=(ROMdec[29]+20)))
{
ETEMPLimit=0;
		tempL=(((RAMrx[7])&0xEF)+16);
		RAMwrite(0x87,tempL); //uC manual load monitor control
		tempL=(((RAMrx[6])&0x3F)+128);
		RAMwrite(0x86,tempL); //uC manual load monitor control

		tempL=(((RAMrx[7])&0xBF));
 		RAMwrite(0x87,tempL);        //FET auto enable
 		LCD5110_set_XY(3,0);
 		LCD5110_write_string("Load&PV ON ");
 		delayms1(300);
}
}
}



void onesecondcounter(void)
{
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
	timer3600=RTC_TimeStructure.RTC_Seconds;

	if (timer3600<swtimer3){timer3600=timer3600+60;}
	 if (0<(timer3600 - swtimer3)) //see if the one second has elapsed.
	    {
	    	swtimer3=RTC_TimeStructure.RTC_Seconds; //get the new time
	    	zz1=1;
            zz2=1;
            zz3=1;
            RAMreadall();
            monitor();
            externalIO();
            externalTEMPI();
	    }
}

//-------------------------------------------------------------------------------
void bktimer(void)
{
	if (swtimer1>3601){return;}

	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
    timer3600=((RTC_TimeStructure.RTC_Minutes*60)+RTC_TimeStructure.RTC_Seconds);

    if (timer3600<swtimer1){timer3600=timer3600+3600;}

    if ((bk[1]*60)<(timer3600 - swtimer1) && bk[1]>0 && swtimer1<3601) //for LCD backlight
    {
    	LED1_OFF;
    	delayms1(60);
    	LED2_OFF;
    	swtimer1=3601;
    }
}

//-------------------------------------------------------------------------------

void unlock(void)
{

	buttondir=0;
	LCD5110_set_XY(0,0);
	LCD5110_write_string("              ");
	LCD5110_set_XY(0,1);
	LCD5110_write_string_neg("Keys to unlock");
	LCD5110_set_XY(0,2);
	LCD5110_write_string("==============");
	LCD5110_set_XY(0,3);
	LCD5110_write_string(" OK UP DOWN   ");
	LCD5110_set_XY(0,4);
	LCD5110_write_string("==============");
	LCD5110_set_XY(0,5);
	LCD5110_write_string("              ");


    if (BOK==1)
    {
    LCD5110_set_XY(0,3);
    LCD5110_write_string_neg(" OK");
    lockkey1=1;

    }
    if ((b4>0)&&(lockkey1==1))
    {
    LCD5110_set_XY(0,3);
    LCD5110_write_string_neg(" OK UP");
    lockkey2=1;

    }
    if ((b3>0)&&(lockkey1==1)&&(lockkey2==1))
    {
    LCD5110_set_XY(0,3);
    LCD5110_write_string_neg(" OK UP DOWN   ");
    lockkey1=0;
    lockkey2=0;
    lockkey=0;
    update1=1;
    delayms1(200);
    LCD5110_clear();
    buttondir=buttondirm;
    }


}



void lockKtimer(void)
{
	if (swtimer4>3601){return;}

	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
    timer3600=((RTC_TimeStructure.RTC_Minutes*60)+RTC_TimeStructure.RTC_Seconds);

    if (timer3600<swtimer4){timer3600=timer3600+3600;}

    if ((bk[2]*60)<(timer3600 - swtimer4) && bk[2]>0 && swtimer4<3601) //for LCD backlight
    {
    	lockkey=1;
    	c3333=c3;
    	c222=c2;
    	i5=0;
    	buttondirm=buttondir;
    	swtimer4=3601;
    }
}


//------------------------------------------------------------------------


void UsartDataHeader(void)
{
	uint8_t nextln;
	char intro1[] = "ElectroDacus,SBMS4080";
	char datalogheader[] = "Y,M,D,H,M,S,,Cell1[mV],Cell2[mV],Cell3[mV],Cell4[mV],Cell5[mV],Cell6[mV],Cell7[mV],Cell8[mV],BattCurrent[A/10],SollarCurrent[A/10]";
	char *l1,*l2;
	l1 = intro1;
	l2 = datalogheader;
	nextln=10;  //char for new line
	while(*l1)
	{
	 while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	 USART_SendData(USART2, *l1++);
	}

	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, nextln);   //new line

	while(*l2)
		{
		 while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
		 USART_SendData(USART2, *l2++);
		}

	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, nextln);   //new line

}





void dataaquisition(void)
{

	if (Serial[0]!=1){return;}
	//USART_Config();
	uint8_t usart_data[4][10];
	uint8_t nextln,nextval,batdir;
	int ttr;

	RAMreadall();
	monitor();

	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
	timer3600=((RTC_TimeStructure.RTC_Minutes*60)+RTC_TimeStructure.RTC_Seconds);  //load current time

    if (timer3600<swtimer2){timer3600=timer3600+3600;}

    if ((Serial[2]-1)<(timer3600 - swtimer2)) //see if the set time interval has elapsed.
    {
    	swtimer2=((RTC_TimeStructure.RTC_Minutes*60)+RTC_TimeStructure.RTC_Seconds); //get the new time

        datestamp1();
    	timestamp1();
    	nextln=10;  //char for new line
    	nextval=44;  //char for new column in standard .csv files


    	for (ttr=0;ttr<8;ttr++){
    	    	    	     while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    	    	    	     USART_SendData(USART2, datestamp[ttr]);   //send date.
    	    	    	        }

    	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    	USART_SendData(USART2, nextval);


    	for (ttr=0;ttr<8;ttr++){
    	    	            while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    	    	            USART_SendData(USART2, timestamp[ttr]);  //send time
    	    	               }

    	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    	USART_SendData(USART2, nextval);

    	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    	USART_SendData(USART2, nextval);

for (ttr=0;ttr<8;ttr++){
    	usart_data[0][ttr] = (BV[ttr][3] / 1000 ) + 0x30;
    	usart_data[1][ttr] = (BV[ttr][3] % 1000/100 ) + 0x30;
    	usart_data[2][ttr] = (BV[ttr][3] % 100/10 ) + 0x30;
    	usart_data[3][ttr] = (BV[ttr][3] % 10 ) + 0x30;

}

    	for (ttr=0;ttr<8;ttr++){
    	            while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    	            USART_SendData(USART2, usart_data[0][ttr]);
    	            while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    	            USART_SendData(USART2, usart_data[1][ttr]);
    	            while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    	            USART_SendData(USART2, usart_data[2][ttr]);
    	            while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    	            USART_SendData(USART2, usart_data[3][ttr]);

    	            while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    	            USART_SendData(USART2, nextval);
    	} // send individual cell voltage [mV]

    	        tempH=((RAMrx[2]) & 0x0C);  //+/- charge or discharge for battery current
    	                    if (tempH==0){batdir=45;}
    	          	        if (tempH==4){batdir=43;}
    	         	        if (tempH==8){batdir=45;}


    	        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    	    	USART_SendData(USART2, batdir);  //battery current direction

    	    	usart_data[0][8] = (BatteryCurrent / 1000 ) + 0x30;
    	    	usart_data[1][8] = (BatteryCurrent % 1000/100 ) + 0x30;
    	    	usart_data[2][8] = (BatteryCurrent % 100/10 ) + 0x30;
    	    	usart_data[3][8] = (BatteryCurrent % 10 ) + 0x30;

    	    	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    	    	USART_SendData(USART2, usart_data[0][8]);
    	    	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    	    	USART_SendData(USART2, usart_data[1][8]);
    	    	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    	    	USART_SendData(USART2, usart_data[2][8]);
    	    	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    	    	USART_SendData(USART2, usart_data[3][8]);  //battery current

    	    	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    	    	USART_SendData(USART2, nextval);

    	    	usart_data[0][9] = (SolarCurrent / 1000 ) + 0x30;
    	    	usart_data[1][9] = (SolarCurrent % 1000/100 ) + 0x30;
    	    	usart_data[2][9] = (SolarCurrent % 100/10 ) + 0x30;
    	    	usart_data[3][9] = (SolarCurrent % 10 ) + 0x30;

    	    	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    	    	USART_SendData(USART2, usart_data[0][9]);
    	    	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    	    	USART_SendData(USART2, usart_data[1][9]);
    	    	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    	    	USART_SendData(USART2, usart_data[2][9]);
    	    	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    	    	USART_SendData(USART2, usart_data[3][9]);//solar PV current

    	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    	USART_SendData(USART2, nextln);   //new line

    }

}

//--------------------------------


//========================================================================================
// Perform the TS acquisition and the processing of the sensors state machine
//========================================================================================


void TSL_Action1(void)
{
  uint32_t idx_bank;
  // Acquire all Banks
  //TSLPRM_TOTAL_BANKS;
  for (idx_bank = 0; idx_bank < 3; idx_bank++)
  {
    // Configure Bank
    TSL_acq_BankConfig(idx_bank);
    // Start Bank acquisition
    TSL_acq_BankStartAcq();
    // Wait Bank End of Acquisition
        while (TSL_acq_BankWaitEOC() == TSL_STATUS_BUSY) {}
    // Get Bank Result
    TSL_acq_BankGetResult(idx_bank, 0, 0);
  }
  // Process Objects
  TSL_obj_GroupProcess(&MyObjGroup);
  // DxS processing (if TSLPRM_USE_DXS option is set)
  TSL_dxs_FirstObj(&MyObjGroup);
  if (TSL_tim_CheckDelay_ms(125, &ECS_last_tick) == TSL_STATUS_OK)
    {
      if (TSL_ecs_Process(&MyObjGroup) == TSL_STATUS_OK)
      {
        process_sensor = 0;
      }
      else
      {
        process_sensor = 1;
      }
    }
}

//========================================================================================
// ISL94203 related functions.
//========================================================================================


void RAMreadall(void)
	{

	sEE_Init();
    NumData = RAMSize;
    sEE_ReadBuffer(RAMrx, RAMStart, (uint16_t *)(&NumData));
    sEE_DeInit();

	}

void RAMwrite(uint8_t RAMaddress,uint8_t Tx1)
	{

	sEE_Init();
	  sEE_WriteBuffer((uint8_t*)(&Tx1), RAMaddress, 1);  // write one byte to RAM
	sEE_DeInit();
	delayms2(6);
	}


void ROMwrite(uint8_t ROMaddress,uint8_t Tx1)
	{
	RAMwrite(0x89,1);
	delayms2(6);
	sEE_Init();
 	sEE_WriteBuffer((uint8_t*)(&Tx1), ROMaddress, 1); //write one byte to EEPROM
 	delayms2(6);
 	sEE_DeInit();

 	RAMwrite(0x89,0);

	}

void ROMreadall()
{
int y1,y2,y3;

	for(y1=0;y1<=21;y1++)
	{
	y2=y1*4;
	ROMread(y2);
	for(y3=0;y3<=3;y3++)
	{
	ROMrx[y2+y3]=ROMt[y3];
	ROMtemp[y2+y3]=ROMrx[y2+y3];
	}
	}


}


void ROMwriteall()
{
int y1,y2,y3;

    ROMtemp[74]=48; // manual write some EEPROM location
    ROMtemp[40]=255;
    ROMtemp[41]=15;
    ROMtemp[42]=255;
    ROMtemp[43]=15;
    ROMtemp[44]=0;
    ROMtemp[45]=0;
    ROMtemp[46]=0;
    ROMtemp[47]=0;
    ROMtemp[48]=0;
    ROMtemp[49]=0;
    ROMtemp[50]=0;
    ROMtemp[51]=0;
    ROMtemp[52]=255;
    ROMtemp[53]=15;
    ROMtemp[54]=255;
    ROMtemp[55]=15;
    ROMtemp[56]=0;
    ROMtemp[57]=0;
    ROMtemp[58]=0;
    ROMtemp[59]=0;
    ROMtemp[60]=255;
    ROMtemp[61]=15;
    ROMtemp[62]=255;
    ROMtemp[63]=15;

	for(y1=0;y1<=21;y1++)
	{
	y2=y1*4;
	for(y3=0;y3<=3;y3++)
	{
	ROMwrite(y2+y3,ROMtemp[y2+y3]);
	LCD5110_set_XY(0,5);
	//LCD5110_write_Dec_neg(2,0,y2+y3);
	if ((y2+y3)>5){
	LCD5110_bar((y2+y3)-5,82);
	}
	}
	delayms2(6);
	}


}

void ROMread(uint8_t ROMStart)
	{

       RAMwrite(0x89,1);

 	   sEE_Init();
 	   NumData = 4;
 	   sEE_ReadBuffer(ROMt, ROMStart, (uint16_t *)(&NumData)); //read 4byte EEPROM

 	   sEE_DeInit();
 	   RAMwrite(0x89,0);

	}
//----------------------------------------------------------------------------------------
//convert binary to decimal
//----------------------------------------------------------------------------------------

void ROMv1bd(unsigned int pp,unsigned int loop22)
{
	tempH=((ROMtemp[loop22+1]) & 0x0F);
	ROMdec[pp]=(ROMtemp[loop22]+(tempH*256))*v1+0.5;
}
void ROMunitbd(unsigned int pp,unsigned int loop22)
{
	tempH=((ROMtemp[loop22+1]) & 0x03);
	ROMdec[pp]=(ROMtemp[loop22]+(tempH*256));
	tempL=((ROMtemp[loop22+1]) & 0x0C);
	if(tempL==0){ROMunit[pp]="us";}
	if(tempL==4){ROMunit[pp]="ms";}
	if(tempL==8){ROMunit[pp]="s ";}
	if(tempL==12){ROMunit[pp]="m ";}
}

void ROMconvertbd(void)
{

unsigned int l,loop1,loop2;
static float const v2=0.4395604;


for (l=0;l<34;l++)   //load limits from table bmsmenu.h
{
d[l][0]=1;
d[l][1]=0;
ROMdec[l]=0;
ROMunit[l]=bmsmenu[l][2];
d[l][0]=bmsmenuval[l][0];
d[l][1]=bmsmenuval[l][1];
}


//-------------------------------------------------------------------

    ROMdec[0]=ROMtemp[80];  //preset cell type stored in user eeprom

    if ((ROMdec[0]==6)||(ROMdec[0]==7))
          {
          ROMtemp[82]=((ROMtemp[82]) & 0x7F)+128;
          }
    else
    {
     ROMtemp[82]=((ROMtemp[82]) & 0x7F);
    }


//-------------------------------------------------------------------
	if(ROMtemp[73]==131){ROMdec[1]=3;}  //number of cells
	if(ROMtemp[73]==195){ROMdec[1]=4;}
	if(ROMtemp[73]==199){ROMdec[1]=5;}
	if(ROMtemp[73]==231){ROMdec[1]=6;}
	if(ROMtemp[73]==239){ROMdec[1]=7;}
	if(ROMtemp[73]==255){ROMdec[1]=8;}
//-------------------------------------------------------------------
	tempH=((ROMtemp[82]) & 0x7F);
	ROMdec[2]=(ROMtemp[81]+(tempH*256));
	tempL=((ROMtemp[82]) & 0x80);
    if(tempL==0){ROMunit[2]="Ah";}
	if(tempL==128){ROMunit[2]="0F";}
//-------------------------------------------------------------------
	 ROMv1bd(3,0); // overvoltage threshold;
//-------------------------------------------------------------------
 	    ROMunitbd(4,16);
//-------------------------------------------------------------------
	 ROMv1bd(5,2); // overvoltage recovery;
//-------------------------------------------------------------------
	 ROMv1bd(6,4); // Undervoltage threshold;
//-------------------------------------------------------------------
	 ROMunitbd(7,18); // undervoltage delay;
//-------------------------------------------------------------------
   loop2=6;  // Undervoltage recovery, Overvoltage Lock-out, Undervoltage Lock-out, EOC,  Low voltage charge;
 	     	for(loop1=8;loop1<13;loop1++)
 	     	{
 	     		ROMv1bd(loop1,loop2);
 	     		loop2=loop2+2;
 	     	}
//-------------------------------------------------------------------
   	    tempH=((ROMtemp[25]) & 0x70);       //Charge current;
   	    if(tempH!=0){ROMdec[13]=realcurrent[tempH/16][2];ROMdec2[0]=tempH/16;}
   	   	else{ROMdec[13]=realcurrent[0][2];ROMdec2[0]=0;}
//-------------------------------------------------------------------
   	    ROMunitbd(14,24);  // Charge Current Time;
//-------------------------------------------------------------------
   	 	tempH=((ROMtemp[23]) & 0x70);            //Discharge current
   	    if(tempH!=0){ROMdec[15]=realcurrent[tempH/16][1];ROMdec2[1]=tempH/16;}
   	    else{ROMdec[15]=realcurrent[0][1];ROMdec2[1]=0;}
//-------------------------------------------------------------------
   	    ROMunitbd(16,22);  // Discharge Current Time;
//-------------------------------------------------------------------
  	 	tempH=((ROMtemp[27]) & 0x70);            //Short circuit current
  	 	if(tempH!=0){ROMdec[17]=realcurrent[tempH/16][3];ROMdec2[2]=tempH/16;}
  	 	else{ROMdec[17]=realcurrent[0][3];ROMdec2[2]=0;}

//-------------------------------------------------------------------
  	 	ROMunitbd(18,26);  // Short circuit current time;
//-------------------------------------------------------------------
  	    tempH=((ROMtemp[75]) & 0x80);
  	   	if(tempH!=0){ROMdec[19]=tempH/128;}
  	   	else{ROMdec[19]=0;}// CB during discharge
//-------------------------------------------------------------------
  	   	tempH=((ROMtemp[75]) & 0x40);
  	   	if(tempH!=0){ROMdec[20]=tempH/64;}
  	   	else{ROMdec[20]=0;}// CB during charge
//-------------------------------------------------------------------
  	   // CB min,max voltage and CB min, max delta
  	   	ROMv1bd(21,28);
  	   	ROMv1bd(22,30);
  	    tempH=((ROMtemp[33]) & 0x0F);
  	  	ROMdec[23]=((ROMtemp[32]+(tempH*256))*(v1*10))+0.5;
  	    tempH=((ROMtemp[35]) & 0x0F);
  	    ROMdec[24]=((ROMtemp[34]+(tempH*256))*(v1*10))+0.5;
//-------------------------------------------------------------------
  	   	ROMunitbd(25,36);    	  // CB ON Time;
//-------------------------------------------------------------------
  	    ROMunitbd(26,38);  // CB OFF Time;
//-------------------------------------------------------------------
  	    //dac ROMdec[27]=ROMtemp[83];
  	    tempH=((ROMtemp[65]) & 0x0F);
  	    ROMdec[27]=(ROMtemp[64]+(tempH*256))*v2+0.5;

//-------------------------------------------------------------------
        //tempH=((ROMtemp[65]) & 0x0F);
        //ROMdec[28]=(ROMtemp[64]+(tempH*256))*v2+0.5;
  	    tempH=ROMtemp[85];
  	    ROMdec[28]=(ROMtemp[84]+(tempH*256));


//-------------------------------------------------------------------
        //tempH=((ROMtemp[67]) & 0x0F);
        //ROMdec[29]=(ROMtemp[66]+(tempH*256))*v2+0.5;
  	    tempH=ROMtemp[87];
  	    ROMdec[29]=(ROMtemp[86]+(tempH*256));
//-------------------------------------------------------------------
        ROMv1bd(30,68); // Low voltage sleep
//-------------------------------------------------------------------
  	    tempH=((ROMtemp[71]) & 0x03);
  	    ROMdec[31]=(ROMtemp[70]+(tempH*256));  // Low voltage sleep delay
  	    tempL=((ROMtemp[71]) & 0x04);
  	    if(tempL==0){ROMunit[31]="ms";}
  	    if(tempL==4){ROMunit[31]="s";}
//-------------------------------------------------------------------
  	    tempH=((ROMtemp[72]) & 0x0F);
  	    ROMdec[32]=tempH;  // Doze Time
//-------------------------------------------------------------------
  	    tempH=((ROMtemp[72]) & 0xF0);
  	    ROMdec[33]=tempH;  // Sleep Time
//-------------------------------------------------------------------
}


//------------------------------------------------------------------------------------------
//convert decimal to binary
//------------------------------------------------------------------------------------------

void ROMv1db(unsigned int pp,unsigned int loop22)
{
	uint16_t temp16;
	temp16 = (ROMdec[pp]/v1)+0.5;
	ROMtemp[loop22]=(uint8_t)temp16;
	ROMtemp[loop22+1]=((uint8_t)(temp16>>8));
}
void ROMunitdb(unsigned int pp,unsigned int loop22,unsigned int timeu)
{
	uint16_t temp16;
	temp16 = ROMdec[pp];
	ROMtemp[loop22]=(uint8_t)temp16;
    ROMtemp[loop22+1]=(uint8_t)(temp16>>8)+timeu;
}


void ROMconvertdb(void)
{


static float const v2=0.4395604;
uint16_t temp16;
unsigned int loop,loop2;

//-------------------------------------------------------------------

    ROMtemp[80]=ROMdec[0];  //preset cell type stored in user eeprom

//-------------------------------------------------------------------

	if(ROMdec[1]==3){ROMtemp[73]=131;}  //number of cells
	if(ROMdec[1]==4){ROMtemp[73]=195;}
	if(ROMdec[1]==5){ROMtemp[73]=199;}
	if(ROMdec[1]==6){ROMtemp[73]=231;}
	if(ROMdec[1]==7){ROMtemp[73]=239;}
	if(ROMdec[1]==8){ROMtemp[73]=255;}
//-------------------------------------------------------------------
	 //capacity
	temp16 = ROMdec[2];
	ROMtemp[81]=(uint8_t)temp16;
	tempL=((ROMtemp[82]) & 0x80);
	ROMtemp[82]=(uint8_t)(temp16>>8)+(tempL*128); //for capacitor or battery

//-------------------------------------------------------------------
	// Overvoltage threshold;
	ROMv1db(3,0);
	ROMtemp[1]=ROMtemp[1]+16;//the +16 is for 1ms

//-------------------------------------------------------------------
  	//Overvoltage delay
 	ROMunitdb(4,16,8); //the 8 is for seconds
//-------------------------------------------------------------------
 	// Overvoltage recovery;
 	ROMv1db(5,2);
//-------------------------------------------------------------------
    // Undervoltage threshold;
    ROMv1db(6,4);
    ROMtemp[5]=ROMtemp[5]+16; //the +16 is for 1ms
//-------------------------------------------------------------------
 	// Undervoltage delay;
 	ROMunitdb(7,18,8);// the 8 is for seconds
//-------------------------------------------------------------------
 	loop2=6;    // Undervoltage recovery, Overvoltage Lock-out, Undervoltage Lock-out, EOC,  Low voltage charge;
 	 	     for(loop=8;loop<13;loop++)
 	 	     {
 	 	    	ROMv1db(loop,loop2);
 	 	        loop2=loop2+2;
 	 	     }

//-------------------------------------------------------------------
 	ROMdec[13]=ROMdec2[0];
 	ROMdec[15]=ROMdec2[1];
 	ROMdec[17]=ROMdec2[2];
//-------------------------------------------------------------------
 	ROMunitdb(14,24,4); // Charge Current Time   the 4 is for ms
    ROMtemp[25]=ROMtemp[25]+ROMdec[13]*16;  //Charge current;
//-------------------------------------------------------------------
    ROMunitdb(16,22,4);   // Discharge Current Time  the 4 is for ms
    ROMtemp[23]=ROMtemp[23]+ROMdec[15]*16;  //Discharge current
//-------------------------------------------------------------------
    ROMunitdb(18,26,0);  // Short circuit current time, the 0 for us
    ROMtemp[27]=ROMtemp[27]+ROMdec[17]*16;//Short circuit current
//-------------------------------------------------------------------
  	// CB during discharge
  	ROMtemp[75]=ROMdec[19]*128;
  	// CB during charge
  	ROMtemp[75]=ROMtemp[75]+ROMdec[20]*64;
//-------------------------------------------------------------------
  	// CB min voltage;
  	ROMv1db(21,28);
//-------------------------------------------------------------------
  	// CB max voltage;
  	ROMv1db(22,30);
//-------------------------------------------------------------------
    // CB min delta;
  	temp16 = (ROMdec[23]/(v1*10))+0.5;
  	ROMtemp[32]=(uint8_t)temp16;
  	ROMtemp[33]=((uint8_t)(temp16>>8));
//-------------------------------------------------------------------
    // CB max delta;
    temp16 = (ROMdec[24]/(v1*10))+0.5;
    ROMtemp[34]=(uint8_t)temp16;
    ROMtemp[35]=((uint8_t)(temp16>>8));
//-------------------------------------------------------------------
  	// CB ON Time;
  	ROMunitdb(25,36,8);  // the 8 is for seconds

//-------------------------------------------------------------------
    // CB ON Time;
    ROMunitdb(26,38,8);  // the 8 is for seconds
//-------------------------------------------------------------------
   //dac ROMtemp[83]=ROMdec[27];
   //internal overtemp and recovery
    temp16 = (ROMdec[27]/v2)+0.5;
    ROMtemp[64]=(uint8_t)temp16;
    ROMtemp[65]=(uint8_t)(temp16>>8);

    temp16 = ((ROMdec[27]-40)/v2)+0.5;
    ROMtemp[66]=(uint8_t)temp16;
    ROMtemp[67]=(uint8_t)(temp16>>8);

//-------------------------------------------------------------------
    //dac temp16 = (ROMdec[28]/v2)+0.5;
    //ROMtemp[64]=(uint8_t)temp16;
    //ROMtemp[65]=(uint8_t)(temp16>>8);
    temp16 = ROMdec[28];
    ROMtemp[84]=(uint8_t)temp16;
    ROMtemp[85]=(uint8_t)(temp16>>8);

//-------------------------------------------------------------------
    //temp16 = (ROMdec[29]/v2)+0.5;
    //ROMtemp[66]=(uint8_t)temp16;
    //ROMtemp[67]=(uint8_t)(temp16>>8);
    temp16 = ROMdec[29];
    ROMtemp[86]=(uint8_t)temp16;
    ROMtemp[87]=(uint8_t)(temp16>>8);

//-------------------------------------------------------------------
  	// Low voltage sleep
  	ROMv1db(30,68);
//-------------------------------------------------------------------
  	// Low voltage sleep delay
  	temp16 = ROMdec[31];
  	ROMtemp[70]=(uint8_t)temp16;
  	ROMtemp[71]=(uint8_t)(temp16>>8)+ 0xFC;  //FC is fixed for 31 sec WD and seconds
//-------------------------------------------------------------------
    ROMtemp[72]=ROMdec[32];

  	ROMtemp[72]=ROMtemp[72]+ROMdec[33];  // Sleep Time
//-------------------------------------------------------------------

}

//========================================================================================
// *****************          Menu                *****************
//========================================================================================
void mainmenu1(void)
{
	if ((c2<25)||(c2==0)){return;}

	unsigned int v2,s,i2;
    s=5;
    refresh1=1;  //to refresh the parameter settings


    if (c3<1){c3=s;}
	if (c3>=(s+1)){c3=1;}


	LCD5110_set_XY(0,0);
	LCD5110_write_Dec1_neg(1,0,c3);
	LCD5110_set_XY(1,0);
	LCD5110_write_string_neg("/");
	LCD5110_set_XY(2,0);
	LCD5110_write_Dec1_neg(1,0,s);
	LCD5110_set_XY(3,0);
    LCD5110_write_string_neg("  MainMenu  ");

	if ( (c3<=(s+1)) && (c3>=1) )
		                    {
		            v2=c3+(s-6);
		            if (v2<s)
		            {
		            LCD5110_set_XY(0,1); //dacus test menu
				    LCD5110_write_string(mainmenu[0][0]);
				    LCD5110_set_XY(0,2); //dacus test menu
				    LCD5110_write_string(mainmenu[1][0]);
				    LCD5110_set_XY(0,3); //dacus test menu
				    LCD5110_write_string(mainmenu[2][0]);
				    LCD5110_set_XY(0,4); //dacus test menu
				    LCD5110_write_string(mainmenu[3][0]);
				    LCD5110_set_XY(0,5); //dacus test menu
				    LCD5110_write_string(mainmenu[4][0]);
				    LCD5110_set_XY(0,c3); //dacus test menu
				    LCD5110_write_string_neg(mainmenu[c3-1][0]);

				    for(i2=0;i2<15;i2++)  //clear text buffer for the first 10 locations
				    {
				    chr[i2]=0x20;
				    }

				    helptext=mainmenu[c3-1][1];
				    scrolllenght=14;
				    nr1=0;
				    c33=c3;
				    z=0;
				    i4=0;   //delay time for scrolling text
				    i5=1;   //activate scrolling text

				    while (*helptext!='\0')
				                  {
				                     chr[z]=*helptext;
				                     z++;
				                     helptext++;
				                  }
				    if (z>=8)
				    {
				        for (i2=0;i2<scrolllenght;i2++)
				        {
				         chr[z+i2]=chr[i2];
				        }

				    }




		            }

      }


	if (BOK==1)
		{
		BOK=0;
		c2=c3;
		c333=c3;
		c3=1;
		i5=0;
		LCD5110_clear();
         	}

	}

//---------------------------------------------------------------------------------------
// *************************         Monitor            **********************
//---------------------------------------------------------------------------------------

void monitor (void)

{

	unsigned int k,k1;

	for (k=0;k<16;k=k+2)   //cell voltages
	   {
	    c8=((((RAMrx[17+k]*256)+RAMrx[16+k])*(1.8*8))/(4095*3))*1000;
	     if (k>=2)
	     {
	     BV[k/2][2]=BV[k/2][1];
	     BV[k/2][1]=BV[k/2][0];
	     BV[k/2][0]=c8;
	     }
	     else
	     {
	     BV[0][2]=BV[0][1];
	     BV[0][1]=BV[0][0];
	     BV[0][0]=c8;
	     }

	    }

	for(k=0;k<8;k++)
	{
		if ((BV[k][3]>(((BV[k][0]+BV[k][1]+BV[k][2])/3))-2) && (BV[k][3]<(((BV[k][0]+BV[k][1]+BV[k][2])/3))+2) && (filter2<30)) {filter2++;}
				    	     else
				    	     {
				    	    	 BV[k][3]=((BV[k][0]+BV[k][1]+BV[k][2])/3);
				    	    	 filter2=0;
				    	     }
	}

	BVmin[0]=BV[0][3];
	BVmin[1]=0;
	BVmax[0]=BV[0][3];
	BVmax[1]=0;
	for (k=1;k<8;k++)  //find min and max cell
	{
		if((BV[k][3]<BVmin[0])&&(BV[k][3]>100))
		{BVmin[0]=BV[k][3];BVmin[1]=k;}
		if(BV[k][3]>BVmax[0])
		{BVmax[0]=BV[k][3];BVmax[1]=k;}
	}
   BVdelta=BVmax[0]-BVmin[0];


   //cell balancing.

   k1=0;
   for (k=1;k<129;k=k+k)
   {
	  tempH=((RAMrx[4]) & k);
	  if (tempH>0)
	  {CB[k1]=1;}
	  else
	  {CB[k1]=0;}
	  k1++;
   }


   // error flags

   k1=0;
      for (k=1;k<9;k=k+k)    //OV, OVLO, UV, UVLO
      {
   	  tempH=((RAMrx[0]) & k);
   	  if (tempH>0)
   	  {EF[k1]=1;}
   	  else
   	  {EF[k1]=0;}
   	  k1++;
      }

      for (k=1;k<33;k=k+k)   //IOT, COC, DOC, DSC, CELLF, OPEN
      {
      tempH=((RAMrx[1]) & k);
      if (tempH>0)
      {EF[k1]=1;}
      else
      {EF[k1]=0;}
      k1++;
      }

       tempH=((RAMrx[1]) & 128);  //EOC
       if (tempH>0)
       {EF[13]=1;}
       else
       {EF[13]=0;}

       tempH=((RAMrx[2]) & 32);  //ECC_Faill
       if (tempH>0)
       {EF[11]=1;}
       else
       {EF[11]=0;}

       tempH=((RAMrx[2]) & 128); //LVC
       if (tempH>0)
       {EF[10]=1;}
       else
       {EF[10]=0;}

       tempH=((RAMrx[6]) & 1);   //DFET
       if (tempH>0)
       {EF[14]=1;DFET=1;}
       else
       {EF[14]=0;DFET=0;}

       tempH=((RAMrx[6]) & 2);   //CFET
       if (tempH>0)
       {EF[12]=1;CFET=1;}
       else
       {EF[12]=0;CFET=0;}

       UVlock=ROMdec[10]*10;   //undervoltage lock for the bar graph low limit
       OVlock=ROMdec[9]*10;    //over voltage lock for the bar graph high limit



	    tempH=((RAMrx[2]) & 0x0C);  //+/- charge or discharge
                    if (tempH==0){direction=" ";Cdirection=0;}
          	        if (tempH==4){direction="+";Cdirection=1;}
         	        if (tempH==8){direction="-";Cdirection=0;}



	    tempL=((RAMrx[5]) & 0x30);  //read the battery current amplifier gain
	    if (tempL==0){gain=50;}  //50
	    if (tempL==16){gain=5;}  //5
        if (tempL==32 || tempL==48){gain=500;}  //500

        ITEMP=((RAMrx[33]*256)+RAMrx[32]);  //Internal temp

        ETEMP=((RAMrx[35]*256)+RAMrx[34]);  //ext temp T1 (TEMPI on the 6 pin connector)

        c8=((RAMrx[37]*256)+RAMrx[36]);  //ext temp T2 that is solar current from MAX8040

	    c9=(RAMrx[15]*256)+RAMrx[14];  //battery current


	    if ((gain==50)&&(c9>=250))
	    {
	    c9cal=c9calx50;
	    rangeswitch=0;
	    }
	    if ((gain==50)&&(c9<250))
	    {
	    rangeswitch++; //on high noise loads like inverters stay on x50 gain
	    if (rangeswitch>200){
	    RAMwrite(0x85,32); //set current amplifier gain x50 (00) x5 (16) x500 (32) x500 (48)
	    j=0;
	    filter3=0;
	    rangeswitch=0;
	    }
	    c9cal=c9calx50;
	    }
	    if ((gain==500)&&(c9<=3500))
	    {
	    c9cal=c9calx500;
	    }
	    if ((gain==500)&&(c9>3500))
	    {
	    RAMwrite(0x85,0); //set current amplifier gain x50 (00) x5 (16) x500 (32) x500 (48)
	    c9cal=c9calx500;
	    j=0;
	    filter3=0;
	    }




	    //LCD5110_set_XY(2,1);
        //LCD5110_write_Dec1_neg(4,0,c9);

                    if ((tempH!=4) && (c9>c9cal+3))
                    {c9=c9-c9cal;}
                    if (tempH==4)
                    {c9=c9+c9cal;}
                    if (c9<=c9cal+3)
                    {c9=0;}


                    if (c8>c8cal+8)
                    {c8=c8-c8cal+10;}
                    else{c8=0;}


	   BatteryVoltage=((BV[0][3]+BV[1][3]+BV[2][3]+BV[3][3]+BV[4][3]+BV[5][3]+BV[6][3]+BV[7][3])/10); //sum of cell voltage.

       BatteryVoltage2=((((RAMrx[39]*256)+RAMrx[38])*(1.925*32))/(4095))*100;  //direct battery reading

	    if (sj>=4)
	            {
	    	    SolarCurrent=(((((cs7+sc1+sc2)/12)*1.8)/(4095*PVshunt*PVgain))*10);   //PV current
	            sj=0;
	            sc2=sc1;
	            sc1=cs7;
	            cs7=0;
	            filter1=0;
	            }
	            else
	            {
	            if ((c8<((cs7+sc1+sc2)/12)-3)&&(c8>((cs7+sc1+sc2)/12)+3)&&(filter1<=4)){filter1++;}
	            else {cs7=c8+cs7;sj++;}
	            }


	    if (j>=32)
	        {
	        BatteryCurrent=(((((c7+bc1+bc2)/96)*1.8)/(4095*Battshunt*gain))*10);  //Battery current
	    	j=0;
	        bc2=bc1;
	        bc1=c7;
	        c7=0;
	        filter3=0;
	        }
	        else
	        {
	        if ((c9<((c7+bc1+bc2)/96)-3)&&(c9>((c7+bc1+bc2)/96)+3)&&(filter3<=4)){filter3++;}
	        else {c7=c9+c7;j++;}
	        }

	    //calculate load current

	        if (tempH!=4)
	  	    {
	  	    	LoadCurrent=(SolarCurrent)+BatteryCurrent;
	  	    }
	  	    if (tempH==4)
	  	    {
	  	       if (SolarCurrent>=BatteryCurrent)
	  	       {
	  	    	LoadCurrent=(SolarCurrent)-BatteryCurrent;
	  	       }
	  	       else LoadCurrent=0;
	  	    }






	    BatteryW=(BatteryCurrent*BatteryVoltage)/1000; //Battery power [W]
	    SolarW=(SolarCurrent*BatteryVoltage)/1000; //Solar power [W]
	    LoadW=(LoadCurrent*BatteryVoltage)/1000; //Battery power [W]

	    //calculate SOC and BatteryAh BatteryWh counters
	    if(zz2==1)
	    {

	    	if (tempH!=4)
	    	{
	    	BatterySOC=BatterySOC-((BatteryCurrent*1000)/36);
	    	BatteryuAh=((BatteryCurrent*1000)/36)+BatteryuAh;
	    	BatteryuWh=((((BatteryCurrent*1000)/36)*BatteryVoltage)/100)+BatteryuWh;
	    	BatteryuWh2=((((BatteryCurrent*1000)/36)*BatteryVoltage)/100)+BatteryuWh2;
	    	BatterymAh=(BatteryuAh/1000);
	    	BatteryWh=(BatteryuWh/1000000);
	    	BatterykWh2=(BatteryuWh2/1000000000);
	    	}
	    	else
	    	{
	    	BatterySOC=((BatteryCurrent*1000)/36)+BatterySOC;
	    	}

	    	SolaruAh=((SolarCurrent*1000)/36)+SolaruAh;
	    	SolarmAh=SolaruAh/1000;
	    	LoaduAh=((LoadCurrent*1000)/36)+LoaduAh;
	    	LoadmAh=LoaduAh/1000;
	    	SolaruWh=((((SolarCurrent*1000)/36)*BatteryVoltage)/100)+SolaruWh;
	    	SolaruWh2=((((SolarCurrent*1000)/36)*BatteryVoltage)/100)+SolaruWh2;
	    	SolarWh=(SolaruWh/1000000);
	    	SolarkWh2=(SolaruWh2/1000000000);
	    	LoaduWh=((((LoadCurrent*1000)/36)*BatteryVoltage)/100)+LoaduWh;
	    	LoadWh=(LoaduWh/1000000);



	    		    	if ((BatterySOC>=BattCapacity*2)||(EF[13]==1)){BatterySOC=BattCapacity*2;} //I need to add end of charge as condition
	    		    	if (BatterySOC<=BattCapacity){BatterySOC=BattCapacity;} //cell undervoltage needed as condition.
	    		    	if (BatterymAh>=10000000){BatterymAh=0;BatteryuAh=0;}  //reset BatteryAh counter when over 9999Ah
	    		    	if (BatteryWh>=10000000){BatteryWh=0;BatteryuWh=0;}  //reset BatteryWh counter when over 9999kWh
	    		    	if (BatterykWh2>=65000){BatterykWh2=0;BatteryuWh2=0;}  //reset BatteryWh counter when over 64999kWh
	    		    	if (SolarmAh>=10000000){SolarmAh=0;SolaruAh=0;}  //reset SolarAh counter when over 9999Ah
	    		    	if (SolarWh>=10000000){SolarWh=0;SolaruWh=0;}  //reset SolarWh counter when over 9999kWh
	    		    	if (SolarkWh2>=65000){SolarkWh2=0;SolaruWh2=0;}  //reset SolarWh counter when over 64999kWh
	    		    	if (LoadmAh>=10000000){LoadmAh=0;LoaduAh=0;}  //reset LoadAh counter when over 9999Ah
	    		    	if (LoadWh>=10000000){LoadWh=0;LoaduWh=0;}  //reset LoadWh counter when over 9999kWh


                        if ((ROMdec[0]==6)||(ROMdec[0]==7))
                        {
                            if ((BVmin[0]/10)>(ROMdec[6]))
                            		{
                        	SOC1=(((BVmin[0]/10)*(BVmin[0]/10))-(ROMdec[6]*ROMdec[6]))/(((ROMdec[11]*ROMdec[11])-(ROMdec[6]*ROMdec[6]))/100);
                            if (SOC1>=100) {SOC1=100;}
                            if (SOC1<=1) {SOC1=1;}
                            		}
                            if ((BVmin[0]/10)<=(ROMdec[6])) {SOC1=0;}
                        }
                        else
                        {
                        	SOC1=((BatterySOC-BattCapacity)*100)/BattCapacity;
                        }

	    }




zz2=0; //reset the 1 second timer for the energy counter.

}


//---------------------------------------------------------------------------------------
// *************************         Monitoring         **********************
//---------------------------------------------------------------------------------------

void monitoring(void)
	{

	if (c2!=1){return;}

	buttondir=1;

	unsigned int k,ky,ki,s;
    s=7;

    if (b3==1||b4==1)
    {
    LCD5110_clear();
    delayms1(1);
    }

    monitor();

    if (c3<1){c3=s;}
    if (c3>=(s+1)){c3=1;}


    LCD5110_set_XY(0,0);
    LCD5110_write_Dec1_neg(1,0,c3);
    LCD5110_set_XY(1,0);
    LCD5110_write_string_neg("/");
    LCD5110_set_XY(2,0);
    LCD5110_write_Dec1_neg(1,0,s);
    LCD5110_set_XY(3,0);
    LCD5110_write_string_neg(" ");
    if (SOC1<10)
    {
    LCD5110_write_string_neg("  ");
    LCD5110_write_Dec1_neg(1,0,SOC1);
    }
    if ((SOC1>=10)&&(SOC1<100))
    {
    LCD5110_write_string_neg(" ");
    LCD5110_write_Dec1_neg(2,0,SOC1);
    }
    if (SOC1==100)
    {
    LCD5110_write_Dec1_neg(3,0,SOC1);
    }
    LCD5110_LCD_write_byte(0x00,1);
    LCD5110_write_string("%");
    LCD5110_LCD_write_byte(0x00,1);
    LCD5110_LCD_write_byte(0x00,1);
    LCD5110_bar((SOC1/3.3),30);
    LCD5110_LCD_write_byte(0x18,1);
    if (BOK==0){release1=1;}
    if (BBack==0){release2=1;}




if (c3==1)  //first monitor page

{


for (k=0;k<4;k++)

{
	if (BV[k][3]!=0)
	    {
	LCD5110_set_XY(0,1+k);
    if ((BVmin[1]==k)||(BVmax[1]==k))
    {
    LCD5110_write_char_neg(k+23);
    }
    else
    {
    LCD5110_write_char(k+23);
    }

    LCD5110_write_Dec(4,3,BV[k][3]);

    if (CB[k]==1)
    {
    LCD5110_write_string("<");
    }
    else
    {
    LCD5110_write_string(" ");
    }

}

    if (BV[k+4][3]!=0)
        {
    LCD5110_set_XY(7,1+k);
    if ((BVmin[1]==k+4)||(BVmax[1]==k+4))
    {
    LCD5110_write_char_neg(k+27);
    }
    else
    {
    LCD5110_write_char(k+27);
    }

    LCD5110_write_Dec(4,3,BV[k+4][3]);

    if (CB[k+4]==1)
        {
        LCD5110_write_string("<");
        }
        else
        {
        LCD5110_write_string(" ");
        }
        }

}




    LCD5110_set_XY(8,5);
    if (BatteryVoltage>=1000){
        LCD5110_write_Dec(4,2,BatteryVoltage);
        LCD5110_write_string("V");
        }
        else
        {
        LCD5110_write_Dec(3,2,BatteryVoltage);
        LCD5110_write_string("V");
        }


    LCD5110_set_XY(0,5);
    LCD5110_write_string("D");

    if (BVdelta>=1000){
   	LCD5110_write_Dec1(4,0,BVdelta);  // battery pack current
   	LCD5110_write_string("mV");
   	}
    if ((BVdelta>=100)&&(BVdelta<1000)){
    LCD5110_write_Dec1(3,0,BVdelta);  // battery pack current
    LCD5110_write_string("mV ");
    }
   	if ((BVdelta>=10)&&(BVdelta<100)){
   	LCD5110_write_Dec1(2,0,BVdelta);  // battery pack current
   	LCD5110_write_string("mV  ");
   	}
   	if (BVdelta<10)
   	{
   	LCD5110_write_Dec1(1,0,BVdelta);  // battery pack current
   	LCD5110_write_string("mV   ");
   	}



}

if (c3==2)

{



	for (k=0;k<4;k++)
	{

	    LCD5110_set_XY(0,1+k);
	    if ( ((((BV[k][3]-UVlock)/((OVlock-UVlock)/30)))>0) && ((((BV[k][3]-UVlock)/((OVlock-UVlock)/30)))<=31)&&(BV[k][3]!=0) )
	    {
	    LCD5110_write_char(k+23);
	    LCD5110_bar((((BV[k][3]-UVlock)/((OVlock-UVlock)/30))),31);
        LCD5110_write_string(" ");
	    }
	    LCD5110_set_XY(7,1+k);
	    if ( ((((BV[k+4][3]-UVlock)/((OVlock-UVlock)/30)))>0) && ((((BV[k+4][3]-UVlock)/((OVlock-UVlock)/30)))<=31)&&(BV[k+4][3]!=0) )
	    {
        LCD5110_write_char(k+27);
        LCD5110_bar((((BV[k+4][3]-UVlock)/((OVlock-UVlock)/30))),31);
	    }
	    //      LCD5110_write_string(" ");
		}
	LCD5110_set_XY(8,5);
	    if (BatteryVoltage>=1000){
	        LCD5110_write_Dec(4,2,BatteryVoltage);
	        LCD5110_write_string("V");
	        }
	        else
	        {

	        LCD5110_write_Dec(3,2,BatteryVoltage);
	        LCD5110_write_string("V");
	        }

	    LCD5110_set_XY(0,5);
	        LCD5110_write_string(direction);

	        if (BatteryCurrent>=1000){
	       	LCD5110_write_Dec1(4,1,BatteryCurrent);  // battery pack current
	       	LCD5110_write_string("A");
	       	}
	       	if ((BatteryCurrent>=100)&&(BatteryCurrent<1000)){
	       	LCD5110_write_Dec1(3,1,BatteryCurrent);  // battery pack current
	       	LCD5110_write_string("A ");
	       	}
	       	if (BatteryCurrent<100)
	       	{
	       	LCD5110_write_Dec1(2,1,BatteryCurrent);  // battery pack current
	       	LCD5110_write_string("A  ");
	       	}


}



if (c3==3)
{
	        LCD5110_set_XY(0,5);
	        LCD5110_write_string_neg("TI");
		    LCD5110_write_Dec1(3,0,ITEMP);
		    LCD5110_write_string("mV");
		    LCD5110_write_string_neg("TE");
		    LCD5110_write_Dec1(4,0,ETEMP);
		    LCD5110_write_string("b");

	LCD5110_set_XY(0,1);
	LCD5110_write_string("Voltage ");
	LCD5110_set_XY(8,1);
	    if (BatteryVoltage>=1000){
	    LCD5110_write_Dec(4,2,BatteryVoltage);
	    LCD5110_write_string("V");
	    }
	    else
	    {
	    LCD5110_write_Dec(3,2,BatteryVoltage);
	    LCD5110_write_string("V");
	    }

	LCD5110_set_XY(0,2);
	LCD5110_bar((BatteryVoltage-(ROMdec[10]*ROMdec[1]))/(((ROMdec[9]-ROMdec[10])*8)/80),82);

	LCD5110_set_XY(0,3);
	LCD5110_write_string("Current ");
	LCD5110_set_XY(8,3);
	LCD5110_write_string(direction);
	    if (BatteryCurrent>=100){
	    LCD5110_write_Dec1(3,1,BatteryCurrent);  // battery pack current
	    LCD5110_write_string("A");
	    }
	    else
	    {
	    LCD5110_write_Dec1(2,1,BatteryCurrent);  // battery pack current
	    LCD5110_write_string("A ");
	    }

    LCD5110_set_XY(0,4);
    if (Cdirection==1)
    {
	LCD5110_bar(((BatteryCurrent*(8000/48))/1000),82); //ROMdec[13] can also be used
    }
    if (Cdirection==0)
    {
    LCD5110_bar(((BatteryCurrent*(8000/96))/1000),82); //ROMdec[15] can also be used
    }




}

if (c3==4)
{
       ki=0;
        for (k=0;k<5;k++)
        {
        for (ky=0;ky<11;ky=ky+5)
        {
            LCD5110_set_XY(ky,k+1);
	        if (EF[ki]==0)
	        {
	   	    LCD5110_write_string(EFT[ki]);
	        }
	   	    if (EF[ki]==1)
	   	    {
	   	    LCD5110_write_string_neg(EFT[ki]);
	   	    }
            ki++;
        }
        }
        for (k=0;k<5;k++)
        {
        LCD5110_set_XY(4,k+1);
        LCD5110_write_string(" ");
        LCD5110_set_XY(9,k+1);
        LCD5110_write_string(" ");
        }


}

//------------------------------------------------------------------------------
if (c3==5)  // page 5 Battery power and energy
{


	//Battery Voltage

		    LCD5110_set_XY(0,1);
		    LCD5110_write_string_neg("BattV  ");
		    LCD5110_set_XY(7,1);
		    if (BatteryVoltage>=1000){
		    LCD5110_write_Dec1(4,2,BatteryVoltage);
		    LCD5110_write_string("V ");
		    }
		    if (BatteryVoltage<1000)
		    {
		    LCD5110_write_Dec1(3,2,BatteryVoltage);
		    LCD5110_write_string(" V ");
		    }


	// Battery Current
	        LCD5110_set_XY(0,2);
		    LCD5110_write_string_neg("BattC ");
		    LCD5110_set_XY(6,2);
		    LCD5110_write_string_neg(direction);
		      		       		    if (BatteryCurrent>=1000){
		      		       		    LCD5110_write_Dec1(4,1,BatteryCurrent);  // battery pack current
		      		       		    LCD5110_write_string("A ");
		      		       		    }
		      		       		    if ((BatteryCurrent>=100)&&(BatteryCurrent<1000)){
		      		       		    LCD5110_write_Dec1(3,1,BatteryCurrent);  // battery pack current
		      		       		    LCD5110_write_string(" A ");
		      		       		    }
		      		       		    if (BatteryCurrent<100)
		      		       		    {
		      		       		    LCD5110_write_Dec1(2,1,BatteryCurrent);  // battery pack current
		      		       		    LCD5110_write_string("  A ");
		      		       		    }






	    //Battery energy & power
	    //Power
	    LCD5110_set_XY(0,3);
		LCD5110_write_string_neg("BattP ");
	    LCD5110_set_XY(6,3);
	    LCD5110_write_string_neg(direction);

	    	    if (BatteryW>=1000){
	    	    LCD5110_write_Dec1(4,0,BatteryW);
	    	    LCD5110_write_string(" W ");
	    	    }

	    	    if ((BatteryW>=100)&&(BatteryW<1000)){
	    	    LCD5110_write_Dec1(3,0,BatteryW);
	    	    LCD5110_write_string("  W ");
	    	    }

	    	    if ((BatteryW>=10)&&(BatteryW<100)){
	    	    LCD5110_write_Dec1(2,0,BatteryW);
	    	    LCD5110_write_string("   W ");
	    	    }

	    	    if (BatteryW<10)
	    	    {
	    	    LCD5110_write_Dec1(1,0,BatteryW);
	    	    LCD5110_write_string("    W ");
	    	    }

         //Energy Ah
	    LCD5110_set_XY(0,4);
	    LCD5110_write_string_neg("BattAh ");
	    LCD5110_set_XY(7,4);


	            if (BatterymAh>=1000000){
	    	    LCD5110_write_Dec1(4,0,BatterymAh/1000);
	    	    LCD5110_write_string(" Ah");
	            }
	            if ((BatterymAh>=100000)&&(BatterymAh<1000000)){
	    	    LCD5110_write_Dec1(4,1,BatterymAh/100);
	    	    LCD5110_write_string("Ah");
	            }
	            if ((BatterymAh>=10000)&&(BatterymAh<100000)){
	    	    LCD5110_write_Dec1(4,2,BatterymAh/10);
	    	    LCD5110_write_string("Ah");
	            }
	            if ((BatterymAh>=1000)&&(BatterymAh<10000)){
	       	    LCD5110_write_Dec1(4,3,BatterymAh);
	       	    LCD5110_write_string("Ah");
	       	    }
	       	    if ((BatterymAh>=100)&&(BatterymAh<1000)){
	      	    LCD5110_write_Dec1(3,0,BatterymAh);
	       	    LCD5110_write_string(" mAh");
	       	    }
	      	    if ((BatterymAh>=10)&&(BatterymAh<100)){
	       	    LCD5110_write_Dec1(2,0,BatterymAh);
	       	    LCD5110_write_string("  mAh");
	       	    }
	       	    if (BatterymAh<10)
	       	    {
	       	    LCD5110_write_Dec1(1,0,BatterymAh);
	       	    LCD5110_write_string("   mAh");
	       	    }

	      	//Energy Wh

	       	      LCD5110_set_XY(0,5);
	       	      LCD5110_write_string_neg("BattWh ");
	      	      LCD5110_set_XY(7,5);


	      	                if (BatteryWh>=1000000){
	      		    	    LCD5110_write_Dec1(4,0,BatteryWh/1000);
	      		    	    LCD5110_write_string("kWh");
	      		            }
	      		            if ((BatteryWh>=100000)&&(BatteryWh<1000000)){
	      		    	    LCD5110_write_Dec1(3,0,BatteryWh/1000);
	      		    	    LCD5110_write_string(" kWh");
	      		            }
	      		            if ((BatteryWh>=10000)&&(BatteryWh<100000)){
	      		    	    LCD5110_write_Dec1(3,1,BatteryWh/100);
	      		    	    LCD5110_write_string("kWh");
	      		            }
	      		            if ((BatteryWh>=1000)&&(BatteryWh<10000)){
	      		       	    LCD5110_write_Dec1(3,2,BatteryWh/10);
	      		       	    LCD5110_write_string("kWh");
	      		       	    }
	      		       	    if ((BatteryWh>=100)&&(BatteryWh<1000)){
	      		      	    LCD5110_write_Dec1(3,0,BatteryWh);
	      		       	    LCD5110_write_string("  Wh");
	      		       	    }
	      		      	    if ((BatteryWh>=10)&&(BatteryWh<100)){
	      		       	    LCD5110_write_Dec1(2,0,BatteryWh);
	      		       	    LCD5110_write_string("   Wh");
	      		       	    }
	      		       	    if (BatteryWh<10)
	      		       	    {
	      		       	    LCD5110_write_Dec1(1,0,BatteryWh);
	      		       	    LCD5110_write_string("    Wh");
	      		       	    }


}

//---------------------------------------------------------------------

if (c3==6)  // page 6 Solar power and energy
{


	//Battery Voltage

		    LCD5110_set_XY(0,1);
		    LCD5110_write_string_neg("BattV  ");
		    LCD5110_set_XY(7,1);
		    if (BatteryVoltage>=1000){
		    LCD5110_write_Dec1(4,2,BatteryVoltage);
		    LCD5110_write_string("V ");
		    }
		    if (BatteryVoltage<1000)
		    {
		    LCD5110_write_Dec1(3,2,BatteryVoltage);
		    LCD5110_write_string(" V ");
		    }


	// Solar Current
	        LCD5110_set_XY(0,2);
		    LCD5110_write_string_neg("PVC    ");


		      		       		    if (SolarCurrent>=1000){
		      		       		    LCD5110_write_Dec1(4,1,SolarCurrent);  // battery pack current
		      		       		    LCD5110_write_string("A ");
		      		       		    }
		      		       		    if ((SolarCurrent>=100)&&(SolarCurrent<1000)){
		      		       		    LCD5110_write_Dec1(3,1,SolarCurrent);  // battery pack current
		      		       		    LCD5110_write_string(" A ");
		      		       		    }
		      		       		    if (SolarCurrent<100)
		      		       		    {
		      		       		    LCD5110_write_Dec1(2,1,SolarCurrent);  // battery pack current
		      		       		    LCD5110_write_string("  A ");
		      		       		    }






	    //Solar energy & power
	    //Power
	    LCD5110_set_XY(0,3);
		LCD5110_write_string_neg("PVP    ");


	    	    if (SolarW>=1000){
	    	    LCD5110_write_Dec1(4,0,SolarW);
	    	    LCD5110_write_string(" W ");
	    	    }
	    	    if ((SolarW>=100)&&(SolarW<1000)){
	    	    LCD5110_write_Dec1(3,0,SolarW);
	    	    LCD5110_write_string("  W ");
	    	    }
	    	    if ((SolarW>=10)&&(SolarW<100)){
	    	    LCD5110_write_Dec1(2,0,SolarW);
	    	    LCD5110_write_string("   W ");
	    	    }
	    	    if (SolarW<10)
	    	    {
	    	    LCD5110_write_Dec1(1,0,SolarW);
	    	    LCD5110_write_string("    W ");
	    	    }

         //Energy Ah
	    LCD5110_set_XY(0,4);
	    LCD5110_write_string_neg("PVAh   ");
	    LCD5110_set_XY(7,4);


	            if (SolarmAh>=1000000){
	    	    LCD5110_write_Dec1(4,0,SolarmAh/1000);
	    	    LCD5110_write_string(" Ah");
	            }
	            if ((SolarmAh>=100000)&&(SolarmAh<1000000)){
	    	    LCD5110_write_Dec1(4,1,SolarmAh/100);
	    	    LCD5110_write_string("Ah");
	            }
	            if ((SolarmAh>=10000)&&(SolarmAh<100000)){
	    	    LCD5110_write_Dec1(4,2,SolarmAh/10);
	    	    LCD5110_write_string("Ah");
	            }
	            if ((SolarmAh>=1000)&&(SolarmAh<10000)){
	       	    LCD5110_write_Dec1(4,3,SolarmAh);
	       	    LCD5110_write_string("Ah");
	       	    }
	       	    if ((SolarmAh>=100)&&(SolarmAh<1000)){
	      	    LCD5110_write_Dec1(3,0,SolarmAh);
	       	    LCD5110_write_string(" mAh");
	       	    }
	      	    if ((SolarmAh>=10)&&(SolarmAh<100)){
	       	    LCD5110_write_Dec1(2,0,SolarmAh);
	       	    LCD5110_write_string("  mAh");
	       	    }
	       	    if (SolarmAh<10)
	       	    {
	       	    LCD5110_write_Dec1(1,0,SolarmAh);
	       	    LCD5110_write_string("   mAh");
	       	    }

	      	//Energy Wh

	       	      LCD5110_set_XY(0,5);
	       	      LCD5110_write_string_neg("PVWh   ");
	      	      LCD5110_set_XY(7,5);


	      	                if (SolarWh>=1000000){
	      		    	    LCD5110_write_Dec1(4,0,SolarWh/1000);
	      		    	    LCD5110_write_string("kWh");
	      		            }
	      		            if ((SolarWh>=100000)&&(SolarWh<1000000)){
	      		    	    LCD5110_write_Dec1(3,0,SolarWh/1000);
	      		    	    LCD5110_write_string(" kWh");
	      		            }
	      		            if ((SolarWh>=10000)&&(SolarWh<100000)){
	      		    	    LCD5110_write_Dec1(3,1,SolarWh/100);
	      		    	    LCD5110_write_string("kWh");
	      		            }
	      		            if ((SolarWh>=1000)&&(SolarWh<10000)){
	      		       	    LCD5110_write_Dec1(3,2,SolarWh/10);
	      		       	    LCD5110_write_string("kWh");
	      		       	    }
	      		       	    if ((SolarWh>=100)&&(SolarWh<1000)){
	      		      	    LCD5110_write_Dec1(3,0,SolarWh);
	      		       	    LCD5110_write_string("  Wh");
	      		       	    }
	      		      	    if ((SolarWh>=10)&&(SolarWh<100)){
	      		       	    LCD5110_write_Dec1(2,0,SolarWh);
	      		       	    LCD5110_write_string("   Wh");
	      		       	    }
	      		       	    if (SolarWh<10)
	      		       	    {
	      		       	    LCD5110_write_Dec1(1,0,SolarWh);
	      		       	    LCD5110_write_string("    Wh");
	      		       	    }


}

//--------------------------------------------------------------------------

if (c3==7)  // page 7 Load power and energy
{


	//Battery Voltage

		    LCD5110_set_XY(0,1);
		    LCD5110_write_string_neg("BattV  ");
		    LCD5110_set_XY(7,1);
		    if (BatteryVoltage>=1000){
		    LCD5110_write_Dec1(4,2,BatteryVoltage);
		    LCD5110_write_string("V ");
		    }
		    if (BatteryVoltage<1000)
		    {
		    LCD5110_write_Dec1(3,2,BatteryVoltage);
		    LCD5110_write_string(" V ");
		    }


	// Load Current
	        LCD5110_set_XY(0,2);
		    LCD5110_write_string_neg("LoadC  ");


		      		       		    if (LoadCurrent>=1000){
		      		       		    LCD5110_write_Dec1(4,1,LoadCurrent);  // battery pack current
		      		       		    LCD5110_write_string("A ");
		      		       		    }
		      		       		    if ((LoadCurrent>=100)&&(LoadCurrent<1000)){
		      		       		    LCD5110_write_Dec1(3,1,LoadCurrent);  // battery pack current
		      		       		    LCD5110_write_string(" A ");
		      		       		    }
		      		       		    if (LoadCurrent<100)
		      		       		    {
		      		       		    LCD5110_write_Dec1(2,1,LoadCurrent);  // battery pack current
		      		       		    LCD5110_write_string("  A ");
		      		       		    }






	    //Load energy & power
	    //Power
	    LCD5110_set_XY(0,3);
		LCD5110_write_string_neg("LoadP  ");


	    	    if (LoadW>=1000){
	    	    LCD5110_write_Dec1(4,0,LoadW);
	    	    LCD5110_write_string(" W ");
	    	    }
	    	    if ((LoadW>=100)&&(LoadW<1000)){
	    	    LCD5110_write_Dec1(3,0,LoadW);
	    	    LCD5110_write_string("  W ");
	    	    }
	    	    if ((LoadW>=10)&&(LoadW<100)){
	    	    LCD5110_write_Dec1(2,0,LoadW);
	    	    LCD5110_write_string("   W ");
	    	    }
	    	    if (LoadW<10)
	    	    {
	    	    LCD5110_write_Dec1(1,0,LoadW);
	    	    LCD5110_write_string("    W ");
	    	    }

         //Energy Ah
	    LCD5110_set_XY(0,4);
	    LCD5110_write_string_neg("LoadAh ");
	    LCD5110_set_XY(7,4);


	            if (LoadmAh>=1000000){
	    	    LCD5110_write_Dec1(4,0,LoadmAh/1000);
	    	    LCD5110_write_string(" Ah");
	            }
	            if ((LoadmAh>=100000)&&(LoadmAh<1000000)){
	    	    LCD5110_write_Dec1(4,1,LoadmAh/100);
	    	    LCD5110_write_string("Ah");
	            }
	            if ((LoadmAh>=10000)&&(LoadmAh<100000)){
	    	    LCD5110_write_Dec1(4,2,LoadmAh/10);
	    	    LCD5110_write_string("Ah");
	            }
	            if ((LoadmAh>=1000)&&(LoadmAh<10000)){
	       	    LCD5110_write_Dec1(4,3,LoadmAh);
	       	    LCD5110_write_string("Ah");
	       	    }
	       	    if ((LoadmAh>=100)&&(LoadmAh<1000)){
	      	    LCD5110_write_Dec1(3,0,LoadmAh);
	       	    LCD5110_write_string(" mAh");
	       	    }
	      	    if ((LoadmAh>=10)&&(LoadmAh<100)){
	       	    LCD5110_write_Dec1(2,0,LoadmAh);
	       	    LCD5110_write_string("  mAh");
	       	    }
	       	    if (LoadmAh<10)
	       	    {
	       	    LCD5110_write_Dec1(1,0,LoadmAh);
	       	    LCD5110_write_string("   mAh");
	       	    }

	      	//Energy Wh

	       	      LCD5110_set_XY(0,5);
	       	      LCD5110_write_string_neg("LoadWh ");
	      	      LCD5110_set_XY(7,5);


	      	                if (LoadWh>=1000000){
	      		    	    LCD5110_write_Dec1(4,0,LoadWh/1000);
	      		    	    LCD5110_write_string("kWh");
	      		            }
	      		            if ((LoadWh>=100000)&&(LoadWh<1000000)){
	      		    	    LCD5110_write_Dec1(3,0,LoadWh/1000);
	      		    	    LCD5110_write_string(" kWh");
	      		            }
	      		            if ((LoadWh>=10000)&&(LoadWh<100000)){
	      		    	    LCD5110_write_Dec1(3,1,LoadWh/100);
	      		    	    LCD5110_write_string("kWh");
	      		            }
	      		            if ((LoadWh>=1000)&&(LoadWh<10000)){
	      		       	    LCD5110_write_Dec1(3,2,LoadWh/10);
	      		       	    LCD5110_write_string("kWh");
	      		       	    }
	      		       	    if ((LoadWh>=100)&&(LoadWh<1000)){
	      		      	    LCD5110_write_Dec1(3,0,LoadWh);
	      		       	    LCD5110_write_string("  Wh");
	      		       	    }
	      		      	    if ((LoadWh>=10)&&(LoadWh<100)){
	      		       	    LCD5110_write_Dec1(2,0,LoadWh);
	      		       	    LCD5110_write_string("   Wh");
	      		       	    }
	      		       	    if (LoadWh<10)
	      		       	    {
	      		       	    LCD5110_write_Dec1(1,0,LoadWh);
	      		       	    LCD5110_write_string("    Wh");
	      		       	    }


}



    if ((BBack==1)&&(release2==1)&&(lockkey!=1))
        {
        c3=1;
        key1=1;
        c2=25;
        BBack=0;
        buttondir=0;
        }

    if ((BOK==1)&&((c3==5)||(c3==6)||(c3==7))&&(release1==1)&&(lockkey!=1))
            {
            c333=c3;
    	    c3=1;
            key1=1;
            c2=15;
            BOK=0;
            buttondir=0;
            }

   if (lockkey!=1) {update1=1;}
   if (zz4==6) {update1=1;}


	}


void counterR(void)
 {


	if (c2!=15){return;}

	unsigned int v2,s,i2,i3,f1;
	    s=4;
	    f1=0;



	    if (c3<1){c3=s;}
		if (c3>=(s+1)){c3=1;}


		LCD5110_set_XY(0,0);
		LCD5110_write_string_neg(" Reset ");
		LCD5110_set_XY(0,5);
		LCD5110_write_string("_______");



		if ( (c3<=(s+1)) && (c3>=1) )
			                    {

			            v2=c3+(s-5);
			            if (v2<s)
			            {

					    for (i3=0;i3<s;i3++)
					    {
					    	LCD5110_set_XY(f1,i3+1);
					    	LCD5110_write_string(counterreset[i3][0]);

					    }


					    LCD5110_set_XY(f1,c3); //selected line
					    LCD5110_write_string_neg(counterreset[c3-1][0]);



					    for(i2=0;i2<15;i2++)  //clear text buffer for the first 10 locations
					    {
					    chr[i2]=0x20;
					    }

					    helptext=counterreset[c3-1][1];
					    scrolllenght=7;
					    nr1=0;
					    c33=c3;
					    z=0;
					    i4=0;   //delay time for scrolling text
					    i5=1;   //activate scrolling text

					    while (*helptext!='\0')
					                  {
					                     chr[z]=*helptext;
					                     z++;
					                     helptext++;
					                  }
					    if (z>=8)
					    {
					        for (i2=0;i2<scrolllenght;i2++)
					        {
					         chr[z+i2]=chr[i2];
					        }

					    }




			            }

			            }

	if (BOK==1)
	{
	 c2=1;
	 //BOK=0;
	 release1=0;
	 key1=1;
     if (c333==5)
     {
      if (c3==1){}
      if (c3==2){BatterymAh=0;BatteryuAh=0;}
      if (c3==3){BatteryWh=0;BatteryuWh=0;}
      if (c3==4){BatterymAh=0;BatteryuAh=0;BatteryWh=0;BatteryuWh=0;}
     }
     if (c333==6)
          {
           if (c3==1){}
           if (c3==2){SolarmAh=0;SolaruAh=0;}
           if (c3==3){SolarWh=0;SolaruWh=0;}
           if (c3==4){SolarmAh=0;SolaruAh=0;SolarWh=0;SolaruWh=0;}
          }
     if (c333==7)
          {
           if (c3==1){}
           if (c3==2){LoadmAh=0;LoaduAh=0;}
           if (c3==3){LoadWh=0;LoaduWh=0;}
           if (c3==4){LoadmAh=0;LoaduAh=0;LoadWh=0;LoaduWh=0;}
          }


	 c3=c333;
	 i5=0;
		}

		if (BMENU==1)
			{
			c2=3;
	  BMENU=0;
	  BOK=0;
	 	c3=1;
	 	key1=1;
			}

		if (BBack==1)
				{
			c3=c333;
			release2=0;
			key1=1;
		    c2=1;
		    //BBack=0;
		    i5=0;
			    }




 }


//----------------------------------------------------------------------------------------
// *************************     Parameter Settings     ************************
//----------------------------------------------------------------------------------------

void parameterset(void)
{
	if (c2!=2){return;}
	unsigned int v2,s,i2,i3,f1,f2,f3;
    s=35;
    f1=0;
    f2=7;
    f3=12;


    if (c3<1){c3=1;}
	if (c3>=(s+1)){c3=s;}


	LCD5110_set_XY(0,0);
	LCD5110_write_Dec1_neg(2,0,c3);
	LCD5110_set_XY(2,0);
	LCD5110_write_string_neg("/");
	LCD5110_set_XY(3,0);
	LCD5110_write_Dec1_neg(2,0,s);
	LCD5110_set_XY(5,0);
	LCD5110_write_string_neg(" ParamSet");

	scrolllenght=8;
	if (c3==35){scrolllenght=13;}

	if (refresh1==1){ROMconvertbd(); refresh1=0;}


	if ( (c3<=(s+1)) && (c3>=1) )
		                    {
		            v2=c3+(s-6);
		            if (v2<s)
		            {

				    for (i3=0;i3<5;i3++)
				    {
				    	LCD5110_set_XY(f2,i3+1);
				    	LCD5110_write_Dec(d[i3][0],d[i3][1],ROMdec[i3]);
				    	LCD5110_set_XY(f1,i3+1); //line4
				    	LCD5110_write_string(bmsmenu[i3][0]);
				    	LCD5110_set_XY(f3,i3+1);
				    	LCD5110_write_string(ROMunit[i3]);
				    }

				    LCD5110_set_XY(f2,c3);
				    LCD5110_write_Dec(d[c3-1][0],d[c3-1][1],ROMdec[c3-1]);
				    LCD5110_set_XY(f1,c3); //selected line
				    LCD5110_write_string_neg(bmsmenu[c3-1][0]);
				    LCD5110_set_XY(f3,c3);
				    LCD5110_write_string(ROMunit[c3-1]);


				    for(i2=0;i2<15;i2++)  //clear text buffer for the first 15 locations
				    {
				    chr[i2]=0x20;
				    }

				    helptext=bmsmenu[c3-1][1];




				    nr1=0;
				    c33=c3;
				    z=0;
				    i4=0;   //delay time for scrolling text
				    i5=1;   //activate scrolling text

				    while (*helptext!='\0')
				                  {
				                     chr[z]=*helptext;
				                     z++;
				                     helptext++;
				                  }
				    if (z>=8)
				    {
				        for (i2=0;i2<scrolllenght;i2++)
				        {
				         chr[z+i2]=chr[i2];
				        }

				    }




		            }
		            if ((v2>=s)&&(v2<(s+(s-5))))
		            {

		            for(i3=0;i3<5;i3++)
		            {
		            	LCD5110_set_XY(f2,i3+1);
		            	LCD5110_write_Dec(d[c3-(5-i3)][0],d[c3-(5-i3)][1],ROMdec[c3-(5-i3)]);
		            	LCD5110_set_XY(f1,i3+1); //
		            	LCD5110_write_string(bmsmenu[c3-(5-i3)][0]);
		            	LCD5110_set_XY(f3,i3+1);
		            	LCD5110_write_string(ROMunit[c3-(5-i3)]);
		            }

		            LCD5110_set_XY(f2,5);
		            LCD5110_write_Dec(d[c3-1][0],d[c3-1][1],ROMdec[c3-1]);
		            LCD5110_set_XY(f1,5); //selected line
		            LCD5110_write_string_neg(bmsmenu[c3-1][0]);
		            LCD5110_set_XY(f3,5);
		            LCD5110_write_string(ROMunit[c3-1]);


		            for(i2=0;i2<15;i2++)  //clear text buffer
		            {
		            chr[i2]=0x20;
		            }
		            helptext=bmsmenu[c3-1][1];
		            nr1=0;
		            c33=c3;
		            z=0;
		            i4=0;   //delay time for scrolling text
		            i5=1;   //activate scrolling text

		            while (*helptext!='\0')
		            				                  {
		            				                     chr[z]=*helptext;
		            				                     z++;
		            				                     helptext++;
		            				                  }
		            if (z>=8)
		            			 	    {
		            			 	    for (i2=0;i2<8;i2++)
		            			 	    {
		            			 	     chr[z+i2]=chr[i2];
		            			 	    }
		            			 	    }



		            }

		            }



	if (BOK==1)
	{
	if (c3==35)
	{
	ROMconvertdb();
	ROMwriteall();
	LCD5110_set_XY(0,5);
	LCD5110_write_string_neg("    ROMwOK    ");
	}
	else
	{
	c22=c2;
	c2=6;
    BOK=0;
    key1=1;
    c33=c3;
    c3=ROMdec[c33-1];

    if ((c33!=14)&&(c33!=16)&&(c33!=18))
     		{
     		c3=ROMdec[c33-1];
     		}
     		if (c33==14)
     		{
     		c3=ROMdec2[0];
     		//ROMdec[c33-1]=realcurrent[c3][2];
     		}
     		if (c33==16)
     		{
     		c3=ROMdec2[1];
     		//ROMdec[c33-1]=realcurrent[c3][1];
     		}
     		if (c33==18)
     		{
     		c3=ROMdec2[2];
     		//ROMdec[c33-1]=realcurrent[c3][3];
     		}





	}
	}

	if (BBack==1)
    {
    c3=2;
    key1=1;
    c2=25;
    BBack=0;
    }


}
//--------------------------
//set value for parameterset.

void valueset(void)
 {


	if (c2!=6){return;}
    unsigned int l1;
	buttondir=1;
	step1=bmsmenuval[c33-1][4];
 	if(c3<bmsmenuval[c33-1][2]){c3=bmsmenuval[c33-1][2];}
 	if(c3>bmsmenuval[c33-1][3]){c3=bmsmenuval[c33-1][3];}




	   if (c33<5)
 	   {
 	   LCD5110_set_XY(7,c33);
 	   LCD5110_write_Dec_neg(d[c33-1][0],d[c33-1][1],c3);
 	   }



	   if ((c33>=5)&&(c33!=14)&&(c33!=16)&&(c33!=18))
 	   {
 	   LCD5110_set_XY(7,5);
 	   LCD5110_write_Dec_neg(d[c33-1][0],d[c33-1][1],c3);
 	   }

	   if (c33==14)
	   {
	   LCD5110_set_XY(7,5);
	   LCD5110_write_Dec_neg(d[c33-1][0],d[c33-1][1],realcurrent[c3][2]);
	   }
	   if (c33==16)
	   {
	   LCD5110_set_XY(7,5);
	   LCD5110_write_Dec_neg(d[c33-1][0],d[c33-1][1],realcurrent[c3][1]);
	   }
	   if (c33==18)
	   {
	   LCD5110_set_XY(7,5);
	   LCD5110_write_Dec_neg(d[c33-1][0],d[c33-1][1],realcurrent[c3][3]);
	   }



 	   if (BOK==1)
 	   {
 		if (c33==1)
 		{
 		 //Load preset parameters from table

 		if ((c3!=6)&&(c3!=7))
 		{
 		ROMunit[2]="Ah";
 		for (l1=0;l1<34;l1++)
 		{
 		ROMdec[l1]=BatteryType[l1][c3-1];
 		}
 		ROMdec2[0]=5;
 		ROMdec2[1]=3;
 		ROMdec2[2]=2;
 		}

 		if ((c3==6)||(c3==7))
 		 		{
 		 		ROMunit[2]="0F";
 		 		for (l1=0;l1<34;l1++)
 		 		{
 		 		ROMdec[l1]=BatteryType[l1][c3-1];
 		 		}
 		 		ROMdec2[0]=5;
 		 		ROMdec2[1]=3;
 		 		ROMdec2[2]=2;
 		 		}
 		}




 		if ((c33!=14)&&(c33!=16)&&(c33!=18))
 		{
 		ROMdec[c33-1]=c3;
 		}
 		if (c33==14)
 		{
 		ROMdec2[0]=c3;
 		ROMdec[c33-1]=realcurrent[c3][2];
 		}
 		if (c33==16)
 		{
 		ROMdec2[1]=c3;
 		ROMdec[c33-1]=realcurrent[c3][1];
 		}
 		if (c33==18)
 		{
 		ROMdec2[2]=c3;
 		ROMdec[c33-1]=realcurrent[c3][3];
 		}
 		c2=c22;
 		BOK=0;
 		c3=c33;
 		buttondir=0;
 		step1=1;
 	   }

 	  if (BBack==1)
 	      {
 		 c2=c22;
 		 BBack=0;
 		 c3=c33;
 		 buttondir=0;
 		 step1=1;
 	      }


 }


//------------------------------------------------------------------------------------------
// ***************************          Device Settings       ************************
//------------------------------------------------------------------------------------------

void deviceset(void)
{
	if (c2!=3){return;}
	LCD5110_set_XY(3,0);
	LCD5110_write_string_neg("DevSet");


		unsigned int v2,s,i2;
	    s=5;

	    if (c3<1){c3=s;}
		if (c3>=(s+1)){c3=1;}


		LCD5110_set_XY(0,0);
		LCD5110_write_Dec1_neg(1,0,c3);
		LCD5110_set_XY(1,0);
		LCD5110_write_string_neg("/");
		LCD5110_set_XY(2,0);
		LCD5110_write_Dec1_neg(1,0,s);
		LCD5110_set_XY(3,0);
		LCD5110_write_string_neg("   DevSet  ");

		if ( (c3<=(s+1)) && (c3>=1) )
			                    {
			            v2=c3+(s-6);
			            if (v2<s)
			            {
			            LCD5110_set_XY(0,1); //dacus test menu
					    LCD5110_write_string(devset[0][0]);
					    LCD5110_set_XY(0,2); //dacus test menu
					    LCD5110_write_string(devset[1][0]);
					    LCD5110_set_XY(0,3); //dacus test menu
					    LCD5110_write_string(devset[2][0]);
					    LCD5110_set_XY(0,4); //dacus test menu
					    LCD5110_write_string(devset[3][0]);
					    LCD5110_set_XY(0,5); //dacus test menu
					    LCD5110_write_string(devset[4][0]);
					    LCD5110_set_XY(0,c3); //dacus test menu
					    LCD5110_write_string_neg(devset[c3-1][0]);

					    for(i2=0;i2<15;i2++)  //clear text buffer for the first 10 locations
					    {
					    chr[i2]=0x20;
					    }

					    helptext=devset[c3-1][1];
					    scrolllenght=14;
					    nr1=0;
					    c33=c3;
					    z=0;
					    i4=0;   //delay time for scrolling text
					    i5=1;   //activate scrolling text

					    while (*helptext!='\0')
					                  {
					                     chr[z]=*helptext;
					                     z++;
					                     helptext++;
					                  }
					    if (z>=8)
					    {
					        for (i2=0;i2<scrolllenght;i2++)
					        {
					         chr[z+i2]=chr[i2];
					        }

					    }




			            }

	      }


		if (BOK==1)
			{
			LCD5110_clear();
			BOK=0;
			c2=c3+7;
			c333=c3;
			c3=1;
			i5=0;
			c8=0;

	  }

		if (BMENU==1)
				{
				c2=25;
				c3=3;
				BMENU=0;
				key1=1;
				}

		if (BBack==1)
		    {
		    c3=3;
		    key1=1;
		    c2=25;
		    BBack=0;
		    }

}

//-----------------
//time and date submenu.

void timeanddate(void)
{
	if (c2!=8){return;}

    RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
    RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);

		unsigned int v2,s,i2,i3,f1,f2,f3;
	    s=5;
	    f1=0;
	    f2=7;
	    f3=12;



        td[0]=RTC_TimeStructure.RTC_Hours;
        td[1]=RTC_TimeStructure.RTC_Minutes;
        td[2]=RTC_DateStructure.RTC_Date;
        td[3]=RTC_DateStructure.RTC_Month;
        td[4]=RTC_DateStructure.RTC_Year+2000;


	 if (c3<1){c3=s;}
		if (c3>=(s+1)){c3=1;}


		LCD5110_set_XY(0,0);
		LCD5110_write_Dec1_neg(1,0,c3);
		LCD5110_set_XY(1,0);
		LCD5110_write_string_neg("/");
		LCD5110_set_XY(2,0);
		LCD5110_write_Dec1_neg(1,0,s);
		LCD5110_set_XY(3,0);
		LCD5110_write_string_neg("   ");



		if ( (c3<=(s+1)) && (c3>=1) )
			                    {

			            v2=c3+(s-6);
			            if (v2<s)
			            {

					    for (i3=0;i3<5;i3++)
					    {
					    	LCD5110_set_XY(f2,i3+1);
					    	LCD5110_write_Dec(timedateval[i3][0],timedateval[i3][1],td[i3]);
					    	LCD5110_set_XY(f1,i3+1); //line4
					    	LCD5110_write_string(timedate[i3][0]);
					    	LCD5110_set_XY(f3,i3+1);
					    	LCD5110_write_string(timedate[i3][2]);
					    }

					    LCD5110_set_XY(f2,c3);
					    LCD5110_write_Dec(timedateval[c3-1][0],timedateval[c3-1][1],td[c3-1]);
					    LCD5110_set_XY(f1,c3); //selected line
					    LCD5110_write_string_neg(timedate[c3-1][0]);
					    LCD5110_set_XY(f3,c3);
					    LCD5110_write_string(timedate[c3-1][2]);


					    for(i2=0;i2<15;i2++)  //clear text buffer for the first 10 locations
					    {
					    chr[i2]=0x20;
					    }

					    helptext=timedate[c3-1][1];
					    scrolllenght=8;
					    nr1=0;
					    c33=c3;
					    z=0;
					    i4=0;   //delay time for scrolling text
					    i5=1;   //activate scrolling text

					    while (*helptext!='\0')
					                  {
					                     chr[z]=*helptext;
					                     z++;
					                     helptext++;
					                  }
					    if (z>=8)
					    {
					        for (i2=0;i2<scrolllenght;i2++)
					        {
					         chr[z+i2]=chr[i2];
					        }

					    }




			            }

			            }



		if (BOK==1)
		{
	 c22=c2;
	 c2=7;
	 BOK=0;
	 key1=1;
	 c33=c3;
	 c3=td[c33-1];
	 i5=0;
		}




		if (BMENU==1)
			{
			c2=3;
	  BMENU=0;
	  BOK=0;
	 	c3=1;
	 	key1=1;
			}

		if (BBack==1)
				    {
				    c3=1;
				    key1=1;
				    c2=3;
				    BBack=0;
				    }


   //update1=1;


}

//------------------------
//time and date value set.
void valueset2(void)
 {


	if (c2!=7){return;}

    buttondir=1;
	step1=timedateval[c33-1][4];
 	if(c3<timedateval[c33-1][2]){c3=timedateval[c33-1][2];}
 	if(c3>timedateval[c33-1][3]){c3=timedateval[c33-1][3];}


	 if (c33<5)
 	   {
 	   LCD5110_set_XY(7,c33);
 	   LCD5110_write_Dec_neg(timedateval[c33-1][0],timedateval[c33-1][1],c3);
 	   }
 	   if (c33>=5)
 	   {
 	   LCD5110_set_XY(7,5);
 	   LCD5110_write_Dec_neg(timedateval[c33-1][0],timedateval[c33-1][1],c3);
 	   }

 	   if (BOK==1)
 	   {
 		td[c33-1]=c3;

 		RTC_TimeStructure.RTC_Hours=td[0];
  		RTC_TimeStructure.RTC_Minutes=td[1];
  		RTC_DateStructure.RTC_Date=td[2];
  		RTC_DateStructure.RTC_Month=td[3];
  		RTC_DateStructure.RTC_Year=td[4]-2000;

  		if(RTC_SetTime(RTC_Format_BIN, &RTC_TimeStructure) == ERROR)
  		  {
  			LCD5110_set_XY(0,0);
  			LCD5110_write_string("Time set Error");
  		    delayms1(1500);
  		  }
  		else
  		{
  		}

  		if(RTC_SetDate(RTC_Format_BIN, &RTC_DateStructure) == ERROR)
  		  {
  			LCD5110_set_XY(0,0);
  			LCD5110_write_string("Date set Error");
  			delayms1(1500);
  		  }
  		  else
  		  {
  		RTC_WriteBackupRegister(RTC_BKP_DR0, 0x32F2);
  		  }


  		 		c2=c22;
  		 		BOK=0;
  		 		c3=c33;
  		 		buttondir=0;
  		 		step1=1;
  		 		key1=1;
  		 		lk1();
  		 		bk1();

 	   }

 	  if (BBack==1)
 	  		    {
 		        c2=c22;
 		   	    c3=c33;
 		   	    buttondir=0;
 		   	    step1=1;
 		   		key1=1;
 	  		    BBack=0;
 	  		    }

 	    if (BMENU==1)
 	    {
 	    c2=8;
 	    BOK=0;
 	    BMENU=0;
 	    step1=1;
 	    key1=1;
 	    }


 }

void lk1 (void)

{
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
	swtimer4=((RTC_TimeStructure.RTC_Minutes*60)+RTC_TimeStructure.RTC_Seconds);

}



//------------------------
//backlight value set.
void bk1 (void)

{
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
	swtimer1=((RTC_TimeStructure.RTC_Minutes*60)+RTC_TimeStructure.RTC_Seconds);

	if (bk[0]==0)
	{
	LED1_OFF
	LED2_OFF
	}
	if (bk[0]==1)
	{
	LED1_ON
	LED2_OFF
	}
	if(bk[0]==2)
	{
	LED1_ON
	LED2_ON
	}
}

//-------
//Backlight submenu
void backlight(void)
{
	if (c2!=9){return;}



		unsigned int v2,s,i2,i3,f1,f2,f3;
	    s=3;
	    f1=0;
	    f2=8;
	    f3=13;



	    if (c3<1){c3=s;}
		if (c3>=(s+1)){c3=1;}


		LCD5110_set_XY(0,0);
		LCD5110_write_Dec1_neg(1,0,c3);
		LCD5110_set_XY(1,0);
		LCD5110_write_string_neg("/");
		LCD5110_set_XY(2,0);
		LCD5110_write_Dec1_neg(1,0,s);
		LCD5110_set_XY(3,0);
		LCD5110_write_string_neg("LCD BKlight");



		if ( (c3<=(s+1)) && (c3>=1) )
			                    {

			            v2=c3+(s-4);
			            if (v2<s)
			            {

					    for (i3=0;i3<s;i3++)
					    {
					    	LCD5110_set_XY(f2,i3+1);
					    	LCD5110_write_Dec(backlightval[i3][0],backlightval[i3][1],bk[i3]);
					    	LCD5110_set_XY(f1,i3+1); //line4
					    	LCD5110_write_string(LCDbacklight[i3][0]);
					    	LCD5110_set_XY(f3,i3+1);
					    	LCD5110_write_string(LCDbacklight[i3][2]);
					    }

					    LCD5110_set_XY(f2,c3);
					    LCD5110_write_Dec(backlightval[c3-1][0],backlightval[c3-1][1],bk[c3-1]);
					    LCD5110_set_XY(f1,c3); //selected line
					    LCD5110_write_string_neg(LCDbacklight[c3-1][0]);
					    LCD5110_set_XY(f3,c3);
					    LCD5110_write_string(LCDbacklight[c3-1][2]);


					    for(i2=0;i2<15;i2++)  //clear text buffer for the first 10 locations
					    {
					    chr[i2]=0x20;
					    }

					    helptext=LCDbacklight[c3-1][1];
					    scrolllenght=8;
					    nr1=0;
					    c33=c3;
					    z=0;
					    i4=0;   //delay time for scrolling text
					    i5=1;   //activate scrolling text

					    while (*helptext!='\0')
					                  {
					                     chr[z]=*helptext;
					                     z++;
					                     helptext++;
					                  }
					    if (z>=8)
					    {
					        for (i2=0;i2<scrolllenght;i2++)
					        {
					         chr[z+i2]=chr[i2];
					        }

					    }




			            }

			            }

		if (BOK==1)
		{
		c22=c2;
		c2=13;
	 BOK=0;
	 key1=1;
	 c33=c3;
	 c3=bk[c33-1];
	 i5=0;
		}

		if (BMENU==1)
			{
			c2=3;
	  BMENU=0;
	  BOK=0;
	 	c3=1;
	 	key1=1;
			}

		if (BBack==1)
				{
			c3=2;
			key1=1;
		    c2=3;
		    BBack=0;
			    }


}

//------------------------------------------------------------------------------------------

void valueset3(void)
 {


	if (c2!=13){return;}

    buttondir=1;
	step1=backlightval[c33-1][4];
 	if(c3<backlightval[c33-1][2]){c3=backlightval[c33-1][2];}
 	if(c3>backlightval[c33-1][3]){c3=backlightval[c33-1][3];}


	 if (c33<5)
 	   {
 	   LCD5110_set_XY(8,c33);
 	   LCD5110_write_Dec_neg(backlightval[c33-1][0],backlightval[c33-1][1],c3);
 	   }
 	   if (c33>=5)
 	   {
 	   LCD5110_set_XY(8,5);
 	   LCD5110_write_Dec_neg(backlightval[c33-1][0],backlightval[c33-1][1],c3);
 	   }



 	   if (BOK==1)
 	   {
 		bk[c33-1]=c3;
 		c2=c22;
 		BOK=0;
 		c3=c33;
 		buttondir=0;
 		step1=1;
 		bk1();
 	   }

		if (BBack==1)
				{
			c2=c22;
			c3=c33;
			buttondir=0;
			step1=1;
		    BBack=0;
			    }

 	    if (BMENU==1)
 	    {
 	    c2=8;
 	    BOK=0;
 	    BMENU=0;
 	    step1=1;
 	    key1=1;
 	    }


 }

//USART-submenu
void USARTMenu(void)
{
	if (c2!=10){return;}



		unsigned int v2,s,i2,i3,f1,f2,f3;
	    s=3;
	    f1=0;
	    f2=8;
	    f3=13;

        //bk1();
	    if (c3<1){c3=s;}
		if (c3>=(s+1)){c3=1;}


		LCD5110_set_XY(0,0);
		LCD5110_write_Dec1_neg(1,0,c3);
		LCD5110_set_XY(1,0);
		LCD5110_write_string_neg("/");
		LCD5110_set_XY(2,0);
		LCD5110_write_Dec1_neg(1,0,s);
		LCD5110_set_XY(3,0);
		LCD5110_write_string_neg("   USART   ");



		if ( (c3<=(s+1)) && (c3>=1) )
			                    {
			            c9=1;
			            v2=c3+(s-4);
			            if (v2<s)
			            {

					    for (i3=0;i3<s;i3++)
					    {
					    	LCD5110_set_XY(f2,i3+1);
					    	LCD5110_write_Dec(USARTval[i3][0],USARTval[i3][1],Serial[i3]);
					    	LCD5110_set_XY(f1,i3+1); //line4
					    	LCD5110_write_string(USARTmenu[i3][0]);
					    	LCD5110_set_XY(f3,i3+1);
					    	LCD5110_write_string(USARTmenu[i3][2]);
					    }

					    LCD5110_set_XY(f2,c3);
					    LCD5110_write_Dec(USARTval[c3-1][0],USARTval[c3-1][1],Serial[c3-1]);
					    LCD5110_set_XY(f1,c3); //selected line
					    LCD5110_write_string_neg(USARTmenu[c3-1][0]);
					    LCD5110_set_XY(f3,c3);
					    LCD5110_write_string(USARTmenu[c3-1][2]);


					    for(i2=0;i2<15;i2++)  //clear text buffer for the first 10 locations
					    {
					    chr[i2]=0x20;
					    }

					    helptext=USARTmenu[c3-1][1];
					    scrolllenght=8;
					    nr1=0;
					    c33=c3;
					    z=0;
					    i4=0;   //delay time for scrolling text
					    i5=1;   //activate scrolling text

					    while (*helptext!='\0')
					                  {
					                     chr[z]=*helptext;
					                     z++;
					                     helptext++;
					                  }
					    if (z>=8)
					    {
					        for (i2=0;i2<scrolllenght;i2++)
					        {
					         chr[z+i2]=chr[i2];
					        }

					    }




			            }

			            }

		if (BOK==1)
		{
		c22=c2;
		c2=14;
	 BOK=0;
	 key1=1;
	 c33=c3;
	 c3=Serial[c33-1];
	 i5=0;
		}

		if (BMENU==1)
			{
			c2=3;
	  BMENU=0;
	  BOK=0;
	 	c3=1;
	 	key1=1;
			}

		if (BBack==1)
			{
			c3=3;
			key1=1;
		    c2=3;
		    BBack=0;
		    }



}

//------------------------------------------------------------------------------------------

void valueset4(void)
 {


	if (c2!=14){return;}

    buttondir=1;
	step1=USARTval[c33-1][4];
 	if(c3<USARTval[c33-1][2]){c3=USARTval[c33-1][2];}
 	if(c3>USARTval[c33-1][3]){c3=USARTval[c33-1][3];}


	 if (c33<5)
 	   {
 	   LCD5110_set_XY(8,c33);
 	   LCD5110_write_Dec_neg(USARTval[c33-1][0],USARTval[c33-1][1],c3);
 	   }
 	   if (c33>=5)
 	   {
 	   LCD5110_set_XY(8,5);
 	   LCD5110_write_Dec_neg(USARTval[c33-1][0],USARTval[c33-1][1],c3);
 	   }



 	   if (BOK==1)
 	   {
 		Serial[c33-1]=c3;
 		if (c33==2){USART_Config();}
 		if (c33==1 && c3==1){UsartDataHeader();}
 		c2=c22;
 		BOK=0;
 		c3=c33;
 		buttondir=0;
 		step1=1;
 	   }

 	  if (BBack==1)
 	  	{
 	    c2=c22;
 	    c3=c33;
 		buttondir=0;
 		step1=1;
 	    BBack=0;
 	    }


 	    if (BMENU==1)
 	    {
 	    c2=8;
 	    BOK=0;
 	    BMENU=0;
 	    step1=1;
 	    key1=1;
 	    }


 }

//---------------------------------------
void uSDMenu(void)
{
	if (c2!=11){return;}
	//unsigned int v2,s,i2,i3,f1,f2,f3;
		//    s=1;
		  //  f1=0;
		   // f2=8;
		    //f3=13;

		  //  if (c3<1){c3=s;}
		  //  if (c3>=(s+1)){c3=1;}

    //LCD5110_set_XY(0,0);
	//LCD5110_write_Dec1_neg(1,0,c3);
	//LCD5110_set_XY(1,0);
	//LCD5110_write_string_neg("/");
	//LCD5110_set_XY(2,0);
	//LCD5110_write_Dec1_neg(1,0,s);
	LCD5110_set_XY(0,0);
	LCD5110_write_string_neg("    uSDMenu   ");

	if (BBack==1)
				{
				c3=4;
				key1=1;
			    c2=3;
			    BBack=0;
			    }


}

void EXT1ANDEXT2(void)
{
	if (c2!=12){return;}
	unsigned int v2,s,i2,i3,f1,f2,f3;
		    s=4;
		    f1=0;
		    f2=8;
		    f3=13;

		    if (c3<1){c3=s;}
		    if (c3>=(s+1)){c3=1;}

	LCD5110_set_XY(0,0);
	LCD5110_write_Dec1_neg(1,0,c3);
	LCD5110_set_XY(1,0);
	LCD5110_write_string_neg("/");
	LCD5110_set_XY(2,0);
	LCD5110_write_Dec1_neg(1,0,s);
	LCD5110_set_XY(3,0);
	LCD5110_write_string_neg("  EXT 1 & 2 ");
	LCD5110_set_XY(0,5);
	LCD5110_write_string_neg("K/D/GND/E1/E2 ");


	if ( (c3<=(s+1)) && (c3>=1) )
			                    {

			            v2=c3+(s-(s+1));
			            if (v2<s)
			            {

					    for (i3=0;i3<s;i3++)
					    {
					    	LCD5110_set_XY(f2,i3+1);
					    	LCD5110_write_Dec(EXT12val[i3][0],EXT12val[i3][1],EXT[i3]);
					    	LCD5110_set_XY(f1,i3+1); //line4
					    	LCD5110_write_string(EXT12[i3][0]);
					    	LCD5110_set_XY(f3,i3+1);
					    	LCD5110_write_string(EXT12[i3][2]);
					    }

					    LCD5110_set_XY(f2,c3);
					    LCD5110_write_Dec(EXT12val[c3-1][0],EXT12val[c3-1][1],EXT[c3-1]);
					    LCD5110_set_XY(f1,c3); //selected line
					    LCD5110_write_string_neg(EXT12[c3-1][0]);
					    LCD5110_set_XY(f3,c3);
					    LCD5110_write_string(EXT12[c3-1][2]);


					    for(i2=0;i2<15;i2++)  //clear text buffer for the first 15 locations
					    {
					    chr[i2]=0x20;
					    }

					    helptext=EXT12[c3-1][1];
					    scrolllenght=8;
					    nr1=0;
					    c33=c3;
					    z=0;
					    i4=0;   //delay time for scrolling text
					    i5=1;   //activate scrolling text

					    while (*helptext!='\0')
					                  {
					                     chr[z]=*helptext;
					                     z++;
					                     helptext++;
					                  }
					    if (z>=8)
					    {
					        for (i2=0;i2<scrolllenght;i2++)
					        {
					         chr[z+i2]=chr[i2];
					        }

					    }




			            }

			            }

	if (BOK==1)
	{
	c22=c2;
	c2=16;
	BOK=0;
	key1=1;
	c33=c3;
	c3=EXT[c33-1];
	i5=0;
	}

	if (BBack==1)
				{
				c3=5;
				key1=1;
			    c2=3;
			    BBack=0;
			    }

}


void valueset5(void)
 {

	if (c2!=16){return;}

    buttondir=1;
	step1=EXT12val[c33-1][4];
 	if(c3<EXT12val[c33-1][2]){c3=EXT12val[c33-1][2];}
 	if(c3>EXT12val[c33-1][3]){c3=EXT12val[c33-1][3];}


	 if (c33<5)
 	   {
 	   LCD5110_set_XY(8,c33);
 	   LCD5110_write_Dec_neg(EXT12val[c33-1][0],EXT12val[c33-1][1],c3);
 	   }
 	   if (c33>=5)
 	   {
 	   LCD5110_set_XY(8,5);
 	   LCD5110_write_Dec_neg(EXT12val[c33-1][0],EXT12val[c33-1][1],c3);
 	   }



 	   if (BOK==1)
 	   {
 		EXT[c33-1]=c3;
 		c2=c22;
 		BOK=0;
 		c3=c33;
 		buttondir=0;
 		step1=1;
 		key1=1;
 	   }

		if (BBack==1)
				{
			c2=c22;
			c3=c33;
			buttondir=0;
			step1=1;
		    BBack=0;
			    }

 	    if (BMENU==1)
 	    {
 	    c2=8;
 	    BOK=0;
 	    BMENU=0;
 	    step1=1;
 	    key1=1;
 	    }

 }

//------------------------------------------------------------------------------------------
// ***************************          Diagnostics           ************************
//------------------------------------------------------------------------------------------

void diagnostics(void)
{
	if (c2!=4){return;}
	LCD5110_set_XY(3,0);
	LCD5110_write_string_neg("    PAR IC ");
	LCD5110_set_XY(0,0);
	LCD5110_write_Dec1_neg(3,0,i2c);

	c8=((RAMrx[37]*256)+RAMrx[36]);  //ext temp T2 that is solar current from MAX8040
    c9=(RAMrx[15]*256)+RAMrx[14];  //battery current

	LCD5110_set_XY(1,1);
	LCD5110_write_Dec1(4,0,c8);
	LCD5110_set_XY(1,2);
	LCD5110_write_Dec1(4,0,c8cal);
	LCD5110_set_XY(1,3);
	LCD5110_write_Dec1(4,0,c9);
	LCD5110_set_XY(1,4);
	LCD5110_write_Dec1(4,0,c9cal);


	buttondir=1;
	ROMconvertdb();
	//if (refresh1==1){c3=0; refresh1=0;}

	if (c3<=1){c3=1;}
	if (c3>=35){c3=34;}
	//if (c3==35){c3=34;}

    if (c3<=22)
{
c6=(c3-1)*4;
ROMread(c6);
c10=0;
LCD5110_set_XY(7,5);
LCD5110_write_string_neg("ROM ");

LCD5110_set_XY(11,1);
LCD5110_write_Dec1(3,0,ROMt[0]);
LCD5110_set_XY(11,2);
LCD5110_write_Dec1(3,0,ROMt[1]);
LCD5110_set_XY(11,3);
LCD5110_write_Dec1(3,0,ROMt[2]);
LCD5110_set_XY(11,4);
LCD5110_write_Dec1(3,0,ROMt[3]);


LCD5110_set_XY(7,1);
LCD5110_write_Dec1(3,0,ROMtemp[c6+0]);
LCD5110_set_XY(7,2);
LCD5110_write_Dec1(3,0,ROMtemp[c6+1]);
LCD5110_set_XY(7,3);
LCD5110_write_Dec1(3,0,ROMtemp[c6+2]);
LCD5110_set_XY(7,4);
LCD5110_write_Dec1(3,0,ROMtemp[c6+3]);

LCD5110_set_XY(11,5);
LCD5110_write_Dec1_neg(3,0,(c3-1)*4);
}
    if ((c3>=23)&&(c3<=35))
{
c6=(c3-23)*4;
LCD5110_set_XY(7,5);
c10=1;
LCD5110_write_string_neg("RAM ");
LCD5110_set_XY(11,1);
LCD5110_write_Dec1(3,0,RAMrx[0+c6]);
LCD5110_set_XY(11,2);
LCD5110_write_Dec1(3,0,RAMrx[1+c6]);
LCD5110_set_XY(11,3);
LCD5110_write_Dec1(3,0,RAMrx[2+c6]);
LCD5110_set_XY(11,4);
LCD5110_write_Dec1(3,0,RAMrx[3+c6]);

LCD5110_set_XY(7,1);
LCD5110_write_string("   ");
LCD5110_set_XY(7,2);
LCD5110_write_string("   ");
LCD5110_set_XY(7,3);
LCD5110_write_string("   ");
LCD5110_set_XY(7,4);
LCD5110_write_string("   ");

LCD5110_set_XY(11,5);
LCD5110_write_Dec1_neg(3,0,((c3-1)*4)-8);

}

LCD5110_set_XY(0,5);
LCD5110_write_string_neg("Address");





//if (BOK==1)
//	{
//	LCD5110_set_XY(0,3);
//	LCD5110_write_string_neg("ROMwrite");
//	ROMwriteall();
//	LCD5110_set_XY(0,2);
//	LCD5110_write_string_neg("ROMwOK");

//	}

if (BBack==1)
	{
	c2=25;
	c3=4;
    BBack=0;
    key1=1;
    buttondir=0;
	}


}

//------------------------------------------------------------------------------------------
// ***************************          About            ************************
//------------------------------------------------------------------------------------------

void about(void)
{

	if (c2!=5){return;}
	LCD5110_set_XY(0,0);
	LCD5110_write_string("@ ");
	LCD5110_set_XY(2,0);
	LCD5110_write_string_neg("ElectroDacus");

	LCD5110_set_XY(0,2);
	LCD5110_write_string("Model SBMS4080");
	LCD5110_set_XY(0,3);
	LCD5110_write_string("Software: 0.9c");
	LCD5110_set_XY(0,4);
	LCD5110_write_string("Hardware: 0.3c");

	//update1=1;

	if (BBack==1)
		{
		c2=25;
		c3=5;
 	    BBack=0;
 	    key1=1;
		}


}

//------------------------------------------------------------------------------------------
// ***************************          Load ON/OFF      ************************
//------------------------------------------------------------------------------------------

void LoadONOFF(void)
{
//	if (BLoad!=1){return;}
	bk1();
	if (lockkey!=0){key1=1;return;}
	RAMreadall();

	tempH=((RAMrx[6]) & 0x01);
    if(tempH!=0){
        tempL=(((RAMrx[7])&0xBF)+64);
    	RAMwrite(0x87,tempL); //uC manual control
    	LCD5110_set_XY(3,0);
    	LCD5110_write_string("Load&PV OFF");
    	tempL=(RAMrx[6] & 0xFC);
    	RAMwrite(0x86,tempL);//load and PV is OFF
    	// need to do also charge OFF
        delayms1(300);
    }
 	else
 	{
 		tempL=(((RAMrx[7])&0xEF)+16);
 		RAMwrite(0x87,tempL); //uC manual load monitor control
 		tempL=(((RAMrx[6])&0x3F)+128);
 		RAMwrite(0x86,tempL); //uC manual load monitor control

 		tempL=(((RAMrx[7])&0xBF));
 		RAMwrite(0x87,tempL);        //FET auto enable
 		LCD5110_set_XY(3,0);
 		LCD5110_write_string("Load&PV ON ");
 		delayms1(300);
 	}
RAMreadall();
}

//------------------------------------------------------------------------------------------
// ***************************        Scrolling Help.    ************************
//------------------------------------------------------------------------------------------

void scrollhelp(void)
{

  if ((i5!=1)||(lockkey==1)){return;}

	unsigned int pos,i1;
    pos=c33;

                if (pos>=5)
                 {
                	pos=5;
                 }

                 if (i4<=200)
                 {
                 i4++;
                 return;
                 }
                 if ((i3<5)||(c33==0)) {i3++;return;}

                 for (i1=0;i1<scrolllenght;i1++)
                 {
                    if (nr1>=z+1)
                       {
                        nr1=0;
                       }
                 LCD5110_set_XY(i1,pos);
                 LCD5110_write_char_neg(chr[nr1+i1]);
                 }
                nr1++;
                i3=0;


}


//==========================================================================================
// Touch Key processing.
//==========================================================================================

void ProcessSensors(void)
{

	if (TEST_TKEY(4-buttondir))  // UP button
	  {
		if(b3==0){c3=c3+step1;key1=1;}
		if(b3>30){c3=c3+step1;key1=1;}
		if(b3>100){c3=c3+(step1*3);key1=1;}
		b3++;
	  }
	  else
	  {
		  b3=0;
	  }
    if (TEST_TKEY(3+buttondir))  //Down Button
    {
    	if(b4==0){c3=c3-step1;key1=1;}
    	if(b4>30){c3=c3-step1;key1=1;}
    	if(b4>100){c3=c3-(step1*3);key1=1;}
    	b4++;
     }
    else
    {
    b4=0;
     }

  if (TEST_TKEY(0))   //OK Button
        {
    BOK=1;
    if (b1==0){key1=1;}
    b1=1;
        }
        else
        {
        BOK=0;
        b1=0;
        }

  if (TEST_TKEY(2))   // Menu Button
            {
      BMENU=1;
      if (lockkey!=1){c2=25;}
      if (b2==0){key1=1;}
      b2=1;
      if (lockkey!=1){c3=1;buttondir=0;}
            }
            else
            {
      BMENU=0;
      b2=0;
            }

  if (TEST_TKEY(1))   // Load ON/OFF Button
              {
        BLoad=1;
        if (b5==0){LoadONOFF();}
        b5=1;
              }
              else
              {
        BLoad=0;
        b5=0;
              }

   if (TEST_TKEY(5))   // Back Button
              {
          BBack=1;
          if (b6==0){key1=1;}
          b6=1;
              }
              else
             {
         BBack=0;
         b6=0;
             }


return;
}


//==========================================================================================
// Time Display.
//==========================================================================================

void Time1 (unsigned int r)

{

	    RTC_TimeTypeDef RTC_TimeStructure;
	    RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
	//    subsec = 100 - ((uint32_t)((uint32_t)RTC_GetSubSecond() * 100) / (uint32_t)8275);

	    LCD5110_set_XY(6,r);
	    LCD5110_write_char_neg((RTC_TimeStructure.RTC_Hours / 10) + 0x30);
	    LCD5110_set_XY(7,r);
	    LCD5110_write_char_neg((RTC_TimeStructure.RTC_Hours % 10) + 0x30);
	    LCD5110_set_XY(8,r);
	    LCD5110_write_string_neg(":");
	    LCD5110_set_XY(9,r);
	    LCD5110_write_char_neg((RTC_TimeStructure.RTC_Minutes / 10) + 0x30);
	    LCD5110_set_XY(10,r);
	    LCD5110_write_char_neg((RTC_TimeStructure.RTC_Minutes % 10) + 0x30);
	    LCD5110_set_XY(11,r);
	    LCD5110_write_string_neg(":");
	    LCD5110_set_XY(12,r);
	    LCD5110_write_char_neg((RTC_TimeStructure.RTC_Seconds / 10) + 0x30);
	    LCD5110_set_XY(13,r);
	    LCD5110_write_char_neg((RTC_TimeStructure.RTC_Seconds % 10) + 0x30);
	    //LCD5110_set_XY(11,r);
	    //LCD5110_write_string(":");
	    //LCD5110_set_XY(12,r);
	    //LCD5110_write_char(((subsec % 100)/10) + 0x30);
	    //LCD5110_set_XY(13,r);
	    //LCD5110_write_char((subsec % 10) + 0x30);

	//LCD5110_set_XY(1,1);
	//LCD5110_write_Dec(4,0,c3);
	    zz1=0;
	    //update1=1;


}

//==========================================================================================
// Time Stamp.
//==========================================================================================

void timestamp1 (void)

{
	 RTC_TimeTypeDef RTC_TimeStructure;
	 RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);

	    timestamp[0]=(RTC_TimeStructure.RTC_Hours / 10) + 0x30;
	    timestamp[1]=(RTC_TimeStructure.RTC_Hours % 10) + 0x30;
	    timestamp[2]=44;
	    timestamp[3]=(RTC_TimeStructure.RTC_Minutes / 10) + 0x30;
	    timestamp[4]=(RTC_TimeStructure.RTC_Minutes % 10) + 0x30;
	    timestamp[5]=44;
	    timestamp[6]=(RTC_TimeStructure.RTC_Seconds / 10) + 0x30;
	    timestamp[7]=(RTC_TimeStructure.RTC_Seconds % 10) + 0x30;
}


//==========================================================================================
// Date Stamp.
//==========================================================================================

void datestamp1 (void)

{
	 RTC_DateTypeDef RTC_DateStructure;
	 RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);

	    datestamp[0]=(RTC_DateStructure.RTC_Year / 10) + 0x30;
	    datestamp[1]=(RTC_DateStructure.RTC_Year % 10) + 0x30;
	    datestamp[2]=44;
	    datestamp[3]=(RTC_DateStructure.RTC_Month / 10) + 0x30;
	    datestamp[4]=(RTC_DateStructure.RTC_Month % 10) + 0x30;
	    datestamp[5]=44;
	    datestamp[6]=(RTC_DateStructure.RTC_Date / 10) + 0x30;
	    datestamp[7]=(RTC_DateStructure.RTC_Date % 10) + 0x30;
}



//==========================================================================================
// Configures the USART.
//==========================================================================================

void USART_Config(void)
{
	  USART_InitTypeDef USART_InitStructure;
	  GPIO_InitTypeDef GPIO_InitStructure;
	  //NVIC_InitTypeDef NVIC_InitStructure;

	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource14, GPIO_AF_1);
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_1);

	  /* Configure USART2 pins:  Rx and Tx ----------------------------*/
	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14 | GPIO_Pin_15;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  /* Enable USART2 IRQ */
	   // NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	   // NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	   // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	   // NVIC_Init(&NVIC_InitStructure);


	  USART_InitStructure.USART_BaudRate = (Serial[1]*100);
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	  USART_InitStructure.USART_Parity = USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	  USART_Init(USART2, &USART_InitStructure);

	  USART_Cmd(USART2,ENABLE);

}



#if USE_LPM > 0
// Enter in Low-Power mode when all sensors are in Release state

void LowPower(void)
{
  // Check that all sensors are in Release state
  if (MyObjGroup.StateMask == TSL_STATE_RELEASE_BIT_MASK)
  {

    // Set alarm to wakeup the MCU from STOP mode
    RTC_AlarmConfig();



    // Set GPIOs in "Low-Power mode"
    Init_LPM_GPIOs();

    //+++++++++++++
    // +++ STOP +++ --> Exit by RTC alarm
    //+++++++++++++
    // Request to enter STOP mode with regulator in low power mode
    PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);



    // Disable the RTC Alarm interrupt
    RTC_ITConfig(RTC_IT_ALRA, DISABLE);
    RTC_AlarmCmd(RTC_Alarm_A, DISABLE);

    // Configures system clock after wake-up from STOP
    SYSCLKConfig_STOP();

    // Re-initializes the GPIOs
    Init_Reset_GPIOs();
    Init_Std_GPIOs();
    //Init_TS_GPIOs();
    TSL_user_Init();
    LCD5110_GPIO_Config();


  }
}







//==========================================================================================
// Configures the RTC.
//==========================================================================================

void RTC_Config(void)
{
  RTC_TimeTypeDef   RTC_TimeStructure;
  RTC_InitTypeDef   RTC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configures the KEY button */
  // Uncomment to enable KEY button on the board
  // Used to exit from STOP mode manually without waiting the RTC alarm.
  //STM_EVAL_PBInit(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Enable the PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* Allow access to RTC */
  PWR_BackupAccessCmd(ENABLE);

  /* Enable the LSE or LSI */
  RCC_LSEConfig(RCC_LSE_ON);
  // RCC_LSICmd(ENABLE);

  /* Wait till LSE or LSI is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
 // while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
  {}

  /* Select the RTC Clock Source = LSE or LSI */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
  //RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

  /* Enable the RTC Clock */
  RCC_RTCCLKCmd(ENABLE);

  /* Wait for RTC APB registers synchronisation */
  RTC_WaitForSynchro();

  /* Set RTC calendar clock to 1 Hz */
  RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
  // RTC_AsynchPrediv max =   127
  // RTC_SynchPrediv  max = 32767
  // For LSI (40000 Hz)
  RTC_InitStructure.RTC_AsynchPrediv = 127; // 40000 Hz / 2 = 20000 Hz
  RTC_InitStructure.RTC_SynchPrediv  = 255; //   20000 Hz / 20000 =   1 Hz

  if (RTC_Init(&RTC_InitStructure) == ERROR)
  {
    while(1);
  }

  /* Set the time to 00h 00mn 00s AM */
  RTC_TimeStructure.RTC_H12     = RTC_H12_AM;
  RTC_TimeStructure.RTC_Hours   = 0x00;
  RTC_TimeStructure.RTC_Minutes = 0x00;
  RTC_TimeStructure.RTC_Seconds = 0x00;

  RTC_SetTime(RTC_Format_BCD, &RTC_TimeStructure);

  /* Configure EXTI line 17 (connected to the RTC Alarm event) */
  EXTI_ClearITPendingBit(EXTI_Line17);
  EXTI_InitStructure.EXTI_Line = EXTI_Line17;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable the RTC Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}


/**
  * @brief  Configures the RTC alarm.
  * @note   You can change the alarm configuration: 1s or less than 1s by commenting/uncommenting
  *         the code below.
  * @param  None
  * @retval None
  */
//==========================================================================================
// Configures the RTC alarm.
//==========================================================================================

void RTC_AlarmConfig(void)
{
  RTC_TimeTypeDef   RTC_TimeStructure;
  RTC_AlarmTypeDef  RTC_AlarmStructure;

  /* Get current time */
  RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Set the alarm to current time + 1s
/*
  RTC_AlarmStructure.RTC_AlarmTime.RTC_H12     = RTC_H12_AM;
  RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours   = RTC_TimeStructure.RTC_Hours;
  RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = RTC_TimeStructure.RTC_Minutes;
  RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = RTC_TimeStructure.RTC_Seconds + 2;
  RTC_AlarmStructure.RTC_AlarmDateWeekDay = 0x31;
  RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;
  RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay | RTC_AlarmMask_Minutes |
                                     RTC_AlarmMask_Hours;


  RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);
*/
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Set the alarm to a value below 1 second

  RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_All;
  RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);

  // Set AlarmA subseconds and enable SubSec Alarm:
  //RTC_AlarmSubSecondConfig(RTC_Alarm_A, 0xFF, RTC_AlarmSubSecondMask_SS14_5); // One interrupt every 125 ms
  RTC_AlarmSubSecondConfig(RTC_Alarm_A, 0xFF, RTC_AlarmSubSecondMask_SS14_6); // One interrupt every 250 ms

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  /* Enable the RTC Alarm A interrupt */
  RTC_ITConfig(RTC_IT_ALRA, ENABLE);

  /* Enable the alarm */
  RTC_AlarmCmd(RTC_Alarm_A, ENABLE);

  /* Clear the Alarm A Pending Bit */
  RTC_ClearITPendingBit(RTC_IT_ALRA);

}

//==========================================================================================
// Configures system clock after wake-up from STOP: enable HSE, PLL  (not implemented)
//==========================================================================================

void SYSCLKConfig_STOP(void)
{
  /* After wake-up from STOP reconfigure the system clock */
  /* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
  {}

  /* Enable PLL */
  RCC_PLLCmd(ENABLE);

  /* Wait till PLL is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
  {}

  /* Select PLL as system clock source */
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

  /* Wait till PLL is used as system clock source */
  while (RCC_GetSYSCLKSource() != 0x08)
  {}
}

//==========================================================================================
// Reset GPIOs MODER registers.
//==========================================================================================

void Init_Reset_GPIOs(void)
{
  // Enable GPIO clocks
  RCC->AHBENR |= (RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN | RCC_AHBENR_GPIOFEN);

  GPIOA->MODER = 0;
  GPIOB->MODER = 0;
  GPIOC->MODER = 0;
  GPIOD->MODER = 0;
  GPIOF->MODER = 0;
}

//==========================================================================================
// Configure GPIOs in "Low Power mode" : Analog Mode, No Pull-Up nor Pull-Down
//==========================================================================================

void Init_LPM_GPIOs(void)
{
  // Enable GPIO clocks
  RCC->AHBENR |= (RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN | RCC_AHBENR_GPIOFEN);

  GPIOA->MODER |= 0xFFFCFFFF; // If PA8 is used (DEBUG_IO)
  //GPIOA->MODER |= 0xFFFFFFFF; // If PA8 is not used (DEBUG_IO)
  GPIOA->PUPDR = 0;

  GPIOB->MODER = 0xFFFFFFFF;
  GPIOB->PUPDR = 0;

  GPIOC->MODER = 0xFFFFFFFF;
  GPIOC->PUPDR = 0;

  GPIOD->MODER = 0xFFFFFFFF;
  GPIOD->PUPDR = 0;

  GPIOF->MODER = 0xFFFFFFFF;
  GPIOF->PUPDR = 0;
}
#endif // USE_LPM

//==========================================================================================
// In case of I2C error print and LCD message.
//==========================================================================================

uint32_t sEE_TIMEOUT_UserCallback(void)
{
  /* Use application may try to recover the communication by resetting I2C
    peripheral (calling the function I2C_SoftwareResetCmd()) then re-start
    the transmission/reception from a previously stored recover point.
    For simplicity reasons, this example only shows a basic way for errors
    managements which consists of stopping all the process and requiring system
    reset. */

	//LCD5110_clear();
	i2c++;
	LCD5110_set_XY(0,1);
	LCD5110_write_string_neg("I2C error");
	delayms1(200);

  /* Block communication and all processes */
return 0;
}


void MyTKeys_ErrorStateProcess(void)
{
  // Add here your own processing when a sensor is in Error state
  //TSL_tkey_SetStateOff();
}


void MyTKeys_OffStateProcess(void)
{
  // Add here your own processing when a sensor is in Off state
  //process_sensor = 0;
}

void MyLinRots_ErrorStateProcess(void)
{
  // Add here your own processing when a sensor is in Error state
 // TSL_linrot_SetStateOff();

}

void MyLinRots_OffStateProcess(void)
{
  // Add here your own processing when a sensor is in Off state
//  process_sensor = 0;
}


//-------------------
// CallBack functions
//-------------------

/**
  * @brief  Executed at each timer interruption (option must be enabled)
  * @param  None
  * @retval None
  */
void TSL_CallBack_TimerTick(void)
{
  //LED1_ON;
  //delayms1(200);
  //LED2_OFF;
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  for (;;)
  {
  }
}
#endif
