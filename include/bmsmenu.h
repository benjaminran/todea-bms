/*
==========================================================================
    ElectroDacus Solar BMS "SBMS4080" menu file.

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

static char* const mainmenu[][2] =
{
		{"  Monitoring  ","> Battery monitor "},
		{" Parameter Set","> Battery parameter settings "},
		{"  Device Set  ","> Device settings "},
		{"  Diagnostic  ","> Diagnostic "},
		{"    About     "," www.electrodacus.com "}

};

static char* const devset[][2] =
{
		{"  Time&Date   ","> Time and date "},
		{"LCD & TouchKey","> LCD brightness and TouchKey lock settings "},
		{"    USART     ","> USART settings  "},
		{" uSD-datalog  ","> micro-SD data log settings "},
		{"EXT1 & EXT2 IO","> The two external IO pins available on the 5 pin programming connector pin 3-GND 4-EXT1 5-EXT2"}
};

static char* const timedate[][3] =
{
		{"Hour    ","> Set Hour ","h "},
		{"Minutes ","> Set minutes ","m "},
		{"Day     ","> Set day ","  "},
		{"Month   ","> Set month ","  "},
		{"Year    ","> Set year  ","  "}
};

static char* const LCDbacklight[][3] =
{
		{"BK-Light","> Set Back-light level "," "},
		{"BK-time ","> Set Backlight time on in minutes ","m"},
		{"KeyLock ","> Set Capacitive Touch Key's lock time in minutes ","m"}
};

static char* const EXT12[][3] =
{
		{"EXT1Type","> EXT1 (0=OFF) (1=HVD High Voltage Disconnect) (2=SOC State of Charge Alarm level set if EXT1 < SOC).  "," "},
		{"EXT1 SOC","> EXT1 SOC % level. Active if EXT1Type is set at option 2.  ","%"},
		{"EXT2Type","> EXT2 (0=OFF) (1=LVD Low Voltage Disconnect) (2=SOC State of Charge Alarm level set if EXT2 > SOC).  "," "},
		{"EXT2 SOC","> EXT2 SOC % level. Active if EXT2Type is set at option 2.  ","%"}
};



static char* const counterreset[][2] =
{
		{" NONE  ","> No reset on any counter  "},
		{"  Ah   ","> Reset the Ah counter "},
		{"  Wh   ","> Reset the Wh counter "},
		{"Ah & Wh","> Reset both the Ah and Wh counter "}
};



static unsigned int const timedateval[5][5] =
{
		{2,0,0,23,1},     //1
		{2,0,0,59,1},     //2
		{2,0,1,31,1},     //4
		{2,0,1,12,1},     //4
		{4,0,2014,2114,1},//5
};

static unsigned int const realcurrent[8][4] =
{
		{0,16,4,64},     //1  ID,Discharge,Charge,ShortCircuit
		{1,32,8,96},     //2
		{2,64,16,128},     //3
		{3,96,24,192},     //4
		{4,128,32,256},     //5
		{5,192,48,256},     //6
		{6,192,64,256},     //7
		{7,192,64,256},     //8
};


static unsigned int const backlightval[3][5] =
{
		{1,0,0,2,1},     //1
		{2,0,0,59,1},     //2
		{2,0,0,59,1},     //3
};

static unsigned int const EXT12val[4][5] =
{
		{1,0,0,2,1},     //
		{2,0,1,99,1},     //
		{1,0,0,2,1},     //
		{2,0,1,99,1},     //
};





static char* const USARTmenu[][3] =
{
		{"Data Log","> USART Data Logging enable or disable "," "},
		{"Baud    ","> Set USART Baud rate default 10.0K works OK for 9600 ","K"},
		{"Interval","> Set the data logging time interval ","s"}
};

static unsigned int const USARTval[5][5] =
{
		{1,0,0,1,1},     //1
		{4,1,1,1500,1},     //2
		{4,0,1,3599,1},     //4
};

static char* const EFT[15]=
{
"OV  ",
"OVLO",
"UV  ",
"UVLO",
"IOT ",
"COC ",
"DOC ",
"DSC ",
"CELF",
"OPEN",
"LVC ",
"ECCF",
"CFET",
"EOC ",
"DFET"
};

static unsigned int const BatteryType [34][8] =
{
		{1,2,3,4,5,6,7},                     //1
		{8,8,8,8,8,8,8},                     //2
		{100,100,100,100,100,300,300},       //3
		{355,365,395,410,420,225,265},       //4
		{1,1,1,1,1,1,1},                     //5
		{345,350,385,390,415,205,245},       //6
		{300,280,340,320,270,100,100},       //7
		{1,1,1,1,1,1,1},                     //8
		{320,310,360,350,300,120,120},       //9
		{365,370,420,425,435,230,270},       //10
		{270,250,300,290,180,90,90},         //11
		{347,350,390,405,420,215,260},       //12
		{230,230,230,230,230,80,80},         //13
		{48,48,48,48,48,48,48},              //14
		{160,160,160,160,160,160,160},       //15
		{96,96,96,96,96,96,96},              //16
		{160,160,160,160,160,160,160},       //17
		{128,128,128,128,128,128,128},       //18
		{200,200,200,200,200,200,200},       //19
		{0,0,0,0,0,0,0},                     //20
		{1,1,1,1,1,1,1},                     //21
		{320,320,330,330,310,170,190},       //22
		{350,360,395,410,413,225,265},       //23
		{15,15,19,19,19,10,10},              //24
		{300,400,400,500,500,700,700},       //25
		{5,5,5,5,5,2,2},                     //26
		{2,2,2,2,2,2,2},                     //27
		{719,719,719,719,719,719,719},       //28
		{4096,4096,4096,4096,4096,4096,4096},//29
		{0,0,0,0,0,0,0},                     //30
		{220,220,200,200,200,80,80},         //31
		{15,15,15,15,15,15,15},       //32
		{15,15,15,15,15,15,15},              //33
		{240,240,240,240,240,240,240}        //34


};



static char* const bmsmenu[][3] =
{
		{"CellType",">(1 LiFePO4 3.5V)(2 LiFePO4 3.6V)(3 LiCoO2 3.95V)(4 LiCoO2 4.1V)(5 LiCoO2 4.2V)(6 SuperCap 2.3V)(7 SuperCap 2.7V)","  "},
		{"Nr-Cells",">Number of series connected cells. For 3cell(1,2,8) 4cell(1,2,7,8) 5cell(1,2,3,7,8) 6cell(1,2,3,6,7,8) 7cell(1,2,3,4,6,7,8)","  "},
		{"Capacity",">Capacity of the bank in Ah or Farad","HH"},
		{"OverV   ",">Overvoltage Threshold *If any cell voltage is above this for an overvoltage delay time the charge FET is turned OFF.","V "},
		{"OVdelay ",">Overvoltage Delay Time Out *Sets the time that is required for any cell to be above the overvoltage threshold before condition is detected.","s "},
		{"OverVR  ",">Overvoltage Recovery *If all cells fall below this the charge FET is turned ON.","V "},
		{"UnderV  ",">Undervoltage Threshold *If any cell voltage is below this for an undervoltage delay time the discharge FET is turned OFF.","V "},
		{"UVdelay ",">Undervoltage Delay Time Out *Sets the time that is required for any cell to be below the undervoltage threshold before condition is detected.","s "},
		{"UnderVR ",">Undervoltage Recovery *If all cells rise above this (and there is no load) the discharge FET is turned ON.","V "},
		{"OverVLK ",">Overvoltage Lock-out Threshold *If any cell is above this the charge FET is turned OFF the cell balance FET's are turned OFF and OVLO bit is set.","V "},
		{"UnderVLK",">Undervoltage Lock-out Threshold *If any cell is below this the discharge FET is turned OFF and the UVLO bit is set.","V "},
		{"EndOfChg",">End-of-Charge Threshold *If any cell exceeds this the EOC bit is set.","V "},
		{"LowVCL  ",">Low Voltage Charge Level *If the voltage on any cell is below this then precharge FET turns ON to disable this set to zero.","V "},
		{"OverIChg",">Charge Overcurrent Threshold *If current is above this for the time set at ICdelay.","A "},
		{"ICdelay ",">Charge Overcurrent Time Out *A charge overcurrent condition needs to remain for this time period prior to entering a charge overcurrent condition.","s "},
		{"OverIDsg",">Discharge Overcurent Threshold *If current is above this for the time set at IDdelay.","A "},
		{"IDdelay ",">Discharge Overcurent Time Out *A discharge overcurent condition needs to remain for this time period to entering a discharge overcurrent condition.","s "},
		{"IDShort ",">Discharge Short Circuit Threshold.","A "},
		{"IDSdelay",">Discharge Short Circuit Time Out A short circuit current needs to remain for this time prior entering a short circuit condition.","us"},
		{"CBONDsg ",">If this is 1 = ON if 0 = OFF.","  "},
		{"CBONChg ",">If this is 1 = ON if 0 = OFF.","  "},
		{"CBmin   ",">Cell balance minimum.","V "},
		{"CBmax   ",">Cell balance maximum.","V "},
		{"CBminDV ",">Cell balance min delta.","mV"},
		{"CBmaxDV ",">Cell balance max delta.","mV"},
		{"CB-ON-t ",">Cell balance ON time.","s "},
		{"CB-OFF-t",">Cell balance OFF time.","s "},
		{"InOverT ",">Internal over temperature according to spec 730mV==115C and 690mV==95C so this is the default setting do not change leave 730mV","mV"},
		{"MaxTEMPI",">External over (temperature or something else) the values are direct digital conversion 12bit ADC so 0 to 4095 the uC will decide what to do","b "},
		{"MinTEMPI",">External under (temperature or something else) the values are direct digital conversion 12bit ADC so 0 to 4095 the uC will decide what to do","b "},
		{"SleepV  ",">Sleep Voltage.","V "},
		{"SleepDly",">Sleep Delay.","s "},
		{"Doze    ",">Doze timer.","m "},
		{"Sleep   ",">Sleep timer.","m "},
		{">   Store   <",">Store all parameters to main IC. It will need a disconnect and reconnect to battery for the changes to take effect.",">>"},
		{" "," "," "},
		{'\0','\0'}

};

static unsigned int const bmsmenuval[36][5] =
{
		{1,0,1,7,1},    //1
		{1,0,3,8,1},     //2
		{4,0,1,9999,1},  //3
		{3,2,80,480,1},  //4
		{3,0,0,999,1},   //5
		{3,2,80,480,1},  //6
		{3,2,80,480,1},  //7
		{3,0,0,999,1},   //8
		{3,2,80,480,1},  //9
		{3,2,80,480,1},  //10
		{3,2,80,480,1},  //11
		{3,2,80,480,1},  //12
		{3,2,80,480,1},  //13
		{2,0,0,6,1},     //14
		{3,0,0,999,1},   //15
		{3,0,0,4,1},     //16
		{3,0,0,999,1},   //17
		{3,0,0,3,1},     //18
		{3,0,0,999,1},   //19
		{1,0,0,1,1},     //20
		{1,0,0,1,1},     //21
		{3,2,80,480,1},  //22
		{3,2,80,480,1},  //23
		{4,0,0,4800,1},  //24
		{4,0,0,4800,1},  //25
		{3,0,0,999,1},   //26
		{3,0,0,999,1},   //27
		{3,0,690,730,1}, //28
		{4,0,0,4096,1},  //29
		{4,0,0,4096,1},  //30
		{3,2,80,480,1},  //31
		{3,0,0,999,1},   //32
		{2,0,0,16,1},    //33
		{3,0,0,240,1},   //34



};
