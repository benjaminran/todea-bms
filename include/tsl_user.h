/**
  ******************************************************************************
  * @file    STM32F0518_Ex01_3TKeys_EVAL\inc\tsl_user.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    22-February-2013
  * @brief   Touch-Sensing user configuration and api file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TSL_USER_H
#define __TSL_USER_H

#include "tsl.h"



//================================
// GPIOs pins and modes definition
//================================

#define PIN_00 ((uint32_t)0)
#define PIN_01 ((uint32_t)1)
#define PIN_02 ((uint32_t)2)
#define PIN_03 ((uint32_t)3)
#define PIN_04 ((uint32_t)4)
#define PIN_05 ((uint32_t)5)
#define PIN_06 ((uint32_t)6)
#define PIN_07 ((uint32_t)7)
#define PIN_08 ((uint32_t)8)
#define PIN_09 ((uint32_t)9)
#define PIN_10 ((uint32_t)10)
#define PIN_11 ((uint32_t)11)
#define PIN_12 ((uint32_t)12)
#define PIN_13 ((uint32_t)13)
#define PIN_14 ((uint32_t)14)
#define PIN_15 ((uint32_t)15)

#define MODE_OUT ((uint32_t)1)
#define MODE_AF  ((uint32_t)2)
#define MODE_ANA ((uint32_t)3)
#define OTYPE_OD ((uint32_t)1)
#define SPEED_50 ((uint32_t)2)

//=======================
// Channel IOs definition
//=======================

//rotary

#define CHANNEL_3_IO_MSK    (TSL_GROUP1_IO2)
#define CHANNEL_3_GRP_MSK   (TSL_GROUP1)
#define CHANNEL_3_SRC       (0) // Index in source register (TSC->IOGXCR[])
#define CHANNEL_3_DEST      (3) // Index in destination result array

#define CHANNEL_4_IO_MSK    (TSL_GROUP1_IO3)
#define CHANNEL_4_GRP_MSK   (TSL_GROUP1)
#define CHANNEL_4_SRC       (0) // Index in source register (TSC->IOGXCR[])
#define CHANNEL_4_DEST      (4) // Index in destination result array

#define CHANNEL_5_IO_MSK    (TSL_GROUP1_IO4)
#define CHANNEL_5_GRP_MSK   (TSL_GROUP1)
#define CHANNEL_5_SRC       (0) // Index in source register (TSC->IOGXCR[])
#define CHANNEL_5_DEST      (5) // Index in destination result array

//touch key

#define CHANNEL_0_IO_MSK    (TSL_GROUP2_IO1)
#define CHANNEL_0_GRP_MSK   (TSL_GROUP2)
#define CHANNEL_0_SRC       (1) // Index in source register (TSC->IOGXCR[])
#define CHANNEL_0_DEST      (0) // Index in destination result array

#define CHANNEL_1_IO_MSK    (TSL_GROUP2_IO2)
#define CHANNEL_1_GRP_MSK   (TSL_GROUP2)
#define CHANNEL_1_SRC       (1) // Index in source register (TSC->IOGXCR[])
#define CHANNEL_1_DEST      (1) // Index in destination result array

#define CHANNEL_2_IO_MSK    (TSL_GROUP2_IO3)
#define CHANNEL_2_GRP_MSK   (TSL_GROUP2)
#define CHANNEL_2_SRC       (1) // Index in source register (TSC->IOGXCR[])
#define CHANNEL_2_DEST      (2) // Index in destination result array





// Mask for all Channel IOs (used in main)
//#define CHANNEL_ALL_MSK     (CHANNEL_0_IO_MSK | CHANNEL_1_IO_MSK | CHANNEL_2_IO_MSK | CHANNEL_3_IO_MSK | CHANNEL_4_IO_MSK | CHANNEL_5_IO_MSK )

//======================
// Shield IOs definition
//======================

//#define SHIELD_IO_MSK  (0) //none

// Mask for all Channel IOs (used in main)
//#define SHIELD_ALL_MSK      (SHIELD_IO_MSK)

//========================
// Sampling IOs definition
//========================

//#define SAMPLING_0_IO_MSK   (TSL_GROUP1_IO1) // CHANNELS rottary
//#define SAMPLING_0_IO_MSK   (TSL_GROUP2_IO4) // CHANNELS keys

// Mask for all Sampling IOs (used in main)
#define SAMPLING_ALL_MSK    (SAMPLING_0_IO_MSK)

//=================
// Banks definition
//=================

// Bank1 with Shield
#define BANK_0_NBCHANNELS    (2)
#define BANK_0_MSK_CHANNELS  (CHANNEL_0_IO_MSK | CHANNEL_3_IO_MSK  )
#define BANK_0_MSK_GROUPS    (CHANNEL_0_GRP_MSK | CHANNEL_3_GRP_MSK ) // Only these groups will be acquired

// Bank2 with Shield
#define BANK_1_NBCHANNELS    (2)
#define BANK_1_MSK_CHANNELS  (CHANNEL_1_IO_MSK | CHANNEL_4_IO_MSK )
#define BANK_1_MSK_GROUPS    (CHANNEL_1_GRP_MSK | CHANNEL_4_GRP_MSK ) // Only these groups will be acquired

//Bank3 with Shield
#define BANK_2_NBCHANNELS    (2)
#define BANK_2_MSK_CHANNELS  (CHANNEL_2_IO_MSK | CHANNEL_5_IO_MSK )
#define BANK_2_MSK_GROUPS    (CHANNEL_2_GRP_MSK | CHANNEL_5_GRP_MSK ) // Only these groups will be acquired

// Bank4 with Shield
//#define BANK_0_NBCHANNELS    (1)
//#define BANK_0_MSK_CHANNELS  (CHANNEL_3_IO_MSK )
//#define BANK_0_MSK_GROUPS    (CHANNEL_3_GRP_MSK ) // Only these groups will be acquired

// Bank4 with Shield
//#define BANK_1_NBCHANNELS    (1)
//#define BANK_1_MSK_CHANNELS  ( CHANNEL_4_IO_MSK )
//#define BANK_1_MSK_GROUPS    ( CHANNEL_4_GRP_MSK ) // Only these groups will be acquired

// Bank4 with Shield
//#define BANK_2_NBCHANNELS    (1)
//#define BANK_2_MSK_CHANNELS  ( CHANNEL_5_IO_MSK)
//#define BANK_2_MSK_GROUPS    ( CHANNEL_5_GRP_MSK) // Only these groups will be acquired





// User Parameters
extern CONST TSL_Bank_T MyBanks[];
extern CONST TSL_TouchKey_T MyTKeys[];
extern CONST TSL_LinRot_T MyLinRots[];
extern CONST TSL_Object_T MyObjects[];
extern TSL_ObjectGroup_T MyObjGroup;

void TSL_user_Init(void);
TSL_Status_enum_T TSL_user_Action(void);
void TSL_user_SetThresholds(void);

#endif /* __TSL_USER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
