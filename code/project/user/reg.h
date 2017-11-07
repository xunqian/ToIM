/*****************************************************************************/
/*                                                                                                                                                      */
/*    Copyright (C) - WINGAIN Intelligent Equipment - All rights reserved                                                */
/*                                                                                                                                                      */
/*****************************************************************************/
/*                                                                                                                                                      */
/*  Except if expressly provided in a dedicated License Agreement, you are                                           */
/*  not authorized to:                                                                                                                         */
/*                                                                                                                                                      */
/*  1. Use, copy, modify or transfer this software component, module or                                             */
/*  product, including any accompanying electronic or paper documentation                                          */
/*  (together, the "Software").                                                                                                            */
/*                                                                                                                                                       */
/*  2. Remove any product identification, copyright, proprietary notices or                                            */
/*  labels from the Software.                                                                                                               */
/*                                                                                                                                                      */
/*  3. Modify, reverse engineer, decompile, disassemble or otherwise attempt                                      */
/*  to reconstruct or discover the source code, or any parts of it, from the                                           */
/*  binaries of the Software.                                                                                                                */
/*                                                                                                                                                       */
/*  4. Create derivative works based on the Software (e.g. incorporating the                                        */
/*  Software in another software or commercial product or service without a                                        */
/*  proper license).                                                                                                                              */
/*                                                                                                                                                      */
/*  By installing or using the "Software", you confirm your acceptance of the                                        */
/*  hereabove terms and conditions.                                                                                                   */
/*                                                                                                                                                      */
/*****************************************************************************/

/*****************************************************************************/
/*  History:                                                                                                                                        */
/*****************************************************************************/
/*  Date       * Author          * Changes                                                                                               */
/*****************************************************************************/
/*  2017-6-21 * xunqian.hu      * Creation of the file                                                                         */
/*             *                 *                                                                                                                    */
/*****************************************************************************/
/*  Target : stm32                                                                                                                               */
/*  Crystal: 72Mhz                                                                                                                              */
/*****************************************************************************/


/*****************************************************************************/
/*                                                                                                                                                      */
/*  Include Files                                                                                                                                   */
/*                                                                                                                                                      */
/*****************************************************************************/

#ifndef _HEADER_REG_
#define _HEADER_REG_
#include "stm32f10x.h"
// safety zone sensors
#define SENS1_PORT                (GPIOA)
#define SENS1_BIT_NUM             (GPIO_Pin_8)//IN

#define SENS2_PORT                (GPIOC)
#define SENS2_BIT_NUM             (GPIO_Pin_9)//IN

#define SENS3_PORT                (GPIOC)
#define SENS3_BIT_NUM             (GPIO_Pin_8)//IN

#define SENS4_PORT                (GPIOC)
#define SENS4_BIT_NUM             (GPIO_Pin_7)//IN


#define SENS5_PORT                (GPIOC)
#define SENS5_BIT_NUM             (GPIO_Pin_6)//IN

#define SENS6_PORT                (GPIOD)
#define SENS6_BIT_NUM             (GPIO_Pin_15)//IN

#define SENS7_PORT                (GPIOD)
#define SENS7_BIT_NUM             (GPIO_Pin_14)//IN

#define SENS8_PORT                (GPIOD)
#define SENS8_BIT_NUM             (GPIO_Pin_13)//IN

#define SENS9_PORT                (GPIOD)
#define SENS9_BIT_NUM             (GPIO_Pin_12)//IN

#define SENS10_PORT               (GPIOD)
#define SENS10_BIT_NUM            (GPIO_Pin_11)//IN

#define SENS11_PORT               (GPIOD)
#define SENS11_BIT_NUM            (GPIO_Pin_10)//IN

#define SENS12_PORT               (GPIOD)
#define SENS12_BIT_NUM            (GPIO_Pin_9)//IN

#define SENS_POWER_PORT           (GPIOD)
#define SENS_POWER_BIT_NUM        (GPIO_Pin_8)//OUT

#define V_CHECK2_PORT             (GPIOE)
#define V_CHECK2_BIT_NUM          (GPIO_Pin_2)//IN

#define V_CHECK1_PORT             (GPIOE)
#define V_CHECK1_BIT_NUM          (GPIO_Pin_3)//IN

#define MOTOR_1F1_PORT            (GPIOE)
#define MOTOR_1F1_BIT_NUM         (GPIO_Pin_4)//OUT

#define MOTOR_1F2_PORT            (GPIOE)
#define MOTOR_1F2_BIT_NUM         (GPIO_Pin_5)//OUT

#define MOTOR_2F1_PORT            (GPIOE)
#define MOTOR_2F1_BIT_NUM         (GPIO_Pin_6)//OUT

#define MOTOR_2F2_PORT            (GPIOC)
#define MOTOR_2F2_BIT_NUM         (GPIO_Pin_13)//OUT

#define LED_A_PORT                (GPIOC)
#define LED_A_BIT_NUM             (GPIO_Pin_14)//OUT

#define LED_B_PORT                (GPIOC)
#define LED_B_BIT_NUM             (GPIO_Pin_15)//OUT

#define LED_C_PORT                (GPIOC)
#define LED_C_BIT_NUM             (GPIO_Pin_0)//OUT

#define LED_DP_PORT               (GPIOC)
#define LED_DP_BIT_NUM            (GPIO_Pin_1)//OUT

#define LED_D_PORT                (GPIOC)
#define LED_D_BIT_NUM             (GPIO_Pin_2)//OUT

#define LED_E_PORT                (GPIOC)
#define LED_E_BIT_NUM             (GPIO_Pin_3)//OUT

#define LED_F_PORT                (GPIOA)
#define LED_F_BIT_NUM             (GPIO_Pin_1)//OUT

#define LED_G_PORT                (GPIOA)
#define LED_G_BIT_NUM             (GPIO_Pin_0)//OUT

#define RFID_OUT5_PORT            (GPIOD)
#define RFID_OUT5_BIT_NUM         (GPIO_Pin_0)//OUT


#define RFID_OUT4_PORT            (GPIOD)
#define RFID_OUT4_BIT_NUM         (GPIO_Pin_1)//OUT

#define RFID_OUT3_PORT            (GPIOB)
#define RFID_OUT3_BIT_NUM         (GPIO_Pin_0)//OUT

#define RFID_OUT2_PORT            (GPIOC)
#define RFID_OUT2_BIT_NUM         (GPIO_Pin_5)//OUT

#define RFID_OUT1_PORT            (GPIOC)
#define RFID_OUT1_BIT_NUM         (GPIO_Pin_4)//OUT

#define OUT9_PORT                 (GPIOA)
#define OUT9_BIT_NUM              (GPIO_Pin_12)//OUT

#define OUT8_PORT                 (GPIOA)
#define OUT8_BIT_NUM              (GPIO_Pin_11)//OUT

#define OUT7_PORT                 (GPIOE)
#define OUT7_BIT_NUM              (GPIO_Pin_7)//OUT

#define OUT6_PORT                 (GPIOE)
#define OUT6_BIT_NUM              (GPIO_Pin_8)//OUT

#define OUT5_PORT                 (GPIOE)
#define OUT5_BIT_NUM              (GPIO_Pin_9)//OUT

#define OUT4_PORT                 (GPIOE)
#define OUT4_BIT_NUM              (GPIO_Pin_10)//OUT

#define OUT3_PORT                 (GPIOE)
#define OUT3_BIT_NUM              (GPIO_Pin_11)//OUT

#define OUT2_PORT                 (GPIOE)
#define OUT2_BIT_NUM              (GPIO_Pin_12)//OUT

#define OUT1_PORT                 (GPIOE)
#define OUT1_BIT_NUM              (GPIO_Pin_13)//OUT

#define KEY1_PORT                 (GPIOE)
#define KEY1_BIT_NUM              (GPIO_Pin_14)//IN

#define KEY2_PORT                 (GPIOE)
#define KEY2_BIT_NUM              (GPIO_Pin_15)//IN

#define SOLE1_PORT                (GPIOE)
#define SOLE1_BIT_NUM             (GPIO_Pin_1)//OUT

#define SOLE2_PORT                (GPIOE)
#define SOLE2_BIT_NUM             (GPIO_Pin_0)//OUT

#define SOLE3_PORT                (GPIOB)
#define SOLE3_BIT_NUM             (GPIO_Pin_9)//OUT

#define SOLE4_PORT                (GPIOB)
#define SOLE4_BIT_NUM             (GPIO_Pin_8)//OUT

#define SOLE5_PORT                (GPIOB)
#define SOLE5_BIT_NUM             (GPIO_Pin_7)//OUT

#define IN6_PORT                  (GPIOB)
#define IN6_BIT_NUM               (GPIO_Pin_6)//IN

#define IN5_PORT                  (GPIOB)
#define IN5_BIT_NUM               (GPIO_Pin_5)//IN

#define IN4_PORT                  (GPIOD)
#define IN4_BIT_NUM               (GPIO_Pin_7)//IN

#define IN3_PORT                  (GPIOD)
#define IN3_BIT_NUM               (GPIO_Pin_6)//IN

#define IN2_PORT                  (GPIOD)
#define IN2_BIT_NUM               (GPIO_Pin_5)//IN

#define IN1_PORT                  (GPIOD)
#define IN1_BIT_NUM               (GPIO_Pin_4)//IN

// bitmaps of GPIO bits used
#define GPA_IN_BITMAPS     (GPIO_Pin_8)
#define GPA_OUT_BITMAPS    (GPIO_Pin_12|GPIO_Pin_11|GPIO_Pin_1|GPIO_Pin_0)

#define GPB_IN_BITMAPS     (GPIO_Pin_6|GPIO_Pin_5)
#define GPB_OUT_BITMAPS    (GPIO_Pin_9|GPIO_Pin_8|GPIO_Pin_7|GPIO_Pin_0)

#define GPC_IN_BITMAPS     (GPIO_Pin_9|GPIO_Pin_8|GPIO_Pin_7|GPIO_Pin_6)
#define GPC_OUT_BITMAPS    (GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15|GPIO_Pin_0\
							|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5)

#define GPD_IN_BITMAPS     (GPIO_Pin_15|GPIO_Pin_14|GPIO_Pin_13|GPIO_Pin_12\
														|GPIO_Pin_11|GPIO_Pin_10|GPIO_Pin_9|GPIO_Pin_7\
														|GPIO_Pin_6|GPIO_Pin_5|GPIO_Pin_4)
#define GPD_OUT_BITMAPS    (GPIO_Pin_8|GPIO_Pin_1|GPIO_Pin_0)

#define GPE_IN_BITMAPS     (GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_14|GPIO_Pin_15)
#define GPE_OUT_BITMAPS    (GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8\
														|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13\
														|GPIO_Pin_1|GPIO_Pin_0)
														
#define SOLE1_ON   GPIO_WriteBit(SOLE1_PORT, SOLE1_BIT_NUM, Bit_SET)
#define SOLE1_OFF  GPIO_WriteBit(SOLE1_PORT, SOLE1_BIT_NUM, Bit_RESET)

#define SOLE2_ON   GPIO_WriteBit(SOLE2_PORT, SOLE2_BIT_NUM,Bit_SET)
#define SOLE2_OFF  GPIO_WriteBit(SOLE2_PORT, SOLE2_BIT_NUM, Bit_RESET)

#define SOLE3_ON   GPIO_WriteBit(SOLE3_PORT, SOLE3_BIT_NUM, Bit_SET)
#define SOLE3_OFF  GPIO_WriteBit(SOLE3_PORT, SOLE3_BIT_NUM, Bit_RESET)

#define SOLE4_ON   GPIO_WriteBit(SOLE4_PORT, SOLE4_BIT_NUM, Bit_SET)
#define SOLE4_OFF  GPIO_WriteBit(SOLE4_PORT, SOLE4_BIT_NUM, Bit_RESET)

#define SOLE5_ON   GPIO_WriteBit(SOLE5_PORT, SOLE5_BIT_NUM, Bit_SET)
#define SOLE5_OFF  GPIO_WriteBit(SOLE5_PORT, SOLE5_BIT_NUM, Bit_RESET)

#define MOTOR_1F1_SET GPIO_WriteBit(MOTOR_1F1_PORT, MOTOR_1F1_BIT_NUM, Bit_SET)
#define MOTOR_1F1_RESET GPIO_WriteBit(MOTOR_1F1_PORT,MOTOR_1F1_BIT_NUM, Bit_RESET)

#define MOTOR_1F2_SET GPIO_WriteBit(MOTOR_1F2_PORT, MOTOR_1F2_BIT_NUM, Bit_SET)
#define MOTOR_1F2_RESET GPIO_WriteBit(MOTOR_1F2_PORT, MOTOR_1F2_BIT_NUM, Bit_RESET)

//传感器电源控制
#define  SensorP_ON     GPIO_WriteBit(SENS_POWER_PORT, SENS_POWER_BIT_NUM, Bit_SET)
#define  SensorP_OFF    GPIO_WriteBit(SENS_POWER_PORT, SENS_POWER_BIT_NUM, Bit_RESET)
//心跳灯指示
#define  WORK_LIGHT_ON     GPIO_WriteBit(OUT9_PORT, OUT9_BIT_NUM, Bit_SET)
#define  WORK_LIGHT_OFF    GPIO_WriteBit(OUT9_PORT, OUT9_BIT_NUM, Bit_RESET) 

//天线区有票指示D16
#define  antenna_LED_ON    GPIO_WriteBit(OUT2_PORT, OUT2_BIT_NUM, Bit_SET)
#define  antenna_LED_OFF   GPIO_WriteBit(OUT2_PORT, OUT2_BIT_NUM, Bit_RESET)
//备票区有票指示D17
#define  antennaA_LED_ON    GPIO_WriteBit(OUT3_PORT, OUT3_BIT_NUM, Bit_SET)
#define  antennaA_LED_OFF   GPIO_WriteBit(OUT3_PORT, OUT3_BIT_NUM, Bit_RESET)


//RFID天线1

//#define  RFID_ANTENNA1_ON    GPIO_WriteBit(OUT7_PORT, OUT7_BIT_NUM, Bit_RESET)
//#define  RFID_ANTENNA1_OFF   GPIO_WriteBit(OUT7_PORT, OUT7_BIT_NUM, Bit_SET)


#define  RFID_ANTENNA1_ON    GPIO_WriteBit(RFID_OUT1_PORT, RFID_OUT1_BIT_NUM, Bit_RESET)
#define  RFID_ANTENNA1_OFF   GPIO_WriteBit(RFID_OUT1_PORT, RFID_OUT1_BIT_NUM, Bit_SET)

//RFID天线2
#define  RFID_ANTENNA2_ON    GPIO_WriteBit(RFID_OUT2_PORT, RFID_OUT2_BIT_NUM, Bit_RESET)
#define  RFID_ANTENNA2_OFF   GPIO_WriteBit(RFID_OUT2_PORT, RFID_OUT2_BIT_NUM, Bit_SET)

//RFID天线3
#define  RFID_ANTENNA3_ON    GPIO_WriteBit(RFID_OUT3_PORT, RFID_OUT3_BIT_NUM, Bit_RESET)
#define  RFID_ANTENNA3_OFF   GPIO_WriteBit(RFID_OUT3_PORT, RFID_OUT3_BIT_NUM, Bit_SET)

//RFID天线4
#define  RFID_ANTENNA4_ON    GPIO_WriteBit(RFID_OUT4_PORT, RFID_OUT4_BIT_NUM, Bit_RESET)
#define  RFID_ANTENNA4_OFF   GPIO_WriteBit(RFID_OUT4_PORT, RFID_OUT4_BIT_NUM, Bit_SET)

//RFID天线5
#define  RFID_ANTENNA5_ON    GPIO_WriteBit(RFID_OUT5_PORT, RFID_OUT5_BIT_NUM, Bit_RESET)
#define  RFID_ANTENNA5_OFF   GPIO_WriteBit(RFID_OUT5_PORT, RFID_OUT5_BIT_NUM, Bit_SET)







#endif
