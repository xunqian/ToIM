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

#ifndef _HEADER_VARIABLES_
#define _HEADER_VARIABLES_
#include "stm32f10x.h"
#include "main.h"

//版本号
extern uint8 MODULE_VERSION[8];
extern uint8 CPU_VERSION[7] ;
//序列号
extern uint8 MODULE_SERIAL[20];

//看门狗定时复位
extern uint8   wdt_reseten;
extern uint16  wdt_resetreg;

 
 //uart0,与ECU通信发送接收用变量定义
 //通信缓存
 extern uint8	 receive_ok;//uart0数据接收完成标志  1:成功  0:否
 extern uint8	 inbox[60];//接收数据缓存
 extern uint8	n;
 extern uint8   receive_buff[256];
 extern uint8 sens_selfcheck;  //传感器自检标志
 extern uint8  result;
 extern uint8  detection; 		 //检测区是否有卡标志
 
 extern uint16 drop_tic_time ;	 //落票延时,每次打开电磁铁需从定时器延时
 extern uint16 init_over_time;  //初始化超时
 extern uint16 over_time;	  //通信超时
 extern uint32 rotation_over_time ;  //电机每两步之间的时间间隔
 extern uint32 roll_over_time ;//电机转动超时1s 
 extern uint32 sole_open_over_time ;//入票口电磁铁打开超时1s 
 extern uint32 sole_openleave_over_time ;//入票口电磁铁打开后离开超时100ms 
 extern uint32 sole_close_over_time ;//入票口电磁铁打开超时1s 
 extern uint32 sole2_open_over_time ;//入票口电磁铁打开超时1s 
 extern uint32 sole2_close_over_time;//入票口电磁铁打开超时1s 
 extern uint16 init_delay; 	 //初始化延时，为了等待hopper初始化转动

 
 
extern uint8 antenna;          //天线区是否有卡标志
extern uint8 antennaA ;          //备票区是否有卡标志

extern uint8 clear_tic ;        //初始化与通道清理清除的票数量

 //Hopper出票选择
 extern uint8	hopper;
 //hopper状态
 extern uint8 hopper1_state,hopper2state;

 
 //心跳灯控制  1表示启动心跳灯，频率1HZ   ；2心跳灯加速，有故障
 //0停止跳动
extern  uint8 normal_start; 
 //退币口指示灯标志
 //上位机控制标志 
 //模块控制标志
 extern uint8 LED_EN;
 //退币口指示灯闪烁时间计数
extern  uint16 flash_cnt;
 
 //传感器状态
 extern SENSORS sens;
 extern SENSORS sens_last;
 //单次回收票卡数量
 extern uint8 once_rec_num;
 //电机循环次数
extern  uint8 cycl_count;
 //E2循环计数标记
 extern uint8 A_count_SIGN;
 extern uint8 B_count_SIGN;
 extern uint8 C_count_SIGN;
 //ee存储基本单位
 extern uint8 data[4];
 //ee存储票卡数量地址区间
 extern EE_SAVE e2_Abox[MAX_EE_AREA];
 extern EE_SAVE e2_Bbox[MAX_EE_AREA];
 extern EE_SAVE e2_Cbox[MAX_EE_AREA];
 extern EE_SAVE *headA;
 extern EE_SAVE *headB;
 extern EE_SAVE *headC;
 //回收or退换票卡数量
 extern uint32 box_num[3];
 extern uint8 sum_open_over_time;
  
 //通信步骤
extern uint8 communication_step;
 
 //清币数量
 extern uint8 tic_num1[2],tic_num2[2];
 //清币完成标志
 extern uint8 tic_flag1,tic_flag2;
 
extern RESEIVE hop1_re,hop2_re;
extern uint8 sta_hopper1,sta_hopper2;  //hopper状态
 //模块类型变量
 extern uint8 Module_Type;

 
 //RFID超时延时
extern  uint16 g_cbWaitRespDly;
extern  uint16 work_light_cnt;
 //退币口指示灯控制标志
extern uint8 LED_control_flag;//0:  模块控制1 :上位机控制
 
extern uint16 OutGPA;
extern uint16 OutGPB;
extern uint16 OutGPC;
extern uint16 OutGPD;
extern uint16 OutGPE;

//RFID超时延时
extern uint16 g_cbWaitRespDly ;

extern uint8 communication_step;

extern uint8 Check_LRC(uint8 *ptr, uint8 len) ;
extern void cmd_act(uint8 cmd);

#endif

