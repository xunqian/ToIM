/*****************************************************************************/
/*                                                                                                                                                      */
/*    Copyright (C) - WINGAIN Intelligent Equipment - All rights reserved                                                */
/*                                                                                                                                                      */
/*****************************************************************************/
/*                                                                                                                                                      */
/*  Except if expressly provided in a dedicated License Agreement, you are                         */
/*  not authorized to:                                                                                                                         */
/*                                                                                                                                                      */
/*  1. Use, copy, modify or transfer this software component, module or                           */
/*  product, including any accompanying electronic or paper documentation                       */
/*  (together, the "Software").                                                                                                            */
/*                                                                                                                                                       */
/*  2. Remove any product identification, copyright, proprietary notices or                         */
/*  labels from the Software.                                                                                       */
/*                                                                                                                          */
/*  3. Modify, reverse engineer, decompile, disassemble or otherwise attempt                    */
/*  to reconstruct or discover the source code, or any parts of it, from the                         */
/*  binaries of the Software.                                                                                        */
/*                                                                                                                           */
/*  4. Create derivative works based on the Software (e.g. incorporating the                      */
/*  Software in another software or commercial product or service without a                        */
/*  proper license).                                                                                                     */
/*                                                                                                                             */
/*  By installing or using the "Software", you confirm your acceptance of the                       */
/*  hereabove terms and conditions.                                                                              */
/*                                                                                                                                                      */
/*****************************************************************************/

/*****************************************************************************/
/*  History:                                                                                                                                        */
/*****************************************************************************/
/*  Date       * Author          * Changes                                                                                               */
/*****************************************************************************/
/*  2017-7-15 *xunqian.hu      * Creation of the file                                                       */
/*             *                 *                                                                                                                    */
/*****************************************************************************/
/*  Target : stm32                                                                                                      */
/*  Crystal: 72Mhz                                                                                                       */
/*****************************************************************************/


/*****************************************************************************/
/*                                                                                                                             */
/*  Include Files                                                                                                          */
/*                                                                                                                             */
/*****************************************************************************/

#ifndef _HEADER_MAIN_
#define _HEADER_MAIN_
#include "stm32f10x.h"
//
#define CMD_NO_ACTION 0
//#define CMD_TOrwAREA 1
#define CMD_A_RECOVER 1//回收到A
#define CMD_B_RECOVER 2//回收到B
#define CMD_RETURN 3//回收到C或退币
#define CMD_REJECTACCEPT 4//禁止接受票卡
#define CMD_ACCEPT 5//允许接受票卡



#define OFF 0
#define ON 1

//cmd
#define WXDLE 0X10        //控制符
#define WXSTX 0X02        //数据包开始标志
#define WXETX 0X03        //数据包结束标志
#define WXEOT 0X04        //通讯中止标志
#define WXENQ 0X05        //命令执行确认
#define WXACK 0X06        //接收正确回应
#define WXNAK 0X15        //接收错误回应

//E2地址
//#define E2ERR_ADDR 0x22	//E2故障存储的地址
//#define E2BOXA_STATUS 0x23	//票箱装卸存储地址
//#define E2BOXB_STATUS 0x24	//票箱装卸存储地址
//#define E2BOXC_STATUS 0x25	//票箱装卸存储地址
#define ABOX_TICKETS_ADDR 0x40       //A票卡计数基地址，循环存10个32位累计值和10个地址8位
#define BBOX_TICKETS_ADDR 0x80       //B票卡计数基地址，循环存10个32位累计值和10个地址8位
#define CBOX_TICKETS_ADDR 0x120       //C票卡计数基地址，循环存10个32位累计值和10个地址8位
#define MAX_EE_AREA 10
#define AREA_A 1
#define AREA_B 2
#define AREA_C 3
#define A 0
#define B 1
#define C 2
#define model_ADDR   0x20//模块类型存储地址
#define model_SERIAL_ADDR   0x21//模块序列号基地址，存20个8位序列号
//模块类型
#define HAVE_TEMPORARY_AREA 1   //有暂存区，默认
#define NO_TEMPORARY_AREA 2     //无暂存区

//电机转动方向，速度
//uint8 cw;
//uint16 speed;
//旋转方向
#define STOP 0        //停止
#define COROTATION 1        //正转
#define ROLLBACK 2        //反转

//落票时间
#define DROP_T1  300
#define DROP_T2  150
#define DROP_T3  200
#define DCT_T1     20
#define DCT_T2     20
#define DCT_T3     40

// 读取信息时间
#define READ_MAS  20   //2//读状态超时是60ms
#define INTI_OVER_TIME 500
//一秒时间
#define SECOND  5000
//一秒时间
#define COM_OVER_TIME  10

//票卡状态
#define EXISTENCE 1         //存在
#define INEXISTENCE 0       //不存在
#define DIG_TICKET 2        //挖票过程中
#define EMPTY 4            //表示hopper已空
#define NO_STA  5            //等待超时
//
#define CMD_NO_ACTION 0
#define CMD_BOX1SCOOP 1
#define CMD_BOX2SCOOP 2
#define CMD_TICKETSELL 3
#define CMD_TICKETRECYCLE 4

typedef struct//结构体
{
	uint32  limit_cnt;
	uint8	limit_en;
}CNT;

typedef union//共用体、联合体
{
	uint8	status[18];
	struct
	{			
		uint8   checkticks1;//sens1:备票区对射传感器，遮挡为0
		uint8   checkticks2;//sens2:天线区对射传感器，遮挡为0
		uint8   checkticks3;//sens3:回收口对射传感器，遮挡为0
		uint8   checkticks4;//备用
		uint8   checkticks5;//备用
		uint8   checkticks6;//备用
		uint8   checkticks7;//备用
		uint8   checkticks8;//备用		
		uint8   checkticks9;//备用
		uint8   checkticks10;//备用
		uint8   checkticks11;//备用
		//到位开关
		uint8   checkticks12;//s1：废票箱到位  0：到位 1：未到位
		uint8   checkticks13;//s2：清票箱到位  0：到位 1：未到位
		uint8   checkticks14;//s3:	模块到位      0:    到位 1:  	未到位
		uint8   checkticks15;//备用
		uint8   checkticks16;//备用
		uint8   checkticks17;//备用
		uint8   checkticks18;//备用
	}SENSORS_STATUS;
}SENSORS;

typedef struct
{
	uint8	addr;
	uint8	data[4];
}EE_SAVE;

typedef union
{
    uint8   code[53];
    struct
    {
        uint8   len;        //当前包的长度
    	uint8	act_code;   //响应码
    	uint8	result;     //返回结果:'e'   'w'   's'
        uint8   err_code;   //故障代码
        uint8   info[49];   //各种信息
    }MESSAGE;
}RETURN_CODE;

typedef enum //枚举
{
	com_ok = 0,					    //无错误
	no_card_at_RW_area,		    //无卡在天线区
	read_clearnum_be_clear,      //未清除币读取清币数量
	card_at_RW_area,              //有卡在天线区告警
	box1_non_existent = 0x39,			//A票箱未到位
	box2_non_existent,			//B票箱未到位
	boxA_empty = 0x3c,			    //A票箱空
	boxB_empty,		            //B票箱空
	card_block_at_RW_area = 0x3f,//票卡在天线区卡票
	card_block_at_exit_area,		//票卡在出票口卡票
	card_block_at_Aexit_area,	//票卡在A出票口卡票
	AWK_no_card,					//A挖卡轮挖不出票卡
	BWK_no_card,					//B挖卡轮挖不出票卡
	equ_busy = 0x4a,				 //设备忙
	clr_err,                        //清空失败
	motor_err = 0x50,              //电机转动故障
	Antenna_sens_err=0x61,        //天线区传感器故障
	TS_sens_err,                  //备票区传感器故障
	Recy_sens_err,	                //回收口传感器故障
//	checkticks_err,				//通道传感器故障
	sole1_err,                     //电磁铁1故障
	sole2_err,                     //电磁铁2故障
	rf232A_err,                    //串口A故障
	rf232B_err,                    //串口B故障
	sole3_err,                     //电磁铁3故障
	sole4_err,                     //电磁铁4故障
	motor_sens_err,                //电机定位传感器故障
	Hopper1_Outcoin_sens_err,    //HOPPER1出币传感器故障
	Hopper2_Outcoin_sens_err,    //HOPPER2出币传感器故障
	Hopper1_Clrcoin_sole_err,    //Hopper1清币电磁铁故障
	Hopper2_Clrcoin_sole_err,    //Hopper2清币电磁铁故障
	no_RFID_card = 0xa1,          //无法检测到卡箱的 RFID 卡
    RFID_iden_err,                //卡箱 RFID 卡认证失败
    RW_RFID_err,                  //读写卡箱 RFID 卡参数错
    read_SN_err,                  //读卡箱 SN 号错
    write_SN_err,                 //写卡箱 SN 号错
    invalid_parameter=0x31,//参数无效
}CMD_STATUS;

typedef struct
{
    uint8 re_data[30];              //接收到的数据
    uint8 re_index;                 //接收计数
    uint8 re_flag;                  //是否接收完成标志1:接收完成0:未完成
}RESEIVE;
extern RESEIVE hop1_re,hop2_re;
extern uint8 sta_hopper1,sta_hopper2;  //hopper状态

extern void SOLEA_OFF(void);

#endif

