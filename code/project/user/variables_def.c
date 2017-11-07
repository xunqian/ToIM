#include "variables_def.h"


//版本号
uint8 MODULE_VERSION[8] = "ToIM001A";
uint8 CPU_VERSION[7] = "V1.0R01";

//序列号
uint8 MODULE_SERIAL[20] = "12345678901234567890";

//看门狗定时复位
uint8   wdt_reseten = 1;
uint16  wdt_resetreg;  

 uint8 sens_selfcheck=0;  //传感器自检标志
 uint8 result=0;
 uint8 detection=0; 		 //检测区是否有卡标志

 uint16 drop_tic_time = 0;	 //通道切换超时时间
 uint16 init_over_time = 0;  //初始化超时
 uint16 over_time = 0;	  //通信超时
 uint32 rotation_over_time = 0;  //电机每两步之间的时间间隔
 uint32 roll_over_time = 0;//电机转动超时1s 
 uint32 sole_open_over_time = 0;//入票口电磁铁打开超时1s 
 uint32 sole_openleave_over_time = 0;//入票口电磁铁打开后离开超时100ms 
 uint32 sole_close_over_time = 0;//入票口电磁铁打开超时1s 
 uint32 sole2_open_over_time = 0;//入票口电磁铁打开超时1s 
 uint32 sole2_close_over_time = 0;//入票口电磁铁打开超时1s 
 uint16 init_delay = 0; 	 //初始化延时，为了等待hopper初始化转动

 
 //模块类型变量
 uint8 Module_Type = 1;  //2:无暂存区模块，1:有暂存区模块
 uint8 antenna = 0; 		 //天线区是否有卡标志
 uint8 antennaA = 0;		  //备票区是否有卡标志
 
 uint8 clear_tic = 0;		 //初始化与通道清理清除的票数量
//Hopper出票选择
uint8  hopper=0;
//hopper状态
uint8 hopper1_state,hopper2state;


//通道位置
uint8 current_gallery;
 //心跳灯控制  1表示启动心跳灯，频率1HZ   ；2心跳灯加速，有故障
 //0停止跳动
 uint8 normal_start = 1; 
 //退币口指示灯标志
 uint8 LED_EN=OFF;//模块控制标志
 //退币口指示灯闪烁时间计数
 uint16 flash_cnt=0; 
 //回收or退换票卡数量
 uint32 box_num[3]={0,0,0};
 uint8 sum_open_over_time=0;
 
 //通信步骤  
 uint8 communication_step = 1;
 
 //清币数量
 uint8 tic_num1[2],tic_num2[2];
 //清币完成标志
 uint8 tic_flag1=1,tic_flag2=1;

RESEIVE hop1_re,hop2_re;
uint8 sta_hopper1=0,sta_hopper2=0;  //hopper状态
  //uart0,与ECU通信发送接收用变量定义
 //通信缓存
uint8	 receive_ok;//uart0数据接收完成标志  1:成功  0:否
uint8	 inbox[60];//接收数据缓存
 //传感器状态
SENSORS sens;
SENSORS sens_last;  
 
 //RFID超时延时
 uint16 g_cbWaitRespDly = 0;
 //心跳灯计数
 uint16 work_light_cnt=0;
 //测试回收性能使用
 uint32 time_count=0; 
uint8 LED_control_flag;//0:  模块控制1 :上位机控制
uint16 OutGPA;
uint16 OutGPB;
uint16 OutGPC;
uint16 OutGPD;
uint16 OutGPE;
