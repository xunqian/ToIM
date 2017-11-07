/*******************************************************************************
*                 
*                 		       维冠机电
--------------------------------------------------------------------------------
* 工程名: 	代币式发售模块
* 功  能: 回收TOKEN
****************************************************************************/
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
/*  2017-6-21 * xunqian.hu     * Creation of the file                                                                         */
/*             *                 *                                                                                                                    */
/*****************************************************************************/
/*  Target : stm32                                                                                                                 */
/*  Crystal: 72Mhz                                                                                                                              */
/*****************************************************************************/

/*******************************************************************************/

#include "public.h"
#include "usart.h"
#include "systick.h"
#include "time.h" 
#include "main.h"
#include "variables_def.h"
#include "reg.h"
#include "string.h"
#include "RC522.h"
#include "iwdg.h"
#include "flash.h"
#include "Hopper.h"

const u8 TEXT_Buffer[10]={0x01,0x02,0x01,0x02,0x01,0x02,0x01,0x02,0x01,0x02};
	#define SIZE (10)
	uint8_t buff[10];
 

void read_sensorstatus(void);

/*******************************************************************************
* 函 数 名         : delay
* 函数功能		   : 延时函数，delay(6000000)延时约1s
* 输    入         : i
* 输    出         : 无
*******************************************************************************/
void delay(u32 i)
{
	while(i--);
}


/*****************************************************************************/
/* Function Description:                                                     */
/*****************************************************************************/
/*   点亮八段码LED                                                   */
/*                                                                           */
/*****************************************************************************/
/* Parameters:                                                               */
/*****************************************************************************/
/*   number                                                                  */
/*  number对应 (abcdefg dp)，置高点亮LED                                                     */
/*****************************************************************************/
/* Return Values:                                                            */
/*****************************************************************************/
/*   none                                                                    */
/*                                                                           */
/*****************************************************************************/

void Write_LED(uint8 number)
{	//number对应 (abcdefg dp)
	if((number&0x80)==0x80)
		GPIO_WriteBit(LED_A_PORT, LED_A_BIT_NUM, Bit_RESET);
	else
		GPIO_WriteBit(LED_A_PORT, LED_A_BIT_NUM, Bit_SET);
	if((number&0x40)==0x40)
		GPIO_WriteBit(LED_B_PORT, LED_B_BIT_NUM, Bit_RESET);
	else
		GPIO_WriteBit(LED_B_PORT, LED_B_BIT_NUM, Bit_SET);
	if((number&0x20)==0x20)
		GPIO_WriteBit(LED_C_PORT, LED_C_BIT_NUM, Bit_RESET);
	else
		GPIO_WriteBit(LED_C_PORT, LED_C_BIT_NUM, Bit_SET);
	if((number&0x10)==0x10)
		GPIO_WriteBit(LED_D_PORT, LED_D_BIT_NUM, Bit_RESET);
	else
		GPIO_WriteBit(LED_D_PORT, LED_D_BIT_NUM, Bit_SET);
	if((number&0x08)==0x08)
		GPIO_WriteBit(LED_E_PORT, LED_E_BIT_NUM, Bit_RESET);
	else
		GPIO_WriteBit(LED_E_PORT, LED_E_BIT_NUM, Bit_SET);
	if((number&0x04)==0x04)
		GPIO_WriteBit(LED_F_PORT, LED_F_BIT_NUM, Bit_RESET);
	else
		GPIO_WriteBit(LED_F_PORT, LED_F_BIT_NUM, Bit_SET);
	if((number&0x02)==0x02)
		GPIO_WriteBit(LED_G_PORT, LED_G_BIT_NUM, Bit_RESET);
	else
		GPIO_WriteBit(LED_G_PORT, LED_G_BIT_NUM, Bit_SET);	
}
/*****************************************************************************/
/* Function Description:                                                     */
/*****************************************************************************/
/*   点亮八段码LED 显示的0-F及DP点                                                 */
/*                                                                           */
/*****************************************************************************/
/* Parameters:                                                               */
/*****************************************************************************/
/*   number                                                                  */
/*  number对应0-F,	点亮DP,number第五位置1                                                    */
/*****************************************************************************/
/* Return Values:                                                            */
/*****************************************************************************/
/*   none                                                                    */
/*                                                                           */
/*****************************************************************************/

void LED_Display(uint8 number)
{
	if((number&0x10)==0x10)
		GPIO_WriteBit(LED_DP_PORT, LED_DP_BIT_NUM, Bit_RESET);
	else
		GPIO_WriteBit(LED_DP_PORT, LED_DP_BIT_NUM, Bit_SET);
   switch(number&0x0f)
   {
   	case 1://ef		
		Write_LED(0x0c);
		break;
	case 2://abdeg
		Write_LED(0xda);
		break;
	case 3://abcdg
		Write_LED(0xf2);
		break;
	case 4://bcfg
		Write_LED(0x66);
		break;
	case 5://acdfg
		Write_LED(0xb6);
		break;
	case 6://acdefg
		Write_LED(0xbe);
		break;
	case 7://abc
		Write_LED(0xe0);
		break;
	case 8://abcdefg
		Write_LED(0xfe);
		break;
	case 9://abcfg
		Write_LED(0xe6);
		break;
	case 0://abcdef
		Write_LED(0xfc);
		break;
	case 0x0a://cdeg
		Write_LED(0x3a);
		break;
	case 0x0b://cdefg
		Write_LED(0x3e);
		break;
	case 0x0c://deg
		Write_LED(0x1a);
		break;
	case 0x0d://bcdeg
		Write_LED(0x7a);
		break;
	case 0x0e://adefg
		Write_LED(0x9e);
		break;
	case 0x0f://aefg
		Write_LED(0x8e);
		break;
	default:
		Write_LED(0xff);
		break;
   }			
}
/*****************************************************************************/
/* Function Description:                                                     */
/*****************************************************************************/
/*   GPIO initialization                                                     */
/*                                                                           */
/*****************************************************************************/
/* Parameters:                                                               */
/*****************************************************************************/
/*   none                                                                    */
/*                                                                           */
/*****************************************************************************/
/* Return Values:                                                            */
/*****************************************************************************/
/*   none                                                                    */
/*                                                                           */
/*****************************************************************************/
static void vGPIOInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // configure the following GPA IO as input FLOATING IO
    // ------------------------------------------------------------------
    // | 15  14  13  12  11  10  9    8     7    6     5    4      3   2   1        0  |
    // |                                    SENS1                                      |
    // ------------------------------------------------------------------
    GPIO_InitStructure.GPIO_Pin =  GPA_IN_BITMAPS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = (GPIOSpeed_TypeDef)0;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	// configure the following GPA IO as OUTPUT IO
	// ------------------------------------------------------------------
	// | 15  14  13  12  11  10  9	 8	 7	  6 	5	 4		3	2	1		 0	|
	// |									                                       LED_F    LED_G|
	// ------------------------------------------------------------------
	GPIO_InitStructure.GPIO_Pin =  GPA_OUT_BITMAPS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// configure the following GPBIO as input FLOATING IO
	// ------------------------------------------------------------------
	// | 15  14  13  12  11  10  9 8	 7	  6 	  5	 4		3	2	1		 0	|
	// |								 IN6 IN5                                                 |
	// ------------------------------------------------------------------
	GPIO_InitStructure.GPIO_Pin =  GPB_IN_BITMAPS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = (GPIOSpeed_TypeDef)0;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// configure the following GPB IO as OUTPUT  IO
	// ------------------------------------------------------------------
	// | 15  14  13  12  11  10   9	   8	     7	6 	5	 4		3	2	1		 0	|
	// |					    SOLE3	SOLE4 SOLE5 		                                               G3     |
	// ------------------------------------------------------------------
	GPIO_InitStructure.GPIO_Pin =  GPB_OUT_BITMAPS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// configure the following GPC IO as input FLOATING IO
	// ------------------------------------------------------------------
	// | 15  14  13  12  11  10  9	 8	     7	   6 	5	 4		3	2	1		 0	|
	// |					   SENS2 SENS3 SENS4 SENS5                                                     |
	// ------------------------------------------------------------------
	GPIO_InitStructure.GPIO_Pin =  GPC_IN_BITMAPS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = (GPIOSpeed_TypeDef)0;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	// configure the following GPC IO as OUTPUT IO
	// ------------------------------------------------------------------
	// |   15         14           13          12  11  10  9  8  7  6 	5    4      3	         2	          1		   0	      |
	// |LED_B    LED_A   MOTOR_2F2					G2 G1 LED_E   LED_D      LED_DP   LED_C    |
	// ------------------------------------------------------------------
	GPIO_InitStructure.GPIO_Pin =  GPC_OUT_BITMAPS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	// configure the following GPD IO as input FLOATING IO
	// ------------------------------------------------------------------
	// | 15      14        13      12       11       10          9	      8  7	  6    5	4    3 2	1	 0	|
	// |SENS6 SENS7 SENS8 SENS9 SENS10 SENS11	 SENS12	  IN4 IN3	IN2 IN1				|
	// ------------------------------------------------------------------
	GPIO_InitStructure.GPIO_Pin =  GPD_IN_BITMAPS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = (GPIOSpeed_TypeDef)0;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	// configure the following GPD IO as input OUTPUT IO
	// ------------------------------------------------------------------
	// | 15  14  13  12  11  10  9	   8	      7	  6 	5	 4		3	2	1		 0	|
	// |					       SENS_POWER	                                             G4            G5   |
	// ------------------------------------------------------------------
	GPIO_InitStructure.GPIO_Pin =  GPD_OUT_BITMAPS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	// configure the following GPE IO as input FLOATING IO
	// ------------------------------------------------------------------
	// | 15      14   13  12   11  10  9  8  7  6 5 4        3	        2	      1 0	|
	// |KEY2 KEY1		                               V_CHECK1 V_CHECK2      |
	// ------------------------------------------------------------------
	GPIO_InitStructure.GPIO_Pin =  GPE_IN_BITMAPS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = (GPIOSpeed_TypeDef)0;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	// configure the following GPE IO as OUTPUT IO
	// ------------------------------------------------------------------
	// | 15  14  13      12     11    10       9        8       7	           6 	    |  
	// |		   OUT1 OUT2 OUT3 OUT4  OUT5  OUT6 OUT7 MOTOR_2F1   |
	// |      5	           4	        3 2	          1           0	    |
	// |MOTOR_1F2  MOTOR_1F1             SOLE1  SOLE2    |
	// ------------------------------------------------------------------
	GPIO_InitStructure.GPIO_Pin =  GPE_OUT_BITMAPS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

    OutGPA = (u16)(0x0003);//0000 0000 0000 0011
    OutGPB = (u16)(0x0001);//0000 0000 0000 0001   
    OutGPC = (u16)(0xc03f);//1100 0000 0011 1111
    //传感器电源初始化为上电状态
    OutGPD = (u16)(0x0103);//0000 0000 0000 0011
	OutGPE = (u16)(0x0000);//0000 0000 0000 0000
	GPIO_Write(GPIOA, OutGPA);
    GPIO_Write(GPIOB, OutGPB);
    GPIO_Write(GPIOC, OutGPC);
	GPIO_Write(GPIOD, OutGPD);
	GPIO_Write(GPIOE, OutGPE);
}
/*****************************************************************************/
/* Function Description:                   							                                  */
/*****************************************************************************/
/*   Pre-setup hardware registers    						                                         */
/*                                                       								                      */
/*****************************************************************************/
/* Parameters:                                                            							   */
/*****************************************************************************/
/*   none                                                                  								  */
/*                                                                      							        */
/*****************************************************************************/
/* Return Values:                                                   							         */
/*****************************************************************************/
/*   none                                                                							        */
/*                                                                        						      		   */
/*****************************************************************************/
static void prvSetupHardware(void)
{
    /* Start with the clocks in their expected state. */
    RCC_DeInit();

    /* Enable HSE (high speed external clock). */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Wait till HSE is ready. */
    while(RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
    {
    }

    /* 2 wait states required on the flash. */
    *((unsigned long *) 0x40022000) = 0x02;

    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);

    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1);

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* PLLCLK = 8MHz * 9 = 72 MHz. */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

    /* Enable PLL. */
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready. */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source. */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source. */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }

    /* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC
                           | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);

    /* SPI2 Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);


    /* Set the Vector Table base address at 0x08000000 */
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* Configure HCLK clock as SysTick clock source. */
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

    vGPIOInit();
}

/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//校验
//参数*ptr  数据指针
//参数 len  校验数据长度
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          是否成功                                                                                             */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   NULL                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/

uint8 Check_LRC(uint8 *ptr, uint8 len)   
{  
  uint8 lrc=0; 
  while(len--!=0)   
  { 
    lrc ^= (*ptr);
    ptr++;  
  }  
  return(lrc);  
}
/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//电磁铁驱动
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          NULL                                                              	                      */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   NULL                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/
 void SOLEA_ON(void)//备票区电磁铁打开
    {
    	SOLE1_ON;			
    }
 void SOLEA_OFF(void)//备票区电磁铁关闭
    {
      SOLE1_OFF;
   }    
 void SOLEB_ON(void)//天线区电磁铁打开
    {     
        SOLE2_ON;		  
	} 
 void SOLEB_OFF(void)//天线区电磁铁关闭
    {
        SOLE2_OFF;	 
    }
 void SOLEC_ON(void)//回收口电磁铁打开
    {     
        SOLE3_ON;		  
	} 
 void SOLEC_OFF(void)//回收口电磁铁关闭
    {
        SOLE3_OFF;	 
    }
  void SOLED_ON(void)//备用电磁铁1打开
    {     
        SOLE4_ON;		  
	} 
 void SOLED_OFF(void)//备用1电磁铁1关闭
    {
        SOLE4_OFF;	 
    }
 void SOLEE_ON(void)//备用电磁铁2打开
    {     
        SOLE5_ON;		  
	} 
 void SOLEE_OFF(void)//备用电磁铁2关闭
    {
        SOLE5_OFF;	 
    }
 
 /*****************************************************************************/
 /* Function Description:																													   */
 /*****************************************************************************/
 //模块电磁铁打开函数
 //参数time 打开电磁铁时间
 
 /*****************************************************************************/
 /* Parameters: 																																 */
 /*****************************************************************************/
 /* 																																					*/
 /* 																																					*/
 /* 																																					*/
 /*****************************************************************************/
 /* Return Values:			是否成功																							 */
 /*****************************************************************************/
 /* 																																					 */
 /*   NULL																																			 */
 /* 																																					 */
 /*****************************************************************************/
 void SoleB_Open(uint16 time)
 {
	 SOLEB_ON(); 
	 drop_tic_time = time;
	 while(drop_tic_time)
	 {
		 read_sensorstatus();
		 if(antenna ==INEXISTENCE)
			 break;  
	 }
	 if(drop_tic_time==0)
	 {
		 SOLEB_OFF();			 
	 }
		 
 }
  void SoleC_Open(uint16 time)
 {	 
	 drop_tic_time = time;
	 while(drop_tic_time)
	 {
		 read_sensorstatus();
		 if((sens_last.SENSORS_STATUS.checkticks3 ==0x01)&&(sens.SENSORS_STATUS.checkticks3 ==0x00))
		 {		 		
			 break; 
		 }
	 }
	
		 
 }
 /*****************************************************************************/
 /* Function Description:																													   */
 /*****************************************************************************/
 //模块电磁铁打开函数
 //参数time 打开电磁铁时间
 
 /*****************************************************************************/
 /* Parameters: 																																 */
 /*****************************************************************************/
 /* 																																					*/
 /* 																																					*/
 /* 																																					*/
 /*****************************************************************************/
 /* Return Values:			是否成功																							 */
 /*****************************************************************************/
 /* 																																					 */
 /*   NULL																																			 */
 /* 																																					 */
 /*****************************************************************************/
 void SoleA_Open(uint16 time)
 {
	 SOLEA_ON();
	 drop_tic_time = time;
	 while(drop_tic_time)
	 {
		 read_sensorstatus();
		 if(antenna ==EXISTENCE)
			 break;
		 
	 }
 }
 /*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//模块电磁铁打开函数
//参数time 打开电磁铁时间
//参数mode 判断是进入天线还是出天线
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          是否成功                                                                                             */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   NULL                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/
void Sole_Open(uint16 time,uint8 mode)
{
    drop_tic_time = time;
    while(drop_tic_time)
    {
        read_sensorstatus();
        if(antenna == mode)
            break;
    }
} 
 /*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//保持电磁铁驱动函数

/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          是否成功                                                                                             */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   NULL                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/
void KEEP_SOLE_STOP(void)
{
	MOTOR_1F1_RESET;
	MOTOR_1F2_RESET;
}
void KEEP_SOLE_A(void)
{
 	MOTOR_1F1_RESET;
	MOTOR_1F2_SET;
	drop_tic_time = 50;//1秒
	 while(drop_tic_time)
	 {
		 read_sensorstatus();
		 if((sens.SENSORS_STATUS.checkticks7==0x00)&&(sens.SENSORS_STATUS.checkticks8 !=0x00))
			 break;			 
	 }
	 KEEP_SOLE_STOP();		
}
void KEEP_SOLE_B(void)
{
	MOTOR_1F1_SET;
	MOTOR_1F2_RESET;
	//delay_ms(50);
	drop_tic_time = 50;//1秒
 	while(drop_tic_time)
	 {
		 read_sensorstatus();
		 if((sens.SENSORS_STATUS.checkticks7!=0x00)&&(sens.SENSORS_STATUS.checkticks8 ==0x00))
			 break;
	 }
 	KEEP_SOLE_STOP();
}
 /*****************************************************************************/
 /* Function Description:																													   */
 /*****************************************************************************/
 //BIT置1程序
 //参数:b要置位的字节
 //参数:index要置位的序列号
 /*****************************************************************************/
 /* Parameters: 																																 */
 /*****************************************************************************/
 /* 																																					*/
 /* 																																					*/
 /* 																																					*/
 /*****************************************************************************/
 /* Return Values:			置位完的字节																						*/
 /*****************************************************************************/
 /* 																																					 */
 /* 																																   */
 /* 																																					 */
 /*****************************************************************************/
 
 uint8 SetBit(uint8 b, uint8 index)//将第index位设为1
{ 
	 b |=(1 << index);
	 return b;			 
}
 /*****************************************************************************/
 /* Function Description:																													   */
 /*****************************************************************************/
 //BIT置0程序
 //参数:b要置位的字节
 //参数:index要置位的序列号
 /*****************************************************************************/
 /* Parameters: 																																 */
 /*****************************************************************************/
 /* 																																					*/
 /* 																																					*/
 /* 																																					*/
 /*****************************************************************************/
 /* Return Values:			置位完的字节																						*/
 /*****************************************************************************/
 /* 																																					 */
 /* 																																   */
 /* 																																					 */
 /*****************************************************************************/
 
 
 uint8 ResetBit(uint8 b, uint8 index) //将第index位设为0
{
	 b&= ~(1 << index);//置0
	 return b;
}
/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//命令响应函数
//参数cmd  响应的命令字节
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          是否成功                                                                                             */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   NULL                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/
void cmd_act(uint8 cmd)
{    
	USART_SendData(USART1,WXDLE);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==Bit_RESET);	
	USART_SendData(USART1,cmd); 
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==Bit_RESET);	
}
/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//回复上位机数据
//参数*cmd 回复的数据指针
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          是否成功                                                                                             */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   NULL                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/
void cmd_reseive(RETURN_CODE *cmd)
{
    uint8 i,temp,len;
    uint8 *p;
    p = cmd->code;
    len = *p;
    p++;
    temp = Check_LRC(p,len);    
	USART_SendData(USART1,WXDLE);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==Bit_RESET);	
	USART_SendData(USART1,WXSTX);  
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==Bit_RESET);	
    for(i=0;i<len;i++)
    {
        if(*p == WXDLE)
        {
            USART_SendData(USART1,WXDLE);
			while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==Bit_RESET);	
        }
		
		USART_SendData(USART1,*p); 
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==Bit_RESET);	
        p++;
    }
    USART_SendData(USART1,WXDLE);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==Bit_RESET);	
    USART_SendData(USART1,WXETX);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==Bit_RESET);	
	USART_SendData(USART1,temp);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==Bit_RESET);	
}
/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//清狗，读取传感器状态
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          是否成功                                                                                             */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   NULL                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/
void read_sensorstatus(void)
{   
    if(wdt_reseten == 1)
	IWDG_ReloadCounter();	   //1.28s
	sens_last = sens;	//SENSORS联合体变量
	sens.status[0] = GPIO_ReadInputDataBit(SENS1_PORT, SENS1_BIT_NUM);
	sens.status[1] = GPIO_ReadInputDataBit(SENS2_PORT, SENS2_BIT_NUM);
	sens.status[2] = GPIO_ReadInputDataBit(SENS3_PORT, SENS3_BIT_NUM);
	//sens.status[3] = GPIO_ReadInputDataBit(SENS4_PORT, SENS4_BIT_NUM);
	//sens.status[4] = GPIO_ReadInputDataBit(SENS5_PORT, SENS5_BIT_NUM);
	//sens.status[5] = GPIO_ReadInputDataBit(SENS6_PORT, SENS6_BIT_NUM);
	//sens.status[6] = GPIO_ReadInputDataBit(SENS7_PORT, SENS7_BIT_NUM);
	//sens.status[7] = GPIO_ReadInputDataBit(SENS8_PORT, SENS8_BIT_NUM);
	//sens.status[8] = GPIO_ReadInputDataBit(SENS9_PORT, SENS9_BIT_NUM);
	//sens.status[9] = GPIO_ReadInputDataBit(SENS10_PORT, SENS10_BIT_NUM);
	//sens.status[10] = GPIO_ReadInputDataBit(SENS11_PORT, SENS11_BIT_NUM);
	//sens.status[11] = GPIO_ReadInputDataBit(SENS12_PORT, SENS12_BIT_NUM);
	//到位开关
	sens.status[12] = GPIO_ReadInputDataBit(IN1_PORT, IN1_BIT_NUM);
	sens.status[13] = GPIO_ReadInputDataBit(IN2_PORT, IN2_BIT_NUM);
	sens.status[14] = GPIO_ReadInputDataBit(IN3_PORT, IN3_BIT_NUM);
	//sens.status[15] = GPIO_ReadInputDataBit(IN4_PORT, IN4_BIT_NUM);
	//sens.status[16] = GPIO_ReadInputDataBit(IN5_PORT, IN5_BIT_NUM);
	//sens.status[17] = GPIO_ReadInputDataBit(IN6_PORT, IN6_BIT_NUM);
	if(sens.SENSORS_STATUS.checkticks1 ==0x01)
    {	
    	//备票区有票
        antennaA = EXISTENCE;
        antennaA_LED_ON;
    }
	else if(sens.SENSORS_STATUS.checkticks1 ==0x00)
	{
        //备票区无票
        antennaA = INEXISTENCE;
        antennaA_LED_OFF;
    }    
    if(sens.SENSORS_STATUS.checkticks2 ==0x01)
    {
        antenna = EXISTENCE;
        antenna_LED_ON;
    }
	else if(sens.SENSORS_STATUS.checkticks2 ==0x00)
	{        
        antenna = INEXISTENCE;
        antenna_LED_OFF;
    } 
	if((sens_last.SENSORS_STATUS.checkticks3 ==0x01)&&(sens.SENSORS_STATUS.checkticks3 ==0x00))
	{
		clear_tic++;
	}
}
/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//上电断电检测传感器
//
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          NULL                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   NULL                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/
void sensor_self_check(RETURN_CODE *re_code)

{
	sens_selfcheck=1;  //传感器自检标志
	//初始化关闭八段码故障显示，清除上传故障码
	LED_Display(0x00);
	Write_LED(0x00);
	//所有传感器发射端断电为0；上电无遮挡为1；
	SensorP_OFF;
	delay_ms(50);
    read_sensorstatus();     
    SensorP_ON; 
	delay_ms(50);  
	read_sensorstatus();

	if((sens.SENSORS_STATUS.checkticks1 ==0)&&(sens_last.SENSORS_STATUS.checkticks1==1))
    {
		//re_code->MESSAGE.info[1] =ResetBit(re_code->MESSAGE.info[1], 0);
    }
	else
	{
    	//re_code->MESSAGE.info[1] =SetBit(re_code->MESSAGE.info[1], 0);
     	re_code->MESSAGE.err_code = TS_sens_err;
		LED_Display(0x11);
	}
	if((sens.SENSORS_STATUS.checkticks2 ==0)&&(sens_last.SENSORS_STATUS.checkticks2==1))
    {
		//re_code->MESSAGE.info[1] =ResetBit(re_code->MESSAGE.info[1], 1);
    }
	else
	{
    	//re_code->MESSAGE.info[1] =SetBit(re_code->MESSAGE.info[1], 1);
     	re_code->MESSAGE.err_code = Antenna_sens_err;
		LED_Display(0x12);
	}
	if((sens.SENSORS_STATUS.checkticks3 ==0)&&(sens_last.SENSORS_STATUS.checkticks3==1))
    {
		//re_code->MESSAGE.info[1] =ResetBit(re_code->MESSAGE.info[1], 2);
    }
	else
	{
    	//re_code->MESSAGE.info[1] =SetBit(re_code->MESSAGE.info[1], 2);
     	re_code->MESSAGE.err_code = Recy_sens_err;
		LED_Display(0x13);
	}
    
    antenna = INEXISTENCE;    //清空天线区标志
    antennaA= INEXISTENCE;   //清空备票区标志
    sens_selfcheck=0;  //传感器自检标志
}

/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//故障判断
//参数*re_comm 要发送的数据
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          是否成功                                                                                             */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   NULL                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/
void box_status_anto(RETURN_CODE *re_comm)
{   
    if(sens.SENSORS_STATUS.checkticks13 == 0)
        re_comm->MESSAGE.info[0] |= 0x02;        //废票箱到位
    else
        re_comm->MESSAGE.info[0] &= 0xfd;        //废票箱未到位
    if(sens.SENSORS_STATUS.checkticks14 == 0)
        re_comm->MESSAGE.info[0] |= 0x08;        //清票箱到位
    else
        re_comm->MESSAGE.info[0] &= 0xf7;        //清票箱未到位
    if(sens.SENSORS_STATUS.checkticks15 == 0)
        re_comm->MESSAGE.info[0] |= 0x80;        //模块到位
    else
        re_comm->MESSAGE.info[0] &= 0x7f;        //模块未到位
        
    WriteHopper(hopper1,SDP_CMD_STATE_QUERY);//状态查询
    hop1_re.re_data[5] = 0;
    drop_tic_time = READ_MAS;
    while(drop_tic_time)//hop1_re.re_flag == 0)
    {
        read_sensorstatus();
        if((hop1_re.re_data[5] == SDP_ACK_CMD_INTIME )&& (hop1_re.re_data[6] == SDP_CMD_STATE_QUERY )&& (hop1_re.re_data[16] == 0x80))
            break;
    }
    if(drop_tic_time == 0)
    {
        if(sta_hopper1 & 0x10)
            re_comm->MESSAGE.info[0] |= 0x10;        //A空
        else
            re_comm->MESSAGE.info[0] &= 0xef;        //A非空
        if(sta_hopper1 & 0x20)
            re_comm->MESSAGE.info[0] |= 0x01;        //A将空
        else
            re_comm->MESSAGE.info[0] &= 0xfe;        //A非空
    }
    else
    {
        if(hop1_re.re_data[11] & 0x10)
            re_comm->MESSAGE.info[0] |= 0x10;        //A空
        else
            re_comm->MESSAGE.info[0] &= 0xef;        //A非空
        if(hop1_re.re_data[11] & 0x20)
            re_comm->MESSAGE.info[0] |= 0x01;        //A将空
        else
            re_comm->MESSAGE.info[0] &= 0xfe;        //A非空
    }
 //   re_comm->MESSAGE.info[1] &= 0xfd; 
    
    WriteHopper(hopper2,SDP_CMD_STATE_QUERY);
    hop2_re.re_data[5] = 0;
    drop_tic_time = READ_MAS;
    while(drop_tic_time)//
    {
        read_sensorstatus();
        if((hop2_re.re_data[5] == SDP_ACK_CMD_INTIME) && (hop2_re.re_data[6] == SDP_CMD_STATE_QUERY )&& (hop2_re.re_data[16] == 0x80))
            break;
    }
    if(drop_tic_time == 0)
    {
        if(sta_hopper2 & 0x10)
            re_comm->MESSAGE.info[0] |= 0x20;        //B空
        else
            re_comm->MESSAGE.info[0] &= 0xdf;        //B非空
        if(sta_hopper2 & 0x20)
            re_comm->MESSAGE.info[0] |= 0x04;        //B将空
        else
            re_comm->MESSAGE.info[0] &= 0xfb;        //B非空
    }
    else
    {
        if(hop2_re.re_data[11] & 0x10)
            re_comm->MESSAGE.info[0] |= 0x20;        //B空
        else
            re_comm->MESSAGE.info[0] &= 0xdf;        //B非空
        if(hop2_re.re_data[11] & 0x20)
            re_comm->MESSAGE.info[0] |= 0x04;        //B将空
        else
            re_comm->MESSAGE.info[0] &= 0xfb;        //B非空
    }    
    if(sens.SENSORS_STATUS.checkticks3 == 0)
    { 
        re_comm->MESSAGE.info[1] &= 0xfe;        //回收口无票//无错误
    }
    else
    {
        re_comm->MESSAGE.info[1] |= 0x01;        //回收口无票//有错误
    }
    
    if(antenna == EXISTENCE)
        re_comm->MESSAGE.info[0] |= 0x40;        //天线区有卡
    else
        re_comm->MESSAGE.info[0] &= 0xbf;        //天线区无卡
}
/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//上电断电检测传感器
//
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          NULL                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   NULL                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/
void Power_On_Off_Sensor(void)//-------------------------------传感器实时状态反馈
{    
	//先上电读sensor状态再断电读sensor状态
	delay_ms(10);
    read_sensorstatus();
    SensorP_OFF;
    delay_ms(10);
    read_sensorstatus();  
    SensorP_ON;
    delay_ms(10);
}


/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
/*   传感器自检                                                                                                                     */
/*                                                                                                                                                      */
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   *re_code: 返回的传感器自检状态                                                                      */
/*                                                                                                                                                      */
/*****************************************************************************/
/* Return Values:                                                                                                                               */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   NULL                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/
void test_sensorstatus(RETURN_CODE *re_comm)
{	
	sens_selfcheck=1;  //传感器自检标志
	//初始化关闭八段码故障显示，清除上传故障码
	LED_Display(0x00);
	Write_LED(0x00);
	//所有传感器发射端断电为0；上电无遮挡为1；
	Power_On_Off_Sensor();
	if((sens.SENSORS_STATUS.checkticks1 ==0)&&(sens_last.SENSORS_STATUS.checkticks1==1))
    {
		re_comm->MESSAGE.info[0] &= 0xfe; //备票区传感器正常
    }
	else
	{
    	re_comm->MESSAGE.info[0] |= 0x01; //备票区传感器故障
     	re_comm->MESSAGE.err_code = TS_sens_err;
		LED_Display(0x11);
	}
	if((sens.SENSORS_STATUS.checkticks2 ==0)&&(sens_last.SENSORS_STATUS.checkticks2==1))
    {
		re_comm->MESSAGE.info[0] &= 0xfd; //天线区传感器正常
    }
	else
	{
    	re_comm->MESSAGE.info[0] |= 0x02; //天线区传感器故障
     	re_comm->MESSAGE.err_code = Antenna_sens_err;
		LED_Display(0x12);
	}
	if((sens.SENSORS_STATUS.checkticks3 ==0)&&(sens_last.SENSORS_STATUS.checkticks3==1))
    {
		re_comm->MESSAGE.info[0] &= 0xfb; //票卡到废票口传感器正常
    }
	else
	{
    	re_comm->MESSAGE.info[0] |= 0x04; //票卡到废票口传感器故障
     	re_comm->MESSAGE.err_code = Recy_sens_err;
		LED_Display(0x13);
	}	
	re_comm->MESSAGE.info[0] &=0xdf;
  //  read_sensorstatus();//增加读传感器
	antenna = INEXISTENCE;
	antenna_LED_OFF;
	//**read HOPPERA传感器的状态值**//
	hop1_re.re_data[5] = 0;
	hop1_re.re_data[6] = 0;
	hop1_re.re_data[16] = 0;
	WriteHopper(hopper1,SDP_CMD_TEST_SENSOR);//HOPPERA传感器测试指令
	drop_tic_time = 3 * DROP_T2;//时间设置为1000ms
	while(drop_tic_time)
	{
		read_sensorstatus();
		if((hop1_re.re_data[5] == SDP_ACK_CMD_INTIME) && (hop1_re.re_data[6] == SDP_CMD_TEST_SENSOR )&& (hop1_re.re_data[16] == 0x80))
			break;
	}
	hop1_re.re_flag = 0;		   
	if(drop_tic_time != 0)
	{
		if(hop1_re.re_data[7] & 0x01)
			re_comm->MESSAGE.info[1] |= 0x01;		 //A出币口传感器1自检故障
		else
			re_comm->MESSAGE.info[1] &= 0xfe;		 //A出币口传感器1自检正常
		if(hop1_re.re_data[7] & 0x02)
			re_comm->MESSAGE.info[1] |= 0x02;		 //A出币口传感器2自检故障
		else
			re_comm->MESSAGE.info[1] &= 0xfd;		 //A出币口传感器2自检正常
		if(hop1_re.re_data[7] & 0x04)
			re_comm->MESSAGE.info[1] |= 0x04;		 //A电磁铁到位传感器自检故障
		else
			re_comm->MESSAGE.info[1] &= 0xfb;		 //A电磁铁到位传感器自检正常
		if(hop1_re.re_data[11] & 0x10)
			re_comm->MESSAGE.info[1] |= 0x08;		 //A空自检故障
		else
			re_comm->MESSAGE.info[1] &= 0xf7;		 //A空自检正常
		if(hop1_re.re_data[11] & 0x20)
			re_comm->MESSAGE.info[1] |= 0x10;		 //A将空自检故障
		else
			re_comm->MESSAGE.info[1] &= 0xef;		 //A将空自检正常
		  
	 //   re_comm->MESSAGE.info[1]&=0x3f;
	}
	//**read HOPPERB传感器的状态值**//
	hop2_re.re_data[5] = 0;
	hop2_re.re_data[6] = 0;
	hop2_re.re_data[16] = 0;
	WriteHopper(hopper2,SDP_CMD_TEST_SENSOR);//HOPPERB传感器测试指令
	drop_tic_time = 3 * DROP_T2;//时间设置为1000ms;
	while(drop_tic_time)
	{
		read_sensorstatus();
		if((hop2_re.re_data[5] == SDP_ACK_CMD_INTIME) &&(hop2_re.re_data[6] == SDP_CMD_TEST_SENSOR )&& (hop2_re.re_data[16] == 0x80))
			break;
	}
	hop2_re.re_flag = 0;		   
	if(drop_tic_time != 0)
	{
		if(hop2_re.re_data[7] & 0x01)
			re_comm->MESSAGE.info[2] |= 0x01;		 //B出币口传感器1自检故障
		else
			re_comm->MESSAGE.info[2] &= 0xfe;		 //B出币口传感器1自检正常
		if(hop2_re.re_data[7] & 0x02)
			re_comm->MESSAGE.info[2] |= 0x02;		 //B出币口传感器2自检故障
		else
			re_comm->MESSAGE.info[2] &= 0xfd;		 //B出币口传感器2自检正常
		if(hop2_re.re_data[7] & 0x04)
			re_comm->MESSAGE.info[2] |= 0x04;		 //B电磁铁到位传感器自检故障
		else
			re_comm->MESSAGE.info[2] &= 0xfb;		 //B电磁铁到位传感器自检正常
		if(hop2_re.re_data[11] & 0x10)
			re_comm->MESSAGE.info[2] |= 0x08;		 //B空自检故障
		else
			re_comm->MESSAGE.info[2] &= 0xf7;		 //B空自检正常
		if(hop2_re.re_data[11] & 0x20)
			re_comm->MESSAGE.info[2] |= 0x10;		 //B将空自检故障
		else
			re_comm->MESSAGE.info[2] &= 0xef;		 //B将空自检正常
	  //   re_comm->MESSAGE.info[2]&=0x3f;
	}	
	antenna = INEXISTENCE;    //清空天线区标志
    antennaA= INEXISTENCE;   //清空备票区标志
    sens_selfcheck=0;  //传感器自检标志
	
} 
 /*****************************************************************************/
 /* Function Description:																													   */
 /*****************************************************************************/
 //检查状态
 //参数 *re_comm 要发送的数据
 /*****************************************************************************/
 /* Parameters: 																																 */
 /*****************************************************************************/
 /* 																																					*/
 /* 																																					*/
 /* 																																					*/
 /*****************************************************************************/
 /* Return Values:			是否成功																							 */
 /*****************************************************************************/
 /* 																																					 */
 /*   NULL																																			 */
 /* 																																					 */
 /*****************************************************************************/
 void check_sensorstatus(RETURN_CODE *re_comm)//-------------------------------传感器实时状态反馈
 {
	 read_sensorstatus();
	 box_status_anto(re_comm);
 }
 
 /*****************************************************************************/
 /* Function Description:																													   */
 /*****************************************************************************/
 //挖票函数
 //
 /*****************************************************************************/
 /* Parameters: 																																 */
 /*****************************************************************************/
 /* 																																					*/
 /* 																																					*/
 /* 																																					*/
 /*****************************************************************************/
 /* 																							 */
 /*****************************************************************************/
 /* 																																					 */
 /*   NULL																																			 */
 /* 																																					 */
 /*****************************************************************************/
 
 void DIG_Tic(void)
 {
	 if(hopper == hopper1)
	 {
		 if((sta_hopper1 & 0x10) == 0)
		 {
			 //Ahopper未空
			 if(Module_Type == HAVE_TEMPORARY_AREA)
			 {
				 if(antennaA == INEXISTENCE)
				 {
					 //寄存区空
					 antennaA = DIG_TICKET;
					 WriteHopper(hopper1,SDP_CMD_CHG);
				 }
			 }
		 }
		 else
		 {
			 //A空
			 antennaA= EMPTY;
		 }		 
	 }	 
	 if(hopper == hopper2)
	 {	  
		 if((sta_hopper2 & 0x10) == 0)
		 {
			 //Bhopper未空
			 
			 if(Module_Type == HAVE_TEMPORARY_AREA)
			 {
				 if(antennaA == INEXISTENCE)
				 {
					 //寄存区空
					 antennaA = DIG_TICKET;
					 WriteHopper(hopper2,SDP_CMD_CHG);
				 }
			 }
		 }
		 else
		 {
			 //B空
			 antennaA = EMPTY;
		 }
	 }	 
 }
 /*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//工作模式
//参数cmd 当前的命令
//参数*act 返回的数据指针
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/* 出票、送票、回收       票函数                                                                                                                                             */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          是否成功                                                                                             */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   NULL                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/
void normal_working(uint8 cmd,RETURN_CODE *act)
 {
	 
	 switch(cmd)
	 {
		 case CMD_BOX1SCOOP://BOX1送票
		 {
			 uint8 time=0;
			 if(antenna == INEXISTENCE)//天线区无票
			 {				 
				 if(antennaA == EXISTENCE)//备票区有票
				 {
					 if(Module_Type == HAVE_TEMPORARY_AREA)//有备票区
					 {
						 for(time=0;time<3;time++)
						 {							
							 SOLE1_ON;
							 Sole_Open(DCT_T2,EXISTENCE);
							 if(antenna == EXISTENCE) //判断出票成功跳出
							 {									 
								 SOLE1_OFF;
								 break;
							 }
							 SOLE1_OFF;
							 delay(500);							
						 }
					 }
					 else//无配票区
					 {
						 WriteHopper(hopper1,SDP_CMD_CHG);
						 drop_tic_time = DROP_T1;
						 while(drop_tic_time)
						 {
							 read_sensorstatus();
							 if(antennaA == EXISTENCE)
							 {
								  for(time=0;time<3;time++)
								 {							
									 SOLE1_ON;
									 Sole_Open(DCT_T2,EXISTENCE);
									 if(antenna == EXISTENCE) //判断出票成功跳出
									 {									 
										 SOLE1_OFF;
										 break;
									 }
									 SOLE1_OFF;
									 delay(500);							
								 }
							 }									
						 }
					 }
				 }
				 else //备票区没票
			 	{
					 WriteHopper(hopper1,SDP_CMD_CHG);
					 drop_tic_time = DROP_T1;
					 while(drop_tic_time)
					 {
						 read_sensorstatus();
						 if(antennaA == EXISTENCE)
						 {
							  for(time=0;time<3;time++)
							 {							
								 SOLE1_ON;
								 Sole_Open(DCT_T2,EXISTENCE);
								 if(antenna == EXISTENCE) //判断出票成功跳出
								 {									 
									 SOLE1_OFF;
									 break;
								 }
								 SOLE1_OFF;
								 delay(500);							
							 }
						 }									
					 }	
			 	}				 
				 if(antennaA == Hopper1_Outcoin_sens_err)
				 {
					 act->MESSAGE.info[2] = 0;
					 act->MESSAGE.result = 'e';
					 act->MESSAGE.err_code = Hopper1_Outcoin_sens_err;
					 break;
				 }
				 else if(antennaA == Hopper1_Clrcoin_sole_err)
				 {
					 act->MESSAGE.info[2] = 0;
					 act->MESSAGE.result = 'e';
					 act->MESSAGE.err_code = Hopper1_Clrcoin_sole_err;
					 break;
				 }
				 if(Module_Type == HAVE_TEMPORARY_AREA)
				 {
					 drop_tic_time = DROP_T1;
					 if(antennaA == DIG_TICKET)
					 {
						 //如果当前HOPPER还在挖票
						 
						 while(drop_tic_time)
						 {
							 read_sensorstatus();
							 if(antennaA != DIG_TICKET)
								 break;
						 }
					 }
					 if((drop_tic_time == 0 )|| (antennaA != EXISTENCE))
					 {
						 //时间到或者没挖出票报挖票失败
						 act->MESSAGE.info[2] = 0;
						 act->MESSAGE.result = 'e';
						 act->MESSAGE.err_code = AWK_no_card;
					 }
				 }
			  //   if(antennaA == EXISTENCE)
				 {
					 if(sens.SENSORS_STATUS.checkticks2 == 1)
					 {						
						 if(drop_tic_time == 0 || time >= 3)
						 {
							 //如果超时表示挖票失败
							 act->MESSAGE.info[2] = 0;
							 act->MESSAGE.result = 'e';
							 act->MESSAGE.err_code = AWK_no_card;
						 }
						 else
						 {
							 act->MESSAGE.info[2] = 1;
							 act->MESSAGE.result = 's';
							 act->MESSAGE.err_code = com_ok;
						 }
					 }
					 else
					 {						 
						 act->MESSAGE.info[2] = 1;
						 act->MESSAGE.result = 's';
						 act->MESSAGE.err_code = com_ok;
					 }
				 }
			 }
			 else//一上来就有票发出警告
			 {
				 //有票则报警
				 act->MESSAGE.result = 'w';
				 act->MESSAGE.err_code = card_at_RW_area;
				 act->MESSAGE.info[2] = 0;
			 }
			 break;
		 }
		 case CMD_BOX2SCOOP://BOX2送票
		 {
			 uint8 time=0;
			 if(antenna == INEXISTENCE)//天线区无票
			 {				 
				 if(antennaA == EXISTENCE)//备票区有票
				 {
					 if(Module_Type == HAVE_TEMPORARY_AREA)//有备票区
					 {
						 for(time=0;time<3;time++)
						 {							
							 SOLE1_ON;
							 Sole_Open(DCT_T2,EXISTENCE);
							 if(antenna == EXISTENCE) //判断出票成功跳出
							 {									 
								 SOLE1_OFF;
								 break;
							 }
							 SOLE1_OFF;
							 delay(500);							
						 }
					 }
					 else//无配票区
					 {
						 WriteHopper(hopper2,SDP_CMD_CHG);
						 drop_tic_time = DROP_T1;
						 while(drop_tic_time)
						 {
							 read_sensorstatus();
							 if(antennaA == EXISTENCE)
							 {
								  for(time=0;time<3;time++)
								 {							
									 SOLE1_ON;
									 Sole_Open(DCT_T2,EXISTENCE);
									 if(antenna == EXISTENCE) //判断出票成功跳出
									 {									 
										 SOLE1_OFF;
										 break;
									 }
									 SOLE1_OFF;
									 delay(500);							
								 }
							 }									
						 }
					 }
				 }
				 else //备票区没票
			 	{
					 WriteHopper(hopper2,SDP_CMD_CHG);
					 drop_tic_time = DROP_T1;
					 while(drop_tic_time)
					 {
						 read_sensorstatus();
						 if(antennaA == EXISTENCE)
						 {
							  for(time=0;time<3;time++)
							 {							
								 SOLE1_ON;
								 Sole_Open(DCT_T2,EXISTENCE);
								 if(antenna == EXISTENCE) //判断出票成功跳出
								 {									 
									 SOLE1_OFF;
									 break;
								 }
								 SOLE1_OFF;
								 delay(500);							
							 }
						 }									
					 }	
			 	}				 
				 if(antennaA == Hopper2_Outcoin_sens_err)
				 {
					 act->MESSAGE.info[2] = 0;
					 act->MESSAGE.result = 'e';
					 act->MESSAGE.err_code = Hopper2_Outcoin_sens_err;
					 break;
				 }
				 else if(antennaA == Hopper2_Clrcoin_sole_err)
				 {
					 act->MESSAGE.info[2] = 0;
					 act->MESSAGE.result = 'e';
					 act->MESSAGE.err_code = Hopper2_Clrcoin_sole_err;
					 break;
				 }
				 if(Module_Type == HAVE_TEMPORARY_AREA)
				 {
					 drop_tic_time = DROP_T1;
					 if(antennaA == DIG_TICKET)
					 {
						 //如果当前HOPPER还在挖票
						 
						 while(drop_tic_time)
						 {
							 read_sensorstatus();
							 if(antennaA != DIG_TICKET)
								 break;
						 }
					 }
					 if((drop_tic_time == 0 )|| (antennaA != EXISTENCE))
					 {
						 //时间到或者没挖出票报挖票失败
						 act->MESSAGE.info[2] = 0;
						 act->MESSAGE.result = 'e';
						 act->MESSAGE.err_code = AWK_no_card;
					 }
				 }
			  //   if(antennaA == EXISTENCE)
				 {
					 if(sens.SENSORS_STATUS.checkticks2 == 1)
					 {						
						 if(drop_tic_time == 0 || time >= 3)
						 {
							 //如果超时表示挖票失败
							 act->MESSAGE.info[2] = 0;
							 act->MESSAGE.result = 'e';
							 act->MESSAGE.err_code = AWK_no_card;
						 }
						 else
						 {
							 act->MESSAGE.info[2] = 1;
							 act->MESSAGE.result = 's';
							 act->MESSAGE.err_code = com_ok;
						 }
					 }
					 else
					 {					
						 act->MESSAGE.info[2] = 1;
						 act->MESSAGE.result = 's';
						 act->MESSAGE.err_code = com_ok;
					 }
				 }
			 }
			 else//一上来就有票发出警告
			 {
				 //有票则报警
				 act->MESSAGE.result = 'w';
				 act->MESSAGE.err_code = card_at_RW_area;
				 act->MESSAGE.info[2] = 0;
			 }
			 break;
		 }
		 case CMD_TICKETSELL://卖票
		 {
			 uint8 time=3;
			 //HOPPER1_LED_OFF;
			 //HOPPER2_LED_OFF;
			 if(antenna == EXISTENCE)
			 {
				 //天线区有票					
				 while(time--)
				 {
					 SOLE2_ON;
					 Sole_Open(DROP_T2,INEXISTENCE);
					 SOLE2_OFF;
					 if(drop_tic_time == 0 && antenna == EXISTENCE)
					 {
						 delay(500);
						 continue;
					 }
					 else
					 {
					 	break;
					 }
				 }
				 if(antenna == INEXISTENCE)
				 {
					 act->MESSAGE.info[2] = 1;
					 act->MESSAGE.result = 's';
					 act->MESSAGE.err_code = com_ok;
				 }
				 else
				 {
					 act->MESSAGE.info[2] = 0;
					 act->MESSAGE.result = 'e';
					 act->MESSAGE.err_code = card_block_at_exit_area;
				 }			 
				 
			 }
			 else
			 {
				 //无票则告警
				 act->MESSAGE.result = 'w';
				 act->MESSAGE.err_code = no_card_at_RW_area;
				 act->MESSAGE.info[2] = 0;
			 }
		   //  delay(10);
			 break;
		 }
		 case CMD_TICKETRECYCLE://回收
		 {
			 uint8 time=3;
			 //HOPPER1_LED_OFF;
			 //HOPPER2_LED_OFF;
			 if(antenna == EXISTENCE)
			 {
				 //天线区有票					 
				 while(time--)
				 {
					 SOLE3_ON;
					 SOLE2_ON;
					 SoleC_Open(DROP_T3);					 
					 SOLE2_OFF;
					 SOLE3_OFF;					 
					 if(drop_tic_time == 0 && antenna == EXISTENCE)
					 {
						 delay(500);
						 continue;
					 }
					 else
				 	 {
						break;
				 	 }
				 }
				 if(antenna == INEXISTENCE)
				 {
					 act->MESSAGE.info[2] = 1;
					 act->MESSAGE.result = 's';
					 act->MESSAGE.err_code = com_ok;
				 }
				 else
				 {
					 act->MESSAGE.info[2] = 0;
					 act->MESSAGE.result = 'e';
					 act->MESSAGE.err_code = card_block_at_exit_area;
				 }
				 
				 
			 }
			 else
			 {
				 //无票则告警
				 act->MESSAGE.result = 'w';
				 act->MESSAGE.err_code = no_card_at_RW_area;
				 act->MESSAGE.info[2] = 0;
			 }
			// delay(10);
			 break;
		 }
			 
	 }
 }


 
/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//模块初始化
//参数*re_comm 回复的数据指针
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          是否成功                                                                                             */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   NULL                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/
void module_init(RETURN_CODE *re_comm)
{
    uint8 i;
	hopper = 0;
    read_sensorstatus();
    antenna_LED_OFF;
    //HOPPER1_LED_OFF;
    //HOPPER2_LED_OFF;
    //SENSOR_LED_OFF;
    antenna = INEXISTENCE;       //清空天线区标志
    clear_tic = 0;              //清空初始化计数数值
    hop1_re.re_data[5] = 0;
    hop2_re.re_data[5] = 0;
  	//Hopper自检
    WriteHopper(hopper1,SDP_CMD_TEST_SENSOR);//自检命令
    WriteHopper(hopper2,SDP_CMD_TEST_SENSOR);    
    //通道清理和检测电磁铁1、3
    SOLE3_ON;
	SOLE1_ON;	
	Sole_Open(DCT_T3,NO_STA);//时间需调试
	SOLE1_OFF;
	SOLE3_OFF;
	re_comm->MESSAGE.info[2] = clear_tic;  
    //检测电磁铁2
    SOLE2_ON;
	delay_ms(100);//时间需调试
	SOLE2_OFF;
	//传感器自检      
    sensor_self_check(re_comm); 
	//获取及分析Hopper自检信息
    if(Module_Type == NO_TEMPORARY_AREA)
    {
        init_delay = SECOND*3;
        while(init_delay)
        {
            read_sensorstatus();
            if((hop1_re.re_data[5] == SDP_ACK_CMD_INTIME)&& (hop1_re.re_data[6] == SDP_CMD_TEST_SENSOR)
                &&( hop2_re.re_data[5] == SDP_ACK_CMD_INTIME )&& (hop2_re.re_data[6] == SDP_CMD_TEST_SENSOR))
            {
                break;
            }
        }
    }
    if((hop1_re.re_data[5] == SDP_ACK_CMD_INTIME) && (hop1_re.re_data[6] == SDP_CMD_TEST_SENSOR))
    {
        if(hop1_re.re_data[7] & 0x03 == 0x03)
        {
            re_comm->MESSAGE.err_code = Hopper1_Outcoin_sens_err;
            antennaA = Hopper1_Outcoin_sens_err;
        }
        if(hop1_re.re_data[7] & 0x04 != 0)
        {
            re_comm->MESSAGE.err_code = Hopper1_Clrcoin_sole_err;
            antennaA = Hopper1_Clrcoin_sole_err;
        }
    }
    if((hop2_re.re_data[5] == SDP_ACK_CMD_INTIME )&& (hop2_re.re_data[6] == SDP_CMD_TEST_SENSOR))
    {
        if(hop2_re.re_data[7] & 0x03 == 0x03)
        {
            re_comm->MESSAGE.err_code = Hopper2_Outcoin_sens_err;
            antennaA = Hopper2_Outcoin_sens_err;
        }
        if(hop2_re.re_data[7] & 0x04 != 0)
        {
            re_comm->MESSAGE.err_code = Hopper2_Clrcoin_sole_err;
            antennaA = Hopper2_Clrcoin_sole_err;
        }
    }
    for(i=0;i<20;i++)
    {
        hop1_re.re_data[i] = 0;
        hop2_re.re_data[i] = 0;
    }
    hop1_re.re_flag = 0;
    hop2_re.re_flag = 0;	
    read_sensorstatus();    
    antenna_LED_OFF;
    //在出币前先判断HOPPER是否为空
    WriteHopper(hopper1,SDP_CMD_STATE_QUERY);
    WriteHopper(hopper2,SDP_CMD_STATE_QUERY);
    drop_tic_time = DCT_T2;
    while(drop_tic_time);
    if((hop1_re.re_data[5] == SDP_ACK_CMD_INTIME) && (hop1_re.re_data[6] == SDP_CMD_STATE_QUERY )&& (hop1_re.re_data[16] == 0x80))
    {
        sta_hopper1 = hop1_re.re_data[11];
        if((hop1_re.re_data[11] & 0x10) == 0)
        {
            //非空挖票
           // WriteHopper(hopper1,SDP_CMD_CHG);
       //     sta_hopper1 &= 0xef;
        }
        else
            antennaA = EMPTY;
        re_comm->MESSAGE.info[1] &= 0xfd;
    }
    else
    {
        re_comm->MESSAGE.err_code = rf232A_err;
        re_comm->MESSAGE.info[1] |= 0x2;
    }
    if((hop2_re.re_data[5] == SDP_ACK_CMD_INTIME) && (hop2_re.re_data[6] == SDP_CMD_STATE_QUERY) && (hop2_re.re_data[16] == 0x80))
    {
        sta_hopper2 = hop2_re.re_data[11];
        if((hop2_re.re_data[11] & 0x10) == 0)
        {
            //非空挖票
         //   WriteHopper(hopper2,SDP_CMD_CHG);
          //  sta_hopper2 &= 0xef;
        }
        else
            antennaA = EMPTY;
        re_comm->MESSAGE.info[1] &= 0xfb;
    }
    else
    {
        re_comm->MESSAGE.err_code = rf232B_err;
        re_comm->MESSAGE.info[1] |= 0x4;
    }
    antenna = INEXISTENCE;
} 
/*****************************************************************************/
/* Function Description:                                                                                               */
/*****************************************************************************/
//RFID读写器通道选择
/*****************************************************************************/
/* Parameters:                                                                                                            */
/*****************************************************************************/
/*   Galleryy: 通道选择                                                                                                                          */
/*                                                                                                                              */
/*                                                                                                                            */
/*****************************************************************************/
/* Return Values:                                                                                                      */
/*****************************************************************************/
/*                                                                                                                              */
/*  无                                                                                                                      */
/*                                                                                                                              */
/*****************************************************************************/

void RFID_Gallery_select(uint8 Gallery)
{
	switch(Gallery)
	{
		case 5:
			RFID_ANTENNA1_ON;
			RFID_ANTENNA2_OFF;
			RFID_ANTENNA3_OFF;
			RFID_ANTENNA4_OFF;
			RFID_ANTENNA5_OFF;

		break;
		case 3:
			RFID_ANTENNA1_OFF;
			RFID_ANTENNA2_ON;
			RFID_ANTENNA3_OFF;
			RFID_ANTENNA4_OFF;
			RFID_ANTENNA5_OFF;
		break;
		case 4:
			RFID_ANTENNA1_OFF;
			RFID_ANTENNA2_OFF;
			RFID_ANTENNA3_ON;
			RFID_ANTENNA4_OFF;
			RFID_ANTENNA5_OFF;
		break;
		case 2:
			RFID_ANTENNA1_OFF;
			RFID_ANTENNA2_OFF;
			RFID_ANTENNA3_OFF;
			RFID_ANTENNA4_ON;
			RFID_ANTENNA5_OFF;

		break;
		case 1:
			RFID_ANTENNA1_OFF;
			RFID_ANTENNA2_OFF;
			RFID_ANTENNA3_OFF;
			RFID_ANTENNA4_OFF;
			RFID_ANTENNA5_ON;
		break;
		default:
		break;

	}
}

/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//读取票箱电子芯片块数据
//参数box_no 票箱编号取值范围为:2,3,4
//参数block_no块地址取值范围为:8,9,10\12,13,14\等，详见S50等IC卡手册
//参数*pDATA返回的电子芯片块数据指针
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                           */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          是否成功                                                                                             */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   0x00:OK  其他错误                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/

uint8 ReadRFIDBLOCK(uint8 box_no,uint8 block_no,uint8 *pDATA)
{
	unsigned char status=0xfe;
	unsigned char g_ucTempbuf[30];	
	unsigned char DefaultKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
	RFID_Gallery_select(box_no-2);
	status = PcdRequest(PICC_REQALL, g_ucTempbuf);//寻卡
	if (status != MI_OK)
	{
		//寻卡超时时间设为0.5s
		g_cbWaitRespDly=25;
		while(g_cbWaitRespDly!=0)
		{
			read_sensorstatus();
			PcdReset();
			PcdAntennaOff(); 
			PcdAntennaOn(); 
			status = PcdRequest(PICC_REQALL, g_ucTempbuf);//寻卡
			if(status==MI_OK)
				break;
		}		
	}
	//超时时间设为0.5s
	g_cbWaitRespDly=25;	
	do{
			read_sensorstatus();
		status = PcdAnticoll(g_ucTempbuf);//防碰撞
			if(status == MI_OK)
				break;
		}while(g_cbWaitRespDly!=0);		 
	//超时时间设为0.5s
	g_cbWaitRespDly=25;	
	do{
			status = PcdSelect(g_ucTempbuf);//选定卡片
			if(status == MI_OK)
				break;
		}while(g_cbWaitRespDly!=0);	
	//超时时间设为0.5s
	g_cbWaitRespDly=25;	
	do{
			read_sensorstatus();
		//验证卡片密码
			status = PcdAuthState(PICC_AUTHENT1A, block_no, DefaultKey, g_ucTempbuf);
			if(status == MI_OK)
				break;
		}while(g_cbWaitRespDly!=0);
	g_cbWaitRespDly=50;	
	do{			
			read_sensorstatus();
		status = PcdRead(block_no, pDATA);//读块
			if(status == MI_OK)
				break;
		}while(g_cbWaitRespDly!=0); 

	return status;
}
/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//写入票箱电子芯片块数据
//参数box_no 票箱编号取值范围为:2,3,4
//参数block_no块地址取值范围为:8,9,10\12,13,14\等，详见S50等IC卡手册
//参数*pDATA写入电子芯片块数据指针
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                           */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          是否成功                                                                                             */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   0x00:OK  其他错误                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/

uint8 WriteRFIDBLOCK(uint8 box_no,uint8 block_no,uint8 *pDATA)


{
	unsigned char status=0xfe;
	unsigned char g_ucTempbuf[30];	
	unsigned char DefaultKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
	RFID_Gallery_select(box_no-2);
	status = PcdRequest(PICC_REQALL, g_ucTempbuf);//寻卡
	if (status != MI_OK)
	{
		//寻卡超时时间设为0.5s
		g_cbWaitRespDly=25;
		while(g_cbWaitRespDly!=0)
		{
			read_sensorstatus();
			PcdReset();
			PcdAntennaOff(); 
			PcdAntennaOn(); 
			status = PcdRequest(PICC_REQALL, g_ucTempbuf);//寻卡
			if(status==MI_OK)
				break;
		}		
	}
	//超时时间设为0.5s
	g_cbWaitRespDly=25;	
	do{
		read_sensorstatus();
		status = PcdAnticoll(g_ucTempbuf);//防碰撞
			if(status == MI_OK)
				break;
		}while(g_cbWaitRespDly!=0);		 
	//超时时间设为0.5s
	g_cbWaitRespDly=25;	
	do{
			read_sensorstatus();
		status = PcdSelect(g_ucTempbuf);//选定卡片
			if(status == MI_OK)
				break;
		}while(g_cbWaitRespDly!=0);	
	//超时时间设为0.5s
	g_cbWaitRespDly=25;	
	do{
			read_sensorstatus();
		//验证卡片密码
			status = PcdAuthState(PICC_AUTHENT1A, block_no, DefaultKey, g_ucTempbuf);
			if(status == MI_OK)
				break;
		}while(g_cbWaitRespDly!=0);		 
	//超时时间设为1s
	g_cbWaitRespDly=50;	
	do{
			read_sensorstatus();
		status = PcdWrite(block_no,pDATA);//写块
			if(status == MI_OK)
				break;
		}while(g_cbWaitRespDly!=0); 
	return status;
}
/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//读取票箱电子芯片物理编号
//参数box_no 票箱编号取值范围为:2,3,4
//参数*pDATA返回的电子芯片块数据指针
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                           */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          是否成功                                                                                             */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   0x00:OK  其他错误                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/

uint8 ReadRFID_Serial_Number(uint8 box_no,uint8 *pDATA)
{
	unsigned char status=0xfe;
	//unsigned char g_ucTempbuf[30];	
	RFID_Gallery_select(box_no-2);
	status = PcdRequest(PICC_REQALL,pDATA);//寻卡
	if (status != MI_OK)
	{
		//寻卡超时时间设为0.5s
		g_cbWaitRespDly=25;
		while(g_cbWaitRespDly!=0)
		{
			PcdReset();
			PcdAntennaOff(); 
			PcdAntennaOn(); 
			status = PcdRequest(PICC_REQALL,pDATA);//寻卡
			if(status==MI_OK)
				break;
		}		
	}
	//超时时间设为0.5s
	g_cbWaitRespDly=25; 
	do{
			status = PcdAnticoll(pDATA);//防碰撞
			if(status == MI_OK)
				break;

		}while(g_cbWaitRespDly!=0); 	 
	//超时时间设为0.5s
	g_cbWaitRespDly=25; 
	do{
			status = PcdSelect(pDATA);//选定卡片
			if(status == MI_OK)
				break;

		}while(g_cbWaitRespDly!=0);
	return status;
}
/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//通信命令解析
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          是否成功                                                                                             */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   NULL                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/
void check_command(void)//检查命令
	{
		static RETURN_CODE re_code;
		uint8 *p;
		uint8 i=0;
		memset(re_code.code,0,53);
		if(1 == receive_ok)
		{
			communication_step = 1;
			switch(inbox[0])
			{
				case 0x0:			//重新回复上次命令
				{
					cmd_reseive(&re_code);
					break;
				}
				case 0x81:			//初始化	
				{
					re_code.MESSAGE.act_code = inbox[0];
					re_code.MESSAGE.err_code = com_ok;
					module_init(&re_code);
					check_sensorstatus(&re_code);
					if(re_code.MESSAGE.err_code != com_ok)
					{
						re_code.MESSAGE.result = 'e';
						normal_start = 2;
					}
					else
					{
						re_code.MESSAGE.result = 's';
						normal_start = 1;
					}
					re_code.MESSAGE.len = 6;
					cmd_reseive(&re_code);
					break;
				}
				case 0x82:			//读模块状态
				{
					re_code.MESSAGE.act_code = inbox[0];
					check_sensorstatus(&re_code);
					re_code.MESSAGE.result = 's';
					re_code.MESSAGE.err_code = com_ok;
					re_code.MESSAGE.len = 5;
					cmd_reseive(&re_code);
					break;
				}
				case 0x83:			//通道清理
				{
					re_code.MESSAGE.act_code = inbox[0];
					re_code.MESSAGE.err_code = com_ok;
					module_init(&re_code);
					check_sensorstatus(&re_code);
					if(re_code.MESSAGE.err_code != com_ok)
					{
						re_code.MESSAGE.result = 'e';
						normal_start = 2;
					}
					else
					{
						re_code.MESSAGE.result = 's';
						normal_start = 1;
					}
					re_code.MESSAGE.len = 6;
					cmd_reseive(&re_code);
					break;
				}
				case 0x84:			//挖票至天线区
				{
					re_code.MESSAGE.act_code = inbox[0];
					if(1==inbox[1]||2==inbox[1])
					{
						if(1==inbox[1])
						{
							hopper = hopper1;
						}
						if(2==inbox[1])
						{
							hopper = hopper2;
						}
						normal_working(inbox[1],&re_code);						
						check_sensorstatus(&re_code);
						if(re_code.MESSAGE.result == 's')
						{
							re_code.MESSAGE.err_code = com_ok;
							normal_start = 1;
						}
						else
							normal_start = 2;
					}
					else
					{
						re_code.MESSAGE.result = 'e';
						re_code.MESSAGE.err_code =invalid_parameter;
						normal_start = 2;
					} 
					re_code.MESSAGE.len = 6;
					cmd_reseive(&re_code);
					DIG_Tic();
					break;
				}
				case 0x85:			//售票
				{
					re_code.MESSAGE.act_code = inbox[0];
					normal_working(CMD_TICKETSELL,&re_code);
					check_sensorstatus(&re_code);
					if(re_code.MESSAGE.result == 's')
					{
						re_code.MESSAGE.err_code = com_ok;
						normal_start = 1;
					}
					else
						normal_start = 2;
					re_code.MESSAGE.len = 6;
					cmd_reseive(&re_code);
					break;
				}
				case 0x86:			//回收
				{
					re_code.MESSAGE.act_code = inbox[0];
					normal_working(CMD_TICKETRECYCLE,&re_code);
					check_sensorstatus(&re_code);
					if(re_code.MESSAGE.result == 's')
					{
						re_code.MESSAGE.err_code = com_ok;
						normal_start = 1;
					}
					else
						normal_start = 2;
					re_code.MESSAGE.len = 6;
					cmd_reseive(&re_code);
					break;
				}				
				case 0x88:			//版本号
				{
					p = re_code.MESSAGE.info;
					re_code.MESSAGE.act_code = inbox[0];
					re_code.MESSAGE.result = 's';
					re_code.MESSAGE.err_code = com_ok;
					for(i=0;i<sizeof(MODULE_VERSION);i++)
					{
						*p = MODULE_VERSION[i];
						p++;
					}
					for(i=0;i<sizeof(CPU_VERSION);i++)
					{
						*p = CPU_VERSION[i];
						p++;
					}
					re_code.MESSAGE.len = 18;
					cmd_reseive(&re_code);
					break;
				}
				case 0x89:			//清除token卡
				{
					re_code.MESSAGE.act_code = inbox[0];
					hop1_re.re_data[5] = 0; 	//	必须要清掉，否则下次在清币的时候读数量不会显示设备忙
					hop2_re.re_data[5] = 0;
					if(1==inbox[1]||2==inbox[1])
					{
						WriteHopper(inbox[1],SDP_CMD_CLR);
						if(inbox[1] == hopper1)
						{
							tic_flag1 = 0;
							delay(100);
							if(((hop1_re.re_data[5] == SDP_ACK_CMD_ACK) && (hop1_re.re_data[6] == SDP_CMD_CLR))
								|| ((hop1_re.re_data[5] == SDP_ACK_CMD_UNTIME) && (hop1_re.re_data[6] == SDP_CMD_CLR)))//清空下发命令
							{
								re_code.MESSAGE.result = 's';
								re_code.MESSAGE.err_code = com_ok;
								normal_start = 1;
							}
							else
							{
								re_code.MESSAGE.result = 'e';
								re_code.MESSAGE.err_code = clr_err;
								normal_start = 2;
							}
						}
						else
						{
							tic_flag2 = 0;
							delay(100);
							if((hop2_re.re_data[5] == SDP_ACK_CMD_ACK && hop2_re.re_data[6] == SDP_CMD_CLR)
								|| (hop2_re.re_data[5] == SDP_ACK_CMD_UNTIME && hop2_re.re_data[6] == SDP_CMD_CLR))
							{
								re_code.MESSAGE.result = 's';
								re_code.MESSAGE.err_code = com_ok;
								normal_start = 1;
							}
							else
							{
								re_code.MESSAGE.result = 'e';
								re_code.MESSAGE.err_code = clr_err;
								normal_start = 2;
							}
						}
					}
					else
					{
						re_code.MESSAGE.result = 'e';
						re_code.MESSAGE.err_code =invalid_parameter;
						normal_start = 1;
					}
					//check_sensorstatus(&re_code);
					re_code.MESSAGE.len = 3;
					cmd_reseive(&re_code);
					break;
				}
				case 0x8a:			//读取清空数量
				{	
					re_code.MESSAGE.act_code = inbox[0];
					if(1==inbox[1]||2==inbox[1])
					{
						if(inbox[1] == hopper1)
						{
							if(tic_flag1 == 0)
							{
								re_code.MESSAGE.result = 'e';
								re_code.MESSAGE.err_code = equ_busy;
							}
							else
							{
								re_code.MESSAGE.result = 's';
								re_code.MESSAGE.err_code = com_ok;
								re_code.MESSAGE.info[0] = tic_num1[0];
								re_code.MESSAGE.info[1] = tic_num1[1];
							}
						}
						else
						{
							if(tic_flag2 == 0)
							{
								re_code.MESSAGE.result = 'e';
								re_code.MESSAGE.err_code = equ_busy;
							}
							else
							{
								re_code.MESSAGE.result = 's';
								re_code.MESSAGE.err_code = com_ok;
								re_code.MESSAGE.info[0] = tic_num2[0];
								re_code.MESSAGE.info[1] = tic_num2[1];
							}
						}
					}
					else
					{
						re_code.MESSAGE.result = 'e';
						re_code.MESSAGE.err_code =invalid_parameter;
						normal_start = 2;
					}
					//check_sensorstatus(&re_code);
					re_code.MESSAGE.len = 5;
					cmd_reseive(&re_code);
					break;
				}
				case 0x8b:			//停止清除token卡
				{	
					re_code.MESSAGE.act_code = inbox[0];
					if(1==inbox[1]||2==inbox[1])
					{  
						WriteHopper(inbox[1],SDP_CMD_STOP_CLR);
						re_code.MESSAGE.result = 's';
						re_code.MESSAGE.err_code = com_ok;
						normal_start = 1;
					}
					else
					{
						re_code.MESSAGE.result = 'e';
						re_code.MESSAGE.err_code =invalid_parameter;
						normal_start = 2;
					}
					re_code.MESSAGE.len = 3;
					cmd_reseive(&re_code);
					break;
				}
			
				case 0x97:			//设置卡箱的票卡数量
				{
				   /* re_code.MESSAGE.act_code = inbox[0];
					if((inbox[1] != 0xff) && (inbox[2] != 0xff))
						box1_tick_num = inbox[1] + (inbox[2] << 8);
					if((inbox[3] != 0xff) && (inbox[4] != 0xff))
						box2_tick_num = inbox[3] + (inbox[4] << 8);
					if((inbox[5] != 0xff) && (inbox[6] != 0xff))
						box3_tick_num = inbox[5] + (inbox[6] << 8);
					re_code.MESSAGE.result = 's';
					re_code.MESSAGE.err_code = com_ok;
					re_code.MESSAGE.len = 3;
					cmd_reseive(&re_code);
					//inbox[0] = 0;*/
					break;
				}
				case 0x98:			//更换票箱//暂时不用管
				{
					break;
				}
				case 0x99:				//读取票箱编号
				{
					re_code.MESSAGE.act_code = inbox[0];
					if(3==inbox[1]||4==inbox[1]||5==inbox[1]||6==inbox[1])
					{
						re_code.MESSAGE.err_code=ReadRFIDBLOCK(inbox[1],1,re_code.MESSAGE.info);	//读取票箱的票箱编号（块1）
						if(re_code.MESSAGE.err_code == com_ok)
						{
							re_code.MESSAGE.result = 's';
							normal_start = 1;
						}
						else
						{
							re_code.MESSAGE.result = 'e';
							normal_start = 2;
						}
					}
					else
					{
						re_code.MESSAGE.result = 'e';
						re_code.MESSAGE.err_code =invalid_parameter;
						normal_start = 2;
					}
					re_code.MESSAGE.len = 17;
					cmd_reseive(&re_code);					
					break;
				}
				case 0xa0:			//设置模块类型
				{
					re_code.MESSAGE.act_code = inbox[0];
					re_code.MESSAGE.result = 's';
					re_code.MESSAGE.err_code = com_ok;
					//eeprom_write(E2TYPE_ADDR,inbox[1]);
					Module_Type = inbox[1];
					re_code.MESSAGE.len = 3;
					cmd_reseive(&re_code);
					break;
				}
			/*	case 0xa1:			//读取模块类型
				{
					re_code.MESSAGE.act_code = inbox[0];
					re_code.MESSAGE.result = 's';
					re_code.MESSAGE.err_code = com_ok;
					re_code.MESSAGE.info[0] = Module_Type;
					re_code.MESSAGE.len = 4;
					cmd_reseive(&re_code);
					break;
				}*/
				case 0xe3:				//向 RFID 写一个 Block 数据
				{
					re_code.MESSAGE.act_code = inbox[0];
					if(3==inbox[1]||4==inbox[1]||5==inbox[1]||6==inbox[1])
					{
						re_code.MESSAGE.err_code = WriteRFIDBLOCK(inbox[1],inbox[2],&inbox[3]);
						if(re_code.MESSAGE.err_code == com_ok)
						{
							re_code.MESSAGE.result = 's';
							normal_start = 1;
						}
						else
						{
							re_code.MESSAGE.result = 'e';
							normal_start = 2;
						}
					}
					else
					{
						re_code.MESSAGE.result = 'e';
						re_code.MESSAGE.err_code =invalid_parameter;
						normal_start = 2;
					}
					re_code.MESSAGE.len = 3;
					cmd_reseive(&re_code);					
					break;
				}
				case 0xe4:				//从RFID中读取一个Block数据
				{
					re_code.MESSAGE.act_code = inbox[0];
					if(3==inbox[1]||4==inbox[1]||5==inbox[1]||6==inbox[1])
					{	   
						re_code.MESSAGE.err_code = ReadRFIDBLOCK(inbox[1],inbox[2],re_code.MESSAGE.info);
						if(re_code.MESSAGE.err_code == com_ok)
						{
							re_code.MESSAGE.result = 's';
							normal_start = 1;
						}
						else
						{
							re_code.MESSAGE.result = 'e';
							normal_start = 2;
						}
					}
					else
					{
						re_code.MESSAGE.result = 'e';
						re_code.MESSAGE.err_code =invalid_parameter;
						normal_start = 2;
					}
					re_code.MESSAGE.len = 19;
					cmd_reseive(&re_code);					
					break;
				}
				case 0xe5:				//向 RFID 写一个 Sector 数据
				{
					re_code.MESSAGE.act_code = inbox[0];
					if(3==inbox[1]||4==inbox[1]||5==inbox[1]||6==inbox[1])
					{	   
						//re_code.MESSAGE.err_code = WriteRFIDSECTOR(inbox[1],inbox[2],inbox[3],&inbox[4]);
						if(re_code.MESSAGE.err_code == com_ok)
						{
							re_code.MESSAGE.result = 's';
							normal_start = 1;
						}
						else
						{
							re_code.MESSAGE.result = 'e';
							normal_start = 2;
						}
					}
					else
					{
						re_code.MESSAGE.result = 'e';
						re_code.MESSAGE.err_code =invalid_parameter;
						normal_start = 2;
					}
					re_code.MESSAGE.len = 3;
					cmd_reseive(&re_code);					
					break;
				}
				case 0xe6:				//从RFID中读取一个Sector数据
				{
					re_code.MESSAGE.act_code = inbox[0];
					if(3==inbox[1]||4==inbox[1]||5==inbox[1]||6==inbox[1])
					{
						//re_code.MESSAGE.err_code = ReadRFIDSECTOR(inbox[1],inbox[2],re_code.MESSAGE.info);
						if(re_code.MESSAGE.err_code == com_ok)
						{
							re_code.MESSAGE.result = 's';
							normal_start = 1;
						}
						else
						{
							re_code.MESSAGE.result = 'e';
							normal_start = 2;
						}
					}
					else
					{
						re_code.MESSAGE.result = 'e';
						re_code.MESSAGE.err_code =invalid_parameter;
						normal_start = 2;
					}
					re_code.MESSAGE.len = 51;
					cmd_reseive(&re_code);					
					break;
				}
				case 0xe7:				//读token卡序列号
				{
					re_code.MESSAGE.act_code = inbox[0];
					if(3==inbox[1]||4==inbox[1]||5==inbox[1]||6==inbox[1])
					{
						re_code.MESSAGE.err_code = ReadRFID_Serial_Number(inbox[1],re_code.MESSAGE.info);
						if(re_code.MESSAGE.err_code == com_ok)
						{
							re_code.MESSAGE.result = 's';
							normal_start = 1;
						}
						else
						{
							re_code.MESSAGE.result = 'e';
							normal_start = 2;
						}
					}
					else
					{
						re_code.MESSAGE.result = 'e';
						re_code.MESSAGE.err_code =invalid_parameter;
						normal_start = 2;
					}
					re_code.MESSAGE.len = 9;
					cmd_reseive(&re_code);					
					break;
				}
			case 0xe8:	//生产测试通道传感器、电磁铁打开关断到位
			{
					re_code.MESSAGE.act_code = inbox[0];
					test_sensorstatus(&re_code);
					re_code.MESSAGE.result = 's';
					re_code.MESSAGE.err_code = com_ok;
					re_code.MESSAGE.len = 6;
					cmd_reseive(&re_code);
					break;
			}
			case 0xe9:	//生产读取HOPPER固件版本号
			{
				re_code.MESSAGE.act_code = inbox[0];
				hop1_re.re_data[5] = 0;
				hop1_re.re_data[6] = 0;
				hop1_re.re_data[16] = 0;
				WriteHopper(hopper1,SDP_CMD_HOPPER_VERSION);//HOPPERA固件版本查询
				drop_tic_time = READ_MAS;
				while(drop_tic_time)
				{
					read_sensorstatus();
					if(hop1_re.re_data[5] == SDP_ACK_CMD_INTIME && hop1_re.re_data[6] == SDP_CMD_HOPPER_VERSION && hop1_re.re_data[24] == 0x80)
						break;
				}
				hop1_re.re_flag = 0;		   
				if(drop_tic_time != 0)
				{
					for(i=0;i<16;i++)
					{
						re_code.MESSAGE.info[i]=hop1_re.re_data[i+7];
					}
				}
	
				hop2_re.re_data[5] = 0;
				hop2_re.re_data[6] = 0;
				hop2_re.re_data[16] = 0;
				WriteHopper(hopper2,SDP_CMD_HOPPER_VERSION);//HOPPERB固件版本查询
				drop_tic_time = READ_MAS;
				while(drop_tic_time)
				{
					read_sensorstatus();
					if(hop2_re.re_data[5] == SDP_ACK_CMD_INTIME && hop2_re.re_data[6] == SDP_CMD_HOPPER_VERSION && hop2_re.re_data[24] == 0x80)
						break;
				}
				hop2_re.re_flag = 0;		   
				if(drop_tic_time != 0)
				{
					for(i=16;i<32;i++)
					{
						re_code.MESSAGE.info[i]=hop2_re.re_data[i-9];
					}
				} 
				re_code.MESSAGE.result = 's';
				re_code.MESSAGE.err_code = com_ok;
				re_code.MESSAGE.len = 35;
				cmd_reseive(&re_code);
				break;
			} 
			default:
				break;
			}
			for(i=0;i<60;i++)
				inbox[i] = 0;
			receive_ok = 0;
		}
	}

/*****************************************************************************/
/* Function Description:                                                                                               */
/*****************************************************************************/
//初始化RFID读写器
/*****************************************************************************/
/* Parameters:                                                                                                            */
/*****************************************************************************/
/*                                                                                                                              */
/*                                                                                                                              */
/*                                                                                                                            */
/*****************************************************************************/
/* Return Values:                                                                                                      */
/*****************************************************************************/
/*                                                                                                                              */
/*  无                                                                                                                      */
/*                                                                                                                              */
/*****************************************************************************/

void RFID_INIT(void)
{
	RC522_Init();
	PcdReset();
 	PcdAntennaOff(); 
	delay_ms(10);
 	PcdAntennaOn();
	delay_ms(10);
}

/****************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
****************************************************************************/
int main()
{	
	static RETURN_CODE re_code;	
	normal_start = 1;
	wdt_reseten = 1;
	antenna=INEXISTENCE;//清空天线区标志
	
	LED_control_flag=0;//默认退币口指示灯为模块控制
	
	prvSetupHardware();	
	//iwdg_init();   //独立看门狗初始化
	time_init();  //定时器3初始化	
	usart_all_init();
	module_init(&re_code);
	RFID_INIT();
	FLASH_Init();	
    //FLASH_WriteData((u8*)TEXT_Buffer, 0, SIZE);
	//FLASH_ReadData(buff, 0, SIZE);
	//Hopper自检
    //WriteHopper(hopper1,0x40);//自检命令
    //WriteHopper(hopper2,0x60);//自检命令
   // delay_ms(500);
	while(1)
	{		
		read_sensorstatus();
		//DIG_Tic();
		check_command();		
	}
}



