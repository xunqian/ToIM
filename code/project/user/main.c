/*******************************************************************************
*                 
*                 		       ά�ڻ���
--------------------------------------------------------------------------------
* ������: 	����ʽ����ģ��
* ��  ��: ����TOKEN
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
* �� �� ��         : delay
* ��������		   : ��ʱ������delay(6000000)��ʱԼ1s
* ��    ��         : i
* ��    ��         : ��
*******************************************************************************/
void delay(u32 i)
{
	while(i--);
}


/*****************************************************************************/
/* Function Description:                                                     */
/*****************************************************************************/
/*   �����˶���LED                                                   */
/*                                                                           */
/*****************************************************************************/
/* Parameters:                                                               */
/*****************************************************************************/
/*   number                                                                  */
/*  number��Ӧ (abcdefg dp)���øߵ���LED                                                     */
/*****************************************************************************/
/* Return Values:                                                            */
/*****************************************************************************/
/*   none                                                                    */
/*                                                                           */
/*****************************************************************************/

void Write_LED(uint8 number)
{	//number��Ӧ (abcdefg dp)
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
/*   �����˶���LED ��ʾ��0-F��DP��                                                 */
/*                                                                           */
/*****************************************************************************/
/* Parameters:                                                               */
/*****************************************************************************/
/*   number                                                                  */
/*  number��Ӧ0-F,	����DP,number����λ��1                                                    */
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
    //��������Դ��ʼ��Ϊ�ϵ�״̬
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
//У��
//����*ptr  ����ָ��
//���� len  У�����ݳ���
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          �Ƿ�ɹ�                                                                                             */
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
//���������
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
 void SOLEA_ON(void)//��Ʊ���������
    {
    	SOLE1_ON;			
    }
 void SOLEA_OFF(void)//��Ʊ��������ر�
    {
      SOLE1_OFF;
   }    
 void SOLEB_ON(void)//�������������
    {     
        SOLE2_ON;		  
	} 
 void SOLEB_OFF(void)//������������ر�
    {
        SOLE2_OFF;	 
    }
 void SOLEC_ON(void)//���տڵ������
    {     
        SOLE3_ON;		  
	} 
 void SOLEC_OFF(void)//���տڵ�����ر�
    {
        SOLE3_OFF;	 
    }
  void SOLED_ON(void)//���õ����1��
    {     
        SOLE4_ON;		  
	} 
 void SOLED_OFF(void)//����1�����1�ر�
    {
        SOLE4_OFF;	 
    }
 void SOLEE_ON(void)//���õ����2��
    {     
        SOLE5_ON;		  
	} 
 void SOLEE_OFF(void)//���õ����2�ر�
    {
        SOLE5_OFF;	 
    }
 
 /*****************************************************************************/
 /* Function Description:																													   */
 /*****************************************************************************/
 //ģ�������򿪺���
 //����time �򿪵����ʱ��
 
 /*****************************************************************************/
 /* Parameters: 																																 */
 /*****************************************************************************/
 /* 																																					*/
 /* 																																					*/
 /* 																																					*/
 /*****************************************************************************/
 /* Return Values:			�Ƿ�ɹ�																							 */
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
 //ģ�������򿪺���
 //����time �򿪵����ʱ��
 
 /*****************************************************************************/
 /* Parameters: 																																 */
 /*****************************************************************************/
 /* 																																					*/
 /* 																																					*/
 /* 																																					*/
 /*****************************************************************************/
 /* Return Values:			�Ƿ�ɹ�																							 */
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
//ģ�������򿪺���
//����time �򿪵����ʱ��
//����mode �ж��ǽ������߻��ǳ�����
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          �Ƿ�ɹ�                                                                                             */
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
//���ֵ������������

/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          �Ƿ�ɹ�                                                                                             */
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
	drop_tic_time = 50;//1��
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
	drop_tic_time = 50;//1��
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
 //BIT��1����
 //����:bҪ��λ���ֽ�
 //����:indexҪ��λ�����к�
 /*****************************************************************************/
 /* Parameters: 																																 */
 /*****************************************************************************/
 /* 																																					*/
 /* 																																					*/
 /* 																																					*/
 /*****************************************************************************/
 /* Return Values:			��λ����ֽ�																						*/
 /*****************************************************************************/
 /* 																																					 */
 /* 																																   */
 /* 																																					 */
 /*****************************************************************************/
 
 uint8 SetBit(uint8 b, uint8 index)//����indexλ��Ϊ1
{ 
	 b |=(1 << index);
	 return b;			 
}
 /*****************************************************************************/
 /* Function Description:																													   */
 /*****************************************************************************/
 //BIT��0����
 //����:bҪ��λ���ֽ�
 //����:indexҪ��λ�����к�
 /*****************************************************************************/
 /* Parameters: 																																 */
 /*****************************************************************************/
 /* 																																					*/
 /* 																																					*/
 /* 																																					*/
 /*****************************************************************************/
 /* Return Values:			��λ����ֽ�																						*/
 /*****************************************************************************/
 /* 																																					 */
 /* 																																   */
 /* 																																					 */
 /*****************************************************************************/
 
 
 uint8 ResetBit(uint8 b, uint8 index) //����indexλ��Ϊ0
{
	 b&= ~(1 << index);//��0
	 return b;
}
/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//������Ӧ����
//����cmd  ��Ӧ�������ֽ�
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          �Ƿ�ɹ�                                                                                             */
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
//�ظ���λ������
//����*cmd �ظ�������ָ��
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          �Ƿ�ɹ�                                                                                             */
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
//�幷����ȡ������״̬
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          �Ƿ�ɹ�                                                                                             */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   NULL                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/
void read_sensorstatus(void)
{   
    if(wdt_reseten == 1)
	IWDG_ReloadCounter();	   //1.28s
	sens_last = sens;	//SENSORS���������
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
	//��λ����
	sens.status[12] = GPIO_ReadInputDataBit(IN1_PORT, IN1_BIT_NUM);
	sens.status[13] = GPIO_ReadInputDataBit(IN2_PORT, IN2_BIT_NUM);
	sens.status[14] = GPIO_ReadInputDataBit(IN3_PORT, IN3_BIT_NUM);
	//sens.status[15] = GPIO_ReadInputDataBit(IN4_PORT, IN4_BIT_NUM);
	//sens.status[16] = GPIO_ReadInputDataBit(IN5_PORT, IN5_BIT_NUM);
	//sens.status[17] = GPIO_ReadInputDataBit(IN6_PORT, IN6_BIT_NUM);
	if(sens.SENSORS_STATUS.checkticks1 ==0x01)
    {	
    	//��Ʊ����Ʊ
        antennaA = EXISTENCE;
        antennaA_LED_ON;
    }
	else if(sens.SENSORS_STATUS.checkticks1 ==0x00)
	{
        //��Ʊ����Ʊ
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
//�ϵ�ϵ��⴫����
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
	sens_selfcheck=1;  //�������Լ��־
	//��ʼ���رհ˶��������ʾ������ϴ�������
	LED_Display(0x00);
	Write_LED(0x00);
	//���д���������˶ϵ�Ϊ0���ϵ����ڵ�Ϊ1��
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
    
    antenna = INEXISTENCE;    //�����������־
    antennaA= INEXISTENCE;   //��ձ�Ʊ����־
    sens_selfcheck=0;  //�������Լ��־
}

/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//�����ж�
//����*re_comm Ҫ���͵�����
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          �Ƿ�ɹ�                                                                                             */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   NULL                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/
void box_status_anto(RETURN_CODE *re_comm)
{   
    if(sens.SENSORS_STATUS.checkticks13 == 0)
        re_comm->MESSAGE.info[0] |= 0x02;        //��Ʊ�䵽λ
    else
        re_comm->MESSAGE.info[0] &= 0xfd;        //��Ʊ��δ��λ
    if(sens.SENSORS_STATUS.checkticks14 == 0)
        re_comm->MESSAGE.info[0] |= 0x08;        //��Ʊ�䵽λ
    else
        re_comm->MESSAGE.info[0] &= 0xf7;        //��Ʊ��δ��λ
    if(sens.SENSORS_STATUS.checkticks15 == 0)
        re_comm->MESSAGE.info[0] |= 0x80;        //ģ�鵽λ
    else
        re_comm->MESSAGE.info[0] &= 0x7f;        //ģ��δ��λ
        
    WriteHopper(hopper1,SDP_CMD_STATE_QUERY);//״̬��ѯ
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
            re_comm->MESSAGE.info[0] |= 0x10;        //A��
        else
            re_comm->MESSAGE.info[0] &= 0xef;        //A�ǿ�
        if(sta_hopper1 & 0x20)
            re_comm->MESSAGE.info[0] |= 0x01;        //A����
        else
            re_comm->MESSAGE.info[0] &= 0xfe;        //A�ǿ�
    }
    else
    {
        if(hop1_re.re_data[11] & 0x10)
            re_comm->MESSAGE.info[0] |= 0x10;        //A��
        else
            re_comm->MESSAGE.info[0] &= 0xef;        //A�ǿ�
        if(hop1_re.re_data[11] & 0x20)
            re_comm->MESSAGE.info[0] |= 0x01;        //A����
        else
            re_comm->MESSAGE.info[0] &= 0xfe;        //A�ǿ�
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
            re_comm->MESSAGE.info[0] |= 0x20;        //B��
        else
            re_comm->MESSAGE.info[0] &= 0xdf;        //B�ǿ�
        if(sta_hopper2 & 0x20)
            re_comm->MESSAGE.info[0] |= 0x04;        //B����
        else
            re_comm->MESSAGE.info[0] &= 0xfb;        //B�ǿ�
    }
    else
    {
        if(hop2_re.re_data[11] & 0x10)
            re_comm->MESSAGE.info[0] |= 0x20;        //B��
        else
            re_comm->MESSAGE.info[0] &= 0xdf;        //B�ǿ�
        if(hop2_re.re_data[11] & 0x20)
            re_comm->MESSAGE.info[0] |= 0x04;        //B����
        else
            re_comm->MESSAGE.info[0] &= 0xfb;        //B�ǿ�
    }    
    if(sens.SENSORS_STATUS.checkticks3 == 0)
    { 
        re_comm->MESSAGE.info[1] &= 0xfe;        //���տ���Ʊ//�޴���
    }
    else
    {
        re_comm->MESSAGE.info[1] |= 0x01;        //���տ���Ʊ//�д���
    }
    
    if(antenna == EXISTENCE)
        re_comm->MESSAGE.info[0] |= 0x40;        //�������п�
    else
        re_comm->MESSAGE.info[0] &= 0xbf;        //�������޿�
}
/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//�ϵ�ϵ��⴫����
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
void Power_On_Off_Sensor(void)//-------------------------------������ʵʱ״̬����
{    
	//���ϵ��sensor״̬�ٶϵ��sensor״̬
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
/*   �������Լ�                                                                                                                     */
/*                                                                                                                                                      */
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   *re_code: ���صĴ������Լ�״̬                                                                      */
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
	sens_selfcheck=1;  //�������Լ��־
	//��ʼ���رհ˶��������ʾ������ϴ�������
	LED_Display(0x00);
	Write_LED(0x00);
	//���д���������˶ϵ�Ϊ0���ϵ����ڵ�Ϊ1��
	Power_On_Off_Sensor();
	if((sens.SENSORS_STATUS.checkticks1 ==0)&&(sens_last.SENSORS_STATUS.checkticks1==1))
    {
		re_comm->MESSAGE.info[0] &= 0xfe; //��Ʊ������������
    }
	else
	{
    	re_comm->MESSAGE.info[0] |= 0x01; //��Ʊ������������
     	re_comm->MESSAGE.err_code = TS_sens_err;
		LED_Display(0x11);
	}
	if((sens.SENSORS_STATUS.checkticks2 ==0)&&(sens_last.SENSORS_STATUS.checkticks2==1))
    {
		re_comm->MESSAGE.info[0] &= 0xfd; //����������������
    }
	else
	{
    	re_comm->MESSAGE.info[0] |= 0x02; //����������������
     	re_comm->MESSAGE.err_code = Antenna_sens_err;
		LED_Display(0x12);
	}
	if((sens.SENSORS_STATUS.checkticks3 ==0)&&(sens_last.SENSORS_STATUS.checkticks3==1))
    {
		re_comm->MESSAGE.info[0] &= 0xfb; //Ʊ������Ʊ�ڴ���������
    }
	else
	{
    	re_comm->MESSAGE.info[0] |= 0x04; //Ʊ������Ʊ�ڴ���������
     	re_comm->MESSAGE.err_code = Recy_sens_err;
		LED_Display(0x13);
	}	
	re_comm->MESSAGE.info[0] &=0xdf;
  //  read_sensorstatus();//���Ӷ�������
	antenna = INEXISTENCE;
	antenna_LED_OFF;
	//**read HOPPERA��������״ֵ̬**//
	hop1_re.re_data[5] = 0;
	hop1_re.re_data[6] = 0;
	hop1_re.re_data[16] = 0;
	WriteHopper(hopper1,SDP_CMD_TEST_SENSOR);//HOPPERA����������ָ��
	drop_tic_time = 3 * DROP_T2;//ʱ������Ϊ1000ms
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
			re_comm->MESSAGE.info[1] |= 0x01;		 //A���ҿڴ�����1�Լ����
		else
			re_comm->MESSAGE.info[1] &= 0xfe;		 //A���ҿڴ�����1�Լ�����
		if(hop1_re.re_data[7] & 0x02)
			re_comm->MESSAGE.info[1] |= 0x02;		 //A���ҿڴ�����2�Լ����
		else
			re_comm->MESSAGE.info[1] &= 0xfd;		 //A���ҿڴ�����2�Լ�����
		if(hop1_re.re_data[7] & 0x04)
			re_comm->MESSAGE.info[1] |= 0x04;		 //A�������λ�������Լ����
		else
			re_comm->MESSAGE.info[1] &= 0xfb;		 //A�������λ�������Լ�����
		if(hop1_re.re_data[11] & 0x10)
			re_comm->MESSAGE.info[1] |= 0x08;		 //A���Լ����
		else
			re_comm->MESSAGE.info[1] &= 0xf7;		 //A���Լ�����
		if(hop1_re.re_data[11] & 0x20)
			re_comm->MESSAGE.info[1] |= 0x10;		 //A�����Լ����
		else
			re_comm->MESSAGE.info[1] &= 0xef;		 //A�����Լ�����
		  
	 //   re_comm->MESSAGE.info[1]&=0x3f;
	}
	//**read HOPPERB��������״ֵ̬**//
	hop2_re.re_data[5] = 0;
	hop2_re.re_data[6] = 0;
	hop2_re.re_data[16] = 0;
	WriteHopper(hopper2,SDP_CMD_TEST_SENSOR);//HOPPERB����������ָ��
	drop_tic_time = 3 * DROP_T2;//ʱ������Ϊ1000ms;
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
			re_comm->MESSAGE.info[2] |= 0x01;		 //B���ҿڴ�����1�Լ����
		else
			re_comm->MESSAGE.info[2] &= 0xfe;		 //B���ҿڴ�����1�Լ�����
		if(hop2_re.re_data[7] & 0x02)
			re_comm->MESSAGE.info[2] |= 0x02;		 //B���ҿڴ�����2�Լ����
		else
			re_comm->MESSAGE.info[2] &= 0xfd;		 //B���ҿڴ�����2�Լ�����
		if(hop2_re.re_data[7] & 0x04)
			re_comm->MESSAGE.info[2] |= 0x04;		 //B�������λ�������Լ����
		else
			re_comm->MESSAGE.info[2] &= 0xfb;		 //B�������λ�������Լ�����
		if(hop2_re.re_data[11] & 0x10)
			re_comm->MESSAGE.info[2] |= 0x08;		 //B���Լ����
		else
			re_comm->MESSAGE.info[2] &= 0xf7;		 //B���Լ�����
		if(hop2_re.re_data[11] & 0x20)
			re_comm->MESSAGE.info[2] |= 0x10;		 //B�����Լ����
		else
			re_comm->MESSAGE.info[2] &= 0xef;		 //B�����Լ�����
	  //   re_comm->MESSAGE.info[2]&=0x3f;
	}	
	antenna = INEXISTENCE;    //�����������־
    antennaA= INEXISTENCE;   //��ձ�Ʊ����־
    sens_selfcheck=0;  //�������Լ��־
	
} 
 /*****************************************************************************/
 /* Function Description:																													   */
 /*****************************************************************************/
 //���״̬
 //���� *re_comm Ҫ���͵�����
 /*****************************************************************************/
 /* Parameters: 																																 */
 /*****************************************************************************/
 /* 																																					*/
 /* 																																					*/
 /* 																																					*/
 /*****************************************************************************/
 /* Return Values:			�Ƿ�ɹ�																							 */
 /*****************************************************************************/
 /* 																																					 */
 /*   NULL																																			 */
 /* 																																					 */
 /*****************************************************************************/
 void check_sensorstatus(RETURN_CODE *re_comm)//-------------------------------������ʵʱ״̬����
 {
	 read_sensorstatus();
	 box_status_anto(re_comm);
 }
 
 /*****************************************************************************/
 /* Function Description:																													   */
 /*****************************************************************************/
 //��Ʊ����
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
			 //Ahopperδ��
			 if(Module_Type == HAVE_TEMPORARY_AREA)
			 {
				 if(antennaA == INEXISTENCE)
				 {
					 //�Ĵ�����
					 antennaA = DIG_TICKET;
					 WriteHopper(hopper1,SDP_CMD_CHG);
				 }
			 }
		 }
		 else
		 {
			 //A��
			 antennaA= EMPTY;
		 }		 
	 }	 
	 if(hopper == hopper2)
	 {	  
		 if((sta_hopper2 & 0x10) == 0)
		 {
			 //Bhopperδ��
			 
			 if(Module_Type == HAVE_TEMPORARY_AREA)
			 {
				 if(antennaA == INEXISTENCE)
				 {
					 //�Ĵ�����
					 antennaA = DIG_TICKET;
					 WriteHopper(hopper2,SDP_CMD_CHG);
				 }
			 }
		 }
		 else
		 {
			 //B��
			 antennaA = EMPTY;
		 }
	 }	 
 }
 /*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//����ģʽ
//����cmd ��ǰ������
//����*act ���ص�����ָ��
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/* ��Ʊ����Ʊ������       Ʊ����                                                                                                                                             */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          �Ƿ�ɹ�                                                                                             */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   NULL                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/
void normal_working(uint8 cmd,RETURN_CODE *act)
 {
	 
	 switch(cmd)
	 {
		 case CMD_BOX1SCOOP://BOX1��Ʊ
		 {
			 uint8 time=0;
			 if(antenna == INEXISTENCE)//��������Ʊ
			 {				 
				 if(antennaA == EXISTENCE)//��Ʊ����Ʊ
				 {
					 if(Module_Type == HAVE_TEMPORARY_AREA)//�б�Ʊ��
					 {
						 for(time=0;time<3;time++)
						 {							
							 SOLE1_ON;
							 Sole_Open(DCT_T2,EXISTENCE);
							 if(antenna == EXISTENCE) //�жϳ�Ʊ�ɹ�����
							 {									 
								 SOLE1_OFF;
								 break;
							 }
							 SOLE1_OFF;
							 delay(500);							
						 }
					 }
					 else//����Ʊ��
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
									 if(antenna == EXISTENCE) //�жϳ�Ʊ�ɹ�����
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
				 else //��Ʊ��ûƱ
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
								 if(antenna == EXISTENCE) //�жϳ�Ʊ�ɹ�����
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
						 //�����ǰHOPPER������Ʊ
						 
						 while(drop_tic_time)
						 {
							 read_sensorstatus();
							 if(antennaA != DIG_TICKET)
								 break;
						 }
					 }
					 if((drop_tic_time == 0 )|| (antennaA != EXISTENCE))
					 {
						 //ʱ�䵽����û�ڳ�Ʊ����Ʊʧ��
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
							 //�����ʱ��ʾ��Ʊʧ��
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
			 else//һ��������Ʊ��������
			 {
				 //��Ʊ�򱨾�
				 act->MESSAGE.result = 'w';
				 act->MESSAGE.err_code = card_at_RW_area;
				 act->MESSAGE.info[2] = 0;
			 }
			 break;
		 }
		 case CMD_BOX2SCOOP://BOX2��Ʊ
		 {
			 uint8 time=0;
			 if(antenna == INEXISTENCE)//��������Ʊ
			 {				 
				 if(antennaA == EXISTENCE)//��Ʊ����Ʊ
				 {
					 if(Module_Type == HAVE_TEMPORARY_AREA)//�б�Ʊ��
					 {
						 for(time=0;time<3;time++)
						 {							
							 SOLE1_ON;
							 Sole_Open(DCT_T2,EXISTENCE);
							 if(antenna == EXISTENCE) //�жϳ�Ʊ�ɹ�����
							 {									 
								 SOLE1_OFF;
								 break;
							 }
							 SOLE1_OFF;
							 delay(500);							
						 }
					 }
					 else//����Ʊ��
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
									 if(antenna == EXISTENCE) //�жϳ�Ʊ�ɹ�����
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
				 else //��Ʊ��ûƱ
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
								 if(antenna == EXISTENCE) //�жϳ�Ʊ�ɹ�����
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
						 //�����ǰHOPPER������Ʊ
						 
						 while(drop_tic_time)
						 {
							 read_sensorstatus();
							 if(antennaA != DIG_TICKET)
								 break;
						 }
					 }
					 if((drop_tic_time == 0 )|| (antennaA != EXISTENCE))
					 {
						 //ʱ�䵽����û�ڳ�Ʊ����Ʊʧ��
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
							 //�����ʱ��ʾ��Ʊʧ��
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
			 else//һ��������Ʊ��������
			 {
				 //��Ʊ�򱨾�
				 act->MESSAGE.result = 'w';
				 act->MESSAGE.err_code = card_at_RW_area;
				 act->MESSAGE.info[2] = 0;
			 }
			 break;
		 }
		 case CMD_TICKETSELL://��Ʊ
		 {
			 uint8 time=3;
			 //HOPPER1_LED_OFF;
			 //HOPPER2_LED_OFF;
			 if(antenna == EXISTENCE)
			 {
				 //��������Ʊ					
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
				 //��Ʊ��澯
				 act->MESSAGE.result = 'w';
				 act->MESSAGE.err_code = no_card_at_RW_area;
				 act->MESSAGE.info[2] = 0;
			 }
		   //  delay(10);
			 break;
		 }
		 case CMD_TICKETRECYCLE://����
		 {
			 uint8 time=3;
			 //HOPPER1_LED_OFF;
			 //HOPPER2_LED_OFF;
			 if(antenna == EXISTENCE)
			 {
				 //��������Ʊ					 
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
				 //��Ʊ��澯
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
//ģ���ʼ��
//����*re_comm �ظ�������ָ��
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          �Ƿ�ɹ�                                                                                             */
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
    antenna = INEXISTENCE;       //�����������־
    clear_tic = 0;              //��ճ�ʼ��������ֵ
    hop1_re.re_data[5] = 0;
    hop2_re.re_data[5] = 0;
  	//Hopper�Լ�
    WriteHopper(hopper1,SDP_CMD_TEST_SENSOR);//�Լ�����
    WriteHopper(hopper2,SDP_CMD_TEST_SENSOR);    
    //ͨ������ͼ������1��3
    SOLE3_ON;
	SOLE1_ON;	
	Sole_Open(DCT_T3,NO_STA);//ʱ�������
	SOLE1_OFF;
	SOLE3_OFF;
	re_comm->MESSAGE.info[2] = clear_tic;  
    //�������2
    SOLE2_ON;
	delay_ms(100);//ʱ�������
	SOLE2_OFF;
	//�������Լ�      
    sensor_self_check(re_comm); 
	//��ȡ������Hopper�Լ���Ϣ
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
    //�ڳ���ǰ���ж�HOPPER�Ƿ�Ϊ��
    WriteHopper(hopper1,SDP_CMD_STATE_QUERY);
    WriteHopper(hopper2,SDP_CMD_STATE_QUERY);
    drop_tic_time = DCT_T2;
    while(drop_tic_time);
    if((hop1_re.re_data[5] == SDP_ACK_CMD_INTIME) && (hop1_re.re_data[6] == SDP_CMD_STATE_QUERY )&& (hop1_re.re_data[16] == 0x80))
    {
        sta_hopper1 = hop1_re.re_data[11];
        if((hop1_re.re_data[11] & 0x10) == 0)
        {
            //�ǿ���Ʊ
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
            //�ǿ���Ʊ
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
//RFID��д��ͨ��ѡ��
/*****************************************************************************/
/* Parameters:                                                                                                            */
/*****************************************************************************/
/*   Galleryy: ͨ��ѡ��                                                                                                                          */
/*                                                                                                                              */
/*                                                                                                                            */
/*****************************************************************************/
/* Return Values:                                                                                                      */
/*****************************************************************************/
/*                                                                                                                              */
/*  ��                                                                                                                      */
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
//��ȡƱ�����оƬ������
//����box_no Ʊ����ȡֵ��ΧΪ:2,3,4
//����block_no���ַȡֵ��ΧΪ:8,9,10\12,13,14\�ȣ����S50��IC���ֲ�
//����*pDATA���صĵ���оƬ������ָ��
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                           */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          �Ƿ�ɹ�                                                                                             */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   0x00:OK  ��������                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/

uint8 ReadRFIDBLOCK(uint8 box_no,uint8 block_no,uint8 *pDATA)
{
	unsigned char status=0xfe;
	unsigned char g_ucTempbuf[30];	
	unsigned char DefaultKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
	RFID_Gallery_select(box_no-2);
	status = PcdRequest(PICC_REQALL, g_ucTempbuf);//Ѱ��
	if (status != MI_OK)
	{
		//Ѱ����ʱʱ����Ϊ0.5s
		g_cbWaitRespDly=25;
		while(g_cbWaitRespDly!=0)
		{
			read_sensorstatus();
			PcdReset();
			PcdAntennaOff(); 
			PcdAntennaOn(); 
			status = PcdRequest(PICC_REQALL, g_ucTempbuf);//Ѱ��
			if(status==MI_OK)
				break;
		}		
	}
	//��ʱʱ����Ϊ0.5s
	g_cbWaitRespDly=25;	
	do{
			read_sensorstatus();
		status = PcdAnticoll(g_ucTempbuf);//����ײ
			if(status == MI_OK)
				break;
		}while(g_cbWaitRespDly!=0);		 
	//��ʱʱ����Ϊ0.5s
	g_cbWaitRespDly=25;	
	do{
			status = PcdSelect(g_ucTempbuf);//ѡ����Ƭ
			if(status == MI_OK)
				break;
		}while(g_cbWaitRespDly!=0);	
	//��ʱʱ����Ϊ0.5s
	g_cbWaitRespDly=25;	
	do{
			read_sensorstatus();
		//��֤��Ƭ����
			status = PcdAuthState(PICC_AUTHENT1A, block_no, DefaultKey, g_ucTempbuf);
			if(status == MI_OK)
				break;
		}while(g_cbWaitRespDly!=0);
	g_cbWaitRespDly=50;	
	do{			
			read_sensorstatus();
		status = PcdRead(block_no, pDATA);//����
			if(status == MI_OK)
				break;
		}while(g_cbWaitRespDly!=0); 

	return status;
}
/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//д��Ʊ�����оƬ������
//����box_no Ʊ����ȡֵ��ΧΪ:2,3,4
//����block_no���ַȡֵ��ΧΪ:8,9,10\12,13,14\�ȣ����S50��IC���ֲ�
//����*pDATAд�����оƬ������ָ��
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                           */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          �Ƿ�ɹ�                                                                                             */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   0x00:OK  ��������                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/

uint8 WriteRFIDBLOCK(uint8 box_no,uint8 block_no,uint8 *pDATA)


{
	unsigned char status=0xfe;
	unsigned char g_ucTempbuf[30];	
	unsigned char DefaultKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
	RFID_Gallery_select(box_no-2);
	status = PcdRequest(PICC_REQALL, g_ucTempbuf);//Ѱ��
	if (status != MI_OK)
	{
		//Ѱ����ʱʱ����Ϊ0.5s
		g_cbWaitRespDly=25;
		while(g_cbWaitRespDly!=0)
		{
			read_sensorstatus();
			PcdReset();
			PcdAntennaOff(); 
			PcdAntennaOn(); 
			status = PcdRequest(PICC_REQALL, g_ucTempbuf);//Ѱ��
			if(status==MI_OK)
				break;
		}		
	}
	//��ʱʱ����Ϊ0.5s
	g_cbWaitRespDly=25;	
	do{
		read_sensorstatus();
		status = PcdAnticoll(g_ucTempbuf);//����ײ
			if(status == MI_OK)
				break;
		}while(g_cbWaitRespDly!=0);		 
	//��ʱʱ����Ϊ0.5s
	g_cbWaitRespDly=25;	
	do{
			read_sensorstatus();
		status = PcdSelect(g_ucTempbuf);//ѡ����Ƭ
			if(status == MI_OK)
				break;
		}while(g_cbWaitRespDly!=0);	
	//��ʱʱ����Ϊ0.5s
	g_cbWaitRespDly=25;	
	do{
			read_sensorstatus();
		//��֤��Ƭ����
			status = PcdAuthState(PICC_AUTHENT1A, block_no, DefaultKey, g_ucTempbuf);
			if(status == MI_OK)
				break;
		}while(g_cbWaitRespDly!=0);		 
	//��ʱʱ����Ϊ1s
	g_cbWaitRespDly=50;	
	do{
			read_sensorstatus();
		status = PcdWrite(block_no,pDATA);//д��
			if(status == MI_OK)
				break;
		}while(g_cbWaitRespDly!=0); 
	return status;
}
/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//��ȡƱ�����оƬ������
//����box_no Ʊ����ȡֵ��ΧΪ:2,3,4
//����*pDATA���صĵ���оƬ������ָ��
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                           */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          �Ƿ�ɹ�                                                                                             */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   0x00:OK  ��������                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/

uint8 ReadRFID_Serial_Number(uint8 box_no,uint8 *pDATA)
{
	unsigned char status=0xfe;
	//unsigned char g_ucTempbuf[30];	
	RFID_Gallery_select(box_no-2);
	status = PcdRequest(PICC_REQALL,pDATA);//Ѱ��
	if (status != MI_OK)
	{
		//Ѱ����ʱʱ����Ϊ0.5s
		g_cbWaitRespDly=25;
		while(g_cbWaitRespDly!=0)
		{
			PcdReset();
			PcdAntennaOff(); 
			PcdAntennaOn(); 
			status = PcdRequest(PICC_REQALL,pDATA);//Ѱ��
			if(status==MI_OK)
				break;
		}		
	}
	//��ʱʱ����Ϊ0.5s
	g_cbWaitRespDly=25; 
	do{
			status = PcdAnticoll(pDATA);//����ײ
			if(status == MI_OK)
				break;

		}while(g_cbWaitRespDly!=0); 	 
	//��ʱʱ����Ϊ0.5s
	g_cbWaitRespDly=25; 
	do{
			status = PcdSelect(pDATA);//ѡ����Ƭ
			if(status == MI_OK)
				break;

		}while(g_cbWaitRespDly!=0);
	return status;
}
/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//ͨ���������
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:          �Ƿ�ɹ�                                                                                             */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   NULL                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/
void check_command(void)//�������
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
				case 0x0:			//���»ظ��ϴ�����
				{
					cmd_reseive(&re_code);
					break;
				}
				case 0x81:			//��ʼ��	
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
				case 0x82:			//��ģ��״̬
				{
					re_code.MESSAGE.act_code = inbox[0];
					check_sensorstatus(&re_code);
					re_code.MESSAGE.result = 's';
					re_code.MESSAGE.err_code = com_ok;
					re_code.MESSAGE.len = 5;
					cmd_reseive(&re_code);
					break;
				}
				case 0x83:			//ͨ������
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
				case 0x84:			//��Ʊ��������
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
				case 0x85:			//��Ʊ
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
				case 0x86:			//����
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
				case 0x88:			//�汾��
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
				case 0x89:			//���token��
				{
					re_code.MESSAGE.act_code = inbox[0];
					hop1_re.re_data[5] = 0; 	//	����Ҫ����������´�����ҵ�ʱ�������������ʾ�豸æ
					hop2_re.re_data[5] = 0;
					if(1==inbox[1]||2==inbox[1])
					{
						WriteHopper(inbox[1],SDP_CMD_CLR);
						if(inbox[1] == hopper1)
						{
							tic_flag1 = 0;
							delay(100);
							if(((hop1_re.re_data[5] == SDP_ACK_CMD_ACK) && (hop1_re.re_data[6] == SDP_CMD_CLR))
								|| ((hop1_re.re_data[5] == SDP_ACK_CMD_UNTIME) && (hop1_re.re_data[6] == SDP_CMD_CLR)))//����·�����
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
				case 0x8a:			//��ȡ�������
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
				case 0x8b:			//ֹͣ���token��
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
			
				case 0x97:			//���ÿ����Ʊ������
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
				case 0x98:			//����Ʊ��//��ʱ���ù�
				{
					break;
				}
				case 0x99:				//��ȡƱ����
				{
					re_code.MESSAGE.act_code = inbox[0];
					if(3==inbox[1]||4==inbox[1]||5==inbox[1]||6==inbox[1])
					{
						re_code.MESSAGE.err_code=ReadRFIDBLOCK(inbox[1],1,re_code.MESSAGE.info);	//��ȡƱ���Ʊ���ţ���1��
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
				case 0xa0:			//����ģ������
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
			/*	case 0xa1:			//��ȡģ������
				{
					re_code.MESSAGE.act_code = inbox[0];
					re_code.MESSAGE.result = 's';
					re_code.MESSAGE.err_code = com_ok;
					re_code.MESSAGE.info[0] = Module_Type;
					re_code.MESSAGE.len = 4;
					cmd_reseive(&re_code);
					break;
				}*/
				case 0xe3:				//�� RFID дһ�� Block ����
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
				case 0xe4:				//��RFID�ж�ȡһ��Block����
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
				case 0xe5:				//�� RFID дһ�� Sector ����
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
				case 0xe6:				//��RFID�ж�ȡһ��Sector����
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
				case 0xe7:				//��token�����к�
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
			case 0xe8:	//��������ͨ����������������򿪹ضϵ�λ
			{
					re_code.MESSAGE.act_code = inbox[0];
					test_sensorstatus(&re_code);
					re_code.MESSAGE.result = 's';
					re_code.MESSAGE.err_code = com_ok;
					re_code.MESSAGE.len = 6;
					cmd_reseive(&re_code);
					break;
			}
			case 0xe9:	//������ȡHOPPER�̼��汾��
			{
				re_code.MESSAGE.act_code = inbox[0];
				hop1_re.re_data[5] = 0;
				hop1_re.re_data[6] = 0;
				hop1_re.re_data[16] = 0;
				WriteHopper(hopper1,SDP_CMD_HOPPER_VERSION);//HOPPERA�̼��汾��ѯ
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
				WriteHopper(hopper2,SDP_CMD_HOPPER_VERSION);//HOPPERB�̼��汾��ѯ
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
//��ʼ��RFID��д��
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
/*  ��                                                                                                                      */
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
	antenna=INEXISTENCE;//�����������־
	
	LED_control_flag=0;//Ĭ���˱ҿ�ָʾ��Ϊģ�����
	
	prvSetupHardware();	
	//iwdg_init();   //�������Ź���ʼ��
	time_init();  //��ʱ��3��ʼ��	
	usart_all_init();
	module_init(&re_code);
	RFID_INIT();
	FLASH_Init();	
    //FLASH_WriteData((u8*)TEXT_Buffer, 0, SIZE);
	//FLASH_ReadData(buff, 0, SIZE);
	//Hopper�Լ�
    //WriteHopper(hopper1,0x40);//�Լ�����
    //WriteHopper(hopper2,0x60);//�Լ�����
   // delay_ms(500);
	while(1)
	{		
		read_sensorstatus();
		//DIG_Tic();
		check_command();		
	}
}



