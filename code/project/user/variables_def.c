#include "variables_def.h"


//�汾��
uint8 MODULE_VERSION[8] = "ToIM001A";
uint8 CPU_VERSION[7] = "V1.0R01";

//���к�
uint8 MODULE_SERIAL[20] = "12345678901234567890";

//���Ź���ʱ��λ
uint8   wdt_reseten = 1;
uint16  wdt_resetreg;  

 uint8 sens_selfcheck=0;  //�������Լ��־
 uint8 result=0;
 uint8 detection=0; 		 //������Ƿ��п���־

 uint16 drop_tic_time = 0;	 //ͨ���л���ʱʱ��
 uint16 init_over_time = 0;  //��ʼ����ʱ
 uint16 over_time = 0;	  //ͨ�ų�ʱ
 uint32 rotation_over_time = 0;  //���ÿ����֮���ʱ����
 uint32 roll_over_time = 0;//���ת����ʱ1s 
 uint32 sole_open_over_time = 0;//��Ʊ�ڵ�����򿪳�ʱ1s 
 uint32 sole_openleave_over_time = 0;//��Ʊ�ڵ�����򿪺��뿪��ʱ100ms 
 uint32 sole_close_over_time = 0;//��Ʊ�ڵ�����򿪳�ʱ1s 
 uint32 sole2_open_over_time = 0;//��Ʊ�ڵ�����򿪳�ʱ1s 
 uint32 sole2_close_over_time = 0;//��Ʊ�ڵ�����򿪳�ʱ1s 
 uint16 init_delay = 0; 	 //��ʼ����ʱ��Ϊ�˵ȴ�hopper��ʼ��ת��

 
 //ģ�����ͱ���
 uint8 Module_Type = 1;  //2:���ݴ���ģ�飬1:���ݴ���ģ��
 uint8 antenna = 0; 		 //�������Ƿ��п���־
 uint8 antennaA = 0;		  //��Ʊ���Ƿ��п���־
 
 uint8 clear_tic = 0;		 //��ʼ����ͨ�����������Ʊ����
//Hopper��Ʊѡ��
uint8  hopper=0;
//hopper״̬
uint8 hopper1_state,hopper2state;


//ͨ��λ��
uint8 current_gallery;
 //�����ƿ���  1��ʾ���������ƣ�Ƶ��1HZ   ��2�����Ƽ��٣��й���
 //0ֹͣ����
 uint8 normal_start = 1; 
 //�˱ҿ�ָʾ�Ʊ�־
 uint8 LED_EN=OFF;//ģ����Ʊ�־
 //�˱ҿ�ָʾ����˸ʱ�����
 uint16 flash_cnt=0; 
 //����or�˻�Ʊ������
 uint32 box_num[3]={0,0,0};
 uint8 sum_open_over_time=0;
 
 //ͨ�Ų���  
 uint8 communication_step = 1;
 
 //�������
 uint8 tic_num1[2],tic_num2[2];
 //�����ɱ�־
 uint8 tic_flag1=1,tic_flag2=1;

RESEIVE hop1_re,hop2_re;
uint8 sta_hopper1=0,sta_hopper2=0;  //hopper״̬
  //uart0,��ECUͨ�ŷ��ͽ����ñ�������
 //ͨ�Ż���
uint8	 receive_ok;//uart0���ݽ�����ɱ�־  1:�ɹ�  0:��
uint8	 inbox[60];//�������ݻ���
 //������״̬
SENSORS sens;
SENSORS sens_last;  
 
 //RFID��ʱ��ʱ
 uint16 g_cbWaitRespDly = 0;
 //�����Ƽ���
 uint16 work_light_cnt=0;
 //���Ի�������ʹ��
 uint32 time_count=0; 
uint8 LED_control_flag;//0:  ģ�����1 :��λ������
uint16 OutGPA;
uint16 OutGPB;
uint16 OutGPC;
uint16 OutGPD;
uint16 OutGPE;
