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
/*  2017-8-23 * xunqian.hu    * Creation of the file                                                                         */
/*             *                 *                                                                                                                    */
/*****************************************************************************/
/*  Target : stm32                                                                                                                               */
/*  Crystal: 72Mhz                                                                                                                             */
/*****************************************************************************/


/*****************************************************************************/
/*                                                                                                                                                      */
/*  Include Files                                                                                                                                   */
/*                                                                                                                                                      */
/*****************************************************************************/

#include "Hopper.h"
#include "variables_def.h"
/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//���ڷ��ͺ���
//����:*dat   ָ��������
/*****************************************************************************/
/* Parameters:                                                                                                                                  */
/*****************************************************************************/
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*                                                                                                                                                     */
/*****************************************************************************/
/* Return Values:                                                                                                   */
/*****************************************************************************/
/*                                                                                                                                                      */
/*   NULL                                                                                                                                           */
/*                                                                                                                                                      */
/*****************************************************************************/

void SendTX1(uint8  dat)
{
	USART_SendData(USART2,dat);//ͨ������USARTx���͵�������
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==Bit_RESET);
}

void SendTX2(uint8  dat)
{
	USART_SendData(USART3,dat);//ͨ������USARTx���͵�������
	while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==Bit_RESET);
}

/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//����ָ�����ݵ�hopper
//����:*dat   ָ��������
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

void send_hop1_dat(uint8 *dat)
{
    uint8 i;
    CMD_HOPPER cmd_hop;
    SendTX1(HOP_STX);
	
    for(i=0;i<4+dat[3];i++)
    {
        if(dat[i] == HOP_STX || dat[i] == HOP_ETX)
        {
            SendTX1(HOP_STX);
            SendTX1(dat[i]);
        }
        else
        {
            SendTX1(dat[i]);
        }
    }
    SendTX1(dat[sizeof(cmd_hop)-1]);
    SendTX1(HOP_ETX);
}
void send_hop2_dat(uint8 *dat)
{
    uint8 i;
    CMD_HOPPER cmd_hop;
    SendTX2(HOP_STX);
    for(i=0;i<4+dat[3];i++)
    {
        if(dat[i] == HOP_STX || dat[i] == HOP_ETX)
        {
            SendTX2(HOP_STX);
            SendTX2(dat[i]);
        }
        else
        {
            SendTX2(dat[i]);
        }
    }
    SendTX2(dat[sizeof(cmd_hop)-1]);
    SendTX2(HOP_ETX);
}

void SWHICH_HOP(uint8 hop_id,uint8 *dat)
{
	if(hop_id==hopper1)
		send_hop1_dat(dat);
	else if(hop_id==hopper2)
		send_hop2_dat(dat);
}
/*****************************************************************************/
/* Function Description:                                                                                                                      */
/*****************************************************************************/
//���ɷ��͵�hopper������
//����:box_id  Ʊ��ţ�ȡֵ��Χ1��2
//����:SDP_CMD   ����
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

void WriteHopper(uint8 hop_id, uint8 SDP_CMD)
{
    static CMD_HOPPER cmd_hop = {0,1,0};
    switch(SDP_CMD)
    {
        case SDP_CMD_STATE_QUERY:
            cmd_hop.DLEN = 1;
            cmd_hop.INFO = SDP_CMD_STATE_QUERY;
            cmd_hop.BCC = cmd_hop.RSEQ + cmd_hop.DESA + cmd_hop.SRCA + cmd_hop.DLEN + cmd_hop.INFO;           
            SWHICH_HOP(hop_id,(uint8*)&cmd_hop);            
            break;
        case SDP_CMD_CHG:
            cmd_hop.DLEN = 3;
            cmd_hop.INFO = SDP_CMD_CHG;
            cmd_hop.DAT[0] = 0;
            cmd_hop.DAT[1] = 1;
            cmd_hop.BCC = cmd_hop.RSEQ + cmd_hop.DESA + cmd_hop.SRCA + cmd_hop.DLEN + cmd_hop.INFO
                         + cmd_hop.DAT[0] + cmd_hop.DAT[1];
           SWHICH_HOP(hop_id,(uint8*)&cmd_hop);
            break;
        case SDP_CMD_CLR:
            cmd_hop.DLEN = 3;
            cmd_hop.INFO = SDP_CMD_CLR;
            cmd_hop.DAT[0] = 0;
            cmd_hop.DAT[1] = 0;
            cmd_hop.BCC = cmd_hop.RSEQ + cmd_hop.DESA + cmd_hop.SRCA + cmd_hop.DLEN + cmd_hop.INFO;
            SWHICH_HOP(hop_id,(uint8*)&cmd_hop);
            break;
        case SDP_CMD_HD_SELF:
            cmd_hop.DLEN = 3;
            cmd_hop.INFO = SDP_CMD_HD_SELF;
            cmd_hop.DAT[0] = 0x04;
            cmd_hop.DAT[1] = 0x01;
            cmd_hop.BCC = cmd_hop.RSEQ + cmd_hop.DESA + cmd_hop.SRCA + cmd_hop.DLEN + cmd_hop.INFO
                         + cmd_hop.DAT[0] + cmd_hop.DAT[1];
            SWHICH_HOP(hop_id,(uint8*)&cmd_hop);
            break;
        case SDP_CMD_STOP_HD_SELF:
            cmd_hop.DLEN = 3;
            cmd_hop.INFO = SDP_CMD_HD_SELF;
            cmd_hop.DAT[0] = 0x04;
            cmd_hop.DAT[1] = 0x00;
            cmd_hop.BCC = cmd_hop.RSEQ + cmd_hop.DESA + cmd_hop.SRCA + cmd_hop.DLEN + cmd_hop.INFO
                         + cmd_hop.DAT[0] + cmd_hop.DAT[1];
           SWHICH_HOP(hop_id,(uint8*)&cmd_hop);
            break;
        case SDP_CMD_VERSION_QUERY:
            cmd_hop.DLEN = 1;
            cmd_hop.INFO = SDP_CMD_VERSION_QUERY;
            cmd_hop.BCC = cmd_hop.RSEQ + cmd_hop.DESA + cmd_hop.SRCA + cmd_hop.DLEN + cmd_hop.INFO;
           SWHICH_HOP(hop_id,(uint8*)&cmd_hop);
            break;
        case SDP_CMD_STOP_CLR:
            cmd_hop.DLEN = 3;
            cmd_hop.INFO = SDP_CMD_STOP_CLR;
            cmd_hop.DAT[0] = 0;
            cmd_hop.DAT[1] = 0;
            cmd_hop.BCC = cmd_hop.RSEQ + cmd_hop.DESA + cmd_hop.SRCA + cmd_hop.DLEN + cmd_hop.INFO;
            SWHICH_HOP(hop_id,(uint8*)&cmd_hop);
            break;
        case SDP_CMD_TEST_SENSOR:
            cmd_hop.DLEN = 1;
            cmd_hop.INFO = SDP_CMD_TEST_SENSOR;
            cmd_hop.BCC = cmd_hop.RSEQ + cmd_hop.DESA + cmd_hop.SRCA + cmd_hop.DLEN + cmd_hop.INFO;
           SWHICH_HOP(hop_id,(uint8*)&cmd_hop);
            break;
        case SDP_CMD_HOPPER_VERSION:
            cmd_hop.DLEN = 1;
            cmd_hop.INFO = SDP_CMD_HOPPER_VERSION;
            cmd_hop.BCC = cmd_hop.RSEQ + cmd_hop.DESA + cmd_hop.SRCA + cmd_hop.DLEN + cmd_hop.INFO;
            SWHICH_HOP(hop_id,(uint8*)&cmd_hop);
            break;
        case SDP_CMD_HOPPER_RST:
            cmd_hop.DLEN = 1;
            cmd_hop.INFO = SDP_CMD_HOPPER_RST;
            cmd_hop.BCC = cmd_hop.RSEQ + cmd_hop.DESA + cmd_hop.SRCA + cmd_hop.DLEN + cmd_hop.INFO;
            SWHICH_HOP(hop_id,(uint8*)&cmd_hop);
            break;
        default:
            break;
    }
    if(cmd_hop.RSEQ == 0x7f)
        cmd_hop.RSEQ = 0;
}

