#include "A1Motor_drv.h"

uint8_t SendData[34],RecvData[78];
uint32_t crcdata[7];

USART_SENDRECEIVETYPE Usart2Type __attribute__((section(".ARM.__at_0x24000000")));
USART_SENDRECEIVETYPE Usart3Type __attribute__((section(".ARM.__at_0x24000000")));
uint8_t rx_buffer[50] __attribute__((section(".ARM.__at_0x24000000")));

MOTOR_send LFMotor_Send;
MOTOR_send LBMotor_Send;
MOTOR_send RFMotor_Send;
MOTOR_send RBMotor_Send;
MOTOR_recv LFMotor_Recv;
MOTOR_recv LBMotor_Recv;
MOTOR_recv RFMotor_Recv;
MOTOR_recv RBMotor_Recv;

void joint_motor_set_mode(int MODE)
{	
	
  LFMotor_Send.mode = MODE;
	LFMotor_Send.id = LFMotor_ID;
	LFMotor_Recv.motor_id = LFMotor_ID;
	
	LBMotor_Send.mode = MODE;
	LBMotor_Send.id = LBMotor_ID;
	LBMotor_Recv.motor_id = LBMotor_ID;

	RFMotor_Send.mode = MODE;
	RFMotor_Send.id = RFMotor_ID;
	RFMotor_Recv.motor_id = RFMotor_ID;

	RBMotor_Send.mode = MODE;
	RBMotor_Send.id = RBMotor_ID;
	RBMotor_Recv.motor_id = RBMotor_ID;
}

void LFMotor_T(double T)
{
	LFMotor_Send.K_P=0;
	LFMotor_Send.K_W=0;
	LFMotor_Send.Pos=0;
	LFMotor_Send.T=T;
	modify_data(&LFMotor_Send);
	memcpy(Usart2Type.TX_pData,(uint8_t*)&(LFMotor_Send.motor_send_data),SEND_SIZE);
	SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart2Type.TX_pData),SEND_SIZE);	
	HAL_UART_Transmit_DMA(&huart2,Usart2Type.TX_pData,SEND_SIZE);      
}

void LBMotor_T(double T)
{
	LBMotor_Send.K_P=0;
	LBMotor_Send.K_W=0;
	LBMotor_Send.Pos=0;
	LBMotor_Send.T=T;
	modify_data(&LBMotor_Send);
	memcpy(Usart2Type.TX_pData,(uint8_t*)&(LBMotor_Send.motor_send_data),SEND_SIZE);
	SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart2Type.TX_pData),SEND_SIZE);
	HAL_UART_Transmit_DMA(&huart2,Usart2Type.TX_pData,SEND_SIZE);     
}

void RFMotor_T(double T)
{
	RFMotor_Send.K_P=0;
	RFMotor_Send.K_W=0;
	RFMotor_Send.Pos=0;
	RFMotor_Send.T=T;
	modify_data(&RFMotor_Send);
	memcpy(Usart3Type.TX_pData,(uint8_t*)&(RFMotor_Send.motor_send_data),SEND_SIZE);
	SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart3Type.TX_pData),SEND_SIZE);
	HAL_UART_Transmit_DMA(&huart3,Usart3Type.TX_pData,SEND_SIZE);                                                                                            
}

void RBMotor_T(double T)
{
	RBMotor_Send.K_P=0;
	RBMotor_Send.K_W=0;
	RBMotor_Send.Pos=0;
	RBMotor_Send.T=T;
	modify_data(&RBMotor_Send);
	memcpy(Usart3Type.TX_pData,(uint8_t*)&(RBMotor_Send.motor_send_data),SEND_SIZE);
	SCB_InvalidateDCache_by_Addr((uint8_t*)&(Usart3Type.TX_pData),SEND_SIZE);
	HAL_UART_Transmit_DMA(&huart3,Usart3Type.TX_pData,SEND_SIZE);                                                                                            
}

void modify_data(MOTOR_send *motor_s)
{
    motor_s->hex_len = 34;
    motor_s->motor_send_data.head.start[0] = 0xFE;
    motor_s->motor_send_data.head.start[1] = 0xEE;
    motor_s->motor_send_data.head.motorID = motor_s->id;
    motor_s->motor_send_data.head.reserved = 0x0;
    motor_s->motor_send_data.Mdata.mode = motor_s->mode;
    motor_s->motor_send_data.Mdata.ModifyBit = 0xFF;
    motor_s->motor_send_data.Mdata.ReadBit = 0x0;
    motor_s->motor_send_data.Mdata.reserved = 0x0;
    motor_s->motor_send_data.Mdata.Modify.L = 0;
    motor_s->motor_send_data.Mdata.T = motor_s->T*256;
    motor_s->motor_send_data.Mdata.W = motor_s->W*128;
    motor_s->motor_send_data.Mdata.Pos = (float)((motor_s->Pos/6.2832)*16384.0);
    motor_s->motor_send_data.Mdata.K_P = motor_s->K_P*2048;
    motor_s->motor_send_data.Mdata.K_W = motor_s->K_W*1024;
    
    motor_s->motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
    motor_s->motor_send_data.Mdata.LowHzMotorCmdByte = 0;
    motor_s->motor_send_data.Mdata.Res[0] = motor_s->Res;
    motor_s->motor_send_data.CRCdata.u32 = crc32_core((uint32_t*)(&(motor_s->motor_send_data)), 7);
}

bool extract_data(uint8_t id,MOTOR_recv* motor_r)
{
		if(id==motor_r->motor_recv_data.head.motorID)
		{
			motor_r->mode = motor_r->motor_recv_data.Mdata.mode;
			motor_r->Temp = motor_r->motor_recv_data.Mdata.Temp;
			motor_r->MError = motor_r->motor_recv_data.Mdata.MError;
			motor_r->T = (float)(((float)motor_r->motor_recv_data.Mdata.T) / 256.0f);
			motor_r->W = (float)(((float)motor_r->motor_recv_data.Mdata.W) / 128.0f);
			motor_r->LW = motor_r->motor_recv_data.Mdata.LW;

			motor_r->Acc = (int)motor_r->motor_recv_data.Mdata.Acc;
			motor_r->Pos = 6.2832f*((float)motor_r->motor_recv_data.Mdata.Pos) / 16384.0f;
			
			motor_r->gyro[0] = (float)(((float)motor_r->motor_recv_data.Mdata.gyro[0]) * 0.00107993176f);
			motor_r->gyro[1] = (float)(((float)motor_r->motor_recv_data.Mdata.gyro[1]) * 0.00107993176f);
			motor_r->gyro[2] = (float)(((float)motor_r->motor_recv_data.Mdata.gyro[2]) * 0.00107993176f);
			
			motor_r->acc[0] = (float)(((float)motor_r->motor_recv_data.Mdata.acc[0]) * 0.0023911132f);
			motor_r->acc[1] = (float)(((float)motor_r->motor_recv_data.Mdata.acc[1]) * 0.0023911132f);
			motor_r->acc[2] = (float)(((float)motor_r->motor_recv_data.Mdata.acc[2]) * 0.0023911132f);		
			return true;
		}

		return false;
}
