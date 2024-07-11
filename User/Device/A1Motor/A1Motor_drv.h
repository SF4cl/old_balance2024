#ifndef A1MOTOR_DRV_H
#define A1MOTOR_DRV_H
#include "unitreeMotor.h"
#include "crc32.h"  
#include "string.h"
#include "usart.h"
#include "stm32h7xx_it.h"

#define RECEIVE_SIZE					                                    78
#define SEND_SIZE							                                34

#define RFMotor_ID 															0
#define RBMotor_ID 															2
#define LFMotor_ID 														    0
#define LBMotor_ID 															1

typedef struct
{
	uint8_t RX_flag:1;//IDLE receive flag

	uint16_t RX_Size;//receive length

	uint8_t RX_pData[RECEIVE_SIZE];//DMA receive buffer

	uint8_t TX_pData[SEND_SIZE];
}USART_SENDRECEIVETYPE;

extern USART_SENDRECEIVETYPE Usart2Type;
extern USART_SENDRECEIVETYPE Usart3Type;
extern uint8_t rx_buffer[50];

extern MOTOR_send LFMotor_Send;
extern MOTOR_send LBMotor_Send;
extern MOTOR_send RFMotor_Send;
extern MOTOR_send RBMotor_Send;
extern MOTOR_recv LFMotor_Recv;
extern MOTOR_recv LBMotor_Recv;
extern MOTOR_recv RFMotor_Recv;
extern MOTOR_recv RBMotor_Recv;

extern uint8_t SendData[34],RecvData[78];
extern void joint_motor_set_mode(int MODE);
extern void LFMotor_T(double T);
extern void LBMotor_T(double T);
extern void RFMotor_T(double T);
extern void RBMotor_T(double T);
extern void modify_data(MOTOR_send *motor_s);
extern bool extract_data(uint8_t id,MOTOR_recv* motor_r);

#endif

