#include "3508Motor_drv.h"

FDCAN_TxHeaderTypeDef  can_tx_message;
uint8_t              MotorSendBuffer[8];

feet_motor_measure_t LeftFootMotorMeasure;
feet_motor_measure_t RightFootMotorMeasure;
feet_motor_measure_t YawMotorMeasure;

void FEET_CONTROL(int16_t FEET_MOTOR_LEFT, int16_t FEET_MOTOR_RIGHT)
{
		
    MotorSendBuffer[0] = FEET_MOTOR_LEFT >> 8;
    MotorSendBuffer[1] = FEET_MOTOR_LEFT;
    MotorSendBuffer[2] = FEET_MOTOR_RIGHT >> 8;
    MotorSendBuffer[3] = FEET_MOTOR_RIGHT;
  

    can_tx_message.Identifier=0x200;                           //32λID
    can_tx_message.IdType=FDCAN_STANDARD_ID;                  //��׼ID
    can_tx_message.TxFrameType=FDCAN_DATA_FRAME;              //����֡
    can_tx_message.DataLength=FDCAN_DLC_BYTES_8;              //���ݳ���
    can_tx_message.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
    can_tx_message.BitRateSwitch=FDCAN_BRS_OFF;               //�ر������л�
    can_tx_message.FDFormat=FDCAN_CLASSIC_CAN;                //��ͳ��CANģʽ
    can_tx_message.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //�޷����¼�
    can_tx_message.MessageMarker=0;      

    
    /* ������ݵ�TX FIFO */
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_message, MotorSendBuffer);
		
		
		
} 



