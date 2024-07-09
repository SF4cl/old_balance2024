#include "bsp_can.h"
#include "fdcan.h"
#include "string.h"
#include "chassisRtask.h"

FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

void FDCAN1_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;

  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x0000;
  sFilterConfig.FilterID2 = 0x0000;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

extern chassis_t chassis_move;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
   uint16_t tmp_value;

   if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET){
       if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){
           Error_Handler();
       }
       if(hfdcan==&hfdcan1)
			{
					switch(RxHeader.Identifier)
					{
							case FEET_MOTOR1_RECEIVE_ID:
							{
									get_motor_measure(&LeftFootMotorMeasure,RxData);
									//dx_left=((float)LeftFootMotorMeasure.speed_rpm/19.2f/60.0f)*R*3.14159f*2.0f/*+l0_left*da_left*arm_cos_f32(a_left)+da_left*arm_sin_f32(a_left)-GyroCorrected[2]*R*/;
									
									break;
							}
							case FEET_MOTOR2_RECEIVE_ID:
							{		
									get_motor_measure(&RightFootMotorMeasure,RxData);		
									//dx_right=-((float)RightFootMotorMeasure.speed_rpm/19.2f/60.0f)*R*3.14159f*2.0f/*+l0_right*da_right*arm_cos_f32(a_right)+da_right*arm_sin_f32(a_right)-GyroCorrected[2]*R*/;
									
									break;
							}				
					}
				}
   }
}
