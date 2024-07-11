#include "motor_move.h"
#include "main.h"
#include "usart.h"
#include "freeRTOS.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "chassisRtask.h"
#include "chassisLtask.h"

 #define Nor_printf(...)  HAL_UART_Transmit(&huart1,\
 																				(uint8_t  *)u1_buf,\
 																				sprintf((char*)u1_buf,__VA_ARGS__),0xff)

 uint8_t u1_buf[80];

extern chassis_t chassis_move;
extern vmc_t vmc_R;
extern vmc_t vmc_L;
extern first_order_filter_type_t d_phi1_l_filter;
extern first_order_filter_type_t d_phi4_l_filter;
extern first_order_filter_type_t pitch_gyro_l_filter;
extern first_order_filter_type_t wheel_r_speed_filter;

extern INS_t INS;																				
																				
double flag=0;
void motorMoveTask(void *argument)
{

    while(1)
		{

//			if(flag == 0)
//				Nor_printf("%f, %f\r\n",wheel_r_speed_filter.out, chassis_move.wheel_motor[1].para.speed);

			
        osDelay(1);
    }
}