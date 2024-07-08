#ifndef __MOTOR_MOVE_H
#define __MOTOR_MOVE_H

#include <stdint.h>
#include <stddef.h>
#include "A1Motor_drv.h"
#include "stm32h7xx_it.h"
#include "struct_typedef.h"
#include "main.h"
#include "usart.h"
#include "freeRTOS.h"

void motorMoveTask(void *argument);

#endif
