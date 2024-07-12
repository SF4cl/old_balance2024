#ifndef _BSP_CAN_H
#define _BSP_CAN_H

#include "main.h"
#include "3508Motor_drv.h"

extern void FDCAN1_Config(void);

typedef __PACKED_STRUCT
{
    uint8_t ChassisStatueRequest;
    int16_t FBSpeed;
    int16_t LRSpeed;
    int16_t Hight;
    uint8_t PTZStatusInformation;
}
PTZ_t;

#endif
