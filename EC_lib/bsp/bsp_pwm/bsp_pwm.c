/**
 * @Author       : Specific-Cola specificcola@proton.me
 * @Date         : 2024-03-22 23:03:00
 * @LastEditors  : H0pefu12 573341043@qq.com
 * @LastEditTime : 2024-04-17 02:56:03
 * @Description  :
 * @Filename     : bsp_pwm.c
 * @Copyright (c) 2024 by IRobot, All Rights Reserved.
 * @
 */
#include "bsp_pwm.h"

#include <stdlib.h>
#include <string.h>

#include "main.h"

#if defined(STM32F407xx) || defined(STM32F427xx)
#define PCLK1_MULT (APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos] == 0 ? 1 : 2)
#define PCLK2_MULT (APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos] == 0 ? 1 : 2)
#elif defined(STM32H723xx)
#define PCLK1_MULT (D1CorePrescTable[(RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos] == 0 ? 1 : 2)
#define PCLK2_MULT (D1CorePrescTable[(RCC->D2CFGR & RCC_D2CFGR_D2PPRE2) >> RCC_D2CFGR_D2PPRE2_Pos] == 0 ? 1 : 2)
#endif

static PWM_Device_t* pwm_instance[PWM_DEVICE_CNT] = {NULL};
static uint8_t id_cnt;
static uint32_t PWMSelectTclk(TIM_HandleTypeDef* htim);  // 通过与APB1和APB2比较来猜测pwm通道的时钟频率

static uint32_t PWMSelectTclk(TIM_HandleTypeDef* htim) {
    uintptr_t tclk_temp = ((uintptr_t)((htim)->Instance));
    if ((tclk_temp <= (APB1PERIPH_BASE + 0x2000UL)) && (tclk_temp >= (APB1PERIPH_BASE + 0x0000UL))) {
        return (HAL_RCC_GetPCLK1Freq() * PCLK1_MULT);
    } else if (((tclk_temp <= (APB2PERIPH_BASE + 0x0400UL)) && (tclk_temp >= (APB2PERIPH_BASE + 0x0000UL))) ||
               ((tclk_temp <= (APB2PERIPH_BASE + 0x4800UL)) && (tclk_temp >= (APB2PERIPH_BASE + 0x4000UL)))) {
        return (HAL_RCC_GetPCLK2Freq() * PCLK2_MULT);
    }
    return 0;
}

PWM_Device_t* pwmRegister(PWM_Register_t* config) {
    if (id_cnt >= PWM_DEVICE_CNT)  // 超过最大实例数,考虑增加或查看是否有内存泄漏
        while (1)
            ;
    PWM_Device_t* pwm = (PWM_Device_t*)malloc(sizeof(PWM_Device_t));
    memset(pwm, 0, sizeof(PWM_Device_t));

    pwm->htim = config->htim;
    pwm->channel = config->channel;
    pwm->period = config->period;
    pwm->dutyratio = config->dutyratio;
    pwm->callback = config->callback;
    pwm->parent = config->parent;
    pwm->tclk = PWMSelectTclk(pwm->htim);
    // 启动PWM
    pwmOnActivate(pwm);
    pwmSetPeriod(pwm, pwm->period);
    pwmSetDuty(pwm, pwm->dutyratio);
    pwm_instance[id_cnt++] = pwm;
    return pwm;
}

void pwmOnActivate(PWM_Device_t* pwm) { HAL_TIM_PWM_Start(pwm->htim, pwm->channel); }
void pwmOnDeactivate(PWM_Device_t* pwm) { HAL_TIM_PWM_Stop(pwm->htim, pwm->channel); }

void pwmSetPeriod(PWM_Device_t* pwm, fp32 period) {
    __HAL_TIM_SetAutoreload(pwm->htim, period * ((pwm->tclk) / (pwm->htim->Init.Prescaler + 1)) / 1000);
}
void pwmSetDuty(PWM_Device_t* pwm, float dutyratio) {
    __HAL_TIM_SetCompare(pwm->htim, pwm->channel, dutyratio * (pwm->htim->Instance->ARR));
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim) {
    for (uint8_t i = 0; i < id_cnt; i++) {  // 来自同一个定时器的中断且通道相同
        if (pwm_instance[i]->htim == htim && htim->Channel == (1 << (pwm_instance[i]->channel / 4))) {
            if (pwm_instance[i]->callback)  // 如果有回调函数
                pwm_instance[i]->callback(pwm_instance[i]);
            return;  // 一次只能有一个通道的中断,所以直接返回
        }
    }
}
void PWMStartDMA(PWM_Device_t* pwm, uint32_t* pData, uint32_t Size)  // 一般使用用来点亮ws2812灯带
{
    HAL_TIM_PWM_Start_DMA(pwm->htim, pwm->channel, pData, Size);
}