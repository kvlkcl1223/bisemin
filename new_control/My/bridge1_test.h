#ifndef __BRIDGE1_TEST_H
#define __BRIDGE1_TEST_H

#include "main.h"
#include <stdint.h>

typedef enum
{
    BRIDGE1_OK = 0,
    BRIDGE1_FAULT_ACTIVE,
    BRIDGE1_WDFLT_ACTIVE,
    BRIDGE1_PARAM_ERROR
} Bridge1_Status_t;

void Bridge1_InitSafeState(void);
void Bridge1_SetVref_mV(uint16_t mv);
void Bridge1_Enable(void);
void Bridge1_Disable(void);
void Bridge1_SetDuty(float duty);
Bridge1_Status_t Bridge1_CheckFault(void);
Bridge1_Status_t Bridge1_RunBasicTest(void);

/* INA240A1 + 5mΩ 采样电阻时使用 */
float Bridge1_CurrentFromAdcRaw(uint16_t adc_raw, float vdda, float zero_voltage);
Bridge1_Status_t Bridge1_StartHoldTest(float duty, uint16_t vref_mv);
void Bridge1_StopHoldTest(void);
#endif