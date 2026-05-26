#include "bridge1_test.h"
#include "opamp.h"
#include "adc.h"
#include "dac.h"
/*
 * 需要确认：
 * 1. PWM1 = PE9 时，通常是 TIM1_CH1。
 * 2. 如果你 CubeMX 里 PWM1 不是 htim1 / TIM_CHANNEL_1，改这里。
 */
extern TIM_HandleTypeDef htim1;
extern DAC_HandleTypeDef hdac3;

/* ======== 用户可改配置 ======== */

#define BRIDGE1_PWM_TIM_HANDLE htim1
#define BRIDGE1_PWM_CHANNEL TIM_CHANNEL_1

/* DRV8703 VREF 接 DAC1，PA2 */
#define BRIDGE1_USE_DAC_VREF 1
#define BRIDGE1_DAC_HANDLE hdac1
#define BRIDGE1_DAC_CHANNEL DAC_CHANNEL_1

/* 测试占空比，第一次不要大 */
#define BRIDGE1_TEST_DUTY 0.03f     /* 3% */
#define BRIDGE1_MAX_TEST_DUTY 0.05f /* 测试阶段最大 5% */

/* INA240A1 参数：5mΩ，增益20，输出系数 = 0.1 V/A */
#define BRIDGE1_INA_RSHUNT_OHM 0.005f
#define BRIDGE1_INA_GAIN 20.0f

/* ======== 延时兼容 FreeRTOS / 裸机 ======== */
#ifdef CMSIS_OS_H_
#define BRIDGE_DELAY_MS(ms) osDelay(ms)
#else
#define BRIDGE_DELAY_MS(ms) HAL_Delay(ms)
#endif

static float Bridge1_LimitFloat(float x, float min, float max)
{
    if (x > max)
        return max;
    if (x < min)
        return min;
    return x;
}

static void Bridge1_SetPwmRaw(uint32_t compare)
{
    __HAL_TIM_SET_COMPARE(&BRIDGE1_PWM_TIM_HANDLE, BRIDGE1_PWM_CHANNEL, compare);
}

static uint32_t Bridge1_DutyToCompare(float duty_abs)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&BRIDGE1_PWM_TIM_HANDLE);
    duty_abs = Bridge1_LimitFloat(duty_abs, 0.0f, 0.95f);
    return (uint32_t)((float)(arr + 1U) * duty_abs);
}

/*
 * 上电安全状态：
 * - PWM=0
 * - DIR=0
 * - CS=1
 * - nSLEEP=0
 */
void Bridge1_InitSafeState(void)
{
    HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SLEEP1_GPIO_Port, SLEEP1_Pin, GPIO_PIN_RESET);

    Bridge1_SetPwmRaw(0);

#if BRIDGE1_USE_DAC_VREF
    HAL_DAC_Start(&BRIDGE1_DAC_HANDLE, BRIDGE1_DAC_CHANNEL);
    Bridge1_SetVref_mV(1000); /* 默认 1.0V，约对应 10A，取决于 Rshunt 和增益 */
    HAL_OPAMP_Start(&hopamp1);
#endif
}

/*
 * 设置 DRV8703 VREF。
 * 如果 Rshunt_DRV = 5mΩ，DRV8703 电流采样增益约 20，
 * 那么 VREF=1000mV 大约对应 10A 斩波限流。
 */
void Bridge1_SetVref_mV(uint16_t mv)
{
#if BRIDGE1_USE_DAC_VREF
    if (mv > 3300U)
    {
        mv = 3300U;
    }

    uint32_t dac_value = ((uint32_t)mv * 4095U) / 3300U;
    HAL_DAC_SetValue(&BRIDGE1_DAC_HANDLE,
                     BRIDGE1_DAC_CHANNEL,
                     DAC_ALIGN_12B_R,
                     dac_value);
#else
    (void)mv;
#endif
}

/*
 * 释放 DRV8703。
 * 注意：释放后先等 5~10ms，再输出 PWM。
 */
void Bridge1_Enable(void)
{
    Bridge1_SetPwmRaw(0);
    HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(SLEEP1_GPIO_Port, SLEEP1_Pin, GPIO_PIN_SET);

    BRIDGE_DELAY_MS(10);
}

/*
 * 关闭 DRV8703。
 */
void Bridge1_Disable(void)
{
    Bridge1_SetPwmRaw(0);
    HAL_GPIO_WritePin(SLEEP1_GPIO_Port, SLEEP1_Pin, GPIO_PIN_RESET);
}

/*
 * duty 范围：
 * +0.03 表示正向 3%
 * -0.03 表示反向 3%
 *  0    表示停止
 *
 * DRV8703 PH/EN 模式：
 * DIR1 = PH
 * PWM1 = EN
 */
void Bridge1_SetDuty(float duty)
{
    duty = Bridge1_LimitFloat(duty, -0.5f, 0.5f);

    if (duty > 0.0f)
    {
        HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_SET);
        Bridge1_SetPwmRaw(Bridge1_DutyToCompare(duty));
    }
    else if (duty < 0.0f)
    {
        HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);
        Bridge1_SetPwmRaw(Bridge1_DutyToCompare(-duty));
    }
    else
    {
        Bridge1_SetPwmRaw(0);
    }
}

/*
 * nFAULT / nWDFLT 都是低有效。
 */
Bridge1_Status_t Bridge1_CheckFault(void)
{
    if (HAL_GPIO_ReadPin(FAULT1_GPIO_Port, FAULT1_Pin) == GPIO_PIN_RESET)
    {
        Bridge1_SetDuty(0.0f);
        return BRIDGE1_FAULT_ACTIVE;
    }

    if (HAL_GPIO_ReadPin(WDFLT1_GPIO_Port, WDFLT1_Pin) == GPIO_PIN_RESET)
    {
        Bridge1_SetDuty(0.0f);
        return BRIDGE1_WDFLT_ACTIVE;
    }

    return BRIDGE1_OK;
}

/*
 * 第一路基础测试：
 * 1. 安全关断
 * 2. 设置 VREF
 * 3. 启动 PWM 定时器
 * 4. 释放 nSLEEP
 * 5. 正向 3%
 * 6. 停止
 * 7. 反向 3%
 * 8. 停止
 */
Bridge1_Status_t Bridge1_RunBasicTest(void)
{
    Bridge1_Status_t status;

    Bridge1_InitSafeState();
    BRIDGE_DELAY_MS(100);

    /* 第一次建议 VREF 先设低一点，例如 800~1000mV */
    Bridge1_SetVref_mV(1000);

    /*
     * 启动 PWM。
     * 如果你已经在别处启动过 TIM1_CH1，可以删掉这一句。
     */
    HAL_TIM_PWM_Start(&BRIDGE1_PWM_TIM_HANDLE, BRIDGE1_PWM_CHANNEL);
    Bridge1_SetPwmRaw(0);

    Bridge1_Enable();

    status = Bridge1_CheckFault();
    if (status != BRIDGE1_OK)
    {
        Bridge1_Disable();
        return status;
    }

    /* 正向小占空比测试 */
    Bridge1_SetDuty(BRIDGE1_TEST_DUTY);
    BRIDGE_DELAY_MS(1000);

    status = Bridge1_CheckFault();
    Bridge1_SetDuty(0.0f);
    BRIDGE_DELAY_MS(500);

    if (status != BRIDGE1_OK)
    {
        Bridge1_Disable();
        return status;
    }

    /* 反向小占空比测试 */
    Bridge1_SetDuty(-BRIDGE1_TEST_DUTY);
    BRIDGE_DELAY_MS(1000);

    status = Bridge1_CheckFault();
    Bridge1_SetDuty(0.0f);
    BRIDGE_DELAY_MS(500);

    Bridge1_Disable();

    return status;
}

/*
 * INA240A1 电流换算：
 * Rshunt = 5mΩ
 * Gain = 20
 * Vout = Vzero + I * 0.005 * 20
 * I = (Vout - Vzero) / 0.1
 */
float Bridge1_CurrentFromAdcRaw(uint16_t adc_raw, float vdda, float zero_voltage)
{
    float vadc = ((float)adc_raw * vdda) / 4095.0f;
    float current = (vadc - zero_voltage) / (BRIDGE1_INA_RSHUNT_OHM * BRIDGE1_INA_GAIN);
    return current;
}

Bridge1_Status_t Bridge1_StartHoldTest(float duty, uint16_t vref_mv)
{
    Bridge1_Status_t status;

    /*
     * duty:
     *   +0.03 表示正向 3%
     *   -0.03 表示反向 3%
     *
     * 第一次测试建议：
     *   没焊 MOS：可以 3%~10%
     *   焊了 MOS：先 1%~3%
     */
    duty = Bridge1_LimitFloat(duty, -BRIDGE1_MAX_TEST_DUTY, BRIDGE1_MAX_TEST_DUTY);

    /* 先进入安全状态 */
    Bridge1_InitSafeState();
    BRIDGE_DELAY_MS(50);

    /* 设置 DRV8703 的 VREF，例如 1000mV */
    Bridge1_SetVref_mV(vref_mv);
    BRIDGE_DELAY_MS(10);

    /* 启动 PWM 定时器，但此时比较值仍为 0 */
    HAL_TIM_PWM_Start(&BRIDGE1_PWM_TIM_HANDLE, BRIDGE1_PWM_CHANNEL);
    Bridge1_SetPwmRaw(0);

    /* 拉高 nSLEEP，释放 DRV8703 */
    HAL_GPIO_WritePin(SLEEP1_GPIO_Port, SLEEP1_Pin, GPIO_PIN_SET);

    /*
     * DRV8703 从 sleep 唤醒需要一点时间。
     * 不要一拉高 nSLEEP 就马上给 PWM。
     */
    BRIDGE_DELAY_MS(10);

    /* 检查故障脚 */
    status = Bridge1_CheckFault();
    if (status != BRIDGE1_OK)
    {
        Bridge1_SetDuty(0.0f);
        HAL_GPIO_WritePin(SLEEP1_GPIO_Port, SLEEP1_Pin, GPIO_PIN_RESET);
        return status;
    }

    /* 正式输出固定占空比 PWM，并保持 */
    Bridge1_SetDuty(duty);

    /*
     * 再等一小段时间检查是否触发故障。
     * 如果无故障，函数返回后 PWM 会继续输出。
     */
    BRIDGE_DELAY_MS(20);

    status = Bridge1_CheckFault();
    if (status != BRIDGE1_OK)
    {
        Bridge1_SetDuty(0.0f);
        HAL_GPIO_WritePin(SLEEP1_GPIO_Port, SLEEP1_Pin, GPIO_PIN_RESET);
        return status;
    }

    return BRIDGE1_OK;
}
void Bridge1_StopHoldTest(void)
{
    Bridge1_SetDuty(0.0f);
    BRIDGE_DELAY_MS(5);

    HAL_GPIO_WritePin(SLEEP1_GPIO_Port, SLEEP1_Pin, GPIO_PIN_RESET);
}