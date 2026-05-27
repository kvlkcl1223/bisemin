/*
 * adc1_voltage_current_test.c
 *
 * Purpose:
 *   Test ADC1 voltage/current acquisition on STM32G474.
 *
 * Current CubeMX pin labels from your screenshot:
 *
 *   PA0  ADC1_IN1   ADC_V1
 *   PA1  ADC1_IN2   ADC_V2
 *   PA3  ADC1_IN4   ADC_V3
 *   PB14 ADC1_IN5   ADC_V4
 *   PC0  ADC1_IN6   ADC_V5
 *
 *   PC1  ADC1_IN7   ADC_I1
 *   PC2  ADC1_IN8   ADC_I2
 *   PC3  ADC1_IN9   ADC_I3
 *   PB11 ADC1_IN14  ADC_I4
 *   PB0  ADC1_IN15  ADC_I5
 *
 * How to use:
 *   1. Add this .c file into Keil project.
 *   2. Add the function declarations you use to your own header or app_freertos.c:
 *
 *        void ADC1_Test_Init(void);
 *        void ADC1_Test_CalibrateCurrentZero(uint16_t samples);
 *        void ADC1_Test_RunOnce(void);
 *
 *   3. In a FreeRTOS task:
 *
 *        ADC1_Test_Init();
 *        ADC1_Test_CalibrateCurrentZero(64);   // no current / no load state
 *
 *        for (;;) {
 *            ADC1_Test_RunOnce();
 *            osDelay(200);
 *        }
 *
 *   4. Watch these variables in Keil:
 *
 *        g_adc_v1_input_v ... g_adc_v5_input_v
 *        g_adc_i1_a       ... g_adc_i5_a
 *        g_adc_v1_raw     ... g_adc_i5_raw
 *
 * Important:
 *   - This file reads channels one by one using HAL_ADC_ConfigChannel().
 *     It does not depend on your CubeMX regular-rank order.
 *   - Do not run this test simultaneously with another ADC1 DMA/interrupt task.
 *   - Before current zero calibration, make sure current is really 0 A.
 */

#include "main.h"
#include "adc.h"
#include "cmsis_os.h"
#include <stdint.h>

/* ===================== User adjustable parameters ===================== */

/*
 * ADC reference voltage.
 * If VDDA is not exactly 3.300 V, change this value.
 * For higher precision, enable VREFINT and calculate VDDA dynamically later.
 */
#define ADC1_TEST_VDDA_V                  (3.300f)

/*
 * ADC resolution.
 * STM32G4 ADC is normally 12-bit in CubeMX default.
 */
#define ADC1_TEST_ADC_FULL_SCALE           (4095.0f)

/*
 * Voltage divider ratio:
 *
 *   Vin ---- R_upper ---- ADC pin ---- R_lower ---- AGND
 *
 *   Vin = Vadc * (R_upper + R_lower) / R_lower
 *
 * In your earlier schematic, voltage divider looked like 150 k / 10 k:
 *   ratio = (150k + 10k) / 10k = 16
 *
 * If your actual divider is different, change these values.
 */
#define ADC1_TEST_VOLT_RATIO_V1            (16.0f)
#define ADC1_TEST_VOLT_RATIO_V2            (16.0f)
#define ADC1_TEST_VOLT_RATIO_V3            (16.0f)
#define ADC1_TEST_VOLT_RATIO_V4            (16.0f)
#define ADC1_TEST_VOLT_RATIO_V5            (16.0f)

/*
 * INA240A1 current measurement:
 *
 *   Vout = Vzero + I * Rshunt * Gain
 *   I    = (Vout - Vzero) / (Rshunt * Gain)
 *
 * INA240A1 gain = 20 V/V.
 *
 * If your current-sampling shunt is 5 mΩ, keep 0.005f.
 * If you are using 50 mΩ for this ADC current measurement, change it to 0.050f.
 */
#define ADC1_TEST_INA_GAIN                 (20.0f)

#define ADC1_TEST_RSHUNT_I1_OHM            (0.005f)
#define ADC1_TEST_RSHUNT_I2_OHM            (0.005f)
#define ADC1_TEST_RSHUNT_I3_OHM            (0.005f)
#define ADC1_TEST_RSHUNT_I4_OHM            (0.005f)
#define ADC1_TEST_RSHUNT_I5_OHM            (0.005f)

/*
 * ADC sampling time.
 * Use longer sampling time for high-impedance voltage dividers.
 */
#define ADC1_TEST_SAMPLING_TIME            ADC_SAMPLETIME_247CYCLES_5

/* Average samples for each channel during normal acquisition. */
#define ADC1_TEST_AVG_SAMPLES              (16U)

/* ===================== Keil Watch variables ===================== */

/* Raw ADC values */
volatile uint16_t g_adc_v1_raw = 0;
volatile uint16_t g_adc_v2_raw = 0;
volatile uint16_t g_adc_v3_raw = 0;
volatile uint16_t g_adc_v4_raw = 0;
volatile uint16_t g_adc_v5_raw = 0;

volatile uint16_t g_adc_i1_raw = 0;
volatile uint16_t g_adc_i2_raw = 0;
volatile uint16_t g_adc_i3_raw = 0;
volatile uint16_t g_adc_i4_raw = 0;
volatile uint16_t g_adc_i5_raw = 0;

/* ADC pin voltages */
volatile float g_adc_v1_adc_v = 0.0f;
volatile float g_adc_v2_adc_v = 0.0f;
volatile float g_adc_v3_adc_v = 0.0f;
volatile float g_adc_v4_adc_v = 0.0f;
volatile float g_adc_v5_adc_v = 0.0f;

volatile float g_adc_i1_adc_v = 0.0f;
volatile float g_adc_i2_adc_v = 0.0f;
volatile float g_adc_i3_adc_v = 0.0f;
volatile float g_adc_i4_adc_v = 0.0f;
volatile float g_adc_i5_adc_v = 0.0f;

/* Reconstructed external voltages */
volatile float g_adc_v1_input_v = 0.0f;
volatile float g_adc_v2_input_v = 0.0f;
volatile float g_adc_v3_input_v = 0.0f;
volatile float g_adc_v4_input_v = 0.0f;
volatile float g_adc_v5_input_v = 0.0f;

/* Current zero offsets, calibrated at 0 A */
volatile float g_adc_i1_zero_v = ADC1_TEST_VDDA_V * 0.5f;
volatile float g_adc_i2_zero_v = ADC1_TEST_VDDA_V * 0.5f;
volatile float g_adc_i3_zero_v = ADC1_TEST_VDDA_V * 0.5f;
volatile float g_adc_i4_zero_v = ADC1_TEST_VDDA_V * 0.5f;
volatile float g_adc_i5_zero_v = ADC1_TEST_VDDA_V * 0.5f;

/* Calculated currents */
volatile float g_adc_i1_a = 0.0f;
volatile float g_adc_i2_a = 0.0f;
volatile float g_adc_i3_a = 0.0f;
volatile float g_adc_i4_a = 0.0f;
volatile float g_adc_i5_a = 0.0f;

/* Debug status */
volatile HAL_StatusTypeDef g_adc1_test_last_hal_status = HAL_OK;
volatile uint32_t g_adc1_test_error_count = 0;

/* ===================== Internal helper functions ===================== */

static void ADC1_Test_DelayMs(uint32_t ms)
{
    /*
     * This file is intended for FreeRTOS task usage.
     * If you call it before scheduler starts, replace osDelay(ms) by HAL_Delay(ms).
     */
    osDelay(ms);
}

static float ADC1_Test_RawToVoltage(uint16_t raw)
{
    return ((float)raw * ADC1_TEST_VDDA_V) / ADC1_TEST_ADC_FULL_SCALE;
}

static uint16_t ADC1_Test_ReadChannelRaw(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint32_t sum = 0;
    uint32_t i;
    HAL_StatusTypeDef ret;

    /*
     * Stop ADC before changing channel.
     * This avoids rank/channel residue from previous conversion.
     */
    HAL_ADC_Stop(&hadc1);

    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC1_TEST_SAMPLING_TIME;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    ret = HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    g_adc1_test_last_hal_status = ret;
    if (ret != HAL_OK)
    {
        g_adc1_test_error_count++;
        return 0;
    }

    for (i = 0; i < ADC1_TEST_AVG_SAMPLES; i++)
    {
        ret = HAL_ADC_Start(&hadc1);
        g_adc1_test_last_hal_status = ret;
        if (ret != HAL_OK)
        {
            g_adc1_test_error_count++;
            continue;
        }

        ret = HAL_ADC_PollForConversion(&hadc1, 10);
        g_adc1_test_last_hal_status = ret;
        if (ret == HAL_OK)
        {
            sum += HAL_ADC_GetValue(&hadc1);
        }
        else
        {
            g_adc1_test_error_count++;
        }

        HAL_ADC_Stop(&hadc1);
    }

    return (uint16_t)(sum / ADC1_TEST_AVG_SAMPLES);
}

static float ADC1_Test_CurrentFromVoltage(float vout, float vzero, float rshunt_ohm)
{
    return (vout - vzero) / (rshunt_ohm * ADC1_TEST_INA_GAIN);
}

/* ===================== Public test functions ===================== */

void ADC1_Test_Init(void)
{
    /*
     * Calibrate ADC1.
     * Call this once in a task before ADC1_Test_RunOnce().
     */
    HAL_ADC_Stop(&hadc1);

    g_adc1_test_last_hal_status = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    if (g_adc1_test_last_hal_status != HAL_OK)
    {
        g_adc1_test_error_count++;
    }

    ADC1_Test_DelayMs(10);
}

void ADC1_Test_CalibrateCurrentZero(uint16_t samples)
{
    uint32_t i;
    float sum_i1 = 0.0f;
    float sum_i2 = 0.0f;
    float sum_i3 = 0.0f;
    float sum_i4 = 0.0f;
    float sum_i5 = 0.0f;

    if (samples == 0U)
    {
        samples = 1U;
    }

    /*
     * Calibration condition:
     *   - H bridges disabled or PWM = 0
     *   - no load current
     *   - INA240 outputs should be near VDDA/2
     */
    for (i = 0; i < samples; i++)
    {
        uint16_t raw_i1 = ADC1_Test_ReadChannelRaw(ADC_CHANNEL_7);
        uint16_t raw_i2 = ADC1_Test_ReadChannelRaw(ADC_CHANNEL_8);
        uint16_t raw_i3 = ADC1_Test_ReadChannelRaw(ADC_CHANNEL_9);
        uint16_t raw_i4 = ADC1_Test_ReadChannelRaw(ADC_CHANNEL_14);
        uint16_t raw_i5 = ADC1_Test_ReadChannelRaw(ADC_CHANNEL_15);

        sum_i1 += ADC1_Test_RawToVoltage(raw_i1);
        sum_i2 += ADC1_Test_RawToVoltage(raw_i2);
        sum_i3 += ADC1_Test_RawToVoltage(raw_i3);
        sum_i4 += ADC1_Test_RawToVoltage(raw_i4);
        sum_i5 += ADC1_Test_RawToVoltage(raw_i5);

        ADC1_Test_DelayMs(2);
    }

    g_adc_i1_zero_v = sum_i1 / (float)samples;
    g_adc_i2_zero_v = sum_i2 / (float)samples;
    g_adc_i3_zero_v = sum_i3 / (float)samples;
    g_adc_i4_zero_v = sum_i4 / (float)samples;
    g_adc_i5_zero_v = sum_i5 / (float)samples;
}

void ADC1_Test_RunOnce(void)
{
    /*
     * Read voltage channels.
     */
    g_adc_v1_raw = ADC1_Test_ReadChannelRaw(ADC_CHANNEL_1);
    g_adc_v2_raw = ADC1_Test_ReadChannelRaw(ADC_CHANNEL_2);
    g_adc_v3_raw = ADC1_Test_ReadChannelRaw(ADC_CHANNEL_4);
    g_adc_v4_raw = ADC1_Test_ReadChannelRaw(ADC_CHANNEL_5);
    g_adc_v5_raw = ADC1_Test_ReadChannelRaw(ADC_CHANNEL_6);

    g_adc_v1_adc_v = ADC1_Test_RawToVoltage(g_adc_v1_raw);
    g_adc_v2_adc_v = ADC1_Test_RawToVoltage(g_adc_v2_raw);
    g_adc_v3_adc_v = ADC1_Test_RawToVoltage(g_adc_v3_raw);
    g_adc_v4_adc_v = ADC1_Test_RawToVoltage(g_adc_v4_raw);
    g_adc_v5_adc_v = ADC1_Test_RawToVoltage(g_adc_v5_raw);

    g_adc_v1_input_v = g_adc_v1_adc_v * ADC1_TEST_VOLT_RATIO_V1;
    g_adc_v2_input_v = g_adc_v2_adc_v * ADC1_TEST_VOLT_RATIO_V2;
    g_adc_v3_input_v = g_adc_v3_adc_v * ADC1_TEST_VOLT_RATIO_V3;
    g_adc_v4_input_v = g_adc_v4_adc_v * ADC1_TEST_VOLT_RATIO_V4;
    g_adc_v5_input_v = g_adc_v5_adc_v * ADC1_TEST_VOLT_RATIO_V5;

    /*
     * Read current channels.
     */
    g_adc_i1_raw = ADC1_Test_ReadChannelRaw(ADC_CHANNEL_7);
    g_adc_i2_raw = ADC1_Test_ReadChannelRaw(ADC_CHANNEL_8);
    g_adc_i3_raw = ADC1_Test_ReadChannelRaw(ADC_CHANNEL_9);
    g_adc_i4_raw = ADC1_Test_ReadChannelRaw(ADC_CHANNEL_14);
    g_adc_i5_raw = ADC1_Test_ReadChannelRaw(ADC_CHANNEL_15);

    g_adc_i1_adc_v = ADC1_Test_RawToVoltage(g_adc_i1_raw);
    g_adc_i2_adc_v = ADC1_Test_RawToVoltage(g_adc_i2_raw);
    g_adc_i3_adc_v = ADC1_Test_RawToVoltage(g_adc_i3_raw);
    g_adc_i4_adc_v = ADC1_Test_RawToVoltage(g_adc_i4_raw);
    g_adc_i5_adc_v = ADC1_Test_RawToVoltage(g_adc_i5_raw);

    g_adc_i1_a = ADC1_Test_CurrentFromVoltage(g_adc_i1_adc_v, g_adc_i1_zero_v, ADC1_TEST_RSHUNT_I1_OHM);
    g_adc_i2_a = ADC1_Test_CurrentFromVoltage(g_adc_i2_adc_v, g_adc_i2_zero_v, ADC1_TEST_RSHUNT_I2_OHM);
    g_adc_i3_a = ADC1_Test_CurrentFromVoltage(g_adc_i3_adc_v, g_adc_i3_zero_v, ADC1_TEST_RSHUNT_I3_OHM);
    g_adc_i4_a = ADC1_Test_CurrentFromVoltage(g_adc_i4_adc_v, g_adc_i4_zero_v, ADC1_TEST_RSHUNT_I4_OHM);
    g_adc_i5_a = ADC1_Test_CurrentFromVoltage(g_adc_i5_adc_v, g_adc_i5_zero_v, ADC1_TEST_RSHUNT_I5_OHM);
}

/*
 * Optional task body example.
 * You can copy this into app_freertos.c.
 */
void ADC1_Test_TaskExample(void)
{
    ADC1_Test_Init();

    /*
     * Keep all H-bridges disabled and no current during calibration.
     */
    ADC1_Test_CalibrateCurrentZero(64);

    for (;;)
    {
        ADC1_Test_RunOnce();

        /*
         * Watch:
         *   g_adc_v1_input_v ... g_adc_v5_input_v
         *   g_adc_i1_a       ... g_adc_i5_a
         */
        osDelay(200);
    }
}
