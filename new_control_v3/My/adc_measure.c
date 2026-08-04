#include "adc_measure.h"

#include "adc.h"
#include "cmsis_os.h"
#include "tim.h"

#define ADC_MEASURE_VDDA_V 3.300f
#define ADC_MEASURE_ADC_FULL_SCALE 4095.0f

#define ADC_MEASURE_VOLT_RATIO_V1 16.0f
#define ADC_MEASURE_VOLT_RATIO_V2 16.0f
#define ADC_MEASURE_VOLT_RATIO_V3 16.0f
#define ADC_MEASURE_VOLT_RATIO_V4 16.0f
#define ADC_MEASURE_VOLT_RATIO_V5 16.0f

#define ADC_MEASURE_INA240A1_GAIN_VV 20.0f
#define ADC_MEASURE_RSHUNT_I1_OHM 0.005f
#define ADC_MEASURE_RSHUNT_I2_OHM 0.005f
#define ADC_MEASURE_RSHUNT_I3_OHM 0.005f
#define ADC_MEASURE_RSHUNT_I4_OHM 0.005f
#define ADC_MEASURE_RSHUNT_I5_OHM 0.005f
#define ADC_MEASURE_RSHUNT_IL1_OHM 0.005f
#define ADC_MEASURE_RSHUNT_IL2_OHM 0.005f
#define ADC_MEASURE_RSHUNT_IL3_OHM 0.005f
#define ADC_MEASURE_RSHUNT_IL4_OHM 0.005f
#define ADC_MEASURE_RSHUNT_IL5_OHM 0.005f

#define ADC_MEASURE_ADC1_SAMPLING_TIME ADC_SAMPLETIME_247CYCLES_5
#define ADC_MEASURE_ADC2_SAMPLING_TIME ADC_SAMPLETIME_47CYCLES_5

static const float k_voltage_ratio[5] = {
    ADC_MEASURE_VOLT_RATIO_V1,
    ADC_MEASURE_VOLT_RATIO_V2,
    ADC_MEASURE_VOLT_RATIO_V3,
    ADC_MEASURE_VOLT_RATIO_V4,
    ADC_MEASURE_VOLT_RATIO_V5};

static const float k_adc1_current_rshunt[5] = {
    ADC_MEASURE_RSHUNT_I1_OHM,
    ADC_MEASURE_RSHUNT_I2_OHM,
    ADC_MEASURE_RSHUNT_I3_OHM,
    ADC_MEASURE_RSHUNT_I4_OHM,
    ADC_MEASURE_RSHUNT_I5_OHM};

static const float k_adc2_current_rshunt[5] = {
    ADC_MEASURE_RSHUNT_IL1_OHM,
    ADC_MEASURE_RSHUNT_IL2_OHM,
    ADC_MEASURE_RSHUNT_IL3_OHM,
    ADC_MEASURE_RSHUNT_IL4_OHM,
    ADC_MEASURE_RSHUNT_IL5_OHM};

volatile HAL_StatusTypeDef g_adc_measure_adc1_start_result = HAL_OK;
volatile HAL_StatusTypeDef g_adc_measure_adc2_start_result = HAL_OK;
volatile HAL_StatusTypeDef g_adc_measure_tim7_start_result = HAL_OK;
volatile uint32_t g_adc_measure_adc1_update_count = 0U;
volatile uint32_t g_adc_measure_adc2_update_count = 0U;
volatile uint32_t g_adc_measure_adc1_error_count = 0U;
volatile uint32_t g_adc_measure_adc2_error_count = 0U;

static uint16_t g_adc_measure_adc1_dma[ADC_MEASURE_ADC1_CHANNEL_COUNT] = {0};
static uint16_t g_adc_measure_adc2_dma[ADC_MEASURE_ADC2_CHANNEL_COUNT] = {0};
volatile uint16_t g_adc_measure_adc1_raw[ADC_MEASURE_ADC1_CHANNEL_COUNT] = {0};
volatile uint16_t g_adc_measure_adc2_raw[ADC_MEASURE_ADC2_CHANNEL_COUNT] = {0};

volatile float g_adc_measure_v_adc_v[5] = {0.0f};
volatile float g_adc_measure_v_input_v[5] = {0.0f};
volatile float g_adc_measure_i_adc_v[5] = {0.0f};
volatile float g_adc_measure_i_zero_v[5] = {
    ADC_MEASURE_VDDA_V * 0.5f,
    ADC_MEASURE_VDDA_V * 0.5f,
    ADC_MEASURE_VDDA_V * 0.5f,
    ADC_MEASURE_VDDA_V * 0.5f,
    ADC_MEASURE_VDDA_V * 0.5f};
volatile float g_adc_measure_i_a[5] = {0.0f};
volatile float g_adc_measure_il_adc_v[5] = {0.0f};
volatile float g_adc_measure_il_zero_v[5] = {
    ADC_MEASURE_VDDA_V * 0.5f,
    ADC_MEASURE_VDDA_V * 0.5f,
    ADC_MEASURE_VDDA_V * 0.5f,
    ADC_MEASURE_VDDA_V * 0.5f,
    ADC_MEASURE_VDDA_V * 0.5f};
volatile float g_adc_measure_il_a[5] = {0.0f};

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

volatile float g_adc_v1_adc_v = 0.0f;
volatile float g_adc_v2_adc_v = 0.0f;
volatile float g_adc_v3_adc_v = 0.0f;
volatile float g_adc_v4_adc_v = 0.0f;
volatile float g_adc_v5_adc_v = 0.0f;
volatile float g_adc_v1_input_v = 0.0f;
volatile float g_adc_v2_input_v = 0.0f;
volatile float g_adc_v3_input_v = 0.0f;
volatile float g_adc_v4_input_v = 0.0f;
volatile float g_adc_v5_input_v = 0.0f;
volatile float g_adc_i1_adc_v = 0.0f;
volatile float g_adc_i2_adc_v = 0.0f;
volatile float g_adc_i3_adc_v = 0.0f;
volatile float g_adc_i4_adc_v = 0.0f;
volatile float g_adc_i5_adc_v = 0.0f;
volatile float g_adc_i1_zero_v = ADC_MEASURE_VDDA_V * 0.5f;
volatile float g_adc_i2_zero_v = ADC_MEASURE_VDDA_V * 0.5f;
volatile float g_adc_i3_zero_v = ADC_MEASURE_VDDA_V * 0.5f;
volatile float g_adc_i4_zero_v = ADC_MEASURE_VDDA_V * 0.5f;
volatile float g_adc_i5_zero_v = ADC_MEASURE_VDDA_V * 0.5f;
volatile float g_adc_i1_a = 0.0f;
volatile float g_adc_i2_a = 0.0f;
volatile float g_adc_i3_a = 0.0f;
volatile float g_adc_i4_a = 0.0f;
volatile float g_adc_i5_a = 0.0f;

volatile uint16_t g_adc_il1_raw = 0;
volatile uint16_t g_adc_il2_raw = 0;
volatile uint16_t g_adc_il3_raw = 0;
volatile uint16_t g_adc_il4_raw = 0;
volatile uint16_t g_adc_il5_raw = 0;
volatile float g_adc_il1_adc_v = 0.0f;
volatile float g_adc_il2_adc_v = 0.0f;
volatile float g_adc_il3_adc_v = 0.0f;
volatile float g_adc_il4_adc_v = 0.0f;
volatile float g_adc_il5_adc_v = 0.0f;
volatile float g_adc_il1_zero_v = ADC_MEASURE_VDDA_V * 0.5f;
volatile float g_adc_il2_zero_v = ADC_MEASURE_VDDA_V * 0.5f;
volatile float g_adc_il3_zero_v = ADC_MEASURE_VDDA_V * 0.5f;
volatile float g_adc_il4_zero_v = ADC_MEASURE_VDDA_V * 0.5f;
volatile float g_adc_il5_zero_v = ADC_MEASURE_VDDA_V * 0.5f;
volatile float g_adc_il1_a = 0.0f;
volatile float g_adc_il2_a = 0.0f;
volatile float g_adc_il3_a = 0.0f;
volatile float g_adc_il4_a = 0.0f;
volatile float g_adc_il5_a = 0.0f;

static float AdcMeasure_RawToVoltage(uint16_t raw)
{
    return ((float)raw * ADC_MEASURE_VDDA_V) / ADC_MEASURE_ADC_FULL_SCALE;
}

static float AdcMeasure_CurrentFromVoltage(float vout, float vzero, float rshunt)
{
    return (vout - vzero) / (rshunt * ADC_MEASURE_INA240A1_GAIN_VV);
}

static HAL_StatusTypeDef AdcMeasure_ConfigAdc1Channels(void)
{
    static const uint32_t channels[ADC_MEASURE_ADC1_CHANNEL_COUNT] = {
        ADC_CHANNEL_1,
        ADC_CHANNEL_2,
        ADC_CHANNEL_4,
        ADC_CHANNEL_5,
        ADC_CHANNEL_6,
        ADC_CHANNEL_7,
        ADC_CHANNEL_8,
        ADC_CHANNEL_9,
        ADC_CHANNEL_14,
        ADC_CHANNEL_15};
    static const uint32_t ranks[ADC_MEASURE_ADC1_CHANNEL_COUNT] = {
        ADC_REGULAR_RANK_1,
        ADC_REGULAR_RANK_2,
        ADC_REGULAR_RANK_3,
        ADC_REGULAR_RANK_4,
        ADC_REGULAR_RANK_5,
        ADC_REGULAR_RANK_6,
        ADC_REGULAR_RANK_7,
        ADC_REGULAR_RANK_8,
        ADC_REGULAR_RANK_9,
        ADC_REGULAR_RANK_10};
    ADC_ChannelConfTypeDef config = {0};
    uint8_t ch;
    HAL_StatusTypeDef ret;

    config.SamplingTime = ADC_MEASURE_ADC1_SAMPLING_TIME;
    config.SingleDiff = ADC_SINGLE_ENDED;
    config.OffsetNumber = ADC_OFFSET_NONE;
    config.Offset = 0;

    for (ch = 0U; ch < ADC_MEASURE_ADC1_CHANNEL_COUNT; ch++)
    {
        config.Channel = channels[ch];
        config.Rank = ranks[ch];
        ret = HAL_ADC_ConfigChannel(&hadc1, &config);
        if (ret != HAL_OK)
        {
            g_adc_measure_adc1_error_count++;
            return ret;
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef AdcMeasure_ConfigAdc2Channels(void)
{
    static const uint32_t channels[ADC_MEASURE_ADC2_CHANNEL_COUNT] = {
        ADC_CHANNEL_4,
        ADC_CHANNEL_5,
        ADC_CHANNEL_11,
        ADC_CHANNEL_12,
        ADC_CHANNEL_13};
    static const uint32_t ranks[ADC_MEASURE_ADC2_CHANNEL_COUNT] = {
        ADC_REGULAR_RANK_1,
        ADC_REGULAR_RANK_2,
        ADC_REGULAR_RANK_3,
        ADC_REGULAR_RANK_4,
        ADC_REGULAR_RANK_5};
    ADC_ChannelConfTypeDef config = {0};
    uint8_t ch;
    HAL_StatusTypeDef ret;

    config.SamplingTime = ADC_MEASURE_ADC2_SAMPLING_TIME;
    config.SingleDiff = ADC_SINGLE_ENDED;
    config.OffsetNumber = ADC_OFFSET_NONE;
    config.Offset = 0;

    for (ch = 0U; ch < ADC_MEASURE_ADC2_CHANNEL_COUNT; ch++)
    {
        config.Channel = channels[ch];
        config.Rank = ranks[ch];
        ret = HAL_ADC_ConfigChannel(&hadc2, &config);
        if (ret != HAL_OK)
        {
            g_adc_measure_adc2_error_count++;
            return ret;
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef AdcMeasure_StartAdc1(void)
{
    HAL_StatusTypeDef ret;

    HAL_ADC_Stop_DMA(&hadc1);
    ret = AdcMeasure_ConfigAdc1Channels();
    if (ret != HAL_OK)
    {
        return ret;
    }

    ret = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    if (ret != HAL_OK)
    {
        g_adc_measure_adc1_error_count++;
        return ret;
    }

    ret = HAL_ADC_Start_DMA(&hadc1,
                            (uint32_t *)g_adc_measure_adc1_dma,
                            ADC_MEASURE_ADC1_CHANNEL_COUNT);
    if (ret != HAL_OK)
    {
        g_adc_measure_adc1_error_count++;
    }

    return ret;
}

static HAL_StatusTypeDef AdcMeasure_StartAdc2(void)
{
    HAL_StatusTypeDef ret;

    HAL_ADC_Stop_DMA(&hadc2);
    ret = AdcMeasure_ConfigAdc2Channels();
    if (ret != HAL_OK)
    {
        return ret;
    }

    ret = HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    if (ret != HAL_OK)
    {
        g_adc_measure_adc2_error_count++;
        return ret;
    }

    ret = HAL_ADC_Start_DMA(&hadc2,
                            (uint32_t *)g_adc_measure_adc2_dma,
                            ADC_MEASURE_ADC2_CHANNEL_COUNT);
    if (ret != HAL_OK)
    {
        g_adc_measure_adc2_error_count++;
    }

    return ret;
}

static HAL_StatusTypeDef AdcMeasure_StartTim7(void)
{
    if ((htim7.Instance->CR1 & TIM_CR1_CEN) != 0U)
    {
        return HAL_OK;
    }

    __HAL_TIM_SET_COUNTER(&htim7, 0U);
    return HAL_TIM_Base_Start(&htim7);
}

AdcMeasure_Status_t AdcMeasure_Start(void)
{
    g_adc_measure_adc1_start_result = AdcMeasure_StartAdc1();
    if (g_adc_measure_adc1_start_result != HAL_OK)
    {
        return ADC_MEASURE_ERROR_ADC1;
    }

    g_adc_measure_adc2_start_result = AdcMeasure_StartAdc2();
    if (g_adc_measure_adc2_start_result != HAL_OK)
    {
        return ADC_MEASURE_ERROR_ADC2;
    }

    g_adc_measure_tim7_start_result = AdcMeasure_StartTim7();
    if (g_adc_measure_tim7_start_result != HAL_OK)
    {
        return ADC_MEASURE_ERROR_TIM7;
    }

    return ADC_MEASURE_OK;
}

void AdcMeasure_CalibrateCurrentZero(uint16_t samples)
{
    uint16_t i;
    uint8_t ch;
    float sum_i[5] = {0.0f};
    float sum_il[5] = {0.0f};

    if (samples == 0U)
    {
        samples = 1U;
    }

    for (i = 0U; i < samples; i++)
    {
        osDelay(2);

        for (ch = 0U; ch < 5U; ch++)
        {
            sum_i[ch] += AdcMeasure_RawToVoltage(g_adc_measure_adc1_raw[ch + 5U]);
            sum_il[ch] += AdcMeasure_RawToVoltage(g_adc_measure_adc2_raw[ch]);
        }
    }

    for (ch = 0U; ch < 5U; ch++)
    {
        g_adc_measure_i_zero_v[ch] = sum_i[ch] / (float)samples;
        g_adc_measure_il_zero_v[ch] = sum_il[ch] / (float)samples;
    }
}

void AdcMeasure_Process(void)
{
    uint8_t ch;
    float vout;

    for (ch = 0U; ch < 5U; ch++)
    {
        g_adc_measure_v_adc_v[ch] =
            AdcMeasure_RawToVoltage(g_adc_measure_adc1_raw[ch]);
        g_adc_measure_v_input_v[ch] =
            g_adc_measure_v_adc_v[ch] * k_voltage_ratio[ch];

        vout = AdcMeasure_RawToVoltage(g_adc_measure_adc1_raw[ch + 5U]);
        g_adc_measure_i_adc_v[ch] = vout;
        g_adc_measure_i_a[ch] =
            AdcMeasure_CurrentFromVoltage(vout,
                                          g_adc_measure_i_zero_v[ch],
                                          k_adc1_current_rshunt[ch]);

        vout = AdcMeasure_RawToVoltage(g_adc_measure_adc2_raw[ch]);
        g_adc_measure_il_adc_v[ch] = vout;
        g_adc_measure_il_a[ch] =
            AdcMeasure_CurrentFromVoltage(vout,
                                          g_adc_measure_il_zero_v[ch],
                                          k_adc2_current_rshunt[ch]);
    }

    g_adc_v1_adc_v = g_adc_measure_v_adc_v[0];
    g_adc_v2_adc_v = g_adc_measure_v_adc_v[1];
    g_adc_v3_adc_v = g_adc_measure_v_adc_v[2];
    g_adc_v4_adc_v = g_adc_measure_v_adc_v[3];
    g_adc_v5_adc_v = g_adc_measure_v_adc_v[4];
    g_adc_v1_input_v = g_adc_measure_v_input_v[0];
    g_adc_v2_input_v = g_adc_measure_v_input_v[1];
    g_adc_v3_input_v = g_adc_measure_v_input_v[2];
    g_adc_v4_input_v = g_adc_measure_v_input_v[3];
    g_adc_v5_input_v = g_adc_measure_v_input_v[4];

    g_adc_i1_adc_v = g_adc_measure_i_adc_v[0];
    g_adc_i2_adc_v = g_adc_measure_i_adc_v[1];
    g_adc_i3_adc_v = g_adc_measure_i_adc_v[2];
    g_adc_i4_adc_v = g_adc_measure_i_adc_v[3];
    g_adc_i5_adc_v = g_adc_measure_i_adc_v[4];
    g_adc_i1_zero_v = g_adc_measure_i_zero_v[0];
    g_adc_i2_zero_v = g_adc_measure_i_zero_v[1];
    g_adc_i3_zero_v = g_adc_measure_i_zero_v[2];
    g_adc_i4_zero_v = g_adc_measure_i_zero_v[3];
    g_adc_i5_zero_v = g_adc_measure_i_zero_v[4];
    g_adc_i1_a = g_adc_measure_i_a[0];
    g_adc_i2_a = g_adc_measure_i_a[1];
    g_adc_i3_a = g_adc_measure_i_a[2];
    g_adc_i4_a = g_adc_measure_i_a[3];
    g_adc_i5_a = g_adc_measure_i_a[4];

    g_adc_il1_adc_v = g_adc_measure_il_adc_v[0];
    g_adc_il2_adc_v = g_adc_measure_il_adc_v[1];
    g_adc_il3_adc_v = g_adc_measure_il_adc_v[2];
    g_adc_il4_adc_v = g_adc_measure_il_adc_v[3];
    g_adc_il5_adc_v = g_adc_measure_il_adc_v[4];
    g_adc_il1_zero_v = g_adc_measure_il_zero_v[0];
    g_adc_il2_zero_v = g_adc_measure_il_zero_v[1];
    g_adc_il3_zero_v = g_adc_measure_il_zero_v[2];
    g_adc_il4_zero_v = g_adc_measure_il_zero_v[3];
    g_adc_il5_zero_v = g_adc_measure_il_zero_v[4];
    g_adc_il1_a = g_adc_measure_il_a[0];
    g_adc_il2_a = g_adc_measure_il_a[1];
    g_adc_il3_a = g_adc_measure_il_a[2];
    g_adc_il4_a = g_adc_measure_il_a[3];
    g_adc_il5_a = g_adc_measure_il_a[4];
}

static void AdcMeasure_CopyAdc1Raw(void)
{
    uint8_t ch;

    for (ch = 0U; ch < ADC_MEASURE_ADC1_CHANNEL_COUNT; ch++)
    {
        g_adc_measure_adc1_raw[ch] = g_adc_measure_adc1_dma[ch];
    }

    g_adc_v1_raw = g_adc_measure_adc1_raw[0];
    g_adc_v2_raw = g_adc_measure_adc1_raw[1];
    g_adc_v3_raw = g_adc_measure_adc1_raw[2];
    g_adc_v4_raw = g_adc_measure_adc1_raw[3];
    g_adc_v5_raw = g_adc_measure_adc1_raw[4];
    g_adc_i1_raw = g_adc_measure_adc1_raw[5];
    g_adc_i2_raw = g_adc_measure_adc1_raw[6];
    g_adc_i3_raw = g_adc_measure_adc1_raw[7];
    g_adc_i4_raw = g_adc_measure_adc1_raw[8];
    g_adc_i5_raw = g_adc_measure_adc1_raw[9];
}

static void AdcMeasure_CopyAdc2Raw(void)
{
    uint8_t ch;

    for (ch = 0U; ch < ADC_MEASURE_ADC2_CHANNEL_COUNT; ch++)
    {
        g_adc_measure_adc2_raw[ch] = g_adc_measure_adc2_dma[ch];
    }

    g_adc_il1_raw = g_adc_measure_adc2_raw[0];
    g_adc_il2_raw = g_adc_measure_adc2_raw[1];
    g_adc_il3_raw = g_adc_measure_adc2_raw[2];
    g_adc_il4_raw = g_adc_measure_adc2_raw[3];
    g_adc_il5_raw = g_adc_measure_adc2_raw[4];
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        AdcMeasure_CopyAdc1Raw();
        g_adc_measure_adc1_update_count++;
    }
    else if (hadc->Instance == ADC2)
    {
        AdcMeasure_CopyAdc2Raw();
        g_adc_measure_adc2_update_count++;
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        g_adc_measure_adc1_error_count++;
    }
    else if (hadc->Instance == ADC2)
    {
        g_adc_measure_adc2_error_count++;
    }
}
