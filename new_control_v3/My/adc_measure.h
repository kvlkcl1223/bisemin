#ifndef ADC_MEASURE_H
#define ADC_MEASURE_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADC_MEASURE_ADC1_CHANNEL_COUNT 10U
#define ADC_MEASURE_ADC2_CHANNEL_COUNT 5U

typedef enum
{
    ADC_MEASURE_OK = 0,
    ADC_MEASURE_ERROR_ADC1 = 1,
    ADC_MEASURE_ERROR_ADC2 = 2,
    ADC_MEASURE_ERROR_TIM7 = 3
} AdcMeasure_Status_t;

extern volatile HAL_StatusTypeDef g_adc_measure_adc1_start_result;
extern volatile HAL_StatusTypeDef g_adc_measure_adc2_start_result;
extern volatile HAL_StatusTypeDef g_adc_measure_tim7_start_result;
extern volatile uint32_t g_adc_measure_adc1_update_count;
extern volatile uint32_t g_adc_measure_adc2_update_count;
extern volatile uint32_t g_adc_measure_adc1_error_count;
extern volatile uint32_t g_adc_measure_adc2_error_count;

extern volatile uint16_t g_adc_measure_adc1_raw[ADC_MEASURE_ADC1_CHANNEL_COUNT];
extern volatile uint16_t g_adc_measure_adc2_raw[ADC_MEASURE_ADC2_CHANNEL_COUNT];

extern volatile float g_adc_measure_v_adc_v[5];
extern volatile float g_adc_measure_v_input_v[5];
extern volatile float g_adc_measure_i_adc_v[5];
extern volatile float g_adc_measure_i_zero_v[5];
extern volatile float g_adc_measure_i_a[5];
extern volatile float g_adc_measure_il_adc_v[5];
extern volatile float g_adc_measure_il_zero_v[5];
extern volatile float g_adc_measure_il_a[5];

extern volatile uint16_t g_adc_v1_raw;
extern volatile uint16_t g_adc_v2_raw;
extern volatile uint16_t g_adc_v3_raw;
extern volatile uint16_t g_adc_v4_raw;
extern volatile uint16_t g_adc_v5_raw;
extern volatile uint16_t g_adc_i1_raw;
extern volatile uint16_t g_adc_i2_raw;
extern volatile uint16_t g_adc_i3_raw;
extern volatile uint16_t g_adc_i4_raw;
extern volatile uint16_t g_adc_i5_raw;

extern volatile float g_adc_v1_adc_v;
extern volatile float g_adc_v2_adc_v;
extern volatile float g_adc_v3_adc_v;
extern volatile float g_adc_v4_adc_v;
extern volatile float g_adc_v5_adc_v;
extern volatile float g_adc_v1_input_v;
extern volatile float g_adc_v2_input_v;
extern volatile float g_adc_v3_input_v;
extern volatile float g_adc_v4_input_v;
extern volatile float g_adc_v5_input_v;
extern volatile float g_adc_i1_adc_v;
extern volatile float g_adc_i2_adc_v;
extern volatile float g_adc_i3_adc_v;
extern volatile float g_adc_i4_adc_v;
extern volatile float g_adc_i5_adc_v;
extern volatile float g_adc_i1_zero_v;
extern volatile float g_adc_i2_zero_v;
extern volatile float g_adc_i3_zero_v;
extern volatile float g_adc_i4_zero_v;
extern volatile float g_adc_i5_zero_v;
extern volatile float g_adc_i1_a;
extern volatile float g_adc_i2_a;
extern volatile float g_adc_i3_a;
extern volatile float g_adc_i4_a;
extern volatile float g_adc_i5_a;

extern volatile uint16_t g_adc_il1_raw;
extern volatile uint16_t g_adc_il2_raw;
extern volatile uint16_t g_adc_il3_raw;
extern volatile uint16_t g_adc_il4_raw;
extern volatile uint16_t g_adc_il5_raw;
extern volatile float g_adc_il1_adc_v;
extern volatile float g_adc_il2_adc_v;
extern volatile float g_adc_il3_adc_v;
extern volatile float g_adc_il4_adc_v;
extern volatile float g_adc_il5_adc_v;
extern volatile float g_adc_il1_zero_v;
extern volatile float g_adc_il2_zero_v;
extern volatile float g_adc_il3_zero_v;
extern volatile float g_adc_il4_zero_v;
extern volatile float g_adc_il5_zero_v;
extern volatile float g_adc_il1_a;
extern volatile float g_adc_il2_a;
extern volatile float g_adc_il3_a;
extern volatile float g_adc_il4_a;
extern volatile float g_adc_il5_a;

AdcMeasure_Status_t AdcMeasure_Start(void);
void AdcMeasure_CalibrateCurrentZero(uint16_t samples);
void AdcMeasure_Process(void);

#ifdef __cplusplus
}
#endif

#endif
