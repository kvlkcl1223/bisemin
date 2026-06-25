// #include "bridge1_test.h"
// #include "opamp.h"
// #include "adc.h"
// #include "dac.h"
// #include "spi.h"
// #include "tim.h"
// #include "cmsis_os.h"

// // extern TIM_HandleTypeDef htim5;
// // extern DAC_HandleTypeDef hdac3;
// // extern OPAMP_HandleTypeDef hopamp1;
// // extern SPI_HandleTypeDef hspi1;

// // #define BRIDGE1_PWM_TIM_HANDLE htim5
// // #define BRIDGE1_PWM_CHANNEL TIM_CHANNEL_3
// // #define BRIDGE1_SPI_HANDLE hspi1
// // #define DRV8703_SPI_TIMEOUT_MS 10U

// // #define BRIDGE1_USE_DAC_VREF 1
// // #define BRIDGE1_DAC_HANDLE hdac3
// // #define BRIDGE1_DAC_CHANNEL DAC_CHANNEL_1

// extern TIM_HandleTypeDef htim1;
// extern DAC_HandleTypeDef hdac1;
// extern OPAMP_HandleTypeDef hopamp2;
// extern SPI_HandleTypeDef hspi1;

// #define BRIDGE1_PWM_TIM_HANDLE htim1
// #define BRIDGE1_PWM_CHANNEL TIM_CHANNEL_2
// #define BRIDGE1_SPI_HANDLE hspi1
// #define DRV8703_SPI_TIMEOUT_MS 10U

// #define BRIDGE1_USE_DAC_VREF 1
// #define BRIDGE1_DAC_HANDLE hdac1
// #define BRIDGE1_DAC_CHANNEL DAC_CHANNEL_1

// #define BRIDGE1_MAX_TEST_DUTY 0.50f

// #define BRIDGE1_INA_RSHUNT_OHM 0.005f
// #define BRIDGE1_INA_GAIN 20.0f

// #define DRV8703_REG_FAULT 0x00U
// #define DRV8703_REG_VDS_GDF 0x01U
// #define DRV8703_REG_MAIN_CONTROL 0x02U
// #define DRV8703_REG_IDRIVE_WD 0x03U
// #define DRV8703_REG_VDS_CONTROL 0x04U

// #define BRIDGE_DELAY_MS(ms) osDelay(ms)

// volatile DRV8703_FaultInfo_t g_bridge1_last_fault = {0};
// volatile Bridge1_Status_t g_bridge1_last_spi_status = BRIDGE1_OK;
// volatile Bridge1_Status_t g_bridge1_test_result = BRIDGE1_OK;
// volatile uint8_t g_bridge1_test_phase = 0;

// volatile uint8_t g_drv8703_last_tx0 = 0;
// volatile uint8_t g_drv8703_last_tx1 = 0;
// volatile uint8_t g_drv8703_last_rx0 = 0;
// volatile uint8_t g_drv8703_last_rx1 = 0;
// volatile uint16_t g_drv8703_last_tx16 = 0;
// volatile uint16_t g_drv8703_last_rx16 = 0;

// volatile uint8_t g_drv8703_reg_dump[8] = {0};

// volatile uint8_t g_static_reg0_before_clear = 0xFF;
// volatile uint8_t g_static_reg1_before_clear = 0xFF;
// volatile uint8_t g_static_reg0_after_clear = 0xFF;
// volatile uint8_t g_static_reg1_after_clear = 0xFF;

// volatile uint8_t g_pwm_reg0_after_20ms = 0xFF;
// volatile uint8_t g_pwm_reg1_after_20ms = 0xFF;
// volatile uint8_t g_pwm_reg0_after_200ms = 0xFF;
// volatile uint8_t g_pwm_reg1_after_200ms = 0xFF;
// volatile uint8_t g_pwm_reg0_after_1000ms = 0xFF;
// volatile uint8_t g_pwm_reg1_after_1000ms = 0xFF;

// volatile uint8_t g_monitor_reg0 = 0xFF;
// volatile uint8_t g_monitor_reg1 = 0xFF;

// volatile uint8_t g_main_before = 0xFF;
// volatile uint8_t g_main_write_value = 0xFF;
// volatile uint8_t g_main_after_write = 0xFF;
// volatile uint8_t g_main_after_clear0 = 0xFF;

// volatile uint8_t g_idrive_wd_before = 0xFF;
// volatile uint8_t g_idrive_wd_after = 0xFF;

// volatile uint8_t g_vds_ctrl_before = 0xFF;
// volatile uint8_t g_vds_ctrl_after = 0xFF;

// static float Bridge1_LimitFloat(float x, float min, float max)
// {
//     if (x > max)
//         return max;
//     if (x < min)
//         return min;
//     return x;
// }

// static void Bridge1_SetPwmRaw(uint32_t compare)
// {
//     __HAL_TIM_SET_COMPARE(&BRIDGE1_PWM_TIM_HANDLE, BRIDGE1_PWM_CHANNEL, compare);
// }

// static uint32_t Bridge1_DutyToCompare(float duty_abs)
// {
//     uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&BRIDGE1_PWM_TIM_HANDLE);
//     duty_abs = Bridge1_LimitFloat(duty_abs, 0.0f, 0.95f);
//     return (uint32_t)((float)(arr + 1U) * duty_abs);
// }

// // static void DRV8703_CS1_Low(void)
// // {
// //     HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);
// // }

// // static void DRV8703_CS1_High(void)
// // {
// //     HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
// // }

// static void DRV8703_CS1_Low(void)
// {
//     HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_RESET);
// }

// static void DRV8703_CS1_High(void)
// {
//     HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET);
// }

// static Bridge1_Status_t DRV8703_Transfer16(uint16_t tx, uint16_t *rx)
// {
//     uint8_t tx_buf[2];
//     uint8_t rx_buf[2];
//     HAL_StatusTypeDef hal_ret;

//     if (rx == 0)
//     {
//         return BRIDGE1_PARAM_ERROR;
//     }

//     tx_buf[0] = (uint8_t)((tx >> 8) & 0xFFU);
//     tx_buf[1] = (uint8_t)(tx & 0xFFU);
//     rx_buf[0] = 0;
//     rx_buf[1] = 0;

//     g_drv8703_last_tx0 = tx_buf[0];
//     g_drv8703_last_tx1 = tx_buf[1];
//     g_drv8703_last_tx16 = tx;

//     DRV8703_CS1_Low();
//     osDelay(1);
//     hal_ret = HAL_SPI_TransmitReceive(&BRIDGE1_SPI_HANDLE,
//                                       tx_buf,
//                                       rx_buf,
//                                       2,
//                                       DRV8703_SPI_TIMEOUT_MS);
//     DRV8703_CS1_High();
//     osDelay(1);
//     g_drv8703_last_rx0 = rx_buf[0];
//     g_drv8703_last_rx1 = rx_buf[1];
//     g_drv8703_last_rx16 = ((uint16_t)rx_buf[0] << 8) | (uint16_t)rx_buf[1];

//     if (hal_ret != HAL_OK)
//     {
//         g_bridge1_last_spi_status = BRIDGE1_SPI_ERROR;
//         return BRIDGE1_SPI_ERROR;
//     }

//     *rx = g_drv8703_last_rx16;
//     g_bridge1_last_spi_status = BRIDGE1_OK;
//     return BRIDGE1_OK;
// }

// Bridge1_Status_t DRV8703_ReadRegChecked(uint8_t addr, uint8_t *data)
// {
//     uint16_t tx;
//     uint16_t rx = 0;
//     Bridge1_Status_t ret;

//     if (data == 0)
//     {
//         return BRIDGE1_PARAM_ERROR;
//     }

//     tx = (uint16_t)(0x8000U | ((uint16_t)(addr & 0x0FU) << 11));
//     ret = DRV8703_Transfer16(tx, &rx);

//     if (ret != BRIDGE1_OK)
//     {
//         *data = 0xFFU;
//         return ret;
//     }

//     *data = (uint8_t)(rx & 0x00FFU);
//     return BRIDGE1_OK;
// }

// uint8_t DRV8703_ReadReg(uint8_t addr)
// {
//     uint8_t data = 0xFFU;
//     (void)DRV8703_ReadRegChecked(addr, &data);
//     return data;
// }

// Bridge1_Status_t DRV8703_WriteReg(uint8_t addr, uint8_t data)
// {
//     uint16_t tx;
//     uint16_t rx = 0;

//     tx = (uint16_t)(((uint16_t)(addr & 0x0FU) << 11) | (uint16_t)data);
//     return DRV8703_Transfer16(tx, &rx);
// }

// void Bridge1_InitSafeState(void)
// {
//     HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET);
//     HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
//     HAL_GPIO_WritePin(SLEEP2_GPIO_Port, SLEEP2_Pin, GPIO_PIN_RESET);

//     HAL_TIM_PWM_Start(&BRIDGE1_PWM_TIM_HANDLE, BRIDGE1_PWM_CHANNEL);
//     Bridge1_SetPwmRaw(0);

// #if BRIDGE1_USE_DAC_VREF
//     HAL_DAC_Start(&BRIDGE1_DAC_HANDLE, BRIDGE1_DAC_CHANNEL);
//     Bridge1_SetVref_mV(1000);
//     // HAL_OPAMP_Start(&hopamp1);
//     HAL_OPAMP_Start(&hopamp2);
// #endif
// }

// void Bridge1_SetVref_mV(uint16_t mv)
// {
// #if BRIDGE1_USE_DAC_VREF
//     uint32_t dac_value;

//     if (mv > 3300U)
//     {
//         mv = 3300U;
//     }

//     dac_value = ((uint32_t)mv * 4095U) / 3300U;
//     HAL_DAC_SetValue(&BRIDGE1_DAC_HANDLE,
//                      BRIDGE1_DAC_CHANNEL,
//                      DAC_ALIGN_12B_R,
//                      dac_value);
// #else
//     (void)mv;
// #endif
// }

// void Bridge1_Enable(void)
// {
//     Bridge1_SetPwmRaw(0);
//     HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
//     HAL_GPIO_WritePin(SLEEP2_GPIO_Port, SLEEP2_Pin, GPIO_PIN_SET);
//     BRIDGE_DELAY_MS(10);
// }

// void Bridge1_Disable(void)
// {
//     Bridge1_SetPwmRaw(0);
//     HAL_GPIO_WritePin(SLEEP2_GPIO_Port, SLEEP2_Pin, GPIO_PIN_RESET);
// }

// void Bridge1_SetDuty(float duty)
// {
//     duty = Bridge1_LimitFloat(duty, -0.5f, 0.5f);

//     if (duty > 0.0f)
//     {
//         HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_SET);
//         Bridge1_SetPwmRaw(Bridge1_DutyToCompare(duty));
//     }
//     else if (duty < 0.0f)
//     {
//         HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
//         Bridge1_SetPwmRaw(Bridge1_DutyToCompare(-duty));
//     }
//     else
//     {
//         Bridge1_SetPwmRaw(0);
//     }
// }

// Bridge1_Status_t Bridge1_CheckFaultPins(void)
// {
//     if (HAL_GPIO_ReadPin(FAULT2_GPIO_Port, FAULT2_Pin) == GPIO_PIN_RESET)
//     {
//         Bridge1_SetDuty(0.0f);
//         return BRIDGE1_FAULT_ACTIVE;
//     }

//     if (HAL_GPIO_ReadPin(WDFLT2_GPIO_Port, WDFLT2_Pin) == GPIO_PIN_RESET)
//     {
//         Bridge1_SetDuty(0.0f);
//         return BRIDGE1_WDFLT_ACTIVE;
//     }

//     return BRIDGE1_OK;
// }

// Bridge1_Status_t Bridge1_ReadFaultInfo(DRV8703_FaultInfo_t *info)
// {
//     Bridge1_Status_t ret0;
//     Bridge1_Status_t ret1;
//     uint8_t reg0 = 0xFFU;
//     uint8_t reg1 = 0xFFU;

//     if (info == 0)
//     {
//         return BRIDGE1_PARAM_ERROR;
//     }

//     info->fault_status = 0xFFU;
//     info->vds_gdf_status = 0xFFU;

//     ret0 = DRV8703_ReadRegChecked(DRV8703_REG_FAULT, &reg0);
//     info->fault_status = reg0;

//     ret1 = DRV8703_ReadRegChecked(DRV8703_REG_VDS_GDF, &reg1);
//     info->vds_gdf_status = reg1;

//     g_bridge1_last_fault = *info;

//     if (ret0 != BRIDGE1_OK)
//         return ret0;
//     if (ret1 != BRIDGE1_OK)
//         return ret1;

//     return BRIDGE1_OK;
// }

// Bridge1_Status_t Bridge1_ClearFault(void)
// {
//     uint8_t main_reg;
//     Bridge1_Status_t ret;

//     main_reg = DRV8703_ReadReg(DRV8703_REG_MAIN_CONTROL);
//     g_main_before = main_reg;

//     g_main_write_value = (uint8_t)(main_reg | 0x01U);
//     ret = DRV8703_WriteReg(DRV8703_REG_MAIN_CONTROL, g_main_write_value);
//     if (ret != BRIDGE1_OK)
//         return ret;

//     BRIDGE_DELAY_MS(5);
//     g_main_after_write = DRV8703_ReadReg(DRV8703_REG_MAIN_CONTROL);

//     ret = DRV8703_WriteReg(DRV8703_REG_MAIN_CONTROL, (uint8_t)(main_reg & (uint8_t)(~0x01U)));
//     if (ret != BRIDGE1_OK)
//         return ret;

//     BRIDGE_DELAY_MS(5);
//     g_main_after_clear0 = DRV8703_ReadReg(DRV8703_REG_MAIN_CONTROL);

//     return BRIDGE1_OK;
// }

// Bridge1_Status_t Bridge1_DisableWatchdog(void)
// {
//     uint8_t reg3;
//     Bridge1_Status_t ret;

//     reg3 = DRV8703_ReadReg(DRV8703_REG_IDRIVE_WD);
//     g_idrive_wd_before = reg3;

//     reg3 = (uint8_t)(reg3 & (uint8_t)(~0x20U));
//     ret = DRV8703_WriteReg(DRV8703_REG_IDRIVE_WD, reg3);
//     if (ret != BRIDGE1_OK)
//         return ret;

//     BRIDGE_DELAY_MS(5);
//     g_idrive_wd_after = DRV8703_ReadReg(DRV8703_REG_IDRIVE_WD);

//     return BRIDGE1_OK;
// }

// void Bridge1_DumpRegs(void)
// {
//     uint8_t i;

//     for (i = 0; i < 8U; i++)
//     {
//         g_drv8703_reg_dump[i] = DRV8703_ReadReg(i);
//         BRIDGE_DELAY_MS(1);
//     }
// }

// Bridge1_Status_t Bridge1_StaticMosCheck(void)
// {
//     Bridge1_Status_t ret;

//     g_bridge1_test_phase = 1;

//     Bridge1_InitSafeState();
//     BRIDGE_DELAY_MS(50);

//     Bridge1_SetVref_mV(1000);
//     Bridge1_SetDuty(0.0f);

//     HAL_GPIO_WritePin(SLEEP2_GPIO_Port, SLEEP2_Pin, GPIO_PIN_SET);
//     BRIDGE_DELAY_MS(20);
//     g_idrive_wd_after = DRV8703_ReadReg(0x04);
//     g_idrive_wd_after = DRV8703_ReadReg(0x03);
//     g_idrive_wd_after = DRV8703_ReadReg(0x02);
//     DRV8703_WriteReg(0x02, 0x18); // LOCK[5:3] = 011b������
//     BRIDGE_DELAY_MS(20);
//     g_idrive_wd_after = DRV8703_ReadReg(0x02);
//     BRIDGE_DELAY_MS(20);
//     /* ����� */
//     // Bridge1_ClearFault();
//     // osDelay(20);
//     /* ���¶�ȡ */
//     uint8_t reg0 = DRV8703_ReadReg(0x00);
//     uint8_t reg1 = DRV8703_ReadReg(0x01); /* �ر� watchdog���������㵱ǰ�ļ���ĺ��� */
//                                           /*
//                                            * 0x03 IDRIVE and WD Register
//                                            *
//                                            * bit7~6 TDEAD = 11b���������
//                                            * bit5    WD_EN = 0 ���ر� watchdog
//                                            * bit4~3  WD_DLY = 00
//                                            * bit2~0  IDRIVE = 111���������/Ĭ��������
//                                            *
//                                            * 0xC7 = 1100 0111b
//                                            */
//     DRV8703_WriteReg(0x03, 0xC7);
//     osDelay(20);

//     g_idrive_wd_after = DRV8703_ReadReg(0x03);

//     g_static_reg0_before_clear = DRV8703_ReadReg(DRV8703_REG_FAULT);
//     g_static_reg1_before_clear = DRV8703_ReadReg(DRV8703_REG_VDS_GDF);

//     g_vds_ctrl_before = DRV8703_ReadReg(DRV8703_REG_VDS_CONTROL);

//     ret = Bridge1_DisableWatchdog();
//     if (ret != BRIDGE1_OK)
//         return ret;

//     ret = Bridge1_ClearFault();
//     if (ret != BRIDGE1_OK)
//         return ret;

//     BRIDGE_DELAY_MS(20);

//     g_static_reg0_after_clear = DRV8703_ReadReg(DRV8703_REG_FAULT);
//     g_static_reg1_after_clear = DRV8703_ReadReg(DRV8703_REG_VDS_GDF);

//     g_vds_ctrl_after = DRV8703_ReadReg(DRV8703_REG_VDS_CONTROL);

//     Bridge1_DumpRegs();

//     g_bridge1_test_phase = 2;

//     return BRIDGE1_OK;
// }

// Bridge1_Status_t Bridge1_StartMosPwmHoldTest(float duty, uint16_t vref_mv)
// {
//     Bridge1_Status_t ret;

//     duty = Bridge1_LimitFloat(duty, -BRIDGE1_MAX_TEST_DUTY, BRIDGE1_MAX_TEST_DUTY);

//     g_bridge1_test_phase = 10;

//     ret = Bridge1_StaticMosCheck();
//     if (ret != BRIDGE1_OK)
//     {
//         g_bridge1_test_result = ret;
//         return ret;
//     }

//     if (g_static_reg0_after_clear != 0x00U)
//     {
//         g_bridge1_test_result = BRIDGE1_FAULT_ACTIVE;
//         return BRIDGE1_FAULT_ACTIVE;
//     }

//     g_bridge1_test_phase = 11;

//     Bridge1_SetVref_mV(vref_mv);
//     Bridge1_SetDuty(duty);

//     BRIDGE_DELAY_MS(20);
//     g_pwm_reg0_after_20ms = DRV8703_ReadReg(DRV8703_REG_FAULT);
//     g_pwm_reg1_after_20ms = DRV8703_ReadReg(DRV8703_REG_VDS_GDF);

//     BRIDGE_DELAY_MS(180);
//     g_pwm_reg0_after_200ms = DRV8703_ReadReg(DRV8703_REG_FAULT);
//     g_pwm_reg1_after_200ms = DRV8703_ReadReg(DRV8703_REG_VDS_GDF);

//     BRIDGE_DELAY_MS(800);
//     g_pwm_reg0_after_1000ms = DRV8703_ReadReg(DRV8703_REG_FAULT);
//     g_pwm_reg1_after_1000ms = DRV8703_ReadReg(DRV8703_REG_VDS_GDF);

//     Bridge1_DumpRegs();

//     if (
//         g_pwm_reg0_after_200ms != 0x00U ||
//         g_pwm_reg0_after_1000ms != 0x00U)
//     {
//         Bridge1_StopHoldTest();
//         g_bridge1_test_result = BRIDGE1_FAULT_ACTIVE;
//         return BRIDGE1_FAULT_ACTIVE;
//     }

//     g_bridge1_test_phase = 12;
//     g_bridge1_test_result = BRIDGE1_OK;
//     return BRIDGE1_OK;
// }

// void Bridge1_StopHoldTest(void)
// {
//     Bridge1_SetDuty(0.0f);
//     BRIDGE_DELAY_MS(5);
//     HAL_GPIO_WritePin(SLEEP2_GPIO_Port, SLEEP2_Pin, GPIO_PIN_RESET);
//     g_bridge1_test_phase = 99;
// }

// void Bridge1_MonitorDuringPwm(void)
// {
//     g_monitor_reg0 = DRV8703_ReadReg(DRV8703_REG_FAULT);
//     g_monitor_reg1 = DRV8703_ReadReg(DRV8703_REG_VDS_GDF);

//     if (g_monitor_reg0 != 0x00U)
//     {
//         Bridge1_StopHoldTest();
//         g_bridge1_test_result = BRIDGE1_FAULT_ACTIVE;
//     }
// }

// float Bridge1_CurrentFromAdcRaw(uint16_t adc_raw, float vdda, float zero_voltage)
// {
//     float vadc = ((float)adc_raw * vdda) / 4095.0f;
//     return (vadc - zero_voltage) / (BRIDGE1_INA_RSHUNT_OHM * BRIDGE1_INA_GAIN);
// }
