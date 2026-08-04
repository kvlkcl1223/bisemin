// #ifndef __BRIDGE1_TEST_H
// #define __BRIDGE1_TEST_H

// #include "main.h"
// #include <stdint.h>

// typedef enum
// {
//     BRIDGE1_OK = 0,
//     BRIDGE1_FAULT_ACTIVE,
//     BRIDGE1_WDFLT_ACTIVE,
//     BRIDGE1_PARAM_ERROR,
//     BRIDGE1_SPI_ERROR
// } Bridge1_Status_t;

// #define DRV8703_FAULT_BIT_FAULT (1U << 7)
// #define DRV8703_FAULT_BIT_WDFLT (1U << 6)
// #define DRV8703_FAULT_BIT_GDF (1U << 5)
// #define DRV8703_FAULT_BIT_OCP (1U << 4)
// #define DRV8703_FAULT_BIT_VM_UVFL (1U << 3)
// #define DRV8703_FAULT_BIT_VCP_UVFL (1U << 2)
// #define DRV8703_FAULT_BIT_OTSD (1U << 1)
// #define DRV8703_FAULT_BIT_OTW (1U << 0)

// #define DRV8703_VDSGDF_BIT_H2_GDF (1U << 7)
// #define DRV8703_VDSGDF_BIT_L2_GDF (1U << 6)
// #define DRV8703_VDSGDF_BIT_H1_GDF (1U << 5)
// #define DRV8703_VDSGDF_BIT_L1_GDF (1U << 4)
// #define DRV8703_VDSGDF_BIT_H2_VDS (1U << 3)
// #define DRV8703_VDSGDF_BIT_L2_VDS (1U << 2)
// #define DRV8703_VDSGDF_BIT_H1_VDS (1U << 1)
// #define DRV8703_VDSGDF_BIT_L1_VDS (1U << 0)

// typedef struct
// {
//     uint8_t fault_status;
//     uint8_t vds_gdf_status;
// } DRV8703_FaultInfo_t;

// extern volatile DRV8703_FaultInfo_t g_bridge1_last_fault;
// extern volatile Bridge1_Status_t g_bridge1_last_spi_status;
// extern volatile Bridge1_Status_t g_bridge1_test_result;
// extern volatile uint8_t g_bridge1_test_phase;

// extern volatile uint8_t g_drv8703_last_tx0;
// extern volatile uint8_t g_drv8703_last_tx1;
// extern volatile uint8_t g_drv8703_last_rx0;
// extern volatile uint8_t g_drv8703_last_rx1;
// extern volatile uint16_t g_drv8703_last_tx16;
// extern volatile uint16_t g_drv8703_last_rx16;

// extern volatile uint8_t g_drv8703_reg_dump[8];

// extern volatile uint8_t g_static_reg0_before_clear;
// extern volatile uint8_t g_static_reg1_before_clear;
// extern volatile uint8_t g_static_reg0_after_clear;
// extern volatile uint8_t g_static_reg1_after_clear;

// extern volatile uint8_t g_pwm_reg0_after_20ms;
// extern volatile uint8_t g_pwm_reg1_after_20ms;
// extern volatile uint8_t g_pwm_reg0_after_200ms;
// extern volatile uint8_t g_pwm_reg1_after_200ms;
// extern volatile uint8_t g_pwm_reg0_after_1000ms;
// extern volatile uint8_t g_pwm_reg1_after_1000ms;

// extern volatile uint8_t g_monitor_reg0;
// extern volatile uint8_t g_monitor_reg1;

// extern volatile uint8_t g_main_before;
// extern volatile uint8_t g_main_write_value;
// extern volatile uint8_t g_main_after_write;
// extern volatile uint8_t g_main_after_clear0;

// extern volatile uint8_t g_idrive_wd_before;
// extern volatile uint8_t g_idrive_wd_after;

// extern volatile uint8_t g_vds_ctrl_before;
// extern volatile uint8_t g_vds_ctrl_after;

// void Bridge1_InitSafeState(void);
// void Bridge1_SetVref_mV(uint16_t mv);
// void Bridge1_Enable(void);
// void Bridge1_Disable(void);
// void Bridge1_SetDuty(float duty);
// Bridge1_Status_t Bridge1_CheckFaultPins(void);

// uint8_t DRV8703_ReadReg(uint8_t addr);
// Bridge1_Status_t DRV8703_ReadRegChecked(uint8_t addr, uint8_t *data);
// Bridge1_Status_t DRV8703_WriteReg(uint8_t addr, uint8_t data);

// Bridge1_Status_t Bridge1_ReadFaultInfo(DRV8703_FaultInfo_t *info);
// Bridge1_Status_t Bridge1_ClearFault(void);
// Bridge1_Status_t Bridge1_DisableWatchdog(void);
// void Bridge1_DumpRegs(void);

// Bridge1_Status_t Bridge1_StaticMosCheck(void);
// Bridge1_Status_t Bridge1_StartMosPwmHoldTest(float duty, uint16_t vref_mv);
// void Bridge1_StopHoldTest(void);
// void Bridge1_MonitorDuringPwm(void);

// float Bridge1_CurrentFromAdcRaw(uint16_t adc_raw, float vdda, float zero_voltage);

// #endif
