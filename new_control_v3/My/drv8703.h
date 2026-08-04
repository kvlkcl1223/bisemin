#ifndef __DRV8703_H
#define __DRV8703_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRV8703_REG_FAULT_STATUS       0x00U
#define DRV8703_REG_VDS_GDF_STATUS     0x01U
#define DRV8703_REG_MAIN_CONTROL       0x02U
#define DRV8703_REG_IDRIVE_WD_CONTROL  0x03U
#define DRV8703_REG_VDS_CONTROL        0x04U
#define DRV8703_REG_CONFIG_CONTROL     0x05U
#define DRV8703_REGISTER_COUNT         6U

#define DRV8703_FAULT_FAULT            (1U << 7)
#define DRV8703_FAULT_WDFLT            (1U << 6)
#define DRV8703_FAULT_GDF              (1U << 5)
#define DRV8703_FAULT_OCP              (1U << 4)
#define DRV8703_FAULT_VM_UVFL          (1U << 3)
#define DRV8703_FAULT_VCP_UVFL         (1U << 2)
#define DRV8703_FAULT_OTSD             (1U << 1)
#define DRV8703_FAULT_OTW              (1U << 0)

#define DRV8703_VDSGDF_H2_GDF          (1U << 7)
#define DRV8703_VDSGDF_L2_GDF          (1U << 6)
#define DRV8703_VDSGDF_H1_GDF          (1U << 5)
#define DRV8703_VDSGDF_L1_GDF          (1U << 4)
#define DRV8703_VDSGDF_H2_VDS          (1U << 3)
#define DRV8703_VDSGDF_L2_VDS          (1U << 2)
#define DRV8703_VDSGDF_H1_VDS          (1U << 1)
#define DRV8703_VDSGDF_L1_VDS          (1U << 0)

#define DRV8703_MAIN_LOCK_UNLOCK        (3U << 3)
#define DRV8703_MAIN_LOCK_LOCK          (6U << 3)
#define DRV8703_MAIN_IN1_PH             (1U << 2)
#define DRV8703_MAIN_IN2_EN             (1U << 1)
#define DRV8703_MAIN_CLR_FLT            (1U << 0)

#define DRV8703_IDRIVE_WD_TDEAD_SHIFT   6U
#define DRV8703_IDRIVE_WD_WD_EN         (1U << 5)
#define DRV8703_IDRIVE_WD_WD_DLY_SHIFT  3U
#define DRV8703_IDRIVE_WD_IDRIVE_MASK   0x07U

#define DRV8703_VDS_SO_LIM              (1U << 7)
#define DRV8703_VDS_THRESHOLD_SHIFT     4U
#define DRV8703_VDS_DISABLE_H2          (1U << 3)
#define DRV8703_VDS_DISABLE_L2          (1U << 2)
#define DRV8703_VDS_DISABLE_H1          (1U << 1)
#define DRV8703_VDS_DISABLE_L1          (1U << 0)

#define DRV8703_CONFIG_TOFF_SHIFT       6U
#define DRV8703_CONFIG_CHOP_IDS         (1U << 5)
#define DRV8703_CONFIG_VREF_SCL_SHIFT   3U
#define DRV8703_CONFIG_SH_EN            (1U << 2)
#define DRV8703_CONFIG_GAIN_CS_MASK     0x03U

typedef enum
{
    DRV8703_OK = 0,
    DRV8703_ERROR_PARAM,
    DRV8703_ERROR_SPI,
    DRV8703_ERROR_HAL,
    DRV8703_ERROR_FAULT_PIN,
    DRV8703_ERROR_WDFLT_PIN,
    DRV8703_ERROR_FAULT_REG
} DRV8703_Status_t;

typedef enum
{
    DRV8703_TDEAD_120NS = 0,
    DRV8703_TDEAD_240NS = 1,
    DRV8703_TDEAD_480NS = 2,
    DRV8703_TDEAD_960NS = 3
} DRV8703_DeadTime_t;

typedef enum
{
    DRV8703_WD_DELAY_10MS = 0,
    DRV8703_WD_DELAY_20MS = 1,
    DRV8703_WD_DELAY_50MS = 2,
    DRV8703_WD_DELAY_100MS = 3
} DRV8703_WatchdogDelay_t;

typedef enum
{
    DRV8703_VDS_60MV = 0,
    DRV8703_VDS_145MV = 1,
    DRV8703_VDS_170MV = 2,
    DRV8703_VDS_200MV = 3,
    DRV8703_VDS_120MV = 4,
    DRV8703_VDS_240MV = 5,
    DRV8703_VDS_480MV = 6,
    DRV8703_VDS_960MV = 7
} DRV8703_VdsThreshold_t;

typedef enum
{
    DRV8703_TOFF_25US = 0,
    DRV8703_TOFF_50US = 1,
    DRV8703_TOFF_100US = 2,
    DRV8703_TOFF_200US = 3
} DRV8703_Toff_t;

typedef enum
{
    DRV8703_VREF_SCALE_100 = 0,
    DRV8703_VREF_SCALE_75 = 1,
    DRV8703_VREF_SCALE_50 = 2,
    DRV8703_VREF_SCALE_25 = 3
} DRV8703_VrefScale_t;

typedef enum
{
    DRV8703_GAIN_10VV = 0,
    DRV8703_GAIN_19V8V = 1,
    DRV8703_GAIN_39V4V = 2,
    DRV8703_GAIN_78VV = 3
} DRV8703_CurrentSenseGain_t;

typedef struct
{
    uint8_t fault_status;
    uint8_t vds_gdf_status;
} DRV8703_FaultInfo_t;

typedef struct
{
    SPI_HandleTypeDef *spi;
    TIM_HandleTypeDef *pwm_timer;
    uint32_t pwm_channel;

    DAC_HandleTypeDef *vref_dac;
    uint32_t vref_dac_channel;
    OPAMP_HandleTypeDef *vref_opamp;

    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef *sleep_port;
    uint16_t sleep_pin;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    GPIO_TypeDef *fault_port;
    uint16_t fault_pin;
    GPIO_TypeDef *wdflt_port;
    uint16_t wdflt_pin;

    uint16_t spi_timeout_ms;
    uint16_t wake_delay_ms;
    uint16_t dac_vdda_mv;
    float max_abs_duty;
    uint8_t use_vref_dac;
    uint8_t use_vref_opamp;
    uint8_t active_low_fault_pins;
} DRV8703_Config_t;

typedef struct
{
    DRV8703_Config_t cfg;
    uint16_t last_tx;
    uint16_t last_rx;
    uint8_t last_reg[DRV8703_REGISTER_COUNT];
    uint8_t initialized;
} DRV8703_Handle_t;

typedef struct
{
    DRV8703_DeadTime_t dead_time;
    uint8_t watchdog_enable;
    DRV8703_WatchdogDelay_t watchdog_delay;
    uint8_t idrive;
    DRV8703_VdsThreshold_t vds_threshold;
    uint8_t vds_disable_mask;
    uint8_t so_limit_enable;
    DRV8703_Toff_t toff;
    uint8_t current_chop_disable;
    DRV8703_VrefScale_t vref_scale;
    uint8_t sample_hold_enable;
    DRV8703_CurrentSenseGain_t sense_gain;
} DRV8703_DeviceConfig_t;

DRV8703_Status_t DRV8703_Init(DRV8703_Handle_t *dev, const DRV8703_Config_t *cfg);
DRV8703_Status_t DRV8703_InitSafeState(DRV8703_Handle_t *dev);
DRV8703_Status_t DRV8703_Wake(DRV8703_Handle_t *dev);
DRV8703_Status_t DRV8703_Sleep(DRV8703_Handle_t *dev);
DRV8703_Status_t DRV8703_SetDuty(DRV8703_Handle_t *dev, float duty);
DRV8703_Status_t DRV8703_SetVrefMv(DRV8703_Handle_t *dev, uint16_t mv);

DRV8703_Status_t DRV8703_ReadReg(DRV8703_Handle_t *dev, uint8_t addr, uint8_t *data);
DRV8703_Status_t DRV8703_WriteReg(DRV8703_Handle_t *dev, uint8_t addr, uint8_t data);
DRV8703_Status_t DRV8703_ReadFaultInfo(DRV8703_Handle_t *dev, DRV8703_FaultInfo_t *info);
DRV8703_Status_t DRV8703_ClearFault(DRV8703_Handle_t *dev);
DRV8703_Status_t DRV8703_CheckFaultPins(DRV8703_Handle_t *dev);
DRV8703_Status_t DRV8703_DumpRegs(DRV8703_Handle_t *dev, uint8_t *regs, uint8_t count);

DRV8703_Status_t DRV8703_Unlock(DRV8703_Handle_t *dev);
DRV8703_Status_t DRV8703_Lock(DRV8703_Handle_t *dev);
DRV8703_Status_t DRV8703_Configure(DRV8703_Handle_t *dev, const DRV8703_DeviceConfig_t *config);
DRV8703_DeviceConfig_t DRV8703_DefaultDeviceConfig(void);

float DRV8703_CurrentFromSenseVoltage(float sense_voltage, float zero_voltage,
                                      float shunt_ohm, float gain_vv);
float DRV8703_CurrentFromAdcRaw(uint16_t adc_raw, float vdda, float zero_voltage,
                                float shunt_ohm, float gain_vv);

#ifdef __cplusplus
}
#endif

#endif
