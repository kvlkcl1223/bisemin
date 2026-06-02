#include "drv8703.h"
#include <string.h>

static float DRV8703_LimitFloat(float value, float min_value, float max_value)
{
    if (value > max_value)
        return max_value;
    if (value < min_value)
        return min_value;
    return value;
}

static uint8_t DRV8703_IsValidDevice(const DRV8703_Handle_t *dev)
{
    return (dev != 0) &&
           (dev->cfg.spi != 0) &&
           (dev->cfg.pwm_timer != 0) &&
           (dev->cfg.cs_port != 0) &&
           (dev->cfg.sleep_port != 0) &&
           (dev->cfg.dir_port != 0);
}

static uint32_t DRV8703_DutyToCompare(DRV8703_Handle_t *dev, float duty_abs)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(dev->cfg.pwm_timer);
    duty_abs = DRV8703_LimitFloat(duty_abs, 0.0f, dev->cfg.max_abs_duty);
    return (uint32_t)((float)(arr + 1U) * duty_abs);
}

static void DRV8703_SetPwmRaw(DRV8703_Handle_t *dev, uint32_t compare)
{
    __HAL_TIM_SET_COMPARE(dev->cfg.pwm_timer, dev->cfg.pwm_channel, compare);
}

static void DRV8703_CsLow(DRV8703_Handle_t *dev)
{
    HAL_GPIO_WritePin(dev->cfg.cs_port, dev->cfg.cs_pin, GPIO_PIN_RESET);
}

static void DRV8703_CsHigh(DRV8703_Handle_t *dev)
{
    HAL_GPIO_WritePin(dev->cfg.cs_port, dev->cfg.cs_pin, GPIO_PIN_SET);
}

static DRV8703_Status_t DRV8703_Transfer16(DRV8703_Handle_t *dev, uint16_t tx, uint16_t *rx)
{
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];
    HAL_StatusTypeDef hal_ret;

    if (!DRV8703_IsValidDevice(dev) || (rx == 0))
        return DRV8703_ERROR_PARAM;

    tx_buf[0] = (uint8_t)((tx >> 8) & 0xFFU);
    tx_buf[1] = (uint8_t)(tx & 0xFFU);
    rx_buf[0] = 0U;
    rx_buf[1] = 0U;

    DRV8703_CsHigh(dev);
    HAL_Delay(1U);
    DRV8703_CsLow(dev);
    HAL_Delay(1U);
    hal_ret = HAL_SPI_TransmitReceive(dev->cfg.spi,
                                      tx_buf,
                                      rx_buf,
                                      2U,
                                      dev->cfg.spi_timeout_ms);
    DRV8703_CsHigh(dev);
    HAL_Delay(1U);

    dev->last_tx = tx;
    dev->last_rx = ((uint16_t)rx_buf[0] << 8) | (uint16_t)rx_buf[1];
    *rx = dev->last_rx;

    if (hal_ret != HAL_OK)
        return DRV8703_ERROR_SPI;

    return DRV8703_OK;
}

DRV8703_Status_t DRV8703_Init(DRV8703_Handle_t *dev, const DRV8703_Config_t *cfg)
{
    if ((dev == 0) || (cfg == 0))
        return DRV8703_ERROR_PARAM;

    memset(dev, 0, sizeof(*dev));
    dev->cfg = *cfg;

    if (dev->cfg.spi_timeout_ms == 0U)
        dev->cfg.spi_timeout_ms = 10U;
    if (dev->cfg.wake_delay_ms == 0U)
        dev->cfg.wake_delay_ms = 2U;
    if (dev->cfg.dac_vdda_mv == 0U)
        dev->cfg.dac_vdda_mv = 3300U;
    if (dev->cfg.max_abs_duty <= 0.0f)
        dev->cfg.max_abs_duty = 0.95f;

    if (!DRV8703_IsValidDevice(dev))
        return DRV8703_ERROR_PARAM;

    dev->initialized = 1U;
    return DRV8703_InitSafeState(dev);
}

DRV8703_Status_t DRV8703_InitSafeState(DRV8703_Handle_t *dev)
{
    if (!DRV8703_IsValidDevice(dev))
        return DRV8703_ERROR_PARAM;

    DRV8703_CsHigh(dev);
    HAL_GPIO_WritePin(dev->cfg.dir_port, dev->cfg.dir_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(dev->cfg.sleep_port, dev->cfg.sleep_pin, GPIO_PIN_RESET);

    if (HAL_TIM_PWM_Start(dev->cfg.pwm_timer, dev->cfg.pwm_channel) != HAL_OK)
        return DRV8703_ERROR_HAL;

    DRV8703_SetPwmRaw(dev, 0U);

    if (dev->cfg.use_vref_dac != 0U)
    {
        if (dev->cfg.vref_dac == 0)
            return DRV8703_ERROR_PARAM;
        if (HAL_DAC_Start(dev->cfg.vref_dac, dev->cfg.vref_dac_channel) != HAL_OK)
            return DRV8703_ERROR_HAL;
        if (DRV8703_SetVrefMv(dev, 0U) != DRV8703_OK)
            return DRV8703_ERROR_HAL;
    }

    if (dev->cfg.use_vref_opamp != 0U)
    {
        if (dev->cfg.vref_opamp == 0)
            return DRV8703_ERROR_PARAM;
        if (HAL_OPAMP_Start(dev->cfg.vref_opamp) != HAL_OK)
            return DRV8703_ERROR_HAL;
    }

    return DRV8703_OK;
}

DRV8703_Status_t DRV8703_Wake(DRV8703_Handle_t *dev)
{
    if (!DRV8703_IsValidDevice(dev))
        return DRV8703_ERROR_PARAM;

    DRV8703_SetPwmRaw(dev, 0U);
    HAL_GPIO_WritePin(dev->cfg.sleep_port, dev->cfg.sleep_pin, GPIO_PIN_SET);
    HAL_Delay(dev->cfg.wake_delay_ms);
    return DRV8703_OK;
}

DRV8703_Status_t DRV8703_Sleep(DRV8703_Handle_t *dev)
{
    if (!DRV8703_IsValidDevice(dev))
        return DRV8703_ERROR_PARAM;

    DRV8703_SetPwmRaw(dev, 0U);
    HAL_GPIO_WritePin(dev->cfg.sleep_port, dev->cfg.sleep_pin, GPIO_PIN_RESET);
    return DRV8703_OK;
}

DRV8703_Status_t DRV8703_SetDuty(DRV8703_Handle_t *dev, float duty)
{
    if (!DRV8703_IsValidDevice(dev))
        return DRV8703_ERROR_PARAM;

    duty = DRV8703_LimitFloat(duty, -dev->cfg.max_abs_duty, dev->cfg.max_abs_duty);

    if (duty > 0.0f)
    {
        HAL_GPIO_WritePin(dev->cfg.dir_port, dev->cfg.dir_pin, GPIO_PIN_SET);
        DRV8703_SetPwmRaw(dev, DRV8703_DutyToCompare(dev, duty));
    }
    else if (duty < 0.0f)
    {
        HAL_GPIO_WritePin(dev->cfg.dir_port, dev->cfg.dir_pin, GPIO_PIN_RESET);
        DRV8703_SetPwmRaw(dev, DRV8703_DutyToCompare(dev, -duty));
    }
    else
    {
        DRV8703_SetPwmRaw(dev, 0U);
    }

    return DRV8703_OK;
}

DRV8703_Status_t DRV8703_SetVrefMv(DRV8703_Handle_t *dev, uint16_t mv)
{
    uint32_t dac_value;

    if (!DRV8703_IsValidDevice(dev))
        return DRV8703_ERROR_PARAM;
    if (dev->cfg.use_vref_dac == 0U)
        return DRV8703_OK;
    if ((dev->cfg.vref_dac == 0) || (dev->cfg.dac_vdda_mv == 0U))
        return DRV8703_ERROR_PARAM;

    if (mv > dev->cfg.dac_vdda_mv)
        mv = dev->cfg.dac_vdda_mv;

    dac_value = ((uint32_t)mv * 4095U) / (uint32_t)dev->cfg.dac_vdda_mv;
    if (HAL_DAC_SetValue(dev->cfg.vref_dac,
                         dev->cfg.vref_dac_channel,
                         DAC_ALIGN_12B_R,
                         dac_value) != HAL_OK)
    {
        return DRV8703_ERROR_HAL;
    }

    return DRV8703_OK;
}

DRV8703_Status_t DRV8703_ReadReg(DRV8703_Handle_t *dev, uint8_t addr, uint8_t *data)
{
    uint16_t tx;
    uint16_t rx = 0U;
    DRV8703_Status_t ret;

    if ((data == 0) || (addr > 0x0FU))
        return DRV8703_ERROR_PARAM;

    tx = (uint16_t)(0x8000U | ((uint16_t)(addr & 0x0FU) << 11));
    ret = DRV8703_Transfer16(dev, tx, &rx);
    if (ret != DRV8703_OK)
    {
        *data = 0xFFU;
        return ret;
    }

    *data = (uint8_t)(rx & 0x00FFU);
    if (addr < DRV8703_REGISTER_COUNT)
        dev->last_reg[addr] = *data;

    return DRV8703_OK;
}

DRV8703_Status_t DRV8703_WriteReg(DRV8703_Handle_t *dev, uint8_t addr, uint8_t data)
{
    uint16_t tx;
    uint16_t rx = 0U;
    DRV8703_Status_t ret;

    if (addr > 0x0FU)
        return DRV8703_ERROR_PARAM;

    tx = (uint16_t)(((uint16_t)(addr & 0x0FU) << 11) | (uint16_t)data);
    ret = DRV8703_Transfer16(dev, tx, &rx);
    if ((ret == DRV8703_OK) && (addr < DRV8703_REGISTER_COUNT))
        dev->last_reg[addr] = data;

    return ret;
}

DRV8703_Status_t DRV8703_ReadFaultInfo(DRV8703_Handle_t *dev, DRV8703_FaultInfo_t *info)
{
    DRV8703_Status_t ret;

    if (info == 0)
        return DRV8703_ERROR_PARAM;

    info->fault_status = 0xFFU;
    info->vds_gdf_status = 0xFFU;

    ret = DRV8703_ReadReg(dev, DRV8703_REG_FAULT_STATUS, &info->fault_status);
    if (ret != DRV8703_OK)
        return ret;

    ret = DRV8703_ReadReg(dev, DRV8703_REG_VDS_GDF_STATUS, &info->vds_gdf_status);
    if (ret != DRV8703_OK)
        return ret;

    return DRV8703_OK;
}

DRV8703_Status_t DRV8703_ClearFault(DRV8703_Handle_t *dev)
{
    DRV8703_Status_t ret;

    /*
     * Do not read-modify-write MAIN_CONTROL here. If SPI readback is invalid
     * during wake-up/debug, using that value can overwrite the unlock command.
     */
    ret = DRV8703_WriteReg(dev,
                           DRV8703_REG_MAIN_CONTROL,
                           (uint8_t)(DRV8703_MAIN_LOCK_UNLOCK | DRV8703_MAIN_CLR_FLT));
    if (ret != DRV8703_OK)
        return ret;

    HAL_Delay(1U);
    return DRV8703_WriteReg(dev,
                            DRV8703_REG_MAIN_CONTROL,
                            DRV8703_MAIN_LOCK_UNLOCK);
}

DRV8703_Status_t DRV8703_CheckFaultPins(DRV8703_Handle_t *dev)
{
    GPIO_PinState fault_active;
    GPIO_PinState wdflt_active;

    if (!DRV8703_IsValidDevice(dev))
        return DRV8703_ERROR_PARAM;

    if (dev->cfg.fault_port != 0)
    {
        fault_active = HAL_GPIO_ReadPin(dev->cfg.fault_port, dev->cfg.fault_pin);
        if (dev->cfg.active_low_fault_pins != 0U)
            fault_active = (fault_active == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        if (fault_active == GPIO_PIN_SET)
        {
            // (void)DRV8703_SetDuty(dev, 0.0f);
            return DRV8703_ERROR_FAULT_PIN;
        }
    }

    if (dev->cfg.wdflt_port != 0)
    {
        wdflt_active = HAL_GPIO_ReadPin(dev->cfg.wdflt_port, dev->cfg.wdflt_pin);
        if (dev->cfg.active_low_fault_pins != 0U)
            wdflt_active = (wdflt_active == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        if (wdflt_active == GPIO_PIN_SET)
        {
            // (void)DRV8703_SetDuty(dev, 0.0f);
            return DRV8703_ERROR_WDFLT_PIN;
        }
    }

    return DRV8703_OK;
}

DRV8703_Status_t DRV8703_DumpRegs(DRV8703_Handle_t *dev, uint8_t *regs, uint8_t count)
{
    uint8_t i;
    uint8_t local;
    DRV8703_Status_t ret;

    if (count > DRV8703_REGISTER_COUNT)
        count = DRV8703_REGISTER_COUNT;

    for (i = 0U; i < count; i++)
    {
        ret = DRV8703_ReadReg(dev, i, &local);
        if (ret != DRV8703_OK)
            return ret;
        if (regs != 0)
            regs[i] = local;
    }

    return DRV8703_OK;
}

DRV8703_Status_t DRV8703_Unlock(DRV8703_Handle_t *dev)
{
    return DRV8703_WriteReg(dev, DRV8703_REG_MAIN_CONTROL, DRV8703_MAIN_LOCK_UNLOCK);
}

DRV8703_Status_t DRV8703_Lock(DRV8703_Handle_t *dev)
{
    return DRV8703_WriteReg(dev, DRV8703_REG_MAIN_CONTROL, DRV8703_MAIN_LOCK_LOCK);
}

DRV8703_DeviceConfig_t DRV8703_DefaultDeviceConfig(void)
{
    DRV8703_DeviceConfig_t config;

    config.dead_time = DRV8703_TDEAD_120NS;
    config.watchdog_enable = 0U;
    config.watchdog_delay = DRV8703_WD_DELAY_10MS;
    config.idrive = 7U;
    config.vds_threshold = DRV8703_VDS_960MV;
    config.vds_disable_mask = 0U;
    config.so_limit_enable = 0U;
    config.toff = DRV8703_TOFF_25US;
    config.current_chop_disable = 0U;
    config.vref_scale = DRV8703_VREF_SCALE_100;
    config.sample_hold_enable = 0U;
    config.sense_gain = DRV8703_GAIN_19V8V;

    return config;
}

DRV8703_Status_t DRV8703_Configure(DRV8703_Handle_t *dev, const DRV8703_DeviceConfig_t *config)
{
    uint8_t reg3;
    uint8_t reg4;
    uint8_t reg5;
    DRV8703_Status_t ret;

    if (config == 0)
        return DRV8703_ERROR_PARAM;

    ret = DRV8703_Unlock(dev);
    if (ret != DRV8703_OK)
        return ret;

    reg3 = (uint8_t)(((uint8_t)config->dead_time & 0x03U) << DRV8703_IDRIVE_WD_TDEAD_SHIFT);
    if (config->watchdog_enable != 0U)
        reg3 |= DRV8703_IDRIVE_WD_WD_EN;
    reg3 |= (uint8_t)(((uint8_t)config->watchdog_delay & 0x03U) << DRV8703_IDRIVE_WD_WD_DLY_SHIFT);
    reg3 |= (uint8_t)(config->idrive & DRV8703_IDRIVE_WD_IDRIVE_MASK);

    reg4 = (uint8_t)(((uint8_t)config->vds_threshold & 0x07U) << DRV8703_VDS_THRESHOLD_SHIFT);
    if (config->so_limit_enable != 0U)
        reg4 |= DRV8703_VDS_SO_LIM;
    reg4 |= (uint8_t)(config->vds_disable_mask & 0x0FU);

    reg5 = (uint8_t)(((uint8_t)config->toff & 0x03U) << DRV8703_CONFIG_TOFF_SHIFT);
    if (config->current_chop_disable != 0U)
        reg5 |= DRV8703_CONFIG_CHOP_IDS;
    reg5 |= (uint8_t)(((uint8_t)config->vref_scale & 0x03U) << DRV8703_CONFIG_VREF_SCL_SHIFT);
    if (config->sample_hold_enable != 0U)
        reg5 |= DRV8703_CONFIG_SH_EN;
    reg5 |= (uint8_t)((uint8_t)config->sense_gain & DRV8703_CONFIG_GAIN_CS_MASK);

    ret = DRV8703_WriteReg(dev, DRV8703_REG_IDRIVE_WD_CONTROL, reg3);
    if (ret != DRV8703_OK)
        return ret;
    ret = DRV8703_WriteReg(dev, DRV8703_REG_VDS_CONTROL, reg4);
    if (ret != DRV8703_OK)
        return ret;
    ret = DRV8703_WriteReg(dev, DRV8703_REG_CONFIG_CONTROL, reg5);
    if (ret != DRV8703_OK)
        return ret;

    return DRV8703_ClearFault(dev);
}

float DRV8703_CurrentFromSenseVoltage(float sense_voltage, float zero_voltage,
                                      float shunt_ohm, float gain_vv)
{
    if ((shunt_ohm <= 0.0f) || (gain_vv <= 0.0f))
        return 0.0f;
    return (sense_voltage - zero_voltage) / (shunt_ohm * gain_vv);
}

float DRV8703_CurrentFromAdcRaw(uint16_t adc_raw, float vdda, float zero_voltage,
                                float shunt_ohm, float gain_vv)
{
    float vadc = ((float)adc_raw * vdda) / 4095.0f;
    return DRV8703_CurrentFromSenseVoltage(vadc, zero_voltage, shunt_ohm, gain_vv);
}
