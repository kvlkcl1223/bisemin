#include "app_control.h"

#include "adc_measure.h"
#include "cmsis_os.h"
#include "drv8703_board.h"
#include "pid_controller.h"

#define APP_CONTROL_QUEUE_DEPTH 12U
#define APP_CONTROL_DRV_RETRY_COUNT 3U
#define APP_CONTROL_DRV_VREF_MV 2000U
#define APP_CONTROL_VSUPPLY_MIN_V 20.0f
#define APP_CONTROL_TEMP_PID_DT_S 0.2f
#define APP_CONTROL_TEMP_PID_KP 0.035f
#define APP_CONTROL_TEMP_PID_KI 0.002f
#define APP_CONTROL_TEMP_PID_KD 0.0f
#define APP_CONTROL_MAX_ABS_DUTY 0.45f
#define APP_CONTROL_SHARED_CH5_DUTY 0.20f
#define APP_CONTROL_DRV_FAULT_POLL_MS 500U
#define APP_CONTROL_TEMP_TIMEOUT_MS 1000U
#define APP_CONTROL_TEMP_RESET_COOLDOWN_MS 1500U
#define APP_CONTROL_TEMP_RESET_PULSE_MS 20U
#define APP_CONTROL_TEMP_RESET_MAX_COUNT 3U
#define APP_CONTROL_TEMP_PID_DT_DEFAULT_S 0.05f
#define APP_CONTROL_TEMP_PID_DT_MIN_S 0.02f
#define APP_CONTROL_TEMP_PID_DT_MAX_S 1.0f
#define APP_CONTROL_ERROR_DISPLAY_MS 5000U

/*
 * DRV8703 FAULT bit7 is only a summary flag. Stop the cell only for faults
 * that can damage hardware or make output unsafe. Watchdog timeout and over
 * temperature warning are recorded for debug, but they do not latch a panel
 * error or stop the temperature cell by themselves.
 */
#define APP_CONTROL_DRV_STOP_FAULT_MASK \
    (DRV8703_FAULT_GDF |             \
     DRV8703_FAULT_OCP |             \
     DRV8703_FAULT_OTSD)

typedef struct
{
    uint8_t running;
    uint8_t requested;
    uint8_t pid_update_pending;
    uint8_t sensor_reset_count;
    float target_temp;
    float current_temp;
    float duty;
    PanelError_t error;
    uint32_t error_set_ms;
} AppControlCell_t;

volatile uint8_t g_app_control_simulate_drv8703 = 0U;
volatile uint8_t g_app_control_simulate_voltage_ok = 0U;
volatile AppControl_Status_t g_app_control_init_result = APP_CONTROL_ERROR_QUEUE;
volatile uint32_t g_app_control_loop_count = 0U;
volatile uint32_t g_app_control_cmd_drop_count = 0U;
volatile uint8_t g_app_control_cell_running[APP_CONTROL_CELL_COUNT] = {0};
volatile PanelError_t g_app_control_cell_error[APP_CONTROL_CELL_COUNT] = {PANEL_ERR_NONE, PANEL_ERR_NONE};
volatile float g_app_control_cell_temp[APP_CONTROL_CELL_COUNT] = {25.0f, 25.0f};
volatile float g_app_control_cell_target[APP_CONTROL_CELL_COUNT] = {25.0f, 25.0f};
volatile float g_app_control_cell_duty[APP_CONTROL_CELL_COUNT] = {0.0f, 0.0f};
volatile uint8_t g_app_control_drv_init_attempts[APP_CONTROL_DRV_COUNT] = {0};
volatile uint8_t g_app_control_drv_ready[APP_CONTROL_DRV_COUNT] = {0};
volatile uint8_t g_app_control_drv_awake[APP_CONTROL_DRV_COUNT] = {0};
volatile uint8_t g_app_control_drv_fault[APP_CONTROL_DRV_COUNT] = {0};
volatile uint8_t g_app_control_drv_startup_dump_valid[APP_CONTROL_DRV_COUNT] = {0};
volatile DRV8703_Status_t g_app_control_drv_startup_dump_status[APP_CONTROL_DRV_COUNT] =
{
    DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK
};
volatile uint8_t g_app_control_drv_startup_reg_dump[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT] =
{
    {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
    {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
    {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
    {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
    {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU}
};
volatile uint8_t g_app_control_drv_startup_expected[DRV8703_REGISTER_COUNT] =
{
    0x00U, /* reg0 FAULT_STATUS: no active/latched fault after ClearFault */
    0x00U, /* reg1 VDS_GDF_STATUS: no VDS/GDF event after ClearFault */
    0x18U, /* reg2 MAIN_CONTROL: LOCK=unlock code, CLR_FLT=0 */
    0x07U, /* reg3 IDRIVE_WD: watchdog off, 120 ns dead time, IDRIVE=7 */
    0x70U, /* reg4 VDS_CONTROL: VDS threshold=960 mV, no VDS bits disabled */
    0x01U  /* reg5 CONFIG: TOFF=25 us, VREF=100%, SH off, gain=19.8 V/V */
};
volatile uint8_t g_app_control_last_drv_fault = 0xFFU;
volatile DRV8703_Status_t g_app_control_last_drv_status = DRV8703_OK;
volatile uint8_t g_app_control_drv_fault_snapshot_valid[APP_CONTROL_DRV_COUNT] = {0};
volatile uint32_t g_app_control_drv_fault_capture_count[APP_CONTROL_DRV_COUNT] = {0};
volatile DRV8703_Status_t g_app_control_drv_fault_read_status[APP_CONTROL_DRV_COUNT] =
{
    DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK
};
volatile DRV8703_Status_t g_app_control_drv_dump_status[APP_CONTROL_DRV_COUNT] =
{
    DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK
};
volatile uint8_t g_app_control_drv_fault_status[APP_CONTROL_DRV_COUNT] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
volatile uint8_t g_app_control_drv_vds_gdf_status[APP_CONTROL_DRV_COUNT] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
volatile uint8_t g_app_control_drv_reg_dump[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT] =
{
    {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
    {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
    {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
    {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
    {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU}
};
volatile uint8_t g_app_control_drv_pin_fault_last = 0xFFU;
volatile uint32_t g_app_control_drv_pin_fault_count[APP_CONTROL_DRV_COUNT] = {0};
volatile DRV8703_Status_t g_app_control_drv_pin_fault_status[APP_CONTROL_DRV_COUNT] =
{
    DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK
};
volatile DRV8703_Status_t g_app_control_drv_pin_fault_read_status[APP_CONTROL_DRV_COUNT] =
{
    DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK
};
volatile DRV8703_Status_t g_app_control_drv_pin_fault_dump_status[APP_CONTROL_DRV_COUNT] =
{
    DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK
};
volatile uint8_t g_app_control_drv_pin_fault_fault_status[APP_CONTROL_DRV_COUNT] =
{
    0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU
};
volatile uint8_t g_app_control_drv_pin_fault_vds_gdf_status[APP_CONTROL_DRV_COUNT] =
{
    0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU
};
volatile uint8_t g_app_control_drv_pin_fault_stop_bits[APP_CONTROL_DRV_COUNT] = {0};
volatile uint8_t g_app_control_drv_pin_fault_reg_dump[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT] =
{
    {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
    {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
    {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
    {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
    {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU}
};
volatile uint8_t g_app_control_temp_reset_count[APP_CONTROL_CELL_COUNT] = {0};
volatile uint32_t g_app_control_temp_last_update_tick[4] = {0};
volatile uint32_t g_app_control_temp_update_count[4] = {0};
volatile uint8_t g_app_control_temp_fault_sensor[APP_CONTROL_CELL_COUNT] = {0xFFU, 0xFFU};
volatile uint8_t g_app_control_temp_reset_active = 0U;

static osMessageQueueId_t s_cmd_queue;
static AppControlCell_t s_cell[APP_CONTROL_CELL_COUNT];
static PID_TypeDef s_temp_pid[APP_CONTROL_CELL_COUNT];
static uint32_t s_last_fault_poll_ms = 0U;
static uint32_t s_temp_last_pid_count[APP_CONTROL_CELL_COUNT][2] = {{0U, 0U}, {0U, 0U}};
static uint32_t s_temp_last_pid_ms[APP_CONTROL_CELL_COUNT] = {0U, 0U};
static uint32_t s_temp_reset_release_ms = 0U;
static uint32_t s_temp_last_reset_ms = 0U;

extern osMutexId_t SysStateMutexHandle;
extern volatile float Sys_Temperatures[4];
extern volatile uint8_t Sys_Status[4];
extern volatile uint32_t Sys_TempUpdateCount[4];
extern volatile uint32_t Sys_TempUpdateTick[4];

static void AppControl_Lock(void)
{
    (void)osMutexAcquire(SysStateMutexHandle, osWaitForever);
}

static void AppControl_Unlock(void)
{
    (void)osMutexRelease(SysStateMutexHandle);
}

static float AppControl_Abs(float v)
{
    return (v < 0.0f) ? -v : v;
}

static float AppControl_Clamp(float v, float min_v, float max_v)
{
    if (v < min_v)
        return min_v;
    if (v > max_v)
        return max_v;
    return v;
}

static void AppControl_ServiceTempSensorReset(uint32_t now_ms)
{
    if ((g_app_control_temp_reset_active != 0U) &&
        ((now_ms - s_temp_reset_release_ms) < 0x80000000UL))
    {
        HAL_GPIO_WritePin(NRST_OTHER_GPIO_Port, NRST_OTHER_Pin, GPIO_PIN_SET);
        g_app_control_temp_reset_active = 0U;
    }
}

static uint8_t AppControl_RequestTempSensorReset(uint32_t now_ms)
{
    if (g_app_control_temp_reset_active != 0U)
        return 0U;
    if ((now_ms - s_temp_last_reset_ms) < APP_CONTROL_TEMP_RESET_COOLDOWN_MS)
        return 0U;

    HAL_GPIO_WritePin(NRST_OTHER_GPIO_Port, NRST_OTHER_Pin, GPIO_PIN_RESET);
    s_temp_reset_release_ms = now_ms + APP_CONTROL_TEMP_RESET_PULSE_MS;
    s_temp_last_reset_ms = now_ms;
    g_app_control_temp_reset_active = 1U;
    return 1U;
}

static uint8_t AppControl_CellFirstDrv(uint8_t cell)
{
    return (cell == 0U) ? 0U : 2U;
}

static PanelError_t AppControl_CellDrvError(uint8_t cell)
{
    return (cell == 0U) ? PANEL_ERR_E311_CELL1_DRV : PANEL_ERR_E312_CELL2_DRV;
}

static PanelError_t AppControl_CellVoltageError(uint8_t cell)
{
    return (cell == 0U) ? PANEL_ERR_E301_CELL1_VOLTAGE : PANEL_ERR_E302_CELL2_VOLTAGE;
}

static void AppControl_CaptureDrvStartupRegs(uint8_t drv, DRV8703_Handle_t *dev)
{
    uint8_t regs[DRV8703_REGISTER_COUNT];
    uint8_t i;
    DRV8703_Status_t ret;

    if ((drv >= APP_CONTROL_DRV_COUNT) || (dev == 0))
        return;

    for (i = 0U; i < DRV8703_REGISTER_COUNT; i++)
        g_app_control_drv_startup_reg_dump[drv][i] = 0xFFU;
    g_app_control_drv_startup_dump_valid[drv] = 0U;

    ret = DRV8703_DumpRegs(dev, regs, DRV8703_REGISTER_COUNT);
    g_app_control_drv_startup_dump_status[drv] = ret;
    if (ret == DRV8703_OK)
    {
        for (i = 0U; i < DRV8703_REGISTER_COUNT; i++)
            g_app_control_drv_startup_reg_dump[drv][i] = regs[i];
        g_app_control_drv_startup_dump_valid[drv] = 1U;
    }
}

static void AppControl_CaptureDrvFault(uint8_t drv, DRV8703_Status_t status)
{
    DRV8703_Handle_t *dev;
    DRV8703_FaultInfo_t info;
    uint8_t regs[DRV8703_REGISTER_COUNT];
    uint8_t i;
    DRV8703_Status_t spi_ret;

    if (drv >= APP_CONTROL_DRV_COUNT)
        return;

    g_app_control_last_drv_fault = drv;
    g_app_control_last_drv_status = status;
    g_app_control_drv_fault[drv] = 1U;
    g_app_control_drv_fault_capture_count[drv]++;

    if (g_app_control_simulate_drv8703 != 0U)
        return;

    dev = DRV8703_BoardGet((DRV8703_BoardChannel_t)drv);
    if (dev == 0)
        return;

    if (g_app_control_drv_fault_snapshot_valid[drv] != 0U)
        return;

    spi_ret = DRV8703_ReadFaultInfo(dev, &info);
    g_app_control_drv_fault_read_status[drv] = spi_ret;
    if (spi_ret == DRV8703_OK)
    {
        g_app_control_drv_fault_status[drv] = info.fault_status;
        g_app_control_drv_vds_gdf_status[drv] = info.vds_gdf_status;
        g_app_control_drv_fault_snapshot_valid[drv] = 1U;
    }
    else
    {
        g_app_control_last_drv_status = spi_ret;
    }

    spi_ret = DRV8703_DumpRegs(dev, regs, DRV8703_REGISTER_COUNT);
    g_app_control_drv_dump_status[drv] = spi_ret;
    if (spi_ret == DRV8703_OK)
    {
        for (i = 0U; i < DRV8703_REGISTER_COUNT; i++)
            g_app_control_drv_reg_dump[drv][i] = regs[i];
        g_app_control_drv_fault_snapshot_valid[drv] = 1U;
    }
}

static uint8_t AppControl_DrvFaultRequiresStop(uint8_t fault_status)
{
    uint8_t stop_bits;

    /*
     * Stop faults:
     * - GDF: gate-drive fault
     * - OCP: over-current / VDS over-current protection
     * - OTSD: over-temperature shutdown
     *
     * Non-stop faults:
     * - WDFLT: watchdog timeout. Record it, but do not stop the cell.
     * - VM_UVFL: motor supply undervoltage. Treated as possible false alarm.
     * - VCP_UVFL: charge-pump undervoltage. Treated as possible false alarm.
     * - OTW: over-temperature warning. Record it, but do not stop yet.
     *
     * FAULT bit7 is a summary bit and is intentionally ignored here.
     */
    stop_bits = (uint8_t)(fault_status & APP_CONTROL_DRV_STOP_FAULT_MASK);
    return (stop_bits != 0U) ? 1U : 0U;
}

static uint8_t AppControl_CaptureDrvPinFaultMoment(uint8_t drv, DRV8703_Status_t pin_status)
{
    DRV8703_Handle_t *dev;
    DRV8703_FaultInfo_t info;
    uint8_t regs[DRV8703_REGISTER_COUNT];
    uint8_t i;
    DRV8703_Status_t spi_ret;
    uint8_t should_stop = 0U;

    if (drv >= APP_CONTROL_DRV_COUNT)
        return 0U;

    g_app_control_drv_pin_fault_last = drv;
    g_app_control_drv_pin_fault_count[drv]++;
    g_app_control_drv_pin_fault_status[drv] = pin_status;
    g_app_control_last_drv_fault = drv;
    g_app_control_last_drv_status = pin_status;
    g_app_control_drv_fault[drv] = 1U;
    g_app_control_drv_pin_fault_stop_bits[drv] = 0U;

    g_app_control_drv_pin_fault_fault_status[drv] = 0xFFU;
    g_app_control_drv_pin_fault_vds_gdf_status[drv] = 0xFFU;
    for (i = 0U; i < DRV8703_REGISTER_COUNT; i++)
        g_app_control_drv_pin_fault_reg_dump[drv][i] = 0xFFU;

    if (g_app_control_simulate_drv8703 != 0U)
        return 0U;

    dev = DRV8703_BoardGet((DRV8703_BoardChannel_t)drv);
    if (dev == 0)
        return 0U;

    spi_ret = DRV8703_ReadFaultInfo(dev, &info);
    g_app_control_drv_pin_fault_read_status[drv] = spi_ret;
    if (spi_ret == DRV8703_OK)
    {
        g_app_control_drv_pin_fault_fault_status[drv] = info.fault_status;
        g_app_control_drv_pin_fault_vds_gdf_status[drv] = info.vds_gdf_status;
        g_app_control_drv_pin_fault_stop_bits[drv] =
            (uint8_t)(info.fault_status & APP_CONTROL_DRV_STOP_FAULT_MASK);
        should_stop = AppControl_DrvFaultRequiresStop(info.fault_status);

        if (g_app_control_drv_fault_snapshot_valid[drv] == 0U)
        {
            g_app_control_drv_fault_read_status[drv] = spi_ret;
            g_app_control_drv_fault_status[drv] = info.fault_status;
            g_app_control_drv_vds_gdf_status[drv] = info.vds_gdf_status;
            g_app_control_drv_fault_snapshot_valid[drv] = 1U;
        }
    }

    spi_ret = DRV8703_DumpRegs(dev, regs, DRV8703_REGISTER_COUNT);
    g_app_control_drv_pin_fault_dump_status[drv] = spi_ret;
    if (spi_ret == DRV8703_OK)
    {
        for (i = 0U; i < DRV8703_REGISTER_COUNT; i++)
            g_app_control_drv_pin_fault_reg_dump[drv][i] = regs[i];

        if (g_app_control_drv_fault_snapshot_valid[drv] == 0U)
        {
            g_app_control_drv_dump_status[drv] = spi_ret;
            for (i = 0U; i < DRV8703_REGISTER_COUNT; i++)
                g_app_control_drv_reg_dump[drv][i] = regs[i];
            g_app_control_drv_fault_snapshot_valid[drv] = 1U;
        }
    }

    return should_stop;
}

static uint8_t AppControl_IsTempSensorError(PanelError_t err)
{
    return (err == PANEL_ERR_E121_TEMP_CH1) ||
           (err == PANEL_ERR_E122_TEMP_CH2) ||
           (err == PANEL_ERR_E123_TEMP_CH3) ||
           (err == PANEL_ERR_E124_TEMP_CH4) ||
           (err == PANEL_ERR_E132_SENSOR);
}

static PanelError_t AppControl_TempSensorError(uint8_t sensor)
{
    switch (sensor)
    {
    case 0U:
        return PANEL_ERR_E121_TEMP_CH1;
    case 1U:
        return PANEL_ERR_E122_TEMP_CH2;
    case 2U:
        return PANEL_ERR_E123_TEMP_CH3;
    case 3U:
        return PANEL_ERR_E124_TEMP_CH4;
    default:
        return PANEL_ERR_E132_SENSOR;
    }
}

static uint8_t AppControl_TempInputIsFresh(uint8_t sensor,
                                           const uint8_t status[4],
                                           const uint32_t count[4],
                                           const uint32_t tick[4],
                                           uint32_t now_ms)
{
    if (sensor >= 4U)
        return 0U;
    if (status[sensor] != 0U)
        return 0U;
    if (count[sensor] == 0U)
        return 0U;
    if ((now_ms - tick[sensor]) > APP_CONTROL_TEMP_TIMEOUT_MS)
        return 0U;

    return 1U;
}

static void AppControl_SetCellError(uint8_t cell, PanelError_t err)
{
    if (cell >= APP_CONTROL_CELL_COUNT)
        return;

    if ((err != PANEL_ERR_NONE) && (s_cell[cell].error != err))
        s_cell[cell].error_set_ms = osKernelGetTickCount();
    else if (err == PANEL_ERR_NONE)
        s_cell[cell].error_set_ms = 0U;

    s_cell[cell].error = err;
    g_app_control_cell_error[cell] = err;
    if (!AppControl_IsTempSensorError(err))
        g_app_control_temp_fault_sensor[cell] = 0xFFU;
}

static void AppControl_ServiceErrorDisplayTimeout(uint32_t now_ms)
{
    uint8_t cell;

    for (cell = 0U; cell < APP_CONTROL_CELL_COUNT; cell++)
    {
        if ((s_cell[cell].error != PANEL_ERR_NONE) &&
            (s_cell[cell].error_set_ms != 0U) &&
            ((now_ms - s_cell[cell].error_set_ms) >= APP_CONTROL_ERROR_DISPLAY_MS))
        {
            s_cell[cell].running = 0U;
            s_cell[cell].requested = 0U;
            s_cell[cell].pid_update_pending = 0U;
            s_cell[cell].sensor_reset_count = 0U;
            s_cell[cell].duty = 0.0f;
            PID_Reset(&s_temp_pid[cell]);
            AppControl_SetCellError(cell, PANEL_ERR_NONE);
        }
    }
}

static void AppControl_ApplyDebugState(void)
{
    uint8_t i;

    for (i = 0U; i < APP_CONTROL_CELL_COUNT; i++)
    {
        g_app_control_cell_running[i] = s_cell[i].running;
        g_app_control_cell_temp[i] = s_cell[i].current_temp;
        g_app_control_cell_target[i] = s_cell[i].target_temp;
        g_app_control_cell_duty[i] = s_cell[i].duty;
        g_app_control_cell_error[i] = s_cell[i].error;
    }
}

static DRV8703_Status_t AppControl_PrepareDrv(uint8_t drv)
{
    DRV8703_Handle_t *dev;
    DRV8703_Status_t ret = DRV8703_ERROR_PARAM;
    uint8_t attempt;

    if (drv >= APP_CONTROL_DRV_COUNT)
        return DRV8703_ERROR_PARAM;

    if (g_app_control_simulate_drv8703 != 0U)
    {
        g_app_control_drv_ready[drv] = 1U;
        g_app_control_drv_awake[drv] = 1U;
        g_app_control_drv_fault[drv] = 0U;
        return DRV8703_OK;
    }

    if (g_app_control_drv_ready[drv] != 0U)
        return DRV8703_OK;

    dev = DRV8703_BoardGet((DRV8703_BoardChannel_t)drv);
    if (dev == 0)
        return DRV8703_ERROR_PARAM;

    for (attempt = 1U; attempt <= APP_CONTROL_DRV_RETRY_COUNT; attempt++)
    {
        g_app_control_drv_init_attempts[drv] = attempt;

        ret = DRV8703_BoardInitOne((DRV8703_BoardChannel_t)drv);
        if (ret == DRV8703_OK)
            ret = DRV8703_BoardApplyDefaultConfig((DRV8703_BoardChannel_t)drv);
        if (ret == DRV8703_OK)
            ret = DRV8703_SetVrefMv(dev, APP_CONTROL_DRV_VREF_MV);
        if (ret == DRV8703_OK)
            ret = DRV8703_ClearFault(dev);
        if (ret == DRV8703_OK)
        {
            AppControl_CaptureDrvStartupRegs(drv, dev);
            break;
        }

        AppControl_CaptureDrvFault(drv, ret);
        (void)DRV8703_Sleep(dev);
        osDelay(5);
    }

    g_app_control_drv_ready[drv] = (ret == DRV8703_OK) ? 1U : 0U;
    g_app_control_drv_awake[drv] = (ret == DRV8703_OK) ? 1U : 0U;
    g_app_control_drv_fault[drv] = (ret == DRV8703_OK) ? 0U : 1U;
    return ret;
}

static DRV8703_Status_t AppControl_SetDrvDuty(uint8_t drv, float duty)
{
    DRV8703_Handle_t *dev;
    DRV8703_Status_t ret;

    if (drv >= APP_CONTROL_DRV_COUNT)
        return DRV8703_ERROR_PARAM;

    if (g_app_control_simulate_drv8703 != 0U)
        return DRV8703_OK;

    if ((AppControl_Abs(duty) < 0.001f) && (g_app_control_drv_awake[drv] == 0U))
        return DRV8703_OK;

    dev = DRV8703_BoardGet((DRV8703_BoardChannel_t)drv);
    if (dev == 0)
        return DRV8703_ERROR_PARAM;

    ret = AppControl_PrepareDrv(drv);
    if (ret != DRV8703_OK)
        return ret;

    if (g_app_control_drv_awake[drv] == 0U)
    {
        ret = DRV8703_Wake(dev);
        if (ret != DRV8703_OK)
        {
            g_app_control_drv_awake[drv] = 0U;
            AppControl_CaptureDrvFault(drv, ret);
            return ret;
        }
        g_app_control_drv_awake[drv] = 1U;
    }

    ret = DRV8703_SetDuty(dev, duty);
    if (ret != DRV8703_OK)
        AppControl_CaptureDrvFault(drv, ret);

    return ret;
}

static void AppControl_SleepDrv(uint8_t drv)
{
    DRV8703_Handle_t *dev;

    if (drv >= APP_CONTROL_DRV_COUNT || g_app_control_simulate_drv8703 != 0U)
        return;

    dev = DRV8703_BoardGet((DRV8703_BoardChannel_t)drv);
    if (dev != 0)
    {
        (void)DRV8703_SetDuty(dev, 0.0f);
        (void)DRV8703_Sleep(dev);
        g_app_control_drv_awake[drv] = 0U;
    }
}

static uint8_t AppControl_SharedDrvNeeded(void)
{
    return (s_cell[0].running != 0U) || (s_cell[1].running != 0U);
}

static void AppControl_StopCell(uint8_t cell)
{
    uint8_t first;

    if (cell >= APP_CONTROL_CELL_COUNT)
        return;

    first = AppControl_CellFirstDrv(cell);
    s_cell[cell].running = 0U;
    s_cell[cell].requested = 0U;
    s_cell[cell].pid_update_pending = 0U;
    s_cell[cell].duty = 0.0f;
    PID_Reset(&s_temp_pid[cell]);

    (void)AppControl_SetDrvDuty(first, 0.0f);
    (void)AppControl_SetDrvDuty((uint8_t)(first + 1U), 0.0f);
    AppControl_SleepDrv(first);
    AppControl_SleepDrv((uint8_t)(first + 1U));

    if (!AppControl_SharedDrvNeeded())
    {
        (void)AppControl_SetDrvDuty(4U, 0.0f);
        AppControl_SleepDrv(4U);
    }
}

static uint8_t AppControl_CellHardwareOk(uint8_t cell)
{
    if (cell >= APP_CONTROL_CELL_COUNT)
        return 0U;

    if (s_cell[cell].error != PANEL_ERR_NONE)
        return 0U;

    return 1U;
}

static void AppControl_StartCell(uint8_t cell)
{
    uint8_t first;
    uint8_t failed_drv;
    DRV8703_Status_t ret;

    if (cell >= APP_CONTROL_CELL_COUNT)
        return;

    if (!AppControl_CellHardwareOk(cell))
    {
        AppControl_StopCell(cell);
        return;
    }

    first = AppControl_CellFirstDrv(cell);
    failed_drv = first;
    ret = AppControl_PrepareDrv(first);
    if (ret == DRV8703_OK)
    {
        failed_drv = (uint8_t)(first + 1U);
        ret = AppControl_PrepareDrv((uint8_t)(first + 1U));
    }
    if (ret == DRV8703_OK)
    {
        failed_drv = 4U;
        ret = AppControl_PrepareDrv(4U);
    }

    if (ret != DRV8703_OK)
    {
        if (failed_drv == 4U)
        {
            AppControl_SetCellError(0U, PANEL_ERR_E315_SHARED_DRV);
            AppControl_SetCellError(1U, PANEL_ERR_E315_SHARED_DRV);
            AppControl_StopCell(0U);
            AppControl_StopCell(1U);
        }
        else
        {
            AppControl_SetCellError(cell, AppControl_CellDrvError(cell));
            AppControl_StopCell(cell);
        }
        return;
    }

    PID_Reset(&s_temp_pid[cell]);
    PID_SetTarget(&s_temp_pid[cell], s_cell[cell].target_temp);
    s_temp_last_pid_count[cell][0] = Sys_TempUpdateCount[cell * 2U];
    s_temp_last_pid_count[cell][1] = Sys_TempUpdateCount[cell * 2U + 1U];
    s_temp_last_pid_ms[cell] = osKernelGetTickCount();
    s_cell[cell].pid_update_pending = 0U;
    s_cell[cell].running = 1U;
    s_cell[cell].requested = 1U;
}

static void AppControl_ProcessCommands(void)
{
    AppControlCommand_t cmd;

    while ((s_cmd_queue != 0) &&
           (osMessageQueueGet(s_cmd_queue, &cmd, 0, 0U) == osOK))
    {
        if (cmd.cell >= APP_CONTROL_CELL_COUNT)
            continue;

        if (cmd.type == APP_CONTROL_CMD_START)
            AppControl_StartCell(cmd.cell);
        else if (cmd.type == APP_CONTROL_CMD_STOP)
            AppControl_StopCell(cmd.cell);
    }
}

static void AppControl_UpdateTemperatureInputs(uint32_t now_ms)
{
    float temp[4];
    uint8_t status[4];
    uint32_t count[4];
    uint32_t tick[4];
    uint8_t i;
    uint8_t cell;

    for (i = 0U; i < 4U; i++)
    {
        temp[i] = Sys_Temperatures[i];
        status[i] = Sys_Status[i];
        count[i] = Sys_TempUpdateCount[i];
        tick[i] = Sys_TempUpdateTick[i];
        g_app_control_temp_update_count[i] = count[i];
        g_app_control_temp_last_update_tick[i] = tick[i];
    }

    for (cell = 0U; cell < APP_CONTROL_CELL_COUNT; cell++)
    {
        uint8_t a = (uint8_t)(cell * 2U);
        uint8_t b = (uint8_t)(a + 1U);
        uint8_t a_fresh = AppControl_TempInputIsFresh(a, status, count, tick, now_ms);
        uint8_t b_fresh = AppControl_TempInputIsFresh(b, status, count, tick, now_ms);
        uint8_t both_new = (count[a] != s_temp_last_pid_count[cell][0]) &&
                           (count[b] != s_temp_last_pid_count[cell][1]);

        if ((a_fresh != 0U) && (b_fresh != 0U))
        {
            s_cell[cell].current_temp = (temp[a] + temp[b]) * 0.5f;
            if (both_new)
            {
                s_temp_last_pid_count[cell][0] = count[a];
                s_temp_last_pid_count[cell][1] = count[b];
                s_cell[cell].pid_update_pending = 1U;
                s_cell[cell].sensor_reset_count = 0U;
                g_app_control_temp_reset_count[cell] = 0U;
                if (AppControl_IsTempSensorError(s_cell[cell].error))
                {
                    g_app_control_temp_fault_sensor[cell] = 0xFFU;
                    AppControl_SetCellError(cell, PANEL_ERR_NONE);
                }
            }
        }

        if (s_cell[cell].running == 0U)
            continue;

        if ((a_fresh == 0U) || (b_fresh == 0U))
        {
            uint8_t bad_sensor = (a_fresh == 0U) ? a : b;

            if (s_cell[cell].sensor_reset_count < APP_CONTROL_TEMP_RESET_MAX_COUNT)
            {
                if (AppControl_RequestTempSensorReset(now_ms) != 0U)
                {
                    s_cell[cell].sensor_reset_count++;
                    g_app_control_temp_reset_count[cell] = s_cell[cell].sensor_reset_count;
                }
            }
            else
            {
                g_app_control_temp_fault_sensor[cell] = bad_sensor;
                AppControl_SetCellError(cell, AppControl_TempSensorError(bad_sensor));
                AppControl_StopCell(cell);
            }
        }
    }
}

static void AppControl_CheckVoltageFaults(void)
{
    float v[APP_CONTROL_DRV_COUNT];
    uint8_t i;

    if (g_app_control_simulate_voltage_ok != 0U)
        return;

    for (i = 0U; i < APP_CONTROL_DRV_COUNT; i++)
        v[i] = g_adc_measure_v_input_v[i];

    if ((v[4] > 1.0f) && (v[4] < APP_CONTROL_VSUPPLY_MIN_V))
    {
        AppControl_SetCellError(0U, PANEL_ERR_E305_SHARED_VOLTAGE);
        AppControl_SetCellError(1U, PANEL_ERR_E305_SHARED_VOLTAGE);
        AppControl_StopCell(0U);
        AppControl_StopCell(1U);
        return;
    }

    for (i = 0U; i < APP_CONTROL_CELL_COUNT; i++)
    {
        uint8_t first = AppControl_CellFirstDrv(i);
        if (((v[first] > 1.0f) && (v[first] < APP_CONTROL_VSUPPLY_MIN_V)) ||
            ((v[first + 1U] > 1.0f) && (v[first + 1U] < APP_CONTROL_VSUPPLY_MIN_V)))
        {
            AppControl_SetCellError(i, AppControl_CellVoltageError(i));
            AppControl_StopCell(i);
        }
    }
}

static void AppControl_CheckDrvFaults(uint32_t now_ms)
{
    uint8_t drv;

    if (g_app_control_simulate_drv8703 != 0U)
        return;
    if ((now_ms - s_last_fault_poll_ms) < APP_CONTROL_DRV_FAULT_POLL_MS)
        return;

    s_last_fault_poll_ms = now_ms;

    for (drv = 0U; drv < APP_CONTROL_DRV_COUNT; drv++)
    {
        DRV8703_Handle_t *dev;
        DRV8703_Status_t pin_ret;

        if (g_app_control_drv_ready[drv] == 0U)
            continue;
        if (g_app_control_drv_awake[drv] == 0U)
            continue;

        dev = DRV8703_BoardGet((DRV8703_BoardChannel_t)drv);
        if (dev == 0)
            continue;

        pin_ret = DRV8703_CheckFaultPins(dev);
        if (pin_ret != DRV8703_OK)
        {
            if (AppControl_CaptureDrvPinFaultMoment(drv, pin_ret) == 0U)
                continue;

            if (drv == 4U)
            {
                AppControl_SetCellError(0U, PANEL_ERR_E315_SHARED_DRV);
                AppControl_SetCellError(1U, PANEL_ERR_E315_SHARED_DRV);
                AppControl_StopCell(0U);
                AppControl_StopCell(1U);
            }
            else if (drv <= 1U)
            {
                AppControl_SetCellError(0U, PANEL_ERR_E311_CELL1_DRV);
                AppControl_StopCell(0U);
            }
            else
            {
                AppControl_SetCellError(1U, PANEL_ERR_E312_CELL2_DRV);
                AppControl_StopCell(1U);
            }
        }
    }
}

static void AppControl_RunClosedLoop(void)
{
    uint8_t cell;
    uint8_t any_running = 0U;

    for (cell = 0U; cell < APP_CONTROL_CELL_COUNT; cell++)
    {
        uint8_t first = AppControl_CellFirstDrv(cell);
        float duty;

        if (s_cell[cell].running == 0U || s_cell[cell].error != PANEL_ERR_NONE)
        {
            s_cell[cell].duty = 0.0f;
            continue;
        }

        any_running = 1U;
        if (s_cell[cell].pid_update_pending == 0U)
            continue;

        s_cell[cell].pid_update_pending = 0U;
        if (s_temp_last_pid_ms[cell] != 0U)
        {
            float dt = (float)(osKernelGetTickCount() - s_temp_last_pid_ms[cell]) * 0.001f;
            s_temp_pid[cell].dt = AppControl_Clamp(dt,
                                                   APP_CONTROL_TEMP_PID_DT_MIN_S,
                                                   APP_CONTROL_TEMP_PID_DT_MAX_S);
        }
        else
        {
            s_temp_pid[cell].dt = APP_CONTROL_TEMP_PID_DT_DEFAULT_S;
        }
        s_temp_last_pid_ms[cell] = osKernelGetTickCount();

        PID_SetTarget(&s_temp_pid[cell], s_cell[cell].target_temp);
        duty = PID_Compute(&s_temp_pid[cell], s_cell[cell].current_temp);
        duty = AppControl_Clamp(duty, -APP_CONTROL_MAX_ABS_DUTY, APP_CONTROL_MAX_ABS_DUTY);

        s_cell[cell].duty = duty;

        if (AppControl_Abs(duty) < 0.001f)
            duty = 0.0f;

        if ((AppControl_SetDrvDuty(first, duty) != DRV8703_OK) ||
            (AppControl_SetDrvDuty((uint8_t)(first + 1U), duty) != DRV8703_OK))
        {
            AppControl_SetCellError(cell, AppControl_CellDrvError(cell));
            AppControl_StopCell(cell);
        }
    }

    if (any_running != 0U)
    {
        if (AppControl_SetDrvDuty(4U, APP_CONTROL_SHARED_CH5_DUTY) != DRV8703_OK)
        {
            AppControl_SetCellError(0U, PANEL_ERR_E315_SHARED_DRV);
            AppControl_SetCellError(1U, PANEL_ERR_E315_SHARED_DRV);
            AppControl_StopCell(0U);
            AppControl_StopCell(1U);
        }
    }
}

AppControl_Status_t AppControl_Init(void)
{
    uint8_t i;

    s_cmd_queue = osMessageQueueNew(APP_CONTROL_QUEUE_DEPTH,
                                    sizeof(AppControlCommand_t),
                                    0);
    if (s_cmd_queue == 0)
    {
        g_app_control_init_result = APP_CONTROL_ERROR_QUEUE;
        return APP_CONTROL_ERROR_QUEUE;
    }

    for (i = 0U; i < APP_CONTROL_CELL_COUNT; i++)
    {
        s_cell[i].running = 0U;
        s_cell[i].requested = 0U;
        s_cell[i].pid_update_pending = 0U;
        s_cell[i].sensor_reset_count = 0U;
        s_cell[i].target_temp = 25.0f;
        s_cell[i].current_temp = 25.0f;
        s_cell[i].duty = 0.0f;
        s_cell[i].error = PANEL_ERR_NONE;
        s_cell[i].error_set_ms = 0U;
        PID_Init(&s_temp_pid[i],
                 APP_CONTROL_TEMP_PID_KP,
                 APP_CONTROL_TEMP_PID_KI,
                 APP_CONTROL_TEMP_PID_KD,
                 APP_CONTROL_TEMP_PID_DT_S);
        PID_SetLimits(&s_temp_pid[i],
                      -APP_CONTROL_MAX_ABS_DUTY,
                      APP_CONTROL_MAX_ABS_DUTY,
                      -APP_CONTROL_MAX_ABS_DUTY,
                      APP_CONTROL_MAX_ABS_DUTY);
        s_temp_last_pid_count[i][0] = 0U;
        s_temp_last_pid_count[i][1] = 0U;
        s_temp_last_pid_ms[i] = 0U;
    }

    for (i = 0U; i < APP_CONTROL_DRV_COUNT; i++)
    {
        uint8_t reg;

        g_app_control_drv_fault_snapshot_valid[i] = 0U;
        g_app_control_drv_fault_capture_count[i] = 0U;
        g_app_control_drv_fault_read_status[i] = DRV8703_OK;
        g_app_control_drv_dump_status[i] = DRV8703_OK;
        g_app_control_drv_awake[i] = 0U;
        g_app_control_drv_fault_status[i] = 0xFFU;
        g_app_control_drv_vds_gdf_status[i] = 0xFFU;
        for (reg = 0U; reg < DRV8703_REGISTER_COUNT; reg++)
            g_app_control_drv_reg_dump[i][reg] = 0xFFU;

        g_app_control_drv_startup_dump_valid[i] = 0U;
        g_app_control_drv_startup_dump_status[i] = DRV8703_OK;
        for (reg = 0U; reg < DRV8703_REGISTER_COUNT; reg++)
            g_app_control_drv_startup_reg_dump[i][reg] = 0xFFU;

        g_app_control_drv_pin_fault_count[i] = 0U;
        g_app_control_drv_pin_fault_status[i] = DRV8703_OK;
        g_app_control_drv_pin_fault_read_status[i] = DRV8703_OK;
        g_app_control_drv_pin_fault_dump_status[i] = DRV8703_OK;
        g_app_control_drv_pin_fault_fault_status[i] = 0xFFU;
        g_app_control_drv_pin_fault_vds_gdf_status[i] = 0xFFU;
        g_app_control_drv_pin_fault_stop_bits[i] = 0U;
        for (reg = 0U; reg < DRV8703_REGISTER_COUNT; reg++)
            g_app_control_drv_pin_fault_reg_dump[i][reg] = 0xFFU;
    }

    AppControl_ApplyDebugState();
    g_app_control_init_result = APP_CONTROL_OK;
    return APP_CONTROL_OK;
}

void AppControl_Task(uint32_t now_ms)
{
    AppControl_Lock();
    AppControl_ServiceTempSensorReset(now_ms);
    AppControl_ProcessCommands();
    AppControl_UpdateTemperatureInputs(now_ms);
    AppControl_CheckVoltageFaults();
    AppControl_CheckDrvFaults(now_ms);
    AppControl_ServiceErrorDisplayTimeout(now_ms);
    AppControl_RunClosedLoop();
    AppControl_ApplyDebugState();
    g_app_control_loop_count++;
    AppControl_Unlock();
}

void AppControl_UpdatePanel(TempPanel_t *panel, uint32_t now_ms)
{
    uint8_t cell;
    float temp[APP_CONTROL_CELL_COUNT];
    PanelError_t err[APP_CONTROL_CELL_COUNT];

    if (panel == 0)
        return;

    AppControl_Lock();
    for (cell = 0U; cell < APP_CONTROL_CELL_COUNT; cell++)
    {
        temp[cell] = s_cell[cell].current_temp;
        err[cell] = s_cell[cell].error;
    }
    AppControl_Unlock();

    for (cell = 0U; cell < APP_CONTROL_CELL_COUNT; cell++)
    {
        TempPanel_UpdateMeasuredTemp(panel, cell, temp[cell], now_ms);
        TempPanel_SetCellError(panel, cell, err[cell]);
    }
}

static void AppControl_PostCommand(AppControlCommandType_t type, uint8_t cell)
{
    AppControlCommand_t cmd;

    if (cell >= APP_CONTROL_CELL_COUNT)
        return;
    if (s_cmd_queue == 0)
    {
        g_app_control_cmd_drop_count++;
        return;
    }

    cmd.type = type;
    cmd.cell = cell;
    if (osMessageQueuePut(s_cmd_queue, &cmd, 0U, 0U) != osOK)
        g_app_control_cmd_drop_count++;
}

void Control_StartPid(uint8_t cell)
{
    AppControl_PostCommand(APP_CONTROL_CMD_START, cell);
}

void Control_StopPid(uint8_t cell)
{
    AppControl_PostCommand(APP_CONTROL_CMD_STOP, cell);
}

void Control_SetTargetTemp(uint8_t cell, float target)
{
    if (cell >= APP_CONTROL_CELL_COUNT)
        return;

    AppControl_Lock();
    s_cell[cell].target_temp = target;
    AppControl_Unlock();
}
