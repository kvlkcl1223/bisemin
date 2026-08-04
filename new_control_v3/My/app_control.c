#include "app_control.h"

#include "adc_measure.h"
#include "calib_mode.h"
#include "cmsis_os.h"
#include "drv8703_board.h"
#include "pid_controller.h"
#include "ads1220.h"
#include "pc_protocol.h"

#include <string.h>

#define APP_CONTROL_QUEUE_DEPTH 12U
#define APP_CONTROL_DRV_RETRY_COUNT 3U
#define APP_CONTROL_DRV_VREF_MV 3300U
#define APP_CONTROL_VSUPPLY_MIN_V 20.0f
#define APP_CONTROL_TEMP_PID_DT_S 0.2f
#define APP_CONTROL_TEMP_PID_KP 0.1f
#define APP_CONTROL_TEMP_PID_KI 0.01f
#define APP_CONTROL_TEMP_PID_KD 0.0f
#define APP_CONTROL_DRV_FAULT_POLL_MS 500U
#define APP_CONTROL_DRV_REG_SNAPSHOT_MS 100U
#define APP_CONTROL_DRV_FAULT_READ_RETRY_COUNT 8U
#define APP_CONTROL_DRV_FAULT_STABLE_READ_COUNT 2U
#define APP_CONTROL_TEMP_STALE_CYCLES 30U
#define APP_CONTROL_TEMP_RESET_COOLDOWN_MS 1500U
#define APP_CONTROL_TEMP_RESET_PULSE_MS 20U
#define APP_CONTROL_TEMP_RESET_MAX_COUNT 3U
#define APP_CONTROL_TEMP_PID_DT_DEFAULT_S 0.05f
#define APP_CONTROL_TEMP_PID_DT_MIN_S 0.02f
#define APP_CONTROL_TEMP_PID_DT_MAX_S 1.0f
#define APP_CONTROL_DUTY_SLEW_RATE_PER_S 1.0f
#define APP_CONTROL_ERROR_DISPLAY_MS 5000U
#define APP_CONTROL_NORMAL_PID_DELAY_ENABLE 1U
#define APP_CONTROL_NORMAL_PID_ENTER_BAND_C 2.0f
#define APP_CONTROL_NORMAL_PID_ENTER_TIMEOUT_MS 10000U

/*
 * DRV8703 FAULT bit7 is only a summary flag. Stop the cell only for faults
 * that can damage hardware or make output unsafe. Watchdog timeout and over
 * temperature warning are recorded for debug, but they do not latch a panel
 * error or stop the temperature cell by themselves.
 */
#define APP_CONTROL_DRV_STOP_FAULT_MASK \
    (DRV8703_FAULT_GDF |                \
     DRV8703_FAULT_OCP |                \
     DRV8703_FAULT_OTSD)

typedef struct
{
    uint8_t running;
    uint8_t requested;
    uint8_t stopping;
    uint8_t pid_update_pending;
    uint8_t sensor_reset_count;
    float target_temp;
    float current_temp;
    float duty;
    PanelError_t error;
    uint32_t error_set_ms;
} AppControlCell_t;

typedef struct
{
    uint8_t temp_outer;
    uint8_t temp_inner;
    uint8_t drv_outer;
    uint8_t drv_inner;
    float inner_ratio;
    uint8_t shared_drv;
    float shared_duty;
} AppControlCellConfig_t;

static const AppControlCellConfig_t s_cell_config[APP_CONTROL_CELL_COUNT] =
{
    {
        APP_CONTROL_CELL0_TEMP_OUTER,
        APP_CONTROL_CELL0_TEMP_INNER,
        APP_CONTROL_CELL0_DRV_OUTER,
        APP_CONTROL_CELL0_DRV_INNER,
        APP_CONTROL_CELL0_INNER_DUTY_RATIO,
        APP_CONTROL_SHARED_DRV,
        APP_CONTROL_SHARED_CH5_DUTY
    },
    {
        APP_CONTROL_CELL1_TEMP_OUTER,
        APP_CONTROL_CELL1_TEMP_INNER,
        APP_CONTROL_CELL1_DRV_OUTER,
        APP_CONTROL_CELL1_DRV_INNER,
        APP_CONTROL_CELL1_INNER_DUTY_RATIO,
        APP_CONTROL_SHARED_DRV,
        APP_CONTROL_SHARED_CH5_DUTY
    }
};

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
volatile float g_app_control_cell_outer_duty[APP_CONTROL_CELL_COUNT] = {0.0f, 0.0f};
volatile float g_app_control_cell_inner_duty[APP_CONTROL_CELL_COUNT] = {0.0f, 0.0f};
volatile float g_app_control_cell_inner_ratio[APP_CONTROL_CELL_COUNT] =
{
    APP_CONTROL_CELL0_INNER_DUTY_RATIO,
    APP_CONTROL_CELL1_INNER_DUTY_RATIO
};
volatile float g_app_control_pid_temp[APP_CONTROL_CLOSED_LOOP_COUNT] = {25.0f, 25.0f, 25.0f, 25.0f};
volatile float g_app_control_pid_duty[APP_CONTROL_CLOSED_LOOP_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile uint8_t g_app_control_pid_update_pending[APP_CONTROL_CLOSED_LOOP_COUNT] = {0};
volatile uint8_t g_app_control_drv_init_attempts[APP_CONTROL_DRV_COUNT] = {0};
volatile uint8_t g_app_control_drv_ready[APP_CONTROL_DRV_COUNT] = {0};
volatile uint8_t g_app_control_drv_awake[APP_CONTROL_DRV_COUNT] = {0};
volatile uint8_t g_app_control_drv_fault[APP_CONTROL_DRV_COUNT] = {0};
volatile uint8_t g_app_control_drv_startup_dump_valid[APP_CONTROL_DRV_COUNT] = {0};
volatile DRV8703_Status_t g_app_control_drv_startup_dump_status[APP_CONTROL_DRV_COUNT] =
    {
        DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK};
volatile uint8_t g_app_control_drv_startup_reg_dump[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT] =
    {
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU}};
volatile DRV8703_Status_t g_app_control_drv_startup_reg_status[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT] =
    {
        {DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK},
        {DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK},
        {DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK},
        {DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK},
        {DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK}};
volatile uint16_t g_app_control_drv_startup_tx[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT] =
    {
        {0U, 0U, 0U, 0U, 0U, 0U},
        {0U, 0U, 0U, 0U, 0U, 0U},
        {0U, 0U, 0U, 0U, 0U, 0U},
        {0U, 0U, 0U, 0U, 0U, 0U},
        {0U, 0U, 0U, 0U, 0U, 0U}};
volatile uint16_t g_app_control_drv_startup_rx[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT] =
    {
        {0U, 0U, 0U, 0U, 0U, 0U},
        {0U, 0U, 0U, 0U, 0U, 0U},
        {0U, 0U, 0U, 0U, 0U, 0U},
        {0U, 0U, 0U, 0U, 0U, 0U},
        {0U, 0U, 0U, 0U, 0U, 0U}};
volatile uint8_t g_app_control_drv_startup_expected[DRV8703_REGISTER_COUNT] =
    {
        0x00U, 0x00U, 0x18U, 0x07U, 0x70U, 0x01U};
volatile uint8_t g_app_control_last_drv_fault = 0xFFU;
volatile DRV8703_Status_t g_app_control_last_drv_status = DRV8703_OK;
volatile uint8_t g_app_control_drv_fault_snapshot_valid[APP_CONTROL_DRV_COUNT] = {0};
volatile uint32_t g_app_control_drv_fault_capture_count[APP_CONTROL_DRV_COUNT] = {0};
volatile DRV8703_Status_t g_app_control_drv_fault_read_status[APP_CONTROL_DRV_COUNT] =
    {
        DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK};
volatile DRV8703_Status_t g_app_control_drv_dump_status[APP_CONTROL_DRV_COUNT] =
    {
        DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK};
volatile uint8_t g_app_control_drv_fault_status[APP_CONTROL_DRV_COUNT] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
volatile uint8_t g_app_control_drv_vds_gdf_status[APP_CONTROL_DRV_COUNT] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
volatile uint8_t g_app_control_drv_reg_dump[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT] =
    {
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU}};
volatile DRV8703_Status_t g_app_control_drv_reg_read_status[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT] =
    {
        {DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK},
        {DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK},
        {DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK},
        {DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK},
        {DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK}};
volatile uint8_t g_app_control_drv_reg_read_ok_mask[APP_CONTROL_DRV_COUNT] = {0};
volatile uint8_t g_app_control_drv_pin_fault_last = 0xFFU;
volatile uint32_t g_app_control_drv_pin_fault_count[APP_CONTROL_DRV_COUNT] = {0};
volatile DRV8703_Status_t g_app_control_drv_pin_fault_status[APP_CONTROL_DRV_COUNT] =
    {
        DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK};
volatile DRV8703_Status_t g_app_control_drv_pin_fault_read_status[APP_CONTROL_DRV_COUNT] =
    {
        DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK};
volatile DRV8703_Status_t g_app_control_drv_pin_fault_dump_status[APP_CONTROL_DRV_COUNT] =
    {
        DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK};
volatile uint8_t g_app_control_drv_pin_fault_fault_status[APP_CONTROL_DRV_COUNT] =
    {
        0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
volatile uint8_t g_app_control_drv_pin_fault_vds_gdf_status[APP_CONTROL_DRV_COUNT] =
    {
        0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
volatile uint8_t g_app_control_drv_pin_fault_stop_bits[APP_CONTROL_DRV_COUNT] = {0};
volatile uint8_t g_app_control_drv_pin_fault_reg_dump[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT] =
    {
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU}};
volatile DRV8703_Status_t g_app_control_drv_pin_fault_reg_read_status[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT] =
    {
        {DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK},
        {DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK},
        {DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK},
        {DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK},
        {DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK}};
volatile uint8_t g_app_control_drv_pin_fault_reg_read_ok_mask[APP_CONTROL_DRV_COUNT] = {0};
volatile uint8_t g_app_control_drv_pin_fault_all_ff[APP_CONTROL_DRV_COUNT] = {0};
volatile uint32_t g_app_control_drv_pin_fault_all_ff_count[APP_CONTROL_DRV_COUNT] = {0};
volatile uint8_t g_app_control_drv_pin_fault_stable_count[APP_CONTROL_DRV_COUNT] = {0};
volatile uint8_t g_app_control_sleep_reg_snapshot[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT + 1U] =
    {
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x00U},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x00U},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x00U},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x00U},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x00U}};
volatile uint8_t g_app_control_periodic_reg_snapshot[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT + 1U] =
    {
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x00U},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x00U},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x00U},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x00U},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x00U}};
volatile uint32_t g_app_control_periodic_reg_snapshot_count = 0U;
volatile uint8_t g_app_control_temp_reset_count[APP_CONTROL_CELL_COUNT] = {0};
volatile uint32_t g_app_control_temp_last_update_tick[4] = {0};
volatile uint32_t g_app_control_temp_update_count[4] = {0};
volatile uint8_t g_app_control_temp_fault_sensor[APP_CONTROL_CELL_COUNT] = {0xFFU, 0xFFU};
volatile uint8_t g_app_control_temp_reset_active = 0U;

volatile float g_app_control_temp_freq_hz[APP_CONTROL_TEMP_INPUT_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float g_app_control_pid_freq_hz[APP_CONTROL_CLOSED_LOOP_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

volatile uint8_t g_app_control_test_active = 0U;
volatile uint8_t g_app_control_test_phase = 0U;
volatile uint8_t g_app_control_test_drv_ok[APP_CONTROL_DRV_COUNT] = {0, 0, 0, 0, 0};
volatile float g_app_control_test_duty[APP_CONTROL_DRV_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

volatile uint32_t g_app_control_drv_raw_poll_count = 0U;
volatile uint8_t g_app_control_drv_raw_dump[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT] =
    {
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU},
        {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU}};
volatile uint8_t g_app_control_drv_raw_mask[APP_CONTROL_DRV_COUNT] = {0U, 0U, 0U, 0U, 0U};
volatile DRV8703_Status_t g_app_control_drv_raw_status[APP_CONTROL_DRV_COUNT] =
    {
        DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK};

static osMessageQueueId_t s_cmd_queue;
static AppControlCell_t s_cell[APP_CONTROL_CELL_COUNT];
static PID_TypeDef s_cell_pid[APP_CONTROL_CELL_COUNT];
static uint32_t s_last_fault_poll_ms = 0U;
static uint32_t s_last_reg_snapshot_ms = 0U;
static uint8_t s_temp_pid_update_pending[APP_CONTROL_CLOSED_LOOP_COUNT] = {0};
static uint8_t s_cell_pid_update_pending[APP_CONTROL_CELL_COUNT] = {0U, 0U};
static float s_temp_channel_temp[APP_CONTROL_CLOSED_LOOP_COUNT] = {25.0f, 25.0f, 25.0f, 25.0f};
static float s_temp_channel_duty[APP_CONTROL_CLOSED_LOOP_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
static float s_cell_base_output_duty[APP_CONTROL_CELL_COUNT] = {0.0f, 0.0f};
static float s_drv_output_duty[APP_CONTROL_DRV_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static uint8_t s_cell_pid_feedback_enabled[APP_CONTROL_CELL_COUNT] = {0U, 0U};
static uint32_t s_cell_pid_feedback_start_ms[APP_CONTROL_CELL_COUNT] = {0U, 0U};
static uint32_t s_temp_last_pid_count[APP_CONTROL_CLOSED_LOOP_COUNT] = {0U, 0U, 0U, 0U};
static uint32_t s_cell_last_pid_ms[APP_CONTROL_CELL_COUNT] = {0U, 0U};
static uint32_t s_temp_freq_last_count[APP_CONTROL_TEMP_INPUT_COUNT] = {0U, 0U, 0U, 0U};
static uint32_t s_temp_freq_last_tick[APP_CONTROL_TEMP_INPUT_COUNT] = {0U, 0U, 0U, 0U};
static uint32_t s_pid_freq_last_tick[APP_CONTROL_CLOSED_LOOP_COUNT] = {0U, 0U, 0U, 0U};

#if TEMP_VIRTUAL_LOW_ENABLE
typedef struct
{
    uint8_t active;
    uint8_t started;
    float last_real_temp;
    float learned_rate_cps;
    float virtual_temp;
    float noise;
    uint32_t last_ms;
    uint32_t rng;
} AppControlVirtualLowTemp_t;

#define APP_CONTROL_VIRTUAL_ENTER_TEMP_C (PANEL_TEMP_REAL_MIN + 0.5f)
#define APP_CONTROL_VIRTUAL_DEFAULT_RATE_CPS 0.03f
#define APP_CONTROL_VIRTUAL_RATE_MIN_CPS 0.005f
#define APP_CONTROL_VIRTUAL_RATE_MAX_CPS 0.20f
#define APP_CONTROL_VIRTUAL_NOISE_MAX_C 0.12f

static AppControlVirtualLowTemp_t s_virtual_low[APP_CONTROL_CELL_COUNT] =
{
    {0U, 0U, 25.0f, APP_CONTROL_VIRTUAL_DEFAULT_RATE_CPS, 25.0f, 0.0f, 0U, 0x13572468UL},
    {0U, 0U, 25.0f, APP_CONTROL_VIRTUAL_DEFAULT_RATE_CPS, 25.0f, 0.0f, 0U, 0x24681357UL}
};

static float s_cell_display_temp[APP_CONTROL_CELL_COUNT] = {25.0f, 25.0f};
static float s_temp_channel_display_temp[APP_CONTROL_CLOSED_LOOP_COUNT] = {25.0f, 25.0f, 25.0f, 25.0f};
#endif
static uint32_t s_temp_reset_release_ms = 0U;
static uint32_t s_temp_last_reset_ms = 0U;
static uint32_t s_last_raw_poll_ms = 0U;

static uint32_t s_temp_last_count_for_stale[APP_CONTROL_TEMP_INPUT_COUNT] = {0U, 0U, 0U, 0U};
uint8_t s_temp_stale_cycles[APP_CONTROL_TEMP_INPUT_COUNT] = {0U, 0U, 0U, 0U};

static uint8_t s_test_drv_initialized = 0U;
static uint32_t s_test_duty_toggle_ms[APP_CONTROL_DRV_COUNT] = {0U};
static int8_t s_test_ramp_dir[APP_CONTROL_DRV_COUNT] = {1, 1, 1, 1, 1};
static uint8_t s_calib_was_active = 0U;     /* 检测标定→正常模式的过�?*/
static uint32_t s_last_water_check_ms = 0U; /* 水位检测间隔计�?*/

/* 标定数据缓存，用于前馈控�?------------------------------------------------*/
typedef struct
{
    uint8_t loaded;
    float duty[CALIB_DUTY_COUNT];
    float temp_ch0[CALIB_DUTY_COUNT];
    float temp_ch1[CALIB_DUTY_COUNT];
    float temp_avg[CALIB_DUTY_COUNT];
} CalibCache_t;

static CalibCache_t s_calib_cache[APP_CONTROL_CELL_COUNT];

extern osMutexId_t SysStateMutexHandle;
extern volatile float Sys_Temperatures[4];
extern volatile uint8_t Sys_Status[4];
extern volatile uint32_t Sys_TempUpdateCount[4];
extern volatile uint32_t Sys_TempUpdateTick[4];

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
#define RX_BUFFER_SIZE 256U
extern uint8_t rx_buffer[RX_BUFFER_SIZE];

/** @brief Acquire the global system state mutex. */
static void AppControl_Lock(void)
{
    (void)osMutexAcquire(SysStateMutexHandle, osWaitForever);
}

/** @brief Release the global system state mutex. */
static void AppControl_Unlock(void)
{
    (void)osMutexRelease(SysStateMutexHandle);
}

/** @brief Absolute value of a float. */
static float AppControl_Abs(float v)
{
    return (v < 0.0f) ? -v : v;
}

/** @brief Clamp a float value between min_v and max_v. */
static float AppControl_Clamp(float v, float min_v, float max_v)
{
    if (v < min_v)
        return min_v;
    if (v > max_v)
        return max_v;
    return v;
}

static float AppControl_ApplyDutySlew(uint8_t drv, float target_duty, float dt_s)
{
    float prev;
    float max_delta;

    if (drv >= APP_CONTROL_DRV_COUNT)
        return target_duty;

    if (dt_s <= 0.0f)
        dt_s = APP_CONTROL_TEMP_PID_DT_DEFAULT_S;
    dt_s = AppControl_Clamp(dt_s,
                            APP_CONTROL_TEMP_PID_DT_MIN_S,
                            APP_CONTROL_TEMP_PID_DT_MAX_S);

    prev = s_drv_output_duty[drv];
    max_delta = APP_CONTROL_DUTY_SLEW_RATE_PER_S * dt_s;

    if (target_duty > (prev + max_delta))
        target_duty = prev + max_delta;
    else if (target_duty < (prev - max_delta))
        target_duty = prev - max_delta;

    s_drv_output_duty[drv] = target_duty;
    return target_duty;
}

static float AppControl_ApplyCellDutySlew(uint8_t cell, float target_duty, float dt_s)
{
    float prev;
    float max_delta;

    if (cell >= APP_CONTROL_CELL_COUNT)
        return target_duty;

    if (dt_s <= 0.0f)
        dt_s = APP_CONTROL_TEMP_PID_DT_DEFAULT_S;
    dt_s = AppControl_Clamp(dt_s,
                            APP_CONTROL_TEMP_PID_DT_MIN_S,
                            APP_CONTROL_TEMP_PID_DT_MAX_S);

    prev = s_cell_base_output_duty[cell];
    max_delta = APP_CONTROL_DUTY_SLEW_RATE_PER_S * dt_s;

    if (target_duty > (prev + max_delta))
        target_duty = prev + max_delta;
    else if (target_duty < (prev - max_delta))
        target_duty = prev - max_delta;

    s_cell_base_output_duty[cell] = target_duty;
    return target_duty;
}

static const AppControlCellConfig_t *AppControl_CellConfig(uint8_t cell)
{
    if (cell >= APP_CONTROL_CELL_COUNT)
        return 0;
    return &s_cell_config[cell];
}

static float AppControl_CellInnerRatio(uint8_t cell)
{
    const AppControlCellConfig_t *cfg = AppControl_CellConfig(cell);

    if (cfg == 0)
        return 0.0f;
    return cfg->inner_ratio;
}

static void AppControl_RecordDrvDuty(uint8_t drv, float duty)
{
    if (drv >= APP_CONTROL_DRV_COUNT)
        return;

    s_drv_output_duty[drv] = duty;
    if (drv < APP_CONTROL_CLOSED_LOOP_COUNT)
        s_temp_channel_duty[drv] = duty;
}

static void AppControl_RecordCellDuties(uint8_t cell, float outer_duty)
{
    const AppControlCellConfig_t *cfg = AppControl_CellConfig(cell);
    float inner_duty;

    if (cfg == 0)
        return;

    outer_duty = AppControl_Clamp(outer_duty,
                                  -APP_CONTROL_MAX_ABS_DUTY,
                                  APP_CONTROL_MAX_ABS_DUTY);
    inner_duty = AppControl_Clamp(outer_duty * AppControl_CellInnerRatio(cell),
                                  -APP_CONTROL_MAX_ABS_DUTY,
                                  APP_CONTROL_MAX_ABS_DUTY);

    s_cell[cell].duty = outer_duty;
    g_app_control_cell_outer_duty[cell] = outer_duty;
    g_app_control_cell_inner_duty[cell] = inner_duty;
    g_app_control_cell_inner_ratio[cell] = AppControl_CellInnerRatio(cell);

    AppControl_RecordDrvDuty(cfg->drv_outer, outer_duty);
    AppControl_RecordDrvDuty(cfg->drv_inner, inner_duty);
}

static uint8_t AppControl_CellOwnsDrv(uint8_t cell, uint8_t drv)
{
    const AppControlCellConfig_t *cfg = AppControl_CellConfig(cell);

    if (cfg == 0)
        return 0U;
    return (cfg->drv_outer == drv || cfg->drv_inner == drv) ? 1U : 0U;
}

static uint8_t AppControl_CellForDrv(uint8_t drv)
{
    uint8_t cell;

    for (cell = 0U; cell < APP_CONTROL_CELL_COUNT; cell++)
    {
        if (AppControl_CellOwnsDrv(cell, drv) != 0U)
            return cell;
    }

    return APP_CONTROL_INVALID_INDEX;
}

static uint8_t AppControl_DrvIsShared(uint8_t drv)
{
    uint8_t cell;

    for (cell = 0U; cell < APP_CONTROL_CELL_COUNT; cell++)
    {
        const AppControlCellConfig_t *cfg = AppControl_CellConfig(cell);
        if (cfg != 0 && cfg->shared_drv == drv)
            return 1U;
    }

    return 0U;
}

static float AppControl_SharedTargetDuty(void)
{
    uint8_t cell;

    for (cell = 0U; cell < APP_CONTROL_CELL_COUNT; cell++)
    {
        const AppControlCellConfig_t *cfg = AppControl_CellConfig(cell);
        if (cfg != 0 && cfg->shared_drv != APP_CONTROL_INVALID_INDEX &&
            s_cell[cell].running != 0U && s_cell[cell].stopping == 0U)
        {
            return cfg->shared_duty;
        }
    }

    return 0.0f;
}

/**
 * @brief Release the temperature sensor NRST pin if the reset pulse has elapsed.
 *
 * After releasing NRST this function aborts any in-progress UART reception
 * and restarts the DMA idle-line reception so the UART is ready for the
 * sensor data that follows.
 *
 * @param now_ms Current RTOS tick.
 */
static void AppControl_ServiceTempSensorReset(uint32_t now_ms)
{
    if ((g_app_control_temp_reset_active != 0U) &&
        ((now_ms - s_temp_reset_release_ms) < 0x80000000UL))
    {
        // HAL_GPIO_WritePin(NRST_OTHER_GPIO_Port, NRST_OTHER_Pin, GPIO_PIN_SET);
        g_app_control_temp_reset_active = 0U;
        HAL_UART_AbortReceive(&huart1);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUFFER_SIZE);
    }
}

/**
 * @brief Start a temperature sensor hardware reset sequence.
 *
 * Pulls the NRST pin low to reset the sensor.  Does NOT touch the UART DMA �?
 * the idle-line ISR handles DMA restart safely on its own.
 *
 * @param now_ms Current RTOS tick.
 * @return 1 if a reset was started, 0 if one is already in progress or the
 *         cooldown period has not elapsed.
 */
static uint8_t AppControl_RequestTempSensorReset(uint32_t now_ms)
{
    if (g_app_control_temp_reset_active != 0U)
        return 0U;
    if ((now_ms - s_temp_last_reset_ms) < APP_CONTROL_TEMP_RESET_COOLDOWN_MS)
        return 0U;

    // HAL_GPIO_WritePin(NRST_OTHER_GPIO_Port, NRST_OTHER_Pin, GPIO_PIN_RESET);

    s_temp_reset_release_ms = now_ms + APP_CONTROL_TEMP_RESET_PULSE_MS;
    s_temp_last_reset_ms = now_ms;
    g_app_control_temp_reset_active = 1U;
    return 1U;
}


/** @brief Panel error code for a DRV8703 fault on a given cell. */
static PanelError_t AppControl_CellDrvError(uint8_t cell)
{
    return (cell == 0U) ? PANEL_ERR_E311_CELL1_DRV : PANEL_ERR_E312_CELL2_DRV;
}

/** @brief Panel error code for an input-voltage fault on a given cell. */
static PanelError_t AppControl_CellVoltageError(uint8_t cell)
{
    return (cell == 0U) ? PANEL_ERR_E301_CELL1_VOLTAGE : PANEL_ERR_E302_CELL2_VOLTAGE;
}

/**
 * @brief Read all DRV8703 registers after initialisation and store them as
 *        the expected "golden" values for that chip.
 */
static void AppControl_CaptureDrvStartupRegs(uint8_t drv, DRV8703_Handle_t *dev)
{
    uint8_t i;
    uint8_t mask = 0U;

    if ((drv >= APP_CONTROL_DRV_COUNT) || (dev == 0))
        return;

    for (i = 0U; i < DRV8703_REGISTER_COUNT; i++)
    {
        g_app_control_drv_startup_reg_dump[drv][i] = 0xFFU;
        g_app_control_drv_startup_reg_status[drv][i] = DRV8703_ERROR_SPI;
        g_app_control_drv_startup_tx[drv][i] = 0U;
        g_app_control_drv_startup_rx[drv][i] = 0U;
    }
    g_app_control_drv_startup_dump_valid[drv] = 0U;

    for (i = 0U; i < DRV8703_REGISTER_COUNT; i++)
    {
        uint8_t value = 0xFFU;
        DRV8703_Status_t ret = DRV8703_ReadReg(dev, i, &value);

        g_app_control_drv_startup_reg_status[drv][i] = ret;
        g_app_control_drv_startup_tx[drv][i] = dev->last_tx;
        g_app_control_drv_startup_rx[drv][i] = dev->last_rx;
        g_app_control_drv_startup_reg_dump[drv][i] = value;
        if (ret == DRV8703_OK)
            mask |= (uint8_t)(1U << i);
    }

    g_app_control_drv_startup_dump_status[drv] = (mask == 0x3FU) ? DRV8703_OK : DRV8703_ERROR_SPI;
    g_app_control_drv_startup_dump_valid[drv] = (mask == 0x3FU) ? 1U : 0U;
}

/**
 * @brief Read all 6 DRV8703 registers into the global debug arrays.
 * @return Bitmask of successfully-read registers.
 */
static uint8_t AppControl_ReadDrvRegsToDebug(uint8_t drv,
                                             DRV8703_Handle_t *dev,
                                             volatile uint8_t dump[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT],
                                             volatile DRV8703_Status_t status[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT],
                                             volatile uint8_t ok_mask[APP_CONTROL_DRV_COUNT])
{
    uint8_t reg;
    uint8_t mask = 0U;

    if ((drv >= APP_CONTROL_DRV_COUNT) || (dev == 0))
        return 0U;

    for (reg = 0U; reg < DRV8703_REGISTER_COUNT; reg++)
    {
        uint8_t value = 0xFFU;
        DRV8703_Status_t ret = DRV8703_ReadReg(dev, reg, &value);

        status[drv][reg] = ret;
        dump[drv][reg] = value;
        if (ret == DRV8703_OK)
            mask |= (uint8_t)(1U << reg);
    }

    ok_mask[drv] = mask;
    return mask;
}

/**
 * @brief Read all 6 DRV8703 registers into local buffers without touching
 *        global debug arrays.  Used inside the stable-retry loop.
 * @return Bitmask of successfully-read registers.
 */
static uint8_t AppControl_ReadDrvRegsLocal(DRV8703_Handle_t *dev,
                                           uint8_t regs[DRV8703_REGISTER_COUNT],
                                           DRV8703_Status_t status[DRV8703_REGISTER_COUNT])
{
    uint8_t reg;
    uint8_t mask = 0U;

    if (dev == 0)
        return 0U;

    for (reg = 0U; reg < DRV8703_REGISTER_COUNT; reg++)
    {
        uint8_t value = 0xFFU;
        DRV8703_Status_t ret = DRV8703_ReadReg(dev, reg, &value);

        regs[reg] = value;
        status[reg] = ret;
        if (ret == DRV8703_OK)
            mask |= (uint8_t)(1U << reg);
    }

    return mask;
}

/**
 * @brief Copy a set of local register values into the global debug arrays.
 */
static void AppControl_CommitDrvRegsToDebug(uint8_t drv,
                                            const uint8_t regs[DRV8703_REGISTER_COUNT],
                                            const DRV8703_Status_t status[DRV8703_REGISTER_COUNT],
                                            uint8_t mask,
                                            volatile uint8_t dump[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT],
                                            volatile DRV8703_Status_t debug_status[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT],
                                            volatile uint8_t ok_mask[APP_CONTROL_DRV_COUNT])
{
    uint8_t reg;

    if (drv >= APP_CONTROL_DRV_COUNT)
        return;

    for (reg = 0U; reg < DRV8703_REGISTER_COUNT; reg++)
    {
        dump[drv][reg] = regs[reg];
        debug_status[drv][reg] = status[reg];
    }

    ok_mask[drv] = mask;
}

/**
 * @brief Return the expected value for a DRV8703 configuration register.
 *
 * Uses the startup dump if valid, otherwise falls back to the hard-coded
 * defaults.
 */
static uint8_t AppControl_DrvExpectedReg(uint8_t drv, uint8_t reg)
{
    if ((drv < APP_CONTROL_DRV_COUNT) &&
        (g_app_control_drv_startup_dump_valid[drv] != 0U) &&
        (g_app_control_drv_startup_reg_dump[drv][reg] != 0xFFU))
    {
        return g_app_control_drv_startup_reg_dump[drv][reg];
    }

    return g_app_control_drv_startup_expected[reg];
}

/**
 * @brief Check whether a DRV8703 register snapshot is invalid (bad SPI data).
 *
 * A snapshot is invalid if it cannot be read in full, the fault-status
 * register is 0xFF, any configuration register differs from the expected
 * value, or a GDF/OCP fault is reported without any VDS/GDF detail bits.
 *
 * @return 1 if invalid, 0 if usable.
 */
static uint8_t AppControl_DrvRegSampleIsInvalid(uint8_t drv,
                                                const volatile uint8_t regs[DRV8703_REGISTER_COUNT],
                                                uint8_t mask)
{
    uint8_t reg;
    uint8_t fault_status;

    if ((drv >= APP_CONTROL_DRV_COUNT) || (regs == 0))
        return 1U;

    if (mask != 0x3FU)
        return 1U;

    fault_status = regs[DRV8703_REG_FAULT_STATUS];
    if (fault_status == 0xFFU)
        return 1U;

    for (reg = DRV8703_REG_MAIN_CONTROL; reg <= DRV8703_REG_CONFIG_CONTROL; reg++)
    {
        if (regs[reg] != AppControl_DrvExpectedReg(drv, reg))
            return 1U;
    }

    if (((fault_status & (DRV8703_FAULT_GDF | DRV8703_FAULT_OCP)) != 0U) &&
        (regs[DRV8703_REG_VDS_GDF_STATUS] == 0x00U))
    {
        return 1U;
    }

    return 0U;
}

/** @brief Compare two DRV8703 register snapshots for equality. */
static uint8_t AppControl_DrvRegSamplesEqual(const uint8_t a[DRV8703_REGISTER_COUNT],
                                             const uint8_t b[DRV8703_REGISTER_COUNT])
{
    uint8_t reg;

    for (reg = 0U; reg < DRV8703_REGISTER_COUNT; reg++)
    {
        if (a[reg] != b[reg])
            return 0U;
    }

    return 1U;
}

/**
 * @brief Read DRV8703 registers repeatedly until two consecutive snapshots
 *        are identical, or the retry limit is reached.  Used during fault
 *        capture to reject SPI noise.
 * @return Bitmask of successfully-read registers.
 */
static uint8_t AppControl_ReadDrvRegsToDebugStableRetry(uint8_t drv,
                                                        DRV8703_Handle_t *dev,
                                                        volatile uint8_t dump[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT],
                                                        volatile DRV8703_Status_t status[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT],
                                                        volatile uint8_t ok_mask[APP_CONTROL_DRV_COUNT])
{
    uint8_t retry;
    uint8_t mask = 0U;
    uint8_t regs[DRV8703_REGISTER_COUNT] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
    uint8_t last_valid_regs[DRV8703_REGISTER_COUNT] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
    DRV8703_Status_t reg_status[DRV8703_REGISTER_COUNT] =
        {
            DRV8703_OK, DRV8703_OK, DRV8703_OK,
            DRV8703_OK, DRV8703_OK, DRV8703_OK};
    DRV8703_Status_t last_valid_status[DRV8703_REGISTER_COUNT] =
        {
            DRV8703_OK, DRV8703_OK, DRV8703_OK,
            DRV8703_OK, DRV8703_OK, DRV8703_OK};
    uint8_t have_valid = 0U;
    uint8_t stable_count = 0U;
    uint8_t stable_mask = 0U;

    for (retry = 0U; retry < APP_CONTROL_DRV_FAULT_READ_RETRY_COUNT; retry++)
    {
        uint8_t invalid;

        mask = AppControl_ReadDrvRegsLocal(dev, regs, reg_status);
        invalid = AppControl_DrvRegSampleIsInvalid(drv, regs, mask);

        if (invalid != 0U)
        {
            stable_count = 0U;
            continue;
        }

        if ((have_valid != 0U) && (AppControl_DrvRegSamplesEqual(last_valid_regs, regs) != 0U))
        {
            stable_count++;
        }
        else
        {
            uint8_t reg;

            for (reg = 0U; reg < DRV8703_REGISTER_COUNT; reg++)
            {
                last_valid_regs[reg] = regs[reg];
                last_valid_status[reg] = reg_status[reg];
            }
            have_valid = 1U;
            stable_count = 1U;
            stable_mask = mask;
        }

        if (stable_count >= APP_CONTROL_DRV_FAULT_STABLE_READ_COUNT)
        {
            AppControl_CommitDrvRegsToDebug(drv,
                                            last_valid_regs,
                                            last_valid_status,
                                            stable_mask,
                                            dump,
                                            status,
                                            ok_mask);
            if (drv < APP_CONTROL_DRV_COUNT)
                g_app_control_drv_pin_fault_stable_count[drv] = stable_count;
            return stable_mask;
        }
    }

    AppControl_CommitDrvRegsToDebug(drv, regs, reg_status, mask, dump, status, ok_mask);
    if (drv < APP_CONTROL_DRV_COUNT)
        g_app_control_drv_pin_fault_stable_count[drv] = stable_count;

    return mask;
}

/**
 * @brief Record a DRV8703 fault in the global debug variables and attempt
 *        to read the fault register snapshot over SPI.
 */
static void AppControl_CaptureDrvFault(uint8_t drv, DRV8703_Status_t status)
{
    DRV8703_Handle_t *dev;
    uint8_t mask;

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

    mask = AppControl_ReadDrvRegsToDebug(drv,
                                         dev,
                                         g_app_control_drv_reg_dump,
                                         g_app_control_drv_reg_read_status,
                                         g_app_control_drv_reg_read_ok_mask);
    g_app_control_drv_fault_read_status[drv] =
        ((mask & 0x03U) == 0x03U) ? DRV8703_OK : g_app_control_drv_reg_read_status[drv][0];
    g_app_control_drv_dump_status[drv] =
        ((mask & 0x3FU) == 0x3FU) ? DRV8703_OK : DRV8703_ERROR_SPI;
    g_app_control_drv_fault_status[drv] = g_app_control_drv_reg_dump[drv][0];
    g_app_control_drv_vds_gdf_status[drv] = g_app_control_drv_reg_dump[drv][1];
    if (mask != 0U)
        g_app_control_drv_fault_snapshot_valid[drv] = 1U;
}

/**
 * @brief Determine whether a DRV8703 fault register requires stopping the
 *        temperature cell.  Only GDF, OCP and OTSD are stop-critical.
 * @return 1 if the cell should be stopped, 0 otherwise.
 */
static uint8_t AppControl_DrvFaultRequiresStop(uint8_t fault_status)
{
    uint8_t stop_bits;

    stop_bits = (uint8_t)(fault_status & APP_CONTROL_DRV_STOP_FAULT_MASK);
    return (stop_bits != 0U) ? 1U : 0U;
}

/**
 * @brief Handle a DRV8703 fault detected via the hardware fault pins.
 *
 * Stops PWM on all channels, busy-waits for bridge noise to settle, then
 * reads the SPI registers with stable-retry logic.  Clears the fault
 * afterwards so the chip can recover from latched faults.
 *
 * @return 1 if the cell should be stopped, 0 otherwise.
 */
static uint8_t AppControl_CaptureDrvPinFaultMoment(uint8_t drv, DRV8703_Status_t pin_status)
{
    DRV8703_Handle_t *dev;
    uint8_t i;
    uint8_t mask;
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
    g_app_control_drv_pin_fault_all_ff[drv] = 0U;
    g_app_control_drv_pin_fault_stable_count[drv] = 0U;

    g_app_control_drv_pin_fault_fault_status[drv] = 0xFFU;
    g_app_control_drv_pin_fault_vds_gdf_status[drv] = 0xFFU;
    for (i = 0U; i < DRV8703_REGISTER_COUNT; i++)
        g_app_control_drv_pin_fault_reg_dump[drv][i] = 0xFFU;

    if (g_app_control_simulate_drv8703 != 0U)
        return 0U;

    dev = DRV8703_BoardGet((DRV8703_BoardChannel_t)drv);
    if (dev == 0)
        return 0U;

    {
        uint8_t i;
        for (i = 0U; i < APP_CONTROL_DRV_COUNT; i++)
        {
            DRV8703_Handle_t *d = DRV8703_BoardGet((DRV8703_BoardChannel_t)i);
            if (d != 0)
                (void)DRV8703_SetDuty(d, 0.0f);
        }
    }
    {
        volatile uint32_t wait;
        for (wait = 0U; wait < 2000U; wait++)
            ;
    }

    mask = AppControl_ReadDrvRegsToDebugStableRetry(drv,
                                                    dev,
                                                    g_app_control_drv_pin_fault_reg_dump,
                                                    g_app_control_drv_pin_fault_reg_read_status,
                                                    g_app_control_drv_pin_fault_reg_read_ok_mask);
    if (AppControl_DrvRegSampleIsInvalid(drv, g_app_control_drv_pin_fault_reg_dump[drv], mask) != 0U)
    {
        g_app_control_drv_pin_fault_all_ff[drv] = 1U;
        g_app_control_drv_pin_fault_all_ff_count[drv]++;
        g_app_control_drv_pin_fault_read_status[drv] = DRV8703_ERROR_SPI;
        g_app_control_drv_pin_fault_dump_status[drv] = DRV8703_ERROR_SPI;
        g_app_control_drv_dump_status[drv] = DRV8703_ERROR_SPI;
        g_app_control_drv_reg_read_ok_mask[drv] = mask;

        for (i = 0U; i < DRV8703_REGISTER_COUNT; i++)
        {
            g_app_control_drv_reg_dump[drv][i] = g_app_control_drv_pin_fault_reg_dump[drv][i];
            g_app_control_drv_reg_read_status[drv][i] =
                g_app_control_drv_pin_fault_reg_read_status[drv][i];
        }

        return 0U;
    }

    g_app_control_drv_pin_fault_read_status[drv] =
        ((mask & 0x03U) == 0x03U) ? DRV8703_OK : g_app_control_drv_pin_fault_reg_read_status[drv][0];
    g_app_control_drv_pin_fault_dump_status[drv] =
        ((mask & 0x3FU) == 0x3FU) ? DRV8703_OK : DRV8703_ERROR_SPI;

    if ((mask & 0x01U) != 0U)
    {
        uint8_t fault_status = g_app_control_drv_pin_fault_reg_dump[drv][0];

        g_app_control_drv_pin_fault_fault_status[drv] = fault_status;
        if ((mask & 0x02U) != 0U)
            g_app_control_drv_pin_fault_vds_gdf_status[drv] =
                g_app_control_drv_pin_fault_reg_dump[drv][1];
        g_app_control_drv_pin_fault_stop_bits[drv] =
            (uint8_t)(fault_status & APP_CONTROL_DRV_STOP_FAULT_MASK);
        should_stop = AppControl_DrvFaultRequiresStop(fault_status);

        g_app_control_drv_fault_read_status[drv] = g_app_control_drv_pin_fault_read_status[drv];
        g_app_control_drv_fault_status[drv] = g_app_control_drv_pin_fault_fault_status[drv];
        g_app_control_drv_vds_gdf_status[drv] = g_app_control_drv_pin_fault_vds_gdf_status[drv];
        g_app_control_drv_fault_snapshot_valid[drv] = 1U;
    }

    if (mask != 0U)
    {
        for (i = 0U; i < DRV8703_REGISTER_COUNT; i++)
        {
            g_app_control_drv_reg_dump[drv][i] = g_app_control_drv_pin_fault_reg_dump[drv][i];
            g_app_control_drv_reg_read_status[drv][i] =
                g_app_control_drv_pin_fault_reg_read_status[drv][i];
        }

        g_app_control_drv_reg_read_ok_mask[drv] = mask;
        g_app_control_drv_dump_status[drv] = g_app_control_drv_pin_fault_dump_status[drv];
        g_app_control_drv_fault_snapshot_valid[drv] = 1U;
    }

    (void)DRV8703_ClearFault(dev);

    return should_stop;
}

/**
 * @brief Check whether a PanelError_t is a temperature-sensor related error.
 */
static uint8_t AppControl_IsTempSensorError(PanelError_t err)
{
    return (err == PANEL_ERR_E121_TEMP_CH1) ||
           (err == PANEL_ERR_E122_TEMP_CH2) ||
           (err == PANEL_ERR_E123_TEMP_CH3) ||
           (err == PANEL_ERR_E124_TEMP_CH4) ||
           (err == PANEL_ERR_E132_SENSOR);
}

/**
 * @brief Map a sensor channel index to its corresponding PanelError_t.
 */
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

/**
 * @brief  Check whether a temperature sensor is providing fresh data.
 *
 * Staleness is determined by counting consecutive control cycles where
 * Sys_TempUpdateCount has not changed.  Unlike a time-based deadline this
 * approach is immune to clock drift between HAL tick (TIM6) and RTOS tick
 * (SysTick) and it works identically regardless of the control loop period.
 *
 * @param sensor   Sensor index 0..3.
 * @param status   4-element array of sensor self-reported status (0 = ok).
 * @param count    4-element array of accumulated frame counters.
 * @param now_ms   Current RTOS tick (unused, kept for call-site compatibility).
 * @return 1 if data is fresh, 0 otherwise.
 */
static uint8_t AppControl_TempInputIsFresh(uint8_t sensor,
                                           const uint8_t status[4],
                                           const uint32_t count[4],
                                           uint32_t now_ms)
{
    (void)now_ms;

    if (sensor >= 4U)
        return 0U;
    if (status[sensor] != 0U)
        return 0U;
    if (count[sensor] == 0U)
        return 0U;

    if (s_temp_stale_cycles[sensor] >= APP_CONTROL_TEMP_STALE_CYCLES)
        return 0U;

    return 1U;
}

/**
 * @brief Set or clear a panel error for a given cell.
 *
 * Records the time when an error was first set so that the service routine
 * can automatically clear it after a display timeout.
 */
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

    /* PC 协议：故障时通知上位�?*/
    if (err != PANEL_ERR_NONE)
    {
        PcProto_SendEvent(cell, "ERROR");
        PcProto_SendState(cell);
    }

    if (!AppControl_IsTempSensorError(err))
        g_app_control_temp_fault_sensor[cell] = 0xFFU;
}

/**
 * @brief Clear a cell error after it has been displayed for the configured
 *        timeout period.
 */
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
            {
                const AppControlCellConfig_t *cfg = AppControl_CellConfig(cell);

                PID_Reset(&s_cell_pid[cell]);
                s_cell_pid_update_pending[cell] = 0U;
                s_cell_base_output_duty[cell] = 0.0f;
                if (cfg != 0)
                {
                    AppControl_RecordDrvDuty(cfg->drv_outer, 0.0f);
                    AppControl_RecordDrvDuty(cfg->drv_inner, 0.0f);
                }
                AppControl_RecordCellDuties(cell, 0.0f);
            }
            AppControl_SetCellError(cell, PANEL_ERR_NONE);
        }
    }
}

#if TEMP_VIRTUAL_LOW_ENABLE
static float AppControl_VirtualClamp(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

static float AppControl_VirtualNoise(AppControlVirtualLowTemp_t *state)
{
    float rnd;

    state->rng = state->rng * 1664525UL + 1013904223UL;
    rnd = (float)((state->rng >> 16) & 0x7FFFU) / 16383.5f - 1.0f;
    state->noise = state->noise * 0.92f + rnd * APP_CONTROL_VIRTUAL_NOISE_MAX_C * 0.08f;
    return state->noise;
}

static void AppControl_VirtualReset(uint8_t cell, float real_temp, uint32_t now_ms)
{
    AppControlVirtualLowTemp_t *state;

    if (cell >= APP_CONTROL_CELL_COUNT)
        return;

    state = &s_virtual_low[cell];
    state->active = 0U;
    state->started = 0U;
    state->last_real_temp = real_temp;
    state->learned_rate_cps = APP_CONTROL_VIRTUAL_DEFAULT_RATE_CPS;
    state->virtual_temp = real_temp;
    state->noise = 0.0f;
    state->last_ms = now_ms;
}

static float AppControl_VirtualLowDisplay(uint8_t cell,
                                          float real_temp,
                                          float target_temp,
                                          uint32_t now_ms)
{
    AppControlVirtualLowTemp_t *state;
    float dt_s;
    float target;

    if (cell >= APP_CONTROL_CELL_COUNT)
        return real_temp;

    state = &s_virtual_low[cell];

    if ((s_cell[cell].running == 0U) || (target_temp >= PANEL_TEMP_REAL_MIN))
    {
        AppControl_VirtualReset(cell, real_temp, now_ms);
        return real_temp;
    }

    target = AppControl_VirtualClamp(target_temp, PANEL_TEMP_DISPLAY_MIN, PANEL_TEMP_REAL_MIN);

    if (state->active == 0U)
    {
        state->last_ms = now_ms;
        state->last_real_temp = real_temp;
        state->virtual_temp = real_temp;
    }

    dt_s = (float)(now_ms - state->last_ms) * 0.001f;
    if (dt_s < 0.0f)
        dt_s = 0.0f;
    if (dt_s > 2.0f)
        dt_s = 2.0f;

    if (dt_s > 0.0f)
    {
        float real_rate = (state->last_real_temp - real_temp) / dt_s;
        if (real_rate > 0.0f)
        {
            real_rate = AppControl_VirtualClamp(real_rate,
                                                APP_CONTROL_VIRTUAL_RATE_MIN_CPS,
                                                APP_CONTROL_VIRTUAL_RATE_MAX_CPS);
            state->learned_rate_cps = state->learned_rate_cps * 0.90f + real_rate * 0.10f;
        }
    }

    state->last_ms = now_ms;
    state->last_real_temp = real_temp;
    state->active = 1U;

    if (state->started == 0U)
    {
        state->virtual_temp = real_temp;
        if (real_temp > APP_CONTROL_VIRTUAL_ENTER_TEMP_C)
            return real_temp;

        state->started = 1U;
        if (state->virtual_temp > PANEL_TEMP_REAL_MIN)
            state->virtual_temp = PANEL_TEMP_REAL_MIN;
    }

    if (dt_s > 0.0f)
    {
        if (state->virtual_temp > target)
        {
            float distance = state->virtual_temp - target;
            float slow = AppControl_VirtualClamp(distance / 5.0f, 0.15f, 1.0f);
            float rate = state->learned_rate_cps;
            if (rate < APP_CONTROL_VIRTUAL_RATE_MIN_CPS)
                rate = APP_CONTROL_VIRTUAL_DEFAULT_RATE_CPS;
            state->virtual_temp -= rate * slow * dt_s;
            if (state->virtual_temp < target)
                state->virtual_temp = target;
        }
        else if (state->virtual_temp < target)
        {
            state->virtual_temp = target;
        }
    }

    return AppControl_VirtualClamp(state->virtual_temp + AppControl_VirtualNoise(state),
                                   target - 0.15f,
                                   PANEL_TEMP_REAL_MIN + 0.20f);
}

static void AppControl_UpdateDisplayTemperatures(uint32_t now_ms)
{
    uint8_t cell;

    for (cell = 0U; cell < APP_CONTROL_CELL_COUNT; cell++)
    {
        const AppControlCellConfig_t *cfg = AppControl_CellConfig(cell);
        float real_cell = s_cell[cell].current_temp;
        float display_cell = AppControl_VirtualLowDisplay(cell,
                                                          real_cell,
                                                          s_cell[cell].target_temp,
                                                          now_ms);
        float delta = display_cell - real_cell;

        s_cell_display_temp[cell] = display_cell;
        if (cfg != 0)
        {
            if (cfg->temp_outer < APP_CONTROL_CLOSED_LOOP_COUNT)
                s_temp_channel_display_temp[cfg->temp_outer] =
                    s_temp_channel_temp[cfg->temp_outer] + delta;
            if (cfg->temp_inner < APP_CONTROL_CLOSED_LOOP_COUNT)
                s_temp_channel_display_temp[cfg->temp_inner] =
                    s_temp_channel_temp[cfg->temp_inner] + delta;
        }
    }
}
#endif

/**
 * @brief Copy internal variables into the g_ prefixed volatile debug arrays
 *        so they can be inspected via Keil Watch windows.
 */
static void AppControl_ApplyDebugState(void)
{
    uint8_t i;

#if TEMP_VIRTUAL_LOW_ENABLE
    AppControl_UpdateDisplayTemperatures(osKernelGetTickCount());
#endif

    for (i = 0U; i < APP_CONTROL_CELL_COUNT; i++)
    {
        g_app_control_cell_running[i] = s_cell[i].running;
#if TEMP_VIRTUAL_LOW_ENABLE
        g_app_control_cell_temp[i] = s_cell_display_temp[i];
#else
        g_app_control_cell_temp[i] = s_cell[i].current_temp;
#endif
        g_app_control_cell_target[i] = s_cell[i].target_temp;
        g_app_control_cell_duty[i] = s_cell[i].duty;
        g_app_control_cell_outer_duty[i] = AppControl_GetCellOuterDuty(i);
        g_app_control_cell_inner_duty[i] = AppControl_GetCellInnerDuty(i);
        g_app_control_cell_inner_ratio[i] = AppControl_GetCellInnerDutyRatio(i);
        g_app_control_cell_error[i] = s_cell[i].error;
    }

    for (i = 0U; i < APP_CONTROL_CLOSED_LOOP_COUNT; i++)
    {
#if TEMP_VIRTUAL_LOW_ENABLE
        g_app_control_pid_temp[i] = s_temp_channel_display_temp[i];
#else
        g_app_control_pid_temp[i] = s_temp_channel_temp[i];
#endif
        g_app_control_pid_duty[i] = s_temp_channel_duty[i];
        g_app_control_pid_update_pending[i] = s_temp_pid_update_pending[i];
    }
}

/**
 * @brief Initialise a DRV8703 channel: hardware init, default config,
 *        VREF, ClearFault, Lock, and capture startup register snapshot.
 *
 * Retries up to APP_CONTROL_DRV_RETRY_COUNT times on failure.
 */
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
            (void)DRV8703_Lock(dev);
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

/**
 * @brief Set a DRV8703 channel duty cycle.
 *
 * Automatically prepares the chip if not already ready, wakes it if
 * sleeping (re-applies configuration registers lost during sleep), and
 * locks the registers afterwards.
 */
DRV8703_Status_t AppControl_SetDrvDuty(uint8_t drv, float duty)
{
    DRV8703_Handle_t *dev;
    DRV8703_Status_t ret;

    if (drv >= APP_CONTROL_DRV_COUNT)
        return DRV8703_ERROR_PARAM;

    duty = AppControl_Clamp(duty,
                            -APP_CONTROL_MAX_ABS_DUTY,
                            APP_CONTROL_MAX_ABS_DUTY);
    AppControl_RecordDrvDuty(drv, duty);

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

        ret = DRV8703_BoardApplyDefaultConfig((DRV8703_BoardChannel_t)drv);
        if (ret == DRV8703_OK)
            ret = DRV8703_SetVrefMv(dev, APP_CONTROL_DRV_VREF_MV);
        if (ret == DRV8703_OK)
            ret = DRV8703_ClearFault(dev);
        if (ret == DRV8703_OK)
            (void)DRV8703_Lock(dev);
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

DRV8703_Status_t AppControl_SetCellStackDuty(uint8_t cell, float outer_duty)
{
    const AppControlCellConfig_t *cfg = AppControl_CellConfig(cell);
    DRV8703_Status_t ret;
    float inner_duty;

    if (cfg == 0)
        return DRV8703_ERROR_PARAM;
    if (cfg->drv_outer >= APP_CONTROL_CLOSED_LOOP_COUNT ||
        cfg->drv_inner >= APP_CONTROL_CLOSED_LOOP_COUNT)
        return DRV8703_ERROR_PARAM;

    outer_duty = AppControl_Clamp(outer_duty,
                                  -APP_CONTROL_MAX_ABS_DUTY,
                                  APP_CONTROL_MAX_ABS_DUTY);
    inner_duty = AppControl_Clamp(outer_duty * AppControl_CellInnerRatio(cell),
                                  -APP_CONTROL_MAX_ABS_DUTY,
                                  APP_CONTROL_MAX_ABS_DUTY);

    s_cell_base_output_duty[cell] = outer_duty;
    AppControl_RecordCellDuties(cell, outer_duty);

    ret = AppControl_SetDrvDuty(cfg->drv_outer, outer_duty);
    if (ret != DRV8703_OK)
        return ret;

    ret = AppControl_SetDrvDuty(cfg->drv_inner, inner_duty);
    return ret;
}

uint8_t AppControl_GetCellOuterTempIndex(uint8_t cell)
{
    const AppControlCellConfig_t *cfg = AppControl_CellConfig(cell);
    return (cfg == 0) ? APP_CONTROL_INVALID_INDEX : cfg->temp_outer;
}

uint8_t AppControl_GetCellInnerTempIndex(uint8_t cell)
{
    const AppControlCellConfig_t *cfg = AppControl_CellConfig(cell);
    return (cfg == 0) ? APP_CONTROL_INVALID_INDEX : cfg->temp_inner;
}

float AppControl_GetCellInnerDutyRatio(uint8_t cell)
{
    return AppControl_CellInnerRatio(cell);
}

float AppControl_GetCellOuterDuty(uint8_t cell)
{
    const AppControlCellConfig_t *cfg = AppControl_CellConfig(cell);

    if (cfg == 0 || cfg->drv_outer >= APP_CONTROL_CLOSED_LOOP_COUNT)
        return 0.0f;
    return s_temp_channel_duty[cfg->drv_outer];
}

float AppControl_GetCellInnerDuty(uint8_t cell)
{
    const AppControlCellConfig_t *cfg = AppControl_CellConfig(cell);

    if (cfg == 0 || cfg->drv_inner >= APP_CONTROL_CLOSED_LOOP_COUNT)
        return 0.0f;
    return s_temp_channel_duty[cfg->drv_inner];
}

/**
 * @brief Read all DRV8703 registers and store them in the sleep-register
 *        snapshot array before putting the chip to sleep.
 */
static void AppControl_CaptureDrvSleepRegs(uint8_t drv, DRV8703_Handle_t *dev)
{
    uint8_t reg;
    uint8_t mask = 0U;

    if ((drv >= APP_CONTROL_DRV_COUNT) || (dev == 0))
        return;

    for (reg = 0U; reg < DRV8703_REGISTER_COUNT; reg++)
    {
        uint8_t value = 0xFFU;
        DRV8703_Status_t ret = DRV8703_ReadReg(dev, reg, &value);

        g_app_control_sleep_reg_snapshot[drv][reg] = value;
        if (ret == DRV8703_OK)
            mask |= (uint8_t)(1U << reg);
    }

    g_app_control_sleep_reg_snapshot[drv][DRV8703_REGISTER_COUNT] = mask;
}

/**
 * @brief Periodically capture a register snapshot of all awake DRV8703
 *        chips.  Throttled to APP_CONTROL_DRV_REG_SNAPSHOT_MS.
 */
static void AppControl_CapturePeriodicDrvRegs(uint32_t now_ms)
{
    uint8_t drv;

    if (g_app_control_simulate_drv8703 != 0U)
        return;
    if ((now_ms - s_last_reg_snapshot_ms) < APP_CONTROL_DRV_REG_SNAPSHOT_MS)
        return;

    s_last_reg_snapshot_ms = now_ms;

    for (drv = 0U; drv < APP_CONTROL_DRV_COUNT; drv++)
    {
        DRV8703_Handle_t *dev;
        uint8_t reg;
        uint8_t mask = 0U;

        for (reg = 0U; reg < DRV8703_REGISTER_COUNT; reg++)
            g_app_control_periodic_reg_snapshot[drv][reg] = 0xFFU;
        g_app_control_periodic_reg_snapshot[drv][DRV8703_REGISTER_COUNT] = 0U;

        if (g_app_control_drv_ready[drv] == 0U)
            continue;

        if (g_app_control_drv_awake[drv] == 0U)
            continue;

        dev = DRV8703_BoardGet((DRV8703_BoardChannel_t)drv);
        if (dev == 0)
            continue;

        for (reg = 0U; reg < DRV8703_REGISTER_COUNT; reg++)
        {
            uint8_t value = 0xFFU;
            DRV8703_Status_t ret = DRV8703_ReadReg(dev, reg, &value);

            g_app_control_periodic_reg_snapshot[drv][reg] = value;
            if (ret == DRV8703_OK)
                mask |= (uint8_t)(1U << reg);
        }

        g_app_control_periodic_reg_snapshot[drv][DRV8703_REGISTER_COUNT] = mask;
    }

    g_app_control_periodic_reg_snapshot_count++;
}

/**
 * @brief Put a DRV8703 chip to sleep: zero duty, capture sleep registers, sleep.
 */
static void AppControl_SleepDrv(uint8_t drv)
{
    DRV8703_Handle_t *dev;

    if (drv >= APP_CONTROL_DRV_COUNT || g_app_control_simulate_drv8703 != 0U)
        return;

    dev = DRV8703_BoardGet((DRV8703_BoardChannel_t)drv);
    if (dev != 0)
    {
        (void)DRV8703_SetDuty(dev, 0.0f);
        AppControl_CaptureDrvSleepRegs(drv, dev);
        (void)DRV8703_Sleep(dev);
        g_app_control_drv_awake[drv] = 0U;
    }
}

/**
 * @brief Check whether either cell is running (shared DRV channel needed).
 */
static uint8_t AppControl_SharedDrvNeeded(void)
{
    uint8_t cell;

    for (cell = 0U; cell < APP_CONTROL_CELL_COUNT; cell++)
    {
        const AppControlCellConfig_t *cfg = AppControl_CellConfig(cell);
        if (cfg != 0 && cfg->shared_drv != APP_CONTROL_INVALID_INDEX &&
            s_cell[cell].running != 0U)
        {
            return 1U;
        }
    }

    return 0U;
}

static uint8_t AppControl_ActiveCellNeedsSharedDrv(void)
{
    return (AppControl_SharedTargetDuty() != 0.0f) ? 1U : 0U;
}

/**
 * @brief Stop a temperature cell: zero duty, sleep its DRV8703 channels,
 *        sleep the shared channel if no other cell needs it.
 */
static void AppControl_StopCell(uint8_t cell, AppControlStopMode_t mode)
{
    const AppControlCellConfig_t *cfg;
    uint8_t was_running;

    if (cell >= APP_CONTROL_CELL_COUNT)
        return;

    cfg = AppControl_CellConfig(cell);
    was_running = s_cell[cell].running;

    if (mode == APP_CONTROL_STOP_NORMAL)
    {
        if (was_running == 0U)
        {
            s_cell[cell].requested = 0U;
            s_cell[cell].stopping = 0U;
            return;
        }
        s_cell[cell].requested = 0U;
        s_cell[cell].stopping = 1U;
        s_cell[cell].pid_update_pending = 0U;
        s_cell_pid_update_pending[cell] = 0U;
        PID_Reset(&s_cell_pid[cell]);
        s_cell_pid_feedback_enabled[cell] = 0U;
        s_cell_pid_feedback_start_ms[cell] = 0U;
        return;
    }

    s_cell[cell].running = 0U;
    s_cell[cell].requested = 0U;
    s_cell[cell].stopping = 0U;
    s_cell[cell].pid_update_pending = 0U;
    s_cell[cell].duty = 0.0f;
    s_cell_pid_update_pending[cell] = 0U;
    s_cell_last_pid_ms[cell] = 0U;
    s_cell_base_output_duty[cell] = 0.0f;
    PID_Reset(&s_cell_pid[cell]);
    s_cell_pid_feedback_enabled[cell] = 0U;
    s_cell_pid_feedback_start_ms[cell] = 0U;

    AppControl_RecordCellDuties(cell, 0.0f);
    if (cfg != 0)
    {
        (void)AppControl_SetDrvDuty(cfg->drv_outer, 0.0f);
        (void)AppControl_SetDrvDuty(cfg->drv_inner, 0.0f);
        AppControl_SleepDrv(cfg->drv_outer);
        AppControl_SleepDrv(cfg->drv_inner);
    }

    if (!AppControl_SharedDrvNeeded())
    {
        (void)AppControl_SetDrvDuty(APP_CONTROL_SHARED_DRV, 0.0f);
        AppControl_SleepDrv(APP_CONTROL_SHARED_DRV);
    }

    if (was_running != 0U)
    {
        PcProto_SendEvent(cell, "STOP");
        PcProto_SendState(cell);
    }
}

static void AppControl_ServiceStoppingCell(uint8_t cell)
{
    float outer_duty;

    if (cell >= APP_CONTROL_CELL_COUNT)
        return;

    s_cell_pid_update_pending[cell] = 0U;
    PID_Reset(&s_cell_pid[cell]);

    outer_duty = AppControl_ApplyCellDutySlew(cell,
                                              0.0f,
                                              APP_CONTROL_TEMP_PID_DT_MIN_S);
    if (AppControl_Abs(outer_duty) < 0.001f)
        outer_duty = 0.0f;

    if (AppControl_SetCellStackDuty(cell, outer_duty) != DRV8703_OK)
    {
        AppControl_StopCell(cell, APP_CONTROL_STOP_EMERGENCY);
        return;
    }

    if (outer_duty == 0.0f)
        AppControl_StopCell(cell, APP_CONTROL_STOP_EMERGENCY);
}
/**
 * @brief Check whether a cell currently has no error.
 */
static uint8_t AppControl_CellHardwareOk(uint8_t cell)
{
    if (cell >= APP_CONTROL_CELL_COUNT)
        return 0U;

    if (s_cell[cell].error != PANEL_ERR_NONE)
        return 0U;

    return 1U;
}

/**
 * @brief Start a temperature cell: prepare its DRV8703 channels, initialise
 *        PID controllers, and mark the cell as running.
 */
static void AppControl_StartCell(uint8_t cell)
{
    const AppControlCellConfig_t *cfg;
    uint8_t failed_drv;
    DRV8703_Status_t ret;

    if (cell >= APP_CONTROL_CELL_COUNT)
        return;

    cfg = AppControl_CellConfig(cell);
    if (cfg == 0)
        return;
    if (cfg->temp_outer >= APP_CONTROL_TEMP_INPUT_COUNT ||
        cfg->temp_inner >= APP_CONTROL_TEMP_INPUT_COUNT ||
        cfg->drv_outer >= APP_CONTROL_CLOSED_LOOP_COUNT ||
        cfg->drv_inner >= APP_CONTROL_CLOSED_LOOP_COUNT)
        return;

    if (!AppControl_CellHardwareOk(cell))
    {
        AppControl_StopCell(cell, APP_CONTROL_STOP_EMERGENCY);
        return;
    }

    failed_drv = cfg->drv_outer;
    ret = AppControl_PrepareDrv(cfg->drv_outer);
    if (ret == DRV8703_OK)
    {
        failed_drv = cfg->drv_inner;
        ret = AppControl_PrepareDrv(cfg->drv_inner);
    }
    if (ret == DRV8703_OK && cfg->shared_drv != APP_CONTROL_INVALID_INDEX)
    {
        failed_drv = cfg->shared_drv;
        ret = AppControl_PrepareDrv(cfg->shared_drv);
    }

    if (ret != DRV8703_OK)
    {
        if (AppControl_DrvIsShared(failed_drv) != 0U)
        {
            AppControl_SetCellError(0U, PANEL_ERR_E315_SHARED_DRV);
            AppControl_SetCellError(1U, PANEL_ERR_E315_SHARED_DRV);
            AppControl_StopCell(0U, APP_CONTROL_STOP_EMERGENCY);
            AppControl_StopCell(1U, APP_CONTROL_STOP_EMERGENCY);
        }
        else
        {
            AppControl_SetCellError(cell, AppControl_CellDrvError(cell));
            AppControl_StopCell(cell, APP_CONTROL_STOP_EMERGENCY);
        }
        return;
    }

    PID_Reset(&s_cell_pid[cell]);
    PID_SetTarget(&s_cell_pid[cell], s_cell[cell].target_temp);
    s_temp_last_pid_count[cfg->temp_outer] = Sys_TempUpdateCount[cfg->temp_outer];
    s_temp_last_pid_count[cfg->temp_inner] = Sys_TempUpdateCount[cfg->temp_inner];
    s_cell_last_pid_ms[cell] = osKernelGetTickCount();
    s_cell_pid_update_pending[cell] = 0U;
    s_temp_pid_update_pending[cfg->temp_outer] = 0U;
    s_temp_pid_update_pending[cfg->temp_inner] = 0U;
    s_cell_base_output_duty[cell] = 0.0f;
    AppControl_RecordCellDuties(cell, 0.0f);
    s_cell_pid_feedback_enabled[cell] = 0U;
    s_cell_pid_feedback_start_ms[cell] = osKernelGetTickCount();
    s_cell[cell].pid_update_pending = 0U;
    s_cell[cell].running = 1U;
    s_cell[cell].requested = 1U;
    s_cell[cell].stopping = 0U;

    PcProto_SendEvent(cell, "START");
    PcProto_SendState(cell);
}
/**
 * @brief Dequeue and process one or more pending start/stop commands.
 */
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
            AppControl_StopCell(cmd.cell, cmd.stop_mode);
    }
}

/**
 * @brief Read current temperature inputs from the ISR-populated global
 *        arrays, track staleness per channel, compute temperature update
 *        frequency, evaluate freshness for each cell's sensor pair, and
 *        trigger a sensor reset if data is stale or the sensor reports
 *        an error status.
 *
 * Freshness is count-based: a sensor is considered stale after
 * APP_CONTROL_TEMP_STALE_CYCLES consecutive control cycles with no
 * increment in its frame counter.  This is immune to clock drift between
 * the HAL timer (TIM6) and the FreeRTOS SysTick.
 */
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

        if (count[i] != s_temp_last_count_for_stale[i])
        {
            s_temp_last_count_for_stale[i] = count[i];
            s_temp_stale_cycles[i] = 0U;
        }
        else
        {
            if (s_temp_stale_cycles[i] < APP_CONTROL_TEMP_STALE_CYCLES)
                s_temp_stale_cycles[i]++;
        }

        if (count[i] != s_temp_freq_last_count[i])
        {
            uint32_t dt = now_ms - s_temp_freq_last_tick[i];
            if (dt > 0U && s_temp_freq_last_tick[i] != 0U)
                g_app_control_temp_freq_hz[i] = 1000.0f / (float)dt;
            s_temp_freq_last_count[i] = count[i];
            s_temp_freq_last_tick[i] = now_ms;
        }
    }

    for (cell = 0U; cell < APP_CONTROL_CELL_COUNT; cell++)
    {
        const AppControlCellConfig_t *cfg = AppControl_CellConfig(cell);
        uint8_t a;
        uint8_t b;
        uint8_t a_fresh;
        uint8_t b_fresh;

        if (cfg == 0)
            continue;

        a = cfg->temp_outer;
        b = cfg->temp_inner;
        if (a >= 4U || b >= 4U)
            continue;

        a_fresh = AppControl_TempInputIsFresh(a, status, count, now_ms);
        b_fresh = AppControl_TempInputIsFresh(b, status, count, now_ms);

        if (a_fresh != 0U)
            s_temp_channel_temp[a] = temp[a];
        if (b_fresh != 0U)
            s_temp_channel_temp[b] = temp[b];

        if ((a_fresh != 0U) && (b_fresh != 0U))
        {
            s_cell[cell].current_temp = (temp[a] + temp[b]) * 0.5f;
            if (AppControl_IsTempSensorError(s_cell[cell].error))
            {
                g_app_control_temp_fault_sensor[cell] = 0xFFU;
                AppControl_SetCellError(cell, PANEL_ERR_NONE);
            }

            if (s_cell[cell].sensor_reset_count > 0U)
                s_cell[cell].sensor_reset_count--;
            if (g_app_control_temp_reset_count[cell] > 0U)
                g_app_control_temp_reset_count[cell]--;

            if (count[a] != s_temp_last_pid_count[a] ||
                count[b] != s_temp_last_pid_count[b])
            {
                if (count[a] != s_temp_last_pid_count[a])
                {
                    s_temp_last_pid_count[a] = count[a];
                    s_temp_pid_update_pending[a] = 1U;
                }
                if (count[b] != s_temp_last_pid_count[b])
                {
                    s_temp_last_pid_count[b] = count[b];
                    s_temp_pid_update_pending[b] = 1U;
                }
                s_cell_pid_update_pending[cell] = 1U;
                s_cell[cell].pid_update_pending = 1U;
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
                AppControl_StopCell(cell, APP_CONTROL_STOP_EMERGENCY);
            }
        }
    }
}
/**
 * @brief Check input voltage faults.  Currently inactive (body commented out).
 */
static void AppControl_CheckVoltageFaults(void)
{
}

/**
 * @brief Unconditionally read all 6 registers of all 5 DRV8703 chips for
 *        debug purposes.  Polls every chip regardless of ready/awake state.
 *        Throttled to APP_CONTROL_DRV_RAW_POLL_MS.
 */
static void AppControl_ReadAllDrvRegsRaw(uint32_t now_ms)
{
    uint8_t drv;

    if (g_app_control_simulate_drv8703 != 0U)
        return;
    if ((now_ms - s_last_raw_poll_ms) < APP_CONTROL_DRV_RAW_POLL_MS)
        return;

    s_last_raw_poll_ms = now_ms;

    for (drv = 0U; drv < APP_CONTROL_DRV_COUNT; drv++)
    {
        DRV8703_Handle_t *dev;
        uint8_t reg;
        uint8_t mask = 0U;

        for (reg = 0U; reg < DRV8703_REGISTER_COUNT; reg++)
            g_app_control_drv_raw_dump[drv][reg] = 0xFFU;
        g_app_control_drv_raw_mask[drv] = 0U;
        g_app_control_drv_raw_status[drv] = DRV8703_ERROR_PARAM;

        dev = DRV8703_BoardGet((DRV8703_BoardChannel_t)drv);
        if (dev == 0)
            continue;

        for (reg = 0U; reg < DRV8703_REGISTER_COUNT; reg++)
        {
            uint8_t value = 0xFFU;
            DRV8703_Status_t ret = DRV8703_ReadReg(dev, reg, &value);

            g_app_control_drv_raw_dump[drv][reg] = value;
            if (ret == DRV8703_OK)
                mask |= (uint8_t)(1U << reg);
            else if (g_app_control_drv_raw_status[drv] == DRV8703_ERROR_PARAM)
                g_app_control_drv_raw_status[drv] = ret;
        }

        g_app_control_drv_raw_mask[drv] = mask;
        if (mask != 0U)
            g_app_control_drv_raw_status[drv] =
                (mask == 0x3FU) ? DRV8703_OK : DRV8703_ERROR_SPI;
    }

    g_app_control_drv_raw_poll_count++;
}

/**
 * @brief Poll the hardware fault pins of all awake DRV8703 chips.
 *
 * If a pin fault is detected, capture a register snapshot via SPI, clear
 * the fault, and stop the affected cell if the fault is stop-critical.
 */
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

            if (AppControl_DrvIsShared(drv) != 0U)
            {
                AppControl_SetCellError(0U, PANEL_ERR_E315_SHARED_DRV);
                AppControl_SetCellError(1U, PANEL_ERR_E315_SHARED_DRV);
                AppControl_StopCell(0U, APP_CONTROL_STOP_EMERGENCY);
                AppControl_StopCell(1U, APP_CONTROL_STOP_EMERGENCY);
            }
            else
            {
                uint8_t owner_cell = AppControl_CellForDrv(drv);
                if (owner_cell != APP_CONTROL_INVALID_INDEX)
                {
                    AppControl_SetCellError(owner_cell, AppControl_CellDrvError(owner_cell));
                    AppControl_StopCell(owner_cell, APP_CONTROL_STOP_EMERGENCY);
                }
            }
        }
    }
}

/**
 * @brief Return calibration fault code when any active DRV8703 reports fault.
 */
static uint32_t AppControl_GetCalibDrvFault(void)
{
    uint8_t cell = (uint8_t)g_calib_cell;
    const AppControlCellConfig_t *cfg;

    if (cell >= APP_CONTROL_CELL_COUNT)
        return CALIB_ERR_DRV_FAULT;

    cfg = AppControl_CellConfig(cell);
    if (cfg == 0)
        return CALIB_ERR_DRV_FAULT;

    if (g_app_control_drv_fault[cfg->drv_outer] != 0U)
        return CALIB_ERR_DRV_FAULT;
    if (g_app_control_drv_fault[cfg->drv_inner] != 0U)
        return CALIB_ERR_DRV_FAULT;
    if (cfg->shared_drv != APP_CONTROL_INVALID_INDEX &&
        g_app_control_drv_fault[cfg->shared_drv] != 0U)
        return CALIB_ERR_DRV_FAULT;

    return 0U;
}
/* 标定数据加载与查�?---------------------------------------------------------*/

/**
 * @brief  �?Flash 加载指定 Cell 的标定数据到 RAM 缓存
 * @param  cell Cell 编号 (0 �?1)
 * @note   应在标定模式结束后、正常温控启动前调用
 *         加载失败时设�?loaded=0，后续前馈退化为 0
 */
static void AppControl_LoadCalibData(uint8_t cell)
{
    CalibFlashData_t flash_data;
    uint8_t ret;
    uint8_t i;

    if (cell >= APP_CONTROL_CELL_COUNT)
        return;

    s_calib_cache[cell].loaded = 0U;

    memset(&flash_data, 0, sizeof(flash_data));
    ret = CalibMode_LoadFromFlash(cell, &flash_data);
    if (ret != 0U)
        return;

    for (i = 0U; i < CALIB_DUTY_COUNT; i++)
    {
        s_calib_cache[cell].duty[i] = flash_data.step[i].duty;
        s_calib_cache[cell].temp_ch0[i] = flash_data.step[i].temp_ch0;
        s_calib_cache[cell].temp_ch1[i] = flash_data.step[i].temp_ch1;
        s_calib_cache[cell].temp_avg[i] =
            (flash_data.step[i].temp_ch0 + flash_data.step[i].temp_ch1) * 0.5f;
    }

    s_calib_cache[cell].loaded = 1U;
}

/**
 * @brief  根据目标温度查标定表，返回前馈占空比
 * @param  drv        DRV 通道索引 (0~3)
 * @param  target_temp 目标温度 (°C)
 * @return float 前馈占空�?
 * @note   在标定点之间线性插值，超出范围取最近端�?
 *         标定未加载时返回 0.0f
 */
static float AppControl_FeedforwardDuty(uint8_t cell, float target_temp)
{
    const float *temps;
    const float *dutys;
    uint8_t i;

    if (cell >= APP_CONTROL_CELL_COUNT)
        return 0.0f;
    if (s_calib_cache[cell].loaded == 0U)
        return 0.0f;

    dutys = s_calib_cache[cell].duty;
    temps = s_calib_cache[cell].temp_avg;

    if (target_temp <= temps[0])
        return dutys[0];

    if (target_temp >= temps[CALIB_DUTY_COUNT - 1U])
        return dutys[CALIB_DUTY_COUNT - 1U];

    for (i = 0U; i < (CALIB_DUTY_COUNT - 1U); i++)
    {
        float t_lo = temps[i];
        float t_hi = temps[i + 1U];

        if (t_hi <= t_lo)
            continue;

        if (target_temp >= t_lo && target_temp <= t_hi)
        {
            float ratio = (target_temp - t_lo) / (t_hi - t_lo);
            return dutys[i] + (dutys[i + 1U] - dutys[i]) * ratio;
        }
    }

    for (i = 1U; i < CALIB_DUTY_COUNT; i++)
    {
        if (temps[i] > target_temp)
            return dutys[i - 1U];
    }

    return dutys[CALIB_DUTY_COUNT - 1U];
}
static uint8_t AppControl_NormalPidFeedbackReady(uint8_t cell)
{
#if APP_CONTROL_NORMAL_PID_DELAY_ENABLE
    uint32_t now_ms;
    float error;

    if (cell >= APP_CONTROL_CELL_COUNT)
        return 1U;

    if (g_panel.cell[cell].run_mode == CELL_RUN_PROGRAM)
        return 1U;

    if (s_cell_pid_feedback_enabled[cell] != 0U)
        return 1U;

    now_ms = osKernelGetTickCount();
    error = AppControl_Abs(s_cell[cell].target_temp - s_cell[cell].current_temp);
    if ((error <= APP_CONTROL_NORMAL_PID_ENTER_BAND_C) ||
        ((now_ms - s_cell_pid_feedback_start_ms[cell]) >= APP_CONTROL_NORMAL_PID_ENTER_TIMEOUT_MS))
    {
        PID_Reset(&s_cell_pid[cell]);
        s_cell_pid_feedback_enabled[cell] = 1U;
        return 1U;
    }

    return 0U;
#else
    (void)cell;
    return 1U;
#endif
}

/**
 * @brief Run the closed-loop PID control for all active cells.
 *
 * Each cell uses the mean of its configured outer/inner temperature inputs.
 * The PID result is the outer Peltier duty; the inner Peltier follows it by
 * the configured ratio. Feedforward duty from the calibration table is added
 * to the PID output. The shared DRV8703 channel 5 is driven at a fixed duty
 * whenever any cell is running.
 */
static void AppControl_RunClosedLoop(void)
{
    uint8_t cell;
    uint8_t any_running = 0U;

    for (cell = 0U; cell < APP_CONTROL_CELL_COUNT; cell++)
    {
        const AppControlCellConfig_t *cfg = AppControl_CellConfig(cell);
        float outer_duty;

        if (cfg == 0)
            continue;
        if (cfg->temp_outer >= APP_CONTROL_TEMP_INPUT_COUNT ||
            cfg->temp_inner >= APP_CONTROL_TEMP_INPUT_COUNT ||
            cfg->drv_outer >= APP_CONTROL_CLOSED_LOOP_COUNT ||
            cfg->drv_inner >= APP_CONTROL_CLOSED_LOOP_COUNT)
            continue;

        if (s_cell[cell].running == 0U || s_cell[cell].error != PANEL_ERR_NONE)
        {
            s_cell[cell].duty = 0.0f;
            continue;
        }

        any_running = 1U;
        if (s_cell[cell].stopping != 0U)
        {
            AppControl_ServiceStoppingCell(cell);
            continue;
        }

        if (s_cell_pid_update_pending[cell] == 0U)
            continue;

        s_cell_pid_update_pending[cell] = 0U;
        s_cell[cell].pid_update_pending = 0U;
        s_temp_pid_update_pending[cfg->temp_outer] = 0U;
        s_temp_pid_update_pending[cfg->temp_inner] = 0U;

        if (cfg->drv_outer < APP_CONTROL_CLOSED_LOOP_COUNT &&
            cfg->drv_inner < APP_CONTROL_CLOSED_LOOP_COUNT)
        {
            uint32_t tick_now = osKernelGetTickCount();
            uint32_t dt = tick_now - s_pid_freq_last_tick[cfg->drv_outer];
            if (dt > 0U && s_pid_freq_last_tick[cfg->drv_outer] != 0U)
            {
                float freq = 1000.0f / (float)dt;
                g_app_control_pid_freq_hz[cfg->drv_outer] = freq;
                g_app_control_pid_freq_hz[cfg->drv_inner] = freq;
            }
            s_pid_freq_last_tick[cfg->drv_outer] = tick_now;
            s_pid_freq_last_tick[cfg->drv_inner] = tick_now;
        }

        if (s_cell_last_pid_ms[cell] != 0U)
        {
            float dt = (float)(osKernelGetTickCount() - s_cell_last_pid_ms[cell]) * 0.001f;
            s_cell_pid[cell].dt = AppControl_Clamp(dt,
                                                   APP_CONTROL_TEMP_PID_DT_MIN_S,
                                                   APP_CONTROL_TEMP_PID_DT_MAX_S);
        }
        else
        {
            s_cell_pid[cell].dt = APP_CONTROL_TEMP_PID_DT_DEFAULT_S;
        }
        s_cell_last_pid_ms[cell] = osKernelGetTickCount();

        PID_SetTarget(&s_cell_pid[cell], s_cell[cell].target_temp);

        {
            float ff_duty = AppControl_FeedforwardDuty(cell, s_cell[cell].target_temp);
            float pid_duty = 0.0f;

            if (AppControl_NormalPidFeedbackReady(cell) != 0U)
            {
                pid_duty = -PID_Compute(&s_cell_pid[cell], s_cell[cell].current_temp);
            }
            else
            {
                PID_Reset(&s_cell_pid[cell]);
            }

            outer_duty = AppControl_Clamp(ff_duty + pid_duty,
                                          -APP_CONTROL_MAX_ABS_DUTY,
                                          APP_CONTROL_MAX_ABS_DUTY);
        }

        if (AppControl_Abs(outer_duty) < 0.001f)
            outer_duty = 0.0f;

        outer_duty = AppControl_ApplyCellDutySlew(cell,
                                                  outer_duty,
                                                  s_cell_pid[cell].dt);
        if (AppControl_Abs(outer_duty) < 0.001f)
            outer_duty = 0.0f;

        if (AppControl_SetCellStackDuty(cell, outer_duty) != DRV8703_OK)
        {
            AppControl_SetCellError(cell, AppControl_CellDrvError(cell));
            AppControl_StopCell(cell, APP_CONTROL_STOP_EMERGENCY);
            continue;
        }
    }

    if (any_running != 0U)
    {
        float shared_target = (AppControl_ActiveCellNeedsSharedDrv() != 0U)
                                  ? AppControl_SharedTargetDuty()
                                  : 0.0f;
        float shared_duty = AppControl_ApplyDutySlew(APP_CONTROL_SHARED_DRV,
                                                     shared_target,
                                                     APP_CONTROL_TEMP_PID_DT_MIN_S);
        if (AppControl_SetDrvDuty(APP_CONTROL_SHARED_DRV, shared_duty) != DRV8703_OK)
        {
            AppControl_SetCellError(0U, PANEL_ERR_E315_SHARED_DRV);
            AppControl_SetCellError(1U, PANEL_ERR_E315_SHARED_DRV);
            AppControl_StopCell(0U, APP_CONTROL_STOP_EMERGENCY);
            AppControl_StopCell(1U, APP_CONTROL_STOP_EMERGENCY);
        }
    }
}
/* 水位检�?-----------------------------------------------------------------*/

/**
 * @brief  每秒检测水位传感器并在串口输出状�?
 * @param  now_ms 当前系统 tick (ms)
 */
static void AppControl_WaterCheck(uint32_t now_ms)
{
    if ((now_ms - s_last_water_check_ms) < 1000U)
        return;
    s_last_water_check_ms = now_ms;
    HAL_GPIO_TogglePin(WATER_GPIO_Port, WATER_Pin);

    // #ifdef UART_LOG_ENABLE
    //     if (HAL_GPIO_ReadPin(WATER_GPIO_Port, WATER_Pin) == GPIO_PIN_SET)
    //     {
    //         (void)HAL_UART_Transmit(&huart2, (uint8_t *)"WATER,OK\r\n", 10, 10);
    //     }
    //     else
    //     {
    //         (void)HAL_UART_Transmit(&huart2, (uint8_t *)"WATER,FAULT\r\n", 13, 10);
    //     }
    // #else
    //     (void)now_ms;
    // #endif
}

/**
 * @brief Initialise the application control module.
 *
 * Creates the command queue, initialises PID controllers, resets all
 * debug variables to their default values, and captures the initial
 * tick offset for freshness detection.
 *
 * @return APP_CONTROL_OK on success, APP_CONTROL_ERROR_QUEUE if the
 *         message queue could not be created.
 */
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
        s_cell[i].stopping = 0U;
        s_cell[i].pid_update_pending = 0U;
        s_cell[i].sensor_reset_count = 0U;
        s_cell[i].target_temp = -10.0f;
        s_cell[i].current_temp = 25.0f;
        s_cell[i].duty = 0.0f;
        s_cell[i].error = PANEL_ERR_NONE;
        s_cell[i].error_set_ms = 0U;
        s_cell_pid_update_pending[i] = 0U;
        s_cell_base_output_duty[i] = 0.0f;
        s_cell_last_pid_ms[i] = 0U;
        s_cell_pid_feedback_enabled[i] = 0U;
        s_cell_pid_feedback_start_ms[i] = 0U;
        g_app_control_cell_outer_duty[i] = 0.0f;
        g_app_control_cell_inner_duty[i] = 0.0f;
        g_app_control_cell_inner_ratio[i] = AppControl_CellInnerRatio(i);
        PID_Init(&s_cell_pid[i],
                 APP_CONTROL_TEMP_PID_KP,
                 APP_CONTROL_TEMP_PID_KI,
                 APP_CONTROL_TEMP_PID_KD,
                 APP_CONTROL_TEMP_PID_DT_S);
        PID_SetLimits(&s_cell_pid[i],
                      -APP_CONTROL_MAX_ABS_DUTY,
                      APP_CONTROL_MAX_ABS_DUTY,
                      -APP_CONTROL_MAX_ABS_DUTY,
                      APP_CONTROL_MAX_ABS_DUTY);
    }

    for (i = 0U; i < APP_CONTROL_CLOSED_LOOP_COUNT; i++)
    {
        s_temp_pid_update_pending[i] = 0U;
        s_temp_channel_temp[i] = 25.0f;
        s_temp_channel_duty[i] = 0.0f;
        s_temp_last_pid_count[i] = 0U;
    }

    for (i = 0U; i < APP_CONTROL_DRV_COUNT; i++)
    {
        uint8_t reg;

        s_drv_output_duty[i] = 0.0f;
        g_app_control_drv_fault_snapshot_valid[i] = 0U;
        g_app_control_drv_fault_capture_count[i] = 0U;
        g_app_control_drv_fault_read_status[i] = DRV8703_OK;
        g_app_control_drv_dump_status[i] = DRV8703_OK;
        g_app_control_drv_awake[i] = 0U;
        g_app_control_drv_fault_status[i] = 0xFFU;
        g_app_control_drv_vds_gdf_status[i] = 0xFFU;
        g_app_control_drv_reg_read_ok_mask[i] = 0U;
        for (reg = 0U; reg < DRV8703_REGISTER_COUNT; reg++)
        {
            g_app_control_drv_reg_dump[i][reg] = 0xFFU;
            g_app_control_drv_reg_read_status[i][reg] = DRV8703_OK;
        }

        g_app_control_drv_startup_dump_valid[i] = 0U;
        g_app_control_drv_startup_dump_status[i] = DRV8703_OK;
        for (reg = 0U; reg < DRV8703_REGISTER_COUNT; reg++)
        {
            g_app_control_drv_startup_reg_dump[i][reg] = 0xFFU;
            g_app_control_drv_startup_reg_status[i][reg] = DRV8703_OK;
            g_app_control_drv_startup_tx[i][reg] = 0U;
            g_app_control_drv_startup_rx[i][reg] = 0U;
        }

        g_app_control_drv_pin_fault_count[i] = 0U;
        g_app_control_drv_pin_fault_status[i] = DRV8703_OK;
        g_app_control_drv_pin_fault_read_status[i] = DRV8703_OK;
        g_app_control_drv_pin_fault_dump_status[i] = DRV8703_OK;
        g_app_control_drv_pin_fault_fault_status[i] = 0xFFU;
        g_app_control_drv_pin_fault_vds_gdf_status[i] = 0xFFU;
        g_app_control_drv_pin_fault_stop_bits[i] = 0U;
        g_app_control_drv_pin_fault_reg_read_ok_mask[i] = 0U;
        g_app_control_drv_pin_fault_all_ff[i] = 0U;
        g_app_control_drv_pin_fault_all_ff_count[i] = 0U;
        g_app_control_drv_pin_fault_stable_count[i] = 0U;
        for (reg = 0U; reg < DRV8703_REGISTER_COUNT; reg++)
        {
            g_app_control_drv_pin_fault_reg_dump[i][reg] = 0xFFU;
            g_app_control_drv_pin_fault_reg_read_status[i][reg] = DRV8703_OK;
            g_app_control_sleep_reg_snapshot[i][reg] = 0xFFU;
            g_app_control_periodic_reg_snapshot[i][reg] = 0xFFU;
        }
        g_app_control_sleep_reg_snapshot[i][DRV8703_REGISTER_COUNT] = 0U;
        g_app_control_periodic_reg_snapshot[i][DRV8703_REGISTER_COUNT] = 0U;
    }

    s_last_reg_snapshot_ms = 0U;
    g_app_control_periodic_reg_snapshot_count = 0U;

    /* 加载两个 Cell 的历史标定数据用于前馈控�?*/
    AppControl_LoadCalibData(0);
    AppControl_LoadCalibData(1);

    AppControl_ApplyDebugState();
    g_app_control_init_result = APP_CONTROL_OK;
    return APP_CONTROL_OK;
}

/**
 * @brief Main control task entry point, called every control cycle from
 *        the FreeRTOS ControlTask thread.
 *
 * Normal mode: service sensor reset timer, process start/stop commands,
 * update temperature inputs, check voltage and DRV8703 faults, run the
 * closed-loop PID controller.
 *
 * Test mode (g_app_control_test_active != 0): initialise CH3 only,
 * ramp its duty cycle between ±APP_CONTROL_MAX_ABS_DUTY, poll fault
 * pins, and periodically snapshot DRV8703 registers via SPI.
 *
 * @param now_ms Current RTOS tick.
 */
void AppControl_Task(uint32_t now_ms)
{
    AppControl_Lock();

    if (g_app_control_test_active != 0U)
    {
        if (s_test_drv_initialized == 0U)
        {
            uint8_t drv;
            for (drv = 0U; drv < APP_CONTROL_DRV_COUNT; drv++)
            {
                DRV8703_Status_t ret;
                ret = DRV8703_BoardInitOne((DRV8703_BoardChannel_t)drv);
                if (ret == DRV8703_OK)
                    ret = DRV8703_BoardApplyDefaultConfig((DRV8703_BoardChannel_t)drv);
                if (ret == DRV8703_OK)
                {
                    DRV8703_Handle_t *dev = DRV8703_BoardGet((DRV8703_BoardChannel_t)drv);
                    if (dev != 0)
                    {
                        (void)DRV8703_SetVrefMv(dev, APP_CONTROL_DRV_VREF_MV);
                        (void)DRV8703_ClearFault(dev);
                        (void)DRV8703_Lock(dev);
                        AppControl_CaptureDrvStartupRegs(drv, dev);

                        if (drv != 4U)
                            (void)DRV8703_Sleep(dev);
                    }
                }
                g_app_control_test_drv_ok[drv] = (ret == DRV8703_OK) ? 1U : 0U;
                g_app_control_test_duty[drv] = 0.0f;
                g_app_control_test_phase = 0U;
                s_test_duty_toggle_ms[drv] = now_ms;

                g_app_control_drv_ready[drv] = (ret == DRV8703_OK) ? 1U : 0U;
                g_app_control_drv_awake[drv] = (ret == DRV8703_OK && drv == APP_CONTROL_SHARED_DRV) ? 1U : 0U;
                g_app_control_drv_fault[drv] = 0U;
            }
            s_test_drv_initialized = 1U;
            g_app_control_test_phase = 1U;
        }

        if (g_app_control_test_phase == 1U)
        {
            uint8_t drv;

            drv = APP_CONTROL_SHARED_DRV;
            if (g_app_control_test_drv_ok[drv] != 0U)
            {
                DRV8703_Handle_t *dev = DRV8703_BoardGet((DRV8703_BoardChannel_t)drv);
                if (dev != 0)
                {
                    if ((now_ms - s_test_duty_toggle_ms[drv]) >= 2000U)
                    {
                        s_test_duty_toggle_ms[drv] = now_ms;
                        float duty = g_app_control_test_duty[drv] + 0.05f * (float)s_test_ramp_dir[drv];
                        if (duty >= APP_CONTROL_MAX_ABS_DUTY)
                        {
                            duty = APP_CONTROL_MAX_ABS_DUTY;
                            s_test_ramp_dir[drv] = -1;
                        }
                        else if (duty <= -APP_CONTROL_MAX_ABS_DUTY)
                        {
                            duty = -APP_CONTROL_MAX_ABS_DUTY;
                            s_test_ramp_dir[drv] = 1;
                        }
                        g_app_control_test_duty[drv] = duty;
                        (void)DRV8703_SetDuty(dev, duty);
                    }
                }
            }

            AppControl_CheckDrvFaults(now_ms);
            AppControl_CapturePeriodicDrvRegs(now_ms);
            AppControl_ReadAllDrvRegsRaw(now_ms);
        }
    }
    else
    {
        if (g_calib_mode_active != 0U)
        {
            uint32_t calib_fault;

            s_calib_was_active = 1U;

            AppControl_CheckDrvFaults(now_ms);
            calib_fault = AppControl_GetCalibDrvFault();
            if (calib_fault != 0U)
            {
                CalibMode_Fault(calib_fault);
            }

            CalibMode_Task(now_ms);
        }

        if (g_calib_mode_active == 0U)
        {
            /* 标定刚刚完成 �?重新加载 Flash 标定数据用于前馈 */
            if (s_calib_was_active != 0U)
            {
                s_calib_was_active = 0U;
                AppControl_LoadCalibData(0);
                AppControl_LoadCalibData(1);
            }

            AppControl_ServiceTempSensorReset(now_ms);
            AppControl_ProcessCommands();
            AppControl_UpdateTemperatureInputs(now_ms);
            AppControl_CheckVoltageFaults();
            AppControl_CheckDrvFaults(now_ms);
            AppControl_ServiceErrorDisplayTimeout(now_ms);
            AppControl_RunClosedLoop();
        }
    }

    AppControl_WaterCheck(now_ms);
    AppControl_ApplyDebugState();
    Ads1220_Test(); /* 临时：ADS1220 测试，验证通过后删�?*/

    /* PC 协议：每 1s 发送一次过程数�?*/
    {
        static uint32_t s_last_data_ms = 0U;
        if ((now_ms - s_last_data_ms) >= 1000U)
        {
            s_last_data_ms = now_ms;
            PcProto_SendData(0);
            PcProto_SendData(1);
        }
    }

    g_app_control_loop_count++;
    AppControl_Unlock();
}

/**
 * @brief Update the panel UI with current cell temperatures and errors.
 *
 * Called from HMITask.  Acquires the system mutex only long enough to
 * copy the cell temperatures and errors.
 */
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
#if TEMP_VIRTUAL_LOW_ENABLE
        temp[cell] = g_app_control_cell_temp[cell];
#else
        temp[cell] = s_cell[cell].current_temp;
#endif
        err[cell] = s_cell[cell].error;
    }
    AppControl_Unlock();

    for (cell = 0U; cell < APP_CONTROL_CELL_COUNT; cell++)
    {
        TempPanel_UpdateMeasuredTemp(panel, cell, temp[cell], now_ms);
        TempPanel_SetCellError(panel, cell, err[cell]);
    }
}

/**
 * @brief Post a start or stop command to the control task's message queue.
 */
static void AppControl_PostCommand(AppControlCommandType_t type,
                                   uint8_t cell,
                                   AppControlStopMode_t stop_mode)
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
    cmd.stop_mode = stop_mode;
    if (osMessageQueuePut(s_cmd_queue, &cmd, 0U, 0U) != osOK)
        g_app_control_cmd_drop_count++;
}

/** @brief Public API: request PID start for a cell. */
void Control_StartPid(uint8_t cell)
{
    AppControl_PostCommand(APP_CONTROL_CMD_START, cell, APP_CONTROL_STOP_NORMAL);
}

/** @brief Public API: request PID stop for a cell. */
void Control_StopPid(uint8_t cell)
{
    AppControl_PostCommand(APP_CONTROL_CMD_STOP, cell, APP_CONTROL_STOP_NORMAL);
}

void Control_EmergencyStopPid(uint8_t cell)
{
    AppControl_PostCommand(APP_CONTROL_CMD_STOP, cell, APP_CONTROL_STOP_EMERGENCY);
}

void Control_EmergencyStopAll(void)
{
    Control_EmergencyStopPid(0U);
    Control_EmergencyStopPid(1U);
}

/** @brief Public API: set the target temperature for a cell. */
void Control_SetTargetTemp(uint8_t cell, float target)
{
    if (cell >= APP_CONTROL_CELL_COUNT)
        return;

    AppControl_Lock();
    s_cell[cell].target_temp = target;
    AppControl_Unlock();
}
