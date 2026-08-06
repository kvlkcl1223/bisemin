#include "app_control.h"

#include "adc_measure.h"
#include "calib_mode.h"
#include "cmsis_os.h"
#include "drv8703_board.h"
#include "pid_controller.h"
#include "ads1220.h"
#include "pc_protocol.h"

#include <string.h>

/** @brief 控制命令队列深度，面板/上位机发起的启停命令会先进入该队列。*/
#define APP_CONTROL_QUEUE_DEPTH 12U
/** @brief 每颗 DRV8703 初始化失败后的最大重试次数。*/
#define APP_CONTROL_DRV_RETRY_COUNT 3U
/** @brief 写入 DRV8703 的 VREF 参考电压，单位 mV。*/
#define APP_CONTROL_DRV_VREF_MV 3300U
/** @brief 输入供电电压最低允许值，低于该值时可触发电压故障。*/
#define APP_CONTROL_VSUPPLY_MIN_V 20.0f
/** @brief PID 初始化使用的默认采样周期，单位 s。*/
#define APP_CONTROL_TEMP_PID_DT_S 0.2f
/** @brief 温控 PID 比例系数。*/
#define APP_CONTROL_TEMP_PID_KP 0.1f
/** @brief 温控 PID 积分系数。*/
#define APP_CONTROL_TEMP_PID_KI 0.01f
/** @brief 温控 PID 微分系数。*/
#define APP_CONTROL_TEMP_PID_KD 0.0f
/** @brief DRV8703 故障轮询周期，单位 ms。*/
#define APP_CONTROL_DRV_FAULT_POLL_MS 500U
/** @brief DRV8703 周期寄存器快照间隔，单位 ms。*/
#define APP_CONTROL_DRV_REG_SNAPSHOT_MS 100U
/** @brief FAULT 引脚触发后读取寄存器的稳定重试次数。*/
#define APP_CONTROL_DRV_FAULT_READ_RETRY_COUNT 8U
/** @brief 判定 DRV 故障读数稳定所需的连续一致次数。*/
#define APP_CONTROL_DRV_FAULT_STABLE_READ_COUNT 2U
/** @brief 测温输入连续未更新多少个控制周期后判定为陈旧。*/
#define APP_CONTROL_TEMP_STALE_CYCLES 30U
/** @brief 两次测温传感器硬复位之间的最小冷却时间，单位 ms。*/
#define APP_CONTROL_TEMP_RESET_COOLDOWN_MS 1500U
/** @brief 测温传感器 NRST 复位脉冲宽度，单位 ms。*/
#define APP_CONTROL_TEMP_RESET_PULSE_MS 20U
/** @brief 单个 cell 因测温陈旧允许尝试复位传感器的最大次数。*/
#define APP_CONTROL_TEMP_RESET_MAX_COUNT 3U
/** @brief duty 斜率限制计算使用的默认周期，单位 s。*/
#define APP_CONTROL_TEMP_PID_DT_DEFAULT_S 0.05f
/** @brief duty 斜率限制接受的最小周期，单位 s。*/
#define APP_CONTROL_TEMP_PID_DT_MIN_S 0.02f
/** @brief duty 斜率限制接受的最大周期，单位 s。*/
#define APP_CONTROL_TEMP_PID_DT_MAX_S 1.0f
/** @brief duty 输出变化斜率上限，单位 duty/s，用于避免输出突变。*/
#define APP_CONTROL_DUTY_SLEW_RATE_PER_S 1.0f
/** @brief 故障清除后面板错误显示保留时间，单位 ms。*/
#define APP_CONTROL_ERROR_DISPLAY_MS 5000U
/** @brief 普通模式下是否延迟启用 PID 反馈，先以前馈接近目标温度。*/
#define APP_CONTROL_NORMAL_PID_DELAY_ENABLE 1U
/** @brief 普通模式进入 PID 反馈的温差阈值，单位 degC。*/
#define APP_CONTROL_NORMAL_PID_ENTER_BAND_C 2.0f
/** @brief 普通模式等待进入 PID 反馈的最长时间，单位 ms。*/
#define APP_CONTROL_NORMAL_PID_ENTER_TIMEOUT_MS 10000U
/** @brief DRV 单通道/多通道测试时的 FAULT 引脚轮询周期，单位 ms。 */
#define APP_CONTROL_DRV_TEST_FAULT_POLL_MS 50U
/** @brief DRV 测试模式默认测试通道：默认测试共享 DRV5。 */
#define APP_CONTROL_DRV_TEST_DEFAULT_MASK ((uint8_t)(1U << APP_CONTROL_SHARED_DRV))
/** @brief DRV 测试模式默认 duty，用于调试器只置位 g_app_control_test_active 的场景。 */
#define APP_CONTROL_DRV_TEST_DEFAULT_DUTY APP_CONTROL_SHARED_CH5_DUTY
/** @brief 测试模式发现某一路故障后是否关闭全部测试通道；0 表示只关故障通道。 */
#define APP_CONTROL_DRV_TEST_STOP_ALL_ON_FAULT 0U
/** @brief 所有 DRV 通道组成的位掩码，bit0..bit4 对应 DRV1..DRV5。 */
#define APP_CONTROL_DRV_ALL_MASK ((uint8_t)((1U << APP_CONTROL_DRV_COUNT) - 1U))

/*
 * DRV8703 FAULT bit7 is only a summary flag. Stop the cell only for faults
 * that can damage hardware or make output unsafe. Watchdog timeout and over
 * temperature warning are recorded for debug, but they do not latch a panel
 * error or stop the temperature cell by themselves.
 */
/** @brief 需要立即停机处理的 DRV8703 故障位掩码，排除仅用于记录的告警位。*/
#define APP_CONTROL_DRV_STOP_FAULT_MASK \
    (DRV8703_FAULT_GDF |                \
     DRV8703_FAULT_OCP |                \
     DRV8703_FAULT_OTSD)

/** @brief 单个温控池运行时状态。*/
typedef struct
{
    uint8_t running;            /**< 当前 cell 是否正在闭环/标定输出。*/
    uint8_t requested;          /**< 是否收到过启动请求，供状态同步和调试使用。*/
    uint8_t stopping;           /**< 是否处于正常停止的 duty 缓降阶段。*/
    uint8_t pid_update_pending; /**< 两路测温有新数据后置位，触发下一次 PID 计算。*/
    uint8_t sensor_reset_count; /**< 因测温陈旧已尝试复位传感器的次数。*/
    float target_temp;          /**< 目标温度，单位 degC。*/
    float current_temp;         /**< 当前反馈温度，即外层/内层两路测温均值。*/
    float duty;                 /**< 当前 cell 主路/外层 duty，内层按比例跟随。*/
    PanelError_t error;         /**< 当前 cell 错误码。*/
    uint32_t error_set_ms;      /**< 错误码最近一次设置时间，用于显示保持。*/
} AppControlCell_t;

/** @brief 单个 cell 的硬件通道配置，由 app_control.h 的宏生成。*/
typedef struct
{
    uint8_t temp_outer;  /**< 外层测温输入索引，0..3 对应 CH1..CH4。*/
    uint8_t temp_inner;  /**< 内层测温输入索引，0..3 对应 CH1..CH4。*/
    uint8_t drv_outer;   /**< 外层/主路 DRV 索引，0..4 对应 DRV1..DRV5。*/
    uint8_t drv_inner;   /**< 内层/从路 DRV 索引，输出 duty = 外层 duty * inner_ratio。*/
    float inner_ratio;   /**< 内层 duty 跟随比例。*/
    uint8_t shared_drv;  /**< 该 cell 运行时需要同时打开的共享 DRV，0xFF 表示不用。*/
    float shared_duty;   /**< 共享 DRV 运行时固定 duty。*/
} AppControlCellConfig_t;

/** @brief 两个 cell 的通道映射表；后续换线优先修改 app_control.h 的宏。*/
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

/* 公开调试变量：模块状态、cell 状态、DRV 故障快照和频率统计。*/
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
/** @brief DRV 测试请求掩码，bit0..bit4 对应 DRV1..DRV5。 */
volatile uint8_t g_app_control_drv_test_requested_mask = 0U;
/** @brief DRV 测试当前仍在输出的通道掩码；故障通道会被自动清除。 */
volatile uint8_t g_app_control_drv_test_active_mask = 0U;
/** @brief DRV 测试期间已经检测到故障的通道掩码。 */
volatile uint8_t g_app_control_drv_test_fault_mask = 0U;
/** @brief DRV 测试模式 FAULT 轮询累计次数。 */
volatile uint32_t g_app_control_drv_test_fault_poll_count = 0U;
/** @brief DRV 测试期间每个通道最近一次状态，OK 表示最近未检测到故障。 */
volatile DRV8703_Status_t g_app_control_drv_test_status[APP_CONTROL_DRV_COUNT] =
    {
        DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK, DRV8703_OK};

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

/* 内部运行状态：命令队列、cell/PID 状态、输出斜率限制和测温频率统计。*/
static osMessageQueueId_t s_cmd_queue; /**< 控制命令队列句柄，接收面板/上位机启停请求。*/
static AppControlCell_t s_cell[APP_CONTROL_CELL_COUNT]; /**< 两个 cell 的内部运行状态。*/
static PID_TypeDef s_cell_pid[APP_CONTROL_CELL_COUNT]; /**< 每个 cell 一个 PID，反馈量为两路测温均值。*/
static uint32_t s_last_fault_poll_ms = 0U; /**< 上一次 DRV 故障轮询时间。*/
static uint32_t s_last_reg_snapshot_ms = 0U; /**< 上一次周期寄存器快照时间。*/
static uint8_t s_temp_pid_update_pending[APP_CONTROL_CLOSED_LOOP_COUNT] = {0}; /**< 按闭环 DRV 通道展开的 PID 更新标志。*/
static uint8_t s_cell_pid_update_pending[APP_CONTROL_CELL_COUNT] = {0U, 0U}; /**< 按 cell 记录的 PID 更新标志。*/
static float s_temp_channel_temp[APP_CONTROL_CLOSED_LOOP_COUNT] = {25.0f, 25.0f, 25.0f, 25.0f}; /**< 每个闭环 DRV 通道对应的测温显示值。*/
static float s_temp_channel_duty[APP_CONTROL_CLOSED_LOOP_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f}; /**< 每个闭环 DRV 通道最近 duty。*/
static float s_cell_base_output_duty[APP_CONTROL_CELL_COUNT] = {0.0f, 0.0f}; /**< cell 外层/主路 duty 的斜率限制状态。*/
static float s_drv_output_duty[APP_CONTROL_DRV_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; /**< 每个 DRV 最近实际输出 duty。*/
static uint8_t s_cell_pid_feedback_enabled[APP_CONTROL_CELL_COUNT] = {0U, 0U}; /**< 普通模式 PID 反馈是否已启用。*/
static uint32_t s_cell_pid_feedback_start_ms[APP_CONTROL_CELL_COUNT] = {0U, 0U}; /**< 普通模式等待 PID 反馈启用的起始时间。*/
static uint32_t s_temp_last_pid_count[APP_CONTROL_CLOSED_LOOP_COUNT] = {0U, 0U, 0U, 0U}; /**< 上一次触发 PID 时各测温通道的更新计数。*/
static uint32_t s_cell_last_pid_ms[APP_CONTROL_CELL_COUNT] = {0U, 0U}; /**< 每个 cell 上一个 PID 计算时间。*/
static uint32_t s_temp_freq_last_count[APP_CONTROL_TEMP_INPUT_COUNT] = {0U, 0U, 0U, 0U}; /**< 测温频率统计的上一次计数。*/
static uint32_t s_temp_freq_last_tick[APP_CONTROL_TEMP_INPUT_COUNT] = {0U, 0U, 0U, 0U}; /**< 测温频率统计的上一次时间。*/
static uint32_t s_pid_freq_last_tick[APP_CONTROL_CLOSED_LOOP_COUNT] = {0U, 0U, 0U, 0U}; /**< PID 频率统计的上一次时间。*/

/* 低温虚拟显示：真实传感器到达下限后，用估算值继续给面板显示。*/
#if TEMP_VIRTUAL_LOW_ENABLE
/** @brief 低温虚拟显示状态，仅在 TEMP_VIRTUAL_LOW_ENABLE 打开时编译。*/
typedef struct
{
    uint8_t active;          /**< 是否已经进入虚拟显示区间。*/
    uint8_t started;         /**< 虚拟显示估算是否已完成首次初始化。*/
    float last_real_temp;    /**< 上一次真实测温值。*/
    float learned_rate_cps;  /**< 根据真实温度变化学习到的降温速率。*/
    float virtual_temp;      /**< 当前虚拟显示温度。*/
    float noise;             /**< 当前叠加噪声值。*/
    uint32_t last_ms;        /**< 上一次虚拟温度更新时间。*/
    uint32_t rng;            /**< 简单伪随机数状态，用于噪声。*/
} AppControlVirtualLowTemp_t;

/** @brief 进入低温虚拟显示的真实温度阈值。*/
#define APP_CONTROL_VIRTUAL_ENTER_TEMP_C (PANEL_TEMP_REAL_MIN + 0.5f)
/** @brief 低温虚拟显示默认降温速率，单位 degC/s。*/
#define APP_CONTROL_VIRTUAL_DEFAULT_RATE_CPS 0.03f
/** @brief 低温虚拟显示学习速率下限，单位 degC/s。*/
#define APP_CONTROL_VIRTUAL_RATE_MIN_CPS 0.005f
/** @brief 低温虚拟显示学习速率上限，单位 degC/s。*/
#define APP_CONTROL_VIRTUAL_RATE_MAX_CPS 0.20f
/** @brief 低温虚拟显示叠加噪声最大幅度，单位 degC。*/
#define APP_CONTROL_VIRTUAL_NOISE_MAX_C 0.12f

/** @brief 每个 cell 的低温虚拟显示状态。*/
static AppControlVirtualLowTemp_t s_virtual_low[APP_CONTROL_CELL_COUNT] =
{
    {0U, 0U, 25.0f, APP_CONTROL_VIRTUAL_DEFAULT_RATE_CPS, 25.0f, 0.0f, 0U, 0x13572468UL},
    {0U, 0U, 25.0f, APP_CONTROL_VIRTUAL_DEFAULT_RATE_CPS, 25.0f, 0.0f, 0U, 0x24681357UL}
};

/** @brief 每个 cell 给面板/调试变量使用的显示温度。*/
static float s_cell_display_temp[APP_CONTROL_CELL_COUNT] = {25.0f, 25.0f};
/** @brief 每个闭环 DRV 通道给调试变量使用的显示温度。*/
static float s_temp_channel_display_temp[APP_CONTROL_CLOSED_LOOP_COUNT] = {25.0f, 25.0f, 25.0f, 25.0f};
#endif
/* 测温传感器复位、手动测试、标定切换和水位检测相关内部状态。*/
static uint32_t s_temp_reset_release_ms = 0U; /**< 测温传感器复位释放时间。*/
static uint32_t s_temp_last_reset_ms = 0U; /**< 上一次测温传感器复位请求时间。*/
static uint32_t s_last_raw_poll_ms = 0U; /**< 上一次 DRV 原始寄存器轮询时间。*/

static uint32_t s_temp_last_count_for_stale[APP_CONTROL_TEMP_INPUT_COUNT] = {0U, 0U, 0U, 0U}; /**< 测温陈旧检测用的上一次更新计数。*/
uint8_t s_temp_stale_cycles[APP_CONTROL_TEMP_INPUT_COUNT] = {0U, 0U, 0U, 0U}; /**< 每路测温连续陈旧周期计数，保留非 static 便于调试观察。*/

static uint8_t s_test_drv_initialized = 0U; /**< DRV 测试模式是否已经完成本轮通道初始化。 */
static uint32_t s_drv_test_last_fault_poll_ms = 0U; /**< DRV 测试模式上一次 FAULT 轮询时间。 */
static uint8_t s_calib_was_active = 0U; /**< 记录上一周期是否处于标定模式，用于标定结束后重新加载 Flash。*/
static uint32_t s_last_water_check_ms = 0U; /**< 水位检测心跳上一次执行时间。*/

/* 标定数据缓存，用于前馈控制 ------------------------------------------------*/
/** @brief Flash 标定数据缓存，前馈控制按目标温度查表得到基础 duty。*/
typedef struct
{
    uint8_t loaded;                    /**< 标定数据是否已成功从 Flash 加载。*/
    float duty[CALIB_DUTY_COUNT];      /**< 标定表中的外层/主路 duty 序列。*/
    float temp_ch0[CALIB_DUTY_COUNT];  /**< 标定时第一路测温稳定温度。*/
    float temp_ch1[CALIB_DUTY_COUNT];  /**< 标定时第二路测温稳定温度。*/
    float temp_avg[CALIB_DUTY_COUNT];  /**< 两路测温均值，前馈查表实际使用该曲线。*/
} CalibCache_t;

static CalibCache_t s_calib_cache[APP_CONTROL_CELL_COUNT]; /**< 两个 cell 的前馈标定缓存。*/

/* 外部模块提供的共享资源：系统互斥锁、测温串口状态、UART 句柄和接收缓冲区。*/
extern osMutexId_t SysStateMutexHandle;
extern volatile float Sys_Temperatures[4];
extern volatile uint8_t Sys_Status[4];
extern volatile uint32_t Sys_TempUpdateCount[4];
extern volatile uint32_t Sys_TempUpdateTick[4];

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/** @brief USART1 测温串口 DMA 接收缓冲区大小。*/
#define RX_BUFFER_SIZE 256U
extern uint8_t rx_buffer[RX_BUFFER_SIZE];

/** @brief Acquire the global system state mutex. */
/** @brief 获取系统状态互斥锁，保。app_control 与面。串口共享状态。*/
static void AppControl_Lock(void)
{
    (void)osMutexAcquire(SysStateMutexHandle, osWaitForever);
}

/** @brief Release the global system state mutex. */
/** @brief 释放系统状态互斥锁。*/
static void AppControl_Unlock(void)
{
    (void)osMutexRelease(SysStateMutexHandle);
}

/** @brief Absolute value of a float. */
/** @brief 计算 float 绝对值。*/
static float AppControl_Abs(float v)
{
    return (v < 0.0f) ? -v : v;
}

/** @brief Clamp a float value between min_v and max_v. */
/** @brief 将数值限制在指定上下限之间。*/
static float AppControl_Clamp(float v, float min_v, float max_v)
{
    if (v < min_v)
        return min_v;
    if (v > max_v)
        return max_v;
    return v;
}

/** @brief 对单位 DRV duty 做斜率限制，避免 PWM 输出突变。*/
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

/** @brief 对 cell 主路/外层 duty 做斜率限制。*/
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

/** @brief 获取指定 cell 的硬件通道配置。*/
static const AppControlCellConfig_t *AppControl_CellConfig(uint8_t cell)
{
    if (cell >= APP_CONTROL_CELL_COUNT)
        return 0;
    return &s_cell_config[cell];
}

/** @brief 获取指定 cell 的内。duty 跟随比例。*/
static float AppControl_CellInnerRatio(uint8_t cell)
{
    const AppControlCellConfig_t *cfg = AppControl_CellConfig(cell);

    if (cfg == 0)
        return 0.0f;
    return cfg->inner_ratio;
}

/** @brief 记录指定 DRV 当前 duty 到内部和调试数组。*/
static void AppControl_RecordDrvDuty(uint8_t drv, float duty)
{
    if (drv >= APP_CONTROL_DRV_COUNT)
        return;

    s_drv_output_duty[drv] = duty;
    if (drv < APP_CONTROL_CLOSED_LOOP_COUNT)
        s_temp_channel_duty[drv] = duty;
}

/** @brief 根据外层 duty 计算并记录 cell 的外层/内层 duty。*/
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

/** @brief 判断某个 DRV 是否属于指定 cell 的外层或内层通道。*/
static uint8_t AppControl_CellOwnsDrv(uint8_t cell, uint8_t drv)
{
    const AppControlCellConfig_t *cfg = AppControl_CellConfig(cell);

    if (cfg == 0)
        return 0U;
    return (cfg->drv_outer == drv || cfg->drv_inner == drv) ? 1U : 0U;
}

/** @brief 根据 DRV 索引查找所属 cell，找不到返回 APP_CONTROL_INVALID_INDEX。*/
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

/** @brief 判断某个 DRV 是否为共享通道。*/
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

/** @brief 计算共享 DRV 当前目标 duty；任意 cell 运行时返回固定共享 duty。*/
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
/** @brief 维护测温传感器复位脉冲，到时释放复位并重。UART DMA 接收。*/
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
 * Pulls the NRST pin low to reset the sensor. Does not touch the UART DMA;
 * the idle-line ISR handles DMA restart safely on its own.
 *
 * @param now_ms Current RTOS tick.
 * @return 1 if a reset was started, 0 if one is already in progress or the
 *         cooldown period has not elapsed.
 */
/** @brief 请求启动一次测温传感器硬复位，带冷却时间保护。*/
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
/** @brief 按 cell 编号转换为对应的 DRV 故障面板错误码。*/
static PanelError_t AppControl_CellDrvError(uint8_t cell)
{
    return (cell == 0U) ? PANEL_ERR_E311_CELL1_DRV : PANEL_ERR_E312_CELL2_DRV;
}

/** @brief Panel error code for an input-voltage fault on a given cell. */
/** @brief 按 cell 编号转换为对应的输入电压故障面板错误码。*/
static PanelError_t AppControl_CellVoltageError(uint8_t cell)
{
    return (cell == 0U) ? PANEL_ERR_E301_CELL1_VOLTAGE : PANEL_ERR_E302_CELL2_VOLTAGE;
}

/**
 * @brief Read all DRV8703 registers after initialisation and store them as
 *        the expected "golden" values for that chip.
 */
/** @brief 初始化后读取 DRV8703 全部寄存器，作为启动快照和期望值参考。*/
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
/** @brief 读取指定 DRV8703 全部寄存器到全局调试数组。*/
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
/** @brief 读取指定 DRV8703 全部寄存器到本地缓冲区。*/
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
/** @brief 将本地寄存器快照提交到指定的全局调试数组。*/
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
/** @brief 获取指定 DRV 某个寄存器的期望值，用于判断异常读数。*/
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
 * register is 全 0xFF, any configuration register differs from the expected
 * value, or a GDF/OCP fault is reported without any VDS/GDF detail bits.
 *
 * @return 1 if invalid, 0 if usable.
 */
/** @brief 判断一次 DRV 寄存器采样是否无效，例如全 0xFF 或关键寄存器异常。*/
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
/** @brief 比较两次 DRV 寄存器采样是否完全一致。*/
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
/** @brief 带稳定重试地读取 DRV 寄存器，过滤偶发 SPI 异常。*/
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
/** @brief 捕获一次 DRV 故障，更新故障标志和寄存器快照。*/
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
/** @brief 判断 DRV8703 故障位是否需要停止 cell 以保护硬件。*/
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
/** @brief FAULT 引脚触发瞬间捕获寄存器，并判断是否需要停机。*/
static uint8_t AppControl_CaptureDrvPinFaultMoment(uint8_t drv, DRV8703_Status_t pin_status, uint8_t stop_all_pwm)
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

    if (stop_all_pwm != 0U)
    {
        uint8_t i;
        for (i = 0U; i < APP_CONTROL_DRV_COUNT; i++)
        {
            DRV8703_Handle_t *d = DRV8703_BoardGet((DRV8703_BoardChannel_t)i);
            if (d != 0)
                (void)DRV8703_SetDuty(d, 0.0f);
            AppControl_RecordDrvDuty(i, 0.0f);
        }
    }
    else
    {
        (void)DRV8703_SetDuty(dev, 0.0f);
        AppControl_RecordDrvDuty(drv, 0.0f);
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
/** @brief 判断面板错误码是否属于测温传感器错误。*/
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
/** @brief 将测温输入索引转换为对应的面板温度错误码。*/
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
/** @brief 判断某路测温输入在当前周期内是否仍然新鲜可用。*/
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
/** @brief 设置指定 cell 错误码，并记录错误发生时间和相关调试状态。*/
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

    /* PC 协议：故障时通知上位机。*/
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
/** @brief 故障消失后维持一段显示时间，再自动清除面板错误。*/
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
/** @brief 低温虚拟显示用的数值限幅。*/
static float AppControl_VirtualClamp(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

/** @brief 低温虚拟显示用的轻微噪声生成。*/
static float AppControl_VirtualNoise(AppControlVirtualLowTemp_t *state)
{
    float rnd;

    state->rng = state->rng * 1664525UL + 1013904223UL;
    rnd = (float)((state->rng >> 16) & 0x7FFFU) / 16383.5f - 1.0f;
    state->noise = state->noise * 0.92f + rnd * APP_CONTROL_VIRTUAL_NOISE_MAX_C * 0.08f;
    return state->noise;
}

/** @brief 重置指定 cell 的低温虚拟显示状态。*/
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

/** @brief 根据真实温度和运行状态计算低温区域的虚拟显示温度。*/
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

/** @brief 更新 cell 和测温通道用于显示/调试的温度值。*/
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
/** @brief 将内部状态同步到公开 volatile 调试变量。*/
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
/** @brief 初始化并唤醒指定 DRV，应用默认配置并捕获启动寄存器。*/
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
/** @brief 设置单颗 DRV8703 输出 duty，内部负责唤醒、限幅和故障捕获。*/
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

/** @brief 设置一个 cell 的堆叠输出：外层使用 outer_duty，内层按比例跟随。*/
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
/** @brief DRV 进入 sleep 前捕获寄存器，便于排查停机前状态。*/
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
/** @brief 周期性捕获已就绪且唤醒的 DRV 寄存器快照。*/
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
/** @brief 让指定 DRV 进入 sleep，并记录 sleep 前寄存器快照。*/
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
/** @brief 判断当前是否仍有 cell 需要共享 DRV 保持输出。*/
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

/** @brief 判断运行中的 cell 是否需要共享 DRV 输出非零 duty。*/
static uint8_t AppControl_ActiveCellNeedsSharedDrv(void)
{
    return (AppControl_SharedTargetDuty() != 0.0f) ? 1U : 0U;
}

/**
 * @brief Stop a temperature cell: zero duty, sleep its DRV8703 channels,
 *        sleep the shared channel if no other cell needs it.
 */
/** @brief 停止指定 cell；正常停止走缓降，紧急停止直接关闭输出。*/
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

/** @brief 处理正常停止中的 duty 缓降过程。*/
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
/** @brief 启动前检查指定 cell 当前是否没有硬件/传感器错误。*/
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
/** @brief 启动指定 cell，准备外层/内层/共享 DRV 并初始化 PID 状态。*/
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
/** @brief 从命令队列取出启。停止命令并执行。*/
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
/** @brief 读取四路测温输入，更新 cell 均温，并判断测温陈旧/复位/故障。*/
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
/** @brief 检查输入电压故障；当前函数保留接口，便于后续恢复电压保护。*/
static void AppControl_CheckVoltageFaults(void)
{
}

/**
 * @brief Unconditionally read all 6 registers of all 5 DRV8703 chips for
 *        debug purposes.  Polls every chip regardless of ready/awake state.
 *        Throttled to APP_CONTROL_DRV_RAW_POLL_MS.
 */
/** @brief 周期轮询全部 DRV8703 原始寄存器，不依赖运。故障状态。*/
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
/** @brief 周期检查 DRV8703 FAULT 引脚，必要时停止对应 cell 或全部 cell。*/
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
            if (AppControl_CaptureDrvPinFaultMoment(drv, pin_ret, 1U) == 0U)
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
/** @brief 标定模式下检查当前标定 cell 的外层/内层/共享 DRV 故障。*/
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
/* 标定数据加载与查。--------------------------------------------------------*/

/**
 * @brief  。Flash 加载指定 Cell 的标定数据到 RAM 缓存
 * @param  cell Cell 缂栧彿 (0 鎴?1)
 * @note   应在标定模式结束后、正常温控启动前调用
 *         加载失败时设。loaded=0，后续前馈退化为 0
 */
/** @brief 。Flash 读取指定 cell 标定表，并生成前馈查表缓存。*/
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
 * @return float 前馈占空比。 * @note   在标定点之间线性插值，超出范围取最近端点。 *         标定未加载时返回 0.0f
 */
/** @brief 根据目标温度查询标定均温曲线，线性插值得到前。duty。*/
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
/** @brief 普通模式下判断是否已经可以启用 PID 反馈修正。*/
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
/** @brief 主闭环控制：按 cell 两路测温均值计算外层 duty，内层按比例跟随。*/
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
/* 水位检测 -----------------------------------------------------------------*/

/**
 * @brief  每秒检测水位传感器并在串口输出状态。 * @param  now_ms 当前系统 tick (ms)
 */
/** @brief 水位检测调试心跳处理。*/
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

/** @brief 关闭指定 DRV 的测试输出，并同步调试 duty 变量。 */
static void AppControl_StopDrvTestOutput(uint8_t drv)
{
    DRV8703_Handle_t *dev;

    if (drv >= APP_CONTROL_DRV_COUNT)
        return;

    if (g_app_control_simulate_drv8703 == 0U)
    {
        dev = DRV8703_BoardGet((DRV8703_BoardChannel_t)drv);
        if (dev != 0)
            (void)DRV8703_SetDuty(dev, 0.0f);
    }

    g_app_control_test_duty[drv] = 0.0f;
    AppControl_RecordDrvDuty(drv, 0.0f);
}

/** @brief 关闭当前测试模式中仍在输出的全部 DRV。 */
static void AppControl_StopDrvTestOutputs(uint8_t mask)
{
    uint8_t drv;

    mask &= APP_CONTROL_DRV_ALL_MASK;
    for (drv = 0U; drv < APP_CONTROL_DRV_COUNT; drv++)
    {
        if ((mask & (uint8_t)(1U << drv)) != 0U)
            AppControl_StopDrvTestOutput(drv);
    }
}

/** @brief 兼容调试器直接置位 g_app_control_test_active 的用法，自动补齐默认测试参数。 */
static void AppControl_DrvTestEnsureDefaultRequest(void)
{
    uint8_t drv;

    if ((g_app_control_drv_test_requested_mask & APP_CONTROL_DRV_ALL_MASK) != 0U)
        return;

    g_app_control_drv_test_requested_mask = APP_CONTROL_DRV_TEST_DEFAULT_MASK;
    for (drv = 0U; drv < APP_CONTROL_DRV_COUNT; drv++)
        g_app_control_test_duty[drv] = 0.0f;
    g_app_control_test_duty[APP_CONTROL_SHARED_DRV] = APP_CONTROL_DRV_TEST_DEFAULT_DUTY;
}

/** @brief 初始化测试通道，并只让请求掩码中的 DRV 输出指定 duty。 */
static void AppControl_DrvTestInit(uint32_t now_ms)
{
    uint8_t drv;
    uint8_t requested_mask;

    (void)now_ms;
    AppControl_DrvTestEnsureDefaultRequest();
    requested_mask = (uint8_t)(g_app_control_drv_test_requested_mask & APP_CONTROL_DRV_ALL_MASK);

    g_app_control_drv_test_active_mask = 0U;
    g_app_control_drv_test_fault_mask = 0U;
    g_app_control_drv_test_fault_poll_count = 0U;
    s_drv_test_last_fault_poll_ms = 0U;

    for (drv = 0U; drv < APP_CONTROL_DRV_COUNT; drv++)
    {
        DRV8703_Status_t ret = DRV8703_OK;
        uint8_t bit = (uint8_t)(1U << drv);

        g_app_control_test_drv_ok[drv] = 0U;
        g_app_control_drv_test_status[drv] = DRV8703_OK;

        if ((requested_mask & bit) == 0U)
        {
            AppControl_StopDrvTestOutput(drv);
            continue;
        }

        g_app_control_test_duty[drv] = AppControl_Clamp(g_app_control_test_duty[drv],
                                                        -APP_CONTROL_MAX_ABS_DUTY,
                                                        APP_CONTROL_MAX_ABS_DUTY);
        ret = AppControl_SetDrvDuty(drv, g_app_control_test_duty[drv]);
        g_app_control_drv_test_status[drv] = ret;
        if (ret == DRV8703_OK)
        {
            g_app_control_test_drv_ok[drv] = 1U;
            g_app_control_drv_test_active_mask |= bit;
        }
        else
        {
            g_app_control_drv_test_fault_mask |= bit;
            AppControl_StopDrvTestOutput(drv);
        }
    }

    s_test_drv_initialized = 1U;
    g_app_control_test_phase = (g_app_control_drv_test_active_mask != 0U) ? 1U : 3U;
}

/** @brief 测试模式快速轮询 FAULT 引脚，发现故障后关闭对应测试输出。 */
static void AppControl_DrvTestPollFaults(uint32_t now_ms)
{
    uint8_t drv;

    if ((now_ms - s_drv_test_last_fault_poll_ms) < APP_CONTROL_DRV_TEST_FAULT_POLL_MS)
        return;
    s_drv_test_last_fault_poll_ms = now_ms;
    g_app_control_drv_test_fault_poll_count++;

    if (g_app_control_simulate_drv8703 != 0U)
        return;

    for (drv = 0U; drv < APP_CONTROL_DRV_COUNT; drv++)
    {
        uint8_t bit = (uint8_t)(1U << drv);
        DRV8703_Handle_t *dev;
        DRV8703_Status_t pin_ret;

        if ((g_app_control_drv_test_active_mask & bit) == 0U)
            continue;

        dev = DRV8703_BoardGet((DRV8703_BoardChannel_t)drv);
        if (dev == 0)
        {
            g_app_control_drv_test_status[drv] = DRV8703_ERROR_PARAM;
            g_app_control_drv_test_fault_mask |= bit;
            g_app_control_drv_test_active_mask &= (uint8_t)(~bit);
            AppControl_StopDrvTestOutput(drv);
            continue;
        }

        pin_ret = DRV8703_CheckFaultPins(dev);
        g_app_control_drv_test_status[drv] = pin_ret;
        if (pin_ret == DRV8703_OK)
            continue;

#if APP_CONTROL_DRV_TEST_STOP_ALL_ON_FAULT
        (void)AppControl_CaptureDrvPinFaultMoment(drv, pin_ret, 1U);
        g_app_control_drv_test_fault_mask |= g_app_control_drv_test_active_mask;
        AppControl_StopDrvTestOutputs(g_app_control_drv_test_active_mask);
        g_app_control_drv_test_active_mask = 0U;
        break;
#else
        (void)AppControl_CaptureDrvPinFaultMoment(drv, pin_ret, 0U);
        g_app_control_drv_test_fault_mask |= bit;
        g_app_control_drv_test_active_mask &= (uint8_t)(~bit);
        g_app_control_test_drv_ok[drv] = 0U;
        AppControl_StopDrvTestOutput(drv);
#endif
    }

    if (g_app_control_drv_test_active_mask == 0U)
        g_app_control_test_phase = 3U;
}

/** @brief DRV 测试模式主循环：输出测试 duty、快速查错并保留寄存器快照。 */
static void AppControl_DrvTestTask(uint32_t now_ms)
{
    uint8_t drv;

    if (s_test_drv_initialized == 0U)
        AppControl_DrvTestInit(now_ms);

    for (drv = 0U; drv < APP_CONTROL_DRV_COUNT; drv++)
    {
        uint8_t bit = (uint8_t)(1U << drv);

        if ((g_app_control_drv_test_active_mask & bit) == 0U)
            continue;

        g_app_control_test_duty[drv] = AppControl_Clamp(g_app_control_test_duty[drv],
                                                        -APP_CONTROL_MAX_ABS_DUTY,
                                                        APP_CONTROL_MAX_ABS_DUTY);
        if (AppControl_SetDrvDuty(drv, g_app_control_test_duty[drv]) != DRV8703_OK)
        {
            g_app_control_drv_test_fault_mask |= bit;
            g_app_control_drv_test_active_mask &= (uint8_t)(~bit);
            g_app_control_test_drv_ok[drv] = 0U;
            AppControl_StopDrvTestOutput(drv);
        }
    }

    if (g_app_control_drv_test_active_mask == 0U)
        g_app_control_test_phase = 3U;

    AppControl_DrvTestPollFaults(now_ms);
    AppControl_CapturePeriodicDrvRegs(now_ms);
    AppControl_ReadAllDrvRegsRaw(now_ms);
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
/** @brief 初始化 app_control 模块，创建队列、初始化 PID、清空调试状态并加载标定数据。*/
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
        g_app_control_test_drv_ok[i] = 0U;
        g_app_control_test_duty[i] = 0.0f;
        g_app_control_drv_test_status[i] = DRV8703_OK;
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
    g_app_control_test_active = 0U;
    g_app_control_test_phase = 0U;
    g_app_control_drv_test_requested_mask = 0U;
    g_app_control_drv_test_active_mask = 0U;
    g_app_control_drv_test_fault_mask = 0U;
    g_app_control_drv_test_fault_poll_count = 0U;
    s_test_drv_initialized = 0U;
    s_drv_test_last_fault_poll_ms = 0U;

    /* 加载两个 Cell 的历史标定数据用于前馈控制。*/
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
 * Test mode (g_app_control_test_active != 0): output the requested DRV
 * test duties, poll FAULT pins faster than normal control, stop the failed
 * channel, and periodically snapshot DRV8703 registers via SPI.
 *
 * @param now_ms Current RTOS tick.
 */
/** @brief ControlTask 周期入口，调度标定、测温、故障检测、闭环和上位机数据上报。*/
void AppControl_Task(uint32_t now_ms)
{
    /*
     * DRV 测试调用示例 1：单独测试 DRV1，输出 0.10 duty。
     *
     * 这段示例放在 ControlTask 正常入口处，但保持注释状态，不参与编译。
     * 如果需要临时启用，取消下面代码块的注释即可。注意它必须放在
     * AppControl_Lock() 之前，因为 AppControl_StartDrvTest() 内部会自行加锁。
     * s_drv1_test_example_started 用于避免每个控制周期重复启动测试。
     */
    /*
    static uint8_t s_drv1_test_example_started = 0U;
    if (s_drv1_test_example_started == 0U)
    {
        float drv_test_duty[APP_CONTROL_DRV_COUNT] = {0.0f};
        drv_test_duty[0] = 0.10f;
        (void)AppControl_StartDrvTest((uint8_t)(1U << 0), drv_test_duty);
        s_drv1_test_example_started = 1U;
    }
    */

    AppControl_Lock();

    if (g_app_control_test_active != 0U)
    {
        AppControl_DrvTestTask(now_ms);
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
            /* 标定刚刚完成，重新加载 Flash 标定数据用于前馈。*/
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
    Ads1220_Test(); /* 临时：ADS1220 测试，验证通过后删除。*/

    /* PC 协议：每 1s 发送一次过程数据。*/
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
 * @brief 启动 DRV 测试模式。
 *
 * @param drv_mask 测试通道掩码，bit0..bit4 分别对应 DRV1..DRV5。
 * @param duty 每路 DRV 的测试 duty 数组；只会使用 drv_mask 选中的通道。
 *
 * 测试模式会先紧急停止两个 cell 的温控输出，再关闭所有旧测试输出，随后只打开
 * drv_mask 指定的通道。AppControl_Task 后续会以 APP_CONTROL_DRV_TEST_FAULT_POLL_MS
 * 为周期检查 FAULT 引脚；发现故障后会关闭故障通道，并写入
 * g_app_control_drv_test_fault_mask 供调试观察。
 */
DRV8703_Status_t AppControl_StartDrvTest(uint8_t drv_mask, const float duty[APP_CONTROL_DRV_COUNT])
{
    uint8_t drv;
    float requested_duty[APP_CONTROL_DRV_COUNT];

    drv_mask &= APP_CONTROL_DRV_ALL_MASK;
    if ((drv_mask == 0U) || (duty == 0))
        return DRV8703_ERROR_PARAM;

    for (drv = 0U; drv < APP_CONTROL_DRV_COUNT; drv++)
    {
        requested_duty[drv] = AppControl_Clamp(duty[drv],
                                               -APP_CONTROL_MAX_ABS_DUTY,
                                               APP_CONTROL_MAX_ABS_DUTY);
    }

    AppControl_Lock();

    AppControl_StopCell(0U, APP_CONTROL_STOP_EMERGENCY);
    AppControl_StopCell(1U, APP_CONTROL_STOP_EMERGENCY);
    AppControl_StopDrvTestOutputs(APP_CONTROL_DRV_ALL_MASK);

    g_app_control_drv_test_requested_mask = drv_mask;
    g_app_control_drv_test_active_mask = 0U;
    g_app_control_drv_test_fault_mask = 0U;
    g_app_control_drv_test_fault_poll_count = 0U;
    s_drv_test_last_fault_poll_ms = 0U;
    s_test_drv_initialized = 0U;

    for (drv = 0U; drv < APP_CONTROL_DRV_COUNT; drv++)
    {
        g_app_control_test_drv_ok[drv] = 0U;
        g_app_control_drv_test_status[drv] = DRV8703_OK;
        g_app_control_test_duty[drv] = requested_duty[drv];
    }

    g_app_control_test_phase = 0U;
    g_app_control_test_active = 1U;

    AppControl_Unlock();
    return DRV8703_OK;
}

/**
 * @brief 停止 DRV 测试模式，并关闭所有 DRV 测试输出。
 *
 * 该接口会清除测试 active mask 和 requested mask，但保留 fault mask，方便停止后继续
 * 在调试器里查看本轮测试哪些通道触发过故障。
 */
void AppControl_StopDrvTest(void)
{
    uint8_t drv;

    AppControl_Lock();

    AppControl_StopDrvTestOutputs(APP_CONTROL_DRV_ALL_MASK);
    g_app_control_test_active = 0U;
    g_app_control_test_phase = 2U;
    g_app_control_drv_test_requested_mask = 0U;
    g_app_control_drv_test_active_mask = 0U;
    s_test_drv_initialized = 0U;

    for (drv = 0U; drv < APP_CONTROL_DRV_COUNT; drv++)
        g_app_control_test_drv_ok[drv] = 0U;

    AppControl_Unlock();
}

/** @brief 获取测试模式当前仍在输出的 DRV 掩码。 */
uint8_t AppControl_GetDrvTestActiveMask(void)
{
    return g_app_control_drv_test_active_mask;
}

/** @brief 获取测试模式已经检测到故障的 DRV 掩码。 */
uint8_t AppControl_GetDrvTestFaultMask(void)
{
    return g_app_control_drv_test_fault_mask;
}
/**
 * @brief Update the panel UI with current cell temperatures and errors.
 *
 * Called from HMITask.  Acquires the system mutex only long enough to
 * copy the cell temperatures and errors.
 */
/** @brief HMITask 调用，将当前 cell 温度和错误状态刷新到面板。*/
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
/** @brief 向控制任务命令队列投递 cell 启动/停止请求。*/
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
/** @brief 公开接口：请求启动指定 cell 的 PID 温控。*/
void Control_StartPid(uint8_t cell)
{
    AppControl_PostCommand(APP_CONTROL_CMD_START, cell, APP_CONTROL_STOP_NORMAL);
}

/** @brief Public API: request PID stop for a cell. */
/** @brief 公开接口：请求正常停止指定 cell 的 PID 温控。*/
void Control_StopPid(uint8_t cell)
{
    AppControl_PostCommand(APP_CONTROL_CMD_STOP, cell, APP_CONTROL_STOP_NORMAL);
}

/** @brief 公开接口：请求紧急停止指定 cell 的 PID 温控。*/
void Control_EmergencyStopPid(uint8_t cell)
{
    AppControl_PostCommand(APP_CONTROL_CMD_STOP, cell, APP_CONTROL_STOP_EMERGENCY);
}

/** @brief 公开接口：请求紧急停止全部 cell。*/
void Control_EmergencyStopAll(void)
{
    Control_EmergencyStopPid(0U);
    Control_EmergencyStopPid(1U);
}

/** @brief Public API: set the target temperature for a cell. */
/** @brief 公开接口：设置指定 cell 的目标温度。*/
void Control_SetTargetTemp(uint8_t cell, float target)
{
    if (cell >= APP_CONTROL_CELL_COUNT)
        return;

    AppControl_Lock();
    s_cell[cell].target_temp = target;
    AppControl_Unlock();
}