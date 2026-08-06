#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include "main.h"
#include "drv8703.h"
#include "temp_panel.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief 温控池数量。当前固件保留 Cell 0 和 Cell 1 两个温控池。*/
#define APP_CONTROL_CELL_COUNT 2U
/** @brief DRV8703 芯片数量。DRV1..DRV4 为闭环通道，DRV5 为共享通道。*/
#define APP_CONTROL_DRV_COUNT 5U
/** @brief 测温输入数量。索引 0..3 分别对应实际测温 CH1..CH4。*/
#define APP_CONTROL_TEMP_INPUT_COUNT 4U
/** @brief 参与 cell 闭环控制的 DRV 通道数量，不包含共享 DRV5。*/
#define APP_CONTROL_CLOSED_LOOP_COUNT 4U
/** @brief 无效索引标记，用于表示无共享通道或查找失败。*/
#define APP_CONTROL_INVALID_INDEX 0xFFU

/*
 * 堆叠冷热片通道映射配置。
 *
 * 注意：下面所有索引都从 0 开始。
 *   TEMP 0..3 对应实际测温 CH1..CH4。
 *   DRV  0..4 对应实际 DRV1..DRV5。
 *
 * 每个 cell 使用两路测温取均值做 PID。PID 输出代表外层/主路 duty，
 * 内层/从路 duty 按 inner_ratio 跟随外层。
 */
/** @brief Cell 0 外层测温输入索引，默认实际 CH1。*/
#define APP_CONTROL_CELL0_TEMP_OUTER 0U
/** @brief Cell 0 内层测温输入索引，默认实际 CH2。*/
#define APP_CONTROL_CELL0_TEMP_INNER 1U
/** @brief Cell 0 外层/主路 DRV 索引，默认实际 DRV1。*/
#define APP_CONTROL_CELL0_DRV_OUTER 0U
/** @brief Cell 0 内层/从路 DRV 索引，默认实际 DRV2。*/
#define APP_CONTROL_CELL0_DRV_INNER 1U
/** @brief Cell 0 内层 duty 跟随比例，inner = outer * ratio。*/
#define APP_CONTROL_CELL0_INNER_DUTY_RATIO 0.70f

/** @brief Cell 1 外层测温输入索引，默认实际 CH3。*/
#define APP_CONTROL_CELL1_TEMP_OUTER 2U
/** @brief Cell 1 内层测温输入索引，默认实际 CH4。*/
#define APP_CONTROL_CELL1_TEMP_INNER 3U
/** @brief Cell 1 外层/主路 DRV 索引，默认实际 DRV3。*/
#define APP_CONTROL_CELL1_DRV_OUTER 2U
/** @brief Cell 1 内层/从路 DRV 索引，默认实际 DRV4。*/
#define APP_CONTROL_CELL1_DRV_INNER 3U
/** @brief Cell 1 内层 duty 跟随比例，inner = outer * ratio。*/
#define APP_CONTROL_CELL1_INNER_DUTY_RATIO 0.70f

/** @brief 堆叠方案共享 DRV 索引，默认实际 DRV5。*/
#define APP_CONTROL_SHARED_DRV 4U

/** @brief PID 输出和实际 PWM duty 的最大绝对值限幅。*/
#define APP_CONTROL_MAX_ABS_DUTY 0.45f

/** @brief 只要任意 cell 运行，共享 DRV5 输出的固定 duty。*/
#define APP_CONTROL_SHARED_CH5_DUTY 0.20f

    /** @brief app_control 模块初始化和命令投递的返回状态。*/
    typedef enum
    {
        APP_CONTROL_OK = 0,        /**< 操作成功。*/
        APP_CONTROL_ERROR_QUEUE,   /**< 控制命令队列创建或访问失败。*/
        APP_CONTROL_ERROR_PARAM    /**< 参数非法，例如 cell 编号越界。*/
    } AppControl_Status_t;

    /** @brief 控制任务内部命令类型。*/
    typedef enum
    {
        APP_CONTROL_CMD_START = 0, /**< 启动指定 cell 的温控。*/
        APP_CONTROL_CMD_STOP       /**< 停止指定 cell 的温控。*/
    } AppControlCommandType_t;

    /** @brief 停止方式。*/
    typedef enum
    {
        APP_CONTROL_STOP_NORMAL = 0, /**< 正常停止，允许按斜率逐步降到 0 duty。*/
        APP_CONTROL_STOP_EMERGENCY   /**< 紧急停止，尽快关闭对应输出。*/
    } AppControlStopMode_t;

    /** @brief 投递给 ControlTask 的 cell 控制命令。*/
    typedef struct
    {
        AppControlCommandType_t type;     /**< 命令类型：启动或停止。*/
        uint8_t cell;                     /**< 目标 cell 编号，范围 0..APP_CONTROL_CELL_COUNT-1。*/
        AppControlStopMode_t stop_mode;   /**< 停止命令使用的停止方式，启动命令忽略。*/
    } AppControlCommand_t;

    /** @brief DRV8703 仿真开关，非 0 时跳过真实 DRV 初始化/读写，便于脱板调试。*/
    extern volatile uint8_t g_app_control_simulate_drv8703;
    /** @brief 电压检测仿真开关，非 0 时按电压正常处理。*/
    extern volatile uint8_t g_app_control_simulate_voltage_ok;
    /** @brief AppControl_Init 最近一次初始化结果。*/
    extern volatile AppControl_Status_t g_app_control_init_result;
    /** @brief AppControl_Task 主循环执行次数计数。*/
    extern volatile uint32_t g_app_control_loop_count;
    /** @brief 控制命令队列满或不可用导致丢弃的命令数量。*/
    extern volatile uint32_t g_app_control_cmd_drop_count;
    /** @brief 每个 cell 当前是否处于运行状态。*/
    extern volatile uint8_t g_app_control_cell_running[APP_CONTROL_CELL_COUNT];
    /** @brief 每个 cell 当前面板错误码。*/
    extern volatile PanelError_t g_app_control_cell_error[APP_CONTROL_CELL_COUNT];
    /** @brief 每个 cell 的控制反馈温度，即两路测温均值。*/
    extern volatile float g_app_control_cell_temp[APP_CONTROL_CELL_COUNT];
    /** @brief 每个 cell 当前目标温度。*/
    extern volatile float g_app_control_cell_target[APP_CONTROL_CELL_COUNT];
    /** @brief 每个 cell 主路 duty，保留旧变量名兼容调试观察。*/
    extern volatile float g_app_control_cell_duty[APP_CONTROL_CELL_COUNT];
    /** @brief 每个 cell 外层/主路 DRV 实际输出 duty。*/
    extern volatile float g_app_control_cell_outer_duty[APP_CONTROL_CELL_COUNT];
    /** @brief 每个 cell 内层/从路 DRV 实际输出 duty。*/
    extern volatile float g_app_control_cell_inner_duty[APP_CONTROL_CELL_COUNT];
    /** @brief 每个 cell 内层 duty 跟随比例。*/
    extern volatile float g_app_control_cell_inner_ratio[APP_CONTROL_CELL_COUNT];
    /** @brief 按 DRV 闭环通道记录的反馈温度，主要用于 Keil/上位机调试观察。*/
    extern volatile float g_app_control_pid_temp[APP_CONTROL_CLOSED_LOOP_COUNT];
    /** @brief 按 DRV 闭环通道记录的实际 duty。*/
    extern volatile float g_app_control_pid_duty[APP_CONTROL_CLOSED_LOOP_COUNT];
    /** @brief 按 DRV 闭环通道记录的 PID 更新触发标志。*/
    extern volatile uint8_t g_app_control_pid_update_pending[APP_CONTROL_CLOSED_LOOP_COUNT];
    /** @brief 每个 DRV 初始化尝试次数。*/
    extern volatile uint8_t g_app_control_drv_init_attempts[APP_CONTROL_DRV_COUNT];
    /** @brief 每个 DRV 初始化是否完成。*/
    extern volatile uint8_t g_app_control_drv_ready[APP_CONTROL_DRV_COUNT];
    /** @brief 每个 DRV 当前是否处于唤醒状态。*/
    extern volatile uint8_t g_app_control_drv_awake[APP_CONTROL_DRV_COUNT];
    /** @brief 每个 DRV 是否检测到故障。*/
    extern volatile uint8_t g_app_control_drv_fault[APP_CONTROL_DRV_COUNT];
    /** @brief 启动后 DRV 寄存器快照是否有效。*/
    extern volatile uint8_t g_app_control_drv_startup_dump_valid[APP_CONTROL_DRV_COUNT];
    /** @brief 启动后整片 DRV 寄存器快照读取状态。*/
    extern volatile DRV8703_Status_t g_app_control_drv_startup_dump_status[APP_CONTROL_DRV_COUNT];
    /** @brief 启动后读取到的每个 DRV 寄存器值。*/
    extern volatile uint8_t g_app_control_drv_startup_reg_dump[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT];
    /** @brief 启动后每个 DRV 寄存器的读取状态。*/
    extern volatile DRV8703_Status_t g_app_control_drv_startup_reg_status[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT];
    /** @brief 启动后读取寄存器时最后一次 SPI TX 原始字。*/
    extern volatile uint16_t g_app_control_drv_startup_tx[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT];
    /** @brief 启动后读取寄存器时最后一次 SPI RX 原始字。*/
    extern volatile uint16_t g_app_control_drv_startup_rx[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT];
    /** @brief DRV8703 初始化后期望看到的默认寄存器值。*/
    extern volatile uint8_t g_app_control_drv_startup_expected[DRV8703_REGISTER_COUNT];
    /** @brief 最近一次发生故障的 DRV 索引，0xFF 表示暂无。*/
    extern volatile uint8_t g_app_control_last_drv_fault;
    /** @brief 最近一次 DRV 操作返回状态。*/
    extern volatile DRV8703_Status_t g_app_control_last_drv_status;
    /** @brief 故障瞬间寄存器快照是否有效。*/
    extern volatile uint8_t g_app_control_drv_fault_snapshot_valid[APP_CONTROL_DRV_COUNT];
    /** @brief 每个 DRV 捕获故障快照的次数。*/
    extern volatile uint32_t g_app_control_drv_fault_capture_count[APP_CONTROL_DRV_COUNT];
    /** @brief 读取 DRV 故障状态寄存器的返回状态。*/
    extern volatile DRV8703_Status_t g_app_control_drv_fault_read_status[APP_CONTROL_DRV_COUNT];
    /** @brief 故障快照整片寄存器读取状态。*/
    extern volatile DRV8703_Status_t g_app_control_drv_dump_status[APP_CONTROL_DRV_COUNT];
    /** @brief DRV8703 FAULT_STATUS 寄存器快照值。*/
    extern volatile uint8_t g_app_control_drv_fault_status[APP_CONTROL_DRV_COUNT];
    /** @brief DRV8703 VDS/GDF 相关寄存器快照值。*/
    extern volatile uint8_t g_app_control_drv_vds_gdf_status[APP_CONTROL_DRV_COUNT];
    /** @brief 最近一次故障读取到的全部 DRV 寄存器值。*/
    extern volatile uint8_t g_app_control_drv_reg_dump[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT];
    /** @brief 最近一次故障读取每个寄存器的状态。*/
    extern volatile DRV8703_Status_t g_app_control_drv_reg_read_status[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT];
    /** @brief 最近一次故障读取成功的寄存器位掩码。*/
    extern volatile uint8_t g_app_control_drv_reg_read_ok_mask[APP_CONTROL_DRV_COUNT];
    /** @brief 最近一次 FAULT 引脚触发的 DRV 索引。*/
    extern volatile uint8_t g_app_control_drv_pin_fault_last;
    /** @brief 每个 DRV 由 FAULT 引脚触发的次数。*/
    extern volatile uint32_t g_app_control_drv_pin_fault_count[APP_CONTROL_DRV_COUNT];
    /** @brief FAULT 引脚触发时读取芯片状态的返回值。*/
    extern volatile DRV8703_Status_t g_app_control_drv_pin_fault_status[APP_CONTROL_DRV_COUNT];
    /** @brief FAULT 引脚触发时读。FAULT_STATUS 的返回值。*/
    extern volatile DRV8703_Status_t g_app_control_drv_pin_fault_read_status[APP_CONTROL_DRV_COUNT];
    /** @brief FAULT 引脚触发时读取整片寄存器快照的返回值。*/
    extern volatile DRV8703_Status_t g_app_control_drv_pin_fault_dump_status[APP_CONTROL_DRV_COUNT];
    /** @brief FAULT 引脚触发时捕获的 FAULT_STATUS 寄存器值。*/
    extern volatile uint8_t g_app_control_drv_pin_fault_fault_status[APP_CONTROL_DRV_COUNT];
    /** @brief FAULT 引脚触发时捕获的 VDS/GDF 寄存器值。*/
    extern volatile uint8_t g_app_control_drv_pin_fault_vds_gdf_status[APP_CONTROL_DRV_COUNT];
    /** @brief FAULT 引脚触发时判定需要停机的故障位。*/
    extern volatile uint8_t g_app_control_drv_pin_fault_stop_bits[APP_CONTROL_DRV_COUNT];
    /** @brief FAULT 引脚触发时捕获的全部寄存器值。*/
    extern volatile uint8_t g_app_control_drv_pin_fault_reg_dump[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT];
    /** @brief FAULT 引脚触发时每个寄存器读取状态。*/
    extern volatile DRV8703_Status_t g_app_control_drv_pin_fault_reg_read_status[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT];
    /** @brief FAULT 引脚触发时读取成功的寄存器位掩码。*/
    extern volatile uint8_t g_app_control_drv_pin_fault_reg_read_ok_mask[APP_CONTROL_DRV_COUNT];
    /** @brief FAULT 引脚触发时是否出现全 全 0xFF 异常读数。*/
    extern volatile uint8_t g_app_control_drv_pin_fault_all_ff[APP_CONTROL_DRV_COUNT];
    /** @brief 每个 DRV 出现。全 0xFF 异常读数的累计次数。*/
    extern volatile uint32_t g_app_control_drv_pin_fault_all_ff_count[APP_CONTROL_DRV_COUNT];
    /** @brief FAULT 引脚触发后连续稳定读到故障状态的次数。*/
    extern volatile uint8_t g_app_control_drv_pin_fault_stable_count[APP_CONTROL_DRV_COUNT];
    /** @brief DRV 进入 sleep 前捕获的寄存器快照，最后一列为有效标志。*/
    extern volatile uint8_t g_app_control_sleep_reg_snapshot[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT + 1U];
    /** @brief 周期性捕获的 DRV 寄存器快照，最后一列为有效标志。*/
    extern volatile uint8_t g_app_control_periodic_reg_snapshot[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT + 1U];
    /** @brief 周期性寄存器快照捕获次数。*/
    extern volatile uint32_t g_app_control_periodic_reg_snapshot_count;
    /** @brief 每个 cell 已触发测温传感器复位的次数。*/
    extern volatile uint8_t g_app_control_temp_reset_count[APP_CONTROL_CELL_COUNT];
    /** @brief 每路测温最近更新时间戳。*/
    extern volatile uint32_t g_app_control_temp_last_update_tick[APP_CONTROL_TEMP_INPUT_COUNT];
    /** @brief 每路测温累计更新次数。*/
    extern volatile uint32_t g_app_control_temp_update_count[APP_CONTROL_TEMP_INPUT_COUNT];
    /** @brief 每个 cell 最近判定异常的测温输入索引，0xFF 表示暂无。*/
    extern volatile uint8_t g_app_control_temp_fault_sensor[APP_CONTROL_CELL_COUNT];
    /** @brief 测温传感器复位流程是否正在进行。*/
    extern volatile uint8_t g_app_control_temp_reset_active;

    /** @brief 每路测温输入的实际更新频率，单位 Hz。*/
    extern volatile float g_app_control_temp_freq_hz[APP_CONTROL_TEMP_INPUT_COUNT];
    /** @brief 每路闭环 DRV 通道的 PID 计算触发频率，单位 Hz。*/
    extern volatile float g_app_control_pid_freq_hz[APP_CONTROL_CLOSED_LOOP_COUNT];

    /** @brief USART1 测温串口 DMA 需要重启的标志，由中断置位、控制任务处理。*/
    extern volatile uint8_t g_uart_need_restart;

    /** @brief USART2 上位机协议接收错误后需要重。DMA 的标志。*/
    extern volatile uint8_t g_uart2_need_restart;

    /*
     * DRV8703 手动测试模式。
     * 推荐通过 AppControl_StartDrvTest() 启动；也可以在调试器中设置
     * g_app_control_drv_test_requested_mask、g_app_control_test_duty[]，再置位
     * g_app_control_test_active。测试期间不运行温控闭环，而是按请求通道输出
     * duty，并快速轮询 FAULT 引脚；发现故障后关闭故障通道输出。
     */
    /** @brief 手动测试模式使能标志。*/
    extern volatile uint8_t g_app_control_test_active;
    /** @brief 手动测试模式阶段状态。*/
    extern volatile uint8_t g_app_control_test_phase;
    /** @brief 手动测试模式下每个 DRV 是否初始化成功。*/
    extern volatile uint8_t g_app_control_test_drv_ok[APP_CONTROL_DRV_COUNT];
    /** @brief 手动测试模式下每个 DRV 当前测试 duty。*/
    extern volatile float g_app_control_test_duty[APP_CONTROL_DRV_COUNT];
    /** @brief DRV 测试请求掩码，bit0..bit4 对应 DRV1..DRV5。 */
    extern volatile uint8_t g_app_control_drv_test_requested_mask;
    /** @brief DRV 测试当前仍在输出的通道掩码；故障通道会被自动清除。 */
    extern volatile uint8_t g_app_control_drv_test_active_mask;
    /** @brief DRV 测试期间已经检测到故障的通道掩码。 */
    extern volatile uint8_t g_app_control_drv_test_fault_mask;
    /** @brief DRV 测试模式 FAULT 轮询累计次数。 */
    extern volatile uint32_t g_app_control_drv_test_fault_poll_count;
    /** @brief DRV 测试期间每个通道最近一次状态，OK 表示最近未检测到故障。 */
    extern volatile DRV8703_Status_t g_app_control_drv_test_status[APP_CONTROL_DRV_COUNT];

/*
 * DRV8703 原始寄存器轮询。
 *
 * 该轮询独立于故障快照，会定期尝试读取全部 5 颗 DRV8703，
 * 用于观察芯片通信状态和寄存器原始值。
 */
/** @brief DRV8703 原始寄存器轮询周期，单位 ms。*/
#define APP_CONTROL_DRV_RAW_POLL_MS 200U

    /** @brief DRV8703 原始寄存器轮询次数。*/
    extern volatile uint32_t g_app_control_drv_raw_poll_count;
    /** @brief 周期轮询读取到的 DRV8703 原始寄存器值。*/
    extern volatile uint8_t g_app_control_drv_raw_dump[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT];
    /** @brief 周期轮询中每个 DRV 读取成功的寄存器位掩码。*/
    extern volatile uint8_t g_app_control_drv_raw_mask[APP_CONTROL_DRV_COUNT];
    /** @brief 周期轮询中每个 DRV 最近一次读取状态。*/
    extern volatile DRV8703_Status_t g_app_control_drv_raw_status[APP_CONTROL_DRV_COUNT];

    /** @brief 初始化 app_control 模块、命令队列、cell 状态、PID 和调试变量。*/
    AppControl_Status_t AppControl_Init(void);
    /** @brief 控制任务主循环入口，周期处理命令、测温、故障、闭环、标定和上位机数据。*/
    void AppControl_Task(uint32_t now_ms);
    /** @brief 将 app_control 内部状态同步到 TM1638 面板状态。*/
    void AppControl_UpdatePanel(TempPanel_t *panel, uint32_t now_ms);

    /** @brief 直接设置指定 DRV 的 PWM duty，正负号表示制冷/加热方向。*/
    DRV8703_Status_t AppControl_SetDrvDuty(uint8_t drv, float duty);
    /** @brief 启动 DRV 测试模式；drv_mask 的 bit0..bit4 对应 DRV1..DRV5，测试期间会快速检查故障。 */
    DRV8703_Status_t AppControl_StartDrvTest(uint8_t drv_mask, const float duty[APP_CONTROL_DRV_COUNT]);
    /** @brief 停止 DRV 测试模式，并关闭测试通道输出。 */
    void AppControl_StopDrvTest(void);
    /** @brief 获取测试模式当前仍在输出的 DRV 掩码。 */
    uint8_t AppControl_GetDrvTestActiveMask(void);
    /** @brief 获取测试模式已经检测到故障的 DRV 掩码。 */
    uint8_t AppControl_GetDrvTestFaultMask(void);

    /*
     * DRV 测试调用示例，示例代码保持注释状态，临时调试时再复制到合适位置调用。
     * 注意：DRV 编号是 0 基索引，0=DRV1，1=DRV2，2=DRV3，3=DRV4，4=DRV5。
     * duty 正负号沿用当前驱动方向定义，绝对值会被限制到 APP_CONTROL_MAX_ABS_DUTY 以内。
     *
     * 示例 1：单独测试 DRV1，输出 0.10 duty。
     *
     *     float drv_test_duty[APP_CONTROL_DRV_COUNT] = {0.0f};
     *     drv_test_duty[0] = 0.10f;
     *     (void)AppControl_StartDrvTest((uint8_t)(1U << 0), drv_test_duty);
     *
     * 示例 2：单独测试共享 DRV5，输出 0.20 duty。
     *
     *     float drv_test_duty[APP_CONTROL_DRV_COUNT] = {0.0f};
     *     drv_test_duty[APP_CONTROL_SHARED_DRV] = 0.20f;
     *     (void)AppControl_StartDrvTest((uint8_t)(1U << APP_CONTROL_SHARED_DRV), drv_test_duty);
     *
     * 示例 3：同时测试 DRV1 和 DRV3；如果其中一路报 fault，默认只关闭故障通道。
     *
     *     float drv_test_duty[APP_CONTROL_DRV_COUNT] = {0.0f};
     *     drv_test_duty[0] = 0.10f;
     *     drv_test_duty[2] = -0.10f;
     *     (void)AppControl_StartDrvTest((uint8_t)((1U << 0) | (1U << 2)), drv_test_duty);
     *
     * 停止测试：
     *
     *     AppControl_StopDrvTest();
     *
     * 调试观察：
     *
     *     g_app_control_drv_test_active_mask  // 当前仍在输出的 DRV 掩码
     *     g_app_control_drv_test_fault_mask   // 已检测到故障并关闭的 DRV 掩码
     *     g_app_control_drv_test_status[]     // 每路最近一次测试状态
     */

    /** @brief 设置指定 cell 的堆叠冷热片输出；outer_duty 给外层，内层按比例跟随。*/
    DRV8703_Status_t AppControl_SetCellStackDuty(uint8_t cell, float outer_duty);
    /** @brief 获取指定 cell 外层测温输入索引。*/
    uint8_t AppControl_GetCellOuterTempIndex(uint8_t cell);
    /** @brief 获取指定 cell 内层测温输入索引。*/
    uint8_t AppControl_GetCellInnerTempIndex(uint8_t cell);
    /** @brief 获取指定 cell 内层 duty 跟随比例。*/
    float AppControl_GetCellInnerDutyRatio(uint8_t cell);
    /** @brief 获取指定 cell 外层/主路当前 duty。*/
    float AppControl_GetCellOuterDuty(uint8_t cell);
    /** @brief 获取指定 cell 内层/从路当前 duty。*/
    float AppControl_GetCellInnerDuty(uint8_t cell);

    /** @brief 请求启动指定 cell 的 PID 温控，由 ControlTask 异步执行。*/
    void Control_StartPid(uint8_t cell);
    /** @brief 请求正常停止指定 cell 的 PID 温控，由 ControlTask 异步执行。*/
    void Control_StopPid(uint8_t cell);
    /** @brief 请求紧急停止指定 cell 的 PID 温控。*/
    void Control_EmergencyStopPid(uint8_t cell);
    /** @brief 请求紧急停止全部 cell。*/
    void Control_EmergencyStopAll(void);
    /** @brief 设置指定 cell 的目标温度。*/
    void Control_SetTargetTemp(uint8_t cell, float target);

#ifdef __cplusplus
}
#endif

#endif