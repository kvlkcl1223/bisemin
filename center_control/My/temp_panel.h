#ifndef TEMP_PANEL_H
#define TEMP_PANEL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 1. 系统基础配置
 * ============================================================ */

/*
 * 你的系统中有两个恒温池：
 * cell[0] 表示 1 号恒温池
 * cell[1] 表示 2 号恒温池
 */
#define PANEL_CELL_NUM              2

/*
 * 温度全部采用“0.1℃”为单位。
 *
 * 例如：
 * 25.0℃ 记为 250
 * 37.5℃ 记为 375
 * -5.0℃ 记为 -50
 *
 * 这样做的好处：
 * 1. 不用 float，适合 STM32
 * 2. 显示 0.1℃ 很方便
 * 3. PID 也可以使用整数或定点数
 */
#define PANEL_TEMP_MIN_X10          0       // 最低设定温度：0.0℃
#define PANEL_TEMP_MAX_X10          1100    // 最高设定温度：110.0℃

/*
 * 数码管允许显示的温度范围。
 * 这个范围可以比设定范围更宽。
 *
 * 例如仪器显示范围可能是 -10.0℃ 到 120.0℃，
 * 但是用户允许设定的范围是 0.0℃ 到 110.0℃。
 */
#define PANEL_DISPLAY_MIN_X10       (-100)  // -10.0℃
#define PANEL_DISPLAY_MAX_X10       1200    // 120.0℃

/*
 * 外部测温通信超时时间。
 *
 * 因为你说温度是通过外部通信获取的，
 * 所以如果超过 3000 ms 没有收到某个恒温池的新温度，
 * 就认为测温通信异常，进入 E132 错误。
 */
#define PANEL_SENSOR_TIMEOUT_MS     3000

/*
 * Normal 模式下，当前温度和设定温度自动切换显示的时间。
 *
 * 例如：
 * 前 3 秒显示当前温度
 * 后 3 秒显示设定温度
 * 再后 3 秒显示当前温度
 */
#define PANEL_DISPLAY_SWITCH_MS     3000

/*
 * 设定温度编辑超时时间。
 *
 * 比如你按了“调大”修改设定温度，
 * 数码管会临时显示设定温度。
 * 如果 5 秒内没有继续按键，就自动回到当前温度显示。
 */
#define PANEL_EDIT_TIMEOUT_MS       5000

/*
 * 普通恒温 Jump 模式下的升降温速度。
 *
 * 原仪器的 Temperature Jump Mode 中，设定温度改变后，
 * 温度不是无限快跳变，而是按固定速率变化。
 *
 * 这里设置为 5.0℃/min。
 * 因为单位是 0.1℃/min，所以 5.0℃/min = 50。
 */
#define PANEL_JUMP_RAMP_X10_PER_MIN 50


/* ============================================================
 * 2. 按键定义
 * ============================================================ */

/*
 * 面板按键编号。
 *
 * 这些名字和你的原理图中的按键对应：
 *
 * PANEL_KEY_MODE        模式键
 * PANEL_KEY_START_STOP  开始/结束键
 * PANEL_KEY_UP          调大键
 * PANEL_KEY_DOWN        调小键
 * PANEL_KEY_ENTER       确认键
 * PANEL_KEY_SWITCH      切换键，切换 1 号/2 号恒温池
 */
typedef enum
{
    PANEL_KEY_MODE = 0,
    PANEL_KEY_START_STOP,
    PANEL_KEY_UP,
    PANEL_KEY_DOWN,
    PANEL_KEY_ENTER,
    PANEL_KEY_SWITCH,
    PANEL_KEY_NONE
} PanelKey_t;


/*
 * 按键事件类型。
 *
 * 同一个按键可能有三种操作方式：
 *
 * SHORT  ：短按一次
 * REPEAT ：长按后自动连发
 * LONG   ：长按事件
 *
 * 例如：
 * 调大键短按：+0.1℃
 * 调大键连发：快速增加
 * 调大键长按：更快增加
 */
typedef enum
{
    PANEL_KEY_EVT_SHORT = 0,
    PANEL_KEY_EVT_REPEAT,
    PANEL_KEY_EVT_LONG
} PanelKeyEvent_t;


/* ============================================================
 * 3. 面板模式定义
 * ============================================================ */

/*
 * 面板的工作模式。
 *
 * PANEL_MODE_NORMAL：
 * 普通恒温模式。
 * 用于直接设置目标温度，然后开始控温。
 *
 * PANEL_MODE_PARAM_SET：
 * 参数设置模式。
 * 用于设置温度程序，例如起始温度、保持时间、升降温速度等。
 *
 * PANEL_MODE_EXTERNAL：
 * 外部控制模式。
 * 如果以后你想通过上位机或通信命令控制目标温度，可以使用这个模式。
 */
typedef enum
{
    PANEL_MODE_NORMAL = 0,
    PANEL_MODE_PARAM_SET,
    PANEL_MODE_EXTERNAL
} PanelMode_t;


/* ============================================================
 * 4. 单个恒温池运行状态
 * ============================================================ */

/*
 * 单个恒温池的运行状态。
 *
 * CELL_STOP：
 * 停止控温，PID 和 H 桥关闭。
 *
 * CELL_RUN_JUMP：
 * 普通恒温模式运行。
 * 目标温度由面板调大/调小设置。
 *
 * CELL_RUN_PROGRAM：
 * 程序控温模式运行。
 * 按起始温度、保持时间、升降温速度、下一温度等参数自动执行。
 *
 * CELL_RUN_EXTERNAL：
 * 外部控制模式运行。
 * 目标温度由外部通信更新。
 */
typedef enum
{
    CELL_STOP = 0,
    CELL_RUN_JUMP,
    CELL_RUN_PROGRAM,
    CELL_RUN_EXTERNAL
} CellRunMode_t;


/* ============================================================
 * 5. 数码管当前显示内容类型
 * ============================================================ */

/*
 * 面板当前正在显示什么。
 *
 * PANEL_SHOW_CURRENT：
 * 显示当前温度。
 *
 * PANEL_SHOW_TARGET：
 * 显示设定温度。
 *
 * PANEL_SHOW_PARAM：
 * 显示程序参数。
 *
 * PANEL_SHOW_ERROR：
 * 显示错误代码，例如 E1、E3、E132。
 */
typedef enum
{
    PANEL_SHOW_CURRENT = 0,
    PANEL_SHOW_TARGET,
    PANEL_SHOW_PARAM,
    PANEL_SHOW_ERROR
} PanelShowType_t;


/* ============================================================
 * 6. 错误代码定义
 * ============================================================ */

/*
 * 错误代码。
 *
 * PANEL_ERR_NONE：
 * 没有错误。
 *
 * PANEL_ERR_E1_WATER：
 * 冷却水异常。
 * 原仪器中 E1 与保护帕尔贴的循环水有关。
 *
 * PANEL_ERR_E3_PELTIER：
 * 帕尔贴加热/制冷异常。
 *
 * PANEL_ERR_E132_SENSOR：
 * 找不到温度传感器。
 * 对你来说，也可以理解成“外部测温通信超时”。
 */
typedef enum
{
    PANEL_ERR_NONE = 0,
    PANEL_ERR_E1_WATER = 1,
    PANEL_ERR_E3_PELTIER = 3,
    PANEL_ERR_E132_SENSOR = 132
} PanelError_t;


/* ============================================================
 * 7. 程序控温参数编号
 * ============================================================ */

/*
 * 程序控温模式下，一共有 6 个参数。
 *
 * 你按“确认”键时，会在这些参数之间切换。
 */
typedef enum
{
    PROG_PARAM_START_TEMP = 0,     // 起始温度
    PROG_PARAM_START_HOLD,         // 起始温度保持时间
    PROG_PARAM_RAMP_RATE,          // 升降温速率
    PROG_PARAM_NEXT_TEMP,          // 下一目标温度
    PROG_PARAM_WAIT_TIME,          // 控制点等待时间
    PROG_PARAM_REPEAT_TIMES,       // 重复次数
    PROG_PARAM_COUNT               // 参数数量，不是具体参数
} ProgramParamIndex_t;


/* ============================================================
 * 8. 程序控温参数结构体
 * ============================================================ */

/*
 * 一个完整的温度程序。
 *
 * 举例：
 *
 * start_temp_x10     = 100   表示 10.0℃
 * start_hold_s       = 300   表示起始温度保持 300 秒
 * ramp_x10_per_min   = 10    表示 1.0℃/min
 * next_temp_x10      = 110   表示下一目标温度 11.0℃
 * wait_s             = 30    表示每到一个温度点等待 30 秒
 * repeat_times       = 79    表示重复 79 次
 *
 * 这样可以实现：
 * 10℃ → 11℃ → 12℃ → ... → 90℃
 */
typedef struct
{
    int16_t start_temp_x10;       // 起始温度，单位 0.1℃
    uint16_t start_hold_s;        // 起始温度保持时间，单位 s
    int16_t ramp_x10_per_min;     // 升降温速率，单位 0.1℃/min
    int16_t next_temp_x10;        // 下一目标温度，单位 0.1℃
    uint16_t wait_s;              // 到达每个温度点后的等待时间，单位 s
    uint16_t repeat_times;        // 重复次数
} TempProgram_t;


/* ============================================================
 * 9. 单个恒温池的数据结构
 * ============================================================ */

/*
 * 每一个恒温池都有一份这样的数据。
 *
 * 你的系统有两个恒温池，所以后面会定义：
 *
 * TempCell_t cell[2];
 */
typedef struct
{
    /*
     * 当前温度。
     *
     * 这个值不是本代码测出来的，
     * 而是你通过外部通信收到温度后，
     * 调用 TempPanel_UpdateMeasuredTemp() 更新进来的。
     */
    int16_t current_temp_x10;

    /*
     * 用户设定的目标温度。
     *
     * Normal 模式下，按“调大/调小”修改的就是这个值。
     */
    int16_t target_temp_x10;

    /*
     * 实际发送给 PID 的目标温度。
     *
     * 为什么不直接把 target_temp_x10 送给 PID？
     *
     * 因为如果目标温度从 25℃ 突然改成 80℃，
     * PID 目标值瞬间跳变太大，可能导致输出过猛。
     *
     * 所以这里用 command_temp_x10 作为“缓慢变化的目标温度”。
     * 例如按 5℃/min 慢慢接近 target_temp_x10。
     */
    int16_t command_temp_x10;

    /*
     * 上一次收到该恒温池温度的时间。
     *
     * 单位 ms，一般由 HAL_GetTick() 提供。
     *
     * 用途：
     * 判断测温通信是否超时。
     */
    uint32_t last_temp_update_ms;

    /*
     * 当前恒温池的运行状态。
     */
    CellRunMode_t run_mode;

    /*
     * 当前恒温池的错误状态。
     */
    PanelError_t error;

    /*
     * 当前恒温池的程序控温参数。
     */
    TempProgram_t program;

    /*
     * 程序控温内部状态。
     *
     * 这些变量不需要你手动修改，
     * 是代码在执行程序控温时自动使用的。
     */
    uint8_t program_phase;              // 当前执行到程序的哪一个阶段
    uint16_t program_timer_s;           // 当前阶段已经计时多少秒
    uint16_t program_interval_done;     // 已经完成多少个温度间隔
    int16_t program_step_x10;           // 每次温度变化的步长
    int16_t program_next_target_x10;    // 当前程序阶段的下一个目标温度

    /*
     * PID 是否使能。
     *
     * 这个变量只是逻辑标志。
     * 真正开关 PID 和 H 桥，需要你在 Control_StartPid()
     * 和 Control_StopPid() 里面实现。
     */
    bool pid_enabled;

} TempCell_t;


/* ============================================================
 * 10. 整个面板系统的数据结构
 * ============================================================ */

/*
 * 面板总对象。
 *
 * 这个结构体保存了：
 * 1. 当前面板模式
 * 2. 当前选择的是 1 号池还是 2 号池
 * 3. 当前显示什么
 * 4. 两个恒温池的数据
 */
typedef struct
{
    /*
     * 当前面板模式：
     * Normal / 参数设置 / 外部控制
     */
    PanelMode_t mode;

    /*
     * 当前正在操作哪个恒温池。
     *
     * 0 表示 1 号恒温池
     * 1 表示 2 号恒温池
     */
    uint8_t active_cell;

    /*
     * 当前数码管显示内容类型。
     */
    PanelShowType_t show_type;

    /*
     * 参数设置模式下，当前正在设置第几个参数。
     */
    ProgramParamIndex_t param_index;

    /*
     * 是否正在编辑设定温度。
     *
     * 例如在 Normal 模式下按了“调大/调小”，
     * editing 会变成 true。
     *
     * 这时数码管固定显示设定温度，
     * 不再自动切换到当前温度。
     */
    bool editing;

    /*
     * 上次编辑时间。
     *
     * 用于判断 5 秒没有按键后，退出编辑显示。
     */
    uint32_t edit_tick_ms;

    /*
     * 上次数码管自动切换显示的时间。
     *
     * Normal 模式下，当前温度和设定温度每隔几秒切换一次。
     */
    uint32_t display_tick_ms;

    /*
     * 1 秒节拍。
     *
     * 程序控温中，保持时间、等待时间都以秒为单位，
     * 所以这里需要一个 1 秒计时。
     */
    uint32_t second_tick_ms;

    /*
     * 两个恒温池的数据。
     */
    TempCell_t cell[PANEL_CELL_NUM];

} TempPanel_t;


/* ============================================================
 * 11. 对外函数声明
 * ============================================================ */

/*
 * 初始化面板逻辑。
 *
 * 开机时调用一次。
 */
void TempPanel_Init(TempPanel_t *p);

/*
 * 面板周期任务。
 *
 * 需要在 while(1) 或 FreeRTOS 任务中周期调用。
 *
 * 它会完成：
 * 1. 检查测温通信是否超时
 * 2. 更新 PID 目标温度
 * 3. 程序控温状态推进
 * 4. 刷新显示
 */
void TempPanel_Task(TempPanel_t *p, uint32_t now_ms);

/*
 * 按键事件处理函数。
 *
 * 当你的 TM1638 驱动检测到按键后，
 * 就调用这个函数。
 */
void TempPanel_KeyEvent(TempPanel_t *p,
                        PanelKey_t key,
                        PanelKeyEvent_t evt,
                        uint32_t now_ms);

/*
 * 外部通信收到温度后调用。
 *
 * 例如收到 1 号池温度为 25.3℃：
 *
 * TempPanel_UpdateMeasuredTemp(&g_panel, 0, 253, HAL_GetTick());
 */
void TempPanel_UpdateMeasuredTemp(TempPanel_t *p,
                                  uint8_t cell,
                                  int16_t temp_x10,
                                  uint32_t now_ms);

/*
 * 设置或清除冷却水错误。
 *
 * 因为冷却水一般同时影响两个恒温池，
 * 所以这个函数会同时处理两个池。
 */
void TempPanel_SetWaterError(TempPanel_t *p, bool error);

/*
 * 设置或清除某个恒温池的帕尔贴错误。
 */
void TempPanel_SetPeltierError(TempPanel_t *p,
                               uint8_t cell,
                               bool error);

#ifdef __cplusplus
}
#endif

#endif