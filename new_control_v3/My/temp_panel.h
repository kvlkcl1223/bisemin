#ifndef TEMP_PANEL_H
#define TEMP_PANEL_H

#include <stdint.h>
#include <stdbool.h>
#include "tm1638_board.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /* ============================================================
     * 1. 系统基础配置
     * ============================================================ */

/** @brief 面板支持的温控池数量。当前为 Cell 0 和 Cell 1。 */
#define PANEL_CELL_NUM 2

/** @brief 是否启用低温虚拟显示。真实温度低于传感器下限后可继续显示估算值。 */
#define TEMP_VIRTUAL_LOW_ENABLE 1U
/** @brief 真实测温输入的最低可信温度，单位 degC。 */
#define PANEL_TEMP_REAL_MIN -10.0f
/** @brief 面板允许显示的最低温度，单位 degC。 */
#define PANEL_TEMP_DISPLAY_MIN -20.0f
/** @brief 面板允许设置/显示的最高温度，单位 degC。 */
#define PANEL_TEMP_MAX 110.0f
/** @brief 面板目标温度下限。当前使用显示下限，允许虚拟低温目标。 */
#define PANEL_TEMP_MIN PANEL_TEMP_DISPLAY_MIN

/** @brief 程序控温升降温速率最小值，单位 degC/min。 */
#define PANEL_RAMP_RATE_MIN 0.1f
/** @brief 程序控温升降温速率最大值，单位 degC/min。 */
#define PANEL_RAMP_RATE_MAX 60.0f

/** @brief 历史面板侧测温超时时间，当前测温超时由 app_control 负责。 */
#define PANEL_SENSOR_TIMEOUT_MS 3000

/** @brief 运行时当前温度/目标温度自动切换显示间隔，单位 ms。 */
#define PANEL_DISPLAY_SWITCH_MS 3000

/** @brief 编辑模式无按键自动退出时间，单位 ms。 */
#define PANEL_EDIT_TIMEOUT_MS 5000

/** @brief Normal/Jump 模式目标温度跳变时的命令温度斜率，单位 degC/min。 */
#define PANEL_JUMP_RAMP_PER_MIN 100.0f

    /* ============================================================
     * 2. 按键定义
     * ============================================================ */

    /** @brief 面板抽象按键编号，由 TM1638 物理按键映射得到。 */
    typedef enum
    {
        PANEL_KEY_MODE = 0,     /**< 模式切换键。 */
        PANEL_KEY_START_STOP,   /**< 启动/停止键。 */
        PANEL_KEY_UP,           /**< 增加键。 */
        PANEL_KEY_DOWN,         /**< 减少键。 */
        PANEL_KEY_ENTER,        /**< 确认/切换参数键。 */
        PANEL_KEY_SWITCH,       /**< 切换当前 cell 键。 */
        PANEL_KEY_NONE          /**< 无有效按键。 */
    } PanelKey_t;

    /** @brief 按键事件类型。 */
    typedef enum
    {
        PANEL_KEY_EVT_SHORT = 0, /**< 短按。 */
        PANEL_KEY_EVT_REPEAT,    /**< 长按期间的重复事件。 */
        PANEL_KEY_EVT_LONG       /**< 长按事件。 */
    } PanelKeyEvent_t;

    /* ============================================================
     * 3. 面板模式定义
     * ============================================================ */

    /** @brief 面板当前工作模式。 */
    typedef enum
    {
        PANEL_MODE_NORMAL = 0,    /**< 普通定点控温模式。 */
        PANEL_MODE_PARAM_SET,     /**< 程序控温参数设置/程序运行模式。 */
        PANEL_MODE_EXTERNAL       /**< 外部/上位机模式，当前保留。 */
    } PanelMode_t;

    /* ============================================================
     * 4. 单个温控池运行状态
     * ============================================================ */

    /** @brief 单个 cell 当前运行模式。 */
    typedef enum
    {
        CELL_STOP = 0,      /**< 停止。 */
        CELL_RUN_JUMP,     /**< 普通模式运行，目标温度直接跳转或按内部斜率到达。 */
        CELL_RUN_PROGRAM,  /**< 程序控温运行。 */
        CELL_RUN_EXTERNAL  /**< 外部控制运行，当前保留。 */
    } CellRunMode_t;

    /* ============================================================
     * 5. 数码管当前显示内容类型
     * ============================================================ */

    /** @brief 面板当前显示内容类型。 */
    typedef enum
    {
        PANEL_SHOW_CURRENT = 0, /**< 显示当前测得温度。 */
        PANEL_SHOW_TARGET,      /**< 显示目标/命令温度。 */
        PANEL_SHOW_PARAM,       /**< 显示程序参数名或参数值。 */
        PANEL_SHOW_ERROR        /**< 显示错误码。 */
    } PanelShowType_t;

    /* ============================================================
     * 6. 错误代码定义
     * ============================================================ */

    /** @brief 面板和控制层共用错误码。显示格式为 E + 数字。 */
    typedef enum
    {
        PANEL_ERR_NONE = 0, /**< 无错误。 */

        PANEL_ERR_E1_WATER = 1, /**< E001：水路/制冷系统保留故障，使用时停止两个 cell。 */

        PANEL_ERR_E3_PELTIER = 3, /**< E003：旧版单 cell 冷热片通用故障，保留兼容。 */

        PANEL_ERR_E121_TEMP_CH1 = 121, /**< E121：测温 CH1 故障，默认属于 Cell 0 外层测温。 */
        PANEL_ERR_E122_TEMP_CH2 = 122, /**< E122：测温 CH2 故障，默认属于 Cell 0 内层测温。 */
        PANEL_ERR_E123_TEMP_CH3 = 123, /**< E123：测温 CH3 故障，默认属于 Cell 1 外层测温。 */
        PANEL_ERR_E124_TEMP_CH4 = 124, /**< E124：测温 CH4 故障，默认属于 Cell 1 内层测温。 */

        PANEL_ERR_E132_SENSOR = 132, /**< E132：旧版/未知测温传感器通用故障，保留兼容。 */

        PANEL_ERR_E301_CELL1_VOLTAGE = 301, /**< E301：Cell 0 对应 DRV 供电电压异常。 */
        PANEL_ERR_E302_CELL2_VOLTAGE = 302, /**< E302：Cell 1 对应 DRV 供电电压异常。 */
        PANEL_ERR_E305_SHARED_VOLTAGE = 305, /**< E305：共享 DRV5 供电异常，影响两个 cell。 */

        PANEL_ERR_E311_CELL1_DRV = 311, /**< E311：Cell 0 外层/内层 DRV8703 通信或故障脚异常。 */
        PANEL_ERR_E312_CELL2_DRV = 312, /**< E312：Cell 1 外层/内层 DRV8703 通信或故障脚异常。 */
        PANEL_ERR_E315_SHARED_DRV = 315 /**< E315：共享 DRV5 初始化或运行故障。 */
    } PanelError_t;

    /** @brief 仅由面板交互产生的临时 UI 错误码。 */
    typedef enum
    {
        PANEL_UI_ERR_NONE = 0,                    /**< 无 UI 错误。 */
        PANEL_UI_ERR_MODE_LOCKED = 201,           /**< E201：当前 cell 正在运行，禁止切换模式。 */
        PANEL_UI_ERR_PARAM_EDIT_RUNNING = 202,    /**< E202：程序运行中，禁止 UP/DOWN 修改程序参数。 */
        PANEL_UI_ERR_PARAM_ENTER_RUNNING = 203,   /**< E203：程序运行中，禁止 ENTER 切换参数项。 */
        PANEL_UI_ERR_NORMAL_EDIT_RUNNING = 204,   /**< E204：普通模式运行中，禁止 UP/DOWN 修改目标温度。 */
        PANEL_UI_ERR_NORMAL_ENTER_RUNNING = 205    /**< E205：普通模式运行中，ENTER 无效。 */
    } PanelUiError_t;

    /* ============================================================
     * 7. 程序控温参数编号
     * ============================================================ */

    /** @brief 程序控温参数编辑项索引，ENTER 键按该顺序循环。 */
    typedef enum
    {
        PROG_PARAM_START_TEMP = 0, /**< 起始温度。 */
        PROG_PARAM_START_HOLD,     /**< 起始温度保持时间，单位 s。 */
        PROG_PARAM_RAMP_RATE,      /**< 升降温速率，单位 degC/min。 */
        PROG_PARAM_NEXT_TEMP,      /**< 下一目标温度。 */
        PROG_PARAM_WAIT_TIME,      /**< 到达每个目标温度后的等待时间，单位 s。 */
        PROG_PARAM_REPEAT_TIMES,   /**< 后续重复次数。 */
        PROG_PARAM_COUNT           /**< 参数数量。 */
    } ProgramParamIndex_t;

    /* ============================================================
     * 8. 程序控温参数结构
     * ============================================================ */

    /** @brief 程序控温参数。温度直接使用 float，时间使用秒。 */
    typedef struct
    {
        float start_temp;      /**< 起始温度，单位 degC。 */
        uint16_t start_hold_s; /**< 起始温度保持时间，单位 s。 */
        float ramp_rate;       /**< 从当前目标到下一目标的升降温速率，单位 degC/min。 */
        float next_temp;       /**< 第一个下一目标温度，单位 degC。 */
        uint16_t wait_s;       /**< 每次到达目标温度后的等待时间，单位 s。 */
        uint16_t repeat_times; /**< 继续按相同温差重复的次数。 */
    } TempProgram_t;

    /* ============================================================
     * 9. 单个温控池的数据结构
     * ============================================================ */

    /** @brief 单个 cell 的面板侧状态。 */
    typedef struct
    {
        float current_temp; /**< 当前测得/显示温度，来自 app_control 更新。 */
        float target_temp;  /**< 用户设定目标温度。 */
        float command_temp; /**< 实际发给控制层的命令温度，程序控温和斜率限制会更新它。 */

        uint32_t last_temp_update_ms; /**< 最近一次温度更新时间。 */

        CellRunMode_t run_mode; /**< 当前 cell 运行模式。 */
        PanelError_t error;     /**< 当前 cell 错误码。 */

        TempProgram_t program; /**< 程序控温参数。 */

        uint8_t program_phase;         /**< 程序控温内部阶段。 */
        uint16_t program_timer_s;      /**< 当前程序阶段已计时秒数。 */
        uint16_t program_interval_done;/**< 已完成的重复区间数。 */
        float program_step;            /**< 每次重复时目标温度增加/减少的步长。 */
        float program_next_target;     /**< 程序控温当前阶段要逼近的目标温度。 */

        bool pid_enabled; /**< 面板侧 PID 运行标志；真实 PID 由 app_control 执行。 */

    } TempCell_t;

    /* ============================================================
     * 10. 整个面板系统的数据结构
     * ============================================================ */

    /** @brief 面板模块完整运行状态。 */
    typedef struct
    {
        PanelMode_t mode;                         /**< 当前显示/操作模式。 */
        uint8_t active_cell;                      /**< 当前面板正在操作的 cell。 */
        PanelMode_t cell_mode[PANEL_CELL_NUM];    /**< 每个 cell 记忆的模式，切换 cell 时恢复。 */
        PanelShowType_t show_type;                /**< 当前显示内容类型。 */
        ProgramParamIndex_t param_index;          /**< 当前正在查看/编辑的程序参数项。 */
        bool editing;                             /**< 是否处于编辑状态。 */
        uint32_t edit_tick_ms;                    /**< 最近一次编辑按键时间。 */
        uint32_t display_tick_ms;                 /**< 自动切换显示用计时。 */
        uint32_t second_tick_ms;                  /**< 1 秒节拍计时，用于程序控温。 */
        uint8_t param_show_cnt;                   /**< 参数名短暂显示计数。 */
        uint32_t param_inactive_tick_ms;          /**< 参数模式最近活动时间，保留。 */
        PanelUiError_t ui_error;                  /**< 当前临时 UI 错误。 */
        uint32_t ui_error_until_ms;               /**< 临时 UI 错误显示截止时间。 */
        TempCell_t cell[PANEL_CELL_NUM];          /**< 两个 cell 的面板侧状态。 */
    } TempPanel_t;

    /* ============================================================
     * 11. 对外函数声明
     * ============================================================ */

    /** @brief 初始化面板状态结构，设置默认目标温度和程序参数。 */
    void TempPanel_Init(TempPanel_t *p);
    /** @brief 面板周期任务，处理显示刷新、程序控温节拍、编辑超时和临时 UI 错误。 */
    void TempPanel_Task(TempPanel_t *p, uint32_t now_ms);
    /** @brief 输入一个面板按键事件，更新模式、参数、启停状态。 */
    void TempPanel_KeyEvent(TempPanel_t *p,
                            PanelKey_t key,
                            PanelKeyEvent_t evt,
                            uint32_t now_ms);
    /** @brief 更新指定 cell 的当前测温显示值。 */
    void TempPanel_UpdateMeasuredTemp(TempPanel_t *p,
                                      uint8_t cell,
                                      float temp,
                                      uint32_t now_ms);
    /** @brief 设置或清除水路/公共故障；置位时停止两个 cell。 */
    void TempPanel_SetWaterError(TempPanel_t *p, bool error);
    /** @brief 设置或清除旧版冷热片故障；置位时停止指定 cell。 */
    void TempPanel_SetPeltierError(TempPanel_t *p,
                                   uint8_t cell,
                                   bool error);
    /** @brief 设置指定 cell 的控制层错误码；非 NONE 时紧急停止该 cell。 */
    void TempPanel_SetCellError(TempPanel_t *p,
                                uint8_t cell,
                                PanelError_t err);

    /** @brief 面板侧请求正常停止指定 cell，并同步调用控制层停止。 */
    void TempPanel_Stop(TempPanel_t *p, uint8_t cell);
    /** @brief 从其他上下文请求面板停止指定 cell；实际处理在 TempPanel_ServiceRequests。 */
    void TempPanel_RequestStop(uint8_t cell);
    /** @brief 处理异步停止请求，只更新面板侧状态。 */
    void TempPanel_ServiceRequests(TempPanel_t *p);
    /** @brief 由上位机/外部逻辑启动普通定点控温。返回 1 表示启动成功。 */
    uint8_t TempPanel_StartNormal(TempPanel_t *p,
                                  uint8_t cell,
                                  float target_temp);
    /** @brief 设置指定 cell 的程序控温参数。返回 1 表示参数有效且写入成功。 */
    uint8_t TempPanel_SetProgram(TempPanel_t *p,
                                 uint8_t cell,
                                 const TempProgram_t *program);
    /** @brief 启动指定 cell 的程序控温。返回 1 表示启动成功。 */
    uint8_t TempPanel_StartProgram(TempPanel_t *p,
                                   uint8_t cell);

    /** @brief 将 TM1638 物理按键枚举转换为面板抽象按键。 */
    PanelKey_t PanelKey_FromTM1638(TM1638_Key_t key);

    /** @brief 全局面板状态实例。 */
    extern TempPanel_t g_panel;

    /** @brief 初始化 TM1638 硬件和面板状态。 */
    void Panel_Init(void);
    /** @brief 检查长按/重复按键事件，由主循环或按键扫描流程调用。 */
    void CheckKeyHoldEvents(void);

#ifdef __cplusplus
}
#endif

#endif