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

#define PANEL_CELL_NUM 2

/* 温度范围: 直接使用 float 摄氏度 */
#define PANEL_TEMP_MIN -10.0f
#define PANEL_TEMP_MAX 110.0f

/* 外部测温通信超时时间 (ms) */
#define PANEL_SENSOR_TIMEOUT_MS 3000

/* Normal 模式 当前/设定温度自动切换间隔 (ms) */
#define PANEL_DISPLAY_SWITCH_MS 3000

/* 编辑超时 (5秒无按键自动退出编辑) */
#define PANEL_EDIT_TIMEOUT_MS 5000

/* Jump 模式升降温速度: 5.0℃/min */
#define PANEL_JUMP_RAMP_PER_MIN 5.0f

    /* ============================================================
     * 2. 按键定义
     * ============================================================ */

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

    typedef enum
    {
        PANEL_KEY_EVT_SHORT = 0,
        PANEL_KEY_EVT_REPEAT,
        PANEL_KEY_EVT_LONG
    } PanelKeyEvent_t;

    /* ============================================================
     * 3. 面板模式定义
     * ============================================================ */

    typedef enum
    {
        PANEL_MODE_NORMAL = 0,
        PANEL_MODE_PARAM_SET,
        PANEL_MODE_EXTERNAL
    } PanelMode_t;

    /* ============================================================
     * 4. 单个恒温池运行状态
     * ============================================================ */

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

    typedef enum
    {
        PROG_PARAM_START_TEMP = 0,
        PROG_PARAM_START_HOLD,
        PROG_PARAM_RAMP_RATE,
        PROG_PARAM_NEXT_TEMP,
        PROG_PARAM_WAIT_TIME,
        PROG_PARAM_REPEAT_TIMES,
        PROG_PARAM_COUNT
    } ProgramParamIndex_t;

    /* ============================================================
     * 8. 程序控温参数结构体 (温度直接用 float)
     * ============================================================ */

    typedef struct
    {
        float start_temp;      // 起始温度 (℃)
        uint16_t start_hold_s; // 起始温度保持时间 (秒)
        float ramp_rate;       // 升降温速率 (℃/min)
        float next_temp;       // 下一目标温度 (℃)
        uint16_t wait_s;       // 到达每个温度点后等待时间 (秒)
        uint16_t repeat_times; // 重复次数
    } TempProgram_t;

    /* ============================================================
     * 9. 单个恒温池的数据结构
     * ============================================================ */

    typedef struct
    {
        float current_temp; // 当前实测温度 (℃)
        float target_temp;  // 用户设定的目标温度 (℃)
        float command_temp; // 带斜坡限制后发给 PID 的目标温度 (℃)

        uint32_t last_temp_update_ms;

        CellRunMode_t run_mode;
        PanelError_t error;

        TempProgram_t program;

        /* 程序控温内部状态 */
        uint8_t program_phase;
        uint16_t program_timer_s;
        uint16_t program_interval_done;
        float program_step;
        float program_next_target;

        bool pid_enabled;

    } TempCell_t;

    /* ============================================================
     * 10. 整个面板系统的数据结构
     * ============================================================ */

    typedef struct
    {
        PanelMode_t mode;
        uint8_t active_cell;
        PanelShowType_t show_type;
        ProgramParamIndex_t param_index;
        bool editing;
        uint32_t edit_tick_ms;
        uint32_t display_tick_ms;
        uint32_t second_tick_ms;
        uint8_t param_show_cnt;
        uint32_t param_inactive_tick_ms;
        TempCell_t cell[PANEL_CELL_NUM];
    } TempPanel_t;

    /* ============================================================
     * 11. 对外函数声明
     * ============================================================ */

    void TempPanel_Init(TempPanel_t *p);
    void TempPanel_Task(TempPanel_t *p, uint32_t now_ms);
    void TempPanel_KeyEvent(TempPanel_t *p,
                            PanelKey_t key,
                            PanelKeyEvent_t evt,
                            uint32_t now_ms);
    void TempPanel_UpdateMeasuredTemp(TempPanel_t *p,
                                      uint8_t cell,
                                      float temp,
                                      uint32_t now_ms);
    void TempPanel_SetWaterError(TempPanel_t *p, bool error);
    void TempPanel_SetPeltierError(TempPanel_t *p,
                                   uint8_t cell,
                                   bool error);

    PanelKey_t PanelKey_FromTM1638(TM1638_Key_t key);

    extern TempPanel_t g_panel;

    void Panel_Init(void);
    void CheckKeyHoldEvents(void);

#ifdef __cplusplus
}
#endif

#endif
