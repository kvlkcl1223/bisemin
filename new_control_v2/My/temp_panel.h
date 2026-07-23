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
     * 1. 绯荤粺鍩虹閰嶇疆
     * ============================================================ */

#define PANEL_CELL_NUM 2

/* 娓╁害鑼冨洿: 鐩存帴浣跨敤 float 鎽勬皬锟?*/
#define PANEL_TEMP_MIN -10.0f
#define PANEL_TEMP_MAX 110.0f

#define PANEL_RAMP_RATE_MIN 0.1f
#define PANEL_RAMP_RATE_MAX 60.0f

/* 澶栭儴娴嬫俯閫氫俊瓒呮椂鏃堕棿 (ms) */
#define PANEL_SENSOR_TIMEOUT_MS 3000

/* Normal 妯″紡 褰撳墠/璁惧畾娓╁害鑷姩鍒囨崲闂撮殧 (ms) */
#define PANEL_DISPLAY_SWITCH_MS 3000

/* 缂栬緫瓒呮椂 (5绉掓棤鎸夐敭鑷姩閫€鍑虹紪锟? */
#define PANEL_EDIT_TIMEOUT_MS 5000

/* Jump 妯″紡鍗囬檷娓╅€熷害: 5.0锟?min */
#define PANEL_JUMP_RAMP_PER_MIN 100.0f

    /* ============================================================
     * 2. 鎸夐敭瀹氫箟
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
     * 3. 闈㈡澘妯″紡瀹氫箟
     * ============================================================ */

    typedef enum
    {
        PANEL_MODE_NORMAL = 0,
        PANEL_MODE_PARAM_SET,
        PANEL_MODE_EXTERNAL
    } PanelMode_t;

    /* ============================================================
     * 4. 鍗曚釜鎭掓俯姹犺繍琛岀姸锟?
     * ============================================================ */

    typedef enum
    {
        CELL_STOP = 0,
        CELL_RUN_JUMP,
        CELL_RUN_PROGRAM,
        CELL_RUN_EXTERNAL
    } CellRunMode_t;

    /* ============================================================
     * 5. 鏁扮爜绠″綋鍓嶆樉绀哄唴瀹圭被锟?
     * ============================================================ */

    typedef enum
    {
        PANEL_SHOW_CURRENT = 0,
        PANEL_SHOW_TARGET,
        PANEL_SHOW_PARAM,
        PANEL_SHOW_ERROR
    } PanelShowType_t;

    /* ============================================================
     * 6. 閿欒浠ｇ爜瀹氫箟
     * ============================================================ */

    typedef enum
    {
        PANEL_ERR_NONE = 0,

        /* E001: reserved water/cooling fault. Stops both cells when used. */
        PANEL_ERR_E1_WATER = 1,

        /* E003: legacy per-cell peltier fault. Kept for compatibility. */
        PANEL_ERR_E3_PELTIER = 3,

        /* E121: temperature sensor CH1 fault.
         * CH1 is the first temperature input and belongs to cell 1. The
         * controller pulses NRST_OTHER several times first. If CH1 still has
         * no valid fresh data, cell 1 is stopped and E121 is displayed.
         */
        PANEL_ERR_E121_TEMP_CH1 = 121,

        /* E122: temperature sensor CH2 fault.
         * CH2 is the second temperature input and belongs to cell 1. The
         * controller pulses NRST_OTHER several times first. If CH2 still has
         * no valid fresh data, cell 1 is stopped and E122 is displayed.
         */
        PANEL_ERR_E122_TEMP_CH2 = 122,

        /* E123: temperature sensor CH3 fault.
         * CH3 is the first temperature input belonging to cell 2. The
         * controller pulses NRST_OTHER several times first. If CH3 still has
         * no valid fresh data, cell 2 is stopped and E123 is displayed.
         */
        PANEL_ERR_E123_TEMP_CH3 = 123,

        /* E124: temperature sensor CH4 fault.
         * CH4 is the second temperature input belonging to cell 2. The
         * controller pulses NRST_OTHER several times first. If CH4 still has
         * no valid fresh data, cell 2 is stopped and E124 is displayed.
         */
        PANEL_ERR_E124_TEMP_CH4 = 124,

        /* E132: legacy/generic temperature sensor fault.
         * Kept for compatibility with older panel code and for unknown
         * temperature faults. New control logic reports E121-E124 so the
         * failed sensor route can be identified directly.
         */
        PANEL_ERR_E132_SENSOR = 132,

        /* E301: cell 1 DRV8703 supply voltage fault.
         * One of the DRV8703 supply voltage ADC channels belonging to cell 1
         * is significantly below the 24 V rail threshold, so cell 1 is stopped.
         */
        PANEL_ERR_E301_CELL1_VOLTAGE = 301,

        /* E302: cell 2 DRV8703 supply voltage fault.
         * One of the DRV8703 supply voltage ADC channels belonging to cell 2
         * is significantly below the 24 V rail threshold, so cell 2 is stopped.
         */
        PANEL_ERR_E302_CELL2_VOLTAGE = 302,

        /* E305: shared CH5 DRV8703 supply voltage fault.
         * CH5 is shared by both temperature cells. If its supply is bad,
         * both cell 1 and cell 2 are stopped.
         */
        PANEL_ERR_E305_SHARED_VOLTAGE = 305,

        /* E311: cell 1 DRV8703 communication/fault-pin/register fault.
         * The controller retries DRV8703 startup several times before latching
         * this error. Cell 1 is stopped when this error is active.
         */
        PANEL_ERR_E311_CELL1_DRV = 311,

        /* E312: cell 2 DRV8703 communication/fault-pin/register fault.
         * The controller retries DRV8703 startup several times before latching
         * this error. Cell 2 is stopped when this error is active.
         */
        PANEL_ERR_E312_CELL2_DRV = 312,

        /* E315: shared CH5 DRV8703 fault.
         * CH5 is needed whenever either cell runs. If CH5 fails to initialize
         * or reports a fault, both cell 1 and cell 2 are stopped.
         */
        PANEL_ERR_E315_SHARED_DRV = 315
    } PanelError_t;

    typedef enum
    {
        PANEL_UI_ERR_NONE = 0,
        PANEL_UI_ERR_MODE_LOCKED = 201,         /* E201: selected cell is running, MODE is locked */
        PANEL_UI_ERR_PARAM_EDIT_RUNNING = 202,  /* E202: param mode is running, UP/DOWN ignored */
        PANEL_UI_ERR_PARAM_ENTER_RUNNING = 203, /* E203: param mode is running, ENTER ignored */
        PANEL_UI_ERR_NORMAL_EDIT_RUNNING = 204, /* E204: normal mode is running, UP/DOWN ignored */
        PANEL_UI_ERR_NORMAL_ENTER_RUNNING = 205 /* E205: normal mode is running, ENTER ignored */
    } PanelUiError_t;

    /* ============================================================
     * 7. 绋嬪簭鎺ф俯鍙傛暟缂栧彿
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
     * 8. 绋嬪簭鎺ф俯鍙傛暟缁撴瀯锟?(娓╁害鐩存帴锟?float)
     * ============================================================ */

    typedef struct
    {
        float start_temp;      // 璧峰娓╁害 (锟?
        uint16_t start_hold_s; // 璧峰娓╁害淇濇寔鏃堕棿 (锟?
        float ramp_rate;       // 鍗囬檷娓╅€熺巼 (锟?min)
        float next_temp;       // 涓嬩竴鐩爣娓╁害 (锟?
        uint16_t wait_s;       // 鍒拌揪姣忎釜娓╁害鐐瑰悗绛夊緟鏃堕棿 (锟?
        uint16_t repeat_times; // 閲嶅娆℃暟
    } TempProgram_t;

    /* ============================================================
     * 9. 鍗曚釜鎭掓俯姹犵殑鏁版嵁缁撴瀯
     * ============================================================ */

    typedef struct
    {
        float current_temp; // 褰撳墠瀹炴祴娓╁害 (锟?
        float target_temp;  // 鐢ㄦ埛璁惧畾鐨勭洰鏍囨俯锟?(锟?
        float command_temp; // 甯︽枩鍧￠檺鍒跺悗鍙戠粰 PID 鐨勭洰鏍囨俯锟?(锟?

        uint32_t last_temp_update_ms;

        CellRunMode_t run_mode;
        PanelError_t error;

        TempProgram_t program;

        /* 绋嬪簭鎺ф俯鍐呴儴鐘讹拷?*/
        uint8_t program_phase;
        uint16_t program_timer_s;
        uint16_t program_interval_done;
        float program_step;
        float program_next_target;

        bool pid_enabled;

    } TempCell_t;

    /* ============================================================
     * 10. 鏁翠釜闈㈡澘绯荤粺鐨勬暟鎹粨锟?
     * ============================================================ */

    typedef struct
    {
        PanelMode_t mode;
        uint8_t active_cell;
        PanelMode_t cell_mode[PANEL_CELL_NUM];
        PanelShowType_t show_type;
        ProgramParamIndex_t param_index;
        bool editing;
        uint32_t edit_tick_ms;
        uint32_t display_tick_ms;
        uint32_t second_tick_ms;
        uint8_t param_show_cnt;
        uint32_t param_inactive_tick_ms;
        PanelUiError_t ui_error;
        uint32_t ui_error_until_ms;
        TempCell_t cell[PANEL_CELL_NUM];
    } TempPanel_t;

    /* ============================================================
     * 11. 瀵瑰鍑芥暟澹版槑
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
    void TempPanel_SetCellError(TempPanel_t *p,
                                uint8_t cell,
                                PanelError_t err);

    void TempPanel_Stop(TempPanel_t *p, uint8_t cell);
    uint8_t TempPanel_StartNormal(TempPanel_t *p,
                                  uint8_t cell,
                                  float target_temp);
    uint8_t TempPanel_SetProgram(TempPanel_t *p,
                                 uint8_t cell,
                                 const TempProgram_t *program);
    uint8_t TempPanel_StartProgram(TempPanel_t *p,
                                   uint8_t cell);

    PanelKey_t PanelKey_FromTM1638(TM1638_Key_t key);

    extern TempPanel_t g_panel;

    void Panel_Init(void);
    void CheckKeyHoldEvents(void);

#ifdef __cplusplus
}
#endif

#endif

