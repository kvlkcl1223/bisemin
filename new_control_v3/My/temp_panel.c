#include "temp_panel.h"
#include "main.h"
#include "pid_controller.h"

/* ============================================================
 * 0. 硬件/控制层弱函数前置声明
 * ============================================================ */
/** @brief 弱声明：显示温度，默认实现使用 TM1638_ShowFloat。 */
__attribute__((weak)) void PanelHW_DisplayTemp(float temp);
/** @brief 弱声明：显示整数参数，默认实现限制到数码管可显示范围。 */
__attribute__((weak)) void PanelHW_DisplayNumber(int16_t value);
/** @brief 弱声明：显示错误码，默认显示 E + 数字。 */
__attribute__((weak)) void PanelHW_DisplayError(PanelError_t err);
/** @brief 弱声明：设置指定 LED 亮灭。 */
__attribute__((weak)) void PanelHW_SetLed(uint8_t led_id, bool on);
/** @brief 弱声明：短闪一次，用于提示按键无效。 */
__attribute__((weak)) void PanelHW_BlinkOnce(void);
/** @brief 弱声明：请求控制层启动指定 cell 的 PID。真实实现位于 app_control。 */
__attribute__((weak)) void Control_StartPid(uint8_t cell);
/** @brief 弱声明：请求控制层正常停止指定 cell 的 PID。 */
__attribute__((weak)) void Control_StopPid(uint8_t cell);
/** @brief 弱声明：请求控制层紧急停止指定 cell。 */
__attribute__((weak)) void Control_EmergencyStopPid(uint8_t cell);
/** @brief 弱声明：设置控制层目标温度。 */
__attribute__((weak)) void Control_SetTargetTemp(uint8_t cell, float target);

/** @brief 异步停止请求位图，每一位对应一个 cell，由其他上下文置位。 */
static volatile uint8_t s_panel_stop_request_mask = 0U;

/* ============================================================
 * 1. 内部辅助函数
 * ============================================================ */

/** @brief 将 float 数值限制在指定上下限内。 */
static float clamp_f(float v, float min, float max)
{
    if (v < min)
        return min;
    if (v > max)
        return max;
    return v;
}

/** @brief 返回当前面板正在操作的 cell 编号。 */
static uint8_t active(TempPanel_t *p)
{
    return p->active_cell;
}

/** @brief 获取当前 active_cell 对应的 TempCell_t 指针。 */
static TempCell_t *cur_cell(TempPanel_t *p)
{
    return &p->cell[p->active_cell];
}

/** @brief 判断指定 cell 是否处于非停止状态。 */
static bool cell_is_running(const TempCell_t *c)
{
    return c->run_mode != CELL_STOP;
}

/** @brief 面板侧正常停止 cell，并同步请求控制层停止 PID。 */
static void cell_stop(TempPanel_t *p, uint8_t cell)
{
    TempCell_t *c = &p->cell[cell];
    c->run_mode = CELL_STOP;
    c->pid_enabled = false;
    c->program_phase = 0;
    c->program_timer_s = 0;
    c->program_interval_done = 0;
    Control_StopPid(cell);
}

/** @brief 只清除面板侧运行状态，不再重复调用控制层停止。 */
static void cell_stop_ui_only(TempPanel_t *p, uint8_t cell)
{
    TempCell_t *c = &p->cell[cell];
    c->run_mode = CELL_STOP;
    c->pid_enabled = false;
    c->program_phase = 0;
    c->program_timer_s = 0;
    c->program_interval_done = 0;
}

/** @brief 启动普通定点控温模式。 */
static void cell_start_jump(TempPanel_t *p, uint8_t cell)
{
    TempCell_t *c = &p->cell[cell];
    if (c->error != PANEL_ERR_NONE)
    {
        PanelHW_BlinkOnce();
        return;
    }
    c->run_mode = CELL_RUN_JUMP;
    c->pid_enabled = true;
    c->command_temp = c->target_temp;
    Control_StartPid(cell);
    Control_SetTargetTemp(cell, c->command_temp);
}

/* ============================================================
 * 2. 程序控温阶段定义
 * ============================================================ */
/** @brief 程序控温阶段：先从当前温度逼近起始温度。 */
#define PROG_PHASE_TO_START 1
/** @brief 程序控温阶段：起始温度保持。 */
#define PROG_PHASE_START_HOLD 2
/** @brief 程序控温阶段：按设定速率逼近下一目标温度。 */
#define PROG_PHASE_RAMP_NEXT 3
/** @brief 程序控温阶段：到达目标后等待。 */
#define PROG_PHASE_WAIT_NEXT 4
/** @brief 程序控温阶段：程序完成，保持最后目标。 */
#define PROG_PHASE_FINISHED 5
/** @brief 临时 UI 错误显示持续时间，单位 ms。 */
#define PANEL_UI_ERROR_DISPLAY_MS 1000U

/** @brief 启动程序控温，初始化程序阶段并启动控制层 PID。 */
static void cell_start_program(TempPanel_t *p, uint8_t cell)
{
    TempCell_t *c = &p->cell[cell];
    if (c->error != PANEL_ERR_NONE)
    {
        PanelHW_BlinkOnce();
        return;
    }
    c->run_mode = CELL_RUN_PROGRAM;
    c->pid_enabled = true;
    c->program_phase = PROG_PHASE_TO_START;
    c->program_timer_s = 0;
    c->program_interval_done = 0;
    c->program_step = c->program.next_temp - c->program.start_temp;
    c->program_next_target = c->program.start_temp;
    c->command_temp = c->current_temp;
    Control_StartPid(cell);
    Control_SetTargetTemp(cell, c->command_temp);
}

/*
 * 斜坡函数：让 current 按指定速率逼近 target。
 * ramp_per_min: degC/min。
 * dt_ms: 距离上次更新的毫秒数。
 */
/** @brief 按 degC/min 斜率将当前命令温度推进到目标温度。 */
static float ramp_to_target(float current,
                            float target,
                            float ramp_per_min,
                            uint32_t dt_ms)
{
    if (ramp_per_min < 0.0f)
        ramp_per_min = -ramp_per_min;
    if (ramp_per_min == 0.0f)
        return target;

    float diff = target - current;
    if (diff == 0.0f)
        return target;

    float step = ramp_per_min * (float)dt_ms / 60000.0f;
    if (step < 0.1f)
        step = 0.1f; /* 最小步进 0.1 degC，避免显示/命令长时间不动。 */

    if (diff > 0.0f)
        return (diff <= step) ? target : current + step;
    else
        return (-diff <= step) ? target : current - step;
}

/** @brief 检查程序控温参数是否合法，包含温度范围、速率和最终目标。 */
static bool program_params_valid(const TempProgram_t *program)
{
    float step;
    float final_target;

    if (program == NULL)
        return false;

    if (program->start_temp < PANEL_TEMP_MIN ||
        program->start_temp > PANEL_TEMP_MAX ||
        program->next_temp < PANEL_TEMP_MIN ||
        program->next_temp > PANEL_TEMP_MAX)
        return false;

    if (program->ramp_rate < PANEL_RAMP_RATE_MIN ||
        program->ramp_rate > PANEL_RAMP_RATE_MAX)
        return false;

    step = program->next_temp - program->start_temp;
    final_target = program->next_temp + step * (float)program->repeat_times;

    if (final_target < PANEL_TEMP_MIN || final_target > PANEL_TEMP_MAX)
        return false;

    return true;
}
/** @brief 普通模式 1 秒节拍更新，把目标温度同步给控制层。 */
static void update_jump_control(TempPanel_t *p, uint8_t cell, uint32_t dt_ms)
{
    TempCell_t *c = &p->cell[cell];
    (void)dt_ms;
    c->command_temp = c->target_temp;
    Control_SetTargetTemp(cell, c->command_temp);
}

/** @brief 程序控温 1 秒节拍状态机，更新 command_temp 并下发控制层。 */
static void update_program_control_1s(TempPanel_t *p, uint8_t cell)
{
    TempCell_t *c = &p->cell[cell];
    switch (c->program_phase)
    {
    case PROG_PHASE_TO_START:
        c->command_temp = ramp_to_target(c->command_temp,
                                         c->program.start_temp,
                                         PANEL_JUMP_RAMP_PER_MIN, 1000);
        if (c->command_temp == c->program.start_temp)
        {
            c->program_phase = PROG_PHASE_START_HOLD;
            c->program_timer_s = 0;
        }
        break;

    case PROG_PHASE_START_HOLD:
        c->command_temp = c->program.start_temp;
        if (c->program_timer_s >= c->program.start_hold_s)
        {
            c->program_phase = PROG_PHASE_RAMP_NEXT;
            c->program_timer_s = 0;
            c->program_next_target = c->program.next_temp;
        }
        else
            c->program_timer_s++;
        break;

    case PROG_PHASE_RAMP_NEXT:
        c->command_temp = ramp_to_target(c->command_temp,
                                         c->program_next_target,
                                         c->program.ramp_rate, 1000);
        if (c->command_temp == c->program_next_target)
        {
            c->program_phase = PROG_PHASE_WAIT_NEXT;
            c->program_timer_s = 0;
        }
        break;

    case PROG_PHASE_WAIT_NEXT:
        c->command_temp = c->program_next_target;
        if (c->program_timer_s >= c->program.wait_s)
        {
            c->program_timer_s = 0;
            if (c->program_interval_done >= c->program.repeat_times)
            {
                c->program_phase = PROG_PHASE_FINISHED;
            }
            else
            {
                c->program_interval_done++;
                c->program_next_target += c->program_step;
                c->program_next_target = clamp_f(c->program_next_target,
                                                 PANEL_TEMP_MIN, PANEL_TEMP_MAX);
                c->program_phase = PROG_PHASE_RAMP_NEXT;
            }
        }
        else
            c->program_timer_s++;
        break;

    case PROG_PHASE_FINISHED:
        c->command_temp = c->program_next_target;
        break;

    default:
        c->program_phase = PROG_PHASE_TO_START;
        break;
    }
    Control_SetTargetTemp(cell, c->command_temp);
}

/** @brief 设置 cell 错误码；非 NONE 时清除面板运行状态并紧急停止控制层。 */
static void set_error_and_stop(TempPanel_t *p, uint8_t cell, PanelError_t err)
{
    TempCell_t *c = &p->cell[cell];
    c->error = err;
    if (err != PANEL_ERR_NONE)
    {
        cell_stop_ui_only(p, cell);
        Control_EmergencyStopPid(cell);
    }
}

/** @brief 显示临时 UI 错误，例如运行中禁止编辑。 */
static void show_ui_error(TempPanel_t *p, PanelUiError_t err, uint32_t now_ms)
{
    if (p == NULL || err == PANEL_UI_ERR_NONE)
        return;
    p->ui_error = err;
    p->ui_error_until_ms = now_ms + PANEL_UI_ERROR_DISPLAY_MS;
}

/** @brief 判断错误码是否为测温传感器类错误。 */
static bool panel_error_is_temp_sensor(PanelError_t err)
{
    return (err == PANEL_ERR_E121_TEMP_CH1) ||
           (err == PANEL_ERR_E122_TEMP_CH2) ||
           (err == PANEL_ERR_E123_TEMP_CH3) ||
           (err == PANEL_ERR_E124_TEMP_CH4) ||
           (err == PANEL_ERR_E132_SENSOR);
}


/* ============================================================
 * 3. LED 刷新逻辑
 * ============================================================ */

/** @brief 根据当前模式、运行状态、显示内容和 active_cell 刷新 9 个 LED。 */
static void refresh_leds(TempPanel_t *p)
{
    TempCell_t *c = cur_cell(p);
    PanelHW_SetLed(1, p->mode == PANEL_MODE_NORMAL);
    PanelHW_SetLed(2, p->mode == PANEL_MODE_PARAM_SET);
    PanelHW_SetLed(3, p->mode == PANEL_MODE_EXTERNAL);
    PanelHW_SetLed(4, cell_is_running(c));
    PanelHW_SetLed(5, !cell_is_running(c));
    PanelHW_SetLed(6, p->show_type == PANEL_SHOW_TARGET ||
                          p->show_type == PANEL_SHOW_PARAM);
    PanelHW_SetLed(7, p->show_type == PANEL_SHOW_CURRENT);
    PanelHW_SetLed(8, p->active_cell == 0);
    PanelHW_SetLed(9, p->active_cell == 1);
}

/* ============================================================
 * 4. 参数读写辅助
 * ============================================================ */

/** @brief 读取当前程序参数项的数值，统一转成 float 便于显示/编辑。 */
static float get_param_value(const TempCell_t *c, ProgramParamIndex_t idx)
{
    switch (idx)
    {
    case PROG_PARAM_START_TEMP:
        return c->program.start_temp;
    case PROG_PARAM_START_HOLD:
        return (float)c->program.start_hold_s;
    case PROG_PARAM_RAMP_RATE:
        return c->program.ramp_rate;
    case PROG_PARAM_NEXT_TEMP:
        return c->program.next_temp;
    case PROG_PARAM_WAIT_TIME:
        return (float)c->program.wait_s;
    case PROG_PARAM_REPEAT_TIMES:
        return (float)c->program.repeat_times;
    default:
        return 0.0f;
    }
}

/** @brief 写入当前程序参数项，并按参数类型做范围限制。 */
static void set_param_value(TempCell_t *c, ProgramParamIndex_t idx, float val)
{
    switch (idx)
    {
    case PROG_PARAM_START_TEMP:
        c->program.start_temp = clamp_f(val, PANEL_TEMP_MIN, PANEL_TEMP_MAX);
        break;
    case PROG_PARAM_START_HOLD:
        if (val < 1)
            val = 1;
        c->program.start_hold_s = (uint16_t)val;
        break;
    case PROG_PARAM_RAMP_RATE:
        c->program.ramp_rate = clamp_f(val, PANEL_RAMP_RATE_MIN, PANEL_RAMP_RATE_MAX);
        break;
    case PROG_PARAM_NEXT_TEMP:
        c->program.next_temp = clamp_f(val, PANEL_TEMP_MIN, PANEL_TEMP_MAX);
        break;
    case PROG_PARAM_WAIT_TIME:
        if (val < 1)
            val = 1;
        c->program.wait_s = (uint16_t)val;
        break;
    case PROG_PARAM_REPEAT_TIMES:
        if (val < 1)
            val = 1;
        c->program.repeat_times = (uint16_t)val;
        break;
    default:
        break;
    }
}

/** @brief 获取当前程序参数项每次按键调整的基础步进。 */
static float get_param_step(ProgramParamIndex_t idx)
{
    switch (idx)
    {
    case PROG_PARAM_START_TEMP:
    case PROG_PARAM_NEXT_TEMP:
    case PROG_PARAM_RAMP_RATE:
        return PANEL_RAMP_RATE_MIN;
    default:
        return 1.0f;
    }
}

/** @brief 获取当前程序参数项允许的最大值。 */
static float get_param_max(ProgramParamIndex_t idx)
{
    switch (idx)
    {
    case PROG_PARAM_START_TEMP:
    case PROG_PARAM_NEXT_TEMP:
        return PANEL_TEMP_MAX;
    case PROG_PARAM_START_HOLD:
    case PROG_PARAM_WAIT_TIME:
        return 9999.0f;
    case PROG_PARAM_RAMP_RATE:
        return PANEL_RAMP_RATE_MAX;
    case PROG_PARAM_REPEAT_TIMES:
        return 9999.0f;
    default:
        return 9999.0f;
    }
}

/** @brief 获取当前程序参数项允许的最小值。 */
static float get_param_min(ProgramParamIndex_t idx)
{
    switch (idx)
    {
    case PROG_PARAM_START_TEMP:
    case PROG_PARAM_NEXT_TEMP:
        return PANEL_TEMP_MIN;
    case PROG_PARAM_START_HOLD:
    case PROG_PARAM_WAIT_TIME:
    case PROG_PARAM_REPEAT_TIMES:
        return 1.0f;
    case PROG_PARAM_RAMP_RATE:
        return PANEL_RAMP_RATE_MIN;
    default:
        return -9999.0f;
    }
}

/* ============================================================
 * 5. 数码管显示刷新
 * ============================================================ */

/** @brief 根据当前模式、显示类型、错误状态刷新 TM1638 数码管和 LED。 */
static void refresh_display(TempPanel_t *p)
{
    TempCell_t *c = cur_cell(p);

    if (c->error != PANEL_ERR_NONE)
    {
        p->show_type = PANEL_SHOW_ERROR;
        PanelHW_DisplayError(c->error);
        refresh_leds(p);
        return;
    }

    if (p->mode == PANEL_MODE_PARAM_SET)
    {
        if (cell_is_running(c))
        {
            if (p->show_type == PANEL_SHOW_TARGET)
                PanelHW_DisplayTemp(c->command_temp);
            else
                PanelHW_DisplayTemp(c->current_temp);
        }
        else
        {
            p->show_type = PANEL_SHOW_PARAM;
            if (p->param_show_cnt > 0)
            {
                const char *names[] = {"Strt", "StHd", "rAtE", "nExt", "uAit", "rEPt"};
                TM1638_ShowString(&htm1638, names[p->param_index]);
                p->param_show_cnt--;
            }
            else
            {
                float v = get_param_value(c, p->param_index);
                if (p->param_index == PROG_PARAM_START_TEMP ||
                    p->param_index == PROG_PARAM_NEXT_TEMP ||
                    p->param_index == PROG_PARAM_RAMP_RATE)
                    TM1638_ShowFloat(&htm1638, v, 1);
                else
                    PanelHW_DisplayNumber((int16_t)v);
            }
        }
        refresh_leds(p);
        return;
    }

    if (p->show_type == PANEL_SHOW_TARGET)
        PanelHW_DisplayTemp(c->target_temp);
    else
        PanelHW_DisplayTemp(c->current_temp);

    refresh_leds(p);
}

/* ============================================================
 * 6. 对外 API
 * ============================================================ */

/** @brief 全局面板状态实例。 */
TempPanel_t g_panel;

/** @brief 面板上电默认选中的 cell。 */
#define PANEL_DEFAULT_ACTIVE_CELL 0U

/** @brief 初始化面板状态、默认温度、默认程序参数并刷新显示。 */
void TempPanel_Init(TempPanel_t *p)
{
    int i;

    if (p == NULL)
        return;
    p->mode = PANEL_MODE_NORMAL;
    p->active_cell = PANEL_DEFAULT_ACTIVE_CELL;
    p->show_type = PANEL_SHOW_TARGET;
    p->param_index = PROG_PARAM_START_TEMP;
    p->editing = false;
    p->edit_tick_ms = 0;
    p->display_tick_ms = 0;
    p->second_tick_ms = 0;
    p->param_show_cnt = 0;
    p->param_inactive_tick_ms = 0;
    p->ui_error = PANEL_UI_ERR_NONE;
    p->ui_error_until_ms = 0;

    for (i = 0; i < PANEL_CELL_NUM; i++)
    {
        TempCell_t *c = &p->cell[i];
        p->cell_mode[i] = PANEL_MODE_NORMAL;
        c->current_temp = 25.0f;
        c->target_temp = 25.0f;
        c->command_temp = 25.0f;
        c->last_temp_update_ms = 0;
        c->run_mode = CELL_STOP;
        c->error = PANEL_ERR_NONE;
        c->pid_enabled = false;
        c->program_phase = 0;
        c->program_timer_s = 0;
        c->program_interval_done = 0;
        c->program_step = 0.0f;
        c->program_next_target = 0.0f;
        c->program.start_temp = 25.0f;
        c->program.start_hold_s = 60;
        c->program.ramp_rate = 2.0f;
        c->program.next_temp = 35.0f;
        c->program.wait_s = 30;
        c->program.repeat_times = 3;
    }

    refresh_display(p);
}

/** @brief 面板周期任务，处理程序节拍、显示切换、编辑超时和临时错误显示。 */
void TempPanel_Task(TempPanel_t *p, uint32_t now_ms)
{
    TempCell_t *c;

    if (p == NULL)
        return;
    c = cur_cell(p);

    /* Temperature timeout is owned by app_control.c now.
     * The control layer can identify CH1/CH2/CH3/CH4 separately, retry
     * NRST_OTHER, and then report E121-E124. Keeping a generic panel-side
     * timeout here would hide the failed route behind E132.
     */

    if (now_ms - p->second_tick_ms >= 1000)
    {
        uint8_t i;

        p->second_tick_ms = now_ms;
        for (i = 0; i < PANEL_CELL_NUM; i++)
        {
            TempCell_t *ci = &p->cell[i];
            if (ci->run_mode == CELL_RUN_JUMP)
                update_jump_control(p, i, 1000);
            else if (ci->run_mode == CELL_RUN_PROGRAM)
                update_program_control_1s(p, i);
        }
    }

    if (p->mode == PANEL_MODE_NORMAL && !p->editing && c->error == PANEL_ERR_NONE)
    {
        if (!cell_is_running(c))
        {
            p->show_type = PANEL_SHOW_TARGET;
            p->display_tick_ms = now_ms;
        }
        else if (now_ms - p->display_tick_ms >= PANEL_DISPLAY_SWITCH_MS)
        {
            p->display_tick_ms = now_ms;
            p->show_type = (p->show_type == PANEL_SHOW_CURRENT)
                               ? PANEL_SHOW_TARGET
                               : PANEL_SHOW_CURRENT;
        }
    }

    if (p->mode == PANEL_MODE_PARAM_SET && cell_is_running(c) && c->error == PANEL_ERR_NONE)
    {
        if (now_ms - p->display_tick_ms >= PANEL_DISPLAY_SWITCH_MS)
        {
            p->display_tick_ms = now_ms;
            p->show_type = (p->show_type == PANEL_SHOW_CURRENT)
                               ? PANEL_SHOW_TARGET
                               : PANEL_SHOW_CURRENT;
        }
    }

    if (p->editing && (now_ms - p->edit_tick_ms >= PANEL_EDIT_TIMEOUT_MS))
    {
        p->editing = false;
        p->show_type = (p->mode == PANEL_MODE_NORMAL && !cell_is_running(c))
                           ? PANEL_SHOW_TARGET
                           : PANEL_SHOW_CURRENT;
    }

    if (p->ui_error != PANEL_UI_ERR_NONE)
    {
        if (now_ms < p->ui_error_until_ms)
        {
            PanelHW_DisplayError((PanelError_t)p->ui_error);
            refresh_leds(p);
            return;
        }
        p->ui_error = PANEL_UI_ERR_NONE;
    }

    refresh_display(p);
}
/** @brief 处理一个抽象按键事件，完成模式切换、参数编辑、启停和 cell 切换。 */
void TempPanel_KeyEvent(TempPanel_t *p,
                        PanelKey_t key,
                        PanelKeyEvent_t evt,
                        uint32_t now_ms)
{
    if (p == NULL)
        return;
    TempCell_t *c = cur_cell(p);

    if (evt != PANEL_KEY_EVT_SHORT && key != PANEL_KEY_UP && key != PANEL_KEY_DOWN)
        return;

    if ((c->error != PANEL_ERR_NONE) && (key != PANEL_KEY_SWITCH))
    {
        p->editing = false;
        p->show_type = PANEL_SHOW_ERROR;
        PanelHW_DisplayError(c->error);
        refresh_leds(p);
        return;
    }
    if (p->mode == PANEL_MODE_PARAM_SET)
        p->param_inactive_tick_ms = now_ms;

    /* 步进倍率：短按 1 倍，重复 5 倍，长按 20 倍。 */
    float mult = 1.0f;
    if (evt == PANEL_KEY_EVT_REPEAT)
        mult = 5.0f;
    if (evt == PANEL_KEY_EVT_LONG)
        mult = 20.0f;

    switch (key)
    {
    case PANEL_KEY_SWITCH:
        p->active_cell = (p->active_cell + 1) % PANEL_CELL_NUM;
        p->mode = p->cell_mode[p->active_cell];
        c = cur_cell(p);
        p->editing = false;
        p->display_tick_ms = now_ms;
        if (p->mode == PANEL_MODE_PARAM_SET && !cell_is_running(c))
        {
            p->param_inactive_tick_ms = now_ms;
            if (p->param_show_cnt == 0)
                p->param_show_cnt = 50;
        }
        else
        {
            p->show_type = (p->mode == PANEL_MODE_NORMAL && !cell_is_running(c))
                               ? PANEL_SHOW_TARGET
                               : PANEL_SHOW_CURRENT;
        }
        break;

    case PANEL_KEY_MODE:
        if (!cell_is_running(c))
        {
            p->mode = (PanelMode_t)((p->mode + 1) % 3);
            p->cell_mode[p->active_cell] = p->mode;
            p->editing = false;
            p->show_type = (p->mode == PANEL_MODE_NORMAL)
                               ? PANEL_SHOW_TARGET
                               : PANEL_SHOW_CURRENT;
            if (p->mode == PANEL_MODE_PARAM_SET)
            {
                p->param_show_cnt = 50;
                p->param_inactive_tick_ms = now_ms;
            }
        }
        else
        {
            show_ui_error(p, PANEL_UI_ERR_MODE_LOCKED, now_ms);
        }
        break;

    case PANEL_KEY_START_STOP:
        if (c->run_mode == CELL_STOP)
        {
            if (p->mode == PANEL_MODE_NORMAL)
                cell_start_jump(p, active(p));
            else if (p->mode == PANEL_MODE_PARAM_SET)
                cell_start_program(p, active(p));
            p->show_type = PANEL_SHOW_CURRENT;
            p->display_tick_ms = now_ms;
        }
        else
        {
            cell_stop(p, active(p));
            p->show_type = (p->mode == PANEL_MODE_NORMAL)
                               ? PANEL_SHOW_TARGET
                               : PANEL_SHOW_CURRENT;
        }
        break;

    case PANEL_KEY_ENTER:
        if (p->mode == PANEL_MODE_PARAM_SET)
        {
            if (!cell_is_running(c))
            {
                p->param_index = (ProgramParamIndex_t)((p->param_index + 1) % PROG_PARAM_COUNT);
                p->param_show_cnt = 50;
            }
            else
            {
                show_ui_error(p, PANEL_UI_ERR_PARAM_ENTER_RUNNING, now_ms);
            }
        }
        else if (p->mode == PANEL_MODE_NORMAL && cell_is_running(c))
        {
            show_ui_error(p, PANEL_UI_ERR_NORMAL_ENTER_RUNNING, now_ms);
        }
        break;

    case PANEL_KEY_UP:
        if (!cell_is_running(c))
        {
            if (p->mode == PANEL_MODE_NORMAL)
            {
                c->target_temp = clamp_f(c->target_temp + 0.1f * mult,
                                         PANEL_TEMP_MIN,
                                         PANEL_TEMP_MAX);
                p->editing = true;
                p->edit_tick_ms = now_ms;
                p->show_type = PANEL_SHOW_TARGET;
                p->display_tick_ms = now_ms;
            }
            else if (p->mode == PANEL_MODE_PARAM_SET)
            {
                float step = get_param_step(p->param_index);
                float val = get_param_value(c, p->param_index) + step * mult;
                float max = get_param_max(p->param_index);
                if (val > max)
                    val = max;
                set_param_value(c, p->param_index, val);
                p->editing = true;
                p->edit_tick_ms = now_ms;
            }
        }
        else if (p->mode == PANEL_MODE_PARAM_SET)
        {
            show_ui_error(p, PANEL_UI_ERR_PARAM_EDIT_RUNNING, now_ms);
        }
        else if (p->mode == PANEL_MODE_NORMAL)
        {
            show_ui_error(p, PANEL_UI_ERR_NORMAL_EDIT_RUNNING, now_ms);
        }
        break;

    case PANEL_KEY_DOWN:
        if (!cell_is_running(c))
        {
            if (p->mode == PANEL_MODE_NORMAL)
            {
                c->target_temp = clamp_f(c->target_temp - 0.1f * mult,
                                         PANEL_TEMP_MIN,
                                         PANEL_TEMP_MAX);
                p->editing = true;
                p->edit_tick_ms = now_ms;
                p->show_type = PANEL_SHOW_TARGET;
                p->display_tick_ms = now_ms;
            }
            else if (p->mode == PANEL_MODE_PARAM_SET)
            {
                float step = get_param_step(p->param_index);
                float val = get_param_value(c, p->param_index) - step * mult;
                float min = get_param_min(p->param_index);
                if (val < min)
                    val = min;
                set_param_value(c, p->param_index, val);
                p->editing = true;
                p->edit_tick_ms = now_ms;
            }
        }
        else if (p->mode == PANEL_MODE_PARAM_SET)
        {
            show_ui_error(p, PANEL_UI_ERR_PARAM_EDIT_RUNNING, now_ms);
        }
        else if (p->mode == PANEL_MODE_NORMAL)
        {
            show_ui_error(p, PANEL_UI_ERR_NORMAL_EDIT_RUNNING, now_ms);
        }
        break;

    default:
        break;
    }
}

/** @brief 更新指定 cell 的当前温度；测温类错误在收到新温度后自动清除。 */
void TempPanel_UpdateMeasuredTemp(TempPanel_t *p,
                                  uint8_t cell_idx,
                                  float temp,
                                  uint32_t now_ms)
{
    if (p == NULL || cell_idx >= PANEL_CELL_NUM)
        return;
    TempCell_t *c = &p->cell[cell_idx];
    c->current_temp = temp;
    c->last_temp_update_ms = now_ms;
    if (panel_error_is_temp_sensor(c->error))
        c->error = PANEL_ERR_NONE;
}

/** @brief 设置水路/公共错误；置位时停止全部 cell。 */
void TempPanel_SetWaterError(TempPanel_t *p, bool error)
{
    if (p == NULL)
        return;
    for (uint8_t i = 0; i < PANEL_CELL_NUM; i++)
        set_error_and_stop(p, i, error ? PANEL_ERR_E1_WATER : PANEL_ERR_NONE);
}

/** @brief 设置旧版冷热片错误；置位时停止指定 cell。 */
void TempPanel_SetPeltierError(TempPanel_t *p, uint8_t cell, bool error)
{
    if (p == NULL || cell >= PANEL_CELL_NUM)
        return;
    set_error_and_stop(p, cell, error ? PANEL_ERR_E3_PELTIER : PANEL_ERR_NONE);
}

/** @brief 设置控制层传入的 cell 错误码。 */
void TempPanel_SetCellError(TempPanel_t *p, uint8_t cell, PanelError_t err)
{
    if (p == NULL || cell >= PANEL_CELL_NUM)
        return;
    set_error_and_stop(p, cell, err);
}

/** @brief 面板侧正常停止指定 cell。 */
void TempPanel_Stop(TempPanel_t *p, uint8_t cell)
{
    if (p == NULL || cell >= PANEL_CELL_NUM)
        return;
    cell_stop(p, cell);
}

/** @brief 从中断或其他任务请求停止指定 cell，使用位图延迟到面板任务处理。 */
void TempPanel_RequestStop(uint8_t cell)
{
    if (cell >= PANEL_CELL_NUM)
        return;
    __disable_irq();
    s_panel_stop_request_mask |= (uint8_t)(1U << cell);
    __enable_irq();
}

/** @brief 服务异步停止请求，清除面板侧运行状态。 */
void TempPanel_ServiceRequests(TempPanel_t *p)
{
    uint8_t mask;
    uint8_t cell;

    if (p == NULL)
        return;

    __disable_irq();
    mask = s_panel_stop_request_mask;
    s_panel_stop_request_mask = 0U;
    __enable_irq();
    if (mask == 0U)
        return;

    for (cell = 0U; cell < PANEL_CELL_NUM; cell++)
    {
        if ((mask & (uint8_t)(1U << cell)) != 0U)
        {
            cell_stop_ui_only(p, cell);
            if (p->active_cell == cell)
            {
                p->show_type = (p->mode == PANEL_MODE_NORMAL)
                                   ? PANEL_SHOW_TARGET
                                   : PANEL_SHOW_CURRENT;
            }
        }
    }
}

/** @brief 外部接口：设置目标温度并启动普通定点控温。 */
uint8_t TempPanel_StartNormal(TempPanel_t *p, uint8_t cell, float target_temp)
{
    TempCell_t *c;

    if (p == NULL || cell >= PANEL_CELL_NUM)
        return 0U;

    if (target_temp < PANEL_TEMP_MIN || target_temp > PANEL_TEMP_MAX)
        return 0U;

    c = &p->cell[cell];
    if (cell_is_running(c) || c->error != PANEL_ERR_NONE)
        return 0U;

    c->target_temp = target_temp;

    p->active_cell = cell;
    p->mode = PANEL_MODE_NORMAL;
    p->cell_mode[cell] = PANEL_MODE_NORMAL;
    p->editing = false;
    p->show_type = PANEL_SHOW_CURRENT;
    p->display_tick_ms = 0;

    cell_start_jump(p, cell);
    refresh_display(p);

    return (c->run_mode == CELL_RUN_JUMP) ? 1U : 0U;
}

/** @brief 外部接口：写入指定 cell 的程序控温参数。 */
uint8_t TempPanel_SetProgram(TempPanel_t *p,
                             uint8_t cell,
                             const TempProgram_t *program)
{
    TempCell_t *c;

    if (p == NULL || program == NULL || cell >= PANEL_CELL_NUM)
        return 0U;

    if (!program_params_valid(program))
        return 0U;

    c = &p->cell[cell];
    if (cell_is_running(c))
        return 0U;

    c->program.start_temp = program->start_temp;
    c->program.start_hold_s = program->start_hold_s;
    c->program.ramp_rate = program->ramp_rate;
    c->program.next_temp = program->next_temp;
    c->program.wait_s = program->wait_s;
    c->program.repeat_times = program->repeat_times;

    p->active_cell = cell;
    p->mode = PANEL_MODE_PARAM_SET;
    p->cell_mode[cell] = PANEL_MODE_PARAM_SET;
    p->editing = false;
    p->show_type = PANEL_SHOW_PARAM;
    p->param_index = PROG_PARAM_START_TEMP;
    p->param_show_cnt = 50U;

    return 1U;
}

/** @brief 外部接口：启动指定 cell 的程序控温。 */
uint8_t TempPanel_StartProgram(TempPanel_t *p, uint8_t cell)
{
    TempCell_t *c;

    if (p == NULL || cell >= PANEL_CELL_NUM)
        return 0U;

    c = &p->cell[cell];
    if (cell_is_running(c) || c->error != PANEL_ERR_NONE)
        return 0U;

    p->active_cell = cell;
    p->mode = PANEL_MODE_PARAM_SET;
    p->cell_mode[cell] = PANEL_MODE_PARAM_SET;
    p->editing = false;
    p->show_type = PANEL_SHOW_CURRENT;
    cell_start_program(p, cell);

    return (c->run_mode == CELL_RUN_PROGRAM) ? 1U : 0U;
}

/** @brief 将 TM1638 物理键值映射为面板抽象按键。 */
PanelKey_t PanelKey_FromTM1638(TM1638_Key_t key)
{
    switch (key)
    {
    case KEY_MODE:
        return PANEL_KEY_MODE;
    case KEY_START_STOP:
        return PANEL_KEY_START_STOP;
    case KEY_UP:
        return PANEL_KEY_UP;
    case KEY_DOWN:
        return PANEL_KEY_DOWN;
    case KEY_ENTER:
        return PANEL_KEY_ENTER;
    case KEY_SWITCH:
        return PANEL_KEY_SWITCH;
    default:
        return PANEL_KEY_NONE;
    }
}

/* ============================================================
 * 7. 弱函数默认实现
 * ============================================================ */

/** @brief 默认弱实现：显示一位小数温度。 */
__attribute__((weak)) void PanelHW_DisplayTemp(float temp)
{
    TM1638_ShowFloat(&htm1638, temp, 1);
}

/** @brief 默认弱实现：显示整数参数，并限制到数码管范围。 */
__attribute__((weak)) void PanelHW_DisplayNumber(int16_t value)
{
    float v = (float)value;
    if (v > 9999.0f)
        v = 9999.0f;
    if (v < -999.0f)
        v = -999.0f;
    TM1638_ShowFloat(&htm1638, v, 0);
}

/** @brief 默认弱实现：显示 E + 错误码数字。 */
__attribute__((weak)) void PanelHW_DisplayError(PanelError_t err)
{
    TM1638_ClearDisplay(&htm1638);
    if (err == PANEL_ERR_NONE)
        return;
    uint16_t code = (uint16_t)err;
    uint8_t d3 = (uint8_t)(code / 100U);
    uint8_t d2 = (uint8_t)((code / 10U) % 10U);
    uint8_t d1 = (uint8_t)(code % 10U);
    TM1638_ShowChar(&htm1638, 0, 'E', false);
    if (d3 > 0)
    {
        TM1638_ShowDigit(&htm1638, 1, d3, false);
        TM1638_ShowDigit(&htm1638, 2, d2, false);
        TM1638_ShowDigit(&htm1638, 3, d1, false);
    }
    else
    {
        TM1638_ShowChar(&htm1638, 1, ' ', false);
        TM1638_ShowDigit(&htm1638, 2, d2, false);
        TM1638_ShowDigit(&htm1638, 3, d1, false);
    }
}

/** @brief 默认弱实现：设置 TM1638 的 LED1..LED9。 */
__attribute__((weak)) void PanelHW_SetLed(uint8_t led_id, bool on)
{
    if (led_id >= 1 && led_id <= 9)
        TM1638_SetLED(&htm1638, led_id, on);
}

/** @brief 默认弱实现：短闪运行 LED，提示无效操作。 */
__attribute__((weak)) void PanelHW_BlinkOnce(void)
{
    TM1638_SetLED(&htm1638, 4, true);
    TM1638_SetLED(&htm1638, 4, false);
}

extern PID_TypeDef temp_pid;

/** @brief 控制层启动 PID 的弱实现，占位用；真实实现由 app_control 覆盖。 */
__attribute__((weak)) void Control_StartPid(uint8_t cell)
{
    (void)cell;
    /* PID_TypeDef *pid = get_pid_for_cell(cell);
       PID_Reset(pid);
       HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); ... */
}

/** @brief 控制层正常停止 PID 的弱实现，占位用。 */
__attribute__((weak)) void Control_StopPid(uint8_t cell)
{
    (void)cell;
    /* HAL_TIM_PWM_Stop(...); PID_Reset(pid); */
}

/** @brief 控制层紧急停止 PID 的弱实现，占位用。 */
__attribute__((weak)) void Control_EmergencyStopPid(uint8_t cell)
{
    (void)cell;
}

/** @brief 控制层设置目标温度的弱实现，占位用。 */
__attribute__((weak)) void Control_SetTargetTemp(uint8_t cell, float target)
{
    (void)cell;
    (void)target;
    /* PID_SetTarget(pid, target); */
}

/** @brief 初始化 TM1638 硬件、面板状态和旧版 temp_pid 默认参数。 */
void Panel_Init(void)
{
    TM1638_HW_Init();
    TempPanel_Init(&g_panel);
    PID_Init(&temp_pid, 30.0f, 1.0f, 0.0f, 0.1f);
    PID_SetLimits(&temp_pid, 700.0f, 1699.0f, 700.0f, 1600.0f);
    refresh_display(&g_panel);
}












