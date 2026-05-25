#include "temp_panel.h"
#include "pid_controller.h"

/* ============================================================
 * 0. 硬件接口函数前置声明
 *    (内部辅助函数中会调用它们, 所以必须在调用前声明)
 * ============================================================ */
__attribute__((weak)) void PanelHW_DisplayTempX10(int16_t temp_x10);
__attribute__((weak)) void PanelHW_DisplayNumber(int16_t value);
__attribute__((weak)) void PanelHW_DisplayError(PanelError_t err);
__attribute__((weak)) void PanelHW_SetLed(uint8_t led_id, bool on);
__attribute__((weak)) void PanelHW_BlinkOnce(void);
__attribute__((weak)) void Control_StartPid(uint8_t cell);
__attribute__((weak)) void Control_StopPid(uint8_t cell);
__attribute__((weak)) void Control_SetTargetTemp(uint8_t cell, int16_t target_x10);

/* ============================================================
 * 1. 内部辅助函数
 * ============================================================ */

/*
 * 限幅函数。
 *
 * 作用：
 * 保证 v 不小于 min，也不大于 max。
 *
 * 例如：
 * clamp_i16(1200, 0, 1100) 返回 1100
 * clamp_i16(-10, 0, 1100) 返回 0
 */
static int16_t clamp_i16(int16_t v, int16_t min, int16_t max)
{
    if (v < min)
        return min;
    if (v > max)
        return max;
    return v;
}

/*
 * 返回当前正在操作的恒温池编号。
 *
 * p->active_cell = 0 表示当前操作 1 号池。
 * p->active_cell = 1 表示当前操作 2 号池。
 */
static uint8_t active(TempPanel_t *p)
{
    return p->active_cell;
}

/*
 * 返回当前正在操作的恒温池结构体指针。
 *
 * 这样后面的代码可以直接写：
 *
 * TempCell_t *c = cur_cell(p);
 *
 * 然后 c->target_temp_x10 就是当前恒温池的目标温度。
 */
static TempCell_t *cur_cell(TempPanel_t *p)
{
    return &p->cell[p->active_cell];
}

/*
 * 判断某个恒温池是否正在运行。
 *
 * 只要不是 CELL_STOP，就认为正在运行。
 */
static bool cell_is_running(const TempCell_t *c)
{
    return c->run_mode != CELL_STOP;
}

/*
 * 停止某个恒温池。
 *
 * 这里不仅改变软件状态，
 * 还会调用 Control_StopPid() 关闭真实 PID/H桥。
 */
static void cell_stop(TempPanel_t *p, uint8_t cell)
{
    TempCell_t *c = &p->cell[cell];

    /*
     * 设置为停止状态。
     */
    c->run_mode = CELL_STOP;

    /*
     * 软件标志：PID 已关闭。
     */
    c->pid_enabled = false;

    /*
     * 清除程序控温内部状态。
     */
    c->program_phase = 0;
    c->program_timer_s = 0;
    c->program_interval_done = 0;

    /*
     * 调用底层硬件控制函数。
     * 你要在这里真正关闭 H 桥输出。
     */
    Control_StopPid(cell);
}

/*
 * 启动普通恒温模式，也就是 Jump 模式。
 *
 * 逻辑：
 * 1. 如果有错误，不允许启动
 * 2. 设置运行状态为 CELL_RUN_JUMP
 * 3. 打开 PID
 * 4. command_temp 从当前温度开始
 * 5. 后续由 update_jump_control() 慢慢逼近目标温度
 */
static void cell_start_jump(TempPanel_t *p, uint8_t cell)
{
    TempCell_t *c = &p->cell[cell];

    /*
     * 如果当前恒温池有错误，禁止启动。
     */
    if (c->error != PANEL_ERR_NONE)
    {
        PanelHW_BlinkOnce();
        return;
    }

    /*
     * 进入普通恒温运行状态。
     */
    c->run_mode = CELL_RUN_JUMP;

    /*
     * 标记 PID 已使能。
     */
    c->pid_enabled = true;

    /*
     * command_temp_x10 不直接等于 target_temp_x10，
     * 而是先从当前温度开始。
     *
     * 这样可以避免目标温度突变。
     *
     * 例如当前温度 25℃，目标温度 80℃，
     * 如果直接把 PID 目标值设为 80℃，输出会很大。
     * 所以这里先让 PID 目标值等于当前温度，
     * 再按固定速率慢慢升到 80℃。
     */
    c->command_temp_x10 = c->current_temp_x10;

    /*
     * 启动底层 PID/H桥。
     */
    Control_StartPid(cell);

    /*
     * 先把当前温度作为 PID 初始目标值。
     */
    Control_SetTargetTemp(cell, c->command_temp_x10);
}

/* ============================================================
 * 3. 程序控温阶段定义
 * ============================================================
 *
 * 程序控温被拆成几个阶段：
 *
 * TO_START：
 * 先从当前温度变化到起始温度。
 *
 * START_HOLD：
 * 在起始温度保持一段时间。
 *
 * RAMP_NEXT：
 * 按设定升降温速率变化到下一个目标温度。
 *
 * WAIT_NEXT：
 * 到达该温度点后等待一段时间。
 *
 * FINISHED：
 * 程序结束，保持最终温度。
 */
#define PROG_PHASE_TO_START 1
#define PROG_PHASE_START_HOLD 2
#define PROG_PHASE_RAMP_NEXT 3
#define PROG_PHASE_WAIT_NEXT 4
#define PROG_PHASE_FINISHED 5

/*
 * 启动程序控温模式。
 *
 * 参数来自 c->program。
 */
static void cell_start_program(TempPanel_t *p, uint8_t cell)
{
    TempCell_t *c = &p->cell[cell];

    /*
     * 有错误则不允许启动。
     */
    if (c->error != PANEL_ERR_NONE)
    {
        PanelHW_BlinkOnce();
        return;
    }

    /*
     * 设置运行模式为程序控温。
     */
    c->run_mode = CELL_RUN_PROGRAM;
    c->pid_enabled = true;

    /*
     * 程序从“先到达起始温度”这个阶段开始。
     */
    c->program_phase = PROG_PHASE_TO_START;

    /*
     * 清零程序计时器。
     */
    c->program_timer_s = 0;

    /*
     * 已完成温度间隔数量清零。
     */
    c->program_interval_done = 0;

    /*
     * 计算每次温度变化的步长。
     *
     * 例如：
     * start_temp = 10℃
     * next_temp  = 11℃
     *
     * 那么 program_step = 1℃
     *
     * 后续目标温度会变成：
     * 11℃、12℃、13℃……
     */
    c->program_step_x10 =
        c->program.next_temp_x10 - c->program.start_temp_x10;

    /*
     * 当前程序的下一个目标先设为起始温度。
     */
    c->program_next_target_x10 = c->program.start_temp_x10;

    /*
     * PID 的命令温度从当前温度开始，
     * 避免突然跳变。
     */
    c->command_temp_x10 = c->current_temp_x10;

    /*
     * 启动底层 PID/H桥。
     */
    Control_StartPid(cell);

    /*
     * 设置初始 PID 目标温度。
     */
    Control_SetTargetTemp(cell, c->command_temp_x10);
}

/*
 * 斜坡函数：让 current_x10 按一定速率逼近 target_x10。
 *
 * current_x10：
 * 当前命令温度。
 *
 * target_x10：
 * 目标温度。
 *
 * ramp_x10_per_min：
 * 升降温速率，单位 0.1℃/min。
 *
 * dt_ms：
 * 距离上一次更新过去了多少毫秒。
 *
 * 返回值：
 * 更新后的命令温度。
 */
static int16_t ramp_to_target(int16_t current_x10,
                              int16_t target_x10,
                              int16_t ramp_x10_per_min,
                              uint32_t dt_ms)
{
    /*
     * 如果速率是负数，取绝对值。
     * 方向由 current 和 target 的大小关系决定。
     */
    if (ramp_x10_per_min < 0)
    {
        ramp_x10_per_min = -ramp_x10_per_min;
    }

    /*
     * 如果速率为 0，就直接跳到目标温度。
     */
    if (ramp_x10_per_min == 0)
    {
        return target_x10;
    }

    /*
     * diff 表示距离目标温度还差多少。
     */
    int32_t diff = (int32_t)target_x10 - current_x10;

    /*
     * 已经到达目标，直接返回。
     */
    if (diff == 0)
    {
        return target_x10;
    }

    /*
     * 根据速率和时间间隔计算本次最多变化多少。
     *
     * ramp_x10_per_min 单位是 0.1℃/min。
     * dt_ms 单位是 ms。
     *
     * 所以：
     * step = ramp * dt_ms / 60000
     *
     * 例如：
     * ramp = 50，表示 5.0℃/min
     * dt_ms = 1000，表示 1s
     *
     * step = 50 * 1000 / 60000 = 0
     *
     * 因为整数除法会变成 0，
     * 所以下面强制 step 至少为 1。
     *
     * 注意：
     * 这会让低速率时有一点近似误差。
     * 后续如果你需要更精确，可以改成累积小数余量。
     */
    int32_t step =
        ((int32_t)ramp_x10_per_min * (int32_t)dt_ms) / 60000;

    /*
     * 最小变化 0.1℃。
     *
     * 这样避免 step = 0 导致温度永远不变。
     */
    if (step < 1)
    {
        step = 1;
    }

    /*
     * 如果目标温度比当前命令温度高，就升温。
     */
    if (diff > 0)
    {
        /*
         * 如果剩余差值小于 step，说明这一次就能到达目标。
         */
        if (diff <= step)
            return target_x10;

        /*
         * 否则向上增加 step。
         */
        return current_x10 + step;
    }
    else
    {
        /*
         * 目标温度比当前命令温度低，执行降温。
         */
        if (-diff <= step)
            return target_x10;

        return current_x10 - step;
    }
}

/*
 * 普通恒温 Jump 模式更新。
 *
 * 该函数会周期性被 TempPanel_Task() 调用。
 *
 * 作用：
 * command_temp_x10 按固定速率逼近 target_temp_x10。
 */
static void update_jump_control(TempPanel_t *p, uint8_t cell, uint32_t dt_ms)
{
    TempCell_t *c = &p->cell[cell];

    /*
     * 按固定速率更新 PID 命令温度。
     */
    c->command_temp_x10 =
        ramp_to_target(c->command_temp_x10,
                       c->target_temp_x10,
                       PANEL_JUMP_RAMP_X10_PER_MIN,
                       dt_ms);

    /*
     * 把更新后的命令温度交给 PID。
     */
    Control_SetTargetTemp(cell, c->command_temp_x10);
}

/*
 * 程序控温 1 秒更新一次。
 *
 * 程序控温中的保持时间、等待时间都是以秒为单位，
 * 所以这里按 1s 调用即可。
 */
static void update_program_control_1s(TempPanel_t *p, uint8_t cell)
{
    TempCell_t *c = &p->cell[cell];

    switch (c->program_phase)
    {
    case PROG_PHASE_TO_START:
        /*
         * 阶段 1：
         * 先从当前温度变化到程序设定的起始温度。
         *
         * 这里使用 Jump 模式的固定速率。
         */
        c->command_temp_x10 =
            ramp_to_target(c->command_temp_x10,
                           c->program.start_temp_x10,
                           PANEL_JUMP_RAMP_X10_PER_MIN,
                           1000);

        /*
         * 如果已经到达起始温度，
         * 切换到“起始温度保持”阶段。
         */
        if (c->command_temp_x10 == c->program.start_temp_x10)
        {
            c->program_phase = PROG_PHASE_START_HOLD;
            c->program_timer_s = 0;
        }
        break;

    case PROG_PHASE_START_HOLD:
        /*
         * 阶段 2：
         * 在起始温度保持一段时间。
         */
        c->command_temp_x10 = c->program.start_temp_x10;

        /*
         * 如果保持时间到了，
         * 进入按速率变化到下一目标温度的阶段。
         */
        if (c->program_timer_s >= c->program.start_hold_s)
        {
            c->program_phase = PROG_PHASE_RAMP_NEXT;
            c->program_timer_s = 0;
            c->program_next_target_x10 = c->program.next_temp_x10;
        }
        else
        {
            /*
             * 保持时间未到，继续计时。
             */
            c->program_timer_s++;
        }
        break;

    case PROG_PHASE_RAMP_NEXT:
        /*
         * 阶段 3：
         * 按设定升降温速率变化到下一个目标温度。
         */
        c->command_temp_x10 =
            ramp_to_target(c->command_temp_x10,
                           c->program_next_target_x10,
                           c->program.ramp_x10_per_min,
                           1000);

        /*
         * 到达当前目标温度后，进入等待阶段。
         */
        if (c->command_temp_x10 == c->program_next_target_x10)
        {
            c->program_phase = PROG_PHASE_WAIT_NEXT;
            c->program_timer_s = 0;
        }
        break;

    case PROG_PHASE_WAIT_NEXT:
        /*
         * 阶段 4：
         * 到达当前温度点后，保持等待 wait_s 秒。
         */
        c->command_temp_x10 = c->program_next_target_x10;

        /*
         * 等待时间到了，判断是否继续下一个温度点。
         */
        if (c->program_timer_s >= c->program.wait_s)
        {
            c->program_timer_s = 0;

            /*
             * 如果已经完成规定次数，
             * 程序进入完成状态。
             */
            if (c->program_interval_done >= c->program.repeat_times)
            {
                c->program_phase = PROG_PHASE_FINISHED;
            }
            else
            {
                /*
                 * 继续下一个温度点。
                 */
                c->program_interval_done++;

                /*
                 * 计算新的目标温度。
                 *
                 * 例如：
                 * 当前目标是 11℃
                 * 步长是 1℃
                 * 下一个目标就是 12℃。
                 */
                c->program_next_target_x10 += c->program_step_x10;

                /*
                 * 防止目标温度超过系统允许范围。
                 */
                c->program_next_target_x10 =
                    clamp_i16(c->program_next_target_x10,
                              PANEL_TEMP_MIN_X10,
                              PANEL_TEMP_MAX_X10);

                /*
                 * 回到升降温阶段。
                 */
                c->program_phase = PROG_PHASE_RAMP_NEXT;
            }
        }
        else
        {
            /*
             * 等待时间未到，继续计时。
             */
            c->program_timer_s++;
        }
        break;

    case PROG_PHASE_FINISHED:
        /*
         * 阶段 5：
         * 程序完成。
         *
         * 当前写法：
         * 程序结束后保持最终温度。
         *
         * 如果你希望程序完成后自动停止，
         * 可以把这里改成：
         *
         * cell_stop(p, cell);
         */
        c->command_temp_x10 = c->program_next_target_x10;
        break;

    default:
        /*
         * 如果阶段值异常，重新从第一阶段开始。
         */
        c->program_phase = PROG_PHASE_TO_START;
        break;
    }

    /*
     * 无论当前处于哪个阶段，
     * 最后都把 command_temp_x10 交给 PID。
     */
    Control_SetTargetTemp(cell, c->command_temp_x10);
}

/*
 * 设置错误并停止对应恒温池。
 *
 * 只要出现错误，就立即停止 PID/H桥。
 */
static void set_error_and_stop(TempPanel_t *p, uint8_t cell, PanelError_t err)
{
    TempCell_t *c = &p->cell[cell];

    c->error = err;

    if (err != PANEL_ERR_NONE)
    {
        cell_stop(p, cell);
    }
}

/* ============================================================
 * 4. LED 刷新逻辑
 * ============================================================ */

/*
 * 根据当前系统状态刷新 9 个 LED。
 *
 * LED 对应关系：
 *
 * LED1：正常模式
 * LED2：参数设置模式
 * LED3：外部控制模式
 * LED4：运行中
 * LED5：停止中
 * LED6：显示设定温度/参数
 * LED7：显示当前温度
 * LED8：当前选择 1 号恒温池
 * LED9：当前选择 2 号恒温池
 */
static void refresh_leds(TempPanel_t *p)
{
    TempCell_t *c = cur_cell(p);

    /*
     * 模式指示灯。
     */
    PanelHW_SetLed(1, p->mode == PANEL_MODE_NORMAL);
    PanelHW_SetLed(2, p->mode == PANEL_MODE_PARAM_SET);
    PanelHW_SetLed(3, p->mode == PANEL_MODE_EXTERNAL);

    /*
     * 运行/停止状态指示灯。
     *
     * 注意：
     * 显示的是“当前选中的恒温池”的状态。
     */
    PanelHW_SetLed(4, cell_is_running(c));
    PanelHW_SetLed(5, !cell_is_running(c));

    /*
     * 当前显示内容指示灯。
     */
    PanelHW_SetLed(6,
                   p->show_type == PANEL_SHOW_TARGET ||
                       p->show_type == PANEL_SHOW_PARAM);

    PanelHW_SetLed(7,
                   p->show_type == PANEL_SHOW_CURRENT);

    /*
     * 当前选中的恒温池。
     */
    PanelHW_SetLed(8, p->active_cell == 0);
    PanelHW_SetLed(9, p->active_cell == 1);
}

/*
 * 获取当前参数项的数值。
 *
 * 参数设置模式下，数码管显示哪个数，
 * 就由这个函数决定。
 */
static int16_t get_current_param_value(const TempCell_t *c,
                                       ProgramParamIndex_t index)
{
    switch (index)
    {
    case PROG_PARAM_START_TEMP:
        return c->program.start_temp_x10;

    case PROG_PARAM_START_HOLD:
        return c->program.start_hold_s;

    case PROG_PARAM_RAMP_RATE:
        return c->program.ramp_x10_per_min;

    case PROG_PARAM_NEXT_TEMP:
        return c->program.next_temp_x10;

    case PROG_PARAM_WAIT_TIME:
        return c->program.wait_s;

    case PROG_PARAM_REPEAT_TIMES:
        return c->program.repeat_times;

    default:
        return 0;
    }
}

/*
 * 刷新数码管和 LED。
 *
 * 这是面板显示的总入口。
 */
static void refresh_display(TempPanel_t *p)
{
    TempCell_t *c = cur_cell(p);

    /*
     * 如果当前恒温池有错误，
     * 优先显示错误代码。
     */
    if (c->error != PANEL_ERR_NONE)
    {
        p->show_type = PANEL_SHOW_ERROR;
        PanelHW_DisplayError(c->error);
        refresh_leds(p);
        return;
    }

    /*
     * 参数设置模式下：
     * 刚进入/刚切换参数时 → 先显示参数缩写名 1 秒
     * 之后 → 显示参数值
     */
    if (p->mode == PANEL_MODE_PARAM_SET)
    {
        p->show_type = PANEL_SHOW_PARAM;
        if (p->param_show_name)
        {
            TM1638_ShowString(&htm1638, "Strt"); // 改成 "8888" 或硬编码测试
            // const char *names[] = {"Strt", "StHd", "rAtE", "nExt", "uAit", "rEPt"};
            // TM1638_ShowString(&htm1638, names[p->param_index]);
        }
        else
        {
            PanelHW_DisplayNumber(get_current_param_value(c, p->param_index));
        }
        refresh_leds(p);
        return;
    }

    /*
     * Normal 或 External 模式下：
     * 根据 show_type 显示当前温度或目标温度。
     */
    if (p->show_type == PANEL_SHOW_TARGET)
    {
        PanelHW_DisplayTempX10(c->target_temp_x10);
    }
    else
    {
        PanelHW_DisplayTempX10(c->current_temp_x10);
    }

    refresh_leds(p);
}

/* ============================================================
 * 5. 公开 API 实现
 * ============================================================ */

/* 全局面板对象 */
TempPanel_t g_panel;

void TempPanel_Init(TempPanel_t *p)
{
    if (p == NULL)
        return;

    p->mode = PANEL_MODE_NORMAL;
    p->active_cell = 0;
    p->show_type = PANEL_SHOW_CURRENT;
    p->param_index = PROG_PARAM_START_TEMP;
    p->editing = false;
    p->edit_tick_ms = 0;
    p->display_tick_ms = 0;
    p->second_tick_ms = 0;

    for (int i = 0; i < PANEL_CELL_NUM; i++)
    {
        TempCell_t *c = &p->cell[i];
        c->current_temp_x10 = 250;
        c->target_temp_x10 = 250;
        c->command_temp_x10 = 250;
        c->last_temp_update_ms = 0;
        c->run_mode = CELL_STOP;
        c->error = PANEL_ERR_NONE;
        c->pid_enabled = false;
        c->program_phase = 0;
        c->program_timer_s = 0;
        c->program_interval_done = 0;
        c->program_step_x10 = 0;
        c->program_next_target_x10 = 0;

        c->program.start_temp_x10 = 250;
        c->program.start_hold_s = 300;
        c->program.ramp_x10_per_min = 10;
        c->program.next_temp_x10 = 110;
        c->program.wait_s = 30;
        c->program.repeat_times = 79;
    }

    p->param_show_name = false;
    p->param_show_tick_ms = 0;

    refresh_display(p);
}

void TempPanel_Task(TempPanel_t *p, uint32_t now_ms)
{
    if (p == NULL)
        return;

    TempCell_t *c = cur_cell(p);

    /* 参数名展示 1 秒后自动切回数值 */
    if (p->param_show_name && (now_ms - p->param_show_tick_ms >= 1000))
    {
        p->param_show_name = false;
    }

    for (uint8_t i = 0; i < PANEL_CELL_NUM; i++)
    {
        TempCell_t *ci = &p->cell[i];
        if (ci->run_mode != CELL_STOP && ci->error == PANEL_ERR_NONE)
        {
            if (ci->last_temp_update_ms != 0 &&
                (now_ms - ci->last_temp_update_ms) > PANEL_SENSOR_TIMEOUT_MS)
            {
                set_error_and_stop(p, i, PANEL_ERR_E132_SENSOR);
            }
        }
    }

    if (now_ms - p->second_tick_ms >= 1000)
    {
        p->second_tick_ms = now_ms;

        for (uint8_t i = 0; i < PANEL_CELL_NUM; i++)
        {
            TempCell_t *ci = &p->cell[i];
            if (ci->run_mode == CELL_RUN_JUMP)
            {
                update_jump_control(p, i, 1000);
            }
            else if (ci->run_mode == CELL_RUN_PROGRAM)
            {
                update_program_control_1s(p, i);
            }
        }
    }

    if (p->mode == PANEL_MODE_NORMAL && !p->editing && c->error == PANEL_ERR_NONE)
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
        p->show_type = PANEL_SHOW_CURRENT;
    }

    refresh_display(p);
}

void TempPanel_KeyEvent(TempPanel_t *p,
                        PanelKey_t key,
                        PanelKeyEvent_t evt,
                        uint32_t now_ms)
{
    if (p == NULL)
        return;
    TempCell_t *c = cur_cell(p);

    if (evt != PANEL_KEY_EVT_SHORT)
        return;

    switch (key)
    {

    case PANEL_KEY_SWITCH:
        if (!cell_is_running(c))
        {
            p->active_cell = (p->active_cell + 1) % PANEL_CELL_NUM;
            p->editing = false;
            p->show_type = PANEL_SHOW_CURRENT;
        }
        else
        {
            PanelHW_BlinkOnce();
        }
        break;

    case PANEL_KEY_MODE:
        if (!cell_is_running(c))
        {
            p->mode = (PanelMode_t)((p->mode + 1) % 3);
            p->editing = false;
            p->show_type = PANEL_SHOW_CURRENT;
            /* 进入参数设置模式时, 显示当前参数缩写名 */
            if (p->mode == PANEL_MODE_PARAM_SET)
            {
                p->param_show_name = true;
                p->param_show_tick_ms = now_ms;
            }
        }
        else
        {
            PanelHW_BlinkOnce();
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
        }
        else
        {
            cell_stop(p, active(p));
            p->show_type = PANEL_SHOW_CURRENT;
        }
        break;

    case PANEL_KEY_ENTER:
        if (p->mode == PANEL_MODE_PARAM_SET && !cell_is_running(c))
        {
            p->param_index =
                (ProgramParamIndex_t)((p->param_index + 1) % PROG_PARAM_COUNT);
            p->param_show_name = true;
            p->param_show_tick_ms = now_ms;
        }
        break;

    case PANEL_KEY_UP:
        if (!cell_is_running(c) && p->mode == PANEL_MODE_NORMAL)
        {
            c->target_temp_x10 += 1;
            if (c->target_temp_x10 > PANEL_TEMP_MAX_X10)
                c->target_temp_x10 = PANEL_TEMP_MAX_X10;
            p->editing = true;
            p->edit_tick_ms = now_ms;
            p->show_type = PANEL_SHOW_TARGET;
        }
        break;

    case PANEL_KEY_DOWN:
        if (!cell_is_running(c) && p->mode == PANEL_MODE_NORMAL)
        {
            c->target_temp_x10 -= 1;
            if (c->target_temp_x10 < PANEL_TEMP_MIN_X10)
                c->target_temp_x10 = PANEL_TEMP_MIN_X10;
            p->editing = true;
            p->edit_tick_ms = now_ms;
            p->show_type = PANEL_SHOW_TARGET;
        }
        break;

    default:
        break;
    }
}

void TempPanel_UpdateMeasuredTemp(TempPanel_t *p,
                                  uint8_t cell_idx,
                                  int16_t temp_x10,
                                  uint32_t now_ms)
{
    if (p == NULL || cell_idx >= PANEL_CELL_NUM)
        return;

    TempCell_t *c = &p->cell[cell_idx];
    c->current_temp_x10 = temp_x10;
    c->last_temp_update_ms = now_ms;

    if (c->error == PANEL_ERR_E132_SENSOR)
    {
        c->error = PANEL_ERR_NONE;
    }
}

void TempPanel_SetWaterError(TempPanel_t *p, bool error)
{
    if (p == NULL)
        return;
    for (uint8_t i = 0; i < PANEL_CELL_NUM; i++)
    {
        set_error_and_stop(p, i, error ? PANEL_ERR_E1_WATER : PANEL_ERR_NONE);
    }
}

void TempPanel_SetPeltierError(TempPanel_t *p,
                               uint8_t cell,
                               bool error)
{
    if (p == NULL || cell >= PANEL_CELL_NUM)
        return;
    set_error_and_stop(p, cell, error ? PANEL_ERR_E3_PELTIER : PANEL_ERR_NONE);
}

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
    default:
        return PANEL_KEY_NONE;
    }
}

__attribute__((weak)) void PanelHW_DisplayTempX10(int16_t temp_x10)
{
    float temp_f = (float)temp_x10 * 0.1f;
    TM1638_ShowFloat(&htm1638, temp_f, 1);
}

__attribute__((weak)) void PanelHW_DisplayNumber(int16_t value)
{
    float v = (float)value;
    if (v > 9999.0f)
        v = 9999.0f;
    if (v < -999.0f)
        v = -999.0f;
    TM1638_ShowFloat(&htm1638, v, 0);
}

__attribute__((weak)) void PanelHW_DisplayError(PanelError_t err)
{
    TM1638_ClearDisplay(&htm1638);

    if (err == PANEL_ERR_NONE)
        return;

    uint8_t code = (uint8_t)err;
    uint8_t d3 = code / 100;
    uint8_t d2 = (code / 10) % 10;
    uint8_t d1 = code % 10;

    TM1638_ShowDigit(&htm1638, 0, 0x0E, false);

    if (d3 > 0)
    {
        TM1638_ShowDigit(&htm1638, 1, d3, false);
        TM1638_ShowDigit(&htm1638, 2, d2, false);
        TM1638_ShowDigit(&htm1638, 3, d1, false);
    }
    else
    {
        TM1638_ShowDigit(&htm1638, 1, 0x00, false);
        TM1638_ShowDigit(&htm1638, 2, d2, false);
        TM1638_ShowDigit(&htm1638, 3, d1, false);
    }
}

__attribute__((weak)) void PanelHW_SetLed(uint8_t led_id, bool on)
{
    if (led_id >= 1 && led_id <= 7)
    {
        TM1638_SetLED(&htm1638, led_id, on);
    }
}

__attribute__((weak)) void PanelHW_BlinkOnce(void)
{
    TM1638_SetLED(&htm1638, 4, true);
    TM1638_SetLED(&htm1638, 4, false);
}

extern PID_TypeDef temp_pid;

__attribute__((weak)) void Control_StartPid(uint8_t cell)
{
    (void)cell;
    /*
     * PID_TypeDef *pid = get_pid_for_cell(cell);
     * PID_Reset(pid);
     * if (cell == 0) {
     *     HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
     *     HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
     * }
     */
}

__attribute__((weak)) void Control_StopPid(uint8_t cell)
{
    (void)cell;
    /*
     * if (cell == 0) {
     *     HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
     *     HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
     * }
     * PID_TypeDef *pid = get_pid_for_cell(cell);
     * PID_Reset(pid);
     */
}

__attribute__((weak)) void Control_SetTargetTemp(uint8_t cell, int16_t target_x10)
{
    (void)cell;
    (void)target_x10;
    /*
     * float target = (float)target_x10 * 0.1f;
     * PID_TypeDef *pid = get_pid_for_cell(cell);
     * PID_SetTarget(pid, target);
     */
}

void Panel_Init(void)
{
    TM1638_HW_Init();
    TempPanel_Init(&g_panel);

    PID_Init(&temp_pid, 30.0f, 1.0f, 0.0f, 0.1f);
    PID_SetLimits(&temp_pid, 700.0f, 1699.0f, 700.0f, 1600.0f);

    TM1638_ShowFloat(&htm1638, 25.0f, 1);
    TM1638_SetLED(&htm1638, 4, true);
    TM1638_SetLED(&htm1638, 5, true);
}