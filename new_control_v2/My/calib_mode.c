/**
 ******************************************************************************
 * @file    calib_mode.c
 * @brief   自动化标定模式实现
 * @author  Bisemin Team
 * @version V1.0.0
 * @date    2026-07-02
 *
 * @details
 * 固定占空比扫描（-0.35 ~ +0.35，步进 0.05），每步等待温度稳定后记录
 * 两个通道各自的稳态平均温度。全部 15 步完成后将结果写入 Flash。
 * 此模块由 AppControl_Task 每周期驱动，不创建独立线程。
 ******************************************************************************
 */

#include "calib_mode.h"
#include "app_control.h"
#include "cmsis_os.h"
#include "flash_storage.h"
#include "pid_controller.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

#define CALIB_LOG_PERIOD_MS 1000U
/* 外部温度数据（由 ISR 更新） ----------------------------------------------*/
extern volatile float    Sys_Temperatures[4];
extern volatile uint32_t Sys_TempUpdateCount[4];
extern volatile uint32_t Sys_TempUpdateTick[4];

/* 全局变量定义 --------------------------------------------------------------*/

volatile uint8_t      g_calib_mode_active  = 0U;
volatile CalibState_t g_calib_state        = CALIB_IDLE;
volatile uint8_t      g_calib_step_idx     = 0U;
volatile CalibStep_t  g_calib_result[CALIB_DUTY_COUNT];
volatile uint8_t      g_calib_cell         = 0U;
volatile uint32_t     g_calib_error        = 0U;
volatile uint8_t      g_calib_show_duty    = 1U;
volatile uint8_t      g_calib_running      = 0U;
volatile float        g_calib_temp_ch0     = 0.0f;
volatile float        g_calib_temp_ch1     = 0.0f;
/* 标定状态机私有变量 --------------------------------------------------------*/

static uint32_t s_calib_step_start_ms  = 0U;   /* 当前步开始 tick */
static uint32_t s_calib_stable_start_ms = 0U;  /* 首次判定稳定时的 tick */
static uint8_t  s_calib_stable_reached  = 0U;   /* 是否已进入稳态区间 */
static uint32_t s_calib_display_switch_ms = 0U; /* 数码管显示切换 tick */

/* 温度滑动窗口（用于稳定判定） ----------------------------------------------*/
static float    s_calib_temp_window[CALIB_WINDOW_SAMPLES];
static uint16_t s_calib_window_idx     = 0U;

/* 稳定区间采样累加（两个通道分别累加） ----------------------------------------*/
static float     s_calib_stable_sum_ch0 = 0.0f;
static float     s_calib_stable_sum_ch1 = 0.0f;
static uint32_t  s_calib_stable_count   = 0U;
static uint32_t  s_calib_last_temp_tick = 0U;
static uint32_t  s_calib_last_log_ms    = 0U;
/* CRC16 计算 ----------------------------------------------------------------*/

/**
 * @brief  CRC16-CCITT 查表计算
 */
static uint16_t CalibMode_CRC16(const uint8_t *data, uint32_t len)
{
    static const uint16_t table[256] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
        0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
        0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
        0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
        0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
        0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
        0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
        0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
        0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
        0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
        0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
        0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
        0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
        0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
        0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
        0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
        0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
        0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
        0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
        0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
        0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
        0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
        0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
    };

    uint16_t crc = 0xFFFFU;
    uint32_t i;

    for (i = 0U; i < len; i++)
    {
        crc = (uint16_t)((crc << 8U) ^ table[((crc >> 8U) ^ data[i]) & 0xFFU]);
    }

    return crc;
}

/* 标定步占空比查表 ----------------------------------------------------------*/

/**
 * @brief  根据步索引计算占空比
 * @param  step_idx 步索引 (0 ~ 14)
 * @return float 占空比
 */
float CalibMode_StepDuty(uint8_t step_idx)
{
#ifdef CALIB_FAST_TEST
    (void)step_idx;
    return CALIB_FAST_DUTY;
#else
    return CALIB_DUTY_START + (float)step_idx * CALIB_DUTY_STEP;
#endif
}

/* 温度辅助函数 --------------------------------------------------------------*/

/**
 * @brief  获取被标定 Cell 的平均温度，同时更新两个通道的实时温度
 *         （供数码管显示和稳定判定使用）
 * @return float 当前平均温度 (°C)
 */
float CalibMode_GetCellTemp(void)
{
    uint8_t cell = (uint8_t)g_calib_cell;
    uint8_t a = (uint8_t)(cell * 2U);
    uint8_t b = (uint8_t)(a + 1U);

    float t0 = Sys_Temperatures[a];
    float t1 = Sys_Temperatures[b];

    g_calib_temp_ch0 = t0;
    g_calib_temp_ch1 = t1;

    return (t0 + t1) * 0.5f;
}

/**
 * @brief  检查是否有新的温度数据到达
 * @return uint8_t 1 = 有更新，0 = 无更新
 */
static uint8_t CalibMode_IsNewTempData(void)
{
    uint8_t cell = (uint8_t)g_calib_cell;
    uint8_t a = (uint8_t)(cell * 2U);
    uint32_t tick_a = Sys_TempUpdateTick[a];

    if (tick_a != s_calib_last_temp_tick)
    {
        s_calib_last_temp_tick = tick_a;
        return 1U;
    }

    return 0U;
}

/**
 * @brief  将当前温度值推入滑动窗口
 * @param  temp 当前温度值（平均值，用于稳定判定）
 */
static void CalibMode_PushWindow(float temp)
{
    s_calib_temp_window[s_calib_window_idx % CALIB_WINDOW_SAMPLES] = temp;
    s_calib_window_idx++;
}

/**
 * @brief  检查窗口是否已填满
 * @return uint8_t 1 = 已满，0 = 未满
 */
static uint8_t CalibMode_IsWindowFull(void)
{
    return (s_calib_window_idx >= CALIB_WINDOW_SAMPLES) ? 1U : 0U;
}

/**
 * @brief  计算滑动窗口内温度的最大差值（max - min）
 * @return float 温差 (°C)，窗口未满时返回极大值
 */
static float CalibMode_GetWindowSpan(void)
{
    uint16_t start;
    uint16_t i;
    float max_val, min_val;

    if (!CalibMode_IsWindowFull())
        return 999.0f;

    start = (s_calib_window_idx - CALIB_WINDOW_SAMPLES) % CALIB_WINDOW_SAMPLES;

    max_val = s_calib_temp_window[start];
    min_val = s_calib_temp_window[start];

    for (i = 1U; i < CALIB_WINDOW_SAMPLES; i++)
    {
        float val = s_calib_temp_window[(start + i) % CALIB_WINDOW_SAMPLES];

        if (val > max_val) max_val = val;
        if (val < min_val) min_val = val;
    }

    return max_val - min_val;
}

/**
 * @brief  计算指定通道稳定区间的平均温度
 * @param  sum 该通道的累加和
 * @return float 平均温度 (°C)
 */
static float CalibMode_CalcStableAverageCh(float sum)
{
    if (s_calib_stable_count == 0U)
        return CalibMode_GetCellTemp();

    return sum / (float)s_calib_stable_count;
}

/* 标定数据写入 Flash --------------------------------------------------------*/

static void CalibMode_UartSend(const char *str);

/**
 * @brief  根据 Cell 编号获取对应的 Flash 存储区偏移
 * @param  cell Cell 编号 (0 或 1)
 * @return uint32_t Flash 存储区内偏移
 */
static uint32_t CalibMode_GetFlashOffset(uint8_t cell)
{
    return (cell == 0U) ? CALIB_FLASH_OFFSET_CELL0 : CALIB_FLASH_OFFSET_CELL1;
}

/**
 * @brief  将标定结果写入 Flash
 * @return uint8_t 0 = 成功，非0 = 失败
 */
static uint8_t CalibMode_WriteToFlash(void)
{
    CalibFlashData_t flash_data;
    FlashStorage_Status_t ret;
    uint32_t primask;
    uint8_t i;

    memset(&flash_data, 0, sizeof(flash_data));

    flash_data.magic = CALIB_FLASH_MAGIC;
    flash_data.cell  = (uint8_t)g_calib_cell;

    for (i = 0U; i < CALIB_DUTY_COUNT; i++)
    {
#ifdef CALIB_FAST_TEST
        /* 快速测试：仅第 0 步为真实数据，其余 45 步复制填充 */
        uint8_t src = (i < CALIB_SCAN_COUNT) ? i : 0U;
#else
        uint8_t src = i;
#endif
        flash_data.step[i].duty     = (float)g_calib_result[src].duty;
        flash_data.step[i].temp_ch0 = (float)g_calib_result[src].temp_ch0;
        flash_data.step[i].temp_ch1 = (float)g_calib_result[src].temp_ch1;
        flash_data.step[i].valid    = g_calib_result[src].valid;
        flash_data.step[i].settled  = g_calib_result[src].settled;
    }

    /* 计算 CRC（不包含 crc16 字段本身） */
    flash_data.crc16 = CalibMode_CRC16((const uint8_t *)&flash_data,
                                       sizeof(flash_data) - sizeof(flash_data.crc16));

    /* 擦除目标页（直接调用 HAL，绕过 flash_storage.c） */
    {
        uint32_t           calib_offset = CalibMode_GetFlashOffset((uint8_t)g_calib_cell);
        uint32_t           abs_addr     = FLASH_STORAGE_START_ADDR + calib_offset;
        char               buf[80];
        int                len;

        len = snprintf(buf, sizeof(buf),
                       "CALIB,FLASH_ERASE,CELL:%u,ADDR:0x%08lX\r\n",
                       (unsigned int)g_calib_cell,
                       (unsigned long)abs_addr);
        if (len > 0 && len < (int)sizeof(buf))
            CalibMode_UartSend(buf);

        /* 关全局中断，防止 Flash 操作期间 ISR 取指导致 HardFault */
        primask = __get_PRIMASK();
        __disable_irq();

        /* 直接调 HAL 页擦除（需先解锁 Flash） */
        {
            FLASH_EraseInitTypeDef erase_init;
            uint32_t               page_err = 0U;
            HAL_StatusTypeDef      hal_ret;

            memset(&erase_init, 0, sizeof(erase_init));
            erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
            erase_init.Banks     = FLASH_BANK_2;
            erase_init.Page      = 124U;  /* PNB 硬件截断 → Bank2 页 60 = 0x0803E000 */
            erase_init.NbPages   = 1U;

            HAL_FLASH_Unlock();
            hal_ret = HAL_FLASHEx_Erase(&erase_init, &page_err);
            HAL_FLASH_Lock();

            ret = (hal_ret == HAL_OK) ? FLASH_STORAGE_OK : FLASH_STORAGE_ERROR_ERASE;
        }

        __set_PRIMASK(primask);

        len = snprintf(buf, sizeof(buf),
                       "CALIB,FLASH_ERASE_DONE,STATUS:%u\r\n",
                       (unsigned int)ret);
        if (len > 0 && len < (int)sizeof(buf))
            CalibMode_UartSend(buf);

        /* 验证擦除是否真正生效：关中断内立即读回 */
        {
            uint32_t verify;

            primask = __get_PRIMASK();
            __disable_irq();
            verify = *((volatile uint32_t *)abs_addr);
            __set_PRIMASK(primask);

            len = snprintf(buf, sizeof(buf),
                           "CALIB,FLASH_ERASE_VERIFY,ADDR:0x%08lX,DATA:0x%08lX\r\n",
                           (unsigned long)abs_addr, (unsigned long)verify);
            if (len > 0 && len < (int)sizeof(buf))
                CalibMode_UartSend(buf);
        }

        if (ret == FLASH_STORAGE_OK)
        {
            len = snprintf(buf, sizeof(buf),
                           "CALIB,FLASH_WRITE,ADDR:0x%08lX,SIZE:%u\r\n",
                           (unsigned long)abs_addr,
                           (unsigned int)sizeof(flash_data));
            if (len > 0 && len < (int)sizeof(buf))
                CalibMode_UartSend(buf);

            /* 关全局中断 */
            primask = __get_PRIMASK();
            __disable_irq();

            ret = FlashStorage_Write(calib_offset,
                                     (const uint8_t *)&flash_data,
                                     sizeof(flash_data));

            __set_PRIMASK(primask);

            len = snprintf(buf, sizeof(buf),
                           "CALIB,FLASH_WRITE_DONE,STATUS:%u,CRC:0x%04X\r\n",
                           (unsigned int)ret,
                           (unsigned int)flash_data.crc16);
            if (len > 0 && len < (int)sizeof(buf))
                CalibMode_UartSend(buf);
        }
    }
    if (ret != FLASH_STORAGE_OK)
        return (uint8_t)ret;   /* 返回实际 Flash 错误码供串口输出诊断 */

    return 0U;
}

/* USART2 输出辅助 -----------------------------------------------------------*/

/**
 * @brief  通过 USART2 发送文本字符串（阻塞模式）
 * @param  str 以 null 结尾的字符串
 */
static void CalibMode_UartSend(const char *str)
{
    uint16_t len = 0U;
    const char *p = str;

    while (*p != '\0')
    {
        len++;
        p++;
    }

    if (len > 0U)
    {
        HAL_UART_Transmit(&huart2, (const uint8_t *)str, len, 20U);
    }
}

/* 标定状态机核心 ------------------------------------------------------------*/

/**
 * @brief  初始化标定流程：配置 DRV 通道进入开环模式
 */
static void CalibMode_LogPeriodic(uint32_t now_ms)
{
    char buf[176];
    int len;
    uint8_t step;
    float duty;
    float temp_avg;
    float span;
    uint32_t elapsed;

    if (g_calib_mode_active == 0U)
        return;

    if ((now_ms - s_calib_last_log_ms) < CALIB_LOG_PERIOD_MS)
        return;
    s_calib_last_log_ms = now_ms;

    step = (uint8_t)g_calib_step_idx;
    duty = (step < CALIB_SCAN_COUNT) ? CalibMode_StepDuty(step) : 0.0f;
    temp_avg = CalibMode_GetCellTemp();
    span = (CalibMode_IsWindowFull() != 0U) ? CalibMode_GetWindowSpan() : -1.0f;
    elapsed = (s_calib_step_start_ms == 0U) ? 0U : (now_ms - s_calib_step_start_ms);

    len = snprintf(buf, sizeof(buf),
                   "CALIB,RUN,STATE:%u,CELL:%u,STEP:%u,DUTY:%+.2f,"
                   "T0:%.2f,T1:%.2f,TAVG:%.2f,SPAN:%.3f,ELAP:%lu\r\n",
                   (unsigned int)g_calib_state,
                   (unsigned int)g_calib_cell,
                   (unsigned int)step,
                   (double)duty,
                   (double)g_calib_temp_ch0,
                   (double)g_calib_temp_ch1,
                   (double)temp_avg,
                   (double)span,
                   (unsigned long)elapsed);
    if (len > 0 && len < (int)sizeof(buf))
    {
        CalibMode_UartSend(buf);
    }
}

static void CalibMode_InitStep(void)
{
    s_calib_step_start_ms   = 0U;
    s_calib_stable_start_ms = 0U;
    s_calib_stable_reached  = 0U;
    s_calib_window_idx      = 0U;
    s_calib_stable_sum_ch0  = 0.0f;
    s_calib_stable_sum_ch1  = 0.0f;
    s_calib_stable_count    = 0U;
    s_calib_last_temp_tick  = 0U;

    memset(s_calib_temp_window, 0, sizeof(s_calib_temp_window));
}

/**
 * @brief  设置当前步的占空比（开环，无 PID）
 */
static void CalibMode_SetStepDuty(void)
{
    float duty = CalibMode_StepDuty((uint8_t)g_calib_step_idx);
    uint8_t cell = (uint8_t)g_calib_cell;
    uint8_t first = (uint8_t)(cell * 2U);

    /* 设置本 Cell 的两个 DRV 通道 + 共享通道 */
    (void)AppControl_SetDrvDuty(first, duty);
    (void)AppControl_SetDrvDuty((uint8_t)(first + 1U), duty);
    (void)AppControl_SetDrvDuty(4U, APP_CONTROL_SHARED_CH5_DUTY);
}

/**
 * @brief  结束当前步：记录两个通道的稳态温度，进入下一步或完成
 */
static void CalibMode_FinishStep(uint8_t settled)
{
    float avg_temp_ch0 = CalibMode_CalcStableAverageCh(s_calib_stable_sum_ch0);
    float avg_temp_ch1 = CalibMode_CalcStableAverageCh(s_calib_stable_sum_ch1);

    g_calib_result[g_calib_step_idx].duty     = CalibMode_StepDuty((uint8_t)g_calib_step_idx);
    g_calib_result[g_calib_step_idx].temp_ch0 = avg_temp_ch0;
    g_calib_result[g_calib_step_idx].temp_ch1 = avg_temp_ch1;
    g_calib_result[g_calib_step_idx].valid    = (settled != 0U) ? 1U : 0U;
    g_calib_result[g_calib_step_idx].settled  = settled;

    /* 通过 USART2 发送当前步结果 */
    {
        float duty = CalibMode_StepDuty((uint8_t)g_calib_step_idx);
        char buf[80];
        int len = snprintf(buf, sizeof(buf),
            "CALIB,CELL:%u,STEP:%u,DUTY:%+.2f,T0:%.1f,T1:%.1f,VALID:%u,SETTLED:%u\r\n",
            (unsigned int)g_calib_cell,
            (unsigned int)g_calib_step_idx + 1U,
            (double)duty,
            (double)avg_temp_ch0,
            (double)avg_temp_ch1,
            (unsigned int)((settled != 0U) ? 1U : 0U),
            (unsigned int)settled);
        if (len > 0 && len < (int)sizeof(buf))
        {
            CalibMode_UartSend(buf);
        }
    }

    g_calib_step_idx++;

    if (g_calib_step_idx >= CALIB_SCAN_COUNT)
    {
        /* 全部标定步完成 */
        {
            uint8_t write_ret = CalibMode_WriteToFlash();
            if (write_ret == 0U)
            {
                g_calib_state = CALIB_DONE;
            }
            else
            {
                g_calib_error = (uint32_t)write_ret;
                g_calib_state = CALIB_FAULT;
            }
        }
    }
    else
    {
        /* 进入下一步 */
        CalibMode_InitStep();
        g_calib_state = CALIB_RUN;
    }
}

/* 公有 API 实现 -------------------------------------------------------------*/

/* 全局静态标志，确保 DONE/FAULT/STOP 消息只发送一次 */
static uint8_t s_calib_done_sent   = 0U;
static uint8_t s_calib_fault_sent  = 0U;
static uint8_t s_calib_stop_sent   = 0U;

/**
 * @brief  启动自动标定流程
 * @param  cell Cell 编号 (0 或 1)
 */
void CalibMode_Start(uint8_t cell)
{
    if (cell > 1U)
    {
        CalibMode_UartSend("CALIB,START_REJECT,BAD_CELL\r\n");
        return;
    }

    if (g_calib_mode_active != 0U)
    {
        g_calib_error = 2U;
        CalibMode_UartSend("CALIB,START_REJECT,ACTIVE\r\n");
        return;
    }

    /* 重置全局状态 */
    g_calib_cell         = cell;
    g_calib_step_idx     = 0U;
    g_calib_error        = 0U;
    g_calib_state        = CALIB_INIT;
    g_calib_mode_active  = 1U;
    g_calib_running      = 1U;
    g_calib_show_duty    = 1U;
    g_calib_temp_ch0     = 0.0f;
    g_calib_temp_ch1     = 0.0f;
    s_calib_last_log_ms  = 0U;
    /* 重置单次发送标志 */
    s_calib_done_sent   = 0U;
    s_calib_fault_sent  = 0U;
    s_calib_stop_sent   = 0U;

    memset((void *)g_calib_result, 0, sizeof(g_calib_result));

    {
        char buf[40];
        int len = snprintf(buf, sizeof(buf),
            "CALIB,START,CELL:%u\r\n",
            (unsigned int)cell);
        if (len > 0 && len < (int)sizeof(buf))
        {
            CalibMode_UartSend(buf);
        }
    }
}

/**
 * @brief  标定模式主任务，由 AppControl_Task 每周期调用
 * @param  now_ms 当前系统 tick (ms)
 */
void CalibMode_Task(uint32_t now_ms)
{
    CalibMode_LogPeriodic(now_ms);

    switch (g_calib_state)
    {
    case CALIB_IDLE:
        break;

    case CALIB_INIT:
    {
        /* 初始化 Flash 存储 */
        FlashStorage_Init();
        CalibMode_InitStep();
        {
            char buf[40];
            int len = snprintf(buf, sizeof(buf),
                "CALIB,INIT,CELL:%u\r\n",
                (unsigned int)g_calib_cell);
            if (len > 0 && len < (int)sizeof(buf))
            {
                CalibMode_UartSend(buf);
            }
        }
        g_calib_state = CALIB_RUN;
        break;
    }

    case CALIB_RUN:
    {
        /* 设定当前步占空比，进入等待稳态 */
        CalibMode_SetStepDuty();
        s_calib_step_start_ms = now_ms;
        {
            float duty = CalibMode_StepDuty((uint8_t)g_calib_step_idx);
            char buf[64];
            int len = snprintf(buf, sizeof(buf),
                "CALIB,STEP_START,CELL:%u,STEP:%u,DUTY:%+.2f\r\n",
                (unsigned int)g_calib_cell,
                (unsigned int)g_calib_step_idx,
                (double)duty);
            if (len > 0 && len < (int)sizeof(buf))
            {
                CalibMode_UartSend(buf);
            }
        }
        g_calib_state = CALIB_WAIT_STABLE;
        break;
    }

    case CALIB_WAIT_STABLE:
    {
        uint32_t elapsed = now_ms - s_calib_step_start_ms;

        /* 超时检查 */
        if (elapsed >= (CALIB_MAX_WAIT_SECONDS * 1000U))
        {
            CalibMode_FinishStep(0U);
            break;
        }

        /* 等待新温度数据 */
        if (!CalibMode_IsNewTempData())
            break;

        {
            float temp = CalibMode_GetCellTemp();

            /* 推入滑动窗口 */
            CalibMode_PushWindow(temp);

            if (CalibMode_IsWindowFull())
            {
                float span = CalibMode_GetWindowSpan();

                if (span < CALIB_STABLE_THRESHOLD)
                {
                    if (s_calib_stable_reached == 0U)
                    {
                        /* 首次判定稳定，初始化双通道累加器 */
                        s_calib_stable_reached  = 1U;
                        s_calib_stable_start_ms = now_ms;
                        s_calib_stable_sum_ch0  = 0.0f;
                        s_calib_stable_sum_ch1  = 0.0f;
                        s_calib_stable_count    = 0U;
                    }

                    /* 累加两个通道各自的稳态数据 */
                    s_calib_stable_sum_ch0 += g_calib_temp_ch0;
                    s_calib_stable_sum_ch1 += g_calib_temp_ch1;
                    s_calib_stable_count   += 1U;

                    /* 检查是否已持续稳定足够时间 */
                    if ((now_ms - s_calib_stable_start_ms) >= (CALIB_STABLE_SECONDS * 1000U))
                    {
                        CalibMode_FinishStep(1U);
                    }
                }
                else
                {
                    /* 波动超出阈值，重置稳态标志 */
                    s_calib_stable_reached = 0U;
                }
            }
        }
        break;
    }

    case CALIB_DONE:
    {
        uint8_t cell  = (uint8_t)g_calib_cell;
        uint8_t first = (uint8_t)(cell * 2U);

        (void)AppControl_SetDrvDuty(first, 0.0f);
        (void)AppControl_SetDrvDuty((uint8_t)(first + 1U), 0.0f);
        (void)AppControl_SetDrvDuty(4U, 0.0f);

        /* 通过 USART2 发送完成信息（仅发送一次） */
        if (s_calib_done_sent == 0U)
        {
            s_calib_done_sent = 1U;
            char buf[48];
            int len = snprintf(buf, sizeof(buf),
                "CALIB,DONE,CELL:%u,STEPS:%u\r\n",
                (unsigned int)cell,
                (unsigned int)CALIB_DUTY_COUNT);
            if (len > 0 && len < (int)sizeof(buf))
            {
                CalibMode_UartSend(buf);
            }
        }

        g_calib_mode_active = 0U;
        g_calib_running     = 0U;
        break;
    }

    case CALIB_FAULT:
    {
        uint8_t cell  = (uint8_t)g_calib_cell;
        uint8_t first = (uint8_t)(cell * 2U);

        (void)AppControl_SetDrvDuty(first, 0.0f);
        (void)AppControl_SetDrvDuty((uint8_t)(first + 1U), 0.0f);
        (void)AppControl_SetDrvDuty(4U, 0.0f);

        /* 通过 USART2 发送故障信息（仅发送一次） */
        if (s_calib_fault_sent == 0U)
        {
            s_calib_fault_sent = 1U;
            char buf[48];
            int len = snprintf(buf, sizeof(buf),
                "CALIB,FAULT,CELL:%u,ERROR:%lu\r\n",
                (unsigned int)cell,
                (unsigned long)g_calib_error);
            if (len > 0 && len < (int)sizeof(buf))
            {
                CalibMode_UartSend(buf);
            }
        }

        g_calib_mode_active = 0U;
        g_calib_running     = 0U;
        break;
    }

    default:
        break;
    }
}

/**
 * @brief  Abort calibration with a fault code.
 * @param  error Calibration error code.
 */
void CalibMode_Fault(uint32_t error)
{
    if (g_calib_mode_active == 0U)
        return;

    if (error == 0U)
        error = CALIB_ERR_DRV_FAULT;

    g_calib_error = error;
    g_calib_state = CALIB_FAULT;
    g_calib_running = 0U;
}

/**
 * @brief  紧急停止标定
 */
void CalibMode_Stop(void)
{
    uint8_t cell  = (uint8_t)g_calib_cell;
    uint8_t first = (uint8_t)(cell * 2U);

    /* 所有通道占空比归零 */
    (void)AppControl_SetDrvDuty(first, 0.0f);
    (void)AppControl_SetDrvDuty((uint8_t)(first + 1U), 0.0f);
    (void)AppControl_SetDrvDuty(4U, 0.0f);

    /* 通过 USART2 发送紧急停止信息（仅发送一次） */
    if (s_calib_stop_sent == 0U)
    {
        s_calib_stop_sent = 1U;
        char buf[48];
        int len = snprintf(buf, sizeof(buf),
            "CALIB,STOP,CELL:%u,ERROR:%lu\r\n",
            (unsigned int)cell,
            (unsigned long)g_calib_error);
        if (len > 0 && len < (int)sizeof(buf))
        {
            CalibMode_UartSend(buf);
        }
    }

    g_calib_state       = CALIB_IDLE;
    g_calib_mode_active = 0U;
    g_calib_running     = 0U;
    g_calib_error       = 3U;
}

/**
 * @brief  从 Flash 读取指定 Cell 的上一次标定数据
 * @param  cell Cell 编号 (0 或 1)
 * @param  buf  输出缓冲区指针
 * @return uint8_t 0 = 成功，非0 = 失败
 */
uint8_t CalibMode_LoadFromFlash(uint8_t cell, CalibFlashData_t *buf)
{
    FlashStorage_Status_t ret;
    uint16_t stored_crc, calc_crc;
    uint32_t calib_offset = CalibMode_GetFlashOffset(cell);

    if (buf == 0)
        return 1U;

    ret = FlashStorage_Read(calib_offset,
                            (uint8_t *)buf,
                            sizeof(CalibFlashData_t));
    if (ret != FLASH_STORAGE_OK)
        return 2U;

    /* 校验魔数 */
    if (buf->magic != CALIB_FLASH_MAGIC)
        return 3U;

    /* 校验 CRC */
    stored_crc = buf->crc16;
    calc_crc   = CalibMode_CRC16((const uint8_t *)buf,
                                  sizeof(CalibFlashData_t) - sizeof(buf->crc16));
    if (stored_crc != calc_crc)
        return 4U;

    return 0U;
}

/* Flash 读写测试 --------------------------------------------------------------*/

/**
 * @brief  Flash 写入/读出/校验测试函数
 * @param  cell Cell 编号 (0 或 1)
 * @note   构造模拟标定数据 → 擦除 → 写入 → 读出 → 逐字节校验
 *         全程通过 USART2 输出日志
 *         写入前后关闭全局中断，防止 Flash 操作期间 ISR 取指卡死
 */
void CalibMode_FlashTest(uint8_t cell)
{
    CalibFlashData_t write_data;
    CalibFlashData_t read_data;
    FlashStorage_Status_t ret;
    uint32_t calib_offset;
    uint32_t primask;
    uint8_t page_idx;
    uint8_t i;
    char buf[128];
    int len;

    if (cell > 1U)
    {
        CalibMode_UartSend("FLASH_TEST,REJECT,BAD_CELL\r\n");
        return;
    }

    calib_offset = CalibMode_GetFlashOffset(cell);
    page_idx     = (uint8_t)(calib_offset / FLASH_STORAGE_PAGE_SIZE);

    /* ---- 构造模拟标定数据 ---- */
    memset(&write_data, 0, sizeof(write_data));
    write_data.magic = CALIB_FLASH_MAGIC;
    write_data.cell  = cell;

    for (i = 0U; i < CALIB_DUTY_COUNT; i++)
    {
        write_data.step[i].duty     = CalibMode_StepDuty(i);
        write_data.step[i].temp_ch0 = 25.0f + (float)i;
        write_data.step[i].temp_ch1 = 26.0f + (float)i;
        write_data.step[i].valid    = 1U;
        write_data.step[i].settled  = 1U;
    }
    write_data.crc16 = CalibMode_CRC16((const uint8_t *)&write_data,
                                       sizeof(write_data) - sizeof(write_data.crc16));

    len = snprintf(buf, sizeof(buf),
                   "FLASH_TEST,START,CELL:%u,PAGE:%u,ADDR:0x%08lX,SIZE:%u\r\n",
                   (unsigned int)cell, (unsigned int)page_idx,
                   (unsigned long)(FLASH_STORAGE_START_ADDR + calib_offset),
                   (unsigned int)sizeof(CalibFlashData_t));
    if (len > 0 && len < (int)sizeof(buf))
        CalibMode_UartSend(buf);

    /* ---- 擦除 ---- */
    CalibMode_UartSend("FLASH_TEST,ERASE_BEGIN\r\n");

    primask = __get_PRIMASK();
    __disable_irq();

    ret = FlashStorage_ErasePage(page_idx);

    __set_PRIMASK(primask);

    len = snprintf(buf, sizeof(buf),
                   "FLASH_TEST,ERASE_DONE,STATUS:%u\r\n",
                   (unsigned int)ret);
    if (len > 0 && len < (int)sizeof(buf))
        CalibMode_UartSend(buf);

    if (ret != FLASH_STORAGE_OK)
    {
        CalibMode_UartSend("FLASH_TEST,FAIL,ERASE\r\n");
        return;
    }

    /* ---- 写入 ---- */
    CalibMode_UartSend("FLASH_TEST,WRITE_BEGIN\r\n");

    primask = __get_PRIMASK();
    __disable_irq();

    ret = FlashStorage_Write(calib_offset,
                             (const uint8_t *)&write_data,
                             sizeof(write_data));

    __set_PRIMASK(primask);

    len = snprintf(buf, sizeof(buf),
                   "FLASH_TEST,WRITE_DONE,STATUS:%u\r\n",
                   (unsigned int)ret);
    if (len > 0 && len < (int)sizeof(buf))
        CalibMode_UartSend(buf);

    if (ret != FLASH_STORAGE_OK)
    {
        CalibMode_UartSend("FLASH_TEST,FAIL,WRITE\r\n");
        return;
    }

    /* ---- 读回 ---- */
    memset(&read_data, 0xFFU, sizeof(read_data));
    ret = FlashStorage_Read(calib_offset,
                            (uint8_t *)&read_data,
                            sizeof(read_data));

    len = snprintf(buf, sizeof(buf),
                   "FLASH_TEST,READ_DONE,STATUS:%u,MAGIC:0x%08lX,CELL:%u,CRC16:0x%04X\r\n",
                   (unsigned int)ret,
                   (unsigned long)read_data.magic,
                   (unsigned int)read_data.cell,
                   (unsigned int)read_data.crc16);
    if (len > 0 && len < (int)sizeof(buf))
        CalibMode_UartSend(buf);

    if (ret != FLASH_STORAGE_OK)
    {
        CalibMode_UartSend("FLASH_TEST,FAIL,READ\r\n");
        return;
    }

    /* ---- 逐字节校验 ---- */
    if (memcmp(&write_data, &read_data, sizeof(write_data)) == 0)
    {
        CalibMode_UartSend("FLASH_TEST,PASS,VERIFY_OK\r\n");
    }
    else
    {
        const uint8_t *w = (const uint8_t *)&write_data;
        const uint8_t *r = (const uint8_t *)&read_data;

        CalibMode_UartSend("FLASH_TEST,FAIL,VERIFY_MISMATCH\r\n");

        for (i = 0U; i < (uint8_t)sizeof(write_data); i++)
        {
            if (w[i] != r[i])
            {
                len = snprintf(buf, sizeof(buf),
                               "FLASH_TEST,MISMATCH,OFFSET:%u,W:0x%02X,R:0x%02X\r\n",
                               (unsigned int)i,
                               (unsigned int)w[i],
                               (unsigned int)r[i]);
                if (len > 0 && len < (int)sizeof(buf))
                    CalibMode_UartSend(buf);
                break;
            }
        }
    }
}

/* Flash 标定数据读取与串口输出 ------------------------------------------------*/

/**
 * @brief  读取指定 Cell 的 Flash 标定数据并通过 USART2 输出
 * @param  cell Cell 编号 (0 或 1)
 * @note   启动标定前调用，用于查看上一次标定结果
 */
void CalibMode_DumpFlashData(uint8_t cell)
{
    CalibFlashData_t data;
    uint8_t ret;
    uint8_t i;
    char buf[128];
    int len;

    if (cell > 1U)
        return;

    len = snprintf(buf, sizeof(buf),
                   "CALIB,LOAD,CELL:%u,BEGIN\r\n",
                   (unsigned int)cell);
    if (len > 0 && len < (int)sizeof(buf))
        CalibMode_UartSend(buf);

    memset(&data, 0, sizeof(data));
    ret = CalibMode_LoadFromFlash(cell, &data);

    if (ret != 0U)
    {
        len = snprintf(buf, sizeof(buf),
                       "CALIB,LOAD,CELL:%u,NOT_FOUND,ERR:%u\r\n",
                       (unsigned int)cell, (unsigned int)ret);
        if (len > 0 && len < (int)sizeof(buf))
            CalibMode_UartSend(buf);
        return;
    }

    len = snprintf(buf, sizeof(buf),
                   "CALIB,LOAD,CELL:%u,FOUND,CELL_STORED:%u,CRC:0x%04X\r\n",
                   (unsigned int)cell,
                   (unsigned int)data.cell,
                   (unsigned int)data.crc16);
    if (len > 0 && len < (int)sizeof(buf))
        CalibMode_UartSend(buf);

    for (i = 0U; i < CALIB_DUTY_COUNT; i++)
    {
        len = snprintf(buf, sizeof(buf),
                       "CALIB,STEP,%u,%+.2f,%.1f,%.1f,%u\r\n",
                       (unsigned int)i,
                       (double)data.step[i].duty,
                       (double)data.step[i].temp_ch0,
                       (double)data.step[i].temp_ch1,
                       (unsigned int)data.step[i].valid);
        if (len > 0 && len < (int)sizeof(buf))
            CalibMode_UartSend(buf);
    }

    len = snprintf(buf, sizeof(buf),
                   "CALIB,LOAD,CELL:%u,END\r\n",
                   (unsigned int)cell);
    if (len > 0 && len < (int)sizeof(buf))
        CalibMode_UartSend(buf);
}