/**
 * @file calib_mode.c
 * @brief Slew-limited duty calibration with per-point Flash journal/resume.
 * Each stable or timed-out point is committed before advancing. An unfinished
 * compatible scan resumes after the last durable point; a full scan restarts.
 * Driven by AppControl_Task without a separate thread.
 */

#include "calib_mode.h"
#include "app_control.h"
#include "app_fault_log.h"
#include "cmsis_os.h"
#include "flash_storage.h"
#include "pid_controller.h"
#include "usart.h"
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#define CALIB_LOG_PERIOD_MS 1000U
static uint32_t s_calib_last_output_ms = 0U;
static uint8_t s_calib_outputs_settled = 0U;
/* 外部温度数据（由 ISR 更新�?----------------------------------------------*/
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

static uint32_t s_calib_step_start_ms  = 0U;   /* 当前步开�?tick */
static uint32_t s_calib_stable_start_ms = 0U;  /* 首次判定稳定时的 tick */
static uint8_t  s_calib_stable_reached  = 0U;   /* 是否已进入稳态区�?*/
static uint32_t s_calib_display_switch_ms = 0U; /* 数码管显示切�?tick */

/* 温度滑动窗口（用于稳定判定） ----------------------------------------------*/
static float    s_calib_temp_window[CALIB_WINDOW_SAMPLES];
static uint16_t s_calib_window_idx     = 0U;

/* 稳定区间采样累加（两个通道分别累加�?----------------------------------------*/
static float     s_calib_stable_sum_ch0 = 0.0f;
static float     s_calib_stable_sum_ch1 = 0.0f;
static uint32_t  s_calib_stable_count   = 0U;
static uint32_t  s_calib_last_temp_tick_outer = 0U;
static uint32_t  s_calib_last_temp_tick_inner = 0U;
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
 * @param  step_idx 步索�?(0 ~ 14)
 * @return float 占空�?
 */
float CalibMode_StepDuty(uint8_t step_idx)
{
#ifdef CALIB_FAST_TEST
    (void)step_idx;
    return CALIB_FAST_DUTY;
#else
    if (step_idx >= (CALIB_DUTY_COUNT - 1U))
        return CALIB_DUTY_END;
    return CALIB_DUTY_START + (float)step_idx * CALIB_DUTY_STEP;
#endif
}

/* 温度辅助函数 --------------------------------------------------------------*/

/**
 * @brief  获取被标�?Cell 的平均温度，同时更新两个通道的实时温�?
 *         （供数码管显示和稳定判定使用�?
 * @return float 当前平均温度 (°C)
 */
float CalibMode_GetCellTemp(void)
{
    uint8_t cell = (uint8_t)g_calib_cell;
    uint8_t a = AppControl_GetCellOuterTempIndex(cell);
    uint8_t b = AppControl_GetCellInnerTempIndex(cell);
    float t0;
    float t1;

    if (a >= 4U || b >= 4U)
        return 0.0f;

    t0 = Sys_Temperatures[a];
    t1 = Sys_Temperatures[b];

    g_calib_temp_ch0 = t0;
    g_calib_temp_ch1 = t1;

    return (t0 + t1) * 0.5f;
}

/**
 * @brief  检查是否有新的温度数据到达
 * @return uint8_t 1 = 有更新，0 = 无更�?
 */
static uint8_t CalibMode_IsNewTempData(void)
{
    uint8_t cell = (uint8_t)g_calib_cell;
    uint8_t a = AppControl_GetCellOuterTempIndex(cell);
    uint8_t b = AppControl_GetCellInnerTempIndex(cell);
    uint32_t tick_a;
    uint32_t tick_b;

    if (a >= 4U || b >= 4U)
        return 0U;

    tick_a = Sys_TempUpdateTick[a];
    tick_b = Sys_TempUpdateTick[b];

    if (tick_a != s_calib_last_temp_tick_outer ||
        tick_b != s_calib_last_temp_tick_inner)
    {
        s_calib_last_temp_tick_outer = tick_a;
        s_calib_last_temp_tick_inner = tick_b;
        return 1U;
    }

    return 0U;
}

/**
 * @brief  将当前温度值推入滑动窗�?
 * @param  temp 当前温度值（平均值，用于稳定判定�?
 */
static void CalibMode_PushWindow(float temp)
{
    s_calib_temp_window[s_calib_window_idx % CALIB_WINDOW_SAMPLES] = temp;
    s_calib_window_idx++;
}

/**
 * @brief  检查窗口是否已填满
 * @return uint8_t 1 = 已满�? = 未满
 */
static uint8_t CalibMode_IsWindowFull(void)
{
    return (s_calib_window_idx >= CALIB_WINDOW_SAMPLES) ? 1U : 0U;
}

/**
 * @brief  计算滑动窗口内温度的最大差值（max - min�?
 * @return float 温差 (°C)，窗口未满时返回极大�?
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
 * @brief  计算指定通道稳定区间的平均温�?
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
 * @brief  根据 Cell 编号获取对应�?Flash 存储区偏�?
 * @param  cell Cell 编号 (0 �?1)
 * @return uint32_t Flash 存储区内偏移
 */
static uint32_t CalibMode_GetFlashOffset(uint8_t cell)
{
    return (cell == 0U) ? CALIB_FLASH_OFFSET_CELL0 : CALIB_FLASH_OFFSET_CELL1;
}

/**
 * @brief  将标定结果写�?Flash
 * @return uint8_t 0 = 成功，非0 = 失败
 */
/* One journal occupies only the first 2 KB of each existing 4 KB cell region.
 * The final doubleword commits a record; torn records are skipped, never reused. */
#define CALIB_JOURNAL_MAGIC 0x42495348UL
#define CALIB_JOURNAL_COMMIT 0x43414C31UL

typedef struct
{
    float start, end, step, cooling_ratio, heating_ratio, shared_duty, threshold;
    uint32_t stable_seconds, max_wait_seconds, window_samples;
    uint8_t count, outer_drv, inner_drv, shared_drv;
    uint8_t temp0, temp1, shared_enabled, fast_test;
} CalibJournalConfig_t;

typedef struct
{
    uint32_t magic, cell;
    CalibJournalConfig_t config;
    uint16_t crc16, reserved;
    uint32_t padding;
    uint32_t commit, commit_inverse;
} CalibJournalHeader_t;

typedef struct
{
    CalibStep_t step;
    uint32_t index;
    uint16_t crc16, reserved;
    uint32_t commit, commit_inverse;
} CalibJournalPoint_t;

#define CALIB_JOURNAL_SLOTS ((FLASH_STORAGE_PAGE_SIZE - sizeof(CalibJournalHeader_t)) / sizeof(CalibJournalPoint_t))
typedef char CalibHeaderLayoutCheck[(sizeof(CalibJournalHeader_t) == 72U &&
    offsetof(CalibJournalHeader_t, commit) == 64U) ? 1 : -1];
typedef char CalibPointLayoutCheck[(sizeof(CalibJournalPoint_t) == 32U &&
    offsetof(CalibJournalPoint_t, commit) == 24U &&
    CALIB_JOURNAL_SLOTS >= CALIB_SCAN_COUNT) ? 1 : -1];

volatile uint8_t g_calib_saved_count = 0U;
volatile uint8_t g_calib_resumed = 0U;
volatile uint32_t g_calib_flash_status = 0U;
static uint16_t s_calib_next_slot = 0U;

static void CalibMode_JournalConfig(uint8_t cell, CalibJournalConfig_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->start = CALIB_DUTY_START;
    cfg->end = CALIB_DUTY_END;
    cfg->step = CALIB_DUTY_STEP;
    cfg->cooling_ratio = (cell == 0U) ? APP_CONTROL_CELL0_COOLING_INNER_DUTY_RATIO :
                                       APP_CONTROL_CELL1_COOLING_INNER_DUTY_RATIO;
    cfg->heating_ratio = (cell == 0U) ? APP_CONTROL_CELL0_HEATING_OUTER_DUTY_RATIO :
                                       APP_CONTROL_CELL1_HEATING_OUTER_DUTY_RATIO;
    cfg->shared_duty = APP_CONTROL_SHARED_CH5_DUTY;
    cfg->threshold = CALIB_STABLE_THRESHOLD;
    cfg->stable_seconds = CALIB_STABLE_SECONDS;
    cfg->max_wait_seconds = CALIB_MAX_WAIT_SECONDS;
    cfg->window_samples = CALIB_WINDOW_SAMPLES;
    cfg->count = CALIB_SCAN_COUNT;
    cfg->outer_drv = (cell == 0U) ? APP_CONTROL_CELL0_DRV_OUTER : APP_CONTROL_CELL1_DRV_OUTER;
    cfg->inner_drv = (cell == 0U) ? APP_CONTROL_CELL0_DRV_INNER : APP_CONTROL_CELL1_DRV_INNER;
    cfg->shared_drv = APP_CONTROL_SHARED_DRV;
    cfg->temp0 = AppControl_GetCellOuterTempIndex(cell);
    cfg->temp1 = AppControl_GetCellInnerTempIndex(cell);
    cfg->shared_enabled = APP_CONTROL_SHARED_DRV_ENABLE;
#ifdef CALIB_FAST_TEST
    cfg->fast_test = 1U;
    cfg->start = CALIB_FAST_DUTY;
#endif
}

static uint8_t CalibMode_AllErased(const void *data, uint32_t size)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t i;
    for (i = 0U; i < size; i++)
        if (p[i] != 0xFFU)
            return 0U;
    return 1U;
}

/* 0: journal found, 1: absent/invalid, 2: read error, 3: settings changed. */
static uint8_t CalibMode_ReadJournal(uint8_t cell, CalibStep_t *steps,
                                    uint8_t *completed, uint16_t *next_slot)
{
    CalibJournalHeader_t header;
    CalibJournalConfig_t cfg;
    CalibJournalPoint_t point;
    uint16_t slot;
    uint32_t offset = CalibMode_GetFlashOffset(cell);

    *completed = 0U;
    *next_slot = 0U;
    if (FlashStorage_Read(offset, (uint8_t *)&header, sizeof(header)) != FLASH_STORAGE_OK)
        return 2U;
    if (header.magic != CALIB_JOURNAL_MAGIC || header.cell != cell ||
        header.commit != CALIB_JOURNAL_COMMIT ||
        header.commit_inverse != (uint32_t)~CALIB_JOURNAL_COMMIT ||
        header.crc16 != CalibMode_CRC16((const uint8_t *)&header,
                                        offsetof(CalibJournalHeader_t, crc16)))
        return 1U;
    CalibMode_JournalConfig(cell, &cfg);
    if (memcmp(&cfg, &header.config, sizeof(cfg)) != 0)
        return 3U;
    for (slot = 0U; slot < CALIB_JOURNAL_SLOTS; slot++)
    {
        if (FlashStorage_Read(offset + sizeof(header) + slot * sizeof(point),
                              (uint8_t *)&point, sizeof(point)) != FLASH_STORAGE_OK)
            return 2U;
        if (CalibMode_AllErased(&point, sizeof(point)) != 0U)
            continue;
        *next_slot = slot + 1U;
        if (*completed >= CALIB_SCAN_COUNT || point.index != *completed ||
            point.commit != CALIB_JOURNAL_COMMIT ||
            point.commit_inverse != (uint32_t)~CALIB_JOURNAL_COMMIT ||
            point.crc16 != CalibMode_CRC16((const uint8_t *)&point,
                                           offsetof(CalibJournalPoint_t, crc16)) ||
            point.step.duty != CalibMode_StepDuty(*completed))
            continue;
        if (steps != 0)
            steps[*completed] = point.step;
        (*completed)++;
    }
    return 0U;
}

static uint8_t CalibMode_NewJournal(uint8_t cell)
{
    CalibJournalHeader_t header, verify;
    FlashStorage_Status_t ret;
    uint32_t primask;
    uint32_t offset = CalibMode_GetFlashOffset(cell);
    uint32_t pos;
    uint8_t erased[32];

    memset(&header, 0, sizeof(header));
    header.magic = CALIB_JOURNAL_MAGIC;
    header.cell = cell;
    CalibMode_JournalConfig(cell, &header.config);
    header.crc16 = CalibMode_CRC16((const uint8_t *)&header,
                                  offsetof(CalibJournalHeader_t, crc16));
    header.commit = CALIB_JOURNAL_COMMIT;
    header.commit_inverse = (uint32_t)~CALIB_JOURNAL_COMMIT;
    primask = __get_PRIMASK();
    __disable_irq();
    ret = FlashStorage_ErasePage((uint8_t)(offset / FLASH_STORAGE_PAGE_SIZE));
    __set_PRIMASK(primask);
    if (ret != FLASH_STORAGE_OK)
        goto fail;

    /* Retain the existing base-page erase mapping; verify its actual result. */
    for (pos = 0U; pos < FLASH_STORAGE_PAGE_SIZE; pos += sizeof(erased))
    {
        ret = FlashStorage_Read(offset + pos, erased, sizeof(erased));
        if (ret != FLASH_STORAGE_OK)
            goto fail;
        if (CalibMode_AllErased(erased, sizeof(erased)) == 0U)
        {
            ret = FLASH_STORAGE_ERROR_ERASE;
            goto fail;
        }
    }
    primask = __get_PRIMASK();
    __disable_irq();
    ret = FlashStorage_ProgramErased(offset, (const uint8_t *)&header, sizeof(header));
    __set_PRIMASK(primask);
    if (ret != FLASH_STORAGE_OK)
        goto fail;
    ret = FlashStorage_Read(offset, (uint8_t *)&verify, sizeof(verify));
    if (ret == FLASH_STORAGE_OK && memcmp(&header, &verify, sizeof(header)) != 0)
        ret = FLASH_STORAGE_ERROR_WRITE;
fail:
    g_calib_flash_status = (uint32_t)ret;
    return (ret == FLASH_STORAGE_OK) ? 0U : CALIB_ERR_FLASH_SAVE;
}

static uint8_t CalibMode_RestoreProgress(void)
{
    uint8_t completed;
    uint16_t next_slot;
    uint8_t cell = (uint8_t)g_calib_cell;
    uint8_t ret = CalibMode_ReadJournal(cell, (CalibStep_t *)g_calib_result,
                                       &completed, &next_slot);
    char buf[96];
    int len;

    if (ret == 2U)
    {
        g_calib_flash_status = (uint32_t)g_flash_storage_last_error;
        return CALIB_ERR_FLASH_SAVE;
    }
    if (ret == 0U && completed < CALIB_SCAN_COUNT)
    {
        g_calib_step_idx = completed;
        g_calib_saved_count = completed;
        g_calib_resumed = 1U;
        s_calib_next_slot = next_slot;
        if (next_slot >= CALIB_JOURNAL_SLOTS)
            return CALIB_ERR_FLASH_FULL;
        len = snprintf(buf, sizeof(buf),
            "CALIB,RESUME,CELL:%u,SAVED:%u,NEXT_STEP:%u\r\n",
            (unsigned int)cell, (unsigned int)completed, (unsigned int)completed + 1U);
    }
    else
    {
        memset((void *)g_calib_result, 0, sizeof(g_calib_result));
        ret = CalibMode_NewJournal(cell);
        if (ret != 0U)
            return ret;
        g_calib_step_idx = 0U;
        g_calib_saved_count = 0U;
        g_calib_resumed = 0U;
        s_calib_next_slot = 0U;
        len = snprintf(buf, sizeof(buf), "CALIB,NEW,CELL:%u,NEXT_STEP:1\r\n",
                       (unsigned int)cell);
    }
    if (len > 0 && len < (int)sizeof(buf))
        CalibMode_UartSend(buf);
    return 0U;
}

static uint8_t CalibMode_SavePoint(void)
{
    CalibJournalPoint_t point, verify;
    FlashStorage_Status_t ret;
    uint32_t primask;
    uint32_t offset;
    char buf[96];
    int len;

    if (s_calib_next_slot >= CALIB_JOURNAL_SLOTS)
        return CALIB_ERR_FLASH_FULL;
    memset(&point, 0, sizeof(point));
    point.step = g_calib_result[g_calib_step_idx];
    point.index = g_calib_step_idx;
    point.crc16 = CalibMode_CRC16((const uint8_t *)&point,
                                 offsetof(CalibJournalPoint_t, crc16));
    point.commit = CALIB_JOURNAL_COMMIT;
    point.commit_inverse = (uint32_t)~CALIB_JOURNAL_COMMIT;
    offset = CalibMode_GetFlashOffset((uint8_t)g_calib_cell) +
             sizeof(CalibJournalHeader_t) + s_calib_next_slot * sizeof(point);
    primask = __get_PRIMASK();
    __disable_irq();
    ret = FlashStorage_ProgramErased(offset, (const uint8_t *)&point, sizeof(point));
    __set_PRIMASK(primask);
    s_calib_next_slot++;
    if (ret == FLASH_STORAGE_OK)
    {
        ret = FlashStorage_Read(offset, (uint8_t *)&verify, sizeof(verify));
        if (ret == FLASH_STORAGE_OK && memcmp(&point, &verify, sizeof(point)) != 0)
            ret = FLASH_STORAGE_ERROR_WRITE;
    }
    g_calib_flash_status = (uint32_t)ret;
    if (ret != FLASH_STORAGE_OK)
        return CALIB_ERR_FLASH_SAVE;
    g_calib_saved_count = (uint8_t)(g_calib_step_idx + 1U);
    len = snprintf(buf, sizeof(buf),
        "CALIB,SAVED,CELL:%u,STEP:%u,TOTAL:%u,VALID:%u\r\n",
        (unsigned int)g_calib_cell, (unsigned int)g_calib_saved_count,
        (unsigned int)CALIB_SCAN_COUNT, (unsigned int)point.step.valid);
    if (len > 0 && len < (int)sizeof(buf))
        CalibMode_UartSend(buf);
    return 0U;
}

/* USART2 输出辅助 -----------------------------------------------------------*/

/**
 * @brief  通过 USART2 发送文本字符串（阻塞模式）
 * @param  str �?null 结尾的字符串
 */
static void CalibMode_UartSend(const char *str)
{
#ifdef UART_LOG_ENABLE
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
#else
    (void)str;
#endif
}

/* 标定状态机核心 ------------------------------------------------------------*/

/**
 * @brief  初始化标定流程：配置 DRV 通道进入开环模�?
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
    s_calib_outputs_settled = 0U;
    s_calib_step_start_ms   = 0U;
    s_calib_stable_start_ms = 0U;
    s_calib_stable_reached  = 0U;
    s_calib_window_idx      = 0U;
    s_calib_stable_sum_ch0  = 0.0f;
    s_calib_stable_sum_ch1  = 0.0f;
    s_calib_stable_count    = 0U;
    s_calib_last_temp_tick_outer = 0U;
    s_calib_last_temp_tick_inner = 0U;

    memset(s_calib_temp_window, 0, sizeof(s_calib_temp_window));
}

/**
 * @brief  设置当前步的占空比（开环，�?PID�?
 */
static uint8_t CalibMode_SetStepDuty(uint32_t now_ms)
{
    uint32_t elapsed_ms = now_ms - s_calib_last_output_ms;
    DRV8703_Status_t ret;

    if (elapsed_ms < CALIB_OUTPUT_INTERVAL_MS)
        return s_calib_outputs_settled;
    s_calib_last_output_ms = now_ms;
    ret = AppControl_SetCalibDuty((uint8_t)g_calib_cell,
                                  CalibMode_StepDuty((uint8_t)g_calib_step_idx),
                                  (float)elapsed_ms * 0.001f,
                                  &s_calib_outputs_settled);
    if (ret != DRV8703_OK)
    {
        s_calib_outputs_settled = 0U;
        CalibMode_Fault(CALIB_ERR_DRV_FAULT);
    }
    return s_calib_outputs_settled;
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
        float ratio = AppControl_GetCellInnerDutyRatio((uint8_t)g_calib_cell);
        float avg_temp = (avg_temp_ch0 + avg_temp_ch1) * 0.5f;
        char buf[128];
        int len = snprintf(buf, sizeof(buf),
            "CALIB,CELL:%u,STEP:%u,DOUT:%+.2f,DIN:%+.2f,RATIO:%.2f,T0:%.1f,T1:%.1f,TAVG:%.1f,VALID:%u,SETTLED:%u\r\n",
            (unsigned int)g_calib_cell,
            (unsigned int)g_calib_step_idx + 1U,
            (double)AppControl_GetCellOuterDuty((uint8_t)g_calib_cell),
            (double)AppControl_GetCellInnerDuty((uint8_t)g_calib_cell),
            (double)ratio,
            (double)avg_temp_ch0,
            (double)avg_temp_ch1,
            (double)avg_temp,
            (unsigned int)((settled != 0U) ? 1U : 0U),
            (unsigned int)settled);
        if (len > 0 && len < (int)sizeof(buf))
        {
            CalibMode_UartSend(buf);
        }
    }

    /* Advance only after the new point has been read back successfully. */
    {
        uint8_t save_ret = CalibMode_SavePoint();
        if (save_ret != 0U)
        {
            CalibMode_Fault(save_ret);
            return;
        }
    }
    g_calib_step_idx++;

    if (g_calib_step_idx >= CALIB_SCAN_COUNT)
    {
        g_calib_state = CALIB_DONE;
    }
    else
    {
        /* 进入下一�?*/
        CalibMode_InitStep();
        g_calib_state = CALIB_RUN;
    }
}

/* 公有 API 实现 -------------------------------------------------------------*/

/* 全局静态标志，确保 DONE/FAULT/STOP 消息只发送一�?*/
static uint8_t s_calib_done_sent   = 0U;
static uint8_t s_calib_fault_sent  = 0U;
static uint8_t s_calib_stop_sent   = 0U;

/**
 * @brief  启动自动标定流程
 * @param  cell Cell 编号 (0 �?1)
 */
void CalibMode_Start(uint8_t cell)
{
    DRV8703_Status_t prepare_ret;
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

    /* 重置全局状�?*/
    prepare_ret = AppControl_PrepareCalibration(cell);
    g_calib_cell         = cell;
    g_calib_step_idx     = 0U;
    g_calib_saved_count  = 0U;
    g_calib_resumed      = 0U;
    g_calib_flash_status = 0U;
    g_calib_error        = 0U;
    g_calib_state        = CALIB_INIT;
    g_calib_mode_active  = 1U;
    g_calib_running      = 1U;
    g_calib_show_duty    = 1U;
    g_calib_temp_ch0     = 0.0f;
    g_calib_temp_ch1     = 0.0f;
    s_calib_last_log_ms  = 0U;
    /* 重置单次发送标�?*/
    s_calib_done_sent   = 0U;
    s_calib_fault_sent  = 0U;
    s_calib_stop_sent   = 0U;

    memset((void *)g_calib_result, 0, sizeof(g_calib_result));
    if (prepare_ret != DRV8703_OK)
        CalibMode_Fault(CALIB_ERR_DRV_FAULT);

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
 * @brief  标定模式主任务，�?AppControl_Task 每周期调�?
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
        /* 初始�?Flash 存储 */
        uint8_t restore_ret = CalibMode_RestoreProgress();
        if (restore_ret != 0U)
        {
            CalibMode_Fault(restore_ret);
            break;
        }
        CalibMode_InitStep();
        s_calib_last_output_ms = now_ms;
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
        /* 设定当前步占空比，进入等待稳�?*/
        if (CalibMode_SetStepDuty(now_ms) == 0U)
            break;
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

        if (CalibMode_SetStepDuty(now_ms) == 0U)
        {
            if (g_calib_state != CALIB_FAULT)
            {
                CalibMode_InitStep();
                g_calib_state = CALIB_RUN;
            }
            break;
        }

        /* 超时检�?*/
        if (elapsed >= (CALIB_MAX_WAIT_SECONDS * 1000U))
        {
            CalibMode_FinishStep(0U);
            break;
        }

        /* 等待新温度数�?*/
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
                        /* 首次判定稳定，初始化双通道累加�?*/
                        s_calib_stable_reached  = 1U;
                        s_calib_stable_start_ms = now_ms;
                        s_calib_stable_sum_ch0  = 0.0f;
                        s_calib_stable_sum_ch1  = 0.0f;
                        s_calib_stable_count    = 0U;
                    }

                    /* 累加两个通道各自的稳态数�?*/
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
                    /* 波动超出阈值，重置稳态标�?*/
                    s_calib_stable_reached = 0U;
                }
            }
        }
        break;
    }

    case CALIB_DONE:
    {
        uint8_t cell  = (uint8_t)g_calib_cell;

        (void)AppControl_SetCellStackDuty(cell, 0.0f);
#if APP_CONTROL_SHARED_DRV_ENABLE
        (void)AppControl_SetDrvDuty(APP_CONTROL_SHARED_DRV, 0.0f);
#endif

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

        (void)AppControl_SetCellStackDuty(cell, 0.0f);
#if APP_CONTROL_SHARED_DRV_ENABLE
        (void)AppControl_SetDrvDuty(APP_CONTROL_SHARED_DRV, 0.0f);
#endif

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
            {
                char detail[176];
                (void)snprintf(detail, sizeof(detail),
                    "CALIB,FAULT_DETAIL,CELL:%u,INDEX:%u,SAVED:%u,RESUMED:%u,FLASH_STATUS:%lu,NEXT_SLOT:%u,SLOTS:%u\r\n",
                    (unsigned int)cell, (unsigned int)g_calib_step_idx,
                    (unsigned int)g_calib_saved_count, (unsigned int)g_calib_resumed,
                    (unsigned long)g_calib_flash_status, (unsigned int)s_calib_next_slot,
                    (unsigned int)CALIB_JOURNAL_SLOTS);
                CalibMode_UartSend(detail);
                AppFault_LogCell("CALIB", cell, g_calib_error);
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

    (void)AppControl_SetCellStackDuty((uint8_t)g_calib_cell, 0.0f);
#if APP_CONTROL_SHARED_DRV_ENABLE
    (void)AppControl_SetDrvDuty(APP_CONTROL_SHARED_DRV, 0.0f);
#endif
    s_calib_outputs_settled = 0U;
    g_calib_error = error;
    g_calib_state = CALIB_FAULT;
    g_calib_running = 0U;
}

/**
 * @brief  紧急停止标�?
 */
void CalibMode_Stop(void)
{
    uint8_t cell  = (uint8_t)g_calib_cell;

    (void)AppControl_SetCellStackDuty(cell, 0.0f);
#if APP_CONTROL_SHARED_DRV_ENABLE
    (void)AppControl_SetDrvDuty(APP_CONTROL_SHARED_DRV, 0.0f);
#endif

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
 * @brief  �?Flash 读取指定 Cell 的上一次标定数�?
 * @param  cell Cell 编号 (0 �?1)
 * @param  buf  输出缓冲区指�?
 * @return uint8_t 0 = 成功，非0 = 失败
 */
uint8_t CalibMode_LoadFromFlash(uint8_t cell, CalibFlashData_t *buf)
{
    uint8_t completed, ret;
    uint16_t next_slot;
    uint32_t magic;
    if (cell > 1U || buf == 0)
        return 1U;
    if (FlashStorage_Read(CalibMode_GetFlashOffset(cell), (uint8_t *)&magic,
                          sizeof(magic)) != FLASH_STORAGE_OK)
        return 2U;
    if (magic == CALIB_JOURNAL_MAGIC)
    {
        memset(buf, 0, sizeof(*buf));
        ret = CalibMode_ReadJournal(cell, buf->step, &completed, &next_slot);
        if (ret != 0U)
            return (ret == 2U) ? 2U : 4U;
        /* Partial progress must not be exposed as a complete feedforward table. */
        if (completed != CALIB_SCAN_COUNT)
            return 5U;
#ifdef CALIB_FAST_TEST
        {
            uint8_t i;
            for (i = CALIB_SCAN_COUNT; i < CALIB_DUTY_COUNT; i++)
                buf->step[i] = buf->step[0];
        }
#endif
        buf->magic = CALIB_FLASH_MAGIC;
        buf->cell = cell;
        buf->crc16 = CalibMode_CRC16((const uint8_t *)buf,
                                    offsetof(CalibFlashData_t, crc16));
        return 0U;
    }

    /* Existing complete BISG tables remain readable; they have no resume index. */
    if (FlashStorage_Read(CalibMode_GetFlashOffset(cell), (uint8_t *)buf,
                          sizeof(*buf)) != FLASH_STORAGE_OK)
        return 2U;
    if (buf->magic != CALIB_FLASH_MAGIC || buf->cell != cell)
        return 3U;
    if (buf->crc16 != CalibMode_CRC16((const uint8_t *)buf,
                                     offsetof(CalibFlashData_t, crc16)))
        return 4U;
    return 0U;
}

/* Flash 读写测试 --------------------------------------------------------------*/

/**
 * @brief  Flash 写入/读出/校验测试函数
 * @param  cell Cell 编号 (0 �?1)
 * @note   构造模拟标定数�?�?擦除 �?写入 �?读出 �?逐字节校�?
 *         全程通过 USART2 输出日志
 *         写入前后关闭全局中断，防�?Flash 操作期间 ISR 取指卡死
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

    /* ---- 构造模拟标定数�?---- */
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
                                       offsetof(CalibFlashData_t, crc16));

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

    /* ---- 逐字节校�?---- */
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

/* Flash 标定数据读取与串口输�?------------------------------------------------*/

void CalibMode_ImportLegacyCell1IfMissing(void)
{
    CalibMode_UartSend("CALIB,IMPORT,CELL:1,SKIP,ASYMMETRIC_DUTY_RANGE\r\n");
}
/**
 * @brief  读取指定 Cell �?Flash 标定数据并通过 USART2 输出
 * @param  cell Cell 编号 (0 �?1)
 * @note   启动标定前调用，用于查看上一次标定结�?
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
