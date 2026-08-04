/**
 ******************************************************************************
 * @file    calib_mode.h
 * @brief   自动化标定模式 — 固定占空比扫描，记录稳态平均温度
 * @author  Bisemin Team
 * @version V1.0.0
 * @date    2026-07-02
 *
 * @details
 * 从 +0.45（制冷）到 -0.45（加热）每 -0.02 步进，共 46 步。
 * 每一步设定固定占空比后等待温度稳定（波动 < CALIB_STABLE_THRESHOLD
 * 持续 CALIB_STABLE_SECONDS 秒），记录稳定区间的平均温度。
 * 若超时未稳定则记录当前值并标记 valid=0。
 * 全部 46 步完成后写入 Flash。
 * Cell 0 和 Cell 1 的标定数据分别存储在 Flash 不同页，互不覆盖。
 ******************************************************************************
 */

#ifndef __CALIB_MODE_H
#define __CALIB_MODE_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "flash_storage.h"

/* 标定参数常量 --------------------------------------------------------------*/

/** @brief 占空比扫描范围：起始值（正=制冷） */
#define CALIB_DUTY_START (0.45f)

/** @brief 占空比扫描范围：结束值（负=加热） */
#define CALIB_DUTY_END (-0.45f)

/** @brief 占空比扫描步进（负步进=从制冷到加热） */
#define CALIB_DUTY_STEP (-0.02f)

/** @brief 扫描点数 = (START - END) / |STEP| + 1 */
#define CALIB_DUTY_COUNT (46U)

/** @brief 稳定判定阈值 (°C)，温度 max-min 小于此值认为稳定 */
#define CALIB_STABLE_THRESHOLD 0.1f

/** @brief 稳定需持续秒数 */
#define CALIB_STABLE_SECONDS 10U

/** @brief 单步最大等待秒数（超时强制进入下一步） */
#define CALIB_MAX_WAIT_SECONDS 600U

/** @brief 标定过程温度采样间隔 (ms)，应与 ControlTask 周期一致 */
#define CALIB_SAMPLE_INTERVAL_MS 50U

/** @brief 稳定判定所需最小采样点数 */
#define CALIB_STABLE_SAMPLES ((CALIB_STABLE_SECONDS * 1000U) / CALIB_SAMPLE_INTERVAL_MS)

/** @brief 滑动窗口大小（用于稳定判定），与稳定采样点数相同 */
#define CALIB_WINDOW_SAMPLES (CALIB_STABLE_SAMPLES)

  /*
   * 快速测试模式 — 取消下行注释即可启用
   * 仅标定 1 步（duty=0.0），其余 45 步复制填充，~10 秒完成
   * 用于验证 Flash 写入是否正常
   */
  // #define CALIB_FAST_TEST

#ifdef CALIB_FAST_TEST
/** @brief 快速测试：实际标定步数 */
#define CALIB_SCAN_COUNT 1U

/** @brief 快速测试：占空比固定为 0（不加热也不制冷） */
#define CALIB_FAST_DUTY 0.0f

/** @brief 快速测试：立即判定稳定 */
#define CALIB_FAST_THRESHOLD 999.0f
#define CALIB_FAST_STABLE_SEC 1U
#define CALIB_FAST_MAX_WAIT 10U

/* 用快速参数覆写正常参数 */
#undef CALIB_STABLE_THRESHOLD
#define CALIB_STABLE_THRESHOLD CALIB_FAST_THRESHOLD
#undef CALIB_STABLE_SECONDS
#define CALIB_STABLE_SECONDS CALIB_FAST_STABLE_SEC
#undef CALIB_MAX_WAIT_SECONDS
#define CALIB_MAX_WAIT_SECONDS CALIB_FAST_MAX_WAIT
#else
/** @brief 正常模式：实际标定步数 = 46 */
#define CALIB_SCAN_COUNT CALIB_DUTY_COUNT
#endif

  /* 标定状态机 ----------------------------------------------------------------*/

  typedef enum
  {
    CALIB_IDLE = 0,    /**< 空闲，等待启动 */
    CALIB_INIT,        /**< 初始化：准备 DRV 通道，擦除 Flash */
    CALIB_RUN,         /**< 运行中：设定当前步占空比 */
    CALIB_WAIT_STABLE, /**< 等待温度稳定 */
    CALIB_DONE,        /**< 标定完成，已写入 Flash */
    CALIB_FAULT        /**< 故障中止 */
  } CalibState_t;

  /* 单步标定记录 --------------------------------------------------------------*/

  typedef struct
  {
    float duty;      /**< 占空比 */
    float temp_ch0;  /**< 通道0 稳态平均温度 (°C) */
    float temp_ch1;  /**< 通道1 稳态平均温度 (°C) */
    uint8_t valid;   /**< 是否有效 (1 = 稳定达到, 0 = 超时/故障) */
    uint8_t settled; /**< 是否达到稳态 (1 = 是, 0 = 超时未稳定) */
    uint16_t reserved;
  } CalibStep_t;

  /* Flash 存储数据结构 ---------------------------------------------------------*/

#define CALIB_FLASH_MAGIC 0x42495345UL /**< "BISE" */

/** @brief Flash 存储区偏移：Cell 0（硬件 4KB 页 126） */
#define CALIB_FLASH_OFFSET_CELL0 0U

/** @brief Flash 存储区偏移：Cell 1（硬件 4KB 页 127，与 Cell 0 不在同一页） */
#define CALIB_FLASH_OFFSET_CELL1 4096U

  typedef struct
  {
    uint32_t magic;                     /**< 魔数，用于校验数据有效性 */
    uint8_t cell;                       /**< 标定的 Cell 编号 (0 或 1) */
    uint8_t reserved[3];                /**< 对齐填充 */
    CalibStep_t step[CALIB_DUTY_COUNT]; /**< 46 步标定记录 */
    uint16_t crc16;                     /**< 整个结构体的 CRC16 校验 */
  } CalibFlashData_t;

  /* 全局变量声明 --------------------------------------------------------------*/

  /** @brief 标定模式是否激活 */
  extern volatile uint8_t g_calib_mode_active;

  /** @brief 当前标定状态 */
  extern volatile CalibState_t g_calib_state;

  /** @brief 当前扫描步索引 (0 ~ 45) */
  extern volatile uint8_t g_calib_step_idx;

  /** @brief 46 步标定结果（运行时） */
  extern volatile CalibStep_t g_calib_result[CALIB_DUTY_COUNT];

  /** @brief 被标定的 Cell 编号 */
  extern volatile uint8_t g_calib_cell;

  /** @brief 标定错误码（0 = 无错误） */
  extern volatile uint32_t g_calib_error;

  /** @brief 标定模式下数码管显示标志：1 = 显示占空比，0 = 显示温度 */
  extern volatile uint8_t g_calib_show_duty;

  /** @brief 标定是否处于运行中（供 LED 灯判断，1 = 运行中） */
  extern volatile uint8_t g_calib_running;

  /** @brief 标定过程中通道0实时温度（供数码管显示） */
  extern volatile float g_calib_temp_ch0;

  /** @brief 标定过程中通道1实时温度（供数码管显示） */
  extern volatile float g_calib_temp_ch1;

  /* Error codes ---------------------------------------------------------------*/

#define CALIB_ERR_DRV_FAULT 10U

  /* API 函数声明 --------------------------------------------------------------*/

  /**
   * @brief  启动自动标定流程
   * @param  cell Cell 编号 (0 或 1)
   * @note   必须在 ControlTask 上下文中调用
   *         只能在一个 Cell 停止时启动
   */
  void CalibMode_Start(uint8_t cell);

  /**
   * @brief  标定模式主任务，由 AppControl_Task 每周期调用
   * @param  now_ms 当前系统 tick (ms)
   */
  void CalibMode_Task(uint32_t now_ms);

  /**
   * @brief  Abort calibration with a fault code.
   * @param  error Calibration error code.
   */
  void CalibMode_Fault(uint32_t error);

  /**
   * @brief  紧急停止标定（占空比归零，回到 IDLE）
   * @note   可在任何上下文中调用
   */
  void CalibMode_Stop(void);

  /**
   * @brief  从 Flash 读取指定 Cell 的上一次标定数据
   * @param  cell Cell 编号 (0 或 1)
   * @param  buf  输出缓冲区指针
   * @return 0 = 成功, 非0 = 失败（数据无效或读取错误）
   */
  uint8_t CalibMode_LoadFromFlash(uint8_t cell, CalibFlashData_t *buf);

  /**
   * @brief  获取当前步的占空比（供数码管显示）
   * @param  step_idx 步索引 (0 ~ 45)
   * @return float 占空比
   */
  float CalibMode_StepDuty(uint8_t step_idx);

  /**
   * @brief  获取被标定 Cell 的平均温度（供数码管显示）
   * @return float 当前平均温度 (°C)
   */
  float CalibMode_GetCellTemp(void);

  /**
   * @brief  Flash 写入/读出/校验测试函数
   * @param  cell Cell 编号 (0 或 1)
   * @note   构造模拟标定数据 → 擦除 → 写入 → 读出 → 逐字节校验
   *         全程通过 USART2 输出日志
   */
  void CalibMode_FlashTest(uint8_t cell);

  /**
   * @brief  读取指定 Cell 的 Flash 标定数据并通过 USART2 输出
   * @param  cell Cell 编号 (0 或 1)
   * @note   启动标定前调用，用于查看上一次标定结果
   */
  void CalibMode_DumpFlashData(uint8_t cell);

  /**
   * @brief  Import saved legacy calibration data for Cell 1 if Flash is empty/invalid.
   * @note   Safe to call at boot. Existing valid Cell 1 data is not overwritten.
   */
  void CalibMode_ImportLegacyCell1IfMissing(void);

#ifdef __cplusplus
}
#endif

#endif /* __CALIB_MODE_H */
