/**
 ******************************************************************************
 * @file    ads1220.h
 * @brief   TI ADS1220 24-bit ADC 驱动（PT1000 温度采集）
 * @author  Bisemin Team
 * @version V1.0.0
 * @date    2026-07-11
 *
 * @details
 * 基于 temperature 工程（STM32F103C8T6）的 ADS1220 代码移植。
 * SPI3 总线，支持 1~2 片 ADS1220，每片配独立的 CS 和 DRDY 引脚。
 * 连续转换模式：上电发一次 START/SYNC，后续轮询 DRDY 直接读取。
 ******************************************************************************
 */

#ifndef __ADS1220_H
#define __ADS1220_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ADS1220 通道配置 */
typedef struct
{
    GPIO_TypeDef *cs_port;   /**< 片选引脚端口 */
    uint16_t      cs_pin;    /**< 片选引脚编号 */
    GPIO_TypeDef *drdy_port; /**< 数据就绪引脚端口 */
    uint16_t      drdy_pin;  /**< 数据就绪引脚编号 */
    uint8_t       ch_id;     /**< 通道号 (1~2)，用于日志 */
} Ads1220_Cfg_t;

/* API ----------------------------------------------------------------------*/

/**
 * @brief  初始化一个 ADS1220：复位 → 写寄存器 → 启动连续转换
 * @param  cfg 通道配置指针
 */
void Ads1220_Init(const Ads1220_Cfg_t *cfg);

/**
 * @brief  初始化所有已启用的 ADS1220 通道（调用 Ads1220_InitAll）
 */
void Ads1220_InitAll(void);

/**
 * @brief  检查 DRDY 是否拉低（转换数据就绪）
 * @param  cfg 通道配置指针
 * @return true 就绪，false 未就绪
 */
bool Ads1220_IsReady(const Ads1220_Cfg_t *cfg);

/**
 * @brief  从 ADS1220 读取 24bit 数据并转换为阻值（连续模式，无需 RDATA 命令）
 * @param  cfg 通道配置指针
 * @return float PT1000 阻值 (Ω)，读取失败返回 -1.0
 * @note   调用前应先确认 Ads1220_IsReady() == true
 */
float Ads1220_FetchResistance(const Ads1220_Cfg_t *cfg);

/**
 * @brief  PT1000 阻值 → 温度（IEC 60751 Callendar-Van Dusen, Newton-Raphson 迭代）
 * @param  resistance 阻值 (Ω)
 * @return float 温度 (°C)
 */
float Ads1220_ResistanceToTemp(float resistance);

/**
 * @brief  测试函数：每秒轮询所有通道，通过 USART2 输出阻值和温度
 * @note   放在 AppControl_Task 中周期调用。验证通过后可删除。
 */
void Ads1220_Test(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADS1220_H */
