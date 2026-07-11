/**
 ******************************************************************************
 * @file    flash_storage.h
 * @brief   STM32G4 内部 Flash 参数存储库
 * @author  Bisemin Team
 * @version V1.0.0
 * @date    2026-07-01
 *
 * @details
 * 提供基于 STM32G474 内部 Flash 的非易失性数据存储功能。
 * 存储区位于 Flash 末尾 8KB（0x0807E000 ~ 0x0807FFFF），共 4 个 2KB 页。
 *
 * 主要特性：
 *   - 双字（64 位）对齐写入，匹配 G4 硬件最小编程单位
 *   - 先擦后写：写入前自动判断是否需要擦除目标页
 *   - 直接指针读取，无需缓冲区
 *   - 支持全擦除和单页擦除
 *
 * @note
 *   - 写入操作会阻塞 CPU（HAL_FLASH 内部轮询），避免在 ISR 中调用
 *   - 擦除/写入期间所有 Flash 访问被暂停，包括取指和执行
 *   - 建议在擦除前禁用全局中断以防时序问题
 ******************************************************************************
 */

#ifndef __FLASH_STORAGE_H
#define __FLASH_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include <stdint.h>

/* 存储区地址与大小配置（可修改）----------------------------------------------*/

/** @brief Flash 存储区起始地址（256KB Flash 末尾 8KB） */
#define FLASH_STORAGE_START_ADDR    (0x0803E000ULL)

/** @brief Flash 存储区总容量（字节） */
#define FLASH_STORAGE_SIZE          (8192U)

/** @brief STM32G4 单页大小（字节） */
#define FLASH_STORAGE_PAGE_SIZE     (2048U)

/** @brief 存储区占用的页数 */
#define FLASH_STORAGE_PAGE_COUNT    (FLASH_STORAGE_SIZE / FLASH_STORAGE_PAGE_SIZE)

/** @brief Flash 硬件最小编程单位（双字 = 64 位） */
#define FLASH_STORAGE_PROGRAM_UNIT  (8U)

/** @brief Flash 存储区结束地址 */
#define FLASH_STORAGE_END_ADDR      (FLASH_STORAGE_START_ADDR + FLASH_STORAGE_SIZE - 1U)

/* 状态枚举 ----------------------------------------------------------------*/

/**
 * @brief Flash 存储操作状态
 */
typedef enum
{
    FLASH_STORAGE_OK            = 0,    /**< 操作成功 */
    FLASH_STORAGE_ERROR_PARAM   = 1,    /**< 参数错误（地址越界、空指针等） */
    FLASH_STORAGE_ERROR_UNLOCK  = 2,    /**< Flash 解锁失败 */
    FLASH_STORAGE_ERROR_ERASE   = 3,    /**< 页擦除失败 */
    FLASH_STORAGE_ERROR_WRITE   = 4,    /**< 写入失败 */
    FLASH_STORAGE_ERROR_BUSY    = 5,    /**< Flash 控制器繁忙 */
    FLASH_STORAGE_ERROR_LOCK    = 6     /**< Flash 上锁失败 */
} FlashStorage_Status_t;

/* 全局变量声明（调试用）----------------------------------------------------*/

/** @brief 最后一次操作的错误码 */
extern volatile FlashStorage_Status_t g_flash_storage_last_error;

/** @brief 存储区累计擦除次数（全擦除或单页擦除均计入） */
extern volatile uint32_t g_flash_storage_erase_count;

/** @brief 存储区累计写入次数（每次 Write 调用计 1 次） */
extern volatile uint32_t g_flash_storage_write_count;

/* API 函数声明 ------------------------------------------------------------*/

/**
 * @brief  初始化 Flash 存储模块（解锁 Flash 控制寄存器）
 * @return FlashStorage_Status_t 操作状态
 * @note   必须在任何擦除/写入操作之前调用
 */
FlashStorage_Status_t FlashStorage_Init(void);

/**
 * @brief  擦除存储区全部页面
 * @return FlashStorage_Status_t 操作状态
 * @note   擦除后所有字节变为 0xFF
 */
FlashStorage_Status_t FlashStorage_EraseAll(void);

/**
 * @brief  擦除存储区中的单个页面
 * @param  page_idx 页索引（0 ~ FLASH_STORAGE_PAGE_COUNT - 1）
 * @return FlashStorage_Status_t 操作状态
 */
FlashStorage_Status_t FlashStorage_ErasePage(uint8_t page_idx);

/**
 * @brief  向 Flash 存储区写入数据
 * @param  offset 存储区内偏移地址（字节，0 ~ FLASH_STORAGE_SIZE - 1）
 * @param  data   源数据指针
 * @param  len    写入长度（字节）
 * @return FlashStorage_Status_t 操作状态
 * @note   写入地址按 8 字节对齐，未对齐部分会被填充为 0xFF
 *         如果目标页未擦除（非 0xFF），将自动先擦除再写入
 */
FlashStorage_Status_t FlashStorage_Write(uint32_t offset, const uint8_t *data, uint32_t len);

/**
 * @brief  从 Flash 存储区读取数据
 * @param  offset 存储区内偏移地址（字节，0 ~ FLASH_STORAGE_SIZE - 1）
 * @param  buf    目标缓冲区指针
 * @param  len    读取长度（字节）
 * @return FlashStorage_Status_t 操作状态
 * @note   直接通过指针解引用读取，速度等同于 SRAM
 *         不需要先调用 FlashStorage_Init()
 */
FlashStorage_Status_t FlashStorage_Read(uint32_t offset, uint8_t *buf, uint32_t len);

/**
 * @brief  获取存储区起始地址
 * @return uint32_t 起始地址
 */
static inline uint32_t FlashStorage_GetStartAddr(void)
{
    return (uint32_t)FLASH_STORAGE_START_ADDR;
}

/**
 * @brief  获取存储区总容量
 * @return uint32_t 容量（字节）
 */
static inline uint32_t FlashStorage_GetSize(void)
{
    return (uint32_t)FLASH_STORAGE_SIZE;
}

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_STORAGE_H */