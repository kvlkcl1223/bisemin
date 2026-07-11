/**
 ******************************************************************************
 * @file    flash_storage.c
 * @brief   STM32G4 内部 Flash 参数存储库（实现）
 * @author  Bisemin Team
 * @version V1.0.0
 * @date    2026-07-01
 *
 * @details
 * 基于 HAL_FLASH 驱动实现，关键硬件约束：
 *   - STM32G474 Flash 512KB，单页 2KB，共 256 页
 *   - 最小编程单位：双字（64 位 / 8 字节）
 *   - 擦除操作以页为单位，擦除后全部字节 = 0xFF
 *   - 写入前必须先擦除（只能将 1 变 0，不能将 0 变 1）
 *   - 擦除/写入期间 CPU 暂停取指（HAL_FLASH 内部轮询等待）
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "flash_storage.h"
#include <string.h>

/* 全局变量定义 -------------------------------------------------------------*/

/** @brief 最后一次操作的错误码（调试用） */
volatile FlashStorage_Status_t g_flash_storage_last_error = FLASH_STORAGE_OK;

/** @brief 存储区累计擦除次数 */
volatile uint32_t g_flash_storage_erase_count = 0U;

/** @brief 存储区累计写入次数 */
volatile uint32_t g_flash_storage_write_count = 0U;

/* 私有函数声明 -------------------------------------------------------------*/

/**
 * @brief  将 Flash 页索引转换为绝对地址
 * @param  page_idx 页索引（相对于存储区起始）
 * @return uint32_t 绝对 Flash 地址
 */
static uint32_t FlashStorage_PageAddr(uint8_t page_idx);

/**
 * @brief  获取指定偏移地址所在的页索引
 * @param  offset 存储区内偏移（字节）
 * @return uint8_t 页索引
 */
static uint8_t FlashStorage_OffsetToPage(uint32_t offset);

/**
 * @brief  检查目标区域是否全部为 0xFF（已擦除状态）
 * @param  addr  绝对起始地址
 * @param  len   检查长度（字节）
 * @return uint8_t 1 = 已擦除，0 = 未擦除
 */
static uint8_t FlashStorage_IsErased(uint32_t addr, uint32_t len);

/**
 * @brief  执行 Flash 页擦除（内部实现）
 * @param  absolute_addr 绝对页起始地址
 * @return FlashStorage_Status_t
 */
static FlashStorage_Status_t FlashStorage_DoErasePage(uint32_t absolute_addr);

/**
 * @brief  执行双字（64 位）Flash 写入
 * @param  addr   绝对 Flash 地址（必须 8 字节对齐）
 * @param  data64 64 位数据
 * @return FlashStorage_Status_t
 */
static FlashStorage_Status_t FlashStorage_ProgramDoubleWord(uint32_t addr, uint64_t data64);

/**
 * @brief  锁定 Flash 控制寄存器并上报错误
 * @param  status 本次操作的结果
 * @return FlashStorage_Status_t 最终状态
 */
static FlashStorage_Status_t FlashStorage_LockAndReturn(FlashStorage_Status_t status);

/* 私有函数实现 -------------------------------------------------------------*/

/**
 * @brief  将 Flash 页索引转换为绝对地址
 * @param  page_idx 页索引（0 ~ FLASH_STORAGE_PAGE_COUNT - 1）
 * @return uint32_t 绝对 Flash 地址
 */
static uint32_t FlashStorage_PageAddr(uint8_t page_idx)
{
    return (uint32_t)(FLASH_STORAGE_START_ADDR + (uint32_t)page_idx * FLASH_STORAGE_PAGE_SIZE);
}

/**
 * @brief  获取指定偏移地址所在的页索引
 * @param  offset 存储区内偏移（字节）
 * @return uint8_t 页索引
 */
static uint8_t FlashStorage_OffsetToPage(uint32_t offset)
{
    return (uint8_t)(offset / FLASH_STORAGE_PAGE_SIZE);
}

/**
 * @brief  检查目标区域是否全部为 0xFF（已擦除状态）
 * @param  addr  绝对起始地址
 * @param  len   检查长度（字节）
 * @return uint8_t 1 = 已擦除，0 = 未擦除
 * @note   逐字节扫描，对于大区域耗时较长
 */
static uint8_t FlashStorage_IsErased(uint32_t addr, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)addr;
    uint32_t i;

    for (i = 0U; i < len; i++)
    {
        if (p[i] != 0xFFU)
        {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief  执行 Flash 页擦除（内部实现）
 * @param  absolute_addr 绝对页起始地址（必须页对齐）
 * @return FlashStorage_Status_t
 * @note   调用 HAL_FLASHEx_Erase，内部轮询等待完成
 */
static FlashStorage_Status_t FlashStorage_DoErasePage(uint32_t absolute_addr)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error;
    HAL_StatusTypeDef hal_ret;
    uint32_t page_num;
    uint32_t banks;

    /*
     * STM32G4 HAL 的 FLASH_EraseInitTypeDef.Page 必须是 Bank 内页号，
     * 不能传绝对地址。需根据单/双 Bank 模式分别计算。
     *
     * 注意：不能用 HAL 的 FLASH_BANK_SIZE（依赖 FLASH_SIZE 寄存器，
     * 在 STM32G474 上该寄存器返回错误值）。改用存储区位置反推 Bank 边界：
     *   存储区位于 Flash 末尾 → Flash 总大小可推算 → Bank 分界 = 总大小 / 2
     */
#if defined(FLASH_OPTR_DBANK)
    if (READ_BIT(FLASH->OPTR, FLASH_OPTR_DBANK) != 0U)
    {
        uint32_t flash_total = (FLASH_STORAGE_START_ADDR + FLASH_STORAGE_SIZE)
                               - FLASH_BASE;
        uint32_t bank_boundary = FLASH_BASE + (flash_total / 2U);

        if (absolute_addr >= bank_boundary)
        {
            /* Bank 2 */
            page_num = (absolute_addr - bank_boundary)
                       / FLASH_STORAGE_PAGE_SIZE;
            banks    = FLASH_BANK_2;
        }
        else
        {
            /* Bank 1 */
            page_num = (absolute_addr - FLASH_BASE)
                       / FLASH_STORAGE_PAGE_SIZE;
            banks    = FLASH_BANK_1;
        }
    }
    else
#endif
    {
        /* 单 Bank 模式 */
        page_num = (absolute_addr - FLASH_BASE) / FLASH_STORAGE_PAGE_SIZE;
        banks    = FLASH_BANK_1;
    }

    memset(&erase_init, 0, sizeof(erase_init));
    erase_init.TypeErase    = FLASH_TYPEERASE_PAGES;
    erase_init.Page         = page_num;
    erase_init.NbPages      = 1U;
    erase_init.Banks        = banks;

    /*
     * HAL_FLASHEx_Erase 会阻塞直到擦除完成或出错
     * 在此期间 CPU 无法从 Flash 取指——确保调用者已将关键代码放在 SRAM 中
     */
    hal_ret = HAL_FLASHEx_Erase(&erase_init, &page_error);
    if (hal_ret != HAL_OK)
    {
        return FLASH_STORAGE_ERROR_ERASE;
    }

    g_flash_storage_erase_count++;

    return FLASH_STORAGE_OK;
}

/**
 * @brief  执行双字（64 位）Flash 写入
 * @param  addr   绝对 Flash 地址（必须 8 字节对齐）
 * @param  data64 64 位数据
 * @return FlashStorage_Status_t
 * @note   使用 HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, ...)
 */
static FlashStorage_Status_t FlashStorage_ProgramDoubleWord(uint32_t addr, uint64_t data64)
{
    HAL_StatusTypeDef hal_ret;

    /*
     * STM32G4 的 HAL_FLASH_Program 在双字模式下需要传递两个 32 位字
     * 分别写入 addr 和 addr + 4
     */
    hal_ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                addr,
                                data64);
    if (hal_ret != HAL_OK)
    {
        return FLASH_STORAGE_ERROR_WRITE;
    }

    return FLASH_STORAGE_OK;
}

/**
 * @brief  锁定 Flash 控制寄存器并上报错误
 * @param  status 本次操作的结果
 * @return FlashStorage_Status_t 最终状态
 */
static FlashStorage_Status_t FlashStorage_LockAndReturn(FlashStorage_Status_t status)
{
    HAL_StatusTypeDef hal_ret;

    g_flash_storage_last_error = status;

    hal_ret = HAL_FLASH_Lock();
    if (hal_ret != HAL_OK && status == FLASH_STORAGE_OK)
    {
        g_flash_storage_last_error = FLASH_STORAGE_ERROR_LOCK;
        return FLASH_STORAGE_ERROR_LOCK;
    }

    return status;
}

/* 公有 API 实现 ------------------------------------------------------------*/

/**
 * @brief  初始化 Flash 存储模块（解锁 Flash 控制寄存器）
 * @return FlashStorage_Status_t 操作状态
 * @note   必须在任何擦除/写入操作之前调用
 *         解锁失败可能因为 Flash 控制器正在执行其他操作
 */
FlashStorage_Status_t FlashStorage_Init(void)
{
    HAL_StatusTypeDef hal_ret;

    /* 检查 Flash 控制器是否空闲 */
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != 0U)
    {
        g_flash_storage_last_error = FLASH_STORAGE_ERROR_BUSY;
        return FLASH_STORAGE_ERROR_BUSY;
    }

    hal_ret = HAL_FLASH_Unlock();
    if (hal_ret != HAL_OK)
    {
        g_flash_storage_last_error = FLASH_STORAGE_ERROR_UNLOCK;
        return FLASH_STORAGE_ERROR_UNLOCK;
    }

    g_flash_storage_last_error = FLASH_STORAGE_OK;
    return FLASH_STORAGE_OK;
}

/**
 * @brief  擦除存储区全部页面
 * @return FlashStorage_Status_t 操作状态
 * @note   擦除后所有字节变为 0xFF
 *         操作前自动调用 FlashStorage_Init() 解锁
 */
FlashStorage_Status_t FlashStorage_EraseAll(void)
{
    FlashStorage_Status_t ret;
    uint8_t page_idx;

    /* 遍历擦除存储区中每一页 */
    for (page_idx = 0U; page_idx < FLASH_STORAGE_PAGE_COUNT; page_idx++)
    {
        ret = FlashStorage_ErasePage(page_idx);
        if (ret != FLASH_STORAGE_OK)
        {
            return ret;
        }
    }

    return FLASH_STORAGE_OK;
}

/**
 * @brief  擦除存储区中的单个页面
 * @param  page_idx 页索引（0 ~ FLASH_STORAGE_PAGE_COUNT - 1）
 * @return FlashStorage_Status_t 操作状态
 * @note   擦除前自动解锁 Flash，擦除后上锁
 */
FlashStorage_Status_t FlashStorage_ErasePage(uint8_t page_idx)
{
    FlashStorage_Status_t ret;
    uint32_t page_addr;

    /* 参数校验 */
    if (page_idx >= FLASH_STORAGE_PAGE_COUNT)
    {
        g_flash_storage_last_error = FLASH_STORAGE_ERROR_PARAM;
        return FLASH_STORAGE_ERROR_PARAM;
    }

    page_addr = FlashStorage_PageAddr(page_idx);

    /* 解锁 Flash */
    ret = FlashStorage_Init();
    if (ret != FLASH_STORAGE_OK)
    {
        return ret;
    }

    /* 执行擦除 */
    ret = FlashStorage_DoErasePage(page_addr);

    /* 上锁并返回 */
    return FlashStorage_LockAndReturn(ret);
}

/**
 * @brief  向 Flash 存储区写入数据
 * @param  offset 存储区内偏移地址（字节，0 ~ FLASH_STORAGE_SIZE - 1）
 * @param  data   源数据指针
 * @param  len    写入长度（字节）
 * @return FlashStorage_Status_t 操作状态
 * @note   写入策略：
 *         - 按 8 字节（双字）对齐写入
 *         - 写入前检查目标区域是否已擦除；未擦除则自动擦除所在页
 *         - 如果对齐后超出原数据范围，超出的字节填充为 0xFF
 *         - 跨页写入时，每页独立判断是否需要擦除
 */
FlashStorage_Status_t FlashStorage_Write(uint32_t offset, const uint8_t *data, uint32_t len)
{
    FlashStorage_Status_t ret;
    uint32_t aligned_start;             /* 对齐后的起始偏移 */
    uint32_t aligned_end;               /* 对齐后的结束偏移（不含） */
    uint8_t  write_buf[FLASH_STORAGE_PROGRAM_UNIT];  /* 双字对齐写入缓冲区 */
    uint32_t pos;                        /* 当前写入偏移 */
    uint8_t  last_page;                 /* 上次擦除的页索引（避免重复擦除） */

    /* 参数校验 */
    if (data == 0 || len == 0U)
    {
        g_flash_storage_last_error = FLASH_STORAGE_ERROR_PARAM;
        return FLASH_STORAGE_ERROR_PARAM;
    }

    if (offset >= FLASH_STORAGE_SIZE)
    {
        g_flash_storage_last_error = FLASH_STORAGE_ERROR_PARAM;
        return FLASH_STORAGE_ERROR_PARAM;
    }

    if (len > (FLASH_STORAGE_SIZE - offset))
    {
        /* 写入长度超出存储区末尾，截断到末尾 */
        len = FLASH_STORAGE_SIZE - offset;
    }

    /* 计算对齐边界 */
    aligned_start = offset & ~(FLASH_STORAGE_PROGRAM_UNIT - 1U);           /* 向下对齐到 8 字节 */
    aligned_end   = (offset + len + FLASH_STORAGE_PROGRAM_UNIT - 1U)
                    & ~(FLASH_STORAGE_PROGRAM_UNIT - 1U);                  /* 向上对齐到 8 字节 */

    /* 解锁 Flash */
    ret = FlashStorage_Init();
    if (ret != FLASH_STORAGE_OK)
    {
        return ret;
    }

    /*
     * 遍历写入每一个对齐的双字
     * 策略：
     *   1. 如果当前双字所在页未擦除，先擦除该页（同一页只擦一次）
     *   2. 构造双字内容：对齐范围内的原始 Flash 数据 + 本次要写的数据
     *   3. 对齐超出原数据范围的部分保持 0xFF
     */
    last_page = 0xFFU;  /* 初始化为无效值 */

    for (pos = aligned_start; pos < aligned_end; pos += FLASH_STORAGE_PROGRAM_UNIT)
    {
        uint8_t  current_page;
        uint32_t flash_addr;
        uint32_t i;
        uint32_t buf_offset;
        uint32_t src_offset;

        current_page = FlashStorage_OffsetToPage(pos);

        /* 检查当前页是否需要擦除 */
        if (current_page != last_page)
        {
            uint32_t page_start = FlashStorage_PageAddr(current_page);

            if (FlashStorage_IsErased(page_start, FLASH_STORAGE_PAGE_SIZE) == 0U)
            {
                ret = FlashStorage_DoErasePage(page_start);
                if (ret != FLASH_STORAGE_OK)
                {
                    return FlashStorage_LockAndReturn(ret);
                }
            }
            last_page = current_page;
        }

        /* 构造双字写入内容 */
        flash_addr = FlashStorage_PageAddr(current_page)
                     + (pos - (uint32_t)current_page * FLASH_STORAGE_PAGE_SIZE);

        for (i = 0U; i < FLASH_STORAGE_PROGRAM_UNIT; i++)
        {
            buf_offset = pos + i;  /* 存储区内的绝对偏移 */

            if (buf_offset < offset)
            {
                /*
                 * 对齐导致的前缀填充区：读取 Flash 原有数据
                 * 因为该位置不在用户数据的范围内，需要保留原有内容
                 */
                write_buf[i] = *((uint8_t *)(FLASH_STORAGE_START_ADDR + buf_offset));
            }
            else if (buf_offset < (offset + len))
            {
                /* 用户数据区域 */
                src_offset = buf_offset - offset;
                write_buf[i] = data[src_offset];
            }
            else
            {
                /* 对齐导致的后缀填充区：保留 Flash 原有数据 */
                if ((FLASH_STORAGE_START_ADDR + buf_offset) < FLASH_STORAGE_END_ADDR)
                {
                    write_buf[i] = *((uint8_t *)(FLASH_STORAGE_START_ADDR + buf_offset));
                }
                else
                {
                    write_buf[i] = 0xFFU;
                }
            }
        }

        /*
         * 执行双字写入
         * 注意：HAL_FLASH_Program 第二个参数为 64 位数据
         */
        {
            uint64_t dword_val;

            dword_val =  ((uint64_t)write_buf[0])
                      |  ((uint64_t)write_buf[1] << 8U)
                      |  ((uint64_t)write_buf[2] << 16U)
                      |  ((uint64_t)write_buf[3] << 24U)
                      |  ((uint64_t)write_buf[4] << 32U)
                      |  ((uint64_t)write_buf[5] << 40U)
                      |  ((uint64_t)write_buf[6] << 48U)
                      |  ((uint64_t)write_buf[7] << 56U);

            ret = FlashStorage_ProgramDoubleWord(flash_addr, dword_val);
            if (ret != FLASH_STORAGE_OK)
            {
                return FlashStorage_LockAndReturn(ret);
            }
        }
    }

    g_flash_storage_write_count++;

    /* 上锁并返回成功 */
    return FlashStorage_LockAndReturn(FLASH_STORAGE_OK);
}

/**
 * @brief  从 Flash 存储区读取数据
 * @param  offset 存储区内偏移地址（字节，0 ~ FLASH_STORAGE_SIZE - 1）
 * @param  buf    目标缓冲区指针
 * @param  len    读取长度（字节）
 * @return FlashStorage_Status_t 操作状态
 * @note   直接通过指针解引用读取，不需要初始化
 *         读取操作不会触发 Flash 控制器操作，可在 ISR 中安全调用
 */
FlashStorage_Status_t FlashStorage_Read(uint32_t offset, uint8_t *buf, uint32_t len)
{
    uint32_t flash_addr;

    /* 参数校验 */
    if (buf == 0 || len == 0U)
    {
        g_flash_storage_last_error = FLASH_STORAGE_ERROR_PARAM;
        return FLASH_STORAGE_ERROR_PARAM;
    }

    if (offset >= FLASH_STORAGE_SIZE)
    {
        g_flash_storage_last_error = FLASH_STORAGE_ERROR_PARAM;
        return FLASH_STORAGE_ERROR_PARAM;
    }

    if (len > (FLASH_STORAGE_SIZE - offset))
    {
        len = FLASH_STORAGE_SIZE - offset;
    }

    flash_addr = (uint32_t)(FLASH_STORAGE_START_ADDR + offset);

    /*
     * G4 的 Flash 可直接通过地址指针读取，与 SRAM 无异
     * 使用 memcpy 可处理未对齐地址和任意长度
     */
    memcpy(buf, (const void *)flash_addr, len);

    g_flash_storage_last_error = FLASH_STORAGE_OK;
    return FLASH_STORAGE_OK;
}