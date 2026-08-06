/**
 ******************************************************************************
 * @file    ads1220.c
 * @brief   TI ADS1220 24-bit ADC 驱动实现
 * @author  Bisemin Team
 * @version V1.0.0
 * @date    2026-07-11
 ******************************************************************************
 */

#include "ads1220.h"
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "cmsis_os.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* 外部 SPI3 句柄（CubeMX 生成） */
extern SPI_HandleTypeDef hspi3;

/* ADS1220 命令 */
#define CMD_RESET       0x06U
#define CMD_START_SYNC  0x08U
#define CMD_RDATA       0x10U

/* ============================================================
 * 通道配置表 —— 修改此处控制启用的通道数量
 * ============================================================ */
static const Ads1220_Cfg_t g_ads_cfgs[] =
{
    { SENSOR_CS1_GPIO_Port, SENSOR_CS1_Pin, INT1_GPIO_Port, INT1_Pin, 1 },
    { SENSOR_CS2_GPIO_Port, SENSOR_CS2_Pin, INT2_GPIO_Port, INT2_Pin, 2 },
};

/** @brief 实际使用的 ADS1220 数量：1 或 2 */
#define ADS1220_ACTIVE_COUNT  1U

/* ============================================================
 * 私有变量
 * ============================================================ */
static uint16_t g_last_cs_pin = 0U;  /* 上次选中的 CS，避免重复操作 */

/* ============================================================
 * 底层 SPI 与安全互斥片选
 * ============================================================ */

/**
 * @brief  安全片选：先关断上一通道，再打开目标通道
 */
static void Safe_CS_Select(const Ads1220_Cfg_t *cfg)
{
    uint16_t target = cfg->cs_pin;

    /* 仅关断上一次选中的通道（如果与本次不同） */
    if (g_last_cs_pin != 0U && g_last_cs_pin != target)
    {
        /* 需要找到上次通道对应的端口——简化处理：全部拉高 */
        uint8_t i;
        for (i = 0U; i < ADS1220_ACTIVE_COUNT; i++)
        {
            HAL_GPIO_WritePin(g_ads_cfgs[i].cs_port, g_ads_cfgs[i].cs_pin, GPIO_PIN_SET);
        }
    }

    HAL_GPIO_WritePin(cfg->cs_port, target, GPIO_PIN_RESET);
    g_last_cs_pin = target;

    /* 满足 ADS1220 时序要求：CS 下降沿到 SCLK 上升沿 ≥ 50ns */
    __NOP();
    __NOP();
}

/**
 * @brief  安全片选关断
 */
static void Safe_CS_Deselect(const Ads1220_Cfg_t *cfg)
{
    HAL_GPIO_WritePin(cfg->cs_port, cfg->cs_pin, GPIO_PIN_SET);
}

/**
 * @brief  SPI 单字节收发
 */
static uint8_t ADS1220_SPI_Transfer(uint8_t data)
{
    uint8_t rx = 0U;
    (void)HAL_SPI_TransmitReceive(&hspi3, &data, &rx, 1U, 100U);
    return rx;
}

/**
 * @brief  写 ADS1220 寄存器
 */
static void ADS1220_WriteReg(const Ads1220_Cfg_t *cfg,
                             uint8_t reg_addr, uint8_t data)
{
    Safe_CS_Select(cfg);
    (void)ADS1220_SPI_Transfer(0x40U | (reg_addr << 2U));
    (void)ADS1220_SPI_Transfer(data);
    Safe_CS_Deselect(cfg);
}

/* ============================================================
 * 公开 API
 * ============================================================ */

/**
 * @brief  初始化一个 ADS1220 通道
 */
void Ads1220_Init(const Ads1220_Cfg_t *cfg)
{
    if (cfg == 0)
        return;

    /* 确保 SPI3 为 8-bit 模式（CubeMX 可能配成 4-bit） */
    if (hspi3.Init.DataSize != SPI_DATASIZE_8BIT)
    {
        hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
        (void)HAL_SPI_Init(&hspi3);
    }

    /* 拉高 CS，避免上电瞬间误触发 */
    HAL_GPIO_WritePin(cfg->cs_port, cfg->cs_pin, GPIO_PIN_SET);

    /* 1. 软件复位 */
    Safe_CS_Select(cfg);
    (void)ADS1220_SPI_Transfer(CMD_RESET);
    Safe_CS_Deselect(cfg);
    osDelay(1U);  /* RESET 后需 ≥ 50μs + 32×t_CLK */

    /*
     * 2. 写配置寄存器（与 temperature 工程完全一致）
     *    Reg0=0x32: MUX=AIN1/AIN2, PGA=2x, PGA enabled
     *    Reg1=0x04: 20SPS, normal mode, continuous conversion
     *    Reg2=0x54: external VREF, 50/60Hz rejection, IDAC=250μA
     *    Reg3=0x20: IDAC1→AIN0, IDAC2 disabled
     */
    ADS1220_WriteReg(cfg, 0x00U, 0x32U);
    ADS1220_WriteReg(cfg, 0x01U, 0x04U);
    ADS1220_WriteReg(cfg, 0x02U, 0x54U);
    ADS1220_WriteReg(cfg, 0x03U, 0x20U);

    osDelay(1U);

    /* 3. 发一次 START/SYNC，进入连续转换模式 */
    Safe_CS_Select(cfg);
    (void)ADS1220_SPI_Transfer(CMD_START_SYNC);
    Safe_CS_Deselect(cfg);
}

/**
 * @brief  初始化所有已启用的 ADS1220 通道
 */
void Ads1220_InitAll(void)
{
    uint8_t i;

    /* 上电后先拉高所有 CS */
    for (i = 0U; i < ADS1220_ACTIVE_COUNT; i++)
    {
        HAL_GPIO_WritePin(g_ads_cfgs[i].cs_port,
                          g_ads_cfgs[i].cs_pin, GPIO_PIN_SET);
    }
    osDelay(100U);

    /* 依次初始化并启动连续转换 */
    for (i = 0U; i < ADS1220_ACTIVE_COUNT; i++)
    {
        Ads1220_Init(&g_ads_cfgs[i]);
        osDelay(10U);
    }
}

/**
 * @brief  检查 DRDY 是否就绪
 */
bool Ads1220_IsReady(const Ads1220_Cfg_t *cfg)
{
    if (cfg == 0)
        return false;
    return (HAL_GPIO_ReadPin(cfg->drdy_port, cfg->drdy_pin) == GPIO_PIN_RESET);
}

/**
 * @brief  读取 24bit 数据并转换为阻值（连续模式）
 */
float Ads1220_FetchResistance(const Ads1220_Cfg_t *cfg)
{
    uint32_t raw;
    int32_t  signed_val;

    if (cfg == 0)
        return -1.0f;

    /* 连续模式下 DRDY=0 时直接读 3 字节（无需 RDATA 命令） */
    Safe_CS_Select(cfg);
    raw  = (uint32_t)ADS1220_SPI_Transfer(0xFFU) << 16U;
    raw |= (uint32_t)ADS1220_SPI_Transfer(0xFFU) << 8U;
    raw |= (uint32_t)ADS1220_SPI_Transfer(0xFFU);
    Safe_CS_Deselect(cfg);

    /* 24-bit 补码 → 32-bit 有符号数 */
    if ((raw & 0x800000UL) != 0U)
        signed_val = (int32_t)(raw | 0xFF000000UL);
    else
        signed_val = (int32_t)raw;

    /*
     * 阻值换算：
     *   VREF = 外部 4kΩ 参考电阻 × IDAC 250μA = 1V
     *   公式：R = signedData × VREF / (PGA × 2^23)
     *       = signedData × 4000 / (2 × 8388608)
     */
    return ((float)signed_val * 4000.0f) / (2.0f * 8388608.0f);
}

/**
 * @brief  PT1000 阻值 → 温度（IEC 60751 Callendar-Van Dusen）
 */
float Ads1220_ResistanceToTemp(float resistance)
{
    const float R0 = 1000.0f;
    const float A  =  3.9083e-3f;
    const float B  = -5.775e-7f;
    const float C  = -4.183e-12f;
    int   iter;
    float t;

    /* 初始估计：线性近似 */
    t = (resistance - R0) / (R0 * A);
    if (t > 850.0f)  t = 850.0f;
    if (t < -200.0f) t = -200.0f;

    /* Newton-Raphson 迭代（最多 5 次，收敛阈值 0.0001°C） */
    for (iter = 0; iter < 5; iter++)
    {
        float rt, deriv, delta;

        if (t >= 0.0f)
        {
            rt    = R0 * (1.0f + A * t + B * t * t);
            deriv = R0 * (A + 2.0f * B * t);
        }
        else
        {
            float t2 = t * t;
            float t3 = t2 * t;
            rt    = R0 * (1.0f + A * t + B * t2
                               + C * (t - 100.0f) * t3);
            deriv = R0 * (A + 2.0f * B * t
                          + C * (4.0f * t3 - 300.0f * t2));
        }

        delta = (resistance - rt) / deriv;
        t += delta;
        if (fabsf(delta) < 0.0001f)
            break;
    }

    return t;
}

/* ============================================================
 * 测试函数
 * ============================================================ */

/**
 * @brief  每秒轮询所有 ADS1220 通道，USART2 输出阻值和温度
 */
void Ads1220_Test(void)
{
    static uint32_t s_last_ms = 0U;
    uint32_t now;
    uint8_t  i;

    now = osKernelGetTickCount();
    if ((now - s_last_ms) < 1000U)
        return;
    s_last_ms = now;

    for (i = 0U; i < ADS1220_ACTIVE_COUNT; i++)
    {
        uint8_t ch = g_ads_cfgs[i].ch_id;

        if (!Ads1220_IsReady(&g_ads_cfgs[i]))
        {
#ifdef UART_LOG_ENABLE
            {
                char buf[32];
                int  len = snprintf(buf, sizeof(buf),
                                    "ADS1220,CH%u,NOT_READY\r\n",
                                    (unsigned int)ch);
                if (len > 0 && len < (int)sizeof(buf))
                    (void)HAL_UART_Transmit(&huart2, (uint8_t *)buf,
                                            (uint16_t)len, 20U);
            }
#endif
            continue;
        }

        {
            float r = Ads1220_FetchResistance(&g_ads_cfgs[i]);
            float t = Ads1220_ResistanceToTemp(r);

#ifdef UART_LOG_ENABLE
            {
                char buf[48];
                int  len = snprintf(buf, sizeof(buf),
                                    "ADS1220,CH%u,R:%.1f,T:%.2f\r\n",
                                    (unsigned int)ch, (double)r, (double)t);
                if (len > 0 && len < (int)sizeof(buf))
                    (void)HAL_UART_Transmit(&huart2, (uint8_t *)buf,
                                            (uint16_t)len, 20U);
            }
#else
            (void)r;
            (void)t;
#endif
        }
    }
}
