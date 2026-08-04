/**
 ******************************************************************************
 * @file    pc_protocol.h
 * @brief   PC 上位机通信协议（二进制帧 + ASCII 载荷）
 * @author  Bisemin Team
 * @version V1.0.0
 * @date    2026-07-12
 *
 * @details
 * USART2 双向通信，1M bps。
 * 接收：DMA + 空闲中断 → ISR 内解析帧 → 入命令队列
 * 发送：组装二进制帧 → 阻塞发送
 ******************************************************************************
 */

#ifndef __PC_PROTOCOL_H
#define __PC_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 帧类型 */
typedef enum
{
    PC_FRAME_CMD       = 0x01U,  /**< PC→MCU 命令 */
    PC_FRAME_ACK       = 0x02U,  /**< MCU→PC 命令成功 */
    PC_FRAME_NACK      = 0x03U,  /**< MCU→PC 命令失败 */
    PC_FRAME_STATE     = 0x04U,  /**< MCU→PC 状态快照 */
    PC_FRAME_DATA      = 0x05U,  /**< MCU→PC 过程数据 */
    PC_FRAME_EVENT     = 0x06U,  /**< MCU→PC 事件通知 */
    PC_FRAME_HELLO     = 0x07U,  /**< 双向握手 */
    PC_FRAME_HEARTBEAT = 0x08U   /**< 双向心跳 */
} PcFrameType_t;

/* 命令队列条目 */
typedef struct
{
    uint8_t       pending;      /**< 1 = 有待处理命令 */
    PcFrameType_t type;         /**< 帧类型 */
    uint16_t      seq;          /**< PC 发来的 SEQ（ACK/NACK 回传用） */
    char          payload[256]; /**< ASCII 载荷（含结尾 \0） */
} PcCmdQueue_t;

/* API ----------------------------------------------------------------------*/

/** @brief 初始化：启动 USART2 DMA 空闲线接收 */
void PcProto_Init(void);

/** @brief 发送一帧（自动 seq，用于周期性 STATE/DATA/EVENT） */
void PcProto_SendFrame(PcFrameType_t type, const char *payload);

/** @brief 发送一帧（指定 seq，用于 ACK/NACK/HEARTBEAT 回传） */
void PcProto_SendFrameSeq(PcFrameType_t type, uint16_t seq,
                          const char *payload);

/** @brief 发送 Cell 状态帧 */
void PcProto_SendState(uint8_t cell);

/** @brief 发送 Cell 过程数据帧 */
void PcProto_SendData(uint8_t cell);

/** @brief 发送事件通知 */
void PcProto_SendEvent(uint8_t cell, const char *event_type);

/** @brief 主循环处理：执行收到的命令（非 ISR 上下文） */
void PcProto_Process(void);

/** @brief ISR 回调：处理内部 DMA 缓冲区中 len 字节的数据 */
void PcProto_OnRxData(uint16_t len);

/** @brief 重新启动 DMA 接收（ISR 中调用） */
void PcProto_RestartRx(void);

/** @brief USART2 需要重新启动接收的标志 */
extern volatile uint8_t g_uart2_need_restart;

#ifdef __cplusplus
}
#endif

#endif /* __PC_PROTOCOL_H */
