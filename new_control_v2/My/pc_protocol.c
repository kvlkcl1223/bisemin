/**
 ******************************************************************************
 * @file    pc_protocol.c
 * @brief   PC 上位机通信协议实现
 * @author  Bisemin Team
 * @version V1.0.0
 * @date    2026-07-12
 ******************************************************************************
 */

#include "pc_protocol.h"
#include "app_control.h"
#include "main.h"
#include "usart.h"
#include "cmsis_os.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ============================================================
 * 常量
 * ============================================================ */

#define PC_RX_BUF_SIZE      512U
#define PC_PAYLOAD_MAX      256U
#define PC_FRAME_LEN_MAX    (1U + 2U + PC_PAYLOAD_MAX)   /* TYPE+SEQ+PAYLOAD = 259 */
#define PC_BODY_BUF_SIZE    (PC_FRAME_LEN_MAX)

#define PC_SOF0  0xA5U
#define PC_SOF1  0x5AU
#define PC_EOF0  0x0DU
#define PC_EOF1  0x0AU

/* 帧状态机 */
#define PC_STATE_WAIT_SOF0  0U
#define PC_STATE_WAIT_SOF1  1U
#define PC_STATE_READ_LEN   2U
#define PC_STATE_READ_BODY  3U
#define PC_STATE_READ_CRC   4U
#define PC_STATE_READ_EOF   5U

/* ============================================================
 * 全局变量
 * ============================================================ */

volatile uint8_t g_uart2_need_restart = 0U;

/* ============================================================
 * 私有变量
 * ============================================================ */

static uint8_t s_rx_buf[PC_RX_BUF_SIZE];

/* 帧状态机 */
static uint8_t  s_fsm_state       = PC_STATE_WAIT_SOF0;
static uint16_t s_frm_len         = 0U;
static uint8_t  s_len_buf[2];
static uint8_t  s_len_idx         = 0U;
static uint8_t  s_body_buf[PC_BODY_BUF_SIZE];
static uint16_t s_body_idx        = 0U;
static uint8_t  s_crc_buf[2];
static uint8_t  s_crc_idx         = 0U;
static uint8_t  s_eof_prev_0d     = 0U;

/* 命令队列 */
static volatile PcCmdQueue_t s_cmd_queue;

/* 序号 */
static uint16_t s_tx_seq = 0U;

/* ============================================================
 * CRC16-CCITT（与 calib_mode.c 相同算法，独立实现）
 * ============================================================ */

static uint16_t PcProto_CRC16(const uint8_t *data, uint32_t len)
{
    static const uint16_t table[256] =
    {
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

/* ============================================================
 * 帧状态机（逐字节）
 * ============================================================ */

static void PcProto_FeedByte(uint8_t byte)
{
    switch (s_fsm_state)
    {
    case PC_STATE_WAIT_SOF0:
        if (byte == PC_SOF0)
        {
            s_fsm_state = PC_STATE_WAIT_SOF1;
        }
        break;

    case PC_STATE_WAIT_SOF1:
        if (byte == PC_SOF1)
        {
            s_fsm_state  = PC_STATE_READ_LEN;
            s_len_idx    = 0U;
        }
        else if (byte != PC_SOF0)
        {
            s_fsm_state = PC_STATE_WAIT_SOF0;
        }
        /* byte == PC_SOF0: stay in WAIT_SOF1 (re-sync) */
        break;

    case PC_STATE_READ_LEN:
        s_len_buf[s_len_idx] = byte;
        s_len_idx++;
        if (s_len_idx >= 2U)
        {
            s_frm_len = (uint16_t)s_len_buf[0]
                      | ((uint16_t)s_len_buf[1] << 8U);

            if (s_frm_len < 3U || s_frm_len > PC_FRAME_LEN_MAX)
            {
                /* LEN 非法，回到搜索帧头 */
                s_fsm_state = PC_STATE_WAIT_SOF0;
            }
            else
            {
                s_fsm_state = PC_STATE_READ_BODY;
                s_body_idx  = 0U;
            }
        }
        break;

    case PC_STATE_READ_BODY:
        if (s_body_idx < PC_BODY_BUF_SIZE)
        {
            s_body_buf[s_body_idx] = byte;
        }
        s_body_idx++;
        if (s_body_idx >= s_frm_len)
        {
            s_fsm_state = PC_STATE_READ_CRC;
            s_crc_idx   = 0U;
        }
        break;

    case PC_STATE_READ_CRC:
        s_crc_buf[s_crc_idx] = byte;
        s_crc_idx++;
        if (s_crc_idx >= 2U)
        {
            uint16_t rcv_crc = (uint16_t)s_crc_buf[0]
                             | ((uint16_t)s_crc_buf[1] << 8U);
            uint16_t calc_crc = PcProto_CRC16(s_body_buf, s_frm_len);

            if (rcv_crc == calc_crc)
            {
                s_fsm_state    = PC_STATE_READ_EOF;
                s_eof_prev_0d  = 0U;
            }
            else
            {
                s_fsm_state = PC_STATE_WAIT_SOF0;
            }
        }
        break;

    case PC_STATE_READ_EOF:
        if (s_eof_prev_0d != 0U)
        {
            if (byte == PC_EOF1)
            {
                /* 帧完整通过校验 → 入命令队列 */
                if (s_cmd_queue.pending == 0U)
                {
                    uint16_t pay_len = s_frm_len - 3U; /* TYPE+SEQ = 3 */

                    s_cmd_queue.type = (PcFrameType_t)s_body_buf[0];

                    if (pay_len > 0U && pay_len < sizeof(s_cmd_queue.payload))
                    {
                        (void)memcpy(s_cmd_queue.payload,
                                     &s_body_buf[3U], pay_len);
                        s_cmd_queue.payload[pay_len] = '\0';
                    }
                    else
                    {
                        s_cmd_queue.payload[0] = '\0';
                    }

                    s_cmd_queue.pending = 1U;
                }
                /* else: 上一命令尚未处理，丢弃 */
            }
            s_fsm_state = PC_STATE_WAIT_SOF0;
        }
        else if (byte == PC_EOF0)
        {
            s_eof_prev_0d = 1U;
        }
        else
        {
            s_fsm_state = PC_STATE_WAIT_SOF0;
        }
        break;

    default:
        s_fsm_state = PC_STATE_WAIT_SOF0;
        break;
    }
}

/* ============================================================
 * 公开 API
 * ============================================================ */

/**
 * @brief  启动 DMA 空闲线接收
 */
void PcProto_Init(void)
{
    HAL_StatusTypeDef ret;

    s_fsm_state = PC_STATE_WAIT_SOF0;
    s_cmd_queue.pending = 0U;
    g_uart2_need_restart = 0U;

    ret = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_rx_buf, PC_RX_BUF_SIZE);
    if (ret == HAL_OK)
    {
        if (huart2.hdmarx != NULL)
        {
            __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
        }
    }
}

/**
 * @brief  重新启动 DMA 接收（ISR 回调中调用）
 */
void PcProto_RestartRx(void)
{
    HAL_StatusTypeDef ret;

    ret = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_rx_buf, PC_RX_BUF_SIZE);
    if (ret == HAL_OK)
    {
        if (huart2.hdmarx != NULL)
        {
            __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
        }
    }
    else
    {
        g_uart2_need_restart = 1U;
    }
}

/**
 * @brief  ISR 回调：处理内部 DMA 缓冲区中 len 字节的数据
 */
void PcProto_OnRxData(uint16_t len)
{
    uint16_t i;

    if (len == 0U || len > PC_RX_BUF_SIZE)
        return;

    for (i = 0U; i < len; i++)
    {
        PcProto_FeedByte(s_rx_buf[i]);
    }
}

/**
 * @brief  发送一帧
 */
void PcProto_SendFrame(PcFrameType_t type, const char *payload)
{
    uint8_t  buf[280];  /* 最大帧 2+2+259+2+2 = 267 */
    uint16_t pay_len;
    uint16_t frm_len;
    uint16_t crc;
    uint16_t idx;
    uint16_t seq;

    pay_len = 0U;
    if (payload != NULL)
    {
        while (payload[pay_len] != '\0')
            pay_len++;
    }

    /* LEN = TYPE(1) + SEQ(2) + PAYLOAD(N) */
    frm_len = 1U + 2U + pay_len;

    idx = 0U;

    /* SOF */
    buf[idx++] = PC_SOF0;
    buf[idx++] = PC_SOF1;

    /* LEN (小端) */
    buf[idx++] = (uint8_t)(frm_len & 0xFFU);
    buf[idx++] = (uint8_t)((frm_len >> 8U) & 0xFFU);

    /* TYPE */
    buf[idx++] = (uint8_t)type;

    /* SEQ (小端) */
    seq = s_tx_seq;
    s_tx_seq++;
    buf[idx++] = (uint8_t)(seq & 0xFFU);
    buf[idx++] = (uint8_t)((seq >> 8U) & 0xFFU);

    /* PAYLOAD */
    if (pay_len > 0U && payload != NULL)
    {
        (void)memcpy(&buf[idx], payload, pay_len);
        idx += pay_len;
    }

    /* CRC (小端) —— 覆盖 LEN+TYPE+SEQ+PAYLOAD */
    crc = PcProto_CRC16(&buf[2], (uint32_t)(idx - 2U));
    buf[idx++] = (uint8_t)(crc & 0xFFU);
    buf[idx++] = (uint8_t)((crc >> 8U) & 0xFFU);

    /* EOF */
    buf[idx++] = PC_EOF0;
    buf[idx++] = PC_EOF1;

    (void)HAL_UART_Transmit(&huart2, buf, idx, 50U);
}

/**
 * @brief  发送 Cell 状态帧
 */
void PcProto_SendState(uint8_t cell)
{
    char pay[256];
    int  len;

    if (cell >= APP_CONTROL_CELL_COUNT)
        return;

    len = snprintf(pay, sizeof(pay),
                   "t=%lu,cell=%u,mode=%s,owner=%s,running=%u,"
                   "target=%.1f,command=%.1f,current=%.1f,"
                   "t0=%.1f,t1=%.1f,duty=%.3f,error=%u,phase=0",
                   (unsigned long)osKernelGetTickCount(),
                   (unsigned int)cell,
                   (g_app_control_cell_running[cell] != 0U) ? "NORMAL" : "STOP",
                   "PANEL",
                   (unsigned int)g_app_control_cell_running[cell],
                   (double)g_app_control_cell_target[cell],
                   (double)g_app_control_cell_target[cell],
                   (double)g_app_control_cell_temp[cell],
                   (double)((cell == 0U) ? g_app_control_pid_temp[0]
                                         : g_app_control_pid_temp[2]),
                   (double)((cell == 0U) ? g_app_control_pid_temp[1]
                                         : g_app_control_pid_temp[3]),
                   (double)g_app_control_cell_duty[cell],
                   (unsigned int)g_app_control_cell_error[cell]);

    if (len > 0 && len < (int)sizeof(pay))
        PcProto_SendFrame(PC_FRAME_STATE, pay);
}

/**
 * @brief  发送 Cell 过程数据帧
 */
void PcProto_SendData(uint8_t cell)
{
    /* DATA 格式与 STATE 相同 */
    PcProto_SendState(cell);
}

/**
 * @brief  发送事件通知
 */
void PcProto_SendEvent(uint8_t cell, const char *event_type)
{
    char pay[128];
    int  len;

    if (cell >= APP_CONTROL_CELL_COUNT || event_type == NULL)
        return;

    len = snprintf(pay, sizeof(pay),
                   "t=%lu,type=%s,cell=%u",
                   (unsigned long)osKernelGetTickCount(),
                   event_type,
                   (unsigned int)cell);

    if (len > 0 && len < (int)sizeof(pay))
        PcProto_SendFrame(PC_FRAME_EVENT, pay);
}

/* ============================================================
 * 命令处理（主循环中调用，非 ISR）
 * ============================================================ */

/**
 * @brief  简单的 key=value 解析：在 payload 中查找 key 并返回 value
 * @return value 字符串指针，找不到返回 NULL
 */
static const char *PcProto_GetValue(const char *payload, const char *key)
{
    const char *p;
    uint32_t    key_len;

    if (payload == NULL || key == NULL)
        return NULL;

    key_len = (uint32_t)strlen(key);

    p = payload;
    while (*p != '\0')
    {
        /* 匹配 key= */
        if (strncmp(p, key, key_len) == 0 && p[key_len] == '=')
        {
            return &p[key_len + 1U];
        }

        /* 跳到下一个逗号 */
        while (*p != '\0' && *p != ',')
            p++;
        if (*p == ',')
            p++;
    }

    return NULL;
}

/**
 * @brief  主循环处理：执行收到的命令
 */
void PcProto_Process(void)
{
    const char *pay;    /* volatile 转 const */
    const char *op;
    const char *val;
    uint8_t     cell;
    float       temp;

    if (s_cmd_queue.pending == 0U)
        return;

    s_cmd_queue.pending = 0U;

    /* 只处理 CMD 类型 */
    if (s_cmd_queue.type != PC_FRAME_CMD)
        return;

    pay = (const char *)s_cmd_queue.payload;

    op = PcProto_GetValue(pay, "op");
    if (op == NULL)
        return;

    /* ---- GET_STATE ---- */
    if (strcmp(op, "GET_STATE") == 0)
    {
        PcProto_SendFrame(PC_FRAME_ACK, "ok=1");
        PcProto_SendState(0);
        PcProto_SendState(1);
        return;
    }

    /* cell 参数 */
    val  = PcProto_GetValue(pay, "cell");
    cell = (val != NULL) ? (uint8_t)(val[0] - '0') : 0xFFU;
    if (cell >= APP_CONTROL_CELL_COUNT && strcmp(op, "STOP_ALL") != 0)
    {
        PcProto_SendFrame(PC_FRAME_NACK, "ok=0,err=1002,msg=BAD_CELL");
        return;
    }

    /* ---- SET_TARGET ---- */
    if (strcmp(op, "SET_TARGET") == 0)
    {
        val = PcProto_GetValue(pay, "temp");
        if (val == NULL)
        {
            PcProto_SendFrame(PC_FRAME_NACK, "ok=0,err=1002,msg=MISSING_TEMP");
            return;
        }
        temp = (float)atof(val);

        Control_SetTargetTemp(cell, temp);
        PcProto_SendFrame(PC_FRAME_ACK, "ok=1");
        PcProto_SendState(cell);
        return;
    }

    /* ---- START_NORMAL ---- */
    if (strcmp(op, "START_NORMAL") == 0)
    {
        val = PcProto_GetValue(pay, "temp");
        if (val != NULL)
        {
            temp = (float)atof(val);
            Control_SetTargetTemp(cell, temp);
        }

        Control_StartPid(cell);
        PcProto_SendFrame(PC_FRAME_ACK, "ok=1");
        PcProto_SendEvent(cell, "START");
        PcProto_SendState(cell);
        return;
    }

    /* ---- STOP ---- */
    if (strcmp(op, "STOP") == 0)
    {
        Control_StopPid(cell);
        PcProto_SendFrame(PC_FRAME_ACK, "ok=1");
        PcProto_SendEvent(cell, "STOP");
        PcProto_SendState(cell);
        return;
    }

    /* ---- STOP_ALL ---- */
    if (strcmp(op, "STOP_ALL") == 0)
    {
        Control_StopPid(0);
        Control_StopPid(1);
        PcProto_SendFrame(PC_FRAME_ACK, "ok=1");
        PcProto_SendEvent(0, "STOP");
        PcProto_SendEvent(1, "STOP");
        return;
    }

    /* 未知命令 */
    PcProto_SendFrame(PC_FRAME_NACK, "ok=0,err=1003,msg=UNKNOWN_OP");
}
