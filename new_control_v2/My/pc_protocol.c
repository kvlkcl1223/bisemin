/**
 ******************************************************************************
 * @file    pc_protocol.c
 * @brief   PC 涓婁綅鏈洪€氫俊鍗忚瀹炵幇
 * @author  Bisemin Team
 * @version V1.1.0
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
 * 甯搁噺
 * ============================================================ */

#define PC_RX_BUF_SIZE      512U
#define PC_PAYLOAD_MAX      256U
#define PC_FRAME_LEN_MAX    (2U + 1U + 2U + PC_PAYLOAD_MAX)  /* LEN+TYPE+SEQ+PAYLOAD */
#define PC_BODY_BUF_SIZE    (PC_FRAME_LEN_MAX)

#define PC_SOF0  0xA5U
#define PC_SOF1  0x5AU
#define PC_EOF0  0x0DU
#define PC_EOF1  0x0AU

/* 甯х姸鎬佹満 */
#define PC_STATE_WAIT_SOF0  0U
#define PC_STATE_WAIT_SOF1  1U
#define PC_STATE_READ_LEN   2U
#define PC_STATE_READ_BODY  3U
#define PC_STATE_READ_CRC   4U
#define PC_STATE_READ_EOF   5U

/* ============================================================
 * 鍏ㄥ眬鍙橀噺
 * ============================================================ */

volatile uint8_t g_uart2_need_restart = 0U;

/* ============================================================
 * 绉佹湁鍙橀噺
 * ============================================================ */

static uint8_t s_rx_buf[PC_RX_BUF_SIZE];

/* 甯х姸鎬佹満 */
static uint8_t  s_fsm_state       = PC_STATE_WAIT_SOF0;
static uint16_t s_frm_len         = 0U;   /* LEN + TYPE + SEQ + PAYLOAD (鍚?LEN) */
static uint8_t  s_len_buf[2];
static uint8_t  s_len_idx         = 0U;
static uint8_t  s_body_buf[PC_BODY_BUF_SIZE];
static uint16_t s_body_idx        = 0U;
static uint8_t  s_crc_buf[2];
static uint8_t  s_crc_idx         = 0U;
static uint8_t  s_eof_prev_0d     = 0U;

/* 鍛戒护闃熷垪 */
static volatile PcCmdQueue_t s_cmd_queue;

/* 鍙戦€佸簭鍙凤紙鍛ㄦ湡鎬у抚鐢級 */
static uint16_t s_tx_seq = 0U;
static uint8_t  s_pc_owner[APP_CONTROL_CELL_COUNT] = {0U, 0U};

static const char *PcProto_ModeString(uint8_t cell)
{
    if (cell >= APP_CONTROL_CELL_COUNT)
        return "STOP";

    if (g_panel.cell[cell].run_mode == CELL_RUN_PROGRAM)
        return "PROGRAM";
    if (g_panel.cell[cell].run_mode == CELL_RUN_EXTERNAL)
        return "EXTERNAL";
    if (g_app_control_cell_running[cell] != 0U ||
        g_panel.cell[cell].run_mode == CELL_RUN_JUMP)
        return "NORMAL";

    return "STOP";
}

static const char *PcProto_OwnerString(uint8_t cell)
{
    if (cell < APP_CONTROL_CELL_COUNT && s_pc_owner[cell] != 0U)
        return "PC";
    return "PANEL";
}

/* ============================================================
 * CRC16-CCITT
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
 * payload key=value 瑙ｆ瀽
 * ============================================================ */

/**
 * @brief  浠?payload 涓彇 key=value锛屾嫹璐濆埌 dst 骞惰ˉ \0
 * @return 1=鎵惧埌, 0=鏈壘鍒? */
static uint8_t PcProto_GetValue(const char *payload, const char *key,
                                char *dst, uint16_t dst_size)
{
    uint32_t key_len;
    uint16_t vi;
    const char *p;

    if (payload == NULL || key == NULL || dst == NULL || dst_size == 0U)
        return 0U;

    key_len = (uint32_t)strlen(key);

    p = payload;
    while (*p != '\0')
    {
        if (strncmp(p, key, key_len) == 0 && p[key_len] == '=')
        {
            /* 鎵惧埌 key=锛屾嫹璐?value 鐩村埌閫楀彿鎴栫粨灏?*/
            p += (key_len + 1U);
            vi = 0U;
            while (*p != '\0' && *p != ',' && vi < (dst_size - 1U))
            {
                dst[vi++] = *p;
                p++;
            }
            dst[vi] = '\0';
            return 1U;
        }

        /* 璺冲埌涓嬩竴涓€楀彿 */
        while (*p != '\0' && *p != ',')
            p++;
        if (*p == ',')
            p++;
    }

    return 0U;
}

/* ============================================================
 * 甯х姸鎬佹満锛堥€愬瓧鑺傦級
 * ============================================================ */

static void PcProto_FeedByte(uint8_t byte)
{
    switch (s_fsm_state)
    {
    case PC_STATE_WAIT_SOF0:
        if (byte == PC_SOF0)
            s_fsm_state = PC_STATE_WAIT_SOF1;
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
        break;

    case PC_STATE_READ_LEN:
        s_len_buf[s_len_idx] = byte;
        s_len_idx++;
        if (s_len_idx >= 2U)
        {
            uint16_t raw_len;

            raw_len = (uint16_t)s_len_buf[0]
                    | ((uint16_t)s_len_buf[1] << 8U);

            if (raw_len < 3U || raw_len > (PC_FRAME_LEN_MAX - 2U))
            {
                /* LEN 闈炴硶 */
                s_fsm_state = PC_STATE_WAIT_SOF0;
            }
            else
            {
                /*
                 * Fix #1: CRC 瑕嗙洊 LEN+TYPE+SEQ+PAYLOAD
                 * 鎶?LEN 鍐欏叆 body_buf 寮€澶达紝CRC 鏍￠獙鏃?s_body_buf[0..] 鍖呭惈 LEN
                 */
                s_body_buf[0] = s_len_buf[0];
                s_body_buf[1] = s_len_buf[1];
                s_body_idx    = 2U;               /* TYPE+SEQ+PAYLOAD 浠庡亸绉?2 寮€濮?*/
                s_frm_len     = raw_len + 2U;     /* CRC 瑕嗙洊 LEN(2) + raw_len */
                s_fsm_state   = PC_STATE_READ_BODY;
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
            uint16_t rcv_crc  = (uint16_t)s_crc_buf[0]
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
                /* 甯ф牎楠岄€氳繃 鈫?鍏ュ懡浠ら槦鍒?*/
                if (s_cmd_queue.pending == 0U)
                {
                    uint16_t pay_len;
                    uint16_t body_offset;

                    /* s_body_buf: [LEN_L][LEN_H][TYPE][SEQ_L][SEQ_H][PAYLOAD...] */
                    s_cmd_queue.type = (PcFrameType_t)s_body_buf[2];
                    s_cmd_queue.seq  = (uint16_t)s_body_buf[3]
                                     | ((uint16_t)s_body_buf[4] << 8U);

                    body_offset = 5U;  /* 璺宠繃 LEN(2)+TYPE(1)+SEQ(2) */
                    pay_len     = s_frm_len - body_offset;

                    if (pay_len > 0U && pay_len < sizeof(s_cmd_queue.payload))
                    {
                        (void)memcpy(s_cmd_queue.payload,
                                     &s_body_buf[body_offset], pay_len);
                        s_cmd_queue.payload[pay_len] = '\0';
                    }
                    else
                    {
                        s_cmd_queue.payload[0] = '\0';
                    }

                    s_cmd_queue.pending = 1U;
                }
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
 * 鍏紑 API 鈥?鍒濆鍖?/ 鎺ユ敹 / 閲嶅惎
 * ============================================================ */

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

/* ============================================================
 * 鍏紑 API 鈥?鍙戦€? * ============================================================ */

/**
 * @brief  鍙戦€佷竴甯э紙鑷姩 seq锛? */
void PcProto_SendFrame(PcFrameType_t type, const char *payload)
{
    PcProto_SendFrameSeq(type, s_tx_seq, payload);
    s_tx_seq++;
}

/**
 * @brief  鍙戦€佷竴甯э紙鎸囧畾 seq锛? */
void PcProto_SendFrameSeq(PcFrameType_t type, uint16_t seq,
                          const char *payload)
{
    uint8_t  buf[280];
    uint16_t pay_len;
    uint16_t frm_len;
    uint16_t crc;
    uint16_t idx;

    pay_len = 0U;
    if (payload != NULL)
    {
        while (payload[pay_len] != '\0')
            pay_len++;
    }

    /* LEN = TYPE(1) + SEQ(2) + PAYLOAD(N) 鈥?涓嶅惈 LEN 鑷韩 */
    frm_len = 1U + 2U + pay_len;

    idx = 0U;

    /* SOF */
    buf[idx++] = PC_SOF0;
    buf[idx++] = PC_SOF1;

    /* LEN (灏忕) */
    buf[idx++] = (uint8_t)(frm_len & 0xFFU);
    buf[idx++] = (uint8_t)((frm_len >> 8U) & 0xFFU);

    /* TYPE */
    buf[idx++] = (uint8_t)type;

    /* SEQ (灏忕) */
    buf[idx++] = (uint8_t)(seq & 0xFFU);
    buf[idx++] = (uint8_t)((seq >> 8U) & 0xFFU);

    /* PAYLOAD */
    if (pay_len > 0U && payload != NULL)
    {
        (void)memcpy(&buf[idx], payload, pay_len);
        idx += pay_len;
    }

    /* CRC (灏忕) 鈥斺€?瑕嗙洊 LEN+TYPE+SEQ+PAYLOAD */
    crc = PcProto_CRC16(&buf[2], (uint32_t)(idx - 2U));
    buf[idx++] = (uint8_t)(crc & 0xFFU);
    buf[idx++] = (uint8_t)((crc >> 8U) & 0xFFU);

    /* EOF */
    buf[idx++] = PC_EOF0;
    buf[idx++] = PC_EOF1;

    (void)HAL_UART_Transmit(&huart2, buf, idx, 50U);
}

/**
 * @brief  鍙戦€?Cell 鐘舵€佸抚
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
                   "t0=%.1f,t1=%.1f,duty=%.3f,error=%u,phase=%u",
                   (unsigned long)osKernelGetTickCount(),
                   (unsigned int)cell,
                   PcProto_ModeString(cell),
                   PcProto_OwnerString(cell),
                   (unsigned int)g_app_control_cell_running[cell],
                   (double)g_app_control_cell_target[cell],
                   (double)g_panel.cell[cell].command_temp,
                   (double)g_app_control_cell_temp[cell],
                   (double)((cell == 0U) ? g_app_control_pid_temp[0]
                                         : g_app_control_pid_temp[2]),
                   (double)((cell == 0U) ? g_app_control_pid_temp[1]
                                         : g_app_control_pid_temp[3]),
                   (double)g_app_control_cell_duty[cell],
                   (unsigned int)g_app_control_cell_error[cell],
                   (unsigned int)g_panel.cell[cell].program_phase);

    if (len > 0 && len < (int)sizeof(pay))
        PcProto_SendFrame(PC_FRAME_STATE, pay);
}

/**
 * @brief  鍙戦€?Cell 杩囩▼鏁版嵁甯э紙Fix #5: 鐪熸鍙?DATA 绫诲瀷锛? */
void PcProto_SendData(uint8_t cell)
{
    char pay[256];
    int  len;

    if (cell >= APP_CONTROL_CELL_COUNT)
        return;

    len = snprintf(pay, sizeof(pay),
                   "t=%lu,cell=%u,mode=%s,owner=%s,running=%u,"
                   "target=%.1f,command=%.1f,current=%.1f,"
                   "t0=%.1f,t1=%.1f,duty=%.3f,error=%u,phase=%u",
                   (unsigned long)osKernelGetTickCount(),
                   (unsigned int)cell,
                   PcProto_ModeString(cell),
                   PcProto_OwnerString(cell),
                   (unsigned int)g_app_control_cell_running[cell],
                   (double)g_app_control_cell_target[cell],
                   (double)g_panel.cell[cell].command_temp,
                   (double)g_app_control_cell_temp[cell],
                   (double)((cell == 0U) ? g_app_control_pid_temp[0]
                                         : g_app_control_pid_temp[2]),
                   (double)((cell == 0U) ? g_app_control_pid_temp[1]
                                         : g_app_control_pid_temp[3]),
                   (double)g_app_control_cell_duty[cell],
                   (unsigned int)g_app_control_cell_error[cell],
                   (unsigned int)g_panel.cell[cell].program_phase);

    if (len > 0 && len < (int)sizeof(pay))
        PcProto_SendFrame(PC_FRAME_DATA, pay);
}

/**
 * @brief  鍙戦€佷簨浠堕€氱煡
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
 * 鍛戒护澶勭悊锛堜富寰幆涓皟鐢紝闈?ISR锛? * ============================================================ */

void PcProto_Process(void)
{
    char        op_buf[32];
    char        val_buf[32];
    uint16_t    rcv_seq;
    const char *pay;
    uint8_t     cell;
    float       temp;

    /* Fix #6: UART2 鎺ユ敹閿欒鍚庤嚜鍔ㄦ仮澶?*/
    if (g_uart2_need_restart != 0U)
    {
        g_uart2_need_restart = 0U;
        PcProto_RestartRx();
    }

    if (s_cmd_queue.pending == 0U)
        return;

    s_cmd_queue.pending = 0U;
    rcv_seq = s_cmd_queue.seq;
    pay     = (const char *)s_cmd_queue.payload;

    /* ---- HELLO ---- */
    if (s_cmd_queue.type == PC_FRAME_HELLO)
    {
        PcProto_SendFrameSeq(PC_FRAME_HELLO, rcv_seq,
                             "role=MCU,proto=1,fw=1.0.0,cells=2");
        return;
    }

    /* ---- HEARTBEAT ---- */
    if (s_cmd_queue.type == PC_FRAME_HEARTBEAT)
    {
        PcProto_SendFrameSeq(PC_FRAME_HEARTBEAT, rcv_seq, "ok=1");
        return;
    }

    /* 鍙鐞?CMD 绫诲瀷 */
    if (s_cmd_queue.type != PC_FRAME_CMD)
        return;

    /* 瑙ｆ瀽 op */
    if (!PcProto_GetValue(pay, "op", op_buf, sizeof(op_buf)))
        return;

    /* ---- GET_STATE ---- */
    if (strcmp(op_buf, "GET_STATE") == 0)
    {
        PcProto_SendFrameSeq(PC_FRAME_ACK, rcv_seq, "ok=1");
        PcProto_SendState(0);
        PcProto_SendState(1);
        return;
    }

    /* cell 鍙傛暟 */
    if (!PcProto_GetValue(pay, "cell", val_buf, sizeof(val_buf)))
        cell = 0xFFU;
    else
        cell = (uint8_t)atoi(val_buf);

    if (cell >= APP_CONTROL_CELL_COUNT && strcmp(op_buf, "STOP_ALL") != 0)
    {
        PcProto_SendFrameSeq(PC_FRAME_NACK, rcv_seq,
                             "ok=0,err=1002,msg=BAD_CELL");
        return;
    }

    /* ---- SET_TARGET ---- */
    if (strcmp(op_buf, "SET_TARGET") == 0)
    {
        if (!PcProto_GetValue(pay, "temp", val_buf, sizeof(val_buf)))
        {
            PcProto_SendFrameSeq(PC_FRAME_NACK, rcv_seq,
                                 "ok=0,err=1002,msg=MISSING_TEMP");
            return;
        }
        temp = (float)atof(val_buf);

        Control_SetTargetTemp(cell, temp);
        s_pc_owner[cell] = 1U;
        PcProto_SendFrameSeq(PC_FRAME_ACK, rcv_seq, "ok=1");
        PcProto_SendState(cell);
        return;
    }

    /* ---- START_NORMAL ---- */
    if (strcmp(op_buf, "START_NORMAL") == 0)
    {
        if (PcProto_GetValue(pay, "temp", val_buf, sizeof(val_buf)))
        {
            temp = (float)atof(val_buf);
            Control_SetTargetTemp(cell, temp);
        }

        Control_StartPid(cell);
        s_pc_owner[cell] = 1U;
        PcProto_SendFrameSeq(PC_FRAME_ACK, rcv_seq, "ok=1");
        PcProto_SendEvent(cell, "START");
        PcProto_SendState(cell);
        return;
    }

    /* ---- STOP ---- */
    if (strcmp(op_buf, "STOP") == 0)
    {
        TempPanel_Stop(&g_panel, cell);
        s_pc_owner[cell] = 1U;
        PcProto_SendFrameSeq(PC_FRAME_ACK, rcv_seq, "ok=1");
        PcProto_SendEvent(cell, "STOP");
        PcProto_SendState(cell);
        return;
    }

    /* ---- STOP_ALL ---- */
    if (strcmp(op_buf, "STOP_ALL") == 0)
    {
        TempPanel_Stop(&g_panel, 0);
        TempPanel_Stop(&g_panel, 1);
        s_pc_owner[0] = 1U;
        s_pc_owner[1] = 1U;
        PcProto_SendFrameSeq(PC_FRAME_ACK, rcv_seq, "ok=1");
        PcProto_SendEvent(0, "STOP");
        PcProto_SendEvent(1, "STOP");
        return;
    }
    /* ---- SET_PROGRAM ---- */
    if (strcmp(op_buf, "SET_PROGRAM") == 0)
    {
        TempProgram_t program;

        if (!PcProto_GetValue(pay, "start", val_buf, sizeof(val_buf)))
        {
            PcProto_SendFrameSeq(PC_FRAME_NACK, rcv_seq,
                                 "ok=0,err=1002,msg=MISSING_START");
            return;
        }
        program.start_temp = (float)atof(val_buf);

        if (!PcProto_GetValue(pay, "hold", val_buf, sizeof(val_buf)))
        {
            PcProto_SendFrameSeq(PC_FRAME_NACK, rcv_seq,
                                 "ok=0,err=1002,msg=MISSING_HOLD");
            return;
        }
        program.start_hold_s = (uint16_t)atoi(val_buf);

        if (!PcProto_GetValue(pay, "rate", val_buf, sizeof(val_buf)))
        {
            PcProto_SendFrameSeq(PC_FRAME_NACK, rcv_seq,
                                 "ok=0,err=1002,msg=MISSING_RATE");
            return;
        }
        program.ramp_rate = (float)atof(val_buf);

        if (!PcProto_GetValue(pay, "next", val_buf, sizeof(val_buf)))
        {
            PcProto_SendFrameSeq(PC_FRAME_NACK, rcv_seq,
                                 "ok=0,err=1002,msg=MISSING_NEXT");
            return;
        }
        program.next_temp = (float)atof(val_buf);

        if (!PcProto_GetValue(pay, "wait", val_buf, sizeof(val_buf)))
        {
            PcProto_SendFrameSeq(PC_FRAME_NACK, rcv_seq,
                                 "ok=0,err=1002,msg=MISSING_WAIT");
            return;
        }
        program.wait_s = (uint16_t)atoi(val_buf);

        if (!PcProto_GetValue(pay, "repeat", val_buf, sizeof(val_buf)))
        {
            PcProto_SendFrameSeq(PC_FRAME_NACK, rcv_seq,
                                 "ok=0,err=1002,msg=MISSING_REPEAT");
            return;
        }
        program.repeat_times = (uint16_t)atoi(val_buf);

        if (TempPanel_SetProgram(&g_panel, cell, &program) == 0U)
        {
            PcProto_SendFrameSeq(PC_FRAME_NACK, rcv_seq,
                                 "ok=0,err=1002,msg=BAD_STATE");
            return;
        }

        s_pc_owner[cell] = 1U;
        PcProto_SendFrameSeq(PC_FRAME_ACK, rcv_seq, "ok=1");
        PcProto_SendEvent(cell, "PROGRAM_PARAM");
        PcProto_SendState(cell);
        return;
    }

    /* ---- START_PROGRAM ---- */
    if (strcmp(op_buf, "START_PROGRAM") == 0)
    {
        if (TempPanel_StartProgram(&g_panel, cell) == 0U)
        {
            PcProto_SendFrameSeq(PC_FRAME_NACK, rcv_seq,
                                 "ok=0,err=1002,msg=BAD_STATE");
            return;
        }

        s_pc_owner[cell] = 1U;
        PcProto_SendFrameSeq(PC_FRAME_ACK, rcv_seq, "ok=1");
        PcProto_SendEvent(cell, "START_PROGRAM");
        PcProto_SendState(cell);
        return;
    }
/* ---- LOG_START / LOG_STOP (鍗犱綅) ---- */
    if (strcmp(op_buf, "LOG_START") == 0
        || strcmp(op_buf, "LOG_STOP") == 0)
    {
        PcProto_SendFrameSeq(PC_FRAME_ACK, rcv_seq, "ok=1");
        return;
    }

    /* 鏈煡鍛戒护 */
    PcProto_SendFrameSeq(PC_FRAME_NACK, rcv_seq,
                         "ok=0,err=1003,msg=UNKNOWN_OP");
}



