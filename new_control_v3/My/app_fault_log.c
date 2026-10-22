#include "app_fault_log.h"
#include "app_control.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#ifdef UART_LOG_ENABLE
static void AppFault_Send(const char *text)
{
    (void)HAL_UART_Transmit(&huart2, (const uint8_t *)text,
                           (uint16_t)strlen(text), 20U);
}

static void AppFault_LogRegs(uint8_t drv, const char *source,
                             uint8_t mask, uint8_t rejected,
                             const volatile uint8_t *regs,
                             const volatile DRV8703_Status_t *status)
{
    char buf[224];
    uint8_t i;
    uint8_t all_ff = 1U;
    uint8_t r[DRV8703_REGISTER_COUNT];

    for (i = 0U; i < DRV8703_REGISTER_COUNT; i++)
    {
        r[i] = regs[i];
        if (r[i] != 0xFFU)
            all_ff = 0U;
    }
    (void)snprintf(buf, sizeof(buf),
        "FAULT,DRV:%u,SOURCE:%s,READ_OK:0x%02X,ALL_FF:%u,REJECTED:%u\r\n",
        (unsigned int)drv + 1U, source,
        (unsigned int)mask, (unsigned int)all_ff, (unsigned int)rejected);
    AppFault_Send(buf);
    (void)snprintf(buf, sizeof(buf),
        "FAULT,DRV:%u,REG0:0x%02X,REG1:0x%02X,REG2:0x%02X,REG3:0x%02X,REG4:0x%02X,REG5:0x%02X\r\n",
        (unsigned int)drv + 1U, (unsigned int)r[0], (unsigned int)r[1],
        (unsigned int)r[2], (unsigned int)r[3], (unsigned int)r[4], (unsigned int)r[5]);
    AppFault_Send(buf);
    (void)snprintf(buf, sizeof(buf),
        "FAULT,DRV:%u,REG_STATUS:%u,%u,%u,%u,%u,%u\r\n",
        (unsigned int)drv + 1U, (unsigned int)status[0], (unsigned int)status[1],
        (unsigned int)status[2], (unsigned int)status[3],
        (unsigned int)status[4], (unsigned int)status[5]);
    AppFault_Send(buf);

    if (strcmp(source, "CONFIG_LAST") == 0)
        return;
    /* Do not decode failed reads or rejected samples as hardware faults. */
    if ((mask & 0x03U) != 0x03U || r[0] == 0xFFU || r[1] == 0xFFU || rejected != 0U)
    {
        (void)snprintf(buf, sizeof(buf), "FAULT,DRV:%u,DECODE:UNAVAILABLE\r\n",
                       (unsigned int)drv + 1U);
        AppFault_Send(buf);
        return;
    }
    (void)snprintf(buf, sizeof(buf),
        "FAULT,DRV:%u,BITS0,FAULT:%u,WDFLT:%u,GDF:%u,OCP:%u,VM_UVFL:%u,VCP_UVFL:%u,OTSD:%u,OTW:%u\r\n",
        (unsigned int)drv + 1U,
        !!(r[0] & DRV8703_FAULT_FAULT), !!(r[0] & DRV8703_FAULT_WDFLT),
        !!(r[0] & DRV8703_FAULT_GDF), !!(r[0] & DRV8703_FAULT_OCP),
        !!(r[0] & DRV8703_FAULT_VM_UVFL), !!(r[0] & DRV8703_FAULT_VCP_UVFL),
        !!(r[0] & DRV8703_FAULT_OTSD), !!(r[0] & DRV8703_FAULT_OTW));
    AppFault_Send(buf);
    (void)snprintf(buf, sizeof(buf),
        "FAULT,DRV:%u,BITS1,H2_GDF:%u,L2_GDF:%u,H1_GDF:%u,L1_GDF:%u,H2_VDS:%u,L2_VDS:%u,H1_VDS:%u,L1_VDS:%u\r\n",
        (unsigned int)drv + 1U,
        !!(r[1] & DRV8703_VDSGDF_H2_GDF), !!(r[1] & DRV8703_VDSGDF_L2_GDF),
        !!(r[1] & DRV8703_VDSGDF_H1_GDF), !!(r[1] & DRV8703_VDSGDF_L1_GDF),
        !!(r[1] & DRV8703_VDSGDF_H2_VDS), !!(r[1] & DRV8703_VDSGDF_L2_VDS),
        !!(r[1] & DRV8703_VDSGDF_H1_VDS), !!(r[1] & DRV8703_VDSGDF_L1_VDS));
    AppFault_Send(buf);
}
#endif

void AppFault_LogCell(const char *mode, uint8_t cell, uint32_t error)
{
#ifdef UART_LOG_ENABLE
    char buf[224];
    uint8_t drv;
    uint8_t outer, inner;
    if (mode == 0 || cell >= APP_CONTROL_CELL_COUNT)
        return;
    outer = (cell == 0U) ? APP_CONTROL_CELL0_DRV_OUTER : APP_CONTROL_CELL1_DRV_OUTER;
    inner = (cell == 0U) ? APP_CONTROL_CELL0_DRV_INNER : APP_CONTROL_CELL1_DRV_INNER;
    (void)snprintf(buf, sizeof(buf),
        "FAULT,BEGIN,MODE:%s,CELL:%u,ERROR:%lu,LAST_DRV_INDEX:%u,LAST_STATUS:%u,SOURCE:STORED_SNAPSHOTS\r\n",
        mode, (unsigned int)cell, (unsigned long)error,
        (unsigned int)g_app_control_last_drv_fault, (unsigned int)g_app_control_last_drv_status);
    AppFault_Send(buf);
    for (drv = 0U; drv < APP_CONTROL_DRV_COUNT; drv++)
    {
        uint8_t relevant = (drv == outer || drv == inner ||
            (APP_CONTROL_SHARED_DRV_ENABLE && drv == APP_CONTROL_SHARED_DRV));
        uint8_t config_mask = 0U;
        uint8_t reg;
        if (relevant == 0U && g_app_control_drv_fault[drv] == 0U)
            continue;
        (void)snprintf(buf, sizeof(buf),
            "FAULT,DRV:%u,RELATED:%u,FLAG:%u,READY:%u,AWAKE:%u,VERIFY_OK:%u,MISMATCH:0x%02X,PIN_STATUS:%u,STOP_BITS:0x%02X,PIN_COUNT:%lu,CAPTURE_COUNT:%lu\r\n",
            (unsigned int)drv + 1U, (unsigned int)relevant,
            (unsigned int)g_app_control_drv_fault[drv],
            (unsigned int)g_app_control_drv_ready[drv], (unsigned int)g_app_control_drv_awake[drv],
            (unsigned int)g_app_control_drv_config_verify_ok[drv],
            (unsigned int)g_app_control_drv_config_verify_mismatch_mask[drv],
            (unsigned int)g_app_control_drv_pin_fault_status[drv],
            (unsigned int)g_app_control_drv_pin_fault_stop_bits[drv],
            (unsigned long)g_app_control_drv_pin_fault_count[drv],
            (unsigned long)g_app_control_drv_fault_capture_count[drv]);
        AppFault_Send(buf);
        if (g_app_control_drv_pin_fault_count[drv] != 0U)
            AppFault_LogRegs(drv, "PIN_LAST",
                g_app_control_drv_pin_fault_reg_read_ok_mask[drv],
                g_app_control_drv_pin_fault_all_ff[drv],
                g_app_control_drv_pin_fault_reg_dump[drv],
                g_app_control_drv_pin_fault_reg_read_status[drv]);
        if (g_app_control_drv_fault_capture_count[drv] != 0U)
        {
            uint8_t rejected = 0U;
            /* The pin handler also copies rejected data into the general cache. */
            if (g_app_control_drv_pin_fault_count[drv] != 0U &&
                g_app_control_drv_pin_fault_all_ff[drv] != 0U)
            {
                rejected = 1U;
                for (reg = 0U; reg < DRV8703_REGISTER_COUNT; reg++)
                    if (g_app_control_drv_reg_dump[drv][reg] !=
                        g_app_control_drv_pin_fault_reg_dump[drv][reg])
                        rejected = 0U;
            }
            AppFault_LogRegs(drv, "FAULT_CACHE",
                g_app_control_drv_reg_read_ok_mask[drv], rejected,
                g_app_control_drv_reg_dump[drv], g_app_control_drv_reg_read_status[drv]);
        }
        if (g_app_control_drv_config_verify_attempts[drv] != 0U)
        {
            for (reg = 0U; reg < DRV8703_REGISTER_COUNT; reg++)
                if (g_app_control_drv_config_verify_reg_status[drv][reg] == DRV8703_OK)
                    config_mask |= (uint8_t)(1U << reg);
            AppFault_LogRegs(drv, "CONFIG_LAST",
                config_mask, (uint8_t)(g_app_control_drv_config_verify_mismatch_mask[drv] != 0U),
                g_app_control_drv_config_verify_reg_dump[drv],
                g_app_control_drv_config_verify_reg_status[drv]);
        }
        if (g_app_control_drv_pin_fault_count[drv] == 0U &&
            g_app_control_drv_fault_capture_count[drv] == 0U &&
            g_app_control_drv_config_verify_attempts[drv] == 0U)
        {
            (void)snprintf(buf, sizeof(buf), "FAULT,DRV:%u,SOURCE:NONE,REGS:UNAVAILABLE\r\n",
                           (unsigned int)drv + 1U);
            AppFault_Send(buf);
        }
    }
    (void)snprintf(buf, sizeof(buf),
        "FAULT,EXPECTED,REG2:0x%02X,REG3:0x%02X,REG4:0x%02X,REG5:0x%02X\r\n",
        (unsigned int)g_app_control_drv_startup_expected[2],
        (unsigned int)g_app_control_drv_startup_expected[3],
        (unsigned int)g_app_control_drv_startup_expected[4],
        (unsigned int)g_app_control_drv_startup_expected[5]);
    AppFault_Send(buf);
    AppFault_Send("FAULT,END\r\n");
#else
    (void)mode;
    (void)cell;
    (void)error;
#endif
}
