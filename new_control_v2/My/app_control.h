#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include "main.h"
#include "drv8703.h"
#include "temp_panel.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define APP_CONTROL_CELL_COUNT 2U
#define APP_CONTROL_DRV_COUNT 5U
#define APP_CONTROL_TEMP_INPUT_COUNT 4U
#define APP_CONTROL_CLOSED_LOOP_COUNT 4U

/** @brief Maximum absolute duty cycle used for PID output clamping */
#define APP_CONTROL_MAX_ABS_DUTY 0.35f

/** @brief Fixed shared channel 5 duty cycle when a cell is running */
#define APP_CONTROL_SHARED_CH5_DUTY 0.20f

    typedef enum
    {
        APP_CONTROL_OK = 0,
        APP_CONTROL_ERROR_QUEUE,
        APP_CONTROL_ERROR_PARAM
    } AppControl_Status_t;

    typedef enum
    {
        APP_CONTROL_CMD_START = 0,
        APP_CONTROL_CMD_STOP
    } AppControlCommandType_t;

    typedef struct
    {
        AppControlCommandType_t type;
        uint8_t cell;
    } AppControlCommand_t;

    extern volatile uint8_t g_app_control_simulate_drv8703;
    extern volatile uint8_t g_app_control_simulate_voltage_ok;
    extern volatile AppControl_Status_t g_app_control_init_result;
    extern volatile uint32_t g_app_control_loop_count;
    extern volatile uint32_t g_app_control_cmd_drop_count;
    extern volatile uint8_t g_app_control_cell_running[APP_CONTROL_CELL_COUNT];
    extern volatile PanelError_t g_app_control_cell_error[APP_CONTROL_CELL_COUNT];
    extern volatile float g_app_control_cell_temp[APP_CONTROL_CELL_COUNT];
    extern volatile float g_app_control_cell_target[APP_CONTROL_CELL_COUNT];
    extern volatile float g_app_control_cell_duty[APP_CONTROL_CELL_COUNT];
    extern volatile float g_app_control_pid_temp[APP_CONTROL_CLOSED_LOOP_COUNT];
    extern volatile float g_app_control_pid_duty[APP_CONTROL_CLOSED_LOOP_COUNT];
    extern volatile uint8_t g_app_control_pid_update_pending[APP_CONTROL_CLOSED_LOOP_COUNT];
    extern volatile uint8_t g_app_control_drv_init_attempts[APP_CONTROL_DRV_COUNT];
    extern volatile uint8_t g_app_control_drv_ready[APP_CONTROL_DRV_COUNT];
    extern volatile uint8_t g_app_control_drv_awake[APP_CONTROL_DRV_COUNT];
    extern volatile uint8_t g_app_control_drv_fault[APP_CONTROL_DRV_COUNT];
    extern volatile uint8_t g_app_control_drv_startup_dump_valid[APP_CONTROL_DRV_COUNT];
    extern volatile DRV8703_Status_t g_app_control_drv_startup_dump_status[APP_CONTROL_DRV_COUNT];
    extern volatile uint8_t g_app_control_drv_startup_reg_dump[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT];
    extern volatile DRV8703_Status_t g_app_control_drv_startup_reg_status[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT];
    extern volatile uint16_t g_app_control_drv_startup_tx[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT];
    extern volatile uint16_t g_app_control_drv_startup_rx[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT];
    extern volatile uint8_t g_app_control_drv_startup_expected[DRV8703_REGISTER_COUNT];
    extern volatile uint8_t g_app_control_last_drv_fault;
    extern volatile DRV8703_Status_t g_app_control_last_drv_status;
    extern volatile uint8_t g_app_control_drv_fault_snapshot_valid[APP_CONTROL_DRV_COUNT];
    extern volatile uint32_t g_app_control_drv_fault_capture_count[APP_CONTROL_DRV_COUNT];
    extern volatile DRV8703_Status_t g_app_control_drv_fault_read_status[APP_CONTROL_DRV_COUNT];
    extern volatile DRV8703_Status_t g_app_control_drv_dump_status[APP_CONTROL_DRV_COUNT];
    extern volatile uint8_t g_app_control_drv_fault_status[APP_CONTROL_DRV_COUNT];
    extern volatile uint8_t g_app_control_drv_vds_gdf_status[APP_CONTROL_DRV_COUNT];
    extern volatile uint8_t g_app_control_drv_reg_dump[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT];
    extern volatile DRV8703_Status_t g_app_control_drv_reg_read_status[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT];
    extern volatile uint8_t g_app_control_drv_reg_read_ok_mask[APP_CONTROL_DRV_COUNT];
    extern volatile uint8_t g_app_control_drv_pin_fault_last;
    extern volatile uint32_t g_app_control_drv_pin_fault_count[APP_CONTROL_DRV_COUNT];
    extern volatile DRV8703_Status_t g_app_control_drv_pin_fault_status[APP_CONTROL_DRV_COUNT];
    extern volatile DRV8703_Status_t g_app_control_drv_pin_fault_read_status[APP_CONTROL_DRV_COUNT];
    extern volatile DRV8703_Status_t g_app_control_drv_pin_fault_dump_status[APP_CONTROL_DRV_COUNT];
    extern volatile uint8_t g_app_control_drv_pin_fault_fault_status[APP_CONTROL_DRV_COUNT];
    extern volatile uint8_t g_app_control_drv_pin_fault_vds_gdf_status[APP_CONTROL_DRV_COUNT];
    extern volatile uint8_t g_app_control_drv_pin_fault_stop_bits[APP_CONTROL_DRV_COUNT];
    extern volatile uint8_t g_app_control_drv_pin_fault_reg_dump[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT];
    extern volatile DRV8703_Status_t g_app_control_drv_pin_fault_reg_read_status[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT];
    extern volatile uint8_t g_app_control_drv_pin_fault_reg_read_ok_mask[APP_CONTROL_DRV_COUNT];
    extern volatile uint8_t g_app_control_drv_pin_fault_all_ff[APP_CONTROL_DRV_COUNT];
    extern volatile uint32_t g_app_control_drv_pin_fault_all_ff_count[APP_CONTROL_DRV_COUNT];
    extern volatile uint8_t g_app_control_drv_pin_fault_stable_count[APP_CONTROL_DRV_COUNT];
    extern volatile uint8_t g_app_control_sleep_reg_snapshot[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT + 1U];
    extern volatile uint8_t g_app_control_periodic_reg_snapshot[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT + 1U];
    extern volatile uint32_t g_app_control_periodic_reg_snapshot_count;
    extern volatile uint8_t g_app_control_temp_reset_count[APP_CONTROL_CELL_COUNT];
    extern volatile uint32_t g_app_control_temp_last_update_tick[APP_CONTROL_TEMP_INPUT_COUNT];
    extern volatile uint32_t g_app_control_temp_update_count[APP_CONTROL_TEMP_INPUT_COUNT];
    extern volatile uint8_t g_app_control_temp_fault_sensor[APP_CONTROL_CELL_COUNT];
    extern volatile uint8_t g_app_control_temp_reset_active;

    /* Temperature update rate and PID calculation rate per channel (Hz) */
    extern volatile float g_app_control_temp_freq_hz[APP_CONTROL_TEMP_INPUT_COUNT];
    extern volatile float g_app_control_pid_freq_hz[APP_CONTROL_CLOSED_LOOP_COUNT];

    /* UART DMA restart flag — set in ISR, serviced in AppControl_Task */
    extern volatile uint8_t g_uart_need_restart;

    /* ============================================================
     * Test mode: manual DRV8703 duty cycling (no closed-loop, no temp sensor)
     * Set g_app_control_test_active = 1 to enable.
     * ============================================================ */
    extern volatile uint8_t g_app_control_test_active;
    extern volatile uint8_t g_app_control_test_phase;
    extern volatile uint8_t g_app_control_test_drv_ok[APP_CONTROL_DRV_COUNT];
    extern volatile float g_app_control_test_duty[APP_CONTROL_DRV_COUNT];

/* ============================================================
 * DRV8703 raw register polling (independent of existing snapshots)
 * Polls all 5 DRV8703 chips regardless of awake/ready state.
 * Updated by AppControl_ReadAllDrvRegsRaw() inside the control loop.
 * ============================================================ */
#define APP_CONTROL_DRV_RAW_POLL_MS 200U

    extern volatile uint32_t g_app_control_drv_raw_poll_count;
    extern volatile uint8_t g_app_control_drv_raw_dump[APP_CONTROL_DRV_COUNT][DRV8703_REGISTER_COUNT];
    extern volatile uint8_t g_app_control_drv_raw_mask[APP_CONTROL_DRV_COUNT];
    extern volatile DRV8703_Status_t g_app_control_drv_raw_status[APP_CONTROL_DRV_COUNT];

    AppControl_Status_t AppControl_Init(void);
    void AppControl_Task(uint32_t now_ms);
    void AppControl_UpdatePanel(TempPanel_t *panel, uint32_t now_ms);

    DRV8703_Status_t AppControl_SetDrvDuty(uint8_t drv, float duty);

    void Control_StartPid(uint8_t cell);
    void Control_StopPid(uint8_t cell);
    void Control_SetTargetTemp(uint8_t cell, float target);

#ifdef __cplusplus
}
#endif

#endif
