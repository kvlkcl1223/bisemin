#ifndef APP_FAULT_LOG_H
#define APP_FAULT_LOG_H

#include <stdint.h>

/* Control-task only, after outputs have been stopped. Uses stored snapshots;
 * never reads SPI, clears faults, changes PWM, or acquires the control mutex. */
void AppFault_LogCell(const char *mode, uint8_t cell, uint32_t error);

#endif
