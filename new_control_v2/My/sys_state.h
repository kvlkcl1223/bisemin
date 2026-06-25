/* sys_state.h */
#ifndef __SYS_STATE_H
#define __SYS_STATE_H

#include <stdint.h>

// 1. 定义系统状态结构体类型
typedef struct
{
    float current_temp; // 当前实际温度 (串口接收更新)
    float target_temp;  // 目标设定温度 (按键修改)
    uint8_t work_mode;  // 工作模式 (0:停止, 1:加热)
} SystemState_t;

// 2. 使用 extern 声明全局变量
extern SystemState_t g_SysState;
// 只能通过下面这些函数来操作

void System_SetTargetTemp(float temp);
float System_GetTargetTemp(void);

#endif
