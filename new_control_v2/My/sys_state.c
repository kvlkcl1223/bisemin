/* sys_state.c */
#include "sys_state.h"
#include "cmsis_os.h"

// 不用 extern！使用 static 让它变成当前文件的私有变量，别的任务无法直接摸到它
SystemState_t g_SysState = {25.0f, 25.0f, 0};

extern osMutexId_t SysStateMutexHandle;

// 封装一个“设置目标温度”的函数
void System_SetTargetTemp(float temp)
{
    if (osMutexAcquire(SysStateMutexHandle, 10) == osOK)
    {
        g_SysState.target_temp = temp;
        osMutexRelease(SysStateMutexHandle);
    }
}

// 封装一个“读取目标温度”的函数
float System_GetTargetTemp(void)
{
    float temp = 0.0f;
    if (osMutexAcquire(SysStateMutexHandle, 10) == osOK)
    {
        temp = g_SysState.target_temp;
        osMutexRelease(SysStateMutexHandle);
    }
    return temp;
}
