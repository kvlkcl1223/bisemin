#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stddef.h>

    /*
     * 定义数据类型，方便以后根据平台算力改成 double 或 int32_t (定点数)
     */
    typedef float pid_data_t;

    /* PID 控制器结构体 */
    typedef struct
    {
        // 调参参数
        pid_data_t Kp; // 比例系数
        pid_data_t Ki; // 积分系数
        pid_data_t Kd; // 微分系数

        // 状态变量
        pid_data_t setpoint;     // 目标设定值
        pid_data_t integral;     // 积分累计项
        pid_data_t prev_measure; // 上一次的测量值 (用于消除微分冲击)

        // 限制参数
        pid_data_t out_max;      // 最终输出上限
        pid_data_t out_min;      // 最终输出下限
        pid_data_t integral_max; // 积分项上限 (抗饱和)
        pid_data_t integral_min; // 积分项下限

        // 运行参数
        pid_data_t dt; // 采样周期 (单位：秒)
    } PID_TypeDef;

    /* 函数声明 */

    // 初始化 PID 参数
    void PID_Init(PID_TypeDef *pid, pid_data_t kp, pid_data_t ki, pid_data_t kd, pid_data_t dt);

    // 设置输出和积分的限幅 (非常重要，防止 PWM 溢出和积分死区)
    void PID_SetLimits(PID_TypeDef *pid, pid_data_t out_min, pid_data_t out_max, pid_data_t int_min, pid_data_t int_max);

    // 设置目标值
    void PID_SetTarget(PID_TypeDef *pid, pid_data_t target);

    // 核心计算函数，建议在定时器中断或固定频率的任务中调用
    pid_data_t PID_Compute(PID_TypeDef *pid, pid_data_t measurement);

    // 重置内部状态 (当电机停止或系统复位时调用)
    void PID_Reset(PID_TypeDef *pid);

#ifdef __cplusplus
}
#endif

#endif // PID_CONTROLLER_H
