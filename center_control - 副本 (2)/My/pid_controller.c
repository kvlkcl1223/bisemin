#include "pid_controller.h"

void PID_Init(PID_TypeDef *pid, pid_data_t kp, pid_data_t ki, pid_data_t kd, pid_data_t dt)
{
    if (pid == NULL)
        return;

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->dt = dt;

    // 设置默认的极值范围 (建议初始化后手动覆盖)
    pid->out_max = 100.0f;
    pid->out_min = -100.0f;
    pid->integral_max = 50.0f;
    pid->integral_min = -50.0f;

    PID_Reset(pid);
}

void PID_SetLimits(PID_TypeDef *pid, pid_data_t out_min, pid_data_t out_max, pid_data_t int_min, pid_data_t int_max)
{
    if (pid == NULL)
        return;

    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_min = int_min;
    pid->integral_max = int_max;
}

void PID_SetTarget(PID_TypeDef *pid, pid_data_t target)
{
    if (pid == NULL)
        return;
    pid->setpoint = target;
}

void PID_Reset(PID_TypeDef *pid)
{
    if (pid == NULL)
        return;

    pid->integral = 0.0f;
    pid->prev_measure = 0.0f;
    pid->setpoint = 0.0f;
}

pid_data_t PID_Compute(PID_TypeDef *pid, pid_data_t measurement)
{
    if (pid == NULL || pid->dt <= 0.0f)
        return 0.0f;

    // 1. 计算当前误差 (Error)
    pid_data_t error = pid->setpoint - measurement;

    // 2. 比例项 (Proportional)
    pid_data_t p_out = pid->Kp * error;

    // 3. 积分项 (Integral)
    pid->integral += pid->Ki * error * pid->dt;

    // --- 积分限幅 (Anti-Windup) ---
    if (pid->integral > pid->integral_max)
    {
        pid->integral = pid->integral_max;
    }
    else if (pid->integral < pid->integral_min)
    {
        pid->integral = pid->integral_min;
    }
    pid_data_t i_out = pid->integral;

    // 4. 微分项 (Derivative on Measurement)
    // 使用 -Kd * (dMeasurement / dt) 代替 Kd * (dError / dt)
    // 这消除了设定值(setpoint)突变造成的微分冲击
    pid_data_t d_out = -pid->Kd * (measurement - pid->prev_measure) / pid->dt;

    // 保存本次测量值以供下次计算使用
    pid->prev_measure = measurement;

    // 5. 计算总输出
    pid_data_t output = p_out + i_out + d_out;

    // --- 最终输出限幅 ---
    if (output > pid->out_max)
    {
        output = pid->out_max;
    }
    else if (output < pid->out_min)
    {
        output = pid->out_min;
    }

    return output;
}