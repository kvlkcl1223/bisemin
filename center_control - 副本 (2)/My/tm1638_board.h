#ifndef __TM1638_BOARD_H__
#define __TM1638_BOARD_H__

#include <stdint.h>
#include <stdbool.h>

/* 按键枚举，对应板子上的5个按键 */
typedef enum
{
    KEY_MODE = 0,
    KEY_START_STOP,
    KEY_UP,
    KEY_DOWN,
    KEY_ENTER,
    KEY_MAX_NUM
} TM1638_Key_t;

/* 按键回调函数类型 */
typedef void (*TM1638_KeyCallback_t)(void);

/* 硬件接口抽象层结构体 - 用于移植 */
typedef struct
{
    void (*SetSTB)(bool state);   // 设置STB引脚电平 (true=高, false=低)
    void (*SetCLK)(bool state);   // 设置CLK引脚电平
    void (*SetDIO)(bool state);   // 设置DIO引脚电平
    bool (*GetDIO)(void);         // 读取DIO引脚电平 (读取前需要确保DIO开漏输出高电平或切换为输入)
    void (*DelayUs)(uint32_t us); // 微秒级延时函数 (TM1638要求很低，通常1-2us即可)
} TM1638_HW_t;

/* TM1638 设备对象结构体 */
typedef struct
{
    TM1638_HW_t hw;                                  // 底层硬件接口
    uint8_t brightness;                              // 亮度 (0-7)
    bool display_on;                                 // 显示开关
    uint8_t key_states[KEY_MAX_NUM];                 // 记录按键当前状态(用于消抖和边缘检测)
    TM1638_KeyCallback_t key_callbacks[KEY_MAX_NUM]; // 按键按下时的回调函数
} TM1638_HandleTypeDef;

/* 核心初始化与控制API */
void TM1638_Init(TM1638_HandleTypeDef *htmp, TM1638_HW_t *hw_interface);
void TM1638_SetBrightness(TM1638_HandleTypeDef *htmp, uint8_t brightness, bool on);

/* 数码管显示API */
void TM1638_ShowDigit(TM1638_HandleTypeDef *htmp, uint8_t position, uint8_t num, bool show_dp);
void TM1638_ShowChar(TM1638_HandleTypeDef *htmp, uint8_t position, char c, bool show_dp);
void TM1638_ClearDisplay(TM1638_HandleTypeDef *htmp);
void TM1638_ShowFloat(TM1638_HandleTypeDef *htmp, float num, uint8_t decimal_places);
/* 独立LED控制API */
void TM1638_SetLED(TM1638_HandleTypeDef *htmp, uint8_t led_index, bool state); // led_index: 1~7
void TM1638_SetAllLEDs(TM1638_HandleTypeDef *htmp, uint8_t led_mask);

/* 按键扫描与回调API */
void TM1638_RegisterKeyCallback(TM1638_HandleTypeDef *htmp, TM1638_Key_t key, TM1638_KeyCallback_t callback);
void TM1638_ProcessKeys(TM1638_HandleTypeDef *htmp); // 需放在主循环或定时器中定期调用，建议10-20ms调用一次


void TM1638_WriteData(TM1638_HandleTypeDef *htmp, uint8_t addr, uint8_t data);
#endif // __TM1638_BOARD_H__