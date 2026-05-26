#include "tm1638_board.h"
#include "main.h"

/* 消抖阈值: 连续读到相同值达此次数才确认状态变化 (3次 × 20ms = 60ms) */
#define TM1638_DEBOUNCE_CNT 3

/* ========================================================= */
/* 全局 TM1638 设备句柄                                      */
/* ========================================================= */
TM1638_HandleTypeDef htm1638;

/* TM1638 指令宏定义 */
#define CMD_DATA_AUTO 0x40  // 地址自动加1模式
#define CMD_DATA_READ 0x42  // 读按键数据
#define CMD_DATA_FIXED 0x44 // 固定地址模式
#define CMD_DISP_CTRL 0x80  // 显示控制指令基址
#define CMD_ADDR_BASE 0xC0  // 地址指令基址

/* 共阴极数码管 0-9 字模 */
static const uint8_t FONT_DIGIT[] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F // 0-9
};

/*
 * 扩展字母字模 (仅包含7段数码管可辨识的字母)
 * 索引: A=0, b=1, C=2, c=3, d=4, E=5, F=6, G=7, H=8, h=9,
 *       I=10, J=11, L=12, n=13, O=14, o=15, P=16, r=17,
 *       S=18, t=19, U=20, u=21, y=22, -=23, 空格=24
 *
 * 用法: TM1638_CharToSeg(letter) → 返回段码或 0x00(不可显示)
 */
static const uint8_t FONT_LETTER[] = {
    0x77, 0x7C, 0x39, 0x58, 0x5E, 0x79, 0x71, 0x7D, 0x76, 0x74,
    0x30, 0x1E, 0x38, 0x54, 0x3F, 0x5C, 0x73, 0x50,
    0x6D, 0x78, 0x3E, 0x1C, 0x6E, 0x40, 0x00};

static uint8_t TM1638_CharToSeg(char c)
{
    if (c >= '0' && c <= '9')
        return FONT_DIGIT[c - '0'];
    c = (c >= 'a' && c <= 'z') ? (c - 'a' + 'A') : c;
    switch (c)
    {
    case 'A':
        return FONT_LETTER[0];
    case 'B':
        return FONT_LETTER[1];
    case 'C':
        return FONT_LETTER[2];
    case 'D':
        return FONT_LETTER[4];
    case 'E':
        return FONT_LETTER[5];
    case 'F':
        return FONT_LETTER[6];
    case 'G':
        return FONT_LETTER[7];
    case 'H':
        return FONT_LETTER[8];
    case 'I':
        return FONT_LETTER[10];
    case 'J':
        return FONT_LETTER[11];
    case 'L':
        return FONT_LETTER[12];
    case 'N':
        return FONT_LETTER[13];
    case 'O':
        return FONT_LETTER[14];
    case 'P':
        return FONT_LETTER[16];
    case 'R':
        return FONT_LETTER[17];
    case 'S':
        return FONT_LETTER[18];
    case 'T':
        return FONT_LETTER[19];
    case 'U':
        return FONT_LETTER[20];
    case 'Y':
        return FONT_LETTER[22];
    case '-':
        return FONT_LETTER[23];
    case ' ':
        return FONT_LETTER[24];
    default:
        return 0x00;
    }
}

/* 内部底层通讯函数 */
static void TM1638_SendByte(TM1638_HandleTypeDef *htmp, uint8_t data)
{
    /*
     * TM1638 低位先发。
     * DIO 为开漏输出：SetDIO(true) 表示释放总线，由外部上拉拉高。
     */
    for (int i = 0; i < 8; i++)
    {
        htmp->hw.SetCLK(false);
        htmp->hw.SetDIO((data & 0x01) != 0);
        data >>= 1;

        htmp->hw.DelayUs(10);
        htmp->hw.SetCLK(true);
        htmp->hw.DelayUs(10);
    }

    /*
     * 每发送完 1 字节，都强制恢复到安全空闲态：
     * CLK=LOW，DIO=释放。
     * 这样可以降低读键/写显示切换时 DIO 被 MCU 长时间拉低的风险。
     */
    htmp->hw.SetCLK(false);
    htmp->hw.SetDIO(true);
    htmp->hw.DelayUs(5);
}

static uint8_t TM1638_ReadByte(TM1638_HandleTypeDef *htmp)
{
    uint8_t data = 0;

    /*
     * 读按键前必须释放 DIO。
     * 开漏模式下 SetDIO(true) 不是主动输出高电平，而是 Hi-Z。
     */
    htmp->hw.SetDIO(true);
    htmp->hw.DelayUs(20);

    for (int i = 0; i < 8; i++)
    {
        data >>= 1;

        htmp->hw.SetCLK(false);
        htmp->hw.DelayUs(10);

        htmp->hw.SetCLK(true);
        htmp->hw.DelayUs(5);

        if (htmp->hw.GetDIO())
        {
            data |= 0x80;
        }

        htmp->hw.DelayUs(10);
    }

    /* 读完后恢复总线空闲态 */
    htmp->hw.SetCLK(false);
    htmp->hw.SetDIO(true);
    htmp->hw.DelayUs(5);

    return data;
}

static void TM1638_WriteCommand(TM1638_HandleTypeDef *htmp, uint8_t cmd)
{
    /*
     * 每次发送命令前，先把总线恢复到确定状态。
     * 这样可以避免上一帧异常结束后，STB/CLK/DIO 处于不确定状态。
     */
    htmp->hw.SetSTB(true);
    htmp->hw.SetCLK(false);
    htmp->hw.SetDIO(true);
    htmp->hw.DelayUs(5);

    htmp->hw.SetSTB(false);
    htmp->hw.DelayUs(10);

    TM1638_SendByte(htmp, cmd);

    /*
     * STB 上升沿结束本帧。
     * 保证 STB 上升时 CLK=LOW，DIO 已释放。
     */
    htmp->hw.SetCLK(false);
    htmp->hw.SetDIO(true);
    htmp->hw.SetSTB(true);
    htmp->hw.DelayUs(10);
}

void TM1638_WriteData(TM1638_HandleTypeDef *htmp, uint8_t addr, uint8_t data)
{
    TM1638_WriteCommand(htmp, CMD_DATA_FIXED); // 固定地址模式

    htmp->hw.SetSTB(true);
    htmp->hw.SetCLK(false);
    htmp->hw.SetDIO(true);
    htmp->hw.DelayUs(5);

    htmp->hw.SetSTB(false);
    htmp->hw.DelayUs(10);

    TM1638_SendByte(htmp, CMD_ADDR_BASE | (addr & 0x0F));
    TM1638_SendByte(htmp, data);

    htmp->hw.SetCLK(false);
    htmp->hw.SetDIO(true);
    htmp->hw.SetSTB(true);
    htmp->hw.DelayUs(10);
}

/* 刷新显示控制寄存器 (控制亮度与开关) */
static void TM1638_UpdateDisplayCtrl(TM1638_HandleTypeDef *htmp)
{
    uint8_t cmd = CMD_DISP_CTRL;
    if (htmp->display_on)
    {
        cmd |= 0x08;                      // 打开显示
        cmd |= (htmp->brightness & 0x07); // 加上亮度值
    }
    TM1638_WriteCommand(htmp, cmd);
}
static void TM1638_BusRecover(TM1638_HandleTypeDef *htmp)
{
    /*
     * TM1638 没有独立复位脚。
     * 这里通过释放 STB/DIO、拉低 CLK，并在 STB=HIGH 时给空时钟，
     * 尽量把可能卡在半帧/半字节状态的串行接口恢复到空闲状态。
     */
    htmp->hw.SetSTB(true);
    htmp->hw.SetCLK(false);
    htmp->hw.SetDIO(true);
    htmp->hw.DelayUs(100);

    for (int i = 0; i < 16; i++)
    {
        htmp->hw.SetCLK(true);
        htmp->hw.DelayUs(5);
        htmp->hw.SetCLK(false);
        htmp->hw.DelayUs(5);
    }

    htmp->hw.SetSTB(true);
    htmp->hw.SetCLK(false);
    htmp->hw.SetDIO(true);
    htmp->hw.DelayUs(100);
}
/* ========================================================= */
/* 开放API实现 */

void TM1638_Init(TM1638_HandleTypeDef *htmp, TM1638_HW_t *hw_interface)
{
    htmp->hw = *hw_interface;
    htmp->brightness = 5;
    htmp->display_on = true;

    for (int i = 0; i < KEY_MAX_NUM; i++)
    {
        htmp->key_states[i] = 0;
        htmp->key_callbacks[i] = 0;
        htmp->debounce_cnt[i] = 0;
    }

    /*
     * 关键修改：
     * 1. 初始化最前面先做总线恢复；
     * 2. 等待 TM1638 5V 供电和内部 POR 稳定；
     * 3. 不先发送 0x80 关闭显示，避免后续开显示命令丢失时一直黑屏；
     * 4. 连续发送显示开启命令，提高上电成功率。
     */
    TM1638_BusRecover(htmp);
    HAL_Delay(100);

    TM1638_ClearDisplay(htmp);
    HAL_Delay(2);

    TM1638_UpdateDisplayCtrl(htmp);
    HAL_Delay(2);
    TM1638_UpdateDisplayCtrl(htmp);
    HAL_Delay(2);
    TM1638_UpdateDisplayCtrl(htmp);

    TM1638_BusRecover(htmp);
}

void TM1638_SetBrightness(TM1638_HandleTypeDef *htmp, uint8_t brightness, bool on)
{
    htmp->brightness = brightness;
    htmp->display_on = on;
    TM1638_UpdateDisplayCtrl(htmp);
}

void TM1638_ShowDigit(TM1638_HandleTypeDef *htmp, uint8_t position, uint8_t num, bool show_dp)
{
    /*
     * FONT_DIGIT 只有 0~9。
     * 原代码允许 num=10~15，会数组越界，可能引发不可预期问题。
     */
    if (position >= 4 || num > 9)
        return;

    uint8_t addr = position * 2; // GR1->0x00, GR2->0x02, GR3->0x04, GR4->0x06
    uint8_t data = FONT_DIGIT[num];
    if (show_dp)
        data |= 0x80; // SEG8 是 DP

    TM1638_WriteData(htmp, addr, data);
}

void TM1638_ShowString(TM1638_HandleTypeDef *htmp, const char *str)
{
    TM1638_ClearDisplay(htmp);
    if (str == NULL)
        return;

    for (int i = 0; i < 4 && str[i] != '\0'; i++)
    {
        uint8_t seg = TM1638_CharToSeg(str[i]);
        TM1638_WriteData(htmp, i * 2, seg);
    }
}

void TM1638_ClearDisplay(TM1638_HandleTypeDef *htmp)
{
    TM1638_WriteCommand(htmp, CMD_DATA_AUTO);

    htmp->hw.SetSTB(true);
    htmp->hw.SetCLK(false);
    htmp->hw.SetDIO(true);
    htmp->hw.DelayUs(5);

    htmp->hw.SetSTB(false);
    htmp->hw.DelayUs(10);

    TM1638_SendByte(htmp, CMD_ADDR_BASE); // 从0地址开始
    for (int i = 0; i < 16; i++)
    {
        TM1638_SendByte(htmp, 0x00);
    }

    htmp->hw.SetCLK(false);
    htmp->hw.SetDIO(true);
    htmp->hw.SetSTB(true);
    htmp->hw.DelayUs(10);
}
/**
 * @brief 显示浮点数 (支持指定小数位数和自动高位消隐)
 * @param htmp: 控制句柄
 * @param num: 要显示的浮点数
 * @param decimal_places: 小数点保留位数 (支持 0 ~ 3)
 */
void TM1638_ShowFloat(TM1638_HandleTypeDef *htmp, float num, uint8_t decimal_places)
{
    // 4位数码管最多支持3位小数
    if (decimal_places > 3)
    {
        decimal_places = 3;
    }

    // 处理负数逻辑
    bool is_negative = false;
    if (num < 0.0f)
    {
        is_negative = true;
        num = -num;
    }

    // 1. 根据指定的小数位数，对浮点数进行放大取整 (加0.5f是为了实现四舍五入)
    uint32_t int_val = 0;
    float multiplier = 1.0f;
    for (int i = 0; i < decimal_places; i++)
    {
        multiplier *= 10.0f;
    }
    int_val = (uint32_t)(num * multiplier + 0.5f);

    // 防止溢出4位数码管的显示上限 (9999)
    if (int_val > 9999)
    {
        int_val = 9999;
    }

    // 2. 提取4个位置的数字
    uint8_t digits[4];
    digits[3] = int_val % 10;          // 个位 (最右侧)
    digits[2] = (int_val / 10) % 10;   // 十位
    digits[1] = (int_val / 100) % 10;  // 百位
    digits[0] = (int_val / 1000) % 10; // 千位 (最左侧)

    // 3. 高位消隐与显示逻辑
    bool blank = true;                 // 默认开启消隐
    int dp_index = 3 - decimal_places; // 计算小数点应该在哪个数码管上显示

    for (int i = 0; i < 4; i++)
    {
        bool show_dp = (i == dp_index);

        // 核心消隐逻辑：遇到第一个非零数字，或者到达"小数点前的个位"时，强制停止消隐
        // 这样可以确保 0.1 显示为 "0.1" 而不是 ".1"
        if (digits[i] != 0 || i == dp_index)
        {
            blank = false;
        }

        if (blank)
        {
            // 当前位需要消隐（即高位无用的0）
            // 预判下一个位置是否即将显示数字，如果是，且当前是负数，则在这里画一个负号
            bool next_will_show = (i + 1 == dp_index) || (digits[i + 1] != 0);

            if (is_negative && next_will_show)
            {
                TM1638_WriteData(htmp, i * 2, 0x40); // 0x40 是数码管的段码 'g'，即中间那一横 '-'
                is_negative = false;                 // 负号已显示完毕
            }
            else
            {
                TM1638_WriteData(htmp, i * 2, 0x00); // 写入 0x00，彻底熄灭该位数码管
            }
        }
        else
        {
            // 正常显示数字，并根据 show_dp 决定是否叠加小数点
            TM1638_ShowDigit(htmp, i, digits[i], show_dp);
        }
    }
}

// 缓存LED状态，避免每次设置一个灯时影响其他灯
static uint8_t led_cache = 0x00;

void TM1638_SetLED(TM1638_HandleTypeDef *htmp, uint8_t led_index, bool state)
{
    if (led_index < 1 || led_index > 7)
        return; // 原理图为 LED1 ~ LED7

    if (state)
    {
        led_cache |= (1 << (led_index - 1));
    }
    else
    {
        led_cache &= ~(1 << (led_index - 1));
    }
    // LED 挂载在 GR5，对应地址 0x08
    TM1638_WriteData(htmp, 0x08, led_cache);
}

void TM1638_SetAllLEDs(TM1638_HandleTypeDef *htmp, uint8_t led_mask)
{
    led_cache = led_mask;
    TM1638_WriteData(htmp, 0x08, led_cache);
}

void TM1638_RegisterKeyCallback(TM1638_HandleTypeDef *htmp, TM1638_Key_t key, TM1638_KeyCallback_t callback)
{
    if (key < KEY_MAX_NUM)
    {
        htmp->key_callbacks[key] = callback;
    }
}

/* 核心函数：读取按键并触发回调 (含消抖) */
void TM1638_ProcessKeys(TM1638_HandleTypeDef *htmp)
{
    uint8_t key_bytes[4];

    /*
     * 注意：本函数不要放在中断服务函数里直接调用。
     * 若需要定时扫描，建议中断只置标志位，在主循环/任务中调用本函数。
     */

    /* 确保 STB↓ 时 CLK=LOW，DIO 已释放 */
    htmp->hw.SetSTB(true);
    htmp->hw.SetCLK(false);
    htmp->hw.SetDIO(true);
    htmp->hw.DelayUs(5);

    htmp->hw.SetSTB(false);
    htmp->hw.DelayUs(10);

    TM1638_SendByte(htmp, CMD_DATA_READ);

    for (int i = 0; i < 4; i++)
    {
        key_bytes[i] = TM1638_ReadByte(htmp);
    }

    htmp->hw.SetCLK(false);
    htmp->hw.SetDIO(true);
    htmp->hw.SetSTB(true);
    htmp->hw.DelayUs(10);

    bool raw[KEY_MAX_NUM];
    raw[KEY_MODE] = (key_bytes[0] & 0x04) != 0;
    raw[KEY_START_STOP] = (key_bytes[0] & 0x40) != 0;
    raw[KEY_UP] = (key_bytes[1] & 0x04) != 0;
    raw[KEY_DOWN] = (key_bytes[1] & 0x40) != 0;
    raw[KEY_ENTER] = (key_bytes[2] & 0x04) != 0;

    /* 消抖 + 边沿检测 */
    for (int i = 0; i < KEY_MAX_NUM; i++)
    {
        if (raw[i] == htmp->key_states[i])
        {
            /* 读数与当前稳定状态相同 → 清空消抖计数器 */
            htmp->debounce_cnt[i] = 0;
        }
        else
        {
            /* 读数与当前稳定状态不同 → 累加消抖计数器 */
            htmp->debounce_cnt[i]++;
            if (htmp->debounce_cnt[i] >= TM1638_DEBOUNCE_CNT)
            {
                /* 连续 N 次读到相反值, 确认状态翻转 */
                htmp->debounce_cnt[i] = 0;
                htmp->key_states[i] = raw[i];

                if (raw[i])
                {
                    /* 上升沿: 按键按下 */
                    if (htmp->key_callbacks[i] != 0)
                    {
                        htmp->key_callbacks[i]();
                    }
                }
                /* 下降沿 (按键松开) 不需要触发回调 */
            }
        }
    }
}

/* ========================================================= */
/* 硬件抽象层 (HW Layer)                                     */
/* 将底层的 GPIO 操作封装在此, 使得上层库不直接依赖 BSP     */
/* ========================================================= */

static void HW_SetSTB(bool state)
{
    HAL_GPIO_WritePin(STB_GPIO_Port, STB_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void HW_SetCLK(bool state)
{
    HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void HW_SetDIO(bool state)
{
    HAL_GPIO_WritePin(DIO_GPIO_Port, DIO_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static bool HW_GetDIO(void)
{
    return HAL_GPIO_ReadPin(DIO_GPIO_Port, DIO_Pin) == GPIO_PIN_SET;
}

static void HW_DelayUs(uint32_t us)
{
    /*
     * 简单空循环微秒延时。
     * 注意：若 SystemCoreClock 未更新或编译优化等级变化，实际延时会有偏差。
     * 若仍有不稳定，建议换成 DWT->CYCCNT 延时。
     */
    if (us == 0)
        return;

    uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0)
        cycles_per_us = 1;

    uint32_t delay = us * (cycles_per_us / 4U);
    if (delay == 0)
        delay = us;

    while (delay--)
    {
        __NOP();
    }
}

/* ========================================================= */
/* 一键硬件初始化                                             */
/* 在 main() 或 FreeRTOS 启动前调用一次即可                  */
/* ========================================================= */
void TM1638_HW_Init(void)
{
    TM1638_HW_t hw = {HW_SetSTB, HW_SetCLK, HW_SetDIO, HW_GetDIO, HW_DelayUs};

    /*
     * 上电后先等一段时间，避免 STM32 已经开始发命令，
     * 但 TM1638 的 5V 或内部上电复位还没稳定。
     */
    HAL_Delay(100);

    TM1638_Init(&htm1638, &hw);

    /*
     * 再重复一次显示控制命令。
     * 由于 TM1638 无 ACK，这种冗余发送能提高上电可靠性。
     */
    TM1638_SetBrightness(&htm1638, 5, true);
    HAL_Delay(2);
    TM1638_SetBrightness(&htm1638, 5, true);
}
