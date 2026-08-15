# new_control_v3 固件说明

这是 Bisemin 温控板的 STM32G4 固件工程。当前版本面向“堆叠冷热片”方案：系统保留 2 个温控池 cell，每个 cell 使用两路测温计算平均温度，并用这个平均温度做闭环控制；每个 cell 同时驱动外层和内层两片冷热片。

## 当前控制方案

系统共有 2 个 cell、5 路 DRV8703；当前临时配置关闭 DRV1，使用 DRV5 作为 Cell 0 主路，并关闭共享通道逻辑：

| Cell | 测温输入 | 控制温度 | 外层 DRV | 内层 DRV | 默认内层比例 |
|---|---|---|---|---|---|
| Cell 0 | CH1 + CH2 | `(CH1 + CH2) / 2` | DRV5 | DRV2 | `0.70` |
| Cell 1 | CH3 + CH4 | `(CH3 + CH4) / 2` | DRV3 | DRV4 | `0.70` |
| DRV1 | - | - | 禁用 | - | 不初始化/不唤醒 |

PID 输出表示主路电压，也就是外层冷热片的 PWM duty。内层冷热片不再单独做 PID，而是跟随外层：

```text
outer_duty = PID(mean_temp, target_temp) + feedforward
inner_duty = outer_duty * inner_ratio
```

默认配置里，内层 duty 是外层 duty 的 `0.70` 倍。当前 `APP_CONTROL_SHARED_DRV_ENABLE` 为 `0`，DRV5 不再作为共享通道额外输出，而是作为 Cell 0 外层/主路 DRV 使用。

## 通道映射配置

堆叠冷热片的硬件连接后续可能调整，通道关系集中放在 [My/app_control.h](My/app_control.h) 顶部宏定义中：

```c
#define APP_CONTROL_CELL0_TEMP_OUTER 0U
#define APP_CONTROL_CELL0_TEMP_INNER 1U
#define APP_CONTROL_CELL0_DRV_OUTER 4U
#define APP_CONTROL_CELL0_DRV_INNER 1U
#define APP_CONTROL_CELL0_INNER_DUTY_RATIO 0.70f

#define APP_CONTROL_CELL1_TEMP_OUTER 2U
#define APP_CONTROL_CELL1_TEMP_INNER 3U
#define APP_CONTROL_CELL1_DRV_OUTER 2U
#define APP_CONTROL_CELL1_DRV_INNER 3U
#define APP_CONTROL_CELL1_INNER_DUTY_RATIO 0.70f

#define APP_CONTROL_SHARED_DRV 4U
#define APP_CONTROL_DRV_ENABLE_MASK ((uint8_t)((1U << 1) | (1U << 2) | (1U << 3) | (1U << 4)))
#define APP_CONTROL_SHARED_DRV_ENABLE 0U
#define APP_CONTROL_SHARED_CH5_DUTY 0.20f
```

注意：这些索引都是从 0 开始的。`TEMP 0..3` 对应实际测温 CH1..CH4，`DRV 0..4` 对应实际 DRV1..DRV5。

常见改法：

- 只改内外层比例：修改 `APP_CONTROL_CELLx_INNER_DUTY_RATIO`。
- 互换某个 cell 的内外层 DRV：修改 `APP_CONTROL_CELLx_DRV_OUTER` 和 `APP_CONTROL_CELLx_DRV_INNER`。
- 调整某个 cell 使用的两路测温：修改 `APP_CONTROL_CELLx_TEMP_OUTER` 和 `APP_CONTROL_CELLx_TEMP_INNER`。
- 关闭/打开某路 DRV：修改 `APP_CONTROL_DRV_ENABLE_MASK`，bit0..bit4 对应 DRV1..DRV5。
- 打开共享通道逻辑：将 `APP_CONTROL_SHARED_DRV_ENABLE` 改为 `1U`，再按需要修改 `APP_CONTROL_SHARED_CH5_DUTY`。

## 主要代码结构

| 路径 | 作用 |
|---|---|
| [Core/Src/main.c](Core/Src/main.c) | HAL 入口、外设初始化、按键中断和温度串口接收 |
| [Core/Src/app_freertos.c](Core/Src/app_freertos.c) | FreeRTOS 任务入口，启动 ADC、控制循环、面板循环 |
| [My/app_control.c](My/app_control.c) | 温控核心：PID、前馈、DRV 启停、故障处理、cell 平均温度控制 |
| [My/app_control.h](My/app_control.h) | cell/DRV/测温映射和控制参数宏 |
| [My/temp_panel.c](My/temp_panel.c) | TM1638 面板、按键、普通模式和程序升温流程 |
| [My/calib_mode.c](My/calib_mode.c) | 自动标定流程，扫描 duty 并写入 Flash |
| [My/drv8703.c](My/drv8703.c) | DRV8703 SPI 驱动 |
| [My/drv8703_board.c](My/drv8703_board.c) | 5 路 DRV8703 的片选、睡眠脚、PWM 映射 |
| [My/adc_measure.c](My/adc_measure.c) | 电压、电流 ADC 采样和换算 |
| [My/pc_protocol.c](My/pc_protocol.c) | 上位机串口协议解析和状态上报 |
| [My/flash_storage.c](My/flash_storage.c) | 内部 Flash 参数存储 |
| [My/ads1220.c](My/ads1220.c) | ADS1220/PT1000 初始化和测试支持 |

## 标定与前馈

标定数据用于生成前馈 duty。当前标定逻辑在 [My/calib_mode.h](My/calib_mode.h) 和 [My/calib_mode.c](My/calib_mode.c)：

- duty 从 `+0.30` 扫到 `-0.40`。
- 步长 `-0.02`，共 `36` 步。
- 每步等待温度稳定，稳定条件是温度窗口波动小于 `0.1 degC` 并持续 `10 s`。
- 单步最长等待 `600 s`，超时会记录当前值并标记为未稳定。
- 每个 cell 记录两路测温和均值；闭环前馈使用均值曲线。
- 标定时外层输出当前扫描 duty，内层按配置比例跟随；当前共享通道逻辑关闭，不会额外输出共享 duty。
- 当前虚拟低温显示已关闭，面板和上位机使用真实温度下限 `-10.0 degC`。

Flash 存储布局见 [FLASH_LAYOUT.md](FLASH_LAYOUT.md)。当前代码使用内部 Flash 末尾 8 KB：

| 数据 | 偏移 | 地址 |
|---|---:|---|
| Cell 0 标定数据 | `0` | `0x0803E000` |
| Cell 1 标定数据 | `4096` | `0x0803F000` |

## 上位机协议

上位机协议见 [PC_COMM_PROTOCOL.md](PC_COMM_PROTOCOL.md)。协议采用二进制帧头/CRC 加 ASCII payload 的格式。当前 `STATE`/`DATA` 上报中，和堆叠控制相关的字段包括：

```text
duty        外层/主路 duty，保留旧字段名便于兼容
duty_outer  外层/主路 duty
duty_inner  内层/跟随 duty
ratio       内层跟随比例
t0/t1       当前 cell 的两路测温
temp        两路测温均值，也是 PID 使用的反馈温度
aux_temp    ADS1220/PT1000 定时测得的环境/水温
aux_valid   aux_temp 是否有效，1=有效，0=无效或暂未就绪
```

上位机仍然只发送 cell 级目标温度和启动/停止意图，不直接控制单个 DRV 的 PWM。

当前协议已支持从上位机发起和读取标定：

```text
op=START_CALIB,cell=0
op=STOP_CALIB
op=GET_CALIB_STATUS
op=GET_CALIB_RESULT,cell=0
op=GET_CALIB_RESULT,cell=0,index=0
```

读取结果时先读取 `CALIB_META` 获取 `count=36`，再逐条读取 `CALIB_STEP`，避免一次 payload 超过 256 字节。

## 运行流程

`ControlTask` 启动后会依次执行：

1. 启动 PWM 时间基准和 ADC 测量。
2. 初始化控制模块 `AppControl_Init()`。
3. 初始化 ADS1220。
4. 从 Flash 加载标定数据并打印标定表。
5. 循环处理 UART 重启、ADC 数据、温控任务和上位机协议。

`HMITask` 负责 TM1638 面板显示、按键处理、标定界面显示和面板发起的控制请求。

## 故障处理摘要

- 某个 cell 的任一路测温超时，会尝试复位测温串口；多次失败后进入温度错误。
- 外层或内层 DRV8703 故障会停止对应 cell。
- 当前共享逻辑关闭，DRV5 故障按 Cell 0 外层/主路 DRV 故障处理。
- 电压/电流采样由 `adc_measure` 维护，相关错误码在 `temp_panel.h` 中定义。

## 构建方式

- Keil MDK-ARM 工程：`MDK-ARM/bisemin.uvprojx`
- CubeMX 配置：`bisemin.ioc`
- 目标平台：STM32G474 系列，当前工程文件使用 STM32G474VCTx 目标
- 生成代码目录：`Core/`、`Drivers/`、`Middlewares/`
- 业务代码目录：`My/`

当前环境没有检测到可用的 ARM 编译器，因此本次只做源码级修改和检查，没有在本机完成固件编译。

## 开发注意事项

- 修改 cell 与硬件通道关系时，优先改 `My/app_control.h` 的映射宏，不要在控制逻辑里写死 `cell * 2` 或固定 DRV 编号。
- PID 只针对 cell 均温运行；不要再为内层 DRV 增加独立 PID，除非控制策略整体改版。
- 新增上位机字段时注意 `PC_PAYLOAD_MAX` 限制。
- 工程里部分历史注释存在编码不一致的问题，批量改编码前需要确认 Keil 和上位机工具链的读取方式。
