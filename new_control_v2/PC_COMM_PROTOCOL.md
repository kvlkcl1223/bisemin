# PC 通信协议设计

## 1. 设计目标

本文档定义上位机 Qt 软件与 MCU 之间的串口通信协议。

设计目标：

- MCU 是唯一控制真相源，负责 PID、程序升温、故障保护、DRV 输出和温度采集。
- 面板和 Qt 软件地位平等，都是人机入口。
- Qt 不直接控制 PWM，不计算 PID，只发送用户意图。
- MCU 接收命令后修改内部状态，并通过状态帧广播给 Qt。
- Qt 根据 MCU 返回的数据更新界面，保证软件显示与面板状态一致。
- Qt 负责过程数据记录、曲线显示和 Excel/CSV 导出。

## 2. 基本架构

推荐状态模型：

```text
control_mode  = STOP / NORMAL / PROGRAM
control_owner = NONE / PANEL / PC
```

含义：

- `STOP`：停止控温。
- `NORMAL`：普通模式，也就是目标温度跳转/斜坡控温。
- `PROGRAM`：程序升温模式，即多参数程序控温。
- `PANEL`：最近一次控制来源为面板。
- `PC`：最近一次控制来源为上位机。
- `NONE`：无控制来源。

不建议把“外部模式”做成独立温控算法。建议把外部模式理解为：

```text
control_owner = PC
```

这样普通模式和程序升温模式都可以由面板或 Qt 启动，控制逻辑仍然只有一套。

## 3. 串口参数

建议参数：

| 项目 | 建议值 |
|---|---|
| 波特率 | 115200 或 921600 |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验 | None |
| 流控 | None |
| 编码 | ASCII |
| 帧结束 | `\r\n` |

若需要实时曲线和大量日志，建议使用 `921600`。

## 4. 帧格式

本协议采用二进制外壳 + ASCII 载荷的混合帧格式：

- 二进制外壳负责边界、长度、类型、序号、CRC。
- ASCII 载荷负责可读参数，方便串口调试和 Qt 解析。

### 4.1 总体格式

```text
SOF       LEN       TYPE      SEQ       PAYLOAD      CRC16     EOF
2 bytes   2 bytes   1 byte    2 bytes   N bytes      2 bytes   2 bytes
```

字节布局：

| 偏移 | 长度 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 2 | `SOF` | 帧头，固定 `0xA5 0x5A` |
| 2 | 2 | `LEN` | 从 `TYPE` 到 `PAYLOAD` 结束的长度，低字节在前 |
| 4 | 1 | `TYPE` | 帧类型 |
| 5 | 2 | `SEQ` | 序号，低字节在前 |
| 7 | N | `PAYLOAD` | ASCII 载荷，格式为 `key=value,key=value` |
| 7+N | 2 | `CRC16` | CRC16-CCITT，低字节在前 |
| 9+N | 2 | `EOF` | 帧尾，固定 `0x0D 0x0A` |

其中：

```text
LEN = 1(TYPE) + 2(SEQ) + N(PAYLOAD)
```

最小空载荷帧长度为 11 字节。

### 4.2 字节序

多字节整数统一使用小端序：

```text
uint16 low byte first
uint32 low byte first
```

浮点参数不使用二进制 float 传输，统一放在 ASCII `PAYLOAD` 中，例如：

```text
temp=37.5,duty=0.123
```

### 4.3 帧头与帧尾

帧头：

```text
SOF0 = 0xA5
SOF1 = 0x5A
```

帧尾：

```text
EOF0 = 0x0D
EOF1 = 0x0A
```

接收方状态机建议：

1. 搜索 `0xA5 0x5A`。
2. 读取 `LEN`。
3. 若 `LEN` 超过最大载荷限制，丢弃并重新找帧头。
4. 读取 `TYPE + SEQ + PAYLOAD + CRC16 + EOF`。
5. 校验 EOF 是否为 `0x0D 0x0A`。
6. 校验 CRC。
7. 解析 `PAYLOAD`。

### 4.4 最大长度

建议限制：

```text
PAYLOAD_MAX = 256 bytes
LEN_MAX     = 1 + 2 + PAYLOAD_MAX = 259 bytes
FRAME_MAX   = 2 + 2 + LEN_MAX + 2 + 2 = 267 bytes
```

若后续需要批量传输历史数据，可单独定义大包帧类型，不建议复用控制帧。

### 4.5 TYPE 定义

| TYPE 值 | 名称 | 方向 | 说明 |
|---:|---|---|---|
| `0x01` | `CMD` | PC -> MCU | 上位机命令 |
| `0x02` | `ACK` | MCU -> PC | 命令成功确认 |
| `0x03` | `NACK` | MCU -> PC | 命令失败 |
| `0x04` | `STATE` | MCU -> PC | 当前状态快照 |
| `0x05` | `DATA` | MCU -> PC | 过程数据记录帧 |
| `0x06` | `EVENT` | MCU -> PC | 事件通知 |
| `0x07` | `HELLO` | 双向 | 握手 |
| `0x08` | `HEARTBEAT` | 双向 | 心跳 |

### 4.6 PAYLOAD 格式

载荷为 ASCII 字符串，不包含结尾 `\0`：

```text
key=value,key=value,key=value
```

示例：

```text
op=START_NORMAL,cell=0,temp=37.5
ok=1
t=123456,cell=0,mode=NORMAL,owner=PC,running=1,target=37.5,current=25.8,duty=0.123,error=0
```

字段规则：

- 字段采用 `key=value`。
- 字段之间用英文逗号 `,` 分隔。
- 字段名建议小写。
- 枚举值使用大写字符串，如 `NORMAL`、`PROGRAM`、`PC`。
- 未知字段必须忽略，便于后续扩展。
- 必需字段缺失时返回 `NACK`。
- 字符串字段不允许包含 `,`、`=`、`\r`、`\n`。

### 4.7 CRC 计算范围

CRC 计算范围：

```text
LEN低字节, LEN高字节, TYPE, SEQ低字节, SEQ高字节, PAYLOAD...
```

不包含：

- `SOF`
- `CRC16` 本身
- `EOF`

也就是说，CRC 覆盖长度字段和有效内容，可以发现长度被破坏的问题。

### 4.8 CRC 算法

推荐使用 CRC16-CCITT：

| 项目 | 值 |
|---|---|
| 多项式 | `0x1021` |
| 初始值 | `0xFFFF` |
| 输入反转 | 否 |
| 输出反转 | 否 |
| 结果异或 | `0x0000` |

### 4.9 编码示例

命令：

```text
TYPE    = CMD = 0x01
SEQ     = 12
PAYLOAD = op=SET_TARGET,cell=0,temp=37.5
```

载荷长度：

```text
PAYLOAD_LEN = 31
LEN = 1 + 2 + 31 = 34 = 0x0022
```

发送帧：

```text
A5 5A 22 00 01 0C 00 6F 70 3D 53 45 54 5F 54 41
52 47 45 54 2C 63 65 6C 6C 3D 30 2C 74 65 6D 70
3D 33 37 2E 35 CRC_L CRC_H 0D 0A
```

其中 ASCII 部分为：

```text
op=SET_TARGET,cell=0,temp=37.5
```

### 4.10 推荐 C 结构定义

注意：由于 `PAYLOAD` 是变长字段，不建议直接把整个帧映射为 C 结构体读写。推荐只定义头部：

```c
#define PC_PROTO_SOF0 0xA5U
#define PC_PROTO_SOF1 0x5AU
#define PC_PROTO_EOF0 0x0DU
#define PC_PROTO_EOF1 0x0AU
#define PC_PROTO_PAYLOAD_MAX 256U

typedef enum
{
    PC_FRAME_CMD       = 0x01U,
    PC_FRAME_ACK       = 0x02U,
    PC_FRAME_NACK      = 0x03U,
    PC_FRAME_STATE     = 0x04U,
    PC_FRAME_DATA      = 0x05U,
    PC_FRAME_EVENT     = 0x06U,
    PC_FRAME_HELLO     = 0x07U,
    PC_FRAME_HEARTBEAT = 0x08U
} PcFrameType_t;
```

接收缓冲建议：

```c
typedef struct
{
    uint8_t  type;
    uint16_t seq;
    uint16_t payload_len;
    char     payload[PC_PROTO_PAYLOAD_MAX + 1U];
} PcFrame_t;
```

`payload` 末尾可补 `'\0'`，便于使用字符串解析。

### 4.11 推荐 Python 编码伪代码

```python
def encode_frame(frame_type: int, seq: int, payload: str) -> bytes:
    body = bytearray()
    payload_bytes = payload.encode("ascii")
    length = 1 + 2 + len(payload_bytes)

    body += length.to_bytes(2, "little")
    body += bytes([frame_type])
    body += seq.to_bytes(2, "little")
    body += payload_bytes

    crc = crc16_ccitt(body)
    return b"\xA5\x5A" + body + crc.to_bytes(2, "little") + b"\r\n"
```

## 5. 帧类型

| TYPE | 方向 | 说明 |
|---|---|---|
| `CMD` | PC -> MCU | 上位机命令 |
| `ACK` | MCU -> PC | 命令成功确认 |
| `NACK` | MCU -> PC | 命令失败 |
| `STATE` | MCU -> PC | 当前状态快照 |
| `DATA` | MCU -> PC | 过程数据记录帧 |
| `EVENT` | MCU -> PC | 事件通知 |
| `HELLO` | 双向 | 握手 |
| `HEARTBEAT` | 双向 | 心跳 |

## 6. 通用字段

| 字段 | 类型 | 说明 |
|---|---|---|
| `seq` | uint32 | 命令序号，由 PC 递增生成 |
| `t` | uint32 | MCU tick，单位 ms |
| `cell` | uint8 | Cell 编号，`0` 或 `1` |
| `ok` | uint8 | `1` 成功，`0` 失败 |
| `err` | int | 错误码 |
| `msg` | string | 简短错误文本，可选 |

## 7. 握手与同步

### 7.1 PC 发起握手

```text
TYPE=HELLO(0x07), SEQ=1, PAYLOAD="role=PC,proto=1,app=BiseminQt"
```

MCU 回复：

```text
TYPE=HELLO(0x07), SEQ=1, PAYLOAD="role=MCU,proto=1,fw=1.0.0,cells=2"
```

握手后，PC 应立即请求完整状态：

```text
TYPE=CMD(0x01), SEQ=2, PAYLOAD="op=GET_STATE"
```

MCU 回复 `ACK`，并发送两个 Cell 的 `STATE`。

## 8. 命令列表

### 8.1 查询状态

```text
TYPE=CMD(0x01), SEQ=10, PAYLOAD="op=GET_STATE"
```

MCU：

```text
TYPE=ACK(0x02),   SEQ=10, PAYLOAD="ok=1"
TYPE=STATE(0x04), SEQ=0,  PAYLOAD="t=1000,cell=0,..."
TYPE=STATE(0x04), SEQ=0,  PAYLOAD="t=1000,cell=1,..."
```

### 8.2 设置普通模式目标温度

```text
TYPE=CMD(0x01), SEQ=11, PAYLOAD="op=SET_TARGET,cell=0,temp=37.5"
```

说明：

- 只修改目标温度，不自动启动。
- 温度范围由 MCU 校验。

### 8.3 启动普通模式

```text
TYPE=CMD(0x01), SEQ=12, PAYLOAD="op=START_NORMAL,cell=0,temp=37.5"
```

说明：

- `temp` 可选。
- 若带 `temp`，MCU 先设置目标温度，再启动普通模式。
- MCU 设置 `mode=NORMAL`，`owner=PC`。

### 8.4 停止 Cell

```text
TYPE=CMD(0x01), SEQ=13, PAYLOAD="op=STOP,cell=0"
```

说明：

- 停止指定 Cell。
- 输出归零。
- PID 状态复位。

### 8.5 急停全部 Cell

```text
TYPE=CMD(0x01), SEQ=14, PAYLOAD="op=STOP_ALL"
```

说明：

- 两个 Cell 全部停止。
- 共享 DRV CH5 停止。
- Qt 和面板都应显示停止状态。

### 8.6 设置程序升温参数

```text
TYPE=CMD(0x01), SEQ=20,
PAYLOAD="op=SET_PROGRAM,cell=0,start=25.0,hold=60,rate=2.0,next=65.0,wait=30,repeat=3"
```

字段说明：

| 字段 | 单位 | 说明 |
|---|---|---|
| `start` | degC | 起始温度 |
| `hold` | s | 起始温度保持时间 |
| `rate` | degC/min | 升降温速率 |
| `next` | degC | 下一目标温度 |
| `wait` | s | 到达每个目标点后的等待时间 |
| `repeat` | 次 | 重复次数 |

### 8.7 读取程序升温参数

```text
TYPE=CMD(0x01), SEQ=21, PAYLOAD="op=GET_PROGRAM,cell=0"
```

MCU 回复：

```text
TYPE=ACK(0x02), SEQ=21, PAYLOAD="ok=1"
TYPE=EVENT(0x06), SEQ=0,
PAYLOAD="t=1000,type=PROGRAM_PARAM,cell=0,start=25.0,hold=60,rate=2.0,next=65.0,wait=30,repeat=3"
```

### 8.8 启动程序升温

```text
TYPE=CMD(0x01), SEQ=22, PAYLOAD="op=START_PROGRAM,cell=0"
```

说明：

- 使用 MCU 当前保存的程序参数。
- MCU 设置 `mode=PROGRAM`，`owner=PC`。

### 8.9 记录控制

PC 侧负责记录，一般不要求 MCU 存储历史数据。

开始记录：

```text
TYPE=CMD(0x01), SEQ=30, PAYLOAD="op=LOG_START,cell=0,period=1000"
```

停止记录：

```text
TYPE=CMD(0x01), SEQ=31, PAYLOAD="op=LOG_STOP,cell=0"
```

说明：

- `period` 单位 ms，建议 `1000`。
- MCU 可根据该命令调整 `DATA` 上报周期。
- 若 MCU 固定每 1s 上报，则该命令可以只作为 PC 内部状态，不一定下发。

## 9. ACK 与 NACK

### 9.1 ACK

```text
TYPE=ACK(0x02), SEQ=12, PAYLOAD="ok=1"
```

### 9.2 NACK

```text
TYPE=NACK(0x03), SEQ=12, PAYLOAD="ok=0,err=1002,msg=BAD_PARAM"
```

常用错误码：

| err | 名称 | 说明 |
|---|---|---|
| `0` | OK | 成功 |
| `1001` | BAD_CRC | CRC 错误 |
| `1002` | BAD_PARAM | 参数错误 |
| `1003` | UNKNOWN_OP | 未知命令 |
| `1004` | BUSY | 当前忙 |
| `1005` | DENIED | 当前控制权不允许 |
| `1006` | HW_FAULT | 硬件故障 |
| `1007` | TEMP_FAULT | 温度传感器故障 |
| `1008` | MODE_CONFLICT | 模式冲突 |

## 10. 状态上报

MCU 应在以下情况发送 `STATE`：

- PC 请求 `GET_STATE`。
- 面板修改目标温度。
- 面板启动/停止。
- PC 命令执行成功。
- 模式切换。
- 故障发生或清除。
- 程序升温阶段变化。
- 周期性上报，建议 1s 一次。

示例：

```text
TYPE=STATE(0x04), SEQ=0,
PAYLOAD="t=123456,cell=0,mode=NORMAL,owner=PC,running=1,target=37.5,command=36.2,current=35.8,t0=35.7,t1=35.9,duty=0.083,error=0,phase=0"
```

字段说明：

| 字段 | 说明 |
|---|---|
| `mode` | `STOP` / `NORMAL` / `PROGRAM` |
| `owner` | `NONE` / `PANEL` / `PC` |
| `running` | `0` 停止，`1` 运行 |
| `target` | 用户目标温度 |
| `command` | 斜坡处理后送入 PID 的目标温度 |
| `current` | Cell 平均温度 |
| `t0` | Cell 内第 1 路温度 |
| `t1` | Cell 内第 2 路温度 |
| `duty` | Cell 平均 duty |
| `error` | 面板/控制错误码 |
| `phase` | 程序升温阶段，普通模式为 `0` |

## 11. 数据记录帧

`DATA` 用于 Qt 记录过程曲线和导出 Excel/CSV。

建议 MCU 每 1s 发送一次：

```text
TYPE=DATA(0x05), SEQ=0,
PAYLOAD="t=123456,cell=0,mode=PROGRAM,phase=3,target=65.0,command=48.2,current=48.0,t0=47.9,t1=48.1,duty=0.126,error=0"
```

Qt 保存字段：

| 字段 | Excel 列名 |
|---|---|
| `t` | MCU时间ms |
| PC本地时间 | 电脑时间 |
| `cell` | Cell |
| `mode` | 模式 |
| `phase` | 程序阶段 |
| `target` | 用户目标温度 |
| `command` | PID目标温度 |
| `current` | 平均温度 |
| `t0` | 通道0/2温度 |
| `t1` | 通道1/3温度 |
| `duty` | 输出占空比 |
| `error` | 错误码 |

Qt 导出建议：

- 第一阶段先导出 `.csv`，简单可靠。
- 后续可使用 `QXlsx` 导出 `.xlsx`。
- 每次运行生成一个 session，文件名建议：

```text
YYYYMMDD_HHMMSS_cell0_program.csv
```

## 12. 事件帧

事件用于非周期状态变化。

示例：

```text
TYPE=EVENT(0x06), SEQ=0, PAYLOAD="t=123000,type=START,cell=0,mode=NORMAL,owner=PANEL"
TYPE=EVENT(0x06), SEQ=0, PAYLOAD="t=124000,type=STOP,cell=0,owner=PC"
TYPE=EVENT(0x06), SEQ=0, PAYLOAD="t=125000,type=ERROR,cell=0,error=121"
TYPE=EVENT(0x06), SEQ=0, PAYLOAD="t=126000,type=PROGRAM_PHASE,cell=0,phase=3"
```

事件类型：

| type | 说明 |
|---|---|
| `START` | Cell 启动 |
| `STOP` | Cell 停止 |
| `ERROR` | 故障 |
| `ERROR_CLEAR` | 故障清除 |
| `TARGET_CHANGED` | 目标温度变化 |
| `PROGRAM_PARAM` | 程序参数 |
| `PROGRAM_PHASE` | 程序阶段变化 |
| `PROGRAM_DONE` | 程序结束 |

## 13. 心跳与断线处理

PC 每 1s 发送：

```text
TYPE=HEARTBEAT(0x08), SEQ=100, PAYLOAD="t=5000"
```

MCU 回复：

```text
TYPE=HEARTBEAT(0x08), SEQ=100, PAYLOAD="t=5050"
```

建议策略：

- PC 连续 3s 未收到 MCU 任何帧：显示通信断开。
- MCU 连续 5s 未收到 PC 心跳：仅清除 `owner=PC` 的在线标志，不建议自动停止控温。
- 如果产品安全策略要求 PC 掉线必须停止，可增加配置项 `pc_loss_action=STOP`。

## 14. 控制权策略

建议默认策略：

- 面板和 PC 都可以停止。
- 急停永远优先。
- PC 控制时，面板仍可停止。
- 面板控制时，PC 也可停止。
- 启动或修改参数时，如果另一个来源正在运行，MCU 可返回 `MODE_CONFLICT`，也可以允许接管。

推荐初版使用“最后操作接管”：

```text
谁成功发送启动/设置命令，owner 就变成谁。
```

更严格版本可以加入锁：

```text
TYPE=CMD(0x01), SEQ=40, PAYLOAD="op=LOCK_OWNER,owner=PC"
TYPE=CMD(0x01), SEQ=41, PAYLOAD="op=UNLOCK_OWNER"
```

## 15. 程序升温阶段定义

建议与 MCU 内部阶段保持一致：

| phase | 名称 | 说明 |
|---|---|---|
| `0` | NONE | 非程序模式 |
| `1` | TO_START | 斜坡到起始温度 |
| `2` | START_HOLD | 起始温度保持 |
| `3` | RAMP_NEXT | 斜坡到下一目标 |
| `4` | WAIT_NEXT | 到达后等待 |
| `5` | FINISHED | 程序结束 |

## 16. 面板与 Qt 一致性规则

Qt 侧必须遵守：

- 不根据按钮点击直接认为状态已改变。
- 点击后等待 `ACK` 或 `STATE`。
- 收到 `STATE` 后，以 `STATE` 覆盖 Qt 本地界面状态。
- 收到面板触发的 `STATE/EVENT` 后，Qt 也必须同步更新。

MCU 侧必须遵守：

- 任意来源修改状态后，都发送 `STATE`。
- 任意来源启动/停止后，都发送 `EVENT` 和 `STATE`。
- 周期性发送 `STATE` 或 `DATA`，防止 Qt 状态长期漂移。

## 17. 推荐实现顺序

1. 实现 MCU `STATE` 周期上报。
2. Qt 实现串口连接、帧解析、CRC 校验、实时显示。
3. 实现 `GET_STATE`、`START_NORMAL`、`SET_TARGET`、`STOP`。
4. 实现程序参数 `SET_PROGRAM`、`GET_PROGRAM`、`START_PROGRAM`。
5. 实现 `DATA` 记录和 CSV 导出。
6. 实现 Excel 导出。
7. 实现控制权锁定和断线策略。

## 18. 示例时序

### 18.1 Qt 启动普通模式

```text
PC  -> MCU: TYPE=CMD,   SEQ=1, PAYLOAD="op=START_NORMAL,cell=0,temp=37.5"
MCU -> PC : TYPE=ACK,   SEQ=1, PAYLOAD="ok=1"
MCU -> PC : TYPE=EVENT, SEQ=0, PAYLOAD="t=1000,type=START,cell=0,mode=NORMAL,owner=PC"
MCU -> PC : TYPE=STATE, SEQ=0, PAYLOAD="t=1000,cell=0,mode=NORMAL,owner=PC,running=1,target=37.5,..."
MCU -> PC : TYPE=DATA,  SEQ=0, PAYLOAD="t=2000,cell=0,..."
MCU -> PC : TYPE=DATA,  SEQ=0, PAYLOAD="t=3000,cell=0,..."
```

### 18.2 面板修改目标温度

```text
Panel changes target to 40.0
MCU -> PC : TYPE=EVENT, SEQ=0, PAYLOAD="t=5000,type=TARGET_CHANGED,cell=0,owner=PANEL,target=40.0"
MCU -> PC : TYPE=STATE, SEQ=0, PAYLOAD="t=5000,cell=0,mode=NORMAL,owner=PANEL,running=1,target=40.0,..."
```

### 18.3 程序升温结束并导出

```text
MCU -> PC : TYPE=EVENT, SEQ=0, PAYLOAD="t=600000,type=PROGRAM_DONE,cell=0"
MCU -> PC : TYPE=STATE, SEQ=0, PAYLOAD="t=600000,cell=0,mode=PROGRAM,phase=5,running=1,..."
PC saves current log session
PC exports CSV/XLSX
```

## 19. 兼容性约定

- `proto=1` 为初版协议。
- 新增字段不得破坏旧字段含义。
- Qt 和 MCU 都必须忽略未知字段。
- 新增命令必须通过 `UNKNOWN_OP` 对旧固件优雅失败。
- 字段名固定使用小写或大写时需统一；本文建议字段名小写，`TYPE` 和 `op` 值大写。
