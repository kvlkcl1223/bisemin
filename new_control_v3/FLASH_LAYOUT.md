# Flash 存储布局

## MCU 信息

| 项目 | �?|
|------|-----|
| 型号 | STM32G474VET6 |
| Flash 可寻址范围 | **256 KB**（链接脚�?`0x00040000`�?|
| Flash 起始地址 | `0x08000000` |
| Flash 结束地址 | `0x0803FFFF` |
| 页大小（逻辑�?| 2 KB�?048 字节�?|
| 总页数（逻辑�?| 128（页�?0 ~ 127�?|

> **注意**：芯片标�?512KB，但链接脚本和实际可寻址范围均为 256KB。`0x08040000` 及以上地址不可访问（擦�?写入均无效）�?

---

## 整体布局

```
0x08000000 ┌─────────────────────�?
           �?                    �?
           �?  固件代码�?         �? ~60 KB
           �?  (链接脚本预留 256KB) �?
           �?                    �?
0x0800F000 ├─────────────────────�? �?固件结束（约�?
           �?                    �?
           �?  未使�?            �? ~188 KB
           �?                    �?
0x0803E000 ├─────────────────────�? �?Flash 存储区起�?
           �? Cell 0 标定数据     �? 0x0803E000 ~ 0x0803EFFF (4KB)
0x0803F000 ├─────────────────────�?
           �? Cell 1 标定数据     �? 0x0803F000 ~ 0x0803FFFF (4KB)
0x08040000 └─────────────────────�? �?256KB Flash 结束
```

---

## 标定数据存储

### 存储区配�?(`flash_storage.h`)

| �?| �?|
|------|-----|
| `FLASH_STORAGE_START_ADDR` | `0x0803E000` |
| `FLASH_STORAGE_SIZE` | `8192`�? KB�?|
| `FLASH_STORAGE_PAGE_SIZE` | `2048`�? KB，逻辑页） |
| `FLASH_STORAGE_PAGE_COUNT` | `4` |

### Cell 0

| 项目 | �?|
|------|-----|
| 宏定�?| `CALIB_FLASH_OFFSET_CELL0 = 0U` |
| 存储区偏�?| 0 字节 |
| 绝对地址 | **`0x0803E000`** |
| HAL 擦除页号 | `Page = 124`, `Bank = FLASH_BANK_1` |
| 占用大小 | 588 字节（含对齐�?|

### Cell 1

| 项目 | �?|
|------|-----|
| 宏定�?| `CALIB_FLASH_OFFSET_CELL1 = 4096U` |
| 存储区偏�?| 4096 字节 |
| 绝对地址 | **`0x0803F000`** |
| HAL 擦除页号 | `Page = 126`, `Bank = FLASH_BANK_1` |
| 占用大小 | 588 字节（含对齐�?|

### 擦除页号计算 (`flash_storage.c`)

```c
page_num = 124U + (absolute_addr - FLASH_STORAGE_START_ADDR) / FLASH_STORAGE_PAGE_SIZE;
banks    = FLASH_BANK_1;
```

| Cell | 地址 | 公式 | Page |
|:---:|------|------|:---:|
| 0 | `0x0803E000` | 124 + 0/2048 | **124** |
| 1 | `0x0803F000` | 124 + 4096/2048 | **126** |

> **注意**：此芯片�?FLASH_SIZE 寄存器返回值异常，不能�?HAL �?`FLASH_BANK_SIZE`。擦除页号使用上述硬编码映射，已验证可正常工作�?

---

## 标定数据结构�?`CalibFlashData_t`

### C 定义 (`calib_mode.h`)

```c
typedef struct
{
    uint32_t     magic;                        // 4 字节，魔�?0x42495346 ("BISF")
    uint8_t      cell;                         // 1 字节，标定的 Cell 编号 (0 �?1)
    uint8_t      reserved[3];                  // 3 字节，对齐填�?
    CalibStep_t  step[CALIB_DUTY_COUNT];      // 36 步标定记�?(36 x 16 = 576 字节)
    uint16_t     crc16;                        // 2 字节，CRC16-CCITT 校验
    // 编译器对齐填充：2 字节
} CalibFlashData_t;
// 总大�? 588 字节（含对齐�?
```

### 内存布局�?

| 偏移 | 大小 | 字段 | 类型 | 说明 |
|------|------|------|------|------|
| 0 | 4 | `magic` | `uint32_t` | 魔数 `0x42495346` ("BISF") |
| 4 | 1 | `cell` | `uint8_t` | Cell 编号�? �?1�?|
| 5 | 3 | `reserved` | `uint8_t[3]` | 对齐填充（写 0�?|
| 8 | 576 | `step[36]` | `CalibStep_t[36]` | 36 步标定记�?|
| 584 | 2 | `crc16` | `uint16_t` | CRC16-CCITT 校验�?|
| 586 | 2 | (padding) | �?| 编译器对齐填�?|

---

## 单步记录 `CalibStep_t`

### C 定义

```c
typedef struct
{
    float   duty;          // 4 字节，占空比（正=制冷, �?加热�?
    float   temp_ch0;      // 4 字节，CH0 / CH2 稳态平均温�?(°C)
    float   temp_ch1;      // 4 字节，CH1 / CH3 稳态平均温�?(°C)
    uint8_t valid;         // 1 字节，是否有�?(1=稳定达标, 0=超时)
    uint8_t settled;       // 1 字节，是否达到稳�?
    uint16_t reserved;     // 2 字节，对齐填�?
} CalibStep_t;
// 总大�? 16 字节
```

### 内存布局�?

| 偏移 | 大小 | 字段 | 类型 | 说明 |
|------|------|------|------|------|
| 0 | 4 | `duty` | `float` | 占空比，范围 `+0.30` ~ `-0.40` |
| 4 | 4 | `temp_ch0` | `float` | CH0 稳态温�?(°C) |
| 8 | 4 | `temp_ch1` | `float` | CH1 稳态温�?(°C) |
| 12 | 1 | `valid` | `uint8_t` | `1`=稳定达标, `0`=超时 |
| 13 | 1 | `settled` | `uint8_t` | `1`=达到稳�? `0`=未达�?|
| 14 | 2 | `reserved` | `uint16_t` | 对齐填充 |

---

## 36 步占空比序列

| Step | 占空�?| 效果 |
|:---:|:---:|:---:|
| 0 | **+0.30** | ❄️ 制冷（最大值） |
| 1 | +0.28 | ❄️ 制冷 |
| ... | ... | ... |
| 14 | +0.02 | ❄️ 微冷 |
| 15 | 0.00 | 🔥 微热 |
| ... | ... | ... |
| 34 | -0.38 | 🔥 加热 |
| 35 | **-0.40** | 🔥 加热（最大值） |

步进：`-0.02`（从制冷 �?加热�?

---

## CRC16 校验

| 项目 | �?|
|------|-----|
| 算法 | CRC16-CCITT（查表法�?|
| 初始�?| `0xFFFF` |
| 多项�?| `0x1021` |
| 校验范围 | `magic` �?`step[35]`（不�?`crc16` 字段本身�?|
| 代码位置 | `My/calib_mode.c` �?`CalibMode_CRC16()` |

---

## 快速测试模�?

�?`calib_mode.h` 中取消注�?`#define CALIB_FAST_TEST` 可启用：

| 项目 | 正常模式 | 快速测�?|
|------|:---:|:---:|
| 标定步数 | 36 | 1 |
| 占空�?| +0.30 �?-0.40 | 固定 0.0 |
| 稳定阈�?| 0.1°C | 999°C（立稳） |
| 总耗时 | ~1~2 小时 | < 10 �?|

---

## 相关文件

| 文件 | 内容 |
|------|------|
| `My/calib_mode.h` | 标定数据结构体、Flash 偏移宏、占空比参数、快速测试开�?|
| `My/calib_mode.c` | Flash 读写/擦除、CRC16 实现 |
| `My/flash_storage.h` | Flash 存储区基地址、页大小等宏定义 |
| `My/flash_storage.c` | Flash 擦除/写入/读取底层实现 |
| `MDK-ARM/stm32g474xx_flash.sct` | 链接脚本（固件占 256KB�?|

---

## 版本信息

- 固件版本：V1.0.0
- 最后更新：2026-07-11
- Calibration mode: 36 steps, +0.30 to -0.40, step -0.02
- Flash 存储区地址：`0x0803E000`�?56KB 末尾 8KB�?

---

## Current calibration format note

Current firmware uses an asymmetric calibration range:

```text
CALIB_DUTY_START = +0.30
CALIB_DUTY_END   = -0.40
CALIB_DUTY_STEP  = -0.02
CALIB_DUTY_COUNT = 36
CALIB_FLASH_MAGIC = 0x42495346 ("BISF")
```

`CalibFlashData_t` contains 36 `CalibStep_t` records. Each `CalibStep_t` is 16 bytes, so the step table is 576 bytes and the full structure is 588 bytes with compiler padding.

The PC protocol reads calibration results as one metadata frame plus one frame per step, because a full table does not fit in the 256-byte ASCII payload limit.