# 标定逐点保存与断点续标

每个标定点结束后，MCU 将占空比、两路温度、`valid`、`settled` 和点索引追加到 Flash；读回校验成功后才进入下一点。启动入口不变，上电自动标定、面板和 PC 的 `START_CALIB` 都使用此逻辑。

## 恢复规则

- 本池有未完成且参数匹配的记录：保留已保存点，从下一点继续。中断时尚未保存的点重新测量，PWM 仍从零按原速率变化，重新等待温度稳定。
- 本池已完成全部 38 点：下次启动创建新一轮，从第 1 点开始。
- 无有效记录，或扫描范围、配比、DRV/测温映射、共享设置、稳定判据发生变化：创建新一轮。旧固件未保存到 Flash 的半程数据无法恢复。
- 超时点保持原行为：记录 `valid=0, settled=0`，也计入已执行点；恢复时不自动补测这些点。
- 手动停止、DRV 故障不会删除已保存进度。两个温控池分别恢复各自进度。
- 新一轮开始时会擦除本池上一轮结果。未完成表不会作为完整前馈表加载；`GET_CALIB_RESULT` 返回 `CALIB_NOT_FOUND,detail=5`，完整后原有 PC 读取方式不变。

## 存储布局

保持每池原有 4 KB 分区和擦除入口，只使用分区前 2 KB：Cell 0 基址 `0x0803E000`，Cell 1 基址 `0x0803F000`。未扩大链接预留区。

新的物理存储格式为 `BISH`（`0x42495348`）：72 字节头部，加最多 61 个 32 字节点记录。头部保存标定参数与 CRC；点记录保存结果、点索引与 CRC。最后一个 8 字节双字包含提交标记及其反码，写完并读回一致才确认成功。

每轮仅开始时擦除一次；点记录使用 `FlashStorage_ProgramErased()`，该接口只写已擦除且 8 字节对齐的区域，绝不会自动擦页。写到一半的非空记录跳过，下一次追加使用后面的空槽，不覆盖已有记录。38 个正常点之外有 23 个备用槽；若反复写入中断耗尽槽位，报错停止并保留现场，不自动擦掉进度。

`CalibMode_LoadFromFlash()` 将完整日志重建为原来的 620 字节 `CalibFlashData_t`，因此 PC 帧格式保持兼容。旧 `BISG` 完整表仍可读取，但没有断点索引；旧 `BISF` 表仍不支持。调试函数 `CalibMode_FlashTest()` 会覆盖所选池数据，不应在需要保留进度时调用。

## 串口与 Watch

```text
CALIB,NEW,CELL:0,NEXT_STEP:1
CALIB,SAVED,CELL:0,STEP:12,TOTAL:38,VALID:1
CALIB,RESUME,CELL:0,SAVED:12,NEXT_STEP:13
CALIB,DONE,CELL:0,STEPS:38
```

`SAVED` / `NEXT_STEP` 使用从 1 开始的点号；已有 `RUN` / `STEP_START` 的 `STEP` 仍是从 0 开始的索引。`SAVED` 表示写入和读回校验成功，普通结果行只表示测量结束。完成仍输出 `DONE`，不再在最后另写整张表。

| Watch 变量 | 含义 |
|---|---|
| `g_calib_saved_count` | 本轮已持久保存点数，0～38，含超时点 |
| `g_calib_resumed` | 本次启动是否恢复已有未完成记录 |
| `g_calib_step_idx` | 当前点索引，0～37，全部完成后为 38 |
| `g_calib_flash_status` | 最近保存操作的 Flash 状态，0 成功 |

`ERROR:10` 仍为 DRV 故障。新增 `ERROR:11` 表示 Flash 擦除、读取、写入或校验失败；`ERROR:12` 表示记录槽耗尽。两者都会立即停止本池及共享输出，保存失败的点不会推进。

## 故障详细日志

标定中止会在原有 `CALIB,FAULT` 后输出一次 `CALIB,FAULT_DETAIL` 和 `FAULT,BEGIN...FAULT,END` 诊断块。正常控温因错误紧急停止时也会输出诊断块，`MODE:APP`；标定对应 `MODE:CALIB`。先执行原有停止逻辑，再打印文本，不改变故障保护和重试策略。

诊断包括本池外层、内层、启用的共享 DRV，以及其他故障标志置位的通道：

- `DRV:1..5` 为物理通道；`CELL:0/1` 为池1/2；`LAST_DRV_INDEX:0..4` 是原始零基索引，255 表示未记录。
- `FLAG/READY/AWAKE/VERIFY_OK/MISMATCH` 为故障、初始化、唤醒和配置校验状态。
- `PIN_LAST` 是最后一次引脚故障快照，清除芯片故障位前已捕获；`FAULT_CACHE` 是现有故障缓存；`CONFIG_LAST` 是初始化配置校验读值。它们可能来自不同时间，不能将配置快照或历史故障当作本次新增故障。没有记录时显示 `SOURCE:NONE,REGS:UNAVAILABLE`。
- 每组快照打印 `REG0`～`REG5`、`READ_OK` 和逐寄存器 `REG_STATUS`。`READ_OK` 的 bit0～bit5 表示对应 SPI 读取调用成功，`0x3F` 不保证线路返回数据可信。
- `ALL_FF:1` 表示六个保存值全为 `0xFF`；`REJECTED:1` 表示该快照被拒绝。原 Watch 变量名包含 `all_ff`，但也可能因配置不匹配等原因置位，日志将这两种含义分开。
- 故障寄存器读取失败、值为 `0xFF` 或快照被拒绝时显示 `DECODE:UNAVAILABLE`；其余故障快照打印 `BITS0/BITS1` 中各位的 0/1。配置快照不作为故障位证据解码。
- `PIN_COUNT/CAPTURE_COUNT` 是本次上电以来的相应捕获次数。`EXPECTED` 是默认配置寄存器预期值。配置校验过程中的 REG2 可能处于解锁状态，不能仅凭它与锁定后的预期不同认定写入错误。

DRV 状态码：`0=OK`、`1=PARAM`、`2=SPI`、`3=HAL`、`4=FAULT_PIN`、`5=WDFLT_PIN`、`6=FAULT_REG`。

`CALIB,FAULT_DETAIL` 另含当前零基 `INDEX`、已保存 `SAVED`、恢复标志 `RESUMED`、`FLASH_STATUS` 和日志槽位置。Flash 状态码：`0=OK`、`1=PARAM`、`2=UNLOCK`、`3=ERASE`、`4=WRITE`、`5=BUSY`、`6=LOCK`。`ERROR:11` 时结合此字段判断，`ERROR:12` 时查看槽位是否耗尽。

日志读取现有 RAM 快照，不重新访问 SPI，也不会为了打印而再次清除故障寄存器。

## 验证边界

主机测试模拟 Flash 和硬件接口，验证追加、断电式截断、恢复、参数变化、两池隔离和完整读取。真实芯片的 Flash 擦除映射、掉电/ECC 行为和运行中写入仍需板上验证；本次沿用工程原有硬件页映射。
