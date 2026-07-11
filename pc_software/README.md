# Bisemin PC Control

Python + Qt 上位机工程，用于通过串口控制 MCU 温控系统。

## 功能目标

- 串口连接 MCU。
- 控制 Cell 0 / Cell 1 的普通控温模式。
- 设置并启动程序升温模式。
- 接收 MCU 状态帧与过程数据帧。
- 记录温度-时间数据。
- 导出 CSV，后续可扩展 XLSX。

## 环境

```bash
pip install -r requirements.txt
python main.py
```

## 目录

```text
pc_software/
  main.py
  requirements.txt
  app/
    main_window.py
    models.py
    logger.py
    protocol.py
    serial_worker.py
    widgets/
      cell_panel.py
      log_panel.py
```
